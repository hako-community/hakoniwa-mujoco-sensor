// SAMPLE (not a shipped artifact). An IRadar3DController that produces
// radar_points with the mujoco-sensor Radar MODEL via the C-ABI, ray casting the
// live Godot scene (GodotRayCaster). Drop-in alternative to
// Default3DRadarController: same interface and same radar_points PDU layout
// (x,y,z + Doppler velocity in the intensity slot), so RadarPointCloudVisualizer
// renders it via the PDU read path. No core-pro/PDU/MuJoCo process is needed to
// sense. See README.md.

using System;
using System.Globalization;
using hakoniwa.objects.core.sensors;
using hakoniwa.pdu.interfaces;
using hakoniwa.pdu.msgs.sensor_msgs;
using hakoniwa.pdu.godot;
using Godot;

namespace hakoniwa.mujoco.sensor.sample
{
    public partial class MujocoRadar3DController : Node3D, IRadar3DController
    {
        [Export] public bool Enabled = true;
        [Export] public float Range = 20f;
        [Export] public float HorizontalFOV = 60f;
        [Export] public float VerticalFOV = 20f;
        [Export] public int PointsPerSecond = 1500;
        [Export] public int UpdateRateHz = 10;
        [Export] public int NoiseSeed = 1;
        [Export] public bool DrawDebugPoints = false;

        [Export] public bool ExternalSensing { get; set; } = false;

        private const int PointStep = 16;
        private const int MaxDataArraySize = 176656;

        private string robotName;
        private IntPtr handle = IntPtr.Zero;
        private GodotRayCaster caster;
        private float[] scanBuf;
        private byte[] data;
        private PointField[] pointFields;
        private int updateCycle = 1;
        private int count = 0;

        // exposed for parity with Default3DRadarController (optional in-process viz)
        public int LastDetections { get; private set; } = 0;
        public byte[] LastScanData => data;

        private Vector3 prevPos;
        private bool havePrev = false;

        public bool SetParams(Radar3DParams p)
        {
            Enabled = p.Enabled; Range = p.Range; HorizontalFOV = p.HorizontalFOV;
            VerticalFOV = p.VerticalFOV; PointsPerSecond = p.PointsPerSecond;
            UpdateRateHz = Math.Max(1, p.UpdateRateHz); NoiseSeed = p.NoiseSeed;
            DrawDebugPoints = p.DrawDebugPoints;
            return true;
        }

        public Radar3DParams GetParams() => new Radar3DParams
        {
            Enabled = Enabled, Range = Range, HorizontalFOV = HorizontalFOV, VerticalFOV = VerticalFOV,
            PointsPerSecond = PointsPerSecond, UpdateRateHz = UpdateRateHz, NoiseSeed = NoiseSeed,
            DrawDebugPoints = DrawDebugPoints
        };

        private string BuildConfigJson()
        {
            var c = CultureInfo.InvariantCulture;
            return "{"
                + $"\"range\":{Range.ToString(c)},"
                + $"\"horizontal_fov_deg\":{HorizontalFOV.ToString(c)},"
                + $"\"vertical_fov_deg\":{VerticalFOV.ToString(c)},"
                + $"\"points_per_second\":{PointsPerSecond},"
                + $"\"noise_seed\":{NoiseSeed}"
                + "}";
        }

        public void DoInitialize(string robot_name, IPduManager pduManager)
        {
            GD.Print("Initialize MujocoRadar3DController for " + robot_name);
            robotName = robot_name;
            if (ExternalSensing)
            {
                GD.Print("MujocoRadar3DController: ExternalSensing=true -> skip (another producer fills radar_points).");
                return;
            }

            caster = new GodotRayCaster(this);
            handle = SensorNative.hako_sensor_create("radar", BuildConfigJson(), caster.Fn, IntPtr.Zero);
            if (handle == IntPtr.Zero) throw new Exception("hako_sensor_create(radar) failed");

            scanBuf = new float[(MaxDataArraySize / PointStep) * 4];
            data = new byte[MaxDataArraySize];

            float pdt = (float)GetPhysicsProcessDeltaTime();
            if (pdt <= 0f) pdt = 1f / 60f;
            updateCycle = Mathf.Max(1, Mathf.RoundToInt((1.0f / Math.Max(1, UpdateRateHz)) / pdt));

            INamedPdu pdu = pduManager.CreateNamedPdu(robotName, Default3DRadarController.pdu_name_radar_points);
            if (pdu == null) throw new ArgumentException($"ERROR: can not find pdu({robotName}/radar_points)");
            var pc = new PointCloud2(pdu);
            pointFields = MakeFields(pduManager);
            pc.fields = pointFields;
            pduManager.WriteNamedPdu(pdu);
            pduManager.FlushNamedPdu(pdu);
        }

        public void DoControl(IPduManager pduManager)
        {
            if (!Enabled || ExternalSensing || handle == IntPtr.Zero) return;
            count++;
            if (count < updateCycle) return;
            float pdt = (float)GetPhysicsProcessDeltaTime();
            if (pdt <= 0f) pdt = 1f / 60f;
            float dt = count * pdt;
            count = 0;

            Vector3 pos = GlobalPosition;
            Vector3 vel = (havePrev && dt > 1e-6f) ? (pos - prevPos) / dt : Vector3.Zero;
            prevPos = pos; havePrev = true;

            double[] state = SampleState.FromNode(this, vel);
            int rc = SensorNative.hako_sensor_scan(handle, state, dt, scanBuf, scanBuf.Length / 4,
                                                   out int _, out int _, out int pts);
            if (rc != 0)
            {
                GD.PrintErr("hako_sensor_scan(radar) rc=" + rc + " err=" + SensorNative.LastError(handle));
                return;
            }

            int bytes = Math.Min(pts * PointStep, data.Length);
            Buffer.BlockCopy(scanBuf, 0, data, 0, bytes);
            LastDetections = Math.Min(pts, data.Length / PointStep);

            INamedPdu pdu = pduManager.CreateNamedPdu(robotName, Default3DRadarController.pdu_name_radar_points);
            if (pdu == null) return;
            var pc = new PointCloud2(pdu);
            TimeStamp.Set(pc.header);
            pc.header.frame_id = "front_radar_frame";
            pc.height = 1;
            pc.width = (uint)LastDetections;
            pc.is_bigendian = false;
            pc.fields = pointFields;
            pc.point_step = PointStep;
            pc.row_step = (uint)(PointStep * LastDetections);
            pc.data = data;
            pc.is_dense = true;
            pduManager.WriteNamedPdu(pdu);
            pduManager.FlushNamedPdu(pdu);
        }

        private static PointField[] MakeFields(IPduManager pduManager)
        {
            string[] names = { "x", "y", "z", "intensity" };  // intensity = Doppler velocity
            var fields = new PointField[4];
            for (int i = 0; i < 4; i++)
            {
                var f = new PointField(pduManager.CreatePduByType("fields", "sensor_msgs", "PointField"));
                f.name = names[i];
                f.offset = (uint)(i * 4);
                f.datatype = 7; // FLOAT32
                f.count = 1;
                fields[i] = f;
            }
            return fields;
        }

        public override void _ExitTree()
        {
            if (handle != IntPtr.Zero) { SensorNative.hako_sensor_destroy(handle); handle = IntPtr.Zero; }
        }
    }
}
