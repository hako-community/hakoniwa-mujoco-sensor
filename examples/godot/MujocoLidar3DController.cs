// SAMPLE (not a shipped artifact). An ILiDAR3DController that produces the
// lidar_points cloud with the mujoco-sensor 3D LiDAR MODEL via the C-ABI, ray
// casting against the live Godot scene (GodotRayCaster). It is a drop-in
// alternative to Default3DLiDARController: same interface, same lidar_points PDU
// layout, so LiDARPointCloudVisualizer renders it unchanged. The difference is
// the scan geometry/scheduler come from hakoniwa-mujoco-sensor rather than
// Godot's hand-rolled ScanEnvironment -- and NO core-pro/PDU/MuJoCo process is
// needed to sense (the cast is in-process Godot).
//
// See README.md. To use: replace the Default3DLiDARController node in the drone
// scene with this one (keep ExternalSensing=false: THIS node is the producer).

using System;
using System.Globalization;
using hakoniwa.objects.core.sensors;
using hakoniwa.pdu.interfaces;
using hakoniwa.pdu.msgs.sensor_msgs;
using hakoniwa.pdu.godot;
using Godot;

namespace hakoniwa.mujoco.sensor.sample
{
    public partial class MujocoLidar3DController : Node3D, ILiDAR3DController
    {
        [Export] public bool Enabled = true;
        [Export] public int NumberOfChannels = 15;
        [Export] public int RotationsPerSecond = 10;
        [Export] public int PointsPerSecond = 54150;
        [Export] public float MaxDistance = 20f;
        [Export] public float VerticalFOVUpper = 15f;
        [Export] public float VerticalFOVLower = -15f;
        [Export] public float HorizontalFOVStart = -180f;
        [Export] public float HorizontalFOVEnd = 180f;
        [Export] public bool DrawDebugPoints = false;

        // If true, some OTHER producer (mujoco-sensor Pattern A) fills lidar_points
        // and this node does nothing -- same semantics as Default3DLiDARController.
        [Export] public bool ExternalSensing { get; set; } = false;

        private const int PointStep = 16;
        private const int MaxDataArraySize = 176656;

        private string robotName;
        private IntPtr handle = IntPtr.Zero;
        private GodotRayCaster caster;      // holds the delegate alive
        private float[] scanBuf;
        private byte[] data;
        private PointField[] pointFields;
        private int updateCycle = 1;
        private int count = 0;

        public bool SetParams(LiDAR3DParams p)
        {
            Enabled = p.Enabled;
            NumberOfChannels = p.NumberOfChannels;
            RotationsPerSecond = p.RotationsPerSecond;
            PointsPerSecond = p.PointsPerSecond;
            MaxDistance = p.MaxDistance;
            VerticalFOVUpper = p.VerticalFOVUpper;
            VerticalFOVLower = p.VerticalFOVLower;
            HorizontalFOVStart = p.HorizontalFOVStart;
            HorizontalFOVEnd = p.HorizontalFOVEnd;
            DrawDebugPoints = p.DrawDebugPoints;
            return true;
        }

        public LiDAR3DParams GetParams() => new LiDAR3DParams
        {
            Enabled = Enabled, NumberOfChannels = NumberOfChannels,
            RotationsPerSecond = RotationsPerSecond, PointsPerSecond = PointsPerSecond,
            MaxDistance = MaxDistance, VerticalFOVUpper = VerticalFOVUpper,
            VerticalFOVLower = VerticalFOVLower, HorizontalFOVStart = HorizontalFOVStart,
            HorizontalFOVEnd = HorizontalFOVEnd, DrawDebugPoints = DrawDebugPoints
        };

        private string BuildConfigJson()
        {
            var c = CultureInfo.InvariantCulture;
            return "{"
                + $"\"channels\":{NumberOfChannels},"
                + $"\"rotations_per_second\":{RotationsPerSecond},"
                + $"\"points_per_second\":{PointsPerSecond},"
                + $"\"max_distance\":{MaxDistance.ToString(c)},"
                + $"\"min_distance\":0.05,"
                + $"\"vertical_fov_upper_deg\":{VerticalFOVUpper.ToString(c)},"
                + $"\"vertical_fov_lower_deg\":{VerticalFOVLower.ToString(c)},"
                + $"\"horizontal_fov_start_deg\":{HorizontalFOVStart.ToString(c)},"
                + $"\"horizontal_fov_end_deg\":{HorizontalFOVEnd.ToString(c)}"
                + "}";
        }

        public void DoInitialize(string robot_name, IPduManager pduManager)
        {
            GD.Print("Initialize MujocoLidar3DController for " + robot_name);
            robotName = robot_name;
            if (ExternalSensing)
            {
                GD.Print("MujocoLidar3DController: ExternalSensing=true -> skip (another producer fills lidar_points).");
                return;
            }

            caster = new GodotRayCaster(this);
            handle = SensorNative.hako_sensor_create("lidar3d", BuildConfigJson(), caster.Fn, IntPtr.Zero);
            if (handle == IntPtr.Zero) throw new Exception("hako_sensor_create(lidar3d) failed");

            scanBuf = new float[(MaxDataArraySize / PointStep) * 4];
            data = new byte[MaxDataArraySize];

            float period = 1.0f / Math.Max(1, RotationsPerSecond);
            float pdt = (float)GetPhysicsProcessDeltaTime();
            if (pdt <= 0f) pdt = 1f / 60f;
            updateCycle = Mathf.Max(1, Mathf.RoundToInt(period / pdt));

            INamedPdu pdu = pduManager.CreateNamedPdu(robotName, Default3DLiDARController.pdu_name_lidar_point_cloud);
            if (pdu == null) throw new ArgumentException($"ERROR: can not find pdu({robotName}/lidar_points)");
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
            count = 0;

            double[] state = SampleState.FromNode(this, Vector3.Zero);
            int rc = SensorNative.hako_sensor_scan(handle, state, 0.1, scanBuf, scanBuf.Length / 4,
                                                   out int height, out int width, out int pts);
            if (rc != 0)
            {
                GD.PrintErr("hako_sensor_scan(lidar) rc=" + rc + " err=" + SensorNative.LastError(handle));
                return;
            }

            int bytes = Math.Min(pts * PointStep, data.Length);
            Buffer.BlockCopy(scanBuf, 0, data, 0, bytes); // float[x,y,z,i] -> bytes (LE, PointField FLOAT32)

            INamedPdu pdu = pduManager.CreateNamedPdu(robotName, Default3DLiDARController.pdu_name_lidar_point_cloud);
            if (pdu == null) return;
            var pc = new PointCloud2(pdu);
            TimeStamp.Set(pc.header);
            pc.header.frame_id = "front_lidar_frame";
            pc.height = (uint)height;
            pc.width = (uint)width;
            pc.is_bigendian = false;
            pc.fields = pointFields;
            pc.point_step = PointStep;
            pc.row_step = (uint)(PointStep * width);
            pc.data = data;
            pc.is_dense = true;
            pduManager.WriteNamedPdu(pdu);
            pduManager.FlushNamedPdu(pdu);
        }

        private static PointField[] MakeFields(IPduManager pduManager)
        {
            string[] names = { "x", "y", "z", "intensity" };
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
