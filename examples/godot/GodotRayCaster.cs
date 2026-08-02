// SAMPLE (not a shipped artifact). Implements the C-ABI's hako_raycast_fn with
// Godot's PhysicsDirectSpaceState3D.IntersectRay, so the mujoco-sensor model can
// "see" the Godot scene. mujoco-sensor never learns the caster is Godot -- it
// only calls the injected function pointer.
//
// Frame convention (kept trivial on purpose): the controller expresses the
// sensor pose to the model directly in GODOT WORLD coordinates -- origin =
// GlobalPosition, and forward/left/up = the sensor node's Godot basis mapped to
// ROS axis meaning (x fwd = -Basis.Z, y left = -Basis.X, z up = +Basis.Y). The
// model builds each ray direction as a linear combination of those world basis
// vectors, so `dir` arrives here already in Godot world and needs NO conversion.
// The model then derives the returned point cloud in sensor-local ROS
// coordinates purely from angle+depth, matching the Pattern A / PDU layout that
// LiDARPointCloudVisualizer / RadarPointCloudVisualizer already render.

using System;
using System.Runtime.InteropServices;
using Godot;

namespace hakoniwa.mujoco.sensor.sample
{
    public sealed class GodotRayCaster
    {
        private readonly Node3D owner;

        // Keep the delegate instance alive for the lifetime of this object: once
        // it is marshalled to a native function pointer, the GC must not collect
        // it while the native handle can still call back.
        public HakoRaycastFn Fn { get; }

        public GodotRayCaster(Node3D owner)
        {
            this.owner = owner;
            this.Fn = Cast;
        }

        private int Cast(IntPtr user, IntPtr origin, IntPtr dir, double maxDist,
                         IntPtr outDist, IntPtr outPoint, IntPtr outNormal,
                         IntPtr outTargetVel, IntPtr outTargetId)
        {
            var o = new double[3]; Marshal.Copy(origin, o, 0, 3);
            var d = new double[3]; Marshal.Copy(dir, d, 0, 3);

            var from = new Vector3((float)o[0], (float)o[1], (float)o[2]);
            var dirv = new Vector3((float)d[0], (float)d[1], (float)d[2]); // unit
            var to = from + dirv * (float)maxDist;

            var space = owner.GetWorld3D().DirectSpaceState;
            var query = PhysicsRayQueryParameters3D.Create(from, to);
            var hit = space.IntersectRay(query);
            if (hit.Count == 0) return 0;

            var pos = (Vector3)hit["position"];
            double dist = from.DistanceTo(pos);
            Marshal.Copy(new[] { dist }, 0, outDist, 1);
            Marshal.Copy(new double[] { pos.X, pos.Y, pos.Z }, 0, outPoint, 3);

            Vector3 nrm = hit.ContainsKey("normal") ? (Vector3)hit["normal"] : Vector3.Zero;
            Marshal.Copy(new double[] { nrm.X, nrm.Y, nrm.Z }, 0, outNormal, 3);

            // Doppler: report the hit body's world velocity when it is a rigid
            // body; otherwise zero (a sample would add per-collider finite diff).
            Vector3 tv = Vector3.Zero;
            int targetId = -1;
            if (hit.ContainsKey("collider") && hit["collider"].Obj is GodotObject collObj)
            {
                if (collObj is RigidBody3D rb) tv = rb.LinearVelocity;
                if (collObj is Node3D n3) targetId = (int)(n3.GetInstanceId() & 0x7fffffff);
            }
            Marshal.Copy(new double[] { tv.X, tv.Y, tv.Z }, 0, outTargetVel, 3);
            Marshal.WriteInt32(outTargetId, targetId);
            return 1;
        }
    }
}
