// SAMPLE (not a shipped artifact). Builds the C-ABI's state[15] from a Godot
// sensor node, expressed in GODOT WORLD coordinates (see GodotRayCaster.cs for
// why no ENU conversion is needed):
//   origin[3], forward[3], left[3], up[3], linear_velocity[3]
// with ROS axis meaning mapped onto Godot's basis:
//   forward (x) = -Basis.Z, left (y) = -Basis.X, up (z) = +Basis.Y.

using Godot;

namespace hakoniwa.mujoco.sensor.sample
{
    public static class SampleState
    {
        public static double[] FromNode(Node3D node, Vector3 linearVelocity)
        {
            Basis b = node.GlobalTransform.Basis;
            Vector3 origin = node.GlobalPosition;
            Vector3 forward = -b.Z;
            Vector3 left = -b.X;
            Vector3 up = b.Y;
            return new double[15]
            {
                origin.X,  origin.Y,  origin.Z,
                forward.X, forward.Y, forward.Z,
                left.X,    left.Y,    left.Z,
                up.X,      up.Y,      up.Z,
                linearVelocity.X, linearVelocity.Y, linearVelocity.Z,
            };
        }
    }
}
