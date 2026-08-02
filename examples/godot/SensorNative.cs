// SAMPLE (not a shipped artifact). P/Invoke bindings for libhako_mujoco_sensor_capi.so.
//
// This is the Godot-side companion to the Phase 2 C-ABI deliverable
// (capi/hako_sensor_capi.h). It lets Godot drive the mujoco-sensor LiDAR/Radar
// models IN-PROCESS (no core-pro, no PDU, no MuJoCo) by injecting a Godot ray
// cast through hako_raycast_fn. See README.md for how to drop this into
// hakoniwa-godot-drone for a verification run.
//
// No `unsafe` is used (godot-drone's csproj does not enable it): pointer
// arguments are marshalled as IntPtr and read/written with System.Runtime
// .InteropServices.Marshal.

using System;
using System.Runtime.InteropServices;

namespace hakoniwa.mujoco.sensor.sample
{
    // Mirrors hako_raycast_fn. Return 1 on hit, 0 on miss. All array pointers are
    // double[3] (origin/dir/out_point/out_normal/out_target_vel); out_dist is
    // double*, out_target_id is int*. Every pointer is non-null when called.
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public delegate int HakoRaycastFn(
        IntPtr user, IntPtr origin, IntPtr dir, double maxDist,
        IntPtr outDist, IntPtr outPoint, IntPtr outNormal,
        IntPtr outTargetVel, IntPtr outTargetId);

    public static class SensorNative
    {
        // Resolved by hakoniwa-godot-drone's HakoLibLoader (add the name there) or
        // by LD_LIBRARY_PATH pointing at the .so. See README.md.
        private const string Lib = "hako_mujoco_sensor_capi";

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr hako_sensor_create(
            string type, string configJson, HakoRaycastFn raycast, IntPtr user);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern int hako_sensor_scan(
            IntPtr h, double[] state, double dtSec,
            float[] outPointsXyzi, int maxPoints,
            out int outHeight, out int outWidth, out int outCount);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern void hako_sensor_reset(IntPtr h);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern void hako_sensor_destroy(IntPtr h);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr hako_sensor_last_error(IntPtr h);

        public static string LastError(IntPtr h)
        {
            IntPtr p = hako_sensor_last_error(h);
            return p == IntPtr.Zero ? "" : (Marshal.PtrToStringAnsi(p) ?? "");
        }
    }
}
