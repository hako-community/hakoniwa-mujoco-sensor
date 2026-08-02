# Godot API-direct sensor sample (Phase 2 / 4th slide)

**These files are a SAMPLE, not a shipped artifact.** The shipped Phase 2
deliverable is the C-ABI in [`../../capi/`](../../capi/)
(`libhako_mujoco_sensor_capi.so`). This directory shows one consumer of that
ABI: Godot driving the mujoco-sensor LiDAR/Radar **models in-process**, ray
casting the live Godot scene — **no core-pro, no PDU transport, no MuJoCo
process** is needed to sense.

`hakoniwa-godot-drone` stays independent: nothing here is committed to it, and
its default build never references `libhako_mujoco_sensor_capi.so`.

## Files

| File | Role |
|------|------|
| `SensorNative.cs` | `[DllImport]` P/Invoke bindings for the C-ABI (no `unsafe`). |
| `GodotRayCaster.cs` | Implements `hako_raycast_fn` with `PhysicsDirectSpaceState3D.IntersectRay`. mujoco-sensor never learns the caster is Godot. |
| `SampleState.cs` | Builds `state[15]` from a Godot sensor node (Godot-world frame). |
| `MujocoLidar3DController.cs` | `ILiDAR3DController` producing `lidar_points` via the C-ABI. Drop-in for `Default3DLiDARController`. |
| `MujocoRadar3DController.cs` | `IRadar3DController` producing `radar_points` via the C-ABI. Drop-in for `Default3DRadarController`. |

## Why no coordinate conversion in the ray caster

The controller expresses the sensor pose to the model **directly in Godot-world
coordinates**: `origin = GlobalPosition`, and the ROS basis mapped onto Godot's
basis — `forward (x) = -Basis.Z`, `left (y) = -Basis.X`, `up (z) = +Basis.Y`.
The model builds each ray direction as a linear combination of those world basis
vectors, so the direction handed to `GodotRayCaster` is already Godot-world and
casts directly. The model then derives the point cloud in **sensor-local ROS**
(x fwd, y left, z up) purely from angle+depth — identical to the Pattern A / PDU
layout `LiDARPointCloudVisualizer` / `RadarPointCloudVisualizer` already render,
so nothing downstream changes.

## Build the C-ABI shared library

```bash
cd hakoniwa-mujoco-sensor/capi
bash build.bash          # -> libhako_mujoco_sensor_capi.so (+ smoke_capi)
./smoke_capi             # backend-free check: lidar3d + radar return points
```
(or via CMake: the `hako_mujoco_sensor_capi` SHARED target + `ctest -R
hako_sensor_capi_smoke`.)

## Run it in godot-drone (temporary verification only)

1. Copy this directory's `*.cs` into a folder under `hakoniwa-godot-drone/`
   (e.g. `Scripts/Samples/MujocoSensor/`).
2. Put the shared library where the loader/`LD_LIBRARY_PATH` can find it:
   ```bash
   cp capi/libhako_mujoco_sensor_capi.so \
      hakoniwa-godot-drone/Plugins/Linux/x86_64/
   ```
   Either add `"hako_mujoco_sensor_capi"` to `HakoLibLoader.IsHakoLibrary`
   (temporary), or ensure `Plugins/Linux/x86_64` is on `LD_LIBRARY_PATH` (the
   `localsim` launchers already prepend it) so the default resolver finds it.
3. In the drone scene, **replace** the `Default3DLiDARController` /
   `Default3DRadarController` node with `MujocoLidar3DController` /
   `MujocoRadar3DController`. Keep `ExternalSensing = false`: these nodes are the
   producers. `DroneAvatar` finds them via `ILiDAR3DController` /
   `IRadar3DController` and drives them exactly like the defaults.
   - LiDAR: `LiDARPointCloudVisualizer` reads `lidar_points` from the PDU — works
     unchanged.
   - Radar: leave the radar visualizer's source unset so it reads `radar_points`
     from the PDU (the same path Pattern A uses); do **not** call `SetSource`
     (that method takes the concrete `Default3DRadarController`).
4. `dotnet build`, run, confirm the point cloud renders (with `HAKO_ENV_OBB` set,
   against the reconstructed room walls).
5. **Revert**: delete the copied `*.cs`, remove the `.so`, undo any
   `HakoLibLoader` edit, restore the scene. `hakoniwa-godot-drone` is clean again.

## Relationship to Phase 1 (PDU path)

Phase 1 (Pattern A) delivers sensor point clouds over core-pro/PDU and is the
godot-drone standard. This Phase 2 API-direct path is an **additive option** a
consumer enables via this sample; the two coexist. See
`devai/arch/phase2_api_layer_plan_20260701.md`.
