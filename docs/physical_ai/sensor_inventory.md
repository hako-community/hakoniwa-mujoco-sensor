# Phase S0 sensor/profile/capability inventory

Snapshot date: 2026-09-19.  This inventory describes the current repository,
not promised S1 behavior.  `truth` below means the existing deterministic
path, not a new runtime profile file.

| Sensor family | Current implementation | Current profile state | Backend capability / known limit |
| --- | --- | --- | --- |
| IMU / MJCF IMU | `imu_sensor`, `mjcf_imu_sensor` | Per-axis noise configuration; no contract profile selection | MuJoCo truth; no common timing/health layer. |
| Joint state | `joint_state_sensor` | Direct qpos/qvel path, optional noise settings | MuJoCo truth; no encoder transfer model. |
| Force/torque | `force_torque_sensor` | Axis noise configuration | No 6x6 calibration/cross-talk contract yet. |
| Contact | `contact_sensor` | Thresholded Bool path | No raw-force/hysteresis health contract yet. |
| RGB / depth / RGBD / stereo | camera sensors and MuJoCo renderer | Loader reads RGB Gaussian-noise config; capture TODO leaves it unapplied | GL renderer; no declared distortion, depth-invalid, latency, or drop support. |
| 2D/3D LiDAR | `lidar_scan_sensor`, `lidar3d_sensor` | Range/axis-noise configuration | `IRayCaster`: single ray/hit; no declared multi-return, material response, or per-point time. |
| Radar | `radar_sensor` | Radar equation/RCS settings and `noise_seed` in A-2 examples | Single-hit ray model with target velocity/RCS fallback; no clutter or multipath capability. |
| Ultrasonic | `ultrasonic_sensor` | Range-noise configuration | Ray-based; no cone/crosstalk capability declaration. |
| Odometry / TF | `odometry_sensor`, `tf_publisher` | Existing direct state path | No common timing/health layer. |
| Battery | `battery_model` | Model-specific configuration | No common timing/health layer. |

Current A-2 manifests instantiate `lidar_3d`, `lidar_2d`, and `radar` through
`runtime/sensor_runtime.hpp`.  Other sensor implementations are consumed by
their owning robot integrations.  The existing `noise_seed` parameter is
specific to the A-2 radar path; it is **not** the Phase S1 seed-tree contract.

## Truth regression baseline

The following pre-S1 tests are the retained truth baseline.  Their input
fixtures and expected assertions must remain unchanged under the `truth`
profile.  The new S0 `sensor_contract_test` verifies the pure contract only;
it does not claim a runtime health implementation.

| Baseline | Fixture / assertion |
| --- | --- |
| `radar_math_test` | Deterministic radar geometry/equation math. |
| `sensor_stamp_test` | Range-sensor scan timestamps and converted PDU headers, when PDU types are available. |
| `sensor_mount_velocity_test` | Analytic mount lever-arm velocity, when PDU types are available. |
| `pdudef_channels_test` | PDU channel layout from pdudef. |
| `mujoco_radar_integration_test` | MuJoCo ray-caster integration when matching MuJoCo headers/library are available. |
| `hako_sensor_capi_smoke` | Backend-injected LiDAR/Radar C-ABI smoke test. |

The build/test command and toolchain resolution are intentionally environment
dependent; the exact CTest result, compiler, MuJoCo version, PDU type version,
and git revision must be captured when producing a binary golden trace.  No
binary trace is committed in S0 because this checkout may not have matching
runtime/PDU/MuJoCo dependencies; fabricating one would not be a golden result.
