# SENS-013–017: offline workflow and range models

## Scope and validation

These changes implement the backlog acceptance conditions at library/offline-tool
level. They do not certify a real sensor or an Isaac physics/rendering installation.
Tests use explicit synthetic fixtures and an injected ray caster.

- SENS-013: insufficient fidelity is rejected by the CLI and SensorFactory.
- SENS-014: independent-run validation produces a hashed report; a passing
  validation additionally produces a `sens005-v1` fitted profile.
- SENS-015: RGB/RGBA, RGB-D and LiDAR exported arrays become a common trace.
  Isaac runtime capture and matching MuJoCo/Isaac scenario execution remain
  unverified here; these are separate S5 environment integration tasks.
- SENS-016: channel elevation/firing tables, point offsets and empirical intensity.
- SENS-017: seeded probability/clutter, range/Doppler bins, per-ray metrics.

## Capability gate

```sh
python tools/sensor_workflow.py gate config/capabilities/mujoco_geometric_raycast.json config/capabilities/require_multipath.json --report build/gate.json
```

This example must exit 2. Boolean capabilities and named fidelity strings are
matched exactly. Missing keys never satisfy a requirement (even `false`).
`on_missing` supports `fail`, `skip`, `degraded`: all three exit nonzero and
`pass=false`; a degraded backend cannot pass a high-fidelity gate. Status bit 32
is BACKEND_DEGRADED. A successful gate has status bit 1.

For runtime manifests, put `required_capabilities` on the sensor component next
to `params`. SensorFactory throws on mismatch before constructing the sensor.
Runtime uses a conservative descriptor for the legacy PDU transport; an injected
ray caster alone does not establish support for optical materials or multipath.

## Fit and validation

```sh
python tools/sensor_workflow.py fit training.csv holdout.csv --thresholds thresholds.json --profile-id experiment.sensor.v1 --profile build/fitted.json --report build/validation.json
```

CSV header: `run_id,timestamp_ns,sensor_id,axis,unit,truth,measured`; optional
`status` must be 1. Timestamps are nonnegative, strictly increasing per run/axis.
Inputs must contain finite, synchronized numeric measurements, curated in SI.
Each axis requires at least two samples. Different runs and hashes are required
for fitting and holdout. Sensor/axis/unit sets must match.

Threshold example:

```json
{"imu":{"x":{"unit":"m/s2","max_bias":0.02,"max_rmse":0.1}}}
```

Bias and population standard deviation are estimated from training residuals
`measured-truth`. Holdout applies the fixed training bias and checks corrected
bias/RMSE. Failed holdout exits 2 and writes only a failing report. New distinct
output paths are mandatory, preventing a stale passing profile from being
mistaken for the result. The fitted profile references the exact SHA256 of the
training log and serialized holdout report; the report includes holdout hash,
run IDs, sample counts, thresholds and metrics.

This is a generic additive scalar model for extracted sensor features, not an
Allan/PSD, camera extrinsic, latency identification or 6x6 FT solver. Raw image
pixels and point clouds need explicit feature extraction before this CLI. It
does not install or activate profiles in consumers.

## Isaac export and common trace

Requires Python >=3.9 and NumPy for `isaac_trace_export.py`; the converter and
calibration CLI otherwise use only the standard library.

Use `IsaacTraceExporter(directory, manifest)` in the acquisition script. Call
`capture(metadata, arrays)` for each synchronized acquisition, then `close()`.
Arrays are CPU NumPy arrays under these explicit annotator names:

| Name | dtype / shape | Meaning |
|---|---|---|
| rgb / rgba | uint8 H×W×3 / H×W×4 | color |
| distance_to_image_plane | float32-le H×W | optical-axis depth, metres |
| distance_to_camera | float32-le H×W | radial distance, metres |
| pointcloud | float32-le N×3 | XYZ, metres in declared frame |

The acquisition layer explicitly converts stage units/coordinates and provides
physics-clock nanoseconds. No wall-clock/timeline fallback, projection-depth
conversion or implicit world-to-sensor transform is applied. RGB-D dimensions
must match; arrays passed together must come from the same acquisition.
NaN/Inf invalid depth samples are preserved byte-for-byte in binary blobs.
An empty `[0,3]` cloud remains a valid frame. No intensity/timing channels are
invented for an Isaac XYZ export.

Manifest fields: `run_id`, `scenario_id`, `robot_id`, `backend`,
`backend_version`, `git_commit`, `model_sha256`, `profile_id`, `calibration_id`,
`seed_tree`, `descriptor` (backend/capabilities), `scenario`
(scenario_id/required_capabilities). Supply actual provenance, not defaults.

Metadata fields: `sensor_id`, `frame_id`, `sequence`, `source_time_ns`,
`scheduled_time_ns`, `publish_time_ns`, `consume_time_ns`, `health`
(status_bits/drop_count/queue_depth), `context` (pose, lighting, materials etc.).
`raw_truth` and `observation` are optional and become null if unavailable.
These roles are not inferred from a renderer's output. Every trace row retains
identity/provenance, four distinct timestamps, sequence, data, health/age, context.

For an already exported `manifest.json`, JSONL records and raw blobs:

```sh
python tools/common_trace.py export/manifest.json export/records.jsonl --output build/common.jsonl
```

Records carry `annotators` entries with URI, SHA256, shape, dtype and metric unit.
The converter validates capability, monotonic streams, causal timestamps,
dimensions, byte counts and content hashes before writing a trace. Output blob
URIs are absolute file URIs; retain the export directory when moving a trace.
The same explicit export contract can be supplied by MuJoCo and real-log writers.

Reference APIs (the exporter is independent of their imports/version changes):
- [Isaac Camera API](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/py/source/extensions/isaacsim.sensors.camera/docs/index.html)
- [RTX LiDAR annotators](https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_rtx_based_lidar.html)

## LiDAR

`channel_elevation_deg` and `channel_firing_offset_sec` are optional channel-sized
tables. Offsets must lie within a column interval. Default offsets interleave
channels; organized point storage remains channel-major. `point_time_offset_sec`
is parallel to points and relative to scan start (`stamp_sec - update_period`).
`ScanAt(state, scan_end_sec, frame)` accepts an explicit simulation timestamp.
The pose/world is still sampled once: firing times do not imply motion distortion.
Offsets remain internal; the existing XYZI PDU and C ABI remain unchanged.

Enable `empirical_intensity` for clipped
`reflectivity * incidence_cosine * (reference_distance / range)^2`.
Missing surface normals assume normal incidence. No return gives zero intensity;
the old default gives one on hits. This is a scalar approximation, not optical
material simulation. Parameters are exposed through runtime manifest `params`.

## Radar

New optional manifest keys:
`detection_probability_scale` (0..1, multiplies existing range/RCS Pd),
`false_alarm_probability` (0..1 per empty ray), `clutter_velocity_stddev`,
`doppler_stddev`, `doppler_resolution`, `range_resolution` (SI).
Clutter uses uniform range/FOV ray opportunities and zero-mean Gaussian velocity;
it does not implement multipath or physical distributed clutter.
Noise and clutter have seeded streams; Reset restarts scan time and RNGs.

`RadarScanFrame` exposes target_trials/target_detections/empty_trials/false_alarms
and doppler_squared_error_sum. Aggregate counters across scans before computing:
Pd = detections/target_trials; Pfa = false_alarms/empty_trials;
Doppler RMSE = sqrt(error_sum/target_detections). A zero denominator means no
estimate, not zero error. These are per-ray opportunities, not object tracking
metrics. Legacy detection PDU layout does not change.

## Reproduce tests

Build `range_fidelity_test`, `radar_math_test`, `sensor_contract_test`,
`sensor_pipeline_test`, `sensor_stamp_test`, `sensor_mount_velocity_test` and run
their CTest entries plus `sensor_workflow_test`. Python tests can also run via
`python tests/sensor_workflow_test.py` (NumPy required for exporter coverage).
The range fixture uses 10,000 independent ray opportunities and checks Pd/Pfa
within 0.02, deterministic Reset, Doppler quantization error, LiDAR firing times,
inverse-square intensity, no-return and invalid configurations.
