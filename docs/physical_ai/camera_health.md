# Phase S2: camera formation and health integration

## Implemented camera contract

`CameraNoiseConfig.type` is now `none` by default.  When configured as
`gaussian`, RGB noise is applied after encoding in **8-bit pixel-value units**
and clamped to `[0, 255]`; depth noise is applied in **metres** only to finite,
positive samples.  Each concrete camera owns an RNG seeded by `noise.seed`, so
the stream advances frame by frame and is reproducible for a fixed toolchain.

This applies to mono RGB, depth, RGBD, and stereo cameras.  Capture failures
continue to produce an empty frame, preserving prior behavior.  Depth invalid
is contractually IEEE `NaN`; it remains `NaN` through noise processing and is
serialized as zero only for the existing `DEPTH_U16_MM` PDU representation.

The loader accepts optional `intrinsics` (`fx`, `fy`, `cx`, `cy`,
`distortion[]`) for RGB and depth configuration.  They are retained in the
profile now but are not silently applied to the MuJoCo renderer; distortion is
an S2 follow-up processor once an OpenCV-free/portable implementation and
CameraInfo sidecar are selected.

`CameraBundleMetadata` fixes the join fields for a future RGB/depth/CameraInfo
bundle: `sequence`, `source_time_ns`, and `frame_id`.

## SensorHealth PDU dependency

`SensorEnvelope` / transport-neutral `SensorHealth` are already available in
`sensors/common/sensor_contract.hpp`.  The checked-out PDU registry has no
generated `hako_msgs/SensorHealth` C++ PDU type, therefore an adapter cannot
be added safely without inventing an incompatible wire layout.  Add the message
to the Hakoniwa PDU registry, generate/install its `pdu_cpptype_*` headers, and
then implement `adapter/hako_msgs/sensor_health.hpp` using the existing
BatteryStatus adapter pattern.  Until that type exists, legacy payload PDUs
remain intentionally unchanged.

> Superseded 2026-09-25 (SENS-006 plan A): no registry type was added. SensorHealth
> is encoded by the owned `sensors/common/sensor_health_wire.hpp` codec and sent as
> standard `std_msgs/UInt8MultiArray` by `adapter/std_msgs/sensor_health.hpp`.
> See `docs/sensor_health_transport.md`.
