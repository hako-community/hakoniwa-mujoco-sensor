# Physical AI sensor contract — Phase S0

Status: accepted for `hakoniwa-mujoco-sensor` on 2026-09-19.  This is an
additive contract: it does not change any existing sensor payload or PDU
channel.  Runtime implementation is Phase S1/S2 work.

## Time and sequence

All contract times are signed 64-bit nanoseconds on the **Hakoniwa simulation
clock**.  Wall clock, renderer/UI clock, and a sensor-internal scan counter
must not be used as a source timestamp.  `0` is valid (simulation start), not
a missing-value sentinel.

| Field | Definition |
| --- | --- |
| `source_time_ns` | Time at which truth was sampled/captured. |
| `scheduled_time_ns` | Nominal release time after sampling schedule and modeled latency. |
| `publish_time_ns` | Simulation time at which the payload was emitted. |
| `sequence` | Monotonically increasing acquisition attempt number, reset only by `episode_replay`. |
| `consume_time_ns` | Consumer-side trace field; never written back into the sensor envelope. |

`publish_time_ns >= source_time_ns` is required.  Within one sensor stream,
published envelopes preserve increasing sequence order.  A dropped attempt
still consumes a sequence number, so a gap is observable.

## Envelope and health sidecar

The normative in-process types are in
`include/sensors/common/sensor_contract.hpp`.

```cpp
SensorEnvelope<Payload> { payload, sequence, source_time_ns,
  scheduled_time_ns, publish_time_ns, status, sensor_id, frame_id,
  profile_id, calibration_id };
```

Existing PDU payloads remain byte-compatible.  A future
`hako_msgs/SensorHealth` sidecar is emitted with the same `sensor_id`,
`sequence`, and `source_time_ns`; these three fields are the join key.  It
adds cumulative `dropped_count` and current `queue_depth`.  A health message
is emitted whenever its corresponding payload is emitted.  For a dropped
sample, the next published health message has `DROPPED_PREVIOUS` and the
updated count.  This lets Bool contact PDUs expose age and validity without
breaking their payload type.

| Bit | Name | Meaning |
| ---: | --- | --- |
| 0 | `VALID` | Numeric payload is usable. |
| 1 | `STALE` | At publication it exceeded configured maximum age. |
| 2 | `DROPPED_PREVIOUS` | At least one acquisition since the previous publish was dropped. |
| 3 | `SATURATED` | At least one output was clipped to its declared range. |
| 4 | `CALIBRATION_EXPIRED` | Calibration version/date is not valid. |
| 5 | `BACKEND_DEGRADED` | Requested fidelity is unavailable from the truth backend. |
| 6 | `FAULT_INJECTED` | A configured fault schedule altered this sample. |

`VALID` may coexist with `SATURATED`, `STALE`, or `BACKEND_DEGRADED`; it is
clear only when the measurement is invalid.  Consumers must not infer health
from a repeated payload value.

## Profiles and calibration

Profiles describe a population/error model.  Calibration describes a concrete
unit.  They have separate IDs and provenance, and both IDs are logged in every
envelope/health record.

Allowed profile kinds: `truth`, `ideal`, `datasheet`, `fitted`, `randomized`,
and `fault`.  Provenance is one of `assumed`, `datasheet`, `measured`, or
`fitted`.  `truth` is required for deterministic regression and must not gain
noise, latency, dropout, or calibration error.  A datasheet profile is not a
fitted profile.

`calibration_id: "none"` is permitted only for `truth`/`ideal`.  A deployed
calibration must declare its provenance and, where applicable, validity date.

## Seed and reset semantics

S1 will implement `SeedManager`, but its externally visible derivation is
fixed now:

```
stream seed = derive(experiment_seed, sensor_id, stream_id, axis_or_plane, episode_id)
```

The derivation must be stable across processes/platforms and must use explicit
domain separation for `measurement`, `latency`, `dropout`, and each axis.  No
two axes become correlated merely because they share a default RNG state.

`episode_replay` clears transient state, queue, sequence and restores every
stream to its episode seed.  `runtime_continue` clears only explicitly named
model state and never reseeds nor clears a transport queue.  Configurations
must select one of these modes; ambiguous `Reset()` behavior is forbidden.

## Capability and degradation

Backends publish machine-readable booleans for `multi_return`, `per_point_time`,
`material_response`, `doppler`, and `depth_invalid`.  A scenario requiring a
capability that is false must set `BACKEND_DEGRADED`; high-fidelity acceptance
gates then skip or fail according to the scenario policy.  A simple ray hit is
not evidence of material response or multi-return support.

## Configuration and trace

`config/schemas/sensor-contract.schema.json` is the versioned additive schema
used by new manifests/profiles.  It specifies only contract vocabulary; it
does not retroactively invalidate existing A-2 manifests.

Every run trace records: run/scenario/robot/sensor IDs; backend and version;
git commit/model hash; profile and calibration IDs; full seed tree; all three
sensor times plus consume time; sequence; status/drop/queue fields; truth and
post-model values.  Images and point clouds are external blobs referenced by
content hash and URI.
