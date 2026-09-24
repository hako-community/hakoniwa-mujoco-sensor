# Phase S1 common core

Phase S1 adds portable C++20 primitives only.  No current PDU payload has been
changed and no operating-system clock, thread, filesystem, dynamic library, or
compiler-specific random/hash API is used.  The code therefore has the same
source behavior with MSVC (Windows), Apple Clang (macOS), and GCC/Clang
(Linux), subject to their C++20 standard-library implementations.

## Components

- `common/seed_manager.hpp`: specified FNV-1a plus SplitMix64 derivation,
  independent of implementation-defined `std::hash`.
- `noise/correlated_noise.hpp`: explicit 3x3 covariance (Cholesky) sampling.
  Independent axis streams are derived separately; correlation is intentional
  and supplied as covariance.
- `common/timed_sensor_pipeline.hpp`: simulation-clock scheduler, latency
  queue, normal-clipped jitter, Bernoulli/Gilbert-Elliott drops, stale status,
  overflow accounting, fault windows, and reset semantics.
- `common/measurement_limits.hpp`: output-range clamp with explicit saturation
  result; existing Gaussian quantization remains in `noise/noise.hpp`.

All randomness is local `std::mt19937` state initialized from the specified
seed manager.  Given a conforming C++ standard library, exact normal-distribution
sample values are implementation-defined; reproducibility is guaranteed for a
fixed platform/toolchain/version.  Cross-platform acceptance compares contract
invariants, sequence/status/timing, and distribution statistics rather than
bit-identical Gaussian values.

## Integration boundary

Existing sensors remain on `UpdateScheduler` during S1.  A sensor integration
in S2/S3 obtains all due instants from `SensorSchedule`, samples truth at each
instant, calls `TimedSensorPipeline::Enqueue`, then emits every result of
`PublishDue(simulation_time_ns)` through its existing payload adapter plus the
health sidecar.  This avoids changing legacy PDU layouts while supporting
catch-up on a slow physics step.

`EpisodeReplay` clears queue/counters and reseeds using the supplied episode;
`RuntimeContinue` is deliberately a no-op so a runtime reset cannot erase
in-flight observations or silently change the random stream.

Fault windows are evaluated against `source_time_ns`.  `Drop` omits the sample
and increments the normal loss accounting.  `Invalidate` publishes it with
`FAULT_INJECTED` and without `VALID`, preserving the traceable sequence while
requiring consumer fallback.  Sensor-specific stuck/biased/corrupted faults
can be layered as a measurement model in S2 without changing this timing
contract.
