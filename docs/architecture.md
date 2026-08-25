# Architecture

## Invariants

1. One physical foundation serves games and engineering workflows. Fidelity changes computation, never parameter meaning.
2. Physics owns its clock. Rendering reads/interpolates states and never supplies the authoritative timestep.
3. State transitions follow stable vehicle order and wheel order. Authoritative stepping does not use parallel reductions.
4. Platform adapters stay outside the plant. The browser API is a thin export layer over `PhysicsWorld`.
5. Approximation boundaries are explicit and replaceable.

## Runtime flow

```text
timed driver input
       │
       ▼
  ABS / TC / ESC ────── separate controller
       │ control commands
       ▼
 engine ─ clutch ─ gearbox ─ open differential
       │                         │
       └──────────────────► wheel rotation
                                 │
road state ─► tire model ─► forces / moments
                                 │
wind ─► aero ─────────────► 6-DoF chassis
                                 │
                       collision / physical damage
                                 │
                     snapshot + telemetry + events
```

The native and WASM paths call this same flow. Non-player LOD caches expensive force evaluation for 4 or 10 base ticks while continuing rigid-body integration every 1 ms. A first-order fidelity transition plus 50 ms cached-force blending avoids abrupt LOD changes. Device benchmarks set an automatic fidelity ceiling and applications can override it.

## Determinism policy (v0.1)

- Fixed `f64` operations, fixed iteration order and fixed timestep for authoritative runs
- No random source in the physics step
- No GPU computation or nondeterministic task scheduling in the authoritative path
- Quaternion normalization every integration step
- FNV-1a state fingerprint used by regression tests
- Snapshots clone every authoritative world field; input frames are indexed by physics step
- Canonical little-endian archives carry an explicit version, bounded lengths and a whole-payload checksum

Current tests establish repeatability on one toolchain/platform. They do not claim identical bits across all CPUs, browsers or compiler versions. v1.0 needs documented math routines, compiler flags, plugin rules and a tested determinism matrix.

## Model extension seams

`TireModel` is the first public replaceable physical interface. Suspension, differential, powertrain and aerodynamics are kept in clearly bounded modules/data and will receive stable traits after reference behavior is validated. Freezing all plugin interfaces in v0.1 would preserve the wrong abstractions.

Vehicle definitions separate constants from runtime state. Built-in engineering-reference and race-gameplay presets use one schema and the same physical equations. Fixed parameter groups carry origin (`measured`, `derived`, `fitted`, `estimated`, `authored`), source, revision, uncertainty and named validity ranges with units. The current built-ins make no measured-data claim. See [Vehicle presets and parameter provenance](vehicle-data.md).

## Snapshot compatibility

`Snapshot::to_bytes` produces the canonical v0.1 wire format and `Snapshot::from_bytes` validates its magic, version, lengths, finite floating-point values, quaternion norms and checksum. Timed input history uses a separately typed archive. Format changes require a new version and migration policy; silent reinterpretation is forbidden.
