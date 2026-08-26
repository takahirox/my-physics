# Architecture

## Invariants

1. One physical foundation serves games and engineering workflows. Fidelity changes computation, never parameter meaning.
2. Physics owns its clock. Rendering reads/interpolates states and never supplies the authoritative timestep.
3. State transitions follow stable vehicle order and wheel order. Authoritative stepping does not use parallel reductions.
4. Platform adapters stay outside the plant. The browser API is a thin export layer over `PhysicsWorld`.
5. Approximation boundaries are explicit and replaceable.

## Runtime flow

```text
raw device sample
       │
       ▼
calibration / normalization
       │
       ▼
deterministic input policy
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

The browser exposes the raw, normalized, policy, plant-input and post-aid stages with sample and physics-step numbers. Input-policy state is stepped on the physics clock and saved with the browser controller snapshot. Calibration and active-device selection remain platform-adapter concerns; they do not modify tire, rack or chassis parameters.

Game-facing profiles are controller configuration, not vehicle definitions. Accessible and Sport apply different lateral-acceleration targets to keyboard/gamepad gain and slew; Accessible also selects ESC on. Simulation sends normalized raw commands and leaves vehicle aids explicit. A calibrated wheel always bypasses speed gain and remains linear 1:1. Profile/controller/ESC state is included in browser snapshot restore.

The Arcade keyboard drift experiment adds another policy-layer controller after the normal speed-sensitive keyboard mapping. A deliberate physical handbrake, service-brake or throttle-lift entry must produce measured sideslip and yaw before it blends steering-only fine countersteer. Non-steering controls pass through unchanged, controller phase is snapshot state, and gamepad/wheel/Simulation paths bypass it. Its collision-free Flat proving-ground composition isolates the controller without changing the `ArcadeFun` vehicle or any tire/chassis force.

The compact WASM diagnostic getters identify stages as `0=raw`, `1=normalized`, `2=policy`, `3=plant input`, `4=post-aid`, and devices as `1=keyboard`, `2=gamepad`, `3=wheel`. Post-aid diagnostics additionally expose per-wheel braking and ABS plus TC/ESC activity. These are observability contracts; the existing `physics_set_input` raw application API remains available.

The native and WASM paths call this same flow. Non-player LOD caches expensive force evaluation for 4 or 10 base ticks while continuing rigid-body integration every 1 ms. A first-order fidelity transition plus 50 ms cached-force blending avoids abrupt LOD changes. Device benchmarks set an automatic fidelity ceiling and applications can override it.

The demo circuit is a physical three-dimensional road, not a rendering offset. Its cyclic centerline authors elevation, derives bounded banking from signed curvature, and exposes a deterministic local point/normal/forward/right frame. Suspension travel, tire forces, chassis-floor fallback, detached-body contact and barrier OBBs all use that frame. `SimulationConfig::ground_surface` selects it explicitly for circuit worlds; flat proving grounds and correlation runs retain `Flat`. WASM exports segment XYZ and the complete orthonormal frame, plus interpolated vehicle quaternions, so browser geometry can consume the authoritative definition without recreating elevation or banking in JavaScript.

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

`Snapshot::to_bytes` produces the canonical v0.1 wire format and `Snapshot::from_bytes` validates its magic, version, lengths, finite floating-point values, quaternion norms and checksum. Timed input history uses a separately typed archive. Format changes require a new version and migration policy; silent reinterpretation is forbidden. Snapshot v5 records the selected physical ground surface; v1-v4 migrate explicitly to `Flat`, matching the only ground geometry those archives supported.
