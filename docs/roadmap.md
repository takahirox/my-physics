# Roadmap

## v0.1 technical release (implemented)

The accepted vertical slice includes deterministic state/timing, a controllable four-wheel RWD ICE vehicle, stateful tires, simplified suspension, thermal powertrain/brakes, driver aids, environment, collision/damage, ten-vehicle LOD, versioned snapshots/replay, telemetry, Audio/FFB contracts, headless execution and Chromium/WASM 3D delivery. See the [acceptance matrix](v0.1-acceptance.md).

Release-administration items before tagging v0.1.0:

- obtain measured/fitted reference-vehicle and tire parameters;
- select the permissive license.

## v0.2–v0.x foundation hardening

- Versioned vehicle-data schema with units, validation and provenance
- Stable tire/suspension/differential/powertrain/aero contracts
- Brush and transient-brush tire implementations
- Suspension hard-point solver and compliance hooks
- Snapshot migrations, correction/resimulation and networking fixtures
- Broad-phase acceleration with deterministic pair ordering
- Road tiles, track import and richer wet/dry evolution
- Parameter identification and telemetry comparison tools

## v1.0 general-purpose release

- Documented Web, Windows and Linux APIs; Unity, Unreal and C ABI adapters
- Tested determinism profiles and cross-platform support matrix
- Physics LOD at approximately 100 vehicles under documented hardware budgets
- ICE, EV and hybrid powertrains; FF/FR/MR/RR/AWD; MT/AT/DCT/CVT; replaceable differentials
- Hard-point suspension, chassis flexibility and map-driven aero/wake effects
- Advanced track/weather state, richer mechanical reliability and physical damage
- Engineering and game-authoring vehicle-data paths
- Race Driver and deterministic Test Driver controller packages
- Real-data validation, parameter fitting and DIL/HIL-oriented external timing adapters
- Vehicle-class abstractions suitable for karts, trucks, buses and motorcycles without a four-wheel-only core

Still intentionally TBD: authoritative multiplayer topology, exact FFB bandwidth, detailed WebGPU partition, full drainage, detailed airflow cooling, fuel slosh, advanced poor-shift damage, full debris secondary damage, occupant injury analysis and automatic setup optimization.
