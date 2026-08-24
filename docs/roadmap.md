# Roadmap

## v0.1 technical prototype (this repository)

The vertical slice is functional: deterministic state and timing, a controllable four-wheel RWD ICE vehicle, high-fidelity stateful tires, simplified suspension, thermal powertrain and brakes, driver aids, environment, collision/damage, ten-vehicle LOD, snapshots/replay, telemetry, headless execution and Chromium/WASM delivery.

Before tagging v0.1.0:

- profile Chromium on representative desktop tiers and publish frame/step budgets;
- add canonical snapshot serialization and browser-side replay controls;
- add collision fixtures for every declared primitive and curb cases;
- make automatic transmission policy explicit and add keyboard clutch control;
- add structured audio/FFB events and telemetry export from the browser;
- obtain measured/fitted reference-vehicle and tire parameters;
- select the permissive license.

## v0.2–v0.x foundation hardening

- Versioned vehicle-data schema with units, validation and provenance
- Stable tire/suspension/differential/powertrain/aero contracts
- Brush and transient-brush tire implementations
- Suspension hard-point solver and compliance hooks
- Canonical snapshots, correction/resimulation and networking fixtures
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
