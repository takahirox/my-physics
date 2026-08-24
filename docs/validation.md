# Validation and acceptance evidence

Run the complete local gate with:

```bash
cargo fmt --check
cargo clippy --all-targets -- -D warnings
cargo test --all-targets
cargo build --release --target wasm32-unknown-unknown --lib
```

The integration suite currently covers:

| Test | Evidence |
|---|---|
| deterministic repeatability | independent runs produce the same full-state fingerprint |
| snapshot/replay equivalence | restore plus timed inputs reaches the original fingerprint |
| 1000 Hz priority timing | 1000 fixed steps advance both world and player exactly one second |
| render decoupling | different render batching reaches the identical state |
| longitudinal plausibility | full throttle accelerates the RWD reference car in local forward (-Z) |
| wet-road behavior | water reduces force and produces a non-zero hydroplaning state |
| tire failure progression | a finite puncture leaks pressure, changes patch state and advances failure |
| timestep guard | invalid variable timesteps are rejected |
| braking and brake heat | braking reduces speed and raises brake temperature |
| mutable mass and CG | consuming fuel changes both total mass and center of gravity |
| detached components | severe damage spawns an independent body and removes its mass |
| cumulative damage | thermal damage increases with exposure duration |

## Numerical strategy

The chassis uses semi-implicit Euler integration at 1 ms, which is robust for the current stiff but bounded forces. Tire forces are constrained by a combined-slip friction ellipse. Suspension travel, normal force and high-risk ratios are bounded. Every step rejects non-finite primary vehicle state.

## Required next validation

- Analytical free-fall, constant-force, yaw-inertia and energy-dissipation fixtures
- Skid pad, coast-down, braking-distance and steady-state cornering envelopes
- Parameter sweeps for timestep stability and thermal equilibrium
- Golden telemetry traces checked with tolerances, not only state hashes
- Comparison against instrumented reference-vehicle data
- Chromium performance traces for 1/10/100 vehicles and LOD transitions
- Cross-platform determinism matrix and snapshot fuzz/property tests

No claim of real-vehicle correlation is made until measured telemetry and parameter provenance are available.
