# Validation and acceptance evidence

Run the complete local gate with:

```bash
./scripts/verify-v01.sh
```

The integration suite currently covers:

| Test | Evidence |
|---|---|
| deterministic repeatability | independent runs produce the same full-state fingerprint |
| snapshot/replay equivalence | restore plus timed inputs reaches the original fingerprint; the WASM K/L fixture also restores keyboard slew/command, input mode and autopilot before reproducing all public physical values |
| 1000 Hz priority timing | 1000 fixed steps advance both world and player exactly one second |
| render decoupling | different render batching reaches the identical state |
| browser motion correspondence | Chromium telemetry confirms rendered world displacement tracks physical speed while camera lag remains bounded |
| visual speed cues | curb bands, fence posts, asphalt/rubber details and camera presets use tested metric spacing/configuration independent of circuit segment count; Chromium frame-time evidence guards rendering cost |
| circuit collision correspondence | WebGL road segments and barriers are generated from the same 240-segment, 2.06 km closed spline as the Rust collision core; a remote curved barrier has a regression test |
| circuit scale envelope | sampled centerline radius is required to remain at least 25 m; the current minimum is 26.41 m, corresponding to about 62 km/h at 1.15 g |
| complete-lap behavior | `cargo run --release --example circuit_lap` requires a damage-free lap inside the safe lateral envelope and reports lap time, speed and line error |
| keyboard steering | the WASM fixture requires raw A/D to reach normalized full rack input after one physics step; the optional adapter is checked at 50/100/140 km/h for monotonic half/full response and less than 15 degrees peak front slip, and its digital-controller lap must finish without damage |
| high-speed steering and ESC | raw plant input remains effective and left/right symmetric; oversteer and opposite-yaw fixtures select physically corrective brake corners; the browser race preset starts with ESC off and exposes an E-key toggle |
| longitudinal plausibility | full throttle accelerates the RWD reference car in local forward (-Z) |
| automatic shift acceleration | a 20-second full-throttle run shifts sequentially, remains below the limiter, avoids over-rev failure and continues accelerating |
| wet-road behavior | water reduces force and produces a non-zero hydroplaning state |
| tire failure progression | a finite puncture leaks pressure, changes patch state and advances failure |
| timestep guard | invalid variable timesteps are rejected |
| braking and brake heat | braking reduces speed and raises brake temperature |
| mutable mass and CG | consuming fuel changes both total mass and center of gravity |
| detached components | severe damage spawns an independent body and removes its mass |
| cumulative damage | thermal damage increases with exposure duration |
| persistent archives | complete snapshot equality, checksum rejection and timed-input round trip |
| analytical integration | semi-implicit constant-acceleration result matches its discrete closed form |
| force constraint | combined tire force remains within the computed friction circle |
| dry longitudinal tire curve | the reference tire peaks between 10–20% slip and locked-wheel sliding force remains 70–90% of peak |
| dry ABS envelope | 100 km/h stopping distance, lock duration, target-slip occupancy, ABS-off comparison and mid-stop snapshot/resimulation are bounded |
| limit cornering | two friction-feasible high-g steering ramps remain forward-facing, low-slip and left/right symmetric; a low-g ramp remains within 5% of the bicycle reference |
| collision primitives | rotated box, capsule, convex and vehicle pair fixtures produce contacts/damage |
| effective failures | clutch/gearbox wear reaches physical failure and emits events |
| long-run stability | quaternion norm and finite state remain bounded in fixed/variable scenarios |
| smooth LOD | fidelity changes are bounded per tick and converge to the selected profile |
| continuous interfaces | Audio/FFB state and discrete physical events are populated |
| golden regression | the fixed two-second ten-vehicle scenario matches reviewed cross-platform telemetry tolerances |
| conservation/plausibility | pair-collision planar momentum and reference braking distance remain within bounds |

## Numerical strategy

The chassis uses semi-implicit Euler integration at 1 ms, which is robust for the current stiff but bounded forces. Tire forces are constrained by a combined-slip friction ellipse. Flat-road suspension reactions and tire forces use the road-normal/tangent basis rather than feeding chassis roll into the contact plane. Suspension travel, normal force and high-risk ratios are bounded. Every step rejects non-finite primary vehicle state.

The engineering-reference RWD definition uses symmetric 1.0 tire-fitment scales. The separate browser race preset assigns authored rear-tire scales of 1.05 cornering stiffness and 1.06 peak grip to provide a measurable understeer gradient. It is a game calibration, not a measured vehicle fit or a claim of real-world correlation. Both presets use identical physical equations and expose their provenance.

## Required next validation

- Additional free-fall, yaw-inertia and energy-dissipation fixtures
- Skid pad, coast-down, braking-distance and steady-state cornering envelopes
- Parameter sweeps for timestep stability and thermal equilibrium
- Golden telemetry traces checked with tolerances, not only state hashes
- Comparison against instrumented reference-vehicle data
- Performance traces for additional hardware tiers and the v1.0 100-vehicle target
- Cross-platform determinism matrix and snapshot fuzz/property tests

No claim of real-vehicle correlation is made until measured telemetry and parameter provenance are available.

Detailed steering investigation and tuning iterations are recorded in [Steering and circuit validation](steering-validation.md).
