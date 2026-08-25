# Validation and acceptance evidence

Run the complete local gate with:

```bash
./scripts/verify-v01.sh
```

## Declarative maneuver harness

The headless maneuver catalog covers coast-down, 0–100 km/h acceleration,
100–0 km/h ABS braking, steady steer/skidpad, step steer and slalom. Every
scenario declares its initial condition, deterministic input program, duration,
sample rate and quantitative acceptance bounds in `src/validation.rs`.

Run the fast summary acceptance used by CI:

```bash
cargo run --release --bin maneuver-validation -- --summary
```

Generate reproducible JSON and CSV summary plus time-series artifacts:

```bash
./scripts/run-maneuver-validation.sh target/maneuver-validation
```

Time series include speed, yaw rate, vehicle sideslip, acceleration, all four
wheel longitudinal slips, slip angles and normal loads. The integration suite
also checks independent-run determinism, left/right steering symmetry,
aerodynamic speed-squared scaling and fixed-timestep convergence.

These bounds establish regression continuity and physical plausibility only.
They are **not real-vehicle correlation**: no measured reference telemetry was
used to set them, and passing them must not be presented as proof that this
prototype matches a particular car.

The initial acceptance envelopes deliberately surround already-reviewed v0.1
behavior rather than claiming measured targets:

| Scenario | Quantitative envelope and rationale |
|---|---|
| coast-down | 12 s final speed 20.0–27.7 m/s, near-zero sideslip and positive wheel loads; catches missing/explosive resistance without pretending a measured coast curve exists |
| 0–100 km/h | target reached in 2–15 s with less than 0.03 rad sideslip; catches broken propulsion, shifts or straight-line symmetry |
| 100–0 km/h | 2 m/s threshold in 1.5–4.0 s and 25–50 m; contains the separately reviewed dry-ABS 31–42 m envelope with margin for the harness stop threshold |
| steady steer | final absolute yaw 0.04–0.30 rad/s, sideslip below 0.12 rad and wheel slip below 0.30; brackets the low-g bicycle-model regression |
| step steer | peak yaw 0.08–0.60 rad/s, sideslip below 0.15 rad and wheel slip below 0.35; rejects absent response and unstable reversal |
| slalom | peak yaw 0.08–0.80 rad/s, sideslip below 0.20 rad and 5–12 yaw sign changes; verifies response to the declared 0.5 Hz input |

For the 100–0 maneuver, reported distance is captured at the first 2 m/s
threshold crossing. The runner continues to the declared duration for a fixed
time-series shape, but post-stop rolling is excluded from braking distance.
Every artifact records the selected vehicle preset and definition/provenance
revision so future measured and gameplay definitions cannot be silently mixed.

The integration suite currently covers:

| Test | Evidence |
|---|---|
| deterministic repeatability | independent runs produce the same full-state fingerprint |
| snapshot/replay equivalence | restore plus timed inputs reaches the original fingerprint; the WASM K/L fixture also restores controller slew/command, profile, ESC, input mode and autopilot before reproducing all public physical values |
| 1000 Hz priority timing | 1000 fixed steps advance both world and player exactly one second |
| render decoupling | different render batching reaches the identical state |
| browser motion correspondence | Chromium telemetry confirms rendered world displacement tracks physical speed while camera lag remains bounded |
| visual speed cues | curb bands, fence posts, asphalt/rubber details and camera presets use tested metric spacing/configuration independent of circuit segment count; Chromium frame-time evidence guards rendering cost |
| circuit collision correspondence | WebGL road segments and barriers are generated from the same 240-segment, 2.06 km closed spline as the Rust collision core; a remote curved barrier has a regression test |
| circuit scale envelope | sampled centerline radius is required to remain at least 25 m; the current minimum is 26.41 m, corresponding to about 62 km/h at 1.15 g |
| complete-lap behavior | `cargo run --release --example circuit_lap` requires a damage-free lap inside the safe lateral envelope and reports lap time, speed and line error |
| input pipeline | raw, normalized, policy, plant and aid stages carry sample/physics-step numbers; snapshots restore all controller stages, while 30/60/120/144 Hz application grouping reaches identical public state |
| keyboard steering | Sport Adaptive is the browser default and is checked at 50/100/140 km/h for monotonic half/full response and less than 15 degrees peak front slip; explicit settled Simulation Digital Raw/Test reaches full rack, and mode/device changes are bumpless |
| gamepad and wheel | Accessible (7.5 m/s²) and Sport (10.0 m/s²) gamepad targets are sampled at 50/100/140 km/h for quarter/half monotonic response; Simulation is normalized raw. Pure-function tests cover deadzones/expo/calibration and the wheel remains calibrated linear 1:1 in every profile. Both keyboard and gamepad policy produce identical state under 30/60/120/144 Hz render grouping. |
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
| lateral tire curve and transients | low-slip gradient is retained; lateral force has a finite peak/sliding branch, aligning trail decays, distance-based relaxation is speed/dt invariant and serialized |
| tire thermal energy | tread/bulk/road/air ledger, severe-slip and release bounds, 0.5–20 ms convergence and 50/100/140 km/h steering matrices |
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

The reference tire's relaxation and two-node thermal constants are authored and
dimensionally explicit. Their physical-law gates cover force bounds, symmetry,
distance response and energy balance; temperature and handling bands remain
authored regression criteria until measured tire data is available. Detailed
equations and before/after traces are in [Reference tire](tire-model.md).

## Required next validation

- Additional free-fall, yaw-inertia and energy-dissipation fixtures
- Parameter sweeps for timestep stability and thermal equilibrium
- Golden telemetry traces checked with tolerances, not only state hashes
- Comparison against instrumented reference-vehicle data
- Performance traces for additional hardware tiers and the v1.0 100-vehicle target
- Cross-platform determinism matrix and snapshot fuzz/property tests

No claim of real-vehicle correlation is made until measured telemetry and parameter provenance are available.

Detailed steering investigation and tuning iterations are recorded in [Steering and circuit validation](steering-validation.md).
