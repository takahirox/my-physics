# Steering and circuit validation

The original steering complaint was reproduced before changing the physical rack. At 140 km/h, raw full input commanded 30.9 degrees of road-wheel angle, produced roughly 44–54 degrees of front slip and could turn less than half input. The compact circuit's sampled minimum radius was 10.09 m, whose 1.15 g speed envelope is only 38.4 km/h. The issue was therefore a circuit/input/control mismatch rather than missing rack travel.

## Tuning iterations

All values below come from the deterministic native probes. `Max error` is absolute lateral centerline error unless noted.

| Iteration | Geometry/controller | Lap | Max speed | Max error | Damage | Outcome |
|---|---|---:|---:|---:|---:|---|
| Baseline | 0.720 km, 160 segments, 10.09 m minimum radius | 63.8 s | 85.0 km/h | 1.55 m | 0.000 | Clean only at a kart-like average speed; raw digital steering saturated at road speed. |
| Scale prototype | 1.800 km, 160 segments, coarse minimum 25.2 m | no lap | about 122 km/h | 6.81 m center-distance metric | 0.297 | Sample spacing and aggressive preview produced a spin/contact; rejected. |
| Conservative controller | 1.800 km, 160 segments, distance preview and race ESC preset | 102.8 s | 108.9 km/h | 3.86 m | 0.000 | Safe but visually coarse and lacked a convincing high-speed section. |
| Final | 2.063 km, 240 segments, 26.41 m minimum radius, braking-envelope AI | 102.2 s | 136.0 km/h | 2.41 m | 0.000 | Accepted analog reference lap. |
| Final digital | Same core geometry; binary direction through keyboard adapter | 102.3 s | 136.1 km/h | 2.46 m | 0.000 | Accepted digital-controller lap. |

## Speed-specific steering response

The permanent `steering_validation` probe settles a neutral-driveline vehicle, assigns wheel speed consistently, disables race ESC for the fixture and holds each command for one physics second.

| Entry speed | Half-command heading | Full-command heading | Full-command peak front slip | Final normalized full output |
|---:|---:|---:|---:|---:|
| 50 km/h | 20.08° | 38.35° | 12.83° | 0.2999 |
| 100 km/h | 9.77° | 19.11° | 8.77° | 0.0671 |
| 140 km/h | 6.64° | 13.02° | 6.90° | 0.0337 |

The physical maximum remains 0.54 rad (30.9°). Sport Adaptive is the normal browser profile. Accessible targets 7.5 m/s² and enables ESC; Sport targets 10.0 m/s²; Simulation exposes Digital Raw/Test keyboard and normalized raw gamepad input. Keyboard/gamepad gain and slew execute at the configured 1 ms physics timestep, so render cadence cannot alter response. Wheel input remains calibrated linear 1:1 with no speed assistance. None of these profiles changes the physical plant.

Run the evidence with:

```bash
cargo test --all-targets
cargo run --release --example circuit_lap
cargo run --release --example steering_validation
./scripts/build-wasm.sh
node scripts/benchmark-wasm.mjs
```
