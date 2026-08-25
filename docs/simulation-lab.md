# Simulation Validation Lab

Open `?demo=simulation-lab` to use the browser laboratory. It is intentionally
separate from the default circuit and `?demo=arcade` experiences.

The live proving ground uses one `EngineeringReference` vehicle, fixed 1 ms
steps, full fidelity, no automatic Physics LOD, no static collision barriers,
and the Simulation Raw controller profile. It uses the same Rust integration,
tire, suspension, powertrain, controls, road, damage, and serialization code as
the other demos. The demo selector only composes the initial world; there is no
Simulation-Lab branch in the physical plant.

## Official catalog

The UI calls the existing Rust `validation::SCENARIOS` catalog directly rather
than duplicating its fixtures or bounds in JavaScript:

- neutral coast-down from 100 km/h;
- 0–100 km/h full-throttle acceleration;
- 100–0 km/h ABS braking;
- steady low-g steer;
- one-degree step steer;
- 0.5 Hz slalom.

Each report exposes the complete sampled speed, yaw-rate, body-sideslip,
acceleration, four-wheel slip, slip-angle, and normal-load series. Every
acceptance envelope displays its actual value and min/max bound. `RUN ALL FAST`
runs all six catalog entries in WASM. `REPEATABILITY VERIFY` compares two
independent deterministic runs. `MIDPOINT SNAPSHOT REPLAY` saves halfway,
restores, re-applies the timed input program, and compares the final state
fingerprint.

The default/circuit profile remains in the shared input configuration. The Lab
always selects Simulation Raw for its own session and cannot overwrite the
default or Arcade profile selection. Device calibration remains shared.

## Interpretation and limits

These results are physical-plausibility and regression evidence only. Current
EngineeringReference values are authored or estimated. They are not a measured
fit, correlation with a measured real vehicle, or certification. Dynamic dry
versus wet comparison is planned for Lab v2; v0.1 already validates road/tire
wet behavior in the core test suite but does not present it as a Lab envelope.
