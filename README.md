# My Physics

My Physics is a deterministic, headless-first vehicle and motorsport physics engine written in Rust. The implemented v0.1 technical release runs the same physical model natively and in a Chromium browser through WebAssembly.

> This is an early validation platform, not yet a certified engineering tool. The model boundaries are designed for higher fidelity, while several v0.1 implementations are deliberately documented approximations.

## What works today

- Fixed 1000 Hz reference stepping and bounded variable-timestep mode
- 6-DoF chassis state with quaternion orientation, mutable mass, CG, inertia tensor and physical damage
- Replaceable tire-model trait with a Magic Formula-family implementation
- Combined slip, finite lateral-force peak/sliding branch, distance-based slip relaxation, two-node thermal energy state, wear, pressure, contact-patch state, wet grip and hydroplaning
- Progressive puncture, blowout, bead unseating and damaged-carcass behavior
- Four-wheel spring/damper suspension, travel, bump stops and anti-roll coupling
- RWD ICE, torque curve, RPM inertia, fuel mass, clutch, manual/automatic-compatible gear requests and open differential
- Engine/coolant/oil temperatures, oil pressure and duration-dependent overheat/low-oil/over-rev damage
- Thermal brakes and plant-independent ABS, traction control and stability control
- Drag, downforce and uniform wind
- Spatial road temperature, rubber, contamination and water state with tire interaction
- Oriented box, capsule and convex narrow phases for vehicles, curbs and static environments
- Dynamics-affecting deformation, wheel/suspension/aero damage and independent detached bodies
- Ten-vehicle 2.06 km physically elevated and banked racing circuit with a numerically guarded 26.4 m minimum centerline radius, synchronized road contact/barriers, deterministic AI line following and smoothly changing physics LOD
- Complete versioned/checksummed snapshot archives, persistent timed input history, restore and deterministic re-simulation
- Detailed telemetry, expanded headless CSV, continuous Audio/FFB frames and discrete physical events
- Raw WebAssembly API and a complete three-lap WebGL2 race demo with countdown, ten-car ranking, results/restart, telemetry VFX/audio and chase, hood and cockpit cameras
- Explicit engineering-reference, race-gameplay and arcade-fun vehicle-data presets with parameter-group provenance and validity metadata

## Run it

Requirements: stable Rust with `wasm32-unknown-unknown`, Python 3, and a Chromium-based browser.

```bash
cargo test --all-targets
./scripts/build-wasm.sh
./scripts/serve-web.sh
```

Open <http://localhost:8080> for the [three-lap Browser Race](docs/race-demo.md), <http://localhost:8080/?demo=arcade> for Arcade Fun, or <http://localhost:8080/?demo=simulation-lab> for the reproducible [Simulation Validation Lab](docs/simulation-lab.md). All three use the same Rust/WASM plant; Arcade selects an authored `VehicleDefinition` and a separate 1.22 g controller policy, while the Lab selects the EngineeringReference definition and Simulation Raw input. Sport is the normal profile on the existing URL; Accessible lowers the target and enables ESC, while Simulation exposes normalized raw gamepad and Digital Raw/Test keyboard commands. Arcade has its own stored profile, but shares wheel/pedal calibration with Simulation. Select a profile in the circuit UI or use `?driveProfile=accessible|sport|simulation|arcade`; I switches between the current demo's assisted profile and Simulation. Keyboard and gamepad policy is stepped at 1,000 Hz and never changes physical rack travel, tires, mass or timestep. Calibrated wheels remain linear 1:1 with no speed assist in every profile. Use WASD/arrows to drive, Space for the handbrake, C for chase/hood/cockpit camera, M for telemetry audio, P for AI, E for ESC, Shift for clutch, T for automatic, R/Enter/Start to restart, 1–6 for gears, and K/L for paired physics/race snapshots.

The Device Setup button captures only the current steering center and released pedal positions for the matching controller id. Full endpoints and axis mapping can be authored through URL parameters, for example `?steerAxis=0&throttleAxis=1&brakeAxis=2&clutchAxis=3&steerMin=-1&steerMax=1&inputDeadzone=.08&inputOuterDeadzone=.04&inputExpo=1.55`. UI choices and captured rest calibration persist locally; URL values take precedence. An idle connected controller cannot steal input from the active device.

For a rendering-free simulation and CSV telemetry:

```bash
cargo run --release --bin my-physics-headless -- 10 > telemetry.csv
cargo run --release --example circuit_lap
cargo run --release --example steering_validation
./scripts/run-maneuver-validation.sh target/maneuver-validation
```

Licensed external telemetry can be compared through the dataset-independent
[real-world correlation framework](docs/real-world-correlation.md). Its strict
manifest, split, alignment and provenance rules do not alter the physical
plant, and no third-party raw dataset is included in this repository. The
generic command compares hash-verified time series; the IO-VNBD-specific runner
separately proves deterministic 1 ms `PhysicsWorld` execution and emits visual
measured-versus-simulated evidence.

## Architecture

The physical plant is reusable and has no renderer, DOM, browser or game-engine dependency. `PhysicsWorld` owns deterministic timing, road state, collisions, LOD, snapshots and vehicles. Each `Vehicle` composes chassis, wheels, tires, suspension, powertrain, damage and telemetry. Driver aids consume sensor values and return control commands through a separate module.

All internal quantities use SI units and radians. Coordinates are right-handed and Three.js-compatible: +X right, +Y up and -Z vehicle-forward. See the [v0.1 acceptance matrix](docs/v0.1-acceptance.md), [Architecture](docs/architecture.md), [Vehicle data and provenance](docs/vehicle-data.md), [Validation](docs/validation.md), [Simulation Lab](docs/simulation-lab.md), [Real-world correlation](docs/real-world-correlation.md), [Performance](docs/performance.md), and the [Roadmap](docs/roadmap.md).

## Fidelity and honest limitations

The tire implementation is Magic Formula-family with first-order slip relaxation, not a fitted proprietary Pacejka parameter set or a transient brush model. Its effective thermal constants are authored rather than measured. See [Reference tire](docs/tire-model.md). Suspension uses vertical spring/damper rays rather than hard-point kinematics. Deformation modifies a parameterized collision envelope rather than a finite-element mesh. The collision and deformation models are suitable for vehicle behavior and regression work, not crash analysis. Cross-CPU bitwise determinism, advanced aero maps, transient brush tires, detailed debris interactions and production multiplayer correction are roadmap items.

WebGPU is intentionally not used in v0.1: ten vehicles at this fidelity fit the WASM/CPU path, and GPU parallel reduction would complicate deterministic execution without a measured benefit. The architecture reserves accelerators for explicitly non-authoritative or deterministically validated workloads.

## Project status and license

The specification intentionally leaves the permissive license choice (MIT versus Apache-2.0 or another option) open. No license grant is implied until that decision is made; see [LICENSE-DECISION.md](LICENSE-DECISION.md). Choose and add an OSI-approved permissive license before calling a release open source.
