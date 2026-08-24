# My Physics

My Physics is a deterministic, headless-first vehicle and motorsport physics engine written in Rust. The v0.1 technical prototype runs the same physical model natively and in a Chromium browser through WebAssembly.

> This is an early validation platform, not yet a certified engineering tool. The model boundaries are designed for higher fidelity, while several v0.1 implementations are deliberately documented approximations.

## What works today

- Fixed 1000 Hz reference stepping and bounded variable-timestep mode
- 6-DoF chassis state with quaternion orientation, mutable mass, CG, inertia-related state and physical damage
- Replaceable tire-model trait with a Magic Formula-family implementation
- Combined slip, load/camber sensitivity, thermal state, wear, pressure, contact-patch state, wet grip and hydroplaning
- Progressive puncture, blowout, bead unseating and damaged-carcass behavior
- Four-wheel spring/damper suspension, travel, bump stops and anti-roll coupling
- RWD ICE, torque curve, RPM inertia, fuel mass, clutch, manual/automatic-compatible gear requests and open differential
- Engine/coolant/oil temperatures, oil pressure and duration-dependent overheat/low-oil/over-rev damage
- Thermal brakes and plant-independent ABS, traction control and stability control
- Drag, downforce and uniform wind
- Spatial road temperature, rubber, contamination and water state with tire interaction
- Box, capsule and convex collider data; deterministic v0.1 vehicle/static collision approximation
- Dynamics-affecting deformation, wheel/suspension/aero damage and independent detached bodies
- Ten-vehicle demo with smoothly changing physics LOD
- Complete in-memory snapshots, timed input history, restore and deterministic re-simulation
- Detailed telemetry, CSV-capable headless runner and audio/FFB-ready state access
- Raw WebAssembly API and rendering-independent WebGL2 3D chase-camera demo

## Run it

Requirements: stable Rust with `wasm32-unknown-unknown`, Python 3, and a Chromium-based browser.

```bash
cargo test --all-targets
./scripts/build-wasm.sh
./scripts/serve-web.sh
```

Open <http://localhost:8080>. Drive with WASD or arrow keys, use Space for the handbrake, R to reset, and 1–6 to request a gear. A gamepad is detected through the browser Gamepad API.

For a rendering-free simulation and CSV telemetry:

```bash
cargo run --release --bin my-physics-headless -- 10 > telemetry.csv
```

## Architecture

The physical plant is reusable and has no renderer, DOM, browser or game-engine dependency. `PhysicsWorld` owns deterministic timing, road state, collisions, LOD, snapshots and vehicles. Each `Vehicle` composes chassis, wheels, tires, suspension, powertrain, damage and telemetry. Driver aids consume sensor values and return control commands through a separate module.

All internal quantities use SI units and radians. Coordinates are right-handed and Three.js-compatible: +X right, +Y up and -Z vehicle-forward. See [Architecture](docs/architecture.md), [Validation](docs/validation.md), and the [Roadmap](docs/roadmap.md).

## Fidelity and honest limitations

The tire implementation is Magic Formula-family, not a fitted proprietary Pacejka parameter set. Suspension uses vertical spring/damper rays rather than hard-point kinematics. Convex shapes use conservative bounds in the current narrow phase. The collision and deformation models are suitable for prototype behavior and regression work, not crash analysis. Cross-CPU bitwise determinism, stable binary snapshot serialization, advanced aero maps, transient brush tires, detailed debris interactions and production multiplayer correction are roadmap items.

WebGPU is intentionally not used in v0.1: ten vehicles at this fidelity fit the WASM/CPU path, and GPU parallel reduction would complicate deterministic execution without a measured benefit. The architecture reserves accelerators for explicitly non-authoritative or deterministically validated workloads.

## Project status and license

The specification intentionally leaves the permissive license choice (MIT versus Apache-2.0 or another option) open. No license grant is implied until that decision is made; see [LICENSE-DECISION.md](LICENSE-DECISION.md). Choose and add an OSI-approved permissive license before calling a release open source.
