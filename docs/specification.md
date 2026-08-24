# Product specification baseline

Status: implementation-planning draft<br>
Baseline date: 2026-08-24<br>
Primary language/runtime: Rust, WebAssembly, selective WebGPU<br>
Primary v0.1 target: desktop Chromium

The product is one high-fidelity physical foundation shared by racing games, motorsport/vehicle engineering, automated tests, replay/multiplayer and future DIL/HIL workflows. Fidelity profiles may change numerical/model detail, but not the meaning of vehicle parameters.

The authoritative internal convention is SI units, radians, quaternions and Three.js-compatible right-handed axes (+X right, +Y up, -Z forward), with the vehicle-local origin at CG by default. Fixed and variable timesteps are supported; the high-fidelity fixed reference is 1000 Hz. Rendering is always independent and headless operation is mandatory.

The v0.1 acceptance target is a controllable RWD ICE car plus nine LOD vehicles in Chromium: stateful combined-slip tires, pressure/failure/thermal/wear behavior, simplified suspension, fuel and thermal powertrain, clutch/transmission, thermal brakes, ABS/TC/ESC, wind/aero, spatial rubber/water/temperature, hydroplaning, collisions, physical damage, detached components, snapshots, deterministic replay, telemetry and automated validation.

v1.0 expands the same architecture to stable multi-platform APIs, documented deterministic profiles, approximately 100 LOD vehicles, multiplayer correction primitives, replaceable physical models, hard-point suspension, flexible chassis, additional powertrains/layouts, advanced aero/environment/damage, provenance-aware data, telemetry correlation/parameter fitting, AI driver roles and DIL/HIL-ready adapters.

The full implementation brief deliberately defers the license choice, detailed LSD/e-diff, automatic setup optimization, injury analysis, exact FFB bandwidth, non-player frequencies, multiplayer authority model, exact cross-platform floating-point policy, WebGPU partition, water drainage, v0.1 spatial wind, fuel slosh, detailed airflow cooling, poor-shift damage and full debris secondary interactions. This repository does not silently select those policies.
