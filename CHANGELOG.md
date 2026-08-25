# Changelog

- Split the engineering-reference and race-gameplay vehicle presets without changing the physical plant, and added versioned parameter provenance metadata.

## 0.1.0 — implementation complete, release administration pending

- Added deterministic fixed/variable timing, 6-DoF chassis and smooth multi-vehicle LOD.
- Added stateful Magic Formula-family tires, simplified suspension, RWD ICE powertrain, thermal systems, aids, aero and dynamic road state.
- Added oriented box/capsule/convex collision, dynamics-affecting deformation, component failures and detached bodies.
- Added versioned snapshot/input archives, deterministic replay, interpolation, telemetry and Audio/FFB contracts.
- Added headless CSV, raw WASM API, WebGL2 ten-vehicle demo and automatic/manual device quality selection.
- Added the 42-test v0.1 validation suite, golden regression and native/WASM/Chromium performance evidence.
- Fixed automatic-transmission chain shifting and over-rev failure by adding shift torque interruption, smooth clutch disengagement and a deterministic rev limiter.
- Improved high-speed visual perception with bounded chase-camera lag, speed-sensitive field of view and denser world-space road references without changing physical motion.
- Replaced the motorway-like scene with a narrower procedural race straight featuring synchronized long-distance barriers, red/white curbs, catch fencing, rubbered asphalt, braking boards, starting grids, light gantries and grandstands.
- Corrected friction-limited ESC yaw targeting and per-corner brake selection so full keyboard steering remains effective; active A/D input now also takes priority over a connected wheel axis.
- Replaced the straight demo with a 0.72 km closed F1-style circuit shared by Rust collisions, vehicle grids, AI controllers and WebGL rendering; added lap progress, a P-key AI-driver toggle and a deterministic clean-lap validation example.
- Rescaled the compact prototype into a 2.06 km/240-segment circuit with a tested 26.4 m minimum radius and a 136 km/h damage-free reference lap; the same geometry still drives WebGL, barriers, AI and lap progress.
- Added physics-clock, speed-sensitive keyboard steering with controlled slew while preserving raw gamepad/wheel axes; the browser race preset starts with ESC off and exposes an E-key toggle.
- Made simulator-strict raw keyboard steering the browser default; A/D now requests the full physical rack angle immediately, while the speed-sensitive assist is an explicit I-key opt-in mode.
- Calibrated the Magic Formula-family dry longitudinal curve to peak at 15% slip with realistic locked-wheel sliding force, and replaced binary ABS release with persistent, snapshot-serialized pressure modulation.
- Corrected flat-road contact forces to use the road-plane basis and added an explicitly authored rear cornering-stiffness balance, preventing friction-feasible high-g ramps from developing a non-physical roll/contact feedback spin.
- Advanced snapshots to archive version 2 while retaining version-1 decoding with documented defaults for the new controller and wheel-fit state.
- Reworked WebGL speed perception without changing physics scale: curbs, fence posts, barrier joints, asphalt patches and rubber detail now follow cumulative circuit distance, while closer chase and hood camera presets are switchable with C or the UI.

The source implementation is complete for the documented v0.1 acceptance matrix. A public release tag and open-source license grant remain pending the owner’s explicitly deferred license decision.
