# Arcade keyboard drift experiment

Open <http://localhost:8080/?demo=arcade&playground=drift> after building and
serving the WASM demo. The page starts the unchanged `ArcadeFun` vehicle at
75 km/h on a repeatable, collision-free short test lane. Press A or D to state
turn intent, use Space to apply the physical handbrake, keep or release W as
desired, and use R/Enter to restore the exact entry state.

This is an input-controller experiment, not a separate drift physics model.
The proving-ground world uses the existing `GroundSurface::Flat`, the normal
1 ms fixed step, one high-fidelity vehicle and the common tire/chassis plant.
It removes circuit walls and other cars so a barrier impact cannot be confused
with controller behavior. The normal `?demo=arcade` circuit and its ten-car
composition are unchanged.

## Responsibility boundary

`ArcadeKeyboardDriftAssist` runs in the keyboard policy layer. Steering,
braking, throttle lift or handbrake must first produce at least 6 degrees of
physical body slip with corresponding yaw. Only then can the controller blend
a continuous steering command that a digital key cannot express. It tracks
Grip, Entry, Slide, Recovery and Spin phases.

The controller:

- changes steering policy output only;
- copies throttle, service brake, clutch, handbrake and gear requests exactly;
- never writes chassis state, yaw torque, tire force or grip;
- does not enter Slide from steering alone;
- does not guarantee recovery from excessive entry angle or bad timing;
- is active only for assisted keyboard input in the Arcade experience;
- is included in the paired browser physics/controller snapshot.

Gamepad, calibrated wheel and Simulation Raw paths remain outside this
controller. `ArcadeFun` vehicle, tire, suspension, mass, aero and powertrain
parameters were not retuned for this experiment. The isolated Playground world
starts with TC disabled as an ordinary authored driver-aid selection so it does
not cancel a player-requested physical handbrake slide; changing profiles does
not rewrite TC and this selection does not alter the plant.

## Observable evidence

The Canvas overlay shows raw key intent, assisted steering, physical road-wheel
angle, assist correction, body sideslip beta, yaw rate, rear longitudinal slip
and rear slip angle. The same values are available through WASM diagnostics and
`window.__MY_PHYSICS_INPUT__.arcadeDrift`.

The presentation classifier keeps the physical controller phases separate from
player-facing outcomes. It distinguishes normal Grip, front-slip-dominant
Understeer, controlled Slide, successful Recovery, a low-speed/high-angle Poor
Exit and Spin. The classifier is display-only and has unit fixtures for all six
outcomes; it never feeds a correction back into simulation.

The deterministic native fixture compares steering-only, controlled 0.8 s
handbrake, left/right symmetry and excessive 1.3 s handbrake cases. The
controlled reference currently produces about 19.2 degrees peak body slip,
about 0.78 s of readable slide, about 0.42 s of countersteer and recovery in
about 0.62 s. The excessive case reaches about 90 degrees and is not recovered.
These values are regression evidence for the authored scenario, not a claim
that one timing is optimal for human play.

The Chromium smoke test sends real browser key events and verifies Entry, Slide,
Recovery, readable physical sideslip, countersteer, zero damage, exact
non-steering passthrough, finite wheel telemetry, a full-window Canvas and
visible Overlay telemetry. Precise dynamic envelopes remain in the fixed-step
Rust test because wall-clock browser event duration varies with render load.

Run the deterministic checks with:

```bash
cargo test --test arcade_keyboard_drift
node scripts/test-wasm-controller-snapshot.mjs
```

For the Chromium check, start Chrome with a remote-debugging port and run:

```bash
CHROME_DEBUG_PORT=9229 \
DEMO_URL='http://127.0.0.1:8080/?demo=arcade&playground=drift' \
node scripts/test-chromium-drift.mjs
```

## Limitations and next feedback

Automated checks establish control authority, physical causation, recovery and
failure space; they cannot establish that the interaction is fun. Human
playtests should report entry timing, desired angle, perceived correction,
line choice and exit quality together with the visible telemetry. The next
iteration should tune only the Arcade keyboard controller first. Vehicle-data
tuning or presentation changes should be evaluated separately so controller,
plant and presentation effects remain attributable.
