# Browser race demo

The default browser entry point is a complete three-lap race built on the same
Rust `PhysicsWorld`, `VehicleDefinition`, tire model, collision system and
1,000 Hz fixed timestep used by the headless and engineering workflows. The
Arcade URL changes declared vehicle data and controller policy; it does not use
a second plant or apply hidden grip, yaw, velocity or force corrections.

## Race flow

`web/race-state.mjs` is a deterministic application-layer race director. It
consumes authoritative physics time and each vehicle's projected circuit
progress. A race moves through countdown, racing and finished phases, records
three timed laps, ranks all ten cars and produces a restartable result table.
Sequential quarter-lap checkpoints reject reverse crossings, discontinuous
nearest-segment jumps and shortcuts. The browser snapshot buttons pair the
Rust snapshot with the race-director snapshot.

During the countdown, the WASM controller applies the normal service brakes to
all cars while `PhysicsWorld` continues to advance. This is an ordinary driver
input gate, not a frozen or repositioned rigid body. At GO, AI and human inputs
again pass through the same `DriverInput` and plant path.

## Physical 3D circuit

The circuit's elevation and banking are authored in `src/circuit.rs`. One
deterministic sampled frame supplies:

- suspension ray/contact height and road normal;
- tire longitudinal/lateral tangent directions;
- chassis start poses and oriented collision barriers;
- detached-body and chassis-floor contact;
- the exact center, forward, right and up vectors exported to the renderer.

Flat proving grounds and real-world correlation explicitly retain
`GroundSurface::Flat`. Snapshot archive v5 stores the ground-surface choice;
older snapshots migrate to Flat rather than silently acquiring the demo road.

## Presentation boundary

The WebGL2 renderer uses the exported physical frame for the road, barriers and
full vehicle quaternion. Procedural low-poly cars, curbs, grandstands, trees,
summit tower, bridge and canyon keep the demo self-contained and asset-license
free. Camera motion, tire smoke, wet spray, sparks, speed streaks and WebAudio
consume read-only telemetry. Their mappings are finite, bounded and covered by
Node tests, and no presentation output is sent back to physics.

Keyboard, gamepad and steering-wheel paths remain distinct. WASD/arrows,
controller triggers/sticks and calibrated wheel/pedal axes all become normal
driver inputs. C cycles chase, hood and cockpit cameras; M toggles telemetry
audio; R, Enter or gamepad Start restarts after the finish.

## Verification

Run the complete local gate:

```bash
./scripts/verify-v01.sh
```

The gate covers Rust unit/integration tests, maneuver validation, web module
tests, release WASM construction, snapshot/controller checks and the ten-car
real-time benchmark. A Chromium smoke test additionally checks that the page
loads without exceptions, holds the grid through the countdown, enters the
racing phase, renders the physical elevation and advances the shared plant.

