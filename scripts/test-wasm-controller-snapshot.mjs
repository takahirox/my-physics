import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

const repository = new URL('../', import.meta.url);
const binary = await readFile(new URL('web/physics.wasm', repository));
const { instance } = await WebAssembly.instantiate(binary, {});
const physics = instance.exports;

// The game-facing keyboard adapter is adaptive by default. Digital Raw/Test
// remains explicit and is immediate after its bumpless mode transition settles.
physics.physics_reset();
assert.equal(physics.physics_keyboard_assist(), 1);
physics.physics_set_keyboard_input(1, 0, 0, 0, 0, 0);
physics.physics_step(1);
assert(physics.physics_steering(0) > 0 && physics.physics_steering(0) < 0.01);
physics.physics_set_keyboard_assist(0);
physics.physics_step(1);
assert(Math.abs(physics.physics_steering(0) - 0.0028) < 0.004, 'mode change must not jump the rack');
physics.physics_step(400);
assert.equal(physics.physics_steering(0), 1);
physics.physics_set_keyboard_input(-1, 0, 0, 0, 0, 0);
physics.physics_step(1);
assert.equal(physics.physics_steering(0), -1, 'settled Digital Raw/Test must reach full rack on the next step');

// Switching from keyboard to a normalized gamepad command is also bumpless.
const beforeDeviceSwitch = physics.physics_steering(0);
physics.physics_set_device_input(2, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
physics.physics_step(1);
assert(Math.abs(physics.physics_steering(0) - beforeDeviceSwitch) <= 0.004, 'device switch must slew from the applied rack');
assert.equal(physics.physics_input_device(), 2);
assert.equal(physics.physics_input_stage_steering(0), -1);
assert.equal(physics.physics_input_stage_steering(1), -1);
physics.physics_step(600);
physics.physics_set_device_input(2, 0.4, 0, 0, 0, 0, 0.4, 0, 0, 0, 0, 0);
physics.physics_step(1);
assert.equal(physics.physics_steering(0), 0.4, 'settled gamepad policy must preserve analog authority');
const beforeWheelSwitch = physics.physics_steering(0);
physics.physics_set_device_input(3, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0);
physics.physics_step(1);
assert(Math.abs(physics.physics_steering(0) - beforeWheelSwitch) <= 0.004, 'wheel switch must be bumpless');
physics.physics_step(300);
assert.equal(physics.physics_input_stage_steering(1), 1);
assert.equal(physics.physics_input_stage_steering(2), 1, 'wheel must remain calibrated 1:1 without speed assist');
assert.equal(physics.physics_input_transitioning(), 0);
physics.physics_set_keyboard_assist(1);
assert.equal(physics.physics_input_transitioning(), 0, 'keyboard policy changes must not alter an active wheel');
physics.physics_set_keyboard_assist(0);
assert.equal(physics.physics_input_transitioning(), 0);
const beforeKeyboardSwitch = physics.physics_steering(0);
physics.physics_set_keyboard_input(-1, 0, 0, 0, 0, 0);
physics.physics_step(1);
assert(Math.abs(physics.physics_steering(0) - beforeKeyboardSwitch) <= 0.004, 'return to keyboard must be bumpless');

function publicState() {
  const values = [physics.physics_time(), physics.physics_player_autopilot(), physics.physics_player_esc()];
  for (let vehicle = 0; vehicle < physics.physics_vehicle_count(); vehicle += 1) {
    values.push(
      physics.physics_x(vehicle),
      physics.physics_y(vehicle),
      physics.physics_z(vehicle),
      physics.physics_yaw(vehicle),
      physics.physics_speed(vehicle),
      physics.physics_rpm(vehicle),
      physics.physics_gear(vehicle),
      physics.physics_steering(vehicle),
      physics.physics_damage(vehicle),
    );
    for (let wheel = 0; wheel < 4; wheel += 1) {
      values.push(physics.physics_tire_temp(vehicle, wheel), physics.physics_tire_pressure(vehicle, wheel));
    }
  }
  return values;
}

function pipelineState() {
  const stages = [];
  for (let stage = 0; stage < 5; stage += 1) {
    stages.push(
      physics.physics_input_stage_steering(stage),
      physics.physics_input_stage_throttle(stage),
      physics.physics_input_stage_brake(stage),
      physics.physics_input_stage_clutch(stage),
      physics.physics_input_stage_handbrake(stage),
      physics.physics_input_stage_gear(stage),
    );
  }
  for (let wheel = 0; wheel < 4; wheel += 1) {
    stages.push(physics.physics_input_aid_brake(wheel), physics.physics_input_abs_active(wheel));
  }
  stages.push(physics.physics_input_tc_active(), physics.physics_input_esc_active());
  return [
    physics.physics_step_index(),
    physics.physics_input_sample_sequence(),
    physics.physics_input_applied_step(),
    physics.physics_input_device(),
    physics.physics_input_transitioning(),
    ...stages,
  ];
}

physics.physics_reset();
physics.physics_set_keyboard_assist(1);
physics.physics_set_keyboard_input(1, 0.65, 0, 0, 0, 0);
physics.physics_step(73);
const savedBytes = physics.physics_snapshot_save();
assert(savedBytes > 0);
const savedPipeline = pipelineState();

physics.physics_step(480);
const expected = publicState();

// Mutate every controller item that is external to PhysicsWorld.
physics.physics_set_keyboard_input(-1, 0, 0.4, 0, 0, 0);
physics.physics_set_input(-0.75, 0, 0, 0, 0, 0);
physics.physics_set_player_autopilot(1);
physics.physics_set_keyboard_assist(0);
physics.physics_step(31);

assert.equal(physics.physics_snapshot_restore(), 1);
assert.equal(physics.physics_player_autopilot(), 0);
assert.equal(physics.physics_keyboard_assist(), 1);
assert.deepEqual(pipelineState(), savedPipeline, 'snapshot must restore controller and all input-pipeline stages');
physics.physics_step(480);
assert.deepEqual(publicState(), expected, 'restored keyboard branch must reproduce every public physical value');

// Autopilot mode is also part of the browser execution state.
physics.physics_set_player_autopilot(1);
physics.physics_step(10);
physics.physics_snapshot_save();
physics.physics_set_player_autopilot(0);
assert.equal(physics.physics_snapshot_restore(), 1);
assert.equal(physics.physics_player_autopilot(), 1);

function scheduleRun(refreshHz) {
  physics.physics_reset();
  physics.physics_set_keyboard_assist(1);
  const events = [
    [0, 1, 0.8, 0],
    [220, -1, 0.8, 0],
    [470, 0, 0, 1],
    [640, 0.5, 0.55, 0],
    [820, 0, 0, 0],
  ];
  let eventIndex = 0;
  let step = 0;
  let frame = 1;
  while (step < 1000) {
    const frameEnd = Math.min(1000, Math.floor((frame * 1000) / refreshHz + 1e-9));
    while (eventIndex < events.length && events[eventIndex][0] <= frameEnd) {
      const [eventStep, steering, throttle, brake] = events[eventIndex];
      if (eventStep > step) physics.physics_step(eventStep - step);
      step = eventStep;
      physics.physics_set_keyboard_input(steering, throttle, brake, 0, 0, 0);
      eventIndex += 1;
    }
    if (frameEnd > step) physics.physics_step(frameEnd - step);
    step = frameEnd;
    frame += 1;
  }
  return [...publicState(), ...pipelineState()];
}

const scheduleReference = scheduleRun(30);
for (const refreshHz of [60, 120, 144]) {
  assert.deepEqual(scheduleRun(refreshHz), scheduleReference, `${refreshHz} Hz grouping changed deterministic policy output`);
}

console.log(JSON.stringify({
  savedBytes,
  comparedValues: expected.length,
  pipelineValues: savedPipeline.length,
  renderSchedulesHz: [30, 60, 120, 144],
  autopilotRestored: true,
}, null, 2));
