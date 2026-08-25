import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

const repository = new URL('../', import.meta.url);
const binary = await readFile(new URL('web/physics.wasm', repository));
const { instance } = await WebAssembly.instantiate(binary, {});
const physics = instance.exports;

// The game-facing keyboard adapter is adaptive by default. Digital Raw/Test
// remains explicit and is immediate after its bumpless mode transition settles.
physics.physics_reset();
assert.equal(physics.physics_demo_vehicle_preset(), 1, 'existing URL/default remains Race Gameplay');
assert.equal(physics.physics_keyboard_assist(), 1);
assert.equal(physics.physics_experience_profile(), 1, 'Sport is the normal game default');
assert.equal(physics.physics_policy_lateral_accel_target(), 10);
assert.equal(physics.physics_gamepad_assist(), 1);
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
assert(Math.abs(physics.physics_steering(0) + 1) <= 0.004, 'new gamepad samples slew on physics ticks');
physics.physics_step(410);
assert(Math.abs(physics.physics_steering(0) - 0.4) < 1e-12, 'settled gamepad policy preserves analog authority at walking speed');
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

// Profiles select controller/aid behavior, never plant parameters. At speed,
// gamepad commands are bounded by the profile target; Simulation is explicit
// normalized raw, while a calibrated wheel is always linear 1:1.
physics.physics_reset();
physics.physics_set_device_input(2, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0);
physics.physics_step(8_000);
const policySpeedMps = physics.physics_speed(0);
assert(policySpeedMps > 25, `test vehicle did not reach policy speed: ${policySpeedMps}`);
physics.physics_set_device_input(2, 0.5, 1, 0, 0, 0, 0.5, 1, 0, 0, 0, 0);
physics.physics_step(500);
const sportHalf = physics.physics_input_stage_steering(2);
assert(sportHalf > 0 && sportHalf < 0.08, `Sport half-pad was not speed bounded: ${sportHalf}`);
const sportRequestedMps2 = policySpeedMps ** 2 * Math.tan(sportHalf * 0.54) / 2.51;
assert(sportRequestedMps2 <= 5.5, `Sport half-pad exceeded half-target envelope: ${sportRequestedMps2}`);
const beforeProfileSwitch = physics.physics_steering(0);
physics.physics_set_experience_profile(2);
physics.physics_step(1);
assert(Math.abs(physics.physics_steering(0) - beforeProfileSwitch) <= 0.004, 'profile change must be bumpless');
physics.physics_step(500);
assert.equal(physics.physics_input_stage_steering(2), 0.5, 'Simulation gamepad must be normalized raw');
const beforeWheel = physics.physics_steering(0);
physics.physics_set_device_input(3, 0.5, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0);
physics.physics_step(1);
assert(Math.abs(physics.physics_steering(0) - beforeWheel) <= 0.004, 'wheel activation must be bumpless');
physics.physics_step(500);
assert.equal(physics.physics_input_stage_steering(2), 0.5, 'wheel must remain 1:1 in every profile');
for (const profile of [0, 1, 2, 3]) {
  physics.physics_reset();
  physics.physics_set_experience_profile(profile);
  physics.physics_set_device_input(3, 0.5, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0);
  physics.physics_step(500);
  assert.equal(physics.physics_input_device(), 3);
  assert.equal(physics.physics_input_stage_steering(2), 0.5, `wheel was not 1:1 in profile ${profile}`);
}

function sampleGamepadPolicy(profile, targetKmh, normalizedSteering) {
  physics.physics_reset();
  physics.physics_set_experience_profile(profile);
  physics.physics_set_device_input(2, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0);
  let batches = 0;
  while (physics.physics_speed(0) * 3.6 < targetKmh && batches < 200) {
    physics.physics_step(100);
    batches += 1;
  }
  assert(batches < 200, `could not reach ${targetKmh} km/h`);
  physics.physics_set_device_input(
    2, normalizedSteering, 0, 0, 0, 0, normalizedSteering, 0, 0, 0, 0, 0,
  );
  physics.physics_step(220);
  return {
    profile,
    targetKmh,
    input: normalizedSteering,
    actualKmh: physics.physics_speed(0) * 3.6,
    policy: physics.physics_input_stage_steering(2),
  };
}

const policySamples = [];
for (const profile of [0, 1, 2, 3]) {
  for (const targetKmh of [50, 100, 140]) {
    const quarter = sampleGamepadPolicy(profile, targetKmh, 0.25);
    const half = sampleGamepadPolicy(profile, targetKmh, 0.5);
    assert(half.policy >= quarter.policy, `profile ${profile} at ${targetKmh} km/h was not monotonic`);
    if (profile === 2) {
      assert(Math.abs(quarter.policy - 0.25) < 1e-12 && Math.abs(half.policy - 0.5) < 1e-12, 'Simulation must be raw');
    } else {
      const targetMps2 = profile === 0 ? 7.5 : profile === 3 ? 12 : 10;
      for (const sample of [quarter, half]) {
        const speedMps = sample.actualKmh / 3.6;
        const limit = Math.max(0.02, Math.min(1, Math.atan(2.51 * targetMps2 / speedMps ** 2) / 0.54));
        const expected = sample.input * limit;
        assert(
          Math.abs(sample.policy - expected) <= Math.max(1e-9, expected * 0.03),
          `profile ${profile} ${targetKmh} km/h expected ${expected}, got ${sample.policy}`,
        );
      }
    }
    policySamples.push(quarter, half);
  }
}

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
physics.physics_set_experience_profile(0);
assert.equal(physics.physics_player_esc(), 1, 'Accessible enables ESC');
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
physics.physics_set_experience_profile(2);
physics.physics_set_player_esc(0);
physics.physics_step(31);

assert.equal(physics.physics_snapshot_restore(), 1);
assert.equal(physics.physics_player_autopilot(), 0);
assert.equal(physics.physics_keyboard_assist(), 1);
assert.equal(physics.physics_experience_profile(), 0);
assert.equal(physics.physics_player_esc(), 1, 'snapshot restores profile aid state');
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

// Demo definition selection survives reset and is browser-snapshot state.
physics.physics_select_demo_vehicle_preset(2);
assert.equal(physics.physics_demo_vehicle_preset(), 2);
physics.physics_set_experience_profile(3);
physics.physics_set_keyboard_input(0.2, 0.7, 0, 0, 0, 0);
physics.physics_step(80);
physics.physics_snapshot_save();
physics.physics_step(120);
const expectedArcade = publicState();
physics.physics_select_demo_vehicle_preset(1);
assert.equal(physics.physics_demo_vehicle_preset(), 1);
assert.equal(physics.physics_snapshot_restore(), 1);
assert.equal(physics.physics_demo_vehicle_preset(), 2, 'snapshot restores authored demo selection');
assert.equal(physics.physics_experience_profile(), 3, 'snapshot restores external Arcade controller profile');
physics.physics_step(120);
assert.deepEqual(publicState(), expectedArcade, 'Arcade browser snapshot must deterministically re-simulate');
physics.physics_reset();
assert.equal(physics.physics_demo_vehicle_preset(), 2, 'reset preserves selected Arcade physical definition');
physics.physics_select_demo_vehicle_preset(1);

function scheduleRun(refreshHz) {
  physics.physics_reset();
  physics.physics_set_experience_profile(1);
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

function gamepadScheduleRun(refreshHz) {
  physics.physics_reset();
  physics.physics_set_experience_profile(1);
  // Build speed before testing input scheduling; the same physical steps occur
  // in every render grouping.
  physics.physics_set_device_input(2, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0);
  physics.physics_step(8_000);
  const events = [
    [0, 0.25],
    [180, 0.5],
    [410, -0.5],
    [700, 0],
  ];
  let eventIndex = 0;
  let step = 0;
  let frame = 1;
  while (step < 1_000) {
    const frameEnd = Math.min(1_000, Math.floor((frame * 1_000) / refreshHz + 1e-9));
    while (eventIndex < events.length && events[eventIndex][0] <= frameEnd) {
      const [eventStep, steering] = events[eventIndex];
      if (eventStep > step) physics.physics_step(eventStep - step);
      step = eventStep;
      physics.physics_set_device_input(2, steering, 0.5, 0, 0, 0, steering, 0.5, 0, 0, 0, 0);
      eventIndex += 1;
    }
    if (frameEnd > step) physics.physics_step(frameEnd - step);
    step = frameEnd;
    frame += 1;
  }
  return [...publicState(), ...pipelineState()];
}

const gamepadScheduleReference = gamepadScheduleRun(30);
for (const refreshHz of [60, 120, 144]) {
  assert.deepEqual(gamepadScheduleRun(refreshHz), gamepadScheduleReference, `${refreshHz} Hz grouping changed gamepad policy`);
}

console.log(JSON.stringify({
  savedBytes,
  comparedValues: expected.length,
  pipelineValues: savedPipeline.length,
  renderSchedulesHz: [30, 60, 120, 144],
  gamepadPolicySpeedKmh: policySpeedMps * 3.6,
  sportHalf,
  sportRequestedMps2,
  policySamples,
  autopilotRestored: true,
}, null, 2));
