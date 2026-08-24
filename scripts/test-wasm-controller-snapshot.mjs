import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

const repository = new URL('../', import.meta.url);
const binary = await readFile(new URL('web/physics.wasm', repository));
const { instance } = await WebAssembly.instantiate(binary, {});
const physics = instance.exports;

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

physics.physics_reset();
physics.physics_set_keyboard_input(1, 0.65, 0, 0, 0, 0);
physics.physics_step(73);
const savedBytes = physics.physics_snapshot_save();
assert(savedBytes > 0);

physics.physics_step(480);
const expected = publicState();

// Mutate every controller item that is external to PhysicsWorld.
physics.physics_set_keyboard_input(-1, 0, 0.4, 0, 0, 0);
physics.physics_set_input(-0.75, 0, 0, 0, 0, 0);
physics.physics_set_player_autopilot(1);
physics.physics_step(31);

assert.equal(physics.physics_snapshot_restore(), 1);
assert.equal(physics.physics_player_autopilot(), 0);
physics.physics_step(480);
assert.deepEqual(publicState(), expected, 'restored keyboard branch must reproduce every public physical value');

// Autopilot mode is also part of the browser execution state.
physics.physics_set_player_autopilot(1);
physics.physics_step(10);
physics.physics_snapshot_save();
physics.physics_set_player_autopilot(0);
assert.equal(physics.physics_snapshot_restore(), 1);
assert.equal(physics.physics_player_autopilot(), 1);

console.log(JSON.stringify({ savedBytes, comparedValues: expected.length, autopilotRestored: true }, null, 2));
