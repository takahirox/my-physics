import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

const binary = await readFile(new URL('../web/physics.wasm', import.meta.url));
const { instance } = await WebAssembly.instantiate(binary, {});
const physics = instance.exports;

physics.physics_select_demo_vehicle_preset(3);
assert.equal(physics.physics_demo_vehicle_preset(), 3);
assert.equal(physics.physics_vehicle_count(), 1);
physics.physics_set_experience_profile(2);
assert.equal(physics.physics_experience_profile(), 2);
assert.equal(physics.physics_keyboard_assist(), 0);

const reports = [];
for (let scenario = 0; scenario < 6; scenario += 1) {
  assert.equal(physics.physics_validation_run(scenario), 1, `catalog scenario ${scenario} failed`);
  const sampleCount = physics.physics_validation_sample_count();
  const checkCount = physics.physics_validation_check_count();
  assert(sampleCount > 1);
  assert(checkCount > 0);
  for (let field = 0; field <= 18; field += 1) {
    assert(Number.isFinite(physics.physics_validation_sample(0, field)), `missing sample field ${field}`);
  }
  for (let check = 0; check < checkCount; check += 1) {
    assert.equal(physics.physics_validation_check(check, 4), 1);
  }
  const fingerprint = `${(physics.physics_validation_fingerprint_high() >>> 0).toString(16).padStart(8, '0')}${
    (physics.physics_validation_fingerprint_low() >>> 0).toString(16).padStart(8, '0')}`;
  assert.equal(physics.physics_validation_run(scenario), 1);
  const repeated = `${(physics.physics_validation_fingerprint_high() >>> 0).toString(16).padStart(8, '0')}${
    (physics.physics_validation_fingerprint_low() >>> 0).toString(16).padStart(8, '0')}`;
  assert.equal(repeated, fingerprint, `scenario ${scenario} was not repeatable`);
  reports.push({ scenario, sampleCount, checkCount, fingerprint });
}

for (let scenario = 0; scenario < 6; scenario += 1) {
  assert.equal(physics.physics_validation_midpoint_replay(scenario), 1, `scenario ${scenario} snapshot replay diverged`);
}

physics.physics_lab_reset_free_drive();
assert.equal(physics.physics_demo_vehicle_preset(), 3);
assert.equal(physics.physics_experience_profile(), 2);
assert.equal(physics.physics_keyboard_assist(), 0);
const bytes = physics.physics_snapshot_save();
assert(bytes > 0);
physics.physics_step(100);
assert.equal(physics.physics_snapshot_restore(), 1);
assert.equal(physics.physics_demo_vehicle_preset(), 3);

console.log(JSON.stringify({ preset: 'engineering_reference', fixedDtS: 0.001, reports }, null, 2));
