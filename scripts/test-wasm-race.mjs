import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { RaceDirector, RACE_PHASE } from '../web/race-state.mjs';

const binary = await readFile(new URL('../web/physics.wasm', import.meta.url));
const { instance } = await WebAssembly.instantiate(binary, {});
const api = instance.exports;
api.physics_select_demo_vehicle_preset(1);
const progresses = () => Array.from({ length: api.physics_vehicle_count() }, (_, index) => api.physics_track_progress(index));
const position = (index) => [api.physics_x(index), api.physics_y(index), api.physics_z(index)];
// One physical lap exercises the entire start/checkpoint/finish path quickly;
// the pure race-director suite separately verifies the configured three laps.
const director = new RaceDirector({ totalLaps: 1 });
director.reset(api.physics_time(), progresses());
api.physics_set_race_running(0);
const staged = Array.from({ length: api.physics_vehicle_count() }, (_, index) => position(index));
api.physics_step(3_010);
for (let index = 0; index < staged.length; index += 1) {
  const current = position(index);
  const horizontalDisplacement = Math.hypot(current[0] - staged[index][0], current[2] - staged[index][2]);
  // Vertical suspension settling remains physical; the gate intentionally
  // uses service brakes rather than freezing/teleporting rigid bodies.
  assert(horizontalDisplacement < 1.0, `vehicle ${index} moved ${horizontalDisplacement}m before GO`);
}
let race = director.update(api.physics_time(), progresses());
assert.equal(race.phase, RACE_PHASE.RACING);
api.physics_set_race_running(1);
api.physics_set_player_autopilot(1);
const start = performance.now();
for (let batch = 0; batch < 5_000 && race.phase !== RACE_PHASE.FINISHED; batch += 1) {
  api.physics_step(50);
  race = director.update(api.physics_time(), progresses());
}
const milliseconds = performance.now() - start;
assert.equal(race.phase, RACE_PHASE.FINISHED, 'physical AI did not complete the validation lap');
assert.equal(race.player.completedLaps, 1);
assert.equal(race.standings.length, 10);
assert(race.player.finishTimeS > 40 && race.player.finishTimeS < 250);
console.log(JSON.stringify({ finishTimeS: race.player.finishTimeS, position: race.player.position, milliseconds }, null, 2));
