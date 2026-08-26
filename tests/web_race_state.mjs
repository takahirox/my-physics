import assert from 'node:assert/strict';
import test from 'node:test';

import { RACE_PHASE, RaceDirector, formatRaceTime } from '../web/race-state.mjs';

function startRace(director, progresses, timeS = 3) {
  const state = director.update(timeS, progresses);
  assert.equal(state.phase, RACE_PHASE.RACING);
  return state;
}

function driveLaps(director, vehicleProgresses, fromTimeS, lapCount, step = 0.05) {
  let timeS = fromTimeS;
  const updates = Math.round(lapCount / step);
  let state;
  for (let update = 0; update < updates; update += 1) {
    timeS += 0.5;
    vehicleProgresses[0] = (vehicleProgresses[0] + step) % 1;
    state = director.update(timeS, vehicleProgresses);
  }
  return { state, timeS };
}

test('configuration rejects ambiguous checkpoint and progress policies', () => {
  assert.throws(() => new RaceDirector({ totalLaps: 0 }), /totalLaps/);
  assert.throws(() => new RaceDirector({ countdownDurationS: -1 }), /countdownDurationS/);
  assert.throws(() => new RaceDirector({ checkpoints: [0.5, 0.25, 1] }), /checkpoints/);
  assert.throws(() => new RaceDirector({ checkpoints: [0.25, 0.75] }), /checkpoints/);
  assert.throws(() => new RaceDirector({ maximumProgressDelta: 0.5 }), /maximumProgressDelta/);
});

test('countdown and race clock use authoritative monotonic physics time', () => {
  const director = new RaceDirector();
  director.reset(10, [0, 0.99]);
  assert.deepEqual(director.view(10), {
    phase: RACE_PHASE.COUNTDOWN,
    totalLaps: 3,
    countdownRemainingS: 3,
    countdownValue: 3,
    raceTimeS: 0,
    player: director.view(10).player,
    standings: director.view(10).standings,
  });
  assert.equal(director.update(11.01, [0, 0.99]).countdownValue, 2);
  assert.equal(director.update(12.01, [0, 0.99]).countdownValue, 1);
  const green = director.update(13, [0, 0.99]);
  assert.equal(green.phase, RACE_PHASE.RACING);
  assert.deepEqual(green.events, [{ type: 'race-started', timeS: 13 }]);
  assert.equal(director.update(13.25, [0, 0.99]).raceTimeS, 0.25);
  assert.throws(() => director.update(13.24, [0, 0.99]), /monotonic/);
});

test('cars staged behind the line rank behind the pole row', () => {
  const director = new RaceDirector();
  director.reset(0, [0, 0, 0.99, 0.99, 0.98, 0.98]);
  const standings = director.view(0).standings;
  assert.deepEqual(standings.map(({ index }) => index), [0, 1, 2, 3, 4, 5]);
  assert.ok(Math.abs(standings[2].distanceLaps + 0.01) < 1e-12);
  assert.ok(Math.abs(standings[4].distanceLaps + 0.02) < 1e-12);
});

test('countdown movement is synchronized but never credited as race distance', () => {
  const director = new RaceDirector();
  director.reset(0, [0]);
  director.update(1, [0.1]);
  director.update(2, [0.2]);
  const green = director.update(3, [0.3]);
  assert.equal(green.player.distanceLaps, 0);
  director.update(3.1, [0.35]);
  assert.ok(Math.abs(director.view(3.1).player.distanceLaps - 0.05) < 1e-12);
});

test('sequential checkpoints award exactly three timed laps and finish the player', () => {
  const director = new RaceDirector({ totalLaps: 3 });
  const progresses = [0];
  director.reset(0, progresses);
  startRace(director, progresses);
  const { state } = driveLaps(director, progresses, 3, 3);
  assert.equal(state.phase, RACE_PHASE.FINISHED);
  assert.equal(state.player.completedLaps, 3);
  assert.deepEqual(state.player.lapTimesS, [10, 10, 10]);
  assert.equal(state.player.bestLapTimeS, 10);
  assert.equal(state.player.finishTimeS, 30);
  assert.equal(state.player.finishPosition, 1);
  assert.equal(state.player.position, 1);
  assert.equal(state.raceTimeS, 30);
  assert.equal(state.events.at(-1).type, 'race-finished');
});

test('reverse start-line crossings and discontinuous shortcuts cannot award a lap', () => {
  const director = new RaceDirector({ totalLaps: 1, maximumProgressDelta: 0.08 });
  director.reset(0, [0.01]);
  startRace(director, [0.01]);
  director.update(3.1, [0.99]);
  assert.equal(director.view().player.completedLaps, 0, 'reverse wrap is not a lap');

  const rejected = director.update(3.2, [0.45]);
  assert.equal(rejected.events[0].type, 'progress-rejected');
  assert.equal(director.view().player.invalidProgressSamples, 1);
  assert.equal(director.view().player.completedLaps, 0);

  director.update(3.3, [0.5]);
  director.update(3.4, [0.55]);
  director.update(3.5, [0.6]);
  assert.equal(director.view().player.completedLaps, 0, 'driving after a rejected shortcut still needs every checkpoint');
});

test('position uses race distance, deterministic index ties, and immutable finish order', () => {
  const director = new RaceDirector({ totalLaps: 1, countdownDurationS: 0 });
  const progresses = [0, 0, 0];
  director.reset(5, progresses);
  startRace(director, progresses, 5);
  director.update(5.5, [0.05, 0.05, 0.04]);
  assert.deepEqual(director.view().standings.map(({ index }) => index), [0, 1, 2]);
  director.update(6, [0.05, 0.1, 0.09]);
  assert.deepEqual(director.view().standings.map(({ index }) => index), [1, 2, 0]);
});

test('snapshot restore reproduces race state without sharing mutable lap arrays', () => {
  const original = new RaceDirector({ totalLaps: 2 });
  const progresses = [0, 0.99];
  original.reset(0, progresses);
  startRace(original, progresses);
  driveLaps(original, progresses, 3, 1);
  const snapshot = original.snapshot();

  const restored = new RaceDirector({ totalLaps: 2 });
  assert.deepEqual(restored.restore(snapshot), original.view());
  snapshot.participants[0].lapTimesS.push(999);
  assert.deepEqual(restored.view(), original.view());

  const nextOriginal = original.update(13.5, [0.05, 0.99]);
  const nextRestored = restored.update(13.5, [0.05, 0.99]);
  assert.deepEqual(nextRestored, nextOriginal);
  assert.throws(() => new RaceDirector({ totalLaps: 3 }).restore(restored.snapshot()), /configuration mismatch/);
});

test('race time formatting is compact and stable', () => {
  assert.equal(formatRaceTime(0), '0:00.000');
  assert.equal(formatRaceTime(61.2344), '1:01.234');
  assert.equal(formatRaceTime(-1), '0:00.000');
  assert.equal(formatRaceTime(Number.NaN), '—');
});
