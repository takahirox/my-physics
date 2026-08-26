import assert from 'node:assert/strict';
import test from 'node:test';
import { EFFECT_LIMITS, classifyDriftOutcome, effectEmissionRates, effectIntensities } from '../web/presentation-config.mjs';

test('effects are absent for stationary neutral telemetry', () => {
  const effects = effectIntensities();
  assert.deepEqual(effects.smoke, [0, 0, 0, 0]);
  assert.deepEqual(effects.spray, [0, 0, 0, 0]);
  assert.deepEqual(effects.brakeGlow, [0, 0, 0, 0]);
  assert.equal(effects.sparks, 0);
  assert.equal(effects.exhaustPulse, 0);
  assert.equal(effects.speedStreak, 0);
});

test('scrub produces smoke when dry and spray replaces it when wet', () => {
  const dry = effectIntensities({ speedMps: 35, waterDepthMm: 0, tireScrub: [0.8, 0.4, 0, 0] });
  const wet = effectIntensities({
    speedMps: 35,
    waterDepthMm: 3,
    tireScrub: [0.8, 0.4, 0, 0],
    hydroplaning: [0.8, 0.4, 0.2, 0.1],
  });
  assert.ok(dry.smoke[0] > dry.smoke[1] && dry.smoke[1] > 0);
  assert.ok(dry.spray.every((value) => value === 0));
  assert.ok(wet.smoke.every((value) => value === 0));
  assert.ok(wet.spray[0] > wet.spray[1] && wet.spray[3] > 0);
});

test('all effect intensities and rates remain bounded under hostile input', () => {
  const rates = effectEmissionRates({
    speedMps: 1e9,
    waterDepthMm: 1e9,
    tireScrub: [Infinity, 9, NaN, -2],
    hydroplaning: [9, 9, 9, 9],
    brakeTemperatureK: [9e9, Infinity, NaN, -1],
    impact: 99,
    damage: 99,
    engineLoad: 99,
    engineRpm: 1e9,
    redlineRpm: 0,
  });
  for (const name of ['smoke', 'spray', 'brakeGlow']) {
    assert.ok(rates[name].every((value) => Number.isFinite(value) && value >= 0 && value <= 1));
  }
  assert.ok(rates.smokePerSecond.every((value) => value <= EFFECT_LIMITS.smokePerSecondPerWheel));
  assert.ok(rates.sprayPerSecond.every((value) => value <= EFFECT_LIMITS.sprayPerSecondPerWheel));
  assert.ok(rates.sparksPerSecond <= EFFECT_LIMITS.sparksPerSecond);
});

test('drift presentation distinguishes grip, understeer, slide, recovery, poor exit and spin', () => {
  assert.equal(classifyDriftOutcome().kind, 'grip');
  assert.equal(
    classifyDriftOutcome({ phase: 1, speedKmh: 75, rawSteering: 1, frontSlipDeg: 13, rearSlipDeg: 5 }).kind,
    'understeer',
  );
  assert.equal(classifyDriftOutcome({ phase: 2, betaDeg: 18, speedKmh: 65 }).kind, 'slide');
  assert.equal(classifyDriftOutcome({ phase: 3, betaDeg: 3, speedKmh: 60 }).kind, 'recovery');
  assert.equal(classifyDriftOutcome({ phase: 3, betaDeg: 38, speedKmh: 25 }).kind, 'poor-exit');
  assert.equal(classifyDriftOutcome({ phase: 4, betaDeg: 70 }).kind, 'spin');
});
