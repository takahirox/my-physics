import assert from 'node:assert/strict';
import test from 'node:test';
import {
  CAMERA_PRESETS,
  VISUAL_CUES,
  cameraSettings,
  cueFrequencyHz,
  metricIntervals,
  metricSamples,
} from '../web/visual-config.mjs';

test('metric decoration is invariant to physical segment subdivision', () => {
  const whole = metricSamples(0, 100, 5).map(({ globalM }) => globalM);
  const split = [
    ...metricSamples(0, 17, 5),
    ...metricSamples(17, 26, 5),
    ...metricSamples(43, 57, 5),
  ].map(({ globalM }) => globalM);
  assert.deepEqual(split, whole);

  const wholeBands = metricIntervals(0, 100, VISUAL_CUES.curbBandM).map((band) => ({
    ...band,
    globalStartM: band.localStartM,
    globalEndM: band.localEndM,
  }));
  const segmentStarts = [0, 17, 43];
  const segmentLengths = [17, 26, 57];
  const bands = segmentStarts.flatMap((start, index) =>
    metricIntervals(start, segmentLengths[index], VISUAL_CUES.curbBandM).map((band) => ({
      ...band,
      globalStartM: start + band.localStartM,
      globalEndM: start + band.localEndM,
    })),
  );
  assert.ok(Math.abs(bands.reduce((sum, band) => sum + band.lengthM, 0) - 100) < 1e-9);
  assert.ok(bands.every(({ lengthM }) => lengthM > 0 && lengthM <= VISUAL_CUES.curbBandM + 1e-9));
  for (let index = 1; index < bands.length; index += 1) {
    assert.ok(Math.abs(bands[index].globalStartM - bands[index - 1].globalEndM) < 1e-9);
    const previousRenderedEnd = bands[index - 1].globalEndM + VISUAL_CUES.curbJoinOverlapM * 0.5;
    const renderedStart = bands[index].globalStartM - VISUAL_CUES.curbJoinOverlapM * 0.5;
    const overlapM = previousRenderedEnd - renderedStart;
    assert.ok(overlapM >= -1e-9 && overlapM <= 0.08 + 1e-9);
  }
  for (let distanceM = 0.05; distanceM < 100; distanceM += 0.1) {
    const splitPiece = bands.find((band) => distanceM >= band.globalStartM && distanceM < band.globalEndM);
    const wholePiece = wholeBands.find((band) => distanceM >= band.globalStartM && distanceM < band.globalEndM);
    assert.equal(splitPiece?.band, wholePiece?.band);
    assert.equal(splitPiece?.band % 2, wholePiece?.band % 2);
  }
});

test('near-field cues pass at useful rates at 100 km/h', () => {
  assert.ok(cueFrequencyHz(100, VISUAL_CUES.curbBandM) >= 10);
  assert.ok(cueFrequencyHz(100, VISUAL_CUES.fencePostM) >= 4);
  assert.ok(cueFrequencyHz(100, VISUAL_CUES.rubberDashM) >= 8);
});

test('camera presets remain close, finite and deliberately distinct', () => {
  assert.ok(CAMERA_PRESETS.chase.backM >= 5.5 && CAMERA_PRESETS.chase.backM <= 7.0);
  assert.ok(CAMERA_PRESETS.chase.heightM >= 1.8 && CAMERA_PRESETS.chase.heightM <= 2.5);
  assert.ok(CAMERA_PRESETS.hood.backM < 0);
  for (const preset of Object.keys(CAMERA_PRESETS)) {
    const settings = cameraSettings(preset, 100 / 3.6);
    assert.ok(Number.isFinite(settings.fieldOfViewDegrees));
    assert.ok(settings.fieldOfViewDegrees >= 55 && settings.fieldOfViewDegrees <= 80);
  }
});
