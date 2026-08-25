export const VISUAL_CUES = Object.freeze({
  curbBandM: 1.9,
  fencePostM: 5.0,
  asphaltSeamM: 4.0,
  asphaltPatchM: 3.6,
  rubberDashM: 3.2,
  curbJoinOverlapM: 0.08,
  detailRadiusM: 115.0,
});

export const CAMERA_PRESETS = Object.freeze({
  chase: Object.freeze({
    label: 'CHASE · C',
    backM: 6.3,
    heightM: 1.9,
    targetAheadM: 8.0,
    targetHeightM: 0.25,
    baseFovDeg: 62,
    speedFovGainDeg: 10,
    responsePerS: 14,
    maxLagM: 0.65,
  }),
  hood: Object.freeze({
    label: 'HOOD · C',
    backM: -1.45,
    heightM: 1.05,
    targetAheadM: 15.0,
    targetHeightM: 0.65,
    baseFovDeg: 70,
    speedFovGainDeg: 6,
    responsePerS: 20,
    maxLagM: 0.25,
  }),
});

export const CAMERA_PRESET_ORDER = Object.freeze(Object.keys(CAMERA_PRESETS));

export function cueFrequencyHz(speedKmh, spacingM) {
  return speedKmh / 3.6 / spacingM;
}

// Splits a physical segment at a circuit-global metric grid. The returned
// intervals cover the segment exactly and do not depend on segment count.
export function metricIntervals(segmentStartM, segmentLengthM, spacingM) {
  if (!(spacingM > 0) || !(segmentLengthM >= 0)) throw new RangeError('invalid metric interval');
  const endM = segmentStartM + segmentLengthM;
  const intervals = [];
  let localStartM = 0;
  while (segmentStartM + localStartM < endM - 1e-9) {
    const globalM = segmentStartM + localStartM;
    const nextBoundaryM = (Math.floor(globalM / spacingM + 1e-9) + 1) * spacingM;
    const localEndM = Math.min(segmentLengthM, nextBoundaryM - segmentStartM);
    intervals.push({
      localStartM,
      localEndM,
      centerM: (localStartM + localEndM) * 0.5,
      lengthM: localEndM - localStartM,
      band: Math.floor((globalM + 1e-9) / spacingM),
    });
    localStartM = localEndM;
  }
  return intervals;
}

export function metricSamples(segmentStartM, segmentLengthM, spacingM, phaseM = 0) {
  if (!(spacingM > 0) || !(segmentLengthM >= 0)) throw new RangeError('invalid metric samples');
  const firstIndex = Math.ceil((segmentStartM - phaseM - 1e-9) / spacingM);
  const lastM = segmentStartM + segmentLengthM;
  const result = [];
  for (let index = firstIndex; ; index += 1) {
    const globalM = phaseM + index * spacingM;
    if (globalM >= lastM - 1e-9) break;
    if (globalM >= segmentStartM - 1e-9) result.push({ localM: globalM - segmentStartM, globalM, index });
  }
  return result;
}

export function cameraSettings(presetName, speedMps) {
  const preset = CAMERA_PRESETS[presetName] || CAMERA_PRESETS.chase;
  const speedRatio = Math.min(Math.max(speedMps, 0) / 65, 1);
  return { ...preset, fieldOfViewDegrees: preset.baseFovDeg + preset.speedFovGainDeg * speedRatio };
}
