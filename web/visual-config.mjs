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
  cockpit: Object.freeze({
    label: 'COCKPIT · C',
    backM: -0.2,
    heightM: 1.28,
    targetAheadM: 19.0,
    targetHeightM: 0.76,
    baseFovDeg: 68,
    speedFovGainDeg: 8,
    responsePerS: 24,
    maxLagM: 0.16,
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

function finite(value, fallback = 0) {
  return Number.isFinite(value) ? value : fallback;
}

function clamp(value, minimum, maximum) {
  const bounded = Math.min(maximum, Math.max(minimum, finite(value)));
  return bounded === 0 ? 0 : bounded;
}

/**
 * Bounded, rendering-only camera response derived from physical telemetry.
 * The returned envelope contains no phase/noise generator so a renderer can
 * seed shake from the physics step without coupling physics to render timing.
 */
export function cameraTelemetryResponse(telemetry = {}) {
  const speedMps = clamp(telemetry.speedMps, 0, 100);
  const longitudinalAccelerationMps2 = clamp(telemetry.longitudinalAccelerationMps2, -15, 15);
  const lateralAccelerationMps2 = clamp(telemetry.lateralAccelerationMps2, -18, 18);
  const yawRateRadS = clamp(telemetry.yawRateRadS, -4, 4);
  const suspension = Array.isArray(telemetry.suspensionActivity)
    ? telemetry.suspensionActivity.reduce((sum, value) => sum + clamp(value, 0, 1), 0) / Math.max(telemetry.suspensionActivity.length, 1)
    : clamp(telemetry.suspensionActivity, 0, 1);
  const impact = clamp(telemetry.impact, 0, 1);
  return {
    pitchDeg: clamp(-longitudinalAccelerationMps2 * 0.2, -2.8, 2.8),
    rollDeg: clamp(-lateralAccelerationMps2 * 0.15 - yawRateRadS * 0.35, -3.5, 3.5),
    lateralOffsetM: clamp(-lateralAccelerationMps2 * 0.014, -0.22, 0.22),
    longitudinalOffsetM: clamp(-longitudinalAccelerationMps2 * 0.01, -0.14, 0.14),
    shakeEnvelopeM: clamp(suspension * 0.024 + impact * 0.055, 0, 0.075),
    edgeStreak: clamp((speedMps - 18) / 52, 0, 1),
  };
}

export function cameraSettings(presetName, speedMps) {
  const preset = CAMERA_PRESETS[presetName] || CAMERA_PRESETS.chase;
  const speedRatio = clamp(speedMps, 0, 65) / 65;
  return { ...preset, fieldOfViewDegrees: preset.baseFovDeg + preset.speedFovGainDeg * speedRatio };
}
