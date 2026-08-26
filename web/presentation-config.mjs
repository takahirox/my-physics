const WHEELS = 4;

export const EFFECT_LIMITS = Object.freeze({
  smokeParticles: 192,
  sprayParticles: 256,
  sparkParticles: 48,
  speedStreaks: 96,
  smokePerSecondPerWheel: 42,
  sprayPerSecondPerWheel: 64,
  sparksPerSecond: 90,
});

function finite(value, fallback = 0) {
  return Number.isFinite(value) ? value : fallback;
}

export function clamp01(value) {
  return Math.min(1, Math.max(0, finite(value)));
}

function wheelValues(value) {
  if (!Array.isArray(value)) return Array(WHEELS).fill(clamp01(value));
  return Array.from({ length: WHEELS }, (_, wheel) => clamp01(value[wheel]));
}

function wheelTemperatures(value) {
  if (!Array.isArray(value)) return Array(WHEELS).fill(finite(value, 300));
  return Array.from({ length: WHEELS }, (_, wheel) => finite(value[wheel], 300));
}

/**
 * Maps authoritative telemetry to visual intensities. These values are
 * presentation-only: callers must never write them back into the plant.
 */
export function effectIntensities(telemetry = {}) {
  const speedMps = Math.min(100, Math.max(0, finite(telemetry.speedMps)));
  const moving = clamp01((speedMps - 2) / 18);
  const waterDepthMm = Math.min(20, Math.max(0, finite(telemetry.waterDepthMm)));
  const wetness = clamp01(waterDepthMm / 1.5);
  const dryness = 1 - wetness;
  const scrub = wheelValues(telemetry.tireScrub);
  const hydroplaning = wheelValues(telemetry.hydroplaning);
  const brakeTemperatureK = wheelTemperatures(telemetry.brakeTemperatureK);
  const smoke = scrub.map((value) => clamp01((value - 0.14) / 0.7) * dryness * moving);
  const spray = hydroplaning.map((value) => wetness * moving * (0.35 + value * 0.65));
  const brakeGlow = brakeTemperatureK.map((temperature) => clamp01((temperature - 620) / 430));
  const impact = clamp01(telemetry.impact);
  const damage = clamp01(telemetry.damage);
  const engineLoad = clamp01(telemetry.engineLoad);
  const engineRpmRatio = clamp01(finite(telemetry.engineRpm) / Math.max(1, finite(telemetry.redlineRpm, 7_000)));
  return {
    smoke,
    spray,
    brakeGlow,
    sparks: impact * (0.45 + damage * 0.55),
    exhaustPulse: engineLoad * engineRpmRatio,
    speedStreak: clamp01((speedMps - 22) / 48),
  };
}

/** Converts intensities into bounded emitter rates for a pooled renderer. */
export function effectEmissionRates(telemetry = {}) {
  const intensity = effectIntensities(telemetry);
  return {
    ...intensity,
    smokePerSecond: intensity.smoke.map((value) => value * EFFECT_LIMITS.smokePerSecondPerWheel),
    sprayPerSecond: intensity.spray.map((value) => value * EFFECT_LIMITS.sprayPerSecondPerWheel),
    sparksPerSecond: intensity.sparks * EFFECT_LIMITS.sparksPerSecond,
  };
}

/** Classifies observable drift-playground outcomes without feeding the plant. */
export function classifyDriftOutcome(state = {}) {
  const phase = Math.max(0, Math.min(4, Math.trunc(finite(state.phase))));
  const betaDeg = Math.abs(finite(state.betaDeg));
  const speedKmh = Math.max(0, finite(state.speedKmh));
  const rawSteering = Math.abs(finite(state.rawSteering));
  const frontSlipDeg = Math.abs(finite(state.frontSlipDeg));
  const rearSlipDeg = Math.abs(finite(state.rearSlipDeg));
  if (phase === 4 || betaDeg >= 55) return { kind: 'spin', label: 'SPIN — RESTART AND ADJUST ENTRY' };
  if (phase === 3 && (speedKmh < 35 || betaDeg >= 35)) {
    return { kind: 'poor-exit', label: 'POOR EXIT — REDUCE ENTRY ANGLE' };
  }
  if (phase <= 1 && rawSteering >= 0.5 && speedKmh >= 40 && frontSlipDeg >= 8 && frontSlipDeg >= rearSlipDeg + 3) {
    return { kind: 'understeer', label: 'UNDERSTEER — CHANGE SPEED OR ENTRY' };
  }
  if (phase === 2 && betaDeg >= 8) return { kind: 'slide', label: 'CONTROLLED SLIDE — HOLD YOUR LINE' };
  if (phase === 3) return { kind: 'recovery', label: 'RECOVERY — ACCELERATE OUT' };
  if (phase === 1) return { kind: 'entry', label: 'ENTRY ARMED — CREATE PHYSICAL SLIP' };
  return { kind: 'grip', label: 'GRIP — SET UP THE ENTRY' };
}
