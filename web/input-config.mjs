export const INPUT_CONFIG_STORAGE_KEY = 'my-physics.input-config.v1';

export const DEFAULT_INPUT_CONFIG = Object.freeze({
  keyboardAdaptive: true,
  response: 'balanced',
  gamepadDeadzone: 0.08,
  gamepadOuterDeadzone: 0.04,
  gamepadExponent: 1.55,
  wheelDeadzone: 0,
  steeringMin: -1,
  steeringCenter: 0,
  steeringMax: 1,
  throttleReleased: 1,
  throttlePressed: -1,
  brakeReleased: 1,
  brakePressed: -1,
  clutchReleased: 1,
  clutchPressed: -1,
  calibratedDevice: '',
});

function finite(value, fallback) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

export function sanitizeInputConfig(candidate = {}) {
  const defaults = DEFAULT_INPUT_CONFIG;
  const minimum = finite(candidate.steeringMin, defaults.steeringMin);
  const maximum = finite(candidate.steeringMax, defaults.steeringMax);
  const center = finite(candidate.steeringCenter, defaults.steeringCenter);
  const validSteeringRange = minimum < center && center < maximum;
  return {
    keyboardAdaptive: candidate.keyboardAdaptive !== false,
    response: candidate.response === 'direct' ? 'direct' : 'balanced',
    gamepadDeadzone: Math.min(0.35, Math.max(0, finite(candidate.gamepadDeadzone, defaults.gamepadDeadzone))),
    gamepadOuterDeadzone: Math.min(0.2, Math.max(0, finite(candidate.gamepadOuterDeadzone, defaults.gamepadOuterDeadzone))),
    gamepadExponent: Math.min(3, Math.max(0.5, finite(candidate.gamepadExponent, defaults.gamepadExponent))),
    wheelDeadzone: Math.min(0.1, Math.max(0, finite(candidate.wheelDeadzone, defaults.wheelDeadzone))),
    steeringMin: validSteeringRange ? minimum : defaults.steeringMin,
    steeringCenter: validSteeringRange ? center : defaults.steeringCenter,
    steeringMax: validSteeringRange ? maximum : defaults.steeringMax,
    throttleReleased: finite(candidate.throttleReleased, defaults.throttleReleased),
    throttlePressed: finite(candidate.throttlePressed, defaults.throttlePressed),
    brakeReleased: finite(candidate.brakeReleased, defaults.brakeReleased),
    brakePressed: finite(candidate.brakePressed, defaults.brakePressed),
    clutchReleased: finite(candidate.clutchReleased, defaults.clutchReleased),
    clutchPressed: finite(candidate.clutchPressed, defaults.clutchPressed),
    calibratedDevice: String(candidate.calibratedDevice || ''),
  };
}

export function inputConfigFromSources(search = '', storedJson = '') {
  let stored = {};
  try {
    stored = storedJson ? JSON.parse(storedJson) : {};
  } catch {
    stored = {};
  }
  const parameters = search instanceof URLSearchParams ? search : new URLSearchParams(search);
  const combined = { ...DEFAULT_INPUT_CONFIG, ...stored };
  const numericParameters = {
    inputDeadzone: 'gamepadDeadzone',
    inputOuterDeadzone: 'gamepadOuterDeadzone',
    inputExpo: 'gamepadExponent',
    wheelDeadzone: 'wheelDeadzone',
    steerMin: 'steeringMin',
    steerCenter: 'steeringCenter',
    steerMax: 'steeringMax',
    throttleReleased: 'throttleReleased',
    throttlePressed: 'throttlePressed',
    brakeReleased: 'brakeReleased',
    brakePressed: 'brakePressed',
    clutchReleased: 'clutchReleased',
    clutchPressed: 'clutchPressed',
  };
  for (const [parameter, field] of Object.entries(numericParameters)) {
    if (parameters.has(parameter)) {
      combined[field] = parameters.get(parameter);
      if (!['gamepadDeadzone', 'gamepadOuterDeadzone', 'gamepadExponent', 'wheelDeadzone'].includes(field)) {
        combined.calibratedDevice = '';
      }
    }
  }
  if (parameters.has('inputResponse')) combined.response = parameters.get('inputResponse');
  if (parameters.has('keyboardAssist')) combined.keyboardAdaptive = parameters.get('keyboardAssist') !== '0';
  if (parameters.has('inputMode')) combined.keyboardAdaptive = parameters.get('inputMode') !== 'raw';
  return sanitizeInputConfig(combined);
}

export function normalizeCenteredAxis(rawValue, config, wheel = false) {
  const raw = finite(rawValue, config.steeringCenter);
  const span = raw >= config.steeringCenter
    ? config.steeringMax - config.steeringCenter
    : config.steeringCenter - config.steeringMin;
  const unit = span > 1e-9 ? (raw - config.steeringCenter) / span : 0;
  const deadzone = wheel ? config.wheelDeadzone : config.gamepadDeadzone;
  const outer = wheel ? 0 : config.gamepadOuterDeadzone;
  const magnitude = Math.abs(unit);
  const linear = Math.min(1, Math.max(0, (magnitude - deadzone) / Math.max(1e-6, 1 - deadzone - outer)));
  if (linear === 0) return 0;
  const exponent = wheel || config.response === 'direct' ? 1 : config.gamepadExponent;
  return Math.sign(unit) * linear ** exponent;
}

export function normalizePedalAxis(rawValue, released, pressed) {
  const range = pressed - released;
  if (!Number.isFinite(rawValue) || Math.abs(range) <= 1e-9) return 0;
  return Math.min(1, Math.max(0, (rawValue - released) / range));
}

/// A UI-captured rest/center calibration belongs to one concrete controller.
/// URL-authored ranges have no device id and intentionally remain portable.
export function inputConfigForDevice(config, deviceId) {
  if (!config.calibratedDevice || config.calibratedDevice === deviceId) return config;
  return {
    ...config,
    steeringMin: DEFAULT_INPUT_CONFIG.steeringMin,
    steeringCenter: DEFAULT_INPUT_CONFIG.steeringCenter,
    steeringMax: DEFAULT_INPUT_CONFIG.steeringMax,
    throttleReleased: DEFAULT_INPUT_CONFIG.throttleReleased,
    throttlePressed: DEFAULT_INPUT_CONFIG.throttlePressed,
    brakeReleased: DEFAULT_INPUT_CONFIG.brakeReleased,
    brakePressed: DEFAULT_INPUT_CONFIG.brakePressed,
    clutchReleased: DEFAULT_INPUT_CONFIG.clutchReleased,
    clutchPressed: DEFAULT_INPUT_CONFIG.clutchPressed,
  };
}

export function inputActivityMagnitude(input) {
  return Math.max(
    Math.abs(input.steer || 0),
    Math.abs(input.throttle || 0),
    Math.abs(input.brake || 0),
    Math.abs(input.clutch || 0),
    Math.abs(input.handbrake || 0),
  );
}

export class DeviceActivityLatch {
  constructor({ threshold = 0.12, latchMs = 450, idleReleaseMs = 180 } = {}) {
    this.threshold = threshold;
    this.latchMs = latchMs;
    this.idleReleaseMs = idleReleaseMs;
    this.active = 'keyboard';
    this.lastActivityMs = Number.NEGATIVE_INFINITY;
    this.lastSwitchMs = Number.NEGATIVE_INFINITY;
  }

  select(candidates, nowMs) {
    const current = candidates.find(({ key }) => key === this.active);
    if ((current?.magnitude || 0) >= this.threshold) this.lastActivityMs = nowMs;
    const alternatives = candidates
      .filter(({ key, magnitude }) => key !== this.active && magnitude >= this.threshold)
      .sort((a, b) => b.magnitude - a.magnitude);
    const next = alternatives[0];
    if (!next) return this.active;
    const prioritySwitch = next.priority && nowMs - this.lastSwitchMs >= 100;
    const released = nowMs - this.lastActivityMs >= this.idleReleaseMs;
    const latchExpired = nowMs - this.lastSwitchMs >= this.latchMs;
    if (prioritySwitch || (released && latchExpired)) {
      this.active = next.key;
      this.lastSwitchMs = nowMs;
      this.lastActivityMs = nowMs;
    }
    return this.active;
  }
}

export function captureRestCalibration(config, pad, axes) {
  if (!pad) return sanitizeInputConfig(config);
  return sanitizeInputConfig({
    ...config,
    steeringCenter: axes.steering,
    throttleReleased: axes.throttle,
    brakeReleased: axes.brake,
    clutchReleased: axes.clutch,
    calibratedDevice: pad.id,
  });
}
