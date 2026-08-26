import { clamp01 } from './presentation-config.mjs';

function finite(value, fallback = 0) {
  return Number.isFinite(value) ? value : fallback;
}

function mean(values, fallback = 0) {
  if (!Array.isArray(values) || values.length === 0) return finite(values, fallback);
  return values.reduce((sum, value) => sum + finite(value), 0) / values.length;
}

function maximum(values, fallback = 0) {
  if (!Array.isArray(values) || values.length === 0) return finite(values, fallback);
  return values.reduce((result, value) => Math.max(result, finite(value)), 0);
}

/** Pure and bounded WebAudio parameters derived from the plant's AudioFrame. */
export function audioParameters(telemetry = {}) {
  const engineRpm = Math.min(12_000, Math.max(0, finite(telemetry.engineRpm)));
  const engineLoad = clamp01(telemetry.engineLoad);
  const intake = clamp01(telemetry.intake ?? Math.sqrt(engineLoad));
  const exhaust = clamp01(telemetry.exhaust ?? engineLoad * engineRpm / 7_000);
  const tireScrub = clamp01(maximum(telemetry.tireScrub));
  const roadNoise = clamp01(mean(telemetry.roadNoise));
  const speedWind = clamp01(finite(telemetry.speedMps) / 70);
  const wind = clamp01(telemetry.wind ?? speedWind);
  return {
    firingHz: Math.min(420, Math.max(18, engineRpm / 30)),
    engineFundamentalGain: clamp01(0.04 + engineLoad * 0.16) * clamp01(engineRpm / 700),
    engineHarmonicGain: clamp01(exhaust * 0.12 + intake * 0.05),
    tireGain: tireScrub * 0.22,
    roadGain: roadNoise * 0.1,
    windGain: wind * 0.12,
    impactGain: clamp01(telemetry.impact) * 0.32,
  };
}

function setTarget(parameter, value, now, timeConstant = 0.035) {
  if (parameter?.setTargetAtTime) parameter.setTargetAtTime(value, now, timeConstant);
  else if (parameter) parameter.value = value;
}

function deterministicNoise(context) {
  const length = Math.max(1, Math.floor(context.sampleRate * 2));
  const buffer = context.createBuffer(1, length, context.sampleRate);
  const data = buffer.getChannelData(0);
  let state = 0x6d2b79f5;
  for (let index = 0; index < data.length; index += 1) {
    state = (Math.imul(state, 1_664_525) + 1_013_904_223) >>> 0;
    data[index] = state / 0x80000000 - 1;
  }
  return buffer;
}

/**
 * A synthesis-neutral consumer of AudioFrame telemetry. It never constructs
 * an AudioContext until unlock() is called from a user gesture, and degrades
 * to a silent state when WebAudio is unavailable.
 */
export class TelemetryAudioEngine {
  constructor({ AudioContextClass = globalThis.AudioContext || globalThis.webkitAudioContext } = {}) {
    this.AudioContextClass = AudioContextClass;
    this.context = null;
    this.nodes = null;
    this.muted = false;
    this.lastParameters = audioParameters();
  }

  get supported() {
    return typeof this.AudioContextClass === 'function';
  }

  get unlocked() {
    return Boolean(this.context && this.nodes);
  }

  async unlock() {
    if (!this.supported) return false;
    if (!this.context) {
      try {
        this.context = new this.AudioContextClass();
        this.nodes = this.buildGraph(this.context);
      } catch {
        this.context = null;
        this.nodes = null;
        return false;
      }
    }
    if (this.context.state === 'suspended') await this.context.resume();
    this.setMuted(this.muted);
    return true;
  }

  buildGraph(context) {
    const master = context.createGain();
    const compressor = context.createDynamicsCompressor();
    setTarget(compressor.threshold, -18, context.currentTime, 0.01);
    setTarget(compressor.ratio, 6, context.currentTime, 0.01);
    master.connect(compressor);
    compressor.connect(context.destination);

    const engine = context.createOscillator();
    const harmonic = context.createOscillator();
    const impact = context.createOscillator();
    const engineGain = context.createGain();
    const harmonicGain = context.createGain();
    const impactGain = context.createGain();
    engine.type = 'sawtooth';
    harmonic.type = 'square';
    impact.type = 'sine';
    engine.connect(engineGain).connect(master);
    harmonic.connect(harmonicGain).connect(master);
    impact.connect(impactGain).connect(master);
    setTarget(impact.frequency, 58, context.currentTime, 0.01);

    const noise = context.createBufferSource();
    noise.buffer = deterministicNoise(context);
    noise.loop = true;
    const tireFilter = context.createBiquadFilter();
    const roadFilter = context.createBiquadFilter();
    const windFilter = context.createBiquadFilter();
    tireFilter.type = 'bandpass';
    roadFilter.type = 'lowpass';
    windFilter.type = 'highpass';
    const tireGain = context.createGain();
    const roadGain = context.createGain();
    const windGain = context.createGain();
    noise.connect(tireFilter).connect(tireGain).connect(master);
    noise.connect(roadFilter).connect(roadGain).connect(master);
    noise.connect(windFilter).connect(windGain).connect(master);
    engine.start();
    harmonic.start();
    impact.start();
    noise.start();
    return {
      master, compressor, engine, harmonic, impact, engineGain, harmonicGain, impactGain,
      noise, tireFilter, roadFilter, windFilter, tireGain, roadGain, windGain,
    };
  }

  setMuted(muted) {
    this.muted = Boolean(muted);
    if (this.nodes && this.context) setTarget(this.nodes.master.gain, this.muted ? 0 : 0.8, this.context.currentTime, 0.02);
  }

  update(telemetry) {
    const parameters = audioParameters(telemetry);
    this.lastParameters = parameters;
    if (!this.nodes || !this.context) return parameters;
    const now = this.context.currentTime;
    setTarget(this.nodes.engine.frequency, parameters.firingHz, now);
    setTarget(this.nodes.harmonic.frequency, parameters.firingHz * 2, now);
    setTarget(this.nodes.engineGain.gain, parameters.engineFundamentalGain, now);
    setTarget(this.nodes.harmonicGain.gain, parameters.engineHarmonicGain, now);
    setTarget(this.nodes.tireFilter.frequency, 900 + parameters.tireGain * 2_800, now);
    setTarget(this.nodes.roadFilter.frequency, 180 + parameters.roadGain * 1_200, now);
    setTarget(this.nodes.windFilter.frequency, 350 + parameters.windGain * 1_800, now);
    setTarget(this.nodes.tireGain.gain, parameters.tireGain, now);
    setTarget(this.nodes.roadGain.gain, parameters.roadGain, now);
    setTarget(this.nodes.windGain.gain, parameters.windGain, now);
    setTarget(this.nodes.impactGain.gain, parameters.impactGain, now, 0.018);
    return parameters;
  }

  async dispose() {
    if (!this.context) return;
    for (const source of [this.nodes?.engine, this.nodes?.harmonic, this.nodes?.impact, this.nodes?.noise]) {
      try { source?.stop(); } catch { /* Already stopped. */ }
    }
    if (typeof this.context.close === 'function') await this.context.close();
    this.context = null;
    this.nodes = null;
  }
}
