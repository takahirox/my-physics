import assert from 'node:assert/strict';
import test from 'node:test';
import { TelemetryAudioEngine, audioParameters } from '../web/audio-engine.mjs';

class FakeParameter {
  constructor() { this.value = 0; }
  setTargetAtTime(value) { this.value = value; }
}

class FakeNode {
  constructor() {
    this.gain = new FakeParameter();
    this.frequency = new FakeParameter();
    this.threshold = new FakeParameter();
    this.ratio = new FakeParameter();
  }
  connect(node) { return node; }
  start() { this.started = true; }
  stop() { this.stopped = true; }
}

class FakeAudioContext {
  constructor() {
    this.currentTime = 0;
    this.sampleRate = 32;
    this.state = 'suspended';
    this.destination = new FakeNode();
  }
  createGain() { return new FakeNode(); }
  createDynamicsCompressor() { return new FakeNode(); }
  createOscillator() { return new FakeNode(); }
  createBiquadFilter() { return new FakeNode(); }
  createBufferSource() { return new FakeNode(); }
  createBuffer(_channels, length) {
    const data = new Float32Array(length);
    return { getChannelData: () => data };
  }
  async resume() { this.state = 'running'; }
  async close() { this.state = 'closed'; }
}

test('audio mapping is bounded and responds monotonically to load and slip', () => {
  const idle = audioParameters({ engineRpm: 900, engineLoad: 0, tireScrub: [0, 0, 0, 0] });
  const loaded = audioParameters({
    engineRpm: 6_000,
    engineLoad: 1,
    intake: 1,
    exhaust: 1,
    tireScrub: [0, 0.8, 0, 0],
    roadNoise: [0.5, 0.5, 0.5, 0.5],
    wind: 0.7,
    impact: 0.4,
  });
  assert.ok(loaded.firingHz > idle.firingHz);
  assert.ok(loaded.engineFundamentalGain > idle.engineFundamentalGain);
  assert.ok(loaded.tireGain > idle.tireGain);
  assert.ok(Object.values(loaded).every((value) => Number.isFinite(value) && value >= 0));
  const hostile = audioParameters({ engineRpm: Infinity, engineLoad: NaN, tireScrub: [Infinity], wind: -9 });
  assert.ok(Object.values(hostile).every((value) => Number.isFinite(value) && value >= 0));
});

test('audio engine requires explicit unlock and safely supports mute and disposal', async () => {
  const engine = new TelemetryAudioEngine({ AudioContextClass: FakeAudioContext });
  assert.equal(engine.unlocked, false);
  engine.update({ engineRpm: 4_000, engineLoad: 0.8 });
  assert.equal(engine.context, null, 'telemetry update must not violate autoplay policy');
  assert.equal(await engine.unlock(), true);
  assert.equal(engine.unlocked, true);
  engine.update({ engineRpm: 4_000, engineLoad: 0.8, tireScrub: [0.5, 0, 0, 0] });
  assert.ok(engine.nodes.engine.frequency.value > 0);
  engine.setMuted(true);
  assert.equal(engine.nodes.master.gain.value, 0);
  await engine.dispose();
  assert.equal(engine.unlocked, false);
});

test('audio engine is a silent fallback when WebAudio is unavailable', async () => {
  const engine = new TelemetryAudioEngine({ AudioContextClass: null });
  assert.equal(engine.supported, false);
  assert.equal(await engine.unlock(), false);
  assert.doesNotThrow(() => engine.update({ engineRpm: 3_000 }));
  await engine.dispose();
});
