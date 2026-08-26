import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';

const debugPort = process.env.CHROME_DEBUG_PORT || '9229';
const demoUrl = process.env.DEMO_URL || 'http://127.0.0.1:8080/?demo=arcade&playground=drift';
const handbrakeMs = 800;
const steeringKey = 'KeyA';
const version = await fetch(`http://127.0.0.1:${debugPort}/json/version`).then((response) => response.json());
const socket = new WebSocket(version.webSocketDebuggerUrl);
await new Promise((resolve, reject) => {
  socket.addEventListener('open', resolve, { once: true });
  socket.addEventListener('error', reject, { once: true });
});
let sequence = 0;
const pending = new Map();
const exceptions = [];
socket.addEventListener('message', (event) => {
  const message = JSON.parse(event.data);
  if (message.id && pending.has(message.id)) {
    const { resolve, reject } = pending.get(message.id);
    pending.delete(message.id);
    if (message.error) reject(new Error(message.error.message)); else resolve(message.result);
  }
  if (message.method === 'Runtime.exceptionThrown') exceptions.push(message.params.exceptionDetails.text);
});
function command(method, params = {}, sessionId) {
  const id = ++sequence;
  socket.send(JSON.stringify({ id, method, params, ...(sessionId ? { sessionId } : {}) }));
  return new Promise((resolve, reject) => pending.set(id, { resolve, reject }));
}
const { targetId } = await command('Target.createTarget', { url: 'about:blank' });
const { sessionId } = await command('Target.attachToTarget', { targetId, flatten: true });
await command('Runtime.enable', {}, sessionId);
await command('Page.enable', {}, sessionId);
await command('Page.navigate', { url: demoUrl }, sessionId);
const sleep = (milliseconds) => new Promise((resolve) => setTimeout(resolve, milliseconds));
async function evaluate(expression) {
  const result = await command('Runtime.evaluate', { expression, awaitPromise: true, returnByValue: true }, sessionId);
  if (result.exceptionDetails) throw new Error(result.exceptionDetails.text);
  return result.result.value;
}
for (let retry = 0; retry < 150; retry += 1) {
  if (await evaluate("document.querySelector('#status')?.classList.contains('ready') || false")) break;
  await sleep(100);
}
assert.equal(await evaluate("document.querySelector('#status')?.classList.contains('ready')"), true);
assert.equal(await evaluate("document.body.classList.contains('drift-playground')"), true);
assert.equal(await evaluate('window.__MY_PHYSICS_INPUT__.demoVehiclePreset'), 'arcade_fun');
assert.equal(await evaluate('window.__MY_PHYSICS_INPUT__.experienceProfile'), 'arcade');
assert.equal(await evaluate("document.querySelector('#driftPanel').hidden"), false);
assert.equal(await evaluate("getComputedStyle(document.querySelector('#raceHud')).display"), 'none');
const layout = await evaluate(`(() => {
  const rect = (selector) => {
    const { top, right, bottom, left, width, height } = document.querySelector(selector).getBoundingClientRect();
    return { top, right, bottom, left, width, height };
  };
  const canvas = rect('#track');
  const hud = rect('#driftHud');
  const panel = rect('#driftPanel');
  return { innerWidth, innerHeight, canvas, hud, panel };
})()`);
assert.equal(layout.canvas.width, layout.innerWidth);
assert.equal(layout.canvas.height, layout.innerHeight);
assert(layout.hud.top >= 8 && layout.hud.bottom <= layout.innerHeight, `drift HUD is clipped: ${JSON.stringify(layout.hud)}`);
assert(
  layout.panel.width > 0 && layout.panel.top >= 14 && layout.panel.bottom <= layout.innerHeight,
  `drift telemetry panel is not visible in the overlay: ${JSON.stringify(layout.panel)}`,
);

async function key(code, down) {
  await evaluate(`dispatchEvent(new KeyboardEvent('${down ? 'keydown' : 'keyup'}', { code: '${code}' }))`);
}
async function sample() {
  return evaluate(`(() => {
    const input = window.__MY_PHYSICS_INPUT__;
    return {
      speedKmh: window.__MY_PHYSICS_FRAME__.speedMps * 3.6,
      phase: input.arcadeDrift.phase,
      betaDeg: input.arcadeDrift.bodySlipRad * 180 / Math.PI,
      yawDegS: input.arcadeDrift.yawRateRadS * 180 / Math.PI,
      rawSteering: input.stages.raw.steering,
      policySteering: input.stages.policy.steering,
      throttle: [input.stages.raw.throttle, input.stages.policy.throttle],
      brake: [input.stages.raw.brake, input.stages.policy.brake],
      handbrake: [input.stages.raw.handbrake, input.stages.policy.handbrake],
      wheelLongitudinalSlip: input.arcadeDrift.wheelLongitudinalSlip,
      wheelSlipAngleRad: input.arcadeDrift.wheelSlipAngleRad,
      damage: Number(document.querySelector('#damageText').textContent.replace('%', '')),
    };
  })()`);
}
const samples = [];
async function sampleFor(milliseconds) {
  const count = Math.ceil(milliseconds / 50);
  for (let index = 0; index < count; index += 1) {
    await sleep(50);
    samples.push(await sample());
  }
}

await key('KeyW', true);
await key(steeringKey, true);
await sampleFor(350);
await key('Space', true);
await sampleFor(handbrakeMs);
await key('Space', false);
await sampleFor(500);
await key(steeringKey, false);
await sampleFor(900);
await key('KeyW', false);

const peakBodySlipDeg = Math.max(...samples.map(({ betaDeg }) => Math.abs(betaDeg)));
const final = samples.at(-1);
const countersteerSamples = samples.filter(({ rawSteering, policySteering }) => rawSteering * policySteering < 0).length;
assert(samples.some(({ phase }) => phase === 'ENTRY'), 'physical handbrake did not arm drift entry');
assert(samples.some(({ phase }) => phase === 'SLIDE'), 'Arcade keyboard controller did not observe a physical slide');
assert(samples.some(({ phase }) => phase === 'RECOVERY'), 'Arcade keyboard controller did not reach recovery');
assert(peakBodySlipDeg >= 8, `browser key events did not create readable body slip: ${peakBodySlipDeg} degrees`);
assert(countersteerSamples >= 1, 'browser key events never produced continuous countersteer');
assert.equal(final.damage, 0, 'open proving-ground drift caused physical damage');
for (const state of samples) {
  assert.equal(state.throttle[1], state.throttle[0], 'policy changed player throttle');
  assert.equal(state.brake[1], state.brake[0], 'policy changed player brake');
  assert.equal(state.handbrake[1], state.handbrake[0], 'policy changed physical handbrake');
  assert(state.wheelLongitudinalSlip.every(Number.isFinite));
  assert(state.wheelSlipAngleRad.every(Number.isFinite));
}
assert.equal(exceptions.length, 0, `browser exceptions: ${exceptions.join('; ')}`);

await evaluate("document.querySelector('#driftRestart').click()");
await sleep(120);
const restarted = await sample();
assert(
  restarted.speedKmh >= 60 && restarted.speedKmh <= 85,
  `repeatable entry speed was not restored: ${restarted.speedKmh}`,
);
assert.equal(restarted.phase, 'GRIP');

const summary = {
  browser: version.Browser,
  layout,
  sampleCount: samples.length,
  handbrakeMs,
  steeringKey,
  phases: [...new Set(samples.map(({ phase }) => phase))],
  peakBodySlipDeg,
  countersteerSamples,
  final,
  restarted,
  exceptions,
};
console.log(JSON.stringify(summary, null, 2));
if (process.env.DRIFT_SCREENSHOT) {
  const screenshot = await command('Page.captureScreenshot', { format: 'png', fromSurface: true }, sessionId);
  await writeFile(process.env.DRIFT_SCREENSHOT, Buffer.from(screenshot.data, 'base64'));
}
await command('Target.closeTarget', { targetId });
socket.close();
