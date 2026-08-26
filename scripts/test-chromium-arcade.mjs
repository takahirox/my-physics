import assert from 'node:assert/strict';

const debugPort = process.env.CHROME_DEBUG_PORT || '9224';
const demoUrl = process.env.DEMO_URL || 'http://127.0.0.1:8091/?demo=arcade';
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
    if (message.error) reject(new Error(message.error.message));
    else resolve(message.result);
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
for (let retry = 0; retry < 100; retry += 1) {
  if (await evaluate("document.querySelector('#status')?.classList.contains('ready') || false")) break;
  await sleep(100);
}
assert.equal(await evaluate("document.querySelector('#status')?.classList.contains('ready')"), true);
assert.equal(await evaluate('window.__MY_PHYSICS_INPUT__.demoVehiclePreset'), 'arcade_fun');
assert.equal(await evaluate('window.__MY_PHYSICS_INPUT__.experienceProfile'), 'arcade');
assert.equal(await evaluate("document.querySelector('h1').textContent"), 'Arcade Fun Circuit');

async function key(code, down) {
  await evaluate(`dispatchEvent(new KeyboardEvent('${down ? 'keydown' : 'keyup'}', { code: '${code}' }))`);
}
async function reset() {
  await key('KeyR', true);
  await key('KeyR', false);
  await sleep(3_250);
}
async function metrics(label) {
  return evaluate(`({
    label: ${JSON.stringify(label)},
    speedKmh: Number(document.querySelector('#speed').textContent),
    damage: Number(document.querySelector('#damageText').textContent.replace('%','')),
    preset: window.__MY_PHYSICS_INPUT__.demoVehiclePreset,
    profile: window.__MY_PHYSICS_INPUT__.experienceProfile,
    policySteering: window.__MY_PHYSICS_INPUT__.stages.policy.steering,
    benchmark: window.__MY_PHYSICS_BENCHMARK__,
  })`);
}

const runs = [];
await reset();
await key('KeyW', true);
await sleep(3_200);
await key('KeyW', false);
runs.push(await metrics('acceleration'));

await reset();
await key('KeyW', true);
await sleep(1_800);
await key('KeyA', true);
await sleep(250);
await key('KeyA', false);
await sleep(450);
await key('KeyW', false);
runs.push(await metrics('corner-and-recover'));

await reset();
await key('KeyW', true);
await sleep(1_700);
await key('KeyD', true);
await sleep(300);
await key('Space', true);
await sleep(350);
await key('Space', false);
await key('KeyD', false);
await key('KeyA', true);
await sleep(700);
await key('KeyA', false);
await key('KeyW', false);
await sleep(900);
runs.push(await metrics('handbrake-and-countersteer'));

console.log(JSON.stringify({ browser: version.Browser, runs, exceptions }, null, 2));
for (const run of runs) {
  assert.equal(run.preset, 'arcade_fun');
  assert.equal(run.profile, 'arcade');
  assert.equal(run.damage, 0, `${run.label} contacted a barrier`);
  assert(run.speedKmh > 20, `${run.label} failed to retain motion`);
  assert(run.benchmark.realtime > 1, 'ten-vehicle browser physics missed real time');
}
assert(Math.abs(runs[2].policySteering) < 0.01, 'handbrake recovery did not return the rack to center');
assert.equal(exceptions.length, 0, `browser exceptions: ${exceptions.join('; ')}`);
await command('Target.closeTarget', { targetId });
socket.close();
