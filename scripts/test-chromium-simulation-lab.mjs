import assert from 'node:assert/strict';

const debugPort = process.env.CHROME_DEBUG_PORT || '9225';
const baseUrl = process.env.DEMO_URL || 'http://127.0.0.1:8092/';
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
const sleep = (milliseconds) => new Promise((resolve) => setTimeout(resolve, milliseconds));
async function evaluate(expression) {
  const result = await command('Runtime.evaluate', { expression, awaitPromise: true, returnByValue: true }, sessionId);
  if (result.exceptionDetails) throw new Error(result.exceptionDetails.text);
  return result.result.value;
}
async function navigate(url) {
  await command('Page.navigate', { url }, sessionId);
  for (let retry = 0; retry < 120; retry += 1) {
    if (await evaluate("document.querySelector('#status')?.classList.contains('ready') || false")) return;
    await sleep(100);
  }
  throw new Error(`page did not become ready: ${url}`);
}

await navigate(`${baseUrl}?demo=simulation-lab`);
assert.equal(await evaluate("document.querySelector('h1').textContent"), 'Simulation Validation Lab');
assert.equal(await evaluate('window.__MY_PHYSICS_INPUT__.demoVehiclePreset'), 'engineering_reference');
assert.equal(await evaluate('window.__MY_PHYSICS_INPUT__.experienceProfile'), 'simulation');
assert.equal(await evaluate("document.querySelector('#driveProfile').disabled"), true);
assert.match(await evaluate("document.querySelector('#performance').closest('.metric').querySelector('label').textContent"), /1-CAR/);

const iterations = [];
for (const scenario of [2, 4, 5]) {
  const report = await evaluate(`window.__MY_PHYSICS_LAB__.runScenario(${scenario}, true)`);
  assert.equal(report.passed, true);
  assert.equal(await evaluate('window.__MY_PHYSICS_LAB__.replayMatch'), true);
  assert(report.samples.length > 100);
  assert(report.samples[0].wheelSlip.length === 4 && report.samples[0].wheelLoad.length === 4);
  iterations.push({ scenario: report.scenario.id, fingerprint: report.fingerprint, checks: report.checks.length });
}

await evaluate("document.querySelector('#labScenario').value='2'; document.querySelector('#labReplay').click()");
assert.equal(await evaluate('window.__MY_PHYSICS_LAB__.midpointReplayMatch'), true);
await evaluate("document.querySelector('#labScenario').value='5'; document.querySelector('#labReplay').click()");
assert.equal(await evaluate('window.__MY_PHYSICS_LAB__.midpointReplayMatch'), true);
const allReports = await evaluate('window.__MY_PHYSICS_LAB__.runAll()');
assert.equal(allReports.length, 6);
assert(allReports.every((report) => report.passed));

// Persisting shared calibration while Lab is active must retain the normal
// circuit's pre-existing Sport profile rather than saving Lab's Raw profile.
await evaluate(`localStorage.setItem('my-physics.input-config.v1', JSON.stringify({driveProfile:'sport', steeringCenter:0})); document.querySelector('#resetCalibration').click()`);
assert.equal(await evaluate("JSON.parse(localStorage.getItem('my-physics.input-config.v1')).driveProfile"), 'sport');
await navigate(`${baseUrl}?demo=arcade`);
assert.equal(await evaluate('window.__MY_PHYSICS_INPUT__.demoVehiclePreset'), 'arcade_fun');
assert.equal(await evaluate('window.__MY_PHYSICS_INPUT__.experienceProfile'), 'arcade');
await navigate(baseUrl);
assert.equal(await evaluate('window.__MY_PHYSICS_INPUT__.demoVehiclePreset'), 'race_gameplay');
assert.equal(await evaluate('window.__MY_PHYSICS_INPUT__.experienceProfile'), 'sport');

console.log(JSON.stringify({ browser: version.Browser, iterations, catalogCount: allReports.length, exceptions }, null, 2));
assert.equal(exceptions.length, 0, `browser exceptions: ${exceptions.join('; ')}`);
await command('Target.closeTarget', { targetId });
socket.close();
