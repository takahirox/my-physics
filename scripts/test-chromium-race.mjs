import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';

const debugPort = process.env.CHROME_DEBUG_PORT || '9226';
const baseUrl = process.env.DEMO_URL || 'http://127.0.0.1:8093/';
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
await command('Page.navigate', { url: baseUrl }, sessionId);
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
const overlayLayout = await evaluate(`(() => {
  const canvas = document.querySelector('#track');
  const panel = document.querySelector('aside');
  const canvasRect = canvas.getBoundingClientRect();
  const panelRect = panel.getBoundingClientRect();
  return {
    viewport: [innerWidth, innerHeight],
    canvas: [canvasRect.left, canvasRect.top, canvasRect.width, canvasRect.height],
    panel: [panelRect.left, panelRect.top, panelRect.width, panelRect.height],
    panelPosition: getComputedStyle(panel).position,
  };
})()`);
assert(Math.abs(overlayLayout.canvas[0]) < 1 && Math.abs(overlayLayout.canvas[1]) < 1, 'canvas did not start at the window origin');
assert(Math.abs(overlayLayout.canvas[2] - overlayLayout.viewport[0]) < 1, 'canvas did not fit the window width');
assert(Math.abs(overlayLayout.canvas[3] - overlayLayout.viewport[1]) < 1, 'canvas did not fit the window height');
assert.equal(overlayLayout.panelPosition, 'absolute', 'telemetry panel was not an overlay');
assert(overlayLayout.panel[0] > 0 && overlayLayout.panel[0] + overlayLayout.panel[2] <= overlayLayout.viewport[0], 'telemetry overlay escaped the viewport');
await command('Emulation.setDeviceMetricsOverride', { width: 1024, height: 640, deviceScaleFactor: 1, mobile: false }, sessionId);
await sleep(50);
const resizedCanvas = await evaluate(`(() => {
  const rect = document.querySelector('#track').getBoundingClientRect();
  return [innerWidth, innerHeight, rect.left, rect.top, rect.width, rect.height];
})()`);
assert.deepEqual(resizedCanvas, [1024, 640, 0, 0, 1024, 640], 'canvas did not follow a live window resize');
await command('Emulation.clearDeviceMetricsOverride', {}, sessionId);
assert.equal(await evaluate('window.__MY_PHYSICS_RACE__.phase'), 'countdown');
assert.equal(await evaluate('window.__MY_PHYSICS_RACE__.standings.length'), 10);
for (let retry = 0; retry < 30; retry += 1) {
  if (await evaluate('(window.__MY_PHYSICS_FRAME__?.trackElevationRangeM || 0) > 20')) break;
  await sleep(50);
}
const elevationRangeM = await evaluate('window.__MY_PHYSICS_FRAME__.trackElevationRangeM');
assert(elevationRangeM > 20, `physical circuit elevation was not exported: ${elevationRangeM}; ${exceptions.join('; ')}`);
const staged = await evaluate('({time:window.__MY_PHYSICS_FRAME__.simulationTime, position:window.__MY_PHYSICS_FRAME__.playerPosition})');
await sleep(1_000);
const held = await evaluate('({phase:window.__MY_PHYSICS_RACE__.phase, position:window.__MY_PHYSICS_FRAME__.playerPosition})');
assert.equal(held.phase, 'countdown');
assert(Math.hypot(held.position[0] - staged.position[0], held.position[2] - staged.position[2]) < 0.5, 'grid was not held by normal brakes');
for (let retry = 0; retry < 50; retry += 1) {
  if (await evaluate("window.__MY_PHYSICS_RACE__.phase === 'racing'")) break;
  await sleep(100);
}
assert.equal(await evaluate('window.__MY_PHYSICS_RACE__.phase'), 'racing');
await evaluate("dispatchEvent(new KeyboardEvent('keydown',{code:'KeyW'}))");
await sleep(Number(process.env.RACE_DRIVE_MS || 1_800));
await evaluate("dispatchEvent(new KeyboardEvent('keyup',{code:'KeyW'}))");
const result = await evaluate(`({
  speedKmh:Number(document.querySelector('#speed').textContent),
  frame:window.__MY_PHYSICS_FRAME__,
  race:window.__MY_PHYSICS_RACE__,
  benchmark:window.__MY_PHYSICS_BENCHMARK__,
})`);
assert(result.speedKmh > 15, 'player did not accelerate through the common plant');
assert(Number.isFinite(result.frame.playerPosition[1]) && Math.abs(result.frame.playerPosition[1]) > 0.15, 'player physical Y was invalid');
assert(result.frame.drawCalls <= 3, `unexpected draw-call count ${result.frame.drawCalls}`);
assert(result.benchmark.realtime > 1, 'ten-vehicle browser physics missed real time');
assert.equal(exceptions.length, 0, `browser exceptions: ${exceptions.join('; ')}`);
console.log(JSON.stringify({ browser: version.Browser, overlayLayout, result, exceptions }, null, 2));
if (process.env.RACE_SCREENSHOT) {
  const screenshot = await command('Page.captureScreenshot', { format: 'png', fromSurface: true }, sessionId);
  await writeFile(process.env.RACE_SCREENSHOT, Buffer.from(screenshot.data, 'base64'));
}
await command('Target.closeTarget', { targetId });
socket.close();
