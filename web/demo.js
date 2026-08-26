import {
  CAMERA_PRESET_ORDER,
  VISUAL_CUES,
  cameraTelemetryResponse,
  cameraSettings,
  metricIntervals,
  metricSamples,
} from './visual-config.mjs';
import {
  DEFAULT_INPUT_CONFIG,
  INPUT_CONFIG_STORAGE_KEY,
  ARCADE_PROFILE_STORAGE_KEY,
  DeviceActivityLatch,
  captureRestCalibration,
  inputActivityMagnitude,
  inputConfigFromSources,
  inputConfigForDevice,
  normalizeCenteredAxis,
  normalizePedalAxis,
  sharedInputConfigForPersistence,
} from './input-config.mjs';
import { RaceDirector, RACE_PHASE, formatRaceTime } from './race-state.mjs';
import { TelemetryAudioEngine } from './audio-engine.mjs';
import { EFFECT_LIMITS, classifyDriftOutcome, effectEmissionRates } from './presentation-config.mjs';

const status = document.querySelector('#status');
const canvas = document.querySelector('#track');
const inputParameters = new URLSearchParams(location.search);
const ARCADE_DEMO = inputParameters.get('demo') === 'arcade';
const SIMULATION_LAB = inputParameters.get('demo') === 'simulation-lab';
const DRIFT_PLAYGROUND = ARCADE_DEMO && inputParameters.get('playground') === 'drift';
const DRIVE_PROFILES = ['accessible', 'sport', 'simulation', 'arcade'];
const PROFILE_INDEX = Object.freeze({ accessible: 0, sport: 1, simulation: 2, arcade: 3 });
if (ARCADE_DEMO) {
  document.body.classList.add('arcade-demo');
  document.querySelector('h1').textContent = 'Arcade Fun Circuit';
  document.querySelector('.eyebrow').textContent = 'SAME RUST PLANT · AUTHORED ARCADE-FUN-V1 · 1000 HZ';
  document.querySelector('.legend').textContent = 'ARCADE · WASD / arrows · Space: handbrake drift · C: camera · I: Arcade / Simulation raw · R: reset · P: AI';
}
if (DRIFT_PLAYGROUND) {
  document.body.classList.add('drift-playground');
  document.querySelector('h1').textContent = 'Keyboard Drift Playground';
  document.querySelector('.eyebrow').textContent = 'ARCADE INPUT ASSIST · SAME PHYSICAL PLANT · REPEATABLE SHORT COURSE';
  document.querySelector('.legend').textContent = 'DRIFT TEST · A/D: turn intent · Space: physical handbrake · W: throttle · R/Enter: restart entry';
  document.querySelector('#driftHud').hidden = false;
  const driftPanel = document.querySelector('#driftPanel');
  driftPanel.hidden = false;
  document.querySelector('aside').prepend(driftPanel);
}
if (SIMULATION_LAB) {
  document.body.classList.add('simulation-lab');
  document.querySelector('h1').textContent = 'Simulation Validation Lab';
  document.querySelector('.eyebrow').textContent = 'ENGINEERING REFERENCE · FIXED 1000 HZ · RUST VALIDATION CATALOG';
  document.querySelector('.legend').textContent = 'FREE DRIVE · SIMULATION RAW · WASD / arrows · K/L snapshot · validation maneuvers run headlessly in the same WASM plant';
  document.querySelector('#labPanel').hidden = false;
  document.querySelector('#lap').closest('.metric').querySelector('label').textContent = 'ENVIRONMENT';
  document.querySelector('#performance').closest('.metric').querySelector('label').textContent = '1-CAR WASM BENCH';
  const profileSelect = document.querySelector('#driveProfile');
  profileSelect.innerHTML = '<option value="simulation">SIMULATION · RAW · FIXED FOR LAB</option>';
  profileSelect.disabled = true;
}
const ui = Object.fromEntries(
  ['speed', 'rpm', 'gear', 'time', 'lap', 'trackLength', 'lod', 'damage', 'damageText', 'tires', 'performance', 'inputDevice', 'ffb', 'snapshotStatus', 'quality', 'cameraPreset', 'driveProfile', 'keyboardPolicy', 'inputResponse', 'calibrationStatus'].map(
    (id) => [id, document.querySelector(`#${id}`)],
  ),
);
const keys = new Set();
let api;
let previous = performance.now();
let accumulator = 0;
let gear = 0;
let cameraPreset = 'chase';
const raceDirector = SIMULATION_LAB || DRIFT_PLAYGROUND ? null : new RaceDirector({ totalLaps: 3 });
let raceView = null;
let savedRaceState = null;
const audioEngine = new TelemetryAudioEngine();
let audioMuted = false;
let audioEnabled = false;
let previousGamepadStart = false;
let greenUntilPhysicsTime = 0;
const raceUi = Object.fromEntries(
  ['raceHud', 'racePosition', 'raceField', 'raceLap', 'raceTime', 'raceBestLap', 'raceCountdown', 'raceCountdownLabel', 'raceResults', 'raceResultSummary', 'raceResultsList', 'raceRestart'].map(
    (id) => [id, document.querySelector(`#${id}`)],
  ),
);

function vehicleProgresses() {
  return Array.from({ length: api.physics_vehicle_count() }, (_, index) => api.physics_track_progress(index));
}

function resetRace() {
  if (DRIFT_PLAYGROUND) api.physics_arcade_drift_playground_reset();
  else api.physics_reset();
  api.physics_set_experience_profile(PROFILE_INDEX[inputConfig.driveProfile] ?? 1);
  gear = 0;
  accumulator = 0;
  savedRaceState = null;
  if (raceDirector) {
    raceView = raceDirector.reset(api.physics_time(), vehicleProgresses());
    greenUntilPhysicsTime = 0;
    api.physics_set_race_running?.(0);
  }
  if (renderer) {
    renderer.eye = null;
    renderer.resetEffects();
  }
}

function updateAudioButton() {
  const button = document.querySelector('#audioToggle');
  if (button) button.textContent = !audioEnabled ? 'ENABLE · M' : audioMuted ? 'MUTED · M' : 'ON · M';
}

async function enableAudio() {
  if (audioEnabled) return true;
  if (await audioEngine.unlock()) {
    audioEnabled = true;
    audioMuted = false;
    audioEngine.setMuted(false);
    updateAudioButton();
    return true;
  }
  return false;
}

async function toggleAudio() {
  if (!audioEnabled) return enableAudio();
  audioMuted = !audioMuted;
  audioEngine.setMuted(audioMuted);
  updateAudioButton();
}

function updateRaceState() {
  if (!raceDirector) return;
  const next = raceDirector.update(api.physics_time(), vehicleProgresses());
  if (next.events.some(({ type }) => type === 'race-started')) {
    api.physics_set_race_running?.(1);
    greenUntilPhysicsTime = api.physics_time() + 0.8;
  }
  raceView = next;
  window.__MY_PHYSICS_RACE__ = {
    ...raceView,
    physicsStep: api.physics_step_index(),
    restart: resetRace,
  };
}
const storedInputConfig = (() => {
  try {
    return localStorage.getItem(INPUT_CONFIG_STORAGE_KEY) || '';
  } catch {
    return '';
  }
})();
let inputConfig = inputConfigFromSources(inputParameters, storedInputConfig);
// Capture the shared circuit profile before URL/demo overrides are applied.
// Visiting either specialized demo can therefore never poison it.
const sharedDriveProfile = inputConfigFromSources('', storedInputConfig).driveProfile;
if (ARCADE_DEMO && !inputParameters.has('driveProfile')) {
  let arcadeProfile = 'arcade';
  try {
    const stored = localStorage.getItem(ARCADE_PROFILE_STORAGE_KEY);
    if (['accessible', 'sport', 'simulation', 'arcade'].includes(stored)) arcadeProfile = stored;
  } catch {
    // The default remains valid without storage access.
  }
  inputConfig = { ...inputConfig, driveProfile: arcadeProfile, keyboardAdaptive: arcadeProfile !== 'simulation' };
}
if (SIMULATION_LAB) inputConfig = { ...inputConfig, driveProfile: 'simulation', keyboardAdaptive: false };

function persistInputConfig() {
  try {
    const shared = sharedInputConfigForPersistence(inputConfig, sharedDriveProfile, ARCADE_DEMO || SIMULATION_LAB);
    localStorage.setItem(INPUT_CONFIG_STORAGE_KEY, JSON.stringify(shared));
    if (ARCADE_DEMO) localStorage.setItem(ARCADE_PROFILE_STORAGE_KEY, inputConfig.driveProfile);
  } catch {
    // Sandboxed/private browsers may deny persistence; the active config
    // remains valid for this session and URL parameters still work.
  }
}

function setDriveProfile(requested, persist = true) {
  const driveProfile = SIMULATION_LAB
    ? 'simulation'
    : DRIVE_PROFILES.includes(requested) ? requested : (ARCADE_DEMO ? 'arcade' : 'sport');
  const keyboardAdaptive = driveProfile !== 'simulation';
  inputConfig = { ...inputConfig, driveProfile, keyboardAdaptive };
  if (ui.driveProfile) ui.driveProfile.value = driveProfile;
  if (ui.keyboardPolicy) ui.keyboardPolicy.value = keyboardAdaptive ? 'adaptive' : 'raw';
  if (api) api.physics_set_experience_profile(PROFILE_INDEX[driveProfile]);
  if (persist) persistInputConfig();
}

function setKeyboardAdaptive(enabled, persist = true) {
  setDriveProfile(enabled ? (inputConfig.driveProfile === 'accessible' ? 'accessible' : ARCADE_DEMO ? 'arcade' : 'sport') : 'simulation', persist);
}

function selectCameraPreset(name) {
  cameraPreset = CAMERA_PRESET_ORDER.includes(name) ? name : 'chase';
  if (ui.cameraPreset) ui.cameraPreset.value = cameraPreset;
  if (renderer) renderer.eye = null;
}

addEventListener('keydown', (event) => {
  keys.add(event.code);
  if (event.code === 'KeyM' && !event.repeat) toggleAudio();
  else enableAudio();
  if (/^Digit[1-6]$/.test(event.code)) gear = Number(event.code.at(-1));
  if (event.code === 'KeyT') gear = 0;
  if ((event.code === 'KeyR' || event.code === 'Enter') && api && !event.repeat) resetRace();
  if (event.code === 'KeyP' && api && !event.repeat) {
    api.physics_set_player_autopilot(api.physics_player_autopilot() ? 0 : 1);
  }
  if (event.code === 'KeyE' && api && !event.repeat) {
    api.physics_set_player_esc(api.physics_player_esc() ? 0 : 1);
  }
  if (event.code === 'KeyI' && api && !event.repeat) {
    setDriveProfile(inputConfig.driveProfile === 'simulation' ? (ARCADE_DEMO ? 'arcade' : 'sport') : 'simulation');
  }
  if (event.code === 'KeyC' && !event.repeat) {
    const current = CAMERA_PRESET_ORDER.indexOf(cameraPreset);
    selectCameraPreset(CAMERA_PRESET_ORDER[(current + 1) % CAMERA_PRESET_ORDER.length]);
  }
  if (event.code === 'KeyK' && api) {
    const bytes = api.physics_snapshot_save();
    savedRaceState = raceDirector?.snapshot() ?? null;
    ui.snapshotStatus.textContent = `SAVED · ${(bytes / 1024).toFixed(0)} KiB`;
  }
  if (event.code === 'KeyL' && api) {
    const restored = api.physics_snapshot_restore();
    if (restored && raceDirector && savedRaceState) raceView = raceDirector.restore(savedRaceState);
    ui.snapshotStatus.textContent = restored ? 'RESTORED' : 'NO SNAPSHOT';
  }
});
addEventListener('keyup', (event) => keys.delete(event.code));

class InputAdapter {
  constructor() {
    this.parameters = inputParameters;
    this.activity = new DeviceActivityLatch();
    this.lastRumble = 0;
    this.activePad = null;
  }

  axis(pad, name, fallback, missing) {
    const index = Number(this.parameters.get(`${name}Axis`) ?? fallback);
    return pad.axes[index] ?? missing;
  }

  padSample(pad) {
    const isWheel = /wheel|g29|g920|g923|t150|t248|t300|t500|fanatec|moza|simagic/i.test(pad.id);
    const deviceConfig = inputConfigForDevice(inputConfig, pad.id);
    const raw = {
      steer: this.axis(pad, 'steer', 0, deviceConfig.steeringCenter),
      throttle: isWheel ? this.axis(pad, 'throttle', 1, deviceConfig.throttleReleased) : pad.buttons[7]?.value || 0,
      brake: isWheel ? this.axis(pad, 'brake', 2, deviceConfig.brakeReleased) : pad.buttons[6]?.value || 0,
      clutch: isWheel ? this.axis(pad, 'clutch', 3, deviceConfig.clutchReleased) : pad.buttons[4]?.value || 0,
      handbrake: pad.buttons[0]?.value || 0,
    };
    const normalized = {
      steer: normalizeCenteredAxis(raw.steer, deviceConfig, isWheel),
      throttle: isWheel ? normalizePedalAxis(raw.throttle, deviceConfig.throttleReleased, deviceConfig.throttlePressed) : raw.throttle,
      brake: isWheel ? normalizePedalAxis(raw.brake, deviceConfig.brakeReleased, deviceConfig.brakePressed) : raw.brake,
      clutch: isWheel ? normalizePedalAxis(raw.clutch, deviceConfig.clutchReleased, deviceConfig.clutchPressed) : raw.clutch,
      handbrake: raw.handbrake,
    };
    return {
      key: `pad:${pad.index}`,
      raw,
      normalized,
      magnitude: inputActivityMagnitude(normalized),
      device: `${isWheel ? 'WHEEL' : 'GAMEPAD'} · ${pad.id}`,
      deviceKind: isWheel ? 3 : 2,
      pad,
      isWheel,
    };
  }

  read(nowMs = performance.now()) {
    const keyboardSteer =
      (keys.has('ArrowLeft') || keys.has('KeyA') ? -1 : 0) + (keys.has('ArrowRight') || keys.has('KeyD') ? 1 : 0);
    const keyboard = {
      key: 'keyboard',
      raw: {
        steer: keyboardSteer,
        throttle: keys.has('ArrowUp') || keys.has('KeyW') ? 1 : 0,
        brake: keys.has('ArrowDown') || keys.has('KeyS') ? 1 : 0,
        clutch: keys.has('ShiftLeft') || keys.has('ShiftRight') ? 1 : 0,
        handbrake: keys.has('Space') ? 1 : 0,
      },
      device: 'KEYBOARD',
      deviceKind: 1,
      pad: null,
      priority: true,
    };
    keyboard.normalized = { ...keyboard.raw };
    keyboard.magnitude = inputActivityMagnitude(keyboard.normalized);
    const pads = [...(navigator.getGamepads?.() || [])].filter(Boolean).map((pad) => this.padSample(pad));
    const candidates = [keyboard, ...pads];
    if (!candidates.some(({ key }) => key === this.activity.active)) {
      this.activity.active = 'keyboard';
    }
    const activeKey = this.activity.select(candidates, nowMs);
    const active = candidates.find(({ key }) => key === activeKey) || keyboard;
    this.activePad = active.pad;
    return {
      steer: active.normalized.steer,
      keyboardSteer: keyboard.normalized.steer,
      keyboardSteering: active.deviceKind === 1,
      throttle: active.normalized.throttle,
      brake: active.normalized.brake,
      clutch: active.normalized.clutch,
      handbrake: active.normalized.handbrake,
      raw: active.raw,
      normalized: active.normalized,
      device: active.device,
      deviceKind: active.deviceKind,
      pad: active.pad,
    };
  }

  captureRest() {
    const pad = this.activePad || [...(navigator.getGamepads?.() || [])].find(Boolean);
    if (!pad) return null;
    return captureRestCalibration(inputConfig, pad, {
      steering: this.axis(pad, 'steer', 0, 0),
      throttle: this.axis(pad, 'throttle', 1, 1),
      brake: this.axis(pad, 'brake', 2, 1),
      clutch: this.axis(pad, 'clutch', 3, 1),
    });
  }

  rumble(pad, magnitude, now) {
    if (!pad?.vibrationActuator || now - this.lastRumble < 80 || magnitude < 0.08) return;
    this.lastRumble = now;
    pad.vibrationActuator
      .playEffect('dual-rumble', {
        duration: 90,
        weakMagnitude: Math.min(1, magnitude),
        strongMagnitude: Math.min(1, magnitude * 0.65),
      })
      .catch(() => {});
  }
}

const inputAdapter = new InputAdapter();

const vertexShader = `#version 300 es
precision highp float;
layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
uniform mat4 viewProjection;
uniform mat4 model;
out vec3 worldNormal;
out vec3 worldPosition;
void main() {
  vec4 world = model * vec4(position, 1.0);
  worldPosition = world.xyz;
  worldNormal = normalize(mat3(model) * normal);
  gl_Position = viewProjection * world;
}`;

const fragmentShader = `#version 300 es
precision highp float;
uniform vec4 color;
uniform vec3 cameraPosition;
in vec3 worldNormal;
in vec3 worldPosition;
out vec4 outputColor;
void main() {
  vec3 sun = normalize(vec3(-0.35, 0.9, 0.28));
  float diffuse = 0.34 + 0.66 * max(dot(normalize(worldNormal), sun), 0.0);
  float distanceToCamera = length(worldPosition - cameraPosition);
  float fog = smoothstep(95.0, 260.0, distanceToCamera);
  vec3 lit = color.rgb * diffuse;
  outputColor = vec4(mix(lit, vec3(0.055, 0.085, 0.065), fog), color.a);
}`;

const instancedVertexShader = `#version 300 es
precision highp float;
layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
layout(location=2) in vec4 model0;
layout(location=3) in vec4 model1;
layout(location=4) in vec4 model2;
layout(location=5) in vec4 model3;
layout(location=6) in vec4 instanceColor;
uniform mat4 viewProjection;
out vec3 worldNormal;
out vec3 worldPosition;
out vec4 boxColor;
void main() {
  mat4 model = mat4(model0, model1, model2, model3);
  vec4 world = model * vec4(position, 1.0);
  worldPosition = world.xyz;
  worldNormal = normalize(mat3(model) * normal);
  boxColor = instanceColor;
  gl_Position = viewProjection * world;
}`;

const instancedFragmentShader = `#version 300 es
precision highp float;
uniform vec3 cameraPosition;
in vec3 worldNormal;
in vec3 worldPosition;
in vec4 boxColor;
out vec4 outputColor;
void main() {
  vec3 sun = normalize(vec3(-0.35, 0.9, 0.28));
  float diffuse = 0.34 + 0.66 * max(dot(normalize(worldNormal), sun), 0.0);
  float distanceToCamera = length(worldPosition - cameraPosition);
  float fog = smoothstep(95.0, 260.0, distanceToCamera);
  vec3 lit = boxColor.rgb * diffuse;
  outputColor = vec4(mix(lit, vec3(0.055, 0.085, 0.065), fog), boxColor.a);
}`;

function compileShader(gl, type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(shader));
  return shader;
}

function createProgram(gl, vertexSource = vertexShader, fragmentSource = fragmentShader) {
  const program = gl.createProgram();
  gl.attachShader(program, compileShader(gl, gl.VERTEX_SHADER, vertexSource));
  gl.attachShader(program, compileShader(gl, gl.FRAGMENT_SHADER, fragmentSource));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program));
  return program;
}

function cubeGeometry() {
  const faces = [
    [[1, 0, 0], [[1, -1, -1], [1, 1, -1], [1, 1, 1], [1, -1, 1]]],
    [[-1, 0, 0], [[-1, -1, 1], [-1, 1, 1], [-1, 1, -1], [-1, -1, -1]]],
    [[0, 1, 0], [[-1, 1, -1], [-1, 1, 1], [1, 1, 1], [1, 1, -1]]],
    [[0, -1, 0], [[-1, -1, 1], [-1, -1, -1], [1, -1, -1], [1, -1, 1]]],
    [[0, 0, 1], [[1, -1, 1], [1, 1, 1], [-1, 1, 1], [-1, -1, 1]]],
    [[0, 0, -1], [[-1, -1, -1], [-1, 1, -1], [1, 1, -1], [1, -1, -1]]],
  ];
  const vertices = [];
  for (const [normal, corners] of faces) {
    for (const index of [0, 1, 2, 0, 2, 3]) vertices.push(...corners[index].map((value) => value * 0.5), ...normal);
  }
  return new Float32Array(vertices);
}

function perspective(fieldOfView, aspect, near, far) {
  const f = 1 / Math.tan(fieldOfView / 2);
  const range = 1 / (near - far);
  return new Float32Array([f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) * range, -1, 0, 0, 2 * far * near * range, 0]);
}

function normalize(vector) {
  const length = Math.hypot(...vector) || 1;
  return vector.map((value) => value / length);
}

function cross(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function lookAt(eye, center, worldUp = [0, 1, 0]) {
  const z = normalize(eye.map((value, index) => value - center[index]));
  const x = normalize(cross(worldUp, z));
  const y = cross(z, x);
  return new Float32Array([
    x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0, -dot(x, eye), -dot(y, eye), -dot(z, eye), 1,
  ]);
}

function multiply(a, b) {
  const result = new Float32Array(16);
  for (let column = 0; column < 4; column++) {
    for (let row = 0; row < 4; row++) {
      result[column * 4 + row] =
        a[row] * b[column * 4] + a[4 + row] * b[column * 4 + 1] + a[8 + row] * b[column * 4 + 2] + a[12 + row] * b[column * 4 + 3];
    }
  }
  return result;
}

function frameFromYaw(yaw = 0) {
  const cosine = Math.cos(yaw);
  const sine = Math.sin(yaw);
  return { right: [cosine, 0, -sine], up: [0, 1, 0], forward: [-sine, 0, -cosine] };
}

function rotateByQuaternion(vector, quaternion) {
  const [x, y, z] = vector;
  const { w, x: qx, y: qy, z: qz } = quaternion;
  const tx = 2 * (qy * z - qz * y);
  const ty = 2 * (qz * x - qx * z);
  const tz = 2 * (qx * y - qy * x);
  return [x + w * tx + (qy * tz - qz * ty), y + w * ty + (qz * tx - qx * tz), z + w * tz + (qx * ty - qy * tx)];
}

function frameFromQuaternion(quaternion) {
  return {
    right: normalize(rotateByQuaternion([1, 0, 0], quaternion)),
    up: normalize(rotateByQuaternion([0, 1, 0], quaternion)),
    forward: normalize(rotateByQuaternion([0, 0, -1], quaternion)),
  };
}

function asFrame(frame) {
  return typeof frame === 'number' ? frameFromYaw(frame) : frame;
}

function modelMatrix(position, scale, frame = 0) {
  const { right, up, forward } = asFrame(frame);
  return new Float32Array([
    right[0] * scale[0], right[1] * scale[0], right[2] * scale[0], 0,
    up[0] * scale[1], up[1] * scale[1], up[2] * scale[1], 0,
    -forward[0] * scale[2], -forward[1] * scale[2], -forward[2] * scale[2], 0,
    position[0], position[1], position[2], 1,
  ]);
}

function localPoint(position, frame, local) {
  const { right, up, forward } = asFrame(frame);
  return [
    position[0] + right[0] * local[0] + up[0] * local[1] - forward[0] * local[2],
    position[1] + right[1] * local[0] + up[1] * local[1] - forward[1] * local[2],
    position[2] + right[2] * local[0] + up[2] * local[1] - forward[2] * local[2],
  ];
}

function rgb(hex, alpha = 1) {
  const value = Number.parseInt(hex.slice(1), 16);
  return [((value >> 16) & 255) / 255, ((value >> 8) & 255) / 255, (value & 255) / 255, alpha];
}

class Renderer3D {
  constructor(target) {
    this.gl = target.getContext('webgl2', { antialias: true, alpha: false });
    if (!this.gl) throw new Error('WebGL2 is required');
    const gl = this.gl;
    this.canvas = target;
    this.program = createProgram(gl);
    this.viewProjection = gl.getUniformLocation(this.program, 'viewProjection');
    this.model = gl.getUniformLocation(this.program, 'model');
    this.color = gl.getUniformLocation(this.program, 'color');
    this.cameraPosition = gl.getUniformLocation(this.program, 'cameraPosition');
    this.instancedProgram = createProgram(gl, instancedVertexShader, instancedFragmentShader);
    this.instancedViewProjection = gl.getUniformLocation(this.instancedProgram, 'viewProjection');
    this.instancedCameraPosition = gl.getUniformLocation(this.instancedProgram, 'cameraPosition');
    this.vertexArray = gl.createVertexArray();
    gl.bindVertexArray(this.vertexArray);
    const buffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
    const geometry = cubeGeometry();
    this.vertexCount = geometry.length / 6;
    gl.bufferData(gl.ARRAY_BUFFER, geometry, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 24, 0);
    gl.enableVertexAttribArray(1);
    gl.vertexAttribPointer(1, 3, gl.FLOAT, false, 24, 12);
    this.instanceBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, this.instanceBuffer);
    for (let column = 0; column < 4; column += 1) {
      const location = 2 + column;
      gl.enableVertexAttribArray(location);
      gl.vertexAttribPointer(location, 4, gl.FLOAT, false, 80, column * 16);
      gl.vertexAttribDivisor(location, 1);
    }
    gl.enableVertexAttribArray(6);
    gl.vertexAttribPointer(6, 4, gl.FLOAT, false, 80, 64);
    gl.vertexAttribDivisor(6, 1);
    gl.enable(gl.DEPTH_TEST);
    gl.enable(gl.CULL_FACE);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    this.eye = null;
    this.drawCalls = 0;
    this.instances = null;
    this.effects = [];
    this.effectCarry = { smoke: 0, spray: 0, sparks: 0 };
  }

  resize() {
    const bounds = this.canvas.getBoundingClientRect();
    const density = Math.min(devicePixelRatio, 2);
    const width = Math.round(bounds.width * density);
    const height = Math.round(bounds.height * density);
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
    this.gl.viewport(0, 0, width, height);
    return width / Math.max(height, 1);
  }

  box(position, scale, color, frame = 0) {
    if (this.instances) {
      this.instances.push(...modelMatrix(position, scale, frame), ...color);
      return;
    }
    const gl = this.gl;
    gl.uniformMatrix4fv(this.model, false, modelMatrix(position, scale, frame));
    gl.uniform4fv(this.color, color);
    gl.drawArrays(gl.TRIANGLES, 0, this.vertexCount);
    this.drawCalls += 1;
  }

  beginBatch() {
    this.instances = [];
  }

  flushBatch(viewProjection, cameraPosition) {
    const instanceCount = this.instances.length / 20;
    if (!instanceCount) {
      this.instances = null;
      return;
    }
    const gl = this.gl;
    gl.useProgram(this.instancedProgram);
    gl.bindVertexArray(this.vertexArray);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.instanceBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(this.instances), gl.DYNAMIC_DRAW);
    gl.uniformMatrix4fv(this.instancedViewProjection, false, viewProjection);
    gl.uniform3fv(this.instancedCameraPosition, cameraPosition);
    gl.drawArraysInstanced(gl.TRIANGLES, 0, this.vertexCount, instanceCount);
    this.drawCalls += 1;
    this.instances = null;
  }

  localPoint(position, frame, local) {
    return localPoint(position, frame, local);
  }

  car(position, frame, color, player, steering = 0) {
    this.box(this.localPoint(position, frame, [0, -0.53, 0.12]), [2.05, 0.025, 4.65], rgb('#020503', 0.38), frame);
    this.box(this.localPoint(position, frame, [0, 0, 0.12]), [1.82, 0.48, 4.22], color, frame);
    this.box(this.localPoint(position, frame, [0, 0.45, 0.35]), [1.48, 0.54, 1.82], rgb(player ? '#26361d' : '#17201c'), frame);
    this.box(this.localPoint(position, frame, [0, 0.75, 0.31]), [1.34, 0.12, 1.25], rgb('#80a39c'), frame);
    for (const x of [-0.94, 0.94]) {
      for (const z of [-1.35, 1.35]) {
        const wheelFrame = z >= 0 ? frame : {
          ...frame,
          right: normalize(frame.right.map((value, axis) => value * Math.cos(steering) - frame.forward[axis] * Math.sin(steering))),
          forward: normalize(frame.forward.map((value, axis) => value * Math.cos(steering) + frame.right[axis] * Math.sin(steering))),
        };
        this.box(this.localPoint(position, frame, [x, -0.05, z]), [0.25, 0.62, 0.72], rgb('#070907'), wheelFrame);
      }
    }
    for (const x of [-0.57, 0.57]) {
      this.box(this.localPoint(position, frame, [x, 0.02, -2.14]), [0.36, 0.16, 0.05], rgb('#e8f7d1'), frame);
      this.box(this.localPoint(position, frame, [x, 0.02, 2.14]), [0.38, 0.16, 0.05], rgb('#ff3e28'), frame);
    }
  }

  resetEffects() {
    this.effects = [];
    this.effectCarry = { smoke: 0, spray: 0, sparks: 0 };
  }

  emitEffect(type, position, velocity, lifeS, size, color, cap) {
    const sameType = this.effects.reduce((count, effect) => count + Number(effect.type === type), 0);
    if (sameType >= cap) return;
    this.effects.push({ type, position: [...position], velocity: [...velocity], lifeS, ageS: 0, size, color });
  }

  updateEffects(elapsed, position, frame, telemetry, physicsStep) {
    for (const effect of this.effects) {
      effect.ageS += elapsed;
      for (let axis = 0; axis < 3; axis += 1) effect.position[axis] += effect.velocity[axis] * elapsed;
      if (effect.type !== 'spark') effect.velocity[1] += 0.35 * elapsed;
      else effect.velocity[1] -= 6.5 * elapsed;
    }
    this.effects = this.effects.filter((effect) => effect.ageS < effect.lifeS);
    const rates = effectEmissionRates(telemetry);
    const smokeRate = rates.smokePerSecond.reduce((sum, value) => sum + value, 0);
    const sprayRate = rates.sprayPerSecond.reduce((sum, value) => sum + value, 0);
    const emit = (type, rate, cap, factory) => {
      this.effectCarry[type] += rate * elapsed;
      const count = Math.min(10, Math.floor(this.effectCarry[type]));
      this.effectCarry[type] -= count;
      for (let index = 0; index < count; index += 1) factory(index, cap);
    };
    const phase = (physicsStep % 997) * 0.173;
    emit('smoke', smokeRate, EFFECT_LIMITS.smokeParticles, (index, cap) => {
      const side = (index + physicsStep) % 2 ? -0.92 : 0.92;
      const origin = this.localPoint(position, frame, [side, -0.28, 1.45]);
      this.emitEffect('smoke', origin, [Math.sin(phase + index) * 0.5, 1.0, Math.cos(phase + index) * 0.5], 1.2, 0.25, rgb('#d9ddd5', 0.46), cap);
    });
    emit('spray', sprayRate, EFFECT_LIMITS.sprayParticles, (index, cap) => {
      const side = (index + physicsStep) % 2 ? -0.92 : 0.92;
      const origin = this.localPoint(position, frame, [side, -0.3, 1.5]);
      const backwards = frame.forward.map((value) => -value * 5.5);
      this.emitEffect('spray', origin, [backwards[0] + Math.sin(phase + index), 2.4, backwards[2] + Math.cos(phase + index)], 0.65, 0.11, rgb('#a7e8ff', 0.65), cap);
    });
    emit('sparks', rates.sparksPerSecond, EFFECT_LIMITS.sparkParticles, (index, cap) => {
      const origin = this.localPoint(position, frame, [0, -0.35, 1.8]);
      this.emitEffect('spark', origin, [Math.sin(phase + index) * 4, 3 + index * 0.1, Math.cos(phase + index) * 4], 0.42, 0.08, rgb('#ffcf42'), cap);
    });
    for (const effect of this.effects) {
      const fade = Math.max(0, 1 - effect.ageS / effect.lifeS);
      this.box(effect.position, [effect.size * (1 + effect.ageS), effect.size * (1 + effect.ageS), effect.size * (1 + effect.ageS)], [effect.color[0], effect.color[1], effect.color[2], effect.color[3] * fade]);
    }
    return rates;
  }

  grandstand(position, frame, trackHalfWidth, side) {
    const baseX = side * (trackHalfWidth + 4.3);
    for (let row = 0; row < 5; row += 1) {
      const x = baseX + side * row * 0.9;
      this.box(this.localPoint(position, frame, [x, 0.4 + row * 0.48, 0]), [1.0, 0.75, 30], rgb(row % 2 ? '#303a36' : '#46534d'), frame);
      for (let seat = -13; seat <= 13; seat += 2) {
        const seatColor = Math.abs(seat + row) % 4 ? '#c5d0c8' : '#b9ef42';
        this.box(this.localPoint(position, frame, [x - side * 0.5, 0.88 + row * 0.48, seat]), [0.18, 0.16, 1.1], rgb(seatColor), frame);
      }
    }
    this.box(this.localPoint(position, frame, [baseX + side * 2.1, 3.4, 0]), [5.7, 0.22, 32], rgb('#c4cbc4'), frame);
    this.box(this.localPoint(position, frame, [baseX + side * 4.8, 1.7, 0]), [0.18, 3.4, 32], rgb('#6f7974'), frame);
  }

  landmarks(playerPosition, trackHalfWidth) {
    const visible = (segment) => Math.hypot(
      segment.position[0] - playerPosition[0],
      segment.position[1] - playerPosition[1],
      segment.position[2] - playerPosition[2],
    ) < 250;
    const tower = this.circuit[48];
    if (visible(tower)) {
      const base = this.localPoint(tower.position, tower.frame, [trackHalfWidth + 18, 0, 0]);
      for (let level = 0; level < 7; level += 1) {
        this.box(this.localPoint(base, tower.frame, [0, 1.6 + level * 3.0, 0]), [4.8 - level * 0.34, 2.7, 4.8 - level * 0.34], rgb(level % 2 ? '#ff7357' : '#ffe75a'), tower.frame);
      }
      this.box(this.localPoint(base, tower.frame, [0, 23.0, 0]), [1.0, 4.0, 1.0], rgb('#f7f0d4'), tower.frame);
    }
    const bridge = this.circuit[118];
    if (visible(bridge)) {
      for (const side of [-1, 1]) this.box(this.localPoint(bridge.position, bridge.frame, [side * (trackHalfWidth + 1.0), 3.4, -4]), [0.7, 6.8, 0.8], rgb('#224d73'), bridge.frame);
      this.box(this.localPoint(bridge.position, bridge.frame, [0, 6.5, -4]), [trackHalfWidth * 2 + 4, 0.75, 1.2], rgb('#48c7e8'), bridge.frame);
      this.box(this.localPoint(bridge.position, bridge.frame, [0, 7.15, -4]), [6.0, 0.18, 0.22], rgb('#fff15c'), bridge.frame);
    }
    for (const anchorIndex of [76, 82, 88, 174, 180, 186]) {
      const anchor = this.circuit[anchorIndex];
      if (!visible(anchor)) continue;
      for (const side of [-1, 1]) {
        for (let tree = 0; tree < 4; tree += 1) {
          const trunk = this.localPoint(anchor.position, anchor.frame, [side * (trackHalfWidth + 8 + tree * 3.1), 1.3, tree * 4 - 7]);
          this.box(trunk, [0.55, 2.6, 0.55], rgb('#694331'), anchor.frame);
          this.box(this.localPoint(trunk, anchor.frame, [0, 2.2, 0]), [3.2, 2.8, 3.2], rgb(tree % 2 ? '#46a84e' : '#66cc4d'), anchor.frame);
          this.box(this.localPoint(trunk, anchor.frame, [0, 4.0, 0]), [2.1, 1.9, 2.1], rgb('#92dd55'), anchor.frame);
        }
      }
    }
    const canyon = this.circuit[210];
    if (visible(canyon)) {
      for (const side of [-1, 1]) {
        for (let rock = 0; rock < 5; rock += 1) {
          this.box(
            this.localPoint(canyon.position, canyon.frame, [side * (trackHalfWidth + 10 + rock * 2.7), 2 + rock * 0.8, rock * 5 - 10]),
            [4.2 - rock * 0.25, 4 + rock * 1.6, 5.0],
            rgb(rock % 2 ? '#b75842' : '#dd7b4f'),
            canyon.frame,
          );
        }
      }
    }
  }

  raceCourse(physics, playerPosition, trackHalfWidth) {
    if (!this.circuit) {
      let cumulativeDistanceM = 0;
      this.circuit = Array.from({ length: physics.physics_track_segment_count() }, (_, index) => {
        const length = physics.physics_track_segment_length(index);
        const segment = {
          position: [physics.physics_track_segment_x(index), physics.physics_track_segment_y(index), physics.physics_track_segment_z(index)],
          yaw: physics.physics_track_segment_yaw(index),
          frame: {
            forward: [physics.physics_track_segment_forward_x(index), physics.physics_track_segment_forward_y(index), physics.physics_track_segment_forward_z(index)],
            right: [physics.physics_track_segment_right_x(index), physics.physics_track_segment_right_y(index), physics.physics_track_segment_right_z(index)],
            up: [physics.physics_track_segment_up_x(index), physics.physics_track_segment_up_y(index), physics.physics_track_segment_up_z(index)],
          },
          length,
          distanceStartM: cumulativeDistanceM,
          curbBands: metricIntervals(cumulativeDistanceM, length, VISUAL_CUES.curbBandM),
          fencePosts: metricSamples(cumulativeDistanceM, length, VISUAL_CUES.fencePostM),
          seams: metricSamples(cumulativeDistanceM, length, VISUAL_CUES.asphaltSeamM, 0.7),
          patches: metricSamples(cumulativeDistanceM, length, VISUAL_CUES.asphaltPatchM, 1.3),
          rubber: metricSamples(cumulativeDistanceM, length, VISUAL_CUES.rubberDashM, 0.9),
          boards: metricSamples(cumulativeDistanceM, length, 200, 100),
        };
        cumulativeDistanceM += length;
        return segment;
      });
      const elevations = this.circuit.map((segment) => segment.position[1]);
      this.trackElevationRangeM = Math.max(...elevations) - Math.min(...elevations);
    }
    const roadWidth = trackHalfWidth * 2;
    this.box([0, -10.2, 0], [760, 0.32, 760], rgb('#15391e'));

    for (let index = 0; index < this.circuit.length; index += 1) {
      const segment = this.circuit[index];
      const midpoint = this.localPoint(segment.position, segment.frame, [0, 0, -segment.length * 0.5]);
      const distance = Math.hypot(midpoint[0] - playerPosition[0], midpoint[1] - playerPosition[1], midpoint[2] - playerPosition[2]);
      if (distance > 235) continue;
      const joinLength = segment.length + 0.65;
      this.box(this.localPoint(midpoint, segment.frame, [0, -0.11, 0]), [roadWidth + 24, 0.16, joinLength], rgb('#31632e'), segment.frame);
      this.box(this.localPoint(midpoint, segment.frame, [0, -0.015, 0]), [roadWidth, 0.08, joinLength], rgb('#202522'), segment.frame);
      this.box(this.localPoint(midpoint, segment.frame, [-trackHalfWidth + 0.12, 0.05, 0]), [0.16, 0.025, joinLength], rgb('#f0f2ec'), segment.frame);
      this.box(this.localPoint(midpoint, segment.frame, [trackHalfWidth - 0.12, 0.05, 0]), [0.16, 0.025, joinLength], rgb('#f0f2ec'), segment.frame);
      for (const side of [-1, 1]) {
        this.box(this.localPoint(midpoint, segment.frame, [side * (trackHalfWidth + 0.3), 0.34, 0]), [0.6, 0.68, joinLength], rgb('#bcc3be'), segment.frame);
        this.box(this.localPoint(midpoint, segment.frame, [side * (trackHalfWidth + 0.62), 0.24, 0]), [0.035, 0.26, joinLength], rgb('#59615d'), segment.frame);
        this.box(this.localPoint(midpoint, segment.frame, [side * (trackHalfWidth + 0.72), 2.3, 0]), [0.055, 0.055, joinLength], rgb('#8c9691'), segment.frame);
      }
      if (distance <= VISUAL_CUES.detailRadiusM) {
        for (const band of segment.curbBands) {
          const curbColor = band.band % 2 ? rgb('#d9342b') : rgb('#f4f2e9');
          for (const side of [-1, 1]) {
            this.box(
              this.localPoint(segment.position, segment.frame, [side * (trackHalfWidth - 0.3), 0.07, -band.centerM]),
              [0.6, 0.1, band.lengthM + VISUAL_CUES.curbJoinOverlapM],
              curbColor,
              segment.frame,
            );
          }
        }
        for (const sample of segment.fencePosts) {
          for (const side of [-1, 1]) {
            const point = this.localPoint(segment.position, segment.frame, [side * (trackHalfWidth + 0.72), 1.45, -sample.localM]);
            this.box(point, [0.09, 2.9, 0.09], rgb('#77817c'), segment.frame);
          }
        }
        for (const sample of segment.seams) {
          const seamLane = ((sample.index * 3) % 5 - 2) * 1.45;
          this.box(
            this.localPoint(segment.position, segment.frame, [seamLane, 0.033, -sample.localM]),
            [2.1 + (Math.abs(sample.index) % 3) * 0.45, 0.012, 0.04],
            rgb('#151a17', 0.58),
            segment.frame,
          );
        }
        for (const sample of segment.patches) {
          const lane = ((sample.index % 4) - 1.5) * 1.35;
          this.box(
            this.localPoint(segment.position, segment.frame, [lane, 0.029, -sample.localM]),
            [1.05, 0.01, 1.35],
            rgb(sample.index % 3 ? '#252b27' : '#1b211e', 0.72),
            segment.frame,
          );
        }
        for (const sample of segment.rubber) {
          for (const rubber of [-0.88, 0.88]) {
            this.box(
              this.localPoint(segment.position, segment.frame, [rubber, 0.036, -sample.localM]),
              [0.09, 0.014, 2.1],
              rgb('#0e1210', 0.86),
              segment.frame,
            );
          }
        }
      }
      for (const sample of segment.boards) {
        const board = this.localPoint(segment.position, segment.frame, [trackHalfWidth + 1.25, 1.0, -sample.localM]);
        this.box(board, [0.1, 2.0, 0.1], rgb('#707873'), segment.frame);
        this.box(this.localPoint(board, segment.frame, [0, 1.05, 0]), [1.05, 0.75, 0.16], rgb('#f0f1eb'), segment.frame);
      }
    }

    this.landmarks(playerPosition, trackHalfWidth);

    const start = this.circuit[0];
    for (let row = 0; row < 2; row += 1) {
      for (let square = 0; square < 12; square += 1) {
        const local = [-trackHalfWidth + (square + 0.5) * (roadWidth / 12), 0.055, row * 0.55];
        const point = this.localPoint(start.position, start.frame, local);
        this.box(point, [roadWidth / 12 + 0.01, 0.025, 0.56], rgb((square + row) % 2 ? '#171b18' : '#f4f5ee'), start.frame);
      }
    }
    for (const side of [-1, 1]) this.box(this.localPoint(start.position, start.frame, [side * (trackHalfWidth + 0.72), 3.1, 0]), [0.32, 6.2, 0.42], rgb('#87918c'), start.frame);
    this.box(this.localPoint(start.position, start.frame, [0, 5.75, 0]), [roadWidth + 2.1, 0.55, 0.7], rgb('#171d19'), start.frame);
    this.box(this.localPoint(start.position, start.frame, [0, 5.72, 0.37]), [5.2, 0.28, 0.05], rgb('#b9ef42'), start.frame);
    for (let slot = 8; slot <= 44; slot += 8) {
      const lane = Math.floor(slot / 8) % 2 ? -1.65 : 1.65;
      this.box(this.localPoint(start.position, start.frame, [lane, 0.05, slot]), [2.25, 0.025, 0.09], rgb('#d8ddd7'), start.frame);
    }
    const stands = this.localPoint(start.position, start.frame, [0, 0, 24]);
    this.grandstand(stands, start.frame, trackHalfWidth, -1);
    this.grandstand(stands, start.frame, trackHalfWidth, 1);
  }

  laboratoryGround(playerPosition) {
    this.box([playerPosition[0], -0.22, playerPosition[2]], [520, 0.32, 520], rgb('#16252b'));
    const spacing = 20;
    const centerX = Math.round(playerPosition[0] / spacing) * spacing;
    const centerZ = Math.round(playerPosition[2] / spacing) * spacing;
    for (let offset = -200; offset <= 200; offset += spacing) {
      this.box([centerX + offset, 0.012, centerZ], [0.045, 0.014, 400], rgb('#34515b', 0.72));
      this.box([centerX, 0.013, centerZ + offset], [400, 0.014, 0.045], rgb('#34515b', 0.72));
    }
    for (let marker = -180; marker <= 180; marker += 20) {
      this.box([centerX - 3.5, 0.035, centerZ + marker], [0.18, 0.02, 2.5], rgb('#e8edf0'));
      this.box([centerX + 3.5, 0.035, centerZ + marker], [0.18, 0.02, 2.5], rgb('#e8edf0'));
    }
  }

  driftPlaygroundGround(playerPosition) {
    this.box([playerPosition[0], -0.24, playerPosition[2]], [520, 0.32, 520], rgb('#24452b'));
    this.box([0, -0.08, -90], [34, 0.12, 300], rgb('#202522'));
    for (const side of [-1, 1]) {
      this.box([side * 16.2, 0.018, -90], [0.22, 0.025, 300], rgb('#f1efe7'));
      for (let marker = -230; marker <= 50; marker += 10) {
        const color = Math.abs(marker / 10) % 2 ? '#d9342b' : '#f4f2e9';
        this.box([side * 15.7, 0.035, marker], [0.75, 0.05, 4.8], rgb(color));
      }
    }
    for (let marker = -230; marker <= 50; marker += 12) {
      this.box([0, 0.025, marker], [0.14, 0.025, 5.5], rgb('#d9ddd7', 0.55));
    }
    for (const gate of [-45, -90, -135, -180]) {
      for (const side of [-1, 1]) {
        this.box([side * 12.5, 0.42, gate], [0.42, 0.84, 0.42], rgb('#ffb72d'));
        this.box([side * 12.5, 1.05, gate], [0.25, 0.4, 0.25], rgb('#f4f2e9'));
      }
    }
  }

  scene(physics, elapsed, alpha) {
    const gl = this.gl;
    const aspect = this.resize();
    const x = physics.physics_render_x(0, alpha);
    const y = physics.physics_render_y(0, alpha);
    const z = physics.physics_render_z(0, alpha);
    const yaw = physics.physics_render_yaw(0, alpha);
    const playerQuaternion = {
      w: physics.physics_render_orientation_w(0, alpha),
      x: physics.physics_render_orientation_x(0, alpha),
      y: physics.physics_render_orientation_y(0, alpha),
      z: physics.physics_render_orientation_z(0, alpha),
    };
    const playerFrame = frameFromQuaternion(playerQuaternion);
    const speedMps = physics.physics_speed(0);
    const { forward, right, up } = playerFrame;
    const telemetry = readPresentationTelemetry(physics);
    const cameraResponse = cameraTelemetryResponse(telemetry);
    const camera = cameraSettings(cameraPreset, speedMps);
    const shakePhase = physics.physics_step_index() * 0.0618034;
    const shake = cameraResponse.shakeEnvelopeM * Math.sin(shakePhase);
    const desiredEye = [0, 1, 2].map((axis) => (
      [x, y, z][axis]
      - forward[axis] * (camera.backM - cameraResponse.longitudinalOffsetM)
      + up[axis] * (camera.heightM + shake)
      + right[axis] * (cameraResponse.lateralOffsetM + shake * 0.35)
    ));
    if (!this.eye) this.eye = desiredEye;
    const cameraBlend = 1 - Math.exp(-elapsed * camera.responsePerS);
    this.eye = this.eye.map((value, index) => value + (desiredEye[index] - value) * cameraBlend);
    const cameraError = desiredEye.map((value, index) => value - this.eye[index]);
    const unclampedCameraLag = Math.hypot(...cameraError);
    if (unclampedCameraLag > camera.maxLagM) {
      this.eye = desiredEye.map((value, index) => value - (cameraError[index] / unclampedCameraLag) * camera.maxLagM);
    }
    const cameraLag = Math.hypot(...desiredEye.map((value, index) => value - this.eye[index]));
    const pitchTarget = Math.tan(cameraResponse.pitchDeg * Math.PI / 180) * camera.targetAheadM;
    const target = [0, 1, 2].map((axis) => (
      [x, y, z][axis] + forward[axis] * camera.targetAheadM + up[axis] * (camera.targetHeightM + pitchTarget)
    ));
    const cameraUp = normalize([0, 1, 2].map((axis) => up[axis] - right[axis] * Math.tan(cameraResponse.rollDeg * Math.PI / 180)));
    const fieldOfViewDegrees = camera.fieldOfViewDegrees + cameraResponse.edgeStreak * 3.0;
    const projection = perspective((fieldOfViewDegrees * Math.PI) / 180, aspect, 0.08, 700);
    const view = lookAt(this.eye, target, cameraUp);

    window.__MY_PHYSICS_FRAME__ = {
      simulationTime: physics.physics_time(),
      physicsStep: physics.physics_step_index(),
      speedMps,
      yaw,
      steering: physics.physics_steering(0),
      escActive: physics.physics_esc_active(0) !== 0,
      playerPosition: [x, y, z],
      cameraPosition: [...this.eye],
      cameraLag,
      fieldOfViewDegrees,
      cameraPreset,
      cameraResponse,
      trackElevationRangeM: this.trackElevationRangeM || 0,
    };

    gl.clearColor(0.16, 0.32, 0.38, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.useProgram(this.program);
    gl.bindVertexArray(this.vertexArray);
    gl.uniformMatrix4fv(this.viewProjection, false, multiply(projection, view));
    gl.uniform3fv(this.cameraPosition, this.eye);
    this.drawCalls = 0;
    this.beginBatch();

    if (DRIFT_PLAYGROUND) this.driftPlaygroundGround([x, y, z]);
    else if (SIMULATION_LAB) this.laboratoryGround([x, y, z]);
    else this.raceCourse(physics, [x, y, z], physics.physics_track_half_width());

    const colors = ['#b9ef42', '#34d6c6', '#45b9dd', '#24d46b', '#70db31', '#d8e52d', '#f0bf33', '#ff8a38', '#f05e52', '#c766ef'];
    for (let index = 0; index < physics.physics_vehicle_count(); index++) {
      const position = [
        physics.physics_render_x(index, alpha),
        physics.physics_render_y(index, alpha),
        physics.physics_render_z(index, alpha),
      ];
      const quaternion = {
        w: physics.physics_render_orientation_w(index, alpha),
        x: physics.physics_render_orientation_x(index, alpha),
        y: physics.physics_render_orientation_y(index, alpha),
        z: physics.physics_render_orientation_z(index, alpha),
      };
      this.car(position, frameFromQuaternion(quaternion), rgb(colors[index % colors.length]), index === 0, physics.physics_steering(index) * 0.54);
    }
    const effectRates = SIMULATION_LAB ? effectEmissionRates() : this.updateEffects(elapsed, [x, y, z], playerFrame, telemetry, physics.physics_step_index());
    this.flushBatch(multiply(projection, view), this.eye);
    window.__MY_PHYSICS_FRAME__.drawCalls = this.drawCalls;
    window.__MY_PHYSICS_FRAME__.activeParticles = this.effects.length;
    window.__MY_PHYSICS_FRAME__.effectRates = effectRates;
  }
}

function optionalTelemetry(name, ...args) {
  const value = api?.[name]?.(...args);
  return Number.isFinite(value) ? value : 0;
}

function readPresentationTelemetry(physics = api) {
  const scalarScrub = optionalTelemetry('physics_audio_tire_scrub', 0);
  return {
    speedMps: physics.physics_speed(0),
    longitudinalAccelerationMps2: optionalTelemetry('physics_longitudinal_acceleration', 0),
    lateralAccelerationMps2: physics.physics_lateral_acceleration(0),
    yawRateRadS: physics.physics_yaw_rate(0),
    waterDepthMm: physics.physics_road_water_depth_mm(0),
    tireScrub: [0, 1, 2, 3].map((wheel) => optionalTelemetry('physics_audio_tire_scrub_wheel', 0, wheel) || scalarScrub),
    hydroplaning: [0, 1, 2, 3].map((wheel) => optionalTelemetry('physics_hydroplaning', 0, wheel)),
    brakeTemperatureK: [0, 1, 2, 3].map((wheel) => optionalTelemetry('physics_brake_temperature', 0, wheel) || 300),
    suspensionActivity: [0, 1, 2, 3].map((wheel) => optionalTelemetry('physics_audio_suspension_activity', 0, wheel)),
    roadNoise: [0, 1, 2, 3].map((wheel) => optionalTelemetry('physics_audio_road_noise', 0, wheel)),
    impact: optionalTelemetry('physics_audio_impact', 0),
    damage: physics.physics_damage(0),
    engineLoad: physics.physics_audio_engine_load(0),
    engineRpm: physics.physics_rpm(0),
    redlineRpm: optionalTelemetry('physics_engine_redline', 0) || 7_000,
    intake: optionalTelemetry('physics_audio_intake', 0),
    exhaust: optionalTelemetry('physics_audio_exhaust', 0),
    wind: optionalTelemetry('physics_audio_wind', 0),
  };
}

function updateRaceUi() {
  if (!raceView || SIMULATION_LAB) return;
  const player = raceView.player;
  raceUi.racePosition.textContent = player?.position ?? '—';
  raceUi.raceField.textContent = `/ ${raceView.standings.length}`;
  raceUi.raceLap.textContent = `${Math.min(raceView.totalLaps, (player?.completedLaps ?? 0) + 1)} / ${raceView.totalLaps}`;
  raceUi.raceTime.textContent = formatRaceTime(raceView.raceTimeS);
  raceUi.raceBestLap.textContent = formatRaceTime(player?.bestLapTimeS);
  const showGreen = raceView.phase === RACE_PHASE.RACING && api.physics_time() < greenUntilPhysicsTime;
  raceUi.raceCountdown.hidden = raceView.phase !== RACE_PHASE.COUNTDOWN && !showGreen;
  raceUi.raceCountdownLabel.textContent = showGreen ? 'GO!' : raceView.countdownValue;
  const countdownHint = raceUi.raceCountdown.querySelector('small');
  countdownHint.textContent = showGreen ? 'RACE' : 'GET READY';
  raceUi.raceResults.hidden = raceView.phase !== RACE_PHASE.FINISHED;
  if (raceView.phase === RACE_PHASE.FINISHED) {
    raceUi.raceResultSummary.textContent = `P${player.position} OF ${raceView.standings.length} · ${formatRaceTime(player.finishTimeS)}`;
    raceUi.raceResultsList.innerHTML = raceView.standings.map((participant) => (
      `<li data-player="${participant.index === 0}"><b>CAR ${String(participant.index + 1).padStart(2, '0')}</b><time>${
        participant.finishTimeS === null ? `LAP ${Math.min(raceView.totalLaps, participant.completedLaps + 1)}` : formatRaceTime(participant.finishTimeS)
      }</time></li>`
    )).join('');
  }
}

const DRIFT_PHASE_NAMES = ['GRIP', 'ENTRY', 'SLIDE', 'RECOVERY', 'SPIN'];
function updateDriftUi() {
  if (!DRIFT_PLAYGROUND) return;
  const phase = api.physics_arcade_drift_phase();
  const betaDeg = api.physics_body_slip_angle(0) * 180 / Math.PI;
  const yawDegS = api.physics_yaw_rate(0) * 180 / Math.PI;
  const rawSteering = api.physics_input_stage_steering(0);
  const assistedSteering = api.physics_input_stage_steering(2);
  const correction = api.physics_arcade_drift_correction();
  const speedKmh = api.physics_speed(0) * 3.6;
  const axleSlipDeg = [0, 1, 2, 3].map((wheel) => Math.abs(api.physics_wheel_slip_angle(0, wheel) * 180 / Math.PI));
  const frontSlipDeg = (axleSlipDeg[0] + axleSlipDeg[1]) * 0.5;
  const rearSlipDeg = (axleSlipDeg[2] + axleSlipDeg[3]) * 0.5;
  const phaseName = DRIFT_PHASE_NAMES[phase] || 'GRIP';
  const outcome = classifyDriftOutcome({ phase, betaDeg, speedKmh, rawSteering, frontSlipDeg, rearSlipDeg });
  document.querySelector('#driftPhase').textContent = phaseName;
  document.querySelector('#driftOutcome').textContent = outcome.label;
  document.querySelector('#driftHudBeta').textContent = `${betaDeg.toFixed(1)}°`;
  document.querySelector('#driftHudYaw').textContent = `${yawDegS.toFixed(0)}°/s`;
  document.querySelector('#driftRaw').textContent = rawSteering.toFixed(2);
  document.querySelector('#driftAssist').textContent = assistedSteering.toFixed(2);
  document.querySelector('#driftWheel').textContent = `${(api.physics_wheel_steer_angle(0, 0) * 180 / Math.PI).toFixed(1)}°`;
  document.querySelector('#driftCorrection').textContent = correction.toFixed(2);
  document.querySelector('#driftBeta').textContent = `${betaDeg.toFixed(1)}°`;
  document.querySelector('#driftYaw').textContent = `${yawDegS.toFixed(0)}°/s`;
  document.querySelector('#driftRearLong').textContent = [2, 3]
    .map((wheel) => `${(api.physics_wheel_longitudinal_slip(0, wheel) * 100).toFixed(0)}%`).join(' / ');
  document.querySelector('#driftRearLat').textContent = [2, 3]
    .map((wheel) => `${(api.physics_wheel_slip_angle(0, wheel) * 180 / Math.PI).toFixed(1)}°`).join(' / ');
  document.querySelector('#driftHud').dataset.phase = phaseName.toLowerCase();
  document.querySelector('#driftHud').dataset.outcome = outcome.kind;
}

function updateUi() {
  ui.speed.textContent = (api.physics_speed(0) * 3.6).toFixed(1);
  ui.rpm.textContent = Math.round(api.physics_rpm(0));
  ui.gear.textContent = Math.round(api.physics_gear(0));
  ui.time.textContent = api.physics_time().toFixed(2);
  if (SIMULATION_LAB) {
    ui.lap.textContent = 'FLAT PROVING GROUND';
    ui.trackLength.textContent = '1 ENGINEERING VEHICLE';
  } else if (DRIFT_PLAYGROUND) {
    ui.lap.textContent = 'DRIFT SECTION · OPEN';
    ui.trackLength.textContent = 'R / ENTER RESTARTS ENTRY';
  } else {
    const progress = api.physics_track_progress(0);
    ui.lap.textContent = `${Math.min(raceView?.totalLaps ?? 3, (raceView?.player?.completedLaps ?? 0) + 1)} / ${raceView?.totalLaps ?? 3} · ${Math.round(progress * 100)}%`;
    ui.trackLength.textContent = `${(api.physics_track_length() / 1000).toFixed(2)} km`;
  }
  ui.lod.textContent = Math.round(api.physics_fidelity(0) * 100);
  ui.ffb.textContent = `${api.physics_ffb_steering_torque(0).toFixed(1)} Nm`;
  const damage = api.physics_damage(0);
  ui.damage.style.width = `${damage * 100}%`;
  ui.damageText.textContent = `${Math.round(damage * 100)}%`;
  const profileIndex = api.physics_experience_profile();
  const driveProfile = DRIVE_PROFILES[profileIndex] || 'sport';
  const adaptive = driveProfile !== 'simulation';
  inputConfig = { ...inputConfig, driveProfile, keyboardAdaptive: adaptive };
  ui.driveProfile.value = driveProfile;
  ui.keyboardPolicy.value = adaptive ? 'adaptive' : 'raw';
  ui.tires.innerHTML = [0, 1, 2, 3]
    .map(
      (wheel) => `<div class="tire"><span>${['FL', 'FR', 'RL', 'RR'][wheel]}</span><b>${(
        api.physics_tire_temp(0, wheel) - 273.15
      ).toFixed(1)} °C</b><span>${(api.physics_tire_pressure(0, wheel) / 1000).toFixed(0)} kPa</span></div>`,
    )
    .join('');
  if (SIMULATION_LAB && !window.__MY_PHYSICS_LAB__?.liveDisplayPaused) {
    document.querySelector('#labYaw').textContent = `${api.physics_yaw_rate(0).toFixed(3)} rad/s`;
    document.querySelector('#labSlip').textContent = `${(api.physics_body_slip_angle(0) * 180 / Math.PI).toFixed(2)}°`;
    document.querySelector('#labAy').textContent = `${api.physics_lateral_acceleration(0).toFixed(2)} m/s²`;
    document.querySelector('#labWater').textContent = `${api.physics_road_water_depth_mm(0).toFixed(2)} mm`;
  }
  updateDriftUi();
  updateRaceUi();
}

let renderer;
function benchmarkPhysics() {
  api.physics_reset();
  api.physics_step(100);
  api.physics_reset();
  const start = performance.now();
  api.physics_step(1000);
  const milliseconds = performance.now() - start;
  const realtime = 1000 / milliseconds;
  window.__MY_PHYSICS_BENCHMARK__ = { milliseconds, realtime, vehicles: api.physics_vehicle_count(), steps: 1000 };
  ui.performance.textContent = `${milliseconds.toFixed(1)} ms · ${realtime.toFixed(1)}× RT`;
  ui.performance.dataset.pass = String(milliseconds <= 1000);
  const automaticQuality = realtime >= 4 ? 2 : realtime >= 1.5 ? 1 : 0;
  ui.quality.dataset.automatic = String(automaticQuality);
  api.physics_set_quality(automaticQuality);
  api.physics_reset();
}

function readInputStage(stage) {
  const result = {
    steering: api.physics_input_stage_steering(stage),
    throttle: api.physics_input_stage_throttle(stage),
    brake: api.physics_input_stage_brake(stage),
    clutch: api.physics_input_stage_clutch(stage),
    handbrake: api.physics_input_stage_handbrake(stage),
    gear: api.physics_input_stage_gear(stage),
  };
  if (stage === 4) {
    result.brakePerWheel = [0, 1, 2, 3].map((wheel) => api.physics_input_aid_brake(wheel));
    result.absActive = [0, 1, 2, 3].map((wheel) => api.physics_input_abs_active(wheel) !== 0);
    result.tcActive = api.physics_input_tc_active() !== 0;
    result.escActive = api.physics_input_esc_active() !== 0;
  }
  return result;
}

const VALIDATION_SCENARIOS = Object.freeze([
  { id: 'coast_down', title: 'Neutral coast-down from 100 km/h' },
  { id: 'zero_to_100', title: 'Standing full-throttle acceleration' },
  { id: 'hundred_to_zero', title: 'ABS full braking from 100 km/h' },
  { id: 'steady_steer', title: '0.5° ramp-and-hold at 72 km/h' },
  { id: 'step_steer', title: '1° step steer at 90 km/h' },
  { id: 'slalom', title: '0.5 Hz sine steer at 65 km/h' },
]);
const VALIDATION_METRICS = Object.freeze([
  'final speed (m/s)', 'distance (m)', 'target time (s)', 'peak yaw rate (rad/s)',
  'final |yaw| (rad/s)', 'peak sideslip (rad)', 'peak wheel slip', 'minimum wheel load (N)', 'yaw reversals',
]);

function validationFingerprint() {
  return `${(api.physics_validation_fingerprint_high() >>> 0).toString(16).padStart(8, '0')}${
    (api.physics_validation_fingerprint_low() >>> 0).toString(16).padStart(8, '0')}`;
}

function validationReport(index) {
  const passed = api.physics_validation_run(index) !== 0;
  const fingerprint = validationFingerprint();
  const samples = Array.from({ length: api.physics_validation_sample_count() }, (_, sample) => ({
    time: api.physics_validation_sample(sample, 0),
    speed: api.physics_validation_sample(sample, 1),
    yaw: api.physics_validation_sample(sample, 2),
    sideslip: api.physics_validation_sample(sample, 3),
    acceleration: [4, 5, 6].map((field) => api.physics_validation_sample(sample, field)),
    wheelSlip: [7, 8, 9, 10].map((field) => api.physics_validation_sample(sample, field)),
    wheelSlipAngle: [11, 12, 13, 14].map((field) => api.physics_validation_sample(sample, field)),
    wheelLoad: [15, 16, 17, 18].map((field) => api.physics_validation_sample(sample, field)),
  }));
  const checks = Array.from({ length: api.physics_validation_check_count() }, (_, check) => ({
    metric: Math.round(api.physics_validation_check(check, 0)),
    value: api.physics_validation_check(check, 1),
    min: api.physics_validation_check(check, 2),
    max: api.physics_validation_check(check, 3),
    passed: api.physics_validation_check(check, 4) !== 0,
  }));
  return { scenario: VALIDATION_SCENARIOS[index], passed, fingerprint, samples, checks };
}

function drawLabGraph(samples, signal = document.querySelector('#labGraphSignal')?.value || 'motion') {
  const canvas = document.querySelector('#labGraph');
  const context = canvas.getContext('2d');
  const { width, height } = canvas;
  context.clearRect(0, 0, width, height);
  context.fillStyle = '#080d10';
  context.fillRect(0, 0, width, height);
  if (samples.length < 2) return;
  const pad = 28;
  context.strokeStyle = '#25343a';
  context.lineWidth = 1;
  for (let row = 0; row <= 4; row += 1) {
    const y = pad + (height - pad * 2) * row / 4;
    context.beginPath(); context.moveTo(pad, y); context.lineTo(width - pad, y); context.stroke();
  }
  const maxTime = samples.at(-1).time || 1;
  const trace = (values, minimum, maximum, color) => {
    context.strokeStyle = color; context.lineWidth = 2; context.beginPath();
    samples.forEach((sample, index) => {
      const x = pad + sample.time / maxTime * (width - pad * 2);
      const value = values[index];
      const y = height - pad - (value - minimum) / Math.max(1e-9, maximum - minimum) * (height - pad * 2);
      if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
    });
    context.stroke();
  };
  const palette = ['#57d9ff', '#ffbd52', '#d47dff', '#77e58f'];
  let label;
  if (signal === 'motion') {
    const speed = samples.map((sample) => sample.speed);
    const yaw = samples.map((sample) => sample.yaw);
    const beta = samples.map((sample) => sample.sideslip);
    trace(speed, 0, Math.max(1, ...speed), palette[0]);
    const motionPeak = Math.max(0.05, ...yaw.map(Math.abs), ...beta.map(Math.abs));
    trace(yaw, -motionPeak, motionPeak, palette[1]);
    trace(beta, -motionPeak, motionPeak, palette[2]);
    label = 'SPEED · YAW · BETA (independent scales)';
  } else {
    const field = signal === 'slip' ? 'wheelSlip' : signal === 'slipAngle' ? 'wheelSlipAngle' : 'wheelLoad';
    const values = samples.flatMap((sample) => sample[field]);
    const minimum = signal === 'load' ? 0 : Math.min(-0.01, ...values);
    const maximum = Math.max(signal === 'load' ? 1 : 0.01, ...values);
    for (let wheel = 0; wheel < 4; wheel += 1) trace(samples.map((sample) => sample[field][wheel]), minimum, maximum, palette[wheel]);
    label = `${signal === 'slip' ? 'LONGITUDINAL SLIP' : signal === 'slipAngle' ? 'SLIP ANGLE' : 'NORMAL LOAD'} · FL FR RL RR`;
  }
  context.fillStyle = '#8ba1a8'; context.font = '10px ui-monospace, monospace';
  context.fillText(label, pad, 14);
  context.fillStyle = '#8ba1a8'; context.fillText(`${maxTime.toFixed(1)} s`, width - 58, height - 8);
}

function showValidation(report, replayFingerprint = null) {
  const result = document.querySelector('#labResult');
  const replayMatch = replayFingerprint === null || replayFingerprint === report.fingerprint;
  result.className = `lab-result ${report.passed && replayMatch ? 'pass' : 'fail'}`;
  result.textContent = `${report.passed ? 'PASS' : 'FAIL'} · ${report.scenario.title} · fingerprint ${report.fingerprint}${
    replayFingerprint === null ? '' : replayMatch ? ' · REPEAT BIT-EXACT' : ` · REPEAT MISMATCH ${replayFingerprint}`}`;
  document.querySelector('#labChecks').innerHTML = report.checks.map((check) =>
    `<div class="lab-check ${check.passed ? 'pass' : ''}"><span>${VALIDATION_METRICS[check.metric]}</span><b>${check.value.toFixed(4)}</b><span>[${check.min.toFixed(3)}, ${check.max.toFixed(3)}]</span><span>${check.passed ? 'PASS' : 'FAIL'}</span></div>`
  ).join('');
  drawLabGraph(report.samples);
  window.__MY_PHYSICS_LAB__ = { ...window.__MY_PHYSICS_LAB__, lastReport: report, replayMatch };
}

function installSimulationLab() {
  const scenario = document.querySelector('#labScenario');
  const run = () => {
    const report = validationReport(Number(scenario.value));
    showValidation(report);
    return report;
  };
  document.querySelector('#labRun').addEventListener('click', run);
  document.querySelector('#labRepeat').addEventListener('click', () => {
    const first = run();
    const second = validationReport(Number(scenario.value));
    showValidation(second, first.fingerprint);
  });
  document.querySelector('#labReplay').addEventListener('click', () => {
    const report = run();
    const replayMatch = api.physics_validation_midpoint_replay(Number(scenario.value)) !== 0;
    const result = document.querySelector('#labResult');
    result.className = `lab-result ${report.passed && replayMatch ? 'pass' : 'fail'}`;
    result.textContent = `${report.passed ? 'PASS' : 'FAIL'} · ${report.scenario.title} · fingerprint ${report.fingerprint} · MIDPOINT SNAPSHOT ${replayMatch ? 'EXACT' : 'MISMATCH'}`;
    window.__MY_PHYSICS_LAB__ = { ...window.__MY_PHYSICS_LAB__, midpointReplayMatch: replayMatch };
  });
  document.querySelector('#labRunAll').addEventListener('click', () => {
    const reports = VALIDATION_SCENARIOS.map((_, index) => validationReport(index));
    const allPassed = reports.every((report) => report.passed);
    showValidation(reports.at(-1));
    const result = document.querySelector('#labResult');
    result.className = `lab-result ${allPassed ? 'pass' : 'fail'}`;
    result.textContent = `${allPassed ? 'ALL PASS' : 'CATALOG FAILURE'} · ${reports.length}/6 official EngineeringReference maneuvers · fixed dt 0.001 s`;
    document.querySelector('#labChecks').innerHTML = reports.map((report) => `<div class="lab-check ${report.passed ? 'pass' : ''}"><span>${report.scenario.id}</span><b>${report.passed ? 'PASS' : 'FAIL'}</b><span>${report.fingerprint}</span><span>${report.checks.length} envelopes</span></div>`).join('');
    window.__MY_PHYSICS_LAB__ = { ...window.__MY_PHYSICS_LAB__, catalogReports: reports, allPassed };
  });
  document.querySelector('#labFreeDrive').addEventListener('click', () => {
    api.physics_lab_reset_free_drive();
    setDriveProfile('simulation', false);
    document.querySelector('#labResult').className = 'lab-result';
    document.querySelector('#labResult').textContent = 'FREE DRIVE RESET · ENGINEERING REFERENCE · DRY ROAD';
  });
  let liveDisplayPaused = false;
  document.querySelector('#labPause').addEventListener('click', (event) => {
    liveDisplayPaused = !liveDisplayPaused;
    event.currentTarget.textContent = liveDisplayPaused ? 'RESUME LIVE DISPLAY' : 'PAUSE LIVE DISPLAY';
    window.__MY_PHYSICS_LAB__.liveDisplayPaused = liveDisplayPaused;
  });
  document.querySelector('#labGraphSignal').addEventListener('change', () => {
    const report = window.__MY_PHYSICS_LAB__.lastReport;
    if (report) drawLabGraph(report.samples);
  });
  window.__MY_PHYSICS_LAB__ = { runScenario(index = Number(scenario.value), replay = false) {
    scenario.value = String(index);
    const first = validationReport(index);
    const second = replay ? validationReport(index) : null;
    showValidation(second || first, second ? first.fingerprint : null);
    return second || first;
  }, runAll() { document.querySelector('#labRunAll').click(); return window.__MY_PHYSICS_LAB__.catalogReports; } };
}

function frame(now) {
  const elapsed = Math.min((now - previous) / 1000, 0.05);
  previous = now;
  accumulator += elapsed;
  const input = inputAdapter.read();
  const gamepadStart = Boolean(input.pad?.buttons?.[9]?.pressed);
  if (gamepadStart && !previousGamepadStart && raceView?.phase === RACE_PHASE.FINISHED) resetRace();
  previousGamepadStart = gamepadStart;
  const escStatus = api.physics_player_esc() ? 'ESC ON' : 'ESC OFF';
  const profileName = inputConfig.driveProfile.toUpperCase();
  const targetG = api.physics_policy_lateral_accel_target() / 9.80665;
  const targetLabel = targetG > 0 ? ` · ${targetG.toFixed(2)}G TARGET` : '';
  const steeringMode = input.deviceKind === 1
    ? api.physics_keyboard_assist() ? `${profileName} ADAPTIVE KEYBOARD${targetLabel}` : 'SIMULATION DIGITAL RAW/TEST'
    : input.deviceKind === 3
      ? 'CALIBRATED WHEEL · LINEAR 1:1 · NO SPEED ASSIST'
      : inputConfig.driveProfile === 'simulation'
        ? `${inputConfig.response.toUpperCase()} GAMEPAD · NORMALIZED RAW · NO SPEED ASSIST`
        : `${profileName} ${inputConfig.response.toUpperCase()} GAMEPAD · SPEED POLICY${targetLabel}`;
  const keyboardToggleHint = input.deviceKind === 1 && !SIMULATION_LAB ? ' · I: SPORT/SIM' : '';
  ui.inputDevice.textContent = api.physics_player_autopilot()
    ? `AI DRIVER · P · ${escStatus}`
    : `${input.device} · ${steeringMode}${keyboardToggleHint} · ${escStatus} · E`;
  const controlsLocked = raceView?.phase === RACE_PHASE.FINISHED;
  const commanded = controlsLocked
    ? { steer: 0, throttle: 0, brake: 1, clutch: 0, handbrake: 0 }
    : input;
  if (input.keyboardSteering) {
    api.physics_set_keyboard_input(controlsLocked ? 0 : input.keyboardSteer, commanded.throttle, commanded.brake, commanded.clutch, commanded.handbrake, gear);
  } else {
    api.physics_set_device_input(
      input.deviceKind,
      input.raw.steer,
      input.raw.throttle,
      input.raw.brake,
      input.raw.clutch,
      input.raw.handbrake,
      commanded.steer,
      commanded.throttle,
      commanded.brake,
      commanded.clutch,
      commanded.handbrake,
      gear,
    );
  }
  const steps = Math.min(Math.floor(accumulator / 0.001), 50);
  if (steps) {
    api.physics_step(steps);
    accumulator -= steps * 0.001;
  }
  updateRaceState();
  window.__MY_PHYSICS_INPUT__ = {
    worldStep: api.physics_step_index(),
    appliedStep: api.physics_input_applied_step(),
    sampleSequence: api.physics_input_sample_sequence(),
    deviceKind: api.physics_input_device(),
    transitioning: api.physics_input_transitioning() !== 0,
    device: input.device,
    keyboardSteering: input.keyboardSteering,
    keyboardAssist: api.physics_keyboard_assist() !== 0,
    experienceProfile: DRIVE_PROFILES[api.physics_experience_profile()] || 'sport',
    lateralAccelTargetMps2: api.physics_policy_lateral_accel_target(),
    gamepadAssist: api.physics_gamepad_assist() !== 0,
    demoVehiclePreset: api.physics_demo_vehicle_preset() === 3
      ? 'engineering_reference'
      : api.physics_demo_vehicle_preset() === 2 ? 'arcade_fun' : 'race_gameplay',
    vehicleDefinitionRevision: api.physics_demo_vehicle_preset() === 3
      ? 'vehicle-definition-v0.1'
      : api.physics_demo_vehicle_preset() === 2 ? 'arcade-fun-v1' : 'race-gameplay-v1',
    steer: input.steer,
    throttle: input.throttle,
    brake: input.brake,
    clutch: input.clutch,
    handbrake: input.handbrake,
    arcadeDrift: DRIFT_PLAYGROUND ? {
        phase: DRIFT_PHASE_NAMES[api.physics_arcade_drift_phase()] || 'GRIP',
        engagement: api.physics_arcade_drift_engagement(),
        correction: api.physics_arcade_drift_correction(),
        bodySlipRad: api.physics_body_slip_angle(0),
        yawRateRadS: api.physics_yaw_rate(0),
        physicalFrontSteerRad: api.physics_wheel_steer_angle(0, 0),
        wheelLongitudinalSlip: [0, 1, 2, 3].map((wheel) => api.physics_wheel_longitudinal_slip(0, wheel)),
        wheelSlipAngleRad: [0, 1, 2, 3].map((wheel) => api.physics_wheel_slip_angle(0, wheel)),
        wheelTransientSlipAngleRad: [0, 1, 2, 3].map((wheel) => api.physics_wheel_transient_slip_angle(0, wheel)),
      } : null,
    stages: {
      raw: readInputStage(0),
      normalized: readInputStage(1),
      policy: readInputStage(2),
      plant: readInputStage(3),
      aid: readInputStage(4),
    },
  };
  renderer.scene(api, elapsed, Math.min(1, accumulator / 0.001));
  const presentationTelemetry = readPresentationTelemetry();
  audioEngine.update(presentationTelemetry);
  document.documentElement.style.setProperty('--speed-streak', String(effectEmissionRates(presentationTelemetry).speedStreak));
  inputAdapter.rumble(input.pad, api.physics_ffb_vibration(0), now);
  updateUi();
  requestAnimationFrame(frame);
}

try {
  renderer = new Renderer3D(canvas);
  const response = await fetch('./physics.wasm');
  if (!response.ok) throw new Error(`HTTP ${response.status}: run scripts/build-wasm.sh first`);
  const result = await WebAssembly.instantiateStreaming(response, {});
  api = result.instance.exports;
  api.physics_select_demo_vehicle_preset(SIMULATION_LAB ? 3 : ARCADE_DEMO ? 2 : 1);
  document.querySelector('#saveSnapshot').addEventListener('click', () => {
    const bytes = api.physics_snapshot_save();
    savedRaceState = raceDirector?.snapshot() ?? null;
    ui.snapshotStatus.textContent = `SAVED · ${(bytes / 1024).toFixed(0)} KiB`;
  });
  document.querySelector('#restoreSnapshot').addEventListener('click', () => {
    const restored = api.physics_snapshot_restore();
    if (restored && raceDirector && savedRaceState) raceView = raceDirector.restore(savedRaceState);
    ui.snapshotStatus.textContent = restored ? 'RESTORED' : 'NO SNAPSHOT';
  });
  ui.quality.addEventListener('change', () => {
    const level = ui.quality.value === 'auto' ? Number(ui.quality.dataset.automatic || 2) : Number(ui.quality.value);
    api.physics_set_quality(level);
  });
  ui.cameraPreset.addEventListener('change', () => selectCameraPreset(ui.cameraPreset.value));
  raceUi.raceRestart?.addEventListener('click', () => {
    enableAudio();
    resetRace();
  });
  document.querySelector('#driftRestart')?.addEventListener('click', () => {
    enableAudio();
    resetRace();
  });
  document.querySelector('#audioToggle').addEventListener('click', toggleAudio);
  ui.driveProfile.addEventListener('change', () => setDriveProfile(ui.driveProfile.value));
  ui.keyboardPolicy.addEventListener('change', () => setKeyboardAdaptive(ui.keyboardPolicy.value === 'adaptive'));
  ui.inputResponse.addEventListener('change', () => {
    inputConfig = { ...inputConfig, response: ui.inputResponse.value === 'direct' ? 'direct' : 'balanced' };
    persistInputConfig();
  });
  document.querySelector('#calibrateRest').addEventListener('click', () => {
    const captured = inputAdapter.captureRest();
    if (!captured) {
      ui.calibrationStatus.textContent = 'NO ACTIVE PAD/WHEEL';
      return;
    }
    inputConfig = captured;
    persistInputConfig();
    ui.calibrationStatus.textContent = `SAVED · ${captured.calibratedDevice}`;
  });
  document.querySelector('#resetCalibration').addEventListener('click', () => {
    inputConfig = {
      ...DEFAULT_INPUT_CONFIG,
      driveProfile: inputConfig.driveProfile,
      keyboardAdaptive: inputConfig.driveProfile !== 'simulation',
      response: inputConfig.response,
    };
    persistInputConfig();
    ui.calibrationStatus.textContent = 'DEFAULT RANGE';
  });
  selectCameraPreset(new URLSearchParams(location.search).get('camera') || 'chase');
  ui.inputResponse.value = inputConfig.response;
  ui.driveProfile.value = inputConfig.driveProfile;
  ui.calibrationStatus.textContent = inputConfig.calibratedDevice ? `SAVED · ${inputConfig.calibratedDevice}` : 'DEFAULT RANGE';
  benchmarkPhysics();
  setDriveProfile(inputConfig.driveProfile, false);
  if (DRIFT_PLAYGROUND || raceDirector) resetRace();
  if (SIMULATION_LAB) installSimulationLab();
  if (new URLSearchParams(location.search).get('autopilot') === '1') api.physics_set_player_autopilot(1);
  status.textContent = SIMULATION_LAB
    ? 'SIMULATION LAB ONLINE · ENGINEERING REFERENCE · RAW INPUT'
    : DRIFT_PLAYGROUND ? 'DRIFT PLAYGROUND ONLINE · INPUT ASSIST ONLY · COMMON PHYSICAL PLANT'
      : ARCADE_DEMO ? 'ARCADE FUN ONLINE · SAME WASM PLANT · AUTHORED PARAMETERS' : '3D CORE ONLINE · WEBGL2 · FIXED DT 0.001 s';
  status.classList.add('ready');
  requestAnimationFrame(frame);
} catch (error) {
  status.textContent = `LOAD FAILED · ${error.message}`;
  console.error(error);
}
