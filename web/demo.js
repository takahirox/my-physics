const status = document.querySelector('#status');
const canvas = document.querySelector('#track');
const ui = Object.fromEntries(
  ['speed', 'rpm', 'gear', 'time', 'lod', 'damage', 'damageText', 'tires', 'performance', 'inputDevice', 'ffb', 'snapshotStatus', 'quality'].map(
    (id) => [id, document.querySelector(`#${id}`)],
  ),
);
const keys = new Set();
let api;
let previous = performance.now();
let accumulator = 0;
let gear = 0;

addEventListener('keydown', (event) => {
  keys.add(event.code);
  if (/^Digit[1-6]$/.test(event.code)) gear = Number(event.code.at(-1));
  if (event.code === 'KeyT') gear = 0;
  if (event.code === 'KeyR') api?.physics_reset();
  if (event.code === 'KeyK' && api) {
    const bytes = api.physics_snapshot_save();
    ui.snapshotStatus.textContent = `SAVED · ${(bytes / 1024).toFixed(0)} KiB`;
  }
  if (event.code === 'KeyL' && api) {
    ui.snapshotStatus.textContent = api.physics_snapshot_restore() ? 'RESTORED' : 'NO SNAPSHOT';
  }
});
addEventListener('keyup', (event) => keys.delete(event.code));

class InputAdapter {
  constructor() {
    this.parameters = new URLSearchParams(location.search);
    this.lastRumble = 0;
  }

  pedal(axis) {
    return Math.max(0, Math.min(1, (1 - axis) * 0.5));
  }

  axis(pad, name, fallback) {
    const index = Number(this.parameters.get(`${name}Axis`) ?? fallback);
    return pad.axes[index] ?? 1;
  }

  read() {
    let steer = (keys.has('ArrowLeft') || keys.has('KeyA') ? -1 : 0) + (keys.has('ArrowRight') || keys.has('KeyD') ? 1 : 0);
    let throttle = keys.has('ArrowUp') || keys.has('KeyW') ? 1 : 0;
    let brake = keys.has('ArrowDown') || keys.has('KeyS') ? 1 : 0;
    let clutch = keys.has('ShiftLeft') || keys.has('ShiftRight') ? 1 : 0;
    let handbrake = keys.has('Space') ? 1 : 0;
    let device = 'KEYBOARD';
    const pad = [...(navigator.getGamepads?.() || [])].find(Boolean);
    if (pad) {
      const isWheel = /wheel|g29|g920|g923|t150|t248|t300|t500|fanatec|moza|simagic/i.test(pad.id);
      device = isWheel ? `WHEEL · ${pad.id}` : `GAMEPAD · ${pad.id}`;
      if (isWheel) {
        steer = this.axis(pad, 'steer', 0);
        throttle = Math.max(throttle, this.pedal(this.axis(pad, 'throttle', 1)));
        brake = Math.max(brake, this.pedal(this.axis(pad, 'brake', 2)));
        clutch = Math.max(clutch, this.pedal(this.axis(pad, 'clutch', 3)));
        handbrake = Math.max(handbrake, pad.buttons[0]?.value || 0);
      } else {
        steer = Math.abs(pad.axes[0]) > 0.08 ? pad.axes[0] : steer;
        throttle = Math.max(throttle, pad.buttons[7]?.value || 0);
        brake = Math.max(brake, pad.buttons[6]?.value || 0);
        clutch = Math.max(clutch, pad.buttons[4]?.value || 0);
        handbrake = Math.max(handbrake, pad.buttons[0]?.value || 0);
      }
    }
    return { steer, throttle, brake, clutch, handbrake, device, pad };
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

function compileShader(gl, type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(shader));
  return shader;
}

function createProgram(gl) {
  const program = gl.createProgram();
  gl.attachShader(program, compileShader(gl, gl.VERTEX_SHADER, vertexShader));
  gl.attachShader(program, compileShader(gl, gl.FRAGMENT_SHADER, fragmentShader));
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

function lookAt(eye, center) {
  const z = normalize(eye.map((value, index) => value - center[index]));
  const x = normalize(cross([0, 1, 0], z));
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

function modelMatrix(position, scale, yaw = 0) {
  const cosine = Math.cos(yaw);
  const sine = Math.sin(yaw);
  return new Float32Array([
    cosine * scale[0], 0, -sine * scale[0], 0,
    0, scale[1], 0, 0,
    sine * scale[2], 0, cosine * scale[2], 0,
    position[0], position[1], position[2], 1,
  ]);
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
    gl.enable(gl.DEPTH_TEST);
    gl.enable(gl.CULL_FACE);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    this.eye = null;
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

  box(position, scale, color, yaw = 0) {
    const gl = this.gl;
    gl.uniformMatrix4fv(this.model, false, modelMatrix(position, scale, yaw));
    gl.uniform4fv(this.color, color);
    gl.drawArrays(gl.TRIANGLES, 0, this.vertexCount);
  }

  localPoint(position, yaw, local) {
    const cosine = Math.cos(yaw);
    const sine = Math.sin(yaw);
    return [position[0] + cosine * local[0] + sine * local[2], position[1] + local[1], position[2] - sine * local[0] + cosine * local[2]];
  }

  car(position, yaw, color, player) {
    this.box([position[0], 0.025, position[2]], [2.05, 0.025, 4.65], rgb('#020503', 0.38), yaw);
    this.box(this.localPoint(position, yaw, [0, 0, 0.12]), [1.82, 0.48, 4.22], color, yaw);
    this.box(this.localPoint(position, yaw, [0, 0.45, 0.35]), [1.48, 0.54, 1.82], rgb(player ? '#26361d' : '#17201c'), yaw);
    this.box(this.localPoint(position, yaw, [0, 0.75, 0.31]), [1.34, 0.12, 1.25], rgb('#80a39c'), yaw);
    for (const x of [-0.94, 0.94]) {
      for (const z of [-1.35, 1.35]) this.box(this.localPoint(position, yaw, [x, -0.05, z]), [0.25, 0.62, 0.72], rgb('#070907'), yaw);
    }
    for (const x of [-0.57, 0.57]) {
      this.box(this.localPoint(position, yaw, [x, 0.02, -2.14]), [0.36, 0.16, 0.05], rgb('#e8f7d1'), yaw);
      this.box(this.localPoint(position, yaw, [x, 0.02, 2.14]), [0.38, 0.16, 0.05], rgb('#ff3e28'), yaw);
    }
  }

  scene(physics, elapsed, alpha) {
    const gl = this.gl;
    const aspect = this.resize();
    const x = physics.physics_render_x(0, alpha);
    const y = physics.physics_render_y(0, alpha);
    const z = physics.physics_render_z(0, alpha);
    const yaw = physics.physics_render_yaw(0, alpha);
    const speedMps = physics.physics_speed(0);
    const forward = [-Math.sin(yaw), 0, -Math.cos(yaw)];
    const speedRatio = Math.min(speedMps / 65, 1);
    const desiredEye = [x - forward[0] * 8.8, y + 4.2 - speedRatio * 0.45, z - forward[2] * 8.8];
    if (!this.eye) this.eye = desiredEye;
    const cameraBlend = 1 - Math.exp(-elapsed * 9.0);
    this.eye = this.eye.map((value, index) => value + (desiredEye[index] - value) * cameraBlend);
    const cameraError = desiredEye.map((value, index) => value - this.eye[index]);
    const unclampedCameraLag = Math.hypot(...cameraError);
    if (unclampedCameraLag > 2.2) {
      this.eye = desiredEye.map((value, index) => value - (cameraError[index] / unclampedCameraLag) * 2.2);
    }
    const cameraLag = Math.hypot(...desiredEye.map((value, index) => value - this.eye[index]));
    const target = [x + forward[0] * 6.4, y + 0.15, z + forward[2] * 6.4];
    const fieldOfViewDegrees = 58 + speedRatio * 16;
    const projection = perspective((fieldOfViewDegrees * Math.PI) / 180, aspect, 0.08, 700);
    const view = lookAt(this.eye, target);

    window.__MY_PHYSICS_FRAME__ = {
      simulationTime: physics.physics_time(),
      speedMps,
      playerPosition: [x, y, z],
      cameraPosition: [...this.eye],
      cameraLag,
      fieldOfViewDegrees,
    };

    gl.clearColor(0.052, 0.08, 0.061, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.useProgram(this.program);
    gl.bindVertexArray(this.vertexArray);
    gl.uniformMatrix4fv(this.viewProjection, false, multiply(projection, view));
    gl.uniform3fv(this.cameraPosition, this.eye);

    const trackCenter = Math.floor(z / 200) * 200;
    this.box([0, -0.2, trackCenter], [220, 0.3, 600], rgb('#203820'));
    this.box([0, -0.015, trackCenter], [14, 0.08, 600], rgb('#222925'));
    this.box([-7.15, 0.32, trackCenter], [0.3, 0.72, 600], rgb('#dbe1d8'));
    this.box([7.15, 0.32, trackCenter], [0.3, 0.72, 600], rgb('#dbe1d8'));
    for (let marker = Math.floor((z - 180) / 10) * 10; marker < z + 90; marker += 10) {
      this.box([0, 0.045, marker], [0.10, 0.025, 4.5], rgb('#c9d0c8'));
      const curbColor = Math.abs(Math.floor(marker / 5)) % 2 ? rgb('#edf0ea') : rgb('#b9ef42');
      this.box([-6.75, 0.07, marker], [0.45, 0.10, 5], curbColor);
      this.box([6.75, 0.07, marker], [0.45, 0.10, 5], curbColor);
    }
    for (let detail = Math.floor((z - 100) / 5) * 5; detail < z + 55; detail += 5) {
      this.box([-3.5, 0.038, detail], [0.08, 0.018, 2.2], rgb('#6f7771'));
      this.box([3.5, 0.038, detail], [0.08, 0.018, 2.2], rgb('#6f7771'));
      this.box([-6.15, 0.065, detail], [0.12, 0.07, 0.32], rgb('#dbe8b8'));
      this.box([6.15, 0.065, detail], [0.12, 0.07, 0.32], rgb('#dbe8b8'));
    }
    for (let post = Math.floor((z - 160) / 20) * 20; post < z + 100; post += 20) {
      this.box([-10.5, 2.0, post], [0.18, 4, 0.18], rgb('#566059'));
      this.box([10.5, 2.0, post], [0.18, 4, 0.18], rgb('#566059'));
      this.box([-10.5, 3.8, post], [0.9, 0.18, 4.5], rgb('#66706a'));
    }

    const colors = ['#b9ef42', '#34d6c6', '#45b9dd', '#24d46b', '#70db31', '#d8e52d', '#f0bf33', '#ff8a38', '#f05e52', '#c766ef'];
    for (let index = 0; index < physics.physics_vehicle_count(); index++) {
      const position = [
        physics.physics_render_x(index, alpha),
        physics.physics_render_y(index, alpha),
        physics.physics_render_z(index, alpha),
      ];
      this.car(position, physics.physics_render_yaw(index, alpha), rgb(colors[index % colors.length]), index === 0);
    }
  }
}

function updateUi() {
  ui.speed.textContent = (api.physics_speed(0) * 3.6).toFixed(1);
  ui.rpm.textContent = Math.round(api.physics_rpm(0));
  ui.gear.textContent = Math.round(api.physics_gear(0));
  ui.time.textContent = api.physics_time().toFixed(2);
  ui.lod.textContent = Math.round(api.physics_fidelity(0) * 100);
  ui.ffb.textContent = `${api.physics_ffb_steering_torque(0).toFixed(1)} Nm`;
  const damage = api.physics_damage(0);
  ui.damage.style.width = `${damage * 100}%`;
  ui.damageText.textContent = `${Math.round(damage * 100)}%`;
  ui.tires.innerHTML = [0, 1, 2, 3]
    .map(
      (wheel) => `<div class="tire"><span>${['FL', 'FR', 'RL', 'RR'][wheel]}</span><b>${(
        api.physics_tire_temp(0, wheel) - 273.15
      ).toFixed(1)} °C</b><span>${(api.physics_tire_pressure(0, wheel) / 1000).toFixed(0)} kPa</span></div>`,
    )
    .join('');
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

function frame(now) {
  const elapsed = Math.min((now - previous) / 1000, 0.05);
  previous = now;
  accumulator += elapsed;
  const input = inputAdapter.read();
  ui.inputDevice.textContent = input.device;
  api.physics_set_input(input.steer, input.throttle, input.brake, input.clutch, input.handbrake, gear);
  const steps = Math.min(Math.floor(accumulator / 0.001), 50);
  if (steps) {
    api.physics_step(steps);
    accumulator -= steps * 0.001;
  }
  renderer.scene(api, elapsed, Math.min(1, accumulator / 0.001));
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
  document.querySelector('#saveSnapshot').addEventListener('click', () => {
    const bytes = api.physics_snapshot_save();
    ui.snapshotStatus.textContent = `SAVED · ${(bytes / 1024).toFixed(0)} KiB`;
  });
  document.querySelector('#restoreSnapshot').addEventListener('click', () => {
    ui.snapshotStatus.textContent = api.physics_snapshot_restore() ? 'RESTORED' : 'NO SNAPSHOT';
  });
  ui.quality.addEventListener('change', () => {
    const level = ui.quality.value === 'auto' ? Number(ui.quality.dataset.automatic || 2) : Number(ui.quality.value);
    api.physics_set_quality(level);
  });
  benchmarkPhysics();
  status.textContent = '3D CORE ONLINE · WEBGL2 · FIXED DT 0.001 s';
  status.classList.add('ready');
  requestAnimationFrame(frame);
} catch (error) {
  status.textContent = `LOAD FAILED · ${error.message}`;
  console.error(error);
}
