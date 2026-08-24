const status = document.querySelector('#status');
const canvas = document.querySelector('#track');
const ui = Object.fromEntries(
  ['speed', 'rpm', 'gear', 'time', 'lod', 'damage', 'damageText', 'tires'].map((id) => [id, document.querySelector(`#${id}`)]),
);
const keys = new Set();
let api;
let previous = performance.now();
let accumulator = 0;
let gear = 1;

addEventListener('keydown', (event) => {
  keys.add(event.code);
  if (/^Digit[1-6]$/.test(event.code)) gear = Number(event.code.at(-1));
  if (event.code === 'KeyR') api?.physics_reset();
});
addEventListener('keyup', (event) => keys.delete(event.code));

function controls() {
  let steer = (keys.has('ArrowLeft') || keys.has('KeyA') ? -1 : 0) + (keys.has('ArrowRight') || keys.has('KeyD') ? 1 : 0);
  let throttle = keys.has('ArrowUp') || keys.has('KeyW') ? 1 : 0;
  let brake = keys.has('ArrowDown') || keys.has('KeyS') ? 1 : 0;
  let handbrake = keys.has('Space') ? 1 : 0;
  const pad = navigator.getGamepads?.()[0];
  if (pad) {
    steer = Math.abs(pad.axes[0]) > 0.08 ? pad.axes[0] : steer;
    throttle = Math.max(throttle, pad.buttons[7]?.value || 0);
    brake = Math.max(brake, pad.buttons[6]?.value || 0);
    handbrake = Math.max(handbrake, pad.buttons[0]?.value || 0);
  }
  return { steer, throttle, brake, handbrake };
}

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

  scene(physics, elapsed) {
    const gl = this.gl;
    const aspect = this.resize();
    const x = physics.physics_x(0);
    const y = physics.physics_y(0);
    const z = physics.physics_z(0);
    const yaw = physics.physics_yaw(0);
    const forward = [-Math.sin(yaw), 0, -Math.cos(yaw)];
    const desiredEye = [x - forward[0] * 10.5, y + 5.8, z - forward[2] * 10.5];
    if (!this.eye) this.eye = desiredEye;
    const cameraBlend = 1 - Math.exp(-elapsed * 5.5);
    this.eye = this.eye.map((value, index) => value + (desiredEye[index] - value) * cameraBlend);
    const target = [x + forward[0] * 5.2, y + 0.2, z + forward[2] * 5.2];
    const projection = perspective(Math.PI / 3.15, aspect, 0.08, 700);
    const view = lookAt(this.eye, target);

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
    for (let post = Math.floor((z - 160) / 20) * 20; post < z + 100; post += 20) {
      this.box([-10.5, 2.0, post], [0.18, 4, 0.18], rgb('#566059'));
      this.box([10.5, 2.0, post], [0.18, 4, 0.18], rgb('#566059'));
      this.box([-10.5, 3.8, post], [0.9, 0.18, 4.5], rgb('#66706a'));
    }

    const colors = ['#b9ef42', '#34d6c6', '#45b9dd', '#24d46b', '#70db31', '#d8e52d', '#f0bf33', '#ff8a38', '#f05e52', '#c766ef'];
    for (let index = 0; index < physics.physics_vehicle_count(); index++) {
      const position = [physics.physics_x(index), physics.physics_y(index), physics.physics_z(index)];
      this.car(position, physics.physics_yaw(index), rgb(colors[index % colors.length]), index === 0);
    }
  }
}

function updateUi() {
  ui.speed.textContent = (api.physics_speed(0) * 3.6).toFixed(1);
  ui.rpm.textContent = Math.round(api.physics_rpm(0));
  ui.gear.textContent = Math.round(api.physics_gear(0));
  ui.time.textContent = api.physics_time().toFixed(2);
  ui.lod.textContent = Math.round(api.physics_fidelity(0) * 100);
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
function frame(now) {
  const elapsed = Math.min((now - previous) / 1000, 0.05);
  previous = now;
  accumulator += elapsed;
  const input = controls();
  api.physics_set_input(input.steer, input.throttle, input.brake, input.handbrake, gear);
  const steps = Math.min(Math.floor(accumulator / 0.001), 50);
  if (steps) {
    api.physics_step(steps);
    accumulator -= steps * 0.001;
  }
  renderer.scene(api, elapsed);
  updateUi();
  requestAnimationFrame(frame);
}

try {
  renderer = new Renderer3D(canvas);
  const response = await fetch('./physics.wasm');
  if (!response.ok) throw new Error(`HTTP ${response.status}: run scripts/build-wasm.sh first`);
  const result = await WebAssembly.instantiateStreaming(response, {});
  api = result.instance.exports;
  status.textContent = '3D CORE ONLINE · WEBGL2 · FIXED DT 0.001 s';
  status.classList.add('ready');
  requestAnimationFrame(frame);
} catch (error) {
  status.textContent = `LOAD FAILED · ${error.message}`;
  console.error(error);
}
