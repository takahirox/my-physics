import {
  CAMERA_PRESET_ORDER,
  VISUAL_CUES,
  cameraSettings,
  metricIntervals,
  metricSamples,
} from './visual-config.mjs';

const status = document.querySelector('#status');
const canvas = document.querySelector('#track');
const ui = Object.fromEntries(
  ['speed', 'rpm', 'gear', 'time', 'lap', 'trackLength', 'lod', 'damage', 'damageText', 'tires', 'performance', 'inputDevice', 'ffb', 'snapshotStatus', 'quality', 'cameraPreset'].map(
    (id) => [id, document.querySelector(`#${id}`)],
  ),
);
const keys = new Set();
let api;
let previous = performance.now();
let accumulator = 0;
let gear = 0;
let completedLaps = 0;
let previousProgress;
let cameraPreset = 'chase';

function selectCameraPreset(name) {
  cameraPreset = CAMERA_PRESET_ORDER.includes(name) ? name : 'chase';
  if (ui.cameraPreset) ui.cameraPreset.value = cameraPreset;
  if (renderer) renderer.eye = null;
}

addEventListener('keydown', (event) => {
  keys.add(event.code);
  if (/^Digit[1-6]$/.test(event.code)) gear = Number(event.code.at(-1));
  if (event.code === 'KeyT') gear = 0;
  if (event.code === 'KeyR') {
    api?.physics_reset();
    completedLaps = 0;
    previousProgress = undefined;
  }
  if (event.code === 'KeyP' && api && !event.repeat) {
    api.physics_set_player_autopilot(api.physics_player_autopilot() ? 0 : 1);
  }
  if (event.code === 'KeyE' && api && !event.repeat) {
    api.physics_set_player_esc(api.physics_player_esc() ? 0 : 1);
  }
  if (event.code === 'KeyI' && api && !event.repeat) {
    api.physics_set_keyboard_assist(api.physics_keyboard_assist() ? 0 : 1);
  }
  if (event.code === 'KeyC' && !event.repeat) {
    const current = CAMERA_PRESET_ORDER.indexOf(cameraPreset);
    selectCameraPreset(CAMERA_PRESET_ORDER[(current + 1) % CAMERA_PRESET_ORDER.length]);
  }
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
    const keyboardSteer =
      (keys.has('ArrowLeft') || keys.has('KeyA') ? -1 : 0) + (keys.has('ArrowRight') || keys.has('KeyD') ? 1 : 0);
    let steer = keyboardSteer;
    let keyboardSteering = true;
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
        if (keyboardSteer === 0) {
          steer = this.axis(pad, 'steer', 0);
          keyboardSteering = false;
        }
        throttle = Math.max(throttle, this.pedal(this.axis(pad, 'throttle', 1)));
        brake = Math.max(brake, this.pedal(this.axis(pad, 'brake', 2)));
        clutch = Math.max(clutch, this.pedal(this.axis(pad, 'clutch', 3)));
        handbrake = Math.max(handbrake, pad.buttons[0]?.value || 0);
      } else {
        if (keyboardSteer === 0) {
          steer = Math.abs(pad.axes[0]) > 0.08 ? pad.axes[0] : 0;
          keyboardSteering = false;
        }
        throttle = Math.max(throttle, pad.buttons[7]?.value || 0);
        brake = Math.max(brake, pad.buttons[6]?.value || 0);
        clutch = Math.max(clutch, pad.buttons[4]?.value || 0);
        handbrake = Math.max(handbrake, pad.buttons[0]?.value || 0);
      }
    }
    return { steer, keyboardSteer, keyboardSteering, throttle, brake, clutch, handbrake, device, pad };
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
    if (this.instances) {
      this.instances.push(...modelMatrix(position, scale, yaw), ...color);
      return;
    }
    const gl = this.gl;
    gl.uniformMatrix4fv(this.model, false, modelMatrix(position, scale, yaw));
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

  grandstand(position, yaw, trackHalfWidth, side) {
    const baseX = side * (trackHalfWidth + 4.3);
    for (let row = 0; row < 5; row += 1) {
      const x = baseX + side * row * 0.9;
      this.box(this.localPoint(position, yaw, [x, 0.4 + row * 0.48, 0]), [1.0, 0.75, 30], rgb(row % 2 ? '#303a36' : '#46534d'), yaw);
      for (let seat = -13; seat <= 13; seat += 2) {
        const seatColor = Math.abs(seat + row) % 4 ? '#c5d0c8' : '#b9ef42';
        this.box(this.localPoint(position, yaw, [x - side * 0.5, 0.88 + row * 0.48, seat]), [0.18, 0.16, 1.1], rgb(seatColor), yaw);
      }
    }
    this.box(this.localPoint(position, yaw, [baseX + side * 2.1, 3.4, 0]), [5.7, 0.22, 32], rgb('#c4cbc4'), yaw);
    this.box(this.localPoint(position, yaw, [baseX + side * 4.8, 1.7, 0]), [0.18, 3.4, 32], rgb('#6f7974'), yaw);
  }

  raceCourse(physics, playerPosition, trackHalfWidth) {
    if (!this.circuit) {
      let cumulativeDistanceM = 0;
      this.circuit = Array.from({ length: physics.physics_track_segment_count() }, (_, index) => {
        const length = physics.physics_track_segment_length(index);
        const segment = {
          position: [physics.physics_track_segment_x(index), 0, physics.physics_track_segment_z(index)],
          yaw: physics.physics_track_segment_yaw(index),
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
    }
    const roadWidth = trackHalfWidth * 2;
    this.box([0, -0.22, 0], [760, 0.32, 760], rgb('#18351d'));

    for (let index = 0; index < this.circuit.length; index += 1) {
      const segment = this.circuit[index];
      const midpoint = this.localPoint(segment.position, segment.yaw, [0, 0, -segment.length * 0.5]);
      const distance = Math.hypot(midpoint[0] - playerPosition[0], midpoint[2] - playerPosition[2]);
      if (distance > 235) continue;
      const joinLength = segment.length + 0.65;
      this.box([midpoint[0], -0.015, midpoint[2]], [roadWidth, 0.08, joinLength], rgb('#202522'), segment.yaw);
      this.box(this.localPoint(midpoint, segment.yaw, [-trackHalfWidth + 0.12, 0.05, 0]), [0.16, 0.025, joinLength], rgb('#f0f2ec'), segment.yaw);
      this.box(this.localPoint(midpoint, segment.yaw, [trackHalfWidth - 0.12, 0.05, 0]), [0.16, 0.025, joinLength], rgb('#f0f2ec'), segment.yaw);
      for (const side of [-1, 1]) {
        this.box(this.localPoint(midpoint, segment.yaw, [side * (trackHalfWidth + 0.3), 0.34, 0]), [0.6, 0.68, joinLength], rgb('#bcc3be'), segment.yaw);
        this.box(this.localPoint(midpoint, segment.yaw, [side * (trackHalfWidth + 0.62), 0.24, 0]), [0.035, 0.26, joinLength], rgb('#59615d'), segment.yaw);
        this.box(this.localPoint(midpoint, segment.yaw, [side * (trackHalfWidth + 0.72), 2.3, 0]), [0.055, 0.055, joinLength], rgb('#8c9691'), segment.yaw);
      }
      if (distance <= VISUAL_CUES.detailRadiusM) {
        for (const band of segment.curbBands) {
          const curbColor = band.band % 2 ? rgb('#d9342b') : rgb('#f4f2e9');
          for (const side of [-1, 1]) {
            this.box(
              this.localPoint(segment.position, segment.yaw, [side * (trackHalfWidth - 0.3), 0.07, -band.centerM]),
              [0.6, 0.1, band.lengthM + VISUAL_CUES.curbJoinOverlapM],
              curbColor,
              segment.yaw,
            );
          }
        }
        for (const sample of segment.fencePosts) {
          for (const side of [-1, 1]) {
            const point = this.localPoint(segment.position, segment.yaw, [side * (trackHalfWidth + 0.72), 0, -sample.localM]);
            this.box([point[0], 1.45, point[2]], [0.09, 2.9, 0.09], rgb('#77817c'), segment.yaw);
          }
        }
        for (const sample of segment.seams) {
          const seamLane = ((sample.index * 3) % 5 - 2) * 1.45;
          this.box(
            this.localPoint(segment.position, segment.yaw, [seamLane, 0.033, -sample.localM]),
            [2.1 + (Math.abs(sample.index) % 3) * 0.45, 0.012, 0.04],
            rgb('#151a17', 0.58),
            segment.yaw,
          );
        }
        for (const sample of segment.patches) {
          const lane = ((sample.index % 4) - 1.5) * 1.35;
          this.box(
            this.localPoint(segment.position, segment.yaw, [lane, 0.029, -sample.localM]),
            [1.05, 0.01, 1.35],
            rgb(sample.index % 3 ? '#252b27' : '#1b211e', 0.72),
            segment.yaw,
          );
        }
        for (const sample of segment.rubber) {
          for (const rubber of [-0.88, 0.88]) {
            this.box(
              this.localPoint(segment.position, segment.yaw, [rubber, 0.036, -sample.localM]),
              [0.09, 0.014, 2.1],
              rgb('#0e1210', 0.86),
              segment.yaw,
            );
          }
        }
      }
      for (const sample of segment.boards) {
        const board = this.localPoint(segment.position, segment.yaw, [trackHalfWidth + 1.25, 0, -sample.localM]);
        this.box([board[0], 1.0, board[2]], [0.1, 2.0, 0.1], rgb('#707873'), segment.yaw);
        this.box([board[0], 2.05, board[2]], [1.05, 0.75, 0.16], rgb('#f0f1eb'), segment.yaw);
      }
    }

    const start = this.circuit[0];
    for (let row = 0; row < 2; row += 1) {
      for (let square = 0; square < 12; square += 1) {
        const local = [-trackHalfWidth + (square + 0.5) * (roadWidth / 12), 0.055, row * 0.55];
        const point = this.localPoint(start.position, start.yaw, local);
        this.box(point, [roadWidth / 12 + 0.01, 0.025, 0.56], rgb((square + row) % 2 ? '#171b18' : '#f4f5ee'), start.yaw);
      }
    }
    for (const side of [-1, 1]) this.box(this.localPoint(start.position, start.yaw, [side * (trackHalfWidth + 0.72), 3.1, 0]), [0.32, 6.2, 0.42], rgb('#87918c'), start.yaw);
    this.box(this.localPoint(start.position, start.yaw, [0, 5.75, 0]), [roadWidth + 2.1, 0.55, 0.7], rgb('#171d19'), start.yaw);
    this.box(this.localPoint(start.position, start.yaw, [0, 5.72, 0.37]), [5.2, 0.28, 0.05], rgb('#b9ef42'), start.yaw);
    for (let slot = 8; slot <= 44; slot += 8) {
      const lane = Math.floor(slot / 8) % 2 ? -1.65 : 1.65;
      this.box(this.localPoint(start.position, start.yaw, [lane, 0.05, slot]), [2.25, 0.025, 0.09], rgb('#d8ddd7'), start.yaw);
    }
    const stands = this.localPoint(start.position, start.yaw, [0, 0, 24]);
    this.grandstand(stands, start.yaw, trackHalfWidth, -1);
    this.grandstand(stands, start.yaw, trackHalfWidth, 1);
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
    const camera = cameraSettings(cameraPreset, speedMps);
    const desiredEye = [x - forward[0] * camera.backM, y + camera.heightM, z - forward[2] * camera.backM];
    if (!this.eye) this.eye = desiredEye;
    const cameraBlend = 1 - Math.exp(-elapsed * camera.responsePerS);
    this.eye = this.eye.map((value, index) => value + (desiredEye[index] - value) * cameraBlend);
    const cameraError = desiredEye.map((value, index) => value - this.eye[index]);
    const unclampedCameraLag = Math.hypot(...cameraError);
    if (unclampedCameraLag > camera.maxLagM) {
      this.eye = desiredEye.map((value, index) => value - (cameraError[index] / unclampedCameraLag) * camera.maxLagM);
    }
    const cameraLag = Math.hypot(...desiredEye.map((value, index) => value - this.eye[index]));
    const target = [
      x + forward[0] * camera.targetAheadM,
      y + camera.targetHeightM,
      z + forward[2] * camera.targetAheadM,
    ];
    const fieldOfViewDegrees = camera.fieldOfViewDegrees;
    const projection = perspective((fieldOfViewDegrees * Math.PI) / 180, aspect, 0.08, 700);
    const view = lookAt(this.eye, target);

    window.__MY_PHYSICS_FRAME__ = {
      simulationTime: physics.physics_time(),
      speedMps,
      yaw,
      steering: physics.physics_steering(0),
      escActive: physics.physics_esc_active(0) !== 0,
      playerPosition: [x, y, z],
      cameraPosition: [...this.eye],
      cameraLag,
      fieldOfViewDegrees,
      cameraPreset,
    };

    gl.clearColor(0.052, 0.08, 0.061, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.useProgram(this.program);
    gl.bindVertexArray(this.vertexArray);
    gl.uniformMatrix4fv(this.viewProjection, false, multiply(projection, view));
    gl.uniform3fv(this.cameraPosition, this.eye);
    this.drawCalls = 0;
    this.beginBatch();

    this.raceCourse(physics, [x, y, z], physics.physics_track_half_width());

    const colors = ['#b9ef42', '#34d6c6', '#45b9dd', '#24d46b', '#70db31', '#d8e52d', '#f0bf33', '#ff8a38', '#f05e52', '#c766ef'];
    for (let index = 0; index < physics.physics_vehicle_count(); index++) {
      const position = [
        physics.physics_render_x(index, alpha),
        physics.physics_render_y(index, alpha),
        physics.physics_render_z(index, alpha),
      ];
      this.car(position, physics.physics_render_yaw(index, alpha), rgb(colors[index % colors.length]), index === 0);
    }
    this.flushBatch(multiply(projection, view), this.eye);
    window.__MY_PHYSICS_FRAME__.drawCalls = this.drawCalls;
  }
}

function updateUi() {
  ui.speed.textContent = (api.physics_speed(0) * 3.6).toFixed(1);
  ui.rpm.textContent = Math.round(api.physics_rpm(0));
  ui.gear.textContent = Math.round(api.physics_gear(0));
  ui.time.textContent = api.physics_time().toFixed(2);
  const progress = api.physics_track_progress(0);
  if (previousProgress > 0.82 && progress < 0.18) completedLaps += 1;
  if (previousProgress < 0.18 && progress > 0.82) completedLaps = Math.max(0, completedLaps - 1);
  previousProgress = progress;
  ui.lap.textContent = `${completedLaps + 1} · ${Math.round(progress * 100)}%`;
  ui.trackLength.textContent = `${(api.physics_track_length() / 1000).toFixed(2)} km`;
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
  window.__MY_PHYSICS_INPUT__ = {
    steer: input.steer,
    keyboardSteering: input.keyboardSteering,
    throttle: input.throttle,
    brake: input.brake,
    clutch: input.clutch,
    handbrake: input.handbrake,
    device: input.device,
    keyboardAssist: api.physics_keyboard_assist() !== 0,
  };
  const escStatus = api.physics_player_esc() ? 'ESC ON' : 'ESC OFF';
  const steeringMode = api.physics_keyboard_assist() ? 'STEER ASSIST' : 'STEER RAW';
  ui.inputDevice.textContent = api.physics_player_autopilot()
    ? `AI DRIVER · P · ${escStatus}`
    : `${input.device} · ${steeringMode} · I · ${escStatus} · E`;
  if (input.keyboardSteering) {
    api.physics_set_keyboard_input(input.keyboardSteer, input.throttle, input.brake, input.clutch, input.handbrake, gear);
  } else {
    api.physics_set_input(input.steer, input.throttle, input.brake, input.clutch, input.handbrake, gear);
  }
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
  ui.cameraPreset.addEventListener('change', () => selectCameraPreset(ui.cameraPreset.value));
  selectCameraPreset(new URLSearchParams(location.search).get('camera') || 'chase');
  benchmarkPhysics();
  if (new URLSearchParams(location.search).get('keyboardAssist') === '1') api.physics_set_keyboard_assist(1);
  if (new URLSearchParams(location.search).get('autopilot') === '1') api.physics_set_player_autopilot(1);
  status.textContent = '3D CORE ONLINE · WEBGL2 · FIXED DT 0.001 s';
  status.classList.add('ready');
  requestAnimationFrame(frame);
} catch (error) {
  status.textContent = `LOAD FAILED · ${error.message}`;
  console.error(error);
}
