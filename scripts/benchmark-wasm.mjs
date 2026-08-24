import { readFile } from 'node:fs/promises';
import { performance } from 'node:perf_hooks';

const repository = new URL('../', import.meta.url);
const binary = await readFile(new URL('web/physics.wasm', repository));
const { instance } = await WebAssembly.instantiate(binary, {});
const physics = instance.exports;

physics.physics_reset();
physics.physics_step(100);
physics.physics_reset();
const start = performance.now();
physics.physics_step(1000);
const milliseconds = performance.now() - start;
const result = {
  runtime: `Node ${process.version}`,
  vehicles: physics.physics_vehicle_count(),
  baseSteps: 1000,
  simulatedSeconds: physics.physics_time(),
  milliseconds,
  realtimeFactor: 1000 / milliseconds,
  pass: milliseconds <= 1000,
};
console.log(JSON.stringify(result, null, 2));
if (!result.pass) process.exitCode = 1;
