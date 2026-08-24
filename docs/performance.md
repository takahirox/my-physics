# Performance

## v0.1 budget

The acceptance budget is one simulated second of the ten-vehicle reference world in less than one wall-clock second. The player plant evaluates at 1000 Hz; current non-player LOD force evaluations run at 1000, 250 or 100 Hz while every rigid body integrates at the 1 ms base tick.

The browser performs a warm-up and a 1,000-step physics-only benchmark at startup. Its result selects an automatic LOD ceiling:

- High: at least 4× real time
- Medium: at least 1.5× real time
- Low: below 1.5× real time

The application/user can override this from the Quality selector. Rendering remains outside the measured physics interval.

## Reference measurement — 2026-08-24

Desktop Chromium on the development Apple Silicon machine processed the final ten-vehicle, 1,000-step WASM workload in **16.7 ms (59.9× real time)**. The final acceptance gate under Node v24.12.0 measured **16.9 ms (59.1× real time)** on the same machine. Measurements are workload/hardware specific and are evidence for the v0.1 reference setup, not a universal guarantee.

Run the portable WASM measurement with:

```bash
./scripts/build-wasm.sh
node scripts/benchmark-wasm.mjs
```

The live Chromium result is displayed in the demo as `10-CAR BENCH` and exposed as `window.__MY_PHYSICS_BENCHMARK__`.
