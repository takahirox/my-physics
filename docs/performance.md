# Performance

## v0.1 budget

The acceptance budget is one simulated second of the ten-vehicle reference world in less than one wall-clock second. The player plant evaluates at 1000 Hz; current non-player LOD force evaluations run at 1000, 250 or 100 Hz while every rigid body integrates at the 1 ms base tick.

The browser performs a warm-up and a 1,000-step physics-only benchmark at startup. Its result selects an automatic LOD ceiling:

- High: at least 4× real time
- Medium: at least 1.5× real time
- Low: below 1.5× real time

The application/user can override this from the Quality selector. Rendering remains outside the measured physics interval.

## Reference measurement — 2026-08-25

The final acceptance gate under Node v24.12.0 on the development Apple Silicon machine processed the ten-vehicle, 1,000-step WASM workload in **47.3 ms (21.2× real time)**. This includes nine deterministic AI controllers searching the 240-segment full-size circuit and collision broad phases for 480 synchronized barriers. The earlier compact 160-segment circuit measured 16.9 ms, so the fidelity/scale increase has a visible cost while retaining substantial real-time headroom. Measurements are workload/hardware specific and are evidence for the v0.1 reference setup, not a universal guarantee.

Run the portable WASM measurement with:

```bash
./scripts/build-wasm.sh
node scripts/benchmark-wasm.mjs
```

The live Chromium result is displayed in the demo as `10-CAR BENCH` and exposed as `window.__MY_PHYSICS_BENCHMARK__`.

The metric-detail WebGL scene was also sampled for 180 consecutive frames at approximately 100 km/h on the reference 120 Hz display. It averaged 8.33 ms per frame, with 9.3 ms p95, 9.4 ms maximum and no frame above 25 ms. Approximately 1,350–1,400 visible road-detail, barrier and vehicle box instances were submitted in one WebGL2 instanced draw call. This is a hardware-specific rendering check, separate from the deterministic physics benchmark.

At 100 km/h, 1.9 m curb bands pass at 14.6 Hz and 5 m fence posts at 5.56 Hz. A 5 m cue therefore spans about 10.8 frames at 60 Hz (21.6 frames on the measured 120 Hz display); consecutive Chromium captures 0.16–0.17 simulated seconds apart show about one post interval of physical travel.
