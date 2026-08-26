# IO-VNBD correlation result v1

This is the first measured-vehicle correlation baseline for `my-physics`. It
is an objective error report, not a pass/certification claim. Raw telemetry and
full aligned time series are deliberately not committed because the upstream
data repository has no explicit dataset redistribution license.

## Experiment identity

- Dataset snapshot: IO-VNBD commit
  `118939602e3422d47b8ab0807b623751c3ac135b`; every selected raw file is
  identified by byte count and SHA-256 in [`acquisition.tsv`](acquisition.tsv).
- Measured vehicle: the paper's front-wheel-drive Ford Fiesta Titanium. Exact
  model year, engine, gearbox, tires and test loading are unresolved.
- Plant: the normal deterministic `PhysicsWorld`, fixed `dt = 0.001 s`, no
  automatic LOD, no post-`t0` state injection, no IO-VNBD force branch.
- Controls: reconstructed steering/throttle/brake/gear input; ABS enabled,
  traction control and stability control disabled.
- Evaluation grid: measured 10 Hz sensor timestamps with no fitted per-run time
  shift, filtering or dynamic time warping.
- Scored outputs: speed, yaw rate, longitudinal/lateral acceleration, four
  individual wheel speeds and engine RPM.
- Split: three calibration runs, two validation runs and two untouched holdout
  runs, as frozen in [`acquisition.tsv`](acquisition.tsv).

The runner emits the software Git revision/worktree state, exact reference and
candidate fingerprints, the applied 1 ms input-sequence fingerprint, clocks,
vehicle revision and complete per-run metrics. Two complete executions from
the same source and raw files produced byte-identical artifact trees.

## Calibration changes

Only calibration runs contributed fitted values. `V-Vw1` supplied inertial and
brake sensor biases, `V-Vw12` supplied effective rolling radius, and
`V-Vw12`/`V-Vfb02c` supplied overall engine-to-wheel ratios for gears 2–6.
Final-drive factorization, steering ratio/zero, brake full-scale, mass, inertia,
tire coefficients and engine curve remain manufacturer analogues, estimates or
explicit assumptions—not fitted measurements. The complete ledger is
[`reference-vehicle.tsv`](reference-vehicle.tsv); generated runs also contain
`parameter-estimates.manifest` and `fit-trace.csv`.

The aggregate below is the RMS of each channel RMSE divided by a declared
physical normalization scale. It is dimensionless, but it is not a pass score
and has no fitted acceptance threshold. Negative change is improvement.

| Run | Role / maneuver | Baseline | Calibrated | Change |
|---|---|---:|---:|---:|
| `V-Vw1` | calibration / stationary | 0.662396 | 0.657173 | -0.005223 |
| `V-Vw12` | calibration / approximately straight steady speed | 1.823498 | 1.773552 | -0.049947 |
| `V-Vfb02c` | calibration / U-turn and hard braking | 0.944366 | 0.831578 | -0.112788 |
| `V-Vw7` | validation / successive turns | 0.993582 | 0.957009 | -0.036573 |
| `V-Vw16b` | validation / straight hard braking | 1.403049 | 1.354867 | -0.048182 |
| `V-Vta1b` | holdout / wet-muddy braking, pressure A | 1.309881 | 1.317339 | +0.007458 |
| `V-vtb12` | holdout / wet-night roundabout, pressure A | 0.957626 | 0.971836 | +0.014210 |

Calibration improved both validation runs but worsened both holdouts. That is
evidence that the limited fitted parameters do not generalize to the wet,
low-pressure holdout conditions; it is not hidden by selecting only favorable
runs.

## Independent channel results

These are calibrated-plant values. Bias is simulation minus measurement.

| Run | Speed RMSE / bias (m/s) | Speed corr. | Yaw RMSE / corr. (rad/s) | Long. accel. RMSE / corr. (m/s²) | Lat. accel. RMSE / corr. (m/s²) | RPM RMSE / bias |
|---|---:|---:|---:|---:|---:|---:|
| `V-Vw7` | 4.285 / -3.822 | 0.685 | 0.161 / 0.821 | 3.696 / 0.128 | 1.260 / 0.457 | 821 / -721 |
| `V-Vw16b` | 8.973 / -8.255 | 0.831 | 0.052 / 0.429 | 2.295 / 0.417 | 0.892 / 0.103 | 1,118 / -990 |
| `V-Vta1b` | 8.764 / -8.023 | 0.674 | 0.055 / -0.027 | 2.305 / 0.294 | 0.829 / 0.047 | 1,011 / -918 |
| `V-vtb12` | 6.117 / -5.528 | 0.897 | 0.112 / 0.573 | 1.823 / 0.690 | 1.178 / 0.214 | 809 / -686 |

Individual wheel-speed RMSE ranges were 15.35–15.47 rad/s (`V-Vw7`),
32.12–32.53 rad/s (`V-Vw16b`), 31.45–31.59 rad/s (`V-Vta1b`) and
22.00–22.07 rad/s (`V-vtb12`). Full artifacts report MAE, maximum error, bias,
R², correlation, signed peak error/time, bounded diagnostic lag, range and
sample count for every channel. The diagnostic lag never changes the samples
used by the error metrics.

The strongest current result is response-shape correlation in some excited
signals: `V-Vw7` yaw correlation is 0.821 and `V-vtb12` speed correlation is
0.897. Absolute accuracy is not yet strong. Large negative speed/RPM/wheel
biases show that the open-loop powertrain/load/input reconstruction loses too
much speed. Acceleration correlation is weak in most runs. In the independent
hard-brake run `V-Vw16b`, the measured minimum after the first >5 psi brake
event occurs at 54.2 s and the calibrated simulation minimum at 50.8 s. In the
holdout `V-Vta1b`, those times are 83.1 s versus 32.8 s; event segmentation is
only exploratory because later unrelated events may contribute to a run-wide
minimum.

## What the evidence does and does not establish

Currently supported:

- the same common plant can replay measured inputs at 1 kHz and compare them
  reproducibly against multiple independent 10 Hz vehicle-dynamics channels;
- FWD torque delivery, intact under-inflation behavior, per-wheel telemetry,
  strict split enforcement and deterministic result generation are exercised;
- calibration/validation/holdout behavior and failure to generalize are
  quantified rather than judged visually.

Not established:

- an exact Fiesta model or measured tire model—the exact vehicle specification
  is absent;
- causal tire-pressure sensitivity—the pressure-A holdouts are also wet/muddy
  and use different routes/maneuvers, so pressure and surface are confounded;
- ABS, tire-relaxation, shifting, suspension or thermal transients—the source
  sampling rate is 10 Hz;
- road-grade, water-depth, wind or driver-command truth. Water depth is kept
  neutral rather than invented to improve wet-run agreement;
- certification, safety validity or general accuracy outside these runs.

Likely next improvements are better steering/pedal/gear input identification,
independent measurement of test mass/CG/inertia/tire construction, a calibrated
engine/load model, matched dry/wet and pressure-controlled repetitions, and
higher-rate CAN/IMU data. Those changes belong in normal vehicle parameters or
replaceable physical models, never an IO-VNBD-specific correction force.

## Reproduce

```sh
scripts/fetch-io-vnbd.sh target/io-vnbd/raw
cargo run --release --bin correlate-io-vnbd -- \
  --data-root target/io-vnbd/raw \
  --output target/io-vnbd-correlation \
  --split all
```

For stricter scientific workflow, fetch and execute `calibration`, then
`validation`, then `holdout` in separate calls. Inspect `summary.json`, each
run's `correlation-report.json`, `metrics.csv`, `event-metrics.json`,
`run-provenance.json` and `timeseries.svg`. Re-run into a second directory and
compare the trees to verify determinism.
