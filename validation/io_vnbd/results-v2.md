# IO-VNBD correlation result v2

Status: frozen-model real-data result; physical plausibility/correlation only.
This is not a measured tire fit, certification or safety validation.

## What changed before final evaluation

- Fixed a common-plant rolling-resistance energy inconsistency. Rolling
  resistance is now a continuous odd wheel-resisting moment and reaches the
  chassis through ordinary tire force. A no-relative-velocity-reversal impulse
  bound removes the explicit small-slip contact limit cycle without changing
  the authored high-speed rolling-resistance coefficient.
- Reconstructed accelerator demand as `pedal_fraction^0.30`. Calibration-only
  log/log inverse-balance medians were 0.294 (`V-Vw12`) and 0.384
  (`V-Vfb02c`). Calibration-derived 0.30 and 0.35 candidates were evaluated on
  Validation for model selection; no run-specific map was fitted. This effective map is conditional on the
  authored engine curve and is not a throttle-plate measurement.
- Admitted brake pressure only while discrete Brake Position is active (ZOH).
  The effective 200 psi full-command scale is a combined input-map/brake
  capacity/radius/mass product, not an identified hydraulic gain.
- Candidate body acceleration is the explicit boxcar mean of all 100 ordinary
  1 ms plant samples in each 100 ms source interval. No force or state is
  changed. `t0` is initialization only; scoring begins at 0.1 s.

The v2 baseline also uses this corrected common plant and input reconstruction.
It therefore isolates the calibrated wheel-radius/gear-ratio proxy choice, but
is not the same baseline as v1. Direct v1-to-v2 values below disclose the
changed score window and acceleration observation filter.

## Calibration evidence

With hash-verified `V-Vfb02c`, the runner selected 134 brake-active,
accelerator <= 0.01, centered-steering < 5 degree and pressure > 1 psi samples.
OLS gave 0.010441102 g/psi and intercept -0.0120414 g. The explicit proxy gives
2.232058586 g at full command, or 213.776 psi equivalent scale. Frozen 200 psi
differs by 6.44%, inside the implemented 10% runtime consistency gate and provisional
190–250 psi envelope. There is no bootstrap/CI uncertainty claim yet.

Calibration aggregate normalized RMSE after the plant/input corrections:

| Run | v2 baseline | v2 calibrated |
| --- | ---: | ---: |
| `V-Vw12` | 0.517842 | 0.427094 |
| `V-Vfb02c` | 0.425225 | 0.426893 |

`V-Vfb02c` is essentially neutral on the aggregate despite substantially
better brake-response correlation; this unfavorable result is retained.

## Validation model selection

The model-selection objective used was the sample-count-weighted mean of each
complete run's aggregate normalized RMSE; it was not separately preregistered.
For the calibrated proxy, exponent 0.30 scored
0.684337 versus 0.703976 for 0.35. `V-Vw7` favored 0.35, while `V-Vw16b` favored
0.30; 0.30 was frozen before any v2 holdout was rerun.

| Run | v2 baseline | v2 calibrated 0.30 | calibrated 0.35 |
| --- | ---: | ---: | ---: |
| `V-Vw7` | 0.569924 | 0.620368 | 0.574214 |
| `V-Vw16b` | 0.926008 | 0.775370 | 0.888642 |
| sample-weighted | 0.716877 | 0.684337 | 0.703976 |

The calibrated radius/gear proxy was retained because its two-run weighted
score improves the v2 baseline, despite worsening `V-Vw7` alone.

Selected calibrated per-channel RMSE, with v1 values for the same runs:

| Run/channel | v1 calibrated | v2 calibrated | Unit |
| --- | ---: | ---: | --- |
| `V-Vw7` speed | 4.2850 | 3.0756 | m/s |
| `V-Vw7` RPM | 820.68 | 698.17 | rpm |
| `V-Vw7` longitudinal acceleration | 3.6957 | 0.9589 | m/s2 |
| `V-Vw7` yaw | 0.16134 | 0.16918 | rad/s |
| `V-Vw16b` speed | 8.9733 | 5.0176 | m/s |
| `V-Vw16b` RPM | 1118.46 | 727.58 | rpm |
| `V-Vw16b` longitudinal acceleration | 2.2947 | 0.8078 | m/s2 |
| `V-Vw16b` yaw | 0.05166 | 0.05517 | rad/s |

The acceleration improvement is partly the corrected 10 Hz observation
operator. Yaw slightly worsens, so v2 is not claimed to dominate every output.

## Frozen holdout and post-observation regression results

Historical holdouts were rerun without model changes:

| Run | v2 baseline | v2 calibrated |
| --- | ---: | ---: |
| `V-Vta1b` | 0.915069 | 0.909679 |
| `V-vtb12` | 0.590563 | 0.441325 |

`V-Vta14`, `V-Vta30` and `V-Vfb02g` were initially opened before later generic
rolling-resistance/brake-contact fixes. They are therefore post-observation
regressions, not independent final evidence. This reclassification is retained
even though the fixes were dataset-independent. Steering/yaw/lateral
acceleration are excluded because steering semantics are not independently
observable. Gear is context only. The regression
score combines indicated speed, GPS speed, longitudinal acceleration, four
wheel speeds and RPM only on a constant valid gear over current +/-2 samples.

| Run | baseline final score | calibrated final score | Stable RPM samples | calibrated RPM RMSE |
| --- | ---: | ---: | ---: | ---: |
| `V-Vta14` | 1.429119 | 1.312794 | 2,721 | 744.74 rpm |
| `V-Vta30` | 0.875683 | 0.871549 | 16,157 | 694.62 rpm |
| `V-Vfb02g` | 1.447381 | 1.387574 | 26,380 | 748.43 rpm |

All three calibrated regression scores improve their v2 baselines, but the absolute
errors remain material. In particular, RPM retains a large negative bias and
the long-run scores are not evidence of an exact Fiesta variant correlation.

The true independent final suite is sealed as `V-Vw3` (synchronized 3,861
rows, pressure C) and `V-Vtb8` (699 rows, pressure A). Their path, SHA-256,
size, scenario and the same longitudinal/stable-gear-RPM policy are committed
before opening. No result belongs in this document until the clean committed
source runs each once.

## Reproduction and limits

```sh
scripts/fetch-io-vnbd.sh target/io-vnbd/raw --split calibration
scripts/fetch-io-vnbd.sh target/io-vnbd/raw --split validation
scripts/fetch-io-vnbd.sh target/io-vnbd/raw --split holdout
scripts/fetch-io-vnbd.sh --verify-only --output target/io-vnbd/raw --split all
# After the clean source commit only; --split all opens the sealed suite:
cargo run --release --bin correlate-io-vnbd -- \
  --data-root target/io-vnbd/raw --output target/io-vnbd-correlation-v2 --split all
```

The exact model year/engine/loading, CG/inertia, road grade, wind, tire
construction and steering sensor semantics remain unknown. Ten-hertz data
cannot identify ABS cycling, tire relaxation, shift transients or thermal
dynamics. Pressure-A versus D is confounded with route/weather and is not a
causal tire-pressure validation. Raw CSVs remain uncommitted because the
upstream data repository has no explicit dataset license.
