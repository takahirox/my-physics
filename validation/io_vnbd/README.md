# IO-VNBD real-world vehicle correlation

This directory defines the reproducible IO-VNBD data boundary for
`my-physics`. It deliberately contains metadata, source identities, conversion
rules and split policy—but no third-party raw telemetry. The reusable
simulation/alignment/reporting design is documented in
[`docs/real-world-correlation.md`](../../docs/real-world-correlation.md).

## Primary sources and pinned snapshot

- U. Onyekpe, V. Palade, S. Kanarachos and A. Szkolnik,
  “IO-VNBD: Inertial and Odometry benchmark dataset for ground vehicle
  positioning,” *Data in Brief* 35 (2021) 106885,
  <https://doi.org/10.1016/j.dib.2021.106885>.
- Publisher-authorized manuscript copy at Coventry University,
  <https://pure.coventry.ac.uk/ws/portalfiles/portal/40741559/Binder8.pdf>.
- Public data repository, <https://github.com/onyekpeu/IO-VNBD>, pinned here to
  commit [`118939602e3422d47b8ab0807b623751c3ac135b`](https://github.com/onyekpeu/IO-VNBD/tree/118939602e3422d47b8ab0807b623751c3ac135b).

The repository has no versioned data release, so an immutable Git commit plus
the SHA-256 Git LFS object ID and byte size of every selected file are the data
identity. The upstream directory name `Synchronised V abd S datasets` contains
an apparent typo; the acquisition manifest preserves it exactly.

The paper describes approximately 40 hours / 1,300 km of vehicle-extracted
data and 58 hours / 4,400 km of smartphone data. This work uses the 29-channel
vehicle/CAN (`V-`) CSV files, not smartphone IMU as a substitute for vehicle
dynamics telemetry. The VBOX vehicle data are nominally 10 Hz. All selected
materialized files have an explicit sample-period value of 0.1 s at every row.

## License and redistribution boundary

The *article* is explicitly published under CC BY 4.0. As inspected on
2026-08-26, the pinned GitHub data repository contains no `LICENSE`, `COPYING`
or dataset-specific terms, and GitHub reports no detected license. An article
license must not silently be treated as a raw-dataset redistribution license.

Consequently:

- raw IO-VNBD CSV/JPG/ZIP data are not committed or repackaged here;
- dataset manifests must set `license_verified=false` and must not claim an
  SPDX dataset license until the data owner supplies or confirms terms;
- publication/release jobs using `--require-publishable-license` correctly
  reject these manifests in their current state;
- the acquisition script downloads selected objects directly from the public
  upstream and verifies them locally; users remain responsible for confirming
  that their use is permitted.

This is a conservative engineering boundary, not legal advice. If authoritative
dataset terms are later found, record their URL, version/date and reviewer in a
new manifest revision; do not rewrite the historical result.

## Measured vehicle: what is and is not known

The paper identifies the research vehicle used for the vehicle/CAN data as a
**front-wheel-drive Ford Fiesta Titanium**. It does not identify a model year,
generation, engine variant, test mass/loading, gearbox ratios, tire size, CG or
inertia tensor. Smartphone recordings also involved other vehicles; that fact
does not change the identity of these selected `V-` runs.

[`source-facts.tsv`](source-facts.tsv) is the auditable source ledger. Unknown
values stay `unresolved`. A reference vehicle may use an explicitly identified
manufacturer value for a declared generation assumption, an engineering
estimate, or a calibration fit, but each must retain the corresponding
`published`, `assumed`, `estimated` or `fitted` provenance. It must never be
relabeled as a dataset measurement.

The frozen per-parameter implementation ledger is
[`reference-vehicle.tsv`](reference-vehicle.tsv). The measured correlation
baseline, including unfavorable holdout results, is recorded in
[`results-v1.md`](results-v1.md).

The common plant must use its ordinary front-driven-wheel configuration for
this vehicle. IO-VNBD-specific yaw corrections, grip multipliers, damping,
steering assists, state nudging or force branches are prohibited. Sensor bias,
input reconstruction and frame/unit conversion belong in the dataset adapter,
not the physics plant.

## Vehicle channel map and conversion policy

Column order and names below are from the materialized CSV header and Table 3
of the paper. The adapter matches complete source names rather than relying on
column position.

| # | Source channel | Published/header unit | Canonical treatment |
|---:|---|---|---|
| 1 | GPS satellites available | count | context; integer |
| 2 | time since start of day | s | `time_s = source - first_source_time`; monotonicity required |
| 3–4 | GPS latitude / longitude | degree | context only; do not publish route data in reports |
| 5 | GPS velocity | km/h | m/s, multiply by `1/3.6` |
| 6 | GPS heading | degree | unwrap then radian; relative heading only |
| 7 | GPS height | header/paper document km | quarantined: selected values near 91 are physically plausible as metres, not 91 km; do not convert or infer grade until independently verified |
| 8 | GPS vertical velocity | km/h | m/s, multiply by `1/3.6` |
| 9 | sample period | s | quality/context; expected 0.1 s |
| 10 | steering angle | degree | radian; sensor location/ratio is unresolved and must not be guessed |
| 11–14 | wheel speed FL/FR/RL/RR | rad/s | SI identity; four observations remain distinct |
| 15 | yaw rate | degree/s | rad/s, multiply by `pi/180`; positive correlates with a left turn and matches positive internal yaw about +Y |
| 16 | indicated vehicle speed | km/h | m/s, multiply by `1/3.6` |
| 17–18 | indicated longitudinal/lateral acceleration | g | m/s², multiply by standard gravity 9.80665; negate dataset lateral acceleration for the engine's body-right +X convention; bias must be declared |
| 19 | handbrake | 0/1 | discrete input/context; previous-sample hold |
| 20–21 | gear requested / actual | header says 1–5 | quarantined until semantics are identified: requested contains 6 in selected data (and 14 elsewhere), while actual is often stuck at 3; discrete previous-sample hold only after validation |
| 22 | engine speed | rev/min | rad/s, multiply by `2*pi/60`; retain RPM for plots |
| 23 | coolant temperature | degree Celsius | kelvin for plant boundary, Celsius for human-facing plots |
| 24 | clutch position | 0/1 | discrete input; polarity must be validated from transitions |
| 25 | brake pressure | psi | Pa, multiply by 6,894.757293168 after declared sensor-zero treatment |
| 26 | brake position | 0/1 | discrete input/context; previous-sample hold |
| 27 | battery voltage | V | context |
| 28 | air temperature | degree Celsius | kelvin for plant boundary |
| 29 | accelerator pedal position | CSV header says “0 or 1”; paper Table 3 says percent activation | interpret source magnitude as percent with a header-quality flag, then divide by 100 for pedal fraction; pedal-to-engine torque response remains an identified vehicle/input parameter |

Continuous observations use linear interpolation only for evaluation-grid
sampling. Driver inputs and discrete state use previous-sample hold. Physics
still integrates at its configured 1 ms fixed step; the 10 Hz measurement does
not become a 10 Hz plant timestep and must not be upsampled to invent transient
content.

### Frames and sensor semantics

IO-VNBD does not fully document all CAN signal conventions, sensor mounting
point, steering sensor location/ratio, pedal scaling or acquisition latency.
These are identification variables with provenance. A sign/axis decision must
be frozen using calibration data before validation. It is invalid to choose a
different sign, latency or steering ratio per validation/holdout run because
it produces hidden fitting.

Negative near-zero brake pressure and non-zero stationary inertial signals are
present in selected data. A stationary-run bias estimate is allowed as sensor
preprocessing when it is recorded as a calibration artifact; it is not a force
correction. Road grade, wind, tire construction, load and exact road friction
remain unobserved disturbances.

## Tire pressure conditions

The paper's scenario table defines these pressure codes in the order front
right (FR), front left (FL), rear right (RR), rear left (RL), in psi:

| Code | FR | FL | RR | RL |
|---|---:|---:|---:|---:|
| A | 16 | 15 | 14 | 14 |
| B | 31 | 31 | 25 | 25 |
| C | 33 | 33 | 31 | 27 |
| D | 33 | 33 | 26 | 26 |
| E | unavailable | unavailable | unavailable | unavailable |

This selection contains A, C and D. Pressure A holdout runs are also wet or
muddy, while most D runs are on different routes/conditions. Therefore a
pressure-A versus pressure-D difference is **confounded by surface, weather,
maneuver, speed and route**. It may be reported as an exploratory trend but is
not causal validation of pressure sensitivity. A matched repeat or controlled
test dataset is required for that claim.

## Frozen run split

[`acquisition.tsv`](acquisition.tsv) is the machine-readable selection. Runs,
not rows or windows from a run, are the unit of separation.

| Split | Runs | Use |
|---|---|---|
| Calibration | `V-Vw1`, `V-Vw12`, `V-Vfb02c` | stationary sensor bias; steady/straight driveline observations; U-turn and hard-brake steering/brake excitation |
| Validation | `V-Vw7`, `V-Vw16b` | successive-turn model selection; independent straight hard-brake check |
| Holdout | `V-Vta1b`, `V-vtb12` | final wet/muddy braking and wet/night roundabout evaluation after parameters and mappings are frozen |
| CI smoke only | `V-Vw17` | short parser/simulation/report path; never parameter fitting or final evidence |

The split exercises longitudinal and lateral behavior while keeping every run
in exactly one role. All selected runs are Driver E and the same research
vehicle, so holdout independence is by journey/maneuver—not by driver or
vehicle. That limitation must accompany conclusions.

Selected raw-file audit at the pinned snapshot (GPS coordinates excluded):

| Run | Rows | Duration (s) | Indicated speed range (km/h) | Notable quality observation |
|---|---:|---:|---:|---|
| `V-Vw1` | 20,475 | 2,047.4 | 0.00–0.00 | stationary; useful for sensor/noise characterization, not dynamics fit |
| `V-Vw12` | 918 | 91.7 | 82.69–97.07 | steering varies 3.7–16.3 degrees despite approximately straight scenario |
| `V-Vfb02c` | 640 | 63.9 | 2.13–52.39 | vehicle-only file; brake pressure reaches about 48 psi |
| `V-Vw7` | 1,602 | 160.1 | 0.00–41.83 | strong steering/yaw/lateral excitation |
| `V-Vw16b` | 1,126 | 112.5 | 1.57–85.89 | hard-brake excitation; brake pressure reaches about 78 psi |
| `V-Vta1b` | 953 | 95.2 | 0.00–77.91 | steering is exactly zero throughout; do not score steering response |
| `V-vtb12` | 447 | 44.6 | 22.50–71.38 | wet roundabout lateral holdout |

These are descriptive checks, not acceptance thresholds. Missing/unusable
channels are excluded with a recorded reason, never replaced with fabricated
measurements.

## Acquisition and verification

Prerequisites for fetching are Git and Git LFS. Listing and dry-run modes need
no network and write nothing:

```sh
scripts/fetch-io-vnbd.sh --list
scripts/fetch-io-vnbd.sh --dry-run --split calibration
```

Fetch only calibration first, using a local cache. The script checks the source
commit, the repository's LFS pointer OID, materialized SHA-256 and byte size
before copying a read-only CSV to `target/io-vnbd/raw`:

```sh
scripts/fetch-io-vnbd.sh target/io-vnbd/raw --split calibration
scripts/fetch-io-vnbd.sh --verify-only --output target/io-vnbd/raw --split calibration
```

After mappings and fitted parameters have been frozen, fetch validation and
holdout independently:

```sh
scripts/fetch-io-vnbd.sh target/io-vnbd/raw --split validation
scripts/fetch-io-vnbd.sh --verify-only --output target/io-vnbd/raw --split validation
scripts/fetch-io-vnbd.sh target/io-vnbd/raw --split holdout
scripts/fetch-io-vnbd.sh --verify-only --output target/io-vnbd/raw --split holdout
```

An explicit external path is supported with `--output PATH`; `--cache PATH`
controls the Git/LFS cache. The default output is under the ignored `target/`
tree, and the repository-wide `*.csv` ignore rule is an additional guard
against accidentally staging raw or generated telemetry.

Run the complete IO-specific workflow with an external or ignored data root:

```sh
scripts/fetch-io-vnbd.sh /path/to/io-vnbd-data
cargo run --release --bin correlate-io-vnbd -- \
  --data-root /path/to/io-vnbd-data \
  --output target/io-vnbd-correlation \
  --split all
```

For scientific separation, prefer three calls in the protocol order using
`--split calibration`, then `validation`, then `holdout`. The correlation
runner writes deterministic per-run artifacts below
`<output>/<split>/<run-id>/`: `correlation-report.json`, `metrics.csv`,
`aligned-timeseries.csv` and `simulation.csv`. The output root also contains
`summary.json`, split-specific summaries, `parameter-estimates.manifest`,
`fit-trace.csv` and `limitations.md`. These
artifacts, the my-physics Git revision and the acquisition manifest revision
together identify a result.

## Calibration, validation and holdout protocol

1. Record checksums, adapter/mapping revision and initial reference-vehicle
   provenance before running physics.
2. Use `V-Vw1` only to characterize stationary bias/noise. Use `V-Vw12` and
   `V-Vfb02c` for identifiable input/driveline/steering/brake parameters.
3. Freeze unit/frame conversions, input reconstruction, time offset/latency,
   filtering, physical parameters and optimizer settings.
4. Evaluate `V-Vw7` and `V-Vw16b`. Model changes selected from those results
   create a new revision and require validation to be rerun.
5. Open and evaluate `V-Vta1b` and `V-vtb12` once for the final report. A model
   changed after inspecting holdout is a new experiment; the old holdout result
   remains in history and new independent data are needed for an uncontaminated
   final claim.

Each maneuver starts from its measured initial speed, wheel speeds, actual gear
and engine speed where valid. Later state is produced by the common deterministic
plant under reconstructed measured inputs; there is no within-window state
reset. The run manifest must record any excluded warm-up interval. Windows from
one run never cross splits.

## Quantitative evidence and interpretation

For every scored channel, publish aligned measured/simulated/error time series,
sample count, overlap, MAE, RMSE, maximum absolute error, signed bias, R² where
defined and measured range/coverage. Maneuver summaries additionally record
peak magnitude/error, steady-state error where the run has a defensible steady
interval, rise/settling response where excited, and bounded cross-correlation
lag as a diagnostic. Lag is reported; it must not shift each result to make the
curves look better. No dynamic time warping is permitted.

At minimum, score available and valid combinations of indicated/GPS speed,
longitudinal and lateral acceleration, yaw rate, four wheel speeds and engine
RPM. Gear, pedal/clutch/brake signals are primarily reconstructed inputs or
contexts; response is assessed in the output signals. `V-Vta1b` has no usable
steering-angle excitation even though it remains valuable longitudinal holdout.

Do not invent a “pass” threshold from the observed error. Report the baseline
objectively, both before and after calibration, and retain both artifacts. The
final analysis separates:

- areas currently correlated well;
- areas with material bias, amplitude, phase or peak error;
- vehicle-parameter uncertainty and identifiability;
- input reconstruction and sensor limitations;
- missing plant physics and road/environment uncertainty.

Poor correlation is a valid result. It is not permission to add a dataset-only
plant branch. Results characterize this Fiesta proxy, these maneuvers and these
uncertainties; they are not vehicle certification or general proof of model
validity.

## Reuse with another real-world dataset

Only acquisition and provider-specific schema/input reconstruction belong in a
new adapter. The following remain common: SI telemetry schema, provenance,
run-group split enforcement, fixed-step simulation, snapshot/replay identity,
bounded clock alignment, metric calculation and deterministic report artifacts.
That boundary supports later NHTSA, LiRA, comma2k19, OEM or motorsport telemetry
without adding physics corrections for each dataset.
