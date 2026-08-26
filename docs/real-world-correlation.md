# Real-world correlation framework

This application-layer framework compares licensed measured telemetry with an
unmodified simulation. It does not add correction forces, state nudging, or a
dataset branch to the physical plant. State may be initialized at `t0`; every
later sample is produced by the ordinary 1 ms simulation and declared inputs.

## Reproducibility contract

Every external run has a versioned text manifest containing:

- dataset, vehicle-proxy, session, run/journey split-group and source identity;
- pinned content checksum and a separately recorded license-verification flag;
- sensor timestamp semantics, expected sample period and maximum accepted gap;
- exact source-column to canonical-field mapping;
- physical quantity, SI unit, coordinate frame and input/observation role;
- declared affine/unit or wrapped-angle conversion;
- per-channel linear (FOH) or previous-sample (ZOH) resampling;
- frozen mapping, alignment and filter revisions plus provenance.

The strict CSV adapter rejects missing/duplicate columns, duplicate or
non-monotonic timestamps, undeclared gaps and non-finite values. Parquet and
provider-specific formats implement the same `TelemetryAdapter` boundary; the
core deliberately does not guess a Parquet schema or silently reinterpret a
column.

Clock alignment permits only a declared bounded affine correction (scale
0.98–1.02, offset within ±60 s) and non-negative declared latency up to 5 s.
There is no DTW or signal-dependent time warp. The comparison grid covers only
the common time interval and never extrapolates.

## Split policy

`ManifestCatalog` prevents the same run/journey group from appearing in more
than one split. CLI purpose is enforced mechanically:

| Purpose | Permitted split |
| --- | --- |
| parameter fitting | training |
| model selection | validation |
| final evaluation | test |

Adapter mappings, conversion, alignment, latency and filtering are calibration
artifacts and must be frozen before validation/holdout evaluation. Fitted
physical parameters must name training provenance; `ParameterEstimateArtifact`
rejects fitted values attributed to validation or test data.

## Reports

`correlate-telemetry` produces deterministic `correlation-report.json`,
`metrics.csv` and `aligned-timeseries.csv`. Per-channel metrics retain their
declared unit and include sample count, MAE, RMSE, maximum absolute error,
bias, R² and Pearson correlation where defined, signed peak value/time error,
bounded informational best lag, and reference coverage. The best-lag result is
diagnostic only: it never shifts the samples used by the error metrics. A
cross-channel aggregate is formed only from RMSE divided by each channel's
explicit normalization scale, so dimensional errors are never added together.
Reports identify the source checksum, vehicle/session/split,
candidate revision, mapping/alignment/filter revisions and license status.

```text
cargo run --release --bin correlate-telemetry -- \
  --reference-manifest measured.manifest --reference measured.csv \
  --candidate-manifest simulation.manifest --candidate simulation.csv \
  --purpose final-evaluation --report-id holdout-v1 --output artifacts \
  --sample-period 0.1
```

`--require-publishable-license` is an additional release gate. Raw third-party
data must remain outside this repository unless its dataset license—not merely
an associated article license—has been reviewed and permits redistribution.

Correlation quantifies only the declared datasets/scenarios. It is not proof
of general model validity, a measured tire fit, certification or safety
qualification.
