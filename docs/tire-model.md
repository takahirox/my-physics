# Reference tire: transient slip and thermal model

The v0.1 reference remains a Magic-Formula-family approximation. It is not a
licensed Pacejka data set, a transient brush model or a claim of correlation to
a particular tire. All constants below are `Authored`; `MagicFormulaTire::parameter_provenance()`
exposes their source, revision and validity ranges.

## Lateral force and aligning moment

The lateral shape uses

`Sy = sin(Cy atan(By α - Ey(By α - atan(By α))))`

with `By = lateral_stiffness / Cy`. Therefore the zero-slip gradient remains
exactly the prior `lateral_stiffness`, while the authored `Cy=1.35`, `Ey=-1.0`
create a finite peak and sliding branch. At nominal load/temperature the peak
is 15.24 degrees and force at 45.84 degrees is 92.29% of peak. Previously the
curve was still increasing at the 45.84-degree test boundary.

Pneumatic trail is multiplied by `exp(-(abs(alpha_eff)/0.14)^2)`. At the force
peak the effective trail is 2.83% of its low-slip value; the old trail was
constant. This is an authored collapse shape, not a measured overturning or
residual-aligning-torque model.

## Distance-based transient slip

Each `WheelState` stores kinematic slip, relaxed slip and relaxation length.
Force uses the relaxed value:

`alpha_next = target + (alpha_previous - target) exp(-abs(Vx) dt / sigma)`

The reference relaxation length is an authored 0.45 m, adjusted explicitly for
load, pressure and carcass damage. One relaxation length reaches 63.21% and
three reach 95.02%, independent of vehicle speed and integration step. Below
1.5 m/s the target fades to zero and transport speed is bounded only for stable
release; this low-speed regularization is documented behavior.

Friction work does not use the relaxed angle. The world passes the actual
lateral contact velocity separately, so a residual transient force after the
contact stops sliding does not invent heat.

## Two-node thermal energy model

Tread and bulk/carcass temperatures are separate energy nodes:

| Authored parameter | Value |
|---|---:|
| effective tread heat capacity | 14,000 J/K |
| effective bulk heat capacity | 38,000 J/K |
| slip-work fraction to tread | 0.82 |
| tread–bulk conductance | 120 W/K |
| loaded tread–road conductance | 65 W/K |
| still-air bulk conductance | 18 W/K |
| speed-air increment | 1.1 W/K per m/s |

The remaining 18% of friction work and signed tread/road conduction are passed
to the dynamic road as road heat. Rubber deposition and water removal continue
to use total mechanical slip work. Tests close the tread+bulk+road+air energy
ledger to numerical precision and check 0.5–20 ms convergence.

These effective capacities and conductances are regression-calibrated authored
bands. They have not been identified from calorimetry, pyrometer, carcass
thermocouple or tire-rig data. The two-node model omits tread-depth gradients,
gas temperature/pressure coupling, detailed footprint dwell, rim heat flow and
material phase changes.

## Before/after regression evidence

The following same-fixture results use the race preset, dry flat road and two
seconds of input. Temperatures are absolute Celsius. “Half pad” is raw
half-stick after the default balanced-gamepad normalization (`0.317752`).

| Fixture | Old tread / bulk | New tread / bulk | Old minimum mu | New minimum mu |
|---|---:|---:|---:|---:|
| 100 km/h half pad | 264.71 / 64.02 C | 57.08 / 49.97 C | 0.663 | 0.995 |
| 140 km/h half pad | 371.36 / 71.04 C | 62.40 / 49.97 C | 0.661 | 0.995 |
| 100 km/h, fixed 25.8-degree slip | 195.90 / 62.38 C | 56.48 / 49.94 C | 0.706 | 1.177 |
| 140 km/h, fixed 25.8-degree slip | 238.33 / 66.08 C | 59.26 / 49.93 C | 0.699 | 1.177 |

After ten seconds released, the fixed-slip tread/bulk results changed from
133.97/112.14 C to 54.58/49.74 C at 100 km/h, and from 161.03/131.45 C to
57.01/49.72 C at 140 km/h. Temperature is continuous at release; cooling and
tread-to-bulk transfer occur through the energy equations.

Selected 0.5-second-ramp/two-second vehicle traces demonstrate that the change
does not weaken the low-g response while preventing thermal runaway at high
demand:

| Speed / road-wheel steer | Old peak yaw | New peak yaw | Old max slip | New max slip | Old/New peak tread |
|---|---:|---:|---:|---:|---:|
| 50 km/h / 0.5 deg | 0.04691 | 0.04716 rad/s | 0.366 | 0.365 deg | 47.88 / 49.79 C |
| 100 km/h / 1 deg | 0.17998 | 0.17620 rad/s | 2.93 | 2.72 deg | 52.12 / 49.79 C |
| 100 km/h / 2 deg | 0.36840 | 0.33577 rad/s | 7.33 | 5.77 deg | 78.56 / 50.43 C |
| 140 km/h / 1 deg | 0.24532 | 0.22335 rad/s | 5.96 | 4.89 deg | 75.32 / 50.36 C |
| 140 km/h / 2 deg | 0.51275 | 0.39478 rad/s | 19.60 | 9.94 deg | 149.09 / 52.37 C |

The 20 m/s, 0.5-degree low-g steady-ramp bicycle yaw error remains 2.97%
(previously 2.99%), and left/right results remain mirror-equal within numerical
precision. These are authored regression results, not real-vehicle validation.

The reviewed two-second golden trace changed because the dimensionally explicit
thermal state slightly changes temperature-dependent longitudinal grip:

| Channel | Old | New | Relative change |
|---|---:|---:|---:|
| speed | 7.748260 | 7.718630 m/s | -0.38% |
| forward position | -7.177284 | -7.144119 m | -0.46% magnitude |
| engine speed | 3673.300 | 3660.234 rpm | -0.36% |
| fuel | 39.993888 | 39.993944 kg | +0.000056 kg |
| front tread | 321.616 | 322.992 K | +1.376 K |
| rear tread | 323.220 | 322.996 K | -0.225 K |

The differences are finite, small in vehicle motion, and directly attributable
to the replaced thermal equations; the golden centers were updated rather than
loosening their tolerances.
