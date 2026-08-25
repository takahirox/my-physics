# Vehicle presets and parameter provenance

Vehicle presets select data, not different physical equations. All built-in
presets create the same `VehicleDefinition` schema and run through the same
chassis, tire, suspension, powertrain, road and collision code.

| Preset | Intended use | Explicit physical-data difference |
|---|---|---|
| `EngineeringReference` | transparent model validation and strict simulation | symmetric front/rear tire fitment scales of 1.0 |
| `RaceGameplay` | the browser circuit demo | rear cornering-stiffness scale 1.05 and rear peak-grip scale 1.06 |
| `ArcadeFun` | exaggerated game demo at `?demo=arcade` | authored mass/inertia, aero, front/rear tire fitment, suspension/brakes, anti-roll, engine and transmission values |

`VehicleDefinition::default()` remains an alias for the race preset so existing
v0.1 applications retain their established dynamics. New validation code must
select `engineering_reference()` explicitly. The demo also calls
`race_gameplay()` explicitly instead of relying on that compatibility alias.
There are no preset-only force multipliers or alternate equations. Tests
normalize the two documented rear values and require the complete definitions
to become equal.

`ArcadeFun` is derived from `EngineeringReference`. Its complete difference is
machine-tested: 1,160 kg dry mass; 520/1,180/1,320 kg·m² inertia; 0.34 drag and
-1.15 lift coefficients; front/rear cornering scales 1.18/1.238 and peak-grip
scales 1.24/1.234; 48/44 kN/m springs; 4.8/4.5 kN·s/m dampers; 4.1/3.7 kN·m
brakes; 18 kN·m/rad anti-roll; 0.16 kg·m² engine inertia and 1.48× torque;
4.10 final drive, 85 ms shifts and 900 N·m clutch capacity. Geometry, rack
travel, equations and tire-model code are unchanged. The separate Arcade
controller target is not part of the plant; a calibrated wheel stays 1:1 and
Simulation remains raw.

The current values are not measured. The engineering reference is a structured
authored/estimated baseline, while the race rear fitment is an authored balance
calibration. Arcade values are explicitly `Authored` revision `arcade-fun-v1`.
No preset claims OEM or tire-rig correlation.

## Arcade regression envelope

The deterministic 1 kHz headless fixture currently records: 0–100 km/h in
3.239 s; 100 km/h to 2 m/s in 26.478 m / 1.789 s; and 1° road-wheel ramp/hold
heading changes of 4.568°, 8.828° and 11.989° at 50/100/140 km/h. The 100 km/h
2°/0.5 Hz slalom reaches 0.382 rad/s yaw, 2.924° body slip, retains 84.1% speed
and reverses yaw seven times. The authored handbrake maneuver reaches 18.69°
body slip, recovers for a 200 ms stable window after 1.386 s without a yaw
reversal, and retains 60.1% of its starting speed at the declared 3.5 s
endpoint on arm64 macOS. The same test compiled for x86_64 and run through
Rosetta reaches 19.05°, recovers after 1.395 s, retains 59.5%, and also has no
yaw reversal. The acceptance gate is intentionally inside the product limit:
recovery must complete by 1.6 s, leaving at least 200 ms of margin before the
1.8 s target. The maneuver uses a prescribed countersteer phase followed by
rack unwind; continuing to hold opposite lock after alignment is a second
pendulum steering command, not recovery, and proved sensitive to platform
libm differences as well as unlike the browser/human input sequence.
The shared circuit AI also completes a damage-free lap with 2.486 m maximum
lateral error and 50.8 °C maximum tire temperature in the 110 s fixture.

The current tuning deliberately makes the handbrake the predictable drift
initiator. The matching lift-off fixture reaches only 4.56° body slip (below
the earlier 10–22° exploratory target and below the race baseline's roughly
6.2°). It is kept as an explicit, non-regressing finite/recovery gate rather
than presented as a success. Improving lift-off rotation without crossing the
observed narrow spin/recovery boundary requires a wider vehicle-data surface
(for example differential and damper curves) and remains follow-up work; no
hidden yaw damping or preset-only force was added to manufacture the result.

## Provenance schema

`VehicleParameterProvenance` has fixed metadata groups for:

- chassis mass properties;
- aerodynamics;
- front and rear wheels/tires;
- suspension;
- brakes;
- engine;
- transmission/clutch;
- fuel system.

Each group records an origin (`Measured`, `Derived`, `Fitted`, `Estimated` or
`Authored`), source, revision, optional relative one-sigma uncertainty and
named parameter validity ranges with units. `None` uncertainty means unknown,
not zero. Provenance is descriptive and cannot modify a physical value.

Every built-in group has a non-empty source, revision and machine-readable
validity coverage. A test verifies that every numeric built-in parameter lies
inside its declared range and that uncollected data is not marked `Measured`.

Snapshot format v3 stores provenance. Snapshot v1/v2 physical values remain
readable; because those formats did not retain sources, migrated metadata is
truthfully marked `Authored` with an unquantified uncertainty and a legacy
snapshot source. Migration never invents a measured origin.
