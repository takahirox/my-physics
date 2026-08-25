# Vehicle presets and parameter provenance

Vehicle presets select data, not different physical equations. Both built-in
presets create the same `VehicleDefinition` schema and run through the same
chassis, tire, suspension, powertrain, road and collision code.

| Preset | Intended use | Explicit physical-data difference |
|---|---|---|
| `EngineeringReference` | transparent model validation and strict simulation | symmetric front/rear tire fitment scales of 1.0 |
| `RaceGameplay` | the browser circuit demo | rear cornering-stiffness scale 1.05 and rear peak-grip scale 1.06 |

`VehicleDefinition::default()` remains an alias for the race preset so existing
v0.1 applications retain their established dynamics. New validation code must
select `engineering_reference()` explicitly. The demo also calls
`race_gameplay()` explicitly instead of relying on that compatibility alias.
There are no preset-only force multipliers or alternate equations. Tests
normalize the two documented rear values and require the complete definitions
to become equal.

The current values are not measured. The engineering reference is a structured
authored/estimated baseline, while the race rear fitment is an authored balance
calibration. Neither preset claims OEM or tire-rig correlation.

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
