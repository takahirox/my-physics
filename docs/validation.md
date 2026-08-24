# Validation and acceptance evidence

Run the complete local gate with:

```bash
./scripts/verify-v01.sh
```

The integration suite currently covers:

| Test | Evidence |
|---|---|
| deterministic repeatability | independent runs produce the same full-state fingerprint |
| snapshot/replay equivalence | restore plus timed inputs reaches the original fingerprint |
| 1000 Hz priority timing | 1000 fixed steps advance both world and player exactly one second |
| render decoupling | different render batching reaches the identical state |
| longitudinal plausibility | full throttle accelerates the RWD reference car in local forward (-Z) |
| wet-road behavior | water reduces force and produces a non-zero hydroplaning state |
| tire failure progression | a finite puncture leaks pressure, changes patch state and advances failure |
| timestep guard | invalid variable timesteps are rejected |
| braking and brake heat | braking reduces speed and raises brake temperature |
| mutable mass and CG | consuming fuel changes both total mass and center of gravity |
| detached components | severe damage spawns an independent body and removes its mass |
| cumulative damage | thermal damage increases with exposure duration |
| persistent archives | complete snapshot equality, checksum rejection and timed-input round trip |
| analytical integration | semi-implicit constant-acceleration result matches its discrete closed form |
| force constraint | combined tire force remains within the computed friction circle |
| collision primitives | rotated box, capsule, convex and vehicle pair fixtures produce contacts/damage |
| effective failures | clutch/gearbox wear reaches physical failure and emits events |
| long-run stability | quaternion norm and finite state remain bounded in fixed/variable scenarios |
| smooth LOD | fidelity changes are bounded per tick and converge to the selected profile |
| continuous interfaces | Audio/FFB state and discrete physical events are populated |
| golden regression | the fixed two-second ten-vehicle scenario matches reviewed cross-platform telemetry tolerances |
| conservation/plausibility | pair-collision planar momentum and reference braking distance remain within bounds |

## Numerical strategy

The chassis uses semi-implicit Euler integration at 1 ms, which is robust for the current stiff but bounded forces. Tire forces are constrained by a combined-slip friction ellipse. Suspension travel, normal force and high-risk ratios are bounded. Every step rejects non-finite primary vehicle state.

## Required next validation

- Additional free-fall, yaw-inertia and energy-dissipation fixtures
- Skid pad, coast-down, braking-distance and steady-state cornering envelopes
- Parameter sweeps for timestep stability and thermal equilibrium
- Golden telemetry traces checked with tolerances, not only state hashes
- Comparison against instrumented reference-vehicle data
- Performance traces for additional hardware tiers and the v1.0 100-vehicle target
- Cross-platform determinism matrix and snapshot fuzz/property tests

No claim of real-vehicle correlation is made until measured telemetry and parameter provenance are available.
