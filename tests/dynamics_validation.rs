use my_physics::road::RoadCell;
use my_physics::tire::TireState;
use my_physics::{
    DriverInput, MagicFormulaTire, PhysicsWorld, Quat, SimulationConfig, Snapshot, TireInput, TireModel, Vec3,
    VehicleDefinition,
};

fn flat_vehicle(speed_mps: f64) -> PhysicsWorld {
    let mut definition = VehicleDefinition::race_gameplay();
    definition.transmission.automatic = false;
    let mut world = PhysicsWorld::new(SimulationConfig::default());
    let index = world.add_vehicle(definition);
    world.vehicles[index].state.powertrain.gear = 0;
    world.vehicles[index].driver_aids.traction_control_enabled = false;
    world.vehicles[index].driver_aids.stability_control_enabled = false;
    world.step_fixed(2_000).unwrap();

    let vehicle = &mut world.vehicles[index];
    vehicle.state.position_m = Vec3::new(0.0, 0.55, 0.0);
    vehicle.state.orientation = Quat::IDENTITY;
    vehicle.previous_position_m = vehicle.state.position_m;
    vehicle.previous_orientation = vehicle.state.orientation;
    vehicle.state.linear_velocity_mps = Vec3::new(0.0, 0.0, -speed_mps);
    vehicle.state.angular_velocity_rad_s = Vec3::ZERO;
    for (wheel, definition) in vehicle.state.wheels.iter_mut().zip(vehicle.definition.wheels.iter()) {
        wheel.angular_velocity_rad_s = speed_mps / definition.radius_m;
    }
    world
}

#[test]
fn dry_longitudinal_magic_formula_has_peak_and_locked_slide_envelope() {
    let model = MagicFormulaTire::default();
    let force = |slip| {
        model
            .evaluate(
                &mut TireState::default(),
                TireInput {
                    normal_load_n: model.nominal_load_n,
                    longitudinal_slip: slip,
                    slip_angle_rad: 0.0,
                    camber_rad: 0.0,
                    speed_mps: 30.0,
                    road: RoadCell::default(),
                    dt: 0.001,
                },
            )
            .longitudinal_force_n
            .abs()
    };
    let (peak_slip, peak_force) = (0..=200)
        .map(|index| index as f64 / 200.0)
        .map(|slip| (slip, force(slip)))
        .max_by(|left, right| left.1.total_cmp(&right.1))
        .unwrap();
    let locked_ratio = force(1.0) / peak_force;

    assert!((0.10..=0.20).contains(&peak_slip), "peak_slip={peak_slip}");
    assert!((0.70..=0.90).contains(&locked_ratio), "locked_ratio={locked_ratio}");
}

#[derive(Clone, Copy, Debug)]
struct BrakingResult {
    distance_m: f64,
    stopping_time_s: f64,
    locked_ratio: f64,
    target_ratio: f64,
    left_right_slip_rms: f64,
}

fn brake_from_100_kmh(abs_enabled: bool) -> BrakingResult {
    let mut world = flat_vehicle(100.0 / 3.6);
    world.vehicles[0].driver_aids.abs_enabled = abs_enabled;
    let start = world.vehicles[0].state.position_m;
    world.set_input_unrecorded(0, DriverInput { brake: 1.0, ..DriverInput::default() }).unwrap();
    let mut samples = 0_u64;
    let mut locked = 0_u64;
    let mut target = 0_u64;
    let mut symmetry_error_squared = 0.0;
    while world.vehicles[0].state.linear_velocity_mps.length() > 2.0 && samples < 10_000 {
        world.step_fixed(1).unwrap();
        let slip = world.vehicles[0].telemetry.wheel_slip;
        for slip in slip {
            locked += u64::from(slip < -0.9);
            target += u64::from((-0.30..=-0.05).contains(&slip));
        }
        symmetry_error_squared += (slip[0] - slip[1]).powi(2) + (slip[2] - slip[3]).powi(2);
        samples += 1;
    }
    assert!(samples < 10_000, "vehicle did not stop");
    let wheel_samples = samples * 4;
    BrakingResult {
        distance_m: (world.vehicles[0].state.position_m - start).length(),
        stopping_time_s: samples as f64 * 0.001,
        locked_ratio: locked as f64 / wheel_samples as f64,
        target_ratio: target as f64 / wheel_samples as f64,
        left_right_slip_rms: (symmetry_error_squared / (samples * 2) as f64).sqrt(),
    }
}

#[test]
fn persistent_abs_stops_in_dry_reference_envelope_without_locking() {
    let on = brake_from_100_kmh(true);
    let off = brake_from_100_kmh(false);

    assert!((31.0..=42.0).contains(&on.distance_m), "on={on:?}");
    assert!((1.8..=3.5).contains(&on.stopping_time_s), "on={on:?}");
    assert!(on.distance_m <= off.distance_m + 0.5, "on={on:?}, off={off:?}");
    assert!(on.locked_ratio < 0.02, "on={on:?}");
    assert!(on.target_ratio >= 0.80, "on={on:?}");
    assert!(on.left_right_slip_rms < 0.01, "on={on:?}");
}

#[test]
fn active_abs_pressure_round_trips_and_resimulates() {
    let mut original = flat_vehicle(100.0 / 3.6);
    original.set_input_unrecorded(0, DriverInput { brake: 1.0, ..DriverInput::default() }).unwrap();
    original.step_fixed(350).unwrap();
    assert!(original.vehicles[0].control.abs_active.iter().any(|active| *active));

    let archived = Snapshot::from_bytes(&original.snapshot().to_bytes()).unwrap();
    let mut restored = PhysicsWorld::new(SimulationConfig::default());
    restored.restore(&archived);
    original.step_fixed(1_000).unwrap();
    restored.step_fixed(1_000).unwrap();

    assert_eq!(restored.snapshot(), original.snapshot());
}

#[derive(Clone, Copy, Debug)]
struct CornerResult {
    peak_yaw_rate_rad_s: f64,
    peak_slip_rad: f64,
    final_yaw_rate_rad_s: f64,
    final_speed_mps: f64,
    final_forward_speed_mps: f64,
    final_sideslip_rad: f64,
    reversed_yaw: bool,
}

fn ramp_corner(speed_mps: f64, steer_angle_rad: f64, duration_steps: u32) -> CornerResult {
    let mut world = flat_vehicle(speed_mps);
    let normalized = steer_angle_rad / world.vehicles[0].definition.wheels[0].max_steer_rad;
    let expected_yaw_sign = -steer_angle_rad.signum();
    let mut peak_yaw_rate_rad_s: f64 = 0.0;
    let mut peak_slip_rad: f64 = 0.0;
    let mut reversed_yaw = false;
    for step in 0..duration_steps {
        let ramp = (f64::from(step + 1) / 2_000.0).min(1.0);
        world.set_input_unrecorded(0, DriverInput { steering: normalized * ramp, ..DriverInput::default() }).unwrap();
        world.step_fixed(1).unwrap();
        let vehicle = &world.vehicles[0];
        let yaw = vehicle.telemetry.yaw_rate_rad_s;
        peak_yaw_rate_rad_s = peak_yaw_rate_rad_s.max(yaw.abs());
        peak_slip_rad =
            peak_slip_rad.max(vehicle.state.wheels.iter().map(|wheel| wheel.slip_angle_rad.abs()).fold(0.0, f64::max));
        if step > 500 && yaw.abs() > 0.03 && yaw.signum() != expected_yaw_sign {
            reversed_yaw = true;
        }
    }
    let vehicle = &world.vehicles[0];
    let body_forward = vehicle.state.orientation.rotate(Vec3::FORWARD);
    let body_right = vehicle.state.orientation.rotate(Vec3::X);
    let final_forward_speed_mps = vehicle.state.linear_velocity_mps.dot(body_forward);
    let final_sideslip_rad =
        vehicle.state.linear_velocity_mps.dot(body_right).atan2(final_forward_speed_mps.abs().max(0.1));
    CornerResult {
        peak_yaw_rate_rad_s,
        peak_slip_rad,
        final_yaw_rate_rad_s: vehicle.telemetry.yaw_rate_rad_s,
        final_speed_mps: vehicle.telemetry.speed_mps,
        final_forward_speed_mps,
        final_sideslip_rad,
        reversed_yaw,
    }
}

#[test]
fn high_g_ramps_remain_stable_and_left_right_symmetric() {
    let model = MagicFormulaTire::default();
    for (speed_mps, steer_deg) in [(20.0_f64, 3.09_f64), (30.0_f64, 1.55_f64)] {
        let steer_rad = steer_deg.to_radians();
        let requested_acceleration = speed_mps * speed_mps * steer_rad.tan() / 2.51;
        assert!(requested_acceleration < model.peak_mu * 9.80665);

        let left = ramp_corner(speed_mps, steer_rad, 4_000);
        let right = ramp_corner(speed_mps, -steer_rad, 4_000);
        for result in [left, right] {
            assert!(!result.reversed_yaw, "speed={speed_mps}, result={result:?}");
            assert!(result.final_forward_speed_mps > speed_mps * 0.65, "speed={speed_mps}, result={result:?}");
            assert!(result.peak_slip_rad < 10.0_f64.to_radians(), "speed={speed_mps}, result={result:?}");
            assert!(result.final_sideslip_rad.abs() < 6.0_f64.to_radians(), "speed={speed_mps}, result={result:?}");
        }
        assert!(
            (left.peak_yaw_rate_rad_s - right.peak_yaw_rate_rad_s).abs() < 0.005,
            "speed={speed_mps}, left={left:?}, right={right:?}"
        );
        assert!(
            (left.final_speed_mps - right.final_speed_mps).abs() < 0.02,
            "speed={speed_mps}, left={left:?}, right={right:?}"
        );
        assert!(
            (left.final_yaw_rate_rad_s.abs() - right.final_yaw_rate_rad_s.abs()).abs() < 0.005,
            "speed={speed_mps}, left={left:?}, right={right:?}"
        );
    }
}

#[test]
fn low_g_ramp_matches_bicycle_yaw_rate_within_five_percent() {
    let steering = 0.5_f64.to_radians();
    let left = ramp_corner(20.0, steering, 4_000);
    let right = ramp_corner(20.0, -steering, 4_000);
    for result in [left, right] {
        let bicycle = result.final_speed_mps * steering.tan() / 2.51;
        let relative_error = (result.final_yaw_rate_rad_s.abs() - bicycle).abs() / bicycle;
        assert!(relative_error <= 0.05, "result={result:?}, bicycle={bicycle}, error={relative_error}");
    }
    assert!((left.final_yaw_rate_rad_s.abs() - right.final_yaw_rate_rad_s.abs()).abs() < 0.001);
}
