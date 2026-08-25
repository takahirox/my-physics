use my_physics::road::RoadCell;
use my_physics::tire::{TireState, transient_slip_step};
use my_physics::{
    DriverInput, MagicFormulaTire, ParameterOrigin, PhysicsWorld, Quat, SimulationConfig, Snapshot, TireInput,
    TireModel, Vec3, VehicleDefinition,
};

fn flat_vehicle(speed_mps: f64) -> PhysicsWorld {
    let mut definition = VehicleDefinition::race_gameplay();
    definition.transmission.automatic = false;
    let mut world = PhysicsWorld::new(SimulationConfig::default());
    let index = world.add_vehicle(definition);
    world.vehicles[index].state.powertrain.gear = 0;
    world.vehicles[index].driver_aids.abs_enabled = false;
    world.vehicles[index].driver_aids.traction_control_enabled = false;
    world.vehicles[index].driver_aids.stability_control_enabled = false;
    world.step_fixed(2_000).unwrap();
    reset_motion(&mut world, speed_mps);
    world
}

fn reset_motion(world: &mut PhysicsWorld, speed_mps: f64) {
    let vehicle = &mut world.vehicles[0];
    vehicle.state.position_m = Vec3::new(0.0, 0.55, 0.0);
    vehicle.state.orientation = Quat::IDENTITY;
    vehicle.previous_position_m = vehicle.state.position_m;
    vehicle.previous_orientation = vehicle.state.orientation;
    vehicle.state.linear_velocity_mps = Vec3::new(0.0, 0.0, -speed_mps);
    vehicle.state.angular_velocity_rad_s = Vec3::ZERO;
    for (wheel, definition) in vehicle.state.wheels.iter_mut().zip(vehicle.definition.wheels.iter()) {
        wheel.angular_velocity_rad_s = speed_mps / definition.radius_m;
    }
}

#[test]
fn lateral_curve_preserves_low_g_gradient_and_has_peak_sliding_branch_and_trail_decay() {
    let model = MagicFormulaTire::default();
    let force = |angle: f64| {
        let mut state = TireState {
            temperature_k: model.optimum_temperature_k,
            tread_temperature_k: model.optimum_temperature_k,
            ..TireState::default()
        };
        model.evaluate(
            &mut state,
            TireInput {
                normal_load_n: model.nominal_load_n,
                longitudinal_slip: 0.0,
                slip_angle_rad: angle,
                lateral_slip_speed_mps: 30.0 * angle.tan(),
                camber_rad: 0.0,
                speed_mps: 30.0,
                road: RoadCell { rubber: 0.0, ..RoadCell::default() },
                dt: 0.0,
            },
        )
    };
    let small = force(1.0e-5);
    let normalized_gradient = small.lateral_force_n.abs() / (model.peak_mu * model.nominal_load_n * 1.0e-5);
    let (peak_angle, peak_force) = (0..=800)
        .map(|index| index as f64 / 1_000.0)
        .map(|angle| (angle, force(angle).lateral_force_n.abs()))
        .max_by(|left, right| left.1.total_cmp(&right.1))
        .unwrap();
    let slide_ratio = force(0.8).lateral_force_n.abs() / peak_force;
    let low_trail = force(0.03).aligning_moment_nm.abs() / force(0.03).lateral_force_n.abs();
    let peak_trail = force(peak_angle).aligning_moment_nm.abs() / peak_force;
    assert!((normalized_gradient - model.lateral_stiffness).abs() < 0.01);
    assert!((0.12..=0.35).contains(&peak_angle), "peak_angle={peak_angle}");
    assert!((0.75..=0.95).contains(&slide_ratio), "slide_ratio={slide_ratio}");
    assert!(peak_trail < low_trail * 0.35, "peak_trail={peak_trail}, low_trail={low_trail}");
}

#[test]
fn authored_transient_thermal_parameters_are_complete_and_not_claimed_measured() {
    let model = MagicFormulaTire::default();
    let provenance = model.parameter_provenance();
    assert!(provenance.is_complete());
    assert_eq!(provenance.origin, ParameterOrigin::Authored);
    assert!(provenance.source.contains("no tire-rig"));
    for (name, value) in [
        ("lateral_shape_factor", model.lateral_shape_factor),
        ("lateral_curvature_factor", model.lateral_curvature_factor),
        ("pneumatic_trail", model.pneumatic_trail_m),
        ("relaxation_length", model.relaxation_length_m),
        ("tread_heat_capacity", model.tread_heat_capacity_j_k),
        ("bulk_heat_capacity", model.bulk_heat_capacity_j_k),
        ("slip_heat_fraction_to_tread", model.slip_heat_fraction_to_tread),
        ("tread_bulk_conductance", model.tread_bulk_conductance_w_k),
        ("tread_road_conductance", model.tread_road_conductance_w_k),
        ("still_air_conductance", model.still_air_conductance_w_k),
        ("speed_air_conductance", model.speed_air_conductance_w_k_per_mps),
    ] {
        let range = provenance.valid_ranges.iter().find(|range| range.parameter == name).unwrap();
        assert!((range.minimum..=range.maximum).contains(&value), "{name}={value}");
    }
}

#[test]
fn lateral_force_moment_power_and_temperature_are_odd_even_symmetric() {
    let model = MagicFormulaTire::default();
    for angle in [0.01_f64, 0.08, 0.2, 0.45] {
        let evaluate = |signed_angle| {
            let mut state = TireState::default();
            let output = model.evaluate(
                &mut state,
                TireInput {
                    normal_load_n: model.nominal_load_n,
                    longitudinal_slip: 0.0,
                    slip_angle_rad: signed_angle,
                    lateral_slip_speed_mps: 30.0 * signed_angle.tan(),
                    camber_rad: 0.0,
                    speed_mps: 30.0,
                    road: RoadCell::default(),
                    dt: 0.001,
                },
            );
            (output, state)
        };
        let (positive, positive_state) = evaluate(angle);
        let (negative, negative_state) = evaluate(-angle);
        assert_eq!(positive.lateral_force_n, -negative.lateral_force_n);
        assert_eq!(positive.aligning_moment_nm, -negative.aligning_moment_nm);
        assert_eq!(positive.slip_power_w, negative.slip_power_w);
        assert_eq!(positive_state.tread_temperature_k, negative_state.tread_temperature_k);
        assert_eq!(positive_state.temperature_k, negative_state.temperature_k);
    }
}

#[test]
fn released_contact_does_not_create_heat_from_residual_transient_force() {
    let model = MagicFormulaTire::default();
    let mut state = TireState::default();
    let output = model.evaluate(
        &mut state,
        TireInput {
            normal_load_n: model.nominal_load_n,
            longitudinal_slip: 0.0,
            slip_angle_rad: 0.2,
            lateral_slip_speed_mps: 0.0,
            camber_rad: 0.0,
            speed_mps: 30.0,
            road: RoadCell::default(),
            dt: 0.001,
        },
    );
    assert!(output.lateral_force_n.abs() > 1_000.0);
    assert_eq!(output.slip_power_w, 0.0);
}

#[test]
fn severe_slip_heating_is_energy_bounded_and_recovers_after_release() {
    let model = MagicFormulaTire::default();
    for speed_mps in [100.0 / 3.6, 140.0 / 3.6] {
        let mut state = TireState::default();
        let initial_tread = state.tread_temperature_k;
        let mut minimum_mu = f64::MAX;
        let mut stored_energy_j = 0.0;
        let mut energy_balance_j = 0.0;
        for _ in 0..2_000 {
            let old_tread = state.tread_temperature_k;
            let old_bulk = state.temperature_k;
            let output = model.evaluate(
                &mut state,
                TireInput {
                    normal_load_n: model.nominal_load_n,
                    longitudinal_slip: 0.0,
                    slip_angle_rad: 0.45,
                    lateral_slip_speed_mps: speed_mps * 0.45_f64.tan(),
                    camber_rad: 0.0,
                    speed_mps,
                    road: RoadCell::default(),
                    dt: 0.001,
                },
            );
            stored_energy_j += model.tread_heat_capacity_j_k * (state.tread_temperature_k - old_tread)
                + model.bulk_heat_capacity_j_k * (state.temperature_k - old_bulk);
            let air_w = (model.still_air_conductance_w_k + model.speed_air_conductance_w_k_per_mps * speed_mps)
                * (old_bulk - 293.15);
            energy_balance_j += (output.slip_power_w - output.road_heat_w - air_w) * 0.001;
            minimum_mu = minimum_mu.min(output.friction_coefficient);
        }
        let severe_tread = state.tread_temperature_k;
        let severe_bulk = state.temperature_k;
        assert!(severe_tread > initial_tread + 2.0, "speed={speed_mps}, tread={severe_tread}");
        assert!(severe_tread < 370.0, "speed={speed_mps}, tread={severe_tread}");
        assert!(severe_bulk < 335.0, "speed={speed_mps}, bulk={severe_bulk}");
        assert!(minimum_mu > 0.9, "speed={speed_mps}, minimum_mu={minimum_mu}");
        assert!(
            (stored_energy_j - energy_balance_j).abs() / energy_balance_j.abs().max(1.0) < 1.0e-9,
            "speed={speed_mps}, stored={stored_energy_j}, balance={energy_balance_j}"
        );

        for _ in 0..10_000 {
            model.evaluate(
                &mut state,
                TireInput {
                    normal_load_n: model.nominal_load_n,
                    longitudinal_slip: 0.0,
                    slip_angle_rad: 0.0,
                    lateral_slip_speed_mps: 0.0,
                    camber_rad: 0.0,
                    speed_mps,
                    road: RoadCell::default(),
                    dt: 0.001,
                },
            );
        }
        assert!(state.tread_temperature_k < severe_tread, "speed={speed_mps}");
        assert!(state.temperature_k.is_finite() && state.tread_temperature_k.is_finite());
    }
}

#[test]
fn half_pad_vehicle_slip_stays_thermally_bounded_and_relaxes_after_release() {
    // Default balanced-gamepad normalization for a raw half-stick input.
    const HALF_PAD_STEERING: f64 = 0.317_752;
    for speed_mps in [100.0 / 3.6, 140.0 / 3.6] {
        let mut world = flat_vehicle(speed_mps);
        world.set_input_unrecorded(0, DriverInput { steering: HALF_PAD_STEERING, ..DriverInput::default() }).unwrap();
        let mut peak_tread_k: f64 = 0.0;
        let mut peak_bulk_k: f64 = 0.0;
        let mut minimum_mu = f64::MAX;
        for _ in 0..2_000 {
            world.step_fixed(1).unwrap();
            for wheel in &world.vehicles[0].state.wheels {
                peak_tread_k = peak_tread_k.max(wheel.tire.tread_temperature_k);
                peak_bulk_k = peak_bulk_k.max(wheel.tire.temperature_k);
                if wheel.last_normal_load_n > 100.0 {
                    minimum_mu = minimum_mu.min(wheel.last_tire_output.friction_coefficient);
                }
            }
        }
        assert!(peak_tread_k < 390.0, "speed={speed_mps}, tread={peak_tread_k}");
        assert!(peak_bulk_k < 340.0, "speed={speed_mps}, bulk={peak_bulk_k}");
        assert!(minimum_mu > 0.75, "speed={speed_mps}, mu={minimum_mu}");

        reset_motion(&mut world, speed_mps);
        world.set_input_unrecorded(0, DriverInput::default()).unwrap();
        world.step_fixed(2_000).unwrap();
        let residual =
            world.vehicles[0].state.wheels.iter().map(|wheel| wheel.transient_slip_angle_rad.abs()).fold(0.0, f64::max);
        assert!(residual < 0.01, "speed={speed_mps}, residual={residual}");
    }
}

#[derive(Clone, Copy, Debug)]
struct RampResult {
    peak_yaw_rate: f64,
    peak_kinematic_slip: f64,
    peak_transient_slip: f64,
    peak_tread_temperature_k: f64,
}

fn steering_ramp(speed_mps: f64, steer_rad: f64) -> RampResult {
    let mut world = flat_vehicle(speed_mps);
    let normalized = steer_rad / world.vehicles[0].definition.wheels[0].max_steer_rad;
    let mut result = RampResult {
        peak_yaw_rate: 0.0,
        peak_kinematic_slip: 0.0,
        peak_transient_slip: 0.0,
        peak_tread_temperature_k: 0.0,
    };
    for step in 0..2_000 {
        let ramp = (f64::from(step + 1) / 500.0).min(1.0);
        world.set_input_unrecorded(0, DriverInput { steering: normalized * ramp, ..DriverInput::default() }).unwrap();
        world.step_fixed(1).unwrap();
        let vehicle = &world.vehicles[0];
        let yaw = vehicle.telemetry.yaw_rate_rad_s;
        if yaw.abs() > result.peak_yaw_rate.abs() {
            result.peak_yaw_rate = yaw;
        }
        for wheel in &vehicle.state.wheels {
            result.peak_kinematic_slip = result.peak_kinematic_slip.max(wheel.slip_angle_rad.abs());
            result.peak_transient_slip = result.peak_transient_slip.max(wheel.transient_slip_angle_rad.abs());
            result.peak_tread_temperature_k = result.peak_tread_temperature_k.max(wheel.tire.tread_temperature_k);
        }
    }
    result
}

#[test]
fn steering_ramps_at_fifty_one_hundred_and_one_forty_are_finite_and_symmetric() {
    for speed_kmh in [50.0_f64, 100.0, 140.0] {
        for steer_deg in [0.5_f64, 1.0, 2.0] {
            let left = steering_ramp(speed_kmh / 3.6, steer_deg.to_radians());
            let right = steering_ramp(speed_kmh / 3.6, -steer_deg.to_radians());
            for result in [left, right] {
                assert!(result.peak_yaw_rate.is_finite());
                assert!(result.peak_kinematic_slip < 0.8, "speed={speed_kmh}, steer={steer_deg}, {result:?}");
                assert!(result.peak_transient_slip <= result.peak_kinematic_slip + 1.0e-9);
                assert!(result.peak_tread_temperature_k < 370.0, "speed={speed_kmh}, steer={steer_deg}");
            }
            assert!(left.peak_yaw_rate * right.peak_yaw_rate < 0.0);
            assert!((left.peak_yaw_rate.abs() - right.peak_yaw_rate.abs()).abs() < 1.0e-8);
            assert!((left.peak_transient_slip - right.peak_transient_slip).abs() < 1.0e-8);
        }
    }
}

#[test]
fn transient_slip_is_timestep_convergent_and_decays_at_low_speed() {
    let run = |dt: f64, target: f64, speed: f64, initial: f64| {
        let mut state = initial;
        for _ in 0..(1.0 / dt).round() as usize {
            state = transient_slip_step(state, target, speed, 0.45, dt);
        }
        state
    };
    let half = run(0.0005, 0.12, 30.0, 0.0);
    let one = run(0.001, 0.12, 30.0, 0.0);
    let two = run(0.002, 0.12, 30.0, 0.0);
    assert!((half - one).abs() < 1.0e-12 && (one - two).abs() < 1.0e-12);
    for speed in [50.0 / 3.6, 100.0 / 3.6, 140.0 / 3.6] {
        let at_one_length = transient_slip_step(0.0, 1.0, speed, 0.45, 0.45 / speed);
        let at_three_lengths = transient_slip_step(0.0, 1.0, speed, 0.45, 3.0 * 0.45 / speed);
        let at_five_lengths = transient_slip_step(0.0, 1.0, speed, 0.45, 5.0 * 0.45 / speed);
        assert!((at_one_length - (1.0 - (-1.0_f64).exp())).abs() < 1.0e-12);
        assert!((at_three_lengths - (1.0 - (-3.0_f64).exp())).abs() < 1.0e-12);
        let released = transient_slip_step(1.0, 0.0, speed, 0.45, 3.0 * 0.45 / speed);
        assert!((released - (-3.0_f64).exp()).abs() < 1.0e-12);
        assert!((1.0 - at_five_lengths) < 0.01);
        let released_five = transient_slip_step(1.0, 0.0, speed, 0.45, 5.0 * 0.45 / speed);
        assert!(released_five < 0.01);
    }
    let released = run(0.001, 0.0, 0.0, 0.12);
    assert!(released.abs() < 0.04, "released={released}");
}

#[test]
fn relaxed_force_converges_to_quasi_static_force_within_one_percent() {
    let model = MagicFormulaTire::default();
    let speed = 30.0;
    let target = 0.10;
    let relaxed =
        transient_slip_step(0.0, target, speed, model.relaxation_length_m, 5.0 * model.relaxation_length_m / speed);
    let force = |angle| {
        model
            .evaluate(
                &mut TireState::default(),
                TireInput {
                    normal_load_n: model.nominal_load_n,
                    longitudinal_slip: 0.0,
                    slip_angle_rad: angle,
                    lateral_slip_speed_mps: speed * target.tan(),
                    camber_rad: 0.0,
                    speed_mps: speed,
                    road: RoadCell::default(),
                    dt: 0.0,
                },
            )
            .lateral_force_n
    };
    assert!((force(relaxed) - force(target)).abs() / force(target).abs() < 0.01);
}

#[test]
fn temperature_grip_curve_is_finite_positive_and_peaks_near_authored_optimum() {
    let model = MagicFormulaTire::default();
    let mu = |temperature_k| {
        let mut state = TireState { temperature_k, tread_temperature_k: temperature_k, ..TireState::default() };
        model
            .evaluate(
                &mut state,
                TireInput {
                    normal_load_n: model.nominal_load_n,
                    longitudinal_slip: 0.0,
                    slip_angle_rad: 0.1,
                    lateral_slip_speed_mps: 30.0 * 0.1_f64.tan(),
                    camber_rad: 0.0,
                    speed_mps: 30.0,
                    road: RoadCell::default(),
                    dt: 0.0,
                },
            )
            .friction_coefficient
    };
    let optimum = mu(model.optimum_temperature_k);
    for temperature_k in (273..=473).step_by(5) {
        let value = mu(f64::from(temperature_k));
        assert!(value.is_finite() && value > 0.0);
        assert!(value <= optimum * (1.0 + 1.0e-12));
    }
    for offset in [-80.0, 80.0] {
        assert!((0.45..=0.80).contains(&(mu(model.optimum_temperature_k + offset) / optimum)));
    }
}

#[test]
fn severe_slip_thermal_solution_converges_from_half_to_twenty_milliseconds() {
    let model = MagicFormulaTire::default();
    let run = |dt: f64| {
        let mut state = TireState::default();
        for _ in 0..(2.0 / dt).round() as usize {
            model.evaluate(
                &mut state,
                TireInput {
                    normal_load_n: model.nominal_load_n,
                    longitudinal_slip: 0.0,
                    slip_angle_rad: 20.0_f64.to_radians(),
                    lateral_slip_speed_mps: (140.0 / 3.6) * 20.0_f64.to_radians().tan(),
                    camber_rad: 0.0,
                    speed_mps: 140.0 / 3.6,
                    road: RoadCell::default(),
                    dt,
                },
            );
        }
        state
    };
    let reference = run(0.0005);
    let reference_rise = reference.tread_temperature_k - TireState::default().tread_temperature_k;
    for (dt, tolerance) in [(0.001, 0.005), (0.002, 0.005), (0.005, 0.01), (0.010, 0.02), (0.020, 0.04)] {
        let state = run(dt);
        let error = ((state.tread_temperature_k - TireState::default().tread_temperature_k) - reference_rise).abs()
            / reference_rise.abs().max(1.0e-12);
        assert!(error < tolerance, "dt={dt}, error={error}, state={state:?}, reference={reference:?}");
        assert!(state.temperature_k.is_finite() && state.tread_temperature_k.is_finite());
    }
}

#[test]
fn active_transient_slip_round_trips_and_resimulates() {
    let mut original = flat_vehicle(100.0 / 3.6);
    original
        .set_input_unrecorded(0, DriverInput { steering: 1.0_f64.to_radians() / 0.54, ..DriverInput::default() })
        .unwrap();
    original.step_fixed(120).unwrap();
    assert!(original.vehicles[0].state.wheels[0].transient_slip_angle_rad.abs() > 1.0e-5);
    let archived = Snapshot::from_bytes(&original.snapshot().to_bytes()).unwrap();
    let mut restored = PhysicsWorld::new(SimulationConfig::default());
    restored.restore(&archived);
    assert_eq!(restored.snapshot(), original.snapshot());
    original.step_fixed(1_000).unwrap();
    restored.step_fixed(1_000).unwrap();
    assert_eq!(restored.snapshot(), original.snapshot());
}
