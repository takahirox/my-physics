use my_physics::{
    ArcadeDriftPhase, ArcadeDriftSensors, ArcadeKeyboardDriftAssist, DriverInput, KeyboardSteeringAssist, PhysicsWorld,
    Quat, SimulationConfig, Vec3, VehicleDefinition,
};

fn world_at_speed(speed_mps: f64) -> PhysicsWorld {
    let mut definition = VehicleDefinition::arcade_fun();
    definition.transmission.automatic = false;
    let mut world = PhysicsWorld::new(SimulationConfig::default());
    world.add_vehicle(definition);
    world.vehicles[0].driver_aids.traction_control_enabled = false;
    world.vehicles[0].driver_aids.stability_control_enabled = false;
    world.step_fixed(2_000).unwrap();
    let vehicle = &mut world.vehicles[0];
    vehicle.state.position_m = Vec3::new(0.0, 0.55, 0.0);
    vehicle.state.orientation = Quat::IDENTITY;
    vehicle.previous_position_m = vehicle.state.position_m;
    vehicle.previous_orientation = vehicle.state.orientation;
    vehicle.state.linear_velocity_mps = Vec3::new(0.0, 0.0, -speed_mps);
    vehicle.state.angular_velocity_rad_s = Vec3::ZERO;
    vehicle.state.powertrain.gear = 3;
    vehicle.state.powertrain.clutch_engagement = 1.0;
    for (wheel, definition) in vehicle.state.wheels.iter_mut().zip(&vehicle.definition.wheels) {
        wheel.angular_velocity_rad_s = speed_mps / definition.radius_m;
    }
    let ratio = vehicle.definition.transmission.gear_ratios[2] * vehicle.definition.transmission.final_drive;
    vehicle.state.powertrain.engine_rpm =
        vehicle.state.wheels[2].angular_velocity_rad_s * ratio * 60.0 / core::f64::consts::TAU;
    vehicle.update_telemetry(Vec3::ZERO);
    world
}

fn sensors(world: &PhysicsWorld) -> ArcadeDriftSensors {
    let vehicle = &world.vehicles[0];
    let local = vehicle.state.orientation.conjugate().rotate(vehicle.state.linear_velocity_mps);
    ArcadeDriftSensors {
        speed_mps: vehicle.telemetry.speed_mps,
        yaw_rate_rad_s: vehicle.telemetry.yaw_rate_rad_s,
        body_slip_rad: local.x.atan2((-local.z).abs().max(1.0e-9)),
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Metrics {
    peak_beta_deg: f64,
    longest_slide_s: f64,
    longest_countersteer_s: f64,
    yaw_reversals_during_slide: u32,
    recovery_time_s: Option<f64>,
    speed_retained: f64,
    peak_policy_steering: f64,
    entered_slide: bool,
}

fn run(direction: f64, handbrake_duration_s: f64) -> Metrics {
    let mut world = world_at_speed(80.0 / 3.6);
    let starting_speed = world.vehicles[0].telemetry.speed_mps;
    let mut keyboard = KeyboardSteeringAssist::default();
    let mut drift = ArcadeKeyboardDriftAssist::default();
    let mut metrics = Metrics::default();
    let mut stable_steps = 0_u32;
    let mut current_slide_s = 0.0;
    let mut current_countersteer_s = 0.0;
    let mut previous_slide_yaw_sign = 0.0_f64;
    for step in 0..4_000 {
        let time = step as f64 * 0.001;
        let input = if time < 0.35 {
            DriverInput { steering: direction, throttle: 0.35, ..DriverInput::default() }
        } else if time < (0.35 + handbrake_duration_s).min(0.55) {
            DriverInput { steering: direction, throttle: 0.35, handbrake: 1.0, ..DriverInput::default() }
        } else if time < 0.35 + handbrake_duration_s {
            DriverInput { steering: direction, throttle: 0.70, handbrake: 1.0, ..DriverInput::default() }
        } else if time < 1.65 {
            DriverInput { steering: direction, throttle: 0.70, ..DriverInput::default() }
        } else {
            DriverInput { steering: 0.0, throttle: 0.70, ..DriverInput::default() }
        };
        let sensed = sensors(&world);
        let base = keyboard.update_for_target(input.steering, sensed.speed_mps, 0.001, 12.0);
        let steering = drift.update(base, input, sensed, 0.001);
        let policy = DriverInput { steering, ..input };
        world.set_input_unrecorded(0, policy).unwrap();
        world.step_fixed(1).unwrap();

        let sensed = sensors(&world);
        let beta_deg = sensed.body_slip_rad.abs().to_degrees();
        metrics.peak_beta_deg = metrics.peak_beta_deg.max(beta_deg);
        metrics.peak_policy_steering = metrics.peak_policy_steering.max(steering.abs());
        if matches!(drift.phase(), ArcadeDriftPhase::Slide | ArcadeDriftPhase::Recovery) {
            metrics.entered_slide = true;
        }
        if (8.0..=40.0).contains(&beta_deg) && sensed.yaw_rate_rad_s.abs() >= 0.22 {
            current_slide_s += 0.001;
            metrics.longest_slide_s = metrics.longest_slide_s.max(current_slide_s);
            let yaw_sign = sensed.yaw_rate_rad_s.signum();
            if previous_slide_yaw_sign != 0.0 && yaw_sign != previous_slide_yaw_sign {
                metrics.yaw_reversals_during_slide += 1;
            }
            previous_slide_yaw_sign = yaw_sign;
            if input.steering * steering < 0.0 {
                current_countersteer_s += 0.001;
                metrics.longest_countersteer_s = metrics.longest_countersteer_s.max(current_countersteer_s);
            } else {
                current_countersteer_s = 0.0;
            }
        } else {
            current_slide_s = 0.0;
            current_countersteer_s = 0.0;
            previous_slide_yaw_sign = 0.0;
        }
        if time >= 1.65 && beta_deg < 5.0 && sensed.yaw_rate_rad_s.abs() < 0.25 && steering.abs() < 0.08 {
            stable_steps += 1;
            if stable_steps >= 200 && metrics.recovery_time_s.is_none() {
                metrics.recovery_time_s = Some(time + 0.001 - 1.65 - 0.199);
            }
        } else {
            stable_steps = 0;
        }
    }
    metrics.speed_retained = world.vehicles[0].telemetry.speed_mps / starting_speed;
    metrics
}

#[test]
fn drift_entry_requires_a_physical_driver_initiation() {
    let steering_only = run(-1.0, 0.0);
    let handbrake = run(-1.0, 0.80);
    eprintln!("steering-only={steering_only:?} handbrake={handbrake:?}");
    assert!(!steering_only.entered_slide, "steering alone armed the drift controller: {steering_only:?}");
    assert!(handbrake.entered_slide, "physical handbrake entry did not produce a slide: {handbrake:?}");
    assert!(handbrake.peak_beta_deg >= 10.0, "handbrake={handbrake:?}");
}

#[test]
fn keyboard_intent_can_sustain_countersteer_and_recover_symmetrically() {
    let left = run(-1.0, 0.80);
    let right = run(1.0, 0.80);
    eprintln!("left={left:?} right={right:?}");
    for metrics in [left, right] {
        assert!((10.0..=48.0).contains(&metrics.peak_beta_deg), "{metrics:?}");
        assert!(metrics.longest_slide_s >= 0.45, "{metrics:?}");
        assert!(metrics.longest_countersteer_s >= 0.20, "{metrics:?}");
        assert!(metrics.yaw_reversals_during_slide <= 1, "{metrics:?}");
        assert!(metrics.recovery_time_s.is_some_and(|seconds| seconds <= 1.8), "{metrics:?}");
        assert!(metrics.speed_retained >= 0.45, "{metrics:?}");
        assert!(metrics.peak_policy_steering <= 0.72 + 1.0e-12, "{metrics:?}");
    }
    assert!((left.peak_beta_deg - right.peak_beta_deg).abs() <= 1.0);
    assert!((left.longest_slide_s - right.longest_slide_s).abs() <= 0.05);
    assert!((left.recovery_time_s.unwrap() - right.recovery_time_s.unwrap()).abs() <= 0.05);
}

#[test]
fn poor_entry_timing_is_not_automatically_rescued() {
    let controlled = run(-1.0, 0.80);
    let excessive = run(-1.0, 1.30);
    eprintln!("controlled={controlled:?} excessive={excessive:?}");
    assert!(controlled.recovery_time_s.is_some());
    assert!(
        excessive.peak_beta_deg >= 55.0 && (excessive.recovery_time_s.is_none() || excessive.speed_retained < 0.35),
        "an excessive handbrake entry was silently made equivalent to the controlled input: {excessive:?}"
    );
}
