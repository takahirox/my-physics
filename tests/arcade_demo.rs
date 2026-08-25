use my_physics::{DriverInput, PhysicsWorld, Quat, SimulationConfig, Snapshot, Vec3, VehicleDefinition, VehiclePreset};

fn flat_world(preset: VehiclePreset) -> PhysicsWorld {
    let mut definition = VehicleDefinition::from_preset(preset);
    definition.transmission.automatic = false;
    let mut world = PhysicsWorld::new(SimulationConfig::default());
    world.add_vehicle(definition);
    world.vehicles[0].driver_aids.abs_enabled = true;
    world.vehicles[0].driver_aids.traction_control_enabled = false;
    world.vehicles[0].driver_aids.stability_control_enabled = false;
    world.step_fixed(2_000).unwrap();
    world
}

fn set_speed(world: &mut PhysicsWorld, speed_mps: f64, driven_gear: bool) {
    let vehicle = &mut world.vehicles[0];
    vehicle.state.position_m = Vec3::new(0.0, 0.55, 0.0);
    vehicle.state.orientation = Quat::IDENTITY;
    vehicle.previous_position_m = vehicle.state.position_m;
    vehicle.previous_orientation = vehicle.state.orientation;
    vehicle.state.linear_velocity_mps = Vec3::new(0.0, 0.0, -speed_mps);
    vehicle.state.angular_velocity_rad_s = Vec3::ZERO;
    vehicle.state.powertrain.gear = if driven_gear { 3 } else { 0 };
    vehicle.state.powertrain.clutch_engagement = if driven_gear { 1.0 } else { 0.0 };
    for (wheel, definition) in vehicle.state.wheels.iter_mut().zip(&vehicle.definition.wheels) {
        wheel.angular_velocity_rad_s = speed_mps / definition.radius_m;
    }
    if driven_gear {
        let ratio = vehicle.definition.transmission.gear_ratios[2] * vehicle.definition.transmission.final_drive;
        vehicle.state.powertrain.engine_rpm =
            vehicle.state.wheels[2].angular_velocity_rad_s * ratio * 60.0 / core::f64::consts::TAU;
    }
    vehicle.update_telemetry(Vec3::ZERO);
}

fn yaw(q: Quat) -> f64 {
    (2.0 * (q.w * q.y + q.x * q.z)).atan2(1.0 - 2.0 * (q.y * q.y + q.x * q.x))
}

fn sideslip(world: &PhysicsWorld) -> f64 {
    let vehicle = &world.vehicles[0];
    let forward = vehicle.state.orientation.rotate(Vec3::FORWARD);
    let right = vehicle.state.orientation.rotate(Vec3::X);
    vehicle.state.linear_velocity_mps.dot(right).atan2(vehicle.state.linear_velocity_mps.dot(forward).abs().max(0.1))
}

#[test]
fn arcade_acceleration_braking_and_speed_sweep_have_declared_bands() {
    let mut acceleration = PhysicsWorld::new(SimulationConfig::default());
    acceleration.add_vehicle(VehicleDefinition::arcade_fun());
    acceleration.vehicles[0].driver_aids.stability_control_enabled = false;
    acceleration.step_fixed(2_000).unwrap();
    set_speed(&mut acceleration, 0.0, true);
    acceleration.vehicles[0].state.powertrain.gear = 1;
    acceleration.vehicles[0].state.powertrain.engine_rpm = acceleration.vehicles[0].definition.engine.idle_rpm;
    acceleration.vehicles[0].state.powertrain.clutch_engagement = 0.0;
    let mut zero_to_100_s = None;
    for step in 0..8_000 {
        acceleration.set_input_unrecorded(0, DriverInput { throttle: 1.0, ..DriverInput::default() }).unwrap();
        acceleration.step_fixed(1).unwrap();
        if acceleration.vehicles[0].telemetry.speed_mps >= 100.0 / 3.6 {
            zero_to_100_s = Some((step + 1) as f64 * 0.001);
            break;
        }
    }
    let zero_to_100_s = zero_to_100_s.expect("Arcade vehicle did not reach 100 km/h");
    assert!((2.8..=4.2).contains(&zero_to_100_s), "0-100={zero_to_100_s}");

    let mut braking = flat_world(VehiclePreset::ArcadeFun);
    set_speed(&mut braking, 100.0 / 3.6, false);
    let start = braking.vehicles[0].state.position_m;
    let mut braking_time_s = 0.0;
    for step in 0..6_000 {
        braking.set_input_unrecorded(0, DriverInput { brake: 1.0, ..DriverInput::default() }).unwrap();
        braking.step_fixed(1).unwrap();
        if braking.vehicles[0].telemetry.speed_mps <= 2.0 {
            braking_time_s = (step + 1) as f64 * 0.001;
            break;
        }
    }
    let braking_distance_m = (braking.vehicles[0].state.position_m - start).length();
    eprintln!("arcade longitudinal: 0-100={zero_to_100_s:.3}s, 100-2m/s={braking_distance_m:.3}m/{braking_time_s:.3}s");
    assert!((26.0..=38.0).contains(&braking_distance_m), "100-0 distance={braking_distance_m}");
    assert!((1.75..=3.0).contains(&braking_time_s), "100-0 time={braking_time_s}");

    for (speed_kmh, expected_heading_deg) in [(50.0, 4.0..=7.0), (100.0, 7.0..=11.0), (140.0, 9.0..=14.0)] {
        let mut world = flat_world(VehiclePreset::ArcadeFun);
        set_speed(&mut world, speed_kmh / 3.6, false);
        for step in 0..1_000 {
            let road_wheel_rad = 1.0_f64.to_radians() * ((step + 1) as f64 / 250.0).min(1.0);
            let steering = road_wheel_rad / world.vehicles[0].definition.wheels[0].max_steer_rad;
            world.set_input_unrecorded(0, DriverInput { steering, ..DriverInput::default() }).unwrap();
            world.step_fixed(1).unwrap();
        }
        let heading_deg = yaw(world.vehicles[0].state.orientation).abs().to_degrees();
        eprintln!("arcade 1deg ramp/hold: {speed_kmh:.0}km/h heading={heading_deg:.3}deg");
        assert!(expected_heading_deg.contains(&heading_deg), "speed={speed_kmh}, heading={heading_deg}");
    }
}

#[derive(Clone, Copy, Debug)]
struct DriftMetrics {
    peak_sideslip_deg: f64,
    recovery_s: Option<f64>,
    speed_retained: f64,
    yaw_reversals: u32,
}

fn drift_fixture(handbrake: bool) -> DriftMetrics {
    let mut world = flat_world(VehiclePreset::ArcadeFun);
    set_speed(&mut world, 80.0 / 3.6, true);
    let start_speed = world.vehicles[0].telemetry.speed_mps;
    let mut peak_sideslip_deg: f64 = 0.0;
    let mut stable_steps = 0_u32;
    let mut recovery_s = None;
    let mut prior_yaw_sign = 0.0_f64;
    let mut yaw_reversals = 0;
    for step in 0..3_500 {
        let time = step as f64 * 0.001;
        let (steer_deg, throttle, brake): (f64, f64, f64) = if handbrake {
            if time < 0.35 {
                (4.0, 0.35, 0.0)
            } else if time < 0.55 {
                (4.0, 0.35, 1.0)
            } else if time < 0.85 {
                (4.0, 0.70, 1.0)
            } else if time < 1.25 {
                (4.0, 0.70, 0.0)
            } else if time < 2.40 {
                // A deliberate fixed countersteer phase represents the driver
                // catching the slide. Holding opposite lock after the body is
                // aligned caused a second pendulum turn, so unwind the rack at
                // a fixed time as a human driver would.
                (-4.0, 0.70, 0.0)
            } else {
                (0.0, 0.70, 0.0)
            }
        } else if time < 0.55 {
            (4.0, 0.55, 0.0)
        } else if time < 1.10 {
            (4.0, 0.0, 0.0)
        } else if time < 1.25 {
            (4.0, 0.65, 0.0)
        } else {
            (-3.2, 0.65, 0.0)
        };
        let steering = steer_deg.to_radians() / world.vehicles[0].definition.wheels[0].max_steer_rad;
        world
            .set_input_unrecorded(0, DriverInput { steering, throttle, handbrake: brake, ..DriverInput::default() })
            .unwrap();
        world.step_fixed(1).unwrap();
        let beta_deg = sideslip(&world).abs().to_degrees();
        peak_sideslip_deg = peak_sideslip_deg.max(beta_deg);
        let yaw_rate = world.vehicles[0].telemetry.yaw_rate_rad_s;
        if yaw_rate.abs() > 0.25 {
            let sign = yaw_rate.signum();
            if prior_yaw_sign != 0.0 && sign != prior_yaw_sign {
                yaw_reversals += 1;
            }
            prior_yaw_sign = sign;
        }
        if time >= 1.25 && beta_deg < 5.0 && yaw_rate.abs() < 0.25 {
            stable_steps += 1;
            if stable_steps >= 200 && recovery_s.is_none() {
                recovery_s = Some(time + 0.001 - 1.25 - 0.199);
            }
        } else {
            stable_steps = 0;
        }
    }
    DriftMetrics {
        peak_sideslip_deg,
        recovery_s,
        speed_retained: world.vehicles[0].telemetry.speed_mps / start_speed,
        yaw_reversals,
    }
}

#[test]
fn lift_and_handbrake_create_readable_drift_then_recover() {
    let lift = drift_fixture(false);
    let handbrake = drift_fixture(true);
    eprintln!("arcade drift: lift={lift:?}, handbrake={handbrake:?}");
    assert!((4.0..=10.0).contains(&lift.peak_sideslip_deg), "lift={lift:?}");
    assert!((18.0..=38.0).contains(&handbrake.peak_sideslip_deg), "handbrake={handbrake:?}");
    let recovery = handbrake.recovery_s.expect("handbrake drift never recovered");
    // Keep 200 ms of margin inside the original 1.8 s product target so small
    // libm differences cannot leave another architecture on the boundary.
    assert!((1.0..=1.6).contains(&recovery), "handbrake={handbrake:?}");
    assert!(handbrake.speed_retained >= 0.58, "handbrake={handbrake:?}");
    assert_eq!(handbrake.yaw_reversals, 0, "countersteer must recover without a pendulum reversal");
    assert!(lift.speed_retained >= 0.55, "lift={lift:?}");
    assert!(lift.yaw_reversals <= 1, "lift={lift:?}");
    assert!(lift.peak_sideslip_deg < 75.0 && handbrake.peak_sideslip_deg < 75.0);
}

#[test]
fn arcade_slalom_and_snapshot_replay_are_deterministic() {
    fn run(mut world: PhysicsWorld, start_step: u32, steps: u32) -> (PhysicsWorld, f64, f64, u32) {
        let mut peak_yaw: f64 = 0.0;
        let mut peak_sideslip: f64 = 0.0;
        let mut reversals = 0;
        let mut sign = 0.0_f64;
        for step in start_step..start_step + steps {
            let road_wheel = 2.0_f64.to_radians() * (core::f64::consts::TAU * 0.5 * step as f64 * 0.001).sin();
            let steering = road_wheel / world.vehicles[0].definition.wheels[0].max_steer_rad;
            world.set_input_unrecorded(0, DriverInput { steering, ..DriverInput::default() }).unwrap();
            world.step_fixed(1).unwrap();
            peak_yaw = peak_yaw.max(world.vehicles[0].telemetry.yaw_rate_rad_s.abs());
            peak_sideslip = peak_sideslip.max(sideslip(&world).abs());
            let yaw = world.vehicles[0].telemetry.yaw_rate_rad_s;
            if yaw.abs() > 0.05 {
                if sign != 0.0 && sign != yaw.signum() {
                    reversals += 1;
                }
                sign = yaw.signum();
            }
        }
        (world, peak_yaw, peak_sideslip, reversals)
    }
    let mut initial = flat_world(VehiclePreset::ArcadeFun);
    set_speed(&mut initial, 100.0 / 3.6, false);
    let start_speed = initial.vehicles[0].telemetry.speed_mps;
    let (first_half, first_yaw, first_sideslip, first_reversals) = run(initial, 0, 4_000);
    let archived = Snapshot::from_bytes(&first_half.snapshot().to_bytes()).unwrap();
    let (original, second_yaw, second_sideslip, second_reversals) = run(first_half, 4_000, 4_000);
    let mut restored = PhysicsWorld::new(SimulationConfig::default());
    restored.restore(&archived);
    let (replayed, replay_yaw, replay_sideslip, replay_reversals) = run(restored, 4_000, 4_000);
    assert_eq!(original.snapshot(), replayed.snapshot());
    assert_eq!((second_yaw, second_sideslip, second_reversals), (replay_yaw, replay_sideslip, replay_reversals));
    let peak_yaw = first_yaw.max(second_yaw);
    let peak_sideslip = first_sideslip.max(second_sideslip);
    eprintln!(
        "arcade slalom: yaw={peak_yaw:.3}rad/s beta={:.3}deg speed-retain={:.3} reversals={}",
        peak_sideslip.to_degrees(),
        original.vehicles[0].telemetry.speed_mps / start_speed,
        first_reversals + second_reversals
    );
    assert!((0.35..=0.65).contains(&peak_yaw), "peak yaw={peak_yaw}");
    assert!(
        (2.8_f64.to_radians()..=10.0_f64.to_radians()).contains(&peak_sideslip),
        "peak beta={}",
        peak_sideslip.to_degrees()
    );
    assert!(original.vehicles[0].telemetry.speed_mps / start_speed >= 0.80);
    assert!((6..=8).contains(&(first_reversals + second_reversals)));
}

#[test]
fn arcade_ai_completes_a_contact_free_lap_with_bounded_tire_temperature() {
    use my_physics::DEMO_TRACK_HALF_WIDTH_M;
    use my_physics::circuit::{ai_driver_input_with_yaw, nearest_segment, segments};

    let mut world = PhysicsWorld::demo_with_preset(1, VehiclePreset::ArcadeFun);
    let mut previous = nearest_segment(world.vehicles[0].state.position_m);
    let mut laps = 0;
    let mut maximum_lateral_error_m: f64 = 0.0;
    let mut maximum_tire_temperature_k: f64 = 0.0;
    for _ in 0..110_000 {
        let vehicle = &world.vehicles[0];
        let input = ai_driver_input_with_yaw(
            vehicle.state.position_m,
            vehicle.state.orientation,
            vehicle.telemetry.speed_mps,
            vehicle.telemetry.yaw_rate_rad_s,
            0.0,
        );
        world.set_input_unrecorded(0, input).unwrap();
        world.step_fixed(1).unwrap();
        let vehicle = &world.vehicles[0];
        let segment_index = nearest_segment(vehicle.state.position_m);
        let segment = segments()[segment_index];
        maximum_lateral_error_m =
            maximum_lateral_error_m.max(((vehicle.state.position_m - segment.center_m).dot(segment.right)).abs());
        maximum_tire_temperature_k =
            maximum_tire_temperature_k.max(vehicle.telemetry.tire_temperature_k.into_iter().fold(0.0, f64::max));
        if previous > segments().len() * 4 / 5 && segment_index < segments().len() / 5 {
            laps += 1;
        }
        previous = segment_index;
    }
    eprintln!(
        "arcade AI: laps={laps} lateral-error={maximum_lateral_error_m:.3}m max-tire={:.1}C",
        maximum_tire_temperature_k - 273.15
    );
    assert!(laps >= 1);
    assert!(maximum_lateral_error_m < DEMO_TRACK_HALF_WIDTH_M - 1.2);
    assert_eq!(world.vehicles[0].state.damage.body, 0.0);
    assert!(maximum_tire_temperature_k < 390.0);
}
