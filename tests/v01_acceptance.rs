use my_physics::collision::{CollisionShape, StaticCollider};
use my_physics::feedback::FeedbackEventKind;
use my_physics::math::semi_implicit_linear_step;
use my_physics::road::{DynamicRoad, RoadCell};
use my_physics::tire::TireState;
use my_physics::{DriverInput, Fidelity, MagicFormulaTire, PhysicsWorld, Quat, TireInput, TireModel, Vec3};

fn collision_world(shape: CollisionShape, position: Vec3, orientation: Quat) -> PhysicsWorld {
    let mut world = PhysicsWorld::demo(1);
    world.static_colliders.clear();
    world.static_colliders.push(StaticCollider {
        position_m: position,
        orientation,
        shape,
        restitution: 0.1,
        friction: 0.8,
    });
    world.vehicles[0].state.position_m = Vec3::new(0.0, 0.55, 0.0);
    world.vehicles[0].state.orientation = Quat::IDENTITY;
    world.vehicles[0].state.linear_velocity_mps.z = -10.0;
    world
}

#[test]
fn semi_implicit_integrator_matches_discrete_constant_acceleration_solution() {
    let mut position = Vec3::ZERO;
    let mut velocity = Vec3::ZERO;
    let acceleration = Vec3::new(3.0, -9.0, 1.5);
    let dt = 0.001;
    let steps = 10_000;
    for _ in 0..steps {
        semi_implicit_linear_step(&mut position, &mut velocity, acceleration, dt);
    }
    let n = f64::from(steps);
    let expected_position = acceleration * (dt * dt * n * (n + 1.0) * 0.5);
    let expected_velocity = acceleration * (dt * n);
    assert!((position - expected_position).length() < 1.0e-9);
    assert!((velocity - expected_velocity).length() < 1.0e-9);
}

#[test]
fn combined_tire_force_stays_inside_friction_circle() {
    let model = MagicFormulaTire::default();
    for slip in [-1.0, -0.3, 0.0, 0.3, 1.0] {
        for angle in [-0.8, -0.2, 0.0, 0.2, 0.8] {
            let output = model.evaluate(
                &mut TireState::default(),
                TireInput {
                    normal_load_n: 4_000.0,
                    longitudinal_slip: slip,
                    slip_angle_rad: angle,
                    camber_rad: 0.12,
                    speed_mps: 30.0,
                    road: RoadCell::default(),
                    dt: 0.001,
                },
            );
            let resultant = output.longitudinal_force_n.hypot(output.lateral_force_n);
            assert!(resultant <= output.friction_coefficient * 4_000.0 * (1.0 + 1.0e-12));
        }
    }
}

#[test]
fn pressure_changes_contact_patch_and_rolling_resistance() {
    let model = MagicFormulaTire::default();
    let input = TireInput {
        normal_load_n: 3_700.0,
        longitudinal_slip: 0.05,
        slip_angle_rad: 0.02,
        camber_rad: 0.0,
        speed_mps: 20.0,
        road: RoadCell::default(),
        dt: 0.001,
    };
    let mut normal = TireState::default();
    let normal_output = model.evaluate(&mut normal, input);
    let mut low = TireState { pressure_pa: 80_000.0, ..TireState::default() };
    let low_output = model.evaluate(&mut low, input);
    assert!(low.contact_patch_m2 > normal.contact_patch_m2);
    assert!(low_output.rolling_resistance_n > normal_output.rolling_resistance_n);
}

#[test]
fn road_interaction_deposits_rubber_heats_surface_and_removes_water() {
    let mut road = DynamicRoad::new(4, 4, 2.0);
    road.set_uniform_water(0.004);
    let point = Vec3::ZERO;
    let before = road.sample(point);
    road.interact(point, 20_000.0, 370.0, 0.1);
    let after = road.sample(point);
    assert!(after.rubber > before.rubber);
    assert!(after.temperature_k > before.temperature_k);
    assert!(after.water_depth_m < before.water_depth_m);
}

#[test]
fn rotated_box_narrow_phase_applies_collision_damage() {
    let mut world = collision_world(
        CollisionShape::Box { half_extents_m: Vec3::new(2.0, 1.0, 0.4) },
        Vec3::new(0.0, 0.55, -2.3),
        Quat::from_axis_angle(Vec3::Y, 0.28),
    );
    world.step_fixed(1).unwrap();
    assert!(world.vehicles[0].state.damage.body > 0.0);
}

#[test]
fn capsule_narrow_phase_applies_collision_damage() {
    let mut world = collision_world(
        CollisionShape::Capsule { radius_m: 0.5, half_height_m: 0.7 },
        Vec3::new(0.0, 0.55, -2.5),
        Quat::IDENTITY,
    );
    world.step_fixed(1).unwrap();
    assert!(world.vehicles[0].state.damage.body > 0.0);
}

#[test]
fn convex_narrow_phase_applies_collision_damage() {
    let points = vec![
        Vec3::new(-2.0, -1.0, -0.3),
        Vec3::new(2.0, -1.0, -0.3),
        Vec3::new(2.0, 1.0, 0.3),
        Vec3::new(-2.0, 1.0, 0.3),
    ];
    let mut world =
        collision_world(CollisionShape::Convex { points_local_m: points }, Vec3::new(0.0, 0.55, -2.0), Quat::IDENTITY);
    world.step_fixed(1).unwrap();
    assert!(world.vehicles[0].state.damage.body > 0.0);
}

#[test]
fn deformation_changes_collision_shape_and_inertia() {
    let mut world = PhysicsWorld::demo(1);
    let initial_shape = world.vehicles[0].collision_half_extents_m();
    let initial_inertia = world.vehicles[0].inertia_kg_m2();
    world.vehicles[0].state.damage.body = 0.8;
    world.vehicles[0].state.damage.deformation_local_m = Vec3::new(0.04, 0.0, 0.1);
    assert!(world.vehicles[0].collision_half_extents_m().z < initial_shape.z);
    assert_ne!(world.vehicles[0].inertia_kg_m2(), initial_inertia);
}

#[test]
fn clutch_and_gearbox_wear_become_effective_failures() {
    let mut clutch_world = PhysicsWorld::demo(1);
    clutch_world.vehicles[0].state.powertrain.clutch_wear = 1.0;
    clutch_world.step_fixed(1).unwrap();
    assert!(clutch_world.vehicles[0].state.powertrain.clutch_failed);
    assert!(clutch_world.vehicles[0].events.iter().any(|event| event.kind == FeedbackEventKind::ClutchFailure));

    let mut gearbox_world = PhysicsWorld::demo(1);
    gearbox_world.vehicles[0].state.powertrain.gearbox_wear = 1.0;
    gearbox_world.step_fixed(2).unwrap();
    assert!(gearbox_world.vehicles[0].state.powertrain.gearbox_failed);
    assert_eq!(gearbox_world.vehicles[0].state.powertrain.gear, 0);
}

#[test]
fn feedback_interfaces_expose_continuous_and_discrete_physics() {
    let mut world = PhysicsWorld::demo(1);
    world.set_input(0, DriverInput { throttle: 0.8, gear_request: 2, ..DriverInput::default() }).unwrap();
    world.step_fixed(1).unwrap();
    let vehicle = &world.vehicles[0];
    assert!(vehicle.audio.engine_load > 0.0);
    assert!(vehicle.force_feedback.steering_torque_nm.is_finite());
    assert!(vehicle.events.iter().any(|event| event.kind == FeedbackEventKind::GearShift));
}

#[test]
fn quaternion_stays_normalized_during_long_steering_run() {
    let mut world = PhysicsWorld::demo(1);
    world.set_input(0, DriverInput { throttle: 0.6, steering: 0.35, ..DriverInput::default() }).unwrap();
    world.step_fixed(20_000).unwrap();
    let q = world.vehicles[0].state.orientation;
    let norm = (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
    assert!((norm - 1.0).abs() < 1.0e-12);
}

#[test]
fn lod_transition_is_smooth_and_converges() {
    let mut world = PhysicsWorld::demo(2);
    world.vehicles[1].state.position_m = world.vehicles[0].state.position_m + Vec3::new(100.0, 0.0, 0.0);
    let initial = world.vehicles[1].fidelity;
    world.step_fixed(1).unwrap();
    assert!((world.vehicles[1].fidelity - initial).abs() < 0.01);
    world.step_fixed(3_000).unwrap();
    assert!((world.vehicles[1].fidelity - 0.25).abs() < 0.01);
}

#[test]
fn bounded_variable_timestep_remains_finite() {
    let mut world = PhysicsWorld::demo(1);
    world.set_input(0, DriverInput { throttle: 0.7, steering: -0.15, ..DriverInput::default() }).unwrap();
    for _ in 0..500 {
        world.step_variable(0.005).unwrap();
    }
    let state = &world.vehicles[0].state;
    assert!(state.position_m.finite());
    assert!(state.linear_velocity_mps.finite());
}

#[test]
fn vehicle_collision_conserves_planar_momentum_within_external_force_tolerance() {
    let mut world = PhysicsWorld::demo(2);
    world.static_colliders.clear();
    world.vehicles[0].state.position_m = Vec3::new(0.0, 0.55, 0.0);
    world.vehicles[1].state.position_m = Vec3::new(0.0, 0.55, -3.5);
    world.vehicles[0].state.linear_velocity_mps.z = -10.0;
    world.vehicles[1].state.linear_velocity_mps.z = 10.0;
    let before =
        world.vehicles.iter().map(|vehicle| vehicle.mass_kg() * vehicle.state.linear_velocity_mps.z).sum::<f64>();
    world.step_fixed(1).unwrap();
    let after =
        world.vehicles.iter().map(|vehicle| vehicle.mass_kg() * vehicle.state.linear_velocity_mps.z).sum::<f64>();
    assert!((after - before).abs() < 30.0, "before={before}, after={after}");
}

fn world_angular_momentum(vehicle: &my_physics::vehicle::Vehicle) -> Vec3 {
    let orientation = vehicle.state.orientation;
    let omega_body = orientation.conjugate().rotate(vehicle.state.angular_velocity_rad_s);
    let inertia = vehicle.inertia_kg_m2();
    let spin =
        orientation.rotate(Vec3::new(inertia.x * omega_body.x, inertia.y * omega_body.y, inertia.z * omega_body.z));
    spin + vehicle.state.position_m.cross(vehicle.state.linear_velocity_mps * vehicle.mass_kg())
}

fn rotational_energy(vehicle: &my_physics::vehicle::Vehicle) -> f64 {
    let orientation = vehicle.state.orientation;
    let omega = vehicle.state.angular_velocity_rad_s;
    let omega_body = orientation.conjugate().rotate(omega);
    let inertia = vehicle.inertia_kg_m2();
    0.5 * (inertia.x * omega_body.x * omega_body.x
        + inertia.y * omega_body.y * omega_body.y
        + inertia.z * omega_body.z * omega_body.z)
}

fn contact_velocity(vehicle: &my_physics::vehicle::Vehicle, point_m: Vec3) -> Vec3 {
    vehicle.state.linear_velocity_mps + vehicle.state.angular_velocity_rad_s.cross(point_m - vehicle.state.position_m)
}

#[test]
fn torque_free_asymmetric_rigid_body_conserves_world_angular_momentum() {
    let mut world = PhysicsWorld::demo(1);
    world.static_colliders.clear();
    world.config.gravity_mps2 = 0.0;
    let vehicle = &mut world.vehicles[0];
    vehicle.definition.chassis.drag_coefficient = 0.0;
    vehicle.definition.chassis.lift_coefficient = 0.0;
    vehicle.state.powertrain.fuel_kg = 0.0;
    vehicle.state.position_m = Vec3::new(0.0, 100.0, 0.0);
    vehicle.state.linear_velocity_mps = Vec3::ZERO;
    vehicle.state.orientation = Quat::from_axis_angle(Vec3::new(0.3, 0.8, -0.2), 0.7);
    vehicle.state.angular_velocity_rad_s = Vec3::new(1.0, 2.0, 0.4);

    let momentum_before = world_angular_momentum(&world.vehicles[0]);
    let energy_before = rotational_energy(&world.vehicles[0]);
    world.step_fixed(10_000).unwrap();
    let momentum_after = world_angular_momentum(&world.vehicles[0]);
    let energy_after = rotational_energy(&world.vehicles[0]);

    let momentum_relative_error = (momentum_after - momentum_before).length() / momentum_before.length().max(1.0e-12);
    let energy_relative_error = (energy_after - energy_before).abs() / energy_before;
    assert!(momentum_relative_error < 1.0e-10, "angular momentum error={momentum_relative_error:e}");
    assert!(energy_relative_error < 2.0e-5, "rotational energy error={energy_relative_error:e}");
}

#[test]
fn principal_axis_free_rotation_has_no_artificial_decay() {
    let mut world = PhysicsWorld::demo(1);
    world.static_colliders.clear();
    world.config.gravity_mps2 = 0.0;
    let vehicle = &mut world.vehicles[0];
    vehicle.definition.chassis.drag_coefficient = 0.0;
    vehicle.definition.chassis.lift_coefficient = 0.0;
    vehicle.state.powertrain.fuel_kg = 0.0;
    vehicle.state.position_m = Vec3::new(0.0, 100.0, 0.0);
    vehicle.state.orientation = Quat::from_axis_angle(Vec3::new(-0.4, 0.2, 0.7), 1.1);
    let initial_axis_world = vehicle.state.orientation.rotate(Vec3::Y);
    vehicle.state.angular_velocity_rad_s = initial_axis_world * 2.5;
    let momentum_before = world_angular_momentum(vehicle);
    let energy_before = rotational_energy(vehicle);

    world.step_fixed(10_000).unwrap();

    let vehicle = &world.vehicles[0];
    let momentum_error = (world_angular_momentum(vehicle) - momentum_before).length() / momentum_before.length();
    let energy_error = (rotational_energy(vehicle) - energy_before).abs() / energy_before;
    let omega_error = (vehicle.state.angular_velocity_rad_s - initial_axis_world * 2.5).length();
    assert!(momentum_error < 1.0e-10, "angular momentum error={momentum_error:e}");
    assert!(energy_error < 1.0e-10, "energy error={energy_error:e}");
    assert!(omega_error < 1.0e-8, "angular velocity error={omega_error:e}");
}

#[test]
fn torque_free_rotation_is_covariant_under_world_frame_rotation() {
    let mut first = PhysicsWorld::demo(1);
    first.static_colliders.clear();
    first.config.gravity_mps2 = 0.0;
    first.vehicles[0].definition.chassis.drag_coefficient = 0.0;
    first.vehicles[0].definition.chassis.lift_coefficient = 0.0;
    first.vehicles[0].state.powertrain.fuel_kg = 0.0;
    first.vehicles[0].state.position_m = Vec3::new(0.0, 100.0, 0.0);
    first.vehicles[0].state.orientation = Quat::from_axis_angle(Vec3::new(0.2, 0.9, -0.3), 0.8);
    first.vehicles[0].state.angular_velocity_rad_s = Vec3::new(1.1, -0.7, 2.0);

    let world_rotation = Quat::from_axis_angle(Vec3::new(-0.5, 0.4, 0.6), 1.2);
    let mut rotated = first.clone();
    rotated.vehicles[0].state.orientation = world_rotation * first.vehicles[0].state.orientation;
    rotated.vehicles[0].state.angular_velocity_rad_s =
        world_rotation.rotate(first.vehicles[0].state.angular_velocity_rad_s);

    first.step_fixed(4_000).unwrap();
    rotated.step_fixed(4_000).unwrap();

    let expected_omega = world_rotation.rotate(first.vehicles[0].state.angular_velocity_rad_s);
    assert!((rotated.vehicles[0].state.angular_velocity_rad_s - expected_omega).length() < 2.0e-10);
    for basis in [Vec3::X, Vec3::Y, Vec3::FORWARD] {
        let expected = world_rotation.rotate(first.vehicles[0].state.orientation.rotate(basis));
        let actual = rotated.vehicles[0].state.orientation.rotate(basis);
        assert!((actual - expected).length() < 2.0e-10);
    }
}

#[test]
fn offset_vehicle_impact_generates_yaw_and_conserves_planar_angular_momentum() {
    let mut world = PhysicsWorld::demo(2);
    world.static_colliders.clear();
    world.config.gravity_mps2 = 0.0;
    world.config.automatic_lod = false;
    for vehicle in &mut world.vehicles {
        vehicle.definition.chassis.drag_coefficient = 0.0;
        vehicle.definition.chassis.lift_coefficient = 0.0;
        vehicle.state.powertrain.fuel_kg = 0.0;
        vehicle.state.position_m.y = 100.0;
        vehicle.state.orientation = Quat::IDENTITY;
        vehicle.state.angular_velocity_rad_s = Vec3::ZERO;
    }
    world.vehicles[0].state.position_m.x = -0.5;
    world.vehicles[0].state.position_m.z = 0.0;
    world.vehicles[1].state.position_m.x = 0.5;
    world.vehicles[1].state.position_m.z = -3.5;
    world.vehicles[0].state.linear_velocity_mps.z = -10.0;
    world.vehicles[1].state.linear_velocity_mps.z = 10.0;

    let before = world.vehicles.iter().map(world_angular_momentum).fold(Vec3::ZERO, |sum, value| sum + value);
    let impact_point = Vec3::new(0.0, 100.0, -1.75);
    let normal = Vec3::FORWARD;
    let relative_speed_before =
        (world.vehicles[1].state.linear_velocity_mps - world.vehicles[0].state.linear_velocity_mps).dot(normal);
    world.step_fixed(1).unwrap();
    let after = world.vehicles.iter().map(world_angular_momentum).fold(Vec3::ZERO, |sum, value| sum + value);
    let yaw_a = world.vehicles[0].state.angular_velocity_rad_s.y;
    let yaw_b = world.vehicles[1].state.angular_velocity_rad_s.y;

    assert!(yaw_a.abs() > 0.1, "first vehicle yaw rate={yaw_a}");
    assert!(yaw_b.abs() > 0.1, "second vehicle yaw rate={yaw_b}");
    assert!((after.y - before.y).abs() < 1.0e-8, "before Ly={}, after Ly={}", before.y, after.y);
    let relative_speed_after = (contact_velocity(&world.vehicles[1], impact_point)
        - contact_velocity(&world.vehicles[0], impact_point))
    .dot(normal);
    let restitution = relative_speed_after / -relative_speed_before;
    assert!((restitution - 0.18).abs() < 1.0e-10, "restitution={restitution}");
}

#[test]
fn offset_static_wall_impact_applies_angular_impulse() {
    let mut world = collision_world(
        CollisionShape::Box { half_extents_m: Vec3::new(2.0, 1.0, 0.4) },
        Vec3::new(0.0, 100.0, -2.3),
        Quat::IDENTITY,
    );
    world.config.gravity_mps2 = 0.0;
    world.static_colliders[0].friction = 0.0;
    let vehicle = &mut world.vehicles[0];
    vehicle.definition.chassis.drag_coefficient = 0.0;
    vehicle.definition.chassis.lift_coefficient = 0.0;
    vehicle.state.powertrain.fuel_kg = 0.0;
    vehicle.state.position_m = Vec3::new(2.4, 100.0, 0.0);
    vehicle.state.linear_velocity_mps = Vec3::new(0.0, 0.0, -10.0);
    vehicle.state.angular_velocity_rad_s = Vec3::ZERO;

    let impact_point = Vec3::new(1.2, 100.0, -2.03);
    let normal = Vec3::new(0.0, 0.0, 1.0);
    let normal_speed_before = vehicle.state.linear_velocity_mps.dot(normal);
    world.step_fixed(1).unwrap();

    let yaw_rate = world.vehicles[0].state.angular_velocity_rad_s.y;
    assert!(yaw_rate.abs() > 0.1, "yaw rate={yaw_rate}");
    assert!(world.vehicles[0].state.damage.body > 0.0);
    let normal_speed_after = contact_velocity(&world.vehicles[0], impact_point).dot(normal);
    let restitution = normal_speed_after / -normal_speed_before;
    assert!((restitution - 0.1).abs() < 1.0e-10, "restitution={restitution}");
}

#[test]
fn static_collision_friction_obeys_coulomb_limit_and_does_not_add_energy() {
    let mut world = collision_world(
        CollisionShape::Box { half_extents_m: Vec3::new(4.0, 1.0, 0.4) },
        Vec3::new(0.0, 100.0, -2.3),
        Quat::IDENTITY,
    );
    world.config.gravity_mps2 = 0.0;
    world.static_colliders[0].restitution = 0.1;
    world.static_colliders[0].friction = 0.3;
    let vehicle = &mut world.vehicles[0];
    vehicle.definition.chassis.drag_coefficient = 0.0;
    vehicle.definition.chassis.lift_coefficient = 0.0;
    vehicle.state.powertrain.fuel_kg = 0.0;
    vehicle.state.position_m = Vec3::new(0.0, 100.0, 0.0);
    vehicle.state.linear_velocity_mps = Vec3::new(5.0, 0.0, -10.0);
    vehicle.state.angular_velocity_rad_s = Vec3::ZERO;
    let mass = vehicle.mass_kg();
    let energy_before = 0.5 * mass * vehicle.state.linear_velocity_mps.length_squared();
    let velocity_before = vehicle.state.linear_velocity_mps;

    world.step_fixed(1).unwrap();

    let vehicle = &world.vehicles[0];
    let impulse = (vehicle.state.linear_velocity_mps - velocity_before) * mass;
    let normal_impulse = impulse.z.abs();
    let tangent_impulse = impulse.x.abs();
    let energy_after = 0.5 * mass * vehicle.state.linear_velocity_mps.length_squared() + rotational_energy(vehicle);
    assert!(tangent_impulse <= 0.3 * normal_impulse + 1.0e-9);
    assert!(energy_after <= energy_before + 1.0e-8, "before={energy_before}, after={energy_after}");
}

#[test]
fn reference_braking_distance_is_finite_and_plausible() {
    let mut world = PhysicsWorld::demo(1);
    world.set_input(0, DriverInput { throttle: 1.0, ..DriverInput::default() }).unwrap();
    world.step_fixed(4_000).unwrap();
    let speed_before = world.vehicles[0].telemetry.speed_mps;
    let start = world.vehicles[0].state.position_m;
    world.set_input(0, DriverInput { brake: 1.0, ..DriverInput::default() }).unwrap();
    world.step_fixed(3_000).unwrap();
    let distance = (world.vehicles[0].state.position_m - start).length();
    assert!(speed_before > 3.0);
    assert!(distance > 0.5 && distance < 100.0, "distance={distance}");
    assert!(world.vehicles[0].telemetry.speed_mps < speed_before * 0.5);
}

#[test]
fn quality_can_be_selected_automatically_or_manually() {
    let mut world = PhysicsWorld::demo(2);
    world.set_fidelity_ceiling(Fidelity::Low);
    world.step_fixed(2_000).unwrap();
    assert!(world.vehicles.iter().all(|vehicle| vehicle.target_fidelity <= Fidelity::Low.scalar()));
    world.config.automatic_lod = false;
    world.set_vehicle_fidelity(1, Fidelity::High).unwrap();
    assert_eq!(world.vehicles[1].target_fidelity, Fidelity::High.scalar());
}
