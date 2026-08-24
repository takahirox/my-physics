use my_physics::road::RoadCell;
use my_physics::tire::{TireFailure, TireState};
use my_physics::{
    ArchiveError, DriverInput, MagicFormulaTire, PhysicsWorld, SimulationConfig, Snapshot, StepError, TireInput,
    TireModel, decode_input_history, encode_input_history,
};

#[test]
fn fixed_step_is_bit_repeatable() {
    fn run() -> u64 {
        let mut w = PhysicsWorld::demo(2);
        w.set_input(0, DriverInput { throttle: 0.65, steering: 0.08, ..DriverInput::default() }).unwrap();
        w.step_fixed(2500).unwrap();
        w.state_fingerprint()
    }
    assert_eq!(run(), run());
}

#[test]
fn snapshot_restore_and_input_replay_are_equivalent() {
    let mut original = PhysicsWorld::demo(1);
    original.set_input(0, DriverInput { throttle: 0.5, ..DriverInput::default() }).unwrap();
    original.step_fixed(300).unwrap();
    let snapshot = original.snapshot();
    let start = original.recorded_inputs.len();
    original.set_input(0, DriverInput { throttle: 0.85, steering: -0.12, ..DriverInput::default() }).unwrap();
    original.step_fixed(700).unwrap();
    let expected = original.state_fingerprint();
    let inputs = original.recorded_inputs[start..].to_vec();
    let mut replay = PhysicsWorld::new(SimulationConfig::default());
    replay.replay_from(&snapshot, &inputs, 1000).unwrap();
    assert_eq!(expected, replay.state_fingerprint());
}

#[test]
fn high_priority_vehicle_runs_at_one_kilohertz() {
    let mut w = PhysicsWorld::demo(10);
    w.step_fixed(1000).unwrap();
    assert!((w.time_s - 1.0).abs() < 1.0e-12);
    assert!((w.vehicles[0].state.simulation_time_s - 1.0).abs() < 1.0e-12);
}

#[test]
fn throttle_accelerates_reference_vehicle_forward() {
    let mut w = PhysicsWorld::demo(1);
    w.set_input(0, DriverInput { throttle: 1.0, ..DriverInput::default() }).unwrap();
    w.step_fixed(4000).unwrap();
    assert!(w.vehicles[0].state.linear_velocity_mps.z < -0.5, "velocity={:?}", w.vehicles[0].state.linear_velocity_mps);
}

#[test]
fn automatic_full_throttle_accelerates_without_chain_shift_failure() {
    let mut world = PhysicsWorld::demo(1);
    world.set_input(0, DriverInput { throttle: 1.0, gear_request: 0, ..DriverInput::default() }).unwrap();

    world.step_fixed(5_000).unwrap();
    let speed_at_five_seconds = world.vehicles[0].telemetry.speed_mps;
    world.step_fixed(15_000).unwrap();
    let vehicle = &world.vehicles[0];

    assert!(speed_at_five_seconds > 15.0);
    assert!(vehicle.telemetry.speed_mps > speed_at_five_seconds + 20.0);
    assert!(vehicle.state.powertrain.gear >= 3);
    assert!(vehicle.state.powertrain.engine_rpm <= vehicle.definition.engine.redline_rpm + 50.0);
    assert!(vehicle.state.powertrain.overrev_damage < 1.0e-8);
    assert!(!vehicle.state.powertrain.failed);
}

#[test]
fn wet_road_reduces_available_friction() {
    let model = MagicFormulaTire::default();
    let input = |water| TireInput {
        normal_load_n: 3700.0,
        longitudinal_slip: 0.12,
        slip_angle_rad: 0.0,
        camber_rad: 0.0,
        speed_mps: 30.0,
        road: RoadCell { water_depth_m: water, ..RoadCell::default() },
        dt: 0.001,
    };
    let dry = model.evaluate(&mut TireState::default(), input(0.0));
    let wet = model.evaluate(&mut TireState::default(), input(0.004));
    assert!(wet.longitudinal_force_n < dry.longitudinal_force_n);
    assert!(wet.hydroplaning > 0.0);
}

#[test]
fn puncture_has_progressive_physical_state() {
    let model = MagicFormulaTire::default();
    let mut state = TireState { puncture_area_m2: 0.00008, ..TireState::default() };
    let input = TireInput {
        normal_load_n: 3700.0,
        longitudinal_slip: 0.1,
        slip_angle_rad: 0.03,
        camber_rad: 0.0,
        speed_mps: 35.0,
        road: RoadCell::default(),
        dt: 0.01,
    };
    let start_pressure = state.pressure_pa;
    for _ in 0..1000 {
        model.evaluate(&mut state, input);
    }
    assert!(state.pressure_pa < start_pressure);
    assert_ne!(state.failure, TireFailure::Healthy);
    assert!(state.contact_patch_m2 > 0.012);
}

#[test]
fn invalid_variable_steps_are_rejected() {
    let mut w = PhysicsWorld::demo(1);
    assert_eq!(w.step_variable(0.2), Err(StepError::InvalidTimestep));
}

#[test]
fn render_clock_is_not_part_of_physics() {
    let mut a = PhysicsWorld::demo(1);
    let mut b = a.clone();
    a.step_fixed(1000).unwrap();
    for _frame in 0..50 {
        b.step_fixed(20).unwrap();
    }
    assert_eq!(a.state_fingerprint(), b.state_fingerprint());
}

#[test]
fn braking_dissipates_vehicle_speed() {
    let mut w = PhysicsWorld::demo(1);
    w.set_input(0, DriverInput { throttle: 1.0, ..DriverInput::default() }).unwrap();
    w.step_fixed(3500).unwrap();
    let initial = w.vehicles[0].telemetry.speed_mps;
    w.set_input(0, DriverInput { brake: 1.0, ..DriverInput::default() }).unwrap();
    w.step_fixed(2500).unwrap();
    assert!(w.vehicles[0].telemetry.speed_mps < initial * 0.55);
    assert!(w.vehicles[0].state.wheels.iter().any(|wheel| wheel.brake_temperature_k > 300.0));
}

#[test]
fn fuel_mass_moves_the_center_of_gravity() {
    let mut w = PhysicsWorld::demo(1);
    let full_mass = w.vehicles[0].mass_kg();
    let full_cg = w.vehicles[0].cg_local_m();
    w.vehicles[0].state.powertrain.fuel_kg = 0.0;
    assert!(w.vehicles[0].mass_kg() < full_mass);
    assert_ne!(w.vehicles[0].cg_local_m(), full_cg);
}

#[test]
fn severe_physical_damage_detaches_an_independent_body() {
    let mut w = PhysicsWorld::demo(1);
    w.vehicles[0].state.damage.body = 0.8;
    let mass_before = w.vehicles[0].mass_kg();
    w.step_fixed(1).unwrap();
    assert_eq!(w.detached_bodies.len(), 1);
    assert!(w.vehicles[0].mass_kg() < mass_before);
}

#[test]
fn engine_damage_accumulates_with_exposure_duration() {
    let mut w = PhysicsWorld::demo(1);
    w.vehicles[0].state.powertrain.engine_temperature_k = 480.0;
    w.vehicles[0].state.powertrain.coolant_temperature_k = 470.0;
    w.step_fixed(100).unwrap();
    let first = w.vehicles[0].state.powertrain.overheat_damage;
    w.step_fixed(100).unwrap();
    assert!(first > 0.0);
    assert!(w.vehicles[0].state.powertrain.overheat_damage > first);
}

#[test]
fn vehicle_collision_applies_impulse_and_physical_damage() {
    let mut w = PhysicsWorld::demo(2);
    w.vehicles[0].state.position_m.x = 0.0;
    w.vehicles[0].state.position_m.z = 0.0;
    w.vehicles[1].state.position_m.x = 0.0;
    w.vehicles[1].state.position_m.z = -3.5;
    w.vehicles[0].state.linear_velocity_mps.z = -10.0;
    w.vehicles[1].state.linear_velocity_mps.z = 10.0;
    w.step_fixed(1).unwrap();
    assert!(w.vehicles[0].state.damage.body > 0.0);
    assert!(w.vehicles[1].state.damage.body > 0.0);
    assert!(w.vehicles[0].state.linear_velocity_mps.z > -10.0);
}

#[test]
fn versioned_snapshot_archive_round_trips_complete_state() {
    let mut world = PhysicsWorld::demo(10);
    world.wind_mps = my_physics::Vec3::new(2.0, 0.0, -4.0);
    world.rain_rate_m_s = 0.000_01;
    world.set_input(0, DriverInput { throttle: 0.7, steering: 0.1, ..DriverInput::default() }).unwrap();
    world.step_fixed(800).unwrap();
    let original = world.snapshot();
    let bytes = original.to_bytes();
    let decoded = Snapshot::from_bytes(&bytes).unwrap();
    assert_eq!(decoded, original);
    assert_eq!(decoded.fingerprint(), original.fingerprint());
}

#[test]
fn snapshot_archive_detects_corruption() {
    let mut bytes = PhysicsWorld::demo(1).snapshot().to_bytes();
    bytes[32] ^= 0x40;
    assert_eq!(Snapshot::from_bytes(&bytes), Err(ArchiveError::ChecksumMismatch));
}

#[test]
fn timed_input_history_is_persistent() {
    let mut world = PhysicsWorld::demo(1);
    world.set_input(0, DriverInput { throttle: 0.4, ..DriverInput::default() }).unwrap();
    world.step_fixed(10).unwrap();
    world.set_input(0, DriverInput { brake: 0.2, steering: -0.3, ..DriverInput::default() }).unwrap();
    let bytes = encode_input_history(&world.recorded_inputs);
    assert_eq!(decode_input_history(&bytes).unwrap(), world.recorded_inputs);
}

#[test]
fn render_state_interpolates_without_changing_physics() {
    let mut world = PhysicsWorld::demo(1);
    world.vehicles[0].state.linear_velocity_mps.z = -10.0;
    world.step_fixed(1).unwrap();
    let before = world.state_fingerprint();
    let start = world.vehicles[0].interpolated_state(0.0);
    let middle = world.vehicles[0].interpolated_state(0.5);
    let end = world.vehicles[0].interpolated_state(1.0);
    assert!((middle.position_m.z - (start.position_m.z + end.position_m.z) * 0.5).abs() < 1.0e-12);
    assert_eq!(world.state_fingerprint(), before);
}

#[test]
fn reference_scenario_matches_cross_platform_golden_telemetry() {
    let mut world = PhysicsWorld::demo(10);
    world.set_input(0, DriverInput { throttle: 0.72, ..DriverInput::default() }).unwrap();
    world.step_fixed(2_000).unwrap();
    let telemetry = &world.vehicles[0].telemetry;
    assert!((telemetry.speed_mps - 7.953_846_667_792).abs() < 0.02);
    assert!((telemetry.position_m.z - -7.475_064_234_748).abs() < 0.02);
    assert!((telemetry.engine_rpm - 3_986.269_310_490_245).abs() < 5.0);
    assert!((telemetry.fuel_kg - 39.993_009_084_168).abs() < 0.000_1);
    assert!((telemetry.tire_temperature_k[0] - 321.096_009_724_941).abs() < 0.05);
    assert!((telemetry.tire_temperature_k[2] - 324.878_940_340_500).abs() < 0.05);
}
