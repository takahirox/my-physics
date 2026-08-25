//! Native acceptance probe for digital steering and the full-size circuit.

use my_physics::circuit::{nearest_segment, segments};
use my_physics::math::{Quat, Vec3};
use my_physics::{DriverInput, KeyboardSteeringAssist, PhysicsWorld, SimulationConfig, VehicleDefinition};

fn yaw(q: Quat) -> f64 {
    (2.0 * (q.w * q.y + q.x * q.z)).atan2(1.0 - 2.0 * (q.y * q.y + q.x * q.x))
}

fn steering_response(speed_mps: f64, command: f64) -> (f64, f64, f64) {
    let mut definition = VehicleDefinition::race_gameplay();
    definition.transmission.automatic = false;
    let mut world = PhysicsWorld::new(SimulationConfig::default());
    let index = world.add_vehicle(definition);
    world.vehicles[index].state.powertrain.gear = 0;
    world.vehicles[index].driver_aids.stability_control_enabled = false;
    world.vehicles[index].driver_aids.traction_control_enabled = false;
    world.step_fixed(2_000).unwrap();
    {
        let vehicle = &mut world.vehicles[index];
        vehicle.state.orientation = Quat::IDENTITY;
        vehicle.previous_orientation = Quat::IDENTITY;
        vehicle.state.linear_velocity_mps = Vec3::new(0.0, 0.0, -speed_mps);
        vehicle.state.angular_velocity_rad_s = Vec3::ZERO;
        for (wheel, definition) in vehicle.state.wheels.iter_mut().zip(vehicle.definition.wheels.iter()) {
            wheel.angular_velocity_rad_s = speed_mps / definition.radius_m;
        }
    }
    let mut assist = KeyboardSteeringAssist::default();
    let mut maximum_slip_rad: f64 = 0.0;
    let start_yaw = yaw(world.vehicles[index].state.orientation);
    for _ in 0..1_000 {
        let steering = assist.update(command, world.vehicles[index].telemetry.speed_mps, 0.001);
        world.set_input_unrecorded(index, DriverInput { steering, ..DriverInput::default() }).unwrap();
        world.step_fixed(1).unwrap();
        maximum_slip_rad = maximum_slip_rad.max(
            world.vehicles[index].state.wheels[..2].iter().map(|wheel| wheel.slip_angle_rad.abs()).fold(0.0, f64::max),
        );
    }
    let vehicle = &world.vehicles[index];
    ((yaw(vehicle.state.orientation) - start_yaw).abs(), maximum_slip_rad, assist.output())
}

fn digital_lap(deadband: f64) -> (usize, f64, f64, f64, f64) {
    let mut world = PhysicsWorld::demo(1);
    let mut assist = KeyboardSteeringAssist::default();
    let mut previous = nearest_segment(world.vehicles[0].state.position_m);
    let mut laps = 0;
    let mut first_lap_time_s = f64::NAN;
    let mut maximum_speed_mps: f64 = 0.0;
    let mut maximum_lateral_error_m: f64 = 0.0;
    for _ in 0..140_000 {
        let vehicle = &world.vehicles[0];
        let mut input = my_physics::circuit::ai_driver_input_with_yaw(
            vehicle.state.position_m,
            vehicle.state.orientation,
            vehicle.telemetry.speed_mps,
            vehicle.telemetry.yaw_rate_rad_s,
            0.0,
        );
        let direction = if input.steering.abs() > deadband { input.steering.signum() } else { 0.0 };
        input.steering = assist.update(direction, vehicle.telemetry.speed_mps, 0.001);
        world.set_input_unrecorded(0, input).unwrap();
        world.step_fixed(1).unwrap();
        let vehicle = &world.vehicles[0];
        maximum_speed_mps = maximum_speed_mps.max(vehicle.telemetry.speed_mps);
        let segment_index = nearest_segment(vehicle.state.position_m);
        let segment = segments()[segment_index];
        maximum_lateral_error_m =
            maximum_lateral_error_m.max(((vehicle.state.position_m - segment.center_m).dot(segment.right)).abs());
        if previous > segments().len() * 4 / 5 && segment_index < segments().len() / 5 {
            laps += 1;
            if first_lap_time_s.is_nan() {
                first_lap_time_s = world.time_s;
            }
        }
        previous = segment_index;
    }
    (laps, first_lap_time_s, maximum_speed_mps, maximum_lateral_error_m, world.vehicles[0].state.damage.body)
}

fn main() {
    for speed_kmh in [50.0, 100.0, 140.0] {
        let half = steering_response(speed_kmh / 3.6, 0.5);
        let full = steering_response(speed_kmh / 3.6, 1.0);
        println!(
            "STEERING speed_kmh={speed_kmh:.0} half_heading_deg={:.2} full_heading_deg={:.2} full_slip_deg={:.2} full_output={:.4}",
            half.0.to_degrees(),
            full.0.to_degrees(),
            full.1.to_degrees(),
            full.2,
        );
    }
    let deadband = 0.02;
    let (laps, lap_time, maximum_speed, maximum_error, damage) = digital_lap(deadband);
    println!(
        "DIGITAL deadband={deadband:.2} laps={laps} lap_time_s={lap_time:.1} max_speed_kmh={:.1} max_lateral_error_m={maximum_error:.2} damage={damage:.3}",
        maximum_speed * 3.6,
    );
}
