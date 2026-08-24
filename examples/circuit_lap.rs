//! Deterministic headless lap driven by the same controller used by the browser demo.

use my_physics::PhysicsWorld;
use my_physics::circuit::{minimum_radius_m, nearest_segment, segments, total_length_m};

fn main() {
    let mut world = PhysicsWorld::demo(1);
    let mut previous = nearest_segment(world.vehicles[0].state.position_m);
    let mut laps = 0;
    let mut maximum_error_m: f64 = 0.0;
    let mut maximum_error_time_s = 0.0;
    let mut maximum_error_segment = 0;
    let mut maximum_speed_mps: f64 = 0.0;
    let mut first_lap_time_s = None;

    for _ in 0..140_000 {
        let vehicle = &world.vehicles[0];
        let input = my_physics::circuit::ai_driver_input_with_yaw(
            vehicle.state.position_m,
            vehicle.state.orientation,
            vehicle.telemetry.speed_mps,
            vehicle.telemetry.yaw_rate_rad_s,
            0.0,
        );
        world.set_input_unrecorded(0, input).expect("vehicle 0 exists");
        world.step_fixed(1).expect("simulation remains finite");

        let vehicle = &world.vehicles[0];
        let segment_index = nearest_segment(vehicle.state.position_m);
        let segment = segments()[segment_index];
        let offset = vehicle.state.position_m - segment.center_m;
        let error_m = offset.dot(segment.right).abs();
        maximum_speed_mps = maximum_speed_mps.max(vehicle.telemetry.speed_mps);
        if error_m > maximum_error_m {
            maximum_error_m = error_m;
            maximum_error_time_s = world.time_s;
            maximum_error_segment = segment_index;
        }
        if previous > segments().len() * 4 / 5 && segment_index < segments().len() / 5 {
            laps += 1;
            first_lap_time_s.get_or_insert(world.time_s);
        }
        previous = segment_index;
    }

    let vehicle = &world.vehicles[0];
    println!(
        "laps={laps} circuit_km={:.3} min_radius_m={:.2} lap_time_s={:.1} speed_kmh={:.1} max_speed_kmh={:.1} max_lateral_error_m={maximum_error_m:.2} max_error_time_s={maximum_error_time_s:.1} max_error_segment={maximum_error_segment} damage={:.3}",
        total_length_m() / 1000.0,
        minimum_radius_m(),
        first_lap_time_s.unwrap_or(f64::NAN),
        vehicle.telemetry.speed_mps * 3.6,
        maximum_speed_mps * 3.6,
        vehicle.state.damage.body,
    );
    assert!(laps >= 1, "AI driver did not complete a lap");
    assert!(maximum_error_m < 4.4, "AI driver left the safe center envelope");
}
