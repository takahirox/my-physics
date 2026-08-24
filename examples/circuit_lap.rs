//! Deterministic headless lap driven by the same controller used by the browser demo.

use my_physics::PhysicsWorld;
use my_physics::circuit::{ai_driver_input, nearest_segment, segments, total_length_m};

fn main() {
    let mut world = PhysicsWorld::demo(1);
    let mut previous = nearest_segment(world.vehicles[0].state.position_m);
    let mut laps = 0;
    let mut maximum_error_m: f64 = 0.0;
    let mut maximum_error_time_s = 0.0;
    let mut maximum_error_segment = 0;

    for _ in 0..90_000 {
        let vehicle = &world.vehicles[0];
        let input =
            ai_driver_input(vehicle.state.position_m, vehicle.state.orientation, vehicle.telemetry.speed_mps, 0.0);
        world.set_input_unrecorded(0, input).expect("vehicle 0 exists");
        world.step_fixed(1).expect("simulation remains finite");

        let vehicle = &world.vehicles[0];
        let segment_index = nearest_segment(vehicle.state.position_m);
        let segment = segments()[segment_index];
        let offset = vehicle.state.position_m - segment.center_m;
        let error_m = (offset.x * offset.x + offset.z * offset.z).sqrt();
        if error_m > maximum_error_m {
            maximum_error_m = error_m;
            maximum_error_time_s = world.time_s;
            maximum_error_segment = segment_index;
        }
        if previous > segments().len() * 4 / 5 && segment_index < segments().len() / 5 {
            laps += 1;
        }
        previous = segment_index;
    }

    let vehicle = &world.vehicles[0];
    println!(
        "laps={laps} circuit_km={:.3} speed_kmh={:.1} max_centerline_error_m={maximum_error_m:.2} max_error_time_s={maximum_error_time_s:.1} max_error_segment={maximum_error_segment} damage={:.3}",
        total_length_m() / 1000.0,
        vehicle.telemetry.speed_mps * 3.6,
        vehicle.state.damage.body,
    );
    assert!(laps >= 1, "AI driver did not complete a lap");
    assert!(maximum_error_m < 5.2, "AI driver left the circuit");
}
