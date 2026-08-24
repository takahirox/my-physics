use my_physics::{DriverInput, PhysicsWorld};

fn main() {
    let seconds = std::env::args().nth(1).and_then(|s| s.parse::<f64>().ok()).unwrap_or(5.0).clamp(0.001, 3600.0);
    let mut world = PhysicsWorld::demo(10);
    world.set_input(0, DriverInput { throttle: 0.72, ..DriverInput::default() }).expect("vehicle 0 exists");
    println!("time_s,speed_mps,engine_rpm,gear,fuel_kg,x_m,y_m,z_m");
    let steps = (seconds / world.config.fixed_dt_s) as u32;
    for n in 0..steps {
        world.step_fixed(1).expect("simulation remains finite");
        if n % 100 == 0 {
            let t = &world.vehicles[0].telemetry;
            println!(
                "{:.3},{:.6},{:.3},{},{:.6},{:.6},{:.6},{:.6}",
                t.time_s, t.speed_mps, t.engine_rpm, t.gear, t.fuel_kg, t.position_m.x, t.position_m.y, t.position_m.z
            );
        }
    }
    eprintln!("fingerprint={:016x}", world.state_fingerprint());
}
