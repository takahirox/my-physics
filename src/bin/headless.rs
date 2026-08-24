use my_physics::{DriverInput, PhysicsWorld};
use std::fmt::Write as _;
use std::time::Instant;

fn main() {
    let seconds = std::env::args().nth(1).and_then(|s| s.parse::<f64>().ok()).unwrap_or(5.0).clamp(0.001, 3600.0);
    let mut world = PhysicsWorld::demo(10);
    world.set_input(0, DriverInput { throttle: 0.72, ..DriverInput::default() }).expect("vehicle 0 exists");
    let mut header = String::from(
        "time_s,speed_mps,engine_rpm,gear,fuel_kg,x_m,y_m,z_m,engine_temp_k,oil_pressure_pa,clutch_temp_k,clutch_wear,gearbox_wear,body_damage,tc_active,esc_active",
    );
    for wheel in ["fl", "fr", "rl", "rr"] {
        write!(header, ",{wheel}_slip,{wheel}_tire_temp_k,{wheel}_pressure_pa,{wheel}_load_n,{wheel}_brake_temp_k,{wheel}_hydro,{wheel}_abs").unwrap();
    }
    println!("{header}");
    let steps = (seconds / world.config.fixed_dt_s) as u32;
    let wall_start = Instant::now();
    for n in 0..steps {
        world.step_fixed(1).expect("simulation remains finite");
        if n % 100 == 0 {
            let t = &world.vehicles[0].telemetry;
            let mut row = format!(
                "{:.3},{:.6},{:.3},{},{:.6},{:.6},{:.6},{:.6},{:.3},{:.1},{:.3},{:.8},{:.8},{:.8},{},{}",
                t.time_s,
                t.speed_mps,
                t.engine_rpm,
                t.gear,
                t.fuel_kg,
                t.position_m.x,
                t.position_m.y,
                t.position_m.z,
                t.engine_temperature_k,
                t.oil_pressure_pa,
                t.clutch_temperature_k,
                t.clutch_wear,
                t.gearbox_wear,
                t.body_damage,
                t.tc_active,
                t.esc_active,
            );
            for wheel in 0..4 {
                write!(
                    row,
                    ",{:.8},{:.3},{:.1},{:.3},{:.3},{:.8},{}",
                    t.wheel_slip[wheel],
                    t.tire_temperature_k[wheel],
                    t.tire_pressure_pa[wheel],
                    t.normal_load_n[wheel],
                    t.brake_temperature_k[wheel],
                    t.hydroplaning[wheel],
                    t.abs_active[wheel],
                )
                .unwrap();
            }
            println!("{row}");
        }
    }
    let wall_seconds = wall_start.elapsed().as_secs_f64();
    eprintln!(
        "fingerprint={:016x} wall_seconds={wall_seconds:.6} realtime_factor={:.2}",
        world.state_fingerprint(),
        seconds / wall_seconds
    );
}
