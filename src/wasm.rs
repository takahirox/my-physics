//! Raw WebAssembly exports need stable linker symbols. The narrowly scoped
//! `no_mangle` attributes are the only reason this module relaxes the crate's
//! default unsafe-code lint; no unsafe block or pointer access is used.
#![allow(unsafe_code)]

use crate::{DriverInput, PhysicsWorld};
use std::sync::{Mutex, OnceLock};

static DEMO: OnceLock<Mutex<PhysicsWorld>> = OnceLock::new();
fn demo() -> &'static Mutex<PhysicsWorld> {
    DEMO.get_or_init(|| Mutex::new(PhysicsWorld::demo(10)))
}
fn with_world<R>(f: impl FnOnce(&mut PhysicsWorld) -> R) -> R {
    let mut guard = demo().lock().unwrap_or_else(|e| e.into_inner());
    f(&mut guard)
}
fn read_vehicle(index: u32, f: impl FnOnce(&crate::vehicle::Vehicle) -> f64) -> f64 {
    with_world(|w| w.vehicles.get(index as usize).map_or(f64::NAN, f))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_reset() {
    with_world(|w| *w = PhysicsWorld::demo(10));
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_set_input(steering: f64, throttle: f64, brake: f64, handbrake: f64, gear: i32) {
    with_world(|w| {
        let _ = w.set_input_unrecorded(
            0,
            DriverInput { steering, throttle, brake, handbrake, gear_request: gear as i8, ..DriverInput::default() },
        );
    });
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_step(steps: u32) {
    with_world(|w| {
        for _ in 0..steps {
            let time = w.time_s;
            for n in 1..w.vehicles.len() {
                let phase = n as f64 * 0.7;
                let steering = (time * 0.22 + phase).sin() * 0.18;
                let _ = w.set_input_unrecorded(
                    n,
                    DriverInput {
                        steering,
                        throttle: 0.48 + 0.08 * (phase + time * 0.1).sin(),
                        ..DriverInput::default()
                    },
                );
            }
            let _ = w.step_fixed(1);
        }
    });
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_vehicle_count() -> u32 {
    with_world(|w| w.vehicles.len() as u32)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_time() -> f64 {
    with_world(|w| w.time_s)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_x(i: u32) -> f64 {
    read_vehicle(i, |v| v.state.position_m.x)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_y(i: u32) -> f64 {
    read_vehicle(i, |v| v.state.position_m.y)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_z(i: u32) -> f64 {
    read_vehicle(i, |v| v.state.position_m.z)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_yaw(i: u32) -> f64 {
    read_vehicle(i, |v| {
        let q = v.state.orientation;
        (2.0 * (q.w * q.y + q.x * q.z)).atan2(1.0 - 2.0 * (q.y * q.y + q.x * q.x))
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_speed(i: u32) -> f64 {
    read_vehicle(i, |v| v.telemetry.speed_mps)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_rpm(i: u32) -> f64 {
    read_vehicle(i, |v| v.state.powertrain.engine_rpm)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_gear(i: u32) -> f64 {
    read_vehicle(i, |v| v.state.powertrain.gear as f64)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_tire_temp(i: u32, wheel: u32) -> f64 {
    read_vehicle(i, |v| v.state.wheels.get(wheel as usize).map_or(f64::NAN, |w| w.tire.tread_temperature_k))
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_tire_pressure(i: u32, wheel: u32) -> f64 {
    read_vehicle(i, |v| v.state.wheels.get(wheel as usize).map_or(f64::NAN, |w| w.tire.pressure_pa))
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_fidelity(i: u32) -> f64 {
    read_vehicle(i, |v| v.fidelity)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_damage(i: u32) -> f64 {
    read_vehicle(i, |v| v.state.damage.body)
}
