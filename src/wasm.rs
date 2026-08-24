//! Raw WebAssembly exports need stable linker symbols. The narrowly scoped
//! `no_mangle` attributes are the only reason this module relaxes the crate's
//! default unsafe-code lint; no unsafe block or pointer access is used.
#![allow(unsafe_code)]

use crate::{DriverInput, Fidelity, KeyboardSteeringAssist, PhysicsWorld, Quat, Snapshot};
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::{Mutex, OnceLock};

static DEMO: OnceLock<Mutex<PhysicsWorld>> = OnceLock::new();
static SAVED_SNAPSHOT: OnceLock<Mutex<Option<SavedBrowserState>>> = OnceLock::new();
static PLAYER_AUTOPILOT: AtomicBool = AtomicBool::new(false);
static PLAYER_INPUT_MODE: AtomicU8 = AtomicU8::new(0);
static KEYBOARD: OnceLock<Mutex<KeyboardInputState>> = OnceLock::new();

#[derive(Clone, Copy, Debug, Default)]
struct KeyboardInputState {
    assist: KeyboardSteeringAssist,
    command: DriverInput,
}

#[derive(Clone, Debug)]
struct SavedBrowserState {
    world: Snapshot,
    keyboard: KeyboardInputState,
    player_input_mode: u8,
    player_autopilot: bool,
}

fn keyboard() -> &'static Mutex<KeyboardInputState> {
    KEYBOARD.get_or_init(|| Mutex::new(KeyboardInputState::default()))
}
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
fn saved_snapshot() -> &'static Mutex<Option<SavedBrowserState>> {
    SAVED_SNAPSHOT.get_or_init(|| Mutex::new(None))
}
fn yaw(q: Quat) -> f64 {
    (2.0 * (q.w * q.y + q.x * q.z)).atan2(1.0 - 2.0 * (q.y * q.y + q.x * q.x))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_reset() {
    with_world(|w| *w = PhysicsWorld::demo(10));
    PLAYER_AUTOPILOT.store(false, Ordering::Relaxed);
    PLAYER_INPUT_MODE.store(0, Ordering::Relaxed);
    *keyboard().lock().unwrap_or_else(|error| error.into_inner()) = KeyboardInputState::default();
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_set_input(steering: f64, throttle: f64, brake: f64, clutch: f64, handbrake: f64, gear: i32) {
    PLAYER_INPUT_MODE.store(0, Ordering::Relaxed);
    keyboard().lock().unwrap_or_else(|error| error.into_inner()).assist.reset();
    with_world(|w| {
        let _ = w.set_input(0, DriverInput { steering, throttle, brake, clutch, handbrake, gear_request: gear as i8 });
    });
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_set_keyboard_input(
    direction: f64,
    throttle: f64,
    brake: f64,
    clutch: f64,
    handbrake: f64,
    gear: i32,
) {
    PLAYER_INPUT_MODE.store(1, Ordering::Relaxed);
    let mut state = keyboard().lock().unwrap_or_else(|error| error.into_inner());
    state.command =
        DriverInput { steering: direction, throttle, brake, clutch, handbrake, gear_request: gear as i8 }.sanitized();
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_step(steps: u32) {
    with_world(|w| {
        for _ in 0..steps {
            if !PLAYER_AUTOPILOT.load(Ordering::Relaxed) && PLAYER_INPUT_MODE.load(Ordering::Relaxed) == 1 {
                let speed = w.vehicles.first().map_or(0.0, |vehicle| vehicle.telemetry.speed_mps);
                let mut keyboard = keyboard().lock().unwrap_or_else(|error| error.into_inner());
                let command = keyboard.command;
                let steering = keyboard.assist.update(command.steering, speed, w.config.fixed_dt_s);
                let _ = w.set_input_unrecorded(0, DriverInput { steering, ..command });
            }
            let ai_start = if PLAYER_AUTOPILOT.load(Ordering::Relaxed) { 0 } else { 1 };
            for n in ai_start..w.vehicles.len() {
                let vehicle = &w.vehicles[n];
                let lane = if n % 2 == 0 { -0.35 } else { 0.35 };
                let input = crate::circuit::ai_driver_input_with_yaw(
                    vehicle.state.position_m,
                    vehicle.state.orientation,
                    vehicle.telemetry.speed_mps,
                    vehicle.telemetry.yaw_rate_rad_s,
                    lane,
                );
                let _ = w.set_input_unrecorded(n, input);
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
pub extern "C" fn physics_track_half_width() -> f64 {
    crate::world::DEMO_TRACK_HALF_WIDTH_M
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_track_segment_count() -> u32 {
    crate::circuit::segments().len() as u32
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_track_length() -> f64 {
    crate::circuit::total_length_m()
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_track_segment_x(index: u32) -> f64 {
    crate::circuit::segments().get(index as usize).map_or(f64::NAN, |segment| segment.center_m.x)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_track_segment_z(index: u32) -> f64 {
    crate::circuit::segments().get(index as usize).map_or(f64::NAN, |segment| segment.center_m.z)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_track_segment_yaw(index: u32) -> f64 {
    crate::circuit::segments().get(index as usize).map_or(f64::NAN, |segment| segment.yaw_rad)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_track_segment_length(index: u32) -> f64 {
    crate::circuit::segments().get(index as usize).map_or(f64::NAN, |segment| segment.length_m)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_track_progress(index: u32) -> f64 {
    read_vehicle(index, |vehicle| {
        crate::circuit::nearest_segment(vehicle.state.position_m) as f64 / crate::circuit::segments().len() as f64
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_set_player_autopilot(enabled: u32) {
    PLAYER_AUTOPILOT.store(enabled != 0, Ordering::Relaxed);
    if enabled != 0 {
        keyboard().lock().unwrap_or_else(|error| error.into_inner()).assist.reset();
    }
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_set_player_esc(enabled: u32) {
    with_world(|world| {
        if let Some(vehicle) = world.vehicles.first_mut() {
            vehicle.driver_aids.stability_control_enabled = enabled != 0;
        }
    });
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_player_esc() -> u32 {
    with_world(|world| {
        u32::from(world.vehicles.first().is_some_and(|vehicle| vehicle.driver_aids.stability_control_enabled))
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_player_autopilot() -> u32 {
    u32::from(PLAYER_AUTOPILOT.load(Ordering::Relaxed))
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
    read_vehicle(i, |v| yaw(v.state.orientation))
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_render_x(i: u32, alpha: f64) -> f64 {
    read_vehicle(i, |v| v.interpolated_state(alpha).position_m.x)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_render_y(i: u32, alpha: f64) -> f64 {
    read_vehicle(i, |v| v.interpolated_state(alpha).position_m.y)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_render_z(i: u32, alpha: f64) -> f64 {
    read_vehicle(i, |v| v.interpolated_state(alpha).position_m.z)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_render_yaw(i: u32, alpha: f64) -> f64 {
    read_vehicle(i, |v| yaw(v.interpolated_state(alpha).orientation))
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
pub extern "C" fn physics_steering(i: u32) -> f64 {
    read_vehicle(i, |v| v.control.steering)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_esc_active(i: u32) -> f64 {
    read_vehicle(i, |v| if v.control.esc_active { 1.0 } else { 0.0 })
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
#[unsafe(no_mangle)]
pub extern "C" fn physics_ffb_steering_torque(i: u32) -> f64 {
    read_vehicle(i, |v| v.force_feedback.steering_torque_nm)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_ffb_vibration(i: u32) -> f64 {
    read_vehicle(i, |v| v.force_feedback.road_vibration.max(v.force_feedback.abs_pulse))
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_audio_engine_load(i: u32) -> f64 {
    read_vehicle(i, |v| v.audio.engine_load)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_audio_tire_scrub(i: u32) -> f64 {
    read_vehicle(i, |v| v.audio.tire_scrub.into_iter().fold(0.0, f64::max))
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_snapshot_save() -> u32 {
    // Keep the lock order world -> keyboard everywhere that needs both. Raw
    // input drops the keyboard lock before taking world, so restore cannot
    // form a reverse-order cycle.
    let state = with_world(|world| SavedBrowserState {
        world: world.snapshot(),
        keyboard: *keyboard().lock().unwrap_or_else(|error| error.into_inner()),
        player_input_mode: PLAYER_INPUT_MODE.load(Ordering::Relaxed),
        player_autopilot: PLAYER_AUTOPILOT.load(Ordering::Relaxed),
    });
    let size = state.world.to_bytes().len();
    let mut saved = saved_snapshot().lock().unwrap_or_else(|error| error.into_inner());
    *saved = Some(state);
    size as u32
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_snapshot_restore() -> u32 {
    let state = saved_snapshot().lock().unwrap_or_else(|error| error.into_inner()).clone();
    let Some(state) = state else {
        return 0;
    };
    with_world(|world| {
        world.restore(&state.world);
        *keyboard().lock().unwrap_or_else(|error| error.into_inner()) = state.keyboard;
        PLAYER_INPUT_MODE.store(state.player_input_mode, Ordering::Relaxed);
        PLAYER_AUTOPILOT.store(state.player_autopilot, Ordering::Relaxed);
    });
    1
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_set_quality(level: u32) {
    let fidelity = match level {
        0 => Fidelity::Low,
        1 => Fidelity::Medium,
        _ => Fidelity::High,
    };
    with_world(|world| world.set_fidelity_ceiling(fidelity));
}
