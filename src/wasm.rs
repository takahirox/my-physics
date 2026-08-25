//! Raw WebAssembly exports need stable linker symbols. The narrowly scoped
//! `no_mangle` attributes are the only reason this module relaxes the crate's
//! default unsafe-code lint; no unsafe block or pointer access is used.
#![allow(unsafe_code)]

use crate::{DriverInput, Fidelity, KeyboardSteeringAssist, PhysicsWorld, Quat, Snapshot, VehiclePreset};
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::{Mutex, OnceLock};

static DEMO: OnceLock<Mutex<PhysicsWorld>> = OnceLock::new();
static SAVED_SNAPSHOT: OnceLock<Mutex<Option<SavedBrowserState>>> = OnceLock::new();
static VALIDATION_REPORT: OnceLock<Mutex<Option<crate::validation::ScenarioReport>>> = OnceLock::new();
static PLAYER_AUTOPILOT: AtomicBool = AtomicBool::new(false);
static PLAYER_INPUT_MODE: AtomicU8 = AtomicU8::new(0);
static KEYBOARD_ASSIST_ENABLED: AtomicBool = AtomicBool::new(true);
static EXPERIENCE_PROFILE: AtomicU8 = AtomicU8::new(1);
static DEMO_VEHICLE_PRESET: AtomicU8 = AtomicU8::new(1);
static KEYBOARD: OnceLock<Mutex<KeyboardInputState>> = OnceLock::new();

const PROFILE_ACCESSIBLE: u8 = 0;
const PROFILE_SPORT: u8 = 1;
const PROFILE_SIMULATION: u8 = 2;
const PROFILE_ARCADE: u8 = 3;
const ACCESSIBLE_LATERAL_ACCEL_MPS2: f64 = 7.5;
const SPORT_LATERAL_ACCEL_MPS2: f64 = 10.0;
const ARCADE_LATERAL_ACCEL_MPS2: f64 = 12.0;

#[derive(Clone, Copy, Debug, Default)]
struct InputPipelineState {
    raw: DriverInput,
    normalized: DriverInput,
    policy: DriverInput,
    device_kind: u8,
    sample_sequence: u64,
    applied_step: u64,
}

#[derive(Clone, Copy, Debug, Default)]
struct KeyboardInputState {
    assist: KeyboardSteeringAssist,
    command: DriverInput,
    transitioning: bool,
    pipeline: InputPipelineState,
}

#[derive(Clone, Debug)]
struct SavedBrowserState {
    world: Snapshot,
    keyboard: KeyboardInputState,
    player_input_mode: u8,
    player_autopilot: bool,
    keyboard_assist_enabled: bool,
    experience_profile: u8,
    demo_vehicle_preset: u8,
}

fn keyboard() -> &'static Mutex<KeyboardInputState> {
    KEYBOARD.get_or_init(|| Mutex::new(KeyboardInputState::default()))
}
fn demo() -> &'static Mutex<PhysicsWorld> {
    DEMO.get_or_init(|| Mutex::new(selected_demo()))
}
fn selected_demo() -> PhysicsWorld {
    let preset = match DEMO_VEHICLE_PRESET.load(Ordering::Relaxed) {
        2 => VehiclePreset::ArcadeFun,
        3 => VehiclePreset::EngineeringReference,
        _ => VehiclePreset::RaceGameplay,
    };
    if preset == VehiclePreset::EngineeringReference {
        PhysicsWorld::engineering_lab()
    } else {
        PhysicsWorld::demo_with_preset(10, preset)
    }
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
fn validation_report() -> &'static Mutex<Option<crate::validation::ScenarioReport>> {
    VALIDATION_REPORT.get_or_init(|| Mutex::new(None))
}
fn yaw(q: Quat) -> f64 {
    (2.0 * (q.w * q.y + q.x * q.z)).atan2(1.0 - 2.0 * (q.y * q.y + q.x * q.x))
}

fn profile_lateral_accel_target(profile: u8) -> Option<f64> {
    match profile {
        PROFILE_ACCESSIBLE => Some(ACCESSIBLE_LATERAL_ACCEL_MPS2),
        PROFILE_SPORT => Some(SPORT_LATERAL_ACCEL_MPS2),
        PROFILE_ARCADE => Some(ARCADE_LATERAL_ACCEL_MPS2),
        _ => None,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_reset() {
    with_world(|w| *w = selected_demo());
    PLAYER_AUTOPILOT.store(false, Ordering::Relaxed);
    PLAYER_INPUT_MODE.store(0, Ordering::Relaxed);
    KEYBOARD_ASSIST_ENABLED.store(true, Ordering::Relaxed);
    EXPERIENCE_PROFILE.store(PROFILE_SPORT, Ordering::Relaxed);
    let mut state = KeyboardInputState::default();
    state.pipeline.device_kind = 1;
    *keyboard().lock().unwrap_or_else(|error| error.into_inner()) = state;
}

/// Selects the authored physical definition used by subsequent resets and
/// immediately starts that demo. 1 = Race Gameplay, 2 = Arcade Fun,
/// 3 = Engineering Reference (single-vehicle proving ground).
#[unsafe(no_mangle)]
pub extern "C" fn physics_select_demo_vehicle_preset(preset: u32) {
    let preset = if matches!(preset, 2 | 3) { preset as u8 } else { 1 };
    DEMO_VEHICLE_PRESET.store(preset, Ordering::Relaxed);
    physics_reset();
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_demo_vehicle_preset() -> u32 {
    u32::from(DEMO_VEHICLE_PRESET.load(Ordering::Relaxed))
}

/// Restores the collision-free engineering proving ground for free drive.
#[unsafe(no_mangle)]
pub extern "C" fn physics_lab_reset_free_drive() {
    DEMO_VEHICLE_PRESET.store(3, Ordering::Relaxed);
    physics_reset();
    EXPERIENCE_PROFILE.store(PROFILE_SIMULATION, Ordering::Relaxed);
    KEYBOARD_ASSIST_ENABLED.store(false, Ordering::Relaxed);
}

fn set_analog_input(device_kind: u8, raw: DriverInput, normalized: DriverInput) {
    PLAYER_INPUT_MODE.store(0, Ordering::Relaxed);
    let normalized = normalized.sanitized();
    let applied_step = with_world(|world| {
        let step = world.step_index;
        let _ = world.set_input(0, normalized);
        step
    });
    let mut state = keyboard().lock().unwrap_or_else(|error| error.into_inner());
    state.assist.reset();
    state.command = normalized;
    state.transitioning = false;
    state.pipeline.raw = raw;
    state.pipeline.normalized = normalized;
    state.pipeline.policy = normalized;
    state.pipeline.device_kind = device_kind;
    state.pipeline.sample_sequence = state.pipeline.sample_sequence.wrapping_add(1);
    state.pipeline.applied_step = applied_step;
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_set_input(steering: f64, throttle: f64, brake: f64, clutch: f64, handbrake: f64, gear: i32) {
    let input = DriverInput { steering, throttle, brake, clutch, handbrake, gear_request: gear as i8 };
    set_analog_input(0, input, input);
}

/// Browser/device-adapter input. Raw axes are retained for diagnostics while
/// only normalized, sanitized commands reach the controller/plant.
#[unsafe(no_mangle)]
pub extern "C" fn physics_set_device_input(
    device_kind: u32,
    raw_steering: f64,
    raw_throttle: f64,
    raw_brake: f64,
    raw_clutch: f64,
    raw_handbrake: f64,
    steering: f64,
    throttle: f64,
    brake: f64,
    clutch: f64,
    handbrake: f64,
    gear: i32,
) {
    let raw = DriverInput {
        steering: raw_steering,
        throttle: raw_throttle,
        brake: raw_brake,
        clutch: raw_clutch,
        handbrake: raw_handbrake,
        gear_request: gear as i8,
    };
    let normalized = DriverInput { steering, throttle, brake, clutch, handbrake, gear_request: gear as i8 }.sanitized();
    let device_kind = device_kind.clamp(2, 3) as u8;
    PLAYER_INPUT_MODE.store(2, Ordering::Relaxed);
    with_world(|world| {
        let current = world.vehicles.first().map_or(0.0, |vehicle| vehicle.input.steering);
        let mut state = keyboard().lock().unwrap_or_else(|error| error.into_inner());
        if state.pipeline.device_kind != device_kind {
            state.assist.set_output(current);
            state.transitioning = true;
        }
        state.command = normalized;
        state.pipeline.raw = raw;
        state.pipeline.normalized = normalized;
        state.pipeline.device_kind = device_kind;
        state.pipeline.sample_sequence = state.pipeline.sample_sequence.wrapping_add(1);
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
    with_world(|world| {
        let current = world.vehicles.first().map_or(0.0, |vehicle| vehicle.input.steering);
        let mut state = keyboard().lock().unwrap_or_else(|error| error.into_inner());
        if state.pipeline.device_kind != 1 {
            state.assist.set_output(current);
            state.transitioning = true;
        }
        state.command =
            DriverInput { steering: direction, throttle, brake, clutch, handbrake, gear_request: gear as i8 }
                .sanitized();
        state.pipeline.raw = state.command;
        state.pipeline.normalized = state.command;
        state.pipeline.device_kind = 1;
        state.pipeline.sample_sequence = state.pipeline.sample_sequence.wrapping_add(1);
    });
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_step(steps: u32) {
    with_world(|w| {
        for _ in 0..steps {
            let input_mode = PLAYER_INPUT_MODE.load(Ordering::Relaxed);
            if !PLAYER_AUTOPILOT.load(Ordering::Relaxed) && matches!(input_mode, 1 | 2) {
                let speed = w.vehicles.first().map_or(0.0, |vehicle| vehicle.telemetry.speed_mps);
                let mut keyboard = keyboard().lock().unwrap_or_else(|error| error.into_inner());
                let command = keyboard.command;
                let profile = EXPERIENCE_PROFILE.load(Ordering::Relaxed);
                let device_kind = keyboard.pipeline.device_kind;
                let keyboard_assist = input_mode == 1 && KEYBOARD_ASSIST_ENABLED.load(Ordering::Relaxed);
                let gamepad_assist = input_mode == 2 && device_kind == 2 && profile != PROFILE_SIMULATION;
                let assist_enabled = keyboard_assist || gamepad_assist;
                let target_lateral_accel = profile_lateral_accel_target(profile).unwrap_or(SPORT_LATERAL_ACCEL_MPS2);
                let steering = if assist_enabled || keyboard.transitioning {
                    let policy_speed = if assist_enabled { speed } else { 0.0 };
                    let output = keyboard.assist.update_for_target(
                        command.steering,
                        policy_speed,
                        w.config.fixed_dt_s,
                        target_lateral_accel,
                    );
                    let target = command.steering
                        * crate::controls::speed_sensitive_steering_limit_for_target(
                            policy_speed,
                            target_lateral_accel,
                        );
                    if keyboard.transitioning && (output - target).abs() <= 1.0e-9 {
                        keyboard.transitioning = false;
                    }
                    output
                } else {
                    command.steering
                };
                let policy = DriverInput { steering, ..command };
                keyboard.pipeline.policy = policy;
                keyboard.pipeline.applied_step = w.step_index;
                let _ = w.set_input_unrecorded(0, policy);
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
pub extern "C" fn physics_set_experience_profile(profile: u32) {
    let profile = (profile as u8).clamp(PROFILE_ACCESSIBLE, PROFILE_ARCADE);
    let changed = EXPERIENCE_PROFILE.swap(profile, Ordering::Relaxed) != profile;
    KEYBOARD_ASSIST_ENABLED.store(profile != PROFILE_SIMULATION, Ordering::Relaxed);
    let input_mode = PLAYER_INPUT_MODE.load(Ordering::Relaxed);
    with_world(|world| {
        if let Some(vehicle) = world.vehicles.first_mut() {
            // Profiles configure controllers only. ABS/TC retain the authored
            // vehicle configuration; Accessible additionally enables ESC.
            vehicle.driver_aids.stability_control_enabled = profile == PROFILE_ACCESSIBLE;
        }
        if changed {
            let current = world.vehicles.first().map_or(0.0, |vehicle| vehicle.input.steering);
            let mut state = keyboard().lock().unwrap_or_else(|error| error.into_inner());
            let active_policy_changes = input_mode == 1 || (input_mode == 2 && state.pipeline.device_kind == 2);
            if active_policy_changes {
                state.assist.set_output(current);
                state.transitioning = true;
            }
        }
    });
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_experience_profile() -> u32 {
    u32::from(EXPERIENCE_PROFILE.load(Ordering::Relaxed))
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_policy_lateral_accel_target() -> f64 {
    profile_lateral_accel_target(EXPERIENCE_PROFILE.load(Ordering::Relaxed)).unwrap_or(0.0)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_gamepad_assist() -> u32 {
    u32::from(EXPERIENCE_PROFILE.load(Ordering::Relaxed) != PROFILE_SIMULATION)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_set_keyboard_assist(enabled: u32) {
    let enabled = enabled != 0;
    if KEYBOARD_ASSIST_ENABLED.swap(enabled, Ordering::Relaxed) != enabled
        && PLAYER_INPUT_MODE.load(Ordering::Relaxed) == 1
    {
        with_world(|world| {
            let current = world.vehicles.first().map_or(0.0, |vehicle| vehicle.input.steering);
            let mut state = keyboard().lock().unwrap_or_else(|error| error.into_inner());
            state.assist.set_output(current);
            state.transitioning = true;
        });
    }
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_keyboard_assist() -> u32 {
    u32::from(KEYBOARD_ASSIST_ENABLED.load(Ordering::Relaxed))
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

fn input_stage(stage: u32) -> DriverInput {
    match stage {
        0 => keyboard().lock().unwrap_or_else(|error| error.into_inner()).pipeline.raw,
        1 => keyboard().lock().unwrap_or_else(|error| error.into_inner()).pipeline.normalized,
        2 => keyboard().lock().unwrap_or_else(|error| error.into_inner()).pipeline.policy,
        3 => with_world(|world| world.vehicles.first().map_or(DriverInput::default(), |vehicle| vehicle.input)),
        4 => with_world(|world| {
            world.vehicles.first().map_or(DriverInput::default(), |vehicle| DriverInput {
                steering: vehicle.control.steering,
                throttle: vehicle.control.throttle,
                brake: vehicle.control.brake_per_wheel.into_iter().sum::<f64>() * 0.25,
                clutch: vehicle.control.clutch,
                handbrake: 0.0,
                gear_request: vehicle.control.gear_request,
            })
        }),
        _ => DriverInput::default(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_step_index() -> f64 {
    with_world(|world| world.step_index as f64)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_input_sample_sequence() -> f64 {
    keyboard().lock().unwrap_or_else(|error| error.into_inner()).pipeline.sample_sequence as f64
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_input_applied_step() -> f64 {
    keyboard().lock().unwrap_or_else(|error| error.into_inner()).pipeline.applied_step as f64
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_input_device() -> u32 {
    u32::from(keyboard().lock().unwrap_or_else(|error| error.into_inner()).pipeline.device_kind)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_input_transitioning() -> u32 {
    u32::from(keyboard().lock().unwrap_or_else(|error| error.into_inner()).transitioning)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_input_stage_steering(stage: u32) -> f64 {
    input_stage(stage).steering
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_input_stage_throttle(stage: u32) -> f64 {
    input_stage(stage).throttle
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_input_stage_brake(stage: u32) -> f64 {
    input_stage(stage).brake
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_input_stage_clutch(stage: u32) -> f64 {
    input_stage(stage).clutch
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_input_stage_handbrake(stage: u32) -> f64 {
    input_stage(stage).handbrake
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_input_stage_gear(stage: u32) -> f64 {
    f64::from(input_stage(stage).gear_request)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_input_aid_brake(wheel: u32) -> f64 {
    with_world(|world| {
        world
            .vehicles
            .first()
            .and_then(|vehicle| vehicle.control.brake_per_wheel.get(wheel as usize))
            .copied()
            .unwrap_or(f64::NAN)
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_input_abs_active(wheel: u32) -> u32 {
    with_world(|world| {
        u32::from(
            world
                .vehicles
                .first()
                .and_then(|vehicle| vehicle.control.abs_active.get(wheel as usize))
                .copied()
                .unwrap_or(false),
        )
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_input_tc_active() -> u32 {
    with_world(|world| u32::from(world.vehicles.first().is_some_and(|vehicle| vehicle.control.tc_active)))
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_input_esc_active() -> u32 {
    with_world(|world| u32::from(world.vehicles.first().is_some_and(|vehicle| vehicle.control.esc_active)))
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
pub extern "C" fn physics_yaw_rate(i: u32) -> f64 {
    read_vehicle(i, |v| v.telemetry.yaw_rate_rad_s)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_lateral_acceleration(i: u32) -> f64 {
    read_vehicle(i, |v| v.state.orientation.conjugate().rotate(v.telemetry.acceleration_mps2).x)
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_body_slip_angle(i: u32) -> f64 {
    read_vehicle(i, |v| {
        let local = v.state.orientation.conjugate().rotate(v.state.linear_velocity_mps);
        local.x.atan2((-local.z).abs().max(1.0e-9))
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn physics_road_water_depth_mm(i: u32) -> f64 {
    with_world(|world| {
        world
            .vehicles
            .get(i as usize)
            .map_or(f64::NAN, |vehicle| world.road.sample(vehicle.state.position_m).water_depth_m * 1000.0)
    })
}
/// Runs one scenario from the native validation catalog. The report is kept
/// outside the live demo world, so inspecting a maneuver cannot perturb free
/// drive or alter its snapshot/replay history.
#[unsafe(no_mangle)]
pub extern "C" fn physics_validation_run(scenario_index: u32) -> u32 {
    let Some(definition) = crate::validation::SCENARIOS.get(scenario_index as usize) else {
        return 0;
    };
    let report = crate::validation::run_scenario(definition);
    let passed = report.passed();
    *validation_report().lock().unwrap_or_else(|error| error.into_inner()) = Some(report);
    u32::from(passed)
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_validation_midpoint_replay(scenario_index: u32) -> u32 {
    crate::validation::SCENARIOS
        .get(scenario_index as usize)
        .map_or(0, |definition| u32::from(crate::validation::verify_midpoint_snapshot_replay(definition)))
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_validation_sample_count() -> u32 {
    validation_report()
        .lock()
        .unwrap_or_else(|error| error.into_inner())
        .as_ref()
        .map_or(0, |report| report.samples.len() as u32)
}

/// sample fields: 0=time, 1=speed, 2=yaw rate, 3=sideslip,
/// 4..=6=world acceleration xyz, 7..=10=wheel slip,
/// 11..=14=wheel slip angle, 15..=18=wheel normal load.
#[unsafe(no_mangle)]
pub extern "C" fn physics_validation_sample(sample: u32, field: u32) -> f64 {
    validation_report()
        .lock()
        .unwrap_or_else(|error| error.into_inner())
        .as_ref()
        .and_then(|report| report.samples.get(sample as usize))
        .map_or(f64::NAN, |sample| match field {
            0 => sample.time_s,
            1 => sample.speed_mps,
            2 => sample.yaw_rate_rad_s,
            3 => sample.sideslip_rad,
            4..=6 => sample.acceleration_mps2[(field - 4) as usize],
            7..=10 => sample.wheel_slip[(field - 7) as usize],
            11..=14 => sample.wheel_slip_angle_rad[(field - 11) as usize],
            15..=18 => sample.wheel_load_n[(field - 15) as usize],
            _ => f64::NAN,
        })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_validation_check_count() -> u32 {
    validation_report()
        .lock()
        .unwrap_or_else(|error| error.into_inner())
        .as_ref()
        .map_or(0, |report| report.checks.len() as u32)
}

/// check fields: 0=metric enum, 1=value, 2=min, 3=max, 4=passed.
#[unsafe(no_mangle)]
pub extern "C" fn physics_validation_check(check: u32, field: u32) -> f64 {
    validation_report()
        .lock()
        .unwrap_or_else(|error| error.into_inner())
        .as_ref()
        .and_then(|report| report.checks.get(check as usize))
        .map_or(f64::NAN, |check| match field {
            0 => check.metric as u8 as f64,
            1 => check.value.unwrap_or(f64::NAN),
            2 => check.min,
            3 => check.max,
            4 => f64::from(check.passed),
            _ => f64::NAN,
        })
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_validation_fingerprint_low() -> u32 {
    validation_report()
        .lock()
        .unwrap_or_else(|error| error.into_inner())
        .as_ref()
        .map_or(0, |report| report.fingerprint as u32)
}

#[unsafe(no_mangle)]
pub extern "C" fn physics_validation_fingerprint_high() -> u32 {
    validation_report()
        .lock()
        .unwrap_or_else(|error| error.into_inner())
        .as_ref()
        .map_or(0, |report| (report.fingerprint >> 32) as u32)
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
        keyboard_assist_enabled: KEYBOARD_ASSIST_ENABLED.load(Ordering::Relaxed),
        experience_profile: EXPERIENCE_PROFILE.load(Ordering::Relaxed),
        demo_vehicle_preset: DEMO_VEHICLE_PRESET.load(Ordering::Relaxed),
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
        KEYBOARD_ASSIST_ENABLED.store(state.keyboard_assist_enabled, Ordering::Relaxed);
        EXPERIENCE_PROFILE.store(state.experience_profile, Ordering::Relaxed);
        DEMO_VEHICLE_PRESET.store(state.demo_vehicle_preset, Ordering::Relaxed);
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
