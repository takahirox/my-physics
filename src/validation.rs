//! Deterministic, headless maneuver validation.
//!
//! The bounds in this module are regression and physical-plausibility checks.
//! They are not evidence of correlation with a measured real vehicle.

use crate::{DriverInput, PhysicsWorld, Quat, SimulationConfig, Vec3, VehicleDefinition};
use std::fmt::Write as _;

pub const VALIDATION_DISCLAIMER: &str =
    "Physical-plausibility/regression evidence only; not correlation with a measured real vehicle.";
pub const VALIDATION_VEHICLE_PRESET: &str = "engineering_reference";
pub const VEHICLE_DEFINITION_REVISION: &str = "vehicle-definition-v0.1";
pub const VEHICLE_PROVENANCE_REVISION: &str = "provenance-untracked-v0.1";

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum InputProgram {
    Coast,
    FullThrottle,
    FullBrake,
    RampAndHoldSteer { steer_rad: f64, ramp_s: f64 },
    StepSteer { steer_rad: f64, step_at_s: f64 },
    SineSteer { amplitude_rad: f64, frequency_hz: f64 },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScenarioDefinition {
    pub name: &'static str,
    pub description: &'static str,
    pub initial_speed_mps: f64,
    pub duration_s: f64,
    pub sample_period_s: f64,
    pub input: InputProgram,
    pub bounds: &'static [MetricBound],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MetricBound {
    pub metric: Metric,
    pub min: f64,
    pub max: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Metric {
    FinalSpeedMps,
    DistanceM,
    TargetTimeS,
    PeakYawRateRadS,
    FinalYawRateAbsRadS,
    PeakSideslipAbsRad,
    PeakWheelSlipAbs,
    MinimumWheelLoadN,
    YawSignChanges,
}

impl Metric {
    pub const fn name(self) -> &'static str {
        match self {
            Self::FinalSpeedMps => "final_speed_mps",
            Self::DistanceM => "distance_m",
            Self::TargetTimeS => "target_time_s",
            Self::PeakYawRateRadS => "peak_yaw_rate_rad_s",
            Self::FinalYawRateAbsRadS => "final_yaw_rate_abs_rad_s",
            Self::PeakSideslipAbsRad => "peak_sideslip_abs_rad",
            Self::PeakWheelSlipAbs => "peak_wheel_slip_abs",
            Self::MinimumWheelLoadN => "minimum_wheel_load_n",
            Self::YawSignChanges => "yaw_sign_changes",
        }
    }
}

const COAST_BOUNDS: &[MetricBound] = &[
    MetricBound { metric: Metric::FinalSpeedMps, min: 20.0, max: 27.7 },
    MetricBound { metric: Metric::PeakSideslipAbsRad, min: 0.0, max: 0.02 },
    MetricBound { metric: Metric::MinimumWheelLoadN, min: 1_000.0, max: 10_000.0 },
];
const ACCEL_BOUNDS: &[MetricBound] = &[
    MetricBound { metric: Metric::TargetTimeS, min: 2.0, max: 15.0 },
    MetricBound { metric: Metric::PeakSideslipAbsRad, min: 0.0, max: 0.03 },
];
const BRAKE_BOUNDS: &[MetricBound] = &[
    MetricBound { metric: Metric::TargetTimeS, min: 1.5, max: 4.0 },
    MetricBound { metric: Metric::DistanceM, min: 25.0, max: 50.0 },
    // Near-zero-speed slip ratio is intentionally regularized by the tire
    // model and can exceed one after the stop; braking distance/time are the
    // useful acceptance quantities here.
    MetricBound { metric: Metric::PeakWheelSlipAbs, min: 0.05, max: 1.60 },
];
const STEADY_BOUNDS: &[MetricBound] = &[
    MetricBound { metric: Metric::FinalYawRateAbsRadS, min: 0.04, max: 0.30 },
    MetricBound { metric: Metric::PeakSideslipAbsRad, min: 0.0, max: 0.12 },
    MetricBound { metric: Metric::PeakWheelSlipAbs, min: 0.0, max: 0.30 },
];
const STEP_BOUNDS: &[MetricBound] = &[
    MetricBound { metric: Metric::PeakYawRateRadS, min: 0.08, max: 0.60 },
    MetricBound { metric: Metric::PeakSideslipAbsRad, min: 0.0, max: 0.15 },
    MetricBound { metric: Metric::PeakWheelSlipAbs, min: 0.0, max: 0.35 },
];
const SLALOM_BOUNDS: &[MetricBound] = &[
    MetricBound { metric: Metric::PeakYawRateRadS, min: 0.08, max: 0.80 },
    MetricBound { metric: Metric::PeakSideslipAbsRad, min: 0.0, max: 0.20 },
    MetricBound { metric: Metric::YawSignChanges, min: 5.0, max: 12.0 },
];

pub const SCENARIOS: &[ScenarioDefinition] = &[
    ScenarioDefinition {
        name: "coast_down",
        description: "Neutral coast-down from 100 km/h on dry level pavement",
        initial_speed_mps: 100.0 / 3.6,
        duration_s: 12.0,
        sample_period_s: 0.02,
        input: InputProgram::Coast,
        bounds: COAST_BOUNDS,
    },
    ScenarioDefinition {
        name: "zero_to_100",
        description: "Standing full-throttle acceleration to 100 km/h",
        initial_speed_mps: 0.0,
        duration_s: 15.0,
        sample_period_s: 0.02,
        input: InputProgram::FullThrottle,
        bounds: ACCEL_BOUNDS,
    },
    ScenarioDefinition {
        name: "hundred_to_zero",
        description: "ABS full braking from 100 km/h",
        initial_speed_mps: 100.0 / 3.6,
        duration_s: 5.0,
        sample_period_s: 0.01,
        input: InputProgram::FullBrake,
        bounds: BRAKE_BOUNDS,
    },
    ScenarioDefinition {
        name: "steady_steer",
        description: "Ramp to a low-g steady steer at 72 km/h",
        initial_speed_mps: 20.0,
        duration_s: 6.0,
        sample_period_s: 0.01,
        input: InputProgram::RampAndHoldSteer { steer_rad: 0.5_f64.to_radians(), ramp_s: 2.0 },
        bounds: STEADY_BOUNDS,
    },
    ScenarioDefinition {
        name: "step_steer",
        description: "One-degree step steer at 90 km/h",
        initial_speed_mps: 25.0,
        duration_s: 4.0,
        sample_period_s: 0.005,
        input: InputProgram::StepSteer { steer_rad: 1.0_f64.to_radians(), step_at_s: 0.5 },
        bounds: STEP_BOUNDS,
    },
    ScenarioDefinition {
        name: "slalom",
        description: "Deterministic 0.5 Hz sinusoidal steering at 65 km/h",
        initial_speed_mps: 18.0,
        duration_s: 8.0,
        sample_period_s: 0.01,
        input: InputProgram::SineSteer { amplitude_rad: 1.5_f64.to_radians(), frequency_hz: 0.5 },
        bounds: SLALOM_BOUNDS,
    },
];

#[derive(Clone, Debug, PartialEq)]
pub struct TimeSeriesSample {
    pub time_s: f64,
    pub speed_mps: f64,
    pub yaw_rate_rad_s: f64,
    pub sideslip_rad: f64,
    pub acceleration_mps2: [f64; 3],
    pub wheel_slip: [f64; 4],
    pub wheel_slip_angle_rad: [f64; 4],
    pub wheel_load_n: [f64; 4],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScenarioSummary {
    pub initial_speed_mps: f64,
    pub final_speed_mps: f64,
    pub distance_m: f64,
    pub target_time_s: Option<f64>,
    pub peak_yaw_rate_rad_s: f64,
    pub final_yaw_rate_abs_rad_s: f64,
    pub peak_sideslip_abs_rad: f64,
    pub peak_wheel_slip_abs: f64,
    pub minimum_wheel_load_n: f64,
    pub yaw_sign_changes: u32,
}

impl ScenarioSummary {
    pub fn metric(self, metric: Metric) -> Option<f64> {
        match metric {
            Metric::FinalSpeedMps => Some(self.final_speed_mps),
            Metric::DistanceM => Some(self.distance_m),
            Metric::TargetTimeS => self.target_time_s,
            Metric::PeakYawRateRadS => Some(self.peak_yaw_rate_rad_s),
            Metric::FinalYawRateAbsRadS => Some(self.final_yaw_rate_abs_rad_s),
            Metric::PeakSideslipAbsRad => Some(self.peak_sideslip_abs_rad),
            Metric::PeakWheelSlipAbs => Some(self.peak_wheel_slip_abs),
            Metric::MinimumWheelLoadN => Some(self.minimum_wheel_load_n),
            Metric::YawSignChanges => Some(f64::from(self.yaw_sign_changes)),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct BoundResult {
    pub metric: Metric,
    pub value: Option<f64>,
    pub min: f64,
    pub max: f64,
    pub passed: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ScenarioReport {
    pub definition: ScenarioDefinition,
    pub fixed_dt_s: f64,
    pub fingerprint: u64,
    pub summary: ScenarioSummary,
    pub checks: Vec<BoundResult>,
    pub samples: Vec<TimeSeriesSample>,
}

impl ScenarioReport {
    pub fn passed(&self) -> bool {
        self.checks.iter().all(|check| check.passed)
    }

    pub fn summary_json(&self) -> String {
        let mut output = String::new();
        write!(
            output,
            "{{\"schema_version\":1,\"disclaimer\":\"{}\",\"scenario\":\"{}\",\"description\":\"{}\",\"vehicle_preset\":\"{}\",\"vehicle_definition_revision\":\"{}\",\"vehicle_provenance_revision\":\"{}\",\"fixed_dt_s\":{:.9},\"fingerprint\":\"{:016x}\",\"passed\":{},\"summary\":{{",
            VALIDATION_DISCLAIMER,
            self.definition.name,
            self.definition.description,
            VALIDATION_VEHICLE_PRESET,
            VEHICLE_DEFINITION_REVISION,
            VEHICLE_PROVENANCE_REVISION,
            self.fixed_dt_s,
            self.fingerprint,
            self.passed()
        )
        .unwrap();
        let s = self.summary;
        write!(
            output,
            "\"initial_speed_mps\":{:.9},\"final_speed_mps\":{:.9},\"distance_m\":{:.9},\"target_time_s\":",
            s.initial_speed_mps, s.final_speed_mps, s.distance_m
        )
        .unwrap();
        match s.target_time_s {
            Some(value) => write!(output, "{value:.9}").unwrap(),
            None => output.push_str("null"),
        }
        write!(output, ",\"peak_yaw_rate_rad_s\":{:.9},\"final_yaw_rate_abs_rad_s\":{:.9},\"peak_sideslip_abs_rad\":{:.9},\"peak_wheel_slip_abs\":{:.9},\"minimum_wheel_load_n\":{:.9},\"yaw_sign_changes\":{} }},\"checks\":[", s.peak_yaw_rate_rad_s, s.final_yaw_rate_abs_rad_s, s.peak_sideslip_abs_rad, s.peak_wheel_slip_abs, s.minimum_wheel_load_n, s.yaw_sign_changes).unwrap();
        for (index, check) in self.checks.iter().enumerate() {
            if index != 0 {
                output.push(',');
            }
            write!(output, "{{\"metric\":\"{}\",\"value\":", check.metric.name()).unwrap();
            match check.value {
                Some(value) => write!(output, "{value:.9}").unwrap(),
                None => output.push_str("null"),
            }
            write!(output, ",\"min\":{:.9},\"max\":{:.9},\"passed\":{}}}", check.min, check.max, check.passed).unwrap();
        }
        output.push_str("]}");
        output
    }

    pub fn summary_csv(&self) -> String {
        format!(
            "scenario,vehicle_preset,vehicle_definition_revision,vehicle_provenance_revision,passed,fingerprint,initial_speed_mps,final_speed_mps,distance_m,target_time_s,peak_yaw_rate_rad_s,final_yaw_rate_abs_rad_s,peak_sideslip_abs_rad,peak_wheel_slip_abs,minimum_wheel_load_n,yaw_sign_changes\n{},{},{},{},{},{:016x},{:.9},{:.9},{:.9},{},{:.9},{:.9},{:.9},{:.9},{:.9},{}\n",
            self.definition.name,
            VALIDATION_VEHICLE_PRESET,
            VEHICLE_DEFINITION_REVISION,
            VEHICLE_PROVENANCE_REVISION,
            self.passed(),
            self.fingerprint,
            self.summary.initial_speed_mps,
            self.summary.final_speed_mps,
            self.summary.distance_m,
            self.summary.target_time_s.map_or_else(String::new, |value| format!("{value:.9}")),
            self.summary.peak_yaw_rate_rad_s,
            self.summary.final_yaw_rate_abs_rad_s,
            self.summary.peak_sideslip_abs_rad,
            self.summary.peak_wheel_slip_abs,
            self.summary.minimum_wheel_load_n,
            self.summary.yaw_sign_changes,
        )
    }

    pub fn timeseries_csv(&self) -> String {
        let mut output = String::from(
            "time_s,speed_mps,yaw_rate_rad_s,sideslip_rad,accel_x_mps2,accel_y_mps2,accel_z_mps2,fl_slip,fr_slip,rl_slip,rr_slip,fl_slip_angle_rad,fr_slip_angle_rad,rl_slip_angle_rad,rr_slip_angle_rad,fl_load_n,fr_load_n,rl_load_n,rr_load_n\n",
        );
        for s in &self.samples {
            writeln!(output, "{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9}", s.time_s, s.speed_mps, s.yaw_rate_rad_s, s.sideslip_rad, s.acceleration_mps2[0], s.acceleration_mps2[1], s.acceleration_mps2[2], s.wheel_slip[0], s.wheel_slip[1], s.wheel_slip[2], s.wheel_slip[3], s.wheel_slip_angle_rad[0], s.wheel_slip_angle_rad[1], s.wheel_slip_angle_rad[2], s.wheel_slip_angle_rad[3], s.wheel_load_n[0], s.wheel_load_n[1], s.wheel_load_n[2], s.wheel_load_n[3]).unwrap();
        }
        output
    }

    pub fn timeseries_json(&self) -> String {
        let mut output = format!(
            "{{\"schema_version\":1,\"scenario\":\"{}\",\"vehicle_preset\":\"{}\",\"vehicle_definition_revision\":\"{}\",\"vehicle_provenance_revision\":\"{}\",\"samples\":[",
            self.definition.name, VALIDATION_VEHICLE_PRESET, VEHICLE_DEFINITION_REVISION, VEHICLE_PROVENANCE_REVISION,
        );
        for (index, s) in self.samples.iter().enumerate() {
            if index != 0 {
                output.push(',');
            }
            write!(output, "{{\"time_s\":{:.9},\"speed_mps\":{:.9},\"yaw_rate_rad_s\":{:.9},\"sideslip_rad\":{:.9},\"acceleration_mps2\":[{:.9},{:.9},{:.9}],\"wheel_slip\":[{:.9},{:.9},{:.9},{:.9}],\"wheel_slip_angle_rad\":[{:.9},{:.9},{:.9},{:.9}],\"wheel_load_n\":[{:.9},{:.9},{:.9},{:.9}]}}", s.time_s, s.speed_mps, s.yaw_rate_rad_s, s.sideslip_rad, s.acceleration_mps2[0], s.acceleration_mps2[1], s.acceleration_mps2[2], s.wheel_slip[0], s.wheel_slip[1], s.wheel_slip[2], s.wheel_slip[3], s.wheel_slip_angle_rad[0], s.wheel_slip_angle_rad[1], s.wheel_slip_angle_rad[2], s.wheel_slip_angle_rad[3], s.wheel_load_n[0], s.wheel_load_n[1], s.wheel_load_n[2], s.wheel_load_n[3]).unwrap();
        }
        output.push_str("]}");
        output
    }
}

pub fn scenario(name: &str) -> Option<&'static ScenarioDefinition> {
    SCENARIOS.iter().find(|definition| definition.name == name)
}

pub fn run_catalog() -> Vec<ScenarioReport> {
    SCENARIOS.iter().map(run_scenario).collect()
}

pub fn run_scenario(definition: &ScenarioDefinition) -> ScenarioReport {
    run_scenario_with_dt(definition, SimulationConfig::default().fixed_dt_s)
}

pub fn run_scenario_with_dt(definition: &ScenarioDefinition, fixed_dt_s: f64) -> ScenarioReport {
    assert!(fixed_dt_s.is_finite() && fixed_dt_s > 0.0 && fixed_dt_s <= 0.001);
    let mut vehicle_definition = VehicleDefinition::default();
    let is_acceleration = matches!(definition.input, InputProgram::FullThrottle);
    if !is_acceleration {
        vehicle_definition.transmission.automatic = false;
    }
    let mut world =
        PhysicsWorld::new(SimulationConfig { fixed_dt_s, automatic_lod: false, ..SimulationConfig::default() });
    let index = world.add_vehicle(vehicle_definition);
    world.vehicles[index].driver_aids.traction_control_enabled = is_acceleration;
    world.vehicles[index].driver_aids.stability_control_enabled = false;
    world.step_fixed((2.0 / fixed_dt_s).round() as u32).expect("settling fixture stays finite");
    prepare_speed(&mut world, definition.initial_speed_mps, is_acceleration);

    let start = world.vehicles[0].state.position_m;
    let steps = (definition.duration_s / fixed_dt_s).round() as u32;
    let sample_every = (definition.sample_period_s / fixed_dt_s).round().max(1.0) as u32;
    let max_steer = world.vehicles[0].definition.wheels[0].max_steer_rad;
    let mut samples = Vec::with_capacity((steps / sample_every + 2) as usize);
    let mut target_time_s = None;
    let mut target_position_m = None;
    for step in 0..=steps {
        let time_s = f64::from(step) * fixed_dt_s;
        if step % sample_every == 0 || step == steps {
            samples.push(sample(&world, time_s));
        }
        let speed = world.vehicles[0].telemetry.speed_mps;
        if target_time_s.is_none() {
            if is_acceleration && speed >= 100.0 / 3.6 {
                target_time_s = Some(time_s);
            } else if matches!(definition.input, InputProgram::FullBrake) && speed <= 2.0 {
                target_time_s = Some(time_s);
                target_position_m = Some(world.vehicles[0].state.position_m);
            }
        }
        if step == steps {
            break;
        }
        let input = input_at(definition.input, time_s, max_steer);
        world.set_input_unrecorded(0, input).expect("fixture vehicle exists");
        world.step_fixed(1).expect("validation scenario stays finite");
    }
    // Braking distance ends at the declared 2 m/s stop threshold; later
    // low-speed slip regularization or rollback cannot distort the result.
    let measurement_finish = target_position_m.unwrap_or(world.vehicles[0].state.position_m);
    let summary = summarize(&samples, start, measurement_finish, target_time_s);
    let checks = definition
        .bounds
        .iter()
        .map(|bound| {
            let value = summary.metric(bound.metric);
            BoundResult {
                metric: bound.metric,
                value,
                min: bound.min,
                max: bound.max,
                passed: value.is_some_and(|value| (bound.min..=bound.max).contains(&value)),
            }
        })
        .collect();
    ScenarioReport {
        definition: *definition,
        fixed_dt_s,
        fingerprint: world.state_fingerprint(),
        summary,
        checks,
        samples,
    }
}

fn prepare_speed(world: &mut PhysicsWorld, speed_mps: f64, acceleration_run: bool) {
    let vehicle = &mut world.vehicles[0];
    vehicle.state.position_m = Vec3::new(0.0, 0.55, 0.0);
    vehicle.state.orientation = Quat::IDENTITY;
    vehicle.previous_position_m = vehicle.state.position_m;
    vehicle.previous_orientation = vehicle.state.orientation;
    vehicle.state.linear_velocity_mps = Vec3::new(0.0, 0.0, -speed_mps);
    vehicle.state.angular_velocity_rad_s = Vec3::ZERO;
    vehicle.state.powertrain.gear = if acceleration_run { 1 } else { 0 };
    for (wheel, wheel_definition) in vehicle.state.wheels.iter_mut().zip(vehicle.definition.wheels.iter()) {
        wheel.angular_velocity_rad_s = speed_mps / wheel_definition.radius_m;
    }
    vehicle.update_telemetry(Vec3::ZERO);
}

fn input_at(program: InputProgram, time_s: f64, max_steer_rad: f64) -> DriverInput {
    let mut input = DriverInput::default();
    match program {
        InputProgram::Coast => {}
        InputProgram::FullThrottle => input.throttle = 1.0,
        InputProgram::FullBrake => input.brake = 1.0,
        InputProgram::RampAndHoldSteer { steer_rad, ramp_s } => {
            input.steering = steer_rad / max_steer_rad * (time_s / ramp_s).clamp(0.0, 1.0);
        }
        InputProgram::StepSteer { steer_rad, step_at_s } => {
            input.steering = if time_s >= step_at_s { steer_rad / max_steer_rad } else { 0.0 };
        }
        InputProgram::SineSteer { amplitude_rad, frequency_hz } => {
            input.steering = amplitude_rad / max_steer_rad * (core::f64::consts::TAU * frequency_hz * time_s).sin();
        }
    }
    input
}

fn sample(world: &PhysicsWorld, time_s: f64) -> TimeSeriesSample {
    let vehicle = &world.vehicles[0];
    let telemetry = &vehicle.telemetry;
    let forward = vehicle.state.orientation.rotate(Vec3::FORWARD);
    let right = vehicle.state.orientation.rotate(Vec3::X);
    let longitudinal = vehicle.state.linear_velocity_mps.dot(forward);
    let sideslip_rad = vehicle.state.linear_velocity_mps.dot(right).atan2(longitudinal.abs().max(0.1));
    TimeSeriesSample {
        time_s,
        speed_mps: telemetry.speed_mps,
        yaw_rate_rad_s: telemetry.yaw_rate_rad_s,
        sideslip_rad,
        acceleration_mps2: [
            telemetry.acceleration_mps2.x,
            telemetry.acceleration_mps2.y,
            telemetry.acceleration_mps2.z,
        ],
        wheel_slip: telemetry.wheel_slip,
        wheel_slip_angle_rad: vehicle.state.wheels.map(|wheel| wheel.slip_angle_rad),
        wheel_load_n: telemetry.normal_load_n,
    }
}

fn summarize(samples: &[TimeSeriesSample], start: Vec3, finish: Vec3, target_time_s: Option<f64>) -> ScenarioSummary {
    let mut peak_yaw_rate_rad_s: f64 = 0.0;
    let mut peak_sideslip_abs_rad: f64 = 0.0;
    let mut peak_wheel_slip_abs: f64 = 0.0;
    let mut minimum_wheel_load_n = f64::INFINITY;
    let mut yaw_sign_changes = 0;
    let mut previous_yaw_sign = 0.0_f64;
    for sample in samples {
        peak_yaw_rate_rad_s = peak_yaw_rate_rad_s.max(sample.yaw_rate_rad_s.abs());
        peak_sideslip_abs_rad = peak_sideslip_abs_rad.max(sample.sideslip_rad.abs());
        for slip in sample.wheel_slip {
            peak_wheel_slip_abs = peak_wheel_slip_abs.max(slip.abs());
        }
        for load in sample.wheel_load_n {
            minimum_wheel_load_n = minimum_wheel_load_n.min(load);
        }
        if sample.yaw_rate_rad_s.abs() >= 0.02 {
            let sign = sample.yaw_rate_rad_s.signum();
            if previous_yaw_sign != 0.0 && sign != previous_yaw_sign {
                yaw_sign_changes += 1;
            }
            previous_yaw_sign = sign;
        }
    }
    let first = samples.first().expect("scenario always samples its initial state");
    let last = samples.last().expect("scenario always samples its final state");
    ScenarioSummary {
        initial_speed_mps: first.speed_mps,
        final_speed_mps: last.speed_mps,
        distance_m: (finish - start).length(),
        target_time_s,
        peak_yaw_rate_rad_s,
        final_yaw_rate_abs_rad_s: last.yaw_rate_rad_s.abs(),
        peak_sideslip_abs_rad,
        peak_wheel_slip_abs,
        minimum_wheel_load_n,
        yaw_sign_changes,
    }
}
