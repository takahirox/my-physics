//! Replaceable tire model and a deterministic Magic-Formula-family reference.

use crate::math::{clamp01, smoothstep};
use crate::road::RoadCell;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TireFailure {
    Healthy,
    Punctured,
    Blowout,
    BeadUnseated,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TireState {
    pub temperature_k: f64,
    pub tread_temperature_k: f64,
    pub wear: f64,
    pub pressure_pa: f64,
    pub failure: TireFailure,
    pub puncture_area_m2: f64,
    pub carcass_damage: f64,
    pub contact_patch_m2: f64,
}

impl Default for TireState {
    fn default() -> Self {
        Self {
            temperature_k: 323.15,
            tread_temperature_k: 323.15,
            wear: 0.0,
            pressure_pa: 220_000.0,
            failure: TireFailure::Healthy,
            puncture_area_m2: 0.0,
            carcass_damage: 0.0,
            contact_patch_m2: 0.012,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TireInput {
    pub normal_load_n: f64,
    pub longitudinal_slip: f64,
    pub slip_angle_rad: f64,
    pub camber_rad: f64,
    pub speed_mps: f64,
    pub road: RoadCell,
    pub dt: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TireOutput {
    pub longitudinal_force_n: f64,
    pub lateral_force_n: f64,
    pub aligning_moment_nm: f64,
    pub rolling_resistance_n: f64,
    pub hydroplaning: f64,
    pub friction_coefficient: f64,
    pub slip_power_w: f64,
}

pub trait TireModel {
    fn evaluate(&self, state: &mut TireState, input: TireInput) -> TireOutput;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagicFormulaTire {
    pub nominal_load_n: f64,
    pub nominal_pressure_pa: f64,
    pub peak_mu: f64,
    pub longitudinal_stiffness: f64,
    pub lateral_stiffness: f64,
    pub optimum_temperature_k: f64,
}

impl Default for MagicFormulaTire {
    fn default() -> Self {
        Self {
            nominal_load_n: 3700.0,
            nominal_pressure_pa: 220_000.0,
            peak_mu: 1.28,
            longitudinal_stiffness: 11.5,
            lateral_stiffness: 8.5,
            optimum_temperature_k: 353.15,
        }
    }
}

impl TireModel for MagicFormulaTire {
    fn evaluate(&self, state: &mut TireState, i: TireInput) -> TireOutput {
        if i.normal_load_n <= 0.0 {
            self.evolve(state, i, 0.0);
            return TireOutput::default();
        }
        let load_ratio = (i.normal_load_n / self.nominal_load_n).max(0.05);
        let load_sensitivity = (1.0 - 0.10 * (load_ratio - 1.0)).clamp(0.68, 1.12);
        let temp_delta = (state.tread_temperature_k - self.optimum_temperature_k) / 55.0;
        let temp_grip = (-temp_delta * temp_delta).exp().mul_add(0.48, 0.52);
        let wear_grip = (1.0 - 0.48 * smoothstep(0.55, 1.0, state.wear)).max(0.35);
        let pressure_ratio = (state.pressure_pa / self.nominal_pressure_pa).clamp(0.02, 1.4);
        let pressure_grip = (1.0 - 0.30 * (pressure_ratio - 1.0).abs()).clamp(0.38, 1.0);
        let water = i.road.water_depth_m;
        let hydro_speed = (9.0 * (state.pressure_pa / 6894.76).sqrt()) * 0.44704;
        let hydro =
            clamp01((i.speed_mps - hydro_speed * 0.72) / (hydro_speed * 0.45).max(1.0)) * clamp01(water / 0.0025);
        let failure_grip = match state.failure {
            TireFailure::Healthy => 1.0,
            TireFailure::Punctured => 0.72,
            TireFailure::Blowout => 0.32,
            TireFailure::BeadUnseated => 0.16,
        };
        let mu = self.peak_mu
            * load_sensitivity
            * temp_grip
            * wear_grip
            * pressure_grip
            * i.road.grip_scale()
            * failure_grip
            * (1.0 - 0.82 * hydro);

        // Pacejka-like sine(arctan()) curves, normalized through a friction ellipse.
        let sx = (self.longitudinal_stiffness * i.longitudinal_slip).atan().sin();
        let sy = (self.lateral_stiffness * i.slip_angle_rad).atan().sin();
        let combined = (sx * sx + sy * sy).sqrt().max(1.0);
        let peak = mu * i.normal_load_n;
        let raw_fx = peak * sx / combined;
        let camber_thrust = (-i.camber_rad * 0.08 * i.normal_load_n).clamp(-peak * 0.18, peak * 0.18);
        let raw_fy = (-peak * sy / combined + camber_thrust).clamp(-peak, peak);
        let ellipse = (raw_fx * raw_fx + raw_fy * raw_fy).sqrt().max(peak);
        let fx = raw_fx * peak / ellipse;
        let fy = raw_fy * peak / ellipse;
        let trail = 0.055 * (1.0 - state.wear) * pressure_ratio.sqrt();
        let rr_coeff = 0.012 + 0.025 * (1.0 - pressure_ratio).max(0.0) + 0.08 * state.carcass_damage;
        let rr = rr_coeff * i.normal_load_n;
        let slip_power =
            (fx * (i.longitudinal_slip * i.speed_mps)).abs() + (fy * (i.slip_angle_rad * i.speed_mps)).abs();
        state.contact_patch_m2 =
            (i.normal_load_n / (state.pressure_pa.max(15_000.0)) * 0.78 * (1.0 + 0.5 * state.carcass_damage))
                .clamp(0.004, 0.12);
        self.evolve(state, i, slip_power);
        TireOutput {
            longitudinal_force_n: fx,
            lateral_force_n: fy,
            aligning_moment_nm: -fy * trail,
            rolling_resistance_n: rr,
            hydroplaning: hydro,
            friction_coefficient: mu,
            slip_power_w: slip_power,
        }
    }
}

impl MagicFormulaTire {
    fn evolve(&self, s: &mut TireState, i: TireInput, slip_power: f64) {
        let road_exchange = (i.road.temperature_k - s.tread_temperature_k) * 0.045;
        let air_exchange = (293.15 - s.temperature_k) * 0.006;
        s.tread_temperature_k += (slip_power * 0.0018 + road_exchange) * i.dt;
        s.temperature_k += ((s.tread_temperature_k - s.temperature_k) * 0.08 + air_exchange) * i.dt;
        s.wear = (s.wear + slip_power * 2.0e-9 * i.dt * (1.0 + ((s.tread_temperature_k - 390.0) / 30.0).max(0.0)))
            .clamp(0.0, 1.0);
        if s.puncture_area_m2 > 0.0 {
            s.pressure_pa = (s.pressure_pa - s.puncture_area_m2 * 2.8e8 * i.dt).max(0.0);
        }
        if s.failure == TireFailure::Healthy && s.pressure_pa < 120_000.0 {
            s.failure = TireFailure::Punctured;
        }
        if matches!(s.failure, TireFailure::Punctured) && (s.pressure_pa < 35_000.0 || s.carcass_damage > 0.82) {
            s.failure = TireFailure::Blowout;
        }
        if matches!(s.failure, TireFailure::Blowout) && s.pressure_pa < 8_000.0 && i.speed_mps > 12.0 {
            s.failure = TireFailure::BeadUnseated;
        }
        if s.pressure_pa < 80_000.0 {
            s.carcass_damage =
                (s.carcass_damage + (80_000.0 - s.pressure_pa) * i.speed_mps * 2.0e-11 * i.dt).clamp(0.0, 1.0);
        }
    }
}
