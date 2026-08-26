//! Replaceable tire model and a deterministic Magic-Formula-family reference.

use crate::math::{clamp01, smoothstep};
use crate::provenance::{ParameterOrigin, ParameterProvenance, ParameterValidity};
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
    /// Relaxed slip angle used to calculate lateral force.
    pub slip_angle_rad: f64,
    /// Actual lateral contact velocity used for frictional work, kept separate
    /// from the relaxed angle that determines force.
    pub lateral_slip_speed_mps: f64,
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
    /// Frictional partition plus signed tread/road conductive heat.
    pub road_heat_w: f64,
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
    pub lateral_shape_factor: f64,
    pub lateral_curvature_factor: f64,
    pub pneumatic_trail_m: f64,
    pub relaxation_length_m: f64,
    pub tread_heat_capacity_j_k: f64,
    pub bulk_heat_capacity_j_k: f64,
    pub slip_heat_fraction_to_tread: f64,
    pub tread_bulk_conductance_w_k: f64,
    pub tread_road_conductance_w_k: f64,
    pub still_air_conductance_w_k: f64,
    pub speed_air_conductance_w_k_per_mps: f64,
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
            lateral_shape_factor: 1.35,
            lateral_curvature_factor: -1.0,
            pneumatic_trail_m: 0.055,
            relaxation_length_m: 0.45,
            tread_heat_capacity_j_k: 14_000.0,
            bulk_heat_capacity_j_k: 38_000.0,
            slip_heat_fraction_to_tread: 0.82,
            tread_bulk_conductance_w_k: 120.0,
            tread_road_conductance_w_k: 65.0,
            still_air_conductance_w_k: 18.0,
            speed_air_conductance_w_k_per_mps: 1.1,
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

        // Magic-Formula-family longitudinal curve. C and E produce a broad
        // dry peak around 10-20% slip and retain useful sliding friction at a
        // locked-wheel slip of one. Combined demand is normalized below.
        let bx = self.longitudinal_stiffness * i.longitudinal_slip;
        let sx = (1.9 * (bx - 0.94 * (bx - bx.atan())).atan()).sin();
        // Preserve the original small-angle gradient (`d sy / d alpha` is
        // `lateral_stiffness`) while adding a finite peak and a lower sliding
        // branch at large slip. The authored C/E pair is Magic-Formula-family,
        // not a proprietary fitted tire data set.
        let cy = self.lateral_shape_factor.max(1.01);
        let by = self.lateral_stiffness / cy;
        let bay = by * i.slip_angle_rad;
        let sy = (cy * (bay - self.lateral_curvature_factor * (bay - bay.atan())).atan()).sin();
        let combined = (sx * sx + sy * sy).sqrt().max(1.0);
        let peak = mu * i.normal_load_n;
        let raw_fx = peak * sx / combined;
        let camber_thrust = (-i.camber_rad * 0.08 * i.normal_load_n).clamp(-peak * 0.18, peak * 0.18);
        let raw_fy = (-peak * sy / combined + camber_thrust).clamp(-peak, peak);
        let ellipse = (raw_fx * raw_fx + raw_fy * raw_fy).sqrt().max(peak);
        let fx = raw_fx * peak / ellipse;
        let fy = raw_fy * peak / ellipse;
        let trail_decay = (-(i.slip_angle_rad.abs() / 0.14).powi(2)).exp();
        let trail = self.pneumatic_trail_m * trail_decay * (1.0 - state.wear) * pressure_ratio.sqrt();
        let rr_coeff = 0.012 + 0.025 * (1.0 - pressure_ratio).max(0.0) + 0.08 * state.carcass_damage;
        let rr = rr_coeff * i.normal_load_n;
        let slip_power = (fx * (i.longitudinal_slip * i.speed_mps)).abs() + (fy * i.lateral_slip_speed_mps).abs();
        state.contact_patch_m2 =
            (i.normal_load_n / (state.pressure_pa.max(15_000.0)) * 0.78 * (1.0 + 0.5 * state.carcass_damage))
                .clamp(0.004, 0.12);
        let road_heat_w = self.evolve(state, i, slip_power);
        TireOutput {
            longitudinal_force_n: fx,
            lateral_force_n: fy,
            aligning_moment_nm: -fy * trail,
            rolling_resistance_n: rr,
            hydroplaning: hydro,
            friction_coefficient: mu,
            slip_power_w: slip_power,
            road_heat_w,
        }
    }
}

impl MagicFormulaTire {
    pub fn parameter_provenance(&self) -> ParameterProvenance {
        ParameterProvenance::new(
            ParameterOrigin::Authored,
            "v0.1 Magic-Formula-family prototype calibration; no tire-rig or OEM measurement",
            "transient-thermal-v1",
            None,
            vec![
                ParameterValidity::new("lateral_shape_factor", "ratio", 1.01, 2.0),
                ParameterValidity::new("lateral_curvature_factor", "ratio", -3.0, 1.0),
                ParameterValidity::new("pneumatic_trail", "m", 0.0, 0.2),
                ParameterValidity::new("relaxation_length", "m", 0.05, 2.0),
                ParameterValidity::new("tread_heat_capacity", "J/K", 1_000.0, 100_000.0),
                ParameterValidity::new("bulk_heat_capacity", "J/K", 1_000.0, 200_000.0),
                ParameterValidity::new("slip_heat_fraction_to_tread", "ratio", 0.0, 1.0),
                ParameterValidity::new("tread_bulk_conductance", "W/K", 0.0, 1_000.0),
                ParameterValidity::new("tread_road_conductance", "W/K", 0.0, 1_000.0),
                ParameterValidity::new("still_air_conductance", "W/K", 0.0, 1_000.0),
                ParameterValidity::new("speed_air_conductance", "W/K/(m/s)", 0.0, 100.0),
            ],
        )
    }

    fn evolve(&self, s: &mut TireState, i: TireInput, slip_power: f64) -> f64 {
        let tire_fraction = self.slip_heat_fraction_to_tread.clamp(0.0, 1.0);
        let friction_to_tread_w = slip_power * tire_fraction;
        let friction_to_road_w = slip_power * (1.0 - tire_fraction);
        let contact_scale = clamp01(i.normal_load_n / self.nominal_load_n.max(1.0));
        let tread_to_road_w =
            self.tread_road_conductance_w_k.max(0.0) * contact_scale * (s.tread_temperature_k - i.road.temperature_k);
        let tread_to_bulk_w = self.tread_bulk_conductance_w_k.max(0.0) * (s.tread_temperature_k - s.temperature_k);
        let air_conductance =
            self.still_air_conductance_w_k.max(0.0) + self.speed_air_conductance_w_k_per_mps.max(0.0) * i.speed_mps;
        let bulk_to_air_w = air_conductance * (s.temperature_k - 293.15);
        let tread_capacity = self.tread_heat_capacity_j_k.max(1.0);
        let bulk_capacity = self.bulk_heat_capacity_j_k.max(1.0);
        s.tread_temperature_k += (friction_to_tread_w - tread_to_road_w - tread_to_bulk_w) / tread_capacity * i.dt;
        s.temperature_k += (tread_to_bulk_w - bulk_to_air_w) / bulk_capacity * i.dt;
        s.wear = (s.wear + slip_power * 2.0e-9 * i.dt * (1.0 + ((s.tread_temperature_k - 390.0) / 30.0).max(0.0)))
            .clamp(0.0, 1.0);
        if s.puncture_area_m2 > 0.0 {
            if s.failure == TireFailure::Healthy {
                s.failure = TireFailure::Punctured;
            }
            s.pressure_pa = (s.pressure_pa - s.puncture_area_m2 * 2.8e8 * i.dt).max(0.0);
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
        friction_to_road_w + tread_to_road_w
    }
}

/// Exact first-order transient-slip update over a travelled distance. The
/// low-speed fade prevents a noisy atan2 contact velocity from freezing a
/// non-zero tire force as the vehicle stops.
pub fn transient_slip_step(current: f64, kinematic: f64, speed_mps: f64, relaxation_length_m: f64, dt: f64) -> f64 {
    let speed = speed_mps.abs();
    let low_speed_weight = clamp01((speed - 0.25) / 1.25);
    let target = kinematic * low_speed_weight;
    let transport_speed = speed.max(0.5);
    let blend = 1.0 - (-transport_speed * dt.max(0.0) / relaxation_length_m.max(0.01)).exp();
    current + (target - current) * blend
}
