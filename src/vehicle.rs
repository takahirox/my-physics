use crate::controls::{AidSensors, ControlOutput, DriverAids, DriverInput};
use crate::feedback::{AudioFrame, FeedbackEvent, FeedbackEventKind, ForceFeedbackFrame};
use crate::math::{Quat, Vec3, clamp01};
use crate::tire::{MagicFormulaTire, TireInput, TireModel, TireOutput, TireState};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChassisDefinition {
    pub dry_mass_kg: f64,
    pub cg_local_m: Vec3,
    pub inertia_kg_m2: Vec3,
    pub frontal_area_m2: f64,
    pub drag_coefficient: f64,
    pub lift_coefficient: f64,
    pub air_density_kg_m3: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelDefinition {
    pub mount_local_m: Vec3,
    pub radius_m: f64,
    pub inertia_kg_m2: f64,
    pub mass_kg: f64,
    pub spring_rate_n_m: f64,
    pub damper_rate_n_s_m: f64,
    pub rest_length_m: f64,
    pub max_travel_m: f64,
    pub bump_stop_rate_n_m: f64,
    pub max_steer_rad: f64,
    pub driven: bool,
    pub brake_torque_nm: f64,
    /// Relative small-angle lateral stiffness for the fitted tire/wheel.
    pub cornering_stiffness_scale: f64,
    /// Relative peak friction for the fitted tire compound/size.
    pub tire_peak_grip_scale: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct EngineDefinition {
    pub idle_rpm: f64,
    pub redline_rpm: f64,
    pub inertia_kg_m2: f64,
    pub torque_curve: [(f64, f64); 8],
    pub fuel_energy_j_kg: f64,
    pub efficiency: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TransmissionDefinition {
    pub automatic: bool,
    pub gear_ratios: [f64; 7],
    pub reverse_ratio: f64,
    pub final_drive: f64,
    pub shift_time_s: f64,
    pub clutch_capacity_nm: f64,
    pub clutch_stiffness_nm_per_rad_s: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct VehicleDefinition {
    pub name: String,
    pub chassis: ChassisDefinition,
    pub wheels: [WheelDefinition; 4],
    pub engine: EngineDefinition,
    pub transmission: TransmissionDefinition,
    pub fuel_capacity_kg: f64,
    pub fuel_tank_local_m: Vec3,
    pub anti_roll_rate_n_m_rad: f64,
}

impl Default for VehicleDefinition {
    fn default() -> Self {
        let front = WheelDefinition {
            mount_local_m: Vec3::new(0.78, 0.0, -1.23),
            radius_m: 0.33,
            inertia_kg_m2: 1.35,
            mass_kg: 19.0,
            spring_rate_n_m: 40_000.0,
            damper_rate_n_s_m: 4_200.0,
            rest_length_m: 0.31,
            max_travel_m: 0.16,
            bump_stop_rate_n_m: 180_000.0,
            max_steer_rad: 0.54,
            driven: false,
            brake_torque_nm: 3_800.0,
            cornering_stiffness_scale: 1.0,
            tire_peak_grip_scale: 1.0,
        };
        let mut fl = front;
        fl.mount_local_m.x = -0.78;
        let rear = WheelDefinition {
            mount_local_m: Vec3::new(0.78, 0.0, 1.28),
            max_steer_rad: 0.0,
            driven: true,
            brake_torque_nm: 3_100.0,
            // Wider rear fitment gives the reference RWD car a small,
            // measurable understeer gradient at the adhesion limit.
            cornering_stiffness_scale: 1.05,
            tire_peak_grip_scale: 1.06,
            ..front
        };
        let mut rl = rear;
        rl.mount_local_m.x = -0.78;
        Self {
            name: "RWD Prototype".into(),
            chassis: ChassisDefinition {
                dry_mass_kg: 1380.0,
                cg_local_m: Vec3::ZERO,
                inertia_kg_m2: Vec3::new(610.0, 1650.0, 1810.0),
                frontal_area_m2: 2.05,
                drag_coefficient: 0.31,
                lift_coefficient: -0.22,
                air_density_kg_m3: 1.225,
            },
            wheels: [fl, front, rl, rear],
            engine: EngineDefinition {
                idle_rpm: 900.0,
                redline_rpm: 7600.0,
                inertia_kg_m2: 0.22,
                torque_curve: [
                    (900.0, 180.0),
                    (1800.0, 285.0),
                    (2800.0, 355.0),
                    (3800.0, 410.0),
                    (4800.0, 432.0),
                    (5800.0, 420.0),
                    (6800.0, 385.0),
                    (7600.0, 320.0),
                ],
                fuel_energy_j_kg: 43_000_000.0,
                efficiency: 0.31,
            },
            transmission: TransmissionDefinition {
                automatic: true,
                gear_ratios: [3.62, 2.19, 1.54, 1.21, 1.00, 0.84, 0.71],
                reverse_ratio: -3.20,
                final_drive: 3.73,
                shift_time_s: 0.16,
                clutch_capacity_nm: 610.0,
                clutch_stiffness_nm_per_rad_s: 4.0,
            },
            fuel_capacity_kg: 55.0,
            fuel_tank_local_m: Vec3::new(0.0, -0.05, 0.72),
            anti_roll_rate_n_m_rad: 12_000.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelState {
    pub angular_velocity_rad_s: f64,
    pub rotation_rad: f64,
    pub suspension_compression_m: f64,
    pub previous_compression_m: f64,
    pub steer_angle_rad: f64,
    pub camber_rad: f64,
    pub brake_temperature_k: f64,
    pub brake_wear: f64,
    pub wheel_damage: f64,
    pub tire: TireState,
    pub last_tire_output: TireOutput,
    pub last_normal_load_n: f64,
    pub longitudinal_slip: f64,
    pub slip_angle_rad: f64,
}

impl Default for WheelState {
    fn default() -> Self {
        Self {
            angular_velocity_rad_s: 0.0,
            rotation_rad: 0.0,
            suspension_compression_m: 0.0,
            previous_compression_m: 0.0,
            steer_angle_rad: 0.0,
            camber_rad: 0.0,
            brake_temperature_k: 300.0,
            brake_wear: 0.0,
            wheel_damage: 0.0,
            tire: TireState::default(),
            last_tire_output: TireOutput::default(),
            last_normal_load_n: 0.0,
            longitudinal_slip: 0.0,
            slip_angle_rad: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PowertrainState {
    pub engine_rpm: f64,
    pub throttle_actual: f64,
    pub gear: i8,
    pub shift_timer_s: f64,
    pub clutch_engagement: f64,
    pub clutch_temperature_k: f64,
    pub clutch_wear: f64,
    pub gearbox_wear: f64,
    pub clutch_failed: bool,
    pub gearbox_failed: bool,
    pub fuel_kg: f64,
    pub engine_temperature_k: f64,
    pub oil_temperature_k: f64,
    pub coolant_temperature_k: f64,
    pub oil_pressure_pa: f64,
    pub overheat_damage: f64,
    pub oil_damage: f64,
    pub overrev_damage: f64,
    pub failed: bool,
}

impl Default for PowertrainState {
    fn default() -> Self {
        Self {
            engine_rpm: 900.0,
            throttle_actual: 0.0,
            gear: 1,
            shift_timer_s: 0.0,
            clutch_engagement: 0.0,
            clutch_temperature_k: 320.0,
            clutch_wear: 0.0,
            gearbox_wear: 0.0,
            clutch_failed: false,
            gearbox_failed: false,
            fuel_kg: 40.0,
            engine_temperature_k: 350.0,
            oil_temperature_k: 345.0,
            coolant_temperature_k: 345.0,
            oil_pressure_pa: 350_000.0,
            overheat_damage: 0.0,
            oil_damage: 0.0,
            overrev_damage: 0.0,
            failed: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DamageState {
    pub body: f64,
    pub aero: f64,
    pub suspension: [f64; 4],
    pub deformation_local_m: Vec3,
    pub detached_mass_kg: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct VehicleState {
    pub position_m: Vec3,
    pub orientation: Quat,
    pub linear_velocity_mps: Vec3,
    pub angular_velocity_rad_s: Vec3,
    pub wheels: [WheelState; 4],
    pub powertrain: PowertrainState,
    pub damage: DamageState,
    pub simulation_time_s: f64,
}

impl Default for VehicleState {
    fn default() -> Self {
        Self {
            position_m: Vec3::new(0.0, 0.55, 0.0),
            orientation: Quat::IDENTITY,
            linear_velocity_mps: Vec3::ZERO,
            angular_velocity_rad_s: Vec3::ZERO,
            wheels: [WheelState::default(); 4],
            powertrain: PowertrainState::default(),
            damage: DamageState::default(),
            simulation_time_s: 0.0,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Telemetry {
    pub time_s: f64,
    pub position_m: Vec3,
    pub speed_mps: f64,
    pub acceleration_mps2: Vec3,
    pub yaw_rate_rad_s: f64,
    pub engine_rpm: f64,
    pub gear: i8,
    pub fuel_kg: f64,
    pub engine_temperature_k: f64,
    pub oil_pressure_pa: f64,
    pub clutch_temperature_k: f64,
    pub clutch_wear: f64,
    pub gearbox_wear: f64,
    pub wheel_slip: [f64; 4],
    pub tire_temperature_k: [f64; 4],
    pub tire_pressure_pa: [f64; 4],
    pub tire_wear: [f64; 4],
    pub normal_load_n: [f64; 4],
    pub brake_temperature_k: [f64; 4],
    pub hydroplaning: [f64; 4],
    pub abs_active: [bool; 4],
    pub tc_active: bool,
    pub esc_active: bool,
    pub body_damage: f64,
    pub fidelity: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct InterpolatedState {
    pub position_m: Vec3,
    pub orientation: Quat,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Vehicle {
    pub definition: VehicleDefinition,
    pub state: VehicleState,
    pub driver_aids: DriverAids,
    pub input: DriverInput,
    pub control: ControlOutput,
    pub telemetry: Telemetry,
    pub fidelity: f64,
    pub target_fidelity: f64,
    pub previous_position_m: Vec3,
    pub previous_orientation: Quat,
    pub audio: AudioFrame,
    pub force_feedback: ForceFeedbackFrame,
    pub events: Vec<FeedbackEvent>,
    pub(crate) cached_force: Vec3,
    pub(crate) cached_torque: Vec3,
    pub(crate) previous_gear: i8,
    pub(crate) previous_tire_failures: [crate::tire::TireFailure; 4],
    pub(crate) previous_engine_failed: bool,
    pub(crate) previous_clutch_failed: bool,
    pub(crate) previous_gearbox_failed: bool,
}

impl Vehicle {
    pub fn new(definition: VehicleDefinition) -> Self {
        Self {
            definition,
            state: VehicleState::default(),
            driver_aids: DriverAids::default(),
            input: DriverInput::default(),
            control: ControlOutput::default(),
            telemetry: Telemetry::default(),
            fidelity: 1.0,
            target_fidelity: 1.0,
            previous_position_m: Vec3::new(0.0, 0.55, 0.0),
            previous_orientation: Quat::IDENTITY,
            audio: AudioFrame::default(),
            force_feedback: ForceFeedbackFrame::default(),
            events: Vec::new(),
            cached_force: Vec3::ZERO,
            cached_torque: Vec3::ZERO,
            previous_gear: 1,
            previous_tire_failures: [crate::tire::TireFailure::Healthy; 4],
            previous_engine_failed: false,
            previous_clutch_failed: false,
            previous_gearbox_failed: false,
        }
    }
    pub fn mass_kg(&self) -> f64 {
        self.definition.chassis.dry_mass_kg + self.state.powertrain.fuel_kg - self.state.damage.detached_mass_kg
    }
    pub fn cg_local_m(&self) -> Vec3 {
        let dry = self.definition.chassis.dry_mass_kg - self.state.damage.detached_mass_kg;
        (self.definition.chassis.cg_local_m * dry + self.definition.fuel_tank_local_m * self.state.powertrain.fuel_kg)
            / (dry + self.state.powertrain.fuel_kg).max(1.0)
            + self.state.damage.deformation_local_m
    }
    pub fn inertia_kg_m2(&self) -> Vec3 {
        let detached_fraction =
            (self.state.damage.detached_mass_kg / self.definition.chassis.dry_mass_kg).clamp(0.0, 0.2);
        let mut inertia = self.definition.chassis.inertia_kg_m2 * (1.0 - detached_fraction);
        let fuel_offset = self.definition.fuel_tank_local_m - self.cg_local_m();
        inertia.x += self.state.powertrain.fuel_kg * (fuel_offset.y * fuel_offset.y + fuel_offset.z * fuel_offset.z);
        inertia.y += self.state.powertrain.fuel_kg * (fuel_offset.x * fuel_offset.x + fuel_offset.z * fuel_offset.z);
        inertia.z += self.state.powertrain.fuel_kg * (fuel_offset.x * fuel_offset.x + fuel_offset.y * fuel_offset.y);
        let deformation = self.state.damage.deformation_local_m.length();
        inertia * (1.0 - 0.12 * self.state.damage.body + 0.04 * deformation).clamp(0.65, 1.1)
    }
    pub fn collision_half_extents_m(&self) -> Vec3 {
        Vec3::new(
            0.95 * (1.0 - 0.08 * self.state.damage.body),
            0.70 * (1.0 - 0.12 * self.state.damage.body),
            2.15 * (1.0 - 0.22 * self.state.damage.body),
        )
    }
    pub fn interpolated_state(&self, alpha: f64) -> InterpolatedState {
        let alpha = clamp01(alpha);
        InterpolatedState {
            position_m: self.previous_position_m.lerp(self.state.position_m, alpha),
            orientation: self.previous_orientation.nlerp(self.state.orientation, alpha),
        }
    }
    pub fn sensors(&self) -> AidSensors {
        AidSensors {
            wheel_slip: self.state.wheels.map(|w| w.longitudinal_slip),
            speed_mps: self.state.linear_velocity_mps.length(),
            yaw_rate_rad_s: self.state.angular_velocity_rad_s.y,
        }
    }
    pub fn update_controls(&mut self, dt: f64) {
        self.control = self.driver_aids.update(self.input, self.sensors(), dt);
    }
    pub fn engine_torque_nm(&self) -> f64 {
        if self.state.powertrain.failed || self.state.powertrain.fuel_kg <= 0.0 {
            return 0.0;
        }
        let rpm = self.state.powertrain.engine_rpm;
        let curve = &self.definition.engine.torque_curve;
        let mut torque = curve[0].1;
        for pair in curve.windows(2) {
            if rpm >= pair[0].0 && rpm <= pair[1].0 {
                let t = (rpm - pair[0].0) / (pair[1].0 - pair[0].0);
                torque = pair[0].1 + (pair[1].1 - pair[0].1) * t;
                break;
            }
            if rpm > pair[1].0 {
                torque = pair[1].1;
            }
        }
        torque * self.state.powertrain.throttle_actual * (1.0 - 0.7 * self.total_engine_damage())
    }
    pub fn gear_ratio(&self) -> f64 {
        let g = self.state.powertrain.gear;
        if g < 0 {
            self.definition.transmission.reverse_ratio
        } else if g == 0 {
            0.0
        } else {
            self.definition.transmission.gear_ratios[(g as usize - 1).min(6)]
        }
    }
    pub fn total_engine_damage(&self) -> f64 {
        (self.state.powertrain.overheat_damage
            + self.state.powertrain.oil_damage
            + self.state.powertrain.overrev_damage)
            .clamp(0.0, 1.0)
    }
    pub fn update_powertrain(&mut self, dt: f64) -> f64 {
        let d = &self.definition;
        let p = &mut self.state.powertrain;
        p.throttle_actual += (self.control.throttle - p.throttle_actual) * clamp01(dt / 0.075);
        let automatic_request = if d.transmission.automatic && self.control.gear_request == 0 {
            if p.engine_rpm > 6_600.0 && p.gear > 0 && p.gear < 7 {
                p.gear + 1
            } else if p.engine_rpm < 1_650.0 && p.gear > 1 {
                p.gear - 1
            } else {
                p.gear
            }
        } else {
            self.control.gear_request
        };
        if p.gearbox_failed {
            p.gear = 0;
        } else if automatic_request != 0 && automatic_request != p.gear && p.shift_timer_s <= 0.0 {
            let shift_severity = ((p.engine_rpm - 3_500.0).abs() / 5_000.0).clamp(0.0, 1.0);
            p.gearbox_wear = (p.gearbox_wear + 2.0e-6 + shift_severity * 4.0e-6).clamp(0.0, 1.0);
            p.shift_timer_s = d.transmission.shift_time_s;
            p.gear = automatic_request;
        }
        p.shift_timer_s = (p.shift_timer_s - dt).max(0.0);
        let requested_engagement = 1.0 - self.control.clutch;
        let engagement_target = if p.shift_timer_s > 0.0 { 0.0 } else { requested_engagement };
        p.clutch_engagement += (engagement_target - p.clutch_engagement) * clamp01(dt / 0.045);
        let ratio = {
            let g = p.gear;
            if g < 0 {
                d.transmission.reverse_ratio
            } else if g == 0 {
                0.0
            } else {
                d.transmission.gear_ratios[(g as usize - 1).min(6)]
            }
        } * d.transmission.final_drive;
        let driven_omega = (self.state.wheels[2].angular_velocity_rad_s + self.state.wheels[3].angular_velocity_rad_s)
            * 0.5
            * ratio.abs();
        let engine_omega = p.engine_rpm * core::f64::consts::TAU / 60.0;
        let slip = engine_omega - driven_omega;
        let clutch_health = if p.clutch_failed { 0.0 } else { 1.0 - p.clutch_wear * 0.8 };
        let clutch_capacity = d.transmission.clutch_capacity_nm * clutch_health * p.clutch_engagement;
        let clutch_torque = if ratio.abs() <= f64::EPSILON {
            0.0
        } else {
            (slip * d.transmission.clutch_stiffness_nm_per_rad_s).clamp(-clutch_capacity, clutch_capacity)
        };
        let friction = 12.0 + engine_omega * 0.025;
        let engine_torque = {
            let curve = &d.engine.torque_curve;
            let mut torque = curve[0].1;
            for pair in curve.windows(2) {
                if p.engine_rpm >= pair[0].0 && p.engine_rpm <= pair[1].0 {
                    let t = (p.engine_rpm - pair[0].0) / (pair[1].0 - pair[0].0);
                    torque = pair[0].1 + (pair[1].1 - pair[0].1) * t;
                    break;
                }
                if p.engine_rpm > pair[1].0 {
                    torque = pair[1].1;
                }
            }
            let limiter = clamp01((d.engine.redline_rpm - p.engine_rpm) / 250.0);
            let shift_torque_cut = if p.shift_timer_s > 0.0 { 0.0 } else { 1.0 };
            if p.failed || p.fuel_kg <= 0.0 { 0.0 } else { torque * p.throttle_actual * limiter * shift_torque_cut }
        };
        // A running ICE must burn fuel to balance its internal friction at
        // idle. Previously the lower RPM clamp supplied this energy for free.
        // The idle governor only supplies the part not already produced by the
        // driver's throttle and fades out over the first 250 RPM above idle.
        let idle_blend = clamp01((d.engine.idle_rpm + 250.0 - p.engine_rpm) / 250.0);
        let idle_governor_torque =
            if p.failed || p.fuel_kg <= 0.0 { 0.0 } else { (friction - engine_torque).max(0.0) * idle_blend };
        let combustion_torque = engine_torque + idle_governor_torque;
        let domega = (combustion_torque - clutch_torque - friction) / d.engine.inertia_kg_m2 * dt;
        let idle_omega = d.engine.idle_rpm * core::f64::consts::TAU / 60.0;
        let limiter_omega = (d.engine.redline_rpm + 50.0) * core::f64::consts::TAU / 60.0;
        let minimum_omega = if p.failed || p.fuel_kg <= 0.0 { 0.0 } else { idle_omega };
        let new_omega = (engine_omega + domega).clamp(minimum_omega, limiter_omega);
        p.engine_rpm = new_omega * 60.0 / core::f64::consts::TAU;
        let wheel_torque = clutch_torque * ratio * 0.94;
        let slip_power = (clutch_torque * slip).abs();
        p.clutch_temperature_k += (slip_power * 0.0012 + (300.0 - p.clutch_temperature_k) * 0.018) * dt;
        p.clutch_wear = (p.clutch_wear
            + slip_power * 2.2e-10 * dt * (1.0 + ((p.clutch_temperature_k - 520.0) / 100.0).max(0.0)))
        .clamp(0.0, 1.0);
        p.gearbox_wear = (p.gearbox_wear + wheel_torque.abs() * 1.0e-12 * dt).clamp(0.0, 1.0);
        p.clutch_failed = p.clutch_wear >= 1.0;
        p.gearbox_failed = p.gearbox_wear >= 1.0;
        let efficiency = d.engine.efficiency.max(0.05);
        let load_fuel_power = (engine_torque * new_omega).max(0.0) / efficiency;
        let idle_fuel_power = (idle_governor_torque * new_omega).max(0.0) / efficiency;
        let fuel_power = load_fuel_power + idle_fuel_power;
        p.fuel_kg = (p.fuel_kg - fuel_power / d.engine.fuel_energy_j_kg * dt).max(0.0);
        // With no detailed radiator/airflow model in v0.1, separate stationary
        // idle heat rejection from the existing under-load calibration. This
        // gives the idle governor's real fuel energy a stable warm equilibrium
        // instead of incorrectly cooling a running engine to ambient.
        let combustion_heating_k_s = load_fuel_power * 1.2e-5 + idle_fuel_power * 4.25e-4;
        p.engine_temperature_k +=
            (combustion_heating_k_s - (p.engine_temperature_k - p.coolant_temperature_k) * 0.18) * dt;
        p.coolant_temperature_k += (p.engine_temperature_k - p.coolant_temperature_k) * 0.035 * dt
            + (300.0 - p.coolant_temperature_k) * 0.006 * dt;
        p.oil_temperature_k +=
            (p.engine_temperature_k - p.oil_temperature_k) * 0.025 * dt + (300.0 - p.oil_temperature_k) * 0.003 * dt;
        p.oil_pressure_pa = (120_000.0 + p.engine_rpm * 52.0) * (1.0 - 0.75 * p.oil_damage).max(0.1);
        p.overheat_damage = (p.overheat_damage
            + ((p.engine_temperature_k - 405.0) / 70.0).max(0.0).powi(2) * 0.012 * dt)
            .clamp(0.0, 1.0);
        p.oil_damage =
            (p.oil_damage + ((150_000.0 - p.oil_pressure_pa) / 150_000.0).max(0.0).powi(2) * 0.02 * dt).clamp(0.0, 1.0);
        p.overrev_damage = (p.overrev_damage
            + ((p.engine_rpm - d.engine.redline_rpm) / 1000.0).max(0.0).powi(2) * 0.08 * dt)
            .clamp(0.0, 1.0);
        p.failed = p.overheat_damage + p.oil_damage + p.overrev_damage >= 1.0;
        wheel_torque
    }
    pub fn update_telemetry(&mut self, acceleration: Vec3) {
        let s = &self.state;
        self.telemetry = Telemetry {
            time_s: s.simulation_time_s,
            position_m: s.position_m,
            speed_mps: s.linear_velocity_mps.length(),
            acceleration_mps2: acceleration,
            yaw_rate_rad_s: s.angular_velocity_rad_s.y,
            engine_rpm: s.powertrain.engine_rpm,
            gear: s.powertrain.gear,
            fuel_kg: s.powertrain.fuel_kg,
            engine_temperature_k: s.powertrain.engine_temperature_k,
            oil_pressure_pa: s.powertrain.oil_pressure_pa,
            clutch_temperature_k: s.powertrain.clutch_temperature_k,
            clutch_wear: s.powertrain.clutch_wear,
            gearbox_wear: s.powertrain.gearbox_wear,
            wheel_slip: s.wheels.map(|w| w.longitudinal_slip),
            tire_temperature_k: s.wheels.map(|w| w.tire.tread_temperature_k),
            tire_pressure_pa: s.wheels.map(|w| w.tire.pressure_pa),
            tire_wear: s.wheels.map(|w| w.tire.wear),
            normal_load_n: s.wheels.map(|w| w.last_normal_load_n),
            brake_temperature_k: s.wheels.map(|w| w.brake_temperature_k),
            hydroplaning: s.wheels.map(|w| w.last_tire_output.hydroplaning),
            abs_active: self.control.abs_active,
            tc_active: self.control.tc_active,
            esc_active: self.control.esc_active,
            body_damage: s.damage.body,
            fidelity: self.fidelity,
        };
        self.update_feedback();
    }

    fn update_feedback(&mut self) {
        let s = &self.state;
        let front_aligning =
            s.wheels[0].last_tire_output.aligning_moment_nm + s.wheels[1].last_tire_output.aligning_moment_nm;
        let scrub = s.wheels.map(|wheel| (wheel.longitudinal_slip.abs() + wheel.slip_angle_rad.abs()).clamp(0.0, 1.0));
        let suspension = s.wheels.map(|wheel| {
            ((wheel.suspension_compression_m - wheel.previous_compression_m).abs() / 0.02).clamp(0.0, 1.0)
        });
        self.audio = AudioFrame {
            engine_rpm: s.powertrain.engine_rpm,
            engine_load: s.powertrain.throttle_actual,
            intake: s.powertrain.throttle_actual.sqrt(),
            exhaust: (s.powertrain.engine_rpm / self.definition.engine.redline_rpm * s.powertrain.throttle_actual)
                .clamp(0.0, 1.2),
            tire_scrub: scrub,
            road_noise: s.wheels.map(|wheel| (wheel.angular_velocity_rad_s.abs() * 0.015).clamp(0.0, 1.0)),
            suspension_activity: suspension,
            wind: (s.linear_velocity_mps.length() / 70.0).clamp(0.0, 1.0),
            impact: self.audio.impact * 0.82,
        };
        self.force_feedback = ForceFeedbackFrame {
            steering_torque_nm: (front_aligning * 0.018).clamp(-18.0, 18.0),
            aligning_moment_nm: front_aligning,
            rack_force_n: (front_aligning / 0.075).clamp(-12_000.0, 12_000.0),
            road_vibration: suspension[0].max(suspension[1]),
            tire_scrub: scrub[0].max(scrub[1]),
            abs_pulse: if self.control.abs_active.iter().any(|active| *active) { 1.0 } else { 0.0 },
            impact: self.force_feedback.impact * 0.78,
        };
        if s.powertrain.gear != self.previous_gear {
            self.events.push(FeedbackEvent {
                time_s: s.simulation_time_s,
                kind: FeedbackEventKind::GearShift,
                magnitude: 1.0,
                wheel: None,
            });
            self.previous_gear = s.powertrain.gear;
        }
        for (index, wheel) in s.wheels.iter().enumerate() {
            if wheel.tire.failure != self.previous_tire_failures[index] {
                self.events.push(FeedbackEvent {
                    time_s: s.simulation_time_s,
                    kind: FeedbackEventKind::TireFailure,
                    magnitude: wheel.tire.carcass_damage.max(0.25),
                    wheel: Some(index as u8),
                });
                self.previous_tire_failures[index] = wheel.tire.failure;
            }
        }
        for (failed, previous, kind) in [
            (s.powertrain.failed, &mut self.previous_engine_failed, FeedbackEventKind::EngineFailure),
            (s.powertrain.clutch_failed, &mut self.previous_clutch_failed, FeedbackEventKind::ClutchFailure),
            (s.powertrain.gearbox_failed, &mut self.previous_gearbox_failed, FeedbackEventKind::GearboxFailure),
        ] {
            if failed && !*previous {
                self.events.push(FeedbackEvent { time_s: s.simulation_time_s, kind, magnitude: 1.0, wheel: None });
            }
            *previous = failed;
        }
    }
}

pub(crate) fn evaluate_tire(model: &MagicFormulaTire, state: &mut TireState, input: TireInput) -> TireOutput {
    model.evaluate(state, input)
}
