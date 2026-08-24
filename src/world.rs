use crate::collision::{CollisionShape, DetachedBody, StaticCollider, oriented_box_contact, vehicle_static_contact};
use crate::controls::DriverInput;
use crate::math::{Quat, Vec3, clamp01, semi_implicit_linear_step};
use crate::road::DynamicRoad;
use crate::tire::{MagicFormulaTire, TireInput};
use crate::vehicle::{Vehicle, VehicleDefinition, evaluate_tire};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fidelity {
    Low,
    Medium,
    High,
}
impl Fidelity {
    pub fn scalar(self) -> f64 {
        match self {
            Self::Low => 0.25,
            Self::Medium => 0.6,
            Self::High => 1.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimulationConfig {
    pub fixed_dt_s: f64,
    pub gravity_mps2: f64,
    pub max_variable_dt_s: f64,
    pub lod_transition_s: f64,
    pub player_vehicle: usize,
    pub automatic_lod: bool,
    pub fidelity_ceiling: Fidelity,
}
impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            fixed_dt_s: 0.001,
            gravity_mps2: 9.80665,
            max_variable_dt_s: 0.02,
            lod_transition_s: 0.5,
            player_vehicle: 0,
            automatic_lod: true,
            fidelity_ceiling: Fidelity::High,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepError {
    InvalidTimestep,
    NonFiniteState,
    VehicleIndex,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InputFrame {
    pub step: u64,
    pub vehicle: usize,
    pub input: DriverInput,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Snapshot {
    pub(crate) config: SimulationConfig,
    pub(crate) time_s: f64,
    pub(crate) step: u64,
    pub(crate) road: DynamicRoad,
    pub(crate) wind_mps: Vec3,
    pub(crate) rain_rate_m_s: f64,
    pub(crate) vehicles: Vec<Vehicle>,
    pub(crate) static_colliders: Vec<StaticCollider>,
    pub(crate) detached_bodies: Vec<DetachedBody>,
    pub(crate) tire_model: MagicFormulaTire,
}

impl Snapshot {
    pub fn time_s(&self) -> f64 {
        self.time_s
    }
    pub fn step(&self) -> u64 {
        self.step
    }
    pub fn fingerprint(&self) -> u64 {
        fingerprint_snapshot(self)
    }
    pub fn to_bytes(&self) -> Vec<u8> {
        crate::archive::encode_snapshot(self)
    }
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, crate::archive::ArchiveError> {
        crate::archive::decode_snapshot(bytes)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct PhysicsWorld {
    pub config: SimulationConfig,
    pub time_s: f64,
    pub step_index: u64,
    pub road: DynamicRoad,
    pub wind_mps: Vec3,
    pub rain_rate_m_s: f64,
    pub vehicles: Vec<Vehicle>,
    pub static_colliders: Vec<StaticCollider>,
    pub detached_bodies: Vec<DetachedBody>,
    pub recorded_inputs: Vec<InputFrame>,
    tire_model: MagicFormulaTire,
}

impl PhysicsWorld {
    pub fn new(config: SimulationConfig) -> Self {
        Self {
            config,
            time_s: 0.0,
            step_index: 0,
            road: DynamicRoad::new(128, 128, 2.0),
            wind_mps: Vec3::ZERO,
            rain_rate_m_s: 0.0,
            vehicles: Vec::new(),
            static_colliders: Vec::new(),
            detached_bodies: Vec::new(),
            recorded_inputs: Vec::new(),
            tire_model: MagicFormulaTire::default(),
        }
    }
    pub fn demo(vehicle_count: usize) -> Self {
        let mut w = Self::new(SimulationConfig::default());
        for n in 0..vehicle_count {
            let mut v = Vehicle::new(VehicleDefinition::default());
            v.state.position_m = Vec3::new((n % 2) as f64 * 3.2 - 1.6, 0.55, (n / 2) as f64 * 5.5);
            v.target_fidelity = if n == 0 { 1.0 } else { 0.6 };
            w.vehicles.push(v);
        }
        w.static_colliders.push(StaticCollider {
            position_m: Vec3::new(-7.0, 1.0, -25.0),
            orientation: Quat::IDENTITY,
            shape: CollisionShape::Box { half_extents_m: Vec3::new(0.3, 1.0, 60.0) },
            restitution: 0.15,
            friction: 0.8,
        });
        w.static_colliders.push(StaticCollider {
            position_m: Vec3::new(7.0, 1.0, -25.0),
            orientation: Quat::IDENTITY,
            shape: CollisionShape::Box { half_extents_m: Vec3::new(0.3, 1.0, 60.0) },
            restitution: 0.15,
            friction: 0.8,
        });
        for x in [-6.55, 6.55] {
            w.static_colliders.push(StaticCollider {
                position_m: Vec3::new(x, 0.09, -25.0),
                orientation: Quat::IDENTITY,
                shape: CollisionShape::Box { half_extents_m: Vec3::new(0.35, 0.09, 60.0) },
                restitution: 0.05,
                friction: 0.95,
            });
        }
        w
    }
    pub fn add_vehicle(&mut self, definition: VehicleDefinition) -> usize {
        self.vehicles.push(Vehicle::new(definition));
        self.vehicles.len() - 1
    }
    pub fn set_fidelity_ceiling(&mut self, fidelity: Fidelity) {
        self.config.fidelity_ceiling = fidelity;
    }
    pub fn set_vehicle_fidelity(&mut self, index: usize, fidelity: Fidelity) -> Result<(), StepError> {
        let Some(vehicle) = self.vehicles.get_mut(index) else {
            return Err(StepError::VehicleIndex);
        };
        vehicle.target_fidelity = fidelity.scalar();
        Ok(())
    }
    pub fn set_input(&mut self, index: usize, input: DriverInput) -> Result<(), StepError> {
        let Some(v) = self.vehicles.get_mut(index) else {
            return Err(StepError::VehicleIndex);
        };
        let input = input.sanitized();
        v.input = input;
        self.recorded_inputs.push(InputFrame { step: self.step_index, vehicle: index, input });
        Ok(())
    }
    pub fn set_input_unrecorded(&mut self, index: usize, input: DriverInput) -> Result<(), StepError> {
        let Some(v) = self.vehicles.get_mut(index) else {
            return Err(StepError::VehicleIndex);
        };
        v.input = input.sanitized();
        Ok(())
    }
    pub fn step_fixed(&mut self, steps: u32) -> Result<(), StepError> {
        for _ in 0..steps {
            self.step_once(self.config.fixed_dt_s)?;
        }
        Ok(())
    }
    pub fn step_variable(&mut self, dt_s: f64) -> Result<(), StepError> {
        if !dt_s.is_finite() || dt_s <= 0.0 || dt_s > self.config.max_variable_dt_s {
            return Err(StepError::InvalidTimestep);
        }
        self.step_once(dt_s)
    }
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            config: self.config,
            time_s: self.time_s,
            step: self.step_index,
            road: self.road.clone(),
            wind_mps: self.wind_mps,
            rain_rate_m_s: self.rain_rate_m_s,
            vehicles: self.vehicles.clone(),
            static_colliders: self.static_colliders.clone(),
            detached_bodies: self.detached_bodies.clone(),
            tire_model: self.tire_model,
        }
    }
    pub fn restore(&mut self, s: &Snapshot) {
        self.config = s.config;
        self.time_s = s.time_s;
        self.step_index = s.step;
        self.road = s.road.clone();
        self.wind_mps = s.wind_mps;
        self.rain_rate_m_s = s.rain_rate_m_s;
        self.vehicles = s.vehicles.clone();
        self.static_colliders = s.static_colliders.clone();
        self.detached_bodies = s.detached_bodies.clone();
        self.tire_model = s.tire_model;
        self.recorded_inputs.clear();
    }
    pub fn replay_from(&mut self, snapshot: &Snapshot, inputs: &[InputFrame], end_step: u64) -> Result<(), StepError> {
        self.restore(snapshot);
        let mut cursor = 0;
        while self.step_index < end_step {
            while cursor < inputs.len() && inputs[cursor].step == self.step_index {
                let f = inputs[cursor];
                self.set_input_unrecorded(f.vehicle, f.input)?;
                cursor += 1;
            }
            self.step_fixed(1)?;
        }
        Ok(())
    }
    pub fn state_fingerprint(&self) -> u64 {
        self.snapshot().fingerprint()
    }

    fn step_once(&mut self, dt: f64) -> Result<(), StepError> {
        self.update_lod(dt);
        self.road.update_weather(self.rain_rate_m_s, dt);
        for n in 0..self.vehicles.len() {
            self.vehicles[n].events.clear();
            self.vehicles[n].previous_position_m = self.vehicles[n].state.position_m;
            self.vehicles[n].previous_orientation = self.vehicles[n].state.orientation;
            let stride = if n == self.config.player_vehicle || self.vehicles[n].fidelity > 0.8 {
                1
            } else if self.vehicles[n].fidelity > 0.45 {
                4
            } else {
                10
            };
            let recompute = self.step_index.is_multiple_of(stride);
            integrate_vehicle(
                &mut self.vehicles[n],
                &mut self.road,
                IntegrationContext {
                    tire_model: self.tire_model,
                    wind: self.wind_mps,
                    gravity: self.config.gravity_mps2,
                    dt,
                    recompute,
                    lod_stride: stride as f64,
                },
            );
        }
        self.solve_vehicle_collisions();
        self.solve_static_collisions();
        self.spawn_detached_components();
        self.integrate_detached(dt);
        self.time_s += dt;
        self.step_index += 1;
        if self.vehicles.iter().any(|v| !v.state.position_m.finite() || !v.state.linear_velocity_mps.finite()) {
            return Err(StepError::NonFiniteState);
        }
        Ok(())
    }
    fn update_lod(&mut self, dt: f64) {
        let player_position =
            self.vehicles.get(self.config.player_vehicle).map(|v| v.state.position_m).unwrap_or(Vec3::ZERO);
        for (n, v) in self.vehicles.iter_mut().enumerate() {
            if self.config.automatic_lod {
                let d = (v.state.position_m - player_position).length();
                let q = if n == self.config.player_vehicle || d < 18.0 {
                    Fidelity::High
                } else if d < 55.0 {
                    Fidelity::Medium
                } else {
                    Fidelity::Low
                };
                v.target_fidelity = q.scalar().min(self.config.fidelity_ceiling.scalar());
            }
            v.fidelity += (v.target_fidelity - v.fidelity) * clamp01(dt / self.config.lod_transition_s.max(dt));
        }
    }
    fn solve_vehicle_collisions(&mut self) {
        for a in 0..self.vehicles.len() {
            for b in a + 1..self.vehicles.len() {
                let (left, right) = self.vehicles.split_at_mut(b);
                let va = &mut left[a];
                let vb = &mut right[0];
                let delta = vb.state.position_m - va.state.position_m;
                if delta.y.abs() < 1.8
                    && let Some((normal, penetration)) = oriented_box_contact(
                        va.state.position_m,
                        va.state.orientation,
                        va.collision_half_extents_m(),
                        vb.state.position_m,
                        vb.state.orientation,
                        vb.collision_half_extents_m(),
                    )
                {
                    let rel = (vb.state.linear_velocity_mps - va.state.linear_velocity_mps).dot(normal);
                    let inv_a = 1.0 / va.mass_kg();
                    let inv_b = 1.0 / vb.mass_kg();
                    if rel < 0.0 {
                        let impulse = -(1.0 + 0.18) * rel / (inv_a + inv_b);
                        va.state.linear_velocity_mps -= normal * (impulse * inv_a);
                        vb.state.linear_velocity_mps += normal * (impulse * inv_b);
                        let energy = 0.5 * impulse * (-rel);
                        apply_impact_damage(va, energy, -normal);
                        apply_impact_damage(vb, energy, normal);
                    }
                    va.state.position_m -= normal * (penetration * 0.5);
                    vb.state.position_m += normal * (penetration * 0.5);
                }
            }
        }
    }
    fn solve_static_collisions(&mut self) {
        for v in &mut self.vehicles {
            for c in &self.static_colliders {
                if let Some((normal, penetration)) =
                    vehicle_static_contact(v.state.position_m, v.state.orientation, v.collision_half_extents_m(), c)
                {
                    let vn = v.state.linear_velocity_mps.dot(normal);
                    if vn < 0.0 {
                        let speed = -vn;
                        v.state.linear_velocity_mps -= normal * ((1.0 + c.restitution) * vn);
                        let tangent = v.state.linear_velocity_mps - normal * v.state.linear_velocity_mps.dot(normal);
                        v.state.linear_velocity_mps -= tangent * c.friction.clamp(0.0, 1.0) * 0.08;
                        apply_impact_damage(v, 0.5 * v.mass_kg() * speed * speed, normal);
                    }
                    v.state.position_m += normal * penetration;
                }
            }
        }
    }
    fn integrate_detached(&mut self, dt: f64) {
        for b in &mut self.detached_bodies {
            b.linear_velocity_mps.y -= self.config.gravity_mps2 * dt;
            b.position_m += b.linear_velocity_mps * dt;
            b.orientation = b.orientation.integrate_world_angular_velocity(b.angular_velocity_rad_s, dt);
            if b.position_m.y < 0.1 {
                b.position_m.y = 0.1;
                if b.linear_velocity_mps.y < 0.0 {
                    b.linear_velocity_mps.y *= -0.25;
                }
                b.linear_velocity_mps *= 0.995;
            }
        }
    }
    fn spawn_detached_components(&mut self) {
        for v in &mut self.vehicles {
            if v.state.damage.body > 0.72 && v.state.damage.detached_mass_kg == 0.0 {
                let local = Vec3::new(0.0, 0.05, -2.15);
                let position = v.state.position_m + v.state.orientation.rotate(local);
                self.detached_bodies.push(DetachedBody {
                    position_m: position,
                    orientation: v.state.orientation,
                    linear_velocity_mps: v.state.linear_velocity_mps
                        + v.state.orientation.rotate(Vec3::new(0.0, 1.2, -2.0)),
                    angular_velocity_rad_s: Vec3::new(1.0, 2.0, 0.4),
                    mass_kg: 15.0,
                    shape: CollisionShape::Capsule { radius_m: 0.12, half_height_m: 0.75 },
                    damage: v.state.damage.body,
                });
                v.state.damage.detached_mass_kg = 15.0;
            }
        }
    }
}

#[derive(Clone, Copy)]
struct IntegrationContext {
    tire_model: MagicFormulaTire,
    wind: Vec3,
    gravity: f64,
    dt: f64,
    recompute: bool,
    lod_stride: f64,
}

fn integrate_vehicle(v: &mut Vehicle, road: &mut DynamicRoad, context: IntegrationContext) {
    let IntegrationContext { tire_model, wind, gravity, dt, recompute, lod_stride } = context;
    v.update_controls(dt);
    let drive_torque = v.update_powertrain(dt);
    let mass = v.mass_kg();
    let old_velocity = v.state.linear_velocity_mps;
    if recompute {
        let orientation = v.state.orientation;
        let up = orientation.rotate(Vec3::Y);
        let cg_local = v.cg_local_m();
        let relative_air = v.state.linear_velocity_mps - wind;
        let air_speed = relative_air.length();
        let drag_scale = 1.0 + v.state.damage.aero * 0.8;
        let mut force = Vec3::new(0.0, -mass * gravity, 0.0);
        if air_speed > 1.0e-6 {
            force -= relative_air / air_speed
                * (0.5
                    * v.definition.chassis.air_density_kg_m3
                    * v.definition.chassis.drag_coefficient
                    * drag_scale
                    * v.definition.chassis.frontal_area_m2
                    * air_speed
                    * air_speed);
        }
        force += -up
            * (0.5
                * v.definition.chassis.air_density_kg_m3
                * (-v.definition.chassis.lift_coefficient)
                * (1.0 - 0.6 * v.state.damage.aero)
                * v.definition.chassis.frontal_area_m2
                * air_speed
                * air_speed);
        let mut torque = Vec3::ZERO;
        let mut compressions = [0.0; 4];
        for (n, wdef) in v.definition.wheels.iter().enumerate() {
            let mount = v.state.position_m + orientation.rotate(wdef.mount_local_m - cg_local);
            let length = mount.y - wdef.radius_m;
            compressions[n] = (wdef.rest_length_m - length).clamp(0.0, wdef.max_travel_m * 1.35);
        }
        for n in 0..4 {
            let wdef = v.definition.wheels[n];
            let wheel_damage = v.state.wheels[n].wheel_damage;
            let effective_radius = wdef.radius_m * (1.0 - 0.08 * wheel_damage);
            let mount = v.state.position_m + orientation.rotate(wdef.mount_local_m - cg_local);
            let ws = &mut v.state.wheels[n];
            let contact = Vec3::new(mount.x, 0.0, mount.z);
            let r = contact - v.state.position_m;
            let compression = compressions[n];
            let previous_compression = ws.suspension_compression_m;
            let compression_rate = (compression - previous_compression) / (dt * lod_stride);
            let overtravel = (compression - wdef.max_travel_m).max(0.0);
            let axle_other = if n % 2 == 0 { n + 1 } else { n - 1 };
            let anti_roll = (compression - compressions[axle_other]) * v.definition.anti_roll_rate_n_m_rad;
            let suspension_health = 1.0 - 0.7 * v.state.damage.suspension[n];
            let normal = (wdef.spring_rate_n_m * suspension_health * compression
                + wdef.damper_rate_n_s_m * suspension_health * compression_rate
                + wdef.bump_stop_rate_n_m * overtravel
                + anti_roll)
                .clamp(0.0, mass * gravity * 0.8);
            ws.previous_compression_m = previous_compression;
            ws.suspension_compression_m = compression;
            ws.last_normal_load_n = normal;
            ws.steer_angle_rad = v.control.steering * wdef.max_steer_rad;
            ws.camber_rad = wheel_damage * 0.12 * (if n % 2 == 0 { -1.0 } else { 1.0 });
            let steer_q = Quat::from_axis_angle(Vec3::Y, -ws.steer_angle_rad);
            let wheel_forward = orientation.rotate(steer_q.rotate(Vec3::FORWARD));
            let wheel_right = orientation.rotate(steer_q.rotate(Vec3::X));
            let contact_velocity = v.state.linear_velocity_mps + v.state.angular_velocity_rad_s.cross(r);
            let longitudinal = contact_velocity.dot(wheel_forward);
            let lateral = contact_velocity.dot(wheel_right);
            ws.longitudinal_slip =
                (ws.angular_velocity_rad_s * effective_radius - longitudinal) / longitudinal.abs().max(1.0);
            ws.slip_angle_rad = lateral.atan2(longitudinal.abs().max(0.2));
            let tire = evaluate_tire(
                &tire_model,
                &mut ws.tire,
                TireInput {
                    normal_load_n: normal,
                    longitudinal_slip: ws.longitudinal_slip,
                    slip_angle_rad: ws.slip_angle_rad,
                    camber_rad: ws.camber_rad,
                    speed_mps: contact_velocity.length(),
                    road: road.sample(contact),
                    dt: dt * lod_stride,
                },
            );
            ws.last_tire_output = tire;
            let rr_sign =
                if longitudinal.abs() > 0.1 { longitudinal.signum() } else { ws.angular_velocity_rad_s.signum() };
            let wheel_force = up * normal
                + wheel_forward * (tire.longitudinal_force_n - tire.rolling_resistance_n * rr_sign)
                + wheel_right * tire.lateral_force_n;
            force += wheel_force;
            torque += r.cross(wheel_force) + up * tire.aligning_moment_nm;
            let driven = if wdef.driven { drive_torque / 2.0 } else { 0.0 };
            let brake_effect =
                (1.0 - 0.72 * ((ws.brake_temperature_k - 850.0) / 300.0).clamp(0.0, 1.0)) * (1.0 - 0.7 * ws.brake_wear);
            let brake_torque = v.control.brake_per_wheel[n] * wdef.brake_torque_nm * brake_effect;
            let omega_sign = if ws.angular_velocity_rad_s.abs() > 0.1 {
                ws.angular_velocity_rad_s.signum()
            } else {
                longitudinal.signum()
            };
            let angular_accel = (driven - tire.longitudinal_force_n * effective_radius - brake_torque * omega_sign)
                / wdef.inertia_kg_m2;
            ws.angular_velocity_rad_s += angular_accel * dt * lod_stride;
            ws.rotation_rad = (ws.rotation_rad + ws.angular_velocity_rad_s * dt * lod_stride) % core::f64::consts::TAU;
            let brake_power = (brake_torque * ws.angular_velocity_rad_s).abs();
            ws.brake_temperature_k +=
                (brake_power * 0.00055 + (300.0 - ws.brake_temperature_k) * 0.016) * dt * lod_stride;
            ws.brake_wear = (ws.brake_wear + brake_power * 4.0e-11 * dt * lod_stride).clamp(0.0, 1.0);
            road.interact(contact, tire.slip_power_w * dt * lod_stride, ws.tire.tread_temperature_k, dt * lod_stride);
        }
        let lod_blend = if lod_stride <= 1.0 { 1.0 } else { clamp01(dt * lod_stride / 0.05) };
        v.cached_force = v.cached_force.lerp(force, lod_blend);
        v.cached_torque = v.cached_torque.lerp(torque, lod_blend);
    }
    let accel = v.cached_force / mass;
    semi_implicit_linear_step(&mut v.state.position_m, &mut v.state.linear_velocity_mps, accel, dt);
    let inertia = v.inertia_kg_m2();
    let angular_accel =
        Vec3::new(v.cached_torque.x / inertia.x, v.cached_torque.y / inertia.y, v.cached_torque.z / inertia.z);
    v.state.angular_velocity_rad_s += angular_accel * dt;
    v.state.angular_velocity_rad_s *= 1.0 - 0.02 * dt;
    v.state.orientation = v.state.orientation.integrate_world_angular_velocity(v.state.angular_velocity_rad_s, dt);
    if v.state.position_m.y < 0.18 {
        let impact = (-v.state.linear_velocity_mps.y).max(0.0);
        v.state.position_m.y = 0.18;
        v.state.linear_velocity_mps.y = v.state.linear_velocity_mps.y.max(0.0);
        if impact > 2.0 {
            apply_impact_damage(v, 0.5 * mass * impact * impact, Vec3::Y);
        }
    }
    v.state.simulation_time_s += dt;
    v.update_telemetry((v.state.linear_velocity_mps - old_velocity) / dt);
}

fn apply_impact_damage(v: &mut Vehicle, energy_j: f64, normal_world: Vec3) {
    let severity = (energy_j / 220_000.0).clamp(0.0, 0.35);
    v.audio.impact = (v.audio.impact + severity * 3.0).clamp(0.0, 1.0);
    v.force_feedback.impact = (v.force_feedback.impact + severity * 4.0).clamp(0.0, 1.0);
    if severity > 0.01 {
        v.events.push(crate::feedback::FeedbackEvent {
            time_s: v.state.simulation_time_s,
            kind: crate::feedback::FeedbackEventKind::Impact,
            magnitude: severity,
            wheel: None,
        });
    }
    v.state.damage.body = (v.state.damage.body + severity).clamp(0.0, 1.0);
    v.state.damage.aero = (v.state.damage.aero + severity * 0.7).clamp(0.0, 1.0);
    let local_normal = v.state.orientation.conjugate().rotate(normal_world).normalized();
    v.state.damage.deformation_local_m += local_normal * (severity * 0.025);
    if severity > 0.08 {
        let wheel = (v.state.damage.body.to_bits() as usize) % 4;
        v.state.damage.suspension[wheel] = (v.state.damage.suspension[wheel] + severity).clamp(0.0, 1.0);
        v.state.wheels[wheel].wheel_damage = (v.state.wheels[wheel].wheel_damage + severity).clamp(0.0, 1.0);
    }
}

fn fingerprint_snapshot(s: &Snapshot) -> u64 {
    let mut h = 0xcbf29ce484222325u64;
    fn mix(h: &mut u64, v: u64) {
        *h ^= v;
        *h = h.wrapping_mul(0x100000001b3);
    }
    mix(&mut h, s.step);
    mix(&mut h, s.time_s.to_bits());
    for v in &s.vehicles {
        let st = &v.state;
        for x in [
            st.position_m.x,
            st.position_m.y,
            st.position_m.z,
            st.orientation.w,
            st.orientation.x,
            st.orientation.y,
            st.orientation.z,
            st.linear_velocity_mps.x,
            st.linear_velocity_mps.y,
            st.linear_velocity_mps.z,
            st.angular_velocity_rad_s.x,
            st.angular_velocity_rad_s.y,
            st.angular_velocity_rad_s.z,
            st.powertrain.engine_rpm,
            st.powertrain.fuel_kg,
            st.damage.body,
        ] {
            mix(&mut h, x.to_bits());
        }
        for w in st.wheels {
            for x in [
                w.angular_velocity_rad_s,
                w.suspension_compression_m,
                w.tire.temperature_k,
                w.tire.tread_temperature_k,
                w.tire.wear,
                w.tire.pressure_pa,
            ] {
                mix(&mut h, x.to_bits());
            }
        }
    }
    for c in s.road.cells() {
        for x in [c.temperature_k, c.rubber, c.water_depth_m, c.contamination] {
            mix(&mut h, x.to_bits());
        }
    }
    h
}
