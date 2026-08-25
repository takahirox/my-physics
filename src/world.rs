use crate::circuit;
use crate::collision::{CollisionShape, DetachedBody, StaticCollider, oriented_box_contact, vehicle_static_contact};
use crate::controls::DriverInput;
use crate::math::{Quat, Vec3, clamp01, semi_implicit_linear_step};
use crate::road::DynamicRoad;
use crate::tire::{MagicFormulaTire, TireInput};
use crate::vehicle::{Vehicle, VehicleDefinition, evaluate_tire};

/// Inner face of the barriers on the procedural v0.1 demonstration circuit.
pub const DEMO_TRACK_HALF_WIDTH_M: f64 = 5.6;

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
        // Bound the dynamic-road cell count while covering the full-size
        // circuit and its barriers (720 m square at 4.5 m resolution).
        w.road = DynamicRoad::new(160, 160, 4.5);
        let circuit = circuit::segments();
        for n in 0..vehicle_count {
            let mut v = Vehicle::new(VehicleDefinition::default());
            // The circuit demo uses a race preset. ESC remains implemented and
            // can be enabled by applications, but is not allowed to fight rapid
            // driver-requested direction changes by default.
            v.driver_aids.stability_control_enabled = false;
            let row = n / 2;
            let segment = circuit[(circuit.len() + circuit.len() - row * 2) % circuit.len()];
            let lateral = if n % 2 == 0 { -1.55 } else { 1.55 };
            v.state.position_m = segment.center_m + segment.right * lateral + Vec3::new(0.0, 0.55, 0.0);
            v.state.orientation = Quat::from_axis_angle(Vec3::Y, segment.yaw_rad);
            v.previous_position_m = v.state.position_m;
            v.previous_orientation = v.state.orientation;
            v.target_fidelity = if n == 0 { 1.0 } else { 0.6 };
            w.vehicles.push(v);
        }
        for segment in circuit {
            let orientation = Quat::from_axis_angle(Vec3::Y, segment.yaw_rad);
            for side in [-1.0, 1.0] {
                w.static_colliders.push(StaticCollider {
                    position_m: segment.center_m
                        + segment.forward * (segment.length_m * 0.5)
                        + segment.right * side * (DEMO_TRACK_HALF_WIDTH_M + 0.3)
                        + Vec3::new(0.0, 1.0, 0.0),
                    orientation,
                    shape: CollisionShape::Box { half_extents_m: Vec3::new(0.3, 1.0, segment.length_m * 0.5 + 0.12) },
                    restitution: 0.15,
                    friction: 0.8,
                });
            }
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
            let phase = self.step_index;
            self.step_once(self.config.fixed_dt_s, phase, false)?;
            self.step_index += 1;
        }
        Ok(())
    }
    /// Advances one externally visible variable application step.
    ///
    /// The interval is internally substepped at no more than 1 ms. The public
    /// `step_index` advances once, so input frames recorded around variable
    /// calls use application-step indices rather than internal substep indices.
    pub fn step_variable(&mut self, dt_s: f64) -> Result<(), StepError> {
        if !dt_s.is_finite() || dt_s <= 0.0 || dt_s > self.config.max_variable_dt_s {
            return Err(StepError::InvalidTimestep);
        }
        // Vehicle/tire/powertrain dynamics are intentionally integrated at the
        // same maximum rate as the 1 kHz reference mode. A variable application
        // step is one externally visible step, but may contain several internal
        // physics substeps. This preserves input-history and public step-number
        // semantics while avoiding a 5--20 ms step through the stiff wheel and
        // suspension equations.
        const MAX_INTERNAL_DT_S: f64 = 0.001;
        let configured_dt = self.config.fixed_dt_s.clamp(f64::MIN_POSITIVE, MAX_INTERNAL_DT_S);
        let substeps = (dt_s / configured_dt).ceil().max(1.0) as u32;
        let substep_dt = dt_s / f64::from(substeps);
        for substep in 0..substeps {
            // Variable/offline integration favors convergence over LOD force
            // caching. Recompute every internal substep so repeated calls do
            // not derive a drifting LOD phase from the externally visible
            // (one-per-call) step counter.
            self.step_once(substep_dt, u64::from(substep), true)?;
        }
        self.step_index += 1;
        Ok(())
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
    /// Replays fixed-timestep input history. Variable-step applications must
    /// additionally record their application dt sequence; that format is not
    /// part of the v0.1 input-history archive.
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

    fn step_once(&mut self, dt: f64, integration_phase: u64, force_recompute: bool) -> Result<(), StepError> {
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
            let recompute = force_recompute || integration_phase.is_multiple_of(stride);
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
                let half_a = va.collision_half_extents_m();
                let half_b = vb.collision_half_extents_m();
                let broadphase_radius = half_a.length() + half_b.length();
                if delta.length_squared() <= broadphase_radius * broadphase_radius
                    && let Some(contact) = oriented_box_contact(
                        va.state.position_m,
                        va.state.orientation,
                        half_a,
                        vb.state.position_m,
                        vb.state.orientation,
                        half_b,
                    )
                {
                    let normal = contact.normal;
                    let r_a = contact.point_m - va.state.position_m;
                    let r_b = contact.point_m - vb.state.position_m;
                    let velocity_a = va.state.linear_velocity_mps + va.state.angular_velocity_rad_s.cross(r_a);
                    let velocity_b = vb.state.linear_velocity_mps + vb.state.angular_velocity_rad_s.cross(r_b);
                    let rel = (velocity_b - velocity_a).dot(normal);
                    let inv_a = 1.0 / va.mass_kg();
                    let inv_b = 1.0 / vb.mass_kg();
                    if rel < 0.0 {
                        let angular_a =
                            inverse_inertia_world(va.state.orientation, va.inertia_kg_m2(), r_a.cross(normal))
                                .cross(r_a);
                        let angular_b =
                            inverse_inertia_world(vb.state.orientation, vb.inertia_kg_m2(), r_b.cross(normal))
                                .cross(r_b);
                        let effective_inverse_mass = inv_a + inv_b + normal.dot(angular_a + angular_b);
                        let impulse = -(1.0 + 0.18) * rel / effective_inverse_mass.max(1.0e-12);
                        let impulse_world = normal * impulse;
                        va.state.linear_velocity_mps -= impulse_world * inv_a;
                        vb.state.linear_velocity_mps += impulse_world * inv_b;
                        va.state.angular_velocity_rad_s -=
                            inverse_inertia_world(va.state.orientation, va.inertia_kg_m2(), r_a.cross(impulse_world));
                        vb.state.angular_velocity_rad_s +=
                            inverse_inertia_world(vb.state.orientation, vb.inertia_kg_m2(), r_b.cross(impulse_world));
                        let energy = 0.5 * impulse * (-rel);
                        apply_impact_damage(va, energy, -normal);
                        apply_impact_damage(vb, energy, normal);

                        // Damage can change both inertia tensors. Apply the
                        // small corrective impulse required to retain the
                        // requested contact-point coefficient of restitution.
                        let velocity_a = va.state.linear_velocity_mps + va.state.angular_velocity_rad_s.cross(r_a);
                        let velocity_b = vb.state.linear_velocity_mps + vb.state.angular_velocity_rad_s.cross(r_b);
                        let current_relative_speed = (velocity_b - velocity_a).dot(normal);
                        let target_relative_speed = -0.18 * rel;
                        let angular_a =
                            inverse_inertia_world(va.state.orientation, va.inertia_kg_m2(), r_a.cross(normal))
                                .cross(r_a);
                        let angular_b =
                            inverse_inertia_world(vb.state.orientation, vb.inertia_kg_m2(), r_b.cross(normal))
                                .cross(r_b);
                        let corrected_inverse_mass = inv_a + inv_b + normal.dot(angular_a + angular_b);
                        let correction_impulse =
                            normal * ((target_relative_speed - current_relative_speed) / corrected_inverse_mass);
                        va.state.linear_velocity_mps -= correction_impulse * inv_a;
                        vb.state.linear_velocity_mps += correction_impulse * inv_b;
                        va.state.angular_velocity_rad_s -= inverse_inertia_world(
                            va.state.orientation,
                            va.inertia_kg_m2(),
                            r_a.cross(correction_impulse),
                        );
                        vb.state.angular_velocity_rad_s += inverse_inertia_world(
                            vb.state.orientation,
                            vb.inertia_kg_m2(),
                            r_b.cross(correction_impulse),
                        );
                    }
                    let correction = contact.penetration_m / (inv_a + inv_b).max(1.0e-12);
                    va.state.position_m -= normal * (correction * inv_a);
                    vb.state.position_m += normal * (correction * inv_b);
                }
            }
        }
    }
    fn solve_static_collisions(&mut self) {
        for v in &mut self.vehicles {
            for c in &self.static_colliders {
                let delta = c.position_m - v.state.position_m;
                let broadphase_radius = c.shape.bounding_radius() + v.collision_half_extents_m().length() + 0.5;
                if delta.length_squared() > broadphase_radius * broadphase_radius {
                    continue;
                }
                if let Some(contact) =
                    vehicle_static_contact(v.state.position_m, v.state.orientation, v.collision_half_extents_m(), c)
                {
                    let normal = contact.normal;
                    let r = contact.point_m - v.state.position_m;
                    let contact_velocity = v.state.linear_velocity_mps + v.state.angular_velocity_rad_s.cross(r);
                    let vn = contact_velocity.dot(normal);
                    if vn < 0.0 {
                        let speed = -vn;
                        let inverse_mass = 1.0 / v.mass_kg();
                        let inertia = v.inertia_kg_m2();
                        let angular = inverse_inertia_world(v.state.orientation, inertia, r.cross(normal)).cross(r);
                        let normal_impulse_magnitude =
                            -(1.0 + c.restitution) * vn / (inverse_mass + normal.dot(angular)).max(1.0e-12);
                        let normal_impulse = normal * normal_impulse_magnitude;
                        apply_impulse(v, normal_impulse, r);
                        apply_impact_damage(v, 0.5 * normal_impulse_magnitude * speed, normal);

                        let after_damage = v.state.linear_velocity_mps + v.state.angular_velocity_rad_s.cross(r);
                        let current_normal_speed = after_damage.dot(normal);
                        let target_normal_speed = -c.restitution * vn;
                        let corrected_inertia = v.inertia_kg_m2();
                        let corrected_angular =
                            inverse_inertia_world(v.state.orientation, corrected_inertia, r.cross(normal)).cross(r);
                        let correction_magnitude = (target_normal_speed - current_normal_speed)
                            / (inverse_mass + normal.dot(corrected_angular)).max(1.0e-12);
                        apply_impulse(v, normal * correction_magnitude, r);

                        let after_normal = v.state.linear_velocity_mps + v.state.angular_velocity_rad_s.cross(r);
                        let tangent_velocity = after_normal - normal * after_normal.dot(normal);
                        if tangent_velocity.length_squared() > 1.0e-12 {
                            let tangent = tangent_velocity.normalized();
                            let tangent_angular =
                                inverse_inertia_world(v.state.orientation, inertia, r.cross(tangent)).cross(r);
                            let unconstrained =
                                -tangent_velocity.length() / (inverse_mass + tangent.dot(tangent_angular)).max(1.0e-12);
                            let friction_limit = c.friction.clamp(0.0, 1.5) * normal_impulse_magnitude;
                            apply_impulse(v, tangent * unconstrained.max(-friction_limit), r);
                        }
                    }
                    v.state.position_m += normal * contact.penetration_m;
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
    let (orientation, angular_velocity) = integrate_rigid_body_rotation(
        v.state.orientation,
        v.state.angular_velocity_rad_s,
        v.cached_torque,
        inertia,
        dt,
    );
    v.state.orientation = orientation;
    v.state.angular_velocity_rad_s = angular_velocity;
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

fn inertia_world(orientation: Quat, inertia_body: Vec3, vector_world: Vec3) -> Vec3 {
    let body = orientation.conjugate().rotate(vector_world);
    orientation.rotate(Vec3::new(body.x * inertia_body.x, body.y * inertia_body.y, body.z * inertia_body.z))
}

fn inverse_inertia_world(orientation: Quat, inertia_body: Vec3, vector_world: Vec3) -> Vec3 {
    let body = orientation.conjugate().rotate(vector_world);
    orientation.rotate(Vec3::new(
        body.x / inertia_body.x.max(1.0e-12),
        body.y / inertia_body.y.max(1.0e-12),
        body.z / inertia_body.z.max(1.0e-12),
    ))
}

/// Integrates world angular momentum and derives angular velocity from the
/// rotated body inertia. This is Euler's rigid-body equation in world form:
/// `d(I_world * omega)/dt = torque_world`. Reconstructing omega after the
/// orientation update keeps torque-free world angular momentum conserved.
fn integrate_rigid_body_rotation(
    orientation: Quat,
    angular_velocity_world: Vec3,
    torque_world: Vec3,
    inertia_body: Vec3,
    dt: f64,
) -> (Quat, Vec3) {
    let angular_momentum = inertia_world(orientation, inertia_body, angular_velocity_world) + torque_world * dt;
    let omega_start = inverse_inertia_world(orientation, inertia_body, angular_momentum);
    let predicted_orientation = orientation.integrate_world_angular_velocity(omega_start, dt);
    let omega_end = inverse_inertia_world(predicted_orientation, inertia_body, angular_momentum);
    let midpoint_omega = (omega_start + omega_end) * 0.5;
    let next_orientation = orientation.integrate_world_angular_velocity(midpoint_omega, dt);
    let next_omega = inverse_inertia_world(next_orientation, inertia_body, angular_momentum);
    (next_orientation, next_omega)
}

fn apply_impulse(vehicle: &mut Vehicle, impulse_world: Vec3, r_world: Vec3) {
    vehicle.state.linear_velocity_mps += impulse_world / vehicle.mass_kg();
    vehicle.state.angular_velocity_rad_s +=
        inverse_inertia_world(vehicle.state.orientation, vehicle.inertia_kg_m2(), r_world.cross(impulse_world));
}

fn apply_impact_damage(v: &mut Vehicle, energy_j: f64, normal_world: Vec3) {
    // Deformation changes the inertia tensor. Preserve the angular momentum
    // delivered by the impact across that instantaneous tensor change.
    let angular_momentum = inertia_world(v.state.orientation, v.inertia_kg_m2(), v.state.angular_velocity_rad_s);
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
    v.state.angular_velocity_rad_s = inverse_inertia_world(v.state.orientation, v.inertia_kg_m2(), angular_momentum);
}

fn fingerprint_snapshot(s: &Snapshot) -> u64 {
    // The archive is the canonical complete-state representation. Its final
    // eight bytes are an FNV-1a checksum over every serialized field, including
    // configuration, road, definitions, all thermal/wear states, controls,
    // cached forces, colliders and detached bodies. Reusing it here prevents a
    // hand-maintained partial fingerprint from silently omitting new state.
    let bytes = crate::archive::encode_snapshot(s);
    u64::from_le_bytes(bytes[bytes.len() - 8..].try_into().expect("snapshot archive checksum"))
}
