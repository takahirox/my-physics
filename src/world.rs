use crate::circuit;
use crate::collision::{CollisionShape, DetachedBody, StaticCollider, oriented_box_contact, vehicle_static_contact};
use crate::controls::DriverInput;
use crate::math::{Quat, Vec3, clamp01, semi_implicit_linear_step};
use crate::road::DynamicRoad;
use crate::tire::{MagicFormulaTire, TireInput, transient_slip_step};
use crate::vehicle::{Vehicle, VehicleDefinition, VehiclePreset, aerodynamic_drag_magnitude_n, evaluate_tire};

/// Inner face of the barriers on the procedural v0.1 demonstration circuit.
pub const DEMO_TRACK_HALF_WIDTH_M: f64 = 5.6;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fidelity {
    Low,
    Medium,
    High,
}

/// Geometry used for wheel/road contact. Keeping this explicit prevents the
/// circuit elevation profile from leaking into headless proving grounds or
/// real-world correlation runs that intentionally use a flat surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GroundSurface {
    Flat,
    DemoCircuit,
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
    pub ground_surface: GroundSurface,
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
            ground_surface: GroundSurface::Flat,
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
        Self::demo_with_preset(vehicle_count, VehiclePreset::RaceGameplay)
    }
    /// Single-vehicle, collision-free proving ground used by browser and
    /// native engineering tools. Physical models are unchanged; only the
    /// application-level world composition and LOD policy differ.
    pub fn engineering_lab() -> Self {
        let mut world = Self::new(SimulationConfig { automatic_lod: false, ..SimulationConfig::default() });
        let mut vehicle = Vehicle::new(VehicleDefinition::engineering_reference());
        vehicle.driver_aids.stability_control_enabled = false;
        vehicle.target_fidelity = 1.0;
        vehicle.fidelity = 1.0;
        world.vehicles.push(vehicle);
        world
    }
    pub fn demo_with_preset(vehicle_count: usize, preset: VehiclePreset) -> Self {
        let mut w =
            Self::new(SimulationConfig { ground_surface: GroundSurface::DemoCircuit, ..SimulationConfig::default() });
        // Bound the dynamic-road cell count while covering the full-size
        // circuit and its barriers (720 m square at 4.5 m resolution).
        w.road = DynamicRoad::new(160, 160, 4.5);
        let circuit = circuit::segments();
        for n in 0..vehicle_count {
            let mut v = Vehicle::new(VehicleDefinition::from_preset(preset));
            // Circuit demos select their physical definition explicitly. ESC
            // remains available, but does not fight rapid driver-requested
            // direction changes by default.
            v.driver_aids.stability_control_enabled = false;
            let row = n / 2;
            let segment = circuit[(circuit.len() + circuit.len() - row * 2) % circuit.len()];
            let lateral = if n % 2 == 0 { -1.55 } else { 1.55 };
            let surface = circuit::sample_surface(segment.center_m + segment.right * lateral);
            v.state.position_m = surface.point_m + surface.normal * 0.55;
            v.state.orientation = segment.orientation();
            v.previous_position_m = v.state.position_m;
            v.previous_orientation = v.state.orientation;
            v.target_fidelity = if n == 0 { 1.0 } else { 0.6 };
            w.vehicles.push(v);
        }
        for segment in circuit {
            let orientation = segment.orientation();
            for side in [-1.0, 1.0] {
                let midpoint = segment.center_m + segment.forward * (segment.length_m * 0.5);
                let surface =
                    circuit::sample_surface(midpoint + segment.right * side * (DEMO_TRACK_HALF_WIDTH_M + 0.3));
                w.static_colliders.push(StaticCollider {
                    position_m: surface.point_m + surface.normal,
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
                    ground_surface: self.config.ground_surface,
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
                                inverse_inertia_world(v.state.orientation, corrected_inertia, r.cross(tangent))
                                    .cross(r);
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
            let ground = ground_contact(self.config.ground_surface, b.position_m);
            let clearance = (b.position_m - ground.point_m).dot(ground.normal);
            if clearance < 0.1 {
                b.position_m += ground.normal * (0.1 - clearance);
                let normal_speed = b.linear_velocity_mps.dot(ground.normal);
                if normal_speed < 0.0 {
                    b.linear_velocity_mps -= ground.normal * (1.25 * normal_speed);
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
    ground_surface: GroundSurface,
}

#[derive(Clone, Copy)]
struct GroundContact {
    point_m: Vec3,
    normal: Vec3,
}

fn ground_contact(surface: GroundSurface, position_m: Vec3) -> GroundContact {
    match surface {
        GroundSurface::Flat => GroundContact { point_m: Vec3::new(position_m.x, 0.0, position_m.z), normal: Vec3::Y },
        GroundSurface::DemoCircuit => {
            let sample = circuit::sample_surface(position_m);
            GroundContact { point_m: sample.point_m, normal: sample.normal }
        }
    }
}

fn integrate_vehicle(v: &mut Vehicle, road: &mut DynamicRoad, context: IntegrationContext) {
    let IntegrationContext { tire_model, wind, gravity, dt, recompute, lod_stride, ground_surface } = context;
    v.update_controls(dt);
    let inertia_before_powertrain = v.inertia_kg_m2();
    let angular_momentum_before_powertrain =
        inertia_world(v.state.orientation, inertia_before_powertrain, v.state.angular_velocity_rad_s);
    let drive_torque = v.update_powertrain(dt);
    let driven_wheel_count = v.definition.wheels.iter().filter(|wheel| wheel.driven).count();
    let inertia_after_powertrain = v.inertia_kg_m2();
    if inertia_after_powertrain != inertia_before_powertrain {
        // Fuel burn changes the inertia tensor before force integration. Carry
        // forward the chassis angular momentum rather than reinterpreting the
        // previous angular velocity through the new tensor and introducing an
        // artificial torque.
        v.state.angular_velocity_rad_s =
            inverse_inertia_world(v.state.orientation, inertia_after_powertrain, angular_momentum_before_powertrain);
    }
    let mass = v.mass_kg();
    let old_velocity = v.state.linear_velocity_mps;
    if recompute {
        let orientation = v.state.orientation;
        let body_up = orientation.rotate(Vec3::Y);
        let cg_local = v.cg_local_m();
        let relative_air = v.state.linear_velocity_mps - wind;
        let air_speed = relative_air.length();
        let mut force = Vec3::new(0.0, -mass * gravity, 0.0);
        if air_speed > 1.0e-6 {
            force -= relative_air / air_speed
                * aerodynamic_drag_magnitude_n(&v.definition.chassis, air_speed, v.state.damage.aero);
        }
        force += -body_up
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
            let surface = ground_contact(ground_surface, mount);
            let length = (mount - surface.point_m).dot(surface.normal) - wdef.radius_m;
            compressions[n] = (wdef.rest_length_m - length).clamp(0.0, wdef.max_travel_m * 1.35);
        }
        for n in 0..4 {
            let wdef = v.definition.wheels[n];
            let wheel_damage = v.state.wheels[n].wheel_damage;
            let effective_radius = wdef.radius_m * (1.0 - 0.08 * wheel_damage);
            let mount = v.state.position_m + orientation.rotate(wdef.mount_local_m - cg_local);
            let surface = ground_contact(ground_surface, mount);
            let road_normal = surface.normal;
            let ws = &mut v.state.wheels[n];
            let contact = surface.point_m;
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
            let wheel_heading = orientation.rotate(steer_q.rotate(Vec3::FORWARD));
            let wheel_forward = (wheel_heading - road_normal * wheel_heading.dot(road_normal)).normalized();
            let wheel_right = wheel_forward.cross(road_normal).normalized();
            let contact_velocity = v.state.linear_velocity_mps + v.state.angular_velocity_rad_s.cross(r);
            let longitudinal = contact_velocity.dot(wheel_forward);
            let lateral = contact_velocity.dot(wheel_right);
            let wheel_surface_speed = ws.angular_velocity_rad_s * effective_radius;
            ws.longitudinal_slip = (wheel_surface_speed - longitudinal) / longitudinal.abs().max(1.0);
            ws.slip_angle_rad = lateral.atan2(longitudinal.abs().max(0.2));
            let fitted_tire_model = MagicFormulaTire {
                lateral_stiffness: tire_model.lateral_stiffness * wdef.cornering_stiffness_scale.clamp(0.5, 2.0),
                peak_mu: tire_model.peak_mu * wdef.tire_peak_grip_scale.clamp(0.5, 2.0),
                ..tire_model
            };
            let pressure_ratio =
                (ws.tire.pressure_pa / fitted_tire_model.nominal_pressure_pa.max(1.0)).clamp(0.05, 1.5);
            let load_ratio = (normal / fitted_tire_model.nominal_load_n.max(1.0)).clamp(0.05, 3.0);
            ws.relaxation_length_m = (fitted_tire_model.relaxation_length_m * pressure_ratio.sqrt()
                / load_ratio.powf(0.15)
                * (1.0 - 0.4 * ws.tire.carcass_damage))
                .clamp(0.08, 1.2);
            ws.transient_slip_angle_rad = transient_slip_step(
                ws.transient_slip_angle_rad,
                ws.slip_angle_rad,
                longitudinal,
                ws.relaxation_length_m,
                dt * lod_stride,
            );
            let mut tire = evaluate_tire(
                &fitted_tire_model,
                &mut ws.tire,
                TireInput {
                    normal_load_n: normal,
                    longitudinal_slip: ws.longitudinal_slip,
                    slip_angle_rad: ws.transient_slip_angle_rad,
                    lateral_slip_speed_mps: lateral,
                    camber_rad: ws.camber_rad,
                    // Tire slip energy and relaxation use longitudinal contact
                    // speed; lateral sliding speed is Vx*tan(alpha).
                    speed_mps: longitudinal.abs(),
                    road: road.sample(contact),
                    dt: dt * lod_stride,
                },
            );
            // The Magic-Formula force is quasi-static, while wheel and chassis
            // speeds are integrated explicitly. Near rest its small-slip
            // stiffness can otherwise reverse contact relative velocity in a
            // single 1 ms step and excite a non-physical limit cycle. Bound
            // the force by the impulse that can bring relative surface speed
            // to zero, but never through zero. At ordinary speed/slip the tire
            // friction envelope is lower, so this bound is inactive.
            let relative_surface_speed = wheel_surface_speed - longitudinal;
            let effective_inverse_mass = effective_radius * effective_radius / wdef.inertia_kg_m2 + 4.0 / mass.max(1.0);
            let contact_speed = longitudinal.abs().max(wheel_surface_speed.abs());
            let mut no_reversal_force = low_speed_no_reversal_force_limit(
                relative_surface_speed,
                contact_speed,
                dt * lod_stride,
                effective_inverse_mass,
            );
            if v.control.brake_per_wheel[n] > 0.0 && contact_speed < 1.0 {
                // The brake complementarity below may hold omega exactly zero,
                // so do not use the free-wheel relaxation near the final stop.
                // The strict bound remains conservative with the four-wheel
                // chassis coupling and prevents a post-stop chassis reversal.
                let strict_limit =
                    relative_surface_speed.abs() / ((dt * lod_stride) * effective_inverse_mass).max(1.0e-12);
                no_reversal_force = no_reversal_force.min(strict_limit);
            }
            tire.longitudinal_force_n = tire.longitudinal_force_n.clamp(-no_reversal_force, no_reversal_force);
            ws.last_tire_output = tire;
            let wheel_force =
                road_normal * normal + wheel_forward * tire.longitudinal_force_n + wheel_right * tire.lateral_force_n;
            force += wheel_force;
            torque += r.cross(wheel_force) + road_normal * tire.aligning_moment_nm;
            let driven =
                if wdef.driven && driven_wheel_count != 0 { drive_torque / driven_wheel_count as f64 } else { 0.0 };
            let brake_effect =
                (1.0 - 0.72 * ((ws.brake_temperature_k - 850.0) / 300.0).clamp(0.0, 1.0)) * (1.0 - 0.7 * ws.brake_wear);
            let brake_torque = v.control.brake_per_wheel[n] * wdef.brake_torque_nm * brake_effect;
            // Rolling resistance is a dissipative wheel moment. Applying it
            // directly to the chassis as well as omitting the wheel reaction
            // violated wheel/chassis power balance and produced finite-force
            // chatter from numerical omega sign changes near rest. The odd,
            // continuous speed factor is exactly zero at rest; ordinary tire
            // longitudinal force transmits the resulting drag to the chassis.
            let rolling_resistance_moment = rolling_resistance_moment_nm(
                tire.rolling_resistance_n,
                ws.angular_velocity_rad_s * effective_radius,
                effective_radius,
            );
            let unbraked_moment = driven - tire.longitudinal_force_n * effective_radius - rolling_resistance_moment;
            // Static/kinetic brake complementarity: apply the available brake
            // moment against the unbraked end-of-step angular momentum. If
            // capacity is sufficient the wheel lands exactly at zero and is
            // held there; it cannot alternate across zero or release for one
            // step merely because omega was exactly zero.
            let moment_to_zero =
                ws.angular_velocity_rad_s * wdef.inertia_kg_m2 / (dt * lod_stride).max(1.0e-12) + unbraked_moment;
            let brake_moment = moment_to_zero.clamp(-brake_torque, brake_torque);
            let angular_accel = (unbraked_moment - brake_moment) / wdef.inertia_kg_m2;
            ws.angular_velocity_rad_s += angular_accel * dt * lod_stride;
            ws.rotation_rad = (ws.rotation_rad + ws.angular_velocity_rad_s * dt * lod_stride) % core::f64::consts::TAU;
            let brake_power = (brake_moment * ws.angular_velocity_rad_s).abs();
            ws.brake_temperature_k +=
                (brake_power * 0.00055 + (300.0 - ws.brake_temperature_k) * 0.016) * dt * lod_stride;
            ws.brake_wear = (ws.brake_wear + brake_power * 4.0e-11 * dt * lod_stride).clamp(0.0, 1.0);
            road.interact_with_heat(
                contact,
                tire.slip_power_w * dt * lod_stride,
                tire.road_heat_w * dt * lod_stride,
                dt * lod_stride,
            );
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
    // Chassis-floor fallback for severe suspension compression or an inverted
    // vehicle. It follows the selected physical road instead of the legacy
    // y=0 plane, so a downhill section cannot silently become a flat floor.
    let ground = ground_contact(ground_surface, v.state.position_m);
    let clearance = (v.state.position_m - ground.point_m).dot(ground.normal);
    if clearance < 0.18 {
        let normal_speed = v.state.linear_velocity_mps.dot(ground.normal);
        let impact = (-normal_speed).max(0.0);
        v.state.position_m += ground.normal * (0.18 - clearance);
        if normal_speed < 0.0 {
            v.state.linear_velocity_mps -= ground.normal * normal_speed;
        }
        if impact > 2.0 {
            apply_impact_damage(v, 0.5 * mass * impact * impact, ground.normal);
        }
    }
    v.state.simulation_time_s += dt;
    v.update_telemetry((v.state.linear_velocity_mps - old_velocity) / dt);
}

/// Signed dissipative rolling-resistance moment. `surface_speed_mps` is wheel
/// angular speed times effective radius, so the returned sign always opposes
/// wheel rotation. The 0.1 m/s regularization removes a discontinuity at rest
/// while retaining more than 99% of the authored high-speed resistance.
fn rolling_resistance_moment_nm(force_n: f64, surface_speed_mps: f64, effective_radius_m: f64) -> f64 {
    force_n.max(0.0) * effective_radius_m.max(0.0) * surface_speed_mps / (surface_speed_mps.abs() + 0.1)
}

fn low_speed_no_reversal_force_limit(
    relative_surface_speed_mps: f64,
    contact_speed_mps: f64,
    dt_s: f64,
    effective_inverse_mass: f64,
) -> f64 {
    if contact_speed_mps >= 1.0 {
        return f64::INFINITY;
    }
    let zero_crossing_force = relative_surface_speed_mps.abs() / (dt_s * effective_inverse_mass).max(1.0e-12);
    // Keep the exact no-crossing bound through the final 0.2 m/s, then relax
    // continuously toward an inactive bound at 1 m/s. This exists only for the
    // low-speed explicit contact singularity and cannot alter normal driving.
    let relaxation = ((contact_speed_mps - 0.2) / 0.8).clamp(0.0, 1.0);
    zero_crossing_force / (1.0 - relaxation).max(1.0e-6)
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

#[cfg(test)]
mod rolling_resistance_tests {
    use super::*;
    use crate::controls::DriverInput;

    fn kinetic_energy(world: &PhysicsWorld) -> f64 {
        let vehicle = &world.vehicles[0];
        0.5 * vehicle.mass_kg() * vehicle.state.linear_velocity_mps.length_squared()
            + vehicle
                .state
                .wheels
                .iter()
                .zip(vehicle.definition.wheels.iter())
                .map(|(wheel, definition)| 0.5 * definition.inertia_kg_m2 * wheel.angular_velocity_rad_s.powi(2))
                .sum::<f64>()
    }

    #[test]
    fn rolling_resistance_moment_is_zero_at_rest_odd_and_dissipative() {
        assert_eq!(rolling_resistance_moment_nm(100.0, 0.0, 0.3), 0.0);
        for speed in [0.01, 0.1, 1.0, 30.0] {
            let forward = rolling_resistance_moment_nm(100.0, speed, 0.3);
            let reverse = rolling_resistance_moment_nm(100.0, -speed, 0.3);
            assert_eq!(reverse, -forward);
            assert!(forward * speed > 0.0, "resisting power must be positive before subtraction");
        }
        let high_speed = rolling_resistance_moment_nm(100.0, 30.0, 0.3);
        assert!(high_speed / (100.0 * 0.3) > 0.99);
        assert!(low_speed_no_reversal_force_limit(0.01, 2.0, 0.001, 0.08).is_infinite());
        assert!(low_speed_no_reversal_force_limit(0.01, 0.0, 0.001, 0.08).is_finite());
    }

    #[test]
    fn neutral_zero_input_rest_is_quiet_and_coast_energy_does_not_increase() {
        let mut definition = VehicleDefinition::engineering_reference();
        definition.transmission.automatic = false;
        let mut world = PhysicsWorld::new(SimulationConfig { automatic_lod: false, ..SimulationConfig::default() });
        world.add_vehicle(definition);
        world.vehicles[0].state.powertrain.gear = 0;
        world.set_input_unrecorded(0, DriverInput::default()).unwrap();
        world.step_fixed(2_000).unwrap();

        let mut sum = [0.0; 2];
        let mut sum_square = 0.0;
        for _ in 0..100 {
            world.step_fixed(1).unwrap();
            let acceleration = body_acceleration_channels_for_test(&world);
            sum[0] += acceleration[0];
            sum[1] += acceleration[1];
            sum_square += acceleration[0].powi(2) + acceleration[1].powi(2);
        }
        let mean = [sum[0] / 100.0, sum[1] / 100.0];
        let rms = (sum_square / 100.0).sqrt();
        let vehicle = &world.vehicles[0];
        let max_wheel_omega =
            vehicle.state.wheels.iter().map(|wheel| wheel.angular_velocity_rad_s.abs()).fold(0.0, f64::max);
        assert!(mean[0].abs() < 0.02, "mean={mean:?}, rms={rms}");
        assert!(mean[1].abs() < 0.02, "mean={mean:?}, rms={rms}");
        assert!(rms < 0.1, "mean={mean:?}, rms={rms}, max_wheel_omega={max_wheel_omega}");
        assert!(vehicle.state.linear_velocity_mps.length() < 0.01);
        assert!(vehicle.state.angular_velocity_rad_s.y.abs() < 1.0e-3);
        assert!(max_wheel_omega < 0.1, "max_wheel_omega={max_wheel_omega}, mean={mean:?}, rms={rms}");

        let speed_mps = 25.0;
        let vehicle = &mut world.vehicles[0];
        vehicle.state.linear_velocity_mps = Vec3::new(0.0, 0.0, -speed_mps);
        for (wheel, definition) in vehicle.state.wheels.iter_mut().zip(vehicle.definition.wheels.iter()) {
            wheel.angular_velocity_rad_s = speed_mps / definition.radius_m;
        }
        let initial_energy = kinetic_energy(&world);
        world.step_fixed(1_000).unwrap();
        assert!(kinetic_energy(&world) <= initial_energy * (1.0 + 1.0e-9));
    }

    #[test]
    fn straight_coast_converges_across_half_one_and_two_millisecond_steps() {
        fn coast(dt_s: f64) -> (f64, f64) {
            let mut definition = VehicleDefinition::engineering_reference();
            definition.transmission.automatic = false;
            let mut world = PhysicsWorld::new(SimulationConfig {
                fixed_dt_s: dt_s,
                automatic_lod: false,
                ..SimulationConfig::default()
            });
            world.add_vehicle(definition);
            world.vehicles[0].state.powertrain.gear = 0;
            world.set_input_unrecorded(0, DriverInput::default()).unwrap();
            world.step_fixed((2.0 / dt_s).round() as u32).unwrap();
            let vehicle = &mut world.vehicles[0];
            vehicle.state.linear_velocity_mps = Vec3::new(0.0, 0.0, -25.0);
            for (wheel, definition) in vehicle.state.wheels.iter_mut().zip(vehicle.definition.wheels.iter()) {
                wheel.angular_velocity_rad_s = 25.0 / definition.radius_m;
            }
            let start_z = vehicle.state.position_m.z;
            world.step_fixed((2.0 / dt_s).round() as u32).unwrap();
            let vehicle = &world.vehicles[0];
            ((vehicle.state.position_m.z - start_z).abs(), vehicle.state.linear_velocity_mps.length())
        }

        let reference = coast(0.0005);
        for candidate in [coast(0.001), coast(0.002)] {
            assert!(
                (candidate.0 - reference.0).abs() / reference.0 < 0.005,
                "reference={reference:?}, candidate={candidate:?}"
            );
            assert!(
                (candidate.1 - reference.1).abs() / reference.1 < 0.005,
                "reference={reference:?}, candidate={candidate:?}"
            );
        }
    }

    #[test]
    fn brake_to_rest_converges_without_contact_or_wheel_chatter() {
        let mut definition = VehicleDefinition::engineering_reference();
        definition.transmission.automatic = false;
        let mut world = PhysicsWorld::new(SimulationConfig { automatic_lod: false, ..SimulationConfig::default() });
        world.add_vehicle(definition);
        world.vehicles[0].state.powertrain.gear = 0;
        world.vehicles[0].driver_aids.abs_enabled = false;
        world.step_fixed(2_000).unwrap();
        let vehicle = &mut world.vehicles[0];
        vehicle.state.linear_velocity_mps = Vec3::new(0.0, 0.0, -10.0);
        for (wheel, definition) in vehicle.state.wheels.iter_mut().zip(vehicle.definition.wheels.iter()) {
            wheel.angular_velocity_rad_s = 10.0 / definition.radius_m;
        }
        world.set_input_unrecorded(0, DriverInput { brake: 1.0, ..DriverInput::default() }).unwrap();
        let mut minimum_contact_forward_speed: f64 = f64::INFINITY;
        let mut tail_contact_square = 0.0;
        let mut tail_wheel_square = 0.0;
        let mut tail_contact_peak: f64 = 0.0;
        let mut tail_wheel_peak: f64 = 0.0;
        let mut tail_samples = 0_u64;
        let mut wheel_signs = [0_i8; 4];
        let mut wheel_sign_flips = 0_u32;
        for step in 0..20_000 {
            world.step_fixed(1).unwrap();
            let vehicle = &world.vehicles[0];
            let cg = vehicle.cg_local_m();
            for (index, definition) in vehicle.definition.wheels.into_iter().enumerate() {
                let mount = vehicle.state.position_m + vehicle.state.orientation.rotate(definition.mount_local_m - cg);
                let contact = Vec3::new(mount.x, 0.0, mount.z);
                let contact_velocity = vehicle.state.linear_velocity_mps
                    + vehicle.state.angular_velocity_rad_s.cross(contact - vehicle.state.position_m);
                let heading = vehicle.state.orientation.rotate(Vec3::FORWARD);
                let forward = Vec3::new(heading.x, 0.0, heading.z).normalized();
                let contact_forward = contact_velocity.dot(forward);
                minimum_contact_forward_speed = minimum_contact_forward_speed.min(contact_forward);
                if step >= 15_000 {
                    let wheel_omega = vehicle.state.wheels[index].angular_velocity_rad_s;
                    tail_contact_square += contact_forward.powi(2);
                    tail_wheel_square += wheel_omega.powi(2);
                    tail_contact_peak = tail_contact_peak.max(contact_forward.abs());
                    tail_wheel_peak = tail_wheel_peak.max(wheel_omega.abs());
                    tail_samples += 1;
                    let sign = if wheel_omega > 0.02 {
                        1
                    } else if wheel_omega < -0.02 {
                        -1
                    } else {
                        0
                    };
                    if sign != 0 && wheel_signs[index] != 0 && sign != wheel_signs[index] {
                        wheel_sign_flips += 1;
                    }
                    if sign != 0 {
                        wheel_signs[index] = sign;
                    }
                }
            }
        }
        let vehicle = &world.vehicles[0];
        assert!(
            vehicle.state.linear_velocity_mps.length() < 0.05,
            "velocity={:?}, wheel={:?}, min_contact={minimum_contact_forward_speed}",
            vehicle.state.linear_velocity_mps,
            vehicle.state.wheels.map(|wheel| wheel.angular_velocity_rad_s)
        );
        assert!(minimum_contact_forward_speed > -0.05, "min_contact={minimum_contact_forward_speed}");
        let contact_rms = (tail_contact_square / tail_samples as f64).sqrt();
        let wheel_rms = (tail_wheel_square / tail_samples as f64).sqrt();
        assert!(contact_rms < 0.01 && tail_contact_peak < 0.03, "contact rms={contact_rms}, peak={tail_contact_peak}");
        assert!(wheel_rms < 0.02 && tail_wheel_peak < 0.1, "wheel rms={wheel_rms}, peak={tail_wheel_peak}");
        assert!(wheel_sign_flips <= 2, "wheel sign flips={wheel_sign_flips}");
        assert!(
            vehicle.state.wheels.iter().all(|wheel| wheel.angular_velocity_rad_s.abs() < 0.2),
            "wheel omega={:?}",
            vehicle.state.wheels.map(|wheel| wheel.angular_velocity_rad_s)
        );
    }

    fn body_acceleration_channels_for_test(world: &PhysicsWorld) -> [f64; 2] {
        let vehicle = &world.vehicles[0];
        let body = vehicle.state.orientation.conjugate().rotate(vehicle.telemetry.acceleration_mps2);
        [-body.z, body.x]
    }
}
