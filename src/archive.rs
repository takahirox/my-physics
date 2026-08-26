//! Versioned canonical binary archives for complete snapshots and timed input
//! histories. Encoding is dependency-free, little-endian, length-bounded and
//! protected by an FNV-1a checksum.

use core::fmt;

use crate::collision::{CollisionShape, DetachedBody, StaticCollider};
use crate::controls::{ControlOutput, DriverAids, DriverInput};
use crate::feedback::{AudioFrame, FeedbackEvent, FeedbackEventKind, ForceFeedbackFrame};
use crate::math::{Quat, Vec3};
use crate::provenance::{ParameterOrigin, ParameterProvenance, ParameterValidity, VehicleParameterProvenance};
use crate::road::{DynamicRoad, RoadCell};
use crate::tire::{MagicFormulaTire, TireFailure, TireOutput, TireState};
use crate::vehicle::{
    ChassisDefinition, DamageState, EngineDefinition, PowertrainState, Telemetry, TransmissionDefinition, Vehicle,
    VehicleDefinition, VehicleState, WheelDefinition, WheelState,
};
use crate::world::{Fidelity, GroundSurface, InputFrame, SimulationConfig, Snapshot};

const SNAPSHOT_MAGIC: &[u8; 8] = b"MYPHY001";
const INPUT_MAGIC: &[u8; 8] = b"MYINP001";
const VERSION: u32 = 5;
const INPUT_VERSION: u32 = 2;
const MAX_ITEMS: usize = 1_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArchiveError {
    BadMagic,
    UnsupportedVersion,
    Truncated,
    ChecksumMismatch,
    InvalidData,
    TrailingData,
}

impl fmt::Display for ArchiveError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(match self {
            Self::BadMagic => "invalid archive magic",
            Self::UnsupportedVersion => "unsupported archive version",
            Self::Truncated => "truncated archive",
            Self::ChecksumMismatch => "archive checksum mismatch",
            Self::InvalidData => "invalid archive data",
            Self::TrailingData => "unexpected trailing archive data",
        })
    }
}

impl std::error::Error for ArchiveError {}

struct Writer {
    bytes: Vec<u8>,
    version: u32,
}

impl Writer {
    fn with_version(magic: &[u8; 8], version: u32) -> Self {
        debug_assert!((1..=VERSION).contains(&version));
        let mut writer = Self { bytes: Vec::new(), version };
        writer.bytes.extend_from_slice(magic);
        writer.u32(version);
        writer
    }
    fn u8(&mut self, value: u8) {
        self.bytes.push(value);
    }
    fn bool(&mut self, value: bool) {
        self.u8(u8::from(value));
    }
    fn u32(&mut self, value: u32) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }
    fn u64(&mut self, value: u64) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }
    fn usize(&mut self, value: usize) {
        self.u32(u32::try_from(value).expect("archive collection fits u32"));
    }
    fn f64(&mut self, value: f64) {
        self.u64(value.to_bits());
    }
    fn string(&mut self, value: &str) {
        self.usize(value.len());
        self.bytes.extend_from_slice(value.as_bytes());
    }
    fn finish(mut self) -> Vec<u8> {
        let checksum = checksum(&self.bytes);
        self.u64(checksum);
        self.bytes
    }
}

struct Reader<'a> {
    bytes: &'a [u8],
    position: usize,
    version: u32,
}

impl<'a> Reader<'a> {
    fn new(bytes: &'a [u8], magic: &[u8; 8]) -> Result<Self, ArchiveError> {
        if bytes.len() < 20 {
            return Err(ArchiveError::Truncated);
        }
        let payload_length = bytes.len() - 8;
        let expected = u64::from_le_bytes(bytes[payload_length..].try_into().map_err(|_| ArchiveError::Truncated)?);
        if checksum(&bytes[..payload_length]) != expected {
            return Err(ArchiveError::ChecksumMismatch);
        }
        if &bytes[..8] != magic {
            return Err(ArchiveError::BadMagic);
        }
        let mut reader = Self { bytes: &bytes[..payload_length], position: 8, version: 0 };
        let version = reader.u32()?;
        if !(1..=VERSION).contains(&version) {
            return Err(ArchiveError::UnsupportedVersion);
        }
        reader.version = version;
        Ok(reader)
    }
    fn take(&mut self, length: usize) -> Result<&'a [u8], ArchiveError> {
        let end = self.position.checked_add(length).ok_or(ArchiveError::InvalidData)?;
        if end > self.bytes.len() {
            return Err(ArchiveError::Truncated);
        }
        let result = &self.bytes[self.position..end];
        self.position = end;
        Ok(result)
    }
    fn u8(&mut self) -> Result<u8, ArchiveError> {
        Ok(self.take(1)?[0])
    }
    fn bool(&mut self) -> Result<bool, ArchiveError> {
        match self.u8()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(ArchiveError::InvalidData),
        }
    }
    fn u32(&mut self) -> Result<u32, ArchiveError> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().map_err(|_| ArchiveError::Truncated)?))
    }
    fn u64(&mut self) -> Result<u64, ArchiveError> {
        Ok(u64::from_le_bytes(self.take(8)?.try_into().map_err(|_| ArchiveError::Truncated)?))
    }
    fn usize(&mut self) -> Result<usize, ArchiveError> {
        let value = self.u32()? as usize;
        if value > MAX_ITEMS {
            return Err(ArchiveError::InvalidData);
        }
        Ok(value)
    }
    fn f64(&mut self) -> Result<f64, ArchiveError> {
        let value = f64::from_bits(self.u64()?);
        if !value.is_finite() {
            return Err(ArchiveError::InvalidData);
        }
        Ok(value)
    }
    fn string(&mut self) -> Result<String, ArchiveError> {
        let length = self.usize()?;
        String::from_utf8(self.take(length)?.to_vec()).map_err(|_| ArchiveError::InvalidData)
    }
    fn done(self) -> Result<(), ArchiveError> {
        if self.position == self.bytes.len() { Ok(()) } else { Err(ArchiveError::TrailingData) }
    }
}

pub(crate) fn encode_snapshot(snapshot: &Snapshot) -> Vec<u8> {
    encode_snapshot_version(snapshot, VERSION)
}

fn encode_snapshot_version(snapshot: &Snapshot, version: u32) -> Vec<u8> {
    let mut writer = Writer::with_version(SNAPSHOT_MAGIC, version);
    write_config(&mut writer, snapshot.config);
    writer.f64(snapshot.time_s);
    writer.u64(snapshot.step);
    write_road(&mut writer, &snapshot.road);
    write_vec3(&mut writer, snapshot.wind_mps);
    writer.f64(snapshot.rain_rate_m_s);
    write_tire_model(&mut writer, snapshot.tire_model);
    writer.usize(snapshot.vehicles.len());
    for vehicle in &snapshot.vehicles {
        write_vehicle(&mut writer, vehicle);
    }
    writer.usize(snapshot.static_colliders.len());
    for collider in &snapshot.static_colliders {
        write_static_collider(&mut writer, collider);
    }
    writer.usize(snapshot.detached_bodies.len());
    for body in &snapshot.detached_bodies {
        write_detached_body(&mut writer, body);
    }
    writer.finish()
}

pub(crate) fn decode_snapshot(bytes: &[u8]) -> Result<Snapshot, ArchiveError> {
    let mut reader = Reader::new(bytes, SNAPSHOT_MAGIC)?;
    let config = read_config(&mut reader)?;
    let time_s = reader.f64()?;
    let step = reader.u64()?;
    let road = read_road(&mut reader)?;
    let wind_mps = read_vec3(&mut reader)?;
    let rain_rate_m_s = reader.f64()?;
    let tire_model = read_tire_model(&mut reader)?;
    let vehicle_count = reader.usize()?;
    let mut vehicles = Vec::with_capacity(vehicle_count);
    for _ in 0..vehicle_count {
        vehicles.push(read_vehicle(&mut reader)?);
    }
    let collider_count = reader.usize()?;
    let mut static_colliders = Vec::with_capacity(collider_count);
    for _ in 0..collider_count {
        static_colliders.push(read_static_collider(&mut reader)?);
    }
    let body_count = reader.usize()?;
    let mut detached_bodies = Vec::with_capacity(body_count);
    for _ in 0..body_count {
        detached_bodies.push(read_detached_body(&mut reader)?);
    }
    reader.done()?;
    Ok(Snapshot {
        config,
        time_s,
        step,
        road,
        wind_mps,
        rain_rate_m_s,
        vehicles,
        static_colliders,
        detached_bodies,
        tire_model,
    })
}

pub fn encode_input_history(frames: &[InputFrame]) -> Vec<u8> {
    // The input-frame layout did not change with snapshot v3.
    let mut writer = Writer::with_version(INPUT_MAGIC, INPUT_VERSION);
    writer.usize(frames.len());
    for frame in frames {
        writer.u64(frame.step);
        writer.usize(frame.vehicle);
        write_driver_input(&mut writer, frame.input);
    }
    writer.finish()
}

pub fn decode_input_history(bytes: &[u8]) -> Result<Vec<InputFrame>, ArchiveError> {
    let mut reader = Reader::new(bytes, INPUT_MAGIC)?;
    let count = reader.usize()?;
    let mut frames = Vec::with_capacity(count);
    for _ in 0..count {
        frames.push(InputFrame {
            step: reader.u64()?,
            vehicle: reader.usize()?,
            input: read_driver_input(&mut reader)?,
        });
    }
    reader.done()?;
    Ok(frames)
}

fn checksum(bytes: &[u8]) -> u64 {
    let mut value = 0xcbf29ce484222325_u64;
    for byte in bytes {
        value ^= u64::from(*byte);
        value = value.wrapping_mul(0x100000001b3);
    }
    value
}

fn write_config(w: &mut Writer, value: SimulationConfig) {
    w.f64(value.fixed_dt_s);
    w.f64(value.gravity_mps2);
    w.f64(value.max_variable_dt_s);
    w.f64(value.lod_transition_s);
    w.usize(value.player_vehicle);
    w.bool(value.automatic_lod);
    write_fidelity(w, value.fidelity_ceiling);
    if w.version >= 5 {
        w.u8(match value.ground_surface {
            GroundSurface::Flat => 0,
            GroundSurface::DemoCircuit => 1,
        });
    }
}

fn read_config(r: &mut Reader<'_>) -> Result<SimulationConfig, ArchiveError> {
    let fixed_dt_s = r.f64()?;
    let gravity_mps2 = r.f64()?;
    let max_variable_dt_s = r.f64()?;
    let lod_transition_s = r.f64()?;
    let player_vehicle = r.usize()?;
    let automatic_lod = r.bool()?;
    let fidelity_ceiling = read_fidelity(r)?;
    let ground_surface = if r.version >= 5 {
        match r.u8()? {
            0 => GroundSurface::Flat,
            1 => GroundSurface::DemoCircuit,
            _ => return Err(ArchiveError::InvalidData),
        }
    } else {
        GroundSurface::Flat
    };
    Ok(SimulationConfig {
        fixed_dt_s,
        gravity_mps2,
        max_variable_dt_s,
        lod_transition_s,
        player_vehicle,
        automatic_lod,
        fidelity_ceiling,
        ground_surface,
    })
}

fn write_fidelity(w: &mut Writer, value: Fidelity) {
    w.u8(match value {
        Fidelity::Low => 0,
        Fidelity::Medium => 1,
        Fidelity::High => 2,
    });
}

fn read_fidelity(r: &mut Reader<'_>) -> Result<Fidelity, ArchiveError> {
    match r.u8()? {
        0 => Ok(Fidelity::Low),
        1 => Ok(Fidelity::Medium),
        2 => Ok(Fidelity::High),
        _ => Err(ArchiveError::InvalidData),
    }
}

fn write_road(w: &mut Writer, road: &DynamicRoad) {
    w.f64(road.origin_x);
    w.f64(road.origin_z);
    w.f64(road.cell_size_m);
    w.usize(road.width);
    w.usize(road.height);
    w.f64(road.ambient_temperature_k);
    w.f64(road.solar_heating_w_m2);
    w.usize(road.cells().len());
    for cell in road.cells() {
        write_road_cell(w, *cell);
    }
}

fn read_road(r: &mut Reader<'_>) -> Result<DynamicRoad, ArchiveError> {
    let origin_x = r.f64()?;
    let origin_z = r.f64()?;
    let cell_size_m = r.f64()?;
    let width = r.usize()?;
    let height = r.usize()?;
    if width == 0 || height == 0 || width.checked_mul(height).ok_or(ArchiveError::InvalidData)? > MAX_ITEMS {
        return Err(ArchiveError::InvalidData);
    }
    let ambient_temperature_k = r.f64()?;
    let solar_heating_w_m2 = r.f64()?;
    let count = r.usize()?;
    if count != width * height {
        return Err(ArchiveError::InvalidData);
    }
    let mut cells = Vec::with_capacity(count);
    for _ in 0..count {
        cells.push(read_road_cell(r)?);
    }
    let mut road = DynamicRoad::new(width, height, cell_size_m);
    road.origin_x = origin_x;
    road.origin_z = origin_z;
    road.ambient_temperature_k = ambient_temperature_k;
    road.solar_heating_w_m2 = solar_heating_w_m2;
    if !road.replace_cells(cells) {
        return Err(ArchiveError::InvalidData);
    }
    Ok(road)
}

fn write_road_cell(w: &mut Writer, value: RoadCell) {
    for number in [value.temperature_k, value.rubber, value.water_depth_m, value.contamination] {
        w.f64(number);
    }
}

fn read_road_cell(r: &mut Reader<'_>) -> Result<RoadCell, ArchiveError> {
    Ok(RoadCell { temperature_k: r.f64()?, rubber: r.f64()?, water_depth_m: r.f64()?, contamination: r.f64()? })
}

fn write_vehicle(w: &mut Writer, vehicle: &Vehicle) {
    write_vehicle_definition(w, &vehicle.definition);
    write_vehicle_state(w, &vehicle.state);
    w.bool(vehicle.driver_aids.abs_enabled);
    w.bool(vehicle.driver_aids.traction_control_enabled);
    w.bool(vehicle.driver_aids.stability_control_enabled);
    if w.version >= 2 {
        for pressure in vehicle.driver_aids.abs_pressure() {
            w.f64(pressure);
        }
    }
    w.f64(vehicle.driver_aids.integrator());
    write_driver_input(w, vehicle.input);
    write_control_output(w, vehicle.control);
    write_telemetry(w, &vehicle.telemetry);
    w.f64(vehicle.fidelity);
    w.f64(vehicle.target_fidelity);
    write_vec3(w, vehicle.previous_position_m);
    write_quat(w, vehicle.previous_orientation);
    write_audio(w, vehicle.audio);
    write_ffb(w, vehicle.force_feedback);
    w.usize(vehicle.events.len());
    for event in &vehicle.events {
        write_event(w, *event);
    }
    write_vec3(w, vehicle.cached_force);
    write_vec3(w, vehicle.cached_torque);
    w.u8(vehicle.previous_gear as u8);
    for failure in vehicle.previous_tire_failures {
        write_tire_failure(w, failure);
    }
    w.bool(vehicle.previous_engine_failed);
    w.bool(vehicle.previous_clutch_failed);
    w.bool(vehicle.previous_gearbox_failed);
}

fn read_vehicle(r: &mut Reader<'_>) -> Result<Vehicle, ArchiveError> {
    let definition = read_vehicle_definition(r)?;
    let state = read_vehicle_state(r)?;
    let mut driver_aids = DriverAids::default();
    driver_aids.abs_enabled = r.bool()?;
    driver_aids.traction_control_enabled = r.bool()?;
    driver_aids.stability_control_enabled = r.bool()?;
    if r.version >= 2 {
        driver_aids.set_abs_pressure([r.f64()?, r.f64()?, r.f64()?, r.f64()?]);
    }
    driver_aids.set_integrator(r.f64()?);
    let input = read_driver_input(r)?;
    let control = read_control_output(r)?;
    let telemetry = read_telemetry(r)?;
    let fidelity = r.f64()?;
    let target_fidelity = r.f64()?;
    let previous_position_m = read_vec3(r)?;
    let previous_orientation = read_quat(r)?;
    let audio = read_audio(r)?;
    let force_feedback = read_ffb(r)?;
    let event_count = r.usize()?;
    let mut events = Vec::with_capacity(event_count);
    for _ in 0..event_count {
        events.push(read_event(r)?);
    }
    let cached_force = read_vec3(r)?;
    let cached_torque = read_vec3(r)?;
    let previous_gear = r.u8()? as i8;
    let previous_tire_failures =
        [read_tire_failure(r)?, read_tire_failure(r)?, read_tire_failure(r)?, read_tire_failure(r)?];
    let previous_engine_failed = r.bool()?;
    let previous_clutch_failed = r.bool()?;
    let previous_gearbox_failed = r.bool()?;
    Ok(Vehicle {
        definition,
        state,
        driver_aids,
        input,
        control,
        telemetry,
        fidelity,
        target_fidelity,
        previous_position_m,
        previous_orientation,
        audio,
        force_feedback,
        events,
        cached_force,
        cached_torque,
        previous_gear,
        previous_tire_failures,
        previous_engine_failed,
        previous_clutch_failed,
        previous_gearbox_failed,
    })
}

fn write_vehicle_definition(w: &mut Writer, value: &VehicleDefinition) {
    w.string(&value.name);
    write_chassis_definition(w, value.chassis);
    for wheel in value.wheels {
        write_wheel_definition(w, wheel);
    }
    write_engine_definition(w, &value.engine);
    write_transmission_definition(w, &value.transmission);
    w.f64(value.fuel_capacity_kg);
    write_vec3(w, value.fuel_tank_local_m);
    w.f64(value.anti_roll_rate_n_m_rad);
    if w.version >= 3 {
        write_vehicle_provenance(w, &value.provenance);
    }
}

fn read_vehicle_definition(r: &mut Reader<'_>) -> Result<VehicleDefinition, ArchiveError> {
    let mut definition = VehicleDefinition {
        name: r.string()?,
        chassis: read_chassis_definition(r)?,
        wheels: [
            read_wheel_definition(r)?,
            read_wheel_definition(r)?,
            read_wheel_definition(r)?,
            read_wheel_definition(r)?,
        ],
        engine: read_engine_definition(r)?,
        transmission: read_transmission_definition(r)?,
        fuel_capacity_kg: r.f64()?,
        fuel_tank_local_m: read_vec3(r)?,
        anti_roll_rate_n_m_rad: r.f64()?,
        provenance: VehicleParameterProvenance::legacy_archive(r.version),
    };
    if r.version >= 3 {
        definition.provenance = read_vehicle_provenance(r)?;
    }
    Ok(definition)
}

fn write_vehicle_provenance(w: &mut Writer, value: &VehicleParameterProvenance) {
    for (_, group) in value.groups() {
        write_parameter_provenance(w, group);
    }
}

fn read_vehicle_provenance(r: &mut Reader<'_>) -> Result<VehicleParameterProvenance, ArchiveError> {
    let value = VehicleParameterProvenance {
        chassis_mass_properties: read_parameter_provenance(r)?,
        aerodynamics: read_parameter_provenance(r)?,
        front_wheels_and_tires: read_parameter_provenance(r)?,
        rear_wheels_and_tires: read_parameter_provenance(r)?,
        suspension: read_parameter_provenance(r)?,
        brakes: read_parameter_provenance(r)?,
        engine: read_parameter_provenance(r)?,
        transmission_and_clutch: read_parameter_provenance(r)?,
        fuel_system: read_parameter_provenance(r)?,
    };
    if !value.is_complete() {
        return Err(ArchiveError::InvalidData);
    }
    Ok(value)
}

fn write_parameter_provenance(w: &mut Writer, value: &ParameterProvenance) {
    w.u8(match value.origin {
        ParameterOrigin::Measured => 0,
        ParameterOrigin::Derived => 1,
        ParameterOrigin::Fitted => 2,
        ParameterOrigin::Estimated => 3,
        ParameterOrigin::Authored => 4,
    });
    w.string(&value.source);
    w.string(&value.revision);
    w.bool(value.uncertainty_fraction.is_some());
    if let Some(uncertainty) = value.uncertainty_fraction {
        w.f64(uncertainty);
    }
    w.usize(value.valid_ranges.len());
    for range in &value.valid_ranges {
        w.string(&range.parameter);
        w.string(&range.unit);
        w.f64(range.minimum);
        w.f64(range.maximum);
    }
}

fn read_parameter_provenance(r: &mut Reader<'_>) -> Result<ParameterProvenance, ArchiveError> {
    let origin = match r.u8()? {
        0 => ParameterOrigin::Measured,
        1 => ParameterOrigin::Derived,
        2 => ParameterOrigin::Fitted,
        3 => ParameterOrigin::Estimated,
        4 => ParameterOrigin::Authored,
        _ => return Err(ArchiveError::InvalidData),
    };
    let source = r.string()?;
    let revision = r.string()?;
    let uncertainty_fraction = if r.bool()? { Some(r.f64()?) } else { None };
    let range_count = r.usize()?;
    let mut valid_ranges = Vec::with_capacity(range_count);
    for _ in 0..range_count {
        valid_ranges.push(ParameterValidity {
            parameter: r.string()?,
            unit: r.string()?,
            minimum: r.f64()?,
            maximum: r.f64()?,
        });
    }
    let value = ParameterProvenance { origin, source, revision, uncertainty_fraction, valid_ranges };
    if !value.is_complete() {
        return Err(ArchiveError::InvalidData);
    }
    Ok(value)
}

fn write_chassis_definition(w: &mut Writer, value: ChassisDefinition) {
    w.f64(value.dry_mass_kg);
    write_vec3(w, value.cg_local_m);
    write_vec3(w, value.inertia_kg_m2);
    for number in [value.frontal_area_m2, value.drag_coefficient, value.lift_coefficient, value.air_density_kg_m3] {
        w.f64(number);
    }
}

fn read_chassis_definition(r: &mut Reader<'_>) -> Result<ChassisDefinition, ArchiveError> {
    Ok(ChassisDefinition {
        dry_mass_kg: r.f64()?,
        cg_local_m: read_vec3(r)?,
        inertia_kg_m2: read_vec3(r)?,
        frontal_area_m2: r.f64()?,
        drag_coefficient: r.f64()?,
        lift_coefficient: r.f64()?,
        air_density_kg_m3: r.f64()?,
    })
}

fn write_wheel_definition(w: &mut Writer, value: WheelDefinition) {
    write_vec3(w, value.mount_local_m);
    for number in [
        value.radius_m,
        value.inertia_kg_m2,
        value.mass_kg,
        value.spring_rate_n_m,
        value.damper_rate_n_s_m,
        value.rest_length_m,
        value.max_travel_m,
        value.bump_stop_rate_n_m,
        value.max_steer_rad,
    ] {
        w.f64(number);
    }
    w.bool(value.driven);
    w.f64(value.brake_torque_nm);
    if w.version >= 2 {
        w.f64(value.cornering_stiffness_scale);
        w.f64(value.tire_peak_grip_scale);
    }
}

fn read_wheel_definition(r: &mut Reader<'_>) -> Result<WheelDefinition, ArchiveError> {
    let mount_local_m = read_vec3(r)?;
    let radius_m = r.f64()?;
    let inertia_kg_m2 = r.f64()?;
    let mass_kg = r.f64()?;
    let spring_rate_n_m = r.f64()?;
    let damper_rate_n_s_m = r.f64()?;
    let rest_length_m = r.f64()?;
    let max_travel_m = r.f64()?;
    let bump_stop_rate_n_m = r.f64()?;
    let max_steer_rad = r.f64()?;
    let driven = r.bool()?;
    let brake_torque_nm = r.f64()?;
    let cornering_stiffness_scale = if r.version >= 2 { r.f64()? } else { 1.0 };
    let tire_peak_grip_scale = if r.version >= 2 { r.f64()? } else { 1.0 };
    Ok(WheelDefinition {
        mount_local_m,
        radius_m,
        inertia_kg_m2,
        mass_kg,
        spring_rate_n_m,
        damper_rate_n_s_m,
        rest_length_m,
        max_travel_m,
        bump_stop_rate_n_m,
        max_steer_rad,
        driven,
        brake_torque_nm,
        cornering_stiffness_scale,
        tire_peak_grip_scale,
    })
}

fn write_engine_definition(w: &mut Writer, value: &EngineDefinition) {
    for number in [value.idle_rpm, value.redline_rpm, value.inertia_kg_m2] {
        w.f64(number);
    }
    for (rpm, torque) in value.torque_curve {
        w.f64(rpm);
        w.f64(torque);
    }
    w.f64(value.fuel_energy_j_kg);
    w.f64(value.efficiency);
}

fn read_engine_definition(r: &mut Reader<'_>) -> Result<EngineDefinition, ArchiveError> {
    let idle_rpm = r.f64()?;
    let redline_rpm = r.f64()?;
    let inertia_kg_m2 = r.f64()?;
    let mut torque_curve = [(0.0, 0.0); 8];
    for point in &mut torque_curve {
        *point = (r.f64()?, r.f64()?);
    }
    Ok(EngineDefinition {
        idle_rpm,
        redline_rpm,
        inertia_kg_m2,
        torque_curve,
        fuel_energy_j_kg: r.f64()?,
        efficiency: r.f64()?,
    })
}

fn write_transmission_definition(w: &mut Writer, value: &TransmissionDefinition) {
    w.bool(value.automatic);
    for ratio in value.gear_ratios {
        w.f64(ratio);
    }
    for number in [
        value.reverse_ratio,
        value.final_drive,
        value.shift_time_s,
        value.clutch_capacity_nm,
        value.clutch_stiffness_nm_per_rad_s,
    ] {
        w.f64(number);
    }
}

fn read_transmission_definition(r: &mut Reader<'_>) -> Result<TransmissionDefinition, ArchiveError> {
    let automatic = r.bool()?;
    let mut gear_ratios = [0.0; 7];
    for ratio in &mut gear_ratios {
        *ratio = r.f64()?;
    }
    Ok(TransmissionDefinition {
        automatic,
        gear_ratios,
        reverse_ratio: r.f64()?,
        final_drive: r.f64()?,
        shift_time_s: r.f64()?,
        clutch_capacity_nm: r.f64()?,
        clutch_stiffness_nm_per_rad_s: r.f64()?,
    })
}

fn write_vehicle_state(w: &mut Writer, value: &VehicleState) {
    write_vec3(w, value.position_m);
    write_quat(w, value.orientation);
    write_vec3(w, value.linear_velocity_mps);
    write_vec3(w, value.angular_velocity_rad_s);
    for wheel in value.wheels {
        write_wheel_state(w, wheel);
    }
    write_powertrain_state(w, value.powertrain);
    write_damage_state(w, value.damage);
    w.f64(value.simulation_time_s);
}

fn read_vehicle_state(r: &mut Reader<'_>) -> Result<VehicleState, ArchiveError> {
    Ok(VehicleState {
        position_m: read_vec3(r)?,
        orientation: read_quat(r)?,
        linear_velocity_mps: read_vec3(r)?,
        angular_velocity_rad_s: read_vec3(r)?,
        wheels: [read_wheel_state(r)?, read_wheel_state(r)?, read_wheel_state(r)?, read_wheel_state(r)?],
        powertrain: read_powertrain_state(r)?,
        damage: read_damage_state(r)?,
        simulation_time_s: r.f64()?,
    })
}

fn write_wheel_state(w: &mut Writer, value: WheelState) {
    for number in [
        value.angular_velocity_rad_s,
        value.rotation_rad,
        value.suspension_compression_m,
        value.previous_compression_m,
        value.steer_angle_rad,
        value.camber_rad,
        value.brake_temperature_k,
        value.brake_wear,
        value.wheel_damage,
    ] {
        w.f64(number);
    }
    write_tire_state(w, value.tire);
    write_tire_output(w, value.last_tire_output);
    for number in [value.last_normal_load_n, value.longitudinal_slip, value.slip_angle_rad] {
        w.f64(number);
    }
    if w.version >= 4 {
        w.f64(value.transient_slip_angle_rad);
        w.f64(value.relaxation_length_m);
    }
}

fn read_wheel_state(r: &mut Reader<'_>) -> Result<WheelState, ArchiveError> {
    let angular_velocity_rad_s = r.f64()?;
    let rotation_rad = r.f64()?;
    let suspension_compression_m = r.f64()?;
    let previous_compression_m = r.f64()?;
    let steer_angle_rad = r.f64()?;
    let camber_rad = r.f64()?;
    let brake_temperature_k = r.f64()?;
    let brake_wear = r.f64()?;
    let wheel_damage = r.f64()?;
    let tire = read_tire_state(r)?;
    let last_tire_output = read_tire_output(r)?;
    let last_normal_load_n = r.f64()?;
    let longitudinal_slip = r.f64()?;
    let slip_angle_rad = r.f64()?;
    let (transient_slip_angle_rad, relaxation_length_m) = if r.version >= 4 {
        (r.f64()?, r.f64()?)
    } else {
        (slip_angle_rad, MagicFormulaTire::default().relaxation_length_m)
    };
    Ok(WheelState {
        angular_velocity_rad_s,
        rotation_rad,
        suspension_compression_m,
        previous_compression_m,
        steer_angle_rad,
        camber_rad,
        brake_temperature_k,
        brake_wear,
        wheel_damage,
        tire,
        last_tire_output,
        last_normal_load_n,
        longitudinal_slip,
        slip_angle_rad,
        transient_slip_angle_rad,
        relaxation_length_m,
    })
}

fn write_tire_state(w: &mut Writer, value: TireState) {
    for number in [value.temperature_k, value.tread_temperature_k, value.wear, value.pressure_pa] {
        w.f64(number);
    }
    write_tire_failure(w, value.failure);
    for number in [value.puncture_area_m2, value.carcass_damage, value.contact_patch_m2] {
        w.f64(number);
    }
}

fn read_tire_state(r: &mut Reader<'_>) -> Result<TireState, ArchiveError> {
    Ok(TireState {
        temperature_k: r.f64()?,
        tread_temperature_k: r.f64()?,
        wear: r.f64()?,
        pressure_pa: r.f64()?,
        failure: read_tire_failure(r)?,
        puncture_area_m2: r.f64()?,
        carcass_damage: r.f64()?,
        contact_patch_m2: r.f64()?,
    })
}

fn write_tire_failure(w: &mut Writer, value: TireFailure) {
    w.u8(match value {
        TireFailure::Healthy => 0,
        TireFailure::Punctured => 1,
        TireFailure::Blowout => 2,
        TireFailure::BeadUnseated => 3,
    });
}

fn read_tire_failure(r: &mut Reader<'_>) -> Result<TireFailure, ArchiveError> {
    match r.u8()? {
        0 => Ok(TireFailure::Healthy),
        1 => Ok(TireFailure::Punctured),
        2 => Ok(TireFailure::Blowout),
        3 => Ok(TireFailure::BeadUnseated),
        _ => Err(ArchiveError::InvalidData),
    }
}

fn write_tire_output(w: &mut Writer, value: TireOutput) {
    for number in [
        value.longitudinal_force_n,
        value.lateral_force_n,
        value.aligning_moment_nm,
        value.rolling_resistance_n,
        value.hydroplaning,
        value.friction_coefficient,
        value.slip_power_w,
    ] {
        w.f64(number);
    }
    if w.version >= 4 {
        w.f64(value.road_heat_w);
    }
}

fn read_tire_output(r: &mut Reader<'_>) -> Result<TireOutput, ArchiveError> {
    let mut value = TireOutput {
        longitudinal_force_n: r.f64()?,
        lateral_force_n: r.f64()?,
        aligning_moment_nm: r.f64()?,
        rolling_resistance_n: r.f64()?,
        hydroplaning: r.f64()?,
        friction_coefficient: r.f64()?,
        slip_power_w: r.f64()?,
        road_heat_w: 0.0,
    };
    if r.version >= 4 {
        value.road_heat_w = r.f64()?;
    }
    Ok(value)
}

fn write_powertrain_state(w: &mut Writer, value: PowertrainState) {
    w.f64(value.engine_rpm);
    w.f64(value.throttle_actual);
    w.u8(value.gear as u8);
    for number in [
        value.shift_timer_s,
        value.clutch_engagement,
        value.clutch_temperature_k,
        value.clutch_wear,
        value.gearbox_wear,
    ] {
        w.f64(number);
    }
    w.bool(value.clutch_failed);
    w.bool(value.gearbox_failed);
    for number in [
        value.fuel_kg,
        value.engine_temperature_k,
        value.oil_temperature_k,
        value.coolant_temperature_k,
        value.oil_pressure_pa,
        value.overheat_damage,
        value.oil_damage,
        value.overrev_damage,
    ] {
        w.f64(number);
    }
    w.bool(value.failed);
}

fn read_powertrain_state(r: &mut Reader<'_>) -> Result<PowertrainState, ArchiveError> {
    Ok(PowertrainState {
        engine_rpm: r.f64()?,
        throttle_actual: r.f64()?,
        gear: r.u8()? as i8,
        shift_timer_s: r.f64()?,
        clutch_engagement: r.f64()?,
        clutch_temperature_k: r.f64()?,
        clutch_wear: r.f64()?,
        gearbox_wear: r.f64()?,
        clutch_failed: r.bool()?,
        gearbox_failed: r.bool()?,
        fuel_kg: r.f64()?,
        engine_temperature_k: r.f64()?,
        oil_temperature_k: r.f64()?,
        coolant_temperature_k: r.f64()?,
        oil_pressure_pa: r.f64()?,
        overheat_damage: r.f64()?,
        oil_damage: r.f64()?,
        overrev_damage: r.f64()?,
        failed: r.bool()?,
    })
}

fn write_damage_state(w: &mut Writer, value: DamageState) {
    w.f64(value.body);
    w.f64(value.aero);
    for damage in value.suspension {
        w.f64(damage);
    }
    write_vec3(w, value.deformation_local_m);
    w.f64(value.detached_mass_kg);
}

fn read_damage_state(r: &mut Reader<'_>) -> Result<DamageState, ArchiveError> {
    Ok(DamageState {
        body: r.f64()?,
        aero: r.f64()?,
        suspension: read_f64_array(r)?,
        deformation_local_m: read_vec3(r)?,
        detached_mass_kg: r.f64()?,
    })
}

fn write_driver_input(w: &mut Writer, value: DriverInput) {
    for number in [value.steering, value.throttle, value.brake, value.clutch, value.handbrake] {
        w.f64(number);
    }
    w.u8(value.gear_request as u8);
}

fn read_driver_input(r: &mut Reader<'_>) -> Result<DriverInput, ArchiveError> {
    Ok(DriverInput {
        steering: r.f64()?,
        throttle: r.f64()?,
        brake: r.f64()?,
        clutch: r.f64()?,
        handbrake: r.f64()?,
        gear_request: r.u8()? as i8,
    })
}

fn write_control_output(w: &mut Writer, value: ControlOutput) {
    w.f64(value.steering);
    w.f64(value.throttle);
    write_f64_array(w, value.brake_per_wheel);
    w.f64(value.clutch);
    w.u8(value.gear_request as u8);
    write_bool_array(w, value.abs_active);
    w.bool(value.tc_active);
    w.bool(value.esc_active);
}

fn read_control_output(r: &mut Reader<'_>) -> Result<ControlOutput, ArchiveError> {
    Ok(ControlOutput {
        steering: r.f64()?,
        throttle: r.f64()?,
        brake_per_wheel: read_f64_array(r)?,
        clutch: r.f64()?,
        gear_request: r.u8()? as i8,
        abs_active: read_bool_array(r)?,
        tc_active: r.bool()?,
        esc_active: r.bool()?,
    })
}

fn write_telemetry(w: &mut Writer, value: &Telemetry) {
    w.f64(value.time_s);
    write_vec3(w, value.position_m);
    w.f64(value.speed_mps);
    write_vec3(w, value.acceleration_mps2);
    w.f64(value.yaw_rate_rad_s);
    w.f64(value.engine_rpm);
    w.u8(value.gear as u8);
    for number in [
        value.fuel_kg,
        value.engine_temperature_k,
        value.oil_pressure_pa,
        value.clutch_temperature_k,
        value.clutch_wear,
        value.gearbox_wear,
    ] {
        w.f64(number);
    }
    for values in [
        value.wheel_slip,
        value.tire_temperature_k,
        value.tire_pressure_pa,
        value.tire_wear,
        value.normal_load_n,
        value.brake_temperature_k,
        value.hydroplaning,
    ] {
        write_f64_array(w, values);
    }
    write_bool_array(w, value.abs_active);
    w.bool(value.tc_active);
    w.bool(value.esc_active);
    w.f64(value.body_damage);
    w.f64(value.fidelity);
}

fn read_telemetry(r: &mut Reader<'_>) -> Result<Telemetry, ArchiveError> {
    Ok(Telemetry {
        time_s: r.f64()?,
        position_m: read_vec3(r)?,
        speed_mps: r.f64()?,
        acceleration_mps2: read_vec3(r)?,
        yaw_rate_rad_s: r.f64()?,
        engine_rpm: r.f64()?,
        gear: r.u8()? as i8,
        fuel_kg: r.f64()?,
        engine_temperature_k: r.f64()?,
        oil_pressure_pa: r.f64()?,
        clutch_temperature_k: r.f64()?,
        clutch_wear: r.f64()?,
        gearbox_wear: r.f64()?,
        wheel_slip: read_f64_array(r)?,
        tire_temperature_k: read_f64_array(r)?,
        tire_pressure_pa: read_f64_array(r)?,
        tire_wear: read_f64_array(r)?,
        normal_load_n: read_f64_array(r)?,
        brake_temperature_k: read_f64_array(r)?,
        hydroplaning: read_f64_array(r)?,
        abs_active: read_bool_array(r)?,
        tc_active: r.bool()?,
        esc_active: r.bool()?,
        body_damage: r.f64()?,
        fidelity: r.f64()?,
    })
}

fn write_audio(w: &mut Writer, value: AudioFrame) {
    for number in [value.engine_rpm, value.engine_load, value.intake, value.exhaust] {
        w.f64(number);
    }
    write_f64_array(w, value.tire_scrub);
    write_f64_array(w, value.road_noise);
    write_f64_array(w, value.suspension_activity);
    w.f64(value.wind);
    w.f64(value.impact);
}

fn read_audio(r: &mut Reader<'_>) -> Result<AudioFrame, ArchiveError> {
    Ok(AudioFrame {
        engine_rpm: r.f64()?,
        engine_load: r.f64()?,
        intake: r.f64()?,
        exhaust: r.f64()?,
        tire_scrub: read_f64_array(r)?,
        road_noise: read_f64_array(r)?,
        suspension_activity: read_f64_array(r)?,
        wind: r.f64()?,
        impact: r.f64()?,
    })
}

fn write_ffb(w: &mut Writer, value: ForceFeedbackFrame) {
    for number in [
        value.steering_torque_nm,
        value.aligning_moment_nm,
        value.rack_force_n,
        value.road_vibration,
        value.tire_scrub,
        value.abs_pulse,
        value.impact,
    ] {
        w.f64(number);
    }
}

fn read_ffb(r: &mut Reader<'_>) -> Result<ForceFeedbackFrame, ArchiveError> {
    Ok(ForceFeedbackFrame {
        steering_torque_nm: r.f64()?,
        aligning_moment_nm: r.f64()?,
        rack_force_n: r.f64()?,
        road_vibration: r.f64()?,
        tire_scrub: r.f64()?,
        abs_pulse: r.f64()?,
        impact: r.f64()?,
    })
}

fn write_event(w: &mut Writer, value: FeedbackEvent) {
    w.f64(value.time_s);
    w.u8(match value.kind {
        FeedbackEventKind::GearShift => 0,
        FeedbackEventKind::Impact => 1,
        FeedbackEventKind::TireFailure => 2,
        FeedbackEventKind::EngineFailure => 3,
        FeedbackEventKind::ClutchFailure => 4,
        FeedbackEventKind::GearboxFailure => 5,
    });
    w.f64(value.magnitude);
    w.u8(value.wheel.unwrap_or(u8::MAX));
}

fn read_event(r: &mut Reader<'_>) -> Result<FeedbackEvent, ArchiveError> {
    let time_s = r.f64()?;
    let kind = match r.u8()? {
        0 => FeedbackEventKind::GearShift,
        1 => FeedbackEventKind::Impact,
        2 => FeedbackEventKind::TireFailure,
        3 => FeedbackEventKind::EngineFailure,
        4 => FeedbackEventKind::ClutchFailure,
        5 => FeedbackEventKind::GearboxFailure,
        _ => return Err(ArchiveError::InvalidData),
    };
    let magnitude = r.f64()?;
    let wheel = match r.u8()? {
        u8::MAX => None,
        value @ 0..=3 => Some(value),
        _ => return Err(ArchiveError::InvalidData),
    };
    Ok(FeedbackEvent { time_s, kind, magnitude, wheel })
}

fn write_tire_model(w: &mut Writer, value: MagicFormulaTire) {
    for number in [
        value.nominal_load_n,
        value.nominal_pressure_pa,
        value.peak_mu,
        value.longitudinal_stiffness,
        value.lateral_stiffness,
        value.optimum_temperature_k,
    ] {
        w.f64(number);
    }
    if w.version >= 4 {
        for number in [
            value.lateral_shape_factor,
            value.lateral_curvature_factor,
            value.pneumatic_trail_m,
            value.relaxation_length_m,
            value.tread_heat_capacity_j_k,
            value.bulk_heat_capacity_j_k,
            value.slip_heat_fraction_to_tread,
            value.tread_bulk_conductance_w_k,
            value.tread_road_conductance_w_k,
            value.still_air_conductance_w_k,
            value.speed_air_conductance_w_k_per_mps,
        ] {
            w.f64(number);
        }
    }
}

fn read_tire_model(r: &mut Reader<'_>) -> Result<MagicFormulaTire, ArchiveError> {
    let mut value = MagicFormulaTire {
        nominal_load_n: r.f64()?,
        nominal_pressure_pa: r.f64()?,
        peak_mu: r.f64()?,
        longitudinal_stiffness: r.f64()?,
        lateral_stiffness: r.f64()?,
        optimum_temperature_k: r.f64()?,
        ..MagicFormulaTire::default()
    };
    if r.version >= 4 {
        value.lateral_shape_factor = r.f64()?;
        value.lateral_curvature_factor = r.f64()?;
        value.pneumatic_trail_m = r.f64()?;
        value.relaxation_length_m = r.f64()?;
        value.tread_heat_capacity_j_k = r.f64()?;
        value.bulk_heat_capacity_j_k = r.f64()?;
        value.slip_heat_fraction_to_tread = r.f64()?;
        value.tread_bulk_conductance_w_k = r.f64()?;
        value.tread_road_conductance_w_k = r.f64()?;
        value.still_air_conductance_w_k = r.f64()?;
        value.speed_air_conductance_w_k_per_mps = r.f64()?;
    }
    Ok(value)
}

fn write_static_collider(w: &mut Writer, value: &StaticCollider) {
    write_vec3(w, value.position_m);
    write_quat(w, value.orientation);
    write_shape(w, &value.shape);
    w.f64(value.restitution);
    w.f64(value.friction);
}

fn read_static_collider(r: &mut Reader<'_>) -> Result<StaticCollider, ArchiveError> {
    Ok(StaticCollider {
        position_m: read_vec3(r)?,
        orientation: read_quat(r)?,
        shape: read_shape(r)?,
        restitution: r.f64()?,
        friction: r.f64()?,
    })
}

fn write_detached_body(w: &mut Writer, value: &DetachedBody) {
    write_vec3(w, value.position_m);
    write_quat(w, value.orientation);
    write_vec3(w, value.linear_velocity_mps);
    write_vec3(w, value.angular_velocity_rad_s);
    w.f64(value.mass_kg);
    write_shape(w, &value.shape);
    w.f64(value.damage);
}

fn read_detached_body(r: &mut Reader<'_>) -> Result<DetachedBody, ArchiveError> {
    Ok(DetachedBody {
        position_m: read_vec3(r)?,
        orientation: read_quat(r)?,
        linear_velocity_mps: read_vec3(r)?,
        angular_velocity_rad_s: read_vec3(r)?,
        mass_kg: r.f64()?,
        shape: read_shape(r)?,
        damage: r.f64()?,
    })
}

fn write_shape(w: &mut Writer, value: &CollisionShape) {
    match value {
        CollisionShape::Box { half_extents_m } => {
            w.u8(0);
            write_vec3(w, *half_extents_m);
        }
        CollisionShape::Capsule { radius_m, half_height_m } => {
            w.u8(1);
            w.f64(*radius_m);
            w.f64(*half_height_m);
        }
        CollisionShape::Convex { points_local_m } => {
            w.u8(2);
            w.usize(points_local_m.len());
            for point in points_local_m {
                write_vec3(w, *point);
            }
        }
    }
}

fn read_shape(r: &mut Reader<'_>) -> Result<CollisionShape, ArchiveError> {
    match r.u8()? {
        0 => Ok(CollisionShape::Box { half_extents_m: read_vec3(r)? }),
        1 => Ok(CollisionShape::Capsule { radius_m: r.f64()?, half_height_m: r.f64()? }),
        2 => {
            let count = r.usize()?;
            let mut points_local_m = Vec::with_capacity(count);
            for _ in 0..count {
                points_local_m.push(read_vec3(r)?);
            }
            Ok(CollisionShape::Convex { points_local_m })
        }
        _ => Err(ArchiveError::InvalidData),
    }
}

fn write_vec3(w: &mut Writer, value: Vec3) {
    w.f64(value.x);
    w.f64(value.y);
    w.f64(value.z);
}

fn read_vec3(r: &mut Reader<'_>) -> Result<Vec3, ArchiveError> {
    Ok(Vec3::new(r.f64()?, r.f64()?, r.f64()?))
}

fn write_quat(w: &mut Writer, value: Quat) {
    w.f64(value.w);
    w.f64(value.x);
    w.f64(value.y);
    w.f64(value.z);
}

fn read_quat(r: &mut Reader<'_>) -> Result<Quat, ArchiveError> {
    let value = Quat::new(r.f64()?, r.f64()?, r.f64()?, r.f64()?);
    let norm = value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z;
    if !(0.5..=1.5).contains(&norm) {
        return Err(ArchiveError::InvalidData);
    }
    Ok(value)
}

fn write_f64_array<const N: usize>(w: &mut Writer, values: [f64; N]) {
    for value in values {
        w.f64(value);
    }
}

fn read_f64_array<const N: usize>(r: &mut Reader<'_>) -> Result<[f64; N], ArchiveError> {
    let mut values = [0.0; N];
    for value in &mut values {
        *value = r.f64()?;
    }
    Ok(values)
}

fn write_bool_array<const N: usize>(w: &mut Writer, values: [bool; N]) {
    for value in values {
        w.bool(value);
    }
}

fn read_bool_array<const N: usize>(r: &mut Reader<'_>) -> Result<[bool; N], ArchiveError> {
    let mut values = [false; N];
    for value in &mut values {
        *value = r.bool()?;
    }
    Ok(values)
}

#[cfg(test)]
mod tests {
    use super::{decode_snapshot, encode_snapshot_version};
    use crate::provenance::ParameterOrigin;
    use crate::world::{GroundSurface, PhysicsWorld};

    #[test]
    fn version_one_snapshot_defaults_new_abs_and_cornering_state() {
        let snapshot = PhysicsWorld::demo(1).snapshot();
        let decoded = decode_snapshot(&encode_snapshot_version(&snapshot, 1)).unwrap();

        assert_eq!(decoded.step(), snapshot.step());
        assert_eq!(decoded.vehicles[0].driver_aids.abs_pressure(), [0.0; 4]);
        assert!(
            decoded.vehicles[0]
                .definition
                .wheels
                .iter()
                .all(|wheel| wheel.cornering_stiffness_scale == 1.0 && wheel.tire_peak_grip_scale == 1.0)
        );
    }

    #[test]
    fn version_two_snapshot_preserves_physical_data_and_marks_provenance_legacy() {
        let snapshot = PhysicsWorld::demo(1).snapshot();
        let decoded = decode_snapshot(&encode_snapshot_version(&snapshot, 2)).unwrap();

        assert_eq!(decoded.vehicles[0].definition.wheels, snapshot.vehicles[0].definition.wheels);
        assert_eq!(decoded.vehicles[0].definition.chassis, snapshot.vehicles[0].definition.chassis);
        assert!(decoded.vehicles[0].definition.provenance.is_complete());
        for (_, provenance) in decoded.vehicles[0].definition.provenance.groups() {
            assert_eq!(provenance.origin, ParameterOrigin::Authored);
            assert!(provenance.source.contains("legacy snapshot"));
            assert_eq!(provenance.revision, "snapshot-v2");
        }
    }

    #[test]
    fn version_three_snapshot_migrates_transient_tire_state_without_inventing_history() {
        let mut world = PhysicsWorld::demo(1);
        world.vehicles[0].state.wheels[0].slip_angle_rad = 0.12;
        world.vehicles[0].state.wheels[0].transient_slip_angle_rad = 0.04;
        world.vehicles[0].state.wheels[0].relaxation_length_m = 0.72;
        let snapshot = world.snapshot();
        let decoded = decode_snapshot(&encode_snapshot_version(&snapshot, 3)).unwrap();
        let wheel = decoded.vehicles[0].state.wheels[0];

        assert_eq!(wheel.transient_slip_angle_rad, wheel.slip_angle_rad);
        assert_eq!(wheel.relaxation_length_m, crate::tire::MagicFormulaTire::default().relaxation_length_m);
        assert_eq!(decoded.tire_model, crate::tire::MagicFormulaTire::default());
        assert_eq!(decoded.vehicles[0].definition.provenance, snapshot.vehicles[0].definition.provenance);
    }

    #[test]
    fn legacy_snapshot_defaults_to_flat_ground_and_version_five_preserves_circuit_ground() {
        let snapshot = PhysicsWorld::demo(1).snapshot();
        assert_eq!(snapshot.config.ground_surface, GroundSurface::DemoCircuit);

        let legacy = decode_snapshot(&encode_snapshot_version(&snapshot, 4)).unwrap();
        assert_eq!(legacy.config.ground_surface, GroundSurface::Flat);
        let current = decode_snapshot(&encode_snapshot_version(&snapshot, 5)).unwrap();
        assert_eq!(current.config.ground_surface, GroundSurface::DemoCircuit);
    }
}
