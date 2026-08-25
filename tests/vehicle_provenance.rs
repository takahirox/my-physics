use my_physics::{
    ParameterOrigin, ParameterProvenance, PhysicsWorld, VehicleDefinition, VehicleParameterProvenance, VehiclePreset,
};

fn assert_value(group: &ParameterProvenance, parameter: &str, value: f64) {
    let range = group
        .valid_ranges
        .iter()
        .find(|range| range.parameter == parameter)
        .unwrap_or_else(|| panic!("missing provenance range for {parameter}"));
    assert!((range.minimum..=range.maximum).contains(&value), "{parameter}={value}, range={range:?}");
}

fn assert_wheel_group(group: &ParameterProvenance, axle: &str, wheel: &my_physics::vehicle::WheelDefinition) {
    assert_value(group, &format!("{axle}_track_half_width"), wheel.mount_local_m.x.abs());
    assert_value(group, &format!("{axle}_axle_position"), wheel.mount_local_m.z);
    assert_value(group, &format!("{axle}_wheel_radius"), wheel.radius_m);
    assert_value(group, &format!("{axle}_wheel_inertia"), wheel.inertia_kg_m2);
    assert_value(group, &format!("{axle}_wheel_mass"), wheel.mass_kg);
    assert_value(group, &format!("{axle}_max_steer"), wheel.max_steer_rad);
    assert_value(group, &format!("{axle}_cornering_stiffness_scale"), wheel.cornering_stiffness_scale);
    assert_value(group, &format!("{axle}_tire_peak_grip_scale"), wheel.tire_peak_grip_scale);
}

#[test]
fn every_default_parameter_group_has_complete_non_measured_provenance() {
    let definition = VehicleDefinition::default();
    assert_eq!(definition.provenance.groups().len(), VehicleParameterProvenance::GROUP_COUNT);
    assert!(definition.provenance.is_complete());
    for (name, provenance) in definition.provenance.groups() {
        assert_ne!(provenance.origin, ParameterOrigin::Measured, "{name} must not claim uncollected measurements");
    }
    assert!(VehicleDefinition::engineering_reference().provenance.is_complete());

    let chassis = &definition.provenance.chassis_mass_properties;
    assert_value(chassis, "dry_mass", definition.chassis.dry_mass_kg);
    for (name, value) in [
        ("cg_x", definition.chassis.cg_local_m.x),
        ("cg_y", definition.chassis.cg_local_m.y),
        ("cg_z", definition.chassis.cg_local_m.z),
        ("inertia_x", definition.chassis.inertia_kg_m2.x),
        ("inertia_y", definition.chassis.inertia_kg_m2.y),
        ("inertia_z", definition.chassis.inertia_kg_m2.z),
    ] {
        assert_value(chassis, name, value);
    }
    let aero = &definition.provenance.aerodynamics;
    for (name, value) in [
        ("frontal_area", definition.chassis.frontal_area_m2),
        ("drag_coefficient", definition.chassis.drag_coefficient),
        ("lift_coefficient", definition.chassis.lift_coefficient),
        ("reference_air_density", definition.chassis.air_density_kg_m3),
    ] {
        assert_value(aero, name, value);
    }
    assert_wheel_group(&definition.provenance.front_wheels_and_tires, "front", &definition.wheels[0]);
    assert_wheel_group(&definition.provenance.rear_wheels_and_tires, "rear", &definition.wheels[2]);
    let suspension = &definition.provenance.suspension;
    for (name, value) in [
        ("spring_rate", definition.wheels[0].spring_rate_n_m),
        ("damper_rate", definition.wheels[0].damper_rate_n_s_m),
        ("rest_length", definition.wheels[0].rest_length_m),
        ("maximum_travel", definition.wheels[0].max_travel_m),
        ("bump_stop_rate", definition.wheels[0].bump_stop_rate_n_m),
        ("anti_roll_rate", definition.anti_roll_rate_n_m_rad),
    ] {
        assert_value(suspension, name, value);
    }
    assert_value(&definition.provenance.brakes, "front_brake_torque", definition.wheels[0].brake_torque_nm);
    assert_value(&definition.provenance.brakes, "rear_brake_torque", definition.wheels[2].brake_torque_nm);
    let engine = &definition.provenance.engine;
    for (name, value) in [
        ("idle_speed", definition.engine.idle_rpm),
        ("redline", definition.engine.redline_rpm),
        ("engine_inertia", definition.engine.inertia_kg_m2),
        ("fuel_energy", definition.engine.fuel_energy_j_kg),
        ("thermal_efficiency", definition.engine.efficiency),
    ] {
        assert_value(engine, name, value);
    }
    for (speed, torque) in definition.engine.torque_curve {
        assert_value(engine, "torque_curve_speed", speed);
        assert_value(engine, "torque_curve_torque", torque);
    }
    let transmission = &definition.provenance.transmission_and_clutch;
    for ratio in definition.transmission.gear_ratios {
        assert_value(transmission, "forward_gear_ratio", ratio);
    }
    for (name, value) in [
        ("reverse_gear_ratio", definition.transmission.reverse_ratio),
        ("final_drive", definition.transmission.final_drive),
        ("shift_time", definition.transmission.shift_time_s),
        ("clutch_capacity", definition.transmission.clutch_capacity_nm),
        ("clutch_stiffness", definition.transmission.clutch_stiffness_nm_per_rad_s),
    ] {
        assert_value(transmission, name, value);
    }
    let fuel = &definition.provenance.fuel_system;
    for (name, value) in [
        ("fuel_capacity", definition.fuel_capacity_kg),
        ("fuel_tank_x", definition.fuel_tank_local_m.x),
        ("fuel_tank_y", definition.fuel_tank_local_m.y),
        ("fuel_tank_z", definition.fuel_tank_local_m.z),
    ] {
        assert_value(fuel, name, value);
    }
}

#[test]
fn presets_share_equations_and_expose_the_complete_authored_difference() {
    assert_eq!(VehicleDefinition::default(), VehicleDefinition::from_preset(VehiclePreset::RaceGameplay));
    let engineering = VehicleDefinition::engineering_reference();
    let race = VehicleDefinition::from_preset(VehiclePreset::RaceGameplay);
    assert_eq!(race.wheels[2].cornering_stiffness_scale, 1.05);
    assert_eq!(race.wheels[2].tire_peak_grip_scale, 1.06);
    assert_eq!(race.provenance.rear_wheels_and_tires.origin, ParameterOrigin::Authored);
    assert!(race.provenance.rear_wheels_and_tires.source.contains("no measured"));

    let mut normalized = race;
    normalized.name = engineering.name.clone();
    normalized.provenance = engineering.provenance.clone();
    for wheel in &mut normalized.wheels[2..] {
        wheel.cornering_stiffness_scale = 1.0;
        wheel.tire_peak_grip_scale = 1.0;
    }
    assert_eq!(normalized, engineering, "preset has an undocumented physical parameter difference");
}

#[test]
fn demo_selects_the_race_preset_explicitly() {
    let world = PhysicsWorld::demo(1);
    assert_eq!(world.vehicles[0].definition, VehicleDefinition::race_gameplay());
}
