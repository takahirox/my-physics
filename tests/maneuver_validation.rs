use my_physics::validation::{
    InputProgram, SCENARIOS, ScenarioDefinition, run_catalog, run_scenario, run_scenario_with_dt,
};

#[test]
fn catalog_is_deterministic_and_within_declared_bounds() {
    let first = run_catalog();
    let second = run_catalog();
    assert_eq!(first.len(), SCENARIOS.len());
    for (left, right) in first.iter().zip(&second) {
        assert!(left.passed(), "{}: {:?}", left.definition.name, left.checks);
        assert_eq!(left.fingerprint, right.fingerprint, "{}", left.definition.name);
        assert_eq!(left.samples, right.samples, "{}", left.definition.name);
    }
}

#[test]
fn steady_steer_is_left_right_mirrored() {
    let base = *SCENARIOS.iter().find(|scenario| scenario.name == "steady_steer").unwrap();
    let InputProgram::RampAndHoldSteer { steer_rad, ramp_s } = base.input else {
        unreachable!();
    };
    let mirrored = ScenarioDefinition {
        name: "steady_steer_mirrored",
        input: InputProgram::RampAndHoldSteer { steer_rad: -steer_rad, ramp_s },
        ..base
    };
    let left = run_scenario(&base);
    let right = run_scenario(&mirrored);
    assert!((left.summary.final_speed_mps - right.summary.final_speed_mps).abs() < 0.02);
    assert!((left.summary.peak_yaw_rate_rad_s - right.summary.peak_yaw_rate_rad_s).abs() < 0.005);
    assert!((left.summary.final_yaw_rate_abs_rad_s - right.summary.final_yaw_rate_abs_rad_s).abs() < 0.005);
}

#[test]
fn fixed_timestep_converges_when_halved() {
    let base = *SCENARIOS.iter().find(|scenario| scenario.name == "coast_down").unwrap();
    let short = ScenarioDefinition { name: "coast_down_dt", duration_s: 2.0, ..base };
    let coarse = run_scenario_with_dt(&short, 0.001);
    let fine = run_scenario_with_dt(&short, 0.0005);
    assert!((coarse.summary.final_speed_mps - fine.summary.final_speed_mps).abs() < 0.02);
    assert!((coarse.summary.distance_m - fine.summary.distance_m).abs() < 0.05);
}

#[test]
fn aerodynamic_drag_obeys_speed_squared_metamorphic_relation() {
    use my_physics::VehicleDefinition;
    use my_physics::vehicle::aerodynamic_drag_magnitude_n;
    let chassis = VehicleDefinition::default().chassis;
    let low = aerodynamic_drag_magnitude_n(&chassis, 10.0, 0.0);
    let high = aerodynamic_drag_magnitude_n(&chassis, 20.0, 0.0);
    let ratio = high / low;
    assert!((ratio - 4.0).abs() < 1.0e-12, "ratio={ratio}");
}
