use my_physics::validation::{SCENARIOS, run_scenario, verify_midpoint_snapshot_replay};
use my_physics::{PhysicsWorld, VehicleDefinition};

#[test]
fn engineering_lab_is_a_single_full_fidelity_collision_free_world() {
    let world = PhysicsWorld::engineering_lab();
    assert_eq!(world.vehicles.len(), 1);
    assert_eq!(world.vehicles[0].definition, VehicleDefinition::engineering_reference());
    assert_eq!(world.vehicles[0].fidelity, 1.0);
    assert_eq!(world.vehicles[0].target_fidelity, 1.0);
    assert!(!world.config.automatic_lod);
    assert!(world.static_colliders.is_empty());
}

#[test]
fn browser_lab_catalog_semantics_pass_all_official_envelopes() {
    for definition in SCENARIOS {
        let first = run_scenario(definition);
        let second = run_scenario(definition);
        assert!(first.passed(), "{} failed: {:?}", definition.name, first.checks);
        assert_eq!(first, second, "{} independent repeat diverged", definition.name);
    }
}

#[test]
fn midpoint_snapshot_replay_is_exact_for_the_entire_catalog() {
    for definition in SCENARIOS {
        assert!(verify_midpoint_snapshot_replay(definition), "{} replay diverged", definition.name);
    }
}
