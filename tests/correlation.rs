use my_physics::correlation::{
    AlignmentSpec, ChannelMapping, ClockCorrection, CorrelationPurpose, CsvTelemetryAdapter, DatasetFormat,
    DatasetManifest, DatasetSplit, EstimateOrigin, FieldRole, FieldSchema, Frame, ManifestCatalog, ParameterEstimate,
    ParameterEstimateArtifact, ProvenanceRecord, Quantity, Resampling, TelemetryAdapter, Unit, ValueTransform,
    align_and_resample, evaluate,
};
use std::fs;

fn manifest(id: &str, split: DatasetSplit, group: &str) -> DatasetManifest {
    DatasetManifest {
        schema_version: 1,
        dataset_id: id.to_owned(),
        format: DatasetFormat::Csv,
        source_title: "Synthetic correlation fixture".to_owned(),
        source_uri: "local:test-only".to_owned(),
        license_id: "CC0-1.0".to_owned(),
        license_verified: true,
        content_checksum: format!("synthetic-{id}"),
        vehicle_id: "fixture-car".to_owned(),
        session_id: id.to_owned(),
        timestamp_semantics: "sensor-sample-time".to_owned(),
        expected_sample_period_s: 0.1,
        maximum_gap_s: 0.11,
        split,
        split_group: group.to_owned(),
        provenance: ProvenanceRecord {
            origin: "synthetic".to_owned(),
            source: "tests/correlation.rs".to_owned(),
            revision: "fixture-v1".to_owned(),
            notes: "not real data".to_owned(),
        },
        mapping_revision: "fixture-map-v1".to_owned(),
        alignment_revision: "fixture-align-v1".to_owned(),
        filter_revision: "no-filter-v1".to_owned(),
        fields: vec![
            FieldSchema {
                canonical_name: "time_s".to_owned(),
                source_column: "Time".to_owned(),
                quantity: Quantity::Time,
                unit: Unit("s".to_owned()),
                frame: Frame("clock".to_owned()),
                role: FieldRole::Time,
                resampling: Resampling::Linear,
                transform: ValueTransform::Identity,
                metric_normalization_scale: 1.0,
                metric_lag_search_bound_s: 0.0,
            },
            FieldSchema {
                canonical_name: "speed_mps".to_owned(),
                source_column: "Speed mph".to_owned(),
                quantity: Quantity::Speed,
                unit: Unit("m/s".to_owned()),
                frame: Frame("vehicle".to_owned()),
                role: FieldRole::Observation,
                resampling: Resampling::Linear,
                transform: ValueTransform::Affine { scale: 0.44704, offset: 0.0 },
                metric_normalization_scale: 30.0,
                metric_lag_search_bound_s: 0.5,
            },
            FieldSchema {
                canonical_name: "steering_rad".to_owned(),
                source_column: "Steering wrapped deg".to_owned(),
                quantity: Quantity::Angle,
                unit: Unit("rad".to_owned()),
                frame: Frame("steering-wheel-relative".to_owned()),
                role: FieldRole::Input,
                resampling: Resampling::Previous,
                transform: ValueTransform::UnwrapAffine {
                    period: 360.0,
                    scale: core::f64::consts::PI / 180.0,
                    offset: 0.0,
                    relative_to_first: true,
                },
                metric_normalization_scale: 0.5,
                metric_lag_search_bound_s: 0.2,
            },
        ],
    }
}

fn write_temp(name: &str, contents: &str) -> std::path::PathBuf {
    let directory = std::env::temp_dir().join(format!("my-physics-correlation-{}-{}", std::process::id(), name));
    fs::create_dir_all(&directory).unwrap();
    let path = directory.join(name);
    fs::write(&path, contents).unwrap();
    path
}

#[test]
fn manifest_round_trip_preserves_schema_provenance_and_frozen_revisions() {
    let original = manifest("run-a", DatasetSplit::Training, "journey-a");
    original.validate().unwrap();
    assert_eq!(DatasetManifest::from_text(&original.to_text()).unwrap(), original);
}

#[test]
fn manifest_requires_checksum_and_coordinate_frame() {
    let mut value = manifest("run-a", DatasetSplit::Training, "journey-a");
    value.content_checksum.clear();
    assert!(value.validate().is_err());
    value.content_checksum = "sha256:fixture".to_owned();
    value.fields[1].frame.0.clear();
    assert!(value.validate().is_err());
}

#[test]
fn csv_adapter_applies_only_declared_affine_and_unwrap_transforms() {
    let manifest = manifest("run-a", DatasetSplit::Training, "journey-a");
    let path = write_temp("transform.csv", "Time,Speed mph,Steering wrapped deg\n0.0,10,350\n0.1,20,355\n0.2,30,2\n");
    let table = CsvTelemetryAdapter.read_path(&path, &manifest).unwrap();
    assert!((table.channels["speed_mps"][2] - 13.4112).abs() < 1.0e-12);
    assert!(table.channels["steering_rad"][0].abs() < 1.0e-12);
    assert!((table.channels["steering_rad"][1] - 5_f64.to_radians()).abs() < 1.0e-12);
    assert!((table.channels["steering_rad"][2] - 12_f64.to_radians()).abs() < 1.0e-12);
}

#[test]
fn relative_sensor_time_avoids_large_time_of_day_endpoint_roundoff() {
    let mut manifest = manifest("time-of-day", DatasetSplit::Test, "journey-time");
    manifest.fields[0].transform = ValueTransform::RelativeAffine { scale: 1.0, offset: 0.0 };
    let path = write_temp(
        "time-of-day.csv",
        "Time,Speed mph,Steering wrapped deg\n66233.1,10,0\n66233.2,11,0\n66233.3,12,0\n",
    );
    let table = CsvTelemetryAdapter.read_path(&path, &manifest).unwrap();
    assert_eq!(table.time_s[0], 0.0);
    let mut mapping = ChannelMapping::same("speed_mps", "m/s");
    mapping.normalization_scale = 10.0;
    let aligned = align_and_resample(
        &table,
        &table,
        &AlignmentSpec {
            revision: "time-of-day-v1".to_owned(),
            sample_period_s: 0.1,
            reference_clock: ClockCorrection::default(),
            candidate_clock: ClockCorrection::default(),
            mappings: vec![mapping],
        },
    )
    .unwrap();
    assert_eq!(aligned.time_s.len(), 3);
}

#[test]
fn adapter_rejects_duplicate_time_and_undeclared_gaps() {
    let manifest = manifest("run-a", DatasetSplit::Training, "journey-a");
    let duplicate = write_temp("duplicate.csv", "Time,Speed mph,Steering wrapped deg\n0.0,10,0\n0.1,11,0\n0.1,12,0\n");
    assert!(CsvTelemetryAdapter.read_path(&duplicate, &manifest).is_err());
    let gap = write_temp("gap.csv", "Time,Speed mph,Steering wrapped deg\n0.0,10,0\n0.1,11,0\n0.3,12,0\n");
    assert!(CsvTelemetryAdapter.read_path(&gap, &manifest).is_err());
}

#[test]
fn alignment_uses_declared_foh_zoh_and_rejects_unbounded_clock_warp() {
    let manifest = manifest("run-a", DatasetSplit::Test, "journey-a");
    let reference_path =
        write_temp("reference.csv", "Time,Speed mph,Steering wrapped deg\n0.0,0,0\n0.1,10,10\n0.2,20,20\n");
    let candidate_path =
        write_temp("candidate.csv", "Time,Speed mph,Steering wrapped deg\n0.0,0,0\n0.1,8,10\n0.2,18,20\n");
    let reference = CsvTelemetryAdapter.read_path(&reference_path, &manifest).unwrap();
    let candidate = CsvTelemetryAdapter.read_path(&candidate_path, &manifest).unwrap();
    let mut speed = ChannelMapping::same("speed_mps", "m/s");
    speed.reference_resampling = Resampling::Linear;
    speed.candidate_resampling = Resampling::Linear;
    let mut steering = ChannelMapping::same("steering_rad", "rad");
    steering.reference_resampling = Resampling::Previous;
    steering.candidate_resampling = Resampling::Previous;
    let specification = AlignmentSpec {
        revision: "fixture-align-v1".to_owned(),
        sample_period_s: 0.05,
        reference_clock: ClockCorrection::default(),
        candidate_clock: ClockCorrection::default(),
        mappings: vec![speed, steering],
    };
    let aligned = align_and_resample(&reference, &candidate, &specification).unwrap();
    assert!((aligned.reference["speed_mps"][1] - 2.2352).abs() < 1.0e-12);
    assert_eq!(aligned.reference["steering_rad"][1], 0.0, "ZOH must hold the preceding input sample");

    let invalid = AlignmentSpec {
        reference_clock: ClockCorrection { scale: 1.05, ..ClockCorrection::default() },
        ..specification
    };
    assert!(align_and_resample(&reference, &candidate, &invalid).is_err());
}

#[test]
fn journey_groups_cannot_leak_across_splits_and_purpose_is_enforced() {
    let training = manifest("train", DatasetSplit::Training, "same-journey");
    let test = manifest("test", DatasetSplit::Test, "same-journey");
    assert!(ManifestCatalog::new(vec![training.clone(), test]).is_err());
    assert!(training.require_purpose(CorrelationPurpose::FinalEvaluation).is_err());
    training.require_purpose(CorrelationPurpose::ParameterFitting).unwrap();
}

#[test]
fn report_metrics_are_deterministic_and_name_the_dataset_split_and_revisions() {
    let manifest = manifest("holdout", DatasetSplit::Test, "journey-holdout");
    let reference_path =
        write_temp("metrics-reference.csv", "Time,Speed mph,Steering wrapped deg\n0.0,0,0\n0.1,10,0\n0.2,20,0\n");
    let candidate_path =
        write_temp("metrics-candidate.csv", "Time,Speed mph,Steering wrapped deg\n0.0,0,0\n0.1,8,0\n0.2,18,0\n");
    let reference = CsvTelemetryAdapter.read_path(&reference_path, &manifest).unwrap();
    let candidate = CsvTelemetryAdapter.read_path(&candidate_path, &manifest).unwrap();
    let mut speed_mapping = ChannelMapping::same("speed_mps", "m/s");
    speed_mapping.lag_search_bound_s = 0.2;
    let aligned = align_and_resample(
        &reference,
        &candidate,
        &AlignmentSpec {
            revision: manifest.alignment_revision.clone(),
            sample_period_s: 0.1,
            reference_clock: ClockCorrection::default(),
            candidate_clock: ClockCorrection::default(),
            mappings: vec![speed_mapping],
        },
    )
    .unwrap();
    let report =
        evaluate("holdout-report", CorrelationPurpose::FinalEvaluation, &manifest, &manifest, "sim-v1", &aligned)
            .unwrap();
    assert_eq!(report.reference_split, "test");
    assert_eq!(report.mapping_revision, "fixture-map-v1");
    assert_eq!(report.alignment_revision, "fixture-align-v1");
    assert!(report.channel_metrics["speed_mps"].rmse > 0.0);
    assert!(report.channel_metrics["speed_mps"].best_lag_s.is_some(), "negative lag candidates must not panic");
    assert_eq!(report.to_json(), report.to_json());

    let mut empty = aligned.clone();
    empty.mappings.clear();
    assert!(evaluate("empty", CorrelationPurpose::FinalEvaluation, &manifest, &manifest, "v1", &empty).is_err());
    let mut overflow = aligned;
    overflow.candidate.get_mut("speed_mps").unwrap()[1] = 1.0e308;
    assert!(evaluate("overflow", CorrelationPurpose::FinalEvaluation, &manifest, &manifest, "v1", &overflow).is_err());
}

#[test]
fn fitted_parameter_provenance_can_only_name_training_data() {
    let mut artifact = ParameterEstimateArtifact {
        schema_version: 1,
        artifact_id: "proxy-v1".to_owned(),
        vehicle_proxy: "explicitly non-exact fixture".to_owned(),
        frozen_revision: "proxy-v1".to_owned(),
        limitations: vec!["test fixture".to_owned()],
        parameters: vec![ParameterEstimate {
            parameter: "chassis.mass_kg".to_owned(),
            value: 1100.0,
            unit: "kg".to_owned(),
            origin: EstimateOrigin::Fitted,
            source: "training-run".to_owned(),
            source_revision: "training-v1".to_owned(),
            source_split: Some(DatasetSplit::Test),
            uncertainty: "synthetic".to_owned(),
            valid_min: 900.0,
            valid_max: 1400.0,
        }],
    };
    assert!(artifact.validate().is_err());
    artifact.parameters[0].source_split = Some(DatasetSplit::Training);
    artifact.validate().unwrap();
    assert!(artifact.to_text().unwrap().contains("parameter=chassis.mass_kg"));

    artifact.parameters[0].origin = EstimateOrigin::Derived;
    artifact.parameters[0].source_split = Some(DatasetSplit::Validation);
    assert!(artifact.validate().is_err());
}
