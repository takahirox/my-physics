//! Reproducible IO-VNBD vehicle-correlation runner.
//!
//! Raw data remains outside the repository. This adapter reconstructs declared
//! driver inputs, initializes the common plant once at t0, then advances the
//! ordinary fixed 1 ms physics without correction forces or state nudging.

use my_physics::correlation::{
    AlignmentSpec, ChannelMapping, ClockCorrection, CorrelationPurpose, CsvTelemetryAdapter, DatasetFormat,
    DatasetManifest, DatasetSplit, EstimateOrigin, FieldRole, FieldSchema, Frame, ParameterEstimate,
    ParameterEstimateArtifact, ProvenanceRecord, Quantity, Resampling, TelemetryAdapter, TelemetryTable, Unit,
    ValueTransform, align_and_resample, evaluate, sha256_hex, write_report_artifacts,
};
use my_physics::provenance::ParameterOrigin;
use my_physics::{DriverInput, PhysicsWorld, SimulationConfig, Vec3, VehicleDefinition};
use std::collections::BTreeMap;
use std::error::Error;
use std::fmt::Write as _;
use std::fs;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::process::Command;

const MAPPING_REVISION: &str = "io-vnbd-map-v1";
const ALIGNMENT_REVISION: &str = "io-vnbd-sensor-time-no-latency-v1";
const FILTER_REVISION: &str = "no-signal-filter-v1";
const INPUT_RECONSTRUCTION_REVISION: &str = "io-vnbd-foh-pedals-steering-zoh-last-valid-gear-v1";
const PROXY_BASELINE_REVISION: &str = "fiesta-titanium-fwd-proxy-pre-cal-v1";
const PROXY_CALIBRATED_REVISION: &str = "fiesta-titanium-fwd-proxy-cal-v1";
const GRAVITY: f64 = 9.80665;
const PSI_TO_PA: f64 = 6_894.757_293_168;
const CALIBRATION_HASHES: &str = "125a3ab348bca6b5d9fb2d52b7ba1f27d4a31f03b7b60b16ef778f7935bb4380,f474667235581e514d44f371cc81039aacc6c0604669f552f127bdb8c60e370b,c68f61a381555a0c5669b9952b59d7424f1bef82fd05b1cb8fbc26bb549b12aa";

#[derive(Clone, Copy)]
struct Run {
    id: &'static str,
    split: &'static str,
    checksum: &'static str,
    bytes: u64,
    pressure: char,
}

const RUNS: [Run; 7] = [
    Run {
        id: "V-Vw1",
        split: "calibration",
        checksum: "125a3ab348bca6b5d9fb2d52b7ba1f27d4a31f03b7b60b16ef778f7935bb4380",
        bytes: 4_001_856,
        pressure: 'C',
    },
    Run {
        id: "V-Vw12",
        split: "calibration",
        checksum: "f474667235581e514d44f371cc81039aacc6c0604669f552f127bdb8c60e370b",
        bytes: 188_273,
        pressure: 'D',
    },
    Run {
        id: "V-Vfb02c",
        split: "calibration",
        checksum: "c68f61a381555a0c5669b9952b59d7424f1bef82fd05b1cb8fbc26bb549b12aa",
        bytes: 132_991,
        pressure: 'D',
    },
    Run {
        id: "V-Vw7",
        split: "validation",
        checksum: "3ba9d21d1532b4e15dc41ed9e4baa996e1ccf7b447f2fc41ee44564a5dd1f5ef",
        bytes: 346_895,
        pressure: 'D',
    },
    Run {
        id: "V-Vw16b",
        split: "validation",
        checksum: "1e81b217eaf6c2340aafbbc122ad738d876013366737437155a4e722eb6fd8ed",
        bytes: 239_979,
        pressure: 'D',
    },
    Run {
        id: "V-Vta1b",
        split: "holdout",
        checksum: "35b4e74bd1597d9f82895c3451dbe71a2d4084a3718cbdf8ea7fbb04f206db94",
        bytes: 205_277,
        pressure: 'A',
    },
    Run {
        id: "V-vtb12",
        split: "holdout",
        checksum: "dad39fa7ff152dc13ab8e7d903047bcfdb7882c8fe03b7c0f8f12eef07cbc115",
        bytes: 94_747,
        pressure: 'A',
    },
];

#[derive(Clone)]
struct Options {
    data_root: PathBuf,
    output: PathBuf,
    split: String,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("correlate-io-vnbd: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let options = parse_options()?;
    verify_acquisition_manifest()?;
    fs::create_dir_all(&options.output)?;
    fs::write(options.output.join("parameter-estimates.manifest"), parameter_artifact().to_text()?)?;
    fs::write(options.output.join("fit-trace.csv"), fit_trace())?;
    fs::write(options.output.join("limitations.md"), limitations())?;

    let mut summaries = Vec::new();
    for run in RUNS.iter().copied().filter(|run| options.split == "all" || options.split == run.split) {
        let raw_path = options.data_root.join(format!("{}.csv", run.id));
        let metadata = fs::metadata(&raw_path)?;
        if metadata.len() != run.bytes {
            return Err(format!("{} byte size mismatch: expected {}, got {}", run.id, run.bytes, metadata.len()).into());
        }
        verify_exact_header(&raw_path)?;
        let actual_checksum = sha256_hex(&fs::read(&raw_path)?);
        if actual_checksum != run.checksum {
            return Err(
                format!("{} SHA-256 mismatch: expected {}, got {}", run.id, run.checksum, actual_checksum).into()
            );
        }
        let manifest = reference_manifest(run);
        let reference = CsvTelemetryAdapter.read_path(&raw_path, &manifest)?;
        verify_source_semantics(&reference)?;

        let baseline = simulate(&reference, run, false)?;
        let calibrated = simulate(&reference, run, true)?;
        let baseline_manifest = candidate_manifest(&manifest, run, false, &baseline);
        let calibrated_manifest = candidate_manifest(&manifest, run, true, &calibrated);
        let mappings = scored_mappings();
        let align = |candidate: &TelemetryTable| {
            align_and_resample(
                &reference,
                candidate,
                &AlignmentSpec {
                    revision: ALIGNMENT_REVISION.to_owned(),
                    sample_period_s: 0.1,
                    reference_clock: ClockCorrection::default(),
                    candidate_clock: ClockCorrection::default(),
                    mappings: mappings.clone(),
                },
            )
        };
        let baseline_aligned = align(&baseline)?;
        let calibrated_aligned = align(&calibrated)?;
        let purpose = purpose(run.split);
        let baseline_report = evaluate(
            format!("{}-baseline", run.id),
            purpose,
            &manifest,
            &baseline_manifest,
            PROXY_BASELINE_REVISION,
            &baseline_aligned,
        )?;
        let calibrated_report = evaluate(
            format!("{}-calibrated", run.id),
            purpose,
            &manifest,
            &calibrated_manifest,
            PROXY_CALIBRATED_REVISION,
            &calibrated_aligned,
        )?;
        let directory = options.output.join(run.split).join(run.id);
        fs::create_dir_all(&directory)?;
        fs::write(directory.join("baseline-report.json"), baseline_report.to_json())?;
        fs::write(directory.join("baseline-metrics.csv"), baseline_report.metrics_csv())?;
        write_report_artifacts(&directory, &calibrated_report, &calibrated_aligned)?;
        write_simulation_csv(&directory.join("baseline-simulation.csv"), &baseline)?;
        write_simulation_csv(&directory.join("simulation.csv"), &calibrated)?;
        fs::write(directory.join("timeseries.svg"), timeseries_svg(run.id, &calibrated_aligned))?;
        fs::write(directory.join("event-metrics.json"), event_metrics(&reference, &baseline, &calibrated))?;
        fs::write(
            directory.join("run-provenance.json"),
            run_provenance(run, &reference, &baseline_manifest, &calibrated_manifest),
        )?;
        summaries.push((run, baseline_report.aggregate_normalized_rmse, calibrated_report.aggregate_normalized_rmse));
        println!(
            "{} [{}]: normalized RMSE baseline {:.6}, calibrated {:.6}",
            run.id, run.split, baseline_report.aggregate_normalized_rmse, calibrated_report.aggregate_normalized_rmse
        );
    }
    if summaries.is_empty() {
        return Err(format!("no runs selected for split {:?}", options.split).into());
    }
    let summary = summary_json(&summaries);
    fs::write(options.output.join(format!("summary-{}.json", options.split)), &summary)?;
    if options.split == "all" {
        fs::write(options.output.join("summary.json"), summary)?;
    }
    Ok(())
}

fn parse_options() -> Result<Options, Box<dyn Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut data_root = None;
    let mut output = None;
    let mut split = "all".to_owned();
    let mut index = 0;
    while index < args.len() {
        let value = args.get(index + 1).ok_or_else(|| format!("{} requires a value", args[index]))?;
        match args[index].as_str() {
            "--data-root" => data_root = Some(PathBuf::from(value)),
            "--output" => output = Some(PathBuf::from(value)),
            "--split" => split = value.clone(),
            "--help" | "-h" => return Err(
                "usage: correlate-io-vnbd --data-root PATH --output PATH [--split calibration|validation|holdout|all]"
                    .into(),
            ),
            unknown => return Err(format!("unknown argument {unknown:?}").into()),
        }
        index += 2;
    }
    if !matches!(split.as_str(), "calibration" | "validation" | "holdout" | "all") {
        return Err(format!("invalid split {split:?}").into());
    }
    Ok(Options {
        data_root: data_root.ok_or("--data-root is required")?,
        output: output.ok_or("--output is required")?,
        split,
    })
}

fn split(name: &str) -> DatasetSplit {
    match name {
        "calibration" => DatasetSplit::Training,
        "validation" => DatasetSplit::Validation,
        "holdout" => DatasetSplit::Test,
        _ => unreachable!(),
    }
}

fn purpose(name: &str) -> CorrelationPurpose {
    match name {
        "calibration" => CorrelationPurpose::ParameterFitting,
        "validation" => CorrelationPurpose::ModelSelection,
        "holdout" => CorrelationPurpose::FinalEvaluation,
        _ => unreachable!(),
    }
}

#[allow(clippy::too_many_arguments)]
fn field(
    canonical: &str,
    source: &str,
    quantity: Quantity,
    unit: &str,
    frame: &str,
    role: FieldRole,
    resampling: Resampling,
    transform: ValueTransform,
    scale: f64,
    lag: f64,
) -> FieldSchema {
    FieldSchema {
        canonical_name: canonical.to_owned(),
        source_column: source.to_owned(),
        quantity,
        unit: Unit(unit.to_owned()),
        frame: Frame(frame.to_owned()),
        role,
        resampling,
        transform,
        metric_normalization_scale: scale,
        metric_lag_search_bound_s: lag,
    }
}

fn reference_manifest(run: Run) -> DatasetManifest {
    let deg = core::f64::consts::PI / 180.0;
    DatasetManifest {
        schema_version: 1,
        dataset_id: run.id.to_owned(),
        format: DatasetFormat::Csv,
        source_title: "IO-VNBD vehicle/CAN telemetry".to_owned(),
        source_uri: "https://github.com/onyekpeu/IO-VNBD/tree/118939602e3422d47b8ab0807b623751c3ac135b".to_owned(),
        license_id: "UNSPECIFIED-DATASET-LICENSE".to_owned(),
        license_verified: false,
        content_checksum: format!("sha256:{}", run.checksum),
        vehicle_id: "IO-VNBD Ford Fiesta Titanium; exact year/engine unknown".to_owned(),
        session_id: run.id.to_owned(),
        timestamp_semantics: "VBOX sensor sample: seconds since start of day; 10 Hz".to_owned(),
        expected_sample_period_s: 0.1,
        maximum_gap_s: 0.100_001,
        split: split(run.split),
        split_group: run.id.to_owned(),
        provenance: ProvenanceRecord {
            origin: "measured-telemetry".to_owned(),
            source: "Onyekpe et al., Data in Brief 35 (2021) 106885; pinned IO-VNBD Git commit".to_owned(),
            revision: "118939602e3422d47b8ab0807b623751c3ac135b".to_owned(),
            notes: "Raw repository has no explicit dataset license; redistribution disabled".to_owned(),
        },
        mapping_revision: MAPPING_REVISION.to_owned(),
        alignment_revision: ALIGNMENT_REVISION.to_owned(),
        filter_revision: FILTER_REVISION.to_owned(),
        fields: vec![
            field(
                "time_s",
                "Time Since Start of Day (seconds)",
                Quantity::Time,
                "s",
                "sensor-clock",
                FieldRole::Time,
                Resampling::Linear,
                ValueTransform::RelativeAffine { scale: 1.0, offset: 0.0 },
                1.0,
                0.0,
            ),
            field(
                "speed_mps",
                "Indicated Vehicle Speed (km/hr)",
                Quantity::Speed,
                "m/s",
                "vehicle-speed",
                FieldRole::Observation,
                Resampling::Linear,
                ValueTransform::Affine { scale: 1.0 / 3.6, offset: 0.0 },
                5.0,
                0.5,
            ),
            field(
                "steering_wheel_rad",
                "Steering Angle (degrees)",
                Quantity::Angle,
                "rad",
                "positive-right-steering-sensor; location/ratio unresolved",
                FieldRole::Input,
                Resampling::Linear,
                ValueTransform::UnwrapAffine {
                    period: 360.0,
                    scale: deg,
                    offset: -4.8 * deg,
                    relative_to_first: false,
                },
                0.25,
                0.3,
            ),
            field(
                "wheel_omega_fl_rad_s",
                "Wheel Speed Front Left (rad/sec)",
                Quantity::AngularRate,
                "rad/s",
                "wheel-FL",
                FieldRole::Observation,
                Resampling::Linear,
                ValueTransform::Identity,
                20.0,
                0.3,
            ),
            field(
                "wheel_omega_fr_rad_s",
                "Wheel Speed Front Right (rad/sec)",
                Quantity::AngularRate,
                "rad/s",
                "wheel-FR",
                FieldRole::Observation,
                Resampling::Linear,
                ValueTransform::Identity,
                20.0,
                0.3,
            ),
            field(
                "wheel_omega_rl_rad_s",
                "Wheel Speed Rear Left (rad/sec)",
                Quantity::AngularRate,
                "rad/s",
                "wheel-RL",
                FieldRole::Observation,
                Resampling::Linear,
                ValueTransform::Identity,
                20.0,
                0.3,
            ),
            field(
                "wheel_omega_rr_rad_s",
                "Wheel Speed Rear Right (rad/sec)",
                Quantity::AngularRate,
                "rad/s",
                "wheel-RR",
                FieldRole::Observation,
                Resampling::Linear,
                ValueTransform::Identity,
                20.0,
                0.3,
            ),
            field(
                "yaw_rate_rad_s",
                "Yaw Rate (deg/sec)",
                Quantity::AngularRate,
                "rad/s",
                "internal +Y; positive-left",
                FieldRole::Observation,
                Resampling::Linear,
                ValueTransform::Affine { scale: deg, offset: -0.05955 * deg },
                0.2,
                0.5,
            ),
            field(
                "longitudinal_acceleration_forward_mps2",
                "Indicated Longitudinal Acceleration (g)",
                Quantity::Acceleration,
                "m/s^2",
                "body-forward (-Z)",
                FieldRole::Observation,
                Resampling::Linear,
                ValueTransform::Affine { scale: GRAVITY, offset: 0.04107 * GRAVITY },
                2.0,
                0.5,
            ),
            field(
                "lateral_acceleration_right_mps2",
                "Indicated Lateral Acceleration (g)",
                Quantity::Acceleration,
                "m/s^2",
                "body +X right; source sign negated",
                FieldRole::Observation,
                Resampling::Linear,
                ValueTransform::Affine { scale: -GRAVITY, offset: 0.03206 * GRAVITY },
                2.0,
                0.5,
            ),
            field(
                "handbrake_fraction",
                "Handbrake (0 or 1)",
                Quantity::Ratio,
                "1",
                "driver-input",
                FieldRole::Input,
                Resampling::Previous,
                ValueTransform::Identity,
                1.0,
                0.0,
            ),
            field(
                "transmission_gear_evidence",
                "Gear Requested (Number fof gear employed 1-5)",
                Quantity::Other,
                "gear-index",
                "CAN evidence; observed 1-6; not command truth",
                FieldRole::Context,
                Resampling::Previous,
                ValueTransform::Identity,
                1.0,
                0.0,
            ),
            field(
                "engine_rpm",
                "Engine Speed (rev/min)",
                Quantity::AngularRate,
                "rpm",
                "engine-crankshaft",
                FieldRole::Observation,
                Resampling::Linear,
                ValueTransform::Identity,
                1_000.0,
                0.5,
            ),
            field(
                "brake_pressure_pa",
                "Brake Pressure (psi)",
                Quantity::Pressure,
                "Pa",
                "brake-input-sensor",
                FieldRole::Input,
                Resampling::Linear,
                ValueTransform::Affine { scale: PSI_TO_PA, offset: 0.0076 * PSI_TO_PA },
                100_000.0,
                0.2,
            ),
            field(
                "brake_position",
                "Brake Position (0 or 1)",
                Quantity::Ratio,
                "1",
                "driver-input",
                FieldRole::Context,
                Resampling::Previous,
                ValueTransform::Identity,
                1.0,
                0.0,
            ),
            field(
                "accelerator_fraction",
                "Accelerator Pedal Position (0 or 1)",
                Quantity::Ratio,
                "1",
                "driver-input; header erroneous, raw is percent",
                FieldRole::Input,
                Resampling::Linear,
                ValueTransform::Affine { scale: 0.01, offset: 0.0 },
                0.25,
                0.2,
            ),
        ],
    }
}

fn verify_source_semantics(table: &TelemetryTable) -> Result<(), Box<dyn Error>> {
    table.validate()?;
    let accelerator = &table.channels["accelerator_fraction"];
    if accelerator.iter().any(|value| !(0.0..=1.01).contains(value)) {
        return Err("accelerator source violates declared percent-to-fraction mapping".into());
    }
    let gears = &table.channels["transmission_gear_evidence"];
    if gears.iter().any(|value| !value.is_finite()) {
        return Err("non-finite gear evidence".into());
    }
    Ok(())
}

fn fiesta_proxy(calibrated: bool) -> VehicleDefinition {
    let mut definition = VehicleDefinition::engineering_reference();
    definition.name = if calibrated {
        "IO-VNBD Fiesta Titanium FWD proxy calibrated"
    } else {
        "IO-VNBD Fiesta Titanium FWD proxy baseline"
    }
    .to_owned();
    definition.chassis.dry_mass_kg = 1_150.0;
    definition.chassis.inertia_kg_m2 = Vec3::new(420.0, 1_250.0, 1_350.0);
    definition.chassis.frontal_area_m2 = 2.10;
    definition.chassis.drag_coefficient = 0.33;
    definition.chassis.lift_coefficient = 0.0;
    let radius = if calibrated { 0.278_756 } else { 0.292 };
    for (index, wheel) in definition.wheels.iter_mut().enumerate() {
        wheel.radius_m = radius;
        wheel.driven = index < 2;
        wheel.mount_local_m.x = if index % 2 == 0 { -0.735 } else { 0.735 };
        wheel.mount_local_m.z = if index < 2 { -1.045 } else { 1.444 };
        wheel.max_steer_rad = if index < 2 { 0.54 } else { 0.0 };
        wheel.brake_torque_nm = if index < 2 { 2_300.0 } else { 1_300.0 };
    }
    definition.engine.idle_rpm = 800.0;
    definition.engine.redline_rpm = 6_500.0;
    definition.engine.torque_curve = [
        (800.0, 75.0),
        (1_500.0, 105.0),
        (2_000.0, 120.0),
        (3_000.0, 125.0),
        (4_000.0, 122.0),
        (5_000.0, 115.0),
        (6_000.0, 102.0),
        (6_500.0, 90.0),
    ];
    definition.transmission.automatic = false;
    // 3.89 is an explicit analogous-spec assumption. IO-VNBD identifies only
    // the product of gearbox and final drive; retain both public meanings and
    // condition the fitted gearbox ratios on this assumed final drive.
    definition.transmission.final_drive = 3.89;
    definition.transmission.gear_ratios = if calibrated {
        [3.917, 9.3659 / 3.89, 6.4020 / 3.89, 4.4083 / 3.89, 3.1507 / 3.89, 2.6001 / 3.89, 2.6001 / 3.89]
    } else {
        [3.917, 2.429, 1.436, 1.021, 0.867, 0.700, 0.700]
    };
    definition.transmission.clutch_capacity_nm = 300.0;
    definition.fuel_capacity_kg = 35.0;
    let tag = if calibrated { PROXY_CALIBRATED_REVISION } else { PROXY_BASELINE_REVISION };
    let replace =
        |mut provenance: my_physics::provenance::ParameterProvenance, origin: ParameterOrigin, source: &str| {
            provenance.origin = origin;
            provenance.source = source.to_owned();
            provenance.revision = tag.to_owned();
            provenance.uncertainty_fraction = None;
            provenance
        };
    definition.provenance.chassis_mass_properties = replace(
        definition.provenance.chassis_mass_properties,
        ParameterOrigin::Estimated,
        "Fiesta Titanium proxy estimates: 1150 kg dry mass, zero CG offset, estimated inertia; not IO-VNBD measurements",
    );
    definition.provenance.aerodynamics = replace(
        definition.provenance.aerodynamics,
        ParameterOrigin::Estimated,
        "Fiesta-class proxy area/drag and neutral lift; exact IO-VNBD aero unmeasured",
    );
    definition.provenance.front_wheels_and_tires = replace(
        definition.provenance.front_wheels_and_tires,
        if calibrated { ParameterOrigin::Fitted } else { ParameterOrigin::Estimated },
        if calibrated {
            "front-driven Fiesta proxy geometry; effective radius fitted on calibration V-Vw12; tire model not fitted"
        } else {
            "front-driven Fiesta proxy geometry and analogous tire-radius estimate; tire model not fitted"
        },
    );
    definition.provenance.rear_wheels_and_tires = replace(
        definition.provenance.rear_wheels_and_tires,
        if calibrated { ParameterOrigin::Fitted } else { ParameterOrigin::Estimated },
        if calibrated {
            "Fiesta proxy rear geometry; effective radius fitted on calibration V-Vw12; tire model not fitted"
        } else {
            "Fiesta proxy rear geometry and analogous tire-radius estimate; tire model not fitted"
        },
    );
    definition.provenance.suspension = replace(
        definition.provenance.suspension,
        ParameterOrigin::Estimated,
        "unchanged engineering-reference suspension as explicit Fiesta proxy estimate; not dataset fitted",
    );
    definition.provenance.brakes = replace(
        definition.provenance.brakes,
        ParameterOrigin::Estimated,
        "Fiesta proxy front/rear brake capacities; 80 psi pedal normalization is an input-map assumption",
    );
    definition.provenance.engine = replace(
        definition.provenance.engine,
        ParameterOrigin::Estimated,
        "authored small-ICE proxy torque curve; exact IO-VNBD engine variant is unresolved",
    );
    definition.provenance.transmission_and_clutch = replace(
        definition.provenance.transmission_and_clutch,
        if calibrated { ParameterOrigin::Fitted } else { ParameterOrigin::Derived },
        if calibrated {
            "gearbox ratios conditional on assumed 3.89 final drive; overall ratios fitted only on calibration V-Vw12/V-Vfb02c"
        } else {
            "analogous 2015 Fiesta PowerShift manufacturer ratios; does not confirm IO-VNBD exact variant"
        },
    );
    definition.provenance.fuel_system = replace(
        definition.provenance.fuel_system,
        ParameterOrigin::Estimated,
        "35 kg capacity and 30 kg initialized fuel are explicit proxy estimates; fuel load not measured",
    );
    definition
}

fn simulate(reference: &TelemetryTable, run: Run, calibrated: bool) -> Result<TelemetryTable, Box<dyn Error>> {
    let mut world = PhysicsWorld::new(SimulationConfig { automatic_lod: false, ..SimulationConfig::default() });
    world.add_vehicle(fiesta_proxy(calibrated));
    let vehicle = &mut world.vehicles[0];
    vehicle.driver_aids.abs_enabled = true;
    vehicle.driver_aids.traction_control_enabled = false;
    vehicle.driver_aids.stability_control_enabled = false;
    vehicle.state.powertrain.fuel_kg = 30.0;
    let pressures_psi = match run.pressure {
        'A' => [15.0, 16.0, 14.0, 14.0],
        'C' => [33.0, 33.0, 27.0, 31.0],
        'D' => [33.0, 33.0, 26.0, 26.0],
        _ => [32.0; 4],
    };
    for (wheel, pressure) in vehicle.state.wheels.iter_mut().zip(pressures_psi) {
        wheel.tire.pressure_pa = pressure * PSI_TO_PA;
    }
    // Neutral stationary pre-roll settles suspension and ordinary thermal
    // state. Only after it completes is the measured t0 state assigned; there
    // are no later state injections.
    world
        .set_input_unrecorded(0, DriverInput::default())
        .map_err(|error| format!("{} pre-roll input failed: {error:?}", run.id))?;
    world.step_fixed(2_000).map_err(|error| format!("{} pre-roll failed: {error:?}", run.id))?;
    world.time_s = 0.0;
    world.step_index = 0;
    world.recorded_inputs.clear();
    let vehicle = &mut world.vehicles[0];
    vehicle.state.simulation_time_s = 0.0;
    let speed = reference.channels["speed_mps"][0];
    vehicle.state.linear_velocity_mps = Vec3::new(0.0, 0.0, -speed);
    let wheel_names = ["wheel_omega_fl_rad_s", "wheel_omega_fr_rad_s", "wheel_omega_rl_rad_s", "wheel_omega_rr_rad_s"];
    for (wheel, channel) in vehicle.state.wheels.iter_mut().zip(wheel_names) {
        wheel.angular_velocity_rad_s = reference.channels[channel][0];
    }
    vehicle.state.powertrain.engine_rpm = reference.channels["engine_rpm"][0].clamp(800.0, 6_550.0);
    vehicle.state.powertrain.gear = gear_at(reference, 0);
    vehicle.state.powertrain.clutch_engagement = 1.0;
    vehicle.previous_position_m = vehicle.state.position_m;
    let mut candidate = empty_candidate(reference.time_s.clone());
    push_sample(&mut candidate, &world);
    for interval in 0..reference.time_s.len() - 1 {
        let dt = reference.time_s[interval + 1] - reference.time_s[interval];
        let steps = (dt / world.config.fixed_dt_s).round() as u32;
        if (steps as f64 * world.config.fixed_dt_s - dt).abs() > 1.0e-8 {
            return Err(format!("{} interval {interval} is not an integer number of 1ms steps", run.id).into());
        }
        for step in 0..steps {
            let fraction = step as f64 / steps as f64;
            let input = DriverInput {
                steering: steering_input(reference, interval, fraction),
                throttle: linear(reference, "accelerator_fraction", interval, fraction).clamp(0.0, 1.0),
                brake: (linear(reference, "brake_pressure_pa", interval, fraction) / (80.0 * PSI_TO_PA))
                    .clamp(0.0, 1.0),
                clutch: 0.0,
                handbrake: reference.channels["handbrake_fraction"][interval].clamp(0.0, 1.0),
                gear_request: gear_at(reference, interval),
            };
            world.set_input_unrecorded(0, input).map_err(|error| format!("{} input step failed: {error:?}", run.id))?;
            world.step_fixed(1).map_err(|error| format!("{} physics step failed: {error:?}", run.id))?;
        }
        push_sample(&mut candidate, &world);
    }
    candidate.validate()?;
    Ok(candidate)
}

fn steering_input(reference: &TelemetryTable, index: usize, fraction: f64) -> f64 {
    // Steering sensor location and ratio are not published. This frozen proxy
    // uses an explicitly assumed 14.5:1 steering-wheel/road-wheel ratio.
    (linear(reference, "steering_wheel_rad", index, fraction) / (14.5 * 0.54)).clamp(-1.0, 1.0)
}

fn linear(table: &TelemetryTable, channel: &str, index: usize, fraction: f64) -> f64 {
    let values = &table.channels[channel];
    values[index] + (values[index + 1] - values[index]) * fraction
}

fn gear_at(table: &TelemetryTable, index: usize) -> i8 {
    // Column 20 is evidence, not a documented command. Hold the last valid
    // observed 1..=6 value; before the first valid evidence (including the
    // stationary all-zero run) remain neutral instead of inventing a gear.
    table.channels["transmission_gear_evidence"][..=index]
        .iter()
        .rev()
        .map(|value| value.round() as i8)
        .find(|gear| (1..=6).contains(gear))
        .unwrap_or(0)
}

fn empty_candidate(time_s: Vec<f64>) -> TelemetryTable {
    let mut channels = BTreeMap::new();
    for name in [
        "speed_mps",
        "yaw_rate_rad_s",
        "longitudinal_acceleration_forward_mps2",
        "lateral_acceleration_right_mps2",
        "wheel_omega_fl_rad_s",
        "wheel_omega_fr_rad_s",
        "wheel_omega_rl_rad_s",
        "wheel_omega_rr_rad_s",
        "engine_rpm",
    ] {
        channels.insert(name.to_owned(), Vec::with_capacity(time_s.len()));
    }
    TelemetryTable { time_s, channels }
}

fn push_sample(table: &mut TelemetryTable, world: &PhysicsWorld) {
    let vehicle = &world.vehicles[0];
    let body_acceleration = vehicle.state.orientation.conjugate().rotate(vehicle.telemetry.acceleration_mps2);
    let values = [
        ("speed_mps", vehicle.state.linear_velocity_mps.length()),
        ("yaw_rate_rad_s", vehicle.state.angular_velocity_rad_s.y),
        ("longitudinal_acceleration_forward_mps2", -body_acceleration.z),
        ("lateral_acceleration_right_mps2", body_acceleration.x),
        ("wheel_omega_fl_rad_s", vehicle.state.wheels[0].angular_velocity_rad_s),
        ("wheel_omega_fr_rad_s", vehicle.state.wheels[1].angular_velocity_rad_s),
        ("wheel_omega_rl_rad_s", vehicle.state.wheels[2].angular_velocity_rad_s),
        ("wheel_omega_rr_rad_s", vehicle.state.wheels[3].angular_velocity_rad_s),
        ("engine_rpm", vehicle.state.powertrain.engine_rpm),
    ];
    for (name, value) in values {
        table.channels.get_mut(name).unwrap().push(value);
    }
}

fn scored_mappings() -> Vec<ChannelMapping> {
    [
        ("speed_mps", "m/s", 5.0, 0.5),
        ("yaw_rate_rad_s", "rad/s", 0.2, 0.5),
        ("longitudinal_acceleration_forward_mps2", "m/s^2", 2.0, 0.5),
        ("lateral_acceleration_right_mps2", "m/s^2", 2.0, 0.5),
        ("wheel_omega_fl_rad_s", "rad/s", 20.0, 0.3),
        ("wheel_omega_fr_rad_s", "rad/s", 20.0, 0.3),
        ("wheel_omega_rl_rad_s", "rad/s", 20.0, 0.3),
        ("wheel_omega_rr_rad_s", "rad/s", 20.0, 0.3),
        ("engine_rpm", "rpm", 1_000.0, 0.5),
    ]
    .into_iter()
    .map(|(name, unit, normalization_scale, lag_search_bound_s)| ChannelMapping {
        reference_channel: name.to_owned(),
        candidate_channel: name.to_owned(),
        output_name: name.to_owned(),
        unit: unit.to_owned(),
        normalization_scale,
        lag_search_bound_s,
        reference_resampling: Resampling::Linear,
        candidate_resampling: Resampling::Linear,
    })
    .collect()
}

fn candidate_manifest(
    reference: &DatasetManifest,
    run: Run,
    calibrated: bool,
    table: &TelemetryTable,
) -> DatasetManifest {
    let mut manifest = reference.clone();
    let revision = if calibrated { PROXY_CALIBRATED_REVISION } else { PROXY_BASELINE_REVISION };
    manifest.dataset_id = format!("{}-{revision}", run.id);
    manifest.source_title = "my-physics deterministic 1 ms simulation output".to_owned();
    manifest.source_uri = "https://github.com/takahirox/my-physics".to_owned();
    manifest.license_id = "project-license-TBD".to_owned();
    manifest.license_verified = false;
    manifest.content_checksum = format!("sha256:{}", telemetry_fingerprint(table));
    manifest.vehicle_id = "Ford Fiesta Titanium FWD proxy; exact year/engine unknown".to_owned();
    manifest.session_id = format!("{}-simulation", run.id);
    manifest.timestamp_semantics = "reference sensor timestamps sampled from fixed 1 ms plant".to_owned();
    manifest.split_group = format!("{}-candidate", run.id);
    manifest.provenance = ProvenanceRecord {
        origin: "deterministic-common-physics-plant".to_owned(),
        source: format!(
            "my-physics {} git {} with {INPUT_RECONSTRUCTION_REVISION}",
            env!("CARGO_PKG_VERSION"),
            software_revision()
        ),
        revision: revision.to_owned(),
        notes:
            "Stationary pre-roll, measured t0 initialization, then inputs only; no post-t0 nudging or force correction"
                .to_owned(),
    };
    manifest.fields = scored_mappings()
        .into_iter()
        .map(|mapping| {
            field(
                &mapping.output_name,
                &mapping.output_name,
                reference.fields.iter().find(|field| field.canonical_name == mapping.output_name).unwrap().quantity,
                &mapping.unit,
                &reference.fields.iter().find(|field| field.canonical_name == mapping.output_name).unwrap().frame.0,
                FieldRole::Observation,
                Resampling::Linear,
                ValueTransform::Identity,
                mapping.normalization_scale,
                mapping.lag_search_bound_s,
            )
        })
        .chain(std::iter::once(field(
            "time_s",
            "time_s",
            Quantity::Time,
            "s",
            "sensor-clock-relative-to-run-t0",
            FieldRole::Time,
            Resampling::Linear,
            ValueTransform::Identity,
            1.0,
            0.0,
        )))
        .collect();
    manifest
}

fn telemetry_fingerprint(table: &TelemetryTable) -> String {
    let mut bytes = Vec::new();
    for value in &table.time_s {
        bytes.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    for (name, values) in &table.channels {
        bytes.extend_from_slice(name.as_bytes());
        bytes.push(0);
        for value in values {
            bytes.extend_from_slice(&value.to_bits().to_le_bytes());
        }
    }
    sha256_hex(&bytes)
}

fn run_provenance(
    run: Run,
    reference: &TelemetryTable,
    baseline: &DatasetManifest,
    calibrated: &DatasetManifest,
) -> String {
    format!(
        "{{\"schema_version\":1,\"physics_dt_s\":0.001,\"physics_mode\":\"fixed timestep; automatic LOD disabled\",\"physics_package_version\":\"{}\",\"software_git_revision\":\"{}\",\"software_worktree_state\":\"{}\",\"input_reconstruction_revision\":\"{}\",\"driver_aids\":{{\"abs\":true,\"traction_control\":false,\"stability_control\":false}},\"pressure_condition\":\"{}\",\"pressure_source\":\"published IO-VNBD scenario table; FL/FR/RL/RR mapped explicitly\",\"applied_1ms_input_fingerprint_sha256\":\"{}\",\"normalized_reference_table_fingerprint_sha256\":\"{}\",\"baseline_output_fingerprint\":\"{}\",\"calibrated_output_fingerprint\":\"{}\",\"initialization\":\"stationary neutral pre-roll; measured state at t0 only; no later state injection\"}}\n",
        env!("CARGO_PKG_VERSION"),
        software_revision(),
        software_worktree_state(),
        INPUT_RECONSTRUCTION_REVISION,
        run.pressure,
        input_sequence_fingerprint(reference),
        telemetry_fingerprint(reference),
        baseline.content_checksum,
        calibrated.content_checksum,
    )
}

fn software_revision() -> String {
    git_output(&["rev-parse", "HEAD"]).unwrap_or_else(|| "unavailable".to_owned())
}

fn software_worktree_state() -> &'static str {
    match git_output(&["status", "--porcelain", "--untracked-files=no"]) {
        Some(output) if output.is_empty() => "clean",
        Some(_) => "dirty",
        None => "unavailable",
    }
}

fn git_output(arguments: &[&str]) -> Option<String> {
    let output = Command::new("git").args(arguments).current_dir(env!("CARGO_MANIFEST_DIR")).output().ok()?;
    output.status.success().then(|| String::from_utf8_lossy(&output.stdout).trim().to_owned())
}

fn input_sequence_fingerprint(reference: &TelemetryTable) -> String {
    let mut bytes = INPUT_RECONSTRUCTION_REVISION.as_bytes().to_vec();
    bytes.extend_from_slice(&0.001_f64.to_bits().to_le_bytes());
    for interval in 0..reference.time_s.len() - 1 {
        let steps = ((reference.time_s[interval + 1] - reference.time_s[interval]) / 0.001).round() as u32;
        for step in 0..steps {
            let fraction = step as f64 / steps as f64;
            let input = DriverInput {
                steering: steering_input(reference, interval, fraction),
                throttle: linear(reference, "accelerator_fraction", interval, fraction).clamp(0.0, 1.0),
                brake: (linear(reference, "brake_pressure_pa", interval, fraction) / (80.0 * PSI_TO_PA))
                    .clamp(0.0, 1.0),
                clutch: 0.0,
                handbrake: reference.channels["handbrake_fraction"][interval].clamp(0.0, 1.0),
                gear_request: gear_at(reference, interval),
            };
            for value in [input.steering, input.throttle, input.brake, input.clutch, input.handbrake] {
                bytes.extend_from_slice(&value.to_bits().to_le_bytes());
            }
            bytes.push(input.gear_request as u8);
        }
    }
    sha256_hex(&bytes)
}

fn write_simulation_csv(path: &Path, table: &TelemetryTable) -> Result<(), Box<dyn Error>> {
    let names: Vec<_> = table.channels.keys().cloned().collect();
    let mut output = format!("time_s,{}\n", names.join(","));
    for index in 0..table.time_s.len() {
        write!(output, "{:.9}", table.time_s[index])?;
        for name in &names {
            write!(output, ",{:.9}", table.channels[name][index])?;
        }
        output.push('\n');
    }
    fs::write(path, output)?;
    Ok(())
}

fn event_metrics(reference: &TelemetryTable, baseline: &TelemetryTable, calibrated: &TelemetryTable) -> String {
    let brake = &reference.channels["brake_pressure_pa"];
    let onset = brake.iter().position(|value| *value > 5.0 * PSI_TO_PA);
    let time = onset.map(|index| reference.time_s[index]);
    let minimum_after = |table: &TelemetryTable| {
        onset.map(|start| {
            let index = table.channels["speed_mps"][start..]
                .iter()
                .enumerate()
                .min_by(|a, b| a.1.total_cmp(b.1))
                .map_or(start, |(i, _)| start + i);
            (table.channels["speed_mps"][index], table.time_s[index])
        })
    };
    let measured = minimum_after(reference);
    let base = minimum_after(baseline);
    let cal = minimum_after(calibrated);
    format!(
        "{{\"schema_version\":1,\"brake_onset_sensor_time_s\":{},\"measured_min_speed_after_onset\":{},\"baseline_min_speed_after_onset\":{},\"calibrated_min_speed_after_onset\":{},\"note\":\"Event is reported only when measured pressure exceeds 5 psi; no time warp or fitted event shift.\"}}\n",
        json_option(time),
        pair_json(measured),
        pair_json(base),
        pair_json(cal)
    )
}

fn json_option(value: Option<f64>) -> String {
    value.map_or_else(|| "null".to_owned(), |value| format!("{value:.9}"))
}

fn pair_json(value: Option<(f64, f64)>) -> String {
    value
        .map_or_else(|| "null".to_owned(), |(speed, time)| format!("{{\"speed_mps\":{speed:.9},\"time_s\":{time:.9}}}"))
}

fn timeseries_svg(run_id: &str, aligned: &my_physics::correlation::AlignedSeries) -> String {
    let width = 1_120.0;
    let panel_height = 145.0;
    let left = 160.0;
    let right = 24.0;
    let top = 45.0;
    let height = top + panel_height * aligned.mappings.len() as f64 + 25.0;
    let mut svg = format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{width}\" height=\"{height}\" viewBox=\"0 0 {width} {height}\"><rect width=\"100%\" height=\"100%\" fill=\"#10141c\"/><style>text{{fill:#dce6f2;font:12px monospace}} .grid{{stroke:#334155;stroke-width:1}} .ref{{fill:none;stroke:#4cc9f0;stroke-width:1.5}} .sim{{fill:none;stroke:#ffb703;stroke-width:1.5}}</style><text x=\"16\" y=\"24\">{run_id} · measured (cyan) vs calibrated common plant (amber) · sensor timestamps</text>"
    );
    let start = aligned.time_s[0];
    let duration = (aligned.time_s[aligned.time_s.len() - 1] - start).max(0.1);
    for (panel, mapping) in aligned.mappings.iter().enumerate() {
        let y0 = top + panel as f64 * panel_height;
        let reference = &aligned.reference[&mapping.output_name];
        let candidate = &aligned.candidate[&mapping.output_name];
        let min = reference.iter().chain(candidate).copied().fold(f64::INFINITY, f64::min);
        let max = reference.iter().chain(candidate).copied().fold(f64::NEG_INFINITY, f64::max);
        let range = (max - min).max(1.0e-9);
        write!(svg, "<text x=\"8\" y=\"{}\">{} [{}]</text><line class=\"grid\" x1=\"{left}\" y1=\"{y0}\" x2=\"{}\" y2=\"{y0}\"/><line class=\"grid\" x1=\"{left}\" y1=\"{}\" x2=\"{}\" y2=\"{}\"/>", y0 + 18.0, mapping.output_name, mapping.unit, width-right, y0+panel_height-18.0, width-right, y0+panel_height-18.0).unwrap();
        let points = |values: &[f64]| {
            let stride = values.len().div_ceil(800).max(1);
            values
                .iter()
                .enumerate()
                .step_by(stride)
                .map(|(index, value)| {
                    let x = left + (aligned.time_s[index] - start) / duration * (width - left - right);
                    let y = y0 + 8.0 + (max - value) / range * (panel_height - 34.0);
                    format!("{x:.2},{y:.2}")
                })
                .collect::<Vec<_>>()
                .join(" ")
        };
        write!(
            svg,
            "<polyline class=\"ref\" points=\"{}\"/><polyline class=\"sim\" points=\"{}\"/>",
            points(reference),
            points(candidate)
        )
        .unwrap();
    }
    svg.push_str("</svg>\n");
    svg
}

fn summary_json(summaries: &[(Run, f64, f64)]) -> String {
    let mut output = format!(
        "{{\"schema_version\":1,\"mapping_revision\":\"{MAPPING_REVISION}\",\"proxy_baseline_revision\":\"{PROXY_BASELINE_REVISION}\",\"proxy_calibrated_revision\":\"{PROXY_CALIBRATED_REVISION}\",\"calibration_source_hashes\":\"{CALIBRATION_HASHES}\",\"score_definition\":\"RMS of per-channel RMSE divided by declared physical normalization scale; not a dimensional raw-error sum\",\"runs\":["
    );
    for (index, (run, baseline, calibrated)) in summaries.iter().enumerate() {
        if index != 0 {
            output.push(',');
        }
        write!(output, "{{\"run_id\":\"{}\",\"split\":\"{}\",\"baseline_normalized_rmse\":{baseline:.9},\"calibrated_normalized_rmse\":{calibrated:.9},\"change\":{:.9}}}", run.id, run.split, calibrated-baseline).unwrap();
    }
    output.push_str("],\"disclaimer\":\"Proxy correlation only; exact vehicle variant, load, road grade, wind, tire construction and steering sensor ratio are unresolved. No measured tire fit, certification or safety claim.\"}\n");
    output
}

fn parameter_artifact() -> ParameterEstimateArtifact {
    let sourced = |name: &str,
                   value: f64,
                   unit: &str,
                   origin: EstimateOrigin,
                   source: &str,
                   min: f64,
                   max: f64,
                   uncertainty: &str| ParameterEstimate {
        parameter: name.to_owned(),
        value,
        unit: unit.to_owned(),
        origin,
        source: source.to_owned(),
        source_revision: "io-vnbd-fiesta-proxy-provenance-v1".to_owned(),
        source_split: None,
        uncertainty: uncertainty.to_owned(),
        valid_min: min,
        valid_max: max,
    };
    let fitted = |name: &str, value: f64, unit: &str, min: f64, max: f64, uncertainty: &str| ParameterEstimate {
        parameter: name.to_owned(),
        value,
        unit: unit.to_owned(),
        origin: EstimateOrigin::Fitted,
        source: CALIBRATION_HASHES.to_owned(),
        source_revision: "io-vnbd-calibration-fit-v1".to_owned(),
        source_split: Some(DatasetSplit::Training),
        uncertainty: uncertainty.to_owned(),
        valid_min: min,
        valid_max: max,
    };
    ParameterEstimateArtifact {
        schema_version: 1,
        artifact_id: "io-vnbd-fiesta-proxy-cal-v1".to_owned(),
        vehicle_proxy: "Ford Fiesta Titanium FWD proxy; exact IO-VNBD year/engine/trim loading unknown".to_owned(),
        frozen_revision: PROXY_CALIBRATED_REVISION.to_owned(),
        limitations: vec![
            "Only 10 Hz content is identifiable; ABS, tire relaxation, shifting transients and thermal states were not fitted".to_owned(),
            "Steering sensor location and ratio are unobserved; 14.5:1 is an explicit estimate, not a fit claim".to_owned(),
            "Overall driveline ratios are identifiable; gearbox and final-drive factors are not separately identifiable".to_owned(),
        ],
        parameters: vec![
            sourced("layout.front_wheel_drive", 1.0, "boolean", EstimateOrigin::Literature, "Onyekpe et al. 2021: Ford Fiesta Titanium FWD", 1.0, 1.0, "published layout; exact variant unresolved"),
            sourced("chassis.dry_mass", 1_150.0, "kg", EstimateOrigin::Estimated, "Fiesta-class proxy estimate", 900.0, 1_400.0, "test mass/loading unmeasured"),
            sourced("chassis.cg_local", 0.0, "m", EstimateOrigin::Assumed, "neutral CG-origin proxy", -0.5, 0.5, "CG coordinates unmeasured"),
            sourced("chassis.inertia_pitch", 1_250.0, "kg*m^2", EstimateOrigin::Estimated, "Fiesta-class rigid-body proxy", 500.0, 2_500.0, "all inertia components unmeasured"),
            sourced("geometry.wheelbase", 2.489, "m", EstimateOrigin::Manufacturer, "analogous 2015 Ford Fiesta specification; not exact variant confirmation", 2.3, 2.7, "generation assumption"),
            sourced("geometry.track", 1.470, "m", EstimateOrigin::Manufacturer, "analogous 2015 Ford Fiesta specification; not exact variant confirmation", 1.3, 1.7, "generation assumption"),
            fitted("sensor.yaw_bias", 0.05955, "deg/s", -1.0, 1.0, "stationary V-Vw1 mean"),
            fitted("sensor.longitudinal_acceleration_bias", -0.04107, "g", -0.2, 0.2, "stationary V-Vw1 mean"),
            fitted("sensor.lateral_acceleration_bias", 0.03206, "g", -0.2, 0.2, "stationary V-Vw1 mean"),
            fitted("sensor.brake_pressure_bias", -0.0076, "psi", -1.0, 1.0, "stationary V-Vw1 mean; adapter adds 0.0076 psi"),
            fitted("wheel.effective_rolling_radius", 0.278756, "m", 0.2, 0.4, "V-Vw12 median; MAD 0.000425 m"),
            sourced("steering.maximum_road_wheel_angle", 0.54, "rad", EstimateOrigin::Assumed, "Fiesta proxy steering travel", 0.3, 0.8, "not observed"),
            sourced("steering.ratio", 14.5, "ratio", EstimateOrigin::Assumed, "frozen input reconstruction proxy", 8.0, 24.0, "sensor location/ratio unresolved"),
            sourced("steering.sensor_zero", 4.8, "deg", EstimateOrigin::Assumed, "frozen calibration-stage input reconstruction", -30.0, 30.0, "not independently identifiable; adapter subtracts 4.8 deg"),
            sourced("suspension.spring_rate", 40_000.0, "N/m", EstimateOrigin::Estimated, "engineering-reference proxy retained", 10_000.0, 100_000.0, "not fitted"),
            sourced("suspension.damper_rate", 4_200.0, "N*s/m", EstimateOrigin::Estimated, "engineering-reference proxy retained", 1_000.0, 12_000.0, "not fitted"),
            sourced("brakes.front_capacity", 2_300.0, "N*m", EstimateOrigin::Estimated, "Fiesta-class brake proxy", 500.0, 5_000.0, "not fitted"),
            sourced("brakes.rear_capacity", 1_300.0, "N*m", EstimateOrigin::Estimated, "Fiesta-class brake proxy", 300.0, 4_000.0, "not fitted"),
            sourced("input.brake_full_scale", 80.0, "psi", EstimateOrigin::Assumed, "fixed input normalization from calibration excitation range", 20.0, 150.0, "not hydraulic-system identification"),
            sourced("aerodynamics.frontal_area", 2.10, "m^2", EstimateOrigin::Estimated, "Fiesta-class proxy", 1.5, 3.0, "unmeasured"),
            sourced("aerodynamics.drag_coefficient", 0.33, "ratio", EstimateOrigin::Estimated, "Fiesta-class proxy", 0.2, 0.6, "unmeasured"),
            sourced("aerodynamics.lift_coefficient", 0.0, "ratio", EstimateOrigin::Assumed, "neutral aero proxy", -0.5, 0.5, "unmeasured"),
            sourced("engine.idle_speed", 800.0, "rpm", EstimateOrigin::Estimated, "authored small-ICE proxy", 600.0, 1_200.0, "exact engine unknown"),
            sourced("engine.redline", 6_500.0, "rpm", EstimateOrigin::Estimated, "authored small-ICE proxy", 4_500.0, 8_000.0, "exact engine unknown"),
            sourced("engine.peak_torque", 125.0, "N*m", EstimateOrigin::Estimated, "authored small-ICE proxy curve", 60.0, 250.0, "not dyno-fitted"),
            sourced("transmission.final_drive", 3.89, "ratio", EstimateOrigin::Manufacturer, "analogous 2015 Fiesta PowerShift specification; conditional assumption", 2.0, 6.0, "exact final drive unresolved"),
            sourced("transmission.gear1_ratio", 3.917, "ratio", EstimateOrigin::Manufacturer, "analogous 2015 Fiesta PowerShift specification; gear 1 unobserved in calibration", 1.0, 6.0, "exact gearbox unresolved"),
            fitted("driveline.overall_ratio.gear2", 9.3659, "ratio", 1.0, 20.0, "V-Vw12 and V-Vfb02c robust ratio"),
            fitted("driveline.overall_ratio.gear3", 6.4020, "ratio", 1.0, 20.0, "V-Vw12 and V-Vfb02c robust ratio"),
            fitted("driveline.overall_ratio.gear4", 4.4083, "ratio", 1.0, 20.0, "V-Vw12 and V-Vfb02c robust ratio"),
            fitted("driveline.overall_ratio.gear5", 3.1507, "ratio", 1.0, 20.0, "V-Vw12 and V-Vfb02c robust ratio"),
            fitted("driveline.overall_ratio.gear6", 2.6001, "ratio", 1.0, 20.0, "V-Vw12 and V-Vfb02c robust ratio"),
            sourced("clutch.torque_capacity", 300.0, "N*m", EstimateOrigin::Estimated, "Fiesta-class clutch proxy", 100.0, 600.0, "not fitted"),
            sourced("tire.cornering_stiffness_scale", 1.0, "ratio", EstimateOrigin::Assumed, "common reference tire retained", 0.5, 2.0, "construction unobserved; not fitted"),
            sourced("tire.peak_grip_scale", 1.0, "ratio", EstimateOrigin::Assumed, "common reference tire retained", 0.5, 2.0, "surface/tire unobserved; not fitted"),
            sourced("fuel.capacity", 35.0, "kg", EstimateOrigin::Estimated, "Fiesta proxy", 20.0, 60.0, "unmeasured"),
            sourced("fuel.initial_mass", 30.0, "kg", EstimateOrigin::Assumed, "fixed consistent t0 initialization", 0.0, 35.0, "fuel load unmeasured"),
        ],
    }
}

fn fit_trace() -> &'static str {
    "stage,calibration_runs,objective,decision\nsource-bias,V-Vw1,stationary means,yaw/ax/ay/brake bias frozen\nrolling-radius,V-Vw12,median wheel-speed radius,0.278756 m frozen\noverall-ratio,V-Vw12+V-Vfb02c,robust RPM/wheel ratio,gears 2-6 frozen; gear 1 unobserved\ninput-map,V-Vfb02c,excitation observability,80 psi full brake and 14.5 steering ratio remain proxy assumptions\n"
}

fn limitations() -> &'static str {
    "# IO-VNBD correlation limitations\n\n- Physical plausibility and correlation only; not measured tire fit, certification, safety validation or proof of general validity.\n- Exact Fiesta model year, engine, mass/loading, CG, inertia, tire construction, steering sensor location/ratio, road grade and wind are unresolved.\n- The adapter freezes signs, sensor biases and input mapping from calibration; no validation/holdout-specific fit or state nudging occurs.\n- Positive source lateral acceleration is mapped to negative internal body-right acceleration; positive yaw remains internal +Y/left.\n- Gear Requested is used only as transmission-gear evidence. The other Gear column remains quarantined because its semantics conflict with observed behavior.\n- Ten-hertz data cannot identify ABS, tire-relaxation, shift transient or thermal dynamics.\n- Pressure-A holdouts are confounded with wet/muddy routes. Water depth is unmeasured, so the primary correlation keeps the road water state neutral rather than inventing a run-specific correction.\n- Raw dataset redistribution is disabled because the pinned upstream repository has no explicit dataset license.\n"
}

fn verify_acquisition_manifest() -> Result<(), Box<dyn Error>> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("validation/io_vnbd/acquisition.tsv");
    let text = fs::read_to_string(&path)?;
    let rows: Vec<Vec<&str>> = text
        .lines()
        .filter(|line| !line.starts_with('#') && !line.starts_with("run_id\t") && !line.trim().is_empty())
        .map(|line| line.split('\t').collect())
        .collect();
    for run in RUNS {
        let row = rows
            .iter()
            .find(|row| row.first() == Some(&run.id))
            .ok_or_else(|| format!("{} is absent from {}", run.id, path.display()))?;
        if row.len() != 9
            || row[1] != run.split
            || row[4] != run.pressure.to_string()
            || row[5].parse::<u64>()? != run.bytes
            || row[6] != run.checksum
        {
            return Err(format!("{} disagrees with pinned acquisition manifest", run.id).into());
        }
    }
    Ok(())
}

const IO_VNBD_HEADER: [&str; 29] = [
    "No of GPS Satellites Available",
    "Time Since Start of Day (seconds)",
    "Latitude (degrees)",
    "Longitude (degrees)",
    "Velocity (km/hr)",
    "Heading (degrees)",
    "Height (km)",
    "Vertical velocity (km/hr)",
    "Sample period (seconds)",
    "Steering Angle (degrees)",
    "Wheel Speed Front Left (rad/sec)",
    "Wheel Speed Front Right (rad/sec)",
    "Wheel Speed Rear Left (rad/sec)",
    "Wheel Speed Rear Right (rad/sec)",
    "Yaw Rate (deg/sec)",
    "Indicated Vehicle Speed (km/hr)",
    "Indicated Longitudinal Acceleration (g)",
    "Indicated Lateral Acceleration (g)",
    "Handbrake (0 or 1)",
    "Gear Requested (Number fof gear employed 1-5)",
    "Gear (Number fof gear employed 1-5)",
    "Engine Speed (rev/min)",
    "Coolant Temperature (degrees)",
    "Clutch Position (0 or 1)",
    "Brake Pressure (psi)",
    "Brake Position (0 or 1)",
    "Battery Voltage (volts)",
    "Air Temperature (degrees)",
    "Accelerator Pedal Position (0 or 1)",
];

fn verify_exact_header(path: &Path) -> Result<(), Box<dyn Error>> {
    let mut header = String::new();
    BufReader::new(fs::File::open(path)?).read_line(&mut header)?;
    let actual: Vec<_> = header.trim_end_matches(['\r', '\n']).split(',').map(str::trim).collect();
    if actual != IO_VNBD_HEADER {
        return Err(format!("{} does not have the pinned 29-column IO-VNBD header", path.display()).into());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sha256_matches_nist_short_message_vector() {
        assert_eq!(sha256_hex(b"abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    }

    #[test]
    fn frozen_adapter_frame_signs_are_explicit() {
        let manifest = reference_manifest(RUNS[0]);
        let longitudinal = manifest
            .fields
            .iter()
            .find(|field| field.canonical_name == "longitudinal_acceleration_forward_mps2")
            .unwrap();
        let lateral =
            manifest.fields.iter().find(|field| field.canonical_name == "lateral_acceleration_right_mps2").unwrap();
        let yaw = manifest.fields.iter().find(|field| field.canonical_name == "yaw_rate_rad_s").unwrap();
        assert_eq!(longitudinal.frame.0, "body-forward (-Z)");
        assert_eq!(lateral.frame.0, "body +X right; source sign negated");
        assert_eq!(yaw.frame.0, "internal +Y; positive-left");
        assert!(matches!(lateral.transform, ValueTransform::Affine { scale, .. } if scale < 0.0));
        assert!(matches!(yaw.transform, ValueTransform::Affine { scale, .. } if scale > 0.0));
    }

    #[test]
    fn proxy_uses_common_fwd_plant_and_calibration_only_changes_declared_values() {
        let baseline = fiesta_proxy(false);
        let calibrated = fiesta_proxy(true);
        assert_eq!(calibrated.wheels.map(|wheel| wheel.driven), [true, true, false, false]);
        assert_eq!(calibrated.wheels.map(|wheel| wheel.radius_m), [0.278756; 4]);
        assert_ne!(baseline.transmission.gear_ratios, calibrated.transmission.gear_ratios);
        assert_eq!(baseline.chassis, calibrated.chassis);
        assert!(calibrated.provenance.is_complete());
        parameter_artifact().validate().unwrap();
    }
}
