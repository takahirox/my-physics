use my_physics::correlation::{
    AlignmentSpec, ChannelMapping, ClockCorrection, CorrelationError, CorrelationPurpose, CsvTelemetryAdapter,
    DatasetFormat, DatasetManifest, FieldRole, TelemetryAdapter, align_and_resample, evaluate, write_report_artifacts,
};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

fn usage() -> &'static str {
    "usage: correlate-telemetry --reference-manifest FILE --reference FILE --candidate-manifest FILE --candidate FILE --purpose parameter-fitting|model-selection|final-evaluation --report-id ID --output DIR [--sample-period SECONDS] [--reference-offset SECONDS] [--candidate-offset SECONDS] [--reference-clock-scale SCALE] [--candidate-clock-scale SCALE] [--reference-latency SECONDS] [--candidate-latency SECONDS] [--channel REFERENCE:CANDIDATE:OUTPUT]... [--require-publishable-license]"
}

#[derive(Debug)]
struct Arguments {
    reference_manifest: PathBuf,
    reference_path: PathBuf,
    candidate_manifest: PathBuf,
    candidate_path: PathBuf,
    purpose: CorrelationPurpose,
    report_id: String,
    output: PathBuf,
    sample_period_s: f64,
    reference_offset_s: f64,
    candidate_offset_s: f64,
    reference_clock_scale: f64,
    candidate_clock_scale: f64,
    reference_latency_s: f64,
    candidate_latency_s: f64,
    channels: Vec<(String, String, String)>,
    require_publishable_license: bool,
}

fn main() {
    if let Err(error) = execute() {
        eprintln!("{error}\n{}", usage());
        std::process::exit(2);
    }
}

fn execute() -> Result<(), Box<dyn std::error::Error>> {
    let arguments = parse_arguments(env::args().skip(1))?;
    let reference_manifest = read_manifest(&arguments.reference_manifest)?;
    let candidate_manifest = read_manifest(&arguments.candidate_manifest)?;
    if arguments.require_publishable_license {
        reference_manifest.require_license_for_publication()?;
    }
    reference_manifest.require_purpose(arguments.purpose)?;
    let adapter = CsvTelemetryAdapter;
    if reference_manifest.format != DatasetFormat::Csv || candidate_manifest.format != DatasetFormat::Csv {
        return Err(CorrelationError::AdapterUnavailable(
            "this build includes CSV only; register a reviewed Parquet/provider TelemetryAdapter".to_owned(),
        )
        .into());
    }
    let reference = adapter.read_path(&arguments.reference_path, &reference_manifest)?;
    let candidate = adapter.read_path(&arguments.candidate_path, &candidate_manifest)?;
    let mappings = build_mappings(&reference_manifest, &candidate_manifest, &arguments.channels)?;
    let specification = AlignmentSpec {
        revision: reference_manifest.alignment_revision.clone(),
        sample_period_s: arguments.sample_period_s,
        reference_clock: ClockCorrection {
            scale: arguments.reference_clock_scale,
            offset_s: arguments.reference_offset_s,
            declared_latency_s: arguments.reference_latency_s,
        },
        candidate_clock: ClockCorrection {
            scale: arguments.candidate_clock_scale,
            offset_s: arguments.candidate_offset_s,
            declared_latency_s: arguments.candidate_latency_s,
        },
        mappings,
    };
    let aligned = align_and_resample(&reference, &candidate, &specification)?;
    let report = evaluate(
        arguments.report_id,
        arguments.purpose,
        &reference_manifest,
        candidate_manifest.dataset_id,
        candidate_manifest.provenance.revision,
        &aligned,
    )?;
    write_report_artifacts(&arguments.output, &report, &aligned)?;
    println!("{}", report.to_json());
    Ok(())
}

fn read_manifest(path: &Path) -> Result<DatasetManifest, Box<dyn std::error::Error>> {
    Ok(DatasetManifest::from_text(&fs::read_to_string(path)?)?)
}

fn build_mappings(
    reference: &DatasetManifest,
    candidate: &DatasetManifest,
    requested: &[(String, String, String)],
) -> Result<Vec<ChannelMapping>, CorrelationError> {
    let triples: Vec<_> = if requested.is_empty() {
        reference
            .observation_fields()
            .map(|field| (field.canonical_name.clone(), field.canonical_name.clone(), field.canonical_name.clone()))
            .collect()
    } else {
        requested.to_vec()
    };
    let mut mappings = Vec::new();
    for (reference_name, candidate_name, output_name) in triples {
        let reference_field =
            reference.fields.iter().find(|field| field.canonical_name == reference_name).ok_or_else(|| {
                CorrelationError::InvalidManifest(format!("reference field {reference_name:?} is absent"))
            })?;
        let candidate_field =
            candidate.fields.iter().find(|field| field.canonical_name == candidate_name).ok_or_else(|| {
                CorrelationError::InvalidManifest(format!("candidate field {candidate_name:?} is absent"))
            })?;
        if reference_field.role == FieldRole::Time || candidate_field.role == FieldRole::Time {
            return Err(CorrelationError::InvalidManifest("time cannot be a metric channel".to_owned()));
        }
        if reference_field.quantity != candidate_field.quantity
            || reference_field.unit != candidate_field.unit
            || reference_field.frame != candidate_field.frame
        {
            return Err(CorrelationError::InvalidManifest(format!(
                "mapping {reference_name:?}->{candidate_name:?} has incompatible quantity/unit/frame; explicit conversion belongs in a reviewed adapter"
            )));
        }
        mappings.push(ChannelMapping {
            reference_channel: reference_name,
            candidate_channel: candidate_name,
            output_name,
            unit: reference_field.unit.0.clone(),
            normalization_scale: reference_field.metric_normalization_scale,
            lag_search_bound_s: reference_field.metric_lag_search_bound_s,
            reference_resampling: reference_field.resampling,
            candidate_resampling: candidate_field.resampling,
        });
    }
    Ok(mappings)
}

fn parse_arguments(arguments: impl Iterator<Item = String>) -> Result<Arguments, CorrelationError> {
    let mut reference_manifest = None;
    let mut reference_path = None;
    let mut candidate_manifest = None;
    let mut candidate_path = None;
    let mut purpose = None;
    let mut report_id = None;
    let mut output = None;
    let mut sample_period_s = 0.01;
    let mut reference_offset_s = 0.0;
    let mut candidate_offset_s = 0.0;
    let mut reference_clock_scale = 1.0;
    let mut candidate_clock_scale = 1.0;
    let mut reference_latency_s = 0.0;
    let mut candidate_latency_s = 0.0;
    let mut channels = Vec::new();
    let mut require_publishable_license = false;
    let mut arguments = arguments;
    while let Some(argument) = arguments.next() {
        let mut value = || {
            arguments.next().ok_or_else(|| CorrelationError::InvalidManifest(format!("{argument} requires a value")))
        };
        match argument.as_str() {
            "--reference-manifest" => reference_manifest = Some(PathBuf::from(value()?)),
            "--reference" => reference_path = Some(PathBuf::from(value()?)),
            "--candidate-manifest" => candidate_manifest = Some(PathBuf::from(value()?)),
            "--candidate" => candidate_path = Some(PathBuf::from(value()?)),
            "--purpose" => purpose = Some(CorrelationPurpose::parse(&value()?)?),
            "--report-id" => report_id = Some(value()?),
            "--output" => output = Some(PathBuf::from(value()?)),
            "--sample-period" => sample_period_s = parse_number(&argument, &value()?)?,
            "--reference-offset" => reference_offset_s = parse_number(&argument, &value()?)?,
            "--candidate-offset" => candidate_offset_s = parse_number(&argument, &value()?)?,
            "--reference-clock-scale" => reference_clock_scale = parse_number(&argument, &value()?)?,
            "--candidate-clock-scale" => candidate_clock_scale = parse_number(&argument, &value()?)?,
            "--reference-latency" => reference_latency_s = parse_number(&argument, &value()?)?,
            "--candidate-latency" => candidate_latency_s = parse_number(&argument, &value()?)?,
            "--channel" => {
                let channel = value()?;
                let parts: Vec<_> = channel.split(':').map(str::to_owned).collect();
                if parts.len() != 3 || parts.iter().any(String::is_empty) {
                    return Err(CorrelationError::InvalidManifest(format!(
                        "--channel expects REFERENCE:CANDIDATE:OUTPUT, got {channel:?}"
                    )));
                }
                channels.push((parts[0].clone(), parts[1].clone(), parts[2].clone()));
            }
            "--require-publishable-license" => require_publishable_license = true,
            "--help" | "-h" => return Err(CorrelationError::InvalidManifest(usage().to_owned())),
            _ => return Err(CorrelationError::InvalidManifest(format!("unknown argument {argument:?}"))),
        }
    }
    Ok(Arguments {
        reference_manifest: required(reference_manifest, "--reference-manifest")?,
        reference_path: required(reference_path, "--reference")?,
        candidate_manifest: required(candidate_manifest, "--candidate-manifest")?,
        candidate_path: required(candidate_path, "--candidate")?,
        purpose: required(purpose, "--purpose")?,
        report_id: required(report_id, "--report-id")?,
        output: required(output, "--output")?,
        sample_period_s,
        reference_offset_s,
        candidate_offset_s,
        reference_clock_scale,
        candidate_clock_scale,
        reference_latency_s,
        candidate_latency_s,
        channels,
        require_publishable_license,
    })
}

fn required<T>(value: Option<T>, name: &str) -> Result<T, CorrelationError> {
    value.ok_or_else(|| CorrelationError::InvalidManifest(format!("missing {name}")))
}

fn parse_number(label: &str, value: &str) -> Result<f64, CorrelationError> {
    value
        .parse()
        .map_err(|_| CorrelationError::InvalidManifest(format!("{label} requires a finite number, got {value:?}")))
}
