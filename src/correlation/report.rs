use super::{AlignedSeries, ClockCorrection, CorrelationError, CorrelationPurpose, DatasetManifest, Result};
use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::fs;
use std::path::Path;

pub const CORRELATION_DISCLAIMER: &str = "Correlation metrics quantify this declared dataset and scenario only; they are not certification or general validity.";

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ChannelMetrics {
    pub unit: String,
    pub sample_count: usize,
    pub mae: f64,
    pub rmse: f64,
    pub max_abs_error: f64,
    pub bias: f64,
    pub r_squared: Option<f64>,
    pub correlation: Option<f64>,
    pub reference_peak: f64,
    pub candidate_peak: f64,
    pub peak_value_error: f64,
    pub peak_time_error_s: f64,
    pub best_lag_s: Option<f64>,
    pub best_lag_correlation: Option<f64>,
    pub reference_min: f64,
    pub reference_max: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CorrelationReport {
    pub schema_version: u32,
    pub report_id: String,
    pub purpose: CorrelationPurpose,
    pub reference_dataset_id: String,
    pub reference_checksum: String,
    pub reference_vehicle_id: String,
    pub reference_session_id: String,
    pub reference_split: String,
    pub reference_license_id: String,
    pub reference_license_verified: bool,
    pub mapping_revision: String,
    pub alignment_revision: String,
    pub filter_revision: String,
    pub candidate_id: String,
    pub candidate_checksum: String,
    pub candidate_vehicle_id: String,
    pub candidate_session_id: String,
    pub candidate_split: String,
    pub candidate_provenance_origin: String,
    pub candidate_provenance_source: String,
    pub candidate_revision: String,
    pub reference_clock: ClockCorrection,
    pub candidate_clock: ClockCorrection,
    pub sample_period_s: f64,
    pub overlap_start_s: f64,
    pub overlap_end_s: f64,
    pub channel_metrics: BTreeMap<String, ChannelMetrics>,
    pub aggregate_normalized_rmse: f64,
    pub disclaimer: String,
}

pub fn evaluate(
    report_id: impl Into<String>,
    purpose: CorrelationPurpose,
    reference_manifest: &DatasetManifest,
    candidate_manifest: &DatasetManifest,
    candidate_revision: impl Into<String>,
    aligned: &AlignedSeries,
) -> Result<CorrelationReport> {
    reference_manifest.validate()?;
    candidate_manifest.validate()?;
    reference_manifest.require_purpose(purpose)?;
    if candidate_manifest.split != reference_manifest.split {
        return Err(CorrelationError::SplitViolation(format!(
            "candidate split {} does not match reference split {}",
            candidate_manifest.split.name(),
            reference_manifest.split.name()
        )));
    }
    if aligned.time_s.len() < 2 || aligned.mappings.is_empty() {
        return Err(CorrelationError::InvalidAlignment("report requires at least two aligned samples".to_owned()));
    }
    if aligned.time_s.iter().any(|value| !value.is_finite()) || aligned.time_s.windows(2).any(|pair| pair[1] <= pair[0])
    {
        return Err(CorrelationError::InvalidAlignment(
            "report timestamps must be finite and strictly increasing".to_owned(),
        ));
    }
    let mut metrics = BTreeMap::new();
    let mut normalized_squared = 0.0;
    for mapping in &aligned.mappings {
        let reference = aligned.reference.get(&mapping.output_name).ok_or_else(|| {
            CorrelationError::InvalidAlignment(format!("aligned reference {:?} is absent", mapping.output_name))
        })?;
        let candidate = aligned.candidate.get(&mapping.output_name).ok_or_else(|| {
            CorrelationError::InvalidAlignment(format!("aligned candidate {:?} is absent", mapping.output_name))
        })?;
        if reference.len() != aligned.time_s.len() || candidate.len() != aligned.time_s.len() {
            return Err(CorrelationError::InvalidAlignment(format!(
                "aligned channel {:?} has inconsistent sample count",
                mapping.output_name
            )));
        }
        if reference.iter().chain(candidate).any(|value| !value.is_finite()) {
            return Err(CorrelationError::InvalidAlignment(format!(
                "aligned channel {:?} contains non-finite data",
                mapping.output_name
            )));
        }
        let channel = channel_metrics(reference, candidate, &aligned.time_s, &mapping.unit, mapping.lag_search_bound_s);
        if !metrics_are_finite(&channel) {
            return Err(CorrelationError::InvalidAlignment(format!(
                "metrics for {:?} overflowed or became non-finite",
                mapping.output_name
            )));
        }
        normalized_squared += (channel.rmse / mapping.normalization_scale).powi(2);
        metrics.insert(mapping.output_name.clone(), channel);
    }
    let aggregate_normalized_rmse = (normalized_squared / aligned.mappings.len() as f64).sqrt();
    if !aggregate_normalized_rmse.is_finite() {
        return Err(CorrelationError::InvalidAlignment(
            "aggregate normalized metric overflowed or became non-finite".to_owned(),
        ));
    }
    Ok(CorrelationReport {
        schema_version: 1,
        report_id: report_id.into(),
        purpose,
        reference_dataset_id: reference_manifest.dataset_id.clone(),
        reference_checksum: reference_manifest.content_checksum.clone(),
        reference_vehicle_id: reference_manifest.vehicle_id.clone(),
        reference_session_id: reference_manifest.session_id.clone(),
        reference_split: reference_manifest.split.name().to_owned(),
        reference_license_id: reference_manifest.license_id.clone(),
        reference_license_verified: reference_manifest.license_verified,
        mapping_revision: reference_manifest.mapping_revision.clone(),
        alignment_revision: aligned.alignment_revision.clone(),
        filter_revision: reference_manifest.filter_revision.clone(),
        candidate_id: candidate_manifest.dataset_id.clone(),
        candidate_checksum: candidate_manifest.content_checksum.clone(),
        candidate_vehicle_id: candidate_manifest.vehicle_id.clone(),
        candidate_session_id: candidate_manifest.session_id.clone(),
        candidate_split: candidate_manifest.split.name().to_owned(),
        candidate_provenance_origin: candidate_manifest.provenance.origin.clone(),
        candidate_provenance_source: candidate_manifest.provenance.source.clone(),
        candidate_revision: candidate_revision.into(),
        reference_clock: aligned.reference_clock,
        candidate_clock: aligned.candidate_clock,
        sample_period_s: aligned.time_s[1] - aligned.time_s[0],
        overlap_start_s: aligned.time_s[0],
        overlap_end_s: aligned.time_s[aligned.time_s.len() - 1],
        channel_metrics: metrics,
        aggregate_normalized_rmse,
        disclaimer: CORRELATION_DISCLAIMER.to_owned(),
    })
}

fn metrics_are_finite(metrics: &ChannelMetrics) -> bool {
    [
        metrics.mae,
        metrics.rmse,
        metrics.max_abs_error,
        metrics.bias,
        metrics.reference_peak,
        metrics.candidate_peak,
        metrics.peak_value_error,
        metrics.peak_time_error_s,
        metrics.reference_min,
        metrics.reference_max,
    ]
    .into_iter()
    .all(f64::is_finite)
        && [metrics.r_squared, metrics.correlation, metrics.best_lag_s, metrics.best_lag_correlation]
            .into_iter()
            .flatten()
            .all(f64::is_finite)
}

fn channel_metrics(
    reference: &[f64],
    candidate: &[f64],
    time_s: &[f64],
    unit: &str,
    lag_search_bound_s: f64,
) -> ChannelMetrics {
    let count = reference.len();
    let mean = reference.iter().sum::<f64>() / count as f64;
    let mut absolute_sum = 0.0;
    let mut squared_sum = 0.0;
    let mut bias_sum = 0.0;
    let mut max_abs_error: f64 = 0.0;
    let mut total_variance = 0.0;
    let mut reference_min = f64::INFINITY;
    let mut reference_max = f64::NEG_INFINITY;
    let mut candidate_mean = 0.0;
    for (&reference, &candidate) in reference.iter().zip(candidate) {
        let error = candidate - reference;
        absolute_sum += error.abs();
        squared_sum += error * error;
        bias_sum += error;
        max_abs_error = max_abs_error.max(error.abs());
        total_variance += (reference - mean).powi(2);
        reference_min = reference_min.min(reference);
        reference_max = reference_max.max(reference);
        candidate_mean += candidate;
    }
    candidate_mean /= count as f64;
    let covariance = reference
        .iter()
        .zip(candidate)
        .map(|(&reference, &candidate)| (reference - mean) * (candidate - candidate_mean))
        .sum::<f64>();
    let reference_variance = reference.iter().map(|value| (value - mean).powi(2)).sum::<f64>();
    let candidate_variance = candidate.iter().map(|value| (value - candidate_mean).powi(2)).sum::<f64>();
    let correlation = (reference_variance > 1.0e-18 && candidate_variance > 1.0e-18)
        .then_some(covariance / (reference_variance * candidate_variance).sqrt());
    let reference_peak_index = reference
        .iter()
        .enumerate()
        .max_by(|(_, left), (_, right)| left.abs().total_cmp(&right.abs()))
        .map_or(0, |(index, _)| index);
    let candidate_peak_index = candidate
        .iter()
        .enumerate()
        .max_by(|(_, left), (_, right)| left.abs().total_cmp(&right.abs()))
        .map_or(0, |(index, _)| index);
    let reference_peak = reference[reference_peak_index];
    let candidate_peak = candidate[candidate_peak_index];
    let (best_lag_s, best_lag_correlation) = best_lag(reference, candidate, time_s, lag_search_bound_s);
    ChannelMetrics {
        unit: unit.to_owned(),
        sample_count: count,
        mae: absolute_sum / count as f64,
        rmse: (squared_sum / count as f64).sqrt(),
        max_abs_error,
        bias: bias_sum / count as f64,
        r_squared: (total_variance > 1.0e-18).then_some(1.0 - squared_sum / total_variance),
        correlation,
        reference_peak,
        candidate_peak,
        peak_value_error: candidate_peak - reference_peak,
        peak_time_error_s: time_s[candidate_peak_index] - time_s[reference_peak_index],
        best_lag_s,
        best_lag_correlation,
        reference_min,
        reference_max,
    }
}

fn best_lag(reference: &[f64], candidate: &[f64], time_s: &[f64], bound_s: f64) -> (Option<f64>, Option<f64>) {
    if bound_s <= 0.0 || time_s.len() < 3 {
        return (None, None);
    }
    let dt = time_s[1] - time_s[0];
    let steps = (bound_s / dt).floor() as isize;
    let mut best: Option<(isize, f64)> = None;
    for lag in -steps..=steps {
        let pairs: Vec<_> = (0..reference.len())
            .filter_map(|reference_index| {
                let candidate_index = reference_index as isize + lag;
                (0..candidate.len() as isize)
                    .contains(&candidate_index)
                    .then(|| (reference[reference_index], candidate[candidate_index as usize]))
            })
            .collect();
        if pairs.len() < 3.max(reference.len() / 2) {
            continue;
        }
        let reference_mean = pairs.iter().map(|pair| pair.0).sum::<f64>() / pairs.len() as f64;
        let candidate_mean = pairs.iter().map(|pair| pair.1).sum::<f64>() / pairs.len() as f64;
        let covariance = pairs.iter().map(|pair| (pair.0 - reference_mean) * (pair.1 - candidate_mean)).sum::<f64>();
        let reference_variance = pairs.iter().map(|pair| (pair.0 - reference_mean).powi(2)).sum::<f64>();
        let candidate_variance = pairs.iter().map(|pair| (pair.1 - candidate_mean).powi(2)).sum::<f64>();
        if reference_variance <= 1.0e-18 || candidate_variance <= 1.0e-18 {
            continue;
        }
        let correlation = covariance / (reference_variance * candidate_variance).sqrt();
        if best.is_none_or(|(best_lag, best_correlation)| {
            correlation > best_correlation + 1.0e-12
                || ((correlation - best_correlation).abs() <= 1.0e-12 && lag.abs() < best_lag.abs())
        }) {
            best = Some((lag, correlation));
        }
    }
    best.map_or((None, None), |(lag, correlation)| (Some(lag as f64 * dt), Some(correlation)))
}

impl CorrelationReport {
    pub fn to_json(&self) -> String {
        let mut output = String::new();
        write!(
            output,
            "{{\"schema_version\":{},\"report_id\":\"{}\",\"purpose\":\"{}\",\"reference_dataset_id\":\"{}\",\"reference_checksum\":\"{}\",\"reference_vehicle_id\":\"{}\",\"reference_session_id\":\"{}\",\"reference_split\":\"{}\",\"reference_license_id\":\"{}\",\"reference_license_verified\":{},\"mapping_revision\":\"{}\",\"alignment_revision\":\"{}\",\"filter_revision\":\"{}\",\"candidate_id\":\"{}\",\"candidate_checksum\":\"{}\",\"candidate_vehicle_id\":\"{}\",\"candidate_session_id\":\"{}\",\"candidate_split\":\"{}\",\"candidate_provenance\":{{\"origin\":\"{}\",\"source\":\"{}\",\"revision\":\"{}\"}},\"reference_clock\":{{\"scale\":{:.17},\"offset_s\":{:.9},\"declared_latency_s\":{:.9}}},\"candidate_clock\":{{\"scale\":{:.17},\"offset_s\":{:.9},\"declared_latency_s\":{:.9}}},\"sample_period_s\":{:.9},\"overlap_start_s\":{:.9},\"overlap_end_s\":{:.9},\"aggregate_normalized_rmse\":{:.9},\"disclaimer\":\"{}\",\"channels\":{{",
            self.schema_version,
            json_escape(&self.report_id),
            self.purpose.name(),
            json_escape(&self.reference_dataset_id),
            json_escape(&self.reference_checksum),
            json_escape(&self.reference_vehicle_id),
            json_escape(&self.reference_session_id),
            json_escape(&self.reference_split),
            json_escape(&self.reference_license_id),
            self.reference_license_verified,
            json_escape(&self.mapping_revision),
            json_escape(&self.alignment_revision),
            json_escape(&self.filter_revision),
            json_escape(&self.candidate_id),
            json_escape(&self.candidate_checksum),
            json_escape(&self.candidate_vehicle_id),
            json_escape(&self.candidate_session_id),
            json_escape(&self.candidate_split),
            json_escape(&self.candidate_provenance_origin),
            json_escape(&self.candidate_provenance_source),
            json_escape(&self.candidate_revision),
            self.reference_clock.scale,
            self.reference_clock.offset_s,
            self.reference_clock.declared_latency_s,
            self.candidate_clock.scale,
            self.candidate_clock.offset_s,
            self.candidate_clock.declared_latency_s,
            self.sample_period_s,
            self.overlap_start_s,
            self.overlap_end_s,
            self.aggregate_normalized_rmse,
            json_escape(&self.disclaimer)
        )
        .unwrap();
        for (index, (name, metrics)) in self.channel_metrics.iter().enumerate() {
            if index != 0 {
                output.push(',');
            }
            write!(
                output,
                "\"{}\":{{\"unit\":\"{}\",\"sample_count\":{},\"mae\":{:.9},\"rmse\":{:.9},\"max_abs_error\":{:.9},\"bias\":{:.9},\"r_squared\":{},\"correlation\":{},\"reference_peak\":{:.9},\"candidate_peak\":{:.9},\"peak_value_error\":{:.9},\"peak_time_error_s\":{:.9},\"best_lag_s\":{},\"best_lag_correlation\":{},\"reference_min\":{:.9},\"reference_max\":{:.9}}}",
                json_escape(name),
                json_escape(&metrics.unit),
                metrics.sample_count,
                metrics.mae,
                metrics.rmse,
                metrics.max_abs_error,
                metrics.bias,
                metrics.r_squared.map_or_else(|| "null".to_owned(), |value| format!("{value:.9}")),
                option_number(metrics.correlation),
                metrics.reference_peak,
                metrics.candidate_peak,
                metrics.peak_value_error,
                metrics.peak_time_error_s,
                option_number(metrics.best_lag_s),
                option_number(metrics.best_lag_correlation),
                metrics.reference_min,
                metrics.reference_max,
            )
            .unwrap();
        }
        output.push_str("}}\n");
        output
    }

    pub fn metrics_csv(&self) -> String {
        let mut output = String::from(
            "channel,unit,sample_count,mae,rmse,max_abs_error,bias,r_squared,correlation,reference_peak,candidate_peak,peak_value_error,peak_time_error_s,best_lag_s,best_lag_correlation,reference_min,reference_max\n",
        );
        for (name, metrics) in &self.channel_metrics {
            writeln!(
                output,
                "{},{},{},{:.9},{:.9},{:.9},{:.9},{},{},{:.9},{:.9},{:.9},{:.9},{},{},{:.9},{:.9}",
                csv_escape(name),
                csv_escape(&metrics.unit),
                metrics.sample_count,
                metrics.mae,
                metrics.rmse,
                metrics.max_abs_error,
                metrics.bias,
                metrics.r_squared.map_or_else(String::new, |value| format!("{value:.9}")),
                metrics.correlation.map_or_else(String::new, |value| format!("{value:.9}")),
                metrics.reference_peak,
                metrics.candidate_peak,
                metrics.peak_value_error,
                metrics.peak_time_error_s,
                metrics.best_lag_s.map_or_else(String::new, |value| format!("{value:.9}")),
                metrics.best_lag_correlation.map_or_else(String::new, |value| format!("{value:.9}")),
                metrics.reference_min,
                metrics.reference_max
            )
            .unwrap();
        }
        output
    }
}

fn option_number(value: Option<f64>) -> String {
    value.map_or_else(|| "null".to_owned(), |value| format!("{value:.9}"))
}

pub fn write_report_artifacts(directory: &Path, report: &CorrelationReport, aligned: &AlignedSeries) -> Result<()> {
    fs::create_dir_all(directory)?;
    fs::write(directory.join("correlation-report.json"), report.to_json())?;
    fs::write(directory.join("metrics.csv"), report.metrics_csv())?;
    let mut timeseries = String::from("time_s");
    for mapping in &aligned.mappings {
        for prefix in ["reference", "candidate", "error"] {
            write!(timeseries, ",{}", csv_escape(&format!("{prefix}_{}", mapping.output_name))).unwrap();
        }
    }
    timeseries.push('\n');
    for index in 0..aligned.time_s.len() {
        write!(timeseries, "{:.9}", aligned.time_s[index]).unwrap();
        for mapping in &aligned.mappings {
            let reference = aligned.reference[&mapping.output_name][index];
            let candidate = aligned.candidate[&mapping.output_name][index];
            write!(timeseries, ",{reference:.9},{candidate:.9},{:.9}", candidate - reference).unwrap();
        }
        timeseries.push('\n');
    }
    fs::write(directory.join("aligned-timeseries.csv"), timeseries)?;
    Ok(())
}

fn json_escape(value: &str) -> String {
    value.chars().fold(String::new(), |mut output, character| {
        match character {
            '"' => output.push_str("\\\""),
            '\\' => output.push_str("\\\\"),
            '\n' => output.push_str("\\n"),
            '\r' => output.push_str("\\r"),
            '\t' => output.push_str("\\t"),
            character if character.is_control() => write!(output, "\\u{:04x}", character as u32).unwrap(),
            character => output.push(character),
        }
        output
    })
}

fn csv_escape(value: &str) -> String {
    if value.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", value.replace('"', "\"\""))
    } else {
        value.to_owned()
    }
}
