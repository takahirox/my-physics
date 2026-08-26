use super::{CorrelationError, Resampling, Result, TelemetryTable};
use std::collections::BTreeMap;

#[derive(Clone, Debug, PartialEq)]
pub struct ChannelMapping {
    pub reference_channel: String,
    pub candidate_channel: String,
    pub output_name: String,
    pub unit: String,
    /// Physical scale used to normalize this channel before aggregation.
    pub normalization_scale: f64,
    /// Informational bounded lag search; it never shifts the aligned report.
    pub lag_search_bound_s: f64,
    pub reference_resampling: Resampling,
    pub candidate_resampling: Resampling,
}

impl ChannelMapping {
    pub fn same(name: impl Into<String>, unit: impl Into<String>) -> Self {
        let name = name.into();
        Self {
            reference_channel: name.clone(),
            candidate_channel: name.clone(),
            output_name: name,
            unit: unit.into(),
            normalization_scale: 1.0,
            lag_search_bound_s: 0.0,
            reference_resampling: Resampling::Linear,
            candidate_resampling: Resampling::Linear,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClockCorrection {
    /// Bounded clock-rate correction. This is not signal warping.
    pub scale: f64,
    /// Declared synchronization offset applied after scaling.
    pub offset_s: f64,
    /// Declared non-negative sensor/transport latency removed from timestamps.
    pub declared_latency_s: f64,
}

impl Default for ClockCorrection {
    fn default() -> Self {
        Self { scale: 1.0, offset_s: 0.0, declared_latency_s: 0.0 }
    }
}

impl ClockCorrection {
    fn validate(self, label: &str) -> Result<()> {
        if !self.scale.is_finite() || !(0.98..=1.02).contains(&self.scale) {
            return Err(CorrelationError::InvalidAlignment(format!(
                "{label} clock scale {} is outside the declared [0.98, 1.02] bound",
                self.scale
            )));
        }
        if !self.offset_s.is_finite() || self.offset_s.abs() > 60.0 {
            return Err(CorrelationError::InvalidAlignment(format!(
                "{label} clock offset must be finite and within ±60 s"
            )));
        }
        if !self.declared_latency_s.is_finite() || !(0.0..=5.0).contains(&self.declared_latency_s) {
            return Err(CorrelationError::InvalidAlignment(format!(
                "{label} declared latency must be within [0, 5] s"
            )));
        }
        Ok(())
    }
    fn corrected(self, raw_time_s: f64) -> f64 {
        raw_time_s * self.scale + self.offset_s - self.declared_latency_s
    }
    fn raw(self, corrected_time_s: f64) -> f64 {
        (corrected_time_s + self.declared_latency_s - self.offset_s) / self.scale
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct AlignmentSpec {
    /// Frozen calibration artifact revision for this alignment policy.
    pub revision: String,
    pub sample_period_s: f64,
    pub reference_clock: ClockCorrection,
    pub candidate_clock: ClockCorrection,
    pub mappings: Vec<ChannelMapping>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AlignedSeries {
    pub alignment_revision: String,
    pub reference_clock: ClockCorrection,
    pub candidate_clock: ClockCorrection,
    pub time_s: Vec<f64>,
    pub reference: BTreeMap<String, Vec<f64>>,
    pub candidate: BTreeMap<String, Vec<f64>>,
    pub mappings: Vec<ChannelMapping>,
}

pub fn align_and_resample(
    reference: &TelemetryTable,
    candidate: &TelemetryTable,
    specification: &AlignmentSpec,
) -> Result<AlignedSeries> {
    reference.validate()?;
    candidate.validate()?;
    if !specification.sample_period_s.is_finite() || specification.sample_period_s <= 0.0 {
        return Err(CorrelationError::InvalidAlignment("sample_period_s must be finite and positive".to_owned()));
    }
    if specification.revision.trim().is_empty() {
        return Err(CorrelationError::InvalidAlignment("alignment revision must not be empty".to_owned()));
    }
    specification.reference_clock.validate("reference")?;
    specification.candidate_clock.validate("candidate")?;
    if specification.mappings.is_empty() {
        return Err(CorrelationError::InvalidAlignment("at least one explicit channel mapping is required".to_owned()));
    }
    let mut names = std::collections::BTreeSet::new();
    for mapping in &specification.mappings {
        if mapping.output_name.is_empty()
            || mapping.unit.is_empty()
            || !mapping.normalization_scale.is_finite()
            || mapping.normalization_scale <= 0.0
            || !mapping.lag_search_bound_s.is_finite()
            || !(0.0..=5.0).contains(&mapping.lag_search_bound_s)
        {
            return Err(CorrelationError::InvalidAlignment(
                "mapping output/unit, normalization scale or lag-search bound is invalid".to_owned(),
            ));
        }
        if !names.insert(&mapping.output_name) {
            return Err(CorrelationError::InvalidAlignment(format!(
                "duplicate output channel {:?}",
                mapping.output_name
            )));
        }
        if !reference.channels.contains_key(&mapping.reference_channel) {
            return Err(CorrelationError::InvalidAlignment(format!(
                "reference channel {:?} is absent",
                mapping.reference_channel
            )));
        }
        if !candidate.channels.contains_key(&mapping.candidate_channel) {
            return Err(CorrelationError::InvalidAlignment(format!(
                "candidate channel {:?} is absent",
                mapping.candidate_channel
            )));
        }
    }
    let start = specification
        .reference_clock
        .corrected(reference.time_s[0])
        .max(specification.candidate_clock.corrected(candidate.time_s[0]));
    let end = specification
        .reference_clock
        .corrected(reference.time_s[reference.time_s.len() - 1])
        .min(specification.candidate_clock.corrected(candidate.time_s[candidate.time_s.len() - 1]));
    if end < start || end - start < specification.sample_period_s {
        return Err(CorrelationError::InvalidAlignment(format!(
            "insufficient overlap after declared clock correction: start={start}, end={end}"
        )));
    }
    let sample_count = ((end - start) / specification.sample_period_s + 1.0e-9).floor() as usize + 1;
    let time_s: Vec<_> = (0..sample_count).map(|index| start + index as f64 * specification.sample_period_s).collect();
    let mut aligned = AlignedSeries {
        alignment_revision: specification.revision.clone(),
        reference_clock: specification.reference_clock,
        candidate_clock: specification.candidate_clock,
        time_s,
        reference: BTreeMap::new(),
        candidate: BTreeMap::new(),
        mappings: specification.mappings.clone(),
    };
    for mapping in &specification.mappings {
        let reference_values = &reference.channels[&mapping.reference_channel];
        let candidate_values = &candidate.channels[&mapping.candidate_channel];
        aligned.reference.insert(
            mapping.output_name.clone(),
            aligned
                .time_s
                .iter()
                .map(|time| {
                    interpolate(
                        &reference.time_s,
                        reference_values,
                        specification.reference_clock.raw(*time),
                        mapping.reference_resampling,
                    )
                })
                .collect::<Result<_>>()?,
        );
        aligned.candidate.insert(
            mapping.output_name.clone(),
            aligned
                .time_s
                .iter()
                .map(|time| {
                    interpolate(
                        &candidate.time_s,
                        candidate_values,
                        specification.candidate_clock.raw(*time),
                        mapping.candidate_resampling,
                    )
                })
                .collect::<Result<_>>()?,
        );
    }
    Ok(aligned)
}

fn interpolate(time: &[f64], values: &[f64], query: f64, resampling: Resampling) -> Result<f64> {
    let tolerance = 1.0e-9 + 64.0 * f64::EPSILON * query.abs().max(time[time.len() - 1].abs());
    if query < time[0] - tolerance || query > time[time.len() - 1] + tolerance {
        return Err(CorrelationError::InvalidAlignment(format!("query {query} requires extrapolation")));
    }
    let query = query.clamp(time[0], time[time.len() - 1]);
    match time.binary_search_by(|value| value.total_cmp(&query)) {
        Ok(index) => Ok(values[index]),
        Err(0) => Ok(values[0]),
        Err(index) if index == time.len() => Ok(values[values.len() - 1]),
        Err(index) if resampling == Resampling::Previous => Ok(values[index - 1]),
        Err(index) => {
            let lower = index - 1;
            let fraction = (query - time[lower]) / (time[index] - time[lower]);
            Ok(values[lower] + (values[index] - values[lower]) * fraction)
        }
    }
}
