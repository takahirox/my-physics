use super::{CorrelationError, DatasetSplit, Result};
use std::collections::BTreeSet;
use std::fmt::Write as _;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EstimateOrigin {
    DatasetMeasured,
    Manufacturer,
    Literature,
    Estimated,
    Derived,
    Fitted,
    Assumed,
}

impl EstimateOrigin {
    pub fn name(self) -> &'static str {
        match self {
            Self::DatasetMeasured => "dataset-measured",
            Self::Manufacturer => "manufacturer",
            Self::Literature => "literature",
            Self::Estimated => "estimated",
            Self::Derived => "derived",
            Self::Fitted => "fitted",
            Self::Assumed => "assumed",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ParameterEstimate {
    pub parameter: String,
    pub value: f64,
    pub unit: String,
    pub origin: EstimateOrigin,
    pub source: String,
    pub source_revision: String,
    pub source_split: Option<DatasetSplit>,
    pub uncertainty: String,
    pub valid_min: f64,
    pub valid_max: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ParameterEstimateArtifact {
    pub schema_version: u32,
    pub artifact_id: String,
    pub vehicle_proxy: String,
    pub frozen_revision: String,
    pub limitations: Vec<String>,
    pub parameters: Vec<ParameterEstimate>,
}

impl ParameterEstimateArtifact {
    pub fn validate(&self) -> Result<()> {
        if self.schema_version != 1
            || self.artifact_id.trim().is_empty()
            || self.vehicle_proxy.trim().is_empty()
            || self.frozen_revision.trim().is_empty()
        {
            return Err(CorrelationError::InvalidManifest(
                "parameter artifact requires schema v1 and non-empty id/proxy/revision".to_owned(),
            ));
        }
        let mut names = BTreeSet::new();
        for parameter in &self.parameters {
            if !names.insert(&parameter.parameter) {
                return Err(CorrelationError::InvalidManifest(format!(
                    "duplicate parameter estimate {:?}",
                    parameter.parameter
                )));
            }
            if parameter.parameter.is_empty()
                || parameter.unit.is_empty()
                || parameter.source.is_empty()
                || parameter.source_revision.is_empty()
                || parameter.uncertainty.is_empty()
                || !parameter.value.is_finite()
                || !parameter.valid_min.is_finite()
                || !parameter.valid_max.is_finite()
                || parameter.valid_min > parameter.value
                || parameter.value > parameter.valid_max
            {
                return Err(CorrelationError::InvalidManifest(format!(
                    "incomplete/invalid parameter estimate {:?}",
                    parameter.parameter
                )));
            }
            if parameter.origin == EstimateOrigin::Fitted && parameter.source_split != Some(DatasetSplit::Training) {
                return Err(CorrelationError::SplitViolation(format!(
                    "fitted parameter {:?} must be derived from training split only",
                    parameter.parameter
                )));
            }
            if matches!(parameter.origin, EstimateOrigin::Derived | EstimateOrigin::Estimated)
                && parameter.source_split.is_some_and(|split| split != DatasetSplit::Training)
            {
                return Err(CorrelationError::SplitViolation(format!(
                    "derived/estimated parameter {:?} may not cite validation or test data",
                    parameter.parameter
                )));
            }
        }
        Ok(())
    }

    pub fn to_text(&self) -> Result<String> {
        self.validate()?;
        let mut output = String::from("# my-physics parameter-estimates-v1\n");
        writeln!(output, "schema_version={}", self.schema_version).unwrap();
        writeln!(output, "artifact_id={}", self.artifact_id).unwrap();
        writeln!(output, "vehicle_proxy={}", self.vehicle_proxy).unwrap();
        writeln!(output, "frozen_revision={}", self.frozen_revision).unwrap();
        for limitation in &self.limitations {
            writeln!(output, "limitation={limitation}").unwrap();
        }
        for parameter in &self.parameters {
            writeln!(
                output,
                "parameter={}|{:.17}|{}|{}|{}|{}|{}|{}|{:.17}|{:.17}",
                parameter.parameter,
                parameter.value,
                parameter.unit,
                parameter.origin.name(),
                parameter.source,
                parameter.source_revision,
                parameter.source_split.map_or("none", DatasetSplit::name),
                parameter.uncertainty,
                parameter.valid_min,
                parameter.valid_max
            )
            .unwrap();
        }
        Ok(output)
    }
}
