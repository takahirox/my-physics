//! Dataset-independent real-world telemetry correlation tools.
//!
//! This application layer never changes physical state or applies correction
//! forces. It aligns immutable measured and simulated time series, enforces
//! declared data splits, and reports errors with reproducible provenance.

mod alignment;
mod csv;
mod manifest;
mod parameter;
mod report;

pub use alignment::{AlignedSeries, AlignmentSpec, ChannelMapping, ClockCorrection, align_and_resample};
pub use csv::{CsvTelemetryAdapter, TelemetryAdapter};
pub use manifest::{
    CorrelationPurpose, DatasetFormat, DatasetManifest, DatasetSplit, FieldRole, FieldSchema, Frame, ManifestCatalog,
    ProvenanceRecord, Quantity, Resampling, TelemetryTable, Unit, ValueTransform,
};
pub use parameter::{EstimateOrigin, ParameterEstimate, ParameterEstimateArtifact};
pub use report::{ChannelMetrics, CorrelationReport, evaluate, write_report_artifacts};

use std::fmt;
use std::io;

#[derive(Debug)]
pub enum CorrelationError {
    Io(io::Error),
    InvalidManifest(String),
    InvalidTelemetry(String),
    InvalidAlignment(String),
    SplitViolation(String),
    AdapterUnavailable(String),
}

impl fmt::Display for CorrelationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(formatter, "I/O error: {error}"),
            Self::InvalidManifest(message) => write!(formatter, "invalid manifest: {message}"),
            Self::InvalidTelemetry(message) => write!(formatter, "invalid telemetry: {message}"),
            Self::InvalidAlignment(message) => write!(formatter, "invalid alignment: {message}"),
            Self::SplitViolation(message) => write!(formatter, "split violation: {message}"),
            Self::AdapterUnavailable(message) => write!(formatter, "adapter unavailable: {message}"),
        }
    }
}

impl std::error::Error for CorrelationError {}

impl From<io::Error> for CorrelationError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

pub type Result<T> = std::result::Result<T, CorrelationError>;
