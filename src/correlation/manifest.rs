use super::{CorrelationError, Result};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

pub const MANIFEST_SCHEMA_VERSION: u32 = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DatasetFormat {
    Csv,
    Parquet,
}

impl DatasetFormat {
    pub fn name(self) -> &'static str {
        match self {
            Self::Csv => "csv",
            Self::Parquet => "parquet",
        }
    }
    fn parse(value: &str) -> Result<Self> {
        match value {
            "csv" => Ok(Self::Csv),
            "parquet" => Ok(Self::Parquet),
            _ => Err(CorrelationError::InvalidManifest(format!("unsupported format {value:?}"))),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum DatasetSplit {
    Training,
    Validation,
    Test,
}

impl DatasetSplit {
    pub fn name(self) -> &'static str {
        match self {
            Self::Training => "training",
            Self::Validation => "validation",
            Self::Test => "test",
        }
    }
    fn parse(value: &str) -> Result<Self> {
        match value {
            "training" => Ok(Self::Training),
            "validation" => Ok(Self::Validation),
            "test" => Ok(Self::Test),
            _ => Err(CorrelationError::InvalidManifest(format!("invalid split {value:?}"))),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CorrelationPurpose {
    ParameterFitting,
    ModelSelection,
    FinalEvaluation,
}

impl CorrelationPurpose {
    pub fn name(self) -> &'static str {
        match self {
            Self::ParameterFitting => "parameter-fitting",
            Self::ModelSelection => "model-selection",
            Self::FinalEvaluation => "final-evaluation",
        }
    }
    pub fn required_split(self) -> DatasetSplit {
        match self {
            Self::ParameterFitting => DatasetSplit::Training,
            Self::ModelSelection => DatasetSplit::Validation,
            Self::FinalEvaluation => DatasetSplit::Test,
        }
    }
    pub fn parse(value: &str) -> Result<Self> {
        match value {
            "parameter-fitting" => Ok(Self::ParameterFitting),
            "model-selection" => Ok(Self::ModelSelection),
            "final-evaluation" => Ok(Self::FinalEvaluation),
            _ => Err(CorrelationError::InvalidManifest(format!("invalid correlation purpose {value:?}"))),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FieldRole {
    Time,
    Input,
    Observation,
    Context,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Resampling {
    Linear,
    Previous,
}

impl Resampling {
    pub fn name(self) -> &'static str {
        match self {
            Self::Linear => "linear",
            Self::Previous => "previous",
        }
    }
    fn parse(value: &str) -> Result<Self> {
        match value {
            "linear" => Ok(Self::Linear),
            "previous" => Ok(Self::Previous),
            _ => Err(CorrelationError::InvalidManifest(format!("invalid resampling mode {value:?}"))),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ValueTransform {
    Identity,
    Affine { scale: f64, offset: f64 },
    RelativeAffine { scale: f64, offset: f64 },
    UnwrapAffine { period: f64, scale: f64, offset: f64, relative_to_first: bool },
}

impl ValueTransform {
    pub fn text(self) -> String {
        match self {
            Self::Identity => "identity".to_owned(),
            Self::Affine { scale, offset } => format!("affine,{scale:.17e},{offset:.17e}"),
            Self::RelativeAffine { scale, offset } => format!("relative-affine,{scale:.17e},{offset:.17e}"),
            Self::UnwrapAffine { period, scale, offset, relative_to_first } => {
                format!("unwrap-affine,{period:.17e},{scale:.17e},{offset:.17e},{relative_to_first}")
            }
        }
    }
    fn parse(value: &str) -> Result<Self> {
        let parts: Vec<_> = value.split(',').collect();
        let number = |index: usize| {
            parts
                .get(index)
                .ok_or_else(|| CorrelationError::InvalidManifest(format!("incomplete transform {value:?}")))?
                .parse::<f64>()
                .map_err(|_| CorrelationError::InvalidManifest(format!("invalid transform number in {value:?}")))
        };
        let transform = match parts.as_slice() {
            ["identity"] => Self::Identity,
            ["affine", _, _] => Self::Affine { scale: number(1)?, offset: number(2)? },
            ["relative-affine", _, _] => Self::RelativeAffine { scale: number(1)?, offset: number(2)? },
            ["unwrap-affine", _, _, _, relative] => Self::UnwrapAffine {
                period: number(1)?,
                scale: number(2)?,
                offset: number(3)?,
                relative_to_first: relative.parse().map_err(|_| {
                    CorrelationError::InvalidManifest(format!("invalid transform boolean in {value:?}"))
                })?,
            },
            _ => return Err(CorrelationError::InvalidManifest(format!("invalid transform {value:?}"))),
        };
        match transform {
            Self::Identity => {}
            Self::Affine { scale, offset } if scale.is_finite() && offset.is_finite() && scale != 0.0 => {}
            Self::RelativeAffine { scale, offset } if scale.is_finite() && offset.is_finite() && scale != 0.0 => {}
            Self::UnwrapAffine { period, scale, offset, .. }
                if period.is_finite() && period > 0.0 && scale.is_finite() && scale != 0.0 && offset.is_finite() => {}
            _ => return Err(CorrelationError::InvalidManifest(format!("non-finite/degenerate transform {value:?}"))),
        }
        Ok(transform)
    }
}

impl FieldRole {
    pub fn name(self) -> &'static str {
        match self {
            Self::Time => "time",
            Self::Input => "input",
            Self::Observation => "observation",
            Self::Context => "context",
        }
    }
    fn parse(value: &str) -> Result<Self> {
        match value {
            "time" => Ok(Self::Time),
            "input" => Ok(Self::Input),
            "observation" => Ok(Self::Observation),
            "context" => Ok(Self::Context),
            _ => Err(CorrelationError::InvalidManifest(format!("invalid field role {value:?}"))),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quantity {
    Time,
    Speed,
    Position,
    Acceleration,
    Angle,
    AngularRate,
    Force,
    Torque,
    Temperature,
    Pressure,
    Ratio,
    Other,
}

impl Quantity {
    pub fn name(self) -> &'static str {
        match self {
            Self::Time => "time",
            Self::Speed => "speed",
            Self::Position => "position",
            Self::Acceleration => "acceleration",
            Self::Angle => "angle",
            Self::AngularRate => "angular-rate",
            Self::Force => "force",
            Self::Torque => "torque",
            Self::Temperature => "temperature",
            Self::Pressure => "pressure",
            Self::Ratio => "ratio",
            Self::Other => "other",
        }
    }
    fn parse(value: &str) -> Result<Self> {
        match value {
            "time" => Ok(Self::Time),
            "speed" => Ok(Self::Speed),
            "position" => Ok(Self::Position),
            "acceleration" => Ok(Self::Acceleration),
            "angle" => Ok(Self::Angle),
            "angular-rate" => Ok(Self::AngularRate),
            "force" => Ok(Self::Force),
            "torque" => Ok(Self::Torque),
            "temperature" => Ok(Self::Temperature),
            "pressure" => Ok(Self::Pressure),
            "ratio" => Ok(Self::Ratio),
            "other" => Ok(Self::Other),
            _ => Err(CorrelationError::InvalidManifest(format!("invalid quantity {value:?}"))),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Unit(pub String);

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Frame(pub String);

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProvenanceRecord {
    pub origin: String,
    pub source: String,
    pub revision: String,
    pub notes: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FieldSchema {
    pub canonical_name: String,
    pub source_column: String,
    pub quantity: Quantity,
    pub unit: Unit,
    pub frame: Frame,
    pub role: FieldRole,
    pub resampling: Resampling,
    pub transform: ValueTransform,
    /// Declared scale used only to form a dimensionless aggregate metric.
    pub metric_normalization_scale: f64,
    /// Informational lag search bound; it never changes aligned samples.
    pub metric_lag_search_bound_s: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DatasetManifest {
    pub schema_version: u32,
    pub dataset_id: String,
    pub format: DatasetFormat,
    pub source_title: String,
    pub source_uri: String,
    pub license_id: String,
    pub license_verified: bool,
    pub content_checksum: String,
    pub vehicle_id: String,
    pub session_id: String,
    pub timestamp_semantics: String,
    pub expected_sample_period_s: f64,
    pub maximum_gap_s: f64,
    pub split: DatasetSplit,
    pub split_group: String,
    pub provenance: ProvenanceRecord,
    pub mapping_revision: String,
    pub alignment_revision: String,
    pub filter_revision: String,
    pub fields: Vec<FieldSchema>,
}

impl DatasetManifest {
    pub fn validate(&self) -> Result<()> {
        if self.schema_version != MANIFEST_SCHEMA_VERSION {
            return Err(CorrelationError::InvalidManifest(format!(
                "schema_version {} is unsupported; expected {MANIFEST_SCHEMA_VERSION}",
                self.schema_version
            )));
        }
        for (label, value) in [
            ("dataset_id", &self.dataset_id),
            ("content_checksum", &self.content_checksum),
            ("source_title", &self.source_title),
            ("source_uri", &self.source_uri),
            ("vehicle_id", &self.vehicle_id),
            ("session_id", &self.session_id),
            ("timestamp_semantics", &self.timestamp_semantics),
            ("split_group", &self.split_group),
            ("provenance.origin", &self.provenance.origin),
            ("provenance.source", &self.provenance.source),
            ("provenance.revision", &self.provenance.revision),
            ("mapping_revision", &self.mapping_revision),
            ("alignment_revision", &self.alignment_revision),
            ("filter_revision", &self.filter_revision),
        ] {
            if value.trim().is_empty() {
                return Err(CorrelationError::InvalidManifest(format!("{label} must not be empty")));
            }
        }
        if !self.expected_sample_period_s.is_finite()
            || self.expected_sample_period_s <= 0.0
            || !self.maximum_gap_s.is_finite()
            || self.maximum_gap_s < self.expected_sample_period_s
        {
            return Err(CorrelationError::InvalidManifest(
                "expected_sample_period_s must be positive and maximum_gap_s must be at least that period".to_owned(),
            ));
        }
        if self.license_verified && self.license_id.trim().is_empty() {
            return Err(CorrelationError::InvalidManifest(
                "license_verified=true requires a non-empty license_id".to_owned(),
            ));
        }
        if self.fields.is_empty() {
            return Err(CorrelationError::InvalidManifest("at least one field is required".to_owned()));
        }
        let time_fields = self.fields.iter().filter(|field| field.role == FieldRole::Time).count();
        if time_fields != 1 {
            return Err(CorrelationError::InvalidManifest(format!(
                "exactly one time field is required, found {time_fields}"
            )));
        }
        let mut canonical = BTreeSet::new();
        let mut source = BTreeSet::new();
        for field in &self.fields {
            if !canonical.insert(&field.canonical_name) {
                return Err(CorrelationError::InvalidManifest(format!(
                    "duplicate canonical field {:?}",
                    field.canonical_name
                )));
            }
            if !source.insert(&field.source_column) {
                return Err(CorrelationError::InvalidManifest(format!(
                    "duplicate source column {:?}",
                    field.source_column
                )));
            }
            if field.canonical_name.trim().is_empty()
                || field.source_column.trim().is_empty()
                || field.unit.0.trim().is_empty()
                || field.frame.0.trim().is_empty()
            {
                return Err(CorrelationError::InvalidManifest("field names and units must not be empty".to_owned()));
            }
            if field.role == FieldRole::Time && (field.quantity != Quantity::Time || field.unit.0 != "s") {
                return Err(CorrelationError::InvalidManifest(
                    "time field must declare quantity=time and SI unit s".to_owned(),
                ));
            }
            if !field.metric_normalization_scale.is_finite()
                || field.metric_normalization_scale <= 0.0
                || !field.metric_lag_search_bound_s.is_finite()
                || !(0.0..=5.0).contains(&field.metric_lag_search_bound_s)
            {
                return Err(CorrelationError::InvalidManifest(format!(
                    "field {:?} has invalid metric normalization/lag bound",
                    field.canonical_name
                )));
            }
        }
        Ok(())
    }

    pub fn require_license_for_publication(&self) -> Result<()> {
        if !self.license_verified {
            return Err(CorrelationError::InvalidManifest(format!(
                "dataset {} has no verified redistribution/analysis license",
                self.dataset_id
            )));
        }
        Ok(())
    }

    pub fn require_purpose(&self, purpose: CorrelationPurpose) -> Result<()> {
        let required = purpose.required_split();
        if self.split != required {
            return Err(CorrelationError::SplitViolation(format!(
                "purpose {} requires split {}, but dataset {} is {}",
                purpose.name(),
                required.name(),
                self.dataset_id,
                self.split.name()
            )));
        }
        Ok(())
    }

    pub fn observation_fields(&self) -> impl Iterator<Item = &FieldSchema> {
        self.fields.iter().filter(|field| field.role == FieldRole::Observation)
    }

    pub fn to_text(&self) -> String {
        let mut output = String::from("# my-physics correlation-manifest-v1\n");
        for (key, value) in [
            ("schema_version", self.schema_version.to_string()),
            ("dataset_id", self.dataset_id.clone()),
            ("format", self.format.name().to_owned()),
            ("source_title", self.source_title.clone()),
            ("source_uri", self.source_uri.clone()),
            ("license_id", self.license_id.clone()),
            ("license_verified", self.license_verified.to_string()),
            ("content_checksum", self.content_checksum.clone()),
            ("vehicle_id", self.vehicle_id.clone()),
            ("session_id", self.session_id.clone()),
            ("timestamp_semantics", self.timestamp_semantics.clone()),
            ("expected_sample_period_s", format!("{:.17e}", self.expected_sample_period_s)),
            ("maximum_gap_s", format!("{:.17e}", self.maximum_gap_s)),
            ("split", self.split.name().to_owned()),
            ("split_group", self.split_group.clone()),
            ("provenance_origin", self.provenance.origin.clone()),
            ("provenance_source", self.provenance.source.clone()),
            ("provenance_revision", self.provenance.revision.clone()),
            ("provenance_notes", self.provenance.notes.clone()),
            ("mapping_revision", self.mapping_revision.clone()),
            ("alignment_revision", self.alignment_revision.clone()),
            ("filter_revision", self.filter_revision.clone()),
        ] {
            writeln!(output, "{key}={}", escape(&value)).unwrap();
        }
        for field in &self.fields {
            writeln!(
                output,
                "field={}|{}|{}|{}|{}|{}|{}|{}|{:.17e}|{:.17e}",
                escape(&field.canonical_name),
                escape(&field.source_column),
                field.quantity.name(),
                escape(&field.unit.0),
                escape(&field.frame.0),
                field.role.name(),
                field.resampling.name(),
                field.transform.text(),
                field.metric_normalization_scale,
                field.metric_lag_search_bound_s
            )
            .unwrap();
        }
        output
    }

    pub fn from_text(text: &str) -> Result<Self> {
        let mut scalars = BTreeMap::new();
        let mut fields = Vec::new();
        for (line_number, raw) in text.lines().enumerate() {
            let line = raw.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let (key, value) = line
                .split_once('=')
                .ok_or_else(|| CorrelationError::InvalidManifest(format!("line {} has no '='", line_number + 1)))?;
            if key == "field" {
                let parts = split_escaped(value, '|')?;
                if parts.len() != 10 {
                    return Err(CorrelationError::InvalidManifest(format!(
                        "line {} field requires 10 values",
                        line_number + 1
                    )));
                }
                fields.push(FieldSchema {
                    canonical_name: parts[0].clone(),
                    source_column: parts[1].clone(),
                    quantity: Quantity::parse(&parts[2])?,
                    unit: Unit(parts[3].clone()),
                    frame: Frame(parts[4].clone()),
                    role: FieldRole::parse(&parts[5])?,
                    resampling: Resampling::parse(&parts[6])?,
                    transform: ValueTransform::parse(&parts[7])?,
                    metric_normalization_scale: parts[8].parse().map_err(|_| {
                        CorrelationError::InvalidManifest(format!("line {} invalid metric scale", line_number + 1))
                    })?,
                    metric_lag_search_bound_s: parts[9].parse().map_err(|_| {
                        CorrelationError::InvalidManifest(format!("line {} invalid lag bound", line_number + 1))
                    })?,
                });
            } else if scalars.insert(key.to_owned(), unescape(value)?).is_some() {
                return Err(CorrelationError::InvalidManifest(format!("duplicate key {key:?}")));
            }
        }
        let get = |key: &str| {
            scalars.get(key).cloned().ok_or_else(|| CorrelationError::InvalidManifest(format!("missing key {key:?}")))
        };
        let manifest = Self {
            schema_version: get("schema_version")?
                .parse()
                .map_err(|_| CorrelationError::InvalidManifest("schema_version must be an integer".to_owned()))?,
            dataset_id: get("dataset_id")?,
            format: DatasetFormat::parse(&get("format")?)?,
            source_title: get("source_title")?,
            source_uri: get("source_uri")?,
            license_id: get("license_id")?,
            license_verified: get("license_verified")?
                .parse()
                .map_err(|_| CorrelationError::InvalidManifest("license_verified must be true or false".to_owned()))?,
            content_checksum: get("content_checksum")?,
            vehicle_id: get("vehicle_id")?,
            session_id: get("session_id")?,
            timestamp_semantics: get("timestamp_semantics")?,
            expected_sample_period_s: get("expected_sample_period_s")?
                .parse()
                .map_err(|_| CorrelationError::InvalidManifest("invalid expected_sample_period_s".to_owned()))?,
            maximum_gap_s: get("maximum_gap_s")?
                .parse()
                .map_err(|_| CorrelationError::InvalidManifest("invalid maximum_gap_s".to_owned()))?,
            split: DatasetSplit::parse(&get("split")?)?,
            split_group: get("split_group")?,
            provenance: ProvenanceRecord {
                origin: get("provenance_origin")?,
                source: get("provenance_source")?,
                revision: get("provenance_revision")?,
                notes: get("provenance_notes")?,
            },
            mapping_revision: get("mapping_revision")?,
            alignment_revision: get("alignment_revision")?,
            filter_revision: get("filter_revision")?,
            fields,
        };
        manifest.validate()?;
        Ok(manifest)
    }
}

fn escape(value: &str) -> String {
    value.replace('\\', "\\\\").replace('\n', "\\n").replace('|', "\\p")
}

fn unescape(value: &str) -> Result<String> {
    let mut output = String::new();
    let mut chars = value.chars();
    while let Some(character) = chars.next() {
        if character != '\\' {
            output.push(character);
            continue;
        }
        match chars.next() {
            Some('\\') => output.push('\\'),
            Some('n') => output.push('\n'),
            Some('p') => output.push('|'),
            Some(other) => {
                return Err(CorrelationError::InvalidManifest(format!("invalid escape \\{other}")));
            }
            None => return Err(CorrelationError::InvalidManifest("trailing escape".to_owned())),
        }
    }
    Ok(output)
}

fn split_escaped(value: &str, separator: char) -> Result<Vec<String>> {
    let mut result = Vec::new();
    let mut current = String::new();
    let mut escaped = false;
    for character in value.chars() {
        if escaped {
            current.push('\\');
            current.push(character);
            escaped = false;
        } else if character == '\\' {
            escaped = true;
        } else if character == separator {
            result.push(unescape(&current)?);
            current.clear();
        } else {
            current.push(character);
        }
    }
    if escaped {
        return Err(CorrelationError::InvalidManifest("trailing escape".to_owned()));
    }
    result.push(unescape(&current)?);
    Ok(result)
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct TelemetryTable {
    pub time_s: Vec<f64>,
    pub channels: BTreeMap<String, Vec<f64>>,
}

impl TelemetryTable {
    pub fn validate(&self) -> Result<()> {
        if self.time_s.len() < 2 {
            return Err(CorrelationError::InvalidTelemetry("at least two samples are required".to_owned()));
        }
        for pair in self.time_s.windows(2) {
            if !pair[0].is_finite() || !pair[1].is_finite() || pair[1] <= pair[0] {
                return Err(CorrelationError::InvalidTelemetry(
                    "time must be finite and strictly increasing".to_owned(),
                ));
            }
        }
        if self.channels.is_empty() {
            return Err(CorrelationError::InvalidTelemetry("at least one channel is required".to_owned()));
        }
        for (name, values) in &self.channels {
            if values.len() != self.time_s.len() {
                return Err(CorrelationError::InvalidTelemetry(format!(
                    "channel {name:?} has {} samples, expected {}",
                    values.len(),
                    self.time_s.len()
                )));
            }
            if values.iter().any(|value| !value.is_finite()) {
                return Err(CorrelationError::InvalidTelemetry(format!("channel {name:?} contains non-finite data")));
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Default)]
pub struct ManifestCatalog {
    manifests: Vec<DatasetManifest>,
}

impl ManifestCatalog {
    pub fn new(manifests: Vec<DatasetManifest>) -> Result<Self> {
        let mut dataset_ids = BTreeSet::new();
        let mut group_splits: BTreeMap<&str, DatasetSplit> = BTreeMap::new();
        for manifest in &manifests {
            manifest.validate()?;
            if !dataset_ids.insert(&manifest.dataset_id) {
                return Err(CorrelationError::SplitViolation(format!(
                    "duplicate dataset_id {:?}",
                    manifest.dataset_id
                )));
            }
            if let Some(existing) = group_splits.insert(&manifest.split_group, manifest.split)
                && existing != manifest.split
            {
                return Err(CorrelationError::SplitViolation(format!(
                    "split_group {:?} occurs in both {} and {}",
                    manifest.split_group,
                    existing.name(),
                    manifest.split.name()
                )));
            }
        }
        Ok(Self { manifests })
    }

    pub fn manifests(&self) -> &[DatasetManifest] {
        &self.manifests
    }
}
