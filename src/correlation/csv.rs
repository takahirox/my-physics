use super::{CorrelationError, DatasetFormat, DatasetManifest, Result, TelemetryTable, ValueTransform};
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

/// Format boundary for licensed telemetry providers. Dataset-specific column
/// meaning belongs in a reviewed manifest/adapter, never in the physics plant.
pub trait TelemetryAdapter {
    fn format(&self) -> DatasetFormat;
    fn read_path(&self, path: &Path, manifest: &DatasetManifest) -> Result<TelemetryTable>;
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CsvTelemetryAdapter;

impl TelemetryAdapter for CsvTelemetryAdapter {
    fn format(&self) -> DatasetFormat {
        DatasetFormat::Csv
    }

    fn read_path(&self, path: &Path, manifest: &DatasetManifest) -> Result<TelemetryTable> {
        if manifest.format != DatasetFormat::Csv {
            return Err(CorrelationError::AdapterUnavailable(format!(
                "CSV adapter cannot read {}; register a provider adapter for {}",
                manifest.dataset_id,
                manifest.format.name()
            )));
        }
        let text = fs::read_to_string(path)?;
        parse_csv(&text, manifest)
    }
}

pub fn parse_csv(text: &str, manifest: &DatasetManifest) -> Result<TelemetryTable> {
    manifest.validate()?;
    let rows = parse_rows(text)?;
    let header = rows.first().ok_or_else(|| CorrelationError::InvalidTelemetry("CSV has no header".to_owned()))?;
    let mut header_index = BTreeMap::new();
    for (index, name) in header.iter().enumerate() {
        let normalized_name = name.trim();
        if header_index.insert(normalized_name, index).is_some() {
            return Err(CorrelationError::InvalidTelemetry(format!("duplicate CSV column {name:?}")));
        }
    }
    let field_indices: Vec<_> = manifest
        .fields
        .iter()
        .map(|field| {
            header_index.get(field.source_column.as_str()).copied().ok_or_else(|| {
                CorrelationError::InvalidTelemetry(format!(
                    "manifest field {:?} maps to missing CSV column {:?}",
                    field.canonical_name, field.source_column
                ))
            })
        })
        .collect::<Result<_>>()?;
    let time_field = manifest
        .fields
        .iter()
        .position(|field| field.role == super::FieldRole::Time)
        .expect("manifest validation requires time");
    let mut table = TelemetryTable::default();
    let mut transforms = vec![TransformState::default(); manifest.fields.len()];
    for field in &manifest.fields {
        if field.role != super::FieldRole::Time {
            table.channels.insert(field.canonical_name.clone(), Vec::with_capacity(rows.len().saturating_sub(1)));
        }
    }
    for (row_index, row) in rows.iter().enumerate().skip(1) {
        if row.len() != header.len() {
            return Err(CorrelationError::InvalidTelemetry(format!(
                "CSV row {} has {} columns; header has {}",
                row_index + 1,
                row.len(),
                header.len()
            )));
        }
        for (field_index, field) in manifest.fields.iter().enumerate() {
            let source_index = field_indices[field_index];
            let raw_value: f64 = row[source_index].trim().parse().map_err(|_| {
                CorrelationError::InvalidTelemetry(format!(
                    "CSV row {}, column {:?} is not a finite number: {:?}",
                    row_index + 1,
                    field.source_column,
                    row[source_index]
                ))
            })?;
            if !raw_value.is_finite() {
                return Err(CorrelationError::InvalidTelemetry(format!(
                    "CSV row {}, column {:?} is non-finite",
                    row_index + 1,
                    field.source_column
                )));
            }
            let value = transforms[field_index].apply(field.transform, raw_value)?;
            if field_index == time_field {
                table.time_s.push(value);
            } else {
                table.channels.get_mut(&field.canonical_name).expect("channel initialized").push(value);
            }
        }
    }
    table.validate()?;
    for pair in table.time_s.windows(2) {
        if pair[1] - pair[0] > manifest.maximum_gap_s + 1.0e-12 {
            return Err(CorrelationError::InvalidTelemetry(format!(
                "timestamp gap {:.9}s exceeds manifest maximum_gap_s {:.9}",
                pair[1] - pair[0],
                manifest.maximum_gap_s
            )));
        }
    }
    Ok(table)
}

#[derive(Clone, Copy, Debug, Default)]
struct TransformState {
    previous_raw: Option<f64>,
    previous_unwrapped: f64,
    first_unwrapped: f64,
}

impl TransformState {
    fn apply(&mut self, transform: ValueTransform, raw: f64) -> Result<f64> {
        let value = match transform {
            ValueTransform::Identity => raw,
            ValueTransform::Affine { scale, offset } => raw * scale + offset,
            ValueTransform::UnwrapAffine { period, scale, offset, relative_to_first } => {
                let unwrapped = if let Some(previous_raw) = self.previous_raw {
                    let mut delta = raw - previous_raw;
                    delta -= (delta / period).round() * period;
                    self.previous_unwrapped + delta
                } else {
                    // Normalize the first wrapped angle to the principal
                    // interval as well. A source value such as 343.8 degrees
                    // therefore starts at -16.2 rather than one full turn
                    // away from the physically equivalent steering angle.
                    let principal = raw - (raw / period).round() * period;
                    self.first_unwrapped = principal;
                    principal
                };
                self.previous_raw = Some(raw);
                self.previous_unwrapped = unwrapped;
                let base = if relative_to_first { unwrapped - self.first_unwrapped } else { unwrapped };
                base * scale + offset
            }
        };
        if !value.is_finite() {
            return Err(CorrelationError::InvalidTelemetry("field transform produced non-finite value".to_owned()));
        }
        Ok(value)
    }
}

/// RFC 4180-compatible records for UTF-8 telemetry. Embedded newlines in
/// quoted fields are accepted, even though numeric mapped fields will later be
/// rejected unless they parse as one finite number.
fn parse_rows(text: &str) -> Result<Vec<Vec<String>>> {
    let mut rows = Vec::new();
    let mut row = Vec::new();
    let mut field = String::new();
    let mut chars = text.chars().peekable();
    let mut quoted = false;
    while let Some(character) = chars.next() {
        if quoted {
            if character == '"' {
                if chars.peek() == Some(&'"') {
                    chars.next();
                    field.push('"');
                } else {
                    quoted = false;
                }
            } else {
                field.push(character);
            }
            continue;
        }
        match character {
            '"' if field.is_empty() => quoted = true,
            ',' => row.push(std::mem::take(&mut field)),
            '\n' => {
                row.push(std::mem::take(&mut field));
                if !row.iter().all(String::is_empty) {
                    rows.push(std::mem::take(&mut row));
                } else {
                    row.clear();
                }
            }
            '\r' if chars.peek() == Some(&'\n') => {}
            other => field.push(other),
        }
    }
    if quoted {
        return Err(CorrelationError::InvalidTelemetry("unterminated quoted CSV field".to_owned()));
    }
    if !field.is_empty() || !row.is_empty() {
        row.push(field);
        rows.push(row);
    }
    Ok(rows)
}
