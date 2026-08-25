//! Provenance attached to authored vehicle parameter groups.
//!
//! Provenance is descriptive data. It never changes the physical equations or
//! applies a hidden scale to a parameter value.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParameterOrigin {
    Measured,
    Derived,
    Fitted,
    Estimated,
    Authored,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ParameterValidity {
    pub parameter: String,
    pub unit: String,
    pub minimum: f64,
    pub maximum: f64,
}

impl ParameterValidity {
    pub fn new(parameter: &str, unit: &str, minimum: f64, maximum: f64) -> Self {
        Self { parameter: parameter.into(), unit: unit.into(), minimum, maximum }
    }

    pub fn is_valid(&self) -> bool {
        !self.parameter.trim().is_empty()
            && !self.unit.trim().is_empty()
            && self.minimum.is_finite()
            && self.maximum.is_finite()
            && self.minimum <= self.maximum
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ParameterProvenance {
    pub origin: ParameterOrigin,
    pub source: String,
    pub revision: String,
    /// Relative one-sigma uncertainty when known. `None` means unquantified,
    /// not zero uncertainty.
    pub uncertainty_fraction: Option<f64>,
    pub valid_ranges: Vec<ParameterValidity>,
}

impl ParameterProvenance {
    pub fn new(
        origin: ParameterOrigin,
        source: &str,
        revision: &str,
        uncertainty_fraction: Option<f64>,
        valid_ranges: Vec<ParameterValidity>,
    ) -> Self {
        Self { origin, source: source.into(), revision: revision.into(), uncertainty_fraction, valid_ranges }
    }

    pub fn is_complete(&self) -> bool {
        !self.source.trim().is_empty()
            && !self.revision.trim().is_empty()
            && self.uncertainty_fraction.is_none_or(|value| value.is_finite() && value >= 0.0)
            && !self.valid_ranges.is_empty()
            && self.valid_ranges.iter().all(ParameterValidity::is_valid)
    }

    pub(crate) fn legacy_archive(version: u32) -> Self {
        Self::new(
            ParameterOrigin::Authored,
            "legacy snapshot; original parameter source was not archived",
            &format!("snapshot-v{version}"),
            None,
            vec![ParameterValidity::new("legacy_parameter_group", "unspecified", -f64::MAX, f64::MAX)],
        )
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct VehicleParameterProvenance {
    pub chassis_mass_properties: ParameterProvenance,
    pub aerodynamics: ParameterProvenance,
    pub front_wheels_and_tires: ParameterProvenance,
    pub rear_wheels_and_tires: ParameterProvenance,
    pub suspension: ParameterProvenance,
    pub brakes: ParameterProvenance,
    pub engine: ParameterProvenance,
    pub transmission_and_clutch: ParameterProvenance,
    pub fuel_system: ParameterProvenance,
}

impl VehicleParameterProvenance {
    pub const GROUP_COUNT: usize = 9;

    pub fn groups(&self) -> [(&'static str, &ParameterProvenance); Self::GROUP_COUNT] {
        [
            ("chassis_mass_properties", &self.chassis_mass_properties),
            ("aerodynamics", &self.aerodynamics),
            ("front_wheels_and_tires", &self.front_wheels_and_tires),
            ("rear_wheels_and_tires", &self.rear_wheels_and_tires),
            ("suspension", &self.suspension),
            ("brakes", &self.brakes),
            ("engine", &self.engine),
            ("transmission_and_clutch", &self.transmission_and_clutch),
            ("fuel_system", &self.fuel_system),
        ]
    }

    pub fn is_complete(&self) -> bool {
        self.groups().iter().all(|(_, provenance)| provenance.is_complete())
    }

    pub(crate) fn legacy_archive(version: u32) -> Self {
        let value = || ParameterProvenance::legacy_archive(version);
        Self {
            chassis_mass_properties: value(),
            aerodynamics: value(),
            front_wheels_and_tires: value(),
            rear_wheels_and_tires: value(),
            suspension: value(),
            brakes: value(),
            engine: value(),
            transmission_and_clutch: value(),
            fuel_system: value(),
        }
    }
}
