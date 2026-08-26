//! Deterministic, rendering-independent vehicle physics.
//!
//! Coordinates are right-handed and Three.js compatible: +X right, +Y up,
//! and -Z vehicle-forward. All public physical values use SI units and radians.

mod archive;
pub mod circuit;
pub mod collision;
pub mod controls;
pub mod correlation;
pub mod feedback;
pub mod math;
pub mod provenance;
pub mod road;
pub mod tire;
pub mod validation;
pub mod vehicle;
pub mod world;

#[cfg(target_arch = "wasm32")]
mod wasm;

pub use archive::{ArchiveError, decode_input_history, encode_input_history};
pub use circuit::{CIRCUIT_HALF_WIDTH_M, CircuitSegment, CircuitSurfaceSample};
pub use controls::{ControlOutput, DriverAids, DriverInput, KeyboardSteeringAssist, speed_sensitive_steering_limit};
pub use feedback::{AudioFrame, FeedbackEvent, FeedbackEventKind, ForceFeedbackFrame};
pub use math::{Quat, Vec3};
pub use provenance::{ParameterOrigin, ParameterProvenance, ParameterValidity, VehicleParameterProvenance};
pub use tire::{MagicFormulaTire, TireInput, TireModel, TireOutput};
pub use vehicle::{InterpolatedState, Telemetry, VehicleDefinition, VehiclePreset, VehicleState};
pub use world::{
    DEMO_TRACK_HALF_WIDTH_M, Fidelity, GroundSurface, PhysicsWorld, SimulationConfig, Snapshot, StepError,
};
