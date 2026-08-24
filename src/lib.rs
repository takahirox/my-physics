//! Deterministic, rendering-independent vehicle physics.
//!
//! Coordinates are right-handed and Three.js compatible: +X right, +Y up,
//! and -Z vehicle-forward. All public physical values use SI units and radians.

mod archive;
pub mod collision;
pub mod controls;
pub mod feedback;
pub mod math;
pub mod road;
pub mod tire;
pub mod vehicle;
pub mod world;

#[cfg(target_arch = "wasm32")]
mod wasm;

pub use archive::{ArchiveError, decode_input_history, encode_input_history};
pub use controls::{ControlOutput, DriverAids, DriverInput};
pub use feedback::{AudioFrame, FeedbackEvent, FeedbackEventKind, ForceFeedbackFrame};
pub use math::{Quat, Vec3};
pub use tire::{MagicFormulaTire, TireInput, TireModel, TireOutput};
pub use vehicle::{InterpolatedState, Telemetry, VehicleDefinition, VehicleState};
pub use world::{Fidelity, PhysicsWorld, SimulationConfig, Snapshot, StepError};
