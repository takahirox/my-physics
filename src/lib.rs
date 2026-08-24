//! Deterministic, rendering-independent vehicle physics.
//!
//! Coordinates are right-handed and Three.js compatible: +X right, +Y up,
//! and -Z vehicle-forward. All public physical values use SI units and radians.

pub mod collision;
pub mod controls;
pub mod math;
pub mod road;
pub mod tire;
pub mod vehicle;
pub mod world;

#[cfg(target_arch = "wasm32")]
mod wasm;

pub use controls::{ControlOutput, DriverAids, DriverInput};
pub use math::{Quat, Vec3};
pub use tire::{MagicFormulaTire, TireInput, TireModel, TireOutput};
pub use vehicle::{Telemetry, VehicleDefinition, VehicleState};
pub use world::{Fidelity, PhysicsWorld, SimulationConfig, Snapshot, StepError};
