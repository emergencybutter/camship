//! Camship core: the cinematic camera director, free of any X-Plane dependency
//! so every behaviour can be unit-tested with `cargo test`.

pub mod director;
pub mod math;
pub mod settings;
pub mod shots;

pub use director::{CameraPose, Director, Subject};
pub use math::{Attitude, Vec3};
pub use settings::Settings;
pub use shots::{Group, Shot, CATALOG};
