#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

/// Transform graph storage and lookup ([`BufferTree`], [`BufferObserver`]).
pub mod buffer;
/// Runtime configuration and config-file loading ([`BufferConfig`], [`get_config`]).
pub mod config;
/// Error type returned by buffer operations ([`TfError`]).
pub mod error;
/// Core value types: [`StampedIsometry`], [`TransformType`], [`TransformUpdate`].
pub mod types;
/// Loaders that ingest external model files into a [`BufferTree`] ([`UrdfLoader`]).
pub mod utils;

pub use buffer::{BufferObserver, BufferTree};
pub use config::{get_config, BufferConfig};
pub use error::TfError;
pub use types::{StampedIsometry, TransformType, TransformUpdate};
pub use utils::{FormatLoader, UrdfLoader};
