///! # Schiebung-core-rs
///!
///! This crate contains the pure Rust core functionality of the Schiebung library.
///! It provides a buffer for storing and retrieving transforms without any Python dependencies.
///!
///! NOTE: The Buffer must be filled manually and will not interface with ROS or any other system.
///!       Interfaces to ROS are provided by the `schiebung-ros2` crate.
///!
///! ## Usage
///!
///! ```rust
///! use schiebung_core_rs::BufferTree;
///!
///! let buffer = BufferTree::new();
///!
///! let stamped_isometry = StampedIsometry {
///!     isometry: Isometry::from_parts(
///!         Translation3::new(
///!             1.0,
///!             2.0,
///!             3.0,
///!         ),
///!         UnitQuaternion::new_normalize(Quaternion::new(
///!             0.0,
///!             0.0,
///!             0.0,
///!             1.0,
///!         )),
///!     ),
///!     stamp: 1.0
///! };
///! buffer.update("base_link", "target_link", stamped_isometry, TransformType::Static);
///!
///! let transform = buffer.lookup_transform("base_link", "target_link", 1.0);
///! buffer.visualize();
///! ```

pub mod types;
pub mod config;
pub mod error;
pub mod buffer;

pub use types::{StampedIsometry, TransformType};
pub use error::TfError;
pub use buffer::BufferTree;
pub use config::{get_config, BufferConfig};