/// Serialized-size estimation for the generated message types.
///
/// hiroz-codegen emits `impl crate::size_estimation::SizeEstimation for ...`
/// for every message, because upstream it is included into the `hiroz_msgs`
/// crate, which defines this trait. `hiroz_msgs` is not published on crates.io
/// and neither `hiroz` nor `hiroz-cdr` exports the trait, so we declare it here
/// -- it is what the generated code links against.
///
/// hiroz only consults these estimates when sizing shared-memory buffers; the
/// regular CDR pub/sub path does not use them.
pub mod size_estimation {
    /// An upper bound on a message's CDR-serialized size, excluding the header.
    pub trait SizeEstimation {
        /// Estimated CDR size of `self` in bytes.
        fn estimated_cdr_size(&self) -> usize;
    }
}

/// ROS 2 message types generated from the vendored definitions in `ros_msgs/`.
///
/// These are byte-exact copies of the ROS 2 Jazzy definitions (plus the custom
/// `schiebung_msgs` package), so the generated RIHS01 type hashes match a real
/// ROS 2 system running `rmw_zenoh`.
///
/// hiroz-codegen namespaces everything under `ros`, with services in a `srv`
/// submodule -- e.g. `msgs::ros::tf2_msgs::TFMessage` and
/// `msgs::ros::schiebung_msgs::srv::LookupTransform`.
pub mod msgs {
    include!(concat!(env!("OUT_DIR"), "/generated.rs"));
}

pub mod client;
pub mod config;
pub mod conversions;
pub mod error;
pub mod qos;
pub mod server;

pub use client::TransformClient;
pub use config::ZenohConfig;
pub use error::CommsError;
