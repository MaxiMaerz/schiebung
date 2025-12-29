pub mod messages_capnp {
    include!(concat!(env!("OUT_DIR"), "/messages_capnp.rs"));
}

// Re-export the generated Cap'n Proto types
pub use messages_capnp::*;

// Re-export modules
pub mod client;
pub mod config;
pub mod error;
pub mod serializers;
pub mod server;

pub use client::TransformClient;
pub use config::ZenohConfig;
pub use error::CommsError;

// Type conversion helpers
impl From<schiebung::types::TransformType> for messages_capnp::TransformKind {
    fn from(tt: schiebung::types::TransformType) -> Self {
        use schiebung::types::TransformType;
        match tt {
            TransformType::Static => Self::Static,
            TransformType::Dynamic => Self::Dynamic,
        }
    }
}

impl From<messages_capnp::TransformKind> for schiebung::types::TransformType {
    fn from(tk: messages_capnp::TransformKind) -> Self {
        use schiebung::types::TransformType;
        match tk {
            messages_capnp::TransformKind::Static => TransformType::Static,
            messages_capnp::TransformKind::Dynamic => TransformType::Dynamic,
        }
    }
}
