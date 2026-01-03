/// Errors that can occur in the comms package
#[derive(Debug, thiserror::Error)]
pub enum CommsError {
    #[error("Serialization error: {0}")]
    Serialization(#[from] capnp::Error),

    #[error("Cap'n Proto schema error: {0}")]
    NotInSchema(#[from] capnp::NotInSchema),

    #[error("Zenoh error: {0}")]
    Zenoh(String),

    #[error("Transform buffer error: {0}")]
    Buffer(#[from] schiebung::error::TfError),

    #[error("Configuration error: {0}")]
    Config(String),

    #[error("Response ID mismatch: expected {expected}, got {actual}")]
    ResponseIdMismatch { expected: u64, actual: u64 },

    #[error("No response received for transform request")]
    NoResponse,

    #[error("Mutex lock poisoned: {0}")]
    MutexPoisoned(String),

    #[error("UTF-8 error: {0}")]
    Utf8(#[from] std::str::Utf8Error),
}

impl From<String> for CommsError {
    fn from(s: String) -> Self {
        CommsError::Zenoh(s)
    }
}

impl From<&str> for CommsError {
    fn from(s: &str) -> Self {
        CommsError::Zenoh(s.to_string())
    }
}

impl<T> From<std::sync::PoisonError<T>> for CommsError {
    fn from(e: std::sync::PoisonError<T>) -> Self {
        CommsError::MutexPoisoned(e.to_string())
    }
}
