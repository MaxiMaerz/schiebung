/// Errors that can occur in the comms package
#[derive(Debug, thiserror::Error)]
pub enum CommsError {
    #[error("Zenoh error: {0}")]
    Zenoh(String),

    #[error("Transform lookup failed: {0}")]
    LookupFailed(String),

    #[error("Transform buffer error: {0}")]
    Buffer(#[from] schiebung::error::TfError),

    #[error("Configuration error: {0}")]
    Config(String),

    #[error("No response received for transform request")]
    NoResponse,

    #[error("Mutex lock poisoned: {0}")]
    MutexPoisoned(String),
}

impl From<String> for CommsError {
    fn from(s: String) -> Self {
        CommsError::Zenoh(s)
    }
}

/// hiroz surfaces failures as `zenoh::Result`, whose error is a boxed
/// `dyn Error`. Flattening it to a message here keeps `?` usable on every
/// hiroz call without leaking the boxed type into our public API.
impl From<zenoh::Error> for CommsError {
    fn from(e: zenoh::Error) -> Self {
        CommsError::Zenoh(e.to_string())
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
