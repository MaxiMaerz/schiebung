/// Enumerates the different types of errors
#[derive(Clone, Debug)]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    AttemptedLookupInPast(String),
    /// Error due ti the transform not yet being available.
    AttemptedLookUpInFuture(String),
    /// There is no path between the from and to frame.
    CouldNotFindTransform(String),
    /// The graph is cyclic or the target has multiple incoming edges.
    InvalidGraph(String),
}

impl TfError {
    pub fn to_string(&self) -> String {
        match self {
            TfError::AttemptedLookupInPast(msg) => {
                format!("TfError.AttemptedLookupInPast: {}", msg)
            }
            TfError::AttemptedLookUpInFuture(msg) => {
                format!("TfError.AttemptedLookUpInFuture: {}", msg)
            }
            TfError::CouldNotFindTransform(msg) => {
                format!("TfError.CouldNotFindTransform: {}", msg)
            }
            TfError::InvalidGraph(msg) => format!("TfError.InvalidGraph: {}", msg),
        }
    }
}

impl std::fmt::Display for TfError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_string())
    }
}

impl std::error::Error for TfError {}
