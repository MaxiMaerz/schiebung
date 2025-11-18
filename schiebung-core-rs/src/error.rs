/// Enumerates the different types of errors
#[derive(Clone, Debug)]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    AttemptedLookupInPast,
    /// Error due ti the transform not yet being available.
    AttemptedLookUpInFuture,
    /// There is no path between the from and to frame.
    CouldNotFindTransform,
    /// The graph is cyclic or the target has multiple incoming edges.
    InvalidGraph,
}

impl TfError {
    pub fn to_string(&self) -> String {
        match self {
            TfError::AttemptedLookupInPast => "TfError.AttemptedLookupInPast".to_string(),
            TfError::AttemptedLookUpInFuture => "TfError.AttemptedLookUpInFuture".to_string(),
            TfError::CouldNotFindTransform => "TfError.CouldNotFindTransform".to_string(),
            TfError::InvalidGraph => "TfError.InvalidGraph".to_string(),
        }
    }
}

impl std::fmt::Display for TfError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_string())
    }
}

impl std::error::Error for TfError {}
