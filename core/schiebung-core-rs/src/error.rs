/// Errors returned by [`BufferTree`](crate::BufferTree) lookups and updates.
///
/// Each variant carries a human-readable message describing the offending
/// frames or graph state. The wrapped [`String`] is meant for logs and error
/// reporting — match on the variant for programmatic handling.
#[derive(Clone, Debug)]
pub enum TfError {
    /// The requested timestamp is older than the oldest sample retained in
    /// the per-edge history. Increase [`BufferConfig::max_history_size`](crate::BufferConfig)
    /// or query a more recent stamp.
    AttemptedLookupInPast(String),
    /// The requested timestamp is newer than the newest sample on this edge.
    /// The transform has not been published yet.
    AttemptedLookUpInFuture(String),
    /// No connecting path exists between `from` and `to` in the current graph.
    /// The frames may not be linked yet, or one of them is unknown.
    CouldNotFindTransform(String),
    /// Inserting the requested edge would create a cycle, or the child frame
    /// already has a different parent. The graph must remain a forest.
    InvalidGraph(String),
    /// Failed to load or parse a model file (URDF, USD, etc.) into the buffer.
    LoaderError(String),
}

impl TfError {
    /// Render the error as a `TfError.<Variant>: <message>` string suitable
    /// for log output. Same content as the [`Display`](std::fmt::Display)
    /// impl; provided as a method for callers that want it without going
    /// through formatting machinery.
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
            TfError::LoaderError(msg) => format!("TfError.LoaderError: {}", msg),
        }
    }
}

impl std::fmt::Display for TfError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_string())
    }
}

impl std::error::Error for TfError {}
