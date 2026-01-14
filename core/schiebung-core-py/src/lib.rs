use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use ::schiebung::{
    BufferObserver as CoreBufferObserver, BufferTree as CoreBufferTree,
    FormatLoader as CoreFormatLoader, StampedIsometry as CoreStampedIsometry,
    TfError as CoreTfError, TransformType as CoreTransformType, UrdfLoader as CoreUrdfLoader,
};

/// Python wrapper for TfError
#[derive(Clone, Debug, PartialEq, Eq)]
#[pyclass(eq, eq_int)]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    AttemptedLookupInPast,
    /// Error due ti the transform not yet being available.
    AttemptedLookUpInFuture,
    /// There is no path between the from and to frame.
    CouldNotFindTransform,
    /// The graph is cyclic or the target has multiple incoming edges.
    InvalidGraph,
    /// Error loading or parsing a file format (URDF, USD, etc.)
    LoaderError,
}

impl From<CoreTfError> for TfError {
    fn from(err: CoreTfError) -> Self {
        match err {
            CoreTfError::AttemptedLookupInPast(_) => TfError::AttemptedLookupInPast,
            CoreTfError::AttemptedLookUpInFuture(_) => TfError::AttemptedLookUpInFuture,
            CoreTfError::CouldNotFindTransform(_) => TfError::CouldNotFindTransform,
            CoreTfError::InvalidGraph(_) => TfError::InvalidGraph,
            CoreTfError::LoaderError(_) => TfError::LoaderError,
        }
    }
}

fn core_err_to_pyerr(err: CoreTfError) -> PyErr {
    match &err {
        CoreTfError::AttemptedLookupInPast(msg) => {
            PyValueError::new_err(format!("TfError.AttemptedLookupInPast: {}", msg))
        }
        CoreTfError::AttemptedLookUpInFuture(msg) => {
            PyValueError::new_err(format!("TfError.AttemptedLookUpInFuture: {}", msg))
        }
        CoreTfError::CouldNotFindTransform(msg) => {
            PyValueError::new_err(format!("TfError.CouldNotFindTransform: {}", msg))
        }
        CoreTfError::InvalidGraph(msg) => {
            PyValueError::new_err(format!("TfError.InvalidGraph: {}", msg))
        }
        CoreTfError::LoaderError(msg) => {
            PyValueError::new_err(format!("TfError.LoaderError: {}", msg))
        }
    }
}

impl TfError {
    fn to_string(&self) -> String {
        match self {
            TfError::AttemptedLookupInPast => "TfError.AttemptedLookupInPast".to_string(),
            TfError::AttemptedLookUpInFuture => "TfError.AttemptedLookUpInFuture".to_string(),
            TfError::CouldNotFindTransform => "TfError.CouldNotFindTransform".to_string(),
            TfError::InvalidGraph => "TfError.InvalidGraph".to_string(),
            TfError::LoaderError => "TfError.LoaderError".to_string(),
        }
    }
}

impl std::convert::From<TfError> for PyErr {
    fn from(err: TfError) -> PyErr {
        PyValueError::new_err(err.to_string())
    }
}

/// Python wrapper for TransformType
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[pyclass(eq, eq_int)]
pub enum TransformType {
    /// Changes over time
    Dynamic = 0,
    /// Does not change over time
    Static = 1,
}

impl From<CoreTransformType> for TransformType {
    fn from(transform_type: CoreTransformType) -> Self {
        match transform_type {
            CoreTransformType::Dynamic => TransformType::Dynamic,
            CoreTransformType::Static => TransformType::Static,
        }
    }
}

impl From<TransformType> for CoreTransformType {
    fn from(transform_type: TransformType) -> Self {
        match transform_type {
            TransformType::Dynamic => CoreTransformType::Dynamic,
            TransformType::Static => CoreTransformType::Static,
        }
    }
}

#[pymethods]
impl TransformType {
    /// Create a static transform type
    #[staticmethod]
    fn static_transform() -> Self {
        TransformType::Static
    }

    /// Create a dynamic transform type
    #[staticmethod]
    fn dynamic_transform() -> Self {
        TransformType::Dynamic
    }

    fn __repr__(&self) -> String {
        match self {
            TransformType::Static => "TransformType.STATIC".to_string(),
            TransformType::Dynamic => "TransformType.DYNAMIC".to_string(),
        }
    }
}

/// Python wrapper for StampedIsometry
#[derive(Clone, Debug)]
#[pyclass]
pub struct StampedIsometry {
    /// The underlying core stamped isometry (public for inter-crate access)
    pub inner: CoreStampedIsometry,
}

impl From<CoreStampedIsometry> for StampedIsometry {
    fn from(stamped_isometry: CoreStampedIsometry) -> Self {
        StampedIsometry {
            inner: stamped_isometry,
        }
    }
}

#[pymethods]
impl StampedIsometry {
    /// Create a new StampedIsometry with timestamp in nanoseconds
    ///
    /// # Arguments
    /// * `translation` - [x, y, z] position
    /// * `rotation` - [x, y, z, w] quaternion
    /// * `stamp_ns` - Timestamp in nanoseconds since Unix epoch
    #[new]
    fn new(translation: [f64; 3], rotation: [f64; 4], stamp_ns: i64) -> Self {
        StampedIsometry {
            inner: CoreStampedIsometry::new(translation, rotation, stamp_ns),
        }
    }

    /// Create a new StampedIsometry with timestamp in seconds (float)
    /// Convenience constructor for backwards compatibility
    ///
    /// # Arguments
    /// * `translation` - [x, y, z] position
    /// * `rotation` - [x, y, z, w] quaternion
    /// * `stamp_secs` - Timestamp in seconds since Unix epoch (float)
    #[staticmethod]
    fn from_secs(translation: [f64; 3], rotation: [f64; 4], stamp_secs: f64) -> Self {
        StampedIsometry {
            inner: CoreStampedIsometry::from_secs(translation, rotation, stamp_secs),
        }
    }

    /// Get the translation as [x, y, z]
    fn translation(&self) -> [f64; 3] {
        self.inner.translation()
    }

    /// Get the rotation as [x, y, z, w] quaternion
    fn rotation(&self) -> [f64; 4] {
        self.inner.rotation()
    }

    /// Get the timestamp in nanoseconds since Unix epoch
    fn stamp(&self) -> i64 {
        self.inner.stamp()
    }

    /// Get the timestamp in seconds as float
    fn stamp_secs(&self) -> f64 {
        self.inner.stamp_secs()
    }

    /// Get the timestamp as a Python datetime object (UTC)
    ///
    /// Returns a timezone-aware datetime in UTC
    fn stamp_datetime(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
        let datetime_module = py.import("datetime")?;
        let datetime_class = datetime_module.getattr("datetime")?;
        let timezone_class = datetime_module.getattr("timezone")?;
        let utc = timezone_class.getattr("utc")?;

        // Use fromtimestamp with UTC timezone
        let result =
            datetime_class.call_method1("fromtimestamp", (self.inner.stamp_secs(), utc))?;
        Ok(result.unbind())
    }

    /// Get Euler angles (roll, pitch, yaw) in radians
    fn euler_angles(&self) -> [f64; 3] {
        self.inner.euler_angles()
    }

    fn __repr__(&self) -> String {
        format!("{}", self.inner)
    }
}

/// Python callback observer wrapper
/// This struct wraps a Python callable and implements the BufferObserver trait
/// allowing Python functions to be registered as observers
struct PyBufferObserver {
    callback: Py<PyAny>,
}

impl PyBufferObserver {
    fn new(callback: Py<PyAny>) -> Self {
        PyBufferObserver { callback }
    }
}

impl CoreBufferObserver for PyBufferObserver {
    fn on_update(
        &self,
        from: &str,
        to: &str,
        transform: &CoreStampedIsometry,
        kind: CoreTransformType,
    ) {
        // Acquire the GIL to call the Python function
        Python::attach(|py| {
            // Convert Rust types to Python types
            let py_transform = StampedIsometry::from(transform.clone());
            let py_kind = TransformType::from(kind);

            // Call the Python callback
            // We catch errors and log them rather than panicking
            if let Err(e) = self.callback.call1(
                py,
                (from.to_string(), to.to_string(), py_transform, py_kind),
            ) {
                // Print the error to stderr
                eprintln!("Error calling Python observer callback: {}", e);
                // Print the Python traceback if available
                e.print(py);
            }
        });
    }
}

/// Python wrapper for BufferTree
#[pyclass]
pub struct BufferTree {
    /// The underlying core buffer tree (public for inter-crate access)
    pub inner: CoreBufferTree,
}

#[pymethods]
impl BufferTree {
    #[new]
    pub fn new() -> Self {
        BufferTree {
            inner: CoreBufferTree::new(),
        }
    }

    /// Either update or push a transform to the graph
    /// Panics if the graph becomes cyclic
    pub fn update(
        &mut self,
        from: String,
        to: String,
        stamped_isometry: StampedIsometry,
        kind: TransformType,
    ) -> PyResult<()> {
        let core_stamped_isometry = CoreStampedIsometry::new(
            stamped_isometry.translation(),
            stamped_isometry.rotation(),
            stamped_isometry.stamp(),
        );

        self.inner
            .update(&from, &to, core_stamped_isometry, kind.into())
            .map_err(core_err_to_pyerr)?;
        Ok(())
    }

    /// Lookup the latest transform without any checks
    /// This can be used for static transforms or if the user does not care if the
    /// transform is still valid.
    /// NOTE: This might give you outdated transforms!
    pub fn lookup_latest_transform(
        &mut self,
        from: String,
        to: String,
    ) -> PyResult<StampedIsometry> {
        let result = self.inner.lookup_latest_transform(&from, &to);
        match result {
            Ok(transform) => Ok(StampedIsometry::from(transform)),
            Err(e) => Err(TfError::from(e).into()),
        }
    }

    /// Lookup the transform at time
    /// This will look for a transform at the provided time and can "time travel"
    /// If any edge contains a transform older then time a AttemptedLookupInPast is raised
    /// If the time is younger then any transform AttemptedLookUpInFuture is raised
    /// If there is no perfect match the transforms around this time are interpolated
    /// The interpolation is weighted with the distance to the time stamps
    ///
    /// # Arguments
    /// * `from` - Source frame name
    /// * `to` - Target frame name
    /// * `time` - Time in nanoseconds since Unix epoch
    pub fn lookup_transform(
        &mut self,
        from: String,
        to: String,
        time: i64,
    ) -> PyResult<StampedIsometry> {
        let result = self.inner.lookup_transform(&from, &to, time);
        match result {
            Ok(transform) => Ok(StampedIsometry::from(transform)),
            Err(e) => Err(core_err_to_pyerr(e)),
        }
    }

    /// Visualize the buffer tree as a DOT graph
    /// Can not use internal visualizer because we Store the nodes in self.index
    pub fn visualize(&self) -> String {
        self.inner.visualize()
    }

    /// Save the buffer tree as a PDF and dot file
    /// Runs graphiz to generate the PDF, fails if graphiz is not installed
    pub fn save_visualization(&self) -> PyResult<()> {
        self.inner
            .save_visualization()
            .map_err(|e| PyValueError::new_err(format!("Failed to save visualization: {}", e)))
    }

    /// Register a Python callable as an observer
    ///
    /// The callable will be invoked whenever a transform is updated.
    /// The callable signature should be: callback(from: str, to: str, transform: StampedIsometry, kind: TransformType) -> None
    ///
    /// When registered, the observer will immediately receive callbacks for all existing transforms in the buffer.
    ///
    /// # Arguments
    /// * `callback` - A Python callable that will be called on each transform update
    ///
    /// # Example
    /// ```python
    /// def my_observer(from_frame, to_frame, transform, kind):
    ///     print(f"Transform update: {from_frame} -> {to_frame}")
    ///
    /// buffer = BufferTree()
    /// buffer.register_observer(my_observer)
    /// ```
    pub fn register_observer(&mut self, py: Python<'_>, callback: Py<PyAny>) -> PyResult<()> {
        // Verify the callback is callable
        if !callback.bind(py).is_callable() {
            return Err(PyValueError::new_err(
                "Observer must be a callable (function or callable object)",
            ));
        }

        // Create the observer wrapper and register it
        let observer = PyBufferObserver::new(callback);
        self.inner.register_observer(Box::new(observer));
        Ok(())
    }
}

/// Python wrapper for UrdfLoader
#[pyclass]
pub struct UrdfLoader {
    inner: CoreUrdfLoader,
}

#[pymethods]
impl UrdfLoader {
    #[new]
    pub fn new() -> Self {
        UrdfLoader {
            inner: CoreUrdfLoader::new(),
        }
    }

    /// Load transforms from a URDF file into the provided buffer
    pub fn load_into_buffer(&self, path: String, buffer: &mut BufferTree) -> PyResult<()> {
        self.inner
            .load_into_buffer(&path, &mut buffer.inner)
            .map_err(core_err_to_pyerr)?;
        Ok(())
    }
}

/// Python bindings for schiebung-core
#[pymodule]
fn schiebung(_py: Python, m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<BufferTree>()?;
    m.add_class::<StampedIsometry>()?;
    m.add_class::<TransformType>()?;
    m.add_class::<TfError>()?;
    m.add_class::<UrdfLoader>()?;
    Ok(())
}
