use numpy::ndarray::Array2;
use numpy::{IntoPyArray, PyArray1, PyArray2, PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::{PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyFloat, PyType};
use pyo3::PyTypeInfo;

/// Resolve a Python `stamp` argument to nanoseconds.
///
/// Dispatch is **type-driven** to keep the call site short and unambiguous:
/// a Python `int` is treated as nanoseconds since the Unix epoch, and a
/// Python `float` is treated as seconds. This matches the `i64` /
/// `f64`-secs split used by [`CoreStampedIsometry::new`] and
/// [`CoreStampedIsometry::from_secs`].
///
/// Floats are checked before ints because PyO3 happily coerces `int` to
/// `f64`, but we want the integer path to win whenever the caller actually
/// passed an int. Booleans are an `int` subtype and resolve to ns for
/// consistency with Python semantics.
fn stamp_to_ns(stamp: &Bound<'_, PyAny>) -> PyResult<i64> {
    if PyFloat::is_type_of(stamp) {
        let secs: f64 = stamp.extract()?;
        Ok((secs * 1_000_000_000.0) as i64)
    } else if let Ok(ns) = stamp.extract::<i64>() {
        Ok(ns)
    } else {
        Err(PyTypeError::new_err(
            "stamp must be an int (nanoseconds since the Unix epoch) \
             or a float (seconds since the Unix epoch)",
        ))
    }
}

/// Build a 4×4 homogeneous transform matrix from a core StampedIsometry.
fn homogeneous_matrix(iso: &CoreStampedIsometry) -> Array2<f64> {
    // nalgebra's Isometry3 → 4×4 returns column-major; we want a row-major
    // ndarray for numpy. Build it explicitly to keep the layout obvious.
    let m = iso.isometry.to_homogeneous();
    let mut out = Array2::<f64>::zeros((4, 4));
    for r in 0..4 {
        for c in 0..4 {
            out[[r, c]] = m[(r, c)];
        }
    }
    out
}

use ::schiebung::{
    BufferObserver as CoreBufferObserver, BufferTree as CoreBufferTree,
    FormatLoader as CoreFormatLoader, StampedIsometry as CoreStampedIsometry,
    TfError as CoreTfError, TransformType as CoreTransformType,
    TransformUpdate as CoreTransformUpdate, UrdfLoader as CoreUrdfLoader,
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
    /// Create a new `StampedIsometry`.
    ///
    /// The `stamp` argument accepts either an `int` (interpreted as
    /// nanoseconds since the Unix epoch) or a `float` (interpreted as
    /// seconds since the Unix epoch). Dispatch is by Python type, so the
    /// two examples below are equivalent:
    ///
    /// ```python
    /// StampedIsometry([1, 2, 3], [0, 0, 0, 1], 10_500_000_000)  # int → ns
    /// StampedIsometry([1, 2, 3], [0, 0, 0, 1], 10.5)            # float → s
    /// ```
    ///
    /// # Arguments
    /// * `translation` - [x, y, z] position
    /// * `rotation` - [x, y, z, w] quaternion
    /// * `stamp` - Timestamp; `int` for nanoseconds or `float` for seconds.
    #[new]
    fn new(translation: [f64; 3], rotation: [f64; 4], stamp: Bound<'_, PyAny>) -> PyResult<Self> {
        let stamp_ns = stamp_to_ns(&stamp)?;
        Ok(StampedIsometry {
            inner: CoreStampedIsometry::new(translation, rotation, stamp_ns),
        })
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

    /// Build the 4×4 homogeneous transform matrix as a numpy array.
    ///
    /// Row-major layout: top-left 3×3 is the rotation matrix, top-right 3×1
    /// is the translation, bottom row is `[0, 0, 0, 1]`. Composes naturally
    /// via `mat @ point` and `mat1 @ mat2`.
    fn as_matrix<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f64>> {
        homogeneous_matrix(&self.inner).into_pyarray(py)
    }

    /// Get the translation as a numpy array of shape (3,).
    fn as_translation<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.inner.translation().to_vec().into_pyarray(py)
    }

    /// Get the rotation quaternion as a numpy array of shape (4,) in xyzw order.
    fn as_quaternion<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray1<f64>> {
        self.inner.rotation().to_vec().into_pyarray(py)
    }

    /// NumPy interop hook: makes `np.asarray(stamped_iso)` return the 4×4
    /// homogeneous transform matrix. The `dtype` argument is accepted for
    /// numpy compatibility; values that aren't `float64` (or `None`) raise.
    /// The `copy` argument (NumPy 2.0+) is accepted but ignored — we always
    /// return a fresh array.
    ///
    /// This means `StampedIsometry` works directly anywhere numpy expects
    /// an array — `np.linalg.inv(iso)`, `mat @ iso`, `np.array(iso)`, etc.
    #[pyo3(signature = (dtype=None, copy=None))]
    fn __array__<'py>(
        &self,
        py: Python<'py>,
        dtype: Option<Bound<'py, PyAny>>,
        copy: Option<Bound<'py, PyAny>>,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let _ = copy; // we always return a freshly-allocated array
        if let Some(dtype) = dtype {
            // Resolve to a numpy.dtype and check kind/itemsize match float64.
            let np = py.import("numpy")?;
            let requested = np.call_method1("dtype", (dtype,))?;
            let kind: String = requested.getattr("kind")?.extract()?;
            let itemsize: usize = requested.getattr("itemsize")?.extract()?;
            if kind != "f" || itemsize != 8 {
                return Err(PyValueError::new_err(
                    "StampedIsometry only supports float64 arrays; \
                     call .as_matrix().astype(...) for other dtypes",
                ));
            }
        }
        Ok(self.as_matrix(py))
    }

    /// Build a `StampedIsometry` from a 4×4 homogeneous transform matrix.
    ///
    /// Inverse of [`as_matrix`]/`__array__`. The matrix's bottom row is not
    /// validated — callers are expected to pass a well-formed homogeneous
    /// transform. Rotation is decomposed via `nalgebra::Rotation3` so any
    /// non-orthonormal 3×3 block is silently approximated.
    ///
    /// The `stamp` argument follows the same int-or-float convention as
    /// the constructor: `int` is nanoseconds, `float` is seconds.
    #[classmethod]
    #[pyo3(signature = (matrix, stamp=None))]
    fn from_matrix(
        _cls: &Bound<'_, PyType>,
        matrix: PyReadonlyArray2<'_, f64>,
        stamp: Option<Bound<'_, PyAny>>,
    ) -> PyResult<Self> {
        let stamp_ns = match stamp {
            Some(s) => stamp_to_ns(&s)?,
            None => 0,
        };
        let shape = matrix.shape();
        if shape != [4, 4] {
            return Err(PyValueError::new_err(format!(
                "matrix must have shape (4, 4), got {:?}",
                shape
            )));
        }
        let view = matrix.as_array();
        let translation = [view[[0, 3]], view[[1, 3]], view[[2, 3]]];
        let rot = nalgebra::Rotation3::from_matrix_unchecked(nalgebra::Matrix3::new(
            view[[0, 0]],
            view[[0, 1]],
            view[[0, 2]],
            view[[1, 0]],
            view[[1, 1]],
            view[[1, 2]],
            view[[2, 0]],
            view[[2, 1]],
            view[[2, 2]],
        ));
        let q = nalgebra::UnitQuaternion::from_rotation_matrix(&rot);
        let rotation = [q.i, q.j, q.k, q.w];
        Ok(StampedIsometry {
            inner: CoreStampedIsometry::new(translation, rotation, stamp_ns),
        })
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
    fn on_update(&self, updates: &[CoreTransformUpdate]) {
        // Acquire the GIL once for the whole batch and call the Python
        // callback per item, preserving the per-transform user-facing API.
        Python::attach(|py| {
            for update in updates {
                let py_transform = StampedIsometry::from(update.stamped_isometry.clone());
                let py_kind = TransformType::from(update.kind);

                if let Err(e) = self.callback.call1(
                    py,
                    (
                        update.from.clone(),
                        update.to.clone(),
                        py_transform,
                        py_kind,
                    ),
                ) {
                    eprintln!("Error calling Python observer callback: {}", e);
                    e.print(py);
                }
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

    /// Insert a single transform into the buffer.
    ///
    /// For inserting many transforms in one shot — and notifying observers
    /// (e.g. the rerun visualizer) once per batch so columnar observers can
    /// send their data in a single call — use [`update_batch`] instead.
    pub fn update(
        &mut self,
        from: String,
        to: String,
        stamped_isometry: StampedIsometry,
        kind: TransformType,
    ) -> PyResult<()> {
        let core_iso = CoreStampedIsometry::new(
            stamped_isometry.translation(),
            stamped_isometry.rotation(),
            stamped_isometry.stamp(),
        );
        let core_update = CoreTransformUpdate::new(from, to, core_iso, kind.into());

        self.inner
            .update(&[core_update])
            .map_err(core_err_to_pyerr)?;
        Ok(())
    }

    /// Insert many transforms into the buffer in a single bulk call.
    ///
    /// `updates` is a list of `(from, to, stamped_isometry, kind)` tuples.
    /// Observers are notified once per call with the full batch, which lets
    /// columnar observers send their data in one shot.
    ///
    /// The call is fail-fast: if any tuple is rejected (cycle / multiple
    /// parents), the call returns an error and earlier tuples in the list
    /// remain applied.
    pub fn update_batch(
        &mut self,
        updates: Vec<(String, String, StampedIsometry, TransformType)>,
    ) -> PyResult<()> {
        let core_updates: Vec<CoreTransformUpdate> = updates
            .into_iter()
            .map(|(from, to, stamped_isometry, kind)| {
                let core_iso = CoreStampedIsometry::new(
                    stamped_isometry.translation(),
                    stamped_isometry.rotation(),
                    stamped_isometry.stamp(),
                );
                CoreTransformUpdate::new(from, to, core_iso, kind.into())
            })
            .collect();

        self.inner
            .update(&core_updates)
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
    /// * `time` - Timestamp; `int` for nanoseconds or `float` for seconds
    ///   (same dispatch as the [`StampedIsometry`] constructor).
    pub fn lookup_transform(
        &mut self,
        from: String,
        to: String,
        time: Bound<'_, PyAny>,
    ) -> PyResult<StampedIsometry> {
        let time_ns = stamp_to_ns(&time)?;
        let result = self.inner.lookup_transform(&from, &to, time_ns);
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

    /// Register a Python callable as an observer.
    ///
    /// The callable will be invoked once per transform inserted via `update`
    /// or `update_batch`.
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
