use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use std::cmp::Ordering;
use std::fmt;

#[derive(Clone, Copy, Debug)]
pub enum TransformType {
    /// Changes over time
    Dynamic = 0,
    /// Does not change over time
    Static = 1,
}

impl TryFrom<u8> for TransformType {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(TransformType::Dynamic),
            1 => Ok(TransformType::Static),
            _ => Err(()),
        }
    }
}

impl TransformType {
    /// Create a static transform type
    pub fn static_transform() -> Self {
        TransformType::Static
    }

    /// Create a dynamic transform type
    pub fn dynamic_transform() -> Self {
        TransformType::Dynamic
    }
}

impl fmt::Display for TransformType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TransformType::Static => write!(f, "TransformType.STATIC"),
            TransformType::Dynamic => write!(f, "TransformType.DYNAMIC"),
        }
    }
}

#[derive(Clone, Debug)]
pub struct StampedIsometry {
    pub isometry: Isometry3<f64>,
    /// Timestamp in nanoseconds since Unix epoch
    pub stamp: i64,
}

impl PartialEq for StampedIsometry {
    fn eq(&self, other: &Self) -> bool {
        self.stamp == other.stamp
    }
}

impl Eq for StampedIsometry {}

impl Ord for StampedIsometry {
    fn cmp(&self, other: &Self) -> Ordering {
        self.stamp.cmp(&other.stamp)
    }
}

impl PartialOrd for StampedIsometry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl StampedIsometry {
    /// Create a new StampedIsometry with timestamp in nanoseconds
    pub fn new(translation: [f64; 3], rotation: [f64; 4], stamp_ns: i64) -> Self {
        let isometry = Isometry3::from_parts(
            Translation3::new(translation[0], translation[1], translation[2]),
            UnitQuaternion::from_quaternion(Quaternion::new(
                rotation[3], // w
                rotation[0], // x
                rotation[1], // y
                rotation[2], // z
            )),
        );
        StampedIsometry {
            isometry,
            stamp: stamp_ns,
        }
    }

    /// Create a new StampedIsometry with timestamp in seconds (f64)
    /// Convenience constructor for backwards compatibility
    pub fn from_secs(translation: [f64; 3], rotation: [f64; 4], stamp_secs: f64) -> Self {
        Self::new(translation, rotation, (stamp_secs * 1_000_000_000.0) as i64)
    }

    /// Get the translation as [x, y, z]
    pub fn translation(&self) -> [f64; 3] {
        let t = self.isometry.translation.vector;
        [t.x, t.y, t.z]
    }

    /// Get the rotation as [x, y, z, w] quaternion
    pub fn rotation(&self) -> [f64; 4] {
        let q = self.isometry.rotation.into_inner();
        [q.i, q.j, q.k, q.w]
    }

    /// Get the timestamp in nanoseconds
    pub fn stamp(&self) -> i64 {
        self.stamp
    }

    /// Get the timestamp in seconds as f64
    /// Careful here, this will truncate!
    pub fn stamp_secs(&self) -> f64 {
        self.stamp as f64 / 1_000_000_000.0
    }

    /// Get the timestamp as std::time::Duration from Unix epoch
    pub fn stamp_as_duration(&self) -> std::time::Duration {
        std::time::Duration::from_nanos(self.stamp as u64)
    }

    /// Get Euler angles (roll, pitch, yaw) in radians
    pub fn euler_angles(&self) -> [f64; 3] {
        let (roll, pitch, yaw) = self.isometry.rotation.euler_angles();
        [roll, pitch, yaw]
    }

    pub fn norm(&self) -> f64 {
        self.isometry.translation.vector.norm()
    }
}

impl fmt::Display for StampedIsometry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let t = self.translation();
        let r = self.rotation();
        write!(
            f,
            "StampedIsometry(translation=[{:.3}, {:.3}, {:.3}], rotation=[{:.3}, {:.3}, {:.3}, {:.3}], stamp={:.6}s)",
            t[0], t[1], t[2], r[0], r[1], r[2], r[3], self.stamp_secs()
        )
    }
}
