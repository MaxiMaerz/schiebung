use nalgebra::{Isometry, Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
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

#[derive(Debug, Clone)]
#[repr(C)]
pub struct TransformRequest {
    pub id: u128,
    pub from: [char; 100],
    pub to: [char; 100],
    pub time: f64,
}

#[derive(Debug, Clone)]
#[repr(C)]
pub struct TransformResponse {
    pub id: u128,
    pub time: f64,
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
}

#[derive(Debug)]
#[repr(C)]
pub struct NewTransform {
    pub from: [char; 100],
    pub to: [char; 100],
    pub time: f64,
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
    pub kind: u8,
}

#[derive(Clone, Debug)]
pub struct StampedTransform {
    stamp: f64,
    translation: Vector3<f64>,
    rotation: UnitQuaternion<f64>,
}

impl From<TransformResponse> for StampedTransform {
    fn from(response: TransformResponse) -> Self {
        let translation_vector = Vector3::new(
            response.translation[0],
            response.translation[1],
            response.translation[2],
        );

        let rotation_quaternion = UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
            response.rotation[3],
            response.rotation[0],
            response.rotation[1],
            response.rotation[2],
        ));

        StampedTransform {
            stamp: response.time,
            translation: translation_vector,
            rotation: rotation_quaternion,
        }
    }
}

impl fmt::Display for StampedTransform {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Convert quaternion to Euler angles
        write!(
            f,
            "stamp: {},\ntranslation: {:.3}, {:.3}, {:.3},\nrotation (xyzw): {:.3}, {:.3}, {:.3}, {:.3},\nrotation (rpy): {:.3}, {:.3}, {:.3}",
            self.stamp, self.translation.x, self.translation.y, self.translation.z,
            self.rotation.i, self.rotation.j, self.rotation.k, self.rotation.w,
            self.rotation.euler_angles().0, self.rotation.euler_angles().1, self.rotation.euler_angles().2
        )
    }
}

#[derive(Clone, Debug)]
pub struct StampedIsometry {
    pub isometry: Isometry3<f64>,
    pub stamp: f64,
}

impl PartialEq for StampedIsometry {
    fn eq(&self, other: &Self) -> bool {
        self.stamp == other.stamp
    }
}

impl Eq for StampedIsometry {}

impl Ord for StampedIsometry {
    fn cmp(&self, other: &Self) -> Ordering {
        self.stamp.partial_cmp(&other.stamp).unwrap()
    }
}

impl PartialOrd for StampedIsometry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl StampedIsometry {
    pub fn new(translation: [f64; 3], rotation: [f64; 4], stamp: f64) -> Self {
        let isometry = Isometry3::from_parts(
            Translation3::new(translation[0], translation[1], translation[2]),
            UnitQuaternion::from_quaternion(Quaternion::new(
                rotation[3], // w
                rotation[0], // x
                rotation[1], // y
                rotation[2], // z
            )),
        );
        StampedIsometry { isometry, stamp }
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

    /// Get the timestamp
    pub fn stamp(&self) -> f64 {
        self.stamp
    }

    /// Get Euler angles (roll, pitch, yaw) in radians
    pub fn euler_angles(&self) -> [f64; 3] {
        let (roll, pitch, yaw) = self.isometry.rotation.euler_angles();
        [roll, pitch, yaw]
    }
}

impl fmt::Display for StampedIsometry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let t = self.translation();
        let r = self.rotation();
        write!(
            f,
            "StampedIsometry(translation=[{:.3}, {:.3}, {:.3}], rotation=[{:.3}, {:.3}, {:.3}, {:.3}], stamp={:.3})",
            t[0], t[1], t[2], r[0], r[1], r[2], r[3], self.stamp()
        )
    }
}

impl From<TransformResponse> for StampedIsometry {
    fn from(response: TransformResponse) -> Self {
        let isometry = Isometry::from_parts(
            Translation3::new(
                response.translation[0],
                response.translation[1],
                response.translation[2],
            ),
            UnitQuaternion::new_normalize(Quaternion::new(
                response.rotation[3],
                response.rotation[0],
                response.rotation[1],
                response.rotation[2],
            )),
        );
        StampedIsometry {
            isometry,
            stamp: response.time,
        }
    }
}
