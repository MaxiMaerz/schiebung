//! IPC types for iceoryx2 communication between schiebung client and server
//!
//! This module provides zero-copy types for efficient inter-process communication.

use iceoryx2::prelude::*;
use nalgebra::{Isometry, Quaternion, Translation3, UnitQuaternion, Vector3};
use schiebung::types::StampedIsometry;
use std::fmt;

// Re-export TransformType from schiebung-core-rs
pub use schiebung::types::TransformType;

#[derive(Debug, Clone, ZeroCopySend)]
#[repr(C)]
pub struct TransformRequest {
    pub from: [char; 100],
    pub to: [char; 100],
    pub time: f64,
}

#[derive(Debug, Clone, ZeroCopySend)]
#[repr(C)]
pub struct TransformResponse {
    pub time: f64,
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
}

#[derive(Debug, ZeroCopySend)]
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
