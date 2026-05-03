use crate::buffer::BufferTree;
use crate::error::TfError;
use crate::types::{StampedIsometry, TransformType, TransformUpdate};

/// Adapter for bulk-loading transforms from a model file into a
/// [`BufferTree`].
///
/// Implementors typically parse a robot description (URDF, USD, ...) and
/// turn each link-to-link offset into a [`TransformUpdate`] of kind
/// [`TransformType::Static`], then push the whole batch in a single
/// [`BufferTree::update`] call.
pub trait FormatLoader {
    /// Parse the file at `path` and apply every contained transform to
    /// `buffer` in a single batch. Returns [`TfError::LoaderError`] if the
    /// file cannot be read, is malformed, or its transforms violate the
    /// buffer's graph invariants.
    fn load_into_buffer(&self, path: &str, buffer: &mut BufferTree) -> Result<(), TfError>;
}

/// [`FormatLoader`] for URDF (Unified Robot Description Format) files.
///
/// Each `<joint>` becomes one static edge from `parent.link` to `child.link`
/// using the joint's `<origin xyz="..." rpy="..."/>`. All edges land in the
/// buffer with timestamp `0` because URDF describes the rest pose, not a
/// time series — drive the actuated joints separately by calling
/// [`BufferTree::update`] with [`TransformType::Dynamic`] entries on each
/// timestep.
pub struct UrdfLoader;

impl UrdfLoader {
    /// Construct a fresh loader. The struct holds no state, so this is
    /// equivalent to [`UrdfLoader::default`].
    pub fn new() -> Self {
        UrdfLoader
    }
}

impl Default for UrdfLoader {
    fn default() -> Self {
        Self::new()
    }
}

impl FormatLoader for UrdfLoader {
    fn load_into_buffer(&self, path: &str, buffer: &mut BufferTree) -> Result<(), TfError> {
        // Read and parse the URDF file
        let robot = urdf_rs::read_file(path).map_err(|e| {
            TfError::LoaderError(format!("Failed to read URDF file '{}': {}", path, e))
        })?;

        // Collect all joints into a single batch so observers (e.g. rerun) can
        // bulk-log them in one columnar call.
        use nalgebra::UnitQuaternion;
        let mut updates: Vec<TransformUpdate> = Vec::with_capacity(robot.joints.len());
        for joint in &robot.joints {
            let translation = [
                joint.origin.xyz[0],
                joint.origin.xyz[1],
                joint.origin.xyz[2],
            ];
            let (roll, pitch, yaw) = (
                joint.origin.rpy[0],
                joint.origin.rpy[1],
                joint.origin.rpy[2],
            );
            // nalgebra from_euler_angles uses intrinsic ZYX, equivalent to URDF's extrinsic XYZ.
            let rotation_quat = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
            let rotation = [
                rotation_quat.i,
                rotation_quat.j,
                rotation_quat.k,
                rotation_quat.w,
            ];
            let stamped_isometry = StampedIsometry::new(translation, rotation, 0);

            updates.push(TransformUpdate::new(
                joint.parent.link.clone(),
                joint.child.link.clone(),
                stamped_isometry,
                TransformType::Static,
            ));
        }

        buffer
            .update(&updates)
            .map_err(|e| TfError::LoaderError(format!("Failed to load URDF transforms: {}", e)))?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_urdf_loader_basic() {
        // Create a simple URDF string for testing
        let urdf_content = r#"<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>

  <joint name="joint1" type="fixed">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="1.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
  </joint>

  <joint name="joint2" type="fixed">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.0 1.0 0.0" rpy="0.0 0.0 1.5708"/>
  </joint>
</robot>"#;

        // Write to temporary file
        let temp_dir = std::env::temp_dir();
        let urdf_path = temp_dir.join("test_robot.urdf");
        std::fs::write(&urdf_path, urdf_content).unwrap();

        // Load URDF into buffer
        let mut buffer = BufferTree::new();
        let loader = UrdfLoader::new();
        let result = loader.load_into_buffer(urdf_path.to_str().unwrap(), &mut buffer);

        assert!(result.is_ok(), "Failed to load URDF: {:?}", result.err());

        // Verify transforms were loaded
        let tf1 = buffer.lookup_latest_transform("base_link", "link1");
        assert!(tf1.is_ok(), "Failed to lookup base_link -> link1");

        let tf1 = tf1.unwrap();
        let translation1 = tf1.translation();
        assert_relative_eq!(translation1[0], 1.0, epsilon = 1e-6);
        assert_relative_eq!(translation1[1], 0.0, epsilon = 1e-6);
        assert_relative_eq!(translation1[2], 0.0, epsilon = 1e-6);

        // Verify chained transform
        let tf_chain = buffer.lookup_latest_transform("base_link", "link2");
        assert!(tf_chain.is_ok(), "Failed to lookup base_link -> link2");

        // Clean up
        std::fs::remove_file(&urdf_path).ok();
    }

    #[test]
    fn test_urdf_loader_missing_file() {
        let mut buffer = BufferTree::new();
        let loader = UrdfLoader::new();
        let result = loader.load_into_buffer("/nonexistent/path.urdf", &mut buffer);

        assert!(result.is_err());
        assert!(matches!(result, Err(TfError::LoaderError(_))));
    }
}
