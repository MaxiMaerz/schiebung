use nalgebra::UnitQuaternion;
use rerun::RecordingStreamBuilder;
use schiebung::{BufferTree, FormatLoader, StampedIsometry, TransformType, UrdfLoader};
use schiebung_rerun::RerunObserver;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let urdf_path = "../resources/test_robot.urdf";
    let rec = RecordingStreamBuilder::new("urdf_demo").spawn()?;

    let mut buffer = BufferTree::new();

    let loader = UrdfLoader::new();
    loader.load_into_buffer(urdf_path, &mut buffer)?;

    let observer = RerunObserver::new(rec.clone(), false, "stable_time".to_string());
    buffer.register_observer(Box::new(observer));

    rec.log_file_from_path(urdf_path, None, true)?;

    let num_steps = 360;
    let rotation_period = 5.0;

    for i in 0..num_steps {
        let time = i as f64 * (rotation_period / num_steps as f64);

        let angle = (time / rotation_period) * 2.0 * std::f64::consts::PI;
        let base_translation = [0.5, 0.0, 0.0];
        let base_rotation = UnitQuaternion::from_euler_angles(0.0, 1.5708, 0.0);
        let revolute_rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, angle);

        let combined_rotation = base_rotation * revolute_rotation;
        let rotation = [
            combined_rotation.i,
            combined_rotation.j,
            combined_rotation.k,
            combined_rotation.w,
        ];

        let transform = StampedIsometry::from_secs(base_translation, rotation, time);

        buffer.update("link1", "link2", transform, TransformType::Dynamic)?;
    }

    Ok(())
}
