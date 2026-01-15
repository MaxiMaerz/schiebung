use nalgebra::UnitQuaternion;
use rerun::RecordingStreamBuilder;
use schiebung::{BufferTree, FormatLoader, StampedIsometry, TransformType, UrdfLoader};
use schiebung_rerun::RerunObserver;
use std::env;
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();

    let urdf_path = if args.len() > 1 {
        PathBuf::from(&args[1])
    } else {
        let root = env::current_dir()?;
        root.join("resources").join("test_robot.urdf")
    };

    println!("Loading URDF from {}", urdf_path.display());
    let rec = RecordingStreamBuilder::new("urdf_demo").spawn()?;

    let mut buffer = BufferTree::new();

    let loader = UrdfLoader::new();
    loader.load_into_buffer(urdf_path.to_str().unwrap(), &mut buffer)?;

    let observer = RerunObserver::new(rec.clone(), false, "stable_time".to_string());
    buffer.register_observer(Box::new(observer));

    rec.log_file_from_path(urdf_path.to_str().unwrap(), None, true)?;

    // Define all dynamic (revolute) joints from the URDF with their initial transforms
    // Each entry: (parent_link, child_link, xyz, rpy)
    let dynamic_joints: &[(&str, &str, [f64; 3], [f64; 3])] = &[
        // shoulder_pan_joint: base_link -> shoulder_link
        (
            "base_link",
            "shoulder_link",
            [0.0, 0.0, 0.1273],
            [0.0, 0.0, 0.0],
        ),
        // shoulder_lift_joint: shoulder_link -> upper_arm_link
        (
            "shoulder_link",
            "upper_arm_link",
            [0.0, 0.220941, 0.0],
            [0.0, 1.57079632679, 0.0],
        ),
        // elbow_joint: upper_arm_link -> forearm_link
        (
            "upper_arm_link",
            "forearm_link",
            [0.0, -0.1719, 0.612],
            [0.0, 0.0, 0.0],
        ),
        // wrist_1_joint: forearm_link -> wrist_1_link
        (
            "forearm_link",
            "wrist_1_link",
            [0.0, 0.0, 0.5723],
            [0.0, 1.57079632679, 0.0],
        ),
        // wrist_2_joint: wrist_1_link -> wrist_2_link
        (
            "wrist_1_link",
            "wrist_2_link",
            [0.0, 0.1149, 0.0],
            [0.0, 0.0, 0.0],
        ),
        // wrist_3_joint: wrist_2_link -> wrist_3_link
        (
            "wrist_2_link",
            "wrist_3_link",
            [0.0, 0.0, 0.1157],
            [0.0, 0.0, 0.0],
        ),
    ];

    // Animation parameters
    let num_steps = 360;
    let duration = 5.0; // seconds

    for step in 0..num_steps {
        let time = step as f64 * (duration / num_steps as f64);
        let angle = (time / duration) * 2.0 * std::f64::consts::PI;

        for (joint_idx, (parent, child, xyz, rpy)) in dynamic_joints.iter().enumerate() {
            // Apply a phase-shifted sinusoidal rotation to each joint
            let joint_angle = (angle + joint_idx as f64 * 0.5).sin() * 0.5;

            // Determine rotation axis from URDF (simplified: use Z for pan joints, Y for lift/elbow)
            let (axis_roll, axis_pitch, axis_yaw) = match joint_idx {
                0 => (0.0, 0.0, joint_angle), // shoulder_pan: Z axis
                1 => (0.0, joint_angle, 0.0), // shoulder_lift: Y axis
                2 => (0.0, joint_angle, 0.0), // elbow: Y axis
                3 => (0.0, joint_angle, 0.0), // wrist_1: Y axis
                4 => (0.0, 0.0, joint_angle), // wrist_2: Z axis
                5 => (0.0, joint_angle, 0.0), // wrist_3: Y axis
                _ => (0.0, 0.0, 0.0),
            };

            // Combine base rotation from URDF with joint rotation
            let base_rotation = UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]);
            let joint_rotation = UnitQuaternion::from_euler_angles(axis_roll, axis_pitch, axis_yaw);
            let combined = base_rotation * joint_rotation;

            let quat = [combined.i, combined.j, combined.k, combined.w];
            let transform = StampedIsometry::from_secs(*xyz, quat, time);
            buffer.update(parent, child, transform, TransformType::Dynamic)?;
        }
    }

    Ok(())
}
