use nalgebra::UnitQuaternion;
use rerun::RecordingStreamBuilder;
use schiebung::{BufferTree, StampedIsometry, TransformType, TransformUpdate};
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

    // publish_static_transforms = true so the cube static below is logged
    // through the observer on `tf_static`. The URDF's own fixed joints are
    // NOT loaded into the buffer (no `UrdfLoader::load_into_buffer` call)
    // because `rec.log_file_from_path` already populates rerun's transform
    // tree for them; routing them through the observer too would double-log.
    let observer = RerunObserver::new(rec.clone(), true, "stable_time".to_string());
    buffer.register_observer(Box::new(observer));

    rec.log_file_from_path(urdf_path.to_str().unwrap(), None, true)?;

    // Place a small static cube in the robot's workspace so we can visualize
    // the distance from the wrist tip to it as the arm moves. Inserting it as
    // Static into the buffer makes the observer publish `base_link ->
    // target_cube` on the collapsed `tf_static` entity (named-frames graph),
    // registering `target_cube` as a frame at this pose. The box visual below
    // opts into that frame via `with_parent_frame`.
    let cube_pose = StampedIsometry::from_secs([0.6, 0.4, 1.0], [0.0, 0.0, 0.0, 1.0], 0.0);
    buffer.update(&[TransformUpdate::new(
        "base_link",
        "target_cube",
        cube_pose,
        TransformType::Static,
    )])?;

    rec.log_static(
        "target_cube",
        &[
            &rerun::Transform3D::default().with_parent_frame("target_cube".to_string())
                as &dyn rerun::AsComponents,
            &rerun::Boxes3D::from_half_sizes([[0.05, 0.05, 0.05]])
                .with_colors([rerun::Color::from_rgb(220, 60, 60)])
                .with_fill_mode(rerun::FillMode::Solid),
        ],
    )?;

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

        // Build all joint updates for this step into one batch — observer
        // fires once and rerun receives one columnar send per entity path.
        let updates: Vec<TransformUpdate> = dynamic_joints
            .iter()
            .enumerate()
            .map(|(joint_idx, (parent, child, xyz, rpy))| {
                let joint_angle = (angle + joint_idx as f64 * 0.5).sin() * 0.5;

                let (axis_roll, axis_pitch, axis_yaw) = match joint_idx {
                    0 => (0.0, 0.0, joint_angle), // shoulder_pan: Z axis
                    1 => (0.0, joint_angle, 0.0), // shoulder_lift: Y axis
                    2 => (0.0, joint_angle, 0.0), // elbow: Y axis
                    3 => (0.0, joint_angle, 0.0), // wrist_1: Y axis
                    4 => (0.0, 0.0, joint_angle), // wrist_2: Z axis
                    5 => (0.0, joint_angle, 0.0), // wrist_3: Y axis
                    _ => (0.0, 0.0, 0.0),
                };

                let base_rotation = UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]);
                let joint_rotation =
                    UnitQuaternion::from_euler_angles(axis_roll, axis_pitch, axis_yaw);
                let combined = base_rotation * joint_rotation;

                let quat = [combined.i, combined.j, combined.k, combined.w];
                let transform = StampedIsometry::from_secs(*xyz, quat, time);
                TransformUpdate::new(*parent, *child, transform, TransformType::Dynamic)
            })
            .collect();

        buffer.update(&updates)?;

        // Log the vector from the last wrist link to the cube, with the
        // distance as a label. Attach the entity to the URDF loader's
        // existing `wrist_3_link` frame via Transform3D::parent_frame so
        // rerun can trace the frame back to the view origin.
        let to_cube = buffer.lookup_latest_transform("wrist_3_link", "target_cube")?;
        let distance = to_cube.norm();
        rec.set_timestamp_secs_since_epoch("stable_time", time);
        rec.log(
            "wrist_3_link/to_cube",
            &[
                &rerun::Transform3D::default().with_parent_frame("wrist_3_link".to_string())
                    as &dyn rerun::AsComponents,
                &rerun::Arrows3D::from_vectors([to_cube.translation()])
                    .with_labels([format!("{:.2} m", distance)])
                    .with_colors([rerun::Color::from_rgb(220, 60, 60)]),
            ],
        )?;
    }

    Ok(())
}
