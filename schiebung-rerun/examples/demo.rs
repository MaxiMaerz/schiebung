use rerun::RecordingStreamBuilder;
use schiebung::{BufferTree, StampedIsometry, TransformType};
use schiebung_rerun::RerunObserver;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = RecordingStreamBuilder::new("sun_earth_moon_demo").spawn()?;

    let mut buffer = BufferTree::new();

    let observer = RerunObserver::new(rec.clone(), true, "stable_time".to_string());
    buffer.register_observer(Box::new(observer));

    rec.set_timestamp_secs_since_epoch("stable_time", 0.0);
    rec.log(
        "Sun",
        &[
            &rerun::Ellipsoids3D::from_half_sizes([[0.15, 0.15, 0.15]])
                .with_colors([rerun::Color::from_rgb(255, 200, 0)])
                .with_fill_mode(rerun::FillMode::Solid) as &dyn rerun::AsComponents,
            &rerun::CoordinateFrame::new("Sun".to_string()),
        ],
    )?;

    rec.log(
        "Earth",
        &[
            &rerun::Ellipsoids3D::from_half_sizes([[0.08, 0.08, 0.08]])
                .with_colors([rerun::Color::from_rgb(50, 100, 200)])
                .with_fill_mode(rerun::FillMode::Solid) as &dyn rerun::AsComponents,
            &rerun::CoordinateFrame::new("Earth".to_string()),
        ],
    )?;

    rec.log(
        "Moon",
        &[
            &rerun::Ellipsoids3D::from_half_sizes([[0.04, 0.04, 0.04]])
                .with_colors([rerun::Color::from_rgb(180, 180, 180)])
                .with_fill_mode(rerun::FillMode::Solid) as &dyn rerun::AsComponents,
            &rerun::CoordinateFrame::new("Moon".to_string()),
        ],
    )?;

    let earth_orbit_radius = 2.0;
    let moon_orbit_radius = 0.5;
    let num_steps = 1000;
    let earth_period = 10.0;
    let moon_period = 2.0;

    for i in 0..num_steps {
        let time = i as f64 * 0.01;
        let earth_angle = (time / earth_period) * 2.0 * std::f64::consts::PI;
        let earth_x = earth_orbit_radius * earth_angle.cos();
        let earth_y = earth_orbit_radius * earth_angle.sin();

        let earth_transform =
            StampedIsometry::new([earth_x, earth_y, 0.0], [0.0, 0.0, 0.0, 1.0], time);
        buffer.update("Sun", "Earth", earth_transform, TransformType::Dynamic)?;

        let moon_angle = (time / moon_period) * 2.0 * std::f64::consts::PI;
        let moon_x = moon_orbit_radius * moon_angle.cos();
        let moon_y = moon_orbit_radius * moon_angle.sin();

        let moon_transform =
            StampedIsometry::new([moon_x, moon_y, 0.0], [0.0, 0.0, 0.0, 1.0], time);
        buffer.update("Earth", "Moon", moon_transform, TransformType::Dynamic)?;

        let transform = buffer.lookup_latest_transform("Moon", "Sun")?;
        let distance = transform.norm();
        rec.log(
            "Moon/distance",
            &[
                &rerun::Arrows3D::from_vectors([transform.translation(), [0.0, 0.0, 0.0]])
                    .with_labels([format!("{:.2}", distance)])
                    as &dyn rerun::AsComponents,
                &rerun::CoordinateFrame::new("Moon".to_string()),
            ],
        )?;
    }

    Ok(())
}
