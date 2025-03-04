use nalgebra::{Quaternion, Translation3, UnitQuaternion};
use schiebung_client::{ListenerClient, PublisherClient};
use std::time::Instant;
fn main() {
    env_logger::init();
    let pub_client = PublisherClient::new();
    let translation = Translation3::new(0.0, 0.0, 1.0);
    let rotation = UnitQuaternion::new_normalize(Quaternion::new(0.0, 0.0, 0.0, 1.0));
    // pub_client.send_transform(&"foo".to_string(), &"bar".to_string(), translation, rotation, 0.0);

    let sub_client = ListenerClient::new();
    let start = Instant::now(); // Start the timer
    let response = sub_client.request_transform(
        &"base_link_inertia".to_string(),
        &"wrist_3_link".to_string(),
        0.0,
    );
    let duration = start.elapsed(); // Calculate elapsed time
    if let Ok(response) = response {
        println!("Response: {:?}", response);
        println!("Time taken: {:?}", duration);
    };
}
