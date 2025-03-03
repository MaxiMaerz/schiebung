use schiebung_client::{PublisherClient, ListenerClient};
use nalgebra::{Translation3, UnitQuaternion, Quaternion};

fn main() {
    env_logger::init();
    let pub_client = PublisherClient::new();
    let translation = Translation3::new(0.0, 0.0, 1.0);
    let rotation = UnitQuaternion::new_normalize(Quaternion::new(0.0, 0.0, 0.0, 1.0));
    pub_client.send_transform(&"foo".to_string(), &"bar".to_string(), translation, rotation, 0.0);

    let sub_client = ListenerClient::new();
    let response = sub_client.request_transform(&"foo".to_string(), &"bar".to_string(), 0.0);
    if let Ok(response) = response {
        println!("Response: {:?}", response);
    };
}