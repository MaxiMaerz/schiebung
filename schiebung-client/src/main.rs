use log::{error, info};
use schiebung_client::ListenerClient;
use schiebung_types::{StampedIsometry, StampedTransform};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let sub_client = ListenerClient::new()?;

    let response = sub_client.request_transform(
        &"base_link_inertia".to_string(),
        &"wrist_3_link".to_string(),
        0.0,
    );
    match response {
        Ok(response) => {
            let res = response.clone();
            info!("Raw response: {:?}", res);
            let stamped_tf: StampedTransform = res.clone().into();
            let stamped_iso: StampedIsometry = res.clone().into();
            info!("Isometry: {:?}", stamped_iso);
            info!("TF: {:?}", stamped_tf);
        }
        _ => error!("Lookup error!"),
    }
    Ok(())
}
