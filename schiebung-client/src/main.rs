use schiebung_client::ListenerClient;
use std::time::Instant;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let sub_client = ListenerClient::new()?;

    for _ in 0..20 {
        let start = Instant::now();
        let response = sub_client.request_transform(
            &"base_link_inertia".to_string(),
            &"wrist_3_link".to_string(),
            0.0,
        );
        let duration = start.elapsed();
        if let Ok(response) = response {
            println!("Response: {:?}", response);
            println!("Time taken: {:?}", duration);
        }
    }
    Ok(())
}
