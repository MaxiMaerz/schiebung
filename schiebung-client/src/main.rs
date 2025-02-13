use core::time::Duration;
use iceoryx2::prelude::*;
use schiebung_types::TransformRequest;

const CYCLE_TIME: Duration = Duration::from_secs(1);

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let node = NodeBuilder::new().create::<ipc::Service>()?;

    let service = node
        .service_builder(&"tf_request".try_into()?)
        .publish_subscribe::<TransformRequest>()
        .open_or_create()?;

    let publisher = service.publisher_builder().create()?;

    let mut counter: u64 = 0;

    while node.wait(CYCLE_TIME).is_ok() {
        counter += 1;
        let sample = publisher.loan_uninit()?;

        let mut from: [char; 100] = ['\0'; 100];
        let mut to: [char; 100] = ['\0'; 100];
        let input = ['w', 'o', 'r', 'l', 'd'];
        for (i, &c) in input.iter().enumerate() {
            from[i] = c;
        }

        let sample = sample.write_payload(TransformRequest {
            from: from,
            to: to,
            time: 0.0 as f64,
            id: 1 as i32,
        });
        sample.send()?;

        println!("Send sample {} ...", counter);
    }

    println!("exit");

    Ok(())
}