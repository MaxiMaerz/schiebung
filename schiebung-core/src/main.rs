
pub mod lib;

use iceoryx2::prelude::*;
use iceoryx2::port::subscriber::Subscriber;
use core::time::Duration;
use schiebung_types::TransformRequest;


const CYCLE_TIME: Duration = Duration::from_secs(1);

struct Server {
    buffer: lib::BufferTree,
    node: Node<ipc::Service>,
    listener: Subscriber<ipc::Service, TransformRequest, ()>,
}

impl Server {
    fn new() -> Server {
        let buffer = lib::BufferTree::new();
        let node = NodeBuilder::new().create::<ipc::Service>().unwrap();

        // create our port factory by creating or opening the service
        let service = node.service_builder(&"tf_request".try_into().unwrap())
            .publish_subscribe::<TransformRequest>()
            .open_or_create()
            .unwrap();  
        let subscriber = service.subscriber_builder().create().unwrap();

        Server {
            buffer: buffer,
            node: node,
            listener: subscriber,
        }
    }

    pub fn spin(&mut self) {
        while self.node.wait(CYCLE_TIME).is_ok() {
            while let Some(sample) = self.listener.receive().unwrap() {
                println!("received: {:?}", *sample);
            }
        }  
    }

    fn add_transform(&mut self, from: String, to: String, transform: lib::StampedIsometry, kind: lib::TransformType) {
        self.buffer.update(from, to, transform, kind);
    }

    fn lookup_transform(&mut self, from: String, to: String, time: f64) -> Option<lib::StampedIsometry> {
        self.buffer.lookup_transform(from, to, time)
    }
}

fn main() {
    let mut server = Server::new();
    server.spin();
}