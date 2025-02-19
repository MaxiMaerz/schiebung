pub mod lib;

use core::time::Duration;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use iceoryx2::{port::publisher::Publisher, sample};
use schiebung_types::{PubSubEvent, TransformRequest, TransformResponse};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
const CYCLE_TIME: Duration = Duration::from_secs(1);
use std::thread;
struct TFPublisher {
    buffer: Arc<Mutex<lib::BufferTree>>,
    from: String,
    to: String,
    sub_id: i32,
}

fn decode_char_array(arr: &[char; 100]) -> String {
    arr.iter().take_while(|&&c| c != '\0').collect()
}

impl TFPublisher {
    fn new(
        buffer: Arc<Mutex<lib::BufferTree>>,
        sub_id: i32,
        from: String,
        to: String,
    ) -> TFPublisher {
        TFPublisher {
            buffer: buffer,
            from: from,
            to: to,
            sub_id: sub_id,
        };
    }
    fn publish_until_disconect(&mut self) {
        let service_name: ServiceName =
            ServiceName::new(&("tf_replay_".to_owned() + &self.sub_id.clone().to_string()))
                .unwrap();

        let node = NodeBuilder::new().create::<ipc::Service>().unwrap();      
        let service = node
            .service_builder(&service_name)
            .publish_subscribe::<TransformResponse>()
            .open_or_create()
            .unwrap();
        let publisher = service.publisher_builder().create().unwrap();

        let event_service = node
            .service_builder(&service_name)
            .event()
            .open_or_create()
            .unwrap();
        let event_listener = event_service.listener_builder().create().unwrap();

        while node.wait(CYCLE_TIME).is_ok() {
            if let Ok(Some(event)) = event_listener.try_wait_one() {
                let event: PubSubEvent = event.into();
                match event {
                    PubSubEvent::SubscriberDisconnected | PubSubEvent::ProcessDied => return,
                    _ => (),
                }
            }
            let sample = publisher.loan_uninit().unwrap();
            let target_isometry = self
                .buffer
                .lock()
                .unwrap()
                .lookup_latest_transform(self.from.clone(), self.to.clone())
                .unwrap();
            let hom_tf = target_isometry.isometry.to_homogeneous();
            let mut array = [[0.0; 4]; 4];

            for i in 0..4 {
                for j in 0..4 {
                    array[i][j] = hom_tf[(i, j)];
                }
            }

            let sample = sample.write_payload(TransformResponse {
                id: self.sub_id,
                time: target_isometry.stamp,
                isometry: array,
            });
        }
    }

    pub fn start(&mut self) {
        thread::spawn(move || {self.publish_until_disconect();});
    }
}

struct Server {
    buffer: Arc<Mutex<lib::BufferTree>>,
    node: Node<ipc::Service>,
    listener: Subscriber<ipc::Service, TransformRequest, ()>,
    active_publishers: HashMap<i32, TFPublisher>,
}

impl Server {
    fn new() -> Server {
        let buffer = Arc::new(Mutex::new(lib::BufferTree::new()));
        let node = NodeBuilder::new().create::<ipc::Service>().unwrap();

        // create our port factory by creating or opening the service
        let service = node
            .service_builder(&"tf_request".try_into().unwrap())
            .publish_subscribe::<TransformRequest>()
            .open_or_create()
            .unwrap();
        let subscriber = service.subscriber_builder().create().unwrap();

        Server {
            buffer: buffer,
            node: node,
            listener: subscriber,
            active_publishers: HashMap::new(),
        }
    }

    pub fn spin(&mut self) {
        while self.node.wait(CYCLE_TIME).is_ok() {
            while let Some(sample) = self.listener.receive().unwrap() {
                let tf_request = sample.payload();


                self.active_publishers.insert(
                    tf_request.id,
                    TFPublisher::new(
                        self.buffer.clone(),
                        tf_request.id,
                        decode_char_array(&tf_request.from),
                        decode_char_array(&tf_request.to),
                    ),
                );
            }
        }
    }

    fn add_transform(
        &mut self,
        from: String,
        to: String,
        transform: lib::StampedIsometry,
        kind: lib::TransformType,
    ) {
        self.buffer.lock().unwrap().update(from, to, transform, kind);
    }

    fn lookup_transform(
        &mut self,
        from: String,
        to: String,
        time: f64,
    ) -> Option<lib::StampedIsometry> {
        self.buffer.lock().unwrap().lookup_transform(from, to, time)
    }
}

fn main() {
    let mut server = Server::new();
    server.spin();
}
