pub mod lib;

use core::time::Duration;
use iceoryx2::port::listener::Listener;
use iceoryx2::port::publisher::Publisher;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use lib::StampedIsometry;
use nalgebra::{Isometry, Isometry3, Quaternion, Translation, Translation3, UnitQuaternion};
use schiebung_types::{NewTransform, PubSubEvent, TransformRequest, TransformResponse};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
const CYCLE_TIME: Duration = Duration::from_secs(1);
use log::{debug, error, info};
use std::thread;
use env_logger;

fn decode_char_array(arr: &[char; 100]) -> String {
    arr.iter().take_while(|&&c| c != '\0').collect()
}

struct TFPublisher {
    buffer: Arc<Mutex<lib::BufferTree>>,
    from: String,
    to: String,
    sub_id: i32,
    publisher: Publisher<ipc::Service, TransformResponse, ()>,
    event_listener: Listener<ipc::Service>,
}
impl TFPublisher {
    fn new(
        buffer: Arc<Mutex<lib::BufferTree>>,
        sub_id: i32,
        from: String,
        to: String,
        node: Arc<Node<ipc::Service>>,
    ) -> TFPublisher {
        let service_name: ServiceName =
            ServiceName::new(&("tf_replay_".to_owned() + &sub_id.clone().to_string())).unwrap();
        let publisher_service = node
            .service_builder(&service_name)
            .publish_subscribe::<TransformResponse>()
            .open_or_create()
            .unwrap();
        let publisher = publisher_service.publisher_builder().create().unwrap();

        let event_service = node
            .service_builder(&service_name)
            .event()
            .open_or_create()
            .unwrap();
        let event_listener = event_service.listener_builder().create().unwrap();

        TFPublisher {
            buffer: buffer,
            from: from,
            to: to,
            sub_id: sub_id,
            publisher: publisher,
            event_listener: event_listener,
        }
    }
    fn publish(&self) -> Result<(), PubSubEvent> {
        if let Ok(Some(event)) = self.event_listener.try_wait_one() {
            let event: PubSubEvent = event.into();
            match event {
                PubSubEvent::SubscriberDisconnected | PubSubEvent::ProcessDied => {
                    return Err(PubSubEvent::SubscriberDisconnected)
                }
                _ => (),
            }
        }
        let sample = self.publisher.loan_uninit().unwrap();
        let target_isometry = self.buffer.lock().unwrap().lookup_latest_transform(self.from.clone(), self.to.clone());
        match target_isometry {
            Some(target_isometry) => {
                sample.write_payload(TransformResponse {
                    id: self.sub_id,
                    time: target_isometry.stamp,
                    translation: [
                        target_isometry.isometry.translation.x,
                        target_isometry.isometry.translation.y,
                        target_isometry.isometry.translation.z,
                    ],
                    rotation: [
                        target_isometry.isometry.rotation.i,
                        target_isometry.isometry.rotation.j,
                        target_isometry.isometry.rotation.k,
                        target_isometry.isometry.rotation.w,
                    ],
                });
            },
            None => error!("No transform from {} to {}", self.from, self.to)
        }
        Ok(())
    }
}

struct Server {
    buffer: Arc<Mutex<lib::BufferTree>>,
    node: Arc<Node<ipc::Service>>,
    request_listener: Subscriber<ipc::Service, TransformRequest, ()>,
    transform_listener: Subscriber<ipc::Service, NewTransform, ()>,
    active_publishers: HashMap<i32, TFPublisher>,
}

impl Server {
    fn new() -> Server {
        let buffer = Arc::new(Mutex::new(lib::BufferTree::new()));
        let node = Arc::new(NodeBuilder::new().create::<ipc::Service>().unwrap());

        let service = node
            .service_builder(&"tf_request".try_into().unwrap())
            .publish_subscribe::<TransformRequest>()
            .open_or_create()
            .unwrap();
        let subscriber = service.subscriber_builder().create().unwrap();

        let tf_service = node
            .service_builder(&"new_tf".try_into().unwrap())
            .publish_subscribe::<NewTransform>()
            .open_or_create()
            .unwrap();
        let transform_listener = tf_service.subscriber_builder().create().unwrap();

        Server {
            buffer: buffer,
            node: node,
            request_listener: subscriber,
            transform_listener: transform_listener,
            active_publishers: HashMap::new(),
        }
    }

    pub fn spin(&mut self) {
        while self.node.wait(CYCLE_TIME).is_ok() {
            while let Some(sample) = self.request_listener.receive().unwrap() {
                let tf_request = sample.payload();
                self.active_publishers.insert(
                    tf_request.id,
                    TFPublisher::new(
                        self.buffer.clone(),
                        tf_request.id,
                        decode_char_array(&tf_request.from),
                        decode_char_array(&tf_request.to),
                        self.node.clone(),
                    ),
                );
            };
            while let Some(sample) = self.transform_listener.receive().unwrap() {
                let new_tf = sample.payload();
                let iso = StampedIsometry {
                    isometry: Isometry::from_parts(
                        Translation3::new(
                            new_tf.translation[0],
                            new_tf.translation[1],
                            new_tf.translation[2],
                        ),
                        UnitQuaternion::new_normalize(Quaternion::new(
                            new_tf.rotation[0],
                            new_tf.rotation[1],
                            new_tf.rotation[2],
                            new_tf.rotation[3],
                        )),
                    ),
                    stamp: new_tf.time,
                };
                self.buffer.lock().unwrap().update(
                    decode_char_array(&new_tf.from),
                    decode_char_array(&new_tf.to),
                    iso,
                    lib::TransformType::Dynamic,
                );
            };

            let mut inactive_pubs: Vec<i32> = Vec::new();
            for (id, publisher) in self.active_publishers.iter() {
                match publisher.publish() {
                    Err(PubSubEvent::SubscriberDisconnected) => inactive_pubs.push(*id),
                    _ => (),
                }
            }
            for id in inactive_pubs {
                self.active_publishers.remove(&id);
            };
            self.buffer.lock().unwrap().visualize();
        }
    }
}

fn main() {

    env_logger::init();
    let mut server = Server::new();
    server.spin();
}
