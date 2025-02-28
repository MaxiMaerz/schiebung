use core::time::Duration;
use iceoryx2::port::listener::Listener;
use iceoryx2::{config::Event, prelude::*};
use iceoryx2::port::publisher::Publisher;
use iceoryx2::port::subscriber::Subscriber;
use schiebung_types::{NewTransform, PubSubEvent, TransformRequest, TransformResponse};
use nalgebra::{Isometry, Isometry3, Quaternion, Translation, Translation3, UnitQuaternion};
// use env_logger;

const CYCLE_TIME: Duration = Duration::from_secs(1);

fn encode_char_array(input: &String) -> [char; 100] {
    let mut char_array: [char; 100] = ['\0'; 100];
    for (i, c) in input.chars().enumerate() {
        if i < 100 {
            char_array[i] = c;
        } else {
            break;
        }
    }
    char_array
}

pub struct ListenerClient {
    pub tf_listener: Subscriber<ipc::Service, TransformResponse, ()> ,
    pub tf_requester: Publisher<ipc::Service, TransformRequest, ()>,
    pub node: Node<ipc::Service>,
}

impl ListenerClient {
    fn new() -> ListenerClient {
        let node = NodeBuilder::new().create::<ipc::Service>().unwrap();

        let publish_service = node
            .service_builder(&"tf_request".try_into().unwrap())
            .publish_subscribe::<TransformRequest>()
            .open_or_create()
            .unwrap();
        let publisher = publish_service.publisher_builder().create().unwrap();

        let subscribe_service = node
            .service_builder(&"tf_replay_1".try_into().unwrap())
            .publish_subscribe::<TransformResponse>()
            .open_or_create()
            .unwrap();
        let listener = subscribe_service.subscriber_builder().create().unwrap();

        ListenerClient {
            tf_listener: listener,
            tf_requester: publisher,
            node: node,
        }

    }

    pub fn request_transform(&self, from: &String, to: &String, time: f64) {
        // First send the request
        let sample = self.tf_requester.loan_uninit().unwrap();
        let sample = sample.write_payload(TransformRequest {
            from: encode_char_array(from),
            to: encode_char_array(to),
            time: 0.0 as f64,
            id: 1 as i32,
        });
        sample.send().unwrap();

        // Now wait until we get the response
        while self.node.wait(CYCLE_TIME).is_ok() {
            while let Some(sample) = self.tf_listener.receive().unwrap() {
                println!("{:?}", sample.payload());
            }
        }
    }
}


pub struct  PublisherClient {
    pub tf_publisher: Publisher<ipc::Service, NewTransform, ()>,
    pub receiver_event: Listener<ipc::Service>,
    pub node: Node<ipc::Service>,
}

impl PublisherClient {
    fn new() -> PublisherClient {
        let node = NodeBuilder::new().create::<ipc::Service>().unwrap();
        let publish_service = node
            .service_builder(&"new_tf".try_into().unwrap())
            .publish_subscribe::<NewTransform>()
            .open_or_create()
            .unwrap();
        let publisher = publish_service.publisher_builder().create().unwrap();

        let event_service = node
            .service_builder(&"new_tf".try_into().unwrap())
            .event()
            .open_or_create()
            .unwrap();
        let event_listener = event_service.listener_builder().create().unwrap();

        PublisherClient {
            tf_publisher: publisher,
            receiver_event: event_listener,
            node: node,
        }


    }

    fn send_transform(&self, from: &String, to: &String, translation: Translation3<f64>, rotation: UnitQuaternion<f64>, stamp: f64) {
        let new_tf = NewTransform{
            from: encode_char_array(from),
            to: encode_char_array(to),
            time: stamp,
            translation: [translation.x, translation.y, translation.z],
            rotation: [rotation.i, rotation.j, rotation.k, rotation.w],
        };
        let sample = self.tf_publisher.loan_uninit().unwrap();
        let sample = sample.write_payload(new_tf);
        sample.send().unwrap();
        while self.node.wait(CYCLE_TIME).is_ok() {
            if let Ok(Some(event)) = self.receiver_event.try_wait_one() {
                let event: PubSubEvent = event.into();
                match event {
                    PubSubEvent::ReceivedSample => return,
                    _ => (),
                }
            }
        }
    }
}


fn main() {
    env_logger::init();
    let pub_client = PublisherClient::new();
    let translation = Translation3::new(0.0, 0.0, 1.0);
    let rotation = UnitQuaternion::new_normalize(Quaternion::new(0.0, 0.0, 0.0, 1.0));
    pub_client.send_transform(&"foo".to_string(), &"bar".to_string(), translation, rotation, 0.0);
    println!("SENT!!")
}