use iceoryx2::port::client::Client;
use iceoryx2::port::listener::Listener;
use iceoryx2::port::notifier::Notifier;
use iceoryx2::port::publisher::Publisher;
use iceoryx2::prelude::*;
use nalgebra::{Translation3, UnitQuaternion};
use schiebung_commons::{NewTransform, TransformRequest, TransformResponse, TransformType};
use schiebung_server::config::get_config;
use schiebung_server::types::PubSubEvent;

pub struct ListenerClient {
    client: Client<ipc::Service, TransformRequest, (), TransformResponse, ()>,
}

impl ListenerClient {
    pub fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let node = NodeBuilder::new().create::<ipc::Service>()?;
        let service = node
            .service_builder(&"tf_request".try_into()?)
            .request_response::<TransformRequest, TransformResponse>()
            .open_or_create()?;
        let client = service.client_builder().create()?;

        Ok(Self { client })
    }

    pub fn request_transform(
        &self,
        from: &String,
        to: &String,
        time: f64,
    ) -> Result<TransformResponse, Box<dyn std::error::Error>> {
        // Prepare request
        let request = self.client.loan_uninit()?;
        let mut from_array: [char; 100] = ['\0'; 100];
        let mut to_array: [char; 100] = ['\0'; 100];

        for (i, c) in from.chars().enumerate() {
            if i < 100 {
                from_array[i] = c;
            } else {
                break;
            }
        }
        for (i, c) in to.chars().enumerate() {
            if i < 100 {
                to_array[i] = c;
            } else {
                break;
            }
        }

        let request = request.write_payload(TransformRequest {
            from: from_array,
            to: to_array,
            time,
        });

        // Send request and get pending response
        let pending_response = request.send()?;

        // Wait for response (blocking)
        loop {
            if let Some(response) = pending_response.receive()? {
                return Ok(response.payload().clone());
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
}

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

pub struct PublisherClient {
    tf_publisher: Publisher<ipc::Service, NewTransform, ()>,
    tf_publisher_notifier: Notifier<ipc::Service>,
    receiver_event: Listener<ipc::Service>,
}

impl PublisherClient {
    pub fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let config = get_config()?;
        let node = NodeBuilder::new().create::<ipc::Service>()?;
        let publish_service = node
            .service_builder(&"new_tf".try_into()?)
            .publish_subscribe::<NewTransform>()
            .max_publishers(config.max_subscribers)
            .max_subscribers(config.max_subscribers)
            .open_or_create()?;
        let publisher = publish_service.publisher_builder().create()?;

        let event_service = node
            .service_builder(&"new_tf".try_into().unwrap())
            .event()
            .max_listeners(config.max_subscribers)
            .open_or_create()?;
        let publish_service_notifier = event_service.notifier_builder().create()?;
        let event_listener = event_service.listener_builder().create()?;

        Ok(Self {
            tf_publisher: publisher,
            receiver_event: event_listener,
            tf_publisher_notifier: publish_service_notifier,
        })
    }

    pub fn send_transform(
        &self,
        from: &String,
        to: &String,
        translation: Translation3<f64>,
        rotation: UnitQuaternion<f64>,
        stamp: f64,
        kind: TransformType,
    ) {
        let new_tf = NewTransform {
            from: encode_char_array(from),
            to: encode_char_array(to),
            time: stamp,
            translation: [translation.x, translation.y, translation.z],
            rotation: [rotation.i, rotation.j, rotation.k, rotation.w],
            kind: kind as u8,
        };
        let sample = self.tf_publisher.loan_uninit().unwrap();
        let sample = sample.write_payload(new_tf);
        self.tf_publisher_notifier
            .notify_with_custom_event_id(PubSubEvent::SentSample.into())
            .unwrap();
        sample.send().unwrap();
        while let Some(event) = self.receiver_event.blocking_wait_one().unwrap() {
            let event: PubSubEvent = event.into();
            match event {
                PubSubEvent::ReceivedSample => return,
                _ => (),
            }
        }
    }
}

impl Drop for PublisherClient {
    fn drop(&mut self) {
        self.tf_publisher_notifier
            .notify_with_custom_event_id(PubSubEvent::SubscriberDisconnected.into())
            .unwrap();
    }
}

pub struct VisualizerClient {
    visualizer_event: Notifier<ipc::Service>,
}

impl VisualizerClient {
    pub fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let node = NodeBuilder::new().create::<ipc::Service>()?;

        let event_service = node
            .service_builder(&"visualizer".try_into()?)
            .event()
            .open_or_create()?;
        let visualizer_event = event_service.notifier_builder().create()?;

        Ok(Self {
            visualizer_event: visualizer_event,
        })
    }
    pub fn send_visualization_request(&self) {
        self.visualizer_event
            .notify_with_custom_event_id(PubSubEvent::SentSample.into())
            .unwrap();
    }
}

impl Drop for VisualizerClient {
    fn drop(&mut self) {
        self.visualizer_event
            .notify_with_custom_event_id(PubSubEvent::SubscriberDisconnected.into())
            .unwrap();
    }
}
