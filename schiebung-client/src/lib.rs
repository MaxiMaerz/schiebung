use iceoryx2::port::listener::Listener;
use iceoryx2::port::notifier::Notifier;
use iceoryx2::port::publisher::Publisher;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use nalgebra::{Translation3, UnitQuaternion};
use schiebung_types::{
    NewTransform, PubSubEvent, TransformRequest, TransformResponse, TransformType,
};

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
    tf_listener: Subscriber<ipc::Service, TransformResponse, ()>,
    tf_requester: Publisher<ipc::Service, TransformRequest, ()>,
    tf_requester_notifier: Notifier<ipc::Service>,
    tf_listener_event_listener: Listener<ipc::Service>,
    id: u128,
}

impl ListenerClient {
    pub fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let node = NodeBuilder::new().create::<ipc::Service>()?;

        let listener_name = &"tf_request".try_into()?;
        let publish_service = node
            .service_builder(listener_name)
            .publish_subscribe::<TransformRequest>()
            .open_or_create()?;
        let publisher = publish_service.publisher_builder().create()?;
        let publish_service_notifier = node
            .service_builder(listener_name)
            .event()
            .open_or_create()?;
        let publish_service_notifier = publish_service_notifier.notifier_builder().create()?;

        let sub_id = &"tf_response".try_into()?;
        let subscribe_service = node
            .service_builder(sub_id)
            .publish_subscribe::<TransformResponse>()
            .open_or_create()?;
        let listener = subscribe_service.subscriber_builder().create()?;
        let notifier_service = node.service_builder(sub_id).event().open_or_create()?;
        let tf_listener_event_listener = notifier_service.listener_builder().create()?;

        let id = listener.id().value();

        Ok(Self {
            tf_listener: listener,
            tf_requester: publisher,
            tf_requester_notifier: publish_service_notifier,
            tf_listener_event_listener: tf_listener_event_listener,
            id: id.clone(),
        })
    }

    pub fn request_transform(
        &self,
        from: &String,
        to: &String,
        time: f64,
    ) -> Result<TransformResponse, PubSubEvent> {
        // First send the request
        let sample = self.tf_requester.loan_uninit().unwrap();
        let sample = sample.write_payload(TransformRequest {
            from: encode_char_array(from),
            to: encode_char_array(to),
            time: time,
            id: self.tf_requester.id().value(),
        });
        sample.send().unwrap();
        self.tf_requester_notifier
            .notify_with_custom_event_id(PubSubEvent::SentSample.into())
            .unwrap();

        // Now wait until we get the response
        while let Some(event) = self.tf_listener_event_listener.blocking_wait_one().unwrap() {
            let event: PubSubEvent = event.into();
            match event {
                PubSubEvent::SentSample => {
                    let sample = self.tf_listener.receive().unwrap().unwrap();
                    if sample.id == self.id {
                        let result = Ok(sample.clone());
                        let _res = self
                            .tf_requester_notifier
                            .notify_with_custom_event_id(PubSubEvent::ReceivedSample.into());
                        return result;
                    }
                    continue;
                }
                PubSubEvent::Error => {
                    return Err(event);
                }
                _ => (),
            }
        }
        Err(PubSubEvent::Unknown)
    }
}

impl Drop for ListenerClient {
    fn drop(&mut self) {
        self.tf_requester_notifier.notify_with_custom_event_id(PubSubEvent::SubscriberDisconnected.into()).unwrap();
    }
}

pub struct PublisherClient {
    tf_publisher: Publisher<ipc::Service, NewTransform, ()>,
    tf_publisher_notifier: Notifier<ipc::Service>,
    receiver_event: Listener<ipc::Service>,
}

impl PublisherClient {
    pub fn new() -> PublisherClient {
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
        let publish_service_notifier = event_service.notifier_builder().create().unwrap();
        let event_listener = event_service.listener_builder().create().unwrap();

        PublisherClient {
            tf_publisher: publisher,
            receiver_event: event_listener,
            tf_publisher_notifier: publish_service_notifier,
        }
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
