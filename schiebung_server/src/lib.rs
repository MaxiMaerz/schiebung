use iceoryx2::port::listener::Listener;
use iceoryx2::port::notifier::Notifier;
use iceoryx2::port::publisher::Publisher;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use log::{error, info};
use nalgebra::{Isometry, Quaternion, Translation3, UnitQuaternion};
use schiebung_core::BufferTree;
use schiebung_types::{
    NewTransform, PubSubEvent, StampedIsometry, TransformRequest, TransformResponse, TransformType,
};
use std::sync::{Arc, Mutex};

fn decode_char_array(arr: &[char; 100]) -> String {
    arr.iter().take_while(|&&c| c != '\0').collect()
}
pub struct Server {
    buffer: Arc<Mutex<BufferTree>>,
    pub request_listener: Subscriber<ipc::Service, TransformRequest, ()>,
    pub request_listener_notifier: Listener<ipc::Service>,
    request_publisher: Publisher<ipc::Service, TransformResponse, ()>,
    request_publisher_event_notifier: Notifier<ipc::Service>,
    pub transform_listener: Subscriber<ipc::Service, NewTransform, ()>,
    pub transform_listener_notifier: Notifier<ipc::Service>,
    pub transform_listener_event_listener: Listener<ipc::Service>,
}

/// This is needed for the WaitSet to work
impl FileDescriptorBased for Server {
    fn file_descriptor(&self) -> &FileDescriptor {
        self.request_listener_notifier.file_descriptor()
    }
}

/// This is needed for the WaitSet to work
impl SynchronousMultiplexing for Server {}

impl Server {
    pub fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let buffer = Arc::new(Mutex::new(BufferTree::new()));
        let node = Arc::new(NodeBuilder::new().create::<ipc::Service>()?);

        // Listen for incoming requests
        let listener_name = "tf_request".try_into()?;
        let service = node
            .service_builder(&listener_name)
            .publish_subscribe::<TransformRequest>()
            .open_or_create()?;
        let subscriber = service.subscriber_builder().create()?;
        let event_service = node
            .service_builder(&listener_name)
            .event()
            .open_or_create()?;
        let request_listener_notifier = event_service.listener_builder().create()?;
        // publish response
        let response_name = "tf_response".try_into()?;
        let publisher_service = node
            .service_builder(&response_name)
            .publish_subscribe::<TransformResponse>()
            .open_or_create()
            .unwrap();
        let request_publisher = publisher_service.publisher_builder().create().unwrap();
        let publisher_event_service = node
            .service_builder(&response_name)
            .event()
            .open_or_create()
            .unwrap();
        let request_publisher_event_notifier =
            publisher_event_service.notifier_builder().create().unwrap();

        // Publisher
        let publisher_name = "new_tf".try_into()?;
        let tf_service = node
            .service_builder(&publisher_name)
            .publish_subscribe::<NewTransform>()
            .open_or_create()
            .unwrap();
        let transform_listener = tf_service.subscriber_builder().create()?;
        let event_notifier = node
            .service_builder(&publisher_name)
            .event()
            .open_or_create()?;
        let notifier = event_notifier.notifier_builder().create()?;
        let transform_listener_notifier = event_notifier.listener_builder().create()?;

        Ok(Self {
            buffer: buffer,
            request_listener: subscriber,
            transform_listener: transform_listener,
            request_publisher: request_publisher,
            request_publisher_event_notifier: request_publisher_event_notifier,
            transform_listener_notifier: notifier,
            transform_listener_event_listener: transform_listener_notifier,
            request_listener_notifier: request_listener_notifier,
        })
    }

    pub fn handle_listener_event(&self) -> Result<(), Box<dyn std::error::Error>> {
        while let Some(event) = self.request_listener_notifier.try_wait_one()? {
            let event: PubSubEvent = event.into();
            match event {
                PubSubEvent::SentSample => self.process_listener_request()?,
                PubSubEvent::PublisherConnected => println!("new publisher connected"),
                PubSubEvent::PublisherDisconnected => println!("publisher disconnected"),
                _ => (),
            }
        }

        Ok(())
    }

    fn process_listener_request(&self) -> Result<(), Box<dyn std::error::Error>> {
        match self.request_listener.receive()? {
            Some(sample) => {
                let tf_request = sample.payload().clone();
                self.transform_listener_notifier
                    .notify_with_custom_event_id(PubSubEvent::ReceivedSample.into())?;
                let target_isometry = self.buffer.lock().unwrap().lookup_latest_transform(
                    decode_char_array(&tf_request.from),
                    decode_char_array(&tf_request.to),
                );
                match target_isometry {
                    Some(target_isometry) => {
                        let sample = self.request_publisher.loan_uninit().unwrap();
                        let sample = sample.write_payload(TransformResponse {
                            id: tf_request.id,
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
                        sample.send().unwrap();
                        self.request_publisher_event_notifier
                            .notify_with_custom_event_id(PubSubEvent::SentSample.into())
                            .unwrap();
                        error!(
                            "Published transform from: {} to {}:",
                            decode_char_array(&tf_request.from),
                            decode_char_array(&tf_request.to)
                        );
                    }
                    None => {
                        error!(
                            "No transform from {} to {}",
                            decode_char_array(&tf_request.from),
                            decode_char_array(&tf_request.to)
                        );
                        self.request_publisher_event_notifier
                            .notify_with_custom_event_id(PubSubEvent::Error.into())
                            .unwrap();
                    }
                }
                Ok(())
            }
            None => Ok(()),
        }
    }

    pub fn handle_transform_listener_event(&self) -> Result<(), Box<dyn std::error::Error>> {
        while let Some(event) = self.transform_listener_event_listener.try_wait_one()? {
            let event: PubSubEvent = event.into();
            match event {
                PubSubEvent::SentSample => {
                    self.transform_listener_notifier
                        .notify_with_custom_event_id(PubSubEvent::ReceivedSample.into())?;
                    self.process_new_transform()?;
                }
                _ => (),
            }
        }
        Ok(())
    }

    fn process_new_transform(&self) -> Result<(), Box<dyn std::error::Error>> {
        while let Some(sample) = self.transform_listener.receive()? {
            let new_tf = sample.payload();
            info!(
                "Received transform from {} to {}",
                decode_char_array(&new_tf.from),
                decode_char_array(&new_tf.to)
            );
            let iso = StampedIsometry {
                isometry: Isometry::from_parts(
                    Translation3::new(
                        new_tf.translation[0],
                        new_tf.translation[1],
                        new_tf.translation[2],
                    ),
                    UnitQuaternion::new_normalize(Quaternion::new(
                        new_tf.rotation[3],
                        new_tf.rotation[0],
                        new_tf.rotation[1],
                        new_tf.rotation[2],
                    )),
                ),
                stamp: new_tf.time,
            };
            self.buffer.lock().unwrap().update(
                decode_char_array(&new_tf.from),
                decode_char_array(&new_tf.to),
                iso,
                TransformType::try_from(new_tf.kind).unwrap(),
            );
        }
        Ok(())
    }
}

impl Drop for Server {
    fn drop(&mut self) {
        self.transform_listener_notifier
            .notify_with_custom_event_id(PubSubEvent::SubscriberDisconnected.into())
            .unwrap();
    }
}
