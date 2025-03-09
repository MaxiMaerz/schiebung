use iceoryx2::port::listener::Listener;
use iceoryx2::port::notifier::Notifier;
use iceoryx2::port::publisher::Publisher;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use log::{debug, error, info};
use nalgebra::{Isometry, Quaternion, Translation3, UnitQuaternion};
use schiebung_core::BufferTree;
use schiebung_types::{
    NewTransform, PubSubEvent, StampedIsometry, TransformRequest, TransformResponse, TransformType,
};
use std::{collections::HashMap, sync::{Arc, Mutex}};

fn decode_char_array(arr: &[char; 100]) -> String {
    arr.iter().take_while(|&&c| c != '\0').collect()
}

struct TFPublisher {
    buffer: Arc<Mutex<BufferTree>>,
    publisher: Publisher<ipc::Service, TransformResponse, ()>,
    notifier: Notifier<ipc::Service>,
    id: u128,
}

impl TFPublisher {
    pub fn new(buffer: Arc<Mutex<BufferTree>>, id: u128) -> Result<Self, Box<dyn std::error::Error>> {
        let node = NodeBuilder::new().create::<ipc::Service>()?;
        let service_name = ServiceName::new(&("tf_replay_".to_string() + &id.to_string()))?;
        let publisher_service = node
            .service_builder(&service_name)
            .publish_subscribe::<TransformResponse>()
            .open_or_create()?;
        let notifier_service = node
            .service_builder(&service_name)
            .event()
            .open_or_create()?;
        let publisher = publisher_service.publisher_builder().create()?;
        let notifier = notifier_service.notifier_builder().create()?;
        Ok(Self {
            buffer: buffer,
            publisher: publisher,
            notifier: notifier,
            id: id,
        })
    }

    pub fn publish(&self, tf_request: &TransformRequest) -> Result<(), Box<dyn std::error::Error>> {
        let target_isometry = self.buffer.lock().unwrap().lookup_latest_transform(
            decode_char_array(&tf_request.from),
            decode_char_array(&tf_request.to),
        );
        match target_isometry {
            Ok(target_isometry) => {
                let sample = self.publisher.loan_uninit().unwrap();
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
                self.notifier
                    .notify_with_custom_event_id(PubSubEvent::SentSample.into())
                    .unwrap();
                info!(
                    "Published transform from: {} to {}:",
                    decode_char_array(&tf_request.from),
                    decode_char_array(&tf_request.to)
                );
            }
            Err(e) => {
                error!(
                    "No transform from {} to {} err: {:?}",
                    decode_char_array(&tf_request.from),
                    decode_char_array(&tf_request.to),
                    e
                );
                self.notifier
                    .notify_with_custom_event_id(PubSubEvent::Error.into())
                    .unwrap();
            }
        }
        Ok(())
    }
}

pub struct Server {
    pub request_listener: Subscriber<ipc::Service, TransformRequest, ()>,
    pub request_listener_notifier: Listener<ipc::Service>,
    pub transform_listener: Subscriber<ipc::Service, NewTransform, ()>,
    pub transform_listener_event_listener: Listener<ipc::Service>,
    pub transform_listener_notifier: Notifier<ipc::Service>,
    pub visualizer_listener: Listener<ipc::Service>,
    buffer: Arc<Mutex<BufferTree>>,
    active_publishers: Arc<Mutex<HashMap<u128, TFPublisher>>>,
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
            .max_publishers(10)
            .max_subscribers(10)
            .open_or_create()?;
        let subscriber = service.subscriber_builder().create()?;
        let event_service = node
            .service_builder(&listener_name)
            .event()
            .open_or_create()?;
        let request_listener_notifier = event_service.listener_builder().create()?;

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

        // Visualizer
        let visualizer_event_service = node
            .service_builder(&"visualizer".try_into()?)
            .event()
            .open_or_create()?;
        let visualizer_listener = visualizer_event_service.listener_builder().create()?;

        Ok(Self {
            buffer: buffer,
            request_listener: subscriber,
            request_listener_notifier: request_listener_notifier,
            transform_listener: transform_listener,
            transform_listener_event_listener: transform_listener_notifier,
            transform_listener_notifier: notifier,
            visualizer_listener: visualizer_listener,
            active_publishers: Arc::new(Mutex::new(HashMap::new())),
        })
    }

    pub fn handle_listener_event(&self) -> Result<(), Box<dyn std::error::Error>> {
        while let Some(event) = self.request_listener_notifier.try_wait_one()? {
            let event: PubSubEvent = event.into();
            match event {
                PubSubEvent::SentSample => self.process_listener_request()?,
                _ => (),
            }
        }

        Ok(())
    }

    fn process_listener_request(&self) -> Result<(), Box<dyn std::error::Error>> {
        match self.request_listener.receive()? {
            Some(sample) => {
                debug!("Received listener request: {:?}", sample);
                let tf_request = sample.payload().clone();
                let mut active_publishers = self.active_publishers.lock().unwrap();
                if !active_publishers.contains_key(&tf_request.id) {
                    let publisher = TFPublisher::new(self.buffer.clone(), tf_request.id)?;
                    active_publishers.insert(tf_request.id, publisher);
                }
                active_publishers.get(&tf_request.id).unwrap().publish(&tf_request)?;
            }
            None => (),
        }
        Ok(())
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

    pub fn handle_visualizer_event(&self) -> Result<(), Box<dyn std::error::Error>> {
        while let Some(event) = self.visualizer_listener.try_wait_one()? {
            let event: PubSubEvent = event.into();
            match event {
                PubSubEvent::SentSample => self.buffer.lock().unwrap().save_visualization()?,
                _ => (),
            }
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
