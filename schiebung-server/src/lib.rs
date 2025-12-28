use iceoryx2::port::listener::Listener;
use iceoryx2::port::notifier::Notifier;
use iceoryx2::port::server::Server as IoxServer;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::*;
use log::{debug, error, info};
use nalgebra::{Isometry, Quaternion, Translation3, UnitQuaternion};
use std::sync::{Arc, Mutex};

use schiebung::BufferTree;
use schiebung::{types::StampedIsometry, TfError};
use schiebung_commons::{NewTransform, TransformRequest, TransformResponse, TransformType};

pub mod types;
use crate::types::PubSubEvent;
pub mod config;
use crate::config::get_config;

fn decode_char_array(arr: &[char; 100]) -> String {
    arr.iter().take_while(|&&c| c != '\0').collect()
}

pub struct Server {
    pub request_response_server:
        IoxServer<ipc::Service, TransformRequest, (), TransformResponse, ()>,
    pub transform_listener: Subscriber<ipc::Service, NewTransform, ()>,
    pub transform_listener_event_listener: Listener<ipc::Service>,
    pub transform_listener_notifier: Notifier<ipc::Service>,
    pub visualizer_listener: Listener<ipc::Service>,
    buffer: Arc<Mutex<BufferTree>>,
}

impl Server {
    pub fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let config = get_config()?;
        let buffer = Arc::new(Mutex::new(BufferTree::new()));
        let node = Arc::new(NodeBuilder::new().create::<ipc::Service>()?);

        // Create request-response server for transform requests
        let service_name = "tf_request".try_into()?;
        let service = node
            .service_builder(&service_name)
            .request_response::<TransformRequest, TransformResponse>()
            .open_or_create()?;
        let request_response_server = service.server_builder().create()?;

        // Publisher
        let publisher_name = "new_tf".try_into()?;
        let tf_service = node
            .service_builder(&publisher_name)
            .publish_subscribe::<NewTransform>()
            .max_publishers(config.max_subscribers)
            .max_subscribers(config.max_subscribers)
            .open_or_create()
            .unwrap();
        let transform_listener = tf_service.subscriber_builder().create()?;
        let event_notifier = node
            .service_builder(&publisher_name)
            .event()
            .max_listeners(config.max_subscribers)
            .open_or_create()?;
        let notifier = event_notifier.notifier_builder().create()?;
        let transform_listener_notifier = event_notifier.listener_builder().create()?;

        // Visualizer
        let visualizer_event_service = node
            .service_builder(&"visualizer".try_into()?)
            .event()
            .max_listeners(config.max_subscribers)
            .open_or_create()?;
        let visualizer_listener = visualizer_event_service.listener_builder().create()?;

        Ok(Self {
            buffer,
            request_response_server,
            transform_listener,
            transform_listener_event_listener: transform_listener_notifier,
            transform_listener_notifier: notifier,
            visualizer_listener,
        })
    }

    pub fn handle_request_event(&self) -> Result<(), Box<dyn std::error::Error>> {
        while let Some(active_request) = self.request_response_server.receive()? {
            let tf_request = active_request.payload();
            debug!("Received transform request: {:?}", tf_request);

            // Lookup the transform
            let target_isometry: Result<StampedIsometry, TfError> = if tf_request.time == 0.0 {
                let from = decode_char_array(&tf_request.from);
                let to = decode_char_array(&tf_request.to);
                self.buffer
                    .lock()
                    .unwrap()
                    .lookup_latest_transform(&from, &to)
            } else {
                let from = decode_char_array(&tf_request.from);
                let to = decode_char_array(&tf_request.to);
                self.buffer
                    .lock()
                    .unwrap()
                    .lookup_transform(&from, &to, tf_request.time)
            };

            // Send response
            match target_isometry {
                Ok(target_isometry) => {
                    let response = active_request.loan_uninit()?;
                    let response = response.write_payload(TransformResponse {
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
                    response.send()?;
                    info!(
                        "Sent transform response from {} to {}",
                        decode_char_array(&tf_request.from),
                        decode_char_array(&tf_request.to)
                    );
                }
                Err(e) => {
                    error!(
                        "Transform lookup failed from {} to {}: {:?}",
                        decode_char_array(&tf_request.from),
                        decode_char_array(&tf_request.to),
                        e
                    );
                    // Drop the request without sending a response (or we could send an error response)
                }
            }
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
            let from = decode_char_array(&new_tf.from);
            let to = decode_char_array(&new_tf.to);
            let result = self.buffer.lock().unwrap().update(
                &from,
                &to,
                iso,
                TransformType::try_from(new_tf.kind).unwrap(),
            );
            if result.is_err() {
                error!("Error updating transform: {:?}", result.err().unwrap());
            }
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
