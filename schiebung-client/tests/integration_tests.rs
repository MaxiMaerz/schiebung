use iceoryx2::prelude::*;
use nalgebra::{Quaternion, Translation3, UnitQuaternion};
use schiebung_server::Server;
use log::{info, error};
use schiebung_client::{ListenerClient, PublisherClient};
use schiebung_types::TransformType;
use std::{thread, time::Duration};
use std::sync::{Arc, Barrier};
mod common;
const TIMEOUT: Duration = Duration::from_secs(3);


#[test]
/// This test checks if a single client can receive a transform
/// Also checks if errors are handled correctly
pub fn test_basic_interaction() {
    common::setup_logger();
    let server_handle = thread::spawn(|| {
            let server = Server::new().unwrap();

            let waitset = WaitSetBuilder::new().create::<ipc::Service>().unwrap();
            let request_listener_guard = waitset.attach_notification(&server.request_listener_notifier).unwrap();
            let transform_listener_guard =
                waitset.attach_notification(&server.transform_listener_event_listener).unwrap();
            let visualizer_event_guard = 
                waitset.attach_notification(&server.visualizer_listener).unwrap();

            let timeout_guard = waitset.attach_interval(TIMEOUT).unwrap();

            let fn_call = |attachment_id: WaitSetAttachmentId<ipc::Service>| {
                if attachment_id.has_event_from(&request_listener_guard) {
                    server.handle_listener_event().unwrap();
                } else if attachment_id.has_event_from(&transform_listener_guard) {
                    server.handle_transform_listener_event().unwrap();
                } else if attachment_id.has_event_from(&visualizer_event_guard) {
                    server.handle_visualizer_event().unwrap();
                } else if attachment_id.has_event_from(&timeout_guard) {
                    info!("Timeout");
                    return CallbackProgression::Stop
                }
                CallbackProgression::Continue
            };
            waitset.wait_and_process(fn_call).unwrap();
            info!("Server shutting down");
        });

    std::thread::sleep(Duration::from_secs(1));

    let sub_client = ListenerClient::new().unwrap();
    let response = sub_client.request_transform(
        &"root".to_string(),
        &"child_1".to_string(),
        0.0,
    );
    match response {
        Ok(_response) => {
            assert!(false)
        }
        _ => assert!(true)
    }

    let pub_client = PublisherClient::new().unwrap();

    pub_client.send_transform(
        &"root".to_string(),
        &"child_1".to_string(),
        Translation3::new(1.0, 2.0, 3.0),
        UnitQuaternion::new_normalize(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
        1.0,
        TransformType::Static,
    );

    let response = sub_client.request_transform(
        &"root".to_string(),
        &"child_1".to_string(),
        1.0,
    );
    info!("Response: {:?}", response);

    match response {
        Ok(response) => {
            assert_eq!(response.translation, [1.0, 2.0, 3.0]);
            assert_eq!(response.rotation, [0.0, 0.0, 0.0, 1.0]);
        }
        _ => assert!(false)
    }
    server_handle.join().unwrap();
}

#[test]
/// This test checks if multiple clients can receive their requested transforms
/// We create 3 clients:
/// One is just getting one TF and will idle afterwards
/// The other two are sending multiple requests after waiting at a barrier and checking if they receive the correct TF
fn test_multi_client_interaction() {
    common::setup_logger();
    let barrier = Arc::new(Barrier::new(3)); // Create barrier for 3 threads (main + 2 clients)
    let barrier_clone1 = barrier.clone();
    let barrier_clone2 = barrier.clone();

    let server_handle = thread::spawn(|| {
            let server = Server::new().unwrap();

            let waitset = WaitSetBuilder::new().create::<ipc::Service>().unwrap();
            let request_listener_guard = waitset.attach_notification(&server.request_listener_notifier).unwrap();
            let transform_listener_guard =
                waitset.attach_notification(&server.transform_listener_event_listener).unwrap();
            let visualizer_event_guard = 
                waitset.attach_notification(&server.visualizer_listener).unwrap();

            let timeout_guard = waitset.attach_interval(TIMEOUT).unwrap();

            let fn_call = |attachment_id: WaitSetAttachmentId<ipc::Service>| {
                if attachment_id.has_event_from(&request_listener_guard) {
                    server.handle_listener_event().unwrap();
                } else if attachment_id.has_event_from(&transform_listener_guard) {
                    server.handle_transform_listener_event().unwrap();
                } else if attachment_id.has_event_from(&visualizer_event_guard) {
                    server.handle_visualizer_event().unwrap();
                } else if attachment_id.has_event_from(&timeout_guard) {
                    info!("Timeout");
                    return CallbackProgression::Stop
                }
                CallbackProgression::Continue
            };
            waitset.wait_and_process(fn_call).unwrap();
            info!("Server shutting down");
        });

    std::thread::sleep(Duration::from_secs(1));
    let pub_client = PublisherClient::new().unwrap();
    pub_client.send_transform(
        &"root".to_string(),
        &"child_1".to_string(),
        Translation3::new(1.0, 2.0, 3.0),
        UnitQuaternion::new_normalize(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
        1.0,
        TransformType::Static,
    );
    pub_client.send_transform(
        &"root".to_string(),
        &"child_2".to_string(),
        Translation3::new(1.0, 2.0, 1.0),
        UnitQuaternion::new_normalize(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
        1.0,
        TransformType::Static,
    );
    // Wait for the server to process the transforms
    let sync_sub_client = ListenerClient::new().unwrap();
    // Check if the transforms are available
    let response = sync_sub_client.request_transform(
        &"root".to_string(),
        &"child_1".to_string(),
        1.0,
    );
    match response {
        Ok(response) => {
            assert_eq!(response.translation, [1.0, 2.0, 3.0]);
            assert_eq!(response.rotation, [0.0, 0.0, 0.0, 1.0]);
        }
        _ => assert!(false)
    }
    let response = sync_sub_client.request_transform(
        &"root".to_string(),
        &"child_2".to_string(),
        1.0,
    );
    match response {
        Ok(response) => {
            assert_eq!(response.translation, [1.0, 2.0, 1.0]);
            assert_eq!(response.rotation, [0.0, 0.0, 0.0, 1.0]);
        }
        _ => assert!(false)
    }
    info!("Server and clients ready");

    // Test if multiple clients can receive their requested transforms
    let client_1_handle = thread::spawn(move || {
        let sub_client = ListenerClient::new().unwrap();
        barrier_clone1.wait(); // Wait for all threads to be ready
        for _ in 0..100 {
            let response = sub_client.request_transform(
                &"root".to_string(),
                &"child_1".to_string(),
                1.0,
            );
            match response {
                Ok(response) => {
                    assert_eq!(response.translation, [1.0, 2.0, 3.0]);
                    assert_eq!(response.rotation, [0.0, 0.0, 0.0, 1.0]);
                }
                _ => assert!(false)
            }
        }
        info!("Client 1 finished");
    });

    info!("Start Client 2");
    let client_2_handle = thread::spawn(move || {
        let sub_client = ListenerClient::new().unwrap();
        barrier_clone2.wait(); // Wait for all threads to be ready
        for _ in 0..100 {
            let response = sub_client.request_transform(
                &"root".to_string(),
                &"child_2".to_string(),
                1.0,
            );
            match response {
                Ok(response) => {
                    assert_eq!(response.translation, [1.0, 2.0, 1.0]);
                    assert_eq!(response.rotation, [0.0, 0.0, 0.0, 1.0]);
                }
                _ => assert!(false)
            }
        }
        info!("Client 2 finished");
    });

    barrier.wait(); // Main thread waits for clients to be ready
    server_handle.join().unwrap();
    client_1_handle.join().unwrap();
    client_2_handle.join().unwrap();
    // Both client threads will now start their loops simultaneously
}
