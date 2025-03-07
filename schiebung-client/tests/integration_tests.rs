use iceoryx2::prelude::*;
use schiebung_server::Server;
use log::{info, error};
use schiebung_client::{ListenerClient, PublisherClient};
use schiebung_types::{StampedIsometry, StampedTransform};
use std::{thread, time::Duration};


#[test]
pub fn test_interaction() {
    env_logger::Builder::new().filter(None, log::LevelFilter::Debug).init();
    let _server_handle = thread::spawn(|| {
            let server = Server::new().unwrap();

            let waitset = WaitSetBuilder::new().create::<ipc::Service>().unwrap();
            let request_listener_guard = waitset.attach_notification(&server.request_listener_notifier).unwrap();
            let transform_listener_guard =
                waitset.attach_notification(&server.transform_listener_event_listener).unwrap();
            let visualizer_event_guard = 
                waitset.attach_notification(&server.visualizer_listener).unwrap();

            let fn_call = |attachment_id: WaitSetAttachmentId<ipc::Service>| {
                if attachment_id.has_event_from(&request_listener_guard) {
                    server.handle_listener_event().unwrap();
                } else if attachment_id.has_event_from(&transform_listener_guard) {
                    server.handle_transform_listener_event().unwrap();
                } else if attachment_id.has_event_from(&visualizer_event_guard) {
                    server.handle_visualizer_event().unwrap();
                }
                CallbackProgression::Continue
            };
            waitset.wait_and_process(fn_call).unwrap();
        });

    std::thread::sleep(Duration::from_secs(1));
    let sub_client = ListenerClient::new().unwrap();

    let response = sub_client.request_transform(
        &"base_link_inertia".to_string(),
        &"wrist_3_link".to_string(),
        0.0,
    );
    match response {
        Ok(_response) => {
            assert!(false)
        }
        _ => assert!(true)
    }

    // Add a few Tfs
    sub_client.



}