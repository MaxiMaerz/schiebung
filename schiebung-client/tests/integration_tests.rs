use iceoryx2::prelude::*;
use log::info;
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use schiebung_client::{ListenerClient, PublisherClient};
use schiebung_server::Server;
use std::sync::{Arc, Barrier};
use std::{thread, time::Duration};
mod common;
const TIMEOUT: Duration = Duration::from_secs(3);
use approx::assert_relative_eq;
use schiebung_core::types::{StampedIsometry, TransformType};

#[test]
/// This test checks if a single client can receive a transform
/// Also checks if errors are handled correctly
pub fn test_basic_interaction() {
    common::setup_logger();
    let server_handle = thread::spawn(|| {
        let server = Server::new().unwrap();

        let waitset = WaitSetBuilder::new().create::<ipc::Service>().unwrap();
        let request_listener_guard = waitset
            .attach_notification(&server.request_listener_notifier)
            .unwrap();
        let transform_listener_guard = waitset
            .attach_notification(&server.transform_listener_event_listener)
            .unwrap();
        let visualizer_event_guard = waitset
            .attach_notification(&server.visualizer_listener)
            .unwrap();

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
                return CallbackProgression::Stop;
            }
            CallbackProgression::Continue
        };
        waitset.wait_and_process(fn_call).unwrap();
        info!("Server shutting down");
    });

    std::thread::sleep(Duration::from_secs(1));

    let sub_client = ListenerClient::new().unwrap();
    let response = sub_client.request_transform(&"root".to_string(), &"child_1".to_string(), 0.0);
    match response {
        Ok(_response) => {
            assert!(false)
        }
        _ => assert!(true),
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

    let response = sub_client.request_transform(&"root".to_string(), &"child_1".to_string(), 1.0);
    info!("Response: {:?}", response);

    match response {
        Ok(response) => {
            assert_eq!(response.translation, [1.0, 2.0, 3.0]);
            assert_eq!(response.rotation, [0.0, 0.0, 0.0, 1.0]);
        }
        _ => assert!(false),
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
        let request_listener_guard = waitset
            .attach_notification(&server.request_listener_notifier)
            .unwrap();
        let transform_listener_guard = waitset
            .attach_notification(&server.transform_listener_event_listener)
            .unwrap();
        let visualizer_event_guard = waitset
            .attach_notification(&server.visualizer_listener)
            .unwrap();

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
                return CallbackProgression::Stop;
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
    let response =
        sync_sub_client.request_transform(&"root".to_string(), &"child_1".to_string(), 1.0);
    match response {
        Ok(response) => {
            assert_eq!(response.translation, [1.0, 2.0, 3.0]);
            assert_eq!(response.rotation, [0.0, 0.0, 0.0, 1.0]);
        }
        _ => assert!(false),
    }
    let response =
        sync_sub_client.request_transform(&"root".to_string(), &"child_2".to_string(), 1.0);
    match response {
        Ok(response) => {
            assert_eq!(response.translation, [1.0, 2.0, 1.0]);
            assert_eq!(response.rotation, [0.0, 0.0, 0.0, 1.0]);
        }
        _ => assert!(false),
    }
    info!("Server and clients ready");

    // Test if multiple clients can receive their requested transforms
    let client_1_handle = thread::spawn(move || {
        let sub_client = ListenerClient::new().unwrap();
        barrier_clone1.wait(); // Wait for all threads to be ready
        for _ in 0..100 {
            let response =
                sub_client.request_transform(&"root".to_string(), &"child_1".to_string(), 1.0);
            match response {
                Ok(response) => {
                    assert_eq!(response.translation, [1.0, 2.0, 3.0]);
                    assert_eq!(response.rotation, [0.0, 0.0, 0.0, 1.0]);
                }
                _ => assert!(false),
            }
        }
        info!("Client 1 finished");
    });

    info!("Start Client 2");
    let client_2_handle = thread::spawn(move || {
        let sub_client = ListenerClient::new().unwrap();
        barrier_clone2.wait(); // Wait for all threads to be ready
        for _ in 0..100 {
            let response =
                sub_client.request_transform(&"root".to_string(), &"child_2".to_string(), 1.0);
            match response {
                Ok(response) => {
                    assert_eq!(response.translation, [1.0, 2.0, 1.0]);
                    assert_eq!(response.rotation, [0.0, 0.0, 0.0, 1.0]);
                }
                _ => assert!(false),
            }
        }
        info!("Client 2 finished");
    });

    barrier.wait(); // Main thread waits for clients to be ready
    server_handle.join().unwrap();
    client_1_handle.join().unwrap();
    client_2_handle.join().unwrap();
}

/// This test checks if the server can handle complex interpolation
/// Same test as in the core library, check the docu to find the code used to generate the TFs
#[test]
fn test_complex_interpolation() {
    let server_handle = thread::spawn(|| {
        let server = Server::new().unwrap();

        let waitset = WaitSetBuilder::new().create::<ipc::Service>().unwrap();
        let request_listener_guard = waitset
            .attach_notification(&server.request_listener_notifier)
            .unwrap();
        let transform_listener_guard = waitset
            .attach_notification(&server.transform_listener_event_listener)
            .unwrap();
        let visualizer_event_guard = waitset
            .attach_notification(&server.visualizer_listener)
            .unwrap();

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
                return CallbackProgression::Stop;
            }
            CallbackProgression::Continue
        };
        waitset.wait_and_process(fn_call).unwrap();
        info!("Server shutting down");
    });
    let client = PublisherClient::new().unwrap();
    let sub_client = ListenerClient::new().unwrap();

    // First set of transforms at t=0.0
    let transforms_t0 = vec![
        (
            "a",
            "b",
            [0.9542820082386645, -0.6552492462418078, 0.7161777435789107],
            [
                0.5221303556354912,
                0.35012976926397515,
                0.06385453213291199,
                0.06388534296762166,
            ],
        ),
        (
            "b",
            "c",
            [
                -0.19846060797892018,
                0.37060239713344223,
                -0.9325041671812722,
            ],
            [
                0.17508543470264146,
                0.015141878067977513,
                0.7464281310309472,
                0.0633445561984338,
            ],
        ),
        (
            "c",
            "d",
            [-0.794492125974928, 0.3998294717449842, 0.10994520945722774],
            [
                0.09927023004042039,
                0.3127284173757304,
                0.09323219806580624,
                0.49476915451804293,
            ],
        ),
        (
            "d",
            "e",
            [
                -0.10568484318994975,
                -0.25311133155256416,
                -0.5050832697305845,
            ],
            [
                0.34253037231148725,
                0.18360347226679302,
                0.03909759741077618,
                0.43476855801094355,
            ],
        ),
        (
            "e",
            "f",
            [
                0.08519341627411214,
                -0.21820466927246485,
                -0.49430885607234565,
            ],
            [
                0.5030721633460956,
                0.42228251371020586,
                0.05757558742063205,
                0.017069735523066495,
            ],
        ),
    ];

    // Second set of transforms at t=1.0
    let transforms_t1 = vec![
        (
            "a",
            "b",
            [-0.2577564261850547, 0.7493551580360949, 0.9508883926449649],
            [
                0.22516451641196783,
                0.39948597131211394,
                0.2540343540211825,
                0.12131515825473572,
            ],
        ),
        (
            "b",
            "c",
            [
                0.8409405814571027,
                -0.9879602392577504,
                -0.13140102332772097,
            ],
            [
                0.1398908842037251,
                0.2758514837076157,
                0.24490871323462493,
                0.33934891885403434,
            ],
        ),
        (
            "c",
            "d",
            [
                0.22500109579960625,
                -0.1414475909286277,
                -0.14392029811070084,
            ],
            [
                0.19694092483717301,
                0.27122448763510776,
                0.4097865936798704,
                0.12204799384784887,
            ],
        ),
        (
            "d",
            "e",
            [
                -0.20684779237257978,
                -0.7643987654163593,
                -0.6253015724407152,
            ],
            [
                0.27849097201454626,
                0.15911896201926773,
                0.19901604722897315,
                0.3633740187372129,
            ],
        ),
        (
            "e",
            "f",
            [-0.09213549320472025, 0.7601862256435243, -0.84895940549366],
            [
                0.002094505867313596,
                0.13339467043347925,
                0.22297487081296374,
                0.6415359528862433,
            ],
        ),
    ];

    // Add transforms at t=0.0
    for (source, target, translation, rotation) in transforms_t0 {
        let stamped_isometry = StampedIsometry {
            isometry: Isometry3::from_parts(
                Translation3::new(translation[0], translation[1], translation[2]),
                UnitQuaternion::from_quaternion(Quaternion::new(
                    rotation[3],
                    rotation[0],
                    rotation[1],
                    rotation[2],
                )),
            ),
            stamp: 0.0,
        };
        client.send_transform(
            &source.to_string(),
            &target.to_string(),
            stamped_isometry.isometry.translation,
            stamped_isometry.isometry.rotation,
            0.0,
            TransformType::Dynamic,
        );
    }

    // Add transforms at t=1.0
    for (source, target, translation, rotation) in transforms_t1 {
        let stamped_isometry = StampedIsometry {
            isometry: Isometry3::from_parts(
                Translation3::new(translation[0], translation[1], translation[2]),
                UnitQuaternion::from_quaternion(Quaternion::new(
                    rotation[3],
                    rotation[0],
                    rotation[1],
                    rotation[2],
                )),
            ),
            stamp: 1.0,
        };
        client.send_transform(
            &source.to_string(),
            &target.to_string(),
            stamped_isometry.isometry.translation,
            stamped_isometry.isometry.rotation,
            1.0,
            TransformType::Dynamic,
        );
    }

    // Test cases at different timestamps
    let test_cases = vec![
        // a->f at t=0.2
        (
            0.2,
            "a",
            "f",
            [-0.02688966809486315, 0.8302180267299373, 1.6491944090937691],
            [
                -0.23762484510717535,
                0.7704449853702972,
                -0.44625068910795557,
                -0.38834170517242694,
            ],
        ),
        // a->f at t=0.5
        (
            0.5,
            "a",
            "f",
            [-0.7313014953477409, 0.8588360737131203, 1.3897218882465063],
            [
                -0.20299191732296193,
                0.9561102276829774,
                0.10847159958471206,
                -0.1813323636450122,
            ],
        ),
        // a->f at t=0.8
        (
            0.8,
            "a",
            "f",
            [-1.5366396114062963, 0.5615052687815749, 1.2753385241243729],
            [
                0.025710201700027795,
                0.8191599958838035,
                0.5182799902870279,
                0.2443393917080692,
            ],
        ),
        // f->a at t=0.2
        (
            0.2,
            "f",
            "a",
            [1.7623488465323582, 0.4146044950680975, 0.36339631387666715],
            [
                0.23762484510717535,
                0.7704449853702972,
                -0.44625068910795557,
                -0.38834170517242694,
            ],
        ),
        // f->a at t=0.5
        (
            0.5,
            "f",
            "a",
            [0.8453152942269395, 1.4598104847572575, 0.5984342964929825],
            [
                0.20299191732296193,
                0.9561102276829774,
                0.10847159958471206,
                -0.1813323636450122,
            ],
        ),
        // f->a at t=0.8
        (
            0.8,
            "f",
            "a",
            [-0.43273825025921875, 1.1678464326290772, 1.6588882210342657],
            [
                -0.025710201700027795,
                0.8191599958838035,
                0.5182799902870279,
                0.2443393917080692,
            ],
        ),
    ];

    // Test each case
    for (time, source, target, translation, rotation) in test_cases {
        let result: StampedIsometry = sub_client
            .request_transform(&source.to_string(), &target.to_string(), time)
            .unwrap()
            .into();

        let expected_translation =
            Translation3::new(translation[0], translation[1], translation[2]);
        let expected_rotation = UnitQuaternion::from_quaternion(Quaternion::new(
            rotation[0], // w
            rotation[1], // x
            rotation[2], // y
            rotation[3], // z
        ));

        // Assert translation components
        assert_relative_eq!(
            result.isometry.translation,
            expected_translation,
            epsilon = 1e-6,
            max_relative = 1e-6
        );

        // Assert rotation components
        assert_relative_eq!(
            result.isometry.rotation,
            expected_rotation,
            epsilon = 1e-6,
            max_relative = 1e-6
        );
    }
    server_handle.join().unwrap();
}
