use iceoryx2::prelude::*;
use log::info;
use schiebung_server::Server;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::new()
        .filter(None, log::LevelFilter::Error)
        .init();
    let server = Server::new()?;

    let waitset = WaitSetBuilder::new().create::<ipc::Service>()?;
    let request_listener_guard = waitset.attach_notification(&server.request_listener_notifier)?;
    let transform_listener_guard =
        waitset.attach_notification(&server.transform_listener_event_listener)?;
    let visualizer_event_guard = waitset.attach_notification(&server.visualizer_listener)?;

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
    waitset.wait_and_process(fn_call)?;
    info!("Server shutting down");
    Ok(())
}
