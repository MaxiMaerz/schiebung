#[tokio::main]
async fn main() -> Result<(), String> {
    env_logger::Builder::new()
        .filter(None, log::LevelFilter::Info)
        .init();

    comms::server::run_server().await
}
