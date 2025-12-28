#[tokio::main]
async fn main() {
    env_logger::Builder::new()
        .filter(None, log::LevelFilter::Info)
        .init();

    if let Err(e) = comms::server::run_server().await {
        eprintln!("Server error: {}", e);
        std::process::exit(1);
    }
}
