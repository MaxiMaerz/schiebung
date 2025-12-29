use log::error;

#[tokio::main]
async fn main() {
    env_logger::Builder::new()
        .filter(None, log::LevelFilter::Info)
        .init();

    match comms::server::TransformServer::new().await {
        Ok(server) => {
            if let Err(e) = server.run().await {
                error!("Server error: {}", e);
                std::process::exit(1);
            }
        }
        Err(e) => {
            error!("Failed to initialize server: {}", e);
            std::process::exit(1);
        }
    }
}
