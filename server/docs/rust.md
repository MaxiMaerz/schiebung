# Schiebung Server

This is the all-in-one solution that combines both functionalities from:

- [schiebung_rerun](../../visualizer/docs/rust.md)
- [schiebung_zenoh](../../comms/docs/rust.md)

This combines both functionalities and returns an object that stores transforms (in-thread or over the wire) and logs them to Rerun. It also can be queried over the network with zenoh, to provide transforms.

## Usage

There are two scenarios:

### Standalone: Just start the server via the binary

```bash
# Build the binary
cargo build --release --bin server

# Run with a config file
./target/release/server --config server.toml
```

This will log against the provided Rerun recording ID and Application ID. And allow clients to update and request transforms over the network.
This is the closest to the classic tf2 server.

### As a Library

```rust
use schiebung_server::Server;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create and run the server
    // - application_id: identifies the application in Rerun
    // - recording_id: identifies this specific recording session
    // - timeline: timeline name for Rerun
    // - publish_static_transforms: set to false if loading URDF via Rerun
    let server = Server::new("schiebung", "session_001", "stable_time", true).await?;
    server.run().await?;
    Ok(())
}
```

This is our equivalent of in process tf2 Buffer, however it will not yet sync existing transforms over the network.
