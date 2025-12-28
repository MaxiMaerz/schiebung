# Comms Package

Simplified communication layer for schiebung using zenoh-rs and Cap'n Proto.

## Components

### Server Binary

Run the transform server:
```bash
cargo run --bin server
```

The server:
- Uses **zenoh in peer mode** (brokerless)
- Subscribes to `schiebung/transforms/new` topic
- Receives and stores transforms in a BufferTree
- Logs all incoming transforms

### Client Library

The `TransformClient` allows publishing transforms:

```rust
use comms::TransformClient;
use nalgebra::{ Translation3, UnitQuaternion};
use schiebung::types::TransformType;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let publisher = TransformClient::new().await?;

    publisher.send_transform(
        "world",
        "robot",
        Translation3::new(1.0, 2.0, 3.0),
        UnitQuaternion::identity(),
        0.0,  // timestamp
        TransformType::Static,
    ).await?;

    Ok(())
}
```

### Example

Run the example publisher:
```bash
# Terminal 1: Start server
cargo run --bin server

# Terminal 2: Run example
cargo run --example publish_transforms
```

## Architecture

- **Messages**: Cap'n Proto schema in `messages.capnp`
- **Communication Patterns**:
  - **Pub-Sub**: For broadcasting new transforms to all subscribers
  - **Request-Response**: For querying specific transforms via Zenoh queryables
- **Network**: Brokerless zenoh in peer mode
- **Config**: Serde-based `ZenohConfig`

## Key Features

- ✅ Simple pub-sub pattern
- ✅ Type-safe Cap'n Proto serialization
- ✅ Brokerless peer-to-peer communication
- ✅ Async/await with tokio
- ✅ Serde configuration
