# Schiebung Comms

Client-Server communication for Schiebung transforms using Zenoh and capn'proto. Currently no Python bindings are provided; however, it is possible to connect against the server with any zenoh client. If we see the need for users to run the server in a python environment, we can create the bindings.

## Overview

The comms package provides a client-server architecture for sharing transform data between processes. It uses [Zenoh](https://zenoh.io/) for low-latency inter-process communication.
We use a simple capn'proto schema to serialize the data (currently none of the fancy cap'n proto RPC Features are used).

## Architecture

```mermaid
graph LR
    Client_1["Client<br/>- Query<br/>- Publish"]
    Client_N["Client<br/>- Query<br/>- Publish"]
    Server["Server<br/>BufferTree"]

    Client_1 <-->|Zenoh| Server
    Client_N <-->|Zenoh| Server
```

### Transform Update Flow

```mermaid
sequenceDiagram
    participant Client
    participant Zenoh
    participant Server

    Client->>Zenoh: Publish Transform
    Zenoh->>Server: Forward Transform
    Server->>Server: Update BufferTree
    Note over Client,Server: Fire-and-forget (no acknowledgment)
```

### Transform Request Flow

```mermaid
sequenceDiagram
    participant Client
    participant Zenoh
    participant Server

    Client->>Zenoh: Query Transform (from, to)
    Zenoh->>Server: Forward Query
    Server->>Server: Lookup in BufferTree
    Server-->>Zenoh: Transform Result
    Zenoh-->>Client: Return Transform
```

## Usage

### Client Example

```rust
use comms::TransformClient;

#[tokio::main]
async fn main() {
    let client = TransformClient::new().await.unwrap();

    // Publish a transform
    client.publish_transform("base", "sensor", transform).await;

    // Query a transform
    let result = client.request_transform("base", "sensor").await;
}
```
