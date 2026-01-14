# Schiebung Server

Python bindings for schiebung with integrated Rerun visualization. Check the [Rust docs](rust.md) for more information on implementation.

## Installation

```bash
cd schiebung-server-py
maturin develop
```

## Usage

```python
from schiebung_server import Server, TransformClient, StampedIsometry, TransformType

# Start the server (in a separate process/thread)
server = Server("schiebung", "session_001", "stable_time", True)
handle = server.start()

# Access the buffer while server is running
buffer = server.buffer

# Client usage
client = TransformClient()
transform = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0)
client.send_transform("world", "robot", transform, TransformType.static_transform())

# Query transforms via buffer
result = buffer.lookup_latest_transform("world", "robot")
print(result.translation())  # [1.0, 0.0, 0.0]
```

## Example

We provide a docker setup which demonstrates the server and client usage. It initializes the same Sun-Earth-Moon system we use in the rerun only example.
