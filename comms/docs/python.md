# Schiebung Comms — Python

Python bindings for the transport-only transform client/server. Speaks the standard ROS 2 wire format over Zenoh (`rmw_zenoh` compatible) — no ROS installation required. For the combined server with Rerun visualization use the [`schiebung_server`](https://maximaerz.github.io/schiebung/schiebung-server/python/) package, which shares these classes.

## Installation

```bash
cd comms/schiebung-comms-py
maturin develop
```

## Usage

```python
from schiebung_comms import (
    StampedIsometry,
    TransformClient,
    TransformServer,
    TransformType,
    ZenohConfig,
)

# Server (typically its own process). ZenohConfig() defaults to peer mode
# with multicast discovery; pin endpoints where multicast is unavailable.
server = TransformServer(ZenohConfig(listen=["tcp/127.0.0.1:7447"], multicast_scouting=False))
handle = server.start()

# Client
client = TransformClient(ZenohConfig(connect=["tcp/127.0.0.1:7447"], multicast_scouting=False))
transform = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0)
client.send_transform("world", "robot", transform, TransformType.Static)

# Query via the lookup service (time 0 = latest available, tf2 semantics)
result = client.request_transform("world", "robot", 0)
print(result.translation())  # [1.0, 0.0, 0.0]

# Fetch the server's full buffer content (latest sample per edge) as
# (from_frame, to_frame, StampedIsometry, TransformType) tuples
for from_frame, to_frame, transform, kind in client.request_all_transforms():
    print(from_frame, to_frame, transform.translation(), kind)

# Or read the server's buffer directly (same process)
result = server.buffer.lookup_latest_transform("world", "robot")

handle.shutdown()
handle.join()
```

## Configuration

`ZenohConfig` accepts `mode`, `listen`, `connect`, `multicast_scouting` and `domain_id` (the ROS domain id, relevant when talking to a ROS 2 system). `ZenohConfig.from_env()` reads `SCHIEBUNG_LISTEN`, `SCHIEBUNG_CONNECT`, `SCHIEBUNG_MULTICAST` and `ROS_DOMAIN_ID`.

## ROS 2 interop

Because the wire format matches `rmw_zenoh`, a Python client or server started with `connect` pointing at an `rmw_zenohd` router exchanges transforms directly with ROS 2 nodes: the server ingests the live `/tf` graph, and transforms sent with `send_transform` appear on `/tf` / `/tf_static` for ROS tooling. See the [overview](index.md) for details and the automated Docker interop test.
