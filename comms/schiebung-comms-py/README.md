# schiebung_comms

Python bindings for the schiebung transform client/server — standard ROS 2
messages over [Zenoh](https://zenoh.io) ([rmw_zenoh](https://github.com/ros2/rmw_zenoh)
compatible), no ROS installation required.

- `TransformServer` — transport-only server: subscribes `/tf` and `/tf_static`,
  serves lookups via the `schiebung_msgs/LookupTransform` service. (For the
  combined server with Rerun visualization use the `schiebung_server` package.)
- `TransformClient` — publishes transforms and requests lookups.
- `ZenohConfig` — session configuration (endpoints, multicast, ROS domain id).

## Example

```python
from schiebung_comms import (
    StampedIsometry,
    TransformClient,
    TransformServer,
    TransformType,
    ZenohConfig,
)

# Server (e.g. in its own process). Pin endpoints on networks without
# UDP multicast; ZenohConfig() defaults to peer mode with multicast discovery.
server = TransformServer(ZenohConfig(listen=["tcp/127.0.0.1:7447"], multicast_scouting=False))
handle = server.start()

# Client
client = TransformClient(ZenohConfig(connect=["tcp/127.0.0.1:7447"], multicast_scouting=False))
client.send_transform(
    "world", "base",
    StampedIsometry([0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0], 0),
    TransformType.Static,
)
result = client.request_transform("world", "base", 0)  # time 0 = latest
print(result.translation())

handle.shutdown()
handle.join()
```

`ZenohConfig.from_env()` reads `SCHIEBUNG_LISTEN`, `SCHIEBUNG_CONNECT`,
`SCHIEBUNG_MULTICAST` and `ROS_DOMAIN_ID`.

## Development

```bash
cd comms/schiebung-comms-py
uv venv && uv pip install pytest
VIRTUAL_ENV=$PWD/.venv maturin develop --uv
.venv/bin/python -m pytest tests/
```
