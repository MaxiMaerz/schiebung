# Schiebung Server - Python Bindings

All-in-one transform server with remote access and Rerun visualization.

## Installation

```bash
pip install schiebung-server
```

## Quick Start

```python
from schiebung_server import Server

# Create server with Rerun visualization
server = Server("my_app", "recording_1", "stable_time", enable_rerun=True)

# Start server (non-blocking)
handle = server.start()

# Access buffer for local lookups
buffer = server.buffer
```

## Documentation

Full documentation: [https://maximaerz.github.io/schiebung/](https://maximaerz.github.io/schiebung/)
