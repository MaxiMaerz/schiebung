# Schiebung Core - Python Bindings

Python bindings for the schiebung core library - a fast, memory-safe transform buffer for robotics.

## Installation

```bash
pip install schiebung
```

## Quick Start

```python
from schiebung import BufferTree, StampedIsometry, TransformType
import time

# Create buffer
buffer = BufferTree()

# Add a transform (timestamp in nanoseconds)
transform = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], int(time.time() * 1e9))
buffer.update("world", "robot", transform, TransformType.Dynamic)

# Lookup
result = buffer.lookup_latest_transform("world", "robot")
print(f"Translation: {result.translation}")
```

## Documentation

Full documentation: [https://maximaerz.github.io/schiebung/](https://maximaerz.github.io/schiebung/)
