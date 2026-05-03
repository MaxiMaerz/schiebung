# Schiebung Core - Python Bindings

Python bindings for the schiebung core library — a fast, memory-safe in-memory transform graph with time-aware lookups. Useful for multi-sensor fusion, simulation, motion capture, AR/VR, and robotics.

## Installation

```bash
pip install schiebung
```

## Quick Start

```python
import time
import numpy as np
from schiebung import BufferTree, StampedIsometry, TransformType

buffer = BufferTree()

# stamp accepts int (nanoseconds) or float (seconds since the Unix epoch).
transform = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], time.time())

# update() takes a list of (from, to, transform, kind) tuples — push many in one call.
buffer.update([("world", "robot", transform, TransformType.Dynamic)])

result = buffer.lookup_latest_transform("world", "robot")
print("translation:", result.translation())   # python list
print("matrix:\n", result.as_matrix())          # 4x4 numpy array

# StampedIsometry implements __array__, so it works directly with numpy:
inverse = np.linalg.inv(result)
```

## Documentation

Full documentation: [https://maximaerz.github.io/schiebung/](https://maximaerz.github.io/schiebung/) ·
API reference: [docs.rs/schiebung](https://docs.rs/schiebung)
