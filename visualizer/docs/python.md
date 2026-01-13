# Schiebung Rerun Integration

Python bindings for schiebung with integrated Rerun visualization.

While the Rust implementation exposes the recording stream directly when registering the observer, we do not have the same luxury in Python. We use reruns amazing feature to [merge recordings](https://rerun.io/docs/concepts/apps-and-recordings).
This means that by having the same application and recording ID the compiled part of the 'RerunBufferTree' can be accessed and visualized together with your Python code. This even works across Processes.

## Installation

```bash
cd schiebung-rerun-py
maturin develop
```

## Usage

```python
from schiebung_rerun import RerunBufferTree, StampedIsometry, TransformType

# Create a RerunBufferTree with Rerun logging
tree = RerunBufferTree("schiebung", "session_001", "stable_time", True)

# Access the buffer to add transforms (they will be logged to Rerun automatically)
# Timestamp in nanoseconds (1_000_000_000 = 1 second)
t = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 1_000_000_000)
tree.buffer.update("world", "robot", t, TransformType.Static)

# Query transforms via the buffer
result = tree.buffer.lookup_latest_transform("world", "robot")
print(result.translation())  # [1.0, 0.0, 0.0]
```
