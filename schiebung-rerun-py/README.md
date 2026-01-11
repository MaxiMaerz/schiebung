# schiebung_rerun

Python bindings for schiebung with integrated Rerun visualization.

## Installation

```bash
cd schiebung-rerun-py
maturin develop
```

## Usage

```python
from schiebung_rerun import RerunBufferTree, StampedIsometry, TransformType

# Create a buffer with Rerun logging
buf = RerunBufferTree("my_recording", "stable_time", True)

# Add transforms (they will be logged to Rerun automatically)
t = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0)
buf.update("world", "robot", t, TransformType.Static)

# Query transforms
result = buf.lookup_latest_transform("world", "robot")
print(result.translation())  # [1.0, 0.0, 0.0]
```

## API

### RerunBufferTree

Same API as `BufferTree` from `schiebung` but with integrated Rerun logging.

```python
RerunBufferTree(recording_id: str, timeline: str, publish_static_transforms: bool)
```

- `recording_id`: The ID for the Rerun recording
- `timeline`: The name of the timeline for logging transforms
- `publish_static_transforms`: Whether to log static transforms (set to False if loading URDF via Rerun)

### Other Types

- `StampedIsometry`: Same as in `schiebung`
- `TransformType`: Same as in `schiebung`
- `TfError`: Same as in `schiebung`
- `UrdfLoader`: Same as in `schiebung`, but takes `RerunBufferTree` instead of `BufferTree`
