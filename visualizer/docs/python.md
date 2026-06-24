# Schiebung Rerun Integration

Python bindings for schiebung with integrated Rerun visualization.

The default constructors own a Rust Rerun recording. Matching its application
and recording IDs with another producer lets Rerun merge their streams, even
across processes.

For a single in-process sink, `RerunObserver.from_recording` and
`RerunBufferTree.from_recording` instead log into an existing Python
`rerun.RecordingStream`. This is required when regular Python logs and
Schiebung transforms must be written by the same `rerun.FileSink`.

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

## Shared recording and file sink

```python
import rerun as rr
import schiebung
from schiebung_rerun import RerunObserver, StampedIsometry, TransformType

rec = rr.RecordingStream("schiebung", recording_id="session_001")
rec.set_sinks(rr.FileSink("session.rrd"))

buffer = schiebung.BufferTree()
buffer.register_observer(
    RerunObserver.from_recording(rec, "stable_time", True)
)

with rec:
    rec.log("points", rr.Points3D([[1.0, 2.0, 3.0]]))
    transform = StampedIsometry(
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        1_000_000_000,
    )
    buffer.update("world", "robot", transform, TransformType.Dynamic)
```

The recording owns and finalizes its sink. Schiebung does not open the file a
second time.
