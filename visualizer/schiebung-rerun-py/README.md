# Schiebung Rerun - Python Bindings

Python bindings for schiebung's [Rerun](https://rerun.io) visualization adapter — every transform you push is bulk-logged to a Rerun recording stream so you can visualize the graph live.

## Installation

```bash
pip install schiebung-rerun
```

## Quick Start

```python
import time
from schiebung_rerun import RerunBufferTree, StampedIsometry, TransformType

# application_id, recording_id, timeline name, publish_static_transforms.
# Spawns a Rerun viewer by default; pass spawn=False (optionally with
# connect_addr="rerun+http://host:port/proxy") to connect to one that is
# already running instead.
tree = RerunBufferTree("my_app", "recording_1", "stable_time", True)

# stamp accepts int (ns) or float (s).
transform = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], time.time())

# Updates go through tree.buffer; the underlying observer logs each batch to Rerun.
tree.buffer.update("world", "robot", transform, TransformType.Dynamic)
tree.buffer.update_batch([("world", "robot", transform, TransformType.Dynamic)])
```

`StampedIsometry`, `BufferTree`, `TransformType`, `TfError` and `UrdfLoader` are
re-exported from the [`schiebung`](https://pypi.org/project/schiebung/) package —
they are the *same* types, so values pass freely between the two packages
(`schiebung_rerun.StampedIsometry is schiebung.StampedIsometry`).

### Tuning the recording stream

`RerunBufferTree` / `RerunObserver` take an optional `batcher_config` — a
[`rerun.ChunkBatcherConfig`](https://ref.rerun.io/docs/python/stable/common/initialization_functions/#rerun.ChunkBatcherConfig)
(including its `DEFAULT` / `LOW_LATENCY` / `ALWAYS` / `NEVER` presets) controlling
the chunk-batcher flush thresholds:

```python
import rerun as rr
tree = RerunBufferTree("my_app", "recording_1", "stable_time", True,
                       batcher_config=rr.ChunkBatcherConfig.LOW_LATENCY())
```

The `RERUN_FLUSH_TICK_SECS` / `RERUN_FLUSH_NUM_BYTES` / `RERUN_FLUSH_NUM_ROWS` /
`RERUN_CHUNK_MAX_ROWS_IF_UNSORTED` environment variables still override it.

## Documentation

Full documentation: [https://maximaerz.github.io/schiebung/](https://maximaerz.github.io/schiebung/) ·
API reference: [docs.rs/schiebung-rerun](https://docs.rs/schiebung-rerun)
