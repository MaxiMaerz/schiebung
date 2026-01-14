# Schiebung Rerun - Python Bindings

Python bindings for schiebung with Rerun visualization support.

## Installation

```bash
pip install schiebung-rerun
```

## Quick Start

```python
from schiebung_rerun import RerunBuffer, StampedIsometry, TransformType
import time

# Create buffer with Rerun visualization
buffer = RerunBuffer("my_app", "recording_1", "stable_time")

# Add transforms - automatically visualized in Rerun
transform = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], int(time.time() * 1e9))
buffer.update("world", "robot", transform, TransformType.Dynamic)
```

## Documentation

Full documentation: [https://maximaerz.github.io/schiebung/](https://maximaerz.github.io/schiebung/)
