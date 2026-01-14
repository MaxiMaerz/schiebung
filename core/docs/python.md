# Python Bindings for Schiebung

This crate contains the Python bindings for the Schiebung library.
It provides a Python interface to the core functionality of the Schiebung library.

The bindings are generated using [pyo3](https://pyo3.rs/) and [maturin](https://github.com/PyO3/maturin).
The binaries are published to [PyPI](https://pypi.org/project/schiebung/).

## Installation

The bindings can be installed using pip or uv:

```bash
pip install schiebung
```

For a quick demo:

```bash
uv run --with schiebung,ipython ipython
```

## Usage

```python
from schiebung import BufferTree, StampedIsometry, TransformType

buffer = BufferTree()
# Timestamps are in nanoseconds (1_000_000_000 = 1 second)
buffer.update("base_link", "target_link", StampedIsometry(translation=(1, 0, 0), rotation=(0, 0, 0, 1), stamp=1_000_000_000), TransformType.Static)
result = buffer.lookup_transform("base_link", "target_link", 1_000_000_000)

print(f"Translation: {result.translation()}")
print(f"Rotation: {result.rotation()}")
print(f"Euler angles: {result.euler_angles()}")
```

### Dynamic Transforms with Interpolation

```python
from schiebung import BufferTree, StampedIsometry, TransformType
import time

buffer = BufferTree()

# Add transforms at different times (timestamps in nanoseconds)
for i in range(5):
    t_ns = i * 100_000_000  # 100ms intervals in nanoseconds
    transform = StampedIsometry(
        translation=[i * 0.1, 0.0, 0.0],
        rotation=[0.0, 0.0, 0.0, 1.0],
        stamp=t_ns
    )
    buffer.update("base", "end", transform, TransformType.Dynamic)

# Interpolate at intermediate time (250ms in nanoseconds)
result = buffer.lookup_transform("base", "end", 250_000_000)
print(f"Interpolated transform: {result}")
```

## Visualize the buffer

The visualize function returns a graphviz dot string. This can used with any graphviz viewer.

```python
from schiebung import BufferTree, StampedIsometry, TransformType

buffer = BufferTree()
# Timestamp in nanoseconds (1_000_000_000 = 1 second)
iso = StampedIsometry([0,0,1], [0,0,0,1], 1_000_000_000)
buffer.update("a", "b", iso, TransformType.Dynamic)
buffer.update("a", "c", iso, TransformType.Dynamic)
buffer.update("b", "b_1", iso, TransformType.Dynamic)
buffer.update("c", "c_1", iso, TransformType.Dynamic)
print(buffer.visualize())
```
