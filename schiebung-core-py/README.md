# Python Bindings for Schiebung

This crate contains the Python bindings for the Schiebung library.
It provides a Python interface to the core functionality of the Schiebung library.

## Installation

```bash
maturin build
```

## Usage

```python
from schiebung import BufferTree, StampedIsometry, TransformType

buffer = BufferTree()
buffer.update("base_link", "target_link", StampedIsometry(translation=(1, 0, 0), rotation=(0, 0, 0, 1), stamp=1.0), TransformType.Static)
result = buffer.lookup_transform("base_link", "target_link", 1.0)

print(f"Translation: {result.translation()}")
print(f"Rotation: {result.rotation()}")
print(f"Euler angles: {result.euler_angles()}")
```

### Dynamic Transforms with Interpolation

```python
from schiebung import BufferTree, StampedIsometry, TransformType
import time

buffer = BufferTree()

# Add transforms at different times
for i in range(5):
    t = i * 0.1
    transform = StampedIsometry(
        translation=[i * 0.1, 0.0, 0.0],
        rotation=[0.0, 0.0, 0.0, 1.0],
        stamp=t
    )
    buffer.update("base", "end", transform, TransformType.Dynamic)

# Interpolate at intermediate time
result = buffer.lookup_transform("base", "end", 0.25)
print(f"Interpolated transform: {result}")
```