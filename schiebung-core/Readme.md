# Schiebung-core

This crate contains the core functionality of the Schiebung library.
It provides a buffer for storing and retrieving transforms.

## Usage

```rust
use schiebung_core::BufferTree;

let buffer = BufferTree::new();

let stamped_isometry = StampedIsometry {
    isometry: Isometry::from_parts(
        Translation3::new(
            1.0,
            2.0,
            3.0,
        ),
        UnitQuaternion::new_normalize(Quaternion::new(
            0.0,
            0.0,
            0.0,
            1.0,
        )),
    ),
    stamp: 1.0
};
buffer.update("base_link", "target_link", stamped_isometry, TransformType::Static);

let transform = buffer.lookup_transform("base_link", "target_link", 1.0);
buffer.visualize();
```

## Configuration



