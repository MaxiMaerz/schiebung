# Core Library for Schiebung

This crate contains the pure Rust core functionality of the Schiebung library.
It provides a buffer for storing and retrieving transforms without any Python dependencies.

## Installation

```bash
git clone git@github.com:MaxiMaerz/schiebung.git
cd schiebung
cargo build
```

## Usage

Schiebung can be used as a library or as a client-server application.

### Library

This will create a local buffer, this buffer will NOT fill itself!

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
```
