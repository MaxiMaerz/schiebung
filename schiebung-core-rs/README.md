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

### Client-Server

Here the server runs as a standalone process. Clients can connect to the server and request and send transforms.

```bash
cargo run --bin schiebung-server
```

Now the server is running we need to provide transforms, this can be done manually with a client:

Update the server with a new static transform:

```bash
cargo run --bin schiebung-client update --from a --to b --tx 1 --ty 0 --tz 0 --qx 0 --qy 0 --qz 0 --qw 1
cargo run --bin schiebung-client update --from b --to c --tx 0 --ty 1 --tz 0 --qx 0 --qy 0 --qz 0 --qw 1
cargo run --bin schiebung-client update --from c --to d --tx 0 --ty 0 --tz 1 --qx 0 --qy 0 --qz 0 --qw 1
cargo run --bin schiebung-client update --from b --to x --tx 1 --ty 1 --tz 1 --qx 0 --qy 0 --qz 0 --qw 1
```

Request a transform from the server:

```bash
cargo run --bin schiebung-client request --from a --to d
Transform:
a -> d:
stamp: 0,
translation: 1.000, -1.000, 1.000,
rotation (xyzw): 0.000, 0.000, -1.000, -0.000,
rotation (rpy): -0.000, -0.000, 3.142
```

Visualize the transforms:

The default save path is your home directory and may be changed within the server config.

```bash
cargo run --bin schiebung-client visualize
```

![Graph](./resources/graph.png)
