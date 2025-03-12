# Schiebung

Schiebung reimplements the tf2 library without the need for ROS. However the Buffer can be filled with data from ROS.

The ROS implementation can be found here: [tf2](https://github.com/ros2/geometry2/tree/master/tf2/src/buffer)

It also draws inspiration form the rust implementation for ROS 1 [rosrust_tf](https://github.com/arjo129/rustros_tf)

Currently we use [iceoryx2](https://github.com/eclipse-iceoryx/iceoryx2) for inter-process communication.
They claim extremely low latencies. However this has to be evaluated and tested.

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
cargo run --bin schiebung-client update --from a --to b --tx 0 --ty 0 --tz 0 --qx 0 --qy 0 --qz 0 --qw 1
```

Request a transform from the server:

```bash
cargo run --bin schiebung-client request --from a --to b --time 1.0
```

Visualize the transforms:

The default save path is your home directory and may be changed within the server config.

```bash
cargo run --bin schiebung-client visualize
```

However in an integrated system you might already have a ROS-tf2 context and want to use that.
In this case you can use [schiebung_ros2](https://github.com/MaxiMaerz/schiebung_ros2) to update the buffer with ROS-tf2 data.

