# Schiebung

Schiebung is german for "shift" as in "shift the frame" or "shift the coordinate system".

Schiebung offers a library which stores transformations (or isometries) between frames. These isometries are between two frames. 
It is assumed that all frames are either connected or root/leaf nodes. The resulting structure is used to produce any transformation between frames by chaining their transformations. Additionally each pair of frames keeps a history of transformations, this allows a user to ask for transformations in the past, if the exact time cannot be found the transformation between the two best matching times will be interpolated.

The original concept and a far better explanation can be found here: [ROS tf](http://wiki.ros.org/tf)
It also draws inspiration form the rust implementation for ROS 1 [rosrust_tf](https://github.com/arjo129/rustros_tf)

The motivation for this package is a missing implementation for Rust in ROS 2. 

Design goals

* ROS agnostic library. The core functionality should be usable without any ROS dependencies or knowledge.
* A client server architecture which works without ROS. We want to focus on low latency without remote access.
* Integration into the ROS ecosystem without compromising on the points above.

The project consists of the following crates:

* schiebung-core: data-structures and functions to store and lookup transforms.
* schiebung-client/server: A standalone server which can be accessed by multiple clients which can store and lookup transformations. They interface via iceoryx2 IPC.
* schiebung_ros2: A ROS 2 interface to fill the buffer on a schiebung-server with data while still allowing a client to connect to it without ROS. It also offers a library which fills a buffer locally without any server/client access (The traditional ROS implementation).

For small applications a few local instances of the Buffer are of no concern, however for larger projects it can make sense to limit the amount of subscribers to tf and keep a global buffer.

Currently we use [iceoryx2](https://github.com/eclipse-iceoryx/iceoryx2) for inter-process communication.\
They claim extremely low latencies. However this has to be evaluated and tested.

However in an integrated system you might already have a ROS-tf2 context and want to use that.\
In this case you can use schiebung_ros2: [README](schiebung_ros2/README.md).

This library is still under development and the API is not considered stable yet.

## Status

| Crate          | Usable | Published |
|------------------|---------|-----------|
| schiebung-core     | Yes     | No        |
| schiebung-ros2      | Yes     | No        |
| schiebung-client    | Yes      | No        |
| schiebung-server    | Yes      | No        |

The core implementation of the Buffer is tested, we are still missing a representative test for the interpolation / time travel feature.
It yields the same result as the ROS-implementation.

The client and server are tested for correct results, however until iceoryx2 merges the request/response functionality heavy traffic may cause issues.

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

