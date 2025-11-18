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
In this case you can use schiebung_ros2: [README](schiebung_ros2/Readme.md).

This library is still under development and the API is not considered stable yet.

## Status

| Crate          | Usable | Published |
|------------------|---------|-----------|
| schiebung-core     | Yes     | No        |
| schiebung-ros2      | Yes     | No        |
| schiebung-client    | Yes      | No        |
| schiebung-server    | Yes      | No        |

The client and server are tested for correct results, however until iceoryx2 merges the request/response functionality heavy traffic may cause issues.


## Usage

Rust core: [README](schiebung-core/README.md)
Python bindings: [README](schiebung-py/README.md)
Client-Server: [README](schiebung-client/README.md)
