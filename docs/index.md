# Schiebung

Full [Documenation](https://maximaerz.github.io/schiebung/)

Schiebung is german for "shift" as in "shift the frame" or "shift the coordinate system".

Schiebung offers a library which stores transformations (or isometries) between frames. These isometries are between two frames.
It is assumed that all frames are either connected or root/leaf nodes. The resulting structure is used to produce any transformation between frames by chaining their transformations. Additionally each pair of frames keeps a history of transformations, this allows a user to ask for transformations in the past, if the exact time cannot be found the transformation between the two best matching times will be interpolated (lerp/slerp).

The original concept and a far better explanation can be found here: [ROS tf](http://wiki.ros.org/tf)
It also draws inspiration from the rust implementation for ROS 1 [rosrust_tf](https://github.com/arjo129/rustros_tf)

![Demo showing Sun-Earth-Moon transform visualization](demo.gif)

In the video you can see a very basic example utilizing the TransformBuffer in combination with the rerun visualizer:
We publish the bodies once (Sun, Earth and Moon) and just update the transforms in their respective frames:

- earth circles around the sun
- moon circles around the earth
- The distance between the moon and sun is calculated by a simple frame lookup (Moon -> Sun)

This allows the user to make a complexe (in this context) problem complelty trivial. As long as the TransformBuffer is updated all derived transformations
can be calculated at all times (using lerp/slerp). It also makes it possible to calculate transformations without knowing anything of the actual transformation chain as long as the end and start of the chain are known.

## Motivation

TF2 is available in ROS2, the implementation is super sturdy and battle tested.
However some project might not want the huge ROS dependencies overhead. While it is possible to use TF2 without ROS, it lacks some features we need.

It is most likely that our implementation also:

- is faster than TF2 (40-80ns per lookup)
- is memory safe (typical RUST argument)

However TF2 is a great tool and will most likely be the best choice for ROS 2 based projects. This project is very new and Bugs will most likely be found.

## Design goals

- As fast as possible: Sub-microsecond lookups and updates
- ROS (2) agnostic: While it is possible to use alongside TF2, the target is a system outside the ROS ecosystem.
- All features are optional: Depending on user requirements dependencies can be pulled in on required feature set
- Minimal, scalable and fast client server structure: We use [cap'n proto](https://capnproto.org) + [zenoh](https://zenoh.io) for communication.
- Easy to use: We ship a urdf loader and provide a simple [Rerun](https://rerun.io) API.

## Status

This library is still under development and the API is not considered stable yet and might change.

- The core library is fairly well tested, the python bindings work.
- The rerun implementation is a rather shallow wrapper around the core and should be "ok"
- For the server and the full implementation (server + rerun) there is quite some work left.

## Usage

Check the other pages for examples and use cases:

- [Core](schiebung-core/rust.md): Contains a rust and python implementation for the Buffer only, no dependencies to rerun or zenoh.
- [Comms](schiebung-comms/index.md): Contains a rust and python implementation for the client server structure using cap'n proto and zenoh.
- [Visualizer](schiebung-visualizer/rust.md): Contains a rust and python implementation for the visualizer using rerun.
