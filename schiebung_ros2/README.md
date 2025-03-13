# ROS 2 interface for schiebung

This crate provides a ROS 2 interface for the schiebung library.


## Installation

Follow the instructions on: [ros2-rust](https://github.com/ros2-rust/ros2_rust).
Then in a ros workspace clone the geometry2 package. to build the rust bindings for tf messages.

```bash
cd <ros2_workspace>
cd src/ros2
git clone git@github.com:ros2/geometry2.git
```

Then build the workspace.

```bash
colcon build --packages-select schiebung_ros2
```





There are two ways to use the TF Buffer:

* Use the `RosBuffer` struct to listen to the `/tf` and `/tf_static` topics and build a TF buffer in memory.
* Use the `TFRelay` fills a buffer with the latest TF data from the `/tf` and `/tf_static` topics.
  This buffer can be used to lookup transforms and visualize the TF tree. using the schiebung-client library
  which does not use ROS 2.

## RosBuffer

The `RosBuffer` struct listens to the `/tf` and `/tf_static` topics and builds a TF buffer in memory.
This buffer can be used to lookup transforms and visualize the TF tree. it works like the TF buffer in ROS.

NOTE: The executor will NOT spin automatically but must be spun by the user.

```rust
let mut executor = Context::default_from_env()?.create_basic_executor();
let buffer = RosBuffer::new(&executor)?;

while executor.spin(SpinOptions::spin_once()) {
    println!("Lookup transform: {:?}", buffer.lookup_transform("base_link", "odom", 0.0)?);
}
```

## TFRelay

The `TFRelay` fills a buffer with the latest TF data from the `/tf` and `/tf_static` topics.
The Buffer server must be started in another thread (check the documentation of the schiebung-client library for more information).

```bash
ros2 launch schiebung_ros2 schiebung_ros2.launch.xml
```