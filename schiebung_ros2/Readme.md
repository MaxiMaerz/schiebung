# ROS 2 interface for schiebung

This crate provides a ROS 2 interface for the schiebung library.

## Installation

Follow the instructions on: [ros2-rust](https://github.com/ros2-rust/ros2_rust).
Then in a ros workspace clone the geometry2 package. to build the rust bindings for tf messages.

```bash
cd <ros2_workspace>
cd src/ros2
git clone git@github.com:ros2/geometry2.git
cd geometry2
git checkout YOUR_ROS_VERSION
```

Then build the workspace.

```bash
# Ignore the schiebung-core package it has no binaries and colcon will fail
colcon build --packages-ignore schiebung-core
```

Afterwards you should be able to:

```bash
ros2 launch schiebung_ros2 schiebung_ros2.launch.xml
```

If anything fails check the FAQ at [ros2-rust](https://github.com/ros2-rust/ros2_rust/wiki/FAQ).
For me restarting from scratch (delete the src, build and install folders) and starting over fixed most issues.

## Usage

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
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let buffer = RosBuffer::new(&executor)?;

    loop {
        executor.spin(SpinOptions::spin_once());
        let res = buffer.lookup_latest_transform("wrist_1_link", "wrist_3_link");
        match res {
            Ok(transform) => {
                println!("Lookup transform: {:?}", transform);
            }
            Err(e) => {
                println!("Error: {:?}", e);
            }
        }
    }
    Ok(())
}

```

## TFRelay

The `TFRelay` fills a buffer with the latest TF data from the `/tf` and `/tf_static` topics.
The Buffer server must be started in another thread (check the documentation of the schiebung-client library for more information).

```bash
ros2 launch schiebung_ros2 schiebung_ros2.launch.xml
```