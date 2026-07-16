# ROS 2 Interop Test

End-to-end test that schiebung comms interoperates with a real ROS 2 system
running [rmw_zenoh](https://github.com/ros2/rmw_zenoh).

```bash
./comms/docker/test_ros_interop.sh
```

The script:

1. Builds a **ROS 2 Jazzy + rmw_zenoh** image (`Dockerfile.ros`) and runs it on
   the host network. The container starts the `rmw_zenohd` router (port 7447)
   and a `static_transform_publisher` broadcasting `world -> base` (x = 1.0)
   on `/tf_static`.
2. Starts the schiebung `server` binary on the host, connected to the router.
3. Verifies, via `examples/ros_interop_check.rs`:
   - **ROS -> schiebung**: the ROS static transform is retrievable through the
     schiebung lookup service (also proves the RIHS01 type hashes of the
     vendored message definitions match the ROS distro — mismatched hashes
     would silently not match endpoints).
   - **schiebung -> ROS**: dynamic transforms published by the schiebung
     client are visible to `ros2 topic echo /tf` inside the container.
   - **Graph interop**: the schiebung nodes appear in `ros2 node list`.

Requires Docker with host networking (Linux) and the Rust toolchain.
