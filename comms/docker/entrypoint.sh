#!/bin/bash
# Entrypoint for the ROS 2 interop test container: starts the rmw_zenoh
# router and a static_transform_publisher (world -> base, x = 1.0).
set -e
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

echo "Starting rmw_zenohd router..."
ros2 run rmw_zenoh_cpp rmw_zenohd &
ROUTER_PID=$!

sleep 2

echo "Starting static_transform_publisher: world -> base (x=1.0)..."
ros2 run tf2_ros static_transform_publisher --x 1 --frame-id world --child-frame-id base &

wait $ROUTER_PID
