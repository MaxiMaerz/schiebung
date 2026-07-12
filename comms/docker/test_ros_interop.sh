#!/usr/bin/env bash
# End-to-end ROS 2 interop test for schiebung comms.
#
# Runs a ROS 2 Jazzy + rmw_zenoh container (router + static_transform_publisher)
# on the host network, then verifies against it from the host:
#   1. ROS -> schiebung: the ROS static transform world -> base (x=1) is
#      retrievable through the schiebung lookup service.
#   2. schiebung -> ROS: dynamic transforms published by the schiebung client
#      are visible to `ros2 topic echo /tf` inside the container.
#   3. Graph interop: the schiebung nodes appear in `ros2 node list`.
#
# Requires: docker (Linux, host networking), rust toolchain.
set -euo pipefail

cd "$(dirname "$0")/../.."

IMAGE=schiebung-ros-interop
CONTAINER=schiebung-ros-interop
ROUTER_ENDPOINT=tcp/127.0.0.1:7447
SERVER_LOG=$(mktemp /tmp/schiebung_interop_server.XXXXXX.log)

GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

SERVER_PID=""
cleanup() {
    [ -n "$SERVER_PID" ] && kill "$SERVER_PID" 2>/dev/null || true
    docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
}
trap cleanup EXIT

fail() {
    echo -e "${RED}FAIL: $1${NC}"
    echo "--- schiebung server log ---"
    tail -30 "$SERVER_LOG" || true
    echo "--- container log ---"
    docker logs --tail 30 "$CONTAINER" 2>&1 || true
    exit 1
}

in_ros() {
    docker exec "$CONTAINER" bash -c \
        "source /opt/ros/jazzy/setup.bash && export RMW_IMPLEMENTATION=rmw_zenoh_cpp && $1"
}

echo -e "${BLUE}[1/6] Building ROS 2 Jazzy + rmw_zenoh image...${NC}"
docker build -q -t "$IMAGE" -f comms/docker/Dockerfile.ros comms/docker

echo -e "${BLUE}[2/6] Building schiebung server + interop check...${NC}"
cargo build -q -p comms --bin server --example ros_interop_check

echo -e "${BLUE}[3/6] Starting ROS container (rmw_zenohd + static_transform_publisher)...${NC}"
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
docker run -d --name "$CONTAINER" --network host "$IMAGE" >/dev/null
for _ in $(seq 1 30); do
    (exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null && { exec 3>&- 3<&-; break; }
    sleep 1
done
(exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null || fail "rmw_zenohd router did not come up on port 7447"
exec 3>&- 3<&- 2>/dev/null || true

echo -e "${BLUE}[4/6] Starting schiebung server (connected to the router)...${NC}"
SCHIEBUNG_CONNECT=$ROUTER_ENDPOINT SCHIEBUNG_MULTICAST=false RUST_LOG=info \
    ./target/debug/server >"$SERVER_LOG" 2>&1 &
SERVER_PID=$!
sleep 2
kill -0 "$SERVER_PID" 2>/dev/null || fail "schiebung server exited early"

echo -e "${BLUE}[5/6] Check 1 -- ROS -> schiebung (lookup of ROS static transform)...${NC}"
SCHIEBUNG_CONNECT=$ROUTER_ENDPOINT SCHIEBUNG_MULTICAST=false \
    ./target/debug/examples/ros_interop_check lookup \
    || fail "lookup of the ROS-published static transform failed"
echo -e "${GREEN}  ROS -> schiebung OK${NC}"

echo -e "${BLUE}[6/6] Check 2 -- schiebung -> ROS (/tf visible to ros2 topic echo)...${NC}"
SCHIEBUNG_CONNECT=$ROUTER_ENDPOINT SCHIEBUNG_MULTICAST=false \
    ./target/debug/examples/ros_interop_check publish 30 &
PUBLISH_PID=$!
ECHO_OUT=$(in_ros "timeout 25 ros2 topic echo /tf --once" || true)
kill "$PUBLISH_PID" 2>/dev/null || true
wait "$PUBLISH_PID" 2>/dev/null || true
echo "$ECHO_OUT" | grep -q "child_frame_id: sensor" \
    || fail "ros2 topic echo /tf never saw the schiebung-published transform"
echo -e "${GREEN}  schiebung -> ROS OK${NC}"

echo -e "${BLUE}Extra -- graph interop (ros2 node list)...${NC}"
NODES=$(in_ros "timeout 20 ros2 node list" || true)
echo "$NODES" | sed 's/^/  /'
echo "$NODES" | grep -q "schiebung_server" \
    || fail "schiebung_server not visible in ros2 node list"
echo -e "${GREEN}  graph interop OK${NC}"

echo -e "${GREEN}ALL ROS INTEROP CHECKS PASSED${NC}"
