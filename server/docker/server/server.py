import rerun as rr
import time
import os
import sys
import logging
import argparse
from schiebung_server import Server

# Configure logging for Docker visibility
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    stream=sys.stdout,
)
logger = logging.getLogger("server")

def main():
    parser = argparse.ArgumentParser(description="Schiebung Server Demo")
    parser.add_argument("--application-id", type=str, default="schiebung", help="Rerun application ID")
    parser.add_argument("--recording-id", type=str, default="sun_earth_moon_demo", help="Rerun recording ID")
    parser.add_argument("--timeline", type=str, default="stable_time", help="Timeline name")
    parser.add_argument("--grpc-port", type=int, default=9876, help="gRPC server port")
    parser.add_argument("--web-port", type=int, default=9090, help="Web viewer port")
    args = parser.parse_args()

    # Initialize Rerun SDK
    rr.init(args.application_id, recording_id=args.recording_id, spawn=False)

    logger.info("Rerun initialized with application ID: %s, recording ID: %s", args.application_id, args.recording_id)

    # Start the gRPC server - this returns the URI that clients should connect to
    grpc_uri = rr.serve_grpc(grpc_port=args.grpc_port, server_memory_limit="2GB")
    logger.info(f"Rerun gRPC server started at: {grpc_uri}")

    # Start the web viewer, connecting it to our gRPC server
    rr.serve_web_viewer(
        connect_to=grpc_uri,
        web_port=args.web_port,
        open_browser=False,  # Don't try to open browser in Docker
    )
    logger.info(f"Rerun web viewer available at: http://0.0.0.0:{args.web_port}")

    # Set the env var so the Rust Server connects to our gRPC server
    os.environ["RERUN_CONNECT_ADDR"] = grpc_uri

    server = Server(args.application_id, args.recording_id, args.timeline, True)
    server_handle = server.start()

    # Log static geometry (Sun, Earth, Moon visual representation)
    # Note: Server does not publish these, it only visualizes transforms.
    # We use standard Rerun logging here.
    rr.set_time(args.timeline, timestamp=0)


    rr.log(
        "Sun",
        rr.Ellipsoids3D(half_sizes=[[0.15, 0.15, 0.15]], colors=[[255, 200, 0]]),
        rr.CoordinateFrame("Sun"),
    )

    rr.log(
        "Earth",
        rr.Ellipsoids3D(half_sizes=[[0.08, 0.08, 0.08]], colors=[[50, 100, 200]]),
        rr.CoordinateFrame("Earth"),
    )

    rr.log(
        "Moon",
        rr.Ellipsoids3D(half_sizes=[[0.04, 0.04, 0.04]], colors=[[180, 180, 180]]),
        rr.CoordinateFrame("Moon"),
    )

    logger.info("Static geometry logged. Starting Server...")

    try:
        while True:
            iso = server.buffer.lookup_latest_transform("Moon", "Sun")
            t = iso.translation()  # translation() is a method, returns [x, y, z]

            rr.set_time(args.timeline, timestamp=iso.stamp_secs())
            rr.log(
                "Moon_to_Sun",
                rr.Arrows3D(
                    origins=[[0,0,0]],
                    vectors=[[t[0], t[1], t[2]]],
                    colors=[[255, 0, 0]]
                ),
                rr.CoordinateFrame("Moon"),
                )

            time.sleep(0.01)
    except KeyboardInterrupt:
        server_handle.shutdown()
        server_handle.join()
        logger.info("Server stopped.")

if __name__ == "__main__":
    main()
