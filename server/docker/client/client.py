import time
import math
import sys
import logging
import argparse
from schiebung_server import TransformClient, StampedIsometry, TransformType

# Configure logging for Docker visibility
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    stream=sys.stdout,
)
logger = logging.getLogger("client")

def main():
    parser = argparse.ArgumentParser(description="Schiebung Client Demo")
    parser.add_argument("--parent", type=str, required=True, help="Parent frame")
    parser.add_argument("--child", type=str, required=True, help="Child frame")
    parser.add_argument("--radius", type=float, required=True, help="Orbit radius")
    parser.add_argument("--period", type=float, required=True, help="Orbit period (seconds)")
    parser.add_argument("--z-offset", type=float, default=0.0, help="Z offset")
    args = parser.parse_args()

    logger.info(f"Starting client: {args.parent} -> {args.child}, radius={args.radius}, period={args.period}")

    try:
        client = TransformClient()
    except Exception as e:
        logger.error(f"Failed to create client: {e}")
        sys.exit(1)

    # Simulation loop
    start_time = time.time()

    try:
        while True:
            current_time = time.time() - start_time
            # Convert to nanoseconds (int)
            current_time_ns = int(current_time * 1e9)

            angle = (current_time / args.period) * 2.0 * math.pi
            x = args.radius * math.cos(angle)
            y = args.radius * math.sin(angle)

            transform = StampedIsometry(
                [x, y, args.z_offset],
                [0.0, 0.0, 0.0, 1.0],  # No rotation for simplicity
                current_time_ns
            )

            try:
                client.send_transform(
                    args.parent,
                    args.child,
                    transform,
                    TransformType.Dynamic
                )
            except Exception as e:
                logger.error(f"Error sending transform: {e}")

            time.sleep(0.01)  # Update at 100Hz

    except KeyboardInterrupt:
        logger.info("Client stopped.")

if __name__ == "__main__":
    main()
