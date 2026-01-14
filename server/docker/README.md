# Docker Demo

This directory contains a complete Docker-based demo of the Schiebung system.

## Prerequisites

1. **Build Python Wheels**: The Docker build requires the Python wheels to be present in `target/wheels`.

   ```bash
   # From workspace root
   cd server/schiebung-server-py
   maturin build --release
   ```

   Note: Using `--release` is recommended for performance, but `maturin build` works too.

## Running the Demo

1. **Start Containers**:

   ```bash
   # From server/docker directory
   docker compose up --build
   ```

2. **Visualize**:
   Open the Rerun Viewer at [http://localhost:9090](http://localhost:9090).

## Components

- **Server**: Hosts the Rerun gRPC server and web viewer, manages the transform buffer, and logs the static geometry of the solar system.
- **Client (Earth)**: Simulates Earth's orbit around the Sun.
- **Client (Moon)**: Simulates Moon's orbit around the Earth.
- **run_demo.sh**: helper script to build and run everything.
