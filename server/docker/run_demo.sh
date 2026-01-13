#!/bin/bash
set -e

# Setup colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== Schiebung Docker Demo ===${NC}"

# 1. Build wheels
echo -e "${GREEN}[1/3] Building Python wheels...${NC}"
cd ../schiebung-server-py
if maturin build --release; then
    echo -e "${GREEN}✓ Wheels built successfully${NC}"
else
    echo -e "${RED}✗ Wheel build failed${NC}"
    exit 1
fi
cd ../docker

# 2. Launch Docker Demo
echo -e "${GREEN}[2/2] Launching Docker Demo components...${NC}"
echo -e "${BLUE}Starting Rerun Viewer, Server, and Clients...${NC}"
echo -e "${BLUE}Access Rerun Viewer at http://localhost:9090${NC}"

# Launch everything
docker compose up --build
