FROM python:3.12-slim

WORKDIR /app

# Install system dependencies
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     gcc \
#     && rm -rf /var/lib/apt/lists/*

# Copy wheel from target/wheels (context is workspace root)
COPY target/wheels/schiebung_server-*.whl /tmp/

# Install wheel
RUN pip install /tmp/schiebung_server-*.whl "rerun-sdk==0.28.1"

# Copy script
COPY server/docker/server/server.py .

CMD ["python", "server.py"]
