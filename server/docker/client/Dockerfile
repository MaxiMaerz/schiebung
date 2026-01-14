FROM python:3.12-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# Copy wheel from target/wheels (context is workspace root)
COPY target/wheels/schiebung_server-*.whl /tmp/

# Install wheel
RUN pip install /tmp/schiebung_server-*.whl

# Copy script
COPY server/docker/client/client.py .

CMD ["python", "client.py", "--help"]
