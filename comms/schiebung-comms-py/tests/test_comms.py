"""Integration tests for the schiebung_comms Python bindings.

Mirrors the Rust integration test: server and client pinned to a localhost
TCP endpoint with multicast scouting disabled, retrying around startup races
(publishes that land before pub/sub matching are dropped).
"""

import time

import pytest
from schiebung_comms import (
    StampedIsometry,
    TransformClient,
    TransformServer,
    TransformType,
    ZenohConfig,
)

ENDPOINT = "tcp/127.0.0.1:17452"


def publish_and_wait(client, from_frame, to_frame, transform, kind, expect):
    """Publish and poll via lookup until the expectation holds."""
    last_err = None
    for _ in range(20):
        try:
            client.send_transform(from_frame, to_frame, transform, kind)
            time.sleep(0.1)
            result = client.request_transform(from_frame, to_frame, 0)
            if expect(result):
                return result
            last_err = f"unexpected result: {result.translation()}"
        except ValueError as e:  # CommsError is raised as ValueError
            last_err = str(e)
        time.sleep(0.1)
    pytest.fail(f"publish_and_wait({from_frame}, {to_frame}) failed: {last_err}")


@pytest.fixture(scope="module")
def server():
    server = TransformServer(
        ZenohConfig(listen=[ENDPOINT], multicast_scouting=False)
    )
    handle = server.start()
    yield server
    handle.shutdown()
    handle.join()


@pytest.fixture(scope="module")
def client(server):
    return TransformClient(ZenohConfig(connect=[ENDPOINT], multicast_scouting=False))


def test_zenoh_config():
    config = ZenohConfig(listen=["tcp/127.0.0.1:1"], multicast_scouting=False, domain_id=7)
    assert config.mode == "peer"
    assert config.listen == ["tcp/127.0.0.1:1"]
    assert config.connect == []
    assert config.multicast_scouting is False
    assert config.domain_id == 7
    assert "ZenohConfig" in repr(config)


def test_static_transform_roundtrip(client):
    t = StampedIsometry([0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0], 0)
    result = publish_and_wait(
        client, "world", "robot_base", t, TransformType.Static,
        lambda r: abs(r.translation()[2] - 1.0) < 1e-6,
    )
    assert abs(result.translation()[2] - 1.0) < 1e-6


def test_dynamic_transform_roundtrip(client):
    t = StampedIsometry([0.5, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 1_000_000_000)
    result = publish_and_wait(
        client, "robot_base", "tool", t, TransformType.Dynamic,
        lambda r: abs(r.translation()[0] - 0.5) < 1e-6,
    )
    assert abs(result.translation()[0] - 0.5) < 1e-6


def test_composed_lookup(client):
    # Depends on the two transforms published by the previous tests.
    test_static_transform_roundtrip(client)
    test_dynamic_transform_roundtrip(client)
    result = client.request_transform("world", "tool", 0)
    assert abs(result.translation()[0] - 0.5) < 1e-6
    assert abs(result.translation()[2] - 1.0) < 1e-6


def test_unknown_frame_errors(client):
    with pytest.raises(ValueError):
        client.request_transform("world", "nonexistent", 0)


def test_server_buffer_access(server, client):
    test_static_transform_roundtrip(client)
    result = server.buffer.lookup_latest_transform("world", "robot_base")
    assert abs(result.translation()[2] - 1.0) < 1e-6
    assert "world" in server.buffer.visualize()


def test_request_all_transforms(client):
    test_static_transform_roundtrip(client)
    test_dynamic_transform_roundtrip(client)
    edges = {
        (from_frame, to_frame): (transform, kind)
        for from_frame, to_frame, transform, kind in client.request_all_transforms()
    }
    static_t, static_kind = edges[("world", "robot_base")]
    assert static_kind == TransformType.Static
    assert abs(static_t.translation()[2] - 1.0) < 1e-6
    dynamic_t, dynamic_kind = edges[("robot_base", "tool")]
    assert dynamic_kind == TransformType.Dynamic
    assert abs(dynamic_t.translation()[0] - 0.5) < 1e-6


def test_buffer_latest_transforms(server, client):
    test_static_transform_roundtrip(client)
    edges = {
        (from_frame, to_frame): kind
        for from_frame, to_frame, _transform, kind in server.buffer.latest_transforms()
    }
    assert edges[("world", "robot_base")] == TransformType.Static
