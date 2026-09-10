import threading

import pytest

from openpi_client import websocket_client_policy


class _FakePacker:
    def pack(self, value):
        del value
        return b"request"


class _FakeWebsocket:
    def __init__(self, responses=(b"response",)):
        self.responses = list(responses)
        self.closed = False
        self.recv_timeouts = []

    def send(self, value):
        assert value == b"request"

    def recv(self, timeout=None):
        self.recv_timeouts.append(timeout)
        return self.responses.pop(0)

    def close(self):
        self.closed = True


def _policy(websocket, *, request_timeout_s=None):
    policy = object.__new__(websocket_client_policy.WebsocketClientPolicy)
    policy._packer = _FakePacker()
    policy._ws = websocket
    policy._server_metadata = {"session": 1}
    policy._request_timeout_s = request_timeout_s
    policy._connection_lock = threading.Lock()
    return policy


def test_infer_reports_client_transport_segments(monkeypatch):
    websocket = _FakeWebsocket()
    policy = _policy(websocket)
    monkeypatch.setattr(
        websocket_client_policy.msgpack_numpy,
        "unpackb",
        lambda value: {
            "actions": [],
            "server_timing": {
                "request_deserialization_ms": 0.1,
                "queue_ms": 0.2,
                "infer_ms": 0.3,
                "response_serialization_ms": 0.1,
            },
        },
    )

    result = policy.infer({"observation": 1})

    timing = result["client_timing"]
    assert timing["request_serialization_ms"] >= 0.0
    assert timing["transport_round_trip_ms"] >= 0.0
    assert timing["network_round_trip_estimate_ms"] >= 0.0
    assert timing["response_decode_ms"] >= 0.0
    assert timing["total_ms"] >= timing["transport_round_trip_ms"]
    assert websocket.recv_timeouts == [None]


def test_infer_timeout_closes_only_the_timed_out_connection():
    class _TimeoutWebsocket(_FakeWebsocket):
        def recv(self, timeout=None):
            self.recv_timeouts.append(timeout)
            raise TimeoutError

    websocket = _TimeoutWebsocket()
    policy = _policy(websocket, request_timeout_s=2.0)

    with pytest.raises(TimeoutError, match="2.000s"):
        policy.infer({"observation": 1})

    assert websocket.closed
    assert websocket.recv_timeouts == [2.0]
    assert policy._ws is None


def _capture_error(errors, policy):
    try:
        policy.infer({"observation": 1})
    except BaseException as exc:
        errors.append(exc)


def test_close_unblocks_recv_in_another_thread():
    entered = threading.Event()

    class _BlockingWebsocket(_FakeWebsocket):
        def recv(self, timeout=None):
            entered.set()
            while not self.closed:
                threading.Event().wait(0.001)
            raise RuntimeError("connection closed")

    websocket = _BlockingWebsocket()
    policy = _policy(websocket, request_timeout_s=2.0)
    errors = []
    worker = threading.Thread(target=lambda: _capture_error(errors, policy))
    worker.start()
    assert entered.wait(1.0)

    policy.close()
    worker.join(1.0)

    assert not worker.is_alive()
    assert websocket.closed
    assert isinstance(errors[0], RuntimeError)


def test_reconnect_closes_old_connection_and_refreshes_metadata():
    old = _FakeWebsocket()
    new = _FakeWebsocket()
    policy = _policy(old)
    policy._wait_for_server = lambda: (new, {"session": 2})

    metadata = policy.reconnect()

    assert old.closed
    assert policy._ws is new
    assert metadata == {"session": 2}
    assert policy.get_server_metadata() == {"session": 2}


def test_constructor_passes_connect_timeout_to_open_and_metadata_recv(monkeypatch):
    websocket = _FakeWebsocket(responses=(b"metadata",))
    connect_kwargs = []
    monkeypatch.setattr(
        websocket_client_policy.websockets.sync.client,
        "connect",
        lambda *args, **kwargs: connect_kwargs.append(kwargs) or websocket,
    )
    monkeypatch.setattr(websocket_client_policy.msgpack_numpy, "unpackb", lambda value: {"ok": value})

    policy = websocket_client_policy.WebsocketClientPolicy(
        "localhost", 8000, connect_timeout_s=3.0, request_timeout_s=2.0
    )

    assert 0 < connect_kwargs[0]["open_timeout"] <= 3.0
    assert websocket.recv_timeouts[0] <= 3.0
    assert policy.get_server_metadata() == {"ok": b"metadata"}


def test_connect_timeout_stops_retrying(monkeypatch):
    policy = object.__new__(websocket_client_policy.WebsocketClientPolicy)
    policy._uri = "ws://localhost:8000"
    policy._api_key = None
    policy._connect_timeout_s = 1.0
    monotonic_values = iter((0.0, 0.0, 1.1))
    monkeypatch.setattr(websocket_client_policy.time, "monotonic", lambda: next(monotonic_values))
    monkeypatch.setattr(
        websocket_client_policy.websockets.sync.client,
        "connect",
        lambda *args, **kwargs: (_ for _ in ()).throw(ConnectionRefusedError()),
    )

    with pytest.raises(TimeoutError, match="Timed out connecting"):
        policy._wait_for_server()


def test_constructor_defaults_preserve_unbounded_legacy_behavior(monkeypatch):
    websocket = _FakeWebsocket(responses=(b"metadata", b"response"))
    connect_kwargs = []
    monkeypatch.setattr(
        websocket_client_policy.websockets.sync.client,
        "connect",
        lambda *args, **kwargs: connect_kwargs.append(kwargs) or websocket,
    )
    monkeypatch.setattr(websocket_client_policy.msgpack_numpy, "unpackb", lambda value: {"value": value})

    policy = websocket_client_policy.WebsocketClientPolicy("localhost", 8000)
    policy._packer = _FakePacker()
    result = policy.infer({"observation": 1})

    assert connect_kwargs[0]["open_timeout"] is None
    assert websocket.recv_timeouts == [None, None]
    assert result["value"] == b"response"
