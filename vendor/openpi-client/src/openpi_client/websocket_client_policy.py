import logging
import threading
import time
from typing import Dict, Optional, Tuple

from typing_extensions import override
from websockets.exceptions import ConnectionClosed
import websockets.sync.client

from openpi_client import base_policy as _base_policy
from openpi_client import msgpack_numpy


class WebsocketClientPolicy(_base_policy.BasePolicy):
    """Implements the Policy interface by communicating with a server over websocket.

    See WebsocketPolicyServer for a corresponding server implementation.
    """

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: Optional[int] = None,
        api_key: Optional[str] = None,
        connect_timeout_s: Optional[float] = None,
        request_timeout_s: Optional[float] = None,
    ) -> None:
        if host.startswith("ws"):
            self._uri = host
        else:
            self._uri = f"ws://{host}"
        if port is not None:
            self._uri += f":{port}"
        self._packer = msgpack_numpy.Packer()
        self._api_key = api_key
        if connect_timeout_s is not None and connect_timeout_s <= 0:
            raise ValueError("connect_timeout_s must be positive")
        if request_timeout_s is not None and request_timeout_s <= 0:
            raise ValueError("request_timeout_s must be positive")
        self._connect_timeout_s = connect_timeout_s
        self._request_timeout_s = request_timeout_s
        self._connection_lock = threading.Lock()
        self._ws = None
        self._server_metadata = {}
        self.reconnect()

    def get_server_metadata(self) -> Dict:
        with self._connection_lock:
            return self._server_metadata

    def _wait_for_server(self) -> Tuple[websockets.sync.client.ClientConnection, Dict]:
        logging.info(f"Waiting for server at {self._uri}...")
        deadline = None
        if self._connect_timeout_s is not None:
            deadline = time.monotonic() + self._connect_timeout_s
        while True:
            remaining = None if deadline is None else deadline - time.monotonic()
            if remaining is not None and remaining <= 0:
                raise TimeoutError(f"Timed out connecting to policy server at {self._uri}")
            conn = None
            try:
                headers = {"Authorization": f"Api-Key {self._api_key}"} if self._api_key else None
                conn = websockets.sync.client.connect(
                    self._uri,
                    compression=None,
                    max_size=None,
                    additional_headers=headers,
                    open_timeout=remaining,
                )
                metadata = msgpack_numpy.unpackb(conn.recv(timeout=remaining))
                return conn, metadata
            except (ConnectionRefusedError, ConnectionClosed, OSError, TimeoutError) as exc:
                if conn is not None:
                    conn.close()
                if deadline is not None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError(f"Timed out connecting to policy server at {self._uri}") from exc
                logging.info("Still waiting for server...")
                time.sleep(5 if remaining is None else min(5, remaining))

    def _close_connection(self, conn) -> None:
        with self._connection_lock:
            if self._ws is conn:
                self._ws = None
        conn.close()

    def close(self) -> None:
        """Close the active connection, including a recv blocked in another thread."""
        with self._connection_lock:
            conn = self._ws
            self._ws = None
        if conn is not None:
            conn.close()

    def reconnect(self) -> Dict:
        """Replace the active connection and return metadata from the new server session."""
        self.close()
        conn, metadata = self._wait_for_server()
        with self._connection_lock:
            self._ws = conn
            self._server_metadata = metadata
        return metadata

    @override
    def infer(self, obs: Dict) -> Dict:  # noqa: UP006
        with self._connection_lock:
            conn = self._ws
        if conn is None:
            raise RuntimeError("Policy connection is closed")
        started = time.monotonic()
        serialization_started = started
        data = self._packer.pack(obs)
        request_serialization_ms = (time.monotonic() - serialization_started) * 1000.0
        transport_started = time.monotonic()
        try:
            conn.send(data)
            response = conn.recv(timeout=self._request_timeout_s)
        except TimeoutError as exc:
            self._close_connection(conn)
            timeout = self._request_timeout_s
            detail = "" if timeout is None else f" after {timeout:.3f}s"
            raise TimeoutError(f"Policy request timed out{detail}") from exc
        except ConnectionClosed as exc:
            self._close_connection(conn)
            raise ConnectionError("Policy connection closed during inference") from exc
        transport_round_trip_ms = (time.monotonic() - transport_started) * 1000.0
        if isinstance(response, str):
            # we're expecting bytes; if the server sends a string, it's an error.
            raise RuntimeError(f"Error in inference server:\n{response}")
        decode_started = time.monotonic()
        result = msgpack_numpy.unpackb(response)
        response_decode_ms = (time.monotonic() - decode_started) * 1000.0
        if isinstance(result, dict):
            server_timing = result.get("server_timing", {})
            server_path_ms = 0.0
            if isinstance(server_timing, dict):
                for key in (
                    "request_deserialization_ms",
                    "queue_ms",
                    "infer_ms",
                    "response_serialization_ms",
                ):
                    value = server_timing.get(key)
                    if isinstance(value, (int, float)):
                        server_path_ms += max(0.0, float(value))
            result["client_timing"] = {
                "request_serialization_ms": request_serialization_ms,
                "transport_round_trip_ms": transport_round_trip_ms,
                "network_round_trip_estimate_ms": max(0.0, transport_round_trip_ms - server_path_ms),
                "response_decode_ms": response_decode_ms,
                "total_ms": (time.monotonic() - started) * 1000.0,
            }
        return result

    @override
    def reset(self) -> None:
        pass
