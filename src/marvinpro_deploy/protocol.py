"""Framed, bidirectional bridge protocol.

Pickle is deliberately used so the Python 3.10 ROS process can transport JPEG
bytes without extra dependencies. It is unsafe on an untrusted network: only
expose this bridge on the private robot LAN.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import pickle
import socket
import struct
import threading
from typing import Any

from .config import PROTOCOL_VERSION

_HEADER = struct.Struct(">I")
MAX_MESSAGE_BYTES = 32 * 1024 * 1024


class ProtocolError(RuntimeError):
    pass


@dataclass(frozen=True)
class BridgeHello:
    version: int = PROTOCOL_VERSION
    motion_allowed: bool = False
    publish_hz: float = 15.0
    max_joint_step_rad: float = 0.12
    joint_lower: tuple[float, ...] = ()
    joint_upper: tuple[float, ...] = ()


@dataclass(frozen=True)
class RobotObservation:
    seq: int
    captured_monotonic: float
    image: bytes
    image_format: str
    joints: tuple[float, ...]
    gripper_raw_left: float
    gripper_raw_right: float
    input_mode: int | None
    robot_state: tuple[int, ...] | None
    arm_state: tuple[int, ...] | None
    age_state_s: float | None
    age_gripper_left_s: float | None
    age_gripper_right_s: float | None
    age_input_mode_s: float | None
    age_robot_state_s: float | None
    age_arm_state_s: float | None
    motion_gate_open: bool
    gate_reason: str
    last_command_id: int | None = None
    last_command_status: str = "no command"
    version: int = PROTOCOL_VERSION
    extra: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ActionCommand:
    command_id: int
    observation_seq: int
    action: tuple[float, ...]
    execute: bool
    version: int = PROTOCOL_VERSION


@dataclass(frozen=True)
class StopCommand:
    reason: str = "client requested stop"
    version: int = PROTOCOL_VERSION


def send_message(sock: socket.socket, obj: Any, lock: threading.Lock | None = None) -> None:
    data = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
    if len(data) > MAX_MESSAGE_BYTES:
        raise ProtocolError(f"message is too large: {len(data)} bytes")
    packet = _HEADER.pack(len(data)) + data
    if lock is None:
        sock.sendall(packet)
    else:
        with lock:
            sock.sendall(packet)


def _recv_exact(sock: socket.socket, size: int) -> bytes | None:
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            return None
        data.extend(chunk)
    return bytes(data)


def recv_message(sock: socket.socket) -> Any | None:
    header = _recv_exact(sock, _HEADER.size)
    if header is None:
        return None
    (size,) = _HEADER.unpack(header)
    if size <= 0 or size > MAX_MESSAGE_BYTES:
        raise ProtocolError(f"invalid framed message size: {size}")
    body = _recv_exact(sock, size)
    if body is None:
        return None
    try:
        return pickle.loads(body)
    except Exception as exc:
        raise ProtocolError(f"cannot decode bridge message: {exc}") from exc


def require_current_version(message: Any) -> None:
    version = getattr(message, "version", None)
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"protocol version mismatch: peer={version}, local={PROTOCOL_VERSION}")
