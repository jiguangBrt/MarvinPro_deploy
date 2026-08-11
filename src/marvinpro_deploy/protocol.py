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
    # Raw DM motor position feedback in radians. The client applies the
    # training calibration before constructing the 16-dimensional state.
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
    gripper_velocity_left: float | None = None
    gripper_velocity_right: float | None = None
    gripper_torque_left: float | None = None
    gripper_torque_right: float | None = None
    gripper_mos_temperature_left: float | None = None
    gripper_mos_temperature_right: float | None = None
    gripper_motor_temperature_left: float | None = None
    gripper_motor_temperature_right: float | None = None


@dataclass(frozen=True)
class RobotStateUpdate:
    state_seq: int
    sampled_monotonic: float
    joints: tuple[float, ...]
    # Raw DM motor position feedback in radians.
    gripper_raw_left: float
    gripper_raw_right: float
    motion_gate_open: bool
    gate_reason: str
    last_command_id: int | None = None
    last_command_status: str = "no command"
    trajectory_mode: str = "legacy"
    session_id: str | None = None
    plan_id: str | None = None
    timeline_version: int = 0
    phase: float | None = None
    phase_rate: float = 0.0
    raw_reference: tuple[float, ...] | None = None
    sent_target: tuple[float, ...] | None = None
    tracking_error_rad: float | None = None
    servo_error_rad: float | None = None
    arm_clipped: bool = False
    frozen_reason: str | None = None
    active_request_id: str | None = None
    gripper_velocity_left: float | None = None
    gripper_velocity_right: float | None = None
    gripper_torque_left: float | None = None
    gripper_torque_right: float | None = None
    gripper_mos_temperature_left: float | None = None
    gripper_mos_temperature_right: float | None = None
    gripper_motor_temperature_left: float | None = None
    gripper_motor_temperature_right: float | None = None
    version: int = PROTOCOL_VERSION


@dataclass(frozen=True)
class TrajectoryEvent:
    event_seq: int
    event_type: str
    emitted_monotonic: float
    session_id: str
    plan_id: str
    timeline_version: int
    phase: float
    checkpoint_id: int | None = None
    stable_monotonic: float | None = None
    observation_seq_at_stable: int | None = None
    old_remaining_actions_absolute: tuple[tuple[float, ...], ...] = ()
    request_id: str | None = None
    predicted_delay_steps: int | None = None
    actual_delay_steps: int | None = None
    detail: str = ""
    version: int = PROTOCOL_VERSION
    tracking_error_rad: float | None = None
    servo_error_rad: float | None = None
    joint_source_monotonic: float | None = None
    settle_duration_s: float | None = None
    raw_reference: tuple[float, ...] | None = None
    sent_target: tuple[float, ...] | None = None
    arm_clipped: bool = False
    frozen_reason: str | None = None
    boundary_old_velocity: tuple[float, ...] = ()
    boundary_new_velocity: tuple[float, ...] = ()
    boundary_velocity_jump_rad: float | None = None
    boundary_acceleration_jump_rad: float | None = None
    continuous_checkpoint: bool = False


@dataclass(frozen=True)
class ActionCommand:
    command_id: int
    observation_seq: int
    action: tuple[float, ...]
    execute: bool
    version: int = PROTOCOL_VERSION


@dataclass(frozen=True)
class LoadTrajectoryCommand:
    command_id: int
    observation_seq: int
    session_id: str
    plan_id: str
    expected_timeline_version: int
    knots: tuple[tuple[float, ...], ...]
    knot_hz: float
    checkpoint_horizon: int
    execute: bool
    continuous_checkpoint: bool = False
    version: int = PROTOCOL_VERSION


@dataclass(frozen=True)
class ResumeTrajectoryCommand:
    command_id: int
    session_id: str
    plan_id: str
    timeline_version: int
    checkpoint_id: int
    request_id: str
    predicted_delay_steps: int
    version: int = PROTOCOL_VERSION


@dataclass(frozen=True)
class StageRtcChunkCommand:
    command_id: int
    session_id: str
    base_plan_id: str
    replacement_plan_id: str
    timeline_version: int
    checkpoint_id: int
    request_id: str
    predicted_delay_steps: int
    execution_horizon: int
    actions: tuple[tuple[float, ...], ...]
    version: int = PROTOCOL_VERSION


@dataclass(frozen=True)
class TrajectoryHeartbeat:
    session_id: str
    timeline_version: int
    version: int = PROTOCOL_VERSION


@dataclass(frozen=True)
class HoldPositionCommand:
    command_id: int
    session_id: str
    expected_timeline_version: int
    action: tuple[float, ...]
    execute: bool
    reason: str = "client requested hold"
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
