"""Verified Marvin Pro deployment constants.

The joint limits come from the active controller model ``new_m6_696`` and its
M6-S-{L,R}-CCS-696-V4 URDF on 6.6.7.100 (checked 2026-08-05).
"""

from __future__ import annotations

PROTOCOL_VERSION = 1
DEFAULT_BRIDGE_HOST = "6.6.7.100"
DEFAULT_BRIDGE_PORT = 7332
DEFAULT_POLICY_HOST = "192.168.50.73"
DEFAULT_POLICY_PORT = 8000
DEFAULT_PROMPT = "Stack all three red cones into one stable stack."

CONTROL_HZ = 15.0
CUSTOM_INPUT_MODE = 3
# On this controller, /info/{robot,arm}_state carries the active control mode.
# Mode 3 is joint impedance; verified against apex_backend/ros_state.py and the
# live topics on 2026-08-05. Requiring 3 specifically prevents motion in Idle
# (0), position (1), or other non-impedance modes.
READY_STATE = (3, 3)

JOINT_NAMES = tuple(
    [f"Joint{i}_L" for i in range(1, 8)]
    + [f"Joint{i}_R" for i in range(1, 8)]
)

# Canonical order: L1..L7, R1..R7.
JOINT_LOWER = (
    -3.1067,
    -2.0944,
    -3.1067,
    -2.5307,
    -3.1067,
    -1.0472,
    -1.5708,
    -3.1067,
    -2.0944,
    -3.1067,
    -2.5307,
    -3.1067,
    -1.0472,
    -1.5708,
)
JOINT_UPPER = (
    3.1067,
    2.0944,
    3.1067,
    1.0472,
    3.1067,
    1.0472,
    1.5708,
    3.1067,
    2.0944,
    3.1067,
    1.0472,
    3.1067,
    1.0472,
    1.5708,
)

# Training converter calibration: feedback raw 0=open, 1.25=closed.
GRIPPER_OPEN_RAW = 0.0
GRIPPER_CLOSED_RAW = 1.25

TOPIC_JOINT_STATES = "/joint_states"
TOPIC_GRIPPER_FEEDBACK_L = "/info/gripper_feedback_L"
TOPIC_GRIPPER_FEEDBACK_R = "/info/gripper_feedback_R"
TOPIC_QUAD_IMAGE = "/quad_tile/compressed"
TOPIC_INPUT_MODE = "/control/input_mode"
TOPIC_ROBOT_STATE = "/info/robot_state"
TOPIC_ARM_STATE = "/info/arm_state"
TOPIC_USER_CMD_L = "/control/user/joint_cmd_A"
TOPIC_USER_CMD_R = "/control/user/joint_cmd_B"
TOPIC_GRIPPER_CMD_L = "/control/gripperValueL"
TOPIC_GRIPPER_CMD_R = "/control/gripperValueR"
