"""Verified Marvin Pro deployment constants.

The joint limits come from the active controller model ``new_m6_696`` and its
M6-S-{L,R}-CCS-696-V4 URDF on 6.6.7.100 (checked 2026-08-05).
"""

from __future__ import annotations

PROTOCOL_VERSION = 9
DEFAULT_BRIDGE_HOST = "6.6.7.100"
DEFAULT_BRIDGE_PORT = 7332
DEFAULT_POLICY_HOST = "192.168.50.73"
DEFAULT_POLICY_PORT = 8000
DEFAULT_PROMPT = "Stack all three red cones into one stable stack."

CONTROL_HZ = 15.0
CUSTOM_INPUT_MODE = 3
# On this controller, /tj/info/{robot,arm}_state carries the active control mode.
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
# Both active M6-S-{L,R}-CCS-696-V4 URDFs specify 3.1416 rad/s for
# Joint1..Joint7. Deployment may impose a lower RTC blend cap.
JOINT_VELOCITY = (3.1416,) * 14

# Apex currently launches the robot service with ``apex_ros_namespace:=tj``.
# Camera topics remain at the root, while robot/control topics are under /tj.
TOPIC_JOINT_STATES = "/tj/joint_states"
# DM driver layout and dynamic feedback verified on the live /tj topics:
# [position_rad, velocity_rad_s, torque, mos_temperature, motor_temperature].
# Keep these endpoints aligned with the calibration used to build the training
# dataset; feedback position is normalized to the policy's 0=open, 1=closed.
GRIPPER_OPEN_RAW = 0.0
GRIPPER_CLOSED_RAW = 1.25
TOPIC_GRIPPER_FEEDBACK_L = "/tj/info/gripper_feedback_L"
TOPIC_GRIPPER_FEEDBACK_R = "/tj/info/gripper_feedback_R"
TOPIC_QUAD_IMAGE = "/quad_tile/compressed"
TOPIC_INPUT_MODE = "/tj/control/input_mode"
TOPIC_ROBOT_STATE = "/tj/info/robot_state"
TOPIC_ARM_STATE = "/tj/info/arm_state"
TOPIC_USER_CMD_L = "/tj/control/user/joint_cmd_A"
TOPIC_USER_CMD_R = "/tj/control/user/joint_cmd_B"
TOPIC_GRIPPER_CMD_L = "/tj/control/gripperValueL"
TOPIC_GRIPPER_CMD_R = "/tj/control/gripperValueR"
