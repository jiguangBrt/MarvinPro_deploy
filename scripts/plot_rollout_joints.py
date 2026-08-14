#!/usr/bin/env python3
"""Plot measured arm joints, gripper state proxies, commands, phase, and tracking error."""

from __future__ import annotations

import argparse
import ast
import csv
import math
from pathlib import Path
import re
import statistics

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


_STATE_RE = re.compile(
    r"bridge_state .*?source=(?P<source>[0-9.]+).*?phase=(?P<phase>[^ ]+) "
    r"phase_rate=(?P<phase_rate>[^ ]+) tracking_error=(?P<error>[^ ]+) "
    r".*?joints=(?P<joints>\([^)]*\)) raw_reference=(?P<reference>\([^)]*\)|None)"
    r"(?: sent_target=(?P<command>\([^)]*\)|None))?"
)
_EVENT_RE = re.compile(
    r"bridge_event type=(?P<event>[^ ]+).*?joint_source=(?P<source>[0-9.]+)"
)

_JOINT_NAMES = tuple(
    [f"Joint{i}_L" for i in range(1, 8)]
    + [f"Joint{i}_R" for i in range(1, 8)]
)
_ACTION_NAMES = _JOINT_NAMES[:7] + ("Gripper_L",) + _JOINT_NAMES[7:] + ("Gripper_R",)


def _float(value: str | None) -> float | None:
    if value in (None, "", "None"):
        return None
    return float(value)


def _tuple(value: str | None) -> tuple[float, ...] | None:
    if value in (None, "", "None"):
        return None
    parsed = ast.literal_eval(value)
    return tuple(float(item) for item in parsed)


def _csv_vector(row: dict[str, str], prefix: str, names: tuple[str, ...]) -> tuple[float, ...] | None:
    values = [row.get(f"{prefix}_{name}", "") for name in names]
    if not values or all(value == "" for value in values):
        return None
    if any(value == "" for value in values):
        return None
    return tuple(float(value) for value in values)


def _empty_data() -> dict[str, list]:
    return {
        "measured": [],
        "command": [],
        "reference": [],
        "gripper_measured": [],
        "gripper_proxy": [],
        "gripper_torque": [],
        "phase": [],
        "events": [],
    }


def _load_events(path: Path | None, *, clock_offset: float = 0.0) -> list[tuple[float, str]]:
    if path is None or not path.exists():
        return []
    events: list[tuple[float, str]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        event = _EVENT_RE.search(line)
        if event:
            events.append((float(event.group("source")) + clock_offset, event.group("event")))
    return events


def _load_csv(path: Path, event_log: Path | None) -> dict[str, list]:
    data = _empty_data()
    clock_offsets: list[float] = []
    with path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            recorded = _float(row.get("recorded_monotonic"))
            if recorded is None:
                continue
            measured = _csv_vector(row, "measured", _JOINT_NAMES)
            record_type = row.get("record_type")
            if record_type == "bridge_state":
                if measured is not None:
                    data["measured"].append((recorded, measured))
                gripper_measured = (
                    _float(row.get("measured_gripper_position_L")),
                    _float(row.get("measured_gripper_position_R")),
                )
                if all(value is not None for value in gripper_measured):
                    data["gripper_measured"].append((recorded, gripper_measured))
                gripper_proxy = (
                    _float(row.get("gripper_command_proxy_L")),
                    _float(row.get("gripper_command_proxy_R")),
                )
                if all(value is not None for value in gripper_proxy):
                    data["gripper_proxy"].append((recorded, gripper_proxy))
                gripper_torque = (
                    _float(row.get("measured_gripper_torque_L")),
                    _float(row.get("measured_gripper_torque_R")),
                )
                if all(value is not None for value in gripper_torque):
                    data["gripper_torque"].append((recorded, gripper_torque))
                sampled = _float(row.get("sampled_monotonic"))
                if sampled is not None:
                    clock_offsets.append(recorded - sampled)
                command = _csv_vector(row, "bridge_command", _ACTION_NAMES)
                reference = _csv_vector(row, "raw_reference", _ACTION_NAMES)
                phase = _float(row.get("phase"))
                phase_rate = _float(row.get("phase_rate"))
                error = _float(row.get("tracking_error_rad"))
                data["phase"].append((recorded, phase, phase_rate, error))
            else:
                command = _csv_vector(row, "client_command", _ACTION_NAMES)
                reference = _csv_vector(row, "client_reference", _ACTION_NAMES)
            if command is not None:
                data["command"].append((recorded, command))
            if reference is not None:
                data["reference"].append((recorded, reference))

    if event_log is None and path.name.endswith(".telemetry.csv"):
        event_log = path.with_name(f"{path.name.removesuffix('.telemetry.csv')}.log")
    clock_offset = statistics.median(clock_offsets) if clock_offsets else 0.0
    data["events"] = _load_events(event_log, clock_offset=clock_offset)
    return data


def _load_structured_log(path: Path) -> dict[str, list]:
    data = _empty_data()
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        state = _STATE_RE.search(line)
        if state:
            joints = _tuple(state.group("joints"))
            if joints is None or len(joints) != 14:
                continue
            source = float(state.group("source"))
            reference = _tuple(state.group("reference"))
            command = _tuple(state.group("command")) or reference
            data["measured"].append((source, joints))
            if command is not None:
                data["command"].append((source, command))
            if reference is not None:
                data["reference"].append((source, reference))
            data["phase"].append(
                (
                    source,
                    _float(state.group("phase")),
                    _float(state.group("phase_rate")),
                    _float(state.group("error")),
                )
            )
        event = _EVENT_RE.search(line)
        if event:
            data["events"].append((float(event.group("source")), event.group("event")))
    return data


def _load(path: Path, event_log: Path | None = None) -> dict[str, list]:
    if path.suffix.lower() == ".csv":
        data = _load_csv(path, event_log)
    else:
        data = _load_structured_log(path)
    if not data["measured"]:
        raise SystemExit(
            f"{path} has no measured-joint telemetry. Re-run with --log-file or --telemetry-file."
        )
    if not data["command"]:
        raise SystemExit(f"{path} has measured joints but no interpolated command telemetry.")
    return data


def _action_arm_index(joint_index: int) -> int:
    return joint_index if joint_index < 7 else joint_index + 1


def plot(input_path: Path, output_path: Path, event_log: Path | None = None) -> None:
    data = _load(input_path, event_log)
    all_times = [time_s for key in ("measured", "command") for time_s, _ in data[key]]
    origin = min(all_times)

    figure = plt.figure(figsize=(17, 15), constrained_layout=True)
    grid = GridSpec(5, 4, figure=figure, height_ratios=(1, 1, 1, 1, 1.15))
    for index, name in enumerate(_JOINT_NAMES):
        axis = figure.add_subplot(grid[index // 4, index % 4])
        measured_time = [time_s - origin for time_s, _ in data["measured"]]
        measured = [values[index] for _, values in data["measured"]]
        axis.plot(measured_time, measured, color="#1769aa", linewidth=1.1, label="measured")

        action_index = _action_arm_index(index)
        command_time = [time_s - origin for time_s, _ in data["command"]]
        command = [values[action_index] for _, values in data["command"]]
        axis.plot(command_time, command, color="#d95f02", linewidth=1.0, linestyle="--", label="sent command")

        if data["reference"]:
            reference_time = [time_s - origin for time_s, _ in data["reference"]]
            reference = [values[action_index] for _, values in data["reference"]]
            axis.plot(
                reference_time,
                reference,
                color="#636363",
                linewidth=0.7,
                linestyle=":",
                alpha=0.7,
                label="raw reference",
            )
        axis.set_title(name, fontsize=9)
        axis.grid(alpha=0.25)
        axis.tick_params(labelsize=7)
        if index == 0:
            axis.legend(fontsize=7, loc="best")

    for side_index, (name, action_index) in enumerate((("Gripper_L", 7), ("Gripper_R", 15))):
        axis = figure.add_subplot(grid[3, side_index + 2])
        if data["gripper_measured"]:
            measured_time = [time_s - origin for time_s, _ in data["gripper_measured"]]
            measured = [values[side_index] for _, values in data["gripper_measured"]]
            axis.plot(measured_time, measured, color="#1769aa", linewidth=1.1, label="measured position")
        elif data["gripper_proxy"]:
            proxy_time = [time_s - origin for time_s, _ in data["gripper_proxy"]]
            proxy = [values[side_index] for _, values in data["gripper_proxy"]]
            axis.plot(proxy_time, proxy, color="#1769aa", linewidth=1.1, label="command state proxy")
        command_time = [time_s - origin for time_s, _ in data["command"]]
        command = [values[action_index] for _, values in data["command"]]
        axis.plot(command_time, command, color="#d95f02", linewidth=1.0, linestyle="--", label="sent command")
        if data["reference"]:
            reference_time = [time_s - origin for time_s, _ in data["reference"]]
            reference = [values[action_index] for _, values in data["reference"]]
            axis.plot(
                reference_time,
                reference,
                color="#636363",
                linewidth=0.7,
                linestyle=":",
                alpha=0.7,
                label="raw reference",
            )
        axis.set_title(f"{name} (0=open, 1=closed)", fontsize=9)
        axis.set_ylim(-0.05, 1.05)
        axis.grid(alpha=0.25)
        axis.tick_params(labelsize=7)
        handles, labels = axis.get_legend_handles_labels()
        if data["gripper_torque"]:
            torque_axis = axis.twinx()
            torque_time = [time_s - origin for time_s, _ in data["gripper_torque"]]
            torque = [values[side_index] for _, values in data["gripper_torque"]]
            torque_axis.plot(torque_time, torque, color="#238b45", linewidth=0.8, alpha=0.75, label="measured torque")
            torque_axis.set_ylabel("torque", color="#238b45", fontsize=7)
            torque_axis.tick_params(axis="y", colors="#238b45", labelsize=7)
            torque_handles, torque_labels = torque_axis.get_legend_handles_labels()
            handles.extend(torque_handles)
            labels.extend(torque_labels)
        axis.legend(handles, labels, fontsize=7, loc="best")

    phase_axis = figure.add_subplot(grid[4, :2])
    phase_time = [time_s - origin for time_s, _, _, _ in data["phase"]]
    phases = [math.nan if phase is None else phase for _, phase, _, _ in data["phase"]]
    phase_rates = [math.nan if rate is None else rate for _, _, rate, _ in data["phase"]]
    phase_axis.plot(phase_time, phases, color="#2c7fb8", label="phase")
    phase_axis.plot(phase_time, phase_rates, color="#238b45", label="phase_rate")
    phase_axis.set_title("Trajectory phase and governor rate")
    phase_axis.set_xlabel("time since first telemetry sample (s)")
    phase_axis.grid(alpha=0.25)
    phase_axis.legend(fontsize=8)

    error_axis = figure.add_subplot(grid[4, 2:])
    errors = [math.nan if error is None else error for _, _, _, error in data["phase"]]
    error_axis.plot(phase_time, errors, color="#b2182b", label="tracking error (rad)")
    error_axis.axhline(0.01, color="#d6604d", linestyle="--", linewidth=0.8, label="run tolerance")
    error_axis.axhline(0.04, color="#762a83", linestyle=":", linewidth=0.8, label="hard stop")
    error_axis.set_title("Measured tracking error")
    error_axis.set_xlabel("time since first telemetry sample (s)")
    error_axis.set_ylabel("rad")
    error_axis.grid(alpha=0.25)
    error_axis.legend(fontsize=8)

    event_colors = {
        "checkpoint_ready": "#e66101",
        "rtc_resumed": "#5e3c99",
        "rtc_merged": "#1b7837",
        "rtc_waiting_at_deadline": "#b2182b",
        "holding": "#636363",
    }
    for source, event in data["events"]:
        color = event_colors.get(event)
        if color is None:
            continue
        x = source - origin
        for axis in (phase_axis, error_axis):
            axis.axvline(x, color=color, alpha=0.35, linewidth=0.8)
        phase_axis.text(x, 0.98, event, rotation=90, transform=phase_axis.get_xaxis_transform(), fontsize=6)

    figure.suptitle(
        f"MarvinPro joint and gripper telemetry: {input_path.name}\n"
        "Blue=measured arm position and gripper state proxy/legacy measured position, "
        "green=legacy measured gripper torque, "
        "orange dashed=interpolated command sent to the controller, "
        "gray dotted=pre-safety reference",
        fontsize=13,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=160)
    plt.close(figure)
    print(output_path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path, help="rollout.telemetry.csv (preferred) or structured rollout.log")
    parser.add_argument("-o", "--output", type=Path)
    parser.add_argument("--event-log", type=Path, help="structured rollout.log containing bridge_event lines")
    args = parser.parse_args()
    output = args.output or args.input.with_name("joint_diagnostics.png")
    plot(args.input, output, args.event_log)


if __name__ == "__main__":
    main()
