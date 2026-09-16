#!/usr/bin/env python3
"""Generate a compact post-mortem report from a chassis fault snapshot."""

import argparse
import csv
import math
import os
from pathlib import Path
import tempfile

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TELEMETRY_TOPIC = "/agv2/chassis_telemetry"
EVENT_TOPIC = "/agv2/log_event"
ANALYSIS_WINDOW_NS = 60 * 1_000_000_000
SUSTAIN_NS = 800 * 1_000_000


def read_bag(uri: Path):
    """Read telemetry and manual event markers from a rosbag2 directory."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic in (TELEMETRY_TOPIC, EVENT_TOPIC)
    }

    telemetry = []
    events = []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        message_type = message_types.get(topic)
        if message_type is None:
            continue
        message = deserialize_message(data, message_type)
        if topic == TELEMETRY_TOPIC:
            telemetry.append((timestamp, message))
        elif topic == EVENT_TOPIC:
            events.append((timestamp, message.data))
    return telemetry, events


def sustained_start(rows, predicate, duration_ns=SUSTAIN_NS):
    """Return the first timestamp at which a condition persisted long enough."""
    start = None
    for timestamp, message in rows:
        if predicate(message):
            if start is None:
                start = timestamp
            if timestamp - start >= duration_ns:
                return start
        else:
            start = None
    return None


def format_time(timestamp, origin):
    return f"{(timestamp - origin) / 1_000_000_000.0:.3f}s"


def write_csv(rows, destination: Path):
    fields = [
        "time_s",
        "mode",
        "fsm_state",
        "active_source",
        "have_input",
        "requested_front_steer_rad",
        "requested_rear_steer_rad",
        "requested_front_speed_mps",
        "requested_rear_speed_mps",
        "commanded_front_steer_rad",
        "commanded_rear_steer_rad",
        "commanded_front_speed_mps",
        "commanded_rear_speed_mps",
        "front_wheel_status_word",
        "rear_wheel_status_word",
        "front_wheel_error_code",
        "rear_wheel_error_code",
        "front_wheel_actual_velocity_raw",
        "rear_wheel_actual_velocity_raw",
        "front_wheel_feedback_age_ms",
        "rear_wheel_feedback_age_ms",
        "front_steer_feedback_deg",
        "rear_steer_feedback_deg",
        "front_steer_error_deg",
        "rear_steer_error_deg",
        "front_steer_feedback_age_ms",
        "rear_steer_feedback_age_ms",
        "diagnostic_flags",
    ]
    origin = rows[0][0]
    with destination.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for timestamp, message in rows:
            row = {"time_s": (timestamp - origin) / 1_000_000_000.0}
            for field in fields[1:]:
                row[field] = getattr(message, field)
            writer.writerow(row)


def add_candidate(candidates, timestamp, label, detail):
    if timestamp is not None:
        candidates.append((timestamp, label, detail))


def build_candidates(rows):
    """Apply passive, sustained rules to the selected pre-fault window."""
    candidates = []
    first = rows[0][1]

    add_candidate(
        candidates,
        sustained_start(
            rows,
            lambda m: m.front_wheel_feedback_age_ms < 0
            or m.front_wheel_feedback_age_ms > 500,
        ),
        f"前轮毂反馈异常 (status CAN 0x{first.wheel_front_status_can_id:03X})",
        "轮毂反馈连续超时",
    )
    add_candidate(
        candidates,
        sustained_start(
            rows,
            lambda m: m.rear_wheel_feedback_age_ms < 0
            or m.rear_wheel_feedback_age_ms > 500,
        ),
        f"后轮毂反馈异常 (status CAN 0x{first.wheel_rear_status_can_id:03X})",
        "轮毂反馈连续超时",
    )
    add_candidate(
        candidates,
        sustained_start(
            rows,
            lambda m: abs(m.commanded_front_speed_mps) >= 0.05
            and 0 <= m.front_wheel_feedback_age_ms <= 500
            and m.front_wheel_actual_velocity_raw == 0,
        ),
        f"前轮毂疑似失速 (command CAN 0x{first.wheel_front_command_can_id:03X})",
        "存在非零轮速命令，但实际速度原始值持续为零",
    )
    add_candidate(
        candidates,
        sustained_start(
            rows,
            lambda m: abs(m.commanded_rear_speed_mps) >= 0.05
            and 0 <= m.rear_wheel_feedback_age_ms <= 500
            and m.rear_wheel_actual_velocity_raw == 0,
        ),
        f"后轮毂疑似失速 (command CAN 0x{first.wheel_rear_command_can_id:03X})",
        "存在非零轮速命令，但实际速度原始值持续为零",
    )
    add_candidate(
        candidates,
        sustained_start(
            rows,
            lambda m: m.front_steer_feedback_age_ms < 0
            or m.front_steer_feedback_age_ms > 300,
        ),
        f"前舵机无真实反馈 (CAN 0x{first.steer_front_can_id:03X})",
        "剔除本地查询回环后，真实 0x94 响应持续超时",
    )
    add_candidate(
        candidates,
        sustained_start(
            rows,
            lambda m: m.rear_steer_feedback_age_ms < 0
            or m.rear_steer_feedback_age_ms > 300,
        ),
        f"后舵机无真实反馈 (CAN 0x{first.steer_rear_can_id:03X})",
        "剔除本地查询回环后，真实 0x94 响应持续超时",
    )
    add_candidate(
        candidates,
        sustained_start(
            rows,
            lambda m: 0 <= m.front_steer_feedback_age_ms <= 300
            and abs(m.front_steer_error_deg) >= 8.0,
        ),
        f"前舵机疑似未执行转向 (CAN 0x{first.steer_front_can_id:03X})",
        "真实反馈存在，但目标/反馈角度偏差持续超过 8 度",
    )
    add_candidate(
        candidates,
        sustained_start(
            rows,
            lambda m: 0 <= m.rear_steer_feedback_age_ms <= 300
            and abs(m.rear_steer_error_deg) >= 8.0,
        ),
        f"后舵机疑似未执行转向 (CAN 0x{first.steer_rear_can_id:03X})",
        "真实反馈存在，但目标/反馈角度偏差持续超过 8 度",
    )

    for timestamp, message in rows:
        if message.front_wheel_error_code != 0:
            add_candidate(
                candidates,
                timestamp,
                f"前轮毂驱动器错误 (CAN 0x{message.wheel_front_status_can_id:03X})",
                f"错误码 0x{message.front_wheel_error_code:04X}",
            )
            break
    for timestamp, message in rows:
        if message.rear_wheel_error_code != 0:
            add_candidate(
                candidates,
                timestamp,
                f"后轮毂驱动器错误 (CAN 0x{message.wheel_rear_status_can_id:03X})",
                f"错误码 0x{message.rear_wheel_error_code:04X}",
            )
            break

    if rows[-1][1].can_error_frame_count > rows[0][1].can_error_frame_count:
        add_candidate(
            candidates,
            rows[-1][0],
            "公共 CAN 通信异常",
            "分析窗口内收到新的 SocketCAN error frame",
        )
    return sorted(candidates, key=lambda item: item[0])


def write_summary(rows, events, candidates, destination: Path):
    origin = rows[0][0]
    marker = events[-1][0] if events else None
    first = rows[0][1]
    lines = [
        "# 底盘故障日志分析",
        "",
        f"- 遥测样本数：{len(rows)}",
        f"- 分析窗口：{format_time(rows[0][0], origin)} ～ "
        f"{format_time(rows[-1][0], origin)}",
        f"- 前舵机：0x{first.steer_front_can_id:03X}；"
        f"后舵机：0x{first.steer_rear_can_id:03X}",
        f"- 前轮毂：命令 0x{first.wheel_front_command_can_id:03X} / "
        f"状态 0x{first.wheel_front_status_can_id:03X}",
        f"- 后轮毂：命令 0x{first.wheel_rear_command_can_id:03X} / "
        f"状态 0x{first.wheel_rear_status_can_id:03X}",
    ]
    if marker is not None:
        lines.append(f"- 人工故障标记：{format_time(marker, origin)}")
    lines.extend(["", "## 判断结果", ""])
    if candidates:
        for timestamp, label, detail in candidates:
            lines.append(
                f"- `{format_time(timestamp, origin)}` **{label}**：{detail}"
            )
    else:
        lines.append("- 当前阈值下未发现持续异常，请结合时间线和原始 CAN 帧复核。")
    lines.extend(
        [
            "",
            "## 说明",
            "",
            "- 实际轮速未做硬件标定，报告使用原始值是否为零及变化趋势。",
            "- 无反馈只能定位到对应电机/驱动支路，不能单独区分电机、驱动器、供电或线束。",
        ]
    )
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_plot(rows, destination: Path):
    os.environ.setdefault(
        "MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "agv2_matplotlib")
    )
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    origin = rows[0][0]
    times = [(timestamp - origin) / 1_000_000_000.0 for timestamp, _ in rows]
    messages = [message for _, message in rows]
    figure, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)

    wheel_specs = [
        ("Front wheel", "commanded_front_speed_mps", "front_wheel_actual_velocity_raw"),
        ("Rear wheel", "commanded_rear_speed_mps", "rear_wheel_actual_velocity_raw"),
    ]
    for axis, (label, command_field, feedback_field) in zip(axes[:2], wheel_specs):
        axis.plot(times, [getattr(m, command_field) for m in messages], label="Command m/s")
        axis.set_ylabel(f"{label}\ncommand m/s")
        axis.grid(True)
        twin = axis.twinx()
        twin.plot(
            times,
            [getattr(m, feedback_field) for m in messages],
            color="tab:orange",
            label="Actual velocity raw",
        )
        twin.set_ylabel("feedback raw")

    steer_specs = [
        ("Front steer", "commanded_front_steer_rad", "front_steer_feedback_deg"),
        ("Rear steer", "commanded_rear_steer_rad", "rear_steer_feedback_deg"),
    ]
    for axis, (label, command_field, feedback_field) in zip(axes[2:], steer_specs):
        axis.plot(
            times,
            [math.degrees(getattr(m, command_field)) for m in messages],
            label="Target angle",
        )
        axis.plot(
            times,
            [getattr(m, feedback_field) for m in messages],
            label="Feedback angle",
        )
        axis.set_ylabel(f"{label}\nangle deg")
        axis.grid(True)
        axis.legend(loc="upper right")

    axes[-1].set_xlabel("Relative time / s")
    figure.suptitle("AGV2 motor command and feedback timeline")
    figure.tight_layout()
    figure.savefig(destination, dpi=140)
    plt.close(figure)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_directory", type=Path)
    args = parser.parse_args()

    telemetry, events = read_bag(args.bag_directory)
    if not telemetry:
        raise SystemExit(f"{TELEMETRY_TOPIC} not found in {args.bag_directory}")

    marker = events[-1][0] if events else telemetry[-1][0]
    window_start = marker - ANALYSIS_WINDOW_NS
    rows = [
        item
        for item in telemetry
        if window_start <= item[0] <= marker + 1_000_000_000
    ]
    if not rows:
        rows = telemetry

    analysis_dir = args.bag_directory / "analysis"
    analysis_dir.mkdir(parents=True, exist_ok=True)
    candidates = build_candidates(rows)
    write_csv(rows, analysis_dir / "telemetry.csv")
    write_summary(rows, events, candidates, analysis_dir / "summary.md")
    plot_written = write_plot(rows, analysis_dir / "motor_timeline.png")

    print(f"Analysis written to: {analysis_dir}")
    print(f"Summary: {analysis_dir / 'summary.md'}")
    if plot_written:
        print(f"Timeline: {analysis_dir / 'motor_timeline.png'}")


if __name__ == "__main__":
    main()
