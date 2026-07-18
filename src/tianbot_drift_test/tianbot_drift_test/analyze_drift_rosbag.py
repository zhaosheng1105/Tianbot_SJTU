"""Analyze a Tianbot drift rosbag2 SQLite3 file without ROS dependencies.

The tool decodes the CDR payloads used by the drift test, exports topic CSVs,
computes hold-phase metrics, compares the recorded and baseline controller YAML,
and writes standalone SVG plots plus an HTML report.
"""

import argparse
import ast
import csv
import html
import json
import math
import os
import sqlite3
import struct
from pathlib import Path


WHEELS = ("FL", "FR", "RL", "RR")
PHASE_NAMES = {
    0: "idle",
    1: "calibrate",
    2: "accelerate",
    3: "initiate",
    4: "hold",
    5: "stop",
    6: "abort",
}
SERIES_COLORS = ("#2563eb", "#ea580c", "#16a34a", "#dc2626")


class CdrReader:
    """Minimal little-endian CDR reader for the message types in this bag."""

    def __init__(self, payload):
        if len(payload) < 4:
            raise ValueError("CDR payload is shorter than its encapsulation header")
        encapsulation = bytes(payload[:2])
        if encapsulation != b"\x00\x01":
            raise ValueError(
                "only little-endian CDR is supported; encapsulation=%r"
                % encapsulation
            )
        # CDR field alignment starts after the four-byte encapsulation header.
        self.data = memoryview(payload)[4:]
        self.offset = 0

    def align(self, size):
        self.offset = (self.offset + size - 1) & ~(size - 1)

    def uint8(self):
        value = int(self.data[self.offset])
        self.offset += 1
        return value

    def int32(self):
        self.align(4)
        value = struct.unpack_from("<i", self.data, self.offset)[0]
        self.offset += 4
        return value

    def uint32(self):
        self.align(4)
        value = struct.unpack_from("<I", self.data, self.offset)[0]
        self.offset += 4
        return value

    def float32(self):
        self.align(4)
        value = struct.unpack_from("<f", self.data, self.offset)[0]
        self.offset += 4
        return value

    def float64(self):
        self.align(8)
        value = struct.unpack_from("<d", self.data, self.offset)[0]
        self.offset += 8
        return value

    def string(self):
        length = self.uint32()
        raw = bytes(self.data[self.offset : self.offset + length])
        self.offset += length
        return raw.rstrip(b"\x00").decode("utf-8", errors="replace")


def read_header(reader):
    return {
        "stamp_sec": reader.int32(),
        "stamp_nanosec": reader.uint32(),
        "frame_id": reader.string(),
    }


def quaternion_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def decode_imu(payload):
    reader = CdrReader(payload)
    header = read_header(reader)
    quaternion = [reader.float64() for _ in range(4)]
    orientation_covariance = [reader.float64() for _ in range(9)]
    angular_velocity = [reader.float64() for _ in range(3)]
    angular_velocity_covariance = [reader.float64() for _ in range(9)]
    linear_acceleration = [reader.float64() for _ in range(3)]
    linear_acceleration_covariance = [reader.float64() for _ in range(9)]
    return {
        **header,
        "orientation_x": quaternion[0],
        "orientation_y": quaternion[1],
        "orientation_z": quaternion[2],
        "orientation_w": quaternion[3],
        "angular_velocity_x": angular_velocity[0],
        "angular_velocity_y": angular_velocity[1],
        "angular_velocity_z": angular_velocity[2],
        "linear_acceleration_x": linear_acceleration[0],
        "linear_acceleration_y": linear_acceleration[1],
        "linear_acceleration_z": linear_acceleration[2],
        "orientation_covariance": orientation_covariance,
        "angular_velocity_covariance": angular_velocity_covariance,
        "linear_acceleration_covariance": linear_acceleration_covariance,
    }


def decode_odometry(payload):
    reader = CdrReader(payload)
    header = read_header(reader)
    child_frame_id = reader.string()
    position = [reader.float64() for _ in range(3)]
    quaternion = [reader.float64() for _ in range(4)]
    pose_covariance = [reader.float64() for _ in range(36)]
    linear = [reader.float64() for _ in range(3)]
    angular = [reader.float64() for _ in range(3)]
    twist_covariance = [reader.float64() for _ in range(36)]
    return {
        **header,
        "child_frame_id": child_frame_id,
        "x": position[0],
        "y": position[1],
        "z": position[2],
        "yaw": quaternion_yaw(*quaternion),
        "linear_x": linear[0],
        "linear_y": linear[1],
        "linear_z": linear[2],
        "speed": math.hypot(linear[0], linear[1]),
        "angular_x": angular[0],
        "angular_y": angular[1],
        "angular_z": angular[2],
        "pose_covariance": pose_covariance,
        "twist_covariance": twist_covariance,
    }


def decode_mit_command(payload):
    reader = CdrReader(payload)
    values = {}
    for field in ("p_des", "v_des", "kp", "kd", "t_ff"):
        values[field] = [reader.float32() for _ in range(4)]
    return values


def decode_diagnostics(payload):
    reader = CdrReader(payload)
    dimensions = []
    for _ in range(reader.uint32()):
        dimensions.append(
            {
                "label": reader.string(),
                "size": reader.uint32(),
                "stride": reader.uint32(),
            }
        )
    data_offset = reader.uint32()
    values = [reader.float64() for _ in range(reader.uint32())]
    labels = []
    if dimensions:
        labels = [item.strip() for item in dimensions[0]["label"].split(",")]
    decoded = {
        label: values[index]
        for index, label in enumerate(labels)
        if index < len(values)
    }
    decoded["layout_data_offset"] = data_offset
    decoded["values"] = values
    return decoded


def decode_motor_feedback(payload):
    reader = CdrReader(payload)
    header = read_header(reader)
    array_control_mode = reader.string()
    motors = []
    for _ in range(4):
        motors.append(
            {
                "id": reader.uint8(),
                "state": reader.string(),
                "control_mode": reader.string(),
                "output_speed_rad_s": reader.float32(),
                "output_torque_nm": reader.float32(),
                "mos_temp_c": reader.float32(),
                "coil_temp_c": reader.float32(),
                "last_feedback_age_ms": reader.uint32(),
            }
        )
    return {**header, "control_mode": array_control_mode, "motors": motors}


DECODERS = {
    "/tianbot/odom": decode_odometry,
    "/tianbot/imu": decode_imu,
    "/tianbot/wheel_mit_cmd": decode_mit_command,
    "/tianbot/motor_feedback": decode_motor_feedback,
    "/drift_imu_only_pid/diagnostics": decode_diagnostics,
}


def locate_db3(path):
    path = Path(path).expanduser().resolve()
    if path.is_file():
        if path.suffix.lower() != ".db3":
            raise ValueError("input file must be a rosbag2 .db3 file")
        return path
    if not path.is_dir():
        raise FileNotFoundError(path)
    candidates = sorted(path.glob("*.db3"))
    if len(candidates) != 1:
        raise ValueError(
            "expected exactly one .db3 file in %s, found %d"
            % (path, len(candidates))
        )
    return candidates[0]


def load_bag(db_path):
    db_path = Path(db_path)
    connection = sqlite3.connect(str(db_path))
    try:
        topic_rows = connection.execute(
            "SELECT id, name, type, serialization_format FROM topics"
        ).fetchall()
        topics = {
            row[1]: {
                "id": row[0],
                "type": row[2],
                "serialization_format": row[3],
            }
            for row in topic_rows
        }
        missing = sorted(set(DECODERS) - set(topics))
        if missing:
            raise ValueError("bag is missing required topics: %s" % ", ".join(missing))
        unsupported = [
            name
            for name in DECODERS
            if topics[name]["serialization_format"] != "cdr"
        ]
        if unsupported:
            raise ValueError("topics are not CDR serialized: %s" % unsupported)

        first_timestamp = connection.execute(
            "SELECT MIN(timestamp) FROM messages"
        ).fetchone()[0]
        last_timestamp = connection.execute(
            "SELECT MAX(timestamp) FROM messages"
        ).fetchone()[0]
        decoded = {}
        counts = {}
        for topic_name, decoder in DECODERS.items():
            topic_id = topics[topic_name]["id"]
            rows = []
            query = (
                "SELECT timestamp, data FROM messages "
                "WHERE topic_id = ? ORDER BY timestamp"
            )
            for timestamp, payload in connection.execute(query, (topic_id,)):
                item = decoder(payload)
                item["timestamp_ns"] = timestamp
                item["bag_time_s"] = (timestamp - first_timestamp) * 1.0e-9
                rows.append(item)
            decoded[topic_name] = rows
            counts[topic_name] = len(rows)
        return {
            "db_path": str(db_path),
            "first_timestamp_ns": first_timestamp,
            "last_timestamp_ns": last_timestamp,
            "duration_s": (last_timestamp - first_timestamp) * 1.0e-9,
            "topics": topics,
            "counts": counts,
            "data": decoded,
        }
    finally:
        connection.close()


def parse_yaml_scalar(text):
    value = text.strip()
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if lowered in ("null", "none", "~"):
        return None
    try:
        return ast.literal_eval(value)
    except (ValueError, SyntaxError):
        try:
            if any(character in value for character in (".", "e", "E")):
                return float(value)
            return int(value)
        except ValueError:
            return value


def load_ros_parameter_yaml(path):
    if path is None:
        return {}
    path = Path(path)
    parameters = {}
    inside_parameters = False
    parameter_indent = None
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        content = raw_line.split("#", 1)[0].rstrip()
        if not content.strip():
            continue
        indent = len(content) - len(content.lstrip())
        stripped = content.strip()
        if stripped == "ros__parameters:":
            inside_parameters = True
            parameter_indent = indent
            continue
        if not inside_parameters:
            continue
        if indent <= parameter_indent:
            break
        if ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        parameters[key.strip()] = parse_yaml_scalar(value)
    return parameters


def compare_parameters(baseline, recorded):
    differences = []
    for key in sorted(set(baseline) | set(recorded)):
        old = baseline.get(key, "<missing>")
        new = recorded.get(key, "<missing>")
        if old == new:
            continue
        ratio = None
        if (
            isinstance(old, (int, float))
            and not isinstance(old, bool)
            and isinstance(new, (int, float))
            and not isinstance(new, bool)
            and abs(float(old)) > 1.0e-12
        ):
            ratio = float(new) / float(old)
        differences.append({"parameter": key, "baseline": old, "recorded": new, "ratio": ratio})
    return differences


def finite_values(values):
    return [float(value) for value in values if math.isfinite(float(value))]


def mean(values):
    values = finite_values(values)
    return sum(values) / len(values) if values else float("nan")


def standard_deviation(values):
    values = finite_values(values)
    if not values:
        return float("nan")
    center = sum(values) / len(values)
    return math.sqrt(sum((value - center) ** 2 for value in values) / len(values))


def root_mean_square(values):
    values = finite_values(values)
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


def percentile(values, probability):
    values = sorted(finite_values(values))
    if not values:
        return float("nan")
    position = probability * (len(values) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    fraction = position - lower
    return values[lower] * (1.0 - fraction) + values[upper] * fraction


def estimate_oscillation_frequency(times, values):
    pairs = [
        (float(time), float(value))
        for time, value in zip(times, values)
        if math.isfinite(float(value))
    ]
    if len(pairs) < 30:
        return float("nan")
    times = [item[0] for item in pairs]
    values = [item[1] for item in pairs]
    sample_period = mean(
        [later - earlier for earlier, later in zip(times[:-1], times[1:])]
    )
    if not math.isfinite(sample_period) or sample_period <= 0.0:
        return float("nan")
    center = mean(values)
    centered = [value - center for value in values]
    energy = sum(value * value for value in centered)
    if energy <= 1.0e-12:
        return float("nan")
    minimum_lag = max(2, int(0.20 / sample_period))
    maximum_lag = min(len(centered) // 3, int(2.0 / sample_period))
    correlations = []
    for lag in range(minimum_lag, maximum_lag + 1):
        numerator = sum(
            centered[index] * centered[index + lag]
            for index in range(len(centered) - lag)
        )
        denominator = math.sqrt(
            sum(value * value for value in centered[:-lag])
            * sum(value * value for value in centered[lag:])
        )
        correlations.append((numerator / denominator if denominator else 0.0, lag))
    local_peaks = [
        item
        for index, item in enumerate(correlations[1:-1], start=1)
        if item[0] >= correlations[index - 1][0]
        and item[0] >= correlations[index + 1][0]
    ]
    candidates = local_peaks or correlations
    correlation, lag = max(candidates, key=lambda item: item[0])
    if correlation < 0.15:
        return float("nan")
    return 1.0 / (lag * sample_period)


def solve_three_by_three(matrix, vector):
    augmented = [list(row) + [value] for row, value in zip(matrix, vector)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1.0e-12:
            raise ValueError("singular circle-fit matrix")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                current - factor * reference
                for current, reference in zip(augmented[row], augmented[column])
            ]
    return [augmented[index][3] for index in range(3)]


def fit_circle(points):
    points = [
        (float(x), float(y))
        for x, y in points
        if math.isfinite(float(x)) and math.isfinite(float(y))
    ]
    if len(points) < 3:
        return None
    sx = sum(x for x, _ in points)
    sy = sum(y for _, y in points)
    sxx = sum(x * x for x, _ in points)
    syy = sum(y * y for _, y in points)
    sxy = sum(x * y for x, y in points)
    sz = sum(x * x + y * y for x, y in points)
    sxz = sum(x * (x * x + y * y) for x, y in points)
    syz = sum(y * (x * x + y * y) for x, y in points)
    try:
        a, b, c = solve_three_by_three(
            [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, len(points)]],
            [sxz, syz, sz],
        )
    except ValueError:
        return None
    center_x = 0.5 * a
    center_y = 0.5 * b
    radius_squared = c + center_x * center_x + center_y * center_y
    if radius_squared <= 0.0:
        return None
    radius = math.sqrt(radius_squared)
    residuals = [
        math.hypot(x - center_x, y - center_y) - radius for x, y in points
    ]
    return {
        "center_x": center_x,
        "center_y": center_y,
        "radius_m": radius,
        "rms_residual_m": root_mean_square(residuals),
    }


def phase_intervals(diagnostics):
    active = [row for row in diagnostics if int(round(row.get("phase", 0.0))) > 0]
    if not active:
        raise ValueError("diagnostics contain no active controller phase")
    intervals = []
    current_phase = int(round(active[0]["phase"]))
    start = active[0]["bag_time_s"]
    previous_time = start
    for row in active[1:]:
        phase = int(round(row["phase"]))
        current_time = row["bag_time_s"]
        if phase != current_phase:
            intervals.append(
                {
                    "phase": current_phase,
                    "name": PHASE_NAMES.get(current_phase, str(current_phase)),
                    "start_s": start,
                    "end_s": previous_time,
                }
            )
            current_phase = phase
            start = current_time
        previous_time = current_time
    intervals.append(
        {
            "phase": current_phase,
            "name": PHASE_NAMES.get(current_phase, str(current_phase)),
            "start_s": start,
            "end_s": previous_time,
        }
    )
    return intervals


def within(rows, start, end):
    return [row for row in rows if start <= row["bag_time_s"] <= end]


def calculate_metrics(bag, recorded_parameters):
    data = bag["data"]
    diagnostics = data["/drift_imu_only_pid/diagnostics"]
    intervals = phase_intervals(diagnostics)
    active_start = intervals[0]["start_s"]
    active_end = intervals[-1]["end_s"]
    hold_interval = next(
        (interval for interval in intervals if interval["phase"] == 4), None
    )
    if hold_interval is None:
        raise ValueError("diagnostics contain no hold phase")
    hold_start = hold_interval["start_s"]
    hold_end = hold_interval["end_s"]

    hold_diagnostics = within(diagnostics, hold_start, hold_end)
    hold_odom = within(data["/tianbot/odom"], hold_start, hold_end)
    hold_commands = within(data["/tianbot/wheel_mit_cmd"], hold_start, hold_end)
    hold_motors = within(data["/tianbot/motor_feedback"], hold_start, hold_end)

    yaw = [row.get("yaw_rate", float("nan")) for row in hold_diagnostics]
    yaw_reference = [row.get("yaw_ref", float("nan")) for row in hold_diagnostics]
    yaw_error = [actual - reference for actual, reference in zip(yaw, yaw_reference)]
    yaw_frequency = estimate_oscillation_frequency(
        [row["bag_time_s"] for row in hold_diagnostics], yaw
    )

    torque_cap = float(recorded_parameters.get("software_torque_max_nm", 0.0) or 0.0)
    slew_limit = float(recorded_parameters.get("command_slew_nmps", 0.0) or 0.0)
    yaw_kp = float(recorded_parameters.get("yaw_kp_nm_per_radps", 0.0) or 0.0)
    yaw_kd = float(recorded_parameters.get("yaw_kd_nm_per_radps2", 0.0) or 0.0)
    maximum_yaw_correction = float(
        recorded_parameters.get("max_yaw_correction_nm", 0.0) or 0.0
    )
    hold_left = float(
        recorded_parameters.get("hold_left_feedforward_nm", 0.0) or 0.0
    )
    hold_right = float(
        recorded_parameters.get("hold_right_feedforward_nm", 0.0) or 0.0
    )
    all_commands = [row["t_ff"] for row in hold_commands]
    saturation_count = 0
    sample_count = 0
    if torque_cap > 0.0:
        for command in all_commands:
            sample_count += 1
            if any(abs(value) >= 0.98 * torque_cap for value in command):
                saturation_count += 1
    slew_count = 0
    slew_samples = 0
    if slew_limit > 0.0:
        for previous, current in zip(hold_commands[:-1], hold_commands[1:]):
            dt = current["bag_time_s"] - previous["bag_time_s"]
            if dt <= 0.0:
                continue
            rates = [
                abs(now - before) / dt
                for now, before in zip(current["t_ff"], previous["t_ff"])
            ]
            slew_samples += 1
            if any(rate >= 0.95 * slew_limit for rate in rates):
                slew_count += 1

    raw_corrections = [
        yaw_kp * (row.get("yaw_ref", 0.0) - row.get("yaw_rate", 0.0))
        - yaw_kd * row.get("yaw_rate_derivative", 0.0)
        for row in hold_diagnostics
    ]
    correction_saturation_fraction = 0.0
    if maximum_yaw_correction > 0.0 and raw_corrections:
        correction_saturation_fraction = sum(
            abs(value) >= maximum_yaw_correction for value in raw_corrections
        ) / len(raw_corrections)
    predicted_torque_limit_fraction = 0.0
    if torque_cap > 0.0 and raw_corrections:
        hold_base = 0.5 * (hold_left + hold_right)
        hold_difference = 0.5 * (hold_right - hold_left)
        predicted_torque_limit_fraction = sum(
            abs(hold_base - hold_difference - max(-maximum_yaw_correction, min(maximum_yaw_correction, correction))) >= torque_cap
            or abs(hold_base + hold_difference + max(-maximum_yaw_correction, min(maximum_yaw_correction, correction))) >= torque_cap
            for correction in raw_corrections
        ) / len(raw_corrections)

    x_values = [row["x"] for row in hold_odom]
    y_values = [row["y"] for row in hold_odom]
    yaw_pose_values = [row["yaw"] for row in hold_odom]
    pose_available = bool(x_values) and (
        max(x_values) - min(x_values) > 1.0e-6
        or max(y_values) - min(y_values) > 1.0e-6
        or max(yaw_pose_values) - min(yaw_pose_values) > 1.0e-6
    )
    circle = fit_circle([(row["x"], row["y"]) for row in hold_odom]) if pose_available else None
    distance = (
        sum(
            math.hypot(current["x"] - previous["x"], current["y"] - previous["y"])
            for previous, current in zip(hold_odom[:-1], hold_odom[1:])
        )
        if pose_available
        else float("nan")
    )
    speed = [row["speed"] for row in hold_odom]

    command_statistics = {}
    speed_statistics = {}
    feedback_torque_statistics = {}
    for index, wheel in enumerate(WHEELS):
        command_values = [row["t_ff"][index] for row in hold_commands]
        command_statistics[wheel] = {
            "mean_nm": mean(command_values),
            "std_nm": standard_deviation(command_values),
            "min_nm": min(command_values) if command_values else float("nan"),
            "max_nm": max(command_values) if command_values else float("nan"),
        }
        motor_speeds = [row["motors"][index]["output_speed_rad_s"] for row in hold_motors]
        motor_torques = [row["motors"][index]["output_torque_nm"] for row in hold_motors]
        speed_statistics[wheel] = {
            "mean_rad_s": mean(motor_speeds),
            "mean_abs_rad_s": mean([abs(value) for value in motor_speeds]),
            "std_rad_s": standard_deviation(motor_speeds),
            "min_rad_s": min(motor_speeds) if motor_speeds else float("nan"),
            "max_rad_s": max(motor_speeds) if motor_speeds else float("nan"),
        }
        feedback_torque_statistics[wheel] = {
            "mean_nm": mean(motor_torques),
            "std_nm": standard_deviation(motor_torques),
            "min_nm": min(motor_torques) if motor_torques else float("nan"),
            "max_nm": max(motor_torques) if motor_torques else float("nan"),
        }

    return {
        "bag_duration_s": bag["duration_s"],
        "active_start_s": active_start,
        "active_end_s": active_end,
        "active_duration_s": active_end - active_start,
        "hold_start_s": hold_start,
        "hold_end_s": hold_end,
        "hold_duration_s": hold_end - hold_start,
        "phase_intervals": intervals,
        "yaw_rate": {
            "reference_mean_rad_s": mean(yaw_reference),
            "mean_rad_s": mean(yaw),
            "std_rad_s": standard_deviation(yaw),
            "min_rad_s": min(yaw) if yaw else float("nan"),
            "max_rad_s": max(yaw) if yaw else float("nan"),
            "rmse_rad_s": root_mean_square(yaw_error),
            "p05_rad_s": percentile(yaw, 0.05),
            "p95_rad_s": percentile(yaw, 0.95),
            "oscillation_frequency_hz": yaw_frequency,
        },
        "odom": {
            "pose_available": pose_available,
            "mean_speed_m_s": mean(speed),
            "std_speed_m_s": standard_deviation(speed),
            "min_speed_m_s": min(speed) if speed else float("nan"),
            "max_speed_m_s": max(speed) if speed else float("nan"),
            "distance_during_hold_m": distance,
            "circle_fit": circle,
        },
        "command": {
            "published_saturation_fraction": saturation_count / sample_count if sample_count else 0.0,
            "slew_limit_fraction": slew_count / slew_samples if slew_samples else 0.0,
            "raw_yaw_correction_saturation_fraction": correction_saturation_fraction,
            "predicted_raw_torque_limit_fraction": predicted_torque_limit_fraction,
            "per_wheel": command_statistics,
        },
        "motor_speed": speed_statistics,
        "motor_feedback_torque": feedback_torque_statistics,
        "message_counts": bag["counts"],
    }


def format_number(value, digits=3):
    if value is None:
        return "n/a"
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            return "n/a"
        return ("%%.%df" % digits) % value
    return str(value)


def json_compatible(value):
    """Replace non-finite floats with JSON null recursively."""
    if isinstance(value, dict):
        return {key: json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_compatible(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def write_csv(path, fieldnames, rows):
    with Path(path).open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def export_csv_files(output_directory, bag, active_start):
    data = bag["data"]
    diagnostics_rows = []
    for row in data["/drift_imu_only_pid/diagnostics"]:
        item = {key: value for key, value in row.items() if key not in ("values",)}
        item["time_s"] = row["bag_time_s"] - active_start
        diagnostics_rows.append(item)
    diagnostic_fields = [
        "time_s",
        "timestamp_ns",
        "phase",
        "imu_raw",
        "imu_bias",
        "yaw_rate",
        "yaw_rate_derivative",
        "yaw_ref",
        "yaw_turns",
        "odom_speed",
        "odom_radius_raw",
        "odom_radius",
        "radius_rate",
        "radius_yaw_bias",
        "odom_x",
        "odom_y",
        "torque_FL",
        "torque_FR",
        "torque_RL",
        "torque_RR",
    ]
    write_csv(output_directory / "diagnostics.csv", diagnostic_fields, diagnostics_rows)

    odom_rows = []
    for row in data["/tianbot/odom"]:
        item = dict(row)
        item["time_s"] = row["bag_time_s"] - active_start
        odom_rows.append(item)
    write_csv(
        output_directory / "odom.csv",
        [
            "time_s",
            "timestamp_ns",
            "x",
            "y",
            "z",
            "yaw",
            "linear_x",
            "linear_y",
            "speed",
            "angular_z",
        ],
        odom_rows,
    )

    imu_rows = []
    for row in data["/tianbot/imu"]:
        item = dict(row)
        item["time_s"] = row["bag_time_s"] - active_start
        imu_rows.append(item)
    write_csv(
        output_directory / "imu.csv",
        [
            "time_s",
            "timestamp_ns",
            "angular_velocity_x",
            "angular_velocity_y",
            "angular_velocity_z",
            "linear_acceleration_x",
            "linear_acceleration_y",
            "linear_acceleration_z",
        ],
        imu_rows,
    )

    command_rows = []
    for row in data["/tianbot/wheel_mit_cmd"]:
        command_rows.append(
            {
                "time_s": row["bag_time_s"] - active_start,
                "timestamp_ns": row["timestamp_ns"],
                **{"t_ff_%s_nm" % wheel: row["t_ff"][index] for index, wheel in enumerate(WHEELS)},
            }
        )
    write_csv(
        output_directory / "wheel_command.csv",
        ["time_s", "timestamp_ns"] + ["t_ff_%s_nm" % wheel for wheel in WHEELS],
        command_rows,
    )

    motor_rows = []
    for row in data["/tianbot/motor_feedback"]:
        item = {"time_s": row["bag_time_s"] - active_start, "timestamp_ns": row["timestamp_ns"]}
        for index, wheel in enumerate(WHEELS):
            motor = row["motors"][index]
            item["speed_%s_rad_s" % wheel] = motor["output_speed_rad_s"]
            item["torque_%s_nm" % wheel] = motor["output_torque_nm"]
            item["state_%s" % wheel] = motor["state"]
        motor_rows.append(item)
    motor_fields = ["time_s", "timestamp_ns"]
    for wheel in WHEELS:
        motor_fields.extend(
            ["speed_%s_rad_s" % wheel, "torque_%s_nm" % wheel, "state_%s" % wheel]
        )
    write_csv(output_directory / "motor_feedback.csv", motor_fields, motor_rows)


def nice_range(values):
    values = finite_values(values)
    if not values:
        return -1.0, 1.0
    lower = min(values)
    upper = max(values)
    if lower == upper:
        padding = max(abs(lower) * 0.1, 1.0)
    else:
        padding = 0.08 * (upper - lower)
    return lower - padding, upper + padding


def downsample(points, maximum=1800):
    if len(points) <= maximum:
        return points
    step = max(1, math.ceil(len(points) / maximum))
    selected = points[::step]
    if selected[-1] != points[-1]:
        selected.append(points[-1])
    return selected


def svg_style():
    return """
    :root { --bg: #ffffff; --fg: #111827; --muted: #6b7280; --grid: #e5e7eb; --phase: #2563eb; }
    @media (prefers-color-scheme: dark) {
      :root { --bg: #111827; --fg: #f3f4f6; --muted: #9ca3af; --grid: #374151; --phase: #60a5fa; }
    }
    .background { fill: var(--bg); }
    .axis, .grid { stroke: var(--grid); stroke-width: 1; vector-effect: non-scaling-stroke; }
    .grid { opacity: 0.75; }
    .label, .title, .legend { fill: var(--fg); font-family: sans-serif; }
    .tick, .phase-label { fill: var(--muted); font-family: sans-serif; }
    .title { font-size: 20px; font-weight: 500; }
    .label { font-size: 14px; }
    .tick, .legend, .phase-label { font-size: 12px; }
    .series { fill: none; stroke-width: 1.6; vector-effect: non-scaling-stroke; }
    """


def line_plot_svg(path, title, y_label, series, phases, x_limits, y_limits=None):
    width = 1400
    height = 430
    left, right, top, bottom = 92, 32, 58, 62
    plot_width = width - left - right
    plot_height = height - top - bottom
    x_min, x_max = x_limits
    all_y = [point[1] for item in series for point in item["points"]]
    y_min, y_max = y_limits if y_limits else nice_range(all_y)
    if y_max <= y_min:
        y_max = y_min + 1.0

    def sx(value):
        return left + (value - x_min) / (x_max - x_min) * plot_width

    def sy(value):
        return top + (y_max - value) / (y_max - y_min) * plot_height

    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" role="img">'
        % (width, height, width, height),
        "<title>%s</title>" % html.escape(title),
        "<style>%s</style>" % svg_style(),
        '<rect class="background" width="100%" height="100%"/>',
        '<text class="title" x="%d" y="30">%s</text>' % (left, html.escape(title)),
    ]
    phase_fills = ("#64748b", "#0ea5e9", "#f59e0b", "#8b5cf6", "#22c55e", "#ef4444", "#dc2626")
    for phase in phases:
        start = max(x_min, phase["start_s"])
        end = min(x_max, phase["end_s"])
        if end <= start:
            continue
        x = sx(start)
        phase_width = sx(end) - x
        color = phase_fills[min(phase["phase"], len(phase_fills) - 1)]
        lines.append(
            '<rect x="%.2f" y="%d" width="%.2f" height="%d" fill="%s" opacity="0.055"/>'
            % (x, top, phase_width, plot_height, color)
        )
        if phase_width > 55:
            lines.append(
                '<text class="phase-label" x="%.2f" y="%d">%s</text>'
                % (x + 4, top + 14, html.escape(phase["name"]))
            )

    for index in range(7):
        fraction = index / 6.0
        x_value = x_min + fraction * (x_max - x_min)
        x = sx(x_value)
        lines.append('<line class="grid" x1="%.2f" y1="%d" x2="%.2f" y2="%d"/>' % (x, top, x, top + plot_height))
        lines.append('<text class="tick" text-anchor="middle" x="%.2f" y="%d">%.1f</text>' % (x, top + plot_height + 22, x_value))
    for index in range(6):
        fraction = index / 5.0
        y_value = y_min + fraction * (y_max - y_min)
        y = sy(y_value)
        lines.append('<line class="grid" x1="%d" y1="%.2f" x2="%d" y2="%.2f"/>' % (left, y, left + plot_width, y))
        lines.append('<text class="tick" text-anchor="end" x="%d" y="%.2f">%.3g</text>' % (left - 9, y + 4, y_value))
    lines.extend(
        [
            '<line class="axis" x1="%d" y1="%d" x2="%d" y2="%d"/>' % (left, top + plot_height, left + plot_width, top + plot_height),
            '<line class="axis" x1="%d" y1="%d" x2="%d" y2="%d"/>' % (left, top, left, top + plot_height),
            '<text class="label" text-anchor="middle" x="%d" y="%d">time from control start (s)</text>' % (left + plot_width // 2, height - 14),
            '<text class="label" text-anchor="middle" transform="translate(22,%d) rotate(-90)">%s</text>' % (top + plot_height // 2, html.escape(y_label)),
        ]
    )
    legend_x = left + 8
    for item in series:
        points = [point for point in item["points"] if x_min <= point[0] <= x_max and math.isfinite(point[1])]
        points = downsample(points)
        if points:
            polyline = " ".join("%.2f,%.2f" % (sx(x), sy(y)) for x, y in points)
            dash = ' stroke-dasharray="7 5"' if item.get("dashed") else ""
            lines.append('<polyline class="series" points="%s" stroke="%s"%s/>' % (polyline, item["color"], dash))
        lines.append('<line x1="%d" y1="42" x2="%d" y2="42" stroke="%s" stroke-width="2"%s/>' % (legend_x, legend_x + 22, item["color"], ' stroke-dasharray="7 5"' if item.get("dashed") else ""))
        lines.append('<text class="legend" x="%d" y="46">%s</text>' % (legend_x + 28, html.escape(item["label"])))
        legend_x += 28 + 8 * len(item["label"]) + 34
    lines.append("</svg>")
    Path(path).write_text("\n".join(lines), encoding="utf-8")


def trajectory_svg(path, odom_rows, circle, pose_available, time_offset, active_limits):
    rows = [
        row
        for row in odom_rows
        if active_limits[0] <= row["bag_time_s"] - time_offset <= active_limits[1]
    ]
    points = [(row["x"], row["y"]) for row in rows]
    if not points:
        return
    width, height = 760, 700
    left, right, top, bottom = 84, 34, 58, 68
    x_values = [item[0] for item in points]
    y_values = [item[1] for item in points]
    if circle:
        x_values.extend([circle["center_x"] - circle["radius_m"], circle["center_x"] + circle["radius_m"]])
        y_values.extend([circle["center_y"] - circle["radius_m"], circle["center_y"] + circle["radius_m"]])
    x_min, x_max = nice_range(x_values)
    y_min, y_max = nice_range(y_values)
    data_range = max(x_max - x_min, y_max - y_min)
    x_center = 0.5 * (x_min + x_max)
    y_center = 0.5 * (y_min + y_max)
    x_min, x_max = x_center - data_range / 2.0, x_center + data_range / 2.0
    y_min, y_max = y_center - data_range / 2.0, y_center + data_range / 2.0
    plot_width = width - left - right
    plot_height = height - top - bottom

    def sx(value): return left + (value - x_min) / (x_max - x_min) * plot_width
    def sy(value): return top + (y_max - value) / (y_max - y_min) * plot_height

    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" role="img">' % (width, height, width, height),
        "<title>Odometry trajectory</title>",
        "<style>%s</style>" % svg_style(),
        '<rect class="background" width="100%" height="100%"/>',
        '<text class="title" x="%d" y="30">Odometry trajectory</text>' % left,
    ]
    for index in range(6):
        fraction = index / 5.0
        x_value = x_min + fraction * (x_max - x_min)
        y_value = y_min + fraction * (y_max - y_min)
        x, y = sx(x_value), sy(y_value)
        lines.append('<line class="grid" x1="%.2f" y1="%d" x2="%.2f" y2="%d"/>' % (x, top, x, top + plot_height))
        lines.append('<line class="grid" x1="%d" y1="%.2f" x2="%d" y2="%.2f"/>' % (left, y, left + plot_width, y))
        lines.append('<text class="tick" text-anchor="middle" x="%.2f" y="%d">%.2f</text>' % (x, top + plot_height + 22, x_value))
        lines.append('<text class="tick" text-anchor="end" x="%d" y="%.2f">%.2f</text>' % (left - 8, y + 4, y_value))
    if circle:
        lines.append(
            '<circle cx="%.2f" cy="%.2f" r="%.2f" fill="none" stroke="#f59e0b" stroke-width="1.5" stroke-dasharray="7 5"/>'
            % (sx(circle["center_x"]), sy(circle["center_y"]), circle["radius_m"] / (x_max - x_min) * plot_width)
        )
    if pose_available:
        line_points = downsample(points)
        lines.append('<polyline class="series" points="%s" stroke="#2563eb"/>' % " ".join("%.2f,%.2f" % (sx(x), sy(y)) for x, y in line_points))
        lines.append('<circle cx="%.2f" cy="%.2f" r="5" fill="#16a34a"/>' % (sx(points[0][0]), sy(points[0][1])))
        lines.append('<circle cx="%.2f" cy="%.2f" r="5" fill="#dc2626"/>' % (sx(points[-1][0]), sy(points[-1][1])))
    else:
        lines.append(
            '<text class="label" text-anchor="middle" x="%d" y="%d">odom.pose stayed at x=0, y=0, yaw=0; trajectory is unavailable</text>'
            % (left + plot_width // 2, top + plot_height // 2)
        )
    lines.extend(
        [
            '<text class="label" text-anchor="middle" x="%d" y="%d">odom x (m)</text>' % (left + plot_width // 2, height - 16),
            '<text class="label" text-anchor="middle" transform="translate(20,%d) rotate(-90)">odom y (m)</text>' % (top + plot_height // 2),
            '<text class="legend" x="%d" y="46">path</text>' % (left + 30),
            '<line x1="%d" y1="42" x2="%d" y2="42" stroke="#f59e0b" stroke-dasharray="7 5"/>' % (left + 80, left + 104),
            '<text class="legend" x="%d" y="46">hold-phase circle fit</text>' % (left + 110),
            "</svg>",
        ]
    )
    Path(path).write_text("\n".join(lines), encoding="utf-8")


def make_plots(output_directory, bag, metrics):
    data = bag["data"]
    offset = metrics["active_start_s"]
    x_limits = (-0.5, metrics["active_duration_s"] + 1.0)
    phases = [
        {
            **item,
            "start_s": item["start_s"] - offset,
            "end_s": item["end_s"] - offset,
        }
        for item in metrics["phase_intervals"]
    ]

    diagnostics = data["/drift_imu_only_pid/diagnostics"]
    odom = data["/tianbot/odom"]
    commands = data["/tianbot/wheel_mit_cmd"]
    motors = data["/tianbot/motor_feedback"]

    yaw_series = [
        {
            "label": "filtered IMU yaw rate",
            "color": "#2563eb",
            "points": [(row["bag_time_s"] - offset, row.get("yaw_rate", float("nan"))) for row in diagnostics],
        },
        {
            "label": "yaw reference",
            "color": "#dc2626",
            "dashed": True,
            "points": [(row["bag_time_s"] - offset, row.get("yaw_ref", float("nan"))) for row in diagnostics],
        },
        {
            "label": "odom angular.z",
            "color": "#16a34a",
            "points": [(row["bag_time_s"] - offset, row["angular_z"]) for row in odom],
        },
    ]
    line_plot_svg(output_directory / "yaw_rate.svg", "Yaw-rate tracking", "yaw rate (rad/s)", yaw_series, phases, x_limits)

    velocity_series = [
        {
            "label": "odom body-x velocity",
            "color": "#2563eb",
            "points": [(row["bag_time_s"] - offset, row["linear_x"]) for row in odom],
        },
        {
            "label": "odom speed magnitude",
            "color": "#ea580c",
            "points": [(row["bag_time_s"] - offset, row["speed"]) for row in odom],
        },
    ]
    line_plot_svg(output_directory / "velocity.svg", "Odometry velocity", "velocity (m/s)", velocity_series, phases, x_limits)

    command_series = []
    for index, wheel in enumerate(WHEELS):
        command_series.append(
            {
                "label": "%s command" % wheel,
                "color": SERIES_COLORS[index],
                "points": [(row["bag_time_s"] - offset, row["t_ff"][index]) for row in commands],
            }
        )
    line_plot_svg(output_directory / "wheel_torque_command.svg", "Four-wheel MIT torque command", "t_ff (Nm)", command_series, phases, x_limits)

    feedback_series = []
    speed_series = []
    for index, wheel in enumerate(WHEELS):
        feedback_series.append(
            {
                "label": "%s feedback" % wheel,
                "color": SERIES_COLORS[index],
                "points": [(row["bag_time_s"] - offset, row["motors"][index]["output_torque_nm"]) for row in motors],
            }
        )
        speed_series.append(
            {
                "label": "%s raw speed" % wheel,
                "color": SERIES_COLORS[index],
                "points": [(row["bag_time_s"] - offset, row["motors"][index]["output_speed_rad_s"]) for row in motors],
            }
        )
    line_plot_svg(output_directory / "wheel_torque_feedback.svg", "Four-wheel reported output torque", "output torque (Nm)", feedback_series, phases, x_limits)
    line_plot_svg(output_directory / "wheel_speed.svg", "Four-wheel reported output speed", "output speed (rad/s)", speed_series, phases, x_limits)

    trajectory_svg(
        output_directory / "odom_trajectory.svg",
        odom,
        metrics["odom"]["circle_fit"],
        metrics["odom"]["pose_available"],
        offset,
        x_limits,
    )


def parameter_change_table(differences):
    rows = []
    for item in differences:
        ratio = ""
        if item["ratio"] is not None:
            ratio = "%.2fx" % item["ratio"]
        rows.append(
            "| `%s` | %s | %s | %s |"
            % (
                item["parameter"],
                format_number(item["baseline"], 6),
                format_number(item["recorded"], 6),
                ratio,
            )
        )
    return "\n".join(rows)


def write_summary_markdown(path, bag, metrics, differences):
    yaw = metrics["yaw_rate"]
    odom = metrics["odom"]
    circle = odom["circle_fit"]
    lines = [
        "# Drift rosbag analysis",
        "",
        "- Bag: `%s`" % bag["db_path"],
        "- Bag duration: %.3f s" % metrics["bag_duration_s"],
        "- Active controller duration: %.3f s" % metrics["active_duration_s"],
        "- Hold duration: %.3f s" % metrics["hold_duration_s"],
        "",
        "## Recorded YAML changes",
        "",
        "| Parameter | Baseline | Recorded | Ratio |",
        "| --- | ---: | ---: | ---: |",
        parameter_change_table(differences),
        "",
        "## Hold-phase metrics",
        "",
        "- Yaw reference: %.4f rad/s" % yaw["reference_mean_rad_s"],
        "- Yaw rate: mean %.4f, std %.4f, range [%.4f, %.4f] rad/s"
        % (yaw["mean_rad_s"], yaw["std_rad_s"], yaw["min_rad_s"], yaw["max_rad_s"]),
        "- Yaw tracking RMSE: %.4f rad/s" % yaw["rmse_rad_s"],
        "- Estimated yaw oscillation frequency: %s Hz" % format_number(yaw["oscillation_frequency_hz"], 3),
        "- Odom speed: mean %.4f, std %.4f, range [%.4f, %.4f] m/s"
        % (odom["mean_speed_m_s"], odom["std_speed_m_s"], odom["min_speed_m_s"], odom["max_speed_m_s"]),
        "- Distance during hold: %s m" % format_number(odom["distance_during_hold_m"], 3),
        "- Published command saturation fraction: %.1f%%" % (100.0 * metrics["command"]["published_saturation_fraction"]),
        "- Raw yaw-correction saturation fraction: %.1f%%" % (100.0 * metrics["command"]["raw_yaw_correction_saturation_fraction"]),
        "- Predicted raw wheel-torque limiting fraction: %.1f%%" % (100.0 * metrics["command"]["predicted_raw_torque_limit_fraction"]),
        "- Command slew-limit fraction: %.1f%%" % (100.0 * metrics["command"]["slew_limit_fraction"]),
    ]
    if not odom["pose_available"]:
        lines.append(
            "- Odom pose is unavailable: x, y, and pose yaw stayed exactly zero for the whole bag. Only odom twist velocity/angular velocity can be analyzed."
        )
    if circle:
        lines.append(
            "- Odom circle fit: radius %.4f m, residual %.4f m, center (%.4f, %.4f) m"
            % (circle["radius_m"], circle["rms_residual_m"], circle["center_x"], circle["center_y"])
        )
    lines.extend(["", "### Four-wheel hold statistics", ""])
    lines.append("| Wheel | command mean±std (Nm) | feedback mean±std (Nm) | speed mean±std (rad/s) | speed range (rad/s) |")
    lines.append("| --- | ---: | ---: | ---: | ---: |")
    for wheel in WHEELS:
        command = metrics["command"]["per_wheel"][wheel]
        feedback = metrics["motor_feedback_torque"][wheel]
        speed_item = metrics["motor_speed"][wheel]
        lines.append(
            "| %s | %.3f±%.3f | %.3f±%.3f | %.2f±%.2f | [%.2f, %.2f] |"
            % (
                wheel,
                command["mean_nm"],
                command["std_nm"],
                feedback["mean_nm"],
                feedback["std_nm"],
                speed_item["mean_rad_s"],
                speed_item["std_rad_s"],
                speed_item["min_rad_s"],
                speed_item["max_rad_s"],
            )
        )
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            "- The recorded yaw gains are much larger than the baseline. When the yaw error or its derivative changes sign, the raw requested differential torque reaches the configured correction/torque limits and is then shaped mainly by the torque slew limiter.",
            "- A high slew-limit fraction together with a large yaw-rate standard deviation is evidence of a limit-cycle rather than smooth steady-state tracking.",
            "- Increase steady longitudinal drive with the left/right hold feedforward pair; do not use very large yaw Kp/Kd as a substitute for base drive torque.",
            "- Motor speed signs are plotted exactly as reported by the four motor controllers. Opposite signs on the two vehicle sides can be caused by mirrored motor installation and should not be converted to absolute values in the raw-data plot.",
            "",
        ]
    )
    Path(path).write_text("\n".join(lines), encoding="utf-8")


def write_html_report(path, metrics, differences):
    yaw = metrics["yaw_rate"]
    odom = metrics["odom"]
    difference_rows = "".join(
        "<tr><td><code>%s</code></td><td>%s</td><td>%s</td><td>%s</td></tr>"
        % (
            html.escape(item["parameter"]),
            html.escape(format_number(item["baseline"], 6)),
            html.escape(format_number(item["recorded"], 6)),
            "" if item["ratio"] is None else "%.2fx" % item["ratio"],
        )
        for item in differences
    )
    document = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Drift rosbag analysis</title>
<style>
:root { color-scheme: light dark; font-family: system-ui, sans-serif; }
body { max-width: 1440px; margin: 0 auto; padding: 24px; line-height: 1.45; }
.metrics { display: grid; grid-template-columns: repeat(auto-fit,minmax(210px,1fr)); gap: 12px; }
.metric { border: 1px solid color-mix(in srgb,currentColor 20%%,transparent); border-radius: 8px; padding: 12px; }
.metric span { display: block; opacity: .68; font-size: .85rem; }
.metric strong { font-size: 1.25rem; font-weight: 500; }
table { width: 100%%; border-collapse: collapse; }
th,td { text-align: left; border-bottom: 1px solid color-mix(in srgb,currentColor 18%%,transparent); padding: 7px; }
.plot { width: 100%%; height: auto; margin: 12px 0 28px; }
.trajectory { max-width: 760px; }
code { overflow-wrap: anywhere; }
</style>
</head>
<body>
<h1>Drift rosbag analysis</h1>
<div class="metrics">
  <div class="metric"><span>Hold yaw mean ± std</span><strong>%.3f ± %.3f rad/s</strong></div>
  <div class="metric"><span>Yaw tracking RMSE</span><strong>%.3f rad/s</strong></div>
  <div class="metric"><span>Hold odom speed</span><strong>%.3f ± %.3f m/s</strong></div>
  <div class="metric"><span>Raw correction saturation</span><strong>%.1f%%</strong></div>
  <div class="metric"><span>Slew-limit activity</span><strong>%.1f%%</strong></div>
</div>
<h2>Recorded YAML changes</h2>
<table><thead><tr><th>Parameter</th><th>Baseline</th><th>Recorded</th><th>Ratio</th></tr></thead><tbody>%s</tbody></table>
<h2>Odometry</h2><img class="plot trajectory" src="odom_trajectory.svg" alt="Odometry XY trajectory and fitted hold circle">
<h2>Yaw rate</h2><img class="plot" src="yaw_rate.svg" alt="Yaw-rate tracking time series">
<h2>Velocity</h2><img class="plot" src="velocity.svg" alt="Odometry velocity time series">
<h2>Four-wheel torque command</h2><img class="plot" src="wheel_torque_command.svg" alt="Four-wheel MIT torque commands">
<h2>Four-wheel reported torque</h2><img class="plot" src="wheel_torque_feedback.svg" alt="Four-wheel motor torque feedback">
<h2>Four-wheel speed</h2><img class="plot" src="wheel_speed.svg" alt="Four-wheel motor output speed">
</body>
</html>
""" % (
        yaw["mean_rad_s"],
        yaw["std_rad_s"],
        yaw["rmse_rad_s"],
        odom["mean_speed_m_s"],
        odom["std_speed_m_s"],
        100.0 * metrics["command"]["raw_yaw_correction_saturation_fraction"],
        100.0 * metrics["command"]["slew_limit_fraction"],
        difference_rows,
    )
    Path(path).write_text(document, encoding="utf-8")


def analyze(args):
    db_path = locate_db3(args.bag)
    bag_directory = db_path.parent
    output_directory = Path(args.output).expanduser().resolve() if args.output else bag_directory / "analysis"
    output_directory.mkdir(parents=True, exist_ok=True)

    recorded_yaml = Path(args.recorded_yaml).expanduser().resolve() if args.recorded_yaml else bag_directory / "drift_imu_only_pid.yaml"
    baseline_yaml = Path(args.baseline_yaml).expanduser().resolve() if args.baseline_yaml else None
    recorded_parameters = load_ros_parameter_yaml(recorded_yaml) if recorded_yaml.is_file() else {}
    baseline_parameters = load_ros_parameter_yaml(baseline_yaml) if baseline_yaml and baseline_yaml.is_file() else {}
    differences = compare_parameters(baseline_parameters, recorded_parameters) if baseline_parameters else []

    bag = load_bag(db_path)
    metrics = calculate_metrics(bag, recorded_parameters)
    export_csv_files(output_directory, bag, metrics["active_start_s"])
    make_plots(output_directory, bag, metrics)
    write_summary_markdown(output_directory / "summary.md", bag, metrics, differences)
    write_html_report(output_directory / "report.html", metrics, differences)
    (output_directory / "summary.json").write_text(
        json.dumps(
            json_compatible(
                {"metrics": metrics, "parameter_differences": differences}
            ),
            indent=2,
            ensure_ascii=False,
            allow_nan=False,
        ),
        encoding="utf-8",
    )
    print("Analysis written to %s" % output_directory)
    print("Report: %s" % (output_directory / "report.html"))
    print("Summary: %s" % (output_directory / "summary.md"))


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="Decode and plot a Tianbot drift rosbag2 SQLite3 recording."
    )
    parser.add_argument("bag", help="rosbag directory or .db3 path")
    parser.add_argument("--output", help="output directory (default: BAG/analysis)")
    parser.add_argument(
        "--baseline-yaml",
        help="original drift_imu_only_pid.yaml for parameter comparison",
    )
    parser.add_argument(
        "--recorded-yaml",
        help="YAML used for the recording (default: BAG/drift_imu_only_pid.yaml)",
    )
    return parser


def main(argv=None):
    args = build_argument_parser().parse_args(argv)
    analyze(args)


if __name__ == "__main__":
    main()
