#!/usr/bin/env python3
"""Convert YCTC H.264 frames and SEI/IMU metadata into a Kalibr ROS bag.

The parser accepts the legacy V1 record, the 72-byte V2 record, and the
88-byte V3 record.  V3 carries exposure start timestamps separately from the
start-line/timing-reference timestamps; image messages use the per-camera
exposure center, ``exposure_start_pts_us + exposure_time_us / 2``.
"""

import argparse
import csv
import json
import math
import os
import re
import statistics
from dataclasses import dataclass

import cv2
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header


G_TO_MS2 = 9.80665
ACC_G_PER_LSB = 4.0 / 32768.0
GYRO_DPS_PER_LSB = 1000.0 / 32768.0
DPS_TO_RAD = math.pi / 180.0

DUMP_HEADER_RE = re.compile(r"records_found=(\d+) invalid_records=(\d+)")
DUMP_RECORD_RE = re.compile(r"^\s*\[(?P<record_index>\d+)\]\s*(?P<body>.*)$")
DUMP_LAYOUT_RE = re.compile(r"(?:^|\s)layout=(?P<layout>V[123])(?:\s|$)")
DUMP_FIELD_RE = re.compile(
    r"(?P<key>[A-Za-z][A-Za-z0-9_]*)=(?P<value>[-+]?(?:0[xX][0-9A-Fa-f]+|\d+))"
)


@dataclass(frozen=True)
class FrameMetadata:
    record_index: int
    user_data_seq: int
    frame_meta_generation: int
    left_trigger_index: int
    right_trigger_index: int
    left_exposure_start_us: int
    right_exposure_start_us: int
    left_exposure_time_us: int
    right_exposure_time_us: int
    version: int = 0
    layout: str = ""
    left_start_line_rx_pts_us: int = 0
    right_start_line_rx_pts_us: int = 0
    left_pwm_rise_pts_us: int = 0
    right_video_frame_time_ref_pts_us: int = 0

    @property
    def left_center_ns(self):
        return self.left_exposure_start_us * 1000 + self.left_exposure_time_us * 500

    @property
    def right_center_ns(self):
        return self.right_exposure_start_us * 1000 + self.right_exposure_time_us * 500


@dataclass(frozen=True)
class ImuSample:
    pts_ns: int
    gyro: tuple
    acc: tuple


@dataclass(frozen=True)
class DumpRecord:
    """Metadata printed by video_user_data_dump for one YCTC record.

    V1/V2 dumps did not always print a layout name.  The parser therefore
    keeps the inferred version as well as the fields shared by all layouts.
    Missing fields are represented by zero and can be filled from the CSV
    emitted by newer dump tools.
    """

    record_index: int
    version: int
    layout: str
    size: int
    left_start_line_rx_pts_us: int
    right_start_line_rx_pts_us: int
    left_pwm_rise_pts_us: int
    right_video_frame_time_ref_pts_us: int
    left_exposure_time_us: int
    right_exposure_time_us: int
    left_exposure_start_pts_us: int
    right_exposure_start_pts_us: int
    left_gpio_trigger_index: int
    right_gpio_trigger_index: int


def _infer_dump_layout(layout, fields):
    if layout in ("V1", "V2", "V3"):
        return layout, int(layout[1:])
    if (
        ("left_start_line_rx_pts_us" in fields or "right_start_line_rx_pts_us" in fields)
        and (
            "left_pwm_rise_pts_us" in fields
            or "right_video_frame_time_ref_pts_us" in fields
        )
    ):
        return "V3", 3
    if "left_pwm_rise_pts_us" in fields or "right_video_frame_time_ref_pts_us" in fields:
        return "V2", 2
    if (
        "left_v1_pts_us" in fields
        or "right_v1_pts_us" in fields
        or "left_pts" in fields
        or "right_pts" in fields
    ):
        return "V1", 1
    if "left_start_line_rx_pts_us" in fields or "right_start_line_rx_pts_us" in fields:
        return "V1", 1
    return "", 0


def _parse_integer(value, name):
    try:
        return int(value, 0)
    except ValueError:
        try:
            return int(value, 10)
        except ValueError as error:
            raise RuntimeError(f"invalid integer for {name}: {value!r}") from error


def _parse_dump_record(line):
    match = DUMP_RECORD_RE.match(line)
    if match is None:
        return None

    fields = {}
    for field_match in DUMP_FIELD_RE.finditer(match.group("body")):
        fields[field_match.group("key")] = _parse_integer(
            field_match.group("value"), field_match.group("key")
        )
    layout_match = DUMP_LAYOUT_RE.search(match.group("body"))
    layout = layout_match.group("layout") if layout_match is not None else ""
    layout, version = _infer_dump_layout(layout, fields)

    def field(name):
        return fields.get(name, 0)

    return DumpRecord(
        record_index=_parse_integer(match.group("record_index"), "record_index"),
        version=version,
        layout=layout,
        size=field("size"),
        left_start_line_rx_pts_us=(
            field("left_start_line_rx_pts_us")
            or field("left_v1_pts_us")
            or field("left_pts")
        ),
        right_start_line_rx_pts_us=(
            field("right_start_line_rx_pts_us")
            or field("right_v1_pts_us")
            or field("right_pts")
        ),
        left_pwm_rise_pts_us=field("left_pwm_rise_pts_us"),
        right_video_frame_time_ref_pts_us=field("right_video_frame_time_ref_pts_us"),
        left_exposure_time_us=field("left_exposure_time_us"),
        right_exposure_time_us=field("right_exposure_time_us"),
        left_exposure_start_pts_us=field("left_exposure_start_pts_us"),
        right_exposure_start_pts_us=field("right_exposure_start_pts_us"),
        left_gpio_trigger_index=field("left_gpio_trigger_index"),
        right_gpio_trigger_index=field("right_gpio_trigger_index"),
    )


def _merge_dump_and_csv_metadata(dump_record, fields, record_index):
    """Merge redundant metadata and reject mismatches between log and CSV."""

    for name in (
        "left_gpio_trigger_index",
        "right_gpio_trigger_index",
        "left_exposure_start_pts_us",
        "right_exposure_start_pts_us",
        "left_exposure_time_us",
        "right_exposure_time_us",
        "left_start_line_rx_pts_us",
        "right_start_line_rx_pts_us",
        "left_pwm_rise_pts_us",
        "right_video_frame_time_ref_pts_us",
    ):
        dump_value = getattr(dump_record, name, 0)
        csv_value = fields.get(name)
        if dump_value and csv_value is not None and dump_value != csv_value:
            raise RuntimeError(
                f"inconsistent {name} between dump log and IMU CSV in record {record_index}"
            )

    return {
        name: fields.get(name, getattr(dump_record, name, 0))
        for name in (
            "left_gpio_trigger_index",
            "right_gpio_trigger_index",
            "left_exposure_start_pts_us",
            "right_exposure_start_pts_us",
            "left_exposure_time_us",
            "right_exposure_time_us",
            "left_start_line_rx_pts_us",
            "right_start_line_rx_pts_us",
            "left_pwm_rise_pts_us",
            "right_video_frame_time_ref_pts_us",
        )
    }


def _validate_v3_metadata(record):
    if record.version != 3:
        return
    if record.left_exposure_start_us <= 0 or record.right_exposure_start_us <= 0:
        raise RuntimeError(
            f"missing V3 exposure-start timestamp in record {record.record_index}"
        )
    if record.left_exposure_time_us < 0 or record.right_exposure_time_us < 0:
        raise RuntimeError(f"invalid V3 exposure duration in record {record.record_index}")
    if record.left_start_line_rx_pts_us < 0 or record.right_start_line_rx_pts_us < 0:
        raise RuntimeError(f"invalid V3 start-line timestamp in record {record.record_index}")


def _read_dump_records(path):
    records = {}
    expected_records = None
    invalid_records = None
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            if expected_records is None:
                header_match = DUMP_HEADER_RE.search(line)
                if header_match:
                    expected_records = int(header_match.group(1))
                    invalid_records = int(header_match.group(2))
            record = _parse_dump_record(line)
            if record is not None:
                if record.record_index in records:
                    raise RuntimeError(
                        f"duplicate video user-data record {record.record_index} in dump log"
                    )
                records[record.record_index] = record

    if expected_records is None or invalid_records is None:
        raise RuntimeError("failed to parse video_user_data_dump summary")
    if invalid_records != 0:
        raise RuntimeError(f"video_user_data_dump reported {invalid_records} invalid records")
    if len(records) != expected_records:
        raise RuntimeError(
            f"dump log contains {len(records)} frame records, expected {expected_records}"
        )
    expected_indices = list(range(1, expected_records + 1))
    if sorted(records) != expected_indices:
        raise RuntimeError("dump log record indices are not contiguous from 1")
    return records, expected_records


def read_exposure_times(path):
    records, expected_records = _read_dump_records(path)
    exposure_times = {}
    for record_index, record in records.items():
        if record.left_exposure_time_us < 0 or record.right_exposure_time_us < 0:
            raise RuntimeError(f"invalid exposure duration in record {record_index}")
        exposure_times[record_index] = (
            record.left_exposure_time_us,
            record.right_exposure_time_us,
        )
    return exposure_times, expected_records


def read_sensor_data(csv_path, dump_log_path):
    dump_records, expected_records = _read_dump_records(dump_log_path)
    frame_fields = {}
    raw_imu = {}
    duplicate_imu_rows = 0

    metadata_columns = {
        "user_data_seq",
        "frame_meta_generation",
        "left_gpio_trigger_index",
        "right_gpio_trigger_index",
        "left_start_line_rx_pts_us",
        "right_start_line_rx_pts_us",
        "left_pwm_rise_pts_us",
        "right_video_frame_time_ref_pts_us",
        "left_exposure_start_pts_us",
        "right_exposure_start_pts_us",
        "left_exposure_time_us",
        "right_exposure_time_us",
    }

    def row_int(row, name):
        value = row.get(name)
        if value is None or value == "":
            return None
        try:
            return _parse_integer(value, name)
        except RuntimeError as error:
            raise RuntimeError(f"invalid {name} in record {row.get('record_index')}") from error

    with open(csv_path, newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or "record_index" not in reader.fieldnames:
            raise RuntimeError("IMU CSV is missing record_index")
        missing_metadata = {
            "user_data_seq",
            "frame_meta_generation",
            "left_gpio_trigger_index",
            "right_gpio_trigger_index",
        }.difference(reader.fieldnames)
        if missing_metadata:
            names = ", ".join(sorted(missing_metadata))
            raise RuntimeError(f"IMU CSV is missing metadata column(s): {names}")

        for row in reader:
            record_index = row_int(row, "record_index")
            if record_index is None or record_index not in dump_records:
                raise RuntimeError(f"IMU CSV references unknown record {record_index}")
            fields = frame_fields.setdefault(record_index, {})
            for name in metadata_columns:
                value = row_int(row, name)
                if value is None:
                    continue
                previous = fields.get(name)
                if previous is not None and previous != value:
                    raise RuntimeError(f"inconsistent {name} in record {record_index}")
                fields[name] = value

            try:
                pts_us = _parse_integer(row["pts_us"], "pts_us")
            except (KeyError, TypeError, RuntimeError) as error:
                raise RuntimeError(f"invalid pts_us in record {record_index}") from error
            sample_type = row["sample_type"]
            if sample_type not in ("gyro", "acc"):
                raise RuntimeError(f"unknown IMU sample type: {sample_type}")
            try:
                raw_values = (
                    _parse_integer(row["x"], "x"),
                    _parse_integer(row["y"], "y"),
                    _parse_integer(row["z"], "z"),
                )
            except (KeyError, TypeError, RuntimeError) as error:
                raise RuntimeError(f"invalid IMU sample in record {record_index}") from error
            entry = raw_imu.setdefault(pts_us, {})
            if sample_type in entry:
                if entry[sample_type] != raw_values:
                    raise RuntimeError(f"conflicting duplicate {sample_type} sample at {pts_us} us")
                duplicate_imu_rows += 1
            else:
                entry[sample_type] = raw_values

    frames = []
    for record_index in sorted(dump_records):
        dump_record = dump_records[record_index]
        fields = frame_fields.get(record_index, {})

        if dump_record.version not in (1, 2, 3):
            raise RuntimeError(
                f"cannot determine YCTC user-data layout for record {record_index}"
            )

        def metadata_value(name, dump_value=0, zero_is_missing=False):
            value = fields.get(name)
            if value is None or (zero_is_missing and value == 0):
                return dump_value
            return value

        if dump_record.version == 1:
            # V1 calls the legacy image timestamp ``left_pts/right_pts``.
            # Preserve that behavior while using the explicit V2/V3 exposure
            # start fields whenever they are available.
            left_exposure_start = metadata_value(
                "left_exposure_start_pts_us",
                dump_record.left_start_line_rx_pts_us,
                zero_is_missing=True,
            )
            right_exposure_start = metadata_value(
                "right_exposure_start_pts_us",
                dump_record.right_start_line_rx_pts_us,
                zero_is_missing=True,
            )
        else:
            left_exposure_start = metadata_value(
                "left_exposure_start_pts_us", dump_record.left_exposure_start_pts_us
            )
            right_exposure_start = metadata_value(
                "right_exposure_start_pts_us", dump_record.right_exposure_start_pts_us
            )

        merged = _merge_dump_and_csv_metadata(dump_record, fields, record_index)

        frame = FrameMetadata(
            record_index=record_index,
            user_data_seq=metadata_value("user_data_seq"),
            frame_meta_generation=metadata_value("frame_meta_generation"),
            left_trigger_index=merged["left_gpio_trigger_index"],
            right_trigger_index=merged["right_gpio_trigger_index"],
            left_exposure_start_us=left_exposure_start,
            right_exposure_start_us=right_exposure_start,
            left_exposure_time_us=merged["left_exposure_time_us"],
            right_exposure_time_us=merged["right_exposure_time_us"],
            version=dump_record.version,
            layout=dump_record.layout,
            left_start_line_rx_pts_us=merged["left_start_line_rx_pts_us"],
            right_start_line_rx_pts_us=merged["right_start_line_rx_pts_us"],
            left_pwm_rise_pts_us=merged["left_pwm_rise_pts_us"],
            right_video_frame_time_ref_pts_us=merged[
                "right_video_frame_time_ref_pts_us"
            ],
        )
        if frame.left_exposure_start_us <= 0 or frame.right_exposure_start_us <= 0:
            raise RuntimeError(
                f"missing exposure-start timestamp in record {record_index}; "
                "the calibration workflow requires left/right_exposure_start_pts_us "
                "(V3 or a compatible V2 record)"
            )
        _validate_v3_metadata(frame)
        frames.append(frame)

    incomplete_timestamps = [pts for pts, entry in raw_imu.items() if set(entry) != {"gyro", "acc"}]
    if incomplete_timestamps:
        raise RuntimeError(f"found {len(incomplete_timestamps)} incomplete IMU timestamps")

    imu_samples = []
    for pts_us in sorted(raw_imu):
        entry = raw_imu[pts_us]
        gyro = tuple(value * GYRO_DPS_PER_LSB * DPS_TO_RAD for value in entry["gyro"])
        acc = tuple(value * ACC_G_PER_LSB * G_TO_MS2 for value in entry["acc"])
        imu_samples.append(ImuSample(pts_ns=pts_us * 1000, gyro=gyro, acc=acc))

    return frames, imu_samples, duplicate_imu_rows


def ros_time_from_ns(timestamp_ns):
    return rospy.Time(timestamp_ns // 1_000_000_000, timestamp_ns % 1_000_000_000)


def make_image_msg(image, timestamp_ns, frame_id, seq):
    msg = Image()
    msg.header = Header(seq=seq, stamp=ros_time_from_ns(timestamp_ns), frame_id=frame_id)
    msg.height = image.shape[0]
    msg.width = image.shape[1]
    msg.is_bigendian = 0
    if image.ndim == 2:
        msg.encoding = "mono8"
        msg.step = msg.width
    else:
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
    msg.data = image.tobytes()
    return msg


def make_imu_msg(sample, seq):
    msg = Imu()
    msg.header = Header(seq=seq, stamp=ros_time_from_ns(sample.pts_ns), frame_id="imu0")
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = sample.gyro
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = sample.acc
    msg.orientation_covariance[0] = -1.0
    return msg


def prepare_half(frame, side, scale, mono):
    if frame.shape[1] % 2:
        raise RuntimeError(f"decoded frame has odd width: {frame.shape[1]}")
    half_width = frame.shape[1] // 2
    image = frame[:, :half_width] if side == "left" else frame[:, half_width:]
    if scale != 1.0:
        image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    if mono:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return np.ascontiguousarray(image)


def image_quality_metrics(image, dark_pixel_max, bright_pixel_min):
    if image.ndim == 2:
        gray = image
    else:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = np.ascontiguousarray(gray)
    return {
        "laplacian_variance": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
        "dark_pixel_ratio": float(np.mean(gray <= dark_pixel_max)),
        "bright_pixel_ratio": float(np.mean(gray >= bright_pixel_min)),
    }


def quality_rejection_reasons(left_metrics, right_metrics, args):
    reasons = []
    for side, metrics in (("left", left_metrics), ("right", right_metrics)):
        if metrics["laplacian_variance"] < args.min_laplacian_variance:
            reasons.append(f"{side}_blur")
        if metrics["dark_pixel_ratio"] > args.max_dark_ratio:
            reasons.append(f"{side}_underexposed")
        if metrics["bright_pixel_ratio"] > args.max_bright_ratio:
            reasons.append(f"{side}_overexposed")
    return reasons


def numeric_stats(values):
    if not values:
        return None
    return {
        "min": float(min(values)),
        "median": float(statistics.median(values)),
        "max": float(max(values)),
    }


def write_configuration(output_dir, image_width, image_height, imu_rate_hz, args):
    target_path = os.path.join(output_dir, "target.yaml")
    imu_path = os.path.join(output_dir, "imu.yaml")
    placeholder_path = os.path.join(output_dir, "camchain_from_camera_calib.yaml")

    with open(target_path, "w", encoding="utf-8") as stream:
        stream.write(
            "target_type: 'aprilgrid'\n"
            f"tagCols: {args.tag_cols}\n"
            f"tagRows: {args.tag_rows}\n"
            f"tagSize: {args.tag_size:.9g}\n"
            f"tagSpacing: {args.tag_spacing:.9g}\n"
        )

    with open(imu_path, "w", encoding="utf-8") as stream:
        stream.write(
            "rostopic: /imu0\n"
            f"update_rate: {imu_rate_hz:.9g}\n\n"
            f"accelerometer_noise_density: {args.acc_noise_density:.9g}\n"
            f"accelerometer_random_walk: {args.acc_random_walk:.9g}\n"
            f"gyroscope_noise_density: {args.gyro_noise_density:.9g}\n"
            f"gyroscope_random_walk: {args.gyro_random_walk:.9g}\n"
        )

    with open(placeholder_path, "w", encoding="utf-8") as stream:
        stream.write(
            "# Replace this file with the output from kalibr_calibrate_cameras.\n"
            "# rosrun kalibr kalibr_calibrate_cameras --bag calibration.bag "
            "--topics /cam0/image_raw /cam1/image_raw --models pinhole-radtan pinhole-radtan "
            "--target target.yaml --bag-freq 4 --max-corner-reproj-px 0.85 "
            "--max-corner-filter-rounds 10\n"
            f"# Image size: {image_width}x{image_height}\n"
        )

    return target_path, imu_path, placeholder_path


def count_gaps(values, expected_step=1):
    return sum(b - a != expected_step for a, b in zip(values, values[1:]))


def interval_stats_ns(timestamps_ns):
    periods = [b - a for a, b in zip(timestamps_ns, timestamps_ns[1:])]
    return {
        "median_period_ns": int(statistics.median(periods)),
        "min_period_ns": min(periods),
        "max_period_ns": max(periods),
    }


def convert(args):
    video_path = os.path.abspath(args.video)
    imu_csv_path = os.path.abspath(args.imu_csv)
    dump_log_path = os.path.abspath(args.dump_log)
    output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    frames, imu_samples, duplicate_imu_rows = read_sensor_data(imu_csv_path, dump_log_path)
    if len(frames) < 2 or len(imu_samples) < 2:
        raise RuntimeError("insufficient frame or IMU data")

    left_timestamps_ns = [frame.left_center_ns for frame in frames]
    right_timestamps_ns = [frame.right_center_ns for frame in frames]
    imu_timestamps_ns = [sample.pts_ns for sample in imu_samples]
    if left_timestamps_ns != sorted(left_timestamps_ns):
        raise RuntimeError("left image timestamps are not monotonic")
    if right_timestamps_ns != sorted(right_timestamps_ns):
        raise RuntimeError("right image timestamps are not monotonic")

    imu_periods_ns = [b - a for a, b in zip(imu_timestamps_ns, imu_timestamps_ns[1:])]
    imu_rate_hz = 1_000_000_000.0 / statistics.median(imu_periods_ns)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"failed to open H.264 stream: {video_path}")

    bag_path = os.path.join(output_dir, args.output_bag)
    compression = {
        "none": rosbag.Compression.NONE,
        "bz2": rosbag.Compression.BZ2,
        "lz4": rosbag.Compression.LZ4,
    }[args.compression]
    bag = rosbag.Bag(bag_path, "w", compression=compression)
    frame_timestamp_rows = []
    frame_quality_rows = []
    decoded_frame_count = 0
    stride_candidate_count = 0
    stereo_frame_count = 0
    imu_index = 0
    first_shape = None
    quality_rejection_counts = {}
    quality_metric_values = {
        "left_laplacian_variance": [],
        "right_laplacian_variance": [],
        "left_dark_pixel_ratio": [],
        "right_dark_pixel_ratio": [],
        "left_bright_pixel_ratio": [],
        "right_bright_pixel_ratio": [],
    }

    try:
        while True:
            ok, decoded = cap.read()
            if not ok:
                break
            if decoded_frame_count >= len(frames):
                raise RuntimeError("decoded more frames than available SEI records")

            metadata = frames[decoded_frame_count]
            if decoded_frame_count % args.image_stride == 0:
                left = prepare_half(decoded, "left", args.scale, args.mono)
                right = prepare_half(decoded, "right", args.scale, args.mono)
                if first_shape is None:
                    first_shape = left.shape
                if right.shape != left.shape:
                    raise RuntimeError(
                        f"left/right image shape mismatch: {left.shape} != {right.shape}"
                    )

                left_quality = image_quality_metrics(
                    left, args.dark_pixel_max, args.bright_pixel_min
                )
                right_quality = image_quality_metrics(
                    right, args.dark_pixel_max, args.bright_pixel_min
                )
                rejection_reasons = quality_rejection_reasons(
                    left_quality, right_quality, args
                )
                accepted = not rejection_reasons

                quality_values = {
                    "left_laplacian_variance": left_quality["laplacian_variance"],
                    "right_laplacian_variance": right_quality["laplacian_variance"],
                    "left_dark_pixel_ratio": left_quality["dark_pixel_ratio"],
                    "right_dark_pixel_ratio": right_quality["dark_pixel_ratio"],
                    "left_bright_pixel_ratio": left_quality["bright_pixel_ratio"],
                    "right_bright_pixel_ratio": right_quality["bright_pixel_ratio"],
                }
                for name, value in quality_values.items():
                    quality_metric_values[name].append(value)
                for reason in rejection_reasons:
                    quality_rejection_counts[reason] = (
                        quality_rejection_counts.get(reason, 0) + 1
                    )

                frame_quality_rows.append(
                    {
                        "stride_candidate_index": stride_candidate_count,
                        "decoded_frame_index": decoded_frame_count,
                        "record_index": metadata.record_index,
                        "left_exposure_center_ns": metadata.left_center_ns,
                        "right_exposure_center_ns": metadata.right_center_ns,
                        **quality_values,
                        "accepted": int(accepted),
                        "selected_stereo_frame_index": (
                            stereo_frame_count if accepted else ""
                        ),
                        "rejection_reasons": ";".join(rejection_reasons),
                    }
                )
                stride_candidate_count += 1

                if accepted:
                    image_events = sorted(
                        (
                            (metadata.left_center_ns, "/cam0/image_raw", "cam0", left),
                            (metadata.right_center_ns, "/cam1/image_raw", "cam1", right),
                        ),
                        key=lambda event: event[0],
                    )
                    for timestamp_ns, topic, frame_id, image in image_events:
                        while (
                            imu_index < len(imu_samples)
                            and imu_samples[imu_index].pts_ns <= timestamp_ns
                        ):
                            imu_msg = make_imu_msg(imu_samples[imu_index], imu_index)
                            bag.write("/imu0", imu_msg, imu_msg.header.stamp)
                            imu_index += 1
                        image_msg = make_image_msg(
                            image, timestamp_ns, frame_id, stereo_frame_count
                        )
                        bag.write(topic, image_msg, image_msg.header.stamp)

                    frame_timestamp_rows.append(
                        {
                            "selected_stereo_frame_index": stereo_frame_count,
                            "stride_candidate_index": stride_candidate_count - 1,
                            "decoded_frame_index": decoded_frame_count,
                            "record_index": metadata.record_index,
                            "yctc_version": metadata.version,
                            "yctc_layout": metadata.layout,
                            "user_data_seq": metadata.user_data_seq,
                            "left_trigger_index": metadata.left_trigger_index,
                            "right_trigger_index": metadata.right_trigger_index,
                            "left_exposure_start_us": metadata.left_exposure_start_us,
                            "right_exposure_start_us": metadata.right_exposure_start_us,
                            "left_exposure_time_us": metadata.left_exposure_time_us,
                            "right_exposure_time_us": metadata.right_exposure_time_us,
                            "left_exposure_center_ns": metadata.left_center_ns,
                            "right_exposure_center_ns": metadata.right_center_ns,
                            **quality_values,
                        }
                    )
                    stereo_frame_count += 1

            decoded_frame_count += 1

        while imu_index < len(imu_samples):
            imu_msg = make_imu_msg(imu_samples[imu_index], imu_index)
            bag.write("/imu0", imu_msg, imu_msg.header.stamp)
            imu_index += 1
    finally:
        bag.close()
        cap.release()

    if decoded_frame_count != len(frames):
        raise RuntimeError(
            f"decoded {decoded_frame_count} frames but found {len(frames)} SEI records"
        )
    if first_shape is None:
        raise RuntimeError("no stride-selected image candidates were decoded")
    if stereo_frame_count < 2:
        raise RuntimeError(
            "fewer than two stride-selected stereo frames passed the image quality filter"
        )

    timestamps_csv_path = os.path.join(output_dir, "selected_frame_timestamps.csv")
    with open(timestamps_csv_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(frame_timestamp_rows[0]))
        writer.writeheader()
        writer.writerows(frame_timestamp_rows)

    quality_csv_path = os.path.join(output_dir, "frame_quality.csv")
    with open(quality_csv_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(frame_quality_rows[0]))
        writer.writeheader()
        writer.writerows(frame_quality_rows)

    image_height, image_width = first_shape[:2]
    target_path, imu_path, placeholder_path = write_configuration(
        output_dir, image_width, image_height, imu_rate_hz, args
    )

    left_right_skews_ns = [right - left for left, right in zip(left_timestamps_ns, right_timestamps_ns)]
    written_left_timestamps_ns = [
        row["left_exposure_center_ns"] for row in frame_timestamp_rows
    ]
    written_right_timestamps_ns = [
        row["right_exposure_center_ns"] for row in frame_timestamp_rows
    ]
    written_left_right_skews_ns = [
        right - left
        for left, right in zip(written_left_timestamps_ns, written_right_timestamps_ns)
    ]
    trigger_indices = [frame.left_trigger_index for frame in frames]
    user_data_sequences = [frame.user_data_seq for frame in frames]
    summary = {
        "video_path": video_path,
        "imu_csv_path": imu_csv_path,
        "dump_log_path": dump_log_path,
        "bag_path": bag_path,
        "target_yaml": target_path,
        "imu_yaml": imu_path,
        "camchain_placeholder": placeholder_path,
        "selected_frame_timestamps_csv": timestamps_csv_path,
        "frame_quality_csv": quality_csv_path,
        "topics": {"cam0": "/cam0/image_raw", "cam1": "/cam1/image_raw", "imu": "/imu0"},
        "image_timestamp_policy": "per-camera exposure_start_pts_us plus half exposure_time_us",
        "yctc_versions": sorted({frame.version for frame in frames}),
        "yctc_layouts": sorted({frame.layout for frame in frames}),
        "yctc_timestamp_fields": {
            "image_start": "left/right_exposure_start_pts_us",
            "image_center": "exposure_start_pts_us + exposure_time_us / 2",
            "v3_start_line_fields": "left/right_start_line_rx_pts_us (diagnostic only)",
            "v3_timing_reference_fields": (
                "left_pwm_rise_pts_us/right_video_frame_time_ref_pts_us (diagnostic only)"
            ),
        },
        "decoded_frame_count": decoded_frame_count,
        "sei_record_count": len(frames),
        "stride_candidate_stereo_frame_count": stride_candidate_count,
        "quality_rejected_stereo_frame_count": stride_candidate_count - stereo_frame_count,
        "quality_rejection_rate": float(
            (stride_candidate_count - stereo_frame_count) / stride_candidate_count
        ),
        "written_stereo_frame_count": stereo_frame_count,
        "written_image_count": stereo_frame_count * 2,
        "written_imu_count": len(imu_samples),
        "duplicate_imu_rows_removed": duplicate_imu_rows,
        "image_stride": args.image_stride,
        "scale": args.scale,
        "mono": args.mono,
        "compression": args.compression,
        "image_quality_filter": {
            "pair_policy": "reject the stereo pair when either camera fails",
            "blur_metric": "variance of the 3x3 Laplacian on the output-resolution grayscale image",
            "min_laplacian_variance": args.min_laplacian_variance,
            "dark_pixel_max": args.dark_pixel_max,
            "max_dark_pixel_ratio": args.max_dark_ratio,
            "bright_pixel_min": args.bright_pixel_min,
            "max_bright_pixel_ratio": args.max_bright_ratio,
            "rejection_counts_by_reason": dict(sorted(quality_rejection_counts.items())),
            "candidate_metric_statistics": {
                name: numeric_stats(values)
                for name, values in quality_metric_values.items()
            },
        },
        "image_width": image_width,
        "image_height": image_height,
        "image_encoding": "mono8" if args.mono else "bgr8",
        "first_record_index": frames[0].record_index,
        "last_record_index": frames[-1].record_index,
        "first_trigger_index": trigger_indices[0],
        "last_trigger_index": trigger_indices[-1],
        "trigger_gap_count": count_gaps(trigger_indices),
        "user_data_sequence_gap_count": count_gaps(user_data_sequences),
        "frame_meta_generations": sorted({frame.frame_meta_generation for frame in frames}),
        "left_image_timing": interval_stats_ns(left_timestamps_ns),
        "right_image_timing": interval_stats_ns(right_timestamps_ns),
        "written_left_image_timing": interval_stats_ns(written_left_timestamps_ns),
        "written_right_image_timing": interval_stats_ns(written_right_timestamps_ns),
        "left_right_center_skew_ns": {
            "median": int(statistics.median(left_right_skews_ns)),
            "min": min(left_right_skews_ns),
            "max": max(left_right_skews_ns),
        },
        "written_left_right_center_skew_ns": {
            "median": int(statistics.median(written_left_right_skews_ns)),
            "min": min(written_left_right_skews_ns),
            "max": max(written_left_right_skews_ns),
        },
        "first_left_image_timestamp_ns": left_timestamps_ns[0],
        "last_left_image_timestamp_ns": left_timestamps_ns[-1],
        "first_written_left_image_timestamp_ns": written_left_timestamps_ns[0],
        "last_written_left_image_timestamp_ns": written_left_timestamps_ns[-1],
        "first_imu_timestamp_ns": imu_timestamps_ns[0],
        "last_imu_timestamp_ns": imu_timestamps_ns[-1],
        "measured_imu_rate_hz": imu_rate_hz,
        "imu_timing": interval_stats_ns(imu_timestamps_ns),
        "imu_scale": {
            "accelerometer_g_per_lsb": ACC_G_PER_LSB,
            "gyroscope_dps_per_lsb": GYRO_DPS_PER_LSB,
        },
        "tag_cols": args.tag_cols,
        "tag_rows": args.tag_rows,
        "tag_size_m": args.tag_size,
        "tag_spacing_ratio": args.tag_spacing,
    }
    summary_path = os.path.join(output_dir, "conversion_summary.json")
    with open(summary_path, "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
    print(json.dumps(summary, indent=2, sort_keys=True))


def main():
    parser = argparse.ArgumentParser(
        description="Convert a YCTC H.264 stream and video_user_data_dump output to a Kalibr bag."
    )
    parser.add_argument("--video", required=True)
    parser.add_argument("--imu-csv", required=True)
    parser.add_argument("--dump-log", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--output-bag", default="calibration_stride3.bag")
    parser.add_argument("--image-stride", type=int, default=3)
    parser.add_argument("--scale", type=float, default=0.5)
    parser.add_argument(
        "--min-laplacian-variance",
        "--blur-threshold",
        type=float,
        default=100.0,
        help=(
            "reject a stereo pair when either output image has a lower variance-of-Laplacian "
            "sharpness score; use 0 to disable blur rejection"
        ),
    )
    parser.add_argument("--dark-pixel-max", type=int, default=5)
    parser.add_argument("--max-dark-ratio", type=float, default=0.85)
    parser.add_argument("--bright-pixel-min", type=int, default=250)
    parser.add_argument("--max-bright-ratio", type=float, default=0.85)
    parser.set_defaults(mono=True)
    parser.add_argument("--mono", dest="mono", action="store_true")
    parser.add_argument("--color", dest="mono", action="store_false")
    parser.add_argument("--compression", choices=("none", "bz2", "lz4"), default="lz4")
    parser.add_argument("--tag-cols", type=int, default=6)
    parser.add_argument("--tag-rows", type=int, default=6)
    parser.add_argument("--tag-size", type=float, default=0.0352)
    parser.add_argument("--tag-spacing", type=float, default=0.3)
    parser.add_argument("--acc-noise-density", type=float, default=0.02)
    parser.add_argument("--acc-random-walk", type=float, default=1.92084189233e-05)
    parser.add_argument("--gyro-noise-density", type=float, default=0.002)
    parser.add_argument("--gyro-random-walk", type=float, default=2.26588533832e-06)
    args = parser.parse_args()

    if args.image_stride < 1:
        raise SystemExit("--image-stride must be >= 1")
    if args.scale <= 0.0:
        raise SystemExit("--scale must be > 0")
    if args.min_laplacian_variance < 0.0:
        raise SystemExit("--min-laplacian-variance must be >= 0")
    if not 0 <= args.dark_pixel_max < args.bright_pixel_min <= 255:
        raise SystemExit(
            "pixel thresholds must satisfy 0 <= --dark-pixel-max < "
            "--bright-pixel-min <= 255"
        )
    if not 0.0 <= args.max_dark_ratio <= 1.0:
        raise SystemExit("--max-dark-ratio must be in [0, 1]")
    if not 0.0 <= args.max_bright_ratio <= 1.0:
        raise SystemExit("--max-bright-ratio must be in [0, 1]")
    if args.tag_size <= 0.0 or args.tag_spacing < 0.0:
        raise SystemExit("invalid AprilGrid dimensions")
    convert(args)


if __name__ == "__main__":
    main()
