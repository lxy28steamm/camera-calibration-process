#!/usr/bin/env python3
"""Run the YCTC stereo camera/IMU Kalibr workflow.

By default the workflow calibrates the stereo camera chain first. Supplying a
pre-calibrated chain with ``--camchain`` skips that stage, keeps the camera
intrinsics and camera-to-camera extrinsics fixed, and estimates only the
camera-to-IMU extrinsics and temporal offsets.

The source H.264 file is never modified. If pre-IDR VCL NAL units are found,
the script creates a trimmed copy in the output directory while preserving the
non-VCL prefix immediately before the first IDR (YCTC SEI, SPS and PPS).

This workflow intentionally keeps the current timestamp association:
decoded image N uses SEI record N, image timestamps are exposure centers, and
IMU sample PTS values are not adjusted for filter group delay.

The bag converter accepts YCTC V1/V2/V3 records.  For V3, image timestamps
are computed independently for each camera as exposure start plus half of the
reported exposure duration.
"""

from __future__ import annotations

import argparse
import datetime as dt
import filecmp
import json
import math
import mmap
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:
    yaml = None


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_DUMP_TOOL = Path(
    os.environ.get(
        "KALIBR_DUMP_TOOL",
        str(SCRIPT_DIR.parent / "bin" / "video_user_data_dump"),
    )
)
DEFAULT_DOCKER_IMAGE = os.environ.get(
    "KALIBR_IMAGE", "kalibr-h264-imu-demo:20260805"
)
KALIBR_SETUP = "/catkin_ws/devel/setup.bash"
KALIBR_SOURCE_ROOT = SCRIPT_DIR.parent / "kalibr_source"
KALIBR_SOURCE_MOUNT = "/catkin_ws/src/kalibr"
PATCHED_KALIBR_CAMERA_CALIBRATOR = os.environ.get(
    "KALIBR_CAMERA_CALIBRATOR",
    "/catkin_ws/src/kalibr/aslam_offline_calibration/kalibr/python/"
    "kalibr_calibrate_cameras",
)
DUMP_SUMMARY_RE = re.compile(r"records_found=(\d+) invalid_records=(\d+)")
DUMP_LAYOUT_RE = re.compile(r"(?:^|\s)layout=(V[123])(?:\s|$)")
DUMP_START_LINE_RE = re.compile(
    r"(?:^|\s)(?:left_start_line_rx_pts_us|right_start_line_rx_pts_us)="
)
DUMP_TIMING_REF_RE = re.compile(
    r"(?:^|\s)(?:left_pwm_rise_pts_us|right_video_frame_time_ref_pts_us)="
)
DUMP_LEGACY_LAYOUT_RE = re.compile(
    r"(?:^|\s)(?:left_pwm_rise_pts_us|right_video_frame_time_ref_pts_us)="
)
DUMP_V1_LAYOUT_RE = re.compile(r"(?:^|\s)(?:left_v1_pts_us|right_v1_pts_us|left_pts)=")
TIMESHIFT_RE = re.compile(
    r"^\s*timeshift_cam_imu:\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*$"
)
STAGES = ("dump", "bag", "camera", "imu")


class PipelineError(RuntimeError):
    pass


@dataclass(frozen=True)
class H264Inspection:
    file_size: int
    first_idr_offset: int
    trim_offset: int | None
    pre_idr_vcl_count: int
    prefix_nal_types: tuple[int, ...]
    prefix_has_yctc_sei: bool
    prefix_has_sps: bool
    prefix_has_pps: bool


def now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).astimezone().isoformat(timespec="seconds")


def format_command(command: list[str]) -> str:
    return " ".join(shlex.quote(str(part)) for part in command)


def nonempty_file(path: Path) -> bool:
    return path.is_file() and path.stat().st_size > 0


def path_is_within(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def find_annexb_start(data: mmap.mmap, position: int) -> tuple[int, int] | None:
    marker = data.find(b"\x00\x00\x01", position)
    if marker < 0:
        return None
    if marker > 0 and data[marker - 1] == 0:
        return marker - 1, 4
    return marker, 3


def inspect_h264(path: Path) -> H264Inspection:
    """Inspect the NAL prefix and identify a safe suffix beginning before IDR."""
    with path.open("rb") as stream:
        if stream.seek(0, os.SEEK_END) == 0:
            raise PipelineError(f"H.264 file is empty: {path}")
        stream.seek(0)
        with mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as data:
            current = find_annexb_start(data, 0)
            if current is None:
                raise PipelineError(f"not an Annex-B H.264 stream: {path}")

            pre_idr_vcl_count = 0
            prefix_start = None
            prefix_types: list[int] = []
            prefix_has_yctc = False
            prefix_has_sps = False
            prefix_has_pps = False

            while current is not None:
                start, start_code_size = current
                header_offset = start + start_code_size
                if header_offset >= len(data):
                    break
                following = find_annexb_start(data, header_offset + 1)
                nal_end = following[0] if following is not None else len(data)
                nal_type = data[header_offset] & 0x1F

                if nal_type == 5:
                    trim_offset = prefix_start if pre_idr_vcl_count > 0 else None
                    inspection = H264Inspection(
                        file_size=len(data),
                        first_idr_offset=start,
                        trim_offset=trim_offset,
                        pre_idr_vcl_count=pre_idr_vcl_count,
                        prefix_nal_types=tuple(prefix_types),
                        prefix_has_yctc_sei=prefix_has_yctc,
                        prefix_has_sps=prefix_has_sps,
                        prefix_has_pps=prefix_has_pps,
                    )
                    if trim_offset is not None:
                        missing = []
                        if not prefix_has_yctc:
                            missing.append("YCTC SEI")
                        if not prefix_has_sps:
                            missing.append("SPS")
                        if not prefix_has_pps:
                            missing.append("PPS")
                        if missing:
                            names = ", ".join(missing)
                            raise PipelineError(
                                "cannot safely trim before the first IDR: the non-VCL prefix "
                                f"after the last pre-IDR frame is missing {names}"
                            )
                    return inspection

                if nal_type in (1, 2, 3, 4):
                    pre_idr_vcl_count += 1
                    prefix_start = None
                    prefix_types = []
                    prefix_has_yctc = False
                    prefix_has_sps = False
                    prefix_has_pps = False
                else:
                    if prefix_start is None:
                        prefix_start = start
                    prefix_types.append(nal_type)
                    if nal_type == 6 and b"YCTC" in data[header_offset + 1 : nal_end]:
                        prefix_has_yctc = True
                    elif nal_type == 7:
                        prefix_has_sps = True
                    elif nal_type == 8:
                        prefix_has_pps = True

                current = following

    raise PipelineError(f"no IDR NAL unit found in H.264 stream: {path}")


def copy_suffix(source: Path, destination: Path, offset: int) -> None:
    temporary = destination.with_name(destination.name + ".partial")
    if temporary.exists():
        temporary.unlink()
    try:
        with source.open("rb") as source_stream, temporary.open("wb") as destination_stream:
            source_stream.seek(offset)
            shutil.copyfileobj(source_stream, destination_stream, length=4 * 1024 * 1024)
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()


def copy_file_atomic(source: Path, destination: Path) -> None:
    """Copy a regular file without exposing a partially written destination."""
    if destination.exists() and source.samefile(destination):
        return

    temporary = destination.with_name(destination.name + ".partial")
    if temporary.exists():
        temporary.unlink()
    try:
        shutil.copyfile(source, temporary)
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()


def input_identity(path: Path) -> dict:
    stat = path.stat()
    return {
        "path": str(path),
        "size": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
    }


def load_yaml_mapping(path: Path) -> dict[str, Any]:
    if yaml is None:
        raise PipelineError(
            "PyYAML is required only when --camchain is used; install it with "
            "python3 -m pip install --user PyYAML"
        )
    try:
        with path.open(encoding="utf-8") as stream:
            value = yaml.safe_load(stream)
    except yaml.YAMLError as error:
        raise PipelineError(f"invalid YAML in camera chain {path}: {error}") from error
    if not isinstance(value, dict):
        raise PipelineError(f"camera chain must contain a YAML mapping: {path}")
    return value


def finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def validate_numeric_sequence(
    value: Any, *, name: str, lengths: tuple[int, ...] | None = None
) -> list[Any]:
    if not isinstance(value, list):
        raise PipelineError(f"{name} must be a YAML list")
    if lengths is not None and len(value) not in lengths:
        expected = " or ".join(str(length) for length in lengths)
        raise PipelineError(f"{name} must contain {expected} values, got {len(value)}")
    if not all(finite_number(item) for item in value):
        raise PipelineError(f"{name} contains a non-numeric or non-finite value")
    return value


def validate_transform(value: Any, *, name: str) -> None:
    if not isinstance(value, list) or len(value) != 4:
        raise PipelineError(f"{name} must be a 4x4 matrix")
    for row_index, row in enumerate(value):
        validate_numeric_sequence(row, name=f"{name}[{row_index}]", lengths=(4,))
    expected_last_row = (0.0, 0.0, 0.0, 1.0)
    if any(abs(float(value[3][index]) - expected) > 1e-6 for index, expected in enumerate(expected_last_row)):
        raise PipelineError(f"{name} has an invalid homogeneous last row")


def validate_precalibrated_camchain(
    path: Path, expected_resolution: tuple[int, int] | None = None
) -> dict[str, Any]:
    """Validate the fixed camera parameters consumed by Kalibr IMU-camera."""
    chain = load_yaml_mapping(path)
    expected_keys = {"cam0", "cam1"}
    if set(chain) != expected_keys:
        found = ", ".join(sorted(str(key) for key in chain)) or "none"
        raise PipelineError(
            f"camera chain must contain exactly cam0 and cam1; found: {found}"
        )

    intrinsics_lengths = {"pinhole": 4, "omni": 5, "eucm": 6, "ds": 6}
    distortion_lengths = {"radtan": 4, "equidistant": 4, "fov": 1, "none": 0}
    for index in range(2):
        camera_name = f"cam{index}"
        camera = chain[camera_name]
        if not isinstance(camera, dict):
            raise PipelineError(f"{camera_name} must contain a YAML mapping")

        topic = camera.get("rostopic")
        expected_topic = f"/{camera_name}/image_raw"
        if topic != expected_topic:
            raise PipelineError(
                f"{camera_name}.rostopic must be {expected_topic!r}, got {topic!r}"
            )

        camera_model = camera.get("camera_model")
        if camera_model not in intrinsics_lengths:
            raise PipelineError(
                f"{camera_name}.camera_model is unsupported: {camera_model!r}"
            )
        intrinsics = validate_numeric_sequence(
            camera.get("intrinsics"),
            name=f"{camera_name}.intrinsics",
            lengths=(intrinsics_lengths[camera_model],),
        )
        focal_start = 0 if camera_model == "pinhole" else (1 if camera_model == "omni" else 2)
        if float(intrinsics[focal_start]) <= 0.0 or float(intrinsics[focal_start + 1]) <= 0.0:
            raise PipelineError(f"{camera_name}.intrinsics has a non-positive focal length")

        distortion_model = camera.get("distortion_model")
        if distortion_model not in distortion_lengths:
            raise PipelineError(
                f"{camera_name}.distortion_model is unsupported: {distortion_model!r}"
            )
        validate_numeric_sequence(
            camera.get("distortion_coeffs"),
            name=f"{camera_name}.distortion_coeffs",
            lengths=(distortion_lengths[distortion_model],),
        )

        resolution = camera.get("resolution")
        if (
            not isinstance(resolution, list)
            or len(resolution) != 2
            or not all(isinstance(item, int) and not isinstance(item, bool) and item > 0 for item in resolution)
        ):
            raise PipelineError(f"{camera_name}.resolution must contain two positive integers")
        if expected_resolution is not None and tuple(resolution) != expected_resolution:
            raise PipelineError(
                f"{camera_name}.resolution is {resolution[0]}x{resolution[1]}, but this run writes "
                f"{expected_resolution[0]}x{expected_resolution[1]} images; use matching --scale or camchain"
            )

    validate_transform(chain["cam1"].get("T_cn_cnm1"), name="cam1.T_cn_cnm1")
    return chain


def converted_image_resolution(summary_path: Path) -> tuple[int, int]:
    try:
        with summary_path.open(encoding="utf-8") as stream:
            summary = json.load(stream)
        width = summary["image_width"]
        height = summary["image_height"]
    except (OSError, json.JSONDecodeError, KeyError) as error:
        raise PipelineError(
            f"cannot read converted image resolution from {summary_path}: {error}"
        ) from error
    if (
        not isinstance(width, int)
        or isinstance(width, bool)
        or width <= 0
        or not isinstance(height, int)
        or isinstance(height, bool)
        or height <= 0
    ):
        raise PipelineError(f"invalid image resolution in {summary_path}: {width}x{height}")
    return width, height


def write_json_atomic(path: Path, value: dict) -> None:
    temporary = path.with_name(path.name + ".partial")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temporary, path)


def run_logged(
    command: list[str],
    log_path: Path,
    *,
    mirror: bool,
    dry_run: bool,
    cwd: Path | None = None,
) -> float:
    print(f"$ {format_command(command)}")
    if dry_run:
        return 0.0

    started = time.monotonic()
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(
            command,
            cwd=str(cwd) if cwd is not None else None,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
            bufsize=1,
        )
        assert process.stdout is not None
        try:
            for line in process.stdout:
                log.write(line)
                log.flush()
                if mirror:
                    sys.stdout.write(line)
                    sys.stdout.flush()
            return_code = process.wait()
        except KeyboardInterrupt:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=10)
            raise PipelineError("interrupted by user") from None

    elapsed = time.monotonic() - started
    if return_code != 0:
        raise PipelineError(
            f"command failed with exit code {return_code}; see {log_path}"
        )
    return elapsed


def ensure_dump_tool(path: Path, dry_run: bool) -> None:
    if path.is_file() and os.access(path, os.X_OK):
        return
    makefile = path.parent / "Makefile"
    if not makefile.is_file():
        raise PipelineError(f"video_user_data_dump not found or executable: {path}")
    command = ["make", "-C", str(path.parent), path.name]
    print(f"$ {format_command(command)}")
    if not dry_run:
        subprocess.run(command, check=True)
    if not dry_run and not (path.is_file() and os.access(path, os.X_OK)):
        raise PipelineError(f"failed to build video_user_data_dump: {path}")


def ensure_docker(args: argparse.Namespace, dry_run: bool) -> None:
    if shutil.which(args.docker) is None:
        raise PipelineError(f"Docker executable not found: {args.docker}")
    if dry_run:
        return
    result = subprocess.run(
        [args.docker, "image", "inspect", args.docker_image],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        errors="replace",
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or "image inspection failed"
        raise PipelineError(
            f"Docker image is unavailable: {args.docker_image}: {detail}"
        )


def parse_dump_summary(log_path: Path) -> dict:
    layouts = set()
    summary = None
    with log_path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = DUMP_SUMMARY_RE.search(line)
            if match:
                records = int(match.group(1))
                invalid = int(match.group(2))
                if records < 2:
                    raise PipelineError(f"only {records} SEI record(s) found in {log_path}")
                if invalid != 0:
                    raise PipelineError(
                        f"video_user_data_dump reported {invalid} invalid record(s)"
                    )
                summary = {"records_found": records, "invalid_records": invalid}
                layout_match = DUMP_LAYOUT_RE.search(line)
                if layout_match:
                    layouts.add(layout_match.group(1))
                continue
            layout_match = DUMP_LAYOUT_RE.search(line)
            if layout_match:
                layouts.add(layout_match.group(1))
            elif DUMP_START_LINE_RE.search(line) and DUMP_TIMING_REF_RE.search(line):
                layouts.add("V3")
            elif DUMP_LEGACY_LAYOUT_RE.search(line):
                layouts.add("V2")
            elif DUMP_V1_LAYOUT_RE.search(line):
                layouts.add("V1")
    if summary is None:
        raise PipelineError(f"missing video_user_data_dump summary in {log_path}")
    if layouts:
        summary["yctc_layouts"] = sorted(layouts)
        summary["yctc_versions"] = [int(layout[1:]) for layout in sorted(layouts)]
    return summary


def docker_command(
    args: argparse.Namespace,
    output_dir: Path,
    inner_command: list[str],
    selected_video: Path | None = None,
) -> list[str]:
    command = [
        args.docker,
        "run",
        "--rm",
        "--init",
        "--user",
        f"{os.getuid()}:{os.getgid()}",
        "--env",
        "HOME=/tmp/kalibr-home",
        "--env",
        "MPLCONFIGDIR=/tmp/kalibr-matplotlib",
        "--env",
        "ROS_HOME=/tmp/kalibr-ros",
        "--env",
        "PYTHONUNBUFFERED=1",
        "--volume",
        f"{output_dir}:/data",
        "--volume",
        f"{SCRIPT_DIR}:/work:ro",
        "--volume",
        f"{KALIBR_SOURCE_ROOT}:{KALIBR_SOURCE_MOUNT}:ro",
    ]

    if selected_video is not None and not path_is_within(selected_video, output_dir):
        command.extend(["--volume", f"{selected_video}:/input/source.h264:ro"])

    shell_command = (
        "set -eo pipefail; "
        f"source {shlex.quote(KALIBR_SETUP)}; "
        "cd /data; "
        f"exec {format_command(inner_command)}"
    )
    command.extend(
        ["--entrypoint", "/bin/bash", args.docker_image, "-lc", shell_command]
    )
    return command


def container_video_path(selected_video: Path, output_dir: Path) -> str:
    if path_is_within(selected_video, output_dir):
        relative = selected_video.relative_to(output_dir)
        return "/data/" + relative.as_posix()
    return "/input/source.h264"


def required_stage_files(
    output_dir: Path, prefix: str, *, reuse_camera_calibration: bool
) -> dict[str, list[Path]]:
    camera_files = [output_dir / f"{prefix}-camchain.yaml"]
    if not reuse_camera_calibration:
        camera_files.extend(
            [
                output_dir / f"{prefix}-results-cam.txt",
                output_dir / f"{prefix}-report-cam.pdf",
            ]
        )
    return {
        "dump": [
            output_dir / "video_user_data_dump.log",
            output_dir / "imu_raw.csv",
        ],
        "bag": [
            output_dir / f"{prefix}.bag",
            output_dir / "conversion_summary.json",
            output_dir / "selected_frame_timestamps.csv",
            output_dir / "frame_quality.csv",
            output_dir / "target.yaml",
            output_dir / "imu.yaml",
        ],
        "camera": camera_files,
        "imu": [
            output_dir / f"{prefix}-camchain-imucam.yaml",
            output_dir / f"{prefix}-results-imucam.txt",
            output_dir / f"{prefix}-report-imucam.pdf",
        ],
    }


def stage_is_complete(
    manifest: dict, stage: str, required_files: list[Path], resume: bool
) -> bool:
    if not resume:
        return False
    stage_info = manifest.get("stages", {}).get(stage, {})
    return stage_info.get("status") == "completed" and all(
        nonempty_file(path) for path in required_files
    )


def mark_stage(
    manifest: dict,
    manifest_path: Path,
    stage: str,
    status: str,
    *,
    dry_run: bool,
    **details,
) -> None:
    manifest.setdefault("stages", {})[stage] = {
        "status": status,
        "updated_at": now_iso(),
        **details,
    }
    manifest["updated_at"] = now_iso()
    if not dry_run:
        write_json_atomic(manifest_path, manifest)


def stop_after(args: argparse.Namespace, stage: str) -> bool:
    return STAGES.index(stage) >= STAGES.index(args.stop_after)


def configuration(
    args: argparse.Namespace, prefix: str, precalibrated_camchain: Path | None
) -> dict:
    config = {
        "prefix": prefix,
        "trim_pre_idr": not args.no_trim,
        "image_stride": args.image_stride,
        "image_scale": args.scale,
        "mono": not args.color,
        "bag_compression": args.compression,
        "min_laplacian_variance": args.min_laplacian_variance,
        "dark_pixel_max": args.dark_pixel_max,
        "max_dark_ratio": args.max_dark_ratio,
        "bright_pixel_min": args.bright_pixel_min,
        "max_bright_ratio": args.max_bright_ratio,
        "tag_cols": args.tag_cols,
        "tag_rows": args.tag_rows,
        "tag_size_m": args.tag_size,
        "tag_spacing_m": args.tag_spacing,
        "tag_spacing_ratio": args.tag_spacing / args.tag_size,
        "camera_model": args.camera_model,
        "camera_bag_frequency_hz": args.bag_freq,
        "approx_sync_s": args.approx_sync,
        "max_corner_reprojection_px": args.max_corner_reproj_px,
        "max_corner_filter_rounds": args.max_corner_filter_rounds,
        "camera_no_shuffle": args.no_shuffle,
        "imu_time_calibration": not args.no_time_calibration,
        "timeoffset_padding_s": args.timeoffset_padding,
        "time_offset_init_s": args.time_offset_init,
        "max_iterations": args.max_iter,
        "accelerometer_noise_density": args.acc_noise_density,
        "accelerometer_random_walk": args.acc_random_walk,
        "gyroscope_noise_density": args.gyro_noise_density,
        "gyroscope_random_walk": args.gyro_random_walk,
        "timestamp_policy": {
            "frame_association": "decoded image N uses SEI record N",
            "image": "per-camera exposure_start_pts_us + exposure_time_us / 2",
            "imu": "raw pts_us; no UI filter group-delay compensation",
        },
        "docker_image": args.docker_image,
    }
    if precalibrated_camchain is not None:
        config["precalibrated_camchain"] = input_identity(precalibrated_camchain)
    return config


def load_or_create_manifest(
    args: argparse.Namespace,
    manifest_path: Path,
    source_identity: dict,
    config: dict,
) -> dict:
    if args.resume and manifest_path.is_file():
        with manifest_path.open(encoding="utf-8") as stream:
            manifest = json.load(stream)
        if manifest.get("input") != source_identity:
            raise PipelineError(
                "--resume input does not match calibration_run.json; use a new output "
                "directory or --force"
            )
        if manifest.get("configuration") != config:
            raise PipelineError(
                "--resume parameters do not match calibration_run.json; use --force "
                "to rerun with changed parameters"
            )
        return manifest

    if args.resume and manifest_path.parent.exists() and any(manifest_path.parent.iterdir()):
        raise PipelineError(
            f"cannot --resume without a matching manifest: {manifest_path}"
        )

    return {
        "schema_version": 1,
        "created_at": now_iso(),
        "updated_at": now_iso(),
        "input": source_identity,
        "configuration": config,
        "stages": {},
    }


def prepare_video(
    args: argparse.Namespace,
    source: Path,
    output_dir: Path,
    manifest: dict,
    manifest_path: Path,
) -> Path:
    print("[prepare] Inspecting H.264 Annex-B prefix")
    inspection = inspect_h264(source)
    manifest["h264_inspection"] = asdict(inspection)

    if args.no_trim or inspection.trim_offset is None:
        selected = source
        action = "source already starts before its first IDR"
        if args.no_trim and inspection.trim_offset is not None:
            action = "pre-IDR trimming disabled by --no-trim"
    else:
        selected = output_dir / f"{source.stem}.from_first_idr.h264"
        expected_size = inspection.file_size - inspection.trim_offset
        if args.resume and selected.is_file() and selected.stat().st_size == expected_size:
            action = "reused trimmed H.264"
        elif args.dry_run:
            action = f"would copy source bytes from offset {inspection.trim_offset}"
        else:
            print(
                f"[prepare] Removing {inspection.pre_idr_vcl_count} pre-IDR VCL NAL unit(s); "
                f"copy starts at byte {inspection.trim_offset}"
            )
            copy_suffix(source, selected, inspection.trim_offset)
            if selected.stat().st_size != expected_size:
                raise PipelineError(f"trimmed H.264 size check failed: {selected}")
            action = "created trimmed H.264"

    manifest["selected_video"] = str(selected)
    manifest["prepare"] = {
        "status": "completed",
        "action": action,
        "selected_video": str(selected),
        "updated_at": now_iso(),
    }
    if not args.dry_run:
        write_json_atomic(manifest_path, manifest)
    print(f"[prepare] {action}: {selected}")
    return selected


def parse_timeshifts(path: Path) -> list[float]:
    values = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            match = TIMESHIFT_RE.match(line)
            if match:
                values.append(float(match.group(1)))
    return values


def final_summary(
    output_dir: Path,
    prefix: str,
    precalibrated_camchain: Path | None,
) -> dict:
    result_yaml = output_dir / f"{prefix}-camchain-imucam.yaml"
    shifts = parse_timeshifts(result_yaml)
    summary = {
        "completed_at": now_iso(),
        "result_yaml": str(result_yaml),
        "imu_camera_report_pdf": str(output_dir / f"{prefix}-report-imucam.pdf"),
        "timeshift_convention": "t_imu = t_cam + timeshift_cam_imu",
        "timeshift_cam_imu_s": shifts,
    }
    if precalibrated_camchain is None:
        summary["camera_calibration"] = "estimated in this run"
        summary["camera_report_pdf"] = str(output_dir / f"{prefix}-report-cam.pdf")
    else:
        summary["camera_calibration"] = "fixed from a pre-calibrated camera chain"
        summary["precalibrated_camchain"] = input_identity(precalibrated_camchain)
    if shifts:
        summary["timeshift_mean_s"] = sum(shifts) / len(shifts)
    if len(shifts) == 2:
        summary["timeshift_camera_difference_s"] = abs(shifts[0] - shifts[1])
    return summary


def run_pipeline(args: argparse.Namespace) -> int:
    source = Path(args.video).expanduser().resolve()
    if not source.is_file():
        raise PipelineError(f"input H.264 does not exist: {source}")

    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else source.parent / f"kalibr_{source.stem}"
    )
    prefix = args.prefix or f"calibration_stride{args.image_stride}"
    manifest_path = output_dir / "calibration_run.json"
    precalibrated_camchain = None
    if args.camchain:
        precalibrated_camchain = Path(args.camchain).expanduser().resolve()
        if not precalibrated_camchain.is_file():
            raise PipelineError(
                f"pre-calibrated camera chain does not exist: {precalibrated_camchain}"
            )
        validate_precalibrated_camchain(precalibrated_camchain)

    if output_dir.exists() and not output_dir.is_dir():
        raise PipelineError(f"output path is not a directory: {output_dir}")
    if not args.dry_run:
        if output_dir.exists() and any(output_dir.iterdir()) and not (args.resume or args.force):
            raise PipelineError(
                f"output directory is not empty: {output_dir}; use --resume, --force, "
                "or a new directory"
            )
        output_dir.mkdir(parents=True, exist_ok=True)

    config = configuration(args, prefix, precalibrated_camchain)
    source_identity = input_identity(source)
    if args.dry_run:
        manifest = {
            "schema_version": 1,
            "input": source_identity,
            "configuration": config,
            "stages": {},
        }
    else:
        manifest = load_or_create_manifest(
            args, manifest_path, source_identity, config
        )
    selected_video = prepare_video(
        args, source, output_dir, manifest, manifest_path
    )
    required = required_stage_files(
        output_dir,
        prefix,
        reuse_camera_calibration=precalibrated_camchain is not None,
    )

    dump_log = output_dir / "video_user_data_dump.log"
    imu_csv = output_dir / "imu_raw.csv"
    mag_csv = output_dir / "mag_raw.csv"
    ensure_dump_tool(Path(args.dump_tool).expanduser().resolve(), args.dry_run)

    if stage_is_complete(manifest, "dump", required["dump"], args.resume):
        print("[dump] Reusing completed SEI/IMU extraction")
    else:
        print("[dump] Extracting YCTC SEI, IMU and MAG data")
        dump_command = [
            str(Path(args.dump_tool).expanduser().resolve()),
            "--format",
            "h264",
            "--input",
            str(selected_video),
            "--imu-output",
            str(imu_csv),
            "--mag-output",
            str(mag_csv),
        ]
        elapsed = run_logged(
            dump_command,
            dump_log,
            mirror=False,
            dry_run=args.dry_run,
        )
        details = {"elapsed_s": elapsed, "log": str(dump_log)}
        if not args.dry_run:
            if not nonempty_file(imu_csv):
                raise PipelineError(f"IMU CSV is empty: {imu_csv}")
            details.update(parse_dump_summary(dump_log))
        mark_stage(
            manifest,
            manifest_path,
            "dump",
            "completed",
            dry_run=args.dry_run,
            **details,
        )
    if stop_after(args, "dump"):
        print(f"Completed through dump stage: {output_dir}")
        return 0

    converter = Path(args.converter).expanduser().resolve()
    if not converter.is_file():
        raise PipelineError(f"bag converter does not exist: {converter}")
    if converter.parent != SCRIPT_DIR:
        raise PipelineError(
            f"converter must be inside {SCRIPT_DIR} so it is visible in Docker: {converter}"
        )
    ensure_docker(args, args.dry_run)

    bag_path = output_dir / f"{prefix}.bag"
    if stage_is_complete(manifest, "bag", required["bag"], args.resume):
        print("[bag] Reusing completed ROS bag conversion")
    else:
        print("[bag] Decoding stereo frames and writing the Kalibr bag")
        converter_command = [
            "python3",
            f"/work/{converter.name}",
            "--video",
            container_video_path(selected_video, output_dir),
            "--imu-csv",
            "/data/imu_raw.csv",
            "--dump-log",
            "/data/video_user_data_dump.log",
            "--output-dir",
            "/data",
            "--output-bag",
            bag_path.name,
            "--image-stride",
            str(args.image_stride),
            "--scale",
            str(args.scale),
            "--min-laplacian-variance",
            str(args.min_laplacian_variance),
            "--dark-pixel-max",
            str(args.dark_pixel_max),
            "--max-dark-ratio",
            str(args.max_dark_ratio),
            "--bright-pixel-min",
            str(args.bright_pixel_min),
            "--max-bright-ratio",
            str(args.max_bright_ratio),
            "--compression",
            args.compression,
            "--tag-cols",
            str(args.tag_cols),
            "--tag-rows",
            str(args.tag_rows),
            "--tag-size",
            str(args.tag_size),
            "--tag-spacing",
            str(args.tag_spacing / args.tag_size),
            "--acc-noise-density",
            str(args.acc_noise_density),
            "--acc-random-walk",
            str(args.acc_random_walk),
            "--gyro-noise-density",
            str(args.gyro_noise_density),
            "--gyro-random-walk",
            str(args.gyro_random_walk),
            "--color" if args.color else "--mono",
        ]
        command = docker_command(
            args, output_dir, converter_command, selected_video=selected_video
        )
        elapsed = run_logged(
            command,
            output_dir / "conversion.log",
            mirror=not args.quiet,
            dry_run=args.dry_run,
        )
        if not args.dry_run:
            missing = [path for path in required["bag"] if not nonempty_file(path)]
            if missing:
                raise PipelineError(f"bag stage did not create: {missing[0]}")
        mark_stage(
            manifest,
            manifest_path,
            "bag",
            "completed",
            dry_run=args.dry_run,
            elapsed_s=elapsed,
            bag=str(bag_path),
            log=str(output_dir / "conversion.log"),
        )
    if stop_after(args, "bag"):
        print(f"Completed through bag stage: {output_dir}")
        return 0

    camchain_path = output_dir / f"{prefix}-camchain.yaml"
    if precalibrated_camchain is not None:
        expected_resolution = None
        if not args.dry_run:
            expected_resolution = converted_image_resolution(
                output_dir / "conversion_summary.json"
            )
        validate_precalibrated_camchain(
            precalibrated_camchain, expected_resolution=expected_resolution
        )

        camera_stage_complete = stage_is_complete(
            manifest, "camera", required["camera"], args.resume
        )
        if camera_stage_complete and not filecmp.cmp(
            precalibrated_camchain, camchain_path, shallow=False
        ):
            camera_stage_complete = False

        if camera_stage_complete:
            print("[camera] Reusing copied pre-calibrated camera chain")
        else:
            print(
                "[camera] Skipping stereo camera calibration; using pre-calibrated "
                f"chain: {precalibrated_camchain}"
            )
            if not args.dry_run:
                copy_file_atomic(precalibrated_camchain, camchain_path)
            mark_stage(
                manifest,
                manifest_path,
                "camera",
                "completed",
                dry_run=args.dry_run,
                action="copied pre-calibrated camera chain; intrinsics and camera-camera extrinsics fixed",
                source=input_identity(precalibrated_camchain),
                camchain=str(camchain_path),
            )
    elif stage_is_complete(manifest, "camera", required["camera"], args.resume):
        print("[camera] Reusing completed stereo camera calibration")
    else:
        print("[camera] Running stereo camera calibration")
        camera_command = [
            "python3",
            PATCHED_KALIBR_CAMERA_CALIBRATOR,
            "--bag",
            f"/data/{bag_path.name}",
            "--topics",
            "/cam0/image_raw",
            "/cam1/image_raw",
            "--models",
            args.camera_model,
            args.camera_model,
            "--target",
            "/data/target.yaml",
            "--bag-freq",
            str(args.bag_freq),
            "--approx-sync",
            str(args.approx_sync),
            "--max-corner-reproj-px",
            str(args.max_corner_reproj_px),
            "--max-corner-filter-rounds",
            str(args.max_corner_filter_rounds),
            "--dont-show-report",
        ]
        if args.no_shuffle:
            camera_command.append("--no-shuffle")
        command = docker_command(args, output_dir, camera_command)
        elapsed = run_logged(
            command,
            output_dir / "camera_calib.log",
            mirror=not args.quiet,
            dry_run=args.dry_run,
        )
        if not args.dry_run:
            missing = [path for path in required["camera"] if not nonempty_file(path)]
            if missing:
                raise PipelineError(f"camera stage did not create: {missing[0]}")
        mark_stage(
            manifest,
            manifest_path,
            "camera",
            "completed",
            dry_run=args.dry_run,
            elapsed_s=elapsed,
            camchain=str(camchain_path),
            log=str(output_dir / "camera_calib.log"),
        )
    if stop_after(args, "camera"):
        print(f"Completed through camera stage: {output_dir}")
        return 0

    final_yaml = output_dir / f"{prefix}-camchain-imucam.yaml"
    if stage_is_complete(manifest, "imu", required["imu"], args.resume):
        print("[imu] Reusing completed camera/IMU calibration")
    else:
        print("[imu] Running camera/IMU spatial and temporal calibration")
        imu_command = [
            "rosrun",
            "kalibr",
            "kalibr_calibrate_imu_camera",
            "--bag",
            f"/data/{bag_path.name}",
            "--cams",
            f"/data/{camchain_path.name}",
            "--imu",
            "/data/imu.yaml",
            "--target",
            "/data/target.yaml",
            "--bag-freq",
            str(args.bag_freq),
            "--timeoffset-padding",
            str(args.timeoffset_padding),
            "--time-offset-init",
            str(args.time_offset_init),
            "--max-iter",
            str(args.max_iter),
            "--dont-show-report",
        ]
        if args.no_time_calibration:
            imu_command.append("--no-time-calibration")
        command = docker_command(args, output_dir, imu_command)
        elapsed = run_logged(
            command,
            output_dir / "imu_cam.log",
            mirror=not args.quiet,
            dry_run=args.dry_run,
        )
        if not args.dry_run:
            missing = [path for path in required["imu"] if not nonempty_file(path)]
            if missing:
                raise PipelineError(f"IMU stage did not create: {missing[0]}")
        mark_stage(
            manifest,
            manifest_path,
            "imu",
            "completed",
            dry_run=args.dry_run,
            elapsed_s=elapsed,
            result_yaml=str(final_yaml),
            log=str(output_dir / "imu_cam.log"),
        )

    if args.dry_run:
        print(f"Dry run complete; planned output directory: {output_dir}")
        return 0

    summary = final_summary(output_dir, prefix, precalibrated_camchain)
    write_json_atomic(output_dir / "calibration_summary.json", summary)
    manifest["result_summary"] = summary
    manifest["status"] = "completed"
    manifest["updated_at"] = now_iso()
    write_json_atomic(manifest_path, manifest)

    print("Calibration completed")
    print(f"  Result YAML: {final_yaml}")
    shifts = summary.get("timeshift_cam_imu_s", [])
    for index, shift in enumerate(shifts):
        print(f"  cam{index} timeshift: {shift:+.10f} s")
    if "timeshift_mean_s" in summary:
        print(f"  mean timeshift: {summary['timeshift_mean_s']:+.10f} s")
    return 0


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be >= 1")
    return parsed


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be > 0")
    return parsed


def nonnegative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError("must be >= 0")
    return parsed


def finite_float(value: str) -> float:
    parsed = float(value)
    if math.isnan(parsed) or math.isinf(parsed):
        raise argparse.ArgumentTypeError("must be finite")
    return parsed


def unit_interval(value: str) -> float:
    parsed = float(value)
    if not 0.0 <= parsed <= 1.0:
        raise argparse.ArgumentTypeError("must be in [0, 1]")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run YCTC H.264 stereo camera/IMU calibration with Kalibr.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog=(
            "Example:\n"
            "  python3 calibrate_h264_imu.py /mnt/d/Temp/0727/1.h264 "
            "--output-dir /mnt/d/Temp/0727/kalibr_test_01\n\n"
            "Reuse a calibrated stereo camera chain:\n"
            "  python3 calibrate_h264_imu.py /mnt/d/Temp/0727/1.h264 "
            "--output-dir /mnt/d/Temp/0727/imu_only_01 "
            "--camchain /mnt/d/Temp/camera-calibration-camchain.yaml"
        ),
    )
    parser.add_argument("video", help="YCTC Annex-B H.264 recording")
    parser.add_argument("--output-dir", "--output", help="result directory")
    parser.add_argument("--prefix", help="bag and Kalibr result filename prefix")

    grid = parser.add_argument_group("AprilGrid")
    grid.add_argument("--tag-cols", type=positive_int, default=6)
    grid.add_argument("--tag-rows", type=positive_int, default=6)
    grid.add_argument("--tag-size", type=positive_float, default=0.0352, help="tag size in meters")
    grid.add_argument(
        "--tag-spacing",
        type=nonnegative_float,
        default=0.01056,
        help="physical gap between tags in meters",
    )

    images = parser.add_argument_group("video and bag")
    images.add_argument("--no-trim", action="store_true", help="do not remove pre-IDR VCL NAL units")
    images.add_argument("--image-stride", type=positive_int, default=3)
    images.add_argument("--scale", type=positive_float, default=0.5)
    images.add_argument("--color", action="store_true", help="write bgr8 images instead of mono8")
    images.add_argument("--compression", choices=("none", "bz2", "lz4"), default="lz4")
    images.add_argument(
        "--min-laplacian-variance",
        "--blur-threshold",
        type=nonnegative_float,
        default=100.0,
        help="minimum output-image variance of Laplacian; 0 disables blur rejection",
    )
    images.add_argument("--dark-pixel-max", type=int, default=5)
    images.add_argument("--max-dark-ratio", type=unit_interval, default=0.85)
    images.add_argument("--bright-pixel-min", type=int, default=250)
    images.add_argument("--max-bright-ratio", type=unit_interval, default=0.85)

    camera = parser.add_argument_group("Kalibr camera calibration")
    camera.add_argument(
        "--camchain",
        "--camera-chain",
        help=(
            "pre-calibrated two-camera camchain YAML; skip camera calibration and "
            "keep its intrinsics and camera-camera extrinsics fixed"
        ),
    )
    camera.add_argument("--camera-model", default="pinhole-radtan")
    camera.add_argument("--bag-freq", type=positive_float, default=4.0)
    camera.add_argument("--approx-sync", type=positive_float, default=0.02)
    camera.add_argument(
        "--max-corner-reproj-px",
        type=positive_float,
        default=0.85,
        help="fixed final-stage corner reprojection rejection threshold in pixels",
    )
    camera.add_argument(
        "--max-corner-filter-rounds",
        type=positive_int,
        default=10,
        help="maximum fixed-threshold convergence rounds",
    )
    camera.add_argument("--no-shuffle", action="store_true")

    imu = parser.add_argument_group("Kalibr IMU calibration")
    imu.add_argument("--no-time-calibration", action="store_true")
    imu.add_argument("--timeoffset-padding", type=positive_float, default=0.03)
    imu.add_argument(
        "--time-offset-init",
        "--timeoffset-init",
        "--time_offset_init",
        type=finite_float,
        default=0.001,
        help="initial camera-to-IMU time offset increment in seconds",
    )
    imu.add_argument("--max-iter", type=positive_int, default=30)
    imu.add_argument("--acc-noise-density", type=positive_float, default=0.02)
    imu.add_argument("--acc-random-walk", type=positive_float, default=1.92084189233e-05)
    imu.add_argument("--gyro-noise-density", type=positive_float, default=0.002)
    imu.add_argument("--gyro-random-walk", type=positive_float, default=2.26588533832e-06)

    tools = parser.add_argument_group("tools")
    tools.add_argument("--dump-tool", default=str(DEFAULT_DUMP_TOOL))
    tools.add_argument("--converter", default=str(SCRIPT_DIR / "h264_sei_to_kalibr_bag.py"))
    tools.add_argument("--docker", default="docker")
    tools.add_argument("--docker-image", default=DEFAULT_DOCKER_IMAGE)

    execution = parser.add_argument_group("execution")
    mode = execution.add_mutually_exclusive_group()
    mode.add_argument("--resume", action="store_true", help="reuse completed stages from the manifest")
    mode.add_argument("--force", action="store_true", help="rerun stages and overwrite known outputs")
    execution.add_argument("--stop-after", choices=STAGES, default="imu")
    execution.add_argument("--dry-run", action="store_true")
    execution.add_argument("--quiet", action="store_true", help="write Docker output only to log files")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if not 0 <= args.dark_pixel_max < args.bright_pixel_min <= 255:
        parser.error(
            "pixel thresholds must satisfy "
            "0 <= --dark-pixel-max < --bright-pixel-min <= 255"
        )
    try:
        return run_pipeline(args)
    except (PipelineError, OSError, subprocess.SubprocessError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
