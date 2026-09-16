# Kalibr H.264 + IMU Reproduction Demo

This package reproduces stereo camera and IMU calibration from the supplied
YCTC H.264 sample. It intentionally does not include a Docker image tarball.
On the first run, `run_demo.sh` builds the pinned Kalibr source snapshot with
the included Dockerfile; later runs reuse the local image.

## Quick start

Use Ubuntu x86_64 or WSL2 with Docker Desktop integration. From this directory:

```bash
bash run_demo.sh
```

The first run downloads the ROS base image and package dependencies, then
builds Kalibr. The reference calibration itself took about 10 minutes after
the image had been built. Reserve at least 20 GB of free disk space and 16 GB
of RAM. `KALIBR_BUILD_JOBS=2` is the conservative default; increase it only
when the host has sufficient memory.

The generated files are written to `output/`:

- `calibration_stride3-camchain-imucam.yaml`: final camera-to-IMU result
- `calibration_stride3-report-imucam.pdf`: camera-IMU report
- `calibration_stride3-camchain.yaml`: stereo camera calibration
- `calibration_stride3-report-cam.pdf`: stereo camera report
- `calibration_summary.json`: compact result summary

To rerun an interrupted job, run the same command again. The wrapper supplies
`--resume` when it finds a prior `calibration_run.json`. To rebuild the Docker
image explicitly, use:

```bash
bash run_demo.sh --rebuild-image
```

## Included content

- `sample/a01.h264`: validated stereo H.264 recording with YCTC SEI metadata
- `bin/video_user_data_dump`: Linux x86_64 SEI and IMU extraction tool
- `scripts/calibrate_h264_imu.py`: workflow driver
- `scripts/h264_sei_to_kalibr_bag.py`: H.264/SEI to ROS bag converter
- `kalibr_source/`: Kalibr source snapshot used to build the image
- `docker/Dockerfile`: reproducible ROS Noetic/Kalibr build recipe
- `expected_output/`: YAML and PDF results from this exact sample

The Docker build context excludes the sample and expected reports, so image
construction transfers only the Kalibr source snapshot.

## Reference validation

Run an integrity check before use:

```bash
sha256sum -c SHA256SUMS
```

For `sample/a01.h264`, the extraction stage must report:

- `records_found=2363`
- `invalid_records=0`
- `77772` IMU samples in `imu_raw.csv`

The reference result is in `expected_output/`. Its key values are:

- camera-to-IMU mean time shift: `+0.0004129142 s`
- cam0 time shift: `+0.0004315803 s`
- cam1 time shift: `+0.0003942482 s`
- stereo baseline: `0.05995446 m`

The optimizer may not produce byte-identical YAML on different CPUs or library
builds. Treat a result near these values, with valid PDF reports and no failed
pipeline stage, as a successful reproduction.

## Custom input

The sample is not generic H.264. It contains side-by-side stereo images and
YCTC SEI records with exposure timestamps and IMU samples. For another
recording, set an absolute path and a new output directory:

```bash
KALIBR_VIDEO=/absolute/path/recording.h264 \
KALIBR_OUTPUT_DIR=/absolute/path/kalibr_output \
bash run_demo.sh
```

The converter understands YCTC V1, V2, and V3 user-data layouts. V3 records
use `left/right_exposure_start_pts_us` and `left/right_exposure_time_us` to
timestamp each camera at its exposure center; the start-line and timing
reference fields are retained as diagnostic metadata. The dump log and IMU
CSV are cross-checked for matching per-record metadata before a bag is written.

The IMU calibration initializes the optimizable camera-to-IMU time-offset
increment with `--time-offset-init 0.001` (1 ms) by default. Override it in a
direct invocation of `scripts/calibrate_h264_imu.py` when a different initial
value is appropriate; the value is in seconds and may be positive or negative.

The AprilGrid dimensions and IMU noise values in `run_demo.sh` are specific to
this demo. Change them to match the physical target and sensor specification
before calibrating another device. Pass workflow options such as
`--stop-after bag` or `--dry-run` after `run_demo.sh`.

When `--camchain` is used directly with `scripts/calibrate_h264_imu.py`, the
host also needs PyYAML:

```bash
python3 -m pip install --user PyYAML
```

## Prerequisites and restrictions

- Docker must be usable by the invoking user.
- Internet access is needed only for the initial Docker build.
- `video_user_data_dump` is dynamically linked for glibc Linux x86_64; use
  Ubuntu or WSL2, not native Windows or ARM Linux.
- The input should be captured with the supported YCTC SEI protocol. A generic
  H.264 recording has no IMU data and will fail the extraction stage.
- Do not use the demo parameters as production values without independently
  verifying the AprilGrid dimensions, sensor orientation, and IMU noise model.

## Licensing

Kalibr is redistributed under its BSD license, preserved at
`kalibr_source/LICENSE`. The project-specific `video_user_data_dump` binary is
not part of Kalibr; confirm that its vendor/SDK license permits redistribution
to the intended customer before delivery. See `third_party_notices/README.md`.
