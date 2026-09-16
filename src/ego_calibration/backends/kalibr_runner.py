#!/usr/bin/env python3
"""Run Kalibr with planar pinhole initialization for incomplete target views."""
import json
from pathlib import Path
import runpy

import cv2
import numpy as np


def planar_pinhole_seed(object_points, image_points, resolution):
    """Estimate the same four distortion coefficients used by pinhole-radtan."""
    if len(object_points) != len(image_points) or len(object_points) < 6:
        raise ValueError('部分角点初始化至少需要 6 帧有效观测，请补录不同倾角的清晰画面')
    for objects, pixels in zip(object_points, image_points):
        if (objects.ndim != 2 or objects.shape[1] != 3
                or pixels.shape != (len(objects), 2) or len(objects) < 6
                or not np.isfinite(objects).all() or not np.isfinite(pixels).all()
                or not np.allclose(objects[:, 2], 0)
                or np.linalg.matrix_rank(objects[:, :2] - objects[:, :2].mean(axis=0)) < 2
                or np.linalg.matrix_rank(pixels - pixels.mean(axis=0)) < 2):
            raise ValueError('部分角点初始化需要有效、非共线的平面标定板观测')
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(
        [np.asarray(p, dtype=np.float32) for p in object_points],
        [np.asarray(p, dtype=np.float32) for p in image_points],
        tuple(resolution), None, None, flags=cv2.CALIB_FIX_K3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-8))
    intrinsics = matrix[[0, 1, 0, 1], [0, 1, 2, 2]]
    coefficients = distortion.flatten()[:4]
    if (not np.isfinite(np.r_[rms, intrinsics, coefficients]).all()
            or np.any(intrinsics[:2] <= 0)
            or not 0 <= intrinsics[2] < resolution[0]
            or not 0 <= intrinsics[3] < resolution[1]):
        raise ValueError('部分角点初始化未得到合理内参，请增加不同倾角并检查标定板尺寸')
    return intrinsics, coefficients, float(rms)


def install_partial_view_initializer():
    import aslam_cv as acv
    import aslam_cv_backend as acvb
    import kalibr_camera_calibration as kcc

    original = kcc.CameraGeometry.initGeometryFromObservations

    def initialize(camera, observations):
        counts = [len(obs.getCornersImageFrame()) for obs in observations]
        complete = sum(n == obs.target().size() for n, obs in zip(counts, observations))
        if camera.model == acvb.DistortedPinhole and observations and not complete:
            selected = [obs for n, obs in zip(counts, observations) if n >= 6]
            resolution = (observations[0].imCols(), observations[0].imRows())
            if any((o.imCols(), o.imRows()) != resolution for o in observations):
                raise ValueError('角点观测分辨率不一致，不能共用相机内参')
            print('完整角点帧为 0；使用 %d 帧部分角点估计针孔初值，再由 Kalibr 优化。'
                  % len(selected), flush=True)
            intrinsics, coefficients, rms = planar_pinhole_seed(
                [o.getCornersTargetFrame() for o in selected],
                [o.getCornersImageFrame() for o in selected], resolution)
            # Rebuild geometry AND its design variables so optimization and
            # projection use the same camera object and native image resolution.
            distortion = acv.RadialTangentialDistortion(*coefficients)
            projection = acv.DistortedPinholeProjection(*intrinsics, *resolution, distortion)
            camera.geometry = acv.DistortedPinholeCameraGeometry(projection)
            camera.dv = camera.model.designVariable(camera.geometry)
            camera.ctarget = kcc.TargetDetector(camera.ctarget.targetConfig, camera.geometry)
            report = dict(method='opencv_planar_partial_views', resolution=resolution,
                          views=len(selected), complete_views=complete, corners_per_view=counts,
                          intrinsics=intrinsics.tolist(), distortion_coeffs=coefficients.tolist(),
                          initial_fit_rms_px=rms, note='Initial guess only; Kalibr optimization follows.')
            report_path = Path(camera.dataset.bagfile).with_suffix('.initialization.json')
            report_path.write_text(json.dumps(report, indent=2) + '\n')
            print('初值 fx=%.2f、fy=%.2f px，初步拟合 RMS=%.3f px（不是最终结果）'
                  % (intrinsics[0], intrinsics[1], rms), flush=True)
            success = kcc.calibrateIntrinsics(camera, observations)
        else:
            success = original(camera, observations)
        parameters = np.r_[camera.geometry.projection().getParameters().flatten(),
                           camera.geometry.projection().distortion().getParameters().flatten()]
        camera.isGeometryInitialized = bool(success and np.isfinite(parameters).all())
        if not camera.isGeometryInitialized:
            raise ValueError('Kalibr 内参初始化未收敛为有效数值，请检查清晰度、倾角和标定板尺寸')
        return True

    kcc.CameraGeometry.initGeometryFromObservations = initialize


def install_streaming_extractor():
    import kalibr_common as kc

    original = kc.extractCornersFromDataset

    def extract(dataset, detector, multithreading=False, numProcesses=None,
                clearImages=True, noTransformation=False):
        # Kalibr's parallel path queues the entire bag before starting workers.
        # Its camera CLI also retains every image in the results. For headless
        # calibration only corners, timestamps and image dimensions are needed.
        print('逐帧提取角点：保留原始分辨率，避免整段图像缓存和多进程内存峰值。', flush=True)
        return original(dataset, detector, multithreading=False, numProcesses=1,
                        clearImages=True if multithreading else clearImages,
                        noTransformation=noTransformation)

    kc.extractCornersFromDataset = extract


def main():
    import rospkg
    install_streaming_extractor()
    install_partial_view_initializer()
    script = Path(rospkg.RosPack().get_path('kalibr')) / 'python' / 'kalibr_calibrate_cameras'
    runpy.run_path(str(script), run_name='__main__')


if __name__ == '__main__':
    main()
