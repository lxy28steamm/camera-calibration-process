"""Portable OpenCV mono calibration and fixed-parameter video verification."""
from __future__ import annotations
import csv
import html
import json
from dataclasses import asdict
from pathlib import Path
import cv2
import numpy as np
import yaml
from ego_calibration.inspection import InspectionSettings, _camera, error_stats, image_quality
from ego_calibration.models import CalibrationError
from ego_calibration.backends.calibration_flash import calibration


class BoardDetector:
    def __init__(self, settings, camera=None):
        self.settings, self.camera, self.order = settings, camera, None
        self.grid = InspectionSettings(rows=settings.rows, columns=settings.columns, tag_size_m=settings.size_mm/1000, tag_spacing_m=settings.gap_mm/1000)
        self.detectors = []
        for border in (1, 2):
            params = cv2.aruco.DetectorParameters()
            params.markerBorderBits = border
            # AprilTag quad fitting rejects malformed/connected contours in printed grids.
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
            self.detectors.append(cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), params))

    def detect(self, gray):
        s = self.settings
        if s.board == 'checkerboard':
            ok, pixels = cv2.findChessboardCorners(gray, (s.columns, s.rows))
            if not ok:
                return None
            pixels = cv2.cornerSubPix(gray, pixels, (5,5), (-1,-1), (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,.01)).reshape(-1,2)
            points = np.zeros((s.rows*s.columns,3),np.float32)
            points[:,:2] = np.mgrid[:s.columns,:s.rows].T.reshape(-1,2)*s.size_mm/1000
            return points, pixels
        observations = [d.detectMarkers(gray)[:2] for d in self.detectors]
        corners, ids = max(observations, key=lambda v: 0 if v[1] is None else len(v[1]))
        if ids is None:
            return None
        if len(set(ids.flatten())) != len(ids):
            raise CalibrationError('画面含重复标签 ID，请移开其他标定板')
        found = {int(tag):c.reshape(4,2) for tag,c in zip(ids.flatten(),corners) if 0<=tag<s.rows*s.columns}
        ids = sorted(found)
        if len(ids)<6 or len({i//s.columns for i in ids})<2 or len({i%s.columns for i in ids})<2:
            return None
        pixels = np.concatenate([found[i] for i in ids]).astype(np.float64)
        if self.order is None:
            normalized = self.camera.normalized(pixels) if self.camera else pixels
            choices = [tuple(np.roll(v,n)) for v in ((0,1,2,3),(0,3,2,1)) for n in range(4)]
            def score(order):
                xy = self.grid.points(ids,order)[:,:2].reshape(-1,1,2)
                h,_ = cv2.findHomography(xy, normalized, 0)
                return float('inf') if h is None else np.mean(np.linalg.norm(cv2.perspectiveTransform(xy,h).reshape(-1,2)-normalized,axis=1))
            self.order = min(choices,key=score)
        return self.grid.points(ids,self.order), pixels


def camera_from_yaml(text):
    data = calibration(text.encode())
    if data['camera_model'] != 'pinhole' or data['distortion_model'] not in ('radtan','equidistant','none'):
        raise CalibrationError('内置复测支持 pinhole-radtan / equidistant / none；omni、ds 请使用对应模型的独立验证程序')
    fx,fy,cx,cy = data['intrinsics']
    return _camera([[fx,0,cx],[0,fy,cy],[0,0,1]],data['distortion_coeffs'],data['resolution'],data['distortion_model'])


def fit_camera(objects, pixels, size, model):
    if model == 'pinhole-radtan':
        rms,k,d,_,_ = cv2.calibrateCamera([o.astype(np.float32) for o in objects],[p.astype(np.float32) for p in pixels],size,None,None,flags=cv2.CALIB_FIX_K3)
        distortion = d.flatten()[:4]
        name = 'radtan'
    elif model == 'pinhole-equi':
        k = cv2.initCameraMatrix2D([o.astype(np.float32) for o in objects],[p.astype(np.float32) for p in pixels],size)
        rms,k,d,_,_ = cv2.fisheye.calibrate([o.astype(np.float64).reshape(-1,1,3) for o in objects],[p.astype(np.float64).reshape(-1,1,2) for p in pixels],size,k,np.zeros((4,1)),flags=cv2.fisheye.CALIB_USE_INTRINSIC_GUESS|cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC|cv2.fisheye.CALIB_FIX_SKEW,criteria=(3,100,1e-7))
        distortion = d.flatten()
        name = 'equidistant'
    else:
        raise CalibrationError('内置 OpenCV 支持 pinhole-radtan / pinhole-equi；omni-radtan 和 ds-none 请选择 Kalibr')
    camera = _camera(k,distortion,list(size),name)
    if not np.isfinite(rms):
        raise CalibrationError('标定未收敛')
    return camera, float(rms)


def analyze_video(video: Path, settings, output: Path, stop, *, yaml_text=None, progress=lambda _:None):
    output.mkdir(parents=True,exist_ok=True)
    camera = camera_from_yaml(yaml_text) if yaml_text else None
    report_settings = asdict(settings)
    if camera:
        report_settings['model'] = 'pinhole-' + ('equi' if camera.model == 'equidistant' else camera.model)
    detector = BoardDetector(settings,camera)
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise CalibrationError('视频无法解码')
    samples, objects, pixels, signatures, coverage = [],[],[],[],set()
    next_time, previous, size = 0.0,-1.0,None
    try:
        while not stop.is_set():
            ok,frame = cap.read()
            if not ok:
                break
            timestamp = cap.get(cv2.CAP_PROP_POS_MSEC)/1000
            if not np.isfinite(timestamp) or timestamp < previous or (previous>=0 and timestamp==previous):
                raise CalibrationError('视频时间戳缺失或不递增；请使用带真实时间戳的 MKV/MP4')
            previous = timestamp
            actual=(frame.shape[1],frame.shape[0])
            if size and size!=actual:
                raise CalibrationError('视频中途改变尺寸')
            size=actual
            if camera and camera.resolution!=size:
                raise CalibrationError(f'视频尺寸 {size} 与标定 {camera.resolution} 不一致；未缩放内参')
            if timestamp+1e-6<next_time:
                continue
            next_time=timestamp+1/settings.sample_hz
            gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            detected=detector.detect(gray)
            sample={'timestamp_s':timestamp,'accepted':False,'reason':'标定板不足（AprilGrid 至少 6 标签、两行两列）'}
            if detected:
                obj,pix=detected
                quality=image_quality(gray,pix); sample['quality']=quality
                signature=np.r_[pix.mean(axis=0)/size, np.ptp(pix,axis=0)/size]
                if quality['sharpness']<40 or max(quality['dark_fraction'],quality['bright_fraction'])>.8:
                    sample['reason']='清晰度或曝光不足'
                elif any(np.linalg.norm(signature-s)<.035 for s in signatures):
                    sample['reason']='与已有位置、尺寸过于相似'
                else:
                    sample.update(accepted=True,reason='有效观测')
                    objects.append(obj);pixels.append(pix);signatures.append(signature)
                    cells=np.clip((pix/size*3).astype(int),0,2)
                    coverage.update(int(y*3+x) for x,y in cells)
            samples.append(sample)
            progress(f'已分析 {len(samples)} 帧，有效 {len(objects)} 帧，视频 {timestamp:.1f} 秒')
            if len(samples)>=600:
                break
    finally:
        cap.release()
    if stop.is_set():
        raise CalibrationError('已停止单目分析')
    if len(objects)<12:
        raise CalibrationError(f'有效不同姿态观测仅 {len(objects)} 帧，至少需要 12 帧；请补录清晰且覆盖边缘的画面')
    fit_rms=None
    if camera is None:
        camera,fit_rms=fit_camera(objects,pixels,size,settings.model)
        k=camera.K
        yaml_text=yaml.safe_dump({'cam0':{'camera_model':'pinhole','distortion_model':camera.model,'intrinsics':[float(k[0,0]),float(k[1,1]),float(k[0,2]),float(k[1,2])],'distortion_coeffs':camera.D.tolist(),'resolution':list(size),'rostopic':'/cam0/image_raw'}},sort_keys=False)
    errors,normals,depths=[],[],[]
    pose_failures = 0
    accepted=[s for s in samples if s['accepted']]
    for obj,pix,sample in zip(objects,pixels,accepted):
        # Estimate board pose with half the corners and evaluate held-out corners.
        try:
            rotation,translation=camera.pose(obj[::2],pix[::2])
        except (CalibrationError, cv2.error) as exc:
            pose_failures += 1
            sample.update(reason="位姿计算失败："+str(exc), pose_valid=False)
            continue
        residual=np.linalg.norm(camera.project(obj[1::2],rotation,translation)-pix[1::2],axis=1)
        errors.extend(residual);normals.append(rotation[:,2]);depths.append(float(translation[2,0]))
        sample['holdout_rms_px']=error_stats(residual)['rms']
    if not errors:
        raise CalibrationError("所有视角的位姿均无效，请核实镜头模型、标定板排列和内参")
    stats=error_stats(errors)
    angles=np.degrees(np.arccos(np.clip(np.asarray(normals)@np.asarray(normals).T,-1,1)))
    angle=float(angles.max()); depth_ratio=max(depths)/min(depths)
    checks=[{'name':'所有有效观测可估计位姿','passed':pose_failures==0},{'name':'有效不同姿态 ≥ 20','passed':len(objects)-pose_failures>=20}, {'name':'九宫格覆盖 ≥ 7','passed':len(coverage)>=7}, {'name':'倾角变化 ≥ 20°','passed':angle>=20}, {'name':'距离变化 ≥ 1.25 倍','passed':depth_ratio>=1.25}, {'name':'留出角点 RMS ≤ 1 px','passed':stats['rms']<=1}]
    report={'status':('fail' if stats['rms']>1 else 'pass' if all(c['passed'] for c in checks) else 'incomplete') if fit_rms is None else 'calibrated', 'pose_failures':pose_failures, 'board_corner_order':list(map(int,detector.order)) if detector.order else None, 'mode':'fixed_parameter_verification' if fit_rms is None else 'opencv_calibration', 'resolution':list(size), 'settings':report_settings,'sampled':len(samples),'accepted':len(objects),'coverage':sorted(coverage),'tilt_range_deg':angle,'depth_ratio':depth_ratio,'fit_rms_px':fit_rms,'holdout_error_px':stats,'checks':checks,'samples':samples,'scope':'固定内参复测仅重新估计标定板位姿；新求解的视频误差不是独立验收。默认阈值为工作台参考值。'}
    (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    (output/'camera-camchain.yaml').write_text(yaml_text,encoding='utf-8')
    (output/'report.html').write_text('<!doctype html><meta charset="utf-8"><title>单目检测报告</title><h1>单目检测报告</h1><p>'+html.escape(report['scope'])+'</p><pre>'+html.escape(json.dumps({k:v for k,v in report.items() if k!='samples'},ensure_ascii=False,indent=2))+'</pre>',encoding='utf-8')
    with (output/'samples.csv').open('w',newline='',encoding='utf-8-sig') as file:
        writer=csv.DictWriter(file,fieldnames=['timestamp_s','accepted','reason','holdout_rms_px'],extrasaction='ignore');writer.writeheader();writer.writerows(samples)
    (output/'processing.json').write_text(json.dumps({'status':'complete','camera_model':report_settings['model'],'engine':'opencv','mode':report['mode']},indent=2))
    return report
