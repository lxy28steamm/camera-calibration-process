from __future__ import annotations
import http.client
import json
import subprocess
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
import yaml
from ego_calibration.devices import query_capture, capabilities
from ego_calibration.inspection import _camera, StereoModel, InspectionSettings
from ego_calibration.inspection_service import InspectionService
from ego_calibration.models import CameraDevice, CalibrationError
from ego_calibration.mono_service import MonoService, MonoSettings
from ego_calibration.mono_inspection import fit_camera, BoardDetector, analyze_video
from ego_calibration.webapp import CameraWebServer


class UniversalTest(unittest.TestCase):
    def test_v4l2_metadata_nodes_are_not_capture_devices(self):
        import struct
        def ioctl(fd,req,buf):
            struct.pack_into('II',buf,84,0x80000000,0x00800000)
        with patch('ego_calibration.devices.os.open',return_value=11),patch('ego_calibration.devices.os.close'),patch('fcntl.ioctl',side_effect=ioctl):
            self.assertFalse(query_capture('/dev/test'))
        def capture_ioctl(fd,req,buf):
            struct.pack_into('II',buf,84,0x80000000,0x04000001)
        with patch('ego_calibration.devices.os.open',return_value=11),patch('ego_calibration.devices.os.close'),patch('fcntl.ioctl',side_effect=capture_ioctl):
            self.assertTrue(query_capture('/dev/test'))

    def test_generic_camera_cannot_access_vendor_storage(self):
        with tempfile.TemporaryDirectory() as root:
            service=MonoService(Path(root))
            device=CameraDevice('uvc','/dev/fake','Generic')
            self.assertTrue(capabilities(device)['mono'])
            self.assertFalse(capabilities(device)['flash'])
            with self.assertRaises(CalibrationError):
                service.flash('probe',{},device,threading.Event())

    def test_bundled_backend_does_not_execute_legacy_project(self):
        with tempfile.TemporaryDirectory() as root:
            old=Path(root)/'legacy';old.mkdir()
            (old/'run.sh').write_text('exit 99')
            service=MonoService(Path(root)/'data',old)
            self.assertTrue(service.environment()['project_ready'])
            command=service.backend('video_pipeline.py','--help')
            self.assertNotIn(str(old), ' '.join(command))
            self.assertIn('backends/run.sh',command[1])

    def test_headless_import_does_not_load_qt(self):
        result=subprocess.run([sys.executable,'-c','import sys;from ego_calibration.webapp import CameraWebServer;assert not any(k.startswith("PySide6") for k in sys.modules)'],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_generic_stereo_explicit_units_and_resolution(self):
        payload=json.loads((Path(__file__).parents[1]/'config/generic-stereo.example.json').read_text())
        model=StereoModel.from_payload('uvc-stereo',payload,InspectionSettings(width=320,height=240))
        self.assertEqual(model.left.resolution,(1920,1080))
        self.assertAlmostEqual(float(np.linalg.norm(model.T)),.06)
        bad=dict(payload,T_m=[[60],[0],[0]])
        with self.assertRaises(CalibrationError):
            StereoModel.from_payload('uvc-stereo',bad,InspectionSettings())
        with tempfile.TemporaryDirectory() as root:
            service=InspectionService(Path(root));service.selected=CameraDevice('uvc','/dev/fake','Generic')
            service._load_calibration({'payload':payload})
            self.assertEqual(service.selected.identifier,'/dev/fake')
            self.assertEqual(service.selected.kind,'uvc-stereo')

    def test_page_and_actions_work_without_login(self):
        with tempfile.TemporaryDirectory() as root:
            server=CameraWebServer(('127.0.0.1',0),InspectionService(Path(root)))
            thread=threading.Thread(target=server.serve_forever);thread.start()
            try:
                def request(path, body=None, headers=None):
                    c=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=5)
                    c.request('POST' if body is not None else 'GET',path,body,headers or {})
                    r=c.getresponse();r.read()
                    self.assertIsNone(r.getheader('WWW-Authenticate'))
                    status=r.status;c.close();return status
                self.assertEqual(request('/'),200)
                self.assertEqual(request('/api/state'),200)
                body=json.dumps({'action':'scan','data':{}})
                with patch.object(server.service,'dispatch') as dispatch:
                    self.assertEqual(request('/api/action',body),403)
                    dispatch.assert_not_called()
                    self.assertEqual(request('/api/action',body,{'X-Camera-Token':server.token}),202)
                    dispatch.assert_called_once_with('scan',{})
            finally:
                server.shutdown();server.server_close();thread.join()

    def test_opencv_recovers_known_intrinsics_and_handles_strided_points(self):
        camera=_camera([[800,0,640],[0,805,400],[0,0,1]],[.01,-.004,.0001,.0002],[1280,800])
        points=InspectionSettings().points(list(range(36)))
        objects,pixels=[],[]
        for i in range(16):
            rotation=cv2.Rodrigues(np.array([.04*i-.3,.08*(i%5)-.2,.025*i]))[0]
            translation=np.array([[-.25+.025*(i%4)],[-.2+.03*(i%3)],[.9+.04*i]])
            pix=camera.project(points,rotation,translation)
            camera.pose(points[::2],pix[::2])
            objects.append(points);pixels.append(pix)
        fit,rms=fit_camera(objects,pixels,(1280,800),'pinhole-radtan')
        self.assertLess(rms,.001)
        np.testing.assert_allclose(fit.K,camera.K,atol=.01)
        with self.assertRaises(CalibrationError):
            fit_camera(objects,pixels,(1280,800),'ds-none')

    def test_fisheye_pose_and_projection_accept_strided_corner_arrays(self):
        camera=_camera([[800,0,640],[0,805,400],[0,0,1]],[.01,-.004,.0001,.0002],[1280,800],"equidistant")
        obj=InspectionSettings().points(list(range(36)))
        r=cv2.Rodrigues(np.array([.2,-.3,.1]))[0];t=np.array([[-.2],[-.2],[1.]])
        pixels=camera.project(obj,r,t)
        estimate_r,estimate_t=camera.pose(obj[::2],pixels[::2])
        np.testing.assert_allclose(camera.project(obj[1::2],estimate_r,estimate_t),pixels[1::2],atol=.01)

    def test_aprilgrid_pixels_match_known_synthetic_geometry(self):
        from test_web_inspection import rendered_pair
        from test_inspection import inspection_payload, inspection_settings
        settings=inspection_settings()
        frame=rendered_pair(settings, (.2,-.15,.1), 1.1)[:, :1280]
        camera=StereoModel.from_payload("ego-std",inspection_payload(),settings).left
        detected=BoardDetector(MonoSettings(),camera).detect(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY))
        self.assertIsNotNone(detected)
        points,pixels=detected
        r,t=camera.pose(points,pixels)
        rms=np.sqrt(np.mean(np.sum((camera.project(points,r,t)-pixels)**2,axis=1)))
        self.assertLess(rms,1.0)

    def test_mono_video_rejects_calibration_dimension_mismatch(self):
        with tempfile.TemporaryDirectory() as root:
            video=Path(root)/'test.avi'
            writer=cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*'MJPG'),10,(320,240))
            for _ in range(5):writer.write(np.zeros((240,320,3),np.uint8))
            writer.release()
            text=yaml.safe_dump({'cam0':{'camera_model':'pinhole','distortion_model':'radtan','intrinsics':[200,200,160,120],'distortion_coeffs':[0,0,0,0],'resolution':[640,480]}})
            with self.assertRaisesRegex(CalibrationError,'尺寸'):
                analyze_video(video,MonoSettings(),Path(root)/'out',threading.Event(),yaml_text=text)
            self.assertFalse((Path(root)/'out/camera-camchain.yaml').exists())

if __name__=='__main__':unittest.main()
