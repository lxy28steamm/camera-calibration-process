"""Run the preserved customer workflow with a relocatable native or Docker runtime."""
from __future__ import annotations

import argparse
import importlib.util
import os
import subprocess
import sys
import tarfile
import uuid
from pathlib import Path

DELIVERY = Path(__file__).with_name('ego_std_delivery')


def load_pipeline():
    spec = importlib.util.spec_from_file_location('customer_std_pipeline', DELIVERY/'scripts/calibrate_h264_imu.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def prepare_source(directory):
    root = Path(directory).resolve()
    expected = root/'kalibr_source/.extracted'
    if not expected.exists():
        root.mkdir(parents=True, exist_ok=True)
        with tarfile.open(DELIVERY/'kalibr_source.tar.gz') as archive:
            for member in archive:
                target = (root/member.name).resolve()
                if not target.is_relative_to(root):
                    raise RuntimeError('源码归档路径无效')
                if member.isdir():
                    target.mkdir(parents=True, exist_ok=True)
                elif member.isfile():
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with archive.extractfile(member) as source, target.open('wb') as destination:
                        import shutil
                        shutil.copyfileobj(source, destination)
                    target.chmod(member.mode & 0o777)
                # The delivery has cyclic catkin test symlinks, unused at runtime.
        expected.touch()
    return root/'kalibr_source'


def native_command(pipeline, source, output, inner, video=None):
    python_root = source/'aslam_offline_calibration/kalibr/python'
    mapping = {'/data': str(output), '/work': str(DELIVERY/'scripts'),
               pipeline.KALIBR_SOURCE_MOUNT: str(source)}
    if video:
        mapping['/input/source.h264'] = str(video)
    def translate(arg):
        for prefix, replacement in sorted(mapping.items(), key=lambda item: -len(item[0])):
            if arg == prefix or arg.startswith(prefix+'/'):
                return replacement + arg[len(prefix):]
        return arg
    args = [translate(arg) for arg in inner]
    if args[:2] == ['rosrun', 'kalibr']:
        args = [sys.executable, str(Path(__file__).with_name('std_kalibr_runner.py')), str(python_root/args[2]), *args[3:]]
    elif args[0] == 'python3':
        args[0] = sys.executable
        if Path(args[1]).name in ('kalibr_calibrate_cameras', 'kalibr_calibrate_imu_camera'):
            args.insert(1, str(Path(__file__).with_name('std_kalibr_runner.py')))
    # Use delivery Python patches with the installed Kalibr compiled extensions.
    return ['env', 'PYTHONPATH='+str(python_root)+os.pathsep+os.environ.get('PYTHONPATH', ''), *args]


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--runtime', choices=('native', 'docker'), default='native')
    parser.add_argument('--runtime-dir', required=True)
    wrapper, remaining = parser.parse_known_args()
    pipeline = load_pipeline()
    args = pipeline.build_parser().parse_args(remaining)
    args.dump_tool = str(DELIVERY/'bin/video_user_data_dump')
    source = prepare_source(wrapper.runtime_dir)
    pipeline.KALIBR_SOURCE_ROOT = source
    containers = []
    if wrapper.runtime == 'native':
        pipeline.ensure_docker = lambda *_: None
        pipeline.docker_command = lambda _args, output, command, selected_video=None: native_command(pipeline, source, output, command, selected_video)
    else:
        original = pipeline.docker_command
        def docker_command(*positional, **keywords):
            command = original(*positional, **keywords)
            name = 'camera-workbench-'+uuid.uuid4().hex
            command[2:2] = ['--name', name]
            containers.append(name)
            return command
        pipeline.docker_command = docker_command
    try:
        return pipeline.run_pipeline(args)
    except (pipeline.PipelineError, OSError, subprocess.SubprocessError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        for name in containers:
            subprocess.run([args.docker, 'rm', '-f', name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)


if __name__ == '__main__':
    raise SystemExit(main())
