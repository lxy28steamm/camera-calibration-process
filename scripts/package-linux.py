"""Build a relocatable binary + source archive, without recordings or user config."""
import argparse
import hashlib
import re
import shutil
import tarfile
import tempfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
VERSION=re.search(r'__version__ = "([^"]+)"', (ROOT/'src/ego_calibration/__init__.py').read_text()).group(1)
parser=argparse.ArgumentParser()
parser.add_argument('--binary',type=Path,default=ROOT/'dist/ego-calibration-linux-x86_64.bin')
parser.add_argument('--output',type=Path,default=ROOT/'dist')
args=parser.parse_args()
args.output.mkdir(parents=True,exist_ok=True)
with tempfile.TemporaryDirectory() as staging:
    target=Path(staging)/'camera-workbench'
    target.mkdir()
    for directory in ('src','scripts','tests','docs','config'):
        shutil.copytree(ROOT/directory,target/directory,ignore=shutil.ignore_patterns('__pycache__','*.pyc','*.egg-info','local.env'))
    for file in ('pyproject.toml','README.md','start-workbench.sh','requirements-build.txt'):
        shutil.copy2(ROOT/file,target/file)
    shutil.copy2(args.binary,target/'camera-workbench-linux-x86_64.bin')
    (target/'data').mkdir()
    files={str(p.relative_to(target)):hashlib.sha256(p.read_bytes()).hexdigest() for p in target.rglob('*') if p.is_file()}
    (target/'SHA256SUMS').write_text(''.join(f'{digest}  {name}\n' for name,digest in sorted(files.items())))
    archive=args.output/f'camera-workbench-{VERSION}-linux-x86_64.tar.gz'
    with tarfile.open(archive,'w:gz') as tar:
        tar.add(target,arcname=target.name)
    digest=hashlib.sha256(archive.read_bytes()).hexdigest()
    archive.with_suffix(archive.suffix+'.sha256').write_text(f'{digest}  {archive.name}\n')
    print(archive)
