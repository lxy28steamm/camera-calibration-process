from __future__ import annotations

import ctypes.util
import os
import platform
import plistlib
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LINUX_SYSTEM_LIBRARIES = (r"libstdc\+\+\.so(\..*)?", r"libgcc_s\.so(\..*)?")
MAX_LINUX_BUILD_GLIBC = (2, 35)


def target_name() -> str:
    system = {
        "Darwin": "macos",
        "Linux": "linux",
        "Windows": "windows",
    }.get(platform.system(), platform.system().lower())
    architecture = platform.machine().lower().replace("amd64", "x86_64")
    if architecture in {"arm64", "aarch64"}:
        architecture = "arm64"
    suffix = ".exe" if system == "windows" else ".bin"
    return f"ego-calibration-{system}-{architecture}{suffix}"


def linux_extra_binary() -> Path | None:
    if platform.system() != "Linux":
        return None
    candidates = (
        Path("/usr/lib/x86_64-linux-gnu/libxcb-cursor.so.0"),
        Path("/usr/lib/aarch64-linux-gnu/libxcb-cursor.so.0"),
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate
    library = ctypes.util.find_library("xcb-cursor")
    if library and Path(library).is_absolute():
        return Path(library)

    dependency_root = ROOT / ".build-dependencies"
    extracted = next(dependency_root.rglob("libxcb-cursor.so.0"), None)
    if extracted is not None:
        return extracted
    dependency_root.mkdir(exist_ok=True)
    try:
        subprocess.run(
            ["apt-get", "download", "libxcb-cursor0"],
            cwd=dependency_root,
            check=True,
        )
        package = next(dependency_root.glob("libxcb-cursor0_*.deb"))
        subprocess.run(
            ["dpkg-deb", "-x", str(package), str(dependency_root / "root")],
            check=True,
        )
    except (OSError, StopIteration, subprocess.CalledProcessError) as exc:
        raise RuntimeError("无法准备 Linux Qt 运行库 libxcb-cursor.so.0") from exc
    return next((dependency_root / "root").rglob("libxcb-cursor.so.0"))


def main() -> int:
    ensure_linux_build_compatibility()
    name = target_name()
    build_name = name.removesuffix(".exe")
    command = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--noconfirm",
        "--clean",
        "--onefile",
        "--windowed",
        "--name",
        build_name,
        "--paths",
        str(ROOT / "src"),
        "--collect-all",
        "depthai",
        "--collect-all",
        "libusb_package",
        "--collect-data",
        "ego_calibration",
        "--hidden-import",
        "usb.backend.libusb1",
        # Loaded dynamically by the preserved Ego-Std delivery driver.
        "--hidden-import",
        "filecmp",
        "--hidden-import",
        "mmap",
        "--add-data",
        f"{ROOT / 'src/ego_calibration/backends'}:ego_calibration/backends",
        "--distpath",
        str(ROOT / "dist"),
        "--workpath",
        str(ROOT / "build"),
        "--specpath",
        str(ROOT / "build"),
    ]
    extra_binary = linux_extra_binary()
    if extra_binary is not None:
        command.extend(["--add-binary", f"{extra_binary}:."])
    if platform.system() == "Linux":
        command.extend(
            [
                "--add-data",
                f"{ROOT / 'scripts/kalibr-env.sh'}:scripts",
                "--add-data",
                f"{ROOT / 'scripts/kalibr-env-docker.sh'}:scripts",
                "--add-data",
                f"{ROOT / 'scripts/kalibr-docker-command.sh'}:scripts",
                "--add-data",
                f"{ROOT / 'scripts/docker'}:scripts/docker",
            ]
        )
    if platform.system() == "Darwin":
        command.extend(
            ["--osx-bundle-identifier", "com.livsyn.ego-calibration"]
        )
    if platform.system() == "Windows":
        command.extend(["--collect-all", "comtypes"])
    command.append(str(ROOT / "src/ego_calibration/__main__.py"))
    run_pyinstaller(command)

    if platform.system() == "Darwin":
        bundle = ROOT / "dist" / f"{build_name}.app"
        configure_macos_bundle(bundle)
        subprocess.run(
            ["codesign", "--force", "--deep", "--sign", "-", str(bundle)],
            check=True,
        )
        bundle_binary = bundle / f"Contents/MacOS/{build_name}"
        final_binary = ROOT / "dist" / name
        if bundle_binary.exists():
            shutil.copy2(bundle_binary, final_binary)
            final_binary.chmod(0o755)
    print(ROOT / "dist" / name)
    return 0


def run_pyinstaller(command: list[str]) -> None:
    if platform.system() != "Linux":
        subprocess.run(command, cwd=ROOT, check=True)
        return

    from PyInstaller import __main__ as pyinstaller
    from PyInstaller.depend import dylib

    dylib.exclude_list = dylib.MatchList(
        (*dylib._excludes, *LINUX_SYSTEM_LIBRARIES)
    )
    pyinstaller.run(command[3:])
    verify_linux_bundle(ROOT / "dist" / target_name())


def ensure_linux_build_compatibility() -> None:
    if platform.system() != "Linux":
        return
    _libc_name, version = platform.libc_ver()
    current = tuple(int(part) for part in version.split(".")[:2])
    if current > MAX_LINUX_BUILD_GLIBC:
        if os.environ.get("EGO_ALLOW_UNSUPPORTED_GLIBC") == "1":
            print(
                "警告：使用当前 GLIBC 构建本机测试包；该包不保证兼容旧版 Linux。"
            )
            return
        raise RuntimeError(
            "Linux 发布包必须在 Ubuntu 22.04 / GLIBC 2.35 或更旧环境构建；"
            f"当前 GLIBC 为 {version}"
        )


def verify_linux_bundle(binary: Path) -> None:
    from PyInstaller.archive.readers import CArchiveReader

    names = CArchiveReader(str(binary)).toc
    forbidden = {"libstdc++.so.6", "libgcc_s.so.1"} & names.keys()
    if forbidden:
        libraries = ", ".join(sorted(forbidden))
        raise RuntimeError(f"Linux 成品错误包含宿主系统运行库：{libraries}")


def configure_macos_bundle(bundle: Path) -> None:
    info_path = bundle / "Contents/Info.plist"
    with info_path.open("rb") as source:
        info = plistlib.load(source)
    info["NSCameraUsageDescription"] = "用于预览 Ego 相机画面并检查对焦清晰度"
    with info_path.open("wb") as destination:
        plistlib.dump(info, destination, sort_keys=False)


if __name__ == "__main__":
    raise SystemExit(main())
