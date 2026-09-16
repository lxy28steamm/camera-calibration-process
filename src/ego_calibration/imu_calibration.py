from __future__ import annotations

import shlex
import threading
from collections.abc import Callable
from pathlib import Path
from typing import Any

from PySide6.QtCore import QProcess, QThread, Qt, Signal
from PySide6.QtGui import QCloseEvent, QFontDatabase, QImage, QPixmap
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ego_calibration.models import CameraDevice
from ego_calibration.preview import AprilGridPreview
from ego_calibration.oak_imu_calibration import (
    CaptureSummary,
    ImuNoiseCaptureSummary,
    capture_imu_noise_dataset,
    create_session_directory,
    create_imu_noise_session_directory,
    capture_kalibr_dataset,
    default_imu_path,
    detect_kalibr_environment,
    flash_kalibr_result,
    flash_backend_capabilities,
    kalibr_commands,
    kalibr_process_spec,
    load_kalibr_result,
    validate_kalibr_dataset,
    write_default_aprilgrid_yaml,
)


_STYLE = """
QDialog { background: #F3F6FA; color: #172B4D; }
QLabel#title { font-size: 19px; font-weight: 700; }
QLabel#section { font-size: 15px; font-weight: 700; color: #172B4D; }
QLabel#hint { color: #5E6C84; }
QLabel#warning { background: #FFF7ED; color: #9A3412; padding: 8px 10px; }
QLabel#capturePreviewInfo { background: #EFF6FF; color: #1D4ED8; padding: 6px 10px; font-weight: 700; }
QLabel#capturePreview { background: #111827; color: #C9D5E6; border: 1px solid #344563; }
QLineEdit, QSpinBox { background: #FFFFFF; border: 1px solid #D8E0EA; padding: 7px; }
QPushButton { border: 0; padding: 8px 14px; font-weight: 700; }
QPushButton#primary { background: #2563EB; color: #FFFFFF; }
QPushButton#danger { background: #B91C1C; color: #FFFFFF; }
QPushButton#secondary { background: #E9EEF5; color: #253858; }
QPushButton:disabled { background: #E5E7EB; color: #9CA3AF; }
QPlainTextEdit { background: #111827; color: #D1FAE5; border: 0; padding: 8px; }
QProgressBar { background: #E7ECF3; border: 0; text-align: center; min-height: 18px; }
QProgressBar::chunk { background: #2563EB; }
"""


class _CaptureThread(QThread):
    progressed = Signal(float, int, int, int)
    previewed = Signal(QImage, str)
    succeeded = Signal(object)
    failed = Signal(str)

    def __init__(
        self,
        mxid: str,
        directory: Path,
        duration_seconds: int,
    ) -> None:
        super().__init__()
        self.mxid = mxid
        self.directory = directory
        self.duration_seconds = duration_seconds
        self._stop_requested = threading.Event()
        self._preview_builder: AprilGridPreview | None = None

    def stop(self) -> None:
        self._stop_requested.set()

    def run(self) -> None:
        try:
            self._preview_builder = AprilGridPreview()
            result = capture_kalibr_dataset(
                self.mxid,
                self.directory,
                self.duration_seconds,
                should_stop=self._stop_requested.is_set,
                progress=self.progressed.emit,
                preview=self._preview_ready,
            )
        except Exception as exc:
            self.failed.emit(str(exc) or type(exc).__name__)
            return
        self.succeeded.emit(result)

    def _preview_ready(
        self,
        left_frame: Any,
        right_frame: Any,
        _left_count: int,
        _right_count: int,
    ) -> None:
        if self._preview_builder is None:
            return
        try:
            image, status = self._preview_builder.render(left_frame, right_frame)
        except Exception:
            # Preview is auxiliary; a detector or display conversion failure must
            # never interrupt the raw calibration data capture.
            return
        self.previewed.emit(image, status)


class _ImuNoiseCaptureThread(QThread):
    progressed = Signal(float, int)
    succeeded = Signal(object)
    failed = Signal(str)

    def __init__(
        self,
        mxid: str,
        directory: Path,
        duration_seconds: int,
    ) -> None:
        super().__init__()
        self.mxid = mxid
        self.directory = directory
        self.duration_seconds = duration_seconds
        self._stop_requested = threading.Event()

    def stop(self) -> None:
        self._stop_requested.set()

    def run(self) -> None:
        try:
            result = capture_imu_noise_dataset(
                self.mxid,
                self.directory,
                self.duration_seconds,
                should_stop=self._stop_requested.is_set,
                progress=self.progressed.emit,
            )
        except Exception as exc:
            self.failed.emit(str(exc) or type(exc).__name__)
            return
        self.succeeded.emit(result)


class _ActionThread(QThread):
    succeeded = Signal(object)
    failed = Signal(str)

    def __init__(self, action: Callable[[], Any]) -> None:
        super().__init__()
        self.action = action

    def run(self) -> None:
        try:
            result = self.action()
        except Exception as exc:
            self.failed.emit(str(exc) or type(exc).__name__)
            return
        self.succeeded.emit(result)


class CameraImuCalibrationDialog(QDialog):
    def __init__(self, device: CameraDevice, parent: Any = None) -> None:
        super().__init__(parent)
        self.device = device
        self._capture_thread: _CaptureThread | None = None
        self._imu_noise_thread: _ImuNoiseCaptureThread | None = None
        self._flash_thread: _ActionThread | None = None
        self._process: QProcess | None = None
        self._commands: list[tuple[str, ...]] = []
        self._command_index = 0
        self._kalibr_environment = detect_kalibr_environment()
        self._capture_preview_image: QImage | None = None
        self.setWindowTitle(f"OAK 相机与 IMU 标定 · {device.label}")
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        self.setWindowModality(Qt.WindowModality.WindowModal)
        self.resize(980, 900)
        self.setMinimumSize(840, 720)
        self.setStyleSheet(_STYLE)

        self.output_root = QLineEdit(str(Path.home() / "ego-camera-imu-data"))
        self.dataset = QLineEdit()
        self.dataset.setPlaceholderText("采集后自动填写，也可选择已有 Kalibr 数据集")
        self.dataset.textChanged.connect(self._update_dataset_summary)
        self.duration = QSpinBox()
        self.duration.setRange(10, 600)
        self.duration.setValue(120)
        self.duration.setSuffix(" 秒")
        self.capture_progress = QProgressBar()
        self.capture_progress.setRange(0, 1000)
        self.capture_progress.setValue(0)
        self.capture_preview_info = QLabel(
            "动态采集开始后显示已去畸变的左右灰度实时画面和 AprilTag 检测数量"
        )
        self.capture_preview_info.setObjectName("capturePreviewInfo")
        self.capture_preview = QLabel("等待动态采集画面…")
        self.capture_preview.setObjectName("capturePreview")
        self.capture_preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.capture_preview.setMinimumHeight(250)

        self.capture_button = self._button("开始采集", "primary")
        self.capture_button.clicked.connect(self._start_capture)
        self.stop_capture_button = self._button("停止采集", "secondary")
        self.stop_capture_button.setEnabled(False)
        self.stop_capture_button.clicked.connect(self._stop_capture)

        self.imu_noise_dataset = QLineEdit()
        self.imu_noise_dataset.setReadOnly(True)
        self.imu_noise_dataset.setPlaceholderText("静置采集后自动填写")
        self.imu_noise_duration = QSpinBox()
        self.imu_noise_duration.setRange(10, 360)
        self.imu_noise_duration.setValue(120)
        self.imu_noise_duration.setSuffix(" 分钟")
        self.imu_noise_progress = QProgressBar()
        self.imu_noise_progress.setRange(0, 1000)
        self.imu_noise_progress.setValue(0)
        self.imu_noise_capture_button = self._button("开始静置采集", "primary")
        self.imu_noise_capture_button.clicked.connect(self._start_imu_noise_capture)
        self.stop_imu_noise_button = self._button("停止静置采集", "secondary")
        self.stop_imu_noise_button.setEnabled(False)
        self.stop_imu_noise_button.clicked.connect(self._stop_imu_noise_capture)

        self._default_imu_yaml = default_imu_path()
        self.target_yaml = QLineEdit()
        self.target_yaml.setPlaceholderText(
            "采集后自动选择数据集里的默认 target.yaml，也可手动替换"
        )
        self.target_yaml.setToolTip(
            "默认配置为 6x6 tag36H11、tagSize 5.5 cm、间距 1.65 cm（tagSpacing=0.3）；"
            "如使用其他标定板，请手动选择对应 YAML。"
        )
        self.imu_yaml = QLineEdit(str(self._default_imu_yaml))
        self.imu_yaml.setPlaceholderText("请选择 BNO086 实测 IMU 噪声 YAML")
        self.imu_yaml.setToolTip(
            f"默认使用 BNO086 实测参数：{self._default_imu_yaml}；也可手动选择其他 imu.yaml"
        )
        self.kalibr_setup = QLineEdit(
            str(self._kalibr_environment.setup_script)
            if self._kalibr_environment.setup_script is not None
            else ""
        )
        self.kalibr_setup.setPlaceholderText(
            "自动检测；可填写 ROS/Kalibr setup.bash 或 scripts/kalibr-env.sh"
        )
        self.kalibr_setup.setToolTip(
            "若 Kalibr 安装在 Conda 或 catkin 工作空间，请选择能加载该环境的 setup.bash。"
        )
        self.kalibr_status = QLabel()
        self.kalibr_status.setObjectName("hint")
        self.kalibr_check_button = self._button("检查 Kalibr 环境", "secondary")
        self.kalibr_check_button.clicked.connect(self._check_kalibr_environment)
        self.run_button = self._button("运行 Kalibr", "primary")
        self.run_button.clicked.connect(self._run_kalibr)
        self.stop_process_button = self._button("停止求解", "secondary")
        self.stop_process_button.setEnabled(False)
        self.stop_process_button.clicked.connect(self._stop_process)

        self.result_yaml = QLineEdit()
        self.result_yaml.setPlaceholderText("Kalibr 输出的 *-camchain-imucam.yaml")
        self.flash_backend = QComboBox()
        self.flash_backend.addItem(
            "DepthAI V2（默认 · flashCalibration2）", "v2"
        )
        self.flash_backend.addItem("DepthAI V3（flashCalibration）", "v3")
        self.flash_runtime = QLabel()
        self.flash_runtime.setObjectName("hint")
        self._configure_flash_backends()
        self.check_button = self._button("检查结果", "secondary")
        self.check_button.clicked.connect(self._check_result)
        self.flash_button = self._button("备份并写入 EEPROM", "danger")
        self.flash_button.clicked.connect(self._flash_result)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))

        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        content = QWidget()
        content.setMinimumWidth(800)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(20, 18, 20, 18)
        layout.setSpacing(11)
        title = QLabel("OAK 相机与 IMU 联合标定")
        title.setObjectName("title")
        subtitle = QLabel(
            "OAK 包含上方 RGB 单目和左/右灰度双目；动态外参标定使用 "
            "cam0=左灰度 CAM_B、cam1=右灰度 CAM_C，并与 IMU 共用设备单调时钟。"
        )
        subtitle.setObjectName("hint")
        subtitle.setWordWrap(True)
        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addWidget(self._warning())
        layout.addWidget(self._section("1. 采集左/右灰度双目与原始 IMU（动态外参）"))

        capture_grid = QGridLayout()
        capture_grid.setColumnStretch(1, 1)
        capture_grid.addWidget(QLabel("保存根目录"), 0, 0)
        capture_grid.addWidget(self.output_root, 0, 1)
        capture_grid.addWidget(
            self._browse_button("选择…", lambda: self._choose_directory(self.output_root)),
            0,
            2,
        )
        capture_grid.addWidget(QLabel("采集时长"), 1, 0)
        capture_grid.addWidget(self.duration, 1, 1)
        capture_actions = QHBoxLayout()
        capture_actions.addWidget(self.capture_button)
        capture_actions.addWidget(self.stop_capture_button)
        capture_grid.addLayout(capture_actions, 1, 2)
        capture_grid.addWidget(QLabel("数据集目录"), 2, 0)
        capture_grid.addWidget(self.dataset, 2, 1)
        capture_grid.addWidget(
            self._browse_button("选择已有…", lambda: self._choose_directory(self.dataset)),
            2,
            2,
        )
        capture_grid.addWidget(self.capture_progress, 3, 0, 1, 3)
        capture_grid.addWidget(self.capture_preview_info, 4, 0, 1, 3)
        capture_grid.addWidget(self.capture_preview, 5, 0, 1, 3)
        layout.addLayout(capture_grid)

        layout.addWidget(self._section("2. 原始 IMU 静置噪声采集（默认约 2 小时）"))
        layout.addWidget(self._hint_label(
            "只采集 200 Hz 原始加速度计和陀螺仪。设备必须固定在无振动平台；"
            "输出 imu0.csv，之后通过 Allan 方差或 imu_utils 生成实测 imu.yaml。"
            "相同 BNO086 硬件、固件及采样配置可复用同一型号级参数。"
        ))
        noise_grid = QGridLayout()
        noise_grid.setColumnStretch(1, 1)
        noise_grid.addWidget(QLabel("静置时长"), 0, 0)
        noise_grid.addWidget(self.imu_noise_duration, 0, 1)
        noise_actions = QHBoxLayout()
        noise_actions.addWidget(self.imu_noise_capture_button)
        noise_actions.addWidget(self.stop_imu_noise_button)
        noise_grid.addLayout(noise_actions, 0, 2)
        noise_grid.addWidget(QLabel("输出目录"), 1, 0)
        noise_grid.addWidget(self.imu_noise_dataset, 1, 1, 1, 2)
        noise_grid.addWidget(self.imu_noise_progress, 2, 0, 1, 3)
        layout.addLayout(noise_grid)

        layout.addWidget(self._section("3. 使用 Kalibr 求解 IMU → 左灰度相机外参"))
        solve_grid = QGridLayout()
        solve_grid.setColumnStretch(1, 1)
        solve_grid.addWidget(QLabel("数据集目录"), 0, 0)
        self.dataset_summary = QLabel()
        self.dataset_summary.setObjectName("hint")
        self.dataset_summary.setWordWrap(True)
        solve_grid.addWidget(self.dataset_summary, 0, 1)
        solve_grid.addWidget(
            self._browse_button("选择已有数据集…", lambda: self._choose_directory(self.dataset)),
            0,
            2,
        )
        solve_grid.addWidget(QLabel("AprilGrid 配置"), 1, 0)
        solve_grid.addWidget(self.target_yaml, 1, 1)
        solve_grid.addWidget(
            self._browse_button("选择…", self._choose_aprilgrid_yaml),
            1,
            2,
        )
        solve_grid.addWidget(QLabel("IMU 噪声配置"), 2, 0)
        solve_grid.addWidget(self.imu_yaml, 2, 1)
        solve_grid.addWidget(
            self._browse_button("选择…", lambda: self._choose_yaml(self.imu_yaml)),
            2,
            2,
        )
        solve_grid.addWidget(QLabel("Kalibr 环境脚本"), 3, 0)
        solve_grid.addWidget(self.kalibr_setup, 3, 1)
        solve_grid.addWidget(
            self._browse_button("选择…", self._choose_kalibr_setup),
            3,
            2,
        )
        solve_grid.addWidget(self.kalibr_status, 4, 0, 1, 2)
        solve_actions = QHBoxLayout()
        solve_actions.addStretch()
        solve_actions.addWidget(self.stop_process_button)
        solve_actions.addWidget(self.run_button)
        solve_grid.addWidget(self.kalibr_check_button, 4, 2)
        solve_grid.addLayout(solve_actions, 5, 0, 1, 3)
        layout.addLayout(solve_grid)

        layout.addWidget(self._section("4. 检查结果并写入 OAK EEPROM"))
        result_grid = QGridLayout()
        result_grid.setColumnStretch(1, 1)
        result_grid.addWidget(QLabel("Kalibr 结果"), 0, 0)
        result_grid.addWidget(self.result_yaml, 0, 1)
        result_grid.addWidget(
            self._browse_button("选择…", lambda: self._choose_yaml(self.result_yaml)),
            0,
            2,
        )
        result_grid.addWidget(QLabel("EEPROM 写入接口"), 1, 0)
        result_grid.addWidget(self.flash_backend, 1, 1)
        result_grid.addWidget(self.flash_runtime, 2, 1)
        result_actions = QHBoxLayout()
        result_actions.addWidget(self.check_button)
        result_actions.addWidget(self.flash_button)
        result_grid.addLayout(result_actions, 1, 2, 2, 1)
        layout.addLayout(result_grid)
        layout.addWidget(self.log, 1)

        footer = QHBoxLayout()
        footer.addWidget(self._hint_label(
            "Kalibr 求解工具通常只在 Linux/ROS 环境安装；Windows/macOS 可采集和检查结果，"
            "再把数据集复制到 Linux 求解。"
        ), 1)
        close_button = self._button("关闭", "secondary")
        close_button.clicked.connect(self.close)
        footer.addWidget(close_button)
        layout.addLayout(footer)
        scroll.setWidget(content)
        root_layout.addWidget(scroll)
        self._append_log(f"设备：{device.identifier}")
        self._append_log(
            "默认 AprilGrid：6x6 tag36H11、tagSize 5.5 cm、间距 1.65 cm；"
            "采集后自动写入数据集 target.yaml，也可手动替换。"
        )
        self._append_log(f"默认 IMU 噪声配置：{self._default_imu_yaml}")
        self.kalibr_status.setText(
            "Kalibr 环境已就绪（可直接运行）"
            if self._kalibr_environment.ready
            else "尚未检测到 Kalibr；点击“检查 Kalibr 环境”或选择 setup.bash"
        )
        self._update_dataset_summary(self.dataset.text())

    def _configure_flash_backends(self) -> None:
        try:
            import depthai as dai

            version = str(getattr(dai, "__version__", "未知"))
            capabilities = flash_backend_capabilities(dai)
        except Exception:
            version = "不可用"
            capabilities = {"v2": False, "v3": False}
        for index in range(self.flash_backend.count()):
            backend = str(self.flash_backend.itemData(index))
            item = self.flash_backend.model().item(index)
            if item is not None:
                item.setEnabled(capabilities.get(backend, False))
        available = [name.upper() for name, enabled in capabilities.items() if enabled]
        self.flash_runtime.setText(
            f"当前 DepthAI {version} · 可用接口：{', '.join(available) or '无'}"
        )
        if not capabilities.get("v2") and capabilities.get("v3"):
            self.flash_backend.setCurrentIndex(1)

    @staticmethod
    def _button(text: str, object_name: str) -> QPushButton:
        button = QPushButton(text)
        button.setObjectName(object_name)
        return button

    def _browse_button(self, text: str, action: Callable[[], None]) -> QPushButton:
        button = self._button(text, "secondary")
        button.clicked.connect(action)
        return button

    @staticmethod
    def _section(text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("section")
        return label

    @staticmethod
    def _hint_label(text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("hint")
        label.setWordWrap(True)
        return label

    def _warning(self) -> QLabel:
        label = QLabel(
            "采集时固定 AprilGrid，缓慢移动整台相机并充分激励三个旋转轴和三个平移轴；"
            "避免碰撞、强振动、运动模糊和曝光变化。采集后会在数据集根目录生成默认"
            "target.yaml；如标定板不同再手动选择对应配置。IMU 默认使用 BNO086 实测参数。"
        )
        label.setObjectName("warning")
        label.setWordWrap(True)
        return label

    def _choose_directory(self, field: QLineEdit) -> None:
        selected = QFileDialog.getExistingDirectory(
            self,
            "选择目录",
            field.text().strip() or str(Path.home()),
        )
        if selected:
            field.setText(selected)

    def _update_dataset_summary(self, value: str) -> None:
        """在求解区重复显示数据集入口，避免用户只看到采集区。"""
        if not hasattr(self, "dataset_summary"):
            return
        path = value.strip()
        self.dataset_summary.setText(
            path or "尚未选择；请点击右侧“选择已有数据集…”（或先完成动态采集）"
        )
        if path:
            default_target = Path(path).expanduser() / "target.yaml"
            if default_target.is_file():
                self.target_yaml.setText(str(default_target))

    def _choose_yaml(self, field: QLineEdit) -> None:
        selected, _filter = QFileDialog.getOpenFileName(
            self,
            "选择 YAML 文件",
            field.text().strip() or str(Path.home()),
            "YAML 文件 (*.yaml *.yml);;所有文件 (*)",
        )
        if selected:
            field.setText(selected)

    def _choose_aprilgrid_yaml(self) -> None:
        current = self.target_yaml.text().strip()
        dataset = Path(self.dataset.text().strip()).expanduser()
        start = current or (str(dataset) if dataset.is_dir() else str(Path.home()))
        selected, _filter = QFileDialog.getOpenFileName(
            self,
            "选择数据集里的 AprilGrid YAML",
            start,
            "YAML 文件 (*.yaml *.yml);;所有文件 (*)",
        )
        if selected:
            self.target_yaml.setText(selected)

    def _choose_kalibr_setup(self) -> None:
        selected, _filter = QFileDialog.getOpenFileName(
            self,
            "选择 ROS/Kalibr 环境脚本",
            self.kalibr_setup.text().strip() or str(Path.home()),
            "Shell 脚本 (*.bash *.sh);;所有文件 (*)",
        )
        if selected:
            self.kalibr_setup.setText(selected)
            self._check_kalibr_environment()

    def _check_kalibr_environment(self) -> None:
        configured = self.kalibr_setup.text().strip()
        setup = Path(configured).expanduser() if configured else None
        if setup is not None and not setup.is_file():
            corrected = self._correct_project_path_typo(setup)
            if corrected is not None:
                self.kalibr_setup.setText(str(corrected))
                setup = corrected
            else:
                self._kalibr_environment = detect_kalibr_environment(setup)
                self.kalibr_status.setText(
                    f"环境脚本不存在：{setup}；请改选实际存在的 setup.bash 或 kalibr-env.sh"
                )
                return
        self._kalibr_environment = detect_kalibr_environment(setup)
        if self._kalibr_environment.ready:
            locations = ", ".join(
                str(self._kalibr_environment.commands[name])
                for name in ("kalibr_bagcreater", "kalibr_calibrate_imu_camera")
            )
            self.kalibr_status.setText(
                f"Kalibr 环境正常（{self._kalibr_environment.label}）：{locations}"
            )
            return
        missing = "、".join(
            (*self._kalibr_environment.missing, *self._kalibr_environment.runtime_missing)
        )
        self.kalibr_status.setText(
            f"Kalibr 环境未就绪，缺少：{missing}；请确认脚本已加载 ROS/Kalibr，"
            "或运行 scripts/install-kalibr-conda.sh / install-kalibr-docker.sh"
        )

    @staticmethod
    def _correct_project_path_typo(path: Path) -> Path | None:
        """兼容历史目录名 ego-calibration 与项目实际目录 ego-calibartion。"""
        text = str(path)
        alternatives = []
        if "ego-calibration" in text:
            alternatives.append(Path(text.replace("ego-calibration", "ego-calibartion")))
        if "ego-calibartion" in text:
            alternatives.append(Path(text.replace("ego-calibartion", "ego-calibration")))
        return next((candidate for candidate in alternatives if candidate.is_file()), None)

    def _start_capture(self) -> None:
        if self._is_busy():
            return
        root_text = self.output_root.text().strip()
        if not root_text:
            QMessageBox.warning(self, "目录为空", "请先选择数据保存根目录。")
            return
        try:
            directory = create_session_directory(
                Path(root_text), self.device.identifier
            )
        except Exception as exc:
            QMessageBox.critical(self, "无法创建数据集", str(exc))
            return
        self.dataset.setText(str(directory))
        target_yaml = directory / "target.yaml"
        self.target_yaml.setText(str(target_yaml))
        self.capture_progress.setValue(0)
        thread = _CaptureThread(
            self.device.identifier,
            directory,
            self.duration.value(),
        )
        thread.progressed.connect(self._capture_progressed)
        thread.previewed.connect(self._show_capture_preview)
        thread.succeeded.connect(self._capture_succeeded)
        thread.failed.connect(self._capture_failed)
        thread.finished.connect(self._capture_finished)
        self._capture_thread = thread
        self._capture_preview_image = None
        self.capture_preview.clear()
        self.capture_preview.setText("正在等待左右灰度视频帧…")
        self.capture_preview_info.setText(
            "正在连接 OAK · 预览和保存均使用同一条去畸变采集管线"
        )
        self._set_capture_running(True)
        self._append_log(
            f"开始采集：{directory}\n目标速率：双目各 20 Hz，IMU 200 Hz"
            "\n相机图像：按 EEPROM 完整 Perspective 参数实时去畸变后保存"
        )
        thread.start()

    def _start_imu_noise_capture(self) -> None:
        if self._is_busy():
            return
        root_text = self.output_root.text().strip()
        if not root_text:
            QMessageBox.warning(self, "目录为空", "请先选择数据保存根目录。")
            return
        try:
            directory = create_imu_noise_session_directory(
                Path(root_text), self.device.identifier
            )
        except Exception as exc:
            QMessageBox.critical(self, "无法创建静置数据集", str(exc))
            return
        duration_seconds = self.imu_noise_duration.value() * 60
        self.imu_noise_dataset.setText(str(directory))
        self.imu_noise_progress.setValue(0)
        thread = _ImuNoiseCaptureThread(
            self.device.identifier,
            directory,
            duration_seconds,
        )
        thread.progressed.connect(self._imu_noise_progressed)
        thread.succeeded.connect(self._imu_noise_succeeded)
        thread.failed.connect(self._imu_noise_failed)
        thread.finished.connect(self._imu_noise_finished)
        self._imu_noise_thread = thread
        self._set_imu_noise_running(True)
        self._append_log(
            f"开始 IMU 静置采集：{directory}\n"
            f"目标：{self.imu_noise_duration.value()} 分钟，200 Hz 原始 IMU；"
            "采集期间请勿移动、触碰设备或让计算机休眠。"
        )
        thread.start()

    def _stop_imu_noise_capture(self) -> None:
        if self._imu_noise_thread is not None:
            self.stop_imu_noise_button.setEnabled(False)
            self._append_log("正在停止 IMU 静置采集…")
            self._imu_noise_thread.stop()

    def _imu_noise_progressed(self, elapsed: float, imu_samples: int) -> None:
        duration_seconds = self.imu_noise_duration.value() * 60
        ratio = min(elapsed / duration_seconds, 1.0)
        self.imu_noise_progress.setValue(round(ratio * 1000))
        self.imu_noise_progress.setFormat(
            f"{elapsed / 60:.1f} / {duration_seconds / 60:.0f} 分钟 · "
            f"IMU {imu_samples} 条"
        )

    def _imu_noise_succeeded(self, result: ImuNoiseCaptureSummary) -> None:
        duration_seconds = self.imu_noise_duration.value() * 60
        self.imu_noise_progress.setValue(
            round(min(result.elapsed_seconds / duration_seconds, 1.0) * 1000)
        )
        state = "已提前停止" if result.stopped else "静置采集完成"
        self._append_log(
            f"{state}：IMU {result.imu_samples} 条，"
            f"平均 {result.average_rate_hz:.2f} Hz\n"
            f"原始数据：{result.csv_path}\n"
            "下一步：通过 Allan 方差或 imu_utils 计算四项噪声参数并生成 imu.yaml。"
        )
        if not 180.0 <= result.average_rate_hz <= 220.0:
            QMessageBox.warning(
                self,
                "IMU 采样率异常",
                f"本次平均采样率为 {result.average_rate_hz:.2f} Hz，"
                "与目标 200 Hz 偏差较大。建议检查 USB 连接、系统负载和休眠设置后重采。",
            )

    def _imu_noise_failed(self, message: str) -> None:
        self._append_log(f"IMU 静置采集失败：{message}")
        QMessageBox.critical(self, "IMU 静置采集失败", message)

    def _imu_noise_finished(self) -> None:
        self._imu_noise_thread = None
        self._set_imu_noise_running(False)

    def _set_imu_noise_running(self, running: bool) -> None:
        self.imu_noise_capture_button.setEnabled(not running)
        self.stop_imu_noise_button.setEnabled(running)
        self.imu_noise_duration.setEnabled(not running)
        self.output_root.setEnabled(not running)
        self.capture_button.setEnabled(not running)
        self.run_button.setEnabled(not running)
        self.flash_button.setEnabled(not running)

    def _stop_capture(self) -> None:
        if self._capture_thread is not None:
            self.stop_capture_button.setEnabled(False)
            self._append_log("正在停止采集…")
            self._capture_thread.stop()

    def _capture_progressed(
        self,
        elapsed: float,
        left_frames: int,
        right_frames: int,
        imu_samples: int,
    ) -> None:
        ratio = min(elapsed / self.duration.value(), 1.0)
        self.capture_progress.setValue(round(ratio * 1000))
        self.capture_progress.setFormat(
            f"{elapsed:.1f} 秒 · 左 {left_frames} · 右 {right_frames} · IMU {imu_samples}"
        )

    def _show_capture_preview(self, image: QImage, status: str) -> None:
        self._capture_preview_image = image
        self.capture_preview_info.setText(status)
        pixmap = QPixmap.fromImage(image).scaled(
            self.capture_preview.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.capture_preview.setPixmap(pixmap)

    def _capture_succeeded(self, result: CaptureSummary) -> None:
        self.capture_progress.setValue(
            round(min(result.elapsed_seconds / self.duration.value(), 1.0) * 1000)
        )
        state = "已提前停止" if result.stopped else "采集完成"
        unpaired = ""
        if result.unpaired_left_frames or result.unpaired_right_frames:
            unpaired = (
                "\n已按设备时间戳忽略未配对图像："
                f"左 {result.unpaired_left_frames}、右 {result.unpaired_right_frames}"
            )
        self._append_log(
            f"{state}：左 {result.left_frames} 帧，右 {result.right_frames} 帧，"
            f"IMU {result.imu_samples} 条{unpaired}\n数据集：{result.directory}\n"
            f"默认 AprilGrid 配置：{result.directory / 'target.yaml'}\n"
            "如实物标定板不是默认 6x6 tag36H11，请在第 3 步手动替换 YAML。"
        )

    def _capture_failed(self, message: str) -> None:
        self._append_log(f"采集失败：{message}")
        QMessageBox.critical(self, "采集失败", message)

    def _capture_finished(self) -> None:
        self._capture_thread = None
        self._set_capture_running(False)

    def _set_capture_running(self, running: bool) -> None:
        self.capture_button.setEnabled(not running)
        self.stop_capture_button.setEnabled(running)
        self.imu_noise_capture_button.setEnabled(not running)
        self.output_root.setEnabled(not running)
        self.duration.setEnabled(not running)

    def _run_kalibr(self) -> None:
        if self._is_busy():
            return
        try:
            dataset = self._validated_dataset()
            target = self._existing_file(self.target_yaml, "AprilGrid target.yaml")
            imu = self._imu_config(dataset)
            inspection = validate_kalibr_dataset(dataset, target, imu)
        except ValueError as exc:
            QMessageBox.warning(self, "配置不完整", str(exc))
            return
        self._append_log(inspection)
        commands = kalibr_commands(dataset, target, imu)
        configured = self.kalibr_setup.text().strip()
        setup = Path(configured).expanduser() if configured else None
        self._kalibr_environment = detect_kalibr_environment(setup)
        if not self._kalibr_environment.ready:
            missing = (
                *self._kalibr_environment.missing,
                *self._kalibr_environment.runtime_missing,
            )
            rendered = "\n\n".join(_render_command(command) for command in commands)
            self._append_log(
                "当前 Kalibr 环境缺少："
                + ", ".join(missing)
                + "\n请在已安装 Kalibr 的 Linux/ROS 终端运行：\n"
                + rendered
            )
            QMessageBox.information(
                self,
                "需要 Kalibr 环境",
                "数据集已经可以使用，但当前系统未安装 Kalibr。"
                "完整命令已显示在日志中。",
            )
            return
        self._commands = list(commands)
        self._command_index = 0
        self._start_next_process()

    def _imu_config(self, dataset: Path) -> Path:
        configured = self.imu_yaml.text().strip()
        if configured:
            return self._existing_file(self.imu_yaml, "IMU imu.yaml")
        target = dataset / "imu-default.yaml"
        try:
            target.write_bytes(self._default_imu_yaml.read_bytes())
        except OSError as exc:
            raise ValueError(f"无法生成默认 IMU 噪声配置：{exc}") from exc
        self._append_log(
            f"IMU 噪声配置为空：复制默认配置 {self._default_imu_yaml} 到数据集。"
        )
        return target

    def _start_next_process(self) -> None:
        if self._command_index >= len(self._commands):
            self._process = None
            self.run_button.setEnabled(True)
            self.stop_process_button.setEnabled(False)
            self._append_log("Kalibr 求解完成。")
            self._select_generated_result()
            return
        command = self._commands[self._command_index]
        self._append_log(f"运行：{_render_command(command)}")
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        process.setWorkingDirectory(self.dataset.text().strip())
        process.readyReadStandardOutput.connect(self._read_process_output)
        process.finished.connect(self._process_finished)
        process.errorOccurred.connect(self._process_error)
        self._process = process
        self.run_button.setEnabled(False)
        self.stop_process_button.setEnabled(True)
        program, arguments = kalibr_process_spec(command, self._kalibr_environment)
        process.start(program, arguments)

    def _read_process_output(self) -> None:
        if self._process is None:
            return
        output = bytes(self._process.readAllStandardOutput()).decode(
            "utf-8", errors="replace"
        )
        if output:
            self._append_log(output.rstrip())

    def _process_finished(self, exit_code: int, _status: Any) -> None:
        self._read_process_output()
        if exit_code != 0:
            self._append_log(f"Kalibr 命令失败，退出码 {exit_code}")
            self._process = None
            self.run_button.setEnabled(True)
            self.stop_process_button.setEnabled(False)
            return
        self._command_index += 1
        self._process = None
        self._start_next_process()

    def _process_error(self, error: Any) -> None:
        if self._process is not None:
            self._append_log(f"Kalibr 进程错误：{self._process.errorString()} ({error})")
        if error == QProcess.ProcessError.FailedToStart:
            self._process = None
            self.run_button.setEnabled(True)
            self.stop_process_button.setEnabled(False)

    def _stop_process(self) -> None:
        if self._process is not None:
            self._append_log("正在停止 Kalibr…")
            self._process.kill()

    def _select_generated_result(self) -> None:
        dataset = Path(self.dataset.text().strip())
        candidates = sorted(
            dataset.glob("*camchain-imucam*.yaml"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        if candidates:
            self.result_yaml.setText(str(candidates[0]))
            self._check_result()
        else:
            self._append_log("未在数据集目录找到 *-camchain-imucam.yaml，请手动选择结果文件。")

    def _check_result(self) -> None:
        path = Path(self.result_yaml.text().strip())
        try:
            result = load_kalibr_result(path)
        except Exception as exc:
            self._append_log(f"结果检查失败：{exc}")
            QMessageBox.warning(self, "结果无效", str(exc))
            return
        tx, ty, tz = result.translation_m
        shift = (
            "未提供"
            if result.timeshift_seconds is None
            else f"{result.timeshift_seconds:.9g} s"
        )
        matrix = "\n".join(
            "  [" + ", ".join(f"{value:.9g}" for value in row) + "]"
            for row in result.matrix_m
        )
        self._append_log(
            "结果有效：cam0.T_cam_imu（IMU → 左目 CAM_B，平移单位 m）\n"
            f"{matrix}\n"
            f"det={result.determinant:.9g}，正交误差={result.orthogonal_error:.3g}\n"
            f"平移=({tx:.6g}, {ty:.6g}, {tz:.6g}) m，时间偏移={shift}\n"
            "注：DepthAI EEPROM 写入外参矩阵；Kalibr 的时间偏移不会写入 EEPROM。"
        )

    def _flash_result(self) -> None:
        if self._is_busy():
            return
        try:
            result = load_kalibr_result(Path(self.result_yaml.text().strip()))
        except Exception as exc:
            QMessageBox.warning(self, "结果无效", str(exc))
            return
        tx, ty, tz = result.translation_m
        backend = str(self.flash_backend.currentData())
        api_name = "flashCalibration2" if backend == "v2" else "flashCalibration"
        answer = QMessageBox.warning(
            self,
            "确认写入 OAK EEPROM",
            "即将把 cam0.T_cam_imu 作为 IMU → 左目 CAM_B 外参写入当前相机。\n\n"
            f"MXID：{self.device.identifier}\n"
            f"平移：({tx:.6g}, {ty:.6g}, {tz:.6g}) m\n"
            f"det：{result.determinant:.6f}\n\n"
            f"写入方式：DepthAI {backend.upper()} · {api_name}()\n\n"
            "写入前会自动备份原 EEPROM JSON，写入后会立即回读核对。是否继续？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        backup_directory = self._backup_directory(result.source)
        thread = _ActionThread(
            lambda: flash_kalibr_result(
                self.device.identifier,
                result.source,
                backup_directory,
                backend,
            )
        )
        thread.succeeded.connect(self._flash_succeeded)
        thread.failed.connect(self._flash_failed)
        thread.finished.connect(self._flash_finished)
        self._flash_thread = thread
        self.flash_button.setEnabled(False)
        self.flash_backend.setEnabled(False)
        self._append_log(
            f"正在使用 DepthAI {backend.upper()} / {api_name}() 备份并写入 EEPROM，"
            "请勿断开 USB 或关闭程序…"
        )
        thread.start()

    def _flash_succeeded(self, summary: Any) -> None:
        self._append_log(
            f"EEPROM 写入并回读验证成功：DepthAI {summary.depthai_version} · "
            f"{summary.flash_api}() · 最大误差 {summary.maximum_error:.3g}\n"
            f"原 EEPROM 备份：{summary.backup}\n"
            f"实际写入 JSON：{summary.written_calibration}\n"
            f"完整标定报告：{summary.final_report}"
        )
        QMessageBox.information(
            self,
            "写入成功",
            "IMU → 左目外参已写入并回读一致。\n"
            f"原 EEPROM 备份：\n{summary.backup}\n\n"
            f"实际写入 JSON：\n{summary.written_calibration}\n\n"
            f"完整标定报告：\n{summary.final_report}",
        )

    def _flash_failed(self, message: str) -> None:
        self._append_log(f"EEPROM 写入失败：{message}")
        QMessageBox.critical(self, "写入失败", message)

    def _flash_finished(self) -> None:
        self._flash_thread = None
        self.flash_button.setEnabled(True)
        self.flash_backend.setEnabled(True)

    def _validated_dataset(self) -> Path:
        dataset_text = self.dataset.text().strip()
        if not dataset_text:
            raise ValueError(
                "尚未选择 Kalibr 数据集。请在第 1 节“数据集目录”或第 3 节"
                "点击“选择已有数据集…”，选择包含 cam0、cam1、imu0.csv、camchain.yaml 的目录。"
            )
        dataset = Path(dataset_text).expanduser()
        required = (
            dataset / "cam0",
            dataset / "cam1",
            dataset / "imu0.csv",
            dataset / "camchain.yaml",
        )
        missing = [path.name for path in required if not path.exists()]
        if missing:
            raise ValueError(
                "数据集目录不完整，缺少："
                + ", ".join(missing)
                + "。请确认选择的是 Kalibr 数据集根目录，而不是 cam0 子目录。"
            )
        return dataset

    @staticmethod
    def _existing_file(field: QLineEdit, name: str) -> Path:
        path = Path(field.text().strip()).expanduser()
        if not path.is_file():
            raise ValueError(f"请选择有效的 {name}")
        return path

    def _backup_directory(self, result_path: Path) -> Path:
        dataset_text = self.dataset.text().strip()
        return Path(dataset_text) if dataset_text else result_path.parent

    def _is_busy(self) -> bool:
        return (
            self._capture_thread is not None
            or self._imu_noise_thread is not None
            or self._flash_thread is not None
            or self._process is not None
        )

    def _append_log(self, message: str) -> None:
        self.log.appendPlainText(message)

    def closeEvent(self, event: QCloseEvent) -> None:
        if self._capture_thread is not None:
            self._capture_thread.stop()
            if not self._capture_thread.wait(5000):
                QMessageBox.information(self, "仍在停止", "相机仍在关闭，请稍后再试。")
                event.ignore()
                return
        if self._imu_noise_thread is not None:
            self._imu_noise_thread.stop()
            if not self._imu_noise_thread.wait(5000):
                QMessageBox.information(
                    self,
                    "仍在停止",
                    "IMU 静置采集仍在关闭，请稍后再试。",
                )
                event.ignore()
                return
        if self._process is not None:
            self._process.kill()
            if not self._process.waitForFinished(3000):
                event.ignore()
                return
        if self._flash_thread is not None and self._flash_thread.isRunning():
            QMessageBox.warning(self, "正在写入", "EEPROM 写入尚未完成，当前不能关闭窗口。")
            event.ignore()
            return
        super().closeEvent(event)


def _render_command(command: tuple[str, ...]) -> str:
    return " ".join(shlex.quote(part) for part in command)
