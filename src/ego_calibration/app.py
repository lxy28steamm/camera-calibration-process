from __future__ import annotations

import json
import math
import sys
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

from PySide6.QtCore import QObject, QRunnable, Qt, QThreadPool, QTimer, Signal, Slot
from PySide6.QtGui import QColor, QFontDatabase
from PySide6.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QComboBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QSplitter,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ego_calibration import ego_lite, ego_std
from ego_calibration.imu_calibration import CameraImuCalibrationDialog
from ego_calibration.models import CalibrationError, CameraDevice
from ego_calibration.preview import CameraPreviewDialog, read_uvc_resolution
from ego_calibration.validation import ValidationReport, validate_calibration


_STYLE = """
QMainWindow, QWidget#root { background: #F3F6FA; color: #172B4D; }
QFrame#header { background: #172B4D; border: 0; }
QLabel#headerTitle { color: #FFFFFF; font-size: 21px; font-weight: 700; }
QLabel#headerSubtitle { color: #C9D5E6; font-size: 13px; }
QLabel#workflow { background: #203A60; color: #E7EEF8; padding: 7px 10px; }
QFrame#card { background: #FFFFFF; border: 0; }
QLabel#sectionTitle { color: #172B4D; font-size: 16px; font-weight: 700; }
QLabel#fieldLabel { color: #344563; font-weight: 600; }
QLabel#hint { color: #5E6C84; }
QLabel#badge { background: #EFF6FF; color: #1D4ED8; padding: 6px 11px; font-weight: 700; }
QLabel#resultBadge { background: #ECFDF3; color: #15803D; padding: 6px 11px; font-weight: 700; }
QLabel#warningBadge { background: #FFFBEB; color: #B45309; padding: 6px 11px; font-weight: 700; }
QLabel#errorBadge { background: #FEF2F2; color: #B91C1C; padding: 6px 11px; font-weight: 700; }
QComboBox { background: #F8FAFC; border: 1px solid #D8E0EA; padding: 8px; min-height: 20px; }
QComboBox:focus { background: #FFFFFF; border: 1px solid #2563EB; }
QPushButton { border: 0; padding: 9px 15px; font-weight: 700; min-height: 20px; }
QPushButton#primaryButton { background: #2563EB; color: #FFFFFF; }
QPushButton#primaryButton:hover { background: #1D4ED8; }
QPushButton#primaryButton:disabled { background: #AFC6F5; color: #F7F9FC; }
QPushButton#secondaryButton { background: #E9EEF5; color: #253858; }
QPushButton#secondaryButton:hover { background: #DDE5EF; }
QPushButton#secondaryButton:disabled { background: #EEF1F5; color: #A5ADBA; }
QTableWidget, QTreeWidget, QPlainTextEdit {
  background: #F8FAFC; color: #344563; border: 1px solid #D8E0EA;
  selection-background-color: #DBEAFE; selection-color: #1D4ED8;
}
QHeaderView::section { background: #EEF3F8; color: #344563; border: 0; padding: 8px; font-weight: 700; }
QTabWidget::pane { border: 1px solid #D8E0EA; background: #FFFFFF; }
QTabBar::tab { background: #E9EEF5; color: #344563; padding: 9px 18px; }
QTabBar::tab:selected { background: #FFFFFF; color: #1D4ED8; font-weight: 700; }
QProgressBar { background: #E7ECF3; border: 0; max-height: 7px; }
QProgressBar::chunk { background: #2563EB; }
QSplitter::handle { background: #D8E0EA; height: 5px; }
QSplitter::handle:hover { background: #93B4E8; }
"""


@dataclass(frozen=True, slots=True)
class _ScanResult:
    devices: tuple[CameraDevice, ...]
    warnings: tuple[str, ...] = ()


class _TaskSignals(QObject):
    succeeded = Signal(object)
    failed = Signal(str)
    finished = Signal()


class _Task(QRunnable):
    def __init__(self, function: Callable[[], Any]) -> None:
        super().__init__()
        self.function = function
        self.signals = _TaskSignals()

    @Slot()
    def run(self) -> None:
        try:
            self.signals.succeeded.emit(self.function())
        except Exception as exc:
            self.signals.failed.emit(str(exc) or type(exc).__name__)
        finally:
            self.signals.finished.emit()


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Ego 标定读取工具")
        self.resize(1160, 900)
        self.setMinimumSize(960, 800)
        self.setStyleSheet(_STYLE)
        self._thread_pool = QThreadPool.globalInstance()
        self._active_tasks: set[_Task] = set()
        self._devices: tuple[CameraDevice, ...] = ()
        self._calibration: dict[str, Any] | None = None
        self._calibration_identifier = ""
        self._result_kind = ""
        self._validation_report: ValidationReport | None = None
        self._preview_dialog: CameraPreviewDialog | None = None
        self._imu_calibration_dialog: CameraImuCalibrationDialog | None = None
        self._busy = False

        self.camera_type = QComboBox()
        self.camera_type.addItem("自动识别", "auto")
        self.camera_type.addItem("Ego-Lite", "ego-lite")
        self.camera_type.addItem("Ego-Std", "ego-std")
        self.camera_type.currentIndexChanged.connect(self._camera_type_changed)

        self.device = QComboBox()
        self.device.setMinimumWidth(420)
        self.device.currentIndexChanged.connect(self._device_changed)
        self.device.editTextChanged.connect(self._update_read_button)

        self.refresh_button = QPushButton("扫描相机")
        self.refresh_button.setObjectName("secondaryButton")
        self.refresh_button.clicked.connect(self.refresh_devices)
        self.preview_button = QPushButton("实时画面")
        self.preview_button.setObjectName("secondaryButton")
        self.preview_button.setEnabled(False)
        self.preview_button.clicked.connect(self.open_preview)
        self.imu_calibration_button = QPushButton("相机-IMU 标定")
        self.imu_calibration_button.setObjectName("secondaryButton")
        self.imu_calibration_button.setEnabled(False)
        self.imu_calibration_button.setToolTip(
            "采集 OAK 左/右灰度双目和原始 IMU，支持两小时 IMU 静置采集、"
            "Kalibr 求解及 EEPROM 写入"
        )
        self.imu_calibration_button.clicked.connect(self.open_imu_calibration)
        self.read_button = QPushButton("读取标定")
        self.read_button.setObjectName("primaryButton")
        self.read_button.setDefault(True)
        self.read_button.clicked.connect(self.read_calibration)

        self.scan_badge = QLabel("等待扫描")
        self.scan_badge.setObjectName("badge")
        self.protocol = QLabel()
        self.protocol.setObjectName("hint")
        self.protocol.setWordWrap(True)

        self.device_table = QTableWidget(0, 5)
        self.device_table.setHorizontalHeaderLabels(
            ["状态", "识别型号", "序列号 / MXID", "连接方式", "设备路径"]
        )
        self.device_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.device_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.device_table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.device_table.verticalHeader().hide()
        header = self.device_table.horizontalHeader()
        for column in range(4):
            header.setSectionResizeMode(column, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.ResizeMode.Stretch)
        self.device_table.setMinimumHeight(100)
        self.device_table.setMaximumHeight(130)
        self.device_table.cellClicked.connect(self._table_device_selected)

        self.summary = QTreeWidget()
        self.summary.setHeaderLabels(["字段", "值"])
        self.summary.setAlternatingRowColors(True)
        self.summary.header().setStretchLastSection(True)

        self.json_view = QPlainTextEdit()
        self.json_view.setReadOnly(True)
        self.json_view.setPlaceholderText("读取成功后将在这里显示完整 JSON")
        self.json_view.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))

        self.validation_view = QTreeWidget()
        self.validation_view.setHeaderLabels(["类别", "检查项", "结果", "详情"])
        self.validation_view.setAlternatingRowColors(True)
        self.validation_view.setRootIsDecorated(False)
        validation_header = self.validation_view.header()
        validation_header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        validation_header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        validation_header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        validation_header.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)

        self.result_badge = QLabel("尚未读取")
        self.result_badge.setObjectName("badge")
        self.status = QLabel("请选择相机类型并扫描设备")
        self.status.setObjectName("hint")
        self.progress = QProgressBar()
        self.progress.setRange(0, 0)
        self.progress.setMaximumWidth(140)
        self.progress.hide()
        self.validate_button = QPushButton("检验标定")
        self.validate_button.setObjectName("secondaryButton")
        self.validate_button.setEnabled(False)
        self.validate_button.setToolTip("检查信息完整性和外参旋转矩阵，展示设备保存的二维重投影 RMS")
        self.validate_button.clicked.connect(lambda: self._validate_current(True))
        self.copy_button = QPushButton("复制 JSON")
        self.copy_button.setObjectName("secondaryButton")
        self.copy_button.setEnabled(False)
        self.copy_button.clicked.connect(self._copy_json)
        self.save_button = QPushButton("保存 JSON")
        self.save_button.setObjectName("primaryButton")
        self.save_button.setEnabled(False)
        self.save_button.clicked.connect(self._save_json)

        root = QWidget()
        root.setObjectName("root")
        layout = QVBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self._create_header())
        content = QVBoxLayout()
        content.setContentsMargins(18, 14, 18, 16)
        content.setSpacing(10)
        self.main_splitter = QSplitter(Qt.Orientation.Vertical)
        self.main_splitter.setChildrenCollapsible(False)
        self.main_splitter.addWidget(self._create_device_card())
        self.main_splitter.addWidget(self._create_result_card())
        self.main_splitter.setStretchFactor(0, 0)
        self.main_splitter.setStretchFactor(1, 1)
        self.main_splitter.setSizes([300, 520])
        content.addWidget(self.main_splitter, 1)
        layout.addLayout(content, 1)
        self.setCentralWidget(root)

        self._camera_type_changed()
        QTimer.singleShot(0, self.refresh_devices)

    def _create_header(self) -> QFrame:
        header = QFrame()
        header.setObjectName("header")
        layout = QVBoxLayout(header)
        layout.setContentsMargins(24, 16, 24, 14)
        layout.setSpacing(6)
        title = QLabel("Ego 相机标定读取工具")
        title.setObjectName("headerTitle")
        subtitle = QLabel("自动识别 Ego-Lite / Ego-Std，并从硬件协议读取原生标定 JSON")
        subtitle.setObjectName("headerSubtitle")
        workflow = QLabel(
            "1  选择型号  →  2  扫描相机  →  3  预览 / 读取校验 / 相机-IMU 标定  →  4  保存"
        )
        workflow.setObjectName("workflow")
        workflow.setMaximumWidth(660)
        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addWidget(workflow)
        return header

    def _create_device_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(10)

        heading = QHBoxLayout()
        title = QLabel("相机扫描与识别")
        title.setObjectName("sectionTitle")
        heading.addWidget(title)
        heading.addStretch()
        heading.addWidget(self.scan_badge)
        layout.addLayout(heading)

        fields = QGridLayout()
        fields.setHorizontalSpacing(12)
        fields.setVerticalSpacing(8)
        type_label = QLabel("相机型号")
        type_label.setObjectName("fieldLabel")
        device_label = QLabel("识别设备")
        device_label.setObjectName("fieldLabel")
        fields.addWidget(type_label, 0, 0)
        fields.addWidget(self.camera_type, 0, 1)
        fields.addWidget(device_label, 1, 0)
        fields.addWidget(self.device, 1, 1)
        actions = QHBoxLayout()
        actions.addWidget(self.refresh_button)
        actions.addWidget(self.preview_button)
        actions.addWidget(self.imu_calibration_button)
        actions.addWidget(self.read_button)
        fields.addLayout(actions, 1, 2)
        fields.setColumnStretch(1, 1)
        layout.addLayout(fields)
        layout.addWidget(self.protocol)
        layout.addWidget(self.device_table)
        return card

    def _create_result_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        card.setMinimumHeight(380)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(16, 14, 16, 12)
        layout.setSpacing(9)

        heading = QHBoxLayout()
        title = QLabel("标定结果")
        title.setObjectName("sectionTitle")
        heading.addWidget(title)
        heading.addStretch()
        heading.addWidget(self.result_badge)
        layout.addLayout(heading)

        self.result_tabs = QTabWidget()
        self.result_tabs.addTab(self.summary, "摘要")
        self.result_tabs.addTab(self.json_view, "完整 JSON")
        self.result_tabs.addTab(self.validation_view, "标定检验")
        layout.addWidget(self.result_tabs, 1)

        footer = QHBoxLayout()
        footer.addWidget(self.status, 1)
        footer.addWidget(self.progress)
        footer.addWidget(self.validate_button)
        footer.addWidget(self.copy_button)
        footer.addWidget(self.save_button)
        layout.addLayout(footer)
        return card

    def _camera_type_changed(self) -> None:
        kind = self._camera_kind()
        self.device.setEditable(kind == "ego-std")
        protocol = {
            "auto": "同时扫描 DepthAI 与 ZXCZ/YCTC UVC 设备，并按协议、型号和序列号自动分类。",
            "ego-lite": "Luxonis DepthAI 2.32：通过 readCalibration2() 读取设备 EEPROM。",
            "ego-std": (
                "UVC XU V5 Unit 10：兼容 OpenCV schema v1 / Kalibr YAML schema v2，"
                "校验 CRC16/CRC32 并附加默认 IMU 标定。"
            ),
        }
        self.protocol.setText(protocol[kind])
        self.scan_badge.setText("等待扫描")
        self._devices = ()
        self.device.clear()
        self.device_table.setRowCount(0)
        self._clear_result()
        self._update_read_button()
        if self.isVisible():
            self.refresh_devices()

    def refresh_devices(self) -> None:
        if self._busy:
            return
        self._start_task(
            "正在扫描并识别相机…",
            self._scan_selected_devices,
            self._devices_loaded,
        )

    def _scan_selected_devices(self) -> _ScanResult:
        kind = self._camera_kind()
        if kind == "ego-lite":
            return _ScanResult(ego_lite.scan_devices())
        if kind == "ego-std":
            return _ScanResult(ego_std.scan_devices())

        devices: list[CameraDevice] = []
        warnings: list[str] = []
        for label, scanner in (
            ("Ego-Lite", ego_lite.scan_devices),
            ("Ego-Std", ego_std.scan_devices),
        ):
            try:
                devices.extend(scanner())
            except Exception as exc:
                warnings.append(f"{label}: {exc}")
        if not devices and warnings:
            raise CalibrationError("；".join(warnings))
        return _ScanResult(tuple(devices), tuple(warnings))

    def read_calibration(self) -> None:
        if self._busy:
            return
        selected = self._selected_device()
        if selected is None:
            QMessageBox.information(self, "未选择设备", "请先扫描并选择相机。")
            return
        if not selected.accessible:
            QMessageBox.warning(
                self,
                "设备权限不足",
                f"当前用户无法读写：{selected.path}\n请调整设备权限后重新扫描。",
            )
            return
        self._start_task(
            "正在读取并校验硬件标定…",
            lambda: _read_device_calibration(selected),
            self._calibration_loaded,
        )

    def open_preview(self) -> None:
        if self._busy:
            return
        selected = self._selected_device()
        if selected is None:
            QMessageBox.information(self, "未选择设备", "请先扫描并选择相机。")
            return
        if not selected.accessible:
            QMessageBox.warning(
                self,
                "设备权限不足",
                f"当前用户无法打开：{selected.path}\n请调整设备权限后重新扫描。",
            )
            return
        if self._preview_dialog is not None:
            self._preview_dialog.show()
            self._preview_dialog.raise_()
            self._preview_dialog.activateWindow()
            return
        dialog = CameraPreviewDialog(selected, self)
        dialog.resolution_ready.connect(self._preview_resolution_ready)
        dialog.destroyed.connect(self._preview_closed)
        self._preview_dialog = dialog
        dialog.show()
        self.status.setText("实时画面已打开，可检查对焦和模糊情况")

    def _preview_closed(self, *_args: Any) -> None:
        self._preview_dialog = None

    def _preview_resolution_ready(
        self, identifier: str, width: int, height: int
    ) -> None:
        if self._calibration is None or identifier != self._calibration_identifier:
            return
        self._calibration["uvc_preview"] = {"resolution": [width, height]}
        self.json_view.setPlainText(
            json.dumps(self._calibration, ensure_ascii=False, indent=2) + "\n"
        )
        self._render_summary(self._calibration, self._result_kind)

    def open_imu_calibration(self) -> None:
        if self._busy:
            return
        selected = self._selected_device()
        if selected is None or selected.kind != "ego-lite":
            QMessageBox.information(
                self,
                "仅支持 Ego-Lite",
                "当前相机-IMU 标定流程使用 DepthAI 协议，仅适用于 Ego-Lite/OAK。",
            )
            return
        if self._preview_dialog is not None:
            QMessageBox.information(
                self,
                "请先关闭实时画面",
                "实时画面会占用 OAK 设备，请关闭后再开始标定。",
            )
            return
        if self._imu_calibration_dialog is not None:
            self._imu_calibration_dialog.show()
            self._imu_calibration_dialog.raise_()
            self._imu_calibration_dialog.activateWindow()
            return
        dialog = CameraImuCalibrationDialog(selected, self)
        dialog.destroyed.connect(self._imu_calibration_closed)
        self._imu_calibration_dialog = dialog
        dialog.show()
        self.status.setText("相机-IMU 标定窗口已打开")

    def _imu_calibration_closed(self, *_args: Any) -> None:
        self._imu_calibration_dialog = None

    def _start_task(
        self,
        message: str,
        function: Callable[[], Any],
        on_success: Callable[[Any], None],
    ) -> None:
        self._set_busy(True, message)
        task = _Task(function)
        self._active_tasks.add(task)
        task.signals.succeeded.connect(on_success)
        task.signals.failed.connect(self._task_failed)
        task.signals.finished.connect(lambda: self._task_finished(task))
        self._thread_pool.start(task)

    def _task_finished(self, task: _Task) -> None:
        self._active_tasks.discard(task)
        self._set_busy(False)

    def _devices_loaded(self, result: _ScanResult) -> None:
        previous = self._selected_identifier()
        self._devices = result.devices
        self.device.clear()
        self.device_table.setRowCount(len(result.devices))
        for row, item in enumerate(result.devices):
            self.device.addItem(item.label, item)
            values = (
                "可读取" if item.accessible else "权限受限",
                item.model or _kind_label(item.kind),
                item.serial or item.identifier,
                item.transport,
                item.path or item.identifier,
            )
            for column, value in enumerate(values):
                cell = QTableWidgetItem(str(value))
                if column == 0:
                    cell.setData(Qt.ItemDataRole.UserRole, item.identifier)
                    color = "#15803D" if item.accessible else "#B91C1C"
                    cell.setForeground(QColor(color))
                self.device_table.setItem(row, column, cell)
            self.device_table.setRowHeight(row, 34)

        if previous:
            index = next(
                (
                    index
                    for index, item in enumerate(result.devices)
                    if item.identifier == previous
                ),
                -1,
            )
            if index >= 0:
                self.device.setCurrentIndex(index)
        if result.devices and self.device.currentIndex() < 0:
            self.device.setCurrentIndex(0)
        self._device_changed()

        count = len(result.devices)
        badge = f"已识别 {count} 台 · 部分异常" if result.warnings else f"已识别 {count} 台"
        self.scan_badge.setText(badge)
        timestamp = datetime.now().strftime("%H:%M:%S")
        if count:
            self.status.setText(f"扫描完成 · {timestamp} · 请选择设备读取标定")
        elif result.warnings:
            self.status.setText(f"扫描完成但存在异常 · {result.warnings[0]}")
        else:
            self.status.setText(f"未发现匹配相机 · {timestamp} · 请检查 USB 连接和供电")
        self._update_read_button()

    def _calibration_loaded(self, result: tuple[str, dict[str, Any]]) -> None:
        kind, payload = result
        self._result_kind = kind
        self._calibration = payload
        self._calibration_identifier = self._selected_identifier()
        rendered = json.dumps(payload, ensure_ascii=False, indent=2)
        self.json_view.setPlainText(rendered + "\n")
        self._render_summary(payload, kind)
        self.validate_button.setEnabled(True)
        self.copy_button.setEnabled(True)
        self.save_button.setEnabled(True)
        self._validate_current(False)

    def _validate_current(self, show_dialog: bool = True) -> None:
        if self._calibration is None:
            return
        report = validate_calibration(self._result_kind, self._calibration)
        self._validation_report = report
        self._render_validation(report)
        label = _kind_label(self._result_kind)
        if report.failure_count == 0:
            if report.skipped_count:
                self._set_badge(
                    self.result_badge, "warningBadge", f"{label} · 检验未完成"
                )
                message = (
                    f"基础检查通过 {report.passed_count} 项；"
                    f"{report.skipped_count} 项重投影误差无法评估，请查看“标定检验”页"
                )
                title = "标定检验未完成"
                self.result_tabs.setCurrentWidget(self.validation_view)
            else:
                self._set_badge(
                    self.result_badge, "resultBadge", f"{label} · 基础检验通过"
                )
                message = (
                    f"基础检查通过 {report.passed_count} 项"
                    f"（含 {report.rotation_count} 个旋转矩阵）；"
                    f"{report.info_count} 项 RMS 仅展示，未判定误差是否合格"
                )
                title = "标定基础检验通过"
            self.status.setText(message)
            if show_dialog:
                self.result_tabs.setCurrentWidget(self.validation_view)
                QMessageBox.information(self, title, message)
            return

        self._set_badge(
            self.result_badge,
            "errorBadge",
            f"{label} · 检验失败 {report.failure_count} 项",
        )
        self.status.setText(
            f"发现 {report.failure_count} 项异常，请查看“标定检验”页"
        )
        self.result_tabs.setCurrentWidget(self.validation_view)
        if show_dialog:
            QMessageBox.warning(
                self,
                "标定检验失败",
                f"发现 {report.failure_count} 项异常，失败项已在检验列表中标红。",
            )

    def _render_validation(self, report: ValidationReport) -> None:
        self.validation_view.clear()
        order = {False: 0, None: 1, True: 2}
        for check in sorted(report.checks, key=lambda item: order[item.passed]):
            if check.passed is None:
                result, color = "无法评估", QColor("#B45309")
            elif check.informational:
                result, color = "仅展示", QColor("#1D4ED8")
            elif check.passed:
                result, color = "通过", QColor("#15803D")
            else:
                result, color = "失败", QColor("#B91C1C")
            item = QTreeWidgetItem(
                self.validation_view,
                [check.category, check.name, result, check.detail],
            )
            item.setForeground(2, color)
            if check.passed is not True:
                item.setForeground(1, color)
                item.setForeground(3, color)

    def _render_summary(self, payload: dict[str, Any], kind: str) -> None:
        self.summary.clear()
        if kind == "ego-lite":
            device = payload.get("device", {})
            rows = [
                ("型号", "Ego-Lite"),
                ("MXID", device.get("mxid", "")),
                ("DepthAI", payload.get("depthai_version", "")),
                ("USB 速度", device.get("usb_speed", "")),
                ("IMU", device.get("imu_type", "")),
                ("IMU 固件", device.get("imu_firmware", "")),
            ]
            for name, camera in payload.get("cameras", {}).items():
                resolution = camera.get("calibration_resolution", [])
                rows.append(
                    (
                        f"相机 {name}",
                        f"{camera.get('sensor', '')} · {_resolution_text(resolution)}",
                    )
                )
        else:
            header = payload.get("header", {})
            rows = [
                ("型号", "Ego-Std"),
                (
                    "USB 序列号",
                    payload.get("device_identity", {}).get("usb_serial_number", ""),
                ),
                ("标定/固件 SN", header.get("serial_number", "")),
                ("Blob Schema", header.get("schema_version", "")),
            ]
            identity = payload.get("device_identity", {})
            if (
                isinstance(identity, dict)
                and identity.get("usb_serial_number")
                and identity.get("calibration_serial_number")
            ):
                rows.append(
                    (
                        "序列号一致性",
                        (
                            "一致"
                            if identity.get("serials_match")
                            else "编号体系不同：协议未提供直接映射"
                        ),
                    )
                )
            if payload.get("format") == "kalibr_camchain_imucam":
                rows.extend(
                    [
                        ("标定格式", "Kalibr camchain-imucam YAML"),
                        ("YAML 大小", f"{payload.get('payload', {}).get('length', '')} bytes"),
                        ("二维重投影 RMS", "无法评估：当前读取的参数未提供 RMS"),
                    ]
                )
            else:
                metrics = payload.get("metrics", {})
                rows.extend(
                    [
                        ("标定格式", "OpenCV 双目标定"),
                        ("标定样本数", metrics.get("sample_count", "")),
                        ("双目基线", f"{metrics.get('baseline_mm', '')} mm"),
                        ("左目内参 RMS", _rms_text(metrics.get("left_calibrate_rms"))),
                        ("右目内参 RMS", _rms_text(metrics.get("right_calibrate_rms"))),
                        ("双目内外参联合 RMS", _rms_text(metrics.get("stereo_rms"))),
                    ]
                )
            calibration = payload.get("kalibr_calibration", {})
            left = calibration.get("cam0", {}).get("resolution")
            right = calibration.get("cam1", {}).get("resolution")
            calibration_resolution = "未提供（设备标定数据未记录）"
            if left is not None or right is not None:
                calibration_resolution = (
                    _resolution_text(left)
                    if left == right
                    else f"左 {_resolution_text(left)} / 右 {_resolution_text(right)}"
                )
            video = payload.get("uvc_preview", {}).get("resolution")
            video_text = _resolution_text(video) if video else "未测得（请打开实时画面）"
            mono_text = (
                f"{video[0] // 2}×{video[1]}（实测双目画面左右平分）"
                if video and video[0] % 2 == 0
                else "未测得"
            )
            rows.extend(
                [
                    ("单目标定分辨率", calibration_resolution),
                    ("单目画面分辨率", mono_text),
                    ("双目 UVC 输出", video_text),
                    ("检验数据来源", "设备实读标定（不使用 common_calibration）"),
                ]
            )
        for key, value in rows:
            QTreeWidgetItem(self.summary, [str(key), str(value)])
        self.summary.resizeColumnToContents(0)

    def _task_failed(self, message: str) -> None:
        self.status.setText("操作失败，请检查连接、权限和驱动")
        QMessageBox.critical(self, "操作失败", message)

    def _set_busy(self, busy: bool, message: str = "") -> None:
        self._busy = busy
        self.camera_type.setEnabled(not busy)
        self.device.setEnabled(not busy)
        self.refresh_button.setEnabled(not busy)
        self.preview_button.setEnabled(not busy and self._selected_device() is not None)
        selected = self._selected_device()
        self.imu_calibration_button.setEnabled(
            not busy and selected is not None and selected.kind == "ego-lite"
        )
        self.progress.setVisible(busy)
        if busy:
            self.read_button.setEnabled(False)
        else:
            self._update_read_button()
        if message:
            self.status.setText(message)

    def _camera_kind(self) -> str:
        return str(self.camera_type.currentData())

    def _selected_device(self) -> CameraDevice | None:
        selected = self.device.currentData()
        if isinstance(selected, CameraDevice):
            return selected
        identifier = self.device.currentText().strip()
        if identifier and self._camera_kind() == "ego-std":
            return CameraDevice(
                kind="ego-std",
                identifier=identifier,
                label=identifier,
                model="ZXCZ/YCTC Stereo UVC",
                transport="UVC XU V5",
                path=identifier,
            )
        return None

    def _selected_identifier(self) -> str:
        selected = self._selected_device()
        return selected.identifier if selected else ""

    def _device_changed(self) -> None:
        selected = self._selected_device()
        if selected is not None:
            for row in range(self.device_table.rowCount()):
                cell = self.device_table.item(row, 0)
                if cell and cell.data(Qt.ItemDataRole.UserRole) == selected.identifier:
                    self.device_table.selectRow(row)
                    break
        self._update_read_button()

    def _table_device_selected(self, row: int, _column: int) -> None:
        cell = self.device_table.item(row, 0)
        if cell is None:
            return
        identifier = str(cell.data(Qt.ItemDataRole.UserRole) or "")
        for index, item in enumerate(self._devices):
            if item.identifier == identifier:
                self.device.setCurrentIndex(index)
                return

    def _update_read_button(self, *_args: Any) -> None:
        enabled = not self._busy and self._selected_device() is not None
        self.read_button.setEnabled(enabled)
        self.preview_button.setEnabled(enabled)
        selected = self._selected_device()
        self.imu_calibration_button.setEnabled(
            enabled and selected is not None and selected.kind == "ego-lite"
        )

    def _clear_result(self) -> None:
        self._calibration = None
        self._calibration_identifier = ""
        self._result_kind = ""
        self._validation_report = None
        self.summary.clear()
        self.json_view.clear()
        self.validation_view.clear()
        self.validate_button.setEnabled(False)
        self.copy_button.setEnabled(False)
        self.save_button.setEnabled(False)
        self._set_badge(self.result_badge, "badge", "尚未读取")

    @staticmethod
    def _set_badge(widget: QLabel, name: str, text: str) -> None:
        widget.setObjectName(name)
        widget.setText(text)
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def _copy_json(self) -> None:
        QApplication.clipboard().setText(self.json_view.toPlainText())
        self.status.setText("JSON 已复制到剪贴板")

    def _save_json(self) -> None:
        if self._calibration is None:
            return
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        default_name = f"{self._result_kind}-calibration-{stamp}.json"
        filename, _filter = QFileDialog.getSaveFileName(
            self,
            "保存标定 JSON",
            str(Path.home() / default_name),
            "JSON 文件 (*.json)",
        )
        if not filename:
            return
        path = Path(filename)
        if path.suffix.lower() != ".json":
            path = path.with_suffix(".json")
        try:
            path.write_text(self.json_view.toPlainText(), encoding="utf-8")
        except OSError as exc:
            QMessageBox.critical(self, "保存失败", str(exc))
            return
        self.status.setText(f"已保存：{path}")


def _kind_label(kind: str) -> str:
    return {
        "ego-lite": "Ego-Lite", "ego-std": "Ego-Std", "ego-std-235": "Ego-Std-235"
    }.get(kind, kind)


def _rms_text(value: Any) -> str:
    if value is None:
        return "无法评估：未提供 RMS"
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(value)
        or value < 0
    ):
        return "RMS 数值无效，请查看标定检验"
    return f"{value:.6g} px（仅展示，未判定合格）"


def _resolution_text(value: Any) -> str:
    if isinstance(value, list) and len(value) == 2:
        return f"{value[0]}×{value[1]}"
    return "未知分辨率"


def _read_device_calibration(device: CameraDevice) -> tuple[str, dict[str, Any]]:
    if device.kind == "ego-lite":
        return device.kind, ego_lite.read_calibration(device.identifier)
    payload = ego_std.read_calibration(device.identifier)
    try:
        payload["uvc_preview"] = {"resolution": read_uvc_resolution(device.identifier)}
    except Exception as exc:
        # 画面不可用不影响设备标定数据的读取和校验。
        payload["uvc_preview"] = {"error": str(exc)}
    return device.kind, payload


def main() -> int:
    application = QApplication(sys.argv)
    application.setApplicationName("Ego Calibration")
    application.setOrganizationName("LivSyn")
    application.setStyle("Fusion")
    window = MainWindow()
    window.show()
    return application.exec()
