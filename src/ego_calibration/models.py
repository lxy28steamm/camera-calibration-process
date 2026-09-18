from __future__ import annotations

from dataclasses import dataclass


class CalibrationError(RuntimeError):
    """A device or protocol error safe to show in the UI."""


@dataclass(frozen=True, slots=True)
class CameraDevice:
    kind: str
    identifier: str
    label: str
    model: str = ""
    serial: str = ""
    transport: str = ""
    path: str = ""
    accessible: bool = True
    calibration_serial: str = ""
    camera_serial: str = ""
    serial_error: str = ""
