"""Detector interface and a safe placeholder implementation."""

from __future__ import annotations

from typing import Protocol

from .camera import CameraFrame
from .models import Detection


class Detector(Protocol):
    def detect(self, frame: CameraFrame) -> list[Detection]: ...


class NoOpDetector:
    """Returns no detections and performs no external actions."""

    def detect(self, frame: CameraFrame) -> list[Detection]:
        return []

