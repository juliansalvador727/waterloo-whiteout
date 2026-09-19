"""Detector interface, a safe placeholder, and a YOLOv8 vessel detector."""

from __future__ import annotations

from pathlib import Path
from typing import Protocol

from .camera import CameraFrame
from .models import Detection, Pixel


class Detector(Protocol):
    def detect(self, frame: CameraFrame) -> list[Detection]: ...


class NoOpDetector:
    """Returns no detections and performs no external actions."""

    def detect(self, frame: CameraFrame) -> list[Detection]:
        return []


class YoloVesselDetector:
    """YOLOv8 detector that reports only the boat class.

    Loads ``weights`` when present, otherwise ``fallback_weights``. The boat
    class id is resolved by name: 0 for the custom boat/ice model, 8 for COCO.
    """

    def __init__(
        self,
        weights: str | Path = "best.pt",
        fallback_weights: str | Path = "yolov8s.pt",
        *,
        imgsz: int = 1280,
        conf: float = 0.25,
    ) -> None:
        try:
            from ultralytics import YOLO  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("YOLO detection requires the ultralytics package") from exc
        self.weights = str(weights) if Path(weights).exists() else str(fallback_weights)
        self.model = YOLO(self.weights)
        self.imgsz = imgsz
        self.conf = conf
        boat_ids = [index for index, name in self.model.names.items() if name == "boat"]
        if not boat_ids:
            raise ValueError(f"model {self.weights!r} has no 'boat' class")
        self.boat_class_id = boat_ids[0]

    def detect(self, frame: CameraFrame) -> list[Detection]:
        image = frame.decode_bgr()
        result = self.model.predict(
            image,
            imgsz=self.imgsz,
            conf=self.conf,
            classes=[self.boat_class_id],
            verbose=False,
        )[0]
        detections = []
        for (x1, y1, x2, y2), confidence in zip(
            result.boxes.xyxy.tolist(), result.boxes.conf.tolist(), strict=True
        ):
            detections.append(
                Detection(
                    camera=frame.camera,
                    pixel=Pixel((x1 + x2) / 2, (y1 + y2) / 2),
                    confidence=float(confidence),
                    timestamp=frame.timestamp,
                    label="boat",
                    metadata={"bbox": (x1, y1, x2, y2)},
                )
            )
        return detections

