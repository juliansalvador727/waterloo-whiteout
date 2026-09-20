"""Detector interface, a safe placeholder, and a YOLOv8 vessel detector."""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Mapping, Protocol

from .camera import CameraFrame
from .models import BoundingBox, Detection, Pixel
from .redhull import red_fraction, red_hull_box


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

    ``weights_by_camera`` and ``conf_by_camera`` override those defaults for
    named cameras, so a tower-view model can serve the towers while the wide
    aerial model serves the aircraft. ``detect`` is safe to call from several
    camera threads.
    """

    def __init__(
        self,
        weights: str | Path = "best.pt",
        fallback_weights: str | Path = "yolov8s.pt",
        *,
        imgsz: int = 1280,
        conf: float = 0.25,
        weights_by_camera: Mapping[str, str | Path] | None = None,
        conf_by_camera: Mapping[str, float] | None = None,
        red_rescue: bool = False,
        red_rescue_confidence: float = 0.15,
        min_red_fraction: float = 0.0,
    ) -> None:
        self.fallback_weights = str(fallback_weights)
        self.imgsz = imgsz
        self.conf = conf
        self.conf_by_camera = dict(conf_by_camera or {})
        self.red_rescue = red_rescue
        self.red_rescue_confidence = red_rescue_confidence
        self.min_red_fraction = min_red_fraction
        self._lock = threading.Lock()
        self._models: dict[str, tuple[object, int]] = {}
        self.weights = self._resolve(weights)
        self._load(self.weights)
        self.boat_class_id = self._models[self.weights][1]
        self.weights_by_camera = {
            camera: self._resolve(path) for camera, path in (weights_by_camera or {}).items()
        }
        for path in self.weights_by_camera.values():
            self._load(path)

    def _resolve(self, weights: str | Path) -> str:
        return str(weights) if Path(weights).exists() else self.fallback_weights

    def _load(self, weights: str) -> None:
        if weights in self._models:
            return
        try:
            from ultralytics import YOLO  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("YOLO detection requires the ultralytics package") from exc
        model = YOLO(weights)
        boat_ids = [index for index, name in model.names.items() if name == "boat"]
        if not boat_ids:
            raise ValueError(f"model {weights!r} has no 'boat' class")
        self._models[weights] = (model, boat_ids[0])

    def weights_for(self, camera: str) -> str:
        return self.weights_by_camera.get(camera, self.weights)

    def detect(self, frame: CameraFrame) -> list[Detection]:
        image = frame.decode_bgr()
        weights = self.weights_for(frame.camera)
        model, boat_class_id = self._models[weights]
        with self._lock:
            result = model.predict(
                image,
                imgsz=self.imgsz,
                conf=self.conf_by_camera.get(frame.camera, self.conf),
                classes=[boat_class_id],
                verbose=False,
            )[0]
        detections = []
        for (x1, y1, x2, y2), confidence in zip(
            result.boxes.xyxy.tolist(), result.boxes.conf.tolist(), strict=True
        ):
            box = (x1, y1, x2, y2)
            red = red_fraction(image, box)
            if red < self.min_red_fraction:
                continue  # bright ice rather than a red hull
            detections.append(self._detection(frame, box, float(confidence), weights, red, "model"))

        if not detections and self.red_rescue:
            # The model misses vessels only a few pixels wide; their red paint survives.
            hull = red_hull_box(image)
            if hull is not None:
                detections.append(
                    self._detection(
                        frame, hull, self.red_rescue_confidence, weights, red_fraction(image, hull), "red-hull"
                    )
                )
        return sorted(detections, key=lambda d: d.confidence, reverse=True)

    def _detection(
        self,
        frame: CameraFrame,
        box: tuple[float, float, float, float],
        confidence: float,
        weights: str,
        red: float,
        source: str,
    ) -> Detection:
        x1, y1, x2, y2 = box
        return Detection(
            camera=frame.camera,
            pixel=Pixel((x1 + x2) / 2, (y1 + y2) / 2),
            confidence=confidence,
            timestamp=frame.timestamp,
            label="boat",
            metadata={
                "bbox": box,
                "weights": weights,
                "red_fraction": red,
                "source": source,
                "waterline": ((x1 + x2) / 2, y2),
            },
            bbox=BoundingBox(x1, y1, x2, y2),
        )

