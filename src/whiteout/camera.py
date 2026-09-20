"""Resilient MJPEG ingestion without implicit network activity on import."""

from __future__ import annotations

import time
import urllib.request
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from datetime import datetime, timezone
import math

from .geolocation import CameraIntrinsics


@dataclass(frozen=True, slots=True)
class CameraModel:
    """Fixed camera geometry used by the simulator assets."""

    name: str
    width_px: int
    height_px: int
    horizontal_fov_deg: float
    vertical_fov_deg: float

    def __post_init__(self) -> None:
        if self.width_px <= 0 or self.height_px <= 0:
            raise ValueError("camera dimensions must be positive")
        if not 0 < self.horizontal_fov_deg < 180 or not 0 < self.vertical_fov_deg < 180:
            raise ValueError("camera fields of view must be between 0 and 180 degrees")

    @property
    def intrinsics(self) -> CameraIntrinsics:
        """Derive centred pinhole intrinsics from resolution and field of view."""
        cx = (self.width_px - 1) / 2.0
        cy = (self.height_px - 1) / 2.0
        fx = cx / math.tan(math.radians(self.horizontal_fov_deg) / 2.0)
        fy = cy / math.tan(math.radians(self.vertical_fov_deg) / 2.0)
        return CameraIntrinsics(fx, fy, cx, cy)

    @property
    def width(self) -> int:
        return self.width_px

    @property
    def height(self) -> int:
        return self.height_px

    @property
    def hfov_deg(self) -> float:
        return self.horizontal_fov_deg

    @property
    def vfov_deg(self) -> float:
        return self.vertical_fov_deg

    def derived_intrinsics(self) -> CameraIntrinsics:
        return self.intrinsics

    to_intrinsics = derived_intrinsics

    def footprint_m(self, altitude_m: float) -> tuple[float, float]:
        """Return nadir ground-footprint width and height at absolute altitude."""
        altitude = float(altitude_m)
        if altitude <= 0:
            raise ValueError("camera altitude above water must be positive")
        return (
            2.0 * altitude * math.tan(math.radians(self.horizontal_fov_deg) / 2.0),
            2.0 * altitude * math.tan(math.radians(self.vertical_fov_deg) / 2.0),
        )

    def spacing_m(self, altitude_m: float, overlap: float = 0.20) -> tuple[float, float]:
        """Return along/across-track spacing for the requested fractional overlap."""
        if not 0 <= overlap < 1:
            raise ValueError("overlap must be in the range [0, 1)")
        width, height = self.footprint_m(altitude_m)
        return width * (1.0 - overlap), height * (1.0 - overlap)

    def overlap_20_spacing_m(self, altitude_m: float) -> tuple[float, float]:
        return self.spacing_m(altitude_m, 0.20)

    def footprint_corners_m(
        self,
        altitude_m: float,
        *,
        center_north_m: float = 0.0,
        center_east_m: float = 0.0,
        heading_deg: float = 0.0,
    ) -> tuple[tuple[float, float], ...]:
        """Return a nadir footprint polygon as ``(north, east)`` corners."""
        width, height = self.footprint_m(altitude_m)
        heading = math.radians(heading_deg)
        cosine, sine = math.cos(heading), math.sin(heading)
        corners = []
        for forward, right in (
            (-height / 2.0, -width / 2.0),
            (-height / 2.0, width / 2.0),
            (height / 2.0, width / 2.0),
            (height / 2.0, -width / 2.0),
        ):
            corners.append(
                (
                    center_north_m + forward * cosine - right * sine,
                    center_east_m + forward * sine + right * cosine,
                )
            )
        return tuple(corners)

    ground_footprint_m = footprint_m
    overlap_spacing_m = spacing_m

    @classmethod
    def quad(cls) -> CameraModel:
        return QUAD_CAMERA

    quadcopter = quad

    @classmethod
    def fixed_wing(cls) -> CameraModel:
        return FIXED_WING_CAMERA

    @classmethod
    def tower(cls) -> CameraModel:
        return TOWER_CAMERA


QUAD_CAMERA = CameraModel("quadcopter", 960, 720, 114.6, 99.4)
FIXED_WING_CAMERA = CameraModel("fixed-wing", 1280, 720, 69.0, 42.6)
TOWER_CAMERA = CameraModel("tower", 1280, 720, 60.0, 36.1)
QUADCOPTER_CAMERA = QUAD_CAMERA
QUAD_CAMERA_MODEL = QUAD_CAMERA
FIXED_WING_CAMERA_MODEL = FIXED_WING_CAMERA
TOWER_CAMERA_MODEL = TOWER_CAMERA
CAMERA_MODELS = {
    "quadcopter": QUAD_CAMERA,
    "fixed-wing": FIXED_WING_CAMERA,
    "tower-1": TOWER_CAMERA,
    "tower-2": TOWER_CAMERA,
}


def camera_model(name: str) -> CameraModel:
    """Return a fixed model for a configured simulator camera name."""
    normalized = name.lower().replace("_", "-")
    if normalized in {"quad", "quadcopter"}:
        return QUAD_CAMERA
    if normalized in {"fixed-wing", "fixedwing", "plane"}:
        return FIXED_WING_CAMERA
    if normalized in {"tower", "tower-1", "tower-2"}:
        return TOWER_CAMERA
    raise ValueError(f"unknown camera model {name!r}")


def footprint_m(model: CameraModel, altitude_m: float) -> tuple[float, float]:
    return model.footprint_m(altitude_m)


def overlap_20_spacing_m(model: CameraModel, altitude_m: float) -> tuple[float, float]:
    return model.overlap_20_spacing_m(altitude_m)


ground_footprint_m = footprint_m


def overlap_spacing_m(
    model: CameraModel, altitude_m: float, overlap: float = 0.20
) -> tuple[float, float]:
    return model.spacing_m(altitude_m, overlap)


@dataclass(frozen=True, slots=True)
class CameraFrame:
    camera: str
    jpeg: bytes
    timestamp: datetime

    def decode_bgr(self):
        """Decode with OpenCV when its optional dependency is installed."""
        try:
            import cv2  # type: ignore[import-not-found]
            import numpy as np  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("frame decoding requires whiteout[camera]") from exc
        image = cv2.imdecode(np.frombuffer(self.jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("invalid JPEG frame")
        return image


class MjpegCamera:
    def __init__(
        self,
        name: str,
        url: str,
        *,
        reconnect_delay_s: float = 1.0,
        timeout_s: float = 5.0,
        opener: Callable[..., object] = urllib.request.urlopen,
    ) -> None:
        self.name = name
        self.url = url
        self.reconnect_delay_s = reconnect_delay_s
        self.timeout_s = timeout_s
        self._opener = opener

    def frames(self, *, reconnect: bool = True) -> Iterator[CameraFrame]:
        while True:
            try:
                with self._opener(self.url, timeout=self.timeout_s) as response:  # type: ignore[attr-defined]
                    yield from self._read_stream(response)
            except (OSError, TimeoutError, ValueError):
                if not reconnect:
                    return
                time.sleep(self.reconnect_delay_s)

    def _read_stream(self, response: object) -> Iterator[CameraFrame]:
        buffer = bytearray()
        while True:
            chunk = response.read(4096)  # type: ignore[attr-defined]
            if not chunk:
                raise OSError("MJPEG stream ended")
            buffer.extend(chunk)
            while True:
                start = buffer.find(b"\xff\xd8")
                end = buffer.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                if start < 0 or end < 0:
                    break
                jpeg = bytes(buffer[start : end + 2])
                del buffer[: end + 2]
                yield CameraFrame(self.name, jpeg, datetime.now(timezone.utc))
            if len(buffer) > 8_000_000:
                del buffer[:-2]
