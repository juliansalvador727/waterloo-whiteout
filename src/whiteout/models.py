"""Typed domain models shared by the controller components."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Mapping


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


@dataclass(frozen=True, slots=True)
class Pixel:
    x: float
    y: float


@dataclass(frozen=True, slots=True)
class BoundingBox:
    """Pixel-aligned target bounds using inclusive top-left/exclusive bottom-right edges."""

    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def __post_init__(self) -> None:
        if self.x_max <= self.x_min or self.y_max <= self.y_min:
            raise ValueError("bounding box must have positive width and height")

    @property
    def center(self) -> Pixel:
        return Pixel((self.x_min + self.x_max) / 2.0, (self.y_min + self.y_max) / 2.0)


@dataclass(frozen=True, slots=True)
class Detection:
    camera: str
    pixel: Pixel
    confidence: float
    timestamp: datetime = field(default_factory=utc_now)
    label: str = "target"
    metadata: Mapping[str, Any] = field(default_factory=dict)
    bbox: BoundingBox | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.confidence) or not 0 <= self.confidence <= 1:
            raise ValueError("detection confidence must be finite and between 0 and 1")


@dataclass(frozen=True, slots=True)
class GeoEstimate:
    latitude: float
    longitude: float
    uncertainty_m: float
    timestamp: datetime = field(default_factory=utc_now)


@dataclass(frozen=True, slots=True)
class Telemetry:
    vehicle: str
    latitude: float
    longitude: float
    altitude_m: float
    roll_rad: float | None = None
    pitch_rad: float | None = None
    yaw_rad: float | None = None
    timestamp: datetime = field(default_factory=utc_now)
    attitude_timestamp: datetime | None = None

    @property
    def has_attitude(self) -> bool:
        return (
            self.roll_rad is not None
            and self.pitch_rad is not None
            and self.yaw_rad is not None
            and self.attitude_timestamp is not None
        )


@dataclass(frozen=True, slots=True)
class Track:
    track_id: str
    latitude: float
    longitude: float
    velocity_north_mps: float
    velocity_east_mps: float
    uncertainty_m: float
    last_update: datetime = field(default_factory=utc_now)


@dataclass(frozen=True, slots=True)
class TrackEstimate:
    """Operational track state, including confirmation and submission fitness."""

    track: Track
    confirmed: bool = False
    provisional: bool = True
    submit_eligible: bool = True
    source: str | None = None

    @property
    def uncertainty_95_m(self) -> float:
        return 1.959963984540054 * self.track.uncertainty_m

    @property
    def latitude(self) -> float:
        return self.track.latitude

    @property
    def longitude(self) -> float:
        return self.track.longitude

    @property
    def timestamp(self) -> datetime:
        return self.track.last_update


@dataclass(frozen=True, slots=True)
class ControlIntent:
    """An inert request for an executor, never an actuator command by itself."""

    asset: str
    action: str
    target: GeoEstimate | None = None
    reason: str = ""
    pan_deg: float | None = None
    tilt_deg: float | None = None


class SearchMode(str, Enum):
    SEARCH = "SEARCH"
    CONFIRM = "CONFIRM"
    TRACK = "TRACK"
    REACQUIRE = "REACQUIRE"
    RETURN = "RETURN"
    ABORT = "ABORT"


@dataclass(frozen=True, slots=True)
class ActionRecommendation:
    mode: SearchMode
    action: str
    reason: str
    target: GeoEstimate | None = None
