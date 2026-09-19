"""Typed domain models shared by the controller components."""

from __future__ import annotations

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
class Detection:
    camera: str
    pixel: Pixel
    confidence: float
    timestamp: datetime = field(default_factory=utc_now)
    label: str = "target"
    metadata: Mapping[str, Any] = field(default_factory=dict)


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
