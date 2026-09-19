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
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    timestamp: datetime = field(default_factory=utc_now)


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


@dataclass(frozen=True, slots=True)
class ActionRecommendation:
    mode: SearchMode
    action: str
    reason: str
    target: GeoEstimate | None = None

