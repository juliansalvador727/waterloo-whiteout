"""Synchronized context attached to imagery and detector observations."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from enum import Enum

from .camera import CameraFrame
from .models import Telemetry


class SynchronizationStatus(str, Enum):
    SYNCHRONIZED = "synchronized"
    TELEMETRY_MISSING = "telemetry_missing"
    TELEMETRY_STALE = "telemetry_stale"
    ATTITUDE_MISSING = "attitude_missing"
    ATTITUDE_UNSYNCHRONIZED = "attitude_unsynchronized"


@dataclass(frozen=True, slots=True)
class ObservationMetadata:
    """Immutable provenance for a frame without altering existing frame models."""

    frame_timestamp: datetime
    camera: str
    asset: str
    telemetry: Telemetry | None
    synchronization: SynchronizationStatus
    mission: str | None = None
    waypoint_sequence: int | None = None
    tower_pan_pwm: int | None = None
    tower_tilt_pwm: int | None = None

    @classmethod
    def for_frame(
        cls,
        frame: CameraFrame,
        *,
        asset: str,
        telemetry: Telemetry | None = None,
        max_skew_s: float = 0.25,
        mission: str | None = None,
        waypoint_sequence: int | None = None,
        tower_pan_pwm: int | None = None,
        tower_tilt_pwm: int | None = None,
    ) -> ObservationMetadata:
        if not asset.strip():
            raise ValueError("observation asset must not be empty")
        if max_skew_s < 0:
            raise ValueError("maximum timestamp skew cannot be negative")
        if waypoint_sequence is not None and waypoint_sequence < 0:
            raise ValueError("waypoint sequence cannot be negative")
        for value in (tower_pan_pwm, tower_tilt_pwm):
            if value is not None and not 1000 <= value <= 2000:
                raise ValueError("tower pan and tilt PWM must be between 1000 and 2000")

        if telemetry is None:
            status = SynchronizationStatus.TELEMETRY_MISSING
        elif telemetry.vehicle != asset:
            raise ValueError("telemetry vehicle does not match observation asset")
        elif abs((telemetry.timestamp - frame.timestamp).total_seconds()) > max_skew_s:
            status = SynchronizationStatus.TELEMETRY_STALE
        elif not telemetry.has_attitude:
            status = SynchronizationStatus.ATTITUDE_MISSING
        elif telemetry.attitude_timestamp is None or abs(
            (telemetry.attitude_timestamp - frame.timestamp).total_seconds()
        ) > max_skew_s:
            status = SynchronizationStatus.ATTITUDE_UNSYNCHRONIZED
        else:
            status = SynchronizationStatus.SYNCHRONIZED

        return cls(
            frame.timestamp,
            frame.camera,
            asset,
            telemetry,
            status,
            mission,
            waypoint_sequence,
            tower_pan_pwm,
            tower_tilt_pwm,
        )
