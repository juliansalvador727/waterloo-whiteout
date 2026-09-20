"""Telemetry-only MAVLink adapter with an optional GCS discovery heartbeat."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone

from .models import Telemetry


class MavlinkUnavailable(RuntimeError):
    pass


@dataclass(slots=True)
class ReadOnlyMavlink:
    vehicle: str
    host: str
    port: int
    attitude_max_age_s: float = 0.5
    synchronization_tolerance_s: float = 0.25
    max_messages_per_poll: int = 8
    _connection: object | None = None
    _attitude: tuple[float, float, float] | None = field(default=None, init=False)
    _attitude_boot_ms: int | None = field(default=None, init=False)
    _attitude_received_at: datetime | None = field(default=None, init=False)
    _attitude_received_monotonic: float | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        if self.attitude_max_age_s < 0 or self.synchronization_tolerance_s < 0:
            raise ValueError("attitude age and synchronization tolerance cannot be negative")
        if self.max_messages_per_poll < 1:
            raise ValueError("messages per poll must be positive")

    def connect(self, *, initiate_telemetry: bool = False) -> None:
        try:
            from pymavlink import mavutil  # type: ignore[import-not-found]
        except ImportError as exc:
            raise MavlinkUnavailable("MAVLink telemetry requires whiteout[mavlink]") from exc
        self._connection = mavutil.mavlink_connection(f"udpout:{self.host}:{self.port}")
        self._attitude = None
        self._attitude_boot_ms = None
        self._attitude_received_at = None
        self._attitude_received_monotonic = None
        if initiate_telemetry:
            self._connection.mav.heartbeat_send(  # type: ignore[attr-defined]
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )

    def poll(self) -> Telemetry | None:
        if self._connection is None:
            raise RuntimeError("MAVLink connection is not open")
        for _ in range(self.max_messages_per_poll):
            message = self._connection.recv_match(  # type: ignore[attr-defined]
                type=["ATTITUDE", "GLOBAL_POSITION_INT"], blocking=False
            )
            if message is None:
                return None
            message_type = message.get_type()  # type: ignore[attr-defined]
            if message_type == "ATTITUDE":
                self._record_attitude(message)
            elif message_type == "GLOBAL_POSITION_INT":
                return self._position_telemetry(message)
        return None

    def _record_attitude(self, attitude: object) -> None:
        try:
            values = (
                float(attitude.roll),  # type: ignore[attr-defined]
                float(attitude.pitch),  # type: ignore[attr-defined]
                float(attitude.yaw),  # type: ignore[attr-defined]
            )
        except (AttributeError, TypeError, ValueError):
            values = (math.nan, math.nan, math.nan)
        boot_ms = getattr(attitude, "time_boot_ms", None)
        if all(math.isfinite(value) for value in values) and isinstance(boot_ms, int):
            self._attitude = values
            self._attitude_boot_ms = boot_ms
            self._attitude_received_at = datetime.now(timezone.utc)
            self._attitude_received_monotonic = time.monotonic()
        else:
            self._attitude = None
            self._attitude_boot_ms = None
            self._attitude_received_at = None
            self._attitude_received_monotonic = None

    def _position_telemetry(self, message: object) -> Telemetry:
        received_at = datetime.now(timezone.utc)
        values: tuple[float | None, float | None, float | None] = (None, None, None)
        attitude_timestamp = None
        position_boot_ms = getattr(message, "time_boot_ms", None)
        attitude_fresh = (
            self._attitude is not None
            and self._attitude_received_monotonic is not None
            and time.monotonic() - self._attitude_received_monotonic <= self.attitude_max_age_s
        )
        synchronized = (
            attitude_fresh
            and isinstance(position_boot_ms, int)
            and self._attitude_boot_ms is not None
            and abs(position_boot_ms - self._attitude_boot_ms)
            <= self.synchronization_tolerance_s * 1000.0
        )
        if synchronized:
            values = self._attitude  # type: ignore[assignment]
            attitude_timestamp = self._attitude_received_at
        else:
            heading_cdeg = getattr(message, "hdg", 65535)
            yaw_rad = (
                math.radians(float(heading_cdeg) / 100.0)
                if heading_cdeg not in (None, 65535)
                else None
            )
            values = (None, None, yaw_rad)
        return Telemetry(
            vehicle=self.vehicle,
            latitude=float(message.lat) / 1e7,  # type: ignore[attr-defined]
            longitude=float(message.lon) / 1e7,  # type: ignore[attr-defined]
            # GLOBAL_POSITION_INT.alt is altitude above mean sea level. The
            # Fort Ross water plane is z=0, so launch-relative altitude must
            # never be used for vessel geolocation.
            altitude_m=float(message.alt) / 1000.0,  # type: ignore[attr-defined]
            roll_rad=values[0],
            pitch_rad=values[1],
            yaw_rad=values[2],
            timestamp=received_at,
            attitude_timestamp=attitude_timestamp,
        )
