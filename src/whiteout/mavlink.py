"""Telemetry-only MAVLink adapter with an optional GCS discovery heartbeat."""

from __future__ import annotations

from dataclasses import dataclass

from .models import Telemetry


class MavlinkUnavailable(RuntimeError):
    pass


@dataclass(slots=True)
class ReadOnlyMavlink:
    vehicle: str
    host: str
    port: int
    _connection: object | None = None

    def connect(self, *, initiate_telemetry: bool = False) -> None:
        try:
            from pymavlink import mavutil  # type: ignore[import-not-found]
        except ImportError as exc:
            raise MavlinkUnavailable("MAVLink telemetry requires whiteout[mavlink]") from exc
        self._connection = mavutil.mavlink_connection(f"udpout:{self.host}:{self.port}")
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
        message = self._connection.recv_match(type="GLOBAL_POSITION_INT", blocking=False)  # type: ignore[attr-defined]
        if message is None:
            return None
        return Telemetry(
            vehicle=self.vehicle,
            latitude=float(message.lat) / 1e7,
            longitude=float(message.lon) / 1e7,
            altitude_m=float(message.relative_alt) / 1000.0,
        )
