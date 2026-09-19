"""Validation for MAVProxy-compatible QGroundControl WPL missions."""

from __future__ import annotations

from dataclasses import dataclass
import math
from os import PathLike
from pathlib import Path


class MissionValidationError(ValueError):
    """Raised when a mission is not a safe, supported QGC WPL file."""


@dataclass(frozen=True, slots=True)
class MissionWaypoint:
    sequence: int
    current: bool
    frame: int
    command: int
    latitude: float
    longitude: float
    altitude_m: float
    autocontinue: bool


@dataclass(frozen=True, slots=True)
class SearchMission:
    """A validated search mission which has not been uploaded or started."""

    path: Path
    waypoints: tuple[MissionWaypoint, ...]

    @classmethod
    def load(cls, path: str | PathLike[str]) -> SearchMission:
        mission_path = Path(path).expanduser().resolve()
        if not mission_path.is_file():
            raise FileNotFoundError(f"mission not found: {mission_path}")
        try:
            lines = mission_path.read_text(encoding="utf-8").splitlines()
        except UnicodeError as exc:
            raise MissionValidationError("mission must be UTF-8 text") from exc
        if not lines or lines[0].strip() != "QGC WPL 110":
            raise MissionValidationError("mission must start with 'QGC WPL 110'")

        waypoints: list[MissionWaypoint] = []
        for line_number, line in enumerate(lines[1:], start=2):
            if not line.strip():
                continue
            fields = line.split("\t")
            if len(fields) != 12:
                raise MissionValidationError(
                    f"mission line {line_number} must have 12 tab-separated fields"
                )
            try:
                sequence, current, frame, command = map(int, fields[:4])
                parameters = tuple(float(value) for value in fields[4:11])
                autocontinue = int(fields[11])
            except ValueError as exc:
                raise MissionValidationError(
                    f"mission line {line_number} contains an invalid number"
                ) from exc
            if sequence != len(waypoints):
                raise MissionValidationError("mission waypoint sequences must start at 0 and be contiguous")
            if current not in (0, 1) or autocontinue not in (0, 1):
                raise MissionValidationError("current and autocontinue fields must be 0 or 1")
            if not all(math.isfinite(value) for value in parameters):
                raise MissionValidationError("mission values must be finite")
            latitude, longitude, altitude_m = parameters[4:]
            if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
                raise MissionValidationError("mission latitude or longitude is out of range")
            if frame not in (0, 3):
                raise MissionValidationError("only MAV_FRAME_GLOBAL and GLOBAL_RELATIVE_ALT are supported")
            waypoints.append(
                MissionWaypoint(
                    sequence,
                    bool(current),
                    frame,
                    command,
                    latitude,
                    longitude,
                    altitude_m,
                    bool(autocontinue),
                )
            )

        if len(waypoints) < 2:
            raise MissionValidationError("mission must contain at least two waypoints")
        if not waypoints[0].current:
            raise MissionValidationError("mission home row must be marked current")
        if any(waypoint.current for waypoint in waypoints[1:]):
            raise MissionValidationError("mission route rows must not be marked current")
        return cls(mission_path, tuple(waypoints))

    def validate_for_search(self) -> SearchMission:
        """Reject landing, takeoff, or non-navigation commands in a search route."""
        home, *route = self.waypoints
        if home.command != 16 or home.frame != 0:
            raise MissionValidationError("search mission item 0 must be a global home waypoint")
        if not route or any(waypoint.command != 16 for waypoint in route):
            raise MissionValidationError("search missions may contain only MAV_CMD_NAV_WAYPOINT commands")
        if any(waypoint.frame != 3 for waypoint in route):
            raise MissionValidationError("search waypoints must use relative altitude frame 3")
        if any(waypoint.altitude_m <= 0 for waypoint in route):
            raise MissionValidationError("search waypoint altitudes must be positive")
        return self
