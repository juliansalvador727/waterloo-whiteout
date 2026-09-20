"""Adapters that translate approved pipeline effects into live transports."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Protocol

from .control import CopterMode
from .models import ControlIntent, Telemetry, TrackEstimate
from .pose import QUADCOPTER_CAMERA_DOWN_DEG
from .track_api import TrackApiClient


class _CopterController(Protocol):
    def set_mode(self, mode: CopterMode) -> None: ...

    def goto_global(self, latitude: float, longitude: float, relative_altitude_m: float) -> None: ...

    def set_yaw(
        self, angle_deg: float, angular_speed_dps: float, *, relative: bool = False
    ) -> None: ...

    def set_mission_current(self, sequence: int) -> None: ...

    def resume_search(self) -> None: ...


class _TowerController(Protocol):
    def pan(self, angle_deg: float) -> None: ...

    def tilt(self, angle_deg: float) -> None: ...


@dataclass(slots=True)
class SimulatorActuator:
    """Translate coordinator intents into typed simulator-controller calls.

    This object does not provide authorization itself. It is intended to be
    wrapped by :class:`LiveExecutor`, which enforces live enablement and the
    active operator-session gate before invoking it.
    """

    copter: _CopterController
    towers: Mapping[str, _TowerController]
    telemetry_for: Callable[[str], Telemetry | None]
    course_contains: Callable[[float, float], bool] | None = None
    _interrupted_mission_sequence: int | None = None

    def __call__(self, intent: ControlIntent) -> None:
        asset = intent.asset.strip().lower().replace("_", "-")
        if asset == "quadcopter":
            self._actuate_copter(intent)
            return
        if asset in self.towers:
            self._actuate_tower(self.towers[asset], intent)
            return
        raise ValueError(f"no live actuator is configured for {intent.asset!r}")

    def _actuate_copter(self, intent: ControlIntent) -> None:
        if intent.action == "resume_search":
            if self._interrupted_mission_sequence is not None:
                self.copter.set_mission_current(self._interrupted_mission_sequence)
            self.copter.resume_search()
            self._interrupted_mission_sequence = None
            return
        if intent.action not in {"divert", "reacquire"} or intent.target is None:
            raise ValueError(f"unsupported quadcopter intent {intent.action!r}")
        telemetry = self.telemetry_for("quadcopter")
        if telemetry is None:
            raise RuntimeError("quadcopter telemetry is unavailable for target conversion")
        if telemetry.relative_altitude_m is None or telemetry.relative_altitude_m <= 0:
            raise RuntimeError("quadcopter relative altitude is unavailable")
        if self._interrupted_mission_sequence is None:
            self._interrupted_mission_sequence = telemetry.mission_sequence
        candidates = _forward_camera_observation_points(
            telemetry,
            intent.target.latitude,
            intent.target.longitude,
        )
        observation = next(
            (
                candidate
                for candidate in candidates
                if self.course_contains is None
                or self.course_contains(candidate[0], candidate[1])
            ),
            None,
        )
        if observation is None:
            raise RuntimeError(
                "no forward-camera observation point is inside the course bounds"
            )
        latitude, longitude, heading_deg = observation
        self.copter.set_mode(CopterMode.GUIDED)
        self.copter.goto_global(
            latitude,
            longitude,
            telemetry.relative_altitude_m,
        )
        self.copter.set_yaw(heading_deg, 20.0)

    @staticmethod
    def _actuate_tower(controller: _TowerController, intent: ControlIntent) -> None:
        if intent.action not in {"cue", "reacquire"}:
            raise ValueError(f"unsupported tower intent {intent.action!r}")
        if intent.pan_deg is None or intent.tilt_deg is None:
            raise ValueError("tower intent requires pan and tilt angles")
        controller.pan(intent.pan_deg)
        controller.tilt(intent.tilt_deg)


@dataclass(slots=True)
class TrackApiSubmitter:
    """Adapt a gated API client to the pipeline's ``TrackEstimate`` shape."""

    client: TrackApiClient

    def __call__(self, estimate: TrackEstimate) -> None:
        self.client.submit(estimate.track, confirmed=True)


def _local_offset_m(
    start_latitude: float,
    start_longitude: float,
    target_latitude: float,
    target_longitude: float,
) -> tuple[float, float]:
    radius_m = 6_378_137.0
    north_m = math.radians(target_latitude - start_latitude) * radius_m
    mean_latitude = math.radians((start_latitude + target_latitude) / 2.0)
    east_m = (
        math.radians(target_longitude - start_longitude)
        * radius_m
        * math.cos(mean_latitude)
    )
    return north_m, east_m


def _forward_camera_observation_points(
    telemetry: Telemetry,
    target_latitude: float,
    target_longitude: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return two standoff points whose forward camera can centre the target."""
    camera_height_m = float(telemetry.altitude_m)
    if not math.isfinite(camera_height_m) or camera_height_m <= 0:
        raise RuntimeError("quadcopter altitude above water is unavailable")
    standoff_m = camera_height_m / math.tan(math.radians(QUADCOPTER_CAMERA_DOWN_DEG))
    target_north_m, target_east_m = _local_offset_m(
        telemetry.latitude,
        telemetry.longitude,
        target_latitude,
        target_longitude,
    )
    target_range_m = math.hypot(target_north_m, target_east_m)
    if target_range_m > 0.01:
        forward_north = target_north_m / target_range_m
        forward_east = target_east_m / target_range_m
    elif telemetry.yaw_rad is not None and math.isfinite(telemetry.yaw_rad):
        forward_north = math.cos(telemetry.yaw_rad)
        forward_east = math.sin(telemetry.yaw_rad)
    else:
        raise RuntimeError(
            "cannot choose a forward-camera standoff direction without range or yaw"
        )

    preferred = _offset_coordinate(
        target_latitude,
        target_longitude,
        -forward_north * standoff_m,
        -forward_east * standoff_m,
    )
    alternate = _offset_coordinate(
        target_latitude,
        target_longitude,
        forward_north * standoff_m,
        forward_east * standoff_m,
    )
    heading_deg = math.degrees(math.atan2(forward_east, forward_north)) % 360.0
    return (
        (preferred[0], preferred[1], heading_deg),
        (alternate[0], alternate[1], (heading_deg + 180.0) % 360.0),
    )


def _offset_coordinate(
    latitude: float,
    longitude: float,
    north_m: float,
    east_m: float,
) -> tuple[float, float]:
    radius_m = 6_378_137.0
    result_latitude = latitude + math.degrees(north_m / radius_m)
    cosine = math.cos(math.radians(latitude))
    if abs(cosine) < 1e-9:
        raise RuntimeError("cannot calculate longitude offset at the geographic pole")
    result_longitude = longitude + math.degrees(east_m / (radius_m * cosine))
    return result_latitude, result_longitude
