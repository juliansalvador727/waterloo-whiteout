"""Adapters that translate approved pipeline effects into live transports."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Protocol

from .control import CopterMode
from .models import ControlIntent, Telemetry, TrackEstimate
from .track_api import TrackApiClient


class _CopterController(Protocol):
    def set_mode(self, mode: CopterMode) -> None: ...

    def goto_global(self, latitude: float, longitude: float, relative_altitude_m: float) -> None: ...

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
        self.copter.set_mode(CopterMode.GUIDED)
        self.copter.goto_global(
            intent.target.latitude,
            intent.target.longitude,
            telemetry.relative_altitude_m,
        )

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
