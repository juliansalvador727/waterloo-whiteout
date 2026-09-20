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

    def set_position(self, north_m: float, east_m: float, down_m: float) -> None: ...

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
            self.copter.resume_search()
            return
        if intent.action not in {"divert", "reacquire"} or intent.target is None:
            raise ValueError(f"unsupported quadcopter intent {intent.action!r}")
        telemetry = self.telemetry_for("quadcopter")
        if telemetry is None:
            raise RuntimeError("quadcopter telemetry is unavailable for target conversion")
        north_m, east_m = _local_offset_m(
            telemetry.latitude,
            telemetry.longitude,
            intent.target.latitude,
            intent.target.longitude,
        )
        self.copter.set_mode(CopterMode.GUIDED)
        self.copter.set_position(north_m, east_m, 0.0)

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
