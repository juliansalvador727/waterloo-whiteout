"""Explicit side-effect boundaries for control intents and track output."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Protocol

from .models import ControlIntent, TrackEstimate


@dataclass(frozen=True, slots=True)
class OperatorSession:
    operator: str
    active: bool = False

    def __post_init__(self) -> None:
        if not self.operator.strip():
            raise ValueError("operator session requires an operator identity")


class Executor(Protocol):
    def execute(self, intent: ControlIntent) -> None: ...


class TrackSink(Protocol):
    def submit(self, estimate: TrackEstimate) -> None: ...


@dataclass(slots=True)
class RecordingExecutor:
    """Record intents in memory without controller, process, or network access."""

    intents: list[ControlIntent] = field(default_factory=list)

    def execute(self, intent: ControlIntent) -> None:
        self.intents.append(intent)


@dataclass(slots=True)
class RecordingTrackSink:
    """Record track output in memory without opening a transport."""

    estimates: list[TrackEstimate] = field(default_factory=list)

    def submit(self, estimate: TrackEstimate) -> None:
        self.estimates.append(estimate)

    @property
    def tracks(self) -> list[TrackEstimate]:
        return self.estimates


class LiveExecutor:
    """Invoke an injected actuator only with both live safety gates open."""

    def __init__(
        self,
        actuator: Callable[[ControlIntent], None],
        *,
        live_enabled: bool = False,
        operator_session: OperatorSession | None = None,
    ) -> None:
        self._actuator = actuator
        self.live_enabled = live_enabled
        self.operator_session = operator_session

    def execute(self, intent: ControlIntent) -> None:
        _require_live(self.live_enabled, self.operator_session)
        asset = intent.asset.strip().lower().replace("_", "-")
        if asset in {"fixed-wing", "fixedwing", "plane"}:
            raise PermissionError("the fixed-wing aircraft may not be retasked")
        self._actuator(intent)


class LiveTrackSink:
    """Invoke an injected submission transport only with live authorization."""

    def __init__(
        self,
        sender: Callable[[TrackEstimate], None],
        *,
        live_enabled: bool = False,
        operator_session: OperatorSession | None = None,
    ) -> None:
        self._sender = sender
        self.live_enabled = live_enabled
        self.operator_session = operator_session

    def submit(self, estimate: TrackEstimate) -> None:
        _require_live(self.live_enabled, self.operator_session)
        if not estimate.submit_eligible:
            raise PermissionError("track estimate is not submit-eligible")
        self._sender(estimate)


def _require_live(enabled: bool, session: OperatorSession | None) -> None:
    if not enabled:
        raise PermissionError("live operation requires explicit enablement")
    if not isinstance(session, OperatorSession) or not session.active:
        raise PermissionError("live operation requires an active operator session")


# Clear aliases for callers that prefer the offline/live naming pair.
OfflineExecutor = RecordingExecutor
OfflineTrackSink = RecordingTrackSink
