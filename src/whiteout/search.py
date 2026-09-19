"""Detector-independent search mission orchestration.

Outputs are inert data. This module has no controller or transmission dependency.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .models import Detection, SearchMode


class DetectionSource(str, Enum):
    DETECTOR = "detector"
    MANUAL = "manual"
    SYNTHETIC = "synthetic"


class SearchAction(str, Enum):
    CONTINUE_SEARCH = "continue_search"
    CONFIRM_CANDIDATE = "confirm_candidate"
    MAINTAIN_TRACK = "maintain_track"
    REACQUIRE = "reacquire"
    RETURN_TO_LAUNCH = "return_to_launch"
    ABORT_TO_RTL = "abort_to_rtl"


@dataclass(frozen=True, slots=True)
class SearchRecommendation:
    state: SearchMode
    action: SearchAction
    reason: str
    detection: Detection | None = None
    source: DetectionSource | None = None

    @property
    def detection_confirmed(self) -> bool:
        return self.state is SearchMode.TRACK


@dataclass(slots=True)
class SearchOrchestrator:
    confirmations_required: int = 2
    reacquire_after_s: float = 2.0
    return_after_s: float = 15.0
    minimum_confidence: float = 0.0
    state: SearchMode = SearchMode.SEARCH
    _confirmations: int = 0
    _last_detection_s: float | None = None

    def __post_init__(self) -> None:
        if self.confirmations_required < 2:
            raise ValueError("at least two observations are required for confirmation")
        if self.reacquire_after_s < 0 or self.return_after_s <= self.reacquire_after_s:
            raise ValueError("return timeout must be greater than reacquire timeout")
        if not 0 <= self.minimum_confidence <= 1:
            raise ValueError("minimum confidence must be between 0 and 1")

    def observe(
        self,
        now_s: float,
        detection: Detection | None,
        *,
        source: DetectionSource = DetectionSource.DETECTOR,
    ) -> SearchRecommendation:
        """Consume a detection or absence and return inert recommendation data."""
        if self.state is SearchMode.ABORT:
            return self.abort("abort remains active")
        if self.state is SearchMode.RETURN:
            return self.request_return("return remains active")

        if detection is not None and detection.confidence >= self.minimum_confidence:
            self._last_detection_s = now_s
            self._confirmations += 1
            if self._confirmations >= self.confirmations_required:
                self.state = SearchMode.TRACK
                return SearchRecommendation(
                    self.state,
                    SearchAction.MAINTAIN_TRACK,
                    "candidate confirmed by repeated observations",
                    detection,
                    source,
                )
            self.state = SearchMode.CONFIRM
            return SearchRecommendation(
                self.state,
                SearchAction.CONFIRM_CANDIDATE,
                "candidate is unconfirmed; do not pursue or alter flight for it",
                detection,
                source,
            )

        if self._last_detection_s is None:
            self.state = SearchMode.SEARCH
            return SearchRecommendation(
                self.state, SearchAction.CONTINUE_SEARCH, "no candidate observed"
            )

        age = max(0.0, now_s - self._last_detection_s)
        if age >= self.return_after_s:
            return self.request_return("confirmed target could not be reacquired")
        if age >= self.reacquire_after_s:
            self.state = SearchMode.REACQUIRE
            return SearchRecommendation(
                self.state, SearchAction.REACQUIRE, "target temporarily lost"
            )
        action = (
            SearchAction.MAINTAIN_TRACK
            if self.state is SearchMode.TRACK
            else SearchAction.CONFIRM_CANDIDATE
        )
        return SearchRecommendation(self.state, action, "awaiting another observation")

    def inject(
        self,
        now_s: float,
        detection: Detection | None,
        *,
        synthetic: bool = False,
    ) -> SearchRecommendation:
        """Offline/manual input path with explicit provenance."""
        source = DetectionSource.SYNTHETIC if synthetic else DetectionSource.MANUAL
        return self.observe(now_s, detection, source=source)

    def request_return(self, reason: str = "operator requested return") -> SearchRecommendation:
        self.state = SearchMode.RETURN
        return SearchRecommendation(self.state, SearchAction.RETURN_TO_LAUNCH, reason)

    def abort(self, reason: str = "operator requested abort") -> SearchRecommendation:
        self.state = SearchMode.ABORT
        return SearchRecommendation(self.state, SearchAction.ABORT_TO_RTL, reason)
