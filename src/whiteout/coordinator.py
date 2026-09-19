"""Decision state machine that emits recommendations, never commands."""

from __future__ import annotations

from dataclasses import dataclass

from .models import ActionRecommendation, GeoEstimate, SearchMode


@dataclass(slots=True)
class Coordinator:
    confirmations_required: int = 2
    reacquire_after_s: float = 2.0
    search_after_s: float = 10.0
    mode: SearchMode = SearchMode.SEARCH
    _confirmations: int = 0
    _last_seen_s: float | None = None

    def decide(self, now_s: float, estimate: GeoEstimate | None) -> ActionRecommendation:
        if estimate is not None:
            self._last_seen_s = now_s
            self._confirmations += 1
            if self._confirmations >= self.confirmations_required:
                self.mode = SearchMode.TRACK
                return ActionRecommendation(self.mode, "maintain_observation", "target confirmed", estimate)
            self.mode = SearchMode.CONFIRM
            return ActionRecommendation(self.mode, "seek_second_observation", "candidate detected", estimate)

        if self._last_seen_s is None:
            self.mode = SearchMode.SEARCH
            return ActionRecommendation(self.mode, "continue_search_pattern", "no candidate observed")

        age = max(0.0, now_s - self._last_seen_s)
        if age >= self.search_after_s:
            self.mode = SearchMode.SEARCH
            self._confirmations = 0
            return ActionRecommendation(self.mode, "resume_search_pattern", "target stale")
        if age >= self.reacquire_after_s:
            self.mode = SearchMode.REACQUIRE
            return ActionRecommendation(self.mode, "search_near_last_track", "target temporarily lost")
        return ActionRecommendation(self.mode, "hold_observation", "waiting for next observation")

