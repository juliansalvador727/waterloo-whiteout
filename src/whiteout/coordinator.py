"""Mission coordination that emits typed data and never performs effects."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Protocol

from .geolocation import CameraIntrinsics, CameraPose, estimate_flat_water
from .models import (
    ActionRecommendation,
    ControlIntent,
    Detection,
    GeoEstimate,
    Pixel,
    SearchMode,
    Track,
    TrackEstimate,
)
from .observation import ObservationMetadata, SynchronizationStatus
from .tower import FORT_ROSS_TOWERS, TowerCalibration, TowerWorldPose
from .tracker import ConstantVelocityTracker

if TYPE_CHECKING:
    from .config import AppConfig


class Bounds(Protocol):
    def contains(self, latitude: float, longitude: float) -> bool: ...


@dataclass(frozen=True, slots=True)
class CoordinationResult:
    recommendation: ActionRecommendation
    track_estimate: TrackEstimate | None = None
    intents: tuple[ControlIntent, ...] = ()

    @property
    def track(self) -> Track | None:
        return self.track_estimate.track if self.track_estimate is not None else None


@dataclass(slots=True)
class Coordinator:
    confirmations_required: int = 2
    reacquire_after_s: float = 2.0
    search_after_s: float = 15.0
    mode: SearchMode = SearchMode.SEARCH
    submission_after_s: float = 5.0
    maximum_uncertainty_95_m: float = 100.0
    course_contains: Callable[[float, float], bool] | None = None
    course_bounds: Bounds | None = None
    tracker: ConstantVelocityTracker = field(default_factory=ConstantVelocityTracker)
    tower_poses: tuple[TowerWorldPose, ...] = field(
        default_factory=lambda: FORT_ROSS_TOWERS
    )
    _confirmations: int = 0
    _last_seen_s: float | None = None
    _track_started_s: float | None = None

    def __post_init__(self) -> None:
        if self.submission_after_s < 0 or self.maximum_uncertainty_95_m <= 0:
            raise ValueError("submission timeout and uncertainty limit must be positive")

    def decide(self, now_s: float, estimate: GeoEstimate | None) -> ActionRecommendation:
        """Backward-compatible recommendation-only interface."""
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

    @classmethod
    def from_config(cls, config: AppConfig, **kwargs: object) -> Coordinator:
        """Build a coordinator using the configured geographic course bounds."""
        return cls(course_bounds=config.course_bounds, **kwargs)

    def coordinate(
        self,
        now_s: float,
        estimate: GeoEstimate | None,
        *,
        synchronized: bool = True,
        in_course: bool | None = None,
        source: str | None = None,
    ) -> CoordinationResult:
        """Consume a validated geolocation and emit track plus inert control intents."""
        if estimate is not None:
            inside = self._in_course(estimate) if in_course is None else in_course
            if not synchronized or not inside:
                reason = "observation is not synchronized" if not synchronized else "observation is outside course bounds"
                return CoordinationResult(ActionRecommendation(self.mode, "ignore_observation", reason))

            previous_update = self.tracker.track.last_update if self.tracker.track is not None else None
            track = self.tracker.update(estimate)
            accepted = previous_update is None or track.last_update == estimate.timestamp
            if not accepted:
                return CoordinationResult(
                    ActionRecommendation(self.mode, "ignore_observation", self.tracker.last_rejection_reason or "incompatible observation")
                )

            self._last_seen_s = now_s
            if self._track_started_s is None:
                self._track_started_s = now_s
            self._confirmations += 1
            # The integrated path always requires a compatible second detection.
            # ``decide`` retains its original configurable confirmation behavior.
            confirmed = self._confirmations >= max(2, self.confirmations_required)
            self.mode = SearchMode.TRACK if confirmed else SearchMode.CONFIRM
            estimate_state = TrackEstimate(
                track,
                confirmed=confirmed,
                provisional=not confirmed,
                submit_eligible=self._submission_allowed(now_s, track.uncertainty_m),
                source=source,
            )
            action = "maintain_observation" if confirmed else "seek_second_observation"
            reason = "target confirmed" if confirmed else "valid provisional target"
            target = GeoEstimate(track.latitude, track.longitude, track.uncertainty_m, track.last_update)
            intents = self._target_intents("cue", "divert", target, reason)
            return CoordinationResult(ActionRecommendation(self.mode, action, reason, estimate), estimate_state, intents)

        return self._without_observation(now_s)

    def process_detection(
        self,
        now_s: float,
        detection: Detection,
        metadata: ObservationMetadata,
        intrinsics: CameraIntrinsics,
        pose: CameraPose,
    ) -> CoordinationResult:
        """Geolocate the detector waterline and coordinate only synchronized data."""
        if (
            not math.isfinite(detection.confidence)
            or not 0 <= detection.confidence <= 1
            or detection.confidence <= 0
        ):
            return CoordinationResult(
                ActionRecommendation(
                    self.mode, "ignore_observation", "detection confidence is not positive"
                )
            )
        synchronized = metadata.synchronization is SynchronizationStatus.SYNCHRONIZED
        if not synchronized:
            return CoordinationResult(
                ActionRecommendation(
                    self.mode, "ignore_observation", "observation is not synchronized"
                )
            )
        waterline = detection.metadata.get("waterline")
        pixel = Pixel(float(waterline[0]), float(waterline[1])) if isinstance(waterline, (tuple, list)) and len(waterline) == 2 else detection.pixel
        try:
            estimate = estimate_flat_water(
                pixel,
                intrinsics,
                pose,
                observation_timestamp=detection.timestamp,
            )
        except ValueError as exc:
            return CoordinationResult(
                ActionRecommendation(
                    self.mode,
                    "ignore_observation",
                    f"geolocation rejected observation: {exc}",
                )
            )
        return self.coordinate(
            now_s,
            estimate,
            synchronized=synchronized,
            source=detection.camera,
        )

    def _without_observation(self, now_s: float) -> CoordinationResult:
        if self._last_seen_s is None:
            self.mode = SearchMode.SEARCH
            return CoordinationResult(ActionRecommendation(self.mode, "continue_search_pattern", "no candidate observed"))

        age = max(0.0, now_s - self._last_seen_s)
        if age >= self.search_after_s:
            self.mode = SearchMode.SEARCH
            self._confirmations = 0
            self._last_seen_s = None
            self._track_started_s = None
            self.tracker = ConstantVelocityTracker(track_id=self.tracker.track_id)
            reason = "target stale"
            return CoordinationResult(
                ActionRecommendation(self.mode, "resume_search_pattern", reason),
                intents=(ControlIntent("quadcopter", "resume_search", reason=reason),),
            )

        predicted = self.tracker.predict(age)
        track_estimate = None
        if predicted is not None:
            track_estimate = TrackEstimate(
                predicted,
                confirmed=self._confirmations >= max(2, self.confirmations_required),
                provisional=self._confirmations < max(2, self.confirmations_required),
                submit_eligible=self._submission_allowed(now_s, predicted.uncertainty_m),
            )
        if age >= self.reacquire_after_s:
            self.mode = SearchMode.REACQUIRE
            target = None
            intents: tuple[ControlIntent, ...] = ()
            if predicted is not None:
                target = GeoEstimate(predicted.latitude, predicted.longitude, predicted.uncertainty_m, predicted.last_update)
                intents = self._target_intents(
                    "reacquire",
                    "reacquire",
                    target,
                    "target temporarily lost",
                )
            return CoordinationResult(
                ActionRecommendation(self.mode, "search_near_last_track", "target temporarily lost", target),
                track_estimate,
                intents,
            )
        return CoordinationResult(
            ActionRecommendation(self.mode, "hold_observation", "waiting for next observation"),
            track_estimate,
        )

    def _submission_allowed(self, now_s: float, uncertainty_m: float) -> bool:
        if self._last_seen_s is None or now_s - self._last_seen_s >= self.submission_after_s:
            return False
        return 1.959963984540054 * uncertainty_m < self.maximum_uncertainty_95_m

    def _in_course(self, estimate: GeoEstimate) -> bool:
        if self.course_contains is not None:
            return self.course_contains(estimate.latitude, estimate.longitude)
        return self.course_bounds is None or self.course_bounds.contains(
            estimate.latitude, estimate.longitude
        )

    def _target_intents(
        self,
        tower_action: str,
        quad_action: str,
        target: GeoEstimate,
        reason: str,
    ) -> tuple[ControlIntent, ...]:
        intents = list(self._tower_intents(tower_action, target, reason))
        intents.append(ControlIntent("quadcopter", quad_action, target, reason))
        return tuple(intents)

    def _tower_intents(
        self,
        action: str, target: GeoEstimate, reason: str
    ) -> tuple[ControlIntent, ...]:
        calibration = TowerCalibration()
        intents: list[ControlIntent] = []
        for tower in self.tower_poses:
            bearing_deg, horizontal_range_m = _bearing_and_range(
                tower.latitude,
                tower.longitude,
                target.latitude,
                target.longitude,
            )
            pan_deg = calibration.target_pan(bearing_deg)
            tilt_deg = math.degrees(
                # ArcticSim's tower joint convention is positive downward.
                math.atan2(tower.camera_world_z_m, horizontal_range_m)
            )
            if not (
                calibration.pan_min_deg <= pan_deg <= calibration.pan_max_deg
                and calibration.tilt_min_deg <= tilt_deg <= calibration.tilt_max_deg
            ):
                continue
            intents.append(
                ControlIntent(
                    asset=tower.name,
                    action=action,
                    target=target,
                    reason=reason,
                    pan_deg=pan_deg,
                    tilt_deg=tilt_deg,
                )
            )
        return tuple(intents)


def _bearing_and_range(
    start_latitude: float,
    start_longitude: float,
    target_latitude: float,
    target_longitude: float,
) -> tuple[float, float]:
    """Return true initial bearing and great-circle horizontal range."""
    start_lat = math.radians(start_latitude)
    target_lat = math.radians(target_latitude)
    delta_lat = target_lat - start_lat
    delta_lon = math.radians(target_longitude - start_longitude)
    y = math.sin(delta_lon) * math.cos(target_lat)
    x = (
        math.cos(start_lat) * math.sin(target_lat)
        - math.sin(start_lat) * math.cos(target_lat) * math.cos(delta_lon)
    )
    bearing = math.degrees(math.atan2(y, x)) % 360.0
    haversine = (
        math.sin(delta_lat / 2.0) ** 2
        + math.cos(start_lat)
        * math.cos(target_lat)
        * math.sin(delta_lon / 2.0) ** 2
    )
    central_angle = 2.0 * math.atan2(
        math.sqrt(haversine), math.sqrt(max(0.0, 1.0 - haversine))
    )
    return bearing, 6_378_137.0 * central_angle
