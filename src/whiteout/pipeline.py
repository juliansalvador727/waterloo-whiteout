"""End-to-end observation coordination with explicit effect boundaries."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, field

from .camera import CameraFrame
from .coordinator import CoordinationResult, Coordinator
from .detector import Detector
from .execution import Executor, TrackSink
from .geolocation import CameraIntrinsics, CameraPose
from .models import ActionRecommendation, Detection, SearchMode, Telemetry, TrackEstimate
from .observation import ObservationMetadata


@dataclass(frozen=True, slots=True)
class PipelineCycle:
    """Everything produced from one frame or one no-frame timer tick."""

    result: CoordinationResult
    detections: tuple[Detection, ...] = ()
    submitted: TrackEstimate | None = None


@dataclass(slots=True)
class OperationalPipeline:
    """Connect detection, geolocation, tracking, intents, and submissions.

    The pipeline owns no network or vehicle connections. Side effects can only
    occur through the injected executor and track sink, which lets replay and
    tests use recording implementations while live callers use the gated
    implementations.
    """

    detector: Detector
    coordinator: Coordinator
    executor: Executor
    track_sink: TrackSink
    submission_interval_s: float = 1.0
    max_telemetry_skew_s: float = 0.25
    on_frame: Callable[[CameraFrame], None] | None = None
    on_detection: Callable[[Detection], None] | None = None
    on_result: Callable[[CoordinationResult], None] | None = None
    on_submission: Callable[[TrackEstimate], None] | None = None
    _last_submission_s: float | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        if self.submission_interval_s <= 0:
            raise ValueError("submission interval must be positive")
        if self.max_telemetry_skew_s < 0:
            raise ValueError("maximum telemetry skew cannot be negative")

    def process_frame(
        self,
        frame: CameraFrame,
        *,
        telemetry: Telemetry | None,
        intrinsics: CameraIntrinsics,
        pose: CameraPose | None,
        now_s: float | None = None,
        asset: str | None = None,
        mission: str | None = None,
        waypoint_sequence: int | None = None,
        tower_pan_pwm: int | None = None,
        tower_tilt_pwm: int | None = None,
    ) -> PipelineCycle:
        """Process one frame, selecting at most one candidate from that frame.

        Limiting a frame to its best candidate prevents duplicate boxes from a
        single image from incorrectly promoting a provisional track.
        """

        clock = time.monotonic() if now_s is None else float(now_s)
        source_asset = asset or frame.camera
        metadata = ObservationMetadata.for_frame(
            frame,
            asset=source_asset,
            telemetry=telemetry,
            max_skew_s=self.max_telemetry_skew_s,
            mission=mission,
            waypoint_sequence=waypoint_sequence,
            tower_pan_pwm=tower_pan_pwm,
            tower_tilt_pwm=tower_tilt_pwm,
        )
        if self.on_frame is not None:
            self.on_frame(frame)

        detections = tuple(
            sorted(self.detector.detect(frame), key=lambda item: item.confidence, reverse=True)
        )
        for detection in detections:
            if self.on_detection is not None:
                self.on_detection(detection)

        if detections and pose is not None:
            result = self.coordinator.process_detection(
                clock,
                detections[0],
                metadata,
                intrinsics,
                pose,
            )
        elif detections:
            result = CoordinationResult(
                ActionRecommendation(
                    self.coordinator.mode,
                    "ignore_observation",
                    "camera pose is unavailable",
                )
            )
        else:
            result = self.coordinator.coordinate(clock, None)

        return self._publish(clock, result, detections)

    def tick(self, now_s: float | None = None) -> PipelineCycle:
        """Advance prediction, reacquisition, reset, and submission timers."""

        clock = time.monotonic() if now_s is None else float(now_s)
        return self._publish(clock, self.coordinator.coordinate(clock, None), ())

    def _publish(
        self,
        now_s: float,
        result: CoordinationResult,
        detections: tuple[Detection, ...],
    ) -> PipelineCycle:
        if self.on_result is not None:
            self.on_result(result)

        for intent in result.intents:
            self.executor.execute(intent)

        submitted = None
        estimate = result.track_estimate
        cadence_due = (
            self._last_submission_s is None
            or now_s - self._last_submission_s >= self.submission_interval_s
        )
        if estimate is not None and estimate.submit_eligible and cadence_due:
            self.track_sink.submit(estimate)
            self._last_submission_s = now_s
            submitted = estimate
            if self.on_submission is not None:
                self.on_submission(estimate)

        if result.recommendation.mode is SearchMode.SEARCH and estimate is None:
            self._last_submission_s = None
        return PipelineCycle(result, detections, submitted)
