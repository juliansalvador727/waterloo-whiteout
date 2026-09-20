from __future__ import annotations

import unittest
from datetime import datetime, timedelta, timezone

from whiteout.camera import CameraFrame
from whiteout.coordinator import Coordinator
from whiteout.execution import RecordingExecutor, RecordingTrackSink
from whiteout.geolocation import CameraIntrinsics, CameraPose
from whiteout.models import Detection, Pixel, SearchMode, Telemetry
from whiteout.pipeline import OperationalPipeline


class _Detector:
    def __init__(self, detections: list[list[Detection]]) -> None:
        self._detections = iter(detections)

    def detect(self, frame: CameraFrame) -> list[Detection]:
        return next(self._detections, [])


class OperationalPipelineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.timestamp = datetime(2026, 1, 1, tzinfo=timezone.utc)
        self.frame = CameraFrame("quadcopter", b"jpeg", self.timestamp)
        self.telemetry = Telemetry(
            "quadcopter",
            72.0,
            -95.0,
            100.0,
            0.0,
            0.0,
            0.0,
            self.timestamp,
            self.timestamp,
        )
        self.intrinsics = CameraIntrinsics(100, 100, 50, 50)
        self.pose = CameraPose(72.0, -95.0, 100.0)

    def _detection(self, *, confidence: float = 0.9, offset_s: float = 0.0) -> Detection:
        return Detection(
            "quadcopter",
            Pixel(50, 50),
            confidence,
            self.timestamp + timedelta(seconds=offset_s),
            metadata={"waterline": (50, 50)},
        )

    def test_single_valid_detection_cues_assets_and_submits_provisional_track(self) -> None:
        executor = RecordingExecutor()
        sink = RecordingTrackSink()
        pipeline = OperationalPipeline(
            _Detector([[self._detection()]]),
            Coordinator(),
            executor,
            sink,
        )

        cycle = pipeline.process_frame(
            self.frame,
            telemetry=self.telemetry,
            intrinsics=self.intrinsics,
            pose=self.pose,
            now_s=0.0,
        )

        self.assertTrue(cycle.result.track_estimate.provisional)
        self.assertIs(cycle.submitted, cycle.result.track_estimate)
        self.assertEqual(sink.estimates, [cycle.result.track_estimate])
        self.assertIn("quadcopter", {intent.asset for intent in executor.intents})
        self.assertNotIn("fixed-wing", {intent.asset for intent in executor.intents})

    def test_two_boxes_in_one_frame_do_not_confirm_track(self) -> None:
        detection = self._detection()
        pipeline = OperationalPipeline(
            _Detector([[detection, detection]]),
            Coordinator(),
            RecordingExecutor(),
            RecordingTrackSink(),
        )
        cycle = pipeline.process_frame(
            self.frame,
            telemetry=self.telemetry,
            intrinsics=self.intrinsics,
            pose=self.pose,
            now_s=0.0,
        )
        self.assertTrue(cycle.result.track_estimate.provisional)

    def test_dropout_predicts_at_one_hz_then_stops_submission_and_resets(self) -> None:
        executor = RecordingExecutor()
        sink = RecordingTrackSink()
        pipeline = OperationalPipeline(
            _Detector([[self._detection()]]),
            Coordinator(),
            executor,
            sink,
        )
        pipeline.process_frame(
            self.frame,
            telemetry=self.telemetry,
            intrinsics=self.intrinsics,
            pose=self.pose,
            now_s=0.0,
        )

        self.assertIsNotNone(pipeline.tick(1.0).submitted)
        at_reacquire = pipeline.tick(2.0)
        self.assertEqual(at_reacquire.result.recommendation.mode, SearchMode.REACQUIRE)
        self.assertIsNotNone(at_reacquire.submitted)
        # Prediction uncertainty reaches the configured 95% limit before the
        # five-second age limit, so output stops at the earlier safety gate.
        self.assertIsNone(pipeline.tick(3.0).submitted)
        self.assertIsNone(pipeline.tick(5.0).submitted)
        reset = pipeline.tick(15.0)
        self.assertEqual(reset.result.recommendation.mode, SearchMode.SEARCH)
        self.assertIsNone(reset.result.track_estimate)
        self.assertEqual(executor.intents[-1].action, "resume_search")
        self.assertEqual(len(sink.estimates), 3)

    def test_unsynchronized_or_pose_less_detection_has_no_effects(self) -> None:
        executor = RecordingExecutor()
        sink = RecordingTrackSink()
        stale = Telemetry(
            "quadcopter",
            72.0,
            -95.0,
            100.0,
            0.0,
            0.0,
            0.0,
            self.timestamp - timedelta(seconds=1),
            self.timestamp - timedelta(seconds=1),
        )
        pipeline = OperationalPipeline(
            _Detector([[self._detection()], [self._detection()]]),
            Coordinator(),
            executor,
            sink,
        )
        stale_cycle = pipeline.process_frame(
            self.frame,
            telemetry=stale,
            intrinsics=self.intrinsics,
            pose=self.pose,
            now_s=0.0,
        )
        pose_less_cycle = pipeline.process_frame(
            self.frame,
            telemetry=self.telemetry,
            intrinsics=self.intrinsics,
            pose=None,
            now_s=1.0,
        )
        self.assertEqual(stale_cycle.result.recommendation.action, "ignore_observation")
        self.assertEqual(pose_less_cycle.result.recommendation.action, "ignore_observation")
        self.assertEqual(executor.intents, [])
        self.assertEqual(sink.estimates, [])

    def test_ray_that_misses_water_is_rejected_without_crashing(self) -> None:
        pipeline = OperationalPipeline(
            _Detector([[self._detection()]]),
            Coordinator(),
            RecordingExecutor(),
            RecordingTrackSink(),
        )
        cycle = pipeline.process_frame(
            self.frame,
            telemetry=self.telemetry,
            intrinsics=self.intrinsics,
            pose=CameraPose(72.0, -95.0, 100.0, pitch_rad=3.141592653589793),
            now_s=0.0,
        )
        self.assertEqual(cycle.result.recommendation.action, "ignore_observation")
        self.assertIn("geolocation rejected", cycle.result.recommendation.reason)


if __name__ == "__main__":
    unittest.main()
