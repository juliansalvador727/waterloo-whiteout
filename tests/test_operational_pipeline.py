from __future__ import annotations

import math
import unittest
from datetime import datetime, timedelta, timezone
from unittest.mock import patch

from whiteout.camera import (
    FIXED_WING_CAMERA,
    QUAD_CAMERA,
    TOWER_CAMERA,
    CameraFrame,
    camera_model,
)
from whiteout.coordinator import Coordinator
from whiteout.execution import (
    LiveExecutor,
    LiveTrackSink,
    OperatorSession,
    RecordingExecutor,
    RecordingTrackSink,
)
from whiteout.geolocation import CameraIntrinsics, CameraPose, estimate_flat_water
from whiteout.models import (
    ControlIntent,
    Detection,
    GeoEstimate,
    Pixel,
    SearchMode,
    Telemetry,
    Track,
    TrackEstimate,
)
from whiteout.observation import ObservationMetadata, SynchronizationStatus
from whiteout.tower import TowerCalibration, TowerOrientation, TowerWorldPose
from whiteout.tower import (
    MAX_TOWER_SCAN_HORIZONTAL_STEP_DEG,
    MAX_TOWER_SCAN_VERTICAL_STEP_DEG,
    tower_scan_overlap_steps,
    validate_tower_scan_steps,
)
from whiteout.tracker import ConstantVelocityTracker


class CameraGeometryTests(unittest.TestCase):
    def test_fixed_presets_and_derived_intrinsics(self) -> None:
        self.assertEqual((QUAD_CAMERA.width_px, QUAD_CAMERA.height_px), (640, 480))
        self.assertEqual((FIXED_WING_CAMERA.horizontal_fov_deg, FIXED_WING_CAMERA.vertical_fov_deg), (69, 42.6))
        self.assertIs(camera_model("tower-2"), TOWER_CAMERA)
        self.assertEqual(
            (QUAD_CAMERA.intrinsics.cx_px, QUAD_CAMERA.intrinsics.cy_px),
            (319.5, 239.5),
        )
        self.assertEqual(
            (FIXED_WING_CAMERA.intrinsics.cx_px, FIXED_WING_CAMERA.intrinsics.cy_px),
            (319.5, 179.5),
        )
        self.assertAlmostEqual(
            QUAD_CAMERA.intrinsics.fx_px,
            639 / (2 * math.tan(math.radians(114.6) / 2)),
        )

    def test_boundary_pixel_rays_match_half_fields_of_view(self) -> None:
        for model in (QUAD_CAMERA, FIXED_WING_CAMERA, TOWER_CAMERA):
            intrinsics = model.intrinsics
            left = math.degrees(math.atan((0 - intrinsics.cx_px) / intrinsics.fx_px))
            right = math.degrees(
                math.atan((model.width_px - 1 - intrinsics.cx_px) / intrinsics.fx_px)
            )
            top = math.degrees(math.atan((0 - intrinsics.cy_px) / intrinsics.fy_px))
            bottom = math.degrees(
                math.atan((model.height_px - 1 - intrinsics.cy_px) / intrinsics.fy_px)
            )
            self.assertAlmostEqual(left, -model.horizontal_fov_deg / 2)
            self.assertAlmostEqual(right, model.horizontal_fov_deg / 2)
            self.assertAlmostEqual(top, -model.vertical_fov_deg / 2)
            self.assertAlmostEqual(bottom, model.vertical_fov_deg / 2)

    def test_footprint_and_twenty_percent_overlap(self) -> None:
        width, height = FIXED_WING_CAMERA.footprint_m(100)
        spacing = FIXED_WING_CAMERA.overlap_20_spacing_m(100)
        self.assertAlmostEqual(spacing[0], width * 0.8)
        self.assertAlmostEqual(spacing[1], height * 0.8)
        for altitude in (0, -100):
            with self.assertRaises(ValueError):
                FIXED_WING_CAMERA.footprint_m(altitude)

    def test_geolocation_preserves_time_and_requires_positive_world_z(self) -> None:
        observed = datetime(2026, 1, 1, tzinfo=timezone.utc)
        intrinsics = CameraIntrinsics(100, 100, 50, 50)
        estimate = estimate_flat_water(
            Pixel(50, 50), intrinsics, CameraPose(72, -95, 100), observed
        )
        self.assertEqual(estimate.timestamp, observed)
        self.assertEqual((estimate.latitude, estimate.longitude), (72, -95))
        with self.assertRaises(ValueError):
            estimate_flat_water(
                Pixel(50, 50), intrinsics, CameraPose(72, -95, -100), observed
            )


class TowerGeometryTests(unittest.TestCase):
    def test_fort_ross_heading_pan_and_compass(self) -> None:
        calibration = TowerCalibration()
        self.assertAlmostEqual(calibration.base_true_heading_deg, 40.1952)
        self.assertAlmostEqual(calibration.true_heading(-10), 50.1952)
        self.assertAlmostEqual(calibration.target_pan(50.1952), -10)
        calibrated = calibration.with_compass_reference(35.0)
        self.assertAlmostEqual(calibrated.calibrated_compass(35.0), 40.1952)

    def test_heading_and_target_pan_signs_and_wrap_boundaries(self) -> None:
        calibration = TowerCalibration(base_true_heading_deg=10.0)
        # true_heading = wrap(base_true_heading - relative_pan)
        self.assertEqual(calibration.true_heading(20.0), 350.0)
        self.assertEqual(calibration.true_heading(-20.0), 30.0)
        self.assertEqual(calibration.true_heading(-350.0), 0.0)
        # target_pan = wrap(base_true_heading - target_bearing)
        self.assertEqual(calibration.target_pan(20.0), -10.0)
        self.assertEqual(calibration.target_pan(0.0), 10.0)
        self.assertEqual(calibration.target_pan(350.0), 20.0)
        self.assertEqual(calibration.target_pan(190.0), -180.0)
        self.assertEqual(calibration.target_pan(-170.0), -180.0)

    def test_limits_settled_health_and_no_gyro_integration(self) -> None:
        now = datetime.now(timezone.utc)
        orientation = TowerOrientation(
            pan_deg=10,
            tilt_deg=5,
            timestamp=now,
            commanded_pan_deg=10.2,
            commanded_tilt_deg=4.8,
            angular_rate_dps=0.5,
            settled_since=now - timedelta(seconds=0.5),
        )
        self.assertTrue(orientation.settled)
        self.assertTrue(orientation.is_healthy(now + timedelta(seconds=1)))
        self.assertFalse(
            TowerOrientation(
                pan_deg=10,
                tilt_deg=5,
                timestamp=now,
                commanded_pan_deg=10,
                commanded_tilt_deg=5,
                angular_rate_dps=1.0,
                settled_since=now - timedelta(seconds=1),
            ).settled
        )
        self.assertFalse(
            TowerOrientation(
                pan_deg=10,
                tilt_deg=5,
                timestamp=now,
                commanded_pan_deg=10,
                commanded_tilt_deg=5,
                angular_rate_dps=0.1,
                settled_since=now - timedelta(seconds=0.499),
            ).settled
        )
        replaced = orientation.with_telemetry(
            pan_deg=10,
            tilt_deg=5,
            timestamp=now + timedelta(seconds=1),
            gyro_z_dps=100,
        )
        self.assertEqual(replaced.pan_deg, 10)
        self.assertEqual(replaced.angular_rate_dps, 100)
        with self.assertRaises(ValueError):
            TowerOrientation(145, 0, now)

    def test_compass_drift_health_uses_wrapped_angular_difference(self) -> None:
        now = datetime.now(timezone.utc)
        calibration = TowerCalibration(
            base_true_heading_deg=359.0,
            compass_drift_tolerance_deg=3.0,
        )
        within_tolerance = TowerOrientation(
            pan_deg=0.0,
            tilt_deg=0.0,
            timestamp=now,
            compass_heading_deg=1.0,
            calibration=calibration,
        )
        outside_tolerance = TowerOrientation(
            pan_deg=0.0,
            tilt_deg=0.0,
            timestamp=now,
            compass_heading_deg=3.0,
            calibration=calibration,
        )
        self.assertTrue(within_tolerance.is_healthy(now + timedelta(seconds=1)))
        self.assertFalse(outside_tolerance.is_healthy(now + timedelta(seconds=1)))

    def test_head_mounted_compass_tracks_expected_nonzero_pan_heading(self) -> None:
        now = datetime.now(timezone.utc)
        calibration = TowerCalibration().with_compass_reference(35.0)
        expected_heading = calibration.true_heading(10.0)
        raw_at_pan = expected_heading - calibration.compass_bias_deg
        healthy = TowerOrientation(
            pan_deg=10.0,
            tilt_deg=0.0,
            timestamp=now,
            compass_heading_deg=raw_at_pan,
            calibration=calibration,
        )
        drifting = TowerOrientation(
            pan_deg=10.0,
            tilt_deg=0.0,
            timestamp=now,
            compass_heading_deg=raw_at_pan + calibration.compass_drift_tolerance_deg + 0.1,
            calibration=calibration,
        )
        self.assertAlmostEqual(healthy.true_heading_deg, expected_heading)
        self.assertTrue(healthy.is_healthy(now + timedelta(seconds=1)))
        self.assertFalse(drifting.is_healthy(now + timedelta(seconds=1)))

    def test_tower_scan_overlap_steps_enforce_maximum_spacing(self) -> None:
        self.assertEqual(
            tower_scan_overlap_steps(),
            (
                MAX_TOWER_SCAN_HORIZONTAL_STEP_DEG,
                MAX_TOWER_SCAN_VERTICAL_STEP_DEG,
            ),
        )
        self.assertEqual(validate_tower_scan_steps(48, 28.88), (48, 28.88))
        with self.assertRaises(ValueError):
            validate_tower_scan_steps(48.01, 28.88)
        with self.assertRaises(ValueError):
            tower_scan_overlap_steps(horizontal_overlap=0.19)


class EffectsBoundaryTests(unittest.TestCase):
    def setUp(self) -> None:
        now = datetime(2026, 1, 1, tzinfo=timezone.utc)
        track = Track("t", 72, -95, 0, 0, 2, now)
        self.estimate = TrackEstimate(track)
        self.intent = ControlIntent("quadcopter", "divert")

    def test_recording_variants_only_record(self) -> None:
        executor = RecordingExecutor()
        sink = RecordingTrackSink()
        with (
            patch("urllib.request.urlopen") as urlopen,
            patch("subprocess.Popen") as popen,
            patch("whiteout.mavproxy.MavProxySession.start") as mavproxy_start,
        ):
            executor.execute(self.intent)
            sink.submit(self.estimate)
        self.assertEqual(executor.intents, [self.intent])
        self.assertEqual(sink.estimates, [self.estimate])
        urlopen.assert_not_called()
        popen.assert_not_called()
        mavproxy_start.assert_not_called()

    def test_live_variants_require_both_gates(self) -> None:
        actuated: list[ControlIntent] = []
        submitted: list[TrackEstimate] = []
        with self.assertRaises(PermissionError):
            LiveExecutor(actuated.append).execute(self.intent)
        with self.assertRaises(PermissionError):
            LiveTrackSink(submitted.append, live_enabled=True).submit(self.estimate)
        with self.assertRaises(PermissionError):
            LiveExecutor(
                actuated.append,
                live_enabled=True,
                operator_session=True,  # type: ignore[arg-type]
            ).execute(self.intent)
        session = OperatorSession("operator", active=True)
        LiveExecutor(actuated.append, live_enabled=True, operator_session=session).execute(self.intent)
        LiveTrackSink(submitted.append, live_enabled=True, operator_session=session).submit(self.estimate)
        self.assertEqual((actuated, submitted), ([self.intent], [self.estimate]))
        with self.assertRaises(PermissionError):
            LiveExecutor(actuated.append, live_enabled=True, operator_session=session).execute(
                ControlIntent("Fixed_Wing", "divert")
            )
        self.assertEqual(actuated, [self.intent])


class IntegratedCoordinatorTests(unittest.TestCase):
    def test_original_decide_confirmation_configuration_and_positional_mode(self) -> None:
        estimate = GeoEstimate(72, -95, 2)
        coordinator = Coordinator(1, 2, 10, SearchMode.SEARCH)
        self.assertEqual(coordinator.decide(0, estimate).mode, SearchMode.TRACK)

        integrated = Coordinator(confirmations_required=1)
        first = integrated.coordinate(0, estimate)
        self.assertTrue(first.track_estimate.provisional)
        self.assertFalse(first.track_estimate.confirmed)

    def test_provisional_confirmation_timeouts_and_no_plane_intent(self) -> None:
        start = datetime(2026, 1, 1, tzinfo=timezone.utc)
        coordinator = Coordinator(course_contains=lambda lat, lon: 71 < lat < 73)
        first = coordinator.coordinate(
            0,
            GeoEstimate(72, -95, 2, start),
            synchronized=True,
            source="tower-1",
        )
        self.assertTrue(first.track_estimate.submit_eligible)
        self.assertTrue(first.track_estimate.provisional)
        self.assertFalse(first.track_estimate.confirmed)
        self.assertEqual(
            {intent.asset for intent in first.intents},
            {"tower-1", "tower-2", "quadcopter"},
        )
        tower_1 = next(intent for intent in first.intents if intent.asset == "tower-1")
        tower_2 = next(intent for intent in first.intents if intent.asset == "tower-2")
        self.assertAlmostEqual(tower_1.pan_deg, 129.2444665)
        self.assertAlmostEqual(tower_1.tilt_deg, -0.0582903)
        self.assertAlmostEqual(tower_2.pan_deg, 99.6731519)
        self.assertAlmostEqual(tower_2.tilt_deg, -0.0815033)
        second = coordinator.coordinate(
            1, GeoEstimate(72.00001, -95, 2, start + timedelta(seconds=1)), source="quadcopter"
        )
        self.assertTrue(second.track_estimate.confirmed)
        self.assertNotIn("fixed-wing", {intent.asset for intent in second.intents})

        before_reacquire = coordinator.coordinate(2.999, None)
        self.assertEqual(before_reacquire.recommendation.mode.value, "TRACK")
        at_reacquire = coordinator.coordinate(3.0, None)
        self.assertEqual(at_reacquire.recommendation.mode.value, "REACQUIRE")
        self.assertNotIn("fixed-wing", {intent.asset for intent in at_reacquire.intents})
        self.assertIn("tower-1", {intent.asset for intent in at_reacquire.intents})
        self.assertIn("tower-2", {intent.asset for intent in at_reacquire.intents})

        before_submission_stop = coordinator.coordinate(5.999, None)
        self.assertTrue(before_submission_stop.track_estimate.submit_eligible)
        at_submission_stop = coordinator.coordinate(6.0, None)
        self.assertFalse(at_submission_stop.track_estimate.submit_eligible)

        before_search_reset = coordinator.coordinate(15.999, None)
        self.assertEqual(before_search_reset.recommendation.mode.value, "REACQUIRE")
        at_search_reset = coordinator.coordinate(16.0, None)
        self.assertEqual(at_search_reset.recommendation.mode.value, "SEARCH")
        self.assertEqual(
            [(intent.asset, intent.action) for intent in at_search_reset.intents],
            [("quadcopter", "resume_search")],
        )

    def test_unreachable_tower_is_omitted(self) -> None:
        result = Coordinator().coordinate(
            0,
            GeoEstimate(71.99912183839884, -94.81086504031542, 2),
        )
        self.assertNotIn("tower-1", {intent.asset for intent in result.intents})
        self.assertIn("quadcopter", {intent.asset for intent in result.intents})

    def test_generated_tower_poses_override_fallback_calibration(self) -> None:
        generated = (TowerWorldPose("tower-live", 72.0, -95.0, 10.0),)
        result = Coordinator(tower_poses=generated).coordinate(
            0,
            GeoEstimate(72.001, -95.0, 2),
        )
        self.assertEqual(
            {intent.asset for intent in result.intents},
            {"tower-live", "quadcopter"},
        )

    def test_course_and_tracker_compatibility_reject_observations(self) -> None:
        start = datetime(2026, 1, 1, tzinfo=timezone.utc)
        coordinator = Coordinator(course_contains=lambda lat, lon: lat < 73)
        self.assertEqual(
            coordinator.coordinate(0, GeoEstimate(74, -95, 2, start)).recommendation.action,
            "ignore_observation",
        )
        coordinator.coordinate(0, GeoEstimate(72, -95, 2, start))
        incompatible = coordinator.coordinate(
            1, GeoEstimate(73, -95, 0.1, start + timedelta(seconds=1))
        )
        self.assertEqual(incompatible.recommendation.action, "ignore_observation")

    def test_detection_uses_waterline_and_requires_synchronization(self) -> None:
        timestamp = datetime(2026, 1, 1, tzinfo=timezone.utc)
        frame = CameraFrame("quadcopter", b"jpeg", timestamp)
        telemetry = Telemetry(
            "quadcopter", 72, -95, 100, 0, 0, 0, timestamp, timestamp
        )
        metadata = ObservationMetadata.for_frame(
            frame, asset="quadcopter", telemetry=telemetry
        )
        detection = Detection(
            "quadcopter",
            Pixel(50, 50),
            0.9,
            timestamp,
            metadata={"waterline": (50, 60)},
        )
        coordinator = Coordinator()
        result = coordinator.process_detection(
            0,
            detection,
            metadata,
            CameraIntrinsics(100, 100, 50, 50),
            CameraPose(72, -95, 100),
        )
        self.assertEqual(result.track.last_update, timestamp)
        self.assertLess(result.track.latitude, 72)

        unsynchronized = ObservationMetadata(
            timestamp,
            "quadcopter",
            "quadcopter",
            None,
            SynchronizationStatus.TELEMETRY_MISSING,
        )
        ignored = Coordinator().process_detection(
            0,
            detection,
            unsynchronized,
            CameraIntrinsics(100, 100, 50, 50),
            CameraPose(72, -95, 0),
        )
        self.assertEqual(ignored.recommendation.action, "ignore_observation")

    def test_unsynchronized_detection_never_calls_geolocation(self) -> None:
        timestamp = datetime(2026, 1, 1, tzinfo=timezone.utc)
        metadata = ObservationMetadata(
            timestamp,
            "tower-1",
            "tower-1",
            None,
            SynchronizationStatus.TELEMETRY_MISSING,
        )
        detection = Detection("tower-1", Pixel(1, 1), 0.9, timestamp)
        with patch(
            "whiteout.coordinator.estimate_flat_water",
            side_effect=AssertionError("geolocation must not run"),
        ) as geolocate:
            result = Coordinator().process_detection(
                0,
                detection,
                metadata,
                CameraIntrinsics(-1, -1, 0, 0),
                CameraPose(72, -95, 0),
            )
        self.assertEqual(result.recommendation.action, "ignore_observation")
        geolocate.assert_not_called()

    def test_confidence_validation_and_zero_rejection_precede_geolocation(self) -> None:
        timestamp = datetime(2026, 1, 1, tzinfo=timezone.utc)
        for confidence in (-0.1, 1.1, math.nan, math.inf):
            with self.subTest(confidence=confidence), self.assertRaises(ValueError):
                Detection("tower-1", Pixel(1, 1), confidence, timestamp)

        metadata = ObservationMetadata(
            timestamp,
            "tower-1",
            "tower-1",
            None,
            SynchronizationStatus.SYNCHRONIZED,
        )
        zero = Detection("tower-1", Pixel(1, 1), 0.0, timestamp)
        with patch(
            "whiteout.coordinator.estimate_flat_water",
            side_effect=AssertionError("geolocation must not run"),
        ) as geolocate:
            result = Coordinator().process_detection(
                0,
                zero,
                metadata,
                CameraIntrinsics(-1, -1, 0, 0),
                CameraPose(72, -95, 0),
            )
        self.assertEqual(result.recommendation.action, "ignore_observation")
        geolocate.assert_not_called()


class KalmanRequirementsTests(unittest.TestCase):
    def test_uncertainty_propagates_and_gate_rejects_outlier(self) -> None:
        start = datetime(2026, 1, 1, tzinfo=timezone.utc)
        tracker = ConstantVelocityTracker()
        initial = tracker.update(GeoEstimate(72, -95, 1, start))
        predicted = tracker.predict(3)
        self.assertGreater(predicted.uncertainty_m, initial.uncertainty_m)
        rejected = tracker.update(GeoEstimate(73, -95, 0.1, start + timedelta(seconds=1)))
        self.assertEqual(rejected, initial)
        self.assertIn("gate", tracker.last_rejection_reason)

    def test_twenty_metres_per_second_limit_rejects_update(self) -> None:
        start = datetime(2026, 1, 1, tzinfo=timezone.utc)
        tracker = ConstantVelocityTracker(gate_threshold=math.inf)
        initial = tracker.update(GeoEstimate(72, -95, 1, start))
        thirty_metres_north = 72 + 30 / 111_319.490793
        rejected = tracker.update(
            GeoEstimate(thirty_metres_north, -95, 1, start + timedelta(seconds=1))
        )
        self.assertEqual(rejected, initial)
        self.assertIn("20 m/s", tracker.last_rejection_reason)
        self.assertNotIn("gate", tracker.last_rejection_reason)


if __name__ == "__main__":
    unittest.main()
