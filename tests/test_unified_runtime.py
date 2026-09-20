from __future__ import annotations

import unittest
from datetime import datetime, timedelta, timezone
from unittest.mock import Mock

from whiteout.camera import CameraFrame
from whiteout.config import AppConfig, CourseBounds, TowerMotionConfig, TrackApiConfig
from whiteout.coordinator_cli import validate_live_config
from whiteout.models import ControlIntent, Detection, GeoEstimate, Pixel, SearchMode, Telemetry
from whiteout.objects import Copter
from whiteout.runtime import (
    RateLimitedExecutor,
    ManagedTower,
    TelemetryBuffer,
    UnifiedCoordinatorRuntime,
    _course_scan_pans,
    _resolve_tower_poses,
)
from whiteout.tower import FORT_ROSS_TOWERS


class UnifiedRuntimeTests(unittest.TestCase):
    def test_telemetry_buffer_matches_nearest_frame_and_rejects_skew(self) -> None:
        start = datetime(2026, 1, 1, tzinfo=timezone.utc)
        buffer = TelemetryBuffer()
        older = Telemetry("quadcopter", 72, -95, 100, timestamp=start)
        newer = Telemetry(
            "quadcopter", 72, -95, 100,
            timestamp=start + timedelta(milliseconds=200),
        )
        buffer.append(older)
        buffer.append(newer)
        self.assertIs(buffer.nearest(start + timedelta(milliseconds=180), 0.25), newer)
        self.assertIsNone(buffer.nearest(start + timedelta(seconds=1), 0.25))

    def test_executor_coalesces_commands_and_honours_operator_override(self) -> None:
        delegate = Mock()
        telemetry = Telemetry(
            "quadcopter", 72, -95, 100,
            timestamp=datetime.now(timezone.utc), relative_altitude_m=90,
        )
        times = iter((1.0, 1.2))
        executor = RateLimitedExecutor(
            delegate, lambda _asset: telemetry,
            interval_s=1.0, stale_s=2.0, clock=lambda: next(times),
        )
        executor.enabled_assets.add("quadcopter")
        intent = ControlIntent("quadcopter", "divert", GeoEstimate(72, -95, 2))
        executor.execute(intent)
        executor.execute(intent)
        executor.operator_overrides.add("quadcopter")
        executor.execute(intent)
        delegate.execute.assert_called_once_with(intent)

    def test_global_guided_command_contains_target_and_preserved_altitude(self) -> None:
        self.assertEqual(
            Copter().guided(71.99, -94.84, 90.0).render(),
            "guided 71.99 -94.84 90",
        )

    def test_submission_and_course_preflight(self) -> None:
        bounds = CourseBounds(71.98, -94.93, 72.01, -94.74)
        validate_live_config(AppConfig(course_bounds=bounds), submit_tracks=False)
        with self.assertRaisesRegex(ValueError, "course_bounds"):
            validate_live_config(AppConfig(), submit_tracks=False)
        with self.assertRaisesRegex(ValueError, "allow_submission"):
            validate_live_config(AppConfig(course_bounds=bounds), submit_tracks=True)
        with self.assertRaisesRegex(ValueError, "absolute HTTP"):
            validate_live_config(
                AppConfig(
                    course_bounds=bounds,
                    track_api=TrackApiConfig(True, "relative/path"),
                ),
                submit_tracks=True,
            )

    def test_tower_scan_is_reachable_course_facing_and_overlapping(self) -> None:
        bounds = CourseBounds(71.984, -94.921, 72.001, -94.745)
        for tower in FORT_ROSS_TOWERS:
            pans = _course_scan_pans(tower, bounds, 0.20)
            self.assertTrue(pans)
            self.assertTrue(all(-144 <= pan <= 144 for pan in pans))
            self.assertTrue(all(right - left <= 48.000001 for left, right in zip(pans, pans[1:])))

    def test_tower_motion_clamps_targets_and_slews_smoothly(self) -> None:
        controller = Mock()
        tower = ManagedTower(
            controller,
            TowerMotionConfig(
                "tower-1",
                pan_min_deg=-30,
                pan_max_deg=30,
                tilt_min_deg=-10,
                tilt_max_deg=20,
                pan_rate_deg_s=10,
                tilt_rate_deg_s=5,
                command_hz=10,
            ),
        )
        tower.initialize_center(now_s=0.0)
        controller.reset_mock()
        tower.pan(90)
        tower.tilt(-30)

        self.assertTrue(tower.advance(0.1))
        controller.pan.assert_called_once_with(1.0)
        controller.tilt.assert_called_once_with(7.0)
        self.assertEqual(tower.target_pan_deg, 30)
        self.assertEqual(tower.target_tilt_deg, -10)
        self.assertFalse(tower.at_target())

        tower.advance(10.0)
        self.assertEqual(tower.commanded_pan_deg, 30)
        self.assertEqual(tower.commanded_tilt_deg, -10)
        self.assertTrue(tower.at_target())
        self.assertEqual(tower.target_reached_at(), 10.0)

    def test_tower_scan_respects_configured_soft_pan_limits(self) -> None:
        pans = _course_scan_pans(FORT_ROSS_TOWERS[0], None, 0.20, -100, 80)
        self.assertTrue(all(-100 <= pan <= 80 for pan in pans))
        self.assertEqual(pans[0], -100)
        self.assertEqual(pans[-1], 80)

    def test_tower_detection_cancels_scan_target_while_awaiting_settled_pose(self) -> None:
        class Detector:
            def detect(self, frame: CameraFrame):
                return (
                    Detection("tower-1", Pixel(640, 360), 0.9, frame.timestamp),
                )

        controllers = {name: Mock() for name in ("quadcopter", "fixed-wing", "tower-1", "tower-2")}
        runtime = UnifiedCoordinatorRuntime(
            AppConfig(course_bounds=CourseBounds(70, -100, 75, -90)),
            detector=Detector(),
            controllers=controllers,
            telemetry_readers={},
            tower_poses=FORT_ROSS_TOWERS,
            submit_tracks=False,
            operator="tester",
            output=Mock(),
        )
        tower = runtime.towers["tower-1"]
        tower.initialize_center(now_s=0.0)
        tower.pan(60)
        tower.advance(0.1)
        self.assertNotEqual(tower.commanded_pan_deg, tower.target_pan_deg)

        frame = CameraFrame("tower-1", b"jpeg", datetime.now(timezone.utc))
        runtime._detect(runtime._capture_frame_state(frame))

        self.assertEqual(tower.target_pan_deg, tower.commanded_pan_deg)
        self.assertGreater(runtime._tower_detection_hold_until["tower-1"], 0)
        held_target = tower.target_pan_deg
        runtime.executor.enabled_assets.add("tower-1")
        runtime._scan_towers(runtime._tower_detection_hold_until["tower-1"] - 0.1)
        self.assertEqual(tower.target_pan_deg, held_target)

    def test_frame_state_is_frozen_before_inference(self) -> None:
        class Detector:
            def detect(self, _frame: CameraFrame):
                return ()

        controllers = {name: Mock() for name in ("quadcopter", "fixed-wing", "tower-1", "tower-2")}
        runtime = UnifiedCoordinatorRuntime(
            AppConfig(course_bounds=CourseBounds(70, -100, 75, -90)),
            detector=Detector(),
            controllers=controllers,
            telemetry_readers={},
            tower_poses=FORT_ROSS_TOWERS,
            submit_tracks=False,
            operator="tester",
        )
        observed = datetime.now(timezone.utc)
        old = Telemetry(
            "quadcopter", 72, -95, 100,
            roll_rad=0, pitch_rad=0, yaw_rad=0,
            timestamp=observed, attitude_timestamp=observed,
        )
        runtime.telemetry["quadcopter"].append(old)
        state = runtime._capture_frame_state(CameraFrame("quadcopter", b"jpeg", observed))
        newer_time = observed + timedelta(milliseconds=100)
        runtime.telemetry["quadcopter"].append(Telemetry(
            "quadcopter", 73, -96, 200,
            roll_rad=0.1, pitch_rad=0.2, yaw_rad=0.3,
            timestamp=newer_time, attitude_timestamp=newer_time,
        ))

        self.assertIs(state.telemetry, old)
        self.assertIsNotNone(state.pose)
        assert state.pose is not None
        self.assertEqual(state.pose.latitude, 72)
        self.assertEqual(state.pose.altitude_m, 100)

    def test_empty_or_partial_metadata_uses_explicit_tower_poses(self) -> None:
        self.assertIs(_resolve_tower_poses((), FORT_ROSS_TOWERS), FORT_ROSS_TOWERS)
        partial = (FORT_ROSS_TOWERS[0],)
        self.assertIs(_resolve_tower_poses(partial, FORT_ROSS_TOWERS), FORT_ROSS_TOWERS)
        self.assertIs(_resolve_tower_poses(FORT_ROSS_TOWERS, ()), FORT_ROSS_TOWERS)
        with self.assertRaisesRegex(RuntimeError, "no complete tower_poses"):
            _resolve_tower_poses(partial, ())

    def test_fake_runtime_detection_diversion_confirmation_and_reset(self) -> None:
        class Detector:
            def detect(self, _frame: CameraFrame):
                return []

        controllers = {name: Mock() for name in ("quadcopter", "fixed-wing", "tower-1", "tower-2")}
        config = AppConfig(course_bounds=CourseBounds(70, -100, 75, -90))
        runtime = UnifiedCoordinatorRuntime(
            config,
            detector=Detector(),
            controllers=controllers,
            telemetry_readers={},
            tower_poses=FORT_ROSS_TOWERS,
            submit_tracks=False,
            operator="tester",
        )
        with self.assertRaises(PermissionError):
            runtime.launch_quadcopter(90)
        runtime.activate_operator_session()
        runtime.executor.enabled_assets.add("quadcopter")
        timestamp = datetime.now(timezone.utc)
        for offset in (0.0, 0.1):
            observed = timestamp + timedelta(seconds=offset)
            telemetry = Telemetry(
                "quadcopter", 72, -95, 100,
                roll_rad=0, pitch_rad=0, yaw_rad=0,
                timestamp=observed, attitude_timestamp=observed,
                relative_altitude_m=90, mission_sequence=4,
            )
            runtime.telemetry["quadcopter"].append(telemetry)
            frame = CameraFrame("quadcopter", b"jpeg", observed)
            detection = Detection(
                "quadcopter", Pixel(480, 360), 0.9, observed,
                metadata={"waterline": (480, 360)},
            )
            runtime._detected_queue.put((runtime._capture_frame_state(frame), (detection,)))
            runtime._drain_detected()

        self.assertEqual(runtime.pipeline.coordinator.mode, SearchMode.TRACK)
        controllers["quadcopter"].goto_global.assert_called()
        baseline = runtime.pipeline.coordinator._last_seen_s
        assert baseline is not None
        runtime.pipeline.tick(baseline + 15.0)
        controllers["quadcopter"].set_mission_current.assert_called_with(4)
        controllers["quadcopter"].resume_search.assert_called()
        runtime.close(request_rtl=False)


if __name__ == "__main__":
    unittest.main()
