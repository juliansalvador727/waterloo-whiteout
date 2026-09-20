from __future__ import annotations

import math
import unittest
from datetime import datetime, timezone
from unittest.mock import Mock

from whiteout.control import CopterMode
from whiteout.live import SimulatorActuator, TrackApiSubmitter, _local_offset_m
from whiteout.models import ControlIntent, GeoEstimate, Telemetry, Track, TrackEstimate


class SimulatorActuatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.copter = Mock()
        self.tower = Mock()
        self.telemetry = Telemetry(
            "quadcopter", 72.0, -95.0, 100.0,
            relative_altitude_m=90.0, mission_sequence=7,
        )
        self.actuator = SimulatorActuator(
            self.copter,
            {"tower-1": self.tower},
            lambda name: self.telemetry if name == "quadcopter" else None,
        )

    def test_copter_uses_forward_camera_standoff_and_preserves_altitude(self) -> None:
        target = GeoEstimate(72.001, -94.999, 2.0)
        self.actuator(ControlIntent("quadcopter", "divert", target))
        self.copter.set_mode.assert_called_once_with(CopterMode.GUIDED)
        latitude, longitude, altitude = self.copter.goto_global.call_args.args
        self.assertEqual(altitude, 90.0)
        self.assertNotEqual((latitude, longitude), (target.latitude, target.longitude))
        north_m, east_m = _local_offset_m(
            latitude, longitude, target.latitude, target.longitude
        )
        expected_standoff_m = self.telemetry.altitude_m / math.tan(math.radians(20.0))
        self.assertAlmostEqual(math.hypot(north_m, east_m), expected_standoff_m, delta=0.5)
        heading_deg, yaw_rate = self.copter.set_yaw.call_args.args
        self.assertAlmostEqual(
            heading_deg,
            math.degrees(math.atan2(east_m, north_m)) % 360.0,
            places=2,
        )
        self.assertEqual(yaw_rate, 20.0)

        self.actuator(ControlIntent("quadcopter", "resume_search"))
        self.copter.set_mission_current.assert_called_once_with(7)
        self.copter.resume_search.assert_called_once_with()

    def test_resume_and_tower_cue_use_typed_controllers(self) -> None:
        self.actuator(ControlIntent("quadcopter", "resume_search"))
        self.copter.resume_search.assert_called_once_with()

        self.actuator(
            ControlIntent("tower-1", "cue", pan_deg=-20.0, tilt_deg=5.0)
        )
        self.tower.pan.assert_called_once_with(-20.0)
        self.tower.tilt.assert_called_once_with(5.0)

    def test_missing_telemetry_and_unsupported_assets_fail_closed(self) -> None:
        actuator = SimulatorActuator(self.copter, {}, lambda name: None)
        target = GeoEstimate(72.001, -94.999, 2.0)
        with self.assertRaises(RuntimeError):
            actuator(ControlIntent("quadcopter", "divert", target))
        with self.assertRaises(ValueError):
            actuator(ControlIntent("fixed-wing", "divert", target))

    def test_copter_uses_alternate_standoff_when_preferred_point_is_outside_course(self) -> None:
        target = GeoEstimate(72.001, -94.999, 2.0)
        unrestricted = SimulatorActuator(
            self.copter,
            {},
            lambda _name: self.telemetry,
        )
        unrestricted(ControlIntent("quadcopter", "divert", target))
        preferred = self.copter.goto_global.call_args.args[:2]
        self.copter.reset_mock()
        bounded = SimulatorActuator(
            self.copter,
            {},
            lambda _name: self.telemetry,
            lambda latitude, longitude: (latitude, longitude) != preferred,
        )
        bounded(ControlIntent("quadcopter", "divert", target))
        alternate = self.copter.goto_global.call_args.args[:2]
        self.assertNotEqual(alternate, preferred)


class TrackApiSubmitterTests(unittest.TestCase):
    def test_submitter_forwards_track_with_explicit_confirmation(self) -> None:
        client = Mock()
        track = Track(
            "target-1",
            72.0,
            -95.0,
            0.0,
            0.0,
            2.0,
            datetime(2026, 1, 1, tzinfo=timezone.utc),
        )
        TrackApiSubmitter(client)(TrackEstimate(track))
        client.submit.assert_called_once_with(track, confirmed=True)


if __name__ == "__main__":
    unittest.main()
