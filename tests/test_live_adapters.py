from __future__ import annotations

import unittest
from datetime import datetime, timezone
from unittest.mock import Mock

from whiteout.control import CopterMode
from whiteout.live import SimulatorActuator, TrackApiSubmitter
from whiteout.models import ControlIntent, GeoEstimate, Telemetry, Track, TrackEstimate


class SimulatorActuatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.copter = Mock()
        self.tower = Mock()
        self.telemetry = Telemetry("quadcopter", 72.0, -95.0, 100.0)
        self.actuator = SimulatorActuator(
            self.copter,
            {"tower-1": self.tower},
            lambda name: self.telemetry if name == "quadcopter" else None,
        )

    def test_copter_target_becomes_guided_local_position(self) -> None:
        target = GeoEstimate(72.001, -94.999, 2.0)
        self.actuator(ControlIntent("quadcopter", "divert", target))
        self.copter.set_mode.assert_called_once_with(CopterMode.GUIDED)
        north, east, down = self.copter.set_position.call_args.args
        self.assertAlmostEqual(north, 111.319, places=2)
        self.assertAlmostEqual(east, 34.398, places=2)
        self.assertEqual(down, 0.0)

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
