from __future__ import annotations

import unittest
from datetime import datetime, timedelta, timezone

from whiteout.camera import CameraFrame
from whiteout.models import Telemetry
from whiteout.observation import ObservationMetadata, SynchronizationStatus


class ObservationMetadataTests(unittest.TestCase):
    def setUp(self) -> None:
        self.timestamp = datetime(2026, 1, 1, tzinfo=timezone.utc)
        self.frame = CameraFrame("fixed-wing-nadir", b"jpeg", self.timestamp)

    def test_metadata_ties_frame_pose_mission_and_waypoint(self) -> None:
        telemetry = Telemetry(
            "fixed-wing", 71.9, -94.8, 120, 0.1, 0.2, 0.3,
            self.timestamp, self.timestamp,
        )
        metadata = ObservationMetadata.for_frame(
            self.frame,
            asset="fixed-wing",
            telemetry=telemetry,
            mission="missions/fixed_wing_search.waypoints",
            waypoint_sequence=2,
        )
        self.assertEqual(metadata.camera, "fixed-wing-nadir")
        self.assertEqual(metadata.asset, "fixed-wing")
        self.assertEqual(metadata.telemetry.vehicle, metadata.asset)
        self.assertEqual(metadata.synchronization, SynchronizationStatus.SYNCHRONIZED)
        self.assertEqual(metadata.waypoint_sequence, 2)

    def test_missing_stale_and_unsynchronized_attitude_are_explicit(self) -> None:
        missing = Telemetry("fixed-wing", 71.9, -94.8, 120, timestamp=self.timestamp)
        self.assertEqual(
            ObservationMetadata.for_frame(
                self.frame, asset="fixed-wing", telemetry=missing
            ).synchronization,
            SynchronizationStatus.ATTITUDE_MISSING,
        )
        stale = Telemetry(
            "fixed-wing", 71.9, -94.8, 120, 0.0, 0.0, 0.0,
            self.timestamp - timedelta(seconds=1), self.timestamp,
        )
        self.assertEqual(
            ObservationMetadata.for_frame(
                self.frame, asset="fixed-wing", telemetry=stale
            ).synchronization,
            SynchronizationStatus.TELEMETRY_STALE,
        )
        attitude_old = Telemetry(
            "fixed-wing", 71.9, -94.8, 120, 0.0, 0.0, 0.0,
            self.timestamp, self.timestamp - timedelta(seconds=1),
        )
        self.assertEqual(
            ObservationMetadata.for_frame(
                self.frame, asset="fixed-wing", telemetry=attitude_old
            ).synchronization,
            SynchronizationStatus.ATTITUDE_UNSYNCHRONIZED,
        )

    def test_tower_pan_and_tilt_are_recorded(self) -> None:
        metadata = ObservationMetadata.for_frame(
            CameraFrame("tower-1", b"jpeg", self.timestamp),
            asset="tower-1",
            tower_pan_pwm=1400,
            tower_tilt_pwm=1650,
        )
        self.assertEqual((metadata.tower_pan_pwm, metadata.tower_tilt_pwm), (1400, 1650))


if __name__ == "__main__":
    unittest.main()
