from __future__ import annotations

import math
import unittest
from datetime import datetime, timedelta, timezone

from whiteout.geolocation import CameraIntrinsics, estimate_flat_water
from whiteout.models import Pixel, Telemetry
from whiteout.pose import aircraft_camera_pose, tower_camera_pose
from whiteout.tower import FORT_ROSS_TOWERS, TowerCalibration, TowerOrientation


class CameraPoseTests(unittest.TestCase):
    def test_fixed_wing_forward_camera_intersects_ahead(self) -> None:
        now = datetime.now(timezone.utc)
        telemetry = Telemetry(
            "fixed-wing", 72.0, -95.0, 100.0, 0.0, 0.0, 0.0, now, now
        )
        pose = aircraft_camera_pose(telemetry, camera_down_deg=8.0)
        estimate = estimate_flat_water(
            Pixel(50, 50), CameraIntrinsics(100, 100, 50, 50), pose
        )
        self.assertGreater(estimate.latitude, telemetry.latitude)
        self.assertAlmostEqual(estimate.longitude, telemetry.longitude)
        expected_forward_m = 100.0 / math.tan(math.radians(8.0))
        actual_forward_m = (estimate.latitude - telemetry.latitude) * 111_319.490793
        self.assertAlmostEqual(actual_forward_m, expected_forward_m, delta=1.0)

    def test_aircraft_yaw_rotates_forward_intersection_east(self) -> None:
        now = datetime.now(timezone.utc)
        telemetry = Telemetry(
            "quadcopter",
            72.0,
            -95.0,
            100.0,
            0.0,
            0.0,
            math.pi / 2,
            now,
            now,
        )
        pose = aircraft_camera_pose(telemetry, camera_down_deg=20.0)
        estimate = estimate_flat_water(
            Pixel(50, 50), CameraIntrinsics(100, 100, 50, 50), pose
        )
        self.assertAlmostEqual(estimate.latitude, telemetry.latitude, places=6)
        self.assertGreater(estimate.longitude, telemetry.longitude)

    def test_tower_pose_uses_calibrated_world_position_and_settled_orientation(self) -> None:
        now = datetime.now(timezone.utc)
        self.assertEqual(
            (
                FORT_ROSS_TOWERS[0].latitude,
                FORT_ROSS_TOWERS[0].longitude,
                FORT_ROSS_TOWERS[1].latitude,
                FORT_ROSS_TOWERS[1].longitude,
            ),
            (
                71.99912183839884,
                -94.81086504031542,
                71.97880305156609,
                -94.88346035301377,
            ),
        )
        calibration = TowerCalibration()
        orientation = TowerOrientation(
            pan_deg=calibration.target_pan(0.0),
            tilt_deg=10.0,
            timestamp=now,
            commanded_pan_deg=calibration.target_pan(0.0),
            commanded_tilt_deg=10.0,
            angular_rate_dps=0.0,
            settled_since=now - timedelta(seconds=1),
        )
        pose = tower_camera_pose(FORT_ROSS_TOWERS[0], orientation)
        estimate = estimate_flat_water(
            Pixel(50, 50), CameraIntrinsics(100, 100, 50, 50), pose
        )
        self.assertEqual(pose.altitude_m, 6.62)
        self.assertGreater(estimate.latitude, FORT_ROSS_TOWERS[0].latitude)
        self.assertAlmostEqual(estimate.longitude, FORT_ROSS_TOWERS[0].longitude)

    def test_tower_pose_rejects_moving_or_unhealthy_orientation(self) -> None:
        now = datetime.now(timezone.utc)
        moving = TowerOrientation(
            0.0,
            10.0,
            now,
            commanded_pan_deg=0.0,
            commanded_tilt_deg=10.0,
            angular_rate_dps=2.0,
            settled_since=now - timedelta(seconds=1),
        )
        with self.assertRaises(ValueError):
            tower_camera_pose(FORT_ROSS_TOWERS[0], moving)


if __name__ == "__main__":
    unittest.main()
