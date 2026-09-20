from __future__ import annotations

import unittest

from whiteout.geolocation import CameraIntrinsics, CameraPose, estimate_flat_water
from whiteout.models import Pixel


class GeolocationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.intrinsics = CameraIntrinsics(1000.0, 1000.0, 500.0, 400.0)
        self.pose = CameraPose(43.0, -80.0, 100.0)

    def test_principal_point_intersects_below_camera(self) -> None:
        estimate = estimate_flat_water(Pixel(500.0, 400.0), self.intrinsics, self.pose)
        self.assertAlmostEqual(estimate.latitude, 43.0, places=8)
        self.assertAlmostEqual(estimate.longitude, -80.0, places=8)
        self.assertGreater(estimate.uncertainty_m, 1.0)

    def test_right_pixel_moves_east(self) -> None:
        estimate = estimate_flat_water(Pixel(600.0, 400.0), self.intrinsics, self.pose)
        self.assertAlmostEqual(estimate.latitude, 43.0, places=8)
        self.assertGreater(estimate.longitude, -80.0)

    def test_invalid_altitude(self) -> None:
        for altitude in (0, -100):
            with self.subTest(altitude=altitude), self.assertRaises(ValueError):
                estimate_flat_water(
                    Pixel(500.0, 400.0),
                    self.intrinsics,
                    CameraPose(43, -80, altitude),
                )


if __name__ == "__main__":
    unittest.main()
