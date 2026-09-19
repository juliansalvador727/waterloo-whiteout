from __future__ import annotations

import unittest
from datetime import datetime, timedelta, timezone

from whiteout.models import GeoEstimate
from whiteout.tracker import ConstantVelocityTracker


class TrackerTests(unittest.TestCase):
    def test_update_learns_northward_velocity(self) -> None:
        tracker = ConstantVelocityTracker()
        start = datetime(2026, 1, 1, tzinfo=timezone.utc)
        first = tracker.update(GeoEstimate(43.0, -80.0, 5.0, start))
        second = tracker.update(GeoEstimate(43.0001, -80.0, 4.0, start + timedelta(seconds=1)))
        self.assertEqual(first.velocity_north_mps, 0.0)
        self.assertGreater(second.velocity_north_mps, 0.0)
        predicted = tracker.predict(2.0)
        self.assertIsNotNone(predicted)
        assert predicted is not None
        self.assertGreater(predicted.latitude, second.latitude)

    def test_out_of_order_observation_is_ignored(self) -> None:
        tracker = ConstantVelocityTracker()
        now = datetime(2026, 1, 1, tzinfo=timezone.utc)
        initial = tracker.update(GeoEstimate(43.0, -80.0, 5.0, now))
        stale = tracker.update(GeoEstimate(44.0, -81.0, 1.0, now - timedelta(seconds=1)))
        self.assertEqual(stale, initial)


if __name__ == "__main__":
    unittest.main()

