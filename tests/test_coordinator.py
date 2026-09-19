from __future__ import annotations

import unittest

from whiteout.coordinator import Coordinator
from whiteout.models import GeoEstimate, SearchMode


class CoordinatorTests(unittest.TestCase):
    def test_state_progression_and_reacquisition(self) -> None:
        coordinator = Coordinator(confirmations_required=2, reacquire_after_s=2, search_after_s=10)
        estimate = GeoEstimate(43.0, -80.0, 5.0)
        self.assertEqual(coordinator.decide(0, None).mode, SearchMode.SEARCH)
        self.assertEqual(coordinator.decide(1, estimate).mode, SearchMode.CONFIRM)
        self.assertEqual(coordinator.decide(2, estimate).mode, SearchMode.TRACK)
        recommendation = coordinator.decide(5, None)
        self.assertEqual(recommendation.mode, SearchMode.REACQUIRE)
        self.assertIn("search", recommendation.action)
        self.assertEqual(coordinator.decide(13, None).mode, SearchMode.SEARCH)

    def test_outputs_are_recommendations(self) -> None:
        result = Coordinator().decide(0, None)
        self.assertEqual(result.action, "continue_search_pattern")
        self.assertFalse(hasattr(result, "execute"))


if __name__ == "__main__":
    unittest.main()

