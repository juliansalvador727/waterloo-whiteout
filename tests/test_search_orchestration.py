from __future__ import annotations

import unittest

from whiteout.models import Detection, Pixel, SearchMode
from whiteout.search import DetectionSource, SearchAction, SearchOrchestrator


class SearchOrchestrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.detection = Detection("tower-1", Pixel(20, 30), 0.9)

    def test_all_operational_transitions_are_recommendation_only(self) -> None:
        search = SearchOrchestrator(confirmations_required=2, reacquire_after_s=2, return_after_s=10)
        self.assertEqual(search.observe(0, None).state, SearchMode.SEARCH)
        candidate = search.observe(1, self.detection)
        self.assertEqual(candidate.state, SearchMode.CONFIRM)
        self.assertEqual(candidate.action, SearchAction.CONFIRM_CANDIDATE)
        self.assertFalse(candidate.detection_confirmed)
        self.assertFalse(hasattr(candidate, "execute"))
        self.assertEqual(search.observe(2, self.detection).state, SearchMode.TRACK)
        self.assertEqual(search.observe(5, None).state, SearchMode.REACQUIRE)
        self.assertEqual(search.observe(13, None).state, SearchMode.RETURN)
        self.assertEqual(search.abort().state, SearchMode.ABORT)
        self.assertEqual(search.observe(14, self.detection).state, SearchMode.ABORT)

    def test_manual_and_synthetic_injection_have_explicit_provenance(self) -> None:
        search = SearchOrchestrator()
        self.assertEqual(search.inject(1, self.detection).source, DetectionSource.MANUAL)
        self.assertEqual(
            search.inject(2, self.detection, synthetic=True).source,
            DetectionSource.SYNTHETIC,
        )

    def test_low_confidence_input_is_absence_not_confirmation(self) -> None:
        search = SearchOrchestrator(minimum_confidence=0.8)
        weak = Detection("tower-1", Pixel(1, 2), 0.2)
        result = search.inject(0, weak, synthetic=True)
        self.assertEqual(result.state, SearchMode.SEARCH)
        self.assertIsNone(result.detection)

    def test_single_observation_cannot_be_configured_as_confirmation(self) -> None:
        with self.assertRaisesRegex(ValueError, "at least two observations"):
            SearchOrchestrator(confirmations_required=1)


if __name__ == "__main__":
    unittest.main()
