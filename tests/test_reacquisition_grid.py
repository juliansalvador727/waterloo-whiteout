from __future__ import annotations

import unittest

from whiteout.models import GeoEstimate
from whiteout.reacquisition_grid import WeightedReacquisitionGrid


class ReacquisitionGridTests(unittest.TestCase):
    def setUp(self) -> None:
        self.grid = WeightedReacquisitionGrid(
            0.0,
            0.0,
            1.0,
            1.0,
            rows=10,
            columns=10,
            half_life_s=10.0,
            neighborhood_cells=2,
        )

    def test_detection_area_pulls_reacquisition_target_toward_evidence(self) -> None:
        predicted = GeoEstimate(0.50, 0.50, 20.0)
        for _ in range(4):
            self.grid.observe(GeoEstimate(0.58, 0.58, 5.0), 0.0)

        target = self.grid.weighted_target(predicted, 2.0)

        self.assertGreater(target.latitude, predicted.latitude)
        self.assertGreater(target.longitude, predicted.longitude)
        self.assertLess(target.latitude, 0.58)
        self.assertEqual(target.uncertainty_m, predicted.uncertainty_m)

    def test_far_detection_cells_do_not_pull_local_reacquisition(self) -> None:
        self.grid.observe(GeoEstimate(0.90, 0.90, 5.0), 0.0)
        predicted = GeoEstimate(0.20, 0.20, 20.0)

        self.assertEqual(self.grid.weighted_target(predicted, 1.0), predicted)

    def test_old_evidence_decays_and_cells_report_bounds(self) -> None:
        self.grid.observe(GeoEstimate(0.25, 0.35, 5.0), 0.0)
        recent = self.grid.cells(0.0)
        old = self.grid.cells(20.0)

        self.assertEqual((recent[0].row, recent[0].column), (2, 3))
        self.assertAlmostEqual(old[0].weight, recent[0].weight * 0.25)
        self.assertLessEqual(old[0].south, 0.25)
        self.assertGreaterEqual(old[0].north, 0.25)

    def test_positions_on_north_east_edges_use_last_cell(self) -> None:
        self.assertEqual(self.grid.cell_index(1.0, 1.0), (9, 9))
        with self.assertRaises(ValueError):
            self.grid.cell_index(1.01, 1.0)


if __name__ == "__main__":
    unittest.main()
