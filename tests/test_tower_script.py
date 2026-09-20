from __future__ import annotations

import unittest
from unittest.mock import MagicMock, patch

from scripts.test_tower import parse_args, run_test
from whiteout import Tower


class TowerScriptTests(unittest.TestCase):
    def test_parse_args_requires_confirmation_and_valid_angles(self) -> None:
        with self.assertRaises(SystemExit):
            parse_args(("tower-1",))
        with self.assertRaises(SystemExit):
            parse_args(("tower-1", "--pan-deg", "181", "--confirm-movement"))

        args = parse_args(
            (
                "tower-2",
                "--pan-deg",
                "-108",
                "--tilt-deg",
                "22.5",
                "--confirm-movement",
            )
        )
        self.assertEqual(args.tower, "tower-2")
        self.assertEqual(args.pan_deg, -108)
        self.assertEqual(args.tilt_deg, 22.5)

    def test_run_test_moves_then_recenters_tower(self) -> None:
        controller = MagicMock()
        context = MagicMock()
        context.__enter__.return_value = controller
        vehicle = MagicMock()
        vehicle.name = "tower-1"
        vehicle.controller.return_value = context

        with (
            patch.object(Tower, "one", return_value=vehicle),
            patch("scripts.test_tower.time.sleep") as sleep,
        ):
            run_test("tower-1", "sim.example", -108, 22.5, 0.1, 5.0)

        vehicle.controller.assert_called_once_with(connection_timeout_s=5.0)
        self.assertEqual(controller.center.call_count, 2)
        controller.pan.assert_called_once_with(-108)
        controller.tilt.assert_called_once_with(22.5)
        self.assertEqual(sleep.call_count, 2)


if __name__ == "__main__":
    unittest.main()
