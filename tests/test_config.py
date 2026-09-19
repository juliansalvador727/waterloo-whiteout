from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from whiteout.config import ConfigError, load_config, substitute_environment


class ConfigTests(unittest.TestCase):
    def test_environment_substitution_and_defaults(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(json.dumps({"sim_host": "${SIM_HOST}"}), encoding="utf-8")
            config = load_config(path, {"SIM_HOST": "sim.example"})
        self.assertEqual(config.sim_host, "sim.example")
        self.assertEqual(
            [camera.name for camera in config.cameras],
            ["quadcopter", "fixed-wing", "tower-1", "tower-2"],
        )
        self.assertEqual([camera.port for camera in config.cameras], [8600, 8610, 8630, 8640])
        self.assertEqual([camera.path for camera in config.cameras], ["/stream"] * 4)
        self.assertEqual(
            [(camera.width, camera.height, camera.hfov_deg, camera.vfov_deg) for camera in config.cameras],
            [
                (640, 480, 114.6, 99.4),
                (640, 360, 69.0, 42.6),
                (640, 360, 60.0, 36.1),
                (640, 360, 60.0, 36.1),
            ],
        )
        self.assertEqual(
            [link.name for link in config.mavlink],
            ["quadcopter", "fixed-wing", "tower-1", "tower-2"],
        )
        self.assertEqual([link.port for link in config.mavlink], [14550, 14560, 14580, 14590])
        self.assertFalse(config.track_api.allow_submission)
        self.assertEqual(config.track_api.name, "Sierra One")
        self.assertFalse(config.track_api.include_speed)

    def test_substitution_default_and_missing_value(self) -> None:
        self.assertEqual(substitute_environment("${MISSING:-fallback}", {}), "fallback")
        with self.assertRaises(ConfigError):
            substitute_environment("${MISSING}", {})

    def test_submission_requires_endpoint(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(json.dumps({"track_api": {"allow_submission": True}}), encoding="utf-8")
            with self.assertRaises(ConfigError):
                load_config(path, {})


if __name__ == "__main__":
    unittest.main()
