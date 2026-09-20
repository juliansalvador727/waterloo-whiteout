from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from whiteout.config import ConfigError, config_from_mapping, load_config, substitute_environment


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
        self.assertEqual([camera.crop_right_px for camera in config.cameras], [40, 0, 0, 0])
        self.assertEqual(
            [(camera.width, camera.height, camera.hfov_deg, camera.vfov_deg) for camera in config.cameras],
            [
                (960, 720, 114.6, 99.4),
                (1280, 720, 69.0, 42.6),
                (1280, 720, 60.0, 36.1),
                (1280, 720, 60.0, 36.1),
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
        self.assertEqual([item.name for item in config.tower_motion], ["tower-1", "tower-2"])
        self.assertEqual(config.tower_motion[0].pan_min_deg, -135.0)
        self.assertEqual(config.tower_motion[0].pan_rate_deg_s, 24.0)
        self.assertEqual(config.coordinator.tower_detection_hold_s, 1.5)
        self.assertEqual(config.coordinator.reacquire_grid_rows, 12)
        self.assertEqual(config.coordinator.reacquire_grid_columns, 12)

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

    def test_per_tower_motion_config_and_validation(self) -> None:
        config = config_from_mapping({
            "tower_motion": {
                "tower-1": {"pan_min_deg": -90, "pan_rate_deg_s": 8},
                "tower-2": {"tilt_max_deg": 20, "command_hz": 5},
            }
        })
        self.assertEqual(config.tower_motion[0].pan_min_deg, -90)
        self.assertEqual(config.tower_motion[0].pan_rate_deg_s, 8)
        self.assertEqual(config.tower_motion[1].tilt_max_deg, 20)
        self.assertEqual(config.tower_motion[1].command_hz, 5)
        with self.assertRaisesRegex(ConfigError, "pan limits"):
            config_from_mapping({
                "tower_motion": {"tower-1": {"pan_min_deg": -150}}
            })

    def test_camera_crop_validation(self) -> None:
        with self.assertRaisesRegex(ConfigError, "invalid right crop"):
            config_from_mapping({
                "cameras": [{"name": "quadcopter", "port": 8600, "crop_right_px": -1}]
            })
        with self.assertRaisesRegex(ConfigError, "must be smaller than its width"):
            config_from_mapping({
                "cameras": [{
                    "name": "quadcopter",
                    "port": 8600,
                    "width": 40,
                    "crop_right_px": 40,
                }]
            })


if __name__ == "__main__":
    unittest.main()
