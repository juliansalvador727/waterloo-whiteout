from __future__ import annotations

import io
import json
import unittest

from whiteout.simulator_metadata import ArcticSimMetadataClient


class _Response(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *args: object) -> None:
        self.close()


class SimulatorMetadataTests(unittest.TestCase):
    def test_generated_tower_pose_is_used_without_guessing_height(self) -> None:
        payload = {
            "ok": True,
            "assets": [
                {
                    "name": "tower-1",
                    "rostered": True,
                    "pose": {
                        "name": "tower-1",
                        "type": "tower",
                        "latitude": 71.99912183839884,
                        "longitude": -94.81086504031542,
                        "world_x_m": 1,
                        "world_y_m": 2,
                        "ground_world_z_m": 30,
                        "model_world_z_m": 31,
                        "yaw_rad": 0.2,
                        "camera_world_z_m": 33.7,
                    },
                }
            ],
        }
        opened = []

        def opener(url: str, *, timeout: float):
            opened.append((url, timeout))
            return _Response(json.dumps(payload).encode())

        client = ArcticSimMetadataClient("127.0.0.1", opener=opener)
        towers = client.towers()

        self.assertEqual(opened, [("http://127.0.0.1:8090/api/assets", 5.0)])
        self.assertEqual(len(towers), 1)
        self.assertEqual(towers[0].name, "tower-1")
        self.assertAlmostEqual(towers[0].latitude, 71.99912183839884)
        self.assertEqual(towers[0].camera_world_z_m, 33.7)

    def test_missing_generated_pose_is_not_fabricated(self) -> None:
        payload = {"ok": True, "assets": [{"name": "tower-1", "rostered": True, "pose": None}]}
        client = ArcticSimMetadataClient(
            "sim.invalid",
            opener=lambda *args, **kwargs: _Response(json.dumps(payload).encode()),
        )
        self.assertEqual(client.fetch(), ())


if __name__ == "__main__":
    unittest.main()
