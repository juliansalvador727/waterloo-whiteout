from __future__ import annotations

import contextlib
import io
import json
import time
import unittest

from scripts.truth_vessel import (
    SimulationClock,
    _pose_from_message,
    estimate_latlon,
    poses,
)


class TruthVesselTests(unittest.TestCase):
    def test_pose_reader_requests_initial_scene_and_stats(self) -> None:
        class Socket:
            def __init__(self) -> None:
                self.sent: list[dict[str, object]] = []

            def __enter__(self):
                return self

            def __exit__(self, *_args: object) -> None:
                return None

            def send(self, raw: str) -> None:
                self.sent.append(json.loads(raw))

            def recv(self, *, timeout: float) -> str:
                self.timeout = timeout
                return json.dumps(
                    {
                        "topic": "~/scene",
                        "msg": {
                            "model": [
                                {
                                    "name": "target_vessel",
                                    "pose": {"position": {"x": 4, "y": 5, "z": 0}},
                                }
                            ]
                        },
                    }
                )

        socket = Socket()
        stream = poses(
            "sim.invalid",
            "target_vessel",
            timeout=1.0,
            connector=lambda *_args, **_kwargs: socket,
        )
        pose, _clock = next(stream)
        stream.close()

        self.assertEqual(pose["position"]["x"], 4)
        self.assertEqual(
            [message["topic"] for message in socket.sent],
            ["~/scene", "~/pose/info", "~/world_stats"],
        )

    def test_extracts_pose_from_initial_scene_and_live_updates(self) -> None:
        scene = {
            "topic": "~/scene",
            "msg": {
                "model": [
                    {
                        "name": "target_vessel",
                        "pose": {
                            "position": {"x": 12, "y": 34, "z": 0},
                            "orientation": {"w": 1, "x": 0, "y": 0, "z": 0},
                        },
                    }
                ]
            },
        }
        live = {
            "topic": "~/pose/info",
            "msg": {"name": "target_vessel", "position": {"x": 13, "y": 35}},
        }

        self.assertEqual(_pose_from_message(scene, "target_vessel")["position"]["x"], 12)
        self.assertEqual(_pose_from_message(live, "target_vessel")["position"]["x"], 13)
        self.assertIsNone(_pose_from_message(scene, "another_model"))

    def test_simulation_clock_uses_measured_real_time_factor(self) -> None:
        clock = SimulationClock()
        clock.update(
            {
                "sim_time": {"sec": 10, "nsec": 0},
                "real_time": {"sec": 20, "nsec": 0},
                "paused": False,
            },
            100.0,
        )
        self.assertIsNone(clock.current(100.5))
        clock.update(
            {
                "sim_time": {"sec": 12, "nsec": 0},
                "real_time": {"sec": 21, "nsec": 0},
                "paused": False,
            },
            101.0,
        )
        self.assertAlmostEqual(clock.current(101.5), 13.0)

    def test_extracts_dashboard_and_named_collection_estimates(self) -> None:
        dashboard = {
            "target": {
                "track": {"display_latitude": 71.9, "display_longitude": -94.8}
            }
        }
        tracks = {"tracks": [{"name": "Sierra One", "lat": 72.0, "lon": -95.0}]}

        self.assertEqual(estimate_latlon(dashboard), (71.9, -94.8))
        self.assertEqual(estimate_latlon(tracks, "Sierra One"), (72.0, -95.0))
        self.assertIsNone(estimate_latlon(tracks, "missing"))

    def test_deadline_stops_reconnect_loop_without_a_pose(self) -> None:
        def unavailable(*_args, **_kwargs):
            raise OSError("offline")

        started = time.monotonic()
        with contextlib.redirect_stdout(io.StringIO()):
            result = list(
                poses(
                    "sim.invalid",
                    "target_vessel",
                    timeout=1.0,
                    deadline=started + 0.02,
                    connector=unavailable,
                )
            )

        self.assertEqual(result, [])
        self.assertLess(time.monotonic() - started, 0.3)


if __name__ == "__main__":
    unittest.main()
