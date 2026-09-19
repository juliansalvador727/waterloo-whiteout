from __future__ import annotations

import os
import unittest
import urllib.request

from whiteout.arctic_sim import ARCTIC_SIM_FLEET, arctic_sim_fleet
from whiteout.control import CopterController, PlaneController, TowerController
from whiteout.objects import Copter, Plane, Tower


EXPECTED_NAMES = ("quadcopter", "fixed-wing", "tower-1", "tower-2")
EXPECTED_MAVPROXY_COMMANDS = (
    "mavproxy.py --master=udpout:10.99.0.1:14550",
    "mavproxy.py --master=udpout:10.99.0.1:14560",
    "mavproxy.py --master=udpout:10.99.0.1:14580",
    "mavproxy.py --master=udpout:10.99.0.1:14590",
)
EXPECTED_CAMERA_URLS = (
    "http://10.99.0.1:8600/stream",
    "http://10.99.0.1:8610/stream",
    "http://10.99.0.1:8630/stream",
    "http://10.99.0.1:8640/stream",
)


class ArcticSimFleetTests(unittest.TestCase):
    def test_assets_are_in_required_order(self) -> None:
        self.assertEqual(tuple(asset.name for asset in ARCTIC_SIM_FLEET), EXPECTED_NAMES)
        self.assertEqual(
            tuple(type(asset.vehicle) for asset in ARCTIC_SIM_FLEET),
            (Copter, Plane, Tower, Tower),
        )
        self.assertEqual(
            tuple(type(asset.controller()) for asset in ARCTIC_SIM_FLEET),
            (CopterController, PlaneController, TowerController, TowerController),
        )

    def test_mavproxy_commands_match_deployment_in_the_same_order(self) -> None:
        self.assertEqual(
            tuple(asset.mavproxy_command for asset in ARCTIC_SIM_FLEET),
            EXPECTED_MAVPROXY_COMMANDS,
        )
        self.assertEqual(
            tuple(asset.mavproxy_session().master for asset in ARCTIC_SIM_FLEET),
            tuple(command.removeprefix("mavproxy.py --master=") for command in EXPECTED_MAVPROXY_COMMANDS),
        )

    def test_camera_urls_match_deployment_in_the_same_order(self) -> None:
        self.assertEqual(
            tuple(asset.camera_url for asset in ARCTIC_SIM_FLEET),
            EXPECTED_CAMERA_URLS,
        )
        self.assertEqual(
            tuple(asset.camera().url for asset in ARCTIC_SIM_FLEET),
            EXPECTED_CAMERA_URLS,
        )

    def test_host_override_preserves_ports_and_order(self) -> None:
        fleet = arctic_sim_fleet("sim.example")
        self.assertEqual(tuple(asset.name for asset in fleet), EXPECTED_NAMES)
        self.assertEqual(
            tuple(asset.camera_url for asset in fleet),
            (
                "http://sim.example:8600/stream",
                "http://sim.example:8610/stream",
                "http://sim.example:8630/stream",
                "http://sim.example:8640/stream",
            ),
        )
        self.assertEqual(
            tuple(asset.vehicle.port for asset in fleet),
            (14550, 14560, 14580, 14590),
        )


@unittest.skipUnless(
    os.environ.get("WHITEOUT_LIVE_ARCTIC_SIM") == "1",
    "set WHITEOUT_LIVE_ARCTIC_SIM=1 to test the live simulator",
)
class LiveArcticSimTests(unittest.TestCase):
    """Opt-in smoke tests for the real services at 10.99.0.1."""

    def test_all_camera_streams_return_jpeg_data_in_order(self) -> None:
        for asset in ARCTIC_SIM_FLEET:
            with self.subTest(asset=asset.name):
                with urllib.request.urlopen(asset.camera_url, timeout=5.0) as response:
                    data = response.read(64 * 1024)
                self.assertIn(b"\xff\xd8", data, f"{asset.name} stream did not contain a JPEG")

    def test_all_typed_controllers_connect_and_accept_status_in_order(self) -> None:
        for asset in ARCTIC_SIM_FLEET:
            with self.subTest(asset=asset.name):
                with asset.controller(connection_timeout_s=10.0) as controller:
                    controller.status()
                    self.assertTrue(controller.connected)


if __name__ == "__main__":
    unittest.main()
