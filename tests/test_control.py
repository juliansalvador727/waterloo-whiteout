from __future__ import annotations

import math
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from whiteout.control import (
    CopterController,
    CopterMode,
    MavProxyConnectionError,
    PlaneMode,
    TowerController,
)
from whiteout.objects import Copter, Plane, Tower


LANDING_MISSION = (
    Path(__file__).resolve().parents[1]
    / "missions"
    / "arctic_sim_fixed_wing_land.waypoints"
)


class TypedControllerTests(unittest.TestCase):
    def controller_for(self, vehicle: Copter | Plane | Tower):
        session = MagicMock()
        session.running = True
        session.output = "online system 1"
        session.wait_for_output.return_value = True
        patcher = patch.object(type(vehicle), "mavproxy_session", return_value=session)
        patcher.start()
        self.addCleanup(patcher.stop)
        return vehicle.controller(), session

    def test_copter_functions_send_typed_mavproxy_commands(self) -> None:
        controller, session = self.controller_for(Copter())
        self.assertIsInstance(controller, CopterController)

        with controller as copter:
            copter.set_mode(CopterMode.GUIDED)
            copter.arm()
            copter.takeoff(20)
            copter.set_velocity(2, 0, -0.5)
            copter.autoland()

        self.assertEqual(
            [str(call.args[0]) for call in session.send.call_args_list],
            [
                "mode GUIDED",
                "arm throttle",
                "takeoff 20",
                "velocity 2 0 -0.5",
                "mode LAND",
            ],
        )
        session.close.assert_called_once_with()

    def test_copter_rejects_plane_mode_even_if_mode_name_exists(self) -> None:
        controller, _ = self.controller_for(Copter())
        with self.assertRaisesRegex(TypeError, "CopterMode"):
            controller.set_mode(PlaneMode.RTL)  # type: ignore[arg-type]

    def test_plane_takeoff_is_mode_then_arm(self) -> None:
        controller, session = self.controller_for(Plane())
        controller.start()
        controller.takeoff()
        controller.loiter()
        controller.autoland(LANDING_MISSION)
        controller.close()

        commands = [call.args[0] for call in session.send.call_args_list]
        self.assertEqual(
            [str(command) for command in commands[:3]],
            ["mode TAKEOFF", "arm throttle", "mode LOITER"],
        )
        self.assertEqual(
            [str(command) for command in commands[3:7]],
            [
                "param set LAND_PITCH_DEG 4",
                "param set LAND_FLARE_ALT 4",
                "param set LAND_FLARE_SEC 3",
                "param set TECS_LAND_SINK 0.2",
            ],
        )
        self.assertEqual(commands[7].name, "wp")
        self.assertEqual(commands[7].arguments, ("load", str(LANDING_MISSION)))
        self.assertEqual(str(commands[8]), "mode AUTO")
        session.wait_for_output.assert_any_call(
            "Sent all ", timeout=15.0, start=len("online system 1")
        )

    def test_plane_resets_landing_sequence_only_after_disarm(self) -> None:
        controller, session = self.controller_for(Plane())
        with patch.object(controller, "is_armed", return_value=False):
            controller.reset_after_landing()
        self.assertEqual(
            [str(call.args[0]) for call in session.send.call_args_list],
            ["mode MANUAL", "wp clear"],
        )

        session.send.reset_mock()
        with (
            patch.object(controller, "is_armed", return_value=True),
            self.assertRaisesRegex(RuntimeError, "while the plane is armed"),
        ):
            controller.reset_after_landing()
        session.send.assert_not_called()

    def test_wait_until_disarmed_exits_on_first_disarmed_heartbeat(self) -> None:
        controller, _ = self.controller_for(Plane())
        with (
            patch.object(controller, "is_armed", side_effect=(True, True, False)) as armed,
            patch("whiteout.control.time.sleep") as sleep,
        ):
            controller.wait_until_disarmed(timeout_s=30, poll_interval_s=1)
        self.assertEqual(armed.call_count, 3)
        self.assertEqual(sleep.call_count, 2)

    def test_landing_final_approach_is_rotated_ten_degrees_clockwise(self) -> None:
        rows = [line.split("\t") for line in LANDING_MISSION.read_text().splitlines()[1:]]
        threshold_lat, threshold_lon = float(rows[3][8]), float(rows[3][9])
        touchdown_lat, touchdown_lon = float(rows[4][8]), float(rows[4][9])
        mean_lat = math.radians((threshold_lat + touchdown_lat) / 2)
        north = (touchdown_lat - threshold_lat) * 111_320
        east = (touchdown_lon - threshold_lon) * 111_320 * math.cos(mean_lat)
        heading = math.degrees(math.atan2(east, north)) % 360
        self.assertAlmostEqual(heading, 82.97, places=1)

    def test_tower_functions_use_cmdlong_without_optional_servo_module(self) -> None:
        controller, session = self.controller_for(Tower.two())
        self.assertIsInstance(controller, TowerController)
        controller.start()
        controller.pan(1200)
        controller.tilt(1700)
        controller.close()

        self.assertEqual(
            [str(call.args[0]) for call in session.send.call_args_list],
            [
                "cmdlong MAV_CMD_DO_SET_SERVO 1 1200 0 0 0 0 0",
                "cmdlong MAV_CMD_DO_SET_SERVO 2 1700 0 0 0 0 0",
            ],
        )

    def test_controller_fails_closed_when_heartbeat_never_arrives(self) -> None:
        session = MagicMock()
        session.output = "Waiting for heartbeat"
        session.wait_for_output.return_value = False
        with patch.object(Copter, "mavproxy_session", return_value=session):
            controller = Copter().controller(connection_timeout_s=0.01)

        with self.assertRaisesRegex(MavProxyConnectionError, "did not come online"):
            controller.start()
        session.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
