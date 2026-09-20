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

    def test_landing_final_approach_matches_captured_heading(self) -> None:
        rows = [line.split("\t") for line in LANDING_MISSION.read_text().splitlines()[1:]]
        straight_approach = rows[-4:]
        headings = []
        for start, end in zip(straight_approach, straight_approach[1:]):
            start_lat, start_lon = float(start[8]), float(start[9])
            end_lat, end_lon = float(end[8]), float(end[9])
            mean_lat = math.radians((start_lat + end_lat) / 2)
            north = (end_lat - start_lat) * 111_320
            east = (end_lon - start_lon) * 111_320 * math.cos(mean_lat)
            headings.append(math.degrees(math.atan2(east, north)) % 360)
        for heading in headings:
            self.assertAlmostEqual(heading, 259.96, places=1)

    def test_static_landing_starts_over_ocean_with_required_clearance(self) -> None:
        rows = [line.split("\t") for line in LANDING_MISSION.read_text().splitlines()[1:]]
        first_navigation_waypoint = rows[2]
        self.assertEqual(int(first_navigation_waypoint[3]), 16)
        self.assertEqual(
            (float(first_navigation_waypoint[8]), float(first_navigation_waypoint[9])),
            (71.999299, -94.843167),
        )
        self.assertGreaterEqual(float(first_navigation_waypoint[10]), 75.99)

        start_position = rows[0]
        self.assertEqual(
            (float(start_position[8]), float(start_position[9])),
            (71.9982129, -94.8420161),
        )
        land_aim = rows[-1]
        self.assertEqual(
            (float(land_aim[8]), float(land_aim[9])),
            (71.9981315, -94.8435044),
        )
        mountain_latitude = 71.995786
        navigated_rows = [row for row in rows if int(row[3]) in (16, 21)]
        self.assertTrue(
            all(float(row[8]) > mountain_latitude for row in navigated_rows)
        )

    def test_tower_functions_convert_degrees_and_use_servo_set(self) -> None:
        controller, session = self.controller_for(Tower.two())
        self.assertIsInstance(controller, TowerController)
        controller.start()
        controller.pan(-108)
        controller.tilt(22.5)
        controller.close()

        self.assertEqual(
            [str(call.args[0]) for call in session.send.call_args_list],
            [
                "servo set 1 1200",
                "servo set 2 1700",
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
