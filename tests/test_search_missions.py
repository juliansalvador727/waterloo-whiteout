from __future__ import annotations

import tempfile
import unittest
from math import asin, cos, radians, sin, sqrt
from pathlib import Path
from unittest.mock import MagicMock, patch

from whiteout.control import CopterMode, MissionUploadError, PlaneMode
from whiteout.mission import MissionValidationError, SearchMission
from whiteout.objects import Copter, Plane


MISSIONS = Path(__file__).resolve().parents[1] / "missions"
SEARCH_MISSIONS = {
    "fixed_wing_search.waypoints",
    "quadcopter_search.waypoints",
}


def route_length_m(mission: SearchMission) -> float:
    radius_m = 6_371_000.0
    total_m = 0.0
    for start, end in zip(mission.waypoints[1:], mission.waypoints[2:]):
        lat1, lat2 = radians(start.latitude), radians(end.latitude)
        dlat = lat2 - lat1
        dlon = radians(end.longitude - start.longitude)
        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        total_m += 2 * radius_m * asin(sqrt(a))
    return total_m


class SearchMissionTests(unittest.TestCase):
    def controller_for(self, vehicle: Copter | Plane):
        session = MagicMock()
        session.running = True
        session.output = "online system 1"
        session.wait_for_output.return_value = True
        patcher = patch.object(type(vehicle), "mavproxy_session", return_value=session)
        patcher.start()
        self.addCleanup(patcher.stop)
        return vehicle.controller(), session

    def test_bundled_search_missions_are_valid(self) -> None:
        self.assertEqual(
            {path.name for path in MISSIONS.glob("*search.waypoints")},
            SEARCH_MISSIONS,
        )
        plane = SearchMission.load(MISSIONS / "fixed_wing_search.waypoints")
        copter = SearchMission.load(MISSIONS / "quadcopter_search.waypoints")
        self.assertIs(plane.validate_for_search(), plane)
        self.assertIs(copter.validate_for_search(), copter)
        self.assertTrue(all(point.command == 16 for point in plane.waypoints))
        self.assertTrue(all(point.command == 16 for point in copter.waypoints))
        self.assertFalse({21, 22} & {point.command for point in plane.waypoints})
        self.assertFalse({21, 22} & {point.command for point in copter.waypoints})
        self.assertEqual(
            (plane.waypoints[0].latitude, plane.waypoints[0].longitude),
            (71.998195, -94.841967),
        )
        self.assertEqual(
            (copter.waypoints[0].latitude, copter.waypoints[0].longitude),
            (71.995807, -94.839300),
        )
        self.assertEqual(
            (copter.waypoints[1].latitude, copter.waypoints[1].longitude),
            (71.995807, -94.839300),
        )
        self.assertTrue(all(point.altitude_m == 200 for point in plane.waypoints[1:]))
        self.assertTrue(all(point.altitude_m == 90 for point in copter.waypoints[1:]))

        plane_length_m = route_length_m(plane)
        copter_length_m = route_length_m(copter)
        self.assertGreater(plane_length_m, 12_000)
        self.assertLess(copter_length_m, 3_000)
        self.assertGreater(plane_length_m, copter_length_m * 4)

    def test_invalid_or_landing_mission_is_rejected_before_transmission(self) -> None:
        controller, session = self.controller_for(Plane())
        landing = MISSIONS / "arctic_sim_fixed_wing_land.waypoints"
        with self.assertRaisesRegex(MissionValidationError, "only MAV_CMD_NAV_WAYPOINT"):
            controller.upload_search_mission(landing)
        session.send.assert_not_called()

        with tempfile.TemporaryDirectory() as directory:
            invalid = Path(directory) / "invalid.waypoints"
            invalid.write_text("not a mission\n", encoding="utf-8")
            with self.assertRaisesRegex(MissionValidationError, "QGC WPL 110"):
                controller.upload_search_mission(invalid)
        session.send.assert_not_called()

    def test_home_row_must_be_current_even_if_a_route_row_is_current(self) -> None:
        source = (MISSIONS / "quadcopter_search.waypoints").read_text()
        invalid_text = source.replace("0\t1\t0\t16", "0\t0\t0\t16", 1).replace(
            "1\t0\t3\t16", "1\t1\t3\t16", 1
        )
        with tempfile.TemporaryDirectory() as directory:
            invalid = Path(directory) / "route-current.waypoints"
            invalid.write_text(invalid_text, encoding="utf-8")
            with self.assertRaisesRegex(MissionValidationError, "home row.*current"):
                SearchMission.load(invalid)

    def test_route_rows_must_not_be_current(self) -> None:
        source = (MISSIONS / "quadcopter_search.waypoints").read_text()
        invalid_text = source.replace("1\t0\t3\t16", "1\t1\t3\t16", 1)
        with tempfile.TemporaryDirectory() as directory:
            invalid = Path(directory) / "two-current.waypoints"
            invalid.write_text(invalid_text, encoding="utf-8")
            with self.assertRaisesRegex(MissionValidationError, "route rows.*current"):
                SearchMission.load(invalid)

    def test_upload_is_confirmed_but_never_arms_or_enters_auto(self) -> None:
        cases = (
            (Copter(), "quadcopter_search.waypoints"),
            (Plane(), "fixed_wing_search.waypoints"),
        )
        for vehicle, filename in cases:
            with self.subTest(vehicle=vehicle.name):
                controller, session = self.controller_for(vehicle)
                mission_path = MISSIONS / filename
                mission = controller.upload_search_mission(mission_path)
                self.assertEqual(controller.search_mission, mission)
                commands = [str(call.args[0]) for call in session.send.call_args_list]
                self.assertEqual(commands, [f"wp load {mission_path}"])
                self.assertFalse(any(command.startswith("arm") for command in commands))
                self.assertNotIn("mode AUTO", commands)

                controller.start_search()
                self.assertEqual(str(session.send.call_args_list[-1].args[0]), "mode AUTO")

    def test_upload_requires_mavproxy_confirmation(self) -> None:
        controller, session = self.controller_for(Plane())
        mission_path = MISSIONS / "fixed_wing_search.waypoints"
        controller.upload_search_mission(mission_path)
        self.assertIsNotNone(controller.search_mission)
        session.wait_for_output.return_value = False
        with self.assertRaises(MissionUploadError):
            controller.upload_search_mission(mission_path, upload_timeout_s=1)
        self.assertIsNone(controller.search_mission)

    def test_invalid_timeout_preserves_prior_confirmed_mission(self) -> None:
        controller, session = self.controller_for(Copter())
        mission_path = MISSIONS / "quadcopter_search.waypoints"
        confirmed = controller.upload_search_mission(mission_path)
        session.send.reset_mock()
        with self.assertRaisesRegex(ValueError, "timeout must be positive"):
            controller.upload_search_mission(mission_path, upload_timeout_s=0)
        self.assertEqual(controller.search_mission, confirmed)
        session.send.assert_not_called()

    def test_vehicle_specific_pause_resume_abort_and_clear(self) -> None:
        cases = (
            (Copter(), "quadcopter_search.waypoints", CopterMode.STABILIZE),
            (Plane(), "fixed_wing_search.waypoints", PlaneMode.MANUAL),
        )
        for vehicle, filename, safe_mode in cases:
            with self.subTest(vehicle=vehicle.name):
                controller, session = self.controller_for(vehicle)
                controller.upload_search_mission(MISSIONS / filename)
                session.send.reset_mock()
                controller.pause_search()
                controller.resume_search()
                controller.abort_search()
                with patch.object(controller, "is_armed", return_value=False):
                    controller.clear_search_mission()
                self.assertEqual(
                    [str(call.args[0]) for call in session.send.call_args_list],
                    ["mode LOITER", "mode AUTO", "mode RTL", f"mode {safe_mode.value}", "wp clear"],
                )
                self.assertIsNone(controller.search_mission)

    def test_clear_rejects_armed_vehicle(self) -> None:
        controller, session = self.controller_for(Copter())
        controller.upload_search_mission(MISSIONS / "quadcopter_search.waypoints")
        session.send.reset_mock()
        with (
            patch.object(controller, "is_armed", return_value=True),
            self.assertRaisesRegex(RuntimeError, "while the vehicle is armed"),
        ):
            controller.clear_search_mission()
        session.send.assert_not_called()


if __name__ == "__main__":
    unittest.main()
