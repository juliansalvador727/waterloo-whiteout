from __future__ import annotations

import unittest
from unittest.mock import MagicMock, patch

from whiteout.control import (
    CopterController,
    CopterMode,
    MavProxyConnectionError,
    PlaneMode,
    TowerController,
)
from whiteout.objects import Copter, Plane, Tower


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
            copter.return_to_launch()

        self.assertEqual(
            [str(call.args[0]) for call in session.send.call_args_list],
            [
                "mode GUIDED",
                "arm throttle",
                "takeoff 20",
                "velocity 2 0 -0.5",
                "mode RTL",
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
        controller.close()

        self.assertEqual(
            [str(call.args[0]) for call in session.send.call_args_list],
            ["mode TAKEOFF", "arm throttle", "mode LOITER"],
        )

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
