from __future__ import annotations

import unittest

import whiteout
from whiteout.objects import (
    Boat,
    Copter,
    FixedWingPlane,
    MavProxyCommand,
    ObjectType,
    Plane,
    Quadcopter,
    Tower,
    UnsupportedCommand,
    object_for_type,
    tower_angle_to_pwm,
    tower_pwm_to_angle,
)


class ObjectTests(unittest.TestCase):
    def test_default_endpoints_and_factory(self) -> None:
        objects = [Copter(), Plane(), Boat(), Tower.one(), Tower.two()]
        self.assertEqual([item.port for item in objects], [14550, 14560, 14570, 14580, 14590])
        self.assertEqual(Copter().mavproxy_address, "udpout:127.0.0.1:14550")
        self.assertEqual(
            Copter().mavproxy_start_arguments(),
            ("mavproxy.py", "--master=udpout:127.0.0.1:14550"),
        )
        self.assertIsInstance(object_for_type("plane"), Plane)
        self.assertEqual(object_for_type(ObjectType.COPTER).object_type, ObjectType.COPTER)
        self.assertIs(Quadcopter, Copter)
        self.assertIs(FixedWingPlane, Plane)
        self.assertFalse(Boat().bundled)
        self.assertIs(whiteout.Copter, Copter)

    def test_commands_are_plans_and_do_not_execute(self) -> None:
        command = Copter().takeoff(20)
        self.assertIsInstance(command, MavProxyCommand)
        self.assertEqual(command.render(), "takeoff 20")
        self.assertFalse(hasattr(command, "execute"))
        self.assertTrue(hasattr(Copter(), "connect_mavproxy"))

    def test_object_specific_commands(self) -> None:
        copter = Copter()
        self.assertEqual(str(copter.mode("guided")), "mode GUIDED")
        self.assertEqual(str(copter.arm()), "arm throttle")
        self.assertEqual(str(copter.set_yaw(90, 20)), "setyaw 90 20 0")
        self.assertEqual(str(copter.velocity(2, 0, 0)), "velocity 2 0 0")

        tower = Tower.two()
        self.assertEqual(
            str(tower.pan(-108)),
            "servo set 1 1200",
        )
        self.assertEqual(
            str(tower.tilt(22.5)),
            "servo set 2 1700",
        )
        with self.assertRaises(UnsupportedCommand):
            tower.arm()

    def test_tower_servo_angle_and_pwm_conversion(self) -> None:
        self.assertEqual(tower_angle_to_pwm(1, -144), 1100)
        self.assertEqual(tower_angle_to_pwm(1, 0), 1500)
        self.assertEqual(tower_angle_to_pwm(1, 144), 1900)
        self.assertEqual(tower_pwm_to_angle(1, 1200), -108)
        self.assertEqual(tower_angle_to_pwm(2, -22.5), 1100)
        self.assertEqual(tower_angle_to_pwm(2, 7.5), 1500)
        self.assertEqual(tower_angle_to_pwm(2, 37.5), 1900)
        self.assertEqual(tower_pwm_to_angle(2, 1700), 22.5)

    def test_invalid_or_unsupported_values_are_rejected(self) -> None:
        with self.assertRaises(UnsupportedCommand):
            Plane().mode("GUIDED")
        with self.assertRaises(ValueError):
            Copter().rc(3, 2500)
        with self.assertRaises(ValueError):
            Tower().pan(-145)
        with self.assertRaises(ValueError):
            Tower().tilt(38)
        with self.assertRaises(ValueError):
            Tower().servo(3, 0)
        with self.assertRaises(ValueError):
            Copter().watch("ATTITUDE\nreboot")
        with self.assertRaises(ValueError):
            object_for_type("rover")


if __name__ == "__main__":
    unittest.main()
