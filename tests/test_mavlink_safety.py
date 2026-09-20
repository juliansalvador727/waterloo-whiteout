from __future__ import annotations

import sys
import types
import unittest
from math import pi
from unittest.mock import patch

from whiteout.mavlink import ReadOnlyMavlink


class _FakeMav:
    def __init__(self) -> None:
        self.heartbeats: list[tuple[int, ...]] = []

    def heartbeat_send(self, *values: int) -> None:
        self.heartbeats.append(values)


class _FakeConnection:
    def __init__(self, message: object | None = None) -> None:
        self.mav = _FakeMav()
        self.message = message

    def recv_match(self, **kwargs: object) -> object | None:
        return self.message


class _MessageConnection(_FakeConnection):
    def __init__(self, attitude: object | None, position: object | None) -> None:
        super().__init__()
        self.messages = [message for message in (attitude, position) if message is not None]
        self.calls: list[dict[str, object]] = []

    def recv_match(self, **kwargs: object) -> object | None:
        self.calls.append(kwargs)
        return self.messages.pop(0) if self.messages else None


class MavlinkSafetyTests(unittest.TestCase):
    def _module(self, connection: _FakeConnection) -> types.ModuleType:
        mavutil = types.SimpleNamespace(
            mavlink_connection=lambda address: self._capture_address(address, connection),
            mavlink=types.SimpleNamespace(
                MAV_TYPE_GCS=6,
                MAV_AUTOPILOT_INVALID=8,
                MAV_STATE_ACTIVE=4,
            ),
        )
        module = types.ModuleType("pymavlink")
        module.mavutil = mavutil
        return module

    def _capture_address(self, address: str, connection: _FakeConnection) -> _FakeConnection:
        self.address = address
        return connection

    def test_connect_uses_udpout_and_heartbeat_is_explicit(self) -> None:
        connection = _FakeConnection()
        adapter = ReadOnlyMavlink("quadcopter", "sim.invalid", 14550)
        with patch.dict(sys.modules, {"pymavlink": self._module(connection)}):
            adapter.connect()
        self.assertEqual(self.address, "udpout:sim.invalid:14550")
        self.assertEqual(connection.mav.heartbeats, [])

        connection = _FakeConnection()
        with patch.dict(sys.modules, {"pymavlink": self._module(connection)}):
            adapter.connect(initiate_telemetry=True)
        self.assertEqual(connection.mav.heartbeats, [(6, 8, 0, 0, 4)])

    def test_adapter_exposes_no_control_methods(self) -> None:
        forbidden = {
            "arm",
            "set_mode",
            "upload_mission",
            "command_long",
            "setpoint",
            "move",
        }
        self.assertTrue(forbidden.isdisjoint(dir(ReadOnlyMavlink)))

    def test_poll_combines_synchronized_position_and_attitude_nonblocking(self) -> None:
        attitude = types.SimpleNamespace(
            roll=0.1, pitch=-0.2, yaw=1.3, time_boot_ms=10_000,
            get_type=lambda: "ATTITUDE",
        )
        position = types.SimpleNamespace(
            lat=719958070, lon=-948393000, alt=75_000, relative_alt=60_000, time_boot_ms=10_100,
            get_type=lambda: "GLOBAL_POSITION_INT",
        )
        connection = _MessageConnection(attitude, position)
        adapter = ReadOnlyMavlink("quadcopter", "sim.invalid", 14550)
        adapter._connection = connection
        telemetry = adapter.poll()
        self.assertIsNotNone(telemetry)
        assert telemetry is not None
        self.assertTrue(telemetry.has_attitude)
        self.assertEqual(
            (telemetry.roll_rad, telemetry.pitch_rad, telemetry.yaw_rad),
            (0.1, -0.2, 1.3),
        )
        self.assertEqual(telemetry.altitude_m, 75.0)
        self.assertEqual(
            connection.calls,
            [
                {"type": ["ATTITUDE", "GLOBAL_POSITION_INT"], "blocking": False},
                {"type": ["ATTITUDE", "GLOBAL_POSITION_INT"], "blocking": False},
            ],
        )

    def test_missing_or_unsynchronized_attitude_is_not_reported_as_zero(self) -> None:
        position = types.SimpleNamespace(
            lat=719958070, lon=-948393000, alt=75_000, relative_alt=60_000, time_boot_ms=10_000,
            get_type=lambda: "GLOBAL_POSITION_INT",
        )
        connection = _MessageConnection(None, position)
        adapter = ReadOnlyMavlink("quadcopter", "sim.invalid", 14550)
        adapter._connection = connection
        telemetry = adapter.poll()
        assert telemetry is not None
        self.assertFalse(telemetry.has_attitude)
        self.assertEqual(
            (telemetry.roll_rad, telemetry.pitch_rad, telemetry.yaw_rad),
            (None, None, None),
        )

        attitude = types.SimpleNamespace(
            roll=0.0, pitch=0.0, yaw=0.0, time_boot_ms=1_000,
            get_type=lambda: "ATTITUDE",
        )
        position = types.SimpleNamespace(
            lat=719958070, lon=-948393000, alt=75_000, relative_alt=60_000, time_boot_ms=10_000,
            get_type=lambda: "GLOBAL_POSITION_INT",
        )
        adapter._connection = _MessageConnection(attitude, position)
        telemetry = adapter.poll()
        assert telemetry is not None
        self.assertFalse(telemetry.has_attitude)
        self.assertIsNone(telemetry.roll_rad)

    def test_stale_attitude_is_explicitly_absent(self) -> None:
        attitude = types.SimpleNamespace(
            roll=0.1, pitch=0.2, yaw=0.3, time_boot_ms=10_000,
            get_type=lambda: "ATTITUDE",
        )
        position = types.SimpleNamespace(
            lat=719958070, lon=-948393000, alt=75_000, relative_alt=60_000, time_boot_ms=10_000,
            get_type=lambda: "GLOBAL_POSITION_INT",
        )
        adapter = ReadOnlyMavlink(
            "quadcopter", "sim.invalid", 14550, attitude_max_age_s=0.5
        )
        adapter._connection = _MessageConnection(attitude, position)
        with patch("whiteout.mavlink.time.monotonic", side_effect=(1.0, 2.0)):
            telemetry = adapter.poll()
        assert telemetry is not None
        self.assertFalse(telemetry.has_attitude)
        self.assertIsNone(telemetry.attitude_timestamp)

    def test_position_heading_is_exposed_as_yaw(self) -> None:
        message = types.SimpleNamespace(
            lat=719900000,
            lon=-948200000,
            alt=25000,
            relative_alt=12500,
            hdg=9000,
            time_boot_ms=10_000,
            get_type=lambda: "GLOBAL_POSITION_INT",
        )
        adapter = ReadOnlyMavlink("quadcopter", "sim.invalid", 14550)
        adapter._connection = _MessageConnection(None, message)

        telemetry = adapter.poll()

        self.assertIsNotNone(telemetry)
        assert telemetry is not None
        self.assertAlmostEqual(telemetry.yaw_rad, pi / 2)


if __name__ == "__main__":
    unittest.main()
