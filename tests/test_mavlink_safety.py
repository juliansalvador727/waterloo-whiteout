from __future__ import annotations

import sys
import types
import unittest
from unittest.mock import patch

from whiteout.mavlink import ReadOnlyMavlink


class _FakeMav:
    def __init__(self) -> None:
        self.heartbeats: list[tuple[int, ...]] = []

    def heartbeat_send(self, *values: int) -> None:
        self.heartbeats.append(values)


class _FakeConnection:
    def __init__(self) -> None:
        self.mav = _FakeMav()

    def recv_match(self, **kwargs: object) -> None:
        return None


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


if __name__ == "__main__":
    unittest.main()
