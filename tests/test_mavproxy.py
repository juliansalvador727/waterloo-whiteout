from __future__ import annotations

import io
import unittest
from unittest.mock import patch

from whiteout.mavproxy import MavProxyNotRunning, MavProxySession, MavProxyUnavailable
from whiteout.objects import Boat, Copter, Tower, UnsupportedCommand


class _FakeProcess:
    def __init__(self) -> None:
        self.stdin = io.StringIO()
        self.returncode: int | None = None
        self.terminated = False
        self.killed = False

    def poll(self) -> int | None:
        return self.returncode

    def wait(self, timeout: float | None = None) -> int:
        self.returncode = 0
        return 0

    def terminate(self) -> None:
        self.terminated = True
        self.returncode = 0

    def kill(self) -> None:
        self.killed = True
        self.returncode = -9


class MavProxyTests(unittest.TestCase):
    def test_session_launches_mavproxy_with_udpout_and_sends_console_commands(self) -> None:
        process = _FakeProcess()
        with (
            patch("whiteout.mavproxy.shutil.which", return_value="C:/bin/mavproxy.py"),
            patch("whiteout.mavproxy.subprocess.Popen", return_value=process) as popen,
        ):
            copter = Copter(host="sim.example")
            session = copter.connect_mavproxy()
            session.send(copter.mode("GUIDED"))
            session.send(copter.arm())
            session.send(copter.takeoff(20))
            session.close()

        arguments = popen.call_args.args[0]
        self.assertEqual(
            arguments,
            ("C:/bin/mavproxy.py", "--master=udpout:sim.example:14550"),
        )
        self.assertFalse(popen.call_args.kwargs.get("shell", False))
        self.assertEqual(
            process.stdin.getvalue(),
            "mode GUIDED\narm throttle\ntakeoff 20\nexit\n",
        )

    def test_context_manager_starts_and_closes_session(self) -> None:
        process = _FakeProcess()
        with (
            patch("whiteout.mavproxy.shutil.which", return_value="mavproxy.py"),
            patch("whiteout.mavproxy.subprocess.Popen", return_value=process),
        ):
            tower = Tower.two("arctic.example")
            with tower.mavproxy_session() as session:
                session.send(tower.pan(1200))
        self.assertEqual(process.stdin.getvalue(), "module load servo\nservo set 1 1200\nexit\n")

    def test_missing_mavproxy_has_clear_install_message(self) -> None:
        with patch("whiteout.mavproxy.shutil.which", return_value=None):
            with self.assertRaisesRegex(MavProxyUnavailable, r"whiteout\[mavproxy\]"):
                Copter().connect_mavproxy(executable="definitely-not-mavproxy")

    def test_invalid_usage_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            MavProxySession("udp:127.0.0.1:14550")
        with self.assertRaises(ValueError):
            MavProxySession("udpout:127.0.0.1:14550", extra_arguments=("--master=x",))
        with self.assertRaises(UnsupportedCommand):
            Boat().mavproxy_session()
        with self.assertRaises(MavProxyNotRunning):
            Copter().mavproxy_session().send("status")


if __name__ == "__main__":
    unittest.main()
