from __future__ import annotations

import io
import os
import sys
import unittest
from unittest.mock import patch

from whiteout.mavproxy import MavProxyNotRunning, MavProxySession, MavProxyUnavailable
from whiteout.objects import Boat, Copter, Tower, UnsupportedCommand


class _FakeStdin(io.StringIO):
    def __init__(self) -> None:
        super().__init__()
        self.was_closed = False

    def close(self) -> None:
        self.was_closed = True


class _FakeProcess:
    def __init__(self) -> None:
        self.stdin = _FakeStdin()
        self.stdout = None
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
        expected_launcher = (
            (sys.executable, "-u", "-m", "whiteout._mavproxy_headless", "C:/bin/mavproxy.py")
            if os.name == "nt"
            else ("C:/bin/mavproxy.py",)
        )
        self.assertEqual(
            arguments,
            (
                *expected_launcher,
                "--master=udpout:sim.example:14550",
                "--no-console",
                "--no-state",
                "--default-modules=wp,param,arm,mode,rc,misc,cmdlong,battery",
            ),
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
                session.send(tower.pan(-108))
        self.assertEqual(
            process.stdin.getvalue(),
            "module load relay\nservo set 1 1200\nexit\n",
        )

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

    def test_non_interactive_commands_use_mavproxy_cmd_arguments(self) -> None:
        session = Tower.one().mavproxy_session(
            startup_commands=("status",),
            non_interactive=True,
        )
        self.assertIn("--non-interactive", session.arguments)
        self.assertIn("--no-state", session.arguments)
        self.assertIn("--cmd=status", session.arguments)

    def test_windows_non_interactive_launcher_uses_headless_bootstrap(self) -> None:
        session = Copter().mavproxy_session(non_interactive=True)
        with patch("whiteout.mavproxy.shutil.which", return_value="C:/bin/mavproxy.py"):
            launcher = session._resolve_launcher()
        if os.name == "nt":
            self.assertEqual(
                launcher,
                (sys.executable, "-u", "-m", "whiteout._mavproxy_headless", "C:/bin/mavproxy.py"),
            )
        else:
            self.assertEqual(launcher, ("C:/bin/mavproxy.py",))


if __name__ == "__main__":
    unittest.main()
