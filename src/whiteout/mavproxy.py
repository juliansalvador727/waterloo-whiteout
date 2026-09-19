"""Managed MAVProxy subprocess sessions for arctic-sim assets."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import threading
from typing import TYPE_CHECKING, Iterable, Sequence

if TYPE_CHECKING:
    from .objects import MavProxyCommand


class MavProxyUnavailable(RuntimeError):
    """Raised when the MAVProxy executable cannot be found."""


class MavProxyNotRunning(RuntimeError):
    """Raised when a command is sent without a live MAVProxy process."""


class MavProxySession:
    """A persistent MAVProxy console connected to one simulator asset.

    MAVProxy is launched as an argument vector, never through a shell. Commands
    are written to its console on stdin, so all vehicle communication continues
    to use MAVProxy's modules and routing.
    """

    def __init__(
        self,
        master: str,
        *,
        executable: str = "mavproxy.py",
        extra_arguments: Sequence[str] = (),
        startup_commands: Sequence[MavProxyCommand | str] = (),
    ) -> None:
        if not master.startswith("udpout:"):
            raise ValueError("arctic-sim MAVProxy masters must use udpout")
        self.master = master
        self.executable = executable
        self.extra_arguments = tuple(extra_arguments)
        self.startup_commands = tuple(startup_commands)
        self._validate_arguments()
        self._process: subprocess.Popen[str] | None = None
        self._write_lock = threading.Lock()

    def _validate_arguments(self) -> None:
        for argument in self.extra_arguments:
            if not argument or "\x00" in argument or "\n" in argument or "\r" in argument:
                raise ValueError("MAVProxy arguments must be non-empty single-line strings")
            if argument == "--master" or argument.startswith("--master="):
                raise ValueError("master is set by the repository object")

    @property
    def arguments(self) -> tuple[str, ...]:
        return (self.executable, f"--master={self.master}", *self.extra_arguments)

    @property
    def running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def _resolve_executable(self) -> str:
        resolved = shutil.which(self.executable)
        if resolved is not None:
            return resolved
        path = Path(self.executable)
        if path.is_file():
            return os.fspath(path)
        raise MavProxyUnavailable(
            f"{self.executable!r} was not found; install the whiteout[mavproxy] extra"
        )

    def start(self) -> MavProxySession:
        if self.running:
            return self
        executable = self._resolve_executable()
        arguments = (executable, *self.arguments[1:])
        self._process = subprocess.Popen(
            arguments,
            stdin=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            bufsize=1,
        )
        self.send_all(self.startup_commands)
        return self

    def send(self, command: MavProxyCommand | str) -> None:
        """Send one console command to the running MAVProxy process."""
        text = command if isinstance(command, str) else command.render()
        if not text or "\x00" in text or "\n" in text or "\r" in text:
            raise ValueError("MAVProxy commands must be non-empty single-line strings")
        with self._write_lock:
            if not self.running or self._process is None or self._process.stdin is None:
                raise MavProxyNotRunning("MAVProxy session is not running")
            self._process.stdin.write(f"{text}\n")
            self._process.stdin.flush()

    def send_all(self, commands: Iterable[MavProxyCommand | str]) -> None:
        for command in commands:
            self.send(command)

    def close(self, *, timeout: float = 5.0) -> None:
        """Ask MAVProxy to exit, terminating only if it does not respond."""
        process = self._process
        if process is None:
            return
        if process.poll() is None:
            try:
                if process.stdin is not None:
                    process.stdin.write("exit\n")
                    process.stdin.flush()
                process.wait(timeout=timeout)
            except (BrokenPipeError, subprocess.TimeoutExpired):
                process.terminate()
                try:
                    process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=timeout)
        self._process = None

    def __enter__(self) -> MavProxySession:
        return self.start()

    def __exit__(self, *args: object) -> None:
        self.close()
