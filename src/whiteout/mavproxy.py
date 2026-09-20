"""Managed MAVProxy subprocess sessions for arctic-sim assets."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import time
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
        non_interactive: bool = False,
    ) -> None:
        if not master.startswith("udpout:"):
            raise ValueError("arctic-sim MAVProxy masters must use udpout")
        self.master = master
        self.executable = executable
        self.extra_arguments = tuple(extra_arguments)
        self.startup_commands = tuple(startup_commands)
        self.non_interactive = non_interactive
        self._validate_arguments()
        self._process: subprocess.Popen[str] | None = None
        self._write_lock = threading.Lock()
        self._output_condition = threading.Condition()
        self._output_lines: list[str] = []
        self._reader_thread: threading.Thread | None = None
        self._working_directory: tempfile.TemporaryDirectory[str] | None = None

    def _validate_arguments(self) -> None:
        for argument in self.extra_arguments:
            if not argument or "\x00" in argument or "\n" in argument or "\r" in argument:
                raise ValueError("MAVProxy arguments must be non-empty single-line strings")
            if argument == "--master" or argument.startswith("--master="):
                raise ValueError("master is set by the repository object")

    @property
    def arguments(self) -> tuple[str, ...]:
        # MAVProxy does not load its optional GUI console unless ``--console``
        # is supplied.  Older/current releases do not provide a corresponding
        # ``--no-console`` flag, so omitting ``--console`` is the portable
        # headless setting.  Keep only the modules used by the typed control
        # surface and disable state/log files.
        arguments = [
            self.executable,
            f"--master={self.master}",
            "--no-state",
            "--default-modules=wp,param,arm,mode,rc,misc,cmdlong,battery",
            *self.extra_arguments,
        ]
        if self.non_interactive:
            arguments.append("--non-interactive")
            arguments.extend(f"--cmd={self._command_text(command)}" for command in self.startup_commands)
        return tuple(arguments)

    @property
    def running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    @property
    def output(self) -> str:
        with self._output_condition:
            return "".join(self._output_lines)

    @staticmethod
    def _command_text(command: MavProxyCommand | str) -> str:
        text = command if isinstance(command, str) else command.render()
        if not text or "\x00" in text or "\n" in text or "\r" in text:
            raise ValueError("MAVProxy commands must be non-empty single-line strings")
        return text

    def _resolve_launcher(self) -> tuple[str, ...]:
        resolved = shutil.which(self.executable)
        if resolved is None:
            requested = Path(self.executable)
            candidates = (requested, Path(sys.executable).with_name(self.executable))
            resolved = next((os.fspath(path) for path in candidates if path.is_file()), None)
        if resolved is None:
            raise MavProxyUnavailable(
                f"{self.executable!r} was not found; install the whiteout[mavproxy] extra"
            )
        if os.name == "nt" and Path(resolved).suffix.lower() == ".py":
            return (sys.executable, "-u", "-m", "whiteout._mavproxy_headless", resolved)
        return (resolved,)

    def start(self) -> MavProxySession:
        if self.running:
            return self
        with self._output_condition:
            self._output_lines.clear()
        launcher = self._resolve_launcher()
        arguments = (*launcher, *self.arguments[1:])
        # MAVProxy and some modules create mav.tlog, mav.tlog.raw and parameter
        # snapshots even with --no-state.  Isolate those runtime artifacts from
        # the caller's repository and remove them when the session closes.
        self._working_directory = tempfile.TemporaryDirectory(
            prefix="whiteout-mavproxy-"
        )
        try:
            self._process = subprocess.Popen(
                arguments,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                bufsize=1,
                cwd=self._working_directory.name,
            )
        except Exception:
            self._working_directory.cleanup()
            self._working_directory = None
            raise
        output_stream = self._process.stdout
        if output_stream is not None:
            self._reader_thread = threading.Thread(
                target=self._read_output,
                args=(output_stream,),
                daemon=True,
                name="whiteout-mavproxy-output",
            )
            self._reader_thread.start()
        if not self.non_interactive:
            self.send_all(self.startup_commands)
        return self

    def _read_output(self, stream: object) -> None:
        try:
            for line in stream:  # type: ignore[union-attr]
                with self._output_condition:
                    self._output_lines.append(line)
                    self._output_condition.notify_all()
        finally:
            stream.close()  # type: ignore[union-attr]
            with self._output_condition:
                self._output_condition.notify_all()

    def wait_for_output(
        self, text: str, *, timeout: float = 10.0, start: int = 0
    ) -> bool:
        """Wait until captured MAVProxy output contains ``text``."""
        if start < 0:
            raise ValueError("output start position cannot be negative")
        deadline = time.monotonic() + timeout
        with self._output_condition:
            while text not in "".join(self._output_lines)[start:]:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or (self._process is not None and not self.running):
                    return False
                self._output_condition.wait(timeout=remaining)
            return True

    def send(self, command: MavProxyCommand | str) -> None:
        """Send one console command to the running MAVProxy process."""
        if self.non_interactive:
            raise MavProxyNotRunning(
                "non-interactive MAVProxy commands must be supplied before start"
            )
        text = self._command_text(command)
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
        try:
            if process.poll() is None:
                try:
                    if self.non_interactive:
                        process.terminate()
                    elif process.stdin is not None:
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
        finally:
            if process.stdin is not None:
                process.stdin.close()
            if self._reader_thread is not None:
                self._reader_thread.join(timeout=min(timeout, 1.0))
                self._reader_thread = None
            self._process = None
            if self._working_directory is not None:
                self._working_directory.cleanup()
                self._working_directory = None

    def __enter__(self) -> MavProxySession:
        return self.start()

    def __exit__(self, *args: object) -> None:
        self.close()
