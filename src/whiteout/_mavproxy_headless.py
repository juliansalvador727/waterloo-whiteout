"""Windows headless bootstrap for MAVProxy's non-interactive mode."""

from __future__ import annotations

import runpy
import os
import sys
from pathlib import Path


def _startup_script_paths() -> set[str]:
    """Return user MAVProxy startup scripts excluded from managed sessions."""
    paths: set[Path] = set()
    home = os.environ.get("HOME")
    if home:
        paths.add(Path(home) / ".mavinit.scr")
        paths.add(Path(home) / ".mavproxy" / "mavinit.scr")
    local_app_data = os.environ.get("LOCALAPPDATA")
    if local_app_data:
        paths.add(Path(local_app_data) / ".mavproxy" / "mavinit.scr")
    return {os.path.normcase(os.path.abspath(path)) for path in paths}


def _run_isolated(script: str) -> None:
    """Run upstream MAVProxy without importing an operator's startup script."""
    excluded = _startup_script_paths()
    original_exists = os.path.exists

    def isolated_exists(path: object) -> bool:
        try:
            normalized = os.path.normcase(os.path.abspath(os.fspath(path)))
        except TypeError:
            return original_exists(path)  # type: ignore[arg-type]
        if normalized in excluded:
            return False
        return original_exists(path)

    os.path.exists = isolated_exists
    try:
        runpy.run_path(script, run_name="__main__")
    finally:
        os.path.exists = original_exists


def main() -> None:
    if len(sys.argv) < 2:
        raise SystemExit("MAVProxy script path is required")
    script = sys.argv[1]
    sys.argv = [script, *sys.argv[2:]]

    # Upstream MAVProxy constructs its readline PromptSession even when
    # --non-interactive is present. prompt_toolkit's default Windows I/O then
    # requires a console screen buffer. Supply inert I/O only for this headless
    # path; MAVProxy itself still owns the link, modules, command processing,
    # and lifecycle.
    from prompt_toolkit.application import create_app_session
    from prompt_toolkit.input import DummyInput
    from prompt_toolkit.output import DummyOutput

    if "--non-interactive" in sys.argv:
        with create_app_session(input=DummyInput(), output=DummyOutput()):
            _run_isolated(script)
        return

    # prompt_toolkit cannot consume a redirected Windows stdin handle as a
    # console. Replace only MAVProxy's terminal reader with a blocking read on
    # the subprocess pipe; MAVProxy's input queue still parses and dispatches
    # every command.
    from MAVProxy.modules.lib import rline as mavproxy_rline

    def read_pipe_line(_reader: object) -> str:
        line = sys.stdin.readline()
        if line == "":
            raise EOFError
        return line.rstrip("\r\n")

    mavproxy_rline.rline.input = read_pipe_line
    with create_app_session(input=DummyInput(), output=DummyOutput()):
        _run_isolated(script)


if __name__ == "__main__":
    main()
