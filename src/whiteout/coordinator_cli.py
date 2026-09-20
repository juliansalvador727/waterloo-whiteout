"""Interactive command for the unified live search and tracking runtime."""

from __future__ import annotations

import argparse
import getpass
import threading
from urllib.parse import urlparse

from .config import AppConfig, load_config
from .runtime import UnifiedCoordinatorRuntime


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="whiteout coordinator",
        description="Run both search aircraft, both towers, tracking, dashboard, and submissions.",
    )
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--confirm-network", action="store_true")
    parser.add_argument("--confirm-flight", action="store_true")
    parser.add_argument("--submit-tracks", action="store_true")
    parser.add_argument("--operator", default=getpass.getuser())
    parser.add_argument("--session", help="directory for deterministic session recording")
    parser.add_argument("--quadcopter-altitude-m", type=float, default=90.0)
    return parser


def validate_live_config(config: AppConfig, *, submit_tracks: bool) -> None:
    if config.course_bounds is None:
        raise ValueError("coordinator requires configured course_bounds")
    if submit_tracks:
        if not config.track_api.allow_submission:
            raise ValueError("--submit-tracks requires track_api.allow_submission: true")
        endpoint = urlparse(config.track_api.endpoint or "")
        if endpoint.scheme not in {"http", "https"} or not endpoint.netloc:
            raise ValueError("track API endpoint must be an absolute HTTP(S) URL")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.confirm_network:
        raise SystemExit("refusing simulator connections without --confirm-network")
    if not args.confirm_flight:
        raise SystemExit("refusing aircraft or tower movement without --confirm-flight")
    if args.quadcopter_altitude_m <= 0:
        raise SystemExit("--quadcopter-altitude-m must be positive")
    config = load_config(args.config)
    try:
        validate_live_config(config, submit_tracks=args.submit_tracks)
        runtime = UnifiedCoordinatorRuntime.build_live(
            config,
            submit_tracks=args.submit_tracks,
            operator=args.operator,
            session_directory=args.session,
        )
    except (OSError, RuntimeError, ValueError) as exc:
        raise SystemExit(f"coordinator preflight failed: {exc}") from exc

    run_thread: threading.Thread | None = None
    try:
        print(f"Simulator: {config.sim_host}")
        print(f"Detector: {config.coordinator.detector_weights}")
        print(f"Track submission: {'ENABLED' if args.submit_tracks else 'DISABLED'}")
        runtime.connect()
        runtime.upload_missions()
        input(f"Press Enter to activate operator session for {args.operator}: ")
        runtime.activate_operator_session()
        runtime.calibrate_towers()
        run_thread = threading.Thread(target=runtime.run, name="coordinator-loop", daemon=True)
        run_thread.start()

        input("Press Enter to arm and take off the quadcopter: ")
        runtime.launch_quadcopter(args.quadcopter_altitude_m)
        input("After confirming the quadcopter is safely airborne, press Enter for AUTO: ")
        runtime.start_search("quadcopter")

        input("Press Enter to enter TAKEOFF mode and arm the fixed-wing: ")
        runtime.launch_fixed_wing()
        input("After confirming the fixed-wing launch is established, press Enter for AUTO: ")
        runtime.start_search("fixed-wing")

        print("Unified search is running.")
        print("Commands: status <asset|all>, pause <asset|all>, resume <asset|all>, abort <asset|all>, quit")
        while True:
            parts = input("coordinator> ").strip().lower().split()
            if not parts:
                continue
            if parts[0] in {"quit", "exit"}:
                break
            if parts[0] in {"help", "?"}:
                print("Commands: status <asset|all>, pause <asset|all>, resume <asset|all>, abort <asset|all>, quit")
                continue
            if len(parts) != 2 or parts[0] not in {"status", "pause", "resume", "abort"}:
                print("Unknown command; use help")
                continue
            try:
                runtime.command(parts[0], parts[1])
            except (RuntimeError, ValueError) as exc:
                print(f"Command failed: {exc}")
    except KeyboardInterrupt:
        print("\nInterrupted; stopping submissions and requesting RTL")
    finally:
        runtime.close(request_rtl=True)
        if run_thread is not None and run_thread.is_alive():
            run_thread.join(timeout=2.0)
    return 0
