"""Run one of the bundled ArcticSim search missions interactively."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from whiteout import Copter, CopterMode, Plane, VehicleStateError  # noqa: E402


DEFAULT_MISSIONS = {
    "quadcopter": ROOT / "missions" / "quadcopter_search.waypoints",
    "fixed-wing": ROOT / "missions" / "fixed_wing_search.waypoints",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Upload and run a bundled ArcticSim search mission."
    )
    parser.add_argument("aircraft", choices=tuple(DEFAULT_MISSIONS))
    parser.add_argument("--host", default="10.99.0.1")
    parser.add_argument("--mission", type=Path, help="override the bundled QGC WPL route")
    parser.add_argument(
        "--altitude-m",
        type=float,
        default=90.0,
        help="quadcopter takeoff altitude before AUTO (default: 90)",
    )
    parser.add_argument(
        "--upload-timeout-seconds",
        type=float,
        default=15.0,
        help="maximum wait for MAVProxy's Sent all confirmation",
    )
    parser.add_argument(
        "--connection-timeout-seconds",
        type=float,
        default=15.0,
        help="maximum wait for the vehicle heartbeat",
    )
    parser.add_argument(
        "--wait-disarm-seconds",
        type=float,
        default=180.0,
        help="maximum wait for disarm after an abort",
    )
    parser.add_argument(
        "--confirm-flight",
        action="store_true",
        help="required acknowledgement that the simulator vehicle will arm and move",
    )
    args = parser.parse_args()
    if not args.confirm_flight:
        parser.error("--confirm-flight is required")
    if args.altitude_m <= 0:
        parser.error("--altitude-m must be positive")
    if args.upload_timeout_seconds <= 0:
        parser.error("--upload-timeout-seconds must be positive")
    if args.connection_timeout_seconds <= 0:
        parser.error("--connection-timeout-seconds must be positive")
    if args.wait_disarm_seconds <= 0:
        parser.error("--wait-disarm-seconds must be positive")
    args.mission = (args.mission or DEFAULT_MISSIONS[args.aircraft]).expanduser()
    return args


def command_loop(controller: object, *, wait_disarm_seconds: float) -> None:
    """Handle operator commands after the mission has entered AUTO."""
    print("Search is running. Commands: pause, resume, abort, status, wait, clear, quit")
    rtl_requested = False
    while True:
        command = input("search> ").strip().lower()
        if command == "pause":
            if rtl_requested:
                print("RTL is already active; resume is unavailable")
            else:
                controller.pause_search()  # type: ignore[attr-defined]
                print("Search paused in LOITER")
        elif command == "resume":
            if rtl_requested:
                print("RTL is already active; resume is unavailable")
            else:
                controller.resume_search()  # type: ignore[attr-defined]
                print("Search resumed in AUTO")
        elif command in {"abort", "rtl"}:
            controller.abort_search()  # type: ignore[attr-defined]
            rtl_requested = True
            print("Search aborted; vehicle is returning to launch")
        elif command in {"quit", "exit"}:
            if not rtl_requested:
                controller.abort_search()  # type: ignore[attr-defined]
                print("Exiting; vehicle is returning to launch")
            return
        elif command == "status":
            controller.status()  # type: ignore[attr-defined]
            print("Status requested from MAVProxy")
        elif command == "wait":
            try:
                controller.wait_until_disarmed(timeout_s=wait_disarm_seconds)  # type: ignore[attr-defined]
            except VehicleStateError as exc:
                print(f"Disarm wait failed: {exc}")
            else:
                print("Vehicle is disarmed")
        elif command == "clear":
            try:
                controller.clear_search_mission()  # type: ignore[attr-defined]
            except VehicleStateError as exc:
                print(f"Cannot clear mission yet: {exc}")
            else:
                print("Search mission cleared")
        elif command in {"", "help", "?"}:
            print("Commands: pause, resume, abort, status, wait, clear, quit")
        else:
            print("Unknown command; use: pause, resume, abort, status, wait, clear, quit")


def run_quadcopter(args: argparse.Namespace) -> None:
    with Copter(host=args.host).controller(
        connection_timeout_s=args.connection_timeout_seconds
    ) as vehicle:
        mission = vehicle.upload_search_mission(
            args.mission, upload_timeout_s=args.upload_timeout_seconds
        )
        print(f"Uploaded and confirmed {mission.path}")
        try:
            input("Press Enter to arm and take off the quadcopter: ")
            vehicle.set_mode(CopterMode.GUIDED)
            vehicle.arm()
            vehicle.takeoff(args.altitude_m)
            input("After confirming the vehicle is safely airborne, press Enter for AUTO: ")
            vehicle.start_search()
            command_loop(vehicle, wait_disarm_seconds=args.wait_disarm_seconds)
        except KeyboardInterrupt:
            print("\nInterrupted; requesting RTL")
            vehicle.abort_search()
            raise


def run_fixed_wing(args: argparse.Namespace) -> None:
    with Plane(host=args.host).controller(
        connection_timeout_s=args.connection_timeout_seconds
    ) as vehicle:
        mission = vehicle.upload_search_mission(
            args.mission, upload_timeout_s=args.upload_timeout_seconds
        )
        print(f"Uploaded and confirmed {mission.path}")
        try:
            input("Press Enter to enter TAKEOFF mode and arm the fixed-wing: ")
            vehicle.takeoff()
            input("After confirming the launch is established, press Enter for AUTO: ")
            vehicle.start_search()
            command_loop(vehicle, wait_disarm_seconds=args.wait_disarm_seconds)
        except KeyboardInterrupt:
            print("\nInterrupted; requesting RTL")
            vehicle.abort_search()
            raise


def main() -> None:
    args = parse_args()
    if args.aircraft == "quadcopter":
        run_quadcopter(args)
    else:
        run_fixed_wing(args)


if __name__ == "__main__":
    main()
