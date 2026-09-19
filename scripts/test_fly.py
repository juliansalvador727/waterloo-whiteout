"""Bounded, explicitly confirmed arctic-sim flight using typed Python APIs."""

from __future__ import annotations

import argparse
import time

from whiteout import Copter, CopterMode, Plane


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("aircraft", choices=("quadcopter", "fixed-wing"))
    parser.add_argument("--host", default="10.99.0.1")
    parser.add_argument("--hold-seconds", type=float, default=15.0)
    parser.add_argument("--altitude-m", type=float, default=10.0)
    parser.add_argument(
        "--confirm-flight",
        action="store_true",
        help="required acknowledgement that this will arm and move the simulator aircraft",
    )
    args = parser.parse_args()
    if not args.confirm_flight:
        parser.error("--confirm-flight is required")
    if not 0 < args.hold_seconds <= 300:
        parser.error("--hold-seconds must be greater than 0 and at most 300")
    if not 0 < args.altitude_m <= 50:
        parser.error("--altitude-m must be greater than 0 and at most 50")
    return args


def fly_quadcopter(host: str, altitude_m: float, hold_seconds: float) -> None:
    with Copter(host=host).controller() as copter:
        print("Quadcopter online; entering GUIDED mode and taking off")
        copter.set_mode(CopterMode.GUIDED)
        copter.arm()
        copter.takeoff(altitude_m)
        try:
            time.sleep(hold_seconds)
        finally:
            print("Returning quadcopter to launch")
            copter.return_to_launch()


def fly_fixed_wing(host: str, hold_seconds: float) -> None:
    with Plane(host=host).controller() as plane:
        print("Fixed-wing online; entering TAKEOFF mode and arming")
        plane.takeoff()
        try:
            time.sleep(hold_seconds)
        finally:
            print("Returning fixed-wing to launch")
            plane.return_to_launch()


def main() -> None:
    args = parse_args()
    if args.aircraft == "quadcopter":
        fly_quadcopter(args.host, args.altitude_m, args.hold_seconds)
    else:
        fly_fixed_wing(args.host, args.hold_seconds)


if __name__ == "__main__":
    main()
