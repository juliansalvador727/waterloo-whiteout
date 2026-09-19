"""Bounded, explicitly confirmed arctic-sim flight using typed Python APIs."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from whiteout import Copter, CopterMode, Plane


DEFAULT_LANDING_MISSION = (
    Path(__file__).resolve().parents[1]
    / "missions"
    / "arctic_sim_fixed_wing_land.waypoints"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("aircraft", choices=("quadcopter", "fixed-wing"))
    parser.add_argument("--host", default="10.99.0.1")
    parser.add_argument("--hold-seconds", type=float)
    parser.add_argument("--landing-seconds", type=float)
    parser.add_argument("--altitude-m", type=float, default=10.0)
    parser.add_argument(
        "--landing-mission",
        type=Path,
        default=DEFAULT_LANDING_MISSION,
        help="ArduPlane QGC WPL landing mission",
    )
    parser.add_argument(
        "--confirm-flight",
        action="store_true",
        help="required acknowledgement that this will arm and move the simulator aircraft",
    )
    args = parser.parse_args()
    if not args.confirm_flight:
        parser.error("--confirm-flight is required")
    if args.hold_seconds is not None and not 0 < args.hold_seconds <= 300:
        parser.error("--hold-seconds must be greater than 0 and at most 300")
    if args.landing_seconds is not None and not 0 < args.landing_seconds <= 300:
        parser.error("--landing-seconds must be greater than 0 and at most 300")
    if not 0 < args.altitude_m <= 50:
        parser.error("--altitude-m must be greater than 0 and at most 50")
    return args


def fly_quadcopter(
    host: str, altitude_m: float, hold_seconds: float, landing_seconds: float
) -> None:
    with Copter(host=host).controller() as copter:
        print("Quadcopter online; entering GUIDED mode and taking off")
        copter.set_mode(CopterMode.GUIDED)
        copter.arm()
        copter.takeoff(altitude_m)
        try:
            time.sleep(hold_seconds)
        finally:
            print("Autolanding quadcopter")
            copter.autoland()
            time.sleep(landing_seconds)


def fly_fixed_wing(
    host: str,
    hold_seconds: float,
    landing_seconds: float,
    landing_mission: Path,
) -> None:
    with Plane(host=host).controller() as plane:
        print("Fixed-wing online; entering TAKEOFF mode and arming")
        plane.takeoff()
        try:
            time.sleep(hold_seconds)
        finally:
            print(f"Autolanding fixed-wing with {landing_mission}")
            plane.autoland(landing_mission)
            time.sleep(landing_seconds)


def main() -> None:
    args = parse_args()
    if args.aircraft == "quadcopter":
        fly_quadcopter(
            args.host,
            args.altitude_m,
            args.hold_seconds or 15.0,
            args.landing_seconds or 45.0,
        )
    else:
        fly_fixed_wing(
            args.host,
            args.hold_seconds or 45.0,
            args.landing_seconds or 120.0,
            args.landing_mission,
        )


if __name__ == "__main__":
    main()
