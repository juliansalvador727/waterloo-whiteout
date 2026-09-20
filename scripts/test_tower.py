"""Run a bounded, explicitly confirmed tower pan/tilt test in degrees."""

from __future__ import annotations

import argparse
import time
from typing import Sequence

from whiteout import Tower, tower_angle_to_pwm


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tower", choices=("tower-1", "tower-2"))
    parser.add_argument("--host", default="10.99.0.1")
    parser.add_argument("--pan-deg", type=float, default=45.0)
    parser.add_argument("--tilt-deg", type=float, default=20.0)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--connection-timeout-seconds", type=float, default=15.0)
    parser.add_argument(
        "--confirm-movement",
        action="store_true",
        help="required acknowledgement that this moves a simulator tower",
    )
    args = parser.parse_args(argv)
    if not args.confirm_movement:
        parser.error("--confirm-movement is required")
    if not 0 < args.hold_seconds <= 30:
        parser.error("--hold-seconds must be greater than 0 and at most 30")
    if not 0 < args.connection_timeout_seconds <= 60:
        parser.error(
            "--connection-timeout-seconds must be greater than 0 and at most 60"
        )
    try:
        tower_angle_to_pwm(1, args.pan_deg)
        tower_angle_to_pwm(2, args.tilt_deg)
    except ValueError as exc:
        parser.error(str(exc))
    return args


def run_test(
    tower_name: str,
    host: str,
    pan_deg: float,
    tilt_deg: float,
    hold_seconds: float,
    connection_timeout_seconds: float,
) -> None:
    tower = Tower.one(host) if tower_name == "tower-1" else Tower.two(host)
    pan_pwm = tower_angle_to_pwm(1, pan_deg)
    tilt_pwm = tower_angle_to_pwm(2, tilt_deg)

    print(
        f"Connecting to {tower.name}; target pan={pan_deg:g} degrees ({pan_pwm} PWM), "
        f"tilt={tilt_deg:g} degrees ({tilt_pwm} PWM)"
    )
    with tower.controller(
        connection_timeout_s=connection_timeout_seconds
    ) as controller:
        print("Centering tower at pan=0 degrees, tilt=7.5 degrees")
        controller.center()
        time.sleep(hold_seconds)
        try:
            print("Moving tower to requested angles")
            controller.pan(pan_deg)
            controller.tilt(tilt_deg)
            time.sleep(hold_seconds)
        finally:
            print("Returning tower to center")
            controller.center()


def main() -> None:
    args = parse_args()
    run_test(
        args.tower,
        args.host,
        args.pan_deg,
        args.tilt_deg,
        args.hold_seconds,
        args.connection_timeout_seconds,
    )


if __name__ == "__main__":
    main()
