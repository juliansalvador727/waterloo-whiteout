#!/usr/bin/env python3
"""Explicit, read-only TCP reachability checks for configured endpoints."""

from __future__ import annotations

import argparse
import socket

from whiteout.config import load_config


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument(
        "--confirm-network",
        action="store_true",
        help="explicitly permit read-only connection checks",
    )
    args = parser.parse_args()
    if not args.confirm_network:
        parser.error("network checks require --confirm-network")
    config = load_config(args.config)
    failures = 0
    for camera in config.cameras:
        try:
            with socket.create_connection((config.sim_host, camera.port), timeout=1):
                print(f"reachable: camera {camera.name} on {config.sim_host}:{camera.port}")
        except OSError as exc:
            failures += 1
            print(f"unreachable: camera {camera.name}: {exc}")
    print("MAVLink uses UDP; no packets were transmitted to probe it.")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())

