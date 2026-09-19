#!/usr/bin/env python3
"""Opt-in MJPEG recorder that writes individual JPEG frames."""

from __future__ import annotations

import argparse
from pathlib import Path

from whiteout.camera import MjpegCamera
from whiteout.config import load_config


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--camera", required=True)
    parser.add_argument("--output", type=Path, default=Path("recordings"))
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--confirm-network", action="store_true")
    args = parser.parse_args()
    if not args.confirm_network:
        parser.error("camera recording requires --confirm-network")
    if args.frames < 1:
        parser.error("--frames must be positive")

    config = load_config(args.config)
    selected = next((item for item in config.cameras if item.name == args.camera), None)
    if selected is None:
        parser.error(f"unknown camera {args.camera!r}")
    args.output.mkdir(parents=True, exist_ok=True)
    url = f"http://{config.sim_host}:{selected.port}{selected.path}"
    camera = MjpegCamera(selected.name, url)
    for index, frame in enumerate(camera.frames(), start=1):
        (args.output / f"{selected.name}-{index:06d}.jpg").write_bytes(frame.jpeg)
        if index >= args.frames:
            break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

