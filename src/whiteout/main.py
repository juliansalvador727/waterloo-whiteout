"""WHITEOUT pipeline command-line entry point."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import asdict

from .camera import MjpegCamera
from .config import AppConfig, load_config
from .coordinator import Coordinator
from .detector import NoOpDetector
from .telemetry_log import JsonlLogger


def submission_enabled(config: AppConfig, cli_opt_in: bool) -> bool:
    return bool(cli_opt_in and config.track_api.allow_submission)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the WHITEOUT observation pipeline")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument(
        "--submit-tracks",
        action="store_true",
        help="allow outbound track POSTs only when config also opts in",
    )
    parser.add_argument("--check-config", action="store_true", help="validate configuration and exit")
    parser.add_argument("--observe-camera", metavar="NAME", help="run the no-op detector on a camera")
    parser.add_argument("--max-frames", type=int, default=1, help="bounded frame count for observation")
    parser.add_argument("--confirm-network", action="store_true", help="permit the requested camera connection")
    return parser


def observe_camera(config: AppConfig, name: str, max_frames: int) -> int:
    if max_frames < 1:
        raise ValueError("max_frames must be positive")
    selected = next((camera for camera in config.cameras if camera.name == name), None)
    if selected is None:
        raise ValueError(f"unknown camera {name!r}")
    detector = NoOpDetector()
    coordinator = Coordinator()
    logger = JsonlLogger(config.logging.jsonl_path) if config.logging.jsonl_path else None
    camera = MjpegCamera(name, f"http://{config.sim_host}:{selected.port}{selected.path}")
    for count, frame in enumerate(camera.frames(), start=1):
        detections = detector.detect(frame)
        recommendation = coordinator.decide(time.monotonic(), None)
        if logger:
            logger.write(
                "frame_processed",
                {"camera": name, "detections": len(detections), "recommendation": asdict(recommendation)},
            )
        if count >= max_frames:
            return count
    return 0


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if arguments and arguments[0] == "coordinator":
        from .coordinator_cli import main as coordinator_main

        return coordinator_main(arguments[1:])
    args = build_parser().parse_args(arguments)
    config = load_config(args.config)
    if args.submit_tracks and not config.track_api.allow_submission:
        raise SystemExit("refusing submission: track_api.allow_submission is false")
    if args.check_config:
        print(f"configuration valid for {config.placement_owner}; no network connections opened")
        return 0
    if args.observe_camera:
        if not args.confirm_network:
            raise SystemExit("refusing camera connection without --confirm-network")
        try:
            processed = observe_camera(config, args.observe_camera, args.max_frames)
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc
        print(f"processed {processed} frame(s); no flight commands or track submissions were sent")
        return 0
    print("WHITEOUT scaffold ready. No flight commands or outbound submissions were sent.")
    print("Integrate a detector and explicit pipeline runner before live observation use.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
