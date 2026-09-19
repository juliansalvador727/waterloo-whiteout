#!/usr/bin/env python3
"""Send a synthetic track only after both configuration and CLI opt-in."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone

from whiteout.config import load_config
from whiteout.models import Track
from whiteout.track_api import TrackApiClient


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--confirm", action="store_true", help="explicitly permit one POST")
    args = parser.parse_args()
    if not args.confirm:
        parser.error("submission requires --confirm")
    config = load_config(args.config)
    if not config.track_api.allow_submission or not config.track_api.endpoint:
        parser.error("configuration must enable submission and specify endpoint")
    test_track = Track("test-only", 0.0, 0.0, 0.0, 0.0, 9999.0, datetime.now(timezone.utc))
    client = TrackApiClient(
        config.track_api.endpoint,
        name=config.track_api.name,
        allow_submission=config.track_api.allow_submission,
        include_speed=config.track_api.include_speed,
    )
    client.submit(test_track, confirmed=args.confirm)
    print("test track submitted")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
