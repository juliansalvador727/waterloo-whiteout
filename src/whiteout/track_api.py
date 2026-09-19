"""Safety-gated outbound client for the Dominion track API."""

from __future__ import annotations

import json
import math
import urllib.request
from collections.abc import Callable
from typing import Any

from .models import Track


def track_payload(track: Track, *, name: str, include_speed: bool = False) -> dict[str, Any]:
    """Map an internal track to the official Dominion payload."""
    payload: dict[str, Any] = {
        "name": name,
        "lat": track.latitude,
        "lon": track.longitude,
    }
    north = track.velocity_north_mps
    east = track.velocity_east_mps
    if north != 0.0 or east != 0.0:
        payload["heading"] = math.degrees(math.atan2(east, north)) % 360.0
        if include_speed:
            payload["speed"] = math.hypot(north, east)
    return payload


class TrackApiClient:
    """POST tracks only when configuration and the individual call both opt in."""

    def __init__(
        self,
        endpoint: str | None,
        *,
        name: str = "Sierra One",
        allow_submission: bool = False,
        include_speed: bool = False,
        timeout_s: float = 5.0,
        opener: Callable[..., object] = urllib.request.urlopen,
    ) -> None:
        self.endpoint = endpoint
        self.name = name
        self.allow_submission = allow_submission
        self.include_speed = include_speed
        self.timeout_s = timeout_s
        self._opener = opener

    def submit(self, track: Track, *, confirmed: bool = False) -> None:
        if not (self.allow_submission and confirmed):
            raise PermissionError("track submission requires config opt-in and explicit confirmation")
        if not self.endpoint:
            raise ValueError("track API endpoint is not configured")
        request = urllib.request.Request(
            self.endpoint,
            data=json.dumps(
                track_payload(track, name=self.name, include_speed=self.include_speed),
                separators=(",", ":"),
            ).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with self._opener(request, timeout=self.timeout_s) as response:  # type: ignore[attr-defined]
            status = getattr(response, "status", None)
            if status is not None and not 200 <= status < 300:
                raise RuntimeError(f"track submission failed with HTTP {status}")
