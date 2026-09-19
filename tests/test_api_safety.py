from __future__ import annotations

import json
import unittest
from datetime import datetime, timezone

import whiteout.track_api as track_api
from whiteout.models import Track
from whiteout.track_api import TrackApiClient, track_payload


class _FakeResponse:
    status = 202

    def __enter__(self) -> _FakeResponse:
        return self

    def __exit__(self, *args: object) -> None:
        return None


class _FakeOpener:
    def __init__(self) -> None:
        self.calls: list[tuple[object, float]] = []

    def __call__(self, request: object, *, timeout: float) -> _FakeResponse:
        self.calls.append((request, timeout))
        return _FakeResponse()


def _track(north: float = 0.0, east: float = 0.0) -> Track:
    return Track("internal-id", 43.47, -80.54, north, east, 4.0, datetime.now(timezone.utc))


class ApiSafetyTests(unittest.TestCase):
    def test_submission_requires_config_and_call_confirmation(self) -> None:
        for allowed, confirmed in ((False, False), (False, True), (True, False)):
            with self.subTest(allowed=allowed, confirmed=confirmed):
                opener = _FakeOpener()
                client = TrackApiClient(
                    "https://dominion.invalid/tracks",
                    allow_submission=allowed,
                    opener=opener,
                )
                with self.assertRaises(PermissionError):
                    client.submit(_track(), confirmed=confirmed)
                self.assertEqual(opener.calls, [])

    def test_official_payload_and_injected_offline_transport(self) -> None:
        opener = _FakeOpener()
        client = TrackApiClient(
            "https://dominion.invalid/tracks",
            name="Sierra One",
            allow_submission=True,
            opener=opener,
        )
        client.submit(_track(north=0.0, east=2.0), confirmed=True)

        self.assertEqual(len(opener.calls), 1)
        request, timeout = opener.calls[0]
        self.assertEqual(request.full_url, "https://dominion.invalid/tracks")
        self.assertEqual(request.get_method(), "POST")
        self.assertEqual(timeout, 5.0)
        self.assertEqual(
            json.loads(request.data),
            {"name": "Sierra One", "lat": 43.47, "lon": -80.54, "heading": 90.0},
        )

    def test_heading_is_clockwise_from_north_and_speed_is_opt_in(self) -> None:
        without_speed = track_payload(_track(north=-1.0, east=0.0), name="Sierra One")
        self.assertEqual(without_speed["heading"], 180.0)
        self.assertNotIn("speed", without_speed)

        with_speed = track_payload(_track(north=3.0, east=4.0), name="Sierra One", include_speed=True)
        self.assertAlmostEqual(with_speed["heading"], 53.13010235415598)
        self.assertEqual(with_speed["speed"], 5.0)

    def test_track_api_exposes_no_inbound_server(self) -> None:
        self.assertFalse(hasattr(track_api, "create_server"))
        self.assertFalse(hasattr(track_api, "TrackStore"))


if __name__ == "__main__":
    unittest.main()
