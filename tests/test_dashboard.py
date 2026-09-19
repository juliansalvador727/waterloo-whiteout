from __future__ import annotations

import importlib.util
import json
import tempfile
import threading
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path

from whiteout.camera import CameraFrame
from whiteout.dashboard.app import DashboardRuntime, build_parser, create_app, main
from whiteout.dashboard.recording import SessionRecorder, SessionReplay
from whiteout.dashboard.state import (
    AssetActivity,
    MissionStateStore,
    TargetPresentation,
    project_3413,
)
from whiteout.models import (
    ActionRecommendation,
    BoundingBox,
    Detection,
    Pixel,
    SearchMode,
    Telemetry,
    Track,
)


class DashboardStateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.now = datetime(2026, 9, 19, 17, 0, tzinfo=timezone.utc)

    def test_target_presentation_distinguishes_lock_prediction_and_loss(self) -> None:
        store = MissionStateStore()
        track = Track("target-1", 71.99, -94.82, 2.0, 1.0, 8.0, self.now)
        store.update_track(track, observed=True, sensor="tower-1")
        store.update_recommendation(
            ActionRecommendation(SearchMode.TRACK, "maintain_observation", "target confirmed")
        )

        locked = store.snapshot(now=self.now + timedelta(seconds=1))
        predicted = store.snapshot(now=self.now + timedelta(seconds=3))
        lost = store.snapshot(now=self.now + timedelta(seconds=11))

        self.assertEqual(locked["target"]["status"], TargetPresentation.LOCKED.value)
        self.assertEqual(predicted["target"]["status"], TargetPresentation.PREDICTED.value)
        self.assertAlmostEqual(predicted["target"]["track"]["predicted_seconds"], 3.0)
        self.assertEqual(lost["target"]["status"], TargetPresentation.LOST.value)

    def test_frame_telemetry_detection_and_activity_are_visible(self) -> None:
        store = MissionStateStore()
        store.update_frame(CameraFrame("quadcopter", b"jpeg", self.now))
        store.update_telemetry(Telemetry("quadcopter", 71.99, -94.82, 100.0, timestamp=self.now))
        store.update_detection(
            Detection(
                "quadcopter",
                Pixel(100, 80),
                0.91,
                self.now,
                bbox=BoundingBox(80, 60, 120, 100),
            )
        )
        store.update_asset_activity(
            "quadcopter",
            AssetActivity.SEARCHING,
            reason="corridor sweep",
            assignment="sector alpha",
            search_path=[(-1_502_000.0, -1_268_000.0), (-1_501_000.0, -1_267_000.0)],
        )

        asset = store.snapshot(now=self.now + timedelta(milliseconds=200))["assets"]["quadcopter"]
        self.assertEqual(asset["connection"], "ONLINE")
        self.assertEqual(asset["activity"], "SEARCHING")
        self.assertEqual(asset["assignment"], "sector alpha")
        self.assertEqual(asset["detections"][0]["bbox"]["x_min"], 80)
        self.assertIsNotNone(asset["projected"])
        self.assertEqual(store.snapshot(now=self.now)["target"]["source_camera"], "quadcopter")

    def test_arctic_projection_matches_reported_site_bounds(self) -> None:
        x_m, y_m = project_3413(71.99196, -94.822428)
        self.assertGreater(x_m, -1_505_608.1)
        self.assertLess(x_m, -1_499_139.4)
        self.assertGreater(y_m, -1_271_830.9)
        self.assertLess(y_m, -1_265_362.1)

    def test_runtime_exposes_camera_horizontal_fov(self) -> None:
        from whiteout.config import config_from_mapping

        config = config_from_mapping(
            {"cameras": [{"name": "quadcopter", "port": 8600, "hfov_deg": 90}]}
        )
        runtime = DashboardRuntime(
            MissionStateStore(camera_names=("quadcopter",)), mode="live", config=config
        )

        snapshot = runtime.snapshot()
        self.assertEqual(snapshot["assets"]["quadcopter"]["camera_hfov_deg"], 90.0)
        self.assertEqual(
            snapshot["assets"]["quadcopter"]["camera_endpoint"],
            "127.0.0.1:8600/stream",
        )
        self.assertEqual(snapshot["runtime"]["sim_host"], "127.0.0.1")

    def test_map_metadata_does_not_block_live_readers(self) -> None:
        from whiteout.config import config_from_mapping

        runtime = DashboardRuntime(
            MissionStateStore(camera_names=()),
            mode="live",
            config=config_from_mapping({"cameras": [], "mavlink": []}),
        )
        fetch_started = threading.Event()
        release_fetch = threading.Event()

        def blocked_fetch() -> None:
            fetch_started.set()
            release_fetch.wait(timeout=1.0)

        runtime._fetch_site = blocked_fetch  # type: ignore[method-assign]
        runtime.start_live()
        try:
            self.assertTrue(fetch_started.wait(timeout=0.2))
        finally:
            release_fetch.set()
            for thread in runtime._threads:
                thread.join(timeout=1.0)


class DashboardFrontendTests(unittest.TestCase):
    def test_map_import_cannot_block_camera_bootstrap(self) -> None:
        index = (
            Path(__file__).parents[1]
            / "src"
            / "whiteout"
            / "dashboard"
            / "static"
            / "index.html"
        ).read_text(encoding="utf-8")

        self.assertNotIn("import * as maplibregl", index)
        self.assertIn("await import('/static/maplibre-gl.mjs')", index)
        self.assertLess(
            index.rindex("cameraCards();connect();"),
            index.rindex("initializeMap();"),
        )

    def test_camera_marker_anchor_and_selection_use_the_cone_geometry(self) -> None:
        index = (
            Path(__file__).parents[1]
            / "src"
            / "whiteout"
            / "dashboard"
            / "static"
            / "index.html"
        ).read_text(encoding="utf-8")

        self.assertIn("element.dataset.label=asset.name", index)
        self.assertIn("CAMERA_COLORS[asset.name]", index)
        self.assertNotIn("element.textContent=asset.name", index)
        self.assertIn("if(fly&&mapReady)fitCameraCone(name)", index)
        self.assertIn("const coordinates=cameraConeCoordinates", index)
        self.assertNotIn('data-map-mode="topo"', index)
        fit_map = index[index.index("function fitMap()") : index.index("function updateMap(state)")]
        self.assertIn("setFollow(false)", fit_map)


class DashboardRecordingTests(unittest.TestCase):
    def test_recording_redacts_secrets_and_replays_state(self) -> None:
        now = datetime(2026, 9, 19, 17, 0, tzinfo=timezone.utc)
        with tempfile.TemporaryDirectory() as directory:
            recorder = SessionRecorder(
                directory,
                metadata={"sim_host": "10.99.0.1", "MAPBOX_TOKEN": "do-not-record"},
                frame_sample_hz=2.0,
            )
            source = MissionStateStore(recorder=recorder)
            source.update_frame(CameraFrame("tower-1", b"\xff\xd8fake\xff\xd9", now))
            source.update_telemetry(Telemetry("tower-1", 71.99, -94.82, 30.0, timestamp=now))
            source.update_telemetry(
                Telemetry(
                    "tower-2",
                    71.98,
                    -94.81,
                    35.0,
                    roll_rad=0.1,
                    pitch_rad=0.2,
                    yaw_rad=0.3,
                    timestamp=now,
                    attitude_timestamp=now,
                )
            )
            source.update_detection(
                Detection(
                    "tower-1",
                    Pixel(320, 180),
                    0.95,
                    now,
                    bbox=BoundingBox(300, 160, 340, 200),
                )
            )
            source.update_asset_activity(
                "tower-1", AssetActivity.TRACKING, reason="confirmed visual"
            )
            source.update_track(
                Track("target-1", 71.99, -94.82, 0.0, 2.0, 4.0, now),
                observed=True,
                sensor="tower-1",
            )

            manifest = json.loads((Path(directory) / "session.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["metadata"]["MAPBOX_TOKEN"], "[redacted]")

            replay = SessionReplay(directory, MissionStateStore())
            replay.seek(replay.duration_s)
            snapshot = replay.store.snapshot(now=replay.current_time)
            self.assertEqual(snapshot["assets"]["tower-1"]["activity"], "TRACKING")
            self.assertIsNone(snapshot["assets"]["tower-1"]["telemetry"]["roll_rad"])
            self.assertEqual(
                snapshot["assets"]["tower-2"]["telemetry"]["attitude_timestamp"],
                now.isoformat(),
            )
            self.assertEqual(snapshot["target"]["observed_by"], "tower-1")
            self.assertIsNotNone(replay.store.latest_frame("tower-1"))


@unittest.skipUnless(
    importlib.util.find_spec("pyproj"),
    "dashboard map projection dependency is not installed",
)
class DashboardMapTests(unittest.TestCase):
    def test_map_config_uses_existing_site_api_and_public_sources(self) -> None:
        import io

        from whiteout.dashboard.map import MapAssets
        from whiteout.dashboard.state import project_3413

        latitude, longitude = 71.99196, -94.822428
        centre_x, centre_y = project_3413(latitude, longitude)
        metadata = {
            "ok": True,
            "name": "test-site",
            "centre": {"lat": latitude, "lon": longitude},
            "bounds3413": {
                "xmin": centre_x - 1000,
                "ymin": centre_y - 1000,
                "xmax": centre_x + 1000,
                "ymax": centre_y + 1000,
            },
            "extent_m": 2000.0,
        }

        class Response(io.BytesIO):
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                self.close()

        def opener(url: str, **_kwargs):
            self.assertEqual(url, "http://sim.invalid:8090/api/site")
            return Response(json.dumps(metadata).encode())

        assets = MapAssets("sim.invalid", opener=opener)
        config = assets.config()

        self.assertEqual(config["name"], "test-site")
        self.assertEqual(config["center"], [longitude, latitude])
        self.assertLess(config["bounds"][0], longitude)
        self.assertGreater(config["bounds"][2], longitude)
        self.assertEqual(len(config["boundary"]), 5)
        self.assertEqual(config["boundary"][0], config["boundary"][-1])
        self.assertEqual(config["dimensions_m"]["width"], 2000.0)
        self.assertEqual(config["dimensions_m"]["height"], 2000.0)
        self.assertEqual(config["dimensions_m"]["area_km2"], 4.0)
        self.assertIn("World_Imagery", config["imagery_tiles"][0])
        self.assertNotIn("topo_tiles", config)
        self.assertIn("mapterhorn.com", config["terrain_tiles"][0])
        self.assertEqual(config["terrain_encoding"], "terrarium")


@unittest.skipUnless(importlib.util.find_spec("fastapi"), "dashboard extra is not installed")
class DashboardApiSafetyTests(unittest.TestCase):
    def test_dashboard_exposes_only_read_routes_and_state_websocket(self) -> None:
        app = create_app(DashboardRuntime(MissionStateStore(), mode="live"))
        route_methods = {
            (route.path, method)
            for route in app.routes
            for method in getattr(route, "methods", set())
        }
        unsafe = {item for item in route_methods if item[1] in {"POST", "PUT", "PATCH", "DELETE"}}
        self.assertEqual(unsafe, set())
        paths = {route.path for route in app.routes}
        self.assertIn("/ws", paths)
        self.assertFalse(any(word in path for path in paths for word in ("reset", "rebuild", "command", "submit")))

    def test_state_websocket_parameter_is_classified_as_connection(self) -> None:
        app = create_app(DashboardRuntime(MissionStateStore(), mode="live"))
        route = next(route for route in app.routes if route.path == "/ws")

        self.assertEqual(route.dependant.websocket_param_name, "websocket")
        self.assertEqual(route.dependant.query_params, [])

    def test_live_cli_requires_explicit_network_confirmation(self) -> None:
        with self.assertRaisesRegex(SystemExit, "confirm-network"):
            main(["--mode", "live"])


class DashboardCliTests(unittest.TestCase):
    def test_defaults_bind_to_localhost_and_use_safe_port(self) -> None:
        args = build_parser().parse_args([])
        self.assertEqual(args.bind, "127.0.0.1")
        self.assertEqual(args.port, 8070)
        self.assertFalse(args.confirm_network)


if __name__ == "__main__":
    unittest.main()
