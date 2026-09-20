"""FastAPI application and CLI for the read-only mission dashboard."""

import argparse
import asyncio
import json
import logging
import threading
import time
import urllib.request
from contextlib import asynccontextmanager
from importlib.resources import files
from pathlib import Path
from typing import Any, AsyncIterator

from ..camera import CameraFrame, MjpegCamera
from ..config import AppConfig, load_config
from ..mavlink import MavlinkUnavailable, ReadOnlyMavlink
from .map import MapAssets, MapAssetsUnavailable
from .recording import SessionRecorder, SessionReplay
from .state import MissionStateStore, SiteMetadata


logger = logging.getLogger("uvicorn.error")


class DashboardDependencyError(RuntimeError):
    pass


class DashboardRuntime:
    """Own live readers or replay while exposing one current state store."""

    def __init__(
        self,
        store: MissionStateStore,
        *,
        mode: str,
        config: AppConfig | None = None,
        replay: SessionReplay | None = None,
    ) -> None:
        self.store = store
        self.mode = mode
        self.config = config
        self.replay = replay
        self.map_assets = MapAssets(config.sim_host) if config is not None else None
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._replay_task: asyncio.Task[None] | None = None

    @property
    def current_store(self) -> MissionStateStore:
        return self.replay.store if self.replay is not None else self.store

    def snapshot(self) -> dict[str, Any]:
        now = self.replay.current_time if self.replay is not None else None
        state = self.current_store.snapshot(now=now)
        if self.replay is not None:
            state["replay"] = {
                "playing": self.replay.playing,
                "speed": self.replay.speed,
                "position_s": self.replay.position_s,
                "duration_s": self.replay.duration_s,
            }
        if self.config is not None:
            cameras = {camera.name: camera for camera in self.config.cameras}
            for name, asset in state["assets"].items():
                camera = cameras.get(name)
                asset["camera_hfov_deg"] = camera.hfov_deg if camera is not None else None
                if camera is not None:
                    asset["camera_endpoint"] = (
                        f"{self.config.sim_host}:{camera.port}{camera.path}"
                    )
            state["runtime"] = {"mode": self.mode, "sim_host": self.config.sim_host}
        else:
            state["runtime"] = {"mode": self.mode, "sim_host": None}
        return state

    def start_live(self) -> None:
        if self.config is None:
            raise RuntimeError("live runtime requires configuration")
        logger.info("Dashboard simulator host: %s", self.config.sim_host)
        for camera_config in self.config.cameras:
            thread = threading.Thread(
                target=self._camera_worker,
                args=(
                    camera_config.name,
                    camera_config.port,
                    camera_config.path,
                    camera_config.crop_right_px,
                ),
                name=f"camera-{camera_config.name}",
                daemon=True,
            )
            thread.start()
            self._threads.append(thread)
        # Site metadata only drives the map. Fetch it independently so an
        # unavailable control endpoint cannot delay camera or telemetry input.
        thread = threading.Thread(
            target=self._fetch_site,
            name="site-metadata",
            daemon=True,
        )
        thread.start()
        self._threads.append(thread)
        for mavlink_config in self.config.mavlink:
            thread = threading.Thread(
                target=self._telemetry_worker,
                args=(mavlink_config.name, mavlink_config.port),
                name=f"telemetry-{mavlink_config.name}",
                daemon=True,
            )
            thread.start()
            self._threads.append(thread)

    def _fetch_site(self) -> None:
        assert self.config is not None
        url = f"http://{self.config.sim_host}:8090/api/site"
        try:
            with urllib.request.urlopen(url, timeout=3.0) as response:
                payload = json.loads(response.read())
            centre = payload["centre"]
            bounds = payload.get("bounds3413")
            self.store.set_site(
                SiteMetadata(
                    str(payload.get("name", "unknown")),
                    float(centre["lat"]),
                    float(centre["lon"]),
                    float(payload["extent_m"]),
                    (
                        float(bounds["xmin"]),
                        float(bounds["ymin"]),
                        float(bounds["xmax"]),
                        float(bounds["ymax"]),
                    )
                    if bounds
                    else None,
                )
            )
        except (OSError, TimeoutError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            # Camera operation remains useful; the UI clearly reports missing site bounds.
            logger.warning("Simulator site metadata unavailable at %s", url)
            return

    def _camera_worker(
        self, name: str, port: int, path: str, crop_right_px: int = 0
    ) -> None:
        assert self.config is not None
        camera = MjpegCamera(
            name,
            f"http://{self.config.sim_host}:{port}{path}",
            crop_right_px=crop_right_px,
        )
        for frame in camera.frames():
            if self._stop.is_set():
                return
            self.store.update_frame(frame)

    def _telemetry_worker(self, name: str, port: int) -> None:
        assert self.config is not None
        telemetry = ReadOnlyMavlink(name, self.config.sim_host, port)
        try:
            # The simulator endpoints are UDP listeners, so one benign GCS
            # heartbeat is required to register this read-only peer.
            telemetry.connect(initiate_telemetry=True)
        except MavlinkUnavailable:
            return
        while not self._stop.is_set():
            update = telemetry.poll()
            if update is not None:
                self.store.update_telemetry(update)
            time.sleep(0.05)

    async def start_replay(self) -> None:
        if self.replay is None:
            return
        while not self._stop.is_set():
            self.replay.advance()
            await asyncio.sleep(0.02)

    async def start(self) -> None:
        if self.mode == "live":
            self.start_live()
        elif self.replay is not None:
            self._replay_task = asyncio.create_task(self.start_replay())

    async def close(self) -> None:
        self._stop.set()
        if self._replay_task is not None:
            self._replay_task.cancel()
            try:
                await self._replay_task
            except asyncio.CancelledError:
                pass


def _dependency_error(exc: ImportError) -> DashboardDependencyError:
    return DashboardDependencyError(
        "dashboard dependencies are missing; install with `pip install -e '.[dashboard,yaml]'`"
    )


def _decode_frame(frame: CameraFrame):
    return frame.decode_bgr()


def _annotated_jpeg(store: MissionStateStore, camera: str) -> bytes | None:
    try:
        import cv2  # type: ignore[import-not-found]
        import numpy as np  # type: ignore[import-not-found]
    except ImportError as exc:
        raise _dependency_error(exc) from exc

    def placeholder(message: str):
        canvas = np.zeros((360, 640, 3), dtype=np.uint8)
        canvas[:] = (8, 18, 22)
        text_width = cv2.getTextSize(message, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)[0][0]
        cv2.putText(
            canvas,
            message,
            (320 - text_width // 2, 180),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (112, 139, 148),
            2,
            cv2.LINE_AA,
        )
        return canvas

    if camera == "target":
        selected = store.best_target_frame()
        if selected is None:
            image = placeholder("NO ACTIVE TARGET")
        else:
            frame, detection = selected
            image = _decode_frame(frame)
            if image is None or detection.bbox is None:
                image = placeholder("TARGET FRAME UNAVAILABLE")
            else:
                height, width = image.shape[:2]
                bbox = detection.bbox
                padding_x = (bbox.x_max - bbox.x_min) * 0.65
                padding_y = (bbox.y_max - bbox.y_min) * 0.65
                left = max(0, int(bbox.x_min - padding_x))
                top = max(0, int(bbox.y_min - padding_y))
                right = min(width, int(bbox.x_max + padding_x))
                bottom = min(height, int(bbox.y_max + padding_y))
                if right <= left or bottom <= top:
                    image = placeholder("TARGET CROP INVALID")
                else:
                    image = cv2.resize(image[top:bottom, left:right], (640, 360))
                    cv2.drawMarker(image, (320, 180), (72, 255, 189), cv2.MARKER_CROSS, 34, 2)
                    cv2.putText(
                        image,
                        f"{frame.camera.upper()}  CONF {detection.confidence:.0%}",
                        (16, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (240, 245, 247),
                        2,
                        cv2.LINE_AA,
                    )
    else:
        frame = store.latest_frame(camera)
        if frame is None:
            image = placeholder("CAMERA FEED OFFLINE")
        else:
            image = _decode_frame(frame)
        if image is None:
            image = placeholder("CAMERA FRAME INVALID")
        detections = store.latest_detections(camera)
        for detection in detections:
            bbox = detection.bbox
            if bbox is None:
                center = (int(detection.pixel.x), int(detection.pixel.y))
                cv2.drawMarker(image, center, (72, 255, 189), cv2.MARKER_CROSS, 22, 2)
                continue
            left, top = int(bbox.x_min), int(bbox.y_min)
            right, bottom = int(bbox.x_max), int(bbox.y_max)
            # Clearly labelled model-analysis heat, not a synthetic sensor feed.
            raw_heatmap = detection.metadata.get("confidence_heatmap")
            if raw_heatmap is not None:
                heatmap = np.asarray(raw_heatmap, dtype=np.float32)
                if heatmap.ndim == 2 and heatmap.size:
                    heatmap = cv2.resize(heatmap, (image.shape[1], image.shape[0]))
                    heatmap = np.uint8(np.clip(heatmap, 0.0, 1.0) * 255)
                    coloured = cv2.applyColorMap(heatmap, cv2.COLORMAP_TURBO)
                    image = cv2.addWeighted(coloured, 0.24, image, 0.86, 0)
            else:
                overlay = image.copy()
                center = ((left + right) // 2, (top + bottom) // 2)
                axes = (max(14, right - left), max(14, bottom - top))
                cv2.ellipse(overlay, center, axes, 0, 0, 360, (30, 80, 255), -1)
                image = cv2.addWeighted(overlay, 0.16 * detection.confidence, image, 1.0, 0)
            cv2.rectangle(image, (left, top), (right, bottom), (72, 255, 189), 2)
            cv2.putText(
                image,
                f"{detection.label} {detection.confidence:.0%}  ANALYSIS HEAT",
                (max(4, left), max(18, top - 7)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (72, 255, 189),
                2,
                cv2.LINE_AA,
            )
        cv2.putText(
            image,
            camera.upper(),
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (240, 245, 247),
            2,
            cv2.LINE_AA,
        )
    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
    return encoded.tobytes() if ok else None


def create_app(runtime: DashboardRuntime):
    try:
        from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
        from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
    except ImportError as exc:
        raise _dependency_error(exc) from exc

    @asynccontextmanager
    async def lifespan(_app: Any) -> AsyncIterator[None]:
        await runtime.start()
        try:
            yield
        finally:
            await runtime.close()

    app = FastAPI(title="WHITEOUT Mission Dashboard", docs_url=None, redoc_url=None, lifespan=lifespan)

    @app.get("/")
    async def index():
        path = files("whiteout.dashboard").joinpath("static/index.html")
        return FileResponse(str(path))

    @app.get("/static/{asset}")
    async def static_asset(asset: str):
        allowed = {
            "maplibre-gl.css",
            "maplibre-gl.mjs",
            "maplibre-gl-shared.mjs",
            "maplibre-gl-worker.mjs",
            "MAPLIBRE-LICENSE.txt",
        }
        if asset not in allowed:
            raise HTTPException(status_code=404, detail="unknown static asset")
        path = files("whiteout.dashboard").joinpath(f"static/{asset}")
        return FileResponse(str(path))

    @app.get("/health")
    async def health():
        return {"ok": True, "mode": runtime.mode, "sequence": runtime.snapshot()["sequence"]}

    @app.get("/api/state")
    async def api_state():
        return JSONResponse(runtime.snapshot())

    @app.get("/api/map/config")
    async def map_config():
        if runtime.map_assets is None:
            raise HTTPException(status_code=404, detail="map is unavailable in this mode")
        try:
            config = await asyncio.to_thread(runtime.map_assets.config)
        except MapAssetsUnavailable as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return JSONResponse(config)

    @app.get("/video/{camera}")
    async def video(camera: str):
        if camera != "target" and camera not in runtime.current_store.camera_names:
            raise HTTPException(status_code=404, detail="unknown camera")

        async def stream():
            while True:
                jpeg = await asyncio.to_thread(_annotated_jpeg, runtime.current_store, camera)
                if jpeg is not None:
                    yield b"--whiteoutframe\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
                await asyncio.sleep(0.125)

        return StreamingResponse(
            stream(), media_type="multipart/x-mixed-replace; boundary=whiteoutframe"
        )

    @app.websocket("/ws")
    async def websocket_state(websocket: WebSocket):
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(runtime.snapshot())
                try:
                    raw = await asyncio.wait_for(websocket.receive_text(), timeout=0.2)
                except asyncio.TimeoutError:
                    continue
                if runtime.mode != "replay" or runtime.replay is None:
                    await websocket.send_json({"error": "live dashboard is observation-only"})
                    continue
                try:
                    message = json.loads(raw)
                    action = message.get("action")
                    if action == "playing":
                        runtime.replay.set_playing(bool(message["value"]))
                    elif action == "speed":
                        runtime.replay.set_speed(float(message["value"]))
                    elif action == "seek":
                        runtime.replay.seek(float(message["value"]))
                    else:
                        await websocket.send_json({"error": "unsupported replay control"})
                except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                    await websocket.send_json({"error": str(exc)})
        except WebSocketDisconnect:
            return

    return app


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the read-only WHITEOUT mission dashboard")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--mode", choices=("live", "replay"), default="live")
    parser.add_argument("--session", help="session directory for replay or live recording")
    parser.add_argument("--speed", type=float, default=1.0, help="initial replay speed")
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8070)
    parser.add_argument("--frame-sample-hz", type=float, default=2.0)
    parser.add_argument("--confirm-network", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.mode == "live" and not args.confirm_network:
        raise SystemExit("refusing live connections without --confirm-network")
    if args.mode == "replay" and not args.session:
        raise SystemExit("replay mode requires --session")
    if not 1 <= args.port <= 65535:
        raise SystemExit("dashboard port must be between 1 and 65535")

    recorder = None
    config = None
    if args.mode == "live":
        config = load_config(args.config)
        if args.session:
            recorder = SessionRecorder(
                args.session,
                metadata={"mode": "live", "sim_host": config.sim_host},
                frame_sample_hz=args.frame_sample_hz,
            )
    store = MissionStateStore(recorder=recorder)
    replay = SessionReplay(args.session, store, speed=args.speed) if args.mode == "replay" else None
    runtime = DashboardRuntime(store, mode=args.mode, config=config, replay=replay)
    app = create_app(runtime)
    try:
        import uvicorn  # type: ignore[import-not-found]
    except ImportError as exc:
        raise _dependency_error(exc) from exc
    uvicorn.run(app, host=args.bind, port=args.port, log_level="info")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
