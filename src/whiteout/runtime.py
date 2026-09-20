"""Managed live runtime for the unified WHITEOUT coordinator command."""

from __future__ import annotations

import math
import queue
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Protocol

from .camera import CameraFrame, CameraModel, MjpegCamera
from .config import AppConfig, CameraConfig, TowerMotionConfig
from .control import CopterController, CopterMode, PlaneController, TowerController
from .coordinator import CoordinationResult, Coordinator
from .dashboard.app import DashboardRuntime, create_app
from .dashboard.recording import SessionRecorder
from .dashboard.state import AssetActivity, MissionStateStore
from .detector import Detector, YoloVesselDetector
from .execution import LiveExecutor, LiveTrackSink, OperatorSession
from .geolocation import CameraIntrinsics, CameraPose
from .live import SimulatorActuator, TrackApiSubmitter
from .mavlink import ReadOnlyMavlink
from .models import ControlIntent, Detection, SearchMode, Telemetry, TrackEstimate
from .objects import Copter, Plane, Tower, tower_pwm_to_angle
from .pipeline import OperationalPipeline
from .pose import (
    FIXED_WING_CAMERA_DOWN_DEG,
    QUADCOPTER_CAMERA_DOWN_DEG,
    aircraft_camera_pose,
    tower_camera_pose,
)
from .simulator_metadata import ArcticSimMetadataClient
from .tower import (
    TowerCalibration,
    TowerOrientation,
    TowerWorldPose,
    tower_scan_overlap_steps,
    wrap_pan,
)
from .track_api import TrackApiClient


ASSETS = ("quadcopter", "fixed-wing", "tower-1", "tower-2")
AIRCRAFT = ("quadcopter", "fixed-wing")
LOCAL_TELEMETRY_PORTS = {
    "quadcopter": 15550,
    "fixed-wing": 15560,
    "tower-1": 15580,
    "tower-2": 15590,
}


class RuntimeErrorHandler(Protocol):
    def __call__(self, message: str) -> None: ...


@dataclass(slots=True)
class TelemetryBuffer:
    maxlen: int = 120
    _items: deque[Telemetry] = field(default_factory=deque)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def append(self, item: Telemetry) -> None:
        with self._lock:
            self._items.append(item)
            while len(self._items) > self.maxlen:
                self._items.popleft()

    def latest(self) -> Telemetry | None:
        with self._lock:
            return self._items[-1] if self._items else None

    def nearest(self, timestamp: datetime, maximum_skew_s: float) -> Telemetry | None:
        with self._lock:
            if not self._items:
                return None
            item = min(self._items, key=lambda value: abs((value.timestamp - timestamp).total_seconds()))
        return item if abs((item.timestamp - timestamp).total_seconds()) <= maximum_skew_s else None


@dataclass(frozen=True, slots=True)
class FrameState:
    """Immutable observation state captured before asynchronous inference."""

    frame: CameraFrame
    telemetry: Telemetry | None
    intrinsics: CameraIntrinsics
    pose: CameraPose | None
    tower_pan_pwm: int | None = None
    tower_tilt_pwm: int | None = None


@dataclass(slots=True)
class ManagedTower:
    controller: TowerController
    motion: TowerMotionConfig
    calibration: TowerCalibration = field(default_factory=TowerCalibration)
    commanded_pan_deg: float = 0.0
    commanded_tilt_deg: float = 7.5
    target_pan_deg: float = 0.0
    target_tilt_deg: float = 7.5
    commanded_at: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    _last_advance_s: float | None = None
    _last_command_s: float | None = None
    _reached_at_s: float | None = None
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def pan(self, angle_deg: float) -> None:
        with self._lock:
            target = min(self.motion.pan_max_deg, max(self.motion.pan_min_deg, float(angle_deg)))
            if not math.isclose(target, self.target_pan_deg, abs_tol=1e-9):
                self.target_pan_deg = target
                self._reached_at_s = None

    def tilt(self, angle_deg: float) -> None:
        with self._lock:
            target = min(self.motion.tilt_max_deg, max(self.motion.tilt_min_deg, float(angle_deg)))
            if not math.isclose(target, self.target_tilt_deg, abs_tol=1e-9):
                self.target_tilt_deg = target
                self._reached_at_s = None

    def initialize_center(self, now_s: float | None = None) -> None:
        now = time.monotonic() if now_s is None else now_s
        with self._lock:
            self.controller.pan(0.0)
            self.controller.tilt(7.5)
            self.commanded_pan_deg = self.target_pan_deg = 0.0
            self.commanded_tilt_deg = self.target_tilt_deg = 7.5
            self.commanded_at = datetime.now(timezone.utc)
            self._last_advance_s = now
            self._last_command_s = now
            self._reached_at_s = now

    def advance(self, now_s: float | None = None) -> bool:
        now = time.monotonic() if now_s is None else now_s
        with self._lock:
            if self._last_advance_s is None:
                self._last_advance_s = now
                return False
            command_interval = 1.0 / self.motion.command_hz
            if self._last_command_s is not None and now - self._last_command_s < command_interval:
                return False
            elapsed = max(0.0, now - self._last_advance_s)
            self._last_advance_s = now
            next_pan = _move_toward(
                self.commanded_pan_deg,
                self.target_pan_deg,
                self.motion.pan_rate_deg_s * elapsed,
            )
            next_tilt = _move_toward(
                self.commanded_tilt_deg,
                self.target_tilt_deg,
                self.motion.tilt_rate_deg_s * elapsed,
            )
            changed = False
            if not math.isclose(next_pan, self.commanded_pan_deg, abs_tol=1e-9):
                self.controller.pan(next_pan)
                self.commanded_pan_deg = next_pan
                changed = True
            if not math.isclose(next_tilt, self.commanded_tilt_deg, abs_tol=1e-9):
                self.controller.tilt(next_tilt)
                self.commanded_tilt_deg = next_tilt
                changed = True
            if changed:
                self.commanded_at = datetime.now(timezone.utc)
                self._last_command_s = now
            if self._at_target_unlocked():
                self._reached_at_s = self._reached_at_s or now
            else:
                self._reached_at_s = None
            return changed

    def at_target(self) -> bool:
        with self._lock:
            return self._at_target_unlocked()

    def target_reached_at(self) -> float | None:
        with self._lock:
            return self._reached_at_s

    def hold(self, now_s: float | None = None) -> None:
        """Cancel an in-progress slew without issuing another servo command."""
        now = time.monotonic() if now_s is None else now_s
        with self._lock:
            self.target_pan_deg = self.commanded_pan_deg
            self.target_tilt_deg = self.commanded_tilt_deg
            self._last_advance_s = now
            self._reached_at_s = now

    def _at_target_unlocked(self) -> bool:
        return (
            math.isclose(self.commanded_pan_deg, self.target_pan_deg, abs_tol=1e-6)
            and math.isclose(self.commanded_tilt_deg, self.target_tilt_deg, abs_tol=1e-6)
        )

    def orientation(self, telemetry: Telemetry) -> TowerOrientation | None:
        if telemetry.servo_1_pwm is None or telemetry.servo_2_pwm is None:
            return None
        try:
            pan = tower_pwm_to_angle(1, telemetry.servo_1_pwm)
            tilt = tower_pwm_to_angle(2, telemetry.servo_2_pwm)
        except ValueError:
            return None
        rates = tuple(
            value for value in (
                telemetry.roll_rate_dps,
                telemetry.pitch_rate_dps,
                telemetry.yaw_rate_dps,
            ) if value is not None
        )
        angular_rate = max((abs(value) for value in rates), default=math.inf)
        compass = math.degrees(telemetry.yaw_rad) % 360.0 if telemetry.yaw_rad is not None else None
        with self._lock:
            commanded_pan = self.commanded_pan_deg
            commanded_tilt = self.commanded_tilt_deg
            commanded_at = self.commanded_at
        return TowerOrientation(
            pan,
            tilt,
            telemetry.timestamp,
            commanded_pan_deg=commanded_pan,
            commanded_tilt_deg=commanded_tilt,
            compass_heading_deg=compass,
            calibration=self.calibration,
            angular_rate_dps=angular_rate,
            settled_since=commanded_at,
        )


def _move_toward(current: float, target: float, maximum_delta: float) -> float:
    delta = target - current
    if abs(delta) <= maximum_delta:
        return target
    return current + math.copysign(maximum_delta, delta)


class RateLimitedExecutor:
    """Coalesce live intents and stop commands when telemetry or operator state is unsafe."""

    def __init__(
        self,
        delegate: LiveExecutor,
        telemetry_for: Callable[[str], Telemetry | None],
        *,
        interval_s: float,
        stale_s: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.delegate = delegate
        self.telemetry_for = telemetry_for
        self.interval_s = interval_s
        self.stale_s = stale_s
        self.clock = clock
        self.enabled_assets: set[str] = set()
        self.operator_overrides: set[str] = set()
        self._last_sent: dict[str, float] = {}

    def execute(self, intent: ControlIntent) -> None:
        asset = intent.asset.strip().lower().replace("_", "-")
        if asset not in self.enabled_assets or asset in self.operator_overrides:
            return
        telemetry = self.telemetry_for(asset)
        if telemetry is None:
            return
        age = (datetime.now(timezone.utc) - telemetry.timestamp).total_seconds()
        if age < -0.1 or age > self.stale_s:
            return
        now = self.clock()
        if intent.action != "resume_search" and now - self._last_sent.get(asset, -math.inf) < self.interval_s:
            return
        self.delegate.execute(intent)
        self._last_sent[asset] = now


class LatestTrackSink:
    """Single-slot asynchronous submission worker; ambiguous failures are never retried."""

    def __init__(
        self,
        delegate: LiveTrackSink,
        store: MissionStateStore,
        error: RuntimeErrorHandler,
        enabled: bool = True,
    ) -> None:
        self.delegate = delegate
        self.store = store
        self.error = error
        self.enabled = enabled
        self._pending: queue.Queue[TrackEstimate | None] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="track-submission", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def submit(self, estimate: TrackEstimate) -> None:
        if not self.enabled:
            return
        try:
            self._pending.get_nowait()
        except queue.Empty:
            pass
        self._pending.put_nowait(estimate)
        self.store.update_submission("QUEUED", f"track {estimate.track.track_id}")

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                estimate = self._pending.get(timeout=0.2)
            except queue.Empty:
                continue
            if estimate is None:
                return
            try:
                self.delegate.submit(estimate)
            except Exception as exc:  # transport failures are surfaced and not retried
                self.store.update_submission("FAILED", str(exc))
                self.error(f"track submission failed: {exc}")
            else:
                self.store.update_submission("SENT", f"track {estimate.track.track_id}")

    def close(self) -> None:
        self._stop.set()
        try:
            self._pending.get_nowait()
        except queue.Empty:
            pass
        try:
            self._pending.put_nowait(None)
        except queue.Full:
            pass
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)


class UnifiedCoordinatorRuntime:
    """Own all live inputs, control connections, tracking, dashboard state, and shutdown."""

    def __init__(
        self,
        config: AppConfig,
        *,
        detector: Detector,
        controllers: dict[str, object],
        telemetry_readers: dict[str, ReadOnlyMavlink],
        tower_poses: tuple[TowerWorldPose, ...],
        submit_tracks: bool,
        operator: str,
        session_directory: str | None = None,
        output: Callable[[str], None] = print,
    ) -> None:
        self.config = config
        self.detector = detector
        self.controllers = controllers
        self.telemetry_readers = telemetry_readers
        self.tower_poses = tower_poses
        self.submit_tracks = submit_tracks
        self.output = output
        self.operator_session = OperatorSession(operator, active=False)
        recorder = SessionRecorder(
            session_directory,
            metadata={"mode": "coordinator", "sim_host": config.sim_host},
        ) if session_directory else None
        self.store = MissionStateStore(recorder=recorder)
        self.telemetry = {name: TelemetryBuffer() for name in ASSETS}
        self._stop = threading.Event()
        self._tower_motion_enabled = threading.Event()
        self._tower_motion_enabled.set()
        self._threads: list[threading.Thread] = []
        self._detected_queue: queue.Queue[tuple[FrameState, tuple]] = queue.Queue(maxsize=len(ASSETS) * 2)
        self._pending_frames: dict[str, FrameState] = {}
        self._pending_condition = threading.Condition()
        self._camera_seen: set[str] = set()
        self._detector_seen: set[str] = set()
        self._geometry_seen: set[str] = set()
        self._controllers_started: set[str] = set()
        self._aircraft_started: set[str] = set()
        self._aircraft_launched: set[str] = set()
        self._aborted: set[str] = set()
        self._dashboard_server: object | None = None
        self._scan_index = {"tower-1": -1, "tower-2": 0}
        self._scan_at = {"tower-1": 0.0, "tower-2": 0.0}
        self._tower_detection_hold_until = {"tower-1": 0.0, "tower-2": 0.0}
        self._published_mode = SearchMode.SEARCH

        tower_motion = {item.name: item for item in config.tower_motion}
        self.towers = {
            name: ManagedTower(controllers[name], tower_motion[name])  # type: ignore[arg-type]
            for name in ("tower-1", "tower-2")
        }
        actuator = SimulatorActuator(
            controllers["quadcopter"],  # type: ignore[arg-type]
            self.towers,
            self.latest_telemetry,
            config.course_bounds.contains if config.course_bounds is not None else None,
        )
        live_executor = LiveExecutor(
            actuator, live_enabled=True, operator_session=self.operator_session
        )
        self.executor = RateLimitedExecutor(
            live_executor,
            self.latest_telemetry,
            interval_s=config.coordinator.command_interval_s,
            stale_s=config.coordinator.telemetry_stale_s,
        )
        api = TrackApiClient(
            config.track_api.endpoint,
            name=config.track_api.name,
            allow_submission=submit_tracks and config.track_api.allow_submission,
            include_speed=config.track_api.include_speed,
        )
        live_sink = LiveTrackSink(
            TrackApiSubmitter(api),
            live_enabled=submit_tracks,
            operator_session=self.operator_session,
        )
        self.track_sink = LatestTrackSink(
            live_sink, self.store, self.output, enabled=submit_tracks
        )
        coordinator = Coordinator.from_config(config, tower_poses=tower_poses)
        self.pipeline = OperationalPipeline(
            detector,
            coordinator,
            self.executor,
            self.track_sink,
            submission_interval_s=1.0,
            max_telemetry_skew_s=config.coordinator.telemetry_skew_s,
            on_detection=self._publish_detection,
            on_result=self._publish_result,
        )
        self.camera_models = {camera.name: _camera_model(camera) for camera in config.cameras}
        # Intrinsics describe the uncropped sensor. A right-edge-only crop keeps
        # the original pixel origin and principal point, so it must not be
        # recalibrated as though the smaller image were centered.
        self.camera_intrinsics = {
            name: model.intrinsics for name, model in self.camera_models.items()
        }
        self._update_reacquisition_grid()

    @classmethod
    def build_live(
        cls,
        config: AppConfig,
        *,
        submit_tracks: bool,
        operator: str,
        session_directory: str | None = None,
        output: Callable[[str], None] = print,
    ) -> UnifiedCoordinatorRuntime:
        weights = Path(config.coordinator.detector_weights)
        if not weights.is_file():
            raise ValueError(f"detector weights do not exist: {weights}")
        detector = YoloVesselDetector(
            weights,
            fallback_weights=weights,
            imgsz=config.coordinator.detector_imgsz,
            conf=config.coordinator.detector_confidence,
        )
        try:
            generated_poses = ArcticSimMetadataClient(config.sim_host).towers()
        except (KeyError, OSError, RuntimeError, TimeoutError, ValueError):
            generated_poses = ()
        poses = _resolve_tower_poses(generated_poses, config.tower_poses)
        readers: dict[str, ReadOnlyMavlink] = {}
        controllers: dict[str, object] = {}
        for name in ASSETS:
            port = LOCAL_TELEMETRY_PORTS[name]
            reader = ReadOnlyMavlink(name, "127.0.0.1", port, transport="udpin")
            reader.connect()
            readers[name] = reader
            forwarded = (f"--out=udp:127.0.0.1:{port}",)
            if name == "quadcopter":
                controllers[name] = Copter(host=config.sim_host).controller(mavproxy_extra_arguments=forwarded)
            elif name == "fixed-wing":
                controllers[name] = Plane(host=config.sim_host).controller(mavproxy_extra_arguments=forwarded)
            elif name == "tower-1":
                controllers[name] = Tower.one(config.sim_host).controller(mavproxy_extra_arguments=forwarded)
            else:
                controllers[name] = Tower.two(config.sim_host).controller(mavproxy_extra_arguments=forwarded)
        return cls(
            config,
            detector=detector,
            controllers=controllers,
            telemetry_readers=readers,
            tower_poses=poses,
            submit_tracks=submit_tracks,
            operator=operator,
            session_directory=session_directory,
            output=output,
        )

    def connect(self) -> None:
        try:
            for name in ASSETS:
                self.output(f"Connecting {name}...")
                self.controllers[name].start()  # type: ignore[attr-defined]
                self._controllers_started.add(name)
        except Exception:
            self.close(request_rtl=False)
            raise
        self._start_workers()

    def activate_operator_session(self) -> None:
        self.operator_session = OperatorSession(self.operator_session.operator, active=True)
        self.executor.delegate.operator_session = self.operator_session
        self.track_sink.delegate.operator_session = self.operator_session

    def upload_missions(self) -> None:
        paths = {
            "quadcopter": Path(self.config.coordinator.quadcopter_mission),
            "fixed-wing": Path(self.config.coordinator.fixed_wing_mission),
        }
        for name, path in paths.items():
            if not path.is_file():
                raise ValueError(f"mission does not exist: {path}")
            mission = self.controllers[name].upload_search_mission(path)  # type: ignore[attr-defined]
            self.output(f"Uploaded and confirmed {mission.path}")

    def calibrate_towers(self, *, timeout_s: float = 12.0) -> None:
        if not self.operator_session.active:
            raise PermissionError("tower calibration requires an active operator session")
        for tower in self.towers.values():
            tower.initialize_center()
        deadline = time.monotonic() + timeout_s
        samples: dict[str, list[float]] = {name: [] for name in self.towers}
        stable_since: dict[str, float | None] = {name: None for name in self.towers}
        while time.monotonic() < deadline and any(len(value) < 15 for value in samples.values()):
            for name, tower in self.towers.items():
                telemetry = self.latest_telemetry(name)
                if telemetry is None or telemetry.yaw_rad is None or not tower.at_target():
                    continue
                rate = abs(telemetry.yaw_rate_dps) if telemetry.yaw_rate_dps is not None else math.inf
                if rate >= 1.0:
                    stable_since[name] = None
                    samples[name].clear()
                    continue
                stable_since[name] = stable_since[name] or time.monotonic()
                if time.monotonic() - stable_since[name] >= 0.5:
                    samples[name].append(math.degrees(telemetry.yaw_rad) % 360.0)
            time.sleep(0.2)
        for name, values in samples.items():
            if len(values) < 15:
                raise RuntimeError(f"{name} calibration did not receive three seconds of settled heading")
            heading = _circular_mean(values)
            self.towers[name].calibration = self.towers[name].calibration.with_compass_reference(heading)
            self.executor.enabled_assets.add(name)
            self.output(f"Calibrated {name}: compass bias {self.towers[name].calibration.compass_bias_deg:+.2f} deg")

    def launch_quadcopter(self, altitude_m: float) -> None:
        self._require_active_session()
        controller: CopterController = self.controllers["quadcopter"]  # type: ignore[assignment]
        controller.set_mode(CopterMode.GUIDED)
        controller.arm()
        controller.takeoff(altitude_m)
        self._aircraft_launched.add("quadcopter")

    def launch_fixed_wing(self) -> None:
        self._require_active_session()
        controller: PlaneController = self.controllers["fixed-wing"]  # type: ignore[assignment]
        controller.takeoff()
        self._aircraft_launched.add("fixed-wing")

    def start_search(self, asset: str) -> None:
        self._require_active_session()
        if asset in self._aborted:
            raise RuntimeError(f"{asset} was aborted and cannot resume in this session")
        self.controllers[asset].start_search()  # type: ignore[attr-defined]
        self._aircraft_started.add(asset)
        if asset == "quadcopter":
            self.executor.enabled_assets.add(asset)
        self.store.update_asset_activity(asset, AssetActivity.SEARCHING, reason="search mission active")

    def command(self, action: str, asset: str) -> None:
        targets = AIRCRAFT if asset == "all" else (asset,)
        if any(item not in AIRCRAFT for item in targets):
            raise ValueError("asset must be quadcopter, fixed-wing, or all")
        for name in targets:
            controller = self.controllers[name]
            if action == "status":
                controller.status()  # type: ignore[attr-defined]
            elif action == "pause":
                controller.pause_search()  # type: ignore[attr-defined]
                self.executor.operator_overrides.add(name)
            elif action == "resume":
                if name in self._aborted:
                    raise RuntimeError(f"{name} was aborted and cannot resume")
                controller.resume_search()  # type: ignore[attr-defined]
                self.executor.operator_overrides.discard(name)
            elif action == "abort":
                controller.abort_search()  # type: ignore[attr-defined]
                self._aborted.add(name)
                self.executor.operator_overrides.add(name)
            else:
                raise ValueError(f"unknown command {action!r}")

    def _require_active_session(self) -> None:
        if not self.operator_session.active:
            raise PermissionError("flight operation requires an active operator session")

    def run(self) -> None:
        interval = 1.0 / self.config.coordinator.tick_hz
        next_tick = time.monotonic()
        while not self._stop.is_set():
            self._drain_detected()
            now = time.monotonic()
            if now >= next_tick:
                self.pipeline.tick(now)
                self._scan_towers(now)
                next_tick = now + interval
            time.sleep(min(0.01, interval))

    def latest_telemetry(self, asset: str) -> Telemetry | None:
        buffer = self.telemetry.get(asset)
        return buffer.latest() if buffer is not None else None

    def close(self, *, request_rtl: bool = True) -> None:
        self._tower_motion_enabled.clear()
        self.operator_session = OperatorSession(self.operator_session.operator, active=False)
        self.executor.delegate.operator_session = self.operator_session
        self.track_sink.delegate.operator_session = self.operator_session
        self.executor.enabled_assets.clear()
        self.track_sink.enabled = False
        self.track_sink.close()
        rtl_requested: set[str] = set()
        if request_rtl:
            for name in AIRCRAFT:
                telemetry = self.latest_telemetry(name)
                if (
                    name not in self._controllers_started
                    or (name not in self._aircraft_launched and not (telemetry and telemetry.armed))
                ):
                    continue
                try:
                    self.controllers[name].abort_search()  # type: ignore[attr-defined]
                    rtl_requested.add(name)
                    self.output(f"RTL requested for {name}")
                except Exception as exc:
                    self.output(f"RTL request failed for {name}: {exc}")
            deadline = time.monotonic() + 3.0
            pending = set(rtl_requested)
            while pending and time.monotonic() < deadline:
                for name in tuple(pending):
                    telemetry = self.latest_telemetry(name)
                    if telemetry is not None and telemetry.mode == "RTL":
                        self.output(f"RTL confirmed for {name}")
                        pending.remove(name)
                if pending:
                    time.sleep(0.05)
            for name in pending:
                self.output(f"RTL was requested for {name}; mode confirmation was not received")
        self._stop.set()
        if self._dashboard_server is not None:
            self._dashboard_server.should_exit = True  # type: ignore[attr-defined]
        with self._pending_condition:
            self._pending_condition.notify_all()
        for name in reversed(ASSETS):
            if name in self._controllers_started:
                try:
                    self.controllers[name].close()  # type: ignore[attr-defined]
                except Exception as exc:
                    self.output(f"close failed for {name}: {exc}")
        for thread in self._threads:
            if thread is not threading.current_thread() and thread.is_alive():
                thread.join(timeout=1.0)

    def _start_workers(self) -> None:
        self.track_sink.start()
        self._thread(self._tower_motion_worker, "tower-motion")
        self._thread(self._inference_worker, "detector")
        for item in self.config.cameras:
            self._thread(self._camera_worker, f"camera-{item.name}", item)
        for name, reader in self.telemetry_readers.items():
            self._thread(self._telemetry_worker, f"telemetry-{name}", name, reader)
        self._start_dashboard()

    def _thread(self, target: Callable[..., None], name: str, *args: object) -> None:
        thread = threading.Thread(target=target, args=args, name=name, daemon=True)
        thread.start()
        self._threads.append(thread)

    def _camera_worker(self, camera_config: CameraConfig) -> None:
        camera = MjpegCamera(
            camera_config.name,
            f"http://{self.config.sim_host}:{camera_config.port}{camera_config.path}",
            crop_right_px=camera_config.crop_right_px,
        )
        for frame in camera.frames():
            if self._stop.is_set():
                return
            self.store.update_frame(frame)
            if frame.camera not in self._geometry_seen:
                self._geometry_seen.add(frame.camera)
                try:
                    height, visible_width = frame.decode_bgr().shape[:2]
                except (RuntimeError, ValueError) as exc:
                    self.output(f"camera geometry unavailable for {frame.camera}: {exc}")
                else:
                    source_width = visible_width + frame.crop_right_px
                    model = self.camera_models[frame.camera]
                    if (source_width, height) != (model.width_px, model.height_px):
                        adjusted = CameraModel(
                            model.name,
                            source_width,
                            height,
                            model.horizontal_fov_deg,
                            model.vertical_fov_deg,
                        )
                        self.camera_models[frame.camera] = adjusted
                        self.camera_intrinsics[frame.camera] = adjusted.intrinsics
                        self.output(
                            f"Camera geometry adjusted: {frame.camera} "
                            f"{model.width_px}x{model.height_px} -> "
                            f"{source_width}x{height} source, {visible_width}x{height} visible"
                        )
            if frame.camera not in self._camera_seen:
                self._camera_seen.add(frame.camera)
                self.output(f"Camera receiving frames: {frame.camera}")
            with self._pending_condition:
                # One pending frame per camera bounds latency and memory. A new
                # frame replaces an older frame that inference has not begun.
                self._pending_frames.pop(frame.camera, None)
                self._pending_frames[frame.camera] = self._capture_frame_state(frame)
                self._pending_condition.notify()

    def _inference_worker(self) -> None:
        while not self._stop.is_set():
            with self._pending_condition:
                while not self._pending_frames and not self._stop.is_set():
                    self._pending_condition.wait(timeout=0.2)
                if self._stop.is_set():
                    return
                camera = next(iter(self._pending_frames))
                state = self._pending_frames.pop(camera)
            self._detect(state)

    def _detect(self, state: FrameState) -> None:
        if self._stop.is_set():
            return
        frame = state.frame
        try:
            detections = tuple(self.detector.detect(frame))
        except Exception as exc:
            self.output(f"detector failed for {frame.camera}: {exc}")
            return
        if frame.camera not in self._detector_seen:
            self._detector_seen.add(frame.camera)
            self.output(f"Detector running: {frame.camera}")
        if (
            detections
            and frame.camera in self.towers
            and self.pipeline.coordinator.mode is SearchMode.SEARCH
        ):
            self._hold_for_tower_detection(frame.camera)
        self._replace_queue(self._detected_queue, (state, detections))

    def _capture_frame_state(self, frame: CameraFrame) -> FrameState:
        telemetry = self.telemetry[frame.camera].nearest(
            frame.timestamp, self.config.coordinator.telemetry_skew_s
        )
        return FrameState(
            frame=frame,
            telemetry=telemetry,
            intrinsics=self.camera_intrinsics[frame.camera],
            pose=self._pose(frame.camera, telemetry),
            tower_pan_pwm=telemetry.servo_1_pwm if telemetry else None,
            tower_tilt_pwm=telemetry.servo_2_pwm if telemetry else None,
        )

    def _hold_for_tower_detection(self, name: str) -> None:
        now = time.monotonic()
        first_hold = now >= self._tower_detection_hold_until[name]
        self.towers[name].hold(now)
        self._tower_detection_hold_until[name] = (
            now + self.config.coordinator.tower_detection_hold_s
        )
        if first_hold:
            self.output(f"Tower hold: {name} detection awaiting settled geolocation")
        self.store.update_asset_activity(
            name,
            AssetActivity.CONFIRMING,
            reason="detector hold awaiting settled geolocation",
            pan_deg=self.towers[name].target_pan_deg,
            tilt_deg=self.towers[name].target_tilt_deg,
        )

    def _telemetry_worker(self, name: str, reader: ReadOnlyMavlink) -> None:
        while not self._stop.is_set():
            try:
                update = reader.poll()
            except Exception as exc:
                self.output(f"telemetry failed for {name}: {exc}")
                return
            if update is not None:
                self.telemetry[name].append(update)
                self.store.update_telemetry(update)
            time.sleep(0.02)

    def _tower_motion_worker(self) -> None:
        while not self._stop.is_set():
            if not self._tower_motion_enabled.is_set():
                time.sleep(0.02)
                continue
            now = time.monotonic()
            for tower in self.towers.values():
                try:
                    tower.advance(now)
                except Exception as exc:
                    self.output(f"tower movement failed for {tower.motion.name}: {exc}")
            time.sleep(0.02)

    @staticmethod
    def _replace_queue(target: queue.Queue, item: object) -> None:
        try:
            target.put_nowait(item)
        except queue.Full:
            try:
                target.get_nowait()
            except queue.Empty:
                pass
            target.put_nowait(item)

    def _drain_detected(self) -> None:
        while True:
            try:
                state, detections = self._detected_queue.get_nowait()
            except queue.Empty:
                return
            self.pipeline.process_frame(
                state.frame,
                telemetry=state.telemetry,
                intrinsics=state.intrinsics,
                pose=state.pose,
                now_s=time.monotonic(),
                tower_pan_pwm=state.tower_pan_pwm,
                tower_tilt_pwm=state.tower_tilt_pwm,
                detections=detections,
            )

    def _pose(self, camera: str, telemetry: Telemetry | None):
        if telemetry is None:
            return None
        try:
            if camera == "quadcopter":
                return aircraft_camera_pose(telemetry, camera_down_deg=QUADCOPTER_CAMERA_DOWN_DEG)
            if camera == "fixed-wing":
                return aircraft_camera_pose(telemetry, camera_down_deg=FIXED_WING_CAMERA_DOWN_DEG)
            tower = next(pose for pose in self.tower_poses if pose.name == camera)
            orientation = self.towers[camera].orientation(telemetry)
            return tower_camera_pose(tower, orientation) if orientation is not None else None
        except (StopIteration, ValueError):
            return None

    def _publish_result(self, result: CoordinationResult) -> None:
        mode = result.recommendation.mode
        if self._published_mode is SearchMode.SEARCH and mode is not SearchMode.SEARCH:
            now = time.monotonic()
            for tower in self.towers.values():
                tower.hold(now)
        self._published_mode = mode
        self.store.update_recommendation(result.recommendation)
        self._update_reacquisition_grid()
        if result.track_estimate is not None:
            self.store.update_track(
                result.track_estimate.track,
                observed=result.track_estimate.source is not None,
                sensor=result.track_estimate.source,
            )

    def _update_reacquisition_grid(self) -> None:
        grid = self.pipeline.coordinator.reacquisition_grid if hasattr(self, "pipeline") else None
        if grid is None:
            return
        self.store.update_reacquisition_grid({
            "bounds": [grid.west, grid.south, grid.east, grid.north],
            "rows": grid.rows,
            "columns": grid.columns,
            "cells": [
                {
                    "row": cell.row,
                    "column": cell.column,
                    "weight": cell.weight,
                    "evidence_latitude": cell.evidence_latitude,
                    "evidence_longitude": cell.evidence_longitude,
                }
                for cell in grid.cells(time.monotonic())
            ],
        })

    def _publish_detection(self, detection: Detection) -> None:
        self.store.update_detection(detection)
        self.output(
            f"Detection: {detection.camera} {detection.label} "
            f"confidence={detection.confidence:.3f}"
        )

    def _scan_towers(self, now: float) -> None:
        if self.pipeline.coordinator.mode is not SearchMode.SEARCH:
            return
        for name, tower in self.towers.items():
            reached_at = tower.target_reached_at()
            if (
                name not in self.executor.enabled_assets
                or now < self._tower_detection_hold_until[name]
                or reached_at is None
                or now - max(reached_at, self._scan_at[name])
                < self.config.coordinator.tower_dwell_s
            ):
                continue
            pose = next(item for item in self.tower_poses if item.name == name)
            pans = _course_scan_pans(
                pose, self.config.course_bounds,
                self.config.coordinator.tower_horizontal_overlap,
                tower.motion.pan_min_deg,
                tower.motion.pan_max_deg,
            )
            direction = 1 if name == "tower-1" else -1
            self._scan_index[name] = (self._scan_index[name] + direction) % len(pans)
            tower.pan(pans[self._scan_index[name]])
            tower.tilt(7.5)
            self._scan_at[name] = now
            self.store.update_asset_activity(
                name, AssetActivity.PANNING, reason="complementary search scan",
                pan_deg=tower.target_pan_deg, tilt_deg=tower.target_tilt_deg,
            )

    def _start_dashboard(self) -> None:
        try:
            import uvicorn  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("dashboard dependencies are required; install whiteout[dashboard]") from exc
        _require_port_available(
            self.config.coordinator.dashboard_bind,
            self.config.coordinator.dashboard_port,
        )
        dashboard = DashboardRuntime(self.store, mode="coordinator", config=self.config)
        app = create_app(dashboard)
        server = uvicorn.Server(uvicorn.Config(
            app,
            host=self.config.coordinator.dashboard_bind,
            port=self.config.coordinator.dashboard_port,
            log_level="warning",
        ))
        self._dashboard_server = server
        self._thread(server.run, "dashboard")
        self.output(
            f"Dashboard: http://{self.config.coordinator.dashboard_bind}:"
            f"{self.config.coordinator.dashboard_port}"
        )


def _camera_model(config: CameraConfig) -> CameraModel:
    if None in (config.width, config.height, config.hfov_deg, config.vfov_deg):
        raise ValueError(f"camera {config.name!r} requires dimensions and fields of view")
    return CameraModel(
        config.name,
        config.width,  # type: ignore[arg-type]
        config.height,  # type: ignore[arg-type]
        config.hfov_deg,  # type: ignore[arg-type]
        config.vfov_deg,  # type: ignore[arg-type]
    )


def _resolve_tower_poses(
    generated: tuple[TowerWorldPose, ...],
    configured: tuple[TowerWorldPose, ...],
) -> tuple[TowerWorldPose, ...]:
    """Prefer a complete generated pair, otherwise use the explicit config pair."""
    expected = {"tower-1", "tower-2"}
    if {pose.name for pose in generated} == expected:
        return generated
    if {pose.name for pose in configured} == expected:
        return configured
    raise RuntimeError(
        "simulator metadata did not provide both towers and config has no complete "
        "tower_poses fallback for tower-1 and tower-2"
    )


def _circular_mean(values: list[float]) -> float:
    sine = sum(math.sin(math.radians(value)) for value in values)
    cosine = sum(math.cos(math.radians(value)) for value in values)
    return math.degrees(math.atan2(sine, cosine)) % 360.0


def _course_scan_pans(
    tower: TowerWorldPose,
    bounds: object,
    overlap: float,
    pan_min_deg: float = -144.0,
    pan_max_deg: float = 144.0,
) -> list[float]:
    """Return reachable tower pans spanning the configured geographic course."""
    calibration = TowerCalibration()
    if bounds is None:
        values = [-144.0, -96.0, -48.0, 0.0, 48.0, 96.0, 144.0]
        clipped = [min(pan_max_deg, max(pan_min_deg, value)) for value in values]
        return list(dict.fromkeys(clipped))
    corners = (
        (bounds.south, bounds.west),  # type: ignore[attr-defined]
        (bounds.south, bounds.east),  # type: ignore[attr-defined]
        (bounds.north, bounds.west),  # type: ignore[attr-defined]
        (bounds.north, bounds.east),  # type: ignore[attr-defined]
    )
    centre_lat = (bounds.south + bounds.north) / 2.0  # type: ignore[attr-defined]
    centre_lon = (bounds.west + bounds.east) / 2.0  # type: ignore[attr-defined]
    centre = _bearing(tower.latitude, tower.longitude, centre_lat, centre_lon)
    headings = [centre + wrap_pan(_bearing(tower.latitude, tower.longitude, lat, lon) - centre) for lat, lon in corners]
    pans = sorted(calibration.target_pan(heading) for heading in headings)
    lower = max(calibration.pan_min_deg, pan_min_deg, min(pans))
    upper = min(calibration.pan_max_deg, pan_max_deg, max(pans))
    if lower > upper:
        candidate = calibration.target_pan(centre)
        return [min(pan_max_deg, max(pan_min_deg, candidate))]
    step, _ = tower_scan_overlap_steps(overlap, 0.20)
    result = [lower]
    while result[-1] + step < upper:
        result.append(result[-1] + step)
    if upper - result[-1] > 1e-6:
        result.append(upper)
    return [round(value, 6) for value in result]


def _bearing(start_lat: float, start_lon: float, target_lat: float, target_lon: float) -> float:
    latitude_1 = math.radians(start_lat)
    latitude_2 = math.radians(target_lat)
    delta_lon = math.radians(target_lon - start_lon)
    y = math.sin(delta_lon) * math.cos(latitude_2)
    x = (
        math.cos(latitude_1) * math.sin(latitude_2)
        - math.sin(latitude_1) * math.cos(latitude_2) * math.cos(delta_lon)
    )
    return math.degrees(math.atan2(y, x)) % 360.0


def _require_port_available(host: str, port: int) -> None:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.bind((host, port))
    except OSError as exc:
        raise RuntimeError(
            f"dashboard address {host}:{port} is already in use; stop the old dashboard "
            "or configure coordinator.dashboard_port"
        ) from exc
