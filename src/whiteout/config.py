"""Configuration loading with explicit environment substitution."""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

from .tower import TowerWorldPose


class ConfigError(ValueError):
    pass


@dataclass(frozen=True, slots=True)
class CameraConfig:
    name: str
    port: int
    path: str = "/stream"
    width: int | None = None
    height: int | None = None
    hfov_deg: float | None = None
    vfov_deg: float | None = None
    crop_right_px: int = 0


@dataclass(frozen=True, slots=True)
class MavlinkConfig:
    name: str
    port: int


@dataclass(frozen=True, slots=True)
class TrackApiConfig:
    allow_submission: bool = False
    endpoint: str | None = None
    name: str = "Sierra One"
    include_speed: bool = False


@dataclass(frozen=True, slots=True)
class LoggingConfig:
    jsonl_path: str | None = None


@dataclass(frozen=True, slots=True)
class CoordinatorConfig:
    detector_weights: str = "best.pt"
    detector_imgsz: int = 1280
    detector_confidence: float = 0.25
    quadcopter_mission: str = "missions/quadcopter_search.waypoints"
    fixed_wing_mission: str = "missions/fixed_wing_search.waypoints"
    telemetry_skew_s: float = 0.25
    telemetry_stale_s: float = 2.0
    tick_hz: float = 10.0
    command_interval_s: float = 1.0
    tower_dwell_s: float = 1.0
    tower_detection_hold_s: float = 1.5
    tower_horizontal_overlap: float = 0.20
    reacquire_grid_rows: int = 12
    reacquire_grid_columns: int = 12
    reacquire_grid_half_life_s: float = 30.0
    reacquire_grid_neighborhood_cells: int = 2
    dashboard_bind: str = "127.0.0.1"
    dashboard_port: int = 8070


@dataclass(frozen=True, slots=True)
class TowerMotionConfig:
    name: str
    pan_min_deg: float = -135.0
    pan_max_deg: float = 135.0
    tilt_min_deg: float = -15.0
    tilt_max_deg: float = 30.0
    pan_rate_deg_s: float = 24.0
    tilt_rate_deg_s: float = 12.0
    command_hz: float = 10.0


@dataclass(frozen=True, slots=True)
class CourseBounds:
    south: float
    west: float
    north: float
    east: float

    def __post_init__(self) -> None:
        if self.south > self.north or self.west > self.east:
            raise ConfigError("course bounds must be ordered south, west, north, east")

    def contains(self, latitude: float, longitude: float) -> bool:
        return self.south <= latitude <= self.north and self.west <= longitude <= self.east


def _default_cameras() -> tuple[CameraConfig, ...]:
    return (
        CameraConfig(
            "quadcopter",
            8600,
            width=960,
            height=720,
            hfov_deg=114.6,
            vfov_deg=99.4,
            crop_right_px=40,
        ),
        CameraConfig("fixed-wing", 8610, width=1280, height=720, hfov_deg=69.0, vfov_deg=42.6),
        CameraConfig("tower-1", 8630, width=1280, height=720, hfov_deg=60.0, vfov_deg=36.1),
        CameraConfig("tower-2", 8640, width=1280, height=720, hfov_deg=60.0, vfov_deg=36.1),
    )


def _default_mavlink() -> tuple[MavlinkConfig, ...]:
    return tuple(
        MavlinkConfig(name, port)
        for name, port in zip(
            ("quadcopter", "fixed-wing", "tower-1", "tower-2"),
            (14550, 14560, 14580, 14590),
            strict=True,
        )
    )


def _default_tower_motion() -> tuple[TowerMotionConfig, ...]:
    return (
        TowerMotionConfig("tower-1"),
        TowerMotionConfig("tower-2"),
    )


@dataclass(frozen=True, slots=True)
class AppConfig:
    sim_host: str = "127.0.0.1"
    placement_owner: str = "unassigned"
    cameras: tuple[CameraConfig, ...] = field(default_factory=_default_cameras)
    mavlink: tuple[MavlinkConfig, ...] = field(default_factory=_default_mavlink)
    track_api: TrackApiConfig = field(default_factory=TrackApiConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    course_bounds: CourseBounds | None = None
    coordinator: CoordinatorConfig = field(default_factory=CoordinatorConfig)
    tower_poses: tuple[TowerWorldPose, ...] = ()
    tower_motion: tuple[TowerMotionConfig, ...] = field(default_factory=_default_tower_motion)


_ENV_PATTERN = re.compile(r"\$\{([A-Za-z_][A-Za-z0-9_]*)(?::-([^}]*))?\}")


def substitute_environment(text: str, environ: Mapping[str, str] | None = None) -> str:
    values = os.environ if environ is None else environ

    def replace(match: re.Match[str]) -> str:
        name, fallback = match.group(1), match.group(2)
        if name in values:
            return values[name]
        if fallback is not None:
            return fallback
        raise ConfigError(f"environment variable {name!r} is not set")

    return _ENV_PATTERN.sub(replace, text)


def _parse_document(text: str) -> Mapping[str, Any]:
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        try:
            import yaml  # type: ignore[import-not-found]
        except ImportError as exc:
            raise ConfigError(
                "YAML configuration requires PyYAML; install whiteout[yaml] or use JSON"
            ) from exc
        parsed = yaml.safe_load(text)
    if not isinstance(parsed, dict):
        raise ConfigError("configuration root must be a mapping")
    return parsed


def config_from_mapping(data: Mapping[str, Any]) -> AppConfig:
    cameras_data = data.get("cameras")
    cameras = (
        tuple(
            CameraConfig(
                name=str(item["name"]),
                port=int(item["port"]),
                path=str(item.get("path", "/stream")),
                width=int(item["width"]) if item.get("width") is not None else None,
                height=int(item["height"]) if item.get("height") is not None else None,
                hfov_deg=float(item["hfov_deg"]) if item.get("hfov_deg") is not None else None,
                vfov_deg=float(item["vfov_deg"]) if item.get("vfov_deg") is not None else None,
                crop_right_px=int(item.get("crop_right_px", 0)),
            )
            for item in cameras_data
        )
        if cameras_data is not None
        else _default_cameras()
    )
    mavlink_data = data.get("mavlink")
    mavlink = (
        tuple(MavlinkConfig(str(item["name"]), int(item["port"])) for item in mavlink_data)
        if mavlink_data is not None
        else _default_mavlink()
    )
    api_data = data.get("track_api", {})
    log_data = data.get("logging", {})
    coordinator_data = data.get("coordinator", {})
    tower_poses_data = data.get("tower_poses", ())
    tower_motion_data = data.get("tower_motion", {})
    if not isinstance(tower_motion_data, Mapping):
        raise ConfigError("tower_motion must be a mapping keyed by tower name")
    unknown_towers = set(tower_motion_data) - {"tower-1", "tower-2"}
    if unknown_towers:
        raise ConfigError(f"tower_motion contains unknown towers: {sorted(unknown_towers)}")
    default_motion = {item.name: item for item in _default_tower_motion()}
    tower_motion: list[TowerMotionConfig] = []
    for name in ("tower-1", "tower-2"):
        item = tower_motion_data.get(name, {})
        if not isinstance(item, Mapping):
            raise ConfigError(f"tower_motion.{name} must be a mapping")
        defaults = default_motion[name]
        tower_motion.append(TowerMotionConfig(
            name=name,
            pan_min_deg=float(item.get("pan_min_deg", defaults.pan_min_deg)),
            pan_max_deg=float(item.get("pan_max_deg", defaults.pan_max_deg)),
            tilt_min_deg=float(item.get("tilt_min_deg", defaults.tilt_min_deg)),
            tilt_max_deg=float(item.get("tilt_max_deg", defaults.tilt_max_deg)),
            pan_rate_deg_s=float(item.get("pan_rate_deg_s", defaults.pan_rate_deg_s)),
            tilt_rate_deg_s=float(item.get("tilt_rate_deg_s", defaults.tilt_rate_deg_s)),
            command_hz=float(item.get("command_hz", defaults.command_hz)),
        ))
    bounds_data = data.get("course_bounds")
    course_bounds = None
    if bounds_data is not None:
        if isinstance(bounds_data, (list, tuple)) and len(bounds_data) == 4:
            west, south, east, north = (float(value) for value in bounds_data)
        elif isinstance(bounds_data, Mapping):
            south = float(bounds_data.get("south", bounds_data.get("min_latitude")))
            west = float(bounds_data.get("west", bounds_data.get("min_longitude")))
            north = float(bounds_data.get("north", bounds_data.get("max_latitude")))
            east = float(bounds_data.get("east", bounds_data.get("max_longitude")))
        else:
            raise ConfigError("course_bounds must be a mapping or [west, south, east, north]")
        course_bounds = CourseBounds(south, west, north, east)
    config = AppConfig(
        sim_host=str(data.get("sim_host", "127.0.0.1")),
        placement_owner=str(data.get("placement_owner", "unassigned")),
        cameras=cameras,
        mavlink=mavlink,
        track_api=TrackApiConfig(
            allow_submission=bool(api_data.get("allow_submission", False)),
            endpoint=api_data.get("endpoint"),
            name=str(api_data.get("name", "Sierra One")),
            include_speed=bool(api_data.get("include_speed", False)),
        ),
        logging=LoggingConfig(jsonl_path=log_data.get("jsonl_path")),
        course_bounds=course_bounds,
        coordinator=CoordinatorConfig(
            detector_weights=str(coordinator_data.get("detector_weights", "best.pt")),
            detector_imgsz=int(coordinator_data.get("detector_imgsz", 1280)),
            detector_confidence=float(coordinator_data.get("detector_confidence", 0.25)),
            quadcopter_mission=str(coordinator_data.get("quadcopter_mission", "missions/quadcopter_search.waypoints")),
            fixed_wing_mission=str(coordinator_data.get("fixed_wing_mission", "missions/fixed_wing_search.waypoints")),
            telemetry_skew_s=float(coordinator_data.get("telemetry_skew_s", 0.25)),
            telemetry_stale_s=float(coordinator_data.get("telemetry_stale_s", 2.0)),
            tick_hz=float(coordinator_data.get("tick_hz", 10.0)),
            command_interval_s=float(coordinator_data.get("command_interval_s", 1.0)),
            tower_dwell_s=float(coordinator_data.get("tower_dwell_s", 1.0)),
            tower_detection_hold_s=float(coordinator_data.get("tower_detection_hold_s", 1.5)),
            tower_horizontal_overlap=float(coordinator_data.get("tower_horizontal_overlap", 0.20)),
            reacquire_grid_rows=int(coordinator_data.get("reacquire_grid_rows", 12)),
            reacquire_grid_columns=int(coordinator_data.get("reacquire_grid_columns", 12)),
            reacquire_grid_half_life_s=float(coordinator_data.get("reacquire_grid_half_life_s", 30.0)),
            reacquire_grid_neighborhood_cells=int(coordinator_data.get("reacquire_grid_neighborhood_cells", 2)),
            dashboard_bind=str(coordinator_data.get("dashboard_bind", "127.0.0.1")),
            dashboard_port=int(coordinator_data.get("dashboard_port", 8070)),
        ),
        tower_poses=tuple(
            TowerWorldPose(
                str(item["name"]),
                float(item["latitude"]),
                float(item["longitude"]),
                float(item["camera_world_z_m"]),
            )
            for item in tower_poses_data
        ),
        tower_motion=tuple(tower_motion),
    )
    _validate(config)
    return config


def _validate(config: AppConfig) -> None:
    for item in (*config.cameras, *config.mavlink):
        if not 1 <= item.port <= 65535:
            raise ConfigError(f"invalid port for {item.name}: {item.port}")
    for camera in config.cameras:
        if camera.width is not None and camera.width <= 0:
            raise ConfigError(f"invalid camera width for {camera.name}: {camera.width}")
        if camera.height is not None and camera.height <= 0:
            raise ConfigError(f"invalid camera height for {camera.name}: {camera.height}")
        if camera.hfov_deg is not None and not 0 < camera.hfov_deg < 180:
            raise ConfigError(f"invalid horizontal FOV for {camera.name}: {camera.hfov_deg}")
        if camera.vfov_deg is not None and not 0 < camera.vfov_deg < 180:
            raise ConfigError(f"invalid vertical FOV for {camera.name}: {camera.vfov_deg}")
        if camera.crop_right_px < 0:
            raise ConfigError(
                f"invalid right crop for {camera.name}: {camera.crop_right_px}"
            )
        if camera.width is not None and camera.crop_right_px >= camera.width:
            raise ConfigError(
                f"right crop for {camera.name} must be smaller than its width"
            )
    if config.track_api.allow_submission and not config.track_api.endpoint:
        raise ConfigError("track submission opt-in requires endpoint")
    if not config.track_api.name.strip():
        raise ConfigError("track API name must not be empty")
    runtime = config.coordinator
    if runtime.detector_imgsz <= 0 or not 0 < runtime.detector_confidence <= 1:
        raise ConfigError("coordinator detector settings are invalid")
    if runtime.telemetry_skew_s < 0 or runtime.telemetry_stale_s <= 0:
        raise ConfigError("coordinator telemetry timing is invalid")
    if (
        runtime.tick_hz <= 0
        or runtime.command_interval_s <= 0
        or runtime.tower_dwell_s <= 0
        or runtime.tower_detection_hold_s <= 0
    ):
        raise ConfigError("coordinator timing values must be positive")
    if not 0.20 <= runtime.tower_horizontal_overlap < 1:
        raise ConfigError("tower horizontal overlap must be in [0.20, 1)")
    if runtime.reacquire_grid_rows <= 0 or runtime.reacquire_grid_columns <= 0:
        raise ConfigError("reacquisition grid dimensions must be positive")
    if (
        runtime.reacquire_grid_half_life_s <= 0
        or runtime.reacquire_grid_neighborhood_cells < 0
    ):
        raise ConfigError("reacquisition grid decay and neighborhood are invalid")
    if not 1 <= runtime.dashboard_port <= 65535:
        raise ConfigError("coordinator dashboard port must be between 1 and 65535")
    if config.tower_poses and {pose.name for pose in config.tower_poses} != {"tower-1", "tower-2"}:
        raise ConfigError("tower_poses must contain exactly tower-1 and tower-2")
    if (
        len(config.tower_motion) != 2
        or {item.name for item in config.tower_motion} != {"tower-1", "tower-2"}
    ):
        raise ConfigError("tower_motion must contain exactly tower-1 and tower-2")
    for item in config.tower_motion:
        if not -144.0 <= item.pan_min_deg < item.pan_max_deg <= 144.0:
            raise ConfigError(
                f"tower_motion.{item.name} pan limits must be ordered within [-144, 144]"
            )
        if not -22.5 <= item.tilt_min_deg < item.tilt_max_deg <= 37.5:
            raise ConfigError(
                f"tower_motion.{item.name} tilt limits must be ordered within [-22.5, 37.5]"
            )
        if not item.tilt_min_deg <= 7.5 <= item.tilt_max_deg:
            raise ConfigError(f"tower_motion.{item.name} tilt limits must include 7.5 degrees")
        if item.pan_rate_deg_s <= 0 or item.tilt_rate_deg_s <= 0 or item.command_hz <= 0:
            raise ConfigError(f"tower_motion.{item.name} rates and command_hz must be positive")


def load_config(path: str | Path, environ: Mapping[str, str] | None = None) -> AppConfig:
    raw = Path(path).read_text(encoding="utf-8")
    return config_from_mapping(_parse_document(substitute_environment(raw, environ)))
