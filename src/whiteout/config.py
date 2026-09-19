"""Configuration loading with explicit environment substitution."""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping


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


def _default_cameras() -> tuple[CameraConfig, ...]:
    return (
        CameraConfig("quadcopter", 8600, width=640, height=480, hfov_deg=114.6, vfov_deg=99.4),
        CameraConfig("fixed-wing", 8610, width=640, height=360, hfov_deg=69.0, vfov_deg=42.6),
        CameraConfig("tower-1", 8630, width=640, height=360, hfov_deg=60.0, vfov_deg=36.1),
        CameraConfig("tower-2", 8640, width=640, height=360, hfov_deg=60.0, vfov_deg=36.1),
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


@dataclass(frozen=True, slots=True)
class AppConfig:
    sim_host: str = "127.0.0.1"
    placement_owner: str = "unassigned"
    cameras: tuple[CameraConfig, ...] = field(default_factory=_default_cameras)
    mavlink: tuple[MavlinkConfig, ...] = field(default_factory=_default_mavlink)
    track_api: TrackApiConfig = field(default_factory=TrackApiConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)


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
    if config.track_api.allow_submission and not config.track_api.endpoint:
        raise ConfigError("track submission opt-in requires endpoint")
    if not config.track_api.name.strip():
        raise ConfigError("track API name must not be empty")


def load_config(path: str | Path, environ: Mapping[str, str] | None = None) -> AppConfig:
    raw = Path(path).read_text(encoding="utf-8")
    return config_from_mapping(_parse_document(substitute_environment(raw, environ)))
