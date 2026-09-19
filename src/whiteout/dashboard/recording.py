"""Versioned dashboard session recording and deterministic replay."""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping

from ..camera import CameraFrame
from ..models import (
    ActionRecommendation,
    BoundingBox,
    Detection,
    Pixel,
    SearchMode,
    Telemetry,
    Track,
)
from .state import AssetActivity, MissionEvent, MissionStateStore, SiteMetadata


SESSION_SCHEMA_VERSION = 1


def _sanitized(value: Any, *, key: str = "") -> Any:
    lowered = key.lower()
    if any(fragment in lowered for fragment in ("password", "secret", "token", "credential", "api_key")):
        return "[redacted]"
    if isinstance(value, Mapping):
        return {str(item_key): _sanitized(item_value, key=str(item_key)) for item_key, item_value in value.items()}
    if isinstance(value, (list, tuple)):
        return [_sanitized(item) for item in value]
    return value


class SessionRecorder:
    """Append mission events and sampled frames without retaining secrets."""

    def __init__(
        self,
        directory: str | Path,
        *,
        metadata: Mapping[str, Any] | None = None,
        frame_sample_hz: float = 2.0,
    ) -> None:
        if frame_sample_hz <= 0:
            raise ValueError("frame_sample_hz must be positive")
        self.directory = Path(directory)
        self.frames_directory = self.directory / "frames"
        self.directory.mkdir(parents=True, exist_ok=True)
        self.frames_directory.mkdir(parents=True, exist_ok=True)
        self._events_path = self.directory / "events.jsonl"
        self._lock = threading.Lock()
        self._minimum_frame_interval_s = 1.0 / frame_sample_hz
        self._last_frame_at: dict[str, datetime] = {}
        manifest = {
            "schema_version": SESSION_SCHEMA_VERSION,
            "created_at": datetime.now().astimezone().isoformat(),
            "frame_sample_hz": frame_sample_hz,
            "metadata": _sanitized(dict(metadata or {})),
        }
        (self.directory / "session.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8"
        )

    def record_event(self, event: MissionEvent) -> None:
        with self._lock:
            with self._events_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(event.to_dict(), separators=(",", ":")) + "\n")

    def record_frame(self, sequence: int, frame: CameraFrame) -> str | None:
        with self._lock:
            previous = self._last_frame_at.get(frame.camera)
            if previous is not None:
                age = (frame.timestamp - previous).total_seconds()
                if age < self._minimum_frame_interval_s:
                    return None
            self._last_frame_at[frame.camera] = frame.timestamp
            camera_directory = self.frames_directory / frame.camera
            camera_directory.mkdir(parents=True, exist_ok=True)
            relative = Path("frames") / frame.camera / f"{sequence:09d}.jpg"
            (self.directory / relative).write_bytes(frame.jpeg)
            return relative.as_posix()


@dataclass(frozen=True, slots=True)
class RecordedEvent:
    sequence: int
    timestamp: datetime
    kind: str
    source: str
    payload: Mapping[str, Any]


def _parse_timestamp(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


class SessionReplay:
    """Replay a recorded session into the same state store used by live data."""

    def __init__(self, directory: str | Path, store: MissionStateStore, *, speed: float = 1.0) -> None:
        if speed <= 0:
            raise ValueError("speed must be positive")
        self.directory = Path(directory)
        manifest = json.loads((self.directory / "session.json").read_text(encoding="utf-8"))
        if manifest.get("schema_version") != SESSION_SCHEMA_VERSION:
            raise ValueError("unsupported session schema version")
        self.events = self._load_events(self.directory / "events.jsonl")
        self.store = store
        self.speed = speed
        self.cursor = 0
        self.playing = True
        self._wall_anchor = time.monotonic()
        self._recording_anchor = self.events[0].timestamp if self.events else None

    @staticmethod
    def _load_events(path: Path) -> list[RecordedEvent]:
        events: list[RecordedEvent] = []
        if not path.exists():
            return events
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            item = json.loads(line)
            events.append(
                RecordedEvent(
                    int(item["sequence"]),
                    _parse_timestamp(item["timestamp"]),
                    str(item["kind"]),
                    str(item["source"]),
                    dict(item.get("payload", {})),
                )
            )
        events.sort(key=lambda event: (event.timestamp, event.sequence))
        return events

    @property
    def duration_s(self) -> float:
        if len(self.events) < 2:
            return 0.0
        return max(0.0, (self.events[-1].timestamp - self.events[0].timestamp).total_seconds())

    @property
    def position_s(self) -> float:
        if not self.events or self.cursor == 0:
            return 0.0
        index = min(self.cursor - 1, len(self.events) - 1)
        return max(0.0, (self.events[index].timestamp - self.events[0].timestamp).total_seconds())

    @property
    def current_time(self) -> datetime | None:
        if not self.events:
            return None
        if self.cursor == 0:
            return self.events[0].timestamp
        return self.events[min(self.cursor - 1, len(self.events) - 1)].timestamp

    def set_playing(self, playing: bool) -> None:
        self.playing = playing
        self._reset_anchor()

    def set_speed(self, speed: float) -> None:
        if speed not in (0.5, 1.0, 2.0, 4.0):
            raise ValueError("speed must be one of 0.5, 1, 2, or 4")
        self.speed = speed
        self._reset_anchor()

    def seek(self, seconds: float) -> None:
        if not self.events:
            return
        target = max(0.0, min(seconds, self.duration_s))
        self.cursor = 0
        # Rebuild state so seeking backward cannot leave future values behind.
        self.store = MissionStateStore(camera_names=self.store.camera_names)
        while self.cursor < len(self.events):
            elapsed = (self.events[self.cursor].timestamp - self.events[0].timestamp).total_seconds()
            if elapsed > target:
                break
            self._apply(self.events[self.cursor])
            self.cursor += 1
        self._reset_anchor()

    def _reset_anchor(self) -> None:
        self._wall_anchor = time.monotonic()
        self._recording_anchor = self.events[self.cursor].timestamp if self.cursor < len(self.events) else None

    def advance(self) -> int:
        if not self.playing or self.cursor >= len(self.events) or self._recording_anchor is None:
            return 0
        recording_elapsed = (time.monotonic() - self._wall_anchor) * self.speed
        applied = 0
        while self.cursor < len(self.events):
            event = self.events[self.cursor]
            offset = (event.timestamp - self._recording_anchor).total_seconds()
            if offset > recording_elapsed:
                break
            self._apply(event)
            self.cursor += 1
            applied += 1
        return applied

    def _apply(self, event: RecordedEvent) -> None:
        payload = event.payload
        if event.kind == "frame.saved":
            path = self.directory / str(payload["path"])
            if path.is_file():
                self.store.update_frame(CameraFrame(event.source, path.read_bytes(), event.timestamp))
        elif event.kind == "site.updated":
            bounds = payload.get("bounds3413")
            self.store.set_site(
                SiteMetadata(
                    str(payload["name"]),
                    float(payload["centre_lat"]),
                    float(payload["centre_lon"]),
                    float(payload["extent_m"]),
                    tuple(float(value) for value in bounds) if bounds else None,
                )
            )
        elif event.kind == "telemetry.updated":
            attitude_timestamp = payload.get("attitude_timestamp")
            self.store.update_telemetry(
                Telemetry(
                    vehicle=event.source,
                    latitude=float(payload["latitude"]),
                    longitude=float(payload["longitude"]),
                    altitude_m=float(payload["altitude_m"]),
                    roll_rad=(
                        float(payload["roll_rad"])
                        if payload.get("roll_rad") is not None
                        else None
                    ),
                    pitch_rad=(
                        float(payload["pitch_rad"])
                        if payload.get("pitch_rad") is not None
                        else None
                    ),
                    yaw_rad=(
                        float(payload["yaw_rad"])
                        if payload.get("yaw_rad") is not None
                        else None
                    ),
                    timestamp=event.timestamp,
                    attitude_timestamp=(
                        _parse_timestamp(str(attitude_timestamp))
                        if attitude_timestamp is not None
                        else None
                    ),
                )
            )
        elif event.kind == "detection.updated":
            bbox_data = payload.get("bbox")
            bbox = BoundingBox(**bbox_data) if bbox_data else None
            pixel_data = payload["pixel"]
            self.store.update_detection(
                Detection(
                    event.source,
                    Pixel(float(pixel_data["x"]), float(pixel_data["y"])),
                    float(payload["confidence"]),
                    event.timestamp,
                    str(payload.get("label", "target")),
                    bbox=bbox,
                )
            )
        elif event.kind == "detection.cleared":
            self.store.clear_detections(event.source, timestamp=event.timestamp)
        elif event.kind == "asset.activity":
            self.store.update_asset_activity(
                event.source,
                AssetActivity(str(payload["activity"])),
                reason=str(payload.get("reason", "")),
                assignment=payload.get("assignment"),
                pan_deg=payload.get("pan_deg"),
                tilt_deg=payload.get("tilt_deg"),
                search_path=payload.get("search_path"),
                fov_footprint=payload.get("fov_footprint"),
            )
        elif event.kind == "mission.recommendation":
            self.store.update_recommendation(
                ActionRecommendation(
                    SearchMode(str(payload["mode"])),
                    str(payload["action"]),
                    str(payload["reason"]),
                )
            )
        elif event.kind in ("track.observed", "track.predicted"):
            self.store.update_track(
                Track(
                    str(payload["track_id"]),
                    float(payload["latitude"]),
                    float(payload["longitude"]),
                    float(payload["velocity_north_mps"]),
                    float(payload["velocity_east_mps"]),
                    float(payload["uncertainty_m"]),
                    event.timestamp,
                ),
                observed=event.kind == "track.observed",
                sensor=event.source,
            )
        elif event.kind == "submission.status":
            self.store.update_submission(
                str(payload.get("status", "UNKNOWN")),
                str(payload.get("detail", "")),
                timestamp=event.timestamp,
            )
