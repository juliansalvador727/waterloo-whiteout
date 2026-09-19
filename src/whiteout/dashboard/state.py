"""Thread-safe mission state shared by live, replay, and dashboard views."""

from __future__ import annotations

import math
import threading
from collections import deque
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Mapping, Protocol, Sequence

from ..camera import CameraFrame
from ..models import ActionRecommendation, Detection, SearchMode, Telemetry, Track


CAMERA_NAMES = ("quadcopter", "fixed-wing", "tower-1", "tower-2")


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _iso(value: datetime | None) -> str | None:
    return value.isoformat() if value is not None else None


def _age_seconds(now: datetime, then: datetime | None) -> float | None:
    if then is None:
        return None
    return max(0.0, (now - then).total_seconds())


class ConnectionState(str, Enum):
    OFFLINE = "OFFLINE"
    DEGRADED = "DEGRADED"
    ONLINE = "ONLINE"


class AssetActivity(str, Enum):
    IDLE = "IDLE"
    PANNING = "PANNING"
    TRANSIT = "TRANSIT"
    SEARCHING = "SEARCHING"
    CONFIRMING = "CONFIRMING"
    TRACKING = "TRACKING"
    REACQUIRING = "REACQUIRING"
    ERROR = "ERROR"


class TargetPresentation(str, Enum):
    NONE = "NONE"
    FOUND = "FOUND"
    LOCKED = "LOCKED"
    PREDICTED = "PREDICTED"
    LOST = "LOST"


@dataclass(frozen=True, slots=True)
class SiteMetadata:
    name: str
    centre_lat: float
    centre_lon: float
    extent_m: float
    bounds3413: tuple[float, float, float, float] | None = None


@dataclass(frozen=True, slots=True)
class MissionEvent:
    sequence: int
    timestamp: datetime
    kind: str
    source: str
    payload: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "sequence": self.sequence,
            "timestamp": self.timestamp.isoformat(),
            "kind": self.kind,
            "source": self.source,
            "payload": dict(self.payload),
        }


class MissionRecorder(Protocol):
    def record_event(self, event: MissionEvent) -> None: ...

    def record_frame(self, sequence: int, frame: CameraFrame) -> str | None: ...


@dataclass(slots=True)
class _AssetRecord:
    frame: CameraFrame | None = None
    frame_times: deque[datetime] = field(default_factory=lambda: deque(maxlen=30))
    telemetry: Telemetry | None = None
    detections: list[Detection] = field(default_factory=list)
    activity: AssetActivity = AssetActivity.IDLE
    reason: str = "awaiting data"
    assignment: str | None = None
    pan_deg: float | None = None
    tilt_deg: float | None = None
    search_path: list[tuple[float, float]] = field(default_factory=list)
    fov_footprint: list[tuple[float, float]] = field(default_factory=list)
    coverage: deque[list[tuple[float, float]]] = field(default_factory=lambda: deque(maxlen=300))


def project_3413(latitude: float, longitude: float) -> tuple[float, float]:
    """Project WGS84 latitude/longitude to the simulator's EPSG:3413 metre grid."""

    try:
        from pyproj import Transformer  # type: ignore[import-not-found]

        transformer = Transformer.from_crs("EPSG:4326", "EPSG:3413", always_xy=True)
        x, y = transformer.transform(longitude, latitude)
        return float(x), float(y)
    except ImportError:
        # Exact forward form of the polar stereographic definition used by EPSG:3413.
        a = 6_378_137.0
        eccentricity = 0.081819190842621
        latitude_ts = math.radians(70.0)
        longitude_origin = math.radians(-45.0)
        latitude_rad = math.radians(latitude)
        longitude_rad = math.radians(longitude)
        tc = math.tan(math.pi / 4.0 - latitude_ts / 2.0) / (
            ((1.0 - eccentricity * math.sin(latitude_ts)) / (1.0 + eccentricity * math.sin(latitude_ts)))
            ** (eccentricity / 2.0)
        )
        mc = math.cos(latitude_ts) / math.sqrt(
            1.0 - eccentricity * eccentricity * math.sin(latitude_ts) ** 2
        )
        t = math.tan(math.pi / 4.0 - latitude_rad / 2.0) / (
            ((1.0 - eccentricity * math.sin(latitude_rad)) / (1.0 + eccentricity * math.sin(latitude_rad)))
            ** (eccentricity / 2.0)
        )
        rho = a * mc * t / tc
        delta = longitude_rad - longitude_origin
        return rho * math.sin(delta), -rho * math.cos(delta)


class MissionStateStore:
    """Latest-value state bus with bounded history and optional session recording."""

    def __init__(
        self,
        *,
        camera_names: Sequence[str] = CAMERA_NAMES,
        history_s: float = 300.0,
        recorder: MissionRecorder | None = None,
    ) -> None:
        if history_s <= 0:
            raise ValueError("history_s must be positive")
        self._lock = threading.RLock()
        self._assets = {name: _AssetRecord() for name in camera_names}
        self._history_s = history_s
        self._events: deque[MissionEvent] = deque(maxlen=2_000)
        self._sequence = 0
        self._recorder = recorder
        self._site: SiteMetadata | None = None
        self._mode = SearchMode.SEARCH
        self._recommendation: ActionRecommendation | None = None
        self._track: Track | None = None
        self._observed_at: datetime | None = None
        self._observed_by: str | None = None
        self._submission = {"status": "DISABLED", "detail": "no submissions observed", "timestamp": None}

    @property
    def camera_names(self) -> tuple[str, ...]:
        return tuple(self._assets)

    def _asset(self, name: str) -> _AssetRecord:
        try:
            return self._assets[name]
        except KeyError as exc:
            raise ValueError(f"unknown asset {name!r}") from exc

    def _publish(
        self,
        kind: str,
        source: str,
        payload: Mapping[str, Any],
        *,
        timestamp: datetime | None = None,
    ) -> MissionEvent:
        event = MissionEvent(self._sequence + 1, timestamp or _utc_now(), kind, source, payload)
        self._sequence = event.sequence
        self._events.append(event)
        cutoff = event.timestamp.timestamp() - self._history_s
        while self._events and self._events[0].timestamp.timestamp() < cutoff:
            self._events.popleft()
        if self._recorder is not None:
            self._recorder.record_event(event)
        return event

    def set_site(self, site: SiteMetadata, *, publish: bool = True) -> None:
        with self._lock:
            self._site = site
            if publish:
                self._publish("site.updated", "simulator", asdict(site))

    def update_frame(self, frame: CameraFrame) -> None:
        with self._lock:
            asset = self._asset(frame.camera)
            asset.frame = frame
            asset.frame_times.append(frame.timestamp)
            if self._recorder is not None:
                path = self._recorder.record_frame(self._sequence + 1, frame)
                if path:
                    self._publish(
                        "frame.saved",
                        frame.camera,
                        {"camera": frame.camera, "path": path},
                        timestamp=frame.timestamp,
                    )

    def update_telemetry(self, telemetry: Telemetry) -> None:
        with self._lock:
            self._asset(telemetry.vehicle).telemetry = telemetry
            self._publish(
                "telemetry.updated",
                telemetry.vehicle,
                {
                    "vehicle": telemetry.vehicle,
                    "latitude": telemetry.latitude,
                    "longitude": telemetry.longitude,
                    "altitude_m": telemetry.altitude_m,
                    "roll_rad": telemetry.roll_rad,
                    "pitch_rad": telemetry.pitch_rad,
                    "yaw_rad": telemetry.yaw_rad,
                    "attitude_timestamp": _iso(telemetry.attitude_timestamp),
                },
                timestamp=telemetry.timestamp,
            )

    def update_detection(self, detection: Detection) -> None:
        with self._lock:
            asset = self._asset(detection.camera)
            asset.detections = [detection]
            payload: dict[str, Any] = {
                "camera": detection.camera,
                "pixel": asdict(detection.pixel),
                "confidence": detection.confidence,
                "label": detection.label,
            }
            if detection.bbox is not None:
                payload["bbox"] = asdict(detection.bbox)
            self._publish("detection.updated", detection.camera, payload, timestamp=detection.timestamp)

    def clear_detections(self, camera: str, *, timestamp: datetime | None = None) -> None:
        with self._lock:
            self._asset(camera).detections.clear()
            self._publish("detection.cleared", camera, {"camera": camera}, timestamp=timestamp)

    def update_asset_activity(
        self,
        asset_name: str,
        activity: AssetActivity,
        *,
        reason: str,
        assignment: str | None = None,
        pan_deg: float | None = None,
        tilt_deg: float | None = None,
        search_path: Sequence[tuple[float, float]] | None = None,
        fov_footprint: Sequence[tuple[float, float]] | None = None,
    ) -> None:
        """Publish planner state; path and footprint points are EPSG:3413 ``(x, y)`` metres."""

        with self._lock:
            asset = self._asset(asset_name)
            asset.activity = activity
            asset.reason = reason
            asset.assignment = assignment
            asset.pan_deg = pan_deg
            asset.tilt_deg = tilt_deg
            if search_path is not None:
                asset.search_path = list(search_path)
            if fov_footprint is not None:
                asset.fov_footprint = list(fov_footprint)
                if fov_footprint:
                    asset.coverage.append(list(fov_footprint))
            self._publish(
                "asset.activity",
                asset_name,
                {
                    "activity": activity.value,
                    "reason": reason,
                    "assignment": assignment,
                    "pan_deg": pan_deg,
                    "tilt_deg": tilt_deg,
                    "search_path": list(search_path or asset.search_path),
                    "fov_footprint": list(fov_footprint or asset.fov_footprint),
                },
            )

    def update_recommendation(self, recommendation: ActionRecommendation) -> None:
        with self._lock:
            self._recommendation = recommendation
            self._mode = recommendation.mode
            self._publish(
                "mission.recommendation",
                "coordinator",
                {
                    "mode": recommendation.mode.value,
                    "action": recommendation.action,
                    "reason": recommendation.reason,
                },
            )

    def update_track(self, track: Track, *, observed: bool, sensor: str | None = None) -> None:
        with self._lock:
            self._track = track
            if observed:
                self._observed_at = track.last_update
                self._observed_by = sensor
            self._publish(
                "track.observed" if observed else "track.predicted",
                sensor or "tracker",
                {
                    "track_id": track.track_id,
                    "latitude": track.latitude,
                    "longitude": track.longitude,
                    "velocity_north_mps": track.velocity_north_mps,
                    "velocity_east_mps": track.velocity_east_mps,
                    "uncertainty_m": track.uncertainty_m,
                    "observed": observed,
                },
                timestamp=track.last_update,
            )

    def update_submission(self, status: str, detail: str, *, timestamp: datetime | None = None) -> None:
        with self._lock:
            when = timestamp or _utc_now()
            self._submission = {"status": status, "detail": detail, "timestamp": when.isoformat()}
            self._publish("submission.status", "track-api", dict(self._submission), timestamp=when)

    def latest_frame(self, camera: str) -> CameraFrame | None:
        with self._lock:
            return self._asset(camera).frame

    def latest_detections(self, camera: str) -> tuple[Detection, ...]:
        with self._lock:
            asset = self._asset(camera)
            if asset.frame is None:
                return ()
            return tuple(
                detection
                for detection in asset.detections
                if 0.0 <= (asset.frame.timestamp - detection.timestamp).total_seconds() < 2.0
            )

    def best_target_frame(self) -> tuple[CameraFrame, Detection] | None:
        """Return the freshest high-confidence detection that still has its source frame."""

        with self._lock:
            candidates: list[tuple[datetime, float, CameraFrame, Detection]] = []
            for asset in self._assets.values():
                if asset.frame is None:
                    continue
                for detection in asset.detections:
                    detection_age = (asset.frame.timestamp - detection.timestamp).total_seconds()
                    if detection.bbox is not None and 0.0 <= detection_age < 2.0:
                        candidates.append(
                            (detection.timestamp, detection.confidence, asset.frame, detection)
                        )
            if not candidates:
                return None
            _, _, frame, detection = max(candidates, key=lambda item: (item[0], item[1]))
            return frame, detection

    def _target_status(self, now: datetime) -> tuple[TargetPresentation, float | None]:
        observed_age = _age_seconds(now, self._observed_at)
        freshest_detection = min(
            (
                age
                for asset in self._assets.values()
                for detection in asset.detections
                if (age := _age_seconds(now, detection.timestamp)) is not None
            ),
            default=None,
        )
        if self._track is None:
            if freshest_detection is not None and freshest_detection < 2.0:
                return TargetPresentation.FOUND, freshest_detection
            return TargetPresentation.NONE, None
        if observed_age is not None and observed_age < 2.0:
            if self._mode == SearchMode.TRACK:
                return TargetPresentation.LOCKED, observed_age
            return TargetPresentation.FOUND, observed_age
        if observed_age is not None and observed_age < 10.0:
            return TargetPresentation.PREDICTED, observed_age
        return TargetPresentation.LOST, observed_age

    @staticmethod
    def _predict_track(track: Track, seconds: float) -> tuple[float, float]:
        meters_per_degree_lat = 111_319.490793
        meters_per_degree_lon = meters_per_degree_lat * math.cos(math.radians(track.latitude))
        return (
            track.latitude + track.velocity_north_mps * seconds / meters_per_degree_lat,
            track.longitude + track.velocity_east_mps * seconds / meters_per_degree_lon,
        )

    def snapshot(self, *, now: datetime | None = None, event_limit: int = 80) -> dict[str, Any]:
        current = now or _utc_now()
        with self._lock:
            assets: dict[str, Any] = {}
            for name, asset in self._assets.items():
                frame_age = _age_seconds(current, asset.frame.timestamp if asset.frame else None)
                telemetry_age = _age_seconds(current, asset.telemetry.timestamp if asset.telemetry else None)
                ages = [age for age in (frame_age, telemetry_age) if age is not None]
                if not ages or all(age > 3.0 for age in ages):
                    connection = ConnectionState.OFFLINE
                elif frame_age is None or telemetry_age is None or any(age > 1.0 for age in ages):
                    connection = ConnectionState.DEGRADED
                else:
                    connection = ConnectionState.ONLINE
                fps = 0.0
                if len(asset.frame_times) >= 2:
                    span = (asset.frame_times[-1] - asset.frame_times[0]).total_seconds()
                    if span > 0:
                        fps = (len(asset.frame_times) - 1) / span
                telemetry_payload: dict[str, Any] | None = None
                projected: dict[str, float] | None = None
                if asset.telemetry is not None:
                    telemetry_payload = {
                        **asdict(asset.telemetry),
                        "timestamp": asset.telemetry.timestamp.isoformat(),
                        "attitude_timestamp": _iso(asset.telemetry.attitude_timestamp),
                        "age_s": telemetry_age,
                    }
                    x_m, y_m = project_3413(asset.telemetry.latitude, asset.telemetry.longitude)
                    projected = {"x_m": x_m, "y_m": y_m}
                assets[name] = {
                    "name": name,
                    "connection": connection.value,
                    "activity": asset.activity.value,
                    "reason": asset.reason,
                    "assignment": asset.assignment,
                    "pan_deg": asset.pan_deg,
                    "tilt_deg": asset.tilt_deg,
                    "frame_age_s": frame_age,
                    "fps": fps,
                    "telemetry": telemetry_payload,
                    "projected": projected,
                    "search_path": asset.search_path,
                    "fov_footprint": asset.fov_footprint,
                    "coverage": list(asset.coverage),
                    "detections": [
                        {
                            "camera": detection.camera,
                            "pixel": asdict(detection.pixel),
                            "confidence": detection.confidence,
                            "label": detection.label,
                            "timestamp": detection.timestamp.isoformat(),
                            "age_s": _age_seconds(current, detection.timestamp),
                            "bbox": asdict(detection.bbox) if detection.bbox else None,
                        }
                        for detection in asset.detections
                    ],
                }

            status, target_age = self._target_status(current)
            target_frame = self.best_target_frame()
            target: dict[str, Any] = {
                "status": status.value,
                "age_s": target_age,
                "observed_by": self._observed_by,
                "source_camera": (
                    target_frame[0].camera
                    if target_frame is not None and target_age is not None and target_age < 2.0
                    else None
                ),
                "observed_at": _iso(self._observed_at),
                "track": None,
            }
            if self._track is not None:
                seconds = min(target_age or 0.0, 10.0) if status == TargetPresentation.PREDICTED else 0.0
                latitude, longitude = self._predict_track(self._track, seconds)
                x_m, y_m = project_3413(latitude, longitude)
                target["track"] = {
                    **asdict(self._track),
                    "last_update": self._track.last_update.isoformat(),
                    "display_latitude": latitude,
                    "display_longitude": longitude,
                    "x_m": x_m,
                    "y_m": y_m,
                    "predicted_seconds": seconds,
                    "display_uncertainty_m": self._track.uncertainty_m + seconds,
                }

            return {
                "schema_version": 1,
                "generated_at": current.isoformat(),
                "sequence": self._sequence,
                "mission": {
                    "mode": self._mode.value,
                    "recommendation": (
                        {
                            "action": self._recommendation.action,
                            "reason": self._recommendation.reason,
                        }
                        if self._recommendation
                        else None
                    ),
                },
                "site": asdict(self._site) if self._site else None,
                "assets": assets,
                "target": target,
                "submission": dict(self._submission),
                "events": [event.to_dict() for event in list(self._events)[-event_limit:]],
            }
