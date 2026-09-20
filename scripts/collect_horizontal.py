#!/usr/bin/env python3
"""Collect horizontal boat training frames without rebuilding ArcticSim.

The live target normally belongs to ``VesselPathPlugin`` and rejects manual
pose changes on the next simulation update. With ``--take-over-target`` this
tool removes that one model, spawns the same cached Gazebo vessel once under a
unique training name, and moves that copy through the requested poses using
gzweb's existing transport bridge. No container, terrain, world, or camera
service is restarted.

The selected tower is aimed at every pose through its normal MAVLink servo
controller.  Images are the original JPEG frames from the tower MJPEG stream;
the script also writes pose metadata and approximate projected YOLO pre-labels
which must be reviewed before training.

Example (four images around the vessel's current water-safe course position):

    python scripts/collect_horizontal.py --host 10.99.0.1 \
      --camera tower-1 --samples 4 --take-over-target \
      --confirm-network --confirm-simulator-control

Use ``--poses poses.csv`` for explicit world poses.  CSV columns are
``name,x,y,yaw_deg,view_offset_deg``; ``view_offset_deg`` moves the boat across
the camera frame without changing its world position.
"""

from __future__ import annotations

import argparse
from contextlib import suppress
import csv
import hashlib
import json
import math
import threading
import time
from dataclasses import asdict, dataclass
from dataclasses import replace as dataclass_replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from whiteout.camera import CameraFrame, MjpegCamera
from whiteout.control import MavProxyConnectionError
from whiteout.objects import Tower


CAMERA_HEIGHT_M = 2.70
PAN_MIN_RAD = -math.pi
PAN_MAX_RAD = math.pi
TILT_MIN_RAD = math.radians(-45.0)  # up is negative in the Gazebo joint
TILT_MAX_RAD = math.radians(30.0)   # down is positive
BOAT_LENGTH_M = 33.6
BOAT_WIDTH_M = 8.2
BOAT_VISIBLE_HEIGHT_M = 8.0


@dataclass(frozen=True, slots=True)
class PoseSpec:
    name: str
    x: float
    y: float
    yaw_deg: float
    view_offset_deg: float = 0.0


@dataclass(frozen=True, slots=True)
class ModelPose:
    name: str
    model_id: int
    x: float
    y: float
    z: float
    yaw_rad: float


def _finite(value: float, label: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(orientation: dict[str, Any]) -> float:
    w, x, y, z = (float(orientation.get(key, 0.0)) for key in ("w", "x", "y", "z"))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_quaternion(yaw_rad: float) -> dict[str, float]:
    half = yaw_rad / 2.0
    return {"w": math.cos(half), "x": 0.0, "y": 0.0, "z": math.sin(half)}


def parse_model(model: dict[str, Any]) -> ModelPose:
    # ~/model/info nests pose fields under ``pose`` while ~/pose/info sends
    # the same position/orientation fields directly in the message body.
    pose = model.get("pose") or model
    position = pose.get("position") or {}
    return ModelPose(
        name=str(model.get("name") or ""),
        model_id=int(model.get("id") or 0),
        x=float(position.get("x") or 0.0),
        y=float(position.get("y") or 0.0),
        z=float(position.get("z") or 0.0),
        yaw_rad=yaw_from_quaternion(pose.get("orientation") or {}),
    )


def models_from_scene(message: dict[str, Any]) -> dict[str, ModelPose]:
    body = message.get("msg") or {}
    models = body.get("model") or []
    return {
        parsed.name: parsed
        for item in models
        if isinstance(item, dict) and (parsed := parse_model(item)).name
    }


def tower_joint_angles_from_scene(
    message: dict[str, Any], tower_name: str
) -> tuple[float, float]:
    """Return physical pan and positive-up tilt angles from a scene snapshot."""
    models = (message.get("msg") or {}).get("model") or []
    tower = next(
        (
            model
            for model in models
            if isinstance(model, dict) and model.get("name") == tower_name
        ),
        None,
    )
    if tower is None:
        raise RuntimeError(f"{tower_name!r} is absent from the Gazebo scene")
    angles: dict[str, float] = {}
    for joint in tower.get("joint") or []:
        if not isinstance(joint, dict):
            continue
        values = joint.get("angle") or []
        if values:
            angles[str(joint.get("name") or "").rsplit("::", 1)[-1]] = float(values[0])
    if "pan_joint" not in angles or "tilt_joint" not in angles:
        raise RuntimeError(f"{tower_name!r} scene data has no pan/tilt joint angles")
    # Gazebo's tilt joint is positive down; the public tower convention is
    # positive up.
    return math.degrees(angles["pan_joint"]), -math.degrees(angles["tilt_joint"])


def automatic_poses(anchor: ModelPose, count: int) -> list[PoseSpec]:
    """Generate poses close to a known water-safe course point.

    Offsets stay within 20 m laterally and 90 m along the vessel's current
    course.  ArcticSim's generated course has at least 190 m path clearance in
    the active scenario, so this is a conservative default for quick samples.
    """
    if not 1 <= count <= 1000:
        raise ValueError("samples must be between 1 and 1000")
    forward = (math.cos(anchor.yaw_rad), math.sin(anchor.yaw_rad))
    left = (-forward[1], forward[0])
    view_offsets = (-20.0, -7.0, 7.0, 20.0)
    poses: list[PoseSpec] = []
    for index in range(count):
        fraction = 0.0 if count == 1 else index / (count - 1)
        along = -90.0 + 180.0 * fraction
        lateral = (-20.0 if index % 2 == 0 else 20.0) if count > 1 else 0.0
        x = anchor.x + along * forward[0] + lateral * left[0]
        y = anchor.y + along * forward[1] + lateral * left[1]
        poses.append(
            PoseSpec(
                name=f"sample-{index + 1:03d}",
                x=x,
                y=y,
                yaw_deg=(math.degrees(anchor.yaw_rad) + index * 90.0) % 360.0,
                view_offset_deg=view_offsets[index % len(view_offsets)],
            )
        )
    return poses


def load_poses(path: Path) -> list[PoseSpec]:
    with path.open(newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        required = {"name", "x", "y", "yaw_deg"}
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError(f"pose CSV needs columns: {', '.join(sorted(required))}")
        poses = [
            PoseSpec(
                name=(row["name"] or "").strip(),
                x=_finite(row["x"], "x"),
                y=_finite(row["y"], "y"),
                yaw_deg=_finite(row["yaw_deg"], "yaw_deg"),
                view_offset_deg=_finite(row.get("view_offset_deg") or 0.0, "view_offset_deg"),
            )
            for row in reader
        ]
    if not poses or any(not pose.name for pose in poses):
        raise ValueError("pose CSV must contain at least one named pose")
    if len({pose.name for pose in poses}) != len(poses):
        raise ValueError("pose names must be unique")
    return poses


def aim_angles(
    tower: ModelPose, target: PoseSpec
) -> tuple[float, float]:
    dx, dy = target.x - tower.x, target.y - tower.y
    horizontal = math.hypot(dx, dy)
    if horizontal < 1.0:
        raise ValueError("target is too close to the tower to aim safely")
    target_bearing = math.atan2(dy, dx)
    camera_heading = target_bearing + math.radians(target.view_offset_deg)
    pan = wrap_pi(camera_heading - tower.yaw_rad)
    tilt = math.atan2((tower.z + CAMERA_HEIGHT_M), horizontal)
    if not TILT_MIN_RAD <= tilt <= TILT_MAX_RAD:
        raise ValueError(
            f"target requires {math.degrees(tilt):.1f} deg down tilt; "
            "tower range is -45 deg up to 30 deg down"
        )
    return pan, tilt


def aim_pwm(tower: ModelPose, target: PoseSpec) -> tuple[int, int, float, float]:
    pan, tilt = aim_angles(tower, target)
    pan_fraction = (pan - PAN_MIN_RAD) / (PAN_MAX_RAD - PAN_MIN_RAD)
    # PWM 1000 is 30 deg down and PWM 2000 is 45 deg up.
    tilt_fraction = (TILT_MAX_RAD - tilt) / (TILT_MAX_RAD - TILT_MIN_RAD)
    pan_pwm = round(1000 + 1000 * pan_fraction)
    tilt_pwm = round(1000 + 1000 * tilt_fraction)
    return pan_pwm, tilt_pwm, pan, tilt


def angles_from_pwm(pan_pwm: int, tilt_pwm: int) -> tuple[float, float]:
    pan_fraction = (pan_pwm - 1000) / 1000.0
    tilt_fraction = (tilt_pwm - 1000) / 1000.0
    pan = PAN_MIN_RAD + pan_fraction * (PAN_MAX_RAD - PAN_MIN_RAD)
    tilt = TILT_MAX_RAD - tilt_fraction * (TILT_MAX_RAD - TILT_MIN_RAD)
    return pan, tilt


def public_angles_from_pwm(pan_pwm: int, tilt_pwm: int) -> tuple[float, float]:
    """Apply the tower contract: pan and positive-up tilt, both in degrees."""
    return (
        0.36 * (pan_pwm - 1500),
        -30.0 + 0.075 * (tilt_pwm - 1000),
    )


def wait_for_tower_aim(
    *,
    bridge: "GazeboBridge",
    tower_name: str,
    pan_pwm: int,
    tilt_pwm: int,
    timeout_s: float,
    tolerance_deg: float,
) -> tuple[float, float]:
    """Wait until Gazebo's physical joints reach the requested PWM angles."""
    if timeout_s <= 0 or tolerance_deg <= 0:
        raise ValueError("joint timeout and tolerance must be positive")
    expected_pan, expected_tilt = public_angles_from_pwm(pan_pwm, tilt_pwm)
    deadline = time.monotonic() + timeout_s
    observations: list[str] = []
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        try:
            actual_pan, actual_tilt = bridge.tower_joint_angles(
                tower_name,
                timeout_s=min(4.0, remaining),
            )
        except Exception as exc:
            observations.append(f"{type(exc).__name__}: {exc}")
        else:
            pan_error = abs(
                math.degrees(
                    wrap_pi(math.radians(actual_pan - expected_pan))
                )
            )
            tilt_error = abs(actual_tilt - expected_tilt)
            observations.append(
                f"observed pan={actual_pan:.2f}, tilt={actual_tilt:.2f} deg"
            )
            if pan_error <= tolerance_deg and tilt_error <= tolerance_deg:
                return actual_pan, actual_tilt
        time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))
    detail = observations[-1] if observations else "no Gazebo observation"
    raise RuntimeError(
        f"{tower_name} accepted its servo commands but its Gazebo joints did not "
        f"reach pan={expected_pan:.2f}, tilt={expected_tilt:.2f} deg within "
        f"{timeout_s:g} s ({detail})"
    )


def command_and_verify_tower(
    *,
    controller: Any,
    bridge: "GazeboBridge",
    tower_name: str,
    pan_pwm: int,
    tilt_pwm: int,
    ack_timeout_s: float,
    joint_timeout_s: float,
    tolerance_deg: float,
) -> tuple[float, float]:
    """Require MAVLink acceptance followed by physical Gazebo joint motion."""
    controller.pan(pan_pwm, timeout_s=ack_timeout_s)
    controller.tilt(tilt_pwm, timeout_s=ack_timeout_s)
    return wait_for_tower_aim(
        bridge=bridge,
        tower_name=tower_name,
        pan_pwm=pan_pwm,
        tilt_pwm=tilt_pwm,
        timeout_s=joint_timeout_s,
        tolerance_deg=tolerance_deg,
    )


def _rotate_local(x: float, y: float, yaw: float) -> tuple[float, float]:
    return x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw)


def projected_boat_box(
    *,
    width: int,
    height: int,
    hfov_deg: float,
    vfov_deg: float,
    tower: ModelPose,
    target: PoseSpec,
    camera_pan_rad: float,
    camera_tilt_rad: float,
) -> tuple[float, float, float, float] | None:
    """Approximate the known vessel's visible 3-D extent in image pixels."""
    heading = tower.yaw_rad + camera_pan_rad
    forward = (
        math.cos(camera_tilt_rad) * math.cos(heading),
        math.cos(camera_tilt_rad) * math.sin(heading),
        -math.sin(camera_tilt_rad),
    )
    right = (math.sin(heading), -math.cos(heading), 0.0)
    up = (
        math.sin(camera_tilt_rad) * math.cos(heading),
        math.sin(camera_tilt_rad) * math.sin(heading),
        math.cos(camera_tilt_rad),
    )
    camera = (tower.x, tower.y, tower.z + CAMERA_HEIGHT_M)
    fx = width / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))
    fy = height / (2.0 * math.tan(math.radians(vfov_deg) / 2.0))
    points: list[tuple[float, float]] = []
    yaw = math.radians(target.yaw_deg)
    for lx in (-BOAT_LENGTH_M / 2.0, BOAT_LENGTH_M / 2.0):
        for ly in (-BOAT_WIDTH_M / 2.0, BOAT_WIDTH_M / 2.0):
            rx, ry = _rotate_local(lx, ly, yaw)
            for z in (0.0, BOAT_VISIBLE_HEIGHT_M):
                relative = (
                    target.x + rx - camera[0],
                    target.y + ry - camera[1],
                    z - camera[2],
                )
                depth = sum(a * b for a, b in zip(relative, forward, strict=True))
                if depth <= 0.1:
                    continue
                px = width / 2.0 + fx * sum(
                    a * b for a, b in zip(relative, right, strict=True)
                ) / depth
                py = height / 2.0 - fy * sum(
                    a * b for a, b in zip(relative, up, strict=True)
                ) / depth
                points.append((px, py))
    if not points:
        return None
    x1 = max(0.0, min(point[0] for point in points))
    y1 = max(0.0, min(point[1] for point in points))
    x2 = min(float(width), max(point[0] for point in points))
    y2 = min(float(height), max(point[1] for point in points))
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def yolo_line(box: tuple[float, float, float, float], width: int, height: int) -> str:
    x1, y1, x2, y2 = box
    return (
        f"0 {(x1 + x2) / 2.0 / width:.6f} {(y1 + y2) / 2.0 / height:.6f} "
        f"{(x2 - x1) / width:.6f} {(y2 - y1) / height:.6f}"
    )


class GazeboBridge:
    def __init__(self, host: str, *, timeout_s: float = 10.0, connector: Callable[..., Any] | None = None):
        if connector is None:
            from websockets.sync.client import connect

            connector = lambda *args, **kwargs: connect(*args, **kwargs, legacy=True)
        self.host = host
        self.timeout_s = timeout_s
        self._connector = connector
        self._socket = self._connect()

    def _connect(self):
        return self._connector(
            f"ws://{self.host}:8080",
            open_timeout=self.timeout_s,
            max_size=None,
            # gzweb doesn't consistently complete WebSocket keepalive or close
            # handshakes while Gazebo is busy. Application-level timeouts and
            # reconnects below provide the liveness guarantee we need.
            ping_interval=None,
            close_timeout=1.0,
        )

    def close(self) -> None:
        with suppress(Exception):
            self._socket.close()

    def reconnect(self) -> None:
        self.close()
        self._socket = self._connect()

    def __enter__(self) -> "GazeboBridge":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def publish(self, topic: str, message: dict[str, Any]) -> None:
        self._socket.send(json.dumps({"op": "publish", "topic": topic, "msg": message}))

    def _scene_message(
        self,
        *,
        attempts: int,
        timeout_s: float | None = None,
        reconnect_first: bool = False,
    ) -> dict[str, Any]:
        """Fetch one raw scene snapshot, reconnecting while gzweb becomes ready."""
        failures: list[str] = []
        for attempt in range(1, attempts + 1):
            if reconnect_first or attempt > 1:
                self.reconnect()
            try:
                # This gzweb build may also push an initial scene snapshot on
                # connect. Sending the request supports builds that don't.
                self._socket.send(
                    json.dumps(
                        {"op": "subscribe", "topic": "~/scene", "type": "scene"}
                    )
                )
                deadline = time.monotonic() + (
                    self.timeout_s if timeout_s is None else timeout_s
                )
                while time.monotonic() < deadline:
                    try:
                        raw = self._socket.recv(
                            timeout=min(1.0, deadline - time.monotonic())
                        )
                    except TimeoutError:
                        continue
                    message = json.loads(raw)
                    if (
                        isinstance(message, dict)
                        and message.get("topic") == "~/scene"
                    ):
                        return message
                raise TimeoutError("gzweb did not return ~/scene")
            except Exception as exc:
                failures.append(f"attempt {attempt}: {type(exc).__name__}: {exc}")
                if attempt < attempts:
                    print(
                        f"gzweb scene unavailable; reconnecting "
                        f"({attempt}/{attempts})",
                        flush=True,
                    )
        raise RuntimeError(
            f"Gazebo scene unavailable after {attempts} attempts: "
            + " | ".join(failures)
        )

    def scene(self, *, attempts: int = 3) -> dict[str, ModelPose]:
        """Fetch model poses, reconnecting while gzweb becomes ready."""
        return models_from_scene(self._scene_message(attempts=attempts))

    def tower_joint_angles(
        self,
        tower_name: str,
        *,
        timeout_s: float = 4.0,
    ) -> tuple[float, float]:
        """Read live physical tower angles from a fresh Gazebo scene snapshot."""
        message = self._scene_message(
            attempts=1,
            timeout_s=timeout_s,
            reconnect_first=True,
        )
        return tower_joint_angles_from_scene(message, tower_name)

    @staticmethod
    def _at_pose(model: ModelPose, pose: PoseSpec) -> bool:
        return (
            math.hypot(model.x - pose.x, model.y - pose.y) <= 0.5
            and abs(model.z) <= 0.5
            and abs(wrap_pi(model.yaw_rad - math.radians(pose.yaw_deg)))
            <= math.radians(1.0)
        )

    def wait_for_pose_update(
        self,
        name: str,
        *,
        pose: PoseSpec,
        topics: tuple[str, ...],
    ) -> ModelPose:
        """Wait for Gazebo's direct acknowledgement of a create or move."""
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            try:
                raw = self._socket.recv(timeout=min(1.0, deadline - time.monotonic()))
            except TimeoutError:
                continue
            message = json.loads(raw)
            if not isinstance(message, dict) or message.get("topic") not in topics:
                continue
            body = message.get("msg")
            if not isinstance(body, dict) or body.get("name") != name:
                continue
            model = parse_model(body)
            if self._at_pose(model, pose):
                return model
        raise RuntimeError(
            f"Gazebo did not confirm {name!r} at ({pose.x:.2f}, {pose.y:.2f}, "
            f"yaw {pose.yaw_deg:.1f} deg) on {', '.join(topics)} within "
            f"{self.timeout_s:g} s"
        )

    def replace_vessel(
        self,
        pose: PoseSpec,
        *,
        delete_name: str,
        delete_existing: bool,
        model_name: str,
        model_type: str,
        factory_delay_s: float = 0.75,
    ) -> ModelPose:
        # Use a unique training model name. Deletion is asynchronous, but it
        # cannot race factory creation when the two entities have different
        # names. This also works if an earlier failed run already removed the
        # path-controlled target.
        if delete_existing:
            self.publish("~/entity_delete", {"name": delete_name})
            time.sleep(0.25)
        self.publish(
            "~/factory",
            {
                "name": model_name,
                "type": model_type,
                "createEntity": 1,
                "position": {"x": pose.x, "y": pose.y, "z": 0.0},
                "orientation": yaw_quaternion(math.radians(pose.yaw_deg)),
            },
        )
        time.sleep(factory_delay_s)
        return self.wait_for_pose_update(
            model_name,
            pose=pose,
            topics=("~/model/info", "~/pose/info"),
        )

    def move_vessel(
        self,
        pose: PoseSpec,
        *,
        model: ModelPose,
        attempts: int = 3,
    ) -> ModelPose:
        """Move the training model, reconnecting and resending on a lost ack."""
        message = {
            "name": model.name,
            "id": model.model_id,
            "createEntity": 0,
            "position": {"x": pose.x, "y": pose.y, "z": 0.0},
            "orientation": yaw_quaternion(math.radians(pose.yaw_deg)),
        }
        failures: list[str] = []
        for attempt in range(1, attempts + 1):
            try:
                if attempt > 1:
                    self.reconnect()
                    try:
                        current = self.scene().get(model.name)
                    except Exception as exc:
                        failures.append(
                            f"attempt {attempt} verification: "
                            f"{type(exc).__name__}: {exc}"
                        )
                        # scene() may leave this connection unusable after a
                        # timeout; publish the retry on a clean socket.
                        self.reconnect()
                    else:
                        if current is not None and self._at_pose(current, pose):
                            return current
                self.publish("~/model/modify", message)
                time.sleep(0.1)
                return self.wait_for_pose_update(
                    model.name,
                    pose=pose,
                    topics=("~/pose/info", "~/model/info"),
                )
            except Exception as exc:
                failures.append(f"attempt {attempt}: {type(exc).__name__}: {exc}")
                if attempt < attempts:
                    print(
                        f"gzweb did not confirm pose {pose.name!r}; "
                        f"reconnecting and retrying ({attempt}/{attempts})",
                        flush=True,
                    )
        raise RuntimeError(
            f"Gazebo move failed after {attempts} attempts for {model.name!r}: "
            + " | ".join(failures)
        )


class LatestFrameCamera:
    def __init__(self, name: str, url: str) -> None:
        self.name = name
        self.url = url
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._sequence = 0
        self._latest: CameraFrame | None = None
        self._error: str | None = None
        self._thread = threading.Thread(target=self._run, name=f"camera-{name}", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=0.2)

    @property
    def sequence(self) -> int:
        with self._condition:
            return self._sequence

    def snapshot(self) -> tuple[int, CameraFrame | None]:
        with self._condition:
            return self._sequence, self._latest

    def _run(self) -> None:
        camera = MjpegCamera(self.name, self.url, timeout_s=5.0)
        try:
            for frame in camera.frames():
                with self._condition:
                    self._sequence += 1
                    self._latest = frame
                    self._condition.notify_all()
                if self._stop.is_set():
                    return
        except Exception as exc:  # surfaced to the waiting main thread
            with self._condition:
                self._error = f"{type(exc).__name__}: {exc}"
                self._condition.notify_all()

    def wait_after(
        self,
        sequence: int,
        *,
        timeout_s: float,
        different_from: bytes | None = None,
    ) -> tuple[int, CameraFrame]:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._sequence <= sequence or (
                different_from is not None
                and self._latest is not None
                and self._latest.jpeg == different_from
            ):
                if self._error:
                    raise RuntimeError(f"{self.name} camera failed: {self._error}")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    if self._sequence > sequence and different_from is not None:
                        raise TimeoutError(
                            f"{self.name} kept repeating the previous JPEG for "
                            f"{timeout_s:g} s"
                        )
                    raise TimeoutError(
                        f"no new frame from {self.name} within {timeout_s:g} s"
                    )
                self._condition.wait(remaining)
            assert self._latest is not None
            return self._sequence, self._latest


def _decode_frame(jpeg: bytes):
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("frame validation requires whiteout[camera]") from exc
    image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("camera returned an invalid JPEG")
    return image


def red_hull_pixels(image: Any) -> int:
    """Count saturated red pixels using the same cue as the pre-labeler."""
    import cv2

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, saturation, value = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    red = ((hue <= 8) | (hue >= 170)) & (saturation > 90) & (value > 60)
    return int(red.sum())


def calibrated_frame(
    *,
    controller: Any,
    sampler: LatestFrameCamera,
    nominal_pan_pwm: int,
    nominal_tilt_pwm: int,
    view_offset_deg: float,
    settle_s: float,
    ack_timeout_s: float,
    frame_timeout_s: float,
    min_red_pixels: int,
    diagnostics_dir: Path,
    different_from_jpeg: bytes | None = None,
) -> tuple[CameraFrame, int, int, int]:
    """Find the hull near the computed aim, then apply the requested offset."""
    attempts: list[tuple[int, int, int, CameraFrame]] = []

    def candidate(pan_pwm: int, tilt_pwm: int) -> tuple[int, CameraFrame]:
        pan_pwm = min(2000, max(1000, pan_pwm))
        tilt_pwm = min(2000, max(1000, tilt_pwm))
        controller.pan(pan_pwm, timeout_s=ack_timeout_s)
        controller.tilt(tilt_pwm, timeout_s=ack_timeout_s)
        time.sleep(settle_s)
        checkpoint = sampler.sequence
        _, frame = sampler.wait_after(
            checkpoint,
            timeout_s=frame_timeout_s,
            different_from=different_from_jpeg,
        )
        score = red_hull_pixels(_decode_frame(frame.jpeg))
        attempts.append((pan_pwm, tilt_pwm, score, frame))
        return score, frame

    def save_diagnostics() -> int:
        diagnostics_dir.mkdir(parents=True, exist_ok=True)
        hashes: set[str] = set()
        for attempt, (pan_pwm, tilt_pwm, score, frame) in enumerate(attempts, start=1):
            digest = hashlib.sha256(frame.jpeg).hexdigest()
            hashes.add(digest)
            path = diagnostics_dir / (
                f"{attempt:02d}-pan-{pan_pwm}-tilt-{tilt_pwm}-red-{score}.jpg"
            )
            path.write_bytes(frame.jpeg)
        return len(hashes)

    tilt_candidates = [
        min(2000, max(1000, nominal_tilt_pwm + offset))
        for offset in (0, -160, 160)
    ]
    scored_tilts = [
        (*candidate(nominal_pan_pwm, tilt_pwm), tilt_pwm)
        for tilt_pwm in dict.fromkeys(tilt_candidates)
    ]
    _, _, best_tilt_pwm = max(scored_tilts, key=lambda item: item[0])

    pan_candidates = [
        min(2000, max(1000, nominal_pan_pwm + offset))
        for offset in (0, -50, 50)
    ]
    scored_pans = [
        (*candidate(pan_pwm, best_tilt_pwm), pan_pwm)
        for pan_pwm in dict.fromkeys(pan_candidates)
    ]
    best_score, _, best_pan_pwm = max(scored_pans, key=lambda item: item[0])
    if best_score < min_red_pixels:
        unique_frames = save_diagnostics()
        raise RuntimeError(
            f"boat hull was not visible near the computed aim "
            f"(best red-pixel score {best_score}, required {min_red_pixels}; "
            f"saved {len(attempts)} diagnostic frames with {unique_frames} unique "
            f"image(s) in {diagnostics_dir})"
        )

    final_pan_pwm = round(best_pan_pwm + view_offset_deg / 360.0 * 1000.0)
    final_pan_pwm = min(2000, max(1000, final_pan_pwm))
    final_score, final_frame = candidate(final_pan_pwm, best_tilt_pwm)
    if final_score < min_red_pixels:
        save_diagnostics()
        raise RuntimeError(
            f"boat hull left the image at view offset {view_offset_deg:g} deg "
            f"(red-pixel score {final_score}, required {min_red_pixels})"
        )
    return final_frame, final_pan_pwm, best_tilt_pwm, final_score


def camera_definition(name: str) -> tuple[int, float, float]:
    definitions = {
        "tower-1": (8630, 60.0, 36.1),
        "tower-2": (8640, 60.0, 36.1),
    }
    try:
        return definitions[name]
    except KeyError as exc:
        raise ValueError("camera must be tower-1 or tower-2") from exc


def start_tower_controller(
    tower: Tower,
    *,
    attempts: int,
    connection_timeout_s: float = 20.0,
    retry_delay_s: float = 3.0,
):
    """Wait through the tower SITL's post-restart heartbeat delay."""
    if attempts < 1:
        raise ValueError("tower startup attempts must be positive")
    last_error: MavProxyConnectionError | None = None
    for attempt in range(1, attempts + 1):
        controller = tower.controller(connection_timeout_s=connection_timeout_s)
        try:
            return controller.start()
        except MavProxyConnectionError as exc:
            last_error = exc
            controller.close()
            if attempt < attempts:
                print(
                    f"{tower.name} has no heartbeat yet; retrying "
                    f"({attempt}/{attempts})",
                    flush=True,
                )
                time.sleep(retry_delay_s)
    assert last_error is not None
    raise MavProxyConnectionError(
        f"{tower.name} stayed offline after {attempts} startup attempts. "
        f"Last attempt: {last_error}"
    ) from last_error


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="10.99.0.1")
    parser.add_argument("--camera", choices=("tower-1", "tower-2"), default="tower-1")
    parser.add_argument("--samples", type=int, default=4)
    parser.add_argument("--poses", type=Path, help="explicit world-pose CSV")
    parser.add_argument("--output", type=Path, default=Path("recordings/horizontal"))
    parser.add_argument("--model-name", default="target_vessel")
    parser.add_argument(
        "--training-model-name",
        help="stationary model name (default: a unique name for this run)",
    )
    parser.add_argument("--model-type", default="fishing_vessel")
    parser.add_argument("--settle-seconds", type=float, default=1.5)
    parser.add_argument("--factory-delay", type=float, default=0.75)
    parser.add_argument("--calibration-settle", type=float, default=0.35)
    parser.add_argument("--frame-timeout", type=float, default=15.0)
    parser.add_argument(
        "--servo-ack-timeout",
        type=float,
        default=3.0,
        help="seconds to wait for each MAVLink servo acknowledgement (default: 3)",
    )
    parser.add_argument(
        "--joint-timeout",
        type=float,
        default=12.0,
        help="seconds to verify the physical Gazebo pan/tilt joints (default: 12)",
    )
    parser.add_argument(
        "--joint-tolerance-deg",
        type=float,
        default=2.0,
        help="allowed physical pan/tilt error in degrees (default: 2)",
    )
    parser.add_argument(
        "--startup-attempts",
        type=int,
        default=12,
        help="gzweb scene attempts while the simulator starts (default: 12)",
    )
    parser.add_argument(
        "--tower-startup-attempts",
        type=int,
        default=6,
        help="20-second MAVLink heartbeat attempts for the tower (default: 6)",
    )
    parser.add_argument("--min-red-pixels", type=int, default=4)
    parser.add_argument("--take-over-target", action="store_true")
    parser.add_argument("--confirm-network", action="store_true")
    parser.add_argument("--confirm-simulator-control", action="store_true")
    args = parser.parse_args()
    if not args.confirm_network:
        parser.error("camera and gzweb access requires --confirm-network")
    if not args.confirm_simulator_control:
        parser.error("runtime vessel and tower control requires --confirm-simulator-control")
    if not args.take_over_target:
        parser.error(
            "the live VesselPathPlugin overrides pose changes; pass --take-over-target "
            "to replace only target_vessel with a runtime-controlled training copy"
        )
    if args.samples < 1:
        parser.error("--samples must be positive")
    if (
        args.settle_seconds < 0
        or args.factory_delay < 0
        or args.calibration_settle < 0
        or args.frame_timeout <= 0
        or args.servo_ack_timeout <= 0
        or args.joint_timeout <= 0
        or args.joint_tolerance_deg <= 0
        or args.min_red_pixels < 1
        or args.startup_attempts < 1
        or args.tower_startup_attempts < 1
    ):
        parser.error(
            "settle/factory delays must be non-negative; timeouts, pixel count, "
            "joint tolerance, and startup attempts must be positive"
        )

    port, hfov_deg, vfov_deg = camera_definition(args.camera)
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output / run_id
    training_model_name = args.training_model_name or f"horizontal_training_vessel_{run_id}"
    images = output / "images"
    labels = output / "labels"
    metadata_path = output / "metadata.jsonl"

    sampler = LatestFrameCamera(args.camera, f"http://{args.host}:{port}/stream")
    sampler.start()
    controller = None
    bridge = None
    try:
        bridge = GazeboBridge(args.host)
        # The control service explicitly warns that gzweb may need 1-2 minutes
        # after a simulator restart. Retry the scene handshake without asking
        # the operator to restart or rerun this collector.
        scene = bridge.scene(attempts=args.startup_attempts)
        target = scene.get(args.model_name)
        tower_pose = scene.get(args.camera)
        stale_training_models = sorted(
            name for name in scene if name.startswith("horizontal_training_vessel_")
        )
        if target is None and args.poses is None:
            raise RuntimeError(f"{args.model_name!r} is absent from the active Gazebo scene")
        if tower_pose is None:
            raise RuntimeError(f"{args.camera!r} is absent from the active Gazebo scene")
        poses = load_poses(args.poses) if args.poses else automatic_poses(target, args.samples)  # type: ignore[arg-type]

        tower = Tower.one(args.host) if args.camera == "tower-1" else Tower.two(args.host)
        controller = start_tower_controller(
            tower,
            attempts=args.tower_startup_attempts,
        )
        # Prove the camera path before replacing the course-controlled target.
        initial_sequence, initial_frame = sampler.wait_after(
            0, timeout_s=args.frame_timeout
        )

        # Fail before deleting the path-controlled vessel if MAVLink accepts a
        # command but the physical tower joints or rendered camera do not move.
        # The probe is small, reversible, and uses the same path as collection.
        try:
            command_and_verify_tower(
                controller=controller,
                bridge=bridge,
                tower_name=args.camera,
                pan_pwm=1600,
                tilt_pwm=1500,
                ack_timeout_s=args.servo_ack_timeout,
                joint_timeout_s=args.joint_timeout,
                tolerance_deg=args.joint_tolerance_deg,
            )
            sampler.wait_after(
                initial_sequence,
                timeout_s=args.frame_timeout,
                different_from=initial_frame.jpeg,
            )
        finally:
            command_and_verify_tower(
                controller=controller,
                bridge=bridge,
                tower_name=args.camera,
                pan_pwm=1500,
                tilt_pwm=1500,
                ack_timeout_s=args.servo_ack_timeout,
                joint_timeout_s=args.joint_timeout,
                tolerance_deg=args.joint_tolerance_deg,
            )
        print(f"{args.camera} pan/tilt command and camera motion verified", flush=True)

        # Don't leave an empty timestamped run when a live dependency cannot
        # be reached. Once control is established, preserve partial output for
        # diagnosis if a later pose or frame fails.
        images.mkdir(parents=True, exist_ok=False)
        labels.mkdir(parents=True, exist_ok=False)
        for stale_name in stale_training_models:
            bridge.publish("~/entity_delete", {"name": stale_name})
        if stale_training_models:
            print(
                f"removed {len(stale_training_models)} stale training vessel(s)",
                flush=True,
            )
            time.sleep(0.25)
        previous_jpeg: bytes | None = initial_frame.jpeg
        with metadata_path.open("w", encoding="utf-8") as metadata:
            for index, pose in enumerate(poses, start=1):
                if index == 1:
                    spawned = bridge.replace_vessel(
                        pose,
                        delete_name=args.model_name,
                        delete_existing=target is not None,
                        model_name=training_model_name,
                        model_type=args.model_type,
                        factory_delay_s=args.factory_delay,
                    )
                else:
                    spawned = bridge.move_vessel(pose, model=spawned)
                centered_pose = dataclass_replace(pose, view_offset_deg=0.0)
                nominal_pan_pwm, nominal_tilt_pwm, _, _ = aim_pwm(tower_pose, centered_pose)
                command_and_verify_tower(
                    controller=controller,
                    bridge=bridge,
                    tower_name=args.camera,
                    pan_pwm=nominal_pan_pwm,
                    tilt_pwm=nominal_tilt_pwm,
                    ack_timeout_s=args.servo_ack_timeout,
                    joint_timeout_s=args.joint_timeout,
                    tolerance_deg=args.joint_tolerance_deg,
                )
                time.sleep(args.settle_seconds)
                frame, pan_pwm, tilt_pwm, red_pixels = calibrated_frame(
                    controller=controller,
                    sampler=sampler,
                    nominal_pan_pwm=nominal_pan_pwm,
                    nominal_tilt_pwm=nominal_tilt_pwm,
                    view_offset_deg=pose.view_offset_deg,
                    settle_s=args.calibration_settle,
                    ack_timeout_s=args.servo_ack_timeout,
                    frame_timeout_s=args.frame_timeout,
                    min_red_pixels=args.min_red_pixels,
                    diagnostics_dir=output
                    / "diagnostics"
                    / f"{index:03d}-{pose.name}",
                    different_from_jpeg=previous_jpeg,
                )
                joint_pan_deg, joint_tilt_up_deg = wait_for_tower_aim(
                    bridge=bridge,
                    tower_name=args.camera,
                    pan_pwm=pan_pwm,
                    tilt_pwm=tilt_pwm,
                    timeout_s=args.joint_timeout,
                    tolerance_deg=args.joint_tolerance_deg,
                )
                image = _decode_frame(frame.jpeg)
                height, width = image.shape[:2]
                pan_rad, tilt_rad = angles_from_pwm(pan_pwm, tilt_pwm)
                stem = f"{index:03d}-{pose.name}"
                image_path = images / f"{stem}.jpg"
                image_path.write_bytes(frame.jpeg)
                previous_jpeg = frame.jpeg

                box = projected_boat_box(
                    width=width,
                    height=height,
                    hfov_deg=hfov_deg,
                    vfov_deg=vfov_deg,
                    tower=tower_pose,
                    target=pose,
                    camera_pan_rad=pan_rad,
                    camera_tilt_rad=tilt_rad,
                )
                label_path = labels / f"{stem}.txt"
                label_path.write_text(
                    yolo_line(box, width, height) + "\n" if box else "",
                    encoding="utf-8",
                )
                record = {
                    "image": str(image_path.relative_to(output)),
                    "label": str(label_path.relative_to(output)),
                    "camera": args.camera,
                    "captured_at": frame.timestamp.isoformat(),
                    "width": width,
                    "height": height,
                    "tower": asdict(tower_pose),
                    "requested_pose": asdict(pose),
                    "spawned_pose": asdict(spawned),
                    "pan_pwm": pan_pwm,
                    "tilt_pwm": tilt_pwm,
                    "pan_deg": round(math.degrees(pan_rad), 3),
                    "tilt_down_deg": round(math.degrees(tilt_rad), 3),
                    "verified_joint_pan_deg": round(joint_pan_deg, 3),
                    "verified_joint_tilt_up_deg": round(joint_tilt_up_deg, 3),
                    "red_hull_pixels": red_pixels,
                    "projected_prelabel_xyxy": [round(value, 2) for value in box] if box else None,
                    "prelabel_requires_review": True,
                }
                metadata.write(json.dumps(record) + "\n")
                metadata.flush()
                print(f"[{index}/{len(poses)}] {image_path}  pan={pan_pwm} tilt={tilt_pwm}")

        (output / "data.yaml").write_text(
            f"path: {output.resolve()}\ntrain: images\nval: images\nnames:\n  0: boat\n",
            encoding="utf-8",
        )
        print(f"collected {len(poses)} direct simulator frame(s) in {output}")
        print(f"{training_model_name} remains at the final pose until the next Reset")
        return 0
    finally:
        if controller is not None:
            controller.close()
        sampler.close()
        if bridge is not None:
            bridge.close()


if __name__ == "__main__":
    raise SystemExit(main())
