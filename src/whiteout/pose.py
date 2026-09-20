"""Calibrated simulator camera poses in the local north/east/down frame."""

from __future__ import annotations

import math

from .geolocation import CameraPose
from .models import Telemetry
from .tower import TowerOrientation, TowerWorldPose


QUADCOPTER_CAMERA_DOWN_DEG = 20.0
FIXED_WING_CAMERA_DOWN_DEG = 8.0

Matrix3 = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]


def aircraft_camera_pose(
    telemetry: Telemetry,
    *,
    camera_down_deg: float,
    position_uncertainty_m: float = 1.0,
    attitude_uncertainty_rad: float = math.radians(1.0),
) -> CameraPose:
    """Compose aircraft attitude with a forward, downward-pitched camera."""

    if not telemetry.has_attitude:
        raise ValueError("aircraft camera pose requires synchronized attitude")
    if not 0 < camera_down_deg <= 90:
        raise ValueError("camera downward elevation must be in (0, 90] degrees")
    assert telemetry.roll_rad is not None
    assert telemetry.pitch_rad is not None
    assert telemetry.yaw_rad is not None
    body_to_ned = _rpy_matrix(
        telemetry.roll_rad,
        telemetry.pitch_rad,
        telemetry.yaw_rad,
    )
    camera_to_body = _forward_camera_matrix(camera_down_deg)
    return CameraPose(
        telemetry.latitude,
        telemetry.longitude,
        telemetry.altitude_m,
        position_uncertainty_m=position_uncertainty_m,
        attitude_uncertainty_rad=attitude_uncertainty_rad,
        camera_to_ned=_matmul(body_to_ned, camera_to_body),
    )


def tower_camera_pose(
    tower: TowerWorldPose,
    orientation: TowerOrientation,
    *,
    position_uncertainty_m: float = 0.25,
    attitude_uncertainty_rad: float = math.radians(1.0),
) -> CameraPose:
    """Build a fixed-tower pose from settled, healthy pan/tilt orientation."""

    if not orientation.settled:
        raise ValueError("tower camera pose requires a settled orientation")
    if not orientation.healthy:
        raise ValueError("tower camera pose requires a healthy orientation")
    # Tower joint convention is positive down. Camera local z is boresight,
    # local x is image-up, and local y is image-right.
    camera_to_ned = _heading_elevation_matrix(
        orientation.true_heading_deg,
        orientation.tilt_deg,
    )
    return CameraPose(
        tower.latitude,
        tower.longitude,
        tower.camera_world_z_m,
        position_uncertainty_m=position_uncertainty_m,
        attitude_uncertainty_rad=attitude_uncertainty_rad,
        camera_to_ned=camera_to_ned,
    )


def _forward_camera_matrix(down_deg: float) -> Matrix3:
    elevation = math.radians(down_deg)
    cosine, sine = math.cos(elevation), math.sin(elevation)
    # Columns are camera image-up, image-right, and boresight in body NED.
    return (
        (sine, 0.0, cosine),
        (0.0, 1.0, 0.0),
        (-cosine, 0.0, sine),
    )


def _heading_elevation_matrix(heading_deg: float, down_deg: float) -> Matrix3:
    heading = math.radians(heading_deg)
    elevation = math.radians(down_deg)
    ch, sh = math.cos(heading), math.sin(heading)
    ce, se = math.cos(elevation), math.sin(elevation)
    return (
        (se * ch, -sh, ce * ch),
        (se * sh, ch, ce * sh),
        (-ce, 0.0, se),
    )


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> Matrix3:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy),
        (cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy),
        (-sp, sr * cp, cr * cp),
    )


def _matmul(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(
            sum(left[row][axis] * right[axis][column] for axis in range(3))
            for column in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]
