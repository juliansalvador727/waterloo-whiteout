"""Pinhole pixel-ray intersection with a locally flat water surface."""

from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime, timezone

from .models import GeoEstimate, Pixel


@dataclass(frozen=True, slots=True)
class CameraIntrinsics:
    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float
    pixel_uncertainty_px: float = 2.0


@dataclass(frozen=True, slots=True)
class CameraPose:
    latitude: float
    longitude: float
    altitude_m: float
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    position_uncertainty_m: float = 1.0
    attitude_uncertainty_rad: float = math.radians(1.0)


def _rotate_ned(vector: tuple[float, float, float], roll: float, pitch: float, yaw: float) -> tuple[float, float, float]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    x, y, z = vector
    return (
        (cp * cy) * x + (sr * sp * cy - cr * sy) * y + (cr * sp * cy + sr * sy) * z,
        (cp * sy) * x + (sr * sp * sy + cr * cy) * y + (cr * sp * sy - sr * cy) * z,
        (-sp) * x + (sr * cp) * y + (cr * cp) * z,
    )


def estimate_flat_water(pixel: Pixel, intrinsics: CameraIntrinsics, pose: CameraPose) -> GeoEstimate:
    if intrinsics.fx_px <= 0 or intrinsics.fy_px <= 0:
        raise ValueError("focal lengths must be positive")
    if pose.altitude_m <= 0:
        raise ValueError("camera must be above the water surface")

    optical_x = (pixel.x - intrinsics.cx_px) / intrinsics.fx_px
    optical_y = (pixel.y - intrinsics.cy_px) / intrinsics.fy_px
    # At zero attitude the optical axis points down: image up is north, right is east.
    ray_ned = _rotate_ned((-optical_y, optical_x, 1.0), pose.roll_rad, pose.pitch_rad, pose.yaw_rad)
    if ray_ned[2] <= 1e-6:
        raise ValueError("pixel ray does not intersect the water in front of the camera")

    scale = pose.altitude_m / ray_ned[2]
    north_m, east_m = ray_ned[0] * scale, ray_ned[1] * scale
    earth_radius_m = 6_378_137.0
    latitude = pose.latitude + math.degrees(north_m / earth_radius_m)
    cos_latitude = math.cos(math.radians(pose.latitude))
    if abs(cos_latitude) < 1e-9:
        raise ValueError("longitude is undefined near the poles")
    longitude = pose.longitude + math.degrees(east_m / (earth_radius_m * cos_latitude))

    range_m = math.sqrt(north_m * north_m + east_m * east_m + pose.altitude_m**2)
    pixel_angle = max(
        intrinsics.pixel_uncertainty_px / intrinsics.fx_px,
        intrinsics.pixel_uncertainty_px / intrinsics.fy_px,
    )
    uncertainty = math.sqrt(
        pose.position_uncertainty_m**2
        + (range_m * pose.attitude_uncertainty_rad) ** 2
        + (range_m * pixel_angle) ** 2
    )
    return GeoEstimate(latitude, longitude, uncertainty, datetime.now(timezone.utc))

