"""Fort Ross tower calibration and servo-derived orientation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime, timezone


FORT_ROSS_BASE_TRUE_HEADING_DEG = 40.1952
TOWER_SCAN_HORIZONTAL_FOV_DEG = 60.0
TOWER_SCAN_VERTICAL_FOV_DEG = 36.1
MAX_TOWER_SCAN_HORIZONTAL_STEP_DEG = 48.0
MAX_TOWER_SCAN_VERTICAL_STEP_DEG = 28.88
TOWER_SETTLED_RATE_DPS = 1.0
TOWER_SETTLED_DURATION_S = 0.5


@dataclass(frozen=True, slots=True)
class TowerWorldPose:
    name: str
    latitude: float
    longitude: float
    camera_world_z_m: float


FORT_ROSS_TOWERS = (
    TowerWorldPose("tower-1", 72.000588, -94.814426, 44.374),
    TowerWorldPose("tower-2", 72.011778, -94.804721, 229.254),
)


def wrap_heading(angle_deg: float) -> float:
    return float(angle_deg) % 360.0


def wrap_pan(angle_deg: float) -> float:
    """Wrap an angular difference into [-180, 180)."""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def validate_tower_scan_steps(
    horizontal_step_deg: float, vertical_step_deg: float
) -> tuple[float, float]:
    """Validate scan increments that retain at least 20 percent image overlap."""
    horizontal = float(horizontal_step_deg)
    vertical = float(vertical_step_deg)
    if not 0 < horizontal <= MAX_TOWER_SCAN_HORIZONTAL_STEP_DEG:
        raise ValueError("tower horizontal scan step must be in (0, 48] degrees")
    if not 0 < vertical <= MAX_TOWER_SCAN_VERTICAL_STEP_DEG:
        raise ValueError("tower vertical scan step must be in (0, 28.88] degrees")
    return horizontal, vertical


def tower_scan_overlap_steps(
    horizontal_overlap: float = 0.20, vertical_overlap: float = 0.20
) -> tuple[float, float]:
    """Return scan steps while enforcing the tower's minimum 20 percent overlap."""
    if not 0 <= horizontal_overlap < 1 or not 0 <= vertical_overlap < 1:
        raise ValueError("tower scan overlap must be in the range [0, 1)")
    return validate_tower_scan_steps(
        round(TOWER_SCAN_HORIZONTAL_FOV_DEG * (1.0 - horizontal_overlap), 12),
        round(TOWER_SCAN_VERTICAL_FOV_DEG * (1.0 - vertical_overlap), 12),
    )


tower_scan_spacing_deg = tower_scan_overlap_steps


@dataclass(frozen=True, slots=True)
class TowerCalibration:
    base_true_heading_deg: float = FORT_ROSS_BASE_TRUE_HEADING_DEG
    pan_min_deg: float = -144.0
    pan_max_deg: float = 144.0
    tilt_min_deg: float = -22.5
    tilt_max_deg: float = 37.5
    compass_bias_deg: float = 0.0
    settle_tolerance_deg: float = 0.5
    telemetry_timeout_s: float = 2.0
    compass_drift_tolerance_deg: float = 5.0

    def __post_init__(self) -> None:
        if self.pan_min_deg >= self.pan_max_deg or self.tilt_min_deg >= self.tilt_max_deg:
            raise ValueError("tower angle limits must be ordered")
        if (
            self.settle_tolerance_deg < 0
            or self.telemetry_timeout_s <= 0
            or self.compass_drift_tolerance_deg < 0
        ):
            raise ValueError("tower tolerances and timeout must be positive")

    def true_heading(self, relative_pan_deg: float) -> float:
        """Convert base-relative pan to true heading."""
        return wrap_heading(self.base_true_heading_deg - relative_pan_deg)

    def target_pan(self, target_bearing_deg: float) -> float:
        """Convert a true target bearing to base-relative pan."""
        return wrap_pan(self.base_true_heading_deg - target_bearing_deg)

    def calibrated_compass(self, raw_heading_deg: float) -> float:
        return wrap_heading(raw_heading_deg + self.compass_bias_deg)

    def with_compass_reference(
        self, raw_heading_deg: float, true_heading_deg: float | None = None
    ) -> TowerCalibration:
        """Return calibration whose compass matches a known true reference."""
        reference = self.base_true_heading_deg if true_heading_deg is None else true_heading_deg
        return TowerCalibration(
            base_true_heading_deg=self.base_true_heading_deg,
            pan_min_deg=self.pan_min_deg,
            pan_max_deg=self.pan_max_deg,
            tilt_min_deg=self.tilt_min_deg,
            tilt_max_deg=self.tilt_max_deg,
            compass_bias_deg=wrap_pan(reference - raw_heading_deg),
            settle_tolerance_deg=self.settle_tolerance_deg,
            telemetry_timeout_s=self.telemetry_timeout_s,
            compass_drift_tolerance_deg=self.compass_drift_tolerance_deg,
        )

    calibrate_compass = with_compass_reference

    def validate_pan(self, pan_deg: float) -> float:
        pan = float(pan_deg)
        if not self.pan_min_deg <= pan <= self.pan_max_deg:
            raise ValueError(f"tower pan must be between {self.pan_min_deg:g} and {self.pan_max_deg:g} degrees")
        return pan

    def validate_tilt(self, tilt_deg: float) -> float:
        tilt = float(tilt_deg)
        if not self.tilt_min_deg <= tilt <= self.tilt_max_deg:
            raise ValueError(f"tower tilt must be between {self.tilt_min_deg:g} and {self.tilt_max_deg:g} degrees")
        return tilt


@dataclass(frozen=True, slots=True)
class TowerOrientation:
    """Measured tower pose. Angular-rate data is deliberately not integrated."""

    pan_deg: float
    tilt_deg: float
    timestamp: datetime
    commanded_pan_deg: float | None = None
    commanded_tilt_deg: float | None = None
    compass_heading_deg: float | None = None
    calibration: TowerCalibration = TowerCalibration()
    angular_rate_dps: float | None = None
    settled_since: datetime | None = None

    def __post_init__(self) -> None:
        self.calibration.validate_pan(self.pan_deg)
        self.calibration.validate_tilt(self.tilt_deg)
        if self.commanded_pan_deg is not None:
            self.calibration.validate_pan(self.commanded_pan_deg)
        if self.commanded_tilt_deg is not None:
            self.calibration.validate_tilt(self.commanded_tilt_deg)

    @property
    def true_heading_deg(self) -> float:
        if self.compass_heading_deg is not None:
            return self.calibration.calibrated_compass(self.compass_heading_deg)
        return self.calibration.true_heading(self.pan_deg)

    def is_settled(self, tolerance_deg: float | None = None) -> bool:
        tolerance = self.calibration.settle_tolerance_deg if tolerance_deg is None else tolerance_deg
        if tolerance < 0:
            raise ValueError("settle tolerance cannot be negative")
        if (
            self.commanded_pan_deg is None
            or self.commanded_tilt_deg is None
            or self.angular_rate_dps is None
            or self.settled_since is None
        ):
            return False
        return (
            abs(wrap_pan(self.pan_deg - self.commanded_pan_deg)) <= tolerance
            and abs(self.tilt_deg - self.commanded_tilt_deg) <= tolerance
            and math.isfinite(self.angular_rate_dps)
            and abs(self.angular_rate_dps) < TOWER_SETTLED_RATE_DPS
            and (self.timestamp - self.settled_since).total_seconds()
            >= TOWER_SETTLED_DURATION_S
        )

    @property
    def settled(self) -> bool:
        return self.is_settled()

    def is_healthy(self, now: datetime | None = None) -> bool:
        check_time = now or datetime.now(timezone.utc)
        age = (check_time - self.timestamp).total_seconds()
        values_are_healthy = (
            math.isfinite(self.pan_deg)
            and math.isfinite(self.tilt_deg)
            and -0.1 <= age <= self.calibration.telemetry_timeout_s
        )
        if not values_are_healthy or self.compass_heading_deg is None:
            return values_are_healthy
        compass_drift = wrap_pan(
            self.calibration.calibrated_compass(self.compass_heading_deg)
            - self.calibration.true_heading(self.pan_deg)
        )
        return abs(compass_drift) <= self.calibration.compass_drift_tolerance_deg

    @property
    def healthy(self) -> bool:
        return self.is_healthy()

    def with_telemetry(
        self,
        *,
        pan_deg: float,
        tilt_deg: float,
        timestamp: datetime,
        compass_heading_deg: float | None = None,
        angular_rate_dps: float | None = None,
        settled_since: datetime | None = None,
        gyro_z_dps: float | None = None,
    ) -> TowerOrientation:
        """Replace measurements without integrating angular rate into orientation."""
        if angular_rate_dps is not None and gyro_z_dps is not None:
            raise ValueError("provide only one angular-rate measurement")
        measured_rate = angular_rate_dps if angular_rate_dps is not None else gyro_z_dps
        return TowerOrientation(
            pan_deg=pan_deg,
            tilt_deg=tilt_deg,
            timestamp=timestamp,
            commanded_pan_deg=self.commanded_pan_deg,
            commanded_tilt_deg=self.commanded_tilt_deg,
            compass_heading_deg=compass_heading_deg,
            calibration=self.calibration,
            angular_rate_dps=measured_rate,
            settled_since=settled_since,
        )
