"""Dominion Dynamics WHITEOUT safety-first controller scaffold."""

from .arctic_sim import (
    ARCTIC_SIM_FLEET,
    DEFAULT_ARCTIC_SIM_HOST,
    ArcticSimAsset,
    arctic_sim_fleet,
)
from .mavproxy import MavProxyNotRunning, MavProxySession, MavProxyUnavailable
from .mission import MissionValidationError, MissionWaypoint, SearchMission
from .observation import ObservationMetadata, SynchronizationStatus
from .camera import CameraModel, FIXED_WING_CAMERA, QUAD_CAMERA, TOWER_CAMERA
from .config import CourseBounds
from .execution import (
    Executor,
    LiveExecutor,
    LiveTrackSink,
    OperatorSession,
    RecordingExecutor,
    RecordingTrackSink,
    TrackSink,
)
from .models import ControlIntent, TrackEstimate
from .tower import (
    FORT_ROSS_TOWERS,
    MAX_TOWER_SCAN_HORIZONTAL_STEP_DEG,
    MAX_TOWER_SCAN_VERTICAL_STEP_DEG,
    TowerCalibration,
    TowerOrientation,
    tower_scan_overlap_steps,
    validate_tower_scan_steps,
)
from .search import (
    DetectionSource,
    SearchAction,
    SearchOrchestrator,
    SearchRecommendation,
)
from .control import (
    CopterController,
    CopterMode,
    MavProxyConnectionError,
    MissionUploadError,
    ObjectController,
    PlaneController,
    PlaneMode,
    TowerController,
    VehicleStateError,
)
from .objects import (
    Boat,
    Copter,
    FixedWingPlane,
    MavProxyCommand,
    ObjectType,
    Plane,
    Quadcopter,
    RepositoryObject,
    Tower,
    UnsupportedCommand,
    object_for_type,
    tower_angle_to_pwm,
    tower_pwm_to_angle,
)

__all__ = [
    "ARCTIC_SIM_FLEET",
    "DEFAULT_ARCTIC_SIM_HOST",
    "ArcticSimAsset",
    "Boat",
    "Copter",
    "CopterController",
    "CopterMode",
    "CameraModel",
    "ControlIntent",
    "CourseBounds",
    "Executor",
    "FIXED_WING_CAMERA",
    "FORT_ROSS_TOWERS",
    "FixedWingPlane",
    "MavProxyCommand",
    "MavProxyConnectionError",
    "MissionUploadError",
    "MissionValidationError",
    "MissionWaypoint",
    "MavProxyNotRunning",
    "MavProxySession",
    "MavProxyUnavailable",
    "MAX_TOWER_SCAN_HORIZONTAL_STEP_DEG",
    "MAX_TOWER_SCAN_VERTICAL_STEP_DEG",
    "LiveExecutor",
    "LiveTrackSink",
    "ObjectType",
    "ObjectController",
    "ObservationMetadata",
    "OperatorSession",
    "Plane",
    "PlaneController",
    "PlaneMode",
    "Quadcopter",
    "QUAD_CAMERA",
    "RecordingExecutor",
    "RecordingTrackSink",
    "RepositoryObject",
    "SearchAction",
    "SearchMission",
    "SearchOrchestrator",
    "SearchRecommendation",
    "DetectionSource",
    "SynchronizationStatus",
    "TOWER_CAMERA",
    "Tower",
    "TowerCalibration",
    "TowerController",
    "TowerOrientation",
    "TrackEstimate",
    "TrackSink",
    "VehicleStateError",
    "UnsupportedCommand",
    "arctic_sim_fleet",
    "object_for_type",
    "tower_angle_to_pwm",
    "tower_pwm_to_angle",
    "tower_scan_overlap_steps",
    "validate_tower_scan_steps",
]

__version__ = "0.1.0"
