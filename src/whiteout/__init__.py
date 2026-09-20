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
from .config import CoordinatorConfig, CourseBounds, TowerMotionConfig
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
from .pipeline import OperationalPipeline, PipelineCycle
from .live import SimulatorActuator, TrackApiSubmitter
from .pose import (
    FIXED_WING_CAMERA_DOWN_DEG,
    QUADCOPTER_CAMERA_DOWN_DEG,
    aircraft_camera_pose,
    tower_camera_pose,
)
from .simulator_metadata import ArcticSimMetadataClient, GeneratedAssetPose
from .runtime import UnifiedCoordinatorRuntime
from .reacquisition_grid import GridCell, WeightedReacquisitionGrid
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
    "ArcticSimMetadataClient",
    "Boat",
    "Copter",
    "CopterController",
    "CopterMode",
    "CameraModel",
    "ControlIntent",
    "CoordinatorConfig",
    "CourseBounds",
    "TowerMotionConfig",
    "Executor",
    "FIXED_WING_CAMERA",
    "FIXED_WING_CAMERA_DOWN_DEG",
    "FORT_ROSS_TOWERS",
    "FixedWingPlane",
    "GeneratedAssetPose",
    "GridCell",
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
    "OperationalPipeline",
    "OperatorSession",
    "Plane",
    "PlaneController",
    "PlaneMode",
    "PipelineCycle",
    "Quadcopter",
    "QUAD_CAMERA",
    "QUADCOPTER_CAMERA_DOWN_DEG",
    "RecordingExecutor",
    "RecordingTrackSink",
    "RepositoryObject",
    "SearchAction",
    "SearchMission",
    "SearchOrchestrator",
    "SearchRecommendation",
    "SimulatorActuator",
    "DetectionSource",
    "SynchronizationStatus",
    "TOWER_CAMERA",
    "Tower",
    "TowerCalibration",
    "TowerController",
    "TowerOrientation",
    "TrackEstimate",
    "TrackApiSubmitter",
    "TrackSink",
    "UnifiedCoordinatorRuntime",
    "WeightedReacquisitionGrid",
    "VehicleStateError",
    "UnsupportedCommand",
    "arctic_sim_fleet",
    "aircraft_camera_pose",
    "object_for_type",
    "tower_angle_to_pwm",
    "tower_camera_pose",
    "tower_pwm_to_angle",
    "tower_scan_overlap_steps",
    "validate_tower_scan_steps",
]

__version__ = "0.1.0"
