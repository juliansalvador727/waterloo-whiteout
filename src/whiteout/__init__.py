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
    "FixedWingPlane",
    "MavProxyCommand",
    "MavProxyConnectionError",
    "MissionUploadError",
    "MissionValidationError",
    "MissionWaypoint",
    "MavProxyNotRunning",
    "MavProxySession",
    "MavProxyUnavailable",
    "ObjectType",
    "ObjectController",
    "ObservationMetadata",
    "Plane",
    "PlaneController",
    "PlaneMode",
    "Quadcopter",
    "RepositoryObject",
    "SearchAction",
    "SearchMission",
    "SearchOrchestrator",
    "SearchRecommendation",
    "DetectionSource",
    "SynchronizationStatus",
    "Tower",
    "TowerController",
    "VehicleStateError",
    "UnsupportedCommand",
    "arctic_sim_fleet",
    "object_for_type",
    "tower_angle_to_pwm",
    "tower_pwm_to_angle",
]

__version__ = "0.1.0"
