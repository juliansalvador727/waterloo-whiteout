"""Dominion Dynamics WHITEOUT safety-first controller scaffold."""

from .arctic_sim import (
    ARCTIC_SIM_FLEET,
    DEFAULT_ARCTIC_SIM_HOST,
    ArcticSimAsset,
    arctic_sim_fleet,
)
from .mavproxy import MavProxyNotRunning, MavProxySession, MavProxyUnavailable
from .control import (
    CopterController,
    CopterMode,
    MavProxyConnectionError,
    MissionUploadError,
    ObjectController,
    PlaneController,
    PlaneMode,
    TowerController,
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
    "MavProxyNotRunning",
    "MavProxySession",
    "MavProxyUnavailable",
    "ObjectType",
    "ObjectController",
    "Plane",
    "PlaneController",
    "PlaneMode",
    "Quadcopter",
    "RepositoryObject",
    "Tower",
    "TowerController",
    "UnsupportedCommand",
    "arctic_sim_fleet",
    "object_for_type",
]

__version__ = "0.1.0"

