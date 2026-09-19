"""Dominion Dynamics WHITEOUT safety-first controller scaffold."""

from .mavproxy import MavProxyNotRunning, MavProxySession, MavProxyUnavailable
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
    "Boat",
    "Copter",
    "FixedWingPlane",
    "MavProxyCommand",
    "MavProxyNotRunning",
    "MavProxySession",
    "MavProxyUnavailable",
    "ObjectType",
    "Plane",
    "Quadcopter",
    "RepositoryObject",
    "Tower",
    "UnsupportedCommand",
    "object_for_type",
]

__version__ = "0.1.0"

