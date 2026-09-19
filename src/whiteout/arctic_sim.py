"""Ordered WHITEOUT endpoint definitions for the arctic-sim deployment."""

from __future__ import annotations

from dataclasses import dataclass

from .camera import MjpegCamera
from .control import CopterController, PlaneController, TowerController
from .mavproxy import MavProxySession
from .objects import Copter, Plane, RepositoryObject, Tower


DEFAULT_ARCTIC_SIM_HOST = "10.99.0.1"


@dataclass(frozen=True, slots=True)
class ArcticSimAsset:
    """One deployed asset with matching MAVProxy and camera endpoints."""

    vehicle: RepositoryObject
    camera_port: int
    camera_path: str = "/stream"

    def __post_init__(self) -> None:
        if not 1 <= self.camera_port <= 65535:
            raise ValueError(f"invalid camera port: {self.camera_port}")
        if not self.camera_path.startswith("/"):
            raise ValueError("camera path must begin with '/'")

    @property
    def name(self) -> str:
        return self.vehicle.name

    @property
    def mavproxy_command(self) -> str:
        return self.vehicle.mavproxy_start_command()

    @property
    def camera_url(self) -> str:
        return f"http://{self.vehicle.host}:{self.camera_port}{self.camera_path}"

    def camera(self, *, timeout_s: float = 5.0) -> MjpegCamera:
        return MjpegCamera(self.name, self.camera_url, timeout_s=timeout_s)

    def mavproxy_session(
        self,
        *,
        executable: str = "mavproxy.py",
        startup_commands: tuple[str, ...] = (),
        non_interactive: bool = False,
    ) -> MavProxySession:
        return self.vehicle.mavproxy_session(
            executable=executable,
            startup_commands=startup_commands,
            non_interactive=non_interactive,
        )

    def controller(
        self, *, executable: str = "mavproxy.py", connection_timeout_s: float = 15.0
    ) -> CopterController | PlaneController | TowerController:
        """Create the asset's typed Python controller."""
        if isinstance(self.vehicle, Copter):
            return self.vehicle.controller(
                executable=executable,
                connection_timeout_s=connection_timeout_s,
            )
        if isinstance(self.vehicle, Plane):
            return self.vehicle.controller(
                executable=executable,
                connection_timeout_s=connection_timeout_s,
            )
        if isinstance(self.vehicle, Tower):
            return self.vehicle.controller(
                executable=executable,
                connection_timeout_s=connection_timeout_s,
            )
        raise TypeError(f"{type(self.vehicle).__name__} has no deployed controller")


def arctic_sim_fleet(host: str = DEFAULT_ARCTIC_SIM_HOST) -> tuple[ArcticSimAsset, ...]:
    """Return the deployed fleet in operator/UI order."""
    return (
        ArcticSimAsset(Copter(host=host), 8600),
        ArcticSimAsset(Plane(host=host), 8610),
        ArcticSimAsset(Tower.one(host), 8630),
        ArcticSimAsset(Tower.two(host), 8640),
    )


ARCTIC_SIM_FLEET = arctic_sim_fleet()
