"""Typed Python control API backed by a managed MAVProxy process."""

from __future__ import annotations

from enum import Enum
from os import PathLike
from pathlib import Path
import re
import time
from typing import Generic, Self, TypeVar

from .mavproxy import MavProxySession
from .objects import Copter, MavProxyCommand, Plane, RepositoryObject, Tower


class MavProxyConnectionError(ConnectionError):
    """Raised when a managed controller cannot establish a MAVProxy link."""


class MissionUploadError(RuntimeError):
    """Raised when MAVProxy does not confirm a mission upload."""


class VehicleStateError(RuntimeError):
    """Raised when a safe control transition cannot be verified."""


class CopterMode(str, Enum):
    STABILIZE = "STABILIZE"
    LOITER = "LOITER"
    GUIDED = "GUIDED"
    AUTO = "AUTO"
    RTL = "RTL"
    LAND = "LAND"


class PlaneMode(str, Enum):
    MANUAL = "MANUAL"
    FBWA = "FBWA"
    TAKEOFF = "TAKEOFF"
    AUTO = "AUTO"
    RTL = "RTL"
    LOITER = "LOITER"
    CIRCLE = "CIRCLE"


ObjectT = TypeVar("ObjectT", bound=RepositoryObject)


class ObjectController(Generic[ObjectT]):
    """Base for a typed object controller that transmits through MAVProxy."""

    def __init__(
        self,
        vehicle: ObjectT,
        *,
        executable: str = "mavproxy.py",
        connection_timeout_s: float = 15.0,
    ) -> None:
        if connection_timeout_s <= 0:
            raise ValueError("connection timeout must be positive")
        self.vehicle = vehicle
        self.connection_timeout_s = float(connection_timeout_s)
        self.session: MavProxySession = vehicle.mavproxy_session(executable=executable)

    @property
    def connected(self) -> bool:
        return self.session.running and "online system" in self.session.output

    def start(self) -> Self:
        """Start MAVProxy and wait for its first vehicle heartbeat."""
        self.session.start()
        if not self.session.wait_for_output(
            "online system", timeout=self.connection_timeout_s
        ):
            output = self.session.output.strip()
            self.session.close()
            detail = f" MAVProxy output:\n{output}" if output else ""
            raise MavProxyConnectionError(
                f"{self.vehicle.name} did not come online within "
                f"{self.connection_timeout_s:g} seconds.{detail}"
            )
        return self

    def close(self) -> None:
        self.session.close()

    def _send(self, command: MavProxyCommand) -> None:
        self.session.send(command)

    def status(self) -> None:
        self._send(self.vehicle.status())

    def watch(self, message: str) -> None:
        self._send(self.vehicle.watch(message))

    def set_rc(self, channel: int, pwm: int | None) -> None:
        self._send(self.vehicle.rc(channel, pwm))

    def release_rc(self) -> None:
        self._send(self.vehicle.release_rc())

    def show_parameter(self, parameter: str) -> None:
        self._send(self.vehicle.show_parameter(parameter))

    def set_parameter(self, parameter: str, value: float) -> None:
        self._send(self.vehicle.set_parameter(parameter, value))

    def is_armed(self, *, timeout_s: float = 5.0) -> bool:
        """Read the current armed flag from MAVProxy's latest heartbeat."""
        if timeout_s <= 0:
            raise ValueError("state timeout must be positive")
        checkpoint = len(self.session.output)
        self._send(MavProxyCommand("status", ("HEARTBEAT",)))
        if not self.session.wait_for_output(
            "HEARTBEAT {", timeout=timeout_s, start=checkpoint
        ):
            raise VehicleStateError("MAVProxy did not return a HEARTBEAT status")
        output = self.session.output[checkpoint:]
        matches = re.findall(r"base_mode\s*:\s*(\d+)", output)
        if not matches:
            raise VehicleStateError("HEARTBEAT status did not contain base_mode")
        return bool(int(matches[-1]) & 128)

    def wait_until_disarmed(
        self, *, timeout_s: float = 180.0, poll_interval_s: float = 2.0
    ) -> None:
        """Wait for a confirmed disarmed heartbeat after landing."""
        if timeout_s <= 0 or poll_interval_s <= 0:
            raise ValueError("disarm timeout and poll interval must be positive")
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise VehicleStateError(
                    f"vehicle remained armed for more than {timeout_s:g} seconds"
                )
            if not self.is_armed(timeout_s=min(5.0, remaining)):
                return
            time.sleep(min(poll_interval_s, max(0.0, deadline - time.monotonic())))

    def __enter__(self) -> Self:
        return self.start()

    def __exit__(self, *args: object) -> None:
        self.close()


class CopterController(ObjectController[Copter]):
    """Typed flight functions for an ArduCopter-backed object."""

    def set_mode(self, mode: CopterMode) -> None:
        if not isinstance(mode, CopterMode):
            raise TypeError("mode must be a CopterMode")
        self._send(self.vehicle.mode(mode.value))

    def arm(self, *, force: bool = False) -> None:
        self._send(self.vehicle.arm(force=force))

    def disarm(self, *, force: bool = False) -> None:
        self._send(self.vehicle.disarm(force=force))

    def takeoff(self, altitude_m: float) -> None:
        self._send(self.vehicle.takeoff(altitude_m))

    def land(self) -> None:
        self.set_mode(CopterMode.LAND)

    def autoland(self) -> None:
        """Enter ArduCopter LAND mode."""
        self.land()

    def return_to_launch(self) -> None:
        self.set_mode(CopterMode.RTL)

    def set_speed(self, speed_mps: float) -> None:
        self._send(self.vehicle.set_speed(speed_mps))

    def set_yaw(
        self, angle_deg: float, angular_speed_dps: float, *, relative: bool = False
    ) -> None:
        self._send(
            self.vehicle.set_yaw(
                angle_deg, angular_speed_dps, relative=relative
            )
        )

    def set_velocity(
        self, north_mps: float, east_mps: float, down_mps: float
    ) -> None:
        self._send(self.vehicle.velocity(north_mps, east_mps, down_mps))

    def set_position(self, north_m: float, east_m: float, down_m: float) -> None:
        self._send(self.vehicle.position(north_m, east_m, down_m))


class PlaneController(ObjectController[Plane]):
    """Typed flight functions for an ArduPlane-backed object."""

    def set_mode(self, mode: PlaneMode) -> None:
        if not isinstance(mode, PlaneMode):
            raise TypeError("mode must be a PlaneMode")
        self._send(self.vehicle.mode(mode.value))

    def arm(self, *, force: bool = False) -> None:
        self._send(self.vehicle.arm(force=force))

    def disarm(self, *, force: bool = False) -> None:
        self._send(self.vehicle.disarm(force=force))

    def takeoff(self, *, force_arm: bool = False) -> None:
        """Select ArduPlane TAKEOFF mode and arm the plane."""
        self.set_mode(PlaneMode.TAKEOFF)
        self.arm(force=force_arm)

    def loiter(self) -> None:
        self.set_mode(PlaneMode.LOITER)

    def return_to_launch(self) -> None:
        self.set_mode(PlaneMode.RTL)

    def autoland(
        self,
        mission_path: str | PathLike[str],
        *,
        upload_timeout_s: float = 15.0,
    ) -> None:
        """Upload a landing mission and enter ArduPlane AUTO mode."""
        if upload_timeout_s <= 0:
            raise ValueError("mission upload timeout must be positive")
        mission = Path(mission_path).resolve()
        if not mission.is_file():
            raise FileNotFoundError(f"landing mission not found: {mission}")

        # Conservative belly-landing tune for arctic-sim's Skywalker X8. The
        # stock zero-degree pitch caused a flat/nose-first touchdown.
        for parameter, value in (
            ("LAND_PITCH_DEG", 4.0),
            ("LAND_FLARE_ALT", 4.0),
            ("LAND_FLARE_SEC", 3.0),
            ("TECS_LAND_SINK", 0.2),
        ):
            self.set_parameter(parameter, value)

        checkpoint = len(self.session.output)
        self._send(self.vehicle.load_mission(str(mission)))
        if not self.session.wait_for_output(
            "Sent all ", timeout=upload_timeout_s, start=checkpoint
        ):
            raise MissionUploadError(
                f"MAVProxy did not confirm the landing mission upload: {mission}"
            )
        self.set_mode(PlaneMode.AUTO)

    def reset_after_landing(self, *, state_timeout_s: float = 5.0) -> None:
        """Clear the landing sequence after verified disarm for another flight."""
        if self.is_armed(timeout_s=state_timeout_s):
            raise VehicleStateError(
                "refusing to clear the landing mission while the plane is armed"
            )
        self.set_mode(PlaneMode.MANUAL)
        self._send(MavProxyCommand("wp", ("clear",)))

    def set_speed(self, speed_mps: float) -> None:
        self._send(self.vehicle.set_speed(speed_mps))


class TowerController(ObjectController[Tower]):
    """Typed pan/tilt functions for one simulated tower."""

    def pan(self, pwm: int) -> None:
        self._send(self.vehicle.pan(pwm))

    def tilt(self, pwm: int) -> None:
        self._send(self.vehicle.tilt(pwm))

    def center(self) -> None:
        self.pan(1500)
        self.tilt(1500)
