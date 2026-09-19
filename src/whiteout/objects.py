"""Repository object types and their MAVProxy console commands.

The classes describe simulator assets and build reviewable command values.
Connections and transmission happen only through an explicitly opened
``MavProxySession``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
import re
import shlex
from typing import TYPE_CHECKING, ClassVar, Sequence

if TYPE_CHECKING:
    from .mavproxy import MavProxySession


class ObjectType(str, Enum):
    COPTER = "copter"
    PLANE = "plane"
    BOAT = "boat"
    TOWER = "tower"


class UnsupportedCommand(ValueError):
    """Raised when a command is not meaningful for an object type."""


_IDENTIFIER = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def _identifier(value: str, description: str) -> str:
    if not _IDENTIFIER.fullmatch(value):
        raise ValueError(f"invalid {description}: {value!r}")
    return value


def _finite_number(value: float, description: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{description} must be finite")
    return value


def _pwm(value: int) -> int:
    value = int(value)
    if not 1000 <= value <= 2000:
        raise ValueError("PWM must be between 1000 and 2000")
    return value


@dataclass(frozen=True, slots=True)
class MavProxyCommand:
    """A non-executable MAVProxy console command."""

    name: str
    arguments: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        _identifier(self.name, "command name")
        if any("\n" in item or "\r" in item for item in self.arguments):
            raise ValueError("command arguments cannot contain newlines")

    def render(self) -> str:
        return shlex.join((self.name, *self.arguments))

    def __str__(self) -> str:
        return self.render()


@dataclass(frozen=True, slots=True)
class RepositoryObject:
    """Common description and command builders for one simulated object."""

    name: str
    host: str
    port: int
    bundled: bool = True

    object_type: ClassVar[ObjectType]
    supported_modes: ClassVar[frozenset[str]] = frozenset()
    supports_arming: ClassVar[bool] = True
    required_mavproxy_modules: ClassVar[tuple[str, ...]] = ()

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("object name must not be empty")
        if not self.host.strip() or any(character.isspace() for character in self.host):
            raise ValueError("host must be a non-empty address without whitespace")
        if not 1 <= self.port <= 65535:
            raise ValueError(f"invalid MAVLink port: {self.port}")

    @property
    def mavproxy_address(self) -> str:
        return f"udpout:{self.host}:{self.port}"

    def mavproxy_start_arguments(self) -> tuple[str, str]:
        """Return arguments suitable for a subprocess argument vector."""
        return ("mavproxy.py", f"--master={self.mavproxy_address}")

    def mavproxy_start_command(self) -> str:
        """Return the documented operator-facing MAVProxy launch command."""
        return shlex.join(self.mavproxy_start_arguments())

    def mavproxy_session(
        self,
        *,
        executable: str = "mavproxy.py",
        extra_arguments: Sequence[str] = (),
    ) -> MavProxySession:
        """Create, but do not start, a MAVProxy session for this object."""
        if not self.bundled:
            raise UnsupportedCommand(
                f"{self.name} is only a reserved arctic-sim endpoint and has no bundled model"
            )
        from .mavproxy import MavProxySession

        return MavProxySession(
            self.mavproxy_address,
            executable=executable,
            extra_arguments=extra_arguments,
            startup_commands=tuple(
                f"module load {module}" for module in self.required_mavproxy_modules
            ),
        )

    def connect_mavproxy(
        self,
        *,
        executable: str = "mavproxy.py",
        extra_arguments: Sequence[str] = (),
    ) -> MavProxySession:
        """Launch MAVProxy using arctic-sim's required ``udpout`` endpoint."""
        session = self.mavproxy_session(
            executable=executable,
            extra_arguments=extra_arguments,
        )
        return session.start()

    def status(self) -> MavProxyCommand:
        return MavProxyCommand("status")

    def watch(self, message: str) -> MavProxyCommand:
        return MavProxyCommand("watch", (_identifier(message, "MAVLink message"),))

    def mode(self, mode: str) -> MavProxyCommand:
        normalized = mode.upper()
        if normalized not in self.supported_modes:
            choices = ", ".join(sorted(self.supported_modes))
            raise UnsupportedCommand(
                f"{normalized!r} is not a supported {self.object_type.value} mode; "
                f"choose one of: {choices}"
            )
        return MavProxyCommand("mode", (normalized,))

    def arm(self, *, force: bool = False) -> MavProxyCommand:
        if not self.supports_arming:
            raise UnsupportedCommand(f"arming is not meaningful for {self.object_type.value}")
        return MavProxyCommand("arm", ("force" if force else "throttle",))

    def disarm(self, *, force: bool = False) -> MavProxyCommand:
        if not self.supports_arming:
            raise UnsupportedCommand(f"disarming is not meaningful for {self.object_type.value}")
        return MavProxyCommand("disarm", ("force",) if force else ())

    def rc(self, channel: int, pwm: int | None) -> MavProxyCommand:
        if channel < 1:
            raise ValueError("RC channel must be positive")
        rendered_pwm = 0 if pwm is None else _pwm(pwm)
        return MavProxyCommand("rc", (str(channel), str(rendered_pwm)))

    def release_rc(self) -> MavProxyCommand:
        return MavProxyCommand("rc", ("all", "0"))

    def show_parameter(self, parameter: str) -> MavProxyCommand:
        return MavProxyCommand("param", ("show", _identifier(parameter, "parameter name")))

    def set_parameter(self, parameter: str, value: float) -> MavProxyCommand:
        number = _finite_number(value, "parameter value")
        return MavProxyCommand(
            "param", ("set", _identifier(parameter, "parameter name"), format(number, "g"))
        )

    def load_mission(self, path: str) -> MavProxyCommand:
        if not path or "\n" in path or "\r" in path:
            raise ValueError("mission path must be a non-empty single line")
        return MavProxyCommand("wp", ("load", path))

    def automatic_mission(self, path: str) -> tuple[MavProxyCommand, ...]:
        return (self.load_mission(path), self.arm(), self.mode("AUTO"))


@dataclass(frozen=True, slots=True)
class Copter(RepositoryObject):
    name: str = "quadcopter"
    host: str = "127.0.0.1"
    port: int = 14550

    object_type: ClassVar[ObjectType] = ObjectType.COPTER
    supported_modes: ClassVar[frozenset[str]] = frozenset(
        {"STABILIZE", "LOITER", "GUIDED", "AUTO", "RTL", "LAND"}
    )

    def takeoff(self, altitude_m: float) -> MavProxyCommand:
        altitude = _finite_number(altitude_m, "takeoff altitude")
        if altitude <= 0:
            raise ValueError("takeoff altitude must be positive")
        return MavProxyCommand("takeoff", (format(altitude, "g"),))

    def set_speed(self, speed_mps: float) -> MavProxyCommand:
        speed = _finite_number(speed_mps, "speed")
        if speed <= 0:
            raise ValueError("speed must be positive")
        return MavProxyCommand("setspeed", (format(speed, "g"),))

    def set_yaw(
        self, angle_deg: float, angular_speed_dps: float, *, relative: bool = False
    ) -> MavProxyCommand:
        angle = _finite_number(angle_deg, "yaw angle")
        angular_speed = _finite_number(angular_speed_dps, "angular speed")
        if angular_speed < 0:
            raise ValueError("angular speed cannot be negative")
        return MavProxyCommand(
            "setyaw",
            (format(angle, "g"), format(angular_speed, "g"), "1" if relative else "0"),
        )

    def velocity(self, north_mps: float, east_mps: float, down_mps: float) -> MavProxyCommand:
        values = tuple(
            format(_finite_number(value, "velocity component"), "g")
            for value in (north_mps, east_mps, down_mps)
        )
        return MavProxyCommand("velocity", values)

    def position(self, north_m: float, east_m: float, down_m: float) -> MavProxyCommand:
        values = tuple(
            format(_finite_number(value, "position component"), "g")
            for value in (north_m, east_m, down_m)
        )
        return MavProxyCommand("position", values)


@dataclass(frozen=True, slots=True)
class Plane(RepositoryObject):
    name: str = "fixed-wing"
    host: str = "127.0.0.1"
    port: int = 14560

    object_type: ClassVar[ObjectType] = ObjectType.PLANE
    supported_modes: ClassVar[frozenset[str]] = frozenset(
        {"MANUAL", "FBWA", "AUTO", "RTL", "LOITER", "CIRCLE"}
    )

    def set_speed(self, speed_mps: float) -> MavProxyCommand:
        speed = _finite_number(speed_mps, "speed")
        if speed <= 0:
            raise ValueError("speed must be positive")
        return MavProxyCommand("setspeed", (format(speed, "g"),))


@dataclass(frozen=True, slots=True)
class Boat(RepositoryObject):
    name: str = "boat"
    host: str = "127.0.0.1"
    port: int = 14570
    bundled: bool = False

    object_type: ClassVar[ObjectType] = ObjectType.BOAT
    supported_modes: ClassVar[frozenset[str]] = frozenset(
        {"MANUAL", "HOLD", "GUIDED", "AUTO", "RTL"}
    )


@dataclass(frozen=True, slots=True)
class Tower(RepositoryObject):
    name: str = "tower-1"
    host: str = "127.0.0.1"
    port: int = 14580

    object_type: ClassVar[ObjectType] = ObjectType.TOWER
    supported_modes: ClassVar[frozenset[str]] = frozenset({"MANUAL", "AUTO"})
    supports_arming: ClassVar[bool] = False
    required_mavproxy_modules: ClassVar[tuple[str, ...]] = ("servo",)

    @classmethod
    def one(cls, host: str = "127.0.0.1") -> Tower:
        return cls("tower-1", host, 14580)

    @classmethod
    def two(cls, host: str = "127.0.0.1") -> Tower:
        return cls("tower-2", host, 14590)

    def servo(self, servo_number: int, pwm: int) -> MavProxyCommand:
        if servo_number not in (1, 2):
            raise ValueError("tower servo must be 1 (pan) or 2 (tilt)")
        return MavProxyCommand("servo", ("set", str(servo_number), str(_pwm(pwm))))

    def pan(self, pwm: int) -> MavProxyCommand:
        return self.servo(1, pwm)

    def tilt(self, pwm: int) -> MavProxyCommand:
        return self.servo(2, pwm)

    def automatic_mission(self, path: str) -> tuple[MavProxyCommand, ...]:
        raise UnsupportedCommand("missions are not defined for the repository towers")


def object_for_type(
    object_type: ObjectType | str, *, host: str = "127.0.0.1"
) -> RepositoryObject:
    """Create the default repository object for a type name."""
    try:
        normalized = ObjectType(object_type)
    except ValueError as exc:
        choices = ", ".join(item.value for item in ObjectType)
        raise ValueError(f"unknown object type {object_type!r}; choose one of: {choices}") from exc
    classes: dict[ObjectType, type[RepositoryObject]] = {
        ObjectType.COPTER: Copter,
        ObjectType.PLANE: Plane,
        ObjectType.BOAT: Boat,
        ObjectType.TOWER: Tower,
    }
    return classes[normalized](host=host)


# More descriptive aliases for callers that use the repository role names.
Quadcopter = Copter
FixedWingPlane = Plane
