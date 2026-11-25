"""Deterministic support trajectories in metres and simulation seconds."""

from dataclasses import dataclass
import math

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class PoseVelocity:
    position: Vec3
    yaw: float
    velocity: Vec3
    angular_velocity: Vec3


@dataclass(frozen=True)
class OscillatingPlatform:
    center: Vec3
    amplitude: Vec3
    period_s: float
    phase: float = 0.0
    yaw_amplitude: float = 0.0

    def __post_init__(self):
        if not math.isfinite(self.period_s) or self.period_s <= 0:
            raise ValueError("period_s must be positive and finite")
        if not all(math.isfinite(v) for v in (*self.center, *self.amplitude, self.phase, self.yaw_amplitude)):
            raise ValueError("trajectory parameters must be finite")

    def at(self, seconds: float) -> PoseVelocity:
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("simulation time must be finite and nonnegative")
        omega = 2 * math.pi / self.period_s
        angle = self.phase + omega * seconds
        position = tuple(c + a * math.sin(angle) for c, a in zip(self.center, self.amplitude))
        velocity = tuple(a * omega * math.cos(angle) for a in self.amplitude)
        return PoseVelocity(position, self.yaw_amplitude * math.sin(angle), velocity,
                            (0.0, 0.0, self.yaw_amplitude * omega * math.cos(angle)))


@dataclass(frozen=True)
class Turntable:
    center: Vec3
    radius: float
    angular_speed: float
    phase: float = 0.0

    def __post_init__(self):
        if self.radius < 0 or not all(math.isfinite(x) for x in (*self.center, self.radius, self.angular_speed, self.phase)):
            raise ValueError("turntable parameters must be finite; radius must be nonnegative")

    def at(self, seconds: float) -> PoseVelocity:
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("simulation time must be finite and nonnegative")
        theta = self.phase + self.angular_speed * seconds
        c, s = math.cos(theta), math.sin(theta)
        return PoseVelocity((self.center[0] + self.radius*c, self.center[1] + self.radius*s, self.center[2]),
                            theta, (-self.radius*self.angular_speed*s, self.radius*self.angular_speed*c, 0.0),
                            (0.0, 0.0, self.angular_speed))


class MotionClock:
    """A trial's simulation clock; policy calls never advance it implicitly."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.seconds = 0.0
        self.steps = 0

    def advance(self, dt: float):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive and finite")
        self.steps += 1
        self.seconds += dt

