"""Geometric outcome checks that do not depend on a demonstration controller."""

from dataclasses import dataclass
import itertools
import math


def _rotate(vector, q):
    norm = math.sqrt(sum(v*v for v in q))
    if norm < 1e-12 or not math.isfinite(norm):
        raise ValueError("orientation quaternion must be finite and nonzero")
    x, y, z, w = (v/norm for v in q)
    vx, vy, vz = vector
    return ((1-2*(y*y+z*z))*vx + 2*(x*y-z*w)*vy + 2*(x*z+y*w)*vz,
            2*(x*y+z*w)*vx + (1-2*(x*x+z*z))*vy + 2*(y*z-x*w)*vz,
            2*(x*z-y*w)*vx + 2*(y*z+x*w)*vy + (1-2*(x*x+y*y))*vz)


def _sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


@dataclass(frozen=True)
class RigidState:
    position: tuple[float, float, float]
    size: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)


def inside_container(obj: RigidState, target: RigidState, *, floor_thickness: float,
                     wall_thickness: float, released: bool, tolerance_m: float = 0.0) -> bool:
    """The whole object lies in the container's interior, after release.

    `target.position` is the centre of its exterior bounding box. Dimensions include
    the floor and walls. This checks all eight oriented object corners, including Z.
    Stability is checked separately over time, in the target's moving frame.
    """
    if not released:
        return False
    if min(*obj.size, *target.size) <= 0 or min(floor_thickness, wall_thickness, tolerance_m) < 0:
        raise ValueError("sizes must be positive; thicknesses and tolerance must be nonnegative")
    inverse = tuple(-x for x in target.quaternion_xyzw[:3]) + (target.quaternion_xyzw[3],)
    hx, hy, hz = (s/2 for s in target.size)
    lower = (-hx+wall_thickness, -hy+wall_thickness, -hz+floor_thickness)
    upper = (hx-wall_thickness, hy-wall_thickness, hz)
    for signs in itertools.product((-1, 1), repeat=3):
        local = tuple(sign*side/2 for sign, side in zip(signs, obj.size))
        rotated = _rotate(local, obj.quaternion_xyzw)
        world = tuple(p+d for p, d in zip(obj.position, rotated))
        corner = _rotate(_sub(world, target.position), inverse)
        if not all(lo-tolerance_m <= v <= hi+tolerance_m for lo, v, hi in zip(lower, corner, upper)):
            return False
    return True


def stable_relative_to(obj: RigidState, target: RigidState, max_speed_m_s=0.02,
                       max_spin_rad_s=0.5) -> bool:
    """Account for translation and the target's angular velocity at the object's location."""
    rx, ry, rz = _sub(obj.position, target.position)
    wx, wy, wz = target.angular_velocity
    tangential = (wy*rz-wz*ry, wz*rx-wx*rz, wx*ry-wy*rx)
    expected = tuple(v+w for v, w in zip(target.velocity, tangential))
    speed = math.sqrt(sum(v*v for v in _sub(obj.velocity, expected)))
    spin = math.sqrt(sum(v*v for v in _sub(obj.angular_velocity, target.angular_velocity)))
    return speed <= max_speed_m_s and spin <= max_spin_rad_s


def seated_square_ring(ring: RigidState, shaft: RigidState, *, hole_width: float,
                       shaft_radius: float, seat_z: float, ring_thickness: float,
                       released: bool, height_tolerance: float = 0.004,
                       max_tilt_rad: float = math.radians(10)) -> bool:
    """Check a released washer seated on a shaft shoulder in the shaft frame.

    `seat_z` is the shoulder's upper face relative to the shaft body origin.
    The complete circular shaft must fit through both sides of the square hole;
    merely placing the washer next to, above, or below the shoulder earns no credit.
    RigidState positions here must be the authored body-frame origins.
    """
    if not released:
        return False
    if min(hole_width, shaft_radius, ring_thickness) <= 0 or height_tolerance < 0:
        raise ValueError("assembly dimensions must be positive")
    clearance = hole_width/2-shaft_radius
    if clearance <= 0:
        return False
    inverse = tuple(-v for v in shaft.quaternion_xyzw[:3])+(shaft.quaternion_xyzw[3],)
    delta = _rotate(_sub(ring.position, shaft.position), inverse)
    ring_normal = _rotate((0, 0, 1), ring.quaternion_xyzw)
    shaft_normal = _rotate((0, 0, 1), shaft.quaternion_xyzw)
    cosine = sum(x*y for x, y in zip(ring_normal, shaft_normal))
    # A plain washer is physically identical after being turned over.
    if abs(cosine) < math.cos(max_tilt_rad):
        return False
    along_shaft = sum(x*y for x, y in zip(_sub(ring.position, shaft.position), ring_normal))/cosine
    intersection = tuple(p+along_shaft*n for p, n in zip(shaft.position, shaft_normal))
    ring_inverse = tuple(-v for v in ring.quaternion_xyzw[:3])+(ring.quaternion_xyzw[3],)
    hole_offset = _rotate(_sub(intersection, ring.position), ring_inverse)
    # Conservative bounding circle of the shaft's oblique section through the ring.
    clearance = hole_width/2-shaft_radius/abs(cosine)
    return (max(abs(hole_offset[0]), abs(hole_offset[1])) <= clearance
            and abs(delta[2]-(seat_z+ring_thickness/2)) <= height_tolerance
            and stable_relative_to(ring, shaft))


class HeldCondition:
    """Require a condition to hold continuously for a documented duration."""

    def __init__(self, duration_s):
        if not math.isfinite(duration_s) or duration_s <= 0:
            raise ValueError("duration_s must be positive and finite")
        self.duration_s = duration_s
        self.reset()

    def reset(self):
        self.elapsed_s = 0.0

    def update(self, condition: bool, dt: float) -> bool:
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive and finite")
        self.elapsed_s = self.elapsed_s + dt if condition else 0.0
        return self.elapsed_s + 1e-9 >= self.duration_s

