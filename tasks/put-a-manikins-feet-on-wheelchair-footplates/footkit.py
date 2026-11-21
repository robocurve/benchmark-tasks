"""footkit: a folding manual wheelchair with flip-up footplates and a seated adult care manikin whose two lower legs
hang free at the knee, for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from footkit import Chair, Leg, Manikin, Plate, on_reset, on_step

Frames (every part starts with its axes along the world's unless a ``quat`` is given: x away from the robot, y left,
z up). The chair and the manikin face the robot (-x).

- :class:`Chair`: the wheelchair, fixed in place (its wheel locks are on). ``pos`` is the point on the floor under the
  middle of the seat's front edge.
- :class:`Manikin`: the seated body without its lower legs, fixed. ``pos`` is the point midway between its knees.
- :class:`Leg`: one lower leg and foot, a loose rigid part. ``pos`` is the middle of its knee, where a hinge joins it
  to the body. The shank runs down its -z; the toes point along its -x. ``side`` is +1 for the leg on the +y side
  (the manikin's right), -1 for the other.
- :class:`Plate`: one footplate, a loose rigid part hinged to the chair along its outer edge. ``pos`` is the middle of
  that hinge line, level with the plate's top; folded down, the plate reaches from there toward the chair's middle
  and its top is level. ``side`` as for the legs.

All shapes are convex pieces built from point sets in NumPy when the task file is read (``*_pieces()``), so the same
numbers serve the scene, the scoring and the offline checks. Isaac Sim is reached only when the scene is built.

The structure follows the kits used by tasks already in this repository (Box subclasses so the loader accepts the
parts, custom spawners, hooks installed through ``sys.meta_path``, passive joints authored afresh at every reset).
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Chair", "KitState", "Leg", "Manikin", "Plate", "chair_pieces", "env", "finger_points", "foot_marks",
           "gripper_points", "install", "knee_lift", "leg_pieces", "leg_plate_gap", "manikin_pieces", "on_reset",
           "on_step", "place", "plate_lift", "plate_pieces", "rest_lift", "to_local", "turn_x", "turn_y", "write_pose"]

IN = 0.0254
# -- the wheelchair (m), in the Chair's frame: origin on the floor under the middle of the seat's front edge ----------
# From a maker's brochure for a standard folding manual wheelchair: seat 18 x 16 in, seat-to-floor 19.5 in, back 16 in,
# 8 in casters, 24 in rear wheels, 34 in deep without front riggings. Everything else here is an estimate.
SEAT_W, SEAT_D, SEAT_H = 18 * IN, 16 * IN, 19.5 * IN
BACK_H = 16 * IN
CASTER_R, WHEEL_R = 4 * IN, 12 * IN
RAIL_Y = SEAT_W / 2              # the side frames' tubes (an estimate: under the seat's edges)
TUBE = 0.011                     # radius of the frame's tubes (an estimate)
HANGER_TOP = (0.0, 0.46)         # the footrest hanger's upper end: x, z (estimates)
HANGER_FOOT = (-0.130, 0.185)    # ... and its lower end, where the footplate's extension tube goes down
# -- a footplate: composite, flip-up (sizes are estimates; a maker's parts list gives 6 x 6 in for its small plate) ------
PLATE_W = 6 * IN                 # from its hinge toward the chair's middle
PLATE_D = 6 * IN                 # front to back
PLATE_T = 0.012
PLATE_MASS = 0.25                # an estimate
PLATE_UP = 95.0                  # deg: folded up it leans this far from level, 5 degrees past upright, on a stop
PLATE_DRAG = 0.02                # N m of friction in its hinge (an estimate): pushed past upright it falls by itself
HINGE_Y = 0.200                  # the hinge line, out from the chair's middle
PLATE_TOP = 0.160                # the folded-down plate's top above the floor (the footrest's length is adjustable)
PLATE_X = -0.176                 # the middle of the plate, front to back
# -- the manikin: an adult care manikin 58 in tall (the listed size); every size below is an estimate for that height ----
KNEE_Y = 0.110                   # each knee out from the midline
KNEE_X = -0.065                  # the knees are this far in front of the seat's front edge
KNEE_UP = 0.055                  # ... and this far above the seat (half the thigh's thickness)
# -- a lower leg, in the Leg's frame: origin at the knee, shank down -z, toes toward -x -------------------------------
SOLE_DOWN = 0.395                # knee to sole
ANKLE_H = 0.055                  # middle of the ankle above the sole
SHANK = SOLE_DOWN - ANKLE_H      # knee to the middle of the ankle
SOLE = -SOLE_DOWN
FOOT_L = 0.222                   # heel's back to the tip of the toes
FOOT_W = 0.084
HEEL_BACK = 0.044                # the heel's back is this far behind the shank's axis
TOE_X = HEEL_BACK - FOOT_L
LEG_MASS = 0.78                  # lower leg and foot: an estimate, 6.1 percent of the manikin's listed 28 lb
KNEE_R = 0.050
# the foot's sections: distance from the heel's back, half width, underside and top above the sole
FOOT_SECTIONS = ((0.000, 0.019, 0.008, 0.046), (0.017, 0.030, 0.000, 0.062), (0.055, 0.033, 0.000, 0.081),
                 (0.094, 0.037, 0.000, 0.062), (0.145, 0.042, 0.000, 0.043), (0.179, 0.042, 0.000, 0.031),
                 (0.208, 0.034, 0.001, 0.024), (0.222, 0.020, 0.005, 0.018))
# the shank's sections: z below the knee, radius
SHANK_SECTIONS = ((0.000, 0.050), (-0.080, 0.053), (-0.180, 0.045), (-0.270, 0.034), (-0.325, 0.031))
KNEE_FREE = (-20.0, 85.0)        # deg the lower leg can swing back past hanging, and forward (nearly straight)
KNEE_DRAG = 0.05                 # N m of friction in the knee (an estimate): the leg hangs, and swings when let go
HEEL_MID = 0.030                 # the middle of the heel's pad, forward of the heel's back
FOOT_MID = FOOT_L / 2            # the middle of the foot

SKIN, GOWN, HAIR = (0.84, 0.68, 0.58), (0.62, 0.76, 0.80), (0.55, 0.55, 0.56)
STEEL, BLACK, TYRE, GREY = (0.70, 0.71, 0.73), (0.10, 0.10, 0.11), (0.16, 0.16, 0.17), (0.45, 0.46, 0.48)
FRICTION = (0.7, 0.6)            # vinyl skin and composite against the grippers' rubber, and against each other
PALM_BACK = 0.065
FINGER_THICK = 0.012
N_RING = 12


# -- shapes (pure): every piece is a set of points whose convex hull is the piece -----------------------------------------

def _foot_ring(s: float, hw: float, bot: float, top: float) -> np.ndarray:
    """A foot section in the y-z plane: round on top, flatter underneath."""
    zc = bot + 0.45 * (top - bot)
    t = 2 * math.pi * (np.arange(N_RING) + 0.5) / N_RING
    z = np.where(np.sin(t) >= 0, zc + (top - zc) * np.sin(t), zc - (zc - bot) * np.abs(np.sin(t)) ** 0.55)
    return np.column_stack([np.full(N_RING, HEEL_BACK - s), hw * np.cos(t), SOLE + z])


def _disc(centre, r: float, axis: str = "z", ry: float | None = None, n: int = N_RING) -> np.ndarray:
    """A ring of points: radius ``r`` (``ry`` across, for an oval) round ``centre``, facing along ``axis``."""
    t = 2 * math.pi * np.arange(n) / n
    a, b = r * np.cos(t), (r if ry is None else ry) * np.sin(t)
    zero = np.zeros(n)
    cols = {"z": (a, b, zero), "x": (zero, a, b), "y": (a, zero, b)}[axis]
    return np.column_stack(cols) + np.asarray(centre, float)


def _rod(p0, p1, r0: float, r1: float | None = None) -> np.ndarray:
    """A round bar from ``p0`` to ``p1``."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    u = np.cross(d, (0.0, 0.0, 1.0) if abs(d[2]) < 0.9 else (1.0, 0.0, 0.0))
    u /= np.linalg.norm(u)
    v = np.cross(d, u)
    t = 2 * math.pi * np.arange(8) / 8
    ring = np.cos(t)[:, None] * u + np.sin(t)[:, None] * v
    return np.vstack([p0 + r0 * ring, p1 + (r0 if r1 is None else r1) * ring])


def _slab(lo, hi) -> np.ndarray:
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])], float)


def leg_pieces() -> list:
    """A lower leg and foot: (points, colour) pieces in the Leg's frame."""
    out = []
    rings = [_disc((0.0, 0.0, z), r) for z, r in SHANK_SECTIONS]
    out += [(np.vstack(pair), SKIN) for pair in zip(rings, rings[1:])]
    knee = [_disc((0.0, 0.0, z), r) for z, r in ((-0.040, 0.030), (-0.020, 0.046), (0.0, KNEE_R), (0.020, 0.046), (0.040, 0.030))]
    out.append((np.vstack(knee[:3]), SKIN))
    out.append((np.vstack(knee[2:]), SKIN))
    foot = [_foot_ring(*sec) for sec in FOOT_SECTIONS]
    out += [(np.vstack(pair), SKIN) for pair in zip(foot, foot[1:])]
    return out


def plate_pieces(side: float = 1.0) -> list:
    """A footplate in its own frame: a slab from the hinge line (y 0) toward the chair's middle, top at z 0."""
    y0, y1 = sorted((0.0, -side * PLATE_W))
    return [(_slab((-PLATE_D / 2, y0, -PLATE_T), (PLATE_D / 2, y1, 0.0)), BLACK)]


def knee_in_chair(side: float = 0.0) -> np.ndarray:
    """A knee's middle (or the point midway between the knees) in the Chair's frame."""
    return np.array([KNEE_X, side * KNEE_Y, SEAT_H + KNEE_UP])


def hinge_in_chair(side: float) -> np.ndarray:
    """The middle of a plate's hinge line, level with the folded-down plate's top, in the Chair's frame."""
    return np.array([PLATE_X, side * HINGE_Y, PLATE_TOP])


def chair_pieces() -> list:
    """The wheelchair in the Chair's frame: (points, colour, collides) pieces. Only the parts near the feet collide."""
    out = []
    for s in (-1.0, 1.0):
        y = s * RAIL_Y
        out.append((_rod((0.0, y, SEAT_H - 0.010), (SEAT_D, y, SEAT_H - 0.010), TUBE), STEEL, False))          # seat rail
        out.append((_rod((0.0, y, 0.235), (0.0, y, 0.700), TUBE), STEEL, True))                                # front post
        out.append((_rod((SEAT_D + 0.015, y, 0.305), (SEAT_D + 0.015, y, 0.930), TUBE), STEEL, False))         # back cane
        out.append((_rod((SEAT_D + 0.015, y, 0.930), (SEAT_D + 0.110, y, 0.930), 0.015), BLACK, False))        # push handle
        out.append((_rod((0.0, y, 0.300), (SEAT_D + 0.015, y, 0.300), TUBE), STEEL, False))                    # lower rail
        out.append((_rod((0.0, y, 0.700), (0.330, y, 0.700), TUBE), STEEL, False))                             # armrest tube
        out.append((_rod((0.330, y, 0.700), (SEAT_D + 0.015, y, 0.600), TUBE), STEEL, False))
        out.append((_slab((0.040, y - 0.025, 0.705), (0.300, y + 0.025, 0.740)), BLACK, False))                # armrest pad
        out.append((_rod((0.0, y, 0.235), (0.045, y, CASTER_R + 0.010), 0.009), STEEL, True))                  # caster fork
        out.append((np.vstack([_disc((0.045, y - 0.015, CASTER_R), CASTER_R, "y", n=16),
                               _disc((0.045, y + 0.015, CASTER_R), CASTER_R, "y", n=16)]), GREY, True))        # caster
        wy = s * (RAIL_Y + 0.055)
        hub = np.array([SEAT_D + 0.095, wy, WHEEL_R])
        n = 20
        for k in range(n):                                                                                       # tyre and rim
            a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
            pts = [hub + (r * math.cos(a), dy, r * math.sin(a)) for a in (a0, a1) for r in (WHEEL_R - 0.030, WHEEL_R)
                   for dy in (-0.014, 0.014)]
            out.append((np.array(pts), TYRE, False))
            rim = [hub + (r * math.cos(a), s * 0.040 + dy, r * math.sin(a)) for a in (a0, a1)
                   for r in (0.262, 0.274) for dy in (-0.006, 0.006)]
            out.append((np.array(rim), STEEL, False))                                                            # handrim
        for k in range(8):                                                                                       # spokes
            a = 2 * math.pi * k / 8
            out.append((_rod(hub, hub + ((WHEEL_R - 0.030) * math.cos(a), 0.0, (WHEEL_R - 0.030) * math.sin(a)), 0.004), STEEL, False))
        out.append((_rod(hub - (0.0, 0.03, 0.0), hub + (0.0, 0.03, 0.0), 0.022), GREY, False))                 # hub
        out.append((_rod((SEAT_D + 0.015, y, 0.305), hub, TUBE), STEEL, False))                                # axle plate
        # the footrest: hanger tube, extension tube, and the bracket that carries the plate's hinge
        top, foot = (HANGER_TOP[0], y, HANGER_TOP[1]), (HANGER_FOOT[0], y, HANGER_FOOT[1])
        out.append((_rod(top, foot, TUBE), STEEL, True))
        out.append((_rod(foot, (HANGER_FOOT[0], y, PLATE_TOP - 0.006), 0.009), STEEL, True))
        out.append((_slab((HANGER_FOOT[0] - 0.012, min(y, s * HINGE_Y), PLATE_TOP - 0.020),
                          (HANGER_FOOT[0] + 0.012, max(y, s * HINGE_Y), PLATE_TOP - 0.013)), STEEL, False))
    out.append((_slab((0.0, -RAIL_Y, SEAT_H - 0.012), (SEAT_D, RAIL_Y, SEAT_H)), BLACK, True))                  # seat sling
    out.append((_slab((SEAT_D + 0.010, -RAIL_Y, SEAT_H + 0.030), (SEAT_D + 0.020, RAIL_Y, SEAT_H + BACK_H)), BLACK, False))  # back
    for s in (-1.0, 1.0):                                                                                        # cross-brace
        out.append((_rod((0.200, s * RAIL_Y, SEAT_H - 0.010), (0.200, -s * RAIL_Y, 0.300), 0.009), STEEL, False))
    return out


def manikin_pieces() -> list:
    """The seated body without its lower legs, in the Manikin's frame (origin midway between the knees):
    (points, colour, collides) pieces."""
    out = []
    hip_x = 0.400
    mid = np.array([hip_x - 0.015, 0.0, 0.0])
    for s in (-1.0, 1.0):                                    # each thigh stops just short of its knee
        out.append((np.vstack([_disc((KNEE_R + 0.004, s * KNEE_Y, 0.0), KNEE_R, "x"),
                               _disc((hip_x, s * 0.090, 0.008), 0.063, "x")]), GOWN, True))
    levels = [(-0.050, 0.095, 0.165), (0.100, 0.090, 0.150), (0.300, 0.088, 0.160), (0.420, 0.075, 0.170), (0.465, 0.050, 0.095)]
    rings = [_disc(mid + (0.0, 0.0, z), a, "z", b) for z, a, b in levels]
    out += [(np.vstack(pair), GOWN, False) for pair in zip(rings, rings[1:])]
    out.append((np.vstack([_disc(mid + (0.0, 0.0, 0.455), 0.045), _disc(mid + (0.0, 0.0, 0.530), 0.045)]), SKIN, False))   # neck
    head = mid + (-0.012, 0.0, 0.630)
    hr = [(-0.105, 0.040, 0.036), (-0.070, 0.086, 0.070), (0.000, 0.100, 0.078), (0.060, 0.088, 0.070), (0.100, 0.045, 0.038)]
    rings = [_disc(head + (0.0, 0.0, z), a, "z", b) for z, a, b in hr]
    out += [(np.vstack(pair), SKIN if i < 2 else HAIR, False) for i, pair in enumerate(zip(rings, rings[1:]))]
    for s in (-1.0, 1.0):                                    # arms: shoulder to elbow to a hand on the thigh
        sh = mid + (0.0, s * 0.185, 0.410)
        el = mid + (-0.030, s * 0.180, 0.170)
        hand = mid + (-0.250, s * 0.085, 0.085)
        out.append((_rod(sh, el, 0.038, 0.033), GOWN, False))
        out.append((_rod(el, hand, 0.032, 0.025), SKIN, False))
        out.append((_rod(hand, hand + (-0.060, 0.0, -0.008), 0.028, 0.020), SKIN, False))
    return out


def turn_y(angle: float) -> tuple:
    """Quaternion (xyzw) of a turn of ``angle`` rad about y."""
    return (0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))


def turn_x(angle: float) -> tuple:
    """Quaternion (xyzw) of a turn of ``angle`` rad about x."""
    return (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))


def plate_quat(side: float, lift_deg: float) -> tuple:
    """A plate's attitude when it is lifted ``lift_deg`` from level (0 folded down, ``PLATE_UP`` folded up)."""
    return turn_x(-side * math.radians(lift_deg))


def hull_faces(points: np.ndarray, eps: float = 1e-7) -> list:
    """The faces of the convex hull of ``points`` (4 to about 40 of them): each a list of point indices, wound
    counter-clockwise seen from outside. Brute force over triples; coplanar triangles are merged into one polygon."""
    p = np.asarray(points, float)
    n = len(p)
    centre = p.mean(0)
    planes: dict = {}
    for i in range(n - 2):
        for j in range(i + 1, n - 1):
            e1 = p[j] - p[i]
            for k in range(j + 1, n):
                nrm = np.cross(e1, p[k] - p[i])
                size = float(np.linalg.norm(nrm))
                if size < 1e-12:
                    continue
                nrm = nrm / size
                d = (p - p[i]) @ nrm
                if d.max() > eps and d.min() < -eps:
                    continue
                if d.max() > eps:
                    nrm = -nrm
                key = tuple(np.round(np.r_[nrm, nrm @ p[i]], 5))
                planes.setdefault(key, (nrm, set()))[1].update((i, j, k))
    faces = []
    for nrm, idx in planes.values():
        idx = sorted(idx)
        c = p[idx].mean(0)
        u = p[idx[0]] - c
        u /= np.linalg.norm(u)
        v = np.cross(nrm, u)
        ang = [math.atan2((p[m] - c) @ v, (p[m] - c) @ u) for m in idx]
        order = [m for _, m in sorted(zip(ang, idx))]
        if nrm @ (c - centre) < 0:
            order = order[::-1]
        faces.append(order)
    return faces



class _Part(Box):
    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")


@dataclass(frozen=True)
class Chair(_Part):
    """The wheelchair, fixed. ``pos``: on the floor under the middle of the seat's front edge."""
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = 15.0


@dataclass(frozen=True)
class Manikin(_Part):
    """The seated body, fixed. ``pos``: midway between its knees."""
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = 10.0


@dataclass(frozen=True)
class Leg(_Part):
    """A loose lower leg. ``body`` names the :class:`Manikin` it hangs from."""
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = LEG_MASS
    side: float = 1.0
    body: str = "manikin"


@dataclass(frozen=True)
class Plate(_Part):
    """A footplate. ``chair`` names the :class:`Chair` it is hinged to."""
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = PLATE_MASS
    side: float = 1.0
    chair: str = "chair"


@dataclass(frozen=True)
class KitState(ObjectState):
    """A kit object as the simulator has it; ``pos`` is its own origin (see each class)."""

    joints: dict = field(default_factory=dict)


# -- geometry (pure) -------------------------------------------------------------------------------------------------

def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def place(pos, quat, local) -> np.ndarray:
    """A point (or points) given in a part's own frame, in the world."""
    return np.asarray(pos, float) + np.asarray(local, float) @ _rotation(quat).T


def to_local(point, pos, quat) -> np.ndarray:
    """A world point (or points) in a part's own frame."""
    return (np.asarray(point, float) - np.asarray(pos, float)) @ _rotation(quat)


def knee_lift(leg_quat, base_quat=(0.0, 0.0, 0.0, 1.0)) -> float:
    """How far a lower leg is swung forward from hanging (rad; the foot comes toward the robot as it grows)."""
    r = _rotation(base_quat).T @ _rotation(leg_quat)
    return math.atan2(float(r[0, 2]), float(r[2, 2]))


def plate_lift(plate_quat_, side: float, base_quat=(0.0, 0.0, 0.0, 1.0)) -> float:
    """How far a plate is lifted from level (rad; 0 folded down, about 1.66 folded up)."""
    r = _rotation(base_quat).T @ _rotation(plate_quat_)
    return -side * math.atan2(float(r[2, 1]), float(r[1, 1]))


def sole_point(along: float, up: float = 0.0) -> np.ndarray:
    """A point on the foot's midline, ``along`` m forward of the heel's back and ``up`` above the sole (Leg's frame)."""
    return np.array([HEEL_BACK - along, 0.0, SOLE + up])


def foot_marks(leg_pos, leg_quat, plate_pos, plate_quat_, side: float) -> dict:
    """Where the foot is over a plate, in the plate's own frame (m): ``heel`` the middle of the heel's pad and ``mid``
    the middle of the foot, both on the sole (x front to back from the plate's middle, y from the hinge line toward the
    chair's middle counted positive, z above the plate's top); ``low`` the lowest of the sole's points above the top."""
    def loc(p):
        v = to_local(place(leg_pos, leg_quat, p), plate_pos, plate_quat_)
        return np.array([v[0], -side * v[1], v[2]])
    heel, mid = loc(sole_point(HEEL_MID)), loc(sole_point(FOOT_MID))
    under = np.array([[HEEL_BACK - s, y, SOLE + b] for s, hw, b, _ in FOOT_SECTIONS for y in (-0.6 * hw, 0.0, 0.6 * hw)])
    low = float(to_local(place(leg_pos, leg_quat, under), plate_pos, plate_quat_)[:, 2].min())
    return {"heel": heel, "mid": mid, "low": low}


def over_plate(mark) -> bool:
    """Whether a point from :func:`foot_marks` lies over the plate's top."""
    return bool(abs(mark[0]) <= PLATE_D / 2 and 0.0 <= mark[1] <= PLATE_W)


_HULLS: dict = {}


def _leg_planes() -> list:
    """Each convex piece of the leg as outward planes (normals, offsets), for point-to-leg distances."""
    if "leg" not in _HULLS:
        planes = []
        for pts, _ in leg_pieces():
            ns, ds = [], []
            for f in hull_faces(pts):
                a, b, c = pts[f[0]], pts[f[1]], pts[f[2]]
                n = np.cross(b - a, c - a)
                n /= np.linalg.norm(n)
                ns.append(n)
                ds.append(float(n @ a))
            planes.append((np.array(ns), np.array(ds)))
        _HULLS["leg"] = planes
        _HULLS["leg_pts"] = np.vstack([p for p, _ in leg_pieces()])
    return _HULLS["leg"]


def _plate_samples(side: float) -> np.ndarray:
    key = ("plate", side)
    if key not in _HULLS:
        xs = np.linspace(-PLATE_D / 2, PLATE_D / 2, 9)
        ys = np.linspace(0.0, -side * PLATE_W, 9)
        _HULLS[key] = np.array([[x, y, z] for x in xs for y in ys for z in (0.0, -PLATE_T)])
    return _HULLS[key]


def leg_plate_gap(leg_pos, leg_quat, plate_pos, plate_quat_, side: float) -> float:
    """The gap between a leg and a plate (m, 0 when they touch or overlap): the smaller of the leg's surface points'
    distances to the plate's slab and of the plate's surface points' distances to the leg's convex pieces."""
    planes = _leg_planes()
    pts = to_local(place(leg_pos, leg_quat, _HULLS["leg_pts"]), plate_pos, plate_quat_)
    lo = np.array([-PLATE_D / 2, min(0.0, -side * PLATE_W), -PLATE_T])
    hi = np.array([PLATE_D / 2, max(0.0, -side * PLATE_W), 0.0])
    d1 = float(np.linalg.norm(np.maximum(0.0, np.maximum(lo - pts, pts - hi)), axis=1).min())
    q = to_local(place(plate_pos, plate_quat_, _plate_samples(side)), leg_pos, leg_quat)
    d2 = min(float(np.maximum(0.0, (q @ ns.T - ds).max(axis=1)).min()) for ns, ds in planes)
    return min(d1, d2)


def rest_lift(drop: float | None = None) -> float:
    """The lift (rad) at which a hanging leg's sole first meets a level surface ``drop`` m below the knee (default:
    the folded-down plate's top), coming down from in front: the heel lands first."""
    drop = (SEAT_H + KNEE_UP - PLATE_TOP) if drop is None else drop
    under = np.array([[HEEL_BACK - s, SOLE + b] for s, _, b, _ in FOOT_SECTIONS])
    for deg in np.arange(60.0, 0.0, -0.05):
        a = math.radians(deg)
        z = -under[:, 0] * math.sin(a) + under[:, 1] * math.cos(a)
        if float(-z.min()) >= drop:
            return a
    return 0.0


def segment_distance(p, a, b) -> float:
    p, a, b = (np.asarray(v, float) for v in (p, a, b))
    ab = b - a
    t = float(np.clip((p - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def leg_distance(points, leg_pos, leg_quat) -> float:
    """The smallest distance (m, at least 0) from any of ``points`` (world) to a leg's convex pieces."""
    planes = _leg_planes()
    q = to_local(np.atleast_2d(np.asarray(points, float)), leg_pos, leg_quat)
    return min(float(np.maximum(0.0, (q @ ns.T - ds).max(axis=1)).min()) for ns, ds in planes)


def _material(sim_utils, sf, df, combine="average"):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, friction_combine_mode=combine)
    except TypeError:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df)


# -- authoring helpers -----------------------------------------------------------------------------------------------

class _Author:
    def __init__(self, prim_path, translation, orientation):
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        self.sim_utils, self.Gf, self.UsdGeom = sim_utils, Gf, UsdGeom
        self.stage = get_current_stage()
        self.path = prim_path
        double = UsdGeom.XformOp.PrecisionDouble
        root = UsdGeom.Xform.Define(self.stage, prim_path)
        root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
        x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
        root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
        self.root = root.GetPrim()
        self.looks, self.mats = {}, {}

    def look(self, color, rough=0.6, metal=0.0) -> str:
        from yam_arena.station import linear_rgb

        key = (tuple(color), rough, metal)
        if key not in self.looks:
            path = f"{self.path}/Looks/v{len(self.looks)}"
            vis = self.sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal)
            vis.func(path, vis)
            self.looks[key] = path
        return self.looks[key]

    def mat(self, key, sf, df, combine="average") -> str:
        if key not in self.mats:
            path = f"{self.path}/Looks/phys_{key}"
            pm = _material(self.sim_utils, sf, df, combine)
            pm.func(path, pm)
            self.mats[key] = path
        return self.mats[key]

    def body(self, path, mass, lin_damp=0.05, ang_damp=0.1, iterations=None, fixed=False):
        from pxr import PhysxSchema, UsdPhysics

        prim = self.UsdGeom.Xform.Define(self.stage, path).GetPrim()
        rigid = UsdPhysics.RigidBodyAPI.Apply(prim)
        if fixed:
            rigid.CreateKinematicEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        rb.CreateLinearDampingAttr(lin_damp)
        rb.CreateAngularDampingAttr(ang_damp)
        if iterations:
            rb.CreateSolverPositionIterationCountAttr(int(iterations))
            rb.CreateSolverVelocityIterationCountAttr(4)
        return prim

    def hull(self, path, points, look, mat=None, offset=0.0008, patch=0.0):
        """A convex piece: drawn as its hull's faces, and a convex-hull collider if ``mat`` is given."""
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from pxr import PhysxSchema, UsdPhysics, Vt

        pts = np.asarray(points, float)
        faces = hull_faces(pts)
        mesh = self.UsdGeom.Mesh.Define(self.stage, path)
        mesh.CreatePointsAttr(Vt.Vec3fArray([self.Gf.Vec3f(*(float(v) for v in p)) for p in pts]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([len(f) for f in faces]))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for f in faces for i in f]))
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        prim = mesh.GetPrim()
        if look:
            bind_visual_material(str(prim.GetPath()), look, stage=self.stage)
        if mat:
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            cp.CreateContactOffsetAttr(offset)
            cp.CreateRestOffsetAttr(0.0)
            if patch:                                # soft foam meets a pad over a patch, which resists twisting
                cp.CreateTorsionalPatchRadiusAttr(float(patch))
                cp.CreateMinTorsionalPatchRadiusAttr(float(patch))
            bind_physics_material(str(prim.GetPath()), mat, stage=self.stage)
        return prim

    def pieces(self, stem, pieces, mat, rough=0.6, patch=0.0):
        for n, piece in enumerate(pieces):
            pts, color = piece[0], piece[1]
            solid = len(piece) < 3 or piece[2]
            self.hull(f"{stem}_{n}", pts, self.look(color, rough=rough), mat if solid else None, patch=patch)


# -- spawning ----------------------------------------------------------------------------------------------------------

_SPECS: dict = {}
_PATHS: dict = {}                      # where each kit object's prim was spawned


def _start(cfg, prim_path, translation, orientation, iterations=None, fixed=False):
    spec = _SPECS[cfg.kit_name]
    _PATHS[spec.name] = prim_path
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, float(spec.mass), iterations=iterations, fixed=fixed)
    return spec, a, a.mat("part", *FRICTION)


def _spawn_chair(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a, m = _start(cfg, prim_path, translation, orientation, fixed=True)
    a.pieces(f"{prim_path}/p", chair_pieces(), m, rough=0.45)
    return a.root


def _spawn_manikin(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a, m = _start(cfg, prim_path, translation, orientation, fixed=True)
    a.pieces(f"{prim_path}/p", manikin_pieces(), m, rough=0.7)
    return a.root


def _spawn_leg(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a, m = _start(cfg, prim_path, translation, orientation, iterations=32)
    a.pieces(f"{prim_path}/p", leg_pieces(), m, rough=0.7)
    return a.root


def _spawn_plate(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a, m = _start(cfg, prim_path, translation, orientation, iterations=32)
    a.pieces(f"{prim_path}/p", plate_pieces(spec.side), m, rough=0.8)
    return a.root


# -- joints ------------------------------------------------------------------------------------------------------------

def _joints() -> dict:
    """name -> (the fixed body's name, the loose part's name, the anchor in the fixed body, the hinge's axis, friction
    torque, limits in degrees about that axis)."""
    out = {}
    for name, spec in _SPECS.items():
        if name not in _PATHS:
            continue
        if isinstance(spec, Leg) and spec.body in _PATHS:
            out[f"knee_{name}"] = (spec.body, name, (0.0, spec.side * KNEE_Y, 0.0), "Y", KNEE_DRAG, KNEE_FREE)
        if isinstance(spec, Plate) and spec.chair in _PATHS:
            lim = (-PLATE_UP, 0.0) if spec.side > 0 else (0.0, PLATE_UP)
            out[f"hinge_{name}"] = (spec.chair, name, tuple(hinge_in_chair(spec.side)), "X", PLATE_DRAG, lim)
    return out


def _joint_path(name: str, fixed: str) -> str:
    return f"{_PATHS[fixed]}/{name}_joint"


def _author(name: str, spec: tuple) -> None:
    """Author one hinge: disabled first, its frames set, enabled last."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    fixed, loose, anchor, axis, drag, limits = spec
    # Both anchors are fixed points of their parts (the hinge sits at the loose part's own origin), so the hinge is the
    # same wherever the reset left the parts; its zero is the pose in which both parts' axes agree.
    stage = get_current_stage()
    joint = UsdPhysics.RevoluteJoint.Define(stage, _joint_path(name, fixed))
    joint.CreateJointEnabledAttr().Set(False)
    joint.CreateBody0Rel().SetTargets([_PATHS[fixed]])
    joint.CreateBody1Rel().SetTargets([_PATHS[loose]])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*(float(v) for v in anchor)))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateCollisionEnabledAttr().Set(False)
    joint.CreateAxisAttr().Set(axis)
    joint.CreateLowerLimitAttr().Set(float(limits[0]))
    joint.CreateUpperLimitAttr().Set(float(limits[1]))
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")     # dry friction: a stiff damper that tops out
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(0.0)
    drive.CreateDampingAttr().Set(1.0)
    drive.CreateMaxForceAttr().Set(float(drag))
    drive.CreateTargetVelocityAttr().Set(0.0)
    joint.CreateJointEnabledAttr().Set(True)       # last write, after the frames


def _rigid_object(spec, spawn):
    from isaaclab.sim.spawners.spawner_cfg import RigidObjectSpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class KitSpawnerCfg(RigidObjectSpawnerCfg):
        func: object = clone(spawn)
        kit_name: str = ""

    _SPECS[spec.name] = spec
    _PATHS.pop(spec.name, None)
    quat = tuple(float(v) for v in getattr(spec, "quat", (0.0, 0.0, 0.0, 1.0)))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=KitSpawnerCfg(kit_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=quat))


_SPAWNERS = {Chair: _spawn_chair, Manikin: _spawn_manikin, Leg: _spawn_leg, Plate: _spawn_plate}
_BOUNDS: dict = {}


def _points_of(spec) -> np.ndarray:
    if isinstance(spec, Chair):
        return np.vstack([p[0] for p in chair_pieces()])
    if isinstance(spec, Manikin):
        return np.vstack([p[0] for p in manikin_pieces()])
    if isinstance(spec, Plate):
        return np.vstack([p[0] for p in plate_pieces(spec.side)])
    return np.vstack([p[0] for p in leg_pieces()])


# -- run time ----------------------------------------------------------------------------------------------------------

def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    if hasattr(x, "numpy") and not hasattr(x, "detach"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _origin(environment) -> np.ndarray:
    return _np(environment.unwrapped.scene.env_origins[0])


def write_pose(environment, name: str, pos, quat_xyzw=(0.0, 0.0, 0.0, 1.0)) -> None:
    """Put a loose object at ``pos`` (relative to the env origin), ``quat_xyzw``, at rest."""
    import torch

    asset = environment.unwrapped.scene[name]
    dev = _t(asset.data.root_link_pose_w).device
    pose = torch.tensor([[*(np.asarray(pos, float) + _origin(environment)), *quat_xyzw]], dtype=torch.float32, device=dev)
    vel = torch.zeros((1, 6), dtype=torch.float32, device=dev)
    for stem, kw in (("write_root_pose_to_sim", {"root_pose": pose}), ("write_root_velocity_to_sim", {"root_velocity": vel})):
        fn = getattr(asset, f"{stem}_index", None)
        if fn is not None:
            fn(env_ids=[0], **kw)
        else:
            getattr(asset, stem)(*kw.values())


def _pose(environment, name: str):
    pose = _np(environment.unwrapped.scene[name].data.root_link_pose_w)[0]
    return pose[:3], pose[3:7]


def _disarm() -> None:
    """Switch every joint off (before a reset puts the parts back)."""
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    for name, spec in _joints().items():
        prim = stage.GetPrimAtPath(_joint_path(name, spec[0]))
        if prim and prim.IsValid():
            prim.GetAttribute("physics:jointEnabled").Set(False)


def _arm(environment) -> None:
    """Join the parts as the reset left them: each lower leg to its knee, each plate to its hinge."""
    for name, spec in _joints().items():
        _author(name, spec)


def _local_bounds(spec) -> tuple:
    if spec.name not in _BOUNDS:
        pts = _points_of(spec)
        _BOUNDS[spec.name] = (pts.min(0), pts.max(0))
    return _BOUNDS[spec.name]


def _kit_state(environment, spec) -> KitState:
    asset = environment.unwrapped.scene[spec.name]
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - _origin(environment), pose[3:7]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    lo, hi = _local_bounds(spec)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = corners @ _rotation(quat).T + pos
    return KitState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=pts.min(0), high=pts.max(0))


def finger_points(environment) -> list:
    """The grasp point of each gripper, left then right (world, relative to the env origin)."""
    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    origin = _origin(environment)
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        pts.append(pose[:3] + _rotation(pose[3:7]) @ np.array(TCP_OFFSET, float) - origin)
    return pts


def gripper_points(environment) -> list:
    """Points on each gripper that could touch something held between or beside its fingers, left gripper then right
    (world, relative to the env origin): along both fingers' inner and outer faces, from their tips back to the palm,
    and the palm's middle. The fingers' opening is read from their joints."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = _origin(environment)
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        rot = _rotation(pose[3:7])
        tcp = pose[:3] + rot @ np.array(TCP_OFFSET, float) - origin
        q, names = _np(robot.data.joint_pos)[0], list(robot.joint_names)
        pts = [tcp - PALM_BACK * rot[:, 2]]
        for side, joint in ((-1.0, "joint7"), (1.0, "joint8")):
            half = abs(float(q[names.index(joint)]))
            for depth in (PAD_TIP_PAST_TCP, 0.0, -PALM_BACK / 2, -PALM_BACK):
                for face in (0.0, FINGER_THICK):
                    pts.append(tcp + side * (half + face) * rot[:, 0] + depth * rot[:, 2])
        out.append(pts)
    return out


# -- hooking yam_arena -------------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_footkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        for kind, spawn in _SPAWNERS.items():
            if isinstance(spec, kind):
                return _rigid_object(spec, spawn)
        return stock_object(spec)

    def object_states(environment, names):
        mine = {n for n in names if isinstance(_SPECS.get(n), tuple(_SPAWNERS))}
        plain = [n for n in names if n not in mine]
        out = stock_states(environment, plain) if plain else {}
        for n in mine:
            out[n] = _kit_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._footkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_footkit_patched", False):
        return
    stock_step, stock_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return stock_step(self, *a, **k)

    def reset(self, *a, **k):
        # The joints have to be whole before the reset's settling steps and first picture: switch them off, let the
        # environment put the parts back, then author them afresh from where the parts are.
        base, saved, had = None, None, False
        try:
            self._ensure_sim(None)
            base = self._env.unwrapped
            env_reset = base.reset
            saved, had = vars(base).get("reset"), "reset" in vars(base)

            def reset_then_arm(*aa, **kk):
                _disarm()
                result = env_reset(*aa, **kk)
                _arm(self._env)
                return result

            base.reset = reset_then_arm
        except Exception as err:                     # noqa: BLE001
            print(f"[footkit] joints not re-armed around the reset: {err!r}", file=sys.stderr, flush=True)
            base = None
        try:
            obs = stock_reset(self, *a, **k)
        finally:
            if base is not None:
                if had:
                    base.reset = saved
                else:
                    del base.reset
        _ENV["env"] = self._env
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._footkit_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, patch = spec.loader, self.targets[fullname]
        stock_exec = loader.exec_module

        def exec_module(module):
            stock_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena (now for modules already imported, otherwise when they are). Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
