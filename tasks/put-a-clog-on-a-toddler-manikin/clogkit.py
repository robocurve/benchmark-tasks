"""clogkit: a seated toddler care manikin with one free-swinging lower leg, and a foam clog with a pivoting heel strap,
for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from clogkit import Clog, Leg, Manikin, Strap, on_reset, on_step

Frames (every part starts with its axes along the world's: x away from the robot, y left, z up):

- :class:`Manikin`: the seated body, fixed in place. ``pos`` is the middle of its right knee. It faces the robot (-x).
  Its left lower leg hangs from the left knee and wears the other clog; both are part of the fixed body.
- :class:`Leg`: the right lower leg and foot, one loose rigid part. ``pos`` is the middle of the knee, where a hinge
  (``knee``) joins it to the body. The shank runs down its -z; the toes point along its -x.
- :class:`Clog`: ``pos`` is the inside toe end of its footbed (the surface the sole rests on); its x runs back to the
  heel, its z up.
- :class:`Strap`: the clog's heel strap, a U of round foam. ``pos`` is the middle of its pivot axis; its x runs along
  its arms to the crossbar. A hinge (``pivot``) joins it to the clog.

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

__all__ = ["Clog", "KitState", "Leg", "Manikin", "Strap", "bar_in_clog", "clog_pieces", "env", "finger_points", "fit",
           "gripper_points", "install", "knee_flex", "leg_pieces", "manikin_pieces", "on_reset", "on_step", "place",
           "roof", "strap_pieces", "to_local", "turn_y", "width", "write_pose"]

# -- the leg (m), in the Leg's frame: origin at the knee, shank down -z, toes toward -x ------------------------------------
SHANK = 0.190                 # knee to the middle of the ankle
ANKLE_H = 0.035               # middle of the ankle above the sole
SOLE = -(SHANK + ANKLE_H)     # z of the sole
FOOT_L = 0.130                # heel's back to the tip of the toes
HEEL_BACK = 0.032             # the heel's back is this far behind the shank's axis
TOE_X = HEEL_BACK - FOOT_L    # x of the tip of the toes (-0.098)
LEG_MASS = 0.35               # lower leg and foot (an estimate for a vinyl care manikin of this size)
KNEE_R = 0.033
# the foot's sections: distance from the heel's back, half width, underside and top above the sole
FOOT_SECTIONS = ((0.000, 0.012, 0.006, 0.034), (0.010, 0.019, 0.000, 0.046), (0.032, 0.021, 0.000, 0.060),
                 (0.055, 0.024, 0.000, 0.046), (0.085, 0.027, 0.000, 0.032), (0.105, 0.027, 0.000, 0.023),
                 (0.122, 0.022, 0.001, 0.018), (0.130, 0.013, 0.004, 0.013))
# the shank's sections: z below the knee, radius
SHANK_SECTIONS = ((0.000, 0.031), (-0.045, 0.032), (-0.100, 0.027), (-0.155, 0.0205), (-0.180, 0.0195))
KNEE_FREE = (-8.0, 115.0)     # deg the knee can straighten past hanging, and bend back
KNEE_DRAG = 0.03              # N m: the knee hinge's friction (an estimate; the hanging leg's weight overcomes it
                              # beyond about 5 degrees, so the leg swings but comes to rest)

# -- the clog, in the Clog's frame: origin on the footbed at the inside toe end, x back to the heel ----------------------
LIN = 0.148                   # inside length
WALL = 0.005                  # foam wall
SOLE_T = 0.015                # footbed above the ground
VAMP = 0.070                  # the closed front (vamp) reaches this far back; behind it the clog is open on top
ROOF_TOE, ROOF_EDGE = 0.030, 0.058   # inside height under the vamp at the toes, and at the vamp's back edge
COLLAR = 0.022                # side walls behind the vamp
RIM = 0.014                   # heel rim
WIDTH_AT = ((0.000, 0.016), (0.012, 0.026), (0.035, 0.031), (0.070, 0.031), (0.110, 0.028), (0.148, 0.025))  # x, half width
CLOG_MASS = 0.045             # an estimate for a size 6 toddler clog of closed-cell foam
PIVOT = (0.090, 0.024)        # the strap's pivot: x, and height above the footbed
# -- the heel strap, in the Strap's frame: origin on the pivot axis, x along the arms -------------------------------------
STRAP_R = 0.062               # pivot to the middle of the crossbar
ROD = 0.0035                  # radius of the strap's round section
STRAP_Y = 0.0395              # each arm's middle, out from the clog's midline
STRAP_MASS = 0.004
STRAP_DRAG = 0.012            # N m: the rivets' friction (an estimate; it holds the strap wherever it is left)
STRAP_REST = math.asin((PIVOT[1] - (RIM + ROD)) / STRAP_R)   # rad below level when the crossbar lies on the heel rim

# -- the body, in the Manikin's frame: origin at the right knee ----------------------------------------------------------
LEG_GAP = 0.140               # between the two knees
THIGH = 0.200                 # knee to hip
SKIN, VEST, FOAM, WORN = (0.86, 0.70, 0.60), (0.92, 0.92, 0.88), (0.16, 0.50, 0.72), (0.16, 0.50, 0.72)

FRICTION = (0.7, 0.6)         # vinyl skin against foam
FOAM_PATCH = 0.006            # radius of the patch over which the clog's foam meets a gripper pad (an estimate)
PALM_BACK = 0.065
FINGER_THICK = 0.012
N_RING = 12


def width(x: float) -> float:
    """The clog's inside half width at ``x`` (m back from the toe end)."""
    xs, ws = zip(*WIDTH_AT)
    return float(np.interp(x, xs, ws))


def roof(x: float) -> float:
    """The inside height under the vamp at ``x``."""
    return ROOF_TOE + (ROOF_EDGE - ROOF_TOE) * min(1.0, max(0.0, (x - 0.010) / (VAMP - 0.010)))


# -- shapes (pure): every piece is a set of points whose convex hull is the piece -----------------------------------------

def _foot_ring(s: float, hw: float, bot: float, top: float) -> np.ndarray:
    """A foot section in the y-z plane: round on top, flatter underneath."""
    zc = bot + 0.45 * (top - bot)
    t = 2 * math.pi * (np.arange(N_RING) + 0.5) / N_RING
    z = np.where(np.sin(t) >= 0, zc + (top - zc) * np.sin(t), zc - (zc - bot) * np.abs(np.sin(t)) ** 0.55)
    return np.column_stack([np.full(N_RING, HEEL_BACK - s), hw * np.cos(t), SOLE + z])


def _disc(centre, r: float, axis: str = "z", ry: float | None = None) -> np.ndarray:
    """A ring of points: radius ``r`` (``ry`` across, for an oval) round ``centre``, facing along ``axis``."""
    t = 2 * math.pi * np.arange(N_RING) / N_RING
    a, b = r * np.cos(t), (r if ry is None else ry) * np.sin(t)
    zero = np.zeros(N_RING)
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


def leg_pieces() -> list:
    """The lower leg and foot: (points, colour) pieces in the Leg's frame."""
    out = []
    rings = [_disc((0.0, 0.0, z), r) for z, r in SHANK_SECTIONS]
    out += [(np.vstack(pair), SKIN) for pair in zip(rings, rings[1:])]
    knee = [_disc((0.0, 0.0, z), r) for z, r in ((-0.026, 0.020), (-0.012, 0.031), (0.0, KNEE_R), (0.012, 0.031), (0.026, 0.020))]
    out.append((np.vstack(knee[:3]), SKIN))
    out.append((np.vstack(knee[2:]), SKIN))
    foot = [_foot_ring(*sec) for sec in FOOT_SECTIONS]
    out += [(np.vstack(pair), SKIN) for pair in zip(foot, foot[1:])]
    return out


def clog_pieces() -> list:
    """The clog without its strap: (points, colour) pieces in the Clog's frame."""
    out = []
    xs = [x for x, _ in WIDTH_AT]
    # the sole: slabs between the width stations, and a nose in front of the toe end
    def slab(x):
        w = width(x) + WALL
        return np.array([[x, s * w, z] for s in (-1.0, 1.0) for z in (0.0, -SOLE_T + 0.003)]
                        + [[x, s * (w - 0.003), -SOLE_T] for s in (-1.0, 1.0)])
    stations = [slab(x) for x in xs]
    out += [(np.vstack(pair), FOAM) for pair in zip(stations, stations[1:])]
    nose = slab(0.0).copy()
    nose[:, 0] = -WALL
    nose[:, 1] *= 0.8
    out.append((np.vstack([nose, slab(0.0)]), FOAM))
    back = slab(LIN).copy()
    back[:, 0] = LIN + WALL
    out.append((np.vstack([slab(LIN), back]), FOAM))
    # the vamp: an arched shell from the toe end back to VAMP, in wedges
    vx = [0.0, 0.012, 0.035, 0.055, VAMP]
    k = 6

    def arch(x, grow):
        t = math.pi * np.arange(k + 1) / k
        return np.column_stack([np.full(k + 1, x), (width(x) + grow) * np.cos(t), (roof(x) + grow) * np.sin(t)])
    for x0, x1 in zip(vx, vx[1:]):
        i0, o0, i1, o1 = arch(x0, 0.0), arch(x0, WALL), arch(x1, 0.0), arch(x1, WALL)
        for j in range(k):
            out.append((np.vstack([i0[j:j + 2], o0[j:j + 2], i1[j:j + 2], o1[j:j + 2]]), FOAM))
    # the toe cap closing the front of the vamp
    front = arch(0.0, WALL)
    cap = front.copy()
    cap[:, 0] = -WALL
    cap[:, 1] *= 0.8
    cap[:, 2] *= 0.8
    out.append((np.vstack([front, cap]), FOAM))
    # the open back: side walls falling from the vamp's edge to the heel rim, and the rim across the back
    bx = [VAMP, PIVOT[0], 0.110, 0.130, LIN]
    hs = [0.030, COLLAR, 0.018, RIM, RIM]
    for (x0, h0), (x1, h1) in zip(zip(bx, hs), zip(bx[1:], hs[1:])):
        for s in (-1.0, 1.0):
            pts = [[x, s * (width(x) + g), z] for x, h in ((x0, h0), (x1, h1)) for g in (0.0, WALL) for z in (0.0, h)]
            out.append((np.array(pts), FOAM))
    w = width(LIN) + WALL
    out.append((np.array([[x, y, z] for x in (LIN, LIN + WALL) for y in (-w, w) for z in (0.0, RIM)]), FOAM))
    return out


def strap_pieces() -> list:
    """The heel strap: two arms and a crossbar, in the Strap's frame."""
    out = []
    for s in (-1.0, 1.0):
        out.append((_rod((0.0, s * STRAP_Y, 0.0), (STRAP_R, s * STRAP_Y, 0.0), ROD), FOAM))
        out.append((_rod((0.0, s * (STRAP_Y - 0.004), 0.0), (0.0, s * (STRAP_Y + 0.001), 0.0), 0.006), (0.25, 0.27, 0.30)))  # rivet
    out.append((_rod((STRAP_R, -STRAP_Y, 0.0), (STRAP_R, STRAP_Y, 0.0), ROD), FOAM))
    return out


def turn_y(angle: float) -> tuple:
    """Quaternion (xyzw) of a turn of ``angle`` rad about y."""
    return (0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))


def _rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def worn_clog_origin() -> np.ndarray:
    """Where a clog's origin is, in the Leg's frame, when it is on the foot: toes 4 mm from the end, sole on the footbed."""
    return np.array([TOE_X - 0.004, 0.0, SOLE - 0.0005])


def manikin_pieces() -> list:
    """The fixed body, in the Manikin's frame: thighs, hips, trunk, head, arms, and the left lower leg wearing its clog."""
    out = []
    left = np.array([0.0, -LEG_GAP, 0.0])
    mid = np.array([THIGH + 0.015, -LEG_GAP / 2, 0.0])
    for off in (np.zeros(3), left):
        # each thigh stops just short of its knee, which belongs to the lower leg
        out.append((np.vstack([_disc(off + (KNEE_R + 0.003, 0.0, 0.0), KNEE_R, "x"), _disc(off + (THIGH, 0.0, 0.004), 0.040, "x")]), SKIN))
    # trunk: ovals up from the seat
    levels = [(-0.036, 0.075, 0.112), (0.060, 0.072, 0.105), (0.170, 0.066, 0.098), (0.235, 0.058, 0.100), (0.262, 0.036, 0.060)]
    rings = [_disc(mid + (0.0, 0.0, z), a, "z", b) for z, a, b in levels]
    out += [(np.vstack(pair), VEST) for pair in zip(rings, rings[1:])]
    out.append((np.vstack([_disc(mid + (0.0, 0.0, 0.255), 0.028), _disc(mid + (0.0, 0.0, 0.300), 0.028)]), SKIN))   # neck
    head = mid + (-0.008, 0.0, 0.372)
    hr = [(-0.082, 0.030, 0.026), (-0.055, 0.066, 0.058), (0.000, 0.084, 0.072), (0.050, 0.070, 0.060), (0.080, 0.034, 0.030)]
    rings = [_disc(head + (0.0, 0.0, z), a, "z", b) for z, a, b in hr]
    out += [(np.vstack(pair), SKIN) for pair in zip(rings, rings[1:])]
    for s in (-1.0, 1.0):                                   # arms: shoulder to elbow to a hand in the lap
        sh = mid + (0.0, s * 0.112, 0.225)
        el = mid + (-0.035, s * 0.122, 0.095)
        hand = mid + (-0.120, s * 0.045, 0.052)
        out.append((_rod(sh, el, 0.024, 0.021), VEST))
        out.append((_rod(el, hand, 0.021, 0.018), SKIN))
        out.append((_rod(hand, hand + (-0.034, 0.0, -0.004), 0.019, 0.015), SKIN))
    # the left lower leg, hanging, with the other clog on its foot
    out += [(pts + left, col) for pts, col in leg_pieces()]
    on = left + worn_clog_origin()
    out += [(pts + on, WORN) for pts, _ in clog_pieces()]
    strap = on + (PIVOT[0], 0.0, PIVOT[1])
    out += [(pts @ _rot_y(STRAP_REST).T + strap, col if col != FOAM else WORN) for pts, col in strap_pieces()]
    return out


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
class Manikin(_Part):
    """The seated body, fixed. ``pos``: the middle of its right knee. ``leg`` names its loose right lower :class:`Leg`."""
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = 2.0
    leg: str = "leg"


@dataclass(frozen=True)
class Leg(_Part):
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = LEG_MASS


@dataclass(frozen=True)
class Clog(_Part):
    """A loose clog. ``strap`` names its :class:`Strap`."""
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = CLOG_MASS
    strap: str = "strap"


@dataclass(frozen=True)
class Strap(_Part):
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = STRAP_MASS


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


def _quat(rot) -> tuple:
    m = np.asarray(rot, float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    else:
        i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(1e-12, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2
        q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
        q[i], q[j], q[k] = 0.25 * s, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s
    return tuple(float(v) for v in q)


def place(pos, quat, local) -> np.ndarray:
    """A point given in a part's own frame, in the world."""
    return np.asarray(pos, float) + _rotation(quat) @ np.asarray(local, float)


def to_local(point, pos, quat) -> np.ndarray:
    return _rotation(quat).T @ (np.asarray(point, float) - np.asarray(pos, float))


def knee_flex(leg_quat) -> float:
    """How far the knee is bent back from hanging (rad; the foot swings away from the robot as it grows)."""
    r = _rotation(leg_quat)
    return math.atan2(float(-r[0, 2]), float(r[2, 2]))


def bar_in_clog(clog_pos, clog_quat, strap_pos, strap_quat) -> np.ndarray:
    """The middle of the strap's crossbar, in the clog's frame."""
    return to_local(place(strap_pos, strap_quat, (STRAP_R, 0.0, 0.0)), clog_pos, clog_quat)


def fit(clog_pos, clog_quat, leg_pos, leg_quat, strap_pos, strap_quat) -> dict:
    """How the foot sits in the clog, all in the clog's frame (m): ``toe`` the tip of the toes (x back from the toe end,
    y off the midline, z above the footbed); ``heel`` the heel's back at the sole; ``cross`` where the shank's axis
    comes up through the level of the strap's pivot (x, y); ``bar`` the crossbar's middle (x, z); ``ankle_up`` the
    middle of the ankle above the footbed; ``lean`` the angle between the sole and the footbed (rad)."""
    def loc(p):
        return to_local(place(leg_pos, leg_quat, p), clog_pos, clog_quat)
    toe, heel = loc((TOE_X, 0.0, SOLE)), loc((HEEL_BACK, 0.0, SOLE))
    ankle, knee = loc((0.0, 0.0, -SHANK)), loc((0.0, 0.0, 0.0))
    d = knee - ankle
    if abs(d[2]) < 1e-6:
        cross = np.array([math.inf, math.inf])
    else:
        cross = (ankle + d * (PIVOT[1] - ankle[2]) / d[2])[:2]
    bar = bar_in_clog(clog_pos, clog_quat, strap_pos, strap_quat)
    up = _rotation(clog_quat).T @ _rotation(leg_quat) @ np.array([0.0, 0.0, 1.0])
    return {"toe": toe, "heel": heel, "cross": cross, "bar": np.array([bar[0], bar[2]]), "ankle_up": float(ankle[2]),
            "lean": math.acos(max(-1.0, min(1.0, float(up[2])))), "shank_up": bool(d[2] > 0)}


def segment_distance(p, a, b) -> float:
    p, a, b = (np.asarray(v, float) for v in (p, a, b))
    ab = b - a
    t = float(np.clip((p - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


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
        for n, (pts, color) in enumerate(pieces):
            self.hull(f"{stem}_{n}", pts, self.look(color, rough=rough), mat, patch=patch)


# -- spawning ----------------------------------------------------------------------------------------------------------

_SPECS: dict = {}
_PATHS: dict = {}                      # where each kit object's prim was spawned


def _start(cfg, prim_path, translation, orientation, iterations=None, fixed=False):
    spec = _SPECS[cfg.kit_name]
    _PATHS[spec.name] = prim_path
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, float(spec.mass), iterations=iterations, fixed=fixed)
    return spec, a, a.mat("part", *FRICTION)


def _spawn_manikin(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a, m = _start(cfg, prim_path, translation, orientation, fixed=True)
    a.pieces(f"{prim_path}/p", manikin_pieces(), m)
    return a.root


def _spawn_leg(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a, m = _start(cfg, prim_path, translation, orientation, iterations=32)
    a.pieces(f"{prim_path}/p", leg_pieces(), m)
    return a.root


def _spawn_clog(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a, m = _start(cfg, prim_path, translation, orientation, iterations=32)
    a.pieces(f"{prim_path}/p", clog_pieces(), m, rough=0.8, patch=FOAM_PATCH)
    return a.root


def _spawn_strap(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a, m = _start(cfg, prim_path, translation, orientation, iterations=32)
    a.pieces(f"{prim_path}/p", strap_pieces(), m, rough=0.8)
    return a.root


# -- joints ------------------------------------------------------------------------------------------------------------
# name -> (body 0's kind, body 1's kind, the anchor in body 0, friction torque, limits in degrees about +y or None,
#          whether the two bodies still collide)
_JOINT_KINDS = {"knee": ("manikin", "leg", (0.0, 0.0, 0.0), KNEE_DRAG, (-KNEE_FREE[1], -KNEE_FREE[0]), False),
                "pivot": ("clog", "strap", (PIVOT[0], 0.0, PIVOT[1]), STRAP_DRAG, None, True)}


def _named() -> dict:
    kinds = {Manikin: "manikin", Leg: "leg", Clog: "clog", Strap: "strap"}
    return {kinds[type(s)]: n for n, s in _SPECS.items() if type(s) in kinds and n in _PATHS}


def _joint_path(name: str) -> str:
    return f"{_PATHS[_named()[_JOINT_KINDS[name][0]]]}/{name}_joint"


def _author(name: str, environment) -> bool:
    """Author hinge ``name`` (about the first body's y): disabled first, its frames set, enabled last."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    kind0, kind1, anchor, drag, limits, collide = _JOINT_KINDS[name]
    parts = _named()
    if kind0 not in parts or kind1 not in parts:
        return False
    # Both anchors are fixed points of their parts (the hinge sits at the second part's own origin), so the hinge is
    # the same wherever the reset left the parts; its zero is the pose in which both parts' axes agree.
    local1 = np.zeros(3)
    stage = get_current_stage()
    joint = UsdPhysics.RevoluteJoint.Define(stage, _joint_path(name))
    joint.CreateJointEnabledAttr().Set(False)
    joint.CreateBody0Rel().SetTargets([_PATHS[parts[kind0]]])
    joint.CreateBody1Rel().SetTargets([_PATHS[parts[kind1]]])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*(float(v) for v in anchor)))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*(float(v) for v in local1)))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateCollisionEnabledAttr().Set(bool(collide))
    joint.CreateAxisAttr().Set("Y")
    if limits is not None:
        joint.CreateLowerLimitAttr().Set(float(limits[0]))
        joint.CreateUpperLimitAttr().Set(float(limits[1]))
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")     # dry friction: a stiff damper that tops out
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(0.0)
    drive.CreateDampingAttr().Set(1.0)
    drive.CreateMaxForceAttr().Set(float(drag))
    drive.CreateTargetVelocityAttr().Set(0.0)
    joint.CreateJointEnabledAttr().Set(True)       # last write, after the frames
    return True


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


_SPAWNERS = {Manikin: _spawn_manikin, Leg: _spawn_leg, Clog: _spawn_clog, Strap: _spawn_strap}
_PIECES = {Manikin: manikin_pieces, Leg: leg_pieces, Clog: clog_pieces, Strap: strap_pieces}
_BOUNDS: dict = {}


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
    for name in _JOINT_KINDS:
        try:
            prim = stage.GetPrimAtPath(_joint_path(name))
        except KeyError:
            continue
        if prim and prim.IsValid():
            prim.GetAttribute("physics:jointEnabled").Set(False)


def _arm(environment) -> None:
    """Join the parts as the reset left them: the lower leg to the knee, the strap to its clog."""
    for name in _JOINT_KINDS:
        _author(name, environment)


def _local_bounds(spec) -> tuple:
    kind = type(spec)
    if kind not in _BOUNDS:
        pts = np.vstack([p for p, _ in _PIECES[kind]()])
        _BOUNDS[kind] = (pts.min(0), pts.max(0))
    return _BOUNDS[kind]


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
    if getattr(scene, "_clogkit_patched", False):
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
    scene._clogkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_clogkit_patched", False):
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
            print(f"[clogkit] joints not re-armed around the reset: {err!r}", file=sys.stderr, flush=True)
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
    cls._clogkit_patched = True


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
