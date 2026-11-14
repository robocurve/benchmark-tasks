"""gladkit: gladhand air couplers and air hoses for yam_arena task files.

A task file uses it like this (keep this file next to the task file):

    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    from gladkit import AirLine, Coupler, on_step, on_reset

Three things live here:

- the gladhand's geometry (:func:`gladhand_shapes`): a head with the seal face, a handle, a locking lug at the neck
  and a connector plate at the tip whose lip has a ramped underside, a detent and a stop. Every piece is its own
  convex collider, so the lug really slides under the opposite lip: nothing is scripted and no joint is made;
- :class:`Coupler`, a fixed gladhand (a trailer gladhand with a sprung seal, or a dummy coupler without one);
- :class:`AirLine`, one articulation: a hose (a chain of rigid capsule segments joined by three-axis joints with
  bending and twisting stiffness) anchored to a fixed fitting at one end, with a gladhand and its sprung seal on the
  other.

Gladhand frame (metres): origin at the centre of the metal seal face, +z out of the face toward the mating
coupler, the handle (and hose port) along -x, the connector plate at +x.

Importing this module hooks ``yam_arena.scene`` and the embodiment's step and reset without modifying yam_arena.
Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import Box, ObjectState

__all__ = ["AirLine", "Coupler", "GladState", "LineState", "air_line", "coupler", "measure", "on_reset", "on_step",
           "finger_points", "install"]

MM = 0.001

# -- gladhand dimensions (mm, gladhand frame) -----------------------------------------------------------------------
HEAD_R = 25.0  # the seal face is 50 mm across
HEAD_DEPTH = 30.0  # head: z -30 .. 0
HANDLE_X = (-125.0, -15.0)  # the grip: 30 mm thick, 34 mm wide
HANDLE_Y = (-17.0, 17.0)
HANDLE_Z = (-48.5, -18.5)
STUB_X = (-60.0, -15.0)  # a dummy coupler's short body
PORT = (-125.0, 0.0, -29.4)  # where the hose enters the handle
GRIP = (-70.0, 0.0, -33.5)  # middle of the handle: where a gripper holds it
WAIST_X = (-90.0, -50.0)  # finger recess: here the handle is 28 mm wide, between two shoulders
WAIST_Y = (-14.0, 14.0)

SEAL_R = 22.5  # 45 mm seal
SEAL_PROUD = 1.75  # an uncompressed seal stands this far above the face
SEAL_THICK = 6.0
SEAL_TRAVEL = 3.0
DUMMY_BOSS = 0.3  # a dummy coupler's plain steel face, in place of a seal

LUG_R = (24.0, 34.0)  # locking lug at the neck (theta 180 deg)
LUG_HALF_DEG = 10.0
LUG_Z = (-6.0, -1.0)  # its back face (z -6) bears on the opposite lip
LUG_BEVEL_DEG = 3.0
LUG_HOOK_R = 31.0  # outside this radius the lug stands 1.5 mm prouder: a hook that runs outside the lip's rim
HOOK_STEP = 1.5

PLATE_DEG = (-82.0, 16.0)  # connector plate at the tip (theta 0): a quarter-circle hook
PLATE_BASE_R = (24.0, 40.0)
PLATE_BASE_Z = (-6.0, 0.0)
WALL_R = (36.5, 40.0)  # 2.5 mm outside the lug: gladhands are a loose fit
LIP_R = (28.0, 40.0)
LIP_RIM_R = 29.5  # inside this radius the lip hangs HOOK_STEP lower: a rim that keeps the lug's hook from sliding in
LIP_TOP = 16.0
STOP_DEG = (10.5, 16.0)  # the lug's edge meets this wall when it is locked
STOP_R = (27.0, 40.0)
# The lip's underside, height above the face by angle: a short lead-in at the mouth, then one steady ramp down to a
# low point (the detent), then up 0.2 mm to the seat where a locked lug rests against the stop. The ramp starts
# squeezing the seals about 25 degrees into the turn, so a gladhand let go part-way stays put.
LIP_PROFILE = ((-82.0, 12.0), (-76.0, 10.8), (-12.0, 6.4), (-6.0, 6.6), (16.0, 6.6))

GAP_OPEN = 2 * SEAL_PROUD  # face-to-face gap with two uncompressed seals touching
GAP_LOCKED = LIP_PROFILE[-1][1] + LUG_Z[0]  # 0.6 mm: lug back on the seat
LOCK_ANGLE_DEG = STOP_DEG[0] - LUG_HALF_DEG  # 0.5 deg: where the lug meets the stop

BODY_MASS = 0.42  # kg; with the 0.03 kg seal, 0.45 kg
SEAL_MASS = 0.03
SEAL_RATE = 67000.0  # N/m per seal (two in series when two gladhands meet)
SEAL_DAMPING = 120.0

STEEL = (0.4, 0.3)  # static, dynamic friction: lugs, plates, bodies
RUBBER = (1.0, 0.8)  # seals and hoses

IRON = (0.20, 0.20, 0.22)
BLACK = (0.03, 0.03, 0.03)
BRASS = (0.72, 0.58, 0.25)
STEEL_GREY = (0.62, 0.63, 0.65)


def lip_height(theta_deg: float) -> float:
    """Height (mm) of the lip's underside above the face at ``theta_deg``."""
    xs, ys = zip(*LIP_PROFILE)
    return float(np.interp(theta_deg, xs, ys))


@dataclass(frozen=True)
class Shape:
    kind: str  # "box", "cyl" (axis z) or "hull"
    look: str  # "body", "iron", "seal", "brass"
    material: str  # "steel" or "rubber"
    centre: tuple = (0.0, 0.0, 0.0)  # box, cyl (m)
    size: tuple = (0.0, 0.0, 0.0)  # box (m)
    radius: float = 0.0  # cyl (m)
    height: float = 0.0  # cyl (m)
    points: tuple = ()  # hull vertices (m)


def _box(x, y, z, look, material="steel") -> Shape:
    c = tuple((a[0] + a[1]) / 2 * MM for a in (x, y, z))
    s = tuple((a[1] - a[0]) * MM for a in (x, y, z))
    return Shape("box", look, material, centre=c, size=s)


def _cyl(xy, z, radius, look, material="steel") -> Shape:
    return Shape("cyl", look, material, centre=(xy[0] * MM, xy[1] * MM, (z[0] + z[1]) / 2 * MM), radius=radius * MM,
                 height=(z[1] - z[0]) * MM)


def _wedge(r, t0, t1, zlo, zhi, look, centre_deg=0.0) -> Shape:
    """A slice of a ring between angles t0 and t1 (deg, about ``centre_deg``); ``zlo``/``zhi`` are (at t0, at t1)."""
    pts = []
    for t, lo, hi in ((t0, zlo[0], zhi[0]), (t1, zlo[1], zhi[1])):
        a = math.radians(centre_deg + t)
        for rr in r:
            for z in (lo, hi):
                pts.append((rr * math.cos(a) * MM, rr * math.sin(a) * MM, z * MM))
    return Shape("hull", look, "steel", points=tuple(pts))


def _slices(t0: float, t1: float, step: float, breaks=()) -> list[tuple[float, float]]:
    edges = sorted({t0, t1, *[b for b in breaks if t0 < b < t1], *np.arange(t0, t1, step).tolist()})
    return list(zip(edges[:-1], edges[1:]))


def gladhand_shapes(kind: str = "hose") -> list[Shape]:
    """Colliders of a gladhand body: ``"hose"`` (on a hose), ``"trailer"`` (fixed, with its bulkhead fitting) or
    ``"dummy"`` (a dummy coupler: a short steel block with the same lug and plate, and a plain face for a seal)."""
    look = "body"
    shapes = [_cyl((0, 0), (-HEAD_DEPTH, 0), HEAD_R, look)]
    if kind == "dummy":
        shapes.append(_box(STUB_X, HANDLE_Y, HANDLE_Z, look))
    else:  # the handle, with a finger recess between two shoulders
        shapes.append(_box((HANDLE_X[0], WAIST_X[0]), HANDLE_Y, HANDLE_Z, look))
        shapes.append(_box(WAIST_X, WAIST_Y, HANDLE_Z, look))
        shapes.append(_box((WAIST_X[1], HANDLE_X[1]), HANDLE_Y, HANDLE_Z, look))
    # Locking lug at the neck; its back edges are bevelled so it starts under the lip cleanly.
    h, b = LUG_HALF_DEG, LUG_BEVEL_DEG
    for radii, back in (((LUG_R[0], LUG_HOOK_R), LUG_Z[0] + HOOK_STEP), ((LUG_HOOK_R, LUG_R[1]), LUG_Z[0])):
        pts = []
        for t, z in ((-h, LUG_Z[1]), (h, LUG_Z[1]), (-h, back + 1.5), (h, back + 1.5), (-h + b, back), (h - b, back)):
            a = math.radians(180.0 + t)
            pts += [(rr * math.cos(a) * MM, rr * math.sin(a) * MM, z * MM) for rr in radii]
        shapes.append(Shape("hull", "iron", "steel", points=tuple(pts)))
    # Connector plate at the tip: base, outer wall, lip (ramped underside) and stop.
    for t0, t1 in _slices(*PLATE_DEG, 8.0):
        shapes.append(_wedge(PLATE_BASE_R, t0, t1, (PLATE_BASE_Z[0],) * 2, (PLATE_BASE_Z[1],) * 2, "iron"))
        shapes.append(_wedge(WALL_R, t0, t1, (0.0, 0.0), (LIP_TOP, LIP_TOP), "iron"))
    for t0, t1 in _slices(*PLATE_DEG, 4.0, breaks=[p[0] for p in LIP_PROFILE]):
        low = (lip_height(t0), lip_height(t1))
        shapes.append(_wedge((LIP_RIM_R, LIP_R[1]), t0, t1, low, (LIP_TOP, LIP_TOP), "iron"))
        shapes.append(_wedge((LIP_R[0], LIP_RIM_R), t0, t1, (low[0] - HOOK_STEP, low[1] - HOOK_STEP),
                             (LIP_TOP, LIP_TOP), "iron"))
    shapes.append(_wedge(STOP_R, *STOP_DEG, (0.0, 0.0), (LIP_TOP, LIP_TOP), "iron"))
    if kind == "dummy":
        shapes.append(_cyl((0, 0), (0, DUMMY_BOSS), SEAL_R, "iron"))
    if kind == "trailer":
        # 4 cm bulkhead fitting from the port end of the body back to the nose plate.
        shapes.append(_cyl((-105.0, 0.0), (HANDLE_Z[0] - 40.0, HANDLE_Z[0]), 11.0, "brass"))
    return shapes


SEAL_SHAPE = _cyl((0, 0), (-SEAL_THICK / 2, SEAL_THICK / 2), SEAL_R, "seal", "rubber")
SEAL_REST = (0.0, 0.0, (SEAL_PROUD - SEAL_THICK / 2) * MM)  # seal body origin in the gladhand frame, uncompressed

# Boxes around a gladhand in its own frame (m), for "is a gripper clear of it" checks.
GLADHAND_BOXES = (
    (np.array([-0.125, -0.017, -0.0485]), np.array([-0.015, 0.017, -0.0185])),
    (np.array([-0.025, -0.025, -0.030]), np.array([0.040, 0.025, 0.016])),
)


# -- rotations ------------------------------------------------------------------------------------------------------


def quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    r = np.asarray(r, float)
    tr = np.trace(r)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w, x, y, z = (r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w, x, y, z = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w, x, y, z = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s
    return (float(x), float(y), float(z), float(w))


def rot_from_xyzw(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


FLIP_X = np.diag([1.0, -1.0, -1.0])  # half a turn about x: a mating gladhand faces the other way


def mated_pose(coupler_pos, coupler_rot, body_deg: float, gap_mm: float = GAP_LOCKED):
    """World pose (position, rotation) of a gladhand mated face to face with a coupler, its body turned
    ``body_deg`` about the coupler's axis from the coupler's +x (the lock stop is at LOCK_ANGLE_DEG; a gladhand
    starts its turn at -90) and its face ``gap_mm`` from the coupler's."""
    cr = np.asarray(coupler_rot, float)
    rot = cr @ rot_z(math.radians(body_deg + 180.0)) @ FLIP_X
    return np.asarray(coupler_pos, float) + cr @ np.array([0.0, 0.0, gap_mm * MM]), rot


def measure(coupler_pos, coupler_rot, pos, rot) -> dict:
    """A hose gladhand (``pos``, ``rot``) measured against a coupler: the gap between the two faces (mm), how far
    off the coupler's axis it is (mm), its tilt from face to face (deg), its body angle about the coupler's axis
    (deg; LOCK_ANGLE_DEG at the stop, -90 where a turn starts) and whether its lug is fully under the lip."""
    cr = np.asarray(coupler_rot, float)
    p = cr.T @ (np.asarray(pos, float) - np.asarray(coupler_pos, float))
    rel = cr.T @ np.asarray(rot, float)
    gap = float(p[2]) / MM
    lateral = float(np.hypot(p[0], p[1])) / MM
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -rel[2, 2]))))
    body = rel @ np.array([-1.0, 0.0, 0.0])
    angle = math.degrees(math.atan2(body[1], body[0]))
    lo, hi = angle - LUG_HALF_DEG, angle + LUG_HALF_DEG
    back = gap - LUG_Z[0]  # the lug's back face, above the coupler's face
    under = bool(
        lateral <= 3.0
        and tilt <= 5.0
        and lo >= PLATE_DEG[0]
        and hi <= STOP_DEG[0] + 1.0
        and back <= max(lip_height(lo), lip_height(hi)) + 0.5
        and gap >= 0.0
    )
    return {"gap_mm": gap, "lateral_mm": lateral, "tilt_deg": tilt, "angle_deg": angle, "lug_under_lip": under}


# -- task-file objects ----------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Coupler(Box):
    """A fixed gladhand: ``kind`` "trailer" (sprung seal) or "dummy" (plain face). ``pos`` is the centre of its
    face and ``rot`` its rotation (rows of a 3x3 matrix, gladhand frame to world). Build one with :func:`coupler`.
    It subclasses Box only so yam_arena's task-file checks accept it."""

    kind: str = "trailer"
    rot: tuple = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    @property
    def free(self) -> bool:
        return False

    @property
    def rotation(self) -> np.ndarray:
        return np.array(self.rot, float)


def coupler(name: str, pos, rot, kind: str, color) -> Coupler:
    return Coupler(name=name, pos=tuple(float(v) for v in pos), size=(0.164, 0.05, 0.06), color=tuple(color),
                   kind=kind, rot=tuple(tuple(float(v) for v in row) for row in np.asarray(rot, float)))


@dataclass(frozen=True)
class AirLine(Box):
    """An air hose from a fixed fitting to a gladhand. Build one with :func:`air_line`.

    The hose starts along a planned curve in the horizontal plane through the fitting: ``bends`` are the starting
    bend angles (rad, about the vertical) at its joints. ``pos`` is the fitting end of the hose; ``yaw`` the
    direction the hose leaves it."""

    length: float = 1.0
    radius: float = 0.0095
    segments: int = 20
    hose_mass: float = 0.25  # kg, whole hose
    bend_stiffness: float = 0.6  # N·m/rad per joint: a flexural rigidity of 0.03 N·m² over 5 cm segments
    bend_damping: float = 0.05
    twist_stiffness: float = 0.5
    twist_damping: float = 0.05
    armature: float = 2e-5
    max_bend: float = 1.0  # rad per joint
    max_twist: float = 0.6
    bends: tuple = ()
    seal_down: bool = True  # the gladhand starts with its seal face down (mated on a face-up coupler)
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    @property
    def free(self) -> bool:
        return False

    @property
    def seg_len(self) -> float:
        return self.length / self.segments

    def start_layout(self):
        """Starting world poses: (segment centres (n, 3), segment headings (n,), gladhand position, rotation)."""
        L, n = self.seg_len, self.segments
        headings = self.yaw + np.concatenate([[0.0], np.cumsum(self.bends)])
        dirs = np.stack([np.cos(headings), np.sin(headings), np.zeros(n)], axis=1)
        starts = np.asarray(self.pos, float) + np.vstack([np.zeros(3), np.cumsum(dirs, axis=0)[:-1] * L])
        centres = starts + dirs * L / 2
        end = starts[-1] + dirs[-1] * L
        seg_rot = rot_z(headings[-1])
        rot = seg_rot @ (FLIP_X if self.seal_down else np.eye(3))
        pos = end - rot @ (np.array(PORT) * MM)
        return centres, headings, pos, rot


def plan_bends(start_xy, yaw: float, end_xy, end_heading: float, segments: int, seg_len: float,
               turn: float, keep_out=()) -> np.ndarray:
    """Bend angles (rad) of the smoothest hose from ``start_xy`` leaving along ``yaw`` to ``end_xy`` arriving along
    ``end_heading``, turning ``turn`` rad in total (its sign picks the side the loop lies on). ``keep_out`` lists
    boxes (x0, x1, y0, y1) the hose's centreline must stay out of."""
    n = segments
    a = np.full(n - 1, turn / (n - 1))
    target = np.r_[np.asarray(end_xy, float) - np.asarray(start_xy, float), turn]

    def end(a):
        h = yaw + np.concatenate([[0.0], np.cumsum(a)])
        return np.r_[seg_len * np.cos(h).sum(), seg_len * np.sin(h).sum(), a.sum()]

    for _ in range(200):
        c = end(a) - target
        jac = np.zeros((3, n - 1))
        for i in range(n - 1):
            d = a.copy()
            d[i] += 1e-6
            jac[:, i] = (end(d) - end(a)) / 1e-6
        # Smallest bends that satisfy the linearised end constraints.
        goal = jac.T @ np.linalg.solve(jac @ jac.T + 1e-12 * np.eye(3), jac @ a - c)
        a = a + 0.5 * (goal - a)
        if np.linalg.norm(c) < 1e-9 and np.linalg.norm(goal - a) < 1e-9:
            break
    assert abs((yaw + turn - end_heading + math.pi) % (2 * math.pi) - math.pi) < 1e-6, "turn must reach end_heading"
    if not keep_out:
        return a
    from scipy.optimize import minimize

    def points(a):
        h = yaw + np.concatenate([[0.0], np.cumsum(a)])
        steps = seg_len * np.stack([np.cos(h), np.sin(h)], axis=1)
        return np.asarray(start_xy, float) + np.vstack([np.zeros(2), np.cumsum(steps, axis=0)])

    def clear(a):
        """Each joint's distance outside each keep-out box (m; negative inside)."""
        p = points(a)
        return np.concatenate([-np.minimum(np.minimum(p[:, 0] - x0, x1 - p[:, 0]), np.minimum(p[:, 1] - y0, y1 - p[:, 1]))
                               for x0, x1, y0, y1 in keep_out])

    # Push the smoothest curve out of the boxes, then smooth it again while keeping it out.
    out = minimize(lambda a: float(a @ a) + 2e4 * float(np.sum(np.clip(-clear(a), 0.0, None) ** 2)), a, method="SLSQP",
                   constraints=[{"type": "eq", "fun": lambda a: end(a) - target}], options={"maxiter": 500, "ftol": 1e-10})
    out = minimize(lambda a: float(a @ a), out.x, method="SLSQP",
                   constraints=[{"type": "eq", "fun": lambda a: end(a) - target}, {"type": "ineq", "fun": clear}],
                   options={"maxiter": 500, "ftol": 1e-10})
    assert np.linalg.norm(end(out.x) - target) < 1e-6, "the planned hose misses its gladhand"
    assert clear(out.x).min() > -1e-3, f"no hose path clear of the keep-out boxes (inside by {-clear(out.x).min():.3f} m)"
    return out.x


def air_line(name: str, fitting, yaw: float, gladhand_pos, gladhand_rot, color, turn: float, keep_out=(),
             **kwargs) -> AirLine:
    """An air line from ``fitting`` (world, m) leaving along ``yaw`` to a gladhand that starts at ``gladhand_pos``
    with rotation ``gladhand_rot`` (its handle horizontal, its seal face down)."""
    line = AirLine(name=name, pos=tuple(float(v) for v in fitting), size=(1.0, 1.0, 0.1), color=tuple(color), yaw=yaw,
                   **kwargs)
    rot = np.asarray(gladhand_rot, float)
    assert rot[2, 2] < -0.999, "the gladhand must start with its seal face down"
    port = np.asarray(gladhand_pos, float) + rot @ (np.array(PORT) * MM)
    assert abs(port[2] - fitting[2]) < 1e-4, f"fitting height {fitting[2]:.4f} must match the port's {port[2]:.4f}"
    into = rot @ np.array([1.0, 0.0, 0.0])  # the hose runs into the port along the gladhand's +x
    end_heading = math.atan2(into[1], into[0])
    bends = plan_bends(fitting[:2], yaw, port[:2], end_heading, line.segments, line.seg_len, turn, keep_out)
    return AirLine(**{**line.__dict__, "bends": tuple(float(b) for b in bends)})


@dataclass(frozen=True)
class GladState(ObjectState):
    """A gladhand: ``pos`` is the centre of its face, ``seal`` its seal's travel (m; 0 uncompressed, negative
    pressed in; None without a seal)."""

    seal: float | None = None

    def rotation(self) -> np.ndarray:
        return rot_from_xyzw(self.quat_xyzw)

    def to_world(self, local) -> np.ndarray:
        return self.pos + self.rotation() @ np.asarray(local, float)

    def to_local(self, world) -> np.ndarray:
        return self.rotation().T @ (np.asarray(world, float) - self.pos)


@dataclass(frozen=True)
class LineState(GladState):
    """An air line: its gladhand (as GladState) and its hose's segment centres, fitting end first."""

    points: np.ndarray = None
    radius: float = 0.0
    seg_len: float = 0.0
    stretch: float = 0.0  # how far the hose's first segment has left its fitting (m)


# -- Isaac Sim side -------------------------------------------------------------------------------------------------

_SPECS: dict[str, Box] = {}
_FIXED_ENDS: dict[str, np.ndarray] = {}


def _hull_faces(points: np.ndarray):
    from scipy.spatial import ConvexHull

    hull = ConvexHull(points)
    centre = points[hull.vertices].mean(0)
    faces = []
    for simplex, eq in zip(hull.simplices, hull.equations):
        a, b, c = (int(i) for i in simplex)
        normal = np.cross(points[b] - points[a], points[c] - points[a])
        if np.dot(normal, points[a] - centre) < 0:
            b, c = c, b
        faces.append((a, b, c))
    return faces


class _Author:
    """USD authoring helpers shared by the spawners (pxr only)."""

    def __init__(self, prim_path: str, colors: dict):
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils.stage import get_current_stage

        from yam_arena.station import linear_rgb

        self.stage = get_current_stage()
        self.root = prim_path
        self.mats = {}
        for key, (static, dynamic) in (("steel", STEEL), ("rubber", RUBBER)):
            path = f"{prim_path}/Looks/{key}_physics"
            pm = sim_utils.RigidBodyMaterialCfg(static_friction=static, dynamic_friction=dynamic)
            pm.func(path, pm)
            self.mats[key] = path
        self.looks = {}
        for key, color in colors.items():
            path = f"{prim_path}/Looks/{key}"
            metallic = 0.6 if key in ("iron", "brass") else 0.0
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.45, metallic=metallic)
            vis.func(path, vis)
            self.looks[key] = path

    def xform(self, path: str, pos=(0.0, 0.0, 0.0), xyzw=(0.0, 0.0, 0.0, 1.0)):
        from pxr import Gf, UsdGeom

        x = UsdGeom.Xform.Define(self.stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(float(v) for v in pos)))
        qx, qy, qz, qw = (float(v) for v in xyzw)
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, qx, qy, qz))
        return x.GetPrim()

    def body(self, path: str, pos, xyzw, mass: float, inertia: float | None = None, lin_damp=0.05, ang_damp=0.5):
        from pxr import Gf, PhysxSchema, UsdPhysics

        prim = self.xform(path, pos, xyzw)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(mass)
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia, inertia, inertia))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(4)
        rb.CreateLinearDampingAttr(lin_damp)
        rb.CreateAngularDampingAttr(ang_damp)
        return prim

    def _collide(self, prim, shape_look: str, material: str, offset: float):
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from pxr import PhysxSchema, UsdPhysics

        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(offset)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), self.mats[material], stage=self.stage)
        bind_visual_material(str(prim.GetPath()), self.looks[shape_look], stage=self.stage)

    def shapes(self, path: str, shapes, offset: float = 0.001):
        from pxr import Gf, UsdGeom, UsdPhysics, Vt

        for i, s in enumerate(shapes):
            p = f"{path}/{s.kind}_{i}"
            if s.kind == "box":
                g = UsdGeom.Cube.Define(self.stage, p)
                g.CreateSizeAttr(1.0)
                g.AddTranslateOp().Set(Gf.Vec3d(*s.centre))
                g.AddScaleOp().Set(Gf.Vec3f(*s.size))
            elif s.kind == "cyl":
                g = UsdGeom.Cylinder.Define(self.stage, p)
                g.CreateAxisAttr("Z")
                g.CreateRadiusAttr(s.radius)
                g.CreateHeightAttr(s.height)
                g.AddTranslateOp().Set(Gf.Vec3d(*s.centre))
            else:
                pts = np.array(s.points, float)
                faces = _hull_faces(pts)
                g = UsdGeom.Mesh.Define(self.stage, p)
                g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*(float(v) for v in q)) for q in pts]))
                g.CreateFaceVertexCountsAttr([3] * len(faces))
                g.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
                g.CreateSubdivisionSchemeAttr("none")
                g.CreateDoubleSidedAttr(True)
                UsdPhysics.MeshCollisionAPI.Apply(g.GetPrim()).CreateApproximationAttr("convexHull")
            self._collide(g.GetPrim(), s.look, s.material, offset)

    def joint(self, kind: str, path: str, parent: str | None, child: str, pos0, pos1=(0.0, 0.0, 0.0),
              rot0_xyzw=(0.0, 0.0, 0.0, 1.0), axis: str = "X", lower=None, upper=None, stiffness=None,
              damping=None):
        from pxr import Gf, Sdf, UsdPhysics

        cls = {"fixed": UsdPhysics.FixedJoint, "revolute": UsdPhysics.RevoluteJoint,
               "prismatic": UsdPhysics.PrismaticJoint}[kind]
        j = cls.Define(self.stage, path)
        if parent is not None:
            j.CreateBody0Rel().SetTargets([Sdf.Path(parent)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in pos0)))
        x, y, z, w = (float(v) for v in rot0_xyzw)
        j.CreateLocalRot0Attr(Gf.Quatf(w, x, y, z))
        j.CreateLocalPos1Attr(Gf.Vec3f(*(float(v) for v in pos1)))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        if kind == "fixed":
            return j
        j.CreateAxisAttr(axis)
        j.CreateLowerLimitAttr(lower)
        j.CreateUpperLimitAttr(upper)
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear" if kind == "prismatic" else "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(stiffness)
        drive.CreateDampingAttr(damping)
        drive.CreateTargetPositionAttr(0.0)
        return j

    def seal(self, body_path: str, seal_path: str, body_pos, body_rot: np.ndarray, body_xyzw):
        """A rubber seal on a sprung slider along the gladhand's axis; the bodies' poses are in the root frame."""
        pos = np.asarray(body_pos, float) + body_rot @ np.array(SEAL_REST)
        self.body(seal_path, pos, body_xyzw, SEAL_MASS)
        self.shapes(seal_path, [SEAL_SHAPE])
        self.joint("prismatic", f"{seal_path}/seal_slide", body_path, seal_path, SEAL_REST, axis="Z",
                   lower=-SEAL_TRAVEL * MM, upper=0.0, stiffness=SEAL_RATE, damping=SEAL_DAMPING)

    def articulation(self, root_link: str):
        from pxr import PhysxSchema, UsdPhysics

        prim = self.stage.GetPrimAtPath(root_link)
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        art = PhysxSchema.PhysxArticulationAPI.Apply(prim)
        art.CreateEnabledSelfCollisionsAttr(False)
        art.CreateSolverPositionIterationCountAttr(32)
        art.CreateSolverVelocityIterationCountAttr(4)

    def anchor(self, path: str, link: str):
        """Weld ``link`` to the world where it stands."""
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        fj = UsdPhysics.FixedJoint.Define(self.stage, path)
        fj.CreateBody1Rel().SetTargets([Sdf.Path(link)])
        world = UsdGeom.Xformable(self.stage.GetPrimAtPath(link)).ComputeLocalToWorldTransform(0)
        t, q = world.ExtractTranslation(), world.ExtractRotationQuat()
        fj.CreateLocalPos0Attr(Gf.Vec3f(*t))
        fj.CreateLocalRot0Attr(Gf.Quatf(q.GetReal(), *q.GetImaginary()))
        fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))


def _spawn_coupler(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec: Coupler = _SPECS[cfg.spec_name]
    au = _Author(prim_path, {"body": spec.color, "iron": IRON, "seal": BLACK, "brass": BRASS})
    root = au.xform(prim_path, translation or (0.0, 0.0, 0.0), orientation if orientation is not None else (0, 0, 0, 1))
    shapes = gladhand_shapes(spec.kind)
    if spec.kind == "dummy":
        au.shapes(prim_path, shapes)  # static colliders: no rigid body
        return root
    body = f"{prim_path}/body"
    au.body(body, (0, 0, 0), (0, 0, 0, 1), BODY_MASS)
    au.shapes(body, shapes)
    au.seal(body, f"{prim_path}/seal", (0, 0, 0), np.eye(3), (0, 0, 0, 1))
    au.articulation(body)
    au.anchor(f"{prim_path}/anchor", body)
    return root


def _spawn_line(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec: AirLine = _SPECS[cfg.spec_name]
    au = _Author(prim_path, {"body": spec.color, "iron": IRON, "seal": BLACK, "brass": BRASS, "hose": spec.color})
    root = au.xform(prim_path, translation or (0.0, 0.0, 0.0), orientation if orientation is not None else (0, 0, 0, 1))
    L, r, n = spec.seg_len, spec.radius, spec.segments
    seg_mass = spec.hose_mass / n
    centres, headings, g_pos, g_rot = spec.start_layout()
    # Everything below is authored in the root frame: origin at the first segment's centre, x along it.
    to_root = rot_z(-spec.yaw)
    origin = centres[0]

    def local(p):
        return to_root @ (np.asarray(p, float) - origin)

    from pxr import UsdGeom

    seg = []
    for i in range(n):
        path = f"{prim_path}/seg_{i}"
        au.body(path, local(centres[i]), quat_xyzw(rot_z(headings[i] - spec.yaw)), seg_mass, lin_damp=0.3,
                ang_damp=1.0)
        cap = UsdGeom.Capsule.Define(au.stage, f"{path}/geom")
        cap.CreateAxisAttr("X")
        cap.CreateRadiusAttr(r)
        cap.CreateHeightAttr(L - 0.2 * r)  # slight overlap: no visible gaps when bent
        au._collide(cap.GetPrim(), "hose", "rubber", 0.002)
        seg.append(path)
    deg = 180.0 / math.pi
    for i in range(1, n):
        # Three single-axis joints through two tiny links: twist (x), then bending (y and z).
        joint_at = local(centres[i - 1]) + rot_z(headings[i - 1] - spec.yaw) @ np.array([L / 2, 0.0, 0.0])
        prev_q = quat_xyzw(rot_z(headings[i - 1] - spec.yaw))
        a, b = f"{prim_path}/bend_{i}_a", f"{prim_path}/bend_{i}_b"
        for link in (a, b):
            au.body(link, joint_at, prev_q, 2e-4, inertia=2e-8, lin_damp=0.0, ang_damp=0.0)
        au.joint("revolute", f"{a}/bend_{i}_x", seg[i - 1], a, (L / 2, 0, 0), axis="X", lower=-spec.max_twist * deg,
                 upper=spec.max_twist * deg, stiffness=spec.twist_stiffness / deg, damping=spec.twist_damping / deg)
        au.joint("revolute", f"{b}/bend_{i}_y", a, b, (0, 0, 0), axis="Y", lower=-spec.max_bend * deg,
                 upper=spec.max_bend * deg, stiffness=spec.bend_stiffness / deg, damping=spec.bend_damping / deg)
        au.joint("revolute", f"{seg[i]}/bend_{i}_z", b, seg[i], (0, 0, 0), (-L / 2, 0, 0), axis="Z",
                 lower=-spec.max_bend * deg, upper=spec.max_bend * deg, stiffness=spec.bend_stiffness / deg,
                 damping=spec.bend_damping / deg)
    # Brass fitting sleeve on the first segment (fixed end) for looks.
    # The gladhand rides on the last segment.
    body = f"{prim_path}/gladhand"
    g_rot_local = to_root @ g_rot
    g_q = quat_xyzw(g_rot_local)
    au.body(body, local(g_pos), g_q, BODY_MASS, ang_damp=0.5)
    au.shapes(body, gladhand_shapes("hose"))
    last_rot = rot_z(headings[-1] - spec.yaw)
    au.joint("fixed", f"{body}/hose_end", seg[-1], body, (L / 2, 0, 0), np.array(PORT) * MM,
             rot0_xyzw=quat_xyzw(last_rot.T @ g_rot_local))
    au.seal(body, f"{prim_path}/seal", local(g_pos), g_rot_local, g_q)
    au.articulation(seg[0])
    au.anchor(f"{prim_path}/anchor", seg[0])
    return root


def _arena_object(spec):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    is_line = isinstance(spec, AirLine)

    @configclass
    class GladSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_line if is_line else _spawn_coupler)
        spec_name: str = ""

    seal = ImplicitActuatorCfg(joint_names_expr=["seal_slide"], stiffness=SEAL_RATE, damping=SEAL_DAMPING)

    class GladObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {"seal": seal}
            joint_pos = {".*": 0.0}
            if is_line:
                actuators["bend"] = ImplicitActuatorCfg(
                    joint_names_expr=["bend_.*_[yz]"], stiffness=spec.bend_stiffness, damping=spec.bend_damping,
                    armature=spec.armature)
                actuators["twist"] = ImplicitActuatorCfg(
                    joint_names_expr=["bend_.*_x"], stiffness=spec.twist_stiffness, damping=spec.twist_damping,
                    armature=spec.armature)
                # The gladhand starts locked on a dummy coupler, its seal pressed in against the dummy's face.
                pressed = -(SEAL_PROUD + DUMMY_BOSS - GAP_LOCKED) * MM if spec.seal_down else 0.0
                joint_pos = {"seal_slide": pressed, "bend_.*_[xy]": 0.0,
                             **{f"bend_{i + 1}_z": float(b) for i, b in enumerate(spec.bends)}}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            cfg = self._add_initial_pose_to_cfg(cfg)
            cfg.init_state.joint_pos = joint_pos
            return cfg

    if is_line:
        centres, _, _, _ = spec.start_layout()
        pose = Pose(position_xyz=tuple(centres[0].tolist()), rotation_xyzw=quat_xyzw(rot_z(spec.yaw)))
        kind = ObjectType.ARTICULATION
    else:
        pose = Pose(position_xyz=tuple(spec.pos), rotation_xyzw=quat_xyzw(spec.rotation))
        kind = ObjectType.BASE if spec.kind == "dummy" else ObjectType.ARTICULATION
    return GladObject(name=spec.name, object_type=kind, spawner_cfg=GladSpawnerCfg(spec_name=spec.name),
                      initial_pose=pose)


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _static_state(spec: Coupler, seal=None) -> GladState:
    lo = np.asarray(spec.pos, float) - 0.17
    return GladState(pos=np.asarray(spec.pos, float), quat_xyzw=np.array(quat_xyzw(spec.rotation)),
                     lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=lo, high=lo + 0.34, seal=seal)


def _state(environment, spec):
    if isinstance(spec, Coupler) and spec.kind == "dummy":
        return _static_state(spec)
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    joints = list(asset.joint_names)
    seal = float(_np(asset.data.joint_pos)[0][joints.index("seal_slide")])
    if isinstance(spec, Coupler):
        return _static_state(spec, seal)
    names = list(asset.body_names)
    poses = _np(asset.data.body_link_pose_w)[0]
    vels = _np(asset.data.body_link_vel_w)[0] if hasattr(asset.data, "body_link_vel_w") else np.zeros((len(names), 6))
    g = names.index("gladhand")
    pts = np.array([poses[names.index(f"seg_{i}")][:3] - origin for i in range(spec.segments)])
    # Where the hose's fixed end sat when first seen: a hose pulled off its fitting would have moved from there.
    fixed_end = _FIXED_ENDS.setdefault(spec.name, pts[0].copy())
    pos = poses[g][:3] - origin
    return LineState(pos=pos, quat_xyzw=poses[g][3:7], lin_vel=vels[g][:3], ang_vel=vels[g][3:6],
                     low=np.minimum(pts.min(0), pos) - 0.03, high=np.maximum(pts.max(0), pos) + 0.03, seal=seal,
                     points=pts, radius=spec.radius, seg_len=spec.seg_len,
                     stretch=float(np.linalg.norm(pts[0] - fixed_end)))


def finger_points(environment) -> list:
    """Points on both arms' finger pads (world, relative to the env origin): at the TCP and at the pad tips, on each
    pad's inner face. score() does not see the robot, so an ``on_step`` hook uses these for "let go" checks."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        jp, jn = _np(robot.data.joint_pos)[0], list(robot.joint_names)
        half_gap = max(abs(jp[jn.index(j)]) for j in ("joint7", "joint8") if j in jn)
        r = rot_from_xyzw(pose[3:7])
        for along in (-0.02, 0.0, PAD_TIP_PAST_TCP):
            centre = pose[:3] + r @ (np.array(TCP_OFFSET) + [0.0, 0.0, along]) - origin
            pts += [centre + r[:, 0] * side * half_gap for side in (1.0, -1.0)]
    return pts


def distance_to_gladhand(point, pos, rot) -> float:
    """Distance (m) from a world point to a gladhand's body (its handle and head boxes)."""
    p = np.asarray(rot, float).T @ (np.asarray(point, float) - np.asarray(pos, float))
    return float(min(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0)) for lo, hi in GLADHAND_BOXES))


# -- hooking yam_arena ----------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    """Register ``fn(env)`` to run after every control step, before the task file is scored."""
    _STEP.append(fn)
    return fn


def on_reset(fn):
    """Register ``fn(env)`` to run after every reset, once objects have settled."""
    _RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_gladkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, (Coupler, AirLine)):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = orig_states(environment, plain) if plain else {}
        for n in names:
            if n in _SPECS:
                out[n] = _state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._gladkit_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_gladkit_patched", False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = base_reset(self, *a, **k)
        _ENV["env"] = self._env
        for fn in _RESET:
            fn(self._env)
        return obs

    base_ensure = cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        base_ensure(self, *a, **k)
        environment = self._env
        if getattr(environment, "_gladkit_reset", False):
            return
        orig = environment.reset

        def env_reset(*ra, **rk):
            out = orig(*ra, **rk)
            _restore_joints(environment)
            return out

        environment.reset = env_reset
        environment._gladkit_reset = True

    cls._task_step, cls.reset, cls._ensure_sim = _task_step, reset, _ensure_sim
    cls._gladkit_patched = True


def _restore_joints(environment) -> None:
    """Put every hose back on its planned starting curve (and every seal at its start), at rest. Runs as part of
    the environment's reset, before the scene settles. The joint springs keep their targets: a straight hose and an
    uncompressed seal."""
    import torch

    base = environment.unwrapped
    for name, spec in _SPECS.items():
        if isinstance(spec, Coupler) and spec.kind == "dummy":
            continue
        asset = base.scene[name]
        start = _t(asset.data.default_joint_pos).clone()
        asset.write_joint_position_to_sim_index(position=start)
        asset.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(start))
        # Reading the link poses back makes the simulator recompute them from the new joint positions; stepping
        # without that leaves the links where they were and the solver diverges.
        _np(asset.data.body_link_pose_w)


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
        orig_exec = loader.exec_module

        def exec_module(module):
            orig_exec(module)
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
assert task_file
