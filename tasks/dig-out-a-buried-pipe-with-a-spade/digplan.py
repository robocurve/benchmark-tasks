"""The spade's geometry, the layout around it, and the scripted bimanual demo. NumPy only (no Isaac Sim).

Frames: world x forward from the arms, y left, z up (m). The spade's own frame has its origin on the shaft's axis where
the shaft meets the blade's socket, x along the shaft toward the blade, y across the blade (its width), z the blade's
face normal (the side soil is carried on). The trench frame has its origin at the planter's centre on the table, u
along the planter's length (the way the spade digs), v across it.

The hands hold the spade rigidly, so the whole demo is planned as one path of the spade's pose; each hand's TCP target
is that pose times its fixed grip. The plan is sampled at the control rate and every sample is checked with IK
(``fastik``) before the demo is handed to ``yam_scripted``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

import fastik

TOP = 0.7475  # the station's table top (yam_arena.task_file.TABLE_TOP["station"])
BASES = {"left": (0.0, 0.305, 0.75), "right": (0.0, -0.305, 0.75)}  # yam_arena.station.BASES
READY = (0.0, 1.0, 1.0, -0.6, 0.0, 0.0)
DT = 0.05  # 20 Hz

# -- the spade ------------------------------------------------------------------------------------------------------
LIFT = math.radians(10.0)  # the blade bends up from the shaft's line by this much (its "lift")
BLADE_LEN = 0.125  # heel to tip, along the blade
BLADE_W = 0.105  # across the heel
BLADE_T = 0.003  # steel plate
FLAT_HALF = 0.030  # the flat middle of the dished blade, each side of the centre line
WING = math.radians(35.0)  # cupped shovel sides retain loose soil during lifting
BLADE_Z = -0.025  # blade floor below the socket, so the edge reaches soil first
HEEL_RISE = 0.025  # raised rear lip retains a loaded scoop during transport
SOCKET = (-0.070, 0.0, 0.0175)  # tapered steel socket: from, to (x), radius
SHAFT = (-0.190, -0.060, 0.0150)  # ash shaft: from, to (x), radius
D_JOIN, D_BAR, D_HALF, D_R = -0.190, -0.240, 0.048, 0.012  # D-grip: where the yoke starts, crossbar x, half width, bar radius
RIGHT_AT = -0.105  # the right hand holds the wood, clear of the socket's diameter change
LEFT_AT = D_BAR  # the left hand holds the D-grip's crossbar
ALPHA = math.radians(-30.0)  # the right hand comes at the shaft from above, turned this far toward the blade's right
BETA = math.radians(-45.0)  # the left hand comes at the crossbar from above and behind (pushing along the shaft)
SPADE_MASS = 0.45  # includes the two flat rubber grip sleeves
SPADE_COM = (-0.040, 0.0, -0.004)


def blade_dir() -> np.ndarray:
    return np.array([math.cos(LIFT), 0.0, math.sin(LIFT)])


def blade_point(s: float, w: float) -> np.ndarray:
    """A point on the blade's top face: ``s`` m from the heel along the blade, ``w`` m across (spade frame)."""
    d = blade_dir()
    n = np.array([-math.sin(LIFT), 0.0, math.cos(LIFT)])
    rise = max(0.0, abs(w) - FLAT_HALF) * math.tan(WING)
    rise += HEEL_RISE * max(0.0, 1.0-s/0.025)  # planar heel ramp behind the load
    return s * d + np.array([0.0, w, 0.0]) + (BLADE_Z + rise) * n


def half_width(s: float) -> float:
    """The blade's half width ``s`` m from the heel: straight sides, then a rounded point."""
    s0 = 0.070
    if s <= s0:
        return BLADE_W / 2
    f = (s - s0) / (BLADE_LEN - s0)
    return BLADE_W / 2 * math.sqrt(max(0.0, 1.0 - f**1.6))


def tip_local() -> np.ndarray:
    return blade_point(BLADE_LEN, 0.0)


def rot(yaw: float, pitch: float, roll: float = 0.0) -> np.ndarray:
    """The spade frame in the world: turned ``yaw`` about z, then tipped ``pitch`` (blade end down), then rolled
    ``roll`` about its own shaft."""
    cy, sy, cp, sp, cr, sr = (math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll),
                              math.sin(roll))
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def pose_from_tip(tip, yaw, pitch, roll=0.0) -> tuple[np.ndarray, np.ndarray]:
    r = rot(yaw, pitch, roll)
    return np.asarray(tip, float) - r @ tip_local(), r


def _frame(approach, closing) -> np.ndarray:
    a = approach / np.linalg.norm(approach)
    c = closing - np.dot(closing, a) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def grips_local() -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Each hand's TCP (position, rotation) in the spade frame: gripper z is the approach, x the fingers' closing
    line."""
    x, y, z = np.eye(3)
    a_r = -(math.cos(ALPHA) * z + math.sin(ALPHA) * y)
    a_l = -(math.cos(BETA) * z + math.sin(BETA) * x)
    return {"right": (RIGHT_AT * x, _frame(a_r, np.cross(a_r, x))),
            "left": (LEFT_AT * x, _frame(a_l, np.cross(y, a_l)))}


# -- the layout -----------------------------------------------------------------------------------------------------
YAW = math.radians(-37.0)  # the planter's length, and the way the spade digs
CENTRE = np.array([0.525, -0.185])  # the planter's centre on the table
IN_U, IN_V = 0.300, 0.140  # inside length and width
WALL, FLOOR, WALL_H = 0.014, 0.010, 0.080
SOIL_DEPTH = 0.060  # target loose-fill depth; a settled layout is saved for the task
PIPE_R = 0.0167  # test pipe, 33.4 mm outside diameter
PIPE_Z = TOP + FLOOR + PIPE_R + 0.004  # its axis
PIPE_TOP = PIPE_Z + PIPE_R
U = np.array([math.cos(YAW), math.sin(YAW)])
V = np.array([-math.sin(YAW), math.cos(YAW)])
SOIL_TOP = TOP + FLOOR + SOIL_DEPTH


def to_trench(xy) -> np.ndarray:
    d = np.asarray(xy, float)[..., :2] - CENTRE
    return np.stack([d @ U, d @ V], axis=-1)


def from_trench(u: float, v: float) -> np.ndarray:
    return CENTRE + u * U + v * V


# Where the spade starts: lying on two wooden rests to the right of the planter, parallel to it.
REST_TIP = np.array([0.400, -0.312, TOP + 0.064])
REST_PITCH = math.radians(4.0)
REST_POSE = pose_from_tip(REST_TIP, YAW, REST_PITCH)
# The spoil tarp, left of the planter.
TARP_C, TARP_SIZE, TARP_YAW = np.array([0.395, 0.030])-0.075*U, (0.260, 0.240), YAW
DUMP_TIP = np.array([0.455-0.01*U[0], -0.01*U[1], TOP + 0.150])
DUMP_ROLL = math.radians(60.0)


# -- the demo -------------------------------------------------------------------------------------------------------
@dataclass
class Key:
    """One keyframe of the spade's path: its tip (world), yaw, pitch, roll, reached ``t`` s after the previous one."""

    t: float
    tip: tuple
    pitch: float
    roll: float = 0.0
    label: str = ""
    yaw: float = YAW


def _ease(s: float) -> float:
    return s * s * (3 - 2 * s)


def sample(keys: list[Key], start: Key) -> list[tuple[float, np.ndarray, np.ndarray, str]]:
    """(time, spade origin, rotation, label) every control step along the keyframes, eased in and out of each."""
    out = []
    prev = start
    t0 = 0.0
    for k in keys:
        n = max(1, round(k.t / DT))
        a = np.r_[prev.tip, prev.yaw, prev.pitch, prev.roll]
        b = np.r_[k.tip, k.yaw, k.pitch, k.roll]
        for i in range(1, n + 1):
            v = a + (b - a) * _ease(i / n)
            o, r = pose_from_tip(v[:3], v[3], v[4], v[5])
            out.append((t0 + i * DT, o, r, k.label))
        t0 += n * DT
        prev = k
    return out


def scoop(
    u0: float,
    v0: float,
    depth: float,
    push: float = 0.185,
    pitch_in: float = 25.0,
    pitch_mid: float = 22.0,
    pitch_end: float = 16.0,
) -> list[Key]:
    """One scoop: the tip set down at (u0, v0) in the trench, driven ``depth`` below the soil's surface and on along the
    trench by ``push``, curled and lifted out, carried over the tarp, rolled to tip the soil off, rolled back."""
    p = np.radians
    xy0 = from_trench(u0, v0)
    xy_mid = from_trench(u0 + 0.55 * push, v0)
    xy1 = from_trench(u0 + push, v0)
    xy_lift = from_trench(u0 + push + 0.010, v0)
    z_in = SOIL_TOP - depth
    above = (*xy0, SOIL_TOP + 0.045)
    return [
        Key(1.2, above, p(pitch_in), 0.0, "over the trench, blade tipped down"),
        Key(1.3, (*xy0, z_in), p(pitch_in), 0.0, "lower the blade over the marked work span"),
        Key(2.0, (*xy_mid, z_in), p(pitch_mid), 0.0, "slide the blade under the loose layer"),
        Key(1.5, (*xy1, z_in + 0.004), p(pitch_end), 0.0, "flatten blade through far span"),
        Key(1.0, (*xy_lift, TOP + 0.115), p(0), 0.0, "curl blade to hold load"),
        Key(1.0, (*xy1, TOP + 0.175), p(-5), 0.0, "lift the load clear"),
        Key(1.0, (*from_trench(-0.04, v0), TOP + 0.175), p(-4), 0.0, "carry back along the bed before crossing its rim"),
        Key(1.5, tuple(DUMP_TIP), p(-2), 0.0, "carry the load over the spoil tray"),
        Key(1.0, tuple(DUMP_TIP + [0, 0, -0.01]), p(4), DUMP_ROLL, "roll the blade over: soil off"),
        Key(0.4, tuple(DUMP_TIP + [0, 0, 0.0]), p(8), DUMP_ROLL, "let it slide off"),
        Key(0.9, tuple(DUMP_TIP), p(-2), 0.0, "roll it back"),
    ]


# Three angled cuts through the reduced cover. Contact with the fixed pipe is
# allowed; its physical collider guides the blade along the crown on deeper cuts.
SCOOPS = [
    (-0.070, 0.0, 0.014),
    (-0.070, 0.0, 0.023),
    (-0.055, 0.0, 0.027),
]


def spade_path() -> tuple[list[Key], Key]:
    start = Key(0.0, tuple(REST_TIP), REST_PITCH, 0.0, "resting")
    keys = [Key(1.6, tuple(REST_TIP + [0, 0, 0.10]), REST_PITCH, 0.0, "lift the spade off its rests")]
    for u0, v0, depth in SCOOPS:
        keys += scoop(u0, v0, depth)
    keys += [
        Key(1.4, tuple(REST_TIP + [0, 0, 0.06]), REST_PITCH, 0.0, "carry the spade back over its rests"),
        Key(1.0, tuple(REST_TIP + [0, 0, -0.002]), REST_PITCH, 0.0, "lay it down on its rests"),
    ]
    return keys, start


def solve_path(samples, q_start: dict | None = None):
    """IK along the sampled spade path, warm-started: per arm, joints at every sample; raises if any is off."""
    g = grips_local()
    qs = {arm: [] for arm in g}
    q = dict(q_start or {})
    for t, o, r, label in samples:
        for arm, (pl, rl) in g.items():
            p, rr = o + r @ pl, r @ rl
            if arm not in q:
                q[arm] = best_ik(arm, p, rr)
            qn, pe, re = fastik.solve(p, rr, q[arm], BASES[arm], iters=60)
            if pe > 1e-3 or re > 0.01 or np.abs(qn - q[arm]).max() > 0.25:
                raise ValueError(f"{arm} at t={t:.2f} ({label}): IK off by {pe * 1000:.1f} mm, {math.degrees(re):.1f} deg,"
                                 f" jump {np.abs(qn - q[arm]).max():.2f} rad")
            q[arm] = qn
            qs[arm].append(qn)
    return {arm: np.array(v) for arm, v in qs.items()}


def best_ik(arm: str, p, r) -> np.ndarray:
    import itertools

    best = None
    for s in itertools.product((-0.6, 0.0, 0.6), (1.0, 1.8), (0.8, 1.6), (-0.8, 0.3), (-0.8, 0.8), (-1.2, 0.0, 1.2)):
        q, pe, re = fastik.solve(p, r, s, BASES[arm], iters=80)
        if pe < 1e-4 and re < 1e-3:
            room = float(np.minimum(q - fastik.LOWER, fastik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK for the {arm} hand at {np.round(p, 3)}")
    return best[0]
