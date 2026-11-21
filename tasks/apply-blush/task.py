"""Apply blush to both cheeks of a cosmetology practice head with two pre-loaded blush brushes.

On the station table, in front of the two arms, stands a vinyl practice head on a grey stand (fixed in place, facing
the robot: skull 19 cm front to back, 14 cm across, chin 12.5 cm and crown 31 cm above the table, as the accepted
task "put-reading-glasses-on-a-care-manikin"; cheekbone and jaw widths from Zhuang and Bradtmiller 2005). On each
cheek an oval (4.4 x 2.8 cm) is drawn in solid ink, and a limit (6.8 x 4.8 cm) round it in dashed ink. Beside each
arm lie a pressed blush pan (36 mm round tin pan, taped down) and a clean round blush brush (157 mm, 12 mm ferrule,
after Kent TWMU10). The robot has to load each brush from its pan (swirl the bristles on the cake), then sweep it over
its cheek's oval until the oval is covered, without pigment going past the limit.

What makes it hard: the brush has to be picked up by its slim handle, loaded on the cake, and brought to a curved
cheek with the bristles square on it, then moved in sweeps that keep contact: a dry brush leaves nothing, a brush
held off the skin leaves nothing, and one pressed in one place covers only its own 2.4 cm.

Simulation: the head, stand and pans are fixed rigid bodies; each brush a free rigid body (a convex hull for the
bristle cluster). Pigment is modelled, not simulated: while the bristles are within 2 mm of the cake, the brush takes
dose from it; while they are within 2 mm of the skin, each skin point under them takes pigment from the brush's dose
(a brush carries a finite dose), until it shows (see ``score``).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
"""

from __future__ import annotations

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scenekit import Shape, UsdBody, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Apply blush to both cheeks: sweep each brush over the inked oval on its cheek until it is covered."

MM = 0.001
TOP = TABLE_TOP[SCENE]
HERE = pathlib.Path(__file__).resolve().parent
DATA = np.load(HERE / "assets" / "blush_samples.npz")

# -- the practice head: origin at the chin, face towards +x in its own frame; the stand's foot is 125 mm below it.
# Placed facing the robot (yawed 180 degrees), fixed (the stand is taped to the table).
CHIN_H = 125 * MM
BUST_XY = (0.345, 0.0)
BX, BY = BUST_XY
BUST_Z = TOP + CHIN_H
BUST = UsdBody("bust", usd="assets/bust.usdc", pos=(BX, BY, BUST_Z), rot_xyzw=(0.0, 0.0, 1.0, 0.0),
               size=(0.19, 0.14, 0.31), mass=None, friction=(0.6, 0.5))


def bust_to_world(p) -> np.ndarray:
    """Points in the head's frame (chin at the origin, face towards +x) to the world (the head faces -x)."""
    p = np.asarray(p, float)
    return np.stack([BX - p[..., 0], BY - p[..., 1], BUST_Z + p[..., 2]], -1)


SKIN = bust_to_world(DATA["skin"])  # about 3.5 mm apart over the head's surface
# a cheek is named by the arm that reaches it: the right arm (y < 0) reaches the head's +y cheek in its own frame
S_ARC = {"right": DATA["s_pos"], "left": DATA["s_neg"]}  # round the head from the cheek's centre (m)
V_UP = DATA["v"]  # up from the cheek's centre (m)
OVAL = tuple(float(x) for x in DATA["oval"])  # half-width, half-height of the target oval (m)
LIMIT = tuple(float(x) for x in DATA["limit"])
CHEEK_Z = BUST_Z + float(DATA["cheek_centre_pos"][2])
_CC = bust_to_world(DATA["cheek_centre_pos"])  # the right arm's cheek centre in the world
CHEEK_CENTRE = {"right": _CC, "left": np.array([_CC[0], 2 * BY - _CC[1], _CC[2]])}
_N = -np.array([DATA["cheek_normal_pos"][0], DATA["cheek_normal_pos"][1], 0.0])  # outward normal, world
OUTWARD = {"right": _N, "left": np.array([_N[0], -_N[1], 0.0])}


def _inside(s, v, half) -> np.ndarray:
    return (s / half[0]) ** 2 + (v / half[1]) ** 2 <= 1.0


IN_OVAL = {side: _inside(S_ARC[side], V_UP, OVAL) & (np.abs(S_ARC[side]) < 0.06) for side in S_ARC}
IN_LIMIT = _inside(S_ARC["right"], V_UP, LIMIT) | _inside(S_ARC["left"], V_UP, LIMIT)
OUT_LIMIT = ~IN_LIMIT
BRISTLES = DATA["bristles"]  # points over the bristle head's surface, brush frame (origin at the handle's butt)

# -- the brushes: Kent TWMU10-style round blush brush, 157 mm, handle 121 mm, ferrule 12 mm; one beside each arm,
# lying along the direction it will be pointed in, bristles towards the head
BRUSH_MASS = 0.012
BRUSH_L = 157 * MM
GRASP_AT = 0.065  # where the hand holds the handle, from its butt end (m)
YAW = float(np.arctan2(-OUTWARD["right"][1], -OUTWARD["right"][0]))  # the right brush points at its cheek: +yaw
BRUSH_YAW = {"right": YAW, "left": -YAW}
GRASP_XY = {"right": (0.20, -0.17), "left": (0.20, 0.17)}
TILT = float(np.arcsin(8 * MM / BRUSH_L))  # at rest the head (12 mm radius) is up on the table, the butt (4 mm) is not


def _brush(side: str) -> UsdBody:
    u = np.array([np.cos(BRUSH_YAW[side]), np.sin(BRUSH_YAW[side]), 0.0])
    axis = np.cos(TILT) * u + np.sin(TILT) * np.array([0.0, 0.0, 1.0])
    x = np.cross([0.0, 0.0, 1.0], axis)
    x = x / np.linalg.norm(x)
    R = np.column_stack([x, np.cross(axis, x), axis])  # brush z along its length
    q = _quat(R)
    gx, gy = GRASP_XY[side]
    butt = np.array([gx, gy, 0.0]) - GRASP_AT * u
    return UsdBody(f"brush_{side}", usd="assets/brush.usdc", pos=(float(butt[0]), float(butt[1]), TOP + 4 * MM),
                   rot_xyzw=q, size=(0.024, 0.024, BRUSH_L), mass=BRUSH_MASS, friction=(0.7, 0.6),
                   compliance=(5000.0, 5.0), solver_iters=32, report=True)


def _quat(R) -> tuple[float, float, float, float]:
    """A rotation matrix to a plain-float (x, y, z, w) quaternion."""
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0] * 4
        q[i] = s / 4
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s
    return tuple(float(c) for c in np.array(q) / np.linalg.norm(q))


BRUSHES = {"right": _brush("right"), "left": _brush("left")}
# -- the pans: 36 mm round magnetic tin pans (35.94 mm across, 3.94 mm tall; TKB Trading) of pressed blush, stood on
# edge in a small palette stand facing the arm (its face along OUTWARD, the same direction the cheek faces), so the
# brush loads it tip-first the same way it loads the cheek, instead of lying it flat and loading it on its side.
PAN_XY = {"right": (0.30, -0.24), "left": (0.30, 0.24)}
CAKE_TOP = 3.94 * MM
PAN_R = 18 * MM
PAN_RISE = PAN_R + 5 * MM  # the disc's centre height, so its lower rim clears the table


def pan_rotation(side: str) -> np.ndarray:
    """The pan stood on its edge, face normal along ``OUTWARD[side]`` (the cheek's own outward direction)."""
    u = np.array([np.cos(BRUSH_YAW[side]), np.sin(BRUSH_YAW[side]), 0.0])
    w = np.array([-u[1], u[0], 0.0])
    return np.column_stack([-w, np.array([0.0, 0.0, 1.0]), -u])


PANS = {side: UsdBody(f"pan_{side}", usd="assets/blush_pan.usdc", pos=(xy[0], xy[1], TOP + PAN_RISE),
                      rot_xyzw=_quat(pan_rotation(side)), mass=None, size=(0.036, CAKE_TOP, 0.036),
                      friction=(0.5, 0.4))
            for side, xy in PAN_XY.items()}
CAKE = {side: (np.asarray(DATA["cake"]) @ pan_rotation(side).T) + np.array([xy[0], xy[1], TOP + PAN_RISE])
        for side, xy in PAN_XY.items()}
# the cake's face, CAKE_TOP further out than the pan's own origin (at the back of the tin) along its outward normal
PAN_FACE = {side: np.array([xy[0], xy[1], TOP + PAN_RISE]) + CAKE_TOP * OUTWARD[side] for side, xy in PAN_XY.items()}
STANDS = {}
for _side, _xy in PAN_XY.items():
    _u = np.array([np.cos(BRUSH_YAW[_side]), np.sin(BRUSH_YAW[_side]), 0.0])
    STANDS[_side] = Shape(f"pan_stand_{_side}",
                          pos=(_xy[0] + 15 * MM * _u[0], _xy[1] + 15 * MM * _u[1], TOP + PAN_RISE / 2),
                          size=(0.03, 0.015, PAN_RISE), color=(0.85, 0.85, 0.85))
OBJECTS = [BUST, BRUSHES["right"], BRUSHES["left"], PANS["right"], PANS["left"], STANDS["right"], STANDS["left"]]

# -- pigment (modelled; needs contact and pressure): a brush starts clean. While its bristles rest on the cake (a
# bristle point within GAP of the cake) and press it with a force (smoothed over 0.1 s) of at least F_LOAD newtons,
# each cake point under the bristles (within GAP) gives it LOAD units a second, up to DOSE_MAX. While its bristles rest
# on the skin and press it with at least F_SKIN newtons, each skin point within GAP of the bristles takes RATE units
# a second from the dose, up to FULL
# units, until the dose is gone. A point shows pigment from SHOWS units on (0.33 s of contact, about the time a stroke
# at 7 cm/s needs to cross the 2.4 cm head). Hovering a few mm off, or touching without pressing, does nothing.
GAP = 2 * MM  # the bristle footprint once pressed (the brush's bristles splay under the press)
F_LOAD, F_SKIN = 0.3, 0.2  # N, the brush's total normal force on the pan / on the head
DOSE_MAX, LOAD, RATE, FULL, SHOWS = 400.0, 2.0, 3.0, 1.5, 1.0
# -- scoring
COVER_FULL, COVER_HALF = 0.80, 0.40  # the fraction of a cheek's oval that shows pigment
CLEAR = 20 * MM  # full credit needs both brushes' bristles at least this far from the skin at the end
BLEED_MAX = 6  # skin points (about 3.5 mm apart: under 1 cm2 in all) showing pigment outside both limits
_STATE: dict = {"dep": np.zeros(len(SKIN)), "dose": {"right": 0.0, "left": 0.0}, "t": 0.0, "last": -1.0, "clear": True, "f": {}, "dist": 1.0, "clear_s": 0.0}
_SKIN_TREE: list = []


def brush_points(brush) -> np.ndarray:
    """The bristle head's surface points in the world, from the brush's pose."""
    return np.asarray(brush.pos) + BRISTLES @ brush.rotation().T


def touch(side: str, brush, dt: float, f_pan: float, f_skin: float) -> None:
    """One control step of ``dt`` seconds, with the brush's contact force (N) on its pan and on the head: it loads
    from the cake under its bristles while pressing the pan, and moves pigment from its dose to the skin under them
    while pressing the head."""
    pts = brush_points(brush)
    centre = pts.mean(0)
    from scipy.spatial import cKDTree

    if f_pan >= F_LOAD and np.linalg.norm(CAKE[side].mean(0) - centre) < 0.06:
        d, _ = cKDTree(pts).query(CAKE[side], distance_upper_bound=GAP)
        n = int(np.isfinite(d).sum())
        _STATE["dose"][side] = min(DOSE_MAX, _STATE["dose"][side] + LOAD * dt * n)
    if _STATE["dose"][side] <= 0.0 or f_skin < F_SKIN:
        return
    near = np.flatnonzero(np.linalg.norm(SKIN - centre, axis=1) < 0.05)
    if not len(near):
        return
    d, _ = cKDTree(pts).query(SKIN[near], distance_upper_bound=GAP)
    hit = near[np.isfinite(d)]
    if not len(hit):
        return
    room = FULL - _STATE["dep"][hit]
    give = np.minimum(RATE * dt, np.maximum(room, 0.0))
    total = float(give.sum())
    if total > _STATE["dose"][side]:
        give *= _STATE["dose"][side] / total
        total = _STATE["dose"][side]
    _STATE["dep"][hit] += give
    _STATE["dose"][side] -= total


def nearest_skin(brush) -> float:
    """The distance from the bristles to the nearest skin point (m)."""
    from scipy.spatial import cKDTree

    if not _SKIN_TREE:
        _SKIN_TREE.append(cKDTree(SKIN))
    d, _ = _SKIN_TREE[0].query(brush_points(brush)[::6])
    return float(d.min())


def coverage(side: str, dep=None) -> float:
    """The fraction of the cheek's oval (skin points inside it) that shows pigment."""
    dep = _STATE["dep"] if dep is None else dep
    m = IN_OVAL[side]
    return float((dep[m] >= SHOWS).sum() / m.sum())


def bleed(dep=None) -> int:
    """Skin points showing pigment outside both cheeks' limits."""
    dep = _STATE["dep"] if dep is None else dep
    return int(((dep >= SHOWS) & OUT_LIMIT).sum())


def cheek_score(side: str, dep=None) -> float:
    c = coverage(side, dep)
    return 1.0 if c >= COVER_FULL else 0.5 if c >= COVER_HALF else 0.0


def score(objects=None, dep=None, clear=None) -> float:
    """The mean of the two cheeks' scores, each 1.0 for at least 80 % of the oval showing pigment, 0.5 for 40-80 %,
    0 below; but 0 for the whole run if more than 6 skin points (under 1 cm2) show pigment outside the limits, and
    at most 0.75 while a brush's bristles are still within 2 cm of the skin."""
    if bleed(dep) > BLEED_MAX:
        return 0.0
    total = (cheek_score("right", dep) + cheek_score("left", dep)) / 2
    return total if (_STATE["clear"] if clear is None else clear) else min(total, 0.75)


def success(objects) -> bool:
    return score() == 1.0


@on_reset
def _reset(environment) -> None:
    _STATE.update(dep=np.zeros(len(SKIN)), dose={"right": 0.0, "left": 0.0}, t=0.0, last=-1.0, clear=True, f={}, dist=1.0, clear_s=0.0)


def _bleed_where() -> str:
    m = (_STATE["dep"] >= SHOWS) & OUT_LIMIT
    if not m.any():
        return ""
    out = ""
    for side in ("right", "left"):
        k = m & (np.abs(S_ARC[side]) < 0.06)
        if k.any():
            out += f"[{side} s {1e3 * S_ARC[side][k].min():.0f}..{1e3 * S_ARC[side][k].max():.0f} v {1e3 * V_UP[k].min():.0f}..{1e3 * V_UP[k].max():.0f} mm]"
    return out


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states

    dt = environment.unwrapped.step_dt
    import contactkit

    states = object_states(environment, ["brush_right", "brush_left"])
    raw = contactkit.pressure(environment)
    f = {k: 0.5 * v + 0.5 * _STATE["f"].get(k, 0.0) for k, v in raw.items()}  # smoothed over about 0.1 s
    _STATE["f"] = f
    if os.environ.get("BLUSH_FORCES") and any(v > 0 for v in f.values()):
        print("FORCE t=%.2f " % _STATE["t"] + " ".join(f"{k[0]}_{k[1]}={v:.2f}" for k, v in f.items()), flush=True)
    for side in ("right", "left"):
        touch(side, states[f"brush_{side}"], dt, f[(side, "pan")], f[(side, "bust")])
    _STATE["dist"] = min(nearest_skin(states[f"brush_{side}"]) for side in ("right", "left"))
    _STATE["clear"] = _STATE["dist"] > CLEAR
    _STATE["clear_s"] = _STATE["clear_s"] + dt if _STATE["clear"] else 0.0
    _STATE["t"] += dt
    if os.environ.get("BLUSH_LOG") and _STATE["t"] - _STATE["last"] >= 0.5:
        _STATE["last"] = _STATE["t"]
        b = {s: (states[f"brush_{s}"].pos[2] - TOP) * 1000 for s in ("right", "left")}
        print(f"LOG t={_STATE['t']:.1f} cover_right={coverage('right'):.2f} cover_left={coverage('left'):.2f} "
              f"bleed={bleed()}{_bleed_where()} clear={_STATE['clear']} dose_right={_STATE['dose']['right']:.0f} dose_left={_STATE['dose']['left']:.0f} "
              f"brush_z_right={b['right']:.0f}mm brush_z_left={b['left']:.0f}mm score={score():.2f}", flush=True)


# -- the scripted demo: each arm picks up its brush, brings it square onto its cheek and sweeps the oval --------------
HOVER = TOP + 0.150
AHEAD = BRUSH_L - GRASP_AT  # from the hand to the bristle tip, along the brush
_GRID_S = DATA["grid_axis"]  # the cheek's surface on a 1 mm grid in (s, v), bust frame, for the +y cheek


def cheek_point(side: str, s: float, v: float) -> np.ndarray:
    """The skin point ``s`` metres round the head from the oval's centre (towards the ear: positive) and ``v`` up
    from it, on the given arm's cheek (world frame); bilinear on the stored 1 mm grid."""
    g = DATA["grid_pos"]
    fi = float(np.clip((s - _GRID_S[0]) / 0.001, 0, len(_GRID_S) - 1.001))
    fj = float(np.clip((v - _GRID_S[0]) / 0.001, 0, len(_GRID_S) - 1.001))
    i, j = int(fi), int(fj)
    a, b = fi - i, fj - j
    p = (g[i, j] * (1 - a) * (1 - b) + g[i + 1, j] * a * (1 - b) + g[i, j + 1] * (1 - a) * b
         + g[i + 1, j + 1] * a * b)
    if side == "left":
        p = np.array([p[0], -p[1], p[2]])
    return bust_to_world(p)


def hand_for(side: str, tip) -> tuple:
    """Where the hand (TCP) is with the bristle tip at ``tip``: the brush points along its yaw, tilted by TILT."""
    u = np.array([np.cos(BRUSH_YAW[side]), np.sin(BRUSH_YAW[side]), 0.0])
    h = np.asarray(tip) - AHEAD * u - np.array([0.0, 0.0, AHEAD * np.sin(TILT)])
    return tuple(float(x) for x in h)


def hand_for_dip(side: str, dw: float, dz: float, press: float) -> tuple:
    """The hand with the bristle tip on the pan's (upright) face, ``dw`` metres along its width and ``dz`` up from
    its centre, ``press`` metres into the cake along the inward normal (same convention as ``hand_for``/``on``:
    the pan now faces the arm the same way the cheek does, so this is the same tip-first contact, not the cheek)."""
    u = np.array([np.cos(BRUSH_YAW[side]), np.sin(BRUSH_YAW[side]), 0.0])
    w = np.array([-u[1], u[0], 0.0])
    face_point = PAN_FACE[side] + dw * w + dz * np.array([0.0, 0.0, 1.0]) + press * u
    return hand_for(side, face_point)


def demo():
    """Each arm: over its brush, down, grip the handle, lift, carry it to its (upright) pan, press the bristle tip
    in and swirl it (a small square pattern), lift, carry it to a staging point 4.5 cm off its cheek, press the
    bristles in 1 mm, sweep three passes across the oval (v = +4, 0, -4 mm, s = -13 to +13 mm), pressing 4 mm, lift off."""
    from tiltkit import grip, install
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    install()
    plans = {}
    for side in ("right", "left"):
        u = np.array([np.cos(BRUSH_YAW[side]), np.sin(BRUSH_YAW[side]), 0.0])
        w = np.array([-u[1], u[0], 0.0])
        yaw = grip(approach=(0, 0, -1), closing=-w)
        gx, gy = GRASP_XY[side]
        gz = TOP + 0.009  # the handle's axis there
        sy = -0.30 if side == "right" else 0.30
        SWIRL = 4 * MM  # keeps the 12 mm bristle head well inside the 18 mm pan

        def on(s, v, press):  # the hand with the tip ``press`` metres into the skin along the brush
            return hand_for(side, cheek_point(side, s, v) + u * press)

        def dip(dw, dz, press):  # the hand with the tip ``press`` metres into the cake, on the pan's face
            return hand_for_dip(side, dw, dz, press)

        plans[side] = [
            W(2.0, (0.20, sy, HOVER), O, "start", 0.0),
            W(1.5, (gx, gy, HOVER), -0.020, "over the brush", yaw),
            W(1.5, (gx, gy, gz), -0.020, "down beside the handle", yaw),
            W(0.8, (gx, gy, gz), -0.003, "grip the handle", yaw),
            W(1.5, (gx, gy, HOVER), -0.003, "lift", yaw),
            W(1.5, dip(0.0, 0.0, -0.025), -0.003, "over the pan", yaw),
            W(1.0, dip(0.0, 0.0, 0.0), -0.003, "bristles onto the cake", yaw),
            W(0.8, dip(SWIRL, 0.0, 0.0), -0.003, "swirl", yaw),
            W(0.8, dip(0.0, SWIRL, 0.0), -0.003, "swirl", yaw),
            W(0.8, dip(-SWIRL, 0.0, 0.0), -0.003, "swirl", yaw),
            W(0.8, dip(0.0, -SWIRL, 0.0), -0.003, "swirl", yaw),
            W(1.0, dip(0.0, 0.0, -0.030), -0.003, "lift off the cake", yaw),
            W(2.0, on(-13 * MM, 4 * MM, -0.045), -0.003, "carry to the cheek", yaw),
            W(2.0, on(-13 * MM, 4 * MM, 0.004), -0.003, "bristles on the cheek", yaw),
            W(2.5, on(13 * MM, 4 * MM, 0.004), -0.003, "sweep 1", yaw),
            W(0.8, on(13 * MM, 0 * MM, 0.004), -0.003, "down", yaw),
            W(2.5, on(-13 * MM, 0 * MM, 0.004), -0.003, "sweep 2", yaw),
            W(0.8, on(-13 * MM, -4 * MM, 0.004), -0.003, "down", yaw),
            W(2.5, on(13 * MM, -4 * MM, 0.004), -0.003, "sweep 3", yaw),
            W(1.0, on(13 * MM, -4 * MM, -0.045), -0.003, "lift off", yaw),
            W(2.0, (0.20, sy, HOVER), -0.003, "back", yaw),
        ]
    return plans


def self_test(ObjectState=None):
    """The scoring rules on hand-made pigment layouts."""
    n = len(SKIN)

    def paint(sides=(), extra=None, frac=1.0, rng=np.random.default_rng(0)):
        dep = np.zeros(n)
        for side in sides:
            idx = np.flatnonzero(IN_OVAL[side])
            keep = rng.permutation(idx)[: int(round(frac * len(idx)))] if frac < 1.0 else idx
            dep[keep] = FULL
        if extra is not None:
            dep[extra] = FULL
        return dep

    far = np.flatnonzero(OUT_LIMIT)
    ring = np.flatnonzero(IN_LIMIT & ~IN_OVAL["right"] & ~IN_OVAL["left"])
    cases = {
        "nothing": (paint(), 0.0),
        "both ovals covered": (paint(("right", "left")), 1.0),
        "one oval covered": (paint(("right",)), 0.5),
        "both ovals 85 %": (paint(("right", "left"), frac=0.85), 1.0),
        "both ovals 60 %": (paint(("right", "left"), frac=0.60), 0.5),
        "both ovals 30 %": (paint(("right", "left"), frac=0.30), 0.0),
        "right 85 %, left 60 %": (paint(("right",), frac=0.85) + paint(("left",), frac=0.60), 0.75),
        "covered, 5 specks past the limit": (paint(("right", "left"), extra=far[:5]), 1.0),
        "covered, 20 specks past the limit": (paint(("right", "left"), extra=far[:20]), 0.0),
        "covered, a smear inside the limit": (paint(("right", "left"), extra=ring), 1.0),
        "forehead smeared only": (paint(extra=far[:500]), 0.0),
        "faint pigment (under the shows level)": (paint(("right", "left")) * (SHOWS * 0.9 / FULL), 0.0),
    }
    for name, (dep, want) in cases.items():
        got = score(dep=dep)
        assert abs(got - want) < 1e-9, (name, got, want)
    assert score(dep=paint(("right", "left")), clear=False) == 0.75  # both ovals covered, a brush still on the skin
    assert score(dep=paint(("right",)), clear=False) == 0.5
    assert score(dep=paint(), clear=False) == 0.0

    # a hand-made brush at the right cheek's centre, square on it: it takes pigment; a brush held off does not
    class Brush:
        def __init__(self, pos, R):
            self.pos, self._R = pos, R

        def rotation(self):
            return self._R

    u = -OUTWARD["right"]
    z = np.array([0.0, 0.0, 1.0])
    x = np.cross(z, u)
    x /= np.linalg.norm(x)
    R = np.column_stack([x, np.cross(u, x), u])
    FS, FP = 2 * F_SKIN, 2 * F_LOAD  # a firm press
    tip = CHEEK_CENTRE["right"] - 0.2 * MM * u  # resting on the skin
    pos = tip - BRUSH_L * u
    _reset(None)
    _STATE["dose"]["right"] = DOSE_MAX
    for _ in range(20):
        touch("right", Brush(pos, R), 0.05, 0.0, FS)
    assert coverage("right") > 0.05, coverage("right")
    assert _STATE["dose"]["right"] < DOSE_MAX
    for name, off, f in (("held 2 cm off", 20 * MM, FS), ("3 mm off, the handle pressing the head", 3 * MM, FS),
                         ("hovering 1 mm off", 1.0 * MM, 0.0), ("touching, pressing too lightly", 0.0, 0.5 * F_SKIN),
                         ("touching, no press", 0.0, 0.0)):
        _reset(None)
        _STATE["dose"]["right"] = DOSE_MAX
        for _ in range(20):
            touch("right", Brush(pos - off * u, R), 0.05, 0.0, f)
        assert not (_STATE["dep"] > 0).any(), name
    _reset(None)
    for _ in range(20):
        touch("right", Brush(pos, R), 0.05, 0.0, FS)  # a dry brush on the skin
    assert not (_STATE["dep"] > 0).any()

    # a brush pressed on its own cake loads (its bristle tip square on the pan's face, the same way it touches the
    # cheek: the pan stands facing the same direction, OUTWARD), and only its own pan's
    tip_d = PAN_FACE["right"] + 0.2 * MM * 0  # the cake's face
    pos_d = tip_d - BRUSH_L * u
    _reset(None)
    for _ in range(80):
        touch("right", Brush(pos_d, R), 0.05, FP, 0.0)
    assert _STATE["dose"]["right"] >= 0.25 * DOSE_MAX, _STATE["dose"]
    for name, off, f in (("hovering 1.5 mm over the cake", 1.5 * MM, 0.0), ("4 mm over the cake, the handle pressing it", 4 * MM, FP),
                         ("on the cake, pressing too lightly", 0.0, 0.5 * F_LOAD), ("on the cake, no press", 0.0, 0.0)):
        _reset(None)
        for _ in range(80):
            touch("right", Brush(pos_d - off * u, R), 0.05, f, 0.0)
        assert _STATE["dose"]["right"] == 0.0, (name, _STATE["dose"])
    _reset(None)
    for _ in range(40):
        touch("left", Brush(pos_d, R), 0.05, FP, 0.0)
    assert _STATE["dose"]["left"] == 0.0
    _reset(None)
    for _ in range(3):  # a short dip loads part of a dose
        touch("right", Brush(pos_d, R), 0.05, FP, 0.0)
    assert 0.0 < _STATE["dose"]["right"] < DOSE_MAX, _STATE["dose"]
    _reset(None)
    # stated sizes against each other
    assert 35.94 * MM > 24 * MM  # the pan is wider than the bristle head
    assert OVAL[0] * 2 > 24 * MM and OVAL[1] * 2 > 24 * MM * 0.9  # the oval is at least as wide as the head
    assert LIMIT[0] > OVAL[0] and LIMIT[1] > OVAL[1]
    assert 10 * MM < 0.094  # the handle (10 mm) fits the gripper's 94 mm opening
    assert abs(CHEEK_Z - TOP - 0.167) < 0.003  # the cheek's centre 167 mm above the table
    assert all(np.linalg.norm(np.asarray(PAN_XY[s]) - np.asarray(GRASP_XY[s])) > 0.08 for s in PAN_XY)
    return len(cases) + 20


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules, then each arm's plan through the kinematics at 20 Hz.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    print(f"skin points {len(SKIN)}; oval points right {IN_OVAL['right'].sum()}, left {IN_OVAL['left'].sum()}; "
          f"outside both limits {OUT_LIMIT.sum()}")
    task = load(__file__)
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / 20)[0] for k in range(round(plan.duration * 20) + 5)])
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
