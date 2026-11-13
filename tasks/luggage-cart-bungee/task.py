"""On the station table: a folding steel luggage cart stands upright facing the arms, its nose plate flat on the
table and its handle extended; a corrugated carton stands on the nose plate against the frame. A yellow bungee cord
with two coated steel hooks lies straight across the table in front of the cart. The robot must strap the carton to
the cart with the cord: one hook on each side tube of the frame, the cord stretched round the front of the carton.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import add_forces, gripper_points, on_reset, on_step, on_substep, part, pose, set_pose  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0           # the scorer's own test decides; success ends the trial at once
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Strap the carton onto the luggage cart with the bungee cord: hook one end on each side tube of the cart's fram"
               "e, with the cord stretched round the front of the carton at about half its height, then let go of the cord.")

STEEL = (0.62, 0.64, 0.67)
DARK = (0.12, 0.12, 0.13)
RUBBER = (0.08, 0.08, 0.08)
KRAFT = (0.72, 0.55, 0.36)
TAPE = (0.80, 0.70, 0.52)
CORD = (0.95, 0.72, 0.08)      # yellow elastic cord
HOOK = (0.85, 0.20, 0.12)      # red plastic-coated steel hooks

# -- the cart: a folding steel luggage cart, in m, world frame (x forward from the arms, y left, z up) -----------
XF = 0.44                      # front face of the frame (the carton's back stands against it)
RAIL_R = 0.008                 # 16 mm steel side tubes
RAIL_X = XF + RAIL_R
RAIL_Y = 0.17                  # tube centres 34 cm apart
PLATE_T = 0.004
PLATE_Z = TOP + 0.005          # top of the nose plate (it stands on two 1 mm feet)
PLATE_D, PLATE_W = 0.26, 0.38  # nose plate depth (x) and width (y)
RAIL_TOP = PLATE_Z + 0.60      # outer tubes; the telescoping handle continues to 1 m
HANDLE_TOP = TOP + 1.00
WHEEL_R, WHEEL_W, WHEEL_Y = 0.05, 0.025, 0.235
AXLE = np.array([XF + 0.045, 0.0, TOP + WHEEL_R])     # the cart body's origin: the middle of the axle
CROSSBARS = (0.03, 0.10, 0.40)                         # heights of the cross tubes above the nose plate

# -- the carton (free) --------------------------------------------------------------------------------------------
BOX_D, BOX_W, BOX_H = 0.22, 0.24, 0.24
BOX_MASS = 1.2
BOX_START = np.array([XF - BOX_D / 2 - 0.002, 0.0, PLATE_Z + BOX_H / 2 + 0.0005])
CARD_FRICTION = 0.35           # corrugated board on steel (estimate)

# -- the hooks: plate-shaped J-hooks of 6 mm coated steel wire with a crimp sleeve where the cord goes in ----------
# Hook frame: origin where the tube's centre sits when the hook is home in its bend; x' along the shank toward the
# cord; the shank at y' = +G, the tip at y' = -G; the hook lies in its x'y' plane.
W = 0.006
G = 0.013                      # half the distance between shank and tip centres: 20 mm clear opening
BEND_IN = -0.009               # inside of the bend (the tube's radius plus 1 to 2 mm)
TIP_END = 0.014
SHANK_END = 0.044
CRIMP = np.array([0.057, G, 0.0])      # crimp sleeve centre (pressed flat: 10 x 10 mm, 26 mm long)
EYE = np.array([0.070, G, 0.0])        # where the cord leaves the sleeve
def _bend_segments():
    """The hook's bend: a half circle of wire (centre-line radius G) round the tube's seat, as four straight pieces."""
    out = []
    for k in range(4):
        a = math.radians(112.5 + 45.0 * k)                 # from the shank side (90 deg) round to the tip side (270)
        rc = G * math.cos(math.radians(22.5))
        chord = 2 * G * math.sin(math.radians(22.5))
        yaw = a + math.pi / 2                              # piece along the tangent
        q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        out.append(((rc * math.cos(a), rc * math.sin(a), 0.0), (chord + 0.002, W, W), HOOK, q))
    return tuple(out)


HOOK_BOXES = (
    *_bend_segments(),                                                                             # the bend
    ((SHANK_END / 2, G, 0.0), (SHANK_END, W, W), HOOK),                                            # the shank
    ((TIP_END / 2, -G, 0.0), (TIP_END, W, W), HOOK),                                               # the tip
    (tuple(CRIMP), (0.026, 0.010, 0.010), STEEL),                                                  # crimp sleeve
)
HOOK_CYL = ()
HOOK_MASS = 0.03               # 20 g hook with its crimp, plus half the cord's own 20 g
HOOK_Z = TOP + 0.005           # lying flat on the table
HOOK_START = {"hook_left": (np.array([0.30, 0.235, HOOK_Z]), -1), "hook_right": (np.array([0.30, -0.235, HOOK_Z]), 1)}

# -- the cord: a 24 in (61 cm over the hooks) bungee cord, 6 mm; 50 cm between the crimp sleeves when relaxed -------
CORD_REST = 0.50
CORD_K = 112.0                 # N/m: about 56 N to double its length (estimate: an 8 mm cord's rated 100 N, scaled
                               # by cross-section to 6 mm; see README)
CORD_TAU = 0.02                # s: the cord's pull follows its stretch with this lag (its mass and internal damping)
CORD_ALPHA = (1 / 240) / (CORD_TAU + 1 / 240)
N_SEG = 24
SEG_LEN = CORD_REST / N_SEG    # drawn as pieces of this length: gaps open between them as the cord stretches

BAND = (0.06, 0.18)            # the cord must cross the carton's front edges this high above the nose plate


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def _rot(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _quat_from_matrix(m) -> tuple:
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s)
    q = np.array(q)
    return tuple((q / np.linalg.norm(q)).tolist())


def hook_rotation(xdir, zdir) -> np.ndarray:
    """Rotation of a hook whose shank points along ``xdir`` and whose plane normal is ``zdir``."""
    x = np.asarray(xdir, float) / np.linalg.norm(xdir)
    z = np.asarray(zdir, float)
    z = z - np.dot(z, x) * x
    z /= np.linalg.norm(z)
    return np.column_stack([x, np.cross(z, x), z])


def hook_start(name: str) -> tuple[np.ndarray, tuple]:
    """Lying flat on the table, shank (and cord) pointing toward the arms, shank on the outer side."""
    p, s = HOOK_START[name]
    xdir = np.array([-1.0, 0.0, 0.0])
    zdir = np.array([0.0, 0.0, 1.0 if name == "hook_right" else -1.0])
    return p.copy(), _quat_from_matrix(hook_rotation(xdir, zdir))


# -- the cart body (kinematic): in its own frame, origin at the axle's middle -------------------------------------
def _cart_shapes():
    ax = AXLE
    boxes, cyl = [], []

    def rel(p):
        return tuple((np.asarray(p, float) - ax).tolist())

    # nose plate, with two stiffening ribs under it toward the tubes
    boxes.append((rel((XF - PLATE_D / 2 + 0.016, 0.0, PLATE_Z - PLATE_T / 2)), (PLATE_D, PLATE_W, PLATE_T), STEEL))
    for s in (1, -1):
        y = s * RAIL_Y
        cyl.append((rel((RAIL_X, y, (PLATE_Z + RAIL_TOP) / 2)), RAIL_R, RAIL_TOP - PLATE_Z, STEEL, "Z"))
        cyl.append((rel((RAIL_X, y, (RAIL_TOP + HANDLE_TOP) / 2)), 0.0065, HANDLE_TOP - RAIL_TOP, STEEL, "Z"))
        cyl.append((rel((AXLE[0], s * WHEEL_Y, AXLE[2])), WHEEL_R, WHEEL_W, RUBBER, "Y"))
        cyl.append((rel((AXLE[0], s * WHEEL_Y, AXLE[2])), 0.02, WHEEL_W + 0.002, STEEL, "Y"))         # hub
        # bracket from the tube's foot back to the axle
        foot = np.array([RAIL_X, y, PLATE_Z + 0.01])
        d = AXLE + np.array([0.0, y, 0.0]) - foot
        boxes.append((rel(foot + d / 2), (abs(d[0]) + 0.012, 0.010, abs(d[2]) + 0.012), DARK))
    cyl.append((rel(AXLE), 0.006, 2 * WHEEL_Y, STEEL, "Y"))
    for h in CROSSBARS:
        cyl.append((rel((RAIL_X, 0.0, PLATE_Z + h)), RAIL_R, 2 * RAIL_Y, STEEL, "Y"))
    cyl.append((rel((RAIL_X, 0.0, HANDLE_TOP - 0.012)), 0.013, 2 * RAIL_Y + 0.03, RUBBER, "Y"))     # handle grip
    return tuple(boxes), tuple(cyl)


def _carton():
    boxes = (((0.0, 0.0, 0.0), (BOX_D, BOX_W, BOX_H), KRAFT),
             ((0.0, 0.0, BOX_H / 2 + 0.0005), (BOX_D + 0.002, 0.05, 0.001), TAPE))
    return part("carton", pos=tuple(BOX_START), boxes=boxes, mass=BOX_MASS, color=KRAFT, friction=CARD_FRICTION,
                damping=0.5)


def _hooks():
    out = []
    for name in HOOK_START:
        p, q = hook_start(name)
        out.append(part(name, pos=tuple(p), boxes=HOOK_BOXES, cylinders=HOOK_CYL, mass=HOOK_MASS, quat=q,
                        color=HOOK, friction=1.0, damping=6.0))
    return out


def _segments():
    seg = (((0.0, 0.0, 0.0), (SEG_LEN, 0.006, 0.006), CORD),)
    a, b = (hook_start(n)[0] + _rot(hook_start(n)[1]) @ EYE for n in HOOK_START)
    out = []
    for i in range(N_SEG):
        p = a + (b - a) * (i + 0.5) / N_SEG
        out.append(part(f"cord_{i:02d}", pos=tuple(p), boxes=seg, mass=0.001, color=CORD,
                        quat=_quat_from_matrix(hook_rotation(b - a, (0.0, 0.0, 1.0))), ghost=True))
    return out


_CB, _CC = _cart_shapes()
OBJECTS = [part("cart", pos=tuple(AXLE), boxes=_CB, cylinders=_CC, mass=2.6, color=STEEL, friction=CARD_FRICTION,
                fixed=True), _carton(), *_hooks(), *_segments()]


# -- cart pose: fixed while the robot works (a bracket over the axle stands in for a foot on it); the scorer's
# tip-back test moves it --------------------------------------------------------------------------------------------
def cart_rotation(tilt: float, rock: float) -> np.ndarray:
    """Tilted back ``tilt`` rad about the axle, then rocked ``rock`` rad about the frame's own long axis."""
    c, s = math.cos(tilt), math.sin(tilt)
    ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])          # +tilt: the handle goes back (+x), the plate up
    axis = ry @ np.array([0.0, 0.0, 1.0])                      # the frame's long axis after the tilt
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    rk = np.eye(3) + math.sin(rock) * k + (1 - math.cos(rock)) * (k @ k)
    return rk @ ry


def cart_pose(tilt: float, rock: float, fwd: float, lift: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """(origin, rotation) of the cart body: lifted ``lift`` m by the handle, then tilted back about the axle and
    rocked, or tipped forward ``fwd`` rad about the axle."""
    o = AXLE + np.array([0.0, 0.0, lift])
    if fwd > 0:
        c, s = math.cos(fwd), math.sin(fwd)
        return o, np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])     # the handle goes forward (-x), the plate down
    return o, cart_rotation(tilt, rock)


def cart_frame(environment=None) -> tuple[np.ndarray, np.ndarray]:
    """(origin, rotation) of the cart body now."""
    return cart_pose(_S.get("tilt", 0.0), _S.get("rock", 0.0), _S.get("fwd", 0.0), _S.get("lift", 0.0))


def to_cart(p, frame=None) -> np.ndarray:
    o, r = frame if frame is not None else cart_frame()
    return r.T @ (np.asarray(p, float) - o) + AXLE


# -- the cord model -----------------------------------------------------------------------------------------------
# A massless elastic line from one hook's eye to the other's. While it is round the front of the carton (its start
# state: it lies in front of the cart), seen from above it runs straight unless the straight line would cross the
# carton, in which case it wraps the carton's front vertical edges; along the way its height changes evenly with
# length. If the height where it would cross a front edge is above the carton's top, it rides up over the top
# ("over" mode): it then crosses the top where the straight line seen from above meets the carton's outline, until
# both eyes are again below the top and in front of the carton. Tension = CORD_K x stretch (none when slack), on
# both eyes along the cord and on the carton at every edge it bends round.
_S: dict = {}


@on_reset
def _reset(environment=None) -> None:
    _S.update(mode="front", length=None, tension=0.0, path=None, wrapped=(), tilt=0.0, rock=0.0, fwd=0.0, lift=0.0, phase="work",
              ready_steps=0, test_t=0.0, test_ok=True, test_note="", box_ref=None, score=0.0, passed=False,
              hands_off=False, hook_seated={}, sub=0, max_shift=0.0, still_hist=[])


_reset()


def _box_frame(bp, bq):
    return np.asarray(bp, float), _rot(bq)


def _seg_hits_box(p, q, half_x, half_y, shrink=1e-4) -> bool:
    """Does the 2D segment p-q (carton frame) pass through the rectangle |x|<half_x, |y|<half_y?"""
    hx, hy = half_x - shrink, half_y - shrink
    d = q - p
    t0, t1 = 0.0, 1.0
    for k, h in ((0, hx), (1, hy)):
        if abs(d[k]) < 1e-12:
            if abs(p[k]) >= h:
                return False
            continue
        a, b = (-h - p[k]) / d[k], (h - p[k]) / d[k]
        lo, hi = min(a, b), max(a, b)
        t0, t1 = max(t0, lo), min(t1, hi)
        if t0 >= t1:
            return False
    return True


def _seg_box_crossings(p, q, half_x, half_y):
    """Entry and exit points of the 2D segment p-q through the rectangle, or None."""
    d = q - p
    t0, t1 = 0.0, 1.0
    for k, h in ((0, half_x), (1, half_y)):
        if abs(d[k]) < 1e-12:
            if abs(p[k]) >= h:
                return None
            continue
        a, b = (-h - p[k]) / d[k], (h - p[k]) / d[k]
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
        if t0 >= t1:
            return None
    return p + t0 * d, p + t1 * d


def cord_path(a, b, box_p, box_q, mode: str):
    """The cord's corner points (world), the indices of points where it bends round the carton, and its mode."""
    c, r = _box_frame(box_p, box_q)
    la, lb = r.T @ (np.asarray(a) - c), r.T @ (np.asarray(b) - c)
    hx, hy, hz = BOX_D / 2, BOX_W / 2, BOX_H / 2
    a2, b2 = la[:2], lb[:2]
    pts2 = [a2, b2]
    if mode == "front":
        if _seg_hits_box(a2, b2, hx, hy):
            fl, fr = np.array([-hx, hy]), np.array([-hx, -hy])
            pts2 = [a2, fl, fr, b2] if a2[1] >= b2[1] else [a2, fr, fl, b2]
            changed = True
            while changed:                      # drop corners the taut cord does not bend round
                changed = False
                for i in range(1, len(pts2) - 1):
                    if not _seg_hits_box(pts2[i - 1], pts2[i + 1], hx, hy, shrink=1e-3):
                        pts2.pop(i)
                        changed = True
                        break
        s = np.concatenate([[0.0], np.cumsum([np.linalg.norm(q - p) for p, q in zip(pts2, pts2[1:])])])
        S = max(s[-1], 1e-9)
        zs = la[2] + (lb[2] - la[2]) * s / S
        if len(pts2) > 2 and np.any(zs[1:-1] > hz + 0.01):     # pulled more than 1 cm above the top: rides over
            mode = "over"
        else:
            local = [np.array([*p, z]) for p, z in zip(pts2, zs)]
            world = [c + r @ p for p in local]
            return world, tuple(range(1, len(world) - 1)), mode, local
    # over the top
    if la[2] < hz and lb[2] < hz and la[0] < -hx and lb[0] < -hx:
        return cord_path(a, b, box_p, box_q, "front")
    cross = _seg_box_crossings(a2, b2, hx, hy)
    if cross is None or min(la[2], lb[2]) >= hz:
        local = [la, lb]
    else:
        local = [la, np.array([*cross[0], hz]), np.array([*cross[1], hz]), lb]
    world = [c + r @ p for p in local]
    return world, tuple(range(1, len(world) - 1)), "over", local


def _length(pts) -> float:
    return float(sum(np.linalg.norm(q - p) for p, q in zip(pts, pts[1:])))


def _eye(environment, name):
    p, q, _ = pose(environment, name)
    return p + _rot(q) @ EYE


# the scorer's test, as a person checks a strapped load by the handle: tilt back 45 deg on the wheels (2 s), hold 1 s;
# rock +20 deg about the frame's long axis (1.5 s), hold 1 s, to -20 deg (3 s), hold 1 s, back (1.5 s); stand up (2 s);
# lift 15 cm (1 s); tip forward 30 deg about the axle (1.5 s), hold 2 s, back upright (1.5 s), hold 1 s. The verdict is
# taken there, with the cart still held up (setting it down is not part of the test).
TILT, ROCK, FWD, LIFT = math.radians(45), math.radians(20), math.radians(30), 0.15
_PLAN = ((2.0, (0, 0, 0, 0), (1, 0, 0, 0)), (1.0, (1, 0, 0, 0), (1, 0, 0, 0)), (1.5, (1, 0, 0, 0), (1, 1, 0, 0)),
         (1.0, (1, 1, 0, 0), (1, 1, 0, 0)), (3.0, (1, 1, 0, 0), (1, -1, 0, 0)), (1.0, (1, -1, 0, 0), (1, -1, 0, 0)),
         (1.5, (1, -1, 0, 0), (1, 0, 0, 0)), (2.0, (1, 0, 0, 0), (0, 0, 0, 0)), (1.0, (0, 0, 0, 0), (0, 0, 0, 1)),
         (1.5, (0, 0, 0, 1), (0, 0, 1, 1)), (2.0, (0, 0, 1, 1), (0, 0, 1, 1)), (1.5, (0, 0, 1, 1), (0, 0, 0, 1)),
         (1.0, (0, 0, 0, 1), (0, 0, 0, 1)))
TEST_S = sum(p[0] for p in _PLAN)
SHIFT_LIMIT = 0.04            # m: the carton may move this far on the nose plate during the test


def _test_pose(t: float) -> tuple[float, float, float, float]:
    for dur, a, b in _PLAN:
        if t <= dur:
            u = t / dur
            u = u * u * (3 - 2 * u)
            return tuple(k * (x + (y - x) * u) for k, x, y in zip((TILT, ROCK, FWD, LIFT), a, b))
        t -= dur
    return tuple(k * v for k, v in zip((TILT, ROCK, FWD, LIFT), _PLAN[-1][2]))     # the end: held up, upright


@on_substep
def _pull(environment) -> None:
    _S["sub"] += 1
    if _S["phase"] == "test":                       # the scorer moves the cart through the test, 240 times a second
        _S["test_t"] += 1.0 / 240.0
        _S["tilt"], _S["rock"], _S["fwd"], _S["lift"] = _test_pose(_S["test_t"])
        o, r = cart_frame()
        set_pose(environment, "cart", o, _quat_from_matrix(r))
    a, b = _eye(environment, "hook_left"), _eye(environment, "hook_right")
    bp, bq, _ = pose(environment, "carton")
    pts, wrapped, mode, _ = cord_path(a, b, bp, bq, _S["mode"])
    _S["mode"], _S["path"], _S["wrapped"] = mode, pts, wrapped
    L = _length(pts)
    _S["length"] = L
    raw = CORD_K * (L - CORD_REST) if L > CORD_REST else 0.0
    t = _S["tension"] + (raw - _S["tension"]) * CORD_ALPHA       # the cord's own lag (its mass and damping)
    _S["tension"] = t
    for name, p0, p1 in (("hook_left", pts[0], pts[1]), ("hook_right", pts[-1], pts[-2])):
        d = p1 - p0
        n = np.linalg.norm(d)
        # the cord's pull acts on the hook's centre line (crimp, shank and bend share one load line)
        hp, hq, _ = pose(environment, name)
        on_line = hp + _rot(hq) @ np.array([EYE[0], 0.0, 0.0])
        add_forces(environment, name, [(t * d / n if n > 1e-6 else np.zeros(3), on_line)])
    box = []
    for i in wrapped:
        u1, u2 = pts[i - 1] - pts[i], pts[i + 1] - pts[i]
        f = t * (u1 / max(np.linalg.norm(u1), 1e-9) + u2 / max(np.linalg.norm(u2), 1e-9))
        box.append((f, pts[i]))
    add_forces(environment, "carton", box or [(np.zeros(3), bp)])


# -- what can be seen and scored ------------------------------------------------------------------------------------
def seated(name: str, environment) -> str | None:
    """Which side tube ('left' or 'right') this hook is home on: the tube's centre inside the hook's bend (between
    the bend and the end of the tip, between shank and tip), the hook lying within 35 degrees of flat, on the tube
    between the nose plate and the tube's top."""
    p, q, _ = pose(environment, name)
    r = _rot(q)
    o, cr = cart_frame()
    for side, s in (("left", 1), ("right", -1)):
        foot = o + cr @ (np.array([RAIL_X, s * RAIL_Y, PLATE_Z]) - AXLE)
        axis = cr @ np.array([0.0, 0.0, 1.0])
        h = float(np.dot(p - foot, axis))
        if not 0.005 < h < RAIL_TOP - PLATE_Z:
            continue
        if abs(float(np.dot(r[:, 2], axis))) < math.cos(math.radians(35)):
            continue
        t = r.T @ (foot + h * axis - p)            # the tube's centre at the hook's height, in the hook frame
        if BEND_IN - 0.002 < t[0] < TIP_END and abs(t[1]) < G - W / 2 + 0.001:
            return side
    return None


def front_crossing(local_pts) -> list[float]:
    """Heights above the nose plate (in the carton frame: above its bottom) where the cord crosses the carton's
    front edges, if it is wrapped round the front."""
    hx = BOX_D / 2
    out = []
    for p in local_pts[1:-1]:
        if abs(p[0] + hx) < 1e-6 and abs(abs(p[1]) - BOX_W / 2) < 1e-6:
            out.append(float(p[2] + BOX_H / 2))
    return out


def _carton_on_plate(bp, bq) -> tuple[bool, np.ndarray]:
    """Carton bottom centre in the cart frame (start layout coordinates), and whether it stands on the nose plate."""
    r = _rot(bq)
    bottom = np.asarray(bp) - r[:, 2] * BOX_H / 2
    local = to_cart(bottom)
    on = (XF - PLATE_D + 0.016 - 0.01 < local[0] < XF + 0.01 and abs(local[1]) < PLATE_W / 2
          and abs(local[2] - PLATE_Z) < 0.02 and float(np.dot(cart_frame()[1][:, 2], r[:, 2])) > math.cos(0.35))
    return on, local


def _clear_of(points, environment) -> bool:
    """Every gripper point at least 3 cm from both hooks and the carton, and 2 cm from the cord."""
    hooks = []
    for n in HOOK_START:
        p, q, _ = pose(environment, n)
        r = _rot(q)
        hooks += [p + r @ np.array(v) for v in ((BEND_IN, 0, 0), (0.0, G, 0), (0.0, -G, 0), (TIP_END, -G, 0),
                                                 tuple(CRIMP), tuple(EYE), (0.03, G, 0))]
    bp, bq, _ = pose(environment, "carton")
    c, r = _box_frame(bp, bq)
    for g in points:
        if min(np.linalg.norm(g - h) for h in hooks) < 0.03:
            return False
        lo = np.abs(r.T @ (g - c)) - np.array([BOX_D, BOX_W, BOX_H]) / 2
        if np.linalg.norm(np.maximum(lo, 0.0)) < 0.03:
            return False
        for a, b in zip(_S["path"], _S["path"][1:]):
            d = b - a
            u = float(np.clip(np.dot(g - a, d) / max(np.dot(d, d), 1e-12), 0, 1))
            if np.linalg.norm(a + u * d - g) < 0.02:
                return False
    return True


def evaluate(environment) -> dict:
    """The scorer's view of the scene now."""
    seats = {n: seated(n, environment) for n in HOOK_START}
    bp, bq, bv = pose(environment, "carton")
    on_plate, bottom = _carton_on_plate(bp, bq)
    a, b = _eye(environment, "hook_left"), _eye(environment, "hook_right")
    _, _, mode, local = cord_path(a, b, bp, bq, _S["mode"])
    cross = front_crossing(local) if mode == "front" else []
    round_front = len(cross) == 2
    in_band = round_front and all(BAND[0] <= h <= BAND[1] for h in cross)
    grips = gripper_points(environment)
    hands_off = all(_clear_of(pts, environment) for pts, _ in grips.values())
    # still: hooks and carton moved less than 3 mm over the last 0.5 s (10 control steps)
    now = np.concatenate([pose(environment, n)[0] for n in (*HOOK_START, "carton")])
    hist = _S.setdefault("still_hist", [])
    hist.append(now)
    del hist[:-11]
    still = len(hist) == 11 and float(np.max(np.abs(hist[-1] - hist[0]))) < 0.003
    both = sorted(s for s in seats.values() if s) == ["left", "right"]
    return dict(seats=seats, on_plate=on_plate, bottom=bottom, round_front=round_front, cross=cross, in_band=in_band,
                hands_off=hands_off, still=still, both=both, tension=_S["tension"], mode=mode)


@on_step
def _watch(environment) -> None:
    ev = evaluate(environment)
    _S["last"] = ev
    _S["hands_off"] = ev["hands_off"]
    # draw the cord
    path = _S["path"] or [_eye(environment, "hook_left"), _eye(environment, "hook_right")]
    for i, (p, q) in enumerate(_draw_points(path)):
        d = q - p
        n = np.linalg.norm(d)
        x = d / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])
        up = np.array([0.0, 0.0, 1.0]) if abs(x[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        set_pose(environment, f"cord_{i:02d}", (p + q) / 2, _quat_from_matrix(hook_rotation(x, up)))
    ph = _S["phase"]
    if ph == "work":
        ready = (ev["both"] and ev["in_band"] and ev["on_plate"] and ev["hands_off"] and ev["still"]
                 and ev["tension"] > 0)
        _S["ready_steps"] = _S["ready_steps"] + 1 if ready else 0
        if ready:
            _S["score"] = 0.5
        elif ev["on_plate"] and any(ev["seats"].values()) and ev["round_front"] and ev["tension"] > 0:
            _S["score"] = 0.25
        else:
            _S["score"] = 0.0
        if _S["ready_steps"] >= 20:                  # 1 s at 20 Hz: the scorer starts its tip-back test
            _S.update(phase="test", test_t=0.0, box_ref=ev["bottom"].copy(), test_ok=True, max_shift=0.0)
    elif ph == "test":
        shift = float(np.linalg.norm(ev["bottom"] - _S["box_ref"]))
        _S["max_shift"] = max(_S["max_shift"], shift)
        if not ev["on_plate"] or shift > SHIFT_LIMIT:
            _S["test_ok"], _S["test_note"] = False, f"carton moved {100 * shift:.1f} cm on the nose plate"
        if not ev["hands_off"]:
            _S["test_ok"], _S["test_note"] = False, "a gripper came near during the test"
        if _S["test_t"] >= TEST_S:
            ok = _S["test_ok"] and ev["both"] and ev["on_plate"]
            _S.update(phase="done", passed=ok, score=1.0 if ok else 0.5)
            print(f"TEST {'passed' if ok else 'failed'}: max carton shift {100 * _S['max_shift']:.2f} cm "
                  f"{_S['test_note']}", flush=True)


def _draw_points(path):
    """N_SEG pieces along the cord: evenly along each straight span (keeping its corners) when taut; when slack, a
    straight run on the table or a shallow sag (drawn only; the cord has no weight in the model)."""
    pts = [np.asarray(p, float) for p in path]
    spans = np.array([np.linalg.norm(q - p) for p, q in zip(pts, pts[1:])])
    total = max(spans.sum(), 1e-9)
    cum = np.concatenate([[0.0], np.cumsum(spans)])
    slack = max(CORD_REST - total, 0.0)
    out = []
    for s in np.linspace(0.0, total, N_SEG + 1):
        i = min(int(np.searchsorted(cum, s, side="right")) - 1, len(pts) - 2)
        u = (s - cum[i]) / max(spans[i], 1e-9)
        p = pts[i] + u * (pts[i + 1] - pts[i])
        if slack > 0:
            w = s / total
            p = p - np.array([0.0, 0.0, math.sqrt(3 * total * slack / 8) * 4 * w * (1 - w)])
            p[2] = max(p[2], TOP + 0.004)
        out.append(p)
    return list(zip(out, out[1:]))


def success(objects) -> bool:
    """The cord holds both hooks on opposite side tubes round the front of the carton, hands off, and the carton
    stayed on the nose plate through the scorer's tip-back test."""
    return bool(_S["passed"])


def score(objects) -> float:
    """1.0 passed the tip-back test; 0.5 strapped (both hooks home on opposite tubes, cord round the carton's front
    6-18 cm up, hands off, still for 1 s) but the tip-back test failed; 0.25 one hook home and the cord stretched
    round the carton's front; else 0."""
    return float(_S["score"])


def demo() -> dict:
    """Not a demo: an empty plan, so ``--policy yam_scripted`` only holds both arms still (a do-nothing run).
    The full-score demo is demo.py."""
    return {}
