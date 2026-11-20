"""A solid-pine wall-mounted drop-leaf table, folded down, on a wall section standing across the far end of the table.

The table is the 50 x 30 cm size of a common solid-pine wall drop-leaf table: a fixed strip screwed to the wall, a
pine leaf hinged to the strip's front edge on back-flap hinges under it, and two triangular pine brackets, one near
each end, each on its own vertical hinge. Folded, the leaf hangs straight down in front of the two brackets, which
lie flat against the wall behind it. To put the table up, the leaf is raised to level (it cannot go higher: its back
edge meets the strip) and held there while each bracket is swung out 90 degrees, to its stop, under the leaf; then
the leaf is let down onto the brackets. The hanging or half-raised leaf is in the brackets' way, so a bracket can
only swing out while the leaf is held up at level; let go before a bracket is out and the leaf falls back.

Frame: x forward from the arm bases, y left, z up, metres.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (hooks yam_arena so multi-shape bodies and joints are simulated)
from bodykit import Body, BoxShape  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "best"
CONTROL_HZ = 20.0

# -- geometry (reference layout) ------------------------------------------------------------------------------------
X_W = 0.53  # the wall's face toward the robot (x)
Y0 = -0.16  # the wall section's (and the table's) middle (y): to the robot's right of centre
Z_U = TOP + 0.38  # the leaf's underside when level (the hinge line), 380 mm above the table top
STRIP_D = 0.070  # fixed strip, wall to its front edge
LEAF_D, LEAF_W, LEAF_T = 0.230, 0.500, 0.017  # leaf: depth (hinge to front edge), width, thickness (30 cm in all)
X_P = X_W - STRIP_D - 0.001  # the hinge line (x): the leaf's back edge, 1 mm off the strip
LEAF_MASS = 0.98  # kg: 50 x 23 x 1.7 cm of pine at 0.50 g/cc
CLEAT_T, CLEAT_W, CLEAT_H = 0.030, 0.020, 0.220  # the wall cleat each bracket is hinged to
BR_Y = {"bracket_r": -0.190, "bracket_l": 0.190}  # each bracket's hinge line from the middle (y), at its cleat
BR_X = X_W - CLEAT_T  # its x
BR_ARM, BR_H, BR_T, BR_BAR = 0.180, 0.200, 0.020, 0.030  # arm length, height, thickness, bar depth
BR_GAP = 0.003  # bracket top below the leaf's underside
BR_MASS = 0.18  # kg, pine
BR_STOP = 90.0  # degrees: the bracket's stop, square to the wall
WALL_W, WALL_H, PLY = 0.900, 0.650, 0.018
WALL_MASS = 150.0

PINE, PINE_DARK = (0.80, 0.62, 0.40), (0.66, 0.48, 0.30)
WALL_C, CLEAT_C = (0.90, 0.89, 0.86), (0.70, 0.52, 0.33)

WALL_POS = (X_W, Y0, Z_U)
LEAF_POS = (X_P, Y0, Z_U)
BR_POS = {n: (BR_X, Y0 + y, Z_U) for n, y in BR_Y.items()}
# each bracket is hinged on its outer side and folds inward, behind the leaf: +1 lies along +y and opens by +90 deg
BR_SIDE = {"bracket_r": 1.0, "bracket_l": -1.0}


def _box(lo, hi, color, collide: bool = True) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _wall_shapes() -> list:
    zt = TOP - Z_U  # the table top, wall-relative
    s = [
        _box((0.0, -WALL_W / 2, zt), (PLY, WALL_W / 2, zt + WALL_H), WALL_C),  # wall panel (painted plywood)
        _box((PLY, -WALL_W / 2, zt), (0.25, WALL_W / 2, zt + 0.038), CLEAT_C),  # its foot on the table
        _box((-STRIP_D, -LEAF_W / 2, 0.0), (0.0, LEAF_W / 2, LEAF_T), PINE_DARK),  # the table's fixed strip
    ]
    for name, y in BR_Y.items():  # the two cleats the brackets hang on, each on its bracket's hinge side
        y0, y1 = sorted((y, y - BR_SIDE[name] * CLEAT_W))
        s.append(_box((-CLEAT_T, y0, -CLEAT_H), (0.0, y1, 0.0), PINE_DARK))
    return s


def _leaf_shapes() -> list:
    return [_box((-LEAF_D, -LEAF_W / 2, 0.0), (0.0, LEAF_W / 2, LEAF_T), PINE)]


def _bracket_shapes(side: float) -> list:
    """One triangular bracket, folded (the arm along +y for side +1), origin on its hinge line at the leaf's level."""
    top = -BR_GAP
    y = lambda a, b: sorted((side * a, side * b))  # noqa: E731
    stile = _box((-BR_T, y(0.0, BR_BAR)[0], top - BR_H), (0.0, y(0.0, BR_BAR)[1], top), PINE)
    arm = _box((-BR_T, y(0.0, BR_ARM)[0], top - BR_BAR), (0.0, y(0.0, BR_ARM)[1], top), PINE)
    brace = bodykit.segment((-BR_T / 2, side * 0.015, top - BR_H + 0.012), (-BR_T / 2, side * 0.120, top - 0.040),
                            BR_T, PINE)
    return [stile, arm, brace]


def _ry(a: float) -> np.ndarray:
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def _rz(a: float) -> np.ndarray:
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _quat(r: np.ndarray) -> tuple:
    return bodykit._quat_from_matrix(r)


HANG = _ry(math.radians(-90.0))  # the leaf hanging straight down


def _build() -> list:
    wall = Body.build("wall", pos=WALL_POS, mass=WALL_MASS, shapes=_wall_shapes(), color=WALL_C,
                      friction=(1.0, 0.9), solver_iterations=32, damping=(0.5, 0.5))
    leaf = Body.build("leaf", pos=LEAF_POS, mass=LEAF_MASS, shapes=_leaf_shapes(), color=PINE, friction=(0.6, 0.5),
                      solver_iterations=32, damping=(0.0, 0.0), start_xyzw=_quat(HANG), ignore=("wall",))
    br = Body.build("bracket_r", pos=BR_POS["bracket_r"], mass=BR_MASS, shapes=_bracket_shapes(1.0), color=PINE,
                    friction=(0.6, 0.5), solver_iterations=32, ignore=("wall",))
    bl = Body.build("bracket_l", pos=BR_POS["bracket_l"], mass=BR_MASS, shapes=_bracket_shapes(-1.0), color=PINE,
                    friction=(0.6, 0.5), solver_iterations=32, ignore=("wall", "bracket_r"))
    # back-flap hinges under the leaf: from hanging (-90) to level (0), where its back edge meets the strip
    bodykit.hinge("leaf_hinge", "wall", "leaf", anchor=LEAF_POS, axis="Y", limits_deg=(-90.0, 0.0), damping=0.0003)
    bodykit.hinge("hinge_r", "wall", "bracket_r", anchor=BR_POS["bracket_r"], axis="Z", limits_deg=(0.0, BR_STOP),
                  damping=0.0005)
    bodykit.hinge("hinge_l", "wall", "bracket_l", anchor=BR_POS["bracket_l"], axis="Z", limits_deg=(-BR_STOP, 0.0),
                  damping=0.0005)
    return [wall, leaf, br, bl]


OBJECTS = _build()
NAMES = [o.name for o in OBJECTS]


# -- reading the table from poses -------------------------------------------------------------------------------------
def leaf_deg(st) -> float:
    """The leaf's angle from level (degrees; -90 hanging, 0 level)."""
    r = st["wall"].rotation().T @ st["leaf"].rotation()
    return math.degrees(math.atan2(r[0, 2], r[0, 0]))


def bracket_deg(st, name: str) -> float:
    """How far a bracket has swung out from flat against the wall (degrees; 90 at its stop)."""
    r = st["wall"].rotation().T @ st[name].rotation()
    return BR_SIDE[name] * math.degrees(math.atan2(r[1, 0], r[0, 0]))


_LOCAL_BOX = {
    "leaf": (np.array([-LEAF_D, -LEAF_W / 2, 0.0]), np.array([0.0, LEAF_W / 2, LEAF_T])),
    "bracket_r": (np.array([-BR_T, 0.0, -BR_GAP - BR_H]), np.array([0.0, BR_ARM, -BR_GAP])),
    "bracket_l": (np.array([-BR_T, -BR_ARM, -BR_GAP - BR_H]), np.array([0.0, 0.0, -BR_GAP])),
}


def finger_points(p, r, half: float) -> list[np.ndarray]:
    """Points on both fingers of a gripper (TCP at p, rotation r, pads half m from the centre line)."""
    pts = []
    for side in (-1.0, 1.0):
        for x, y, z in ((half + 0.002, 0.0, 0.013), (half + 0.002, 0.0077, 0.008), (half + 0.002, -0.0077, 0.008),
                        (half + 0.004, 0.0, -0.005), (half + 0.008, 0.0, -0.030), (half + 0.015, 0.0, -0.055)):
            pts.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * x, y, z]))
    return pts


def _hand_points(base) -> list:
    from bodykit import as_numpy
    from yam_arena import ik, station

    out = []
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        qj = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names)
        q6 = np.array([qj[names.index(f"joint{i}")] for i in range(1, 7)])
        half = abs(float(qj[names.index("joint7")]))
        p, r = ik.tcp_pose(q6, station.BASES[arm])
        out.extend(finger_points(p, r, half))
    return out


def clearance(st, points) -> float:
    """Smallest distance (m) from the finger points to the leaf or either bracket (their bounding boxes)."""
    best = float("inf")
    for name, (lo, hi) in _LOCAL_BOX.items():
        for p in points:
            loc = st[name].to_local(p)
            d = np.maximum(0.0, np.maximum(lo - loc, loc - hi))
            best = min(best, float(np.linalg.norm(d)))
    return best


# -- episode state, the score ------------------------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["LEAF_DEBUG"], "w") if os.environ.get("LEAF_DEBUG") else None
LEVEL_DEG = 3.0  # the leaf within this of level
FULL_DEG, PART_DEG = 85.0, 30.0  # a bracket at its stop / partly out
HANDS_CLEAR = 0.05  # every finger point at least 5 cm from the leaf and both brackets
STILL_STEPS = 40  # ... and everything still for 2 s
STILL_LEAF, STILL_BR = 0.5, 1.0  # degrees of movement allowed over those 2 s


def _blank() -> None:
    _EP.clear()
    _EP.update(step=0, hist=[], best=0.0, best_at=None, best_state=None, st=None, report={}, clear=float("inf"),
               min_leaf=0.0, max_leaf=-90.0)


_blank()


@bodykit.on_reset
def _reset() -> None:
    _blank()


def tier(leaf: float, br: float, bl: float) -> float:
    if abs(leaf) > LEVEL_DEG:
        return 0.0
    full = (br >= FULL_DEG) + (bl >= FULL_DEG)
    part = (br >= PART_DEG) + (bl >= PART_DEG)
    if full == 2:
        return 1.0
    if part == 2:
        return 0.6
    if part == 1:
        return 0.3
    return 0.0


def _track(st, clear: float) -> None:
    e = _EP
    leaf, br, bl = leaf_deg(st), bracket_deg(st, "bracket_r"), bracket_deg(st, "bracket_l")
    e["max_leaf"] = max(e["max_leaf"], leaf)
    e["hist"].append((leaf, br, bl, clear))
    if len(e["hist"]) > STILL_STEPS:
        e["hist"].pop(0)
    h = e["hist"]
    if len(h) < STILL_STEPS:
        return
    a = np.array(h)
    ok = (a[:, 3].min() >= HANDS_CLEAR and np.ptp(a[:, 0]) <= STILL_LEAF and np.ptp(a[:, 1]) <= STILL_BR
          and np.ptp(a[:, 2]) <= STILL_BR)
    if ok:
        s = tier(leaf, br, bl)
        if s > e["best"]:
            e["best"], e["best_at"] = s, e["step"]
            e["best_state"] = dict(leaf=round(leaf, 2), br=round(br, 1), bl=round(bl, 1), clear=round(clear, 3))


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    _EP["st"] = st
    try:
        clear = clearance(st, _hand_points(env.unwrapped))
    except Exception as err:  # noqa: BLE001
        clear = 0.0
        if _DBG is not None and not _EP.get("hand_err"):
            _DBG.write(f"hand points failed: {err!r}\n")
            _EP["hand_err"] = True
    _EP["clear"] = clear
    _track(st, clear)
    _EP["step"] += 1
    if _DBG is not None and _EP["step"] % 2 == 0:
        _debug(st)


def report(objects=None) -> dict:
    e = _EP
    st = e.get("st")
    now = {}
    if st is not None:
        now = dict(leaf=round(leaf_deg(st), 2), br=round(bracket_deg(st, "bracket_r"), 1),
                   bl=round(bracket_deg(st, "bracket_l"), 1), clear=round(e["clear"], 3))
    return dict(score=e["best"], best_at=e["best_at"], best_state=e["best_state"], now=now,
                max_leaf=round(e["max_leaf"], 2))


def score(objects) -> float:
    """The best state held for 2 s with every finger point at least 5 cm from the leaf and both brackets: 1.0 the leaf
    within 3 degrees of level and both brackets swung out at least 85 degrees (to the 90 degree stop); 0.6 level and
    both brackets out at least 30 degrees; 0.3 level and one bracket out at least 30 degrees; otherwise 0."""
    rep = report(objects)
    _EP["report"] = rep
    return float(rep["score"])


def distance(objects) -> float:
    st = _EP.get("st")
    if st is None:
        return 1.0
    return (abs(leaf_deg(st)) + max(0.0, BR_STOP - bracket_deg(st, "bracket_r"))
            + max(0.0, BR_STOP - bracket_deg(st, "bracket_l"))) / 300.0


def _debug(st) -> None:
    _DBG.write(f"{_EP['step']:4d} leaf={leaf_deg(st):7.2f} br={bracket_deg(st, 'bracket_r'):6.1f} "
               f"bl={bracket_deg(st, 'bracket_l'):6.1f} clear={_EP['clear']:.3f} "
               f"wall=({st['wall'].pos[0]:.4f},{st['wall'].pos[1]:.4f}) best={_EP['best']}\n")
    _DBG.flush()


# -- the seeded start --------------------------------------------------------------------------------------------------
SHIFT_X, SHIFT_Y, YAW_DEG, FOLD_DEG = 0.030, 0.025, 5.0, 2.0
FAIL = os.environ.get("LEAF_FAIL", "")  # wrong outcomes on purpose (README): fixed, early, one, half, holding


def layout(seed: int | None) -> dict:
    """The seeded start: the wall section's shift (m) and turn (degrees), and each bracket's fold (degrees out)."""
    if not seed:
        return dict(dx=0.0, dy=0.0, yaw=0.0, fold_r=0.0, fold_l=0.0)
    rng = np.random.default_rng(seed)
    return dict(dx=float(rng.uniform(-SHIFT_X, SHIFT_X)), dy=float(rng.uniform(-SHIFT_Y, SHIFT_Y)),
                yaw=float(rng.uniform(-YAW_DEG, YAW_DEG)), fold_r=float(rng.uniform(0.0, FOLD_DEG)),
                fold_l=float(rng.uniform(0.0, FOLD_DEG)))


def wall_frame(lay: dict) -> tuple[np.ndarray, np.ndarray]:
    return np.asarray(WALL_POS, float) + np.array([lay["dx"], lay["dy"], 0.0]), _rz(math.radians(lay["yaw"]))


def poses(lay: dict) -> dict:
    """World poses (position, quaternion xyzw) of every body for a layout."""
    pw, rw = wall_frame(lay)

    def at(p):
        return pw + rw @ (np.asarray(p, float) - np.asarray(WALL_POS, float))

    return {
        "wall": (pw, _quat(rw)),
        "leaf": (at(LEAF_POS), _quat(rw @ HANG)),
        "bracket_r": (at(BR_POS["bracket_r"]), _quat(rw @ _rz(math.radians(lay["fold_r"])))),
        "bracket_l": (at(BR_POS["bracket_l"]), _quat(rw @ _rz(math.radians(-lay["fold_l"])))),
    }


@bodykit.on_reset_env
def _lay_out(emb, seed):
    """Put the bodies where this trial's seed says, let them settle, and return the first observation."""
    import torch
    from yam_arena import inspect_embodiment as ie
    from yam_arena import scene as _scene

    base = emb._base
    lay = layout(seed)
    _blank()
    for name, (pos, quat) in poses(lay).items():
        obj = base.scene[name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(np.asarray(pos, float), dtype=pose.dtype, device=pose.device) + base.scene.env_origins
        pose[:, 3:7] = torch.tensor(quat, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
    for _ in range(max(1, round(getattr(ie, "SETTLE_S", 0.5) / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(getattr(ie, "RESET_RENDERS", 3)):
        base.sim.render()
    _blank()
    _EP["st"] = _scene.object_states(emb._env, NAMES)
    _EP["layout"] = lay
    if _DBG is not None:
        _DBG.write(f"seed {seed}: layout {lay}\n")
        _debug(_EP["st"])
    return emb._observation()


# -- the scripted demo (reads object poses from the simulator to plan; moves the parts only through the grippers) ----
LEAF_GRIP_D = float(os.environ.get("LEAF_GRIP_D", "0.130"))  # grip this far out from the hinge (m)
LEAF_GRIP_IN = 0.010  # TCP this far in from the leaf's end (fingertips 23 mm in)
LEAF_SQUEEZE = -(LEAF_T / 2 - 0.006)  # finger target: 6 mm past contact on the 17 mm leaf
BR_SQUEEZE = -(BR_T / 2 - 0.006)
BR_OPEN = -0.024  # fingers part-open round a bracket brace
BRACE_AT = float(os.environ.get("LEAF_BRACE_AT", "0.50"))  # grip the brace this far up it
BR_AZ = {"bracket_r": -45.0, "bracket_l": 0.0}  # each hand's approach to its folded bracket (deg about z from +x)
HOLD_DEG = float(os.environ.get("LEAF_HOLD_DEG", "1.5"))  # the holder aims this far past level (into the stop)
SWING_DEG = float(os.environ.get("LEAF_SWING_DEG", "92"))  # the swing aims this far (the stop is at 90)


def ready_pose(arm: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


PARK_SHIFT = {"left": (0.0, 0.0, 0.0), "right": (-0.05, -0.20, -0.06)}  # the right hand parks clear of the raised leaf


def park(arm: str, seconds: float, finger: float = FINGER_OPEN, label: str = "back to the ready pose") -> Waypoint:
    p, r = ready_pose(arm)
    if label in ("back to the ready pose", "wait clear of the leaf's path", "wait for the leaf to be level"):
        p = p + np.array(PARK_SHIFT[arm])
    return Waypoint(seconds, tuple(p.tolist()), finger, label, r)


def demo():
    """Both arms wait at the ready pose; the real plan is made on the first control step (leafdemo.py)."""
    return {arm: [park(arm, 0.5, label="look")] for arm in ("left", "right")}


def _frame(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) / np.linalg.norm(close)
    return np.column_stack([x, np.cross(z, x), z])


def _wp(sec, pos, finger, label, rot) -> Waypoint:
    return Waypoint(float(sec), tuple(np.asarray(pos, float).tolist()), float(finger), label, np.asarray(rot, float))


def plan_from_state(st, lay=None) -> dict[str, list[Waypoint]]:
    """Both hands' plans, from the start pose of the wall section (the table moves with it)."""
    if FAIL == "fixed":  # wrong on purpose: the reference layout of the setup text, not the poses
        pw, rw = np.asarray(WALL_POS, float), np.eye(3)
    else:
        pw, rw = np.asarray(st["wall"].pos, float), st["wall"].rotation()

    def W(p):  # reference-layout point -> world
        return pw + rw @ (np.asarray(p, float) - np.asarray(WALL_POS, float))

    pin = np.asarray(LEAF_POS, float)
    g_level = np.array([X_P - LEAF_GRIP_D, Y0 + LEAF_W / 2 - LEAF_GRIP_IN, Z_U + LEAF_T / 2])
    r_level = _frame((0.0, -1.0, 0.0), (0.0, 0.0, 1.0))  # pointing -y at the leaf's left end, closing across it

    def leaf_grip(deg):
        r = _ry(math.radians(deg))
        return W(pin + r @ (g_level - pin)), rw @ r @ r_level

    def br_grip(name, deg, back=0.0):
        """TCP on the bracket's diagonal brace, pointing at the wall, fingers closing across the brace; ``back`` m
        out from the brace along the approach. The bracket swung ``deg`` out."""
        side = BR_SIDE[name]
        a = np.asarray(BR_POS[name], float)
        b0 = np.array([0.015, -BR_GAP - BR_H + 0.012])
        b1 = np.array([0.120, -BR_GAP - 0.040])
        s_z = b0 + BRACE_AT * (b1 - b0)
        d = (b1 - b0) / np.linalg.norm(b1 - b0)
        brace = np.array([0.0, side * d[0], d[1]])
        appr = _rz(math.radians(BR_AZ[name])) @ np.array([1.0, 0.0, 0.0])
        r0 = _frame(appr, np.cross(appr, brace))
        grip = a + np.array([-BR_T, side * s_z[0], s_z[1]]) - back * appr
        r = _rz(math.radians(side * deg))
        return W(a + r @ (grip - a)), rw @ r @ r0

    plain = _frame((1.0, 0.0, 0.0), (0.0, 0.0, 1.0))  # pointing at the wall, fingers closing up and down

    def swing(name):
        out = []
        p0, r0 = br_grip(name, 0.0)
        out.append(_wp(1.0, br_grip(name, 0.0, 0.07)[0], BR_OPEN, "in front of the bracket's brace", r0))
        out.append(_wp(0.8, p0, BR_OPEN, "fingers round the brace", r0))
        out.append(_wp(0.6, p0, BR_SQUEEZE, "pinch the brace", r0))
        top = SWING_DEG if not (FAIL == "half" and name == "bracket_r") else 50.0
        for i in range(1, 7):
            p, r = br_grip(name, top * i / 6)
            out.append(_wp(0.6, p, BR_SQUEEZE, "swing the bracket out", r))
        p, r = br_grip(name, top)
        out.append(_wp(0.4, p, BR_SQUEEZE, "hold the bracket at its stop", r))
        out.append(_wp(0.5, p, BR_OPEN, "let go of the bracket", r))
        side_out = np.sign((rw.T @ r[:, 2])[1])  # the hand ends pointing across the bracket: back out sideways
        q = p - rw @ np.array([0.0, side_out * 0.09, 0.0])
        out.append(_wp(1.0, q, BR_OPEN, "back off", r))
        return out

    # left hand: take the leaf's left end near its bottom corner and raise it to level, into the stop
    p_h, r_h = leaf_grip(-90.0)
    left = [
        _wp(2.5, p_h + rw @ np.array([0.0, 0.06, 0.0]), -0.035, "beside the leaf's end", r_h),
        _wp(1.5, p_h, -0.035, "fingers round the leaf's end", r_h),
        _wp(0.8, p_h, LEAF_SQUEEZE, "pinch the leaf", r_h),
    ]
    for i in range(1, 9):
        deg = -90.0 + (90.0 + HOLD_DEG) * i / 8
        p, r = leaf_grip(deg)
        left.append(_wp(0.6, p, LEAF_SQUEEZE, "raise the leaf", r))
    t_level = sum(w.duration for w in left)
    # right hand: wait in front, below the leaf's path; once the leaf is level swing the right bracket out
    p_br, r_st = br_grip("bracket_r", 0.0)
    right = [park("right", 2.5, BR_OPEN, label="wait clear of the leaf's path")]
    right.append(park("right", max(0.5, t_level + 0.3 - 2.5), BR_OPEN, label="wait for the leaf to be level"))
    right.extend(swing("bracket_r"))
    right.append(park("right", 2.0))
    t_right = sum(w.duration for w in right[:-3])  # the right bracket let go of
    p_lv, r_lv = leaf_grip(HOLD_DEG)
    if FAIL == "early":  # wrong on purpose: let go of the leaf as soon as it is level
        left.append(_wp(0.5, p_lv, -0.03, "let go of the leaf", r_lv))
        left.append(_wp(0.8, p_lv + rw @ np.array([0.0, 0.05, -0.02]), -0.03, "back off", r_lv))
        left.append(park("left", 2.0))
        return {"left": left, "right": right}
    left.append(_wp(max(0.5, t_right - t_level), p_lv, LEAF_SQUEEZE, "hold the leaf at level", r_lv))
    if FAIL == "holding":  # wrong on purpose: keep holding the leaf to the end
        left.append(_wp(20.0, p_lv, LEAF_SQUEEZE, "hold the leaf at level", r_lv))
        return {"left": left, "right": right}
    left.append(_wp(0.6, p_lv, -0.03, "let go of the leaf", r_lv))
    left.append(_wp(0.8, p_lv + rw @ np.array([0.0, 0.05, -0.02]), -0.03, "back off", r_lv))
    if FAIL == "one":  # wrong on purpose: the left bracket is never swung out
        left.append(park("left", 2.0))
        return {"left": left, "right": right}
    # left hand: round the leaf's end, under it, and swing the left bracket out
    p_u, r_u = br_grip("bracket_l", 0.0, 0.15)
    left.append(_wp(1.2, W(np.array([X_W - 0.20, Y0 + 0.33, Z_U - 0.08])), BR_OPEN, "down past the leaf's end", r_u))
    left.append(_wp(1.2, p_u, BR_OPEN, "under the leaf", r_u))
    left.extend(swing("bracket_l"))
    left.append(park("left", 2.0))
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    print(f"leafdemo: wall ({pw[0]:.4f}, {pw[1]:.4f}), yaw {math.degrees(math.atan2(rw[1, 0], rw[0, 0])):.2f} deg, "
          f"leaf level at {t_level:.1f} s, plan {max(tl, tr):.1f} s", flush=True)
    return {"left": left, "right": right}


import leafdemo  # noqa: E402  (the demo's control hooks for yam_scripted)

leafdemo.install()
leafdemo_task = True
