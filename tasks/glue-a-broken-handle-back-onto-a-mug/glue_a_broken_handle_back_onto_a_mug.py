"""On a green cutting mat lies a white mug on its side, its mouth to the left, its broken side toward the arms: two
small bare patches of unglazed clay show where its handle snapped off. The handle lies on the mat in front of it, and
beside it a small tube of superglue, its cap off.

Task: glue the handle back on. Hold the mug still with one hand. With the other, take the glue tube and touch its
nozzle to both bare patches, then put the tube back; take the handle and press its two broken ends onto the patches,
and hold it there, without moving, until the glue grabs (3 seconds). Then let go of both and take both arms back to
where they started.

Why two hands: a mug lying on its side rolls as soon as it is pushed from the side, and the glue and the handle are
both pushed onto its side. Pushed on its own, it rolls away from the nozzle and the handle; and the glue grabs only
while the handle is held still against a still mug.

Run it:  inspect-robots run --embodiment yam_arena -E task=glue_a_broken_handle_back_onto_a_mug.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, glue, on_reset,  # noqa: E402
                     on_step, plate, rotation, set_glue_on)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the mat --------------------------------------------------------------------------------------------------------
MX0, MX1, MY0, MY1, MAT_T = 0.180, 0.470, -0.200, 0.170, 0.003
MAT = TOP + MAT_T
# -- the mug, lying on its side: its axis along y, mouth to the left (+y), its broken side toward the arms (-x) -------
MUG_R, MUG_L, WALL = 0.041, 0.096, 0.004
MUGX, MUGY = 0.385, 0.050
MUGZ = MAT + MUG_R  # its axis
NUB_Y = (-0.026, 0.022)  # the two bare patches (lower, by the base; upper, by the mouth), from the mug's middle
NUB_MID = sum(NUB_Y) / 2
# -- the handle: a C, lying on the mat as it will sit on the mug (opening toward +x); its frame's origin is midway
#    between its two broken ends ---------------------------------------------------------------------------------------
ARM_L, ARM_W, ARM_H = 0.032, 0.008, 0.010  # each arm: length (x), width (y), height (z)
BAR_X = -0.036  # the grip bar's middle, from the broken ends
H_START = np.array([0.270, -0.100, MAT + ARM_H / 2])
MOUNT = np.array([MUGX - MUG_R, MUGY + NUB_MID, MUGZ])  # where the handle's origin sits when it is on the mug
# -- the glue tube, lying along x, nozzle forward ----------------------------------------------------------------------
TUBE_R = 0.0075
NOZZLE = 0.043  # the nozzle's tip, from the tube's middle
G_START = np.array([0.250, -0.030, MAT + TUBE_R])
# -- the rules -------------------------------------------------------------------------------------------------------
DAB = 0.006  # the nozzle's tip within this of a patch's middle: glue on it
HOLD_POS, HOLD_DEG = 0.006, 10.0  # the handle this close to where it belongs ...
STILL_V, STILL_W = 0.008, 0.15  # ... the mug moving less than this (m/s, rad/s) ...
BOND_S = 3.0  # ... for this long, with the handle held: the glue grabs

MAT_C, MAT_LINE = (0.12, 0.33, 0.24), (0.42, 0.62, 0.50)
GLAZE, GLAZE_IN, BISQUE, NAVY = (0.95, 0.95, 0.93), (0.90, 0.91, 0.90), (0.80, 0.70, 0.58), (0.12, 0.20, 0.42)
RED, GLUE_C = (0.82, 0.16, 0.16), (0.96, 0.95, 0.80)
ALU, ALU_D, LABEL, LABEL2 = (0.80, 0.81, 0.83), (0.55, 0.56, 0.58), (0.92, 0.75, 0.10), (0.15, 0.15, 0.15)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _ring(n: int, r_out: float, t: float, length: float, color, centre=(0.0, 0.0, 0.0), collide: bool = True) -> list:
    """A tube along y of ``n`` flat plates ``t`` thick, its outer faces ``r_out`` from the axis (a plate faces -x)."""
    rm = r_out - t / 2
    w = 2 * r_out * math.tan(math.pi / n) + 0.0004
    c = np.asarray(centre, float)
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n + math.pi / 2
        n_ = np.array([math.cos(a), 0.0, math.sin(a)])
        m = c + rm * n_
        d = np.array([0.0, length / 2, 0.0])
        out.append(plate(m - d, m + d, w, t, n_, color, collide=collide))
    return out


def _mat() -> Body:
    shapes = [BoxShape((0.0, 0.0, MAT_T / 2), (MX1 - MX0, MY1 - MY0, MAT_T), MAT_C)]
    for k in range(1, 6):  # a cutting mat's printed grid
        x = MX0 + k * (MX1 - MX0) / 6 - (MX0 + MX1) / 2
        shapes.append(BoxShape((x, 0.0, MAT_T + 0.0002), (0.0012, MY1 - MY0 - 0.01, 0.0004), MAT_LINE, collide=False))
    for k in range(1, 8):
        y = MY0 + k * (MY1 - MY0) / 8 - (MY0 + MY1) / 2
        shapes.append(BoxShape((0.0, y, MAT_T + 0.0002), (MX1 - MX0 - 0.01, 0.0012, 0.0004), MAT_LINE, collide=False))
    return Body.build("mat", pos=((MX0 + MX1) / 2, (MY0 + MY1) / 2, TOP), mass=1.0, kinematic=True, color=MAT_C,
                      shapes=shapes, friction=(0.8, 0.7))


DOT_IDX: list = []  # the mug's shapes that are the glue dots (hidden until glue is put on)


def _mug() -> Body:
    shapes = _ring(48, MUG_R, WALL, MUG_L, GLAZE)  # near round: it rolls when pushed from the side
    shapes.append(CylShape((0.0, -MUG_L / 2 + 0.003, 0.0), MUG_R - WALL + 0.0005, 0.006, GLAZE_IN, axis="Y"))
    shapes.append(CylShape((0.0, -MUG_L / 2 + 0.0002, 0.0), MUG_R - 0.004, 0.0006, BISQUE, collide=False, axis="Y"))
    shapes += _ring(24, MUG_R + 0.0003, 0.0006, 0.012, NAVY, centre=(0.0, MUG_L / 2 - 0.016, 0.0), collide=False)
    shapes += _ring(24, MUG_R - WALL - 0.0002, 0.0006, MUG_L - 0.008, GLAZE_IN, centre=(0.0, 0.004, 0.0),
                    collide=False)
    # a red heart printed on the far side, facing up as the mug lies
    for dx, dy, r in ((-0.006, -0.004, 0.0075), (0.006, -0.004, 0.0075)):
        shapes.append(CylShape((dx + 0.012, dy, MUG_R + 0.0004), r, 0.0008, RED, collide=False))
    shapes.append(BoxShape((0.012, -0.010, MUG_R + 0.0004), (0.0135, 0.0135, 0.0008), RED, collide=False,
                           yaw=math.pi / 4))
    for y in NUB_Y:  # the bare patches where the handle broke off, and the glue that will go on them
        shapes.append(CylShape((-MUG_R - 0.0004, y, 0.0), 0.0062, 0.0008, BISQUE, collide=False, axis="X"))
    for y in NUB_Y:
        DOT_IDX.append(len(shapes))
        shapes.append(CylShape((-MUG_R - 0.0010, y, 0.0), 0.0050, 0.0008, GLUE_C, collide=False, axis="X"))
    return Body.build("mug", pos=(MUGX, MUGY, MUGZ + 0.0005), mass=0.30, shapes=shapes, color=GLAZE,
                      friction=(0.7, 0.6), damping=(0.0, 0.05))


def _carrier() -> Body:
    """A hidden part glued to the mug, that the handle is glued to once the glue grabs (a glue straight between the
    mug and the handle would stop them touching even while it is off)."""
    return Body.build("carrier", pos=(MUGX, MUGY, MUGZ + 0.0005), mass=0.01,
                      shapes=[BoxShape((0.0, 0.0, 0.0), (0.004, 0.004, 0.004), None, collide=False)])


def _tube() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.0), TUBE_R, 0.050, ALU, axis="X"),
              CylShape((-0.002, 0.0, 0.0), TUBE_R + 0.0003, 0.030, LABEL, collide=False, axis="X"),
              CylShape((-0.002, 0.0, 0.0), TUBE_R + 0.0005, 0.006, LABEL2, collide=False, axis="X"),
              CylShape((0.027, 0.0, 0.0), 0.0050, 0.004, ALU_D, axis="X"),
              CylShape((0.036, 0.0, 0.0), 0.0022, 0.014, (0.95, 0.95, 0.95), axis="X"),
              BoxShape((-0.027, 0.0, 0.0), (0.006, 0.022, 0.0016), ALU)]  # the crimped end: the tube can't roll
    return Body.build("tube", pos=tuple(G_START.tolist()), mass=0.020, shapes=shapes, color=ALU, friction=(1.2, 1.1))


def _handle() -> Body:
    shapes = []
    ys = [y - NUB_MID for y in NUB_Y]
    for y in ys:
        shapes.append(BoxShape((-ARM_L / 2 - 0.0005, y, 0.0), (ARM_L - 0.001, ARM_W, ARM_H), GLAZE))
        shapes.append(CylShape((BAR_X, y, 0.0), ARM_W / 2 + 0.0005, ARM_H, GLAZE))  # the rounded bends
        shapes.append(BoxShape((-0.0004, y, 0.0), (0.0008, ARM_W - 0.0006, ARM_H - 0.0006), BISQUE, collide=False))
    span = ys[1] - ys[0]
    shapes.append(BoxShape((BAR_X, 0.0, 0.0), (ARM_W, span, ARM_H), GLAZE))
    shapes.append(BoxShape((BAR_X - ARM_W / 2 - 0.0003, 0.0, 0.0), (0.0006, span - 0.006, 0.004), NAVY, collide=False))
    return Body.build("handle", pos=tuple(H_START.tolist()), mass=0.025, shapes=shapes, color=GLAZE,
                      friction=(1.2, 1.1))


glue("carry", "mug", "carrier", anchor=(MUGX, MUGY, MUGZ + 0.0005))
glue("bond", "carrier", "handle", anchor=tuple(H_START.tolist()))
MOUNT_OFF = (MOUNT - np.array([MUGX, MUGY, MUGZ + 0.0005])).tolist()

_DOT_PATHS: list = []
_author_glues_plain = bodykit._author_glues


def _author_glues(stage, prim_path: str, spec) -> None:
    """bodykit's glues, then: the bond holds the handle where it sits on the mug (not where it starts), and the glue
    dots start hidden."""
    _author_glues_plain(stage, prim_path, spec)
    if spec.name == "handle":
        from pxr import Gf, UsdPhysics

        joint = UsdPhysics.FixedJoint(stage.GetPrimAtPath(f"{prim_path}/bond"))
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(*MOUNT_OFF))
        joint.GetJointEnabledAttr().Set(False)  # off from the start, or it pulls the handle on as the sim settles
    if spec.name == "mug":
        from pxr import UsdGeom

        _DOT_PATHS.clear()
        for i in DOT_IDX:
            _DOT_PATHS.append(f"{prim_path}/body/s{i}")
            UsdGeom.Imageable(stage.GetPrimAtPath(_DOT_PATHS[-1])).MakeInvisible()


bodykit._author_glues = _author_glues

OBJECTS = [_mat(), _mug(), _carrier(), _tube(), _handle()]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"glued": [False, False], "held_s": 0.0, "bonded": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("MUG_DEBUG")), "steps": 0}


def _show_dot(i: int, on: bool) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    prim = get_current_stage().GetPrimAtPath(_DOT_PATHS[i])
    if prim:
        img = UsdGeom.Imageable(prim)
        img.MakeVisible() if on else img.MakeInvisible()


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_glue_on("bond", False)
    for i in range(len(_DOT_PATHS)):
        _show_dot(i, False)


def _tcps(base, origin):
    out, held = [], []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
        j7 = robot.joint_names.index("joint7")
        held.append(float(as_numpy(robot.data.joint_pos)[0][j7]) > FINGER_OPEN * 0.15)
    return out, held


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pm, rm = _pose(base, origin, "mug")
    ph, rh = _pose(base, origin, "handle")
    pg, rg = _pose(base, origin, "tube")
    vel = as_numpy(base.scene["mug"].data.root_com_vel_w)[0]
    tcps, closed = _tcps(base, origin)
    tip = pg + rg @ np.array([NOZZLE, 0.0, 0.0])
    for i, y in enumerate(NUB_Y):
        nub = pm + rm @ np.array([-MUG_R, y, 0.0])
        if not _EP["glued"][i] and float(np.linalg.norm(tip - nub)) <= DAB:
            _EP["glued"][i] = True
            _show_dot(i, True)
    # the handle against where it belongs on the mug
    want = pm + rm @ (np.array(MOUNT_OFF) + [0.0, 0.0, 0.0005])
    err = float(np.linalg.norm(ph - want))
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(rm.T @ rh) - 1) / 2))))
    held = any(c and float(np.linalg.norm(t - ph)) <= 0.06 for t, c in zip(tcps, closed))
    still = float(np.linalg.norm(vel[:3])) <= STILL_V and float(np.linalg.norm(vel[3:])) <= STILL_W
    if not _EP["bonded"]:
        if all(_EP["glued"]) and held and still and err <= HOLD_POS and ang <= HOLD_DEG:
            _EP["held_s"] += 0.05
            if _EP["held_s"] >= BOND_S:
                _EP["bonded"] = True
                set_glue_on("bond", True)
        else:
            _EP["held_s"] = 0.0
    on_mat = MX0 + 0.02 <= pm[0] <= MX1 - 0.02 and MY0 + 0.02 <= pm[1] <= MY1 - 0.02 and pm[2] <= MUGZ + 0.02
    _EP["tcps"] = tcps
    _EP["s"] = {"on_mat": on_mat}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            mm = lambda v: np.round((np.asarray(v) - [0, 0, TOP]) * 1000).astype(int).tolist()  # noqa: E731
            rel = rm.T @ rh
            rv = np.array([rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]]) / 2
            print(f"[mug] t={_DEBUG['steps'] / 20:5.1f}s rot={np.round(np.degrees(rv), 1).tolist()} mug={mm(pm)}"
                  f" v={np.linalg.norm(vel[:3]) * 1000:.0f}mm/s"
                  f" w={np.linalg.norm(vel[3:]):.2f} tip={mm(tip)} glued={_EP['glued']} handle={mm(ph)}"
                  f" err={err * 1000:.1f}mm ang={ang:.1f} held={held} still={still} t_held={_EP['held_s']:.2f}"
                  f" bonded={_EP['bonded']} L={mm(tcps[0])} R={mm(tcps[1])} closed={closed} score={score(None)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0: the handle glued back on, the mug still lying on the mat and both arms back where they started, at the
    end; 0.5: the handle glued back on; 0.25: glue on both bare patches; else 0.0. (Glue put on stays on, and the
    handle can only be glued on after both patches have glue.)"""
    if _EP["bonded"]:
        return 1.0 if _EP["s"].get("on_mat") and _arms_home() else 0.5
    return 0.25 if all(_EP["glued"]) else 0.0


_HOME: dict = {}


def _arms_home() -> bool:
    """Both arms back where they started (each TCP within HOME_WITHIN of its start)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def success(objects) -> bool:
    """Full score with the mug and its handle at rest: the trial ends there."""
    return score(objects) >= 1.0 and objects["mug"].resting() and objects["handle"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes down on top of the mug near its mouth, fingers closed, and holds it down, leaning out so the
# wrists stay apart. The right takes the glue tube by its middle, touches the nozzle to the lower patch and then the
# upper one, and lays the tube back. It takes the handle by its lower arm, lifts it level with the patches, presses
# its broken ends onto them and holds it still for 4 s while the glue grabs; then it lets go, the left lets go of the
# mug, and both arms go back where they started.

LEAN_L = math.radians(30.0)  # the left leans out (+y)
LEAN_R = math.radians(12.0)  # the right leans out (-y)
HOLD_FINGER = -0.030  # the left's fingers half open (about 5.4 cm), straddling the mug's top
PRESS = np.array([MUGX, MUGY + 0.036, MUGZ + 0.031])  # its fingertips meet the mug's flanks about 40 deg off the top
PRESS_IN = 0.008  # aimed this far further down: a V of two fingertips holds the mug from rolling
TIP = 0.013  # closed fingertips this far past the TCP
TUBE_DEPTH = 0.008  # the tube's axis this far past the TCP toward the fingertips (the tips clear of the mat)
ARM_DEPTH = 0.010  # the handle's arm likewise
TUBE_FINGER = -0.020  # pre-open wide: the V-jaw narrows fast behind the tips
ARM_FINGER = -0.014
OVER = 0.060
LEAD_S, LEAD_STEPS = 3.0, 15
GRIP_ARM = np.array([-0.017, NUB_Y[0] - NUB_MID, 0.0])  # where the right takes the handle (its lower arm), handle frame
TWIST = 0.0  # radians: turns the right's grip against the pads' twist
AIM_DZ = 0.0  # the handle aimed this much higher than where it belongs
PRESS_ON = 0.010  # the handle aimed this far into the mug: pressed on firmly
HANDLE_ROLL = math.radians(11.5)  # the V-jaw rolls the handle this much about its gripped arm: the grip turns it back


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rot(arm: str, tilt: float = 0.0) -> np.ndarray:
    """Top-down grips closing along y, leaned out (left wrist further +y, right further -y); ``tilt`` turns the right's
    grip back about x (the handle hangs rolled in the V-jaw)."""
    if arm == "left":  # fingers spread along x, across the mug
        return _rx(-LEAN_L) @ grip_rotation(-math.pi / 2)
    return _rx(LEAN_R - tilt) @ _rz(TWIST) @ grip_rotation(0.0)


def _tcp_for(arm: str, point, depth: float, tilt: float = 0.0) -> np.ndarray:
    return np.asarray(point, float) - depth * _rot(arm, tilt)[:, 2]


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.7, 0.0, 0.7), (1.3, 1.9), (1.0, 1.7), (-1.3, -0.8), (-0.4, 0.0, 0.4), (-0.4, 0.4)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def demo():
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}
    up = {a: -OVER * _rot(a)[:, 2] for a in ("left", "right")}

    tilt = {"left": 0.0, "right": 0.0}

    def go(arm, t, pos, finger, label, n=1, to_tilt=None):
        p0 = cur[arm][0]
        p1 = np.asarray(pos, float)
        a0 = tilt[arm]
        a1 = a0 if to_tilt is None else to_tilt
        for j in range(1, n + 1):
            f = j / n
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * f), finger, label,
                                      _rot(arm, a0 + (a1 - a0) * f)))
        cur[arm] = (p1, finger)
        tilt[arm] = a1

    def wait(arm, t, label="wait"):
        p, f = cur[arm]
        plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, _rot(arm, tilt[arm])))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    # the left holds the mug down
    touch = _tcp_for("left", PRESS, TIP)
    pressed = _tcp_for("left", PRESS - [0.0, 0.0, PRESS_IN], TIP)
    plan["left"] += _joint_path("left", np.array(READY, float), _good_ik("left", touch + up["left"], _rot("left")),
                                "over the mug", HOLD_FINGER)
    cur["left"] = (touch + up["left"], HOLD_FINGER)
    go("left", 1.2, touch, HOLD_FINGER, "fingertips down astride the mug's top")
    go("left", 0.6, pressed, HOLD_FINGER, "hold it down")
    # the right takes the glue tube
    tube_grip = _tcp_for("right", G_START + [-0.004, 0.0, 0.0], TUBE_DEPTH)
    plan["right"] += _joint_path("right", np.array(READY, float),
                                 _good_ik("right", tube_grip + up["right"], _rot("right")), "over the glue tube",
                                 TUBE_FINGER)
    cur["right"] = (tube_grip + up["right"], TUBE_FINGER)
    go("right", 1.2, tube_grip, TUBE_FINGER, "down round the tube")
    go("right", 0.6, tube_grip, FINGER_CLOSED, "take the tube")
    wait("left", dur("right") - dur("left"), "hold the mug")

    def tube_at(tip_point):
        """The TCP holding the tube with its nozzle's tip at ``tip_point``."""
        return _tcp_for("right", np.asarray(tip_point, float) - [NOZZLE + 0.004, 0.0, 0.0], TUBE_DEPTH)

    go("right", 1.0, tube_grip + [0.0, 0.0, 0.045], FINGER_CLOSED, "lift it")
    for i, y in enumerate(NUB_Y):
        nub = np.array([MUGX - MUG_R, MUGY + y, MUGZ])
        go("right", 1.6, tube_at(nub - [0.015, 0.0, 0.0]), FINGER_CLOSED,
           "nozzle to the " + ("lower" if i == 0 else "upper") + " bare patch", n=3)
        go("right", 0.8, tube_at(nub + [0.002, 0.0, 0.0]), FINGER_CLOSED, "touch the glue on")
        wait("right", 0.3)
        go("right", 0.6, tube_at(nub - [0.015, 0.0, 0.0]), FINGER_CLOSED, "back off")
    go("right", 1.6, tube_grip + [0.0, 0.0, 0.045], FINGER_CLOSED, "back over its place", n=3)
    go("right", 1.0, tube_grip + [0.0, 0.0, 0.002], FINGER_CLOSED, "lay the tube down")
    go("right", 0.5, tube_grip + [0.0, 0.0, 0.002], TUBE_FINGER, "let go")
    go("right", 0.8, tube_grip + up["right"], TUBE_FINGER, "rise")
    # the right takes the handle and presses it on
    arm_grip = _tcp_for("right", H_START + GRIP_ARM, ARM_DEPTH)
    go("right", 1.4, arm_grip + up["right"], ARM_FINGER, "over the handle's lower arm", n=2)
    go("right", 1.0, arm_grip, ARM_FINGER, "down round it")
    go("right", 0.6, arm_grip, FINGER_CLOSED, "take the handle")
    go("right", 1.0, arm_grip + [0.0, 0.0, MUGZ - H_START[2] + 0.004], FINGER_CLOSED, "lift it level with the patches")

    def handle_at(origin):
        return _tcp_for("right", np.asarray(origin, float) + GRIP_ARM + [0.0, 0.0, AIM_DZ], ARM_DEPTH, HANDLE_ROLL)

    go("right", 1.8, handle_at(MOUNT - [0.020, 0.0, 0.0]), FINGER_CLOSED, "in front of the bare patches", n=4,
       to_tilt=HANDLE_ROLL)
    go("right", 1.2, handle_at(MOUNT + [PRESS_ON, 0.0, 0.0]), FINGER_CLOSED, "press its broken ends onto them", n=2)
    wait("right", 5.0, "hold it still while the glue grabs")
    go("right", 0.6, handle_at(MOUNT + [PRESS_ON, 0.0, 0.0]), ARM_FINGER, "let go")
    go("right", 0.8, handle_at(MOUNT - [0.004, 0.0, 0.0]) + up["right"], ARM_FINGER, "rise clear")
    wait("left", dur("right") - dur("left"), "hold the mug")
    go("left", 0.8, touch + up["left"], HOLD_FINGER, "let go of the mug")
    wait("right", 0.8)
    for arm in ("left", "right"):
        p = cur[arm][0]
        q = _good_ik(arm, p, _rot(arm, tilt[arm]))
        plan[arm] += _joint_path(arm, q, np.array(READY, float), "back to where it started", cur[arm][1])
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, cur[arm][1],
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    return plan
