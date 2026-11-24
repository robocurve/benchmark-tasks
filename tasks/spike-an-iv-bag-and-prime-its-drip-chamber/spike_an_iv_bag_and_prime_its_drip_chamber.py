"""On the station table lies a smooth steel tray, and on it a 500 ml bag of saline, its two ports toward the right arm:
the spike port, closed by a blue pull-off cap, and a medication port. Beside the bag lies an infusion giving set: a
pointed spike on a clear drip chamber, with a short length of tubing and its roller clamp.

Task: get the bag ready for an infusion. The bag slides on the smooth tray at a touch, so one hand holds it still by
the stiff hanger tab folded up on its far end, while the other pulls the blue cap off the spike port, takes the giving set by
its drip chamber, pushes the spike into the port all the way (against the port's grip) until it locks, and then
squeezes the drip chamber twice: each squeeze, let go, draws a third of a chamber of saline in. Then both hands let
go and both arms go back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=spike_an_iv_bag_and_prime_its_drip_chamber.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     rotation, set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the tray: smooth steel ------------------------------------------------------------------------------------------
TRAY_X, TRAY_Y, TRAY_T = (0.16, 0.50), (-0.30, 0.24), 0.004
TT = TOP + TRAY_T

# -- the bag: lying flat, its ports toward -y ---------------------------------------------------------------------------
BAG_XY = (0.32, 0.06)
BAG_W, BAG_L, BAG_H = 0.10, 0.15, 0.022  # across (x), long (y), thick
BAG_MASS = 0.50
PORT_Z = 0.011  # the ports' axis, above the tray
FIN, FIN_Y = (0.028, 0.008, 0.020), BAG_L / 2 - 0.012  # the stiff hanger tab, folded up on the bag's far end
PORT_R, PORT_Y = 0.005, (-0.075, -0.105)  # the spike port: radius, from the bag's end to its mouth (bag frame)
MOUTH = np.array([0.0, PORT_Y[1], PORT_Z])  # the spike port's mouth, in the bag's frame
CAP_L, CAP_R = 0.015, 0.0065
CAP_AT = np.array([BAG_XY[0], BAG_XY[1] + PORT_Y[1] - CAP_L / 2, TT + PORT_Z])
CAP_BREAK = 3.5  # N: the cap's seal tears at this pull

# -- the giving set: lying on the tray, its spike toward +y ---------------------------------------------------------
SET_AT = np.array([0.42, -0.13, TT + 0.010])  # the drip chamber's middle
CH_R, CH_L = 0.010, 0.045  # the drip chamber
SPIKE_TIP = 0.0655  # the spike's tip, ahead of the chamber's middle (set frame, +y)
SEAT = 0.025  # the spike in this deep: its collar on the port's mouth, locked
K_SPIKE, SLIP_S, D_SPIKE = 1500.0, 0.001, 3.0  # the port's grip on the spike: about 1.5 N to push it further
CAPTURE_R, CAPTURE_DEG = 0.007, 12.0

# -- priming: each squeeze of the drip chamber, let go, draws in a third of a chamber ---------------------------------
THIRDS = 3
FILL_OK = (1, 2)  # a third to two thirds full
SQUEEZED, OPENED = FINGER_OPEN * 0.30, FINGER_OPEN * 0.40  # finger joint: closed on the chamber / open again
SQUEEZE_AT = 0.04  # the TCP this close to the chamber's middle

# -- scoring ------------------------------------------------------------------------------------------------------------
CAP_OFF = 0.005
LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

STEEL, BAGC, SEAM, WHITE, BLUE, NAVY, CLEAR, FLUID, DARK, TUBE = (0.62, 0.64, 0.66), (0.86, 0.90, 0.94), \
    (0.74, 0.79, 0.85), (0.96, 0.96, 0.96), (0.10, 0.35, 0.85), (0.10, 0.15, 0.40), (0.85, 0.88, 0.92), \
    (0.35, 0.65, 0.95), (0.12, 0.12, 0.13), (0.78, 0.86, 0.92)
HIDE = (0.32, 0.40, 0.30)  # under the table


def _tray() -> Body:
    return Body.build("tray", pos=((TRAY_X[0] + TRAY_X[1]) / 2, (TRAY_Y[0] + TRAY_Y[1]) / 2, TOP), mass=2.0,
                      shapes=[BoxShape((0.0, 0.0, TRAY_T / 2), (TRAY_X[1] - TRAY_X[0], TRAY_Y[1] - TRAY_Y[0], TRAY_T),
                                       STEEL)],
                      kinematic=True, friction=(0.05, 0.04), slick=True, color=STEEL)


def _bag() -> Body:
    """The bag; origin on the tray under its middle. Its ports point toward -y."""
    shapes = [BoxShape((0.0, 0.0, BAG_H / 2), (BAG_W - 0.012, BAG_L - 0.015, BAG_H), BAGC),  # the filled bag
              BoxShape((0.0, 0.0, BAG_H / 2), (BAG_W, BAG_L, 0.003), SEAM),  # its welded rim
              BoxShape((0.0, FIN_Y, BAG_H + FIN[2] / 2), FIN, SEAM),  # the stiff hanger tab, folded up
              CylShape((0.0, FIN_Y, BAG_H + FIN[2] * 0.6), 0.004, FIN[1] + 0.0006, DARK, axis="Y", collide=False),
              BoxShape((0.0, 0.012, BAG_H + 0.0002), (0.062, 0.08, 0.0004), WHITE, collide=False),  # the label
              BoxShape((0.0, 0.036, BAG_H + 0.0004), (0.050, 0.010, 0.0004), BLUE, collide=False),
              BoxShape((0.0, 0.016, BAG_H + 0.0004), (0.040, 0.004, 0.0004), NAVY, collide=False),
              BoxShape((0.0, 0.004, BAG_H + 0.0004), (0.046, 0.003, 0.0004), NAVY, collide=False),
              BoxShape((0.0, -0.008, BAG_H + 0.0004), (0.030, 0.003, 0.0004), NAVY, collide=False),
              BoxShape((0.0, PORT_Y[0] + 0.004, PORT_Z), (0.05, 0.010, 0.006), SEAM),  # the port block
              CylShape((0.0, (PORT_Y[0] + PORT_Y[1]) / 2, PORT_Z), PORT_R, PORT_Y[0] - PORT_Y[1], WHITE, axis="Y"),
              CylShape((0.0, PORT_Y[1] + 0.0003, PORT_Z), 0.0033, 0.0008, DARK, axis="Y", collide=False),  # its bore
              CylShape((0.024, PORT_Y[0] - 0.009, PORT_Z), 0.0045, 0.018, WHITE, axis="Y"),  # the medication port
              CylShape((0.024, PORT_Y[0] - 0.020, PORT_Z), 0.0055, 0.006, (0.85, 0.85, 0.80), axis="Y")]
    return Body.build("bag", pos=(BAG_XY[0], BAG_XY[1], TT), mass=BAG_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=BAGC, com=(0.0, 0.0, BAG_H / 2))


def _cap() -> Body:
    """The pull-off cap on the spike port, with a tab to pull it by; origin on its axis, in its middle."""
    return Body.build("cap", pos=tuple(CAP_AT.tolist()), mass=0.006, friction=(1.0, 0.9), color=BLUE, shapes=[
        CylShape((0.0, 0.0, 0.0), CAP_R, CAP_L, BLUE, axis="Y"),
        BoxShape((0.0, -0.002, 0.014), (0.010, 0.010, 0.020), BLUE)])


def _giving_set() -> Body:
    """The giving set: spike, drip chamber, a short length of tubing with its roller clamp. Origin in the drip
    chamber's middle; the spike points +y. The chamber is clear (its wall is not drawn): the saline shows in it."""
    shapes = [CylShape((0.0, 0.0, 0.0), CH_R, CH_L, None, axis="Y"),  # the chamber (clear)
              CylShape((0.0, CH_L / 2 - 0.0015, 0.0), CH_R + 0.0003, 0.003, CLEAR, axis="Y", collide=False),
              CylShape((0.0, -CH_L / 2 + 0.0015, 0.0), CH_R + 0.0003, 0.003, CLEAR, axis="Y", collide=False),
              CylShape((0.0, 0.030, 0.0), 0.007, 0.015, WHITE, axis="Y"),  # the spike's base
              CylShape((0.0, 0.039, 0.0), 0.0085, 0.003, WHITE, axis="Y"),  # its collar
              CylShape((0.0, 0.0505, 0.0), 0.0032, 0.020, WHITE, axis="Y"),  # the spike
              CylShape((0.0, SPIKE_TIP - 0.0025, 0.0), 0.0018, 0.005, WHITE, axis="Y"),
              CylShape((0.0, -0.030, 0.0), 0.004, 0.015, WHITE, axis="Y"),  # the outlet
              CylShape((0.0, -0.085, 0.0), 0.0022, 0.095, TUBE, axis="Y"),  # tubing
              BoxShape((0.0, -0.095, 0.0), (0.014, 0.025, 0.012), BLUE)]  # the roller clamp
    for x, z in ((CH_R, 0.0), (-CH_R, 0.0), (0.0, CH_R), (0.0, -CH_R)):  # faint seams down the clear chamber
        shapes.append(BoxShape((x * 0.98, 0.0, z * 0.98), (0.0008 if x else 0.0012, CH_L - 0.004, 0.0012 if x else 0.0008),
                               CLEAR, collide=False))
    for k in range(1, THIRDS):  # marks at a third and two thirds
        shapes.append(CylShape((0.0, -CH_L / 2 + 0.003 + k * (CH_L - 0.006) / THIRDS, 0.0), CH_R + 0.0003, 0.0006,
                               DARK, axis="Y", collide=False))
    return Body.build("giving_set", pos=tuple(SET_AT.tolist()), mass=0.03, shapes=shapes, friction=(1.0, 0.9),
                      color=WHITE, com=(0.0, -0.005, 0.0), ignore=("bag",))


def _fluid(k: int) -> Body:
    """A third of a drip chamber of saline: kept under the table until drawn in, then shown inside the chamber."""
    return Body.build(f"saline_{k}", pos=(HIDE[0], HIDE[1] + 0.03 * k, HIDE[2]), mass=0.001, kinematic=True,
                      color=FLUID, shapes=[CylShape((0.0, 0.0, 0.0), CH_R - 0.0006, (CH_L - 0.006) / THIRDS, FLUID,
                                                    axis="Y", collide=False)])


OBJECTS = [_tray(), _bag(), _cap(), _giving_set(), *[_fluid(k) for k in range(1, THIRDS + 1)]]
hinge("cap_seal", "bag", "cap", anchor=tuple(CAP_AT.tolist()), axis="Y", limits_deg=(-0.5, 0.5), break_force=CAP_BREAK)
thread("spike_in", "bag", "giving_set", anchor0=tuple(MOUTH.tolist()), axis0=(0.0, 1.0, 0.0),
       anchor1=(0.0, SPIKE_TIP, 0.0), axis1=(0.0, 1.0, 0.0), slide_limits=(-0.05, SEAT), turn_damping=0.002,
       slide_stiffness=K_SPIKE, slide_damping=D_SPIKE, slide_max_force=50.0, play=0.002, tilt_deg=4.0)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"on": False, "hold": 0.0, "locked": False, "cap_off": False, "armed": False, "squeezing": False,
            "fill": 0, "tcps": None, "s": {}}


_EP = _fresh()
_ARM_IDS: dict = {}
_DEBUG = {"on": bool(os.environ.get("IVBAG_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _hands(base, origin) -> tuple[list[np.ndarray], list[float]]:
    """Each gripper's TCP (left, right) and its finger joint (0 shut, FINGER_OPEN wide open)."""
    tcps, fingers = [], []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        if arm not in _ARM_IDS:
            names, joints = list(robot.body_names), list(robot.joint_names)
            _ARM_IDS[arm] = (next(i for i, n in enumerate(names) if n == "gripper" or n.endswith("/gripper")),
                             joints.index("joint7"))
        body_i, finger_j = _ARM_IDS[arm]
        g = as_numpy(robot.data.body_link_pose_w)[0][body_i]
        tcps.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
        fingers.append(float(as_numpy(robot.data.joint_pos)[0][finger_j]))
    return tcps, fingers


def _quat_xyzw(m: np.ndarray) -> list[float]:
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    if w > 1e-6:
        return [(m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w), w]
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[3] = (m[k, j] - m[j, k]) / s
    q[i] = s / 4
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    return q


def _place(base, origin, name, pos, rot) -> None:
    import torch

    asset = base.scene[name]
    p = np.asarray(pos, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), *_quat_xyzw(np.asarray(rot, float))]],
                                              dtype=torch.float32, device=asset.device))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    b = as_numpy(base.scene["bag"].data.root_link_pose_w)[0]
    g = as_numpy(base.scene["giving_set"].data.root_link_pose_w)[0]
    c = as_numpy(base.scene["cap"].data.root_link_pose_w)[0]
    b_pos, b_rot, g_pos, g_rot, c_pos = b[:3] - origin, rotation(b[3:7]), g[:3] - origin, rotation(g[3:7]), c[:3] - origin
    seat_c = b_pos + b_rot @ (CAP_AT - np.array([BAG_XY[0], BAG_XY[1], TT]))
    if not _EP["cap_off"] and float(np.linalg.norm(c_pos - seat_c)) > CAP_OFF:
        _EP["cap_off"] = True
    q = b_rot.T @ (g_pos + g_rot @ np.array([0.0, SPIKE_TIP, 0.0]) - b_pos) - MOUTH  # the spike's tip, from the mouth
    off, depth = math.hypot(q[0], q[2]), float(q[1])
    angle = math.degrees(math.acos(max(-1.0, min(1.0, float((b_rot[:, 1]) @ (g_rot[:, 1]))))))
    if not _EP["on"]:
        if _EP["cap_off"] and off <= CAPTURE_R and 0.0 <= depth <= 0.012 and angle <= CAPTURE_DEG:  # in the port
            set_slide_target("spike_in", depth)
            set_thread_on("spike_in", True)
            _EP["on"], _EP["hold"] = True, depth
    elif not _EP["locked"]:
        if depth < -0.003:  # pulled back out
            set_thread_on("spike_in", False)
            _EP["on"] = False
        elif depth >= SEAT - 0.002:  # all the way in: it locks there
            _EP["locked"] = True
            set_slide_target("spike_in", SEAT)
        elif abs(depth - _EP["hold"]) > SLIP_S:  # pushed (or pulled) past the port's grip: it slides, and stays
            _EP["hold"] = depth
            set_slide_target("spike_in", depth)
    tcps, fingers = _hands(base, origin)
    _EP["tcps"] = tcps
    near = float(np.linalg.norm(tcps[1] - g_pos)) <= SQUEEZE_AT
    if _EP["locked"]:  # priming: a squeeze, let go, draws a third of a chamber in (counted only once it is locked)
        if fingers[1] < OPENED:
            if _EP["squeezing"]:
                _EP["fill"] = min(THIRDS, _EP["fill"] + 1)
            _EP["armed"], _EP["squeezing"] = True, False
        elif _EP["armed"] and near and fingers[1] > SQUEEZED:
            _EP["squeezing"] = True
    for k in range(1, THIRDS + 1):  # the saline drawn in shows in the chamber, from its outlet end up
        name = f"saline_{k}"
        if k <= _EP["fill"]:
            seg = (CH_L - 0.006) / THIRDS
            _place(base, origin, name, g_pos + g_rot @ np.array([0.0, -CH_L / 2 + 0.003 + (k - 0.5) * seg, 0.0]), g_rot)
        else:
            _place(base, origin, name, (HIDE[0], HIDE[1] + 0.03 * k, HIDE[2]), np.eye(3))
    mouth_w = b_pos + b_rot @ MOUTH
    clear = all(min(float(np.linalg.norm(t - g_pos)), float(np.linalg.norm(t - mouth_w))) >= LET_GO for t in tcps)
    on_tray = (TRAY_X[0] < b_pos[0] < TRAY_X[1] and TRAY_Y[0] < b_pos[1] < TRAY_Y[1] and abs(b_pos[2] - TT) < 0.01)
    _EP["s"] = {"clear": clear, "on_tray": on_tray}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            print(f"[ivbag] t={_DEBUG['steps'] / 20:5.1f}s cap_off={_EP['cap_off']} depth={depth * 1000:6.1f}mm "
                  f"q={np.round(q * 1000, 1).tolist()} "
                  f"off={off * 1000:4.1f}mm angle={angle:4.1f} on={_EP['on']} locked={_EP['locked']} "
                  f"fill={_EP['fill']} fingerR={fingers[1]:.4f} near={near} clear={clear} "
                  f"bag={np.round(b_pos * 1000).astype(int).tolist()} set={np.round(g_pos * 1000).astype(int).tolist()} "
                  f"leftTCP-bag={np.round((tcps[0] - b_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-set={np.round((tcps[1] - g_pos) * 1000).astype(int).tolist()} "
                  f"rightTCP-cap={np.round((tcps[1] - c_pos) * 1000).astype(int).tolist()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"cap_off": _EP["cap_off"], "locked": _EP["locked"], "primed": FILL_OK[0] <= _EP["fill"] <= FILL_OK[1],
            "clear": bool(st.get("clear")), "on_tray": bool(st.get("on_tray")), "home": _arms_home()}


def _points(s) -> float:
    if s["locked"] and s["primed"] and s["clear"] and s["on_tray"] and s["home"]:
        return 1.0
    if s["locked"]:
        return 0.5
    return 0.25 if s["cap_off"] else 0.0


def score(objects) -> float:
    """1.0 with the spike locked all the way into the port, the drip chamber a third to two thirds full (one or two
    squeezes after the spike locked), the bag still on the tray and both hands let go (each TCP at least 4 cm from
    the drip chamber and the port) and both arms back where they started (each TCP within 5 cm of its start); 0.5
    with the spike locked in the port; 0.25 with the cap pulled off; else 0.0. The spike can only go in once the cap
    is off, and once locked it stays locked."""
    return _points(_status())


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
    """Full score with the bag and the giving set still."""
    return (score(objects) >= 1.0 and objects["bag"].resting() and objects["giving_set"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the bag's hanger tab, fingers closing across it, and holds the bag. The right comes
# down on the blue cap's tab, fingers closing across it, and pulls it straight off the port, then drops it on the tray
# out of the way. It takes the giving set by its drip chamber, lifts it, lines the spike up with the port and pushes it
# in to the collar, where it locks. It lets go and squeezes the drip chamber twice, then rises. The left lets go of
# the bag, and both arms go back to where they started.

PRESS_GRIP = grip_rotation(0.0)  # straight down, fingers closing along y (across the hanger tab)
X_GRIP = grip_rotation(math.pi / 2)  # straight down, fingers closing along x (right arm)
FIN_IN = 0.006  # the left TCP this far below the hanger tab's top
FIN_OPEN = -0.009  # the left's fingers opened only ~5 mm wider than the tab each side
TAB_IN = 0.006  # the right TCP this far below the cap tab's top
CHAMBER_UP = 0.006  # the right TCP this far above the drip chamber's axis
PUSH_PAST = 0.025  # aim past the seat (the hand lags)
DROP_AT = np.array([0.26, -0.13, TT + 0.05])
LEAD_S, LEAD_STEPS = 3.0, 15
SET_OFF = np.array([-0.009, 0.0, -0.003])  # where the spike's tip ends up off the port's mouth if aimed straight at it
# (measured: the bag turns a little about the held tab as the cap is pulled, and the set hangs low in the grip)


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
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


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    press = np.array([BAG_XY[0], BAG_XY[1] + FIN_Y, TT + BAG_H + FIN[2] - FIN_IN])
    out = _lead_in("left", press + [0.0, 0.0, 0.05], PRESS_GRIP, "over the bag's hanger tab", FIN_OPEN)
    out += [_w(1.0, press, FIN_OPEN, "down round the tab", PRESS_GRIP),
            _w(0.5, press, FINGER_CLOSED, "hold the bag by its tab", PRESS_GRIP)]
    held_at = sum(w.duration for w in out)
    up = press + [0.0, 0.0, 0.06]
    out += [_w(max(release_at - held_at, 0.3), press, FINGER_CLOSED, "hold the bag still", PRESS_GRIP),
            _w(0.5, press, FINGER_OPEN, "let go", PRESS_GRIP),
            _w(0.8, up, FINGER_OPEN, "rise", PRESS_GRIP)]
    out += _go_home("left", up, PRESS_GRIP, FINGER_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, held_at


def _right(start: float) -> tuple[list[Waypoint], float]:
    tab = CAP_AT + [0.0, -0.002, 0.004 + 0.020 - TAB_IN]
    out = _lead_in("right", tab + [0.0, 0.0, 0.04], X_GRIP, "over the blue cap's tab", FINGER_OPEN)
    out.append(_w(max(start - LEAD_S, 0.3), tab + [0.0, 0.0, 0.04], FINGER_OPEN, "wait for the bag to be held",
                  X_GRIP))
    out += [_w(0.8, tab, FINGER_OPEN, "down round the tab", X_GRIP),
            _w(0.5, tab, FINGER_CLOSED, "take it", X_GRIP),
            _w(1.5, tab + [0.0, -0.04, 0.0], FINGER_CLOSED, "pull the cap straight off the port", X_GRIP),
            _w(1.2, DROP_AT, FINGER_CLOSED, "carry it out of the way", X_GRIP),
            _w(0.4, DROP_AT, FINGER_OPEN, "drop it on the tray", X_GRIP)]
    grip = SET_AT + [0.0, 0.0, CHAMBER_UP]
    mouth = np.array([BAG_XY[0], BAG_XY[1], TT]) + MOUTH
    line = np.array([mouth[0], mouth[1] - SPIKE_TIP - 0.012, mouth[2] + CHAMBER_UP]) - SET_OFF
    seated = line + [0.0, 0.012 + SEAT + PUSH_PAST, 0.0]
    out += [_w(1.2, grip + [0.0, 0.0, 0.04], FINGER_OPEN, "over the drip chamber", X_GRIP),
            _w(0.8, grip, FINGER_OPEN, "down round it", X_GRIP),
            _w(0.5, grip, FINGER_CLOSED, "take the giving set", X_GRIP),
            _w(1.0, grip + [0.0, 0.0, 0.03], FINGER_CLOSED, "lift it", X_GRIP),
            _w(1.5, line + [0.0, 0.0, 0.02], FINGER_CLOSED, "line the spike up with the port", X_GRIP),
            _w(0.8, line, FINGER_CLOSED, "the spike at the port's mouth", X_GRIP),
            _w(2.5, seated, FINGER_CLOSED, "push the spike all the way in: it locks", X_GRIP),
            _w(0.4, seated, FINGER_CLOSED, "hold", X_GRIP),
            _w(0.5, seated, FINGER_OPEN, "let go", X_GRIP)]
    chamber = np.array([line[0], mouth[1] + SEAT - SPIKE_TIP, line[2]])  # the drip chamber, the spike locked in
    out.append(_w(0.6, chamber, FINGER_OPEN, "round the drip chamber", X_GRIP))
    for k in (1, 2):
        out += [_w(0.6, chamber, FINGER_CLOSED, f"squeeze the drip chamber ({k})", X_GRIP),
                _w(0.3, chamber, FINGER_CLOSED, "hold the squeeze", X_GRIP),
                _w(0.7, chamber, FINGER_OPEN, "let go: saline is drawn in", X_GRIP)]
    up = chamber + [0.0, 0.0, 0.05]
    out.append(_w(0.8, up, FINGER_OPEN, "rise", X_GRIP))
    done = sum(w.duration for w in out)
    out += _go_home("right", up, X_GRIP, FINGER_OPEN)
    return out, done


def demo():
    _, held_at = _left(0.0, 0.0)
    right, done = _right(held_at)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(done + 0.3, total)
    return {"left": left, "right": right}
