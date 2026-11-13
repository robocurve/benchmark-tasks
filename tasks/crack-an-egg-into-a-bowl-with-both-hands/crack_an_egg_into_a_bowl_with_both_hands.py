"""On the kitchen table a brown hen's egg lies on its side on a small beech egg rest, a white mixing bowl stands in
the middle, and a steel scraps tray lies along the front.

Task: crack the egg into the bowl. Tap the egg's middle on the bowl's rim to crack it, hold it over the bowl with one
hand on each end and pull the two halves of the shell apart so the yolk and the white drop into the bowl; then put
both shell halves in the scraps tray (none in the bowl) and take both arms back to where they started. A cracked shell
still holds together (it takes about 4 N to pull the halves apart), and a hand holding the egg by one end has nothing
to pull the other end with: opening it takes one hand on each half.

Run it:  inspect-robots run --embodiment yam_arena -E task=crack_an_egg_into_a_bowl_with_both_hands.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset,  # noqa: E402
                     on_step, plate, rotation, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the egg: two half shells meeting at its middle, lying along y on the rest ------------------------------------------
EX, EY = 0.27, -0.17  # its middle, on the rest
RAIL_X, RAIL_H = 0.013, 0.010  # the rest's two rails (either side of the egg), how tall
PROFILE = ((0.0, 0.024), (0.010, 0.0236), (0.020, 0.0222), (0.028, 0.0185), (0.033, 0.0125))  # (distance from the
# middle, outer radius) of each half: nearly straight where the hands hold it, rounding off toward the end
CAP_T, WALL, N_SEG = 0.0035, 0.003, 20
EZ = TOP + RAIL_H + math.sqrt(PROFILE[0][1] ** 2 - RAIL_X ** 2)  # its axis, resting on the rails
SHELL_MASS = 0.080  # heavier than a real shell: a very light body lets the fingers' hulls sink into it
FIT_N = 4.0  # once cracked, the membrane still holds the halves together with up to this force ...
OPEN_AT = 0.008  # ... until they are pulled this far apart: then they come apart
CRACK_SETTLE = 10  # control steps after the tap before the crack lets go (the membrane holds through the knock)

# -- what is inside: a yolk and six drops of white ----------------------------------------------------------------------
YOLK_R, YOLK_MASS = 0.0105, 0.017
WHITE_R, WHITE_MASS = 0.0055, 0.003
WHITES = ((0.0145, 0.009), (-0.0145, -0.009), (0.006, 0.023), (-0.006, 0.023), (0.006, -0.023), (-0.006, -0.023))

# -- the bowl and the scraps tray -------------------------------------------------------------------------------------
BX, BY = 0.30, 0.0
BOWL_FLOOR_R, BOWL_RIM_R, BOWL_H, BOWL_T = 0.050, 0.078, 0.065, 0.005
RIM_MID = BOWL_RIM_R - BOWL_T / 2  # the rim's middle line, where the egg is tapped
TRAY_X, TRAY_LEN, TRAY_W, TRAY_H = 0.180, 0.34, 0.085, 0.016

MEMBRANE = (0.95, 0.93, 0.86)
SHELL_C, SPECK, YOLK_C, WHITE_C, BOWL_C, BAND_C, BEECH, STEEL = (
    (0.80, 0.60, 0.42), (0.62, 0.42, 0.28), (0.98, 0.68, 0.08), (0.93, 0.93, 0.84), (0.95, 0.95, 0.93),
    (0.25, 0.45, 0.75), (0.85, 0.70, 0.48), (0.72, 0.73, 0.75))

# -- scoring ----------------------------------------------------------------------------------------------------------
CRACK_BAND = 0.020  # the egg's middle within this of the rim's middle line ...
CRACK_GAP = 0.008  # ... its underside within this of the rim's top ...
CRACK_SPEED = 0.03  # ... coming down at least this fast (m/s) in the last 0.3 s: it cracks
WHITES_NEEDED = 5
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _half(name: str, s: float) -> Body:
    """A half shell, its origin on the egg's middle, reaching out along s*y (s = +1 the left half, -1 the right)."""
    shapes = []
    for (u0, r0), (u1, r1) in zip(PROFILE[:-1], PROFILE[1:]):
        m0, m1 = r0 - WALL / 2, r1 - WALL / 2
        for k in range(N_SEG):
            a = 2 * math.pi * (k + 0.5) / N_SEG
            c, d = math.cos(a), math.sin(a)
            shapes.append(plate((m0 * c, s * u0, m0 * d), (m1 * c, s * u1, m1 * d),
                                2 * math.pi * max(m0, m1) / N_SEG * 1.2, WALL, (c, 0.0, d), SHELL_C))
    u_end, r_end = PROFILE[-1]
    shapes.append(CylShape((0.0, s * (u_end + CAP_T / 2 - 0.001), 0.0), r_end + 0.0005, CAP_T, SHELL_C, axis="Y"))
    # the inner membrane: a floor sloping down toward the middle, so what is inside gathers at the crack and drops out
    # as soon as the halves part
    shapes.append(plate((0.0, s * 0.030, -0.006), (0.0, s * 0.003, -0.019), 0.036, 0.002, (0.0, 0.0, 1.0), MEMBRANE))
    for k in range(7):  # a few speckles (seen only)
        a = 2 * math.pi * k / 7 + 0.4 * s
        u = 0.006 + 0.0038 * k
        r = np.interp(u, [p[0] for p in PROFILE], [p[1] for p in PROFILE]) + 0.0003
        shapes.append(SphereShape((r * math.cos(a), s * u, r * math.sin(a)), 0.0011, SPECK, collide=False))
    return Body.build(name, pos=(EX, EY, EZ), mass=SHELL_MASS, shapes=shapes, color=SHELL_C, friction=(1.0, 0.9),
                      com=(0.0, s * 0.015, 0.0), damping=(0.0, 0.1))


def _yolk() -> Body:
    return Body.build("yolk", pos=(EX, EY, EZ - 0.006), mass=YOLK_MASS, color=YOLK_C, friction=(0.3, 0.2),
                      shapes=[SphereShape((0.0, 0.0, 0.0), YOLK_R, YOLK_C)], damping=(0.3, 0.3))


def _white(i: int) -> Body:
    x, y = WHITES[i]
    z = 0.001 if abs(y) > 0.015 else -0.004  # above the sloping membrane
    return Body.build(f"white_{i}", pos=(EX + x, EY + y, EZ + z), mass=WHITE_MASS, color=WHITE_C,
                      friction=(0.3, 0.2), shapes=[SphereShape((0.0, 0.0, 0.0), WHITE_R, WHITE_C)], damping=(0.3, 0.3))


def _rest() -> Body:
    shapes = [BoxShape((0.0, 0.0, 0.002), (0.040, 0.090, 0.004), BEECH)]
    shapes += [BoxShape((sx * RAIL_X, 0.0, RAIL_H / 2), (0.006, 0.070, RAIL_H), BEECH) for sx in (-1.0, 1.0)]
    end = PROFILE[-1][0] + CAP_T - 0.001 + 0.0015  # the egg's end, plus a little play
    shapes += [BoxShape((0.0, sy * (end + 0.003), (EZ - TOP) / 2 + 0.004), (0.022, 0.006, EZ - TOP + 0.008), BEECH)
               for sy in (-1.0, 1.0)]  # an end stop at each end
    return Body.build("egg_rest", pos=(EX, EY, TOP), mass=1.0, shapes=shapes, kinematic=True, color=BEECH)


def _bowl() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.003), BOWL_FLOOR_R + 0.002, 0.006, BOWL_C)]
    n = 28
    for k in range(n):
        a = 2 * math.pi * (k + 0.5) / n
        c, d = math.cos(a), math.sin(a)
        r0, r1 = BOWL_FLOOR_R, RIM_MID
        shapes.append(plate((r0 * c, r0 * d, 0.004), (r1 * c, r1 * d, BOWL_H), 2 * math.pi * r1 / n * 1.15, BOWL_T,
                            (c, d, -(r1 - r0) / BOWL_H), BOWL_C))
    for k in range(n):  # a blue band below the rim (seen only)
        a = 2 * math.pi * (k + 0.5) / n
        r = RIM_MID - 0.0035 + BOWL_T / 2 + 0.0008
        shapes.append(BoxShape((r * math.cos(a), r * math.sin(a), BOWL_H - 0.010), (0.001, 2 * math.pi * r / n * 1.15,
                                                                                   0.006), BAND_C, yaw=a,
                               collide=False))
    return Body.build("mixing_bowl", pos=(BX, BY, TOP), mass=1.0, shapes=shapes, kinematic=True, color=BOWL_C)


def _tray() -> Body:
    t = 0.002
    shapes = [BoxShape((0.0, 0.0, t / 2), (TRAY_W, TRAY_LEN, t), STEEL)]
    shapes += [BoxShape((sx * (TRAY_W / 2 - t / 2), 0.0, TRAY_H / 2), (t, TRAY_LEN, TRAY_H), STEEL) for sx in (-1, 1)]
    shapes += [BoxShape((0.0, sy * (TRAY_LEN / 2 - t / 2), TRAY_H / 2), (TRAY_W, t, TRAY_H), STEEL) for sy in (-1, 1)]
    return Body.build("scraps_tray", pos=(TRAY_X, 0.0, TOP), mass=1.0, shapes=shapes, kinematic=True, color=STEEL)


CONTENTS = ["yolk"] + [f"white_{i}" for i in range(len(WHITES))]
OBJECTS = [_rest(), _bowl(), _tray(), _half("shell_right", -1.0), _half("shell_left", 1.0), _yolk()] + \
    [_white(i) for i in range(len(WHITES))]
# Two joints hold the halves in line, letting them part only along the egg's length: the shell (stiff, strong) until it
# is cracked, then the membrane (FIT_N at most) until the halves are pulled OPEN_AT apart.
slider("egg_shell", "shell_right", "shell_left", anchor=(EX, EY, EZ), axis="Y", limits=(0.0, 0.06),
       stiffness=20000.0, damping=20.0, target=0.0, max_force=500.0)
slider("egg_membrane", "shell_right", "shell_left", anchor=(EX, EY, EZ), axis="Y", limits=(0.0, 0.06),
       stiffness=2000.0, damping=4.0, target=0.0, max_force=FIT_N)


def _set_joint(name: str, on: bool) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    joint = UsdPhysics.Joint(get_current_stage().GetPrimAtPath(bodykit._SLIDER_PATHS[name]))
    attr = joint.GetJointEnabledAttr() or joint.CreateJointEnabledAttr(True)
    attr.Set(bool(on))


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"cracked": False, "opened": False, "crack_t": None, "open_t": None, "prev_z": None, "vz": [], "tcps": None,
            "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("EGG_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    for name in ("egg_shell", "egg_membrane"):  # rebuilt whole, from the reset poses
        _set_joint(name, False)
        _set_joint(name, True)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _in_bowl(p) -> bool:
    return float(np.hypot(p[0] - BX, p[1] - BY)) < BOWL_RIM_R - 0.008 and TOP < p[2] < TOP + BOWL_H


def _in_tray(p) -> bool:
    return (abs(p[0] - TRAY_X) < TRAY_W / 2 - 0.002 and abs(p[1]) < TRAY_LEN / 2 - 0.002
            and TOP < p[2] < TOP + 0.045)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    r_pos, r_rot = _pose(base, origin, "shell_right")
    l_pos, l_rot = _pose(base, origin, "shell_left")
    mid = r_pos  # the egg's middle (the right half's origin), while it is whole
    if _EP["prev_z"] is not None:
        _EP["vz"] = (_EP["vz"] + [(mid[2] - _EP["prev_z"]) * 20.0])[-6:]
    _EP["prev_z"] = float(mid[2])
    if not _EP["cracked"]:
        band = abs(float(np.hypot(mid[0] - BX, mid[1] - BY)) - RIM_MID) < CRACK_BAND
        under = mid[2] - PROFILE[0][1] - (TOP + BOWL_H)
        if band and under < CRACK_GAP and _EP["vz"] and min(_EP["vz"]) < -CRACK_SPEED:
            _EP["cracked"] = True
            _EP["crack_t"] = _DEBUG["steps"] / 20.0
            _EP["crack_step"] = _DEBUG["steps"]
    if _EP["cracked"] and not _EP.get("unglued") and _DEBUG["steps"] - _EP["crack_step"] >= CRACK_SETTLE:
        _EP["unglued"] = True
        _set_joint("egg_shell", False)  # cracked: now only the membrane (FIT_N) holds the halves together
    apart = float((r_rot.T @ (l_pos - r_pos))[1])
    if _EP.get("unglued") and not _EP["opened"] and apart > OPEN_AT:
        _EP["opened"] = True
        _EP["open_t"] = _DEBUG["steps"] / 20.0
        _set_joint("egg_membrane", False)  # pulled apart: the halves are loose
    poses = {n: _pose(base, origin, n)[0] for n in CONTENTS}
    yolk_in = _in_bowl(poses["yolk"])
    whites_in = sum(_in_bowl(poses[n]) for n in CONTENTS[1:])
    shells_in_tray = sum(_in_tray(p) for p in (r_pos, l_pos))
    shell_in_bowl = any(_in_bowl(p) for p in (r_pos, l_pos))
    _EP["tcps"] = tcps = _tcps(base, origin)
    _EP["s"] = {"cracked": _EP["cracked"], "opened": _EP["opened"], "yolk_in": yolk_in, "whites_in": whites_in,
                "shells_in_tray": shells_in_tray, "shell_in_bowl": shell_in_bowl}
    _DEBUG["steps"] += 1
    if _DEBUG["on"] and _DEBUG["steps"] % 10 == 0:
        tl, tr = tcps
        fing = []
        for arm in ("left_robot", "right_robot"):
            rb = base.scene[arm]
            j = [i for i, n in enumerate(rb.joint_names) if n == "joint7"][0]
            fing.append(round(float(as_numpy(rb.data.joint_pos)[0][j]) * 1000, 1))
        print(f"[egg] fingers={fing} t={_DEBUG['steps'] / 20:5.1f}s mid={np.round(mid * 1000).astype(int).tolist()} "
              f"vz={[round(v, 2) for v in _EP['vz']]} apart={apart * 1000:5.1f} cracked={_EP['cracked']}@{_EP['crack_t']} "
              f"opened={_EP['opened']}@{_EP['open_t']} yolk={np.round(poses['yolk'] * 1000).astype(int).tolist()} "
              f"yolk_in={yolk_in} whites_in={whites_in} tray={shells_in_tray} shell_in_bowl={shell_in_bowl} "
              f"home={_arms_home()} L={np.round(tl * 1000).astype(int).tolist()} "
              f"R={np.round(tr * 1000).astype(int).tolist()} score={_points(_status())}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"cracked": bool(st.get("cracked")), "opened": bool(st.get("opened")), "yolk_in": bool(st.get("yolk_in")),
            "whites_in": int(st.get("whites_in", 0)), "shells_in_tray": int(st.get("shells_in_tray", 0)),
            "shell_in_bowl": bool(st.get("shell_in_bowl")), "home": _arms_home()}


def _points(s) -> float:
    if not s["cracked"]:
        return 0.0
    if (s["opened"] and s["yolk_in"] and s["whites_in"] >= WHITES_NEEDED and s["shells_in_tray"] == 2
            and not s["shell_in_bowl"] and s["home"]):
        return 1.0
    if s["opened"] and s["yolk_in"]:
        return 0.5
    return 0.25


def score(objects) -> float:
    """Cracked: at some point the egg's middle came down onto the bowl's rim (within 2 cm of the rim's middle line,
    its underside within 8 mm of the rim's top, coming down at 3 cm/s or faster); opened: after that, the two shell
    halves were pulled 8 mm apart, which lets them go. Judged at the end: 1.0 when it was cracked and opened, the yolk
    and at least 5 of the 6 drops of white are in the bowl (within 7 cm of its middle, below its rim), both shell
    halves are in the scraps tray, neither is in the bowl, and both arms are back where they started (each TCP within
    5 cm of its start); 0.5 when it was cracked and opened and the yolk is in the bowl; 0.25 when it was cracked; else
    0.0."""
    return _points(_status())


_HOME: dict = {}


def _arms_home() -> bool:
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
    """Full score with the yolk, the white and both shell halves still."""
    return (score(objects) >= 1.0 and all(objects[n].resting() for n in CONTENTS)
            and objects["shell_right"].resting() and objects["shell_left"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand, leaning out to the right, and the left, leaning out to the left, take the egg on its rest, one on each
# half, lift it together and hold it over the bowl. Together they bring the egg's middle over the bowl's rim on its
# right side and tap it down on the rim: it cracks. Back over the middle of the bowl, the two pull the halves
# apart: the yolk and the white drop into the bowl. Each hand carries its half to the scraps tray, lets it go and goes
# back to where it started.

LEAN = 40.0
GRIP_U = 0.013  # each hand holds its half this far out from the middle
GRIP_DZ = 0.012  # the TCP this far above the egg's axis (the egg sits low in the jaw, not wedged in it)
HOLD_Z = TOP + BOWL_H + 0.050  # the egg's axis while it is held over the bowl
TAP_FROM, TAP_TO = TOP + BOWL_H + PROFILE[0][1] + 0.022, TOP + BOWL_H + PROFILE[0][1] - 0.025
SLIP = 0.0  # how far the hands' grips shove the held egg toward the right (aim the tap that much left)
PULL = 0.040
TIP = 0.0  # while pulling apart, each hand could tip its half's open end down this far (a grip slips about its
# closing axis, so it is not used)
LEAD_S, LEAD_STEPS = 3.0, 15


def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


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


def _joint_path(arm: str, q0, q1, label: str, finger, total: float = LEAD_S) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(total / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _t(arm) -> float:
    return sum(w.duration for w in arm)


def demo():
    open_w = -(PROFILE[0][1] + 0.006)
    r_rot = _rx(LEAN) @ grip_rotation(math.pi / 2)  # fingers closing along x, leaning out to the right
    l_rot = _rx(-LEAN) @ grip_rotation(-math.pi / 2)  # the same, leaning out to the left
    # both hands take the egg on its rest, one on each half (the rest's end stops keep it from being shoved along)
    pick_r = np.array([EX, EY - GRIP_U, EZ + GRIP_DZ])
    pick_l = np.array([EX, EY + GRIP_U, EZ + GRIP_DZ])
    right = _lead_in("right", pick_r + [0.0, 0.0, 0.06], r_rot, "over the egg's right half", open_w)
    right += [_w(1.2, pick_r, open_w, "down round it", r_rot),
              _w(0.4, pick_r, open_w, "settle", r_rot),
              _w(0.6, pick_r, FINGER_CLOSED, "take the egg by its right half", r_rot)]
    left = _lead_in("left", pick_l + [0.0, 0.0, 0.06], l_rot, "over the egg's left half", open_w)
    left += [_w(1.2, pick_l, open_w, "down round it", l_rot),
             _w(0.4, pick_l, open_w, "settle", l_rot),
             _w(0.6, pick_l, FINGER_CLOSED, "take the egg by its left half", l_rot)]
    t0 = max(_t(left), _t(right)) + 0.3
    for arm, pick, rot in ((left, pick_l, l_rot), (right, pick_r, r_rot)):
        arm.append(_w(t0 - _t(arm), pick, FINGER_CLOSED, "hold it", rot))
    hold_r = np.array([BX, BY - GRIP_U, HOLD_Z + GRIP_DZ])
    hold_l = np.array([BX, BY + GRIP_U, HOLD_Z + GRIP_DZ])
    for arm, pick, hold, rot in ((left, pick_l, hold_l, l_rot), (right, pick_r, hold_r, r_rot)):
        arm += [_w(1.2, pick + [0.0, 0.0, 0.07], FINGER_CLOSED, "together, lift it off the rest", rot),
                _w(2.0, hold, FINGER_CLOSED, "over the middle of the bowl", rot)]
    # together: over the bowl's rim on its right side (the egg's middle over the rim), tap down, back over the middle
    dy = -RIM_MID + SLIP
    for arm, hold, rot in ((left, hold_l, l_rot), (right, hold_r, r_rot)):
        arm += [_w(1.6, [hold[0], hold[1] + dy, TAP_FROM + GRIP_DZ], FINGER_CLOSED,
                   "together, the egg's middle just above the bowl's rim", rot),
                _w(0.4, [hold[0], hold[1] + dy, TAP_FROM + GRIP_DZ], FINGER_CLOSED, "steady", rot),
                _w(0.25, [hold[0], hold[1] + dy, TAP_TO + GRIP_DZ], FINGER_CLOSED, "tap it down on the rim: it cracks",
                   rot),
                _w(0.6, [hold[0], hold[1] + dy, TAP_FROM + GRIP_DZ + 0.02], FINGER_CLOSED, "up again", rot),
                _w(1.6, hold, FINGER_CLOSED, "back over the middle of the bowl", rot)]
    # pull the halves apart over the bowl
    l_tip = _rx(-LEAN + TIP) @ grip_rotation(-math.pi / 2)
    r_tip = _rx(LEAN - TIP) @ grip_rotation(math.pi / 2)
    left += [_w(1.0, hold_l + [0.0, PULL * 0.4, 0.0], FINGER_CLOSED, "pull the halves apart: yolk and white drop in",
                l_rot),
             _w(1.2, hold_l + [0.0, PULL, 0.0], FINGER_CLOSED, "pull them further apart", l_tip),
             _w(1.2, hold_l + [0.0, PULL, 0.0], FINGER_CLOSED, "let it all drop", l_tip)]
    right += [_w(1.0, hold_r - [0.0, PULL * 0.4, 0.0], FINGER_CLOSED, "pull the halves apart: yolk and white drop in",
                 r_rot),
              _w(1.2, hold_r - [0.0, PULL, 0.0], FINGER_CLOSED, "pull them further apart", r_tip),
              _w(1.2, hold_r - [0.0, PULL, 0.0], FINGER_CLOSED, "let it all drop", r_tip)]
    # each hand drops its half in the scraps tray
    for arm, name, rot, sy in ((left, "left", l_tip, 1.0), (right, "right", r_tip, -1.0)):
        drop = np.array([TRAY_X, sy * 0.08, TOP + 0.046])
        arm += [_w(1.6, drop + [0.0, 0.0, 0.03], FINGER_CLOSED, "over the scraps tray", rot),
                _w(0.8, drop, FINGER_CLOSED, "down into it", rot),
                _w(0.6, drop, FINGER_OPEN, "let the shell half go", rot),
                _w(0.8, drop + [0.0, sy * 0.035, 0.010], FINGER_OPEN, "draw back sideways, off the shell", rot),
                _w(1.0, drop + [0.0, sy * 0.035, 0.07], FINGER_OPEN, "rise", rot)]
        arm += _go_home(name, arm[-1].pos, rot, FINGER_OPEN)
    total = max(_t(left), _t(right)) + 3.0
    for arm in (left, right):
        arm.append(_w(max(total - _t(arm), 0.5), arm[-1].pos, FINGER_OPEN,
                      "hold still: the trial ends once the full score has held 1 s", arm[-1].yaw))
    return {"left": left, "right": right}
