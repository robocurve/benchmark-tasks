"""A motor rotor (a steel lamination stack, 20 cm across, standing on its assembly fixture) has four magnet slots
around its rim. Two magnets are set in resin; the third, beside the one empty slot, was only just pushed in and sits
loose. A wooden tray beside the rotor holds two more magnets, one with its red (north) face up and one with its blue
(south) face up. The poles must alternate around the rotor, so the empty slot takes the red-topped magnet.

Neighbouring rotor magnets attract each other. As the new magnet comes down over its slot, the loose magnet beside it
is pulled up out of its own slot and jumps onto the new one, unless a hand holds it down. So one hand presses the loose
magnet down while the other hand brings the red-topped magnet over the empty slot, lowers it in, lets go and presses
it in flush with the rotor's top. Then both hands let go and both arms go back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=fit_the_last_magnet_into_a_motor_rotor.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step,  # noqa: E402
                     rotation)
from yam_arena.constants import FINGER_CLOSED, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the rotor on its fixture -----------------------------------------------------------------------------------------
RC = np.array([0.30, 0.0])  # the rotor's axis
FIX_H = 0.008  # the fixture plate under it
Z0 = TOP + FIX_H  # the rotor's bottom (and the slots' floor)
H = 0.050  # the stack's height
R_OUT = 0.100
R_S = 0.085  # the slots' middle line
SLOT = (0.046, 0.020)  # a slot: along the rim, across (radially)
SLOT_ANGLE = {"red_fixed": 45.0, "loose": 135.0, "empty": 225.0, "blue_fixed": 315.0}
ROTOR_TOP = Z0 + H

# -- the magnets (nickel-plated NdFeB blocks) -------------------------------------------------------------------------
MAG = (0.036, 0.012, 0.050)  # along the rim, across, tall
MAG_MASS = 0.16
F_REF, D_REF, F_CAP = 8.5, 0.11, 12.0  # the pull between two magnets: F_REF at D_REF, as 1/d^4, at most F_CAP (N)
K_STEEL, F_STEEL, F_SINK = 1200.0, 8.0, 1.0  # the steel round a slot pulls a magnet near its mouth onto its middle
# line (N/m, at most F_STEEL N) and down into it (F_SINK N)
TRAY_C = np.array([0.28, -0.20])
TRAY_YAW = -math.pi / 4  # magnets in the tray stand turned the way the empty slot runs
TRAY_SPOTS = {"blue_magnet": -0.030, "red_magnet": 0.030}  # along the tray's length
TRAY_H, POCKET = 0.010, 0.010  # the tray's floor and the walls round each pocket

STEEL, STEEL_LINE, ALU = (0.30, 0.31, 0.33), (0.40, 0.41, 0.43), (0.70, 0.71, 0.73)
NICKEL, RED, BLUE = (0.78, 0.79, 0.81), (0.85, 0.12, 0.10), (0.12, 0.30, 0.85)
RESIN, BEECH = (0.70, 0.48, 0.12), (0.80, 0.64, 0.42)

# -- scoring ----------------------------------------------------------------------------------------------------------
FLUSH = 0.004  # a magnet's top within this of the rotor's top
IN_SLOT_R, IN_SLOT_T = 0.006, 0.009  # its middle this close to the slot's middle line, across and along
UPRIGHT_DEG = 8.0
JUMP = 0.006  # the loose magnet risen this far: it jumped
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _slot_frame(name):
    """The slot's middle (x, y) and its unit vectors: out from the axis, and along the rim."""
    a = math.radians(SLOT_ANGLE[name])
    out = np.array([math.cos(a), math.sin(a)])
    return RC + R_S * out, out, np.array([-out[1], out[0]])


def _rotor() -> Body:
    """The lamination stack with its four slots, the two potted magnets, the shaft stub and the fixture plate; origin
    on the rotor's axis at its bottom."""
    shapes = [CylShape((0.0, 0.0, H / 2), R_S - SLOT[1] / 2, H, STEEL),  # the core, inside the slots
              CylShape((0.0, 0.0, H + 0.020), 0.0125, 0.040, ALU),  # the shaft stub
              BoxShape((0.0, 0.0, -FIX_H / 2), (0.25, 0.25, FIX_H), ALU)]  # the fixture plate
    t_side = 0.010
    for name, deg in SLOT_ANGLE.items():
        a = math.radians(deg)
        yaw = a  # box x along the radius
        out = np.array([math.cos(a), math.sin(a)])
        along = np.array([-out[1], out[0]])
        # the slot's outer wall: a flat plate beyond it
        c = R_S + SLOT[1] / 2 + (R_OUT - R_S - SLOT[1] / 2) / 2
        shapes.append(BoxShape((*(c * out).tolist(), H / 2), (R_OUT - R_S - SLOT[1] / 2, 0.052, H), STEEL, yaw=yaw))
        # its two side walls, square to the rim
        for s in (-1.0, 1.0):
            p = R_S * out + s * (SLOT[0] / 2 + t_side / 2) * along
            shapes.append(BoxShape((*p.tolist(), H / 2), (0.026, t_side, H), STEEL, yaw=yaw))
        # between this slot and the next: three blocks filling the ring
        for k in (30.0, 45.0, 60.0):
            b = a + math.radians(k)
            q = (R_S + 0.002) * np.array([math.cos(b), math.sin(b)])
            shapes.append(BoxShape((*q.tolist(), H / 2), (0.030, 0.040, H), STEEL, yaw=b))
        mouth = R_S * out
        if name.endswith("fixed"):  # set in amber resin, red or blue face up
            shapes.append(BoxShape((*mouth.tolist(), MAG[2] / 2), MAG, NICKEL, yaw=a + math.pi / 2))
            shapes.append(BoxShape((*mouth.tolist(), H + 0.0003), (SLOT[0], SLOT[1], 0.0006), RESIN,
                                   yaw=a + math.pi / 2, collide=False))
            shapes.append(BoxShape((*mouth.tolist(), H + 0.0008), (MAG[0] - 0.002, MAG[1] - 0.002, 0.0006),
                                   RED if name.startswith("red") else BLUE, yaw=a + math.pi / 2, collide=False))
    for k in range(1, 8):  # the laminations' lines round the side
        shapes.append(CylShape((0.0, 0.0, k * H / 8), R_OUT + 0.0006, 0.0010, STEEL_LINE, collide=False))
    return Body.build("rotor", pos=(RC[0], RC[1], Z0), mass=3.0, kinematic=True, shapes=shapes, color=STEEL,
                      friction=(0.10, 0.10), slick=True)


def _tray() -> Body:
    """A beech tray with a pocket for each magnet; origin at the middle of its floor's top."""
    shapes = [BoxShape((0.0, 0.0, -TRAY_H / 2), (0.120, 0.050, TRAY_H), BEECH)]
    pocket = (MAG[0] + 0.006, MAG[1] + 0.006)
    w = 0.006
    for along in TRAY_SPOTS.values():
        for sx in (-1.0, 1.0):  # the walls at the pocket's ends
            shapes.append(BoxShape((along + sx * (pocket[0] / 2 + w / 2), 0.0, POCKET / 2), (w, pocket[1] + 2 * w,
                                                                                          POCKET), BEECH))
        for sy in (-1.0, 1.0):  # and along its sides
            shapes.append(BoxShape((along, sy * (pocket[1] / 2 + w / 2), POCKET / 2), (pocket[0], w, POCKET), BEECH))
    return Body.build("tray", pos=(TRAY_C[0], TRAY_C[1], TOP + TRAY_H), mass=0.3, kinematic=True, shapes=shapes,
                      yaw=TRAY_YAW, color=BEECH)


def _tray_spot(name) -> np.ndarray:
    along = np.array([math.cos(TRAY_YAW), math.sin(TRAY_YAW)])
    return TRAY_C + TRAY_SPOTS[name] * along


def _magnet(name, pos_xy, yaw, top_color) -> Body:
    """A magnet; origin at its middle. Its top face (red: north, blue: south) shows its pole."""
    shapes = [BoxShape((0.0, 0.0, 0.0), MAG, NICKEL),
              BoxShape((0.0, 0.0, MAG[2] / 2 + 0.0003), (MAG[0] - 0.002, MAG[1] - 0.002, 0.0006), top_color,
                       collide=False)]
    loose = name == "loose_magnet"
    z = (Z0 if loose else TOP + TRAY_H) + MAG[2] / 2
    return Body.build(name, pos=(float(pos_xy[0]), float(pos_xy[1]), z), mass=MAG_MASS, shapes=shapes, yaw=yaw,
                      color=NICKEL, friction=(0.15, 0.10) if loose else (1.2, 1.0), slick=loose)


_LOOSE_C, _, _ = _slot_frame("loose")
OBJECTS = [_rotor(), _tray(),
           _magnet("loose_magnet", _LOOSE_C, math.radians(SLOT_ANGLE["loose"]) + math.pi / 2, BLUE),
           _magnet("red_magnet", _tray_spot("red_magnet"), TRAY_YAW, RED),
           _magnet("blue_magnet", _tray_spot("blue_magnet"), TRAY_YAW, BLUE)]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"jumped": False, "rise": 0.0, "tcps": None, "poses": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("ROTOR_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _set_force(asset, f_world, rot) -> None:
    """A force (world frame) at the body's middle until the next step."""
    import torch

    f = rot.T @ np.asarray(f_world, float)
    t = torch.tensor(f, dtype=torch.float32, device=asset.device).view(1, 1, 3)
    asset.permanent_wrench_composer.set_forces_and_torques(forces=t, torques=torch.zeros_like(t), is_global=False)


def _pull(d: np.ndarray) -> np.ndarray:
    """The pull (N) on a magnet toward another magnet the vector d away (opposite poles)."""
    n = float(np.linalg.norm(d))
    if n < 1e-4 or n > 0.30:
        return np.zeros(3)
    return min(F_CAP, F_REF * (D_REF / n) ** 4) * d / n


def _steel(pos) -> np.ndarray:
    """The steel's pull on a magnet near the empty slot's mouth: onto the slot's middle line and down into it."""
    c, _, _ = _slot_frame("empty")
    off = np.asarray(pos[:2], float) - c
    bottom = float(pos[2]) - MAG[2] / 2 - ROTOR_TOP
    if bottom > 0.02 or bottom < -MAG[2] or float(np.linalg.norm(off)) > 0.025:
        return np.zeros(3)
    side = -K_STEEL * off
    n = float(np.linalg.norm(side))
    if n > F_STEEL:
        side *= F_STEEL / n
    return np.array([side[0], side[1], -F_SINK if bottom > -MAG[2] + 0.004 else 0.0])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    poses = {}
    for name in ("loose_magnet", "red_magnet", "blue_magnet"):
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        poses[name] = (p[:3] - origin, rotation(p[3:7]))
    _EP["poses"] = poses
    lp, lr = poses["loose_magnet"]
    total = np.zeros(3)
    for name, sign in (("red_magnet", 1.0), ("blue_magnet", -1.0)):  # red attracts the blue-topped loose magnet
        mp, mr = poses[name]
        f = sign * _pull(mp - lp)
        total += f
        _set_force(base.scene[name], -f + _steel(mp), mr)
    _set_force(base.scene["loose_magnet"], total, lr)
    rise = float(lp[2] - (Z0 + MAG[2] / 2))
    _EP["rise"] = rise
    if rise > JUMP:
        _EP["jumped"] = True
    _EP["tcps"] = _tcps(base, origin)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status()
            tl, tr = _EP["tcps"]
            rp = poses["red_magnet"][0]
            print(f"[rotor] t={_DEBUG['steps'] / 20:5.1f}s rise={rise * 1000:5.1f}mm jumped={_EP['jumped']} "
                  f"pull={np.linalg.norm(total):5.2f}N red={np.round(rp * 1000).astype(int).tolist()} "
                  f"red_slot={st['red']} blue_slot={st['blue']} "
                  f"L={np.round(tl * 1000).astype(int).tolist()} R={np.round(tr * 1000).astype(int).tolist()} "
                  f"R-red_top={np.round((tr - rp - [0, 0, MAG[2] / 2]) * 1000).astype(int).tolist()} "
                  f"home={st['home']} score={_points(st)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _in_empty_slot(pos, rot) -> dict:
    """Where a magnet is against the empty slot: in its footprint, how deep, flush, upright."""
    c, out, along = _slot_frame("empty")
    v = np.asarray(pos, float)
    r = float((v[:2] - c) @ out)
    t = float((v[:2] - c) @ along)
    up = math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))
    top = float(v[2]) + MAG[2] / 2 * math.cos(math.radians(up))
    inside = abs(r) < SLOT[1] / 2 and abs(t) < SLOT[0] / 2
    return {"half": inside and top < ROTOR_TOP + MAG[2] / 2,
            "flush": inside and abs(r) < IN_SLOT_R and abs(t) < IN_SLOT_T and abs(top - ROTOR_TOP) < FLUSH
            and up < UPRIGHT_DEG}


def _status() -> dict:
    poses = _EP["poses"]
    out = {"red": {"half": False, "flush": False}, "blue": {"half": False, "flush": False}}
    for name in ("red", "blue"):
        if f"{name}_magnet" in poses:
            out[name] = _in_empty_slot(*poses[f"{name}_magnet"])
    out["jumped"] = _EP["jumped"]
    out["home"] = _arms_home()
    return out


def _points(s) -> float:
    if s["red"]["flush"] and not s["jumped"] and s["home"]:
        return 1.0
    if s["red"]["flush"] and not s["jumped"]:
        return 0.5
    return 0.25 if (s["red"]["half"] or s["blue"]["half"]) else 0.0


def score(objects) -> float:
    """Judged at the end, except the jump, which counts at any time. Jumped: the loose magnet rose more than 6 mm in its
    slot at some point. Flush: the red-topped magnet in the empty slot, upright within 8 degrees, its middle within 6 mm
    of the slot's middle line across and 9 mm along, its top within 4 mm of the rotor's top. 1.0: the red-topped magnet
    flush, the loose magnet never jumped, and both arms back where they started (each gripper's fingertip centre (TCP)
    within 5 cm of its start); 0.5: the red-topped magnet flush and the loose magnet never jumped; 0.25: either magnet
    at least half its height inside the empty slot; else 0.0."""
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
    """Full score with the red-topped magnet and the loose magnet still."""
    return (score(objects) >= 1.0 and objects["red_magnet"].resting() and objects["loose_magnet"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes down on the loose magnet's top with its fingers closed, leaning a little outward, and presses it
# down. The right hand comes straight down on the red-topped magnet in the tray, fingers closing across it 6 mm below
# its top, lifts it, carries it over the empty slot, waits for the left to press, then lowers it about half way into
# the slot, lets go, closes its fingers and presses it in flush. Both hands rise and both arms go back to where they
# started.

GRIP_YAW = -math.pi / 4  # fingers closing along (1, 1): across the magnets (the empty slot's radius)
GRIP_DEPTH = 0.006  # the right's TCP this far below the magnet's top
PRE_OPEN = -(MAG[1] / 2 + 0.007)
PRESS = 0.030  # aim this far below the surface pressed
HOLD_YAW = math.pi / 4  # the left's closed fingers lie across the loose magnet (along its slot's radius)
AIM = np.array([0.0, -0.017])  # the right aims this far off the empty slot over the rotor (the pull bends its arm)
SKIM = 0.015  # the new magnet's bottom this far above the rotor's top as it comes in over it
ENTRY = np.array([0.24, -0.165])  # where it comes down beside the rotor before going in over it
LEAN_DEG = 15.0  # the left leans outward (its wrist away from the right's)
FINGER_REACH = 0.013  # the closed fingertips this far below the TCP
CARRY_Z = TOP + 0.160
LEAD_S, LEAD_STEPS = 3.0, 15


def _rx(a: float) -> np.ndarray:
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


def _sweep(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _sweep(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _sweep(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    c, _, _ = _slot_frame("loose")
    g = _rx(-math.radians(LEAN_DEG)) @ grip_rotation(HOLD_YAW)
    touch = np.array([c[0], c[1], ROTOR_TOP + FINGER_REACH])
    out = _lead_in("left", touch + [0.0, 0.0, 0.05], g, "over the loose magnet", FINGER_CLOSED)
    out += [_w(1.0, touch + [0.0, 0.0, 0.005], FINGER_CLOSED, "down onto it", g),
            _w(0.8, touch - [0.0, 0.0, PRESS], FINGER_CLOSED, "press it down", g)]
    held_at = sum(w.duration for w in out)
    out += [_w(max(release_at - held_at, 0.3), touch - [0.0, 0.0, PRESS], FINGER_CLOSED,
               "hold it down while the new magnet goes in", g),
            _w(0.8, touch + [0.0, 0.0, 0.05], FINGER_CLOSED, "rise", g)]
    out += _go_home("left", touch + [0.0, 0.0, 0.05], g, FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s", out[-1].yaw))
    return out, held_at


def _right(start: float) -> tuple[list[Waypoint], float]:
    g = grip_rotation(GRIP_YAW)
    spot = _tray_spot("red_magnet")
    top0 = TOP + TRAY_H + MAG[2]
    take = np.array([spot[0], spot[1], top0 - GRIP_DEPTH])
    c, _, _ = _slot_frame("empty")
    skim_z = ROTOR_TOP + SKIM + MAG[2] - GRIP_DEPTH
    beside = np.array([ENTRY[0], ENTRY[1], skim_z])
    over = np.array([c[0] + AIM[0], c[1] + AIM[1], skim_z])
    low = np.array([c[0] + AIM[0], c[1] + AIM[1], ROTOR_TOP + FINGER_REACH + 0.005])  # fingertips just above the top
    out = _lead_in("right", take + [0.0, 0.0, 0.06], g, "over the red-topped magnet", PRE_OPEN)
    out += [_w(1.0, take, PRE_OPEN, "down round it", g),
            _w(0.6, take, FINGER_CLOSED, "take it", g),
            _w(1.0, take + [0.0, 0.0, CARRY_Z - take[2]], FINGER_CLOSED, "lift it", g),
            _w(1.2, beside + [0.0, 0.0, CARRY_Z - skim_z], FINGER_CLOSED, "carry it beside the rotor", g),
            _w(1.0, beside, FINGER_CLOSED, "lower it to just above the rotor's top", g)]
    t = sum(w.duration for w in out)
    if start > t:
        out.append(_w(start - t, beside, FINGER_CLOSED, "wait for the loose magnet to be held down", g))
    out += [_w(2.0, over, FINGER_CLOSED, "bring it in over the empty slot", g),
            _w(2.0, low, FINGER_CLOSED, "lower it into the slot", g),
            _w(0.5, low, PRE_OPEN, "let go", g),
            _w(0.6, low + [0.0, 0.0, 0.045], PRE_OPEN, "rise above it", g),
            _w(0.4, low + [0.0, 0.0, 0.045], FINGER_CLOSED, "close the fingers", g),
            _w(1.5, np.array([c[0], c[1], ROTOR_TOP + FINGER_REACH - PRESS]), FINGER_CLOSED,
               "press it in flush with the rotor's top", g),
            _w(0.6, np.array([c[0], c[1], ROTOR_TOP + FINGER_REACH - PRESS]), FINGER_CLOSED, "hold", g)]
    pressed_at = sum(w.duration for w in out)
    up = np.array([c[0], c[1], ROTOR_TOP + 0.06])
    out += [_w(0.8, up, FINGER_CLOSED, "rise", g)]
    out += _go_home("right", up, g, FINGER_CLOSED)
    return out, pressed_at


def demo():
    _, held_at = _left(0.0, 0.0)
    right, pressed_at = _right(held_at)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_CLOSED, "wait at the start", right[-1].yaw))
    left, _ = _left(pressed_at + 0.3, total)
    return {"left": left, "right": right}
