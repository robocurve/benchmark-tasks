"""A wardrobe rail is refitted between the two side panels of a wardrobe. The left end goes into a closed socket,
the right end drops into an open U socket. The rail is longer than the gap minus the closed socket's depth, so it can't
go in level: the left end is fed into the closed socket with the rail tilted, the right end held high, then the rail
is swung down level into the U while the left end slides home. One gripper on each side of the rail.
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import env, on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Fit the wardrobe rail into its two end supports."

WHITE, CHROME, GREY, FOAM = (0.92, 0.91, 0.88), (0.78, 0.80, 0.83), (0.35, 0.36, 0.38), (0.25, 0.45, 0.80)
AX = 0.32             # x of the socket axis (m)
ZS = 0.08             # socket axis height above the wardrobe base's underside (m)
GAP = 0.300           # inner face of each side panel, |y| (m)
L, H, W = 0.592, 0.030, 0.015   # oval rail: length, height, width (m), modelled as a 30 x 15 mm bar
CUP = 0.010           # closed socket depth (m): lip at y = GAP - CUP
U_DEPTH = 0.015       # open socket depth along y (m)
BX, BZ, WALL = 0.021, 0.038, 0.004   # closed socket bore: width (x), height (z); wall

_lip = GAP - CUP / 2
WARDROBE = part("wardrobe", pos=(AX, 0.0, TOP), mass=25.0, friction=1.2, color=WHITE, boxes=[
    ((0, 0, 0.009), (0.20, 2 * GAP + 0.036, 0.018), WHITE),                 # base board
    ((0, GAP + 0.009, 0.089), (0.20, 0.018, 0.16), WHITE),                  # left side panel
    ((0, -GAP - 0.009, 0.089), (0.20, 0.018, 0.16), WHITE),                 # right side panel
    # closed end support on the left panel: an oval cup, here a 21 x 38 mm bore, 10 mm deep
    ((0, _lip, ZS + BZ / 2 + WALL / 2), (BX + 2 * WALL, CUP, WALL), CHROME),
    ((0, _lip, ZS - BZ / 2 - WALL / 2), (BX + 2 * WALL, CUP, WALL), CHROME),
    ((-BX / 2 - WALL / 2, _lip, ZS), (WALL, CUP, BZ), CHROME),
    ((BX / 2 + WALL / 2, _lip, ZS), (WALL, CUP, BZ), CHROME),
    # open end support on the right panel: floor at the rail's underside, two cheeks, open on top and toward the rail
    ((0, -GAP + U_DEPTH / 2, ZS - H / 2 - WALL / 2), (BX + 2 * WALL, U_DEPTH, WALL), CHROME),
    ((-BX / 2 - WALL / 2, -GAP + U_DEPTH / 2, ZS - H / 2 + 0.015), (WALL, U_DEPTH, 0.030), CHROME),
    ((BX / 2 + WALL / 2, -GAP + U_DEPTH / 2, ZS - H / 2 + 0.015), (WALL, U_DEPTH, 0.030), CHROME)])
RAIL0 = (0.16, 0.0, TOP + 0.05 + H / 2 + 0.001)
RAIL = part("rail", pos=RAIL0, mass=0.35, friction=1.2, color=CHROME, boxes=[((0, 0, 0), (W, L, H), CHROME)])
OBJECTS = [WARDROBE, RAIL,
           Box("foam_left", pos=(0.16, 0.10, TOP + 0.025), size=(0.06, 0.06, 0.05), color=FOAM),
           Box("foam_right", pos=(0.16, -0.10, TOP + 0.025), size=(0.06, 0.06, 0.05), color=FOAM)]

_S: dict = {}


@on_reset
def _reset(environment=None):
    _S.update(recent=[], step=0, scored=None, last=0.0)


@on_step
def _count(environment=None):
    _S["step"] += 1


_reset()


def _arm_clear(rail) -> bool:
    """Both arms: fingers at least 90% open and both pad tips at least 3 cm from the rail's axis."""
    e = env()
    if e is None:
        return False
    scene = e.unwrapped.scene
    a0, a1 = rail.to_world((0, -L / 2, 0)), rail.to_world((0, L / 2, 0))
    org = np.asarray(getattr(scene.env_origins, "torch", scene.env_origins)[0].detach().cpu(), float)
    for name in ("left_robot", "right_robot"):
        arm = scene[name]
        q = np.asarray(getattr(arm.data.joint_pos, "torch", arm.data.joint_pos)[0].detach().cpu(), float)
        fingers = q[arm.find_joints(["joint7", "joint8"], preserve_order=True)[0]]
        if fingers.mean() / FINGER_OPEN < 0.9:
            return False
        tips = getattr(arm.data.body_pos_w, "torch", arm.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        for t in tips[arm.find_bodies(["tip_left", "tip_right"], preserve_order=True)[0]] - org:
            u = np.clip(np.dot(t - a0, a1 - a0) / np.dot(a1 - a0, a1 - a0), 0, 1)
            if np.linalg.norm(t - (a0 + u * (a1 - a0))) < 0.03:
                return False
    return True


def _arm_dbg():
    e = env()
    scene = e.unwrapped.scene
    org = np.asarray(getattr(scene.env_origins, "torch", scene.env_origins)[0].detach().cpu(), float)
    out = []
    for name in ("left_robot", "right_robot"):
        arm = scene[name]
        q = np.asarray(getattr(arm.data.joint_pos, "torch", arm.data.joint_pos)[0].detach().cpu(), float)
        f = q[arm.find_joints(["joint7", "joint8"], preserve_order=True)[0]]
        tips = getattr(arm.data.body_pos_w, "torch", arm.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        t = tips[arm.find_bodies(["tip_left", "tip_right"], preserve_order=True)[0]] - org
        out.append(f"{name[0]} f={np.round(f*1000,1)} tips={np.round(t*1000,0).tolist()}")
    return " ".join(out)


def _state(objects):
    w, r = objects["wardrobe"], objects["rail"]
    left = w.to_local(r.to_world((0, L / 2, 0)))     # rail end centres in the wardrobe frame (m)
    right = w.to_local(r.to_world((0, -L / 2, 0)))
    axis = right - left
    tilt = math.degrees(math.asin(abs(axis[2]) / np.linalg.norm(axis)))
    left_in = (left[1] >= GAP - CUP + 0.008 and abs(left[0]) <= (BX - W) / 2 + 0.001
               and abs(left[2] - ZS) <= (BZ - H) / 2 + 0.001)     # end centre inside the bore, 8 mm or more deep
    right_in = (right[1] <= -GAP + U_DEPTH - 0.005 and abs(right[0]) <= BX / 2 and abs(right[2] - ZS) <= 0.003)
    return left, right, tilt, left_in, right_in


def score(objects) -> float:
    """End state only. 0.4 if the left end is at least 8 mm into the closed socket's bore; 0.4 if the
    right end rests on the U socket's floor (height within 3 mm, at least 5 mm over it); 0.2 if the rail is level
    within 1 degree with both ends in. Credit only if both arms have let go (fingers open, pads 3 cm from the rail)
    and the rail has been still (moved under 1 mm) for the last second."""
    if _S["scored"] == _S["step"]:
        return _S["last"]
    _S["scored"] = _S["step"]
    left, right, tilt, left_in, right_in = _state(objects)
    pos = np.asarray(objects["rail"].pos, float)
    recent = _S["recent"]
    recent.append((pos.copy(), _arm_clear(objects["rail"])))
    del recent[:-20]
    still = len(recent) == 20 and all(abs(p - pos).max() < 0.001 and c for p, c in recent)
    s = 0.4 * left_in + 0.4 * right_in + (0.2 if left_in and right_in and tilt <= 1.0 else 0.0)
    _S["last"] = s if still else 0.0
    if os.environ.get("RAIL_DEBUG") and _S["step"] % 20 == 0:
        print(f"[dbg] step {_S['step']} left {np.round(left*1000,1)} right {np.round(right*1000,1)} tilt {tilt:.1f} "
              f"in {left_in},{right_in} still {still} score {_S['last']:.2f} | {_arm_dbg()}", flush=True)
    return _S["last"]


# -- scripted demo: both arms pinch the rail across its width (fingers closing along x, so the rail can pivot in them),
# carry it in tilted, feed the left end into the closed socket, swing it level into the U, let go and park. --------
GRIP = 0.18          # grasp points, m either side of the rail centre
TCP_UP = 0.008       # TCP above the pad tips


def _grips(x, tip_y, tip_z, theta_deg):
    """TCP targets for both arms, rail left end at (x, tip_y, tip_z) and the right end raised by theta."""
    t = math.radians(theta_deg)
    cy, cz = tip_y - (L / 2) * math.cos(t), tip_z + (L / 2) * math.sin(t)
    lp = (x, cy + GRIP * math.cos(t), cz - GRIP * math.sin(t) + TCP_UP)
    rp = (x, cy - GRIP * math.cos(t), cz + GRIP * math.sin(t) + TCP_UP)
    return lp, rp


def demo():
    v = os.environ.get("RAIL_VARIANT", "")
    Y = math.pi / 2
    YR = float(os.environ.get("RAIL_YR", math.pi / 2))
    zs = TOP + ZS
    DZ = float(os.environ.get("RAIL_DZ", 0.0))  # aim offset for the left end's height (it sags in the pinch)
    YB = float(os.environ.get("RAIL_YB", 0.006))   # the rail lags this far toward the right in the pinch: aim further left
    x0, z0 = RAIL0[0], RAIL0[2]
    keys = [  # (duration, x, left tip y, left tip z, tilt degrees, finger, label)
        (2.0, x0, L / 2, z0 + 0.06, 0.0, FINGER_OPEN, "over the rail"),
        (1.5, x0, L / 2, z0, 0.0, FINGER_OPEN, "down"),
        (0.6, x0, L / 2, z0, 0.0, FINGER_OPEN, "settle"),
        (0.8, x0, L / 2, z0, 0.0, FINGER_CLOSED, "pinch"),
        (1.5, x0, L / 2, z0 + 0.06, 0.0, FINGER_CLOSED, "lift"),
        (2.5, x0, 0.286, zs + 0.01, 13.0, FINGER_CLOSED, "tilt, right end up"),
        (2.5, AX, 0.286, zs - DZ, 13.0, FINGER_CLOSED, "carry in between the panels"),
        (1.5, AX, 0.296 + YB, zs - DZ, 13.0, FINGER_CLOSED, "feed the left end in"),
        (1.5, AX, 0.297 + YB, zs - DZ * 0.8, 9.0, FINGER_CLOSED, "swing down"),
        (1.5, AX, 0.298 + YB, zs - DZ * 0.5, 5.0, FINGER_CLOSED, "swing down"),
        (1.5, AX, 0.299 + YB, zs - DZ * 0.2, 2.0, FINGER_CLOSED, "swing down"),
        (1.5, AX, 0.2995 + YB, zs, 0.0, FINGER_CLOSED, "level, into the U"),
        (0.8, AX, 0.2995 + YB, zs, 0.0, FINGER_CLOSED, "settle"),
    ]
    if v == "level":    # wrong: try to put it in level, left end straight at the socket
        keys = keys[:5] + [(2.5, x0, 0.286, zs, 0.0, FINGER_CLOSED, "level"),
                           (2.5, AX, 0.286, zs, 0.0, FINGER_CLOSED, "carry in"),
                           (2.0, AX, 0.2995, zs, 0.0, FINGER_CLOSED, "push left")]
    if v == "on_top":   # wrong: lay it on top of both sockets
        keys = keys[:5] + [(2.5, AX, 0.286, zs + 0.04, 0.0, FINGER_CLOSED, "carry over"),
                           (1.5, AX, 0.286, zs + 0.03, 0.0, FINGER_CLOSED, "lower onto the tops")]
    if v == "nothing":  # wrong: both arms park
        return {"left": [Waypoint(3.0, (0.18, 0.36, TOP + 0.22), FINGER_OPEN, "park")],
                "right": [Waypoint(3.0, (0.18, -0.36, TOP + 0.22), FINGER_OPEN, "park")]}
    if v == "hold":     # wrong: right, but never let go
        keys = keys + [(4.0, AX, 0.2995, zs, 0.0, FINGER_CLOSED, "hold")]
    left, right = [], []
    XB = float(os.environ.get("RAIL_XB", 0.004))   # the left wrist turns a little short, so the left end sits ~4 mm toward -x
    for i, (dur, x, ty, tz, th, f, lab) in enumerate(keys):
        lp, rp = _grips(x, ty, tz, th)
        if i >= 5:
            lp = (lp[0] + XB, lp[1], lp[2])
        left.append(Waypoint(dur, lp, f, lab, Y))
        right.append(Waypoint(dur, rp, f, lab, YR))
    if v != "hold":
        last_l, last_r = left[-1].pos, right[-1].pos
        left += [Waypoint(0.8, last_l, FINGER_OPEN, "let go", Y),
                 Waypoint(1.2, (last_l[0] - 0.05, last_l[1], last_l[2] + 0.06), FINGER_OPEN, "back off", Y),
                 Waypoint(2.0, (0.18, 0.36, TOP + 0.22), FINGER_OPEN, "park"),
                 Waypoint(2.0, (0.18, 0.36, TOP + 0.22), FINGER_OPEN, "hold")]
        right += [Waypoint(0.8, last_r, FINGER_OPEN, "let go", YR),
                  Waypoint(1.2, (last_r[0] - 0.05, last_r[1], last_r[2] + 0.06), FINGER_OPEN, "back off", YR),
                  Waypoint(2.0, (0.18, -0.36, TOP + 0.22), FINGER_OPEN, "park"),
                  Waypoint(2.0, (0.18, -0.36, TOP + 0.22), FINGER_OPEN, "hold")]
    return {"left": left, "right": right}
