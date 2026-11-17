"""Price-mark three stock boxes with a one-line hand price labeller, then lay the labeller back on its pad.

Real product class: a one-line hand price labeller (235 x 145 x 51 mm class, about 0.2 kg empty, 0.285 kg
with a roll; 21 x 12 mm labels -- seller listings, README). Squeezing the trigger through its full travel prints and
peels one label onto the applicator roller at the nose; the label goes on where the roller is pressed against the
item. The trigger force has no published figure: 25 +- 10 N at the trigger pad is a disclosed estimate (README).

Layout in station coordinates (TOP = station.TABLE_TOP, +X forward, +Y left; arm bases at y = +-0.305 m):
- the labeller lies on its side on a fixed pad at (PAD_X, PAD_Y), nose pointing forward-left (yawed 45 deg), so its
  roller meets a surface facing the arms at the labeller's working angle (nose 45 deg down from its long axis);
- three closed stock boxes (fixed, a carton row) stand in a row at the far side, front faces at x = FACE_X facing the
  arms, each with a printed price spot (a 30 x 20 mm square) on its front face at the roller's working height.
Label model (precedent #1893 carton-tape-seal, merged: tape cells stick by a position rule): a label is laid on a box
face when one full trigger cycle (>= 90 % travel, then back below 20 %) runs while the roller is within 1 mm of (touching) that
face and square to it. The label is a rigid 21 x 12 mm tile carried by three slide joints of its box (along the face,
up the face, out of the face) that the rule drives to the roller's contact point; no adhesion or film is simulated.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import labkit  # noqa: E402
import pitch_demo  # noqa: E402
from labkit import Joint, on_reset, on_step, part  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

# -- labeller geometry (part frame: origin under the trigger hinge on the table; long axis +x with the nose at +x,
#    the labeller's 'up' = +y (head), handle and trigger at -y, thickness along z) -----------------------------------
TH = 0.051  # thickness (lies on its side)
GUN_MASS = 0.255  # body; the trigger adds 0.03 kg (0.285 kg loaded, listing)
TRAVEL = math.radians(20.0)  # trigger swing to full squeeze (estimate: about 20 mm at the pad)
R_PAD = 0.060  # hinge to the trigger pad
F_FULL = 25.0  # N at the pad at full travel (disclosed estimate 25 +- 10 N)
K_TRIG = F_FULL * R_PAD / TRAVEL
HEAD_C, HEAD_S = (0.040, 0.035), (0.235, 0.050)  # head (label roll, print head) centre / size in x, y
HANDLE_C, HANDLE_S = (-0.050, -0.045), (0.032, 0.090)
TRIG_C, TRIG_S = (0.004, -0.045), (0.016, 0.075)
ROLLER_C, ROLLER_R = (0.150, 0.000), 0.008  # applicator roller at the nose, axis along z
YAW = math.pi / 4  # pad yaw: the roller's working direction (local (1, -1)/sqrt2) points along world +x
N_LOCAL = np.array([math.sqrt(0.5), -math.sqrt(0.5), 0.0])  # working direction in the part frame
REACH = float(N_LOCAL[:2] @ np.array(ROLLER_C)) + ROLLER_R  # roller's front point along the working direction

GREY, RED, BLACK, KRAFT, WHITE, DARK = (0.55, 0.56, 0.58), (0.80, 0.15, 0.10), (0.10, 0.10, 0.11), \
    (0.70, 0.55, 0.36), (0.93, 0.93, 0.90), (0.20, 0.20, 0.22)
ORANGE = (1.00, 0.45, 0.05)

TRIGGER = Joint("trigger", kind="hinge", axis="Z", anchor=(0.0, 0.0, TH / 2), lower=-TRAVEL, upper=0.0,
                stiffness=K_TRIG, damping=0.02, target=0.0, initial=0.0, mass=0.03,
                boxes=(((TRIG_C[0], TRIG_C[1], TH / 2), (TRIG_S[0], TRIG_S[1], TH * 0.9), RED),))

PAD_X, PAD_Y = 0.215, -0.010  # trigger-hinge origin of the labeller on its pad
PAD_Z = 0.003  # pad thickness
GUN = part("gun", (PAD_X, PAD_Y, TOP + PAD_Z), mass=GUN_MASS, friction=0.8, yaw=YAW, joints=(TRIGGER,),
           boxes=(((HEAD_C[0], HEAD_C[1], TH / 2), (HEAD_S[0], HEAD_S[1], TH), GREY),
                  ((HANDLE_C[0], HANDLE_C[1], TH / 2), (HANDLE_S[0], HANDLE_S[1], TH), BLACK)),
           cylinders=(((ROLLER_C[0], ROLLER_C[1], TH / 2), ROLLER_R, TH * 0.6, BLACK, "Z"),))

# -- boxes -------------------------------------------------------------------------------------------------------------
FACE_X = 0.420  # front faces of the box row
BOX_D, BOX_W, BOX_H = 0.080, 0.090, 0.070  # depth (x), width (y), height
BOX_Y = (0.230, 0.130, 0.030)  # box centres, left to right as seen from the arms
LIFT = 0.014  # the labeller is carried this far above the pad (it sags a few mm in the jaws; less drags it on the table)
SPOT_Z = 0.037  # price spot centre height above the table (the roller centre while the labeller is carried just clear of the table)
SPOT_W, SPOT_H = 0.030, 0.020  # printed price spot (square) on each face
LAB_W, LAB_H, LAB_T = 0.021, 0.012, 0.0008  # label 21 x 12 mm
LAB_HIDE = 0.004  # a label not yet laid sits this far inside the face (hidden)
LAB_K, LAB_D = 4000.0, 20.0


def _box_part(i):
    """Box i: one fixed body (frame at the centre of its front face on the table) with the price spot printed flush
    on the face, and the label tile on three slide joints: 'u' along the face (y), 'v' up the face (z), 'w' out."""
    c = (FACE_X + BOX_D / 2, BOX_Y[i], TOP)
    body = (((0.0, 0.0, BOX_H / 2), (BOX_D, BOX_W, BOX_H), KRAFT),
            ((-BOX_D / 2 + 0.0002, 0.0, SPOT_Z), (0.0004, SPOT_W, SPOT_H), DARK))  # printed spot, flush
    hid = (-BOX_D / 2 + 0.010, 0.0, SPOT_Z)  # carrier blocks hidden inside the box
    u = Joint("u", kind="slide", axis="Y", anchor=hid, lower=-0.040, upper=0.040, stiffness=LAB_K, damping=LAB_D,
              boxes=((hid, (0.004, 0.004, 0.004), KRAFT),), mass=0.002)
    v = Joint("v", kind="slide", axis="Z", anchor=hid, lower=-0.030, upper=0.030, stiffness=LAB_K, damping=LAB_D,
              boxes=((hid, (0.004, 0.004, 0.004), KRAFT),), mass=0.002, parent="u")
    tile = (-BOX_D / 2 - LAB_T / 2 + LAB_HIDE, 0.0, SPOT_Z)
    w = Joint("w", kind="slide", axis="X", anchor=tile, lower=-LAB_HIDE, upper=0.0, stiffness=LAB_K, damping=LAB_D,
              boxes=((tile, (LAB_T, LAB_W, LAB_H), ORANGE),), mass=0.002, parent="v")
    return part(f"box{i}", c, fixed=True, friction=0.5, boxes=body, joints=(u, v, w))


BOXES = tuple(_box_part(i) for i in range(3))
PAD = part("pad", (PAD_X + 0.03, PAD_Y + 0.03, TOP), fixed=True, friction=0.6,
           boxes=(((0.0, 0.0, PAD_Z / 2), (0.260, 0.260, PAD_Z), DARK),))
OBJECTS = (GUN, *BOXES, PAD)

# -- rules and scoring thresholds --------------------------------------------------------------------------------------
FIRE = 0.90  # trigger travel fraction that prints and peels a label
REARM = 0.20  # travel fraction the trigger must return below to finish the cycle
NEAR = 0.001  # roller touching the face (within 1 mm: contact, not hovering) during the cycle
SQUARE_DEG = 15.0  # working direction within 15 deg of the face normal, labeller flat within 15 deg
SPOT_TOL = 0.010  # label centre within 10 mm of the printed spot centre
PAD_TOL, FLAT_DEG = 0.020, 10.0  # labeller back on its pad: hinge origin within 20 mm, lying flat within 10 deg
HAND_CLEAR = 0.150  # grippers this far from the labeller: the trial ends with the arms clear of the camera view
S_LABEL, S_PAD = 0.25, 0.25
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
PASS_SCORE = 1.0 - 1e-9
STATE_EVERY = 40

_trial: dict = {}
_ids: dict = {}


def _fresh_trial() -> dict:
    return {"step": -1, "tcps": None, "armed": True, "cycle_box": None, "labels": [0, 0, 0],
            "where": [None, None, None], "done_s": 0.0}


_trial.update(_fresh_trial())


def _set_test_trial(**kw) -> None:
    """Explicit test hook for offline scoring tests (never called by demo())."""
    for k, v in kw.items():
        if k not in _trial:
            raise KeyError(k)
        _trial[k] = v


@on_reset
def _reset(_env) -> None:
    _trial.clear()
    _trial.update(_fresh_trial())
    _ids.clear()


def _rot(ps) -> np.ndarray:
    x, y, z, w = (float(v) for v in ps.quat_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def travel(gun) -> float:
    """Trigger travel as a fraction of full squeeze (0 released, 1 full)."""
    return float(gun.joints.get("trigger", 0.0)) / -TRAVEL


def roller_front(gun) -> tuple[np.ndarray, np.ndarray, float]:
    """World point of the roller's front along its working direction, that direction, and the labeller's tilt (deg)."""
    R = _rot(gun)
    p = np.asarray(gun.pos, float) + R @ np.array([ROLLER_C[0], ROLLER_C[1], TH / 2]) + ROLLER_R * (R @ N_LOCAL)
    n = R @ N_LOCAL
    tilt = math.degrees(math.acos(float(np.clip(abs(R[2, 2]), 0.0, 1.0))))
    return p, n, tilt


def box_at_roller(gun):
    """Index of the box whose front face the roller is on (within NEAR, inside the face, square to it), else None."""
    p, n, tilt = roller_front(gun)
    if tilt > SQUARE_DEG or math.degrees(math.acos(float(np.clip(n[0], -1.0, 1.0)))) > SQUARE_DEG:
        return None
    if abs(p[0] - FACE_X) > NEAR:
        return None
    for i, by in enumerate(BOX_Y):
        if abs(p[1] - by) <= BOX_W / 2 - LAB_W / 2 and LAB_H / 2 <= p[2] - TOP <= BOX_H - LAB_H / 2:
            return i
    return None


def _label_rule(gun) -> int | None:
    """One trigger cycle with the roller on a face lays one label there; returns the box index when a label is laid."""
    t = travel(gun)
    if _trial["armed"] and t >= FIRE:
        _trial["armed"] = False
        _trial["cycle_box"] = box_at_roller(gun)
        return None
    if not _trial["armed"] and t <= REARM:
        _trial["armed"] = True
        i, _trial["cycle_box"] = _trial["cycle_box"], None
        if i is not None and box_at_roller(gun) == i:
            p, _, _ = roller_front(gun)
            _trial["labels"][i] += 1  # cycle count for the STATE log only; scoring uses the final place
            _trial["where"][i] = (float(p[1] - BOX_Y[i]), float(p[2] - TOP - SPOT_Z))
            return i
    return None


def _arm_tcps(env):
    try:
        base = env.unwrapped
        out = []
        for side in ("left", "right"):
            asset = base.scene[f"{side}_robot"]
            key = f"arm_{side}"
            if key not in _ids:
                _ids[key] = asset.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = labkit._np(asset.data.joint_pos)[0, _ids[key]]
            tcp_p, _ = ik.tcp_pose(q, station.BASES[side])
            out.append(tcp_p)
        return (out[0], out[1])
    except Exception:
        return None


def _show_label(env, i: int) -> None:
    """Drive box i's label tile to the recorded contact point (the rule's result made visible)."""
    import torch

    asset = env.unwrapped.scene[f"box{i}"]
    key = f"box{i}"
    if key not in _ids:
        names = list(asset.joint_names)
        _ids[key] = [names.index(f"{j}_joint") for j in ("u", "v", "w")]
    du, dv = _trial["where"][i]
    tgt = [float(np.clip(du, -0.040, 0.040)), float(np.clip(dv, -0.030, 0.030)), -LAB_HIDE]
    asset.set_joint_position_target(torch.tensor([tgt], device=asset.device), joint_ids=_ids[key])


@on_step
def _watch(env) -> None:
    gun = labkit._part_state(env, labkit._SPECS["gun"])
    _trial["step"] += 1
    _trial["tcps"] = _arm_tcps(env)
    i = _label_rule(gun)
    if i is not None:
        _show_label(env, i)
    if _trial["step"] % STATE_EVERY == STATE_EVERY - 1:
        p, n, tilt = roller_front(gun)
        print("STATE " + json.dumps({
            "step": _trial["step"] + 1, "travel": round(travel(gun), 3), "labels": list(_trial["labels"]), "where_mm": [None if w is None else [round(1000 * w[0], 1), round(1000 * w[1], 1)] for w in _trial["where"]],
            "roller_mm": [round(float(v) * 1000, 1) for v in p], "tilt_deg": round(tilt, 1), "yaw_deg": round(math.degrees(math.atan2(_rot(gun)[1, 0], _rot(gun)[0, 0])), 1),
            "gun_mm": [round(float(v) * 1000, 1) for v in gun.pos],
            "tcp_mm": [[round(float(x) * 1000, 1) for x in t] for t in (_trial["tcps"] or [])]}), flush=True)
    objs = {"gun": gun}
    dt = float(getattr(env.unwrapped, "step_dt", 0.05))
    _trial["done_s"] = _trial["done_s"] + dt if score(objs) >= PASS_SCORE else 0.0


# -- scoring -------------------------------------------------------------------------------------------------------------

def label_ok(i: int) -> bool:
    w = _trial["where"][i]
    return w is not None and math.hypot(w[0], w[1]) <= SPOT_TOL  # the one visible label's final place; cycle count is not used


def on_pad(gun) -> bool:
    p = np.asarray(gun.pos, float)
    tilt = math.degrees(math.acos(float(np.clip(abs(_rot(gun)[2, 2]), 0.0, 1.0))))
    return math.hypot(p[0] - PAD_X, p[1] - PAD_Y) <= PAD_TOL and tilt <= FLAT_DEG and p[2] <= TOP + PAD_Z + 0.004


def hands_clear(gun) -> bool:
    t = _trial["tcps"]
    if t is None or len(t) < 2:
        return False
    c = np.asarray(gun.pos, float)
    return all(float(np.linalg.norm(np.asarray(x, float) - c)) > HAND_CLEAR for x in t)


def score(objects: dict) -> float:
    """End state (scoring text, as in task.json):
    - A full trigger cycle (at least 90 % of the travel, then back below 20 %) with the labeller's roller touching a box's front face (within 1 mm of it), inside the face, and the labeller flat and square to the face within 15 degrees puts that box's label where the roller was; each box shows at most one label, and a later cycle on the same box moves its label to the new place.
    - 0.25 per box whose label centre ends within 10 mm of the printed spot's centre.
    - 0.25 when the labeller lies back on its pad (within 20 mm of its start, flat within 10 degrees, resting on it within 4 mm of the pad surface) with both grippers more than 150 mm from it, only if at least one box scored.
    Possible: 0, 0.25, 0.5, 0.75, 1.0."""
    n = sum(1 for i in range(3) if label_ok(i))
    s = S_LABEL * n
    gun = objects.get("gun")
    if n and gun is not None and on_pad(gun) and hands_clear(gun):
        s += S_PAD
    return min(s, 1.0)


def success(objects: dict) -> bool:
    return score(objects) >= PASS_SCORE


def distance(objects: dict) -> float:
    return float(3 - sum(1 for i in range(3) if label_ok(i)))


# -- scripted demo (scene constants only) ---------------------------------------------------------------------------------
HEAD_GRIP = (0.110, 0.035)  # left jaw across the head's 50 mm height, just behind the nose (155 mm from the right jaw)
TRIG_GRIP = (-0.027, -0.045)  # right jaw between the handle back and the trigger front, mid-lever
PRESS = 0.010  # roller target this far past the face (arm compliance and slip in the jaws take up most of it; the roller stops on the face; 14 mm twisted the labeller in Isaac)
SPAN0 = (TRIG_C[0] + TRIG_S[0] / 2) - (HANDLE_C[0] - HANDLE_S[0] / 2)  # handle back -> trigger front at rest (86 mm)
PAD_GAP0 = 0.0022  # finger pads' gap at a finger target of 0 (lint-roller measurement)
SQUEEZE_S = 2.0  # a slow squeeze: a fast one jerks the labeller in the jaws
AIM_DY = -0.009  # aim correction along the face: the pressed labeller yaws ~2 deg and the arms give a little


def _world(local, gx, gy, z):
    c, s = math.cos(YAW), math.sin(YAW)
    return (gx + c * local[0] - s * local[1], gy + s * local[0] + c * local[1], z)


def demo():
    """Open loop, scene constants only. The left jaw lifts the labeller by its head (jaws across the head's height) and
    carries it; the right jaw stays above it. At each box: the roller is brought square onto the printed spot and
    pressed 10 mm past the face, the right jaw comes down either side of handle + trigger, squeezes through the full travel, lets the
    trigger return and lifts away; the labeller backs off. Then it is laid back on its pad and both arms park."""
    W = pitch_demo.Waypoint
    o, c = FINGER_OPEN, FINGER_CLOSED
    rl = pitch_demo.rotation(YAW)  # left: jaws along the labeller's 'up' (across the head; yaw 0 = jaws along y)
    rr = pitch_demo.rotation(YAW - math.pi / 2)  # right: jaws along the long axis (handle back <-> trigger front)
    zg = TOP + PAD_Z + TH * 0.6
    za = zg + LIFT
    up = 0.07
    gx = FACE_X + PRESS - REACH  # labeller origin x with its roller pressed on a face (working direction = world +x)
    gy = lambda i: BOX_Y[i] + AIM_DY - (math.sin(YAW) * ROLLER_C[0] + math.cos(YAW) * ROLLER_C[1])  # noqa: E731
    seq = []  # (duration, left (pos, finger), right (pos, finger), label)

    def at(local, x, y, z):
        return _world(local, x, y, z)

    held = -(SPAN0 - PAD_GAP0) / 2 + 0.001  # right jaw resting on handle back + trigger front, trigger not pressed in
    L = lambda x, y, z: at(HEAD_GRIP, x, y, z)  # noqa: E731
    R = lambda x, y, z: at(TRIG_GRIP, x, y, z)  # noqa: E731
    lp, rp = L(PAD_X, PAD_Y, zg), R(PAD_X, PAD_Y, zg)
    seq += [(1.5, ((lp[0], lp[1], zg + up), o), ((rp[0], rp[1], zg + up), o), "over the labeller"),
            (1.0, ((lp[0], lp[1], zg + 0.040), o), ((rp[0], rp[1], zg + 0.040), o), "settle just above it: the drop is then straight down past the trigger"),
            (1.2, (lp, o), (rp, o), "jaws either side of the head and of handle + trigger"),
            (0.8, (lp, c), (rp, o), "hold the head"),
            (0.8, (lp, c), (rp, held), "close on handle + trigger"),
            (1.0, (L(PAD_X, PAD_Y, za), c), (R(PAD_X, PAD_Y, za), held), "lift it off the pad")]
    for i in range(3):
        y = gy(i)
        seq += [(1.8, (L(gx - 0.020, y, za), c), (R(gx - 0.020, y, za), held), f"in front of box {i + 1}"),
                (1.0, (L(gx, y, za), c), (R(gx, y, za), held), f"roller onto box {i + 1}'s spot"),
                (SQUEEZE_S, (L(gx, y, za), c), (R(gx, y, za), c), "squeeze through the full travel"),
                (1.6, (L(gx, y, za), c), (R(gx, y, za), held), "let the trigger return slowly (a fast release swings the labeller out of the left jaw)"),
                (0.8, (L(gx - 0.020, y, za), c), (R(gx - 0.020, y, za), held), "back off")]
    seq += [(2.0, (L(PAD_X, PAD_Y, za), c), (R(PAD_X, PAD_Y, za), held), "back over the pad"),
            (0.8, (lp, c), (rp, held), "lay it on the pad"),
            (0.6, (lp, o), (rp, o), "let go"),
            (1.0, ((lp[0], lp[1], zg + up), o), ((rp[0], rp[1], zg + up), o), "up"),
            (1.5, ((0.06, 0.40, TOP + 0.16), c), ((0.06, -0.40, TOP + 0.16), c), "park"),
            (2.0, ((0.06, 0.40, TOP + 0.16), c), ((0.06, -0.40, TOP + 0.16), c), "hold clear")]
    left = [W(d, lw[0], lw[1], lab, rot=rl) for d, lw, _, lab in seq]
    right = [W(d, rw[0], rw[1], lab, rot=rr) for d, _, rw, lab in seq]
    return {"left": left, "right": right}
