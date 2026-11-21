"""A line cook's pan toss. A true-size 8 in (203 mm) aluminium saute pan with sloped sides and a sleeved handle sits on a
cold burner in front of the left arm, its handle pointing at that arm's base. In it lie twelve 13 mm dice of par-cooked
potato, each with one browned face, browned face down. The robot has to take the pan by its handle, turn the dice over by
moving the pan alone (no utensil is on the table and no part of the robot may touch the food), and set the pan back down
level on the burner.

Run it:  inspect-robots run --embodiment yam_pan -E task=task.py -E object_jitter=0 --policy yam_scripted --no-prompt \\
           --instruction "Toss the diced potatoes in the saute pan to turn them over, without touching the food, then set the pan back on the burner." --scorer task_score

`yam_pan` (pan_toss/embodiment.py) spawns the pan, dice and burner, reports each gripper's fingertip midpoint, and reads
robot-on-food contact from the physics engine.

The pan, each die and both grippers are rigid bodies in real contact; nothing is attached or scripted. What is scored:
- Turned: a die at rest in the pan whose browned face is no longer underneath: the face's outward normal points no lower
  than 30 degrees below horizontal (its vertical component is -0.5 or more). A die lying flat has that face down (-1),
  on a side (0) or up (+1), so the test is far from every resting pose.
- Lost: a die that touched the table, the burner or the floor at any time (latched), or that is not inside the pan at
  the end.
- Food touched: any contact force between any robot link and any die, at any time (latched). The score is then 0.
- Tossed: at some control step, at least TOSS_K dice are in the air over the pan with their centres above the rim's plane
  (latched). Without it the score is 0, so sliding or shaking the dice about earns nothing however many end turned.
"""

import json
import math
import os

import numpy as np

from pan_toss import pan as P

try:
    from yam_arena.task_file import TABLE_TOP, Box
except ImportError:  # the scoring criteria and their tests need no simulator packages
    from dataclasses import dataclass

    TABLE_TOP = {"station": 0.7475}

    @dataclass(frozen=True)
    class Box:
        name: str
        pos: tuple
        size: tuple = (0.04, 0.04, 0.04)
        mass: float | None = None
        yaw: float = 0.0


SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Toss the diced potatoes in the saute pan to turn them over, without touching the food, then set the pan back on the burner."
VARIANT = os.environ.get("PAN_DEMO", "")  # "", "shake", "poke", "touch", "none": the demo and its shortcut variants

# -- layout (world: x forward from the arms, y left, z up; the left arm's base axis is at x 0, y 0.305) ----------------
BASE = (0.0, 0.305, 0.75)
HEADING = math.radians(-25.0)  # the pan's +x in the world: straight away from the left arm's base
REACH = 0.630  # the pan's centre from the base axis
PAN_XY = (BASE[0] + REACH * math.cos(HEADING), BASE[1] + REACH * math.sin(HEADING))
BURNER_TOP = TOP + P.BURNER["h"]
PAN_Z = BURNER_TOP + P.T_WALL  # the cooking surface with the pan on the burner
DICE = tuple(f"die_{i:02d}" for i in range(P.N_DICE))


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))


def pan_to_world(p, pan_xyz=None, heading=HEADING):
    o = np.array((*PAN_XY, PAN_Z)) if pan_xyz is None else np.asarray(pan_xyz, float)
    return o + _rz(heading) @ np.asarray(p, float)


OBJECTS = [
    Box("burner", pos=(*PAN_XY, BURNER_TOP), size=(0.01, 0.01, 0.01), yaw=HEADING),
    Box("pan", pos=(*PAN_XY, PAN_Z + 0.0005), size=(0.01, 0.01, 0.01), mass=P.PAN_MASS, yaw=HEADING),
] + [
    Box(name, pos=tuple(pan_to_world((x, y, P.DIE / 2 + 0.001))), size=(P.DIE,) * 3, mass=P.DIE_MASS, yaw=HEADING + math.radians(yaw))
    for name, (x, y, yaw) in zip(DICE, P.DICE_START)
]

# -- criteria -------------------------------------------------------------------------------------------------------------
TURNED_NZ = -0.5  # the browned face's normal: vertical component at least this (no lower than 30 degrees below horizontal)
HALF_DIAG = P.DIE * math.sqrt(3.0) / 2  # a die's centre this close to a surface must be touching it
FULL_NET = 7  # dice turned, less dice lost, for the top band: more than half of the twelve
BANDS = ((7, 0.6), (5, 0.4), (3, 0.2))  # (net dice turned at least, credit)
HOME_BONUS = 0.2  # top band only: the pan back on the burner, level, let go
HOME_XY, HOME_Z, HOME_TILT = 0.030, 0.005, math.radians(5.0)
CLEAR = 0.08  # each fingertip midpoint this far from the pan and its handle
LIN, ANG, STILL_S = 0.02, 1.0, 1.0  # at rest: under 2 cm/s and 1 rad/s, for the last second
TOSS_K = 3  # dice at once over the pan with their centres above the rim's plane: a toss, seen in the side video
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _fresh():
    return {"touched": False, "lost": np.zeros(P.N_DICE, bool), "still": 0.0, "aloft": 0}


_trial = _fresh()


def trial():
    return _trial


def reset_trial():
    """Forget the trial: the embodiment calls this at every reset. Nothing a robot does clears a latch."""
    _trial.clear()
    _trial.update(_fresh())


def _rot(q):
    x, y, z, w = q
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)), (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def in_pan_frame(objects, point):
    pan = objects["pan"]
    return _rot(pan.quat_xyzw).T @ (np.asarray(point, float) - np.asarray(pan.pos, float))


def tilt_deg(pan):
    """How far the pan's cooking surface is from level."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(_rot(pan.quat_xyzw)[2, 2])))))


def inside(objects, name):
    """The die's centre is within the pan's bowl: above the cooking surface, below the rim, inside the wall."""
    x, y, z = in_pan_frame(objects, objects[name].pos)
    return bool(0.0 < z < P.DEPTH and math.hypot(x, y) < P.wall_radius(z))


def brown_nz(die):
    """The vertical component of the browned face's outward normal: -1 face down, 0 on a side, +1 face up."""
    return float((_rot(die.quat_xyzw) @ P.BROWN_LOCAL)[2])


def above_rim(objects, name):
    """The die is in the air over the pan: its centre above the rim's plane and within the rim's circle, in the pan's frame."""
    x, y, z = in_pan_frame(objects, objects[name].pos)
    return bool(z > P.DEPTH and math.hypot(x, y) < P.R_RIM)


def aloft(objects):
    return sum(above_rim(objects, n) for n in DICE)


def turned(objects, name):
    return brown_nz(objects[name]) >= TURNED_NZ


def spilled(objects, name):
    """The die is out of the pan and on the table, the burner or the floor: its centre is closer to one of them than a
    cube's half diagonal, so it must be touching."""
    p = np.asarray(objects[name].pos, float)
    if inside(objects, name):
        return False
    if p[2] < TOP + HALF_DIAG:  # on the table, or off it and falling
        return True
    b = _rz(HEADING).T @ (p - np.array((*PAN_XY, BURNER_TOP)))
    half = P.BURNER["size"] / 2 + HALF_DIAG
    if abs(b[0]) < half and abs(b[1]) < half and b[2] < HALF_DIAG:
        x, y, z = in_pan_frame(objects, p)  # not if it is in the air above a pan standing on the burner
        return not (z > 0.0 and math.hypot(x, y) < P.R_RIM)
    return False


def note_step(objects, dt, touched=False):
    """Advance the trial's record by one control step: the latches, and how long everything has been at rest."""
    if touched:
        _trial["touched"] = True
    _trial["aloft"] = max(_trial["aloft"], aloft(objects))
    for i, name in enumerate(DICE):
        if spilled(objects, name):
            _trial["lost"][i] = True
    moving = [objects["pan"]] + [objects[n] for n in DICE]
    rest = all(np.linalg.norm(o.lin_vel) < LIN and np.linalg.norm(o.ang_vel) < ANG for o in moving)
    _trial["still"] = _trial["still"] + dt if rest else 0.0


def counts(objects):
    """turned: dice in the pan, never spilled, browned face no longer down. lost: dice spilled at any time or not in the
    pan now. inside: dice in the pan now."""
    ins = [inside(objects, n) for n in DICE]
    lost = [bool(_trial["lost"][i]) or not ins[i] for i in range(P.N_DICE)]
    t = sum(1 for i, n in enumerate(DICE) if not lost[i] and turned(objects, n))
    return {"turned": t, "lost": sum(lost), "inside": sum(ins), "net": max(0, t - sum(lost))}


def pan_home(objects):
    """The pan stands on the burner: centre within 3 cm of the burner's, cooking surface within 5 mm of its resting height
    and within 5 degrees of level."""
    pan = objects["pan"]
    p = np.asarray(pan.pos, float)
    return bool(math.hypot(p[0] - PAN_XY[0], p[1] - PAN_XY[1]) <= HOME_XY and abs(p[2] - PAN_Z) <= HOME_Z and tilt_deg(pan) <= math.degrees(HOME_TILT))


def _hand_gap(objects, hand):
    """The fingertip midpoint's distance from the pan: the nearer of the handle's centre line and the bowl (a cylinder of the
    rim's radius from the underside to the rim)."""
    x, y, z = in_pan_frame(objects, objects[hand].pos)
    a, b = P.HANDLE_ROOT, P.HANDLE_END
    d = b - a
    q = np.array((x, y, z))
    s = min(max(float(np.dot(q - a, d) / np.dot(d, d)), 0.0), 1.0)
    handle = float(np.linalg.norm(q - (a + s * d)))
    dr = max(0.0, math.hypot(x, y) - (P.R_RIM + P.T_WALL))
    dz = max(0.0, -P.T_WALL - z, z - P.DEPTH)
    return min(handle, math.hypot(dr, dz))


def hands_clear(objects):
    return all(_hand_gap(objects, h) >= CLEAR for h in ("left_tcp", "right_tcp"))


def band(net):
    for least, credit in BANDS:
        if net >= least:
            return credit
    return 0.0


def tossed():
    return _trial["aloft"] >= TOSS_K


def score(objects):
    """0 if the robot touched the food, or if no toss was seen (TOSS_K dice at once above the rim). Otherwise the band for net = dice turned - dice lost: under 3: 0; 3-4: 0.2;
    5-6: 0.4; 7 or more: 0.6. In the top band only, 0.8 with the pan back on the burner, level, let go and at rest, and
    1.0 when in addition no die is lost."""
    if _trial["touched"] or not tossed():
        return 0.0
    c = counts(objects)
    credit = band(c["net"])
    if c["net"] < FULL_NET:
        return credit
    if not (pan_home(objects) and hands_clear(objects) and _trial["still"] >= STILL_S):
        return credit
    return 1.0 if c["lost"] == 0 else credit + HOME_BONUS


def success(objects):
    return score(objects) >= 1.0


def distance(objects):
    """Dice still to turn for the top band."""
    return float(max(0, FULL_NET - counts(objects)["net"]))


# -- the open-loop demo: built only from the fixed starting layout above -----------------------------------------------
DT = 0.05  # one target per control step at 20 Hz
READY = (0.0, 1.0, 1.0, -0.6, 0.0, 0.0)
LEVEL = P.HANDLE_RISE  # with the gripper along the handle, this gripper angle below horizontal holds the pan level
GRASP_D = REACH + P.GRASP[0]  # the grasp point's distance from the base axis, pan on the burner
GRASP_Z = PAN_Z + P.GRASP[2]
BACK_OFF = 0.100  # the approach starts this far back along the handle's line
CARRY_UP = 0.080  # the pan is tossed this far above the burner
FOOD = P.N_DICE * P.DIE_MASS
_m = P.PAN_MASS + FOOD
_com = (P.PAN_MASS * P.PAN_COM + FOOD * np.array((0.0, 0.0, P.DIE / 2))) / _m - P.GRASP
PAYLOAD = (_m, (float(_com[0]), float(_com[2])), LEVEL)
# One toss, as timed joint-target offsets (s, shoulder, elbow, wrist in rad) from the carry pose: tip the pan nose-down so
# the dice gather at the far wall (0.5-0.9 s), swing the elbow and wrist up (the targets move at the harness's per-step limit), then come back level
# under the falling dice and let them settle.
TOSS = ((0.0, 0.0, 0.0, 0.0), (0.5, 0.0, 0.0, -0.45), (0.9, 0.0, 0.0, -0.45), (0.95, 0.0, 0.30, 0.30), (1.05, 0.0, 0.30, 0.30),
        (1.35, 0.0, 0.0, 0.0), (2.4, 0.0, 0.0, 0.0))
TOSSES = 5
# The plain-shake shortcut: the pan slid hard back and forth, level, with no flick (the dice slide and hit the walls).
SHAKE = ((0.0, 0.0, 0.0, 0.0), (0.15, 0.2, 0.0, 0.0), (0.3, -0.2, 0.0, 0.0), (0.45, 0.2, 0.0, 0.0), (0.6, -0.2, 0.0, 0.0), (0.75, 0.2, 0.0, 0.0),
         (0.9, -0.2, 0.0, 0.0), (1.05, 0.2, 0.0, 0.0), (1.2, -0.2, 0.0, 0.0), (1.5, 0.0, 0.0, 0.0), (2.4, 0.0, 0.0, 0.0))


def _tosses():
    """The demo's tosses, each a tuple of knots. PAN_TOSS (dev) holds a JSON list of tosses, or names a file with one."""
    override = os.environ.get("PAN_TOSS")
    if override:
        text = open(override).read() if os.path.exists(override) else override
        return [tuple(tuple(k) for k in toss) for toss in json.loads(text)]
    return [SHAKE if VARIANT == "shake" else TOSS] * TOSSES


def _poke(L, b, pushes):
    """Fingers side by side, pointing 50 degrees down: into the pan, push across the cooking surface, and out."""
    b.p = L.Planar(BASE, HEADING, roll=0.0)
    b.q3 = np.array((1.9, 1.0, 0.3))
    steep, low, near, far = math.radians(50), PAN_Z + 0.018, REACH - 0.060, REACH + 0.010
    b.joint_move(3.0, (near, PAN_Z + 0.10, steep), "over the pan")
    b.move(1.5, (near, low, steep), "fingertips into the pan")
    for _ in range(pushes):
        b.move(1.2, (far, low, steep), "push the dice")
        b.move(1.2, (near, low, steep), "drag back")
    b.move(1.5, (near, PAN_Z + 0.10, steep), "out of the pan")
    b.to_joints(2.5, READY, "away")


def demo():
    """Left arm: take the handle, lift, toss, set the pan back and let go. The right arm stays at its ready pose."""
    from pan_toss import plan as L

    L.install()
    planar = L.Planar(BASE, HEADING)
    b = L.Builder(planar, DT, READY, payload=PAYLOAD)
    if VARIANT == "none":
        b.to_joints(2.0, READY, "wait")
        return {"left": [b.plan()]}
    if VARIANT == "poke":  # a shortcut: reach into the pan and push the dice about with the closed fingertips
        b.grip_now = 0.0
        _poke(L, b, pushes=2)
        return {"left": [b.plan()]}
    if VARIANT == "touch":  # one push at the dice with the fingertips, then the full demo: the touch zeroes the trial
        b.grip_now = 0.0
        _poke(L, b, pushes=1)
        b.p, b.q3, b.grip_now = planar, np.array(READY[1:4], float), 1.0
    pre = (GRASP_D - BACK_OFF * math.cos(LEVEL), GRASP_Z + BACK_OFF * math.sin(LEVEL), LEVEL)
    grasp = (GRASP_D, GRASP_Z, LEVEL)
    carry = (GRASP_D, GRASP_Z + CARRY_UP, LEVEL)
    b.joint_move(3.0, pre, "behind the handle")
    b.hold(0.3, "settle")
    b.move(1.5, grasp, "fingers over and under the sleeve")
    b.hold(1.2, "close", grip=0.0)
    b.hold(0.3, "grip")
    b.loaded = True
    b.move(1.5, carry, "lift the pan")
    b.hold(0.5, "steady")
    for k, knots in enumerate(_tosses()):
        b.joint_knots(knots, f"toss {k + 1}")
    b.move(1.5, (grasp[0], grasp[1] + 0.004, LEVEL), "back onto the burner")
    b.loaded = False
    b.hold(0.5, "rest it")
    b.hold(1.2, "let go", grip=1.0)
    b.move(1.5, pre, "draw back off the handle")
    b.to_joints(2.5, READY, "away")
    return {"left": [b.plan()]}


if __name__ == "__main__":
    # Offline check, no GPU: every variant's plan is within the joint limits; prints its length.
    from yam_arena import ik

    for variant in ("", "shake", "poke", "touch", "none"):
        VARIANT = variant
        p = demo()["left"][0]
        assert np.all(p.q >= ik.LOWER - 1e-9) and np.all(p.q <= ik.UPPER + 1e-9), variant
        step = np.abs(np.diff(p.q, axis=0)).max(0) / DT
        print(f"{variant or 'demo':6s}: {len(p.q)} steps ({p.duration:.1f} s), fastest joint targets {np.round(step, 1).tolist()} rad/s")
