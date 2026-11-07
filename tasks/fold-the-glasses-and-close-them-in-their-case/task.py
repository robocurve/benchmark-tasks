"""On the station table: a pair of glasses standing open on its lenses and temple tips, front toward the robot and
temples pointing away, and to the robot's left a fixed hard glasses case with its lid standing open.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BLACK, LENS, CASE_C, LID_C = (0.06, 0.06, 0.07), (0.62, 0.74, 0.80), (0.12, 0.25, 0.45), (0.15, 0.30, 0.52)

# Glasses. Part frame: origin under the middle of the front's bottom edge; x toward the temples (away from the
# robot), y to the robot's left, z up. The front is 4 mm thick (x 0..4 mm), 144 mm wide and 40 mm tall.
FRONT_T, FRONT_W, FRONT_H = 0.004, 0.144, 0.040
LENS_W, LENS_H = 0.060, 0.034
TEMPLE_LEN, TEMPLE_T, TEMPLE_Z = 0.130, 0.004, (0.018, 0.024)  # length, thickness, height band
TIP_H = TEMPLE_Z[0]  # the temple tips reach down to the table
PIVOT_Y = 0.070
PIVOT_X = {"right_temple": 0.010, "left_temple": 0.016}  # folded, the left temple lies behind the right one
TZ = sum(TEMPLE_Z) / 2


def _temple(side: str):
    sy = 1.0 if side == "left_temple" else -1.0
    px = PIVOT_X[side]
    yc = sy * (PIVOT_Y - TEMPLE_T / 2)  # the arm's outer face is on the hinge line
    return parts.Hinge(
        side,
        shapes=[
            parts.box((TEMPLE_LEN, TEMPLE_T, TEMPLE_Z[1] - TEMPLE_Z[0]), at=(px + TEMPLE_LEN / 2, yc, TZ),
                      color=BLACK),
            parts.box((0.006, TEMPLE_T, TIP_H), at=(px + TEMPLE_LEN - 0.003, yc, TIP_H / 2), color=BLACK),
        ],
        pivot=(px, sy * PIVOT_Y, TZ),
        axis=(0.0, 0.0, -sy),  # positive folds the temple in behind the front
        lower=0.0,
        upper=math.pi / 2,
        mass=0.010,
        spring=0.02,
        damping=0.01,
    )


GX, GY = 0.200, -0.030  # the front's bottom middle at the start
GLASSES = parts.Part(
    "glasses",
    pos=(GX, GY, TOP),
    mass=0.020,
    shapes=[
        parts.box((FRONT_T, LENS_W, LENS_H), at=(FRONT_T / 2, 0.035, LENS_H / 2), color=LENS),
        parts.box((FRONT_T, LENS_W, LENS_H), at=(FRONT_T / 2, -0.035, LENS_H / 2), color=LENS),
        parts.box((FRONT_T, FRONT_W, FRONT_H - LENS_H), at=(FRONT_T / 2, 0.0, (FRONT_H + LENS_H) / 2), color=BLACK),
        parts.box((FRONT_T, 0.007, LENS_H), at=(FRONT_T / 2, 0.0685, LENS_H / 2 + 0.0), color=BLACK),
        parts.box((FRONT_T, 0.007, LENS_H), at=(FRONT_T / 2, -0.0685, LENS_H / 2 + 0.0), color=BLACK),
        # Hinge blocks joining each temple to the front.
        parts.box((PIVOT_X["left_temple"] - FRONT_T, 0.006, 0.010), at=((PIVOT_X["left_temple"] + FRONT_T) / 2,
                                                                       0.069, TZ), color=BLACK),
        parts.box((PIVOT_X["right_temple"] - FRONT_T, 0.006, 0.010), at=((PIVOT_X["right_temple"] + FRONT_T) / 2,
                                                                        -0.069, TZ), color=BLACK),
    ],
    joints=[_temple("right_temple"), _temple("left_temple")],
)

# Case. Part frame: origin under the middle of the case; the lid is hinged along the top of its far wall.
CX, CY = 0.240, 0.200
IN_X, IN_Y, WALL, FLOOR, RIM = 0.050, 0.156, 0.004, 0.004, 0.048  # inner depth and length, wall, floor, rim height
OUT_X, OUT_Y = IN_X + 2 * WALL, IN_Y + 2 * WALL
LID_T = 0.004
LID_PIVOT = (OUT_X / 2, 0.0, RIM)
CASE = parts.Part(
    "case",
    pos=(CX, CY, TOP),
    shapes=[
        parts.box((OUT_X, OUT_Y, FLOOR), at=(0.0, 0.0, FLOOR / 2), color=CASE_C),
        parts.box((WALL, OUT_Y, RIM - FLOOR), at=(-OUT_X / 2 + WALL / 2, 0.0, (RIM + FLOOR) / 2), color=CASE_C),
        parts.box((WALL, OUT_Y, RIM - FLOOR), at=(OUT_X / 2 - WALL / 2, 0.0, (RIM + FLOOR) / 2), color=CASE_C),
        parts.box((IN_X, WALL, RIM - FLOOR), at=(0.0, OUT_Y / 2 - WALL / 2, (RIM + FLOOR) / 2), color=CASE_C),
        parts.box((IN_X, WALL, RIM - FLOOR), at=(0.0, -OUT_Y / 2 + WALL / 2, (RIM + FLOOR) / 2), color=CASE_C),
    ],
    joints=[parts.Hinge(
        "lid",
        # Written standing open (upright above the far wall); closing turns it down over the case.
        shapes=[parts.box((LID_T, OUT_Y, OUT_X), at=(OUT_X / 2 + LID_T / 2, 0.0, RIM + OUT_X / 2), color=LID_C)],
        pivot=LID_PIVOT,
        axis=(0.0, 1.0, 0.0),  # negative closes: the lid's free edge swings toward the robot and down
        lower=-math.pi / 2,
        upper=0.0,
        mass=0.020,
        spring=0.06,
        damping=0.01,
    )],
)

OBJECTS = [GLASSES, CASE]

EPS = 1e-6
FOLDED = math.pi / 2 - math.radians(15.0)  # a temple folded: within 15 deg of fully folded
LID_SHUT = -math.pi / 2 + math.radians(3.0)  # the lid closed: within 3 deg of shut
SNAP_AT = -math.pi / 4  # the lid's spring hinge holds it open until it is closed past halfway, then snaps it shut
TEMPLES = ("right_temple", "left_temple")
DETENT_IN, DETENT_OUT = math.pi / 2 - math.radians(8.0), math.pi / 2 - math.radians(20.0)
ON = 0.001  # resting on a surface: lowest point within 1 mm of it
FLOOR_Z = TOP + FLOOR
FELL = 0.005

# Hinge behaviour. A temple's hinge turns freely (its drive target follows it) until it is folded to within 8 deg,
# then a light detent holds it folded until it is pushed back open past 20 deg. The case lid's spring hinge holds it
# open until it is closed past halfway, then pulls it shut.
_held = {t: False for t in TEMPLES}
_prev = {t: 0.0 for t in TEMPLES}


@parts.on_reset
def _reset(sim):
    for t in TEMPLES:
        _held[t], _prev[t] = False, 0.0
        sim.set_target("glasses", t, 0.0)
    sim.set_target("case", "lid", 0.0)


@parts.on_step
def _hinges(sim):
    st = sim.states()
    for t in TEMPLES:
        q = float(st["glasses"].joints[t])
        if not _held[t] and q >= DETENT_IN:
            _held[t] = True
        elif _held[t] and q < DETENT_OUT:
            _held[t] = False
        sim.set_target("glasses", t, math.pi / 2 if _held[t] else q + 0.5 * (q - _prev[t]))
        _prev[t] = q
    lid = float(st["case"].joints["lid"])
    sim.set_target("case", "lid", -math.pi / 2 if lid < SNAP_AT else 0.0)


def _folded(g) -> bool:
    return all(g.joints[t] >= FOLDED - EPS for t in TEMPLES)


UPRIGHT = math.radians(20.0)  # standing on its bottom edge: the front within 20 deg of vertical
TIPPED = math.radians(45.0)  # knocked over: the front more than 45 deg from vertical while down on the table
HANDS_OFF = 0.005  # released: every gripper's fingers at least 5 mm clear of the glasses


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _lean(g) -> float:
    """How far the front's up direction leans off vertical (rad)."""
    return math.acos(float(np.clip(_rot(g.quat_xyzw)[2, 2], -1.0, 1.0)))


def _over_case(g) -> bool:
    c = (np.asarray(g.low) + np.asarray(g.high)) / 2
    return bool(abs(c[0] - CX) < IN_X / 2 and abs(c[1] - CY) < IN_Y / 2)


def _inside(g) -> bool:
    """The glasses' whole outline within the case's inner walls, resting on its floor."""
    lo_x, hi_x, lo_y, hi_y = CX - IN_X / 2, CX + IN_X / 2, CY - IN_Y / 2, CY + IN_Y / 2
    return bool(g.low[0] >= lo_x - EPS and g.high[0] <= hi_x + EPS and g.low[1] >= lo_y - EPS
                and g.high[1] <= hi_y + EPS and abs(g.low[2] - FLOOR_Z) <= ON + EPS)


def _standing(g) -> bool:
    """Standing on its bottom edge (front within 20 deg of vertical), its lowest point within 1 mm of the table, or
    of the case floor with its centre over the case."""
    on = abs(g.low[2] - TOP) <= ON + EPS or (_over_case(g) and abs(g.low[2] - FLOOR_Z) <= ON + EPS)
    return bool(on and _lean(g) <= UPRIGHT + EPS)


def _lid_shut(objects) -> bool:
    return bool(objects["case"].joints["lid"] <= LID_SHUT + EPS)


# Grippers. The scorer is given only the objects, so an on_step callback reads both arms' joints each control step
# and keeps each gripper's tip point (between the finger tips, from the arm's own kinematics) and half opening. The
# glasses are released when every tip point is either more than 5 mm above the glasses' top, or farther than its half
# opening plus 10 mm (a finger's thickness) plus 5 mm from the glasses' outline sideways. Offline there are no grippers.
BASES = {"left_robot": (0.0, 0.305, 0.75), "right_robot": (0.0, -0.305, 0.75)}
FINGER_T = 0.010
_GRIP: dict = {}


@parts.on_step
def _read_grippers(sim) -> None:
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, PAD_TIP_PAST_TCP

    base = sim.env.unwrapped
    for name, bpos in BASES.items():
        robot = base.scene[name]
        arm_ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
        fin_ids = robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        q = robot.data.joint_pos.torch[0].detach().cpu().numpy().astype(float)
        pos, rot = ik.tcp_pose(q[arm_ids], bpos)
        tip = pos + rot @ np.array([0.0, 0.0, PAD_TIP_PAST_TCP])
        _GRIP[name] = (tip, float(np.max(np.abs(q[fin_ids]))))


def _released(g) -> bool:
    lo, hi = np.asarray(g.low, float), np.asarray(g.high, float)
    for tip, half in _GRIP.values():
        if tip[2] > hi[2] + HANDS_OFF:
            continue
        side = np.maximum(np.maximum(lo[:2] - tip[:2], tip[:2] - hi[:2]), 0.0)
        if np.linalg.norm(side) <= half + FINGER_T + HANDS_OFF:
            return False
    return True


# Whole-trial record: were the glasses ever knocked over (front more than 45 deg off vertical with their lowest point
# within 5 mm of the table top, outside the case)? Cleared only by a scene reset: the glasses back within 0.5 mm of
# their start and at rest, after a jump of over 20 mm in one control step.
_START = np.array([GX, GY, TOP])
_record = {"tipped": False, "prev": None}


def _update(g) -> None:
    pos = np.asarray(g.pos, float)
    prev = _record["prev"]
    if prev is not None and np.linalg.norm(pos - _START) < 0.0005 and g.resting() and np.linalg.norm(pos - prev) > 0.020:
        _record["tipped"] = False
    _record["prev"] = pos
    if _lean(g) > TIPPED and g.low[2] <= TOP + FELL and not _over_case(g):
        _record["tipped"] = True


def _done(objects) -> bool:
    g = objects["glasses"]
    return bool(_folded(g) and _inside(g) and _lid_shut(objects) and g.resting() and _released(g))


def success(objects):
    """Both temples folded, the glasses inside the case on its floor, released and still, the lid shut, and the
    glasses never knocked over on the table."""
    _update(objects["glasses"])
    return bool(not _record["tipped"] and _done(objects))


def distance(objects):
    g = objects["glasses"]
    return float(np.hypot(g.pos[0] - CX, g.pos[1] - CY))


def score(objects):
    """1.0 on success; 0.5 folded, inside the case, released and still, lid not shut; 0.25 folded, released and
    still, standing on its bottom edge on the table or the case floor, not inside the case; 0 otherwise, and 0 for
    the whole trial once the glasses have been knocked over on the table."""
    g = objects["glasses"]
    _update(g)
    if _record["tipped"] or not g.resting() or not _released(g) or not _folded(g):
        return 0.0
    if _inside(g):
        return 1.0 if _lid_shut(objects) else 0.5
    return 0.25 if _standing(g) else 0.0


# --- demo ---------------------------------------------------------------------------------------------------------
TIP = 0.013
PAD_HALF = 0.010  # closed fingers: 10 mm either side of the TCP across the closing direction
GRIP_DY = 0.050  # the left arm holds the front over the left lens, 50 mm left of its middle
GRIP_TIPS = 0.030  # finger tips 30 mm above the glasses' bottom: above the temples (24 mm)
PUSH_U = 0.075  # the right arm pushes each temple 75 mm out from its hinge
PUSH_TIPS = 0.012


LEFT_ACROSS = False  # the left arm pushes with the fingers' 40 mm side (yaw along the temple)


def _paddle(side, deg, gx=GX, gy=GY, push=0.003):
    """TCP pushing a temple folded by deg: against its outer face, PUSH_U out from the hinge."""
    sy = 1.0 if side == "left_temple" else -1.0
    phi = -sy * math.radians(deg)  # world heading of the temple (the left temple turns clockwise)
    d = np.array([math.cos(phi), math.sin(phi)])
    n = np.array([-math.sin(phi), math.cos(phi)]) * sy  # outward normal
    across = side == "left_temple" and LEFT_ACROSS
    half, yaw = (PAD_HALF * 2, phi + math.pi / 2) if across else (PAD_HALF, phi)
    p = np.array([gx + PIVOT_X[side], gy + sy * PIVOT_Y]) + PUSH_U * d + (half - push) * n
    return (float(p[0]), float(p[1]), TOP + PUSH_TIPS + TIP), yaw


def _fold(side, label_arm):
    """Waypoints that fold one temple: beside it, then along its arc past folded, then off and up."""
    start, yaw0 = _paddle(side, 0, push=-0.010)
    out = [
        Waypoint(1.5, (start[0], start[1], start[2] + 0.04), FINGER_CLOSED, f"over the {side}", yaw0),
        Waypoint(1.0, start, FINGER_CLOSED, f"beside the {side}", yaw0),
    ]
    for deg in (0, 15, 30, 45, 60, 75, 90, 96):
        p, yaw = _paddle(side, deg)
        out.append(Waypoint(0.6 if deg == 0 else 0.5, p, FINGER_CLOSED, f"fold {side} {deg}", yaw))
    p, yaw = _paddle(side, 96, push=-0.012)
    out += [
        Waypoint(0.6, p, FINGER_CLOSED, f"off the {side}", yaw),
        Waypoint(0.8, (p[0] + 0.02, p[1], p[2] + 0.05), FINGER_CLOSED, f"{label_arm} up", yaw),
    ]
    return out


def demo():
    """Right arm: push the right temple through its arc until it folds in behind the front, then move aside. Left
    arm: fold the left temple the same way (it folds behind the right one), grip the front over the left lens, lift
    the folded glasses into the case upright and let go, then reach behind the open lid and pull it past halfway so
    it snaps shut."""
    hy = -math.pi / 2  # fingers close along x, across the 4 mm front
    right = [Waypoint(1.5, (0.25, -0.22, TOP + 0.10), FINGER_CLOSED, "aside")] + _fold("right_temple", "right")
    right.append(Waypoint(1.5, (0.12, -0.28, TOP + 0.12), FINGER_CLOSED, "park"))
    t_right_done = sum(w.duration for w in right)
    left = [Waypoint(t_right_done, (0.12, 0.28, TOP + 0.12), FINGER_CLOSED, "wait aside"),
            Waypoint(1.0, (0.18, 0.06, TOP + 0.10), FINGER_CLOSED, "in front of the case")]
    left += _fold("left_temple", "left")

    gxy = (GX + FRONT_T / 2, GY + GRIP_DY)
    grip_z = TOP + GRIP_TIPS + TIP
    left += [
        Waypoint(1.5, (*gxy, grip_z + 0.05), FINGER_OPEN, "over the front", hy),
        Waypoint(1.2, (*gxy, grip_z), FINGER_OPEN, "down", hy),
        Waypoint(0.8, (*gxy, grip_z), FINGER_CLOSED, "grip the front", hy),
    ]
    # Into the case: the front's bottom middle to (FX, CY), 4 mm above the floor.
    fx = CX - IN_X / 2 + 0.015
    put = (fx + FRONT_T / 2, CY + GRIP_DY)
    put_z = FLOOR_Z + 0.004 + GRIP_TIPS + TIP
    carry_z = TOP + RIM + 0.03 + GRIP_TIPS + TIP
    left += [
        Waypoint(1.2, (*gxy, carry_z), FINGER_CLOSED, "lift", hy),
        Waypoint(2.0, (*put, carry_z), FINGER_CLOSED, "over the case", hy),
        Waypoint(1.5, (*put, put_z), FINGER_CLOSED, "lower in", hy),
        Waypoint(0.6, (*put, put_z), FINGER_OPEN, "let go", hy),
        Waypoint(1.0, (*put, carry_z + 0.02), FINGER_OPEN, "up", hy),
    ]
    # Lid: pull its top edge toward the robot along its arc to 55 deg closed.
    px, pz = CX + LID_PIVOT[0], TOP + RIM
    r = OUT_X - 0.006  # contact 6 mm below the lid's free edge

    def on_lid(deg, push=0.003):
        a = math.radians(deg)
        back = LID_T + PAD_HALF - push  # the TCP's offset from the hinge line, behind the lid
        x = px + back * math.cos(a) - r * math.sin(a)
        z = pz + back * math.sin(a) + r * math.cos(a)
        return (x, CY, z + TIP - 0.006)  # finger tips 6 mm below the contact point

    s = on_lid(0, push=-0.010)
    left += [
        Waypoint(1.5, (s[0], CY, s[2] + 0.03), FINGER_CLOSED, "behind the lid", hy),
        Waypoint(1.0, s, FINGER_CLOSED, "down behind it", hy),
    ]
    for deg in (0, 15, 30, 45, 55):
        left.append(Waypoint(0.6, on_lid(deg), FINGER_CLOSED, f"pull lid {deg}", hy))
    e = on_lid(55)
    left += [Waypoint(1.0, (e[0], CY, e[2] + 0.06), FINGER_CLOSED, "up away", hy)]
    return {"left": left, "right": right}
