"""A car battery (BCI group 24 size) stands on the bench, its posts along the front edge, the positive one at the
right-hand end. The positive cable's clamp has been set back over the positive post, but it rides high on the post's
taper and a little crooked, and its pinch-bolt nut is loose. A 13 mm combination wrench stands in a holder on a tool
stand in front of the battery's left half.

The positive post is a lead cone, 19.5 mm across at its base and 17.5 mm at its top, 18 mm tall, on a 4 mm boss. The
clamp is a lead collar (16 mm tall, 20.1 mm bore) with two ears and an M8 pinch bolt on its front side (the nut on
the left) and the cable lug on its back side. The collar only grips once it is down on the boss: tightened anywhere
higher, it locks high and loose. Turning the nut pushes on the collar, so a collar that nothing holds turns on the
post or rides up.

Run it:  inspect-robots run --embodiment yam_arena -E task=car_battery_clamp.py \\
           --instruction "Reconnect the positive battery cable to the battery." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (hooks yam_arena so the multi-shape bodies and joints are simulated)
from bodykit import Body, BoxShape, CylShape, plate  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"


def _quat_axis(axis, angle: float) -> tuple[float, float, float, float]:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    s = math.sin(angle / 2)
    return (float(a[0] * s), float(a[1] * s), float(a[2] * s), math.cos(angle / 2))


def _quat_mul(q1, q2) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def _rot(axis: str, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


# -- the battery: BCI group 24, 260 x 173 x 225 mm, held in its tray (fixed) ----------------------------------------
BATT = (0.173, 0.260, 0.225)  # x (front to back), y (end to end), z
BATT_XY = (0.300, 0.000)
BATT_TOP = TOP + BATT[2]
FRONT_X = BATT_XY[0] - BATT[0] / 2  # the battery's front face, toward the robot
POS_XY = (FRONT_X + 0.022, BATT_XY[1] - 0.085)  # positive post: right-hand end, 22 mm in from the front face
NEG_XY = (FRONT_X + 0.022, BATT_XY[1] + 0.085)
BOSS_R, BOSS_H = 0.014, 0.004  # the moulded lead boss each post stands on
BOSS_TOP = BATT_TOP + BOSS_H
POST_H = 0.018
POS_D = (0.0195, 0.0175)  # SAE positive post: diameter at the base, at the top
NEG_D = (0.0179, 0.0159)  # SAE negative post: smaller, so a positive clamp drops past its seat
STEPS = 4  # the taper as four stacked cylinders


BATT_CZ = TOP + BATT[2] / 2  # the battery body's origin: the middle of the case
POS_LOCAL = (POS_XY[0] - BATT_XY[0], POS_XY[1] - BATT_XY[1], BOSS_TOP - BATT_CZ)  # the positive boss top, battery frame
NEG_LOCAL = (NEG_XY[0] - BATT_XY[0], NEG_XY[1] - BATT_XY[1], BOSS_TOP - BATT_CZ)


def _post_shapes(at, d_base: float, d_top: float, color, mark) -> list:
    x, y, z = at
    shapes = [CylShape((x, y, z - BOSS_H / 2), BOSS_R, BOSS_H, color=color)]
    for i in range(STEPS):
        d = d_base + (d_top - d_base) * (i + 0.5) / STEPS
        shapes.append(CylShape((x, y, z + POST_H * (i + 0.5) / STEPS), d / 2, POST_H / STEPS, color=color))
    shapes.append(CylShape((x, y, z + POST_H + 0.0003), d_top / 2 - 0.002, 0.0006, color=mark, collide=False))
    return shapes


LEAD = (0.55, 0.55, 0.53)
RED, BLACK = (0.78, 0.10, 0.08), (0.08, 0.08, 0.08)
CASE = (0.16, 0.17, 0.19)
BATT_KG = 18.0
_LID = BATT[2] / 2 + 0.0003
BATTERY = Body.build(
    "battery", pos=(*BATT_XY, BATT_CZ), mass=BATT_KG, color=CASE, friction=(1.1, 0.9),
    shapes=[BoxShape((0.0, 0.0, 0.0), BATT, color=CASE),
            # the + and - marks moulded into the lid behind each post (paint, nothing collides)
            BoxShape((POS_LOCAL[0] + 0.032, POS_LOCAL[1], _LID), (0.016, 0.004, 0.0006), color=RED, collide=False),
            BoxShape((POS_LOCAL[0] + 0.032, POS_LOCAL[1], _LID), (0.004, 0.016, 0.0006), color=RED, collide=False),
            BoxShape((NEG_LOCAL[0] + 0.032, NEG_LOCAL[1], _LID), (0.016, 0.004, 0.0006), color=(0.85, 0.85, 0.85),
                     collide=False),
            *_post_shapes(POS_LOCAL, *POS_D, LEAD, RED), *_post_shapes(NEG_LOCAL, *NEG_D, LEAD, BLACK)])

# -- the clamp: lead collar, ears, M8 pinch bolt (front, -x, nut on the left), cable lug (back, +x) -----------------------------------
H = 0.016  # collar height
BORE_R = 0.01005  # 20.1 mm bore: 0.3 mm a side over the post's base, 1.3 mm over its top
WALL = 0.0055
R_OUT = BORE_R + WALL
SIDES = 12
EAR = (0.012, 0.006, 0.014)  # x (out from the collar), y (thickness), z
EAR_Y = 0.005  # ear centres either side of the 4 mm split
BOLT_X = -(R_OUT + 0.0048)  # pinch-bolt axis, along y, in the clamp frame
NUT_Y = EAR_Y + EAR[1] / 2 + 0.00325  # nut centre: on the +y ear, toward the left arm
NUT_AF, NUT_T = 0.013, 0.0065  # 13 mm across flats, 6.5 mm thick
CLAMP_KG, NUT_KG = 0.14, 0.030  # the clamp with its lug and the cable stub; the nut (7 g real) is 30 g for the solver
CABLE_L = 0.070  # the stub of 4 AWG cable drawn beyond the lug (the rest is not modelled)
LUG_Z = 0.004  # the lug's axis above the collar's middle
SEATED_Z = BOSS_TOP + H / 2  # clamp origin when its bottom face is on the boss


def _clamp_shapes() -> list:
    shapes = []
    r_mid = BORE_R + WALL / 2
    side = 2 * R_OUT * math.tan(math.pi / SIDES)
    for i in range(SIDES):
        t = 2 * math.pi * i / SIDES  # flats facing front, back and both sides
        shapes.append(BoxShape((r_mid * math.cos(t), r_mid * math.sin(t), 0.0), (WALL, side, H), color=LEAD, yaw=t))
    for sy in (-1, 1):
        shapes.append(BoxShape((-(R_OUT + EAR[0] / 2 - 0.002), sy * EAR_Y, 0.0), EAR, color=LEAD))
    # the bolt: carriage head on the -y ear, shank through both ears and out past the nut (nothing collides)
    shapes.append(CylShape((BOLT_X, -(EAR_Y + EAR[1] / 2 + 0.002), 0.0), 0.008, 0.004, color=(0.45, 0.45, 0.47),
                           collide=False, axis="Y"))
    shapes.append(CylShape((BOLT_X, 0.002, 0.0), 0.004, 0.034, color=(0.45, 0.45, 0.47), collide=False, axis="Y"))
    # the cable lug (crimp barrel, on the collar's upper half) and 7 cm of red cable, out the back
    shapes.append(CylShape((R_OUT + 0.013, 0.0, LUG_Z), 0.0065, 0.028, color=LEAD, axis="X"))
    shapes.append(CylShape((R_OUT + 0.027 + CABLE_L / 2, 0.0, LUG_Z), 0.0058, CABLE_L, color=RED, axis="X"))
    # the paint mark on the +y ear, beside the nut: the nut's start line points up from it
    shapes.append(BoxShape((BOLT_X, EAR_Y + EAR[1] / 2 + 0.0002, 0.0062), (0.0015, 0.0004, 0.003),
                           color=(0.95, 0.95, 0.95), collide=False))
    return shapes


def _nut_shapes() -> list:
    s = NUT_AF / math.sqrt(3)
    shapes = [BoxShape((0.0, 0.0, 0.0), (s, NUT_T, NUT_AF), color=(0.62, 0.62, 0.64),
                       quat_xyzw=_quat_axis((0, 1, 0), k * math.pi / 3)) for k in range(3)]
    # the paint line across the nut's outer face, pointing up (+z) at the start
    shapes.append(BoxShape((0.0, NUT_T / 2 + 0.0002, 0.0035), (0.0015, 0.0004, 0.006), color=(0.95, 0.95, 0.95),
                           collide=False))
    return shapes


# The clamp's start: set on the post but not pushed down, riding high on the taper and a little crooked.
START_TILT_DEG = 3.0
START_LIFT = 0.006  # its origin this far above the seated height
CLAMP_Q0 = _quat_axis((0, 1, 0), math.radians(START_TILT_DEG))  # cable end down
CLAMP_START = (POS_XY[0], POS_XY[1], SEATED_Z + START_LIFT)
CLAMP = Body.build("clamp", pos=CLAMP_START, mass=CLAMP_KG, shapes=_clamp_shapes(), color=LEAD,
                   start_xyzw=CLAMP_Q0, friction=(1.1, 0.9), com=(0.012, 0.0, 0.002))
_R0 = bodykit.rotation(CLAMP_Q0)
NUT_START = tuple((np.asarray(CLAMP_START) + _R0 @ np.array([BOLT_X, NUT_Y, 0.0])).tolist())
NUT = Body.build("nut", pos=NUT_START, mass=NUT_KG, shapes=_nut_shapes(), start_xyzw=CLAMP_Q0,
                 color=(0.62, 0.62, 0.64), ignore=("clamp",))

# -- the wrench: 13 mm combination wrench, 183 mm long, standing open end down in a wooden rack ----------------------
RING_R_IN, RING_R_OUT, RING_T = 0.0085, 0.0112, 0.007
HANDLE = (0.138, 0.005, 0.014)  # along -x from the ring, 5 mm thick along the ring's axis (y), 14 mm wide
WRENCH_KG = 0.105
CHROME = (0.80, 0.81, 0.83)


def _wrench_shapes() -> list:
    shapes = []
    for i in range(12):  # the box end: a 12-point ring round the y axis (drawn only: the catch joint holds the nut)
        t = 2 * math.pi * (i + 0.5) / 12
        r = (RING_R_IN + RING_R_OUT) / 2
        side = 2 * RING_R_OUT * math.tan(math.pi / 12)
        shapes.append(BoxShape((r * math.cos(t), 0.0, r * math.sin(t)), (RING_R_OUT - RING_R_IN, RING_T, side),
                               color=CHROME, collide=False, quat_xyzw=_quat_axis((0, 1, 0), -t)))
    shapes.append(BoxShape((-(RING_R_OUT + HANDLE[0] / 2 - 0.001), 0.0, 0.0), HANDLE, color=CHROME))
    head_x = -(RING_R_OUT + HANDLE[0] - 0.001)
    for sz in (-1, 1):  # the open end's two jaws
        shapes.append(BoxShape((head_x - 0.011, 0.0, sz * 0.0095), (0.022, 0.006, 0.006), color=CHROME))
    shapes.append(BoxShape((head_x - 0.003, 0.0, 0.0), (0.006, 0.006, 0.014), color=CHROME))
    return shapes


WRENCH_TIP = RING_R_OUT + HANDLE[0] - 0.001 + 0.022  # ring centre to the open end's tip (m)
RACK_XY = (0.105, 0.040)  # the tool stand, in front of the battery's left half
STAND = (0.060, 0.070, 0.120)  # steel pedestal: x, y, z
RACK = (0.050, 0.044, 0.030)  # the wooden holder on top: x, y, z; the slot runs along x through its middle
SLOT_W = 0.009
WOOD = (0.86, 0.47, 0.12)  # orange-painted wood
STEEL = (0.30, 0.31, 0.33)
RACK_TOP = TOP + STAND[2] + RACK[2]
RACK_BODY = Body.build(
    "rack", pos=(*RACK_XY, TOP), mass=2.5, color=WOOD, friction=(0.8, 0.7), com=(0.0, 0.0, 0.05),
    shapes=[BoxShape((0.0, 0.0, STAND[2] / 2), STAND, color=STEEL),
            *[BoxShape((0.0, sy * (SLOT_W / 2 + (RACK[1] - SLOT_W) / 4), STAND[2] + RACK[2] / 2),
                       (RACK[0], (RACK[1] - SLOT_W) / 2, RACK[2]), color=WOOD) for sy in (-1, 1)]])
WRENCH_Q0 = _quat_axis((0, 1, 0), -math.pi / 2)  # handle straight down, ring up, ring axis along y
WRENCH_START = (RACK_XY[0], RACK_XY[1], TOP + STAND[2] + WRENCH_TIP + 0.0008)
WRENCH = Body.build("wrench", pos=WRENCH_START, mass=WRENCH_KG, shapes=_wrench_shapes(), start_xyzw=WRENCH_Q0,
                    color=CHROME, friction=(0.8, 0.7), com=(-0.075, 0.0, 0.0))

OBJECTS = [BATTERY, RACK_BODY, CLAMP, NUT, WRENCH]

# -- joints -----------------------------------------------------------------------------------------------------------
# The nut on the bolt: free to turn about the bolt axis against thread friction, otherwise fixed to the clamp.
THREAD_T = 1.0  # N m: the nut turns on the bolt only past this torque (thread friction), at any speed; fingers
THREAD_K = 200.0  # N m/rad    do not reach it, a wrench does. A capped spring, re-anchored when the nut slips.
THREAD_DAMPING = 0.2  # N m s/rad
THREAD_T_ON = 20.0  # N m: with the box end on, the nut is held to the wrench's angle (the box end grips its flats)
CATCH_R = 0.004  # the box end finds the nut (its chamfer) within this of the bolt's axis
CATCH_MIS = 10.0  # degrees: and within this of its direction
bodykit.thread("nut_on_bolt", "clamp", "nut", anchor0=(BOLT_X, NUT_Y, 0.0), axis0=(0.0, 1.0, 0.0),
               anchor1=(0.0, 0.0, 0.0), axis1=(0.0, 1.0, 0.0), slide_limits=(-0.0001, 0.0001),
               turn_damping=THREAD_DAMPING, slide_stiffness=0.0, slide_damping=0.0, slide_max_force=0.0,
               always_on=True)
# The wrench's box end on the nut: switched on when the ring is slid over the nut square to it, off once it is pulled
# back off. It turns with the nut (its turn damping is far above the thread's) and slides freely along the bolt.
bodykit.thread("box_end", "nut", "wrench", anchor0=(0.0, 0.0, 0.0), axis0=(0.0, 1.0, 0.0),
               anchor1=(0.0, 0.0, 0.0), axis1=(0.0, 1.0, 0.0), slide_limits=(-0.012, 0.040),
               turn_damping=300.0, slide_stiffness=300.0, slide_damping=2.0, slide_max_force=4.0)
# The collar on the tapered post: it can tilt a little in the bore and turn about the post, and it slides along the
# post against the taper's grip, a friction force of F_SLIDE: it stays wherever it is left and goes down only when
# pushed with more than that. on_step re-anchors the slide drive after every slip (Coulomb friction from a capped
# spring). Lifted off the top of the post it comes free, and set back on the post it is caught again where it is.
F_SLIDE = 4.0  # N
# The cable is stiff: bent down from where it rests, it pushes the clamp back up (a spring at the lug, CABLE_K, capped at
# CABLE_F). It is what holds the clamp up off its seat at the start, and why the clamp has to be held down until the
# nut has bitten: let go before that, and it rides back up.
CABLE_K, CABLE_F = 4000.0, 12.0  # N/m, N
CABLE_AT = 0.060  # m behind the post's axis, where the cable's pull acts (the hand holds it there)
TILT_FREE = (5.0, 0.8)  # degrees the collar can rock in the bore: front to back (about the bolt), side to side
TILT_K = 15.0  # N m/rad: the bore's fit centres the collar (the pinch-bolt torque rocks it about 5 degrees)
SLIDE_K = 20000.0  # N/m
S_SEATED = H / 2  # the slide coordinate (clamp origin above the boss top) when the collar is down on the boss
S_OFF = H / 2 + POST_H - 0.004  # above this the collar's bottom is clear of the post's top: it comes free
bodykit.thread("collar_on_post", "battery", "clamp", anchor0=POS_LOCAL, axis0=(0.0, 0.0, 1.0),
               anchor1=(0.0, 0.0, 0.0), axis1=(0.0, 0.0, 1.0), slide_limits=(S_SEATED, S_OFF + 0.006),
               turn_damping=0.05, slide_stiffness=SLIDE_K, slide_damping=30.0, slide_max_force=F_SLIDE,
               play=0.0003, tilt_deg=TILT_FREE, always_on=True, tilt_stiffness=TILT_K, tilt_damping=0.05)
# The collar's grip on the post once the nut bites: it stays at the height and heading it had then.
bodykit.thread("collar_grip", "battery", "clamp", anchor0=POS_LOCAL, axis0=(0.0, 0.0, 1.0),
               anchor1=(0.0, 0.0, 0.0), axis1=(0.0, 0.0, 1.0), slide_limits=(0.0, 0.040),
               turn_damping=5000.0, slide_stiffness=20000.0, slide_damping=200.0, slide_max_force=400.0,
               play=0.0003, tilt_deg=0.5)

# -- what a person measures -------------------------------------------------------------------------------------------
GAP_SEATED = 0.0010  # every point under the collar within 1 mm of the boss (feeler gauge)
TILT_SEATED = 2.0  # degrees
ON_POST = 0.003  # clamp axis within 3 mm of the positive post's axis
TURN_BITE = 20.0  # degrees past the mark: the collar starts to pinch the post
TURN_TIGHT = 75.0  # degrees past the mark: tight
TURN_OVER = 360.0  # past a full turn the collar is crushed onto the post
LUG_FULL, LUG_HALF = 10.0, 30.0  # lug heading off the cable's run (+x), degrees
CLEAR = 0.040  # grippers' pads this far from the clamp, the nut and the wrench at the end
WRENCH_OFF = 0.020  # the wrench's ring this far from the nut
WRENCH_LOW = 0.030  # near the battery, every part of the wrench this low (lying on the bench, not on the battery)
STILL_V, STILL_W = 0.01, 0.10  # m/s, rad/s, averaged over the last second (STILL_STEPS control steps)
STILL_STEPS = 20


BATT_MOVED = 0.005  # the battery shifted more than 5 mm on the bench: 0


def post_frame(battery) -> tuple[np.ndarray, float]:
    """The positive post's axis (x, y) and the top of its boss (z), from the battery's pose (world, m)."""
    p = np.asarray(battery.pos, float) + battery.rotation() @ np.asarray(POS_LOCAL)
    return p[:2], float(p[2])


def _collar_points(clamp) -> np.ndarray:
    """Four points under the collar, front, back, left and right, at its outer radius (world)."""
    r = clamp.rotation()
    pts = [(R_OUT, 0.0), (-R_OUT, 0.0), (0.0, R_OUT), (0.0, -R_OUT)]
    return np.array([np.asarray(clamp.pos, float) + r @ np.array([x, y, -H / 2]) for x, y in pts])


def gaps(clamp, battery) -> np.ndarray:
    """Height of each of the four points above the boss (m)."""
    return _collar_points(clamp)[:, 2] - post_frame(battery)[1]


def tilt_deg(clamp) -> float:
    return math.degrees(clamp.tilt())


def off_axis(clamp, battery) -> float:
    return float(np.linalg.norm(np.asarray(clamp.pos[:2], float) - post_frame(battery)[0]))


def lug_deg(clamp) -> float:
    """The lug's heading off its line (straight back across the battery, the cable's run), degrees in (-180, 180]."""
    r = clamp.rotation()
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def wrench_rel_deg(clamp, wrench) -> float:
    """The wrench's angle about the clamp's y axis (the bolt's), degrees: its handle's angle above level, toward the
    robot, when it is on the nut."""
    rel = clamp.rotation().T @ wrench.rotation()
    return math.degrees(math.atan2(rel[0, 2], rel[0, 0]))


def nut_rel_deg(clamp, nut) -> float:
    """The nut's turn on the bolt from its start mark, in (-180, 180]: + is tightening (clockwise seen from the nut's
    end, the left: a turn about the clamp's -y axis)."""
    rel = clamp.rotation().T @ nut.rotation()
    return -math.degrees(math.atan2(rel[0, 2], rel[0, 0]))


# State kept over an episode (on_step sees the simulation; success() and score() only see poses).
_EP: dict = {}


@bodykit.on_reset
def _reset() -> None:
    _EP.update(turn=0.0, last=0.0, raw=0.0, nut_target=0.0, wa=0.0, wa_last=0.0, grip=(0.0, 0.0), locked=False,
               bitten=False, caught=False, pads={}, poses=[], step=0, batt0=None, on_post=True, z_rest=None, cable_f=0.0,
               slide_target=S_SEATED + START_LIFT)


_reset()


def slide(clamp, battery) -> float:
    """The clamp origin's height above the boss top, along the post's axis (m)."""
    base = np.asarray(battery.pos, float) + battery.rotation() @ np.asarray(POS_LOCAL)
    return float((np.asarray(clamp.pos, float) - base) @ battery.rotation()[:, 2])


def _cable_point(clamp) -> np.ndarray:
    return np.asarray(clamp.pos, float) + clamp.rotation() @ np.array([CABLE_AT, 0.0, LUG_Z])


def _cable_push(env, clamp) -> None:
    """Push the clamp up at the cable's point by the cable's spring-back (nothing once the collar is pinched)."""
    import torch

    p = _cable_point(clamp)
    if _EP["z_rest"] is None:
        _EP["z_rest"] = float(p[2])
    f = 0.0 if _EP["locked"] else min(CABLE_F, CABLE_K * max(0.0, _EP["z_rest"] - float(p[2])))
    _EP["cable_f"] = f
    base = env.unwrapped
    obj = base.scene["clamp"]
    dev = base.device
    origin = base.scene.env_origins[0]
    forces = torch.tensor([[[0.0, 0.0, f]]], dtype=torch.float32, device=dev)
    torques = torch.zeros((1, 1, 3), dtype=torch.float32, device=dev)
    pos = (torch.tensor(p, dtype=torch.float32, device=dev) + origin).reshape(1, 1, 3)
    obj.permanent_wrench_composer.reset()
    obj.permanent_wrench_composer.add_forces_and_torques(forces, torques, positions=pos, body_ids=[0], is_global=True)


def _seated(clamp, battery) -> bool:
    g = gaps(clamp, battery)
    return bool(g.max() <= GAP_SEATED and tilt_deg(clamp) <= TILT_SEATED
                and off_axis(clamp, battery) <= ON_POST)


def _ring_vs_nut(nut, wrench):
    """(radial offset, slide along the bolt out from the nut, axis misalignment deg, flat misalignment deg)."""
    axis_n = nut.rotation() @ np.array([0.0, 1.0, 0.0])
    axis_w = wrench.rotation() @ np.array([0.0, 1.0, 0.0])
    d = np.asarray(wrench.pos, float) - np.asarray(nut.pos, float)
    s = float(d @ axis_n)
    radial = float(np.linalg.norm(d - s * axis_n))
    mis = math.degrees(math.acos(max(-1.0, min(1.0, float(axis_n @ axis_w)))))
    rel = nut.rotation().T @ wrench.rotation()
    flat = math.degrees(math.atan2(rel[0, 2], rel[0, 0])) % 30.0
    return radial, s, mis, min(flat, 30.0 - flat)


def _wrap180(a: float) -> float:
    """An angle in degrees, wrapped to [-180, 180): the nut's hold is an orientation, so a turn past half a turn is
    the same target."""
    return (a + 180.0) % 360.0 - 180.0


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, ["battery", "clamp", "nut", "wrench"])
    battery, clamp, nut, wrench = st["battery"], st["clamp"], st["nut"], st["wrench"]
    if _EP["batt0"] is None:
        _EP["batt0"] = np.asarray(battery.pos, float).copy()
        _EP["last"] = _EP["raw"] = nut_rel_deg(clamp, nut)
        _EP["wa_last"] = _EP["wa"] = wrench_rel_deg(clamp, wrench)
    # the nut's turn, unwrapped
    a = nut_rel_deg(clamp, nut)
    step = (a - _EP["last"] + 180.0) % 360.0 - 180.0
    _EP["turn"] += step
    _EP["raw"] += step
    _EP["last"] = a
    # the wrench's angle about the bolt (unwrapped), for the box end's grip on the nut
    w_now = wrench_rel_deg(clamp, wrench)
    _EP["wa"] += (w_now - _EP["wa_last"] + 180.0) % 360.0 - 180.0
    _EP["wa_last"] = w_now
    # the box end catching the nut, and coming off it
    radial, s, mis, flat = _ring_vs_nut(nut, wrench)
    if not _EP["caught"] and radial <= CATCH_R and -0.010 <= s <= 0.006 and mis <= CATCH_MIS and flat <= 7.0:
        bodykit.set_slide_target("box_end", 0.0)  # it sits on the nut until pulled off
        bodykit.set_thread_on("box_end", True)
        _EP["caught"] = True
        _EP["grip"] = (_EP["wa"], -_EP["raw"])  # the wrench's angle and the nut's turn (joint frame) when it caught
        bodykit.set_turn_cap("nut_on_bolt", THREAD_T_ON)
    elif _EP["caught"] and s > 0.009:
        bodykit.set_thread_on("box_end", False)
        _EP["caught"] = False
        bodykit.set_turn_cap("nut_on_bolt", THREAD_T)
        _EP["nut_target"] = -_EP["raw"]
        bodykit.set_turn_hold("nut_on_bolt", THREAD_K, _wrap180(_EP["nut_target"]))
    if _EP["caught"]:
        # a box end grips the flats: the nut turns with the wrench (driven to the wrench's angle)
        _EP["nut_target"] = _EP["grip"][1] + (_EP["wa"] - _EP["grip"][0])
        bodykit.set_turn_hold("nut_on_bolt", THREAD_K, _wrap180(_EP["nut_target"]))
    elif abs(-_EP["raw"] - _EP["nut_target"]) > 0.8 * math.degrees(THREAD_T / THREAD_K):
        # the thread's friction: if the nut has slipped past its hold, it stays where it went
        _EP["nut_target"] = -_EP["raw"]
        bodykit.set_turn_hold("nut_on_bolt", THREAD_K, _wrap180(_EP["nut_target"]))
    # the cable's spring-back, until the collar is pinched
    _cable_push(env, clamp)
    # the collar sliding on the post against the taper's grip; off the top of the post it is free
    s_now = slide(clamp, battery)
    if _EP["on_post"]:
        if s_now > S_OFF:
            bodykit.set_thread_on("collar_on_post", False)
            _EP["on_post"] = False
        elif abs(s_now - _EP["slide_target"]) > 0.8 * F_SLIDE / SLIDE_K:  # it slipped: it stays where it went
            _EP["slide_target"] = s_now
            bodykit.set_slide_target("collar_on_post", s_now)
    elif not _EP["locked"] and s_now < S_OFF - 0.002 and off_axis(clamp, battery) <= 0.0015 and tilt_deg(clamp) <= 4.0:
        _EP["slide_target"] = s_now
        bodykit.set_slide_target("collar_on_post", s_now)
        bodykit.set_thread_on("collar_on_post", True)
        _EP["on_post"] = True
    # the collar biting on the post once the nut has turned TURN_BITE: square, it locks where it is; crooked, it
    # binds crooked and loose (it is left free to rock, and never locks)
    if not _EP["bitten"] and _EP["turn"] >= TURN_BITE and _EP["on_post"]:
        _EP["bitten"] = True
        if tilt_deg(clamp) > TILT_SEATED:
            bodykit.set_tilt_spring("collar_on_post", 0.0)  # pinched crooked: it binds loose and crooked, and flops
        else:
            bodykit.set_slide_target("collar_grip", s_now)
            bodykit.set_thread_on("collar_grip", True)
            bodykit.set_thread_on("collar_on_post", False)  # the pinch replaces the sliding fit
            _EP["on_post"] = False
            _EP["locked"] = True
    # where the finger pads are
    from bodykit import as_numpy

    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        names = list(robot.data.body_names)
        pos = as_numpy(robot.data.body_pos_w)[0]
        _EP["pads"][arm] = [pos[names.index(n)] - origin for n in ("tip_left", "tip_right")]
        if _DBG is not None:
            from yam_arena import ik, station

            q = as_numpy(robot.data.joint_pos)[0][:6]
            _EP.setdefault("tcp", {})[arm] = ik.tcp_pose(q, station.BASES[arm])[0]
    _EP.setdefault("poses", []).append({n: (np.asarray(o.pos, float).copy(), o.rotation()) for n, o in
                                         (("clamp", clamp), ("nut", nut), ("wrench", wrench))})
    del _EP["poses"][:-(STILL_STEPS + 1)]
    _EP["step"] += 1
    if _DBG is not None:
        _debug(battery, clamp, nut, wrench, radial, s, mis, flat)


def _pads_clear(objects) -> bool:
    if not _EP["pads"]:
        return True
    targets = [np.asarray(objects["clamp"].pos, float), np.asarray(objects["nut"].pos, float)]
    w = objects["wrench"]
    for t in np.linspace(0.0, WRENCH_TIP, 8):  # points along the wrench
        targets.append(np.asarray(w.pos, float) + w.rotation() @ np.array([-t, 0.0, 0.0]))
    for pads in _EP["pads"].values():
        for p in pads:
            if min(float(np.linalg.norm(p - t)) for t in targets) < CLEAR:
                return False
    return True


def _wrench_away(objects) -> bool:
    """Off the nut by 2 cm and not resting on or leaning against the battery: no part of the wrench more than 3 cm
    above the bench within 1 cm of the battery's outline seen from above (lying flat on the bench beside it is
    fine; on its lid, across its posts or propped against it is not)."""
    w, nut, b = objects["wrench"], objects["nut"], objects["battery"]
    if _EP["caught"] or float(np.linalg.norm(np.asarray(w.pos) - np.asarray(nut.pos))) < WRENCH_OFF:
        return False
    rb, cb = b.rotation(), np.asarray(b.pos, float)
    half = np.asarray(BATT, float)[:2] / 2 + 0.01 + RING_R_OUT  # the outline plus 1 cm, less the wrench's half-width
    for t in np.linspace(0.0, WRENCH_TIP, 12):
        p = np.asarray(w.pos, float) + w.rotation() @ np.array([-t, 0.0, 0.0])
        local = rb.T @ (p - cb)
        if bool(np.all(np.abs(local[:2]) < half)) and float(p[2]) - TOP > WRENCH_LOW:
            return False
    return True


def _still(objects) -> bool:
    """The clamp, the nut and the wrench each moved less than STILL_V and STILL_W over the last second (what a video
    shows; the solver's instantaneous velocities flicker for a body at rest in contact). Without that history (the
    tests), their velocities now."""
    hist = _EP.get("poses", [])
    if len(hist) > STILL_STEPS:
        then, now = hist[-1 - STILL_STEPS], hist[-1]
        for n in ("clamp", "nut", "wrench"):
            (p0, r0), (p1, r1) = then[n], now[n]
            turn = math.acos(max(-1.0, min(1.0, (float(np.trace(r0.T @ r1)) - 1.0) / 2.0)))
            if float(np.linalg.norm(p1 - p0)) >= STILL_V * 1.0 or turn >= STILL_W * 1.0:
                return False
        return True
    return all(float(np.linalg.norm(objects[n].lin_vel)) < STILL_V and float(np.linalg.norm(objects[n].ang_vel)) < STILL_W
               for n in ("clamp", "nut", "wrench"))


def _batt_moved(objects) -> bool:
    if _EP["batt0"] is None:
        return False
    return float(np.linalg.norm(np.asarray(objects["battery"].pos, float) - _EP["batt0"])) > BATT_MOVED


def _stage(objects) -> float:
    """Partial credit from the clamp, the nut and the lug, before the hands and the wrench are checked."""
    clamp, battery = objects["clamp"], objects["battery"]
    if _batt_moved(objects):
        return 0.0
    if off_axis(clamp, battery) > ON_POST or gaps(clamp, battery).max() > 0.012:
        return 0.0  # not on the positive post
    if not _seated(clamp, battery):
        return 0.0  # high or crooked: tightened there it is a loose joint; not yet tightened it is no progress
    turn = _EP["turn"]
    if turn < TURN_TIGHT or turn > TURN_OVER:
        return 0.25  # seated but loose, or crushed
    lug = abs(lug_deg(clamp))
    if lug > LUG_HALF:
        return 0.25
    if lug > LUG_FULL:
        return 0.5
    return 0.75


def _end_flags(objects) -> str:
    return "".join("1" if f(objects) else "0" for f in (_wrench_away, _pads_clear, _still))


def score(objects) -> float:
    """1.0: collar seated (every point under it within 1 mm of the boss, square within 2 degrees, on the post's axis
    within 3 mm), nut turned 75 to 360 degrees past its mark, lug within 10 degrees of the cable's run, wrench off the
    nut and off the battery, pads 4 cm clear, all still. 0.75: the same with a hand or the wrench still on it or
    something moving. 0.5: lug 10 to 30 degrees off. 0.25: seated but loose (under 75 degrees), crushed (over 360),
    or lug over 30 degrees off. 0: not on the positive post, high or crooked, or the battery moved over 5 mm."""
    base = _stage(objects)
    if _DBG is not None:
        _EP["end"] = _end_flags(objects)
    if base == 0.75 and _wrench_away(objects) and _pads_clear(objects) and _still(objects):
        return 1.0
    return base


def success(objects) -> bool:
    return score(objects) == 1.0


def distance(objects) -> float:
    clamp, battery = objects["clamp"], objects["battery"]
    return float(max(0.0, gaps(clamp, battery).max()) + off_axis(clamp, battery)
                 + max(0.0, TURN_TIGHT - _EP["turn"]) * 1e-4)


# -- the seeded start: each trial moves the battery, tips the clamp a different way and moves the rack -------------
VARY_BATT = (0.012, 0.015)  # battery centre: +- x, +- y (m)
VARY_RACK = (0.015, 0.020)
VARY_TILT = (0.5, 2.5)  # the clamp's tilt (degrees) as it is set down, cable end down, give or take 30 degrees
VARY_LIFT = (0.006, 0.006)  # how high it rides above its seat: always 6 mm (the cable's set holds it there)
VARY_YAW = 4.0  # degrees


def layout(seed: int | None) -> dict:
    """The start for a seed (None: the nominal layout in the setup text): world poses, quaternions x, y, z, w."""
    rng = np.random.default_rng(seed) if seed is not None else None

    def u(lo, hi):
        return float(rng.uniform(lo, hi)) if rng is not None else (lo + hi) / 2

    db = np.array([u(-VARY_BATT[0], VARY_BATT[0]), u(-VARY_BATT[1], VARY_BATT[1]), 0.0])
    dr = np.array([u(-VARY_RACK[0], VARY_RACK[0]), u(-VARY_RACK[1], VARY_RACK[1]), 0.0])
    if rng is None:
        q_clamp, lift = CLAMP_Q0, START_LIFT
    else:
        axis_dir = math.pi / 2 + u(-math.pi / 6, math.pi / 6)  # about +y: the cable end down
        tilt = math.radians(u(*VARY_TILT))
        q_tilt = _quat_axis((math.cos(axis_dir), math.sin(axis_dir), 0.0), tilt)
        q_clamp = _quat_mul(q_tilt, _quat_axis((0, 0, 1), math.radians(u(-VARY_YAW, VARY_YAW))))
        lift = u(*VARY_LIFT)
    batt = np.array([*BATT_XY, BATT_CZ]) + db
    clamp = np.array([POS_XY[0], POS_XY[1], SEATED_Z + lift]) + db
    nut = clamp + bodykit.rotation(q_clamp) @ np.array([BOLT_X, NUT_Y, 0.0])
    return {"battery": (batt, (0.0, 0.0, 0.0, 1.0)), "clamp": (clamp, q_clamp), "nut": (nut, q_clamp),
            "rack": (np.array([*RACK_XY, TOP]) + dr, (0.0, 0.0, 0.0, 1.0)),
            "wrench": (np.array(WRENCH_START) + dr, WRENCH_Q0)}


@bodykit.on_reset_env
def _lay_out(emb, seed):
    """Put the objects where this trial's seed says, let them settle, and return the first observation."""
    import torch
    from yam_arena import inspect_embodiment as ie

    base = emb._base
    lay = layout(seed)
    if _DBG is not None:
        _DBG.write(f"seed {seed}: clamp {lay['clamp'][0][2] - SEATED_Z:.4f} m high, battery {lay['battery'][0][:2]}, "
                   f"rack {lay['rack'][0][:2]}\n")
    _reset()
    _EP["slide_target"] = float(lay["clamp"][0][2] - lay["battery"][0][2] - POS_LOCAL[2])
    bodykit.set_thread_on("collar_on_post", True)
    bodykit.set_tilt_spring("collar_on_post", TILT_K)
    bodykit.set_turn_hold("nut_on_bolt", THREAD_K, 0.0)
    bodykit.set_turn_cap("nut_on_bolt", THREAD_T)
    bodykit.set_slide_target("collar_on_post", _EP["slide_target"])  # before it settles, so it stays this high
    base.scene["clamp"].permanent_wrench_composer.reset()  # no cable push left over from the last trial
    for name, (pos, quat) in lay.items():
        obj = base.scene[name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(pos, dtype=pose.dtype, device=pose.device) + base.scene.env_origins
        pose[:, 3:7] = torch.tensor(quat, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
    for _ in range(max(1, round(getattr(ie, "SETTLE_S", 0.5) / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(getattr(ie, "RESET_RENDERS", 3)):
        base.sim.render()
    target = _EP["slide_target"]
    _reset()
    _EP["slide_target"] = target
    return emb._observation()


# -- the scripted demo ------------------------------------------------------------------------------------------------
DOWN = grip_rotation(0.0)  # straight down; the fingers close along the TCP frame's x axis, here world -y
HOLD_PITCH = math.radians(40.0)
HOLD_YAW = 0.0
HOLD_ROT = _rot("y", -HOLD_PITCH) @ _rot("z", HOLD_YAW) @ DOWN  # leaning forward; the fingers close along y
HOLD_APPROACH = HOLD_ROT[:, 2]
HOLD_OUT = 0.060  # the right hand grips the cable this far behind the post's axis, clear of the wrench
GRIP_ABOVE = 0.0045  # the right hand closes on the cable this far above its seated level (it rides 6 mm up)
PUSH_DOWN = 0.012  # then pushes it this far down (m): past the seat, so the collar stops on the boss
SIDE = _rot("x", -math.pi / 2) @ DOWN  # approaching along -y (from the left), fingers closing along z


def wrench_grip_rot(a: float) -> np.ndarray:
    """Left hand on the wrench's handle when the handle points toward the robot, ``a`` rad above level."""
    return _rot("y", a) @ SIDE  # the fingers close across the handle's width


GRIP_R = 0.110  # the left hand grips the handle this far from the ring centre
# The swing: tightening (clockwise seen from the nut's end, the left) pushes the handle down. Two strokes of 70 and
# 60 degrees: the box end goes on at A_ON, is pushed down to A_END, comes off, goes back on at A_REFIT (60 degrees up:
# the 12-point box end fits every 30 degrees) and is pushed down to A_END again. A_END keeps the handle clear of the
# battery's front edge.
A_ON, A_REFIT, A_END = math.radians(28.0), math.radians(28.0), math.radians(-32.0)


def _handle_point(ring, a: float, r: float = GRIP_R) -> tuple[float, float, float]:
    ring = np.asarray(ring, float)
    return tuple((ring + r * np.array([-math.cos(a), 0.0, math.sin(a)])).tolist())


def hold_plan(post_xy, t_release: float) -> list[Waypoint]:
    """Right hand: grip the cable behind the post, push the clamp down by it until the cable is level at its seated
    height (the collar then sits square on the boss), hold it there until ``t_release``, let go."""
    level = np.array([post_xy[0] + HOLD_OUT, post_xy[1], SEATED_Z + LUG_Z])  # the cable with the clamp seated, square
    c = level + np.array([0.0, 0.0, GRIP_ABOVE])  # where the cable is gripped: at or above where it rides at the start
    pre = c - 0.06 * HOLD_APPROACH
    down = np.array([0.0, 0.0, PUSH_DOWN])  # c - down: past the seated level; the collar stops on the boss
    plan = [
        Waypoint(3.0, tuple(pre), FINGER_OPEN, "over the clamp", HOLD_ROT),
        Waypoint(1.5, tuple(c), FINGER_OPEN, "round the cable", HOLD_ROT),
        Waypoint(1.0, tuple(c), FINGER_CLOSED, "grip the cable", HOLD_ROT),
        Waypoint(1.5, tuple(c - down), FINGER_CLOSED, "push the clamp down onto the boss", HOLD_ROT),
    ]
    used = sum(w.duration for w in plan)
    plan.append(Waypoint(max(0.5, t_release - used), tuple(c - down), FINGER_CLOSED,
                         "hold it down while the nut is turned", HOLD_ROT))
    plan += [
        Waypoint(1.0, tuple(c - down), FINGER_OPEN, "let go", HOLD_ROT),
        Waypoint(1.5, tuple(c + 0.02 * np.array([0, 0, 1.0]) - 0.07 * HOLD_APPROACH), FINGER_OPEN, "back away",
                 HOLD_ROT),
    ]
    return plan


def wrench_plan(post_xy, rack_xy) -> tuple[list[Waypoint], float]:
    """Left hand: the wrench out of its rack, onto the nut, and round in two strokes (the wrist turns only so far, so
    between them the box end comes off, swings back 60 degrees and goes on again, as with any box-end wrench in a tight
    spot), off, and down on the bench in front of the battery. Returns the plan and the time at which the box end is
    off the nut for good."""
    nut_seated = np.array([post_xy[0] + BOLT_X, post_xy[1] + NUT_Y, SEATED_Z])  # the nut with the clamp seated
    rack_ring = np.array([rack_xy[0], rack_xy[1], WRENCH_START[2]])
    a_rack = -math.pi / 2
    grip = np.array(_handle_point(rack_ring, a_rack))
    lift = np.array([0.0, 0.0, 0.075])
    back = np.array([0.0, 0.045, 0.0])  # the left hand comes in from the left
    off = np.array([0.0, 0.045, 0.0])  # the box end slid off the nut, along the bolt
    plan = [
        Waypoint(2.5, tuple(grip + back), FINGER_OPEN, "beside the wrench", wrench_grip_rot(a_rack)),
        Waypoint(1.5, tuple(grip), FINGER_OPEN, "fingers round the handle", wrench_grip_rot(a_rack)),
        Waypoint(1.0, tuple(grip), FINGER_CLOSED, "grip the handle", wrench_grip_rot(a_rack)),
        Waypoint(1.5, tuple(grip + lift), FINGER_CLOSED, "lift it out of the rack", wrench_grip_rot(a_rack)),
    ]

    on = nut_seated + np.array([0.0, 0.001, 0.0])  # the box end slid on to the nut's face

    def fit(a, seconds, settle=1.0):
        plan.append(Waypoint(seconds, _handle_point(nut_seated + off, a), FINGER_CLOSED, "box end lined up with the nut",
                             wrench_grip_rot(a)))
        plan.append(Waypoint(settle, _handle_point(nut_seated + off, a), FINGER_CLOSED, "settle", wrench_grip_rot(a)))
        plan.append(Waypoint(1.5, _handle_point(on, a), FINGER_CLOSED, "slide the box end onto the nut",
                             wrench_grip_rot(a)))

    def stroke(a0, a1, n=7):
        for k in range(1, n + 1):
            a = a0 + (a1 - a0) * k / n
            plan.append(Waypoint(0.45, _handle_point(on, a), FINGER_CLOSED, "push the wrench round",
                                 wrench_grip_rot(a)))
        plan.append(Waypoint(0.5, _handle_point(on, a1), FINGER_CLOSED, "push the wrench round",
                             wrench_grip_rot(a1)))
        plan.append(Waypoint(1.2, _handle_point(nut_seated + off, a1), FINGER_CLOSED, "slide the box end off",
                             wrench_grip_rot(a1)))
        plan.append(Waypoint(0.6, _handle_point(nut_seated + off, a1), FINGER_CLOSED, "box end off",
                             wrench_grip_rot(a1)))

    fit(A_ON, 3.0, settle=2.0)  # the clamp is pushed fully down before the nut is turned
    stroke(A_ON, A_END)
    fit(A_REFIT, 1.5)
    stroke(A_REFIT, A_END)
    t_off = sum(w.duration for w in plan)
    # let it drop onto the bench in front of the battery, well clear of it (the arm reaches no lower with this grip)
    a_down = math.radians(SET_DOWN[3])
    down = np.array([SET_DOWN[0], SET_DOWN[1], TOP + SET_DOWN[2]])
    plan += [
        Waypoint(3.0, tuple(down + np.array([0.0, 0.0, 0.02])), FINGER_CLOSED, "over the bench", wrench_grip_rot(a_down)),
        Waypoint(1.0, tuple(down), FINGER_CLOSED, "lower it", wrench_grip_rot(a_down)),
        Waypoint(1.0, tuple(down), FINGER_OPEN, "let go", wrench_grip_rot(a_down)),
        Waypoint(1.5, tuple(down + np.array([0.0, 0.0, 0.05])), FINGER_OPEN, "back away", wrench_grip_rot(a_down)),
        Waypoint(2.0, tuple(down + np.array([0.03, 0.0, 0.10])), FINGER_OPEN, "clear", wrench_grip_rot(a_down)),
        Waypoint(3.0, tuple(down + np.array([0.03, 0.0, 0.10])), FINGER_OPEN, "wait for it all to settle",
                 wrench_grip_rot(a_down)),
    ]
    return plan, t_off


# where the left hand lets go of the wrench: x, y, height above the bench (m), handle angle (degrees, up toward the
# robot); it drops onto the bench in front of the battery (where it comes to rest varies)
SET_DOWN = (0.060, 0.020, 0.170, 30.0)


def plans(post_xy, rack_xy) -> dict[str, list[Waypoint]]:
    """Both hands' plans for a post and a rack at these positions."""
    left, t_off = wrench_plan(post_xy, rack_xy)
    return {"right": hold_plan(post_xy, t_off + 0.5), "left": left}


def demo():
    """The plans for the nominal layout in the setup text. The scripted policy replaces them, on its first step,
    with plans made from where the top camera sees the battery and the rack (clampdemo.py)."""
    return plans(POS_XY, RACK_XY)


import clampdemo  # noqa: E402  (the demo's eyes: hooks yam_scripted to plan from the first camera frame)

clampdemo.install()
plan_from_image_hook = True  # tells clampdemo this task plans from the first camera frame


# -- debug: CLAMP_DEBUG=/path/to/log writes the state after every step ------------------------------------------------
_DBG = open(os.environ["CLAMP_DEBUG"], "w") if os.environ.get("CLAMP_DEBUG") else None


def _debug(battery, clamp, nut, wrench, radial, s, mis, flat) -> None:
    g = gaps(clamp, battery) * 1000.0
    pads = {k: (v[0] + v[1]) / 2 for k, v in _EP["pads"].items()}
    _DBG.write(f"{_EP['step']:4d} c=({clamp.pos[0]:.4f},{clamp.pos[1]:.4f},{clamp.pos[2]:.4f}) "
               f"gap={g.min():6.2f}..{g.max():6.2f} tilt={tilt_deg(clamp):5.2f} lug={lug_deg(clamp):6.1f} "
               f"off={off_axis(clamp, battery) * 1000:5.1f} turn={_EP['turn']:6.1f} lock={int(_EP['locked'])} "
               f"catch={int(_EP['caught'])} ring r={radial * 1000:5.1f} s={s * 1000:6.1f} mis={mis:5.1f} "
               f"flat={flat:4.1f} wa={math.degrees(math.atan2((clamp.rotation().T @ wrench.rotation())[0, 2], (clamp.rotation().T @ wrench.rotation())[0, 0])):6.1f} w=({wrench.pos[0]:.3f},{wrench.pos[1]:.3f},{wrench.pos[2]:.3f}) "
               f"cf={_EP.get('cable_f', 0.0):4.1f} end={_EP.get('end', '')} "
               f"v=c{np.linalg.norm(clamp.lin_vel):.3f}/{np.linalg.norm(clamp.ang_vel):.2f} "
               f"n{np.linalg.norm(nut.lin_vel):.3f}/{np.linalg.norm(nut.ang_vel):.2f} "
               f"w{np.linalg.norm(wrench.lin_vel):.3f}/{np.linalg.norm(wrench.ang_vel):.2f} L={np.round(pads.get('left', np.zeros(3)), 3)} R={np.round(pads.get('right', np.zeros(3)), 3)} "
               f"tcpL={np.round(_EP.get('tcp', {}).get('left', np.zeros(3)), 4)} "
               f"tcpR={np.round(_EP.get('tcp', {}).get('right', np.zeros(3)), 4)}\n")
    _DBG.flush()
