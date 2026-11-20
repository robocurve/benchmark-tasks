"""On the station table: a plastic storage drawer unit straight ahead of the robots, its one drawer pulled about halfway
out, and a white desk-organizer tray lying on the table to the robots' left.

The tray fits the drawer with a few millimetres to spare all round. Seated, it sits flat on the drawer's floor and the
drawer closes. A black marker pen lies on the drawer's floor near its back, under the unit's top, where it can't be
seen while the drawer is half shut. Laid in on top of the pen, the tray rides up at the back and the drawer will not
close past it.

The task: put the organizer in the drawer, fully seated, and close the drawer.

Run it:  inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=$PWD/task.py -E object_jitter=0 \\
           -E control_hz=20 --scorer task_score --grader none --max-steps 2400 --seed 1 \\
           --instruction "<see task.json>"
"""

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import labkit  # noqa: E402
from labkit import Joint, on_reset, on_step, part  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

UNIT_C = (0.86, 0.87, 0.88)  # light grey polypropylene
DRAWER_C = (0.80, 0.84, 0.88)
HANDLE_C = (0.25, 0.26, 0.28)
TRAY_C = (0.95, 0.95, 0.93)
PEN_C = (0.06, 0.06, 0.07)

# == the drawer unit (fixed) and its drawer =============================================================================
# World coordinates. The unit's front face is at x = FACE; the drawer runs along x, closed at 0, pulled out to -TRAVEL.
FACE = 0.377
BACK = 0.605
HALF_W = 0.150  # the unit is 300 mm wide
BASE_T = 0.015  # bottom panel
SIDE_T = 0.014
TOP_T = 0.015
OPEN_H = 0.072  # the opening's top (the top panel's underside), above the table
TRAVEL = 0.205  # the drawer pulls out 205 mm to its stop: then all of its inside is clear of the unit's top
START = -0.090  # it starts pulled out 90 mm

FLOOR_Z = TOP + BASE_T + 0.003  # the drawer floor's top face (3 mm floor on the bottom panel)
IN_X = (FACE, FACE + 0.200)  # the drawer's inside, closed: 200 mm deep
IN_HALF_W = 0.130  # and 260 mm wide
WALL_T = 0.003
WALL_H = 0.050  # walls 50 mm tall above the floor: their tops 4 mm under the opening's top
FRONT = (FACE - 0.012, FACE)  # the drawer front, 12 mm thick, in front of the unit's face when closed
FRONT_Z = (TOP + 0.012, TOP + 0.090)
FRONT_HALF_W = IN_HALF_W + WALL_T  # the drawer front is as wide as the drawer
HANDLE_X = FRONT[0] - 0.030  # a 10 mm bar pull, 100 mm long, standing 30 mm proud of the drawer front
HANDLE_Z = TOP + 0.066  # 24 mm below the front's top edge
HANDLE_R, HANDLE_L = 0.005, 0.100

_fz = (FRONT_Z[0] + FRONT_Z[1]) / 2
_wz = FLOOR_Z + WALL_H / 2
DRAWER = Joint(
    "drawer", kind="slide", axis="X", anchor=(sum(IN_X) / 2, 0.0, FLOOR_Z), lower=-TRAVEL, upper=0.0,
    initial=START, target=START, stiffness=0.0, damping=6.0, friction=4.0, mass=0.6,
    boxes=[
        ((sum(IN_X) / 2, 0.0, FLOOR_Z - 0.0015), (IN_X[1] - IN_X[0], 2 * (IN_HALF_W + WALL_T), 0.003), DRAWER_C),
        ((sum(IN_X) / 2, +(IN_HALF_W + WALL_T / 2), _wz), (IN_X[1] - IN_X[0], WALL_T, WALL_H), DRAWER_C),
        ((sum(IN_X) / 2, -(IN_HALF_W + WALL_T / 2), _wz), (IN_X[1] - IN_X[0], WALL_T, WALL_H), DRAWER_C),
        ((IN_X[1] + WALL_T / 2, 0.0, _wz), (WALL_T, 2 * (IN_HALF_W + WALL_T), WALL_H), DRAWER_C),  # back wall
        ((sum(FRONT) / 2, 0.0, _fz), (FRONT[1] - FRONT[0], 2 * FRONT_HALF_W, FRONT_Z[1] - FRONT_Z[0]), DRAWER_C),
        *[((HANDLE_X / 2 + FRONT[0] / 2, s * 0.045, HANDLE_Z), (FRONT[0] - HANDLE_X, 0.008, 0.008), HANDLE_C)
          for s in (-1, 1)],  # the handle's two posts
    ],
    cylinders=[((HANDLE_X, 0.0, HANDLE_Z), HANDLE_R, HANDLE_L, HANDLE_C, "Y")],
)

_ux = (FACE + BACK) / 2
UNIT = part(
    "unit", pos=(0.0, 0.0, 0.0), fixed=True, friction=0.4,
    boxes=[
        ((_ux, 0.0, TOP + BASE_T / 2), (BACK - FACE, 2 * HALF_W, BASE_T), UNIT_C),  # bottom
        ((_ux, 0.0, TOP + OPEN_H + TOP_T / 2), (BACK - FACE, 2 * HALF_W, TOP_T), UNIT_C),  # top
        *[((_ux, s * (HALF_W - SIDE_T / 2), TOP + (BASE_T + OPEN_H) / 2), (BACK - FACE, SIDE_T, OPEN_H - BASE_T),
           UNIT_C) for s in (-1, 1)],  # sides
        ((BACK - 0.006, 0.0, TOP + (BASE_T + OPEN_H) / 2), (0.012, 2 * HALF_W, OPEN_H - BASE_T), UNIT_C),  # back
    ],
    joints=[DRAWER],
)

# == the organizer tray (loose) ==========================================================================================
TRAY = (0.180, 0.240, 0.045)  # x, y, z outside: 10 mm to spare all round in the drawer
TRAY_T = 0.003
TRAY_KG = 0.100
TRAY_START = (0.250, 0.262)
_tw = TRAY_T
ORGANIZER = part(
    "organizer", pos=(*TRAY_START, TOP), mass=TRAY_KG, friction=0.8,
    boxes=[
        ((0, 0, _tw / 2), (TRAY[0], TRAY[1], _tw), TRAY_C),  # floor
        ((+(TRAY[0] - _tw) / 2, 0, TRAY[2] / 2), (_tw, TRAY[1], TRAY[2]), TRAY_C),
        ((-(TRAY[0] - _tw) / 2, 0, TRAY[2] / 2), (_tw, TRAY[1], TRAY[2]), TRAY_C),
        ((0, +(TRAY[1] - _tw) / 2, TRAY[2] / 2), (TRAY[0], _tw, TRAY[2]), TRAY_C),
        ((0, -(TRAY[1] - _tw) / 2, TRAY[2] / 2), (TRAY[0], _tw, TRAY[2]), TRAY_C),
        ((0, 0, TRAY[2] / 2), (TRAY[0], _tw, TRAY[2]), TRAY_C),  # the long divider down the middle
        *[((0, s * TRAY[1] / 4, TRAY[2] * 0.35), (_tw, TRAY[1] / 2, TRAY[2] * 0.7), TRAY_C) for s in (-1, 1)],
    ],
)
labkit.TORSIONAL_PATCH["organizer"] = 0.012

# == the pen (loose), on the drawer's floor near its back ================================================================
PEN_R, PEN_L, PEN_KG = 0.007, 0.130, 0.015
PEN_BACK = 0.056  # its axis 56 mm in front of the drawer's back wall (47 mm under the unit's top at the start)
PEN_YS = (-0.060, 0.0, 0.060)  # where along the back it lies: chosen by the trial's seed


def pen_spot(seed) -> float:
    rng = np.random.default_rng(None if seed is None else int(seed) + 104729)
    return float(PEN_YS[int(rng.integers(len(PEN_YS)))])


PEN_TURN_DEG = -130.0  # it lies turned onto its clip, as it comes to rest
CLIP = (0.004, 0.035, 0.002)  # the cap's pocket clip, on top of the pen at one end: it keeps the pen from rolling
PEN = part("pen", pos=(IN_X[1] - PEN_BACK + START, 0.0, FLOOR_Z), mass=PEN_KG, friction=0.4,
           cylinders=[((0, 0, PEN_R), PEN_R, PEN_L, PEN_C, "Y")],
           boxes=[((0, PEN_L / 2 - CLIP[1] / 2 - 0.004, 2 * PEN_R + CLIP[2] / 2 - 0.0005), CLIP, (0.30, 0.30, 0.32))])

OBJECTS = [UNIT, ORGANIZER, PEN]

# == live state ==========================================================================================================
_trial = {"t": 0.0, "objs": None, "pen_y": 0.0}
_DEBUG = bool(os.environ.get("DRAWER_DEBUG"))


@on_reset
def _place(env):
    """Lay the pen at the back of the drawer, at this trial's place along it."""
    import torch

    y = pen_spot(labkit.reset_seed())
    base = env.unwrapped
    obj = base.scene["pen"]
    pose = obj.data.default_root_pose.torch.clone()
    # laid down turned onto its clip (130 deg about its axis), the way it comes to rest, its axis PEN_BACK in front
    # of the back wall: the body origin (the barrel's underside before the turn) then sits up and behind the axis
    a = math.radians(PEN_TURN_DEG)
    axis = np.array([IN_X[1] - PEN_BACK + START, y, FLOOR_Z + PEN_R])
    origin = axis + np.array([-PEN_R * math.sin(a), 0.0, -PEN_R * math.cos(a)])
    pose[:, :3] = torch.tensor(origin, dtype=pose.dtype, device=pose.device) + base.scene.env_origins
    pose[:, 3:7] = torch.tensor([0.0, math.sin(a / 2), 0.0, math.cos(a / 2)], dtype=pose.dtype, device=pose.device)
    obj.write_root_pose_to_sim_index(root_pose=pose)
    obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
    _trial.update(t=0.0, objs=None, pen_y=y)
    print(f"[drawer] seed={labkit.reset_seed()} pen at y={y:+.3f}", file=sys.stderr, flush=True)
    return True


@on_step
def _watch(env):
    objs = {n: labkit._part_state(env, labkit._SPECS[n]) for n in ("unit", "organizer", "pen")}
    _trial["objs"] = objs
    _trial["left_tcp"], _trial["left_rot"] = _tcp(env, "left_robot"), _rot(env, "left_robot")
    _trial["right_tcp"] = _tcp(env, "right_robot")
    _trial["t"] += float(getattr(env.unwrapped, "step_dt", 0.05))
    if _DEBUG and int(round(_trial["t"] / 0.05)) % 10 == 0:
        o, p = objs["organizer"], objs["pen"]
        g = _trial.get("left_rot")
        print(f"[drawer] t={_trial['t']:5.1f} L={np.round(_trial['left_tcp'] * 1000).astype(int).tolist()} R={np.round(_trial['right_tcp'] * 1000).astype(int).tolist()} trayz={np.round(o.rotation()[:, 2], 3).tolist()} "
              f"handz={np.round(g[:, 2], 3).tolist() if g is not None else None} drawer={drawer_pos(objs) * 1000:6.1f}mm "
              f"tray={np.round(o.pos * 1000, 1).tolist()} tilt={o.tilt_deg():.2f} "
              f"pen={np.round(p.pos * 1000, 1).tolist()} penq={np.round(p.quat_xyzw, 4).tolist()} penaxis={np.round(p.to_world((0, 0, PEN_R)) * 1000, 1).tolist()} in={in_drawer(objs)} flat={flat(objs)} "
              f"closed={closed(objs)} score={score(objs):.2f}", file=sys.stderr, flush=True)


def _tcp(env, arm: str) -> np.ndarray:
    from yam_arena.constants import TCP_OFFSET

    base = env.unwrapped
    robot = base.scene[arm]
    names = [n.split("/")[-1] for n in robot.body_names]
    g = labkit._np(robot.data.body_link_pose_w)[0][names.index("gripper")]
    st = labkit.PartState(pos=g[:3] - labkit._np(base.scene.env_origins[0]), quat_xyzw=g[3:7], lin_vel=np.zeros(3),
                          ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))
    return st.to_world(TCP_OFFSET)


def _rot(env, arm: str) -> np.ndarray:
    robot = env.unwrapped.scene[arm]
    names = [n.split("/")[-1] for n in robot.body_names]
    g = labkit._np(robot.data.body_link_pose_w)[0][names.index("gripper")]
    return labkit.PartState(pos=g[:3], quat_xyzw=g[3:7], lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.zeros(3),
                            high=np.zeros(3)).rotation()


# == scoring =============================================================================================================
CLOSED_TOL = 0.005  # closed: within 5 mm of fully shut
FLAT_MM, FLAT_DEG = 0.002, 1.5  # flat: the tray's underside within 2 mm of the drawer's floor, tilted under 1.5 deg
IN_TOL = 0.003


def drawer_pos(objs) -> float:
    return float(objs["unit"].joints["drawer"])


def closed(objs) -> bool:
    return drawer_pos(objs) >= -CLOSED_TOL


def in_drawer(objs) -> bool:
    """The tray's whole footprint inside the drawer's walls (3 mm allowance), and down in it, below the walls' tops."""
    d, o = drawer_pos(objs), objs["organizer"]
    lo = np.array([IN_X[0] + d - IN_TOL, -IN_HALF_W - IN_TOL])
    hi = np.array([IN_X[1] + d + IN_TOL, IN_HALF_W + IN_TOL])
    return bool(np.all(o.low[:2] >= lo) and np.all(o.high[:2] <= hi) and o.low[2] <= FLOOR_Z + WALL_H)


def flat(objs) -> bool:
    o = objs["organizer"]
    return bool(o.low[2] <= FLOOR_Z + FLAT_MM and o.tilt_deg() <= FLAT_DEG)


def score(objs) -> float:
    """1.0: the tray in the drawer, flat on its floor, and the drawer closed. 0.5: in and flat, the drawer not
    closed. 0.25: in the drawer but not flat (riding on the pen, or tipped). 0 otherwise."""
    if not in_drawer(objs):
        return 0.0
    if not flat(objs):
        return 0.25
    return 1.0 if closed(objs) else 0.5


SUCCESS_HOLD_S = 2.0


def success(objs):
    return score(objs) >= 1.0 and objs["organizer"].resting()


# == scripted demo =======================================================================================================
# Left hand: pulls the drawer out to its stop by its handle (which uncovers the back of the drawer and the pen there).
# Right hand: picks the pen up where it is seen and lays it on the table to the right. Left hand: lifts the tray by its
# middle divider, lowers it flat into the drawer, then pushes the drawer shut by its handle. The demo reads what a
# camera sees: the pen's and the drawer's positions.

_prev_action7 = _demo.ArmPlan.action7
if not getattr(_prev_action7, "_clock", False):
    def _action7(self, t, _prev=_prev_action7):
        _demo._plan_clock = float(t)
        return _prev(self, t)

    _action7._clock = True
    _demo.ArmPlan.action7 = _action7


class Orient(float):
    """A Waypoint ``yaw`` that carries a full TCP rotation."""

    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj.rot = np.asarray(rot, float)
        return obj


_down_grip = _demo.grip_rotation
if not getattr(_down_grip, "_orient", False):
    def _grip_rotation(yaw=0.0):
        return yaw.rot if isinstance(yaw, Orient) else _down_grip(yaw)

    _grip_rotation._orient = True
    _demo.grip_rotation = _grip_rotation


def _down(axis) -> Orient:
    """Pointing straight down, the fingers closing along ``axis`` (a world unit vector in the table plane)."""
    z, x = np.array([0.0, 0.0, -1.0]), np.asarray(axis, float)
    return Orient(np.column_stack([x, np.cross(z, x), z]))


TIP = 0.013  # the pad tips below the TCP
L_HANDLE = _down([float(v) for v in os.environ.get("DRAWER_LH", "-1,0,0").split(",")])
R_PEN = _down([float(v) for v in os.environ.get("DRAWER_RP", "1,0,0").split(",")])
L_TRAY = _down([float(v) for v in os.environ.get("DRAWER_LT", "0,-1,0").split(",")])
GRIP_HANDLE_Z = HANDLE_Z + TIP - 0.004  # pad tips 4 mm below the bar's axis
WIDE = -0.014  # pads 28 mm apart round the 10 mm bar: the inner one clear of the drawer front, 30 mm behind it
LEFT_PARK = (0.12, 0.30, TOP + 0.18)
RIGHT_PARK = (0.12, -0.30, TOP + 0.18)
PEN_DROP = (0.200, -0.230)


class _Seen:
    """A position read from what the cameras see when the waypoint is reached (evaluated every control step)."""

    def __init__(self, fn):
        self.fn = fn

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.fn(), dtype=dtype or float)


def _drawer_now() -> float:
    o = _trial["objs"]
    return drawer_pos(o) if o is not None else START


def _pen_now() -> np.ndarray:
    o = _trial["objs"]
    if o is None:
        return np.array([IN_X[1] - PEN_BACK - TRAVEL, 0.0, FLOOR_Z + PEN_R])
    return o["pen"].to_world((0.0, 0.0, PEN_R))


def _dur(wps):
    return sum(w.duration for w in wps)


def _wait(wps, t, label):
    last = wps[-1]
    if t > _dur(wps) + 1e-9:
        wps.append(Waypoint(t - _dur(wps), last.pos, last.finger, label, last.yaw))


CHECK = os.environ.get("DRAWER_CHECK", "")
_grip_off = [0.0, 0.0]  # where the hand is relative to the tray's middle, as seen


def _take_pen(above):
    return [
        Waypoint(1.5, above(0.07), FINGER_OPEN, "above the pen", R_PEN),
        Waypoint(1.2, above(-0.002), -0.018, "fingers either side of it", R_PEN),
        Waypoint(0.6, above(-0.002), FINGER_CLOSED, "pick it up", R_PEN),
        Waypoint(1.0, above(0.08), FINGER_CLOSED, "lift it out", R_PEN),
        Waypoint(1.5, (*PEN_DROP, TOP + 0.08), FINGER_CLOSED, "over the table", R_PEN),
        Waypoint(0.8, (*PEN_DROP, TOP + PEN_R + TIP + 0.002), FINGER_CLOSED, "lay it down", R_PEN),
        Waypoint(0.4, (*PEN_DROP, TOP + PEN_R + TIP + 0.002), -0.018, "let go", R_PEN),
        Waypoint(0.8, (*PEN_DROP, TOP + 0.16), FINGER_OPEN, "up", R_PEN),
    ]


def demo():
    _grip_off[:] = [0.0, 0.0]
    left, right = [], []
    hx0, hx1 = HANDLE_X + START, HANDLE_X - TRAVEL  # the handle at the start and pulled out to the stop
    # -- left: pull the drawer out to its stop
    left += [
        Waypoint(2.0, (hx0, 0.0, GRIP_HANDLE_Z + 0.05), FINGER_OPEN, "above the handle", L_HANDLE),
        Waypoint(1.0, (hx0, 0.0, GRIP_HANDLE_Z), WIDE, "fingers either side of the bar", L_HANDLE),
        Waypoint(0.6, (hx0, 0.0, GRIP_HANDLE_Z), FINGER_CLOSED, "grip the handle", L_HANDLE),
        Waypoint(2.5, (hx1 - 0.004, 0.0, GRIP_HANDLE_Z), FINGER_CLOSED, "pull the drawer out to its stop", L_HANDLE),
        Waypoint(0.5, (hx1, 0.0, GRIP_HANDLE_Z), WIDE, "let go", L_HANDLE),
        Waypoint(1.0, (hx1, 0.0, GRIP_HANDLE_Z + 0.06), FINGER_OPEN, "up", L_HANDLE),
    ]
    t_open = _dur(left)
    # -- right: take the pen out (it shows once the drawer is out) and lay it on the table
    right.append(Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "start"))
    _wait(right, t_open - 1.5, "wait for the drawer")
    above = lambda dz: _Seen(lambda: _pen_now() + [0.0, 0.0, TIP + dz])  # noqa: E731
    if CHECK == "leave_pen":  # shortcut check: never take the pen out; the tray goes in on top of it
        right.append(Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "leave the pen where it is (check)"))
    else:
        right += _take_pen(above)
    t_clear = _dur(right)  # the pen is on the table and the right hand up over it
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "clear of the drawer while the pen comes out"))
    # -- left hand: lift the tray by its middle divider, carry it level and set it down flat in the drawer
    _wait(left, t_clear, "wait for the right hand to clear")
    grip_dz = TRAY[2] - 0.010 + TIP  # pads on the divider's top 10 mm (the drawer's walls stand 5 mm above the tray's)
    tx, ty = TRAY_START
    drop = 0.016  # let go with the tray's underside 16 mm up: the pads stay above the drawer's walls
    mid = lambda: sum(IN_X) / 2 + _drawer_now()  # noqa: E731  the drawer's middle, where it now is
    def aim(frozen=False):
        """The hand's (x, y) that puts the tray's middle, where it is seen in the hand, over the drawer's middle. The
        tray's place in the hand is followed through a slow filter, so a tray that slips in the grip is still set
        down square in the drawer."""
        o, tcp = _trial["objs"], _trial.get("left_tcp")
        if o is not None and tcp is not None and not frozen and o["organizer"].pos[2] > TOP + 0.02:
            seen = np.asarray(tcp[:2]) - np.asarray(o["organizer"].pos[:2])
            _grip_off[:] = (0.9 * np.asarray(_grip_off) + 0.1 * seen).tolist()
        return np.array([mid(), 0.0]) + _grip_off

    CARRY_Z = TOP + 0.17  # the tray's underside 120 mm up: clear of the open drawer's front (90 mm) as it passes over
    left += [
        Waypoint(2.0, (tx, ty, TOP + grip_dz + 0.08), FINGER_OPEN, "above the tray's divider", L_TRAY),
        Waypoint(1.0, (tx, ty, TOP + grip_dz), -0.012, "fingers either side of the divider", L_TRAY),
        Waypoint(0.6, (tx, ty, TOP + grip_dz), FINGER_CLOSED, "grip it", L_TRAY),
        Waypoint(3.0, (tx, ty, CARRY_Z), FINGER_CLOSED, "lift the tray, slowly", L_TRAY),
        Waypoint(4.0, _Seen(lambda: (*aim(), CARRY_Z)), FINGER_CLOSED, "carry it over the drawer", L_TRAY),
        Waypoint(1.0, _Seen(lambda: (*aim(), CARRY_Z)), FINGER_CLOSED, "tray's middle over the drawer's (as seen)",
                 L_TRAY),
        Waypoint(2.0, _Seen(lambda: (*aim(), FLOOR_Z + drop + grip_dz)), FINGER_CLOSED, "lower it in level", L_TRAY),
        Waypoint(0.5, _Seen(lambda: (*aim(frozen=True), FLOOR_Z + drop + grip_dz)), -0.012, "let go", L_TRAY),
        Waypoint(1.0, _Seen(lambda: (mid(), 0.0, TOP + 0.14)), FINGER_OPEN, "up", L_TRAY),
    ]
    right.append(Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park"))
    # -- left: push the drawer shut by its handle
    hx = lambda: HANDLE_X + _drawer_now()  # noqa: E731
    left += [
        Waypoint(1.5, _Seen(lambda: (hx(), 0.0, GRIP_HANDLE_Z + 0.05)), FINGER_OPEN, "above the handle", L_HANDLE),
        Waypoint(1.0, _Seen(lambda: (hx(), 0.0, GRIP_HANDLE_Z)), WIDE, "fingers either side of the bar", L_HANDLE),
        Waypoint(0.6, _Seen(lambda: (hx(), 0.0, GRIP_HANDLE_Z)), FINGER_CLOSED, "grip the handle", L_HANDLE),
        Waypoint(3.0, (HANDLE_X + 0.020, 0.0, GRIP_HANDLE_Z), FINGER_CLOSED, "push the drawer shut", L_HANDLE),
        Waypoint(1.0, (HANDLE_X + 0.020, 0.0, GRIP_HANDLE_Z), FINGER_CLOSED, "hold it shut (its stop holds it)",
                 L_HANDLE),
        Waypoint(0.5, (HANDLE_X + 0.002, 0.0, GRIP_HANDLE_Z), WIDE, "let go", L_HANDLE),
        Waypoint(1.0, (HANDLE_X, 0.0, GRIP_HANDLE_Z + 0.06), FINGER_OPEN, "up", L_HANDLE),
        Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "park"),
    ]
    t_end = max(_dur(left), _dur(right)) + 1.0
    _wait(left, t_end, "rest")
    _wait(right, t_end, "rest")
    return {"left": left, "right": right}
