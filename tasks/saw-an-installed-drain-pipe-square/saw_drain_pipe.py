"""On the station table: a 2-inch PVC drain line, fixed in place, with a pencil mark round it and a 1/2-inch PEX water
line below it; and a compact 12 V one-handed reciprocating saw lying on a rest beside it.

The drain line is a run of 2-inch schedule 40 PVC (60.3 mm outside, 3.9 mm wall) lying across two 2x6 blocks on edge,
each fixed down with a galvanized two-hole strap, 0.96 m apart (within the 4 ft a horizontal PVC drain may span between
supports). It runs straight away from the arms, its axis 170 mm above the table, on the arms' centre line a little
to the left (y = 0.06 m). A pencil line goes round it 540 mm beyond the near block's centre (x = 0.42 m). A 1/2-inch
PEX water line (15.9 mm outside) runs the same length below it, its axis 60 mm to the left of the drain's, through a
hole bored in each block, its top 32 mm (1 1/4 in) below the drain's bottom, under the blade's outer half: the blade
breaking through the drain must be caught before it reaches it.

The saw (about 1.55 kg with its 2 Ah battery, 0.43 m long with its blade) lies level, teeth down, across a rest of two
posts to the right of the pipe and beyond the mark, its blade (a 6-inch bi-metal blade) pointing left toward the pipe.
Its grip is behind the motor, with a paddle trigger under the grip; the shoe at its nose stands at the blade's base.

Goal: cut through the pipe at the pencil mark, square to the pipe, with the saw, without the blade touching the water
line below it; then let go of the trigger and lift the blade out of the cut.

Fidelity notes. The pipe, blocks and straps are fixed boxes and cylinders at their real sizes; the saw is built from
boxes and cylinders (yam_parts) at a typical compact saw's size and weight. PhysX cannot cut a solid, so the cut is our
model (cut_model, cut_runtime): the blade's collider passes through the pipe and rests instead on a small hidden pad
the model keeps under the uncut pipe the blade meets first. The kerf's floor sinks only where the blade bears, and
only while the trigger is pulled (a pad pressing it with at least 35 N, from PhysX contacts), the blade lies within
5 degrees of its kerf and the pad pushes back on it with at least 3 N (full speed, 15 mm/s, from 10 N: about 4 s
through the pipe). While cutting, the work drags the blade toward its tip (the pull stroke, averaged), which the shoe
takes. The blade's stroke, vibration and chips are not simulated. A blade pressed on the pipe off its kerf starts a
second kerf. The kerf is shown by dark marks on the pipe's outside as it goes round. The water line is an ordinary
solid the blade meets; the blade touching it is read from PhysX contacts.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import cut_model as CM  # noqa: E402
import cut_runtime as CR  # noqa: E402
import yam_parts  # noqa: E402
from yam_parts import Part, box, cyl  # noqa: E402
from yam_arena.constants import FINGER_OPEN, READY, SHOULDER, WRIST  # noqa: E402
import yam_arena.demo as motion  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INCH = 0.0254
G = 9.81

# -- the drain line ---------------------------------------------------------------------------------------------------
PIPE_R = 2.375 * INCH / 2  # 60.3 mm outside
PIPE_WALL = 0.154 * INCH  # 3.9 mm, schedule 40
BLOCK = (1.5 * INCH, 6.0 * INCH, 5.5 * INCH)  # a 2x6 on edge across the pipe: x, y, z
PIPE_Y = 0.06
PIPE_Z = TOP + BLOCK[2] + PIPE_R  # the axis
PIPE_X0, PIPE_X1 = -0.30, 1.00  # it runs on past the table's ends
BLOCK_X = (-0.12, 0.84)
MARK_X = 0.42  # the pencil line
WHITE, PENCIL, PINE, STEEL_G = (0.93, 0.93, 0.92), (0.22, 0.22, 0.24), (0.80, 0.66, 0.44), (0.62, 0.63, 0.65)
STRAP_W, STRAP_T = 0.019, 0.0012  # a 3/4-inch galvanized strap


def _block(i, x):
    return Box(f"block_{i}", pos=(x, PIPE_Y, TOP + BLOCK[2] / 2), size=BLOCK, color=PINE)


def _strap(i, x):
    """A galvanized strap round the pipe (a band, as a thin ring) and its two feet screwed to the block."""
    s = [cyl((0.0, 0.0, 0.0), PIPE_R + STRAP_T, STRAP_W, "x")]
    for side in (-1, 1):
        s.append(box((0.0, side * (PIPE_R + 0.014), -PIPE_R + STRAP_T / 2), (STRAP_W, 0.026, STRAP_T)))
    return Part(f"strap_{i}", (x, PIPE_Y, PIPE_Z), s, color=STEEL_G)


PIPE = Part("pipe", ((PIPE_X0 + PIPE_X1) / 2, PIPE_Y, PIPE_Z), [cyl((0.0, 0.0, 0.0), PIPE_R, PIPE_X1 - PIPE_X0, "x")],
            mass=2.0, color=WHITE)  # kinematic at spawn: fixed where it is, as its straps hold it
PENCIL_LINE = Part("pencil_line", (MARK_X, PIPE_Y, PIPE_Z), [cyl((0.0, 0.0, 0.0), PIPE_R + 0.0002, 0.0008, "x")],
                   color=PENCIL)  # no collider (at spawn)

# -- the water line below it: 1/2-inch PEX, through a hole bored in each block ----------------------------------------
PEX_R = 0.625 * INCH / 2  # 15.9 mm outside
PEX_GAP = 1.25 * INCH  # its top this far below the drain's bottom (32 mm)
PEX_Z = PIPE_Z - PIPE_R - PEX_GAP - PEX_R  # its axis
PEX_Y = PIPE_Y + 0.060  # 60 mm left of the drain's axis: under the blade's outer half, and in the top camera's view
BLUE = (0.22, 0.52, 0.95)
PEX_LINE = Part("pex_line", ((PIPE_X0 + PIPE_X1) / 2, PEX_Y, PEX_Z),
                [cyl((0.0, 0.0, 0.0), PEX_R, PIPE_X1 - PIPE_X0, "x")], mass=0.2, color=BLUE)  # kinematic at spawn

# -- the saw (part frame: origin on the saw's axis at the nose's front; +x along the blade; teeth toward -z) ----------
# A compact 12 V one-handed reciprocating saw: about 310 mm long without its blade, 1.38 kg bare and 1.55 kg with a
# 2 Ah battery (estimates from typical models), weight-forward; a 6-inch blade, 125 mm of it out past the shoe.
SAW_MASS = 1.55
BLADE_OUT = 0.125
BLADE_H, BLADE_T = 0.019, 0.001
BLADE_BOTTOM = -0.031  # the teeth, below the saw's axis
SHOE_FRONT = 0.004
GRIP_LOCAL = np.array((-0.205, 0.0, -0.003))  # the hand's TCP: pads on the grip's top and the trigger
TRIGGER = ((-0.235, -0.010, -0.028), (-0.175, 0.010, -0.022))  # the paddle trigger under the grip (low, high corners)
GREY_M, BODY, BLACK, RED, BLADE_C = ((0.45, 0.46, 0.48), (0.25, 0.42, 0.55), (0.08, 0.08, 0.08), (0.75, 0.08, 0.08),
                                     (0.72, 0.72, 0.75))
SAW_SHAPES = [
    box((0.002, 0.0, -0.010), (0.004, 0.030, 0.044), GREY_M),  # 0 shoe
    cyl((-0.040, 0.0, 0.0), 0.024, 0.080, "x", GREY_M),  # 1 gear case (nose)
    cyl((-0.125, 0.0, 0.0), 0.030, 0.090, "x", BODY),  # 2 motor housing
    box((-0.225, 0.0, 0.0), (0.110, 0.036, 0.044), BLACK),  # 3 grip
    box(tuple((np.add(TRIGGER[0], TRIGGER[1]) / 2)), tuple(np.subtract(TRIGGER[1], TRIGGER[0])), RED),  # 4 trigger
    box((-0.300, 0.0, -0.004), (0.040, 0.040, 0.050), BLACK),  # 5 battery
    box((BLADE_OUT / 2, 0.0, BLADE_BOTTOM + BLADE_H / 2), (BLADE_OUT, BLADE_T, BLADE_H), BLADE_C),  # 6 blade
]
BLADE_SHAPE = 6
YAW = np.array(((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)))  # the saw's rotation: blade toward +y


def make_saw(pos):
    return Part("saw", pos, SAW_SHAPES, mass=SAW_MASS, color=BODY, yaw=math.pi / 2, friction=(0.8, 0.7),
                contact_offset=0.001, grip=3)


Y_O_CUT = PIPE_Y - PIPE_R - 0.003 - SHOE_FRONT  # the saw's origin with the shoe 3 mm short of the pipe's side
Z_AXIS_CUT = PIPE_Z + PIPE_R - BLADE_BOTTOM  # with the teeth on the pipe's top
SAW_CUT = np.array((MARK_X, Y_O_CUT, Z_AXIS_CUT))
REST_X = MARK_X + 0.13  # the rest beyond the mark: clear of the saw over the pipe, and of the arm reaching for it
Y_O_REST = -0.17
Z_AXIS_REST = Z_AXIS_CUT + 0.03
SAW_REST = np.array((REST_X, Y_O_REST, Z_AXIS_REST))


def _post(name, x, y, top, size_xy):
    h = top - TOP
    return Box(name, pos=(x, y, TOP + h / 2), size=(size_xy[0], size_xy[1], h), color=(0.30, 0.30, 0.32))


SAW = make_saw(tuple(SAW_REST))

# -- the kerf's marks (dark, kinematic, no colliders, 3 mm wide so a camera sees them; the kerf itself is 1.5 mm):
# they wait under the table until the cut passes their point of the circumference ---------------------------------
N_MARKS = 32
MARK_HIDDEN = np.array((0.3, 0.0, TOP - 0.25))
MARKS = tuple(Part(f"kerf_mark_{i:02d}", tuple(MARK_HIDDEN + np.array((0.0, 0.01 * i, 0.0))),
                   [box((0.0, 0.0, 0.0), (0.003, 2 * math.pi * PIPE_R / N_MARKS + 0.0005, 0.0015))], mass=0.001,
                   color=(0.12, 0.12, 0.13)) for i in range(N_MARKS))

# The pad the blade rests on in place of the pipe (cut_model): small, kinematic, kept under the uncut pipe the blade
# meets first; out of the way under the table otherwise.
PLATE_HIDDEN = np.array((0.3, -0.3, TOP - 0.25))
KERF_PLATE = Part("kerf_plate", tuple(PLATE_HIDDEN), [box((0.0, 0.0, 0.0), (0.006, 0.004, 0.004))], mass=0.01,
                  color=WHITE, friction=(0.4, 0.3))  # PVC on steel

OBJECTS = [
    *(_block(i, x) for i, x in enumerate(BLOCK_X)),
    *(_strap(i, x) for i, x in enumerate(BLOCK_X)),
    _post("rest_motor", REST_X, Y_O_REST - 0.125, Z_AXIS_REST - 0.030, (0.05, 0.06)),
    _post("rest_battery", REST_X, Y_O_REST - 0.300, Z_AXIS_REST - 0.029, (0.05, 0.03)),
    PIPE,
    PENCIL_LINE,
    PEX_LINE,
    KERF_PLATE,
    SAW,
    *MARKS,
]

# The cut model's geometry.
CM.CENTER = np.array((0.0, PIPE_Y, PIPE_Z))
CM.AXIS = np.array((1.0, 0.0, 0.0))
CM.UP = np.array((0.0, 0.0, 1.0))
CM.R_OUT = PIPE_R
CM.R_IN = PIPE_R - PIPE_WALL
CM.X_RANGE = (PIPE_X0, PIPE_X1)
CM.MODEL.clear()
CR.EDGE_LOCAL = (np.array((SHOE_FRONT + 0.002, 0.0, BLADE_BOTTOM)), np.array((BLADE_OUT, 0.0, BLADE_BOTTOM)))
CR.NORMAL_LOCAL = np.array((0.0, 1.0, 0.0))
CR.TRIGGER_LOCAL = (np.array((TRIGGER[0][0] - 0.002, TRIGGER[0][1] - 0.002, TRIGGER[0][2] - 0.002)),
                    np.array((TRIGGER[1][0] + 0.002, TRIGGER[1][1] + 0.002, TRIGGER[0][2] + 0.002)))
CR.BLADE_LOCAL = (np.array((-0.001, -BLADE_T / 2 - 0.001, BLADE_BOTTOM - 0.001)),
                  np.array((BLADE_OUT + 0.001, BLADE_T / 2 + 0.001, BLADE_BOTTOM + BLADE_H + 0.001)))
CR.F_TRIGGER = 35.0
CR.MARKS = tuple(m.name for m in MARKS)
CR.MARK_HIDDEN = MARK_HIDDEN
CR.MARK_LIFT = 0.0
CR.PLATE_HIDDEN = PLATE_HIDDEN
CR.PLATE_SIZE = (0.006, 0.004, 0.004)
CR.prepare()

# At spawn (GPU host only): the pipe and the water line are kinematic (straps and blocks hold them); the saw reports its
# contacts (the trigger, the water line), and its blade does not collide with the pipe (the cut model stands in); the
# pencil line and the kerf marks are looks only.
_original_spawn_part = getattr(yam_parts._spawn_part, "_saw_original", yam_parts._spawn_part)
_PATHS = {}


def _spawn_part(prim_path, cfg, *args, **kwargs):
    prim = _original_spawn_part(prim_path, cfg, *args, **kwargs)
    name = getattr(cfg, "part_name", "")
    _PATHS[name] = prim_path
    from pxr import PhysxSchema, Sdf, UsdPhysics

    if name in ("pipe", "pex_line", "kerf_plate"):
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
    elif name == "saw":
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
        blade = prim.GetStage().GetPrimAtPath(f"{prim_path}/shape_{BLADE_SHAPE}")
        pipe_path = _PATHS.get("pipe")
        if pipe_path is None:
            raise RuntimeError("the pipe must be spawned before the saw")
        UsdPhysics.FilteredPairsAPI.Apply(blade).CreateFilteredPairsRel().SetTargets([Sdf.Path(pipe_path)])
    elif name == "pencil_line" or name.startswith("kerf_mark_"):
        for child in prim.GetChildren():
            if child.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr().Set(False)
        if name.startswith("kerf_mark_"):
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
    return prim


_spawn_part._saw_original = _original_spawn_part
yam_parts._spawn_part = _spawn_part
yam_parts._CLONED_SPAWN_PART = None

# -- scoring ----------------------------------------------------------------------------------------------------------
# A cut off the mark or out of square is a wasted pipe, and a blade that touches the water line nicks it: either scores
# 0. On the mark and square, the cut scores 0.8 times the share of the pipe's circumference it has gone round (a
# flexible tape round the kerf), and the last 0.2 once it is through, the trigger let go and the blade lifted out of
# the cut, the saw still in hand or on its rest.
MARK_TOLERANCE = 0.003  # the kerf's middle from the pencil line, along the pipe (a tape from the near block)
SQUARE_TOLERANCE = math.radians(2.0)  # a square against the cut end: 2 mm of light over the pipe's 60 mm
EXTRA_DEPTH = 0.001  # a second kerf this deep is a second cut
CUT_CREDIT = 0.8
FINISH_CREDIT = 0.2
CLEAR = 0.010  # the blade's teeth this far above the pipe's top, or wholly beside it, once lifted out
DROPPED_Z = TOP + 0.03  # a saw whose lowest point is this low has been put down on the table (or dropped)
PEX_TOUCH = 0.01  # N: any push of the blade on the water line is a touch (a running blade nicks PEX)
TEST_TRIGGER = None  # offline fixtures set whether the trigger is pulled
TEST_PEX = None  # and whether the blade has touched the water line


def _finite(state) -> bool:
    return all(np.all(np.isfinite(np.asarray(v, float))) for v in (state.pos, state.quat_xyzw))


def _rotation(state) -> np.ndarray:
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                     (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def _saw_pose(state):
    """The saw's frame origin and rotation (the state's origin if it has one, as yam_parts' PartState does)."""
    R = _rotation(state)
    origin = getattr(state, "origin", None)
    if origin is None:  # an ObjectState: pos is the bounding-box centre
        return np.asarray(state.pos, float), R
    return np.asarray(origin, float), R


def _valid(objects) -> bool:
    st = objects.get("saw")
    if st is None or not _finite(st):
        return False
    q = np.asarray(st.quat_xyzw, float)
    return abs(float(np.linalg.norm(q)) - 1.0) < 1e-3


def _kerfs():
    """(the cut, whether there is a second cut)."""
    kerfs = sorted(CM.MODEL.kerfs, key=lambda k: -k.depth())
    if not kerfs:
        return None, False
    return kerfs[0], any(k.depth() >= EXTRA_DEPTH for k in kerfs[1:])


def _good(kerf) -> bool:
    return abs(kerf.x() - MARK_X) <= MARK_TOLERANCE and kerf.squareness() <= SQUARE_TOLERANCE


def _pulled() -> bool:
    return CR.STATE["pulled"] if TEST_TRIGGER is None else bool(TEST_TRIGGER)


def _pex_touched() -> bool:
    """Whether the blade has touched the water line at any time in the trial."""
    return CR.STATE["pex"] >= PEX_TOUCH if TEST_PEX is None else bool(TEST_PEX)


def _blade_clear(objects) -> bool:
    """The blade out of the cut: both ends of its edge above the pipe's top by CLEAR, or both off to one side."""
    p, R = _saw_pose(objects["saw"])
    ends = [p + R @ e for e in CR.EDGE_LOCAL]
    if all(q[2] >= PIPE_Z + PIPE_R + CLEAR for q in ends):
        return True
    side = [q[1] - PIPE_Y for q in ends]
    return all(abs(d) > PIPE_R + 0.002 for d in side) and side[0] * side[1] > 0


def _saw_up(objects) -> bool:
    st = objects["saw"]
    return float(np.asarray(st.low, float)[2]) >= DROPPED_Z


def score(objects):
    if not _valid(objects) or _pex_touched():
        return 0.0
    kerf, extra = _kerfs()
    if kerf is None or extra or not _good(kerf):
        return 0.0
    s = CUT_CREDIT * kerf.round_share()
    if kerf.through() and not _pulled() and _blade_clear(objects) and _saw_up(objects):
        s += FINISH_CREDIT
    return float(min(1.0, s))


def success(objects):
    return score(objects) >= 1.0 - 1e-9


def distance(objects):
    if not _valid(objects):
        return 1.0
    p, _ = _saw_pose(objects["saw"])
    return float(np.linalg.norm(p - SAW_CUT))


# -- the scripted demo ------------------------------------------------------------------------------------------------
# The right hand takes the saw by its grip from behind (pads closing on the grip's top and on the trigger), lifts it
# off its rest and carries it over the pipe, the blade 70 mm above it, reading the saw's pose to square it over the mark.
# The arm's joints are springs (shoulder 80 N m/rad, wrist 10) with no feed-forward for a load, so the 1.55 kg saw
# would sag the hand some 15 cm and 20 degrees: every pose the hand holds the saw at is led by K^-1 J^T W (the saw's
# weight and its moment about the grip, and the push on the blade while cutting).
class GripOrientation(float):
    """A full TCP rotation carried through a waypoint's yaw (as the lever-set task does)."""

    def __new__(cls, matrix):
        closure = np.asarray(matrix, float)[:, 0]
        obj = super().__new__(cls, math.atan2(-closure[0], closure[1]))
        obj.matrix = np.asarray(matrix, float).copy()
        return obj


_original_grip_rotation = getattr(motion.grip_rotation, "_original", motion.grip_rotation)


def grip_rotation(yaw=0.0):
    matrix = getattr(yaw, "matrix", None)
    if isinstance(matrix, np.ndarray) and matrix.shape == (3, 3):
        return matrix
    return _original_grip_rotation(yaw)


grip_rotation._original = _original_grip_rotation
motion.grip_rotation = grip_rotation


def side_rotation(sign):
    """Approach +x (from the arms' side), pads closing along z; the wrist camera 7 cm to +y for sign +1, -y for -1."""
    z = np.array((1.0, 0.0, 0.0))
    x = np.array((0.0, 0.0, sign))
    return np.column_stack((x, np.cross(z, x), z))


R_RIGHT = side_rotation(-1.0)
JOINT_K = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
GRIP_HALF = 0.025  # the grip's top (+22 mm) to the trigger's face (-28 mm), about its middle (-3 mm)
HOLD = -(GRIP_HALF - 0.004)  # pads 4 mm in: ~20 N, the trigger at ~27 N with the saw's weight on it (under 35)
SQUEEZE = -(GRIP_HALF - 0.009)  # pads 9 mm in: ~45 N, the trigger pulled
MG = SAW_MASS * G
COM_Y = float(make_saw((0.0, 0.0, 0.0)).volume_centroid()[0] - GRIP_LOCAL[0])  # ahead of the grip (uniform density)
BLADE_Y = (PIPE_Y - Y_O_CUT) - GRIP_LOCAL[0]  # the blade's bite on the pipe's top, ahead of the grip
FEED = 8.0  # N: the push on the blade while cutting (the cut runs at ~11 mm/s)
FEED_LAST = 4.0  # N: lighter through the last wall, so the saw does not lunge when it breaks through
FEED_START = 4.0  # N: gently until the teeth are 3 mm in, so the blade does not skate off its line
FOLLOW_S = 12.0  # s: the plan follows the cut down over this (a little slower than it cuts)
FOLLOW_STEP = 0.05  # s: each step of it (a control step), worked out as it starts: a breakthrough is caught at once
CUT_DEPTH = 2 * PIPE_R + 0.006  # how far the plan follows the cut down (just past the pipe's bottom)
READ_GAIN = 0.7
MAX_SHIFT = 0.05
MAX_TURN = math.radians(12.0)
CARRY_CLEAR = 0.07  # the blade above the pipe's top while carried


def hand(saw_origin, local):
    return np.asarray(saw_origin, float) + YAW @ np.asarray(local, float)


def _jacobian(q, base):
    p0, R0 = ik.tcp_pose(q, base)
    J = np.zeros((6, 6))
    for k in range(6):
        dq = np.zeros(6)
        dq[k] = 1e-6
        p1, R1 = ik.tcp_pose(q + dq, base)
        J[:3, k] = (p1 - p0) / 1e-6
        J[3:, k] = ik._rot_error(R1, R0) / 1e-6
    return J


def pushing(arm, pos, R, wrench):
    """The TCP pose to command so the joint springs exert ``wrench`` (force, torque about the TCP, world) on what the
    hand holds while it stays at (pos, R): q + K^-1 J^T W."""
    base = np.asarray(station.BASES[arm], float)
    q, _, _ = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(R), None, base_pos=base, iters=200)
    q_cmd = np.clip(q + _jacobian(q, base).T @ np.asarray(wrench, float) / JOINT_K, ik.LOWER, ik.UPPER)
    return ik.tcp_pose(q_cmd, base)


def W(d, pos, finger, label, R):
    return Waypoint(d, tuple(float(v) for v in pos), finger, label, GripOrientation(R))


class Lazy(motion.Waypoint):
    """A waypoint worked out when its segment starts, so it can use a read (as the stud and lever-set tasks do)."""

    def __init__(self, duration, where, finger, label):
        super().__init__(duration, (0.0, 0.0, 0.0), finger, label, 0.0)
        self._where, self._resolved = where, None

    def _resolve(self):
        if self._resolved is None:
            p, R = self._where()
            self._resolved = (tuple(float(v) for v in p), GripOrientation(R))
        return self._resolved

    @property
    def pos(self):
        return self._resolve()[0]

    @pos.setter
    def pos(self, value):
        pass

    @property
    def yaw(self):
        return self._resolve()[1]

    @yaw.setter
    def yaw(self, value):
        pass


def _turn_angle(R):
    return math.acos(max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0)))


def _read_saw():
    """(saw origin, rotation, right TCP) on the simulator now, or None offline."""
    env = CR.STATE.get("env")
    if env is None:
        return None
    from yam_arena.constants import ARM_JOINTS

    base = env.unwrapped
    origin = CR._np(base.scene.env_origins)[0][:3]
    pose = CR._np(base.scene.rigid_objects["saw"].data.root_link_pose_w)[0]
    art = base.scene["right_robot"]
    q = CR._np(art.data.joint_pos)[0]
    ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
    tcp, _ = ik.tcp_pose(np.asarray(q)[ids], station.BASES["right"])
    return pose[:3] - origin, CR._quat_matrix(pose[3:7]), np.asarray(tcp, float)


def demo():
    grip_rest = hand(SAW_REST, GRIP_LOCAL)
    fix = {"dp": np.zeros(3), "R": np.eye(3)}
    w_hold = np.array((0.0, 0.0, MG, COM_Y * MG, 0.0, 0.0))  # the hand carrying the saw's weight and moment

    def target(saw_origin):
        return hand(saw_origin, GRIP_LOCAL) + fix["dp"], fix["R"] @ R_RIGHT

    def cmd(saw_origin, feed=0.0):
        def where():
            p, R = target(saw_origin)
            return pushing("right", p, R, w_hold + np.array((0.0, 0.0, -feed, -BLADE_Y * feed, 0.0, 0.0)))

        return where

    def read(saw_origin, label):
        def where():
            st = _read_saw()
            if st is not None:
                p_s, R_s, tcp = st
                full = YAW @ R_s.T  # turns the saw square
                angle = _turn_angle(full)
                share = min(READ_GAIN, MAX_TURN / angle) if angle > 1e-9 else READ_GAIN
                E = ik.slerp_matrix(np.eye(3), full, share)
                moved = tcp + E @ (p_s - tcp)  # the saw's origin once the hand turns by E about the TCP
                fix["dp"] = np.clip(fix["dp"] + READ_GAIN * (np.asarray(saw_origin) - moved), -MAX_SHIFT, MAX_SHIFT)
                fix["R"] = E @ fix["R"]
                err = (p_s - np.asarray(saw_origin)) * 1000
                print(f"SAW_READ {label}: saw off by x {err[0]:+.1f} y {err[1]:+.1f} z {err[2]:+.1f} mm, "
                      f"turned {math.degrees(angle):.1f} deg", flush=True)
            return cmd(saw_origin)()

        return where

    lifted = SAW_REST + np.array((0.0, 0.0, 0.04))
    above = SAW_CUT + np.array((0.0, 0.0, CARRY_CLEAR))
    near = SAW_CUT + np.array((0.0, 0.0, 0.010))
    bottom = SAW_CUT - np.array((0.0, 0.0, CUT_DEPTH))
    right = [
        W(2.5, grip_rest - np.array((0.09, 0.0, 0.0)), FINGER_OPEN, "behind the saw's grip", R_RIGHT),
        W(1.5, grip_rest, FINGER_OPEN, "pads round the grip", R_RIGHT),
        W(1.2, grip_rest, HOLD, "hold the grip (trigger not pulled)", R_RIGHT),
        W(0.8, grip_rest, HOLD, "settle", R_RIGHT),
        Lazy(1.5, cmd(SAW_REST), HOLD, "take the saw's weight"),
        Lazy(0.5, cmd(SAW_REST), HOLD, "settle"),
        Lazy(2.0, cmd(lifted), HOLD, "lift it off its rest"),
        Lazy(0.8, cmd(lifted), HOLD, "settle"),
        Lazy(1.0, read(lifted, "lifted"), HOLD, "square it up"),
        Lazy(0.6, cmd(lifted), HOLD, "settle"),
        Lazy(5.0, cmd(above), HOLD, "carry it over the pipe"),
        Lazy(1.0, cmd(above), HOLD, "settle"),
    ]
    for k in range(2):
        right += [Lazy(1.0, read(above, f"over the mark {k + 1}"), HOLD, "line the blade up over the mark"),
                  Lazy(0.6, cmd(above), HOLD, "settle")]
    right += [Lazy(2.0, cmd(near), HOLD, "down to 10 mm over the pipe"), Lazy(0.6, cmd(near), HOLD, "settle")]
    for k in range(2):
        right += [Lazy(1.0, read(near, f"near {k + 1}"), HOLD, "line it up again"),
                  Lazy(0.6, cmd(near), HOLD, "settle")]
    # The saw is started clear of the work, as a reciprocating saw should be, and lined up again running (the harder
    # squeeze shifts it a little in the hand); then the running blade is brought down onto the mark.
    right += [Lazy(1.2, cmd(near), SQUEEZE, "pull the trigger, the blade clear of the pipe"),
              Lazy(0.6, cmd(near), SQUEEZE, "settle")]
    for k in range(2):
        right += [Lazy(1.0, read(near, f"running {k + 1}"), SQUEEZE, "line the running blade up"),
                  Lazy(0.6, cmd(near), SQUEEZE, "settle")]
    # The running blade is set on the mark first, carried as before (pressed into free air, the press would move the
    # hand instead), then leaned on, and followed down a little slower than it cuts.
    touch = SAW_CUT - np.array((0.0, 0.0, 0.001))
    hover = SAW_CUT + np.array((0.0, 0.0, 0.002))
    right += [
        Lazy(1.5, cmd(hover), SQUEEZE, "the running blade just over the mark"),
        Lazy(0.6, cmd(hover), SQUEEZE, "settle"),
        Lazy(1.0, read(hover, "over the mark, running"), SQUEEZE, "line it up a last time"),
        Lazy(0.6, cmd(hover), SQUEEZE, "settle"),
        Lazy(0.8, cmd(touch), SQUEEZE, "set the running blade on the mark"),
        Lazy(2.0, cmd(touch, FEED_START), SQUEEZE, "lean on the blade, gently to start the cut"),
    ]
    # Followed down a control step at a time, each step worked out as it starts: once the cut is through, the hand
    # stops pushing and brings the saw back to just under the pipe (as one catches a saw breaking through), within
    # 50 ms; the blade drops about 21 mm below the drain, short of the water line 32 mm below. The last wall is fed
    # lighter. (Re-planned only every quarter second, the saw dropped 87 mm.)
    caught = {}

    def following(goal):
        def where():
            k = CM.MODEL.main_kerf()
            if "at" not in caught and k is not None and k.through():
                caught["at"] = through_hold  # back up to just under the pipe, carried as before the cut
            if "at" in caught:
                return cmd(caught["at"])()
            started = k is not None and k.depth() > 0.003
            light = k is not None and k.depth() > 2 * PIPE_R - 0.008
            return cmd(goal, FEED_LAST if light else (FEED if started else FEED_START))()

        return where

    through_hold = SAW_CUT - np.array((0.0, 0.0, 2 * PIPE_R + 0.004))
    steps = int(round(FOLLOW_S / FOLLOW_STEP))
    for i in range(1, steps + 1):
        right.append(Lazy(FOLLOW_STEP, following(touch + (bottom - touch) * i / steps), SQUEEZE,
                          "follow the cut down through the pipe"))
    right += [
        *[Lazy(FOLLOW_STEP, following(bottom), SQUEEZE, "finish the cut")
          for _ in range(int(round(1.5 / FOLLOW_STEP)))],
        Lazy(1.0, lambda: cmd(caught.get("at", bottom))(), SQUEEZE, "ease off"),
        Lazy(1.2, lambda: cmd(caught.get("at", bottom))(), HOLD, "let go of the trigger"),
        Lazy(3.0, cmd(above), HOLD, "lift the blade out of the cut"),
        Lazy(2.0, cmd(above), HOLD, "hold the saw clear"),
    ]
    total = sum(w.duration for w in right)
    ready_p, ready_R = ik.tcp_pose(np.array(READY, float), np.asarray(station.BASES["left"], float))
    left = [W(total, ready_p, FINGER_OPEN, "the left arm stays clear", ready_R)]
    return {"left": left, "right": right}
