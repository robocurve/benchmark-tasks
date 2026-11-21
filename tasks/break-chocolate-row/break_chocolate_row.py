"""On the station table, 30 cm out and 3.6 cm left of the midline: a white dinner plate (27 cm across, 2 cm tall). On it lies a
milk chocolate bar of 12 pieces, 14.5 cm long and 5.7 cm wide, 0.7 cm thick, embossed side up, its length running
left-right (between the arms). Its pieces are in 4 rows of 3; a groove runs between every two neighbouring pieces, and
each groove is as weak as the others. Break one row of three off one end of the bar, keeping the other nine pieces
together, and leave both on the plate.
"""

import importlib.abc
import importlib.util
import math
import os
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena import demo as yam_demo
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder
from yam_arena.urdf import axis_angle_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the plate and the bar --------------------------------------------------------------------------------------------
# The bar: a standard 1.55 oz (43 g) 12-piece milk chocolate bar. Its size here is a draft (web sources disagree) until
# a bought bar is measured: 14.5 x 5.7 x 0.7 cm, 4 rows of 3.
PLATE_R, PLATE_H, PLATE_KG = 0.135, 0.02, 0.6
# The bar's end groove (between its right end row and the next) lies on the midline, between the arms.
PLATE_XY = (0.30, 0.03625)
PLATE_TOP = TOP + PLATE_H
ROWS, COLS = 4, 3  # rows along y (the bar's length), pieces in a row along x (its width)
BAR_L, BAR_W, BAR_T = 0.145, 0.057, 0.007
PIECE = (BAR_W / COLS, BAR_L / ROWS, BAR_T)  # x, y, z: 19.0 x 36.25 x 7 mm
REAL_PIECE_KG = 0.043 / (ROWS * COLS)  # 3.6 g
# WHY ten times heavier than real: at 3.6 g, pieces held by joints slid up to 6 mm apart under finger pads pressing
# with a few newtons (official sim, 2026-11-19); light bodies between stiff contacts are a known PhysX weakness. The
# row weighs 108 g here, about 1 N, small beside the ~7 N lift that snaps the groove.
PIECE_KG = 10 * REAL_PIECE_KG
BROWN = (0.36, 0.20, 0.12)
DARK_BROWN = (0.27, 0.14, 0.08)  # alternate pieces, so the grooves between them show on camera (real ones show by shadow)
REST_GAP = 0.0005  # the bar starts this far above the plate and settles onto it

# Groove strength. A groove breaks when bending it passes GROOVE_TORQUE (N m) for the bar's full width. Planning
# estimate until measured on real bars: about 0.12 N m (a 20-35 N three-point bend snap force over a 50 mm span for
# the full section, scaled to a 3 mm groove in a 7 mm bar). Each joint carries the share of its groove it spans, so
# the strength per metre of groove is the same everywhere.
GROOVE_TORQUE = 0.12
STRENGTH_PER_M = GROOVE_TORQUE / BAR_W
# How a groove breaks. WHY not PhysX's own joint breaking: it uses one torque limit whichever way a joint is loaded,
# so a gentle pinch across the bar's width (which squeezes the bar in its own plane, where real chocolate is far
# stronger) broke the joints before any lift; and no JOINT_BREAK event reached the task in a headless run (official
# sim, 2026-11-19). So each groove is a hinge about its own line, held straight by a spring and rigid in every other
# direction, and the task breaks it (switches the joint off) once it has bent past BEND_BREAK_DEG, which takes its
# share of GROOVE_TORQUE. Chocolate is brittle: it snaps at a small bend (the angle is an estimate).
BEND_BREAK_DEG = 1.5


def name(i: int, k: int) -> str:
    return f"piece_{i}_{k}"


def centre(i: int, k: int) -> np.ndarray:
    return np.array([PLATE_XY[0] + (k - (COLS - 1) / 2) * PIECE[0], PLATE_XY[1] + (i - (ROWS - 1) / 2) * PIECE[1],
                     PLATE_TOP + REST_GAP + BAR_T / 2])


NAMES = [name(i, k) for i in range(ROWS) for k in range(COLS)]
CENTRES = {name(i, k): centre(i, k) for i in range(ROWS) for k in range(COLS)}
# Neighbouring pieces and the groove length each joint between them spans (m).
PAIRS = [(name(i, k), name(i + 1, k), PIECE[0]) for i in range(ROWS - 1) for k in range(COLS)] + \
        [(name(i, k), name(i, k + 1), PIECE[1]) for i in range(ROWS) for k in range(COLS - 1)]


def groove_axis(a: str, b: str) -> str:
    """The line a joint's groove runs along, which it bends about: across the bar (x) between rows, along it (y)
    between the pieces of a row."""
    return "X" if a.rsplit("_", 1)[1] == b.rsplit("_", 1)[1] else "Y"


def joint_stiffness(length: float) -> float:
    """N m per rad: the joint's share of the groove's strength at the breaking bend."""
    return STRENGTH_PER_M * length / math.radians(BEND_BREAK_DEG)
END_ROWS = [frozenset(name(i, k) for k in range(COLS)) for i in (0, ROWS - 1)]


@dataclass(frozen=True)
class BarPiece(Box):
    """A piece of the bar: an ordinary free box. The last one listed also authors every joint of the bar when it is
    spawned (by then the others exist)."""

    author_joints: bool = False


OBJECTS = [
    Cylinder("plate", (*PLATE_XY, TOP + PLATE_H / 2), color=(0.95, 0.95, 0.93), mass=PLATE_KG, radius=PLATE_R,
             height=PLATE_H),
    *[BarPiece(name(i, k), tuple(float(v) for v in CENTRES[name(i, k)]),
               color=BROWN if (i + k) % 2 == 0 else DARK_BROWN, mass=PIECE_KG, size=PIECE,
               author_joints=(name(i, k) == NAMES[-1])) for i in range(ROWS) for k in range(COLS)],
]


# -- the simulator side: the bar's breakable joints, and the grippers for scoring ------------------------------------
def _author_joints(stage, parent: str) -> None:
    """One hinge per neighbouring pair, at the middle of their shared face, about its groove's line, with a spring that
    holds it straight (its stiffness from the groove's strength). Plain rigid-body joints, not an articulation."""
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    # WHY more solver iterations: the pieces weigh 3.6 g and the finger pads press with newtons, so with the default
    # iterations the joints stretch under a pinch.
    for n in NAMES:
        body = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(f"{parent}/{n}"))
        body.CreateSolverPositionIterationCountAttr(64)
        body.CreateSolverVelocityIterationCountAttr(4)
    UsdGeom.Xform.Define(stage, f"{parent}/bar_joints")
    for a, b, length in PAIRS:
        pa, pb = f"{parent}/{a}", f"{parent}/{b}"
        for p in (pa, pb):
            if not stage.GetPrimAtPath(p).IsValid():
                raise RuntimeError(f"break_chocolate_row: {p} doesn't exist when the bar's joints are made")
        mid = (CENTRES[a] + CENTRES[b]) / 2
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"{parent}/bar_joints/{a}__{b}")
        joint.CreateAxisAttr(groove_axis(a, b))
        joint.CreateLowerLimitAttr(-20.0)  # degrees
        joint.CreateUpperLimitAttr(20.0)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        per_degree = math.degrees(1.0)  # USD gives angular drive gains per degree
        drive.CreateStiffnessAttr(joint_stiffness(length) / per_degree)
        drive.CreateDampingAttr(0.0005 / per_degree)
        drive.CreateTargetPositionAttr(0.0)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(pa)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(pb)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*(mid - CENTRES[a]).tolist()))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*(mid - CENTRES[b]).tolist()))
        joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


def _last_piece_spawner():
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils import clone

    @clone
    def spawn_last_piece(prim_path, cfg, translation=None, orientation=None, **kwargs):
        prim = spawn_cuboid.__wrapped__(prim_path, cfg, translation, orientation, **kwargs)
        parent = prim_path.rsplit("/", 1)[0]
        _author_joints(prim.GetStage(), parent)
        _JOINTS_AT[0] = f"{parent}/bar_joints"
        return prim

    return spawn_last_piece


# The joints broken this trial, by name ("piece_0_1__piece_1_1"); cleared, and the joints switched back on, at every
# reset.
_BROKEN: set = set()
_BROKEN_BY: dict = {}  # joint -> "bend" or "apart", for the debug log
_JOINTS_AT = [""]


def bend_deg(states, a: str, b: str) -> float:
    """How far the groove between a and b is bent (degrees, about its own line): b's turn relative to a."""
    r = _rot(states[a].quat_xyzw).T @ _rot(states[b].quat_xyzw)
    if groove_axis(a, b) == "X":
        return math.degrees(math.atan2(r[2, 1], r[1, 1]))
    return math.degrees(math.atan2(r[0, 2], r[0, 0]))


def _set_enabled(joint: str, on: bool) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    prim = get_current_stage().GetPrimAtPath(f"{_JOINTS_AT[0]}/{joint}")
    if prim.IsValid():
        UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(on)


def _joint_record(states) -> "BarJoints":
    """Break every joint whose groove has bent past BEND_BREAK_DEG, or whose pieces have come apart (it stays broken),
    and report the broken ones."""
    for a, b, _ in PAIRS:
        joint = f"{a}__{b}"
        if joint in _BROKEN or a not in states or b not in states:
            continue
        # bent past the breaking bend, or given way some other way (the pieces apart): broken
        bent = abs(bend_deg(states, a, b)) > BEND_BREAK_DEG
        if bent or not joined(states, a, b):
            _BROKEN_BY[joint] = "bend" if bent else "apart"
            _BROKEN.add(joint)
            _set_enabled(joint, False)
    return BarJoints(frozenset(_BROKEN))


def _remake_broken_joints() -> None:
    """After a reset has put the pieces back: switch the broken joints back on."""
    broken = set(_BROKEN)
    _BROKEN.clear()
    _BROKEN_BY.clear()
    if _JOINTS_AT[0]:
        for joint in sorted(broken):
            _set_enabled(joint, True)


@dataclass(frozen=True)
class BarJoints:
    """Which of the bar's joints have broken this trial (for scoring)."""

    broken: frozenset = frozenset()


@dataclass(frozen=True)
class Pads:
    """One gripper after a control step (world frame, m): the span along its pointing axis from the pads' back to
    their tips, and points across the end of each of its two fingertips."""

    back: np.ndarray
    tips: np.ndarray
    fingertips: tuple = ()


PAD_BACK = 0.025  # the pads reach this far behind the TCP
GRIPPERS = ("left_gripper", "right_gripper")
# The end of each fingertip in its own link's frame (m), at its middle and 6 mm either side, from the gripper's meshes
# (as in the accepted flashlight tasks).
FINGERTIP_ENDS = {
    "tip_left": [(x, -0.04454, -0.09016) for x in (0.01799, 0.02399, 0.02999)],
    "tip_right": [(x, 0.04454, -0.09016) for x in (-0.02999, -0.02399, -0.01799)],
}


def _gripper_states(env) -> dict:
    from yam_arena import runtime

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = {}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        poses = runtime.link_poses(robot)
        axis = poses["gripper"][:3, 2]
        tcp = runtime.tcp_world(robot) - origin
        ends = tuple(poses[link][:3, :3] @ np.array(p) + poses[link][:3, 3] - origin
                     for link, points in FINGERTIP_ENDS.items() for p in points)
        out[f"{side}_gripper"] = Pads(back=tcp - PAD_BACK * axis, tips=tcp + PAD_TIP_PAST_TCP * axis, fingertips=ends)
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_chocolate_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        obj = stock_object(spec)
        if getattr(spec, "author_joints", False):
            obj.spawner_cfg.func = _last_piece_spawner()
        return obj

    def object_states(env, names):
        states = stock_states(env, names)
        return {**states, **_gripper_states(env), "bar_joints": _joint_record(states)}

    scene.task_object, scene.object_states = task_object, object_states
    scene._chocolate_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_chocolate_patched", False):
        return
    reset = cls.reset

    def patched_reset(self, *args, **kwargs):
        observation = reset(self, *args, **kwargs)
        _remake_broken_joints()
        return observation

    cls.reset = patched_reset
    cls._chocolate_patched = True


_PATCHES = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}


class _AfterImport(importlib.abc.MetaPathFinder):
    """Patches ``yam_arena.scene`` and ``yam_arena.inspect_embodiment`` right after each is first imported (they need
    Isaac Sim, which starts later)."""

    def find_spec(self, fullname, path, target=None):
        if fullname not in _PATCHES:
            return None
        for finder in sys.meta_path:
            if finder is self or not hasattr(finder, "find_spec"):
                continue
            spec = finder.find_spec(fullname, path, target)
            if spec is not None:
                break
        else:
            return None
        if spec.loader is None:
            return None
        run, patch = spec.loader.exec_module, _PATCHES[fullname]

        def exec_module(module):
            run(module)
            patch(module)

        spec.loader.exec_module = exec_module
        return spec


for _name, _patch in _PATCHES.items():
    if _name in sys.modules:
        _patch(sys.modules[_name])
if not any(isinstance(f, _AfterImport) for f in sys.meta_path):
    sys.meta_path.insert(0, _AfterImport())


# -- scoring: a function of the object states (the bar's broken joints, the pieces, the plate and the grippers) -------
# Two pieces are joined while the joint between them hasn't broken. WHY not from the pieces' poses alone: broken
# pieces lying on the plate settle back into their places, and a cracked bar then looked whole (official sim,
# 2026-11-19: a bar the pinch had broken into four parts read as an end row and nine). The poses are only a fallback
# when the states carry no joint record: pieces apart by more than 1.5 mm or 4 degrees.
JOINED_MM, JOINED_DEG = 1.5, 4.0
CLEAR = 0.01  # hands off: no point along either gripper's pads or fingertips within 1 cm of a piece
COMPLETE_SCORE = 0.9  # score() for the right break; 1.0 comes only from success() holding 1 s, which ends the trial


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def joined(objects, a: str, b: str) -> bool:
    ra, rb = _rot(objects[a].quat_xyzw), _rot(objects[b].quat_xyzw)
    offset = ra.T @ (np.asarray(objects[b].pos, float) - np.asarray(objects[a].pos, float))
    off_mm = np.linalg.norm(offset - (CENTRES[b] - CENTRES[a])) * 1000
    turn = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(ra.T @ rb) - 1) / 2))))
    return off_mm <= JOINED_MM and turn <= JOINED_DEG


def parts(objects) -> list[frozenset]:
    """The bar's pieces grouped into parts that are still joined."""
    root = {n: n for n in NAMES}

    def find(n):
        while root[n] != n:
            root[n] = root[root[n]]
            n = root[n]
        return n

    record = objects.get("bar_joints")
    for a, b, _ in PAIRS:
        intact = f"{a}__{b}" not in record.broken if record is not None else joined(objects, a, b)
        if intact:
            root[find(a)] = find(b)
    groups: dict[str, set] = {}
    for n in NAMES:
        groups.setdefault(find(n), set()).add(n)
    return sorted((frozenset(g) for g in groups.values()), key=len)


def on_plate(objects, n: str) -> bool:
    """A piece's middle over the plate and not below its top (a piece on the table or the floor is off it)."""
    plate = objects["plate"]
    p = np.asarray(objects[n].pos, float)
    centre_xy = (np.asarray(plate.low, float)[:2] + np.asarray(plate.high, float)[:2]) / 2
    return bool(np.linalg.norm(p[:2] - centre_xy) <= PLATE_R and p[2] >= float(plate.high[2]) - 0.002)


def _touches(pads: Pads, piece) -> bool:
    lo, hi = np.asarray(piece.low, float) - CLEAR, np.asarray(piece.high, float) + CLEAR
    along = [pads.back + s * (pads.tips - pads.back) for s in np.linspace(0.0, 1.0, 11)]
    return any(bool(np.all(p >= lo) and np.all(p <= hi)) for p in (*along, *(np.asarray(q, float) for q in pads.fingertips)))


def hands_off(objects) -> bool:
    pads = [objects.get(g) for g in GRIPPERS]
    if any(p is None for p in pads):
        return False
    return not any(_touches(p, objects[n]) for p in pads for n in NAMES)


def grade(objects) -> float:
    """1.0: exactly two parts, an end row of three and the other nine, every piece on the plate and still, both hands
    off. 0.75: that break, but a hand still on a piece or a piece still moving. 0.5: an end row is off on its own but
    the other nine are in two or more parts. 0.25: broken, but no end row on its own. 0: unbroken, or any piece off
    the plate."""
    if not all(on_plate(objects, n) for n in NAMES):
        return 0.0
    groups = parts(objects)
    if len(groups) == 1:
        return 0.0
    lone_row = any(g in END_ROWS for g in groups)
    if lone_row and len(groups) == 2:
        still = all(objects[n].resting() for n in NAMES)
        return 1.0 if still and hands_off(objects) else 0.75
    return 0.5 if lone_row else 0.25


def _debug(objects, g) -> None:
    """With CHOC_DEBUG=path set, one line per control step: the parts and the grade."""
    path = os.environ.get("CHOC_DEBUG")
    if path:
        sizes = [len(p) for p in parts(objects)]

        def at(n):  # a piece's middle, mm: y, and height above where it started
            p = np.asarray(objects[n].pos, float)
            return f"y={p[1] * 1000:.1f} dz={(p[2] - CENTRES[n][2]) * 1000:.2f}"

        def tips(g):
            pads = objects.get(g)
            return "?" if pads is None else "(%.1f,%.1f,%.1f)" % tuple(np.asarray(pads.tips, float) * 1000)

        with open(path, "a") as f:
            record = objects.get("bar_joints")
            gaps = []
            for a, b, _ in PAIRS:
                ra = _rot(objects[a].quat_xyzw)
                off = ra.T @ (np.asarray(objects[b].pos, float) - np.asarray(objects[a].pos, float))
                gaps.append(np.linalg.norm(off - (CENTRES[b] - CENTRES[a])) * 1000)
            widest = max(gaps)
            broken = len(getattr(record, "broken", ()))
            bends = [abs(bend_deg(objects, a, b)) for a, b, _ in PAIRS]
            f.write(f"grade={g} parts={sizes} broken={broken} most_bent_deg={max(bends):.2f} widest_mm={widest:.2f} by={sorted(_BROKEN_BY.items())} hands_off={hands_off(objects)} end_row[{at(name(0, 1))}] "
                    f"row1[{at(name(1, 1))}] far_row[{at(name(ROWS - 1, 1))}] "
                    f"left_tips={tips('left_gripper')} right_tips={tips('right_gripper')}\n")


def score(objects) -> float:
    g = grade(objects)
    _debug(objects, g)
    return min(g, COMPLETE_SCORE)


def success(objects) -> bool:
    return grade(objects) == 1.0


# -- the scripted demo -------------------------------------------------------------------------------------------------
# The left hand pinches the second row across the bar's width, right beside the groove to the end row, and holds it;
# the right hand pinches the end row (the one on its side) and lifts it. The bending is then all at that one groove.
# WHY the groove between the arms and the hands leaning apart: with both hands straight down over the right half,
# 2.7 cm apart, the arms ran into each other and pushed each other 6-8 cm aside, and nothing was pinched (official sim,
# 2026-11-19). Each hand now works on its own side of the midline, its fingertips at the bar and its body leaning away.
ACROSS = math.pi / 2  # yaw: the fingers close along x, across the bar's width (right arm)
# WHY the left hand turned the other way: at +90 degrees the left arm can't reach over the bar (IK off by 1.7 mm and
# 18 degrees, checked 2026-11-19); at -90 it can, and a parallel gripper closes along x either way.
ACROSS_LEFT = -math.pi / 2
LEAN = math.radians(35.0)
TIPS_UP = 0.002  # the pad tips this far above the plate
# WHY a gentle pinch: closed fully, each finger pushes at its 50 N limit, and with the pads a millimetre or two off the
# joints' height that bent the second row's joints past their strength: the left hand's pinch alone broke the bar
# into seven parts (official sim, 2026-11-19). Each finger is aimed this far into the bar instead; at the fingers'
# 5000 N/m that is 15-20 N.
# WHY 4 and 3 mm: at 1.5 mm (7.5 N a finger) the right hand's pads slid up the end row instead of lifting it, and the
# groove came apart sideways after bending only 0.56 degrees (official sim, 2026-11-19, heavier pieces).
SQUEEZE_RIGHT, SQUEEZE_LEFT = 0.004, 0.003
PINCH_RIGHT = -(BAR_W / 2 - SQUEEZE_RIGHT)  # finger joint target (FINGER_CLOSED is 0, pads touching)
PINCH_LEFT = -(BAR_W / 2 - SQUEEZE_LEFT)
HOVER_UP = 0.07
X = PLATE_XY[0]
GROOVE_Y = PLATE_XY[1] - PIECE[1]  # between the end row (row 0, the right arm's side) and row 1: on the midline
END_ROW_Y = float(CENTRES[name(0, 1)][1])
HOLD_Y = GROOVE_Y + 0.0095  # the left fingertips just inside row 1
LIFT = 0.008
# WHY 3 cm: carried 5 cm, the snapped row (held by its two outer pieces) split in the hand (official sim, 2026-11-19).
SET_DOWN_Y = END_ROW_Y - 0.03  # the end row is set down 3 cm away, still on the plate
# Where the hands wait at the end: low, beside the plate on their own sides. WHY: parked 15-18 cm over the table, a hand
# hid the snapped row from the top camera; over its base, pointing down, it was out of reach (official sim, 2026-11-19).
PARK_RIGHT, PARK_LEFT = (0.30, -0.32, TOP + 0.06), (0.30, 0.32, TOP + 0.06)


def _allow_matrix_grips() -> None:
    """Let a Waypoint's ``yaw`` be a full 3x3 TCP rotation instead of an angle, for the leaning hands (yam_arena.demo
    builds each waypoint's rotation with grip_rotation(yaw), which this wraps)."""
    if getattr(yam_demo, "_matrix_grips", False):
        return
    plain = yam_demo.grip_rotation

    def grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else plain(yaw)

    yam_demo.grip_rotation = grip_rotation
    yam_demo._matrix_grips = True


_allow_matrix_grips()


def grip(yaw: float, lean: float) -> np.ndarray:
    """Straight down turned ``yaw``, then leaned about x: lean > 0 points the fingers towards +y, so the hand's body
    leans out towards -y (the right arm's side)."""
    return axis_angle_matrix(np.array([1.0, 0.0, 0.0]), lean) @ yam_demo.grip_rotation(yaw)


RIGHT_GRIP, LEFT_GRIP = grip(ACROSS, LEAN), grip(ACROSS_LEFT, -LEAN)


def tcp(tips_y: float, rot: np.ndarray, up: float = 0.0):
    """The TCP that puts the pad tips at (X, tips_y) and TIPS_UP above the plate, raised by ``up``."""
    tips = np.array([X, tips_y, PLATE_TOP + TIPS_UP + up])
    return tuple(float(v) for v in tips - PAD_TIP_PAST_TCP * rot[:, 2])


# WHY the end row is turned about the groove instead of lifted: lifted straight up, the leaning hand dragged the row
# sideways, and the groove came apart that way after bending only 0.6 degrees (official sim, 2026-11-19). A person
# bends the row up about the groove; so does the demo, which turns the right hand about the groove's line.
# WHY only to 5 degrees: the groove snaps at about 2.3; turned on to 10, the snapped row, held by its two outer pieces,
# split in the hand (official sim, 2026-11-19).
BEND_STEPS_DEG = (3.0, 5.0)
GROOVE_LINE = np.array([X, GROOVE_Y, float(CENTRES[name(0, 1)][2])])  # a point on the groove's line (it runs along x)


def bent(rot: np.ndarray, pos, deg: float):
    """The hand at ``pos`` with ``rot``, turned ``deg`` about the groove's line so the end row's free end goes up."""
    turn = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), -math.radians(deg))  # the end row lies at -y: up is -turn
    p = GROOVE_LINE + turn @ (np.asarray(pos, float) - GROOVE_LINE)
    return tuple(float(v) for v in p), turn @ rot


def _right_break():
    R = RIGHT_GRIP
    pinch = tcp(END_ROW_Y, R)
    bends = [bent(R, pinch, deg) for deg in BEND_STEPS_DEG]
    top_pos, top_rot = bends[-1]
    clear_pos = (top_pos[0], top_pos[1], top_pos[2] + LIFT)
    return [
        Waypoint(2.0, tcp(END_ROW_Y, R, HOVER_UP), FINGER_OPEN, "over the end row, leaning out", R),
        Waypoint(1.5, pinch, FINGER_OPEN, "down beside it", R),
        Waypoint(1.0, pinch, FINGER_OPEN, "wait for the other hand", R),
        Waypoint(1.5, pinch, PINCH_RIGHT, "pinch the end row, slowly", R),
        Waypoint(0.5, pinch, PINCH_RIGHT, "hold", R),
        *[Waypoint(0.6, pos, PINCH_RIGHT, f"bend it up {deg:g} degrees about the groove", rot)
          for (pos, rot), deg in zip(bends, BEND_STEPS_DEG)],
        Waypoint(0.8, clear_pos, PINCH_RIGHT, "lift the snapped row clear", top_rot),
        Waypoint(1.2, tcp(SET_DOWN_Y, R, LIFT), PINCH_RIGHT, "carry it clear, level again", R),
        Waypoint(1.0, tcp(SET_DOWN_Y, R, 0.001), PINCH_RIGHT, "set it down", R),
        Waypoint(0.6, tcp(SET_DOWN_Y, R, 0.001), FINGER_OPEN, "let go", R),
        Waypoint(1.2, tcp(SET_DOWN_Y, R, HOVER_UP), FINGER_OPEN, "up", R),
        Waypoint(1.0, PARK_RIGHT, FINGER_OPEN, "clear", ACROSS),
        Waypoint(3.0, PARK_RIGHT, FINGER_OPEN, "stay", ACROSS),
    ]


def _left_hold():
    L = LEFT_GRIP
    # holds from its pinch until the right hand has bent the row up (right: 6.5 s to 7.7 s), then lets go
    return [
        Waypoint(2.0, tcp(HOLD_Y, L, HOVER_UP), FINGER_OPEN, "over the second row, leaning out", L),
        Waypoint(1.5, tcp(HOLD_Y, L), FINGER_OPEN, "down beside it", L),
        Waypoint(1.5, tcp(HOLD_Y, L), PINCH_LEFT, "pinch it by the groove, slowly", L),
        Waypoint(3.0, tcp(HOLD_Y, L), PINCH_LEFT, "hold while the other hand bends the row off", L),
        Waypoint(0.6, tcp(HOLD_Y, L), FINGER_OPEN, "let go", L),
        Waypoint(1.2, tcp(HOLD_Y, L, HOVER_UP), FINGER_OPEN, "up", L),
        Waypoint(1.0, PARK_LEFT, FINGER_OPEN, "clear", ACROSS_LEFT),
        Waypoint(3.0, PARK_LEFT, FINGER_OPEN, "stay", ACROSS_LEFT),
    ]


def _ends_plan():
    """A wrong way, for checking the score (CHOC_DEMO=ends): each hand pinches an end row and lifts it, so the bar bends
    over the plate like a beam on one support and breaks where the bending is greatest, near the middle."""
    def arm(y, clear, rot, yaw):
        return [
            Waypoint(2.0, tcp(y, rot, HOVER_UP), FINGER_OPEN, "over an end row", rot),
            Waypoint(1.5, tcp(y, rot), FINGER_OPEN, "down beside it", rot),
            Waypoint(0.8, tcp(y, rot), PINCH_RIGHT, "pinch it", rot),
            Waypoint(1.5, tcp(y, rot, 0.012), PINCH_RIGHT, "lift both ends", rot),
            Waypoint(1.0, tcp(y, rot, 0.001), PINCH_RIGHT, "put them down", rot),
            Waypoint(0.6, tcp(y, rot, 0.001), FINGER_OPEN, "let go", rot),
            Waypoint(1.2, tcp(y, rot, HOVER_UP), FINGER_OPEN, "up", rot),
            Waypoint(1.0, clear, FINGER_OPEN, "clear", yaw),
            Waypoint(3.0, clear, FINGER_OPEN, "stay", yaw),
        ]
    far = float(CENTRES[name(ROWS - 1, 1)][1])
    return {"left": arm(far, PARK_LEFT, LEFT_GRIP, ACROSS_LEFT),
            "right": arm(END_ROW_Y, PARK_RIGHT, RIGHT_GRIP, ACROSS)}


def demo():
    if os.environ.get("CHOC_DEMO") == "ends":
        return _ends_plan()
    return {"left": _left_hold(), "right": _right_break()}
