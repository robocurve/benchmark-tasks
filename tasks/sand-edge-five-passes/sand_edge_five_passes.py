"""On the station table, on the robot's right: a small blue bench vise (8.5 cm jaws, 10.8 cm tall), fixed, its jaws
clamping along x. A 3 mm plywood workpiece (20 x 7 cm) stands upright in the jaws, along y, its top edge 3 cm above
them, from y = -0.31 to y = -0.11 at x = 0.34. An orange foam sanding block (10 x 7 x 3 cm, grey sandpaper on its
underside, 60 g) lies flat on the table in front of the vise, at (0.25, -0.35), its long side along y. A steel speed
square and a pair of safety glasses lie on the left of the table.

The task: sand the workpiece's top edge with the block, five full passes along the whole edge, keeping the block
flat on it.

A pass is counted from the block's pose every control step (``on_step``): its sanded face within 1 mm of the edge's
top and tilted under 10 degrees, while its centre moves from within 1 cm of one end of the edge to within 1 cm of the
other. The workpiece is held in the jaws by a bond (``bonds.py``) that gives way to a hard press: pushed too hard, it
slips in the vise, and a workpiece that has moved 5 mm scores 0. The vise is NVIDIA SimReady's
``equipment_bench_vise_a01`` (``simready.py``), scaled to 0.6 with its sliding jaw closed onto the workpiece.

Run it:  inspect-robots run --embodiment yam_arena -E task=sand_edge_five_passes.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from bonds import Bonded, on_reset, on_spawn, on_step  # noqa: E402
from looks import dress  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.constants import PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

# -- the vise ----------------------------------------------------------------------------------------------------------
# SimReady's bench vise, measured in its own file (metres): the fixed jaw's face at x = 0.0831, the sliding jaw's at
# x = 0.1291, the jaw plates from z = 0.145 to their tops at z = 0.1803, 14.1 cm wide (y), centred on y = 0.0003. The
# sliding jaw, its screw and its handle are moved toward the fixed jaw so the jaws stand 6 mm apart at this scale:
# the 3 mm workpiece with 1.5 mm of play each side.
VISE_SCALE = 0.6  # a 3-inch vise: 8.5 cm jaws, 10.8 cm to their tops
FIXED_FACE, SLIDING_FACE, JAW_TOP_FILE, JAW_Y_FILE = 0.0831, 0.1291, 0.1803, 0.0003
PIECE_T = 0.003  # m: the plywood's thickness
JAW_PLAY = 0.0015  # m each side of the workpiece
_gap_file = (PIECE_T + 2 * JAW_PLAY) / VISE_SCALE
JAW_SHIFT = FIXED_FACE + _gap_file - SLIDING_FACE  # file units, along the file's x
GAP_X_FILE = FIXED_FACE + _gap_file / 2  # the middle of the gap, in the file

EDGE_X = 0.34  # m: the workpiece's plane, and so the edge's line, on the table
PIECE_Y = -0.21  # m: the workpiece's middle along y
VISE_XY = (EDGE_X - GAP_X_FILE * VISE_SCALE, PIECE_Y - JAW_Y_FILE * VISE_SCALE)
JAW_TOP = TOP + JAW_TOP_FILE * VISE_SCALE
VISE = SimReady(
    "vise",
    pos=(*VISE_XY, TOP),
    package="equipment_bench_vise_a01",
    usd="sm_equipment_bench_vise_a01_01.usd",
    scale=VISE_SCALE,
    shift=tuple((part, (JAW_SHIFT, 0.0, 0.0)) for part in
                ("slidingJaw_01_obj_00", "screw_01_obj_00", "handle_01_obj_00")),
)

# -- the workpiece: 3 mm plywood standing in the jaws, its top edge 3 cm above them ------------------------------------
PROUD = 0.03  # m: the edge above the jaws' tops
PIECE_SIZE = (PIECE_T, 0.20, 0.07)  # thick (x), long (y), tall: 4 cm in the jaws, its bottom 1 cm above the screw
EDGE_TOP = JAW_TOP + PROUD
PIECE_START = (EDGE_X, PIECE_Y, EDGE_TOP - PIECE_SIZE[2] / 2)
PIECE_MASS = 0.03
PIECE_HOLD = 30.0  # N: the push, in any direction, that makes the workpiece slip in the jaws
# The bond's torque limit is left off: PhysX measured a twist over 2 N m on this world-anchored joint under a 4 N
# press, so a torque limit would break the hold at random. A lever on the workpiece breaks it through the force.
PIECE_TWIST = math.inf
PIECE = Bonded("workpiece", pos=PIECE_START, size=PIECE_SIZE, mass=PIECE_MASS, color=(0.78, 0.62, 0.42),
               hold=PIECE_HOLD, twist=PIECE_TWIST)
PIECE_FRICTION = (0.6, 0.5)  # static, dynamic: sandpaper on plywood (the bench's rubber-like 1.2 elsewhere)

# -- the sanding block: orange foam, grey sandpaper on its underside (its local -z face) --------------------------------
BLOCK_SIZE = (0.07, 0.10, 0.03)  # wide (x), long (y), tall
BLOCK_XY = (0.25, -0.35)
BLOCK = Box("sanding_block", pos=(*BLOCK_XY, TOP + BLOCK_SIZE[2] / 2), size=BLOCK_SIZE, mass=0.06,
            color=(0.95, 0.42, 0.05))

# -- what else lies on the bench: a speed square and safety glasses, fixed, on the left ---------------------------------
SQUARE = SimReady("speed_square", pos=(0.24, 0.24, TOP), package="measurement_ruler_speedsquare_a02",
                  usd="sm_measurement_ruler_speedsquare_a02_01.usd", scale=0.6, yaw=math.radians(-30))
GLASSES = SimReady("safety_glasses", pos=(0.42, 0.14, TOP), package="ppe_eyeProtection_safetyGlasses_a01",
                   usd="sm_ppe_eyeProtection_safetyGlasses_a01_01.usd", yaw=math.radians(200))

OBJECTS = [VISE, SQUARE, GLASSES, PIECE, BLOCK]


@on_spawn
def _sandpaper_friction(name: str, prim) -> None:
    """The workpiece's friction is sandpaper on plywood, whatever touches it (the lower of the two is used)."""
    if name != PIECE.name:
        return
    from pxr import Sdf, Usd, UsdPhysics

    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.MaterialAPI):
            material = UsdPhysics.MaterialAPI(p)
            material.CreateStaticFrictionAttr(PIECE_FRICTION[0])
            material.CreateDynamicFrictionAttr(PIECE_FRICTION[1])
            if "PhysxMaterialAPI" not in p.GetAppliedSchemas():
                p.AddAppliedSchema("PhysxMaterialAPI")
            p.CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set("min")


# What the boxes look like: meshes modelled in Blender (looks/make_looks.py), the colliders unchanged.
LOOKS = {"workpiece": "workpiece.usdc", "sanding_block": "block.usdc"}
dress(LOOKS)

# -- scoring ---------------------------------------------------------------------------------------------------------
FACE_GAP = 0.001  # m: the sanded face this close to the edge's top touches it
FLAT_DEG = 10.0  # a pass needs the block tilted less than this
GOUGE_DEG = 25.0  # pressing the edge tilted this much or more ...
GOUGE_S = 0.5  # ... for this long scores 0
END_ZONE = 0.01  # m: the block's centre within this of an end of the edge (or past it) is at that end
MOVED = 0.005  # m: the workpiece's centre or an end of its top edge this far from its start has slipped
PASSES = 5
PER_PASS = 0.2
OFF_TABLE_Z = -0.02  # m above the table: a centre lower than this has fallen off the table

HALF_BLOCK = np.asarray(BLOCK_SIZE) / 2
HALF_EDGE = PIECE_SIZE[1] / 2
# The edge's top, as points every 2.5 mm along its middle line, and its two ends, in the workpiece's own frame.
EDGE_LOCAL = np.stack([np.zeros(81), np.linspace(-HALF_EDGE, HALF_EDGE, 81), np.full(81, PIECE_SIZE[2] / 2)], 1)
EDGE_ENDS_LOCAL = EDGE_LOCAL[[0, -1]]

# What the episode remembers: the passes so far, the end the current stroke started from, how long the block has
# been pressed on the edge steeply tilted, and why the trial failed.
STATE = {"passes": 0, "from": "", "gouge_s": 0.0, "failed": ""}


@on_reset
def reset_memory(env=None) -> None:
    STATE.update(passes=0, gouge_s=0.0, failed="")
    STATE["from"] = ""


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.quat_xyzw)) and np.all(np.isfinite(s.lin_vel))
               for s in objects.values())


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def edge_points(objects, local=EDGE_LOCAL) -> np.ndarray:
    """Points on the edge's top in the world, following the workpiece."""
    s = objects["workpiece"]
    return np.asarray(s.pos) + local @ _rot(s).T


def in_block(objects, points) -> np.ndarray:
    """``points`` in the block's own frame (x across, y along, z up from its centre)."""
    s = objects["sanding_block"]
    return (np.asarray(points) - np.asarray(s.pos)) @ _rot(s)


def tilt_deg(objects) -> float:
    """The angle between the block's sanded face's normal and straight down."""
    return float(np.degrees(np.arccos(np.clip(_rot(objects["sanding_block"])[2, 2], -1.0, 1.0))))


def face_on_edge(objects) -> bool:
    """Some point of the edge's top lies under the sanded face, within 1 mm of it."""
    p = in_block(objects, edge_points(objects))
    under = (np.abs(p[:, 0]) <= HALF_BLOCK[0]) & (np.abs(p[:, 1]) <= HALF_BLOCK[1])
    return bool(np.any(under & (np.abs(p[:, 2] + HALF_BLOCK[2]) <= FACE_GAP)))


def block_on_edge(objects) -> bool:
    """Some point of the edge's top within 1 mm of the block, on any face."""
    q = np.abs(in_block(objects, edge_points(objects))) - HALF_BLOCK
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
    inside = np.minimum(q.max(axis=1), 0.0)
    return bool(np.min(outside + inside) <= FACE_GAP)


def along_edge(objects) -> float:
    """The block's centre along the edge, from the edge's middle (m, + toward the robot's left)."""
    s = objects["workpiece"]
    return float(((np.asarray(objects["sanding_block"].pos) - np.asarray(s.pos)) @ _rot(s))[1])


def end_zone(objects) -> str:
    """"a" or "b" when the block's centre is within 1 cm of the edge's right or left end, or past it."""
    y = along_edge(objects)
    if y <= -HALF_EDGE + END_ZONE:
        return "a"
    if y >= HALF_EDGE - END_ZONE:
        return "b"
    return ""


def piece_moved(objects) -> bool:
    """The workpiece's centre, or an end of its top edge, 5 mm or more from where it started."""
    now = np.vstack([objects["workpiece"].pos, edge_points(objects, EDGE_ENDS_LOCAL)])
    start = np.vstack([PIECE_START, np.asarray(PIECE_START) + EDGE_ENDS_LOCAL])
    return bool(np.max(np.linalg.norm(now - start, axis=1)) >= MOVED)


def _fell(objects) -> bool:
    return bool(objects["sanding_block"].pos[2] < TOP + OFF_TABLE_Z)


def observe(objects, dt: float = 0.05) -> None:
    """One control step of the episode's memory: the stroke under way, the passes, and the two ways to fail."""
    if STATE["failed"] or not _finite(objects):
        return
    if piece_moved(objects):
        STATE["failed"] = "the workpiece slipped in the vise"
        return
    tilt = tilt_deg(objects)
    STATE["gouge_s"] = STATE["gouge_s"] + dt if tilt >= GOUGE_DEG and block_on_edge(objects) else 0.0
    if STATE["gouge_s"] >= GOUGE_S - 1e-6:
        STATE["failed"] = "the block dug into the edge, tilted"
        return
    if not (tilt < FLAT_DEG and face_on_edge(objects)):
        STATE["from"] = ""  # off the edge, or not flat on it: the stroke is broken
        return
    zone = end_zone(objects)
    if zone and STATE["from"] and zone != STATE["from"]:
        STATE["passes"] += 1
    if zone:
        STATE["from"] = zone


@on_step
def watch(env) -> None:
    """Every control step: the block and the workpiece, into the episode's memory."""
    if STATE["failed"]:
        return
    from yam_arena.scene import object_states

    observe(object_states(env, ["sanding_block", "workpiece"]), float(env.unwrapped.step_dt))


def success(objects) -> bool:
    """Five passes, the workpiece still where it was clamped, and the block never dug in."""
    if not _finite(objects) or STATE["failed"]:
        return False
    return STATE["passes"] >= PASSES and not piece_moved(objects)


def score(objects) -> float:
    """0.2 per pass, 1.0 with five. 0 for the rest of the trial once the workpiece has slipped 5 mm in the vise, or
    the block has pressed on the edge tilted 25 degrees or more for 0.5 s."""
    if not _finite(objects) or STATE["failed"]:
        return 0.0
    if piece_moved(objects):
        STATE["failed"] = "the workpiece slipped in the vise"
        return 0.0
    return min(STATE["passes"], PASSES) * PER_PASS


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Right arm, straight down, the wrist turned a quarter turn so the fingers close along x, across the block's 7 cm:
# grip the block 2.5 cm deep, lift it over the vise, lower it flat onto the edge's right end, then sweep five strokes
# along y between 1 cm past each end, leaning on the edge, and lift it off. The left arm holds still at READY, on
# its own side. Numbers are TCP positions.
GRIP_YAW = math.pi / 2
# A partial grip: the finger joints are sent to 5.5 mm inside the block's sides, so each pad squeezes it with about
# 27 N (5000 N/m) instead of the full 50 N, which makes the light block chatter against the edge.
GRIP = -(BLOCK_SIZE[0] / 2 - 0.0055)
PAD_ABOVE_FACE = 0.005  # m: the pad tips this far above the sanded face
GRIP_Z = TOP + PAD_ABOVE_FACE + PAD_TIP_PAST_TCP  # the TCP over the block lying on the table
FACE_BELOW_TCP = PAD_ABOVE_FACE + PAD_TIP_PAST_TCP
LEAN = 0.006  # m: the TCP is sent this far below where the face meets the edge, so the arm presses on it
SAND_Z = EDGE_TOP + FACE_BELOW_TCP - LEAN
CLEAR_Z = EDGE_TOP + FACE_BELOW_TCP + 0.025  # the face 2.5 cm over the edge
LIFT_Z = TOP + 0.10
OVERRUN = 0.01  # m: each stroke takes the block's centre this far past the edge's end
END_A = PIECE_Y - HALF_EDGE - OVERRUN
END_B = PIECE_Y + HALF_EDGE + OVERRUN
STROKE_S = 2.5


def demo():
    w = Waypoint
    bx, by = BLOCK_XY
    x, yaw = EDGE_X, GRIP_YAW
    strokes = [w(STROKE_S, (x, END_B if i % 2 == 0 else END_A, SAND_Z), GRIP, f"stroke {i + 1}", yaw)
               for i in range(PASSES)]
    end = END_B if PASSES % 2 else END_A
    return {
        "right": [
            w(1.5, (bx, by, LIFT_Z), FINGER_OPEN, "over the block", yaw),
            w(1.2, (bx, by, GRIP_Z), FINGER_OPEN, "down round it", yaw),
            w(0.8, (bx, by, GRIP_Z), GRIP, "close", yaw),
            w(1.2, (bx, by, CLEAR_Z), GRIP, "lift", yaw),
            w(1.5, (x, END_A, CLEAR_Z), GRIP, "over the edge's right end", yaw),
            w(1.2, (x, END_A, SAND_Z), GRIP, "down onto the edge", yaw),
            w(0.5, (x, END_A, SAND_Z), GRIP, "settle", yaw),
            *strokes,
            w(1.0, (x, end, CLEAR_Z), GRIP, "lift off", yaw),
        ],
    }
