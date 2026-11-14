"""On the station table: a grey steel plate stands across the far side with two fixed air-brake gladhands on it,
red on the left and blue on the right, their rubber seal faces toward the robot and their bodies pointing up. A red
air hose runs from a brass fitting on a grey post near the left arm to a red gladhand, which starts locked, seal
face down, on a grey dummy coupler in front of the left arm. The robot must turn the hose's gladhand off its dummy
coupler, press its seal onto the seal of the plate's red gladhand with the two bodies at 90 degrees, turn it a
quarter turn until it stops with its body pointing down, and let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import gladkit  # noqa: E402
import numpy as np  # noqa: E402
import scoring  # noqa: E402
from gladkit import air_line, coupler, finger_points, mated_pose, measure, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the score's own 1 s "let go" window is the hold
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20

INSTRUCTION = "Connect the tractor's air lines to the trailer, then let go."

RED = (0.80, 0.08, 0.08)
BLUE = (0.08, 0.22, 0.80)
GREY = (0.55, 0.56, 0.58)
BRASS = (0.72, 0.58, 0.25)
COLOURS = {"red": RED, "blue": BLUE}  # the two trailer gladhands
SIDE = {"red": 1.0, "blue": -1.0}  # red on the left (+y), blue on the right
LINE = "red"  # the one air line: the red (emergency) hose, on the left

# -- fixed layout (world frame: x forward from the arms, y left, z up, metres; heights above the table top) ---------
PLATE_FRONT_X = 0.45
PLATE_SIZE = (0.005, 0.40, 0.32)  # thickness, width (y), height
FITTING_LEN = 0.040  # bulkhead fitting between the plate and a trailer gladhand's body
TRAILER_Y = 0.10
TRAILER_SEAL_H = 0.25
TRAILER_FACE_X = PLATE_FRONT_X - FITTING_LEN + gladkit.HANDLE_Z[0] * gladkit.MM  # 0.3635: the gladhand's seal face
# Seal face toward the robot (-x), body pointing up (+z).
TRAILER_ROT = np.array([[0.0, 0.0, -1.0], [0.0, -1.0, 0.0], [-1.0, 0.0, 0.0]])

DUMMY_X, DUMMY_Y = 0.22, 0.30
DUMMY_SEAL_H = 0.12
# The dummy coupler's seal face is up. The hose gladhand locked on it points along DUMMY_YAW (rad from straight
# ahead, +x, counterclockwise seen from above); turning it off swings it a quarter turn clockwise.
DUMMY_YAW = 0.0
BRACKET_SIZE = (0.05, 0.05, DUMMY_SEAL_H - gladkit.HEAD_DEPTH * gladkit.MM)

POST_X, POST_Y = 0.10, 0.42
POST_SIZE = (0.06, 0.06, 0.20)
FITTING_H = 0.15
# Brass fitting on the post's outer face (the face away from the other arm); the hose leaves its front end heading
# straight ahead (+x).
FITTING_SIZE = (0.03, 0.02, 0.03)
HOSE_START_X = POST_X + FITTING_SIZE[0] / 2
HOSE_START_Y = POST_Y + POST_SIZE[1] / 2 + FITTING_SIZE[1] / 2


def trailer_pos(colour: str) -> np.ndarray:
    return np.array([TRAILER_FACE_X, SIDE[colour] * TRAILER_Y, TOP + TRAILER_SEAL_H])


def dummy_pos(colour: str = LINE) -> np.ndarray:
    return np.array([DUMMY_X, SIDE[colour] * DUMMY_Y, TOP + DUMMY_SEAL_H])


def dummy_rot(colour: str = LINE) -> np.ndarray:
    return gladkit.rot_z(DUMMY_YAW)


# The hose runs forward from its fitting, loops round clockwise beside the plate and comes back into its gladhand's
# handle: half a turn.
HOSE_TURN = -math.pi
# Where the hose's starting curve must not run: behind or through the nose plate, and within 7 cm of the table's
# side edges (x0, x1, y0, y1).
HOSE_KEEP_OUT = ((PLATE_FRONT_X - 0.04, 1.0, -PLATE_SIZE[1] / 2 - 0.03, PLATE_SIZE[1] / 2 + 0.03),
                 (-1.0, 1.0, 0.58, 1.0), (-1.0, 1.0, -1.0, -0.58))


def _objects():
    out = [Box("nose_plate", pos=(PLATE_FRONT_X + PLATE_SIZE[0] / 2, 0.0, TOP + PLATE_SIZE[2] / 2), size=PLATE_SIZE,
               color=GREY)]
    for colour, rgb in COLOURS.items():
        out.append(coupler(f"trailer_{colour}", trailer_pos(colour), TRAILER_ROT, "trailer", rgb))
    s = SIDE[LINE]
    out.append(Box(f"bracket_{LINE}", pos=(DUMMY_X, s * DUMMY_Y, TOP + BRACKET_SIZE[2] / 2), size=BRACKET_SIZE,
                   color=GREY, yaw=DUMMY_YAW))
    out.append(coupler(f"dummy_{LINE}", dummy_pos(), dummy_rot(), "dummy", GREY))
    out.append(Box(f"post_{LINE}", pos=(POST_X, s * POST_Y, TOP + POST_SIZE[2] / 2), size=POST_SIZE, color=GREY))
    out.append(Box(f"fitting_{LINE}", pos=(POST_X, s * HOSE_START_Y, TOP + FITTING_H), size=FITTING_SIZE,
                   color=BRASS))
    # The hose's gladhand starts locked on the dummy coupler; the hose loops out over the table and comes back into
    # its handle.
    g_pos, g_rot = mated_pose(dummy_pos(), dummy_rot(), 0.0)
    out.append(air_line(f"line_{LINE}", (HOSE_START_X, s * HOSE_START_Y, TOP + FITTING_H), 0.0, g_pos, g_rot,
                        COLOURS[LINE], turn=HOSE_TURN, keep_out=HOSE_KEEP_OUT))
    return out


OBJECTS = _objects()
LINES = {LINE: f"line_{LINE}"}

# -- scoring --------------------------------------------------------------------------------------------------------
CLEAR_GAP = 0.02  # a gripper is clear of a gladhand or hose when every finger pad is at least this far from it
FLOOR_DROP = 0.02  # below the table top by this much: off the table
DETACH_STRETCH = 0.005  # the hose's fixed end has left its fitting by this much
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(clear_steps=0, floor=False, detached=False, fingers=None, finger_error=None, tick=0, scored=-1,
                  score=0.0)


_reset_state()


@on_step
def _fingers(environment) -> None:
    """score() cannot see the robot; keep the finger pad points for the "let go" check."""
    _STATE["tick"] += 1
    try:
        _STATE["fingers"] = finger_points(environment)
    except Exception as exc:  # noqa: BLE001 - report once; an unknown grip never counts as let go
        if _STATE["finger_error"] is None:
            print(f"gladhand-coupling: finger check unavailable: {exc!r}", flush=True)
        _STATE["finger_error"], _STATE["fingers"] = exc, None


def _to_polyline(point, pts) -> float:
    a, b = pts[:-1], pts[1:]
    ab = b - a
    t = np.clip(np.einsum("ij,ij->i", point - a, ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-12), 0.0, 1.0)
    return float(np.min(np.linalg.norm(a + t[:, None] * ab - point, axis=1)))


def finger_clearance(objects) -> float:
    """Smallest distance (m) from any finger pad to any gladhand or the hose; 0 when the fingers are unknown."""
    fingers = _STATE["fingers"]
    if fingers is None:
        return 0.0
    line = objects[LINES[LINE]]
    hose = np.vstack([line.points, line.to_world(np.array(gladkit.PORT) * gladkit.MM)])
    best = math.inf
    for p in fingers:
        best = min(best, gladkit.distance_to_gladhand(p, line.pos, line.rotation()), _to_polyline(p, hose) - line.radius,
                   gladkit.distance_to_gladhand(p, dummy_pos(), dummy_rot()),
                   *(gladkit.distance_to_gladhand(p, trailer_pos(c), TRAILER_ROT) for c in COLOURS))
    return best


def coupling(objects, trailer_colour: str) -> scoring.Coupling:
    line = objects[LINES[LINE]]
    m = measure(trailer_pos(trailer_colour), TRAILER_ROT, line.pos, line.rotation())
    return scoring.Coupling(gap_excess_mm=m["gap_mm"] - gladkit.GAP_LOCKED,
                            angle_from_stop_deg=gladkit.LOCK_ANGLE_DEG - m["angle_deg"],
                            holds_pull=m["lug_under_lip"])


def end_state(objects) -> scoring.EndState:
    line = objects[LINES[LINE]]
    lowest = min(float(line.points[:, 2].min()), float(line.pos[2]))  # hose centres and the gladhand's face
    _STATE["floor"] |= lowest < TOP - FLOOR_DROP
    _STATE["detached"] |= line.stretch > DETACH_STRETCH
    _STATE["clear_steps"] = _STATE["clear_steps"] + 1 if finger_clearance(objects) >= CLEAR_GAP else 0
    return scoring.EndState(
        line=scoring.Line(dummy_distance_mm=float(np.linalg.norm(line.pos - dummy_pos())) * 1000.0,
                          couplings={c: coupling(objects, c) for c in COLOURS}),
        clear_s=_STATE["clear_steps"] / CONTROL_HZ, floor_touched=_STATE["floor"], hose_detached=_STATE["detached"])


def _score(objects) -> float:
    # success() and score() are both called every step; advance the tracked state once per step.
    if _STATE["scored"] != _STATE["tick"]:
        _STATE.update(scored=_STATE["tick"], score=scoring.score(end_state(objects)))
    return _STATE["score"]


def success(objects) -> bool:
    """The hose's gladhand locked to the red trailer gladhand and both grippers clear of every gladhand and the hose
    for the last second."""
    return _score(objects) >= 1.0


def score(objects) -> float:
    """0.2 if the hose's gladhand is off its dummy coupler; 0.8 in place of that if it is locked to the red trailer
    gladhand; plus 0.2 if it is locked there and both grippers have been clear for 1 s. 0 if it is locked to the blue
    trailer gladhand, the gladhand or hose left the table, or the hose came off its fitting."""
    return _score(objects)
