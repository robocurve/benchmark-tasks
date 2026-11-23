"""A 10 inch compound mitre saw sits on the bench, its front toward the arms, set square (0 degrees) with its head
raised. The turntable is held at its detents by a spring plunger; a release paddle beside the front handle lifts the
plunger when it is squeezed against the handle. Detents are at 0, 15, 22.5, 31.6 and 45 degrees either side.

The robot must set the saw to a 45 degree mitre, either side, and leave the turntable locked in that detent: squeeze
the paddle while swinging the turntable by its handle, and let go at the 45 degree detent so the plunger drops in.

Everything is a real product at true size: a 10 inch compound mitre saw (about 15 kg, 280 mm turntable, 254 mm blade)
with a squeeze-type miter detent release on its front handle.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import (Hinge, PartState, finger_points, on_reset, on_step, part, set_hinge_stiffness,  # noqa: E402
                     set_hinge_target)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Set the mitre saw to a 45 degree mitre and leave it locked in the detent."

YELLOW = (0.93, 0.72, 0.10)
DARK = (0.16, 0.17, 0.18)
ALU = (0.70, 0.72, 0.74)
RED = (0.75, 0.15, 0.12)

# Part frame: origin on the bench under the turntable's centre; -x toward the arms (the saw's front), z up.
SAW_XY = (0.400, 0.000)
TABLE_R = 0.140                 # turntable radius
TABLE_TOP_Z = 0.085             # the table's working surface above the bench
WING_Y = (0.145, 0.230)         # the fixed extension wings either side
ARM_X = (-0.230, -0.135)        # the turntable's front arm, out under the handle
HANDLE_X = (-0.2275, -0.2025)   # the front handle: 25 mm deep (radial), 60 mm long, 25 mm tall
HANDLE_Y = 0.030
HANDLE_Z = (0.065, 0.090)
PADDLE_X = (-0.2380, -0.2320)   # the release paddle, 4.5 mm in front of the handle when released
PADDLE_PIVOT = (-0.2350, 0.030, 0.0775)   # it swings about a vertical pin at its +y end
PADDLE_TRAVEL = 0.075           # rad until its free end touches the handle (positive swings it toward the handle)

BASE_BOXES = [
    ((0.025, 0.0, 0.0365), (0.250, 0.290, 0.073), DARK),                               # base under the turntable
    ((0.050, (WING_Y[0] + WING_Y[1]) / 2, 0.0425), (0.200, WING_Y[1] - WING_Y[0], 0.085), DARK),  # wing, +y (rear)
    ((0.050, -(WING_Y[0] + WING_Y[1]) / 2, 0.0425), (0.200, WING_Y[1] - WING_Y[0], 0.085), DARK), # wing, -y (rear)
    ((0.045, 0.145, 0.1275), (0.015, 0.170, 0.055), ALU),                              # fence, +y half
    ((0.045, -0.145, 0.1275), (0.015, 0.170, 0.055), ALU),                             # fence, -y half (15 mm clear below)
]
TABLE_PARTS = [  # on the turntable hinge: the table, its front arm and handle, and the raised head on its rear column
    ((ARM_X[0] / 2 + ARM_X[1] / 2, 0.0, 0.050), (ARM_X[1] - ARM_X[0], 0.050, 0.020), DARK),   # top 60 mm up
    (((HANDLE_X[0] + HANDLE_X[1]) / 2, 0.0, (HANDLE_Z[0] + HANDLE_Z[1]) / 2),
     (HANDLE_X[1] - HANDLE_X[0], 2 * HANDLE_Y, HANDLE_Z[1] - HANDLE_Z[0]), RED),
    ((0.180, 0.0, 0.092), (0.090, 0.060, 0.012), DARK),                                # rear extension, under the fence
    ((0.200, 0.0, 0.204), (0.040, 0.060, 0.212), YELLOW),                              # head pivot column
    ((0.060, 0.0, 0.340), (0.320, 0.070, 0.060), YELLOW),                              # head arm
    ((0.020, 0.075, 0.325), (0.110, 0.080, 0.090), DARK),                              # motor
]
TABLE_CYLS = [((0.0, 0.0, 0.079), TABLE_R, 0.012, ALU, "Z"),
              ((0.000, 0.0, 0.300), 0.127, 0.003, (0.80, 0.80, 0.82), "Y")]            # the blade, raised
PADDLE_BOXES = [(((PADDLE_X[0] + PADDLE_X[1]) / 2, 0.0, (HANDLE_Z[0] + HANDLE_Z[1]) / 2),
                 (PADDLE_X[1] - PADDLE_X[0], 2 * HANDLE_Y, HANDLE_Z[1] - HANDLE_Z[0] - 0.004), DARK)]

DETENTS = [math.radians(d) for d in (-45.0, -31.6, -22.5, -15.0, 0.0, 15.0, 22.5, 31.6, 45.0)]
CATCH = math.radians(1.5)       # the plunger drops into a detent within this angle of it
LOCK_K = 1000.0                 # N m/rad: the steel plunger in a detent
FREE_K = 0.0
PRESSED = 0.6 * PADDLE_TRAVEL   # the paddle squeezed this far lifts the plunger
SAW_MASS = 15.0

SAW = part("mitre_saw", pos=(*SAW_XY, TOP), mass=SAW_MASS, color=DARK, friction=0.9, boxes=BASE_BOXES, hinges=(
    Hinge("table", pivot=(0.0, 0.0, 0.073), boxes=tuple(TABLE_PARTS), cylinders=tuple(TABLE_CYLS),
          lower=-math.radians(50.0), upper=math.radians(50.0), stiffness=LOCK_K, damping=3.0, target=0.0,
          mass=3.0, axis="Z"),
    Hinge("paddle", pivot=PADDLE_PIVOT, boxes=tuple(PADDLE_BOXES), lower=0.0, upper=PADDLE_TRAVEL, stiffness=0.08,
          damping=0.005, target=0.0, mass=0.03, axis="Z", parent="table"),
))
OBJECTS = [SAW]

# -- scoring ------------------------------------------------------------------------------------------------------------
GOAL = math.radians(45.0)
GOAL_TOL = math.radians(0.5)
TURNED = math.radians(10.0)      # partial credit once the table has been swung this far
SAW_MOVE = 0.010
CLEAR = 0.030                    # both grippers this far from the handle and paddle at the end
STILL_STEPS = 20
STILL_TOL = math.radians(0.1)
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(saw0=None, failed=False, why="", turned=False, clear=False, locked_at=None, recent=[], steps=0)


_reset_state()


def table_angle(saw: PartState) -> float:
    return float(saw.hinges.get("table", 0.0))


def paddle_pressed(saw: PartState) -> bool:
    return float(saw.hinges.get("paddle", 0.0)) >= PRESSED


def nearest_detent(angle: float) -> float:
    return min(DETENTS, key=lambda d: abs(d - angle))


def handle_points(saw: PartState) -> list:
    """World points on the handle and paddle (their centres), for the grippers-clear check."""
    th = table_angle(saw)
    c, s = math.cos(th), math.sin(th)
    out = []
    for x in ((HANDLE_X[0] + HANDLE_X[1]) / 2, (PADDLE_X[0] + PADDLE_X[1]) / 2):
        local = np.array([c * x, s * x, (HANDLE_Z[0] + HANDLE_Z[1]) / 2])
        out.append(saw.to_world(local))
    return out


def observe(saw: PartState, fingers) -> None:
    s = _STATE
    if s["saw0"] is None:
        s["saw0"] = np.array(saw.pos, float)
    if float(np.linalg.norm(np.asarray(saw.pos) - s["saw0"])) > SAW_MOVE:
        s["failed"], s["why"] = True, s["why"] or "saw moved"
    th = table_angle(saw)
    if abs(th) >= TURNED:
        s["turned"] = True
    pressed = paddle_pressed(saw)
    d = nearest_detent(th)
    s["locked_at"] = d if (not pressed and abs(th - d) <= CATCH) else None
    s["clear"] = fingers is not None and all(np.linalg.norm(np.asarray(f) - p) >= CLEAR + 0.02
                                             for f in fingers for p in handle_points(saw))
    s["recent"].append(th)
    del s["recent"][:-STILL_STEPS]
    s["steps"] += 1


@on_step
def _plunger(environment) -> None:
    """The detent plunger: lifted while the paddle is squeezed (the table turns freely); released, it drops into a
    detent within 1.5 degrees and locks the table there, and otherwise rides on the detent plate (free)."""
    from partkit import _part_state

    saw = _part_state(environment, SAW)
    th = table_angle(saw)
    d = nearest_detent(th)
    if not paddle_pressed(saw) and abs(th - d) <= CATCH:
        set_hinge_target(environment, SAW.name, "table", d)
        set_hinge_stiffness(environment, SAW.name, "table", LOCK_K)
    else:
        set_hinge_stiffness(environment, SAW.name, "table", FREE_K)
    try:
        fingers = [np.asarray(p, float) for p in finger_points(environment)]
    except Exception as exc:  # noqa: BLE001 - an unreadable gripper never counts as clear
        if not _STATE.get("warned"):
            print(f"miter-saw-45: gripper check unavailable: {exc!r}", flush=True)
            _STATE["warned"] = True
        fingers = None
    observe(saw, fingers)


def _still() -> bool:
    recent = _STATE["recent"]
    return len(recent) == STILL_STEPS and float(np.ptp(np.array(recent))) < STILL_TOL


def distance(objects) -> float:
    """Radians from the nearer 45 degree detent."""
    return abs(abs(table_angle(objects["mitre_saw"])) - GOAL)


def _level(objects) -> tuple[float, bool]:
    if _STATE["failed"]:
        return 0.0, False
    locked = _STATE["locked_at"]
    if locked is not None and abs(abs(locked) - GOAL) < 1e-6:
        if _still() and _STATE["clear"]:
            return 1.0, True
        return 0.75, False
    if locked is not None and abs(locked) > 1e-6:
        return 0.5, False
    if _STATE["turned"]:
        return 0.25, False
    return 0.0, False


def success(objects) -> bool:
    """The turntable locked in a 45 degree detent (either side) with the release paddle let go, both grippers clear of
    the handle and paddle, and the table still for 1 s; the saw never moved 10 mm."""
    return _level(objects)[1]


def score(objects) -> float:
    """1.0 success; 0.75 locked at 45 degrees but a gripper is still at the handle or the table still moves; 0.5
    locked in another mitre detent (15, 22.5 or 31.6 degrees); 0.25 the table was swung 10 degrees at some point;
    else 0. 0 once the saw has moved 10 mm."""
    return _level(objects)[0]
