"""On a plumbing training board on the bench, a 3/4 in copper pipe runs left to right between two pipe stands. It has
a split marked by a red band where it leaks. Beside it on the bench stands a hinged pipe repair clamp: two
rubber-lined half shells hinged together along the top, sprung open, with a snap latch where their lower edges meet.

The robot must clamp the leak: set the clamp over the pipe centred on the red band and close it round the pipe until
its latch snaps shut, so the clamp's lining covers the split. The clamp's halves only close round the pipe once it sits
on it, and the latch only catches when both lower edges are pressed together under the pipe.

Everything is a real product at true size: a 3/4 in copper pipe (22.2 mm outside) and a 75 mm hinged repair clamp
(80 g).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import meshkit as mk  # noqa: E402
import numpy as np  # noqa: E402
from partkit import (Hinge, PartState, finger_points, on_reset, on_step, part, set_hinge_stiffness,  # noqa: E402
                     set_hinge_target)
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Clamp the leak on this pipe with the repair clamp."

COPPER = (0.76, 0.45, 0.25)
RED = (0.80, 0.10, 0.08)
BOARD = (0.70, 0.62, 0.48)
STEEL = (0.74, 0.75, 0.77)
RUBBER = (0.10, 0.10, 0.11)

# -- the pipe (fixed), along y, on two stands
PIPE_X, PIPE_Z = 0.320, 0.075                     # its axis
PIPE_R = 0.0111                                   # 22.2 mm outside
PIPE_Y = (-0.170, 0.170)
LEAK_Y, LEAK_W = -0.040, 0.020                    # the red band over the split
_pipe = mk.revolve([(0.0, PIPE_Y[0]), (PIPE_R, PIPE_Y[0]), (PIPE_R, PIPE_Y[1]), (0.0, PIPE_Y[1])], segments=40)
PIPE = mk.mesh_part("pipe", pos=(PIPE_X, 0.0, TOP + PIPE_Z), shapes=[
    mk.Shape(_pipe.rotated("X", -90), color=COPPER, metallic=0.7, roughness=0.35),
    mk.Shape(mk.revolve([(0.0, LEAK_Y - LEAK_W / 2), (PIPE_R + 0.0003, LEAK_Y - LEAK_W / 2),
                         (PIPE_R + 0.0003, LEAK_Y + LEAK_W / 2), (0.0, LEAK_Y + LEAK_W / 2)], segments=40)
             .rotated("X", -90), color=RED, collider="none")])
SCENERY = [Box("training_board", pos=(PIPE_X, 0.0, TOP + 0.003), size=(0.120, 0.400, 0.006), color=BOARD)]
for k, y in enumerate((-0.150, 0.150)):
    SCENERY.append(Box(f"pipe_stand_{k}", pos=(PIPE_X, y, TOP + (PIPE_Z - PIPE_R) / 2 + 0.003),
                       size=(0.030, 0.020, PIPE_Z - PIPE_R - 0.006), color=STEEL))

# -- the clamp (loose partkit part). Part frame: origin on its bore's axis; y along the bore; the hinge along the top.
BORE_R = PIPE_R + 0.0015                          # the lining's bore: 1.5 mm clear of the pipe
WALL = 0.003
CLAMP_L = 0.075
STAVES = 7
HINGE_Z = BORE_R + WALL + 0.003                   # the hinge pin, above the bore
OPEN = math.radians(28.0)                         # each half's sprung-open stop
LATCH = math.radians(2.0)                         # squeezed to within 2 degrees of shut, the latch snaps
SPRING_K = 0.30                                   # N m per rad holding each half open
CLAMP_MASS = 0.080


def _rot_y(a):
    return (math.cos(a / 2), 0.0, math.sin(a / 2), 0.0)


def _half(side: int):
    """One half shell's boxes, shut (side +1: the far half, x > 0; -1: the near half). Staves from the top to the
    bottom of the bore on that side, a lug at the top round the hinge, and a latch lug along its lower edge."""
    r = BORE_R + WALL / 2
    out = []
    for k in range(STAVES):
        a = math.radians(90.0 - 180.0 * (k + 0.5) / STAVES)        # +90 (top) down to -90 (bottom), far side
        x, z = side * r * math.cos(a), r * math.sin(a)
        chord = 2 * r * math.tan(math.radians(180.0 / STAVES) / 2) * 1.06
        out.append(((x, 0.0, z), (WALL, CLAMP_L, chord), STEEL, _rot_y(-side * a if side > 0 else math.pi + a)))
    out.append(((side * 0.004, 0.0, HINGE_Z), (0.008, CLAMP_L, 0.006), STEEL))                       # hinge leaf
    out.append(((side * 0.003, 0.0, -BORE_R - WALL - 0.003), (0.006, CLAMP_L * 0.6, 0.006), STEEL))  # latch lug
    out.append(((side * (BORE_R + WALL + 0.004), 0.0, -0.002), (0.005, CLAMP_L * 0.7, 0.014), RUBBER))  # grip pad
    return out


def _open(boxes, side):
    """Turn shut-pose boxes about the hinge pin to the open stop (the half swung out)."""
    a = -side * OPEN                                # a -y turn swings the far half's foot out to +x
    c, s = math.cos(a), math.sin(a)
    out = []
    for b in boxes:
        (x, y, z), size, color = b[0], b[1], b[2]
        q = b[3] if len(b) > 3 else (1.0, 0.0, 0.0, 0.0)
        dz = z - HINGE_Z
        nx, nz = x * c + dz * s, HINGE_Z - x * s + dz * c
        qa = _rot_y(a)
        qq = (qa[0] * q[0] - qa[2] * q[2], qa[0] * q[1] + qa[2] * q[3], qa[0] * q[2] + qa[2] * q[0],
              qa[0] * q[3] - qa[2] * q[1])
        out.append(((nx, y, nz), size, color, qq))
    return tuple(out)


def half_name(side: int) -> str:
    return "half_far" if side > 0 else "half_near"


def _shut_angle(side: int) -> float:
    return side * OPEN


def _hinge(side: int) -> Hinge:
    lo, hi = sorted((0.0, _shut_angle(side) + side * math.radians(1.0)))
    return Hinge(half_name(side), pivot=(0.0, 0.0, HINGE_Z), boxes=_open(_half(side), side), lower=lo, upper=hi,
                 stiffness=SPRING_K, damping=0.01, target=0.0, mass=0.030, axis="Y")


CLAMP_START = (0.200, -0.200)
_foot = HINGE_Z - math.cos(OPEN) * (HINGE_Z + BORE_R + WALL + 0.006)   # its lowest point below the hinge, opened
CLAMP = part("clamp", pos=(CLAMP_START[0], CLAMP_START[1], TOP - _foot + 0.002), mass=CLAMP_MASS, color=STEEL,
             friction=0.8, boxes=[((0.0, 0.0, HINGE_Z + 0.006), (0.010, CLAMP_L + 0.012, 0.010), STEEL)],   # hinge knuckle
             cylinders=[((0.0, 0.0, HINGE_Z), 0.004, CLAMP_L + 0.012, STEEL, "Y")],
             hinges=(_hinge(1), _hinge(-1)))
OBJECTS = [PIPE, *SCENERY, CLAMP]

# -- scoring ------------------------------------------------------------------------------------------------------------
ON_PIPE_TOL = 0.004                               # the clamp's bore axis within 4 mm of the pipe's
COVER_TOL = 0.012                                 # its middle within 12 mm of the leak's along the pipe
ALIGN = math.radians(10.0)
CLEAR = 0.030
STILL_STEPS = 20
STILL_TOL = 0.001
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(latched={1: False, -1: False}, closed_once=False, clear=False, recent=[], steps=0)


_reset_state()


def shut_by(clamp: PartState, side: int) -> float:
    return abs(_shut_angle(side) - float(clamp.hinges.get(half_name(side), 0.0)))


def axis_off(clamp: PartState) -> float:
    """How far the clamp's bore axis (at its middle) is from the pipe's axis (m), in the pipe's cross-section."""
    p = np.asarray(clamp.pos, float)
    return math.hypot(float(p[0]) - PIPE_X, float(p[2]) - (TOP + PIPE_Z))


def aligned(clamp: PartState) -> bool:
    return abs(float(clamp.rotation()[1, 1])) >= math.cos(ALIGN)


def on_pipe(clamp: PartState) -> bool:
    return axis_off(clamp) <= ON_PIPE_TOL and aligned(clamp)


def over_leak(clamp: PartState) -> bool:
    return abs(float(clamp.pos[1]) - LEAK_Y) <= COVER_TOL


def latched(_clamp=None) -> bool:
    return all(_STATE["latched"].values())


def box_distance(point, clamp: PartState) -> float:
    p = clamp.to_local(point)
    r = math.hypot(float(p[0]), float(p[2]))
    return math.hypot(max(0.0, r - 0.032), max(0.0, abs(float(p[1])) - CLAMP_L / 2 - 0.006))


def observe(clamp: PartState, fingers) -> None:
    s = _STATE
    both = True
    for side in (1, -1):
        gap = shut_by(clamp, side)
        if gap > math.radians(6.0):
            s["latched"][side] = False
        both = both and gap <= LATCH
    if both and on_pipe(clamp):
        s["latched"] = {1: True, -1: True}          # the latch snaps once both feet meet round the pipe
    if on_pipe(clamp) and over_leak(clamp):
        s["closed_once"] = True
    s["clear"] = fingers is not None and all(box_distance(f, clamp) >= CLEAR for f in fingers)
    s["recent"].append(np.asarray(clamp.pos, float))
    del s["recent"][:-STILL_STEPS]
    s["steps"] += 1


@on_step
def _catch(environment) -> None:
    """Track the trial, and the latch: snapped, it holds both halves shut; open, their spring holds them apart."""
    from partkit import _part_state

    clamp = _part_state(environment, CLAMP)
    try:
        fingers = [np.asarray(p, float) for p in finger_points(environment)]
    except Exception as exc:  # noqa: BLE001 - an unreadable gripper never counts as clear
        if not _STATE.get("warned"):
            print(f"pipe-repair-clamp: gripper check unavailable: {exc!r}", flush=True)
            _STATE["warned"] = True
        fingers = None
    observe(clamp, fingers)
    for side in (1, -1):
        if _STATE["latched"][side]:
            set_hinge_target(environment, CLAMP.name, half_name(side), _shut_angle(side))
            set_hinge_stiffness(environment, CLAMP.name, half_name(side), 5.0)
        else:
            set_hinge_target(environment, CLAMP.name, half_name(side), 0.0)
            set_hinge_stiffness(environment, CLAMP.name, half_name(side), SPRING_K)


def _still() -> bool:
    recent = _STATE["recent"]
    return len(recent) == STILL_STEPS and float(np.ptp(np.array(recent), axis=0).max()) < STILL_TOL


def distance(objects) -> float:
    """Metres from the clamp's bore to its place on the pipe over the leak, plus 1 cm per 10 degrees it is open."""
    clamp = objects["clamp"]
    return (axis_off(clamp) + abs(float(clamp.pos[1]) - LEAK_Y)
            + 0.057 * sum(shut_by(clamp, s) for s in (1, -1)))


def _level(objects) -> tuple[float, bool]:
    clamp = objects["clamp"]
    on, over = on_pipe(clamp), over_leak(clamp)
    if latched() and on and over:
        if _still() and _STATE["clear"]:
            return 1.0, True
        return 0.75, False
    if latched() and on:
        return 0.5, False
    if _STATE["closed_once"]:
        return 0.25, False
    return 0.0, False


def success(objects) -> bool:
    """The clamp latched shut round the pipe over the leak: its bore within 4 mm of the pipe's axis and lined up with
    it, its middle within 12 mm of the red band's; both grippers 3 cm clear and nothing moving for 1 s."""
    return _level(objects)[1]


def score(objects) -> float:
    """1.0 success; 0.75 latched over the leak but a gripper is near or it still moves; 0.5 latched round the pipe but
    not over the leak; 0.25 the clamp sat on the pipe over the leak at some point; else 0."""
    return _level(objects)[0]
