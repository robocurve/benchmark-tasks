"""Take a wheelchair's armrest off for a sliding-board transfer. The wheelchair's left side frame stands on the table
with its outer side toward the arms and its front toward the right arm: two square arm sockets (receivers) fixed to
the frame, the seat beyond them. A removable desk-length armrest (1.2 kg) stands in the sockets on two square steel
posts 20 cm apart, each 10 cm deep in its socket.

A spring lock holds the armrest down: a steel catch on the front post stands 5 mm out through a window in the front
socket's wall. The catch and a red release tab under the front of the armrest are one part that slides along the
armrest: pushing the tab 7 mm toward the back of the chair draws the catch into the post. Lifting does nothing until
the tab is held back, and the two posts, with 1.5 mm of clearance in 10 cm of socket, bind if the armrest is lifted
unevenly. The robot must take the armrest off and lay it down on the table on its own side of the chair.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from chairkit import PartState, Slider, on_reset, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Take the armrest off the wheelchair and lay it down on the table on your side of the chair, so the "
               "patient can slide across to the bed.")

STEEL = (0.74, 0.75, 0.77)
FRAME = (0.12, 0.13, 0.15)        # black powder-coated frame
VINYL = (0.07, 0.07, 0.08)        # black vinyl arm pad
BLUE = (0.10, 0.20, 0.42)         # blue nylon seat and back upholstery
RED = (0.85, 0.10, 0.10)


def _mm(x0, x1, y0, y1, z0, z1, color):
    return (((x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000), ((x1 - x0) / 1000, (y1 - y0) / 1000,
                                                                      (z1 - z0) / 1000), color)


# -- the armrest, in its own frame: origin midway between the two post feet, y along the armrest (front toward -y) ---
ARM_MASS = 1.2
POST_Y = (-0.100, 0.100)     # front and rear post centres
POST = 0.022                 # square steel posts, 22 x 22 mm
PAD_Z = (0.292, 0.330)       # the arm pad, 38 mm thick, on top of the posts
PAD_X = 0.055                # 55 mm wide
PAD_Y = (-0.130, 0.130)      # 26 cm long (a desk-length arm)
DEPTH = 0.100                # each post stands 10 cm deep in its socket
LOCK_TRAVEL = 0.007          # pushing the tab 7 mm toward +y (the back of the chair) draws the catch fully in
CATCH_Z = (0.040, 0.055)     # the catch, 4 to 5.5 cm above the post foot
TAB = (-0.046, -0.011, -0.112, -0.088, 0.200, 0.212)   # the release tab (x0, x1, y0, y1, z0, z1)
CENTRE_Z = 0.165             # roughly the armrest's centre of mass, above the feet


def _armrest_boxes():
    h = POST * 500
    boxes = [_mm(-h, h, y * 1000 - h, y * 1000 + h, 0, PAD_Z[0] * 1000, STEEL) for y in POST_Y]   # posts
    boxes.append(_mm(-PAD_X * 500, PAD_X * 500, PAD_Y[0] * 1000, PAD_Y[1] * 1000, PAD_Z[0] * 1000, PAD_Z[1] * 1000,
                     VINYL))   # arm pad
    return boxes


def _lock():
    """The lock: one 50 g part on a slider along the armrest (y). A red tab (35 x 24 x 12 mm) stands out of the front
    post toward the arms, 20 cm above the post's foot; a steel catch (12 x 10 x 15 mm) stands 5 mm out of the post's
    front face, through a window in the front socket's wall. A 400 N/m spring holds it out; pushing the tab 7 mm toward
    the back of the chair draws the catch in, 3.5 mm clear of the socket wall."""
    tab = _mm(*(v * 1000 for v in TAB), RED)
    catch = _mm(-6, 6, -116, -106, CATCH_Z[0] * 1000, CATCH_Z[1] * 1000, STEEL)
    return Slider("lock", rest=(0.0, POST_Y[0], 0.12), boxes=(tab, catch), travel=LOCK_TRAVEL, rate=400.0,
                  damping=10.0, axis="Y", sign=1, mass=0.05)


# -- the wheelchair's side frame, in its own frame: origin on the table under the midpoint between the sockets -----
# Each socket is a square steel tube, 25 mm inside (1.5 mm clear of a post each side), 3 mm wall, on a 10 mm floor
# plate that the post foot stands on; its mouth is 11 cm above the table. The front socket's front wall has a window
# (16 mm wide, 46 to 67 mm above the table) that the catch stands in; lifting the armrest 2 mm brings the catch up to
# the window's top edge.
FLOOR = 0.010
SOCKET_TOP = FLOOR + DEPTH         # 0.110 above the table
SOCKET_IN = 0.025
WALL = 0.003
WINDOW_X = (-0.008, 0.008)
WINDOW_Z = (0.046, 0.067)


def _socket(py, window=False):
    py *= 1000
    i, o = SOCKET_IN * 500, SOCKET_IN * 500 + WALL * 1000
    t, f = SOCKET_TOP * 1000, FLOOR * 1000
    out = [_mm(-o, o, py - o, py + o, 0, f, STEEL),          # floor plate
           _mm(i, o, py - o, py + o, f, t, STEEL),            # +x wall (seat side)
           _mm(-o, -i, py - o, py + o, f, t, STEEL),          # -x wall (outer side, toward the arms)
           _mm(-i, i, py + i, py + o, f, t, STEEL)]           # +y wall (back)
    if window:
        wx0, wx1 = WINDOW_X[0] * 1000, WINDOW_X[1] * 1000
        wz0, wz1 = WINDOW_Z[0] * 1000, WINDOW_Z[1] * 1000
        out += [_mm(-i, wx0, py - o, py - i, f, t, STEEL), _mm(wx1, i, py - o, py - i, f, t, STEEL),
                _mm(wx0, wx1, py - o, py - i, f, wz0, STEEL), _mm(wx0, wx1, py - o, py - i, wz1, t, STEEL)]
    else:
        out.append(_mm(-i, i, py - o, py - i, f, t, STEEL))   # -y wall (front)
    return out


_o = SOCKET_IN * 500 + WALL * 1000   # 15.5 mm, a socket's outer half-width
CHAIR_BOXES = (
    _socket(POST_Y[0], window=True) + _socket(POST_Y[1])
    + [_mm(_o, _o + 25, y * 1000 - 12.5, y * 1000 + 12.5, 0, 135, FRAME) for y in POST_Y]   # frame uprights
    + [_mm(_o, _o + 25, -250, 225, 110, 135, FRAME),             # seat rail
       _mm(_o, _o + 25, 200, 225, 135, 420, FRAME),              # back cane (push handle above)
       _mm(_o + 25, 400, -240, 200, 110, 150, BLUE),             # seat cushion
       _mm(_o + 25, 400, 200, 215, 150, 420, BLUE)]              # backrest
)

# -- fixed layout (world frame: x forward from the arms, y left, metres) --------------------------------------------
SOCKETS_X = 0.460                     # the sockets' axis line, parallel to y
CHAIR = part("wheelchair", pos=(SOCKETS_X, 0.0, TOP), mass=None, boxes=CHAIR_BOXES, color=FRAME, friction=0.3,
             fixed=True)
ARMREST = part("armrest", pos=(SOCKETS_X, 0.0, TOP + FLOOR), mass=ARM_MASS, boxes=_armrest_boxes(),
               sliders=(_lock(),), color=VINYL, friction=0.3)
OBJECTS = [CHAIR, ARMREST]

# -- scoring -----------------------------------------------------------------------------------------------------------
STARTED = 0.020           # lifted 2 cm (the lock must have been released)
OUT_MARGIN = 0.005        # a post is out when its foot is 5 mm above its socket's mouth (or 3 cm to the side of it)
CLEAR_X = SOCKETS_X - 0.100   # clear of the chair: its centre at least 10 cm out from the sockets toward the arms
LYING_DEG = 75.0          # laid down on its side: tipped at least 75 degrees from upright ...
ON_TABLE = 0.003          # ... with its lowest point within 3 mm of the table
DROP_SPEED = 0.8          # let fall: moving down faster than 0.8 m/s ...
DROP_HEIGHT = 0.050       # ... with its lowest point more than 5 cm above the table
REST_SPEED = 0.010        # at rest: moving slower than 1 cm/s ...
REST_SPIN = 0.05          # ... and turning slower than 0.05 rad/s (3 degrees/s)
_STATE: dict = {}

# The corners of the posts' feet and of the pad, in the armrest's frame (for its lowest point).
_CORNERS = np.array([[sx * POST / 2, y + sy * POST / 2, 0.0] for sx in (-1, 1) for sy in (-1, 1) for y in POST_Y]
                    + [[sx * PAD_X / 2, y, z] for sx in (-1, 1) for y in PAD_Y for z in PAD_Z])


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(started=False, out=False, clear=False, dropped=False)
    _STATE["_held"], _STATE["_step"] = 0, None


_reset_state()


def lift(arm: PartState) -> float:
    """How far the armrest's post feet are above where they stand in the sockets (m)."""
    return float(arm.pos[2] - ARMREST.pos[2])


def tilt_deg(arm: PartState) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, arm.rotation()[2, 2]))))


def lowest(arm: PartState) -> float:
    """Height of the armrest's lowest point above the table (m)."""
    return float(min(arm.to_world(c)[2] for c in _CORNERS) - TOP)


def posts_out(arm: PartState) -> bool:
    for y in POST_Y:
        foot = arm.to_world((0.0, y, 0.0))
        side = math.hypot(foot[0] - SOCKETS_X, foot[1] - y)
        if not (foot[2] - TOP > SOCKET_TOP + OUT_MARGIN or side > 0.030):
            return False
    return True


def centre(arm: PartState) -> np.ndarray:
    return arm.to_world((0.0, 0.0, CENTRE_Z))


def _assess(objects) -> tuple[float, bool]:
    arm = objects["armrest"]
    low = lowest(arm)
    _STATE["dropped"] |= bool(arm.lin_vel[2] < -DROP_SPEED and low > DROP_HEIGHT)
    if _STATE["dropped"]:
        return 0.0, False
    _STATE["started"] |= lift(arm) >= STARTED
    out = posts_out(arm)
    _STATE["out"] |= out
    c = centre(arm)
    clear = bool(out and c[0] <= CLEAR_X)
    _STATE["clear"] |= clear
    still = bool(np.linalg.norm(arm.lin_vel) < REST_SPEED and np.linalg.norm(arm.ang_vel) < REST_SPIN)
    if clear and tilt_deg(arm) >= LYING_DEG and low <= ON_TABLE:
        return 1.0, bool(still)
    if _STATE["clear"]:
        return 0.75, False
    if _STATE["out"]:
        return 0.5, False
    return (0.25 if _STATE["started"] else 0.0), False


FULL_HOLD_STEPS = 20        # the success condition must hold this many control steps in a row (1 s) for 1.0
UNHELD = 0.75                # the score while it has held for less


def _update(objects) -> tuple[float, bool]:
    """This step's (score, success). success() and score() are both called on each step's states; the step is
    assessed once. The score is 1.0 only once the success condition has held for 1 s; until then it is UNHELD."""
    last = _STATE.get("_step")
    if last is not None and last[0] is objects and last[2] < 2:
        _STATE["_step"] = (last[0], last[1], last[2] + 1)
        return last[1]
    raw, ok = _assess(objects)
    _STATE["_held"] = _STATE.get("_held", 0) + 1 if ok else 0
    out = (raw if raw < 1.0 else 1.0 if _STATE["_held"] >= FULL_HOLD_STEPS else UNHELD, ok)
    _STATE["_step"] = (objects, out, 1)
    return out


def success(objects) -> bool:
    """The armrest off the chair and lying at rest on the table on the arms' side (the runtime ends the episode once
    this has held for 1 s)."""
    return _update(objects)[0] >= 1.0   # true only once the full score's 1 s hold is complete


def score(objects) -> float:
    """0.25 lifted 2 cm; 0.5 both posts out of their sockets; 0.75 clear of the chair; 1.0 lying on the table clear of
    it. 0 if it was ever let fall."""
    return _update(objects)[0]
