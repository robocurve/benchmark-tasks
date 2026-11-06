"""A batch queue holds jobs in order, and an urgent job arriving at the intake must go to the head of the queue
while every waiting job keeps its place relative to the others. The jobs have to be shifted back one slot at a
time, starting with the one at the far end, or they collide. Here: a lane of five taped pads, the first one green.
Four coloured boxes wait on pads 1 to 4, the fifth is empty, a red box waits on the yellow intake pad. Put the red box on the green
pad and shift the other four back one pad each, in the same order. The left arm reaches only the first pads
and the intake, the right arm only the last pads; the middle pad is shared.
"""

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
LANE_X = 0.29
PITCH = 0.12
SLOT_Y = [0.24 - PITCH * i for i in range(5)]  # pad 1 (green, head of the lane) to pad 5, left to right
INTAKE = (0.41, 0.24)
BOX = (0.06, 0.06, 0.08)
MASS = 0.1
PAD_T, PAD_SIZE = 0.004, 0.08
TOL = 0.02
GREEN, GREY, YELLOW, MAT = (0.15, 0.65, 0.25), (0.55, 0.57, 0.62), (0.95, 0.80, 0.10), (0.20, 0.25, 0.35)
RED = (0.85, 0.08, 0.08)
JOBS = {"job_blue": (0.15, 0.30, 0.85), "job_white": (0.95, 0.95, 0.95),
        "job_black": (0.10, 0.10, 0.10), "job_purple": (0.60, 0.25, 0.75)}  # in queue order, pad 1 to pad 4
NAMES = list(JOBS)
BOX_Z = TOP + 0.003 + PAD_T + BOX[2] / 2


def _pad(name, x, y, color):
    return Box(name, pos=(x, y, TOP + 0.003 + PAD_T / 2), size=(PAD_SIZE, PAD_SIZE, PAD_T), color=color)


OBJECTS = [
    Box("lane_mat", pos=(LANE_X, 0.0, TOP + 0.0015), size=(0.12, 0.62, 0.003), color=MAT),
    Box("intake_mat", pos=(INTAKE[0], INTAKE[1], TOP + 0.0015), size=(0.10, 0.10, 0.003), color=MAT),
    *[_pad(f"pad_{i + 1}", LANE_X, y, GREEN if i == 0 else GREY) for i, y in enumerate(SLOT_Y)],
    _pad("intake_pad", *INTAKE, YELLOW),
    Box("job_red", pos=(INTAKE[0], INTAKE[1], BOX_Z), size=BOX, color=RED, mass=MASS),
    *[Box(n, pos=(LANE_X, SLOT_Y[i], BOX_Z), size=BOX, color=c, mass=MASS) for i, (n, c) in enumerate(JOBS.items())],
]


def _upright(o):
    return 1 - 2 * (float(o.quat_xyzw[0]) ** 2 + float(o.quat_xyzw[1]) ** 2) > 0.99


def _near(o, y):
    return ((float(o.pos[0]) - LANE_X) ** 2 + (float(o.pos[1]) - y) ** 2) ** 0.5 <= TOL


def score(objects):
    """All five boxes must be upright and on the table, else 0. The red box must be within 2 cm of pad 1.
    Each other box is matched to the pad (2 to 5) it is within 2 cm of; they must fill pads 2 to 5 one each.
    1.0 if they are in the original order; 0.5 if exactly one adjacent pair is swapped; otherwise 0."""
    boxes = [objects["job_red"]] + [objects[n] for n in NAMES]
    if not all(_upright(o) and float(o.low[2]) < TOP + 0.015 for o in boxes):
        return 0.0
    if not _near(objects["job_red"], SLOT_Y[0]):
        return 0.0
    slot_of = []
    for n in NAMES:
        hit = [k for k in range(1, 5) if _near(objects[n], SLOT_Y[k])]
        if not hit:
            return 0.0
        slot_of.append(hit[0])
    if sorted(slot_of) != [1, 2, 3, 4]:
        return 0.0
    swaps = sum(1 for i in range(4) for j in range(i + 1, 4) if slot_of[i] > slot_of[j])
    return 1.0 if swaps == 0 else 0.5 if swaps == 1 and slot_of != sorted(slot_of) else 0.0


TRAVEL_Z = TOP + 0.16
GRIP_Z = BOX_Z - BOX[2] / 2 + PAD_TIP_PAST_TCP + 0.032  # pad tips 3.2 cm above the pad top
PARK = {"left": (0.16, 0.32, TOP + 0.20), "right": (0.12, -0.38, TOP + 0.20)}


LAG = {0: (0.021, 0.014), 1: (0.021, 0.010), 2: (0.021, 0.003), 3: (0.021, 0.029), 4: (0.021, 0.025)}  # measured settle offset (x, y) by destination pad


def _carry(a, b, lag=(0.0, 0.0)):
    w = pick_place_waypoints((a[0], a[1], GRIP_Z), (b[0] + lag[0], b[1] + lag[1], GRIP_Z + 0.003), travel_z=TRAVEL_Z)
    w.insert(3, Waypoint(0.6, (a[0], a[1], GRIP_Z), FINGER_OPEN, "settle"))
    return w


def _lane(i):
    return (LANE_X, SLOT_Y[i])


def _dur(wps):
    return sum(w.duration for w in wps)


def demo():
    """Far end first: the right arm moves the box on pad 4 to pad 5 and the one on pad 3 to pad 4, then the left arm shifts
    the boxes on pads 2 and 1 back one pad each, then fetches the red box from the intake."""
    right = _carry(_lane(3), _lane(4), LAG[4]) + _carry(_lane(2), _lane(3), LAG[3])
    left = [Waypoint(_dur(right) - 2.0, PARK["left"], FINGER_OPEN, "wait for the far end")]
    for i in (1, 0):
        left += _carry(_lane(i), _lane(i + 1), LAG[i + 1])
    left += _carry(INTAKE, _lane(0), LAG[0])
    left.append(Waypoint(2.0, PARK["left"], FINGER_OPEN, "move clear"))
    right.append(Waypoint(2.0, PARK["right"], FINGER_OPEN, "move clear"))
    return {"left": left, "right": right}
