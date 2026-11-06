"""On the station table, between the two arms, an open bay of a GPU server chassis: two fixed steel guide rails 35.6 cm
apart running away from the arms and, across their far ends, a fixed dark backplane bar where the sled's connectors
mate. A loose GPU sled (a wide, shallow box, 34 cm wide and 13 cm deep) has its far 3.5 cm entered between the
guides, square to them. It must be pushed home with both arms at once, one at each side of its near face, until it
meets the backplane, without racking: the sled may turn no more than 2 degrees while it is in the guides, checked
at every step. Scored from the sled's pose at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/gpu-sled-two-hand-insert/gpu_sled_two_hand_insert.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
SLED = (0.130, 0.340, 0.035)  # depth (x, the insertion direction), width (y), height
SLED_KG = 0.500
GAP = 0.004  # air between each side of the sled and its guide rail
BACK_X = 0.440  # the backplane bar's near face: a seated sled's far face
SEAT_X = BACK_X - SLED[0] / 2
RAIL = (0.120, 0.008, 0.025)
RAIL_X0 = BACK_X - RAIL[0]  # the guides' open (near) ends
RAIL_Y = SLED[1] / 2 + GAP + RAIL[1] / 2
BACKPLANE = (0.012, 2 * RAIL_Y + RAIL[1], 0.030)
START_X = RAIL_X0 + 0.035 - SLED[0] / 2  # far face 3.5 cm inside the guides

RACK_DEG = 2.0  # racked: turned more than this while in the guides
SEAT_GAP = 0.002  # seated: far face within 2 mm of the backplane
SEAT_DEG = 1.0  # and square within 1 degree, so the connectors mate

GREY = (0.30, 0.32, 0.35)
STEEL = (0.72, 0.73, 0.75)
DARK = (0.12, 0.13, 0.15)

THE_SLED = Box("gpu_sled", pos=(START_X, 0.0, TOP + SLED[2] / 2), size=SLED, mass=SLED_KG, color=GREY)
FIXTURES = [
    *(Box(f"guide_{s}", pos=(RAIL_X0 + RAIL[0] / 2, sy * RAIL_Y, TOP + RAIL[2] / 2), size=RAIL, color=STEEL)
      for s, sy in (("left", 1.0), ("right", -1.0))),
    Box("backplane", pos=(BACK_X + BACKPLANE[0] / 2, 0.0, TOP + BACKPLANE[2] / 2), size=BACKPLANE, color=DARK),
]
OBJECTS = [THE_SLED, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _yaw_deg(quat_xyzw) -> float:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _tilt(quat_xyzw) -> float:
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _far_face(st) -> float:
    """The sled's far face along x (its centre plus half its depth)."""
    return float(st.pos[0]) + SLED[0] / 2


def _in_guides(st) -> bool:
    return _far_face(st) > RAIL_X0 and abs(float(st.pos[1])) < RAIL_Y


def _seated(st) -> bool:
    return bool(
        BACK_X - SEAT_GAP <= _far_face(st) <= BACK_X + 0.002
        and abs(_yaw_deg(st.quat_xyzw)) <= SEAT_DEG
        and _tilt(st.quat_xyzw) <= 3.0
        and abs(float(st.low[2]) - TOP) <= 0.003
        and st.resting()
    )


# The trial's history: whether the sled has ever been racked in its guides. The task file only sees object states, so
# the history resets whenever the scene is back in its start state (each new trial).
_HISTORY = {"racked": False}


def _update(objects) -> bool:
    st = objects["gpu_sled"]
    if math.dist([float(v) for v in st.pos], THE_SLED.pos) < 0.002 and abs(_yaw_deg(st.quat_xyzw)) < 0.2:
        _HISTORY["racked"] = False
    elif _in_guides(st) and abs(_yaw_deg(st.quat_xyzw)) > RACK_DEG:
        _HISTORY["racked"] = True
    return _HISTORY["racked"]


def racked() -> bool:
    """Whether this trial has turned the sled more than 2 degrees in its guides (for the log)."""
    return _HISTORY["racked"]


def score(objects) -> float:
    """0.25 once the sled is pushed 3 cm further in, 0.5 at 6 cm, 0.75 with its far face within 2 mm of the
    backplane, 1.0 seated: there, square within 1 degree, flat and still. A sled ever turned more than 2 degrees while
    in its guides is racked and scores 0 for the rest of the trial."""
    if _update(objects):
        return 0.0
    st = objects["gpu_sled"]
    if _seated(st):
        return 1.0
    if BACK_X - SEAT_GAP <= _far_face(st) <= BACK_X + 0.002:
        return 0.75
    travel = float(st.pos[0]) - START_X
    if travel >= 0.060:
        return 0.5
    if travel >= 0.030:
        return 0.25
    return 0.0


def success(objects) -> bool:
    """Seated against the backplane, square, never racked; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres still to push."""
    return float(max(0.0, BACK_X - _far_face(objects["gpu_sled"])))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers closed along x, each closed pad pair a 38 mm face across y;
# the sign keeps each wrist inside its joint limits this close to the centre line
TIP = 0.013
PAD = 0.010  # a closed pad's outer face stands 10 mm from the TCP along the closing axis
TRAVEL = TOP + 0.110
PUSH_Z = TOP + 0.004 + TIP  # pad tips 4 mm above the table: the pads bear on the sled's lower 3 cm
PUSH_Y = 0.120  # each arm pushes 12 cm from the sled's centre line (24 cm apart)
CLEAR = 0.015
# Each pushing arm trails its command by about 5.5 mm per newton (measured in sim: with 12 mm commanded past contact
# the 500 g sled stopped 1.2 mm short of the backplane), so the push is commanded 2 cm past contact.
OVERPUSH = 0.020


def demo():
    """Both arms at once, mirrored and in step: closed fingers down 1.5 cm behind the sled's near face, 12 cm either
    side of its centre line, then one slow push together (about 1.3 cm/s) until it meets the backplane, a short
    hold, back off and up. 17.8 s."""
    near = START_X - SLED[0] / 2
    sx = near - PAD - CLEAR
    ex = SEAT_X - SLED[0] / 2 - PAD + OVERPUSH
    bx = ex - OVERPUSH - 0.012  # back off past the lag before lifting
    c = FINGER_CLOSED
    plans = {}
    for side, sy in (("left", 1.0), ("right", -1.0)):
        y, yaw = sy * PUSH_Y, YAW[side]
        plans[side] = [
            Waypoint(1.8, (sx, y, TRAVEL), c, "over the near face", yaw),
            Waypoint(1.5, (sx, y, PUSH_Z + 0.020), c, "descend behind the sled", yaw),
            Waypoint(1.0, (sx, y, PUSH_Z), c, "down slowly", yaw),
            Waypoint(0.8, (sx, y, PUSH_Z), c, "settle", yaw),
            Waypoint(9.5, (ex, y, PUSH_Z), c, "push home together", yaw),
            Waypoint(1.0, (ex, y, PUSH_Z), c, "hold against the backplane", yaw),
            Waypoint(1.0, (bx, y, PUSH_Z), c, "back off", yaw),
            Waypoint(1.2, (bx, y, TRAVEL), c, "up", yaw),
        ]
    return plans


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states, so the racking history is exercised too."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def sled(x, y=0.0, yaw_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw = math.radians(yaw_deg)
        q = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        half = np.array([c * SLED[0] / 2 + s * SLED[1] / 2, s * SLED[0] / 2 + c * SLED[1] / 2, SLED[2] / 2])
        pos = np.array([x, y, TOP + SLED[2] / 2])
        return {"gpu_sled": ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)}

    def run(*frames):
        for f in frames[:-1]:
            score(f)
        return score(frames[-1]), success(frames[-1])

    start = sled(START_X)
    mid = sled(START_X + 0.040, vel=(0.013, 0, 0))
    seated = sled(SEAT_X - 0.0005)
    cases = {
        "start": ((start,), 0.0, False),
        "pushed 4 cm, square": ((start, mid), 0.25, False),
        "pushed 7 cm, square": ((start, sled(START_X + 0.070, vel=(0.013, 0, 0))), 0.5, False),
        "seated square": ((start, mid, seated), 1.0, True),
        "seated, 0.8 degrees off": ((start, mid, sled(SEAT_X - 0.001, yaw_deg=0.8)), 1.0, True),
        "at the backplane but 1.5 degrees off": ((start, mid, sled(SEAT_X - 0.001, yaw_deg=1.5)), 0.75, False),
        "at the backplane, still moving": ((start, mid, sled(SEAT_X - 0.001, vel=(0.02, 0, 0))), 0.75, False),
        "4 mm short of the backplane": ((start, mid, sled(SEAT_X - 0.004)), 0.5, False),
        "racked 3 degrees on the way, squared up after": ((start, sled(START_X + 0.03, yaw_deg=3.0), seated), 0.0, False),
        "racked the other way": ((start, sled(START_X + 0.05, yaw_deg=-2.5), seated), 0.0, False),
        "a new trial after racking starts clean": ((start, sled(START_X + 0.03, yaw_deg=3.0), start, mid, seated), 1.0, True),
        "pulled out of the guides and turned there is no racking": ((start, sled(START_X - 0.06, yaw_deg=5.0), seated), 1.0, True),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    score(start)  # leave the history clean
    assert not racked()
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
