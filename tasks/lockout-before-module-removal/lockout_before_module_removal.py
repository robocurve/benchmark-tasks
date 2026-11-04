"""On the station table, in front of the left arm, a DIN rail lies flat with three relay modules on it: two grey ones
(fixed) either side of an orange one that must come out for repair. Its supply isolator, a fixed red box, stands
further left, already switched off, with a lock pocket on its top and a white tag pad beside it. A red safety padlock
and a yellow danger tag lie nearer the arms, and a black return tray waits further out. Lockout before work: the
padlock goes in the isolator's lock pocket and the tag on its pad before the orange module may leave the rail. The
order is checked: if the module leaves its slot while the lock or the tag is not in place, the removal earns nothing.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/lockout-before-module-removal/lockout_before_module_removal.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
RAIL_X = 0.330
RAIL = (0.035, 0.200, 0.0075)
RAIL_Y = 0.250
RAIL_TOP = TOP + RAIL[2]
MOD = (0.080, 0.045, 0.060)
GAP = 0.0015
SLOT_Y = RAIL_Y
NEIGHBOUR_Y = (RAIL_Y + MOD[1] + GAP, RAIL_Y - MOD[1] - GAP)

ISOLATOR = (0.060, 0.050, 0.040)
ISO_XY = (0.330, 0.420)
ISO_TOP = TOP + ISOLATOR[2]
POCKET_INNER = (0.036, 0.028)
POCKET_WALL = (0.005, 0.015)
TAG_PAD = (0.060, 0.050, 0.003)
TAG_PAD_XY = (0.230, 0.420)
TAG_PAD_TOP = TOP + TAG_PAD[2]

LOCK = (0.030, 0.020, 0.035)
LOCK_KG = 0.060
LOCK_START = (0.170, 0.270)
TAG = (0.050, 0.035, 0.006)
TAG_KG = 0.010
TAG_START = (0.170, 0.360)
RETURN = (0.215, 0.195)
RETURN_INNER = (0.110, 0.075)
RETURN_WALL = (0.006, 0.015)

ORANGE = (0.92, 0.48, 0.10)
GREY = (0.62, 0.63, 0.65)
RED = (0.78, 0.12, 0.10)
YELLOW = (0.95, 0.80, 0.10)
WHITE = (0.92, 0.92, 0.92)
BLACK = (0.08, 0.08, 0.08)
STEEL = (0.75, 0.76, 0.78)


def _frame(name, centre, inner, wall, base_z, color):
    cx, cy = centre
    ix, iy = inner
    t, h = wall
    z = base_z + h / 2
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + t / 2, z), size=(ix + 2 * t, t, h), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - t / 2, z), size=(ix + 2 * t, t, h), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - t / 2, cy, z), size=(t, iy, h), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + t / 2, cy, z), size=(t, iy, h), color=color),
    ]


MODULE = Box("relay_module", pos=(RAIL_X, SLOT_Y, RAIL_TOP + MOD[2] / 2), size=MOD, mass=0.150, color=ORANGE)
PADLOCK = Box("padlock", pos=(*LOCK_START, TOP + LOCK[2] / 2), size=LOCK, mass=LOCK_KG, color=RED)
DANGER_TAG = Box("danger_tag", pos=(*TAG_START, TOP + TAG[2] / 2), size=TAG, mass=TAG_KG, color=YELLOW)
FIXTURES = [
    Box("rail", pos=(RAIL_X, RAIL_Y, TOP + RAIL[2] / 2), size=RAIL, color=STEEL),
    *(Box(f"neighbour_{k + 1}", pos=(RAIL_X, y, RAIL_TOP + MOD[2] / 2), size=MOD, color=GREY) for k, y in enumerate(NEIGHBOUR_Y)),
    Box("isolator", pos=(*ISO_XY, TOP + ISOLATOR[2] / 2), size=ISOLATOR, color=RED),
    *_frame("lock_pocket", ISO_XY, POCKET_INNER, POCKET_WALL, ISO_TOP, BLACK),
    Box("tag_pad", pos=(*TAG_PAD_XY, TOP + TAG_PAD[2] / 2), size=TAG_PAD, color=WHITE),
    *_frame("return_tray", RETURN, RETURN_INNER, RETURN_WALL, TOP, BLACK),
]
OBJECTS = [MODULE, PADLOCK, DANGER_TAG, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _locked(st) -> bool:
    """The padlock at rest with its centre inside the lock pocket, resting on the isolator's top."""
    return bool(
        abs(float(st.pos[0]) - ISO_XY[0]) < POCKET_INNER[0] / 2
        and abs(float(st.pos[1]) - ISO_XY[1]) < POCKET_INNER[1] / 2
        and ISO_TOP - 0.002 < float(st.low[2]) < ISO_TOP + 0.005
        and st.resting()
    )


def _tagged(st) -> bool:
    """The tag at rest on the white pad, its centre inside the pad's outline."""
    return bool(
        abs(float(st.pos[0]) - TAG_PAD_XY[0]) < TAG_PAD[0] / 2
        and abs(float(st.pos[1]) - TAG_PAD_XY[1]) < TAG_PAD[1] / 2
        and TAG_PAD_TOP - 0.002 < float(st.low[2]) < TAG_PAD_TOP + 0.005
        and st.resting()
    )


def _in_slot(st) -> bool:
    """The module still in its slot: centre within 1 cm of the slot and bottom within 1 cm of the rail top."""
    return bool(
        math.hypot(float(st.pos[0]) - RAIL_X, float(st.pos[1]) - SLOT_Y) <= 0.010
        and float(st.low[2]) <= RAIL_TOP + 0.010
    )


def _returned(st) -> bool:
    return bool(
        abs(float(st.pos[0]) - RETURN[0]) < RETURN_INNER[0] / 2
        and abs(float(st.pos[1]) - RETURN[1]) < RETURN_INNER[1] / 2
        and TOP - 0.005 < float(st.low[2]) < TOP + 0.020
        and st.resting()
    )


# The trial's history: whether the module ever left its slot while the lock or the tag was not in place. The task file
# only sees object states, so the history resets whenever the scene is back in its start state (each new trial).
_HISTORY = {"violated": False}


def _at_start(objects) -> bool:
    def near(name, xy):
        st = objects[name]
        return math.hypot(float(st.pos[0]) - xy[0], float(st.pos[1]) - xy[1]) < 0.005

    return near("padlock", LOCK_START) and near("danger_tag", TAG_START) and _in_slot(objects["relay_module"])


def _update(objects) -> bool:
    """Record a violation if the module is out of its slot while the scene is not locked and tagged; returns it."""
    if _at_start(objects):
        _HISTORY["violated"] = False
    elif not _in_slot(objects["relay_module"]) and not (_locked(objects["padlock"]) and _tagged(objects["danger_tag"])):
        _HISTORY["violated"] = True
    return _HISTORY["violated"]


def success(objects) -> bool:
    """Locked, tagged, and the module in the return tray, having left its slot only after both; holds for 1 s."""
    violated = _update(objects)
    return (not violated and _locked(objects["padlock"]) and _tagged(objects["danger_tag"])
            and _returned(objects["relay_module"]))


def score(objects) -> float:
    """0.25 for the padlock in the isolator's lock pocket, 0.25 for the tag on its pad, and 0.5 for the module in the
    return tray, but only if it never left its slot before both were in place. Moving the module first, even if the lock
    and tag follow, forfeits the 0.5."""
    violated = _update(objects)
    s = 0.25 * _locked(objects["padlock"]) + 0.25 * _tagged(objects["danger_tag"])
    if not violated and _returned(objects["relay_module"]):
        s += 0.5
    return s


def violated() -> bool:
    """Whether this trial has moved the module out of its slot before lockout (for the log)."""
    return _HISTORY["violated"]


def distance(objects) -> float:
    m, lk, tg = objects["relay_module"], objects["padlock"], objects["danger_tag"]
    return float(
        math.hypot(float(lk.pos[0]) - ISO_XY[0], float(lk.pos[1]) - ISO_XY[1])
        + math.hypot(float(tg.pos[0]) - TAG_PAD_XY[0], float(tg.pos[1]) - TAG_PAD_XY[1])
        + math.hypot(float(m.pos[0]) - RETURN[0], float(m.pos[1]) - RETURN[1])
    )


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
TRAVEL = TOP + 0.150
SQUEEZE = 0.008


def _move(pick, place, *, size, yaw, depth, pick_bottom, place_bottom, label, open_):
    (px, py), (qx, qy) = pick, place
    width = size[1] if yaw == 0.0 else size[0]
    gz = pick_bottom + size[2] - depth + TIP
    qz = place_bottom + size[2] - depth + TIP + 0.002
    grip = -(width - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), open_, f"{label}: over", yaw),
        Waypoint(1.5, (px, py, gz + 0.020), open_, f"{label}: descend", yaw),
        Waypoint(1.0, (px, py, gz), open_, f"{label}: down slowly", yaw),
        Waypoint(0.5, (px, py, gz), open_, f"{label}: settle", yaw),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close", yaw),
        Waypoint(1.5, (px, py, TRAVEL), grip, f"{label}: lift", yaw),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{label}: carry", yaw),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{label}: settle over", yaw),
        Waypoint(1.5, (qx, qy, qz + 0.020), grip, f"{label}: lower", yaw),
        Waypoint(1.5, (qx, qy, qz), grip, f"{label}: set down slowly", yaw),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle", yaw),
        Waypoint(1.5, (qx, qy, qz), open_, f"{label}: release", yaw),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{label}: retreat", yaw),
    ]


def demo():
    """Left arm only, in the lockout order: the padlock into the isolator's lock pocket (fingers across its 20 mm, the
    pads above the pocket walls), the tag onto its pad, then the module straight up off the rail and into the return
    tray (fingers fully open across its 80 mm depth). 49.5 s."""
    lock = _move(LOCK_START, ISO_XY, size=LOCK, yaw=0.0, depth=0.012, pick_bottom=TOP, place_bottom=ISO_TOP,
                 label="padlock", open_=-(LOCK[1] + 0.024) / 2)
    tag = _move(TAG_START, TAG_PAD_XY, size=TAG, yaw=0.0, depth=0.004, pick_bottom=TOP, place_bottom=TAG_PAD_TOP,
                label="tag", open_=-(TAG[1] + 0.024) / 2)
    module = _move((RAIL_X, SLOT_Y), RETURN, size=MOD, yaw=math.pi / 2, depth=0.025, pick_bottom=RAIL_TOP,
                   place_bottom=TOP, label="module", open_=FINGER_OPEN)
    return {"left": lock + tag + module}


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states, so the history is exercised too."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(size, x, y, bottom, vel=(0.0, 0.0, 0.0)):
        half = np.array(size) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {"relay_module": box(MOD, RAIL_X, SLOT_Y, RAIL_TOP), "padlock": box(LOCK, *LOCK_START, TOP),
             "danger_tag": box(TAG, *TAG_START, TOP)}
    locked = {"padlock": box(LOCK, *ISO_XY, ISO_TOP)}
    tagged = {"danger_tag": box(TAG, *TAG_PAD_XY, TAG_PAD_TOP)}
    lifted = {"relay_module": box(MOD, RAIL_X, SLOT_Y, RAIL_TOP + 0.05, vel=(0, 0, 0.05))}
    returned = {"relay_module": box(MOD, *RETURN, TOP)}

    def run(*frames):
        """Feed the states in order (as the simulator would, step by step) and return the last score and success."""
        for f in frames[:-1]:
            score(f)
        return score(frames[-1]), success(frames[-1])

    s1 = start | locked
    s2 = s1 | tagged
    cases = {
        "start": ((start,), 0.0, False),
        "locked only": ((start, s1), 0.25, False),
        "locked and tagged": ((start, s1, s2), 0.5, False),
        "lockout, then the module out": ((start, s1, s2, s2 | lifted, s2 | returned), 1.0, True),
        "module out first, then lockout": ((start, start | returned, s1 | returned, s2 | returned), 0.5, False),
        "locked, module out before the tag": ((start, s1, s1 | lifted, s1 | returned, s2 | returned), 0.5, False),
        "module out with no lockout at all": ((start, start | lifted, start | returned), 0.0, False),
        "a new trial after a violation starts clean": ((start, start | returned, start, s1, s2, s2 | lifted, s2 | returned), 1.0, True),
        "lockout, module lifted but put back": ((start, s1, s2, s2 | lifted, s2), 0.5, False),
        "padlock on the table beside the isolator": ((start, start | {"padlock": box(LOCK, ISO_XY[0] - 0.06, ISO_XY[1], TOP)}), 0.0, False),
        "tag still sliding on the pad": ((start, start | {"danger_tag": box(TAG, *TAG_PAD_XY, TAG_PAD_TOP, vel=(0.02, 0, 0))}), 0.0, False),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    score(start)  # leave the history clean
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the left arm's plan replayed through the
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
