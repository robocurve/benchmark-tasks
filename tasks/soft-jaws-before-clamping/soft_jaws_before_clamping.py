"""On the station table, in front of the left arm, stands a machine vise: a fixed dark base with two fixed hardened steel
jaws facing each other across x, open 6 cm apart. Two brass soft-jaw plates stand upright on the table nearer the
arms, and a finished blue aluminium part (already machined, its faces must not be marked) stands on a white tray to
the right. Machine-shop practice: a finished surface is never clamped against the bare serrated steel jaws. Soft jaws
go on first, one against the inner face of each steel jaw, and only then is the part set down between them. The order
is checked: if the part enters the vise while either soft jaw is missing, seating it earns nothing.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/soft-jaws-before-clamping/soft_jaws_before_clamping.py \\
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
VX, VY = 0.330, 0.270  # the vise centre
BASE = (0.120, 0.090, 0.010)
BED = TOP + BASE[2]
JAW = (0.020, 0.050, 0.035)
JAW_FACE = 0.030  # each steel jaw's inner face is 3 cm from the vise centre
JAW_TOP = BED + JAW[2]

SOFT = (0.010, 0.070, 0.022)  # brass soft-jaw plate: 1 cm thick, 7 cm long (overhangs the 5 cm jaw), 2.2 cm tall
SOFT_KG = 0.050
SOFT_X = (VX + JAW_FACE - SOFT[0] / 2, VX - JAW_FACE + SOFT[0] / 2)  # fitted: flat against the far / near jaw
SOFT_STARTS = ((0.180, 0.300), (0.235, 0.300))

PART = (0.026, 0.040, 0.060)  # 6 cm tall, 2.6 cm thick: 7 mm clear of each soft jaw (sim runs 2-3)
PART_KG = 0.170
TRAY_XY = (0.200, 0.170)
TRAY = (0.080, 0.080, 0.004)
TRAY_TOP = TOP + TRAY[2]

DARK = (0.22, 0.23, 0.25)
STEEL = (0.50, 0.52, 0.55)
BRASS = (0.80, 0.65, 0.25)
BLUE = (0.15, 0.35, 0.80)
WHITE = (0.92, 0.92, 0.92)

SOFT_JAWS = [Box(f"soft_jaw_{k + 1}", pos=(x, y, TOP + SOFT[2] / 2), size=SOFT, mass=SOFT_KG, color=BRASS)
             for k, (x, y) in enumerate(SOFT_STARTS)]
FINISHED_PART = Box("finished_part", pos=(*TRAY_XY, TRAY_TOP + PART[2] / 2), size=PART, mass=PART_KG, color=BLUE)
FIXTURES = [
    Box("vise_base", pos=(VX, VY, TOP + BASE[2] / 2), size=BASE, color=DARK),
    Box("fixed_jaw", pos=(VX + JAW_FACE + JAW[0] / 2, VY, BED + JAW[2] / 2), size=JAW, color=STEEL),
    Box("moving_jaw", pos=(VX - JAW_FACE - JAW[0] / 2, VY, BED + JAW[2] / 2), size=JAW, color=STEEL),
    Box("parts_tray", pos=(*TRAY_XY, TOP + TRAY[2] / 2), size=TRAY, color=WHITE),
]
OBJECTS = [*SOFT_JAWS, FINISHED_PART, *FIXTURES]
SOFT_NAMES = tuple(o.name for o in SOFT_JAWS)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _upright(st, size) -> bool:
    return float(st.high[2]) - float(st.low[2]) > size[2] - 0.004


def _fitted_at(st, x) -> bool:
    """A soft jaw standing on the vise bed against a steel jaw: centre within 4 mm of flat against it (and never inside
    it), within 8 mm of the vise centre along the jaw, upright, still."""
    return bool(
        abs(float(st.pos[0]) - x) < 0.004
        and abs(float(st.pos[1]) - VY) < 0.008
        and abs(float(st.low[2]) - BED) < 0.004
        and _upright(st, SOFT)
        and st.resting()
    )


def _present_at(st, x) -> bool:
    """A soft jaw in place against a steel jaw, perhaps still settling or briefly nudged: centre within 8 mm of fitted,
    bottom within 6 mm of the bed. The order rule asks only that neither jaw is missing when the part goes in."""
    return bool(
        abs(float(st.pos[0]) - x) < 0.008
        and abs(float(st.pos[1]) - VY) < 0.012
        and abs(float(st.low[2]) - BED) < 0.006
    )


def _present(objects) -> bool:
    sts = [objects[n] for n in SOFT_NAMES]
    return all(any(_present_at(st, x) for st in sts) for x in SOFT_X)


def _fitted(objects) -> tuple[bool, bool]:
    """Whether the far and the near steel jaw each has a soft jaw fitted (either plate may go on either jaw)."""
    sts = [objects[n] for n in SOFT_NAMES]
    return tuple(any(_fitted_at(st, x) for st in sts) for x in SOFT_X)


def _in_vise(st) -> bool:
    """The part between the steel jaws and below their tops: it has entered the vise."""
    return bool(
        abs(float(st.pos[0]) - VX) < JAW_FACE
        and abs(float(st.pos[1]) - VY) < BASE[1] / 2
        and float(st.low[2]) < JAW_TOP
    )


def _seated(st) -> bool:
    """The part standing at rest on the bed, centred between the jaws within 5 mm across them and 8 mm along them."""
    return bool(
        abs(float(st.pos[0]) - VX) < 0.005
        and abs(float(st.pos[1]) - VY) < 0.008
        and abs(float(st.low[2]) - BED) < 0.004
        and _upright(st, PART)
        and st.resting()
    )


# The trial's history: whether the part ever entered the vise while a soft jaw was missing. The task file only sees
# object states, so the history resets whenever the scene is back in its start state (each new trial).
_HISTORY = {"violated": False}


def _at_start(objects) -> bool:
    def near(name, xy):
        st = objects[name]
        return math.hypot(float(st.pos[0]) - xy[0], float(st.pos[1]) - xy[1]) < 0.005

    return near("finished_part", TRAY_XY) and all(near(n, xy) for n, xy in zip(SOFT_NAMES, SOFT_STARTS))


def _update(objects) -> bool:
    if _at_start(objects):
        _HISTORY["violated"] = False
    elif _in_vise(objects["finished_part"]) and not _present(objects):
        _HISTORY["violated"] = True
    return _HISTORY["violated"]


def success(objects) -> bool:
    """Both soft jaws fitted and the part seated between them, having entered only after both; holds for 1 s."""
    violated = _update(objects)
    return not violated and all(_fitted(objects)) and _seated(objects["finished_part"])


def score(objects) -> float:
    """0.25 for each steel jaw with a soft jaw fitted against it; 0.5 for the part seated between them, but only if it
    never entered the vise before both soft jaws were on."""
    violated = _update(objects)
    far, near = _fitted(objects)
    s = 0.25 * far + 0.25 * near
    if not violated and far and near and _seated(objects["finished_part"]):
        s += 0.5
    return s


def violated() -> bool:
    return _HISTORY["violated"]


def distance(objects) -> float:
    sts = [objects[n] for n in SOFT_NAMES]
    d = sum(min(math.hypot(float(st.pos[0]) - x, float(st.pos[1]) - VY) for st in sts) for x in SOFT_X)
    p = objects["finished_part"]
    return float(d + math.hypot(float(p.pos[0]) - VX, float(p.pos[1]) - VY))


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
TRAVEL = TOP + 0.150
SQUEEZE = 0.008
X_TRIM_SOFT = 0.002  # light plates land ~1.5-3 mm short in x
X_TRIM_PART = 0.003
# First sim run: the near plate trailed ~6 mm in x while being lowered, caught the top edge of the moving jaw and fell.
# It is now lowered 8 mm forward of its spot and slid back against the jaw 2 mm above the bed.
NEAR_LEAD = 0.008
# Third sim run: the part trailed ~6.5 mm in x during a fast lower that ended at soft-jaw-top height and clipped the
# near soft jaw. It now stops with its bottom 1 cm above the soft-jaw tops, settles, and goes down slowly.
PART_HOVER = SOFT[2] + 0.010


def _move(pick, place, *, size, depth, pick_bottom, place_bottom, label, lead=0.0, hover=0.0):
    """Fingers closing along y (across the plate's 7 cm length, pads beyond its ends; across the part's 4 cm). With
    ``lead``, the part is lowered that far forward of its spot and then slid back onto it just above the bed. With
    ``hover``, it stops that far above its set-down height, lets the arm's lag settle, then goes down slowly."""
    (px, py), (qx, qy) = pick, place
    lx = qx + lead
    width = size[1]
    gz = pick_bottom + size[2] - depth + TIP
    qz = place_bottom + size[2] - depth + TIP + 0.002
    open_ = max(-(width + 0.024) / 2, FINGER_OPEN)
    grip = -(width - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), open_, f"{label}: over"),
        Waypoint(1.5, (px, py, gz + 0.020), open_, f"{label}: descend"),
        Waypoint(1.0, (px, py, gz), open_, f"{label}: down slowly"),
        Waypoint(0.5, (px, py, gz), open_, f"{label}: settle"),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close"),
        Waypoint(1.5, (px, py, TRAVEL), grip, f"{label}: lift"),
        Waypoint(2.0, (lx, qy, TRAVEL), grip, f"{label}: carry"),
        Waypoint(0.6, (lx, qy, TRAVEL), grip, f"{label}: settle over"),
        *([Waypoint(1.5, (lx, qy, qz + hover), grip, f"{label}: lower to hover"),
           Waypoint(0.8, (lx, qy, qz + hover), grip, f"{label}: settle at hover"),
           Waypoint(2.5, (lx, qy, qz), grip, f"{label}: set down slowly between the jaws")] if hover else
          [Waypoint(1.5, (lx, qy, qz + 0.020), grip, f"{label}: lower"),
           Waypoint(1.5, (lx, qy, qz), grip, f"{label}: set down slowly")]),
        *([Waypoint(1.5, (qx, qy, qz), grip, f"{label}: slide back against the jaw")] if lead else []),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle"),
        Waypoint(1.5, (qx, qy, qz), open_, f"{label}: release"),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{label}: retreat"),
    ]


def demo():
    """Left arm only: the far soft jaw first, then the near one (each gripped by its ends, beyond the steel jaws; the near
    one lowered forward of its spot and slid back against its jaw), then
    the part, gripped across its 4 cm high up so the pads stay above the soft jaws and lowered slowly between them. 52.8 s."""
    far = _move(SOFT_STARTS[1], (SOFT_X[0] + X_TRIM_SOFT, VY), size=SOFT, depth=0.012, pick_bottom=TOP,
                place_bottom=BED, label="soft jaw to the fixed jaw")
    near = _move(SOFT_STARTS[0], (SOFT_X[1] + X_TRIM_SOFT, VY), size=SOFT, depth=0.012, pick_bottom=TOP,
                 place_bottom=BED, label="soft jaw to the moving jaw", lead=NEAR_LEAD)
    part = _move(TRAY_XY, (VX + X_TRIM_PART, VY), size=PART, depth=0.012, pick_bottom=TRAY_TOP, place_bottom=BED,
                 label="part", hover=PART_HOVER)
    return {"left": far + near + part}


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states, so the history is exercised too."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(size, x, y, bottom, vel=(0.0, 0.0, 0.0)):
        half = np.array(size) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    a, b = SOFT_NAMES
    start = {a: box(SOFT, *SOFT_STARTS[0], TOP), b: box(SOFT, *SOFT_STARTS[1], TOP),
             "finished_part": box(PART, *TRAY_XY, TRAY_TOP)}
    far_on = {b: box(SOFT, SOFT_X[0], VY, BED)}
    near_on = {a: box(SOFT, SOFT_X[1], VY, BED)}
    both = start | far_on | near_on
    entering = {"finished_part": box(PART, VX, VY, BED + 0.010, vel=(0, 0, -0.03))}
    seated = {"finished_part": box(PART, VX, VY, BED)}

    def run(*frames):
        for f in frames[:-1]:
            score(f)
        return score(frames[-1]), success(frames[-1])

    cases = {
        "start": ((start,), 0.0, False),
        "one soft jaw on": ((start, start | far_on), 0.25, False),
        "both soft jaws on": ((start, start | far_on, both), 0.5, False),
        "soft jaws, then the part": ((start, start | far_on, both, both | entering, both | seated), 1.0, True),
        "either plate on either jaw": ((start, start | {a: box(SOFT, SOFT_X[0], VY, BED), b: box(SOFT, SOFT_X[1], VY, BED)},
                                        start | {a: box(SOFT, SOFT_X[0], VY, BED), b: box(SOFT, SOFT_X[1], VY, BED)} | seated),
                                       1.0, True),
        "part in the bare vise first, then soft jaws": ((start, start | seated, start | far_on | seated,
                                                         both | seated), 0.5, False),
        "part in with one soft jaw on": ((start, start | far_on, start | far_on | entering, both | seated), 0.5, False),
        "both plates on the same jaw": ((start, start | {a: box(SOFT, SOFT_X[0], VY, BED),
                                                         b: box(SOFT, SOFT_X[0] - 0.003, VY + 0.006, BED)}), 0.25, False),
        "soft jaw 6 mm off the steel jaw": ((start, start | {b: box(SOFT, SOFT_X[0] - 0.006, VY, BED)}), 0.0, False),
        "soft jaw lying flat in the vise": ((start, start | {b: box((SOFT[0], SOFT[1], SOFT[0]), VX, VY, BED)}), 0.0, False),
        "part seated off centre": ((start, both, both | {"finished_part": box(PART, VX + 0.008, VY, BED)}), 0.5, False),
        "a soft jaw nudged (still in place) as the part goes in": ((start, both, both | entering | {a: ObjectState(
            np.array([SOFT_X[1] + 0.003, VY, BED + 0.012]), np.array([0.0, 0.148, 0.0, 0.989]), np.array([0.02, 0, 0]),
            np.zeros(3), np.array([SOFT_X[1] - 0.004, VY - 0.035, BED]), np.array([SOFT_X[1] + 0.012, VY + 0.035, BED + 0.022]))},
            both | seated), 1.0, True),
        "a new trial after a violation starts clean": ((start, start | seated, start, start | far_on, both,
                                                        both | seated), 1.0, True),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    score(start)
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
