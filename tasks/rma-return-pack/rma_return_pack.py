"""On the station table, in front of the left arm, stands an open brown return box (a fixed open frame on the table).
Nearer the arms lie a failed green network card and two grey anti-static foam pads, all flat on the table: one pad to
the left of the card, one to the right. The card goes back to the vendor under a return (RMA) authorisation, and the
return instructions require it to travel cushioned on both faces: one foam pad flat on the box floor, the card flat on
that pad, the other foam pad flat on top of the card. The layers are checked from the stack itself: the card must lie
on a pad, not on the box floor, and a pad must cover it.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/rma-return-pack/rma_return_pack.py \\
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
BOX_XY = (0.330, 0.270)
BOX_INNER = (0.130, 0.130)
BOX_WALL = (0.006, 0.030)

FOAM = (0.110, 0.070, 0.015)
FOAM_KG = 0.015
CARD = (0.100, 0.060, 0.008)
CARD_KG = 0.120
FOAM_STARTS = ((0.170, 0.420), (0.170, 0.180))
CARD_START = (0.170, 0.300)

# Heights of the packed layers.
FLOOR = TOP
CARD_ON = FLOOR + FOAM[2]
COVER_ON = CARD_ON + CARD[2]

CARDBOARD = (0.60, 0.45, 0.28)
FOAM_GREY = (0.35, 0.36, 0.40)
GREEN = (0.10, 0.50, 0.22)


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


FOAMS = [Box(f"foam_pad_{k + 1}", pos=(x, y, TOP + FOAM[2] / 2), size=FOAM, mass=FOAM_KG, color=FOAM_GREY)
         for k, (x, y) in enumerate(FOAM_STARTS)]
NETWORK_CARD = Box("network_card", pos=(*CARD_START, TOP + CARD[2] / 2), size=CARD, mass=CARD_KG, color=GREEN)
OBJECTS = [*FOAMS, NETWORK_CARD, *_frame("return_box", BOX_XY, BOX_INNER, BOX_WALL, TOP, CARDBOARD)]
FOAM_NAMES = tuple(o.name for o in FOAMS)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _flat(st, size) -> bool:
    return float(st.high[2]) - float(st.low[2]) < size[2] + 0.006


def _layer(st, size, bottom, centre_xy, tol) -> bool:
    """Lying flat and still with its bottom within 4 mm of ``bottom`` and its centre within ``tol`` of ``centre_xy``."""
    return bool(
        abs(float(st.pos[0]) - centre_xy[0]) < tol
        and abs(float(st.pos[1]) - centre_xy[1]) < tol
        and abs(float(st.low[2]) - bottom) < 0.004
        and _flat(st, size)
        and st.resting()
    )


def _inside(st) -> bool:
    return bool(abs(float(st.pos[0]) - BOX_XY[0]) < BOX_INNER[0] / 2 and abs(float(st.pos[1]) - BOX_XY[1]) < BOX_INNER[1] / 2)


def _layers(objects) -> tuple[bool, bool, bool]:
    """(a pad on the box floor, the card flat on that pad, the other pad flat on the card)."""
    card = objects["network_card"]
    best = (False, False, False)
    for base_name in FOAM_NAMES:
        base = objects[base_name]
        cover = objects[next(n for n in FOAM_NAMES if n != base_name)]
        b = _inside(base) and _layer(base, FOAM, FLOOR, BOX_XY, 0.030)
        c = b and _inside(card) and _layer(card, CARD, CARD_ON, (float(base.pos[0]), float(base.pos[1])), 0.012)
        t = c and _inside(cover) and _layer(cover, FOAM, COVER_ON, (float(card.pos[0]), float(card.pos[1])), 0.012)
        best = max(best, (b, c, t), key=sum)
    return best


def success(objects) -> bool:
    """Pad, card, pad stacked flat in the box; holds for 1 s."""
    return all(_layers(objects))


def score(objects) -> float:
    """0.25 for a foam pad flat on the box floor; 0.25 more for the card flat on that pad, centred over it within 12 mm;
    0.5 more for the other pad flat on the card, centred over it within 12 mm. Each layer counts only on the one below."""
    b, c, t = _layers(objects)
    return 0.25 * b + 0.25 * c + 0.5 * t


def distance(objects) -> float:
    card = objects["network_card"]
    d = math.hypot(float(card.pos[0]) - BOX_XY[0], float(card.pos[1]) - BOX_XY[1])
    for n in FOAM_NAMES:
        st = objects[n]
        d += math.hypot(float(st.pos[0]) - BOX_XY[0], float(st.pos[1]) - BOX_XY[1])
    return float(d)


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
TRAVEL = TOP + 0.150
SQUEEZE = 0.008
X_TRIM_LIGHT = 0.0015  # light parts land ~1.5 mm short in x
X_TRIM_CARD = 0.003


def _move(pick, place, *, size, depth, pick_bottom, place_bottom, label):
    """Fingers closing along y, across the part's width."""
    (px, py), (qx, qy) = pick, place
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
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{label}: carry"),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{label}: settle over"),
        Waypoint(1.5, (qx, qy, qz + 0.020), grip, f"{label}: lower"),
        Waypoint(1.5, (qx, qy, qz), grip, f"{label}: set down slowly"),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle"),
        Waypoint(1.5, (qx, qy, qz), open_, f"{label}: release"),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{label}: retreat"),
    ]


def demo():
    """Left arm only: the left pad onto the box floor, the card onto it, the right pad on top, each gripped across its
    width with the pad tips a few mm above whatever lies below. 49.5 s."""
    qx, qy = BOX_XY
    base = _move(FOAM_STARTS[0], (qx + X_TRIM_LIGHT, qy), size=FOAM, depth=0.010, pick_bottom=TOP, place_bottom=FLOOR,
                 label="foam base")
    card = _move(CARD_START, (qx + X_TRIM_CARD, qy), size=CARD, depth=0.005, pick_bottom=TOP, place_bottom=CARD_ON,
                 label="card")
    cover = _move(FOAM_STARTS[1], (qx + X_TRIM_LIGHT, qy), size=FOAM, depth=0.010, pick_bottom=TOP,
                  place_bottom=COVER_ON, label="foam cover")
    return {"left": base + card + cover}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(size, x, y, bottom, vel=(0.0, 0.0, 0.0)):
        half = np.array(size) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    a, b = FOAM_NAMES
    bx, by = BOX_XY
    start = {a: box(FOAM, *FOAM_STARTS[0], TOP), b: box(FOAM, *FOAM_STARTS[1], TOP),
             "network_card": box(CARD, *CARD_START, TOP)}
    base = {a: box(FOAM, bx, by, FLOOR)}
    card = {"network_card": box(CARD, bx + 0.003, by, CARD_ON)}
    cover = {b: box(FOAM, bx, by - 0.004, COVER_ON)}
    cases = {
        "start": (start, 0.0, False),
        "a pad on the floor": (start | base, 0.25, False),
        "pad, card": (start | base | card, 0.5, False),
        "pad, card, pad": (start | base | card | cover, 1.0, True),
        "either pad as the base": (start | {b: box(FOAM, bx, by, FLOOR), a: box(FOAM, bx, by, COVER_ON)} | card, 1.0, True),
        "card straight on the box floor": (start | {"network_card": box(CARD, bx, by, FLOOR)}, 0.0, False),
        "card on the floor, a pad on top of it": (start | {"network_card": box(CARD, bx, by, FLOOR),
                                                           a: box(FOAM, bx, by, FLOOR + CARD[2])}, 0.0, False),
        "both pads under the card": (start | base | {b: box(FOAM, bx, by, CARD_ON),
                                                      "network_card": box(CARD, bx, by, CARD_ON + FOAM[2])}, 0.25, False),
        "card hanging off the pad": (start | base | {"network_card": box(CARD, bx + 0.020, by, CARD_ON)}, 0.25, False),
        "cover pad still settling": (start | base | card | {b: box(FOAM, bx, by, COVER_ON, vel=(0, 0, -0.05))}, 0.5, False),
        "stack built outside the box": (start | {a: box(FOAM, 0.17, 0.30, TOP), "network_card": box(CARD, 0.17, 0.30, CARD_ON),
                                                 b: box(FOAM, 0.17, 0.30, COVER_ON)}, 0.0, False),
        "card on its edge on the pad": (start | base | {"network_card": box((CARD[0], CARD[2], CARD[1]), bx, by, CARD_ON)},
                                        0.25, False),
    }
    for name, (objs, want, want_success) in cases.items():
        got, got_success = score(objs), success(objs)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
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
