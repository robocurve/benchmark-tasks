"""On the station table, in front of the left arm: an agarose gel set in its casting tray on the bench, the tray's
long side along y, with the well-forming comb still standing in the gel across the tray's left end (+y). Beyond it
stands a horizontal electrophoresis tank, also along y: a grey gel deck between two buffer chambers, with the black
electrode (the cathode, -) in the chamber at the right end (-y) and the red electrode (the anode, +) at the left end
(+y). A white comb rack stands on the bench to the left. DNA is negatively charged and runs toward the red anode, so
the wells must sit at the black cathode end: here the tray has to be turned 180 degrees on its way into the tank.
Lift the comb out and stand it in the rack, then seat the tray on the deck with its wells end (where the comb was)
at the black electrode. Scored from the poses of the tray and the comb: the wells end is the tray end that held the
comb at the start, and the tray's yaw tells which electrode it faces.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/gel-wells-at-cathode/gel_wells_at_cathode.py \\
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
TRAY = (0.070, 0.110, 0.015)  # casting tray with its gel: 7 cm wide (x), 11 cm long (y), 1.5 cm tall
TRAY_KG = 0.120
TRAY_START = (0.190, 0.270)
COMB = (0.060, 0.012, 0.025)  # across the tray, 12 mm thick, standing 25 mm proud of the gel
COMB_KG = 0.020
COMB_DY = 0.040  # the comb stands 4 cm from the tray's centre toward its +y end: the wells end
WELLS_LOCAL_Y = +1.0  # so the wells end is the tray's own +y end
COMB_START = (TRAY_START[0], TRAY_START[1] + COMB_DY)

DECK_XY = (0.380, 0.250)
DECK = (0.060, 0.100, 0.010)
DECK_TOP = TOP + DECK[2]
TANK_INNER = (0.136, 0.190)
TANK_WALL_T, TANK_WALL_H = 0.006, 0.030
ELECTRODE = (0.080, 0.006, 0.004)
ELECTRODE_DY = 0.075
CATHODE_DIR = -1.0  # the black electrode is at the tank's -y end
SEAT_XY, SEAT_DEG, WELLS_DEG = 0.008, 8.0, 20.0

RACK_XY = (0.150, 0.420)
RACK_INNER = (0.090, 0.070)
RACK_WALL_T, RACK_WALL_H = 0.006, 0.015

GEL = (0.72, 0.84, 0.92)
DARK = (0.12, 0.12, 0.13)
GREY = (0.55, 0.56, 0.58)
CLEAR = (0.80, 0.86, 0.90)
BLACK = (0.04, 0.04, 0.04)
RED = (0.85, 0.10, 0.08)
WHITE = (0.92, 0.92, 0.92)


def _frame(name, centre, inner, t, h, color):
    cx, cy = centre
    ix, iy = inner
    z = TOP + h / 2
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + t / 2, z), size=(ix + 2 * t, t, h), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - t / 2, z), size=(ix + 2 * t, t, h), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - t / 2, cy, z), size=(t, iy, h), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + t / 2, cy, z), size=(t, iy, h), color=color),
    ]


TRAY_OBJ = Box("gel_tray", pos=(*TRAY_START, TOP + TRAY[2] / 2), size=TRAY, mass=TRAY_KG, color=GEL)
COMB_OBJ = Box("comb", pos=(*COMB_START, TOP + TRAY[2] + COMB[2] / 2), size=COMB, mass=COMB_KG, color=DARK)
FIXTURES = [
    Box("gel_deck", pos=(*DECK_XY, TOP + DECK[2] / 2), size=DECK, color=GREY),
    Box("cathode_black", pos=(DECK_XY[0], DECK_XY[1] + CATHODE_DIR * ELECTRODE_DY, TOP + ELECTRODE[2] / 2), size=ELECTRODE,
        color=BLACK),
    Box("anode_red", pos=(DECK_XY[0], DECK_XY[1] - CATHODE_DIR * ELECTRODE_DY, TOP + ELECTRODE[2] / 2), size=ELECTRODE,
        color=RED),
    *_frame("tank", DECK_XY, TANK_INNER, TANK_WALL_T, TANK_WALL_H, CLEAR),
    *_frame("comb_rack", RACK_XY, RACK_INNER, RACK_WALL_T, RACK_WALL_H, WHITE),
]
OBJECTS = [TRAY_OBJ, COMB_OBJ, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _yaw_tilt(quat_xyzw):
    """(yaw about the vertical in radians, tilt of the up axis from the vertical in degrees)."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw, tilt


def seated(tray) -> bool:
    """Down flat on the gel deck, centred within 8 mm, square to the tank within 8 degrees (either way round), still."""
    yaw, tilt = _yaw_tilt(tray.quat_xyzw)
    return bool(
        abs(float(tray.pos[0]) - DECK_XY[0]) <= SEAT_XY
        and abs(float(tray.pos[1]) - DECK_XY[1]) <= SEAT_XY
        and abs(float(tray.low[2]) - DECK_TOP) <= 0.003
        and abs((math.degrees(yaw) + 90.0) % 180.0 - 90.0) <= SEAT_DEG
        and tilt <= 5.0
        and tray.resting()
    )


def wells_at_cathode(tray) -> bool:
    """The tray's wells end (its own +y end, where the comb stood) points at the black electrode within 20 degrees."""
    yaw, _ = _yaw_tilt(tray.quat_xyzw)
    wells = (-math.sin(yaw) * WELLS_LOCAL_Y, math.cos(yaw) * WELLS_LOCAL_Y)  # the tray's +y axis in the world
    return bool(wells[1] * CATHODE_DIR >= math.cos(math.radians(WELLS_DEG)))


def comb_racked(comb) -> bool:
    """The comb at rest inside the comb rack's walls, its bottom on the rack floor (table)."""
    return bool(
        abs(float(comb.pos[0]) - RACK_XY[0]) < RACK_INNER[0] / 2
        and abs(float(comb.pos[1]) - RACK_XY[1]) < RACK_INNER[1] / 2
        and float(comb.low[2]) < TOP + 0.010
        and comb.resting()
    )


def score(objects) -> float:
    """0.25 for the comb at rest in the comb rack; 0.75 for the tray seated on the deck with its wells end at the
    black cathode. A tray seated the wrong way round (wells at the red anode, the DNA would run off the gel) earns
    nothing."""
    tray = objects["gel_tray"]
    s = 0.25 * comb_racked(objects["comb"])
    if seated(tray) and wells_at_cathode(tray):
        s += 0.75
    return float(s)


def success(objects) -> bool:
    """The comb in its rack and the tray seated with the wells at the black electrode; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres to go: the tray to the deck, the comb to the rack, plus 0.1 per radian the wells end is off the cathode."""
    tray, comb = objects["gel_tray"], objects["comb"]
    yaw, _ = _yaw_tilt(tray.quat_xyzw)
    off = abs(((yaw - math.pi) + math.pi) % (2 * math.pi) - math.pi)  # the demo's goal: yaw 180 degrees
    d = math.hypot(float(tray.pos[0]) - DECK_XY[0], float(tray.pos[1]) - DECK_XY[1]) + 0.1 * off
    d += math.hypot(float(comb.pos[0]) - RACK_XY[0], float(comb.pos[1]) - RACK_XY[1])
    return float(d)


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
TRAVEL = TOP + 0.120
SQUEEZE = 0.008
PICK_YAW, PLACE_YAW = -math.pi / 2, math.pi / 2  # both close the fingers along x; the wrist turns 180 degrees between
X_TRIM = 0.002  # a carried 120 g tray lands about 2 mm short in x


def _comb_moves():
    (px, py), (qx, qy) = COMB_START, RACK_XY
    gz = TOP + TRAY[2] + 0.008 + TIP  # pad tips 8 mm above the gel
    qz = TOP + 0.003 + TIP  # standing in the rack, pad tips 3 mm above its floor
    open_ = -(COMB[1] + 0.020) / 2
    grip = -(COMB[1] - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), open_, "comb: over"),
        Waypoint(1.5, (px, py, gz + 0.015), open_, "comb: descend"),
        Waypoint(0.8, (px, py, gz), open_, "comb: down slowly"),
        Waypoint(0.5, (px, py, gz), open_, "comb: settle"),
        Waypoint(1.0, (px, py, gz), grip, "comb: close"),
        Waypoint(1.5, (px, py, gz + 0.040), grip, "comb: draw straight up out of the gel"),
        Waypoint(1.0, (px, py, TRAVEL), grip, "comb: lift"),
        Waypoint(1.8, (qx, qy, TRAVEL), grip, "comb: to the rack"),
        Waypoint(1.5, (qx, qy, qz + 0.015), grip, "comb: lower"),
        Waypoint(0.8, (qx, qy, qz + 0.002), grip, "comb: set down"),
        Waypoint(0.5, (qx, qy, qz + 0.002), grip, "comb: settle"),
        Waypoint(1.0, (qx, qy, qz + 0.002), open_, "comb: release"),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, "comb: retreat"),
    ]


def _tray_moves():
    (px, py) = TRAY_START
    qx, qy = DECK_XY[0] + X_TRIM, DECK_XY[1]
    gz = TOP + 0.004 + TIP  # pad tips 4 mm above the bench
    qz = DECK_TOP + 0.004 + TIP + 0.002
    grip = -(TRAY[0] - SQUEEZE) / 2
    mid = ((px + qx) / 2, (py + qy) / 2)
    return [
        Waypoint(1.8, (px, py, TRAVEL), FINGER_OPEN, "tray: over its middle", PICK_YAW),
        Waypoint(1.5, (px, py, gz + 0.020), FINGER_OPEN, "tray: descend", PICK_YAW),
        Waypoint(1.0, (px, py, gz), FINGER_OPEN, "tray: down slowly", PICK_YAW),
        Waypoint(0.5, (px, py, gz), FINGER_OPEN, "tray: settle", PICK_YAW),
        Waypoint(1.5, (px, py, gz), grip, "tray: close across its width", PICK_YAW),
        Waypoint(1.5, (px, py, TRAVEL), grip, "tray: lift", PICK_YAW),
        Waypoint(2.5, (*mid, TRAVEL), grip, "tray: carry, turning (first 90 degrees)", 0.0),
        Waypoint(2.5, (qx, qy, TRAVEL), grip, "tray: carry, turning (second 90 degrees)", PLACE_YAW),
        Waypoint(0.8, (qx, qy, TRAVEL), grip, "tray: settle over the deck", PLACE_YAW),
        Waypoint(2.0, (qx, qy, qz + 0.020), grip, "tray: lower into the tank", PLACE_YAW),
        Waypoint(1.2, (qx, qy, qz), grip, "tray: set on the deck slowly", PLACE_YAW),
        Waypoint(0.6, (qx, qy, qz), grip, "tray: settle", PLACE_YAW),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, "tray: release", PLACE_YAW),
        Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, "tray: retreat", PLACE_YAW),
    ]


def demo():
    """Left arm only: the comb drawn straight up out of the gel and stood in the rack, then the tray gripped across
    its width at its middle, turned 180 degrees about the vertical while it is carried (the wrist goes from -90 to
    +90 degrees) and set on the gel deck with the wells end at the black electrode. 34.6 s."""
    return {"left": _comb_moves() + _tray_moves()}


def self_test(ObjectState=None):
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(size, x, y, bottom, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        a, t = math.radians(yaw_deg), math.radians(tilt_deg)
        cz, sz, cx, sx = math.cos(a / 2), math.sin(a / 2), math.cos(t / 2), math.sin(t / 2)
        q = np.array([cz * sx, sz * sx, sz * cx, cz * cx])
        hx = abs(math.cos(a)) * size[0] / 2 + abs(math.sin(a)) * size[1] / 2
        hy = abs(math.sin(a)) * size[0] / 2 + abs(math.cos(a)) * size[1] / 2
        half = np.array([hx, hy, size[2] / 2])
        pos = np.array([x, y, bottom + size[2] / 2])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {"gel_tray": box(TRAY, *TRAY_START, TOP), "comb": box(COMB, *COMB_START, TOP + TRAY[2])}
    racked = {"comb": box(COMB, *RACK_XY, TOP)}
    right_way = {"gel_tray": box(TRAY, *DECK_XY, DECK_TOP, yaw_deg=180)}
    cases = {
        "start": (start, 0.0, False),
        "comb racked only": (start | racked, 0.25, False),
        "done": (right_way | racked, 1.0, True),
        "done, turned -180 (the other way round)": ({"gel_tray": box(TRAY, *DECK_XY, DECK_TOP, yaw_deg=-180)} | racked, 1.0, True),
        "done, 5 mm off and 6 degrees skew": ({"gel_tray": box(TRAY, DECK_XY[0] - 0.005, DECK_XY[1] + 0.003, DECK_TOP, 174)} | racked,
                                              1.0, True),
        "not turned: wells at the red anode": ({"gel_tray": box(TRAY, *DECK_XY, DECK_TOP, yaw_deg=0)} | racked, 0.25, False),
        "turned only 90 degrees (wells at the side)": ({"gel_tray": box(TRAY, *DECK_XY, DECK_TOP, yaw_deg=90)} | racked, 0.25, False),
        "right way but the comb left in the gel": (right_way | {"comb": box(COMB, DECK_XY[0], DECK_XY[1] - COMB_DY, DECK_TOP + TRAY[2])},
                                                   0.75, False),
        "right way, comb dropped on the bench": (right_way | {"comb": box(COMB, 0.25, 0.42, TOP, tilt_deg=90)}, 0.75, False),
        "tray 1 cm off the deck centre": ({"gel_tray": box(TRAY, DECK_XY[0] - 0.010, DECK_XY[1], DECK_TOP, 180)} | racked, 0.25, False),
        "tray propped on a wall": ({"gel_tray": box(TRAY, *DECK_XY, DECK_TOP + 0.006, 180, tilt_deg=8)} | racked, 0.25, False),
        "tray still settling": ({"gel_tray": box(TRAY, *DECK_XY, DECK_TOP, 180, vel=(0, 0, -0.03))} | racked, 0.25, False),
        "comb still falling into the rack": (right_way | {"comb": box(COMB, *RACK_XY, TOP, vel=(0, 0, -0.05))}, 0.75, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    return len(cases)


if __name__ == "__main__":
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
