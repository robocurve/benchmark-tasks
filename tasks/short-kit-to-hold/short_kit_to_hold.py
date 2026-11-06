"""On the station table, in front of the left arm, the release point of a kitting cell. Two kit plates wait on a
black audit mat nearer the arms. Each kit is four steel bushings standing on a blue plastic kit plate; the plate on
the outer side holds only three (the far right-hand bushing is missing), the inner one holds all four. Further out
lie two lanes: the grey ship lane (outer side), which feeds the assembly line, and the red hold lane (inner side),
where kits wait for a material review. Between the audit mat and the arms' midline, a small line-side bin holds three
loose bushings of the same kind, which belong to the line. Release rule: a complete kit is released to the ship
lane; a short kit goes to the hold lane as it is and is never topped up from line stock. Scored from the poses at
the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/short-kit-to-hold/short_kit_to_hold.py \\
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
PLATE = (0.110, 0.070, 0.020)  # kit plate: 110 mm along x, 70 mm across (y), 20 mm thick
PLATE_KG = 0.120
BUSH = (0.022, 0.022, 0.018)  # a bushing, modelled as a short square block
BUSH_KG = 0.025
KIT_SPOTS = [(-0.027, 0.017), (0.027, 0.017), (-0.027, -0.017), (0.027, -0.017)]  # on a plate, from its centre
MISSING = 3  # the short kit lacks the far right-hand bushing (spot 3)
MAT_H = 0.002
MAT_TOP = TOP + MAT_H
AUDIT = (0.190, 0.300)
AUDIT_MAT = (0.150, 0.300)
KIT_START = {"short_kit": (0.190, 0.375), "full_kit": (0.190, 0.225)}
SHIP = (0.390, 0.375)
HOLD = (0.390, 0.200)
LANE_MAT = (0.150, 0.120)

BIN = (0.230, 0.110)  # line-side bin of loose bushings
BIN_INNER = (0.120, 0.050)
WALL_T, WALL_H = 0.006, 0.020
SPARE_X = (0.195, 0.230, 0.265)
SPARE_MOVE = 0.010  # a line-side bushing counts as taken once it is 1 cm from its start

STEEL = (0.62, 0.63, 0.66)
BLUE = (0.15, 0.30, 0.65)
BLACK = (0.08, 0.08, 0.08)
GREY = (0.55, 0.56, 0.58)
RED = (0.80, 0.12, 0.10)
YELLOW = (0.90, 0.75, 0.10)


def _walls(name, centre, inner, color):
    cx, cy = centre
    ix, iy = inner
    z = TOP + WALL_H / 2
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + WALL_T / 2, z), size=(ix + 2 * WALL_T, WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - WALL_T / 2, z), size=(ix + 2 * WALL_T, WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - WALL_T / 2, cy, z), size=(WALL_T, iy, WALL_H), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + WALL_T / 2, cy, z), size=(WALL_T, iy, WALL_H), color=color),
    ]


PLATES = [Box(k, pos=(*xy, MAT_TOP + PLATE[2] / 2), size=PLATE, mass=PLATE_KG, color=BLUE) for k, xy in KIT_START.items()]
KIT_PARTS = {k: [j for j in range(4) if not (k == "short_kit" and j == MISSING)] for k in KIT_START}
BUSHINGS = [
    Box(f"{k}_bush_{j}", pos=(KIT_START[k][0] + KIT_SPOTS[j][0], KIT_START[k][1] + KIT_SPOTS[j][1],
                              MAT_TOP + PLATE[2] + BUSH[2] / 2), size=BUSH, mass=BUSH_KG, color=STEEL)
    for k in KIT_START for j in KIT_PARTS[k]
]
SPARES = [Box(f"line_bush_{j}", pos=(x, BIN[1], TOP + BUSH[2] / 2), size=BUSH, mass=BUSH_KG, color=STEEL)
          for j, x in enumerate(SPARE_X)]
FIXTURES = [
    Box("audit_mat", pos=(*AUDIT, TOP + MAT_H / 2), size=(*AUDIT_MAT, MAT_H), color=BLACK),
    Box("ship_lane", pos=(*SHIP, TOP + MAT_H / 2), size=(*LANE_MAT, MAT_H), color=GREY),
    Box("hold_lane", pos=(*HOLD, TOP + MAT_H / 2), size=(*LANE_MAT, MAT_H), color=RED),
    *_walls("line_bin", BIN, BIN_INNER, YELLOW),
]
OBJECTS = [*PLATES, *BUSHINGS, *SPARES, *FIXTURES]
ALL_BUSH = [b.name for b in (*BUSHINGS, *SPARES)]
OWN = {k: {f"{k}_bush_{j}" for j in KIT_PARTS[k]} for k in KIT_START}

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _yaw_tilt(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw, tilt


def _on_lane(st, centre) -> bool:
    """The plate flat on the lane mat (bottom within 4 mm of it, tilt under 5 degrees, still) with its whole
    footprint on the mat."""
    yaw, tilt = _yaw_tilt(st.quat_xyzw)
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    ex, ey = c * PLATE[0] / 2 + s * PLATE[1] / 2, s * PLATE[0] / 2 + c * PLATE[1] / 2
    return bool(
        abs(float(st.pos[0]) - centre[0]) + ex <= LANE_MAT[0] / 2
        and abs(float(st.pos[1]) - centre[1]) + ey <= LANE_MAT[1] / 2
        and abs(float(st.low[2]) - MAT_TOP) <= 0.004
        and tilt <= 5.0
        and st.resting()
    )


def on_plate(objects, plate) -> set:
    """The bushings standing on a plate: centre over the plate's top face (in the plate's own frame), bottom within
    5 mm of that face."""
    p = objects[plate]
    yaw, _ = _yaw_tilt(p.quat_xyzw)
    top = float(p.pos[2]) + PLATE[2] / 2
    out = set()
    for name in ALL_BUSH:
        st = objects[name]
        dx, dy = float(st.pos[0]) - float(p.pos[0]), float(st.pos[1]) - float(p.pos[1])
        u, v = dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)
        if abs(u) < PLATE[0] / 2 and abs(v) < PLATE[1] / 2 and abs(float(st.low[2]) - top) < 0.005:
            out.add(name)
    return out


def _routed(objects, plate, lane) -> bool:
    """The kit on the lane with exactly its own bushings on it: none lost, none added."""
    return bool(_on_lane(objects[plate], lane) and on_plate(objects, plate) == OWN[plate])


def _line_stock_taken(objects) -> bool:
    return any(math.dist([float(v) for v in objects[sp.name].pos], sp.pos) > SPARE_MOVE for sp in SPARES)


def score(objects) -> float:
    """0.5 for the full kit on the ship lane with its four bushings, 0.5 for the short kit on the hold lane with its
    own three bushings and nothing added. A short kit on the ship lane scores 0; a line-side bushing taken from its
    bin caps the score at 0.25."""
    if _on_lane(objects["short_kit"], SHIP):
        return 0.0
    s = 0.5 * _routed(objects, "full_kit", SHIP) + 0.5 * _routed(objects, "short_kit", HOLD)
    if _line_stock_taken(objects):
        s = min(s, 0.25)
    return float(s)


def success(objects) -> bool:
    """Full kit shipped, short kit on hold untouched, line stock untouched; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres to go: each kit plate's centre to its lane's centre (in the plane)."""
    d = 0.0
    for plate, lane in (("full_kit", SHIP), ("short_kit", HOLD)):
        st = objects[plate]
        d += math.hypot(float(st.pos[0]) - lane[0], float(st.pos[1]) - lane[1])
    return float(d)


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
TRAVEL = TOP + 0.130
SQUEEZE = 0.008
DEPTH = 0.016  # pad tips 4 mm above the plate's bottom
HOVER = 0.002  # let go 2 mm above the mat


def _grip_z(bottom):
    return bottom + PLATE[2] - DEPTH + TIP


def _carry(pick, place, label):
    """The kit plate gripped across its 70 mm width at its centre, lifted and carried slowly so the loose bushings
    ride on it, set down on the lane mat. 19.1 s."""
    (px, py), (qx, qy) = pick, place
    gz, qz = _grip_z(MAT_TOP), _grip_z(MAT_TOP) + HOVER
    grip = -(PLATE[1] - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), FINGER_OPEN, f"{label}: over"),
        Waypoint(1.5, (px, py, gz + 0.020), FINGER_OPEN, f"{label}: descend"),
        Waypoint(1.0, (px, py, gz), FINGER_OPEN, f"{label}: down slowly"),
        Waypoint(0.5, (px, py, gz), FINGER_OPEN, f"{label}: settle"),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close"),
        Waypoint(1.5, (px, py, gz + 0.030), grip, f"{label}: lift slowly"),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{label}: lift"),
        Waypoint(3.0, (qx, qy, TRAVEL), grip, f"{label}: carry slowly"),
        Waypoint(0.8, (qx, qy, TRAVEL), grip, f"{label}: settle over the lane"),
        Waypoint(1.5, (qx, qy, qz + 0.030), grip, f"{label}: lower"),
        Waypoint(1.5, (qx, qy, qz), grip, f"{label}: set down slowly"),
        Waypoint(0.8, (qx, qy, qz), grip, f"{label}: settle"),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, f"{label}: release"),
        Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, f"{label}: retreat"),
    ]


def demo():
    """Left arm only: the full kit to the ship lane, then the short kit, as it is, to the hold lane. 38.2 s."""
    return {"left": _carry(KIT_START["full_kit"], SHIP, "full kit") + _carry(KIT_START["short_kit"], HOLD, "short kit")}


def self_test(ObjectState=None):
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(size, x, y, bottom, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        t = math.radians(tilt_deg)
        q = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
        half = np.array(size) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def kit(plate, x, y, bottom=MAT_TOP, parts=None, **kw):
        parts = KIT_PARTS[plate] if parts is None else parts
        st = {plate: box(PLATE, x, y, bottom, **kw)}
        for j in parts:
            st[f"{plate}_bush_{j}"] = box(BUSH, x + KIT_SPOTS[j][0], y + KIT_SPOTS[j][1], bottom + PLATE[2])
        return st

    start = kit("short_kit", *KIT_START["short_kit"]) | kit("full_kit", *KIT_START["full_kit"])
    start |= {sp.name: box(BUSH, sp.pos[0], sp.pos[1], TOP) for sp in SPARES}
    shipped = kit("full_kit", SHIP[0] - 0.005, SHIP[1])
    held = kit("short_kit", HOLD[0] - 0.005, HOLD[1] + 0.003)
    topped = {"line_bush_0": box(BUSH, HOLD[0] + 0.022, HOLD[1] - 0.017, MAT_TOP + PLATE[2])}
    robbed = {"full_kit_bush_3": box(BUSH, HOLD[0] + 0.022, HOLD[1] - 0.017, MAT_TOP + PLATE[2])}

    cases = {
        "start": (start, 0.0, False),
        "full kit shipped": (start | shipped, 0.5, False),
        "short kit held": (start | held, 0.5, False),
        "done": (start | shipped | held, 1.0, True),
        "kits swapped": (start | kit("short_kit", *SHIP) | kit("full_kit", *HOLD), 0.0, False),
        "short kit topped up from the line and shipped": (start | shipped | kit("short_kit", *SHIP) | topped | {"line_bush_0": box(BUSH, SHIP[0] + 0.027, SHIP[1] - 0.017, MAT_TOP + PLATE[2])}, 0.0, False),
        "short kit topped up from the line, then held": (start | shipped | held | topped, 0.25, False),
        "short kit completed by robbing the full kit": (start | kit("full_kit", *SHIP, parts=[0, 1, 2]) | held | robbed, 0.0, False),
        "done but a line bushing knocked out of its bin": (start | shipped | held | {"line_bush_2": box(BUSH, 0.30, 0.15, TOP)}, 0.25, False),
        "full kit shipped but a bushing fell off": (start | kit("full_kit", *SHIP, parts=[0, 1, 2]) | {"full_kit_bush_3": box(BUSH, 0.45, 0.30, TOP)} | held, 0.5, False),
        "full kit hanging over the lane edge": (start | kit("full_kit", SHIP[0] + 0.040, SHIP[1]) | held, 0.5, False),
        "short kit still sliding": (start | shipped | kit("short_kit", *HOLD, vel=(0.05, 0, 0)), 0.5, False),
        "full kit tipped on the mat": (start | kit("full_kit", *SHIP, tilt_deg=12.0) | held, 0.5, False),
        "short kit left on the audit mat": (start | shipped, 0.5, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
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
