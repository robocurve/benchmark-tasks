"""On the station table, in front of the left arm, one station of a pharmaceutical packaging line at changeover. The
line bed is a fixed stainless plate with a white set-out tile at the labeller position. Left on the bed from the
batch just finished: a tall shipper carton, two glass vials and a bundle of leaflets. One of the vials stands
right behind the carton, where the top camera cannot see it. To the left of the line (outer side) is the
reconciliation bin; nearer the arms, on a staging mat, is the new batch's label reel. Line clearance rule: before
any material of the new batch comes onto the line, everything left from the previous batch goes into the
reconciliation bin to be counted; only then is the new label reel set out on the tile. The order is checked every
step. Scored from the object poses.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/gmp-line-clearance/gmp_line_clearance.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
BED = (0.360, 0.290)  # the line bed's centre
BED_SIZE = (0.200, 0.220, 0.002)
BED_TOP = TOP + BED_SIZE[2]
TILE = (0.300, 0.355)  # the labeller's set-out tile
TILE_SIZE = (0.060, 0.060, 0.0025)
TILE_TOP = TOP + TILE_SIZE[2]

CARTON = (0.050, 0.070, 0.120)  # open shipper carton: 5 cm deep (x), 7 cm wide, 12 cm tall
CARTON_KG = 0.090
VIAL_R, VIAL_H, VIAL_KG = 0.011, 0.040, 0.020
LEAFLETS = (0.045, 0.030, 0.016)
LEAFLETS_KG = 0.015
REEL_R, REEL_H, REEL_KG = 0.025, 0.030, 0.060
START = {
    "carton": (0.360, 0.300),
    "vial_hidden": (0.402, 0.305),  # 6 mm behind the carton's far face: hidden from the top camera
    "vial_front": (0.300, 0.225),
    "leaflets": (0.420, 0.215),
    "label_reel": (0.170, 0.220),
}
LEFTOVERS = ("carton", "vial_hidden", "vial_front", "leaflets")

BIN = (0.175, 0.440)  # reconciliation bin centre
BIN_INNER = (0.150, 0.140)
WALL_T, WALL_H = 0.006, 0.030
BIN_SPOT = {"carton": (0.135, 0.440), "vial_front": (0.200, 0.390), "leaflets": (0.200, 0.435),
            "vial_hidden": (0.200, 0.480)}
STAGE = (0.170, 0.220)
STAGE_SIZE = (0.100, 0.100, 0.002)
STAGE_TOP = TOP + STAGE_SIZE[2]

STEEL = (0.80, 0.81, 0.83)
WHITE = (0.95, 0.95, 0.95)
KRAFT = (0.62, 0.47, 0.30)
GLASS = (0.70, 0.82, 0.88)
PAPER = (0.92, 0.90, 0.80)
REEL = (0.20, 0.45, 0.75)
BLUE = (0.10, 0.20, 0.45)
DARK = (0.25, 0.26, 0.28)


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


FREE = [
    Box("carton", pos=(*START["carton"], BED_TOP + CARTON[2] / 2), size=CARTON, mass=CARTON_KG, color=KRAFT),
    Cylinder("vial_hidden", pos=(*START["vial_hidden"], BED_TOP + VIAL_H / 2), radius=VIAL_R, height=VIAL_H,
             mass=VIAL_KG, color=GLASS),
    Cylinder("vial_front", pos=(*START["vial_front"], BED_TOP + VIAL_H / 2), radius=VIAL_R, height=VIAL_H,
             mass=VIAL_KG, color=GLASS),
    Box("leaflets", pos=(*START["leaflets"], BED_TOP + LEAFLETS[2] / 2), size=LEAFLETS, mass=LEAFLETS_KG, color=PAPER),
    Cylinder("label_reel", pos=(*START["label_reel"], STAGE_TOP + REEL_H / 2), radius=REEL_R, height=REEL_H,
             mass=REEL_KG, color=REEL),
]
FIXTURES = [
    Box("line_bed", pos=(*BED, TOP + BED_SIZE[2] / 2), size=BED_SIZE, color=STEEL),
    Box("setout_tile", pos=(*TILE, TOP + TILE_SIZE[2] / 2), size=TILE_SIZE, color=WHITE),
    Box("staging_mat", pos=(*STAGE, TOP + STAGE_SIZE[2] / 2), size=STAGE_SIZE, color=DARK),
    *_walls("reconciliation_bin", BIN, BIN_INNER, BLUE),
]
OBJECTS = [*FREE, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _tilt(quat_xyzw) -> float:
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _on_line(st) -> bool:
    """Centre over the line bed and lowest point less than 3 cm above it (standing, lying or being slid on it)."""
    return bool(
        abs(float(st.pos[0]) - BED[0]) < BED_SIZE[0] / 2
        and abs(float(st.pos[1]) - BED[1]) < BED_SIZE[1] / 2
        and float(st.low[2]) < BED_TOP + 0.030
    )


def _in_bin(st) -> bool:
    """At rest, the centre inside the bin's inner walls, the lowest point within 3 cm of the table."""
    return bool(
        abs(float(st.pos[0]) - BIN[0]) < BIN_INNER[0] / 2
        and abs(float(st.pos[1]) - BIN[1]) < BIN_INNER[1] / 2
        and TOP - 0.005 < float(st.low[2]) < TOP + 0.030
        and st.resting()
    )


def _reel_set_out(st) -> bool:
    """Standing on the set-out tile: centre within 2 cm of the tile centre, bottom within 3 mm of the tile, upright
    within 8 degrees, still."""
    return bool(
        math.hypot(float(st.pos[0]) - TILE[0], float(st.pos[1]) - TILE[1]) <= 0.020
        and abs(float(st.low[2]) - TILE_TOP) <= 0.003
        and _tilt(st.quat_xyzw) <= 8.0
        and st.resting()
    )


# The trial's history: whether the new label reel was ever on the line while a leftover still was. The task file only
# sees object states, so the history resets whenever the scene is back in its start state (each new trial).
_HISTORY = {"violated": False}


def _at_start(objects) -> bool:
    return all(math.dist([float(v) for v in objects[o.name].pos], o.pos) < 0.005 for o in FREE)


def _update(objects) -> bool:
    if _at_start(objects):
        _HISTORY["violated"] = False
    elif _on_line(objects["label_reel"]) and any(_on_line(objects[n]) for n in LEFTOVERS):
        _HISTORY["violated"] = True
    return _HISTORY["violated"]


def score(objects) -> float:
    """0.2 for each previous-batch item at rest in the reconciliation bin, and 0.2 for the new label reel set out on
    the tile once all four are. If the reel was ever on the line while a leftover still was, the score is capped at
    0.4."""
    violated = _update(objects)
    n = sum(_in_bin(objects[name]) for name in LEFTOVERS)
    s = 0.2 * n + 0.2 * (n == len(LEFTOVERS) and _reel_set_out(objects["label_reel"]))
    if violated:
        s = min(s, 0.4)
    return float(round(s, 9))


def success(objects) -> bool:
    """All four leftovers in the bin, the new reel on its tile, and the line cleared before the reel came on; holds
    for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def violated() -> bool:
    """Whether this trial has had the new reel on the line before clearance finished (for the log)."""
    return _HISTORY["violated"]


def distance(objects) -> float:
    """Metres to go: each leftover to the bin's centre plus the reel to the tile (in the plane)."""
    d = 0.0
    for name in LEFTOVERS:
        st = objects[name]
        d += math.hypot(float(st.pos[0]) - BIN[0], float(st.pos[1]) - BIN[1])
    st = objects["label_reel"]
    return float(d + math.hypot(float(st.pos[0]) - TILE[0], float(st.pos[1]) - TILE[1]))


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
SQUEEZE = 0.008
CLEAR = 0.020
TRAVEL = TOP + 0.110
TRAVEL_HIGH = TOP + 0.190  # with the 12 cm carton in hand
ACROSS_X = math.pi / 2  # yaw for a grip that closes along x
# name: (width along the closing axis, height, grip depth below the top, yaw)
GRIP = {
    "carton": (CARTON[1], CARTON[2], 0.030, 0.0),
    "vial_front": (2 * VIAL_R, VIAL_H, 0.022, ACROSS_X),
    "vial_hidden": (2 * VIAL_R, VIAL_H, 0.022, ACROSS_X),
    "leaflets": (LEAFLETS[0], LEAFLETS[2], 0.012, ACROSS_X),
    "label_reel": (2 * REEL_R, REEL_H, 0.020, 0.0),
}


def _move(name, pick, place, label):
    """Top-down pick and place, settling before every close and release; the last 2 cm of the set-down slow. 14.1 s."""
    (px, py, pb), (qx, qy, qb) = pick, place
    width, height, depth, yaw = GRIP[name]
    travel = TRAVEL_HIGH if name == "carton" else TRAVEL
    gz = pb + height - depth + TIP
    qz = qb + height - depth + TIP + 0.002  # let go 2 mm above the surface
    open_ = -min(width + CLEAR, 0.092) / 2
    grip = -(width - SQUEEZE) / 2
    return [
        Waypoint(1.5, (px, py, travel), open_, f"{label}: over", yaw),
        Waypoint(1.5, (px, py, gz), open_, f"{label}: descend", yaw),
        Waypoint(0.5, (px, py, gz), open_, f"{label}: settle", yaw),
        Waypoint(1.2, (px, py, gz), grip, f"{label}: close", yaw),
        Waypoint(1.5, (px, py, travel), grip, f"{label}: lift", yaw),
        Waypoint(2.2, (qx, qy, travel), grip, f"{label}: carry", yaw),
        Waypoint(0.5, (qx, qy, travel), grip, f"{label}: settle over", yaw),
        Waypoint(1.5, (qx, qy, qz + 0.020), grip, f"{label}: lower", yaw),
        Waypoint(1.0, (qx, qy, qz), grip, f"{label}: set down slowly", yaw),
        Waypoint(0.5, (qx, qy, qz), grip, f"{label}: settle", yaw),
        Waypoint(1.2, (qx, qy, qz), open_, f"{label}: release", yaw),
        Waypoint(1.0, (qx, qy, travel), open_, f"{label}: retreat", yaw),
    ]


def demo():
    """Left arm only: the carton into the reconciliation bin, which uncovers the vial behind it; the front vial, the
    leaflets and the uncovered vial into the bin; then the new label reel from the staging mat onto the tile. 70.5 s."""
    plan = []
    for name in ("carton", "vial_front", "leaflets", "vial_hidden"):
        plan += _move(name, (*START[name], BED_TOP), (*BIN_SPOT[name], TOP), name.replace("_", " "))
    plan += _move("label_reel", (*START["label_reel"], STAGE_TOP), (*TILE, TILE_TOP), "label reel")
    return {"left": plan}


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states, so the history is exercised too."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    size = {"carton": CARTON, "leaflets": LEAFLETS, "vial_hidden": (2 * VIAL_R, 2 * VIAL_R, VIAL_H),
            "vial_front": (2 * VIAL_R, 2 * VIAL_R, VIAL_H), "label_reel": (2 * REEL_R, 2 * REEL_R, REEL_H)}

    def at(name, x, y, bottom, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        t = math.radians(tilt_deg)
        q = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
        half = np.array(size[name]) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {n: at(n, *START[n], BED_TOP) for n in LEFTOVERS}
    start["label_reel"] = at("label_reel", *START["label_reel"], STAGE_TOP)

    def binned(*names):
        return {n: at(n, *BIN_SPOT[n], TOP) for n in names}

    reel_on = {"label_reel": at("label_reel", TILE[0] + 0.004, TILE[1] - 0.003, TILE_TOP)}
    reel_over = {"label_reel": at("label_reel", *TILE, TILE_TOP + 0.010, vel=(0, 0, -0.05))}
    a = start | binned("carton")
    b = a | binned("vial_front", "leaflets")
    c = b | binned("vial_hidden")
    d = c | reel_on

    def run(*frames):
        for f in frames[:-1]:
            score(f)
        return score(frames[-1]), success(frames[-1])

    cases = {
        "start": ((start,), 0.0, False),
        "carton in the bin": ((start, a), 0.2, False),
        "three visible leftovers in the bin": ((start, a, b), 0.6, False),
        "all four in the bin": ((start, a, b, c), 0.8, False),
        "done, in order": ((start, a, b, c, c | reel_over, d), 1.0, True),
        "hidden vial missed, reel set out": ((start, a, b, b | reel_over, b | reel_on), 0.4, False),
        "hidden vial missed, reel set out, vial binned afterwards": ((start, a, b, b | reel_on, d), 0.4, False),
        "reel set out first, then everything cleared": ((start, start | reel_on, a | reel_on, d), 0.4, False),
        "reel carried high over the line early (not on it)": ((start, a | {"label_reel": at("label_reel", 0.36, 0.25, TOP + 0.10)}, b, c, d), 1.0, True),
        "carton only pushed aside on the table": ((start, start | {"carton": at("carton", 0.22, 0.30, TOP)}), 0.0, False),
        "hidden vial knocked over in the bin": ((start, a, b, c | {"vial_hidden": at("vial_hidden", 0.21, 0.47, TOP, tilt_deg=90.0)}, c | {"vial_hidden": at("vial_hidden", 0.21, 0.47, TOP, tilt_deg=90.0)} | reel_on), 1.0, True),
        "reel on the tile but tipped": ((start, a, b, c, c | {"label_reel": at("label_reel", *TILE, TILE_TOP, tilt_deg=30.0)}), 0.8, False),
        "reel off the tile on the bed": ((start, a, b, c, c | {"label_reel": at("label_reel", 0.40, 0.25, BED_TOP)}), 0.8, False),
        "a new trial after a violation starts clean": ((start, start | reel_on, start, a, b, c, d), 1.0, True),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    score(start)  # leave the history clean
    assert not violated()
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
