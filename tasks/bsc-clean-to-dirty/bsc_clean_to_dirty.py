"""On the station table, across the front of both arms, the work surface of a Class II biological safety cabinet: a
stainless work tray (x 20-46 cm) between a dark perforated front grille along its near edge (x 16-20 cm) and a dark
rear grille along its far edge (x 46-50 cm), with low white side walls and a back wall. The air curtain that protects
the worker and the work is drawn in through both grilles. Four items were set down in a row just inside the front
grille, 5 cm in: from left to right (+y to -y) a discard pan (dirty), a stack of culture plates (the work), a bottle
of sterile medium (clean) and a box of sterile pipette tips (clean). Cabinet practice: everything at least 10 cm in
from the front grille, nothing on the rear grille, and the surface laid out clean to dirty from left to right, so
the clean supplies are on the left, the work in the middle and the discard pan on the right. Scored from item poses.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/bsc-clean-to-dirty/bsc_clean_to_dirty.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
PLATE_T = 0.003  # the tray and both grilles are 3 mm steel, flush
SURF = TOP + PLATE_T
FRONT_GRILLE = (0.160, 0.200)  # x from, to
REAR_GRILLE = (0.460, 0.500)
HALF_W = 0.360  # the cabinet's inside half-width along y
INSET = 0.100  # 10 cm in from the front grille
ZONE_X = (FRONT_GRILLE[1] + INSET, REAR_GRILLE[0])  # every item's whole footprint must lie in x 0.30 .. 0.46

# name: (shape, dims, mass, colour, role); Box dims (x, y, z), Cylinder dims (radius, height)
ITEMS = {
    "tip_box": ("box", (0.060, 0.045, 0.040), 0.060, (0.30, 0.55, 0.85), "clean"),
    "media_bottle": ("cyl", (0.022, 0.085), 0.180, (0.95, 0.70, 0.55), "clean"),
    "culture_plates": ("cyl", (0.034, 0.032), 0.070, (0.85, 0.88, 0.80), "work"),
    "discard_pan": ("box", (0.064, 0.070, 0.050), 0.120, (0.55, 0.55, 0.58), "dirty"),
}
ROW_X = 0.250  # the start row, only 5 cm in from the front grille
START = {"discard_pan": (ROW_X, 0.100), "culture_plates": (ROW_X, 0.033), "media_bottle": (ROW_X, -0.033),
         "tip_box": (ROW_X, -0.100)}
TARGET = {"tip_box": (0.380, 0.240), "media_bottle": (0.380, 0.090), "culture_plates": (0.380, -0.090),
          "discard_pan": (0.380, -0.240)}  # where the demo sets each one down
ORDER_GAP = 0.030  # centres at least 3 cm apart along y to count as left of / right of


def _height(name):
    shape, dims, *_ = ITEMS[name]
    return dims[2] if shape == "box" else dims[1]


STEEL = (0.78, 0.79, 0.80)
DARK = (0.20, 0.21, 0.23)
WHITE = (0.92, 0.92, 0.92)


def _item(name):
    shape, dims, mass, color, _ = ITEMS[name]
    x, y = START[name]
    if shape == "box":
        return Box(name, pos=(x, y, SURF + dims[2] / 2), size=dims, mass=mass, color=color)
    return Cylinder(name, pos=(x, y, SURF + dims[1] / 2), radius=dims[0], height=dims[1], mass=mass, color=color)


ITEM_OBJECTS = [_item(n) for n in ITEMS]
_mid = (FRONT_GRILLE[1] + REAR_GRILLE[0]) / 2
FIXTURES = [
    Box("work_tray", pos=(_mid, 0.0, TOP + PLATE_T / 2), size=(REAR_GRILLE[0] - FRONT_GRILLE[1], 2 * HALF_W, PLATE_T),
        color=STEEL),
    Box("front_grille", pos=(sum(FRONT_GRILLE) / 2, 0.0, TOP + PLATE_T / 2),
        size=(FRONT_GRILLE[1] - FRONT_GRILLE[0], 2 * HALF_W, PLATE_T), color=DARK),
    Box("rear_grille", pos=(sum(REAR_GRILLE) / 2, 0.0, TOP + PLATE_T / 2),
        size=(REAR_GRILLE[1] - REAR_GRILLE[0], 2 * HALF_W, PLATE_T), color=DARK),
    Box("side_wall_left", pos=(0.340, HALF_W + 0.005, TOP + 0.030), size=(0.360, 0.010, 0.060), color=WHITE),
    Box("side_wall_right", pos=(0.340, -HALF_W - 0.005, TOP + 0.030), size=(0.360, 0.010, 0.060), color=WHITE),
    Box("back_wall", pos=(0.510, 0.0, TOP + 0.030), size=(0.010, 2 * HALF_W + 0.020, 0.060), color=WHITE),
]
OBJECTS = [*ITEM_OBJECTS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _tilt(quat_xyzw) -> float:
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def placed(st) -> bool:
    """Standing upright and still on the work tray with its whole footprint 10 cm or more inside the front grille,
    off the rear grille and inside the side walls."""
    return bool(
        float(st.low[0]) >= ZONE_X[0]
        and float(st.high[0]) <= ZONE_X[1]
        and float(st.low[1]) >= -HALF_W
        and float(st.high[1]) <= HALF_W
        and abs(float(st.low[2]) - SURF) < 0.010
        and _tilt(st.quat_xyzw) < 10.0
        and st.resting()
    )


def ordered(objects) -> bool:
    """Clean to dirty from left (+y) to right: both clean items left of the culture plates, the plates left of the
    discard pan, each by at least 3 cm."""
    y = {n: float(objects[n].pos[1]) for n in ITEMS}
    clean = min(y["tip_box"], y["media_bottle"])
    return bool(clean - y["culture_plates"] >= ORDER_GAP and y["culture_plates"] - y["discard_pan"] >= ORDER_GAP)


def score(objects) -> float:
    """0.15 for each item placed in the work zone (whole footprint 10 cm or more in from the front grille, off the
    rear grille, upright and still); 0.4 more when all four are placed and laid out clean to dirty, left to right."""
    n = sum(placed(objects[name]) for name in ITEMS)
    s = 0.15 * n
    if n == len(ITEMS) and ordered(objects):
        s += 0.4
    return float(s)


def success(objects) -> bool:
    """All four items in the work zone, clean to dirty from left to right; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres from each item to the demo's set-down point."""
    return float(sum(math.hypot(float(objects[n].pos[0]) - TARGET[n][0], float(objects[n].pos[1]) - TARGET[n][1])
                     for n in ITEMS))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # the fingers close along x, so the pads never come down between
# neighbours in the row (along y); each arm turns the way that keeps its wrist inside its limits across the midline
TIP = 0.013
TRAVEL = SURF + 0.120
ROW_TRAVEL = 0.090  # TCP height over the start row (above the tray), which lies across the midline at the edge of
# each arm's reach; raised for the tall bottle so the pad tips stay 1.2 cm above its top
AISLE_X = 0.160  # the parts are carried sideways over the front grille, clear of the row
SQUEEZE = 0.008
CLEAR = 0.020
X_TRIM = {"tip_box": 0.002, "media_bottle": 0.004, "culture_plates": 0.002, "discard_pan": 0.003}


def _across(name):
    """The item's width across the closing axis (x)."""
    shape, dims, *_ = ITEMS[name]
    return dims[0] if shape == "box" else 2 * dims[0]


def _grip_z(name, bottom):
    h = _height(name)
    return bottom + max(0.006, h - 0.035) + TIP  # pad tips 6 mm above the surface, or 35 mm below a tall item's top


def _move(name, arm):
    """Pick from the row (approached and left over the front grille), carry sideways over the front grille, then
    forward to the set-down point. 18.5 s."""
    (px, py), (qx, qy) = START[name], TARGET[name]
    qx += X_TRIM[name]
    yaw = YAW[arm]
    gz, qz = _grip_z(name, SURF), _grip_z(name, SURF) + 0.002
    row = SURF + max(ROW_TRAVEL, _height(name) + TIP + 0.012)
    w = _across(name)
    open_ = -min(w + CLEAR, 0.093) / 2
    grip = -(w - SQUEEZE) / 2
    return [
        Waypoint(1.8, (AISLE_X, py, TRAVEL), open_, f"{name}: to the aisle in front of it", yaw),
        Waypoint(1.2, (px, py, row), open_, f"{name}: over", yaw),
        Waypoint(1.5, (px, py, gz + 0.020), open_, f"{name}: descend", yaw),
        Waypoint(0.8, (px, py, gz), open_, f"{name}: down slowly", yaw),
        Waypoint(0.5, (px, py, gz), open_, f"{name}: settle", yaw),
        Waypoint(1.2, (px, py, gz), grip, f"{name}: close", yaw),
        Waypoint(1.3, (px, py, row), grip, f"{name}: lift", yaw),
        Waypoint(1.2, (AISLE_X, py, TRAVEL), grip, f"{name}: back over the front grille", yaw),
        Waypoint(2.0, (AISLE_X, qy, TRAVEL), grip, f"{name}: across", yaw),
        Waypoint(1.5, (qx, qy, TRAVEL), grip, f"{name}: forward into the work zone", yaw),
        Waypoint(0.5, (qx, qy, TRAVEL), grip, f"{name}: settle over", yaw),
        Waypoint(1.5, (qx, qy, qz + 0.020), grip, f"{name}: lower", yaw),
        Waypoint(1.0, (qx, qy, qz), grip, f"{name}: set down slowly", yaw),
        Waypoint(0.5, (qx, qy, qz), grip, f"{name}: settle", yaw),
        Waypoint(1.0, (qx, qy, qz), open_, f"{name}: release", yaw),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{name}: retreat", yaw),
    ]


def demo():
    """The left arm takes the two clean items out of the row to the left of the work zone, then parks; the right arm
    waits, then takes the culture plates to the middle-right and the discard pan to the far right. 76.0 s."""
    left = _move("tip_box", "left") + _move("media_bottle", "left")
    left.append(Waypoint(2.0, (0.200, 0.320, TRAVEL), -0.02, "park clear of the row", YAW["left"]))
    t_left = sum(w.duration for w in left)
    right = [Waypoint(3.0, (0.200, -0.320, TRAVEL), -0.02, "wait clear of the row", YAW["right"]),
             Waypoint(t_left - 3.0, (0.200, -0.320, TRAVEL), -0.02, "wait", YAW["right"])]
    right += _move("culture_plates", "right") + _move("discard_pan", "right")
    return {"left": left, "right": right}


def self_test(ObjectState=None):
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def item(name, x, y, bottom=SURF, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        shape, dims, *_ = ITEMS[name]
        t = math.radians(tilt_deg)
        q = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
        h = _height(name)
        half = np.array([dims[0] / 2, dims[1] / 2, h / 2]) if shape == "box" else np.array([dims[0], dims[0], h / 2])
        pos = np.array([x, y, bottom + h / 2])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {n: item(n, *START[n]) for n in ITEMS}
    done = {n: item(n, *TARGET[n]) for n in ITEMS}
    cases = {
        "start": (start, 0.0, False),
        "done": (done, 1.0, True),
        "done, the two clean items swapped": (done | {"tip_box": item("tip_box", 0.38, 0.09),
                                                      "media_bottle": item("media_bottle", 0.38, 0.24)}, 1.0, True),
        "all inset but in the start order (dirty on the left)": (
            {n: item(n, 0.38, START[n][1] * 2) for n in ITEMS}, 0.6, False),
        "plates and pan swapped": (done | {"culture_plates": item("culture_plates", 0.38, -0.24),
                                           "discard_pan": item("discard_pan", 0.38, -0.09)}, 0.6, False),
        "media bottle right of the plates": (done | {"media_bottle": item("media_bottle", 0.38, -0.16)}, 0.6, False),
        "pan only 9 cm in from the front grille": (done | {"discard_pan": item("discard_pan", 0.322, -0.24)}, 0.45, False),
        "tip box half on the rear grille": (done | {"tip_box": item("tip_box", 0.45, 0.24)}, 0.45, False),
        "media bottle knocked over": (done | {"media_bottle": item("media_bottle", 0.38, 0.09, tilt_deg=90)}, 0.45, False),
        "pan still moving": (done | {"discard_pan": item("discard_pan", 0.38, -0.24, vel=(0, 0, -0.05))}, 0.45, False),
        "only the clean items moved": (start | {n: done[n] for n in ("tip_box", "media_bottle")}, 0.3, False),
        "plates 2 cm right of the bottle only": (done | {"culture_plates": item("culture_plates", 0.38, 0.07)}, 0.6, False),
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
