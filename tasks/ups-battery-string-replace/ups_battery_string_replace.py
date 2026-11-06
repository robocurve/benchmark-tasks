"""On the station table, in front of the left arm, the battery shelf of a UPS lies open: two separate fixed battery
trays side by side along y with a 4 cm gap between them, one per series string. Each tray is a dark floor plate with
three steel dividers forming two module slots, and a copper link strip along its near edge joining its two slots.
String A (the tray in front of the left arm) has one empty slot, where the technician already pulled the failed
module, and one old module still seated. String B (the tray further right) has both its old modules seated. A cart
nearer the arms carries two new modules; a black recycling bin further out already holds the failed module. Battery
makers' and UPS service rules do not allow new and aged modules in one series string, so the whole of string A is
replaced: its old module goes to the bin and both new modules go into string A, while string B stays untouched.
Scored from module poses, with the trial's history checked every step.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/ups-battery-string-replace/ups_battery_string_replace.py \\
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
MOD = (0.080, 0.045, 0.070)  # battery module: 80 mm long (x), 45 mm wide (y), 70 mm tall
MOD_KG = 0.300
SHELF_X = 0.300  # both trays' centreline, running along y
BASE_T = 0.008
FLOOR = TOP + BASE_T  # slot floor
CLEAR = 0.003  # air between a seated module and each divider
DIV = (0.080, 0.006, 0.035)  # divider: 80 mm long, 6 mm thick, 35 mm tall
PITCH = MOD[1] + 2 * CLEAR + DIV[1]  # 57 mm
TRAY_Y = {"A": 0.270, "B": 0.110}  # tray centres; 4 cm of clear table between the trays
TRAY_LEN = 2 * PITCH + DIV[1]  # 120 mm along y, outer face to outer face of the end dividers
TRAY_FLOOR = (0.120, TRAY_LEN, BASE_T)
LINK = (0.008, TRAY_LEN - 0.016, 0.003)  # copper series link along each tray's near edge, 1 cm clear of a module
LINK_X = SHELF_X - 0.054
SLOT_Y = {f"{s}{k}": TRAY_Y[s] + (1.5 - k) * PITCH for s in TRAY_Y for k in (1, 2)}  # slot 1 at the +y end
STRING = {s: (f"{s}1", f"{s}2") for s in TRAY_Y}

CART_X, CART_Y = 0.160, 0.270
CART = (0.120, 0.160, 0.010)  # the cart's fixed deck
CART_TOP = TOP + CART[2]
HANDLE = (0.010, 0.140, 0.040)
NEW_Y = {"new_1": 0.300, "new_2": 0.240}

BIN_XY = (0.420, 0.270)
BIN_INNER = (0.130, 0.150)
BIN_WALL_T, BIN_WALL_H = 0.006, 0.020
FAILED_XY = (0.420, 0.310)  # the failed module from slot A2, already in the bin
BIN_DROP = (0.420, 0.235)  # where the demo sets string A's old module down

# Who is where at the start: string A's old module in A1 (A2 is the empty slot of the failed module), B both full.
OLD_START = {"old_a1": "A1", "old_b1": "B1", "old_b2": "B2"}
OLD_B = ("old_b1", "old_b2")
NEW = tuple(NEW_Y)

OLD_C = (0.20, 0.21, 0.23)  # aged modules: dark grey
NEW_C = (0.27, 0.29, 0.33)  # new modules: a slightly bluer grey; the string rule, not the shade, gives the meaning
DARK = (0.12, 0.12, 0.13)
STEEL = (0.62, 0.63, 0.66)
COPPER = (0.78, 0.45, 0.20)
CART_C = (0.30, 0.45, 0.70)
BLACK = (0.06, 0.06, 0.06)


def _mod(name, x, y, bottom, color):
    return Box(name, pos=(x, y, bottom + MOD[2] / 2), size=MOD, mass=MOD_KG, color=color)


def _tray(s):
    cy = TRAY_Y[s]
    return [
        Box(f"tray_{s.lower()}_floor", pos=(SHELF_X, cy, TOP + BASE_T / 2), size=TRAY_FLOOR, color=DARK),
        *(Box(f"tray_{s.lower()}_divider_{j}", pos=(SHELF_X, cy + (1 - j) * PITCH, FLOOR + DIV[2] / 2), size=DIV,
              color=STEEL) for j in range(3)),
        Box(f"tray_{s.lower()}_link", pos=(LINK_X, cy, FLOOR + LINK[2] / 2), size=LINK, color=COPPER),
    ]


def _bin():
    cx, cy = BIN_XY
    ix, iy = BIN_INNER
    t, z = BIN_WALL_T, TOP + BIN_WALL_H / 2
    return [
        Box("bin_wall_ypos", pos=(cx, cy + iy / 2 + t / 2, z), size=(ix + 2 * t, t, BIN_WALL_H), color=BLACK),
        Box("bin_wall_yneg", pos=(cx, cy - iy / 2 - t / 2, z), size=(ix + 2 * t, t, BIN_WALL_H), color=BLACK),
        Box("bin_wall_near", pos=(cx - ix / 2 - t / 2, cy, z), size=(t, iy, BIN_WALL_H), color=BLACK),
        Box("bin_wall_far", pos=(cx + ix / 2 + t / 2, cy, z), size=(t, iy, BIN_WALL_H), color=BLACK),
    ]


MODULES = [
    *(_mod(n, SHELF_X, SLOT_Y[slot], FLOOR, OLD_C) for n, slot in OLD_START.items()),
    *(_mod(n, CART_X, y, CART_TOP, NEW_C) for n, y in NEW_Y.items()),
    _mod("failed_a2", *FAILED_XY, TOP, OLD_C),
]
FIXTURES = [
    *_tray("A"),
    *_tray("B"),
    Box("cart_deck", pos=(CART_X, CART_Y, TOP + CART[2] / 2), size=CART, color=CART_C),
    Box("cart_handle", pos=(CART_X - CART[0] / 2 - 0.005, CART_Y, TOP + HANDLE[2] / 2), size=HANDLE, color=STEEL),
    *_bin(),
]
OBJECTS = [*MODULES, *FIXTURES]
START = {m.name: m.pos for m in MODULES}

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

SEAT_X, SEAT_Y, SEAT_Z, SEAT_DEG = 0.010, 0.005, 0.003, 10.0  # seated in a slot
B_MOVE = 0.015  # a string-B module counts as removed once its centre is 15 mm from its start
DROP_Z = TOP - 0.030  # a module whose bottom goes 3 cm below the table top has fallen off it


def _yaw_tilt(quat_xyzw):
    """(yaw about the vertical, tilt of the up axis from the vertical), degrees."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _square(yaw_deg) -> float:
    """How far a module is from square to the tray; one turned 180 degrees is square again."""
    return abs((yaw_deg + 90.0) % 180.0 - 90.0)


def _seated(st, slot) -> bool:
    yaw, tilt = _yaw_tilt(st.quat_xyzw)
    return bool(
        abs(float(st.pos[1]) - SLOT_Y[slot]) <= SEAT_Y
        and abs(float(st.pos[0]) - SHELF_X) <= SEAT_X
        and abs(float(st.low[2]) - FLOOR) <= SEAT_Z
        and tilt <= SEAT_DEG
        and _square(yaw) <= SEAT_DEG
        and st.resting()
    )


def _in_tray(st, s) -> bool:
    """Centre over tray ``s`` (within 5 cm of the centreline, inside its length) and low enough to be in or on it."""
    return bool(
        abs(float(st.pos[0]) - SHELF_X) < 0.050
        and abs(float(st.pos[1]) - TRAY_Y[s]) < TRAY_LEN / 2
        and float(st.low[2]) < FLOOR + MOD[2] + 0.010
    )


def _in_bin(st) -> bool:
    """At rest, the centre inside the bin's inner walls, the bottom within 2 cm of the table."""
    return bool(
        abs(float(st.pos[0]) - BIN_XY[0]) < BIN_INNER[0] / 2
        and abs(float(st.pos[1]) - BIN_XY[1]) < BIN_INNER[1] / 2
        and TOP - 0.005 < float(st.low[2]) < TOP + 0.020
        and st.resting()
    )


def _b_in_place(st, name) -> bool:
    """A string-B module still in its own slot: centre within 15 mm of its start, upright within 10 degrees, bottom
    within 5 mm of the floor."""
    _, tilt = _yaw_tilt(st.quat_xyzw)
    return bool(math.dist([float(v) for v in st.pos], START[name]) <= B_MOVE and tilt <= SEAT_DEG
                and abs(float(st.low[2]) - FLOOR) <= 0.005)


def _b_moved(objects) -> bool:
    return any(math.dist([float(v) for v in objects[n].pos], START[n]) > B_MOVE for n in OLD_B)


def _dropped(objects) -> bool:
    return any(float(objects[m.name].low[2]) < DROP_Z for m in MODULES)


# The trial's history: whether a string-B module was ever lifted or pushed out of its slot, or any module fell off the
# table. The task file only sees object states, so the history resets whenever the scene is back at its start.
_HISTORY = {"b_removed": False, "dropped": False}


def _at_start(objects) -> bool:
    return all(math.dist([float(v) for v in objects[n].pos], p) < 0.005 for n, p in START.items())


def _update(objects):
    if _at_start(objects):
        _HISTORY["b_removed"] = _HISTORY["dropped"] = False
    _HISTORY["b_removed"] |= _b_moved(objects)
    _HISTORY["dropped"] |= _dropped(objects)
    return _HISTORY["b_removed"] or _HISTORY["dropped"]


def score(objects) -> float:
    """0.3 for each new module seated in a string-A slot, 0.3 for string A's old module at rest in the recycling bin,
    and 0.1 more when all three hold. String B must stay in place (both modules in their slots, upright). 0 if a new module ends up in string
    B's tray, a string-B module was ever moved 15 mm from its start (removed), or a module fell off the table."""
    if _update(objects):
        return 0.0
    if any(_in_tray(objects[n], "B") for n in NEW):
        return 0.0
    if not all(_b_in_place(objects[n], n) for n in OLD_B):
        return 0.0
    new_in_a = sum(any(_seated(objects[n], slot) for slot in STRING["A"]) for n in NEW)
    old_out = _in_bin(objects["old_a1"])
    s = 0.3 * new_in_a + 0.3 * old_out
    if new_in_a == 2 and old_out:
        s += 0.1
    return float(round(s, 6))


def success(objects) -> bool:
    """String A fully re-stringed with the two new modules, its old module in the bin, string B never touched; holds
    for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def history() -> dict:
    """The trial's latched violations (for the log)."""
    return dict(_HISTORY)


def distance(objects) -> float:
    """Metres to go: string A's old module to the bin centre plus each new module to its nearer string-A slot."""
    st = objects["old_a1"]
    d = math.hypot(float(st.pos[0]) - BIN_XY[0], float(st.pos[1]) - BIN_XY[1])
    for n in NEW:
        st = objects[n]
        d += min(math.dist((float(st.pos[0]), float(st.pos[1]), float(st.low[2])), (SHELF_X, SLOT_Y[s], FLOOR))
                 for s in STRING["A"])
    return float(d)


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = -math.pi / 2  # the fingers close along x, onto the module's two end faces; the dividers sit beside the pads
TIP = 0.013
TRAVEL = TOP + 0.150
SQUEEZE = 0.008
GRIP_DEPTH = 0.025  # pad tips 25 mm below the module's top
X_TRIM = 0.005  # a carried 300 g module lands 5-6 mm short in x (measured in sim); set it down further out


def _grip_z(bottom):
    return bottom + MOD[2] - GRIP_DEPTH + TIP


def _move(pick, place, label, trim=0.0):
    """Top-down pick and place across the module's 80 mm length with the fingers fully open (94 mm) to clear it,
    settling before every close and release, and lowering the last 4 cm slowly. 18.7 s."""
    (px, py, pb), (qx, qy, qb) = pick, place
    qx += trim
    gz, qz = _grip_z(pb), _grip_z(qb) + 0.002  # set down 2 mm above the surface
    grip = -(MOD[0] - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), FINGER_OPEN, f"{label}: over", YAW),
        Waypoint(1.5, (px, py, gz + 0.020), FINGER_OPEN, f"{label}: descend", YAW),
        Waypoint(1.0, (px, py, gz), FINGER_OPEN, f"{label}: down slowly", YAW),
        Waypoint(0.5, (px, py, gz), FINGER_OPEN, f"{label}: settle", YAW),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close", YAW),
        Waypoint(1.5, (px, py, gz + 0.040), grip, f"{label}: lift straight up", YAW),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{label}: lift", YAW),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{label}: carry", YAW),
        Waypoint(0.8, (qx, qy, TRAVEL), grip, f"{label}: settle over", YAW),
        Waypoint(1.5, (qx, qy, qz + 0.040), grip, f"{label}: lower", YAW),
        Waypoint(2.0, (qx, qy, qz), grip, f"{label}: set down slowly", YAW),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle", YAW),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, f"{label}: release", YAW),
        Waypoint(1.3, (qx, qy, TRAVEL), FINGER_OPEN, f"{label}: retreat", YAW),
    ]


def demo():
    """Left arm only: string A's old module from slot A1 into the recycling bin, then new module 1 into A1 and new
    module 2 into the empty slot A2. String B is never touched. 56.1 s."""
    return {"left": [
        *_move((SHELF_X, SLOT_Y["A1"], FLOOR), (*BIN_DROP, TOP), "old module A1 to the bin"),
        *_move((CART_X, NEW_Y["new_1"], CART_TOP), (SHELF_X, SLOT_Y["A1"], FLOOR), "new_1 into A1", trim=X_TRIM),
        *_move((CART_X, NEW_Y["new_2"], CART_TOP), (SHELF_X, SLOT_Y["A2"], FLOOR), "new_2 into A2", trim=X_TRIM),
    ]}


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states, so the history is exercised too."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(x, y, bottom, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
        x1, y1, z1, w1 = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)
        x2, y2, z2, w2 = math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
        half = np.array(MOD) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {m.name: box(m.pos[0], m.pos[1], m.pos[2] - MOD[2] / 2) for m in MODULES}

    def slot(name, s, dx=0.0, dy=0.0, **kw):
        return {name: box(SHELF_X + dx, SLOT_Y[s] + dy, FLOOR, **kw)}

    old_out = {"old_a1": box(*BIN_DROP, TOP)}
    lift_a1 = {"old_a1": box(SHELF_X, SLOT_Y["A1"], FLOOR + 0.05, vel=(0, 0, 0.05))}
    a = start | lift_a1
    b = start | old_out
    c = b | slot("new_1", "A1")
    done = c | slot("new_2", "A2")
    b1_off = {"old_b1": box(0.20, 0.10, TOP)}
    b_out = start | b1_off

    def run(*frames):
        """Feed the states in order (as the simulator would, step by step); the last score and success."""
        for f in frames[:-1]:
            score(f)
        return score(frames[-1]), success(frames[-1])

    cases = {
        "start": ((start,), 0.0, False),
        "old A1 lifted": ((start, a), 0.0, False),
        "naive: one new module into the empty slot A2": ((start, start | slot("new_1", "A2")), 0.3, False),
        "old A1 in the bin": ((start, a, b), 0.3, False),
        "old A1 in the bin, new_1 in A1": ((start, a, b, c), 0.6, False),
        "done": ((start, a, b, c, done), 1.0, True),
        "done, new modules the other way round": ((start, b, b | slot("new_2", "A1") | slot("new_1", "A2")), 1.0, True),
        "done, a new module turned 180 degrees and 6 mm out in x": ((start, b, done | slot("new_1", "A1", dx=0.006, yaw_deg=180)), 1.0, True),
        "both new in A but old A1 left on the table": ((start, start | {"old_a1": box(0.20, 0.38, TOP)} | slot("new_1", "A1") | slot("new_2", "A2")), 0.6, False),
        "new module in A tilted 15 degrees": ((start, b, c, done | slot("new_2", "A2", tilt_deg=15)), 0.6, False),
        "new module 7 mm off the slot centre along y": ((start, b, c, done | slot("new_2", "A2", dy=0.007)), 0.6, False),
        "new module still moving": ((start, b, c, done | slot("new_2", "A2", vel=(0, 0, -0.05))), 0.6, False),
        "new module held 3 cm above its slot": ((start, b, c, done | {"new_2": box(SHELF_X, SLOT_Y["A2"], FLOOR + 0.03)}), 0.6, False),
        "string B module lifted out and put back, A done": ((start, b, b | {"old_b1": box(SHELF_X, SLOT_Y["B1"], FLOOR + 0.04)}, c, done), 0.0, False),
        "string B module pulled to the bin, A done": ((start, done | {"old_b1": box(0.43, 0.20, TOP)}), 0.0, False),
        "new module into string B (B module swapped)": ((start, b_out, b_out | slot("new_1", "B1")), 0.0, False),
        "B module tipped 15 degrees in its slot, A done": ((start, b, c, done | slot("old_b2", "B2", tilt_deg=15)), 0.0, False),
        "B module nudged 3 mm, A done": ((start, b, c, done | slot("old_b2", "B2", dx=0.003)), 1.0, True),
        "old A1 knocked off the table": ((start, start | {"old_a1": box(0.30, 0.40, TOP - 0.5)}, done), 0.0, False),
        "a new trial after a violation starts clean": ((start, b_out, start, b, c, done), 1.0, True),
        "failed module moved into A1, new_1 in A2": ((start, b | slot("failed_a2", "A1") | slot("new_1", "A2")), 0.6, False),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    score(start)  # leave the history clean
    assert not any(history().values())
    assert distance(done) < 0.2 and distance(start) > distance(done)
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
