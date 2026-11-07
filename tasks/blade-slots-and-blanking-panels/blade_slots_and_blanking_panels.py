"""On the station table, in front of the left arm, a section of a vertical-blade compute chassis lies on its back so its
slots open upward: six slots in a row along y between fixed guide walls, on a fixed base, with an orange end plate at
the slot-1 end. Compute blades already stand in slots 1 and 5. A work order assigns two new blades to slots 2 and 4,
counting from the orange end; every slot still open after that must take a blanking panel (slots 3 and 6). Two new
blades (thick, tall) and three blanking panels (thin, short) stand on a spares mat nearer the arms; one panel is
surplus. Scored per slot from part poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/blade-slots-and-blanking-panels/blade_slots_and_blanking_panels.py \\
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
CH_X, CH_Y = 0.300, 0.250  # chassis centre; slots run along y
SLOTS = 6
BASE_T = 0.010
FLOOR = TOP + BASE_T  # slot floor
BLADE = (0.080, 0.030, 0.070)  # compute blade: 80 mm long (x), 30 mm thick (y), 70 mm tall (z)
PANEL = (0.080, 0.012, 0.040)  # blanking panel: same length, 12 mm thick, 40 mm tall
BLADE_KG, PANEL_KG = 0.200, 0.030
GAP_W = 0.036  # clear width of a slot: 3 mm each side of a blade
WALL = (0.050, 0.006, 0.030)  # guide wall between slots: 50 mm long, 6 mm thick, 30 mm tall
PITCH = GAP_W + WALL[1]
END = (0.060, 0.010, 0.045)  # end plates
SLOT_Y = {k: CH_Y + (2 * k - SLOTS - 1) * PITCH / 2 for k in range(1, SLOTS + 1)}  # slot 1 at the orange end (low y)
END_Y = {"orange": SLOT_Y[1] - GAP_W / 2 - END[1] / 2, "grey": SLOT_Y[SLOTS] + GAP_W / 2 + END[1] / 2}
BASE = (0.100, END_Y["grey"] - END_Y["orange"] + END[1], BASE_T)

INSTALLED = (1, 5)  # slots already holding blades
BLADE_SLOTS = (2, 4)  # the work order
PANEL_SLOTS = tuple(k for k in range(1, SLOTS + 1) if k not in INSTALLED + BLADE_SLOTS)  # (3, 6)

MAT_X = 0.180
MAT = (0.110, 0.260, 0.002)
MAT_Y = 0.250
SPARE_Y = {"panel_a": 0.150, "blade_a": 0.200, "panel_b": 0.250, "blade_b": 0.300, "panel_c": 0.350}

DARK = (0.16, 0.17, 0.19)
STEEL = (0.62, 0.63, 0.66)
ORANGE = (0.93, 0.45, 0.08)
GREEN = (0.10, 0.45, 0.22)  # blades: green circuit-board colour
SILVER = (0.80, 0.81, 0.83)  # panels
MATC = (0.30, 0.32, 0.36)


def _part(name, kind, x, y, bottom):
    size, kg, color = (BLADE, BLADE_KG, GREEN) if kind == "blade" else (PANEL, PANEL_KG, SILVER)
    return Box(name, pos=(x, y, bottom + size[2] / 2), size=size, mass=kg, color=color)


KIND = {f"blade_{k}": "blade" for k in INSTALLED} | {n: n.split("_")[0] for n in SPARE_Y}
NEW_BLADES = [n for n in SPARE_Y if KIND[n] == "blade"]
PANELS = [n for n in SPARE_Y if KIND[n] == "panel"]
PARTS = [
    *(_part(f"blade_{k}", "blade", CH_X, SLOT_Y[k], FLOOR) for k in INSTALLED),
    *(_part(n, KIND[n], MAT_X, y, TOP + MAT[2]) for n, y in SPARE_Y.items()),
]
_wall_z = FLOOR + WALL[2] / 2
FIXTURES = [
    Box("chassis_base", pos=(CH_X, CH_Y, TOP + BASE_T / 2), size=BASE, color=DARK),
    *(Box(f"guide_wall_{k}", pos=(CH_X, (SLOT_Y[k] + SLOT_Y[k + 1]) / 2, _wall_z), size=WALL, color=STEEL)
      for k in range(1, SLOTS)),
    Box("end_plate_orange", pos=(CH_X, END_Y["orange"], FLOOR + END[2] / 2), size=END, color=ORANGE),
    Box("end_plate_grey", pos=(CH_X, END_Y["grey"], FLOOR + END[2] / 2), size=END, color=STEEL),
    Box("spares_mat", pos=(MAT_X, MAT_Y, TOP + MAT[2] / 2), size=MAT, color=MATC),
]
OBJECTS = [*PARTS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

IN_X = 0.020  # in a slot: centre within 2 cm of the chassis centreline across the slots,
IN_Z = 0.010  # inside the slot's clear width along y, bottom within 1 cm of the slot floor
SEAT_X = 0.010  # seated: centre within 1 cm across,
SEAT_Y = {"blade": 0.004, "panel": 0.010}  # within 4 mm (blade) / 1 cm (panel) of the slot centre along y,
SEAT_Z = 0.003  # bottom within 3 mm of the floor,
SEAT_DEG = 8.0  # upright and square within 8 degrees, still


def _yaw_tilt(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _square(yaw_deg) -> float:
    return abs((yaw_deg + 90.0) % 180.0 - 90.0)


def _in_slot(st, k) -> bool:
    return bool(
        abs(float(st.pos[1]) - SLOT_Y[k]) < GAP_W / 2
        and abs(float(st.pos[0]) - CH_X) < IN_X
        and float(st.low[2]) < FLOOR + IN_Z
    )


def _seated(st, k, kind) -> bool:
    yaw, tilt = _yaw_tilt(st.quat_xyzw)
    return bool(
        _in_slot(st, k)
        and abs(float(st.pos[1]) - SLOT_Y[k]) <= SEAT_Y[kind]
        and abs(float(st.pos[0]) - CH_X) <= SEAT_X
        and abs(float(st.low[2]) - FLOOR) <= SEAT_Z
        and tilt <= SEAT_DEG
        and _square(yaw) <= SEAT_DEG
        and st.resting()
    )


def _contents(objects):
    return {k: [n for n in KIND if _in_slot(objects[n], k)] for k in range(1, SLOTS + 1)}


def _slot_ok(objects, contents, k, kind) -> bool:
    """Exactly one part in the slot, a new part of the right kind, seated."""
    held = contents[k]
    return len(held) == 1 and held[0] in SPARE_Y and KIND[held[0]] == kind and _seated(objects[held[0]], k, kind)


def score(objects) -> float:
    """0.25 for each of the four open slots that ends right: a new blade seated in slots 2 and 4, one blanking panel
    seated in slots 3 and 6. Capped at 0.25 if a blanking panel is in a blade slot (2 or 4), a new blade is in a
    panel slot (3 or 6), or either installed blade is no longer seated in slots 1 and 5."""
    c = _contents(objects)
    cap = 1.0
    if not all(_seated(objects[f"blade_{k}"], k, "blade") for k in INSTALLED):
        cap = 0.25
    if any(KIND[n] == "panel" for k in BLADE_SLOTS for n in c[k]):
        cap = 0.25
    if any(n in NEW_BLADES for k in PANEL_SLOTS for n in c[k]):
        cap = 0.25
    good = sum(_slot_ok(objects, c, k, "blade") for k in BLADE_SLOTS)
    good += sum(_slot_ok(objects, c, k, "panel") for k in PANEL_SLOTS)
    return float(min(cap, 0.25 * good))


def success(objects) -> bool:
    """Blades seated in slots 2 and 4, panels seated in 3 and 6, installed blades still seated; holds for 1 s."""
    return score(objects) >= 1.0


def distance(objects) -> float:
    """Metres to go: each new blade and the two nearest panels to their target slots, best assignment."""

    def d(n, k):
        st = objects[n]
        return math.dist((float(st.pos[0]), float(st.pos[1]), float(st.low[2])), (CH_X, SLOT_Y[k], FLOOR))

    a, b = NEW_BLADES
    blades = min(d(a, 2) + d(b, 4), d(a, 4) + d(b, 2))
    panels = min(d(p, PANEL_SLOTS[0]) + d(q, PANEL_SLOTS[1]) for p in PANELS for q in PANELS if p != q)
    return float(blades + panels)


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = -math.pi / 2  # the fingers close along x, onto the part's two end faces; the slot walls sit between the pads
TIP = 0.013
TRAVEL = TOP + 0.160
SQUEEZE = 0.008
GRIP_DEPTH = {"blade": 0.025, "panel": 0.020}  # pad tips this far below the part's top
X_TRIM = {"blade": 0.0, "panel": 0.0}  # none needed: in sim blades land ~4.5 mm short in x, panels ~0 (seat tolerance 1 cm)


def _grip_z(kind, bottom):
    size = BLADE if kind == "blade" else PANEL
    return bottom + size[2] - GRIP_DEPTH[kind] + TIP


def _move(kind, pick, place, label):
    """Top-down pick and place across the part's 80 mm length with the fingers fully open (94 mm) to clear it,
    settling before every close and release, and lowering the last 4 cm into the slot slowly. 18.2 s."""
    (px, py, pb), (qx, qy, qb) = pick, place
    qx += X_TRIM[kind]
    gz, qz = _grip_z(kind, pb), _grip_z(kind, qb) + 0.0015  # set down 1.5 mm above the floor
    size = BLADE if kind == "blade" else PANEL
    grip = -(size[0] - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), FINGER_OPEN, f"{label}: over", YAW),
        Waypoint(1.5, (px, py, gz + 0.020), FINGER_OPEN, f"{label}: descend", YAW),
        Waypoint(1.0, (px, py, gz), FINGER_OPEN, f"{label}: down slowly", YAW),
        Waypoint(0.5, (px, py, gz), FINGER_OPEN, f"{label}: settle", YAW),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close", YAW),
        Waypoint(1.5, (px, py, gz + 0.030), grip, f"{label}: lift straight up", YAW),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{label}: lift", YAW),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{label}: carry", YAW),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{label}: settle over", YAW),
        Waypoint(1.5, (qx, qy, qz + 0.040), grip, f"{label}: lower over the slot", YAW),
        Waypoint(2.0, (qx, qy, qz), grip, f"{label}: down into the slot slowly", YAW),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle", YAW),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, f"{label}: release", YAW),
        Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, f"{label}: retreat", YAW),
    ]


def demo():
    """Left arm only: the two new blades into slots 2 and 4, then blanking panels into slots 3 and 6; the third
    panel stays on the mat. 72.8 s."""
    spare_bottom = TOP + MAT[2]
    plan = []
    for name, k in (("blade_a", 2), ("blade_b", 4), ("panel_a", 3), ("panel_b", 6)):
        kind = KIND[name]
        plan += _move(kind, (MAT_X, SPARE_Y[name], spare_bottom), (CH_X, SLOT_Y[k], FLOOR), f"{name} to slot {k}")
    return {"left": plan}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(kind, x, y, bottom, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
        x1, y1, z1, w1 = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)
        x2, y2, z2, w2 = math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
        half = np.array(BLADE if kind == "blade" else PANEL) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {p.name: box(KIND[p.name], p.pos[0], p.pos[1], p.pos[2] - (BLADE if KIND[p.name] == "blade" else PANEL)[2] / 2)
             for p in PARTS}

    def at(name, k, dy=0.0, **kw):
        return {name: box(KIND[name], CH_X, SLOT_Y[k] + dy, FLOOR, **kw)}

    def scene(*parts):
        out = dict(start)
        for p in parts:
            out |= p
        return out

    done = [at("blade_a", 2), at("blade_b", 4), at("panel_a", 3), at("panel_b", 6)]
    cases = {
        "start": (scene(), 0.0),
        "done": (scene(*done), 1.0),
        "done, blades swapped and panel c used": (scene(at("blade_b", 2), at("blade_a", 4), at("panel_c", 3), at("panel_b", 6)), 1.0),
        "done, panel 8 mm off centre in its slot": (scene(*done[:3], at("panel_b", 6, dy=0.008)), 1.0),
        "blades only": (scene(*done[:2]), 0.5),
        "one blade, one panel": (scene(done[0], done[2]), 0.5),
        "blades and one panel": (scene(*done[:3]), 0.75),
        "panels only": (scene(*done[2:]), 0.5),
        "work order counted from the grey end (blade in 3, panels in 2, 4, 6)": (
            scene(at("blade_a", 3), at("panel_a", 2), at("panel_b", 4), at("panel_c", 6)), 0.25),
        "every open slot blanked": (scene(at("panel_a", 2), at("panel_b", 3), at("panel_c", 4)), 0.25),
        "done but a panel in slot 6 replaced by a blade": (scene(*done[:3], at("blade_a", 6), at("blade_b", 2)), 0.25),
        "blade leaning in slot 2": (scene(at("blade_a", 2, tilt_deg=12), *done[1:]), 0.75),
        "blade turned 15 degrees in slot 4": (scene(done[0], at("blade_b", 4, yaw_deg=15), *done[2:]), 0.75),
        "panel still moving in slot 3": (scene(*done[:2], at("panel_a", 3, vel=(0, 0, -0.05)), done[3]), 0.75),
        "blade held 3 cm above slot 2": (scene({"blade_a": box("blade", CH_X, SLOT_Y[2], FLOOR + 0.03)}, *done[1:]), 0.75),
        "done but installed blade 5 pulled out": (scene(*done, {"blade_5": box("blade", 0.20, 0.10, TOP)}), 0.25),
        "two panels in slot 3": (scene(*done[:2], at("panel_a", 3, dy=-0.010), at("panel_c", 3, dy=0.010), done[3]), 0.75),
        "done, blade turned 180 degrees": (scene(at("blade_a", 2, yaw_deg=180), *done[1:]), 1.0),
    }
    for name, (st, want) in cases.items():
        got = score(st)
        assert abs(got - want) < 1e-9 and success(st) == (want >= 1.0), (name, got, want)
    assert distance(scene(*done)) < 1e-9
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
