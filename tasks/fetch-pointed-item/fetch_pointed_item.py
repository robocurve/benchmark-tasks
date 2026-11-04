"""On the station table, in the left arm's lane, a 1 g bench analogue of a crewmate who cannot speak asking for an
item by pointing, as after a stroke, while intubated, or in a loud module with a failed headset. A fixed crew
stand-in sits across the table facing the arms: a torso and head, and a right forearm, palm and index finger lying
on the table in one straight line, 30 degrees outboard of straight toward the arms. Four identical cylinders stand
in front of the left arm. Only the green one lies on the line of the finger. The red one is nearest the fingertip
and straight in front of the left arm, the blue one is straight toward the arms from the fingertip, nearest the left
arm's base and farthest from the crewmate, and the yellow one is nearest the tray, so every one of those habits
brings the wrong item. The green item has to end upright on the white tray and the other three must stay on their
marks. Every test uses each item's centre and quaternion; the lowest point and the outline seen from above come
from the cylinder's axis, not the axis-aligned bounds.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/fetch-pointed-item/fetch_pointed_item.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "The crewmate cannot speak and is pointing at the item they need. Put that item upright on the white tray and leave every other item where it is." \\
           --scorer task_score --max-steps 2400
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
TIP = (0.330, 0.245)  # the index fingertip
POINT = math.radians(150.0)  # the pointing direction: toward the arms and 30 degrees outboard
DIR = (math.cos(POINT), math.sin(POINT))
ARM_YAW = POINT - math.pi  # each of the forearm, palm and finger boxes has its first dimension along the pointing line
FINGER = (0.080, 0.018, 0.016)
PALM = (0.060, 0.080, 0.030)
FOREARM = (0.140, 0.060, 0.045)
TORSO = (0.120, 0.300, 0.180)  # low enough that the head stays inside the top camera's frame
TORSO_XY = (0.6475, 0.000)  # its front face at x = 0.5875 touches the forearm's back corner
HEAD_R, HEAD_H = 0.060, 0.120

ITEM_R, ITEM_H, ITEM_MASS = 0.020, 0.060, 0.05
GREEN = "item_green"  # the one pointed at
STARTS = {
    GREEN: (0.200, 0.320),  # on the line, 150 mm past the fingertip
    "item_red": (0.335, 0.305),  # 60 mm from the fingertip, 65 degrees off the line, on the left arm's centreline
    "item_blue": (0.130, 0.245),  # 200 mm from the fingertip toward the arms (-x), 30 degrees off: nearest the left base, farthest from the crewmate
    "item_yellow": (0.200, 0.170),  # 150 mm from the fingertip, 60 degrees off: nearest the tray
}
OTHERS = [n for n in STARTS if n != GREEN]

TRAY = (0.170, 0.070)  # tray centre, 290 mm from the left base, 78 degrees off the line
FLOOR = (0.090, 0.090, 0.006)  # the tray floor between the rims
RIM_T, RIM_H = 0.006, 0.015  # rim walls standing on the table around the floor
FLOOR_TOP = TOP + FLOOR[2]
TRAY_HALF = FLOOR[0] / 2 + RIM_T  # 51 mm: the tray's outer outline, rims included

SLEEVE = (0.30, 0.35, 0.50)  # the crew stand-in's flight suit: torso and forearm
SKIN = (0.45, 0.30, 0.22)  # head, palm and finger
WHITE = (0.95, 0.95, 0.95)
COLORS = {GREEN: (0.20, 0.65, 0.25), "item_red": (0.85, 0.15, 0.15), "item_blue": (0.15, 0.35, 0.85), "item_yellow": (0.95, 0.80, 0.10)}


def _along(s):
    """The point s metres back from the fingertip along the pointing line."""
    return (TIP[0] - s * DIR[0], TIP[1] - s * DIR[1])


CREW = [
    Box("crew_finger", pos=(*_along(FINGER[0] / 2), TOP + FINGER[2] / 2), size=FINGER, color=SKIN, yaw=ARM_YAW),
    Box("crew_palm", pos=(*_along(FINGER[0] + PALM[0] / 2), TOP + PALM[2] / 2), size=PALM, color=SKIN, yaw=ARM_YAW),
    Box("crew_forearm", pos=(*_along(FINGER[0] + PALM[0] + FOREARM[0] / 2), TOP + FOREARM[2] / 2), size=FOREARM, color=SLEEVE, yaw=ARM_YAW),
    Box("crew_torso", pos=(*TORSO_XY, TOP + TORSO[2] / 2), size=TORSO, color=SLEEVE),
    Cylinder("crew_head", pos=(*TORSO_XY, TOP + TORSO[2] + HEAD_H / 2), radius=HEAD_R, height=HEAD_H, color=SKIN),
]
RIM_OFF = FLOOR[0] / 2 + RIM_T / 2  # 48 mm, a rim's centre from the tray centre
TRAY_PARTS = [
    Box("tray_floor", pos=(*TRAY, TOP + FLOOR[2] / 2), size=FLOOR, color=WHITE),
    Box("tray_rim_xp", pos=(TRAY[0] + RIM_OFF, TRAY[1], TOP + RIM_H / 2), size=(RIM_T, FLOOR[1], RIM_H), color=WHITE),
    Box("tray_rim_xn", pos=(TRAY[0] - RIM_OFF, TRAY[1], TOP + RIM_H / 2), size=(RIM_T, FLOOR[1], RIM_H), color=WHITE),
    Box("tray_rim_yp", pos=(TRAY[0], TRAY[1] + RIM_OFF, TOP + RIM_H / 2), size=(2 * TRAY_HALF, RIM_T, RIM_H), color=WHITE),
    Box("tray_rim_yn", pos=(TRAY[0], TRAY[1] - RIM_OFF, TOP + RIM_H / 2), size=(2 * TRAY_HALF, RIM_T, RIM_H), color=WHITE),
]
ITEMS = [Cylinder(n, pos=(x, y, TOP + ITEM_H / 2), radius=ITEM_R, height=ITEM_H, mass=ITEM_MASS, color=COLORS[n]) for n, (x, y) in STARTS.items()]
OBJECTS = [*ITEMS, *TRAY_PARTS, *CREW]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

TILT_TOL = math.radians(10.0)  # upright, either end down
SEAT_TOL = 0.003  # the green item's lowest point to the tray floor; one standing on the table is 6 mm off
MOVE_TOL = 0.020  # another item's centre to its mark
TABLE_TOL = 0.004  # another item's lowest point to the table
OVER_TOL = 0.002  # seen from above, an item's outline reaching this far into the tray's outer outline is on the tray
RIM_N = 24  # points on each end circle of a cylinder's outline
TRAY_POLY = np.array([(TRAY[0] + sx * TRAY_HALF, TRAY[1] + sy * TRAY_HALF) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _tilt(state) -> float:
    """Radians between the cylinder's axis and the vertical, either end down."""
    return math.acos(min(1.0, abs(float(_rot(state.quat_xyzw)[2, 2]))))


def _bottom(state) -> float:
    """The cylinder's lowest point from its centre and axis; a lying cylinder's axis-aligned bounds read too low."""
    uz = abs(float(_rot(state.quat_xyzw)[2, 2]))
    return float(state.pos[2]) - (ITEM_H / 2 * uz + ITEM_R * math.sqrt(max(0.0, 1.0 - uz * uz)))


def _hull(points) -> np.ndarray:
    """Convex hull of 2-D points, counter-clockwise (monotone chain)."""
    pts = sorted({(round(float(p[0]), 9), round(float(p[1]), 9)) for p in points})

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and (out[-1][0] - out[-2][0]) * (p[1] - out[-2][1]) - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0]) <= 0:
                out.pop()
            out.append(p)
        return out[:-1]

    return np.array(half(pts) + half(pts[::-1]))


def _outline(state) -> np.ndarray:
    """The cylinder seen from above: the hull of its two end circles, from its centre and quaternion."""
    t = np.linspace(0.0, 2.0 * math.pi, RIM_N, endpoint=False)
    local = np.array([(ITEM_R * math.cos(a), ITEM_R * math.sin(a), s * ITEM_H / 2) for s in (-1.0, 1.0) for a in t])
    world = np.asarray(state.pos, float) + local @ _rot(state.quat_xyzw).T
    return _hull(world[:, :2])


def _overlap(a, b) -> float:
    """Penetration depth of two convex outlines (separating-axis test): negative when they are apart."""
    depth = math.inf
    for poly in (a, b):
        for k in range(len(poly)):
            edge = poly[(k + 1) % len(poly)] - poly[k]
            axis = np.array([-edge[1], edge[0]]) / max(float(np.hypot(*edge)), 1e-12)
            pa, pb = a @ axis, b @ axis
            depth = min(depth, min(pa.max(), pb.max()) - max(pa.min(), pb.min()))
    return float(depth)


def _delivered(state) -> bool:
    """At rest, upright, lowest point within 3 mm of the tray floor, centre inside the 90 x 90 mm floor."""
    x, y = float(state.pos[0]), float(state.pos[1])
    inside = abs(x - TRAY[0]) <= FLOOR[0] / 2 and abs(y - TRAY[1]) <= FLOOR[1] / 2
    return bool(state.resting() and inside and _tilt(state) <= TILT_TOL and abs(_bottom(state) - FLOOR_TOP) <= SEAT_TOL)


def _on_tray(state) -> bool:
    """Any part of the item over the tray's 102 x 102 mm outer outline by 2 mm or more, seen from above, at any height."""
    return _overlap(_outline(state), TRAY_POLY) >= OVER_TOL


def _in_place(state, mark) -> bool:
    """Standing upright on the table with its centre within 20 mm of its mark."""
    moved = math.hypot(float(state.pos[0]) - mark[0], float(state.pos[1]) - mark[1])
    return bool(moved <= MOVE_TOL and _tilt(state) <= TILT_TOL and abs(_bottom(state) - TOP) <= TABLE_TOL)


def _level(objects) -> float:
    """1.0 green delivered, nothing else on the tray, nothing else disturbed; 0.5 the same with one or more of the
    others disturbed; 0 otherwise."""
    if not _delivered(objects[GREEN]) or any(_on_tray(objects[n]) for n in OTHERS):
        return 0.0
    return 1.0 if all(_in_place(objects[n], STARTS[n]) for n in OTHERS) else 0.5


def success(objects) -> bool:
    """The green item upright on the tray, the other three upright on their marks, at rest; holds for 1 s."""
    return _level(objects) == 1.0


def distance(objects) -> float:
    """Metres from the green item's centre to the tray centre, horizontally."""
    g = objects[GREEN]
    return math.hypot(float(g.pos[0]) - TRAY[0], float(g.pos[1]) - TRAY[1])


def score(objects) -> float:
    """1.0 for the green item delivered with the red, blue and yellow items still upright within 20 mm of their marks
    and none of them on the tray; 0.5 for the green item delivered, none of the others on the tray, but one or more
    of them moved, tipped or lifted; 0 for everything else, including any other item on or partly on the tray."""
    return _level(objects)


TRAVEL = TOP + 0.145
GRIP_H = 0.040  # the TCP 40 mm above an item's bottom: pad tips 27 mm up the 60 mm side
PICK_Z = TOP + GRIP_H
PLACE_Z = FLOOR_TOP + 0.005 + GRIP_H  # item bottom 5 mm above the tray floor; pad tips 23 mm above the rims
ENTRY = 0.020  # the pause above the tray: item bottom 16 mm above the 15 mm rims
GRIP_YAW = POINT - math.pi / 2  # +60 degrees: the pads close along the pointing line, the neighbours sit beside them
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part


def _pick_place(pick, place, *, width, entry, travel_z, yaw=0.0, place_yaw=None):
    """A top-down pick and place paced for the simulated arm, whose soft wrist trails a moving target by several
    millimetres. The arm settles before every close and release, stops ``entry`` above the place with the part
    clear of every fixture, and makes the last descent slowly. The fingers close to 8 mm narrower than the part's
    ``width`` across the pads rather than fully shut: each finger then pushes 4 mm x 5000 N/m = 20 N, under its
    50 N cap, so the grip centres the part, whereas a fully commanded close saturates both fingers and lets the part
    slide onto one of them. The gripper opens and closes over 1.5 s, inside the default 0.05 per step limiter.
    16.2 s."""
    (px, py, pz), (qx, qy, qz) = pick, place
    end = yaw if place_yaw is None else place_yaw
    grip = -(width - SQUEEZE) / 2  # finger joint target, m
    return [
        Waypoint(1.5, (px, py, travel_z), FINGER_OPEN, "to the object", yaw),
        Waypoint(1.5, (px, py, pz), FINGER_OPEN, "descend", yaw),
        Waypoint(0.6, (px, py, pz), FINGER_OPEN, "settle", yaw),
        Waypoint(1.5, (px, py, pz), grip, "close", yaw),
        Waypoint(1.2, (px, py, travel_z), grip, "lift", yaw),
        Waypoint(2.0, (qx, qy, travel_z), grip, "carry", end),
        Waypoint(0.6, (qx, qy, travel_z), grip, "settle over the place", end),
        Waypoint(1.2, (qx, qy, qz + entry), grip, "lower to the entry", end),
        Waypoint(0.8, (qx, qy, qz + entry), grip, "settle at the entry", end),
        Waypoint(2.0, (qx, qy, qz), grip, "set in", end),
        Waypoint(0.8, (qx, qy, qz), grip, "settle", end),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, "release", end),
        Waypoint(1.0, (qx, qy, travel_z), FINGER_OPEN, "retreat", end),
    ]


def demo():
    """Left arm only. The green item from its mark to the tray centre, the pads closing across its 40 mm diameter
    along the pointing line, a carry at 14.5 cm and a 5 mm drop onto the tray floor. 16.2 s of plan, then the 3 s
    hold: 384 steps at 20 Hz."""
    gx, gy = STARTS[GREEN]
    return {"left": _pick_place((gx, gy, PICK_Z), (*TRAY, PLACE_Z), width=2 * ITEM_R, entry=ENTRY, travel_z=TRAVEL, yaw=GRIP_YAW)}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(tilt=0.0, about=0.0):  # tilt about the horizontal axis at angle ``about`` from world x
        s = math.sin(tilt / 2)
        return np.array([-math.sin(about) * s, math.cos(about) * s, 0.0, math.cos(tilt / 2)])

    def item(x, y, bottom=TOP, tilt=0.0, about=0.0, vel=(0.0, 0.0, 0.0)):
        q = quat(tilt, about)
        uz = abs(float(_rot(q)[2, 2]))
        half_z = ITEM_H / 2 * uz + ITEM_R * math.sqrt(max(0.0, 1.0 - uz * uz))
        pos = np.array([x, y, bottom + half_z])
        half = np.array([ITEM_R + ITEM_H / 2, ITEM_R + ITEM_H / 2, half_z])  # loose bounds; the scorer does not read them
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {n: item(x, y) for n, (x, y) in STARTS.items()}

    def tray(dx=0.0, dy=0.0, bottom=FLOOR_TOP, **kw):
        return item(TRAY[0] + dx, TRAY[1] + dy, bottom=bottom, **kw)

    def scene(**moved):
        return {**start, **{f"item_{k}": st for k, st in moved.items()}}

    # An item leaning 30 degrees in over the rim, its lower edge on the table outside: seen from above, its outline
    # reaches 10 mm inside the tray's outer outline.
    lean = math.radians(30.0)
    lean_y = TRAY[1] + TRAY_HALF - 0.010 + ITEM_H / 2 * math.sin(lean) + ITEM_R * math.cos(lean)
    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene(green=tray()), 1.0, True),
        "done, green upside down": (scene(green=tray(tilt=math.pi)), 1.0, True),
        "done, green 20 mm off the tray centre in x and in y (28 mm)": (scene(green=tray(dx=0.020, dy=-0.020)), 1.0, True),
        "done, green tilted 8 degrees": (scene(green=tray(tilt=math.radians(8))), 1.0, True),
        "green tilted 12 degrees on the tray": (scene(green=tray(tilt=math.radians(12))), 0.0, False),
        "green lying on its side on the tray": (scene(green=tray(tilt=math.pi / 2)), 0.0, False),
        "green held still 10 mm above the tray floor": (scene(green=tray(bottom=FLOOR_TOP + 0.010)), 0.0, False),
        "green still moving on the tray": (scene(green=tray(vel=(0.02, 0.0, 0.0))), 0.0, False),
        "green standing on the table beside the tray": (scene(green=item(TRAY[0] + 0.080, TRAY[1])), 0.0, False),
        "green on the rim top": (scene(green=item(TRAY[0] + RIM_OFF, TRAY[1], bottom=TOP + RIM_H)), 0.0, False),
        "green untouched, red on the tray (nearest the fingertip, in front of the left arm)": (scene(red=tray()), 0.0, False),
        "green untouched, blue on the tray (nearest the left base, farthest from the crewmate)": (scene(blue=tray()), 0.0, False),
        "green untouched, yellow on the tray (nearest the tray)": (scene(yellow=tray()), 0.0, False),
        "all four on the tray": (scene(green=tray(-0.0225, -0.0225), red=tray(0.0225, -0.0225), blue=tray(-0.0225, 0.0225), yellow=tray(0.0225, 0.0225)), 0.0, False),
        "green and yellow on the tray": (scene(green=tray(-0.0225), yellow=tray(0.0225)), 0.0, False),
        "green delivered, red stacked on it": (scene(green=tray(), red=tray(bottom=FLOOR_TOP + ITEM_H)), 0.0, False),
        "green delivered, red held 50 mm over the tray": (scene(green=tray(-0.022), red=tray(0.022, bottom=FLOOR_TOP + 0.05, vel=(0, 0, 0.02))), 0.0, False),
        "green delivered, yellow leaning on the rim": (scene(green=tray(), yellow=item(TRAY[0], lean_y, tilt=lean, about=-math.pi / 2)), 0.0, False),
        "green delivered, yellow standing against the rim outside": (scene(green=tray(), yellow=item(TRAY[0], TRAY[1] + TRAY_HALF + ITEM_R)), 0.5, False),
        "green delivered, yellow slid 19 mm toward the tray": (scene(green=tray(), yellow=item(STARTS["item_yellow"][0], STARTS["item_yellow"][1] - 0.019)), 1.0, True),
        "green delivered, red knocked over": (scene(green=tray(), red=item(*STARTS["item_red"], tilt=math.pi / 2)), 0.5, False),
        "green delivered, blue slid 25 mm": (scene(green=tray(), blue=item(STARTS["item_blue"][0] + 0.025, STARTS["item_blue"][1])), 0.5, False),
        "green delivered, blue slid 19 mm": (scene(green=tray(), blue=item(STARTS["item_blue"][0] + 0.019, STARTS["item_blue"][1])), 1.0, True),
        "green delivered, red lifted 50 mm over its mark": (scene(green=tray(), red=item(*STARTS["item_red"], bottom=TOP + 0.05)), 0.5, False),
        "green delivered, blue upside down on its mark": (scene(green=tray(), blue=item(*STARTS["item_blue"], tilt=math.pi)), 1.0, True),
        "green delivered, red tipped 12 degrees on its mark": (scene(green=tray(), red=item(*STARTS["item_red"], tilt=math.radians(12))), 0.5, False),
        "green delivered, all three others knocked over": (scene(green=tray(), red=item(*STARTS["item_red"], tilt=math.pi / 2), blue=item(*STARTS["item_blue"], tilt=math.pi / 2), yellow=item(*STARTS["item_yellow"], tilt=math.pi / 2)), 0.5, False),
        "green lifted and held over its mark": (scene(green=item(*STARTS[GREEN], bottom=TOP + 0.08)), 0.0, False),
        "green knocked over on the table": (scene(green=item(*STARTS[GREEN], tilt=math.pi / 2)), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    # The layout claims the scoring text makes.
    tip = np.array(TIP)
    dist = {n: float(np.hypot(*(np.array(p) - tip))) for n, p in STARTS.items()}
    assert min(dist, key=dist.get) == "item_red" and dist["item_red"] < dist[GREEN] - 0.04
    base = (0.0, 0.305)
    to_base = {n: math.hypot(p[0] - base[0], p[1] - base[1]) for n, p in STARTS.items()}
    assert min(to_base, key=to_base.get) == "item_blue" and to_base["item_blue"] < to_base[GREEN] - 0.015
    gap = {n: float(np.hypot(*(np.array(p) - np.clip(p, TRAY_POLY.min(0), TRAY_POLY.max(0))))) for n, p in STARTS.items()}
    assert min(gap, key=gap.get) == "item_yellow"
    lane = {n: abs(p[1] - base[1]) for n, p in STARTS.items()}  # straight in front of the left arm
    assert min(lane, key=lane.get) == "item_red" and lane["item_red"] < lane[GREEN] - 0.010
    for crew in (TORSO_XY, (TORSO_XY[0] - TORSO[0] / 2, TORSO_XY[1])):  # farthest from the torso and head, or its front face
        away = {n: math.hypot(p[0] - crew[0], p[1] - crew[1]) for n, p in STARTS.items()}
        assert max(away, key=away.get) == "item_blue" and away["item_blue"] > away[GREEN] + 0.010
    for n, p in STARTS.items():
        rel = np.array(p) - tip
        cross = DIR[0] * rel[1] - DIR[1] * rel[0]
        off = math.degrees(abs(math.atan2(cross, DIR[0] * rel[0] + DIR[1] * rel[1])))
        assert (off < 0.1) if n == GREEN else (off > 29.0 and abs(cross) >= 0.054), (n, off, cross)
    assert abs(STARTS["item_blue"][1] - TIP[1]) < 1e-9  # straight toward the arms from the fingertip, along -x
    marks = list(STARTS.values())
    assert min(math.hypot(a[0] - b[0], a[1] - b[1]) for i, a in enumerate(marks) for b in marks[i + 1 :]) >= 0.100
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
