"""On the station table, in front of the left arm, an empty 5 x 5 freezer box: a fixed square grid of thin white
dividers, 2 cm tall, making 25 cells on the table. Nearer the arms on the right, a bench rack holds four sample
aliquot tubes, upright, in a row: two thin and two thick, each either tall or short (thin-tall, thin-short,
thick-tall, thick-short). Nearer the arms on the left lies the sample map, a white card printed as a half-size copy of
the box's grid in the same orientation, with four pegs standing on it. Each peg stands in the cell where one tube is
assigned and is a half-size model of that tube (thin or thick, tall or short). Storage rule: each aliquot goes in the
cell the map assigns it, so it can be found later from its box position; a tube in another cell is lost to the
inventory. Scored from the tube poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/freezer-box-by-map/freezer_box_by_map.py \\
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
THIN_R, THICK_R = 0.008, 0.011
TALL_H, SHORT_H = 0.070, 0.045
TUBE_KG = 0.015
TUBES = {  # name: (radius, height)
    "tube_thin_tall": (THIN_R, TALL_H),
    "tube_thin_short": (THIN_R, SHORT_H),
    "tube_thick_tall": (THICK_R, TALL_H),
    "tube_thick_short": (THICK_R, SHORT_H),
}

BOX_XY = (0.330, 0.300)
N = 5
PITCH = 0.032
DIV_T, DIV_H = 0.002, 0.020
CELL_IN = PITCH - DIV_T  # 30 mm square cells


def cell_xy(i, j, centre=BOX_XY, pitch=PITCH):
    """Cell (i, j): i counts rows away from the arms (0 nearest), j counts columns from the right (0 rightmost)."""
    return (centre[0] + (i - (N - 1) / 2) * pitch, centre[1] + (j - (N - 1) / 2) * pitch)


MAP = {  # the cell the sample map assigns each tube
    "tube_thin_tall": (0, 1),
    "tube_thin_short": (1, 3),
    "tube_thick_tall": (3, 0),
    "tube_thick_short": (4, 3),
}

RACK_Y = 0.150
RACK_X = {"tube_thick_tall": 0.160, "tube_thin_tall": 0.200, "tube_thick_short": 0.240, "tube_thin_short": 0.280}
RACK_PITCH, RACK_H = 0.040, 0.015

CARD_XY = (0.170, 0.430)
CARD_PITCH = PITCH / 2
CARD = (N * CARD_PITCH + 0.012, N * CARD_PITCH + 0.012, 0.003)
CARD_TOP = TOP + CARD[2]
LINE = 0.0012  # printed grid line width

WHITE = (0.93, 0.93, 0.92)
GRID_C = (0.88, 0.88, 0.86)
INK = (0.15, 0.15, 0.18)
BLUE = (0.20, 0.35, 0.75)
TUBE_C = (0.86, 0.84, 0.70)
RACK_C = (0.30, 0.55, 0.80)


def _grid(name, centre, pitch, n_x, n_y, height, base_z, thick, color):
    """Divider lines of an n_x by n_y grid: walls along y at the x cell boundaries, and walls along x at the y ones."""
    cx, cy = centre
    lx, ly = n_x * pitch + thick, n_y * pitch + thick
    z = base_z + height / 2
    out = []
    for k in range(n_x + 1):
        x = cx + (k - n_x / 2) * pitch
        out.append(Box(f"{name}_x{k}", pos=(x, cy, z), size=(thick, ly, height), color=color))
    for k in range(n_y + 1):
        y = cy + (k - n_y / 2) * pitch
        out.append(Box(f"{name}_y{k}", pos=(cx, y, z), size=(lx, thick, height), color=color))
    return out


TUBE_OBJS = [Cylinder(n, pos=(RACK_X[n], RACK_Y, TOP + h / 2), radius=r, height=h, mass=TUBE_KG, color=TUBE_C)
             for n, (r, h) in TUBES.items()]
PEGS = [Cylinder(f"map_peg_{n[5:]}", pos=(*cell_xy(*MAP[n], CARD_XY, CARD_PITCH), CARD_TOP + h / 4), radius=r / 2,
                 height=h / 2, color=BLUE) for n, (r, h) in TUBES.items()]
RACK_CENTRE = (sum(RACK_X.values()) / 4, RACK_Y)
FIXTURES = [
    *_grid("box", BOX_XY, PITCH, N, N, DIV_H, TOP, DIV_T, WHITE),
    *_grid("bench_rack", RACK_CENTRE, RACK_PITCH, 4, 1, RACK_H, TOP, 0.004, RACK_C),
    Box("map_card", pos=(*CARD_XY, TOP + CARD[2] / 2), size=CARD, color=GRID_C),
    *_grid("map_line", CARD_XY, CARD_PITCH, N, N, 0.0006, CARD_TOP, LINE, INK),
    *PEGS,
]
OBJECTS = [*TUBE_OBJS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _tilt(quat_xyzw) -> float:
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def in_cell(st, cell) -> bool:
    """At rest, upright (under 10 degrees), the centre inside the cell's dividers and the bottom on the table."""
    cx, cy = cell_xy(*cell)
    return bool(
        abs(float(st.pos[0]) - cx) < CELL_IN / 2
        and abs(float(st.pos[1]) - cy) < CELL_IN / 2
        and float(st.low[2]) < TOP + 0.005
        and _tilt(st.quat_xyzw) < 10.0
        and st.resting()
    )


def stored(objects) -> int:
    return sum(in_cell(objects[n], MAP[n]) for n in TUBES)


def score(objects) -> float:
    """0.25 for each tube standing upright, at rest, in the cell the sample map assigns it. A tube in any other cell
    earns nothing."""
    return float(0.25 * stored(objects))


def success(objects) -> bool:
    """All four tubes upright in their mapped cells; holds for 1 s."""
    return stored(objects) == len(TUBES)


def distance(objects) -> float:
    """Metres to go: each tube's centre to its mapped cell's centre (in the plane)."""
    d = 0.0
    for n in TUBES:
        st, (cx, cy) = objects[n], cell_xy(*MAP[n])
        d += math.hypot(float(st.pos[0]) - cx, float(st.pos[1]) - cy)
    return float(d)


# -- the scripted demo ------------------------------------------------------------------------------------------

TRAVEL = TOP + 0.140  # a carried tube's bottom 10 cm up: clear of the dividers and the tall tubes already stored
SQUEEZE = 0.008
CLEAR = 0.020
TIP = 0.013
TIP_Z = 0.024  # the pad tips 24 mm above the tube's bottom: over the 2 cm dividers at the place
X_TRIM = 0.0015  # light parts land about 1.5 mm short in x


def _move(name, label):
    r, _ = TUBES[name]
    px, py = RACK_X[name], RACK_Y
    qx, qy = cell_xy(*MAP[name])
    qx += X_TRIM
    gz = TOP + TIP_Z + TIP
    qz = gz + 0.002  # let go 2 mm above the floor
    open_ = -(2 * r + CLEAR) / 2
    grip = -(2 * r - SQUEEZE) / 2
    return [
        Waypoint(1.5, (px, py, TRAVEL), open_, f"{label}: over"),
        Waypoint(1.2, (px, py, gz + 0.015), open_, f"{label}: descend"),
        Waypoint(0.8, (px, py, gz), open_, f"{label}: down slowly"),
        Waypoint(0.5, (px, py, gz), open_, f"{label}: settle"),
        Waypoint(1.2, (px, py, gz), grip, f"{label}: close"),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{label}: lift"),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{label}: carry"),
        Waypoint(0.5, (qx, qy, TRAVEL), grip, f"{label}: settle over the cell"),
        Waypoint(1.5, (qx, qy, qz + 0.030), grip, f"{label}: lower"),
        Waypoint(1.2, (qx, qy, qz), grip, f"{label}: into the cell slowly"),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle"),
        Waypoint(1.2, (qx, qy, qz), open_, f"{label}: release"),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{label}: retreat"),
    ]


ORDER = ("tube_thick_tall", "tube_thin_tall", "tube_thick_short", "tube_thin_short")  # rack order, nearest first


def demo():
    """Left arm only: each tube from the bench rack, nearest first, into its mapped cell. 57.6 s."""
    plan = []
    for n in ORDER:
        plan += _move(n, n[5:].replace("_", "-"))
    return {"left": plan}


def self_test(ObjectState=None):
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def tube(name, x, y, bottom=TOP, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        r, h = TUBES[name]
        t = math.radians(tilt_deg)
        q = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
        half = np.array([r, r, h / 2])
        pos = np.array([x, y, bottom + h / 2])
        return {name: ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)}

    start = {}
    for n in TUBES:
        start |= tube(n, RACK_X[n], RACK_Y)

    def put(name, cell, **kw):
        return tube(name, *cell_xy(*cell), **kw)

    done = dict(start)
    for n in TUBES:
        done |= put(n, MAP[n])
    cases = {
        "start": (start, 0.0, False),
        "one stored": (start | put("tube_thin_tall", MAP["tube_thin_tall"]), 0.25, False),
        "three stored": (done | tube("tube_thin_short", RACK_X["tube_thin_short"], RACK_Y), 0.75, False),
        "done": (done, 1.0, True),
        "two tall tubes swapped": (done | put("tube_thin_tall", MAP["tube_thick_tall"])
                                   | put("tube_thick_tall", MAP["tube_thin_tall"]), 0.5, False),
        "thin pair swapped": (done | put("tube_thin_tall", MAP["tube_thin_short"])
                              | put("tube_thin_short", MAP["tube_thin_tall"]), 0.5, False),
        "map read mirrored (columns from the left)": (
            {k: v for n in TUBES for k, v in put(n, (MAP[n][0], N - 1 - MAP[n][1])).items()}, 0.0, False),
        "one cell off": (done | put("tube_thick_short", (4, 2)), 0.75, False),
        "tube fallen over in its cell": (done | put("tube_thin_tall", MAP["tube_thin_tall"], tilt_deg=60.0), 0.75, False),
        "tube still dropping": (done | put("tube_thin_short", MAP["tube_thin_short"], vel=(0, 0, -0.2)), 0.75, False),
        "tube resting on a divider": (done | put("tube_thick_tall", MAP["tube_thick_tall"], bottom=TOP + DIV_H), 0.75,
                                      False),
        "all four on the map card": ({k: v for n in TUBES for k, v in
                                      tube(n, *cell_xy(*MAP[n], CARD_XY, CARD_PITCH), bottom=CARD_TOP).items()}, 0.0,
                                     False),
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
