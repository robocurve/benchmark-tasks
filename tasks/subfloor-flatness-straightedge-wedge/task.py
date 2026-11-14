"""Check a subfloor for large-format tile: straightedge across it, measuring wedge in the deepest gap, PASS or FAIL tag.

On the station table, in front of the arms: a 64 cm by 23 cm section of 3/4 in (1.9 cm) subfloor with two pencil lines
6 cm apart across it, which mark where the check is made. A sag between two joists runs along the joists, so it
crosses the check line as a trough: the trial's case sets how deep it is (0.5 mm steps) and where it is along the line.
Nobody can see a 1-3 mm trough by eye; it shows only under a straightedge.

A 24 in (61.0 cm) aluminum box-beam level, used on its milled bottom edge as the straightedge, stands on its edge in
two rests in front of the panel; it weighs 0.50 kg and is too long to lift level by one end, so both hands carry it. A
builder's aluminum measuring wedge (1:10 slope, 15 cm long, 3.0 cm wide, 0.3 mm thick at the tip, 15.3 mm at the heel,
a mark every 0.5 mm of reading, longer every 1 mm) lies on the table right of the panel. A report board left of the
panel has two pockets, PASS and FAIL, and a 43 g tag puck standing on a ring between them.

The job: lay the straightedge across the panel between the lines, slide the wedge under its edge in the deepest gap
and leave it there, and put the puck in PASS if that gap is within ANSI A108.02's 1/16 in (1.6 mm) in 24 in for tile
with an edge 15 in or longer, or in FAIL if it is not.

Everything is a rigid body with its real size and mass. The panel is fixed: 32 strips, each 2 cm wide along the line,
whose tops are the floor (a dipped strip's top is lower). The reading is what the wedge's scale shows where the
straightedge's near bottom edge crosses it, computed from the wedge's and the straightedge's poses: 0.3 mm + 0.1 x the
length of wedge under the edge. Every threshold below appears once, with the same value in task.json and the README.
"""
from __future__ import annotations

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import yamkit as kit  # noqa: E402
import guardkit as gk  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400  # 2 minutes at 20 Hz
INSTRUCTION = ("Check whether this subfloor is flat enough for large-format tile, along the strip between the two pencil "
               "lines. Leave the measuring wedge in the deepest gap under the straightedge, and put the tag puck in "
               "PASS or FAIL.")

# -- the trial's case: the trough's depth (mm) and the y of its deepest strip's center (cm) ---------------------------
# One real board per case. Unless FLATNESS_CASE=1..8 fixes it, each run draws its board at random (all eight equally
# likely), so no fixed replay can skip the check. The scripted demo is open loop for the case FLATNESS_CASE fixes.
CASES = {1: (1.0, -13), 2: (1.5, 7), 3: (2.0, -5), 4: (2.5, 15), 5: (3.0, -3), 6: (1.5, -9), 7: (2.0, 11), 8: (1.0, 3)}
LIMIT_MM = 1.6  # ANSI A108.02: 1/16 in (1.6 mm) in 24 in, measured from the high points (edge >= 15 in)


def case_number() -> int:
    """FLATNESS_CASE if it names a case, else a random draw (OS entropy, independent of anything the policy sees)."""
    try:
        n = int(os.environ.get("FLATNESS_CASE", ""))
    except ValueError:
        n = 0
    if n in CASES:
        return n
    return sorted(CASES)[random.SystemRandom().randrange(len(CASES))]


CASE = case_number()
print(f"subfloor-flatness: board case {CASE}", file=sys.stderr)  # the run log records which board was drawn
DIP_MM, DIP_Y_CM = CASES[CASE]
VERDICT = "PASS" if DIP_MM <= LIMIT_MM else "FAIL"  # the correct report for this board

# -- the subfloor panel (fixed) ---------------------------------------------------------------------------------------
PANEL_X = (0.220, 0.450)  # 23 cm deep (x): the wedge lies wholly on it, tip under the straightedge
PANEL_Y = (-0.320, 0.320)  # 64 cm long (y), along the check line
PANEL_T = 0.019  # 3/4 in
FLOOR = TOP + PANEL_T  # the panel's flat top
STRIP_W = 0.020  # the dip's resolution: 2 cm strips along the line
N_STRIPS = 32
STEP_MM = 0.5  # one terrace (one 0.5 mm styrene sheet in the real board)
CORE_STRIPS = 3  # the deepest 6 cm: the center strip and one each side
LANE_X = (0.320, 0.380)  # the two pencil lines, 6 cm apart; the straightedge's bottom must lie between them


def strip_centers() -> np.ndarray:
    return PANEL_Y[0] + STRIP_W * (np.arange(N_STRIPS) + 0.5)


def strip_depths_mm(dip_mm: float = DIP_MM, dip_y_cm: float = DIP_Y_CM) -> np.ndarray:
    """Depth below the flat top of each strip: dip_mm on the 3 core strips, 0.5 mm less per strip outward, 0 beyond."""
    j = np.abs(np.round((strip_centers() - dip_y_cm / 100.0) / STRIP_W))
    return np.maximum(0.0, dip_mm - STEP_MM * np.maximum(0.0, j - (CORE_STRIPS - 1) / 2))


DEPTHS_MM = strip_depths_mm()


def floor_z(y) -> np.ndarray:
    """World z of the floor at world y (the strip's top); -inf off the panel's length."""
    y = np.asarray(y, float)
    i = np.floor((y - PANEL_Y[0]) / STRIP_W).astype(int)
    ok = (i >= 0) & (i < N_STRIPS)
    z = FLOOR - DEPTHS_MM[np.clip(i, 0, N_STRIPS - 1)] / 1000.0
    return np.where(ok, z, -np.inf)


def floor_max(y0: float, y1: float) -> float:
    """The highest floor between world y0 and y1 (what a flat-bottomed part spanning them rests on)."""
    ys = np.linspace(min(y0, y1), max(y0, y1), 13)
    return float(np.max(floor_z(ys)))


# -- the straightedge: a 24 in aluminum box-beam level on its edge (object frame: center; y along it, z up) ------------
SE_L, SE_T, SE_H = 0.610, 0.025, 0.063  # 24 in long, 1 in thick, 2.5 in tall
SE_MASS = 0.50
REST_X, REST_Y = 0.190, (-0.200, 0.200)  # two fixed rests in front of the panel
REST_H, REST_SLOT, REST_CHEEK = 0.030, 0.027, 0.015  # rest top 3 cm up; slot 2.7 cm wide, cheeks 1.5 cm tall
SE_START = (REST_X, 0.0, TOP + REST_H + SE_H / 2)

# -- the measuring wedge (object frame: origin on the bottom face's center; tip toward +x) -----------------------------
W_L, W_W = 0.150, 0.030  # 15 cm long, 3.0 cm wide
W_TIP, W_SLOPE = 0.0003, 0.10  # 0.3 mm at the tip, rising 1 mm per 10 mm (1:10): 15.3 mm at the heel
W_MASS = 0.095  # aluminum, 6061: 35 cm^3
TIP_X = W_L / 2  # the tip's x in the wedge frame
W_START = (0.270, -0.390, TOP)  # lying flat on the table right of the panel, tip pointing away from the robot


def wedge_thickness(s):
    """Thickness (m) at distance s (m) back from the tip: what the scale reads there."""
    return W_TIP + W_SLOPE * np.asarray(s, float)


# -- the report board (fixed) and the tag puck ------------------------------------------------------------------------
BOARD_C = (0.270, 0.390)
BOARD_T = 0.005  # the plate under the pockets
POCKET_IN, POCKET_WALL, POCKET_H = 0.050, 0.005, 0.010  # 5.0 cm square inside, 5 mm walls, 1 cm deep
POCKETS = {"PASS": (0.325, 0.380), "FAIL": (0.215, 0.380)}  # pocket centers (x, y): PASS far, FAIL near
RING_C = (0.270, 0.380)  # the puck's start ring, between the pockets
PUCK_R, PUCK_H, PUCK_MASS = 0.016, 0.020, 0.043  # 3.2 cm x 2.0 cm white-anodized aluminum, 43 g
PUCK_START = (RING_C[0], RING_C[1], TOP + BOARD_T)

# -- thresholds (each with its reason in the README and task.json) ---------------------------------------------------
CONTACT = 0.0005  # m: two rigid faces within 0.5 mm count as touching (a 0.5 mm feeler is the real check)
SINK = 0.0015  # m: overlap the solver may leave between resting faces; more is not a real pose
END_ZONE = 0.050  # m: each end's last 5 cm must touch the floor (the straightedge rests on the high points)
SE_TILT = math.radians(5.0)  # standing on its edge: its height axis within 5 degrees of vertical
IN_GAP_MIN = 0.002  # m: at least 2 mm of the wedge under the edge (a tip just touching the face measures nothing)
WEDGE_TILT = math.radians(3.0)  # lying flat on the floor
WEDGE_YAW = math.radians(20.0)  # pointing under the straightedge, within 20 degrees of square to it
READ_TOL_MM = 0.3  # the reading within 0.3 mm of the trough's depth: a strip 0.5 mm shallower is off by 0.5 mm
ON_DEEPEST_MM = 0.1  # and the wedge lying on the deepest 6 cm (its 3 cm width all on it): an end lifted inside the
#                      0.5 mm contact allowance can add up to about 0.4 mm at the wedge, which would otherwise let a
#                      wedge forced in on the next strip out read within 0.3 mm
STILL_SPEED, STILL_SPIN = 0.005, math.radians(6.0)  # at rest: under 5 mm/s and 6 degrees/s
PUCK_TILT = math.radians(15.0)
HELD_CAP = 0.5  # a hand still on the straightedge, wedge or puck at the end caps the score at 0.5
WRONG_CAP = 0.5  # the puck in the wrong pocket caps the score at 0.5
GUESS_CAP = 0.5  # the puck in a pocket before the wedge ever measured a gap caps the score at 0.5 (latched)
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)  # the final state holds 1 s (20 steps)
GRIPPERS = ("left_gripper", "right_gripper")

# -- the scene -------------------------------------------------------------------------------------------------------
PLY = kit.Look((0.80, 0.66, 0.45), 0.8)
PENCIL = kit.Look((0.25, 0.25, 0.27), 0.6)
ALU = kit.Look((0.78, 0.79, 0.80), 0.35, metallic=0.8)
BLACK = kit.Look((0.08, 0.08, 0.08), 0.6)
VIAL = kit.Look((0.55, 0.85, 0.35), 0.1, opacity=0.8)
WEDGE_LOOK = kit.Look((0.85, 0.86, 0.87), 0.3, metallic=0.7)
MARK = kit.Look((0.05, 0.05, 0.05), 0.5)
BOARD_LOOK = kit.Look((0.92, 0.92, 0.90), 0.6)
GREEN = kit.Look((0.15, 0.65, 0.25), 0.5)
RED = kit.Look((0.80, 0.12, 0.10), 0.5)
WHITE = kit.Look((0.96, 0.96, 0.96), 0.5)
RUBBER = kit.Look((0.20, 0.20, 0.22), 0.9)

_pcx = (PANEL_X[0] + PANEL_X[1]) / 2
_pw = PANEL_X[1] - PANEL_X[0]
_strips = []
for _y, _d in zip(strip_centers(), DEPTHS_MM):
    _h = PANEL_T - _d / 1000.0
    _strips.append(((0.0, float(_y), _h / 2), (_pw, STRIP_W, _h)))
_lines = [kit.box((lx - _pcx, float(_y), PANEL_T - _d / 1000.0 + 0.0001), (0.001, STRIP_W, 0.0002), PENCIL)
          for lx in LANE_X for _y, _d in zip(strip_centers(), DEPTHS_MM)]
PANEL = kit.Prop("subfloor", pos=(_pcx, 0.0, TOP), mass=None, size=(_pw, PANEL_Y[1] - PANEL_Y[0], PANEL_T),
                 meshes=tuple(kit.box(c, s, PLY) for c, s in _strips) + tuple(_lines),
                 colliders=tuple(kit.Collider(c, size=s) for c, s in _strips))

_se_mesh = (kit.box((0, 0, 0), (SE_T, SE_L - 0.020, SE_H), ALU),
            kit.box((0, -SE_L / 2 + 0.005, 0), (SE_T + 0.001, 0.010, SE_H + 0.001), BLACK),
            kit.box((0, SE_L / 2 - 0.005, 0), (SE_T + 0.001, 0.010, SE_H + 0.001), BLACK),
            kit.box((-SE_T / 2 - 0.0005, 0.0, 0.010), (0.001, 0.040, 0.015), VIAL),
            kit.box((-SE_T / 2 - 0.0005, -0.200, 0.010), (0.001, 0.040, 0.015), VIAL))
STRAIGHTEDGE = kit.Prop("straightedge", pos=SE_START, mass=SE_MASS, size=(SE_T, SE_L, SE_H), meshes=_se_mesh,
                        colliders=(kit.Collider((0, 0, 0), size=(SE_T, SE_L, SE_H)),))
SE_PARTS = (gk.Block((0, 0, 0), (SE_T, SE_L, SE_H)),)


def _rest(name, y):
    parts = [((0, 0, REST_H / 2), (0.050, 0.040, REST_H)),
             ((-(REST_SLOT / 2 + 0.006), 0, REST_H + REST_CHEEK / 2), (0.012, 0.040, REST_CHEEK)),
             ((REST_SLOT / 2 + 0.006, 0, REST_H + REST_CHEEK / 2), (0.012, 0.040, REST_CHEEK))]
    return kit.Prop(name, pos=(REST_X, y, TOP), mass=None, size=(0.050, 0.040, REST_H + REST_CHEEK),
                    meshes=tuple(kit.box(c, s, RUBBER) for c, s in parts),
                    colliders=tuple(kit.Collider(c, size=s) for c, s in parts))


# the wedge: a prism, 8 corners (bottom tip right/left, bottom heel right/left, then the same on top)
W_HULL = ((TIP_X, -W_W / 2, 0.0), (TIP_X, W_W / 2, 0.0), (-TIP_X, -W_W / 2, 0.0), (-TIP_X, W_W / 2, 0.0),
          (TIP_X, -W_W / 2, W_TIP), (TIP_X, W_W / 2, W_TIP),
          (-TIP_X, -W_W / 2, float(wedge_thickness(W_L))), (-TIP_X, W_W / 2, float(wedge_thickness(W_L))))
W_QUADS = ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5))
W_TRIS = tuple(t for a, b, c, d in W_QUADS for t in ((a, b, c), (a, c, d)))
_marks = []
for _k in range(1, 31):  # a mark every 0.5 mm of reading (5 mm apart), longer every 1 mm, full width every 5 mm
    _r = _k * 0.0005
    _s = (_r - W_TIP) / W_SLOPE
    _w = W_W if _k % 10 == 0 else (W_W / 2 if _k % 2 == 0 else W_W / 4)
    _marks.append(kit.box((TIP_X - _s, -W_W / 2 + _w / 2, float(wedge_thickness(_s)) + 0.0001),
                          (0.0006, _w, 0.0003), MARK))
WEDGE = kit.Prop("wedge", pos=W_START, mass=W_MASS, size=(W_L, W_W, float(wedge_thickness(W_L))),
                 meshes=(kit._mesh(W_HULL, [4] * 6, [i for q in W_QUADS for i in q], WEDGE_LOOK),) + tuple(_marks),
                 colliders=(kit.Collider((0, 0, 0), hull=W_HULL, faces=W_TRIS),))
# for "a hand on the wedge": three steps that bound its sloped top
W_PARTS = tuple(gk.Block((TIP_X - (k + 0.5) * W_L / 3, 0.0, float(wedge_thickness((k + 1) * W_L / 3)) / 2),
                         (W_L / 3, W_W, float(wedge_thickness((k + 1) * W_L / 3)))) for k in range(3))


def _board():
    bx, by = BOARD_C
    parts, looks = [((0, 0, BOARD_T / 2), (0.180, 0.080, BOARD_T))], [BOARD_LOOK]
    for name, (px, py) in POCKETS.items():
        cx, cy = px - bx, py - by
        o = POCKET_IN / 2 + POCKET_WALL / 2
        for dx, dy, sx, sy in ((-o, 0, POCKET_WALL, POCKET_IN + 2 * POCKET_WALL),
                               (o, 0, POCKET_WALL, POCKET_IN + 2 * POCKET_WALL),
                               (0, -o, POCKET_IN, POCKET_WALL), (0, o, POCKET_IN, POCKET_WALL)):
            parts.append(((cx + dx, cy + dy, BOARD_T + POCKET_H / 2), (sx, sy, POCKET_H)))
            looks.append(GREEN if name == "PASS" else RED)
    meshes = [kit.box(c, s, lk) for (c, s), lk in zip(parts, looks)]
    for name, (px, py) in POCKETS.items():  # colored floors, and a check mark or an X beside each pocket
        lk = GREEN if name == "PASS" else RED
        meshes.append(kit.box((px - bx, py - by, BOARD_T + 0.0002), (POCKET_IN, POCKET_IN, 0.0004), lk))
        mx, my = px - bx, 0.030
        if name == "PASS":
            meshes += [kit.box((mx - 0.004, my - 0.003, BOARD_T + 0.0003), (0.003, 0.008, 0.0006), lk),
                       kit.box((mx + 0.003, my + 0.002, BOARD_T + 0.0003), (0.003, 0.016, 0.0006), lk)]
        else:
            meshes += [kit.box((mx, my, BOARD_T + 0.0003), (0.003, 0.014, 0.0006), lk),
                       kit.box((mx, my, BOARD_T + 0.0003), (0.014, 0.003, 0.0006), lk)]
    meshes.append(kit.cylinder((RING_C[0] - bx, RING_C[1] - by, BOARD_T + 0.0002), PUCK_R + 0.003, 0.0004, PENCIL))
    return kit.Prop("report_board", pos=(bx, by, TOP), mass=None, size=(0.180, 0.080, BOARD_T + POCKET_H),
                    meshes=tuple(meshes), colliders=tuple(kit.Collider(c, size=s) for c, s in parts))


PUCK = kit.Prop("tag_puck", pos=PUCK_START, mass=PUCK_MASS, size=(2 * PUCK_R, 2 * PUCK_R, PUCK_H),
                meshes=(kit.cylinder((0, 0, PUCK_H / 2), PUCK_R, PUCK_H, WHITE),),
                colliders=(kit.Collider((0, 0, PUCK_H / 2), radius=PUCK_R, height=PUCK_H),))
PUCK_PARTS = (gk.Ring(0.0, PUCK_R, 0.0, PUCK_H),)
OBJECTS = [PANEL, _rest("rest_right", REST_Y[0]), _rest("rest_left", REST_Y[1]), STRAIGHTEDGE, WEDGE, _board(), PUCK]


# -- geometry of the reading ------------------------------------------------------------------------------------------
def _R(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _pose(o):
    return np.asarray(o.pos, float), _R(o.quat_xyzw)


def se_bottom_edges(objects) -> list:
    """The straightedge's two bottom edges in world coordinates, near one (smaller x) first: each a pair of end points."""
    p, R = _pose(objects["straightedge"])
    down = -1.0 if R[2, 2] >= 0 else 1.0  # whichever long face is lower is the bottom
    edges = []
    for sx in (-1.0, 1.0):
        a = p + R @ np.array([sx * SE_T / 2, -SE_L / 2, down * SE_H / 2])
        b = p + R @ np.array([sx * SE_T / 2, SE_L / 2, down * SE_H / 2])
        edges.append((a, b))
    edges.sort(key=lambda e: e[0][0] + e[1][0])
    return edges


def se_gaps(objects, n: int = 123) -> tuple:
    """Gaps (m) from the straightedge's two bottom edges down to the floor at n points along each, and each point's
    signed place along it (m from the middle, -L/2 .. L/2). Points off the panel count as no floor (inf)."""
    gaps, along = [], []
    for a, b in se_bottom_edges(objects):
        for f in np.linspace(0.0, 1.0, n):
            q = a + f * (b - a)
            on = PANEL_X[0] <= q[0] <= PANEL_X[1]
            fz = float(floor_z(q[1])) if on else -np.inf
            gaps.append(q[2] - fz)
            along.append((f - 0.5) * SE_L)
    return np.array(gaps), np.array(along)


def se_upright(objects) -> bool:
    _, R = _pose(objects["straightedge"])
    return abs(R[2, 2]) >= math.cos(SE_TILT)


def se_in_lane(objects) -> bool:
    """Both bottom edges between the pencil lines, along their whole length, and both ends over the panel."""
    for a, b in se_bottom_edges(objects):
        for q in (a, b):
            if not (LANE_X[0] <= q[0] <= LANE_X[1] and PANEL_Y[0] <= q[1] <= PANEL_Y[1]):
                return False
    return True


def se_on_ends(objects) -> bool:
    """Resting on both high points: within each end's last 5 cm some point of the bottom touches the floor (within
    0.5 mm), and nowhere does it sit deeper in the floor than the solver allows."""
    g, u = se_gaps(objects)
    if np.any(g < -SINK):
        return False
    lo, hi = u <= -SE_L / 2 + END_ZONE, u >= SE_L / 2 - END_ZONE
    return bool(np.any(g[lo] <= CONTACT) and np.any(g[hi] <= CONTACT))


def still(o) -> bool:
    return o.resting(max_speed=STILL_SPEED, max_spin=STILL_SPIN)


def straightedge_set(objects) -> bool:
    """Tier 0.25's condition: on its edge, between the lines, resting on the floor at both ends, still."""
    return (se_upright(objects) and se_in_lane(objects) and se_on_ends(objects) and still(objects["straightedge"]))


def edge_crossing(objects):
    """Where the straightedge's near bottom edge crosses the wedge's centerline, from the two parts' poses only:
    (length of wedge under the edge in m, negative while the tip is short of it; the crossing in the wedge's frame;
    the edge's height there). None if the edge runs parallel to the wedge or misses its centerline."""
    pw, Rw = _pose(objects["wedge"])
    a, b = se_bottom_edges(objects)[0]
    la, lb = Rw.T @ (a - pw), Rw.T @ (b - pw)  # the edge in the wedge's frame
    if abs(lb[1] - la[1]) < 1e-9 or la[1] * lb[1] > 0:
        return None
    f = -la[1] / (lb[1] - la[1])
    return TIP_X - (la + f * (lb - la))[0], la + f * (lb - la), float((a + f * (b - a))[2])


def wedge_reading(objects) -> dict | None:
    """The scale's reading where the straightedge's near bottom edge crosses the wedge (m), the length of wedge under
    the edge (m), and the gaps from the wedge's top to the edge and from its bottom to the floor (m). None when the
    edge doesn't cross the wedge or the wedge is not lying flat and pointing under it."""
    pw, Rw = _pose(objects["wedge"])
    if Rw[2, 2] < math.cos(WEDGE_TILT):
        return None
    if abs(math.atan2(Rw[1, 0], Rw[0, 0])) > WEDGE_YAW:  # tip toward +x, under the straightedge
        return None
    c = edge_crossing(objects)
    if c is None or not (0.0 <= c[0] <= W_L):
        return None
    s, q, edge_z = c
    t = float(wedge_thickness(s))
    top = pw + Rw @ np.array([q[0], 0.0, t])
    sides = [pw + Rw @ np.array([q[0], sy, 0.0]) for sy in (-W_W / 2, W_W / 2)]
    base = floor_max(sides[0][1], sides[1][1])  # the wedge rests on the highest floor under its width
    bottom_gap = min(float(c[2]) for c in sides) - base
    return {"reading": t, "under": s, "top_gap": float(edge_z - top[2]), "bottom_gap": bottom_gap,
            "floor_mm": (FLOOR - base) * 1000.0}


def wedge_in_gap(objects) -> dict | None:
    """Tier 0.5's condition: at least 2 mm of wedge under the edge, its top touching the straightedge and its bottom on
    the floor (each within 0.5 mm), and still. Returns the reading's record, or None."""
    r = wedge_reading(objects)
    if r is None or r["under"] < IN_GAP_MIN:
        return None
    if not (-SINK <= r["top_gap"] <= CONTACT and -SINK <= r["bottom_gap"] <= CONTACT):
        return None
    return r if still(objects["wedge"]) else None


def puck_pocket(objects) -> str | None:
    """The pocket the puck stands in (upright, inside its walls, on its floor, at rest), or None."""
    o = objects["tag_puck"]
    p, R = _pose(o)
    if R[2, 2] < math.cos(PUCK_TILT) or not still(o):
        return None
    for name, (px, py) in POCKETS.items():
        inside = abs(p[0] - px) <= POCKET_IN / 2 - PUCK_R + 0.001 and abs(p[1] - py) <= POCKET_IN / 2 - PUCK_R + 0.001
        if inside and p[2] <= TOP + BOARD_T + 0.003:
            return name
    return None


def puck_in_any_pocket(objects) -> bool:
    """The puck's center over a pocket's floor at or below its rim (resting or not): a report has been made."""
    p = np.asarray(objects["tag_puck"].pos, float)
    return any(abs(p[0] - px) <= POCKET_IN / 2 and abs(p[1] - py) <= POCKET_IN / 2 and
               p[2] <= TOP + BOARD_T + POCKET_H for px, py in POCKETS.values())


def hands_on(objects) -> tuple:
    """Grippers whose own shape touches the straightedge, the wedge or the puck (guardkit: contact within 2 mm)."""
    out = set()
    for name, parts in (("straightedge", SE_PARTS), ("wedge", W_PARTS), ("tag_puck", PUCK_PARTS)):
        out.update(gk.touching(objects, name, parts, grippers=GRIPPERS))
    return tuple(sorted(out))


# -- one trial's record --------------------------------------------------------------------------------------------
class Ledger:
    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.measured = False  # the wedge has measured a gap under a set straightedge (any reading)
        self.guess = False  # the puck reached a pocket before that
        self.tiers = []  # tier per control step (last HOLD_STEPS kept)
        self.held = False
        self.last = {}


_L = Ledger()
MEASURED = gk.Latch(steps=2)


def tier(objects) -> float:
    """The ladder, before caps: 0.25 straightedge set; 0.5 wedge in a gap under it; 0.75 the reading within 0.3 mm of
    the trough's depth with the wedge on the deepest 6 cm; 1.0 the puck in the correct pocket."""
    if not straightedge_set(objects):
        return 0.0
    r = wedge_in_gap(objects)
    if r is None:
        return 0.25
    if abs(r["reading"] * 1000.0 - DIP_MM) > READ_TOL_MM:
        return 0.5
    if r["floor_mm"] < DIP_MM - ON_DEEPEST_MM:  # its whole width on the deepest 6 cm, not a shallower strip
        return 0.5
    return 1.0 if puck_pocket(objects) == VERDICT else 0.75


def _update(objects):
    """Advance the trial's record once per control step (success() and score() share it)."""
    global _L
    if _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _L.key) or (key is None and objects is _L.objs):
        return
    _L.key, _L.objs = key, objects
    gap_now = straightedge_set(objects) and wedge_in_gap(objects) is not None
    _L.measured = MEASURED.feed(objects, gap_now)
    if not _L.measured and puck_in_any_pocket(objects):
        _L.guess = True
    _L.tiers = (_L.tiers + [tier(objects)])[-HOLD_STEPS:]
    _L.held = bool(hands_on(objects))
    _L.last = {"pocket": puck_pocket(objects), "reading": (wedge_reading(objects) or {}).get("reading")}


def caps() -> float:
    """A hand still on a tool or the puck 0.5; the puck in the wrong pocket 0.5; a report made before any measurement
    0.5 (latched)."""
    cap = 1.0
    if _L.held:
        cap = min(cap, HELD_CAP)
    if _L.last.get("pocket") not in (None, VERDICT):
        cap = min(cap, WRONG_CAP)
    if _L.guess:
        cap = min(cap, GUESS_CAP)
    return cap


def score(objects):
    """0.25 straightedge on its edge between the pencil lines, resting on the floor in both end zones, still; 0.5 and
    the wedge in a gap under its near edge, touching both, still; 0.75 and the reading within 0.3 mm of the trough's
    depth, the wedge on the deepest 6 cm; 1.0 and the puck in the correct pocket. Each tier must hold for the last 1 s.
    Caps (0.5): a hand on a tool or the puck at the end, the wrong pocket, a report before any measurement. Doing
    nothing scores 0."""
    _update(objects)
    s = min(_L.tiers) if len(_L.tiers) >= HOLD_STEPS else 0.0
    return min(s, caps())


def success(objects):
    _update(objects)
    return bool(_L.tiers) and _L.tiers[-1] == 1.0 and caps() == 1.0


# -- scripted demo: open loop for the board fixed by FLATNESS_CASE ----------------------------------------------------
# It is planned from this file's constants for the chosen case (where the deepest strip is, how deep, and so the
# report), like any fixed layout, and reads nothing from the simulator while it runs. A model gets the random draw
# and has to find the trough with the wedge.
YAW_X = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers closing along x (each arm's reachable sign)
YAW_Y = 0.0  # fingers closing along y
SE_GRIP_Y = 0.150  # each hand 15 cm from the straightedge's middle
SE_PAD_DOWN = 0.025  # pad tips 2.5 cm below the straightedge's top
SE_X = 0.360  # the straightedge's middle 2 cm short of the far pencil line, its far face 7.5 mm short (its bottom stays between the lines)
WEDGE_GRIP = -0.025  # grip the wedge 2.5 cm behind its middle (10.3 mm thick there)
PAD_CLEAR = 0.002  # pad tips 2 mm above what the wedge lies on
WEDGE_OPEN = 0.5 * FINGER_OPEN  # half open at the wedge (pads 8.5 mm off its sides), so the fingers stay off the panel
TIP_BACK = 0.015  # the wedge is laid down with its tip 1.5 cm short of the straightedge's face
FIST_BACK = 0.012  # closed fingertips' front face ahead of the tool point
FIST_GAP = 0.006  # the fist comes down 6 mm behind the heel
FIST_Z = 0.004  # pad tips 4 mm above the floor behind the heel (15.3 mm tall)
PUSH_FAST, PUSH_SLOW = 0.006, 0.002  # m/s: until the tip is 2 mm short of the edge, then on under it
SLOW_SHORT = 0.002  # m: the slow push starts with the tip this far short of the face
PUSH_OVER = 0.004  # m: the hand's goal ends 4 mm past where the wedge stops (the arm gives about that much under the
#                    wedge's ~1 N of friction, so the wedge still reaches the stop; it then presses about 1 N)
BACK_OFF = 0.003  # m: then the hand backs off so it doesn't keep loading the wedge
HOME_POS = {"left": (0.20, 0.25, TOP + 0.16), "right": (0.20, -0.25, TOP + 0.16)}
PUCK_RELEASE = 0.38 * FINGER_OPEN  # let go of the puck by opening only this far: fingertips about 2 mm off the puck
#                                    and 1 mm inside the pocket's walls, until they are lifted out of it
END_WAIT = 3.0  # s: both hands aside at the end while the final state holds (the run stops when the plan ends)


def _tcp(x, y, pad_z):
    return (float(x), float(y), float(pad_z + PAD_TIP_PAST_TCP))


def _wp(d, x, y, pad_z, finger, label, yaw):
    return Waypoint(d, _tcp(x, y, pad_z), finger, label, yaw)


def _straightedge_moves(side):
    """Both hands lift the straightedge out of its rests and set it on its edge on the lane's center line."""
    O, C = FINGER_OPEN, FINGER_CLOSED
    y, yaw = (1 if side == "left" else -1) * SE_GRIP_Y, YAW_X[side]
    pad0 = TOP + REST_H + SE_H - SE_PAD_DOWN
    lift = pad0 + 0.05
    pad_set = FLOOR + 0.001 + SE_H - SE_PAD_DOWN
    return [_wp(2.0, REST_X, y, pad0 + 0.05, O, "over the straightedge", yaw),
            _wp(1.5, REST_X, y, pad0, O, "pads either side of it", yaw),
            _wp(0.8, REST_X, y, pad0, C, "grip it with both hands", yaw),
            _wp(1.5, REST_X, y, lift, C, "lift it out of the rests", yaw),
            _wp(2.5, SE_X, y, lift, C, "carry it over the line", yaw),
            _wp(2.0, SE_X, y, pad_set, C, "set it down on its edge", yaw),
            _wp(1.0, SE_X, y, pad_set, O, "let go", yaw),
            _wp(1.2, SE_X, y, pad_set + 0.05, O, "lift clear", yaw),
            Waypoint(2.0, HOME_POS[side], O, "aside", YAW_Y)]


def wedge_stop_under() -> float:
    """How far (m) the wedge's tip goes under the straightedge's face before it fills the gap there, on this case's
    board (the wedge reads the trough's depth there)."""
    return (DIP_MM / 1000.0 - W_TIP) / W_SLOPE


def _wedge_moves():
    """The right hand takes the wedge off the table, lays it on the floor at the deepest strip with its tip 1.5 cm short
    of the straightedge's face, closes, and pushes its heel in +x until the tip is under the face by as far as the
    trough's depth allows (plus 4 mm of the arm's give), then backs off and goes aside."""
    O, C = FINGER_OPEN, FINGER_CLOSED
    y = DIP_Y_CM / 100.0
    x_face = SE_X - SE_T / 2
    ox = x_face - TIP_BACK - TIP_X  # the wedge's middle where it is laid down
    gx, gy = W_START[0] + WEDGE_GRIP, W_START[1]
    carry = FLOOR + PAD_CLEAR + 0.012
    over = TOP + REST_H + REST_CHEEK + PAD_CLEAR + 0.015  # the wedge 1.5 cm over the rests' cheeks on the way
    H = WEDGE_OPEN
    fist0 = ox - TIP_X - FIST_BACK - FIST_GAP
    fist_slow = x_face - SLOW_SHORT - W_L - FIST_BACK
    fist_end = x_face + wedge_stop_under() - W_L - FIST_BACK + PUSH_OVER
    fz = FLOOR + FIST_Z
    return [_wp(2.0, gx, gy, TOP + PAD_CLEAR + 0.05, H, "over the wedge", YAW_Y),
            _wp(1.2, gx, gy, TOP + PAD_CLEAR, H, "pads either side", YAW_Y),
            _wp(0.8, gx, gy, TOP + PAD_CLEAR, C, "grip the wedge", YAW_Y),
            _wp(1.2, gx, gy, over, C, "lift it over the rests' height", YAW_Y),
            _wp(2.5, ox + WEDGE_GRIP, y, over, C, "to the deepest strip", YAW_Y),
            _wp(1.0, ox + WEDGE_GRIP, y, carry, C, "down near the floor", YAW_Y),
            _wp(0.8, ox + WEDGE_GRIP, y, FLOOR + PAD_CLEAR + 0.0015, C, "lower it", YAW_Y),
            _wp(0.6, ox + WEDGE_GRIP, y, FLOOR + PAD_CLEAR + 0.0015, H, "let it lie", YAW_Y),
            _wp(0.8, ox + WEDGE_GRIP, y, FLOOR + 0.04, H, "lift the fingers", YAW_Y),
            _wp(0.8, fist0, y, FLOOR + 0.04, C, "close; behind the heel", YAW_Y),
            _wp(0.8, fist0, y, fz, C, "fingertips behind the heel", YAW_Y),
            _wp((fist_slow - fist0) / PUSH_FAST, fist_slow, y, fz, C, "push the wedge to the edge", YAW_Y),
            _wp((fist_end - fist_slow) / PUSH_SLOW, fist_end, y, fz, C, "push it slowly under the edge", YAW_Y),
            _wp(1.0, fist_end, y, fz, C, "hold", YAW_Y),
            _wp(0.5, fist_end - BACK_OFF, y, fz, C, "back off", YAW_Y),
            _wp(0.8, fist_end - BACK_OFF, y, FLOOR + 0.04, O, "hand up", YAW_Y),
            Waypoint(1.5, HOME_POS["right"], O, "aside", YAW_Y)]


def _puck_to(pocket):
    """The left hand takes the tag puck off its ring and stands it in the pocket."""
    O, C = FINGER_OPEN, FINGER_CLOSED
    px, py = PUCK_START[0], PUCK_START[1]
    qx, qy = POCKETS[pocket]
    pz, hi = TOP + BOARD_T + 0.002, TOP + 0.07  # fingertips 2 mm above the board: they hold the puck's whole height
    return [_wp(2.0, px, py, hi, O, "over the puck", YAW_Y),
            _wp(1.2, px, py, pz, O, "pads around the puck", YAW_Y),
            _wp(0.8, px, py, pz, C, "grip the puck", YAW_Y),
            _wp(1.2, px, py, hi, C, "lift it", YAW_Y),
            _wp(1.5, qx, qy, hi, C, f"over {pocket}", YAW_Y),
            _wp(1.5, qx, qy, pz + 0.002, C, "lower it in", YAW_Y),
            _wp(0.8, qx, qy, pz + 0.002, PUCK_RELEASE, "let go", YAW_Y),
            _wp(0.8, qx, qy, pz + 0.025, PUCK_RELEASE, "fingers up out of the pocket", YAW_Y),
            _wp(1.0, qx, qy, hi, O, "lift clear", YAW_Y),
            Waypoint(1.5, HOME_POS["left"], O, "aside", YAW_Y)]


def demo():
    """Open loop for the board FLATNESS_CASE fixes. Both hands set the straightedge on its edge on the line; the right
    hand lays the wedge at the deepest strip and pushes it under the edge until it fills the gap; then the left hand
    stands the puck in the pocket for this board's depth (1.6 mm or less: PASS)."""
    left, right = _straightedge_moves("left"), _straightedge_moves("right")
    right += _wedge_moves()
    t_right = sum(w.duration for w in right)
    t_left = sum(w.duration for w in left)
    left.append(Waypoint(t_right - t_left + 1.0, HOME_POS["left"], FINGER_OPEN, "wait for the wedge", YAW_Y))
    left += _puck_to(VERDICT)
    t_end = sum(w.duration for w in left) + END_WAIT
    left.append(Waypoint(END_WAIT, HOME_POS["left"], FINGER_OPEN, "wait while the result holds", YAW_Y))
    right.append(Waypoint(t_end - sum(w.duration for w in right), HOME_POS["right"], FINGER_OPEN, "wait", YAW_Y))
    return {"left": left, "right": right}
