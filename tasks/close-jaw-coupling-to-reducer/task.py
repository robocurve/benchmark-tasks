"""On a drive's steel base plate on the station table, in front of the arms: a right-angle worm gear reducer with its
motor, its output shaft pointing to the robot's right, and in line with it a driven shaft in two pillow-block bearings.
The two shafts are being coupled with a Lovejoy L-100 jaw coupling (L-type, two sintered-iron hubs and one SOX NBR
spider). The reducer's hub is already set flush with its shaft end, and the spider is already pushed onto its three
jaws. The driven hub slides on its keyed shaft and has been pushed back, so its jaws stand 45 mm from the reducer hub's
face. The driven shaft turns freely in its bearings and the hub turns with it.

The job is to close the coupling: slide the driven hub up to the spider, turn it until its three jaws line up with the
spider's three free pockets (they only go in at one phase in every 120 degrees, and the start phase puts each jaw
square on a spider leg), slide it in until its jaws are fully in the spider, and leave it at Lovejoy's gap between the
hub faces (0.750 in, plus or minus 0.040 in, for the L-100), hands off, ready for the set screws.

Every part is rigid and the meshing is by contact alone: the jaws and the spider's legs are convex hulls, so the hub
goes in only when it is turned to the right phase. The reducer and its hub are fixed (the reducer is locked out). The
spider is a free body resting on the reducer hub's jaws. The driven shaft is on a hinge (its bearings) and the hub on a
slide along it (its keyed slip fit). The hinge has stops at +-180 degrees from the start; a phase that meshes lies
within 120 degrees either way.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import gripkit  # noqa: E402
import yamgeom as yg  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Close the jaw coupling between the gear reducer and the driven shaft."
IN = 0.0254  # m per inch

# -- the coupling: Lovejoy L-100 (the Timken/Lovejoy L100-HUB-1"-1/4X1/8KW hub page and the L/SW installation guide) --
R_HUB = 2.54 * IN / 2  # hub outside diameter 2.54 in
LTB = 1.38 * IN  # hub length through bore 1.38 in (the solid body round the bore)
JAW_L = 2.09 * IN - LTB  # the jaws stand out of the body by the hub length (2.09 in) less the length through bore
OAL = 3.480 * IN  # overall length of the closed coupling 3.480 in
GAP = 0.750 * IN  # the installation guide's gap 'G' between hub faces (Table 5, L / SW 100 row), 0.750 in ...
# (the hub page's own "Dimension G", 0.71 in, is the jaw length: hub length less length through bore)
GAP_TOL = 0.040 * IN  # ... and its allowed change in G, 0.040 in
HUB_KG = 1.40 * 0.4536  # hub weight 1.40 lb (1 in bore with keyway)
JAW_R0 = 0.0175  # m: the jaws' inner radius (the 35 mm maximum bore)
N_JAWS = 3
JAW_DEG = 36.0  # each jaw's width round the hub (estimated from maker photos)
# The spider (L/AL099/100 SOX, NBR, solid center, 0.06 lb): six legs round a solid center. Its legs fill the space
# between the hub faces when the coupling is closed (OAL less two lengths through bore).
SPIDER_L = OAL - 2 * LTB  # 18.29 mm
SPIDER_KG = 0.06 * 0.4536
SPIDER_R = 0.0305  # m: the legs' outer radius (estimated from maker photos)
SPIDER_C = 0.0172  # m: the solid center's radius, 0.3 mm inside the jaws' inner radius: rides on their faces
LEG_DEG = 18.0  # each leg's width round the axis (estimated from maker photos; the clearance below is widened)
CLOCK = 90.0  # the reducer hub is clocked with a jaw straight down (phi = 90 is -z), so the spider rests centred on
# that jaw's inner face and the driven hub's jaws, 60 degrees round, keep their radial clearance
# Each pocket between two legs is 60 - LEG_DEG = 42 degrees wide for a 36 degree jaw: 3 degrees of play a side
# (1.2 mm at mid-jaw). The real NBR legs fit the jaws snugly and squeeze; here the spider is rigid, so the play stands in
# for that squeeze.
MESH_MIN = JAW_L + SPIDER_L  # m: the face gap at which the driven hub's jaw tips reach the spider's legs (36.3 mm)

# -- layout (world frame: x forward from the arms, y to the robot's left, z up) -----------------------------------------
XC = 0.330  # the shafts' axis runs along y at this x ...
PLATE_T = 0.5 * IN  # ... over a 1/2 in steel base plate on the table ...
CENTER_H = 0.065  # ... this high above the plate's top
AXIS_Z = TOP + PLATE_T + CENTER_H
YA = 0.020  # world y of the reducer hub's jaw-side face (its body runs +y from here, its jaws -y)
GAP_START = 0.045  # m: the driven hub's face starts 45 mm from the reducer hub's face
SHAFT_R = 0.5 * IN  # 1 in shafts (both hubs bored 1 in; the L-100 takes 11-35 mm)
HOUSING_Y0 = YA + LTB + 0.012  # the reducer's output face (12 mm of its shaft shows behind the hub)
DRIVEN_END = YA - GAP - 0.0015  # the driven shaft's end: 1.5 mm inside the hub's bore when the gap is G
BLOCK_YS = (-0.115, -0.205)  # the two pillow blocks' centers along the driven shaft
SHAFT_Y1 = -0.240  # the driven shaft's far end
START_TURN_DEG = 30.0  # the driven shaft starts turned so that each of its hub's jaws stands square on a spider leg
IRON = kit.Look((0.42, 0.43, 0.45), 0.45, 0.6)
STEEL = kit.Look((0.62, 0.63, 0.65), 0.35, 0.8)
PAINT = kit.Look((0.16, 0.30, 0.55), 0.55)  # the reducer's and motor's enamel
BLOCK_LOOK = kit.Look((0.22, 0.22, 0.24), 0.6)
NBR = kit.Look((0.06, 0.06, 0.06), 0.8)
IRON_MAT = (0.40, 0.23, "average")  # sintered iron hubs, cast-iron value: on mild steel, clean and dry 0.4 / 0.23
# (Engineering ToolBox)
NBR_MAT = (0.80, 0.70, "average")  # NBR: inside dry rubber's 0.6-0.9 on hard surfaces (Engineering ToolBox)


def ry(deg: float) -> np.ndarray:
    """Rotation about +y by `deg` degrees."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def radial(phi_deg: float, r: float) -> np.ndarray:
    """The point at radius r and angle phi (degrees) round the +y axis, in the plane y = 0 (phi = 0 along +x)."""
    return ry(phi_deg) @ np.array([r, 0.0, 0.0])


def prism(poly, y0: float, y1: float) -> tuple:
    """(points, triangles) of a convex prism: the convex polygon `poly` (points in the plane y = 0, in order round
    it) running from y0 to y1."""
    p = np.asarray(poly, float)
    n = len(p)
    pts = [q + np.array([0.0, y0, 0.0]) for q in p] + [q + np.array([0.0, y1, 0.0]) for q in p]
    tris = [(0, i, i + 1) for i in range(1, n - 1)] + [(n, n + i + 1, n + i) for i in range(1, n - 1)]
    for i in range(n):
        j = (i + 1) % n
        tris += [(i, j, n + j), (i, n + j, n + i)]
    return pts, tris


def sector(a0: float, a1: float, r0: float, r1: float, k: int = 3) -> list:
    """A convex polygon covering the annular sector from angle a0 to a1 (degrees), radius r0 to r1."""
    outer = [radial(a0 + (a1 - a0) * i / k, r1) for i in range(k + 1)]
    return [radial(a0, r0)] + outer + [radial(a1, r0)]


def ngon(r: float, n: int = 24) -> list:
    """A regular n-gon round the +y axis whose flats are at radius r."""
    rc = r / math.cos(math.pi / n)
    return [radial(360.0 * i / n, rc) for i in range(n)]


def hull_col(pts, tris, material=None) -> kit.Collider:
    return kit.Collider((0.0, 0.0, 0.0), hull=tuple(tuple(float(v) for v in q) for q in pts),
                        faces=tuple(tris), material=material)


def hull_mesh(pts, tris, look) -> kit.Mesh:
    return kit._mesh(pts, [3] * len(tris), [i for t in tris for i in t], look)


def jaw_parts(centers, y0, y1, look, material):
    """Meshes and colliders of the jaws centered at angles `centers`, from y0 to y1: each jaw is three convex hulls
    side by side (one hull for a whole jaw would bridge its curved inner face)."""
    meshes, cols = [], []
    for c in centers:
        a = c - JAW_DEG / 2
        for i in range(3):
            pts, tris = prism(sector(a + JAW_DEG * i / 3, a + JAW_DEG * (i + 1) / 3, JAW_R0, R_HUB, 2), y0, y1)
            meshes.append(hull_mesh(pts, tris, look))
            cols.append(hull_col(pts, tris, material))
    return meshes, cols


def body_parts(y0, y1, look, material):
    """A hub's solid body round its bore (24 flats), from y0 to y1."""
    pts, tris = prism(ngon(R_HUB), y0, y1)
    return [kit.cylinder((0.0, (y0 + y1) / 2, 0.0), R_HUB, abs(y1 - y0), look, axis="y", n=48)], \
        [hull_col(pts, tris, material)]


def _box(lo, hi, look, material=None):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c, s = tuple((lo + hi) / 2), tuple(hi - lo)
    return kit.box(c, s, look), kit.Collider(c, size=s, material=material)


# -- the reducer side (fixed): base plate, reducer and motor, output shaft, the reducer hub (its frame: origin on the
#    axis at the hub's jaw-side face, axes as the world's) --------------------------------------------------------------
A_JAWS = [CLOCK + 120.0 * k for k in range(N_JAWS)]  # the reducer hub's jaws, one straight down
_PLATE_Z = -CENTER_H - PLATE_T
_a_meshes, _a_cols = body_parts(0.0, LTB, IRON, IRON_MAT)
_m, _c = jaw_parts(A_JAWS, -JAW_L, 0.0, IRON, IRON_MAT)
_a_meshes, _a_cols = _a_meshes + _m, _a_cols + _c
_REDUCER_PARTS = [
    _box((-0.080, -0.260 - YA, _PLATE_Z), (0.280, 0.200 - YA, -CENTER_H), STEEL),  # the base plate
    _box((-0.050, HOUSING_Y0 - YA, -CENTER_H), (0.060, HOUSING_Y0 - YA + 0.110, 0.060), PAINT),  # the gear case
    _box((0.060, HOUSING_Y0 - YA + 0.020, -0.010), (0.085, HOUSING_Y0 - YA + 0.090, 0.080), PAINT),  # input flange
]
_MOTOR_Y = HOUSING_Y0 - YA + 0.055
_REDUCER_MESHES = [m for m, _ in _REDUCER_PARTS] + _a_meshes + [
    kit.cylinder((0.165, _MOTOR_Y, 0.035), 0.060, 0.160, PAINT, axis="x", n=40),  # the motor
    kit.cylinder((0.0, (LTB + HOUSING_Y0 - YA) / 2, 0.0), SHAFT_R, HOUSING_Y0 - YA - LTB + 0.002, STEEL, axis="y"),
    kit.cylinder((0.0, HOUSING_Y0 - YA - 0.002, 0.0), 0.026, 0.004, BLOCK_LOOK, axis="y")]  # the output seal
_REDUCER_COLS = [c for _, c in _REDUCER_PARTS] + _a_cols + [
    kit.Collider((0.165, _MOTOR_Y, 0.035), radius=0.060, height=0.160, axis="X"),
    kit.Collider((0.0, (LTB + HOUSING_Y0 - YA) / 2, 0.0), radius=SHAFT_R, height=HOUSING_Y0 - YA - LTB, axis="Y")]
REDUCER = kit.Prop(
    "reducer", pos=(XC, YA, AXIS_Z), mass=None, size=(0.36, 0.46, CENTER_H + PLATE_T + 0.10),
    meshes=tuple(_REDUCER_MESHES), colliders=tuple(_REDUCER_COLS), material=IRON_MAT)

# -- the driven side: pillow blocks (fixed base), the driven shaft (on its bearings) and the driven hub (sliding on the
#    shaft's key). The base frame: origin on the axis at world y = 0 -------------------------------------------------------
B_JAWS = [CLOCK + 60.0 + 120.0 * k for k in range(N_JAWS)]  # the driven hub's jaws (own frame), meshing at turn 0
YB0 = YA - GAP_START  # world y of the driven hub's face at the start
_BLOCK = []
for _y in BLOCK_YS:  # each pillow block: a cast housing round the bearing on a foot, on a riser bar
    _BLOCK += [_box((-0.064, _y - 0.019, -CENTER_H), (0.064, _y + 0.019, -0.040), BLOCK_LOOK),  # riser and foot
               _box((-0.040, _y - 0.019, -0.040), (0.040, _y + 0.019, -0.018), BLOCK_LOOK)]  # the housing's base
_BLOCK_MESHES = [m for m, _ in _BLOCK] + [kit.cylinder((0.0, y, 0.0), 0.032, 0.038, BLOCK_LOOK, axis="y")
                                         for y in BLOCK_YS]
_shaft_len = DRIVEN_END - SHAFT_Y1
# The keyed slip fit and the two bearings resist motion lightly, so the hub and shaft stay where they are left (without
# it the frictionless joints coast on after the hand lets go): drag, a force against speed applied each control step
SLIP_DRAG = 8.0  # N s/m on the hub's slide (0.08 N at 1 cm/s)
BEARING_DRAG = 0.004  # N m s/rad on the shaft's turn
SHAFT = kit.Link(
    "shaft", "revolute", "Y", anchor=(0.0, 0.0, 0.0), lower=-math.pi, upper=math.pi,
    start=math.radians(START_TURN_DEG), mass=7.85e3 * math.pi * SHAFT_R ** 2 * _shaft_len, damping=0.005,
    drag=BEARING_DRAG,
    meshes=(kit.cylinder((0.0, (DRIVEN_END + SHAFT_Y1) / 2, 0.0), SHAFT_R, _shaft_len, STEEL, axis="y"),
            kit.box((0.0, (BLOCK_YS[1] + SHAFT_Y1) / 2 - 0.004, SHAFT_R - 0.0005), (0.00635, 0.024, 0.0032), STEEL)),
    colliders=(kit.Collider((0.0, (DRIVEN_END + SHAFT_Y1) / 2, 0.0), radius=SHAFT_R, height=_shaft_len, axis="Y"),))
_b_meshes, _b_cols = body_parts(-LTB, 0.0, IRON, IRON_MAT)
_m, _c = jaw_parts(B_JAWS, 0.0, JAW_L, IRON, IRON_MAT)
HUB = kit.Link(
    "hub", "prismatic", "Y", anchor=(0.0, YB0, 0.0), lower=-0.010, upper=GAP_START - JAW_L + 0.0005, start=0.0,
    mass=HUB_KG, damping=4.0, drag=SLIP_DRAG, parent="shaft",
    meshes=tuple(_b_meshes + _m), colliders=tuple(_b_cols + _c))
DRIVE = kit.Mechanism(
    "drive", pos=(XC, 0.0, AXIS_Z), size=(0.13, 0.30, CENTER_H + 0.04),
    meshes=tuple(_BLOCK_MESHES), colliders=tuple(c for _, c in _BLOCK), links=(SHAFT, HUB), material=IRON_MAT)

# -- the spider: a free body resting on the reducer hub's jaws (its frame: origin on its axis at its reducer-side end,
#    y along the axis, its legs running -y from there) ----------------------------------------------------------------
LEGS = [CLOCK + 30.0 + 60.0 * k for k in range(6)]  # legs between the reducer hub's jaws (a leg either side of each)
_s_meshes, _s_cols = [], []
for _a in LEGS:
    _pts, _tris = prism(sector(_a - LEG_DEG / 2, _a + LEG_DEG / 2, SPIDER_C - 0.001, SPIDER_R, 3), -SPIDER_L, 0.0)
    _s_meshes.append(hull_mesh(_pts, _tris, NBR))
    _s_cols.append(hull_col(_pts, _tris))
for _k in range(6):  # the solid center, six wedges
    _pts, _tris = prism([np.zeros(3)] + [radial(60.0 * _k + 10.0 * i, SPIDER_C) for i in range(7)], -SPIDER_L, 0.0)
    _s_cols.append(hull_col(_pts, _tris))
_s_meshes.append(kit.cylinder((0.0, -SPIDER_L / 2, 0.0), SPIDER_C, SPIDER_L, NBR, axis="y", n=36))
SPAWN_GAP = 0.0002  # m: the spider spawns this far off the reducer hub's face, centered on the axis
SPIDER_START = (XC, YA - SPAWN_GAP, AXIS_Z)
SPIDER = kit.Prop(
    "spider", pos=SPIDER_START, mass=SPIDER_KG, size=(2 * SPIDER_R, SPIDER_L, 2 * SPIDER_R),
    meshes=tuple(_s_meshes), colliders=tuple(_s_cols), material=NBR_MAT, contact_offset=0.0008,
    solver_iterations=(32, 8))

OBJECTS = [REDUCER, DRIVE, SPIDER]
gripkit.use_rubber_pads()  # the YAM's finger pads are rubber-faced

# -- scored thresholds --------------------------------------------------------------------------------------------------
LINE_LO, LINE_HI = GAP - GAP_TOL, GAP + GAP_TOL  # at the line: the face gap within G +- delta-G (18.03-20.07 mm)
FORCED = SPIDER_L - 0.0008  # a face gap below this means a hub driven into the rigid spider (not a real state): 0
DEEP = 0.005  # m: meshed: the jaws at least 5 mm into the spider's pockets
ENTERED = 0.001  # m: in phase: the jaws at least 1 mm into the pockets
PHASE_DEG = 6.0  # the driven hub's jaws within 6 degrees of the spider's free pockets (the play is 3 a side)
SEAT_AX = 0.0015  # spider seated: its reducer-side end within 1.5 mm of the reducer hub's face ...
SEAT_OFF = 0.0015  # ... its axis within 1.5 mm of the shafts' axis ...
SEAT_TILT = 4.0  # ... and within 4 degrees of it ...
SEAT_PHASE = 5.0  # ... its legs within 5 degrees of their places either side of the reducer hub's jaws
FALLEN = 0.015  # m: the spider knocked off: its center 15 mm below the axis with no hand on it (latched)
TOUCH = 0.001  # m: an arm link touches a part when within 1 mm of its outline
CLEAR = 0.005  # m: hands clear: every arm link at least 5 mm from the driven hub, the driven shaft and the spider
STILL_STEP = 0.0005  # m per control step (1 cm/s), and STILL_DEG degrees per step
STILL_DEG = 0.5
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
ROBOTS = ("left_robot", "right_robot")
LINK_POSES: dict = {}  # "right_robot/gripper" -> (pos, R) in the env frame at this control step (filled in the sim)


def rotm(q) -> np.ndarray:
    """Rotation matrix of an (x, y, z, w) quaternion."""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def wrap(deg: float, period: float) -> float:
    """deg folded into [-period/2, period/2)."""
    return (deg + period / 2) % period - period / 2


def turn_about_y(R) -> float:
    """The turn (degrees) about +y of a rotation that keeps y (nearly) fixed."""
    R = np.asarray(R, float)
    return math.degrees(math.atan2(R[0, 2], R[0, 0]))


# -- reading the coupling -------------------------------------------------------------------------------------------------
def coupling(objects) -> dict:
    """The coupling's state from the parts' poses: face gap (m), how far the driven hub's jaws are into the spider's
    pockets (m), the driven hub's phase error (degrees), and the spider's seat on the reducer hub."""
    h, s = objects["drive/hub"], objects["spider"]
    hp, hR = np.asarray(h.pos, float), rotm(h.quat_xyzw)
    sp, sR = np.asarray(s.pos, float), rotm(s.quat_xyzw)
    gap = YA - float(hp[1])
    depth = (float(hp[1]) + JAW_L) - (float(sp[1]) - SPIDER_L)  # jaw tips past the legs' ends
    phase = wrap(turn_about_y(hR), 120.0)
    s_ax = YA - float(sp[1])
    s_off = float(math.hypot(sp[0] - XC, sp[2] - AXIS_Z))
    s_tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(sR[1, 1])))))
    s_phase = wrap(turn_about_y(sR), 60.0)
    seated = (-0.0005 <= s_ax <= SEAT_AX and s_off <= SEAT_OFF and s_tilt <= SEAT_TILT and abs(s_phase) <= SEAT_PHASE)
    return {"gap": gap, "depth": depth, "phase": phase, "seated": seated, "s_ax": s_ax, "s_off": s_off,
            "s_tilt": s_tilt, "s_phase": s_phase, "s_low": AXIS_Z - float(sp[2]), "turn": turn_about_y(hR)}


# -- the outlines the arms are checked against ----------------------------------------------------------------------------
def _seg_gap(q, a, b, r):
    """Distance (m, 0 inside) from points q to a flat-ended cylinder from a to b of radius r."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = float(np.linalg.norm(b - a))
    u = (b - a) / L
    d = q - a
    s = d @ u
    rad = np.linalg.norm(d - s[:, None] * u, axis=1)
    return np.hypot(np.maximum(rad - r, 0.0), np.maximum(np.maximum(-s, s - L), 0.0))


def hub_gap(q):
    """Distance from hub-frame points to the driven hub's outline (body and jaws as one cylinder)."""
    return _seg_gap(q, (0, -LTB, 0), (0, JAW_L, 0), R_HUB)


def shaft_gap(q):
    return _seg_gap(q, (0, SHAFT_Y1, 0), (0, DRIVEN_END, 0), SHAFT_R)


def spider_gap(q):
    return _seg_gap(q, (0, -SPIDER_L, 0), (0, 0, 0), SPIDER_R)


def _parts(objects):
    out = {}
    for name, key, gap, reach in (("hub", "drive/hub", hub_gap, 0.05), ("shaft", "drive/shaft", shaft_gap, 0.26),
                                  ("spider", "spider", spider_gap, 0.04)):
        o = objects[key]
        out[name] = (np.asarray(o.pos, float), rotm(o.quat_xyzw), gap, reach)
    return out


def link_gap(parts, key, part) -> float:
    """Smallest gap (m) between arm link `key` ("right_robot/tip_left") as posed and `part`; 0.02 for a link more
    than 2 cm away."""
    o, R = LINK_POSES[key]
    co, Rc, gap, reach = parts[part]
    link = key.split("/")[1]
    _, cen, rad = yg.cloud(link)
    o_, R_ = np.asarray(o, float), np.asarray(R, float)
    centre = (o_ + R_ @ cen - co) @ Rc
    if float(np.linalg.norm(centre)) - rad > reach + 0.02:
        return 0.02
    q = yg.near(link, lambda x: (o_ + x @ R_.T - co) @ Rc, gap, 0.02)
    return min(0.02, float(gap(q).min())) if len(q) else 0.02


def arm_gaps(objects) -> dict:
    """Smallest gap (m) between any arm link (link1 to the fingertips) of either arm and each part."""
    parts = _parts(objects)
    return {p: min((link_gap(parts, k, p) for k in LINK_POSES), default=0.02) for p in parts}


# -- the trial's ledger ----------------------------------------------------------------------------------------------------
class Ledger:
    """One trial: the knocked-off latch, this step's coupling state, hands clear, still, and the hold count."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.knocked = False
        self.c = None
        self.gaps = {}
        self.clear = False
        self.last = None
        self.still = False
        self.good = 0
        self.stable = 0
        self.prev_tier = 0.0
        self.n = 0

    def step(self, objects) -> None:
        self.n += 1
        self.c = c = coupling(objects)
        self.gaps = arm_gaps(objects)
        touched = self.gaps["spider"] <= TOUCH
        if not c["seated"] and not touched and c["s_low"] > FALLEN:
            self.knocked = True
        self.clear = min(self.gaps.values()) > CLEAR
        sp = np.asarray(objects["spider"].pos, float)
        now = (c["gap"], c["turn"], sp, turn_about_y(rotm(objects["spider"].quat_xyzw)))
        self.still = self.last is not None and (
            abs(now[0] - self.last[0]) <= STILL_STEP and abs(wrap(now[1] - self.last[1], 360.0)) <= STILL_DEG
            and float(np.linalg.norm(now[2] - self.last[2])) <= STILL_STEP
            and abs(wrap(now[3] - self.last[3], 360.0)) <= STILL_DEG)
        self.last = now
        t = self.tier()
        done = t == 0.5 and self.at_line() and self.still
        self.good = self.good + 1 if done else 0
        self.stable = self.stable + 1 if (t > 0.0 and t == self.prev_tier and self.still) else 0
        self.prev_tier = t

    def at_line(self) -> bool:
        return LINE_LO <= self.c["gap"] <= LINE_HI

    def tier(self) -> float:
        c = self.c
        if c is None or self.knocked or not c["seated"] or not self.clear or c["gap"] < FORCED:
            return 0.0
        if abs(c["phase"]) > PHASE_DEG:
            return 0.0
        if c["depth"] >= DEEP:
            return 0.5
        if c["depth"] >= ENTERED:
            return 0.25
        return 0.0

    def score(self) -> float:
        s = self.tier()
        if s == 0.5 and self.at_line() and self.good >= HOLD_STEPS:
            return 1.0
        return s if self.stable >= HOLD_STEPS else 0.0  # 0.25 and 0.5 also need the state held, still, for 1 s


_L = None


def _read_links(base) -> None:
    """Every arm link's pose (env frame) at this control step, into LINK_POSES. A missing arm or link, or finger pads
    left without their rubber, stops the trial: the hands-clear checks would otherwise pass silently."""
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError as e:
            raise RuntimeError(f"jaw coupling: no {robot} in the scene for the hands-clear checks") from e
        names = list(art.body_names)
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        missing = [link for link in ARM_LINKS if link not in names]
        if missing:
            raise RuntimeError(f"jaw coupling: {robot} has no link(s) {missing} for the hands-clear checks")
        for link in ARM_LINKS:
            i = names.index(link)
            LINK_POSES[f"{robot}/{link}"] = (P[i] - origin, rotm(Q[i]))
    if gripkit._STATE.get("bound", 0) < 0:
        raise RuntimeError("jaw coupling: the finger pads' rubber material could not be bound")
    if not _READ_ONCE:
        _READ_ONCE.append(1)
        msg = (f"jaw coupling: hands-clear checks on {len(LINK_POSES)} arm links; pad shapes with rubber: "
               f"{gripkit._STATE.get('bound')}")
        print(msg, file=sys.stderr, flush=True)


_READ_ONCE: list = []
kit.STATE_HOOKS.append(_read_links)
if "isaaclab" in sys.modules:  # in the simulator: read and check the hulls as the task loads, not in a trial's step
    yg.warm(ARM_LINKS)


def _update(objects):
    global _L
    if _L is None or _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else ("obj", id(objects))
    if key == _L.key:
        return
    _L.key = key
    _L.step(objects)


def success(objects):
    """The coupling closed to its line with the spider seated and the hands clear, all still: this step counts toward
    the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """1.0: the driven hub turned into phase and slid in until the gap between the hub faces is Lovejoy's G for the
    L-100 (0.750 in, within its 0.040 in allowance), the spider still seated on the reducer hub's jaws, every arm link at
    least 5 mm from the driven hub, the driven shaft and the spider, all still (under 1 cm/s and 0.5 degrees a step) for
    1 s. 0.5: in phase and meshed (the jaws at least 5 mm into the spider's pockets), hands clear, still for 1 s, but
    off the line. 0.25: in phase with the jaws just entered (1 to 5 mm into the pockets), hands clear, still for 1 s.
    0: anything else, including a state still moving or not yet held for 1 s, doing nothing, the jaws pushed square
    onto the spider's legs (out of phase), a hand still on the hub, the shaft or the spider, the spider pulled off or
    knocked off (latched), or the hub closed without the spider."""
    _update(objects)
    return _L.score()


# -- scripted demo: open loop from the fixed layout --------------------------------------------------------------------------
# The right hand does it, as a fitter does with one hand: it comes down over the driven hub with its fingers either side
# (closing across the shafts' axis, along x) and grips the hub's body with the tool point on the axis. It slides the hub
# along its shaft until the jaws meet the spider's legs end-on (out of phase, they stop there) and keeps a light push on
# while it turns the hub back 31.5 degrees by tilting the wrist about the axis (the shaft turns in its bearings with
# it): at the matching phase the jaws drop into the pockets, and it pauses 1 s. It then slides the hub in until its face
# stops against the spider's legs, lets go, backs the hand out along its own axis, lifts clear and goes home. The left
# arm stays at its ready pose.
YAW_X = math.pi / 2  # fingers closing along world x
TILT0_DEG = 0.0  # the wrist's tilt about the axis (world y) when it grips; it turns the hub to TILT0 + TURN_DEG
TURN_DEG = -(START_TURN_DEG + 1.5)  # 1.5 degrees past the pockets' middle: the jaws drop in at the first edge they
# reach coming from +30 (the spider may have settled up to 3 degrees round), and the play takes up the rest
HUB_SQUEEZE = 0.0025  # m a side: the right closes this far past the hub's side
HUB_OPEN = 0.010  # m: the pads stand this far off the hub's side, open
GRIP_BACK = 0.021  # m: the tool point this far behind the hub's face (the pads on the body's back half, clear of the
# spider)
FINGER_HALF_OPEN = 0.047  # m: each finger's inner face from the tool point when fully open
PRESS = 0.0006  # m: aimed this far past contact, so the arm's give presses lightly
VARIANTS = ("", "nothing", "noturn", "short", "hold", "undone")


def finger_for(half: float) -> float:
    """Gripper command whose finger faces stand `half` m from the tool point."""
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * min(FINGER_HALF_OPEN, max(0.0, half)) / FINGER_HALF_OPEN


def hub_tcp(gap: float, up: float = 0.0) -> tuple:
    """The right's tool point on the axis, mid-body of the driven hub with its face `gap` m from the reducer hub's."""
    return (XC, YA - gap - GRIP_BACK, AXIS_Z + up)


def _tw(duration, pos, finger, label, tilt_deg=0.0):
    return kit.Tilted(float(duration), tuple(float(v) for v in pos), float(finger), label, yaw=YAW_X,
                      pitch=math.radians(tilt_deg))


def _t(seq):
    return sum(w.duration for w in seq)


def demo():
    """Right closes the coupling; left stays at its ready pose. JAW_VARIANT=nothing: both arms stay home; noturn: the
    hub is pushed straight in without turning (jaws stopped on the legs); short: turned and pushed in only 9 mm into
    the pockets, then let go; hold: closed but never let go; undone: closed, let go, then pulled back out of the
    spider."""
    variant = os.environ.get("JAW_VARIANT", "")
    if variant not in VARIANTS:
        raise RuntimeError(f"jaw coupling: unknown JAW_VARIANT {variant!r}")
    if variant == "nothing":
        return {"left": [kit.Home(10.0, label="stay at the ready pose")],
                "right": [kit.Home(10.0, label="stay at the ready pose")]}
    hold, open_ = finger_for(R_HUB - HUB_SQUEEZE), finger_for(R_HUB + HUB_OPEN)
    t0, t1 = TILT0_DEG, TILT0_DEG + (0.0 if variant == "noturn" else TURN_DEG)
    touch = MESH_MIN - PRESS  # face gap with the jaws pressed lightly on the legs' ends
    end_gap = MESH_MIN - 0.009 if variant == "short" else SPIDER_L - 0.0004
    right = [kit.Home(0.5, label="start"),
             _tw(3.0, hub_tcp(GAP_START, up=0.070), open_, "over the driven hub", t0),
             _tw(2.5, hub_tcp(GAP_START), open_, "fingers either side of the hub", t0),
             _tw(0.5, hub_tcp(GAP_START), open_, "settle", t0),
             _tw(1.3, hub_tcp(GAP_START), hold, "grip the hub", t0),
             _tw(0.5, hub_tcp(GAP_START), hold, "settle", t0),
             _tw(2.0, hub_tcp(MESH_MIN + 0.002), hold, "slide it up to the spider", t0),
             _tw(1.0, hub_tcp(touch), hold, "jaws against the spider's legs", t0),
             _tw(0.5, hub_tcp(touch), hold, "settle", t0)]
    if variant == "noturn":
        right += [_tw(2.0, hub_tcp(SPIDER_L - 0.0004), hold, "push straight in (no turn)", t0),
                  _tw(1.0, hub_tcp(SPIDER_L - 0.0004), hold, "settle", t0)]
        p_end, tilt = hub_tcp(SPIDER_L - 0.0004), t0
    else:
        right += [_tw(3.0, hub_tcp(touch), hold, "turn it until the jaws drop into the pockets", t1),
                  _tw(1.0, hub_tcp(touch), hold, "settle with the jaws dropped in", t1),
                  _tw(3.0, hub_tcp(end_gap), hold, "slide it in to the spider", t1),
                  _tw(0.8, hub_tcp(end_gap), hold, "settle", t1)]
        p_end, tilt = hub_tcp(end_gap), t1
    if variant == "hold":
        right += [_tw(6.0, p_end, hold, "keep holding the hub", tilt)]
        return {"left": [kit.Home(_t(right), label="stay at the ready pose")], "right": right}
    if variant == "undone":
        right += [_tw(2.0, hub_tcp(MESH_MIN + 0.004), hold, "pull it back out of the spider", tilt),
                  _tw(0.5, hub_tcp(MESH_MIN + 0.004), hold, "settle", tilt)]
        p_end = hub_tcp(MESH_MIN + 0.004)
    back = ry(tilt) @ np.array([0.0, 0.0, 0.045])  # straight back out along the tilted hand's own axis, so the open
    # pads slide off the hub's sides, then up
    p_out = tuple(float(v) for v in np.asarray(p_end) + back)
    right += [_tw(1.2, p_end, open_, "let go", tilt),
              _tw(1.5, p_out, open_, "back the hand out", tilt),
              _tw(1.5, (p_out[0], p_out[1], p_out[2] + 0.020), open_, "lift clear", tilt),
              kit.Home(4.0, label="back to the ready pose"),
              kit.Home(2.5, label="wait")]
    return {"left": [kit.Home(_t(right), label="stay at the ready pose")], "right": right}
