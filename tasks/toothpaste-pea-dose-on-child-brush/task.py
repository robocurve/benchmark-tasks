"""On a bathroom counter (the station table), in front of the arms: a child's manual toothbrush lies on its back,
bristles up, and a kids' fluoride toothpaste tube (a Kid's Crest Bubblegum 4.2 oz tube, partly used) lies on its
side, its screw cap on.

The job is to get the brush ready for a 4-year-old: unscrew the cap, put a pea-sized dab of toothpaste (0.25 g; 0.15
to 0.35 g scores full) on the bristles and nothing off them, then screw the cap back on fully and let go. The US
guidance for ages 3 to 6 is no more than a pea-sized amount, and over 38% of children that age use more (CDC MMWR
2019;68(4):87-90). Too much is the real failure; a tube left open or loosely capped dries out and leaks.

The cap is a free body on a real thread (capkit.py): the neck's male ridge and the cap's female ridge are helices of
convex hulls, so the cap comes off and goes back on only by turning it, about 1.1 turns from seated to free, and it
stays on its thread by contact alone. Paste can leave the nozzle only while the nozzle is out of the cap. The paste
is squeezekit's flow model: the tube's body is four sprung wall panels around a rigid core; a hand dispenses only by
pushing the panels in (read from their joints), and the paste flows through the open nozzle only while the pressure
inside beats its yield pressure, then keeps flowing briefly after the squeeze stops. It drops plumb from the nozzle
onto the bristle field, a 1 mm grid that moves with the brush, or off it.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dataclasses  # noqa: E402

import numpy as np  # noqa: E402

import capkit as ck  # noqa: E402
import gripkit  # noqa: E402
import squeezekit as sq  # noqa: E402
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
INSTRUCTION = "Get the toothbrush ready for a 4-year-old, then leave the toothpaste capped."

# -- the toothbrush (its own frame: origin at the middle of the bristle field's top, x along the brush toward the
#    head's tip, z out of the bristles; lying on its back its frame matches the world's) -------------------------------
BRISTLE = (0.020, 0.009, 0.009)  # m: the bristle field, 20 x 9 mm, 9 mm tall
HEAD = (0.026, 0.012, 0.005)  # the head under the bristles: 26 x 12 x 5 mm
NECK = (0.030, 0.006, 0.005)
HANDLE = (0.095, 0.016, 0.012)  # a chunky child's handle: 95 x 16 x 12 mm
_HEAD_Z = -BRISTLE[2] - HEAD[2] / 2
_FLOOR = -BRISTLE[2] - HEAD[2]  # the brush's underside (lying on its back, all three rest on the counter)
BRUSH_PARTS_BOX = (((0.0, 0.0, -BRISTLE[2] / 2), BRISTLE),
                   ((0.0, 0.0, _HEAD_Z), HEAD),
                   ((-(HEAD[0] + NECK[0]) / 2, 0.0, _FLOOR + NECK[2] / 2), NECK),
                   ((-HEAD[0] / 2 - NECK[0] - HANDLE[0] / 2, 0.0, _FLOOR + HANDLE[2] / 2), HANDLE))
BRUSH_G = 14.0  # g
BRUSH_START = (0.300, 0.020, TOP - _FLOOR)  # the bristle top's middle, 14 mm above the counter, head away from the robot
BRUSH = kit.Prop(
    "brush", pos=BRUSH_START, mass=BRUSH_G / 1000, size=(0.151, HANDLE[1], -_FLOOR),
    meshes=(kit.box(*BRUSH_PARTS_BOX[0], kit.Look((0.95, 0.95, 0.97), 0.8)),
            *(kit.box(c, s, kit.Look((0.30, 0.65, 0.90), 0.5)) for c, s in BRUSH_PARTS_BOX[1:])),
    colliders=tuple(kit.Collider(c, size=s) for c, s in BRUSH_PARTS_BOX))

# -- the tube (its own frame: origin at the middle of the body, x along its axis from the nozzle (-x) to the crimp (+x)).
#    It starts lying on its side, z up -------------------------------------------------------------------------------
BODY_R = 0.018  # m: the round body, 36 mm across (the listing's package is 1.45 in deep)
BODY_X = (-0.050, 0.050)  # 10 cm of body
# The used end, rolled flat up to the paste (as the reset rolls it): a flat roll 55 mm wide and 10 mm thick, 2.4 cm
# long, lying flat on the counter with the round body's underside.
CRIMP = ((0.062, 0.0, -BODY_R + 0.005), (0.024, 0.050, 0.010))  # 50 mm: the listing's package is 2 in wide
X_NECK0 = BODY_X[0] - ck.SHOULDER_L  # where the neck leaves the shoulder
TIP = (X_NECK0 - ck.NECK_L, 0.0, 0.0)  # the nozzle's mouth, at the neck's tip
AXIS_TO_TIP = (-1.0, 0.0, 0.0)
NOZZLE_D = 0.0075  # m: the orifice, 7.5 mm across (generic tubes 0.28-0.32 in, US 9567138)
NOZZLE_L = 0.009
# The neck frame (capkit) in the tube frame: neck z = tube -x, neck x = tube z, neck y = tube y.
M_NECK = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
NECK_ORIGIN = np.array([X_NECK0, 0.0, 0.0])
PANEL_X = 0.015  # the middle of each wall panel along the tube, toward the crimp from the body's middle
PANEL = (0.060, 0.003, 0.020)  # each panel: 6 cm along the tube, 3 mm thick, 2 cm wide
PANEL_TRAVEL = 0.010
_CORE_W = 2 * (BODY_R - PANEL[1] - PANEL_TRAVEL)
CORE = (BODY_X[1] - BODY_X[0], _CORE_W, _CORE_W)
BAND_X = (BODY_X[0], PANEL_X - PANEL[0] / 2 - 0.004)  # the body's stiff front, by the shoulder: one round hull
PANEL_SIGN = {"panel_l": -1.0, "panel_r": 1.0, "panel_t": -1.0, "panel_b": 1.0}  # inward push = sign * joint pos
PASTE_ML = 46.0  # mL left: 60 g of paste of the tube's 4.2 oz (119 g), the middle of the 50-70 g reset fill
AIR_ML = 8.0  # mL of air inside at rest (a laminate tube springs back and draws some in; fitted on the trial tube)
TUBE_G = 12.0  # g: the empty laminate tube, without its cap (weighed on the trial tube, README)
DENSITY = 1.30  # g/mL: a silica toothpaste (pastes run 1.3-1.6, silica low, chalk high); real trials weigh grams
PANEL_KG = 0.0015
TUBE_KG = (TUBE_G + PASTE_ML * DENSITY) / 1000 - 4 * PANEL_KG
K_WALL = 2500.0  # N/m per panel: the laminate wall's spring (sets only the grip force, not the dose)
PANEL_AREA_M2 = PANEL[0] * PANEL[2]
AREA_ML_PER_CM = 1.0  # mL displaced per cm of one panel's push (fitted on the trial tube, README)
TAU_Y_PA = 300.0  # Pa: the paste's yield stress (100 Pa on a model paste, arXiv 1903.10888; ~500 Pa on a
# commercial-style formula, US 5202112): the middle of that range. The demo's dose is checked at 100, 300 and 500 Pa.


def p_yield_kpa(tau_pa: float = TAU_Y_PA) -> float:
    """Pressure (kPa) to push a yield-stress plug through the nozzle: 4 tau L / D."""
    return 4.0 * tau_pa * NOZZLE_L / NOZZLE_D / 1000.0


P_YIELD_KPA = p_yield_kpa()  # 1.44 kPa
FLOW = 0.10  # mL/s per kPa past the yield (fitted on the trial tube, README)
LAMINATE = kit.Look((0.92, 0.92, 0.95), 0.5)
TUBE_LOOK = kit.Look((0.55, 0.30, 0.85), 0.5)  # a purple kids' tube
CAP_LOOK = kit.Look((0.25, 0.75, 0.95), 0.45)
PASTE_RGB = (0.35, 0.75, 0.95)  # blue kids' gel

# Friction (static, dynamic; PhysX averages the two touching surfaces). The cap is polypropylene and the tube's
# laminate and neck polyethylene: polyethylene on polyethylene, clean and dry, 0.2 (Engineering ToolBox, friction
# coefficients), for every surface of the tube and the cap. The YAM's finger pads are rubber-faced: 1.0 / 0.8
# (gripkit), so a pad on the cap or the tube grips at about 0.6 / 0.5.
PLASTIC = (0.20, 0.20, "average")
CONTACT_OFFSET = 0.001  # m: PhysX contact offset of the tube's and the cap's colliders (the thread's gaps are < 1 mm)


def to_tube(p: np.ndarray) -> np.ndarray:
    """Neck-frame points to the tube frame."""
    return NECK_ORIGIN + np.asarray(p, float) @ M_NECK.T


def _axis_x(p: np.ndarray) -> np.ndarray:
    """Points of a shape built about z, laid along the tube's x."""
    p = np.asarray(p, float)
    return np.stack([p[:, 2], p[:, 1], p[:, 0]], axis=1)


class _Wall(sq.Wall):
    """squeezekit's wall spring, with the panel's travel opened only once the spring acts (the first control step).
    The arena settles the scene before its first step with no spring yet: an open panel sags to its stop and throws
    the tube on the first step. Until then the panel is held at its rest by its own joint limits."""

    sets_limits = True

    def __init__(self, bottle, sign, k, area_m2, lo, hi):
        super().__init__(bottle, sign, k=k, area_m2=area_m2)
        object.__setattr__(self, "lim", (lo, hi))

    def drive(self, key, q, qd, J, mem):
        mem["limits"] = self.lim
        return super().drive(key, q, qd, J, mem)


PANEL_INSET = 0.0005  # m: the panels' faces stand this far inside the round body, so the lying tube rests on its stiff
# front and its rolled end, not on a sprung panel


def _panel(link: str, axis: str, side: float):
    off = side * (BODY_R - PANEL_INSET - PANEL[1] / 2)
    anchor = (PANEL_X, off, 0.0) if axis == "Y" else (PANEL_X, 0.0, off)
    size = PANEL if axis == "Y" else (PANEL[0], PANEL[2], PANEL[1])
    lo, hi = (-PANEL_TRAVEL, 0.0) if side > 0 else (0.0, PANEL_TRAVEL)
    return kit.Link(link, "prismatic", axis, anchor=anchor, lower=-1e-4, upper=1e-4, start=0.0, mass=PANEL_KG,
                    meshes=(kit.box((0, 0, 0), size, TUBE_LOOK),), colliders=(kit.Collider((0, 0, 0), size=size),),
                    damping=0.0, law=_Wall("tube", -side, k=K_WALL, area_m2=PANEL_AREA_M2, lo=lo, hi=hi))


_NECK_COLS = ck.colliders(ck.neck_point_sets(BODY_R), to_tube)
_BAND = ck.colliders([_axis_x(np.asarray(ck.prism(BODY_R / math.cos(math.pi / ck.N_SIDES), BAND_X[0], BAND_X[1])))])

# The tube lies on its side along x, the cap toward the robot, the rolled end flat on the counter.
R_LIE = np.eye(3)
TUBE_XY = (0.300, 0.110)
WORK_XY = (0.300, 0.020)  # where the tube is held level while the cap is turned (over the brush, 11.5 cm up)
SPAWN_GAP = 0.0001  # m: the tube spawns this far over the counter, and the neck's tip this far off the cap's seat


def seated_cap_in_tube(gap: float = 0.0):
    """The cap's origin and rotation in the tube frame when screwed fully on (its seat `gap` m off the neck's tip)."""
    return NECK_ORIGIN + M_NECK @ np.array([0.0, 0.0, ck.SEAT_Z - gap]), M_NECK


TUBE_START = (TUBE_XY[0], TUBE_XY[1], TOP + BODY_R + SPAWN_GAP)  # resting on its round front and its rolled end
_NECK_LATHE = [(BODY_R, -ck.SHOULDER_L), (ck.SHOULDER_TOP_R, 0.0), (ck.R_CREST - 0.0002, 0.0),
               (ck.R_CREST - 0.0002, ck.NECK_L - ck.TIP_CHAMFER), (ck.TIP_R, ck.NECK_L), (NOZZLE_D / 2, ck.NECK_L),
               (NOZZLE_D / 2, ck.NECK_L - 0.001)]

TUBE = sq.free_body(kit.Mechanism(
    "tube", pos=TUBE_START, yaw=0.0, mass=TUBE_KG,
    size=(CRIMP[0][0] + CRIMP[1][0] / 2 - TIP[0], CRIMP[1][1], 2 * BODY_R),
    meshes=(kit.box((PANEL_X, 0, 0), CORE, LAMINATE),
            kit.cylinder((0, 0, 0), BODY_R - 0.001, BODY_X[1] - BODY_X[0], TUBE_LOOK, axis="x"),
            kit.box(*CRIMP, TUBE_LOOK),
            kit.lathe(_NECK_LATHE, LAMINATE, center=tuple(float(v) for v in NECK_ORIGIN), axis=(-1.0, 0.0, 0.0), n=40)),
    colliders=(kit.Collider((PANEL_X, 0, 0), size=CORE), kit.Collider(*CRIMP), *_BAND, *_NECK_COLS),
    links=(_panel("panel_l", "Y", 1.0), _panel("panel_r", "Y", -1.0),
           _panel("panel_t", "Z", 1.0), _panel("panel_b", "Z", -1.0)),
    material=PLASTIC, contact_offset=CONTACT_OFFSET),
    com=(0.0, 0.0, 0.0))

# -- the cap: a free body screwed fully on at the start ------------------------------------------------------------------
_CAP_O, _CAP_R = seated_cap_in_tube(SPAWN_GAP)
CAP_START = tuple(float(v) for v in np.asarray(TUBE_START) + R_LIE @ _CAP_O)
CAP_COM, CAP_INERTIA = ck.cap_mass_props()
_CAP_PROFILE = [(0.0, ck.H_CAP), (ck.R_CAP - 0.0015, ck.H_CAP), (ck.R_CAP - 0.0004, ck.H_CAP - 0.0006),
                (ck.R_CAP, ck.H_CAP - 0.0018), (ck.R_CAP, 0.0006), (ck.R_CAP - 0.0006, 0.0),
                (ck.MOUTH_R, 0.0), (ck.R_BORE, ck.LEAD_IN), (ck.R_BORE, ck.H_IN), (0.0, ck.H_IN)]
CAP = kit.Prop(
    "cap", pos=CAP_START, mass=ck.CAP_G / 1000, tilt_xyzw=ck.quat(R_LIE @ _CAP_R), inertia=CAP_INERTIA, com=CAP_COM,
    size=(2 * ck.R_CAP, 2 * ck.R_CAP, ck.H_CAP),
    meshes=(kit.lathe(_CAP_PROFILE, CAP_LOOK, n=48),),
    colliders=ck.colliders(ck.cap_point_sets()),
    material=PLASTIC, contact_offset=CONTACT_OFFSET, max_depenetration=0.1, solver_iterations=(32, 8))

OBJECTS = [BRUSH, TUBE, CAP]
gripkit.use_rubber_pads()  # the YAM's finger pads are rubber-faced

# -- the paste model (squeezekit) and the scored thresholds ---------------------------------------------------------------
# The paste does not drip from an open tube turned nozzle-down (its yield stress holds it), so its own weight is left
# out (neck 0, a huge cross-section). A tube is full of paste up to the nozzle (no air there), so it is fed at the nozzle
# whichever way the nozzle points and a squeeze never vents air.
PASTE = sq.Bottle("tube", tip=TIP, axis=AXIS_TO_TIP, panels=tuple(PANEL_SIGN.items()), capacity=PASTE_ML + AIR_ML,
                  fill=PASTE_ML, area=AREA_ML_PER_CM, cross=1e9, neck=0.0, neck_ml=1.0, p_yield=P_YIELD_KPA,
                  flow=FLOW, air_in=2.0, down_deg=180.0, settle=0.0)
_SEALED = dataclasses.replace(PASTE, p_yield=1e9, air_in=0.0, down_deg=180.0)  # nozzle in the cap: no flow or venting
BEAD_CM = 0.45  # cm: a cell of bristle field holds a 4.5 mm tall bead before it slumps (a pea dab stands 4-5 mm)
BRISTLES = sq.Face("bristles", center=(0.0, 0.0, 0.0), size=BRISTLE[:2], cell=0.001, rest=BEAD_CM)
LAND_DEG = 60.0  # paste lands on the bristles only while they face within 60 degrees of straight up
PEA_G = (0.15, 0.35)  # g on the bristles: a pea-sized dab (0.25 g) for ages 3-6 ...
SMEAR_G = 0.05  # ... at least this much (under 0.15 g) is a smear or rice grain, the under-3 dose ...
OVER_G = 0.70  # ... above 0.35 g is too much (a half load), above 0.70 g a full adult strip
OFF_G = 0.05  # g of paste off the bristles (counter, head, neck, handle) allowed for 0.75 and 1.0
UP_DEG = 45.0  # at the end the bristles face within 45 degrees of straight up
OPEN_CAP = 0.5  # a tube left open or not screwed fully on caps the score here
# Screwed fully on: the cap within SEAT_TURN_DEG of a turn from its seat (its rise up the thread at most that turn's
# share of the pitch), its axis within CAP_OFF of the neck's and CAP_TILT_DEG of parallel.
# The turn is read from the cap's own angle about the neck (0 when seated) and its rise must be under half a pitch, so
# a cap a turn short, or pushed down into the thread's play, is not taken for seated.
SEAT_TURN_DEG = 45.0
SEAT_RISE = ck.PITCH / 2
CAP_OFF = 0.0006
CAP_TILT_DEG = 2.0
# Contact with the arms: their own collision hulls (link3 to the fingertips, both arms) against the tube's and the cap's
# outline. A link touches when one of its surface points lies within TOUCH of the outline (on it or inside); "clear"
# means no point within CLEAR.
TOUCH = 0.001
CLEAR = 0.005
DOWN_GAP = 0.003  # m: the tube (with its cap, if on) and the brush rest on the counter: lowest point within this
NOZZLE_DOWN_DEG = 60.0  # paste lands on the bristles only from a nozzle pointing within 60 degrees of straight down ...
NOZZLE_HIGH = 0.030  # ... and at most 3 cm above the bristles' plane; otherwise it lands off them
# Put down, not dropped or thrown: each time the hands let go of the tube or the brush it must already rest on the
# counter (lowest point within PLACE_GAP of it) and move under PLACE_V.
PLACE_GAP = 0.005
PLACE_V = 0.30  # m/s
FLIP_DEG = 90.0  # the bristles turned past sideways with paste on them: the dab is counted as fallen off
STILL_STEP = 0.0005  # m per control step (1 cm/s)
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
ROBOTS = ("left_robot", "right_robot")
LINK_POSES: dict = {}  # "right_robot/gripper" -> (pos, R) in the env frame at this control step (filled in the sim)


def rotm(q) -> np.ndarray:
    return ck.rotm(q)


def bristles_up_deg(q) -> float:
    """Angle (degrees) between the bristles' facing (the brush's z) and straight up."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rotm(q)[2, 2])))))


# -- the cap on the neck ------------------------------------------------------------------------------------------------
def neck_pose(objects):
    """(origin, rotation) of the neck frame in the world."""
    t = objects["tube"]
    R = rotm(t.quat_xyzw)
    return np.asarray(t.pos, float) + R @ NECK_ORIGIN, R @ M_NECK


def cap_fit(objects) -> dict:
    """The cap on the neck (capkit.relative): rise (m, 0 = seated), off (m), tilt (degrees), turn (degrees)."""
    o, R = neck_pose(objects)
    c = objects["cap"]
    return ck.relative(o, R, np.asarray(c.pos, float), rotm(c.quat_xyzw))


def capped_fully(objects) -> bool:
    """The cap screwed fully on: its turn about the neck within SEAT_TURN_DEG of the seated turn, on the last turn of
    the thread (rise under half a pitch), coaxial and square."""
    f = cap_fit(objects)
    turn = (f["turn"] + 180.0) % 360.0 - 180.0
    return (abs(turn) <= SEAT_TURN_DEG and f["rise"] < SEAT_RISE and f["off"] <= CAP_OFF
            and f["tilt"] <= CAP_TILT_DEG)


def cap_on_neck(objects) -> bool:
    """The cap on the neck at all (anywhere along the thread, roughly coaxial)."""
    f = cap_fit(objects)
    return -0.002 <= f["rise"] <= ck.H_IN and f["off"] <= ck.MOUTH_R and f["tilt"] <= 30.0


def nozzle_covered(objects) -> bool:
    """The nozzle's mouth inside the cap (in its bore, under its top): nothing can leave it."""
    t, c = objects["tube"], objects["cap"]
    tip = np.asarray(t.pos, float) + rotm(t.quat_xyzw) @ np.asarray(TIP)
    p = rotm(c.quat_xyzw).T @ (tip - np.asarray(c.pos, float))
    return -0.001 <= float(p[2]) <= ck.H_IN + 0.0005 and math.hypot(float(p[0]), float(p[1])) <= ck.MOUTH_R


# -- the outlines the arms are checked against ----------------------------------------------------------------------------
def _seg_gap(q, a, b, r):
    """Distance (m, 0 inside) from points q to a flat-ended cylinder from a to b of radius r."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = float(np.linalg.norm(b - a))
    u = (b - a) / L
    d = q - a
    s = d @ u
    radial = np.linalg.norm(d - s[:, None] * u, axis=1)
    return np.hypot(np.maximum(radial - r, 0.0), np.maximum(np.maximum(-s, s - L), 0.0))


def _box_gap(q, c, half):
    d = np.maximum(np.abs(q - np.asarray(c, float)) - np.asarray(half, float), 0.0)
    return np.linalg.norm(d, axis=1)


def tube_gap(qt):
    """Distance from tube-frame points to the tube's outline (body, shoulder and neck, crimp)."""
    g = _seg_gap(qt, (X_NECK0, 0, 0), (BODY_X[1], 0, 0), BODY_R)
    g = np.minimum(g, _seg_gap(qt, (TIP[0], 0, 0), (X_NECK0, 0, 0), ck.R_CREST))
    return np.minimum(g, _box_gap(qt, CRIMP[0], np.asarray(CRIMP[1]) / 2))


def cap_gap(qc):
    """Distance from cap-frame points to the cap's outline."""
    return _seg_gap(qc, (0, 0, 0), (0, 0, ck.H_CAP), ck.R_CAP)


def _parts(objects):
    t, c = objects["tube"], objects["cap"]
    return {"tube": (np.asarray(t.pos, float), rotm(t.quat_xyzw), tube_gap, 0.12),
            "cap": (np.asarray(c.pos, float), rotm(c.quat_xyzw), cap_gap, 0.03)}


_GAP_MEMO: dict = {}


def link_gap(objects, key, part) -> float:
    """Smallest gap (m) between arm link `key` ("right_robot/tip_left") as posed and `part` ("tube" or "cap"); 0.02
    for a link more than 2 cm away."""
    o, R = LINK_POSES[key]
    co, Rc, gap, reach = _parts(objects)[part]
    memo = (key, part, np.asarray(o).tobytes(), np.asarray(R).tobytes(), co.tobytes(), Rc.tobytes())
    if memo in _GAP_MEMO:
        return _GAP_MEMO[memo]
    link = key.split("/")[1]
    _, cen, rad = yg.cloud(link)
    o_, R_ = np.asarray(o, float), np.asarray(R, float)
    centre = (o_ + R_ @ cen - co) @ Rc
    if float(np.linalg.norm(centre)) - rad > reach + 0.02:
        out = 0.02  # the whole link far from the part
    else:
        q = yg.near(link, lambda x: (o_ + x @ R_.T - co) @ Rc, gap, 0.02)
        out = min(0.02, float(gap(q).min())) if len(q) else 0.02
    if len(_GAP_MEMO) > 1024:
        _GAP_MEMO.clear()
    _GAP_MEMO[memo] = out
    return out


def arm_gap(objects, part, robots=ROBOTS) -> float:
    """Smallest gap (m) between any arm link (link3 to the fingertips) of `robots` and `part`; 0.02 when none is
    within 2 cm or there are no link poses."""
    keys = [k for k in LINK_POSES if k.split("/")[0] in robots]
    return min((link_gap(objects, k, part) for k in keys), default=0.02)


def lowest_z(objects) -> float:
    """World height of the lowest point of the tube with its cap (rims of the body, the cap and the crimp's corners)."""
    t, c = objects["tube"], objects["cap"]
    Rt, Rc = rotm(t.quat_xyzw), rotm(c.quat_xyzw)
    a = np.linspace(0, 2 * math.pi, 24, endpoint=False)
    ring = np.stack([np.zeros_like(a), np.cos(a), np.sin(a)], axis=1)
    pts = [np.asarray(t.pos, float) + (np.array([x, 0, 0]) + BODY_R * ring) @ Rt.T for x in (BODY_X[0], BODY_X[1])]
    ringc = np.stack([np.cos(a), np.sin(a), np.zeros_like(a)], axis=1)
    if cap_on_neck(objects):  # the cap counts as part of the tube only while it is on the neck
        pts += [np.asarray(c.pos, float) + (np.array([0, 0, z]) + ck.R_CAP * ringc) @ Rc.T for z in (0.0, ck.H_CAP)]
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * np.asarray(CRIMP[1]) / 2
    pts.append(np.asarray(t.pos, float) + (np.asarray(CRIMP[0]) + corners) @ Rt.T)
    return float(np.min(np.concatenate(pts)[:, 2]))


def brush_low_z(objects) -> float:
    b = objects["brush"]
    R = rotm(b.quat_xyzw)
    pts = []
    for c, s in BRUSH_PARTS_BOX:
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * np.asarray(s) / 2
        pts.append(np.asarray(b.pos, float) + (np.asarray(c) + corners) @ R.T)
    return float(np.min(np.concatenate(pts)[:, 2]))


def brush_gap(objects) -> float:
    """Smallest gap (m) between the arms' links and the brush's colliders (as boxes in the brush's frame)."""
    b = objects["brush"]
    bo, Rb = np.asarray(b.pos, float), rotm(b.quat_xyzw)

    def gap(q):
        return np.min([_box_gap(q, c, np.asarray(s) / 2) for c, s in BRUSH_PARTS_BOX], axis=0)

    best = 0.02
    for key, (o, R) in LINK_POSES.items():
        link = key.split("/")[1]
        _, cen, rad = yg.cloud(link)
        o_, R_ = np.asarray(o, float), np.asarray(R, float)
        if float(np.linalg.norm((o_ + R_ @ cen - bo) @ Rb)) - rad > 0.16:  # the brush reaches 0.14 m from its origin
            continue
        q = yg.near(link, lambda x: (o_ + x @ R_.T - bo) @ Rb, gap, 0.02)
        if len(q):
            best = min(best, float(gap(q).min()))
    return best


# -- the paste ---------------------------------------------------------------------------------------------------------------
class BrushPaste(sq.Sauce):
    """squeezekit's Sauce with its landing face carried by the brush: the nozzle's plumb line is cut with the bristle
    plane and the hit point is given in the brush's frame, so the dab stays where it landed on the bristles wherever
    the brush is held or laid afterwards. With the bristles more than LAND_DEG from facing up, nothing lands on them."""

    def __init__(self):
        super().__init__(PASTE, [BRISTLES])
        self.brush = (np.array(BRUSH_START, float), np.eye(3))

    def tip_world(self, pos, quat_xyzw) -> np.ndarray:
        t = super().tip_world(pos, quat_xyzw)
        bp, r = self.brush
        n = r[:, 2]
        nozzle = rotm(quat_xyzw) @ np.asarray(AXIS_TO_TIP, float)
        if n[2] < math.cos(math.radians(LAND_DEG)) or -nozzle[2] < math.cos(math.radians(NOZZLE_DOWN_DEG)):
            return np.array([1.0, 1.0, -1.0])  # bristles tipped, or the nozzle not pointing down: lands off the field
        s = float(n @ (t - bp)) / float(n[2])  # height of the nozzle above the bristle plane, plumb
        if s > NOZZLE_HIGH:
            return np.array([1.0, 1.0, -1.0])  # squeezed from high up: the ribbon strings and drifts, lands off
        local = r.T @ (t - np.array([0.0, 0.0, s]) - bp)
        return np.array([local[0], local[1], s])

    def step(self, pos, quat_xyzw, deflections, dt: float = DT, sub: int = 10, open_: bool = True) -> float:
        """squeezekit's step; with the nozzle in the cap it is sealed (pressure builds, nothing leaves)."""
        if open_:
            return super().step(pos, quat_xyzw, deflections, dt, sub)
        b, self.b = self.b, _SEALED
        try:
            return super().step(pos, quat_xyzw, deflections, dt, sub)
        finally:
            self.b = b

    def on_g(self) -> float:
        return self.on("bristles") * DENSITY

    def off_g(self) -> float:
        return self.off_target() * DENSITY


def pushes(objects) -> list:
    """Each wall panel's inward push (m), read from its joint: what the hand did to the tube."""
    out = []
    for link, sign in PANEL_SIGN.items():
        try:
            out.append(sign * kit.joint(objects, f"tube/{link}").pos)
        except KeyError:  # no joint state (not in the sim): no squeeze
            out.append(0.0)
    return out


def dose_tier(on_g: float, off_g: float) -> float:
    """The paste part of the score from the grams on the bristles and off them."""
    if on_g > OVER_G + 1e-12 or on_g < SMEAR_G - 1e-12:
        return 0.0
    if not PEA_G[0] - 1e-12 <= on_g <= PEA_G[1] + 1e-12:
        return 0.25  # a smear (under 0.15 g) or too much (0.35 to 0.70 g)
    return 0.75 if off_g <= OFF_G + 1e-12 else 0.5


class Ledger:
    """One trial: the paste, the cap, the brush's flip latch, the let-go checks and the still/hold count."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.paste = BrushPaste()
        self.flipped = False
        self.last = None
        self.good = 0
        self.held = {"brush": False, "tube": False}
        self.placed = {"brush": True, "tube": True}  # each was last let go resting on the counter (both start there)
        self.capped = True
        self.clear = False
        self.n = 0

    def step(self, objects) -> None:
        self.n += 1
        b, t, c = objects["brush"], objects["tube"], objects["cap"]
        bp, bq = np.asarray(b.pos, float), np.asarray(b.quat_xyzw, float)
        self.paste.brush = (bp, rotm(bq))
        sq.register(self.paste)
        covered = nozzle_covered(objects)
        self.paste.step(t.pos, t.quat_xyzw, pushes(objects), dt=DT, open_=not covered)
        up = bristles_up_deg(bq)
        if up > FLIP_DEG and self.paste.on_g() > 0.0:
            self.flipped = True
        tp, cp = np.asarray(t.pos, float), np.asarray(c.pos, float)
        still = self.last is not None and max(float(np.linalg.norm(bp - self.last[0])),
                                              float(np.linalg.norm(tp - self.last[1])),
                                              float(np.linalg.norm(cp - self.last[2]))) <= STILL_STEP
        gaps = {part: arm_gap(objects, part) for part in ("tube", "cap")}
        bgap = brush_gap(objects)
        low = lowest_z(objects)
        for name, held, p, prev, rest in (
                ("brush", bgap <= TOUCH, bp, None if self.last is None else self.last[0],
                 brush_low_z(objects) - TOP),
                ("tube", min(gaps.values()) <= TOUCH, tp, None if self.last is None else self.last[1], low - TOP)):
            if self.held[name] and not held:  # let go this step: laid down, or dropped or thrown?
                v = 0.0 if prev is None else float(np.linalg.norm(p - prev)) / DT
                self.placed[name] = -0.01 <= rest <= PLACE_GAP and v <= PLACE_V
            self.held[name] = held
        self.capped = capped_fully(objects)
        self.clear = min(min(gaps.values()), bgap) > CLEAR
        self.last = (bp, tp, cp)
        brush_rest = brush_low_z(objects) - TOP
        done = (self.tier() == 0.75 and self.capped and self.clear and up <= UP_DEG and -0.01 <= low - TOP <= DOWN_GAP
                and -0.01 <= brush_rest <= DOWN_GAP and not self.held["brush"] and self.placed["tube"]
                and self.placed["brush"] and still)
        self.good = self.good + 1 if done else 0
        if _PROBE:
            f = cap_fit(objects)
            fing = FINGERS.get("right_robot", (float("nan"),) * 2)
            with open(_PROBE, "a") as fh:
                fh.write("%d rise %.5f off %.5f tilt %.2f turn %.1f cov %d p %.3f on %.4f off %.4f up %.1f low %.4f "
                         "gt %.4f gc %.4f clr %d cap %s tube %s tq %s R %s Rq %s fing %.4f %.4f P %s\n" % (
                             self.n, f["rise"], f["off"], f["tilt"], f["turn"], int(covered), self.paste.p,
                             self.paste.on_g(), self.paste.off_g(), up, low - TOP, gaps["tube"], gaps["cap"],
                             int(self.clear), " ".join("%.4f" % v for v in cp), " ".join("%.4f" % v for v in tp),
                             " ".join("%.3f" % v for v in t.quat_xyzw),
                             " ".join("%.4f" % v for v in objects["right_gripper"].pos),
                             " ".join("%.3f" % v for v in objects["right_gripper"].quat_xyzw), fing[0], fing[1],
                             " ".join("%.5f" % v for v in pushes(objects))))

    def tier(self) -> float:
        t = dose_tier(self.paste.on_g(), self.paste.off_g())
        return min(t, 0.25) if self.flipped else t

    def score(self) -> float:
        s = self.tier()
        if not self.capped:
            return min(s, OPEN_CAP)
        if s == 0.75 and self.good >= HOLD_STEPS:
            return 1.0
        return s


_L = None
_PROBE = os.environ.get("TOOTH_PROBE")
FINGERS: dict = {}  # "right_robot" -> (joint7, joint8), for the probe log


def _read_links(base) -> None:
    """Every arm link's pose (env frame) at this control step, into LINK_POSES (and the finger joints, for the log).
    A missing arm or link, or finger pads left without their rubber, stops the trial: the contact checks and the grip
    would otherwise pass silently."""
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError as e:
            raise RuntimeError(f"toothpaste: no {robot} in the scene for the contact checks") from e
        names = list(art.body_names)
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        missing = [link for link in ARM_LINKS if link not in names]
        if missing:
            raise RuntimeError(f"toothpaste: {robot} has no link(s) {missing} for the contact checks")
        for link in ARM_LINKS:
            i = names.index(link)
            LINK_POSES[f"{robot}/{link}"] = (P[i] - origin, rotm(Q[i]))
        jn = list(art.joint_names)
        q = getattr(art.data.joint_pos, "torch", art.data.joint_pos)[0].detach().cpu().numpy()
        FINGERS[robot] = (float(q[jn.index("joint7")]), float(q[jn.index("joint8")]))
    if gripkit._STATE.get("bound", 0) < 0:
        raise RuntimeError("toothpaste: the finger pads' rubber material could not be bound")
    if not _READ_ONCE:
        _READ_ONCE.append(1)
        print(f"toothpaste: contact checks on {len(LINK_POSES)} arm links; pad shapes with rubber: "
              f"{gripkit._STATE.get('bound')}", file=sys.stderr, flush=True)


_READ_ONCE: list = []


kit.STATE_HOOKS.append(_read_links)
if "isaaclab" in sys.modules:  # in the simulator: read and check the hulls as the task loads, not in a trial's step
    yg.warm(ARM_LINKS)


# -- the cap starts screwed on wherever the tube starts --------------------------------------------------------------------
# The arena may shift each free object on the counter by a random few cm at reset (object_jitter). The cap is its own
# body, so at the first physics step of every settle (after any shift) it is put back screwed on under the tube, at
# rest. This is the starting layout, written once per trial before anything moves.
_RESET = {"pending": False, "depth": 0}


def _seat_cap(scene) -> None:
    import torch

    try:
        tube, cap = scene["tube"], scene["cap"]
    except KeyError:
        return
    tp = getattr(tube.data.root_link_pose_w, "torch", tube.data.root_link_pose_w)[0].detach().cpu().numpy()
    tp = tp.astype(float)
    o, R = seated_cap_in_tube(SPAWN_GAP)
    Rt = rotm(tp[3:7])
    pos = tp[:3] + Rt @ o
    q = ck.quat(Rt @ R)
    dev = getattr(cap.data.root_link_pose_w, "torch", cap.data.root_link_pose_w).device
    pose = torch.tensor([[*pos, *q]], dtype=torch.float32, device=dev)
    cap.write_root_pose_to_sim_index(root_pose=pose)
    cap.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=dev))


def _patch_resets() -> None:
    try:
        from isaaclab.envs import ManagerBasedEnv
        from isaaclab.scene import InteractiveScene
    except ImportError:  # offline
        return
    if getattr(InteractiveScene, "_toothpaste_cap", False):
        return
    InteractiveScene._toothpaste_cap = True
    write, reset_scene, reset_env = InteractiveScene.write_data_to_sim, InteractiveScene.reset, ManagerBasedEnv.reset

    def write_data_to_sim(self):
        if _RESET["pending"] and _RESET["depth"] == 0:  # the settle's first physics step, after any jitter
            _RESET["pending"] = False
            _seat_cap(self)
        return write(self)

    def scene_reset(self, env_ids=None):
        out = reset_scene(self, env_ids)
        _RESET["pending"] = True
        return out

    def env_reset(self, *a, **k):
        _RESET["depth"] += 1
        try:
            return reset_env(self, *a, **k)
        finally:
            _RESET["depth"] -= 1

    InteractiveScene.write_data_to_sim, InteractiveScene.reset, ManagerBasedEnv.reset = (
        write_data_to_sim, scene_reset, env_reset)


if "isaaclab" in sys.modules:
    _patch_resets()


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
    _TILES.draw(objects, _L)


def success(objects):
    """A pea-sized dab on the bristles, nothing off them, the cap screwed fully on, the tube resting on the counter with
    no hand on it or its cap, nothing dropped or thrown, all still: this step counts toward the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """Grams on the bristles (mL x 1.30 g/mL): 0.15 to 0.35 g with at most 0.05 g off the bristles scores 0.75, with
    more off them 0.5; 0.05 to 0.15 g (a smear) or 0.35 to 0.70 g (too much) 0.25; under 0.05 g or over 0.70 g 0. A
    cap not screwed fully on caps the score at 0.5. 1.0: the 0.75 state with the cap screwed fully on, the tube resting
    on the counter with both arms at least 5 mm clear of it and its cap, the bristles within 45 degrees of facing up,
    the tube and brush each let go resting on the counter, all still (under 1 cm/s) for 1 s. Turning the bristles past
    sideways with paste on them caps the score at 0.25. Doing nothing scores 0."""
    _update(objects)
    return _L.score()


# -- the paste the cameras see ---------------------------------------------------------------------------------------------
class _Tiles(sq.Tiles):
    """The dab drawn on the bristles as tiles in the brush's own frame (turned and carried with it every step), with
    Isaac Lab's VisualizationMarkers; the env origin is read once from the brush's prim. Only a picture: on any error
    it turns itself off and scoring carries on."""

    def __init__(self):
        super().__init__(color=PASTE_RGB, min_ml=2e-5)
        self.origin = None

    def draw(self, objects, ledger):  # noqa: D102  (overrides squeezekit's world-fixed draw)
        if self.off:
            return
        if self.origin is None:
            try:
                import omni.usd
                from pxr import UsdGeom

                stage, cache = omni.usd.get_context().get_stage(), UsdGeom.XformCache()
                self.origin = np.zeros(3)
                for prim in stage.Traverse():
                    if prim.GetName() == "brush":
                        w = np.array(cache.GetLocalToWorldTransform(prim).ExtractTranslation(), float)
                        self.origin = w - np.asarray(objects["brush"].pos, float)
                        break
            except Exception:  # noqa: BLE001 (no simulator: unit tests)
                self.off = True
                return
        g = ledger.paste.grids["bristles"]
        bp, r = ledger.paste.brush
        xs, ys = g.centers()
        ii, jj = np.nonzero(g.v > self.min_ml)
        tr, sc = [], []
        for i, j in zip(ii, jj):
            h = float(g.v[i, j]) / (g.f.cell * 100.0) ** 2 / 100.0  # m
            tr.append(bp + r @ np.array([xs[i], ys[j], h / 2]))
            sc.append((g.f.cell, g.f.cell, max(h, 2e-4)))
        sig = (len(tr), round(float(np.sum(sc)) if sc else 0.0, 7), tuple(np.round(bp, 4)),
               tuple(np.round(r[:, 2], 3)))
        if sig == self.shown:
            return
        try:
            if self.markers is None:
                if not tr:
                    return
                import isaaclab.sim as sim_utils
                from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

                proto = sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=tuple(sq._linear(x) for x in self.color), roughness=0.5))
                self.markers = VisualizationMarkers(VisualizationMarkersCfg(prim_path=self.path,
                                                                            markers={"paste": proto}))
            self.shown = sig
            if not tr:
                self.markers.set_visibility(False)
                return
            q = np.asarray(sq.quat(r), np.float32)
            self.markers.set_visibility(True)
            self.markers.visualize(translations=np.array(tr, np.float32) + np.asarray(self.origin, np.float32),
                                   orientations=np.tile(q, (len(tr), 1)), scales=np.array(sc, np.float32),
                                   marker_indices=np.zeros(len(tr), int))
        except Exception as e:  # noqa: BLE001 (cosmetic only)
            self.off = True
            print(f"toothpaste: tiles off ({e!r}); scoring is unaffected", file=sys.stderr, flush=True)


_TILES = _Tiles()

# -- scripted demo: open loop from the fixed layout --------------------------------------------------------------------------
# Two hands, as a person does it: the left hand holds the tube and the right turns the cap. The left takes the lying
# tube from above, lightly across its body (a holding push on two wall panels, under the paste's yield), and lifts it
# level, the cap toward the robot. The right comes in pointing forward, closes on the cap and unscrews it
# counterclockwise (seen from the cap's top) half a turn per wrist stroke, following the thread and aiming a little into
# it; between strokes it opens, turns back and closes again. Once the thread is free it draws the cap straight off and
# holds it. The left turns the tube nozzle-down over the brush lying on the counter, the nozzle 4 mm above the
# bristles, squeezes 1.75 mm a side for 1.2 s, eases to the holding grip, waits 1.5 s for the flow to die, lifts off and
# brings the tube back level. The right puts the cap back on the neck, pressing it on lightly, and screws it on
# clockwise in the same strokes, a little past the seat (the grips slip once it is tight), lets go and backs away; the
# left lays the capped tube down, lets go and lifts clear.
HOLD_PUSH = 0.0004  # m a side: holds the tube (2.2 N a side, 1.0 kPa inside, under the 1.44 kPa yield)
FIRM_PUSH = 0.0006  # m a side: a firmer hold while the cap is on and seals the nozzle (nothing can flow); eased to the
# light hold before the cap comes free. Squeezed much harder, the narrow sprung wall panels let the tube squirt out of the
# pinch
DOSE_PUSH = 0.00175  # m a side: the dosing squeeze (a pea-sized dose in the offline model at 300 Pa)
DOSE_S = 1.2  # s held
VARIANTS = {"much": (0.0025, 1.5), "smear": (0.0012, 0.6)}  # TOOTH_VARIANT checks of the scorer in the simulator
TAIL_S = 1.5  # s for the flow to die after easing off
FINGER_HALF_OPEN = 0.047  # m: each finger's inner face from the tool point when fully open
NOZZLE_GAP = 0.004  # m: the nozzle's mouth over the bristle top while squeezing
GRIP_X = 0.0  # m: the left holds the tube across its center of mass (the body's middle): a two-finger pinch turns
# freely about its closing line, so an offset lets the tube's weight swing it (25 mm off swings it tens of degrees)
WORK_Z = TOP + 0.130  # height of the tube's axis while the cap is turned
HELD_DZ = -0.0019  # m: the held tube sits this much under the left's command (the arm gives under its load); the
# right aims at the cap where it really is, so its fingers close on it centered
CAP_GRIP_Z = 0.010  # m: the right's tool point on the cap's axis, this far in from its mouth (the fingertips then stop
# 3 mm short of the mouth, clear of the tube's shoulder)
CAP_SQUEEZE = 0.0010  # m a side: the right closes this far past the cap's side (about 5 N a finger on the rigid cap)
CAP_OPEN = 0.010  # m: the right's pads stand this far off the cap's side when open between strokes
YAW_LO, YAW_HI = -math.pi / 2, math.pi / 2  # the right wrist's stroke about its pointing axis
UNSCREW_DEG = 450.0  # turns from seated to past free (capkit's thread frees after about 400 degrees)
SCREW_DEG = UNSCREW_DEG + 30.0  # screwing back: to the seat and a little past (the grips slip once it is tight)
PRESS = 0.0001  # m: each stroke aims the cap this far into the thread (the arm's give makes a light push)
PUSH_ON = 0.003  # m: screwing back on, the gripped cap is aimed this far past where the thread puts it, so the arms'
# give presses it lightly onto the neck while it turns (as a hand does). The arms' joint friction leaves their tool
# points a millimetre or so off command (seen: 1.1 mm), more than a pitch's worth of aim, so a cap aimed only at the
# thread can hang just off the neck and turn in air
STEP_DEG = 90.0  # waypoints along a stroke (each a smooth start and stop)
STROKE_S = 3.5  # s per half turn
DRAW_OFF = 0.025  # m: the cap drawn straight back off the neck once free
CAP_PARK = (0.240, -0.140, TOP + 0.160)  # where the right holds the cap while the left doses (on the right's own side)
K_FINGER = 5000.0  # N/m: the YAM finger's position servo (yam_arena GRIPPER stiffness), in series with the wall
PITCH_FWD = -math.pi / 2  # a hand pointing forward (+x)


def finger_for(half: float) -> float:
    """Gripper command whose finger faces stand `half` m from the tool point."""
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * min(FINGER_HALF_OPEN, max(0.0, half)) / FINGER_HALF_OPEN


def push_force(push: float) -> float:
    """N on one wall panel pushed in `push` m, both opposite panels pushed alike: its spring plus the air's pressure."""
    ml = 2 * AREA_ML_PER_CM * push * 100.0
    p = sq.P_ATM * (AIR_ML / max(1e-3, AIR_ML - ml) - 1.0)
    return K_WALL * push + p * 1000.0 * PANEL_AREA_M2


def tube_grip(push: float) -> float:
    """Gripper command that pushes each gripped panel `push` m in: the finger's servo is a spring in series with the
    panel, so it is commanded past the panel by the force it must make over its stiffness."""
    return finger_for(BODY_R - PANEL_INSET - push - push_force(push) / K_FINGER)


def work_tube():
    """The tube's pose (origin, rotation) held level at the work height, where it was picked up."""
    return np.array([WORK_XY[0], WORK_XY[1], WORK_Z]), R_LIE


def cap_tcp(turned_deg: float, back: float = 0.0) -> tuple:
    """The right's tool point on the cap's axis with the cap turned `turned_deg` off its seat along the thread (pushed
    PRESS into it), drawn `back` m further off along the axis, the tube at its work pose."""
    o, R = work_tube()
    co, cR = seated_cap_in_tube(0.0)
    axis = R @ cR[:, 2]  # the cap's axis, out of its top (world -x)
    p = o + R @ co + axis * (CAP_GRIP_Z + ck.PITCH * turned_deg / 360.0 - PRESS + back) + np.array([0.0, 0.0, HELD_DZ])
    return tuple(float(v) for v in p)


def left_tcp_level(o, R) -> tuple:
    return tuple(float(v) for v in np.asarray(o) + R @ np.array([GRIP_X, 0.0, 0.0]))


def _tw(duration, pos, finger, label, yaw=0.0, pitch=0.0):
    return kit.Tilted(float(duration), tuple(float(v) for v in pos), float(finger), label, yaw=float(yaw),
                      pitch=float(pitch))


def strokes(total_deg: float, sign: float, start_yaw: float, turned0: float, hold: float, label: str,
            push: float = 0.0) -> tuple:
    """The right's waypoints that turn the cap `total_deg` (sign +1 unscrews: counterclockwise seen from its top) in
    wrist strokes between YAW_LO and YAW_HI, following the thread, regripping between strokes; and the yaw and the
    cap's turn at the end (still gripped). With `push` (m) the gripped cap is aimed that far onto the neck (it opens
    and closes on the cap where the thread puts it, then presses)."""
    out, yaw, turned, left, k = [], start_yaw, turned0, total_deg, 0
    open_ = finger_for(ck.R_CAP + CAP_OPEN)
    while left > 1e-6:
        room = math.degrees(YAW_HI - yaw) if sign > 0 else math.degrees(yaw - YAW_LO)
        if room < 1.0:  # regrip: open, turn back half a turn clear of the cap, close
            p = cap_tcp(max(0.0, turned))
            out.append(_tw(1.0, p, open_, f"{label}: open", yaw, PITCH_FWD))
            for _ in range(2):  # two quarter turns, so the wrist turns back the way it came
                yaw -= sign * math.pi / 2
                out.append(_tw(1.6, p, open_, f"{label}: turn back half a turn", yaw, PITCH_FWD))
            out += [_tw(1.3, p, hold, f"{label}: grip again", yaw, PITCH_FWD),
                    _tw(0.3, p, hold, f"{label}: settle", yaw, PITCH_FWD)]
            if push > 0.0:
                out.append(_tw(0.5, cap_tcp(max(0.0, turned), back=-push), hold, f"{label}: press it on", yaw,
                               PITCH_FWD))
            continue
        k += 1
        d = min(left, room)
        n = max(1, int(math.ceil(d / STEP_DEG)))
        for i in range(1, n + 1):
            yaw_i = yaw + sign * math.radians(d * i / n)
            t_i = turned + sign * d * i / n
            out.append(_tw(STROKE_S * d / 180.0 / n, cap_tcp(max(0.0, t_i), back=-push), hold, f"{label}: stroke {k}",
                           yaw_i, PITCH_FWD))
        yaw += sign * math.radians(d)
        turned += sign * d
        left -= d
    return out, yaw, turned


def _hold(seq, duration, label):
    """Repeat a sequence's last waypoint for `duration` s (waiting for the other hand)."""
    if duration > 1e-6:
        w = seq[-1]
        if isinstance(w, kit.Home):
            seq.append(kit.Home(float(duration), label=label))
        else:
            seq.append(kit.Tilted(float(duration), w.pos, w.finger, label, yaw=w.yaw, roll=w.roll, pitch=w.pitch))


def _t(seq):
    return sum(w.duration for w in seq)


def demo():
    """Left holds the tube, right unscrews the cap; left doses; right screws the cap back on; left lays the tube down.
    (TOOTH_VARIANT=nothing: both arms stay at their ready pose: the scorer's do-nothing check.)"""
    if os.environ.get("TOOTH_VARIANT") == "nothing":
        return {"left": [kit.Home(10.0, label="stay at the ready pose")],
                "right": [kit.Home(10.0, label="stay at the ready pose")]}
    push, dose_s = VARIANTS.get(os.environ.get("TOOTH_VARIANT", ""), (DOSE_PUSH, DOSE_S))
    hold, dose, firm = tube_grip(HOLD_PUSH), tube_grip(push), tube_grip(FIRM_PUSH)
    l_open = finger_for(FINGER_HALF_OPEN)
    cap_hold, cap_open = finger_for(ck.R_CAP - CAP_SQUEEZE), finger_for(ck.R_CAP + CAP_OPEN)
    o0 = np.asarray(TUBE_START, float)
    pick = left_tcp_level(o0 - np.array([0.0, 0.0, SPAWN_GAP]), R_LIE)
    work = left_tcp_level(*work_tube())
    # left: take the tube and hold it level at the work height
    left = [_tw(3.0, (pick[0], pick[1], pick[2] + 0.10), l_open, "over the tube"),
            _tw(2.0, pick, l_open, "fingers either side of the body"),
            _tw(0.5, pick, l_open, "settle"),
            _tw(1.3, pick, hold, "grip the body"),
            _tw(0.3, pick, hold, "settle"),
            _tw(2.0, work, hold, "lift it level, cap toward the robot"),
            _tw(1.0, work, hold, "hold it")]
    # right: in to the cap once the tube is held still, grip, unscrew, draw it off
    right = [kit.Home(max(0.5, _t(left) - 1.5), label="wait at the ready pose"),
             _tw(3.0, cap_tcp(0.0, back=0.05), cap_open, "pointing at the cap", YAW_LO, PITCH_FWD),
             _tw(2.0, cap_tcp(0.0), cap_open, "fingers either side of the cap", YAW_LO, PITCH_FWD),
             _tw(0.5, cap_tcp(0.0), cap_open, "settle", YAW_LO, PITCH_FWD),
             _tw(2.0, cap_tcp(0.0), cap_hold, "grip the cap", YAW_LO, PITCH_FWD),
             _tw(0.5, cap_tcp(0.0), cap_hold, "settle", YAW_LO, PITCH_FWD)]
    # the left holds lightly while the right closes (its give takes up any misfit between the two hands), then firmly
    # for the turning, once the cap is gripped
    _hold(left, _t(right) - _t(left), "hold it lightly while the cap is gripped")
    left += [_tw(1.5, work, firm, "hold it firmly (the cap seals the nozzle)"), _tw(0.5, work, firm, "settle")]
    _hold(right, _t(left) - _t(right), "wait for the firm hold")
    wps, yaw, turned = strokes(UNSCREW_DEG, +1.0, YAW_LO, 0.0, cap_hold, "unscrew")
    opens = [i for i, w in enumerate(wps) if w.label == "unscrew: open"]
    t_ease = _t(right) + sum(w.duration for w in wps[:opens[-1]])  # the regrip before the stroke that frees the cap
    right += wps
    _hold(left, t_ease - _t(left), "hold it firmly while the cap is turned")
    left.append(_tw(0.8, work, hold, "ease to a light hold before the cap comes free"))
    right.append(_tw(1.5, cap_tcp(turned, back=DRAW_OFF), cap_hold, "draw the cap off", yaw, PITCH_FWD))
    right.append(_tw(2.5, CAP_PARK, cap_hold, "hold the cap aside, out of the left arm's way", yaw, PITCH_FWD))
    _hold(left, _t(right) - _t(left), "hold it while the cap comes off")
    # left: nozzle-down over the bristles, dose, back level
    bx, by, bz = BRUSH_START
    r_dose = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])  # the hand pointing forward: nozzle down
    at_o = np.array([bx, by, bz + NOZZLE_GAP]) - r_dose @ np.asarray(TIP)
    at = left_tcp_level(at_o, r_dose)
    over = (at[0], at[1], at[2] + 0.040)
    high_level = (work[0], work[1], TOP + 0.15)
    high_dose = (at[0], at[1], TOP + 0.15)
    left += [_tw(4.0, high_level, hold, "turn it nozzle-down", pitch=PITCH_FWD),
             _tw(2.0, high_dose, hold, "over the brush", pitch=PITCH_FWD),
             _tw(1.5, over, hold, "nozzle 4 cm over the bristles", pitch=PITCH_FWD),
             _tw(1.2, at, hold, "nozzle 4 mm over the bristles", pitch=PITCH_FWD),
             _tw(0.5, at, hold, "settle", pitch=PITCH_FWD),
             _tw(0.3, at, dose, "squeeze", pitch=PITCH_FWD),
             _tw(dose_s, at, dose, "hold the squeeze", pitch=PITCH_FWD),
             _tw(0.3, at, hold, "ease off", pitch=PITCH_FWD),
             _tw(TAIL_S, at, hold, "let the flow die", pitch=PITCH_FWD),
             _tw(1.0, over, hold, "lift off the dab", pitch=PITCH_FWD),
             _tw(1.5, high_dose, hold, "up", pitch=PITCH_FWD),
             _tw(2.0, high_level, hold, "back over its place", pitch=PITCH_FWD),
             _tw(4.0, work, hold, "turn it level at the work height"),
             _tw(0.5, work, hold, "hold it")]
    _hold(right, _t(left) - _t(right), "hold the cap while the tube is away")
    # right: cap back on the neck, screwed on
    right += [_tw(2.5, cap_tcp(turned, back=DRAW_OFF), cap_hold, "back in front of the neck", yaw, PITCH_FWD),
              _tw(1.5, cap_tcp(turned, back=0.003), cap_hold, "cap to the neck", yaw, PITCH_FWD),
              _tw(1.0, cap_tcp(turned, back=-PUSH_ON), cap_hold, "onto the neck, pressed lightly", yaw, PITCH_FWD),
              _tw(0.3, cap_tcp(turned, back=-PUSH_ON), cap_hold, "settle", yaw, PITCH_FWD)]
    _hold(left, _t(right) - _t(left), "hold it while the cap goes back on")
    left.append(_tw(0.8, work, firm, "hold it firmly again (the cap covers the nozzle)"))
    _hold(right, _t(left) - _t(right), "settle")
    wps, yaw, turned = strokes(SCREW_DEG, -1.0, yaw, turned, cap_hold, "screw on", push=PUSH_ON)
    right += wps
    p_end = cap_tcp(max(0.0, turned))
    right += [_tw(1.0, p_end, cap_open, "let go of the cap", yaw, PITCH_FWD),
              _tw(1.5, (p_end[0] - 0.05, p_end[1], p_end[2]), cap_open, "back away", yaw, PITCH_FWD),
              kit.Home(6.0, label="back to the ready pose")]
    _hold(left, _t(right) - 6.0 - _t(left), "hold it while the right lets go")
    # left: lay the capped tube down, let go, lift clear
    down = left_tcp_level(o0 + np.array([0.0, 0.0, 0.001 - SPAWN_GAP]), R_LIE)
    left += [_tw(2.0, (down[0], down[1], down[2] + 0.02), hold, "down to the counter"),
             _tw(1.0, down, hold, "lay it down"),
             _tw(0.3, down, hold, "settle"),
             _tw(1.0, down, l_open, "let go"),
             _tw(1.5, (down[0], down[1], down[2] + 0.08), l_open, "lift clear"),
             kit.Home(2.5, label="back to the ready pose"),
             kit.Home(3.0, label="wait")]
    _hold(right, _t(left) - _t(right), "wait")
    return {"left": left, "right": right}
