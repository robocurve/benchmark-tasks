"""A wall oxygen flowmeter stands on a plywood mock wall at the back of the table, its DISS oxygen outlet (a 9/16-18
male thread) pointing straight down. In front of it, to the right, a filled bubble humidifier (an AirLife / Salter
Labs 7600, 350 cc, filled to its 290 mL maximum line) stands on the table, its quad wing nut on top.

The job is to fit the humidifier onto the flowmeter: lift the bottle under the outlet, start the wing nut square on
the thread and turn it on until it is seated hand-tight, then let go so the bottle hangs from the outlet, upright.
A nut started crooked cross-threads (the bottle then leaks oxygen out of the top of the bottle, Apria's caution); a
bottle dropped or tipped far enough that water reaches its gas outlet fails the trial.

The nut is a real thread (threadkit.py): the outlet's male ridge and the nut's female ridge are helices of convex
hulls with the Unified 60-degree section, so the nut goes on only by turning it, about two and a half turns from the first
catch to the seat, and the bottle stays on the outlet by thread contact alone. The nut is a captive swivel on the lid's
tower (a revolute joint to the bottle, as on the real bottle: it turns without turning the bottle). The water is a
rigid 290 g mass at the jar's bottom (its center of mass and inertia authored), not a fluid.
"""
import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import gripkit  # noqa: E402
import threadkit as tk  # noqa: E402
import yamgeom as yg  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 3600
INSTRUCTION = "Put the humidifier bottle on the oxygen flowmeter."

# -- the flowmeter and its outlet (the outlet frame: origin on the outlet's axis at the flat underside of its hex
#    body, z straight down along the stub) -------------------------------------------------------------------------
OUTLET_XY = (0.360, 0.000)
HANG_GAP = 0.040  # m: the hung bottle's bottom stands this far above the table
JAR_H = 0.125  # the jar: 125 mm tall ...
JAR_R = 0.033  # ... 66 mm across
JAR_WALL = 0.002
LID_R = 0.0413  # the lid (3.25 in across: the listing's 3.25 x 7.00 in) ...
LID_SKIRT = (0.120, 0.135)  # ... its threaded skirt over the jar's mouth, then its dome
LID_TOP = 0.150
TOWER_R, TOWER_TOP = 0.008, 0.165  # the connecting tower the nut swivels on
NUT_MOUTH = TOWER_TOP + tk.NUT_H  # the nut's mouth: 178 mm (7.00 in) up from the bottle's bottom
Z_UNDER = TOP + HANG_GAP + NUT_MOUTH + tk.SEAT_Z  # the outlet's underside (world z)
HEX_R, HEX_H = 0.0110, 0.016  # the outlet's hex body under the flowmeter (22 mm across), 16 mm tall
BODY = (0.090, 0.060, 0.180)  # the flowmeter's body over it (a 0-15 L/min Thorpe-tube flowmeter with its knob)
BOARD_X = 0.080  # the mock wall's front face, behind the outlet's axis
BOARD = (0.018, 0.140, 0.450)  # plywood: 18 mm thick, 14 cm wide, 45 cm tall, standing on the table
R_OUT = np.diag([1.0, -1.0, -1.0])  # outlet frame -> flowmeter (prop) frame: z down


def _to_fm(p):
    return np.asarray(p, float) @ R_OUT.T


def outlet_pose():
    """(origin, rotation) of the outlet frame in the world."""
    return np.array([OUTLET_XY[0], OUTLET_XY[1], Z_UNDER]), R_OUT.copy()


# Friction (static, dynamic, combine). The bottle, nut and lid are polypropylene and the outlet brass: plastic on
# clean metal or plastic, dry, 0.2 (Engineering ToolBox, friction coefficients), averaged with the other surface.
# The YAM's finger pads are rubber-faced, 1.0 / 0.8 (gripkit), so a pad on the bottle grips at about 0.6 / 0.5.
PLASTIC = (0.20, 0.20, "average")
BRASS = (0.20, 0.20, "average")
CONTACT_OFFSET = 0.001  # m (the thread's gaps are under 1 mm)

BRASS_LOOK = kit.Look((0.78, 0.66, 0.36), 0.35, metallic=0.85)
FM_BODY = kit.Look((0.90, 0.91, 0.92), 0.4)
GREEN = kit.Look((0.10, 0.55, 0.25), 0.5)  # oxygen green knob
WOOD = kit.Look((0.78, 0.66, 0.48), 0.8)
JAR_LOOK = kit.Look((0.85, 0.90, 0.95), 0.15, opacity=0.45)
WATER = kit.Look((0.55, 0.75, 0.95), 0.1, opacity=0.6)
LID_LOOK = kit.Look((0.07, 0.07, 0.08), 0.5)  # the 7600's black lid
NUT_LOOK = kit.Look((0.92, 0.92, 0.90), 0.5)

_BOARD_C = (BOARD_X + BOARD[0] / 2, 0.0, (TOP + BOARD[2] / 2) - Z_UNDER)
_BODY_C = (BOARD_X - BODY[0] / 2, 0.0, HEX_H + BODY[2] / 2)  # against the board
FLOWMETER = kit.Prop(
    "flowmeter", pos=(OUTLET_XY[0], OUTLET_XY[1], Z_UNDER), mass=None,
    size=(2 * (BOARD_X + BOARD[0]), BOARD[1], BOARD[2]),
    meshes=(kit.box(_BOARD_C, BOARD, WOOD),
            kit.box(_BODY_C, BODY, FM_BODY),
            kit.cylinder((BOARD_X - BODY[0] - 0.001, 0.0, HEX_H + 0.095), 0.009, 0.120, kit.Look((0.95, 0.97, 1.0), 0.05, opacity=0.5)),
            kit.cylinder((BOARD_X - BODY[0] / 2, 0.0, HEX_H + BODY[2] + 0.012), 0.016, 0.024, GREEN),
            kit.cylinder((0.0, 0.0, HEX_H / 2), HEX_R, HEX_H, BRASS_LOOK),
            kit.cylinder((0.0, 0.0, -tk.STUB_L / 2), tk.R_CREST - 0.0001, tk.STUB_L, BRASS_LOOK)),
    colliders=(kit.Collider(_BOARD_C, size=BOARD),
               kit.Collider(_BODY_C, size=BODY),
               kit.Collider((0.0, 0.0, HEX_H / 2), radius=HEX_R, height=HEX_H),
               *tk.colliders(tk.stub_point_sets(), _to_fm)),
    material=BRASS, contact_offset=CONTACT_OFFSET)

# -- the humidifier (bottle frame: origin at the middle of its bottom, z up its axis) -----------------------------------
WATER_ML = 290.0  # filled to the maximum line (AirLife 7600 user guide: 290 mL)
DRY_KG = 0.090  # the empty bottle with its lid and nut
NUT_KG = 0.008
WATER_KG = WATER_ML / 1000.0
WATER_H = WATER_ML * 1e-6 / (math.pi * (JAR_R - JAR_WALL) ** 2)  # 96 mm deep in the 62 mm bore
BOTTLE_KG = DRY_KG - NUT_KG + WATER_KG


def bottle_mass_props():
    """(center of mass, principal moments) of the bottle without its nut: the water as a solid cylinder on the jar's
    floor, the dry bottle as a thin shell of the jar's size with the lid's mass at its top."""
    rw, hw, zw = JAR_R - JAR_WALL, WATER_H, JAR_WALL + WATER_H / 2
    md = DRY_KG - NUT_KG
    zd = 0.085  # the dry bottle's own center of mass: jar shell plus the lid and tower over it
    m = WATER_KG + md
    zc = (WATER_KG * zw + md * zd) / m
    izz = WATER_KG * rw ** 2 / 2 + md * JAR_R ** 2
    ixx = (WATER_KG * (3 * rw ** 2 + hw ** 2) / 12 + WATER_KG * (zw - zc) ** 2
           + md * (JAR_R ** 2 / 2 + 0.150 ** 2 / 12) + md * (zd - zc) ** 2)
    return (0.0, 0.0, zc), (ixx, ixx, izz)


BOTTLE_COM, BOTTLE_INERTIA = bottle_mass_props()
R_NUT = np.diag([1.0, -1.0, -1.0])  # nut frame -> the nut link's frame (the link's z up, the nut's z down)


def _to_link(p):
    return np.asarray(p, float) @ R_NUT.T


NUT_START = 0.0  # the nut's swivel at the start: its wings point at 0, 90, 180 and 270 degrees
NUT_SWIVEL_FRICTION = 0.002  # N m: the nut turns freely on its tower
BOTTLE_START = (0.380, -0.140)

NUT_LINK = kit.Link(
    "nut", "revolute", "Z", anchor=(0.0, 0.0, NUT_MOUTH), lower=-100.0, upper=100.0, start=NUT_START, mass=NUT_KG,
    meshes=(kit.cylinder((0.0, 0.0, -tk.NUT_H / 2), tk.HUB_R, tk.NUT_H, NUT_LOOK),
            *(kit.box((0.5 * (tk.HUB_R + tk.WING_SPAN / 2) * math.cos(a), -0.5 * (tk.HUB_R + tk.WING_SPAN / 2)
                       * math.sin(a), -tk.WING_H / 2),
                      (abs(math.cos(a)) * (tk.WING_SPAN / 2 - tk.HUB_R) + abs(math.sin(a)) * tk.WING_T,
                       abs(math.sin(a)) * (tk.WING_SPAN / 2 - tk.HUB_R) + abs(math.cos(a)) * tk.WING_T, tk.WING_H),
                      NUT_LOOK) for a in tk.wing_angles())),
    colliders=tk.colliders(tk.nut_point_sets(), _to_link),
    damping=0.0, friction=NUT_SWIVEL_FRICTION)

_LID_PROFILE = [(0.0, LID_TOP), (TOWER_R + 0.012, LID_TOP), (LID_R - 0.004, LID_SKIRT[1] + 0.006),
                (LID_R, LID_SKIRT[1]), (LID_R, LID_SKIRT[0]), (JAR_R, LID_SKIRT[0])]
BOTTLE = kit.Mechanism(
    "bottle", pos=(BOTTLE_START[0], BOTTLE_START[1], TOP + 0.0001), mass=BOTTLE_KG,
    size=(2 * LID_R, 2 * LID_R, NUT_MOUTH), com=BOTTLE_COM, inertia=BOTTLE_INERTIA,
    meshes=(kit.cylinder((0.0, 0.0, JAR_H / 2), JAR_R, JAR_H, JAR_LOOK),
            kit.cylinder((0.0, 0.0, JAR_WALL + WATER_H / 2), JAR_R - JAR_WALL - 0.0003, WATER_H, WATER),
            kit.lathe(_LID_PROFILE, LID_LOOK, n=40),
            kit.cylinder((0.0, 0.0, (LID_TOP + TOWER_TOP) / 2), TOWER_R, TOWER_TOP - LID_TOP + 0.001, LID_LOOK)),
    colliders=tk.colliders([tk.prism(tk._polygon_r(JAR_R), 0.0, LID_SKIRT[0]),
                            tk.frustum(LID_R, LID_SKIRT[0], LID_R, LID_SKIRT[1]),
                            tk.frustum(LID_R, LID_SKIRT[1], TOWER_R + 0.012, LID_TOP),
                            tk.prism(TOWER_R, LID_TOP - 0.001, TOWER_TOP)]),
    links=(NUT_LINK,), material=PLASTIC, contact_offset=CONTACT_OFFSET)

OBJECTS = [FLOWMETER, BOTTLE]
gripkit.use_rubber_pads()

# -- free-body root (as squeezekit.free_body: the articulation's root on the bottle's own body, its center of mass
#    pinned) ----------------------------------------------------------------------------------------------------------
_ROOT = 'def Xform "Root" (\n    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n)\n{\n'
_BASE = ('    def Xform "base" (\n        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", '
         '"PhysxRigidBodyAPI"]\n    )\n    {\n')


def _root_on_base(text: str, com) -> str:
    if _ROOT not in text or _BASE not in text:
        raise RuntimeError("o2humid: unexpected yamkit USD layout (articulation root)")
    lines = text.replace(_ROOT, 'def Xform "Root"\n{\n', 1).splitlines(True)
    settings = [ln for ln in lines if ln.startswith("    ") and not ln.startswith("     ") and "physxArticulation:" in ln]
    text = "".join(ln for ln in lines if ln not in settings)
    text = text.replace(_BASE, _BASE.replace('"PhysxRigidBodyAPI"]', '"PhysxRigidBodyAPI", "PhysicsArticulationRootAPI", '
                                             '"PhysxArticulationAPI"]') + "".join("    " + ln for ln in settings), 1)
    i = text.index('    def Xform "base" (')
    if "physics:centerOfMass" not in text[i:text.index("def ", i + 30)]:
        eol = text.index("\n", text.index("float physics:mass", i)) + 1
        text = text[:eol] + "        point3f physics:centerOfMass = (%.6g, %.6g, %.6g)\n" % tuple(com) + text[eol:]
    return text


if not getattr(kit, "_o2humid_root", False):
    _inner_usda = kit._mech_usda

    def _mech_usda(m):
        text = _inner_usda(m)
        return _root_on_base(text, BOTTLE_COM) if m.name == "bottle" else text

    kit._mech_usda = _mech_usda
    kit._o2humid_root = True

# -- reading the trial ----------------------------------------------------------------------------------------------------
# Seated (screwed fully on, hand-tight): the nut's turn within SEAT_TURN_DEG of its seated turn, on the thread's last
# turn (within half the 1.41 mm pitch of its seat), its axis within NUT_OFF of the outlet's and NUT_TILT_DEG of
# parallel. Read from the nut's own pose on the outlet.
SEAT_TURN_DEG = 45.0
SEAT_RISE = tk.PITCH / 2
NUT_OFF = 0.0006
NUT_TILT_DEG = 2.0
# Pushed straight up without turning, the nut stops where its first full thread meets the stub's last: with its turn
# at the seated turn (mod 360), at STOP0 off its seat (2.70 mm; the ridges' run-outs let it past the ends' first 45
# degrees; test_task --slow checks it). From any turn it stops within a pitch above that (2.70 to 4.1 mm).
STOP0 = 0.00270
CATCH_RISE = STOP0 + tk.PITCH  # m: above this the threads cannot touch
# Started square: the nut a full turn past where a straight push stops it at best (so at least one turn of thread has
# been turned on), square and centered.
ON_RISE = STOP0 - tk.PITCH
# Cross-threaded: the nut turned at least a quarter turn onto the thread (its rise a quarter pitch under STOP0) with its
# axis tilted more than CROSS_DEG off the outlet's: its first thread then lands a half pitch out across the pitch
# diameter on one side, so the far side rides in the next groove (tilt = atan(P/2 / d2), 3.0 degrees). Once, and the
# trial has failed (a cross-threaded plastic nut damages its own thread and leaks).
CROSS_DEG = math.degrees(math.atan(tk.PITCH / 2 / tk.D2))
CROSS_RISE = STOP0 - tk.PITCH / 4
CROSS_OFF = 0.003
# Tipped: the bottle tipped so far that the 290 mL reaches the gas passage under its lid (TIP_DEG, from the jar's and
# lid's inside shape: see water_reaches_port). Dropped: falling faster than DROP_V at any moment (a fall of 8 mm).
DROP_V = 0.40  # m/s
CLEAR = 0.005
STILL_STEP = 0.0005  # m per control step (1 cm/s)
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
ROBOTS = ("left_robot", "right_robot")
LINK_POSES: dict = {}


def rotm(q) -> np.ndarray:
    return tk.rotm(q)


PORT = (0.0, 0.0, 0.144)  # the gas passage's entry under the lid's dome (bottle frame)


def water_reaches_port(tilt_deg: float, n: int = 48) -> bool:
    """Whether 290 mL in the jar (62 mm bore, its floor 2 mm up, the lid's inside narrowing to the passage) covers the
    passage's entry with the bottle tipped tilt_deg. Grid of the inside volume, level water plane."""
    rin = JAR_R - JAR_WALL
    xs = np.linspace(-rin, rin, n)
    zs = np.linspace(JAR_WALL, PORT[2] + 0.002, 3 * n)
    X, Y, Z = np.meshgrid(xs, xs, zs, indexing="ij")
    rlim = np.where(Z <= LID_SKIRT[1], rin, rin + (0.010 - rin) * (Z - LID_SKIRT[1]) / (PORT[2] + 0.002 - LID_SKIRT[1]))
    inside = X ** 2 + Y ** 2 <= rlim ** 2
    dv = (xs[1] - xs[0]) ** 2 * (zs[1] - zs[0])
    th = math.radians(tilt_deg)
    up = np.array([-math.sin(th), 0.0, math.cos(th)])
    h = np.sort((X * up[0] + Z * up[2])[inside])
    level = h[min(len(h) - 1, int(WATER_ML * 1e-6 / dv))]
    return float(np.asarray(PORT) @ up) <= level


def _tip_deg() -> float:
    for d in range(30, 181, 1):
        if water_reaches_port(float(d)):
            return float(d)
    return 180.0


TIP_DEG = 83.0  # = _tip_deg() (test_task checks it)


def nut_pose(objects):
    """(origin, rotation) of the nut frame in the world, from the nut link's pose."""
    n = objects["bottle/nut"]
    return np.asarray(n.pos, float), rotm(n.quat_xyzw) @ R_NUT


def nut_fit(objects) -> dict:
    o, R = outlet_pose()
    no, nR = nut_pose(objects)
    return tk.relative(o, R, no, nR)


def seated(objects) -> bool:
    f = nut_fit(objects)
    turn = (f["turn"] + 180.0) % 360.0 - 180.0
    return (abs(turn) <= SEAT_TURN_DEG and -SEAT_RISE < f["rise"] < SEAT_RISE and f["off"] <= NUT_OFF
            and f["tilt"] <= NUT_TILT_DEG)


def on_thread(objects) -> bool:
    """The nut at least a full turn of thread onto the outlet, square and centered."""
    f = nut_fit(objects)
    return -SEAT_RISE <= f["rise"] <= ON_RISE and f["off"] <= NUT_OFF and f["tilt"] <= NUT_TILT_DEG


def over_thread(objects) -> bool:
    """The nut turned onto the thread (a quarter turn past the best straight-push stop), its axis within CROSS_OFF of
    the outlet's."""
    f = nut_fit(objects)
    return f["rise"] <= CROSS_RISE and f["off"] <= CROSS_OFF


def bottle_tilt_deg(objects) -> float:
    R = rotm(objects["bottle"].quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))


# -- the outlines the arms are checked against ------------------------------------------------------------------------
def _seg_gap(q, a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = float(np.linalg.norm(b - a))
    u = (b - a) / L
    d = q - a
    s = d @ u
    radial = np.linalg.norm(d - s[:, None] * u, axis=1)
    return np.hypot(np.maximum(radial - r, 0.0), np.maximum(np.maximum(-s, s - L), 0.0))


def bottle_gap(qb):
    """Distance from bottle-frame points to the bottle's outline (jar, lid, tower)."""
    g = _seg_gap(qb, (0, 0, 0), (0, 0, LID_SKIRT[0]), JAR_R)
    g = np.minimum(g, _seg_gap(qb, (0, 0, LID_SKIRT[0]), (0, 0, LID_TOP), LID_R))
    return np.minimum(g, _seg_gap(qb, (0, 0, LID_TOP), (0, 0, TOWER_TOP), TOWER_R))


def nut_gap(qn):
    """Distance from nut-link-frame points to the nut's outline (hub and wings, as one disc)."""
    return _seg_gap(qn, (0, 0, -tk.NUT_H), (0, 0, 0), tk.WING_SPAN / 2)


def _parts(objects):
    b, n = objects["bottle"], objects["bottle/nut"]
    return {"bottle": (np.asarray(b.pos, float), rotm(b.quat_xyzw), bottle_gap, 0.17),
            "nut": (np.asarray(n.pos, float), rotm(n.quat_xyzw), nut_gap, 0.03)}


_GAP_MEMO: dict = {}


def link_gap(objects, key, part) -> float:
    """Smallest gap (m) between arm link `key` ("right_robot/tip_left") as posed and `part`; 0.02 beyond 2 cm."""
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
        out = 0.02
    else:
        q = yg.near(link, lambda x: (o_ + x @ R_.T - co) @ Rc, gap, 0.02)
        out = min(0.02, float(gap(q).min())) if len(q) else 0.02
    if len(_GAP_MEMO) > 1024:
        _GAP_MEMO.clear()
    _GAP_MEMO[memo] = out
    return out


def arm_gap(objects, robots=ROBOTS) -> float:
    """Smallest gap (m) between any arm link of `robots` and the bottle or its nut; 0.02 when none is within 2 cm."""
    keys = [k for k in LINK_POSES if k.split("/")[0] in robots]
    return min((link_gap(objects, k, p) for k in keys for p in ("bottle", "nut")), default=0.02)


class Ledger:
    """One trial: the latches (cross-threaded, dropped, tipped), the furthest the nut got, the hold count."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.crossed = self.dropped = self.tipped = False
        self.started = False  # the nut at least a full turn onto the thread, square, at some step
        self.seat = False
        self.on = False
        self.clear = False
        self.good = 0
        self.last = None
        self.n = 0

    def step(self, objects) -> None:
        self.n += 1
        b = objects["bottle"]
        bp = np.asarray(b.pos, float)
        f = nut_fit(objects)
        tilt = bottle_tilt_deg(objects)
        if over_thread(objects) and f["tilt"] > CROSS_DEG:
            self.crossed = True
        if tilt > TIP_DEG:
            self.tipped = True
        vz = 0.0 if self.last is None else float(bp[2] - self.last[2]) / DT
        if vz < -DROP_V:
            self.dropped = True
        if on_thread(objects):
            self.started = True
        self.seat = seated(objects)
        self.on = on_thread(objects)
        gap = arm_gap(objects)
        self.clear = gap > CLEAR
        still = self.last is not None and float(np.linalg.norm(bp - self.last)) <= STILL_STEP
        self.last = bp
        done = self.seat and self.clear and still and not self.failed()
        self.good = self.good + 1 if done else 0
        if _PROBE:
            with open(_PROBE, "a") as fh:
                fh.write("%d rise %.5f off %.5f tilt %.2f turn %.1f btilt %.2f vz %.3f gap %.4f seat %d started %d "
                         "X %d D %d T %d good %d q %.3f B %s L %s R %s\n" % (
                             self.n, f["rise"], f["off"], f["tilt"], f["turn"], tilt, vz, gap, int(self.seat),
                             int(self.started), int(self.crossed), int(self.dropped), int(self.tipped), self.good,
                             _nut_q(objects), " ".join("%.4f" % v for v in bp), _tcp("left_robot"),
                             _tcp("right_robot")))

    def failed(self) -> bool:
        return self.crossed or self.dropped or self.tipped

    def score(self) -> float:
        if self.failed():
            return 0.0
        if self.seat and self.good >= HOLD_STEPS:
            return 1.0
        if self.seat:
            return 0.5
        return 0.25 if self.on else 0.0


def _tcp(robot) -> str:
    """The arm's tool point (world), for the probe log."""
    if f"{robot}/gripper" not in LINK_POSES:
        return "nan nan nan"
    o, R = LINK_POSES[f"{robot}/gripper"]
    from yam_arena.constants import TCP_OFFSET
    return " ".join("%.4f" % v for v in np.asarray(o) + np.asarray(R) @ np.asarray(TCP_OFFSET))


def _nut_q(objects) -> float:
    try:
        return kit.joint(objects, "bottle/nut").pos
    except KeyError:
        return float("nan")


_L = None
_PROBE = os.environ.get("O2_PROBE")


def _read_links(base) -> None:
    """Every arm link's pose (env frame) at this control step, into LINK_POSES. A missing arm or link, or finger pads
    without their rubber, stops the trial: the contact checks and the grip would otherwise pass silently."""
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError as e:
            raise RuntimeError(f"o2humid: no {robot} in the scene for the contact checks") from e
        names = list(art.body_names)
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        missing = [link for link in ARM_LINKS if link not in names]
        if missing:
            raise RuntimeError(f"o2humid: {robot} has no link(s) {missing} for the contact checks")
        for link in ARM_LINKS:
            i = names.index(link)
            LINK_POSES[f"{robot}/{link}"] = (P[i] - origin, rotm(Q[i]))
    if gripkit._STATE.get("bound", 0) < 0:
        raise RuntimeError("o2humid: the finger pads' rubber material could not be bound")
    if not _READ_ONCE:
        _READ_ONCE.append(1)
        print(f"o2humid: contact checks on {len(LINK_POSES)} arm links; pad shapes with rubber: "
              f"{gripkit._STATE.get('bound')}", file=sys.stderr, flush=True)


_READ_ONCE: list = []
kit.STATE_HOOKS.append(_read_links)
if "isaaclab" in sys.modules:  # in the simulator: read and check the hulls as the task loads
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
    """The nut seated hand-tight and square on the outlet, the bottle hanging from it with every arm link at least
    5 mm clear, still, and no cross-thread, drop or tip all trial: this step counts toward the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """1.0: the nut seated (its turn within 45 degrees of the seat, within half a pitch of it, its axis within 0.6 mm
    and 2 degrees of the outlet's), the bottle hanging from the outlet with every arm link at least 5 mm clear, still
    (under 1 cm/s) for 1 s. 0.5: seated, but not yet let go or still. 0.25: the nut started square, a full turn onto
    the thread, but not seated. 0: otherwise; and always 0 once the nut was cross-threaded (tilted more than 3.0
    degrees off the outlet's axis while over its thread), the bottle dropped (falling faster than 0.4 m/s) or tipped
    so far that its water reaches the gas outlet (83 degrees). Doing nothing scores 0."""
    _update(objects)
    return _L.score()


# -- scripted demo: a fixed plan of waypoints for this layout (object_jitter=0); it reads no object or simulator state --
# demoplan.json holds each arm's waypoints in order: a tool-point position (m, world), a grip rotation (3x3), a finger
# command and a duration (s), or a move back to the ready pose. Two hands, as a person does it:
# - the right comes down over the bottle pointing toward +y and 15 degrees down, pinches the lid's skirt (50 mm under
#   the nut, 70 mm over the full bottle's center of mass), lifts it 20 mm, carries it under the outlet, aimed about
#   25 mm high and turned so the bottle hangs plumb (the arm gives under the 380 g bottle), and raises the nut onto the
#   thread's end, aimed 4 mm higher so the arm's give presses it lightly up;
# - the left comes in pointing at the nut, pinches the wing nearest the robot's left and turns the nut a quarter turn
#   counterclockwise seen from above, moving round the nut's axis and following the thread up; it opens, swings back
#   round clear of the wings and takes the next wing; ten strokes, the last one 15 degrees past the seat;
# - after a turn the right lets go and backs away, so the bottle hangs from the thread; the left lets go at the end.
# Variants (O2_VARIANT), each the same plan changed: "nothing" stays at the ready pose; "crooked" holds the bottle 6
# degrees off plumb from the line-up on; "short" stops turning 0.6 turn before the seat; "held" never lets go.
FINGER_HALF_OPEN = 0.047
GRIP_Z = (LID_SKIRT[0] + LID_SKIRT[1]) / 2  # m up the bottle: the right's pinch line on the lid's skirt
PLAN_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demoplan.json")
CROOKED_DEG = 6.0
SHORT_TURNS = 0.6


def finger_for(half: float) -> float:
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * min(FINGER_HALF_OPEN, max(0.0, half)) / FINGER_HALF_OPEN


@dataclasses.dataclass
class Fixed:
    """A demo waypoint with the grip's whole rotation given (yamkit's plan player honors `rot`)."""

    duration: float
    pos: tuple
    finger: float
    label: str = ""
    rot: object = None
    yaw: float = 0.0


def _rx(deg):
    a = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


def _heading(R) -> float:
    """Where a forward-pointing hand points, seen from above (degrees)."""
    R = np.asarray(R, float)
    return math.degrees(math.atan2(R[1, 2], R[0, 2]))


def _rows(variant: str) -> dict:
    import json

    with open(PLAN_FILE) as fh:
        plan = json.load(fh)
    if variant == "crooked":  # from the line-up on, the right's grip turned 6 degrees about x through the nut's mouth
        rows, on = plan["right"], False
        for r in rows:
            on = on or r["label"].startswith("look")
            if on and not r.get("home"):
                R6, p = _rx(CROOKED_DEG), np.asarray(r["pos"], float)
                pivot = p + np.array([0.0, 0.0, NUT_MOUTH - GRIP_Z])
                r["pos"] = (pivot + R6 @ (p - pivot)).tolist()
                r["rot"] = (R6 @ np.asarray(r["rot"], float)).tolist()
    if variant == "short":  # the left's strokes stop SHORT_TURNS before the plan's end (it opens and holds there)
        rows = plan["left"]
        turns, total = [], 0.0
        for k, r in enumerate(rows):
            if "turn" in r["label"] and k > 0 and not rows[k - 1].get("home"):
                d = (_heading(r["rot"]) - _heading(rows[k - 1]["rot"]) + 180.0) % 360.0 - 180.0
                if "turn" in rows[k - 1]["label"] or "pinch" in rows[k - 1]["label"]:
                    total += max(0.0, d)
            turns.append(total)
        stop = total - SHORT_TURNS * 360.0
        k0 = next(k for k, t in enumerate(turns) if t > stop)
        hold = dict(rows[k0 - 1])
        for k in range(k0, len(rows)):
            if not rows[k].get("home"):
                rows[k] = dict(hold, duration=rows[k]["duration"], label="hold short of the seat",
                               finger=finger_for(tk.WING_T / 2 + 0.008))
    if variant == "held":  # the right never lets go: it keeps its last holding pose
        rows = plan["right"]
        k0 = next(k for k, r in enumerate(rows) if "or let go" in r["label"] and r["finger"] < rows[k - 1]["finger"])
        hold = dict(rows[k0 - 1])
        for k in range(k0, len(rows)):
            rows[k] = dict(hold, duration=rows[k]["duration"], label="keep holding it")
    return plan


def demo():
    """Each arm's fixed plan (demoplan.json), or a variant of it (O2_VARIANT: nothing, crooked, short, held)."""
    variant = os.environ.get("O2_VARIANT", "")
    if variant == "nothing":
        return {"left": [kit.Home(10.0, label="stay at the ready pose")],
                "right": [kit.Home(10.0, label="stay at the ready pose")]}
    out = {}
    for side, rows in _rows(variant).items():
        out[side] = [kit.Home(float(r["duration"]), label=r["label"]) if r.get("home") else
                     Fixed(float(r["duration"]), tuple(float(v) for v in r["pos"]), float(r["finger"]), r["label"],
                           np.asarray(r["rot"], float)) for r in rows]
    return out


def _t(seq):
    return sum(w.duration for w in seq)
