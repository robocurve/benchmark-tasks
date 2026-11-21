"""Spot-weld a nickel strip onto a 2x2 pack of 18650 cells with a capacitor spot welder's pen.

On the bench: an assembled 2x2 pack of 18650 cells standing loose; a nickel strip in its tray; and a battery spot
welder, its power unit at the back of the bench and its welding pen standing in a holster, tied to the unit by its
heavy lead. The pen has two copper needles, each on its own spring: in the welder's automatic mode it fires as soon as
both needles are pressed in to their set force (here 200 g each), with both on the metal.

One hand lays the strip on the pack, its bars over the cells; the other takes the pen out of its holster, presses its
needles onto the strip over each cell's + terminal in turn, and puts the pen back.
"""

# Impact on jobs. Joining cells into packs is assembly work: team assemblers (O*NET 51-2092.00, counted by BLS with
# other miscellaneous assemblers and fabricators, about 1.47 million US jobs in 2024) rotate through every task of
# their line. On lithium-ion pack lines and repair benches the cells are joined by resistance spot-welding nickel strip
# onto each terminal with a pen on leads from a power unit.
#
# This is the idea of TK-PF8K7B (PR #1300, another team's; withdrawn, hold lapsed). Its review asked for two things,
# both done here: the welder is modelled as used in pack assembly (a welding pen on a heavy lead to a power unit, its
# needles pressed at a stated force for each weld) and the pack stands loose, so it shifts if knocked.
#
# World frame: x forward (away from the arms), y left, z up, metres. The arm bases stand at x 0, y +-0.305.

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import weldkit  # noqa: E402,F401  (hooks yam_arena so the lead below is simulated)
import cableshape  # noqa: E402  (lays the lead at every reset)
from labkit import Joint, part  # noqa: E402
from taskbase import Trial, fake_state  # noqa: E402
from weldkit import Plug, cable  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Lay the nickel strip on the battery pack, then spot-weld it to all four cells with the welding pen "
               "and put the pen back in its holster.")

BLACK, DARK, GREY = (0.06, 0.06, 0.07), (0.16, 0.17, 0.19), (0.55, 0.57, 0.60)
YELLOW, NICKEL, COPPER = (0.88, 0.72, 0.16), (0.74, 0.74, 0.72), (0.80, 0.46, 0.24)
RED, PANEL, WHITE = (0.70, 0.12, 0.10), (0.20, 0.30, 0.36), (0.92, 0.92, 0.90)

# -- the battery pack (loose, 201 g, one rigid body: a press fit) ------------------------------------------------------
# Two halves of a black ABS 2x2 18650 holder (41.2 mm square, 20 mm pitch) round four 18 x 65 mm cells, + up. The top
# half is a frame with a window over each cell; the cells' + caps stand 0.8 mm proud of it, so the strip lies on them.
PACK_POS = (0.28, 0.0)
HALF = 0.0206
PITCH = 0.02
CELL_R, CELL_H = 0.009, 0.065
CAP_R, CAP_H = 0.0035, 0.0008
PACK_TOP = CELL_H  # holder top face, above the pack's bottom
CAP_TOP = CELL_H + CAP_H
CELLS = [(sx * PITCH / 2, sy * PITCH / 2) for sx, sy in ((-1, 1), (1, 1), (1, -1), (-1, -1))]  # cell 1..4 (pack frame)


def _pack_shapes():
    boxes = [((0, 0, 0.005), (2 * HALF, 2 * HALF, 0.010), BLACK)]  # bottom half
    zt, ht = PACK_TOP - 0.0045, 0.009  # top half: a frame 9 mm deep
    for s in (-1, 1):
        boxes.append(((s * (HALF - 0.0008), 0, zt), (0.0016, 2 * HALF, ht), BLACK))
        boxes.append(((0, s * (HALF - 0.0008), zt), (2 * HALF, 0.0016, ht), BLACK))
    boxes += [((0, 0, zt), (0.002, 2 * HALF, ht), BLACK), ((0, 0, zt), (2 * HALF, 0.002, ht), BLACK)]
    cyls = []
    for x, y in CELLS:
        cyls.append(((x, y, 0.0015 + (CELL_H - 0.0015) / 2), CELL_R, CELL_H - 0.0015, YELLOW))
        cyls.append(((x, y, CELL_H + CAP_H / 2), CAP_R, CAP_H, NICKEL))
    return boxes, cyls


_pb, _pc = _pack_shapes()
PACK = part("battery_pack", pos=(PACK_POS[0], PACK_POS[1], TOP), mass=0.201, boxes=_pb, cylinders=_pc, color=BLACK,
            friction=1.2)  # the strip grips the + caps harder than the needles grip the strip (friction is averaged)

# -- the nickel strip (loose, 1 g) --------------------------------------------------------------------------------------
# An H of 0.15 mm pure nickel for a 2x2 pack in parallel: two 8 x 28 mm bars 20 mm apart (along x, over the cell pairs)
# joined by an 8 mm bridge; a grip tab folded up from the bridge's middle into a loop 8 mm wide, 14 mm tall and 3 mm
# across. It collides as 0.5 mm plates, the tab as a solid 3 mm block.
STRIP_T = 0.0005
TRAY_POS = (0.24, 0.17)
TRAY_FLOOR = 0.003
STRIP_START = (TRAY_POS[0], TRAY_POS[1], TOP + TRAY_FLOOR)
TAB_H, TAB_W, TAB_T = 0.014, 0.008, 0.003
STRIP = part("nickel_strip", pos=STRIP_START, mass=0.0012, color=NICKEL, friction=1.0, boxes=[
    ((0, PITCH / 2, STRIP_T / 2), (0.028, 0.008, STRIP_T), NICKEL),
    ((0, -PITCH / 2, STRIP_T / 2), (0.028, 0.008, STRIP_T), NICKEL),
    ((0, 0, STRIP_T / 2), (0.008, 0.012, STRIP_T), NICKEL),
    ((0, 0, STRIP_T + TAB_H / 2), (TAB_W, TAB_T, TAB_H), NICKEL)])
TRAY = part("strip_tray", pos=(TRAY_POS[0], TRAY_POS[1], TOP), fixed=True, color=GREY, boxes=[
    ((0, 0, TRAY_FLOOR / 2), (0.04, 0.04, TRAY_FLOOR), GREY),
    *[((s * 0.0185, 0, 0.0025), (0.003, 0.04, 0.005), GREY) for s in (-1, 1)],
    *[((0, s * 0.0185, 0.0025), (0.034, 0.003, 0.005), GREY) for s in (-1, 1)]])

# -- the welding pen (loose, 110 g, on its lead) ------------------------------------------------------------------------
# Pen frame: z along the pen, 0 at the nose's end. Two 2.5 mm copper needles 4 mm apart stand 18 mm out of the nose,
# each on its own slide and spring; the welder fires once both are pressed in 1.5 mm, which takes 200 g (1.96 N) each.
# Above the nose, a red finger guard 42 mm across; above it the grip, and the lead entering the pen's side at the top.
NEEDLE_X = 0.002
NEEDLE_R, NEEDLE_OUT = 0.00125, 0.018
FIRE = 0.0015  # needle travel at which the welder fires (m)
FIRE_N = 0.200 * 9.81
NEEDLE_K = FIRE_N / FIRE  # N/m
GUARD_Z0, GUARD_Z1, GUARD_R = 0.020, 0.026, 0.021
BODY_R, BODY_TOP = 0.012, 0.105
PEN_TOP = 0.113
LEAD_AT = (BODY_R + 0.006, 0.0, 0.100)  # where the lead enters, pen frame
GRIP_Z = GUARD_Z1 + 0.001 + 0.013  # tool point, pen frame: fingertips 1 mm above the guard


def _needle(name, x):
    return Joint(name, kind="slide", axis="Z", anchor=(x, 0.0, 0.0), lower=0.0, upper=0.004, stiffness=NEEDLE_K,
                 damping=2.0, target=0.0, mass=0.003,
                 cylinders=[((x, 0.0, -NEEDLE_OUT + 0.015), NEEDLE_R, 0.030, COPPER)])


# The holster (fixed): a square cup the pen stands in, its guard resting on the rim.
HOLSTER_RIM = 0.067


def pen_seat(hx, hy):
    return (hx, hy, TOP + HOLSTER_RIM - GUARD_Z0)


# The lead: 5 mm round, 0.2 kg/m. Its far end is a moulded plug seated in a socket on the power unit's front at bench
# level (the socket's slot is narrower than the plug, so a pull can't take it out); from there the lead lies back
# along the bench, rises beside the holster and curves over into the pen's side. It spawns straight, already ending at
# the pen's side (its plug end floating clear for that moment); every reset then lays it in that shape with cableshape
# (from the accepted cable tasks) and lets it settle; the shape stays its set, as a heavy lead keeps its set.
LEAD_R = 0.005
LEAD_Z = TOP + 0.0065  # the lead's centre on the bench, level with the plug's middle
SOCKET_X = 0.483  # the socket's front plate (outer face); the lead leaves through its slot
LEAD_X0 = SOCKET_X + 0.0035  # the lead's plug end: the plug sits just behind the plate
HOLSTER_POS = (0.176, -0.20)
PEN_START = (HOLSTER_POS[0], HOLSTER_POS[1], TOP + HOLSTER_RIM - GUARD_Z0)
PEN_HOME = PEN_START
_ATTACH = np.array(PEN_START) + np.array(LEAD_AT)
PLUG = Plug(size=(0.026, 0.024, 0.012), mass=0.040, color=(0.12, 0.12, 0.13))


def _lead_path():
    """The lead's centreline from the socket: along the bench, a 4 cm bend up, straight up, a 4 cm bend over toward
    the pen, 1 cm into its side. Returns (points, length)."""
    R, S = 0.04, 0.01
    y, z0 = _ATTACH[1], LEAD_Z
    xa = _ATTACH[0] + 2 * R + S
    V = _ATTACH[2] - z0 - 2 * R
    pts = [np.array([LEAD_X0, y, z0])]
    for u in np.linspace(0, 1, 12)[1:]:
        pts.append(np.array([LEAD_X0 + (xa - LEAD_X0) * u, y, z0]))
    for a in np.linspace(0, math.pi / 2, 9)[1:]:
        pts.append(np.array([xa - R * math.sin(a), y, z0 + R - R * math.cos(a)]))
    for u in np.linspace(0, 1, 5)[1:]:
        pts.append(np.array([xa - R, y, z0 + R + V * u]))
    for a in np.linspace(0, math.pi / 2, 9)[1:]:
        pts.append(np.array([xa - 2 * R + R * math.cos(a), y, z0 + R + V + R * math.sin(a)]))
    pts.append(np.array(_ATTACH))
    pts = np.array(pts)
    return pts, float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


LEAD_PATH, LEAD_LEN = _lead_path()
_N = round(LEAD_LEN / 0.02)
SPAWN_YAW = math.radians(240.0)  # straight spawn: from back left, ending at the pen's side, over the pack
_spawn_start = _ATTACH - LEAD_LEN * np.array([math.cos(SPAWN_YAW), math.sin(SPAWN_YAW), 0.0])
LEAD = cable("weld_lead", start=tuple(float(v) for v in _spawn_start), yaw=SPAWN_YAW, length=LEAD_LEN, radius=LEAD_R,
             segment=LEAD_LEN / _N, anchor_start=False, start_plug=PLUG, attach_to="weld_pen", attach_at=LEAD_AT,
             cable_mass=0.2 * LEAD_LEN, bend_stiffness=0.10, bend_damping=0.01, armature=4e-5, friction=0.7,
             color=(0.08, 0.08, 0.08))


def _shape_lead(emb):
    """Every reset: lay the lead from the socket to the pen and let it settle."""
    env, base, _ = cableshape._ctx(emb)
    origin = cableshape._t(base.scene.env_origins)[0].cpu().numpy()
    pen = base.scene["weld_pen"]

    def report(tag):
        ls = weldkit._cable_state(env, LEAD)
        pp = cableshape._t(pen.data.root_pose_w)[0].cpu().numpy()
        print(f"[weld] {tag}: plug {np.round(ls.start_plug, 3)} lead end {np.round(ls.end_tip, 3)}; pen "
              f"{np.round(pp[:3] - origin, 3)}", flush=True)

    report("spawned")
    cableshape.preshape(emb, LEAD, LEAD_PATH, hold="set", settle_s=1.5, lift=0.0)
    report("laid and settled")


cableshape.on_reset(_shape_lead)

PEN = part("weld_pen", pos=PEN_START, mass=0.110, color=BLACK, friction=0.8, joints=[
    _needle("needle_a", NEEDLE_X), _needle("needle_b", -NEEDLE_X)], cylinders=[
    ((0, 0, GUARD_Z0 / 2), 0.008, GUARD_Z0, BLACK),
    ((0, 0, (GUARD_Z0 + GUARD_Z1) / 2), GUARD_R, GUARD_Z1 - GUARD_Z0, RED),
    ((0, 0, (GUARD_Z1 + BODY_TOP) / 2), BODY_R, BODY_TOP - GUARD_Z1, BLACK),
    ((0, 0, (BODY_TOP + PEN_TOP) / 2), 0.009, PEN_TOP - BODY_TOP, DARK),
    ((BODY_R + 0.003, 0, LEAD_AT[2]), 0.0065, 0.008, DARK, "X")])  # the lead's strain relief

CUP = 0.0165  # cup inside half-width
HOLSTER = part("pen_holster", pos=(HOLSTER_POS[0], HOLSTER_POS[1], TOP), fixed=True, color=GREY, boxes=[
    ((0, 0, 0.003), (0.07, 0.07, 0.006), DARK),
    *[((s * (CUP + 0.0025), 0, HOLSTER_RIM / 2), (0.005, 2 * CUP + 0.010, HOLSTER_RIM), GREY) for s in (-1, 1)],
    *[((0, s * (CUP + 0.0025), HOLSTER_RIM / 2), (2 * CUP, 0.005, HOLSTER_RIM), GREY) for s in (-1, 1)]])

# The power unit (fixed): a bench capacitor spot welder, 14 x 23 x 23 cm, at the back right of the bench, its display,
# knobs and the lead's gland on the front.
def _socket_boxes():
    """The lead's socket on the unit's front (unit frame): a pocket for the plug, closed by a front plate with a
    14 mm slot for the lead (the plug is 24 mm wide and 12 mm tall), and a roof."""
    ux = 0.585
    x0, x1 = SOCKET_X - ux, 0.515 - ux  # the front plate's outer face, the unit's face
    cx, L = (x0 + x1) / 2, x1 - x0
    h = 0.017
    return [((cx, s * 0.0155, h / 2), (L, 0.004, h), BLACK) for s in (-1, 1)] + [
        ((cx, 0.0, h + 0.0015), (L, 0.035, 0.003), BLACK),  # roof
        *[((x0 + 0.0015, s * 0.0118, h / 2), (0.003, 0.0095, h), BLACK) for s in (-1, 1)],  # front posts
        ((x0 + 0.0015, 0.0, (0.0125 + h) / 2), (0.003, 0.014, h - 0.0125), BLACK)]  # lintel over the slot


UNIT = part("welder_unit", pos=(0.585, -0.20, TOP), fixed=True, color=DARK, boxes=[
    ((0, 0, 0.115), (0.14, 0.23, 0.23), DARK),
    ((-0.0705, 0.03, 0.17), (0.002, 0.10, 0.05), PANEL),  # display
    ((-0.0705, -0.075, 0.17), (0.002, 0.05, 0.05), BLACK),
    *[((s * 0.05, t * 0.09, 0.004), (0.02, 0.03, 0.008), BLACK) for s in (-1, 1) for t in (-1, 1)],
    *_socket_boxes()], cylinders=[
    ((-0.074, 0.03, 0.09), 0.012, 0.008, GREY, "X"), ((-0.074, 0.08, 0.09), 0.012, 0.008, GREY, "X"),
    ((-0.074, -0.075, 0.09), 0.010, 0.008, RED, "X"),  # mode switch
    ])

OBJECTS = [PACK, STRIP, TRAY, PEN, HOLSTER, UNIT, LEAD]

# -- scoring -----------------------------------------------------------------------------------------------------------
#
# Defects (0 for the rest of the trial, whatever follows): D1 the pack knocked: its centre more than 1 cm from where it
# started, or tilted over 10 degrees; D2 the pen dropped: tilted over 60 degrees.
# The strip is on the pack when it lies flat (under 10 degrees), resting on the + caps (its underside within 2 mm of
# them), its centre within 4 mm of the pack's centre and its bars within 10 degrees of the pack's x axis.
# A weld on cell k: at one control step both needles are pressed in at least 1.5 mm (each at 200 g or more), both needle
# tips are within 3.5 mm of cell k's axis and within 1.5 mm of the strip's top face, and the strip is on the pack.
# The pen is home when it stands in its holster: its axis within 6 mm of the cup's centre at the rim, its guard within
# 4 mm of the rim, leaning under 12 degrees, after it was taken out (lifted 3 cm).
# Score: 0.2 for the strip on the pack, 0.15 for each cell welded (once welded, a cell stays welded), 0.1 for the pen
# home. 1.0 when all of these hold, the strip, pack and pen are still and both tool points are at least 4 cm from the
# pen, the strip and the pack, held for 1 s (the trial then ends).

PACK_SHIFT, PACK_TILT = 0.01, 10.0
PEN_DROP_DEG = 60.0
WELD_R = 0.0035
HANDS_OFF = 0.04


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tilt_deg(state) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(_rot(state.quat_xyzw)[2, 2])))))


def _local(state, p):
    return _rot(state.quat_xyzw).T @ (np.asarray(p, float) - np.asarray(state.pos, float))


def strip_on(objects) -> bool:
    pk, st = objects["battery_pack"], objects["nickel_strip"]
    c = _local(pk, st.pos)
    R = _rot(pk.quat_xyzw).T @ _rot(st.quat_xyzw)
    bar_deg = math.degrees(abs(math.atan2(R[1, 0], R[0, 0])))
    bar_deg = min(bar_deg, 180.0 - bar_deg)
    return (tilt_deg(st) < 10.0 and abs(c[2] - CAP_TOP) <= 0.002 and math.hypot(c[0], c[1]) <= 0.004
            and bar_deg <= 10.0)


def needle_tips(pen):
    """World positions of the two needle tips (each pressed in by its joint's travel)."""
    R, p = _rot(pen.quat_xyzw), np.asarray(pen.pos, float)
    j = getattr(pen, "joints", None) or {}
    return [p + R @ np.array([x, 0.0, -NEEDLE_OUT + float(j.get(n, 0.0))])
            for n, x in (("needle_a", NEEDLE_X), ("needle_b", -NEEDLE_X))]


def fired(pen) -> bool:
    j = getattr(pen, "joints", None) or {}
    return all(float(j.get(n, 0.0)) >= FIRE - 1e-9 for n in ("needle_a", "needle_b"))


def weld_cell(objects):
    """The cell (1-4) a weld lands on at this step, else None."""
    pen, pk = objects["weld_pen"], objects["battery_pack"]
    if not fired(pen) or not strip_on(objects):
        return None
    tips = [_local(pk, t) for t in needle_tips(pen)]
    strip_top = _local(pk, objects["nickel_strip"].pos)[2] + STRIP_T
    for k, (cx, cy) in enumerate(CELLS, start=1):
        if all(math.hypot(t[0] - cx, t[1] - cy) <= WELD_R and abs(t[2] - strip_top) <= 0.0015 for t in tips):
            return k
    return None


SEAT = np.array(PEN_HOME)


def pen_home(pen) -> bool:
    """Standing in its holster: where the pen's axis crosses the rim's height is within 6 mm of the cup's centre (the
    cup has 4.5 mm of play), its guard within 4 mm of the rim, and it leans under 12 degrees."""
    guard = np.asarray(pen.pos, float) + _rot(pen.quat_xyzw) @ np.array([0.0, 0.0, GUARD_Z0])
    return (math.hypot(guard[0] - HOLSTER_POS[0], guard[1] - HOLSTER_POS[1]) <= 0.006
            and abs(guard[2] - (TOP + HOLSTER_RIM)) <= 0.004 and tilt_deg(pen) < 12.0)


def evaluate(objects, trial):
    pk, pen = objects["battery_pack"], objects["weld_pen"]
    if (math.hypot(float(pk.pos[0]) - PACK_POS[0], float(pk.pos[1]) - PACK_POS[1]) > PACK_SHIFT
            or tilt_deg(pk) > PACK_TILT):
        trial.defect("D1 pack knocked")
    if tilt_deg(pen) > PEN_DROP_DEG:
        trial.defect("D2 pen dropped")
    on = strip_on(objects)
    k = weld_cell(objects)
    if k:
        trial.info.setdefault("welds_at", {}).setdefault(k, round(trial.t, 2))
    welds = sum(trial.once(f"cell_{i}", k == i) for i in range(1, 5))
    taken = trial.once("pen_out", float(pen.pos[2]) > SEAT[2] + 0.03)
    home = taken and pen_home(pen)
    guard = np.asarray(pen.pos, float) + _rot(pen.quat_xyzw) @ np.array([0.0, 0.0, GUARD_Z0])
    sc = _local(objects["battery_pack"], objects["nickel_strip"].pos)
    trial.info.update(strip_on=bool(on), welds=int(welds), pen_home=bool(home),
                      strip_off_mm=round(1000 * math.hypot(sc[0], sc[1]), 1),
                      pen_rim_mm=round(1000 * math.hypot(guard[0] - HOLSTER_POS[0], guard[1] - HOLSTER_POS[1]), 1),
                      pen_guard_dz_mm=round(1000 * (guard[2] - TOP - HOLSTER_RIM), 1), pen_tilt=round(tilt_deg(pen), 1))
    partial = 0.2 * on + 0.15 * welds + 0.1 * home
    pts = [np.asarray(objects[n].pos, float) for n in ("weld_pen", "nickel_strip", "battery_pack")]
    pts += [np.asarray(pen.pos, float) + _rot(pen.quat_xyzw) @ np.array([0.0, 0.0, z]) for z in (GRIP_Z, PEN_TOP)]
    done = (on and welds == 4 and home and all(trial.still(n) for n in ("weld_pen", "nickel_strip", "battery_pack"))
            and trial.clear_of(pts, HANDS_OFF))
    return partial, done


TRIAL = Trial(evaluate, trace_env="WELD_TRACE")
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def success(objects) -> bool:
    return TRIAL.success(objects)


def score(objects) -> float:
    return TRIAL.score(objects)


def distance(objects) -> float:
    return float(np.linalg.norm(np.asarray(objects["nickel_strip"].pos[:2], float) - np.asarray(PACK_POS)))


# -- the demonstration ---------------------------------------------------------------------------------------------------
#
# Scripted, open loop, fingers pointing down. The left arm takes the strip by its tab, lays it on the pack and goes
# back. The right arm then takes the pen out of its holster just above the guard, carries it over the pack and, for
# each cell, brings the needles down onto the strip over the + cap and presses them in 3 mm (the welder fires at 2),
# holds, and lifts; then it stands the pen back in its holster and goes back.

TIP = 0.013  # fingertips below the tool point
STRIP_GRIP = TIP + 0.0075  # tool point above the strip's base: fingertips 7.5 mm up the tab, above the tray's 5 mm walls
CAP_Z = TOP + CAP_TOP
PRESS = 0.008  # how far below the strip the demo aims the needle tips (the arm gives about 3.5 mm under the load)
PEN_TRIM = (0.006, 0.0065)  # the held pen lands this much short and to the right of where the arm aims (measured)
HOVER = 0.02
L_CLEAR = (0.15, 0.27, TOP + 0.15)
R_CLEAR = (0.15, -0.27, TOP + 0.15)


def _pen_tcp(tip_x, tip_y, tip_z):
    """Tool point holding the pen upright with its needle tips (pair centre) at (tip_x, tip_y, tip_z), aimed past the
    target by PEN_TRIM: under the pen and its lead the arm's drives leave it short (measured in the demo run)."""
    return (tip_x + PEN_TRIM[0], tip_y + PEN_TRIM[1], tip_z + NEEDLE_OUT + GRIP_Z)


def demo():
    sx, sy, sz = STRIP_START
    px, py = PACK_POS
    lay_z = CAP_Z + 0.0005  # the strip's underside, just above the caps, at release
    left = [Waypoint(1.8, (sx, sy, sz + STRIP_GRIP + 0.04), FINGER_OPEN, "over the strip's tab"),
            Waypoint(1.2, (sx, sy, sz + STRIP_GRIP), FINGER_OPEN, "down round the tab"),
            Waypoint(0.8, (sx, sy, sz + STRIP_GRIP), FINGER_CLOSED, "pinch the tab"),
            Waypoint(1.0, (sx, sy, sz + STRIP_GRIP + 0.04), FINGER_CLOSED, "lift it out of the tray"),
            Waypoint(2.0, (px, py, lay_z + STRIP_GRIP + 0.03), FINGER_CLOSED, "carry it over the pack"),
            Waypoint(1.2, (px, py, lay_z + STRIP_GRIP), FINGER_CLOSED, "lower it onto the caps"),
            Waypoint(0.6, (px, py, lay_z + STRIP_GRIP), FINGER_CLOSED, "let the arm catch up"),
            Waypoint(0.6, (px, py, lay_z + STRIP_GRIP), FINGER_OPEN, "let go"),
            Waypoint(0.8, (px, py, lay_z + STRIP_GRIP + 0.04), FINGER_OPEN, "straight up"),
            Waypoint(1.5, L_CLEAR, FINGER_OPEN, "clear")]
    t_left = sum(w.duration for w in left)

    hx, hy, hz = PEN_HOME
    grip = (hx, hy, hz + GRIP_Z)
    out_z = hz + GRIP_Z + 0.065  # needles above the rim
    right = [Waypoint(t_left - 1.0, R_CLEAR, FINGER_OPEN, "wait for the strip"),
             Waypoint(1.5, (hx, hy, grip[2] + 0.05), FINGER_OPEN, "over the pen"),
             Waypoint(1.2, grip, FINGER_OPEN, "down round its grip, just above the guard"),
             Waypoint(0.8, grip, FINGER_CLOSED, "take hold"),
             Waypoint(1.4, (hx, hy, out_z), FINGER_CLOSED, "lift it out of the holster")]
    strip_top = CAP_Z + STRIP_T
    for k, (cx, cy) in enumerate(CELLS, start=1):
        x, y = px + cx, py + cy
        right += [Waypoint(1.6 if k == 1 else 1.0, _pen_tcp(x, y, strip_top + HOVER), FINGER_CLOSED,
                           f"needles over cell {k}'s + cap"),
                  Waypoint(1.0, _pen_tcp(x, y, strip_top + 0.002), FINGER_CLOSED, "down to just above the strip"),
                  Waypoint(1.0, _pen_tcp(x, y, strip_top - PRESS), FINGER_CLOSED, "press both needles in: weld"),
                  Waypoint(0.5, _pen_tcp(x, y, strip_top - PRESS), FINGER_CLOSED, "hold"),
                  Waypoint(0.6, _pen_tcp(x, y, strip_top + HOVER), FINGER_CLOSED, "lift off")]
    tx, ty = hx + PEN_TRIM[0], hy + PEN_TRIM[1]
    right += [Waypoint(1.8, (tx, ty, out_z), FINGER_CLOSED, "back over the holster"),
              Waypoint(1.4, (tx, ty, grip[2] + 0.002), FINGER_CLOSED, "lower it in, guard just above the rim"),
              Waypoint(0.6, (tx, ty, grip[2] + 0.002), FINGER_OPEN, "let go"),
              Waypoint(1.0, (hx, hy, grip[2] + 0.07), FINGER_OPEN, "straight up, clear of the pen"),
              Waypoint(1.5, R_CLEAR, FINGER_OPEN, "clear"),
              Waypoint(2.0, R_CLEAR, FINGER_OPEN, "wait")]
    t_right = sum(w.duration for w in right)
    left.append(Waypoint(max(0.5, t_right - t_left), L_CLEAR, FINGER_OPEN, "wait"))
    return {"left": left, "right": right}


# -- offline cases (check_task.py) ---------------------------------------------------------------------------------------

def CASES():
    px, py = PACK_POS
    strip_on_pack = (px, py, TOP + CAP_TOP)
    flat = (0, 0, 0, 1.0)

    def st(strip=STRIP_START, pen=PEN_HOME, pj=None, pack=(px, py, TOP), pq=flat, penq=flat, sq=flat):
        return {"battery_pack": fake_state(pack, quat=pq),
                "nickel_strip": fake_state(strip, quat=sq),
                "strip_tray": fake_state((TRAY_POS[0], TRAY_POS[1], TOP)),
                "weld_pen": fake_state(pen, quat=penq, joints=pj or {"needle_a": 0.0, "needle_b": 0.0}),
                "pen_holster": fake_state((HOLSTER_POS[0], HOLSTER_POS[1], TOP)),
                "welder_unit": fake_state((0.585, -0.2, TOP)),
                "weld_lead": fake_state((0.4, -0.2, TOP))}

    def pen_on(k, press=PRESS, strip_ok=True):
        cx, cy = CELLS[k - 1]
        tip_z = TOP + CAP_TOP + STRIP_T
        pos = (px + cx, py + cy, tip_z + NEEDLE_OUT - press)
        return st(strip_on_pack if strip_ok else STRIP_START, pos, {"needle_a": press, "needle_b": press})

    up = (PEN_HOME[0], PEN_HOME[1], PEN_HOME[2] + 0.06)
    lifted = st(strip_on_pack, up)
    weld_seq = [lifted] + sum(([pen_on(k)] * 2 + [lifted] for k in (1, 2, 3, 4)), [])
    home = st(strip_on_pack, PEN_HOME)
    far = lambda: [np.array((0.15, 0.27, 0.9)), np.array((0.15, -0.27, 0.9))]
    on_pen = lambda: [np.array((0.15, 0.27, 0.9)), np.array(PEN_HOME) + (0, 0, GRIP_Z)]
    tip45 = (math.sin(math.radians(22.5)), 0, 0, math.cos(math.radians(22.5)))
    tip70 = (math.sin(math.radians(35)), 0, 0, math.cos(math.radians(35)))
    return [
        ("doing nothing", [st()] * 30, far, 0.0, False),
        ("strip laid, nothing welded", [st(strip_on_pack)] * 30, far, 0.2, False),
        ("strip laid, four welds, pen home, hands off, held 1 s", weld_seq + [home] * 30, far, 1.0, True),
        ("all done but still holding the pen", weld_seq + [home] * 30, on_pen, 0.9, False),
        ("all done 0.75 s before the run limit", weld_seq + [home] * 15, far, 0.9, True),
        ("pressed only 1 mm (under 200 g): no welds", [lifted] + [pen_on(1, 0.001)] * 3 + [home] * 30, far,
         0.3, False),
        ("needles on cell 2 with no strip: no weld", [st(STRIP_START, up)] + [pen_on(2, strip_ok=False)] * 3
         + [st(STRIP_START, PEN_HOME)] * 30, far, 0.1, False),
        ("two welds, pen left out on the pack", weld_seq[:7] + [lifted] * 30, far, 0.5, False),
        ("pack knocked 2 cm", [st(strip_on_pack, pack=(px + 0.02, py, TOP))] * 3 + weld_seq + [home] * 30, far,
         0.0, False),
        ("pack tipped 45 degrees", [st(pq=tip45)] * 30, far, 0.0, False),
        ("pen dropped", [st(pen=up, penq=tip70)] * 30, far, 0.0, False),
    ]
