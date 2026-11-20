"""The rear bench of a car, seen from its open rear door: the robot leans in across the door-side outboard seat, as a
parent does to reach the centre seat. In the centre seating position a rear-facing infant car-seat base is installed
(fixed: belted to under 2.5 cm of movement), its front pushed back against the vehicle seat back as its manual requires.
An empty infant carrier sits on it. Carrier and base
are a full-size reconstruction of the Graco SnugRide 35 Lite LX and its matching base (carrier envelope 70 x 38 x 47 cm
with the handle up, 3.45 kg empty): a steel locking bar, 12 mm across, runs across the carrier's underside at its front
(foot) end, and the base has a lock at each end of that bar. Each lock is a pocket with a sprung pawl: the pawl's ramp is
cammed back as the bar is pushed down into the pocket, then springs forward over the bar and holds it (a real latch:
lifting the bar presses on the pawl's flat underside, which the spring does not oppose).

World frame: x forward from the arms, into the car; y to the arms' left, here towards the front of the car; z up.
The cabin around the seat is fixed structure: the rear bench's cushion (sloping 5 degrees down towards the seat back)
and its seat back (reclined 20 degrees) behind the carrier's front end, the two front seats' backs (reclined 20 degrees)
and the centre console between them beyond its head end, and the door opening's two pillars. The roof is not modelled
(see the README).

Why the centre seat: the arms always start at a fixed ready pose with the grippers about 33 cm in front of their bases and
26 cm up. A carrier in the door-side outboard seat would enclose that pose, and one far enough in to clear it has its far
lock about 70 cm out, beyond reach. The centre position (which the manual permits) is the outboard-most one that both
clears the ready pose and keeps the door-side lock in reach.

Fault: the carrier was set down skewed. The inboard end of its bar is in its pocket under the pawl; the door-side end
rests on top of that pawl's ramp, so it sits about 1.5 cm high and the carrier is rolled about 2.7 degrees. A carrier
latched on one side only can twist off the base in a crash. Pushing the door-side front corner down (about 20 N,
estimated) cams the pawl back and the bar clicks in. The release handle is not modelled.

Distractors: a bottle of water and a takeaway coffee cup on the floor of the rear footwell.
"""

import math
import pathlib
import sys

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- carrier and base, in the base's own frame (metres): x from the bar towards the carrier's head end, y across ----------
ZB = 0.12  # the locking bar's axis above the base's underside when the carrier is seated
R_BAR = 0.006  # 12 mm steel bar
BAR_HALF = 0.17
LOCK_Y = 0.155  # the locks' centres, |y|
LOCK_W = 0.040  # pocket width along y (0.135 .. 0.175)
PAWL_W = 0.030
SLOT = 0.008  # the pocket's half-width in x (bar radius 6 mm, 2 mm play)
GAP = 0.002  # pawl underside above the seated bar's top
Z_PAWL = R_BAR + GAP  # pawl underside, above the bar's axis
TIP = -0.004  # the closed pawl's tip, x from the bar's axis (covers the bar's back two thirds)
RAMP_DEG = 60.0  # ramp face, from horizontal
PAWL_H = 0.020
PAWL_BACK = 0.040
PAWL_TRAVEL = 0.016
PAWL_K = 800.0  # N/m, the pawl spring in the trial (estimated, not a measured product force)
PAWL_PRELOAD = 8.0  # N: the spring holds the pawl forward against its stop with this force
PAWL_K_SETTLE = 40000.0  # N/m: only during the reset's 0.5 s settle, so the carrier settles on the ramp
CARRIER_MASS = 3.45  # kg, empty (7.6 lb, measured carrier-only weight in a published review)
CARRIER_COM = (0.26, 0.0, 0.11)

# -- where the base is installed (world) ----------------------------------------------------------------------------------
XC = 0.58  # the base's centreline, 58 cm in front of the arm bases: the centre seating position
YB = -0.335  # the locking bar's axis (the base's front is 2.5 cm behind it, against the seat back)
HB = 0.035  # the base's underside above the table top (it sits on the cushion, pressed into the foam at its rear)
BASE_O = np.array([XC, YB, TOP + HB])
YAW = math.radians(90.0)  # base +x (towards the carrier's head) is world +y (the front of the car)
RZ = np.array([[math.cos(YAW), -math.sin(YAW), 0.0], [math.sin(YAW), math.cos(YAW), 0.0], [0.0, 0.0, 1.0]])
NEAR, FAR = 1, -1  # base +y is world -x: the base's left lock (+y) is on the door side, its right lock (-y) inboard
FAULT = NEAR  # the door-side end of the bar rests on its pawl's ramp; the inboard end is latched

# -- the cabin (world, relative to the table top) -------------------------------------------------------------------------
SEAT_X = (0.10, 1.18)  # the rear bench across the car: door-side outboard seat (to 0.37), centre (to 0.79), far outboard
SEAMS_X = (0.37, 0.79)
BIGHT_Y = YB - 0.025  # where the rear seat back meets the cushion: the base's front touches it
CUSHION_Z = 0.030  # the cushion's top at the bight, above the table top
CUSHION_DEPTH = 0.50
CUSHION_DEG = 5.0  # the cushion rises 5 degrees towards its front edge
BACK_DEG = 20.0  # rear seat back recline
BACK_T, BACK_H = 0.14, 0.66
FRONT_Y = 0.50  # the front seats' backs: rear face at the foot, 20 cm above the table top
FRONT_FOOT_Z = 0.20
FRONT_DEG = 20.0
FRONT_SEATS_X = ((0.0, 0.44), (0.74, 1.18))  # the door-side front seat and the far one
CONSOLE_X = (0.46, 0.72)  # centre console and armrest between the front seats
CONSOLE = (0.48, 0.32)  # its rear face's y, its top's height above the table top
PILLAR_Y = (-0.47, 0.57)  # the door opening's rear and front pillars (their centres), beside the arm bases
PILLAR_X = -0.06
PILLAR_H = 0.90

LOOKS = {
    "shell": m.Look(m.rgb(0.16, 0.17, 0.19), roughness=0.55),
    "fabric": m.Look(m.rgb(0.42, 0.45, 0.50), roughness=0.95),
    "trim": m.Look(m.rgb(0.10, 0.10, 0.11), roughness=0.8),
    "strap": m.Look(m.rgb(0.07, 0.07, 0.08), roughness=0.9),
    "buckle": m.Look(m.rgb(0.75, 0.10, 0.10), roughness=0.4),
    "handle": m.Look(m.rgb(0.20, 0.21, 0.23), roughness=0.5),
    "steel": m.Look(m.rgb(0.70, 0.71, 0.73), metallic=0.9, roughness=0.3),
    "base": m.Look(m.rgb(0.13, 0.13, 0.14), roughness=0.6),
    "lock": m.Look(m.rgb(0.30, 0.31, 0.33), roughness=0.5),
    "pawl": m.Look(m.rgb(0.55, 0.56, 0.58), metallic=0.5, roughness=0.4),
    "level": m.Look(m.rgb(0.15, 0.45, 0.85), roughness=0.4),
    "upholstery": m.Look(m.rgb(0.52, 0.47, 0.40), roughness=0.95),
    "seam": m.Look(m.rgb(0.40, 0.36, 0.31), roughness=0.95),
    "pillar": m.Look(m.rgb(0.62, 0.62, 0.60), roughness=0.7),
}


def _side_wall(y0, y1):
    """The shell's side, a profile in x-z (foot end low at -x, head end high at +x), between y0 and y1."""
    outline = [(-0.04, 0.03), (0.60, 0.03), (0.66, 0.12), (0.665, 0.30), (0.63, 0.36), (0.56, 0.33), (0.35, 0.20),
               (0.10, 0.15), (-0.04, 0.15)]
    # the extrusion helper makes (x, p, z) with p the profile's height; turning +90 about x maps (x, p, z) to (x, -z, p)
    return m.prism(outline, -y1, -y0).turned(90.0, "x")


def _carrier():
    parts, cols = [], []
    W = 0.150  # inner face of the side walls
    for s in (-1, 1):
        y0, y1 = (W, W + 0.02) if s > 0 else (-W - 0.02, -W)
        wall = _side_wall(y0, y1)
        parts.append((f"wall{s + 1}", wall, "shell"))
        cols.append(m.col_hull(f"wall{s + 1}_c", wall))
        # moulded foot-end corner lip, where a hand pushes the carrier down
        lip = m.box((0.08, 0.05, 0.02), (0.0, s * 0.125, 0.14))
        parts.append((f"lip{s + 1}", lip, "shell"))
        cols.append(m.col_box(f"lip{s + 1}_c", (0.08, 0.05, 0.02), (0.0, s * 0.125, 0.14)))
        runner = (0.47, 0.06, 0.036)
        rc = (0.265, s * 0.09, -R_BAR + 0.018)
        parts.append((f"runner{s + 1}", m.box(runner, rc), "shell"))
        cols.append(m.col_box(f"runner{s + 1}_c", runner, rc))
        # handle hub on the shell's side
        parts.append((f"hub{s + 1}", m.cylinder(0.035, 0.012, (0.30, s * (W + 0.026), 0.17), axis="y"), "handle"))
    bottom = (0.67, 2 * W, 0.02)
    parts.append(("bottom", m.box(bottom, (0.295, 0.0, 0.04)), "shell"))
    cols.append(m.col_box("bottom_c", bottom, (0.295, 0.0, 0.04)))
    foot = (0.02, 2 * W, 0.10)
    parts.append(("foot", m.box(foot, (-0.03, 0.0, 0.10)), "shell"))
    cols.append(m.col_box("foot_c", foot, (-0.03, 0.0, 0.10)))
    head = (0.035, 2 * W, 0.30)
    parts.append(("head", m.box(head, (0.6475, 0.0, 0.18)), "shell"))
    cols.append(m.col_box("head_c", head, (0.6475, 0.0, 0.18)))
    # padded seat: pan, and a reclined back up to the head end
    pan = m.box((0.32, 2 * W - 0.01, 0.03), (0.15, 0.0, 0.065))
    parts.append(("pan", pan, "fabric"))
    cols.append(m.col_box("pan_c", (0.32, 2 * W - 0.01, 0.03), (0.15, 0.0, 0.065)))
    back_len = math.hypot(0.32, 0.25)
    back_deg = math.degrees(math.atan2(0.25, 0.32))
    back = m.box((back_len, 2 * W - 0.01, 0.03), (0.0, 0.0, 0.0)).turned(-back_deg, "y").moved((0.47, 0.0, 0.19))
    parts.append(("back", back, "fabric"))
    cols.append(m.col_hull("back_c", back))
    # harness: two shoulder straps and a crotch strap to a buckle
    for s in (-1, 1):
        strap = m.box((0.20, 0.025, 0.004), (0.0, 0.0, 0.0)).turned(-back_deg, "y").moved((0.42, s * 0.05, 0.172))
        parts.append((f"strap{s + 1}", strap, "strap"))
    parts.append(("crotch", m.box((0.12, 0.025, 0.004), (0.20, 0.0, 0.082)), "strap"))
    parts.append(("buckle", m.box((0.04, 0.035, 0.012), (0.26, 0.0, 0.086)), "buckle"))
    # padded trim along the shell's rim
    for s in (-1, 1):
        parts.append((f"rim{s + 1}", m.tube([(-0.03, s * 0.16, 0.155), (0.10, s * 0.16, 0.155), (0.35, s * 0.16, 0.205),
                                              (0.56, s * 0.16, 0.335), (0.63, s * 0.16, 0.365)], 0.010), "trim"))
    # carry handle, upright (the manual requires it upright in the vehicle), pivoting at the hubs (fixed here)
    hy = W + 0.035
    arc = [(0.30, hy, 0.17), (0.30, hy, 0.36), (0.30, 0.15, 0.43), (0.30, 0.07, 0.45), (0.30, -0.07, 0.45),
           (0.30, -0.15, 0.43), (0.30, -hy, 0.36), (0.30, -hy, 0.17)]
    parts.append(("handle", m.tube(arc, 0.014), "handle"))
    cols.append(m.col_hull("handle_l_c", m.tube(arc[:3], 0.014)))
    cols.append(m.col_hull("handle_t_c", m.tube(arc[2:6], 0.014)))
    cols.append(m.col_hull("handle_r_c", m.tube(arc[5:], 0.014)))
    # the locking bar
    bar = m.cylinder(R_BAR, 2 * BAR_HALF, (0.0, 0.0, 0.0), axis="y", segments=20)
    parts.append(("bar", bar, "steel"))
    cols.append(m.col_hull("bar_c", m.cylinder(R_BAR, 2 * BAR_HALF, (0.0, 0.0, 0.0), axis="y", segments=16)))
    return parts, cols


def _pawl_mesh(s):
    """A lock's pawl, closed: flat underside Z_PAWL above the bar's axis, a 60 degree ramp facing the bar."""
    run = PAWL_H / math.tan(math.radians(RAMP_DEG))
    outline = [(TIP, ZB + Z_PAWL), (PAWL_BACK, ZB + Z_PAWL), (PAWL_BACK, ZB + Z_PAWL + PAWL_H),
               (TIP + run, ZB + Z_PAWL + PAWL_H)]
    y = s * LOCK_Y
    return m.prism(outline, -(y + PAWL_W / 2), -(y - PAWL_W / 2)).turned(90.0, "x")


def _base():
    parts, cols, links = [], [], []
    body_h = ZB - 0.03
    body = (0.45, 0.36, body_h)
    parts.append(("body", m.box(body, (0.20, 0.0, body_h / 2)), "base"))
    cols.append(m.col_box("body_c", body, (0.20, 0.0, body_h / 2)))
    for s in (-1, 1):
        # side rails at the rear, outside the carrier's shell
        rail = (0.24, 0.015, 0.09)
        parts.append((f"rail{s + 1}", m.box(rail, (0.30, s * 0.1855, body_h + 0.045 - 0.03)), "base"))
        cols.append(m.col_box(f"rail{s + 1}_c", rail, (0.30, s * 0.1855, body_h + 0.045 - 0.03)))
        # rear pads under the carrier's runners
        pad = (0.10, 0.05, 0.024)
        pc = (0.33, s * 0.09, ZB - R_BAR - 0.012)
        parts.append((f"pad{s + 1}", m.box(pad, pc), "lock"))
        cols.append(m.col_box(f"pad{s + 1}_c", pad, pc))
        y = s * LOCK_Y
        # lock pocket: floor under the bar, front wall, rear block below the pawl
        floor = (2 * SLOT + 0.008, LOCK_W, 0.024)
        fc = (0.0, y, ZB - R_BAR - 0.012)
        front = (0.016, LOCK_W, 0.058)  # a guide wall up to the pawl's top: the bar cannot ride forward off the ramp
        frc = (-SLOT - 0.008, y, ZB - 0.03 + 0.029)
        rear = (PAWL_BACK - SLOT, LOCK_W, ZB + R_BAR - (ZB - 0.03))
        rc = ((PAWL_BACK + SLOT) / 2, y, (ZB + R_BAR + ZB - 0.03) / 2)
        for nm, size, c in (("floor", floor, fc), ("front", front, frc), ("rear", rear, rc)):
            parts.append((f"{nm}{s + 1}", m.box(size, c), "lock"))
            cols.append(m.col_box(f"{nm}{s + 1}_c", size, c))
        cover = (PAWL_BACK - SLOT + 0.01, LOCK_W + 0.006, 0.006)
        parts.append((f"cover{s + 1}", m.box(cover, ((PAWL_BACK + SLOT) / 2 + 0.008, y, ZB + Z_PAWL + PAWL_H + 0.004)), "lock"))
        pawl = _pawl_mesh(s)
        links.append(m.slider(f"pawl{'L' if s > 0 else 'R'}", "X", 0.0, PAWL_TRAVEL,
                              parts=(("pawl", pawl, "pawl"),), colliders=(m.col_hull(f"pawl{s + 1}_c", pawl),),
                              at=(0.0, y, ZB + Z_PAWL), mass=0.08, spring=PAWL_K_SETTLE, damping=3.0, effort=2000.0,
                              friction=0.05, armature=0.005))
    # recline level indicator window on the base's door-side face (cosmetic)
    parts.append(("level", m.box((0.04, 0.002, 0.02), (0.10, 0.181, body_h - 0.02)), "level"))
    return parts, cols, links


def _slab(size, deg, anchor, anchor_local):
    """A box of `size` turned `deg` about world x, moved so that its local point `anchor_local` lands on `anchor`."""
    a = math.radians(deg)
    rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    c = np.asarray(anchor, float) - rx @ np.asarray(anchor_local, float)
    return m.box(size, (0.0, 0.0, 0.0)).turned(deg, "x").moved(tuple(float(v) for v in c))


def _cabin():
    """The rear bench (cushion, seat back), the front seats' backs and the console between them, and the door opening's
    pillars, all fixed. The frame is the world's, with its origin on the table top."""
    parts, cols = [], []
    wx = SEAT_X[1] - SEAT_X[0]
    cx = (SEAT_X[0] + SEAT_X[1]) / 2
    # cushion: 20 cm thick, its top through (bight, CUSHION_Z), rising towards its front edge; it runs 6 cm under the back
    d = CUSHION_DEPTH + 0.06
    cush = _slab((wx, d, 0.20), CUSHION_DEG, (cx, BIGHT_Y, CUSHION_Z), (0.0, -d / 2 + 0.06, 0.10))
    parts.append(("cushion", cush, "upholstery"))
    cols.append(m.col_hull("cushion_c", cush))
    # the bench's seat back, reclined: its face meets the cushion at the bight
    back = _slab((wx, BACK_T, BACK_H), BACK_DEG, (cx, BIGHT_Y, CUSHION_Z - 0.04), (0.0, BACK_T / 2, -BACK_H / 2))
    parts.append(("seatback", back, "upholstery"))
    cols.append(m.col_hull("seatback_c", back))
    # seams between the three seating positions (cosmetic)
    for i, x in enumerate(SEAMS_X):
        parts.append((f"seam_back{i}", _slab((0.012, 0.004, BACK_H - 0.08), BACK_DEG, (x, BIGHT_Y + 0.001, CUSHION_Z + 0.02),
                                             (0.0, 0.0, -(BACK_H - 0.08) / 2)), "seam"))
        parts.append((f"seam_cushion{i}", m.box((0.012, CUSHION_DEPTH - 0.04, 0.004), (0.0, 0.0, 0.0)).turned(CUSHION_DEG, "x")
                      .moved((x, BIGHT_Y + 0.02 + (CUSHION_DEPTH - 0.04) / 2,
                              CUSHION_Z + 0.001 + (0.02 + (CUSHION_DEPTH - 0.04) / 2) * math.tan(math.radians(CUSHION_DEG)))),
                      "seam"))
    # the two front seats' backs, reclined towards the carrier, on their cushions
    for i, (x0, x1) in enumerate(FRONT_SEATS_X):
        fw, fx = x1 - x0, (x0 + x1) / 2
        fback = _slab((fw, 0.12, 0.62), FRONT_DEG, (fx, FRONT_Y, FRONT_FOOT_Z), (0.0, -0.06, -0.31))
        parts.append((f"frontback{i}", fback, "upholstery"))
        cols.append(m.col_hull(f"frontback{i}_c", fback))
        fcush = (fw, 0.48, 0.20)
        parts.append((f"frontcushion{i}", m.box(fcush, (fx, FRONT_Y + 0.26, 0.10)), "upholstery"))
        cols.append(m.col_box(f"frontcushion{i}_c", fcush, (fx, FRONT_Y + 0.26, 0.10)))
    # centre console with its armrest, between the front seats
    con = (CONSOLE_X[1] - CONSOLE_X[0], 0.50, CONSOLE[1])
    cc = ((CONSOLE_X[0] + CONSOLE_X[1]) / 2, CONSOLE[0] + 0.25, CONSOLE[1] / 2)
    parts.append(("console", m.box(con, cc), "trim"))
    cols.append(m.col_box("console_c", con, cc))
    # door opening: the rear and front pillars, 6 x 8 cm
    for i, y in enumerate(PILLAR_Y):
        parts.append((f"pillar{i}", m.box((0.06, 0.08, PILLAR_H), (PILLAR_X, y, PILLAR_H / 2)), "pillar"))
        cols.append(m.col_box(f"pillar{i}_c", (0.06, 0.08, PILLAR_H), (PILLAR_X, y, PILLAR_H / 2)))
    return parts, cols


def ramp_rest_height(q=0.0):
    """Height of the bar's axis above its seated height when it rests on a pawl's ramp, the pawl q metres back."""
    a = math.radians(RAMP_DEG)
    x0, z0 = TIP + q, Z_PAWL
    return z0 + ((0.0 - x0) * math.sin(a) + R_BAR) / math.cos(a)


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def _start_pose():
    """Carrier rolled about the latched (inboard) lock, bar seated there, until the door-side end of its bar rests on that
    side's pawl ramp. Returns (quat xyzw, origin, roll in degrees)."""
    h = ramp_rest_height(0.0)
    phi = FAULT * math.asin(h / (2 * LOCK_Y))  # about base +x; positive raises the base's +y (door) side
    c, s = math.cos(phi), math.sin(phi)
    rx = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    pivot = np.array([0.0, -FAULT * LOCK_Y, ZB])
    origin_b = pivot + rx @ (np.array([0.0, 0.0, ZB]) - pivot) + np.array([0.0, 0.0, 0.0015])
    q = _quat_mul((0.0, 0.0, math.sin(YAW / 2), math.cos(YAW / 2)), (math.sin(phi / 2), 0.0, 0.0, math.cos(phi / 2)))
    return q, BASE_O + RZ @ origin_b, math.degrees(phi)


def to_world(p_base):
    return BASE_O + RZ @ np.asarray(p_base, float)


def to_base(p_world):
    return (np.asarray(p_world, float) - BASE_O) @ RZ  # RZ.T applied to each row


START_Q, START_POS, START_ROLL = _start_pose()
CARRIER_PARTS, CARRIER_COLS = _carrier()
BASE_PARTS, BASE_COLS, BASE_LINKS = _base()
CABIN_PARTS, CABIN_COLS = _cabin()

CABIN = m.fixed("cabin", (0.0, 0.0, TOP), CABIN_PARTS, CABIN_COLS, LOOKS, friction=(1.0, 0.8))
BASE = m.mechanism("base", tuple(float(v) for v in BASE_O), BASE_LINKS, LOOKS, frame_parts=BASE_PARTS,
                   frame_colliders=BASE_COLS, yaw_deg=90.0, friction=(0.4, 0.35), solver_iters=48)
CARRIER = m.free("carrier", tuple(float(v) for v in START_POS), CARRIER_PARTS, CARRIER_COLS, LOOKS, mass=CARRIER_MASS,
                 friction=(0.5, 0.45), solver_iters=48, start_quat=tuple(float(v) for v in START_Q), com=CARRIER_COM)

WATER_XY = (0.15, 0.21)
CUP_XY = (0.26, 0.23)
OBJECTS = [
    CABIN,
    BASE,
    CARRIER,
    # Distractors (SimReady props) on the rear footwell's floor, which scoring ignores. pos is each one's origin (bottom centre).
    SimReady("water", pos=(WATER_XY[0], WATER_XY[1], TOP), size=(0.068, 0.068, 0.245),
             mass=0.52, package="food_beverage_bottledWater_b01", usd="sm_food_beverage_bottledWater_b01_01.usd"),
    SimReady("coffee_cup", pos=(CUP_XY[0], CUP_XY[1], TOP), size=(0.085, 0.085, 0.117),
             mass=0.25, package="food_beverage_coffeeCup_a01", usd="sm_food_beverage_coffeeCup_a01_01.usd"),
]

# -- scoring ------------------------------------------------------------------------------------------------------------
#
# The trial is judged on its final state only. success() never ends the episode early, so the run always goes to its
# step limit, and score() at the last step is the result: 1.0 needs the ready conditions to hold continuously over the
# last 3 seconds of the run. A carrier that was ready earlier and was then moved, tilted or held scores what its final
# state earns.

SUCCESS_HOLD_S = 0.0
SCORE_OVER = "end"
HOLD_SECONDS = 3.0
PAWL_CLOSED = 0.0015  # a pawl within this of fully forward is over the bar
BAR_SEATED = 0.003  # bar end's axis no more than 3 mm above its seated height
RUNNER_TOL = 0.003  # rear runners within 3 mm of their pads
LEVEL_DEG = 1.0
CLEAR = 0.03
ENVELOPE = (np.array([-0.04, -0.21, -R_BAR]), np.array([0.665, 0.21, 0.47]))  # the carrier's box, its own frame
TIP_OVER_DEG = 25.0  # the carrier tipped this far: it would have spilled the infant or fallen
DROP = 0.05  # either end of the bar more than 5 cm below its seated height: off the base, on the seat or floor


def bar_end(cs, s):
    """The bar's axis at lock side s (NEAR or FAR), in the base's frame, relative to its seated position."""
    return to_base(cs.to_world(np.array([0.0, s * LOCK_Y, 0.0]))[0]) - np.array([0.0, s * LOCK_Y, ZB])


def latched(cs, bs, s):
    """The bar is down in the pocket at side s and that side's pawl is forward over it."""
    pawl = bs.joint("pawlL" if s > 0 else "pawlR")
    d = bar_end(cs, s)
    return bool(pawl <= PAWL_CLOSED and d[2] <= BAR_SEATED and abs(d[0]) <= SLOT)


def runner_gaps(cs):
    pts = np.array([[x, s * 0.09, -R_BAR] for x in (0.29, 0.37) for s in (-1, 1)])
    return to_base(cs.to_world(pts))[:, 2] - (ZB - R_BAR)


def _surface_samples(parts, spacing=0.012):
    """Points over every face of the carrier's meshes (its own frame), no more than about `spacing` apart."""
    pts = []
    for _, mesh, *_ in parts:
        v = mesh.v
        for a, b, c in mesh.f:
            p0, p1, p2 = v[a], v[b], v[c]
            n = max(1, int(math.ceil(max(np.linalg.norm(p1 - p0), np.linalg.norm(p2 - p0), np.linalg.norm(p2 - p1)) / spacing)))
            for i in range(n + 1):
                for j in range(n + 1 - i):
                    pts.append(p0 + (p1 - p0) * (i / n) + (p2 - p0) * (j / n))
    return np.unique(np.round(np.array(pts), 4), axis=0)


SURFACE = _surface_samples(CARRIER_PARTS)
try:
    from scipy.spatial import cKDTree

    _TREE = cKDTree(SURFACE)
except ImportError:  # plain NumPy below if SciPy is missing
    _TREE = None


def hands_off(cs, tips):
    """Every sampled fingertip point more than CLEAR from the carrier's surface (sampled at 1.2 cm or closer)."""
    if tips is None:
        return False
    local = cs.to_local(tips)
    lo, hi = ENVELOPE
    if np.linalg.norm(np.maximum(np.maximum(lo - local, local - hi), 0.0), axis=1).min() > CLEAR:
        return True  # outside the carrier's bounding box altogether
    if _TREE is not None:
        return bool(_TREE.query(local)[0].min() > CLEAR)
    return bool(min(((SURFACE - p) ** 2).sum(axis=1).min() for p in local) > CLEAR ** 2)


def arm_springs(environment):
    """After the reset's settle: the pawls get their trial spring, preloaded forward against their stops."""
    for link in ("pawlL", "pawlR"):
        asset, j = m._joint(environment, "base", link)
        m._TOUCHED.add(("base", link))  # the next reset gives the joints back their settle gains
        asset.write_joint_effort_limit_to_sim(m._tensor(asset, 200.0), joint_ids=[j])
        asset.write_joint_stiffness_to_sim(m._tensor(asset, PAWL_K), joint_ids=[j])
        asset.set_joint_position_target(m._tensor(asset, -PAWL_PRELOAD / PAWL_K), joint_ids=[j])


class Trial:
    """Updated once per control step after physics; score() and success() only read it."""

    def __init__(self):
        self.reset()

    def reset(self, environment=None):  # the only place the hold and the harm record are cleared
        self.hold = 0.0
        self.value = 0.0
        self.both = False
        self.ready = False
        self.harm = False
        if environment is not None:
            arm_springs(environment)

    def update(self, cs, bs, tips, dt):
        near, far = bar_end(cs, NEAR), bar_end(cs, FAR)
        if cs.tilt_deg() > TIP_OVER_DEG or min(near[2], far[2]) < -DROP:
            self.harm = True
        both = latched(cs, bs, NEAR) and latched(cs, bs, FAR)
        seated = bool(np.all(np.abs(runner_gaps(cs)) <= RUNNER_TOL) and cs.tilt_deg() <= LEVEL_DEG)
        still = float(np.linalg.norm(cs.lin_vel)) < 0.005 and float(np.linalg.norm(cs.ang_vel)) < 0.05
        self.ready = bool(both and seated and still and hands_off(cs, tips) and not self.harm)
        # the hold counts only an unbroken run of ready steps up to now: any step not ready starts it again
        self.hold = self.hold + float(dt) if self.ready else 0.0
        self.both = bool(both)
        if self.harm:
            self.value = 0.0
        elif self.ready and self.hold >= HOLD_SECONDS - 1e-9:
            self.value = 1.0
        else:
            self.value = 0.5 if both else 0.0

    def step(self, environment):
        self.update(m.state(environment, "carrier"), m.state(environment, "base"), m.fingertips(environment),
                    environment.unwrapped.step_dt)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects):
    """This step's score; the trial's result is its value at the last step (SCORE_OVER = "end")."""
    return float(TRIAL.value)


def success(objects):
    """Never ends the trial early: the result is judged on the final state at the step limit."""
    return False


# -- the scripted demo (open loop, from the published starting layout) --------------------------------------------------

from yam_arena import demo as _demo  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402


class Orient(float):
    """A waypoint 'yaw' that carries a whole TCP rotation."""

    rot = np.eye(3)


_plain_grip = getattr(_demo.grip_rotation, "_plain", _demo.grip_rotation)
if not getattr(_demo.grip_rotation, "_orient", False):
    def _grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return rot if rot is not None else _plain_grip(yaw)

    _grip_rotation._orient = True
    _grip_rotation._plain = _plain_grip
    _demo.grip_rotation = _grip_rotation


def tilted(theta_deg, phi_deg):
    """Tilted theta from straight down towards phi (from +x), fingers closing across that direction."""
    phi = math.radians(phi_deg)
    o = Orient(0.0)
    o.rot = axis_angle_matrix(np.array([math.sin(phi), -math.cos(phi), 0.0]), math.radians(theta_deg)) @ _plain_grip(phi)
    return o


PUSH = to_world((0.0, FAULT * 0.125, ZB + 0.15))  # top of the door-side front-corner lip, when seated
TILT, PHI = 55.0, 0.0
POINT = np.array([math.sin(math.radians(TILT)) * math.cos(math.radians(PHI)),
                  math.sin(math.radians(TILT)) * math.sin(math.radians(PHI)), -math.cos(math.radians(TILT))])
PAD = 0.013  # fingertips beyond the TCP
OVER = 0.08  # command this far below the seated lip, so the arm presses hard enough at this reach
CLEAR_POSE = (0.24, -0.30, TOP + 0.36)  # back over the door-side seat, clear of the carrier


def demo():
    """The right arm reaches in over the door-side seat to the raised door-side front corner of the carrier, fingers closed
    and pointing forward and down, and pushes it down until the bar clicks under that pawl; it holds a moment, lifts off
    and moves back over the door-side seat. The left arm is not used."""
    g = tilted(TILT, PHI)
    tcp = PUSH - PAD * POINT
    z = np.array([0.0, 0.0, 1.0])
    right = [
        Waypoint(2.5, tuple(tcp + 0.08 * z - 0.03 * POINT), FINGER_CLOSED, "above the raised corner", g),
        Waypoint(1.5, tuple(tcp + 0.045 * z), FINGER_CLOSED, "just above the lip", g),
        Waypoint(1.5, tuple(tcp - OVER * z), FINGER_CLOSED, "push the corner down", g),
        Waypoint(1.0, tuple(tcp - OVER * z), FINGER_CLOSED, "hold: it clicks", g),
        Waypoint(1.0, tuple(tcp + 0.07 * z), FINGER_CLOSED, "lift off", g),
        Waypoint(2.0, CLEAR_POSE, FINGER_CLOSED, "back over the door-side seat", g),
        Waypoint(6.0, CLEAR_POSE, FINGER_CLOSED, "wait", g),
    ]
    return {"right": right}
