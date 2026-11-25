"""Geometry, lampholder model constants and start states of the t8-tube task (pure NumPy; no simulator).

Scene (station frame: x forward from the line between the arm bases, y to the robot's left of their midpoint, z up;
table top z = TOP): a 2-ft one-lamp T8 strip fixture is screwed under a wall shelf above the bench. A dead F17T8
lamp (610 mm class, G13 medium bi-pin) is locked in the fixture's two rotor-lock ("turn-type") G13 lampholders. An
open lamp-recycling carton for 2-ft lamps lies on the bench in front of the shelf.

Fixture frame F: origin at the seated lamp's centre (on its axis), +y along the lamp axis (to the robot's left at yaw
0), +x toward the wall, +z up. The lamp hangs off the fixture (one articulation, fixed base) through a chain of joints
(slides along F's x, y, z, then rotations about z (yaw), x (pitch) and the lamp's own axis y (spin)); the chain's zero
is the seated pose, so the chain coordinates are the lamp's displacement from its seat.

Lampholder geometry follows the G13 standard (pins 2.41 mm diameter, 12.7 mm apart, IEC 60061-1 sheet 7004-51) and a
typical rotor-lock tombstone; lamp dimensions follow typical F17T8 data (T8 = 1 in tube, 604 mm base face to base
face). Masses, torques and break forces are estimates, stated in README / DESIGN.md.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena station TABLE_TOP)


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ry(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rx(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


# ------------------------------------------------------------------------------------------------ lamp (F17T8)
LAMP_L = 0.604  # base face to base face (MOL 23.78 in class, typical F17T8 datasheets)
HALF = LAMP_L / 2
TUBE_R = 0.0127  # T8: 8/8 in diameter
CAP_L = 0.012  # aluminium end cap (base shell) length
CAP_R = 0.0131
PIN_D = 0.00241  # G13 pins
PIN_L = 0.0066
PIN_SPACING = 0.0127
LAMP_MASS = 0.14  # kg, 2-ft T8 (estimate: a 4-ft T8 weighs about 0.25-0.28 kg)

# ------------------------------------------------------------------------------------------------ lampholders
FACE_GAP = 0.0010  # base face to lampholder face, each end (the lamp has 2 mm of end play between the faces)
TS_T = 0.016  # tombstone thickness along the lamp axis
TS_W = 0.034  # width across (x)
SLOT_L = 0.022  # slot from the rotor centre (lamp axis) down to the tombstone's bottom edge
DROP = 0.055  # lamp axis below the fixture channel's underside (about 2.2 in)
SLOT_W = 0.0032  # slot width (pins 2.41 mm)
ROTOR_D = 0.0175
OPEN_DEG = 90.0  # rotor turned this far from locked: pins line up with the slot (either direction)
OPEN_TOL_DEG = 2.6  # slot clearance (3.2 - 2.41) / 2 mm at the pin radius 6.35 mm
# rotor detent (per lampholder; the lamp turns both rotors at once): spring contacts click it into the locked
# position; between there and open the contacts drag on the pins. Estimates (fingertip effort for a person).
T_CLICK = 0.20  # N m per lampholder, peak torque to turn the lamp out of the locked detent
DETENT_DEG = 12.0  # detent well half-width
T_RUN = 0.08  # N m per lampholder, running drag between locked and open
T_STOP = 3.0  # N m per lampholder: rotor stops at locked +- 90 deg
# lampholder strength (estimate for a snap-in PBT tombstone): a sustained side / pull load above this on one
# lampholder breaks it (or tears it off its mounting)
F_BREAK = 60.0  # N
BREAK_HOLD_S = 0.10

# ------------------------------------------------------------------------------------------------ fixture + shelf
CH_W = 0.060  # strip channel width (x)
CH_H = 0.035  # channel depth
CH_L = LAMP_L + 2 * (FACE_GAP + TS_T) + 0.020
SHELF_T = 0.019
SHELF_D = 0.280  # shelf depth (x): front edge flush with the channel front, back edge at the wall
SHELF_W = 0.900
SHELF_FRONT = -CH_W / 2  # F frame

# ------------------------------------------------------------------------------------------------ carton
BOX_IN = (0.105, 0.650, 0.070)  # inner x, y, z (open-top corrugated lamp-recycling carton for 2-ft lamps; estimate)
BOX_WALL = 0.004
BOX_MASS = 0.30

FRICTION = dict(lamp=0.8, fixture=0.5, carton=0.6, table=0.5)

# ------------------------------------------------------------------------------------------------ hazards / scoring
V_DROP = 0.80  # m/s: a lamp moving faster than this with no hand on it and out of the lampholders has fallen
# freely more than ~3 cm: counted broken (glass lamps are fragile; handling guidance is never to drop them)

# ------------------------------------------------------------------------------------------------ start states
NOMINAL = dict(fx=0.390, fy=0.000, fz=0.300, fyaw=0.0, bx=0.215, by=0.000, byaw=0.0)
FX = (0.360, 0.435)  # fixture (seated lamp centre) x
FY = (-0.040, 0.040)
FZ = (0.270, 0.330)  # lamp axis height above the bench
FYAW = (4.0, 8.0)  # magnitude; either sign (the shelf is not square to the robot). The yaw alone does NOT defeat a
# mirrored (arm-swapped) trace: the mirror image of a start is itself in the distribution, so setup_spec also guards
# against the mirrored default pinch
BX = (0.195, 0.235)  # carton centre
BY = (-0.040, 0.040)
BYAW = (-6.0, 6.0)
NAMED_SETUPS = (
    dict(fx=0.395, fy=0.020, fz=0.305, fyaw=5.0, bx=0.220, by=-0.020, byaw=-3.0),
    dict(fx=0.375, fy=-0.030, fz=0.290, fyaw=-6.0, bx=0.205, by=0.025, byaw=4.0),
    dict(fx=0.410, fy=0.010, fz=0.315, fyaw=7.0, bx=0.230, by=0.000, byaw=0.0),
)

OFFICIAL_SEED = 1415158438  # inspect-robots trial seed of eval seed 0 (the default start)


GRASP_G = 0.15  # the reference demo pinches the lamp this far either side of its centre


def _fixture_draw(rng):
    u = lambda r: float(rng.uniform(*r))  # noqa: E731
    yaw = u(FYAW) * (1.0 if rng.uniform() < 0.5 else -1.0)
    return dict(fx=u(FX), fy=u(FY), fz=u(FZ), fyaw=yaw)


def _seeded(n):
    rng = np.random.default_rng(int(n))
    sp = _fixture_draw(rng)
    sp.update(bx=float(rng.uniform(*BX)), by=float(rng.uniform(*BY)), byaw=float(rng.uniform(*BYAW)), name=f"seed {int(n)}")
    return sp


def lamp_axis(spec):
    """Seated lamp centre and unit axis (station frame)."""
    p, r = fixture_pose(spec)
    return p, r[:, 1]


APPROACH_DEG = 80.0  # the reference pinch: approach this far round from straight up (90 = horizontal toward the wall)
# capture window of a replayed pinch (offsets of the lamp axis from the pinch point, mm): along the approach the lamp
# can sit up to CAP_FAR farther (pad tips 6 mm past the axis) or CAP_NEAR nearer (deep in the open jaw, up to the
# palm); across it, within CAP_ACROSS (open fingers 24 mm from the axis, lamp radius 12.7 mm). GUARD_MARGIN is added
# (10 mm: a replay was seen to succeed 0.2-0.7 mm outside a 6 mm margin).
CAP_FAR, CAP_NEAR, CAP_ACROSS, GUARD_MARGIN = 15.0, 45.0, 14.0, 10.0
GUARD_MARGIN_ACROSS = 16.0  # a mirrored replay succeeded at 24.1-24.6 mm across (ev3, seed 15285): limit 30 mm


def pinch_offsets(spec, ref, mirror=False):
    """Per hand (left, right): (along, across) offsets (mm) of this start's lamp axis from the reference start's pinch
    point (GRASP_G either side of the lamp centre, approach APPROACH_DEG; mirrored in y when ``mirror``), in the
    reference pinch frame; ``along`` > 0: the lamp is farther than the pinch point."""
    c0, a0 = lamp_axis(ref)
    c1, a1 = lamp_axis(spec)
    out = []
    for g in (GRASP_G, -GRASP_G):
        p = c0 + g * a0
        ax = a0.copy()
        if mirror:
            p = p * np.array([1.0, -1.0, 1.0])
            ax = ax * np.array([1.0, -1.0, 1.0])
        u = np.cross(ax, [0.0, 0.0, 1.0])
        u /= np.linalg.norm(u)
        up = np.cross(u, ax)
        al = math.radians(APPROACH_DEG)
        app = math.sin(al) * u + math.cos(al) * up
        acr = np.cross(app, ax)
        v = p - c1
        v = v - (v @ a1) * a1  # from the axis to the pinch point
        out.append((float(-(v @ app)) * 1e3, float(v @ acr) * 1e3))
    return out


def captured(off, margin=0.0, margin_across=None):
    along, across = off
    ma = margin if margin_across is None else margin_across
    return -CAP_NEAR - margin < along < CAP_FAR + margin and abs(across) < CAP_ACROSS + ma


def replay_can_pinch(spec, ref, mirror=False, margin=GUARD_MARGIN, margin_across=GUARD_MARGIN_ACROSS):
    """Could either hand of a replayed reference trajectory still pinch this start's lamp?"""
    return any(captured(o, margin, margin_across) for o in pinch_offsets(spec, ref, mirror))


def setup_spec(setup):
    """Start state dict. ``setup`` is a named-setup index or ('seed', n).

    Seeded starts: fixture (with its shelf) and carton drawn from seed n. Open-loop guard: when either hand of the
    default (official-seed) start's pinch, or of its mirrored (arm-swapped) pinch, would still find this start's lamp
    inside its capture window (plus a 10 mm margin, 16 mm across; see ``captured``), the fixture is redrawn from a second seeded
    stream until none would."""
    if isinstance(setup, (tuple, list)) and setup[0] == "seed":
        n = int(setup[1])
        sp = _seeded(n)
        if n == OFFICIAL_SEED:
            return sp
        ref = _seeded(OFFICIAL_SEED)
        if replay_can_pinch(sp, ref) or replay_can_pinch(sp, ref, mirror=True):
            rng = np.random.default_rng([n, 7])
            for _ in range(5000):
                sp.update(_fixture_draw(rng))
                if not (replay_can_pinch(sp, ref) or replay_can_pinch(sp, ref, mirror=True)):
                    break
        return sp
    k = int(setup) % len(NAMED_SETUPS)
    return dict(NAMED_SETUPS[k], name=f"setup {k}")


def fixture_pose(spec):
    """Fixture frame F (origin at the seated lamp centre) in the station frame."""
    return np.array([spec["fx"], spec["fy"], TOP + spec["fz"]]), rz(spec["fyaw"])


def carton_pose(spec):
    """Carton frame: origin at the centre of its outer bottom face (on the bench), +y along its length."""
    return np.array([spec["bx"], spec["by"], TOP]), rz(spec["byaw"])


def in_carton(p_center, r_lamp, spec_pose):
    """Lamp (centre ``p_center``, rotation ``r_lamp``; axis = r_lamp[:, 1]) lying inside the carton: both base
    faces inside its inner outline and below its rim."""
    bp, br = spec_pose
    ax = r_lamp[:, 1]
    hx, hy, hz = BOX_IN[0] / 2, BOX_IN[1] / 2, BOX_IN[2]
    for s in (-1.0, 1.0):
        e = br.T @ (p_center + s * HALF * ax - bp)
        if not (abs(e[0]) < hx and abs(e[1]) < hy and BOX_WALL - 0.002 < e[2] < BOX_WALL + hz):
            return False
    return True


def resting_surfaces(center, rotation, carton):
    """Return supported placements for the scorer. Both end caps must be at support height."""
    axis = rotation[:, 1]
    ends = np.array([center - HALF * axis, center + HALF * axis])
    carton_pos, carton_rot = carton
    local = (ends - carton_pos) @ carton_rot
    tolerance = 0.004
    inside = (in_carton(center, rotation, carton)
              and np.all(np.abs(local[:, 2] - (BOX_WALL + CAP_R)) <= tolerance))
    on_bench = np.all(np.abs(ends[:, 2] - (TOP + CAP_R)) <= tolerance)
    # The station tabletop is 1.10 x 1.30 m, centred at (0.36, 0).
    within_table = np.all(np.abs(ends[:, :2] - [0.36, 0.0]) <= np.array([0.55, 0.65]) - CAP_R)
    # Separating axes of the carton footprint and the projected tube segment.
    a, b = local[:, :2]
    direction = b - a
    normal = np.array([-direction[1], direction[0]])
    half_widths = np.array(BOX_IN[:2]) / 2 + BOX_WALL
    separated = False
    for test_axis in (np.array([1., 0.]), np.array([0., 1.]), normal):
        projected = local[:, :2] @ test_axis
        edge = np.dot(half_widths, np.abs(test_axis)) + CAP_R * np.linalg.norm(test_axis)
        if np.min(projected) > edge or np.max(projected) < -edge:
            separated = True
            break
    return bool(inside), bool(on_bench and within_table and separated)


# ------------------------------------------------------------------------------------------------ shapes
def box(center, size):
    hx, hy, hz = (s / 2 for s in size)
    c = np.asarray(center, float)
    return np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) + c


def box_xyz(xr, yr, zr):
    return box(((xr[0] + xr[1]) / 2, (yr[0] + yr[1]) / 2, (zr[0] + zr[1]) / 2),
               (xr[1] - xr[0], yr[1] - yr[0], zr[1] - zr[0]))


def cyl_y(center, r, h, n=24, phase=0.0):
    """Cylinder along y."""
    c = np.asarray(center, float)
    a = np.linspace(0, 2 * math.pi, n, endpoint=False) + phase
    return np.array([[r * math.cos(t), y, r * math.sin(t)] for y in (-h / 2, h / 2) for t in a]) + c


def arc_patch_y(r0, r1, a0, a1, y0, y1, n=4):
    """Thin curved patch on a cylinder along y (angle a measured from +x toward +z)."""
    pts = []
    for a in np.linspace(a0, a1, n):
        c, s = math.cos(a), math.sin(a)
        for r in (r0, r1):
            for y in (y0, y1):
                pts.append([r * c, y, r * s])
    return np.array(pts)
