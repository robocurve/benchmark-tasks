"""Scene layout and geometry of the saw-depth task (station frame: x forward from the arm bases, y to the robot's left,
z up; metres). Engine-independent (numpy only).

Saw frame S (the shoe's body frame): origin on the shoe's bottom face, in the blade's mid-plane, at the shoe's
mid-length; axes u = toward the saw's front, v = to the saw's left (motor side), w = up. On the bench the saw faces the
robot's right: u = -y, v = +x, w = +z, so the blade (on the saw's right, v = 0) is on the side nearest the robot and
the motor sits over the board.

Upper assembly frame U (motor, gearcase, upper guard, blade, rear handle): origin on the depth pivot axis, axes equal
to S at depth angle 0. It turns about the pivot by theta about -v: theta > 0 lowers the blade (more depth).
Depth lever frame K: origin on the lever's clamp bolt, axes equal to U at lever angle 0 (locked); phi > 0 (about -v)
lifts the lever's tab: released.

Representative DeWalt DWE575-class geometry (7-1/4 in blade, 0 to 2-9/16 in depth at 90 deg, 8.8 lb): overall sizes
from the published specification; the internal layout (pivot, arbor, gear offset, quadrant, lever) is our estimate,
see README.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena TABLE_TOP["station"])
IN = 0.0254

# ---------------------------------------------------------------- stock on the bench
BOARDS = (  # (name, actual thickness m): six stock classes whose 1/8-1/4 in target bands do not overlap
    ("1/4 in tempered hardboard", 0.0064),
    ("15/32 in (1/2 in nominal) plywood", 0.0119),
    ("23/32 in (3/4 in nominal) plywood", 0.0183),
    ("5/4 x 6 deck board (1 in actual)", 0.0254),
    ("2x6 framing lumber (1-1/2 in actual)", 0.0381),
    ("1-3/4 in LVL (laminated veneer lumber)", 0.0445),
)
BOARD_W = 0.140  # all boards ripped / sized 5-1/2 in wide (a 2x6's width)
BOARD_L = 0.600
BLOCK_H = 0.0635  # two 2x3 offcuts on edge (2-1/2 in actual) under the board's ends
BLOCK_W = 0.038  # 1-1/2 in (along y)
BLOCK_L = 0.180  # along x
BLOCK_Y = (-0.25, 0.25)
UNDERSIDE_Z = TOP + BLOCK_H  # board underside (same for every board)
XB = 0.355  # blade mid-plane x (saw frame v = 0)
BLADE_GAP = 0.008  # blade plane to the board's near edge (blade body half-thickness 0.8 mm, kerf 1.1 mm)
BOARD_X0 = XB + BLADE_GAP  # board near edge
Y0 = -0.060  # saw frame origin y (shoe mid-length) before the per-trial jitter
SAW_Y_JITTER = 0.012

R_SAW = np.array(
    [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
)  # columns: u, v, w in the station frame


def board_top(t):
    return UNDERSIDE_Z + t


def board_center(t):
    return np.array([BOARD_X0 + BOARD_W / 2, 0.0, UNDERSIDE_Z + t / 2])


# ---------------------------------------------------------------- the saw (saw-frame coordinates u, v, w)
BLADE_R = 0.0921  # 7-1/4 in blade (184 mm), tip radius
BLADE_GULLET_R = 0.0855
BLADE_TEETH = 24
BLADE_T = 0.0016  # plate thickness (kerf 2.2 mm at the carbide tips)
KERF_T = 0.0022
PIVOT = np.array([0.125, 0.0, 0.045])  # depth pivot (front of the shoe)
ARBOR0 = np.array([-0.010, 0.0, 0.094])  # arbor at depth angle 0 (blade 2 mm inside the shoe)
DEPTH_MAX = 0.0651  # 2-9/16 in at 90 deg bevel
SHOE_L = 0.300
SHOE_T = 0.004
SHOE_V = (-0.030, 0.122)  # shoe edges (right, left); slot for the blade +-6 mm
SLOT_V = 0.006
SLOT_U = (-0.125, 0.105)
MOTOR_C = np.array([-0.035, 0.0, 0.139])  # motor axis (spur-gear offset above and behind the arbor)
MOTOR_R = 0.040
MOTOR_V = (0.016, 0.104)
CAP_V = (0.104, 0.116)
CAP_R = 0.033
GEAR_BOX = ((-0.045, 0.030), (0.003, 0.016), (0.100, 0.175))  # u, v, w ranges
GUARD_R = 0.104
GUARD_V = (-0.009, 0.003)
GUARD_ARC = (15.0, 165.0)  # deg, upper guard covers the blade from 15 to 165 deg (0 = front)
# rear handle (closed D loop): grip bar Hf -> Hr, rear post Hr -> Hb, lower bar Hb -> Hm (into the motor top)
H_F = np.array([-0.100, 0.035, 0.275])
H_R = np.array([-0.220, 0.035, 0.240])
H_B = np.array([-0.190, 0.035, 0.185])
H_M = np.array([-0.070, 0.035, 0.180])
H_FRONT_FOOT = np.array([-0.075, 0.035, 0.200])
GRIP_BAR = (0.028, 0.030)  # grip bar section: v width, thickness normal to the bar in the u-w plane
POST_SEC = (0.026, 0.022)
GRIP_S = 0.5  # the reference grasp: middle of the grip bar
# depth bracket (quadrant, on the shoe) and the depth lever (on the upper assembly)
BOLT0 = np.array([-0.060, -0.022, 0.150])  # lever clamp bolt at depth angle 0 (U frame offset added below)
QUAD_V = (-0.016, -0.012)
LEVER_V = (-0.034, -0.026)
LEVER_HUB_V = (-0.026, -0.017)
LEVER_LEN = 0.050  # bolt to tab tip, pointing forward (+u) when locked
LEVER_TAB_W = 0.014  # tab height (w)
LEVER_PHI_MAX = math.radians(60.0)  # released stop
LEVER_PHI_MID = math.radians(25.0)  # over-centre point of the cam
LEVER_PHI_ENGAGE = math.radians(6.0)  # cam clamps the bracket below this
LEVER_TOGGLE_NM = 0.45  # cam torque either side of the over-centre point (about 10 N at the tab)
LEVER_DAMP = 0.002  # N m s / rad

# masses (kg): DWE575 8.8 lb = 4.0 kg
M_MOTOR, M_CAP, M_GEAR, M_GUARD, M_BLADE, M_HANDLE = 1.75, 0.15, 0.30, 0.45, 0.35, 0.40
M_UPPER = M_MOTOR + M_CAP + M_GEAR + M_GUARD + M_BLADE + M_HANDLE  # 3.40
M_SHOE = 0.60  # shoe + depth bracket
M_LEVER = 0.040

# depth clamp (cam lever on the bracket) and the loose bracket
CLAMP_NM = 25.0  # holding torque about the pivot when locked (gravity 5.6 N m + about 65 N on the handle)
CLAMP_K = 2.0e4  # N m / rad, implicit articulation drive (stick spring)
CLAMP_C = 40.0
LOOSE_NM = 1.0  # released bracket: friction-like damping, capped (gravity on the motor is 5.5 N m)
LOOSE_C = 3.0

# friction (static = dynamic)
F_WOOD_ALU = 0.35  # shoe (aluminium) on wood
F_RUBBER = 1.0  # handle overmould
F_PLASTIC = 0.6
F_WOOD = 0.5
F_STEEL = 0.4

THETA_MAX = None  # set below (depth angle at DEPTH_MAX)


def rz(deg):
    """Rotation about the station z axis (deg)."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_v(theta):
    """Rotation by theta about -v in saw coordinates (rear goes down for theta > 0)."""
    c, s = math.cos(theta), math.sin(theta)
    # about +v by -theta: u' = u c - w s ; w' = u s + w c
    return np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])


def arbor_s(theta):
    """Arbor position in the saw frame at depth angle theta."""
    return PIVOT + rot_v(theta) @ (ARBOR0 - PIVOT)


def depth_of(theta):
    """Blade projection below the shoe's bottom face (m) at depth angle theta (negative: blade inside the shoe)."""
    return BLADE_R - arbor_s(theta)[2]


def theta_of_depth(d):
    """Inverse of depth_of (bisection on 0..THETA_MAX)."""
    lo, hi = 0.0, THETA_MAX
    for _ in range(60):
        mid = (lo + hi) / 2
        if depth_of(mid) < d:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def _theta_max():
    lo, hi = 0.0, 0.8
    for _ in range(60):
        mid = (lo + hi) / 2
        if BLADE_R - (PIVOT + rot_v(mid) @ (ARBOR0 - PIVOT))[2] < DEPTH_MAX:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


THETA_MAX = _theta_max()


# ---------------------------------------------------------------- start states
# named setups (unseeded resets): (board index, start blade depth below the shoe m)
SETUPS = ((2, 0.062), (4, 0.001), (1, 0.061), (3, 0.0), (5, 0.063), (0, 0.060))
# seeded starts: the saw is left either retracted (blade 0-2 mm below the shoe) or deep (59.5-64.1 mm), each with
# probability 1/2 and independent of the board, so the start depth says nothing about the board. Every start is at least
# 4.4 mm above or 15 mm below the board's underside (the full-score band is 3.2-6.4 mm below it).
START_RETRACTED = (0.0, 0.002)
START_DEEP = (0.0595, DEPTH_MAX - 0.001)


def setup_spec(setup):
    """Start state: {'board': index, 't': m, 'depth0': m, 'dy': m, 'name': str}. ``setup`` is a named setup index
    or ('seed', n): from seed stream 1 of n, the start depth (retracted or deep, 1/2 each, uniform within), the board
    (uniform over the six BOARDS) and a shift of the saw along the board of up to +-12 mm, drawn independently."""
    if isinstance(setup, tuple) and setup[0] == "seed":
        rng = np.random.default_rng([int(setup[1]), 1])
        u_dep = float(rng.uniform())
        b = int(rng.integers(len(BOARDS)))
        dy = float(rng.uniform(-SAW_Y_JITTER, SAW_Y_JITTER))
        lo, hi = START_RETRACTED if u_dep < 0.5 else START_DEEP
        f = (u_dep * 2.0) % 1.0
        d0 = lo + f * (hi - lo)
        return dict(board=b, t=BOARDS[b][1], depth0=d0, dy=dy, name=f"seed {int(setup[1])}")
    k = int(setup) % len(SETUPS)
    b, d0 = SETUPS[k]
    return dict(board=b, t=BOARDS[b][1], depth0=d0, dy=0.0, name=f"setup {k}")


def shoe_pose(spec):
    """Authored shoe pose (pos, rot): resting flat on the board top, blade plane at XB."""
    return np.array([XB, Y0 + spec["dy"], board_top(spec["t"])]), R_SAW.copy()


def upper_pose(shoe_pos, shoe_rot, theta):
    return shoe_pos + shoe_rot @ PIVOT, shoe_rot @ rot_v(theta)


def lever_pose(upper_pos, upper_rot, phi):
    return upper_pos + upper_rot @ (BOLT0 - PIVOT), upper_rot @ rot_v(phi)


# ---------------------------------------------------------------- measurements from poses
def blade_low_point(upper_pos, upper_rot):
    """Lowest point of the blade's tooth circle (station frame)."""
    arbor = upper_pos + upper_rot @ (ARBOR0 - PIVOT)
    n = upper_rot[:, 1]  # blade plane normal (v)
    down = np.array([0.0, 0.0, -1.0])
    d = down - n * (down @ n)
    d /= np.linalg.norm(d)
    return arbor + BLADE_R * d


def projection_below(upper_pos, upper_rot):
    """How far the lowest tooth is below the board's underside (m; negative = above it)."""
    return float(UNDERSIDE_Z - blade_low_point(upper_pos, upper_rot)[2])


def grip_point_u(s=GRIP_S):
    """Point on the grip bar's axis in the U frame, and the bar direction (unit, U frame)."""
    a, b = H_F - PIVOT, H_R - PIVOT
    d = (b - a) / np.linalg.norm(b - a)
    return a + s * (b - a), d


def lever_tip_k():
    """Lever tab tip centre in the K frame."""
    return np.array([LEVER_LEN, (LEVER_V[0] + LEVER_V[1]) / 2 - BOLT0[1], 0.0])


# ---------------------------------------------------------------- convex pieces (body-local vertices)
def box_pts(c, size, rot=None):
    hx, hy, hz = (s / 2 for s in size)
    p = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    if rot is not None:
        p = p @ np.asarray(rot).T
    return p + np.asarray(c, float)


def range_box(ur, vr, wr):
    return box_pts(((ur[0] + ur[1]) / 2, (vr[0] + vr[1]) / 2, (wr[0] + wr[1]) / 2),
                   (ur[1] - ur[0], vr[1] - vr[0], wr[1] - wr[0]))


def bar_pts(a, b, width_v, thick):
    """Box along segment a->b (in the u-w plane) with v width and thickness normal to the segment."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    ln = np.linalg.norm(d)
    x = d / ln
    y = np.array([0.0, 1.0, 0.0])
    z = np.cross(x, y)
    rot = np.column_stack([x, y, z])
    return box_pts((a + b) / 2, (ln + thick * 0.3, width_v, thick), rot)


def v_cylinder(c, r, vr, n=24):
    """Cylinder with its axis along v through c (u, w), from v0 to v1."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    pts = []
    for v in vr:
        pts += [[c[0] + r * math.cos(t), v, c[2] + r * math.sin(t)] for t in a]
    return np.array(pts)


def blade_polygon(teeth=True):
    """Blade outline in its own plane (u, w around the arbor): tooth tips and gullets (24 teeth, hook-faced)."""
    pts = []
    for i in range(BLADE_TEETH):
        a0 = 2 * math.pi * i / BLADE_TEETH
        if teeth:
            pts.append((BLADE_R * math.cos(a0), BLADE_R * math.sin(a0)))
            a1 = a0 + 2 * math.pi / BLADE_TEETH * 0.18
            pts.append((BLADE_GULLET_R * math.cos(a1), BLADE_GULLET_R * math.sin(a1)))
            a2 = a0 + 2 * math.pi / BLADE_TEETH * 0.80
            pts.append((0.989 * BLADE_R * math.cos(a2), 0.989 * BLADE_R * math.sin(a2)))
        else:
            pts.append((BLADE_R * math.cos(a0), BLADE_R * math.sin(a0)))
    return np.array(pts)


def blade_hull_u():
    """Blade collision hull (U frame): the 24 tooth tips at kerf thickness."""
    c = ARBOR0 - PIVOT
    poly = blade_polygon(teeth=False)
    pts = []
    for v in (-KERF_T / 2, KERF_T / 2):
        pts += [[c[0] + p[0], v, c[2] + p[1]] for p in poly]
    return np.array(pts)


def guard_hull_u():
    c = ARBOR0 - PIVOT
    a = np.radians(np.linspace(GUARD_ARC[0], GUARD_ARC[1], 25))
    pts = []
    for v in GUARD_V:
        # u axis forward: angle 0 = front (+u), 90 = up
        pts += [[c[0] + GUARD_R * math.cos(t), v, c[2] + GUARD_R * math.sin(t)] for t in a]
        pts += [[c[0] + 0.02 * math.cos(t), v, c[2] + 0.02 * math.sin(t)] for t in (a[0], a[-1])]
    return np.array(pts)


def upper_pieces():
    """[(name, vertices U frame, colour key, friction)] for the upper assembly (collision + visual)."""
    P = PIVOT
    out = [
        ("motor", v_cylinder(MOTOR_C - P, MOTOR_R, MOTOR_V, 28), "black", F_PLASTIC),
        ("cap", v_cylinder(MOTOR_C - P, CAP_R, CAP_V, 20), "yellow", F_PLASTIC),
        ("gear", range_box(*(np.add(r, P[i] * -1) for i, r in enumerate(GEAR_BOX))), "alu", F_STEEL),
        ("guard", guard_hull_u(), "yellow", F_PLASTIC),
        ("blade", blade_hull_u(), "steel", F_STEEL),
        ("grip", bar_pts(H_F - P, H_R - P, *GRIP_BAR), "rubber", F_RUBBER),
        ("post", bar_pts(H_R - P, H_B - P, *POST_SEC), "yellow", F_PLASTIC),
        ("lower", bar_pts(H_B - P, H_M - P, *POST_SEC), "yellow", F_PLASTIC),
        ("front", bar_pts(H_F - P, H_FRONT_FOOT - P, *POST_SEC), "yellow", F_PLASTIC),
        ("knob", v_cylinder(np.array([0.060, 0, 0.205]) - P, 0.017, (0.020, 0.060), 16), "rubber", F_RUBBER),
        ("knob_post", range_box((0.050, 0.070), (0.025, 0.055), (0.150, 0.195)) - P, "yellow", F_PLASTIC),
        ("boss", range_box((-0.075, -0.045), (-0.012, -0.008), (0.135, 0.165)) - P, "alu", F_STEEL),
    ]
    return out


def upper_com_u():
    P = PIVOT
    parts = [
        (M_MOTOR, MOTOR_C + [0, (MOTOR_V[0] + MOTOR_V[1]) / 2, 0]),
        (M_CAP, MOTOR_C + [0, (CAP_V[0] + CAP_V[1]) / 2, 0]),
        (M_GEAR, np.array([np.mean(r) for r in GEAR_BOX])),
        (M_GUARD, ARBOR0 + [0, -0.003, 0.055]),
        (M_BLADE, ARBOR0),
        (M_HANDLE, (H_F + H_R + H_B + H_M) / 4),
    ]
    m = sum(p[0] for p in parts)
    return sum(p[0] * p[1] for p in parts) / m - P


def upper_inertia():
    """Diagonal inertia about the COM (U axes), lumped estimate."""
    com = upper_com_u() + PIVOT
    parts = [
        (M_MOTOR, MOTOR_C + [0, 0.06, 0], 0.040),
        (M_CAP, MOTOR_C + [0, 0.11, 0], 0.03),
        (M_GEAR, np.array([np.mean(r) for r in GEAR_BOX]), 0.04),
        (M_GUARD, ARBOR0 + [0, 0, 0.055], 0.07),
        (M_BLADE, ARBOR0, 0.065),
        (M_HANDLE, (H_F + H_R + H_B + H_M) / 4, 0.05),
    ]
    j = np.zeros(3)
    for m, c, r in parts:
        d = c - com
        j += m * (np.array([d[1] ** 2 + d[2] ** 2, d[0] ** 2 + d[2] ** 2, d[0] ** 2 + d[1] ** 2]) + 0.4 * r * r)
    return j


def shoe_pieces():
    """[(name, vertices S frame, colour, friction)]: the shoe plate around its blade slot, its rim, the depth bracket
    (quadrant) and the pivot bracket."""
    u0, u1 = -SHOE_L / 2, SHOE_L / 2
    out = [
        ("plate_l", range_box((u0, u1), (SLOT_V, SHOE_V[1]), (0.0, SHOE_T)), "alu", F_WOOD_ALU),
        ("plate_r", range_box((u0, u1), (SHOE_V[0], -SLOT_V), (0.0, SHOE_T)), "alu", F_WOOD_ALU),
        ("plate_f", range_box((SLOT_U[1], u1), (-SLOT_V, SLOT_V), (0.0, SHOE_T)), "alu", F_WOOD_ALU),
        ("plate_b", range_box((u0, SLOT_U[0]), (-SLOT_V, SLOT_V), (0.0, SHOE_T)), "alu", F_WOOD_ALU),
        ("rim_l", range_box((u0, u1), (SHOE_V[1] - 0.003, SHOE_V[1]), (SHOE_T, 0.012)), "alu", F_STEEL),
        ("pivot_l", range_box((0.110, 0.140), (0.012, 0.018), (SHOE_T, 0.055)), "alu", F_STEEL),
        ("pivot_r", range_box((0.110, 0.140), (-0.016, -0.010), (SHOE_T, 0.055)), "alu", F_STEEL),
    ]
    # depth bracket: an arc plate following the bolt's path (bolt at depth 0 .. max), plus its foot on the shoe
    path = bolt_path_s(9)
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        out.append((f"quad_{i}", bar_pts(a * [1, 0, 1] + [0, (QUAD_V[0] + QUAD_V[1]) / 2, 0],
                                        b * [1, 0, 1] + [0, (QUAD_V[0] + QUAD_V[1]) / 2, 0],
                                        QUAD_V[1] - QUAD_V[0], 0.024), "alu", F_STEEL))
    lo = path[-1]
    out.append(("quad_foot", range_box((lo[0] - 0.015, lo[0] + 0.015), QUAD_V, (SHOE_T, lo[2])), "alu", F_STEEL))
    return out


def bolt_path_s(n=9):
    """Bolt centre (saw frame) from depth angle 0 to THETA_MAX."""
    return [PIVOT + rot_v(t) @ (BOLT0 - PIVOT) for t in np.linspace(0, THETA_MAX, n)]


def depth_ticks_s():
    """Depth-scale tick marks on the bracket (saw frame): the bolt centre's position at each 1/4 in of depth,
    [(position, depth in)]."""
    out = []
    k = 0
    while k * 0.25 * IN <= DEPTH_MAX + 1e-9:
        d = k * 0.25 * IN
        out.append((PIVOT + rot_v(theta_of_depth(max(d, 1e-5))) @ (BOLT0 - PIVOT), k * 0.25))
        k += 1
    return out


def lever_pieces():
    """[(name, vertices K frame, colour, friction)]: hub and tab (pointing +u at phi 0)."""
    vm = -BOLT0[1]
    hub = v_cylinder(np.zeros(3), 0.010, (LEVER_HUB_V[0] + vm, LEVER_HUB_V[1] + vm), 16)
    tab = range_box((0.004, LEVER_LEN), (LEVER_V[0] + vm, LEVER_V[1] + vm), (-LEVER_TAB_W / 2, LEVER_TAB_W / 2))
    head = range_box((LEVER_LEN - 0.014, LEVER_LEN + 0.004), (LEVER_V[0] + vm - 0.002, LEVER_V[1] + vm),
                     (-LEVER_TAB_W / 2 - 0.002, LEVER_TAB_W / 2 + 0.002))
    return [("hub", hub, "red", F_PLASTIC), ("tab", tab, "red", F_PLASTIC), ("head", head, "red", F_PLASTIC)]


def stock_pieces(t):
    """[(name, vertices station frame relative to the stock body origin (0,0,0)), colour, friction]: board + blocks."""
    out = [("board", box_pts(board_center(t), (BOARD_W, BOARD_L, t)), "wood", F_WOOD)]
    for i, y in enumerate(BLOCK_Y):
        out.append((f"block_{i}", box_pts((BOARD_X0 + BOARD_W / 2, y, TOP + BLOCK_H / 2),
                                          (BLOCK_L, BLOCK_W, BLOCK_H)), "wood2", F_WOOD))
    return out
