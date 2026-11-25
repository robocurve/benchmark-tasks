"""Scene layout and geometry of the recessed-trim task (station frame: x forward from the arm bases, y to the robot's
left, z up; metres). Engine-independent (numpy only).

Housing frame H: origin on the housing (can) axis at the ceiling's underside, z up, x/y = station x/y.
Trim frame T: origin on the trim axis at the flange's bottom face, z up along the trim axis; spring A sits at trim
angle +90 deg (the +y_T side), spring B at -90 deg. A trim yaw psi turns T about z.
Leg frame: origin at the leg's root pivot on the spring coil, z along the straight leg, x = radial (outward) = the
pivot axis.
Representative dimensions of a 6 in. IC airtight new-construction housing with torsion-spring brackets and a 6 in.
shallow baffle trim with torsion springs (catalogue spec sheets were not reachable from the build host; every number
here is a stated estimate, see the README fidelity section).
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena station.TABLE_TOP)


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rx(rad):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


# ---------------------------------------------------------------- ceiling and housing
H = np.array([0.30, 0.0, 1.20])  # housing axis at the ceiling underside (station frame)
DRYWALL_T = 0.0127  # 1/2 in. gypsum board
CUTOUT_R = 0.0857  # 6-3/4 in. ceiling cut-out
CEIL_X = (0.16, 0.60)  # ceiling panel extent (station x)
CEIL_Y = (-0.17, 0.17)  # one joist bay (the YAM forearms pass beside it, see README fidelity)
CAN_R = (0.0860, 0.0875)  # housing (can) wall: 172 mm inside diameter, 1.5 mm steel
CAN_Z = (DRYWALL_T, 0.190)  # wall from the drywall's top face to the can top (above the ceiling underside)
LAMP_R = 0.0475  # BR30 lamp, 95 mm diameter
LAMP_Z = (0.065, 0.165)  # lamp face 65 mm above the ceiling underside
# torsion-spring brackets: an L-bracket on the can wall at angle beta with a horizontal tab and a rectangular slot
TAB_Z = 0.0200  # tab top face above the ceiling underside
TAB_T = 0.0015
TAB_R = (0.0745, 0.0860)  # tab radial span (from the can wall inward)
TAB_U = 0.020  # tab half-width (tangential)
SLOT_R = (0.0765, 0.0815)  # slot radial span (5 mm)
SLOT_U = 0.0080  # slot half-length (tangential): 16 mm slot
BRACKET_PLATE_Z = (DRYWALL_T + 0.0005, TAB_Z + 0.012)  # vertical leg of the bracket on the can wall, above the drywall

# ---------------------------------------------------------------- trim (6 in. shallow baffle with torsion springs)
TRIM_MASS = 0.30  # kg (spun aluminium baffle + flange + springs)
FLANGE_R = (0.0760, 0.0970)  # flange: 194 mm outside diameter
FLANGE_T = 0.0015
BAFFLE_R = (0.0740, 0.0580)  # cone wall: radius at the flange, radius at the top
BAFFLE_H = 0.075
BAFFLE_T = 0.0015
BAFFLE_SEG = 40  # cone collision segments
TRIM_FRICTION = 0.4  # painted aluminium / steel on drywall, steel
# torsion springs (music wire 1.6 mm, two legs each)
SPRING_ANGLES = {"A": 90.0, "B": -90.0}  # trim angle of each spring (deg)
COIL_Z = 0.006  # coil height above the flange's bottom face
LEG_R = 0.0790  # radius of the legs' plane (tangent plane to the trim at this radius)
ROOT_U = 0.0030  # leg roots +-3 mm (tangential) from the coil centre
LEG_L = 0.070  # straight leg length (2-3/4 in.)
WIRE = 0.0016  # wire diameter
HOOK_L = 0.0030  # tip hook, bent tangentially outward
LEG_MASS = 0.006  # per leg (wire + its share of the coil)
THETA_FREE = math.radians(50.0)  # free half-angle of each leg from the trim axis (outward)
THETA_MIN = math.radians(-1.0)  # squeezed shut: the legs meet (a stiff computed stop, STOP_K, from here)
THETA_HARD = math.radians(-8.0)  # hard joint limit far past it (backup only)
STOP_K = 3.0  # N m / rad, the legs' meeting stop (computed, like the spring: a hard joint stop pressed by a finger
STOP_C = 0.002  # made the light leg jump out of the fingers in dev runs d3 / d4 / d7)
THETA_MAX = math.radians(80.0)  # hard limit (legs cannot fold flat)
SPRING_K = 0.11  # N m / rad per leg (about 12 N per finger to squeeze it shut 8 mm from the coil)
SPRING_C = 0.0008  # N m s / rad, leg damping (computed torque)
WIRE_FRICTION = 0.35  # music wire on the steel bracket

# ---------------------------------------------------------------- bench: the trim rests flange-down on its carton
CARTON = np.array([0.30, 0.0, TOP])  # carton bottom centre (setup default)
CARTON_SIZE = (0.150, 0.150, 0.120)  # 6 in. square, 120 mm tall corrugated carton (closed)
# named setups: (bracket angle phi, trim yaw psi0 on the carton) deg; phi turns both brackets about the housing axis
# (0 = brackets on the station y axis), psi0 turns the trim's spring line from the station y axis
SETUPS = ((-4.5, 2.0), (4.0, -2.0), (5.0, 1.0), (-3.5, 2.5), (3.5, -2.5))
# seeded trials: phi uniform over these segments (never within 3.5 deg of the robot's left-right line), psi0 uniform in
# +-3 deg with |psi0 - phi| >= 4 deg (the trim must always be turned), carton moved up to +-15 mm in x and +-6 mm in y.
# The ranges are what both arms reach with a side pinch on the springs (offline IK scans, DESIGN.md)
SEEDED_PHI = ((-5.0, -3.5), (3.5, 5.0))
SEEDED_PSI = 3.0
SEEDED_MIN_TURN = 4.0
SEEDED_CARTON_JITTER = (0.015, 0.006)


def bracket_angles(phi_deg):
    """Station-frame angle (deg) of bracket A (robot's left) and B (right) for bracket rotation phi."""
    return {"A": 90.0 + phi_deg, "B": -90.0 + phi_deg}


def setup_spec(setup):
    """Start state: {'phi', 'psi0', 'dx', 'dy', 'name'}. ``setup`` is a named setup index or ('seed', n)."""
    if isinstance(setup, tuple) and setup[0] == "seed":
        rng = np.random.default_rng(int(setup[1]))
        total = sum(hi - lo for lo, hi in SEEDED_PHI)
        u = float(rng.uniform(0.0, total))
        phi = SEEDED_PHI[-1][1]
        for lo, hi in SEEDED_PHI:
            if u < hi - lo:
                phi = lo + u
                break
            u -= hi - lo
        psi0 = phi
        while abs(psi0 - phi) < SEEDED_MIN_TURN:
            psi0 = float(rng.uniform(-SEEDED_PSI, SEEDED_PSI))
        dx = float(rng.uniform(-SEEDED_CARTON_JITTER[0], SEEDED_CARTON_JITTER[0]))
        dy = float(rng.uniform(-SEEDED_CARTON_JITTER[1], SEEDED_CARTON_JITTER[1]))
        return dict(phi=phi, psi0=psi0, dx=dx, dy=dy, name=f"seed {int(setup[1])}")
    k = int(setup) % len(SETUPS)
    phi, psi0 = SETUPS[k]
    return dict(phi=phi, psi0=psi0, dx=0.0, dy=0.0, name=f"setup {k}")


def carton_pos(setup):
    sp = setup_spec(setup)
    return CARTON + np.array([sp["dx"], sp["dy"], 0.0])


def trim_start_pose(setup):
    """Trim resting flange-down on the carton (T origin, rotation)."""
    sp = setup_spec(setup)
    return carton_pos(setup) + np.array([0.0, 0.0, CARTON_SIZE[2] + 0.0002]), rz(sp["psi0"])


# ---------------------------------------------------------------- springs and legs (trim frame)
def spring_frame(name):
    """(coil centre, radial unit, tangential unit) of spring ``name`` in the trim frame."""
    a = math.radians(SPRING_ANGLES[name])
    rad = np.array([math.cos(a), math.sin(a), 0.0])
    tan = np.array([-math.sin(a), math.cos(a), 0.0])
    return LEG_R * rad + np.array([0.0, 0.0, COIL_Z]), rad, tan


LEGS = ("Ap", "Am", "Bp", "Bm")  # spring, side (p = +tangential, m = -tangential)


def leg_root(leg):
    c, rad, tan = spring_frame(leg[0])
    s = 1.0 if leg[1] == "p" else -1.0
    return c + s * ROOT_U * tan


def leg_rot(leg, theta):
    """Leg frame rotation in the trim frame at half-angle theta (rad, + = spread outward): x = radial (pivot axis),
    z = along the leg."""
    _, rad, tan = spring_frame(leg[0])
    s = 1.0 if leg[1] == "p" else -1.0
    z0 = np.array([0.0, 0.0, 1.0])
    # spread: rotate z toward s*tan
    zl = math.cos(theta) * z0 + math.sin(theta) * s * tan
    xl = rad
    yl = np.cross(zl, xl)
    return np.column_stack([xl, yl, zl])


def leg_axis_sign(leg):
    """Rotation about the leg frame's +x (radial) by +d theta spreads the leg if this is +1."""
    _, rad, tan = spring_frame(leg[0])
    s = 1.0 if leg[1] == "p" else -1.0
    # d z / d theta = s * tan; rotation about x by +a sends z -> z cos a - y sin a ... y = z x x
    # with x = rad, z = up: y = up x rad; rotating about +x moves z toward -y = rad x up = tan (for rad . (up x tan)...)
    y = np.cross(np.array([0.0, 0.0, 1.0]), rad)
    return 1.0 if float(np.dot(-y, s * tan)) > 0 else -1.0


def leg_tip_local():
    """Tip of the straight leg in the leg frame."""
    return np.array([0.0, 0.0, LEG_L])


def leg_theta(r_trim, r_leg, leg):
    """Half-angle (rad) of a leg from the two world rotations (trim, leg)."""
    rl = r_trim.T @ r_leg  # leg in trim frame
    zl = rl[:, 2]
    _, rad, tan = spring_frame(leg[0])
    s = 1.0 if leg[1] == "p" else -1.0
    return math.atan2(float(zl @ (s * tan)), float(zl[2]))


def spring_torque(theta, omega=0.0):
    """Torque (N m, + = toward spreading) of one leg's spring at half-angle theta (rad) and rate omega (rad/s), plus
    the stop where the squeezed legs meet."""
    t = SPRING_K * (THETA_FREE - theta) - SPRING_C * omega
    if theta < THETA_MIN:
        t += STOP_K * (THETA_MIN - theta) - STOP_C * omega
    return t


def squeeze_force(theta, at_m):
    """Finger force (N) needed to hold a leg at half-angle theta, pressing at ``at_m`` from its pivot."""
    return SPRING_K * (THETA_FREE - theta) / at_m


# ---------------------------------------------------------------- trim geometry pieces (trim frame)
def annulus_cells(r0, r1, n, phase_deg=0.0):
    cells = []
    for i in range(n):
        a0 = math.radians(phase_deg + i * 360.0 / n)
        a1 = math.radians(phase_deg + (i + 1) * 360.0 / n)
        cells.append(
            np.array(
                [
                    [r0 * math.cos(a0), r0 * math.sin(a0)],
                    [r1 * math.cos(a0), r1 * math.sin(a0)],
                    [r1 * math.cos(a1), r1 * math.sin(a1)],
                    [r0 * math.cos(a1), r0 * math.sin(a1)],
                ]
            )
        )
    return cells


def prism(poly, z0, z1):
    poly = np.asarray(poly, float)
    k = len(poly)
    return np.r_[np.c_[poly, np.full(k, z0)], np.c_[poly, np.full(k, z1)]]


def cylinder(r, z0, z1, n=32, center=(0.0, 0.0)):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = lambda rr, z: np.c_[center[0] + rr * np.cos(a), center[1] + rr * np.sin(a), np.full(n, z)]  # noqa: E731
    return np.r_[ring(r, z0), ring(r, z1)]


def flange_pieces():
    return [prism(c, 0.0, FLANGE_T) for c in annulus_cells(FLANGE_R[0], FLANGE_R[1], 36)]


def baffle_pieces():
    """Cone wall segments (convex hulls of 8 points)."""
    out = []
    n = BAFFLE_SEG
    for i in range(n):
        pts = []
        for a in (2 * math.pi * i / n, 2 * math.pi * (i + 1) / n):
            for z, r in ((0.0, BAFFLE_R[0]), (BAFFLE_H, BAFFLE_R[1])):
                for dr in (0.0, -BAFFLE_T):
                    pts.append([(r + dr) * math.cos(a), (r + dr) * math.sin(a), z])
        out.append(np.array(pts))
    return out


def baffle_r_at(z):
    return BAFFLE_R[0] + (BAFFLE_R[1] - BAFFLE_R[0]) * z / BAFFLE_H


def leg_hull(leg):
    """Straight leg + tip hook, in the leg frame (two boxes as point sets)."""
    w = WIRE / 2
    straight = np.array([[x, y, z] for x in (-w, w) for y in (-w, w) for z in (0.0, LEG_L)])
    # hook: bent tangentially outward at the tip; the leg frame's outward tangential direction is -y*sign... computed
    return straight


def hook_dir_local(leg):
    """Unit vector (leg frame) of the hook: tangentially outward, perpendicular to the leg."""
    _, rad, tan = spring_frame(leg[0])
    s = 1.0 if leg[1] == "p" else -1.0
    r0 = leg_rot(leg, 0.0)
    return r0.T @ (s * tan)


def hook_hull(leg):
    w = WIRE / 2
    d = hook_dir_local(leg)
    x = np.array([1.0, 0.0, 0.0])
    zc = np.array([0.0, 0.0, LEG_L])
    pts = []
    for t in (-w, HOOK_L):
        for a in (-w, w):
            for b in (-w, w):
                pts.append(zc + t * d + a * x + b * np.array([0.0, 0.0, 1.0]))
    return np.array(pts)


# ---------------------------------------------------------------- housing pieces (housing frame)
def bracket_frame(beta_deg):
    a = math.radians(beta_deg)
    rad = np.array([math.cos(a), math.sin(a), 0.0])
    tan = np.array([-math.sin(a), math.cos(a), 0.0])
    return rad, tan


def tab_boxes(beta_deg):
    """Bracket tab around the slot: list of (centre, size(r, u, z), yaw_deg) boxes in the housing frame, plus the
    vertical plate on the can wall."""
    rad, tan = bracket_frame(beta_deg)
    z = TAB_Z - TAB_T / 2
    boxes = []

    def add(r0, r1, u0, u1, z0, z1):
        c = rad * (r0 + r1) / 2 + tan * (u0 + u1) / 2 + np.array([0.0, 0.0, (z0 + z1) / 2])
        boxes.append((c, (r1 - r0, u1 - u0, z1 - z0), beta_deg))

    zt0, zt1 = TAB_Z - TAB_T, TAB_Z
    add(TAB_R[0], SLOT_R[0], -TAB_U, TAB_U, zt0, zt1)  # inner strip
    add(SLOT_R[1], TAB_R[1], -TAB_U, TAB_U, zt0, zt1)  # outer strip
    add(SLOT_R[0], SLOT_R[1], SLOT_U, TAB_U, zt0, zt1)  # side +
    add(SLOT_R[0], SLOT_R[1], -TAB_U, -SLOT_U, zt0, zt1)  # side -
    add(CAN_R[0] - 0.0015, CAN_R[0], -TAB_U, TAB_U, *BRACKET_PLATE_Z)  # plate on the wall
    del z
    return boxes


def slot_center(beta_deg):
    rad, _ = bracket_frame(beta_deg)
    return rad * (SLOT_R[0] + SLOT_R[1]) / 2 + np.array([0.0, 0.0, TAB_Z])


def ceiling_pieces():
    """Drywall as convex pieces in the housing frame: 48 sector cells from the cut-out to a 260 mm square, plus four
    rectangles from the square to the panel edges."""
    z0, z1 = 0.0, DRYWALL_T
    sq = 0.130
    out = []

    def on_square(a):
        c, s = math.cos(a), math.sin(a)
        t = sq / max(abs(c), abs(s))
        return [t * c, t * s]

    n = 48
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        pts = [[CUTOUT_R * math.cos(a0), CUTOUT_R * math.sin(a0)], on_square(a0)]
        for k in range(4):  # square corners inside the sector
            ac = math.pi / 4 + k * math.pi / 2
            if a0 < ac < a1:
                pts.append([sq * math.copysign(1, math.cos(ac)), sq * math.copysign(1, math.sin(ac))])
        pts += [on_square(a1), [CUTOUT_R * math.cos(a1), CUTOUT_R * math.sin(a1)]]
        out.append(prism(np.array(pts), z0, z1))
    x0, x1 = CEIL_X[0] - H[0], CEIL_X[1] - H[0]
    y0, y1 = CEIL_Y[0] - H[1], CEIL_Y[1] - H[1]

    def rect(xa, xb, ya, yb):
        return prism(np.array([[xa, ya], [xb, ya], [xb, yb], [xa, yb]]), z0, z1)

    out.append(rect(x0, x1, sq, y1))
    out.append(rect(x0, x1, y0, -sq))
    out.append(rect(x0, -sq, -sq, sq))
    out.append(rect(sq, x1, -sq, sq))
    return out


def flange_points(n=12):
    """Points on the flange's top face at r = 92 mm (trim frame), where the gap to the ceiling is checked."""
    return [
        np.array([0.092 * math.cos(2 * math.pi * k / n), 0.092 * math.sin(2 * math.pi * k / n), FLANGE_T])
        for k in range(n)
    ]
