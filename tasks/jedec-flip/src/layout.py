"""Scene layout and geometry of the jedec-flip task (station frame: x forward from the arm bases, y to the robot's left,
z up; metres). Engine-independent (numpy only).

Tray frame T: origin at the centre of the tray outline at the bottom edge of its walls; x along the 322.6 mm length,
y along the 135.9 mm width, z up out of the pocket (top) side. The pin-1 chamfer is at the (+x, +y) corner.
Chip frame P: origin at the centre of the ball field's bottom (the seating plane of the balls); z out of the lid side.

Tray: JEDEC matrix-tray outline (12.7 x 5.35 in, 322.6 x 135.9 mm, 7.62 mm stack pitch); the pocket, rib and stacking
details are representative (vendor drawings differ; see README). 3 x 7 pockets for 35 x 35 mm FCBGA packages.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena TABLE_TOP["station"])
MM = 1e-3


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rx(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def ry(rad):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# ---------------------------------------------------------------- tray (JEDEC outline, representative interior)
TRAY_L = 322.6 * MM
TRAY_W = 135.9 * MM
TRAY_H = 7.62 * MM  # wall height = stack pitch
CHAMFER = 8.0 * MM  # pin-1 corner chamfer legs (45 deg), at (+x, +y)
WALL_OUT_T = 2.0 * MM  # outer wall skin, full height
WALL_IN_T = 2.0 * MM  # inner wall layer (inset 2-4 mm), rebated 1.5 mm at the bottom
REBATE_H = 1.5 * MM
LIP_INSET = (2.5 * MM, 3.5 * MM)  # stacking lip on the wall top, inset from the outline
LIP_H = 1.4 * MM
LIP_CHAMFER = 0.4 * MM  # lead-in on the lip's outer top edge
LIP_END_GAP = 14.0 * MM  # lip segments stop this far from the outline corners
WEB_Z = (3.0 * MM, 4.2 * MM)  # floor web
TOP_RIB_Z = (4.2 * MM, 7.6 * MM)  # pocket walls on the top side
UNDER_RIB_Z = (0.4 * MM, 3.0 * MM)  # cell walls on the underside (hold the parts dead-bug after a flip)
POCKET = 36.0 * MM  # pocket opening (0.5 mm a side on a 35 mm package)
RIB_T = 4.0 * MM
PITCH = POCKET + RIB_T  # 40 mm
NX, NY = 7, 3
GRID_HALF = (
    (NX * POCKET + (NX - 1) * RIB_T) / 2,
    (NY * POCKET + (NY - 1) * RIB_T) / 2,
)  # 138, 58 mm
PERIM_T = 2.0 * MM  # perimeter rib around the pocket grid
SLOT_X = (GRID_HALF[0] + PERIM_T + 0.5 * MM, TRAY_L / 2 - WALL_OUT_T - WALL_IN_T)  # end handling slot, along x
SLOT_HALF_W = 35.0 * MM  # slot width 70 mm (across y)
TRAY_MASS = 0.20  # kg, carbon-loaded mPPE tray (representative)
TRAY_COLOR = (0.07, 0.07, 0.08)
TRAY_FRICTION = 0.4  # tray on tray / riser / table (estimate)
STACK_DZ = TRAY_H  # nested: upper tray frame = lower tray frame + 7.62 mm in z


def pocket_centers():
    """(x, y) of the 21 pocket centres in the tray frame, row-major (j across y, i along x)."""
    out = []
    for j in range(NY):
        for i in range(NX):
            out.append(
                np.array([(i - (NX - 1) / 2) * PITCH, (j - (NY - 1) / 2) * PITCH])
            )
    return out


def box_v(x0, x1, y0, y1, z0, z1):
    return np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)])


def _chamfer_clip(v):
    """Cut a box's vertices by the pin-1 chamfer plane (x + y <= L/2 + W/2 - CHAMFER) if it crosses it."""
    lim = TRAY_L / 2 + TRAY_W / 2 - CHAMFER
    return v if (v[:, 0] + v[:, 1]).max() <= lim + 1e-12 else None


def tray_pieces():
    """Convex collision pieces of one tray in its frame: list of (name, vertices). All boxes (a few wedges)."""
    L2, W2 = TRAY_L / 2, TRAY_W / 2
    P = []
    c = CHAMFER
    # outer wall skin (0-2 mm inset), full height; the +x/+y walls stop at the chamfer, a diagonal piece closes it
    t = WALL_OUT_T
    H = TRAY_H
    P.append(("wall_xn", box_v(-L2, -L2 + t, -W2, W2, 0.0, H)))
    P.append(("wall_xp", box_v(L2 - t, L2, -W2, W2 - c, 0.0, H)))
    P.append(("wall_yn", box_v(-L2, L2, -W2, -W2 + t, 0.0, H)))
    P.append(("wall_yp", box_v(-L2, L2 - c, W2 - t, W2, 0.0, H)))
    # diagonal chamfer wall: a prism between the chamfer line and a parallel line 2*sqrt2 inside
    d = 2.0 * t * math.sqrt(2.0)
    a = np.array([L2, W2 - c])
    b = np.array([L2 - c, W2])
    n_in = np.array([-1.0, -1.0]) / math.sqrt(2.0)
    quad = [a, b, b + n_in * d / 2 * math.sqrt(2.0), a + n_in * d / 2 * math.sqrt(2.0)]
    P.append(("wall_ch", prism(quad, 0.0, H)))
    # inner wall layer (inset 2-4 mm), from the rebate top to the wall top
    t2 = WALL_IN_T
    z0 = REBATE_H
    P.append(("iwall_xn", box_v(-L2 + t, -L2 + t + t2, -W2 + t, W2 - t, z0, H)))
    P.append(("iwall_xp", box_v(L2 - t - t2, L2 - t, -W2 + t, W2 - c - 2 * t, z0, H)))
    P.append(("iwall_yn", box_v(-L2 + t, L2 - t, -W2 + t, -W2 + t + t2, z0, H)))
    P.append(("iwall_yp", box_v(-L2 + t, L2 - c - 2 * t, W2 - t - t2, W2 - t, z0, H)))
    a2, b2 = a + n_in * d / 2 * math.sqrt(2.0), b + n_in * d / 2 * math.sqrt(2.0)
    P.append(
        (
            "iwall_ch",
            prism([a2, b2, b2 + n_in * d / 2 * math.sqrt(2.0), a2 + n_in * d / 2 * math.sqrt(2.0)], z0, H),
        )
    )
    # stacking lip on the wall top: trapezoid section (lead-in chamfer on the outer top edge), straight segments
    i0, i1 = LIP_INSET
    g = LIP_END_GAP

    def lip_y(sign):
        y_out, y_in = sign * (W2 - i0), sign * (W2 - i1)
        y_top = sign * (W2 - i0 - LIP_CHAMFER)
        v = []
        for x in (-L2 + g, L2 - g):
            v += [[x, y_out, H], [x, y_in, H], [x, y_in, H + LIP_H], [x, y_top, H + LIP_H]]
        return np.array(v)

    def lip_x(sign):
        x_out, x_in = sign * (L2 - i0), sign * (L2 - i1)
        x_top = sign * (L2 - i0 - LIP_CHAMFER)
        v = []
        for y in (-W2 + g, W2 - g):
            v += [[x_out, y, H], [x_in, y, H], [x_in, y, H + LIP_H], [x_top, y, H + LIP_H]]
        return np.array(v)

    P += [("lip_yp", lip_y(1)), ("lip_yn", lip_y(-1)), ("lip_xp", lip_x(1)), ("lip_xn", lip_x(-1))]
    # floor web, with the two end handling slots left open
    wi = t + t2  # web meets the inner wall face
    sx0, sx1 = SLOT_X
    zw0, zw1 = WEB_Z
    P.append(("web_mid", box_v(-sx0, sx0, -W2 + wi, W2 - wi, zw0, zw1)))
    for s, nm in ((1, "p"), (-1, "n")):
        xa, xb = sorted((s * sx0, s * (L2 - wi)))
        P.append((f"web_{nm}_a", box_v(xa, xb, SLOT_HALF_W, W2 - wi - (c if s > 0 else 0.0), zw0, zw1)))
        P.append((f"web_{nm}_b", box_v(xa, xb, -W2 + wi, -SLOT_HALF_W, zw0, zw1)))
    # pocket walls (top) and cell walls (underside): same plan
    gx, gy = GRID_HALF
    pt = PERIM_T
    for side, (za, zb) in (("t", TOP_RIB_Z), ("u", UNDER_RIB_Z)):
        P.append((f"{side}_per_yp", box_v(-gx - pt, gx + pt, gy, gy + pt, za, zb)))
        P.append((f"{side}_per_yn", box_v(-gx - pt, gx + pt, -gy - pt, -gy, za, zb)))
        P.append((f"{side}_per_xp", box_v(gx, gx + pt, -gy, gy, za, zb)))
        P.append((f"{side}_per_xn", box_v(-gx - pt, -gx, -gy, gy, za, zb)))
        for j in range(1, NY):
            y = -gy + j * PITCH - RIB_T / 2
            P.append((f"{side}_rx{j}", box_v(-gx, gx, y - RIB_T / 2, y + RIB_T / 2, za, zb)))
        for i in range(1, NX):
            x = -gx + i * PITCH - RIB_T / 2
            P.append((f"{side}_ry{i}", box_v(x - RIB_T / 2, x + RIB_T / 2, -gy, gy, za, zb)))
    return P


def tray_inertia(m=TRAY_MASS):
    a, b, h = TRAY_L, TRAY_W, TRAY_H
    return np.array([m * (b * b + h * h) / 12, m * (a * a + h * h) / 12, m * (a * a + b * b) / 12])


TRAY_COM = (0.0, 0.0, 3.8 * MM)

# ---------------------------------------------------------------- package: 35 x 35 mm lidded FCBGA, 1156 balls
CHIP_BALL = (33.0 * MM, 0.45 * MM)  # ball field (one collision slab), side, height
CHIP_SUB = (35.0 * MM, 1.15 * MM)  # organic substrate
CHIP_LID = (31.0 * MM, 1.80 * MM)  # nickel-plated copper lid
CHIP_H = CHIP_BALL[1] + CHIP_SUB[1] + CHIP_LID[1]  # 3.40 mm
CHIP_MASS = 0.012  # kg
CHIP_FRICTION = 0.35
CHIP_COM = (0.0, 0.0, 2.0 * MM)


def chip_pieces():
    hb, hs, hl = CHIP_BALL[1], CHIP_SUB[1], CHIP_LID[1]
    b, s, lid = CHIP_BALL[0] / 2, CHIP_SUB[0] / 2, CHIP_LID[0] / 2
    return [
        ("balls", box_v(-b, b, -b, b, 0.0, hb)),
        ("sub", box_v(-s, s, -s, s, hb, hb + hs)),
        ("lid", box_v(-lid, lid, -lid, lid, hb + hs, hb + hs + hl)),
    ]


def chip_inertia(m=CHIP_MASS):
    a, h = CHIP_SUB[0], CHIP_H
    return np.array([m * (a * a + h * h) / 12, m * (a * a + h * h) / 12, m * a * a / 6])


# ---------------------------------------------------------------- bench layout
RISER = np.array([0.330, 0.0, TOP])  # riser bottom centre on the table
RISER_SIZE = (0.150, 0.220, 0.055)  # x (across the tray) 150, y (along the tray) 220, 55 mm tall
RISER_TOP = TOP + RISER_SIZE[2]
RISER_COLOR = (0.55, 0.12, 0.10)  # ESD-safe riser (dissipative), dark red
B_HOME = np.array([0.150, 0.0, TOP])  # empty tray on the table, nominal tray-frame origin
A_HOME = np.array([RISER[0], 0.0, RISER_TOP])  # loaded tray on the riser
TRAY_YAW0 = 90.0  # tray x (length) along world +y; pin-1 chamfer toward the robot's left and front

# start variation (seeded): loaded tray moved on the riser, empty tray moved on the table; never near-aligned
A_JITTER = dict(dx=0.010, dy=0.005, dyaw=4.0)
B_JITTER = dict(dx=0.012, dy=0.015, dyaw=8.0)
MIN_REL_OFFSET = 0.008  # the empty tray starts at least 8 mm (x/y) off the loaded tray's centreline or 3 deg off in yaw
MIN_REL_YAW = 3.0
MIN_A_MISFIT = 0.003  # the loaded tray starts >= 3 mm off its nominal pose at a tray end (lip capture is ~0.9 mm)


def a_misfit(a):
    """Largest offset of the loaded tray's ends from its nominal pose (x/y shift plus yaw at the 161 mm half-length)."""
    end = math.radians(abs(a[2])) * TRAY_L / 2
    return max(abs(a[0]) + end, abs(a[1]))


SETUPS = (  # named setups: (A dx, dy, dyaw), (B dx, dy, dyaw); metres / deg
    ((0.006, -0.003, -2.5), (-0.010, 0.012, 6.0)),
    ((-0.008, 0.004, 3.0), (0.010, -0.014, -7.0)),
    ((0.004, -0.003, 1.5), (0.012, 0.010, 4.0)),
    ((0.009, 0.002, 3.5), (-0.012, -0.008, -5.0)),
    ((-0.005, -0.004, -3.5), (0.006, 0.015, 7.5)),
)


def setup_spec(setup):
    """{'a': (dx, dy, dyaw), 'b': (dx, dy, dyaw), 'name'} for a named setup index or ('seed', n)."""
    if isinstance(setup, tuple) and setup[0] == "seed":
        rng = np.random.default_rng(int(setup[1]))
        for _ in range(200):
            a = (
                float(rng.uniform(-A_JITTER["dx"], A_JITTER["dx"])),
                float(rng.uniform(-A_JITTER["dy"], A_JITTER["dy"])),
                float(rng.uniform(-A_JITTER["dyaw"], A_JITTER["dyaw"])),
            )
            b = (
                float(rng.uniform(-B_JITTER["dx"], B_JITTER["dx"])),
                float(rng.uniform(-B_JITTER["dy"], B_JITTER["dy"])),
                float(rng.uniform(-B_JITTER["dyaw"], B_JITTER["dyaw"])),
            )
            # offset of the empty tray from where a fixed (home-to-home) move would put it, and of the loaded tray
            # from its nominal pose (so a cover put down on the nominal pose without looking never fits the lip)
            if (
                max(abs(b[1] - a[1]), abs(b[0] - a[0])) >= MIN_REL_OFFSET or abs(b[2] - a[2]) >= MIN_REL_YAW
            ) and a_misfit(a) >= MIN_A_MISFIT:
                break
        return dict(a=a, b=b, name=f"seed {int(setup[1])}")
    k = int(setup) % len(SETUPS)
    a, b = SETUPS[k]
    return dict(a=a, b=b, name=f"setup {k}")


def tray_pose(home, d):
    dx, dy, dyaw = d
    return np.asarray(home, float) + np.array([dx, dy, 0.0]), rz(TRAY_YAW0 + dyaw)


def start_poses(setup):
    """{'A': (pos, rot), 'B': (pos, rot), 'chips': [(pos, rot)] * 21} at the start of a trial."""
    sp = setup_spec(setup)
    pa, ra = tray_pose(A_HOME, sp["a"])
    pb, rb = tray_pose(B_HOME, sp["b"])
    chips = []
    for c in pocket_centers():
        chips.append((pa + ra @ np.array([c[0], c[1], WEB_Z[1] + 0.00002]), ra.copy()))
    return dict(A=(pa + np.array([0, 0, 0.00002]), ra), B=(pb + np.array([0, 0, 0.00002]), rb), chips=chips)


def prism(poly, z0, z1):
    poly = np.asarray(poly, float)
    k = len(poly)
    return np.r_[np.c_[poly, np.full(k, z0)], np.c_[poly, np.full(k, z1)]]


# ---------------------------------------------------------------- relations used by the scorer and the demo
def rel(pa, ra, pb, rb):
    """Pose of frame b in frame a: (p, R)."""
    return ra.T @ (np.asarray(pb) - np.asarray(pa)), ra.T @ rb


def yaw_of(r):
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def nest_error(pa, ra, pb, rb):
    """Upper tray b relative to its nested pose on tray a: (dx, dy, dz, dyaw_deg, tilt_deg) in a's frame."""
    p, r = rel(pa, ra, pb, rb)
    tilt = math.degrees(math.acos(float(np.clip(r[2, 2], -1.0, 1.0))))
    return (
        float(p[0]),
        float(p[1]),
        float(p[2] - STACK_DZ),
        math.degrees(math.atan2(r[1, 0], r[0, 0])),
        tilt,
    )
