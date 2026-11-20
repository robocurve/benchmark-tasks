"""Insole outline, sneaker shape and start layout (pure numpy). Metres.

Shoe frame: origin on the table directly under the inside of the heel counter, on the centre line; +x heel -> toe,
+z up. The bed (the shoe's floor under the insole) is at height bed(x) above the table.
The shoe is a Brooks Ghost 18, men's US 9 (README "Sources"). Published or lab-measured numbers are marked (P);
every number marked (M) is not published and must be measured on the real shoe and insole (build/README.md).
"""
import math
import os

import numpy as np

TOP = 0.7475                      # station table top (world z)

# ---- insole: Spenco PolySorb Walker/Runner, size 3 (US men's 8/9), trimmed to the shoe's own insole ------------
L = 0.267                         # length heel -> toe after trimming: 4 mm shorter than the inside (M)
T = 0.007                         # thickness; maker: heel 7.50 mm, forefoot 6.80 mm (P)
MASS = 0.040                      # kg, one trimmed insole (M)
E = 4.0e6 * float(os.environ.get("INSOLE_E_SCALE", "1"))   # Pa, foam bending modulus: named parameter (M);
#   the env scale is for sensitivity runs only
NU = 0.3
W_U = np.array([0.0, 0.04, 0.10, 0.25, 0.42, 0.60, 0.72, 0.82, 0.90, 0.96, 1.0])
W_W = np.array([0.040, 0.056, 0.064, 0.066, 0.060, 0.082, 0.092, 0.090, 0.078, 0.056, 0.030])   # outline width (M)


def width(u):
    return np.interp(u, W_U, W_W)


def area():
    u = np.linspace(0, 1, 401)
    return float(np.trapezoid(width(u), u * L))


def ei_bend():
    """Bending stiffness of the whole insole about its width axis, N m^2 (mean width)."""
    return E * (area() / L) * T ** 3 / 12


# ---- sneaker: Brooks Ghost 18, men's US 9, its own insole taken out ---------------------------------------------
LI = 0.271                        # inside length on the bed, heel counter to toe box end (P: RunRepeat 271.0 mm)
THROAT = 0.130                    # the collar opening runs from the heel counter to the top eyelets (M)
H_COUNTER = 0.065                 # heel counter / back collar height above the bed (M)
H_SIDE = 0.050                    # side collar at the ankle (M)
# underside of the laced vamp above the local bed (M); toe box 31 mm = 25.8 mm toebox height (P) + 5 mm insole (P)
ROOF = ((0.130, 0.058), (0.185, 0.046), (0.230, 0.031), (0.271, 0.018))
WALL = 0.003
RT = 0.008                        # vamp (roof) collider thickness
WS = WALL                         # side wall collider thickness (outward of the inside outline)
BT = 0.012                        # bed plate thickness (into the midsole)
# bed height above the table: heel stack 36 mm and forefoot stack 26 mm (P, 10 mm drop), less the 5 mm original
# insole (P); the heel seat is level to 35 mm, the drop runs out at 180 mm, and the toe spring lifts the sole ahead
# of 190 mm, 10 mm at the outside tip (M)
B_HEEL, B_FORE = 0.031, 0.021
X_SEAT, X_BALL = 0.035, 0.180
X_TS, TS = 0.190, 0.010
X_TIP = LI + 0.014                # outside toe tip
X_BACK = -0.016                   # outside heel
SHOE_MASS = 0.27                  # kg: 283 g for a men's 9 with laces and insole (P), less the insole (M)
SHOE_XY = (0.33, 0.03)            # world position of the shoe-frame origin (toe along +x)
SHOE_Z = TOP

UPPER_X = (0.0, 0.04, 0.09, 0.20, 0.232, LI)
UPPER_HW = (0.030, 0.036, 0.046, 0.049, 0.044, 0.020)   # upper's inner half-width above the bed (M)
MID_X = (X_BACK, 0.0, 0.03, 0.10, 0.19, 0.235, LI, X_TIP)
MID_HW = (0.030, 0.042, 0.048, 0.042, 0.058, 0.052, 0.032, 0.012)   # midsole outer half-width: 96 / 116 mm (P)
CUP_X, CUP_RISE = 0.075, 0.008 * float(os.environ.get("INSOLE_CUP_SCALE", "1"))   # (env: diagnostics)    # heel cup: the bed curves up 8 mm at the sides of the heel, fading by 75 mm (M)


def toe_spring(x):
    x = np.asarray(x, float)
    return TS * np.clip((x - X_TS) / (X_TIP - X_TS), 0, None) ** 2


def bed(x):
    """Height of the bed (under the insole) above the table at x."""
    x = np.asarray(x, float)
    t = np.clip((x - X_SEAT) / (X_BALL - X_SEAT), 0, 1)
    out = B_FORE + (B_HEEL - B_FORE) * 0.5 * (1 + np.cos(math.pi * t)) + toe_spring(x)
    return float(out) if out.ndim == 0 else out


def bed_slope(x, h=1e-4):
    return (bed(x + h) - bed(x - h)) / (2 * h)


def half_in(x):
    return max(float(width(np.clip(x / L, 0, 1))) / 2 + 0.004, float(np.interp(x, UPPER_X, UPPER_HW)))


def bed_half(x):
    """Half-width of the level part of the bed: the heel cup rises outside it."""
    if x >= CUP_X:
        return half_in(x)
    return min(half_in(x), float(width(np.clip(x / L, 0, 1))) / 2 + 0.002)


def roof_h(x):
    xs, hs = zip(*ROOF)
    return float(np.interp(x, xs, hs))


def collar_h(x):
    return float(np.interp(x, [0, 0.05, 0.09, THROAT], [H_COUNTER, H_COUNTER - 0.006, H_SIDE, H_SIDE + 0.006]))


def top_h(x):
    """Top of the upper above the bed: the collar behind the throat, the laced vamp in front."""
    return collar_h(x) if x < THROAT else roof_h(x) + RT


def _q(yaw=0.0, pitch=0.0, roll=0.0):
    """Rz(yaw) Ry(pitch) Rx(roll) as xyzw."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


UPPER = (0.20, 0.21, 0.24)        # charcoal engineered mesh
OVERLAY = (0.42, 0.44, 0.48)      # grey eyelet stays, heel overlay
PAD = (0.30, 0.31, 0.35)          # collar padding and tongue
LINING = (0.74, 0.74, 0.72)       # light lining and the bed (strobel board)
MIDSOLE = (0.95, 0.95, 0.93)      # white EVA midsole
OUTSOLE = (0.10, 0.10, 0.11)      # black rubber outsole
LACE = (0.97, 0.97, 0.97)
EYELET = (0.06, 0.06, 0.07)

EYELETS_X = tuple(np.linspace(0.135, 0.215, 6))   # six eyelet pairs (M)
EYELET_Y = 0.021                                    # lace opening 42 mm wide (M)


def shoe_boxes():
    """(centre, size, colour, quat[, collide]) in the shoe frame. One rigid body; collide=False is drawn only."""
    out = []

    def box(c, s, col, q=(0, 0, 0, 1), collide=True):
        out.append((tuple(map(float, c)), tuple(map(float, s)), col, tuple(map(float, q)), collide))

    # outsole and white midsole, following the outline, with the toe spring: a fill per slice (narrowest width of
    # the slice, collides) and smooth sidewall pieces along the outline (drawn only)
    xs = np.linspace(X_BACK, X_TIP, 31)
    hwf = lambda x: float(np.interp(x, MID_X, MID_HW))
    for a, b in zip(xs[:-1], xs[1:]):
        xm, dx = (a + b) / 2, b - a
        hw = min(hwf(a), hwf(b))
        z0 = float(toe_spring(xm))
        pitch = -math.atan2(float(toe_spring(b) - toe_spring(a)), dx)
        top = bed(xm) - 0.004 if 0 <= xm <= LI else z0 + 0.016
        hmid = max(top - z0 - 0.003, 0.006)
        box((xm, 0, z0 + 0.0015), (dx + 0.002, 2 * hw + 0.002, 0.003), OUTSOLE, _q(0, pitch))
        box((xm, 0, z0 + 0.003 + hmid / 2), (dx + 0.002, 2 * hw, hmid), MIDSOLE, _q(0, pitch))
        hs = hmid + (0.010 if 0 <= xm <= LI else 0.0)       # the sidewall rises above the bed outside the upper
        for sd in (1, -1):
            pa, pb = np.array([a, sd * hwf(a)]), np.array([b, sd * hwf(b)])
            d = pb - pa
            yaw = math.atan2(d[1], d[0])
            c = (pa + pb) / 2 - sd * 0.004 * np.array([-d[1], d[0]]) / np.linalg.norm(d)
            box((c[0], c[1], z0 + 0.003 + hs / 2), (np.linalg.norm(d) + 0.002, 0.008, hs), MIDSOLE, _q(yaw, pitch),
                False)
            box((c[0], c[1], z0 + 0.0015), (np.linalg.norm(d) + 0.002, 0.009, 0.003), OUTSOLE, _q(yaw, pitch), False)
    # the bed: plates following the drop and the toe spring, level part plus the heel cup's raised sides
    xs = np.linspace(0, LI, 28)
    for a, b in zip(xs[:-1], xs[1:]):
        xm, dx = (a + b) / 2, b - a
        pitch = -math.atan(bed_slope(xm))
        bh = bed_half(xm)
        box((xm, 0, bed(xm) - BT / 2), (dx + 0.001, 2 * bh, BT), LINING, _q(0, pitch))
        run = half_in(xm) - bh
        if xm < CUP_X and run > 0.001:
            rise = CUP_RISE * (1 - xm / CUP_X)
            ang = math.atan2(rise, run)
            ln = math.hypot(rise, run) + 0.002
            for s in (1, -1):
                # the plate's top face runs from the level bed's edge up to the wall; its centre is half its
                # thickness below that face, along the face's normal
                yc = s * (bh + run / 2 + 0.004 * math.sin(ang))
                box((xm, yc, bed(xm) + rise / 2 - 0.004 * math.cos(ang)), (dx + 0.001, ln, 0.008), LINING,
                    _q(0, pitch, s * ang))
    # side walls (inner face follows half_in), top = collar behind the throat, vamp roof in front
    xs = np.linspace(0, LI, 21)
    for a, b in zip(xs[:-1], xs[1:]):
        xm = (a + b) / 2
        h = top_h(xm) + 0.002
        for s in (1, -1):
            pa, pb = np.array([a, s * half_in(a)]), np.array([b, s * half_in(b)])
            d = pb - pa
            yaw = math.atan2(d[1], d[0])
            nrm = np.array([-d[1], d[0]]) / np.linalg.norm(d)
            c = (pa + pb) / 2 + s * WS / 2 * nrm
            z0 = bed(xm) - 0.007        # the upper is lasted onto the midsole
            ztop = bed(xm) + h
            box((c[0], c[1], (z0 + ztop) / 2), (np.linalg.norm(d) + 0.004, WS, ztop - z0), UPPER, _q(yaw))
            if xm < THROAT:
                # padded collar along the top edge and the light lining inside it
                cp = (pa + pb) / 2 + s * (WS + 0.001) * nrm
                box((cp[0], cp[1], ztop - 0.005), (np.linalg.norm(d) + 0.004, 0.010, 0.010), PAD, _q(yaw))
                cl = (pa + pb) / 2 - s * 0.0006 * nrm
                box((cl[0], cl[1], bed(xm) + h / 2 + 0.004), (np.linalg.norm(d) + 0.004, 0.001, h - 0.010), LINING,
                    _q(yaw), False)
            else:
                # grey side overlay band (drawn only)
                co = (pa + pb) / 2 + s * (WS + 0.0006) * nrm
                box((co[0], co[1], bed(xm) + 0.5 * h), (np.linalg.norm(d) + 0.002, 0.001, 0.010), OVERLAY, _q(yaw),
                    False)
    # heel counter (back wall) with its padded top, the flared heel tab, and the toe box end
    zc = bed(0.0)
    box((-WALL / 2, 0.0, zc + (H_COUNTER - 0.007) / 2), (WALL, 2 * half_in(0.0) + 0.02, H_COUNTER + 0.007),
        UPPER, (0, 0, 0, 1))
    box((-WALL - 0.001, 0.0, zc + H_COUNTER - 0.005), (0.010, 2 * half_in(0.0) + 0.02, 0.010), PAD, (0, 0, 0, 1))
    box((-0.010, 0.0, zc + H_COUNTER + 0.004), (0.014, 0.036, 0.005), PAD, _q(0, -math.radians(35)))
    box((-WALL - 0.0015, 0.0, zc + 0.030), (0.001, 2 * half_in(0.0) + 0.016, 0.026), OVERLAY, (0, 0, 0, 1), False)
    zt = bed(LI)
    box((LI + WALL / 2, 0.0, zt + (roof_h(LI) - 0.007) / 2), (WALL, 0.05, roof_h(LI) + 0.007), UPPER, (0, 0, 0, 1))
    # vamp roof (the laced instep and the toe box), following the bed
    rx = np.linspace(THROAT, LI, 9)
    for a, b in zip(rx[:-1], rx[1:]):
        za, zb = bed(a) + roof_h(a), bed(b) + roof_h(b)
        pitch = -math.atan2(zb - za, b - a)
        ln = math.hypot(b - a, zb - za)
        wd = 2 * half_in((a + b) / 2) + 2 * WS
        box(((a + b) / 2, 0.0, (za + zb) / 2 + RT / 2), (ln + 0.004, wd, RT), UPPER, _q(0.0, pitch))
    # lacing (drawn only; the roof under it is the collider): tongue between the eyelet stays, six eyelet pairs,
    # criss-cross laces and a bow at the top pair
    def on_roof(x, y, dz=0.0):
        return np.array([x, y, bed(x) + roof_h(x) + RT + dz])

    xs0, xs1 = EYELETS_X[0] - 0.008, EYELETS_X[-1] + 0.008
    for s in (1, -1):
        ca, cb = on_roof(xs0, s * EYELET_Y, 0.0008), on_roof(xs1, s * EYELET_Y, 0.0008)
        pitch = -math.atan2(cb[2] - ca[2], cb[0] - ca[0])
        box((ca + cb) / 2, (np.linalg.norm(cb - ca), 0.012, 0.0016), OVERLAY, _q(0, pitch), False)
        for x in EYELETS_X:
            box(on_roof(x, s * EYELET_Y, 0.0018), (0.005, 0.005, 0.001), EYELET, (0, 0, 0, 1), False)
    ca, cb = on_roof(xs0, 0, 0.0005), on_roof(xs1, 0, 0.0005)
    box((ca + cb) / 2, (np.linalg.norm(cb - ca), 2 * EYELET_Y - 0.012, 0.001), PAD,
        _q(0, -math.atan2(cb[2] - ca[2], cb[0] - ca[0])), False)
    for a, b in zip(EYELETS_X[:-1], EYELETS_X[1:]):
        for s in (1, -1):
            pa, pb = on_roof(a, s * EYELET_Y, 0.0030), on_roof(b, -s * EYELET_Y, 0.0030)
            d = pb - pa
            box((pa + pb) / 2, (np.linalg.norm(d) + 0.003, 0.004, 0.0025), LACE,
                _q(math.atan2(d[1], d[0]), -math.atan2(d[2], math.hypot(d[0], d[1]))), False)
    x0 = EYELETS_X[0]
    box(on_roof(x0, 0, 0.003), (0.004, 2 * EYELET_Y + 0.004, 0.0025), LACE, (0, 0, 0, 1), False)
    for s in (1, -1):   # bow: two loops lying on the vamp and two loose ends
        for yaw, ln, off in ((s * math.radians(65), 0.030, 0.013), (s * math.radians(115), 0.030, 0.013)):
            c = on_roof(x0, 0, 0.0045) + off * np.array([math.cos(yaw), math.sin(yaw), 0])
            box(c, (ln, 0.005, 0.0025), LACE, _q(yaw), False)
        c = on_roof(x0 + 0.020, s * 0.010, 0.0045)
        box(c, (0.040, 0.004, 0.0025), LACE, _q(s * math.radians(20)), False)
    return out


def to_world(p_local):
    return np.asarray(p_local, float) + np.array([SHOE_XY[0], SHOE_XY[1], SHOE_Z])


# ---- the tongue: a soft padded flap sewn to the vamp under the laces; its free part stands up from the top eyelets
# and leans back over the front of the opening (non-gusseted, 9 mm padding (P)) ---------------------------------
TONGUE_N = 4
TONGUE_FREE = 0.050               # free length above the top lace (M)
TONGUE_W = 0.052                  # width (M)
TONGUE_T = 0.009                  # padding thickness (P: RunRepeat 9.0 mm)
TONGUE_MASS = 0.012               # kg, free part (M)
TONGUE_REST_DEG = 60.0            # rest pose: leans back, 60 deg above the horizontal (30 deg from upright) (M)
TONGUE_K = 0.015 * float(os.environ.get("INSOLE_TONGUE_K_SCALE", "1"))   # N m/rad per joint (4 joints): soft, it
#   sags under its own weight (M); the env scale is for the rigid-tongue diagnostic only
TONGUE_ROOT = (THROAT, 0.0, bed(THROAT) + roof_h(THROAT) + RT + TONGUE_T / 2 - 0.002)


TONGUE_MID_DEG = 115.0           # the root joint's frame: in the middle of the tongue's range, from leaning back at
#   60 deg to folded forward on the laces at about 175 deg (angles above the horizontal, measured toward the heel)


def tongue_rest_rot(deg=None):
    """Rotation (columns: the segment's x along the tongue to its tip, y, z = outer face) in the shoe frame."""
    a = math.radians(TONGUE_REST_DEG if deg is None else deg)
    x = np.array([-math.cos(a), 0.0, math.sin(a)])
    y = np.array([0.0, -1.0, 0.0])
    return np.column_stack([x, y, np.cross(x, y)])


def tongue_layout():
    """Segment centres (shoe frame) and the common orientation of the free tongue at rest."""
    R = tongue_rest_rot()
    s = TONGUE_FREE / TONGUE_N
    c = np.array(TONGUE_ROOT) + np.outer((np.arange(TONGUE_N) + 0.5) * s, R[:, 0])
    return c, R, s


# ---- start layout: the shoe box beside the shoe, the insole lying on its lid, heel end overhanging the near edge --
BOX_SIZE = (0.30, 0.19, 0.12)     # shoe box, cardboard (M)
BOX_C = (0.55, -0.17, TOP + 0.06)
OVERHANG = 0.040


def insole_start():
    """Heel point (x, y, z at mid-thickness) and heading of the insole: top up, toe away from the robot."""
    box_top = BOX_C[2] + BOX_SIZE[2] / 2 + T / 2 + 0.0005
    near = BOX_C[0] - BOX_SIZE[0] / 2
    return (near - OVERHANG, BOX_C[1], box_top), 0.0


def chain_layout(n, heel, yaw=0.0):
    s = L / n
    u = (np.arange(n) + 0.5) / n
    d = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    return np.asarray(heel, float) + np.outer(u * L, d), width(u), s


if __name__ == "__main__":
    print("area m2", round(area(), 5), "EI N m2", round(ei_bend(), 6), "w N/m", round(MASS * 9.81 / L, 3))
    print("bed heel/ball/toe mm", [round(bed(x) * 1e3, 1) for x in (0, 0.1, 0.18, 0.23, LI)])
    print("boxes", len(shoe_boxes()), "colliders", sum(1 for b in shoe_boxes() if b[4]), "insole heel", insole_start())
    print("tongue", tongue_layout()[0].round(4).tolist())
