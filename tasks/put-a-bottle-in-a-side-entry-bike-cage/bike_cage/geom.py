"""Geometry of the bike's main triangle, the bottle cage and the bottle. NumPy only, so the task file and the tests use it
without Isaac Sim. Created 11_16_26 (America/New_York).

Frame: Salsa Rangefinder, size XS. Every in-plane position below was read from Salsa's own 1:1 drawing of that frame's
front triangle ("Salsa Rangefinder - Front Triangle Frame Bag Guide", page 1, "Drawing size is 68 x 44 in, scale 1:1",
dimensions +-2 mm; https://cdn.shopify.com/s/files/1/0912/4973/8003/files/470236_SAL_Frame_Bag_Guide_Rangefinder_VF_OL.pdf).
The drawing is vector art at true scale (1 pt = 25.4/72 mm; its 127 mm scale bar measures 127.00 mm), so tube edges and
the three down-tube bottle bosses were taken from the drawing's own line coordinates (dev/frame_from_drawing.py). Its
head tube axis lies 67.9 degrees from the page's horizontal, against Salsa's published 68.0 degrees head angle (27.5+
build; 68.6 for the 29 in build), so the page's horizontal is taken as level ground. Dimensions it labels and we
reproduce: bosses 64 mm apart, the lowest 143.1 mm up the down tube from the seat-tube/down-tube corner (we measure
143.0), down tube 551.7 mm and top tube 425.6 mm along their inner edges.

Frame coordinates (m): X forward along the bike, Z up, origin at the bottom bracket's centre; the frame's mid-plane is
X-Z. The task sets the frame in the world with X along world +y and the mid-plane at world x = XF, so a frame point
(X, Z) is the world point (XF + lateral, BB_Y + X, TOP + BB_H + Z), lateral along world x (+ away from the robot).

Not published, chosen and stated: tube widths across the frame (down tube 45 mm, top tube 40 mm, seat tube 34.9 mm,
head tube 50 mm); the cage's slot position on the bosses (the cage's floor CAGE_BELOW below the lower bolt) and the
bottle's stand-off from the down tube (STANDOFF). Cage: Elite Custom Race X (74 mm, 147.3 mm tall, 28.5 g; spec sheet
https://api.elite-it.com/assets/documents/Schede-tecniche/CUSTOM-RACE-X_2026_EN.pdf). Bottle: Elite Fly Tex 550 ml
(74 mm, 184.5 mm, 51 g; https://api.elite-it.com/assets/documents/FLY-TEX_2026_EN.pdf; 54 mm mouth opening,
https://www.elite-it.com/en/water-bottles/sport/fly-tex).
"""

import math

import numpy as np

MM = 0.001


def _p(x, z):
    return np.array((x, z), float) * MM


# -- frame, from the drawing (mm from the bottom bracket centre, X forward, Z up) ---------------------------------------
DT_IN = (_p(70.3, 76.8), _p(289.2, 470.0))  # down tube: its inner (upper) edge, straight part
DT_DEPTH = 49.1 * MM  # down tube, in the frame's plane (drawing)
DT_BEND = (_p(10.1, 28.4), _p(70.3, 76.8))  # the bend into the bottom bracket (R95.5 on the drawing), as one chord
TT_LOW = tuple(_p(*q) for q in ((-48.4, 275.7), (16.3, 318.6), (81.5, 360.9), (147.1, 402.1), (199.2, 434.0), (251.5, 465.2), (304.2, 496.0)))
TT_DEPTH = 31.0 * MM  # top tube, in the frame's plane (drawing: its upper edge runs about 31 mm above the lower)
ST_FRONT = tuple(_p(*q) for q in ((10.1, 28.4), (-4.4, 89.1), (-11.6, 117.5), (-23.4, 157.4), (-56.6, 270.2), (-75.0, 332.0)))
ST_DEPTH = 33.0 * MM  # seat tube, in the frame's plane (drawing)
BOSSES = (_p(105.0, 139.2), _p(136.2, 195.1), _p(167.3, 251.0))  # down-tube bottle bosses, on its inner edge (drawing)
HT_AXIS_DEG = 67.9  # head tube axis from horizontal (drawing; Salsa: 68.0)
HT_CENTRE = _p(478.0, 500.0)  # middle of the head tube (drawing, approximate: only its edges are drawn)
HT_LEN = 95.0 * MM  # XS head tube length (Salsa geometry table)
BB_R = 19.0 * MM  # bottom-bracket shell radius (drawing: 37.7 mm circle)
BB_W = 73.0 * MM  # shell width (73 mm mountain-bike shell)
WIDTH = {"dt": 45.0 * MM, "tt": 40.0 * MM, "st": 34.9 * MM, "ht": 50.0 * MM}  # across the frame (not published)

_d = DT_IN[1] - DT_IN[0]
U = _d / np.linalg.norm(_d)  # up the down tube (and the seated bottle's axis), in the frame plane
N = np.array((-U[1], U[0]))  # square to it, away from the down tube into the triangle: the cage's "front"
DT_DEG = math.degrees(math.atan2(U[1], U[0]))  # 60.9 degrees above horizontal

# -- cage (Elite Custom Race X) ---------------------------------------------------------------------------------------
BOSS_PAIR = (1, 2)  # the cage is bolted to the upper two of the three bosses
CAGE_H = 147.3 * MM  # spec sheet
BOTTLE_R = 37.0 * MM  # 74 mm
CAGE_BELOW = 35.0 * MM  # the cage's floor this far below its lower bolt, along the tube (slotted holes; not published)
STANDOFF = 8.0 * MM  # bottle surface to down-tube surface, through the cage's spine (not published)
AXIS_OFF = BOTTLE_R + STANDOFF  # the seated bottle's axis from the down tube's inner edge
FLOOR_T = 4.0 * MM
CAGE_O = BOSSES[BOSS_PAIR[0]] - CAGE_BELOW * U + AXIS_OFF * N  # frame point: the seated bottle's axis at its base
SPINE = dict(n=(-AXIS_OFF, -BOTTLE_R - 1.0 * MM), s=(-12.0 * MM, 12.0 * MM), a=(-FLOOR_T, CAGE_H - FLOOR_T))
FLOOR = dict(n=(-AXIS_OFF, BOTTLE_R + 5.0 * MM), s=(-40.0 * MM, 40.0 * MM), a=(-FLOOR_T, 0.0))
LIP_FRONT = dict(n=(BOTTLE_R + 1.0 * MM, BOTTLE_R + 4.5 * MM), s=(-30.0 * MM, 30.0 * MM), a=(-FLOOR_T, 8.0 * MM))
LIP_NEAR = dict(n=(-22.0 * MM, 22.0 * MM), s=(BOTTLE_R + 1.0 * MM, BOTTLE_R + 4.5 * MM), a=(-FLOOR_T, 5.0 * MM))  # the bottom lip the base clicks over
WALL_FAR = dict(n=(-30.0 * MM, 22.0 * MM), s=(-BOTTLE_R - 4.5 * MM, -BOTTLE_R - 1.0 * MM), a=(0.0, 70.0 * MM))  # the far side's low wall
# the sprung arm: hinged at the far-back corner, wrapping round the far side and the front to a tip on the near side
HINGE = np.array((-41.0 * MM, -30.0 * MM))  # (s, n) in the cage frame
ARM_R = (BOTTLE_R + 1.5 * MM, BOTTLE_R + 4.5 * MM)  # inner and outer radius of its band
ARM_A = (48.0 * MM, 92.0 * MM)  # the band's span up the cage (the real cage's front band)
ARM_PHI = (-35.0, 146.0)  # degrees round the bottle from the far side (0) through the front (90) to the near side
ARM_SEGS = 9
ARM_MASS = 0.008  # kg (the whole cage is 28.5 g)
ARM_LIMIT_DEG = 40.0  # it opens no further (the real arm would crack)
OVERBENT_DEG = 30.0  # opened past this at any time: over-bent (the plastic is strained past its working flex)
# the spring: a preloaded torsion spring at the hinge (stiffness per degree, aimed past closed, so it rests on its stop)
SPRING_K = 0.015  # N m per degree
SPRING_AIM_DEG = -8.0  # 0.12 N m preload when closed
TIP_LEVER = None  # set below: hinge to arm tip


def arm_point(phi_deg, r):
    """(s, n) of a point of the arm's band at angle phi round the bottle and radius r (cage frame, closed arm)."""
    p = math.radians(phi_deg)
    return np.array((-r * math.cos(p), r * math.sin(p)))


TIP = arm_point(ARM_PHI[1], ARM_R[0])
TIP_LEVER = float(np.linalg.norm(TIP - HINGE))


def tip_force(angle_deg):
    """Force (N) at the arm's tip, square to its lever, to hold it open at angle_deg (0 = closed)."""
    return SPRING_K * (angle_deg - SPRING_AIM_DEG) / TIP_LEVER


# -- bottle (Elite Fly Tex 550 ml) --------------------------------------------------------------------------------------
BOTTLE_H = 184.5 * MM
BOTTLE_MASS = 0.051  # empty
BODY_TOP = 160.0 * MM  # the body's straight part ends here; above it the cap (approximate)
CAP_R = 29.5 * MM  # cap skirt: the 54 mm mouth plus its wall (approximate)

# -- the cage frame in frame coordinates -------------------------------------------------------------------------------
# cage frame: s across the frame (+ toward the robot, the near side, = world -x), n = N, a = U (up the cage)


def cage_to_frame(s, n, a):
    """(lateral, X, Z) in frame coordinates of the cage-frame point (s, n, a); lateral + is world +x (far side)."""
    p = CAGE_O + n * N + a * U
    return np.array((-s, p[0], p[1]))


def cage_rotation():
    """3x3 rotation whose columns are the cage frame's s, n, a axes in frame coordinates (lateral, X, Z)."""
    return np.column_stack(((-1.0, 0.0, 0.0), (0.0, N[0], N[1]), (0.0, U[0], U[1])))


def headroom_along_axis(n_offset=BOTTLE_R):
    """Distance (m) from the seated bottle's top, at offset n_offset from its axis, straight up its axis to the top tube's
    lower edge: how far the bottle could be lifted out of the cage along its axis."""
    p = CAGE_O + n_offset * N + BOTTLE_H * U
    best = math.inf
    for a, b in zip(TT_LOW[:-1], TT_LOW[1:]):
        m = np.column_stack((U, -(b - a)))
        try:
            t, s = np.linalg.solve(m, a - p)
        except np.linalg.LinAlgError:
            continue
        if 0.0 <= s <= 1.0 and t > 0.0:
            best = min(best, float(t))
    return best


def gap_to_top_tube():
    """Smallest distance (m) from the seated bottle's outline (in the frame plane) to the top tube's lower edge."""
    best = math.inf
    for a_ in np.linspace(0.0, BOTTLE_H, 120):
        for side in (-BOTTLE_R if a_ < BODY_TOP else -CAP_R, BOTTLE_R if a_ < BODY_TOP else CAP_R):
            p = CAGE_O + side * N + a_ * U
            for a, b in zip(TT_LOW[:-1], TT_LOW[1:]):
                ab = b - a
                t = float(np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0.0, 1.0))
                best = min(best, float(np.linalg.norm(p - (a + t * ab))))
    return best
