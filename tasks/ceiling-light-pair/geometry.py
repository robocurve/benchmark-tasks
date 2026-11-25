"""Every size, place and constant of the ceiling-light-pair task (metres, kg, N, s).

World frame of yam_pair's `facing` layout: origin midway between the two stations, x from station A toward station B
(A's arms face +x, B's face -x), y to A's left, z up. Heights are given above the table top TOP. Station frames are
yam_arena's: x forward from the arms, y to their left. Items marked "estimate" have no published source; the keyhole
sizes are "estimate; the real slots decide"."""

from __future__ import annotations

import math

TOP = 0.7475                      # yam_pair TABLE_TOP
END_GAP = 0.045                   # each end of the mounted light (its outline) is 45 mm in front of its station's arm bases
HOUSING_L, HOUSING_W, HOUSING_D = 1.218, 0.320, 0.055     # length: spec sheet; width (back panel) and depth: estimates
SEPARATION = 2 * (END_GAP + HOUSING_L / 2)                 # 1.308 m between the station origins

# -- ceiling stand-in -----------------------------------------------------------------------------------------------
CEIL = TOP + 0.760                # panel underside, 760 mm above the table
PANEL_T = 0.018                   # 23/32 in plywood
PANEL_X = (-0.70, 0.70)           # 1,400 x 600 mm
PANEL_Y = (-0.30, 0.30)
RAIL = (0.038, 0.089)             # two 2x4 rails on edge along the panel's top, over its long edges (stiffening)
LEG = (0.089, 0.038)              # 2x4 legs, wide face along x
LEG_Y = PANEL_Y[1] + LEG[1] / 2  # legs flush against the panel's long edges (y +/-319), near the middle, clear of the
LEG_XY = ((-0.12, LEG_Y), (0.12, LEG_Y), (-0.12, -LEG_Y), (0.12, -LEG_Y))   # arms; each screwed to the panel's edge
                                                                            # and its rail, up to the rail's top
LEG_FOOT = (0.150, 0.150, 0.018)  # plywood foot under each leg, clamped to the table
PLUG_R = 0.025                    # 50 mm plywood plug (2 in hole saw) at each screw
HOLE_OPEN = 0.0028                # half width of the panel collider's opening round each shank (4.2 mm), less than the head

# -- the light's bare housing (Commercial Electric WR484080LSL, lens and channel cover off) --------------------------
# housing frame: origin at the middle of the back's top face, x along the length (toward station B when mounted),
# y across, z up; the open side faces down
HOUSING_MASS = 3.7                # estimate: 10.41 lb (4.72 kg) with lens per the spec sheet, lens about 1.0 kg
SHEET_T = 0.0015                  # back plate collider (sheet steel about 0.6 mm; thicker for contact robustness)
WALL_T = 0.003                    # walls, collider
TRIM_T, TRIM_H, TRIM_L = 0.008, 0.024, 0.080   # push-on rubber edge trim at the four grip spots (estimate)
GRIP_IN = 0.080                   # grip spots: on each long wall's lower edge, 80 mm in from each end

# keyholes (research.md Q2: maker's drawings and a product photo of the back): four, about 32.5 in (826 mm) apart
# lengthwise and 7.5 in (190 mm) across, each a round centre hole with a narrow slot running both ways along the length.
# Sizes: estimate; the real slots decide.
KEY_X = 0.413                     # centre holes at x +-413 mm (about 196 mm in from each end)
KEY_Y = 0.095                     # and y +-95 mm
KEY_HOLE_D = 0.0095               # round centre hole, 3/8 in (drawing: about 0.33 in)
KEY_SLOT_W = 0.0048               # narrow slot, 3/16 in (drawing: about 0.17 in)
KEY_SLOT_HALF = 0.0125            # the slot runs 12.5 mm each way from the centre (about 1 in overall)
KEYS = tuple((sx * KEY_X, sy * KEY_Y) for sx in (-1, 1) for sy in (1, -1))   # A-end pair first (x < 0)

# -- where the light goes: its pencil outline, the screws, and where it starts ---------------------------------------
# Mounted (inside its outline) the light's centre is at the world origin. The screws were driven where the B-side narrow
# ends of the keyholes are then (10 mm toward B from each centre hole), so the light is hung by bringing the centre
# holes under the heads 10 mm toward B of the outline, raising it, and sliding it 10 mm toward A, into its outline.
KEY_LOCK = 0.010                  # centre hole to the screw, along the length, when mounted
# each screw, named by the station and the arm on its side (B faces -x, so B's left is world -y), and its keyhole
SCREW_KEY = {"a_left": 0, "a_right": 1, "b_left": 3, "b_right": 2}
SCREWS = {n: (KEYS[k][0] + KEY_LOCK, KEYS[k][1]) for n, k in SCREW_KEY.items()}   # world xy
STATION_SCREWS = {"a": ("a_left", "a_right"), "b": ("b_left", "b_right")}
START_SHIFT = 0.015               # on its stands the light lies 15 mm toward B of its outline
START_DROP = 0.030                # and its back 30 mm below the ceiling

# -- stands and screwdriver holders -----------------------------------------------------------------------------------
STAND_X = (-0.30, 0.30)           # two stands under the light, crossbars across it
STAND_POST = (0.038, 0.089)       # 2x4 post, its wide face along y
CROSSBAR = (0.089, 0.400, 0.038)  # 2x4 laid flat, across the light, with 10 mm foam on top
FOAM_T = 0.010
# one screwdriver per arm, on a holder outboard of the arm (station frame); the hand comes down over it with its
# approach level and turned HOLDER_YAW from straight ahead (outward), the peg turned square to that
HOLDER_LOCAL = {"left": (0.120, 0.450), "right": (0.120, -0.450)}
HOLDER_YAW = {"left": 60.0, "right": -60.0}
HOLDER_FLOOR = 0.350              # the driver's bottom (the peg's top), above the table
PEG_W, PEG_H = 0.022, 0.030
RISER = 0.089                     # 4x4 riser under each peg

# -- screwdriver (Bosch GO class, push to start) ------------------------------------------------------------------------
DRV_R = 0.018                     # grip, 36 mm across (estimate)
DRV_BODY_L = 0.150
DRV_COLLAR_R, DRV_COLLAR_L = 0.010, 0.020
DRV_BIT_HALF, DRV_BIT_L = 0.003, 0.025      # #2 square bit
DRV_TIP = DRV_BODY_L + DRV_COLLAR_L + DRV_BIT_L             # bottom to bit tip, 195 mm
DRV_GRIP = 0.030                  # the hands grip it 30 mm above its bottom, 165 mm below the bit tip
DRV_MASS = 0.31                   # Bosch GO data sheet, with battery
RPM = 360.0
CLUTCH = 0.6                      # N m, lowest clutch setting (estimate; Bosch publishes only 2.5 / 5 N m maxima)
PUSH_START = 3.0                  # N, push that switches the motor on (estimate)

# -- screws (#8 x 3/4 in pan-head square-drive wood screw), driven into the ceiling before the trial ------------------
SCREW_L = 0.01905                 # under the head
SHANK_R = 0.0021                  # 0.164 in basic diameter (B18.6.1)
HEAD_R, HEAD_H = 0.0040, 0.0028   # pan head 0.306-0.322 in across, 0.105-0.115 in high (B18.6.1)
SCREW_MASS = 0.003
HEAD_FRICTION = (0.3, 0.25)       # zinc-plated head on the painted steel back (estimate)
PITCH = 0.0254 / 15               # 15 threads per inch (B18.6.1 #8): 1.69 mm per turn
GAP = 0.0095                      # the maker's 3/8 in gap between the screw head and the ceiling, set at reset
PZ0 = SCREW_L - GAP               # the screw's tip above the ceiling as set: 9.55 mm into the wood
TRAVEL = SCREW_L - SHEET_T        # tip height when the head clamps the back flush (head on the 1.5 mm collider)
THREAD_DRAG = 0.25                # N m, running torque in the plywood (estimate)
THREAD_EFF = 0.115                # thread efficiency, so the clutch slips near 150 N of clamp (fidelity note)
THREAD_K, THREAD_C = 4.0e4, 40.0  # the thread's coupling drive (N/m, N s/m)
BIT_K = 1.0e4                     # the bit's contact with the head, a spring in task code (N/m; estimate)
BIT_D = 40.0                      # N s/m, its damping on the driver
PUSH_MAX = 60.0                   # N, the most that spring passes
RECESS_K = 3.0e3                  # N/m, the recess centring a pushed bit (at most the push; estimate)

# -- scoring -------------------------------------------------------------------------------------------------------------
SEAT_GAP = 0.0003                 # tight: head within 0.3 mm of the housing's back
SEAT_F = 50.0                     # and clamping it with 50 N or more
FLUSH_GAP = 0.002                 # and a 2 mm shim does not go in at the nearest long edge, in line with the screw
HANG_S = 3.0                      # no robot touching it for the last 3 s
TOUCH_F = 0.05                    # N: a contact that counts as touching
HOLD_OPEN = 0.25                  # a gripper holds the end it touches while it is closed on it (opened no more than
HOLD_SLIP = 0.002                 # this fraction, about 23 mm between the pads) and the light moves with it: over the
HOLD_S = 0.25                     # last HOLD_S (s) it touched the light and its place in the light's frame moved less
                                  # than HOLD_SLIP (m). Open fingers the light rests on or slides along do not hold it.
ROOM_HALF = 2.0                   # m: the room's walls (a stand-in, task code): a driver is stopped this far out
FALL_LOOKBACK = 0.25              # s: a fall is measured from the end's highest point in this long before it was let go
FALL = 0.010                      # drop: an end comes down 10 mm or more, unheld and not on its screws, onto something

# -- motor ratings (research.md Q1: Damiao tables and manuals; the YAM runs at 24 V; the lower of each version) -------
RATED = (9.0, 9.0, 9.0, 3.0, 3.0, 3.0)        # N m: DM-J4340 (J1-J3) 9 N m; DM-J4310 V1.1 (J4-J6) 3 N m
PEAK = (27.0, 27.0, 27.0, 7.0, 7.0, 7.0)      # N m: 27 / 7 (also yam_arena's effort limits)
TARGET = 0.70                                  # sustained (10 s average) torque target: 70 % of rated


def subtract(rects, cut):
    """Rectangles (x0, x1, y0, y1) minus one rectangle, as non-overlapping rectangles."""
    cx0, cx1, cy0, cy1 = cut
    out = []
    for x0, x1, y0, y1 in rects:
        if cx1 <= x0 or cx0 >= x1 or cy1 <= y0 or cy0 >= y1:
            out.append((x0, x1, y0, y1))
            continue
        if y0 < cy0:
            out.append((x0, x1, y0, cy0))
        if cy1 < y1:
            out.append((x0, x1, cy1, y1))
        ya, yb = max(y0, cy0), min(y1, cy1)
        if x0 < cx0:
            out.append((x0, cx0, ya, yb))
        if cx1 < x1:
            out.append((cx1, x1, ya, yb))
    return out


def keyhole_cuts(cx: float, cy: float) -> list[tuple]:
    """One keyhole around its centre (cx, cy), housing frame: the slot along x and the round hole as its square."""
    h, w = KEY_SLOT_HALF, KEY_SLOT_W / 2
    r = KEY_HOLE_D / 2
    return [(cx - h, cx + h, cy - w, cy + w), (cx - r, cx + r, cy - r, cy + r)]


def rot_z(a: float):
    import numpy as np

    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
