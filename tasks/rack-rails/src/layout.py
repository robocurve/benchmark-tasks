"""Scene layout and geometry of the rack-rails task (station frame: x forward from the arm bases, y to the robot's left,
z up; metres). Engine-independent (numpy only).

A 12U four-post open-frame rack stands on the bench with its depth along y: the near-side REAR post at +y (robot's
left), the near-side FRONT post at -y (robot's right), both with EIA-310 square-hole mounting flanges facing each other
across the rack's mounting depth. A 1U server is already installed on its rails (front and rear posts); the new server's
rail must go in the free U directly below it. The rail lies closed in a foam cradle on the bench in front of the rack.

Rail frames (both axis-aligned with the station when yaw = 0: x across the rack, y along the rail toward the rack's
rear, z up):
* O, outer member (rear end): origin on the rear end bracket's outer face, on the peg axis (x) and at the peg pair's
  mid-height (z), which is the U centre when mounted.
* I, inner member (front end, telescopes out of O toward -y): origin on the front end bracket's outer face (faces -y),
  same x / z convention. Closed: I = O + (0, -L0, 0); extended by e: (0, -L0 - e, 0).
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena TABLE_TOP["station"])


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# ---------------------------------------------------------------- EIA-310 rack (StarTech 4POSTRACK12U class, 12U)
U = 0.04445  # 1U = 1.75 in
HOLE_OFFSETS = (0.00635, 0.022225, 0.0381)  # hole centres above each U's lower boundary (1/4, 7/8, 1 1/2 in)
HOLE = 0.0095  # square hole 0.375 in
N_U = 12
U1_BOTTOM = TOP + 0.012  # lower boundary of U1 (rack on its base frame on the bench, casters off)
HOLE_COL_X = 0.300  # x of the near-side posts' hole column (nominal; per-trial jitter)
HOLE_SPAN_X = 0.4651  # hole column to hole column across the rack (18.31 in)
Y_F = 0.420  # mounting flanges' inner faces at y = +-Y_F (840 mm mounting depth, in the rack's 22-40 in range)
FLANGE_T = 0.002  # flange sheet thickness
FLANGE_HALF_W = 0.016  # flange plate half width about the hole column
LEG_W = 0.045  # post's outer leg (y), runs outward from the flange
OPEN_EDGE = 0.0075  # 19 in opening edge: hole column + 7.5 mm (inside the rack)


def u_bottom(u):
    return U1_BOTTOM + (u - 1) * U


def u_center(u):
    return u_bottom(u) + U / 2


def hole_zs():
    """Hole centre heights of one post, U1..U12 (36 holes)."""
    return [u_bottom(u) + h for u in range(1, N_U + 1) for h in HOLE_OFFSETS]


def u_of_z(z):
    """U number whose centre is nearest to height z, and the offset from that centre (m)."""
    u = int(round((z - u_center(1)) / U)) + 1
    return u, z - u_center(u)


# ---------------------------------------------------------------- the rail (tool-less 1U sliding rail, stab-in pegs)
PEG = 0.0090  # square peg 9.0 mm in the 9.5 mm hole: 0.25 mm a side
PEG_L = 0.008  # protrudes 8 mm from the bracket face
PEG_CH = 0.0015  # tip taper: 1.5 mm a side over the last PEG_CH_L (rounded-nose stab-in peg as a frustum)
PEG_CH_L = 0.0030
PEG_DZ = 0.015875  # pegs at +-5/8 in: the U's top and bottom holes (31.75 mm apart only inside one U)
BRACKET_T = 0.003
BRACKET_X = (-0.014, 0.016)  # end bracket across the flange (peg axis at 0)
BRACKET_Z = 0.021  # +-21 mm (inside one U)
OUTER_X = (0.010, 0.024)  # outer member profile (14 x 40 mm), inside the rack opening
OUTER_Z = 0.020
OUTER_LEN = 0.620
INNER_X = (0.012, 0.022)  # inner (telescoping) member profile (10 x 30 mm), nested in the outer
INNER_Z = 0.015
INNER_LEN = 0.500
L0 = 0.800  # closed length, bracket face to bracket face
E_MAX = 0.080  # telescoping travel
OUTER_MASS = 0.90
INNER_MASS = 0.55
RAIL_FRICTION = 0.5  # zinc-plated steel on the gripper pads / on painted steel (estimate)
HARD_FRICTION = 0.4
SLIDE_FRICTION_N = 4.0  # telescoping slide drag (ball-bearing slide, estimate)
SLIDE_VISC = 150.0  # N per m/s below saturation (computed friction; see rackkit)
# latch (spring catch at each end): the bracket's last LATCH_ZONE of travel to the flange needs a push of more than
# LATCH_PUSH_N; at SEAT_GAP the catch drops in; afterwards the end cannot back out (retention up to LATCH_HOLD_N)
LATCH_ZONE = 0.0025
LATCH_PUSH_N = 10.0
SEAT_GAP = 0.0004
LATCH_TILT_DEG = 1.5
LATCH_ALIGN_M = 0.0012  # peg axis within this of its hole pair's axis (x and z) while in the zone
LATCH_K = 20000.0
LATCH_C = 60.0
LATCH_HOLD_N = 150.0

# ---------------------------------------------------------------- cradle (foam box insert) on the bench
CRADLE_X = 0.190  # rail peg-axis x at rest (nominal)
CRADLE_SLOT_Z = TOP + 0.025  # slot floor: the outer member's bottom rests here
CRADLE_SLOT_W = 0.018
CRADLE_BLOCK = (0.070, 0.040, 0.035)
CRADLE_AT = (-0.220, -0.580)  # cradle blocks along the rail (O-frame y)

# ---------------------------------------------------------------- installed 1U server (on its own rails) below the target U
SERVER_H = 0.0434  # 1U chassis height (1.75 in - 1/32 in)
SERVER_DEPTH = 0.760  # chassis depth from the front flange
SERVER_X = (0.026, HOLE_SPAN_X - 0.026)  # chassis between its two rails (hole column = 0)
SERVER_EAR_T = 0.003

# ---------------------------------------------------------------- start states
TARGET_US = (2, 3, 4)  # free U directly above the installed server (reach of the arms over the rack's bottom third)
# named setups: (target U, rack dx, rack dy, rail dx, rail dy, rail yaw deg)
SETUPS = (
    (3, 0.0, 0.0, 0.0, 0.0, 0.0),
    (2, 0.005, -0.004, 0.008, -0.010, 2.0),
    (4, -0.006, 0.005, -0.007, 0.009, -2.5),
    (2, -0.004, 0.003, -0.005, 0.011, 1.5),
    (4, 0.006, -0.005, 0.009, -0.008, -2.0),
)
RACK_DX_MIN = 0.004
JITTER = dict(rack_dx=0.008, rack_dy=0.006, rail_dx=0.010, rail_dy=0.012, rail_yaw=3.0)


def setup_spec(setup):
    """Start state of a setup: int = named setup, ('seed', s) = drawn from the trial seed."""
    if isinstance(setup, (tuple, list)) and len(setup) == 2 and setup[0] == "seed":
        rng = np.random.default_rng(int(setup[1]))
        u = int(rng.choice(TARGET_US))
        j = JITTER
        vals = [
            float(rng.uniform(-j[k], j[k]))
            for k in ("rack_dx", "rack_dy", "rail_dx", "rail_dy", "rail_yaw")
        ]
        # exclusion band: the rack never sits within 4 mm (x) of its nominal place, so the peg noses (1.5 mm taper) of an
        # open-loop replay recorded at a nominal start cannot find the holes (draw order unchanged)
        if abs(vals[0]) < RACK_DX_MIN:
            vals[0] = RACK_DX_MIN if vals[0] >= 0 else -RACK_DX_MIN
        spec = (u, *vals)
    else:
        spec = SETUPS[int(setup) % len(SETUPS)]
    u, rdx, rdy, ldx, ldy, yaw = spec
    return dict(
        target_u=int(u),
        server_u=int(u) - 1,
        rack_dx=rdx,
        rack_dy=rdy,
        rail_dx=ldx,
        rail_dy=ldy,
        rail_yaw=yaw,
    )


def rack_origin(spec):
    """Rack reference point: (hole column x, depth centre y, 0)."""
    return np.array([HOLE_COL_X + spec["rack_dx"], spec["rack_dy"], 0.0])


def rail_rest_pose(spec):
    """O-frame pose of the closed rail in its cradle."""
    yaw = spec["rail_yaw"]
    r = rz(yaw)
    centre = np.array(
        [CRADLE_X + spec["rail_dx"], spec["rail_dy"], CRADLE_SLOT_Z + OUTER_Z]
    )
    # rail centre (between the bracket faces) at `centre`; O sits L0/2 toward +y along the rail
    return centre + r @ np.array([0.0, L0 / 2, 0.0]), r


def inner_from_outer(po, ro, e=0.0):
    return po + ro @ np.array([0.0, -(L0 + e), 0.0]), ro


def mounted_outer_pose(spec, u=None):
    """O-frame pose with the rear end seated on the rear flange at U u (default: the target)."""
    o = rack_origin(spec)
    u = spec["target_u"] if u is None else u
    return np.array([o[0], o[1] + Y_F, u_center(u)]), np.eye(3)


def flange_face_y(spec, end):
    """y of the inner face of the near-side flange at the 'rear' (+y) or 'front' (-y) end."""
    o = rack_origin(spec)
    return o[1] + (Y_F if end == "rear" else -Y_F)


def outward(end):
    """Unit vector from the rack interior toward the post for that end (the peg direction when mounted)."""
    return np.array([0.0, 1.0 if end == "rear" else -1.0, 0.0])


# ---------------------------------------------------------------- end geometry used by the latch rule and the scorer
def end_state(end, pos, rot, spec):
    """Mounting state of one rail end from its member's frame (O for 'rear', I for 'front').

    gap: distance from the bracket's outer face to the flange's inner face along the post normal (m; < 0 = pressed in);
    axis_dx / axis_dz: peg-pair axis offset from the hole column (x) and from the nearest U centre (z);
    u: U number of that nearest centre; tilt_deg: angle between the peg direction and the post normal;
    in_holes: the pegs have passed into the flange (gap below PEG_L - 1 mm) with the axis inside the holes' play."""
    n = outward(end)
    a = rot[:, 1] * (1.0 if end == "rear" else -1.0)  # peg direction of this end
    gap = float(n[1] * (flange_face_y(spec, end) - pos[1]))
    u, dz = u_of_z(float(pos[2]))
    dx = float(pos[0] - rack_origin(spec)[0])
    tilt = math.degrees(math.acos(float(np.clip(a @ n, -1.0, 1.0))))
    in_holes = gap < PEG_L - 0.001 and abs(dx) < 0.002 and abs(dz) < 0.002
    return dict(gap=gap, axis_dx=dx, axis_dz=dz, u=u, tilt_deg=tilt, in_holes=in_holes)


# ---------------------------------------------------------------- shapes (convex pieces, local frames)
def box_corners(x0, x1, y0, y1, z0, z1):
    return np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)])


def peg_points(sign, dz):
    """Square peg on a bracket face at y = 0 pointing toward sign*y, centred at (0, dz), with a chamfered tip."""
    h, hc = PEG / 2, PEG / 2 - PEG_CH
    pts = []
    for y, hh in ((0.0, h), (sign * (PEG_L - PEG_CH_L), h), (sign * PEG_L, hc)):
        pts += [[sx * hh, y, dz + sz * hh] for sx in (-1, 1) for sz in (-1, 1)]
    return np.array(pts)


def flange_pieces(sign):
    """Square-hole flange of one near-side post as boxes in the rack frame (x from the hole column, y from the rack
    depth centre); sign +1 = rear post (+y), -1 = front post."""
    y0, y1 = sorted((sign * Y_F, sign * (Y_F + FLANGE_T)))
    h = HOLE / 2
    z_lo, z_hi = TOP, u_bottom(N_U + 1) + 0.010
    pieces = [
        box_corners(-FLANGE_HALF_W, -h, y0, y1, z_lo, z_hi),
        box_corners(h, FLANGE_HALF_W, y0, y1, z_lo, z_hi),
    ]
    zs = [z_lo] + [v for z in hole_zs() for v in (z - h, z + h)] + [z_hi]
    for k in range(0, len(zs), 2):
        pieces.append(box_corners(-h, h, y0, y1, zs[k], zs[k + 1]))
    return pieces


# 7-segment digit strokes (unit cell 0..1 x 0..2): a b c d e f g
_SEG = {
    "a": ((0, 2), (1, 2)),
    "b": ((1, 1), (1, 2)),
    "c": ((1, 0), (1, 1)),
    "d": ((0, 0), (1, 0)),
    "e": ((0, 0), (0, 1)),
    "f": ((0, 1), (0, 2)),
    "g": ((0, 1), (1, 1)),
}
_DIGITS = {
    "0": "abcdef",
    "1": "bc",
    "2": "abged",
    "3": "abgcd",
    "4": "fgbc",
    "5": "afgcd",
    "6": "afgedc",
    "7": "abc",
    "8": "abcdefg",
    "9": "abcdfg",
}


def digit_strokes(text, height=0.010):
    """Line segments ((u0, v0), (u1, v1)) of a 7-segment number, text width along u, height along v, origin at the
    centre."""
    w = height / 2
    gap = height * 0.35
    total = len(text) * w + (len(text) - 1) * gap
    out = []
    for i, ch in enumerate(text):
        u0 = -total / 2 + i * (w + gap)
        for s in _DIGITS[ch]:
            (a0, b0), (a1, b1) = _SEG[s]
            out.append(
                (
                    (u0 + a0 * w, -height / 2 + b0 * w),
                    (u0 + a1 * w, -height / 2 + b1 * w),
                )
            )
    return out
