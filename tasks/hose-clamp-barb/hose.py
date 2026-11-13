"""Geometry and measurements for fitting a coolant hose onto a pump's barbed outlet and clamping it.

On the bench: a coolant pump bolted down with its 12 mm barbed outlet pointing at the right arm, a
length of rubber hose lying in front of it with a worm-drive clamp run down loosely 12 mm from its
end, and an 8 mm nut driver standing in a holder. The hose has to be pushed over the barb's bead
right up to the outlet boss, which puts the band on the barb's shank behind the bead, and the
clamp's screw driven tight.
Frame: metres, x away from the arm bases, y left, z up; TOP is the bench.
"""
from __future__ import annotations

import math

import numpy as np

from simkit import Hinge, Part, Piece, Spring

TOP = 0.7475

RUBBER = (0.07, 0.07, 0.08)
PUMP_COL = (0.18, 0.32, 0.55)        # painted pump casting
BRASS = (0.78, 0.62, 0.30)
STEEL = (0.72, 0.73, 0.75)
MARK = (0.92, 0.92, 0.88)

# -- pump and its barbed outlet (fixed) -------------------------------------------------------
AXIS_X = 0.270                       # outlet axis, x
AXIS_Z = TOP + 0.060                 # outlet axis height
PUMP = (0.110, 0.100, 0.110)         # casting: x by y by z
PUMP_C = (AXIS_X + 0.010, 0.085)     # its centre, x y; its face toward the hose at y = 0.035
FACE_Y = PUMP_C[1] - PUMP[1] / 2
BOSS = (0.013, 0.010)                # outlet boss: radius, length (from the face toward -y)
BOSS_Y = FACE_Y - BOSS[1]            # where a hose pushed fully home stops
SHANK_R = 0.0056                     # barb shank under the hose (12 mm barb, 11.2 mm shank): as drawn
SHANK_COLLIDE_R = 0.0048             # its collider: rubber gives where a rigid bore would bind, so the rigid stand-in has play
BEAD_R = 0.0068                      # the bead the hose has to be forced over
BARB_L = 0.034                       # boss face to barb tip
TIP_Y = BOSS_Y - BARB_L
BEAD_Y = TIP_Y + 0.010               # the bead's crest, 10 mm in from the tip

# -- hose (free) ------------------------------------------------------------------------------
HOSE_RI = 0.0061                     # 12 mm bore
HOSE_RO = 0.0095                     # 19 mm outside
HOSE_L = 0.200
HOSE_MASS = 0.055
HOSE_REST = (AXIS_X, -0.120)         # mouth end, x y, lying on the bench pointing at the outlet
WALLS = 8
INSERT = BARB_L                      # a hose pushed fully home covers the whole barb
MARK_AT = INSERT                     # painted insertion mark this far back from the mouth: level with the barb's tip when home


def hose_centre_rest():
    return np.array([HOSE_REST[0], HOSE_REST[1] - HOSE_L / 2, TOP + HOSE_RO])


def hose_object() -> Part:
    """An octagonal rubber tube along y (mouth at +y) built from wall staves, so the barb fits its bore."""
    c = hose_centre_rest()
    rm = (HOSE_RI + HOSE_RO) / 2
    w = 2 * HOSE_RO * math.tan(math.pi / WALLS) + 0.0004
    t = HOSE_RO - HOSE_RI
    # root: the top stave, at pos; the others are placed relative to it
    pos = c + np.array([0.0, 0.0, rm])
    staves = []
    for k in range(1, WALLS):
        a = 2 * math.pi * k / WALLS
        off = np.array([rm * math.sin(a), 0.0, rm * math.cos(a)]) - np.array([0.0, 0.0, rm])
        staves.append(Piece("box", (w, HOSE_L, t), tuple(off), rot=(0.0, a, 0.0)))
    skin = Piece("cylinder", (HOSE_RO + 0.0002, HOSE_L, HOSE_RO + 0.0002), (0.0, 0.0, -rm), axis="Y", collide=False)
    mark = Piece("cylinder", (HOSE_RO + 0.0005, 0.003, HOSE_RO + 0.0005), (0.0, HOSE_L / 2 - MARK_AT, -rm), axis="Y",
                 color=MARK, collide=False)
    return Part("hose", pos=tuple(float(v) for v in pos), size=(w, HOSE_L, t), exact_z=True, mass=HOSE_MASS,
                color=RUBBER, friction=(1.1, 0.9), restitution=0.0, contact_offset=0.0015,
                com=(0.0, 0.0, -rm), pieces=(*staves, skin, mark),
                spring=Spring(axis="Y", lower=-0.010, upper=INSERT + 0.001, stiffness=0.0, damping=GRIP_DAMPING,
                              target=0.0, anchor=(AXIS_X, TIP_Y, AXIS_Z), local=(0.0, HOSE_L / 2, -rm), enabled=False))


def hose_axis(pose):
    """Mouth centre and unit axis (pointing out of the mouth) of the hose from its root pose."""
    p = np.asarray(pose, float)
    r = quat_matrix(p[3:7])
    rm = (HOSE_RI + HOSE_RO) / 2
    centre = p[:3] + r @ np.array([0.0, 0.0, -rm])
    axis = r @ np.array([0.0, 1.0, 0.0])
    return centre + axis * HOSE_L / 2, axis


# -- worm-drive clamp (free, sliding on the hose) -----------------------------------------------
BAND_W = 0.009
BAND_T = 0.0007
BAND_R = HOSE_RO + 0.0012            # opened just enough to slide along the hose
CLAMP_AT = 0.012                     # band centre back from the hose mouth: slipped on loosely near the end, ready
HOUSING = (0.009, 0.011, 0.013)      # the worm housing, on the band's arm side (-x): x by y by z
CLAMP_MASS = 0.008
SCREW_R = 0.0040                     # 8 mm (5/16 in) hex head, drawn and collided as a round head
SCREW_H = 0.010                      # hex head, tall enough for the socket to sit well down on it
TURNS_SNUG = 0.4                     # screw turns from as found (run down by hand till the band touches) to snug
TURNS_TIGHT = 0.8                    # ... and to tight (about 3 N m on a real clamp): more than one wrist stroke
BAND_DRAWN_TIGHT = 0.25              # turns at which the band is drawn pulled down onto the hose
BAND_GRIPS = 0.5                     # turns from which the band grips the hose on the barb firmly (held there)
SNUG_TORQUE = 0.0                    # no reaction torque is put on the driver (kept for the record; a light rigid tool spins out under one)
TIGHT_TORQUE = 0.0
CLAMP_TARGET = 0.012                 # demo: band centre this far back from the mouth
SHANK_BAND = (0.0065, 0.0175)        # band centre from the mouth for the 9 mm band to lie wholly on the shank, behind the bead (22 mm on)

# rubber on the barb: once the bore is over the bead the hose grips it -- a slide joint along the barb,
# switched on then, with a damping drive standing in for the rubber's drag
BEAD_ENGAGE = 0.006                  # depth (from the barb tip) at which the hose's bore meets the bead
GRIP_DAMPING = 250.0                 # N s/m: about 3 N to keep it sliding at 12 mm/s
CLAMPED_K = 20000.0                  # N/m holding it where it is once the clamp is tight
SEATED_K = 3000.0                    # N/m once pushed home: the rubber's grip on the barb's whole length (~30 N per 10 mm)
PULL_OFF_DEPTH = 0.004               # pulled back out this far, it lets go of the barb


BAND_PIECES = 16                     # the clamp's pieces: loose band 0..15, the band drawn tight 16..31


def clamp_pos_rest():
    c = hose_centre_rest()
    return np.array([c[0], HOSE_REST[1] - CLAMP_AT, c[2]])


def housing_offset():
    return np.array([-(BAND_R + HOUSING[0] / 2), 0.0, 0.0])


def clamp_object() -> Part:
    """A stainless band round the hose with its worm housing on the arm side; slides along the hose."""
    pos = clamp_pos_rest()
    band = []
    n = 16
    w = 2 * BAND_R * math.tan(math.pi / n) + 0.0003
    for k in range(n):
        a = 2 * math.pi * k / n
        band.append(Piece("box", (w, BAND_W, BAND_T), (BAND_R * math.sin(a), 0.0, BAND_R * math.cos(a)),
                          rot=(0.0, a, 0.0), color=STEEL, collide=False))
    tight_r = HOSE_RO + 0.0004
    wt = 2 * tight_r * math.tan(math.pi / n) + 0.0003
    for k in range(n):
        a = 2 * math.pi * k / n
        band.append(Piece("box", (wt, BAND_W, BAND_T), (tight_r * math.sin(a), 0.0, tight_r * math.cos(a)),
                          rot=(0.0, a, 0.0), color=(0.85, 0.86, 0.88), collide=False))
    hose_pos = hose_object().pos
    return Part("clamp", pos=tuple(float(v) for v in pos + housing_offset()), size=HOUSING, exact_z=True,
                mass=CLAMP_MASS, color=STEEL, friction=(0.8, 0.6), restitution=0.0, contact_offset=0.001,
                com=tuple(float(v) for v in -housing_offset()),
                pieces=tuple(replace_offset(p, np.asarray(p.offset) - housing_offset()) for p in band),
                spring=Spring(axis="Y", lower=-0.12, upper=CLAMP_AT - 0.003, stiffness=2000.0, damping=2.0, target=0.0,
                              parent="hose", parent_pos=tuple(float(v) for v in hose_pos)))


def replace_offset(piece, off):
    from dataclasses import replace
    return replace(piece, offset=tuple(float(v) for v in off))


def screw_pos_rest():
    return clamp_pos_rest() + housing_offset() + np.array([0.0, 0.0, HOUSING[2] / 2 + SCREW_H / 2])


def screw_object() -> Part:
    """The worm screw's hex head on top of the housing: turns about z when driven."""
    pos = screw_pos_rest()
    slot = Piece("box", (SCREW_R * 2.6, 0.0018, 0.0010), (0.0, 0.0, SCREW_H / 2 + 0.0003), color=(0.95, 0.80, 0.10),
                 collide=False)                                          # a painted mark, so its turning shows
    return Part("screw", pos=tuple(float(v) for v in pos), size=(SCREW_R, SCREW_H, SCREW_R), shape="cylinder",
                axis="Z", exact_z=True, mass=0.002, color=STEEL, friction=(0.8, 0.6), restitution=0.0,
                contact_offset=0.001, pieces=(slot,),
                hinge=Hinge(pivot=tuple(float(v) for v in pos), axis="Z", damping=0.0005, lower_deg=-1e6,
                            upper_deg=1e6, stiffness=0.02, target_deg=0.0, parent="clamp",
                            parent_pos=tuple(float(v) for v in clamp_pos_rest() + housing_offset())))


# -- stubby 8 mm nut driver in its holder -------------------------------------------------------------------
HOLDER_XY = (0.200, 0.200)
HOLDER = (0.040, 0.040, 0.040)
HOLE = 0.016
DRIVER_HANDLE = (0.015, 0.048)       # radius, length: a stubby nut driver
DRIVER_SHAFT = (0.0045, 0.020)
SOCKET = (0.0070, 0.016)             # 8 mm (5/16 in) socket's outside radius, length
SOCKET_BORE = (0.0046, 0.009)        # its bore (9.2 mm, 0.6 mm play on the head): slips 9 mm down over it, so the driver stands on it
DRIVER_MASS = 0.090
DRIVER_LIFT = 0.004                  # socket end above the holder floor


def holder_object() -> Part:
    x, y = HOLDER_XY
    lx, ly, h = HOLDER
    t = (lx - HOLE) / 2
    walls = (Piece("box", (lx, t, h), (0.0, (HOLE + t) / 2, 0.0)),
             Piece("box", (lx, t, h), (0.0, -(HOLE + t) / 2, 0.0)),
             Piece("box", (t, HOLE, h), ((HOLE + t) / 2, 0.0, 0.0)),
             Piece("box", (t, HOLE, h), (-(HOLE + t) / 2, 0.0, 0.0)))
    return Part("holder", pos=(x, y, TOP + 0.002), size=(lx, ly, 0.004), exact_z=True, color=(0.25, 0.25, 0.27),
                friction=(0.6, 0.5), pieces=tuple(replace_offset(p, (p.offset[0], p.offset[1], h / 2 - 0.002)) for p in walls))


def driver_tip_rest():
    return np.array([HOLDER_XY[0], HOLDER_XY[1], TOP + 0.004 + DRIVER_LIFT])


def driver_object() -> Part:
    """Nut driver standing socket-down in the holder: root is the shaft; socket below, handle above."""
    tip = driver_tip_rest()
    shaft_c = tip + np.array([0.0, 0.0, SOCKET[1] + DRIVER_SHAFT[1] / 2])
    base_z = -DRIVER_SHAFT[1] / 2
    solid = SOCKET[1] - SOCKET_BORE[1]
    socket = [Piece("cylinder", (SOCKET[0], solid, SOCKET[0]), (0.0, 0.0, base_z - solid / 2), color=STEEL)]
    n = 8
    rm = (SOCKET_BORE[0] + SOCKET[0]) / 2
    w = 2 * SOCKET[0] * math.tan(math.pi / n) + 0.0003
    for k in range(n):
        a = 2 * math.pi * k / n
        socket.append(Piece("box", (w, SOCKET[0] - SOCKET_BORE[0], SOCKET_BORE[1]),
                            (rm * math.sin(a), rm * math.cos(a), base_z - solid - SOCKET_BORE[1] / 2),
                            rot=(0.0, 0.0, -a), color=STEEL))
    # a flat-sided (square-section) grip, so the jaws bear on faces and it cannot lean in the hand
    handle = Piece("box", (2 * DRIVER_HANDLE[0], 2 * DRIVER_HANDLE[0], DRIVER_HANDLE[1]),
                   (0.0, 0.0, DRIVER_SHAFT[1] / 2 + DRIVER_HANDLE[1] / 2), color=(0.85, 0.65, 0.10),
                   friction=(1.6, 1.4))
    band = Piece("box", (2 * DRIVER_HANDLE[0] + 0.0008, 2 * DRIVER_HANDLE[0] + 0.0008, 0.016),
                 (0.0, 0.0, DRIVER_SHAFT[1] / 2 + 0.014), color=(0.10, 0.10, 0.11), collide=False)
    return Part("driver", pos=tuple(float(v) for v in shaft_c), size=(DRIVER_SHAFT[0], DRIVER_SHAFT[1], DRIVER_SHAFT[0]),
                shape="cylinder", axis="Z", exact_z=True, mass=DRIVER_MASS, color=STEEL, friction=(0.6, 0.5),
                restitution=0.0, contact_offset=0.0015, max_depenetration=0.2,
                com=(0.0, 0.0, -DRIVER_SHAFT[1] / 2), pieces=(*socket, handle, band))   # steel shaft and socket outweigh the grip


def pump_object() -> Part:
    """Pump casting with its outlet boss and brass barb pointing -y, toward the hose."""
    cx, cy = PUMP_C
    pc = np.array([cx, cy, TOP + PUMP[2] / 2])

    def at(x, y, z):
        return tuple(float(v) for v in np.array([x, y, z]) - pc)

    shank_l = BOSS_Y - (BEAD_Y + 0.002)
    pieces = (
        Piece("cylinder", (BOSS[0], BOSS[1], BOSS[0]), at(AXIS_X, FACE_Y - BOSS[1] / 2, AXIS_Z), axis="Y", color=PUMP_COL),
        Piece("cylinder", (SHANK_COLLIDE_R, shank_l, SHANK_COLLIDE_R), at(AXIS_X, BOSS_Y - shank_l / 2, AXIS_Z), axis="Y",
              color=BRASS),
        Piece("cylinder", (SHANK_COLLIDE_R, BEAD_Y + 0.002 - (TIP_Y + 0.010), SHANK_COLLIDE_R),
              at(AXIS_X, (BEAD_Y + 0.002 + TIP_Y + 0.010) / 2, AXIS_Z), axis="Y", color=BRASS),
        Piece("cone", (SHANK_COLLIDE_R, 0.010, SHANK_COLLIDE_R), at(AXIS_X, TIP_Y + 0.005, AXIS_Z), axis="Y", rot=(math.pi, 0.0, 0.0),
              color=BRASS),                                                   # tapered lead-in, apex toward the hose
        Piece("box", (0.150, 0.130, 0.012), (0.0, 0.010, -PUMP[2] / 2 + 0.006), color=(0.30, 0.30, 0.32)),  # base plate
        Piece("cylinder", (0.040, 0.050, 0.040), (0.0, 0.065, 0.0), axis="Y", color=PUMP_COL),             # motor end
    )
    return Part("pump", pos=tuple(float(v) for v in pc), size=PUMP, exact_z=True, color=PUMP_COL, friction=(0.6, 0.5),
                pieces=pieces)


def barb_trim_object() -> Part:
    """The barb as drawn -- 11.2 mm shank and its bead -- with no collider at all (see SHANK_COLLIDE_R)."""
    shank_l = BOSS_Y - (BEAD_Y + 0.002)
    pos = (AXIS_X, BOSS_Y - shank_l / 2, AXIS_Z)
    bead = Piece("cylinder", (BEAD_R, 0.004, BEAD_R), (0.0, BEAD_Y - pos[1], 0.0), axis="Y", color=BRASS, collide=False)
    return Part("barb_trim", pos=pos, size=(SHANK_R, shank_l, SHANK_R), shape="cylinder", axis="Y", exact_z=True,
                color=BRASS, collision="none", pieces=(bead,))


# -- measurements ----------------------------------------------------------------------------

def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tilt_deg(q, local=(0.0, 0.0, 1.0)):
    v = quat_matrix(q) @ np.asarray(local)
    return math.degrees(math.acos(np.clip(v[2], -1, 1)))


def on_barb(hose_pose):
    """How far the hose is pushed onto the barb (m, along the barb from its tip), and whether it is
    lined up with it (mouth centred on the axis, hose axis along it). 0 when not on it."""
    mouth, axis = hose_axis(hose_pose)
    off = math.hypot(mouth[0] - AXIS_X, mouth[2] - AXIS_Z)
    ang = math.degrees(math.acos(np.clip(axis[1], -1, 1)))
    depth = mouth[1] - TIP_Y
    lined = off < 0.003 and ang < 12
    return (float(depth) if lined and depth > 0 else 0.0), lined


def clamp_back_from_mouth(hose_pose, clamp_pose):
    """Band centre's distance back from the hose mouth, along the hose (m)."""
    mouth, axis = hose_axis(hose_pose)
    band = np.asarray(clamp_pose, float)[:3] + quat_matrix(np.asarray(clamp_pose)[3:7]) @ (-housing_offset())
    return float(np.dot(mouth - band, axis))


def near_hose(hose_pose, tcp, reach=0.035):
    """A hand's TCP within reach of the hose's axis, between its ends."""
    mouth, axis = hose_axis(hose_pose)
    rel = np.asarray(tcp) - mouth
    along = float(np.dot(rel, axis))
    return bool(-HOSE_L <= along <= 0.0 and np.linalg.norm(rel - along * axis) < reach)


def clamp_on_shank(back):
    return bool(SHANK_BAND[0] <= back <= SHANK_BAND[1])


def driver_tip(pose):
    """Bottom centre of the driver's socket, and its axis (unit, toward the handle)."""
    p = np.asarray(pose, float)
    r = quat_matrix(p[3:7])
    axis = r @ np.array([0.0, 0.0, 1.0])
    return p[:3] - axis * (DRIVER_SHAFT[1] / 2 + SOCKET[1]), axis
