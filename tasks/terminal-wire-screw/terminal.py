"""Geometry and measurements for landing a stripped conductor in a screw-clamp rail terminal.

On the bench: a control-panel mounting plate laid flat, as panels are wired on the bench, with a DIN
rail of five 35 mm2 screw-clamp terminal blocks across it. Four already have their wires landed; the
middle one is open, and the panel's incoming 35 mm2 conductor comes up out of the cable duct beside
it, stripped 18 mm, springing back off its line. A screwdriver stands in a holder. The bare end has
to go all the way into the middle block's clamp (insulation up to the block, no bare copper showing)
and be held there while the clamp screw is driven down tight on it.
Frame: metres, x away from the arm bases, y left, z up; TOP is the bench. The rail runs along x; the
conductors come into the blocks along +y, from the duct on the right.
"""
from __future__ import annotations

import math

import numpy as np

from simkit import Hinge, Part, Piece, Spring

TOP = 0.7475

PLATE_COL = (0.80, 0.78, 0.70)       # galvanised mounting plate
RAIL_COL = (0.75, 0.76, 0.78)
BLOCK_COL = (0.55, 0.57, 0.60)       # grey polyamide terminal blocks
STEEL = (0.72, 0.73, 0.75)
COPPER = (0.80, 0.45, 0.25)
BLACK_INS = (0.10, 0.10, 0.11)       # the incoming conductor's insulation
BROWN_INS = (0.42, 0.24, 0.12)
DUCT_COL = (0.45, 0.47, 0.50)

# -- mounting plate, rail and terminal blocks (fixed) -------------------------------------------
PLATE = (0.30, 0.44, 0.003)
PLATE_C = (0.23, -0.08)
BATTEN = 0.040                       # the panel lies on two timber battens, as panels are wired on the bench
PLATE_TOP = TOP + BATTEN + PLATE[2]
RAIL = (0.30, 0.035, 0.0075)         # TS35 top-hat rail along x
RAIL_C = (0.23, 0.030)
RAIL_TOP = PLATE_TOP + RAIL[2]
BLOCK = (0.0152, 0.060, 0.052)       # one 35 mm2 block: x wide, y deep, z tall above the rail
PITCH = BLOCK[0]
TARGET_X = 0.230                     # the open block, middle of five
FACE_Y = RAIL_C[1] - BLOCK[1] / 2    # its wire-entry face, toward the duct (-y)
WINDOW = (0.0095, 0.0095)            # clamp opening: x wide, z tall
WINDOW_Z = RAIL_TOP + 0.015          # its centre height
CAGE_DEPTH = 0.022                   # how far the clamp channel runs into the block
SCREW_Y = FACE_Y + 0.012             # clamp screw, from the top, above the clamp
LIP = 0.002                          # the bore's top lip ...
LIP_R = 0.0040                       # ... narrows it to 8 mm: it guides the 6.5 mm flat tip and its shaft
ACCESS_R = 0.0055                    # access hole in the block's top down to the screw head
SCREW_R = 0.0050                     # M6 slotted clamp screw (10 mm head, 1.2 mm slot), down its 11 mm guide hole
SCREW_H = 0.004
HEAD_TOP = RAIL_TOP + BLOCK[2] - 0.014   # screw head 14 mm down its access hole (it guides the blade)

# -- the incoming conductor ------------------------------------------------------------------
COND_R = 0.0035                      # 35 mm2 copper (7 mm)
INS_R = 0.0055                       # 11 mm over the insulation
STRIP = 0.018                        # stripped length
INS_L = 0.220                        # insulated length from the duct exit to the stripped end
ROOT = np.array([TARGET_X, FACE_Y - INS_L - 0.025, WINDOW_Z])   # where it leaves the duct (its pivot)
REST_YAW = 8.0                       # deg: springs back off its line (its end toward the arms, -x)
REST_PITCH = 2.0                     # deg: and droops a little below the opening
SPRING_K = 40.0                      # N/m: the conductor's springback along its run (about 1 N home)
YAW_K = 1.5                          # N m / rad about its exit from the duct
PITCH_K = 1.5
WIRE_MASS = 0.065
DUCT = (0.30, 0.040, 0.050)          # slotted cable duct along x beside the rail
DUCT_C = (0.23, ROOT[1] - DUCT[1] / 2 + 0.004)

# turns of the clamp screw from as found (open, wire free) -- 0.3 turn the clamp bears on the
# conductor, 0.6 tight (about 3.5 N m on a real 35 mm2 block): more than one stroke of the wrist
TURNS_CLAMPED = 0.3
TURNS_TIGHT = 0.6
CLAMPED_K = 20000.0                  # N/m holding the conductor where it is once clamped
FULL_MM = 2.0                        # insulation within this of the block face: no bare copper showing


def plate_object() -> Part:
    battens = tuple(Piece("box", (0.045, PLATE[1] - 0.02, BATTEN), (sgn * 0.11, 0.0, -PLATE[2] / 2 - BATTEN / 2),
                          color=(0.70, 0.55, 0.35)) for sgn in (-1, 1))
    return Part("plate", pos=(PLATE_C[0], PLATE_C[1], PLATE_TOP - PLATE[2] / 2), size=PLATE, exact_z=True, color=PLATE_COL,
                friction=(0.5, 0.4), pieces=battens)


def rail_object() -> Part:
    """The top-hat rail and the five blocks on it: the middle one open, the others' wires landed."""
    c = np.array([RAIL_C[0], RAIL_C[1], PLATE_TOP + RAIL[2] / 2])

    def at(x, y, z):
        return tuple(float(v) for v in np.array([x, y, z]) - c)

    pieces = []
    ws, hs = WINDOW
    wb = BLOCK[0] - 0.0004
    for k in range(-2, 3):
        x = TARGET_X + k * PITCH
        zb = RAIL_TOP
        y0 = FACE_Y
        floor_h = WINDOW_Z - hs / 2 - zb
        pieces.append(Piece("box", (wb, CAGE_DEPTH, floor_h), at(x, y0 + CAGE_DEPTH / 2, zb + floor_h / 2), color=BLOCK_COL))
        roof_h = BLOCK[2] - floor_h - hs
        top_z = zb + BLOCK[2]
        # the roof round the screw's bore, and at its top a lip whose narrower hole guides the blade
        for hole, hz, hh in ((2 * ACCESS_R, top_z - LIP - (roof_h - LIP) / 2, roof_h - LIP),
                             (2 * LIP_R, top_z - LIP / 2, LIP)):
            front = SCREW_Y - hole / 2 - y0
            pieces.append(Piece("box", (wb, front, hh), at(x, y0 + front / 2, hz), color=BLOCK_COL))
            back = y0 + CAGE_DEPTH - (SCREW_Y + hole / 2)
            pieces.append(Piece("box", (wb, back, hh), at(x, SCREW_Y + hole / 2 + back / 2, hz), color=BLOCK_COL))
            side = (wb - hole) / 2
            for sgn in (-1, 1):
                pieces.append(Piece("box", (side, hole, hh), at(x + sgn * (hole + side) / 2, SCREW_Y, hz), color=BLOCK_COL))
        side = (wb - ws) / 2
        for sgn in (-1, 1):
            pieces.append(Piece("box", (side, CAGE_DEPTH, hs), at(x + sgn * (ws + side) / 2, y0 + CAGE_DEPTH / 2, WINDOW_Z),
                                color=BLOCK_COL))
        rest = BLOCK[1] - CAGE_DEPTH
        pieces.append(Piece("box", (wb, rest, BLOCK[2]), at(x, y0 + CAGE_DEPTH + rest / 2, zb + BLOCK[2] / 2), color=BLOCK_COL))
        # the clamp yoke at the back of the channel, drawn
        pieces.append(Piece("box", (ws - 0.0004, 0.002, hs - 0.0004), at(x, y0 + CAGE_DEPTH - 0.001, WINDOW_Z), color=STEEL,
                            collide=False))
        if k != 0:
            # landed neighbours: a conductor in from the duct, its screw down
            run = y0 - (ROOT[1] + 0.004)
            col = {-2: BROWN_INS, -1: (0.30, 0.30, 0.32), 1: (0.12, 0.30, 0.70), 2: BROWN_INS}[k]
            pieces.append(Piece("cylinder", (INS_R, run, INS_R), at(x, y0 - run / 2, WINDOW_Z), axis="Y", color=col,
                                collide=False))
            pieces.append(Piece("cylinder", (SCREW_R, SCREW_H, SCREW_R), at(x, SCREW_Y, HEAD_TOP - SCREW_H / 2 - 0.003),
                                color=STEEL, collide=False))
    # end plate and end stops either side of the row
    for sgn in (-1, 1):
        x = TARGET_X + sgn * (2.5 * PITCH + 0.0015)
        pieces.append(Piece("box", (0.002, BLOCK[1], BLOCK[2]), at(x, RAIL_C[1], RAIL_TOP + BLOCK[2] / 2), color=BLOCK_COL))
        pieces.append(Piece("box", (0.009, 0.045, 0.032), at(x + sgn * 0.0065, RAIL_C[1], RAIL_TOP + 0.016), color=(0.15, 0.15, 0.16)))
    return Part("rail", pos=tuple(float(v) for v in c), size=RAIL, exact_z=True, color=RAIL_COL, friction=(0.5, 0.4),
                contact_offset=0.001, pieces=tuple(pieces))


def duct_object() -> Part:
    """Slotted cable duct the conductors come up out of (open-topped, drawn with its fingers)."""
    c = (DUCT_C[0], DUCT_C[1], PLATE_TOP + DUCT[2] / 2)
    t = 0.002
    pieces = [Piece("box", (DUCT[0], t, DUCT[2]), (0.0, -DUCT[1] / 2 + t / 2, 0.0)),
              Piece("box", (DUCT[0], DUCT[1], t), (0.0, 0.0, -DUCT[2] / 2 + t / 2))]
    n = 15
    w = DUCT[0] / n
    for k in range(n):
        x = -DUCT[0] / 2 + w * (k + 0.5)
        if abs(c[0] + x - TARGET_X) < 2.6 * PITCH:
            hgt = DUCT[2] - (WINDOW_Z + 0.008 - PLATE_TOP)       # slots the conductors come out over
            pieces.append(Piece("box", (w * 0.55, t, hgt), (x, DUCT[1] / 2 - t / 2, DUCT[2] / 2 - hgt / 2)))
            continue
        pieces.append(Piece("box", (w * 0.55, t, DUCT[2]), (x, DUCT[1] / 2 - t / 2, 0.0)))
    return Part("duct", pos=c, size=(DUCT[0], t, DUCT[2]), exact_z=True, color=DUCT_COL, collision="none",
                pieces=tuple(pieces))


def wire_rot(yaw_deg, pitch_deg):
    """Rotation taking +y (the conductor's run) to its direction: yaw about z, then pitch about x
    (+: end down)."""
    a, b = math.radians(yaw_deg), math.radians(pitch_deg)
    rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    rx = np.array([[1, 0, 0], [0, math.cos(-b), -math.sin(-b)], [0, math.sin(-b), math.cos(-b)]])
    return rz @ rx


def carrier_object() -> Part:
    """Invisible slider in the duct: the slack the conductor can be drawn out by, sprung back."""
    return Part("carrier", pos=tuple(float(v) for v in ROOT), size=(0.004, 0.004, 0.004), exact_z=True, mass=0.300,
                collision="none", opacity=0.0,
                spring=Spring(axis="Y", lower=-0.004, upper=0.040, stiffness=SPRING_K, damping=6.0, target=0.0))


def yoke_object() -> Part:
    """Invisible swivel at the duct exit: the conductor's sideways swing, sprung to its rest line."""
    return Part("yoke", pos=tuple(float(v) for v in ROOT), size=(0.004, 0.004, 0.004), exact_z=True, mass=0.300,
                collision="none", opacity=0.0,
                hinge=Hinge(pivot=tuple(float(v) for v in ROOT), axis="Z", damping=0.02, lower_deg=-30, upper_deg=30,
                            stiffness=YAW_K * math.pi / 180, target_deg=REST_YAW, parent="carrier",
                            parent_pos=tuple(float(v) for v in ROOT)))


def wire_centre_rest():
    return ROOT + np.array([0.0, INS_L / 2, 0.0])


def wire_object() -> Part:
    """The conductor from the duct exit: insulation along +y, the stripped end beyond it. Spawned on
    its straight line; its hinges' springs take it to its rest (off to one side, drooping)."""
    c = wire_centre_rest()
    bare = Piece("cylinder", (COND_R, STRIP, COND_R), (0.0, INS_L / 2 + STRIP / 2, 0.0), axis="Y", color=COPPER,
                 friction=(0.5, 0.4))
    return Part("wire", pos=tuple(float(v) for v in c), size=(INS_R, INS_L, INS_R), shape="cylinder", axis="Y",
                exact_z=True, mass=WIRE_MASS, color=BLACK_INS, friction=(1.2, 1.0), restitution=0.0,
                contact_offset=0.001, max_depenetration=0.2, pieces=(bare,),
                hinge=Hinge(pivot=tuple(float(v) for v in ROOT), axis="X", damping=0.02, lower_deg=-14, upper_deg=25,
                            stiffness=PITCH_K * math.pi / 180, target_deg=-REST_PITCH, parent="yoke",
                            parent_pos=tuple(float(v) for v in ROOT)))


RECESS = (0.0037, 0.003)             # the head's drive slot, drawn as a round recess: floor radius, depth (the tip seats in it)
FUNNEL = 0.0047                      # its radius at the top: the recess flares, so the blade's tip finds it


def screw_object() -> Part:
    """The clamp screw's head down its access hole, with a flared driving recess the blade's tip seats
    in: turns about z when driven. Root: the recess floor; the recess wall is a ring of tilted staves."""
    floor_t = SCREW_H - RECESS[1]
    pos = (TARGET_X, SCREW_Y, HEAD_TOP - SCREW_H + floor_t / 2)
    n = 8
    t = 0.0006
    tilt = math.atan2(FUNNEL - RECESS[0], RECESS[1])
    rm = (RECESS[0] + FUNNEL) / 2 + t / 2
    h = math.hypot(FUNNEL - RECESS[0], RECESS[1])
    w = 2 * FUNNEL * math.tan(math.pi / n) + 0.0004
    zc = floor_t / 2 + RECESS[1] / 2
    ring = tuple(Piece("box", (w, t, h), (rm * math.sin(2 * math.pi * k / n), rm * math.cos(2 * math.pi * k / n), zc),
                       rot=(-tilt, 0.0, -2 * math.pi * k / n)) for k in range(n))
    mark = Piece("box", (0.0012, 0.0012, 0.0006), (0.0, FUNNEL + 0.0002, floor_t / 2 + RECESS[1] + 0.0002),
                 color=(0.95, 0.80, 0.10), collide=False)                 # painted mark, so its turning shows
    return Part("screw", pos=pos, size=(SCREW_R, floor_t, SCREW_R), shape="cylinder", axis="Z", exact_z=True, mass=0.002,
                color=STEEL, friction=(0.8, 0.6), restitution=0.0, contact_offset=0.0008, pieces=(*ring, mark),
                hinge=Hinge(pivot=pos, axis="Z", damping=0.0005, lower_deg=-1e6, upper_deg=1e6, stiffness=0.02,
                            target_deg=0.0))


# -- screwdriver in its holder ---------------------------------------------------------------
HOLDER_XY = (0.180, 0.200)
HOLDER = (0.040, 0.040, 0.040)
HOLE = 0.016
HANDLE = (0.015, 0.050)              # half-width, length: square-section grip
SHAFT = (0.0035, 0.045)              # stubby slotted screwdriver, 1.0 x 6.5 mm tip (ISO 2380-1): 7 mm round shaft
BIT_R = 0.0032                       # its 6.5 mm wide flat tip, drawn round (6.4 mm)
DRIVER_MASS = 0.090
DRIVER_LIFT = 0.004


def replace_offset(piece, off):
    from dataclasses import replace
    return replace(piece, offset=tuple(float(v) for v in off))


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
    """Screwdriver standing tip-down in the holder: root is the shaft; the grip above it."""
    tip = driver_tip_rest()
    c = tip + np.array([0.0, 0.0, SHAFT[1] / 2])
    handle = Piece("box", (2 * HANDLE[0], 2 * HANDLE[0], HANDLE[1]), (0.0, 0.0, SHAFT[1] / 2 + HANDLE[1] / 2),
                   color=(0.80, 0.12, 0.10), friction=(1.6, 1.4))
    band = Piece("box", (2 * HANDLE[0] + 0.0008, 2 * HANDLE[0] + 0.0008, 0.012), (0.0, 0.0, SHAFT[1] / 2 + 0.010),
                 color=(0.95, 0.80, 0.10), collide=False)
    flute = Piece("box", (0.0012, 2 * BIT_R + 0.0004, 0.006), (0.0, 0.0, -SHAFT[1] / 2 + 0.003), color=(0.2, 0.2, 0.2),
                  collide=False)
    return Part("driver", pos=tuple(float(v) for v in c), size=(SHAFT[0], SHAFT[1], SHAFT[0]), shape="cylinder", axis="Z",
                exact_z=True, mass=DRIVER_MASS, color=STEEL, friction=(0.6, 0.5), restitution=0.0, contact_offset=0.0012,
                max_depenetration=0.2, com=(0.0, 0.0, SHAFT[1] / 2 + 0.010), pieces=(handle, band, flute))


# -- measurements ----------------------------------------------------------------------------

def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def wire_end(pose):
    """End of the insulation (where the bare copper starts), the bare tip, and the unit axis (+y local)."""
    p = np.asarray(pose, float)
    axis = quat_matrix(p[3:7]) @ np.array([0.0, 1.0, 0.0])
    ins_end = p[:3] + axis * INS_L / 2
    return ins_end, ins_end + axis * STRIP, axis


def in_clamp(pose):
    """How far the bare end is into the target block's clamp channel (m, tip past the entry face) when
    it is lined up with the window, and the insulation's gap to the face (m)."""
    ins_end, tip, axis = wire_end(pose)
    lined = (abs(tip[0] - TARGET_X) < WINDOW[0] / 2 and abs(tip[2] - WINDOW_Z) < WINDOW[1] / 2
             and abs(ins_end[0] - TARGET_X) < 0.005 and abs(ins_end[2] - WINDOW_Z) < 0.005)
    depth = float(tip[1] - FACE_Y)
    gap = float(FACE_Y - ins_end[1])
    return (depth if lined and depth > 0 else 0.0), gap, lined


def driver_tip(pose):
    """Bottom of the screwdriver's tip, and its axis (unit, toward the grip)."""
    p = np.asarray(pose, float)
    axis = quat_matrix(p[3:7]) @ np.array([0.0, 0.0, 1.0])
    return p[:3] - axis * SHAFT[1] / 2, axis


def head_centre():
    """Centre of the recess floor, where a seated blade's tip rests."""
    return np.array([TARGET_X, SCREW_Y, HEAD_TOP - RECESS[1]])
