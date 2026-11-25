"""Bodies and joints of the bathroom-sink pop-up stopper task, built from ``parts.py``.

- ``basin`` (fixed): bathroom wall panel, Duravit ME by Starck 550 x 440 x 180 mm wall-hung basin (rim deck, aprons,
  stepped bowl, floor with the drain hole), and a rest post under the pivot rod's rear end.
- ``trap`` (fixed): Dearborn P9701 1-1/4 in P-trap on the tailpiece: J-bend swung 45 deg to the left, quarter bend
  and trap arm into the wall, slip nuts.
- ``drain`` (kinematic, for contact sensing): 1-1/4 in pop-up body, flange, rear port (bore, ball seat, the port end's
  lead) facing +x, and the port's first full thread crest (the land at the thread start), which the nut passes as a
  real nut's thread passes the port's crests (``passes``); before its thread catches, the nut's washer meets the
  seated ball there.
- ``stopper`` (free, 36 g), ``pivot_rod`` (free, 44 g), ``pivot_nut`` (free, 10 g, its washer beads inside).
- Hidden thread carrier (``HIDDEN_NAMES``): a slide along the port axis and three hinges about it in series, and the
  weld ``w_thread`` that joins the nut to it where the nut stands when the thread catches (as the hole-saw arbor task).
"""

from __future__ import annotations

import math

import mechkit as mk
import parts as P
from mechkit import body, box, capsule, cylinder, hinge, slide, soften, sphere, weld

MECHKIT_HELPER = True

WALL_TILE = (0.86, 0.87, 0.86)
CERAMIC = (0.96, 0.97, 0.98)
TRAP_WHITE = (0.93, 0.93, 0.91)
POST_WOOD = (0.72, 0.58, 0.40)
DRAIN_GREY = (0.62, 0.64, 0.68)
CHROME = (0.68, 0.71, 0.76)
STOPPER_CAP = (0.36, 0.40, 0.46)
SEAL_RUBBER = (0.12, 0.12, 0.13)
BALL_WHITE = (0.96, 0.96, 0.94)
TIP_GREY = (0.52, 0.54, 0.57)
STEEL_ZINC = (0.92, 0.77, 0.28)
NUT_BLUE = (0.16, 0.36, 0.72)
NUT_RIB = (0.10, 0.24, 0.52)
LAND_COL = (0.48, 0.50, 0.54)
HIDDEN_COL = (0.2, 0.2, 0.2)
SLIP = (0.18, 0.15)

HIDDEN_NAMES = ("thread_carrier", "thread_a", "thread_b", "thread_turner")
TURN_JOINTS = ("turn_1", "turn_2", "turn_3")
FREE_NAMES = ("stopper", "pivot_rod", "pivot_nut")


def _quat_x(angle: float) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in mk.matrix_to_quat(mk.axis_angle((1.0, 0.0, 0.0), angle)))


def _ring_x(r_in: float, r_out: float, x0: float, x1: float, n: int, color, centre=(0.0, 0.0), **kw) -> list:
    """``n`` boxes forming a ring about an axis along x (at y, z = ``centre``), from ``x0`` to ``x1``."""
    w = 2.0 * r_out * math.tan(math.pi / n) * 1.03
    r_mid = 0.5 * (r_in + r_out)
    out = []
    for k in range(n):
        a = 2.0 * math.pi * k / n
        out.append(box((x1 - x0, r_out - r_in, w), pos=(0.5 * (x0 + x1), centre[0] + r_mid * math.cos(a), centre[1] + r_mid * math.sin(a)),
                       quat=_quat_x(a), color=color, **kw))
    return out


def _segment(p0, p1, radius: float, color, **kw):
    """A capsule from ``p0`` to ``p1`` (body frame)."""
    import numpy as np

    a, b = np.asarray(p0, float), np.asarray(p1, float)
    d = b - a
    length = float(np.linalg.norm(d))
    f = mk.frame_with_x(d / length)
    q = mk.matrix_to_quat(np.column_stack([f[:, 1], f[:, 2], f[:, 0]]))  # capsules run along their local z
    return capsule(radius, max(1e-4, length), pos=tuple(0.5 * (a + b)), quat=tuple(float(v) for v in q), color=color, **kw)


def bowl_half(h: float) -> tuple[float, float]:
    """Bowl opening half length and half width at height fraction ``h`` (0: the floor, 1: under the deck)."""
    s = math.sqrt(max(0.0, 1.0 - (1.0 - h) ** 2))
    a = P.BOWL_FLOOR[0] + (P.BOWL_L / 2.0 - P.BOWL_FLOOR[0]) * s
    b = P.BOWL_FLOOR[1] + (P.BOWL_W / 2.0 - P.BOWL_FLOOR[1]) * s
    return a, b


def build_basin(top: float) -> None:
    """Wall panel, the ME by Starck basin hung on it, and the rod's rest post."""
    xd, xf, xw = P.X_DRAIN, P.X_FRONT, P.X_WALL
    half_w = P.BASIN_W / 2.0
    zr, t = P.Z_RIM, P.CERAMIC_T
    shapes = [box((P.WALL_T, 0.90, 0.60), pos=(xw + P.WALL_T / 2.0, 0.0, 0.30), color=WALL_TILE)]

    # Rim deck round the 520 x 290 mm bowl opening, the tap deck behind it, and the front and side faces under the rim
    bx0, bx1 = xd - P.BOWL_L / 2.0, xd + P.BOWL_L / 2.0
    by = P.BOWL_W / 2.0
    dz = zr - P.DECK_T / 2.0
    shapes += [
        box((bx0 - xf, 2.0 * half_w, P.DECK_T), pos=(0.5 * (xf + bx0), 0.0, dz), color=CERAMIC),
        box((xw - bx1, 2.0 * half_w, P.DECK_T), pos=(0.5 * (bx1 + xw), 0.0, dz), color=CERAMIC),
        box((xw - bx1, 2.0 * half_w, 0.070), pos=(0.5 * (bx1 + xw), 0.0, zr - P.DECK_T - 0.035), color=CERAMIC),
        box((0.012, 2.0 * half_w, P.APRON_H), pos=(xf + 0.006, 0.0, zr - P.APRON_H / 2.0), color=CERAMIC),
    ]
    for s in (-1.0, 1.0):
        shapes += [
            box((bx1 - bx0, half_w - by, P.DECK_T), pos=(0.5 * (bx0 + bx1), s * 0.5 * (by + half_w), dz), color=CERAMIC),
            box((xw - xf, 0.012, P.APRON_H), pos=(0.5 * (xf + xw), s * (half_w - 0.006), zr - P.APRON_H / 2.0), color=CERAMIC),
        ]

    # Stepped bowl: frames from the floor up to the deck, each reaching under the next so that the bowl is closed
    z0, z1 = P.Z_FLOOR, zr - P.DECK_T
    n = P.BOWL_TERRACES
    inner = [bowl_half((k + 0.5) / n) for k in range(n)] + [(P.BOWL_L / 2.0, P.BOWL_W / 2.0)]
    for k in range(n):
        za, zb = z0 + (z1 - z0) * k / n, z0 + (z1 - z0) * (k + 1) / n
        (a, b), (a2, b2) = inner[k], inner[k + 1]
        ta, tb = max(t, a2 - a + 0.003), max(t, b2 - b + 0.003)
        h, zm = zb - za, 0.5 * (za + zb)
        shapes += [
            box((ta, 2.0 * (b + tb), h), pos=(xd - a - ta / 2.0, 0.0, zm), color=CERAMIC),
            box((ta, 2.0 * (b + tb), h), pos=(xd + a + ta / 2.0, 0.0, zm), color=CERAMIC),
            box((2.0 * a, tb, h), pos=(xd, b + tb / 2.0, zm), color=CERAMIC),
            box((2.0 * a, tb, h), pos=(xd, -(b + tb / 2.0), zm), color=CERAMIC),
        ]
    # Floor under the flange with the drain hole, reaching under the first step
    fa, fb = max(P.BOWL_FLOOR[0] + t, inner[0][0] + 0.003), max(P.BOWL_FLOOR[1] + t, inner[0][1] + 0.003)
    hole = P.DRAIN_OD / 2.0 + 0.0008
    fz, ft = 0.5 * (P.Z_UNDER + P.Z_FLOOR), P.Z_FLOOR - P.Z_UNDER
    shapes += [
        box((fa - hole, 2.0 * fb, ft), pos=(xd - 0.5 * (fa + hole), 0.0, fz), color=CERAMIC),
        box((fa - hole, 2.0 * fb, ft), pos=(xd + 0.5 * (fa + hole), 0.0, fz), color=CERAMIC),
        box((2.0 * hole, fb - hole, ft), pos=(xd, 0.5 * (fb + hole), fz), color=CERAMIC),
        box((2.0 * hole, fb - hole, ft), pos=(xd, -0.5 * (fb + hole), fz), color=CERAMIC),
    ]

    # Rest post under the rod's rear end (15 x 16 mm timber, 3 mm V-cheeks)
    px0, px1 = xd + P.REST_U[0], xd + P.REST_U[1]
    post_top = P.PORT_Z - P.ROD_START_DROP - P.ROD_D / 2.0
    shapes.append(box((px1 - px0, 0.016, post_top), pos=(0.5 * (px0 + px1), P.Y_DRAIN, post_top / 2.0), color=POST_WOOD,
                      friction=SLIP, friction_combine="min"))
    for s in (-1.0, 1.0):
        shapes.append(box((px1 - px0, 0.003, 0.004), pos=(0.5 * (px0 + px1), P.Y_DRAIN + s * 0.0045, post_top + 0.0015), color=POST_WOOD,
                          friction=SLIP, friction_combine="min"))
    body("basin", shapes, pos=(0.0, 0.0, top), color=CERAMIC)


def build_trap(top: float) -> None:
    """Dearborn P9701 P-trap: J-bend under the tailpiece, swung toward +y, quarter bend and trap arm to the wall."""
    import numpy as np

    r = P.TRAP_OD / 2.0
    xd, yd = P.X_DRAIN, P.Y_DRAIN
    sw = math.radians(P.TRAP_SWING_DEG)
    s = np.array([math.cos(sw), math.sin(sw), 0.0])
    inlet = np.array([xd, yd, 0.0])
    zc = P.TRAP_U_Z
    centre = inlet + s * (P.TRAP_CC / 2.0) + np.array([0.0, 0.0, zc])
    ox, oy = P.TRAP_OUT_XY
    rq = 0.025
    shapes = [
        # Inlet slip nut on the tailpiece (drawn only: the drain's bore is inside it) and the J's inlet leg
        cylinder(P.SLIP_NUT_OD / 2.0, P.SLIP_NUT_L, pos=(xd, yd, P.J_INLET_TOP - P.SLIP_NUT_L / 2.0), color=TRAP_WHITE, solid=False),
        cylinder(r, P.J_INLET_TOP - P.SLIP_NUT_L - 0.104, pos=(xd, yd, 0.5 * (P.J_INLET_TOP - P.SLIP_NUT_L + 0.104)), color=TRAP_WHITE, solid=False),
        _segment((xd, yd, zc), (xd, yd, 0.104), r, TRAP_WHITE),
    ]
    pts = []
    for k in range(7):
        phi = math.pi * k / 6.0
        pts.append(centre - s * (P.TRAP_CC / 2.0) * math.cos(phi) - np.array([0.0, 0.0, (P.TRAP_CC / 2.0) * math.sin(phi)]))
    shapes += [_segment(pts[k], pts[k + 1], r, TRAP_WHITE) for k in range(6)]
    z_top_leg = P.TRAP_ARM_Z - rq
    shapes += [
        _segment((ox, oy, zc), (ox, oy, z_top_leg), r, TRAP_WHITE),
        cylinder(P.SLIP_NUT_OD / 2.0, P.SLIP_NUT_L, pos=(ox, oy, z_top_leg - 0.010), color=TRAP_WHITE, solid=False),
    ]
    bend = [(ox + rq * (1.0 - math.cos(a)), oy, z_top_leg + rq * math.sin(a)) for a in (0.0, math.pi / 6.0, math.pi / 3.0, math.pi / 2.0)]
    shapes += [_segment(bend[k], bend[k + 1], r, TRAP_WHITE) for k in range(3)]
    x_arm0 = ox + rq
    shapes += [
        cylinder(r, P.X_WALL - x_arm0, pos=(0.5 * (x_arm0 + P.X_WALL), oy, P.TRAP_ARM_Z), axis="x", color=TRAP_WHITE),
        cylinder(0.034, 0.006, pos=(P.X_WALL - 0.003, oy, P.TRAP_ARM_Z), axis="x", color=CHROME),
    ]
    body("trap", shapes, pos=(0.0, 0.0, top), color=TRAP_WHITE)


def build_drain(top: float) -> None:
    """1-1/4 in pop-up body: bore with the port window, flange, ball seat and the port (bore round the ball, lead)."""
    shapes = []
    n_bore = 16
    r_in, r_out = P.DRAIN_ID / 2.0, P.DRAIN_OD / 2.0
    wall, r_mid = r_out - r_in, 0.5 * (r_in + r_out)
    chord = 2.0 * r_out * math.tan(math.pi / n_bore)
    z_lo, z_hi = P.Z_DRAIN_BOT, P.Z_FLOOR
    win_lo, win_hi = P.PORT_Z - 0.011, P.PORT_Z + 0.011
    for k in range(n_bore):
        phi = 2.0 * math.pi * k / n_bore
        cx, cy = r_mid * math.cos(phi), r_mid * math.sin(phi)
        spans = [(z_lo, win_lo), (win_hi, z_hi)] if k in (0, 1, n_bore - 1) else [(z_lo, z_hi)]
        for za, zb in spans:
            shapes.append(box((wall, chord, zb - za), pos=(cx, cy, 0.5 * (za + zb)), yaw=phi, color=DRAIN_GREY, friction=SLIP, friction_combine="min"))
    fl_t, fl_r = 0.0025, P.FLANGE_D / 2.0
    fl_chord = 2.0 * fl_r * math.tan(math.pi / n_bore)
    for k in range(n_bore):
        phi = 2.0 * math.pi * k / n_bore
        rm = 0.5 * (r_in + fl_r)
        shapes.append(box((fl_r - r_in, fl_chord, fl_t), pos=(rm * math.cos(phi), rm * math.sin(phi), P.FLANGE_Z - fl_t / 2.0), yaw=phi,
                          color=CHROME, friction=SLIP, friction_combine="min"))
    # Ball seat inside the port (the ball's front bears on it) and the port tube round the ball out to its lead
    shapes += _ring_x(P.SEAT_RING_R[0], P.SEAT_RING_R[1], P.SEAT_RING_U - 0.0008, P.SEAT_RING_U + 0.0008, 8, DRAIN_GREY,
                      centre=(0.0, P.PORT_Z), friction=SLIP, friction_combine="min")
    # The port's lead: the bore round the ball; the nut's crests ride over it, so it does not touch the nut (a caught nut is
    # held by the thread, one not yet caught stops where its washer meets the seated ball)
    shapes += _ring_x(P.PORT_BORE_R, P.PILOT_R, P.SEAT_RING_U, P.U_PORT_END, P.PORT_FACETS, DRAIN_GREY, centre=(0.0, P.PORT_Z),
                      passes=("pivot_nut",), friction=SLIP, friction_combine="min")
    # The first full thread crest at the thread start: the nut's thread passes it (filtered), fingers do not
    shapes += _ring_x(P.PILOT_R, P.LAND_OUTER_R, P.U_START - P.LAND_T, P.U_START, 12, LAND_COL, centre=(0.0, P.PORT_Z),
                      passes=("pivot_nut",), friction=SLIP, friction_combine="min")
    body("drain", shapes, pos=(P.X_DRAIN, P.Y_DRAIN, top), kinematic=True, color=DRAIN_GREY)


def build_stopper(top: float) -> None:
    """Danco 11041 stopper (origin at the cap's seal; starts closed with its hole facing the port, +x)."""
    r_head = P.STOP_HEAD_D / 2.0
    upper_top = -0.0010
    eye_top_outer = -(P.EYE_DEPTH - P.EYE_H / 2.0 - 0.0032)
    upper_len, upper_mid = upper_top - eye_top_outer, 0.5 * (upper_top + eye_top_outer)
    eye_bot_outer = -(P.EYE_DEPTH + P.EYE_H / 2.0 + 0.0032)
    lower_len, lower_mid = eye_bot_outer - (-P.STOP_BELOW_SEAL), 0.5 * (eye_bot_outer - P.STOP_BELOW_SEAL)
    bar_r = 0.0016
    side_h = P.EYE_H + 4.0 * bar_r
    side_y = P.EYE_W / 2.0 + P.EYE_WALL / 2.0
    kw = dict(friction=SLIP, friction_combine="min")
    shapes = [
        cylinder(r_head - 0.0008, 0.0025, pos=(0.0, 0.0, 0.00125), color=SEAL_RUBBER, **kw),
        cylinder(r_head, 0.0075, pos=(0.0, 0.0, 0.00625), color=STOPPER_CAP, friction=mk.GRIPPY),
        cylinder(r_head - 0.0022, 0.0025, pos=(0.0, 0.0, 0.01125), color=STOPPER_CAP, friction=mk.GRIPPY),
        box((P.GUIDE_SPAN, P.FIN_THICK, upper_len), pos=(0.0, 0.0, upper_mid), color=CHROME, **kw),
        box((P.FIN_THICK, P.GUIDE_SPAN, upper_len), pos=(0.0, 0.0, upper_mid), color=CHROME, **kw),
        capsule(bar_r, P.EYE_W, pos=(0.0, 0.0, -(P.EYE_DEPTH - P.EYE_H / 2.0 - bar_r)), axis="y", color=CHROME, **kw),
        capsule(bar_r, P.EYE_W, pos=(0.0, 0.0, -(P.EYE_DEPTH + P.EYE_H / 2.0 + bar_r)), axis="y", color=CHROME, **kw),
        box((P.EYE_THICK, P.EYE_WALL, side_h), pos=(0.0, side_y, -P.EYE_DEPTH), color=CHROME, **kw),
        box((P.EYE_THICK, P.EYE_WALL, side_h), pos=(0.0, -side_y, -P.EYE_DEPTH), color=CHROME, **kw),
        box((P.EYE_THICK, P.EYE_W + 2.0 * P.EYE_WALL, max(lower_len, 0.0008)), pos=(0.0, 0.0, lower_mid), color=CHROME, **kw),
    ]
    com, inertia = P.stopper_com_and_inertia()
    body("stopper", shapes, pos=(P.X_DRAIN, P.Y_DRAIN, top + P.FLANGE_Z + P.OVER), mass=P.STOP_MASS, color=CHROME,
         center=com, inertia=inertia, contacts=True)


def build_pivot_rod(top: float) -> None:
    """Danco 88532 ball rod (origin at the ball's centre), level, its tip resting in the seat and its rear on the post."""
    stub_back, stub_front = -0.0050, -P.TIP_REACH
    stub_cyl = (stub_back - stub_front) - 2.0 * P.TIP_R
    steel_len = P.ROD_OUTER_L - P.ROD_THREAD_IN
    shapes = [
        sphere(P.BALL_R, color=BALL_WHITE, friction=(1.0, 0.95)),
        capsule(P.TIP_R, stub_cyl, pos=(0.5 * (stub_back + stub_front), 0.0, 0.0), axis="x", color=TIP_GREY, friction=SLIP, friction_combine="min"),
        cylinder(P.ROD_D / 2.0, steel_len, pos=(P.BALL_R + steel_len / 2.0, 0.0, 0.0), axis="x", color=STEEL_ZINC, friction=mk.GRIPPY),
        box(P.CLEVIS_BLOCK, pos=(P.CLEVIS_AT_X, 0.0, 0.0), color=STEEL_ZINC, friction=mk.GRIPPY),
    ]
    com, inertia = P.rod_com_and_inertia()
    body("pivot_rod", shapes, pos=(P.X_DRAIN + P.U_BALL_START, P.Y_DRAIN, top + P.PORT_Z - P.ROD_START_DROP + 0.0002), mass=P.ROD_MASS,
         color=STEEL_ZINC, center=com, inertia=inertia, contacts=True)


def nut_start_u() -> float:
    """The nut's front face at the start: on the rod behind the ball, its washer beads just on the ball."""
    touch = math.sqrt((P.BALL_R + P.WASHER_BEAD_R) ** 2 - P.WASHER_RING_R**2)
    return P.U_BALL_START + touch + P.WASHER_BEAD_R - P.NUT_SKIRT_L + 0.0002


def build_pivot_nut(top: float) -> None:
    """The pop-up's pivot nut (origin at its front face's centre, +x toward its back): knurled skirt, back wall with the
    rod hole, and the washer's four rubber beads that bear on the ball's rear."""
    hidden = HIDDEN_NAMES
    r_o = P.NUT_OD / 2.0
    shapes = _ring_x(P.NUT_MINOR_R, r_o, 0.0, P.NUT_FRONT_BAND, P.NUT_FACETS, NUT_BLUE, passes=hidden, friction=mk.GRIPPY)
    shapes += _ring_x(P.NUT_MINOR_R, r_o, P.NUT_FRONT_BAND, P.NUT_SKIRT_L, P.NUT_BACK_FACETS, NUT_BLUE, passes=hidden, friction=mk.GRIPPY)
    shapes += _ring_x(P.NUT_HOLE_R, r_o, P.NUT_SKIRT_L, P.NUT_L, P.NUT_BACK_FACETS, NUT_BLUE, passes=hidden, friction=mk.GRIPPY)
    for k in range(12):
        a = 2.0 * math.pi * (k + 0.5) / 12.0
        shapes.append(box((P.NUT_L - 0.002, 0.0012, 0.0016), pos=(P.NUT_L / 2.0, (r_o + 0.0003) * math.cos(a), (r_o + 0.0003) * math.sin(a)),
                          quat=_quat_x(a), color=NUT_RIB, solid=False))
    for k in range(4):
        a = math.pi / 4.0 + k * math.pi / 2.0
        bead = sphere(P.WASHER_BEAD_R, pos=(P.NUT_SKIRT_L - P.WASHER_BEAD_R, P.WASHER_RING_R * math.cos(a), P.WASHER_RING_R * math.sin(a)),
                      color=SEAL_RUBBER, passes=hidden, friction=(P.WASHER_MU, P.WASHER_MU), friction_combine="max")
        shapes.append(soften(bead, P.WASHER_K, P.WASHER_DAMP))
    com, inertia = P.nut_com_and_inertia()
    z = top + P.PORT_Z - P.ROD_START_DROP + 0.0002 + P.NUT_START_SHIFT[1]
    droop = tuple(float(v) for v in mk.matrix_to_quat(mk.axis_angle((0.0, 1.0, 0.0), -math.radians(P.NUT_START_DROOP_DEG))))
    body("pivot_nut", shapes, pos=(P.X_DRAIN + nut_start_u() + P.NUT_START_SHIFT[0], P.Y_DRAIN, z), quat=droop, mass=P.NUT_MASS,
         color=NUT_BLUE, center=com, inertia=inertia, contacts=True, linear_damping=0.5, angular_damping=1.0)


def build_carrier(top: float) -> None:
    """Hidden thread carrier at the thread start: slide ``thread_s`` along +x, three hinges about -x in series (turning
    the nut on is positive), and the weld ``w_thread`` (made where the nut stands when the thread catches)."""
    at = (P.X_DRAIN + P.U_START, P.Y_DRAIN, top + P.PORT_Z)
    every = ("basin", "trap", "drain", "stopper", "pivot_rod", "pivot_nut") + HIDDEN_NAMES
    for idx, name in enumerate(HIDDEN_NAMES):
        ghost = [box((0.004, 0.004, 0.004), pos=(-0.012 - 0.0015 * idx, 0.0, 0.0), color=HIDDEN_COL, passes=every)]
        body(name, ghost, pos=at, mass=P.HIDDEN_MASS, inertia=(P.HIDDEN_INERTIA,) * 3, color=HIDDEN_COL)
    slide("thread_s", "thread_carrier", "drain", axis=(1.0, 0.0, 0.0), limits=(-P.S_START - 0.0015, 0.0035), anchor=at, damping=12.0)
    for j_name, child, parent in zip(TURN_JOINTS, HIDDEN_NAMES[1:], HIDDEN_NAMES[:-1]):
        hinge(j_name, child, parent, anchor=at, axis=(-1.0, 0.0, 0.0), limits=(P.HINGE_LOWER_RAD, P.HINGE_UPPER_RAD),
              spring=P.THREAD_SPRING_K, rest=0.0, damping=P.THREAD_DAMP)
    weld("w_thread", "pivot_nut", "thread_turner", made=False)


def build(top: float) -> None:
    build_basin(top)
    build_trap(top)
    build_drain(top)
    build_stopper(top)
    build_pivot_rod(top)
    build_pivot_nut(top)
    build_carrier(top)
    mk.sense_contact("stopper", "drain", "pivot_rod")
    mk.sense_contact("pivot_rod", "drain", "stopper", "pivot_nut")
    mk.sense_contact("pivot_nut", "drain", "pivot_rod")
