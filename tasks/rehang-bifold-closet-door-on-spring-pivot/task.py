"""The station table stands in for a bedroom floor. Across it stands the front wall of a reach-in closet with a 24 x
80-3/4 in finished opening (61.0 cm wide, 205.1 cm high) and, in the opening, a full-size two-panel bifold closet
door: two louvred pine panels 30.0 cm wide, 200.0 cm tall and 3.5 cm thick (a 24 x 80 in louvred pine bifold, ReliaBilt,
listed at 15.2 lb), hinged together, white primer finish, a round knob on the lead panel. The door hangs from a 1 in aluminium track under the head jamb. The pivot
panel's spring-loaded top pivot pin is in the top pivot bracket next to the left jamb, and the lead panel's guide is
in the track. At the floor, the pivot panel's bottom pivot bolt (a toothed wheel and a pin) belongs in a steel jamb
bracket screwed to the left jamb and the floor: a 7 cm slot along the opening, with toothed rails on both sides of it.

The door is folded open, both panels standing out into the room in front of the left jamb, and its bottom has come out
of the bracket: the pin stands on the floor 5.5 cm in front of the bracket's slot and 1.3 cm further from the jamb than
the bracket's centre hole (directly under the top pivot), so the door hangs out of plumb. A folded towel lies on the
floor at the right, a storage box stands inside the closet and a small waste bin stands at the left.
"""

import math
import pathlib
import sys

import numpy as np

from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402  importing it hooks yam_arena
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # the floor stand-in

# -- real dimensions (inches from the makers' instructions, see README) -------------------------------------------------
IN = 0.0254
OPEN_W = 24.0 * IN  # finished opening width
OPEN_H = 80.75 * IN  # finished opening height (Masonite chart for a 79 in panel)
H = 78.75 * IN  # panel height (ReliaBilt 24 x 80 in louvred pine bifold: actual 23.63 x 78.75 x 1.375 in)
W = 11.75 * IN  # each panel's width (two panels and the hinge gap = 23.62 in, 1/4 in clear at the jamb)
T = 1.375 * IN  # panel thickness
PIV_IN = 1.25 * IN  # pivot and guide holes' centres from the panel's edges (JELD-WEN JII103)
JAMB_GAP = 0.25 * IN  # pivot panel to jamb
HINGE_GAP = 0.003  # between the two panels at their hinges
TRACK_H = 1.0 * IN  # "1 in top track"
TOP_GAP = 0.375 * IN  # door top to track, as adjusted
SEAT_BOTTOM = OPEN_H - TRACK_H - TOP_GAP - H  # door bottom above the floor when seated: 5/8 in

# bottom pivot bolt and jamb bracket (reconstruction; see README)
STEM = 0.0015 + 0.25 * IN  # bolt shank showing between the door's bottom and the wheel (adjusted for the 78-3/4 in panel)
WHEEL_R, WHEEL_T = 0.011, 0.003  # the toothed adjusting wheel
PIN_R, PIN_L, PIN_CHAMFER = 0.004, 0.004, 0.0015  # the pin below the wheel, with its rounded-off tip
PLATE_T = 0.002  # bracket's floor plate
RAIL_H, RAIL_W = 0.003, 0.004  # toothed rails either side of the slot
SLOT_W = 0.010
RAIL_TOP = PLATE_T + RAIL_H  # 5 mm above the floor
SEAT_TIP = RAIL_TOP - PIN_L  # the pin's tip when the wheel sits on the rails: 1 mm above the floor
SLOT_Y0, SLOT_Y1 = 0.012, 0.082  # slot runs this far from the jamb face
BRACKET_D = 0.032  # plate's depth (x)
BRACKET_L = 0.090  # plate's length from the jamb (y)
SPRING_K = 300.0  # top pivot spring, N/m (estimate)
SPRING_SEATED = 5.0  # its push when the door is seated, N (estimate)

# -- layout (world) ------------------------------------------------------------------------------------------------------
XW = 0.52  # room face of the closet's front wall
WALL_T = 0.116  # 2x4 stud wall with drywall both sides (4-9/16 in jamb)
XP = XW + 0.0365  # door's mid-plane when closed: the track's centre line, pivots and guide on it
YL = 0.12  # the left (pivot) jamb's face
YR = YL - OPEN_W  # the right jamb's face
YP = YL - JAMB_GAP - PIV_IN  # pivot axis, under the top pivot
X_BACK = 0.90  # closet back wall (the bench ends at 0.91)

KICK_X, KICK_Y = -0.055, -0.013  # where the bottom pin starts, from the point under the top pivot
SWING0 = -85.0  # pivot panel's start angle from closed (negative: out into the room)

# Door mechanism frame: origin on the pivot axis at the door's top, with the door seated.
DOOR_TOP_SEATED = SEAT_BOTTOM + H
O = np.array([XP, YP, TOP + DOOR_TOP_SEATED])
PIN_TIP = np.array([0.0, 0.0, -(H + STEM + WHEEL_T + PIN_L)])  # in the door frame, seated: tip at SEAT_TIP


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _quat_xyzw(r):
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
        q = [0.0, 0.0, 0.0, (r[k, j] - r[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (r[j, i] + r[i, j]) / s
        q[k] = (r[k, i] + r[i, k]) / s
    q = np.array(q)
    return tuple(float(v) for v in q / np.linalg.norm(q))


def _kick():
    """The door frame's starting rotation about the top pivot: the bottom pin out at (KICK_X, KICK_Y)."""
    L = -PIN_TIP[2]
    b = math.asin(-KICK_X / L)  # about y: the bottom toward -x
    a = math.asin(KICK_Y / (L * math.cos(b)))  # about x: the bottom toward -y
    return _rx(a) @ _ry(b)


KICK_R = _kick()
# Lower the frame so that the tilted pin's tip stands 0.5 mm above the floor; it settles onto it.
_tip0 = KICK_R @ PIN_TIP
O_START = O + np.array([0.0, 0.0, (TOP + 0.0005) - (O[2] + _tip0[2])])
Q_SEAT = (TOP + SEAT_TIP) - O_START[2] - PIN_TIP[2]  # lift joint with the door plumb and its wheel down on the rails


# -- door geometry, built closed (both panels across the opening, in the frame), then folded and swung ------------------

def _slab(y0, y1, z0=-H, z1=0.0, x0=-T / 2, x1=T / 2):
    return m.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


HINGE_AXIS_CLOSED = np.array([T / 2, -(W - PIV_IN), 0.0])  # on the back faces, at the panels' shared edge
B_Y0, B_Y1 = -(W - PIV_IN) - HINGE_GAP - W, -(W - PIV_IN) - HINGE_GAP
GUIDE_CLOSED = np.array([0.0, B_Y0 + PIV_IN, 0.0])
GUIDE_R, GUIDE_UP = 0.008, 0.0185  # guide roller, standing this far above the lead panel's top


def _fold_point(p, fold_deg):
    """A point of the lead panel (closed frame) after folding by fold_deg about the hinge axis."""
    return _rz(math.radians(fold_deg)) @ (np.asarray(p, float) - HINGE_AXIS_CLOSED) + HINGE_AXIS_CLOSED


def _guide_x(fold_deg, swing_deg):
    return float((_rz(math.radians(swing_deg)) @ _fold_point(GUIDE_CLOSED, fold_deg))[0])


def _solve_fold(swing_deg):
    lo, hi = 90.0, 180.0
    f_lo = _guide_x(lo, swing_deg)
    for _ in range(80):
        mid = (lo + hi) / 2
        f = _guide_x(mid, swing_deg)
        if (f > 0) == (f_lo > 0):
            lo, f_lo = mid, f
        else:
            hi = mid
    return (lo + hi) / 2


FOLD0 = _solve_fold(SWING0)
R_SWING0 = _rz(math.radians(SWING0))


def _pose_a(mesh):
    return m.Mesh(mesh.v @ R_SWING0.T, mesh.f)


def _pose_b(mesh):
    r = _rz(math.radians(FOLD0))
    v = (mesh.v - HINGE_AXIS_CLOSED) @ r.T + HINGE_AXIS_CLOSED
    return m.Mesh(v @ R_SWING0.T, mesh.f)


def _panel_faces(y0, y1, prefix, look_line):
    """A full-louvre face on both sides, drawn on the panel's solid slab: a frame line 5.7 cm in from the edges (the
    stiles), a 6 cm top rail and a 15 cm bottom rail, and the slats' shadow lines every 3.2 cm between them."""
    out = []
    stile, top_rail, bottom_rail, pitch, g = 0.057, 0.06, 0.15, 0.032, 0.004
    zb, zt = -H + bottom_rail, -top_rail
    for side, x in (("f", -T / 2 - 0.0004), ("b", T / 2 + 0.0004)):
        ya, yb = y0 + stile, y1 - stile
        lines = [m.box((0.0008, g, zt - zb), (x, ya, (zb + zt) / 2)), m.box((0.0008, g, zt - zb), (x, yb, (zb + zt) / 2)),
                 m.box((0.0008, yb - ya, g), (x, (ya + yb) / 2, zt)), m.box((0.0008, yb - ya, g), (x, (ya + yb) / 2, zb))]
        z = zb + pitch
        while z < zt - 0.01:
            lines.append(m.box((0.0008, yb - ya, 0.006), (x, (ya + yb) / 2, z)))
            z += pitch
        out.append((f"{prefix}{side}", m.join(*lines), look_line))
    return out


def _pivot_panel_link():
    y0, y1 = -(W - PIV_IN), PIV_IN
    slab = _slab(y0, y1)
    parts = [("slab", _pose_a(slab), "door")]
    parts += [(n, _pose_a(mesh), look) for n, mesh, look in _panel_faces(y0, y1, "pf", "groove")]
    zb = -H
    hw = [("stem", m.cylinder(0.004, STEM + 0.001, (0, 0, zb - STEM / 2 + 0.0005)), "steel"),
          ("wheel", m.cylinder(WHEEL_R, WHEEL_T, (0, 0, zb - STEM - WHEEL_T / 2), segments=24), "nylon"),
          ("pin", m.lathe([(0.0, zb - STEM - WHEEL_T - PIN_L), (PIN_R - PIN_CHAMFER, zb - STEM - WHEEL_T - PIN_L),
                           (PIN_R, zb - STEM - WHEEL_T - PIN_L + PIN_CHAMFER), (PIN_R, zb - STEM - WHEEL_T), (0.0, zb - STEM - WHEEL_T)], 20), "steel"),
          ("toppin", m.cylinder(0.0048, 0.012, (0, 0, 0.006)), "steel")]
    parts += hw
    # hinge leaves on the back faces at the panels' shared edge, 11 in from the top and bottom
    hx, hy = HINGE_AXIS_CLOSED[0], HINGE_AXIS_CLOSED[1]
    for k, z in enumerate((-0.28, -H + 0.28)):
        parts.append((f"hinge{k}", _pose_a(m.box((0.002, 0.025, 0.075), (hx + 0.001, hy + 0.0125, z))), "steel"))
        parts.append((f"knuckle{k}", _pose_a(m.cylinder(0.0035, 0.075, (hx, hy - HINGE_GAP / 2, z), segments=12)), "steel"))
    cols = [m.col_hull("slab_c", _pose_a(slab)),
            m.col_hull("wheel_c", m.cylinder(WHEEL_R, WHEEL_T + STEM, (0, 0, zb - (STEM + WHEEL_T) / 2), segments=16)),
            m.col_hull("pin_c", m.lathe([(0.0, zb - STEM - WHEEL_T - PIN_L), (PIN_R - PIN_CHAMFER, zb - STEM - WHEEL_T - PIN_L),
                                         (PIN_R, zb - STEM - WHEEL_T - PIN_L + PIN_CHAMFER), (PIN_R, zb - STEM - WHEEL_T + 0.0005), (0.0, zb - STEM - WHEEL_T + 0.0005)], 16))]
    com = R_SWING0 @ np.array([0.0, (y0 + y1) / 2, -H / 2])
    return m.hinge("pivot_panel", "Z", (0.0, 0.0, 0.0), -10.0 + 0.0, 85.0, parts, cols, mass=PANEL_MASS, gravity=True,
                   com=tuple(float(v) for v in com), inertia=_panel_inertia(), friction=0.0, damping=0.4, parent="lift")


def _lead_panel_link():
    y0, y1 = B_Y0, B_Y1
    slab = _slab(y0, y1)
    parts = [("slab", _pose_b(slab), "door")]
    parts += [(n, _pose_b(mesh), look) for n, mesh, look in _panel_faces(y0, y1, "lf", "groove")]
    knob_c = np.array([-T / 2, (y0 + y1) / 2, -H + 36.0 * IN])
    knob = m.lathe([(0.0, 0.0), (0.008, 0.0), (0.008, 0.012), (0.016, 0.020), (0.016, 0.030), (0.0, 0.032)], 24).aimed("-X")
    parts.append(("knob", _pose_b(m.Mesh(knob.v + knob_c, knob.f)), "knob"))
    guide = m.cylinder(GUIDE_R, GUIDE_UP, (GUIDE_CLOSED[0], GUIDE_CLOSED[1], GUIDE_UP / 2), segments=16)
    parts.append(("guide", _pose_b(guide), "nylon"))
    cols = [m.col_hull("slab_c", _pose_b(slab)), m.col_hull("guide_c", _pose_b(guide)),
            m.col_hull("knob_c", _pose_b(m.Mesh(knob.v + knob_c, knob.f)))]
    com = R_SWING0 @ _fold_point(np.array([0.0, (y0 + y1) / 2, -H / 2]), FOLD0)
    at = R_SWING0 @ HINGE_AXIS_CLOSED
    return m.hinge("lead_panel", "Z", tuple(float(v) for v in at), -FOLD0, 179.0 - FOLD0, parts, cols, mass=PANEL_MASS, gravity=True,
                   com=tuple(float(v) for v in com), inertia=_panel_inertia(), friction=0.0, damping=0.3, parent="pivot_panel")


PANEL_MASS = 6.90 / 2  # kg each: the real pair's listed 15.2 lb (6.90 kg), README


def _panel_inertia():
    return (PANEL_MASS * (H ** 2 + T ** 2) / 12, PANEL_MASS * (H ** 2 + W ** 2) / 12, PANEL_MASS * (W ** 2 + T ** 2) / 12)


def _door():
    links = [
        m.hinge("tilt_x", "X", (0.0, 0.0, 0.0), -5.0, 5.0, (), (), mass=0.05, inertia=(1e-3, 1e-3, 1e-3), friction=0.0, damping=0.5),
        m.hinge("tilt_y", "Y", (0.0, 0.0, 0.0), -5.0, 5.0, (), (), mass=0.05, inertia=(1e-3, 1e-3, 1e-3), friction=0.0, damping=0.5, parent="tilt_x"),
        # the door slides up the spring-loaded top pivot pin until its top meets the track
        m.slider("lift", "Z", -0.003, Q_SEAT + TOP_GAP, (), (), mass=0.05, inertia=(1e-3, 1e-3, 1e-3), spring=SPRING_K, damping=6.0, friction=0.0,
                 effort=80.0, parent="tilt_y"),
        _pivot_panel_link(),
        _lead_panel_link(),
    ]
    looks = {"door": m.Look(m.rgb(0.93, 0.93, 0.91), roughness=0.55), "groove": m.Look(m.rgb(0.80, 0.80, 0.78), roughness=0.6),
             "steel": m.Look(m.rgb(0.70, 0.71, 0.73), metallic=0.8, roughness=0.35), "nylon": m.Look(m.rgb(0.92, 0.92, 0.88), roughness=0.5),
             "knob": m.Look(m.rgb(0.75, 0.62, 0.38), metallic=0.7, roughness=0.3)}
    return m.mechanism("door", tuple(float(v) for v in O_START), links, looks, solver_iters=48, start_quat=_quat_xyzw(KICK_R),
                       friction=(0.9, 0.8), frame_inertia=(0.01, 0.01, 0.01))


# -- the closet: front wall with its opening, the track, the jamb bracket ----------------------------------------------

def _closet():
    looks = {"wall": m.Look(m.rgb(0.86, 0.84, 0.80), roughness=0.8), "trim": m.Look(m.rgb(0.95, 0.95, 0.93), roughness=0.5),
             "inside": m.Look(m.rgb(0.78, 0.76, 0.72), roughness=0.85), "alu": m.Look(m.rgb(0.78, 0.79, 0.80), metallic=0.8, roughness=0.35),
             "steel": m.Look(m.rgb(0.62, 0.63, 0.65), metallic=0.8, roughness=0.4)}
    parts, cols = [], []
    top = OPEN_H
    tall = 2.40
    y_left_end, y_right_end = 0.65, -0.65
    x0, x1 = XW, XW + WALL_T
    for name, ya, yb in (("wall_l", YL, y_left_end), ("wall_r", y_right_end, YR)):
        b = m.box((x1 - x0, yb - ya, tall), ((x0 + x1) / 2, (ya + yb) / 2, tall / 2))
        parts.append((name, b, "wall"))
        cols.append(m.col_box(name + "_c", (x1 - x0, yb - ya, tall), ((x0 + x1) / 2, (ya + yb) / 2, tall / 2)))
    hb = (x1 - x0, YL - YR, tall - top)
    parts.append(("header", m.box(hb, ((x0 + x1) / 2, (YL + YR) / 2, top + hb[2] / 2)), "wall"))
    cols.append(m.col_box("header_c", hb, ((x0 + x1) / 2, (YL + YR) / 2, top + hb[2] / 2)))
    # casing trim round the opening on the room face
    for name, size, c in (("casing_l", (0.012, 0.064, top + 0.064), (x0 - 0.006, YL + 0.032, (top + 0.064) / 2)),
                          ("casing_r", (0.012, 0.064, top + 0.064), (x0 - 0.006, YR - 0.032, (top + 0.064) / 2)),
                          ("casing_t", (0.012, YL - YR + 0.128, 0.064), (x0 - 0.006, (YL + YR) / 2, top + 0.032)),
                          ("base_l", (0.012, y_left_end - YL - 0.064, 0.09), (x0 - 0.006, (YL + 0.064 + y_left_end) / 2, 0.045)),
                          ("base_r", (0.012, YR - 0.064 - y_right_end, 0.09), (x0 - 0.006, (YR - 0.064 + y_right_end) / 2, 0.045))):
        parts.append((name, m.box(size, c), "trim"))
        cols.append(m.col_box(name + "_c", size, c))
    # inside of the closet: back and side walls (shallower than a real 24 in closet: the bench ends at x 0.91)
    for name, size, c in (("back", (0.02, YL - YR + 0.30, tall), (X_BACK, (YL + YR) / 2, tall / 2)),
                          ("side_l", (X_BACK - x1, 0.02, tall), ((x1 + X_BACK) / 2, YL + 0.15, tall / 2)),
                          ("side_r", (X_BACK - x1, 0.02, tall), ((x1 + X_BACK) / 2, YR - 0.15, tall / 2))):
        parts.append((name, m.box(size, c), "inside"))
    # the track under the head jamb: a 1 x 1 in aluminium channel open at the bottom, centred on XP
    ty0, ty1 = YR + 0.003, YL - 0.003
    ly, lc = ty1 - ty0, (ty0 + ty1) / 2
    zt0 = top - TRACK_H
    inner = 0.0112
    parts.append(("track_roof", m.box((0.0254, ly, 0.0015), (XP, lc, top - 0.00075)), "alu"))
    for s in (-1, 1):
        c = (XP + s * (inner + 0.00075), lc, zt0 + TRACK_H / 2)
        parts.append((f"track_side{s + 1}", m.box((0.0015, ly, TRACK_H), c), "alu"))
        cols.append(m.col_box(f"track_side{s + 1}_c", (0.0015, ly, TRACK_H), c))
    parts.append(("pivot_bracket", m.box((2 * inner - 0.001, 0.04, 0.012), (XP, YP, top - 0.0015 - 0.006)), "steel"))
    # jamb bracket: a floor plate with a slot along the opening and toothed rails both sides of it, an upturned leg on the jamb
    yb0, yb1 = YL - BRACKET_L, YL
    s0, s1 = YL - SLOT_Y1, YL - SLOT_Y0
    half = SLOT_W / 2
    for name, (ya, yb_), (xa, xb) in (("plate_f", (yb0, yb1), (XP - BRACKET_D / 2, XP - half)),
                                      ("plate_b", (yb0, yb1), (XP + half, XP + BRACKET_D / 2)),
                                      ("plate_e0", (yb0, s0), (XP - half, XP + half)),
                                      ("plate_e1", (s1, yb1), (XP - half, XP + half))):
        size, c = (xb - xa, yb_ - ya, PLATE_T), ((xa + xb) / 2, (ya + yb_) / 2, PLATE_T / 2)
        parts.append((name, m.box(size, c), "steel"))
        cols.append(m.col_box(name + "_c", size, c))
    for s in (-1, 1):  # rails, their inner top edges rounded off 1 mm
        xi = XP + s * half
        xo = xi + s * RAIL_W
        outline = [(xi, PLATE_T), (xo, PLATE_T), (xo, RAIL_TOP), (xi + s * 0.001, RAIL_TOP), (xi, RAIL_TOP - 0.001)]
        if s < 0:
            outline = outline[::-1]
        pr = m.prism(outline, s0 - 0.003, s1 + 0.003)  # outline in (x, z), extruded along y
        mesh = m.Mesh(pr.v[:, [0, 2, 1]], pr.f[:, ::-1])
        hull = m.col_hull(f"rail{s + 1}_c", mesh)
        cols.append(hull)
        parts.append((f"rail{s + 1}", m.box((RAIL_W, s1 - s0 + 0.006, RAIL_H), ((xi + xo) / 2, (s0 + s1) / 2, PLATE_T + RAIL_H / 2)), "steel"))
        for k in range(int((s1 - s0) / 0.003)):  # teeth
            yk = s0 + 0.0015 + 0.003 * k
            parts.append((f"tooth{s + 1}_{k}", m.box((RAIL_W * 0.8, 0.0012, 0.0006), ((xi + xo) / 2, yk, RAIL_TOP + 0.0003)), "steel"))
    leg = (BRACKET_D, PLATE_T, 0.032)
    parts.append(("leg", m.box(leg, (XP, YL - PLATE_T / 2, 0.016)), "steel"))
    parts.append(("mark", m.box((0.0015, 0.0008, 0.0004), (XP - half - RAIL_W - 0.001, YP, PLATE_T + 0.0002)), "trim"))
    return m.fixed("closet", (0.0, 0.0, TOP), parts, cols, looks, friction=(0.9, 0.8))


OBJECTS = [
    _closet(),
    _door(),
    SimReady("towel", pos=(0.24, -0.45, TOP), size=(0.205, 0.212, 0.034), mass=0.12, yaw=0.3,
             package="consumable_microfiber_towel_folded_small_b01", usd="sm_consumable_microfiber_towel_folded_small_b01_01.usd"),
    SimReady("storage_box", pos=(0.75, -0.30, TOP), size=(0.345, 0.264, 0.166), mass=1.2, yaw=1.5708,
             package="box_cardboard_d01", usd="sm_box_cardboard_d01_01.usd"),
    SimReady("waste_bin", pos=(0.38, 0.52, TOP), size=(0.238, 0.180, 0.280), mass=1.3, yaw=0.0,
             package="bin_waste_a01", usd="sm_bin_waste_a01_01.usd"),
]

# -- reading the door -----------------------------------------------------------------------------------------------------


def _bodies(env):
    asset = env.unwrapped.scene["door"]
    origin = m._np(env.unwrapped.scene.env_origins[0])
    poses = m._np(asset.data.body_link_pose_w)[0]
    vels = m._np(asset.data.body_link_vel_w)[0]
    out = {}
    for name in ("pivot_panel", "lead_panel"):
        i = asset.body_names.index(name)
        out[name] = (poses[i, :3] - origin, m.quat_matrix(poses[i, 3:7]), vels[i, :3], vels[i, 3:6])
    return out


def pin_tip_world(st):
    """World position of the bottom pivot pin's tip, from the door's pose and its tilt and lift joints (what a camera sees)."""
    r = m.quat_matrix(st.quat_xyzw) @ _rx(st.joint("tilt_x")) @ _ry(st.joint("tilt_y"))
    return np.asarray(st.pos, float) + r @ (PIN_TIP + np.array([0.0, 0.0, st.joint("lift")]))


# -- scoring -------------------------------------------------------------------------------------------------------------
SUCCESS_HOLD_S = 0.0  # the tracker counts its own hold
HOLD_S = 2.0
PLUMB_TOL = 0.004  # pin centre within 4 mm of the mark under the top pivot
SLOT_X_TOL = 0.0015
SEAT_Z_MAX = SEAT_TIP + 0.0015  # pin tip this low: the wheel is down on the rails
OVER_X_TOL = 0.006
CLEAR = 0.03
STILL_V, STILL_W = 0.005, 0.01
GUIDE_OUT = 0.014  # guide centre this far off the track's centre line: out of the track


def _panel_box_dist(local, y0, y1):
    lo = np.array([-T / 2, y0, -H])
    hi = np.array([T / 2, y1, 0.0])
    return np.linalg.norm(np.maximum(np.maximum(lo - local, local - hi), 0.0), axis=1)


def hands_gap(env):
    """Smallest distance (m) from any sampled finger point to either panel (both as their full slabs)."""
    tips = m.fingertips(env)
    b = _bodies(env)
    p, r, _, _ = b["pivot_panel"]
    la = ((tips - p) @ r) @ R_SWING0  # into the pivot panel's closed frame
    d = _panel_box_dist(la, -(W - PIV_IN), PIV_IN)
    p, r, _, _ = b["lead_panel"]
    lb = ((tips - p) @ r) @ R_SWING0
    rf = _rz(math.radians(FOLD0))
    lb = (lb - HINGE_AXIS_CLOSED) @ rf + HINGE_AXIS_CLOSED
    d = np.minimum(d, _panel_box_dist(lb, B_Y0, B_Y1))
    return float(d.min())


def guide_world(env):
    p, r, _, _ = _bodies(env)["lead_panel"]
    g = R_SWING0 @ _fold_point(GUIDE_CLOSED, FOLD0)
    return p + r @ g


TEETH_ON = SEAT_TIP + 0.0008  # the wheel's teeth mesh with the rails' teeth once it is down this far
TEETH_OFF = SEAT_TIP + 0.0015  # and come free once it is lifted this far


class Teeth:
    """The bracket's teeth: while the wheel is down on the rails inside the slot, its teeth mesh with theirs, and the
    bottom cannot slide along the slot (the tilt joint that moves it along the slot is held where it is). Lifting the
    door frees them, as the makers' instructions describe ("lift door and slide bottom pivot bolt into desired
    position")."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.engaged = False

    def step(self, env):
        tip = pin_tip_world(m.state(env, "door"))
        z = tip[2] - TOP
        in_slot = abs(tip[0] - XP) <= SLOT_X_TOL and (YL - SLOT_Y1 + PIN_R) <= tip[1] <= (YL - SLOT_Y0 - PIN_R)
        if not self.engaged and in_slot and z <= TEETH_ON:
            m.lock(env, "door", "tilt_x")
            self.engaged = True
        elif self.engaged and (z > TEETH_OFF or not in_slot):
            m.unlock(env, "door", "tilt_x")
            self.engaged = False


TEETH = Teeth()
m.on_step(TEETH.step)
m.on_reset(TEETH.reset)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the ONLY place a recorded harm is cleared
        self.harm = False
        self.hold = 0
        self.full = False
        self.value = 0.0
        self.info = {}
        if env is not None:
            m.set_target(env, "door", "lift", Q_SEAT - SPRING_SEATED / SPRING_K)

    def step(self, env):
        st = m.state(env, "door")
        tip = pin_tip_world(st)
        g = guide_world(env)
        if abs(g[0] - XP) > GUIDE_OUT:
            self.harm = True
        dz = tip[2] - TOP
        in_slot_y = (YL - SLOT_Y1 + PIN_R) <= tip[1] <= (YL - SLOT_Y0 - PIN_R)
        seated = bool(abs(tip[0] - XP) <= SLOT_X_TOL and dz <= SEAT_Z_MAX and in_slot_y)
        over = bool(abs(tip[0] - XP) <= OVER_X_TOL and in_slot_y)
        plumb = bool(abs(tip[1] - YP) <= PLUMB_TOL)
        b = _bodies(env)
        still = all(np.linalg.norm(v) < STILL_V and np.linalg.norm(w) < STILL_W for _, _, v, w in b.values())
        gap = hands_gap(env)
        ok = seated and plumb and still and gap > CLEAR and not self.harm
        self.hold = self.hold + 1 if ok else 0
        self.full = bool(self.hold >= round(HOLD_S / env.unwrapped.step_dt))
        if self.harm:
            self.value = 0.0
        elif self.full:
            self.value = 1.0
        elif seated:
            self.value = 0.5
        elif over:
            self.value = 0.25
        else:
            self.value = 0.0
        self.info = dict(tip=tip, seated=seated, over=over, plumb=plumb, still=still, gap=gap, guide=g, lift=st.joint("lift"))


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


# -- embodiment: two i2rt YAM Ultra (v2) arms --------------------------------------------------------------------------
# yam_arena models the standard YAM (DM4340 motors at joints 1-3, DM4310 at 4-6). The YAM Ultra v2 has the same
# kinematics to within a few millimetres, drives joint 4 with a DM4340 too and has heavier links 2 and 3: i2rt's
# i2rt/robots/config/yam_ultra_v2.yml (motors, gains) and i2rt/robot_models/arm/yam_ultra/v2/README.md (link masses).
# The task applies those to both arms at every reset, in the simulation only: joint 4 gets the DM4340's limits that
# yam_arena already uses for joints 1-3 (27 Nm peak, 5.5 rad/s) and Ultra v2's gains (kp 40, kd 3). It also caps each
# finger at 40 N (yam_arena allows i2rt's driver limit of 50 N).
J4 = dict(effort=27.0, velocity=5.5, stiffness=40.0, damping=3.0)
FINGER_CAP = 40.0  # N a finger
ULTRA_MASS = {"base": 0.940115, "link1": 0.101218, "link2": 1.63122, "link3": 1.06053, "link4": 0.46079, "link5": 0.403307}


def _ultra_arms(env=None):
    if env is None:
        return
    import torch
    base = env.unwrapped
    for side in ("left", "right"):
        r = base.scene[f"{side}_robot"]
        j4 = r.find_joints("joint4")[0]
        one = lambda v: torch.full((1, 1), float(v), device=r.device)  # noqa: E731
        r.write_joint_effort_limit_to_sim_index(limits=one(J4["effort"]), joint_ids=j4)
        r.write_joint_velocity_limit_to_sim_index(limits=one(J4["velocity"]), joint_ids=j4)
        r.write_joint_stiffness_to_sim_index(stiffness=one(J4["stiffness"]), joint_ids=j4)
        r.write_joint_damping_to_sim_index(damping=one(J4["damping"]), joint_ids=j4)
        names = list(ULTRA_MASS)
        ids = [r.find_bodies(n)[0][0] for n in names]
        masses = torch.tensor([[ULTRA_MASS[n] for n in names]], dtype=torch.float32, device=r.data.body_mass.torch.device)
        r.set_masses_index(masses=masses, body_ids=ids)
        fingers = r.find_joints("joint[7-8]")[0]
        r.write_joint_effort_limit_to_sim_index(limits=torch.full((1, len(fingers)), FINGER_CAP, device=r.device), joint_ids=fingers)


m.on_reset(_ultra_arms)


def score(objects):
    return float(TRIAL.value)


def success(objects):
    return bool(TRIAL.full)


# -- the scripted demonstration (closed loop: demokit.py) -----------------------------------------------------------------


def _door_points(st):
    """Functions from authored door-frame points to world points, for the pivot panel and the lead panel, from the
    door's visible pose (its frame and joints), as a camera-based policy could estimate them."""
    base = np.asarray(st.pos, float)
    r = m.quat_matrix(st.quat_xyzw) @ _rx(st.joint("tilt_x")) @ _ry(st.joint("tilt_y"))
    lift = np.array([0.0, 0.0, st.joint("lift")])
    rs = _rz(st.joint("pivot_panel"))
    hinge = R_SWING0 @ HINGE_AXIS_CLOSED
    rf = _rz(st.joint("lead_panel"))

    def a(p):
        return base + r @ (rs @ np.asarray(p, float) + lift)

    def b(p):
        return a(rf @ (np.asarray(p, float) - hinge) + hinge)

    return a, b, r @ rs


# -- the held door's weight, fed forward to the arm commands -----------------------------------------------------------
# The embodiment takes joint position targets, and its joints hold a load only by lagging their targets (torque = gain
# x lag). Once the hands have the door, the demo adds to each arm's target the lag that its half of the door's weight
# needs, from a static model of the arm on the URDF's kinematics, as a load-compensating controller would.
KP_ARM = np.array([80.0, 80.0, 80.0, J4["stiffness"], 10.0, 10.0])
_STATIC = {}


def load_torque(q, force):
    """Joint torques an arm at joints `q` needs to hold `force` (N, world frame) pushing on its TCP."""
    from yam_arena.urdf import forward_kinematics
    if not _STATIC:
        from yam_arena import YAM_URDF
        from yam_arena.constants import ARM_JOINTS, TCP_OFFSET
        from yam_arena.urdf import parse_joints
        _STATIC.update(urdf=YAM_URDF, joints=parse_joints(YAM_URDF), names=ARM_JOINTS, tcp=np.array(TCP_OFFSET))
    st = _STATIC
    P = forward_kinematics(st["urdf"], {n: float(v) for n, v in zip(st["names"], q)})
    tcp = P["gripper"][:3, :3] @ st["tcp"] + P["gripper"][:3, 3]
    tau = np.zeros(6)
    for i, n in enumerate(st["names"]):
        j = st["joints"][n]
        o = P[j.child][:3, 3]
        ax = P[j.child][:3, :3] @ (j.axis / np.linalg.norm(j.axis))
        tau[i] = -np.cross(tcp - o, np.asarray(force, float)) @ ax
    return tau


GRIP_UP = {"left": 0.14, "right": 0.34}  # each hand's grip height above the door's bottom edge
GRIP_IN = 0.026  # TCP this far in from the panels' front (hinge) edge
OVER = 0.0085  # pin tip this high while carried over the rails (rail tops at 5 mm)
SERVO_GAIN, SERVO_STEP = 0.2, 0.004
LIFT_S, UNLOAD_S = 1.5, 1.2  # the hands take the door's weight, and give it to the bracket, over these times


def demo_script(ctx):
    """Both hands pinch the folded panels at their hinged edge, one low and one higher up, as a person takes a door at
    its edge with both hands. The left hand steers the bottom pin by what it sees of the door; the right hand moves
    with it, so that both carry the door. Mode "onearm" does the same with the left hand alone."""
    import demokit as d

    mode = ctx.mode
    sides = ("left",) if mode == "onearm" else ("left", "right")
    arms = [ctx.arm(sd) for sd in sides]

    def door():
        return ctx.state("door")

    def tip():
        return pin_tip_world(door())

    def grip_point(side):
        a, _, rot = _door_points(door())
        c = np.array([T / 2, -(W - PIV_IN) + GRIP_IN, -H + GRIP_UP[side]])
        p = a(R_SWING0 @ c)
        inward = rot @ (R_SWING0 @ np.array([0.0, 1.0, 0.0]))  # along the panels, toward the pivot
        return p, inward

    def together(gens):
        yield from ctx.together(*gens)

    def carry(goal, tol, timeout, label):
        """Shift both hands' commands together until the pin's tip is at `goal` (closed loop on the door's pose)."""
        c0 = {a.side: d.ik.tcp_pose(a.q, a.base)[0] for a in arms}
        shift = np.zeros(3)
        good = 0
        for _ in range(max(1, round(timeout * d.CONTROL_HZ))):
            err = np.asarray(goal, float) - tip()
            good = good + 1 if np.linalg.norm(err) < tol else 0
            if good >= 3:
                return True
            shift = shift + np.clip(SERVO_GAIN * err, -SERVO_STEP, SERVO_STEP)
            for a in arms:
                mm, deg = a._solve(c0[a.side] + shift, a.grip.rot)
                a._note(mm, deg, label)
            yield
        print(f"[demo] '{label}': stopped {np.linalg.norm(err) * 1000:.1f} mm short", flush=True)
        return False

    if mode == "none":
        yield from ctx.until(lambda: False, timeout=20.0)
        return

    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import ik as _ik

    share = {"left": 0.0, "right": 0.0}  # N of the door's weight each hand is set to carry
    total = 2 * PANEL_MASS * 9.81 + SPRING_SEATED + SPRING_K * (OVER + 0.004)  # the door and the pivot spring, lifted

    def action():
        cmds = []
        for sd in ("left", "right"):
            c = np.array(ctx.arm(sd).cmd, dtype=float)
            if share[sd] > 0.0:
                c[:6] = np.clip(c[:6] + load_torque(c[:6], (0.0, 0.0, -share[sd])) / KP_ARM, _ik.LOWER, _ik.UPPER)
            cmds.append(c)
        return Action(data=pack(cmds[0], cmds[1]))

    ctx.action = action

    def ramp(to, t):
        """Ramp each hand's share to `to` times its half of the load."""
        start = dict(share)
        n = max(1, round(t * d.CONTROL_HZ))
        for k in range(1, n + 1):
            for a in arms:
                share[a.side] = start[a.side] + (to * total / len(arms) - start[a.side]) * k / n
            yield
    _, inward = grip_point("left")
    phi = math.degrees(math.atan2(inward[1], inward[0]))
    g = d.level(phi)
    yield from together([a.move(grip_point(a.side)[0] - 0.07 * inward + np.array([0.0, 0.0, 0.03]), grip=g, finger=d.FINGER_OPEN, t=2.5, label="in front of the folded door") for a in arms])
    yield from together([a.move(grip_point(a.side)[0] - 0.07 * inward, t=0.8, label="to grip height") for a in arms])
    yield from together([a.servo(grip_point(a.side)[0] - 0.07 * inward, timeout=1.0, label="line up") for a in arms])
    yield from together([a.move(grip_point(a.side)[0], t=1.5, label="fingers round the panels' edges") for a in arms])
    yield from together([a.servo(grip_point(a.side)[0], timeout=1.0, label="round the edges") for a in arms])
    yield from together([a.close(width=2 * T, force=40.0, label="pinch both panels") for a in arms])
    yield from ctx.left.wait(0.3)
    t0 = tip()
    if mode == "drag":  # careless: push the bottom back toward the bracket without lifting it
        yield from carry(np.array([XP, YP, t0[2]]), 0.002, 6.0, "drag the pin")
    else:
        yield from ramp(1.0, LIFT_S)
        yield from carry(t0 + np.array([0.0, 0.0, OVER + 0.002 - (t0[2] - TOP)]), 0.0015, 3.0, "lift the door on its top pivot spring")
        lane = np.array([XP - 0.006, t0[1] if mode == "straight_back" else YP, TOP + OVER + 0.002])
        yield from carry(lane, 0.002, 5.0, "carry the pin back toward the bracket")
        if mode != "drop_early":  # drop_early: the cheapest shortcut, letting go once the pin is roughly back
            over = lane + np.array([0.006, 0.0, 0.0])
            yield from carry(over, 0.0012, 4.0, "pin over the slot at the mark")
            yield from ctx.left.wait(0.4)
            yield from carry(over, 0.0010, 2.0, "pin over the slot at the mark")
            yield from carry(np.array([over[0], over[1], TOP + SEAT_TIP + 0.0003]), 0.0008, 3.0, "lower the pin into the slot")
            # unload the arms before letting go: command them down until the door's weight is on the bracket, so that
            # their sag does not spring back up and drag the panels with them while the fingers open
            yield from together([a.move(a.at() + np.array([0.0, 0.0, -0.008]), t=UNLOAD_S, label="let the bracket take the weight") for a in arms] + [ramp(0.0, UNLOAD_S)])
            yield from ctx.left.wait(0.4)
            if mode == "hold":  # seat it but keep holding
                yield from ctx.until(lambda: False, timeout=8.0)
                return
    yield from ramp(0.0, 0.3)  # nothing to carry once the fingers open
    yield from together([a.open(t=0.4, label="let go") for a in arms])
    yield from together([a.move(a.at() - 0.10 * inward, t=1.5, label="back off along the panels") for a in arms])
    yield from together([a.move(a.at() + np.array([-0.02, 0.06 if a.side == "left" else -0.06, 0.10]), t=1.5, label="away") for a in arms])
    yield from ctx.left.wait(4.0)
