"""The bodies and joints of the hot test-tube transfer task, built from parts.py's figures:
- ``mat``: blue TRU COMPONENTS silicone work mat (fixed on the bench);
- ``bath``: FOUR E'S TC0401001 dry-bath incubator with sloped front ``95.0`` LED readout and recessed 96 x 76 mm
  aluminium block carrying the chamfered 45 mm deep bore;
- ``rack``: BOCHEM 8291 18/10 stainless-steel Z-shape cooling rack with all 10 x 5 = 50 chamfered 18.4 mm apertures
  across upper and middle guide tiers over a solid floor;
- ``tube``: DWK PYREX 1620/09M 16 x 100 mm borosilicate rimmed test tube standing in the dry-bath well;
- ``long_arm`` and ``short_lever``: Eisco CH0726B 260.4 mm self-closing wooden clothespin holder joined by the
  preloaded torsion hinge ``jaw_h``, with force-gated anti-creep barrel weld ``w_grip`` (no computed push).
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as mk
import parts as P
from mechkit import body, box, cylinder, hinge, weld

MECHKIT_HELPER = True  # declares on behalf of task.py

MAT_BLUE = (0.11, 0.31, 0.58)
CASE_WHITE = (0.90, 0.91, 0.93)
BEZEL_DARK = (0.10, 0.11, 0.13)
KNOB_BLUE = (0.16, 0.44, 0.80)
LED_RED = (1.00, 0.14, 0.06)
ALU_BLOCK = (0.44, 0.46, 0.50)
WELL_DARK = (0.16, 0.17, 0.19)
STAINLESS = (0.68, 0.71, 0.75)
STAINLESS_EDGE = (0.56, 0.59, 0.63)
GLASS = (0.80, 0.89, 0.95)
GLASS_RIM = (0.88, 0.94, 0.98)
PYREX_PATCH = (0.97, 0.97, 0.99)
SAMPLE_AMBER = (0.86, 0.46, 0.09)
BEECH = (0.82, 0.64, 0.41)
BEECH_JAW = (0.74, 0.56, 0.34)
STEEL_SPRING = (0.28, 0.30, 0.33)

HOT_SHAPES: tuple[int, ...] = ()  # populated by build_bath(): solid shape indices of the 96 x 76 mm hot block


def _floats(v) -> tuple[float, ...]:
    return tuple(float(x) for x in v)


def _quat(matrix: np.ndarray) -> tuple[float, ...]:
    return tuple(float(v) for v in mk.matrix_to_quat(matrix))


# --- 7-segment "95.0" LED digits on the sloped front bezel of the dry bath ------------------------------------------
# Segments in the bezel plane: u right (along -y in world when facing -x), v up along the slope.
_SEGMENTS_7 = {
    "9": ("t", "tl", "tr", "m", "br", "b"),
    "5": ("t", "tl", "m", "br", "b"),
    "0": ("t", "tl", "tr", "bl", "br", "b"),
}


def _digit_boxes(cx: float, cy_bezel: float, w: float, h: float, t: float, segs: tuple[str, ...]) -> list[ tuple[tuple[float, float, float], tuple[float, float]] ]:
    """List of ((sx, sy, sz), (du, dv)) in bezel coordinates for one 7-segment LED character."""
    out = []
    hw, hh = w / 2.0, h / 2.0
    for s in segs:
        if s == "t":
            out.append(((t, w, t), (cx, cy_bezel + hh)))
        elif s == "m":
            out.append(((t, w, t), (cx, cy_bezel)))
        elif s == "b":
            out.append(((t, w, t), (cx, cy_bezel - hh)))
        elif s == "tl":
            out.append(((t, t, hh), (cx - hw, cy_bezel + hh / 2.0)))
        elif s == "tr":
            out.append(((t, t, hh), (cx + hw, cy_bezel + hh / 2.0)))
        elif s == "bl":
            out.append(((t, t, hh), (cx - hw, cy_bezel - hh / 2.0)))
        elif s == "br":
            out.append(((t, t, hh), (cx + hw, cy_bezel - hh / 2.0)))
    return out


def build_mat(top: float) -> None:
    body(
        "mat",
        [box((P.MAT_L, P.MAT_W, P.MAT_T), color=MAT_BLUE)],
        pos=(P.MAT_XY[0], P.MAT_XY[1], top + P.MAT_T / 2.0),
        color=MAT_BLUE,
        friction=mk.GRIPPY,
    )


def build_bath(top: float) -> None:
    """FOUR E'S TC0401001 dry bath: polymer shell + recessed 96 x 76 mm aluminium heating block + sloped 95.0 C LED."""
    global HOT_SHAPES
    bx, by = P.BATH_XY
    # Local frame of bath: at (BATH_XY[0], BATH_XY[1], top), z = 0 on bench, z = BATH_H at top.
    # Hot block cavity in local frame: hx in [hx0 - 0.038, hx0 + 0.038], hy in [-0.048, +0.048]
    hx0 = P.HOT_BLOCK_XY[0] - bx  # +0.015 m
    hy0 = P.HOT_BLOCK_XY[1] - by  # 0.000 m
    hd, hw = P.HOT_BLOCK_D, P.HOT_BLOCK_W  # 0.076, 0.096 m

    shapes = []
    # Outer white polymer housing walls around the 76 x 96 mm block recess:
    front_d = (hx0 - hd / 2.0) - (-P.BATH_D / 2.0)  # (-0.023) - (-0.095) = 0.072 m front section!
    rear_d = (P.BATH_D / 2.0) - (hx0 + hd / 2.0)  # 0.095 - 0.053 = 0.042 m rear rim
    side_w = (P.BATH_W - hw) / 2.0  # (0.152 - 0.096) / 2 = 0.028 m side cheeks

    # Front lower housing + sloped front bezel so top_cam reads "95.0" head-on:
    shapes.append(box((front_d, P.BATH_W, 0.054), pos=(-P.BATH_D / 2.0 + front_d / 2.0, 0.0, 0.027), color=CASE_WHITE))
    shapes.append(box((0.036, P.BATH_W, P.BATH_H), pos=(hx0 - hd / 2.0 - 0.018, 0.0, P.BATH_H / 2.0), color=CASE_WHITE))
    bezel_tilt = math.radians(36.0)  # normal points (-cos 36, 0, +sin 36) -> directly toward top_cam!
    bezel_rot = mk.axis_angle((0.0, 1.0, 0.0), bezel_tilt)
    bezel_q = _quat(bezel_rot)
    bezel_c = np.array([-0.074, 0.0, 0.066])
    shapes.append(box((0.012, P.BATH_W - 0.004, 0.042), pos=_floats(bezel_c), quat=bezel_q, color=CASE_WHITE))
    # Dark acrylic LED window on the sloped bezel (solid=False: visual readout only, unclipped from top_cam):
    win_c = bezel_c + np.array([-0.006, 0.0, -0.006]) + bezel_rot @ np.array([-0.0075, 0.014, 0.002])
    shapes.append(box((0.002, 0.068, 0.026), pos=_floats(win_c), quat=bezel_q, color=BEZEL_DARK, solid=False))
    # Blue rotary TEMP/TIMER knob to the right (-y) of the display:
    knob_c = bezel_c + np.array([-0.006, 0.0, -0.006]) + bezel_rot @ np.array([-0.010, -0.044, 0.000])
    shapes.append(cylinder(0.011, 0.008, pos=_floats(knob_c), quat=_quat(bezel_rot @ mk.axis_angle((0.0, 1.0, 0.0), math.pi / 2.0)), color=KNOB_BLUE, solid=False))
    # Glowing red 7-segment "95.0" + red HOT indicator lamp on the window:
    # Note: in bezel frame, local -x points outward toward camera, +y points left (+y world -> right in image? Wait:
    # top_cam sits at x = -0.166 looking +x, so +y world is LEFT in the camera image!
    # Therefore reading left-to-right in top_cam goes from +y down to -y!)
    seg_specs = []
    for ch, u_left_to_right in (("9", 0.022), ("5", 0.008), ("0", -0.010)):
        # Map u_left_to_right (+ = left in image = +y world): note in _digit_boxes cx+hw was right, so flip sign of dx_u:
        for (sx, sy, sz), (du, dv) in _digit_boxes(0.0, 0.0, 0.009, 0.016, 0.0022, _SEGMENTS_7[ch]):
            y_pos = u_left_to_right - du  # left-to-right in camera = +y to -y in world!
            seg_specs.append(((sx, sy, sz), (y_pos, dv)))
    # Decimal point between '5' and '0' + red HOT indicator dot:
    seg_specs.append(((0.0022, 0.0028, 0.0028), (-0.001, -0.008)))
    seg_specs.append(((0.0022, 0.0050, 0.0050), (-0.025, 0.005)))  # HOT indicator LED
    for dims, (yb, zb) in seg_specs:
        pt = win_c + bezel_rot @ np.array([-0.0016, yb, zb])
        shapes.append(box(dims, pos=_floats(pt), quat=bezel_q, color=LED_RED, solid=False, glow=0.92))

    # Rear and side polymer walls of the housing:
    shapes.append(box((rear_d, P.BATH_W, P.BATH_H), pos=(P.BATH_D / 2.0 - rear_d / 2.0, 0.0, P.BATH_H / 2.0), color=CASE_WHITE))
    for s in (-1.0, 1.0):
        shapes.append(box((hd, side_w, P.BATH_H), pos=(hx0, s * (hw / 2.0 + side_w / 2.0), P.BATH_H / 2.0), color=CASE_WHITE))

    # --- The 96 x 76 mm aluminium heating block (solid indices recorded in HOT_SHAPES) ---
    hot_indices: list[int] = []

    def add_hot(shape: mk.Shape) -> None:
        if shape.solid:
            hot_indices.append(len(shapes))
        shapes.append(shape)

    # Solid aluminium base under the well floor (z = 0 .. WELL_FLOOR_Z = 41 mm):
    add_hot(box((hd, hw, P.WELL_FLOOR_Z), pos=(hx0, hy0, P.WELL_FLOOR_Z / 2.0), color=ALU_BLOCK))
    # Visual dark disc on the well bottom so the bore reads dark from above:
    wx, wy = P.WELL_XY[0] - bx, P.WELL_XY[1] - by  # (-0.005, 0.0)
    shapes.append(cylinder(P.WELL_IN_D / 2.0, 0.0008, pos=(wx, wy, P.WELL_FLOOR_Z + 0.0004), color=WELL_DARK, solid=False))

    # 8-wall octagon bore of exact inscribed diameter WELL_IN_D (18.6 mm >= 17.0 mm) + 8 top chamfer bevels:
    r_in = P.WELL_IN_D / 2.0
    w_r = 0.0080
    h_wall = P.WELL_DEPTH - P.WELL_CHAMFER
    z_wall = P.WELL_FLOOR_Z + h_wall / 2.0
    w_tan = 2.0 * (r_in + w_r) * math.tan(math.pi / 8.0) + 0.001
    ch = P.WELL_CHAMFER
    ch_thick = 0.0030
    for k in range(8):
        phi = k * math.pi / 4.0
        c, s = math.cos(phi), math.sin(phi)
        rc = r_in + w_r / 2.0
        add_hot(box((w_r, w_tan, h_wall), pos=(wx + rc * c, wy + rc * s, z_wall), yaw=phi, color=ALU_BLOCK, friction=(0.25, 0.20)))
        # 45-deg outward top mouth chamfer: inner face flares outward from r_in at z = BATH_H - ch to r_in + ch at z = BATH_H
        rot_ch = mk.axis_angle((0.0, 0.0, 1.0), phi) @ mk.axis_angle((0.0, 1.0, 0.0), math.pi / 4.0)
        p_mid = np.array([wx, wy, P.BATH_H - ch / 2.0]) + (r_in + ch / 2.0) * np.array([c, s, 0.0]) + (ch_thick / 2.0) * (rot_ch[:, 0])
        add_hot(box((ch_thick, 2.0 * (r_in + 0.003) * math.tan(math.pi / 8.0), ch * math.sqrt(2.0)), pos=_floats(p_mid), quat=_quat(rot_ch), color=ALU_BLOCK, friction=(0.25, 0.20)))

    # Surrounding solid aluminium block slabs outside the octagon (r_clear = r_in + w_r = 16.6 mm) filling 76 x 96 mm:
    r_clear = r_in + w_r
    z_up = P.WELL_FLOOR_Z + P.WELL_DEPTH / 2.0
    # Left (+y) and right (-y) slabs of the 96 mm wide block:
    y_hi_slab = (hy0 + hw / 2.0) - (wy + r_clear)
    y_lo_slab = (wy - r_clear) - (hy0 - hw / 2.0)
    add_hot(box((hd, y_hi_slab, P.WELL_DEPTH), pos=(hx0, wy + r_clear + y_hi_slab / 2.0, z_up), color=ALU_BLOCK))
    add_hot(box((hd, y_lo_slab, P.WELL_DEPTH), pos=(hx0, wy - r_clear - y_lo_slab / 2.0, z_up), color=ALU_BLOCK))
    # Rear (+x) and front (-x) strips between y = wy - r_clear and wy + r_clear:
    x_hi_slab = (hx0 + hd / 2.0) - (wx + r_clear)
    x_lo_slab = (wx - r_clear) - (hx0 - hd / 2.0)
    if x_hi_slab > 0.001:
        add_hot(box((x_hi_slab, 2.0 * r_clear, P.WELL_DEPTH), pos=(wx + r_clear + x_hi_slab / 2.0, wy, z_up), color=ALU_BLOCK))
    if x_lo_slab > 0.001:
        add_hot(box((x_lo_slab, 2.0 * r_clear, P.WELL_DEPTH), pos=(wx - r_clear - x_lo_slab / 2.0, wy, z_up), color=ALU_BLOCK))

    # Visual dark discs for the other 14 wells of the 3 x 5 TC0401001-84 block:
    for r_idx, dx_w in enumerate((-0.020, 0.000, 0.020)):
        x_w = wx if r_idx == 0 else (hx0 + dx_w + 0.005)
        for c_idx in range(5):
            y_w = hy0 + (c_idx - 2) * 0.019
            if r_idx == 0 and c_idx == 2:
                continue  # the active open well at (wx, wy)
            shapes.append(cylinder(0.0080, 0.0006, pos=(x_w, y_w, P.BATH_H + 0.0003), color=WELL_DARK, solid=False))

    HOT_SHAPES = tuple(hot_indices)
    body(
        "bath",
        shapes,
        pos=(bx, by, top + P.OVER),
        mass=P.BATH_MASS,
        color=CASE_WHITE,
        friction=(P.BATH_FRICTION, P.BATH_FRICTION),
        contact_offset=P.COLLIDER_OFFSET,
        contacts=True,
    )


def build_rack(top: float) -> None:
    """BOCHEM 8291 18/10 stainless Z-rack: 220 x 110 x 50 mm, 10 x 5 chamfered 18.4 mm apertures on two tiers + solid floor."""
    rx, ry = P.RACK_XY
    shapes = [
        # Solid bottom plate (z = 0 .. 2.0 mm):
        box((P.RACK_L, P.RACK_W, P.RACK_SHEET_T), pos=(0.0, 0.0, P.RACK_SHEET_T / 2.0), color=STAINLESS),
        # Z-profile vertical webs: front/right (-y) from floor to middle tier, left (+y) from middle tier to top tier:
        box((P.RACK_L, 0.0018, P.RACK_MID_Z - P.RACK_SHEET_T), pos=(0.0, -P.RACK_W / 2.0 + 0.0009, (P.RACK_MID_Z + P.RACK_SHEET_T) / 2.0), color=STAINLESS_EDGE),
        box((P.RACK_L, 0.0018, P.RACK_H - P.RACK_MID_Z), pos=(0.0, P.RACK_W / 2.0 - 0.0009, (P.RACK_H + P.RACK_MID_Z) / 2.0), color=STAINLESS_EDGE),
    ]
    bar_w = P.RACK_PITCH - P.RACK_HOLE_IN  # 0.0200 - 0.0184 = 0.0016 m -> 18.4 mm clear inscribed opening in every cell!
    # Middle tier (z = 23..25 mm) and upper tier (z = 48..50 mm with stepped entry chamfer to 18.8 mm at top):
    xs = [(i - P.RACK_COLS / 2.0) * P.RACK_PITCH for i in range(P.RACK_COLS + 1)]  # 11 transverse ribs
    ys = [(j - P.RACK_ROWS / 2.0) * P.RACK_PITCH for j in range(P.RACK_ROWS + 1)]  # 6 longitudinal ribs
    low_slip = (0.22, 0.18)
    for x in xs:
        shapes.append(box((bar_w, P.RACK_W, P.RACK_SHEET_T), pos=(x, 0.0, P.RACK_MID_Z), color=STAINLESS, friction=low_slip))
        shapes.append(box((bar_w, P.RACK_W, 0.0012), pos=(x, 0.0, P.RACK_H - 0.0014), color=STAINLESS, friction=low_slip))
        shapes.append(box((0.0012, P.RACK_W, 0.0008), pos=(x, 0.0, P.RACK_H - 0.0004), color=STAINLESS_EDGE, friction=low_slip))
    for y in ys:
        shapes.append(box((P.RACK_L, bar_w, P.RACK_SHEET_T), pos=(0.0, y, P.RACK_MID_Z), color=STAINLESS, friction=low_slip))
        shapes.append(box((P.RACK_L, bar_w, 0.0012), pos=(0.0, y, P.RACK_H - 0.0014), color=STAINLESS, friction=low_slip))
        shapes.append(box((P.RACK_L, 0.0012, 0.0008), pos=(0.0, y, P.RACK_H - 0.0004), color=STAINLESS_EDGE, friction=low_slip))

    body(
        "rack",
        shapes,
        pos=(rx, ry, top + P.OVER),
        mass=P.RACK_MASS,
        color=STAINLESS,
        friction=(0.75, 0.65),
        contact_offset=P.COLLIDER_OFFSET,
        contacts=True,
    )


def build_tube(top: float) -> None:
    """PYREX 1620/09M 16 x 100 mm borosilicate rimmed test tube standing upright on the dry-bath well floor."""
    barrel_l = P.TUBE_L - P.RIM_H
    shapes = [
        cylinder(P.TUBE_R, barrel_l, pos=(0.0, 0.0, -P.RIM_H / 2.0), color=GLASS, friction=(P.TUBE_FRICTION, P.TUBE_FRICTION)),
        cylinder(P.RIM_R, P.RIM_H, pos=(0.0, 0.0, P.TUBE_L / 2.0 - P.RIM_H / 2.0), color=GLASS_RIM, friction=(P.TUBE_FRICTION, P.TUBE_FRICTION)),
        # Visual high-contrast dry indicators (borosilicate frosted patch + amber dry-bath sample band):
        cylinder(P.TUBE_R + 0.0002, 0.018, pos=(0.0, 0.0, 0.024), color=PYREX_PATCH, solid=False),
        cylinder(P.TUBE_R + 0.0001, 0.028, pos=(0.0, 0.0, -0.030), color=SAMPLE_AMBER, solid=False),
    ]
    z0 = top + P.OVER + P.WELL_FLOOR_Z + P.TUBE_L / 2.0 + P.OVER
    body(
        "tube",
        shapes,
        pos=(P.WELL_XY[0], P.WELL_XY[1], z0),
        mass=P.TUBE_SOLVER_MASS,
        color=GLASS,
        friction=(P.TUBE_FRICTION, P.TUBE_FRICTION),
        contact_offset=P.COLLIDER_OFFSET,
        contacts=True,
    )


def _v_cheeks(y_sign: float) -> list[mk.Shape]:
    """Two 30-deg hardwood V-cradle cheeks around (P.X_JAW, 0, 0) on side ``y_sign`` (-1: long_arm, +1: short_lever)."""
    t_c, l_c = 0.0050, 0.0110
    high_grip = (1.60, 1.40)  # unfinished end-grain hardwood V-groove against borosilicate glass
    sa, ca = math.sin(P.V_HALF_ANGLE), math.cos(P.V_HALF_ANGLE)
    out = []
    for s in (-1.0, 1.0):
        # Inward normal into the V-pocket points opposite y_sign:
        n = np.array([-s * sa, -y_sign * ca, 0.0])
        t_vec = np.array([ca, -s * y_sign * sa, 0.0])
        centre = np.array([P.X_JAW, 0.0, 0.0]) - (P.R_CLOSED + t_c / 2.0) * n + (s * 0.0015) * t_vec
        out.append(
            box(
                (t_c, l_c, P.BAR_H),
                pos=_floats(centre),
                yaw=float(math.atan2(n[1], n[0])),
                color=BEECH_JAW,
                friction=high_grip,
                friction_combine="max",
            )
        )
    return out


def build_clamp(top: float) -> None:
    """Eisco CH0726B 260.4 mm cross-action wooden clothespin holder on the blue silicone mat."""
    pin = (P.CLAMP_START_PIN[0], P.CLAMP_START_PIN[1], top + P.CLAMP_START_PIN[2])
    # 1. long_arm (y < 0 side): straight shaft from X_BUTT (-0.1894) to +0.0390, outer jaw spine, and -y 90-deg V-cheeks:
    shaft_x1 = 0.0390
    shaft_len = shaft_x1 - P.X_BUTT
    long_shapes = [
        box((shaft_len, P.BAR_W, P.BAR_H), pos=((P.X_BUTT + shaft_x1) / 2.0, P.LONG_Y, 0.0), color=BEECH, friction=mk.GRIPPY),
        box((P.X_TIP - shaft_x1, 0.0060, P.BAR_H), pos=((shaft_x1 + P.X_TIP) / 2.0, -0.0120, 0.0), color=BEECH, friction=mk.GRIPPY),
        *_v_cheeks(-1.0),
        cylinder(0.0045, P.BAR_H + 0.001, pos=(0.0, 0.0, 0.0), axis="z", color=STEEL_SPRING, solid=False),
    ]
    body(
        "long_arm",
        long_shapes,
        pos=pin,
        mass=P.LONG_MASS,
        color=BEECH,
        friction=mk.GRIPPY,
        contact_offset=P.COLLIDER_OFFSET,
        contacts=True,
    )

    # 2. short_lever (y > 0 side): flared rear squeeze tail (x = -0.073..0.000), front spine, and +y 90-deg V-cheeks:
    tail_dx = abs(P.X_SHORT_END)
    tail_len = tail_dx / math.cos(P.TAIL_FLARE)
    tail_mid = (-tail_dx / 2.0, P.SHORT_PIN_Y + (tail_dx / 2.0) * math.tan(P.TAIL_FLARE), 0.0)
    short_shapes = [
        box((tail_len, P.BAR_W, P.BAR_H), pos=tail_mid, yaw=-P.TAIL_FLARE, color=BEECH, friction=mk.GRIPPY),
        box((shaft_x1, P.BAR_W, P.BAR_H), pos=(shaft_x1 / 2.0, P.SHORT_PIN_Y, 0.0), color=BEECH, friction=mk.GRIPPY),
        box((P.X_TIP - shaft_x1, 0.0060, P.BAR_H), pos=((shaft_x1 + P.X_TIP) / 2.0, 0.0120, 0.0), color=BEECH, friction=mk.GRIPPY),
        *_v_cheeks(1.0),
    ]
    body(
        "short_lever",
        short_shapes,
        pos=pin,
        mass=P.SHORT_MASS,
        color=BEECH,
        friction=mk.GRIPPY,
        contact_offset=P.COLLIDER_OFFSET,
        contacts=True,
    )

    # Positive travel about +z rotates short_lever's front jaw (+x) toward +y (OPEN) and rear tail (-x) toward -y (SQUEEZED):
    hinge(
        "jaw_h",
        "short_lever",
        "long_arm",
        anchor=pin,
        axis=(0.0, 0.0, 1.0),
        limits=(0.0, P.THETA_MAX),
        spring=P.JAW_K,
        rest=P.JAW_REST,
        damping=P.JAW_DAMPING,
    )
    # Anti-creep barrel hold between tube and long_arm (not joined by any hinge): starts unmade, engages only when
    # both real jaws press >= 6 N on the seated tube after opening >= 10 deg, and cuts as soon as jaw_h opens >= 11.5 deg.
    weld("w_grip", "tube", "long_arm", made=False)
    mk.sense_contact("tube", "long_arm", "short_lever", "bath", "rack")


def build(top: float) -> None:
    build_mat(top)
    build_bath(top)
    build_rack(top)
    build_tube(top)
    build_clamp(top)


# --- hand collision-hull and bounding-box geometry helpers (used by task.py) -----------------------------------------
_HULLS: dict = {}
_LINKS: dict = {}


def read_hands_into(state: dict, sides: tuple[str, ...] = ("left", "right")) -> None:
    """Populate ``state['hands']`` with every collision-hull vertex of both grippers in the world."""
    from yam_arena import analysis

    if not _HULLS:
        for link in ("gripper", "tip_left", "tip_right"):
            _HULLS[link] = np.vstack([h.vertices for h in analysis.link_hulls().get(link, [])])
    base = mk.MECH.environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    hands = {}
    for side in sides:
        robot = base.scene[f"{side}_robot"]
        points = []
        for link, local in _HULLS.items():
            key = (id(base), side, link)
            if key not in _LINKS:
                found = robot.find_bodies(link)[0]
                if not found:
                    raise RuntimeError(f"the {side} arm has no body {link!r}")
                _LINKS[key] = found[0]
            pose = robot.data.body_link_pose_w.torch[0, _LINKS[key]].detach().cpu().numpy().astype(float)
            points.append(local @ mk.quat_to_matrix(pose[3:7]).T + pose[:3] - origin)
        hands[side] = np.vstack(points)
    state["hands"] = hands


def box_distance(points: np.ndarray, name: str) -> float:
    position, rotation = mk.pose(name)
    low, high = mk.MECH.bodies[name].bounds()
    local = (points - position) @ rotation
    gap = np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0)
    return float(np.linalg.norm(gap, axis=1).min())


def bodies_gap(a: str, b: str) -> float:
    pa, ra = mk.pose(a)
    la, ha = mk.MECH.bodies[a].bounds()
    corners = np.array([[x, y, z] for x in (la[0], ha[0]) for y in (la[1], ha[1]) for z in (la[2], ha[2])]) @ ra.T + pa
    return box_distance(corners, b)
