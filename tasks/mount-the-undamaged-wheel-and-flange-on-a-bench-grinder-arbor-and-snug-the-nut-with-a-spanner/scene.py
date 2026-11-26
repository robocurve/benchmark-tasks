"""Bodies and joints of the bench-grinder wheel task, built from ``parts.py``:
- ``grinder`` (fixed): unplugged Einhell TC-BG 150 motor housing, bolted baseplate with OFF switch and visible
  disconnected mains cord/plug, left (+y) wheel guard and adjustable workpiece rest (2 mm gap), bare right (-y) inner
  guard plate;
- ``tray`` (fixed): bench organizer rack with V-slots for the two 150 mm wheels, stepped slots for the outer flange and
  arbor nut, and an edge cradle for the bare 19 mm open-ended spanner;
- ``spindle`` (free on ``hinge('spindle')``): continuous horizontal motor shaft carrying the factory-installed +y fine
  wheel (K60, Ø 150 x 16 mm) and, on the -y service end, the keyed Ø 52 mm inner driving flange and bare Ø 12.7 mm
  right-hand M12 x 1.75 arbor;
- ``nut_hub`` (free on ``hinge('nut_turn')`` on ``spindle``): coaxial thread carrier inside the -y arbor tip;
- ``wheel_sound`` and ``wheel_cracked`` (free): two Ø 150 x 16 mm blottered vitrified wheels (16-stave Ø 13.6 mm bore),
  one intact and one with a missing rim chunk and radial crack across both faces;
- ``outer_flange`` (free): Ø 52 x 8 mm stamped steel dished flange (5 mm concave relief on one side, zero added tabs);
- ``arbor_nut`` (free until threaded onto ``nut_hub`` via ``weld('nut_catch', made=False)``): 19 mm AF flanged hex nut
  with a white witness stripe;
- ``wrench`` (free): bare GEDORE No. 894 19 single open-ended spanner (170 mm long, 7 mm thick, 15 deg jaw set).
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as mk
import parts as P
from mechkit import body, box, cylinder, hinge, sphere, weld

MECHKIT_HELPER = True

RED_EINHELL = (0.78, 0.14, 0.14)
DARK_IRON = (0.18, 0.19, 0.21)
GUARD_BLACK = (0.13, 0.14, 0.16)
STEEL = (0.68, 0.71, 0.75)
STEEL_DARK = (0.30, 0.33, 0.37)
PHOSPHATE = (0.26, 0.28, 0.30)  # manganese-phosphated GEDORE 894 spanner
STONE_COARSE = (0.48, 0.50, 0.53)
STONE_FINE = (0.56, 0.60, 0.64)
BLOTTER_TAN = (0.85, 0.74, 0.52)
CRACK_DARK = (0.08, 0.07, 0.07)
RACK_BLUE = (0.16, 0.24, 0.34)
CORD_BLACK = (0.10, 0.10, 0.11)
WHITE = (0.96, 0.96, 0.95)
BRASS = (0.76, 0.64, 0.32)
# Bare steel where the spanner's jaw meets the hex flats (the shank and collar keep the pad sleeves' grip)
STEEL_ON_STEEL = (P.STEEL_MU, 0.15)
# Jaw flanks on the hex flats: a stand-in grip, not steel's 0.2. At 0.2 the corners slid along the cheeks and the
# solver let the held spanner's cheek sink 2.5 mm into the hex at ~17 N while the nut stood (runs gw_p13-p16).
# Combined by "max", it outranks the hex's own steel "min", so fingers and arbor still meet steel's 0.2 on the nut
# (at 1.2 on the hex the carried nut stopped 4 mm short and fell off the arbor: runs gw_d1-d3); the lead-in
# chamfers, tips and throat keep steel's 0.2 so the jaw still slides onto the flats.
JAW_ON_FLATS = (1.2, 1.0)


def _floats(v) -> tuple[float, ...]:
    return tuple(float(x) for x in v)


def _quat(matrix: np.ndarray) -> tuple[float, ...]:
    return _floats(mk.matrix_to_quat(matrix))


def _ry(angle: float) -> np.ndarray:
    return mk.axis_angle((0.0, 1.0, 0.0), angle)


def _rz(angle: float) -> np.ndarray:
    return mk.axis_angle((0.0, 0.0, 1.0), angle)


def arbor_anchor(top: float) -> tuple[float, float, float]:
    return (P.GRINDER_X, P.GRINDER_Y, top + P.ARBOR_Z)


def nut_anchor(top: float) -> tuple[float, float, float]:
    return (P.GRINDER_X, P.Y_NUT_SNUG, top + P.ARBOR_Z)


def _wheel_shapes(stone_color, *, cracked: bool = False) -> list:
    """16-stave hollow wheel in its local frame (bore along local y, Ø 150 mm in local x-z, Ø 13.6 mm central bore)."""
    shapes = []
    n = 16
    # Inner blottered hub ring: r from BORE_R (6.8 mm) to BLOTTER_R (30.0 mm)
    r_in, r_mid, r_out = P.BORE_R, P.BLOTTER_R, P.WHEEL_R
    hub_len = r_mid - r_in
    hub_rc = 0.5 * (r_in + r_mid)
    hub_w = 2.0 * r_mid * math.sin(math.pi / n) + 0.0008
    rim_len = r_out - (r_mid - 0.001)
    rim_rc = 0.5 * (r_out + r_mid - 0.001)
    rim_w = 2.0 * r_out * math.sin(math.pi / n) + 0.0006

    # Brass 32 -> 12.7 mm factory reducing bushing visual ring inside blotter
    bush_r = 0.016
    for k in range(n):
        a = 2.0 * math.pi * k / n
        rot_y = _ry(-a)  # local x rotated in x-z plane by angle a: x' = (cos a, 0, sin a)
        q = _quat(rot_y)
        ca, sa = math.cos(a), math.sin(a)
        shapes.append(
            box((hub_len, P.WHEEL_T, hub_w), pos=(hub_rc * ca, 0.0, hub_rc * sa), quat=q, color=BLOTTER_TAN)
        )
        if cracked and k == 7:  # upper-front sector (-x, +z: a = 157.5 deg): missing 18 mm rim chunk!
            short_len = rim_len - 0.018
            short_rc = (r_mid - 0.001) + 0.5 * short_len
            shapes.append(
                box((short_len, P.WHEEL_T, rim_w * 0.82), pos=(short_rc * ca, 0.0, short_rc * sa), quat=q, color=CRACK_DARK)
            )
        else:
            # Split radial rim into inner and outer sub-rings so PhysX generates multi-point anti-droop pad contacts!
            half_rim = 0.5 * rim_len
            for sub, sub_w in ((-0.25, rim_w * 0.72), (0.25, rim_w)):
                rc_sub = rim_rc + sub * rim_len
                shapes.append(
                    box((half_rim, P.WHEEL_T, sub_w), pos=(rc_sub * ca, 0.0, rc_sub * sa), quat=q, color=stone_color)
                )
    # Flush factory bushing ring tint around the bore on both faces
    shapes.append(cylinder(bush_r, P.WHEEL_T + 0.0004, axis="y", color=BRASS, solid=False))
    if cracked:
        # Jagged dark radial crack fissure from chipped sector (a = 157.5 deg) inward across the wheel on both faces,
        # plus a secondary branch so the crack is unmistakable from top and wrist cameras at 320 x 176.
        for a_cr, r0, r1, w_cr in (
            (math.radians(157.5), 0.014, 0.060, 0.0042),
            (math.radians(146.0), 0.032, 0.068, 0.0032),
        ):
            ca, sa = math.cos(a_cr), math.sin(a_cr)
            rc = 0.5 * (r0 + r1)
            shapes.append(
                box(
                    (r1 - r0, P.WHEEL_T + 0.0014, w_cr),
                    pos=(rc * ca, 0.0, rc * sa),
                    quat=_quat(_ry(-a_cr)),
                    color=CRACK_DARK,
                    solid=False,
                )
            )
    return shapes


def _flange_shapes() -> list:
    """Ø 52 x 8 mm OSHA-relieved stamped steel outer flange in its local frame (bore along local y).
    Local +y (+4.0 mm): flat Ø 52 mm outer annular bearing rim (r = 19..26 mm) surrounding a 5.0 mm concave recess.
    Local -y (-4.0 mm): central hub collar (r = 6.85..20 mm). Zero added grip tabs."""
    shapes = []
    n = 12
    # Central recessed web: y_local in [-4.0, -1.0] mm (thickness 3.0 mm), r from 6.85 to 20.5 mm
    r_bore, r_web = P.BORE_R + 0.00005, 0.0205
    web_len = r_web - r_bore
    web_rc = 0.5 * (r_bore + r_web)
    web_w = 2.0 * r_web * math.sin(math.pi / n) + 0.0006
    # Conical skirt & outer bearing ring: y_local in [+1.5, +4.0] mm (thickness 2.5 mm) + bridge [-1.5, +1.5] mm, r = 19..26 mm
    r_rim_in, r_rim_out = 0.0190, P.FLANGE_R
    rim_len = r_rim_out - r_rim_in
    rim_rc = 0.5 * (r_rim_in + r_rim_out)
    rim_w = 2.0 * r_rim_out * math.sin(math.pi / n) + 0.0006
    for k in range(n):
        a = 2.0 * math.pi * k / n
        q = _quat(_ry(-a))
        ca, sa = math.cos(a), math.sin(a)
        shapes.append(
            box((web_len, P.FLANGE_RIM_T, web_w), pos=(web_rc * ca, -P.FLANGE_DEPTH / 2 + P.FLANGE_RIM_T / 2, web_rc * sa), quat=q, color=STEEL)
        )
        # Outer rim spans y_local in [-1.5, +4.0] mm (5.5 mm skirt ending at +4.0 mm bearing plane; leaves 2.5 mm step on -y and 5.0 mm concave dish on +y!)
        skirt_t = 0.0055
        shapes.append(
            box((rim_len, skirt_t, rim_w), pos=(rim_rc * ca, P.FLANGE_DEPTH / 2 - skirt_t / 2, rim_rc * sa), quat=q, color=STEEL)
        )
    # Dark gunmetal recess tint inside the concave +y dish so camera views clearly distinguish recessed (+y) vs convex (-y)
    shapes.append(cylinder(0.0188, 0.0008, pos=(0.0, -0.0006, 0.0), axis="y", color=DARK_IRON, solid=False))
    return shapes


def _nut_shapes() -> list:
    """19.0 mm AF M12 flanged hex nut (local y = thread axis, length 14 mm = [-7, +7] mm; inner Ø 28 mm collar at +y)."""
    shapes = []
    # 6-flat hollow hexagon: outer AF = 19.0 mm (r_flat = 9.50 mm), inner inscribed radius = 6.85 mm (Ø 13.7 mm bore)
    r_in = P.BORE_R + 0.00005
    r_flat = P.NUT_AF / 2.0
    wall = r_flat - r_in
    rc = 0.5 * (r_in + r_flat)
    flat_w = P.NUT_AF * math.tan(math.pi / 6.0)
    hex_y = -P.NUT_LEN / 2.0 + P.NUT_HEX_LEN / 2.0  # -1.5 mm
    for k in range(6):
        # k=0 (+x) and k=3 (-x) give vertical +-y flats in the cradle (yaw=-90 deg), flush with parallel gripper pads!
        a = k * (math.pi / 3.0)
        q = _quat(_ry(-a))
        ca, sa = math.cos(a), math.sin(a)
        shapes.append(box((wall, P.NUT_HEX_LEN, flat_w), pos=(rc * ca, hex_y, rc * sa), quat=q, color=STEEL, friction=STEEL_ON_STEEL, friction_combine="min"))
    # Solid hexagonal core for the spanner's jaw (the union of three 19.0 x 10.97 mm boxes at 0/60/120 deg is exactly the
    # hexagon): the thin walls above would let a jaw cheek slip through the seam at a corner. It passes the arbor and the
    # thread carrier, so the bore stays open on the shaft.
    for k in range(3):
        a = k * (math.pi / 3.0)
        shapes.append(
            box((P.NUT_AF, P.NUT_HEX_LEN, flat_w), pos=(0.0, hex_y, 0.0), quat=_quat(_ry(-a)), color=STEEL,
                passes=("spindle", "nut_hub"), friction=STEEL_ON_STEEL, friction_combine="min")
        )
    # 12-stave Ø 28 x 3 mm inner washer collar at y_local in [+4.0, +7.0] mm
    r_col = P.NUT_COLLAR_D / 2.0
    col_len = r_col - r_in
    col_rc = 0.5 * (r_in + r_col)
    col_w = 2.0 * r_col * math.sin(math.pi / 12.0) + 0.0005
    col_y = P.NUT_LEN / 2.0 - P.NUT_COLLAR_T / 2.0  # +5.5 mm
    for k in range(12):
        a = 2.0 * math.pi * k / 12.0
        q = _quat(_ry(-a))
        ca, sa = math.cos(a), math.sin(a)
        shapes.append(box((col_len, P.NUT_COLLAR_T, col_w), pos=(col_rc * ca, col_y, col_rc * sa), quat=q, color=STEEL_DARK))
    # White witness stripe at local +115 deg: on top (+z) next to the arbor's 0-deg tick when the nut is hand-snug
    w_ang = math.radians(115.0)
    q_wit = _quat(_ry(-w_ang))
    shapes.append(
        box((0.0014, P.NUT_LEN + 0.0004, 0.0032), pos=((r_flat + 0.0005) * math.cos(w_ang), 0.0, (r_flat + 0.0005) * math.sin(w_ang)), quat=q_wit, color=WHITE, solid=False)
    )
    shapes.append(
        box((0.0050, P.NUT_COLLAR_T + 0.0004, 0.0032), pos=(0.0115 * math.cos(w_ang), col_y, 0.0115 * math.sin(w_ang)), quat=q_wit, color=WHITE, solid=False)
    )
    return shapes


def _wrench_shapes() -> list:
    """Bare GEDORE No. 894 19 single open-ended spanner (170 mm total length, 7.0 mm thick, 15 deg jaw set, zero sleeves).
    Origin (0, 0, 0) is the centre of the 19.6 mm jaw slot; handle runs along -x to x = -0.156 m (head front at +0.014 m)."""
    jaw_rot = _ry(math.radians(P.WRENCH_JAW_DEG))
    q_jaw = _quat(jaw_rot)
    cheek_w = (P.WRENCH_HEAD_W - P.WRENCH_JAW_GAP) / 2.0  # (40.0 - 19.6)/2 = 10.2 mm
    cheek_zc = P.WRENCH_JAW_GAP / 2.0 + cheek_w / 2.0      # 14.9 mm in jaw frame
    # Two parallel 7 mm-thick steel jaw cheeks (x_jaw in [-11, +14] mm) + DIN 894 mouth lead-in chamfers + throat backstop
    shapes = []
    for s in (1.0, -1.0):
        # gripping flank x_jaw in [-11.0, +8.5] mm, then a 3.5 mm steel tip: a square tip edge that grips stops a hex
        # corner it meets on the way in (the jaw stalled 17.5 mm short: run gw_p18)
        p_cheek = jaw_rot @ np.array([-0.00125, 0.0, s * cheek_zc])
        shapes.append(box((0.0195, P.WRENCH_T, cheek_w), pos=_floats(p_cheek), quat=q_jaw, color=PHOSPHATE, friction=JAW_ON_FLATS, friction_combine="max"))
        p_tip = jaw_rot @ np.array([0.01025, 0.0, s * cheek_zc])
        shapes.append(box((0.0035, P.WRENCH_T, cheek_w), pos=_floats(p_tip), quat=q_jaw, color=PHOSPHATE, friction=STEEL_ON_STEEL, friction_combine="min"))
        chamfer_rot = jaw_rot @ _ry(-s * math.radians(18.0))
        p_cham = jaw_rot @ np.array([0.0115, 0.0, s * (cheek_zc + 0.0014)])
        shapes.append(box((0.006, P.WRENCH_T, cheek_w * 0.85), pos=_floats(p_cham), quat=_quat(chamfer_rot), color=PHOSPHATE, friction=STEEL_ON_STEEL, friction_combine="min"))
    p_throat = jaw_rot @ np.array([-0.0155, 0.0, 0.0])
    shapes.append(box((0.011, P.WRENCH_T, P.WRENCH_HEAD_W * 0.92), pos=_floats(p_throat), quat=q_jaw, color=PHOSPHATE, friction=STEEL_ON_STEEL, friction_combine="min"))
    # Bare 7.0 x 18.0 mm steel shank from x = -0.018 to -0.156 m, tiled as 2 x 6 flush sub-boxes so PhysX GPU contact
    # generates spatially separated contact patches across the 40 mm finger pads (true torsional patch resistance!).
    seg_n = 6
    seg_len = 0.138 / seg_n
    half_h = P.WRENCH_SHANK_W / 2.0
    for i in range(seg_n):
        xc = -0.018 - (i + 0.5) * seg_len
        for sz in (-0.5, 0.5):
            shapes.append(box((seg_len, P.WRENCH_T, half_h), pos=(xc, 0.0, sz * half_h), color=PHOSPHATE))
    return shapes


def build_grinder(top: float) -> None:
    """Fixed unplugged Einhell TC-BG 150 housing, bolted base, left guard + 2 mm tool rest, and disconnected mains plug."""
    gx, gy = P.GRINDER_X, P.GRINDER_Y
    az = P.ARBOR_Z
    bx, by, bz = P.BASE_SIZE
    # Frame at (gx, gy, top): local z is height over bench
    shapes = [
        # Bolted cast baseplate + front OFF ('0') switch housing
        box((bx, by, bz), pos=(0.0, 0.0, bz / 2.0), color=DARK_IRON),
        box((0.056, 0.104, az - bz - 0.032), pos=(0.0, 0.0, bz + (az - bz - 0.032) / 2.0), color=DARK_IRON),
        box((0.012, 0.026, 0.020), pos=(-bx / 2.0 - 0.004, 0.0, 0.016), color=GUARD_BLACK),
        box((0.004, 0.014, 0.012), pos=(-bx / 2.0 - 0.010, 0.0, 0.016), color=RED_EINHELL, solid=False),
        # Red Einhell motor barrel along y
        cylinder(P.MOTOR_R, P.MOTOR_LEN, pos=(0.0, 0.0, az), axis="y", color=RED_EINHELL),
        # Right (-y) inner guard backplate (16 mm behind keyed inner flange, outer hood 4 & right rest removed per sec. 5.5)
        cylinder(0.080, 0.005, pos=(0.0, P.Y_INNER_FACE - gy + 0.016, az), axis="y", color=GUARD_BLACK),
        # Left (+y) C-shaped rear wheel guard (covers rear half x > -8 mm, leaving front 67 mm of the 16 mm K60 wheel
        # completely exposed above the 2 mm workpiece rest as in Einhell manual Fig. 1 and Fig. 12!)
        box((0.088, 0.005, 0.158), pos=(0.036, P.LEFT_WHEEL_Y - gy - 0.016, az + 0.001), color=GUARD_BLACK),
        box((0.088, 0.005, 0.158), pos=(0.036, P.LEFT_WHEEL_Y - gy + 0.016, az + 0.001), color=GUARD_BLACK),
        box((0.088, 0.036, 0.010), pos=(0.036, P.LEFT_WHEEL_Y - gy, az + 0.079), color=GUARD_BLACK),
        box((0.012, 0.036, 0.146), pos=(0.080, P.LEFT_WHEEL_Y - gy, az + 0.002), color=GUARD_BLACK),
        box((0.110, 0.036, 0.008), pos=(0.025, P.LEFT_WHEEL_Y - gy, az - 0.078), color=GUARD_BLACK),
        # Left adjustable steel workpiece support (Werkstueckauflage 7), 2.0 mm in front of the +y Ø 150 mm wheel rim
        box((0.026, 0.030, 0.006), pos=(-P.WHEEL_R - P.REST_GAP - 0.013, P.LEFT_WHEEL_Y - gy, az - 0.016), color=STEEL),
        box((0.016, 0.012, 0.028), pos=(-P.WHEEL_R + 0.006, P.LEFT_WHEEL_Y - gy - 0.014, az - 0.028), color=STEEL_DARK),
        # Visibly disconnected mains power cord and unplugged 2-pin Schuko plug resting on the bench behind/left
        box((0.095, 0.007, 0.007), pos=(0.065, 0.115, 0.005), yaw=0.35, color=CORD_BLACK, solid=False),
        cylinder(0.018, 0.038, pos=(0.095, 0.165, 0.018), axis="y", color=CORD_BLACK),
        cylinder(0.0024, 0.019, pos=(0.095, 0.193, 0.022), axis="y", color=STEEL, solid=False),
        cylinder(0.0024, 0.019, pos=(0.095, 0.193, 0.013), axis="y", color=STEEL, solid=False),
    ]
    body("grinder", shapes, pos=(gx, gy, top), kinematic=True, color=RED_EINHELL)


def build_spindle_and_nut_hub(top: float) -> None:
    """Continuous motor shaft with installed +y K60 wheel, keyed -y inner flange, and bare Ø 12.7 mm right arbor."""
    gx, gy = P.GRINDER_X, P.GRINDER_Y
    az = top + P.ARBOR_Z
    shapes = [
        # Continuous machined steel Ø 12.7 mm shaft from the -y arbor, through the motor, to the +y wheel; low
        # friction (0.15, min) so the wheel and flange slide onto it without binding. It ends ARBOR_R short of the tip.
        cylinder(
            P.ARBOR_R,
            (P.LEFT_WHEEL_Y + 0.024) - (P.Y_ARBOR_TIP + P.ARBOR_R),
            pos=(0.0, 0.5 * (P.LEFT_WHEEL_Y + 0.024 + P.Y_ARBOR_TIP + P.ARBOR_R) - gy, 0.0),
            axis="y",
            color=STEEL,
            friction=(0.15, 0.12),
            friction_combine="min",
            passes=("grinder",),
        ),
        # Domed end of the arbor (half of a Ø 12.7 mm ball): the lead-in every bore finds by itself; it stands in for
        # the chamfered end of the real threaded arbor
        sphere(P.ARBOR_R, pos=(0.0, P.Y_ARBOR_TIP + P.ARBOR_R - gy, 0.0), color=STEEL, friction=(0.15, 0.12), friction_combine="min", passes=("grinder",)),
        # Keyed -y inner driving flange (Ø 52 x 5 mm, wheel-bearing face at Y_INNER_FACE)
        cylinder(P.FLANGE_R, 0.005, pos=(0.0, P.Y_INNER_FACE - gy + 0.0025, 0.0), axis="y", color=STEEL_DARK, passes=("grinder",)),
        # Installed +y fine grinding wheel (K60, Ø 150 x 16 mm): segmented into 16 radial boxes for multi-point counterhold!
        *[
            box(
                (P.WHEEL_R - 0.026, P.WHEEL_T, 2.0 * P.WHEEL_R * math.sin(math.pi / 16.0) + 0.0006),
                pos=(0.5 * (P.WHEEL_R + 0.026) * math.cos(2.0 * math.pi * k / 16.0), P.LEFT_WHEEL_Y - gy, 0.5 * (P.WHEEL_R + 0.026) * math.sin(2.0 * math.pi * k / 16.0)),
                quat=_quat(_ry(-2.0 * math.pi * k / 16.0)),
                color=STONE_FINE,
                friction=(P.COUNTER_RUBBER_MU, 1.0),
                friction_combine="max",
                passes=("grinder",),
            )
            for k in range(16)
        ],
        cylinder(P.BLOTTER_R, P.WHEEL_T + 0.0006, pos=(0.0, P.LEFT_WHEEL_Y - gy, 0.0), axis="y", color=BLOTTER_TAN, solid=False),
        cylinder(P.FLANGE_R, P.WHEEL_T + 0.008, pos=(0.0, P.LEFT_WHEEL_Y - gy, 0.0), axis="y", color=STEEL_DARK, passes=("grinder",)),
        # Witness stripe on +y wheel face so any spindle slip is visible in camera frames
        box((0.028, P.WHEEL_T + 0.0012, 0.004), pos=(-0.052, P.LEFT_WHEEL_Y - gy, 0.0), color=WHITE, solid=False),
        # White 0-deg reference tick on top of the arbor, between the nut's seat and the dome
        box((0.0025, 0.006, 0.0015), pos=(0.0, P.Y_ARBOR_TIP + P.ARBOR_R - gy + 0.0038, P.ARBOR_R + 0.0004), color=WHITE, solid=False),
    ]
    body("spindle", shapes, pos=(gx, gy, az), mass=1.05, color=STEEL, contact_offset=0.001, contacts=True)
    hinge("spindle", "spindle", "grinder", anchor=(gx, gy, az), axis=(0.0, 1.0, 0.0), limits=None, damping=P.SPINDLE_DAMPING)

    # Coaxial thread carrier inside the -y arbor tip: built at +30 deg past snug so [-140, +140] deg covers all turns
    nh_at = nut_anchor(top)
    body(
        "nut_hub",
        [cylinder(0.0045, 0.010, axis="y", color=STEEL, passes=("wheel_sound", "outer_flange", "arbor_nut", "wrench", "spindle"))],
        pos=nh_at,
        mass=P.NUT_SOLVER_MASS,
        inertia=P.SOLVER_SPIN_INERTIA,
        color=STEEL,
    )
    hinge(
        "nut_turn",
        "nut_hub",
        "spindle",
        anchor=nh_at,
        axis=(0.0, 1.0, 0.0),
        limits=P.NUT_LIMITS,
        spring=P.NUT_DRIVE_K,
        rest=0.0,
        damping=P.NUT_DRIVE_D,
    )


def build_tray_and_parts(top: float) -> None:
    """Bench organizer rack + free wheels, outer flange, arbor nut, and bare 19 mm GEDORE spanner."""
    tray_shapes = []

    # 1. Sound wheel low V-slot at SOUND_START (1.0 mm mat + 12 mm side guides so +22 mm lift clears everything!)
    sx, sy, sz = P.SOUND_START
    syaw = P.SOUND_YAW
    R_s = _rz(syaw)
    q_s = _quat(R_s)
    pad_h = max(0.001, sz - P.WHEEL_R)  # 1.0 mm mat
    tray_shapes.append(box((0.084, 0.032, pad_h), pos=(sx, sy, pad_h / 2.0), quat=q_s, color=RACK_BLUE))
    for side in (-1.0, 1.0):
        off_cheek = R_s @ np.array([0.0, side * (P.WHEEL_T / 2.0 + 0.0030), 0.006])
        tray_shapes.append(box((0.070, 0.004, 0.012), pos=_floats(np.array([sx, sy, 0.0]) + off_cheek), quat=q_s, color=RACK_BLUE))
        off_stop = R_s @ np.array([side * 0.036, 0.0, 0.0055])
        tray_shapes.append(box((0.006, 0.018, 0.011), pos=_floats(np.array([sx, sy, 0.0]) + off_stop), quat=q_s, color=RACK_BLUE))

    # 2. Cracked wheel V-slot at CRACKED_START (yawed CRACKED_YAW = -22 deg for clear camera inspection)
    cx, cy, cz = P.CRACKED_START
    cyaw = P.CRACKED_YAW
    R_c = _rz(cyaw)
    q_c = _quat(R_c)
    pad_c = cz - P.WHEEL_R  # 8.0 mm floor pad under the cracked wheel
    tray_shapes.append(box((0.088, 0.034, pad_c), pos=(cx, cy, pad_c / 2.0), quat=q_c, color=RACK_BLUE))
    for side in (-1.0, 1.0):
        off_cheek = R_c @ np.array([0.0, side * (P.WHEEL_T / 2.0 + 0.0032), 0.013])
        tray_shapes.append(box((0.084, 0.004, 0.026), pos=_floats(np.array([cx, cy, 0.0]) + off_cheek), quat=q_c, color=RACK_BLUE))
        off_stop = R_c @ np.array([side * 0.044, 0.0, 0.0095])
        tray_shapes.append(box((0.008, 0.020, 0.019), pos=_floats(np.array([cx, cy, 0.0]) + off_stop), quat=q_c, color=RACK_BLUE))

    # 3. Outer flange slot at FLANGE_START (upright, yaw = -90 deg: local +y recessed face points +x; +90 deg turn seats it!)
    fx, fy, fz = P.FLANGE_START
    # The pedestal begins under the flange's -x face and the side stops sit forward of it: the fingertips, which come
    # down in front of the flange at an angle, reach down to 15 mm there and must find nothing.
    f_ped_h = fz - P.FLANGE_R  # 20.0 mm
    tray_shapes.append(box((P.FLANGE_DEPTH + 0.006, 0.064, f_ped_h), pos=(fx + 0.003, fy, f_ped_h / 2.0), color=RACK_BLUE))
    tray_shapes.append(box((0.004, 0.056, 0.012), pos=(fx + (P.FLANGE_DEPTH / 2.0 + 0.0032), fy, f_ped_h + 0.006), color=RACK_BLUE))
    for side in (-1.0, 1.0):
        tray_shapes.append(box((0.010, 0.005, 0.012), pos=(fx + 0.004, fy + side * (P.FLANGE_R + 0.004), f_ped_h + 0.006), color=RACK_BLUE))

    # 4. Arbor nut pedestal at NUT_START (yaw = -90 deg: bore along x, collar at +x, two flats facing +-y)
    nx, ny, nz = P.NUT_START
    n_ped_h = nz - P.NUT_COLLAR_D / 2.0  # 28.0 mm
    tray_shapes.append(box((0.028, 0.036, n_ped_h), pos=(nx, ny, n_ped_h / 2.0), color=RACK_BLUE))
    # Two rails 19.0 mm apart carry the hex on its two lower corners: the nut stands corner-up/down, flats facing +-y
    rail_h = (nz - 0.5 * P.NUT_AF * math.tan(math.pi / 6.0)) - n_ped_h  # rail top = height of the lower corners
    for side in (-1.0, 1.0):
        tray_shapes.append(box((P.NUT_HEX_LEN, 0.003, rail_h), pos=(nx - 0.0015, ny + side * (0.5 * P.NUT_AF + 0.0015), n_ped_h + rail_h / 2.0), color=RACK_BLUE))
        tray_shapes.append(box((0.016, 0.004, 0.007), pos=(nx + 0.004, ny + side * (P.NUT_COLLAR_D / 2.0 + 0.0025), n_ped_h + 0.0035), color=RACK_BLUE))
        tray_shapes.append(box((0.004, 0.018, 0.006), pos=(nx + side * (P.NUT_LEN / 2.0 + 0.003), ny, n_ped_h + 0.003), color=RACK_BLUE))

    # 5. Bare 7 mm-thick GEDORE spanner level edge cradle at WRENCH_START with 45-deg V-funnel cheeks (30 mm entry mouth!)
    wx, wy, wz = P.WRENCH_START
    shank_bottom = wz - P.WRENCH_SHANK_W / 2.0  # 39.0 mm
    tray_shapes.append(box((0.096, 0.044, 0.008), pos=(wx - 0.046, wy, 0.004), color=RACK_BLUE))
    for s_fork in (-0.028, -0.068):
        fx_pos = wx + s_fork
        tray_shapes.append(box((0.012, 0.036, shank_bottom), pos=(fx_pos, wy, shank_bottom / 2.0), color=RACK_BLUE))
        for side in (-1.0, 1.0):
            # 32-deg flared V-funnel cheeks: 12.5 mm bottom throat (> 7.0 mm shank!) widening to 35 mm at the top!
            flare = mk.axis_angle((1.0, 0.0, 0.0), -side * math.radians(32.0))
            tray_shapes.append(
                box(
                    (0.012, 0.004, 0.016),
                    pos=(fx_pos, wy + side * (P.WRENCH_T / 2.0 + 0.0090), shank_bottom + 0.006),
                    quat=_quat(flare),
                    color=RACK_BLUE,
                )
            )
    body("tray", tray_shapes, pos=(0.0, 0.0, top), kinematic=True, color=RACK_BLUE)

    # Free bodies
    body(
        "wheel_sound",
        _wheel_shapes(STONE_COARSE, cracked=False),
        pos=(sx, sy, top + sz + P.OVER),
        yaw=syaw,
        mass=P.WHEEL_MASS,
        color=STONE_COARSE,
        friction=(P.COUNTER_RUBBER_MU, 1.0),
        contact_offset=0.001,
        contacts=True,
    )
    body(
        "wheel_cracked",
        _wheel_shapes(STONE_COARSE, cracked=True),
        pos=(cx, cy, top + cz + P.OVER),
        yaw=cyaw,
        mass=P.WHEEL_MASS - 0.005,
        color=STONE_COARSE,
        friction=(P.COUNTER_RUBBER_MU, 1.0),
        contact_offset=0.001,
        contacts=True,
    )
    body(
        "outer_flange",
        _flange_shapes(),
        pos=(fx, fy, top + fz + P.OVER),
        yaw=-math.pi / 2.0,  # local +y (concave recessed face) points +x; +90 deg yaw orients it +y onto arbor!
        mass=P.FLANGE_MASS,
        color=STEEL,
        friction=(P.COUNTER_RUBBER_MU, 1.0),
        contact_offset=0.001,
        contacts=True,
    )
    body(
        "arbor_nut",
        _nut_shapes(),
        pos=(nx, ny, top + nz + P.OVER),
        yaw=-math.pi / 2.0,  # local +y (inner Ø 28 mm collar) points +x; +90 deg yaw aligns bore coaxially with +y!
        mass=P.NUT_MASS,
        color=STEEL,
        friction=(P.COUNTER_RUBBER_MU, 1.0),
        contact_offset=0.001,
        contacts=True,
    )
    weld("nut_catch", "arbor_nut", "nut_hub", made=False)

    body(
        "wrench",
        _wrench_shapes(),
        pos=(wx, wy, top + wz + P.OVER),
        quat=_quat(_ry(P.WRENCH_REST_PITCH)),
        mass=P.WRENCH_MASS,
        color=PHOSPHATE,
        friction=(P.COUNTER_RUBBER_MU, 1.0),
        friction_combine="max",  # the nitrile pad sleeves grip the phosphated shank with the sleeves' own friction
        contact_offset=0.001,
        contacts=True,
    )
    mk.sense_contact("wrench", "arbor_nut")
    mk.sense_contact("wheel_sound", "spindle", "outer_flange")
    mk.sense_contact("outer_flange", "wheel_sound", "arbor_nut")


def build(top: float) -> None:
    build_grinder(top)
    build_spindle_and_nut_hub(top)
    build_tray_and_parts(top)
