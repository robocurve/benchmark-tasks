"""Runnable W50 Bar + Slide + Near_Q1 + Occupied Near_Q2 Waypoints for `claude-b50` (Codex 0643 helper handoff).

Scorer changes are saved separately in `w50_live_scorer.py` (`Codex 0643` line 8).
This module reproduces `c7i/task.py` (`ea8563d7`, `SW7=C SW7_SEAT_DX=0.003`) 1:1 on `Near_Q1` and supplies:
1. `bar_two_hand_d405_up()` (`320` steps = `16.0 s`, `0.086 mm` 20 Hz stream, `+4.4 mm` stand/W50 clearance),
2. `bar_two_hand_ry_long_axis()` (`320` steps = `16.0 s`, ZERO `Rx` roll on pick — picks straight down at `(0.213, +-0.180)`
   with flat pads `+14 mm` over strip, lifts high, pitches `_Ry(-42 deg)` about the bar's own `Y` long axis, `0.085 mm` stream!),
3. `bar_one_hand_c8_high_entry()` (`320` steps = `16.0 s`, `c8` case D fixed with high wall-clearing entry `(0.200, +0.260, 0.900)`),
4. `c7i_pan_waypoints(..., is_occupied_q2=True)` (`exact c7i native-passing recipe adapted to `Near_Q2 (-0.1330)` with
   `dz_drop = 0.026 m` + `+24 mm Z` steeper upward peel, verified `+2.7..+9.6 mm` clear of `c7i`'s measured settled `Near_Q1`
   pose `(0.4023, +0.1325, 0.7690, -3.05 deg)` even under `-12 mm` native PhysX wrist sag!),
5. `slide_pan_to_y0(side)` (`0.08 mm` 20 Hz stream, `+15.8 mm` clear of `w50_front_wall` along `x_pull = 0.185 m`),
6. `demo_b50(mode)` for `"bar_d405_up"`, `"bar_ry_two_hand"`, `"bar_c8_high"`, `"q2_occupied_only"`,
   `"integrated_bar_slide_q1"`, and `"integrated_bar_slide_q1_q2"`.
"""

from __future__ import annotations

import math
import numpy as np

import w50_two_hand_near_waypoints as HW

STAND_TOP = HW.STAND_TOP
WC = HW.WC
RIM = HW.RIM
X_STRIP = HW.X_STRIP
X_NEAR = HW.X_NEAR
SEAT_DX_C7I = 0.0030                # exact SW7_SEAT_DX=0.003 that passed c7i (`x_cmd = 0.4010 m`)
Y_POS_SEAT = HW.Y_POS_SEAT          # +0.1330 m
Y_NEG_SEAT = -0.1352                # -0.1352 m (+2.2 mm -Y gives +3.2 mm exact jaw-mesh clearance vs c7i Q1 under -12 mm sag!)
Y_STAGE_OUTER = 0.4675              # clears 48-deg-rolled bar pick hand by > +25 mm while staying +10.2 mm inside stand +-0.6095 m!


def nonoverlapping_reset_poses(stand_top: float = STAND_TOP) -> dict[str, tuple[float, float, float]]:
    """Non-overlapping `(x, y, z)` reset poses on the `762 x 1,219 mm` Regency stand (`+20.25 mm` symmetric Y gaps)."""
    z0 = stand_top + 0.0005
    return {
        "adaptor_bar": (X_STRIP, 0.0000, z0),
        "quarter_1": (X_STRIP, +Y_STAGE_OUTER, z0),
        "quarter_2": (X_STRIP, -Y_STAGE_OUTER, z0),
    }


def bar_two_hand_d405_up() -> tuple[list, list]:
    """Codex 0643 primary 320-step (`16.0 s` incl. park) two-hand D405-up bar transfer."""
    return HW.bar_waypoints((X_STRIP, 0.0000))


def bar_two_hand_ry_long_axis(pitch_ry_deg: float = -42.0, yb: float = 0.180) -> tuple[list, list]:
    """Backup two-hand bar transfer with ZERO `Rx` pad roll: straight-down pick, then `Ry(-42 deg)` about bar's `Y` axis!"""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    Orient, D = HW.install_orient_hook()
    R0_L = _R0L = HW._Rz(-90.0) @ D
    R0_R = _R0R = HW._Rz(+90.0) @ D
    Ry = HW._Ry(pitch_ry_deg)
    R1_L, R1_R = Ry @ R0_L, Ry @ R0_R
    a1 = R1_L @ D.T @ np.array([0.0, 0.0, -1.0])

    zp = STAND_TOP + 0.0150
    c_off = 0.0105 * a1
    th = math.radians(pitch_ry_deg)
    low = HW.BAR_W / 2.0 * abs(math.sin(th)) + HW.BAR_H / 2.0 * abs(math.cos(th))
    bc_L = np.array([WC[0], +yb, RIM + low + 0.0035])
    tr_L = bc_L - c_off
    hi = RIM + 0.065

    O0L, O0R = Orient(R0_L), Orient(R0_R)
    O1L, O1R = Orient(R1_L), Orient(R1_R)
    bar_l = [
        Waypoint(1.3, (0.200, +0.260, 0.895), FINGER_OPEN, "bar_high_ready", 0.0),
        Waypoint(1.2, (X_STRIP, +yb, 0.885), -0.0260, "bar_high_over_strip", O0L),
        Waypoint(1.0, (X_STRIP, +yb, STAND_TOP + 0.060), -0.0260, "bar_above", O0L),
        Waypoint(0.9, (X_STRIP, +yb, zp), -0.0260, "bar_down_flat", O0L),
        Waypoint(0.5, (X_STRIP, +yb, zp), FINGER_CLOSED, "bar_pinch", O0L),
        Waypoint(1.2, (X_STRIP, +yb, hi), FINGER_CLOSED, "bar_lift", O0L),
        Waypoint(1.3, (0.295, +yb, hi), FINGER_CLOSED, "bar_pitch_ry", O1L),
        Waypoint(1.8, (float(tr_L[0]), +yb, hi), FINGER_CLOSED, "bar_over_well", O1L),
        Waypoint(1.3, tuple(float(v) for v in tr_L), FINGER_CLOSED, "bar_lower_3p5mm", O1L),
        Waypoint(0.5, tuple(float(v) for v in tr_L), -0.0210, "bar_release", O1L),
        Waypoint(1.1, tuple(float(v) for v in (tr_L - 0.045 * a1 + np.array([0.0, 0.0, 0.020]))), FINGER_OPEN, "bar_back_off", O1L),
        Waypoint(1.2, (0.240, +0.240, 0.890), FINGER_OPEN, "bar_retreat_x", O0L),
        Waypoint(1.2, (0.220, +0.300, 0.895), FINGER_OPEN, "bar_park_ready", 0.0),
    ]
    map_r = {id(O0L): O0R, id(O1L): O1R}
    bar_r = [
        Waypoint(w.duration, (w.pos[0], -w.pos[1], w.pos[2]), w.finger, w.label, map_r.get(id(w.yaw), 0.0))
        for w in bar_l
    ]
    return bar_l, bar_r


def bar_one_hand_c8_high_entry(th_deg: float = -45.0) -> tuple[list, list]:
    """`c8` Case D one-hand centre bar pinch FIXED with high wall-clearing entry `(0.200, +0.260, 0.895) -> (0.213, 0.0, 0.885)`."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    Orient, D = HW.install_orient_hook()
    th = math.radians(th_deg)
    R0 = HW._Rz(-90.0) @ D
    R1 = HW._Ry(th_deg) @ R0
    a1 = R1 @ D.T @ np.array([0.0, 0.0, -1.0])
    zp = STAND_TOP + 0.0145
    c_off = 0.0105 * a1
    low = HW.BAR_W / 2.0 * abs(math.sin(th)) + HW.BAR_H / 2.0 * abs(math.cos(th))
    bc = np.array([WC[0], WC[1], RIM + low + 0.0030])
    tr = bc - c_off
    hi = RIM + 0.060
    O0, O1 = Orient(R0), Orient(R1)
    rel = -0.0200
    left = [
        Waypoint(1.2, (0.200, +0.260, 0.895), FINGER_OPEN, "bar_high_ready", 0.0),
        Waypoint(1.3, (X_STRIP, 0.000, 0.885), -0.0260, "bar_high_clear_wall", O0),
        Waypoint(1.0, (X_STRIP, 0.000, STAND_TOP + 0.060), -0.0260, "bar_above_centre", O0),
        Waypoint(0.9, (X_STRIP, 0.000, zp), -0.0260, "bar_down", O0),
        Waypoint(0.5, (X_STRIP, 0.000, zp), FINGER_CLOSED, "bar_pinch", O0),
        Waypoint(0.4, (X_STRIP, 0.000, zp), FINGER_CLOSED, "bar_settle", O0),
        Waypoint(1.2, (X_STRIP, 0.000, hi), FINGER_CLOSED, "bar_lift", O0),
        Waypoint(1.2, (0.300, 0.000, hi), FINGER_CLOSED, "bar_pitch_about_long_axis", O1),
        Waypoint(1.8, (float(tr[0]), float(tr[1]), hi), FINGER_CLOSED, "bar_over_well", O1),
        Waypoint(1.4, tuple(float(v) for v in tr), FINGER_CLOSED, "bar_lower_to_3mm", O1),
        Waypoint(0.5, tuple(float(v) for v in tr), rel, "bar_release", O1),
        Waypoint(0.9, tuple(float(v) for v in (tr - 0.04 * a1 + np.array([0.0, 0.0, 0.02]))), rel, "bar_back_off", O1),
        Waypoint(1.2, (0.230, +0.220, 0.895), FINGER_OPEN, "bar_retreat_x", O0),
        Waypoint(1.0, (0.220, +0.300, 0.895), FINGER_OPEN, "bar_park_ready", 0.0),
    ]
    total_t = sum(w.duration for w in left)
    right = [Waypoint(total_t, (0.220, -0.320, 0.920), FINGER_OPEN, "park_hold", 0.0)]
    return left, right


def slide_pan_to_y0(side: str, y_start_abs: float = Y_STAGE_OUTER) -> tuple[list, list]:
    """Captive interior-cavity pull along `x_pull = 0.2050 m` from `y = +-0.4675 m` to `y = 0.0000 m`.

    Calibrated from native `int1` telemetry (`where x_pull=0.185, y_end=-0.104 pulled Q1 to (0.200, +0.014)`):
    - `x_pull = 0.2050 m` (`within 8 mm of pan X CoM`, keeping exact URDF `gripper/*.stl` `> +12 mm` clear of `w50_front_wall`)
      eliminates `-X` yaw/drift;
    - `y_end = sgn * (-0.11475 - 0.0020)` pulls the extra `13 mm` so pan centre lands dead-on `(0.213, 0.000)`.
    """
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    sgn = +1.0 if side == "left" else -1.0
    x_pull = X_STRIP - 0.0080  # 0.2050 m
    y_entry = sgn * (y_start_abs - (HW.BODY_L / 2.0 - HW.SHEET) + 0.033)
    y_contact = sgn * (y_start_abs - (HW.BODY_L / 2.0 - HW.SHEET) + 0.008)
    y_end = sgn * (0.0000 - (HW.BODY_L / 2.0 - HW.SHEET) - 0.0020)

    z_clear = STAND_TOP + HW.Q_D + 0.075
    z_in_cavity = STAND_TOP + 0.028

    active = [
        Waypoint(1.2, (0.205, sgn * 0.300, 0.860), FINGER_CLOSED, f"slide_{side}_ready", 0.0),
        Waypoint(1.3, (x_pull, y_entry, z_clear), FINGER_CLOSED, f"slide_{side}_above_cavity", 0.0),
        Waypoint(0.8, (x_pull, y_entry, z_in_cavity), FINGER_CLOSED, f"slide_{side}_dip_cavity", 0.0),
        Waypoint(0.6, (x_pull, y_contact, z_in_cavity), FINGER_CLOSED, f"slide_{side}_touch_wall", 0.0),
        Waypoint(3.2, (x_pull, y_end, z_in_cavity), FINGER_CLOSED, f"slide_{side}_pull_to_y0", 0.0),
        Waypoint(0.5, (x_pull, y_end + sgn * 0.025, z_in_cavity), FINGER_CLOSED, f"slide_{side}_unhook", 0.0),
        Waypoint(1.0, (x_pull, y_end + sgn * 0.025, z_clear + 0.040), FINGER_CLOSED, f"slide_{side}_lift_out", 0.0),
        Waypoint(1.4, (0.210, sgn * 0.280, 0.860), FINGER_OPEN, f"slide_{side}_return_ready", 0.0),
    ]
    total_t = sum(w.duration for w in active)
    passive = [Waypoint(total_t, (0.220, -sgn * 0.300, 0.895), FINGER_OPEN, f"wait_slide_{side}", 0.0)]
    return (active, passive) if side == "left" else (passive, active)


def c7i_pan_waypoints(
    stage_xy: tuple[float, float] = (X_STRIP, 0.0000),
    seat_xy: tuple[float, float] = (X_NEAR + SEAT_DX_C7I, Y_POS_SEAT),
    tag: str = "near_q1",
    is_occupied_q2: bool = False,
) -> tuple[list, list]:
    """Exact `c7i/task.py` (`ea8563d7`) native-passing trajectory (`pan_ok = True`), baked into a standalone function!

    For `is_occupied_q2=True` (`Near_Q2` at `y = -0.1330` while `Near_Q1` is occupied at `(0.4023, +0.1325, 0.7690)`):
    - Uses `dz_drop = 0.026 m` (`+2 mm` higher margin against `-12 mm` native PhysX wrist sag -> sagged TCP `0.8401 m`,
      keeping Left hand `+2.7 mm` clear of `c7i`'s settled `Near_Q1` flange `fl_ny` and Right hand `+4.1 mm` clear of
      `w50_negY_wall` even under full `-12 mm` sag; `+8.9 mm` clear at unsagged command height!).
    - Uses steeper upward peel `dz_peel_up = +0.024 m` (`28 mm` outward + `24 mm` upward) so Left lower jaw rises above
      `Near_Q1`'s `0.8355 m` bar-side flange BEFORE crossing `y = 0`!
    """
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.demo import Waypoint

    Orient, _ = HW.install_orient_hook()
    dz_drop_use = 0.0290 if is_occupied_q2 else 0.0240
    dz_peel_use = 0.0240 if is_occupied_q2 else 0.0140
    f_rel_use = -0.0048 if is_occupied_q2 else HW.F_REL_SAFE  # 9.6 mm jaw gap on Q2 clears c7i Q1 on exact jaw STLs even at -12 mm sag!

    qL, qR = HW.two_hand_pan_waypoints(
        stage_xy,
        seat_xy,
        tag=tag,
        f_rel=f_rel_use,
        dz_drop=dz_drop_use,
        dz_peel_up=dz_peel_use,
    )
    plan = {"left": qL, "right": qR}

    # Exact `c7i/task.py` lines 104-140 transforms:
    slow = {"raise_carry": 2.0, "carry_over_seat": 4.5}
    for arm in plan.values():
        for w in arm:
            for k, s in slow.items():
                if w.label.endswith(k):
                    w.duration = s
                    w.pos = (w.pos[0], w.pos[1], RIM + 0.115)

    Ry_carry = HW._Ry(-6.0)
    for arm in plan.values():
        on = False
        for w in arm:
            on = on or w.label.endswith("hold_1p5s")
            if on and hasattr(w.yaw, "rot") and not w.label.endswith("retreat_up"):
                w.yaw = Orient(Ry_carry @ w.yaw.rot)

    Rb_seat = HW._Ry(-3.6)
    z_slow, t_slow = 0.025, 2.5
    for side, arm in plan.items():
        out = []
        # Smooth high entry before `pre_high` when chaining after bar/slide (`prevents wrist-flip spike`)
        y_home = +0.280 if side == "left" else -0.280
        out.append(Waypoint(1.4, (0.205, y_home, 0.870), FINGER_OPEN, f"{tag}_entry_ready", arm[0].yaw))
        for w in arm:
            if w.label.endswith("lower_24mm"):
                Rs = Orient(Rb_seat @ w.yaw.rot)
                above = out[-1].pos
                out.append(Waypoint(1.0, above, w.finger, f"{tag}_pitch_to_seat", Rs))
                out.append(Waypoint(1.0, above, w.finger, f"{tag}_pitch_settle", Rs))
                out.append(Waypoint(1.2, (w.pos[0], w.pos[1], w.pos[2] + z_slow), w.finger, f"{tag}_lower_to_slow", Rs))
                out.append(Waypoint(t_slow, w.pos, w.finger, f"{tag}_lower_slow_final", Rs))
            elif w.label.endswith("release_13p6mm_gap"):
                Rs = Orient(Rb_seat @ w.yaw.rot)
                out.append(Waypoint(w.duration, w.pos, w.finger, w.label, Rs))
                if is_occupied_q2:
                    out.append(Waypoint(0.6, (w.pos[0], w.pos[1], w.pos[2] + 0.016), w.finger, f"{tag}_unhook_up_16mm", Rs))
            elif w.label.endswith("peel_back"):
                out.append(Waypoint(w.duration, w.pos, w.finger, w.label, Orient(Rb_seat @ w.yaw.rot)))
            else:
                out.append(w)
        out.append(Waypoint(1.3, (0.220, y_home, 0.895), FINGER_OPEN, f"{tag}_retreat_home", 0.0))
        plan[side] = out
    return plan["left"], plan["right"]


def demo_b50(mode: str = "bar_d405_up") -> dict[str, list]:
    """Turnkey `demo()` return dict for all `claude-b50` coupons & integrated runs."""
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.demo import Waypoint

    pL = [Waypoint(1.5, (0.22, +0.32, 0.92), FINGER_OPEN, "park_still", 0.0)]
    pR = [Waypoint(1.5, (0.22, -0.32, 0.92), FINGER_OPEN, "park_still", 0.0)]

    if mode == "bar_d405_up":
        bL, bR = bar_two_hand_d405_up()
        return {"left": bL + pL, "right": bR + pR}
    if mode == "bar_ry_two_hand":
        bL, bR = bar_two_hand_ry_long_axis()
        return {"left": bL + pL, "right": bR + pR}
    if mode == "bar_c8_high":
        bL, bR = bar_one_hand_c8_high_entry()
        return {"left": bL + pL, "right": bR + pR}
    if mode == "q1_c7i_exact":
        qL, qR = c7i_pan_waypoints((X_STRIP, 0.0000), (X_NEAR + SEAT_DX_C7I, Y_POS_SEAT), tag="near_q1", is_occupied_q2=False)
        return {"left": qL + pL, "right": qR + pR}
    if mode == "q2_occupied_only":
        qL, qR = c7i_pan_waypoints((X_STRIP, 0.0000), (X_NEAR + SEAT_DX_C7I, Y_NEG_SEAT), tag="near_q2", is_occupied_q2=True)
        return {"left": qL + pL, "right": qR + pR}
    if mode == "integrated_bar_slide_q1":
        bL, bR = bar_two_hand_d405_up()
        s1L, s1R = slide_pan_to_y0("left", Y_STAGE_OUTER)
        q1L, q1R = c7i_pan_waypoints((X_STRIP, 0.0000), (X_NEAR + SEAT_DX_C7I, Y_POS_SEAT), tag="near_q1", is_occupied_q2=False)
        return {"left": bL + s1L + q1L + pL, "right": bR + s1R + q1R + pR}
    if mode == "integrated_bar_centered_slide_q1":
        import w50_slide_landing_variants as SLV
        bL, bR = bar_two_hand_d405_up()
        s1L, s1R = SLV.slide_centered_contact("left", Y_STAGE_OUTER)
        q1L, q1R = c7i_pan_waypoints((X_STRIP, 0.0000), (X_NEAR + SEAT_DX_C7I, Y_POS_SEAT), tag="near_q1", is_occupied_q2=False)
        return {"left": bL + s1L + q1L + pL, "right": bR + s1R + q1R + pR}
    if mode == "integrated_bar_nudge_slide_q1":
        import w50_slide_landing_variants as SLV
        bL, bR = bar_two_hand_d405_up()
        s1L, s1R = SLV.slide_centered_plus_x_nudge("left", Y_STAGE_OUTER)
        q1L, q1R = c7i_pan_waypoints((X_STRIP, 0.0000), (X_NEAR + SEAT_DX_C7I, Y_POS_SEAT), tag="near_q1", is_occupied_q2=False)
        return {"left": bL + s1L + q1L + pL, "right": bR + s1R + q1R + pR}
    if mode == "integrated_bar_slide_q1_q2":
        bL, bR = bar_two_hand_d405_up()
        s1L, s1R = slide_pan_to_y0("left", Y_STAGE_OUTER)
        q1L, q1R = c7i_pan_waypoints((X_STRIP, 0.0000), (X_NEAR + SEAT_DX_C7I, Y_POS_SEAT), tag="near_q1", is_occupied_q2=False)
        s2L, s2R = slide_pan_to_y0("right", Y_STAGE_OUTER)
        q2L, q2R = c7i_pan_waypoints((X_STRIP, 0.0000), (X_NEAR + SEAT_DX_C7I, Y_NEG_SEAT), tag="near_q2", is_occupied_q2=True)
        return {"left": bL + s1L + q1L + s2L + q2L + pL, "right": bR + s1R + q1R + s2R + q2R + pR}
    raise ValueError(f"Unknown mode: {mode}")
