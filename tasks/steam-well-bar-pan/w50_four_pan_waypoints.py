"""Standalone Avantco W50 + Vollrath 75020 + 4 x Vollrath 30422 Waypoint & Scorer Helper (antigravity helper).

Delivers all helper items requested in 0509 / 0522 / 0543:
1. `seated_four_pan_poses(wc_xy, z_rim, y_mag=0.1330, x_mag=0.0810)`:
   - Corrected `Ry(+chan_sign * 2.834 deg)` cross-tilt lifting inner flanges +8.0 mm onto the 75020 bar while
     keeping outer flanges flush on the W50 31.75 mm flat rim.
2. `four_pan_waypoints(...)`:
   - Complete synchronized `{"left": [...], "right": [...]}` `Waypoint` trajectories covering:
     * Bimanual `75020` bar transfer (lifting to `z_rim + 0.060 m` BEFORE `+X` carry to clear the W50 front wall),
     * Far-channel `Q1` (Left, +Y) and `Q2` (Right, -Y) while the near channel is still empty,
     * Final near-channel `Q3` (Left, +Y) and `Q4` (Right, -Y),
     * Controlled jaw release `F_RELEASE_FLANGE = -0.0065 m` (13 mm symmetric opening around the 2 mm flange at
       `z_drop = z_rim + 0.025 m`, avoiding the `-46.95 mm` lower-jaw plunge into the 75020 bar / rim),
     * Clean `>= 80 mm` park retreat + `1.2 s` stillness hold.
3. `bar_seated_strict`, `pan_ok_bar_frame`, and `score_bar_frame`:
   - Evaluates both bar end-land patches and inner-flange check points in the ACTUAL rotated `bar_R` frame
     (`16 deg` yawed bar -> `0.0`) and enforces occupied/duplicate-quadrant zero (`any_duplicate_zero=True` -> `0.0`).
"""

from __future__ import annotations

import math
import numpy as np

W_OUT = (0.3683, 0.5715, 0.2286)
W_IN = (0.3048, 0.5080, 0.15875)
BAR_W, BAR_L, BAR_H = 0.0254, 0.5310, 0.0080
Q_W, Q_L, Q_D, Q_FL, SHEET = 0.1618, 0.2635, 0.0635, 0.0150, 0.0020

F_RELEASE_FLANGE = -0.0065  # 13.0 mm symmetric opening around 2.0 mm flange (prevents -47 mm lower-jaw bar hit!)


def _Rx(d: float) -> np.ndarray:
    a = math.radians(d)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]], dtype=np.float64)


def _Ry(d: float) -> np.ndarray:
    a = math.radians(d)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]], dtype=np.float64)


def _Rz(d: float) -> np.ndarray:
    a = math.radians(d)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def seated_pan_pose(
    wc_xy: tuple[float, float],
    z_rim: float,
    chan_sign: float,
    y_off: float,
    x_mag: float = 0.0810,
) -> tuple[np.ndarray, np.ndarray]:
    """Return `(pos_w, R_w)` for one seated Vollrath 30422 quarter pan (`chan_sign = +1` far, `-1` near)."""
    alpha_deg = math.degrees(math.asin(BAR_H / Q_W))  # +2.834 deg
    R = _Ry(+chan_sign * alpha_deg)
    cx = wc_xy[0] + chan_sign * x_mag
    cy = wc_xy[1] + y_off
    z_out_rel = float((R @ np.array([chan_sign * Q_W / 2.0, 0.0, Q_D - SHEET]))[2])
    cz = z_rim - z_out_rel
    return np.array([cx, cy, cz], dtype=np.float64), R


def seated_four_pan_poses(
    wc_xy: tuple[float, float] = (0.35, 0.0),
    z_rim: float = 0.9786,
    y_mag: float = 0.1330,
    x_mag: float = 0.0810,
) -> list[tuple[str, np.ndarray, np.ndarray]]:
    """Return `[("Q1_far_posY", p, R), ("Q2_far_negY", p, R), ("Q3_near_posY", p, R), ("Q4_near_negY", p, R)]`."""
    quads = [
        ("Q1_far_posY", +1.0, +y_mag),
        ("Q2_far_negY", +1.0, -y_mag),
        ("Q3_near_posY", -1.0, +y_mag),
        ("Q4_near_negY", -1.0, -y_mag),
    ]
    return [(nm, *seated_pan_pose(wc_xy, z_rim, cs, ym, x_mag=x_mag)) for nm, cs, ym in quads]


def bar_seated_strict(bar_pos, bar_R, wc_xy, rim_z, land_min=0.005, gap_tol=0.003, yaw_tol_deg=5.0) -> bool:
    """Codex 0543 line 6: verify BOTH bar end-land patches in the actual `bar_R` frame on the W50 end rims."""
    bp = np.asarray(bar_pos, dtype=np.float64)
    Rb = np.asarray(bar_R, dtype=np.float64)
    tilt = math.degrees(math.acos(float(np.clip(Rb[2, 2], -1.0, 1.0))))
    if tilt > 2.0 or abs(float(bp[0]) - wc_xy[0]) > 0.010:
        return False
    ax = Rb @ np.array([0.0, 1.0, 0.0])
    if math.degrees(math.acos(min(1.0, abs(float(ax[1])) / max(float(np.linalg.norm(ax[:2])), 1e-9)))) > yaw_tol_deg:
        return False
    for sy in (-1.0, +1.0):
        for sx in (-1.0, +1.0):
            under = bp + Rb @ np.array([sx * BAR_W / 2.0, sy * BAR_L / 2.0, 0.0])
            if abs(float(under[2]) - rim_z) > gap_tol:
                return False
            dy = abs(float(under[1]) - wc_xy[1])
            if dy < W_IN[1] / 2.0 + land_min or dy > W_OUT[1] / 2.0 + 0.005:
                return False
            if abs(float(under[0]) - wc_xy[0]) > W_IN[0] / 2.0:
                return False
    return True


def pan_ok_bar_frame(pos, R, wc_xy, rim_z, bar_pos, bar_R, gap_tol=0.003, min_overlap=0.005) -> bool:
    """Codex 0543 line 6: evaluate inner-flange check points in the ACTUAL rotated `bar_R` frame."""
    p = np.asarray(pos, dtype=np.float64)
    Rp = np.asarray(R, dtype=np.float64)
    bp = np.asarray(bar_pos, dtype=np.float64)
    Rb = np.asarray(bar_R, dtype=np.float64)
    if math.degrees(math.acos(float(np.clip(Rp[2, 2], -1.0, 1.0)))) > 5.0:
        return False
    for sx in (-1.0, +1.0):
        for sy in (-1.0, +1.0):
            c = p + Rp @ np.array([sx * (Q_W / 2.0 - Q_FL), sy * (Q_L / 2.0 - Q_FL), 0.0])
            if abs(float(c[0]) - wc_xy[0]) > W_IN[0] / 2.0 or abs(float(c[1]) - wc_xy[1]) > W_IN[1] / 2.0:
                return False
    side = 1.0 if float(p[0]) < wc_xy[0] else -1.0
    rim_edge_x = wc_xy[0] - side * (W_IN[0] / 2.0)
    for yl in (-0.8 * Q_L / 2.0, +0.8 * Q_L / 2.0):
        e_out = p + Rp @ np.array([side * Q_W / 2.0, yl, Q_D - SHEET])
        e_in = p + Rp @ np.array([side * (Q_W / 2.0 - Q_FL), yl, Q_D - SHEET])
        lo = Rb.T @ (e_out - bp)
        li = Rb.T @ (e_in - bp)
        if abs(float(lo[1])) > BAR_L / 2.0 or abs(float(lo[2]) - BAR_H) > gap_tol:
            return False
        f0, f1 = sorted((float(li[0]), float(lo[0])))
        if min(f1, BAR_W / 2.0) - max(f0, -BAR_W / 2.0) < min_overlap:
            return False
        o_under = p + Rp @ np.array([-side * Q_W / 2.0, yl, Q_D - SHEET])
        if abs(float(o_under[2]) - rim_z) > gap_tol:
            return False
        o_ov = (rim_edge_x - float(o_under[0])) if side > 0 else (float(o_under[0]) - rim_edge_x)
        if o_ov < min_overlap:
            return False
    return True


def score_bar_frame(pans, bar, wc_xy, rim_z, any_duplicate_zero: bool = True) -> float:
    """Return `1.0` (4 distinct quadrants), `0.5` (1 complete channel, no duplicate), or `0.0`."""
    if not bar_seated_strict(bar[0], bar[1], wc_xy, rim_z):
        return 0.0
    quad = lambda p: (float(p[0]) < wc_xy[0], float(p[1]) < wc_xy[1])
    if any_duplicate_zero:
        in_well = [
            quad(p)
            for p, _ in pans
            if abs(float(p[0]) - wc_xy[0]) < W_OUT[0] / 2.0 and abs(float(p[1]) - wc_xy[1]) < W_OUT[1] / 2.0
        ]
        if len(in_well) != len(set(in_well)):
            return 0.0
    ok = [pan_ok_bar_frame(p, R, wc_xy, rim_z, bar[0], bar[1]) for p, R in pans]
    good = [quad(p) for (p, _), k in zip(pans, ok) if k]
    if len(set(good)) == 4:
        return 1.0
    near = sum(1 for q in set(good) if q[0])
    far = sum(1 for q in set(good) if not q[0])
    return 0.5 if (near == 2 or far == 2) else 0.0


def _install_orient_hook():
    import yam_arena.demo as _demo

    class Orient(float):
        def __new__(cls, rot):
            obj = float.__new__(cls, 0.0)
            obj._rot = np.asarray(rot, dtype=np.float64)
            return obj

        @property
        def rot(self):
            return self._rot

    orig = _demo.grip_rotation
    if not getattr(orig, "_orient", False):
        def _grip_rotation(yaw=0.0):
            return yaw.rot if hasattr(yaw, "rot") else orig(yaw)
        _grip_rotation._orient = True
        _demo.grip_rotation = _grip_rotation
    return Orient, _demo.grip_rotation(0.0)


def four_pan_waypoints(
    wc_xy: tuple[float, float] = (0.35, 0.0),
    z_rim: float = 0.9786,
    z_stage_surface: float = 0.7500,
    stage_xy_map: dict[str, tuple[float, float]] | None = None,
    bar_stage_xy: tuple[float, float] | None = None,
    x_mag: float = 0.0810,
    y_mag: float = 0.1330,
) -> dict[str, list]:
    """Build complete synchronized Left + Right `Waypoint` lists for Bar + Q1 + Q2 + Q3 + Q4 + Park.

    Sequence order:
    1. Bimanual `75020` bar pick -> lift above `z_rim + 60 mm` (clearing W50 front wall) -> seat on end rims -> retreat.
    2. Simultaneous / paired far-channel transfer (`Q1_far_posY` via Left arm + `Q2_far_negY` via Right arm) while the
       near channel is empty, using controlled symmetric release `F_RELEASE_FLANGE = -0.0065 m` at `z_rim + 25 mm`.
    3. Simultaneous / paired near-channel transfer (`Q3_near_posY` via Left arm + `Q4_near_negY` via Right arm) with
       the same controlled release and `-X` slide-out into free air outside the W50 `-X` rim.
    4. Retreat both hands `>= 100 mm` clear to `park_l` / `park_r` and hold still (`1.5 s`).
    """
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    Orient, D = _install_orient_hook()
    R_bar = Orient(_Rz(90.0) @ D)
    R_side = Orient(_Rx(90.0) @ _Ry(-90.0) @ D)

    if stage_xy_map is None:
        stage_xy_map = {
            "Q1_far_posY": (0.35, +0.43),
            "Q2_far_negY": (0.35, -0.43),
            "Q3_near_posY": (0.22, +0.43),
            "Q4_near_negY": (0.22, -0.43),
        }
    if bar_stage_xy is None:
        bar_stage_xy = (wc_xy[0] - W_OUT[0] / 2.0 - 0.050, 0.0)

    park_l = (0.18, +0.34, z_rim + 0.080)
    park_r = (0.18, -0.34, z_rim + 0.080)

    # Phase 1: Bimanual 75020 bar transfer (clears W50 front wall by lifting to z_rim + 0.060 m BEFORE moving +X)
    yb = 0.19
    zb_pick = z_stage_surface + 0.0145
    zb_place = z_rim + 0.0145
    bar_l = [
        Waypoint(1.4, (bar_stage_xy[0], +yb, z_stage_surface + 0.055), FINGER_OPEN, "bar_pre", R_bar),
        Waypoint(0.9, (bar_stage_xy[0], +yb, zb_pick), FINGER_OPEN, "bar_down", R_bar),
        Waypoint(0.5, (bar_stage_xy[0], +yb, zb_pick), FINGER_CLOSED, "bar_pinch", R_bar),
        Waypoint(1.1, (bar_stage_xy[0], +yb, z_rim + 0.060), FINGER_CLOSED, "bar_lift_clear_wall", R_bar),
        Waypoint(1.5, (wc_xy[0], +yb, z_rim + 0.060), FINGER_CLOSED, "bar_carry_over_well", R_bar),
        Waypoint(1.0, (wc_xy[0], +yb, zb_place), FINGER_CLOSED, "bar_seat_end_rims", R_bar),
        Waypoint(0.5, (wc_xy[0], +yb, zb_place), FINGER_OPEN, "bar_release", R_bar),
        Waypoint(0.9, (wc_xy[0], +yb, z_rim + 0.075), FINGER_OPEN, "bar_retreat_up", R_bar),
    ]
    bar_r = [Waypoint(w.duration, (w.pos[0], -w.pos[1], w.pos[2]), w.finger, w.label, w.yaw) for w in bar_l]

    def _single_pan_seq(nm: str, cs: float, ym: float) -> list[Waypoint]:
        sx, sy = stage_xy_map[nm]
        cx = wc_xy[0] + cs * x_mag
        cy = wc_xy[1] + ym
        gx_st = sx - Q_W / 2.0 - 0.0018
        gx_se = cx - Q_W / 2.0 - 0.0018
        zf_st = z_stage_surface + Q_D - SHEET / 2.0
        z_carry = z_rim + 0.095
        z_drop = z_rim + 0.025  # keeps lower jaw at F_RELEASE_FLANGE clear above 8.0 mm 75020 bar & W50 rim!
        return [
            Waypoint(1.3, (gx_st - 0.045, sy, zf_st + 0.030), FINGER_OPEN, f"{nm}_pre", R_side),
            Waypoint(0.7, (gx_st - 0.045, sy, zf_st), F_RELEASE_FLANGE, f"{nm}_align", R_side),
            Waypoint(0.8, (gx_st, sy, zf_st), F_RELEASE_FLANGE, f"{nm}_slide_on", R_side),
            Waypoint(0.5, (gx_st, sy, zf_st), FINGER_CLOSED, f"{nm}_pinch", R_side),
            Waypoint(1.0, (gx_st, sy, z_carry), FINGER_CLOSED, f"{nm}_lift", R_side),
            Waypoint(1.4, (gx_se, cy, z_carry), FINGER_CLOSED, f"{nm}_carry", R_side),
            Waypoint(1.0, (gx_se, cy, z_drop), FINGER_CLOSED, f"{nm}_lower_25mm", R_side),
            Waypoint(0.5, (gx_se, cy, z_drop), F_RELEASE_FLANGE, f"{nm}_release_controlled", R_side),
            Waypoint(0.8, (gx_se - 0.045, cy, z_drop + 0.004), F_RELEASE_FLANGE, f"{nm}_slide_out_minus_x", R_side),
            Waypoint(0.9, (gx_se - 0.045, cy, z_carry), FINGER_OPEN, f"{nm}_retreat_up", R_side),
        ]

    # Phase 2: Far channel (Q1 Left + Q2 Right) while near channel is empty
    q1_l = _single_pan_seq("Q1_far_posY", +1.0, +y_mag)
    q2_r = _single_pan_seq("Q2_far_negY", +1.0, -y_mag)

    # Phase 3: Final near channel (Q3 Left + Q4 Right)
    q3_l = _single_pan_seq("Q3_near_posY", -1.0, +y_mag)
    q4_r = _single_pan_seq("Q4_near_negY", -1.0, -y_mag)

    # Phase 4: Clean >= 80 mm park & 1.5 s stillness hold
    end_l = [Waypoint(1.2, park_l, FINGER_OPEN, "park_retreat", 0.0), Waypoint(1.5, park_l, FINGER_OPEN, "park_still", 0.0)]
    end_r = [Waypoint(1.2, park_r, FINGER_OPEN, "park_retreat", 0.0), Waypoint(1.5, park_r, FINGER_OPEN, "park_still", 0.0)]

    return {
        "left": bar_l + q1_l + q3_l + end_l,
        "right": bar_r + q2_r + q4_r + end_r,
    }
