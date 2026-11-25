"""Codex 0827 Isolated Occupied-Neighbor (`Near_Q2`, `Far_Q1`, `Far_Q2`, and `Near-next-to-Far`) Motion & Named-Pair Contact Audit.

Addresses `20261121-0827-codex-w50-guard-ack-transition.md` Bullet 3:
- Continues actual occupied `Near_Q2` and `Far`-pan (`Far_Q1`, `Far_Q2`, plus `Near_Q1` next to occupied `Far_Q1/Q2`) motions
  in strict isolation from staged Q1 (`task_w50_owner_int7_ready.py`); defers all packet text drafting.
- `NamedPairContactAuditor` retains exact distal (`{L/R}:{tip_left,tip_right}`) vs housing (`{L/R}:{link1..link5,gripper}`) vs
  pan-body (`{active_pan}:{subpart}->{neighbor}:{subpart}`) pair names at every 20 Hz step and explicitly distinguishes
  phase-specific **intentional push/grasp surfaces** from **unintended contacts** (overhanging `-X` flange during under-flange
  push, occupied neighbor pans, `adaptor_bar`, `W50` walls/rims, `stand_top`, and opposite arm).
"""

from __future__ import annotations

import json
import math
import pathlib
import struct
import sys
import xml.etree.ElementTree as ET
import numpy as np

STAND_TOP = 0.5975
RIM = 0.8261
X_STRIP = 0.2130
X_NEAR_2PAN = 0.4010      # c7i Near_Q1/Q2 target when Far channel is empty (settled x=0.4023)
X_NEAR_4PAN = 0.3990      # 4-pan Near target sharing 25.4 mm 75020 bar with Far_Q1/Q2 (+2.60 mm flange-to-flange gap!)
X_FAR_SEAT = 0.5625       # Far channel target (+4.0 mm back-wall body clearance, 11.6 mm bar bearing)
Y_POS_SEAT = +0.1330
Y_NEG_SEAT = -0.1352      # +4.20 mm flange-to-flange gap to occupied +Y neighbor (+1.75 mm -Y end-wall clearance)

Q_W, Q_L, Q_D, Q_FL, SHEET = 0.1618, 0.2635, 0.0635, 0.0150, 0.0020
BODY_W, BODY_L = Q_W - 2.0 * Q_FL, Q_L - 2.0 * Q_FL
BODY_HALF_X, FLANGE_HALF_X = BODY_W / 2.0, Q_W / 2.0

HOUSING_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper")
DISTAL_LINKS = ("tip_left", "tip_right")
ALL_LINKS = HOUSING_LINKS + DISTAL_LINKS


def _Ry_deg(d: float) -> np.ndarray:
    a = math.radians(d)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]], float)


def _Rx_deg(d: float) -> np.ndarray:
    a = math.radians(d)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]], float)


def _Rz_deg(d: float) -> np.ndarray:
    a = math.radians(d)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]], float)


def _rpy_to_R(rpy) -> np.ndarray:
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    return _Rz_deg(math.degrees(y)) @ _Ry_deg(math.degrees(p)) @ _Rx_deg(math.degrees(r))


def side_deck_pull_minus_x(side: str = "right", x_deck: float = 0.4500, y_abs: float = 0.4675) -> tuple[list, list]:
    """Pulls a side-deck staged pan `(0.4500, +-0.4675)` in `-X` onto the front-strip outer stage `(0.2130, +-0.4675)`."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    sgn = +1.0 if side == "left" else -1.0
    wall_inner_x = BODY_HALF_X - SHEET  # 0.06390 m
    finger_half_x = 0.02004             # closed finger +-X face in D_DOWN(0)
    x_entry = x_deck - wall_inner_x + finger_half_x + 0.020
    x_contact = x_deck - wall_inner_x + finger_half_x + 0.002
    x_end = X_STRIP - wall_inner_x + finger_half_x          # pulls -X interior wall so pan center lands at x = 0.2130 m
    z_hi = STAND_TOP + Q_D + 0.075
    z_in = STAND_TOP + 0.028

    act = [
        Waypoint(1.0, (0.240, sgn * 0.340, 0.860), FINGER_CLOSED, f"side_pull_{side}_ready", 0.0),
        Waypoint(1.1, (x_entry, sgn * y_abs, z_hi), FINGER_CLOSED, f"side_pull_{side}_above", 0.0),
        Waypoint(0.7, (x_entry, sgn * y_abs, z_in), FINGER_CLOSED, f"side_pull_{side}_dip", 0.0),
        Waypoint(0.5, (x_contact, sgn * y_abs, z_in), FINGER_CLOSED, f"side_pull_{side}_touch_nx", 0.0),
        Waypoint(2.2, (x_end, sgn * y_abs, z_in), FINGER_CLOSED, f"side_pull_{side}_pull_to_strip", 0.0),
        Waypoint(0.5, (x_end + 0.022, sgn * y_abs, z_in), FINGER_CLOSED, f"side_pull_{side}_unhook", 0.0),
        Waypoint(0.8, (x_end + 0.022, sgn * y_abs, z_hi + 0.040), FINGER_CLOSED, f"side_pull_{side}_lift", 0.0),
        Waypoint(1.0, (0.210, sgn * 0.300, 0.860), FINGER_OPEN, f"side_pull_{side}_done", 0.0),
    ]
    T = sum(w.duration for w in act)
    wait = [Waypoint(T, (0.220, -sgn * 0.300, 0.895), FINGER_OPEN, f"wait_side_pull_{side}", 0.0)]
    return (act, wait) if side == "left" else (wait, act)


def two_hand_seat_waypoints_isolated(
    stage_xy: tuple[float, float],
    seat_xy: tuple[float, float],
    channel: str,
    is_occupied_q2: bool,
    tag: str,
) -> tuple[list, list]:
    """Two-hand end-flange pick at `stage_xy = (0.2130, 0.0000)`, high carry, slope-matched descent, and shared-seam-safe exit.

    - `channel == 'near'`: `yaw = +-10 deg, pitch = 60 deg`, wrist carry tilt `Ry(-6.0 deg)` + seat slope `Ry(-3.6 deg) = Ry(-9.6 deg)`.
    - `channel == 'far'`:  `yaw = +-20 deg, pitch = 55 deg` (`+38.5 mm` stage `link4` margin & `+16.4 mm` occupied `Far_Q1` margin!),
      wrist carry tilt `Ry(-6.0 deg)` + far-seat `+2.8 deg` slope = `Ry(-3.2 deg)`.
    - When `is_occupied_q2=True` (`y_seat = -0.1352 m` next to an occupied `+Y` neighbor at `y = +0.1325..+0.1330 m`),
      exits via **Shared-Seam Straight-Up Unhook**: opens to narrow `9.6 mm` gap (`f = -0.0048 m`) and lifts **straight up** in
      `+Z` to `z = 0.900 m` BEFORE opening to `FINGER_OPEN` or peeling in `Y`, so `L:tip_left` never moves toward the `+Y` neighbor!
    """
    import w50_two_hand_near_waypoints as HW
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    Orient, D_DOWN = HW.install_orient_hook()
    if channel == "near":
        yaw_a, pitch_a = 10.0, 60.0
        R_carry_tilt = _Ry_deg(-6.0)
        R_seat_tilt = _Ry_deg(-9.6)
        z_carry = RIM + 0.115
        z_seat_tcp = RIM + (0.0290 if is_occupied_q2 else 0.0240)
    else:
        yaw_a, pitch_a = 20.0, 55.0
        R_carry_tilt = _Ry_deg(-6.0)
        R_seat_tilt = _Ry_deg(-3.2)
        z_carry = RIM + 0.095          # 0.9211 m over front wall (pan bottom +32.5 mm above W50 rim, <= 0.09 mm IK!)
        z_above_seat = RIM + 0.076     # 0.9021 m over far seat (x=0.5625) -> <= 0.09 mm IK in both Oc & Os, pan-bottom +14.5 mm over rim!
        z_seat_tcp = RIM + 0.0300

    RL0 = _Rz_deg(+yaw_a) @ _Rx_deg(-pitch_a) @ D_DOWN
    RR0 = _Rz_deg(-yaw_a) @ _Rx_deg(+pitch_a) @ D_DOWN
    aL0 = RL0 @ D_DOWN.T @ np.array([0.0, 0.0, -1.0])
    aR0 = RR0 @ D_DOWN.T @ np.array([0.0, 0.0, -1.0])

    zf = STAND_TOP + 0.0005 + Q_D - SHEET / 2.0
    bite = 0.0085
    gL = np.array([stage_xy[0], stage_xy[1] + Q_L / 2.0 - bite, zf])
    gR = np.array([stage_xy[0], stage_xy[1] - Q_L / 2.0 + bite, zf])
    sL = np.array([seat_xy[0], seat_xy[1] + Q_L / 2.0 - bite, z_seat_tcp])
    sR = np.array([seat_xy[0], seat_xy[1] - Q_L / 2.0 + bite, z_seat_tcp])

    f_pre = -0.0080
    f_rel = -0.0048 if is_occupied_q2 else -0.0068

    if channel == "near":
        z_above_seat = z_carry

    def build_side(g, s, a0, R0, park_y):
        O0 = Orient(R0)
        Oc = Orient(R_carry_tilt @ R0)
        Os = Orient(R_seat_tilt @ R0)
        wps = [
            Waypoint(1.2, tuple(g - 0.028 * a0 + np.array([0.0, 0.0, 0.115])), FINGER_OPEN, f"{tag}_pre_high", O0),
            Waypoint(1.0, tuple(g - 0.028 * a0), f_pre, f"{tag}_pre_line_up", O0),
            Waypoint(0.9, tuple(g), f_pre, f"{tag}_slide_on", O0),
            Waypoint(0.6, tuple(g), FINGER_CLOSED, f"{tag}_pinch", O0),
            Waypoint(1.0, tuple(g + np.array([0.0, 0.0, 0.035])), FINGER_CLOSED, f"{tag}_lift_35mm", O0),
            Waypoint(1.2, tuple(g + np.array([0.0, 0.0, 0.035])), FINGER_CLOSED, f"{tag}_hold_1p5s", Oc),
            Waypoint(2.0, (float(g[0]), float(g[1]), z_carry), FINGER_CLOSED, f"{tag}_raise_carry", Oc),
        ]
        if channel == "far":
            wps.append(Waypoint(2.0, (0.380, float(s[1]), z_carry), FINGER_CLOSED, f"{tag}_cross_front_rim", Oc))
            wps.append(Waypoint(2.4, (float(s[0]), float(s[1]), z_above_seat), FINGER_CLOSED, f"{tag}_carry_over_seat", Oc))
        else:
            wps.append(Waypoint(4.2, (float(s[0]), float(s[1]), z_above_seat), FINGER_CLOSED, f"{tag}_carry_over_seat", Oc))
        wps.extend([
            Waypoint(1.0, (float(s[0]), float(s[1]), z_above_seat), FINGER_CLOSED, f"{tag}_pitch_to_seat", Os),
            Waypoint(1.2, (float(s[0]), float(s[1]), float(s[2]) + 0.025), FINGER_CLOSED, f"{tag}_lower_to_slow", Os),
            Waypoint(2.4, tuple(s), FINGER_CLOSED, f"{tag}_lower_slow_final", Os),
            Waypoint(0.6, tuple(s), f_rel, f"{tag}_release_controlled", Os),
        ])
        x_exit_hi = min(float(s[0]), 0.500)
        if is_occupied_q2:
            # Shared-seam straight-up unhook: lift straight up in +Z with narrow 9.6 mm jaw gap BEFORE opening or retreating!
            wps.extend([
                Waypoint(0.8, (float(s[0]), float(s[1]), float(s[2]) + 0.022), f_rel, f"{tag}_unhook_straight_up", Os),
                Waypoint(1.0, (x_exit_hi, float(s[1]), 0.912), f_rel, f"{tag}_rise_above_neighbors", Oc),
                Waypoint(1.3, (0.220, park_y, 0.930), FINGER_OPEN, f"{tag}_retreat_park", 0.0),
            ])
        else:
            a_s = (R_seat_tilt @ R0) @ D_DOWN.T @ np.array([0.0, 0.0, -1.0])
            wps.extend([
                Waypoint(0.8, tuple(s - 0.028 * a_s + np.array([0.0, 0.0, 0.016])), f_rel, f"{tag}_peel_back", Os),
                Waypoint(1.0, (x_exit_hi, float(s[1]), 0.912), FINGER_OPEN, f"{tag}_retreat_up", Oc),
                Waypoint(1.3, (0.220, park_y, 0.930), FINGER_OPEN, f"{tag}_retreat_park", 0.0),
            ])
        return wps

    return build_side(gL, sL, aL0, RL0, +0.320), build_side(gR, sR, aR0, RR0, -0.320)


def is_intentional_distal_surface(arm: str, label: str, obj_name: str, subpart: str, active_pan: str) -> bool:
    """Distinguishes intentional distal push/grasp bearing surfaces from all unintended contacts (`Codex 0827` Bullet 3).

    Notice: `pan_fl_nx` (overhanging `-X` flange) is NEVER marked intentional during `sqA_*` under-flange pushes,
    and occupied neighbor pans (`obj_name != active_pan`) are NEVER marked intentional!
    """
    if obj_name != active_pan:
        return False
    if "side_pull_" in label and any(k in label for k in ("dip", "touch_nx", "pull_to_strip", "unhook")):
        return subpart in ("pan_w_nx", "pan_fl_nx", "pan_bot")
    if "slide_" in label and any(k in label for k in ("dip", "touch_wall", "pull_to_y0", "unhook")):
        return subpart in ("pan_w_ny", "pan_w_py", "pan_fl_ny", "pan_fl_py", "pan_bot")
    if "yguard_" in label and any(k in label for k in ("dip", "nudge", "unhook")):
        return subpart in ("pan_w_ny", "pan_w_py", "pan_fl_ny", "pan_fl_py", "pan_bot")
    if "sqA_" in label and any(k in label for k in ("push_plus_x", "hold_wall", "retract")):
        return subpart == "pan_w_nx"  # ONLY the broad vertical body wall; overhanging `pan_fl_nx` stays UNINTENDED!
    if any(k in label for k in ("line_up", "slide_on", "pinch", "settle", "lift_", "hold_1p5s", "raise_carry", "cross_front_rim", "carry_over_seat", "pitch_to_seat", "pitch_settle", "lower_", "release_", "unhook_straight_up", "peel_back")):
        if arm == "L":
            return subpart in ("pan_fl_py", "pan_w_py")
        if arm == "R":
            return subpart in ("pan_fl_ny", "pan_w_ny")
    return False


if __name__ == "__main__":
    sys.path.insert(0, r"C:/Users/jingy/AppData/Local/Temp/robocurve-gpt-yam-offline")
    import yam_arena

    URDF_PATH = pathlib.Path(r"C:/Users/jingy/AppData/Local/Temp/robocurve-gpt-yam-hand-reference/yam_arm.urdf")
    URDF_DIR = URDF_PATH.parent
    yam_arena.YAM_URDF = str(URDF_PATH)
    from yam_arena import constants as C, demo as dm, ik
    from yam_arena.urdf import forward_kinematics

    HERE = pathlib.Path(__file__).resolve().parent
    sys.path.insert(0, str(HERE))
    import w50_squareup_and_crossing_witness as SQW
    import w50_isolated_q2_and_latched_interpenetration as IQ2

    UPPER_MESH_DIR = pathlib.Path(r"C:/Users/jingy/bt-riser/tasks/shared-bin-coworker/assets/yam_pro_linear_4310/meshes")

    def _read_stl_pts(p: pathlib.Path, dense_tip: bool = False) -> np.ndarray:
        data = p.read_bytes()
        n_tri = struct.unpack_from("<I", data, 80)[0]
        dt = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("attr", "<u2")])
        tris = np.frombuffer(data, dt, count=n_tri, offset=84)["v"].reshape(-1, 3, 3).astype(np.float64)
        if not dense_tip:
            return np.unique(np.round(tris.reshape(-1, 3), 4), axis=0)
        bary = [(1.0 - i / 4 - j / 4, i / 4, j / 4) for i in range(5) for j in range(5 - i)]
        B = np.asarray(bary, float)
        pts = np.einsum("bk,tkd->tbd", B, tris).reshape(-1, 3)
        return np.unique(np.round(pts, 3), axis=0)

    def _load_9links():
        tree = ET.parse(URDF_PATH)
        clouds = {}
        for lnk in tree.getroot().findall("link"):
            nm = lnk.attrib["name"]
            ps = []
            for col in lnk.findall("collision"):
                m = col.find("geometry/mesh")
                if m is not None and (URDF_DIR / m.attrib["filename"]).exists():
                    v = _read_stl_pts(URDF_DIR / m.attrib["filename"], dense_tip=(nm in ("tip_left", "tip_right")))
                    orig = col.find("origin")
                    if orig is not None:
                        xyz = np.asarray([float(x) for x in orig.attrib.get("xyz", "0 0 0").split()], float)
                        rpy = [float(x) for x in orig.attrib.get("rpy", "0 0 0").split()]
                        v = (_rpy_to_R(rpy) @ v.T).T + xyz
                    ps.append(v)
            if ps:
                merged = np.unique(np.round(np.vstack(ps), 3), axis=0)
                if len(merged) > 1200:
                    merged = merged[np.linspace(0, len(merged) - 1, 1200).astype(int)]
                clouds[nm] = merged
                continue
            vis = lnk.find("visual")
            if vis is not None:
                m = vis.find("geometry/mesh")
                if m is not None:
                    mp = UPPER_MESH_DIR / pathlib.Path(m.attrib["filename"]).name
                    if mp.exists():
                        v = _read_stl_pts(mp, dense_tip=False)
                        orig = vis.find("origin")
                        if orig is not None:
                            xyz = np.asarray([float(x) for x in orig.attrib.get("xyz", "0 0 0").split()], float)
                            rpy = [float(x) for x in orig.attrib.get("rpy", "0 0 0").split()]
                            v = (_rpy_to_R(rpy) @ v.T).T + xyz
                        clouds[nm] = v
        return clouds

    CLOUDS = _load_9links()
    LEFT_BASE = np.asarray(C.LEFT_BASE_POS, float)
    RIGHT_BASE = np.asarray(C.RIGHT_BASE_POS, float)

    def arm_world(arm: str, q6: np.ndarray, f_m: float):
        base = LEFT_BASE if arm == "left" else RIGHT_BASE
        jm = {f"joint{i+1}": float(q6[i]) for i in range(6)}
        jm["joint7"] = float(f_m)
        jm["joint8"] = float(f_m)
        fkm = forward_kinematics(str(URDF_PATH), jm)
        return {k: (fkm[k][:3, :3] @ CLOUDS[k].T).T + fkm[k][:3, 3] + base for k in ALL_LINKS}

    def obb_mm(pts: np.ndarray, c: np.ndarray, R: np.ndarray, s: tuple[float, float, float]) -> float:
        rel = (pts - c) @ R
        d = np.abs(rel) - 0.5 * np.asarray(s, float)
        out = np.linalg.norm(np.maximum(d, 0.0), axis=1)
        ins = np.minimum(np.max(d, axis=1), 0.0)
        return float(np.min(out + ins) * 1e3)

    def obb_corners(c: np.ndarray, R: np.ndarray, s: tuple[float, float, float]) -> np.ndarray:
        hx, hy, hz = 0.5 * s[0], 0.5 * s[1], 0.5 * s[2]
        loc = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
        return (R @ loc.T).T + c

    I3 = np.eye(3)
    STATIC_OBBS = {
        ("env", "stand_top"): (np.array([0.5110, 0.0, 0.5875]), I3, (0.7620, 1.2190, 0.0200)),
        ("env", "w50_front_wall"): (np.array([0.4800 - 0.1524 - 0.015875, 0.0, 0.5975 + 0.1143]), I3, (0.03175, 0.5715, 0.2286)),
        ("env", "w50_back_wall"): (np.array([0.4800 + 0.1524 + 0.015875, 0.0, 0.5975 + 0.1143]), I3, (0.03175, 0.5715, 0.2286)),
        ("env", "w50_posY_wall"): (np.array([0.4800, +0.2540 + 0.015875, 0.5975 + 0.1143]), I3, (0.3048, 0.03175, 0.2286)),
        ("env", "w50_negY_wall"): (np.array([0.4800, -0.2540 - 0.015875, 0.5975 + 0.1143]), I3, (0.3048, 0.03175, 0.2286)),
        ("adaptor_bar", "bar_strip"): (np.array([0.4805, -0.0007, RIM + 0.0040]), I3, (0.0254, 0.5310, 0.0080)),
    }

    def pan_obbs(pos, pitch_deg: float = 0.0, yaw_deg: float = 0.0):
        R = _rpy_to_R((0.0, math.radians(pitch_deg), math.radians(yaw_deg)))
        p0 = np.asarray(pos, float)
        specs = [
            ("pan_bot", (0, 0, SHEET / 2), (BODY_W, BODY_L, SHEET)),
            ("pan_w_nx", (-(BODY_W / 2 - SHEET / 2), 0, Q_D / 2), (SHEET, BODY_L, Q_D)),
            ("pan_w_px", (+(BODY_W / 2 - SHEET / 2), 0, Q_D / 2), (SHEET, BODY_L, Q_D)),
            ("pan_w_ny", (0, -(BODY_L / 2 - SHEET / 2), Q_D / 2), (BODY_W, SHEET, Q_D)),
            ("pan_w_py", (0, +(BODY_L / 2 - SHEET / 2), Q_D / 2), (BODY_W, SHEET, Q_D)),
            ("pan_fl_nx", (-(Q_W / 2 - Q_FL / 2), 0, Q_D - SHEET / 2), (Q_FL, Q_L, SHEET)),
            ("pan_fl_px", (+(Q_W / 2 - Q_FL / 2), 0, Q_D - SHEET / 2), (Q_FL, Q_L, SHEET)),
            ("pan_fl_ny", (0, -(Q_L / 2 - Q_FL / 2), Q_D - SHEET / 2), (BODY_W, Q_FL, SHEET)),
            ("pan_fl_py", (0, +(Q_L / 2 - Q_FL / 2), Q_D - SHEET / 2), (BODY_W, Q_FL, SHEET)),
        ]
        return {nm: (p0 + R @ np.asarray(loc, float), R, sz) for nm, loc, sz in specs}

    def audit_isolated_motion(
        motion_name: str,
        wpsL: list,
        wpsR: list,
        active_pan_name: str,
        pan_start_xy: tuple[float, float],
        pan_seat_xy: tuple[float, float],
        pan_seat_pitch_deg: float,
        occupied_pans: dict[str, tuple[tuple[float, float, float], float]],
        seat_wrist_sag_m: float = 0.0,
    ) -> dict:
        plL = dm.ArmPlan(wpsL, C.LEFT_BASE_POS)
        plR = dm.ArmPlan(wpsR, C.RIGHT_BASE_POS)
        n_st = int(round(plL.duration * 20.0))

        occ_obbs = {}
        for onm, (opos, opitch) in occupied_pans.items():
            for sub, obb in pan_obbs(opos, pitch_deg=opitch).items():
                occ_obbs[(onm, sub)] = obb

        px, py, pz, ppitch = pan_start_xy[0], pan_start_xy[1], STAND_TOP + 0.0005, 0.0

        min_housing = (999.0, "none")
        min_unintended_distal = (999.0, "none")
        min_unintended_distal_to_occ = (999.0, "none")
        min_housing_to_occ = (999.0, "none")
        min_intentional_distal = (999.0, "none")
        min_pan_to_occ = (999.0, "none")

        for st in range(1, n_st + 1):
            t = min(st * 0.05, plL.duration)
            plL.action7(t)
            plR.action7(t)
            posL, rotL, fL, labL = plL.target(t)
            posR, rotR, fR, labR = plR.target(t)

            qL_eff, qR_eff = plL.q, plR.q
            sag_active = abs(seat_wrist_sag_m) > 1e-6 and any(k in labL for k in ("lower_slow_final", "release_controlled", "unhook_straight_up", "peel_back"))
            if sag_active:
                qL_eff, _, _ = ik.solve(posL + np.array([0.0, 0.0, seat_wrist_sag_m]), ik.matrix_to_quat_xyzw(rotL), plL.q, LEFT_BASE, iters=60)
                qR_eff, _, _ = ik.solve(posR + np.array([0.0, 0.0, seat_wrist_sag_m]), ik.matrix_to_quat_xyzw(rotR), plR.q, RIGHT_BASE, iters=60)

            lwL = arm_world("left", qL_eff, float(fL))
            lwR = arm_world("right", qR_eff, float(fR))

            # Update quasi-static active pan pose along slide, square-up, carry, and seat!
            if "side_pull_" in labL or "side_pull_" in labR:
                act_lw = lwL if "side_pull_left" in labL else lwR
                act_lab = labL if "side_pull_left" in labL else labR
                if "pull_to_strip" in act_lab:
                    hd = np.vstack([act_lw["tip_left"], act_lw["tip_right"]])
                    ic = hd[hd[:, 2] <= STAND_TOP + Q_D]
                    if len(ic) > 0:
                        px = min(px, float(np.min(ic[:, 0])) + (BODY_HALF_X - SHEET))
            elif "slide_" in labL or "slide_" in labR:
                is_l = "slide_left" in labL
                act_lw = lwL if is_l else lwR
                act_lab = labL if is_l else labR
                if "pull_to_y0" in act_lab:
                    hd = np.vstack([act_lw["tip_left"], act_lw["tip_right"]])
                    ic = hd[hd[:, 2] <= STAND_TOP + Q_D]
                    if len(ic) > 0:
                        if is_l:
                            py = min(py, float(np.min(ic[:, 1])) + (BODY_L / 2.0 - SHEET))
                        else:
                            py = max(py, float(np.max(ic[:, 1])) - (BODY_L / 2.0 - SHEET))
                        px = 0.1972 + (0.2130 - 0.1972) * (abs(py) / 0.4675)
            elif "sqA_" in labL or "sqA_" in labR:
                act_lw = lwL if "sqA_L" in labL else lwR
                act_lab = labL if "sqA_L" in labL else labR
                if "push_plus_x" in act_lab or "hold_wall" in act_lab:
                    hd = np.vstack([act_lw["tip_left"], act_lw["tip_right"]])
                    bf = hd[(hd[:, 2] >= STAND_TOP + 0.004) & (hd[:, 2] <= STAND_TOP + 0.056)]
                    if len(bf) > 0:
                        px = max(px, float(np.max(bf[:, 0])) + BODY_HALF_X)
                        py = 0.0
            elif any(k in labL for k in ("lift_35mm", "hold_1p5s", "raise_carry", "cross_front_rim", "carry_over_seat", "pitch_to_seat", "lower_to_slow", "lower_slow_final")):
                mid = 0.5 * (posL + posR) + (np.array([0.0, 0.0, seat_wrist_sag_m]) if sag_active else 0.0)
                px, py = float(mid[0]), float(mid[1])
                pz = max(0.7690 if px > 0.35 else STAND_TOP + 0.0005, float(mid[2]) - (Q_D - SHEET / 2.0))
                if any(k in labL for k in ("pitch_to_seat", "lower_to_slow", "lower_slow_final")):
                    ppitch = pan_seat_pitch_deg
            elif any(k in labL for k in ("release_controlled", "unhook_straight_up", "peel_back", "rise_above", "retreat_")):
                px, py, pz, ppitch = pan_seat_xy[0], pan_seat_xy[1], 0.7690, pan_seat_pitch_deg

            critical_phase = any(
                k in (labL + "|" + labR)
                for k in ("touch", "pull", "unhook", "push", "hold_wall", "retract_before", "slide_on", "pinch", "lower_", "release_", "peel_back")
            )
            if not critical_phase and (st % 4 != 0) and st != n_st:
                continue

            act_obbs = {(active_pan_name, sub): obb for sub, obb in pan_obbs((px, py, pz), pitch_deg=ppitch).items()}
            all_scene_obbs = {}
            all_scene_obbs.update(STATIC_OBBS)
            all_scene_obbs.update(occ_obbs)
            all_scene_obbs.update(act_obbs)

            # Evaluate every arm link against every named scene/neighbor/active OBB!
            for arm_tag, lw, lab in (("L", lwL, labL), ("R", lwR, labR)):
                for (obj_nm, sub_nm), (oc, oR, osz) in all_scene_obbs.items():
                    for hlnk in HOUSING_LINKS:
                        d = obb_mm(lw[hlnk], oc, oR, osz)
                        pstr = f"{arm_tag}:{hlnk}->{obj_nm}:{sub_nm}@{lab}(step={st})"
                        if d < min_housing[0]:
                            min_housing = (round(d, 2), pstr)
                        if obj_nm in occupied_pans and d < min_housing_to_occ[0]:
                            min_housing_to_occ = (round(d, 2), pstr)
                    for dlnk in DISTAL_LINKS:
                        d = obb_mm(lw[dlnk], oc, oR, osz)
                        pstr = f"{arm_tag}:{dlnk}->{obj_nm}:{sub_nm}@{lab}(step={st})"
                        if is_intentional_distal_surface(arm_tag, lab, obj_nm, sub_nm, active_pan_name):
                            if d < min_intentional_distal[0]:
                                min_intentional_distal = (round(d, 2), pstr)
                        else:
                            if d < min_unintended_distal[0]:
                                min_unintended_distal = (round(d, 2), pstr)
                            if obj_nm in occupied_pans and d < min_unintended_distal_to_occ[0]:
                                min_unintended_distal_to_occ = (round(d, 2), pstr)

            # Evaluate moving pan's 9 OBBs against all occupied neighbor pans' 9 OBBs using exact 15-axis SAT (`Codex 0916`)!
            if occ_obbs:
                import w50_multi_pan_isolated_runner as _MPR_SAT
                for (_, asub), (ac, aR, asz) in act_obbs.items():
                    for (onm, osub), (oc, oR, osz) in occ_obbs.items():
                        d = _MPR_SAT.obb_separation_margin_mm(ac, aR, asz, oc, oR, osz)
                        if d < min_pan_to_occ[0]:
                            min_pan_to_occ = (round(d, 2), f"{active_pan_name}:{asub}->{onm}:{osub}@{labL}(step={st})")

        return {
            "motion": motion_name,
            "duration_s": round(float(plL.duration), 2),
            "steps_20hz": n_st,
            "seat_wrist_sag_mm": round(seat_wrist_sag_m * 1e3, 1),
            "worst_ik_L_R_mm": [round(float(plL.worst_ik_mm), 3), round(float(plR.worst_ik_mm), 3)],
            "housing_min_mm": min_housing,
            "housing_to_occupied_neighbor_min_mm": min_housing_to_occ if min_housing_to_occ[0] < 900 else None,
            "unintended_distal_min_mm": min_unintended_distal,
            "unintended_distal_to_occupied_neighbor_min_mm": min_unintended_distal_to_occ if min_unintended_distal_to_occ[0] < 900 else None,
            "intentional_push_or_grasp_distal_min_mm": min_intentional_distal,
            "moving_pan_to_occupied_neighbor_min_mm": min_pan_to_occ if min_pan_to_occ[0] < 900 else None,
        }

    # Build & audit all isolated downstream motions!
    # 1. Occupied Near_Q2 (next to c7i's settled Near_Q1 at (0.4023, +0.1325, 0.7690, pitch=-3.05 deg)): 0 mm & -12 mm sag!
    s2L, s2R = SQW.slide_int2_x0185_trimmed("right", 0.4675, y_trim=+0.0078)
    sq2L, sq2R = IQ2.squareup_push_A_pitched_left_arm(0.0, X_STRIP)
    q2L, q2R = two_hand_seat_waypoints_isolated((X_STRIP, 0.0), (X_NEAR_2PAN, Y_NEG_SEAT), channel="near", is_occupied_q2=True, tag="near_q2")
    occ_n1_c7i = {"near_q1": ((0.4023, +0.1325, 0.7690), -3.05)}

    res_nq2_unsagged = audit_isolated_motion(
        "occupied_near_q2_unsagged", s2L + sq2L + q2L, s2R + sq2R + q2R, "near_q2",
        (X_STRIP, -0.4675), (X_NEAR_2PAN, Y_NEG_SEAT), -3.05, occ_n1_c7i, seat_wrist_sag_m=0.0
    )
    res_nq2_sag12mm = audit_isolated_motion(
        "occupied_near_q2_sag12mm", s2L + sq2L + q2L, s2R + sq2R + q2R, "near_q2",
        (X_STRIP, -0.4675), (X_NEAR_2PAN, Y_NEG_SEAT), -3.05, occ_n1_c7i, seat_wrist_sag_m=-0.012
    )

    # 2. Isolated Far_Q1 (from front strip (0.2130, +0.4675) -> Far +Y seat (0.5625, +0.1330), +2.83 deg slope):
    sf1L, sf1R = SQW.slide_int2_x0185_trimmed("left", 0.4675, y_trim=-0.0078)
    sqf1L, sqf1R = SQW.squareup_push_A_pitched(0.0, X_STRIP)
    qf1L, qf1R = two_hand_seat_waypoints_isolated((X_STRIP, 0.0), (X_FAR_SEAT, Y_POS_SEAT), channel="far", is_occupied_q2=False, tag="far_q1")

    res_fq1 = audit_isolated_motion(
        "isolated_far_q1", sf1L + sqf1L + qf1L, sf1R + sqf1R + qf1R, "far_q1",
        (X_STRIP, +0.4675), (X_FAR_SEAT, Y_POS_SEAT), +2.83, {}, seat_wrist_sag_m=0.0
    )

    # 3. Occupied Far_Q2 (side-deck (0.4500, -0.4675) -X pull -> 0.185 slide -> Pitched-A -> Far -Y seat next to occupied Far_Q1): 0 & -12 mm sag!
    sd2L, sd2R = side_deck_pull_minus_x("right", 0.4500, 0.4675)
    sf2L, sf2R = SQW.slide_int2_x0185_trimmed("right", 0.4675, y_trim=+0.0078)
    sqf2L, sqf2R = IQ2.squareup_push_A_pitched_left_arm(0.0, X_STRIP)
    qf2L, qf2R = two_hand_seat_waypoints_isolated((X_STRIP, 0.0), (X_FAR_SEAT, Y_NEG_SEAT), channel="far", is_occupied_q2=True, tag="far_q2")
    occ_f1 = {"far_q1": ((X_FAR_SEAT, Y_POS_SEAT, 0.7690), +2.83)}

    res_fq2_unsagged = audit_isolated_motion(
        "occupied_far_q2_from_sidedeck_unsagged", sd2L + sf2L + sqf2L + qf2L, sd2R + sf2R + sqf2R + qf2R, "far_q2",
        (0.4500, -0.4675), (X_FAR_SEAT, Y_NEG_SEAT), +2.83, occ_f1, seat_wrist_sag_m=0.0
    )
    res_fq2_sag12mm = audit_isolated_motion(
        "occupied_far_q2_from_sidedeck_sag12mm", sd2L + sf2L + sqf2L + qf2L, sd2R + sf2R + sqf2R + qf2R, "far_q2",
        (0.4500, -0.4675), (X_FAR_SEAT, Y_NEG_SEAT), +2.83, occ_f1, seat_wrist_sag_m=-0.012
    )

    # 4. Near_Q1 seated next to occupied Far_Q1 + Far_Q2 across the shared 25.4 mm 75020 bar (`X_NEAR_4PAN = 0.3990 m`)!
    sd1L, sd1R = side_deck_pull_minus_x("left", 0.4500, 0.4675)
    qn1_4L, qn1_4R = two_hand_seat_waypoints_isolated((X_STRIP, 0.0), (X_NEAR_4PAN, Y_POS_SEAT), channel="near", is_occupied_q2=False, tag="near_q1_4pan")
    occ_both_far = {
        "far_q1": ((X_FAR_SEAT, Y_POS_SEAT, 0.7690), +2.83),
        "far_q2": ((X_FAR_SEAT, Y_NEG_SEAT, 0.7690), +2.83),
    }
    res_nq1_next_to_far = audit_isolated_motion(
        "near_q1_next_to_occupied_far_channel", sd1L + sf1L + sqf1L + qn1_4L, sd1R + sf1R + sqf1R + qn1_4R, "near_q1",
        (0.4500, +0.4675), (X_NEAR_4PAN, Y_POS_SEAT), -3.05, occ_both_far, seat_wrist_sag_m=0.0
    )

    all_runs = [res_nq2_unsagged, res_nq2_sag12mm, res_fq1, res_fq2_unsagged, res_fq2_sag12mm, res_nq1_next_to_far]
    for r in all_runs:
        assert max(r["worst_ik_L_R_mm"]) < 0.75, f"IK exceeded 0.75 mm on {r['motion']}: {r}"
        assert r["housing_min_mm"][0] > 5.0, f"Housing clearance < 5 mm on {r['motion']}: {r}"
        assert r["unintended_distal_min_mm"][0] > 1.5, f"Unintended distal clearance < 1.5 mm on {r['motion']}: {r}"
        if r["moving_pan_to_occupied_neighbor_min_mm"] is not None:
            assert r["moving_pan_to_occupied_neighbor_min_mm"][0] > 1.5, f"Pan-to-neighbor clash on {r['motion']}: {r}"

    out = {
        "status": "PASS",
        "provenance_note": "All clearances are OFFLINE 9-LINK URDF MODEL CLEARANCES with named distal/housing/pan pairs and explicit intentional-surface separation (W50 native run allowance remains used).",
        "motions": {r["motion"]: r for r in all_runs},
    }
    out_path = HERE / "occupied_q2_and_far_isolated_audit_0827.json"
    out_path.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print("WROTE:", out_path)
    print(json.dumps(out, indent=2))
