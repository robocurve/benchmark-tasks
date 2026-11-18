"""Squeeze a 50 mm spring collar open and slide it past the sleeve shoulder flush against the outer weight plate.

Real product class: a benchtop 50 mm (2-inch) standard loadable barbell sleeve assembly (50 mm chrome loading sleeve
with a 58 mm retention shoulder, two rubber-coated fractional plates of 134 mm and 118 mm OD, inner flange, and a
radial 28 mm knurled stabilizer handle seated across a steel cradle stand, 0.780 kg) paired with a 0.320 kg
dual-handle spring clip collar (three-turn chrome coil body plus two crossed spring-wire stirrup jaws with flared red
vinyl squeeze handles).

Bimanual physics: on a spring clip collar, the two wire handle ends cross over the top of the coil so squeezing the two
handles inward translates the opposing clamp shoes radially outward off the 50 mm sleeve. At rest, the clamp shoes grip
the 50 mm pilot and block against the 58 mm sleeve shoulder lugs; squeezing opens them to 70 mm span so the collar
glides forward flush against the outer plate hub. Without stabilizing the unbolted barbell assembly in its low-friction
cradle, the axial seating push slides the barbell forward out of the stand.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import mechkit  # noqa: E402, F401  (must precede labkit so labkit wraps mechkit's import finder)
import labkit  # noqa: E402
import pitch_demo  # noqa: E402
from labkit import Joint, on_reset, part  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

# -- distinct PBR colours ---------------------------------------------------------------------------------------------
STAND_STEEL = (0.18, 0.20, 0.23)
UHMW_PAD = (0.25, 0.31, 0.38)
RUBBER_PAD = (0.12, 0.13, 0.15)
ALIGN_GOLD = (0.82, 0.64, 0.20)
STEEL_SHAFT = (0.64, 0.67, 0.71)
KNURL_DARK = (0.34, 0.37, 0.41)
STEEL_CHROME = (0.80, 0.83, 0.87)
BRASS_ACCENT = (0.78, 0.61, 0.26)
PLATE_RUBBER = (0.15, 0.16, 0.18)
PLATE_GREEN = (0.18, 0.56, 0.28)
PLATE_RED = (0.78, 0.18, 0.18)
VINYL_RED = (0.82, 0.15, 0.16)
HANDLE_BLK = (0.14, 0.15, 0.17)

# -- 1:1 hardware dimensions (metres, kg, N) --------------------------------------------------------------------------
BAR_X, BAR_Y = 0.295, -0.025  # 50 mm sleeve axis runs along world X at y = -0.025 m
BAR_H = 0.076  # 76 mm sleeve centreline above tabletop (clears 134 mm OD plate by 9 mm)
BAR_Z = TOP + BAR_H

SADDLE_X1, SADDLE_X2 = -0.156, 0.126  # 282 mm wheelbase on main axis; >= 35 mm +X runway when unheld
GRIP_OFFSET_X, GRIP_OFFSET_Y = 0.035, 0.160  # radial 28 mm knurled stabilizer handle at world (0.330, +0.135)

R_SHAFT = 0.0140  # 28.0 mm knurled barbell shaft & radial stabilizer handle
R_SLEEVE = 0.0245  # 50.0 mm standard sleeve envelope (49.0 mm core + 50.0 mm top/bottom anti-rotation key flats)
SHOULDER_SPAN = 0.058  # 58.0 mm retention shoulder lug outer span (+-29.0 mm in Y)
SHOULDER_X = -0.068  # sleeve shoulder centre along barbell X (world x = 0.227 m)
PLATE_FACE_X = -0.010  # outer fractional plate stainless hub face (world x = 0.285 m)

BAR_MASS = 0.780  # 780 g loadable barbell sleeve + two fractional plates (all bodies >= 25 g)
COLLAR_BODY_MASS = 0.250  # 250 g 3-turn chrome coil housing + 2 x 35 g crossed wire jaws = 320 g spring collar
COLLAR_JAW_MASS = 0.035  # 35 g per sprung cross-over jaw stirrup (>= 25 g minimum)

GUIDE_HALF_X = 0.025  # 50 mm concentric guide bushing length along sleeve X
START_REL_X = -0.108  # initial collar position on outer sleeve pilot (world x = 0.187 m, 40 mm outside shoulder)
SEAT_REL_X = PLATE_FACE_X - GUIDE_HALF_X  # -0.035 m (world x = 0.260 m): front face flush with outer plate hub
JAW_INIT_M = 0.0018  # 1.8 mm spawn clearance (~0.3 mm air gap outside sleeve; target=0.0 clamps tight)
JAW_OPEN_MAX_M = 0.0080  # 8.0 mm squeeze stroke per handle (opens clamp shoes from 50 mm to 63 mm span > 58 mm shoulder)
JAW_K = 2200.0  # 2200 N/m per crossed spring leg (~35 N squeeze at full 8.0 mm stroke)
HANDLE_Z_REL = 0.042  # vertical squeeze handle centre 42 mm above sleeve axis


def _build_stand():
    """310 x 240 mm steel benchtop barbell cradle stand with precision polymer guide channels and rubber feet."""
    foot_h, base_t = 0.003, 0.007
    s_top = BAR_H - R_SHAFT
    boxes = [
        ((-0.015, 0.065, foot_h + base_t / 2), (0.310, 0.240, base_t), STAND_STEEL),
        ((-0.015, -0.046, foot_h + base_t + 0.004), (0.294, 0.016, 0.008), STAND_STEEL),
        ((-0.015, 0.176, foot_h + base_t + 0.004), (0.294, 0.016, 0.008), STAND_STEEL),
        ((GRIP_OFFSET_X, GRIP_OFFSET_Y, foot_h + base_t + 0.001), (0.010, 0.070, 0.002), ALIGN_GOLD),
    ]
    for fx in (-0.156, 0.126):
        for fy in (-0.042, 0.172):
            boxes.append(((fx, fy, foot_h / 2), (0.022, 0.022, foot_h), RUBBER_PAD))
    col_h = s_top - (foot_h + base_t) - 0.006
    for sx in (SADDLE_X1, SADDLE_X2):
        boxes.extend([
            ((sx, 0.0, foot_h + base_t + col_h / 2), (0.024, 0.052, col_h), STAND_STEEL),
            ((sx, 0.0, s_top - 0.003), (0.030, 0.032, 0.006), UHMW_PAD),
            ((sx, -0.0197, BAR_H + 0.004), (0.026, 0.010, 0.036), UHMW_PAD),
            ((sx, 0.0197, BAR_H + 0.004), (0.026, 0.010, 0.036), UHMW_PAD),
        ])
    out_h = s_top - (foot_h + base_t)
    boxes.append(((GRIP_OFFSET_X, 0.110, foot_h + base_t + out_h / 2), (0.058, 0.024, out_h), UHMW_PAD))
    return part("stand", (BAR_X, BAR_Y, TOP), fixed=True, friction=0.10, color=STAND_STEEL, boxes=boxes)


def _build_barbell():
    """780 g loadable barbell sleeve with two rubber fractional plates, 58 mm sleeve shoulder, and 28 mm radial handle."""
    lug_y = SHOULDER_SPAN / 2.0 - 0.0035  # +-25.5 mm centre, 7 mm width -> 58.0 mm outer lug span
    cyls = [
        ((-0.168, 0.0, 0.0), R_SHAFT, 0.060, STEEL_SHAFT, "X"),
        ((-0.200, 0.0, 0.0), 0.0155, 0.004, BRASS_ACCENT, "X"),
        ((-0.074, 0.0, 0.0), R_SLEEVE, 0.128, STEEL_CHROME, "X"),
        ((SHOULDER_X, 0.0, 0.0), 0.0244, 0.008, BRASS_ACCENT, "X"),
        ((0.003, 0.0, 0.0), 0.0590, 0.015, PLATE_RUBBER, "X"),
        ((0.003, 0.0, 0.0), 0.0450, 0.0156, PLATE_RED, "X"),
        ((0.000, 0.0, 0.0), 0.0320, 0.020, STEEL_CHROME, "X"),
        ((0.020, 0.0, 0.0), 0.0670, 0.015, PLATE_RUBBER, "X"),
        ((0.020, 0.0, 0.0), 0.0510, 0.0156, PLATE_GREEN, "X"),
        ((0.020, 0.0, 0.0), 0.0320, 0.0162, STEEL_CHROME, "X"),
        ((0.034, 0.0, 0.0), 0.0360, 0.010, STEEL_CHROME, "X"),
        ((0.040, 0.0, 0.0), 0.0260, 0.003, BRASS_ACCENT, "X"),
        ((0.098, 0.0, 0.0), R_SHAFT, 0.116, STEEL_SHAFT, "X"),
        ((GRIP_OFFSET_X, 0.105, 0.0), 0.0125, 0.150, STEEL_SHAFT, "Y"),
        ((GRIP_OFFSET_X, GRIP_OFFSET_Y, 0.0), R_SHAFT, 0.046, KNURL_DARK, "Y"),
        ((GRIP_OFFSET_X, 0.184, 0.0), 0.0165, 0.005, BRASS_ACCENT, "Y"),
    ]
    boxes = [
        ((-0.074, 0.0, 0.0), (0.128, 0.026, 0.0500), STEEL_CHROME),
        ((SADDLE_X1 - 0.014, 0.0, -0.0095), (0.058, 0.0276, 0.009), STEEL_SHAFT),
        ((SADDLE_X2 - 0.014, 0.0, -0.0095), (0.058, 0.0276, 0.009), STEEL_SHAFT),
        ((GRIP_OFFSET_X - 0.010, 0.110, -0.0095), (0.046, 0.022, 0.009), STEEL_SHAFT),
        ((GRIP_OFFSET_X, GRIP_OFFSET_Y, 0.0), (0.024, 0.044, 0.024), KNURL_DARK),
        ((SHOULDER_X, -lug_y, 0.0), (0.008, 0.007, 0.018), BRASS_ACCENT),
        ((SHOULDER_X, lug_y, 0.0), (0.008, 0.007, 0.018), BRASS_ACCENT),
    ]
    return part("barbell", (BAR_X, BAR_Y, BAR_Z + 0.0003), mass=BAR_MASS, friction=0.10,
                color=STEEL_CHROME, boxes=boxes, cylinders=cyls)


def _jaw_joint(name: str, shoe_sign_y: float) -> Joint:
    """Cross-over spring-wire stirrup: pushing the -shoe_sign_y handle inward slides the +shoe_sign_y shoe outward."""
    # Handle spans [-28.7, -6.7 mm] / [+6.7, +28.7 mm] at spawn (q=1.8 mm) and [-22.5, -0.5 mm] / [+0.5, +22.5 mm]
    # at full 8.0 mm stroke -> clear gap across y=0 at max squeeze (zero visual handle interpenetration!).
    hy = -shoe_sign_y * 0.0195
    pad_t = 0.0055
    pad_yc = shoe_sign_y * (0.0235 + pad_t / 2.0)
    z_cross = 0.0285 if shoe_sign_y > 0 else 0.0325
    boxes = (
        ((0.0, hy, HANDLE_Z_REL), (0.034, 0.022, 0.022), VINYL_RED),
        ((-0.0210, -shoe_sign_y * 0.0210, HANDLE_Z_REL), (0.005, 0.022, 0.020), HANDLE_BLK),
        ((0.0200, -shoe_sign_y * 0.0210, HANDLE_Z_REL), (0.005, 0.022, 0.020), HANDLE_BLK),
        ((0.0, 0.0, z_cross), (0.010, 0.056, 0.0035), STEEL_CHROME),
        ((0.0, shoe_sign_y * 0.0275, 0.015), (0.016, 0.005, 0.028), STEEL_CHROME),
        ((0.0, pad_yc, 0.0), (0.018, pad_t, 0.016), HANDLE_BLK),
    )
    lo, hi = (0.0, JAW_OPEN_MAX_M) if shoe_sign_y > 0 else (-JAW_OPEN_MAX_M, 0.0)
    init = JAW_INIT_M if shoe_sign_y > 0 else -JAW_INIT_M
    return Joint(
        name, kind="slide", axis="Y", anchor=(0.0, 0.0, 0.0),
        lower=lo, upper=hi, initial=init, target=0.0,
        stiffness=JAW_K, damping=18.0, friction=0.0, mass=COLLAR_JAW_MASS, boxes=boxes,
    )


def _build_collar():
    """320 g 3-turn chrome 50 mm standard spring clip collar with concentric guide bushings and crossed wire jaws."""
    boxes = [
        ((0.0, 0.0, 0.0279), (2 * GUIDE_HALF_X, 0.026, 0.005), UHMW_PAD),
        ((0.0, 0.0, -0.0279), (2 * GUIDE_HALF_X, 0.026, 0.005), UHMW_PAD),
        ((0.0, 0.0225, 0.0185), (2 * GUIDE_HALF_X, 0.005, 0.006), UHMW_PAD),
        ((0.0, -0.0225, 0.0185), (2 * GUIDE_HALF_X, 0.005, 0.006), UHMW_PAD),
        ((0.0, 0.0225, -0.0185), (2 * GUIDE_HALF_X, 0.005, 0.006), UHMW_PAD),
        ((0.0, -0.0225, -0.0185), (2 * GUIDE_HALF_X, 0.005, 0.006), UHMW_PAD),
        ((0.0, 0.0, -0.0365), (0.034, 0.028, 0.010), STEEL_CHROME),
    ]
    r_coil = 0.0320
    wall = 0.0044
    chord8 = 2.0 * r_coil * math.tan(math.pi / 8.0) + 0.001
    for xc in (-0.0105, 0.0, 0.0105):
        for k in range(8):
            ang = k * math.pi / 4.0
            cy, cz = r_coil * math.cos(ang), r_coil * math.sin(ang)
            sy = wall if abs(math.cos(ang)) > 0.5 else chord8
            sz = chord8 if abs(math.cos(ang)) > 0.5 else wall
            if k % 2 == 0:
                boxes.append(((xc, cy, cz), (0.0060, sy, sz), STEEL_CHROME))
            else:
                boxes.append(((xc, cy, cz), (0.0060, 0.0075, 0.0075), STEEL_SHAFT))
    return part(
        "collar", (BAR_X + START_REL_X, BAR_Y, BAR_Z),
        mass=COLLAR_BODY_MASS, friction=0.08, color=STEEL_CHROME,
        boxes=boxes, joints=(_jaw_joint("jaw_l", 1.0), _jaw_joint("jaw_r", -1.0)),
    )


STAND = _build_stand()
BARBELL = _build_barbell()
COLLAR = _build_collar()
OBJECTS = (STAND, BARBELL, COLLAR)

# -- pure stateless end-state scoring thresholds (every number in task.json scoring appears here as a literal) ---------
CRADLE_TOL = 0.012  # 12 mm: barbell horizontal retention in the bench stand cradle
BAR_Z_TOL = 0.008  # 8 mm: barbell vertical retention in the bench stand cradle
BAR_TILT_DEG = 6.0  # 6 deg: barbell axis tilt limit
COAXIAL_TOL = 0.010  # 10 mm: collar stays coaxial with the 50 mm barbell sleeve
COLLAR_TILT_DEG = 12.0  # 12 deg: collar axis alignment with the barbell sleeve
CLAMP_SHUT_M = 0.004  # 4 mm: released spring jaws clamped onto the 50 mm sleeve (rest ~0.9 mm vs ~12 mm squeezed)
PARTIAL_SLIDE_M = 0.030  # 30 mm: minimum inward slide along sleeve from start for partial progress tier
PLATE_GAP_TOL = 0.008  # 8 mm: collar front face flush against the outer weight plate hub
CLEAR_DIST = 0.120  # 120 mm: both grippers clear of collar, barbell sleeve, and radial stabilizer handle
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.0  # 3 s continuous hold at 1.0 enforced statelessly by the runner (60 steps at 20 Hz)
PASS_SCORE = 1.0 - 1e-9

_IDS: dict = {}


@on_reset
def _reset(_env) -> None:
    _IDS.clear()


def _rot(ps) -> np.ndarray:
    x, y, z, w = (float(v) for v in ps.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def bar_tilt_deg(barbell) -> float:
    rb = _rot(barbell)
    return math.degrees(math.acos(float(np.clip(rb[0, 0], -1.0, 1.0))))


def barbell_seated(barbell) -> bool:
    p = np.asarray(barbell.pos, float)
    return (math.hypot(p[0] - BAR_X, p[1] - BAR_Y) <= CRADLE_TOL
            and abs(p[2] - BAR_Z) <= BAR_Z_TOL
            and bar_tilt_deg(barbell) <= BAR_TILT_DEG)


def collar_rel(collar, barbell) -> tuple[float, float, float]:
    """Return (x_along_sleeve, radial_err_yz, axis_angle_deg) of collar in barbell frame."""
    rb = _rot(barbell)
    rc = _rot(collar)
    rel = rb.T @ (np.asarray(collar.pos, float) - np.asarray(barbell.pos, float))
    rad_yz = math.hypot(float(rel[1]), float(rel[2]))
    ang = math.degrees(math.acos(float(np.clip(abs(float(rb[:, 0] @ rc[:, 0])), -1.0, 1.0))))
    return float(rel[0]), rad_yz, ang


def collar_coaxial(collar, barbell) -> bool:
    _, rad_yz, ang = collar_rel(collar, barbell)
    return rad_yz <= COAXIAL_TOL and ang <= COLLAR_TILT_DEG


def _collar_joints(collar) -> dict:
    j = getattr(collar, "joints", None)
    if j:
        return j
    env = labkit.env()
    return labkit._part_state(env, COLLAR).joints if env is not None else {}


def jaws_clamped(collar) -> bool:
    j = _collar_joints(collar)
    jl = abs(float(j.get("jaw_l", 0.0)))
    jr = abs(float(j.get("jaw_r", 0.0)))
    return jl <= CLAMP_SHUT_M and jr <= CLAMP_SHUT_M


def collar_at_plate(collar, barbell) -> bool:
    x_rel, _, _ = collar_rel(collar, barbell)
    return abs(x_rel - SEAT_REL_X) <= PLATE_GAP_TOL and x_rel > SHOULDER_X + 0.015


def _arm_tcps(env):
    try:
        base = env.unwrapped
        out = []
        for side in ("left", "right"):
            asset = base.scene[f"{side}_robot"]
            key = f"arm_{side}"
            if key not in _IDS:
                _IDS[key] = asset.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = labkit._np(asset.data.joint_pos)[0, _IDS[key]]
            tcp_p, _ = ik.tcp_pose(q, station.BASES[side])
            out.append(tcp_p)
        return (out[0], out[1])
    except Exception:
        return None


def _current_tcps():
    env = labkit.env()
    return _arm_tcps(env) if env is not None else None


def hands_clear(collar, barbell, tcps=None) -> bool:
    pts = _current_tcps() if tcps is None else tcps
    if pts is None or len(pts) < 2:
        return False
    pc = np.asarray(collar.pos, float)
    pb = np.asarray(barbell.pos, float)
    rb = _rot(barbell)
    p_handle = pb + rb @ np.array([GRIP_OFFSET_X, GRIP_OFFSET_Y, 0.0])
    p_handle_tip = pb + rb @ np.array([GRIP_OFFSET_X, 0.184, 0.0])
    anchors = (pc, pb, p_handle, p_handle_tip)
    return all(float(np.linalg.norm(np.asarray(x, float) - a)) > CLEAR_DIST for x in pts for a in anchors)


def components(objects: dict, tcps=None) -> dict:
    """Pure observable physical end-state predicates (zero step-accumulated score state)."""
    collar, barbell = objects.get("collar"), objects.get("barbell")
    if collar is None or barbell is None:
        return {
            "barbell_seated": False, "collar_coaxial": False, "slid_partial": False,
            "at_plate": False, "jaws_clamped": False, "hands_clear": False,
        }
    x_rel, _, _ = collar_rel(collar, barbell)
    return {
        "barbell_seated": barbell_seated(barbell),
        "collar_coaxial": collar_coaxial(collar, barbell),
        "slid_partial": (x_rel - START_REL_X) >= PARTIAL_SLIDE_M,
        "at_plate": collar_at_plate(collar, barbell),
        "jaws_clamped": jaws_clamped(collar),
        "hands_clear": hands_clear(collar, barbell, tcps),
    }


def score(objects: dict, tcps=None) -> float:
    """Judged from pure physical end-state geometry; Possible scores 0.0, 0.25, 0.50, 0.75, 1.0."""
    c = components(objects, tcps)
    if not (c["barbell_seated"] and c["collar_coaxial"] and c["slid_partial"]):
        return 0.0
    if not c["at_plate"]:
        return 0.25
    if not c["jaws_clamped"]:
        return 0.50
    if not c["hands_clear"]:
        return 0.75
    return 1.0


def success(objects: dict) -> bool:
    return score(objects) >= PASS_SCORE


def distance(objects: dict) -> float:
    collar, barbell = objects.get("collar"), objects.get("barbell")
    if collar is None or barbell is None:
        return 1.0
    x_rel, rad_yz, _ = collar_rel(collar, barbell)
    return float(abs(x_rel - SEAT_REL_X) + rad_yz)


# -- scripted demo (scene constants only) -----------------------------------------------------------------------------
SLIDE_OVERDRIVE_X = 0.025  # 25 mm compliant +X drive seats collar flush against outer plate (shoves unheld bar!)


def demo():
    """Open-loop bimanual plan using scene constants only (left clamps radial handle along X; right squeezes & slides)."""
    W = pitch_demo.Waypoint
    o, c = FINGER_OPEN, FINGER_CLOSED
    sq = -0.015
    rl = pitch_demo.rotation(-math.pi / 2.0)
    r0 = pitch_demo.rotation(0.0)

    lx, ly, lz = BAR_X + GRIP_OFFSET_X, BAR_Y + GRIP_OFFSET_Y, BAR_Z + 0.002
    lx_brace = lx - 0.008
    lx_rel = lx
    ry = BAR_Y
    rx0 = BAR_X + START_REL_X
    rx_mid = BAR_X + SHOULDER_X + 0.012
    rx_seat = BAR_X + SEAT_REL_X + SLIDE_OVERDRIVE_X
    rz = BAR_Z + HANDLE_Z_REL + 0.0075
    rz_slide = rz - 0.005
    z_hov = TOP + 0.182

    seq = [
        (0.7, (lx, ly, z_hov), o, rl, (rx0, ry, z_hov), o, r0, "hover over radial handle and collar"),
        (0.7, (lx, ly, lz), o, rl, (rx0, ry, z_hov), o, r0, "descend left over radial stabilizer handle"),
        (0.8, (lx, ly, lz), c, rl, (rx0, ry, z_hov), o, r0, "clamp radial handle along X at zero offset"),
        (0.7, (lx, ly, lz), c, rl, (rx0, ry, rz), o, r0, "descend right over spring collar handles"),
        (1.0, (lx, ly, lz), c, rl, (rx0, ry, rz), sq, r0, "squeeze collar handles open"),
        (1.1, (lx_brace, ly, lz), c, rl, (rx_mid, ry, rz_slide), sq, r0, "glide open collar over sleeve shoulder"),
        (1.4, (lx_brace, ly, lz), c, rl, (rx_seat, ry, rz_slide), sq, r0, "press collar flush against outer plate"),
        (0.8, (lx_rel, ly, lz), c, rl, (rx_seat, ry, rz_slide), o, r0, "release handles in place so coil clamps sleeve"),
        (0.8, (lx_rel, ly, lz), o, rl, (rx_seat, ry, z_hov), o, r0, "open left grip in place and lift right straight up"),
        (0.8, (lx_rel, ly, z_hov), o, rl, (rx_seat, -0.22, z_hov), o, r0, "lift left straight up and swing right clear"),
        (0.9, (0.18, 0.30, TOP + 0.17), o, r0, (0.18, -0.30, TOP + 0.17), o, r0, "park both arms clear"),
        (4.2, (0.18, 0.30, TOP + 0.17), o, r0, (0.18, -0.30, TOP + 0.17), o, r0, "hold 3 s post-release"),
    ]
    left = [W(d, lp, lf, lab, rot=lr) for d, lp, lf, lr, _, _, _, lab in seq]
    right = [W(d, rp, rf, lab, rot=rr) for d, _, _, _, rp, rf, rr, lab in seq]
    return {"left": left, "right": right}
