"""Thread a stock 50 mm spring clip collar onto a barbell sleeve and seat it flush against the outer plate.

Real product class: a 50 mm (2-inch) loadable barbell sleeve assembly (a plain round 49 mm chrome sleeve,
26 mm rear shaft, a 134 mm and a 118 mm rubber-coated fractional plate, 0.760 kg) resting in a benchtop
cradle stand, paired with a stock 0.320 kg dual-handle spring clip collar (three-turn chrome coil body plus
two crossed spring-wire stirrup jaws with vinyl-dipped shoes and flared red vinyl squeeze handles).

Bimanual physics: a spring clip collar is a closed ring, so it goes on only over the bar's free end and only
with both crossed handles squeezed inward, which springs the two clamp shoes open from 41 mm to 66 mm span.
At rest the vinyl-dipped shoes are preloaded 4 mm inside the sleeve radius, so an unsqueezed collar cannot be
slid onto the sleeve at all: the shoe faces butt against the sleeve's end step. The collar is carried from its
bench tray to beyond the end cap, threaded along the end shaft and the full sleeve, and seated against the
outer plate. After release the wire jaws clamp the sleeve; the retention check then drags the collar axially
off the plate with a light friction-limited pinch (a spring-scale tug in a real trial) and the collar must
stay within 3 mm of the plate. Without holding the bar, the axial drag of the shoes walks the unheld bar
along its low-friction cradle.
"""

from __future__ import annotations

import json
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
from labkit import Joint, on_reset, on_step, part  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

# -- distinct PBR colours ---------------------------------------------------------------------------------------------
STAND_STEEL = (0.18, 0.20, 0.23)
UHMW_PAD = (0.25, 0.31, 0.38)
RUBBER_PAD = (0.12, 0.13, 0.15)
STEEL_SHAFT = (0.64, 0.67, 0.71)
KNURL_DARK = (0.34, 0.37, 0.41)
STEEL_CHROME = (0.80, 0.83, 0.87)
BRASS_ACCENT = (0.78, 0.61, 0.26)
PLATE_RUBBER = (0.15, 0.16, 0.18)
PLATE_GREEN = (0.18, 0.56, 0.28)
PLATE_RED = (0.78, 0.18, 0.18)
VINYL_RED = (0.82, 0.15, 0.16)
HANDLE_BLK = (0.14, 0.15, 0.17)
TRAY_WOOD = (0.44, 0.33, 0.22)

# -- 1:1 stock hardware dimensions (metres, kg, N) --------------------------------------------------------------------
BAR_X, BAR_Y = 0.250, -0.035  # bar frame origin (the task sleeve's plate-side end region); the axis runs along world +Y
BAR_YAW = math.pi / 2.0  # the bar, stand, tray and collar are built along their local X and turned onto world +Y:
# the right arm then threads the collar straight away from its own base and the left arm holds the handle on its side
BAR_H = 0.080  # 80 mm sleeve centreline above tabletop: the 134 mm OD plates clear the 10 mm stand base by 3 mm
BAR_Z = TOP + BAR_H

# The cradle holds the dumbbell by its centre handle, like a dumbbell rack, so nothing stands in the collar's path
# from the bar's free end (world x 0.093) to the outer plate face (0.285): a closed collar must thread over all of it.
SADDLE_X1, SADDLE_X2 = 0.070, 0.150  # bar frame, on the 136 mm handle (0.0415 .. 0.1775): the handle's midpoint lies between them
R_SHAFT = 0.0140  # 28.0 mm end shaft and knurled centre handle
R_SLEEVE = 0.0245  # 49.0 mm plain round chrome loading sleeve (no key flats, no shoulder ring)
PLATE_FACE_X = -0.010  # outer fractional plate -X face (plate, colour ring and hub are coplanar), world x = 0.285 m
COLLAR_HALF_X = 0.0165  # collar half width along the sleeve: coil, handles and end caps all lie within +-16.5 mm
SEAT_REL_X = PLATE_FACE_X - COLLAR_HALF_X  # -0.0265 m (world x = 0.2685 m): front face flush with the plate face
THREAD_REL_X = -0.140  # -0.140 m (world x = 0.155 m): collar centre at the free -X end of the sleeve
BAR_END_REL_X = -0.202  # -0.202 m (world x = 0.093 m): the bar's free end face (brass end cap)

TRAY_X, TRAY_Y = 0.400, -0.300  # felted timber collar rest: 150 mm beside the axis and beyond the bar's free end
TRAY_FLOOR_T = 0.005  # felt-lined timber floor
TRAY_WALL_T = 0.008
TRAY_WALL_H = 0.038  # slot walls reach the resting coil's equator, so the squeeze can neither skid nor tip the collar
TRAY_POCKET_HALF_X = 0.0185  # slot half length along the coil axis: 2 mm clearance each side of the 33 mm collar
TRAY_POCKET_HALF_Y = 0.0387  # slot half width across the coil: 2.5 mm clearance each side of the 72 mm coil
COIL_R = 0.0340  # 68 mm coil centreline radius (8-gon of 4.4 mm wall): bore 59.5-63.6 mm over the 49 mm sleeve
COIL_WALL = 0.0044
COLLAR_REST_Z = TRAY_FLOOR_T + COIL_R + COIL_WALL / 2.0  # 41.2 mm above the tabletop: the coil rests on the tray floor

BAR_MASS = 1.200  # 1.20 kg loadable dumbbell handle with two fractional plates on each sleeve (all bodies >= 25 g)
HANDLE_X0, HANDLE_X1 = 0.0415, 0.1775  # knurled centre handle between the two plate stacks (bar frame)
HOLD_OFF_AXIS = 0.060  # left gripper clamps the far 134 mm plate across its faces, 60 mm off the axis on the +x rim
HOLD_Z_REL = 0.030  # pad tips 17 mm above the axis: 13 mm of plate rim between the pads
COLLAR_BODY_MASS = 0.250  # 250 g 3-turn chrome coil housing + 2 x 35 g crossed wire jaws = 320 g collar
COLLAR_JAW_MASS = 0.035  # 35 g per sprung cross-over jaw stirrup (>= 25 g minimum)

JAW_REST_IN = 0.0205  # clamp-shoe inner face at rest: 4.0 mm inside the 24.5 mm sleeve radius (the preload)
JAW_OPEN_MAX_M = 0.0125  # 12.5 mm handle stroke: shoe face to 32.0 mm, a 64 mm span, clear of the 49 mm sleeve
JAW_K = 2200.0  # 2200 N/m per crossed leg: 8.8 N per shoe at the 4 mm preload, 27.5 N at the full 12.5 mm stroke
JAW_FRICTION = 0.4  # N: light grease-free wire joint, holds the shoe where it is left
HANDLE_HALF_Y = 0.0055  # 11 mm wide vinyl handle: the pair stays 27 mm apart at full squeeze
HANDLE_Z_REL = 0.053  # vertical squeeze handle centre 53 mm above the sleeve axis: a 34 mm tall vinyl loop over the coil
COLLAR_FRICTION = 0.60  # vinyl-dipped jaw shoes and handles on the chrome sleeve


def _build_stand():
    """Benchtop cradle: a 310 x 240 mm steel base with two polymer-cradled stanchions straddling the plates."""
    foot_h, base_t = 0.003, 0.007
    s_top = BAR_H - R_SHAFT
    cx = (SADDLE_X1 + SADDLE_X2) / 2.0
    span = SADDLE_X2 - SADDLE_X1 + 0.040
    boxes = [
        ((cx, 0.0, foot_h + base_t / 2), (span, 0.150, base_t), STAND_STEEL),
    ]
    for sx in (SADDLE_X1, SADDLE_X2):
        boxes.extend([
            ((sx, -0.052, foot_h / 2), (0.022, 0.022, foot_h), RUBBER_PAD),
            ((sx, 0.052, foot_h / 2), (0.022, 0.022, foot_h), RUBBER_PAD),
        ])
        col_h = s_top - (foot_h + base_t) - 0.014
        boxes.extend([
            ((sx, 0.0, foot_h + base_t + col_h / 2), (0.024, 0.044, col_h), STAND_STEEL),
            # channel floor: the 28 mm shaft rests on this pad (top face at BAR_H - R_SHAFT)
            ((sx, 0.0, BAR_H - R_SHAFT - 0.0045), (0.026, 0.040, 0.007), UHMW_PAD),
            # two side walls cradling the shaft (inner faces at +-14.4 mm), stopping lateral roll
            ((sx, -0.0172, BAR_H + 0.002), (0.026, 0.0056, 0.030), UHMW_PAD),
            ((sx, 0.0172, BAR_H + 0.002), (0.026, 0.0056, 0.030), UHMW_PAD),
        ])
    return part("stand", (BAR_X, BAR_Y, TOP), fixed=True, yaw=BAR_YAW, friction=0.10, color=STAND_STEEL, boxes=boxes)


def _build_tray():
    """Timber collar holster: a felt-lined slot that holds the collar upright on its coil, off the sleeve, for pickup."""
    hl, hw, h, f = TRAY_POCKET_HALF_X, TRAY_POCKET_HALF_Y, TRAY_WALL_H, TRAY_FLOOR_T
    t = TRAY_WALL_T
    boxes = [
        ((0.0, 0.0, f / 2), (2 * hl + 2 * t, 2 * hw + 2 * t, f), TRAY_WOOD),
        ((hl + t / 2, 0.0, f + h / 2), (t, 2 * hw + 2 * t, h), TRAY_WOOD),
        ((-hl - t / 2, 0.0, f + h / 2), (t, 2 * hw + 2 * t, h), TRAY_WOOD),
        ((0.0, hw + t / 2, f + h / 2), (2 * hl, t, h), TRAY_WOOD),
        ((0.0, -hw - t / 2, f + h / 2), (2 * hl, t, h), TRAY_WOOD),
    ]
    return part("tray", (TRAY_X, TRAY_Y, TOP), fixed=True, yaw=BAR_YAW, friction=0.60, color=TRAY_WOOD, boxes=boxes)


def _build_barbell():
    """1.20 kg loadable dumbbell handle: two plain round 49 mm sleeves, each loaded with two rubber fractional plates,
    on a 28 mm knurled centre handle. The near (-X) sleeve is the task's; the far one mirrors it about the handle."""
    near = [
        ((-0.168, 0.0, 0.0), R_SHAFT, 0.060, STEEL_SHAFT, "X"),
        ((-0.200, 0.0, 0.0), 0.0155, 0.004, BRASS_ACCENT, "X"),
        ((-0.072, 0.0, 0.0), R_SLEEVE, 0.124, STEEL_CHROME, "X"),
        ((-0.135, 0.0, 0.0), R_SLEEVE - 0.00075, 0.002, STEEL_CHROME, "X"),  # the stock sleeve's 1.5 mm end chamfer,
        ((-0.137, 0.0, 0.0), R_SLEEVE - 0.00150, 0.002, STEEL_CHROME, "X"),  # as two 2 mm steps
        ((PLATE_FACE_X + 0.0075, 0.0, 0.0), 0.0590, 0.015, PLATE_RUBBER, "X"),
        ((PLATE_FACE_X + 0.0075, 0.0, 0.0), 0.0450, 0.0150, PLATE_RED, "X"),
        ((PLATE_FACE_X + 0.0100, 0.0, 0.0), 0.0320, 0.020, STEEL_CHROME, "X"),
        ((0.020, 0.0, 0.0), 0.0670, 0.015, PLATE_RUBBER, "X"),
        ((0.020, 0.0, 0.0), 0.0510, 0.0156, PLATE_GREEN, "X"),
        ((0.020, 0.0, 0.0), 0.0320, 0.0162, STEEL_CHROME, "X"),
        ((0.034, 0.0, 0.0), 0.0360, 0.010, STEEL_CHROME, "X"),
        ((0.040, 0.0, 0.0), 0.0260, 0.003, BRASS_ACCENT, "X"),
    ]
    mid = (HANDLE_X0 + HANDLE_X1) / 2.0
    far = [((2.0 * mid - c[0], 0.0, 0.0), r, h, k, ax) for (c, r, h, k, ax) in near]
    handle = [((mid, 0.0, 0.0), R_SHAFT, HANDLE_X1 - HANDLE_X0, KNURL_DARK, "X")]
    return part("barbell", (BAR_X, BAR_Y, BAR_Z), mass=BAR_MASS, friction=0.10, yaw=BAR_YAW,
                color=STEEL_CHROME, cylinders=near + handle + far)


def _jaw_joint(name: str, shoe_sign_y: float) -> Joint:
    """Cross-over spring-wire stirrup: pushing the -shoe_sign_y handle inward slides the +shoe_sign_y shoe outward."""
    hy = -shoe_sign_y * 0.0195  # handle centre 19.5 mm out from the sleeve axis; the pad presses its outer face
    pad_t = 0.0090  # 9 mm: thicker than the ~3.5 mm contact penetration, so a 20 N pad load cannot pop
                    # the pad through the 11 mm handle face. NOTE: at 7 mm the outer shoe corner clears the
                    # 29.5 mm plate radius in Isaac, but jitter3 and the retention margin regress (measured)
    pad_yc = shoe_sign_y * (JAW_REST_IN + pad_t / 2.0)
    z_cross = 0.0285 if shoe_sign_y > 0 else 0.0325
    # The crossed wire rides over the coil at a different x per jaw, so the two handles pass each other.
    hx = 0.0015 if shoe_sign_y > 0 else -0.0015
    boxes = (
        ((hx, hy, HANDLE_Z_REL), (0.030, 2 * HANDLE_HALF_Y, 0.034), VINYL_RED),
        ((0.0, 0.0, z_cross), (0.010, 0.056, 0.0035), STEEL_CHROME),
        ((0.0, shoe_sign_y * 0.0275, 0.015), (0.016, 0.005, 0.028), STEEL_CHROME),
        ((0.0, pad_yc, 0.0), (0.018, pad_t, 0.016), HANDLE_BLK),
    )
    lo, hi = (0.0, JAW_OPEN_MAX_M) if shoe_sign_y > 0 else (-JAW_OPEN_MAX_M, 0.0)
    return Joint(
        name, kind="slide", axis="Y", anchor=(0.0, 0.0, 0.0),
        lower=lo, upper=hi, initial=0.0, target=0.0,
        stiffness=JAW_K, damping=18.0, friction=JAW_FRICTION, mass=COLLAR_JAW_MASS, boxes=boxes,
    )


def _build_collar():
    """320 g stock 3-turn chrome 50 mm spring clip collar; the wire jaws are its only sleeve interface."""
    boxes = []
    r_coil, wall = COIL_R, COIL_WALL
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
                boxes.append(((xc, cy, cz), (0.0060, 0.0060, 0.0060), STEEL_SHAFT))
    return part(
        "collar", (TRAY_X, TRAY_Y, TOP + COLLAR_REST_Z + 0.001), yaw=BAR_YAW,
        mass=COLLAR_BODY_MASS, friction=COLLAR_FRICTION, color=STEEL_CHROME,
        boxes=boxes, joints=(_jaw_joint("jaw_l", 1.0), _jaw_joint("jaw_r", -1.0)),
    )


STAND = _build_stand()
TRAY = _build_tray()
BARBELL = _build_barbell()
COLLAR = _build_collar()
OBJECTS = (STAND, TRAY, BARBELL, COLLAR)

# -- pure stateless end-state scoring thresholds (every number in task.json scoring appears here as a literal) ---------
CRADLE_TOL = 0.012  # 12 mm: barbell horizontal retention in the bench stand cradle
BAR_Z_TOL = 0.008  # 8 mm: barbell vertical retention in the bench stand cradle
BAR_TILT_DEG = 6.0  # 6 deg: barbell axis tilt limit (digital protractor)
COAXIAL_TOL = 0.010  # 10 mm: collar stays coaxial with the 49 mm barbell sleeve
COLLAR_TILT_DEG = 12.0  # 12 deg: collar axis alignment with the barbell sleeve
PARTIAL_SLIDE_M = 0.030  # 30 mm: minimum inward slide along the sleeve from the free end for partial progress
NEAR_TOL = 0.008  # 8 mm: collar close to the outer plate face
FLUSH_TOL = 0.003  # 3 mm feeler-gauge gap, measured the way a person at the bench sees it: from the collar's
                   # FOREMOST plate-side surface (its two black plastic squeeze handles, which stand 3.0 mm proud
                   # of the coil) to the outer plate face. Measured, not assumed: the collision hull spans
                   # +-16.5 mm along the sleeve and the handles are its +x extreme; the coil's own face is at
                   # +13.5 mm, so a coil-to-plate reading 3 mm further in would be a different number, not a
                   # different gate. The same measure after the retention pull.
CLEAR_DIST = 0.120  # 120 mm: both grippers clear of the collar and the barbell sleeve
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.0  # 3 s continuous hold at 1.0 enforced statelessly by the runner (60 steps at 20 Hz)
PASS_SCORE = 1.0 - 1e-9

_IDS: dict = {}


STATE_EVERY = 40  # print a geometry line every 40 control steps (2 s) so a probe run is diagnosable
_STEP = {"n": 0}


@on_reset
def _reset(_env) -> None:
    _IDS.clear()
    _STEP["n"] = 0


@on_step
def _state_line(env) -> None:
    """Observation only: prints geometry every STATE_EVERY steps. Writes nothing the scorer reads."""
    _STEP["n"] += 1
    if _STEP["n"] % STATE_EVERY:
        return
    try:
        collar = labkit._part_state(env, labkit._SPECS["collar"])
        barbell = labkit._part_state(env, labkit._SPECS["barbell"])
    except Exception:
        return
    x_rel, rad, ang = collar_rel(collar, barbell)
    row = {
        "step": _STEP["n"], "x_rel_mm": round(x_rel * 1000, 1), "gap_mm": round(collar_gap(collar, barbell) * 1000, 1),
        "radial_mm": round(rad * 1000, 1), "tilt_deg": round(ang, 1), "tilt_axis": _tilt_axis(collar, barbell),
        "bar_drift_mm": round(float(np.hypot(barbell.pos[0] - BAR_X, barbell.pos[1] - BAR_Y)) * 1000, 1),
        "bar_dz_mm": round((float(barbell.pos[2]) - BAR_Z) * 1000, 1),
    }
    # jaw stroke and the right hand, so a failed probe says WHY: jaws still open means no clamp, a hand still
    # inside the collar envelope means the pads are in the way, and hand x vs collar x says who moved
    try:
        joints = getattr(collar, "joints", {}) or {}
        row["jaw_l_mm"] = round(float(joints.get("jaw_l", float("nan"))) * 1000, 1)
        row["jaw_r_mm"] = round(float(joints.get("jaw_r", float("nan"))) * 1000, 1)
    except Exception:
        pass
    try:
        tcps = _arm_tcps(env)
        if tcps is not None:
            rb = _rot(barbell)
            rel = rb.T @ (np.asarray(tcps[1], float) - np.asarray(barbell.pos, float))
            row["hand_x_mm"] = round(float(rel[0]) * 1000, 1)
            row["hand_dz_mm"] = round(float(rel[2]) * 1000, 1)
            row["hand_collar_dx_mm"] = round(float(rel[0] - x_rel) * 1000, 1)
    except Exception:
        pass
    print("STATE " + json.dumps(row), flush=True)


def _rot(ps) -> np.ndarray:
    x, y, z, w = (float(v) for v in ps.quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def bar_tilt_deg(barbell) -> float:
    """Digital protractor on the sleeve: the bar axis's angle from horizontal."""
    rb = _rot(barbell)
    return math.degrees(math.asin(float(np.clip(abs(rb[2, 0]), 0.0, 1.0))))


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


def _tilt_axis(collar, barbell) -> list[float]:
    """The collar's tilt as a unit axis in the sleeve frame plus its sign, so a failed run says WHICH way the ring is
    cocked and a counter-roll can be sized from data instead of guessed. Built from the same two poses the scorer
    reads; observation only."""
    rb = _rot(barbell)
    rc = _rot(collar)
    # rotation taking the collar's axis onto the sleeve's, expressed in the sleeve frame
    rel = rb.T @ rc
    ang = math.acos(float(np.clip((float(np.trace(rel)) - 1.0) / 2.0, -1.0, 1.0)))
    if ang < 1e-9:
        return [0.0, 0.0, 0.0, 0.0]
    ax = np.array([rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]]) / (2.0 * math.sin(ang))
    return [round(float(v), 2) for v in ax] + [round(math.degrees(math.atan2(ax[1], ax[0])), 1)]


def collar_coaxial(collar, barbell) -> bool:
    _, rad_yz, ang = collar_rel(collar, barbell)
    return rad_yz <= COAXIAL_TOL and ang <= COLLAR_TILT_DEG


def collar_gap(collar, barbell) -> float:
    """Caliper reading at the bench: the collar's FOREMOST plate-side surface to the outer plate face along the
    sleeve. On this stock collar the foremost surface is the two plastic squeeze handles, 3.0 mm proud of the coil
    (+16.5 mm vs +13.5 mm in the collar frame), so the gap closes to 0 when the handles touch the plate."""
    x_rel, _, _ = collar_rel(collar, barbell)
    return abs(x_rel - SEAT_REL_X)


def collar_on_sleeve(collar, barbell) -> bool:
    """The collar is threaded on the loading sleeve (past its free -X end, short of the plate face)."""
    x_rel, rad_yz, _ = collar_rel(collar, barbell)
    return (THREAD_REL_X - 0.010) <= x_rel <= (PLATE_FACE_X + 0.005) and rad_yz <= COAXIAL_TOL


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
    anchors = (pc, pb, pb + rb @ np.array([PLATE_FACE_X + 0.040, 0.0, 0.0]),
               pb + rb @ np.array([-0.138, 0.0, 0.0]))
    return all(float(np.linalg.norm(np.asarray(x, float) - a)) > CLEAR_DIST for x in pts for a in anchors)


def components(objects: dict, tcps=None) -> dict:
    """Pure observable physical end-state predicates (zero step-accumulated score state)."""
    collar, barbell = objects.get("collar"), objects.get("barbell")
    if collar is None or barbell is None:
        return {
            "barbell_seated": False, "collar_coaxial": False, "collar_on_sleeve": False,
            "slid_partial": False, "near_plate": False, "flush": False, "hands_clear": False,
        }
    x_rel, _, _ = collar_rel(collar, barbell)
    gap = collar_gap(collar, barbell)
    return {
        "barbell_seated": barbell_seated(barbell),
        "collar_coaxial": collar_coaxial(collar, barbell),
        "collar_on_sleeve": collar_on_sleeve(collar, barbell),
        "slid_partial": (x_rel - THREAD_REL_X) >= PARTIAL_SLIDE_M,
        "near_plate": gap <= NEAR_TOL,
        "flush": gap <= FLUSH_TOL,
        "hands_clear": hands_clear(collar, barbell, tcps),
    }


def score(objects: dict, tcps=None) -> float:
    """Judged from pure measured end-state geometry; Possible scores 0.0, 0.25, 0.50, 1.0."""
    c = components(objects, tcps)
    if not (c["barbell_seated"] and c["collar_on_sleeve"] and c["collar_coaxial"] and c["slid_partial"]):
        return 0.0
    if not c["near_plate"]:
        return 0.25
    if not (c["flush"] and c["hands_clear"]):
        return 0.50
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
SEAT_PRESS_X = 0.015  # 15 mm compliant drive along the axis, held 1.0 s: the pair is what carries the collar
                      # through the released-jaw slip of the retention tug. Measured (MuJoCo 4.0.1 rigid-body check, this file,
                      # five start jitters + friction x0.7 / x1.3): at 14 mm / 0.8 s jitter1 released at 1.66 mm and
                      # the tug dragged it back out to 3.02 mm -> flush failed by 0.02 mm; 15 mm / 1.0 s passes
                      # every case with the worst end gap 2.42 mm. Deeper presses do not help and cost margin:
                      # 16-18 mm slip the same way on other seeds (jitter2 3.31 mm, jitter4 4.15 mm) and at 18 mm
                      # friction x0.7 pushes the bar 12.5 mm along its low-friction cradle, outside the 12 mm window.
SEAT_DWELL_S = 1.0  # hold the squeeze at the press target before releasing: the compliant press takes up its
                    # drag over ~0.6 s, the released jaws then settle onto the sleeve, and releasing sooner
                    # leaves the collar short of the plate
SQUARE_UP_DEG = 4.0  # wrist counter-roll about its own x axis while seating: the collar tilts as it lands and a
                       # cocked ring bears one handle corner on the plate. Measured: see the r_sq block in demo().
RELEASE_S = 1.0  # open the fingers over a full second WHILE the arm still holds the collar pressed against the
                # plate: a person lets go with the collar home, not while pulling away. Measured on this plan, the
                # whole release tail is a knife edge: 1.0 s held-in-place passes demo + all five start jitters +
                # both friction scales (worst end gap 2.65 mm), while 0.4 s of opening followed by a separate
                # settle and a square-up nudge scores 0.5 on the demo and 3.3-3.4 mm on two jitters. The extra
                # motions drag the released ring; doing less, for longer, is what seats it.
SEAT_LAND_X = 0.006  # land the collar square at a shallow target first: driving straight in cocks it ~5 deg and
                     # the cocked rim meets the plate edge instead of seating flat
TUG_M = 0.020  # 20 mm axial withdrawal off the plate: the retention check
TUG_FINGER = -0.035  # light pinch on the 72 mm coil: pads 70 mm apart, a friction-limited spring-scale tug
SQUEEZE_FINGER = -0.015  # pads 24 mm apart: drives each 11 mm handle ~6.5 mm inward, the shoes 3 mm clear of the sleeve
TOUCH_FINGER = -0.027  # pads 54 mm apart: 2 mm short of the handles' outer faces, where the slow squeeze starts
ENTRY_CLEAR_X = 0.010  # collar front face 10 mm beyond the bar's end cap before the thread starts
CARRY_Z_REL = 0.150  # TCP carry height: the hanging collar is carried round the bar's free end, never over the bar
CARRY_SAG = 0.007  # the collar hangs 7 mm low on its squeezed handles (measured in the MuJoCo check)
ENTRY_DX = 0.0045  # measured entry correction: the carried collar reaches the end cap 5.5 mm off-axis in world x
ENTRY_DZ = -0.0020  # and 2.4 mm high in world z, so the open shoes strike the end step instead of passing it


def _on_axis(rel_x: float) -> tuple[float, float]:
    """World (x, y) of a point on the sleeve axis at bar-frame x = rel_x (the axis runs along world +Y)."""
    return (BAR_X, BAR_Y + rel_x)


def demo():
    """Open-loop bimanual plan using scene constants only."""
    W = pitch_demo.Waypoint
    o, c = FINGER_OPEN, FINGER_CLOSED
    sq = SQUEEZE_FINGER
    r_right = pitch_demo.rotation(math.pi / 2.0)  # fingers close along world x: across the collar's handles / coil
    # Square-up measured, not guessed: as the collar lands it picks up a repeatable tilt and ends with one corner of
    # its handle face bearing on the plate instead of the face settling flat. Turning the wrist +4 deg about its own
    # x axis while threading, landing, driving and holding keeps the ring face square. Measured in the offline
    # pre-check, same plan, one variable: peak tilt 2.72 -> 2.34 deg and end gap 2.21 -> 1.33 mm at +4 deg, while
    # -4 deg regresses it hard (peak 6.37 deg, gap 5.07 mm, score 0.5), so the sign is the collar's own tilt, not an
    # arbitrary offset.
    _SQ_AX = np.array([1.0, 0.0, 0.0])
    _th = math.radians(SQUARE_UP_DEG)
    _c, _s = math.cos(_th), math.sin(_th)
    _K = np.array([[0.0, -_SQ_AX[2], _SQ_AX[1]], [_SQ_AX[2], 0.0, -_SQ_AX[0]], [-_SQ_AX[1], _SQ_AX[0], 0.0]])
    r_sq = r_right @ (np.eye(3) + _s * _K + (1.0 - _c) * (_K @ _K))
    r_left = pitch_demo.rotation(0.0)  # fingers close along world y: across the far plate's faces

    z_hov = TOP + 0.182
    z_tray = TOP + COLLAR_REST_Z + HANDLE_Z_REL - 0.0005
    z_carry = TOP + CARRY_Z_REL
    rz = BAR_Z + HANDLE_Z_REL - 0.0005 + CARRY_SAG
    tug_z = BAR_Z + 0.018
    entry = _on_axis(BAR_END_REL_X - COLLAR_HALF_X - ENTRY_CLEAR_X)
    seat = _on_axis(SEAT_REL_X)
    press = _on_axis(SEAT_REL_X + SEAT_PRESS_X)
    land = _on_axis(SEAT_REL_X + SEAT_LAND_X)
    tug = _on_axis(SEAT_REL_X - TUG_M)
    hold_z = BAR_Z + HOLD_Z_REL
    far_plate = _on_axis(HANDLE_X0 + HANDLE_X1 - 0.020)  # the far 134 mm plate's centre plane
    hold = (far_plate[0] + HOLD_OFF_AXIS, far_plate[1])
    hold_in = (hold[0] + 0.060, hold[1] + 0.060)

    right = [
        W(0.8, (TRAY_X, TRAY_Y, z_hov), o, "hover over the collar in the tray", rot=r_right),
        W(0.8, (TRAY_X, TRAY_Y, z_tray), o, "descend onto the two spring handles", rot=r_right),
        W(0.6, (TRAY_X, TRAY_Y, z_tray), TOUCH_FINGER, "close to the two spring handles", rot=r_right),
        W(3.4, (TRAY_X, TRAY_Y, z_tray), sq, "squeeze the crossed jaws open and take the collar", rot=r_right),
        W(0.5, (TRAY_X, TRAY_Y, z_tray), sq, "settle the squeeze", rot=r_right),
        W(1.6, (TRAY_X, TRAY_Y, z_carry), sq, "lift the open collar out of the holster", rot=r_right),
        W(1.6, (entry[0] + ENTRY_DX, entry[1], z_carry + ENTRY_DZ), sq,
          "carry the open collar round to beyond the bar's free end", rot=r_right),
        W(1.0, (entry[0] + ENTRY_DX, entry[1], rz + ENTRY_DZ), sq,
          "lower the collar onto the bar axis, centred so the shoes pass the end step", rot=r_right),
        W(5.0, (*press, rz), sq, "thread the open collar over the end and along the sleeve to the plate", rot=r_sq),
        W(0.3, (*land, rz), sq, "land the collar square on the plate", rot=r_sq),
        W(0.8, (*press, rz), sq, "drive the seated collar the last millimetres home", rot=r_sq),
        W(SEAT_DWELL_S, (*press, rz), sq, "hold the squeeze at the seat press: the arm takes up its drag", rot=r_sq),
        W(RELEASE_S, (*press, rz), o, "open the fingers while still pressing the collar against the plate", rot=r_right),
        W(0.8, (*seat, z_hov), o, "retract clear of the seated collar", rot=r_right),
        W(0.9, (*seat, tug_z), o, "descend to the collar body for the retention pull", rot=r_right),
        W(0.8, (*seat, tug_z), TUG_FINGER, "close a light pinch on the coil", rot=r_right),
        W(1.5, (*tug, tug_z), TUG_FINGER, "pull 20 mm axially off the plate (retention check)", rot=r_right),
        W(0.7, (*tug, tug_z), o, "open the pinch", rot=r_right),
        W(0.9, (*tug, z_hov), o, "retract clear", rot=r_right),
        W(4.5, (0.18, -0.30, z_hov), o, "park and hold 3 s", rot=r_right),
    ]
    # The left hand must hold the bar until the retention pull is over: the tug's axial drag is exactly what walks an
    # unheld bar along its low-friction cradle. The hold is DERIVED from the right arm's own schedule so it cannot go
    # stale when the plan changes (a fixed 17.6 s stopped covering the tug as soon as the release moved).
    t_r, tug_end = 0.0, 0.0
    for w in right:
        t_r += w.duration
        if w.label.startswith("pull "):
            tug_end = t_r
    left_pre = 0.8 + 1.3 + 0.9 + 0.8
    hold_s = round(max(0.5, tug_end + 0.3 - left_pre), 2)
    left = [
        W(0.8, (0.18, 0.30, z_hov), o, "hold clear while the right takes the collar", rot=r_left),
        W(1.3, (*hold_in, TOP + 0.150), o, "swing in over the far plate's +x rim", rot=r_left),
        W(0.9, (*hold, hold_z), o, "descend over the far plate's rim", rot=r_left),
        W(0.8, (*hold, hold_z), c, "clamp the far plate across its faces: one-hand hold", rot=r_left),
        W(hold_s, (*hold, hold_z), c, "hold the bar through the thread, seat, release and tug", rot=r_left),
        W(0.7, (*hold, hold_z), o, "open the hold", rot=r_left),
        W(1.0, (*hold_in, TOP + 0.150), o, "lift clear", rot=r_left),
        W(4.5, (0.18, 0.30, z_hov), o, "park and hold 3 s", rot=r_left),
    ]
    return {"left": left, "right": right}
