"""cap1: Press three snap caps onto 14 mL culture tubes to the vented first stop.

Physical assembly built with ``labkit`` to the published dimensions of a 14 mL round-bottom polypropylene culture
tube (17 x 100 mm) with its dual-position snap cap, standing in a 3-place 17 mm benchtop rack; the three loose caps
wait open-side down on an absorbent bench pad beside the rack.

The tube is one rigid body. The cap's two stops are a real contact detent: two rows of four spring-loaded rollers sit
in the tube wall below the rim (4 mm radius rollers on radial slide joints with springs), and the cap skirt carries
an inner lip with a lead-in chamfer at its open end.
- A cap set on the tube rests with its lip on the upper row (cavity ceiling 9 mm above the rim).
- First stop (vented, scored 4-6 mm): pressing pushes the lip over the upper row (``ROW1_K`` springs, rollers 0.5 mm
  proud of the lip); the cap then sits 5 mm lower, on the lower row.
- Second stop (sealed, scores 0): a harder press pushes the lip over the lower row (``ROW2_K``, 0.75 mm proud) until
  the cavity ceiling lands on the rim, 9 mm down. Pulling the cap up passes the rows the other way.
No code holds or moves the cap or the rollers: springs, geometry and contact only (travel and forces are our estimates).
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import labkit  # noqa: E402
from yam_arena import demo as yam_demo, station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

SCENE = "station"
# Visual models authored to the real products (assets_build/make_visuals.py); the boxes/cylinders below stay as the
# invisible colliders the physics and scoring were verified with.
VIS = pathlib.Path(__file__).resolve().parent / "assets" / "visual"
TOP = station.TABLE_TOP
SUCCESS_HOLD_S = 1.0

# Shared dimensions & scoring thresholds (never duplicate elsewhere).
TUBE_OD = 0.0170
TUBE_R = 0.0082
TUBE_H = 0.1000
CAP_H = 0.0200
CAP_CAVITY_H = 0.0155
CAP_SKIRT_R_IN = 0.0096
CAP_WALL_T = 0.0010
CAP_CROWN_R = 0.0106
CENTER_CAP = -0.0100
GRIP_CAP = -0.0070

CAPS_XY = (0.240, -0.180)
PAD_SIZE = (0.110, 0.110, 0.0020)  # fixed absorbent bench-protector square the caps wait on

RACK_XY = (0.300, 0.000)
RACK_DECK_H = 0.0580
TUBE_YS = (0.050, 0.000, -0.050)
STAGE_XY = (0.220, 0.000)

# Open-side down on the bench in a triangle (>= 46.6 mm pitch; aligned X for the right-side pair).
CAP_SPAWNS = (
    (CAPS_XY[0] - 0.025, CAPS_XY[1], TOP + PAD_SIZE[2]),
    (CAPS_XY[0] + 0.015, CAPS_XY[1] - 0.024, TOP + PAD_SIZE[2]),
    (CAPS_XY[0] + 0.015, CAPS_XY[1] + 0.024, TOP + PAD_SIZE[2]),
)

# Scoring thresholds shared between score(), success(), and tests.
ENGAGE_MIN_M = 0.0040
ENGAGE_MAX_M = 0.0060
RESTING_MAX_M = 0.0030
SEALED_MIN_M = 0.0080
ALIGN_XY_MAX_M = 0.0050
UPRIGHT_COS_MIN = math.cos(math.radians(20.0))  # 0.9397: tilted 20 deg or less counts
REST_LIN_SPEED_MAX = 0.03
REST_ANG_SPEED_MAX = 0.30

# Contact detent (see module docstring). All lengths in metres.
REST_GAP = 0.0090  # cavity ceiling above the tube rim when the cap rests on the upper row
ROW_PITCH = 0.0050  # upper row to lower row = rest to first stop
LIP_R_IN = 0.0088  # narrowest radius of the cap's inner lip (its top edge)
LIP_R_OUT = 0.0096  # lip radius at the cap's open end: a 30 degree lead-in chamfer guides the cap onto the tube
LIP_ANGLE = math.radians(30.0)  # chamfer face, from vertical
LIP_H = (LIP_R_OUT - LIP_R_IN) / math.tan(LIP_ANGLE)
LIP_T = 0.0006
BALL_R = 0.0040
BALL_LEN = 0.0040
ROW1_PEAK_R = 0.0093  # upper-row roller crest radius at rest: 0.5 mm proud of the lip
ROW2_PEAK_R = 0.00955  # lower row: 0.75 mm proud, stiffer spring: the second stop needs a clearly harder press
BALL_TRAVEL = 0.0014


def _lip_seat_dz(peak_r: float) -> float:
    """Height of the cap's open end above a roller axis when the lip rests on that roller (chamfer face or top edge)."""
    rc = peak_r - BALL_R
    pr = rc + BALL_R * math.cos(LIP_ANGLE)  # where the chamfer face would touch the roller
    if pr >= LIP_R_IN:
        return BALL_R * math.sin(LIP_ANGLE) - (LIP_R_OUT - pr) / math.tan(LIP_ANGLE)
    return math.sqrt(BALL_R**2 - (LIP_R_IN - rc) ** 2) - LIP_H


ROW1_Z = TUBE_H + REST_GAP - CAP_CAVITY_H - _lip_seat_dz(ROW1_PEAK_R)
ROW2_Z = TUBE_H + REST_GAP - CAP_CAVITY_H - ROW_PITCH - _lip_seat_dz(ROW2_PEAK_R)
ROW1_K = 700.0
ROW2_K = 2000.0
ROW1_KD, ROW1_MASS = 40.0, 0.05  # roller masses are large on purpose: they keep the springs stable at the 240 Hz
ROW2_KD, ROW2_MASS = 120.0, 0.10  # physics step and change nothing else (the tube is fixed)
ROW_FRICTION = 0.05  # N of sliding friction in each roller guide; with the heavy damping it keeps a seated cap still
PRESS_DEPTH = 0.0070  # demo: finger tips this far below the resting cap top
COLOR_BEAD = (0.80, 0.84, 0.88)

# Palette (muted lab plastics & clear glassware).
COLOR_PP_CLEAR = (0.86, 0.90, 0.93)
COLOR_PS_CLEAR = (0.82, 0.90, 0.94)
COLOR_BROTH = (0.78, 0.52, 0.14)
COLOR_PATCH = (0.94, 0.94, 0.92)
COLOR_GRAD = (0.14, 0.28, 0.56)
COLOR_LDPE_CAP = (0.92, 0.90, 0.84)
COLOR_LDPE_RING = (0.78, 0.75, 0.68)
COLOR_RACK_BLUE = (0.18, 0.42, 0.56)
COLOR_RACK_LABEL = (0.90, 0.91, 0.92)
COLOR_RACK_FOOT = (0.16, 0.18, 0.20)
COLOR_PAD = (0.95, 0.95, 0.93)

labkit.GLASS_COLORS.add(COLOR_PP_CLEAR)
labkit.GLASS_COLORS.add(COLOR_PS_CLEAR)


def _ring_boxes(radius_in: float, thickness: float, z0: float, height: float, n: int, color) -> list[tuple]:
    """Build an n-sided hollow cylindrical wall out of oriented boxes."""
    r_mid = radius_in + thickness / 2.0
    width = 2.0 * (radius_in + thickness) * math.tan(math.pi / n) + 0.0002
    zc = z0 + height / 2.0
    out = []
    for k in range(n):
        th = 2.0 * math.pi * k / n
        out.append(((r_mid * math.cos(th), r_mid * math.sin(th), zc), (thickness, width, height), color, th))
    return out


def _rack_deck_boxes(z0: float, height: float, color) -> list[tuple]:
    """Build the 3-hole upper or middle guide deck around tubes at RACK_XY."""
    # Rack footprint along X: [-0.028, +0.028], along Y: [-0.080, +0.080].
    # Keep 18.4 mm square-octagonal pockets centered at y in (+0.050, 0.000, -0.050).
    zc = z0 + height / 2.0
    hole_half = 0.0092
    x_side_w = 0.028 - hole_half  # 0.0188
    x_side_c = hole_half + x_side_w / 2.0  # 0.0186
    boxes = [
        ((-x_side_c, 0.0, zc), (x_side_w, 0.160, height), color),
        ((+x_side_c, 0.0, zc), (x_side_w, 0.160, height), color),
    ]
    # Bridges between and outside the three y = +0.05, 0.00, -0.05 holes along x in [-hole_half, +hole_half].
    span_w = 2.0 * hole_half
    y_bands = [
        (0.050 + hole_half, 0.080),
        (hole_half, 0.050 - hole_half),
        (-0.050 + hole_half, -hole_half),
        (-0.080, -0.050 - hole_half),
    ]
    for y_lo, y_hi in y_bands:
        boxes.append(((0.0, 0.5 * (y_lo + y_hi), zc), (span_w, y_hi - y_lo, height), color))
    return boxes


def _make_rack() -> labkit.Part:
    boxes = [
        ((0.0, 0.0, 0.003), (0.056, 0.160, 0.004), COLOR_RACK_BLUE),
        ((0.0, 0.0, 0.0005), (0.054, 0.158, 0.001), COLOR_RACK_FOOT),
    ]
    # Four corner uprights leave front and back bays open to show the tubes & amber broth.
    for sx in (-0.024, 0.024):
        for sy in (-0.076, 0.076):
            boxes.append(((sx, sy, RACK_DECK_H / 2.0), (0.008, 0.008, RACK_DECK_H), COLOR_RACK_BLUE))
    # End ribs and white bench label strip facing -X.
    for sy in (-0.076, 0.076):
        boxes.append(((0.0, sy, RACK_DECK_H / 2.0), (0.048, 0.006, RACK_DECK_H), COLOR_RACK_BLUE))
    boxes.append(((-0.0282, 0.0, 0.055), (0.0008, 0.130, 0.0045), COLOR_RACK_LABEL))
    boxes.extend(_rack_deck_boxes(0.026, 0.004, COLOR_RACK_BLUE))
    boxes.extend(_rack_deck_boxes(0.052, 0.006, COLOR_RACK_BLUE))
    return labkit.part("rack", pos=(*RACK_XY, TOP), fixed=True, mass=None, boxes=boxes, friction=1.0,
                       visuals=[(VIS / "tube_rack.usda", (0.0, 0.0, 0.0))])


def _make_pad() -> labkit.Part:
    boxes = [((0.0, 0.0, PAD_SIZE[2] / 2.0), PAD_SIZE, COLOR_PAD)]
    return labkit.part("pad", pos=(*CAPS_XY, TOP), fixed=True, mass=None, boxes=boxes, friction=1.1,
                       visuals=[(VIS / "bench_pad.usda", (0.0, 0.0, 0.0))])


def _lip_boxes(n: int) -> list[tuple]:
    """The cap's inner snap lip: n slabs whose inner face is the lead-in chamfer (LIP_R_OUT at the open end, narrowing
    to LIP_R_IN at height LIP_H)."""
    length = LIP_H / math.cos(LIP_ANGLE)
    rm = 0.5 * (LIP_R_IN + LIP_R_OUT) + 0.5 * LIP_T * math.cos(LIP_ANGLE)
    zm = 0.5 * LIP_H + 0.5 * LIP_T * math.sin(LIP_ANGLE)
    width = 2.0 * (LIP_R_OUT + LIP_T) * math.tan(math.pi / n) + 0.0002
    out = []
    for k in range(n):
        th = 2.0 * math.pi * k / n
        out.append(((rm * math.cos(th), rm * math.sin(th), zm), (LIP_T, width, length), COLOR_LDPE_CAP, (th, -LIP_ANGLE)))
    return out


def _make_cap(idx: int) -> labkit.Part:
    # Hollow octagonal lower skirt (0..17.5 mm) + overhanging solid crown (17.5..20.0 mm).
    boxes = _ring_boxes(CAP_SKIRT_R_IN, CAP_WALL_T, 0.0, CAP_CAVITY_H, 8, COLOR_LDPE_CAP)
    boxes += _lip_boxes(16)
    cyls = [
        ((0.0, 0.0, 0.5 * (CAP_CAVITY_H + CAP_H)), CAP_CROWN_R, CAP_H - CAP_CAVITY_H, COLOR_LDPE_CAP),
        ((0.0, 0.0, CAP_H + 0.0001), 0.0070, 0.0004, COLOR_LDPE_RING),
    ]
    return labkit.part(f"cap_{idx}", pos=CAP_SPAWNS[idx], mass=0.005, boxes=boxes, cylinders=cyls, friction=0.65,
                       visuals=[(VIS / "snap_cap.usda", (0.0, 0.0, 0.0))])


def _detent_rollers() -> list[labkit.Joint]:
    """Two rows of four spring-loaded rollers in the tube wall: +X, -X, +Y, -Y, each sliding radially."""
    out = []
    for row, (z, k, peak, kd, mass) in enumerate(
        ((ROW1_Z, ROW1_K, ROW1_PEAK_R, ROW1_KD, ROW1_MASS), (ROW2_Z, ROW2_K, ROW2_PEAK_R, ROW2_KD, ROW2_MASS)), start=1
    ):
        rc = peak - BALL_R  # roller axis radius at rest
        for tag, axis, sign in (("xp", "X", 1.0), ("xn", "X", -1.0), ("yp", "Y", 1.0), ("yn", "Y", -1.0)):
            c = (sign * rc, 0.0, z) if axis == "X" else (0.0, sign * rc, z)
            lo, hi = (-BALL_TRAVEL, 0.0) if sign > 0 else (0.0, BALL_TRAVEL)
            out.append(labkit.Joint(
                name=f"r{row}{tag}", kind="slide", axis=axis, anchor=c, lower=lo, upper=hi, stiffness=k,
                damping=kd, friction=ROW_FRICTION, target=0.0, initial=0.0, mass=mass,
                cylinders=[(c, BALL_R, BALL_LEN, COLOR_BEAD, "Y" if axis == "X" else "X")],
            ))
    return out


def _make_tube(idx: int, y: float) -> labkit.Part:
    cyls = [
        ((0.0, 0.0, 0.0470), TUBE_R, 0.0780, COLOR_PP_CLEAR),
        ((0.0, 0.0, 0.0050), 0.0068, 0.0060, COLOR_PP_CLEAR),
        ((0.0, 0.0, 0.0180), 0.0072, 0.0240, COLOR_BROTH),
        ((0.0, 0.0, 0.0360), 0.0085, 0.0012, COLOR_GRAD),
        ((0.0, 0.0, 0.0600), 0.0085, 0.0012, COLOR_GRAD),
        ((0.0, 0.0, 0.09175), TUBE_R, 0.0115, COLOR_PP_CLEAR),  # rigid mouth
        ((0.0, 0.0, 0.09875), 0.0074, 0.0025, COLOR_PP_CLEAR),  # stepped rim, top at TUBE_H
    ]
    boxes = [((-TUBE_R - 0.0002, 0.0, 0.070), (0.0005, 0.0065, 0.016), COLOR_PATCH)]
    return labkit.part(
        f"tube_{idx}",
        pos=(RACK_XY[0], y, TOP),
        fixed=True,
        mass=None,
        cylinders=cyls,
        boxes=boxes,
        joints=_detent_rollers(),
        friction=0.35,
        visuals=[(VIS / "tube_body.usda", (0.0, 0.0, 0.0)), (VIS / "tube_mouth.usda", (0.0, 0.0, 0.0))],
    )


OBJECTS = [_make_rack(), _make_pad(), *(_make_tube(i, y) for i, y in enumerate(TUBE_YS)), *(_make_cap(i) for i in range(3))]
TUBE_NAMES = tuple(f"tube_{i}" for i in range(3))
CAP_NAMES = tuple(f"cap_{i}" for i in range(3))

def _up_z(state) -> float:
    """Return the Z component of an object's local +Z axis in world frame from quat_xyzw."""
    qx, qy, qz, qw = [float(v) for v in state.quat_xyzw]
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm > 0.0:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return 1.0 - 2.0 * (qx * qx + qy * qy)


def _cap_engagement_m(cap, tube) -> float | None:
    """Return how far the cap sits below its resting height on the upper detent row (m), or None if not on the tube."""
    if _up_z(cap) < UPRIGHT_COS_MIN:
        return None
    dx = float(cap.pos[0] - tube.pos[0])
    dy = float(cap.pos[1] - tube.pos[1])
    if math.hypot(dx, dy) > ALIGN_XY_MAX_M:
        return None
    engagement = (float(tube.pos[2]) + TUBE_H + REST_GAP) - (float(cap.pos[2]) + CAP_CAVITY_H)
    if engagement < -0.002 or engagement > 0.020:
        return None
    return engagement


def _rack_upright(objects: dict) -> bool:
    rack = objects.get("rack")
    if rack is not None and _up_z(rack) < UPRIGHT_COS_MIN:
        return False
    return all(_up_z(objects[tn]) >= UPRIGHT_COS_MIN for tn in TUBE_NAMES)


def _tube_vented_status(objects: dict) -> tuple[int, bool]:
    """Count tubes cleanly at vented first stop and whether all caps are stationary."""
    if not _rack_upright(objects):
        return 0, False
    caps = [objects[cn] for cn in CAP_NAMES]
    vented = 0
    all_settled = True
    for tn in TUBE_NAMES:
        tube = objects[tn]
        best_eng = None
        best_cap = None
        for c in caps:
            eng = _cap_engagement_m(c, tube)
            if eng is not None and (best_eng is None or abs(eng - 0.005) < abs(best_eng - 0.005)):
                best_eng = eng
                best_cap = c
        if best_eng is not None and ENGAGE_MIN_M <= best_eng <= ENGAGE_MAX_M:
            vented += 1
            if (
                float(np.linalg.norm(best_cap.lin_vel)) >= REST_LIN_SPEED_MAX
                or float(np.linalg.norm(best_cap.ang_vel)) >= REST_ANG_SPEED_MAX
            ):
                all_settled = False
        else:
            all_settled = False
    return vented, all_settled


def score(objects: dict) -> float:
    """Counted tubes / 3 (vented first stop 4-6 mm, rack upright): 0.0, 0.33, 0.67, 1.0.
    1.0 also needs all three counted caps still; three counted caps with any cap still moving score 0.67."""
    vented, settled = _tube_vented_status(objects)
    if vented == 3 and not settled:
        return 0.67
    return round(vented / 3.0, 2)


def success(objects: dict) -> bool:
    """True only when all 3 tubes are seated at the vented first stop, upright, and settled."""
    vented, settled = _tube_vented_status(objects)
    return vented == 3 and settled


# One scored sub-action per tube: its cap pressed to the vented first stop. Printed to the run log (observe-only).
SUBACTIONS = tuple((f"press snap cap onto {tn} to the vented first stop", tn) for tn in TUBE_NAMES)
FIRST_STOP_ENGAGE_M = 0.0050  # nominal first-stop engagement; scored band ENGAGE_MIN_M..ENGAGE_MAX_M
_TRIAL = {"step": -1, "ok_s": 0.0}


@labkit.on_reset
def _reset_trial(_env) -> None:
    _TRIAL["step"], _TRIAL["ok_s"] = -1, 0.0


def _log_subaction_end(objects: dict, step: int, run_end: bool) -> None:
    """Print one SUBACTION_END JSON line per tube: the capping cap's origin in env-frame m (body_pos_m), its measured
    axial engagement below its resting height, the 0-based step, and run_end (the success hold
    is complete, so the runtime ends the run on this step)."""
    caps = {cn: objects[cn] for cn in CAP_NAMES}
    for name, tn in SUBACTIONS:
        tube = objects[tn]
        best = None
        for cn, c in caps.items():
            eng = _cap_engagement_m(c, tube)
            if eng is not None and (best is None or abs(eng - FIRST_STOP_ENGAGE_M) < abs(best[1] - FIRST_STOP_ENGAGE_M)):
                best = (cn, eng)
        cn, eng = best if best is not None else (None, None)
        rec = {
            "name": name,
            "joint_or_body": cn,
            "end_value": [round(float(v), 6) for v in caps[cn].pos] if cn else None,
            "engagement_m": round(float(eng), 6) if eng is not None else None,
            "band_m": [ENGAGE_MIN_M, ENGAGE_MAX_M],
            "step": step,
            "run_end": run_end,
        }
        print("SUBACTION_END " + json.dumps(rec), flush=True)


@labkit.on_step
def _watch_subactions(env) -> None:
    """Observe-only: on every step of the full-score hold print the sub-action end states (no state is changed)."""
    _TRIAL["step"] += 1
    names = ("rack", *TUBE_NAMES, *CAP_NAMES)
    objects = {n: labkit._part_state(env, labkit._SPECS[n]) for n in names}
    dt = float(getattr(env.unwrapped, "step_dt", 0.05))
    _TRIAL["ok_s"] = _TRIAL["ok_s"] + dt if success(objects) else 0.0
    if _TRIAL["ok_s"] > 0.0:
        _log_subaction_end(objects, _TRIAL["step"], _TRIAL["ok_s"] >= SUCCESS_HOLD_S - 0.5 * dt)


def _pick_centered_cap(
    xy: tuple[float, float],
    label: str,
    floor_offset: float = 0.0,
    align_yaw: float = math.pi / 2,
    pre_open: float = -0.0180,
) -> list[Waypoint]:
    """Two-axis centering grasp: gentle X squeeze prevents climbing, then firm Y grasp & lift."""
    x, y = xy
    z_grip = TOP + floor_offset + 0.0080 + PAD_TIP_PAST_TCP
    z_turn = TOP + 0.052
    z_lift = TOP + 0.142
    return [
        Waypoint(0.8, (x, y, z_turn), pre_open, f"over {label}", yaw=align_yaw),
        Waypoint(0.7, (x, y, z_grip), pre_open, f"down X-align {label}", yaw=align_yaw),
        Waypoint(0.4, (x, y, z_grip), CENTER_CAP, f"center X {label}", yaw=align_yaw),
        Waypoint(0.3, (x, y, z_grip), pre_open, f"release X {label}", yaw=align_yaw),
        Waypoint(0.4, (x, y, z_turn), pre_open, f"clear cap X {label}", yaw=align_yaw),
        Waypoint(0.5, (x, y, z_turn), pre_open, f"turn Y {label}", yaw=0.0),
        Waypoint(0.6, (x, y, z_grip), pre_open, f"down Y-align {label}", yaw=0.0),
        Waypoint(0.5, (x, y, z_grip), GRIP_CAP, f"grasp {label}", yaw=0.0),
        Waypoint(0.9, (x, y, z_lift), GRIP_CAP, f"lift {label}", yaw=0.0),
    ]


def _place_and_press(
    target_xy: tuple[float, float],
    grip_z_above_cap_bottom: float,
    label: str,
    park_after: tuple[float, float, float] | None = None,
) -> list[Waypoint]:
    """Carry over tube, settle wrist, lower the cap until its open end is 1.5 mm below the rim and release it onto the
    upper detent row, then press to the 5 mm first stop (finger tips PRESS_DEPTH below the resting cap top: deep enough
    for the first click, short of the second)."""
    tx, ty = target_xy
    z_carry = TOP + 0.142
    z_place = TOP + TUBE_H - 0.0015 + grip_z_above_cap_bottom
    z_press_clear = TOP + TUBE_H + REST_GAP + 0.022 + PAD_TIP_PAST_TCP
    z_press_down = TOP + TUBE_H + REST_GAP + (CAP_H - CAP_CAVITY_H) - PRESS_DEPTH + PAD_TIP_PAST_TCP
    pre_open = -0.022
    wps = [
        Waypoint(1.1, (tx, ty, z_carry), GRIP_CAP, f"carry over {label}", yaw=0.0),
        Waypoint(0.45, (tx, ty, z_carry), GRIP_CAP, f"settle over {label}", yaw=0.0),
        Waypoint(0.6, (tx, ty, z_place + 0.012), GRIP_CAP, f"hover over {label}", yaw=0.0),
        Waypoint(0.4, (tx, ty, z_place + 0.012), GRIP_CAP, f"settle hover {label}", yaw=0.0),
        Waypoint(0.5, (tx, ty, z_place), GRIP_CAP, f"seat on {label}", yaw=0.0),
        Waypoint(0.4, (tx, ty, z_place), pre_open, f"release {label}", yaw=0.0),
        Waypoint(0.4, (tx, ty, z_press_clear), pre_open, f"up to crown {label}", yaw=0.0),
        Waypoint(0.4, (tx, ty, z_press_clear), FINGER_CLOSED, f"close tips {label}", yaw=0.0),
        Waypoint(1.2, (tx, ty, z_press_down), FINGER_CLOSED, f"press first stop {label}", yaw=0.0),
        Waypoint(0.2, (tx, ty, z_press_down), FINGER_CLOSED, f"hold press {label}", yaw=0.0),
        Waypoint(0.3, (tx, ty, z_carry), FINGER_CLOSED, f"retract {label}", yaw=0.0),
    ]
    if park_after is not None:
        # The run ends 1 s after the last click: be clear of the rack by then.
        wps.append(Waypoint(0.6, park_after, FINGER_CLOSED, f"park after {label}", yaw=0.0))
    return wps


def demo() -> dict[str, list[Waypoint]]:
    """Right arm stages cap_0 at midline for left arm, then caps tube_2 and tube_1; left arm caps tube_0."""
    grip_off_bench = 0.0210  # TCP above cap bottom when placing a cap picked by the right arm
    grip_off_stage = 0.0190

    left_park = (0.14, 0.25, TOP + 0.140)
    right_park = (0.14, -0.25, TOP + 0.140)
    sx, sy = STAGE_XY

    # Step 1 (Right): pick cap_0 from the bench, place at midline STAGE_XY, retreat back over the caps.
    r_stage_cap0 = [
        *_pick_centered_cap(CAP_SPAWNS[0][:2], "cap_0", floor_offset=PAD_SIZE[2], align_yaw=math.pi / 2),
        Waypoint(1.2, (sx, sy, TOP + 0.142), GRIP_CAP, "carry cap_0 to stage", yaw=0.0),
        Waypoint(0.5, (sx, sy, TOP + 0.142), GRIP_CAP, "settle stage xy", yaw=0.0),
        Waypoint(0.9, (sx, sy, TOP + grip_off_bench), GRIP_CAP, "lower cap_0 onto stage", yaw=0.0),
        Waypoint(0.35, (sx, sy, TOP + grip_off_bench), GRIP_CAP, "settle cap_0 stage", yaw=0.0),
        Waypoint(0.4, (sx, sy, TOP + grip_off_bench), -0.022, "release cap_0 on stage", yaw=0.0),
        Waypoint(0.5, (sx, sy, TOP + 0.120), -0.022, "clear stage", yaw=0.0),
        Waypoint(0.9, (CAP_SPAWNS[2][0], CAP_SPAWNS[2][1], TOP + 0.120), -0.0180, "retreat to caps", yaw=math.pi / 2),
    ]
    t_stage_done = sum(w.duration for w in r_stage_cap0) - 0.9

    # Left picks cap_0 from stage with X+Y centering after Right retreats, caps tube_0 (y=+0.05), and parks.
    l_wait_stage = t_stage_done + 0.2
    l_cap0 = [
        Waypoint(l_wait_stage, left_park, FINGER_OPEN, "wait for stage handoff", yaw=0.0),
        *_pick_centered_cap(STAGE_XY, "staged cap_0", floor_offset=0.0, align_yaw=-math.pi / 2, pre_open=-0.022),
        *_place_and_press((RACK_XY[0], TUBE_YS[0]), grip_off_stage, "tube_0", park_after=left_park),
    ]
    # Time when Left arm has finished picking staged cap_0 & carried over tube_0 (y=+0.050):
    t_l_clear_midline = l_wait_stage + 5.1 + 1.0
    r_wait_for_l_pick = max(0.2, t_l_clear_midline - sum(w.duration for w in r_stage_cap0))

    # Step 2 (Right): wait for Left to clear midline, then cap tube_2 and tube_1.
    r_cap2_and_1 = [
        Waypoint(r_wait_for_l_pick, (CAP_SPAWNS[2][0], CAP_SPAWNS[2][1], TOP + 0.120), -0.0170, "wait left clear stage", yaw=math.pi / 2),
        *_pick_centered_cap(CAP_SPAWNS[2][:2], "cap_2", floor_offset=PAD_SIZE[2], align_yaw=math.pi / 2),
        *_place_and_press((RACK_XY[0], TUBE_YS[2]), grip_off_bench, "tube_2"),
        *_pick_centered_cap(CAP_SPAWNS[1][:2], "cap_1", floor_offset=PAD_SIZE[2], align_yaw=math.pi / 2),
        *_place_and_press((RACK_XY[0], TUBE_YS[1]), grip_off_bench, "tube_1", park_after=right_park),
    ]

    return {"left": l_cap0, "right": [*r_stage_cap0, *r_cap2_and_1]}
