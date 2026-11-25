"""Two-piece board-game box on the station bench: open packed base (tile stack, card deck, wooden token) inside four
yellow corner tape marks, and matching telescoping lid top-up in front. Both horizontal spans exceed one 94 mm jaw, and
the four 53 mm skirts clear the base by 1.5 mm per side: both arms squeeze opposite outer short walls, carry level, and
seat all four sides without wedging, spilling contents, or shifting the base.

All figures live in ``parts.py``. Pure 240 Hz PhysX 5-wall compound contact (``contact_offset = 0.4 mm``,
``sense_contact("box_lid", "box_base")``): zero welds, zero hidden joints, zero computed pushes.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import kit_extra as kx  # noqa: E402
import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
from mechkit import body, sense_contact  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = P.SUCCESS_HOLD_S  # 3.5 s after score == 1.0 so both arms finish parking at READY
INSTRUCTION = (
    "Put the lid squarely onto the packed game box so it seats all the way down over the base "
    "without spilling the contents or moving the box off its marks."
)
RUN_LIMIT_STEPS = P.RUN_LIMIT_STEPS  # 1,200 steps (60 s at 20 Hz)
TOP = TABLE_TOP[SCENE]

CONTENTS = ("tile_stack", "card_deck", "wood_pawn")
FREE_BODIES = ("box_base", "box_lid", *CONTENTS)
_CS = P.content_specs()

body("box_base", P.base_shapes(), pos=(P.BASE_AT[0], P.BASE_AT[1], TOP + P.OVER), mass=P.BASE_MASS,
     friction=(P.BOARD_FRICTION, P.BOARD_FRICTION), contact_offset=P.BOX_CONTACT_OFFSET, contacts=True)  # fmt: skip
body("box_lid", P.lid_shapes(), pos=(P.LID_AT[0], P.LID_AT[1], TOP + P.OVER), mass=P.LID_MASS,
     friction=(P.BOARD_FRICTION, P.BOARD_FRICTION), contact_offset=P.BOX_CONTACT_OFFSET, contacts=True)  # fmt: skip
for _name, _fn in (("tile_stack", P.tile_stack_shapes), ("card_deck", P.card_deck_shapes), ("wood_pawn", P.wood_pawn_shapes)):
    _lp = _CS[_name]["local_pos"]
    body(_name, _fn(), pos=(P.BASE_AT[0] + _lp[0], P.BASE_AT[1] + _lp[1], TOP + P.OVER + _lp[2]),
         mass=_CS[_name]["mass"], friction=(P.BOARD_FRICTION, P.BOARD_FRICTION), contact_offset=P.CONTENT_CONTACT_OFFSET)  # fmt: skip
body("corner_marks", P.corner_mark_shapes(), pos=(P.BASE_AT[0], P.BASE_AT[1], TOP), mass=None)

sense_contact("box_lid", "box_base")
OBJECTS = list(mk.MECH.bodies.values())

_CORNER_SIGNS = ((-1.0, -1.0), (-1.0, +1.0), (+1.0, -1.0), (+1.0, +1.0))
BASE_START_CORNERS_XY = np.array(
    [[P.BASE_AT[0] + 0.5 * sx * P.BASE_W, P.BASE_AT[1] + 0.5 * sy * P.BASE_L] for sx, sy in _CORNER_SIGNS], dtype=float
)

EVENTS = kx.Events({"lid on box": 0.5, "lid seated": 1.0})
STATE: dict = {}


def begin() -> None:
    STATE.clear()
    STATE.update(
        fault=None,
        events=[],
        last_time=0.0,
        full_held_s=0.0,
        eng_held_s=0.0,
        max_base_tilt_deg=0.0,
        max_corner_shift_m=0.0,
        max_lid_base_force_n=0.0,
    )
    EVENTS.clear()


begin()


@mk.on_reset
def start() -> None:
    begin()


def base_tilt_deg() -> float:
    _, br = mk.pose("box_base")
    return math.degrees(math.acos(max(-1.0, min(1.0, float(br[2, 2])))))


def base_corner_shift_m() -> float:
    bp, br = mk.pose("box_base")
    return max(
        float(np.linalg.norm((bp + br @ np.array([0.5 * sx * P.BASE_W, 0.5 * sy * P.BASE_L, 0.0]))[:2] - mark_xy))
        for (sx, sy), mark_xy in zip(_CORNER_SIGNS, BASE_START_CORNERS_XY)
    )


def content_in_base(name: str) -> bool:
    bp, br = mk.pose("box_base")
    rel = br.T @ (mk.pose(name)[0] - bp)
    return bool(abs(float(rel[0])) <= 0.5 * P.BASE_W - P.BOARD_T and abs(float(rel[1])) <= 0.5 * P.BASE_L - P.BOARD_T and 0.0 <= float(rel[2]) <= P.BASE_H)


def spilled_contents() -> list[str]:
    return [name for name in CONTENTS if not content_in_base(name)]


def lid_metrics() -> dict:
    bp, br = mk.pose("box_base")
    lp, lr = mk.pose("box_lid")
    rel_rot = br.T @ lr
    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(rel_rot[2, 2])))))
    long_xy = rel_rot[:2, 1]
    horiz = float(np.hypot(long_xy[0], long_xy[1]))
    yaw_deg = math.degrees(math.atan2(abs(float(long_xy[0])), abs(float(long_xy[1])))) if horiz > 1e-9 else 90.0
    mid_in_b = br.T @ ((lp + lr @ np.array([0.0, 0.0, 0.5 * P.LID_OUT_H])) - bp)
    roof_in_b = br.T @ ((lp + lr @ np.array([0.0, 0.0, P.LID_OUT_H])) - bp)
    corner_z = [
        float((br.T @ ((lp + lr @ np.array([sx * 0.5 * P.LID_OUT_W, sy * 0.5 * P.LID_OUT_L, P.LID_OUT_H])) - bp))[2])
        for sx in (-1.0, +1.0)
        for sy in (-1.0, +1.0)
    ]
    return {
        "roof_h": float(roof_in_b[2]),
        "tilt_deg": tilt_deg,
        "yaw_deg": yaw_deg,
        "offset_m": float(np.hypot(mid_in_b[0], mid_in_b[1])),
        "mid_in_b": (float(mid_in_b[0]), float(mid_in_b[1])),
        "corner_z": corner_z,
        "corner_dev_mm": 1000.0 * max(abs(z - P.CLOSED_H) for z in corner_z),
    }


def _obb_distance(point: np.ndarray, center: np.ndarray, rot: np.ndarray, half_extents: np.ndarray) -> float:
    return float(np.linalg.norm(np.maximum(np.abs(rot.T @ (point - center)) - half_extents, 0.0)))


def gripper_clearance_m(side: str) -> float:
    tcp = np.asarray(mk.gripper(side)[0], dtype=float)
    bp, br = mk.pose("box_base")
    lp, lr = mk.pose("box_lid")
    d_base = _obb_distance(tcp, bp + br @ np.array([0.0, 0.0, 0.5 * P.BASE_H]), br, np.array([0.5 * P.BASE_W, 0.5 * P.BASE_L, 0.5 * P.BASE_H]))
    d_lid = _obb_distance(tcp, lp + lr @ np.array([0.0, 0.0, 0.5 * P.LID_OUT_H]), lr, np.array([0.5 * P.LID_OUT_W, 0.5 * P.LID_OUT_L, 0.5 * P.LID_OUT_H]))
    return min(d_base, d_lid)


def grippers_clear() -> bool:
    return all(gripper_clearance_m(side) >= P.GRIPPER_CLEAR_MIN for side in ("left", "right"))


def all_at_rest() -> bool:
    for name in FREE_BODIES:
        v_lin, v_ang = mk.velocity(name)
        if float(np.linalg.norm(v_lin)) >= P.REST_LIN_MAX or float(np.linalg.norm(v_ang)) >= P.REST_ANG_MAX:
            return False
    return True


def skirts_straddle_base() -> bool:
    """True iff all 4 inner skirt walls of ``box_lid`` straddle strictly outside all 4 outer walls of ``box_base``."""
    bp, br = mk.pose("box_base")
    lp, lr = mk.pose("box_lid")
    for sx in (-1.0, +1.0):
        for sy in (-1.0, +1.0):
            c_in_l = lr.T @ ((bp + br @ np.array([0.5 * sx * P.BASE_W, 0.5 * sy * P.BASE_L, P.BASE_H])) - lp)
            z_l = float(c_in_l[2])
            if z_l < 0.0:
                return False
            flare = P.CHAMFER_IN * max(0.0, 1.0 - z_l / max(P.CHAMFER_H, 1e-6)) if z_l < P.CHAMFER_H else 0.0
            if abs(float(c_in_l[0])) >= 0.5 * P.LID_IN_W + flare + P.STRADDLE_SOLVER_TOL:
                return False
            if abs(float(c_in_l[1])) >= 0.5 * P.LID_IN_L + flare + P.STRADDLE_SOLVER_TOL:
                return False
    rel_rot = br.T @ lr
    if abs(float(rel_rot[1, 1])) < abs(float(rel_rot[0, 1])):
        return False
    sgn_y = 1.0 if float(rel_rot[1, 1]) >= 0.0 else -1.0
    sgn_x = 1.0 if float(rel_rot[0, 0]) >= 0.0 else -1.0
    for sgn in (-1.0, +1.0):
        wx_b = br.T @ ((lp + lr @ np.array([sgn * 0.5 * P.LID_IN_W, 0.0, 0.0])) - bp)
        wy_b = br.T @ ((lp + lr @ np.array([0.0, sgn * 0.5 * P.LID_IN_L, 0.0])) - bp)
        if sgn * sgn_x * float(wx_b[0]) <= 0.5 * P.BASE_W:
            return False
        if sgn * sgn_y * float(wy_b[1]) <= 0.5 * P.BASE_L:
            return False
    return True


def lid_over_base() -> bool:
    """True when the lid's mid-height center sits inside box_base's outer footprint (the lid is over the box)."""
    mid_x, mid_y = lid_metrics()["mid_in_b"]
    return abs(mid_x) <= 0.5 * P.BASE_W and abs(mid_y) <= 0.5 * P.BASE_L


def seated_now() -> bool:
    """Seated (the 1.0 geometry): roof height only -- every lid roof corner within SEAT_CORNER_TOL of CLOSED_H,
    lid tilt at most SEAT_TILT_MAX_DEG, and the lid over the box. No centring offset: the walls bound it."""
    if STATE["fault"] is not None or spilled_contents() or not grippers_clear() or not all_at_rest():
        return False
    if base_corner_shift_m() > P.BASE_SHIFT_MAX or base_tilt_deg() > P.BASE_TILT_MAX_DEG:
        return False
    m = lid_metrics()
    return bool(m["corner_dev_mm"] <= 1000.0 * P.SEAT_CORNER_TOL and m["tilt_deg"] <= P.SEAT_TILT_MAX_DEG and lid_over_base())


def on_box_now() -> bool:
    """Half level: the lid rests ON the box but is not seated -- all free bodies at rest, hands clear, the lid
    over the box and pressing on it (contact >= ON_BOX_MIN_CONTACT_N), tilt at most ON_BOX_TILT_MAX_DEG, and the
    seated geometry above fails (a corner stands proud). Straddle is not required: a lid cocked with one skirt
    inside the base is the everyday half result."""
    if STATE["fault"] is not None or spilled_contents() or not grippers_clear() or not all_at_rest():
        return False
    if not lid_over_base() or mk.contact_force("box_lid", "box_base") < P.ON_BOX_MIN_CONTACT_N:
        return False
    if lid_metrics()["tilt_deg"] > P.ON_BOX_TILT_MAX_DEG:
        return False
    return not seated_now()


@mk.on_step
def step() -> None:
    now = mk.now()
    dt = max(0.0, now - STATE["last_time"]) if STATE["last_time"] > 0.0 else (now if now > 0.0 else 1.0 / P.CONTROL_HZ)
    STATE["last_time"] = now

    tilt, shift, spilled = base_tilt_deg(), base_corner_shift_m(), spilled_contents()
    force = mk.contact_force("box_lid", "box_base")
    STATE["max_base_tilt_deg"] = max(STATE["max_base_tilt_deg"], tilt)
    STATE["max_corner_shift_m"] = max(STATE["max_corner_shift_m"], shift)
    STATE["max_lid_base_force_n"] = max(STATE["max_lid_base_force_n"], force)

    if STATE["fault"] is None:
        if tilt > P.BASE_TILT_MAX_DEG:
            STATE["fault"] = (now, f"box_base tilted {tilt:.2f} deg > {P.BASE_TILT_MAX_DEG:.1f} deg")
            STATE["events"].append(STATE["fault"])
        elif shift > P.BASE_SHIFT_MAX:
            STATE["fault"] = (now, f"box_base corner shifted {1000 * shift:.2f} mm > {1000 * P.BASE_SHIFT_MAX:.1f} mm")
            STATE["events"].append(STATE["fault"])
        elif spilled:
            STATE["fault"] = (now, f"contents left box_base: {', '.join(spilled)}")
            STATE["events"].append(STATE["fault"])

    eng, full = on_box_now(), seated_now()
    STATE["full_held_s"] = round(STATE["full_held_s"] + dt, 9) if full else 0.0
    STATE["eng_held_s"] = round(STATE["eng_held_s"] + dt, 9) if eng else 0.0
    EVENTS.note("lid on box", eng, now)
    EVENTS.note("lid seated", full, now)


def score(_objects) -> float:
    if STATE["fault"] is not None:
        return 0.0
    if seated_now() and STATE["full_held_s"] >= P.FULL_HOLD_S - 1e-9:
        return 1.0
    return 0.5 if on_box_now() and STATE["eng_held_s"] >= P.ENGAGE_HOLD_S - 1e-9 else 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out: dict = {
        "preset": P.BOX_PRESET,
        "fault": None if STATE["fault"] is None else f"{STATE['fault'][0]:.2f} s: {STATE['fault'][1]}",
        "events": [f"{t:.2f} s: {what}" for t, what in STATE["events"]],
        "order": EVENTS.report(),
        "full_held_s": round(STATE["full_held_s"], 3),
        "eng_held_s": round(STATE["eng_held_s"], 3),
        "max_base_tilt_deg": round(STATE["max_base_tilt_deg"], 2),
        "max_corner_shift_mm": round(1000 * STATE["max_corner_shift_m"], 2),
        "max_lid_base_force_N": round(STATE["max_lid_base_force_n"], 3),
    }
    if mk.MECH.poses:
        m = lid_metrics()
        out.update(
            {
                "roof_h_mm": round(1000 * m["roof_h"], 2),
                "closed_h_mm": round(1000 * P.CLOSED_H, 2),
                "roof_corner_dev_mm": round(m["corner_dev_mm"], 2),
                "lid_tilt_deg": round(m["tilt_deg"], 2),
                "lid_yaw_deg": round(m["yaw_deg"], 2),
                "lid_offset_mm": round(1000 * m["offset_m"], 2),
                "lid_over_base": lid_over_base(),
                "lid_on_base_contact_N": round(mk.contact_force("box_lid", "box_base"), 3),
                "skirts_straddle": skirts_straddle_base(),
                "base_corner_shift_mm": round(1000 * base_corner_shift_m(), 2),
                "spilled_contents": spilled_contents(),
                "grippers_clear": grippers_clear() if mk.MECH.grippers else None,
                "all_at_rest": all_at_rest(),
            }
        )
    return out
