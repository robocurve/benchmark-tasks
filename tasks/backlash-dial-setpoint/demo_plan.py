"""Scripted bimanual demo and ablations for backlash dial setpoint. Pure Python + NumPy.

A plan is a list of OPS executed sequentially:
  ("lock", side)                       press LOCK button beside rail with side ("l" or "r") arm
  ("push", side, s_from, s_to)         push knob from setting s_from to s_to (mm)

Modes (selected by environment variables, see run.sh):
  demo         Left knob pushed 30 -> 65 mm (setpoint), locked.
               Right knob pushed 70 -> 30 mm (overshoot down by 10 mm past 40 mm setpoint), reversed and pushed 30 -> 40 mm (setpoint), locked.
               Final hold at home with open grippers (score 1.00).
  naive        Left knob pushed 30 -> 65 mm, locked.
               Right knob pushed 70 -> 40 mm directly without reversing (backlash not taken up, display lags at 46 mm),
               locked outside band (score 0.50).
  no_lock      Both knobs positioned at setpoints in band, but neither arm presses LOCK (score 0.25).
  settle       Parts rest at start positions, arms hold at home for 8 s with open grippers (score 0.00).

Knobs for GPU tuning:
  DEMO_TIME_SCALE (default 1.8)
  DEMO_PUSH_OVERSHOOT_MM (default 8.0, proven on GPU in Task G)
  DEMO_PRESS_OVERSHOOT_MM (default 5.0)
  DEMO_PUSH_SPEED_MM_S (default 40.0)
  DEMO_YAW_SHIFT_LEFT, DEMO_YAW_SHIFT_RIGHT (default 0.0)
"""

from __future__ import annotations

import os

import geometry as g
import logic
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint

PAD_TO_TCP = g.PAD_TO_TCP
Z_HOVER = 0.840
HOME = {
    "l": (0.200, 0.220, Z_HOVER),
    "r": (0.200, -0.220, Z_HOVER),
}
TRACK_Y = {"l": g.TRACK_Y["left"], "r": g.TRACK_Y["right"]}
START_MM = {"l": g.KNOB_START_SETTING_MM["left"], "r": g.KNOB_START_SETTING_MM["right"]}
SETPOINTS_MM = {"l": g.SETPOINT_MM["left"], "r": g.SETPOINT_MM["right"]}

GPU_PUSH_OVERSHOOT_MM = 6.5  # mm; Task G used 8, but GPU run 3 landed the right dial at 42.6 (target 40, band edge 42.5)
GPU_LEFT_OVERSHOOT_MM = 6.0  # GPU run 2: speed 20 + overshoot 8 put left at 67.6 (target 65)
GPU_LEFT_SPEED_MM_S = 20.0   # GPU run 1: speed 40 stalled the left knob at 50 mm
DEFAULT_TIME_SCALE = 1.8


def _b(name: str, default: bool = False) -> bool:
    return os.environ.get(name, "1" if default else "0") in ("1", "true", "True")


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if _b("DEMO_SETTLE_ONLY"):
        return "settle"
    if _b("DEMO_NAIVE"):
        return "naive"
    if _b("DEMO_NO_LOCK"):
        return "no_lock"
    return "demo"


def build_ops(mode: str) -> list[tuple]:
    """Return sequence of operations for the given mode."""
    if mode == "settle":
        return []

    sp_l = SETPOINTS_MM["l"]  # 65.0
    sp_r = SETPOINTS_MM["r"]  # 40.0
    st_l = START_MM["l"]      # 30.0
    st_r = START_MM["r"]      # 70.0

    if mode == "demo":
        # Left dial: approach from below (30 -> 65 mm), then LOCK
        # Right dial: overshoot DOWN by 10 mm past setpoint 40 mm (70 -> 30 mm),
        # then reverse and approach UPWARD (30 -> 40 mm), then LOCK
        return [
            ("push", "l", st_l, sp_l),
            ("lock", "l"),
            ("push", "r", st_r, 30.0),
            ("push", "r", 30.0, sp_r),
            ("lock", "r"),
        ]

    if mode == "naive":
        # Naive ablation: single push to nominal target from wrong direction without reversal
        # Right dial: pushed from 70 mm directly down to 40 mm -> display at 46 mm (outside band)
        return [
            ("push", "l", st_l, sp_l),
            ("lock", "l"),
            ("push", "r", st_r, sp_r),
            ("lock", "r"),
        ]

    if mode == "no_lock":
        # Positions both dials correctly in band but never presses LOCK
        return [
            ("push", "l", st_l, sp_l),
            ("push", "r", st_r, 30.0),
            ("push", "r", 30.0, sp_r),
        ]

    raise ValueError(f"unknown mode {mode}")


def knob_state_after(ops: list[tuple]) -> list[tuple[float, float]]:
    """Nominal (left, right) knob settings after each op."""
    sl, sr = START_MM["l"], START_MM["r"]
    out = []
    for o in ops:
        if o[0] == "push":
            if o[1] == "l":
                sl = o[3]
            else:
                sr = o[3]
        out.append((sl, sr))
    return out


def _z_push() -> float:
    return g.KNOB_BOTTOM_Z + _f("DEMO_PAD_HEIGHT_MM", g.PUSH_PAD_HEIGHT * 1000) / 1000 + PAD_TO_TCP


def _z_press() -> float:
    """TCP z commanding pad below end stop of LOCK button."""
    over = _f("DEMO_PRESS_OVERSHOOT_MM", g.BUTTON_PRESS_OVERSHOOT * 1000) / 1000
    return g.BUTTON_CAP_TOP_REST_Z - g.BUTTON_TRAVEL - over + PAD_TO_TCP


def push_targets(s_from: float, s_to: float, side: str = "r") -> tuple[float, float]:
    """TCP x at start and end of a push (m). Overshoot/speed can differ per side (GPU run 2: left needs slower+less)."""
    w = g.push_contact_half_depth()
    over = _f(f"DEMO_PUSH_OVERSHOOT_{side.upper()}_MM", _f("DEMO_PUSH_OVERSHOOT_MM", GPU_LEFT_OVERSHOOT_MM if side == "l" else GPU_PUSH_OVERSHOOT_MM)) / 1000.0
    if s_to >= s_from:  # forward push: fingers behind knob, contact at rear top edge
        rear = lambda s: g.CHANNEL_NEAR_STOP_X + s / 1000.0
        return rear(s_from) - w - g.PUSH_APPROACH_CLEAR, rear(s_to) - w + over
    # backward push: fingers in front of knob, contact at front top edge
    front = lambda s: g.CHANNEL_NEAR_STOP_X + s / 1000.0 + g.KNOB_L
    return front(s_from) + w + g.PUSH_APPROACH_CLEAR, front(s_to) + w - over


def plans(mode: str | None = None, with_meta: bool = False):
    """Waypoint plans for both arms (and, with ``with_meta``, label -> info dict)."""
    mode = mode or mode_from_env()
    yaw = {"l": _f("DEMO_YAW_SHIFT_LEFT", 0.0), "r": _f("DEMO_YAW_SHIFT_RIGHT", 0.0)}
    speed = _f("DEMO_PUSH_SPEED_MM_S", 40.0)
    time_scale = _f("DEMO_TIME_SCALE", DEFAULT_TIME_SCALE)

    seqs: dict[str, list[Waypoint]] = {"l": [], "r": []}
    meta: dict[str, dict] = {}

    def add(side, dur, pos, finger, label, op, kind, **extra):
        full = f"{side}: {label} #{op:02d}"
        scaled_dur = dur * time_scale
        seqs[side].append(Waypoint(scaled_dur, pos, finger, full, yaw[side]))
        meta[full] = dict(op=op, kind=kind, side=side, base_dur=dur, **extra)

    if mode == "settle":
        hold = _f("DEMO_SETTLE_HOLD_S", 8.0)
        for s in ("l", "r"):
            add(s, hold, HOME[s], FINGER_OPEN, "settle hold", 0, "hold")
        out = {"left": seqs["l"], "right": seqs["r"]}
        return (out, meta) if with_meta else out

    ops = build_ops(mode)
    after = knob_state_after(ops)
    z_push, z_press = _z_push(), _z_press()

    for i, o in enumerate(ops):
        side = o[1]
        other = "r" if side == "l" else "l"
        t0 = sum(w.duration for w in seqs[side])

        if o[0] == "lock":
            side_name = "left" if side == "l" else "right"
            btn_xy = g.BUTTON_POS_XY[side_name]
            add(side, 0.6, (*btn_xy, Z_HOVER), FINGER_CLOSED, "hover lock", i, "btn_hover")
            add(side, 0.5, (*btn_xy, z_press), FINGER_CLOSED, "press lock", i, "btn_press")
            add(side, 0.4, (*btn_xy, z_press), FINGER_CLOSED, "hold lock", i, "btn_press")
            add(side, 0.5, (*btn_xy, Z_HOVER), FINGER_CLOSED, "release lock", i, "btn_hover")
            add(side, 0.6, HOME[side], FINGER_CLOSED, "back home", i, "transit")
        else:
            _, _, s_from, s_to = o
            x_from, x_to = push_targets(s_from, s_to, side)
            ty = TRACK_Y[side]
            side_speed = _f(f"DEMO_PUSH_SPEED_{side.upper()}_MM_S", GPU_LEFT_SPEED_MM_S if side == "l" else speed)
            dur = 0.6 + abs(s_to - s_from) / side_speed
            kw = dict(pushed=side, s_from=s_from, s_to=s_to)
            add(side, 0.6, (x_from, ty, Z_HOVER), FINGER_CLOSED, "hover push", i, "push_hover", **kw)
            add(side, 0.5, (x_from, ty, z_push), FINGER_CLOSED, "descend push", i, "push_descend", **kw)
            add(side, dur, (x_to, ty, z_push), FINGER_CLOSED, "push to", i, "push_contact", **kw)
            add(side, 0.5, (x_to, ty, Z_HOVER), FINGER_CLOSED, "lift push", i, "push_lift", **kw)
            add(side, 0.6, HOME[side], FINGER_CLOSED, "back home", i, "transit")

        # Other arm waits at home during this operation
        d = (sum(w.duration for w in seqs[side]) - t0) / time_scale
        add(other, d, HOME[other], FINGER_CLOSED, "wait", i, "wait")

    # Final hold with open grippers for scoring
    for s in ("l", "r"):
        add(s, 2.5, HOME[s], FINGER_OPEN, "final hold", len(ops), "hold")

    start_pair = (START_MM["l"], START_MM["r"])
    for label, m in meta.items():
        k = m["op"]
        m["before"] = after[k - 1] if k > 0 else start_pair
        m["after"] = after[k] if k < len(ops) else (after[-1] if ops else start_pair)

    out = {"left": seqs["l"], "right": seqs["r"]}
    return (out, meta) if with_meta else out


def duration_steps(seqs: dict, hz: float = 20.0) -> int:
    """Return total steps for waypoint sequences."""
    return int(round(max(sum(w.duration for w in v) for v in seqs.values()) * hz))
