"""Scripted bimanual demo and ablations for coupled knob peak tuning. Pure Python + NumPy.

A plan is a list of OPS executed one after another (the other arm waits at home):
  ("read", side)                       press READ with the left ("l") or right ("r") arm
  ("push", side, s_from, s_to)         push the left ("l") or right ("r") knob from setting s_from to s_to (mm)
Forward pushes (s_to > s_from) put the closed fingers BEHIND the knob; backward pushes put them IN FRONT (possible for
s_from <= BACKWARD_MAX_S_MM only, because the wedge-shaped fingers need room before the far end stop).

Modes (selected by environment variables, see run.sh):
  demo         5 readings along the hidden ridge toward the peak of the selected variant (TASK_VARIANT, default 0)
  recover      as demo, but the first left push overshoots by 6 mm and is pulled back (needs the backward push)
  axis_search  scripted single-axis search (oracle line searches, 8 readings, knobs left where the 8th was taken)
  no_read      moves both knobs straight to the variant-0 peak without any reading
  memorize     hard-coded variant-0 peak with 3 readings (works on variant 0 only; run it with TASK_VARIANT=1 or 2)
  settle       holds at home (scores 0.0)
Knobs for GPU tuning: DEMO_PUSH_OVERSHOOT_MM, DEMO_PRESS_OVERSHOOT_MM, DEMO_PUSH_SPEED_MM_S, DEMO_YAW_SHIFT_LEFT/RIGHT.
"""

from __future__ import annotations

import os

import geometry as g
import logic
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint

PAD_TO_TCP = g.PAD_TO_TCP
Z_HOVER = 0.840
HOME = {"l": (0.200, 0.160, Z_HOVER), "r": (0.200, -0.160, Z_HOVER)}
TRACK_Y = {"l": g.TRACK_Y["left"], "r": g.TRACK_Y["right"]}
# Backward pushes put the finger between the knob and the far stop. At the knob's top edge the finger is 11.35 mm
# deep each side, at the wall-top height 12.6 mm: front face + approach + 2 x ~12 mm + 2 mm must fit before x = 0.305.
BACKWARD_MAX_S_MM = 70.0
OVERSHOOT_RECOVER = 6.0  # mm, the deliberate overshoot of the recover mode
START_MM = (g.KNOB_START_SETTING_MM, g.KNOB_START_SETTING_MM)

MODE_FLAGS = ("DEMO_SETTLE_ONLY", "DEMO_NO_READ", "DEMO_AXIS_SEARCH", "DEMO_MEMORIZE", "DEMO_RECOVER")


def _b(name: str, default: bool = False) -> bool:
    return os.environ.get(name, "1" if default else "0") in ("1", "true", "True")


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    for flag, mode in (("DEMO_SETTLE_ONLY", "settle"), ("DEMO_NO_READ", "no_read"), ("DEMO_AXIS_SEARCH", "axis_search"),
                       ("DEMO_MEMORIZE", "memorize"), ("DEMO_RECOVER", "recover")):
        if _b(flag):
            return mode
    return "demo"


def _moves(points: list[tuple[float, float]], reads: list[bool]) -> list[tuple]:
    """Ops that visit consecutive points (left push when x changes, then right push when y changes), optional read."""
    ops: list[tuple] = []
    reader = 0
    if reads[0]:
        ops.append(("read", "r"))
        reader += 1
    for k in range(1, len(points)):
        (x0, y0), (x1, y1) = points[k - 1], points[k]
        if abs(x1 - x0) > 1e-9:
            ops.append(("push", "l", x0, x1))
        if abs(y1 - y0) > 1e-9:
            ops.append(("push", "r", y0, y1))
        if reads[k]:
            ops.append(("read", "r" if reader % 2 == 0 else "l"))
            reader += 1
    return ops


def build_ops(mode: str, variant: int | None = None) -> list[tuple]:
    v = logic.VARIANT if variant is None else variant
    if mode == "settle":
        return []
    if mode == "demo":
        pts = logic.ridge_path(v)
        return _moves(pts, [True] * len(pts))
    if mode == "recover":
        pts = logic.ridge_path(v)
        ops = _moves(pts, [True] * len(pts))
        # first left push (30 -> x1) overshoots by 6 mm, a reading shows it, then the knob is pulled back
        i = next(k for k, o in enumerate(ops) if o[0] == "push")
        _, side, a, b = ops[i]
        assert b + OVERSHOOT_RECOVER <= BACKWARD_MAX_S_MM, "overshoot too far to recover with the wedge fingers"
        ops[i : i + 1] = [("push", side, a, b + OVERSHOOT_RECOVER), ("read", "l"), ("push", side, b + OVERSHOOT_RECOVER, b)]
        return ops
    if mode == "axis_search":
        path = logic.coordinate_search_path(v, readings=8)
        pts = [(round(x, 1), round(y, 1)) for x, y, _r in path]
        return _moves(pts[:8], [True] * 8)  # 8 readings (start + 7 moves) and the knobs stay where the 8th was taken
    if mode == "no_read":
        x0, y0 = logic.VARIANTS[0]
        return [("push", "l", START_MM[0], x0), ("push", "r", START_MM[1], y0)]
    if mode == "memorize":
        x0, y0 = logic.VARIANTS[0]
        pts = [START_MM, (55.0, 40.0), (x0, y0)]
        return _moves(pts, [True] * 3)
    raise ValueError(f"unknown mode {mode}")


def knob_state_after(ops: list[tuple]) -> list[tuple[float, float]]:
    """Nominal (left, right) knob settings after each op."""
    sl, sr = START_MM
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
    """TCP z that puts the pad PRESS_OVERSHOOT below the end stop (cap top at full travel)."""
    over = _f("DEMO_PRESS_OVERSHOOT_MM", g.BUTTON_PRESS_OVERSHOOT * 1000) / 1000
    return g.BUTTON_CAP_TOP_REST_Z - g.BUTTON_TRAVEL - over + PAD_TO_TCP


# GPU run 1: the 1.45 mm model value left every push 3-7 mm short (the left knob stalled at 70 mm); 8 mm reached 1.0.
GPU_PUSH_OVERSHOOT_MM = 8.0


def push_targets(s_from: float, s_to: float) -> tuple[float, float]:
    """TCP x at the start and at the end of a push (m)."""
    w = g.push_contact_half_depth()
    over = _f("DEMO_PUSH_OVERSHOOT_MM", GPU_PUSH_OVERSHOOT_MM) / 1000
    if s_to >= s_from:  # fingers behind the knob, contact at the rear top edge
        rear = lambda s: g.CHANNEL_NEAR_STOP_X + s / 1000.0
        return rear(s_from) - w - g.PUSH_APPROACH_CLEAR, rear(s_to) - w + over
    front = lambda s: g.CHANNEL_NEAR_STOP_X + s / 1000.0 + g.KNOB_L
    return front(s_from) + w + g.PUSH_APPROACH_CLEAR, front(s_to) + w - over


def plans(mode: str | None = None, variant: int | None = None, with_meta: bool = False):
    """Waypoint plans for both arms (and, with ``with_meta``, a label -> info dict for the checks)."""
    mode = mode or mode_from_env()
    yaw = {"l": _f("DEMO_YAW_SHIFT_LEFT", 0.0), "r": _f("DEMO_YAW_SHIFT_RIGHT", 0.0)}
    speed = _f("DEMO_PUSH_SPEED_MM_S", 40.0)
    seqs: dict[str, list[Waypoint]] = {"l": [], "r": []}
    meta: dict[str, dict] = {}

    def add(side, dur, pos, finger, label, op, kind, **extra):
        full = f"{side}: {label} #{op:02d}"
        seqs[side].append(Waypoint(dur, pos, finger, full, yaw[side]))
        meta[full] = dict(op=op, kind=kind, side=side, **extra)

    if mode == "settle":
        hold = _f("DEMO_SETTLE_HOLD_S", 8.0)
        for s in ("l", "r"):
            add(s, hold, HOME[s], FINGER_OPEN, "settle hold", 0, "hold")
        out = {"left": seqs["l"], "right": seqs["r"]}
        return (out, meta) if with_meta else out

    ops = build_ops(mode, variant)
    after = knob_state_after(ops)
    z_push, z_press = _z_push(), _z_press()
    btn = (g.BUTTON_BASE_CX, g.BUTTON_BASE_CY)
    for i, o in enumerate(ops):
        side = o[1]
        other = "r" if side == "l" else "l"
        t0 = sum(w.duration for w in seqs[side])
        if o[0] == "read":
            add(side, 0.6, (*btn, Z_HOVER), FINGER_CLOSED, "hover btn", i, "btn_hover")
            add(side, 0.5, (*btn, z_press), FINGER_CLOSED, "press btn", i, "btn_press")
            add(side, 0.4, (*btn, z_press), FINGER_CLOSED, "hold btn", i, "btn_press")
            add(side, 0.5, (*btn, Z_HOVER), FINGER_CLOSED, "release btn", i, "btn_hover")
            add(side, 0.6, HOME[side], FINGER_CLOSED, "back home", i, "transit")
        else:
            _, _, s_from, s_to = o
            x_from, x_to = push_targets(s_from, s_to)
            ty = TRACK_Y[side]
            dur = 0.6 + abs(s_to - s_from) / speed
            kw = dict(pushed=side, s_from=s_from, s_to=s_to)
            add(side, 0.6, (x_from, ty, Z_HOVER), FINGER_CLOSED, "hover push", i, "push_hover", **kw)
            add(side, 0.5, (x_from, ty, z_push), FINGER_CLOSED, "descend push", i, "push_descend", **kw)
            add(side, dur, (x_to, ty, z_push), FINGER_CLOSED, "push to", i, "push_contact", **kw)
            add(side, 0.5, (x_to, ty, Z_HOVER), FINGER_CLOSED, "lift push", i, "push_lift", **kw)
            add(side, 0.6, HOME[side], FINGER_CLOSED, "back home", i, "transit")
        d = sum(w.duration for w in seqs[side]) - t0
        add(other, d, HOME[other], FINGER_CLOSED, "wait", i, "wait")
    for s in ("l", "r"):
        add(s, 2.5, HOME[s], FINGER_OPEN, "final hold", len(ops), "hold")
    for label, m in meta.items():
        k = m["op"]
        m["before"] = after[k - 1] if k > 0 else START_MM
        m["after"] = after[k] if k < len(ops) else (after[-1] if ops else START_MM)
    out = {"left": seqs["l"], "right": seqs["r"]}
    return (out, meta) if with_meta else out


def duration_steps(seqs: dict, hz: float = 20.0) -> int:
    return int(round(max(sum(w.duration for w in v) for v in seqs.values()) * hz))
