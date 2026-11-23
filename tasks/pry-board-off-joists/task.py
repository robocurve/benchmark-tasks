"""On the station table, at real size: a 1x4 pine board nailed across two 2x4 joists, and a flat pry bar.

The frame (fixed), a piece of floor or deck framing: two 2x4 SPF joists (38 x 89 mm actual, 612 mm long) standing on
edge, running away from the robot from 25 to 86 cm out, 16 in (406 mm) apart on centre either side of the middle, and
a 2x4 rim joist across their back ends, held to the table's back edge by two 6 in C-clamps.

The board: a 600 mm length of 1x4 common pine (19 x 89 mm actual, 0.46 kg) lying flat across the joists' tops,
centred between them (its ends overhang each joist by 78 mm), held by one 3d box nail (1-1/4 in, bright steel) into
each joist, driven square through its centre line, head flush.

The pry bar: a Vaughan B215 Original SuperBar (part VN45001), 381 mm and 24 oz (0.680 kg), blue spring steel
with a flared blade, a curved rocker head and three nail openings. Fine dimensions are approximated (README).
It lies on the table in front of the frame, parallel to the board, its claw to the robot's right, curled down.

Goal: the board pried off the joists with the bar and laid flat on the table; then both grippers clear of it. See
scoring.py. The usual way: slip the blade under the board beside a joist, rest the bar across that joist's top and
push the handle down; loosen one end, then the other, then lift the board off.

The nails (a rule standing in for wood's grip on a nail, see ``_nails``): the board is glued to a hidden carrier
inside the frame, which can slide straight up and tip end to end (about a line across the board, along x, on the
joists' tops), so one end can rise while the other is still nailed; its two springs are the two nails. Every control step the rule lets each nail slide out once the board pulls on it with more than its
withdrawal resistance (150 N freshly driven, falling in proportion to the length still in the joist), and holds it
wherever it was left. Once both nails have left the joists (12.7 mm out), the glue lets go and the board is loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score ...
Demo:    python demokit.py task.py <out_dir>          (see demokit.py)
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import layout as L  # noqa: E402
import mechkit as mk  # noqa: E402  (importing it hooks yam_arena so the assemblies are built as articulations)
import rigstate  # noqa: E402
import scoring as S  # noqa: E402

rigstate.install()

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = L.SUCCESS_HOLD_S

INSTRUCTION = "Pry the board off the joists with the pry bar and lay it flat on the table."

MM = L.MM


# -- the objects ----------------------------------------------------------------------------------------------------
FRAME_ORIGIN = np.array([L.BOARD_X, L.BOARD_Y, L.TOP])


def _frame_shapes():
    return [mk.box(tuple(np.asarray(c) - FRAME_ORIGIN), s, colour) for c, s, colour in L.frame_shapes()]


# The nails' carrier: small links inside the joists (they never show and meet nothing they could collide with), on a
# slider straight up and a hinge about x (across the board) through its middle, on the joists' tops: it tips end to end.
_BOX = (6 * MM, 6 * MM, 6 * MM)
LIFT = mk.slider("lift", "Z", [mk.box((0.05, L.JOIST_Y[0] - L.BOARD_Y, L.CARRIER_Z), _BOX, L.HIDDEN)],
                 lower=L.LIFT_RANGE[0], upper=L.LIFT_RANGE[1], mass=L.CARRIER_MASS, spring=L.LIFT_K,
                 damping=L.LIFT_DAMPING)
TIP = mk.hinge("tip", "X", (0.0, 0.0, L.JOIST_H),
               [mk.box((0.0, y - L.BOARD_Y, L.CARRIER_Z), _BOX, L.HIDDEN) for y in L.NAILS.values()],
               lower=L.TIP_RANGE[0], upper=L.TIP_RANGE[1], mass=L.CARRIER_MASS, spring=L.TIP_K,
               damping=L.TIP_DAMPING, parent="lift")
FRAME = mk.assembly("frame", tuple(FRAME_ORIGIN), _frame_shapes(), joints=(LIFT, TIP), color=L.SPF,
                    friction=(0.6, 0.5))


def _board_shapes():
    e = L.EASE  # the eased long edges, as 3 mm steps: a core full thickness, a slab full width
    out = [mk.box((0.0, 0.0, 0.0), (L.BOARD_W - 2 * e, L.BOARD_L, L.BOARD_T), L.PINE),
           mk.box((0.0, 0.0, 0.0), (L.BOARD_W, L.BOARD_L, L.BOARD_T - 2 * e), L.PINE)]
    for y in L.NAILS.values():  # the nails' heads, flush (drawn 0.2 mm proud so they show)
        out.append(mk.cylinder((0.0, y - L.BOARD_Y, L.BOARD_T / 2 + 0.1 * MM), L.NAIL_HEAD_D / 2, 0.2 * MM,
                               color=L.NAIL_HEAD))
    return out


BOARD = mk.assembly("board", L.board_start_centre(), _board_shapes(), mass=L.BOARD_MASS, color=L.PINE,
                    friction=L.BOARD_FRICTION)

_BAR_TILT, _BAR_Z = L.bar_rest()


def _bar_shapes():
    """The bar lying curl-down, its claw end raised by the tilt that rests it on the blade's edge and the claw."""
    c, s = math.cos(_BAR_TILT), math.sin(_BAR_TILT)
    out = []
    for (cx, cy, cz), size, pitch, colour in L.bar_shapes():
        # turned about the bar's width by -tilt (raises +x): x' = x c - z s, z' = x s + z c
        out.append(mk.box((cx * c - cz * s, cy, cx * s + cz * c), size, colour, ypr=(0.0, pitch - _BAR_TILT, 0.0)))
    return out


BAR = mk.assembly("pry_bar", (L.BAR_START_X, L.BAR_START_Y_TIP, L.TOP + _BAR_Z + 1.0 * MM), _bar_shapes(),
                  mass=L.BAR_MASS, yaw=L.BAR_YAW, color=L.BAR_STEEL, friction=L.BAR_FRICTION, tight_fit=True)

# The nails' grip: the board glued to the carrier where it was nailed (re-glued at every reset), let go by the rule
# once both nails have left the joists.
GLUE = mk.bond("nails", BOARD, (FRAME, "tip"))

OBJECTS = [FRAME, BOARD, BAR]


# -- the nails, every control step ----------------------------------------------------------------------------------
NAILS = S.Nails()
_EP = {"step": 0, "released": False}
_TRACE = os.environ.get("TASK_TRACE")


def _new_episode() -> None:
    NAILS.reset()
    _EP.update(step=0, released=False)


mk.on_reset(_new_episode)
rigstate.on_reset(_new_episode)


_R_BAR = np.array(L.bar_frame_rotation())


def bar_point_world(bar_state, name: str) -> np.ndarray:
    return np.asarray(bar_state.to_world(_R_BAR @ np.asarray(L.bar_point(name))), float)


def _bar_at(bar_state) -> dict:
    """Nail name -> the bar is levering at it now (scoring.bar_at_nail), from the bar's pose."""
    try:
        tip, heel = bar_point_world(bar_state, "tip"), bar_point_world(bar_state, "heel")
    except (AttributeError, TypeError, ValueError):
        return {n: False for n in S.NAIL_NAMES}
    return {n: S.bar_at_nail(tip, heel, n) for n in S.NAIL_NAMES}


@mk.on_step
def _nails(states) -> None:
    _EP["step"] += 1
    frame = states["frame"]
    lift, tip = float(frame.joint("lift")), float(frame.joint("tip"))
    bar_at = _bar_at(states.get("pry_bar"))
    was = dict(NAILS.out)
    lift_t, tip_t = NAILS.step(lift, tip, bar_at)
    mk.set_targets(FRAME, "lift", lift_t)
    mk.set_targets(FRAME, "tip", tip_t)
    for n in S.NAIL_NAMES:
        if NAILS.out[n] and not was[n]:
            print(f"[pry-board-off-joists] step {_EP['step']}: the {n} nail has left its joist", flush=True)
    if NAILS.both_out and not _EP["released"]:
        GLUE.release()
        _EP["released"] = True
        print(f"[pry-board-off-joists] step {_EP['step']}: both nails out: the board is loose", flush=True)
    if _TRACE:
        with open(_TRACE, "a") as f:
            f.write(json.dumps({"step": _EP["step"], "lift_mm": round(lift / MM, 2),
                                "tip_deg": round(math.degrees(tip), 3),
                                "slid_mm": {n: round(v / MM, 2) for n, v in NAILS.slid.items()},
                                "by_bar_mm": {n: round(v / MM, 2) for n, v in NAILS.slid_bar.items()},
                                "bar_at": bar_at, "out": dict(NAILS.out)}) + "\n")


# -- scoring ----------------------------------------------------------------------------------------------------------
def reading(objects) -> S.Reading:
    board = objects["board"]
    arms = rigstate.arms()
    released = rigstate.released(arms, board.distance, clearance=L.CLEAR)
    still = float(np.linalg.norm(board.lin_vel)) < L.STILL_V
    return S.judge(board.rotation(), board.origin, dict(NAILS.out), released, still, NAILS.by_bar())


def score(objects) -> float:
    """See scoring.py: 1.0 off the joists and laid flat on the table, let go; 0.75 off the joists; 0.5 both nails
    loosened 6 mm; 0.25 one; 0 otherwise."""
    return reading(objects).score


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- demo -------------------------------------------------------------------------------------------------------------
def demo_script(ctx):
    """The scripted demo (demo.py), run by demokit.py. TASK_DEMO=full (default): the whole task; right: the right
    nail only; idle: the arms stay at their ready pose (the setup screenshot); look: close-up passes of the wrist
    cameras (nothing touched)."""
    if ctx.mode in ("full", "right", "aim"):
        import demo

        yield from demo.demo_script(ctx)
        return
    if ctx.mode == "look":
        from demokit import level

        r, l = ctx.right, ctx.left
        yield from r.move((0.20, -0.12, L.TOP + 0.16), grip=level(30), t=2.5)
        for _ in range(30):
            yield
        yield from l.move((0.20, 0.12, L.TOP + 0.16), grip=level(-30), t=2.5)
        for _ in range(30):
            yield
        return
    for _ in range(40):
        yield
