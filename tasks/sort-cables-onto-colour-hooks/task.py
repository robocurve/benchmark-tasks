"""On the station table stands a grey cable rack: an upright board (44 cm wide, 15 cm tall, 12 mm thick, front face
27.5 cm in front of the robot) running left-right, with three short hooks sticking out of its front face towards the robot,
18 cm apart: a red hook on the left (y +18 cm), a green hook in the middle (y 0) and a blue hook on the right
(y -18 cm). Each hook is a 60 x 20 x 10 mm peg (top 11 cm above the table) with a 15 mm upturned lip at its tip,
both in the hook's colour, and a matching colour tag is fixed to the board just above it. Three loose black 6 mm
cables, 28 cm long, lie straight on the table, running left-right, with their free (bare) ends towards the outside
and a coloured plug (35 x 16 x 12 mm) on their inner ends: a blue-plug cable and a red-plug cable on the left half
of the table (the blue plug farther from the robot and farther left than the red plug) and a green-plug cable on the
right half. Each cable must be hung on the hook of its plug's colour - draped over the peg so that it hangs down on
both sides of it - and then let go. No plug lies in its hook's left-to-right place or nearest to its own hook; the
blue cable lies out of the right arm's reach and must be passed across (the left arm sets it down on the midline).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Sort the cables onto the rack: hang each cable on the hook that has the same colour as the plug on "
               "its end, draping it over the hook so that it hangs down on both sides of the hook. Then let go of "
               "the cables.")

# -- rack: board + three coloured hooks (pegs along x with a lip at the tip) -----------------------------------------
RED = (0.85, 0.08, 0.06)
GREEN = (0.10, 0.65, 0.20)
BLUE = (0.10, 0.30, 0.90)
GREY = (0.60, 0.62, 0.64)
COLORS = {"red": RED, "green": GREEN, "blue": BLUE}

BOARD_X0, BOARD_T, BOARD_W, BOARD_H = 0.275, 0.012, 0.44, 0.15  # board front face at x = BOARD_X0
HOOK_Z = TOP + 0.11  # top face of every peg (the hook line)
PEG_L, PEG_W, PEG_T = 0.060, 0.020, 0.010  # along x (out of the board), across (y), thickness
PEG_X0, PEG_X1 = BOARD_X0 - PEG_L, BOARD_X0  # peg spans x 21.5 - 27.5 cm
LIP_T, LIP_H = 0.006, 0.015  # lip at the peg tip: 6 mm thick, rises 15 mm above the peg top
HOOK_Y = {"red": 0.18, "green": 0.0, "blue": -0.18}


def _hook(c: str) -> list[Box]:
    y, col = HOOK_Y[c], COLORS[c]
    return [
        Box(f"hook_{c}", pos=((PEG_X0 + PEG_X1) / 2, y, HOOK_Z - PEG_T / 2), size=(PEG_L, PEG_W, PEG_T), color=col),
        Box(f"hook_lip_{c}", pos=(PEG_X0 + LIP_T / 2, y, HOOK_Z - PEG_T + (PEG_T + LIP_H) / 2),
            size=(LIP_T, PEG_W, PEG_T + LIP_H), color=col),
        Box(f"tag_{c}", pos=(BOARD_X0 - 0.0015, y, HOOK_Z + 0.025), size=(0.003, 0.030, 0.025), color=col),
    ]


BOARD = Box("rack_board", pos=(BOARD_X0 + BOARD_T / 2, 0.0, TOP + BOARD_H / 2), size=(BOARD_T, BOARD_W, BOARD_H),
            color=GREY)
RACK = [BOARD, *(p for c in HOOK_Y for p in _hook(c))]

# -- three free cables, bare end outside, coloured plug on the inner end -------------------------------------------
R = 0.003
CORD_LEN = 0.28
PLUG_SIZE = (0.035, 0.016, 0.012)
PLUG_MASS = 0.008
Z0 = TOP + PLUG_SIZE[2] / 2 + 0.0005


def _cord(c: str, x: float, y_bare: float, sign: int):
    """Cable along y at x; bare end at y_bare, running towards the midline (sign = -1: towards -y)."""
    return cable(f"cable_{c}", start=(x, y_bare, Z0), yaw=sign * math.pi / 2, length=CORD_LEN, radius=R,
                 end_plug=Plug(size=PLUG_SIZE, mass=PLUG_MASS, color=COLORS[c]), color=(0.05, 0.05, 0.05),
                 bend_stiffness=0.01, bend_damping=0.001)


CABLES = {
    "blue": _cord("blue", 0.165, 0.42, -1),  # plug from y 14.0 to 10.5 cm (far row, left; nearest hook: red)
    "red": _cord("red", 0.11, 0.34, -1),  # plug from y 6.0 to 2.5 cm (near row, left; nearest hook: green)
    "green": _cord("green", 0.14, -0.42, +1),  # plug from y -14.0 to -10.5 cm (right; nearest hook: blue)
}
OBJECTS = [*RACK, *CABLES.values()]

# -- scoring ---------------------------------------------------------------------------------------------------------
HANG = 0.04  # the cable must hang at least this far below the hook line on both sides of the peg
ON_TOL = 0.004  # lateral tolerance for "lying on the peg"
RELEASE_DIST = 0.03


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _line(cs) -> np.ndarray:
    """Cable centre line from the bare end through the plug centre, densely sampled."""
    poly = cs.polyline()
    if cs.end_plug is not None:
        poly = np.vstack([poly, cs.end_plug])
    return _dense(poly)


def on_hook(cs, hook: str) -> bool:
    """The cable is draped over the hook: part of it rests on the peg top (over the peg, between the lip and the
    board, at most 2 cm above the peg top), and on BOTH sides of the peg (left and right of it) the cable continues
    down to at least 4 cm below the hook line, one side on each side of that resting part."""
    pts = _line(cs)
    y = HOOK_Y[hook]
    on = ((pts[:, 0] >= PEG_X0) & (pts[:, 0] <= PEG_X1) & (np.abs(pts[:, 1] - y) <= PEG_W / 2 + ON_TOL)
          & (pts[:, 2] >= HOOK_Z) & (pts[:, 2] <= HOOK_Z + 0.02))
    if not on.any():
        return False
    idx = np.flatnonzero(on)
    i0, i1 = idx.min(), idx.max()
    low = pts[:, 2] <= HOOK_Z - HANG
    out = np.abs(pts[:, 1] - y) > PEG_W / 2
    side = np.sign(pts[:, 1] - y)
    before = low[:i0] & out[:i0]
    after = low[i1 + 1:] & out[i1 + 1:]
    if not (before.any() and after.any()):
        return False
    sb, sa = set(side[:i0][before]), set(side[i1 + 1:][after])
    return any(-s in sa for s in sb)  # the two hanging sides are on opposite sides of the peg


def hooks_of(objects, c: str) -> list[str]:
    return [h for h in HOOK_Y if on_hook(objects[f"cable_{c}"], h)]


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_sort_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._sort_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads), [] if unknown."""
    env = _ENV["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        ax = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + ax * 0.134)
    return out


def released(objects) -> bool:
    """Both grippers at least 3 cm from every cable and plug."""
    pts = np.vstack([_line(objects[f"cable_{c}"]) for c in CABLES])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def tally(objects) -> tuple[int, int]:
    """(cables hung on their own hook only, cables hung on any wrong hook)."""
    right = wrong = 0
    for c in CABLES:
        hs = hooks_of(objects, c)
        if hs == [c]:
            right += 1
        elif hs:
            wrong += 1
    return right, wrong


def score(objects) -> float:
    """0 unless both grippers have let go. Otherwise (correct - wrong) / 3, floored at 0: a cable hung only on the
    hook of its plug's colour counts +1, a cable hung on any other hook (also on two hooks at once) counts -1, a cable
    not hung counts 0."""
    if not released(objects):
        return 0.0
    right, wrong = tally(objects)
    return max(0.0, (right - wrong) / 3.0)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9 and all(objects[f"cable_{c}"].resting(max_speed=0.02) for c in CABLES)


def distance(objects) -> float:
    """Number of cables not (only) on their own hook, plus 1 if a gripper still holds a cable."""
    right, _ = tally(objects)
    return float(len(CABLES) - right) + (0.0 if released(objects) else 1.0)


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------

GRASP_OPEN = -0.030  # finger opening for plug grasps (FINGER_OPEN is -0.047)
DRAPE_X = (PEG_X0 + LIP_T + PEG_X1) / 2  # plug carried over the middle of the peg (between lip and board)
OFF = 0.05  # plug centre this far past the peg centre when lowered on the far side
OFF_MID = 0.06  # middle hook: the hand reaching across the midline needs more room beside the peg
CARRY_DZ = 0.030  # plug centre above the hook line while crossing over the peg
DROP_DZ = -0.070  # plug centre relative to the hook line when let go (low: the plug side must outweigh the other)
DROP_DZ_MID = -0.035


def _log(ctx, tag):
    st = ctx.states()
    parts = []
    for c in CABLES:
        cs = st[f"cable_{c}"]
        parts.append(f"{c}:{hooks_of(st, c)} plug={np.round(cs.end_plug, 3)}")
    print(f"[demo t={ctx.t:5.1f}] {tag}: {' | '.join(parts)} rel={released(st)} score={score(st):.2f}", flush=True)


def _dbg(ctx, arm, c, tag):
    if os.environ.get("SORT_DEBUG"):
        st = ctx.states()[f"cable_{c}"]
        print(f"[dbg t={ctx.t:5.1f}] {c} {tag}: tcp_cmd={np.round(arm.tcp(), 3)} plug={np.round(st.end_plug, 3)} "
              f"grippers={[np.round(g, 3) for g in grippers()]}", flush=True)


def _hang(ctx, arm, c: str, hook: str, f_open, f_closed, *, let_go: bool = True):
    """Pick cable c by its plug and drape it over `hook`: lift, come to the hook from the cable's own side (the bare
    end side), carry the plug over the peg to the other side and lower it, so the hanging cable catches on the peg."""
    cs = ctx.states()[f"cable_{c}"]
    p = cs.end_plug
    s = 1.0 if cs.start_tip[1] > p[1] else -1.0  # side of the plug where the rest of the cable lies (+y / -y)
    yaw = -math.pi / 2 if arm is ctx.left else math.pi / 2  # fingers close along x, across the plug (along y)
    hi = TOP + 0.06
    f_open = max(f_open, GRASP_OPEN)  # open the fingers only part way: a smaller hand footprint near the rack
    # come in from the side, in front of the rack (a straight path from the ready pose would sweep over the board)
    yield from arm.move((0.13, math.copysign(0.22, p[1] if abs(p[1]) > 0.05 else (1 if arm is ctx.left else -1)),
                         TOP + 0.16), yaw=yaw, finger=f_open, duration=1.2)
    yield from arm.move((p[0], p[1], hi), yaw=yaw, finger=f_open, duration=1.2)
    p = ctx.states()[f"cable_{c}"].end_plug  # closed loop: the plug may have shifted
    yield from arm.move((p[0], p[1], p[2] + 0.010), yaw=yaw, duration=0.9)
    yield from arm.grip(f_closed, 0.6)
    _dbg(ctx, arm, c, "gripped")
    yh = HOOK_Y[hook]
    carry = HOOK_Z + CARRY_DZ + 0.010  # TCP height while crossing the peg
    yield from arm.move((p[0], p[1], carry + 0.04), yaw=yaw, duration=1.5)
    _dbg(ctx, arm, c, "lifted")
    off = OFF if abs(yh) > 0.05 else OFF_MID
    ya, yb = yh + s * off, yh - s * off  # cable's own side of the hook, far side
    yield from arm.move((DRAPE_X, ya, carry + 0.03), yaw=yaw, duration=1.8)
    yield from arm.move((DRAPE_X, ya, carry), yaw=yaw, duration=0.6)
    yield from arm.wait(0.3)
    yield from arm.move((DRAPE_X, yb, carry), yaw=yaw, duration=1.6)
    # let go low, so the plug side outweighs the other side and the cable cannot slide back over the peg; only the
    # left arm at the middle hook lets go higher (there a low left hand traps the cable)
    drop = DROP_DZ_MID if (abs(yh) <= 0.05 and arm is ctx.left) else DROP_DZ
    yield from arm.move((DRAPE_X, yb, HOOK_Z + drop + 0.010), yaw=yaw, duration=1.0)
    _log(ctx, f"{c} lowered past hook {hook}")
    _dbg(ctx, arm, c, "lowered")
    if not let_go:
        return
    yield from arm.grip(f_open, 0.5)
    # back straight out towards the robot, clear of the hook lips, then up
    yield from arm.move((DRAPE_X - 0.12, yb, HOOK_Z + drop + 0.02), yaw=yaw, duration=1.0)
    yield from arm.move((DRAPE_X - 0.12, yb, TOP + 0.16), yaw=yaw, duration=0.6)


RELAY = (0.13, -0.005)  # midline spot where the left arm sets the blue plug down for the right arm


def _relay(ctx, arm, c: str, f_open, f_closed):
    """Pick cable c by its plug and set the plug down flat on the table at RELAY (reachable by both arms)."""
    cs = ctx.states()[f"cable_{c}"]
    p = cs.end_plug
    yaw = -math.pi / 2 if arm is ctx.left else math.pi / 2
    f_open = max(f_open, GRASP_OPEN)
    hi = TOP + 0.06
    yield from arm.move((0.13, 0.22 if arm is ctx.left else -0.22, TOP + 0.16), yaw=yaw, finger=f_open, duration=1.2)
    yield from arm.move((p[0], p[1], hi), yaw=yaw, finger=f_open, duration=1.2)
    p = ctx.states()[f"cable_{c}"].end_plug
    yield from arm.move((p[0], p[1], p[2] + 0.010), yaw=yaw, duration=0.9)
    yield from arm.grip(f_closed, 0.6)
    yield from arm.move((p[0], p[1], TOP + 0.08), yaw=yaw, duration=0.8)
    yield from arm.move((RELAY[0], RELAY[1], TOP + 0.08), yaw=yaw, duration=1.4)
    yield from arm.move((RELAY[0], RELAY[1], p[2] + 0.012), yaw=yaw, duration=0.8)
    yield from arm.grip(f_open, 0.5)
    yield from arm.move((RELAY[0], RELAY[1], TOP + 0.10), yaw=yaw, duration=0.6)
    _log(ctx, f"{c} relayed")


def demo_script(ctx):
    """SORT_DEMO = correct (default) | nothing | order (hang the cables on the hooks in their left-to-right order on
    the table, ignoring colour: blue on red, red on green, green on blue) | hold (correct, but keeps holding the last
    plug) | nearest_one (hang only the right-most cable on the right-most hook, the one nearest to its plug).

    Correct: the right arm hangs green on the green (middle) hook, then the left arm hangs red on the red hook; the
    left arm sets the blue plug down on the midline and the right arm takes it and hangs blue on the blue hook."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    mode = os.environ.get("SORT_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    fo, fc = FINGER_OPEN, FINGER_CLOSED
    park_l = (0.13, 0.25, TOP + 0.16)
    park_r = (0.13, -0.25, TOP + 0.16)
    _log(ctx, "start")
    if mode == "nothing":
        yield from L.wait(4.0)
        _log(ctx, "end")
        return
    if mode == "nearest_one":
        yield from _hang(ctx, Rt, "green", "blue", fo, fc)
        yield from Rt.move(park_r, yaw=0.0, duration=1.0)
        _log(ctx, "parked")
        yield from Rt.wait(2.0)
        _log(ctx, "end")
        return
    if mode == "order":
        yield from ctx.both(_hang(ctx, L, "blue", "red", fo, fc), _hang(ctx, Rt, "green", "blue", fo, fc))
        yield from Rt.move(park_r, yaw=0.0, duration=1.0)
        _log(ctx, "first two hung")
        yield from _hang(ctx, L, "red", "green", fo, fc)
        yield from L.move(park_l, yaw=0.0, duration=1.2)
        _log(ctx, "parked")
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    # the right arm reaches over to the middle hook, so the two arms take turns there
    yield from _hang(ctx, Rt, "green", "green", fo, fc)
    yield from ctx.both(_hang(ctx, L, "red", "red", fo, fc), Rt.move(park_r, yaw=0.0, duration=1.0))
    yield from _relay(ctx, L, "blue", fo, fc)
    _log(ctx, "first two hung, blue relayed")
    yield from L.move(park_l, yaw=0.0, duration=1.0)
    yield from _hang(ctx, Rt, "blue", "blue", fo, fc, let_go=mode != "hold")
    if mode == "hold":
        yield from Rt.wait(4.0)
        _log(ctx, "end (held)")
        return
    yield from Rt.move(park_r, yaw=0.0, duration=1.2)
    _log(ctx, "parked")
    yield from Rt.wait(2.0)
    _log(ctx, "end")
