"""On the station table, three grey anchor cups (open-top PLA boxes, inner 104 x 56 mm, 20 mm walls) are fixed in a row
36 cm in front of the robot, at 17 cm (anchor 1, left), 0 cm (anchor 2, centre) and -17 cm (anchor 3, right) from the
midline, each with a white card and a black numeral (1, 2, 3) just beyond it. Each cup is turned so that its slotted
wall (two 10 mm cord slots) faces an orange harness block fixed to the table front-right of centre. Two 6 mm lanyard
cords, red and blue (37.4 cm each), leave the harness block side by side and run straight, diagonally, into anchor 1;
each ends in a black hook block (30 x 14 x 12 mm) and both hook blocks lie in anchor 1. The robot must move both
hooks from anchor 1 to anchor 2 and on to anchor 3, one hook at a time, so that at every moment at least one hook is
in an anchor cup (100% tie-off with a twin-leg lanyard); then let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the cords below are simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Move both lanyard hooks from anchor 1 to anchor 2 and then to anchor 3, one hook at a time: at every "
               "moment at least one hook must be in an anchor cup. Finish with both hooks in anchor 3 and let go.")

# -- layout ---------------------------------------------------------------------------------------------------------
CUP_X = 0.36
CUP_YS = (0.17, 0.0, -0.17)  # anchors 1, 2, 3
HARNESS = np.array([0.08, -0.10])  # midpoint of the two cord exits on the harness block face
IN_L, IN_W = 0.056, 0.104  # cup inner size: along the cup axis (towards the harness) x across
WALL, CUP_H, FLOOR = 0.004, 0.020, 0.003
SLOT_W = 0.010
LEG_OFF = 0.020  # the two cords / hooks sit 20 mm either side of the cup axis (40 mm apart)
GREY = (0.55, 0.57, 0.60)
ORANGE = (0.95, 0.45, 0.08)
WHITE = (0.93, 0.93, 0.92)
BLACK = (0.04, 0.04, 0.04)


def _unit(a):
    a = np.asarray(a, dtype=float)
    return a / np.linalg.norm(a)


CUPS = [np.array([CUP_X, y]) for y in CUP_YS]
CUP_YAW = [math.atan2(*(c - HARNESS)[::-1]) for c in CUPS]  # cup axis points away from the harness


def _cup_boxes(k: int) -> list[Box]:
    c, yaw = CUPS[k], CUP_YAW[k]
    ax = np.array([math.cos(yaw), math.sin(yaw)])
    ac = np.array([-math.sin(yaw), math.cos(yaw)])
    ol, ow = IN_L + 2 * WALL, IN_W + 2 * WALL
    zc = TOP + CUP_H / 2
    tag = str(k + 1)

    def at(u, v):
        p = c + u * ax + v * ac
        return float(p[0]), float(p[1])

    out = [Box(f"cup{tag}_floor", pos=(*at(0, 0), TOP + FLOOR / 2), size=(ol, ow, FLOOR), yaw=yaw, color=GREY),
           Box(f"cup{tag}_far", pos=(*at(IN_L / 2 + WALL / 2, 0), zc), size=(WALL, ow, CUP_H), yaw=yaw, color=GREY)]
    for s, n in ((1, "l"), (-1, "r")):
        out.append(Box(f"cup{tag}_side_{n}", pos=(*at(0, s * (IN_W / 2 + WALL / 2)), zc), size=(IN_L, WALL, CUP_H),
                       yaw=yaw, color=GREY))
    # slotted wall facing the harness: slots (SLOT_W) centred at +-LEG_OFF, from the floor up
    a, b = LEG_OFF - SLOT_W / 2, LEG_OFF + SLOT_W / 2
    u = -IN_L / 2 - WALL / 2
    out.append(Box(f"cup{tag}_near_mid", pos=(*at(u, 0), zc), size=(WALL, 2 * a, CUP_H), yaw=yaw, color=GREY))
    w = IN_W / 2 + WALL - b
    for s, n in ((1, "l"), (-1, "r")):
        out.append(Box(f"cup{tag}_near_{n}", pos=(*at(u, s * (b + w / 2)), zc), size=(WALL, w, CUP_H), yaw=yaw,
                       color=GREY))
    return out


# -- numeral cards beyond the cups (5x7 block digits from thin boxes; tops point away from the robot) ----------------
GLYPHS = {"1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
          "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
          "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"]}
PX = 0.004
CARD_X = 0.455
CARD = (0.040, 0.032, 0.002)


def _digit(k: int) -> list[Box]:
    ch, cy = str(k + 1), CUPS[k][1]
    out = [Box(f"card{ch}", pos=(CARD_X, float(cy), TOP + CARD[2] / 2), size=CARD, color=WHITE)]
    z = TOP + CARD[2]
    for r, row in enumerate(GLYPHS[ch]):
        c = 0
        while c < 5:
            if row[c] == "1":
                c0 = c
                while c < 5 and row[c] == "1":
                    c += 1
                x = CARD_X + 3.5 * PX - (r + 0.5) * PX
                y = cy + 2.5 * PX - (c0 + (c - c0) / 2) * PX
                out.append(Box(f"num{ch}_{r}_{c0}", pos=(x, float(y), z + 0.0005), size=(PX, (c - c0) * PX, 0.001),
                               color=BLACK))
            else:
                c += 1
    return out


# -- harness block and the two lanyard legs --------------------------------------------------------------------------
R = 0.003
HOOK = Plug(size=(0.030, 0.014, 0.012), mass=0.015, color=BLACK)
LEG_YAW = CUP_YAW[0]
_AX1 = np.array([math.cos(LEG_YAW), math.sin(LEG_YAW)])
_AC1 = np.array([-math.sin(LEG_YAW), math.cos(LEG_YAW)])
LEG_Z = TOP + FLOOR + HOOK.size[2] / 2 + 0.0005
LEG_LEN = float(np.linalg.norm(CUPS[0] - HARNESS)) - HOOK.size[0] / 2
HB = (0.040, 0.080, 0.040)  # harness block: along the legs, across, height
_hb = HARNESS - _AX1 * (0.006 + HB[0] / 2)
HARNESS_BLOCK = Box("harness_block", pos=(float(_hb[0]), float(_hb[1]), TOP + HB[2] / 2), size=HB, yaw=LEG_YAW,
                    color=ORANGE)


def _leg(name: str, side: int, color) -> object:
    s = HARNESS + side * LEG_OFF * _AC1
    return cable(name, start=(float(s[0]), float(s[1]), LEG_Z), yaw=LEG_YAW, length=LEG_LEN, radius=R,
                 anchor_start=True, end_plug=HOOK, color=color, bend_stiffness=0.004, bend_damping=0.0008)


RED = _leg("red_leg", -1, (0.85, 0.08, 0.06))  # the leg on the anchor-2 side of anchor 1
BLUE = _leg("blue_leg", +1, (0.08, 0.25, 0.85))
LEGS = (RED.name, BLUE.name)

OBJECTS = [*(b for k in range(3) for b in _cup_boxes(k)), *(b for k in range(3) for b in _digit(k)), HARNESS_BLOCK,
           RED, BLUE]

TWIN_EXTRA = [
    {"name": "leg_anchors", "kind": "joint", "real": "cord ends knotted through two 8 mm holes in the harness block",
     "note": "each cord's first 2 cm segment is fixed in place, starting 6 mm in front of the block face"},
    {"name": "hook_blocks", "kind": "joint", "real": "3D-printed PLA block glued/crimped onto each cord end"},
]

# -- episode memory (reset at every episode start) and gripper positions ---------------------------------------------
_EP: dict = {"env": None, "broken": False, "broken_t": None, "visited2": {n: False for n in LEGS}, "t": 0}


def _reset_episode() -> None:
    _EP.update(broken=False, broken_t=None, t=0, visited2={n: False for n in LEGS})


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline use)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls.reset, "_task_reset_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._task_reset_hooks:
                fn()
            return base_reset(self, *a, **k)

        reset._task_reset_hooks = []
        cls.reset = reset
    if _reset_episode not in cls.reset._task_reset_hooks:
        cls.reset._task_reset_hooks.append(_reset_episode)
    if not getattr(cls._task_step, "_lanyard_hooked", False):
        base_step = cls._task_step

        def _task_step(self, *a, **k):
            _EP["env"] = self._env
            _EP["t"] += 1
            return base_step(self, *a, **k)

        _task_step._lanyard_hooked = True
        cls._task_step = _task_step


_install_hooks()


def grippers() -> list[np.ndarray]:
    """World TCP positions of both grippers (yam_arena.runtime.tcp_world, env 0); [] if unknown (offline)."""
    env = _EP["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- scoring --------------------------------------------------------------------------------------------------------
RELEASE_DIST = 0.03  # every gripper TCP at least 3 cm from both hook blocks at the end


def hook(objects, name: str) -> np.ndarray:
    p = objects[name].end_plug
    return np.full(3, np.nan) if p is None else np.asarray(p, dtype=float)


def cup_of(p) -> int:
    """Anchor number (1-3) whose cup holds the point (hook-block centre inside the inner walls, below the rim), else 0."""
    if p is None or not np.isfinite(p).all() or p[2] > TOP + CUP_H:
        return 0
    for k, (c, yaw) in enumerate(zip(CUPS, CUP_YAW)):
        d = p[:2] - c
        u = d[0] * math.cos(yaw) + d[1] * math.sin(yaw)
        v = -d[0] * math.sin(yaw) + d[1] * math.cos(yaw)
        if abs(u) <= IN_L / 2 and abs(v) <= IN_W / 2:
            return k + 1
    return 0


def where(objects) -> tuple[int, int]:
    return tuple(cup_of(hook(objects, n)) for n in LEGS)


_START = {n: c.start_points() for n, c in ((RED.name, RED), (BLUE.name, BLUE))}


def _track(objects) -> None:
    """Latch rule breaks and anchor-2 visits (idempotent per state, so calling it several times a step is harmless)."""
    if all(np.abs(objects[n].points - _START[n]).max() < 0.004 for n in LEGS) and all(
            cup_of(hook(objects, n)) == 1 for n in LEGS) and _EP["t"] <= 1:
        _reset_episode()  # untouched starting layout at the first step => new episode (offline fallback)
    w = where(objects)
    for n, k in zip(LEGS, w):
        if k == 2:
            _EP["visited2"][n] = True
    if max(w) == 0 and not _EP["broken"]:
        _EP["broken"] = True
        _EP["broken_t"] = _EP["t"]


def released(objects) -> bool:
    hooks = [hook(objects, n) for n in LEGS]
    return all(float(np.linalg.norm(h - g)) >= RELEASE_DIST for g in grippers() for h in hooks)


def level(objects) -> int:
    """How far both legs have moved: the lowest anchor number holding a hook (a hook counts in anchor 3 only if it was
    in anchor 2 before); 0 if a hook is in no cup."""
    out = 3
    for n, k in zip(LEGS, where(objects)):
        if k == 3 and not _EP["visited2"][n]:
            return 0
        out = min(out, k)
    return out


def score(objects) -> float:
    """0 if at any step neither hook block was in an anchor cup, if at the end a gripper is within 3 cm of a hook block,
    a hook is in no cup, or a hook is in anchor 3 without having been in anchor 2. Else 0.5 x (lowest anchor number
    holding a hook - 1): both hooks in anchor 2 or beyond = 0.5, both in anchor 3 = 1.0 (one hook alone = 0)."""
    _track(objects)
    if _EP["broken"] or not released(objects):
        return 0.0
    lv = level(objects)
    return 0.0 if lv < 1 else 0.5 * (lv - 1)


def success(objects) -> bool:
    return score(objects) >= 1.0 and all(objects[n].resting(max_speed=0.02) for n in LEGS)


def distance(objects) -> float:
    """Sum of hook-to-anchor-3 distances (m); 1 m once the tie-off rule was broken."""
    _track(objects)
    if _EP["broken"]:
        return 1.0
    return float(sum(np.linalg.norm(hook(objects, n)[:2] - CUPS[2]) for n in LEGS))


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    _track(objects)
    w = where(objects)
    return {"red_visited_2": float(_EP["visited2"][RED.name]), "blue_visited_2": float(_EP["visited2"][BLUE.name]),
            "red_in_3": float(w[0] == 3), "blue_in_3": float(w[1] == 3), "tie_off_kept": float(not _EP["broken"]),
            "released": float(released(objects))}


VARIATION = {
    "obj:harness_block": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},  # the legs are welded to it: keep fixed
    "cable:red_leg": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},  # legs spawn straight into anchor 1; their start is the
    "cable:blue_leg": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},  # harness exit, so cords cannot be jittered independently
}

# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
PINCH = -0.015  # finger joint: pads 30 mm apart around a 14 mm hook block
GRASP_UP = 0.010  # TCP above the hook centre when grasping
CARRY = TOP + 0.075
DROP = TOP + FLOOR + HOOK.size[2] + 0.004 + GRASP_UP - HOOK.size[2] / 2  # hook bottom 4 mm above the cup floor
PARK = {"left": (0.18, 0.26, TOP + 0.14), "right": (0.18, -0.26, TOP + 0.14)}


def _yaw_of(objects, name: str) -> float:
    d = objects[name].end_dir
    y = math.atan2(float(d[1]), float(d[0]))
    while y > math.pi / 2:
        y -= math.pi
    while y < -math.pi / 2:
        y += math.pi
    return y


def _slot(k: int, v: float) -> np.ndarray:
    """Point in anchor k (1-3) at across-offset v from its axis, centred along it."""
    c, yaw = CUPS[k - 1], CUP_YAW[k - 1]
    return c + v * np.array([-math.sin(yaw), math.cos(yaw)])


def demo_script(ctx):
    """The left arm reaches anchors 1 and 2, the right arm anchors 2 and 3 (neither reaches the far anchor), so each
    hook goes 1 -> 2 with the left arm and 2 -> 3 with the right arm. Red first: left lifts it from anchor 1 (blue
    stays in anchor 1), sets it in anchor 2 and backs off; right takes it on to anchor 3 (blue still in 1). Then blue
    the same way (red now in anchor 3). Grasps follow the measured hook blocks; fingers close across each block, with
    the wrist camera on the block's free end (looking ahead at the cup)."""
    from yam_arena.constants import FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("LANYARD_CHEAT", "")  # test only: "none" | "hold" | "one" | "parallel"

    def log(tag):
        st = ctx.states()
        print(f"[demo t={ctx.t:5.1f}] {tag}: where={where(st)} broken={_EP['broken']}@{_EP['broken_t']} "
              f"v2={_EP['visited2']} rel={released(st)} score={score(st):.2f} "
              f"hooks={np.round(hook(st, RED.name), 3).tolist()} {np.round(hook(st, BLUE.name), 3).tolist()}",
              flush=True)

    def take(arm, name):
        st = ctx.states()
        p, y = hook(st, name), _yaw_of(st, name)
        yield from arm.move((p[0], p[1], CARRY), yaw=y, finger=PINCH, duration=1.5)
        p = hook(ctx.states(), name)
        yield from arm.move((p[0], p[1], p[2] + 0.03), yaw=y, duration=0.8)
        p = hook(ctx.states(), name)
        yield from arm.move((p[0], p[1], p[2] + GRASP_UP), yaw=y, duration=0.8)
        yield from arm.grip(0.0)
        return p, y

    def lift(arm, p, y, toward):
        d = _unit(np.asarray(toward) - p[:2])
        q = p[:2] + 0.02 * d
        yield from arm.move((q[0], q[1], p[2] + GRASP_UP + 0.03), yaw=y, duration=0.8)
        yield from arm.move((q[0], q[1], CARRY), yaw=y, duration=0.8)

    def place(arm, target, y, park, hold=False):
        yield from arm.move((target[0], target[1], CARRY), yaw=y, duration=1.6)
        yield from arm.move((target[0], target[1], DROP + 0.02), yaw=y, duration=0.8)
        yield from arm.move((target[0], target[1], DROP), yaw=y, duration=0.6)
        yield from arm.grip(PINCH)
        if hold:
            yield from arm.grip(0.0)
            return
        yield from arm.wait(0.3)
        yield from arm.move((target[0], target[1], CARRY), yaw=y, duration=0.8)
        yield from arm.move(park, yaw=0.0, finger=FINGER_OPEN, duration=1.2)

    log("start")
    if cheat == "none":
        yield from L.wait(3.0)
        log("end")
        return

    def move_leg(name, to3_v, hold=False):
        p, y = yield from take(L, name)
        yield from lift(L, p, y, CUPS[1])
        log(f"{name} lifted from 1")
        yield from place(L, _slot(2, 0.0), y, PARK["left"])
        log(f"{name} in 2")
        p, y = yield from take(Rt, name)
        yield from lift(Rt, p, y, CUPS[2])
        log(f"{name} lifted from 2")
        yield from place(Rt, _slot(3, to3_v), y, PARK["right"], hold=hold)
        log(f"{name} in 3")

    if cheat == "parallel":
        # both arms work at once: left lifts blue out of anchor 1 while the right arm carries red to anchor 3
        p, y = yield from take(L, RED.name)
        yield from lift(L, p, y, CUPS[1])
        yield from place(L, _slot(2, 0.0), y, PARK["left"])
        log("red in 2")

        def right_red():
            p2, y2 = yield from take(Rt, RED.name)
            yield from lift(Rt, p2, y2, CUPS[2])
            yield from place(Rt, _slot(3, -LEG_OFF), y2, PARK["right"])

        def left_blue():
            yield from L.wait(1.0)
            p3, y3 = yield from take(L, BLUE.name)
            yield from lift(L, p3, y3, CUPS[1])
            yield from L.wait(2.0)
            yield from place(L, _slot(2, 0.0), y3, PARK["left"])

        yield from ctx.both(right_red(), left_blue())
        log("parallel done")
        p, y = yield from take(Rt, BLUE.name)
        yield from lift(Rt, p, y, CUPS[2])
        yield from place(Rt, _slot(3, LEG_OFF), y, PARK["right"])
    else:
        yield from move_leg(RED.name, -LEG_OFF)
        if cheat != "one":
            yield from move_leg(BLUE.name, LEG_OFF, hold=(cheat == "hold"))
    for _ in range(4):
        yield from L.wait(0.5)
        log("settle")
    log("end")
