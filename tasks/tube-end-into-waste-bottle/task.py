"""On the station table: a grey pump housing (100 x 80 x 60 mm box) stands in front of the left arm. Its outlet tube
(orange 6 mm silicone tube, 55 cm) leaves the pump's right side at table level and lies straight across the table
towards the right arm; a 30 x 10 x 10 mm stainless sinker weight is fixed on the tube's free end. Behind the tube
stands a row of four identical open-top square bottles (60 x 60 mm inside, 80 mm tall, off-white), each with a white
label card lying on the table in front of it, printed in black capitals. From the robot's left to its right the
labels read FEED, WASTE, PRODUCT, RINSE. The tube's sinker end must end up inside the bottle labelled WASTE, at
least 3 cm below its rim, with both grippers let go; the tube must not dip into any other bottle.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from boxfont import text_boxes, text_width  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("This tube is the pump's outlet. Put the weighted end of the tube into the bottle labelled WASTE so it "
               "rests at least 3 cm below the rim, then let go of the tube. Do not let the tube go into any other "
               "bottle.")

# -- bottles: four identical open-top square bottles in a row (thin fixed boxes) -------------------------------------
BOT_X = 0.36
BOT_YS = (0.255, 0.085, -0.085, -0.255)
LABELS = ("FEED", "WASTE", "PRODUCT", "RINSE")
WASTE_I = LABELS.index("WASTE")
BOT_IN, BOT_H, BOT_T, BOT_FLOOR = 0.060, 0.080, 0.0025, 0.002
RIM_Z = TOP + BOT_H
BOT_COLOR = (0.90, 0.89, 0.85)


def _bottle(tag: str, y: float) -> list[Box]:
    x, ix, t = BOT_X, BOT_IN, BOT_T
    o = ix + 2 * t
    zc = TOP + BOT_H / 2
    return [
        Box(f"{tag}_floor", pos=(x, y, TOP + BOT_FLOOR / 2), size=(ix, ix, BOT_FLOOR), color=BOT_COLOR),
        Box(f"{tag}_wall_far", pos=(x + ix / 2 + t / 2, y, zc), size=(t, o, BOT_H), color=BOT_COLOR),
        Box(f"{tag}_wall_near", pos=(x - ix / 2 - t / 2, y, zc), size=(t, o, BOT_H), color=BOT_COLOR),
        Box(f"{tag}_wall_l", pos=(x, y + ix / 2 + t / 2, zc), size=(ix, t, BOT_H), color=BOT_COLOR),
        Box(f"{tag}_wall_r", pos=(x, y - ix / 2 - t / 2, zc), size=(ix, t, BOT_H), color=BOT_COLOR),
    ]


# -- labels: white cards on the table in front of each bottle, black block capitals (5x7 font, 4.2 mm per dot) ---------
PX = 0.0042
CARD_MARGIN = 0.006
CARD_D = 7 * PX + 2 * CARD_MARGIN  # card depth along x
CARD_X = BOT_X - BOT_IN / 2 - BOT_T - 0.010 - CARD_D / 2


def _label(tag: str, word: str, y: float) -> list[Box]:
    w = text_width(word, PX) + 2 * CARD_MARGIN
    card = Box(f"{tag}_card", pos=(CARD_X, y, TOP + 0.0005), size=(CARD_D, w, 0.001), color=(0.98, 0.98, 0.98))
    return [card, *text_boxes(Box, f"{tag}_txt", word, (CARD_X, y), TOP + 0.001, PX)]


BOTTLES = [b for i, y in enumerate(BOT_YS) for b in _bottle(f"bottle{i}", y)]
LABEL_BOXES = [b for i, (y, w) in enumerate(zip(BOT_YS, LABELS)) for b in _label(f"label{i}", w, y)]

# -- pump and outlet tube --------------------------------------------------------------------------------------------
PUMP_SIZE = (0.10, 0.08, 0.06)
PUMP_XY = (0.18, 0.43)
PUMP = Box("pump", pos=(PUMP_XY[0], PUMP_XY[1], TOP + PUMP_SIZE[2] / 2), size=PUMP_SIZE, color=(0.42, 0.44, 0.47))
R = 0.003  # 6 mm tube; soft bending (0.002) so the sinker can pull the tube over a rim, like real silicone
TUBE_LEN = 0.55
SINKER = Plug(size=(0.030, 0.010, 0.010), mass=0.014, color=(0.62, 0.64, 0.66))
TUBE_Z = TOP + SINKER.size[2] / 2 + 0.0005  # axis height: the sinker rests on the table, the tube 2 mm above it
TUBE_START = (PUMP_XY[0], PUMP_XY[1] - PUMP_SIZE[1] / 2 - 0.006, TUBE_Z)
TUBE = cable("tube", start=TUBE_START, yaw=-math.pi / 2, length=TUBE_LEN, radius=R, anchor_start=True,
             end_plug=SINKER, color=(0.90, 0.48, 0.12), bend_stiffness=0.002,
             bend_damping=0.0005)

OBJECTS = [PUMP, TUBE, *BOTTLES, *LABEL_BOXES]

# -- scoring ---------------------------------------------------------------------------------------------------------
DEPTH_FULL = 0.03  # sinker centre at least this far below the rim for full credit
RELEASE_DIST = 0.03  # every gripper TCP at least this far from the tube and sinker


def _inside(p, i: int) -> bool:
    """Point ``p`` within bottle ``i``'s inner footprint (horizontal) and below its rim."""
    return bool(abs(p[0] - BOT_X) <= BOT_IN / 2 and abs(p[1] - BOT_YS[i]) <= BOT_IN / 2 and p[2] < RIM_Z)


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_tube_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._tube_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads, 13 mm above the pad tips), [] if unknown."""
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
    t = objects["tube"]
    pts = np.vstack([_dense(t.polyline())] + ([t.end_plug[None]] if t.end_plug is not None else []))
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def in_wrong_bottle(objects) -> bool:
    """Any part of the tube or the sinker inside a bottle other than WASTE (within its footprint, below its rim)."""
    t = objects["tube"]
    pts = list(_dense(t.polyline())) + ([t.end_plug] if t.end_plug is not None else [])
    return any(_inside(p, i) for p in pts for i in range(len(BOT_YS)) if i != WASTE_I)


def sinker_depth(objects) -> float:
    """Metres the sinker centre lies below the WASTE bottle's rim, or -1 if it is not inside that bottle."""
    s = objects["tube"].end_plug
    if s is None or not _inside(s, WASTE_I):
        return -1.0
    return float(RIM_Z - s[2])


def score(objects) -> float:
    """0 if a gripper is within 3 cm of the tube/sinker, if any part of the tube or sinker is inside another bottle,
    or if the sinker is not inside the WASTE bottle. Sinker inside WASTE: 1.0 at >= 3 cm below the rim, else 0.5."""
    if in_wrong_bottle(objects) or not released(objects):
        return 0.0
    d = sinker_depth(objects)
    if d < 0:
        return 0.0
    return 1.0 if d >= DEPTH_FULL else 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["tube"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Horizontal distance (m) from the sinker to the WASTE bottle's centre."""
    s = objects["tube"].end_plug
    return float(np.hypot(s[0] - BOT_X, s[1] - BOT_YS[WASTE_I]))


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------

def _yaw_along(d) -> float:
    """Grip yaw (in [-pi/2, pi/2]) whose fingers close across a body lying along direction ``d``."""
    h = math.atan2(d[1], d[0])
    while h > math.pi / 2:
        h -= math.pi
    while h < -math.pi / 2:
        h += math.pi
    return h


def demo_script(ctx):
    """Right arm picks the sinker (which lies on its side of the table) and sets it down on the midline; the left
    arm, which reaches the WASTE bottle, picks it up there, carries it above the WASTE bottle and lets it drop in.
    Closed loop: every grasp is aimed at the sinker's measured position and heading."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("TUBE_CHEAT", "")  # test only: "none" | "nearest" | "hold"
    hi = TOP + 0.07
    carry = TOP + 0.14  # sinker bottom ~2.5 cm above the rims while carried
    over = 0.008  # TCP above the sinker centre when grasping

    def log(tag):
        st = ctx.states()
        t = st["tube"]
        print(f"[demo t={ctx.t:5.1f}] {tag}: sinker={np.round(t.end_plug, 3)} depth={sinker_depth(st):.3f} "
              f"wrong={in_wrong_bottle(st)} rel={released(st)} score={score(st):.2f}", flush=True)

    def sinker():
        t = ctx.states()["tube"]
        return np.asarray(t.end_plug, dtype=float), _yaw_along(t.end_dir)

    log("start")
    if cheat == "none":
        yield from L.wait(3.0)
        log("end")
        return

    # 1) right arm takes the sinker
    p, yaw = sinker()
    yaw = abs(yaw) if abs(abs(yaw) - math.pi / 2) < 0.6 else yaw  # right arm: +pi/2 rather than -pi/2
    yield from Rt.move((p[0], p[1], hi), yaw=yaw, finger=FINGER_OPEN, duration=1.5)
    yield from Rt.move((p[0], p[1], p[2] + over), yaw=yaw, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED)
    yield from Rt.move((p[0], p[1], carry), yaw=yaw, duration=1.0)
    log("right holds sinker")

    if cheat == "nearest":  # drop it into the bottle nearest to where the tube end lay (PRODUCT)
        y = BOT_YS[2]
        yield from Rt.move((BOT_X, y, carry), yaw=yaw, duration=1.5)
        yield from Rt.move((BOT_X, y, RIM_Z + 0.025), yaw=yaw, duration=1.0)
        yield from Rt.grip(FINGER_OPEN)
        yield from Rt.move((BOT_X - 0.10, y, carry + 0.04), yaw=yaw, duration=1.2)
        yield from Rt.wait(2.0)
        log("end (PRODUCT)")
        return

    # 2) set it down on the midline in front of the labels, let go, move the right arm away
    hand = (0.24, 0.0)
    yield from Rt.move((hand[0], hand[1], carry), yaw=yaw, duration=1.5)
    yield from Rt.move((hand[0], hand[1], TOP + SINKER.size[2] / 2 + over + 0.002), yaw=yaw, duration=1.0)
    yield from Rt.grip(FINGER_OPEN)
    yield from Rt.move((hand[0], hand[1], hi), yaw=yaw, duration=0.6)
    yield from Rt.move((0.20, -0.25, TOP + 0.15), yaw=0.0, duration=1.2)
    yield from L.wait(0.5)
    log("handed over")

    # 3) left arm takes it and lowers it above the WASTE bottle
    p, yaw = sinker()
    yield from L.move((p[0], p[1], hi), yaw=yaw, finger=FINGER_OPEN, duration=1.5)
    yield from L.move((p[0], p[1], p[2] + over), yaw=yaw, duration=1.0)
    yield from L.grip(FINGER_CLOSED)
    yield from L.move((p[0], p[1], carry), yaw=yaw, duration=1.0)
    wy = BOT_YS[WASTE_I]
    yield from L.move((BOT_X, wy, carry), yaw=yaw, duration=1.8)
    drop = RIM_Z + 0.025  # TCP: sinker bottom ~1 cm above the rim
    yield from L.move((BOT_X, wy, drop), yaw=yaw, duration=1.0)
    yield from L.wait(0.3)
    log("over WASTE")
    if cheat == "hold":  # keep holding it at the bottle mouth
        yield from L.wait(5.0)
        log("end (held)")
        return
    yield from L.grip(FINGER_OPEN)
    yield from L.wait(0.5)
    yield from L.move((BOT_X, wy, carry + 0.03), yaw=yaw, duration=1.0)
    yield from L.move((0.20, 0.25, TOP + 0.15), yaw=0.0, duration=1.2)
    yield from L.wait(1.5)
    log("end")
