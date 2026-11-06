"""Anchoring mock-up on the station table. A light-wood deck (fixed box 26 x 80 cm, 6 cm tall) covers the near part
of the table, from 10 cm to 36 cm in front of the robot; its far edge is the boat's side. Beyond it, on the table
("the water"), lies a blue seabed patch (11 x 16 cm) in front of the right half. A white 6 mm anchor line, 55 cm
long, lies straight across the deck from left to right, 20 cm in front of the robot. Its inboard (bitter) end, on the
left, carries a red toggle block (30 x 22 x 14 mm); its outboard end carries a black anchor block (40 x 28 x 14 mm,
20 g) on the right. On the left of the deck, 8 cm further forward than the toggle, stands the bitter-end cleat: a
grey open-top pocket (inner 50 x 50 mm, 20 mm walls) whose right-hand wall has a 14 mm slot from the deck up. With
the toggle in the pocket and the line led out through the slot, the line cannot pull free (the toggle is wider than
the slot).

The robot must lower the anchor over the side onto the seabed patch. Good seamanship: make the bitter end fast
first, so that the line cannot run out after the anchor, then lower (not drop) the anchor and let go of everything.

Why two arms: the cleat and the toggle are on the left (y +21 cm) where the right arm cannot reach; the anchor lies
at y -36 cm where the left arm cannot reach.

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
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Anchor the boat: get the black anchor over the side onto the blue seabed patch, so that neither the "
               "anchor nor its line can be lost overboard. Then let go of everything.")

# -- deck (fixed) and the water beyond its far edge ------------------------------------------------------------
DECK_X0, DECK_X1 = 0.10, 0.36  # deck from 10 to 36 cm in front of the robot; x = 0.36 is the boat's side
DECK_H = 0.06
DECK = TOP + DECK_H  # deck top
WOOD = (0.72, 0.58, 0.40)
DECK_BOX = Box("deck", pos=((DECK_X0 + DECK_X1) / 2, 0.0, TOP + DECK_H / 2), size=(DECK_X1 - DECK_X0, 0.80, DECK_H),
               color=WOOD)
EDGE_X = DECK_X1
MAT_X0, MAT_X1, MAT_Y0, MAT_Y1 = 0.39, 0.50, -0.20, -0.04  # seabed patch on the table beyond the side
MAT_T = 0.002
MAT = Box("seabed", pos=((MAT_X0 + MAT_X1) / 2, (MAT_Y0 + MAT_Y1) / 2, TOP + MAT_T / 2),
          size=(MAT_X1 - MAT_X0, MAT_Y1 - MAT_Y0, MAT_T), color=(0.12, 0.30, 0.70))

# -- bitter-end cleat: open-top pocket on the deck with a slot in its right-hand (-y) wall ----------------------
CLEAT = np.array([0.28, 0.21])
IN_X, IN_Y, WALL, CL_H = 0.050, 0.050, 0.004, 0.020
SLOT_W = 0.014
GREY = (0.40, 0.42, 0.46)


def _cleat() -> list[Box]:
    cx, cy = float(CLEAT[0]), float(CLEAT[1])
    zc = DECK + CL_H / 2
    ox = IN_X + 2 * WALL
    side = (ox - SLOT_W) / 2
    return [
        Box("cleat_near", pos=(cx - IN_X / 2 - WALL / 2, cy, zc), size=(WALL, IN_Y + 2 * WALL, CL_H), color=GREY),
        Box("cleat_far", pos=(cx + IN_X / 2 + WALL / 2, cy, zc), size=(WALL, IN_Y + 2 * WALL, CL_H), color=GREY),
        Box("cleat_back", pos=(cx, cy + IN_Y / 2 + WALL / 2, zc), size=(IN_X, WALL, CL_H), color=GREY),
        Box("cleat_slot_a", pos=(cx - ox / 2 + side / 2, cy - IN_Y / 2 - WALL / 2, zc), size=(side, WALL, CL_H),
            color=GREY),
        Box("cleat_slot_b", pos=(cx + ox / 2 - side / 2, cy - IN_Y / 2 - WALL / 2, zc), size=(side, WALL, CL_H),
            color=GREY),
    ]


CLEAT_BOXES = _cleat()

# -- anchor line: toggle on the bitter end, anchor block on the outboard end -----------------------------------
R = 0.003
LINE_LEN = 0.55
LINE_X = 0.20
LINE_Y0 = 0.21  # bitter end (toggle side)
TOGGLE = Plug(size=(0.030, 0.022, 0.014), mass=0.010, color=(0.85, 0.10, 0.08))
ANCHOR = Plug(size=(0.040, 0.028, 0.014), mass=0.020, color=(0.06, 0.06, 0.07))
LINE = cable("anchor_line", start=(LINE_X, LINE_Y0, DECK + TOGGLE.size[2] / 2 + 0.0005), yaw=-math.pi / 2,
             length=LINE_LEN, radius=R, start_plug=TOGGLE, end_plug=ANCHOR, cable_mass=0.06,
             color=(0.93, 0.93, 0.90), bend_stiffness=0.004, bend_damping=0.0008)

OBJECTS = [DECK_BOX, MAT, *CLEAT_BOXES, LINE]

TWIN_EXTRA = [
    {"name": "toggle_and_anchor", "kind": "joint", "real": "3D-printed PLA blocks glued/crimped onto each line end"},
]

# -- env hook: env handle, control-step counter, reset of the episode memory -----------------------------------
_ENV: dict = {"env": None, "step": 0}
_MON: dict = {}


def _reset_episode() -> None:
    _MON.clear()
    _MON.update(over=False, order_broken=False, fell=False, last_step=-1, prev_anchor=None, prev_step=None,
                max_down=0.0, cleated_ever=False)


_reset_episode()


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_anchor_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            _ENV["step"] += 1
            return base(self, *a, **k)

        _task_step._anchor_hooked = True
        cls._task_step = _task_step
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


_install_hooks()


def grippers() -> list[np.ndarray]:
    """World TCP positions of both grippers (yam_arena.runtime.tcp_world, env 0); [] offline."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


def _dt() -> float:
    env = _ENV["env"]
    try:
        return float(env.unwrapped.step_dt)
    except Exception:  # noqa: BLE001
        return 0.05


# -- geometry ------------------------------------------------------------------------------------------------
RELEASE_DIST = 0.03  # every gripper TCP >= 3 cm from every line point, the toggle and the anchor
TOGGLE_FREE = 0.035  # "made fast" needs no gripper TCP within 3.5 cm of the toggle centre
LINE_FREE = 0.03  # ... and none within 3 cm of the inboard half of the line (the anchor arm holds the outboard end)
FALL_V = 0.5  # anchor moving down faster than 0.5 m/s = let fall, not lowered
ON_BOTTOM_Z = TOP + MAT_T + ANCHOR.size[2] / 2 + 0.006  # anchor centre at most 6 mm above resting on the patch


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    poly = np.asarray(poly, dtype=float)
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _ok(c) -> bool:
    return bool(np.isfinite(c.points).all() and c.start_plug is not None and np.isfinite(c.start_plug).all()
                and c.end_plug is not None and np.isfinite(c.end_plug).all())


def toggle_in_cleat(objects) -> bool:
    """Toggle centre inside the pocket walls and below the rim, and the line leaves through the slot."""
    c = objects[LINE.name]
    if not _ok(c):
        return False
    t = c.start_plug
    inside = abs(t[0] - CLEAT[0]) <= IN_X / 2 and abs(t[1] - CLEAT[1]) <= IN_Y / 2 and t[2] <= DECK + CL_H
    if not inside:
        return False
    # the line where it crosses the slotted wall's line (y = CLEAT_y - IN_Y/2 - WALL/2) passes inside the slot
    wall_y = CLEAT[1] - IN_Y / 2 - WALL / 2
    p = _dense(c.polyline())
    k = int(np.argmax(p[:, 1] <= wall_y)) if (p[:, 1] <= wall_y).any() else -1
    return bool(k > 0 and abs(p[k, 0] - CLEAT[0]) <= SLOT_W / 2 + 0.002 and p[k, 2] <= DECK + CL_H)


def toggle_free(objects) -> bool:
    """No gripper on the bitter end: every TCP 3.5 cm from the toggle and 3 cm from the inboard half of the line."""
    c = objects[LINE.name]
    pts = _dense(c.polyline())
    inboard = pts[: len(pts) // 2]
    return all(float(np.linalg.norm(c.start_plug - g)) >= TOGGLE_FREE
               and float(np.linalg.norm(inboard - g, axis=1).min()) >= LINE_FREE for g in grippers())


def made_fast(objects) -> bool:
    return toggle_in_cleat(objects) and toggle_free(objects)


def anchor_over(objects) -> bool:
    a = objects[LINE.name].end_plug
    return bool(a is not None and np.isfinite(a).all() and a[0] > EDGE_X)


def anchor_on_bottom(objects) -> bool:
    a = objects[LINE.name].end_plug
    return bool(a is not None and np.isfinite(a).all() and MAT_X0 <= a[0] <= MAT_X1 and MAT_Y0 <= a[1] <= MAT_Y1
                and a[2] <= ON_BOTTOM_Z)


def released(objects) -> bool:
    c = objects[LINE.name]
    pts = np.vstack([_dense(c.polyline()), c.start_plug[None], c.end_plug[None]])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def _track(objects) -> None:
    """Latched episode facts, updated once per control step: order broken (anchor went over the side while the
    bitter end was not made fast) and anchor let fall (downward speed > 0.5 m/s at any step)."""
    step = _ENV["step"]
    if step == _MON["last_step"]:
        return
    _MON["last_step"] = step
    c = objects[LINE.name]
    if not _ok(c):
        return
    a = np.asarray(c.end_plug, dtype=float)
    if _MON["prev_anchor"] is not None and step > _MON["prev_step"]:
        vz = (a[2] - _MON["prev_anchor"][2]) / ((step - _MON["prev_step"]) * _dt())
        _MON["max_down"] = max(_MON["max_down"], -vz)
        if -vz > FALL_V:
            _MON["fell"] = True
    _MON["prev_anchor"], _MON["prev_step"] = a, step
    mf = made_fast(objects)
    _MON["cleated_ever"] |= mf
    if anchor_over(objects) and not _MON["over"]:
        _MON["over"] = True
        if not mf:
            _MON["order_broken"] = True


def score(objects) -> float:
    """0 if the anchor went over the side (centre past the deck edge) before the toggle was in the cleat and let go
    (no gripper within 3.5 cm of the toggle or 3 cm of the inboard half of the line), if the anchor ever moved down faster than 0.5 m/s (let fall), if any gripper is within 3 cm of the line, toggle
    or anchor at the end, or if the toggle is not in the cleat at the end. Else 0.4 (made fast, anchor still on
    deck), 0.7 (anchor over the side but not resting on the seabed patch), 1.0 (anchor resting on the patch)."""
    _track(objects)
    c = objects[LINE.name]
    if not _ok(c) or _MON["order_broken"] or _MON["fell"]:
        return 0.0
    if not released(objects) or not toggle_in_cleat(objects):
        return 0.0
    if anchor_on_bottom(objects):
        return 1.0
    if anchor_over(objects):
        return 0.7
    return 0.4


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects[LINE.name].resting(max_speed=0.02)


def distance(objects) -> float:
    """Anchor centre distance (m) to the patch centre; 1 m once a disqualifier fired or before the line is made fast."""
    _track(objects)
    c = objects[LINE.name]
    if not _ok(c) or _MON["order_broken"] or _MON["fell"]:
        return 1.0
    mc = np.array([(MAT_X0 + MAT_X1) / 2, (MAT_Y0 + MAT_Y1) / 2, ON_BOTTOM_Z])
    return float(np.linalg.norm(c.end_plug - mc)) + (0.0 if toggle_in_cleat(objects) else 0.5)


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    _track(objects)
    return {"made_fast": float(made_fast(objects)), "order_kept": float(not _MON["order_broken"]),
            "anchor_over": float(anchor_over(objects)), "lowered_not_dropped": float(not _MON["fell"]),
            "on_bottom": float(anchor_on_bottom(objects)), "released": float(released(objects))}


VARIATION = {
    "cable:anchor_line": {"dx": 0.015, "dy": 0.01, "yaw": 0.04},
}

# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------


def demo_script(ctx):
    """Right arm takes the anchor and lifts it clear of the deck (still inboard) while the left arm takes the toggle,
    sets it in the cleat pocket with the line led out through the slot and lets go. Only then does the right arm
    carry the anchor over the side, lower it slowly to just above the seabed patch and let go. Closed loop: grasps
    follow the measured blocks, the toggle is placed by its measured offset in the gripper, and the anchor is lowered
    until it is about 2 mm above the patch, steering its measured centre to the target, then opens and rises
    straight up before moving away."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    # test only: none | first | hand | hand_line | skip | drop | hold; tier04, tier07 = partial-credit checks
    cheat = os.environ.get("ANCHOR_CHEAT", "")
    PINCH = -0.015  # pads ~30 mm apart: fits inside the 50 mm pocket
    yl = yr = -math.pi / 2  # fingers close along x across each block; wrist cameras on the -y side
    hi = DECK + 0.05
    carry = TOP + 0.12

    def log(tag):
        st = ctx.states()
        c = st[LINE.name]
        print(f"[demo t={ctx.t:5.1f}] {tag}: score={score(st):.2f} fast={made_fast(st)} incl={toggle_in_cleat(st)} "
              f"over={anchor_over(st)} bottom={anchor_on_bottom(st)} rel={released(st)} mon={ {k: _MON[k] for k in ('order_broken', 'fell', 'max_down')} } "
              f"toggle={np.round(c.start_plug, 3)} anchor={np.round(c.end_plug, 3)} "
              f"a_up={(c.end_plug[2] - TOP - MAT_T - ANCHOR.size[2] / 2) * 1000:.1f}mm a_dirz={c.end_dir[2]:.2f}", flush=True)

    log("start")
    if cheat == "none":
        yield from L.wait(3.0)
        log("end")
        return

    st = ctx.states()
    t0, a0 = st[LINE.name].start_plug, st[LINE.name].end_plug

    def take_anchor():
        yield from Rt.move((a0[0], a0[1], hi), yaw=yr, finger=FINGER_OPEN, duration=1.5)
        yield from Rt.move((a0[0], a0[1], a0[2] + 0.010), yaw=yr, duration=1.0)
        yield from Rt.grip(FINGER_CLOSED)
        yield from Rt.move((a0[0], a0[1], carry), yaw=yr, duration=1.0)

    def make_fast():
        yield from L.move((t0[0], t0[1], hi), yaw=yl, finger=PINCH, duration=1.5)
        yield from L.move((t0[0], t0[1], t0[2] + 0.010), yaw=yl, duration=1.0)
        yield from L.grip(FINGER_CLOSED)
        yield from L.move((t0[0], t0[1], hi), yaw=yl, duration=0.8)
        # place the toggle by its measured offset from the TCP
        g = grippers()
        tp = ctx.states()[LINE.name].start_plug
        off = (tp - g[0]) if g else np.zeros(3)
        tx, ty = CLEAT[0] - off[0], CLEAT[1] - off[1]
        yield from L.move((tx, ty, hi), yaw=yl, duration=1.2)
        yield from L.wait(0.3)
        tp = ctx.states()[LINE.name].start_plug
        g = grippers()
        if g:
            off = tp - g[0]
            tx, ty = CLEAT[0] - off[0], CLEAT[1] - off[1]
        drop = DECK + 0.004 + TOGGLE.size[2] / 2 + 0.010
        yield from L.move((tx, ty, drop + 0.02), yaw=yl, duration=0.8)
        yield from L.move((tx, ty, drop), yaw=yl, duration=0.6)
        if cheat == "hand":  # keep holding the bitter end in the pocket while the anchor goes over
            return
        yield from L.grip(PINCH)
        yield from L.move((tx, ty, hi + 0.03), yaw=yl, duration=0.8)
        if cheat == "hand_line":  # lets go of the toggle but closes on the line just outside the slot instead
            p = ctx.states()[LINE.name].polyline()
            q = p[int(np.argmax(p[:, 1] <= CLEAT[1] - IN_Y / 2 - WALL - 0.04))]
            yield from L.move((q[0], q[1], hi), yaw=yl, duration=0.8)
            yield from L.move((q[0], q[1], q[2] + 0.010), yaw=yl, duration=0.8)
            yield from L.grip(FINGER_CLOSED)
            log("left arm on the line")
            return
        yield from L.move((0.16, 0.26, hi + 0.03), yaw=yl, duration=0.8)

    def lower_anchor():
        tgt = np.array([0.44, -0.25] if cheat == "tier07" else [0.445, -0.11])  # tier07: on the table beside the patch
        g = grippers()
        ap = ctx.states()[LINE.name].end_plug
        off = (ap - g[1]) if g else np.zeros(3)
        yield from Rt.move((a0[0] + 0.02, a0[1] + 0.06, carry), yaw=yr, duration=1.2)
        yield from Rt.move((tgt[0] - off[0], tgt[1] - off[1], carry), yaw=yr, duration=2.0)
        if cheat == "drop":  # let go from carry height over the patch
            yield from Rt.grip(FINGER_OPEN)
            return
        yield from Rt.wait(0.5)
        bottom = TOP + MAT_T + ANCHOR.size[2] / 2
        cx, cy = tgt[0] - off[0], tgt[1] - off[1]
        z = carry
        # pay out slowly, closed loop on the anchor's height and position: aim its measured centre at the target
        for _ in range(40):
            ap = ctx.states()[LINE.name].end_plug
            gap = ap[2] - (bottom + 0.002)
            cx, cy = cx + 0.7 * (tgt[0] - ap[0]), cy + 0.7 * (tgt[1] - ap[1])
            if gap <= 0.0015 and np.hypot(tgt[0] - ap[0], tgt[1] - ap[1]) <= 0.006:
                break
            z -= float(np.clip(gap, 0.0, 0.02))
            yield from Rt.move((cx, cy, z), yaw=yr, duration=0.6)
        log("lowered")
        if cheat == "hold":
            return
        yield from Rt.grip(FINGER_OPEN)
        yield from Rt.wait(0.4)
        g = Rt.tcp()
        yield from Rt.move((g[0], g[1], g[2] + 0.05), yaw=yr, duration=0.8)  # straight up first: no drag on the block
        yield from Rt.move((cx - 0.02, cy - 0.03, carry), yaw=yr, duration=0.8)

    if cheat == "first":  # anchor over the side first, then make the bitter end fast
        yield from take_anchor()
        yield from lower_anchor()
        log("anchor down")
        yield from make_fast()
    elif cheat == "skip":  # never make the bitter end fast
        yield from take_anchor()
        yield from lower_anchor()
    elif cheat == "tier04":  # made fast, anchor left on the deck
        yield from make_fast()
    else:
        yield from ctx.both(make_fast(), take_anchor())
        log("made fast")
        yield from lower_anchor()
    for _ in range(4):
        yield from L.wait(0.5)
    log("end")
    if cheat in ("hand", "hand_line"):
        yield from L.grip(PINCH)
        yield from L.move((0.16, 0.26, hi + 0.03), yaw=yl, duration=0.8)
        for _ in range(3):
            yield from L.wait(0.5)
        log("end2")
