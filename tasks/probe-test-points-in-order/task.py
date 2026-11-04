"""Electronics test bench on the station table. A yellow bench meter (fixed block 120 x 120 x 70 mm) stands at the far
left. Its thin red test lead (5 mm, 50 cm) leaves the meter's right side and lies straight along the far edge of the
work area, ending in a red probe (rigid body 45 x 10 x 10 mm, tip pointing right) whose tip sits in a black probe
holder (an open-top slot, 14 mm wide, 14 mm deep) at the far right. In front of the lead lies a fixed green circuit
board (210 x 280 x 4 mm) with three square gold test pads (20 x 20 mm) marked 1, 2 and 3 in white digits on the
board just in front of each pad. The pads lie on a diagonal: pad 1 in the middle, pad 2 near-left, pad 3 far-right.
Three small loose components stand on the board between the pads and the lead (two blue electrolytic capacitors and
a black relay, a few grams each). The robot must touch the probe tip on pad 1, then 2, then 3, holding each for at
least 1 s, without knocking any component (keep the lead off the board), then put the probe back in its holder and
let go of everything.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (env var PROBE_DEMO): correct (default) | nothing | nearest | drag.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from boxfont import text_boxes  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Probe the gold test pads in numbered order: touch the red probe's tip on pad 1, then 2, then 3, "
               "holding it still on each for at least one second. Keep the red lead off the board so no component "
               "is knocked over. Then put the probe back in its black holder and let go.")

RED = (0.80, 0.08, 0.06)
YELLOW = (0.95, 0.75, 0.10)
BLACK = (0.06, 0.06, 0.07)
PCB = (0.08, 0.42, 0.18)
GOLD = (0.85, 0.68, 0.25)
SILK = (0.97, 0.97, 0.95)
CAP_BLUE = (0.12, 0.25, 0.75)

# -- test lead (anchored at the meter) and probe ------------------------------------------------------------------
R = 0.0025  # 5 mm lead
LEAD_LEN = 0.50
LEAD_X = 0.40
ANCHOR_Y = 0.22
PL, PW, PH = 0.045, 0.010, 0.010  # probe body: length along the lead, width, height
SPAWN_Z = TOP + PH / 2 + 0.0005
LEAD = cable("lead", start=(LEAD_X, ANCHOR_Y, SPAWN_Z), yaw=-math.pi / 2, length=LEAD_LEN, radius=R,
             anchor_start=True, end_plug=Plug(size=(PL, PW, PH), mass=0.012, color=RED), color=RED,
             bend_stiffness=0.01, bend_damping=0.001)
PROBE_Y0 = ANCHOR_Y - LEAD_LEN - PL / 2  # probe centre y at spawn
TIP_Y0 = PROBE_Y0 - PL / 2

# -- meter (fixed): its right face 6 mm left of the lead's start ---------------------------------------------------
METER_W = 0.12
METER = Box("meter", pos=(LEAD_X, ANCHOR_Y + 0.006 + METER_W / 2, TOP + 0.035), size=(0.12, METER_W, 0.07),
            color=YELLOW)
METER_SCREEN = Box("meter_display", pos=(LEAD_X + 0.02, ANCHOR_Y + 0.006 + METER_W / 2, TOP + 0.0705),
                   size=(0.05, 0.09, 0.001), color=(0.15, 0.18, 0.16))

# -- probe holder (fixed): open-top slot along y, closed at its -y end -------------------------------------------------
SLOT_W, SLOT_H, WALL = 0.014, 0.014, 0.004
SLOT_MOUTH = PROBE_Y0 + PL / 2 - 0.026  # slot opening (y): the tip's last 19 mm lie in the slot at spawn
SLOT_END = TIP_Y0 - 0.003  # inner face of the end wall
_sl = SLOT_MOUTH - SLOT_END
HOLDER = [
    Box("holder_wall_far", pos=(LEAD_X + SLOT_W / 2 + WALL / 2, (SLOT_MOUTH + SLOT_END - WALL) / 2, TOP + SLOT_H / 2),
        size=(WALL, _sl + WALL, SLOT_H), color=BLACK),
    Box("holder_wall_near", pos=(LEAD_X - SLOT_W / 2 - WALL / 2, (SLOT_MOUTH + SLOT_END - WALL) / 2, TOP + SLOT_H / 2),
        size=(WALL, _sl + WALL, SLOT_H), color=BLACK),
    Box("holder_end", pos=(LEAD_X, SLOT_END - WALL / 2, TOP + SLOT_H / 2), size=(SLOT_W, WALL, SLOT_H), color=BLACK),
]

# -- circuit board, pads, silkscreen digits ---------------------------------------------------------------------------
BX0, BX1, BY0, BY1, BT_H = 0.15, 0.36, -0.25, 0.03, 0.004
BT = TOP + BT_H  # board top
BOARD = Box("board", pos=((BX0 + BX1) / 2, (BY0 + BY1) / 2, TOP + BT_H / 2), size=(BX1 - BX0, BY1 - BY0, BT_H),
            color=PCB)
PAD, PAD_H = 0.020, 0.001
PT = BT + PAD_H  # pad top
PADS = {1: (0.265, -0.125), 2: (0.205, -0.050), 3: (0.325, -0.200)}
PX = 0.0035  # digit pixel: digits 17.5 x 24.5 mm
DIGIT_DX = -0.032  # digit centre in front of (nearer the robot than) its pad
PAD_BOXES = [Box(f"pad_{k}", pos=(x, y, BT + PAD_H / 2), size=(PAD, PAD, PAD_H), color=GOLD) for k, (x, y) in PADS.items()]
DIGITS = [b for k, (x, y) in PADS.items() for b in text_boxes(Box, f"silk_{k}", str(k), (x + DIGIT_DX, y), BT, PX,
                                                             color=SILK)]

# -- loose components (free) --------------------------------------------------------------------------------------
CAP_R, CAP_H, CAP_M = 0.005, 0.020, 0.003
RELAY = (0.016, 0.012, 0.016)
COMPONENTS = [
    Cylinder("cap_a", pos=(0.340, -0.005, BT + CAP_H / 2 + 0.0005), radius=CAP_R, height=CAP_H, mass=CAP_M,
             color=CAP_BLUE),
    Cylinder("cap_b", pos=(0.300, 0.005, BT + CAP_H / 2 + 0.0005), radius=CAP_R, height=CAP_H, mass=CAP_M,
             color=CAP_BLUE),
    Box("relay", pos=(0.340, -0.085, BT + RELAY[2] / 2 + 0.0005), size=RELAY, mass=0.004, color=BLACK),
]
COMP_NAMES = [c.name for c in COMPONENTS]

OBJECTS = [LEAD, METER, METER_SCREEN, *HOLDER, BOARD, *PAD_BOXES, *DIGITS, *COMPONENTS]

# -- scoring ---------------------------------------------------------------------------------------------------------
CONTROL_HZ = 20
DWELL_STEPS = 20  # 1 s of continuous touch on one pad = a visit
TOUCH_GAP = 0.005  # tip's lower edge at most 5 mm above the pad surface ...
TOUCH_PEN = 0.004  # ... and not more than 4 mm below it (numerical contact)
KNOCK_DIST = 0.010  # a component moved more than 1 cm (horizontally) ...
KNOCK_TILT = math.radians(30.0)  # ... or tilted more than 30 deg = knocked
RELEASE_DIST = 0.035  # m: both grippers' TCP at least this far from the probe and every lead point at the end
HOLDER_IN = 0.008  # probe tip at least 8 mm past the slot mouth


class _Episode:
    def __init__(self):
        self.comp0: dict | None = None
        self.knocked: str | None = None
        self.cur_pad = 0  # pad touched in the previous step (0 = none)
        self.run = 0  # consecutive steps on cur_pad
        self.counted = False  # this contact already registered as a visit
        self.visits: list[int] = []
        self.progress = 0  # pads visited in correct order so far
        self.bad: str | None = None  # first out-of-order visit
        self.steps = 0


_EP = _Episode()
_ENV: dict = {"env": None}


def reset_episode() -> None:
    global _EP
    _EP = _Episode()


def _probe(objects) -> tuple[np.ndarray, np.ndarray] | None:
    st = objects[LEAD.name]
    p = np.asarray(st.end_plug, dtype=float) if st.end_plug is not None else None
    if p is None or not np.isfinite(p).all():
        return None
    a = p - np.asarray(st.points[-1], dtype=float)
    return p, a / (np.linalg.norm(a) + 1e-12)


def probe_tip(objects) -> np.ndarray | None:
    """Centre of the probe's tip end face (the end away from the lead)."""
    pr = _probe(objects)
    return None if pr is None else pr[0] + pr[1] * PL / 2


def touched_pad(objects) -> int:
    """Pad number the probe tip is on (tip over the pad, its lower edge within 5 mm of the pad surface); 0 if none."""
    t = probe_tip(objects)
    if t is None:
        return 0
    low = t[2] - PH / 2 - PT
    if not -TOUCH_PEN <= low <= TOUCH_GAP:
        return 0
    for k, (x, y) in PADS.items():
        if abs(t[0] - x) <= PAD / 2 and abs(t[1] - y) <= PAD / 2:
            return k
    return 0


def _tilt(q) -> float:
    x, y, z, w = (float(v) for v in q)
    c = 1.0 - 2.0 * (x * x + y * y)  # world z of the body's z axis
    return math.acos(max(-1.0, min(1.0, c)))


def _update(objects) -> None:
    ep = _EP
    ep.steps += 1
    if ep.comp0 is None:
        ep.comp0 = {n: np.asarray(objects[n].pos, float).copy() for n in COMP_NAMES}
    for n in COMP_NAMES:
        st = objects[n]
        d = float(np.linalg.norm(np.asarray(st.pos, float)[:2] - ep.comp0[n][:2]))
        if ep.knocked is None and (d > KNOCK_DIST or _tilt(st.quat_xyzw) > KNOCK_TILT or not np.isfinite(d)):
            ep.knocked = n
            print(f"[probe] component {n} knocked (moved {d * 1000:.0f} mm, tilt "
                  f"{math.degrees(_tilt(st.quat_xyzw)):.0f} deg) at step {ep.steps}", flush=True)
    k = touched_pad(objects)
    if k != ep.cur_pad:
        ep.cur_pad, ep.run, ep.counted = k, 0, False
    if k:
        ep.run += 1
        if ep.run >= DWELL_STEPS and not ep.counted:
            ep.counted = True
            ep.visits.append(k)
            if k == ep.progress + 1:
                ep.progress = k
            elif k != ep.progress and ep.bad is None:  # re-touching the pad just visited is harmless
                ep.bad = f"pad {k} visited after {ep.visits[:-1]}"
            print(f"[probe] visit pad {k} at step {ep.steps}: visits {ep.visits}, progress {ep.progress}, "
                  f"bad={ep.bad}", flush=True)


def _install_hooks() -> None:
    """Record the trial history every control step (latched until the next reset); keep a handle on the env."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_probe_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            mod = getattr(getattr(self, "task", None), "module", None)
            getattr(mod, "reset_episode", reset_episode)()
            return orig(self, *a, **k)

        reset._probe_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_probe_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            upd = getattr(getattr(self.task, "module", None), "_update", None)
            if upd is not None:
                from yam_arena.scene import object_states

                getattr(self.task.module, "_ENV", _ENV)["env"] = self._env
                upd(object_states(self._env, self.task.names))
            return base(self, *a, **k)

        _task_step._probe_hooked = True
        cls._task_step = _task_step


_install_hooks()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP; [] if unknown (offline)."""
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
    st = objects[LEAD.name]
    pts = [np.asarray(p, float) for p in st.points]
    if st.end_plug is not None:
        pts.append(np.asarray(st.end_plug, float))
    return all(float(np.linalg.norm(p - g)) >= RELEASE_DIST for g in grippers() for p in pts)


def in_holder(objects) -> bool:
    """Probe tip inside the holder slot: between its walls, at least 8 mm past the mouth, below the wall tops."""
    t = probe_tip(objects)
    if t is None:
        return False
    return bool(abs(t[0] - LEAD_X) <= SLOT_W / 2 and SLOT_END - 0.002 <= t[1] <= SLOT_MOUTH - HOLDER_IN
                and t[2] <= TOP + SLOT_H)


def score(objects) -> float:
    """0 if: a component moved > 1 cm or tilted > 30 deg at any time; a pad was visited out of order (a visit is >= 1 s
    of continuous tip contact; a visit to any pad other than the next one, or the one just visited, is out of order);
    at the end the probe tip is not in its holder or a gripper TCP is within 3.5 cm of the probe or lead.
    Otherwise 1/3 per pad visited in order (1, then 2, then 3)."""
    if _EP.knocked or _EP.bad or not in_holder(objects) or not released(objects):
        return 0.0
    return round(_EP.progress / 3.0, 3)


def success(objects) -> bool:
    return bool(score(objects) == 1.0 and np.linalg.norm(objects[LEAD.name].lin_vel) < 0.02)


def distance(objects) -> float:
    """How far (m) the probe tip is from the next pad to visit (from the holder once all three are done)."""
    t = probe_tip(objects)
    if t is None:
        return 1.0
    if _EP.progress < 3:
        x, y = PADS[_EP.progress + 1]
        goal = np.array([x, y, PT + PH / 2])
    else:
        goal = np.array([LEAD_X, TIP_Y0, TOP + PH / 2])
    return float(np.linalg.norm(t - goal))


# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------------
# Right arm: pick the probe out of its holder by its back end and hold it up beyond the board's far-right corner.
# Left arm: pinch the lead 22 cm from the meter and lift it; for each pad it first moves to a high hold point beside
# the board (the lead then hangs from it to the probe with 2-3 cm slack, clear of the components), then the right arm
# carries the probe over at 12 cm and sets its tip on pads 1, 2, 3 (closed loop on the tip's measured position),
# 1.6 s each. The left arm lays the lead back down beyond the board and lets go; the right arm puts the probe back in
# the slot, lets go, and both arms move clear.

GRASP_OPEN = -0.014  # pads ~28 mm apart for the lead
FINGER_PINCH = -0.020  # pads ~44 mm apart for the probe
LEAD_GRASP_S = 0.22  # m along the lead from the meter
PROBE_GRASP_BACK = 0.010  # pinch the probe this far from its lead end
PROBE_TCP_DZ = 0.010  # TCP this far above the probe centre when pinching
HI = TOP + 0.08
# Right-arm grip yaw = direction of the probe tip, so the wrist camera looks at the tip and the pads: -90 deg (tip
# along -y) at the holder, -60 deg at the pads (the wrist cannot turn to -90 deg over pads 1 and 2).
PAD_YAW = math.radians(-60.0)


def _log(ctx, tag=""):
    st = ctx.states()
    t = probe_tip(st)
    lead = np.asarray(st[LEAD.name].points)
    over = (lead[:, 0] < BX1 + 0.005) & (lead[:, 0] > BX0) & (lead[:, 1] > BY0) & (lead[:, 1] < BY1)
    lowest = float(lead[over, 2].min() - BT) if over.any() else float("nan")
    comp = {n: np.round((np.asarray(st[n].pos)[:2] - (_EP.comp0 or {}).get(n, np.zeros(3))[:2]) * 1000, 1).tolist()
            for n in COMP_NAMES} if _EP.comp0 else {}
    print(f"[t={ctx.t:5.2f}] {tag} tip={np.round(t, 4) if t is not None else None} pad={touched_pad(st)} "
          f"visits={_EP.visits} prog={_EP.progress} bad={_EP.bad} knocked={_EP.knocked} lead_over_board_min_z="
          f"{lowest * 1000:.0f}mm holder={in_holder(st)} rel={released(st)} comp_mm={comp} score={score(st)}",
          flush=True)


def _lead_point(objects, s: float) -> np.ndarray:
    pts = np.asarray(objects[LEAD.name].points, float)
    i = int(np.clip(round(s / LEAD.seg_len - 0.5), 0, len(pts) - 1))
    return pts[i]


def _pick_probe(ctx):
    from yam_arena.constants import FINGER_CLOSED

    arm = ctx.right
    p, a = _probe(ctx.states())
    g = p - a * (PL / 2 - PROBE_GRASP_BACK)
    yield from arm.move((g[0], g[1], HI), yaw=-math.pi / 2, finger=FINGER_PINCH, duration=1.5)
    yield from arm.move((g[0], g[1], p[2] + PROBE_TCP_DZ), yaw=-math.pi / 2, duration=1.2)
    yield from arm.grip(FINGER_CLOSED, 0.6)
    yield from arm.move((g[0], g[1], HI), yaw=-math.pi / 2, duration=1.0)
    return np.array([g[0], g[1], HI])


def _tip_offset(ctx, tcp) -> np.ndarray:
    """Probe tip position relative to the commanded TCP (measured while the arm holds still)."""
    return probe_tip(ctx.states()) - np.asarray(tcp, float)


def _touch(ctx, k, tcp, low_z, hold_s=1.6):
    """Move the held probe's tip over pad ``k``, set it down on the pad, hold, lift. Returns the new TCP."""
    arm = ctx.right
    off = _tip_offset(ctx, tcp)
    x, y = PADS[k]
    above = np.array([x - off[0], y - off[1], tcp[2]])
    yield from arm.move(above, yaw=PAD_YAW, duration=1.5)
    yield from arm.wait(0.3)
    off = _tip_offset(ctx, above)
    above = np.array([x - off[0], y - off[1], above[2]])
    # tip lower edge 1 mm into the pad surface (pressing lightly)
    down_z = PT + PH / 2 - 0.001 - off[2]
    yield from arm.move((above[0], above[1], low_z), yaw=PAD_YAW, duration=0.8)
    yield from arm.move((above[0], above[1], down_z), yaw=PAD_YAW, duration=0.8)
    _log(ctx, f"on pad {k}")
    yield from arm.wait(hold_s)
    _log(ctx, f"held pad {k}")
    yield from arm.move((above[0], above[1], low_z), yaw=PAD_YAW, duration=0.6)
    return np.array([above[0], above[1], low_z])


def _stow(ctx, tcp):
    """Put the held probe back in the holder slot (tip first, closed loop), let go, move clear."""
    from yam_arena.constants import FINGER_CLOSED

    arm = ctx.right
    tip_goal = np.array([LEAD_X, TIP_Y0 + 0.001])
    off = _tip_offset(ctx, tcp)
    above = np.array([tip_goal[0] - off[0], tip_goal[1] - off[1], tcp[2]])
    yield from arm.move(above, yaw=-math.pi / 2, duration=0.6)
    yield from arm.wait(0.3)
    off = _tip_offset(ctx, above)
    above = np.array([tip_goal[0] - off[0], tip_goal[1] - off[1], above[2]])
    yield from arm.move(above, yaw=-math.pi / 2, duration=0.5)
    down_z = TOP + PH / 2 + 0.001 - off[2]
    yield from arm.move((above[0], above[1], down_z + 0.02), yaw=-math.pi / 2, duration=1.2)
    yield from arm.move((above[0], above[1], down_z), yaw=-math.pi / 2, duration=0.8)
    yield from arm.grip(FINGER_PINCH, 0.6)
    _log(ctx, "probe stowed")
    yield from arm.move((above[0], above[1], HI + 0.04), yaw=-math.pi / 2, duration=1.0)
    yield from arm.move((0.25, -0.30, TOP + 0.16), yaw=0.0, finger=FINGER_CLOSED, duration=1.2)


def _lift_lead(ctx, hold):
    """Pinch the lead LEAD_GRASP_S from the meter (where it still lies on the table) and lift it to ``hold``."""
    from yam_arena.constants import FINGER_CLOSED

    arm = ctx.left
    g = _lead_point(ctx.states(), LEAD_GRASP_S)
    yield from arm.move((g[0], g[1], HI), yaw=-math.pi / 2, finger=GRASP_OPEN, duration=1.5)
    yield from arm.move((g[0], g[1], g[2] + 0.010), yaw=-math.pi / 2, duration=1.0)
    yield from arm.grip(FINGER_CLOSED, 0.6)
    yield from arm.move((g[0], g[1], g[2] + 0.05), yaw=-math.pi / 2, duration=0.8)
    yield from arm.move(hold, yaw=-math.pi / 2, duration=1.2)


def _lay_lead(ctx):
    """Lay the held lead back down on the table beyond the board's far edge, let go, move clear."""
    from yam_arena.constants import FINGER_CLOSED

    arm = ctx.left
    put = np.array([LEAD_X + 0.005, ANCHOR_Y - LEAD_GRASP_S + 0.015, TOP + R + 0.010])
    yield from arm.move((put[0], put[1], put[2] + 0.05), yaw=-math.pi / 2, duration=1.5)
    yield from arm.move(put, yaw=-math.pi / 2, duration=0.8)
    yield from arm.grip(GRASP_OPEN, 0.6)
    yield from arm.move((put[0], put[1], put[2] + 0.08), yaw=-math.pi / 2, duration=0.8)
    yield from arm.move((0.20, 0.30, TOP + 0.16), yaw=0.0, finger=FINGER_CLOSED, duration=1.2)


# Left-hand lead hold points (beside / above the board's far-left part, within reach of the lead's 22 cm from the
# meter) while the probe is staged (0) or on each pad: the lead hangs from the hand to the probe with 2-3 cm slack.
HOLD = {0: np.array([0.37, 0.05, TOP + 0.12]), 1: np.array([0.30, 0.13, TOP + 0.16]),
        2: np.array([0.34, 0.14, TOP + 0.16]), 3: np.array([0.35, 0.07, TOP + 0.14])}
STAGE = np.array([0.40, -0.13, TOP + 0.12])  # right TCP: probe held up beyond the board's far-right corner
TRANSIT_Z = TOP + 0.12  # right TCP height while the probe moves between pads
STOW_HI = np.array([0.40, -0.25, TOP + 0.10])  # right TCP: probe held up near the holder while the lead is laid


def demo_script(ctx):
    mode = os.environ.get("PROBE_DEMO", "correct")  # test only: correct | nothing | nearest | drag
    _log(ctx, f"start ({mode})")
    if mode == "nothing":
        for _ in range(6):
            yield from ctx.left.wait(1.0)
        _log(ctx, "end")
        return
    order = {"nearest": (3, 1, 2)}.get(mode, (1, 2, 3))
    lead_hand = mode != "drag"  # drag: one-armed, the lead trails over the board
    low_z = TOP + (0.035 if mode == "drag" else 0.06)
    tcp = yield from _pick_probe(ctx)
    if lead_hand:
        yield from ctx.right.move(STAGE, yaw=PAD_YAW, duration=1.5)
        tcp = STAGE
        yield from _lift_lead(ctx, HOLD[0])
    _log(ctx, "probe picked")
    for k in order:
        if lead_hand:  # lift the probe high, move the lead hand, then the probe: the lead never sweeps low
            tcp = np.array([tcp[0], tcp[1], TRANSIT_Z])
            yield from ctx.right.move(tcp, yaw=PAD_YAW, duration=0.8)
            yield from ctx.left.move(HOLD[k], yaw=-math.pi / 2, duration=1.2)
        tcp = yield from _touch(ctx, k, tcp, low_z)
    if lead_hand:
        yield from ctx.both(ctx.right.move(STOW_HI, yaw=-math.pi / 2, duration=1.8),
                            ctx.left.move(HOLD[0], yaw=-math.pi / 2, duration=1.8))
        tcp = STOW_HI
        yield from _lay_lead(ctx)
    yield from _stow(ctx, tcp)
    yield from ctx.left.wait(1.0)
    _log(ctx, "end")
