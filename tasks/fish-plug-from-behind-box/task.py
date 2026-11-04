"""On the station table, a fixed grey back panel (20 mm thick, 36 cm long, 25 cm tall) stands across the far side of
the table, like a wall. A free brown storage box (300 x 120 x 200 mm, 2 kg, not fixed) stands in front of it with
its long side parallel to the panel and a 35 mm gap between them. An unplugged black phone charger (45 x 26 x 16 mm
brick, 20 g) has fallen into that gap, hidden behind the box: it lies flat on the table in the gap, and its white
6 mm cord (25 cm, a small silver USB-C plug on its far end) runs along the gap and out past the box's left end, so
its last ~15 cm and the USB-C plug lie on the open table in front of the left arm. A green open tray is fixed to the
table to the right of the box, in front of the right arm.

The charger can't be reached directly (box and panel are too tall for a gripper to reach down into the gap), so it
must be fished out by its cord without moving the box. The gap is narrower than the charger is long, so the charger
cannot turn inside it: pulling the cord towards the robot wraps it round the box's back corner and wedges the
charger between that corner and the panel, and further pulling drags the box. The cord has to be drawn out
lengthwise along the gap until the charger has come out past the panel's end, and only then brought round. Only
the left arm reaches the cord and the gap's exit; the tray is in front of the right arm only, so the charger is
handed across.

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

INSTRUCTION = ("The unplugged phone charger has fallen into the gap behind the brown box. Fish it out by its white "
               "cord and put it in the green tray, then let go. Don't move the box: keep it in place (within 2 cm) "
               "and upright.")

GREY = (0.55, 0.56, 0.58)
BROWN = (0.62, 0.45, 0.28)
GREEN = (0.20, 0.62, 0.30)

# -- box (free) and back panel (fixed) ------------------------------------------------------------------------------
BOX_SIZE = (0.12, 0.30, 0.20)  # x (depth), y (length), z (height)
BOX_MASS = 2.0
BOX_BACK_X = 0.43  # box's back face (towards the panel)
BOX_Y0, BOX_Y1 = -0.10, 0.20  # box's right and left ends
BOX_C = np.array([BOX_BACK_X - BOX_SIZE[0] / 2, (BOX_Y0 + BOX_Y1) / 2])
BOX = Box("storage_box", pos=(float(BOX_C[0]), float(BOX_C[1]), TOP + BOX_SIZE[2] / 2 + 0.0005), size=BOX_SIZE,
          mass=BOX_MASS, color=BROWN)

GAP = 0.035
PANEL_T, PANEL_H = 0.02, 0.25
PANEL_X0 = BOX_BACK_X + GAP  # panel front face
PANEL_Y0, PANEL_Y1 = -0.14, 0.22  # panel ends 2 cm past the box's left end
PANEL = Box("back_panel", pos=(PANEL_X0 + PANEL_T / 2, (PANEL_Y0 + PANEL_Y1) / 2, TOP + PANEL_H / 2),
            size=(PANEL_T, PANEL_Y1 - PANEL_Y0, PANEL_H), color=GREY)
GAP_X = BOX_BACK_X + GAP / 2  # gap centre line

# -- charger on a free cord, lying along the gap ---------------------------------------------------------------------
R = 0.003  # 6 mm cord
CORD_LEN = 0.25
CHARGER = Plug(size=(0.045, 0.026, 0.016), mass=0.02, color=(0.06, 0.06, 0.07))
CONNECTOR = Plug(size=(0.025, 0.012, 0.008), mass=0.005, color=(0.75, 0.75, 0.76))  # USB-C plug on the far end
CHARGER_Y0 = 0.09  # charger centre at the start (behind the box)
CORD_START = (GAP_X, CHARGER_Y0 + CHARGER.size[0] / 2, TOP + CHARGER.size[2] / 2 + 0.0005)
CORD = cable("charger_cord", start=CORD_START, yaw=math.pi / 2, length=CORD_LEN, radius=R, anchor_start=False,
             start_plug=CHARGER, end_plug=CONNECTOR, color=(0.93, 0.93, 0.92), bend_stiffness=0.01, bend_damping=0.001,
             cable_mass=0.04)

# -- tray (fixed): inner 100 x 100 mm, 4 mm floor, 15 mm walls 3 mm thick --------------------------------------------
TRAY_C = np.array([0.33, -0.235])
TRAY_IN = (0.10, 0.10)
TRAY_FLOOR, TRAY_WALL_H, TRAY_T = 0.004, 0.015, 0.003


def _tray() -> list[Box]:
    x, y = map(float, TRAY_C)
    ix, iy = TRAY_IN
    ox, oy = ix + 2 * TRAY_T, iy + 2 * TRAY_T
    zc = TOP + TRAY_WALL_H / 2
    return [
        Box("tray_floor", pos=(x, y, TOP + TRAY_FLOOR / 2), size=(ox, oy, TRAY_FLOOR), color=GREEN),
        Box("tray_wall_far", pos=(x + ix / 2 + TRAY_T / 2, y, zc), size=(TRAY_T, oy, TRAY_WALL_H), color=GREEN),
        Box("tray_wall_near", pos=(x - ix / 2 - TRAY_T / 2, y, zc), size=(TRAY_T, oy, TRAY_WALL_H), color=GREEN),
        Box("tray_wall_right", pos=(x, y - iy / 2 - TRAY_T / 2, zc), size=(ix, TRAY_T, TRAY_WALL_H), color=GREEN),
        Box("tray_wall_left", pos=(x, y + iy / 2 + TRAY_T / 2, zc), size=(ix, TRAY_T, TRAY_WALL_H), color=GREEN),
    ]


OBJECTS = [BOX, PANEL, CORD, *_tray()]

# -- scoring ---------------------------------------------------------------------------------------------------------
BOX_TOL = 0.02  # no box corner may move more than this (m) in the horizontal plane, at any time
TILT_DEG = 10.0  # box leaning more than this = tipped
RELEASE_DIST = 0.03  # at the end every gripper (TCP) at least this far from the cord, charger and USB-C plug
OUT_X = BOX_BACK_X - 0.05  # partial credit: charger centre at least 5 cm nearer the robot than the box's back face


def _yaw(q) -> float:
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _tilt_deg(q) -> float:
    x, y, z, w = q
    cz = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, cz))))


def box_corners(box) -> np.ndarray:
    """Horizontal positions (4, 2) of the box's corners (from its centre and yaw)."""
    c, a = box.pos[:2], _yaw(box.quat_xyzw)
    rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    hx, hy = BOX_SIZE[0] / 2, BOX_SIZE[1] / 2
    loc = np.array([[hx, hy], [hx, -hy], [-hx, hy], [-hx, -hy]])
    return c + loc @ rot.T


_START_CORNERS = BOX_C + np.array([[1, 1], [1, -1], [-1, 1], [-1, -1]]) * np.array(BOX_SIZE[:2]) / 2
_EP = {"max_shift": 0.0, "max_tilt": 0.0, "fell": False}


def _reset_episode() -> None:
    _EP.update(max_shift=0.0, max_tilt=0.0, fell=False)


_ENV: dict = {"env": None}


def _install_hooks() -> None:
    """Reset the episode memory at every embodiment reset; keep a handle on the env for the gripper check."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
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
    cls.reset._task_reset_hooks.append(_reset_episode)
    if not getattr(cls._task_step, "_fish_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._fish_hooked = True
        cls._task_step = _task_step


_install_hooks()
_START_PTS = CORD.start_points()


def _track(objects) -> None:
    box, c = objects["storage_box"], objects["charger_cord"]
    shift = float(np.linalg.norm(box_corners(box) - _START_CORNERS, axis=1).max())
    if np.abs(c.points - _START_PTS).max() < 0.01 and shift < 0.002 and box.resting():
        _reset_episode()  # untouched starting layout => new episode
    _EP["max_shift"] = max(_EP["max_shift"], shift)
    _EP["max_tilt"] = max(_EP["max_tilt"], _tilt_deg(box.quat_xyzw))
    if box.pos[2] < TOP - 0.02 or c.start_plug[2] < TOP - 0.02:
        _EP["fell"] = True


def box_disturbed() -> bool:
    return _EP["max_shift"] > BOX_TOL or _EP["max_tilt"] > TILT_DEG or _EP["fell"]


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
    c = objects["charger_cord"]
    pts = np.vstack([c.polyline(), c.start_plug[None], c.end_plug[None]])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def charger_in_tray(objects) -> bool:
    """Charger centre over the tray's inner floor, at least 5 mm in from the walls, lying on the floor."""
    p = objects["charger_cord"].start_plug
    d = np.abs(p[:2] - TRAY_C)
    ok_xy = d[0] <= TRAY_IN[0] / 2 - 0.005 and d[1] <= TRAY_IN[1] / 2 - 0.005
    return bool(ok_xy and p[2] <= TOP + TRAY_FLOOR + CHARGER.size[2] / 2 + 0.008)


def charger_out(objects) -> bool:
    """Charger fished out: its centre at least 5 cm nearer the robot than the box's back face, low (on the table
    or in the tray)."""
    p = objects["charger_cord"].start_plug
    return bool(p[0] <= OUT_X and p[2] <= TOP + CHARGER.size[2] / 2 + 0.02)


def success(objects) -> bool:
    _track(objects)
    c = objects["charger_cord"]
    return bool(not box_disturbed() and charger_in_tray(objects) and released(objects) and c.resting(0.02, 1.0))


def score(objects) -> float:
    """0 if at any time a box corner moved more than 2 cm horizontally from its start, the box tilted more than
    10 deg, or the box or charger fell off the table; 0 if at the end a gripper is within 3 cm of the cord, the
    charger or the USB-C plug. Otherwise 1.0 for the charger lying in the tray; 0.5 for the charger fished out
    (centre at least 5 cm nearer the robot than the box's back face, on the table) but not in the tray; else 0."""
    _track(objects)
    if box_disturbed() or not released(objects):
        return 0.0
    if charger_in_tray(objects):
        return 1.0
    if charger_out(objects):
        return 0.5
    return 0.0


def distance(objects) -> float:
    """Charger centre to the tray centre (m); 1 m once the box has been disturbed."""
    _track(objects)
    if box_disturbed():
        return 1.0
    return float(np.linalg.norm(objects["charger_cord"].start_plug[:2] - TRAY_C))


def progress(objects) -> dict:
    """Stage diagnostic (not the score): charger out of the gap, fished out in front, in tray, box kept, released."""
    _track(objects)
    p = objects["charger_cord"].start_plug
    return {
        "charger_out_of_gap": float(p[1] - CHARGER.size[0] / 2 > PANEL_Y1 or p[0] < BOX_BACK_X - 0.01),
        "charger_fished_out": float(charger_out(objects)),
        "charger_in_tray": float(charger_in_tray(objects)),
        "box_kept": float(not box_disturbed()),
        "released": float(released(objects)),
    }


# Seeded variation ranges for a later runner (the demo runs on the nominal layout). The gap stays 31-39 mm (charger
# 26 mm wide fits, 45 mm long cannot turn), the cord stays in the gap.
VARIATION = {
    "cable:charger_cord": {"dx": 0.002, "dy": 0.02, "yaw": 0.0},
    "obj:storage_box": {"dx": 0.004, "dy": 0.015, "yaw": 0.0},
}

# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------


def _log(ctx, tag=""):
    st = ctx.states()
    c = st["charger_cord"]
    print(f"[t={ctx.t:5.2f}] {tag}: charger={np.round(c.start_plug, 3)} cord_end={np.round(c.end_tip, 3)} "
          f"box_shift={_EP['max_shift']:.3f} tilt={_EP['max_tilt']:.1f} in_tray={charger_in_tray(st)} "
          f"out={charger_out(st)} released={released(st)} score={score(st):.2f}", flush=True)


def _yaw_near(heading: float, want: float) -> float:
    """The grip yaw equivalent to ``heading`` (mod pi) closest to ``want``."""
    return min((heading + k * math.pi for k in range(-3, 4)), key=lambda a: abs(a - want))


def _charger_heading(c) -> float:
    d = c.start_dir
    return math.atan2(d[1], d[0])


HAUL_Y1 = 0.55  # the left arm draws the connector along the gap line to here
HAND_XY = np.array([0.23, 0.0])  # hand-over spot in front of the box
HAND_YAW = -1.1  # charger axis at the hand-over; the right arm's wrist camera then looks towards the tray
CARRY_Z = TOP + 0.085  # carry height (8.5 cm); the carry goes round the box's end, not over it
YAW_HAUL = -math.pi / 2  # left wrist camera looks back along the gap at the box corner / charger
CLEAR_Y = PANEL_Y1 + 0.01  # charger's trailing end beyond this = out of the gap, free to turn


def charger_clear(objects) -> bool:
    c = objects["charger_cord"]
    return bool(c.start_plug[1] - CHARGER.size[0] / 2 > CLEAR_Y)


def _grasp_connector(ctx, arm, f_open, f_closed):
    c = ctx.states()["charger_cord"]
    p = c.end_plug
    gy = _yaw_near(math.atan2(c.end_dir[1], c.end_dir[0]), YAW_HAUL)
    yield from arm.move((p[0], p[1], TOP + 0.06), yaw=gy, finger=f_open, duration=1.5)
    p = ctx.states()["charger_cord"].end_plug
    yield from arm.move((p[0], p[1], p[2] + 0.008), yaw=gy, duration=1.0)
    yield from arm.grip(f_closed, 0.6)
    return p, gy


def _haul(ctx, L, f_open, f_closed):
    """Draw the cord out lengthwise along the gap until the charger is past the panel's end (closed loop)."""
    for _ in range(3):
        p, gy = yield from _grasp_connector(ctx, L, f_open, f_closed)
        yield from L.move((GAP_X, HAUL_Y1, p[2] + 0.010), yaw=gy, duration=4.0)
        _log(ctx, "hauled")
        yield from L.grip(f_open, 0.5)
        yield from L.move((GAP_X - 0.02, HAUL_Y1, TOP + 0.06), yaw=gy, duration=0.6)
        if charger_clear(ctx.states()):
            return True
    return False


def _left_charger_to_hand(ctx, L, f_open, f_closed):
    c = ctx.states()["charger_cord"]
    pp = c.start_plug
    gy = _yaw_near(_charger_heading(c), YAW_HAUL)
    yield from L.move((pp[0], pp[1], TOP + 0.06), yaw=gy, duration=1.2)
    pp = ctx.states()["charger_cord"].start_plug
    yield from L.move((pp[0], pp[1], pp[2] + 0.008), yaw=gy, duration=1.0)
    yield from L.grip(f_closed, 0.6)
    yield from L.move((pp[0], pp[1], CARRY_Z), yaw=gy, duration=0.8)
    yield from L.move((0.24, 0.26, CARRY_Z), yaw=gy, duration=1.6)  # round the box's left end, not over it
    yield from L.move((HAND_XY[0], HAND_XY[1] + 0.01, CARRY_Z), yaw=HAND_YAW, duration=1.6)
    off = ctx.states()["charger_cord"].start_plug[:2] - L.tcp()[:2]
    gx, gy2 = HAND_XY - off
    yield from L.move((gx, gy2, TOP + CHARGER.size[2] / 2 + 0.010), yaw=HAND_YAW, duration=1.0)
    yield from L.grip(f_open, 0.5)
    yield from L.move((gx - 0.02, gy2 + 0.06, CARRY_Z), yaw=HAND_YAW, duration=0.8)
    yield from L.move((0.15, 0.30, TOP + 0.12), yaw=0.0, duration=1.0)


def _right_charger_to_tray(ctx, Rt, f_open, f_closed, *, release=True):
    c = ctx.states()["charger_cord"]
    pp = c.start_plug
    gy = _yaw_near(_charger_heading(c), HAND_YAW)
    yield from Rt.move((pp[0], pp[1], TOP + 0.06), yaw=gy, finger=f_open, duration=1.2)
    pp = ctx.states()["charger_cord"].start_plug
    yield from Rt.move((pp[0], pp[1], pp[2] + 0.008), yaw=gy, duration=1.0)
    yield from Rt.grip(f_closed, 0.6)
    yield from Rt.move((pp[0], pp[1], CARRY_Z), yaw=gy, duration=0.8)
    _log(ctx, "right picked")
    yield from Rt.move((HAND_XY[0], -0.15, CARRY_Z), yaw=gy, duration=1.2)  # clear of the box's front-right corner
    yield from Rt.move((float(TRAY_C[0]), float(TRAY_C[1]), CARRY_Z), yaw=gy, duration=1.2)
    _log(ctx, "over tray")
    off = ctx.states()["charger_cord"].start_plug[:2] - Rt.tcp()[:2]
    gx, gy2 = TRAY_C - off
    yield from Rt.move((gx, gy2, CARRY_Z), yaw=gy, duration=0.6)
    yield from Rt.move((gx, gy2, TOP + TRAY_FLOOR + CHARGER.size[2] / 2 + 0.010), yaw=gy, duration=1.0)
    if not release:
        return
    yield from Rt.grip(0.5 * f_open, 0.5)  # half open: the pads stay inside the tray walls
    yield from Rt.move((gx, gy2, CARRY_Z), yaw=gy, duration=0.8)
    yield from Rt.move((0.15, -0.30, TOP + 0.12), yaw=0.0, duration=1.0)


def demo_script(ctx):
    """FISH_DEMO=correct (default) | nothing | yank (grip the USB-C plug and pull the cord straight towards the
    robot, then carry the charger to the tray if it came out) | shove (push the box's left end 12 cm to the right
    first, then the correct retrieval) | hold (correct, but the right arm keeps holding the charger in the tray)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    mode = os.environ.get("FISH_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "nothing":
        yield from L.wait(5.0)
        _log(ctx, "end")
        return
    if mode == "yank":
        p, gy = yield from _grasp_connector(ctx, L, 0.6 * FINGER_OPEN, FINGER_CLOSED)
        yield from L.move((0.15, p[1], p[2] + 0.010), yaw=gy, duration=4.0)  # straight towards the robot
        _log(ctx, "yanked")
        yield from L.grip(FINGER_OPEN, 0.5)
        yield from L.move((0.15, p[1], TOP + 0.08), yaw=gy, duration=0.6)
        if charger_out(ctx.states()):
            yield from _left_charger_to_hand(ctx, L, FINGER_OPEN, FINGER_CLOSED)
            yield from _right_charger_to_tray(ctx, Rt, FINGER_OPEN, FINGER_CLOSED)
        _log(ctx, "end")
        return
    if mode == "shove":
        # push the box's left end 12 cm to the right (closed fingers against its end face)
        z = TOP + 0.05
        yield from L.move((BOX_C[0], BOX_Y1 + 0.05, z + 0.08), yaw=0.0, finger=FINGER_CLOSED, duration=1.5)
        yield from L.move((BOX_C[0], BOX_Y1 + 0.03, z), yaw=0.0, duration=1.0)
        yield from L.move((BOX_C[0], BOX_Y1 - 0.12, z), yaw=0.0, duration=4.0)
        yield from L.move((BOX_C[0], BOX_Y1 + 0.05, z + 0.08), yaw=0.0, duration=1.0)
        _log(ctx, "shoved")
    if not (yield from _haul(ctx, L, 0.6 * FINGER_OPEN, FINGER_CLOSED)):
        _log(ctx, "charger still in the gap: giving up")
        return
    yield from _left_charger_to_hand(ctx, L, FINGER_OPEN, FINGER_CLOSED)
    _log(ctx, "handed over")
    yield from _right_charger_to_tray(ctx, Rt, FINGER_OPEN, FINGER_CLOSED, release=mode != "hold")
    yield from ctx.both(L.wait(1.0), Rt.wait(1.0))
    _log(ctx, "end")
