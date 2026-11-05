"""On the station table, in front of the robot, a white open-ended cable tray (40 cm long left-right, 8 cm wide
inside, 3 cm walls) holds four 6 mm cables lying side by side on its floor, 15 mm apart, held in by two black
bridge clips spanning the tray (10 mm clearance under each). From the robot outwards: grey A, blue, grey B, grey C.
The three grey cables are live; the blue one is decommissioned. All four run under both clips and out of the
tray's right end, each ending in an identical black plug on the table; the plugs are staggered so that neither the
plug nearest the robot (grey A) nor the one sticking out furthest (grey C) is the blue one. The blue cable's left
end lies inside the tray, left of the left clip, so it can only come out lengthwise through the clips. It must be
drawn out of the tray's right end and left entirely outside the tray, without pulling the live cables out of the
clips or moving their plugs more than 3 cm, and then let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the cables below are simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("The blue cable in the white tray is decommissioned. Pull it out of the tray through the two black "
               "clips and lay it on the table completely outside the tray, then let go. The three grey cables are "
               "live: they must stay in the tray under both clips.")

# -- tray (open at both ends), fixed -----------------------------------------------------------------------------
TX0, TX1 = 0.26, 0.34  # inner faces of the near / far wall (x)
TY0, TY1 = -0.20, 0.20  # tray ends (y)
WT = 0.004  # wall thickness
FLOOR_T = 0.003
FLOOR = TOP + FLOOR_T  # tray floor surface
WALL_H = 0.03  # wall height above the floor
WALL_TOP = FLOOR + WALL_H
TRAY_C = ((TX0 + TX1) / 2, (TY0 + TY1) / 2)
WHITE = (0.93, 0.93, 0.91)
TRAY = [
    Box("tray_floor", pos=(TRAY_C[0], TRAY_C[1], TOP + FLOOR_T / 2), size=(TX1 - TX0 + 2 * WT, TY1 - TY0, FLOOR_T),
        color=WHITE),
    Box("tray_wall_near", pos=(TX0 - WT / 2, TRAY_C[1], TOP + (FLOOR_T + WALL_H) / 2),
        size=(WT, TY1 - TY0, FLOOR_T + WALL_H), color=WHITE),
    Box("tray_wall_far", pos=(TX1 + WT / 2, TRAY_C[1], TOP + (FLOOR_T + WALL_H) / 2),
        size=(WT, TY1 - TY0, FLOOR_T + WALL_H), color=WHITE),
]

# -- clips: bridges spanning the tray between the walls ----------------------------------------------------------
CLIP_Y = (0.06, -0.10)  # left, right
CLIP_W = 0.02  # along y
CLIP_CLEAR = 0.010  # floor to clip underside
CLIP_T = 0.005
CLIPS = [Box(f"clip_{k}", pos=(TRAY_C[0], y, FLOOR + CLIP_CLEAR + CLIP_T / 2), size=(TX1 - TX0, CLIP_W, CLIP_T),
             color=(0.08, 0.08, 0.08)) for k, y in zip(("left", "right"), CLIP_Y)]

# -- cables: four 6 mm cables side by side on the tray floor, all leaving the tray's right end with a plug ----------
R = 0.003  # 6 mm cables
PITCH = 0.015  # loosely laid: 9 mm gaps between neighbours
NAMES = ("grey_a", "blue", "grey_b", "grey_c")  # from the robot outwards
GREYS = ("grey_a", "grey_b", "grey_c")
ROW_X = {n: 0.2775 + i * PITCH for i, n in enumerate(NAMES)}
Z0 = FLOOR + R + 0.0005
START_Y = {"grey_a": 0.18, "blue": 0.09, "grey_b": 0.18, "grey_c": 0.18}  # left (free) ends, inside the tray
END_Y = {"grey_a": -0.28, "blue": -0.34, "grey_b": -0.28, "grey_c": -0.40}  # right cable ends (plugs beyond)
COLOR = {"grey_a": (0.55, 0.56, 0.58), "blue": (0.08, 0.30, 0.85), "grey_b": (0.50, 0.51, 0.53),
         "grey_c": (0.60, 0.60, 0.62)}
PLUG = Plug(size=(0.03, 0.010, 0.010), mass=0.008, color=(0.06, 0.06, 0.06))
CABLES = {n: cable(n, start=(ROW_X[n], START_Y[n], Z0), yaw=-math.pi / 2, length=START_Y[n] - END_Y[n], radius=R,
                   end_plug=PLUG, color=COLOR[n], bend_stiffness=0.01, bend_damping=0.001) for n in NAMES}
PLUG_START = {n: np.array([ROW_X[n], END_Y[n] - PLUG.size[0] / 2]) for n in NAMES}

OBJECTS = [*TRAY, *CLIPS, *CABLES.values()]

# -- scoring --------------------------------------------------------------------------------------------------
RELEASE_DIST = 0.04  # every gripper TCP at least this far from every cable point at the end


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _pts(objects, name) -> np.ndarray:
    c = objects[name]
    extra = [p[None] for p in (c.start_plug, c.end_plug) if p is not None]
    return np.vstack([_dense(c.polyline())] + extra)


def under_clip(pts: np.ndarray, clip_y: float) -> bool:
    """Some cable point lies under the clip: within its width, between the walls, below its underside."""
    ok = (np.abs(pts[:, 1] - clip_y) <= CLIP_W / 2) & (pts[:, 0] >= TX0) & (pts[:, 0] <= TX1) \
        & (pts[:, 2] < FLOOR + CLIP_CLEAR)
    return bool(ok.any())


def over_tray(pts: np.ndarray) -> np.ndarray:
    """Per point: over the tray's footprint in top view (walls included, any height)."""
    return (pts[:, 0] >= TX0 - WT - R) & (pts[:, 0] <= TX1 + WT + R) & (pts[:, 1] >= TY0 - R) \
        & (pts[:, 1] <= TY1 + R)


PLUG_TOL = 0.03  # a live cable's plug may move at most this far (horizontally) from its starting mark


def grey_intact(objects, name) -> bool:
    """A live cable still passes under both clips and its plug is within 3 cm of its starting mark."""
    c = objects[name]
    p = _pts(objects, name)
    return all(under_clip(p, y) for y in CLIP_Y) and c.end_plug is not None \
        and float(np.linalg.norm(c.end_plug[:2] - PLUG_START[name])) <= PLUG_TOL


def blue_unclipped(objects) -> bool:
    p = _pts(objects, "blue")
    return not any(under_clip(p, y) for y in CLIP_Y)


def blue_out(objects) -> bool:
    return not bool(over_tray(_pts(objects, "blue")).any())


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_bundle_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._bundle_hooked = True
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
    pts = np.vstack([_pts(objects, n) for n in NAMES])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def score(objects) -> float:
    """0 unless both grippers are >= 4 cm from every cable and every grey cable still passes under both clips with
    its plug within 3 cm of its starting mark. Then 1.0 if no part of the blue cable (or its plug) is over the tray in top view;
    0.5 if the blue cable no longer passes under either clip but part of it is still over the tray; else 0."""
    if not released(objects):
        return 0.0
    if not all(grey_intact(objects, n) for n in GREYS):
        return 0.0
    if blue_out(objects):
        return 1.0
    return 0.5 if blue_unclipped(objects) else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["blue"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Total blue cable length (m) still over the tray (1 m if a grey cable has been disturbed)."""
    if not all(grey_intact(objects, n) for n in GREYS):
        return 1.0
    d = _dense(objects["blue"].polyline(), 0.004)
    return float(over_tray(d).sum() * 0.004)


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------


def _poly(objects, name) -> tuple[np.ndarray, np.ndarray]:
    pts = _dense(objects[name].polyline(), 0.002)
    return pts, np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _regrasp_point(objects, name: str) -> tuple[np.ndarray, float] | None:
    """A point on cable ``name`` outside the tray (>= 3 cm beyond its right end), >= 3 cm from every other cable and
    plug, lying low; the one closest to the tray along the cable. Returns (point, cable heading) or None."""
    pts, s = _poly(objects, name)
    others = np.vstack([_pts(objects, n) for n in NAMES if n != name])
    for j in range(len(pts)):
        p = pts[j]
        if p[1] > TY0 - 0.03 or p[2] > TOP + 0.02:
            continue
        if float(np.linalg.norm(others[:, :2] - p[:2], axis=1).min()) < 0.03:
            continue
        if s[j] > s[-1] - 0.03:  # keep off the plug
            continue
        d = pts[min(j + 4, len(pts) - 1)] - pts[max(j - 4, 0)]
        return p, math.atan2(d[1], d[0])
    return None


def demo_script(ctx):
    """Left arm holds the live cables down on the tray floor, left of the blue cable's end (fingers straddling the
    bundle, pads on grey A and grey C). Right arm draws the blue cable out of the tray's right end hand over hand:
    grasp its plug, pull it up and to the right, towards the robot, let go, re-grasp the blue cable outside the tray
    where no other cable is near, pull again, until no part of it is over the tray; then both arms let go and lift.
    BUNDLE_CHEAT=nothing|nearest|longest runs a shortcut instead (each must score 0): nearest/longest pull the cable
    nearest the robot (grey A) / with the plug sticking out furthest (grey C) the same way."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("BUNDLE_CHEAT", "")
    target = {"nearest": "grey_a", "longest": "grey_c"}.get(cheat, "blue")
    hi = TOP + 0.10
    over = 0.010
    ry = -math.pi / 2  # right grip: fingers close along x, across a cable running along y

    def log(tag):
        st = ctx.states()
        g = {n: (int(grey_intact(st, n)), np.round(st[n].end_plug[:2] - PLUG_START[n], 3).tolist()) for n in GREYS}
        b = st["blue"]
        print(f"[demo] {tag}: score={score(st):.2f} blue_out={blue_out(st)} unclipped={blue_unclipped(st)} "
              f"blue_start={np.round(b.start_tip, 3)} plug={np.round(b.end_plug, 3)} greys={g} "
              f"dist={distance(st):.3f} grippers={[np.round(x, 3).tolist() for x in grippers()]}", flush=True)

    log("start")
    if cheat == "nothing":
        yield from L.wait(3.0)
        log("end")
        return

    # 1) left arm: hold the live cables down on the tray floor, left of the blue cable's end
    press = (0.30, 0.135)
    press_z = FLOOR + 0.005 + 0.013  # pad tips 5 mm above the floor (1 mm into the 6 mm cables)
    yield from L.move((press[0], press[1], hi), yaw=math.pi / 2, finger=-0.0225, duration=1.2)
    yield from L.move((press[0], press[1], press_z), yaw=math.pi / 2, duration=0.8)
    log("held")

    # 2) right arm: grab the plug and pull up/right/towards the robot
    pull_to = [(0.22, -0.50, TOP + 0.12), (0.20, -0.52, TOP + 0.10), (0.22, -0.50, TOP + 0.10)]
    pp = ctx.states()[target].end_plug
    yield from Rt.move((pp[0], pp[1], hi), yaw=ry, finger=-0.014, duration=1.2)
    pp = ctx.states()[target].end_plug
    yield from Rt.move((pp[0], pp[1], pp[2] + over), yaw=ry, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((pp[0], pp[1] - 0.02, TOP + 0.04), yaw=ry, duration=0.5)
    yield from Rt.move(pull_to[0], yaw=ry, duration=2.0)
    yield from Rt.move((pull_to[0][0], pull_to[0][1], TOP + 0.025), yaw=ry, duration=0.6)
    yield from Rt.grip(FINGER_OPEN, 0.4)
    yield from Rt.move((pull_to[0][0], pull_to[0][1], hi), yaw=ry, duration=0.5)
    log("pull 1")

    for k in range(5):
        st = ctx.states()
        if target == "blue" and blue_out(st):
            break
        g = _regrasp_point(st, target)
        if g is None:
            print("[demo] no regrasp point", flush=True)
            break
        p, h = g
        gy = _grip_yaw(h)
        yield from Rt.move((p[0], p[1], hi), yaw=gy, finger=-0.014, duration=1.0)
        yield from Rt.move((p[0], p[1], p[2] + over), yaw=gy, duration=0.7)
        yield from Rt.grip(FINGER_CLOSED, 0.5)
        yield from Rt.move((p[0], p[1], TOP + 0.05), yaw=gy, duration=0.4)
        tgt = pull_to[1 + k % 2]
        yield from Rt.move(tgt, yaw=ry, duration=1.8)
        yield from Rt.move((tgt[0], tgt[1], TOP + 0.025), yaw=ry, duration=0.6)
        yield from Rt.grip(FINGER_OPEN, 0.4)
        yield from Rt.move((tgt[0], tgt[1], hi), yaw=ry, duration=0.5)
        log(f"pull {k + 2}")

    # 3) let go of the live cables, both arms clear
    yield from ctx.both(L.move((press[0], press[1], hi), yaw=math.pi / 2, finger=FINGER_OPEN, duration=0.8),
                        Rt.move((0.18, -0.35, hi), yaw=ry, duration=0.8))
    log("released")
    yield from L.wait(1.0)
    log("end")
