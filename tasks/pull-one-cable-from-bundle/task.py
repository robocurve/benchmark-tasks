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


# -- camera-based estimates (top camera only) ------------------------------------------------------------------------
# The scripted demo sees the scene only through the top camera image (ctx.obs.images["top_cam"]) and knows the arms'
# own poses and the fixed layout from the setup (tray, clips, where the left hand presses); it never reads cable or
# plug poses from the simulator. vision.py (the kit's camera helpers) back-projects pixels onto horizontal planes with
# the station's published top-camera calibration. A hue/saturation mask picks the blue cable; dark pixels pick plugs;
# grey or dark pixels next to the blue cable are treated as other cables (obstacles for a regrasp).
import vision  # noqa: E402

CABLE_Z = TOP + R  # centre height of a cable lying on the table (the tray floor is only 3 mm higher)
PLUG_TOP_Z = TOP + PLUG.size[2]  # back-project plug pixels onto their top face


def _blue(img):  # thin blue cable: blends with the table, so a low saturation floor
    return vision.hsv_mask(img, 222, 22, min_sat=0.15)


def _dark(img):  # black plugs (and clips, arm parts: use only inside a region of interest)
    return np.asarray(img)[..., :3].max(axis=-1) < 70


def _greyish(img):  # grey cables, black plugs: unsaturated, not white (the oak table is saturated ~0.3-0.45)
    f = np.asarray(img, dtype=float)[..., :3]
    mx, mn = f.max(axis=-1), f.min(axis=-1)
    return ((mx - mn) / (mx + 1e-9) < 0.16) & (mx < 200)


def see_blue(ctx) -> np.ndarray:
    """(N, 2) table-plane positions of every blue pixel in the top camera image."""
    img = vision.top_image(ctx)
    return vision.top_camera(img.shape).to_plane(_blue(img), CABLE_Z)


def see_blue_plug(ctx):
    """Image estimate of the blue cable's plug centre (x, y): the blue cable's line outside the tray's right end gives
    x and the end of the blue run gives y; the dark plug pixels just beyond that end refine it. None if the blue
    cable is not seen outside the tray."""
    img = vision.top_image(ctx)
    cam = vision.top_camera(img.shape)
    pts = cam.to_plane(_blue(img), CABLE_Z)
    out = pts[(pts[:, 1] < TY0 - 0.02) & (pts[:, 1] > TY0 - 0.30) & (pts[:, 0] > TX0 - 0.03)
              & (pts[:, 0] < TX1 + 0.03)]
    if len(out) < 5:
        return None
    x, y_end = float(np.median(out[:, 0])), float(np.percentile(out[:, 1], 3))
    guess = np.array([x, y_end - PLUG.size[0] / 2])
    dk = cam.to_plane(_dark(img), PLUG_TOP_Z)
    near = dk[(np.abs(dk[:, 0] - x) < 0.010) & (dk[:, 1] < y_end + 0.005) & (dk[:, 1] > y_end - PLUG.size[0] - 0.01)]
    return near.mean(axis=0) if len(near) >= 3 else guess


def see_blue_over_tray(ctx) -> tuple[int, int]:
    """(blue pixels over the tray's footprint, blue pixels seen in all)."""
    pts = see_blue(ctx)
    over = (pts[:, 0] >= TX0 - WT - 0.005) & (pts[:, 0] <= TX1 + WT + 0.005) & (pts[:, 1] >= TY0 - 0.005) \
        & (pts[:, 1] <= TY1 + 0.005)
    return int(over.sum()), len(pts)


def see_regrasp(ctx):
    """Image estimate of where to re-grasp the blue cable: walking the blue pixels outward from where the cable
    leaves the tray's right end, the first point at least 3 cm beyond the tray end and at least 3 cm from every
    grey or dark pixel that is not part of the blue cable (other cables, plugs). Returns (xy, cable heading) or None."""
    img = vision.top_image(ctx)
    cam = vision.top_camera(img.shape)
    blue = cam.to_plane(_blue(img), CABLE_Z)
    out = blue[blue[:, 1] < TY0 + 0.01]
    exit_pts = out[out[:, 1] > TY0 - 0.02]
    if len(out) < 10 or len(exit_pts) < 2:
        return None
    anchor = np.array([np.median(exit_pts[:, 0]), TY0])
    poly = vision.trace_cable(img, _blue(img) & (_pixel_y(img, cam) < TY0 + 0.01), CABLE_Z, anchor,
                              start_ring=(0.01, 0.04), exclude=0.0)
    if poly is None:
        return None
    grey = cam.to_plane(_greyish(img) & ~_dark(img), CABLE_Z)
    if len(grey):  # drop the mixed edge pixels along the blue cable itself
        d_blue = np.array([np.linalg.norm(blue - o, axis=1).min() for o in grey])
        grey = grey[d_blue > 0.007]
    oth = np.vstack([grey, cam.to_plane(_dark(img), CABLE_Z)])
    oth = oth[(oth[:, 1] < TY0 + 0.02) & (oth[:, 1] > TY0 - 0.35) & (oth[:, 0] > 0.15) & (oth[:, 0] < 0.45)]
    for s in np.arange(0.03, poly.length - 0.03, 0.005):
        xy, h = poly.at(s)
        if xy[1] > TY0 - 0.03:
            continue
        if len(oth) and float(np.linalg.norm(oth - xy, axis=1).min()) < 0.03:
            continue
        return xy, h
    return None


def _pixel_y(img, cam):
    """Per pixel: table-plane y (for masking image regions by y)."""
    h, w = img.shape[:2]
    full = np.ones((h, w), bool)
    return cam.to_plane(full, CABLE_Z)[:, 1].reshape(h, w)


# -- closed-loop scripted demo (cabledemo.py): camera estimates + the arms' own poses only ---------------------------
PARK_R = (0.10, -0.55, TOP + 0.06)  # right arm out of the top camera's view of the tray end and the pulled cable
PRESS = (0.30, 0.135)  # where the left hand holds the bundle down (fixed layout: left of the blue cable's end)
PRESS_Z = FLOOR + 0.005 + 0.013  # pad tips 5 mm above the floor (1 mm into the 6 mm cables)
PULL_TO = [(0.22, -0.50, TOP + 0.12), (0.20, -0.52, TOP + 0.10), (0.22, -0.50, TOP + 0.10)]


def _log(ctx, tag):
    """Print what the camera sees; with BUNDLE_DEBUG=1 also the simulator's truth (diagnostics and offline checks
    only: nothing printed here feeds back into the demo's motions). BUNDLE_DUMP=dir saves the raw top image."""
    over, seen = see_blue_over_tray(ctx)
    line = f"[demo t={ctx.t:5.2f}] {tag}: blue px seen={seen} over tray={over}"
    if os.environ.get("BUNDLE_DEBUG"):
        st = ctx.states()
        g = {n: (int(grey_intact(st, n)), np.round(st[n].end_plug[:2] - PLUG_START[n], 3).tolist()) for n in GREYS}
        b = st["blue"]
        line += (f" | TRUE score={score(st):.2f} blue_out={blue_out(st)} unclipped={blue_unclipped(st)} "
                 f"blue_start={np.round(b.start_tip, 3)} plug={np.round(b.end_plug, 3)} greys={g} "
                 f"dist={distance(st):.3f}")
    print(line, flush=True)
    d = os.environ.get("BUNDLE_DUMP")
    if d:
        os.makedirs(d, exist_ok=True)
        np.save(os.path.join(d, f"{tag.replace(' ', '_')}_{ctx.t:06.2f}.npy"), np.asarray(ctx.obs.images["top_cam"]))


def _debug_truth(ctx, what, est):
    if os.environ.get("BUNDLE_DEBUG") and est is not None:
        st = ctx.states()
        if what == "plug":
            print(f"[demo] plug est={np.round(est, 3)} TRUE={np.round(st['blue'].end_plug, 3)}", flush=True)
        else:
            pts = _dense(st["blue"].polyline(), 0.002)
            others = np.vstack([_pts(st, n) for n in GREYS])
            print(f"[demo] regrasp est={np.round(est[0], 3)} yaw={math.degrees(est[1]):.0f} "
                  f"off_blue_mm={1000 * np.linalg.norm(pts[:, :2] - est[0], axis=1).min():.0f} "
                  f"to_grey_mm={1000 * np.linalg.norm(others[:, :2] - est[0], axis=1).min():.0f}", flush=True)


def _seen(ctx, fn, tries=4):
    """Look up to ``tries`` times (0.3 s apart): the first estimate that is not None."""
    for _ in range(tries):
        est = fn(ctx)
        if est is not None:
            return est
        yield from ctx.left.wait(0.3)
    return None


def demo_script(ctx):
    """Left arm holds the live cables down on the tray floor, left of the blue cable's end (fingers straddling the
    bundle, pads on grey A and grey C) at a fixed spot from the layout. Right arm, closed loop on the top camera
    only: it parks out of the camera's view, finds the blue cable's plug in the image (blue run outside the tray,
    dark plug at its end), grasps it, pulls it up and to the right, towards the robot, lets go; then looks again and
    re-grasps the blue cable outside the tray at the first point at least 3 cm from every other cable and plug, pulls
    again, until the camera sees no blue over the tray; then both arms let go and lift.
    BUNDLE_CHEAT=nothing|nearest|longest runs a shortcut instead (each must score 0)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    cheat = os.environ.get("BUNDLE_CHEAT", "")
    if cheat:
        yield from _cheat_script(ctx, cheat)
        return
    L, Rt = ctx.left, ctx.right
    hi = TOP + 0.10
    over = 0.010
    ry = -math.pi / 2  # right grip: fingers close along x, across a cable running along y

    # 1) left arm to its hold spot; right arm out of the camera's way
    yield from ctx.both(L.move((PRESS[0], PRESS[1], hi), yaw=math.pi / 2, finger=-0.0225, duration=1.2),
                        Rt.move(PARK_R, yaw=ry, finger=-0.014, duration=1.2))
    yield from ctx.both(L.move((PRESS[0], PRESS[1], PRESS_Z), yaw=math.pi / 2, duration=0.8), Rt.wait(0.8))
    _log(ctx, "held")

    # 2) find the blue plug in the image, grab it, pull up/right/towards the robot
    pp = yield from _seen(ctx, see_blue_plug)
    _debug_truth(ctx, "plug", pp)
    if pp is None:
        print("[demo] blue plug not seen", flush=True)
        return
    yield from Rt.move((pp[0], pp[1], hi), yaw=ry, duration=1.0)
    yield from Rt.move((pp[0], pp[1], TOP + PLUG.size[2] / 2 + over), yaw=ry, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((pp[0], pp[1] - 0.02, TOP + 0.04), yaw=ry, duration=0.5)
    yield from Rt.move(PULL_TO[0], yaw=ry, duration=2.0)
    yield from Rt.move((PULL_TO[0][0], PULL_TO[0][1], TOP + 0.025), yaw=ry, duration=0.6)
    yield from Rt.grip(FINGER_OPEN, 0.4)
    yield from Rt.move((PULL_TO[0][0], PULL_TO[0][1], hi), yaw=ry, duration=0.5)

    for k in range(5):
        yield from Rt.move(PARK_R, yaw=ry, finger=-0.014, duration=0.8)
        yield from Rt.wait(0.3)
        _log(ctx, f"look {k + 1}")
        over_px, seen = see_blue_over_tray(ctx)
        if seen >= 20 and over_px <= 2:
            break
        g = yield from _seen(ctx, see_regrasp)
        _debug_truth(ctx, "regrasp", g)
        if g is None:
            print("[demo] no regrasp point seen", flush=True)
            break
        p, h = g
        gy = _grip_yaw(h)
        yield from Rt.move((p[0], p[1], hi), yaw=gy, duration=1.0)
        yield from Rt.move((p[0], p[1], CABLE_Z + over), yaw=gy, duration=0.7)
        yield from Rt.grip(FINGER_CLOSED, 0.5)
        yield from Rt.move((p[0], p[1], TOP + 0.05), yaw=gy, duration=0.4)
        tgt = PULL_TO[1 + k % 2]
        yield from Rt.move(tgt, yaw=ry, duration=1.8)
        yield from Rt.move((tgt[0], tgt[1], TOP + 0.025), yaw=ry, duration=0.6)
        yield from Rt.grip(FINGER_OPEN, 0.4)
        yield from Rt.move((tgt[0], tgt[1], hi), yaw=ry, duration=0.5)

    # 3) let go of the live cables, both arms clear
    yield from ctx.both(L.move((PRESS[0], PRESS[1], hi), yaw=math.pi / 2, finger=FINGER_OPEN, duration=0.8),
                        Rt.move((0.18, -0.35, hi), yaw=ry, duration=0.8))
    _log(ctx, "released")
    yield from L.wait(1.0)
    _log(ctx, "end")


# -- shortcut variants (test only; they may read the simulator, they only have to score 0) --------------------------


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


def _cheat_script(ctx, cheat):
    """nothing: wait. nearest/longest: pull the cable nearest the robot (grey A) / with the plug sticking out
    furthest (grey C) exactly the way the demo pulls blue (positions read from the simulator: a shortcut only has to
    show that it scores 0)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    target = {"nearest": "grey_a", "longest": "grey_c"}.get(cheat, "blue")
    hi = TOP + 0.10
    over = 0.010
    ry = -math.pi / 2
    _log(ctx, "start")
    if cheat == "nothing":
        yield from L.wait(3.0)
        _log(ctx, "end")
        return
    yield from L.move((PRESS[0], PRESS[1], hi), yaw=math.pi / 2, finger=-0.0225, duration=1.2)
    yield from L.move((PRESS[0], PRESS[1], PRESS_Z), yaw=math.pi / 2, duration=0.8)
    pp = ctx.states()[target].end_plug
    yield from Rt.move((pp[0], pp[1], hi), yaw=ry, finger=-0.014, duration=1.2)
    pp = ctx.states()[target].end_plug
    yield from Rt.move((pp[0], pp[1], pp[2] + over), yaw=ry, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((pp[0], pp[1] - 0.02, TOP + 0.04), yaw=ry, duration=0.5)
    yield from Rt.move(PULL_TO[0], yaw=ry, duration=2.0)
    yield from Rt.move((PULL_TO[0][0], PULL_TO[0][1], TOP + 0.025), yaw=ry, duration=0.6)
    yield from Rt.grip(FINGER_OPEN, 0.4)
    yield from Rt.move((PULL_TO[0][0], PULL_TO[0][1], hi), yaw=ry, duration=0.5)
    _log(ctx, "pull 1")
    for k in range(5):
        g = _regrasp_point(ctx.states(), target)
        if g is None:
            print("[demo] no regrasp point", flush=True)
            break
        p, h = g
        gy = _grip_yaw(h)
        yield from Rt.move((p[0], p[1], hi), yaw=gy, finger=-0.014, duration=1.0)
        yield from Rt.move((p[0], p[1], p[2] + over), yaw=gy, duration=0.7)
        yield from Rt.grip(FINGER_CLOSED, 0.5)
        yield from Rt.move((p[0], p[1], TOP + 0.05), yaw=gy, duration=0.4)
        tgt = PULL_TO[1 + k % 2]
        yield from Rt.move(tgt, yaw=ry, duration=1.8)
        yield from Rt.move((tgt[0], tgt[1], TOP + 0.025), yaw=ry, duration=0.6)
        yield from Rt.grip(FINGER_OPEN, 0.4)
        yield from Rt.move((tgt[0], tgt[1], hi), yaw=ry, duration=0.5)
        _log(ctx, f"pull {k + 2}")
    yield from ctx.both(L.move((PRESS[0], PRESS[1], hi), yaw=math.pi / 2, finger=FINGER_OPEN, duration=0.8),
                        Rt.move((0.18, -0.35, hi), yaw=ry, duration=0.8))
    _log(ctx, "released")
    yield from L.wait(1.0)
    _log(ctx, "end")
