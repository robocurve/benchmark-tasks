"""On the station table a black mooring bollard stands on the midline between the two arms, 32 cm in front of the
arm bases: a 50 mm round post, 6 cm tall, with a 70 mm round cap (1 cm thick) on top (total height 7 cm). In front
of it and to its right, a low grey deck eye (30 x 40 x 12 mm block) holds the fixed end of a 68 cm orange 8 mm
mooring line, which lies straight on the table towards the left (95 deg from the forward direction), passing just
in front of the bollard post (1 cm clear), and ends in a black 35 x 16 x 12 mm whipped end (a rigid plug) on the
far left. On the right, out of the left arm's reach, stands an open grey tray (inner 10 x 8 cm, walls 2 cm tall).
The line must take one full round turn (330 deg or more: a full turn with 30 deg tolerance) round the bollard post, under the cap, and its end must be
laid in the tray; then both grippers let go. Neither arm alone can do it: only the left arm reaches the line's end,
only the right arm reaches the tray.

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
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Take a round turn with the orange line on the black bollard: lead the line's black end all the way "
               "around the bollard post, under its cap, so the line goes at least one full turn around it, then lay "
               "the end in the grey tray on the right and let go of the line with both grippers.")

# -- bollard -------------------------------------------------------------------------------------------------------
BC = np.array([0.32, 0.0])  # bollard axis (x, y)
RB = 0.025  # post radius (50 mm post)
POST_H = 0.060
CAP_R, CAP_T = 0.035, 0.010  # 70 mm cap, 1 cm thick: keeps the turns from riding off the top
IRON = (0.10, 0.10, 0.11)
BOLLARD = [
    Cylinder("bollard_post", pos=(float(BC[0]), float(BC[1]), TOP + POST_H / 2), radius=RB, height=POST_H, color=IRON),
    Cylinder("bollard_cap", pos=(float(BC[0]), float(BC[1]), TOP + POST_H + CAP_T / 2), radius=CAP_R, height=CAP_T,
             color=IRON),
]

# -- line: fixed in the deck eye, whipped end on the free end --------------------------------------------------------
R = 0.004  # 8 mm line
LINE_LEN = 0.68
WHIP = Plug(size=(0.035, 0.016, 0.012), mass=0.012, color=(0.05, 0.05, 0.05))
LINE_S = np.array([0.29, -0.10])  # where the line leaves the deck eye (in front of the bollard, right of it)
LINE_YAW = math.radians(95.0)  # runs left past the near side of the post, 1 cm clear
_DIR = np.array([math.cos(LINE_YAW), math.sin(LINE_YAW)])
EYE_SIZE = (0.030, 0.040, 0.012)
_ec = LINE_S - _DIR * (EYE_SIZE[0] / 2 + 0.006)  # line leaves 6 mm in front of the eye's face
EYE = Box("deck_eye", pos=(float(_ec[0]), float(_ec[1]), TOP + EYE_SIZE[2] / 2), size=EYE_SIZE, yaw=LINE_YAW,
          color=(0.55, 0.57, 0.60))
LINE = cable("line", start=(float(LINE_S[0]), float(LINE_S[1]), TOP + WHIP.size[2] / 2 + 0.0005), yaw=LINE_YAW,
             length=LINE_LEN, radius=R, anchor_start=True, end_plug=WHIP, cable_mass=0.05,
             color=(0.95, 0.45, 0.05), bend_stiffness=0.01, bend_damping=0.001,
             collision_sides=0)  # round capsule colliders: the line must slide smoothly round the post

# -- tray on the right (four fixed walls; the table is its floor) -----------------------------------------------------
TR_X0, TR_X1 = 0.33, 0.43  # inner area
TR_Y0, TR_Y1 = -0.22, -0.14
TR_WALL, TR_H = 0.010, 0.020
TR_TOP = TOP + TR_H
TR_C = ((TR_X0 + TR_X1) / 2, (TR_Y0 + TR_Y1) / 2)
GREY = (0.62, 0.64, 0.66)
TRAY = [
    Box("tray_near", pos=(TR_X0 - TR_WALL / 2, TR_C[1], TOP + TR_H / 2),
        size=(TR_WALL, TR_Y1 - TR_Y0 + 2 * TR_WALL, TR_H), color=GREY),
    Box("tray_far", pos=(TR_X1 + TR_WALL / 2, TR_C[1], TOP + TR_H / 2),
        size=(TR_WALL, TR_Y1 - TR_Y0 + 2 * TR_WALL, TR_H), color=GREY),
    Box("tray_inner", pos=(TR_C[0], TR_Y1 + TR_WALL / 2, TOP + TR_H / 2),
        size=(TR_X1 - TR_X0, TR_WALL, TR_H), color=GREY),
    Box("tray_outer", pos=(TR_C[0], TR_Y0 - TR_WALL / 2, TOP + TR_H / 2),
        size=(TR_X1 - TR_X0, TR_WALL, TR_H), color=GREY),
]

OBJECTS = [*BOLLARD, EYE, LINE, *TRAY]

# -- grippers (score requires both to have let go) ----------------------------------------------------------------
RELEASE_DIST = 0.035  # each gripper TCP (between the finger pads) at least this far from the line and its end
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_bollard_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._bollard_hooked = True
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


# -- scoring geometry ------------------------------------------------------------------------------------------------


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


BAND = 0.03  # winding counts line centre-line points within 3 cm of the post surface (surface to line surface) ...
LOW_Z = TOP + POST_H  # ... and below the underside of the cap
TOUCH = 0.01  # the line must come within 1 cm of the post somewhere (surface to line surface)
MIN_TURN = 330.0  # deg: a full round turn, with 30 deg tolerance


def _radial(pts: np.ndarray) -> np.ndarray:
    """Gap between the post surface and the line surface (m) for each centre-line point."""
    return np.linalg.norm(pts[:, :2] - BC, axis=1) - RB - R


def winding_deg(objects) -> float:
    """Absolute winding angle (deg) of the line about the bollard axis, top view: following the line from the deck
    eye to its end, sum the signed angle steps (about the axis) between consecutive centre-line points that are both
    within 3 cm of the post surface and below the cap."""
    pts = _dense(objects["line"].polyline())
    near = (_radial(pts) <= BAND) & (pts[:, 2] <= LOW_Z)
    ang = np.arctan2(pts[:, 1] - BC[1], pts[:, 0] - BC[0])
    d = (np.diff(ang) + math.pi) % (2 * math.pi) - math.pi
    use = near[:-1] & near[1:]
    return abs(float(np.degrees(d[use].sum())))


def touches(objects) -> bool:
    """Some part of the line (below the cap) lies within 1 cm of the post."""
    pts = _dense(objects["line"].polyline())
    return bool(np.any((_radial(pts) <= TOUCH) & (pts[:, 2] <= LOW_Z)))


def released(objects) -> bool:
    c = objects["line"]
    pts = _dense(c.polyline())
    if c.end_plug is not None:
        pts = np.vstack([pts, c.end_plug[None, :], (c.end_tip + c.end_dir * WHIP.size[0])[None, :]])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def in_tray(objects) -> bool:
    """Whipped-end centre inside the tray's inner walls (x 33-43 cm, y -14 to -22 cm) and below the wall tops."""
    p = objects["line"].end_plug
    return bool(p is not None and TR_X0 <= p[0] <= TR_X1 and TR_Y0 <= p[1] <= TR_Y1 and p[2] <= TR_TOP)


def score(objects) -> float:
    """0 unless both grippers are >= 3.5 cm from the line and its end, the line touches the post (within 1 cm, below
    the cap) and winds >= 330 deg round the post (points within 3 cm of it, below the cap). Then 0.5, and 1.0 if
    the whipped end lies in the tray."""
    if not released(objects) or not touches(objects) or winding_deg(objects) < MIN_TURN:
        return 0.0
    return 1.0 if in_tray(objects) else 0.5


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score): end grasped, line touching the post, winding >= 90/180/270/
    330 deg (a full turn), end in the tray, both grippers clear."""
    c = objects["line"]
    g = grippers()
    w = winding_deg(objects)
    return {
        "end_grasped": int(c.end_plug is not None and any(float(np.linalg.norm(x - c.end_plug)) < 0.03 for x in g)),
        "touching_bollard": int(touches(objects)),
        "wind_90": int(w >= 90.0),
        "wind_180": int(w >= 180.0),
        "wind_270": int(w >= 270.0),
        "full_turn": int(w >= MIN_TURN),
        "end_in_tray": int(in_tray(objects)),
        "released": int(released(objects)),
    }


# Seeded reset variation for a later runner (the demo runs on the nominal layout and follows the live line).
VARIATION = {"cable:line": {"dx": 0.01, "dy": 0.005, "yaw": 0.04}}


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["line"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Missing winding (in turns, 0-1) plus the whipped end's distance to the tray centre (m)."""
    p = objects["line"].end_plug
    d = float(np.linalg.norm(p[:2] - np.array(TR_C))) if p is not None else 1.0
    return max(0.0, 1.0 - winding_deg(objects) / 360.0) + d


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------

RG_LOOSE = 0.075  # right arm circle
RG = 0.070  # radius of the held end's circle around the bollard axis
Z_CARRY = TOP + 0.035  # TCP while leading the end round (end body ~2 cm above the table, under the cap)


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _arc(a0: float, a1: float, yaw_fn, step: float = 15.0, r: float = RG, z: float = Z_CARRY, r1=None):
    """Waypoints (x, y, z, yaw rad) round the bollard from angle a0 to a1 (deg), radius r (-> r1)."""
    n = max(1, int(round(abs(a1 - a0) / step)))
    r1 = r if r1 is None else r1
    out = []
    for k, a in enumerate(np.linspace(a0, a1, n + 1)):
        rr = r + (r1 - r) * k / n
        out.append((BC[0] + rr * math.cos(math.radians(a)), BC[1] + rr * math.sin(math.radians(a)), z,
                    math.radians(yaw_fn(a))))
    return out


def demo_script(ctx):
    """Left arm picks the whipped end, brings it to the left side of the bollard and leads it clockwise round the far
    side to the right side, where it sets it down; the right arm picks it up there, hauls it out to the right (so the
    slack goes to the tail side of the half turn) and leads it on round the near side (over the standing part) and
    the left side to the far side again (about 1.3 turns in all), then lays it in the tray, lets go and both arms move clear. While leading, each gripper's wrist camera faces the direction of
    travel or the bollard. ROUND_CHEAT=nothing | half (only round the far side, then into the tray) | boxonly
    (end carried round the near side of the eye to the tray, no turn) | hold (full turn, end held in the tray)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("ROUND_CHEAT", "")

    def log(tag):
        st = ctx.states()
        c = st["line"]
        print(f"[demo t={ctx.t:5.1f}] {tag}: end={np.round(c.end_plug, 3)} wind={winding_deg(st):.0f} "
              f"touch={touches(st)} tray={in_tray(st)} rel={released(st)} score={score(st):.2f}", flush=True)
        if os.environ.get("DEMO_DEBUG"):
            pc = np.round(c.points * 100).astype(int)
            print("   line cm:", " ".join(f"{x},{y},{z - 75}" for x, y, z in pc), flush=True)

    def run(arm, path, dur=0.35):
        for x, y, z, yw in path:
            yield from arm.move((x, y, z), yaw=yw, duration=dur)

    log("start")
    if cheat == "nothing":
        yield from L.wait(4.0)
        log("end")
        return

    # 1) Left arm pinches the whipped end at its middle and lifts it.
    c = ctx.states()["line"]
    p = c.end_plug
    gy = _grip_yaw(math.atan2(c.end_dir[1], c.end_dir[0]))
    yield from L.move((p[0], p[1], TOP + 0.06), yaw=gy, finger=FINGER_OPEN, duration=1.5)
    yield from L.move((p[0], p[1], p[2] + 0.010), yaw=gy, duration=0.8)
    yield from L.grip(FINGER_CLOSED)
    yield from L.move((p[0], p[1], TOP + 0.05), yaw=gy, duration=0.6)
    log("left has end")

    def left_yaw(a):  # clockwise travel heading a-90, clipped to the wrist window
        return float(np.clip(a - 90.0, -90.0, 90.0))

    if cheat == "boxonly":
        # carry the end round the near side of the deck eye (away from the bollard) and set it down for the right arm
        yield from L.move((0.20, 0.20, TOP + 0.05), yaw=-math.pi / 2, duration=1.2)
        yield from L.move((0.14, 0.02, TOP + 0.05), yaw=-math.pi / 2, duration=1.0)
        yield from L.move((0.17, -0.10, TOP + 0.05), yaw=-math.pi / 2, duration=1.0)
        hand = (0.17, -0.10)
    else:
        # 2) To the left side of the bollard, then clockwise round the far side to the right side.
        first = _arc(125.0, 125.0, left_yaw)[0]
        yield from L.move((first[0], first[1], TOP + 0.05), yaw=first[3], duration=1.5)
        yield from L.move(first[:3], yaw=first[3], duration=0.6)
        yield from run(L, _arc(125.0, -45.0, left_yaw))
        hand = (BC[0] + 0.075 * math.cos(math.radians(-45)), BC[1] + 0.075 * math.sin(math.radians(-45)))
        log("left round the far side")
    # 3) Set it down for the right arm and move the left arm clear.
    yw = -math.pi / 2
    yield from L.move((hand[0], hand[1], TOP + 0.020), yaw=yw, duration=0.6)
    yield from L.grip(FINGER_OPEN, duration=0.5)
    yield from L.move((hand[0], hand[1], TOP + 0.10), yaw=yw, duration=0.6)
    yield from L.move((0.18, 0.35, TOP + 0.15), yaw=0.0, duration=1.2)
    log("set down")

    # 4) Right arm picks the end up where it lies.
    c = ctx.states()["line"]
    p = c.end_plug
    ry = _grip_yaw(math.atan2(c.end_dir[1], c.end_dir[0]))
    yield from Rt.move((p[0], p[1], TOP + 0.06), yaw=ry, finger=FINGER_OPEN, duration=1.5)
    p = ctx.states()["line"].end_plug
    yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=ry, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED)
    yield from Rt.move((p[0], p[1], Z_CARRY + 0.01), yaw=ry, duration=0.6)
    log("right has end")

    def right_yaw(a):  # same heading, unwrapped one turn up so the wrist turns smoothly through the near side
        return float(np.clip(a + 270.0, -90.0, 90.0))

    if cheat not in ("half", "boxonly"):
        # 5) On round: right side, near side (over the standing part), left side, far side.
        # first haul the end out to the right so the slack goes to the tail side of the half turn (slack cannot be
        # pulled through the turn later: friction round the post holds it), then lead it on in a 7.5 cm circle,
        # shrinking to 7 cm on the left side, where the right arm's reach ends
        yield from Rt.move((BC[0] + 0.08, BC[1] - 0.20, Z_CARRY), yaw=-math.pi / 2, duration=1.2)
        path = _arc(-90.0, -240.0, right_yaw, r=RG_LOOSE) + _arc(-240.0, -300.0, right_yaw, r=RG_LOOSE, r1=RG)[1:]
        yield from Rt.move(path[0][:3], yaw=path[0][3], duration=0.8)
        yield from run(Rt, path[1:])
        log("round turn made")
    # 6) Out to the tray, lay the end in it.
    yw = -math.pi / 2  # wrist camera looks along the way to the tray
    cur = Rt.tcp()
    yield from Rt.move((max(cur[0], 0.40), cur[1] - 0.04, TOP + 0.05), yaw=yw, duration=0.8)
    ty = TR_C[1] - 0.020  # aim a little deep: the line's pull drags the end back towards the bollard
    yield from Rt.move((TR_C[0], ty, TR_TOP + 0.045), yaw=yw, duration=2.5)
    yield from Rt.move((TR_C[0], ty, TOP + 0.022), yaw=yw, duration=0.8)
    log("end in tray (held)")
    if cheat == "hold":
        yield from Rt.wait(3.0)
        log("end (held)")
        return
    yield from Rt.grip(FINGER_OPEN, duration=0.5)
    yield from Rt.move((TR_C[0], ty, TOP + 0.12), yaw=yw, duration=0.8)
    yield from Rt.move((0.20, -0.33, TOP + 0.15), yaw=0.0, duration=1.0)
    log("released")
    yield from Rt.wait(1.5)
    log("end")
