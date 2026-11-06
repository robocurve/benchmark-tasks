"""On the station table, in front of the right arm, stands a twin-post cleat (a pair of bitts): two grey upright
posts, 30 mm in diameter and 8 cm tall with a 42 mm flared head (10 mm thick) on top, 8 cm apart centre to centre,
on a line running left-right 32 cm in front of the robot (left post at y -6 cm, right post at y -14 cm). A 60 cm
braided rope (8 mm, dark blue) comes out of the right-hand face of a fixed grey block (the deck fitting,
5 x 6 x 4 cm) 8 cm closer to the robot than the posts, and lies straight on the table towards the right, passing
6.5 cm in front of both posts. The rope must be laid round the posts in one full figure-eight: around the outer side
of the right post, diagonally back between the posts, around the outer side of the left post (turning the other
way) and diagonally back between the posts again, crossing the first diagonal. Then the robot lets go. The right
post must be done first: following the rope from the block, its first pass must be round the right post.

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
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Make one full figure-eight with the blue rope around the two grey posts: take it around the outer "
               "side of the right post, back diagonally between the posts, around the outer side of the left post, "
               "and back diagonally between the posts again so it crosses the first diagonal. Keep the rope low on "
               "the posts, below their heads, then let go of it.")

# -- cleat: two fixed posts on a line along y -----------------------------------------------------------------
POST_X = 0.32
POST_A = np.array([POST_X, -0.06])  # left (near) post
POST_B = np.array([POST_X, -0.14])  # right (far) post, the one the line goes around first
POST_R = 0.015
POST_H = 0.08  # to the top of the head
HEAD_R, HEAD_T = 0.021, 0.010  # flared head (42 mm disc, 10 mm thick) on top of each post: keeps turns from riding off
GREY = (0.50, 0.52, 0.55)
POSTS = [o for t, p in (("a", POST_A), ("b", POST_B)) for o in (
    Cylinder(f"post_{t}", pos=(float(p[0]), float(p[1]), TOP + (POST_H - HEAD_T) / 2), radius=POST_R,
             height=POST_H - HEAD_T, color=GREY),
    Cylinder(f"head_{t}", pos=(float(p[0]), float(p[1]), TOP + POST_H - HEAD_T / 2), radius=HEAD_R, height=HEAD_T,
             color=GREY))]

# -- deck fitting (anchor block) and rope ------------------------------------------------------------------------
R = 0.004
ROPE_LEN = 0.60
ROPE_X = 0.24
ROPE_Y0 = 0.02  # rope leaves the block's right (-y) face here
BLOCK = Box("deck_block", pos=(ROPE_X, ROPE_Y0 + 0.03, TOP + 0.02), size=(0.05, 0.06, 0.04), color=(0.40, 0.42, 0.45))
ROPE = cable("rope", start=(ROPE_X, ROPE_Y0, TOP + R + 0.0005), yaw=-math.pi / 2, length=ROPE_LEN, radius=R,
             anchor_start=True, color=(0.10, 0.16, 0.45), bend_stiffness=0.01, bend_damping=0.001,
             collision_sides=0)  # round capsule colliders: the rope must slide smoothly round the posts

OBJECTS = [*POSTS, BLOCK, ROPE]

# -- scoring -------------------------------------------------------------------------------------------------------
# Cleat frame (top view): u along the cleat line from post A to post B (origin midway), v across it.
_E = (POST_B - POST_A) / np.linalg.norm(POST_B - POST_A)
_N = np.array([-_E[1], _E[0]])
_MID = (POST_A + POST_B) / 2
HALF = float(np.linalg.norm(POST_B - POST_A) / 2)  # u of post B (post A at -HALF)
OUTER = 0.045  # an outer-side pass counts if the rope crosses the cleat line within 4.5 cm of the post centre
LOW_Z = TOP + POST_H - HEAD_T - 0.010  # crossings only count with the rope centre >= 1 cm below the heads
RELEASE_DIST = 0.03  # every gripper (centre between the pads) at least this far from the rope
FULL_EVENTS = 4  # outer B, middle, outer A, middle (one complete figure-eight)


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def crossings(poly: np.ndarray) -> list[tuple[str, int]]:
    """Ordered crossings of the cleat line by the rope (from the anchored end), each (region, direction):
    region "A"/"B" = on the outer side of that post (within OUTER of its centre), "M" = between the post centres;
    direction +1/-1 = sign of the change in v. Crossings farther out, or with the rope high on the posts, are
    ignored."""
    pts = _dense(poly)
    d = pts[:, :2] - _MID
    u, v = d @ _E, d @ _N
    low = pts[:, 2] <= LOW_Z
    out = []
    for i in range(len(pts) - 1):
        if (v[i] >= 0) == (v[i + 1] >= 0) or not (low[i] and low[i + 1]):
            continue
        t = v[i] / (v[i] - v[i + 1])
        uc = u[i] + t * (u[i + 1] - u[i])
        sgn = 1 if v[i + 1] > v[i] else -1
        if abs(uc) < HALF:
            out.append(("M", sgn))
        elif HALF < uc <= HALF + OUTER:
            out.append(("B", sgn))
        elif -HALF - OUTER <= uc < -HALF:
            out.append(("A", sgn))
    return out


def _run_from(events: list[tuple[str, int]], i: int) -> int:
    """Number of crossings from events[i] on that follow the figure-eight pattern: events[i] is an outer-side pass
    of one post, then middle / outer-side passes alternate, the outer passes alternating between the posts, every
    outer pass in one direction and every middle pass in the other (= opposite turns around the two posts)."""
    reg, sgn = events[i]
    if reg == "M":
        return 0
    n, last_post = 1, reg
    for reg2, sgn2 in events[i + 1:]:
        want_mid = n % 2 == 1
        if want_mid and reg2 == "M" and sgn2 == -sgn:
            n += 1
        elif not want_mid and reg2 not in ("M", last_post) and sgn2 == sgn:
            n += 1
            last_post = reg2
        else:
            break
    return n


def figure_eight_run(events: list[tuple[str, int]]) -> int:
    """Scored run length. The right post must be done first: the FIRST crossing along the rope (from the block)
    must be the outer-side pass of the right post B, and the run is counted from that crossing only. Starting with
    the left post (or between the posts) gives 0. A round turn on one post gives at most 2; going around both posts
    the same way (an oval) gives 1."""
    if not events or events[0][0] != "B":
        return 0
    return _run_from(events, 0)


def any_start_run(events: list[tuple[str, int]]) -> int:
    """Longest figure-eight run starting at any post, ignoring which post came first (demo logs only: shows what a
    left-post-first wrap would get without the order rule)."""
    return max((_run_from(events, i) for i in range(len(events))), default=0)


def winding_deg(poly: np.ndarray, centre: np.ndarray) -> float:
    """Signed angle (deg, counter-clockwise +) the rope sweeps around a post axis in top view (for logs)."""
    a = np.unwrap(np.arctan2(poly[:, 1] - centre[1], poly[:, 0] - centre[0]))
    return float(np.degrees(a[-1] - a[0]))


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_cleat_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._cleat_hooked = True
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
    pts = _dense(objects["rope"].polyline())
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def events_done(objects) -> int:
    return figure_eight_run(crossings(objects["rope"].polyline()))


def score(objects) -> float:
    """0 unless the rope is released. n = crossings in the figure-eight run that must start with the first crossing
    along the rope, round the outer side of the RIGHT post (see figure_eight_run; left post first -> n = 0):
    n <= 2 (nothing, a drape, one post only, an oval, left post first) -> 0; 3 (round the right post, between,
    round the left post) -> 0.5; 4 or more (back between the posts: one complete figure-eight) -> 1."""
    if not released(objects):
        return 0.0
    n = events_done(objects)
    return 0.0 if n < 3 else (0.5 if n == 3 else 1.0)


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["rope"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Crossings still missing from the full figure-eight run (0-4)."""
    return float(max(0, FULL_EVENTS - events_done(objects)))


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------

RG = 0.060  # radius of the gripper's circles around a post (rope point held)
Z_LOW = TOP + 0.012  # held rope point while wrapping (fingertips just above the table)
Z_HIGH = TOP + POST_H + 0.030  # carry over the posts for the diagonals
YAW_LO, YAW_HI = -75.0, 105.0  # grip yaw window (deg) the right wrist can follow along the whole path
GRASP_OPEN = -0.014  # finger joint: pads ~28 mm apart round the 8 mm rope


def _arc(c, a0, a1, z, ccw: bool, step=15.0, r0=RG, r1=RG):
    """Points round post centre ``c`` from angle a0 to a1 (deg), radius r0 -> r1, with the grip yaw that follows
    the direction of travel (clipped to the wrist window)."""
    n = max(1, int(abs(a1 - a0) / step))
    out = []
    for k, a in enumerate(np.linspace(a0, a1, n + 1)):
        r = r0 + (r1 - r0) * k / n
        h = a + 90 if ccw else a - 90
        h = (h + 180) % 360 - 180
        out.append((c[0] + r * math.cos(math.radians(a)), c[1] + r * math.sin(math.radians(a)), z,
                    float(np.clip(h, YAW_LO, YAW_HI))))
    return out


def _up(p, z=Z_HIGH):
    return (p[0], p[1], z, p[3])


GRIP_S = 0.50  # rope arclength (from the block) the right arm holds: 10 cm from the free end


def wrap_path(variant: str = "") -> list[tuple[float, float, float, float]]:
    """Held-rope-point waypoints (world x, y, z, grip yaw deg) after the grasp. Default (figure-eight): round the
    outer side of B (counter-clockwise), over the posts to the front of A, round the outer side of A (clockwise),
    over the posts to the front of B. Cheat variants: "oneh" = twice round B only; "oval" = once round both posts
    together; "drape" = straight across the post heads; "leftfirst" = the same figure-eight but starting round the
    left post A; "tier05" = round B, between, round A, then stop (partial credit)."""
    if variant == "leftfirst":  # mirror of the default: A (clockwise), over to the front of B, B (counter-clockwise)
        a1 = _arc(POST_A, 170, -10, Z_LOW, False)
        b = _arc(POST_B, 190, 370, Z_LOW, True)
        a2 = _arc(POST_A, 200, 160, Z_LOW, False)
        return a1 + [_up(a1[-1]), _up(b[0])] + b + [_up(b[-1]), _up(a2[0])] + a2
    b1 = _arc(POST_B, 190, 370, Z_LOW, True)
    if variant == "tier05":  # stop behind-left of A, so the tail lies away from the middle
        a = _arc(POST_A, 200, 30, Z_LOW, False)
        return b1 + [_up(b1[-1]), _up(a[0])] + a
    if variant == "oneh":
        b2 = _arc(POST_B, 160, 330, Z_LOW, True)
        return b1 + [_up(b1[-1]), _up(b2[0])] + b2
    if variant == "oval":  # front of B -> outer side of B -> behind both -> outer side of A -> front
        b = _arc(POST_B, 190, 360, Z_LOW, True)
        return b + _arc(POST_A, 0, 200, Z_LOW, True)
    if variant == "drape":
        z = TOP + POST_H + 0.02
        return [(0.30, -0.02, z, -90.0), (0.34, -0.20, z, -90.0), (0.34, -0.24, TOP + POST_H + 0.005, -90.0)]
    a = _arc(POST_A, 200, -20, Z_LOW, False)
    b2 = _arc(POST_B, 160, 200, Z_LOW, True)
    return b1 + [_up(b1[-1]), _up(a[0])] + a + [_up(a[-1]), _up(b2[0])] + b2


def demo_script(ctx):
    """The right arm takes the rope 10 cm from its free end and winds it: a low circle (6 cm radius) round the
    outer side of the far post, then it lifts over the post heads and lowers in front of the near post, circles its
    outer side the other way, and lifts over again to the front of the far post, so the rope crosses between the
    posts at table level and the second diagonal lies over the first. The grasp follows the live rope. Then it
    lays the tail on the table and lets go.
    FIG8_CHEAT=none|drape|oneh|oval|hold|leftfirst runs a cheat instead (each must score 0); FIG8_CHEAT=tier05 runs
    the partial-credit case (right post, between, left post; then let go: 0.5)."""
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    cheat = os.environ.get("FIG8_CHEAT", "")
    over = 0.010  # TCP above the held rope point

    def log(tag):
        st = ctx.states()
        poly = st["rope"].polyline()
        print(f"[demo t={ctx.t:5.1f}] {tag}: ev={crossings(poly)} run={figure_eight_run(crossings(poly))} "
              f"any_start_run={any_start_run(crossings(poly))} "
              f"windA={winding_deg(poly, POST_A):.0f} windB={winding_deg(poly, POST_B):.0f} rel={released(st)} "
              f"score={score(st):.2f}", flush=True)
        if os.environ.get("DEMO_DEBUG"):
            pc = np.round(st["rope"].points * 100).astype(int)
            print("   rope cm:", " ".join(f"{x},{y},{z - 75}" for x, y, z in pc), flush=True)

    log("start")
    if cheat == "none":
        yield from Rt.wait(3.0)
        log("end")
        return
    # 1) grasp the rope at GRIP_S (live position), fingers across the rope
    st = ctx.states()
    poly = _dense(st["rope"].polyline(), 0.002)
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    k = min(int(np.searchsorted(s, GRIP_S)), len(poly) - 1)
    p, d = poly[k], poly[min(k + 1, len(poly) - 1)] - poly[max(k - 1, 0)]
    h = math.degrees(math.atan2(d[1], d[0]))
    y0 = math.radians(float(np.clip((h + 90) % 180 - 90, YAW_LO, YAW_HI)))
    yield from Rt.move((p[0], p[1], TOP + 0.06), yaw=y0, finger=GRASP_OPEN, duration=1.5)
    yield from Rt.move((p[0], p[1], p[2] + over), yaw=y0, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((p[0], p[1], Z_LOW + over + 0.01), yaw=y0, duration=0.5)
    log("grasped")
    # 2) wind
    path = wrap_path(cheat if cheat in ("oneh", "oval", "drape", "leftfirst", "tier05") else "")
    for i, (x, y, z, yaw) in enumerate(path):
        prev = path[i - 1] if i else None
        dur = 0.3 if prev is not None and abs(prev[2] - z) < 1e-6 else 0.9
        yield from Rt.move((x, y, z + over), yaw=math.radians(yaw), duration=dur)
        if z > Z_LOW + 0.01 and prev is not None and abs(prev[2] - z) > 1e-6:
            log(f"lifted at wp {i}")
    log("wrapped")
    # 3) lay the tail on the table and let go
    x, y, z, yaw = path[-1]
    yield from Rt.move((x, y, max(z - 0.008, TOP + R) + over), yaw=math.radians(yaw), duration=0.6)
    if cheat == "hold":
        yield from Rt.wait(3.0)
        log("end (held)")
        return
    yield from Rt.grip(GRASP_OPEN, 0.5)
    yield from Rt.move((x, y, TOP + POST_H + 0.06), yaw=math.radians(yaw), duration=0.8)
    yield from Rt.move((0.25, -0.35, TOP + 0.15), yaw=0.0, duration=1.0)
    log("released")
    yield from Rt.wait(1.0)
    log("end")
