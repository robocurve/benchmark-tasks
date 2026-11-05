"""On the station table a short electric-fence line runs left-right 30 cm in front of the robot: a corner post on the
far left (y +31 cm) and three line posts at y +15, 0 and -15 cm. Every post is a grey 25 x 25 mm square post, 18 cm
tall (it stands for a steel/timber post that grounds the fence if the live wire touches it). Each line post carries a
white offset insulator on its robot-facing side: a 19 mm standoff arm that holds an open-top U slot (14 mm wide,
10 mm deep, running left-right) 3 cm in front of the post, its floor 15 cm above the table. A white strain insulator
block on the corner post holds the start of the fence wire: a white 6 mm polyrope, 53.5 cm long, tied off at about 15 cm
height, with a yellow insulated gate handle (40 x 14 x 12 mm) on its free end. At the start the wire hangs from the
corner insulator down to the table and lies diagonally on the table in front of the posts, the handle in front of
the right arm.

The robot must hang the wire in all three insulator slots so that no part of the wire or handle touches (comes within
3 mm of) any post (3 mm to 1 cm scores 0.5), nothing touches the table (the tail and handle hang free past the third insulator), and
let go. The left arm alone reaches the left insulator, the right arm alone the right one; one arm holds the wire
under tension while the other lays it.

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

INSTRUCTION = ("This electric fence wire will be switched on later. Hang it in the white insulator slots on all "
               "three posts so that no part of the wire or its yellow handle touches a post or the table: keep it at "
               "least 1 cm from every post and let the handle hang free past the last post. Then let go of the wire.")

WHITE = (0.93, 0.93, 0.90)
GREY = (0.42, 0.44, 0.47)
YELLOW = (0.98, 0.80, 0.05)

# -- posts and insulators (all fixed) --------------------------------------------------------------------------------
POST_X = 0.30  # post centres
POST_W = 0.025
POST_H = 0.18
POST_FACE = POST_X - POST_W / 2  # robot-facing post face
LINE_YS = (0.15, 0.0, -0.15)  # line posts, left to right
CORNER_Y = 0.31
H = TOP + 0.15  # top of the slot floors (the wire line)
SLOT_X = POST_FACE - 0.030  # slot centre line (3 cm in front of the post face)
SLOT_G = 0.014  # inner gap (x)
SLOT_L = 0.020  # length along the wire (y)
WALL_T, WALL_H = 0.004, 0.010  # wall thickness, height above the floor
FLOOR_T = 0.008


def _post(tag: str, y: float) -> list:
    return [Box(f"post_{tag}", pos=(POST_X, y, TOP + POST_H / 2), size=(POST_W, POST_W, POST_H), color=GREY)]


def _insulator(tag: str, y: float) -> list:
    back = SLOT_X + SLOT_G / 2 + WALL_T  # outer face of the back wall
    fx = SLOT_G + 2 * WALL_T
    return [
        Box(f"ins_floor_{tag}", pos=(SLOT_X, y, H - FLOOR_T / 2), size=(fx, SLOT_L, FLOOR_T), color=WHITE),
        Box(f"ins_wall_front_{tag}", pos=(SLOT_X - SLOT_G / 2 - WALL_T / 2, y, H - FLOOR_T + (FLOOR_T + WALL_H) / 2),
            size=(WALL_T, SLOT_L, FLOOR_T + WALL_H), color=WHITE),
        Box(f"ins_wall_back_{tag}", pos=(SLOT_X + SLOT_G / 2 + WALL_T / 2, y, H - FLOOR_T + (FLOOR_T + WALL_H) / 2),
            size=(WALL_T, SLOT_L, FLOOR_T + WALL_H), color=WHITE),
        Box(f"ins_arm_{tag}", pos=((back + POST_FACE) / 2, y, H - FLOOR_T / 2), size=(POST_FACE - back, 0.012, FLOOR_T),
            color=WHITE),
    ]


R = 0.003
CORNER_BLOCK = 0.020  # strain insulator block (cube) on the corner post's front face
WIRE_START = (SLOT_X, CORNER_Y - CORNER_BLOCK / 2 - 0.006, H + R)
FIXED = (
    _post("corner", CORNER_Y)
    + [Box("ins_corner", pos=((SLOT_X - CORNER_BLOCK / 2 + POST_FACE) / 2, CORNER_Y, H + R),
           size=(POST_FACE - SLOT_X + CORNER_BLOCK / 2, CORNER_BLOCK, CORNER_BLOCK), color=WHITE)]
    + [p for k, y in enumerate(LINE_YS) for p in _post(str(k + 1), y) + _insulator(str(k + 1), y)]
)
POST_BOXES = {  # (lo, hi) of every post body (insulators excluded)
    tag: (np.array([POST_X - POST_W / 2, y - POST_W / 2, TOP]), np.array([POST_X + POST_W / 2, y + POST_W / 2, TOP + POST_H]))
    for tag, y in (("corner", CORNER_Y), ("1", LINE_YS[0]), ("2", LINE_YS[1]), ("3", LINE_YS[2]))
}

# -- the fence wire: start welded in front of the corner strain insulator; spawns straight at wire height pointing
#    diagonally towards the robot's right and falls, so it hangs from the corner insulator and lies on the table ------
WIRE_LEN = 0.535
WIRE_YAW = math.radians(-102.0)
HANDLE = Plug(size=(0.040, 0.014, 0.012), mass=0.010, color=YELLOW)
WIRE = cable("wire", start=WIRE_START, yaw=WIRE_YAW, length=WIRE_LEN, radius=R, anchor_start=True,
             end_plug=HANDLE, color=WHITE, bend_stiffness=0.01, bend_damping=0.001)

OBJECTS = [*FIXED, WIRE]

# -- scoring ---------------------------------------------------------------------------------------------------------
CLEAR_FULL = 0.010  # wire/handle surface at least this far from every post body for full credit
CLEAR_MIN = 0.003  # closer than this counts as touching a post -> 0
GROUND_MIN = 0.020  # every part of the wire and handle at least this far above the table
RELEASE_DIST = 0.03  # every gripper centre at least this far from the wire and handle
HANDLE_HALF = 0.5 * math.hypot(HANDLE.size[1], HANDLE.size[2])  # handle cross-section half-diagonal


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _handle_axis(c) -> np.ndarray:
    """Points along the handle's long axis (centre +- half length)."""
    if c.end_plug is None:
        return np.zeros((0, 3))
    d = c.end_dir / (np.linalg.norm(c.end_dir) + 1e-12)
    return np.array([c.end_plug + d * t for t in np.linspace(-HANDLE.size[0] / 2, HANDLE.size[0] / 2, 7)])


def in_slot(objects, k: int) -> bool:
    """Some part of the wire lies in line insulator k's slot: between the walls, over the floor, below the wall tops."""
    pts = _dense(objects["wire"].polyline())
    y = LINE_YS[k]
    ok = (np.abs(pts[:, 0] - SLOT_X) <= SLOT_G / 2) & (np.abs(pts[:, 1] - y) <= SLOT_L / 2) \
        & (pts[:, 2] >= H) & (pts[:, 2] <= H + WALL_H)
    return bool(ok.any())


def _box_dist(pts: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    d = np.maximum(np.maximum(lo - pts, pts - hi), 0.0)
    return np.linalg.norm(d, axis=1)


def post_clearance(objects) -> float:
    """Smallest gap (m) between the wire / handle surface and any post body (negative = inside)."""
    c = objects["wire"]
    wp, hp = _dense(c.polyline()), _handle_axis(c)
    gaps = []
    for lo, hi in POST_BOXES.values():
        gaps.append(float(_box_dist(wp, lo, hi).min()) - R)
        if len(hp):
            gaps.append(float(_box_dist(hp, lo, hi).min()) - HANDLE_HALF)
    return min(gaps)


def ground_clearance(objects) -> float:
    """Height (m) of the lowest part of the wire / handle above the table."""
    c = objects["wire"]
    z = float(c.polyline()[:, 2].min()) - R
    hp = _handle_axis(c)
    if len(hp):
        z = min(z, float(hp[:, 2].min()) - HANDLE_HALF)
    return z - TOP


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_fence_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._fence_hooked = True
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
    c = objects["wire"]
    pts = np.vstack([_dense(c.polyline()), _handle_axis(c)])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def score(objects) -> float:
    """0 unless: the grippers have let go, the wire lies in all three insulator slots, no part of the wire or handle
    is within 3 mm of a post body (touching), and every part is >= 2 cm above the table. Then 1.0 if the wire and
    handle are >= 1 cm from every post body, else 0.5."""
    if not np.all(np.isfinite(objects["wire"].points)):
        return 0.0  # physics blow-up: no valid state
    if not released(objects):
        return 0.0
    if not all(in_slot(objects, k) for k in range(3)):
        return 0.0
    if ground_clearance(objects) < GROUND_MIN:
        return 0.0
    gap = post_clearance(objects)
    if gap < CLEAR_MIN:
        return 0.0
    return 1.0 if gap >= CLEAR_FULL else 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["wire"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Number of empty slots plus metres short of 1 cm post clearance."""
    if not np.all(np.isfinite(objects["wire"].points)):
        return 9.0
    miss = sum(not in_slot(objects, k) for k in range(3))
    return float(miss + max(0.0, CLEAR_FULL - post_clearance(objects)))


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    if not np.all(np.isfinite(objects["wire"].points)):
        return {"slots_filled": 0.0, "posts_clear": 0.0, "off_ground": 0.0, "released": 0.0}
    return {
        "slots_filled": sum(in_slot(objects, k) for k in range(3)) / 3.0,
        "posts_clear": float(post_clearance(objects) >= CLEAR_FULL),
        "off_ground": float(ground_clearance(objects) >= GROUND_MIN),
        "released": float(released(objects)),
    }


VARIATION = {"cable:wire": {"dx": 0.0, "dy": 0.0, "yaw": 0.05}}

# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
# Right arm: takes the handle, lifts it and holds the wire end beyond the right post (wrist camera facing +y, towards
# the insulators). Left arm: takes the wire between the corner and the left insulator (wrist camera facing -y,
# towards the insulators). Both raise the wire level about 6 cm above the slot floors, the right arm pulls it straight
# (closed loop on the measured sag), both lower it into the three slots, the left lets go, the right lowers the
# handle so it hangs past the right insulator and lets go.

GRIP_S = 0.075  # arc length (m) from the corner where the left arm takes the wire
CABLE_OPEN = -0.014  # finger joint: pads ~28 mm apart round the 6 mm wire


def _heading_yaw(d) -> float:
    a = math.atan2(d[1], d[0])
    while a > math.pi / 2:
        a -= math.pi
    while a < -math.pi / 2:
        a += math.pi
    return a


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("FENCE_CHEAT", "")  # test only: none | tops | two | touch | hold
    yl, yr = -math.pi / 2, math.pi / 2

    def log(tag):
        st = ctx.states()
        c = st["wire"]
        print(f"[demo t={ctx.t:5.1f}] {tag}: slots={[in_slot(st, k) for k in range(3)]} "
              f"post_gap={post_clearance(st):.4f} ground={ground_clearance(st):.4f} rel={released(st)} "
              f"score={score(st):.2f} handle={np.round(c.end_plug, 3)}", flush=True)

    log("start")
    if cheat == "none":
        yield from L.wait(3.0)
        log("end")
        return

    # 1) right arm takes the handle at its middle and lifts it; left arm hovers near the corner
    st = ctx.states()
    c = st["wire"]
    hp, hy = c.end_plug, _heading_yaw(c.end_dir)
    yield from ctx.both(Rt.move((hp[0], hp[1], TOP + 0.06), yaw=hy, finger=FINGER_OPEN, duration=1.5),
                        L.move((SLOT_X - 0.03, 0.22, H + 0.06), yaw=yl, finger=FINGER_OPEN, duration=1.5))
    yield from Rt.move((hp[0], hp[1], hp[2] + 0.008), yaw=hy, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((hp[0], hp[1], H + 0.02), yaw=hy, duration=1.0)
    yield from Rt.move((SLOT_X, -0.21, H + 0.04), yaw=yr, duration=1.2)
    yield from Rt.wait(0.5)
    log("handle up")

    # 2) left arm takes the wire GRIP_S from the corner (closed loop on the live wire)
    st = ctx.states()
    c = st["wire"]
    i = int(round(GRIP_S / c.seg_len - 0.5))
    p = c.points[i]
    gy = _heading_yaw(c.points[i + 1] - c.points[i - 1])
    gy = yl if abs(gy - yl) < 0.6 or abs(gy + yl) < 0.6 else gy
    yield from L.move((p[0], p[1], p[2] + 0.05), yaw=yl, finger=CABLE_OPEN, duration=1.0)
    st = ctx.states()
    p = st["wire"].points[i]
    yield from L.move((p[0], p[1], p[2] + 0.010), yaw=yl, duration=0.8)
    yield from L.grip(FINGER_CLOSED, 0.5)
    log(f"left grasped seg {i} at {np.round(p, 3)}")

    # 3) raise the wire level about 6 cm above the slot floors, in line with them; right pulls it straight
    lay_hi = H + R + 0.060 + 0.010
    gl_y = 0.225
    yr_pos = -0.20
    if cheat == "tops":  # drape the wire over the post tops instead of using the insulators
        top_z = TOP + POST_H + R + 0.012
        yield from ctx.both(L.move((POST_X, gl_y, top_z + 0.03), yaw=yl, duration=1.5),
                            Rt.move((POST_X, yr_pos, top_z + 0.03), yaw=yr, duration=1.5))
        yield from ctx.both(L.move((POST_X, gl_y, top_z), yaw=yl, duration=1.0),
                            Rt.move((POST_X, yr_pos - 0.02, top_z), yaw=yr, duration=1.0))
        yield from ctx.both(L.grip(FINGER_OPEN), Rt.grip(FINGER_OPEN))
        yield from ctx.both(L.move((POST_X - 0.10, gl_y, top_z + 0.04), yaw=yl, duration=1.0),
                            Rt.move((POST_X - 0.10, yr_pos, top_z + 0.04), yaw=yr, duration=1.0))
        yield from L.wait(2.0)
        log("end (draped over post tops)")
        return
    front = SLOT_X - 0.04
    yield from ctx.both(L.move((front, gl_y, lay_hi), yaw=yl, duration=1.2),
                        Rt.move((front, yr_pos, lay_hi), yaw=yr, duration=1.2))
    yield from L.wait(0.5)
    for _ in range(6):
        st = ctx.states()
        pts = st["wire"].points
        mid = (pts[:, 1] < gl_y - 0.02) & (pts[:, 1] > yr_pos + 0.03)
        sag = float(lay_hi - 0.010 - pts[mid, 2].min()) if mid.any() else 0.0
        log(f"tension sag={sag:.4f} yr={yr_pos:.3f}")
        if sag < 0.008:
            break
        yr_pos = max(yr_pos - min(0.02, 0.5 * sag), -0.262)
        yield from Rt.move((front, yr_pos, lay_hi), yaw=yr, duration=0.6)
        yield from Rt.wait(0.4)
    yield from ctx.both(L.move((SLOT_X, gl_y, lay_hi), yaw=yl, duration=1.0),
                        Rt.move((SLOT_X, yr_pos, lay_hi), yaw=yr, duration=1.0))
    yield from L.wait(0.4)
    log("over the slots")

    # 4) lower into the slots
    lay_lo = H + R + 0.002 + 0.010
    r_lay = (SLOT_X, yr_pos, lay_lo)
    yield from ctx.both(L.move((SLOT_X, gl_y, lay_lo), yaw=yl, duration=1.2),
                        Rt.move(r_lay, yaw=yr, duration=1.2))
    yield from L.wait(0.6)
    log("lowered")
    yield from L.grip(CABLE_OPEN, 0.5)
    yield from L.move((SLOT_X - 0.04, gl_y, lay_lo + 0.05), yaw=yl, duration=0.8)
    yield from L.move((0.15, 0.22, TOP + 0.20), yaw=yl, duration=1.0)
    log("left released")

    if cheat == "touch":  # park the handle on the right insulator's standoff arm, against the post
        park = (SLOT_X + SLOT_G / 2 + WALL_T + POST_FACE) / 2 + 0.001  # between the back wall and the post face
        yield from Rt.move((SLOT_X, -0.24, H + 0.055), yaw=yr, duration=1.0)
        yield from Rt.move((park, LINE_YS[2] - 0.004, H + 0.055), yaw=yr, duration=1.2)
        yield from Rt.move((park, LINE_YS[2] - 0.004, H + 0.045), yaw=yr, duration=0.6)
        yield from Rt.wait(0.4)
        yield from Rt.grip(FINGER_OPEN, 0.5)
        yield from Rt.move((park - 0.06, -0.22, H + 0.08), yaw=yr, duration=1.0)
        yield from Rt.move((0.15, -0.22, TOP + 0.20), yaw=yr, duration=1.0)
        yield from L.wait(2.0)
        log("end (handle parked against the right post)")
        return

    # 5) right lowers the handle so it hangs past the right insulator, lets go and backs away
    hang_y = -0.185
    hang_x = SLOT_X
    yield from Rt.move((hang_x, hang_y, H - 0.05), yaw=yr, duration=1.2)
    yield from Rt.wait(0.6)
    log("handle hanging (held)")
    if cheat == "two":  # lift the wire out of the right insulator and let it hang in front of it (2 of 3 used)
        yield from Rt.move((SLOT_X - 0.04, hang_y - 0.04, H + 0.02), yaw=yr, duration=1.0)
        yield from Rt.move((SLOT_X - 0.04, hang_y, H - 0.05), yaw=yr, duration=1.0)
        yield from Rt.wait(0.4)
        hang_x = SLOT_X - 0.04
    if cheat == "hold":
        yield from Rt.wait(4.0)
        log("end (held)")
        return
    yield from Rt.grip(FINGER_OPEN, 0.5)
    yield from Rt.move((hang_x - 0.07, hang_y - 0.02, H - 0.05), yaw=yr, duration=0.8)
    yield from Rt.move((0.15, -0.22, TOP + 0.20), yaw=yr, duration=1.0)
    log("released")
    yield from L.wait(2.0)
    log("end")
