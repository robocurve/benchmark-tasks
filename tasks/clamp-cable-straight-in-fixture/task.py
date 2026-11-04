"""Clamp a cable straight in a fixture.

On the station table a grey fixture end block (40 x 40 x 40 mm) stands in front of the left arm at x 30 cm, y +31.5 cm;
a black 6 mm cable, 50 cm long, leaves its -y face through a grommet (held at the exit point 4 mm above the table,
free to swing sideways) and ends in a small black plug (30 x 14 x 10 mm). The fixture line runs straight from the
cable exit along -y at x 30 cm. On that line, 24 cm from the exit (y +4.9 cm), a grey U-seat: two fixed blocks (10 x 40 x 16 mm) on either side of the line with a 34 mm gap.
Blue tick marks (5 mm wide strips, 30 mm long, on both sides of the line, 8 mm clear of it) mark the line at
y +25 cm (near mark) and y -10 cm (far mark, 39 cm from the exit, beyond the seat). A free orange aluminium clamp block
(30 x 30 x 20 mm, 50 g) lies on the table at x 18 cm, y +15 cm. The cable starts straight but swung 17 degrees off the
line towards the robot, so its plug lies at about x 15 cm, y -20 cm and the cable passes beside (not through) the U.

Goal: pull the cable onto the line and taut (straight between the marks), seat the clamp block in the U on top of the
cable, then let go of both the block and the cable.

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

INSTRUCTION = ("Pull the black cable straight and taut along the fixture line marked by the blue ticks, so it runs "
               "through the grey U-seat, then place the orange clamp block in the U on top of the cable. Let go of "
               "both the block and the cable.")

GREY = (0.55, 0.57, 0.60)
DARK = (0.06, 0.06, 0.06)
BLUE = (0.10, 0.30, 0.90)
ORANGE = (0.95, 0.40, 0.05)

# -- fixture line (along -y at x = LX), anchor block, U-seat, tick marks -------------------------------------------
LX = 0.30
R = 0.003  # 6 mm cable
CORD_LEN = 0.50
ANCHOR_BOX = 0.040
ANCHOR_FACE_Y = 0.295
EXIT_Y = ANCHOR_FACE_Y - 0.006  # cable start 6 mm in front of the block face
SEAT_ALONG = 0.24
SEAT_Y = EXIT_Y - SEAT_ALONG
GAP = 0.034  # inner gap of the U (block is 30 mm)
WALL_T, WALL_L, WALL_H = 0.010, 0.040, 0.016
MARK_NEAR_Y, MARK_FAR_Y = 0.25, -0.10
MARK_IN, MARK_LEN, MARK_W, MARK_T = 0.008, 0.030, 0.005, 0.0005

FIXTURE = [Box("anchor_block", pos=(LX, ANCHOR_FACE_Y + ANCHOR_BOX / 2, TOP + ANCHOR_BOX / 2),
               size=(ANCHOR_BOX, ANCHOR_BOX, ANCHOR_BOX), color=GREY)]
for _tag, _s in (("near", -1), ("far", 1)):
    FIXTURE.append(Box(f"seat_wall_{_tag}", pos=(LX + _s * (GAP / 2 + WALL_T / 2), SEAT_Y, TOP + WALL_H / 2),
                       size=(WALL_T, WALL_L, WALL_H), color=GREY))
for _mt, _my in (("near", MARK_NEAR_Y), ("far", MARK_FAR_Y)):
    for _side, _s in (("a", -1), ("b", 1)):
        FIXTURE.append(Box(f"mark_{_mt}_{_side}", pos=(LX + _s * (MARK_IN + MARK_LEN / 2), _my, TOP + MARK_T / 2),
                           size=(MARK_LEN, MARK_W, MARK_T), color=BLUE))

# -- cable: anchored at the exit, starts straight but swung off the line towards the robot -------------------------
SWING = math.radians(17.0)
CORD_YAW = -math.pi / 2 - SWING
PLUG = Plug(size=(0.030, 0.014, 0.010), mass=0.012, color=DARK)
CORD = cable("cord", start=(LX, EXIT_Y, TOP + R + 0.001), yaw=CORD_YAW, length=CORD_LEN, radius=R,
             anchor_start=True, end_plug=PLUG, color=(0.03, 0.03, 0.03), bend_stiffness=0.01, bend_damping=0.001)



def _install_exit_pivot() -> None:
    """Replace the kit's rigid world weld of the cable start by a pin about the vertical at the exit point: the cable
    leaves the fixture block through a round grommet, held in place but free to swing sideways. (A rigid weld would
    keep the first 2 cm segment at the 17 degree start angle, so the cable could never lie on the line.)"""
    orig = cablekit._spawn_cable
    if getattr(orig, "_exit_pivot", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CORD.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdPhysics

        stage = get_current_stage()
        stage.RemovePrim(f"{prim_path}/anchor")
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/exit_pivot")
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, CORD.start)))
        j.CreateLocalRot0Attr(Gf.Quatf(math.cos(CORD_YAW / 2), 0.0, 0.0, math.sin(CORD_YAW / 2)))
        j.CreateLocalPos1Attr(Gf.Vec3f(-CORD.seg_len / 2, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateAxisAttr("Z")
        j.CreateExcludeFromArticulationAttr(True)
        return prim

    spawn._exit_pivot = True
    cablekit._spawn_cable = spawn


_install_exit_pivot()

# -- free clamp block -------------------------------------------------------------------------------------------------
BLK = (0.030, 0.030, 0.020)
BLOCK_XY = (0.18, 0.15)
CLAMP = Box("clamp", pos=(BLOCK_XY[0], BLOCK_XY[1], TOP + BLK[2] / 2 + 0.0005), size=BLK, mass=0.050, color=ORANGE)

OBJECTS = [*FIXTURE, CORD, CLAMP]

# -- scoring ---------------------------------------------------------------------------------------------------------
DEV_FULL, DEV_MAX = 0.005, 0.015  # max deviation from the line between the marks: full credit / zero
SEAT_GAP = 0.006  # block bottom at most this far above the cable top under it
RELEASE_DIST = 0.03


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def deviation(objects) -> float:
    """Max horizontal distance (m) of the cable centreline from the fixture line, from the cable exit to the far
    mark; inf if the cable does not reach past the far mark."""
    pts = _dense(objects["cord"].polyline())
    if pts[:, 1].min() > MARK_FAR_Y:
        return float("inf")
    sel = pts[pts[:, 1] >= MARK_FAR_Y]
    return float(np.abs(sel[:, 0] - LX).max())


def seated(objects) -> bool:
    """Block centre inside the U (between the walls, within their length), the cable passes under the block
    footprint, and the block rests on it (bottom no more than 6 mm above the cable top, judged from the centre)."""
    b = objects["clamp"]
    c = b.pos
    if abs(c[0] - LX) > GAP / 2 or abs(c[1] - SEAT_Y) > WALL_L / 2:
        return False
    pts = _dense(objects["cord"].polyline(), 0.002)
    m = 0.003
    under = (pts[:, 0] >= b.low[0] + m) & (pts[:, 0] <= b.high[0] - m) & \
            (pts[:, 1] >= b.low[1] + m) & (pts[:, 1] <= b.high[1] - m) & (pts[:, 2] < c[2])
    if not under.any():
        return False
    cable_top = float(pts[under, 2].max()) + R
    return bool(c[2] - BLK[2] / 2 <= cable_top + SEAT_GAP)


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_clamp_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._clamp_hooked = True
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
    """Every gripper TCP at least 3 cm from the cable (and plug) and from the clamp block."""
    c = objects["cord"]
    pts = np.vstack([_dense(c.polyline())] + [p[None] for p in (c.start_plug, c.end_plug) if p is not None])
    b = objects["clamp"]
    for g in grippers():
        if float(np.linalg.norm(pts - g, axis=1).min()) < RELEASE_DIST:
            return False
        if float(np.linalg.norm(g - np.clip(g, b.low, b.high))) < RELEASE_DIST:
            return False
    return True


def score(objects) -> float:
    """0 unless both grippers have let go, the clamp block is seated in the U on top of the cable, and the cable
    deviates at most 15 mm from the fixture line between the exit and the far mark. Then 0.5 + 0.5 x straightness:
    deviation <= 5 mm -> 1.0, linear down to 0.5 at 15 mm."""
    if not released(objects) or not seated(objects):
        return 0.0
    d = deviation(objects)
    if not d <= DEV_MAX:
        return 0.0
    return float(0.5 + 0.5 * np.clip((DEV_MAX - d) / (DEV_MAX - DEV_FULL), 0.0, 1.0))


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["clamp"].resting(max_speed=0.01) \
        and objects["cord"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Deviation from the line (m) once the block is seated, else 1 m."""
    if not seated(objects):
        return 1.0
    return min(1.0, deviation(objects))


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------

PAD = 0.010  # TCP above the grasped part's centre
HI = TOP + 0.09
LIFT_L, LIFT_R = TOP + 0.06, TOP + 0.06  # heights the cable is carried at (clear of the 16 mm U walls)
GRASP_Y = 0.10  # left arm takes the cable at the point nearest this y (about 19 cm from the exit)
GRASP_OPEN = -0.014  # finger joint: pads ~28 mm apart round the 6 mm cable
STRAIGHT_PLUG_Y = EXIT_Y - CORD_LEN - PLUG.size[0] / 2  # plug centre when the cable lies straight on the line


def _log(ctx, tag):
    st = ctx.states()
    c, b = st["cord"], st["clamp"]
    print(f"[demo t={ctx.t:5.1f}] {tag}: dev={deviation(st) * 1000:.1f}mm seated={seated(st)} rel={released(st)} "
          f"score={score(st):.3f} block={np.round(b.pos, 4)} plug={np.round(c.end_plug, 3)}", flush=True)
    if os.environ.get("CLAMP_DEBUG"):
        pts = c.points
        print("   cable x,y,z(mm above top):", " ".join(f"({p[0]:.3f},{p[1]:.3f},{(p[2] - TOP) * 1000:.0f})"
                                                     for p in pts[::2]), flush=True)


def demo_script(ctx):
    """Left arm takes the cable 19 cm from the exit, right arm takes the plug; both lift the cable over the U walls,
    carry it onto the fixture line and lay it down, so it drops into the U. The left arm sets the exit-side part
    straight (it puts its grasp point exactly 19 cm along the line), then the right arm pulls the rest taut along the
    line (closed loop on the measured deviation) and keeps holding. The left arm lets go of the cable, picks the
    clamp block, lowers it into the U onto the cable and lets go; then the right arm lets go of the plug.

    CLAMP_CHEAT (test only): none (do nothing) | slack (lay the cable through the U with 3 cm of slack towards the
    plug, never pull it taut, then clamp) | loose (as slack, with 3 cm more slack near the exit) | noblock (pull it
    straight, let go, no block) | hold (pull straight and clamp, but keep holding the block)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("CLAMP_CHEAT", "")
    _log(ctx, "start")
    if cheat == "none":
        yield from L.wait(5.0)
        _log(ctx, "end")
        return
    st = ctx.states()
    c = st["cord"]
    k = int(np.argmin(np.abs(c.points[:, 1] - GRASP_Y)))
    s_k = (k + 0.5) * c.seg_len  # arclength of the left grasp point from the exit
    q = c.points[k]
    p = c.end_plug
    y0 = CORD_YAW + math.pi  # cable heading normalised into [-pi/2, pi/2]
    ry = math.pi / 2  # on the line (heading -pi/2)
    # 1) left arm over the cable point, right arm over the plug; grasp both
    yield from ctx.both(L.move((q[0], q[1], HI), yaw=y0, finger=GRASP_OPEN, duration=1.5),
                        Rt.move((p[0], p[1], HI), yaw=y0, finger=FINGER_OPEN, duration=1.5))
    st = ctx.states()
    q, p = st["cord"].points[k], st["cord"].end_plug
    yield from ctx.both(L.move((q[0], q[1], q[2] + PAD), yaw=y0, duration=1.0),
                        Rt.move((p[0], p[1], p[2] + PAD), yaw=y0, duration=1.0))
    yield from ctx.both(L.grip(FINGER_CLOSED), Rt.grip(FINGER_CLOSED))
    yield from ctx.both(L.move((q[0], q[1], LIFT_L), yaw=y0, duration=0.8),
                        Rt.move((p[0], p[1], LIFT_R), yaw=y0, duration=0.8))
    _log(ctx, "cable lifted")
    # 2) carry the lifted cable over the line (it passes above the U walls) and lay it down
    yl = EXIT_Y - s_k - 0.004  # 4 mm past straight: the exit-side part is pulled taut (any slack bows it)
    if cheat == "loose":
        yl += 0.03  # 3 cm of slack between exit and grasp point
        tgt = (LX, -0.17)  # and ~5 cm of slack towards the plug
    elif cheat == "slack":
        tgt = (LX, STRAIGHT_PLUG_Y + 0.03)  # laid through the U with 3 cm of slack towards the plug, never pulled
    else:
        tgt = (LX, STRAIGHT_PLUG_Y + 0.008)  # nearly taut between the hands: no sag/swing
    yield from ctx.both(L.move((LX, yl, LIFT_L), yaw=ry, duration=2.0),
                        Rt.move((tgt[0], tgt[1], LIFT_R), yaw=ry, duration=2.0))
    yield from L.wait(0.8)
    _log(ctx, "over the line")
    plug_z = TOP + PLUG.size[2] / 2 + PAD
    yield from ctx.both(L.move((LX, yl, TOP + R + PAD), yaw=ry, duration=1.2),
                        Rt.move((tgt[0], tgt[1], plug_z), yaw=ry, duration=1.2))
    yield from L.wait(0.3)
    _log(ctx, "laid on line")
    # 3) closed loop: the right arm pulls along the line until the cable is straight
    py, px = tgt[1], tgt[0]
    if cheat not in ("slack", "loose"):
        for _ in range(6):
            d = deviation(ctx.states())
            if d < 0.002:
                break
            py = max(py - 0.010, STRAIGHT_PLUG_Y - 0.004)
            yield from Rt.move((px, py, plug_z), yaw=ry, duration=0.7)
            yield from Rt.wait(0.3)
            _log(ctx, f"pull y={py:.3f}")
    # 4) left arm lets go of the cable, fetches the clamp block
    yield from L.grip(FINGER_OPEN, 0.5)
    yield from L.move((LX, yl, HI), yaw=ry, duration=0.6)
    _log(ctx, "left let go of the cable")
    if cheat not in ("slack", "loose") and deviation(ctx.states()) > 0.003:  # pull the whole length once more from the plug
        py = STRAIGHT_PLUG_Y - 0.008
        yield from Rt.move((px, py, plug_z), yaw=ry, duration=0.8)
        yield from Rt.wait(0.3)
        _log(ctx, f"final pull y={py:.3f}")
    if cheat != "noblock":
        b = ctx.states()["clamp"].pos
        yield from L.move((b[0], b[1], HI), yaw=0.0, duration=1.3)
        b = ctx.states()["clamp"].pos
        yield from L.move((b[0], b[1], b[2] + PAD), yaw=0.0, duration=0.9)
        yield from L.grip(FINGER_CLOSED)
        dxy = ctx.states()["clamp"].pos[:2] - np.array(b[:2])  # block offset from the TCP after closing
        yield from L.move((b[0], b[1], HI), yaw=0.0, duration=0.8)
        # 5) seat it in the U on the cable and let go
        tx, ty = LX - dxy[0], SEAT_Y - dxy[1]
        place_z = TOP + 2 * R + BLK[2] / 2 + 0.003 + PAD
        yield from L.move((tx, ty, HI), yaw=0.0, duration=1.5)
        yield from L.move((tx, ty, place_z + 0.02), yaw=0.0, duration=0.8)
        yield from L.move((tx, ty, place_z), yaw=0.0, duration=0.6)
        if cheat == "hold":
            yield from Rt.grip(FINGER_OPEN)
            yield from Rt.move((px - 0.06, py, TOP + 0.08), yaw=ry, duration=1.0)
            _log(ctx, "holding the block")
            yield from L.wait(3.0)
            _log(ctx, "end (held)")
            return
        yield from L.grip(FINGER_OPEN)
        yield from L.move((tx, ty, HI), yaw=0.0, duration=0.8)
        yield from L.move((0.20, 0.20, HI), yaw=0.0, duration=0.8)
        _log(ctx, "block placed")
    # 6) right arm lets go of the plug and moves clear
    yield from Rt.grip(FINGER_OPEN)
    yield from Rt.move((px, py, TOP + 0.08), yaw=ry, duration=0.6)
    yield from Rt.move((px - 0.08, py + 0.03, TOP + 0.08), yaw=ry, duration=0.8)
    _log(ctx, "released")
    yield from L.wait(2.0)
    _log(ctx, "end")
