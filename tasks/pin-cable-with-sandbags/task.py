"""Pin a cable with sandbags at the marks.

On the station table a grey stage box (80 x 60 x 60 mm) stands in front of the left arm, centre x 34.5 cm,
y +36 cm. A black 6 mm cable, 55 cm long, leaves its -y face through a grommet (held at the exit point, free to swing
sideways) and ends in a black plug body (35 x 16 x 12 mm). A taped cable lane runs straight from the exit along -y at
x 34.5 cm: two yellow tape strips (10 mm wide, 61 cm long) with a 40 mm clear lane between them. Three red cross marks
(6 mm wide, 11 cm long, across the lane) lie 15, 30 and 45 cm from the exit (y +17.4, +2.4, -12.6 cm). The cable starts
straight but swung 22 degrees off the lane towards the robot (plug at about x 13, y -21 cm), so it is not in the lane.
Three free olive-drab sandbags (50 x 40 x 25 mm, 80 g) wait off to the side: two in front of the left arm (x 16, y +25 and
y +17 cm), one beyond the far side of the lane near its end (x 45, y -26 cm).

Goal: lay the cable straight in the lane, put one sandbag on the cable at each red mark, then let go of everything.

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

INSTRUCTION = ("Lay the black cable straight inside the yellow taped lane, then pin it with the sandbags: put one "
               "sandbag on top of the cable at each of the three red marks. Let go of the cable and the bags.")

DARK = (0.05, 0.05, 0.05)
BOX_GREY = (0.48, 0.49, 0.52)
YELLOW = (0.95, 0.80, 0.05)
RED = (0.85, 0.08, 0.06)
TAN = (0.30, 0.34, 0.16)  # olive-drab canvas

# -- lane (along -y at x = LX), stage box, tape, marks --------------------------------------------------------------
LX = 0.345
R = 0.003  # 6 mm cable
CORD_LEN = 0.55
BOX_SIZE = (0.080, 0.060, 0.060)
BOX_FACE_Y = 0.330
EXIT_Y = BOX_FACE_Y - 0.006  # cable start 6 mm in front of the box face
LANE_HALF = 0.020  # clear lane 40 mm wide
TAPE_W, TAPE_T = 0.010, 0.0003
LANE_LEN = 0.61
LANE_END_Y = BOX_FACE_Y - LANE_LEN
MARK_S = (0.15, 0.30, 0.45)  # along the lane from the exit
MARK_Y = tuple(EXIT_Y - s for s in MARK_S)
MARK_W, MARK_LEN, MARK_T = 0.006, 0.110, 0.0004

FIXED = [Box("stage_box", pos=(LX, BOX_FACE_Y + BOX_SIZE[1] / 2, TOP + BOX_SIZE[2] / 2), size=BOX_SIZE,
             color=BOX_GREY)]
for _tag, _s in (("near", -1), ("far", 1)):
    FIXED.append(Box(f"lane_tape_{_tag}", pos=(LX + _s * (LANE_HALF + TAPE_W / 2), BOX_FACE_Y - LANE_LEN / 2,
                                               TOP + TAPE_T / 2), size=(TAPE_W, LANE_LEN, TAPE_T), color=YELLOW))
for _i, _my in enumerate(MARK_Y):
    FIXED.append(Box(f"mark_{_i + 1}", pos=(LX, _my, TOP + MARK_T / 2), size=(MARK_LEN, MARK_W, MARK_T), color=RED))

# -- cable: pivoted at the exit, starts straight but swung off the lane towards the robot ---------------------------
SWING = math.radians(22.0)
CORD_YAW = -math.pi / 2 - SWING
PLUG = Plug(size=(0.035, 0.016, 0.012), mass=0.015, color=DARK)
CORD = cable("cord", start=(LX, EXIT_Y, TOP + R + 0.0015), yaw=CORD_YAW, length=CORD_LEN, radius=R,
             anchor_start=True, end_plug=PLUG, color=(0.03, 0.03, 0.03), bend_stiffness=0.01, bend_damping=0.001)


def _install_exit_pivot() -> None:
    """Replace the kit's rigid world weld of the cable start by a pin about the vertical at the exit point: the cable
    leaves the stage box through a grommet, held in place but free to swing sideways. (A rigid weld would keep the
    first 2 cm segment at the 22 degree start angle, so the cable could never lie straight in the lane.)"""
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

# -- free sandbags --------------------------------------------------------------------------------------------------
BAG = (0.050, 0.040, 0.025)
BAG_MASS = 0.080
BAG_XY = ((0.16, 0.25), (0.16, 0.17), (0.45, -0.26))
BAGS = [Box(f"bag_{i + 1}", pos=(x, y, TOP + BAG[2] / 2 + 0.0005), size=BAG, mass=BAG_MASS, color=TAN)
        for i, (x, y) in enumerate(BAG_XY)]
BAG_NAMES = [b.name for b in BAGS]

OBJECTS = [*FIXED, CORD, *BAGS]

# -- scoring ---------------------------------------------------------------------------------------------------------
LANE_FRAC_MIN = 0.90  # share of the cable length that must lie inside the lane
MARK_TOL = 0.020  # bag centre within 2 cm of the mark, measured along the lane
INSET = 0.005  # cable must pass under the bag at least 5 mm in from its edges
LIFT_MIN, LIFT_MAX = 0.002, 0.012  # bag bottom (at its centre) lifted by the cable: 2..12 mm above the table
RELEASE_DIST = 0.035


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def lane_fraction(objects) -> float:
    """Share of the cable centreline (exit to cable end, plug excluded) inside the lane: within 20 mm of the lane
    centre line sideways and between the box face and the lane end."""
    pts = _dense(objects["cord"].polyline(), 0.002)
    inside = (np.abs(pts[:, 0] - LX) <= LANE_HALF) & (pts[:, 1] <= BOX_FACE_Y) & (pts[:, 1] >= LANE_END_Y)
    return float(inside.mean())


def _rot(q_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in q_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def bag_on_cable(objects, name: str) -> tuple[bool, float]:
    """(rests on the cable, lift in m). The cable centreline passes under the bag's footprint at least 5 mm in from
    its edges, and the middle of the bag's bottom face is lifted 2..12 mm above the table (it lies on the 6 mm
    cable, not flat on the table beside it and not on another bag)."""
    b = objects[name]
    rm = _rot(b.quat_xyzw)
    up = int(np.argmax(np.abs(rm[2, :])))  # bag axis closest to vertical
    half = np.array(BAG) / 2
    bottom = np.asarray(b.pos) - np.sign(rm[2, up]) * rm[:, up] * half[up]
    lift = float(bottom[2] - TOP)
    pts = _dense(objects["cord"].polyline(), 0.002)
    local = (pts - np.asarray(b.pos)) @ rm  # cable points in the bag frame
    side = [a for a in range(3) if a != up]
    under = np.ones(len(pts), bool)
    for a in side:
        under &= np.abs(local[:, a]) <= half[a] - INSET
    under &= local[:, up] * np.sign(rm[2, up]) < 0  # below the bag centre
    return bool(under.any() and LIFT_MIN <= lift <= LIFT_MAX), lift


def pinned_marks(objects) -> list[int]:
    """Marks (0-based) that have a sandbag resting on the cable with its centre within 2 cm of the mark along the
    lane; each bag counts for one mark only."""
    used, out = set(), []
    for m, my in enumerate(MARK_Y):
        for n in BAG_NAMES:
            if n in used:
                continue
            if abs(float(objects[n].pos[1]) - my) <= MARK_TOL and bag_on_cable(objects, n)[0]:
                used.add(n)
                out.append(m)
                break
    return out


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_sandbag_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._sandbag_hooked = True
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
    """Every gripper TCP at least 3.5 cm from the cable (and plug) and from every sandbag."""
    c = objects["cord"]
    pts = np.vstack([_dense(c.polyline())] + [p[None] for p in (c.start_plug, c.end_plug) if p is not None])
    for g in grippers():
        if float(np.linalg.norm(pts - g, axis=1).min()) < RELEASE_DIST:
            return False
        for n in BAG_NAMES:
            b = objects[n]
            if float(np.linalg.norm(g - np.clip(g, b.low, b.high))) < RELEASE_DIST:
                return False
    return True


def score(objects) -> float:
    """0 if a gripper is within 3.5 cm of the cable or a bag, or under 90 % of the cable length lies inside the lane.
    Otherwise 1/3 per red mark with a sandbag resting on the cable within 2 cm of it."""
    if not released(objects) or lane_fraction(objects) < LANE_FRAC_MIN:
        return 0.0
    return len(pinned_marks(objects)) / 3.0


def success(objects) -> bool:
    return score(objects) >= 0.999 and all(objects[n].resting(max_speed=0.01) for n in BAG_NAMES) \
        and objects["cord"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Share of the cable outside the lane plus 0.3 per unpinned mark."""
    return float((1.0 - lane_fraction(objects)) + 0.3 * (3 - len(pinned_marks(objects))))


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------

PAD = 0.010  # TCP above the grasped part's centre
HI = TOP + 0.09
LIFT = TOP + 0.06  # height the cable is carried at
GRASP_Y = 0.10  # left arm takes the cable at the point nearest this y (about 24 cm from the exit)
GRASP_OPEN = -0.014  # finger joint: pads ~28 mm apart round the 6 mm cable
BAG3_YAW = math.pi / 2  # right arm carries bag 3 from beyond the lane end towards +y: camera faces its mark
STRAIGHT_PLUG_Y = EXIT_Y - CORD_LEN - PLUG.size[0] / 2  # plug centre when the cable lies straight in the lane


def deviation(objects) -> float:
    """Max sideways distance (m) of the cable centreline from the lane centre line."""
    pts = _dense(objects["cord"].polyline())
    return float(np.abs(pts[:, 0] - LX).max())


def _log(ctx, tag):
    st = ctx.states()
    bags = " ".join(f"{n}={np.round(st[n].pos, 3)} lift={bag_on_cable(st, n)[1] * 1000:.1f}mm" for n in BAG_NAMES)
    print(f"[demo t={ctx.t:5.1f}] {tag}: dev={deviation(st) * 1000:.1f}mm lane={lane_fraction(st):.2f} "
          f"pinned={pinned_marks(st)} rel={released(st)} score={score(st):.3f} plug={np.round(st['cord'].end_plug, 3)}"
          f"\n    {bags}", flush=True)


def _place_bag(ctx, arm, name, target, cheat, yaw=0.0):
    """Pick sandbag `name` and set it down centred over `target` (x, y) on the cable, then let go and lift clear
    (unless cheat == 'hold'). yaw 0 closes the fingers across its 40 mm side (wrist camera facing +x), yaw pi/2
    across its 50 mm side (camera facing +y): pick the one whose camera faces the way the bag is carried."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    b = ctx.states()[name].pos
    yield from arm.move((b[0], b[1], HI), yaw=yaw, finger=FINGER_OPEN, duration=1.3)
    b = ctx.states()[name].pos
    yield from arm.move((b[0], b[1], b[2] + PAD), yaw=yaw, duration=0.9)
    yield from arm.grip(FINGER_CLOSED)
    dxy = ctx.states()[name].pos[:2] - np.array(b[:2])  # bag offset from the TCP after closing
    yield from arm.move((b[0], b[1], HI), yaw=yaw, duration=0.8)
    tx, ty = target[0] - dxy[0], target[1] - dxy[1]
    place_z = TOP + 2 * R + BAG[2] / 2 + 0.003 + PAD
    yield from arm.move((tx, ty, HI), yaw=yaw, duration=1.4)
    yield from arm.move((tx, ty, place_z + 0.02), yaw=yaw, duration=0.8)
    yield from arm.move((tx, ty, place_z), yaw=yaw, duration=0.6)
    if cheat == "hold":
        return
    yield from arm.grip(FINGER_OPEN, 0.5)
    yield from arm.move((tx, ty, HI), yaw=yaw, duration=0.7)


def _cable_x_at(ctx, y):
    pts = _dense(ctx.states()["cord"].polyline())
    return float(pts[int(np.argmin(np.abs(pts[:, 1] - y))), 0])


def demo_script(ctx):
    """Wrist cameras: the left arm grips the cable turned so its camera looks along the cable towards the plug end
    (not back at the box); the right arm carries the third bag with its camera facing the bag's mark. The plug grip
    cannot be turned that way (right wrist joint limit), so its camera looks along the cable towards the box.

    Left arm takes the cable ~24 cm from the exit, right arm takes the plug; both lift the cable, carry it over
    the lane and lay it down; the right arm pulls it straight (closed loop on the measured deviation) and keeps
    holding the plug. The left arm lets go and sets sandbags on the cable at the near and middle marks (near first,
    so the cable is pinned close to the exit before more weight lands). Then the right arm lets go of the plug and
    sets the third bag at the far mark. Both arms move clear.

    PIN_CHEAT (test only): none (do nothing) | bagsonly (put the bags on the cable where it lies, 15/30/45 cm from
    the exit, without straightening it) | straight (lay the cable straight in the lane, let go, no bags) | beside
    (straighten it, then set the bags in the lane at the marks but beside the cable) | hold (all done but the left
    arm keeps holding the middle bag)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("PIN_CHEAT", "")
    _log(ctx, "start")
    if cheat == "none":
        yield from L.wait(5.0)
        _log(ctx, "end")
        return
    if cheat == "bagsonly":
        targets = [CORD.point_at(s)[:2] for s in MARK_S]
        yield from _place_bag(ctx, L, BAG_NAMES[0], targets[0], cheat)
        yield from _place_bag(ctx, L, BAG_NAMES[1], targets[1], cheat)
        yield from L.move((0.20, 0.25, HI), yaw=0.0, duration=0.8)
        yield from _place_bag(ctx, Rt, BAG_NAMES[2], targets[2], cheat, yaw=BAG3_YAW)
        yield from ctx.both(L.move((0.20, 0.25, HI), yaw=0.0, duration=0.8),
                            Rt.move((0.20, -0.25, HI), yaw=0.0, duration=0.8))
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    st = ctx.states()
    c = st["cord"]
    k = int(np.argmin(np.abs(c.points[:, 1] - GRASP_Y)))
    s_k = (k + 0.5) * c.seg_len  # arclength of the left grasp point from the exit
    q = c.points[k]
    p = c.end_plug
    y0 = CORD_YAW + math.pi  # cable heading normalised into [-pi/2, pi/2]
    ry = math.pi / 2  # along the lane (heading -pi/2)
    # left arm: same grip turned 180 deg, so its wrist camera looks along the cable towards the plug end and the
    # lane, not back at the stage box (the right arm cannot: its wrist limit forbids yaw -112 deg at the plug)
    ly0, lry = CORD_YAW, -math.pi / 2
    # 1) left arm over the cable point, right arm over the plug; grasp both
    yield from ctx.both(L.move((q[0], q[1], HI), yaw=ly0, finger=GRASP_OPEN, duration=1.5),
                        Rt.move((p[0], p[1], HI), yaw=y0, finger=FINGER_OPEN, duration=1.5))
    st = ctx.states()
    q, p = st["cord"].points[k], st["cord"].end_plug
    yield from ctx.both(L.move((q[0], q[1], q[2] + PAD), yaw=ly0, duration=1.0),
                        Rt.move((p[0], p[1], p[2] + PAD), yaw=y0, duration=1.0))
    yield from ctx.both(L.grip(FINGER_CLOSED), Rt.grip(FINGER_CLOSED))
    yield from ctx.both(L.move((q[0], q[1], LIFT), yaw=ly0, duration=0.8),
                        Rt.move((p[0], p[1], LIFT), yaw=y0, duration=0.8))
    _log(ctx, "cable lifted")
    # 2) carry the lifted cable over the lane and lay it down
    yl = EXIT_Y - s_k - 0.004  # 4 mm past straight: the exit-side part is pulled taut
    tgt = (LX, STRAIGHT_PLUG_Y + 0.008)
    yield from ctx.both(L.move((LX, yl, LIFT), yaw=lry, duration=2.0),
                        Rt.move((tgt[0], tgt[1], LIFT), yaw=ry, duration=2.0))
    yield from L.wait(0.6)
    plug_z = TOP + PLUG.size[2] / 2 + PAD
    yield from ctx.both(L.move((LX, yl, TOP + R + PAD), yaw=lry, duration=1.2),
                        Rt.move((tgt[0], tgt[1], plug_z), yaw=ry, duration=1.2))
    yield from L.wait(0.3)
    _log(ctx, "laid in lane")
    # 3) closed loop: the right arm pulls along the lane until the cable is straight
    py, px = tgt[1], tgt[0]
    for _ in range(6):
        d = deviation(ctx.states())
        if d < 0.003:
            break
        py = max(py - 0.008, STRAIGHT_PLUG_Y - 0.004)
        yield from Rt.move((px, py, plug_z), yaw=ry, duration=0.6)
        yield from Rt.wait(0.3)
        _log(ctx, f"pull y={py:.3f}")
    # 4) left arm lets go of the cable
    yield from L.grip(FINGER_OPEN, 0.5)
    yield from L.move((LX, yl, HI), yaw=lry, duration=0.6)
    if deviation(ctx.states()) > 0.004:  # pull the whole length once more from the plug
        py = STRAIGHT_PLUG_Y - 0.008
        yield from Rt.move((px, py, plug_z), yaw=ry, duration=0.8)
        yield from Rt.wait(0.3)
    _log(ctx, "left let go of the cable")
    if cheat == "straight":
        yield from Rt.grip(FINGER_OPEN)
        yield from Rt.move((px, py, TOP + 0.08), yaw=ry, duration=0.6)
        yield from ctx.both(L.move((0.20, 0.25, HI), yaw=0.0, duration=0.8),
                            Rt.move((0.20, -0.25, HI), yaw=0.0, duration=0.8))
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    off = 0.034 if cheat == "beside" else 0.0  # beside: bag centre 34 mm off the cable (in the lane + tape)
    # 5) left arm: near mark, then middle mark (target = where the cable actually is at the mark)
    for i in (0, 1):
        my = MARK_Y[i]
        yield from _place_bag(ctx, L, BAG_NAMES[i], (_cable_x_at(ctx, my) - off, my),
                              cheat if i == 1 else "")
        _log(ctx, f"bag {i + 1} placed")
    if cheat == "hold":
        yield from L.wait(0.1)
    else:
        yield from L.move((0.20, 0.25, HI), yaw=0.0, duration=1.0)
    # 6) right arm lets go of the plug and sets the third bag at the far mark
    yield from Rt.grip(FINGER_OPEN, 0.5)
    yield from Rt.move((px, py, TOP + 0.08), yaw=ry, duration=0.6)
    _log(ctx, "right let go of the plug")
    my = MARK_Y[2]
    yield from _place_bag(ctx, Rt, BAG_NAMES[2], (_cable_x_at(ctx, my) - off, my), "", yaw=BAG3_YAW)
    yield from Rt.move((0.20, -0.25, HI), yaw=0.0, duration=1.0)
    _log(ctx, "bag 3 placed")
    yield from L.wait(2.0)
    _log(ctx, "end")
