"""Staple a cable near the box and at a maximum spacing.

On the station table a blue PVC outlet box (75 x 50 x 57 mm) stands in front of the left arm, centre x 34.5 cm,
y +35.5 cm. A dark-stained pine stud strip (640 x 38 x 5 mm) runs straight from the box's -y face along -y at x 34.5 cm
(the strip stands in for the face of a wall stud). A black end mark (38 x 5 mm) crosses the strip 55 cm from the box
face (y -22 cm). A white 6 mm NM-style cable, 62 cm long, leaves the box face through a snug entry (held at the exit,
free to swing sideways), but lies straight swung 15 degrees off the strip towards the robot (end at about x 18.5,
y -27.5 cm). Four free orange PLA staple blocks (36 x 15 x 20 mm, 15 g, a 12 mm wide x 9 mm high slot under the
middle) stand upright on the table: two in front of the left arm (x 16, y +25 and y +17 cm), two beyond the far side
of the strip (x 43, y -10 and x 44, y -19 cm).

Goal: lay the cable straight along the stud and fix it with the staple blocks (cable through the slot, both legs on
the stud): one within 6 cm of the box, one within 6 cm of the end mark, no more than 20 cm between neighbouring
staples. Then let go of everything. Four staples are the fewest that can satisfy the rules.

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
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Lay the white cable straight along the wooden stud and fix it with the orange staple blocks, each "
               "set over the cable so the cable runs through its slot: one staple within 6 cm of the box, one within "
               "6 cm of the black end mark, and no more than 20 cm between neighbouring staples. Then let go.")

BOX_BLUE = (0.10, 0.30, 0.75)
WOOD = (0.45, 0.27, 0.12)  # dark-stained pine: stands out from the light table
BLACK = (0.03, 0.03, 0.03)
WHITE = (0.93, 0.93, 0.90)
ORANGE = (0.95, 0.45, 0.05)

# -- outlet box, stud strip, end mark --------------------------------------------------------------------------------
SX = 0.345  # stud centre line (x)
BOX_SIZE = (0.075, 0.050, 0.057)
BOX_FACE_Y = 0.330
STUD_W, STUD_T, STUD_LEN = 0.038, 0.005, 0.640
STUD_TOP = TOP + STUD_T
RUN = 0.55  # end mark distance from the box face along the stud
END_Y = BOX_FACE_Y - RUN
MARK_W, MARK_T = 0.005, 0.0004

FIXED = [
    Box("outlet_box", pos=(SX, BOX_FACE_Y + BOX_SIZE[1] / 2, TOP + BOX_SIZE[2] / 2), size=BOX_SIZE, color=BOX_BLUE),
    Box("stud", pos=(SX, BOX_FACE_Y - STUD_LEN / 2, TOP + STUD_T / 2), size=(STUD_W, STUD_LEN, STUD_T), color=WOOD),
    Box("end_mark", pos=(SX, END_Y, STUD_TOP + MARK_T / 2), size=(STUD_W, MARK_W, MARK_T), color=BLACK),
]

# -- cable: pivoted at the box entry, starts straight but swung off the stud towards the robot ----------------------
R = 0.003
CORD_LEN = 0.62
EXIT_Y = BOX_FACE_Y - 0.006  # cable start 6 mm in front of the box face
SWING = math.radians(15.0)
CORD_YAW = -math.pi / 2 - SWING
CORD = cable("cord", start=(SX, EXIT_Y, STUD_TOP + R + 0.0015), yaw=CORD_YAW, length=CORD_LEN, radius=R,
             anchor_start=True, color=WHITE, bend_stiffness=0.01, bend_damping=0.001)

# -- staple blocks: each is a top bar plus two legs, welded into one rigid part ------------------------------------
ST_W, ST_D, ST_H = 0.036, 0.015, 0.020  # across the cable (x), along it (y), height
SLOT_W, SLOT_H = 0.012, 0.009
LEG_W = (ST_W - SLOT_W) / 2
BAR_H = ST_H - SLOT_H
BAR_DZ = SLOT_H + BAR_H / 2  # top-bar centre above the staple bottom
LEG_DX = SLOT_W / 2 + LEG_W / 2
LEG_DZ = SLOT_H / 2 - BAR_DZ  # leg centre relative to the top-bar centre
ST_MASS_BAR, ST_MASS_LEG = 0.008, 0.0035
STAPLE_XY = ((0.16, 0.25), (0.16, 0.17), (0.43, -0.10), (0.44, -0.19))
STAPLES, STAPLE_NAMES, LEGS = [], [], []
for _i, (_x, _y) in enumerate(STAPLE_XY):
    _n = f"staple_{_i + 1}"
    STAPLE_NAMES.append(_n)
    STAPLES.append(Box(_n, pos=(_x, _y, TOP + BAR_DZ + 0.0005), size=(ST_W, ST_D, BAR_H), mass=ST_MASS_BAR,
                       color=ORANGE))
    for _tag, _s in (("a", -1), ("b", 1)):
        LEGS.append((_n, Box(f"{_n}_leg_{_tag}", pos=(_x + _s * LEG_DX, _y, TOP + SLOT_H / 2 + 0.0005),
                             size=(LEG_W, ST_D, SLOT_H), mass=ST_MASS_LEG, color=ORANGE), _s * LEG_DX))

OBJECTS = [*FIXED, CORD, *STAPLES, *(leg for _, leg, _ in LEGS)]

TWIN_EXTRA = [
    {"name": "cable_entry", "kind": "joint", "real": "snug cable clamp in the box knockout",
     "note": "cable held at the box face, free to swing sideways (vertical pin in sim)"},
    {"name": "staple_block", "kind": "body", "size_mm": [36, 15, 20], "colour": "orange",
     "pose": [0.16, 0.25, 0.01], "note": "one PLA print: 6 mm top bar over two 12 x 9 mm legs, 12 x 9 mm slot"},
]


def _install_hooks() -> None:
    """(1) Replace the kit's rigid world weld of the cable start by a pin about the vertical at the box entry (the
    cable is clamped in the box knockout but can swing sideways; a rigid weld would keep the first segment at the
    15 degree start angle). (2) Weld each staple's two legs to its top bar (one rigid printed part)."""
    orig = cablekit._spawn_cable
    if getattr(orig, "_staple_hooks", False):
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
        parent = str(prim_path).rsplit("/", 1)[0]
        for bar, leg, dx in LEGS:
            w = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/weld_{leg.name}")
            w.CreateBody0Rel().SetTargets([Sdf.Path(f"{parent}/{bar}")])
            w.CreateBody1Rel().SetTargets([Sdf.Path(f"{parent}/{leg.name}")])
            w.CreateLocalPos0Attr(Gf.Vec3f(float(dx), 0.0, float(LEG_DZ)))
            w.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            w.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            w.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            w.CreateExcludeFromArticulationAttr(True)
        return prim

    spawn._staple_hooks = True
    cablekit._spawn_cable = spawn


_install_hooks()

_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_staple_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._staple_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World TCP positions of both grippers (yam_arena.runtime.tcp_world, env 0); [] if unknown (offline)."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- scoring ---------------------------------------------------------------------------------------------------------
OFF_STUD = 0.020  # cable centreline may be at most 2 cm sideways from the stud centre line (box face to end mark)
NEAR = 0.060  # a staple within 6 cm of the box face, and one within 6 cm of the end mark
MAX_GAP = 0.200  # neighbouring staples at most 20 cm apart
SEAT_TOL = 0.003  # staple bottom (both legs) within 3 mm of the stud top
TILT_MAX = math.radians(10.0)
YAW_MAX = math.radians(25.0)  # slot runs along the stud within 25 deg
RELEASE_DIST = 0.035
REACH_LADDER = ((0.45, 0.75), (0.30, 0.5), (0.15, 0.25))  # (chain reach from the box face, score)


def _dense(poly: np.ndarray, step: float = 0.002) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _rot(q_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in q_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def off_stud(objects) -> float:
    """Max sideways distance (m) of the cable centreline from the stud centre line, box face to end mark."""
    pts = _dense(objects["cord"].polyline())
    run = pts[(pts[:, 1] <= BOX_FACE_Y) & (pts[:, 1] >= END_Y)]
    if len(run) == 0:
        return 1.0
    return float(np.abs(run[:, 0] - SX).max())


def staple_fit(objects, name: str) -> dict:
    """Geometry of one staple: tilt, slot yaw error, bottom height above the stud top, whether the cable centreline
    passes through its slot, and its distance from the box face along the stud."""
    b = objects[name]
    rm = _rot(b.quat_xyzw)
    tilt = math.acos(max(-1.0, min(1.0, float(rm[2, 2]))))
    slot_dir = rm[:, 1]
    yaw_err = math.acos(min(1.0, abs(float(slot_dir[1])) / (np.linalg.norm(slot_dir[:2]) + 1e-12)))
    bottom = np.asarray(b.pos) - rm[:, 2] * BAR_DZ
    seat = float(bottom[2] - STUD_TOP)
    pts = _dense(objects["cord"].polyline())
    local = (pts - bottom) @ rm  # cable points in the staple frame (origin: middle of the bottom)
    inslot = (np.abs(local[:, 0]) <= SLOT_W / 2) & (np.abs(local[:, 1]) <= ST_D / 2) & \
             (local[:, 2] >= -0.002) & (local[:, 2] <= SLOT_H)
    on_stud = abs(float(bottom[0]) - SX) <= STUD_W / 2 and END_Y - 0.08 <= float(bottom[1]) <= BOX_FACE_Y
    return {"tilt": tilt, "yaw_err": yaw_err, "seat": seat, "in_slot": bool(inslot.any()), "on_stud": on_stud,
            "s": float(BOX_FACE_Y - bottom[1])}


def straddles(objects, name: str) -> bool:
    """The staple sits flat on the stud (bottom within 3 mm of the stud top, tilt under 10 deg, slot along the stud
    within 25 deg) with the cable running through its slot."""
    f = staple_fit(objects, name)
    return (f["in_slot"] and f["on_stud"] and abs(f["seat"]) <= SEAT_TOL and f["tilt"] <= TILT_MAX
            and f["yaw_err"] <= YAW_MAX)


def staple_positions(objects) -> list[float]:
    """Distances (m) from the box face along the stud of the staples that straddle the cable, sorted."""
    return sorted(staple_fit(objects, n)["s"] for n in STAPLE_NAMES if straddles(objects, n))


def chain(objects) -> tuple[float, bool]:
    """(reach, complete). Start at the staple farthest from the box that is still within 6 cm of it; step to the next
    staple along the stud while the gap is at most 20 cm. reach = distance of the last staple reached from the box
    face (-1 if no staple is within 6 cm of the box); complete = a reached staple lies within 6 cm of the end mark."""
    s = staple_positions(objects)
    near = [v for v in s if v <= NEAR]
    if not near:
        return -1.0, False
    cur = max(near)
    reached = [cur]
    for v in s:
        if v <= cur:
            continue
        if v - cur > MAX_GAP + 1e-9:
            break
        cur = v
        reached.append(v)
    return cur, any(abs(v - RUN) <= NEAR for v in reached)


def released(objects) -> bool:
    """Every gripper TCP at least 3.5 cm from the cable and from every staple block."""
    pts = _dense(objects["cord"].polyline(), 0.004)
    for g in grippers():
        if float(np.linalg.norm(pts - g, axis=1).min()) < RELEASE_DIST:
            return False
        for n in [*STAPLE_NAMES, *(leg.name for _, leg, _ in LEGS)]:
            b = objects[n]
            if float(np.linalg.norm(g - np.clip(g, b.low, b.high))) < RELEASE_DIST:
                return False
    return True


def score(objects) -> float:
    """0 if a gripper is within 3.5 cm of the cable or a staple, or the cable centreline is more than 2 cm sideways
    from the stud centre line anywhere between the box and the end mark, or no straddling staple is within 6 cm of
    the box. Otherwise follow straddling staples from that one with gaps of at most 20 cm: 1.0 if the chain reaches a
    staple within 6 cm of the end mark; else 0.75 if it reaches 45 cm or more from the box, 0.5 if 30 cm or more,
    0.25 if 15 cm or more, 0 below (a lone staple at the box earns nothing)."""
    if not released(objects) or off_stud(objects) > OFF_STUD:
        return 0.0
    reach, complete = chain(objects)
    if reach < 0:
        return 0.0
    if complete:
        return 1.0
    for th, v in REACH_LADDER:
        if reach >= th:
            return v
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 0.999 and all(objects[n].resting(max_speed=0.01) for n in STAPLE_NAMES) \
        and objects["cord"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Cable offset beyond 2 cm plus how far the stapling chain is from the end mark (m)."""
    reach, complete = chain(objects)
    gap = 0.0 if complete else (RUN - NEAR - max(reach, 0.0)) + (0.1 if reach < 0 else 0.0)
    return float(max(0.0, off_stud(objects) - OFF_STUD) + max(gap, 0.0))


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    reach, complete = chain(objects)
    n = len(staple_positions(objects))
    return {"cable_on_stud": float(off_stud(objects) <= OFF_STUD),
            "staples_straddling": n / 4.0,
            "box_staple": float(reach >= 0),
            "reach_frac": float(min(1.0, max(reach, 0.0) / (RUN - NEAR))),
            "end_staple": float(complete),
            "released": float(released(objects))}


VARIATION = {
    "cable:cord": {"dx": 0.0, "dy": 0.0, "yaw": 0.06},
    "obj:staple_1": {"dx": 0.015, "dy": 0.01, "yaw": 0.15},
    "obj:staple_2": {"dx": 0.015, "dy": 0.01, "yaw": 0.15},
    "obj:staple_3": {"dx": 0.015, "dy": 0.01, "yaw": 0.15},
    "obj:staple_4": {"dx": 0.015, "dy": 0.01, "yaw": 0.15},
}

# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------

PAD = 0.010  # TCP above the grasped part's centre
HI = TOP + 0.09
LIFT = TOP + 0.06
GRASP_Y = 0.09  # left arm takes the cable at the point nearest this y (~24 cm from the exit)
END_GRASP_S = 0.58  # right arm takes the cable this far from the exit (4 cm from its end)
GRASP_OPEN = -0.014  # finger joint: pads ~28 mm apart round the 6 mm cable
TARGET_S = (0.035, 0.195, 0.355, 0.515)  # staple centres from the box face (gaps 16 cm)
STAPLE_GRIP_DZ = 0.0105  # TCP above the top-bar centre when holding a staple


def _log(ctx, tag):
    st = ctx.states()
    fits = []
    for n in STAPLE_NAMES:
        f = staple_fit(st, n)
        fits.append(f"{n}: s={f['s'] * 100:.1f} seat={f['seat'] * 1000:.1f}mm tilt={math.degrees(f['tilt']):.0f} "
                    f"slot={int(f['in_slot'])} ok={int(straddles(st, n))}")
    print(f"[demo t={ctx.t:5.1f}] {tag}: off={off_stud(st) * 1000:.1f}mm chain={chain(st)} rel={released(st)} "
          f"score={score(st):.3f}\n    " + "\n    ".join(fits), flush=True)


def _cable_x_at(ctx, y):
    pts = _dense(ctx.states()["cord"].polyline())
    return float(pts[int(np.argmin(np.abs(pts[:, 1] - y))), 0])


def _place_staple(ctx, arm, name, target_s, cheat="", side=0.0):
    """Pick staple `name` by its top bar (fingers close along the cable direction, wrist camera facing +x across the
    stud) and set it down over the cable `target_s` from the box face, centred on where the cable actually is (plus
    `side` metres sideways for the beside-the-cable shortcut). Lets go and lifts clear unless cheat == 'hold'."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    yaw = 0.0
    b = ctx.states()[name].pos
    yield from arm.move((b[0], b[1], HI), yaw=yaw, finger=FINGER_OPEN, duration=1.2)
    b = ctx.states()[name].pos
    yield from arm.move((b[0], b[1], b[2] + STAPLE_GRIP_DZ), yaw=yaw, duration=0.9)
    yield from arm.grip(FINGER_CLOSED)
    dxy = ctx.states()[name].pos[:2] - np.array(b[:2])
    yield from arm.move((b[0], b[1], HI), yaw=yaw, duration=0.8)
    ty = BOX_FACE_Y - target_s
    tx = _cable_x_at(ctx, ty) + side
    gx, gy = tx - dxy[0], ty - dxy[1]
    place_z = STUD_TOP + BAR_DZ + STAPLE_GRIP_DZ + 0.002
    yield from arm.move((gx, gy, HI), yaw=yaw, duration=1.3)
    yield from arm.move((gx, gy, place_z + 0.02), yaw=yaw, duration=0.7)
    st = ctx.states()  # closed loop: re-centre on the cable right before the last few cm
    tx = _cable_x_at(ctx, ty) + side
    dxy = st[name].pos[:2] - np.array([gx, gy])
    gx, gy = tx - dxy[0], ty - dxy[1]
    yield from arm.move((gx, gy, place_z + 0.008), yaw=yaw, duration=0.4)
    yield from arm.move((gx, gy, place_z), yaw=yaw, duration=0.5)
    if cheat == "hold":
        return
    yield from arm.grip(FINGER_OPEN, 0.4)
    yield from arm.move((gx, gy, HI), yaw=yaw, duration=0.6)


def demo_script(ctx):
    """Left arm takes the cable ~24 cm from the box, right arm ~4 cm from its end; both lift it, carry it over the
    stud and lay it down; the right arm pulls along the stud until it is straight and keeps holding the end. The left
    arm lets go and staples near the box and at ~20 cm; the right arm then lets go of the cable end and staples at
    ~36 cm and ~52 cm (3.5 cm before the end mark). Both arms park clear.

    STAPLE_CHEAT (test only): none (do nothing) | where (staple the cable where it lies, without laying it on the
    stud) | far (lay it, staples evenly at 9, 24, 39, 54 cm: first one 9 cm from the box) | beside (lay it, staples
    at the target distances but 4 cm to the side: beside the cable and the stud) | pile (lay it, two staples on the
    cable within 6 cm of the box, at 2 and 5 cm) | two (lay it, only the first two staples: an
    honest partial, 0.25) | hold (all done but the right arm keeps
    holding the last staple)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("STAPLE_CHEAT", "")
    targets = list(TARGET_S)
    if cheat == "far":
        targets = [0.09, 0.24, 0.39, 0.54]
    side = 0.040 if cheat == "beside" else 0.0
    _log(ctx, "start")
    if cheat == "none":
        yield from L.wait(5.0)
        _log(ctx, "end")
        return
    if cheat != "where":
        st = ctx.states()
        c = st["cord"]
        k = int(np.argmin(np.abs(c.points[:, 1] - GRASP_Y)))
        s_k = (k + 0.5) * c.seg_len
        ke = min(len(c.points) - 1, int(END_GRASP_S / c.seg_len))
        s_e = (ke + 0.5) * c.seg_len
        q, e = c.points[k], c.points[ke]
        ly0, ry0 = CORD_YAW, CORD_YAW + math.pi  # cross-cable grips; left camera looks along the cable to its end
        lry, rry = -math.pi / 2, math.pi / 2
        yield from ctx.both(L.move((q[0], q[1], HI), yaw=ly0, finger=GRASP_OPEN, duration=1.5),
                            Rt.move((e[0], e[1], HI), yaw=ry0, finger=GRASP_OPEN, duration=1.5))
        st = ctx.states()
        q, e = st["cord"].points[k], st["cord"].points[ke]
        yield from ctx.both(L.move((q[0], q[1], q[2] + PAD), yaw=ly0, duration=1.0),
                            Rt.move((e[0], e[1], e[2] + PAD), yaw=ry0, duration=1.0))
        yield from ctx.both(L.grip(FINGER_CLOSED), Rt.grip(FINGER_CLOSED))
        yield from ctx.both(L.move((q[0], q[1], LIFT), yaw=ly0, duration=0.8),
                            Rt.move((e[0], e[1], LIFT), yaw=ry0, duration=0.8))
        _log(ctx, "cable lifted")
        yl = EXIT_Y - s_k - 0.004
        ye = EXIT_Y - s_e - 0.004
        yield from ctx.both(L.move((SX, yl, LIFT), yaw=lry, duration=2.0),
                            Rt.move((SX, ye + 0.010, LIFT), yaw=rry, duration=2.0))
        yield from L.wait(0.5)
        lay_z = STUD_TOP + R + PAD
        yield from ctx.both(L.move((SX, yl, lay_z), yaw=lry, duration=1.2),
                            Rt.move((SX, ye + 0.010, lay_z), yaw=rry, duration=1.2))
        yield from L.wait(0.3)
        _log(ctx, "laid on stud")
        py = ye + 0.010
        for _ in range(5):
            if off_stud(ctx.states()) < 0.004:
                break
            py = max(py - 0.006, ye - 0.006)
            yield from Rt.move((SX, py, lay_z), yaw=rry, duration=0.5)
            yield from Rt.wait(0.3)
        yield from L.grip(GRASP_OPEN, 0.4)
        yield from L.move((SX, yl, HI), yaw=lry, duration=0.6)
        _log(ctx, "left let go of the cable")
    # left arm: the two staples nearest the box; right arm (after letting go of the cable end): the two far ones
    plan = [(L, STAPLE_NAMES[0], targets[0]), (L, STAPLE_NAMES[1], targets[1]),
            (Rt, STAPLE_NAMES[2], targets[2]), (Rt, STAPLE_NAMES[3], targets[3])]
    if cheat == "pile":
        plan = [(L, STAPLE_NAMES[0], 0.020), (L, STAPLE_NAMES[1], 0.050)]
    if cheat == "two":
        plan = plan[:2]
    holding_end = cheat != "where"
    for i, (arm, n, ts) in enumerate(plan):
        if arm is Rt and holding_end:
            yield from L.move((0.20, 0.30, HI), yaw=0.0, duration=0.9)
            yield from Rt.grip(GRASP_OPEN, 0.4)
            yield from Rt.move((SX, py, HI), yaw=rry, duration=0.6)
            holding_end = False
            _log(ctx, "right let go of the cable")
        if cheat == "where":  # where the swung cable lies at that distance from the exit
            ts = BOX_FACE_Y - CORD.point_at(ts - 0.006)[1]
        hold = cheat == "hold" and i == len(plan) - 1
        yield from _place_staple(ctx, arm, n, ts, "hold" if hold else "", side)
        _log(ctx, f"{n} placed")
    if holding_end:
        yield from Rt.grip(GRASP_OPEN, 0.4)
        yield from Rt.move((SX, py, HI), yaw=rry, duration=0.6)
    yield from L.move((0.20, 0.30, HI), yaw=0.0, duration=0.9)
    if cheat != "hold":
        yield from Rt.move((0.20, -0.30, HI), yaw=0.0, duration=1.0)
    yield from L.wait(2.0)
    _log(ctx, "end")
