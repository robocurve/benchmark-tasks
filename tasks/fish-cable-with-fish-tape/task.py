"""On the station table: a matte grey baseboard (140 x 600 x 5 mm) lies along the robot's left-right axis in front of
the arms. A straight light-grey conduit (300 mm long, square 26 x 26 mm bore, 3 mm walls) is fixed on it, running
left-right. A yellow 4.8 mm round fibreglass fish tape (46 cm) is already threaded through the conduit: its steel hook
(a 20 mm stem with a 4 x 4 mm prong standing 20 mm up at its tip) sticks out 6 cm from the right-hand (near) mouth,
and its other end, with a black handle, sticks out 12 cm from the left-hand (far) mouth. An orange 6 mm cable (66 cm)
lies loose on the table in front of the right arm; its free end carries a steel pulling eyelet: a 28 mm lug and a
square ring (18 mm outside, 12 mm hole, 3 mm bars). The eyelet must be hooked over the prong, then the tape pulled
back through the conduit (hand over hand) until the eyelet comes out of the far mouth.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Pull the orange cable through the grey conduit with the yellow fish tape: hook the cable's eyelet "
               "onto the fish-tape hook at the conduit's right-hand end, then pull the tape back out of the left-hand "
               "end until the eyelet is at least 5 cm out of the conduit.")

# -- baseboard and conduit (fixed) -------------------------------------------------------------------------------
BOARD_T = 0.005
BT = TOP + BOARD_T  # board top
CX = 0.36  # conduit axis x
Y0, Y1 = -0.16, 0.14  # near (right, -y) and far (left, +y) mouths
BORE = 0.026
WALL = 0.003
FZ = BT + WALL  # bore floor
C_LEN = Y1 - Y0
CY = (Y0 + Y1) / 2
GREY_BOARD = (0.42, 0.43, 0.45)
GREY_PIPE = (0.78, 0.79, 0.80)

BOARD = Box("baseboard", pos=(CX, 0.01, TOP + BOARD_T / 2), size=(0.14, 0.60, BOARD_T), color=GREY_BOARD)
OUT_W = BORE + 2 * WALL
CONDUIT = [
    Box("conduit_floor", pos=(CX, CY, BT + WALL / 2), size=(OUT_W, C_LEN, WALL), color=GREY_PIPE),
    Box("conduit_roof", pos=(CX, CY, BT + WALL + BORE + WALL / 2), size=(OUT_W, C_LEN, WALL), color=GREY_PIPE),
    Box("conduit_side_far", pos=(CX + BORE / 2 + WALL / 2, CY, BT + OUT_W / 2), size=(WALL, C_LEN, OUT_W),
        color=GREY_PIPE),
    Box("conduit_side_near", pos=(CX - BORE / 2 - WALL / 2, CY, BT + OUT_W / 2), size=(WALL, C_LEN, OUT_W),
        color=GREY_PIPE),
]

# -- fish tape: a stiff round rod chain; start = handle end (far side), end = hook (near side) ---------------------
TAPE_R = 0.0024
TAPE_LEN = 0.46
TAPE_Z = FZ + TAPE_R + 0.0005
TAPE_END_Y = Y0 - 0.04  # chain end 4 cm out of the near mouth; the hook stem continues 2 cm
HANDLE = Plug(size=(0.04, 0.014, 0.009), mass=0.008, color=(0.08, 0.08, 0.08))
STEM = Plug(size=(0.02, 0.006, 0.0045), mass=0.006, color=(0.70, 0.71, 0.73))
TAPE = cable("fish_tape", start=(CX, TAPE_END_Y + TAPE_LEN, TAPE_Z), yaw=-math.pi / 2, length=TAPE_LEN,
             radius=TAPE_R, start_plug=HANDLE, end_plug=STEM, color=(0.95, 0.80, 0.10), cable_mass=0.02,
             bend_stiffness=0.03, bend_damping=0.003, friction=0.5)
PRONG = (0.004, 0.004)  # x-y section (m)
PRONG_OFF = STEM.size[0] / 2 - PRONG[0] / 2 - 0.001  # prong axis, along the stem from the stem centre
PRONG_Z0, PRONG_Z1 = -STEM.size[2] / 2, 0.020  # prong spans these heights relative to the tape axis

# -- cable with a pulling eyelet (lug + square ring) on its free end ----------------------------------------------
R = 0.003
CABLE_LEN = 0.66
LUG = Plug(size=(0.028, 0.009, 0.007), mass=0.006, color=(0.70, 0.71, 0.73))
RING_OUT, RING_BAR = 0.018, 0.003
RING_IN = RING_OUT - 2 * RING_BAR  # 12 mm hole
RING_OFF = LUG.size[0] / 2 + RING_OUT / 2 - 0.001  # ring centre, along the cable from the lug centre
CABLE_HEAD = math.radians(150)  # direction the eyelet end points at the start
_e = np.array([math.cos(CABLE_HEAD), math.sin(CABLE_HEAD), 0.0])
RING0 = np.array([0.24, -0.28, TOP + R + 0.0005])  # starting ring centre
_end_tip = RING0 - (RING_OFF + LUG.size[0] / 2) * _e
CABLE_START = _end_tip - CABLE_LEN * _e
CABLE = cable("cable", start=tuple(map(float, CABLE_START)), yaw=CABLE_HEAD, length=CABLE_LEN, radius=R,
              end_plug=LUG, color=(0.95, 0.42, 0.08), bend_stiffness=0.01, bend_damping=0.001)

OBJECTS = [BOARD, *CONDUIT, TAPE, CABLE]


# -- extra colliders: the ring on the cable's lug, the prong on the tape's hook stem ------------------------------
# cablekit plugs are single boxes. The cable spawn function is wrapped so that, right after a cable is authored,
# extra box colliders are added under its end-plug rigid body (a compound rigid body: the ring and prong are rigid
# parts of the lug / hook stem and move with them).


def _extra_parts(name: str) -> list[tuple[str, tuple, tuple]]:
    """(part name, centre in the plug frame, size) for the extra colliders on ``name``'s end plug."""
    if name == CABLE.name:
        c, h, b = RING_OFF, RING_OUT / 2, RING_BAR
        return [
            ("ring_front", (c + h - b / 2, 0.0, 0.0), (b, RING_OUT, b)),
            ("ring_back", (c - h + b / 2, 0.0, 0.0), (b, RING_OUT, b)),
            ("ring_l", (c, h - b / 2, 0.0), (RING_OUT, b, b)),
            ("ring_r", (c, -h + b / 2, 0.0), (RING_OUT, b, b)),
        ]
    if name == TAPE.name:
        return [("prong", (PRONG_OFF, 0.0, (PRONG_Z0 + PRONG_Z1) / 2), (PRONG[0], PRONG[1], PRONG_Z1 - PRONG_Z0))]
    return []


def _install_parts() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_fish_parts", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        parts = _extra_parts(getattr(cfg, "cable_name", ""))
        if not parts:
            return prim
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

        stage = get_current_stage()
        for pname, c, s in parts:
            g = UsdGeom.Cube.Define(stage, f"{prim_path}/end_plug/{pname}")
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, s)))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(g.GetPath()), f"{prim_path}/Looks/cable_physics", stage=stage)
            bind_visual_material(str(g.GetPath()), f"{prim_path}/Looks/end_plug", stage=stage)
        return prim

    spawn._fish_parts = True
    cablekit._spawn_cable = spawn


_install_parts()

# -- geometry used by scoring ----------------------------------------------------------------------------------------
HOOK_TOL = 0.0095  # prong axis within this of the ring centre (x and y in the ring's frame): the prong is in the hole
# (prong axis can be at most 4 mm from the centre when it passes through the 12 mm hole, and at least 11 mm when it
# is outside the ring, so 9.5 mm separates the two with room for a tilted ring).
INSIDE = 0.02  # eyelet counts as inside the conduit once its centre is 2 cm past the near mouth
OUT_GOAL = 0.05  # full score: eyelet centre at least 5 cm past the far mouth
UNHOOKED_STEPS = 6  # eyelet inside the conduit, off the hook, for this many consecutive checks (0.3 s) -> score 0


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so scoring can read the end plugs' orientations (CableState has positions only)."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_fish_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._fish_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def _quat_matrix(x, y, z, w) -> np.ndarray:
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def plug_frame(state, name: str) -> np.ndarray:
    """Rotation matrix of ``name``'s end plug (columns: along the cable, across, up). Read from the sim when it is
    available; otherwise built from the end direction (no roll), for offline tests."""
    env = _ENV["env"]
    if env is not None:
        try:
            art = env.unwrapped.scene[name]
            i = list(art.body_names).index("end_plug")
            q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0, i].detach().cpu().numpy()
            q = q.astype(np.float64)
            # the quaternion layout (xyzw vs wxyz) is picked as the one whose x axis follows the end segment
            cands = [_quat_matrix(*q), _quat_matrix(q[1], q[2], q[3], q[0])]
            return max(cands, key=lambda m: float(m[:, 0] @ state.end_dir))
        except Exception:
            pass
    e = state.end_dir / (np.linalg.norm(state.end_dir) + 1e-12)
    lat = np.cross([0.0, 0.0, 1.0], e)
    lat /= np.linalg.norm(lat) + 1e-12
    return np.stack([e, lat, np.cross(e, lat)], axis=1)


def ring_centre(c) -> np.ndarray:
    return c.end_plug + plug_frame(c, "cable")[:, 0] * RING_OFF


def prong_axis(t) -> tuple[np.ndarray, np.ndarray]:
    """Bottom and top points of the prong axis."""
    m = plug_frame(t, "fish_tape")
    base = t.end_plug + m[:, 0] * PRONG_OFF
    return base + m[:, 2] * PRONG_Z0, base + m[:, 2] * PRONG_Z1


def prong_base(t) -> np.ndarray:
    """Point on the prong axis at the tape axis height."""
    return t.end_plug + plug_frame(t, "fish_tape")[:, 0] * PRONG_OFF


def hooked(objects) -> bool:
    """The prong passes through the eyelet's hole: in the ring's own frame, the prong axis crosses the ring's plane
    (between the prong's bottom and top) within HOOK_TOL of the ring centre along and across the ring."""
    c, t = objects["cable"], objects["fish_tape"]
    m = plug_frame(c, "cable")
    rc = c.end_plug + m[:, 0] * RING_OFF
    a, b = (m.T @ (p - rc) for p in prong_axis(t))  # prong ends in the ring frame
    if (a[2] > 0) == (b[2] > 0) or abs(b[2] - a[2]) < 1e-6:
        return False  # the prong does not cross the ring's plane
    s = a[2] / (a[2] - b[2])
    x = a + s * (b - a)
    return bool(abs(x[0]) <= HOOK_TOL and abs(x[1]) <= HOOK_TOL)


def in_bore(p, tol: float = 0.004) -> bool:
    """Point inside the conduit's bore (cross-section with a small tolerance, between the mouths)."""
    return bool(abs(p[0] - CX) <= BORE / 2 + tol and FZ - tol <= p[2] <= FZ + BORE + tol and Y0 <= p[1] <= Y1)


def _in_section(p, tol: float = 0.004) -> bool:
    return bool(abs(p[0] - CX) <= BORE / 2 + tol and FZ - tol <= p[2] <= FZ + BORE + tol)


def eyelet_inside(objects) -> bool:
    rc = ring_centre(objects["cable"])
    return bool(_in_section(rc, 0.01) and Y0 + INSIDE <= rc[1] <= Y1)


def routed_from_near(objects) -> bool:
    """Following the cable back from the eyelet, it runs inside the bore and leaves through the NEAR mouth: points
    past the far mouth are skipped, then every point must be inside the bore section until the cable reaches the
    near mouth. False if it never enters, leaves the bore sideways, or comes in through the far mouth."""
    pts = objects["cable"].points[::-1]
    i, n = 0, len(pts)
    while i < n and pts[i][1] > Y1 - 0.005 and not in_bore(pts[i]):
        i += 1  # outside, beyond the far mouth
    entered = False
    while i < n:
        p = pts[i]
        if p[1] <= Y0 + 0.005:
            return entered or ring_centre(objects["cable"])[1] > Y0
        if not _in_section(p):
            return False
        entered = True
        i += 1
    return False


def eyelet_out(objects) -> float:
    """How far the eyelet centre is past the far mouth (m; negative = not yet out)."""
    return float(ring_centre(objects["cable"])[1] - Y1)


# -- episode memory -----------------------------------------------------------------------------------------------
_EP = {"unhooked_run": 0, "pushed": False, "max_depth": -1.0}


def _reset_episode() -> None:
    _EP.update(unhooked_run=0, pushed=False, max_depth=-1.0)


def _install_reset_hook() -> None:
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
    if _reset_episode not in cls.reset._task_reset_hooks:
        cls.reset._task_reset_hooks.append(_reset_episode)


_install_reset_hook()
_START = (CABLE.start_points(), TAPE.start_points())


def _track(objects) -> None:
    c, t = objects["cable"], objects["fish_tape"]
    if np.abs(c.points - _START[0]).max() < 0.003 and np.abs(t.points - _START[1]).max() < 0.003:
        _reset_episode()  # untouched starting layout => new episode
    inside = eyelet_inside(objects)
    if inside and not hooked(objects):
        _EP["unhooked_run"] += 1
        if _EP["unhooked_run"] >= UNHOOKED_STEPS:
            _EP["pushed"] = True
    else:
        _EP["unhooked_run"] = 0
    _EP["max_depth"] = max(_EP["max_depth"], float(ring_centre(c)[1] - Y0))


def score(objects) -> float:
    """0 if the eyelet was ever inside the conduit (centre 2 cm or more past the near mouth) without being on the hook
    (pushed in by hand). Otherwise, with the eyelet on the hook at the end: 1.0 if it is at least 5 cm past the far
    mouth and the cable runs back through the bore and out of the near mouth; 0.75 if it is out of the far mouth by
    less than that (same route); 0.5 if it is inside the conduit (same route); 0.25 if it is still at the near mouth
    with no part of the cable inside the conduit. Eyelet off the hook at the end = 0."""
    _track(objects)
    if _EP["pushed"] or not hooked(objects):
        return 0.0
    rc = ring_centre(objects["cable"])
    if rc[1] < Y0 + INSIDE:
        # hooked at the near mouth, not yet pulled in: no part of the cable may be inside the conduit yet
        return 0.0 if any(in_bore(p) for p in objects["cable"].points) else 0.25
    if not routed_from_near(objects):
        return 0.0
    out = eyelet_out(objects)
    if out >= OUT_GOAL:
        return 1.0
    if out >= 0.0:
        return 0.75
    return 0.5


def success(objects) -> bool:
    s = score(objects)
    return bool(s >= 1.0 and objects["cable"].resting(0.02, 1.0) and objects["fish_tape"].resting(0.02, 1.0))


def distance(objects) -> float:
    """Distance (m) the eyelet still has to travel to be 5 cm past the far mouth; 1 m after a by-hand push."""
    _track(objects)
    if _EP["pushed"]:
        return 1.0
    return float(max(0.0, Y1 + OUT_GOAL - ring_centre(objects["cable"])[1]))


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    rc = ring_centre(objects["cable"])
    h = hooked(objects)
    return {
        "eyelet_hooked": int(h),
        "eyelet_entered": int(h and rc[1] >= Y0 + INSIDE),
        "eyelet_out": float(np.clip(eyelet_out(objects) / OUT_GOAL, 0.0, 1.0)) if h else 0.0,
        "routed_through_bore": int(routed_from_near(objects)),
        "not_pushed_by_hand": int(not _EP["pushed"]),
    }


# Seeded variation ranges for a later runner (the demo runs on the nominal layout and reads live states).
VARIATION = {
    "cable:cable": {"dx": 0.015, "dy": 0.015, "yaw": 0.10},
    "cable:fish_tape": {"dx": 0.0, "dy": 0.015, "yaw": 0.0},  # slid along the conduit: hook 4.5-7.5 cm out
}


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------


def _log(ctx, tag=""):
    st = ctx.states()
    c, t = st["cable"], st["fish_tape"]
    rc, pb = ring_centre(c), prong_base(t)
    print(f"[t={ctx.t:5.2f}] {tag}: ring={np.round(rc, 4)} prong={np.round(pb, 4)} hooked={hooked(st)} "
          f"inside={eyelet_inside(st)} routed={routed_from_near(st)} out={eyelet_out(st):.3f} "
          f"pushed={_EP['pushed']} score={score(st):.2f} tcpR={np.round(ctx.right.tcp(), 3)} "
          f"tcpL={np.round(ctx.left.tcp(), 3)}", flush=True)


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _heading(v) -> float:
    return math.atan2(v[1], v[0])


HI = TOP + 0.075  # travel height
SET_LUG = (0.25, -0.27)  # where the right arm squares up the eyelet before re-pinching it
GRASP_OPEN = -0.014  # pads ~28 mm apart round the 4.8 mm tape


GRIP_Y = Y1 + 0.04  # the left arm grips the bare tape 4 cm out of the far mouth
STROKE = 0.20  # each pull stroke (hand over hand)


def _grip_tape(ctx, L, f_closed):
    """Pinch the bare tape GRIP_Y (or the nearest segment there) and hold it: steadies the tape for hooking."""
    t = ctx.states()["fish_tape"]
    gy = max(GRIP_Y, float(prong_base(t)[1]) + 0.07)  # keep the fingers clear of the hook and the eyelet
    k = int(np.argmin(np.abs(t.points[:, 1] - gy) + 10 * (np.abs(t.points[:, 0] - CX) > 0.03)))
    g = t.points[k]
    yield from L.move((g[0], g[1], HI), yaw=-math.pi / 2, finger=GRASP_OPEN, duration=1.2)
    g = ctx.states()["fish_tape"].points[k]
    yield from L.move((g[0], g[1], g[2] + 0.010), yaw=-math.pi / 2, duration=0.8)
    yield from L.grip(f_closed, 0.6)


def _pick_eyelet(ctx, Rt, f_open, f_closed):
    """Pinch the lug, lift it and turn it so the ring points along +y (towards the conduit)."""
    c = ctx.states()["cable"]
    lug = c.end_plug
    gy = _grip_yaw(_heading(c.end_dir))
    yield from Rt.move((lug[0], lug[1], HI), yaw=gy, finger=f_open, duration=1.5)
    c = ctx.states()["cable"]
    lug = c.end_plug
    yield from Rt.move((lug[0], lug[1], lug[2] + 0.010), yaw=gy, duration=1.0)
    yield from Rt.grip(f_closed, 0.6)
    yield from Rt.move((lug[0], lug[1], HI), yaw=gy, duration=1.0)
    _log(ctx, f"lug lifted (grip yaw {math.degrees(gy):.0f})")
    # Turn the eyelet so the ring points along +y (towards the conduit) and set it down: with this pinch that needs
    # yaw -90 deg, which would leave the wrist camera looking away from the hook. So the lug is put down squared up
    # and re-pinched at yaw +90 deg (same pinch, camera now facing the hook and the mouth).
    yield from Rt.move((SET_LUG[0], SET_LUG[1], HI), yaw=-math.pi / 2, duration=1.5)
    yield from Rt.move((SET_LUG[0], SET_LUG[1], TOP + 0.016), yaw=-math.pi / 2, duration=1.0)
    yield from Rt.grip(f_open, 0.5)
    yield from Rt.move((SET_LUG[0], SET_LUG[1], HI), yaw=-math.pi / 2, duration=0.6)
    c = ctx.states()["cable"]
    lug, h = c.end_plug, _heading(c.end_dir)
    gy2 = min(max(h, -math.pi / 2), math.pi / 2)  # camera along the ring's direction
    _log(ctx, f"lug set down: lug={np.round(lug, 4)} heading={math.degrees(h):.0f}")
    yield from Rt.move((lug[0], lug[1], HI), yaw=gy2, duration=1.2)
    lug = ctx.states()["cable"].end_plug
    yield from Rt.move((lug[0], lug[1], lug[2] + 0.008), yaw=gy2, duration=1.0)
    yield from Rt.grip(f_closed, 0.6)
    yield from Rt.move((lug[0], lug[1], HI), yaw=gy2, duration=0.8)
    yield from Rt.move((lug[0], lug[1], HI), yaw=math.pi / 2, duration=0.5)


def _hook_eyelet(ctx, Rt, f_open):
    """Line the ring's hole up over the prong (closed loop, re-measured just above the prong tip), lower it over the
    prong, check it went on (retry up to 3 times), then let go."""
    st = ctx.states()
    pb = prong_base(st["fish_tape"])
    off = ring_centre(st["cable"]) - Rt.tcp()
    tgt = pb - off
    yield from Rt.move((tgt[0], tgt[1], HI), yaw=math.pi / 2, duration=1.2)
    for attempt in range(3):
        # stage: ring centre 8 mm above the prong tip, re-measured twice
        for k in range(3):
            st = ctx.states()
            pb = prong_base(st["fish_tape"])
            off = ring_centre(st["cable"]) - Rt.tcp()
            tgt = pb - off
            yield from Rt.move((tgt[0], tgt[1], pb[2] + PRONG_Z1 + 0.008 - off[2]), yaw=math.pi / 2,
                               duration=1.0 if k == 0 else 0.5)
            yield from Rt.wait(0.3)
        # lower until the ring centre is 6 mm above the tape axis (ring on the hook stem, prong through it)
        st = ctx.states()
        off = ring_centre(st["cable"]) - Rt.tcp()
        tgt = pb - off
        yield from Rt.move((tgt[0], tgt[1], pb[2] + 0.006 - off[2]), yaw=math.pi / 2, duration=1.0)
        yield from Rt.wait(0.3)
        _log(ctx, f"ring lowered (try {attempt + 1})")
        if hooked(ctx.states()):
            break
    yield from Rt.grip(f_open, 0.5)
    p = Rt.tcp()
    yield from Rt.move((p[0], p[1], HI), yaw=math.pi / 2, duration=0.8)
    yield from Rt.move((0.20, -0.30, HI), yaw=math.pi / 2, duration=1.0)


def _pull(ctx, L, f_closed, *, holding=False, goal=OUT_GOAL + 0.02, max_strokes=4):
    """Hand over hand: grip the tape near the far mouth, pull it STROKE along +y, let go, reach back; repeat until the
    eyelet (or, without a cable on the hook, the hook prong) is ``goal`` past the far mouth."""
    for n in range(max_strokes):
        st = ctx.states()
        lead = ring_centre(st["cable"]) if hooked(st) else prong_base(st["fish_tape"])
        if lead[1] - Y1 >= goal:
            return
        if not holding:
            yield from _grip_tape(ctx, L, f_closed)
        holding = False
        p = L.tcp()
        steps = 10
        for i in range(steps):
            yield from L.move((CX, p[1] + STROKE * (i + 1) / steps, p[2] + 0.004), yaw=-math.pi / 2, duration=0.5)
        _log(ctx, f"stroke {n + 1}")
        yield from L.grip(GRASP_OPEN, 0.4)
        p = L.tcp()
        yield from L.move((p[0], p[1], HI), yaw=-math.pi / 2, duration=0.6)


def demo_script(ctx):
    """FISH_DEMO=correct (default) | nothing | tape_only (pull the tape through without hooking the cable) |
    push (feed the eyelet into the near mouth by hand, off the hook)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    mode = os.environ.get("FISH_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "nothing":
        yield from L.wait(5.0)
        _log(ctx, "end")
        return
    if mode == "push":
        # the tape is pulled out of the conduit first (its prong would block the eyelet), then the right arm pushes
        # the eyelet into the empty near mouth by hand
        yield from _pull(ctx, L, FINGER_CLOSED, goal=0.06)
        yield from L.move((0.25, 0.30, HI), yaw=-math.pi / 2, duration=1.0)
        yield from _pick_eyelet(ctx, Rt, FINGER_OPEN, FINGER_CLOSED)
        st = ctx.states()
        off = ring_centre(st["cable"]) - Rt.tcp()
        x = CX
        yield from Rt.move((x - off[0], Y0 - 0.03 - off[1], HI), yaw=math.pi / 2, duration=1.0)
        yield from Rt.move((x - off[0], Y0 - 0.03 - off[1], FZ + BORE / 2 - off[2]), yaw=math.pi / 2, duration=1.0)
        yield from Rt.move((x - off[0], Y0 + 0.035 - off[1], FZ + BORE / 2 - off[2]), yaw=math.pi / 2, duration=2.0)
        _log(ctx, "pushed in")
        yield from Rt.grip(FINGER_OPEN, 0.5)
        yield from Rt.move((0.20, -0.30, HI), yaw=math.pi / 2, duration=1.0)
        _log(ctx, "end")
        return

    # Correct: the left arm pinches the tape at the far mouth and holds it steady while the right arm hooks the
    # eyelet over the prong; then the left arm pulls the tape back hand over hand until the eyelet is 7 cm out.
    holding = False
    if mode != "tape_only":
        yield from ctx.both(_grip_tape(ctx, L, FINGER_CLOSED), _pick_eyelet(ctx, Rt, FINGER_OPEN, FINGER_CLOSED))
        _log(ctx, "eyelet picked")
        yield from _hook_eyelet(ctx, Rt, FINGER_OPEN)
        _log(ctx, "hooked")
        holding = True
    yield from _pull(ctx, L, FINGER_CLOSED, holding=holding)
    yield from L.move((0.25, 0.30, HI), yaw=-math.pi / 2, duration=1.0)
    yield from L.wait(1.0)
    _log(ctx, "end")
