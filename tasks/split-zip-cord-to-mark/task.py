"""Split a zip cord apart to the white mark.

On the station table a grey clamp block (40 x 30 x 20 mm) stands in front of the left arm at x 30 cm, y +37.6 cm.
A brown two-conductor zip cord (lamp cord) leaves its -y face and lies straight along -y at x 30 cm: two 4 mm legs
side by side, bonded along their web. The near leg (towards the robot) is 54 cm long, the far leg 50 cm, so the near
leg sticks out 4 cm past the far leg's end; the last 2 cm of the far leg are already split from the near leg. A white
tape band 2 cm wide is wrapped round both legs, centred 15 cm from the end of the far (short) leg (y +0.5 cm).
The clamp holds the cord end 3 mm above the table.

Goal: separate the two legs from the free end up to the white band, not past it, then let go.

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
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Separate the two legs of the brown lamp cord, starting at its free end, up to the white band. "
               "Do not split it past the band. Let go of the cord when you are done.")

GREY = (0.55, 0.57, 0.60)
BROWN = (0.33, 0.20, 0.11)
WHITE = (0.95, 0.95, 0.93)

# -- geometry ----------------------------------------------------------------------------------------------------------
LX = 0.30  # cord centre line (x)
R = 0.002  # 4 mm legs
GAP = 0.001  # web between the legs
D = 2 * R + GAP  # leg centre spacing
SEG = 0.02
LEN_A, LEN_B = 0.54, 0.50  # near (peeled) leg, far leg
CLAMP_FACE_Y = 0.361
START_Y = CLAMP_FACE_Y - 0.006  # cord ends 6 mm in front of the clamp face
YAW = -math.pi / 2
Z0 = TOP + R + 0.0005
CLAMP_LIFT = 0.003  # the clamp holds the cord end 3 mm up, clear of the table (no contact fighting the weld)
XA, XB = LX - D / 2, LX + D / 2
N_A, N_B = round(LEN_A / SEG), round(LEN_B / SEG)  # 27, 25 segments
PRESPLIT = 1  # the last pair (1 cm from the short leg's end) is not bonded
N_BOND = N_B - PRESPLIT  # pairs 0..23 bonded
MARK_FROM_END = 0.15  # band centre, from the short leg's end
BAND_SEG = N_B - 1 - round((MARK_FROM_END - SEG / 2) / SEG)  # segment 17 of both legs (14..16 cm from the end)
MARK_Y = START_Y - LEN_B + MARK_FROM_END

F_BOND = 2.0  # N: break force of one web bond (pull only)
BOND_SLACK = 0.0002

CLAMP = Box("clamp_block", pos=(LX, CLAMP_FACE_Y + 0.015, TOP + 0.010), size=(0.040, 0.030, 0.020), color=GREY)
CORD_A = cable("leg_near", start=(XA, START_Y, Z0), yaw=YAW, length=LEN_A, radius=R, segment=SEG,
               color=BROWN, bend_stiffness=0.01, bend_damping=0.001, friction=0.5)
CORD_B = cable("leg_far", start=(XB, START_Y, Z0), yaw=YAW, length=LEN_B, radius=R, segment=SEG,
               color=BROWN, bend_stiffness=0.01, bend_damping=0.001, friction=0.5)
assert CORD_A.segments == N_A and CORD_B.segments == N_B


def bond_from_end(i: int) -> float:
    """Distance (m) of bond i (pair of segments i) from the short leg's free end."""
    return LEN_B - (i + 0.5) * SEG


# -- breakable web bonds between the legs, clamp weld ------------------------------------------------------------
_EV: dict = {"paths": {}}


def _install_welds() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_zip_welds", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        name = getattr(cfg, "cable_name", "")
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
        from yam_arena.station import linear_rgb

        stage = get_current_stage()
        if name in (CORD_A.name, CORD_B.name):
            _EV["paths"][name] = str(prim_path)
            look = f"{prim_path}/Looks/band"
            mat = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(WHITE), roughness=0.6)
            mat.func(look, mat)
            bind_visual_material(f"{prim_path}/seg_{BAND_SEG}/geom", look, stage=stage)
            # clamp: weld seg_0 to the world 3 mm up (outside the articulation; clear of the table)
            seg0 = f"{prim_path}/seg_0"
            fj = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/clamp_weld")
            fj.CreateBody1Rel().SetTargets([Sdf.Path(seg0)])
            spec = CORD_A if name == CORD_A.name else CORD_B
            fj.CreateLocalPos0Attr(Gf.Vec3f(*map(float, spec.point_at(SEG / 2) + np.array([0.0, 0.0, CLAMP_LIFT]))))
            fj.CreateLocalRot0Attr(Gf.Quatf(math.cos(YAW / 2), 0.0, 0.0, math.sin(YAW / 2)))
            fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            fj.CreateExcludeFromArticulationAttr(True)
        if len(_EV["paths"]) == 2 and name in (CORD_A.name, CORD_B.name):
            pa, pb = _EV["paths"][CORD_A.name], _EV["paths"][CORD_B.name]
            # B lies at +D along world x = +y in the legs' local frame (cable heading -y)
            # web bond: a distance joint (pulls only, never pushes: a pinch across the pair goes through contact)
            for i in range(N_BOND):
                j = UsdPhysics.DistanceJoint.Define(stage, f"{pa}/bond_{i}")
                j.CreateBody0Rel().SetTargets([Sdf.Path(f"{pa}/seg_{i}")])
                j.CreateBody1Rel().SetTargets([Sdf.Path(f"{pb}/seg_{i}")])
                j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
                j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
                j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
                j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
                j.CreateMinDistanceAttr(-1.0)
                j.CreateMaxDistanceAttr(D + BOND_SLACK)
                j.CreateCollisionEnabledAttr(True)
                j.CreateExcludeFromArticulationAttr(True)
                j.CreateBreakForceAttr(F_BOND)
        return prim

    spawn._zip_welds = True
    cablekit._spawn_cable = spawn


_install_welds()

OBJECTS = [CLAMP, CORD_A, CORD_B]

TWIN_EXTRA = [
    {"name": "web", "kind": "joint", "real": "PVC web of SPT-1 lamp cord", "note": "breakable weld every 2 cm (2 N)"},
    {"name": "clamp_weld", "kind": "joint", "real": "screw cord clamp in the grey block", "note": "holds both legs"},
    {"name": "band", "kind": "decal", "size_mm": [20, 9], "colour": "white", "pose": [LX, MARK_Y, 0.004]},
]

# -- episode state (latched) -----------------------------------------------------------------------------------------
_START_A = CORD_A.start_points() + np.array([0, 0, CLAMP_LIFT])
_START_B = CORD_B.start_points() + np.array([0, 0, CLAMP_LIFT])
_ST: dict = {"broken": np.zeros(N_BOND, bool), "unclamped": False}
SEP_BROKEN = D + 0.003  # pair centres further apart than this: that bond is broken
CLAMP_MOVE = 0.01  # a leg's first segment more than 1 cm from its start (NaN guard; the weld does not break)


def _reset_episode() -> None:
    _ST["broken"][:] = False
    _ST["unclamped"] = False


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


def _track(objects) -> None:
    a, b = objects[CORD_A.name].points, objects[CORD_B.name].points
    if not (np.isfinite(a).all() and np.isfinite(b).all()):
        _ST["unclamped"] = True
        return
    sep = np.linalg.norm(a[:N_BOND] - b[:N_BOND], axis=1)
    _ST["broken"] |= sep > SEP_BROKEN
    if np.linalg.norm(a[0] - _START_A[0]) > CLAMP_MOVE or np.linalg.norm(b[0] - _START_B[0]) > CLAMP_MOVE:
        _ST["unclamped"] = True


def split_length(objects) -> float:
    """Split length (m): from the short leg's end to the first joined point (first intact bond from the end)."""
    _track(objects)
    br = _ST["broken"]
    for i in range(N_BOND - 1, -1, -1):
        if not br[i]:
            return bond_from_end(i)
    return LEN_B


def overshoot(objects) -> bool:
    """Legs separated anywhere more than 1 cm past the band (towards the clamp)."""
    _track(objects)
    far = [i for i in range(N_BOND) if bond_from_end(i) > MARK_FROM_END + 0.01 + 1e-6]
    return bool(_ST["broken"][far].any()) or split_length(objects) > MARK_FROM_END + 0.01 + 1e-6


# -- grippers / release ----------------------------------------------------------------------------------------------
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_zip_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._zip_hooked = True
    cls._task_step = _task_step


_install_env_hook()

RELEASE_DIST = 0.03


def grippers() -> list[np.ndarray]:
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


def released(objects) -> bool:
    pts = np.vstack([objects[CORD_A.name].polyline(), objects[CORD_B.name].polyline()])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


# -- scoring ---------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    """0 if the cord came out of the clamp, the legs are split anywhere more than 1 cm past the band, or a gripper is
    within 3 cm of the cord. Else by split length S (short leg's end to the first joined point), band at 15 cm:
    14-16 cm -> 1.0; 12-14 -> 0.6; 9-12 -> 0.3; under 9 cm -> 0."""
    _track(objects)
    if _ST["unclamped"] or overshoot(objects) or not released(objects):
        return 0.0
    s = split_length(objects) + 1e-6
    if s >= MARK_FROM_END - 0.01:
        return 1.0
    if s >= MARK_FROM_END - 0.03:
        return 0.6
    if s >= MARK_FROM_END - 0.06:
        return 0.3
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects[CORD_A.name].resting(max_speed=0.02) \
        and objects[CORD_B.name].resting(max_speed=0.02)


def distance(objects) -> float:
    """|split length - 15 cm| (m); 1.0 once a disqualifier fired."""
    _track(objects)
    if _ST["unclamped"] or overshoot(objects):
        return 1.0
    return abs(split_length(objects) - MARK_FROM_END)


def progress(objects) -> dict:
    _track(objects)
    s = split_length(objects)
    return {"split_started": float(s >= 0.05), "split_half": float(s >= 0.09), "split_near_band": float(s >= 0.13),
            "at_band": float(abs(s - MARK_FROM_END) <= 0.01), "no_overshoot": float(not overshoot(objects)),
            "clamp_kept": float(not _ST["unclamped"]), "released": float(released(objects))}


VARIATION = {"cable:leg_near": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},  # both legs and the clamp move together
             "cable:leg_far": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},
             "obj:clamp_block": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},
             "group:cord_and_clamp": {"dx": 0.02, "dy": 0.015, "yaw": 0.0}}

# -- closed-loop scripted demo ---------------------------------------------------------------------------------------
PAD = 0.010
HI = TOP + 0.07
PINCH_Y = MARK_Y + 0.008  # left pinch centre: covers the band bond, leaves the next bond (2 cm nearer the end) free
GRIP_SEG = N_A - 1  # right arm takes the near leg's last segment (beyond the short leg's end)
THETA = math.radians(60)  # peel angle of the free near leg away from the cord line, towards the robot
PULL = 0.006  # beyond the free length: loads the crack-tip bond


def _seg_y(i: int) -> float:
    return START_Y - (i + 0.5) * SEG


def demo_script(ctx):
    """Left arm pinches both legs just behind the white band (the tear stop); right arm takes the near leg's tail and
    peels it back in a V at 60 degrees, a bond or two per step, closed loop on the measured split, and stops when the
    split reaches the band. Both let go.

    ZIP_CHEAT (test only): none | hold (split to the band, keep both grippers closed) | yank (no pinch: one arm pulls
    the tail fast straight sideways) | axial (pull both legs together along the cord, away from the clamp)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("ZIP_CHEAT", "")

    def log(tag):
        st = ctx.states()
        br = "".join("x" if v else "." for v in _ST["broken"])
        print(f"[demo t={ctx.t:5.1f}] {tag}: S={split_length(st) * 100:.0f}cm score={score(st):.2f} "
              f"unclamped={_ST['unclamped']} over={overshoot(st)} bonds={br}", flush=True)
        if os.environ.get("ZIP_DEBUG"):
            print("   seg0 off mm", np.round((st[CORD_A.name].points[0] - _START_A[0]) * 1000, 2),
                  np.round((st[CORD_B.name].points[0] - _START_B[0]) * 1000, 2), flush=True)

    log("start")
    if cheat == "none":
        yield from L.wait(5.0)
        log("end")
        return
    st = ctx.states()
    ga = st[CORD_A.name].points[GRIP_SEG]
    ry = math.pi / 2  # right fingers across the near leg (along y); wrist cam faces +y, the crack
    ly = -math.pi / 2  # left wrist cam faces -y, the split
    pz = Z0 + PAD
    # 1) left over the band, right over the near leg's tail
    if cheat == "axial":
        gb = st[CORD_B.name].points[N_B - 1]
        yield from Rt.move((LX, gb[1] + 0.005, HI), yaw=ry, finger=FINGER_OPEN, duration=1.5)
        yield from Rt.move((LX, gb[1] + 0.005, pz), yaw=ry, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED)
        log("both legs gripped")
        for k in range(8):
            yield from Rt.move((LX, gb[1] + 0.005 - 0.01 * (k + 1), pz + 0.005), yaw=ry, duration=0.4)
            log(f"axial pull {k}")
        yield from Rt.grip(FINGER_OPEN)
        yield from Rt.move((LX, gb[1] - 0.08, HI), yaw=ry, duration=0.8)
        yield from L.wait(2.0)
        log("end")
        return
    moves = [Rt.move((ga[0], ga[1], HI), yaw=ry, finger=FINGER_OPEN, duration=1.5)]
    if cheat != "yank":
        moves.insert(0, L.move((LX, PINCH_Y, HI), yaw=ly, finger=FINGER_OPEN, duration=1.5))
        yield from ctx.both(*moves)
        yield from ctx.both(L.move((LX, PINCH_Y, pz), yaw=ly, duration=0.8),
                            Rt.move((ga[0], ga[1], pz), yaw=ry, duration=0.8))
        yield from ctx.both(L.grip(FINGER_CLOSED), Rt.grip(FINGER_CLOSED))
    else:
        yield from moves[0]
        yield from Rt.move((ga[0], ga[1], pz), yaw=ry, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED)
    log("gripped")
    if cheat == "yank":
        yield from Rt.move((ga[0] - 0.14, ga[1] + 0.03, pz + 0.01), yaw=ry, duration=0.3)
        yield from Rt.wait(0.5)
        log("yanked")
    else:
        # 2) peel: crack between the first intact bond and the broken one beyond it
        lift = pz + 0.008
        for it in range(40):
            st = ctx.states()
            s = split_length(st)
            if s >= MARK_FROM_END - 1e-6 or _ST["unclamped"] or overshoot(st):
                break
            i_tip = int(round((LEN_B - s) / SEG - 0.5))  # first intact bond
            yc = _seg_y(i_tip) - SEG / 2  # crack location (between bonds)
            free = yc - (START_Y - (GRIP_SEG + 0.5) * SEG)  # free near-leg length from crack to grasp
            r = free + PULL
            tx, ty = XA - r * math.sin(THETA), yc - r * math.cos(THETA)
            yaw = math.pi / 2 - THETA
            yield from Rt.move((tx, ty, lift), yaw=yaw, duration=0.6)
            yield from Rt.wait(0.3)
            log(f"peel step {it}")
    # 3) let go
    if cheat == "hold":
        log("holding")
        yield from L.wait(3.0)
        log("end (held)")
        return
    yield from Rt.grip(FINGER_OPEN, 0.5)
    rp = Rt.tcp()
    yield from Rt.move((rp[0], rp[1], HI), yaw=math.pi / 2 - THETA, duration=0.6)
    log("right let go")
    if cheat != "yank":
        yield from L.grip(FINGER_OPEN, 0.5)
        yield from L.move((LX, PINCH_Y, HI), yaw=ly, duration=0.6)
    yield from ctx.both(L.move((0.25, 0.20, HI + 0.03), yaw=ly, duration=0.8),
                        Rt.move((0.20, -0.25, HI + 0.03), yaw=math.pi / 2, duration=0.8))
    for _ in range(4):
        yield from L.wait(0.5)
    log("end")

