"""On the station table a spring cord reel (red housing, 9 cm wide, 6 cm tall) lies along the robot's left-right axis
on the left, 42 cm in front of the robot. Its black 5 mm cord comes out of a slot in the reel's right-hand face and
runs straight to the right, 34 cm, to a yellow work light (200 x 30 x 20 mm bar) parked against a grey stop in front
of the right arm. The reel always pulls its cord in (about 1 N with the cord fully out, 0.5 N near home). In front of
the reel face is a grey dock: two low rails that hold the light in line with the cord slot. The light must be
brought home into the dock without ever being let go while the reel can pull it: a free light would be yanked
across the table and slammed into the reel. Only the right arm reaches the parked light and only the left arm
reaches the dock, so the light has to change hands while the reel is pulling.

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

INSTRUCTION = ("Let the cord reel wind its cord back in and put the yellow work light into the grey dock in front of "
               "the reel, then let go of the light.")

# -- layout (m; x forward from the robot, y to the robot's left) ------------------------------------------------------
LINE_X = float(os.environ.get("REEL_LINE_X", "0.42"))  # the cord's line (along y)
MOUTH_Y = float(os.environ.get("REEL_MOUTH_Y", "0.29"))  # the reel's right-hand face (cord slot)
PARK_REAR_Y = -0.05  # the parked light's cord end
R = 0.0025  # 5 mm cord
LIGHT = Plug(size=(0.20, 0.030, 0.020), mass=0.025, color=(0.95, 0.78, 0.05))
LL, LW, LH = LIGHT.size
CORD_Z = TOP + LH / 2 + 0.0005
CORD_Y0 = MOUTH_Y + 0.012  # cord's inner end, just inside the slot
CORD_LEN = CORD_Y0 - PARK_REAR_Y
CORD = cable("cord", start=(LINE_X, CORD_Y0, CORD_Z), yaw=-math.pi / 2, length=CORD_LEN, radius=R,
             end_plug=LIGHT, cable_mass=0.03, color=(0.05, 0.05, 0.05), bend_stiffness=0.01, bend_damping=0.001)

# reel housing: a hollow red shell round a 12 x 16 mm cord tunnel along the line
RED = (0.72, 0.10, 0.08)
GREY = (0.60, 0.62, 0.64)
H_W, H_H, H_L = 0.09, 0.06, 0.36
TUN_W, TUN_H = 0.012, 0.016
_hy = MOUTH_Y + H_L / 2
_side_w = (H_W - TUN_W) / 2
HOUSING = [
    Box("reel_side_front", pos=(LINE_X + TUN_W / 2 + _side_w / 2, _hy, TOP + H_H / 2), size=(_side_w, H_L, H_H),
        color=RED),
    Box("reel_side_back", pos=(LINE_X - TUN_W / 2 - _side_w / 2, _hy, TOP + H_H / 2), size=(_side_w, H_L, H_H),
        color=RED),
    Box("reel_roof", pos=(LINE_X, _hy, TOP + TUN_H + (H_H - TUN_H) / 2), size=(TUN_W, H_L, H_H - TUN_H), color=RED),
]
REEL_PAYOUT = CORD_LEN  # travel of the cord's inner end into the housing (limit)
REEL_K = 1.4  # N/m
REEL_TARGET = 0.72  # m beyond the start: 1.0 N with the cord fully out, ~0.5 N with the light home
REEL_DAMP = 0.2

# dock: two rails in front of the reel face (inner gap 40 mm) under the light's free half; the 12 cm next to the
# face are left open so a gripper can reach the light's cord end and middle
DOCK_GAP, RAIL_T, RAIL_H = LW + 0.010, 0.005, 0.006
DOCK_Y0, DOCK_Y1 = MOUTH_Y - LL - 0.005, MOUTH_Y - 0.12
DOCK = [Box(f"dock_rail_{k}", pos=(LINE_X + s * (DOCK_GAP / 2 + RAIL_T / 2), (DOCK_Y0 + DOCK_Y1) / 2, TOP + RAIL_H / 2),
            size=(RAIL_T, DOCK_Y1 - DOCK_Y0, RAIL_H), color=GREY) for k, s in (("front", 1), ("back", -1))]
# parking stop: two 10 mm blocks either side of the cord, against the light's cord end
STOP_H = 0.010
STOP = [Box(f"park_stop_{k}", pos=(LINE_X + s * (0.006 + 0.0075), PARK_REAR_Y + 0.002 + 0.005, TOP + STOP_H / 2),
            size=(0.015, 0.010, STOP_H), color=GREY) for k, s in (("front", 1), ("back", -1))]

OBJECTS = [*HOUSING, *DOCK, *STOP, CORD]

TWIN_EXTRA = [
    {"name": "reel_spring", "kind": "joint",
     "real": "spring-driven retractable cord reel (constant-force spring, ~1 N pull)",
     "note": "sim: the cord's inner end slides on a spring-loaded straight track inside the housing"},
]


# -- the reel: the cord's inner end slides along the line on a prismatic joint with a spring drive ------------------
def _install_reel() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_retract_reel", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CORD.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        stage = get_current_stage()
        seg = f"{prim_path}/seg_0"
        world = UsdGeom.Xformable(stage.GetPrimAtPath(seg)).ComputeLocalToWorldTransform(0)
        q = world.ExtractRotationQuat()
        tip = world.Transform(Gf.Vec3d(-CORD.seg_len / 2, 0.0, 0.0))
        j = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/reel")
        j.CreateBody1Rel().SetTargets([Sdf.Path(seg)])
        j.CreateAxisAttr("X")  # +x = along the cord, out of the reel (payout)
        j.CreateLocalPos0Attr(Gf.Vec3f(*tip))
        j.CreateLocalRot0Attr(Gf.Quatf(q.GetReal(), *q.GetImaginary()))
        j.CreateLocalPos1Attr(Gf.Vec3f(-CORD.seg_len / 2, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLowerLimitAttr(-REEL_PAYOUT)
        j.CreateUpperLimitAttr(0.01)
        j.CreateExcludeFromArticulationAttr(True)
        drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(REEL_K)
        drv.CreateDampingAttr(REEL_DAMP)
        drv.CreateTargetPositionAttr(-REEL_TARGET)
        return prim

    spawn._retract_reel = True
    cablekit._spawn_cable = spawn


_install_reel()

# -- env handle (gripper TCPs and finger openings), per-episode memory --------------------------------------------
_ENV = {"env": None, "step": 0}
_MON = {"violated": False, "last_step": -1, "prev_out": None, "max_speed": 0.0, "free_steps": 0, "handover": False,
        "both_held": False}


def _reset_episode() -> None:
    _MON.update(violated=False, last_step=-1, prev_out=None, max_speed=0.0, free_steps=0, handover=False,
                both_held=False)
    _ENV["step"] = 0


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline import)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_reel_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            _ENV["step"] += 1
            return base(self, *a, **k)

        _task_step._reel_hooked = True
        cls._task_step = _task_step
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


_install_hooks()

CLOSED_MAX = 0.030  # |finger joint| below this = fingers closed (open is 0.047)
HOLD_DIST = 0.030  # closed gripper TCP within 3 cm of the light's centre line = holding the light
RELEASE_DIST = 0.030  # at the end every TCP at least 3 cm from the light and the cord
WOUND = 0.02  # m: at most this much cord out = "wound in"; below it a free light is no longer a violation
MOVE_V = 0.03  # m/s of cord retraction that counts as "the reel pulling the cord in"
SNAP_V = 0.25  # m/s: faster retraction = the light got away (it slid out of a closed gripper), even if "held"


def grippers() -> list[tuple[np.ndarray, float]]:
    """(TCP position in env frame, |finger joint|) per arm; [] offline."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    out = []
    for k in ("left_robot", "right_robot"):
        art = base.scene[k]
        jp = getattr(art.data.joint_pos, "torch", art.data.joint_pos)[0].detach().cpu().numpy()
        f = float(abs(jp[list(art.joint_names).index("joint7")]))
        out.append((np.asarray(tcp_world(art), dtype=np.float64) - origin, f))
    return out


# -- geometry ---------------------------------------------------------------------------------------------------------
def light_pose(objects) -> tuple[np.ndarray, np.ndarray]:
    """Light centre and unit axis from its cord end (rear) to its free end (front)."""
    c = objects[CORD.name]
    a = np.asarray(c.end_dir, dtype=float)
    return np.asarray(c.end_plug, dtype=float), a / (np.linalg.norm(a) + 1e-12)


def cord_out(objects) -> float:
    """Cord out of the reel (m): cord length minus what lies inside the housing behind the face."""
    c = objects[CORD.name]
    return float(CORD_LEN - (c.start_tip[1] - MOUTH_Y))


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def holders(objects) -> list[bool]:
    """Per arm: fingers closed and TCP within 3 cm of the light's centre line."""
    p, a = light_pose(objects)
    ra, fa = p - a * (LL / 2 - 0.01), p + a * (LL / 2 - 0.01)
    return [f < CLOSED_MAX and _seg_dist(g, ra, fa) <= HOLD_DIST for g, f in grippers()]


def released(objects) -> bool:
    """Every gripper centre at least 3 cm from the light (its whole body axis) and from the cord."""
    p, a = light_pose(objects)
    ra, fa = p - a * LL / 2, p + a * LL / 2
    poly = objects[CORD.name].polyline()
    for g, _ in grippers():
        if _seg_dist(g, ra, fa) < RELEASE_DIST:
            return False
        if min(_seg_dist(g, u, v) for u, v in zip(poly[:-1], poly[1:])) < RELEASE_DIST:
            return False
    return True


def in_dock(objects) -> bool:
    """Light centre between the rails, along the dock, lying on the table."""
    p, _ = light_pose(objects)
    return bool(abs(p[0] - LINE_X) <= (DOCK_GAP - LW) / 2 + 0.004 and DOCK_Y0 + 0.05 <= p[1] <= MOUTH_Y
                and p[2] <= TOP + LH / 2 + 0.006)


def seated(objects) -> bool:
    """In the dock with its cord end within 1 cm of the reel face and its axis within 10 deg of the cord line."""
    if not in_dock(objects):
        return False
    p, a = light_pose(objects)
    rear = p - a * LL / 2
    ang = math.degrees(math.acos(min(1.0, abs(float(a[1])))))
    return bool(MOUTH_Y - rear[1] <= 0.010 and ang <= 10.0)


def _finite(objects) -> bool:
    c = objects[CORD.name]
    return bool(np.isfinite(c.points).all() and c.end_plug is not None and np.isfinite(c.end_plug).all())


def _track(objects) -> None:
    """Once per control step: latch a 'free moment': the reel moves the cord faster than 3 cm/s while no closed
    gripper is on the light, or draws it in faster than 25 cm/s at all (the light slipped out of a grip), unless at
    most 2 cm of cord is still out."""
    if not _finite(objects):
        _MON["violated"] = True
        return
    step = _ENV["step"]
    if step == _MON["last_step"]:
        return
    out = cord_out(objects)
    prev, dt_steps = _MON["prev_out"], step - _MON["last_step"]
    _MON["last_step"], _MON["prev_out"] = step, out
    if prev is None or _ENV["env"] is None or dt_steps <= 0:
        return
    speed = (prev - out) / (0.05 * dt_steps)  # retraction speed, m/s
    _MON["max_speed"] = max(_MON["max_speed"], speed)
    h = holders(objects)
    if all(h) and len(h) == 2:
        _MON["both_held"] = True
    near_home = out <= WOUND
    if not near_home and (speed > SNAP_V or (abs(speed) > MOVE_V and not any(h))):
        _MON["free_steps"] += 1
        _MON["violated"] = True


def score(objects) -> float:
    """End states. 0 after any free moment, if a gripper is still within 3 cm of the light or cord, or if more than
    2 cm of cord is out of the reel. Otherwise (cord wound in, released): 1.0 if the light is seated in the dock
    (centre between the rails, cord end within 1 cm of the reel face, axis within 10 deg of the line), else 0.5
    if both grippers held the light at the same moment (handed over and wound in safely, but left outside the dock
    or crooked), else 0."""
    _track(objects)
    if _MON["violated"] or not _finite(objects):
        return 0.0
    if not released(objects) or cord_out(objects) > WOUND:
        return 0.0
    if seated(objects):
        return 1.0
    return 0.5 if _MON["both_held"] else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects[CORD.name].resting(max_speed=0.02)


def distance(objects) -> float:
    _track(objects)
    if _MON["violated"] or not _finite(objects):
        return 3.0
    p, a = light_pose(objects)
    return float(abs(MOUTH_Y - (p - a * LL / 2)[1]))


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    _track(objects)
    out = cord_out(objects) if _finite(objects) else CORD_LEN
    return {"retracted": round(float(np.clip((CORD_LEN - out) / (CORD_LEN - 0.01), 0, 1)), 2),
            "handed_over": int(_MON["both_held"]), "in_dock": int(_finite(objects) and in_dock(objects)),
            "seated": int(_finite(objects) and seated(objects)), "never_free": int(not _MON["violated"]),
            "released": int(_finite(objects) and released(objects))}


VARIATION = {"cable:cord": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}  # the cord is fixed to the reel's line


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------
GRIP_IN = 0.025  # grasp points this far in from each end of the light (15 cm apart)
PINCH = -0.024  # finger opening for the approach (pads ~52 mm on a 30 mm light)
HAND_Y = -0.02  # right arm's grasp point at the hand-over


def _log(ctx, tag):
    st = ctx.states()
    p, a = light_pose(st)
    print(f"[demo] {tag}: out={cord_out(st):.3f} light=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) ax=({a[0]:.2f},{a[1]:.2f},{a[2]:.2f}) "
          f"hold={holders(st)} viol={_MON['violated']} vmax={_MON['max_speed']:.2f} dock={in_dock(st)} "
          f"seat={seated(st)} score={score(st)}", flush=True)


def _grip_pt(ctx, from_rear: bool):
    p, a = light_pose(ctx.states())
    d = LL / 2 - GRIP_IN
    return p - a * d if from_rear else p + a * d


def _level(ctx, arm, yaw, n=2):
    """Closed loop: lower/raise the held light until it rests on the table (a pinched light rides up ~1 cm)."""
    for _ in range(n):
        p, _ = light_pose(ctx.states())
        dz = float(np.clip(p[2] - (TOP + LH / 2 + 0.0015), -0.01, 0.015))
        g = arm.tcp()
        z = max(g[2] - dz, TOP + 0.016)  # fingertips stay >= 3 mm above the table
        if abs(z - g[2]) < 0.002:
            return
        yield from arm.move((g[0], g[1], z), yaw=yaw, duration=0.4)


def _tier05(ctx, L, yaw, hi):
    """Partial-credit variant (0.5), run from the docked, still-held end state: the cord is wound in, so the light
    may be re-gripped. The left arm opens, slides to the light's middle (staying within 3 cm of it, so it never counts
    as released), pinches it there, lifts it 1.5 cm, swings it 40 deg about its cord end out of the rails, sets it
    down slanted at the slot and lets go: wound in and released, but not in the dock."""
    th = math.radians(40)
    p, a = light_pose(ctx.states())
    rear = p - a * LL / 2
    g = L.tcp()
    yield from L.grip(PINCH, 0.5)
    yield from L.move((g[0], g[1], g[2] + 0.004), yaw=yaw, duration=0.4)
    zc = float(p[2]) + 0.008
    yield from L.move((float(p[0]), float(p[1]), zc + 0.004), yaw=yaw, duration=1.2)
    yield from L.move((float(p[0]), float(p[1]), zc), yaw=yaw, duration=0.5)
    yield from L.grip(0.0, 0.8)
    _log(ctx, "middle pinched")
    for k in range(0, 5):  # lift 1.5 cm, then swing about the cord end in 10 deg steps
        t = th * k / 4
        fdir = np.array([-math.sin(t), -math.cos(t)])  # cord end -> free end (swings toward the robot)
        c = rear[:2] + fdir * LL / 2
        yield from L.move((float(c[0]), float(c[1]), zc + 0.015), yaw=yaw - t, duration=0.6 if k else 0.5)
    yield from L.move((float(c[0]), float(c[1]), zc + 0.001), yaw=yaw - th, duration=0.8)
    _log(ctx, "slanted at slot")
    yield from L.grip(PINCH, 0.5)
    g = L.tcp()
    yield from L.move((g[0], g[1], hi), yaw=yaw - th, duration=0.8)
    yield from L.move((0.20, 0.40, hi), yaw=0.0, duration=1.0)
    for k in range(4):  # 6 s: shows the slanted light has come to rest
        yield from L.wait(1.5)
        _log(ctx, f"settle {k}")
    _log(ctx, "end")


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("REEL_CHEAT", "")
    yaw = math.pi / 2  # fingers close across the light (along x); wrist cameras look along +y, toward the reel
    gz = TOP + LH / 2 + 0.010  # TCP 10 mm above the light's centre: fingertips 3 mm below its centre
    hi = TOP + 0.07
    _log(ctx, "start")
    if cheat == "nothing":
        yield from L.wait(4.0)
        _log(ctx, "end")
        return

    # 1) right arm takes the parked light near its free end, lifts it over the stop and lowers it to the table line
    g = _grip_pt(ctx, from_rear=False)
    yield from Rt.move((g[0], g[1], hi), yaw=yaw, finger=PINCH, duration=1.5)
    g = _grip_pt(ctx, from_rear=False)
    yield from Rt.move((g[0], g[1], gz), yaw=yaw, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED, 0.6)
    yield from Rt.move((g[0], g[1], gz + 0.016), yaw=yaw, duration=0.6)
    yield from Rt.move((g[0], g[1] + 0.03, gz + 0.016), yaw=yaw, duration=0.6)
    yield from Rt.move((LINE_X, g[1] + 0.04, gz + 0.002), yaw=yaw, duration=0.5)
    yield from _level(ctx, Rt, yaw)
    _log(ctx, "right has light")
    if cheat == "letgo":  # lift it out and let go: the reel yanks it home
        yield from Rt.grip(FINGER_OPEN, 0.4)
        yield from Rt.move((g[0], g[1], hi), yaw=yaw, duration=0.8)
        yield from L.wait(3.0)
        _log(ctx, "end")
        return
    if cheat == "hold_still":
        yield from Rt.wait(5.0)
        _log(ctx, "end")
        return

    if cheat == "diagonal":  # right arm alone, on its closest diagonal to the slot (cord end ~4 cm short), lets go
        g = Rt.tcp()
        yield from Rt.move((0.36, -0.06, g[2]), yaw=yaw, duration=2.0)
        yield from Rt.move((0.30, 0.11, g[2]), yaw=0.98, duration=3.0)
        _log(ctx, "right at closest diagonal")
        yield from Rt.grip(FINGER_OPEN, 0.4)
        yield from Rt.move((0.30, 0.11, hi), yaw=0.98, duration=0.8)
        yield from Rt.move((0.20, -0.30, hi), yaw=0.0, duration=1.2)
        yield from L.wait(3.0)
        _log(ctx, "end")
        return

    # 2) right arm walks it toward the reel along the line, as far as it reaches; the reel takes up the cord
    g = Rt.tcp()
    yield from Rt.move((LINE_X, HAND_Y, g[2]), yaw=yaw, duration=3.0)
    _log(ctx, "right at hand-over")
    if cheat == "onearm":  # let go at the right arm's reach limit: the reel pulls it the rest of the way
        yield from Rt.grip(FINGER_OPEN, 0.4)
        yield from Rt.move((LINE_X - 0.05, HAND_Y, hi), yaw=yaw, duration=0.8)
        yield from L.wait(3.0)
        _log(ctx, "end")
        return

    # 3) hand-over: left arm closes on the cord end of the light, then the right arm lets go
    g = _grip_pt(ctx, from_rear=True)
    yield from L.move((g[0], g[1], hi), yaw=yaw, finger=PINCH, duration=1.5)
    g = _grip_pt(ctx, from_rear=True)
    yield from L.move((g[0], g[1], max(g[2] + 0.006, TOP + 0.016)), yaw=yaw, duration=1.0)
    yield from L.grip(FINGER_CLOSED, 0.8)
    yield from L.wait(0.3)
    _log(ctx, "both hold")
    if cheat == "drop":  # both let go during the hand-over
        yield from ctx.both(L.grip(FINGER_OPEN, 0.4), Rt.grip(FINGER_OPEN, 0.4))
        yield from L.wait(3.0)
        _log(ctx, "end")
        return
    yield from Rt.grip(PINCH, 0.8)
    rt = Rt.tcp()
    yield from Rt.move((rt[0], rt[1], hi), yaw=yaw, duration=0.6)
    yield from Rt.move((0.20, -0.30, hi), yaw=0.0, duration=1.2)
    _log(ctx, "left has light")

    # 4) left arm guides it home along the line, the light sliding on the table between the rails; it stops
    #    with the cord end ~8 mm short of the reel face (the reel draws it the last few mm once it is let go)
    yield from _level(ctx, L, yaw)
    for _ in range(5):
        p, a = light_pose(ctx.states())
        gap = MOUTH_Y - (p - a * LL / 2)[1]
        if gap <= 0.006:
            break
        g = L.tcp()
        yield from L.move((g[0] + (LINE_X - p[0]), g[1] + gap - 0.003, g[2]), yaw=yaw,
                          duration=max(0.5, gap / 0.06))
        yield from _level(ctx, L, yaw)
    _log(ctx, "home")
    if cheat == "hold":  # docked, but never let go
        yield from L.wait(3.0)
        _log(ctx, "end")
        return

    if cheat == "tier05":
        yield from _tier05(ctx, L, yaw, hi)
        return

    # 5) let go, rise, park
    yield from L.grip(PINCH, 0.5)
    g = L.tcp()
    yield from L.move((g[0], g[1], hi), yaw=yaw, duration=0.8)
    yield from L.move((0.20, 0.40, hi), yaw=0.0, duration=1.0)
    for _ in range(3):
        yield from L.wait(0.5)
    _log(ctx, "end")
