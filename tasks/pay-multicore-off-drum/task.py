"""On the station table a cable drum (grey hub, 5 cm across, 7 cm tall, with a white bar on top that shows it
turning) stands on a vertical axle at the left, 29 cm in front of the robot. A red 6 mm multicore, 64 cm long, is
wound 2.5 turns round the hub (inner end fixed to the hub); 15 cm of it plus its 35 mm connector lies out on the
table, pointing toward the right along a yellow-taped cable run (two tape lines 8 cm apart). At the far end of the
run, in front of the right arm, is a white cradle the connector must rest in. The drum turns freely with light drag,
so a fast pull spins it and it keeps turning for a moment after the pull stops. Only the left arm reaches the
connector at the start and only the right arm reaches the cradle, so the cable has to change hands once, near the
middle of the table, while the drum must never turn with the cable unheld.

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
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Pay the red cable off the drum along the yellow-taped run and rest its connector in the white "
               "cradle, then let go.")

# -- layout (m; x forward from the robot, y to the robot's left) ------------------------------------------------------
GREY, WHITE, YELLOW, RED = (0.45, 0.47, 0.50), (0.93, 0.93, 0.90), (0.95, 0.80, 0.10), (0.78, 0.08, 0.06)
HUB_XY = (0.29, 0.31)
HUB_R, HUB_H = 0.025, 0.07
HUB_MASS = float(os.environ.get("DRUM_MASS", "0.4"))
DRUM_DAMP = float(os.environ.get("DRUM_DAMP", "2e-5"))  # axle drag, N m s per degree
DRUM = Cylinder("drum", pos=(*HUB_XY, TOP + HUB_H / 2 + 0.001), radius=HUB_R, height=HUB_H, color=GREY,
                mass=HUB_MASS)
FL_R, FL_H = 0.06, 0.006  # top flange: gives the drum the inertia of a real cable drum (it coasts when let go)
FLANGE_MASS = float(os.environ.get("DRUM_FLANGE_MASS", "0.3"))
FLANGE = Cylinder("drum_flange", pos=(*HUB_XY, TOP + HUB_H + 0.001 + 0.0005 + FL_H / 2), radius=FL_R, height=FL_H,
                  color=GREY, mass=FLANGE_MASS)
MARK_SIZE = (0.11, 0.012, 0.003)
DRUM_MARK = Box("drum_mark", pos=(HUB_XY[0], HUB_XY[1], FLANGE.pos[2] + FL_H / 2 + MARK_SIZE[2] / 2 + 0.0005),
                size=MARK_SIZE, color=WHITE, mass=0.004)

R = 0.003  # 6 mm multicore
TURNS = 2.5
CABLE_LEN = 0.64
CONN = Plug(size=(0.035, 0.016, 0.012), mass=0.015, color=(0.12, 0.12, 0.12))
HEAD = -1.4  # heading of the cable run (rad): mostly toward the right arm, slightly away from the robot
U = np.array([math.cos(HEAD), math.sin(HEAD)])
NV = np.array([-U[1], U[0]])  # left-hand normal of the run
THETA_T = math.atan2(U[0], -U[1])  # hub angle where a clockwise-wound cable leaves along U
_Z = TOP + R + 0.0005
_proto = cable("multicore", start=(0.20, 0.20, _Z), yaw=0.0, length=CABLE_LEN, radius=R)
WIND = dict(start_angle=THETA_T + math.pi, sense=-1)  # clockwise from above; ends at THETA_T, tail along U
WPATH = cableshape.wind_path(_proto, (*HUB_XY, HUB_R), TURNS, **WIND)
SP = cableshape.spawn_for(_proto, WPATH)
CABLE = cable("multicore", start=SP["start"], yaw=SP["yaw"], length=CABLE_LEN, radius=R, end_plug=CONN,
              color=RED, bend_stiffness=0.003, bend_damping=0.0005)
WIND_R = float(np.hypot(*(WPATH[0, :2] - np.array(HUB_XY))))  # centreline radius of the wound turns
T0 = np.array(HUB_XY) + WIND_R * np.array([math.cos(THETA_T), math.sin(THETA_T)])  # where the run leaves the hub


def run_pt(s: float, off: float = 0.0) -> np.ndarray:
    """Point on the run centreline s m from the hub's leaving point, off m to its left."""
    return T0 + s * U + off * NV


# yellow tape: two 10 mm lines, inner edges 4 cm either side of the run centreline
TAPE_S0, TAPE_S1 = 0.07, 0.52
CORR_HALF = 0.04
TAPE = [Box(f"tape_{k}", pos=(*run_pt((TAPE_S0 + TAPE_S1) / 2, sg * (CORR_HALF + 0.005)), TOP + 0.0005),
            size=(TAPE_S1 - TAPE_S0, 0.010, 0.001), yaw=HEAD, color=YELLOW) for k, sg in (("l", 1), ("r", -1))]

# cradle: open-top seat for the connector at the far end of the run (axis along the run). Low corner posts leave the
# middle of each side open for the fingers; the near wall has a 10 mm slot for the cable (the 16 mm connector can't
# pass it, so it must be lowered in from above).
CR_S = 0.555
CR_C = run_pt(CR_S)
IN_HALF_U, IN_HALF_V, WALL_H, WT = 0.024, 0.011, 0.012, 0.006


def _cr(name, u, v, su, sv):
    p = CR_C + u * U + v * NV
    return Box(name, pos=(p[0], p[1], TOP + WALL_H / 2), size=(su, sv, WALL_H), yaw=HEAD, color=WHITE)


CRADLE = [
    _cr("cradle_end", IN_HALF_U + WT / 2, 0.0, WT, 2 * (IN_HALF_V + WT)),
    _cr("cradle_slot_l", -IN_HALF_U - WT / 2, (0.005 + IN_HALF_V + WT) / 2, WT, IN_HALF_V + WT - 0.005),
    _cr("cradle_slot_r", -IN_HALF_U - WT / 2, -(0.005 + IN_HALF_V + WT) / 2, WT, IN_HALF_V + WT - 0.005),
    *[_cr(f"cradle_post_{a}{b}", su * (IN_HALF_U - 0.0045), sv * (IN_HALF_V + WT / 2), 0.009, WT)
      for a, su in (("f", 1), ("n", -1)) for b, sv in (("l", 1), ("r", -1))],
]

OBJECTS = [DRUM, FLANGE, DRUM_MARK, *TAPE, *CRADLE, CABLE]

TWIN_EXTRA = [
    {"name": "drum_axle", "kind": "joint", "real": "608ZZ bearing on an M8 bolt in a printed base",
     "note": "drum turns about the vertical with light drag (~1e-3 N m s/rad); the hub weighs 0.4 kg"},
    {"name": "hub_anchor", "kind": "joint", "real": "inner cable end clamped in a slot on the hub top",
     "note": "the cable's inner end is fixed to the hub"},
    {"name": "drum_flange", "kind": "body", "size_mm": [120, 120, 6], "colour": "grey", "pose": [0.29, 0.31, 0.075],
     "note": "steel or printed disc fixed on the hub top, 0.3 kg (drum inertia)"},
    {"name": "drum_mark", "kind": "body", "size_mm": [110, 12, 3], "colour": "white", "pose": [0.29, 0.31, 0.08],
     "note": "bar glued across the flange, shows the drum turning"},
]


# -- the drum: revolute z axle world->hub, the cable's inner end and the mark bar fixed to the hub ------------------
def _install_drum() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_payout_drum", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CABLE.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        hub = f"{root}/{DRUM.name}"
        w = UsdGeom.Xformable(stage.GetPrimAtPath(root)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        h = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/drum_axle")
        h.CreateBody1Rel().SetTargets([Sdf.Path(hub)])
        h.CreateAxisAttr("Z")
        h.CreateLocalPos0Attr(Gf.Vec3f(float(w[0] + DRUM.pos[0]), float(w[1] + DRUM.pos[1]),
                                       float(w[2] + DRUM.pos[2])))
        h.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        h.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        h.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        h.CreateExcludeFromArticulationAttr(True)
        drv = UsdPhysics.DriveAPI.Apply(h.GetPrim(), "angular")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(0.0)
        drv.CreateDampingAttr(DRUM_DAMP)
        j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/hub_weld")
        j.CreateBody0Rel().SetTargets([Sdf.Path(hub)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        off = np.asarray(SP["seg0_pos"]) - np.asarray(DRUM.pos)
        x, y, z, qw = (float(v) for v in SP["seg0_quat_xyzw"])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, off)))
        j.CreateLocalRot0Attr(Gf.Quatf(qw, x, y, z))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
        for part in (FLANGE, DRUM_MARK):
            m = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/{part.name}_weld")
            m.CreateBody0Rel().SetTargets([Sdf.Path(hub)])
            m.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{part.name}")])
            m.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, float(part.pos[2] - DRUM.pos[2])))
            m.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
            m.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            m.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            m.CreateExcludeFromArticulationAttr(True)
        return prim

    spawn._payout_drum = True
    cablekit._spawn_cable = spawn


_install_drum()

# -- pre-wound start (coil set while on the drum) ---------------------------------------------------------------------
_SH: dict = {}


def _shape(emb):
    _SH["sh"] = cableshape.prewind(emb, CABLE, (*HUB_XY, HUB_R), TURNS, hold="set", settle_s=0, **WIND)
    _SH["relaxed"] = None
    cableshape.settle(emb, 2.0)


cableshape.on_reset(_shape)

OFF_DRUM = WIND_R + 0.015  # a bend joint whose two segments both lie this far from the axle has left the drum


def _relax_off_drum(env) -> None:
    """Coil set only while on the drum: once both segments at a bend joint have left the drum, that joint's rest
    angle goes to straight (latched), so the paid-out cable lies straight on the table like a real multicore."""
    sh = _SH.get("sh")
    if sh is None or os.environ.get("DRUM_RELAX", "1") == "0":
        return
    import torch
    from cablekit import _cable_state

    pts = _cable_state(env, CABLE).points
    if not np.isfinite(pts).all():
        return
    d = np.hypot(pts[:, 0] - HUB_XY[0], pts[:, 1] - HUB_XY[1])
    off = (d[:-1] > OFF_DRUM) & (d[1:] > OFF_DRUM)  # joint i+1 between seg i and seg i+1
    rel = _SH["relaxed"]
    rel = off.copy() if rel is None else (rel | off)
    if _SH["relaxed"] is not None and np.array_equal(rel, _SH["relaxed"]):
        return
    _SH["relaxed"] = rel
    base = env.unwrapped
    art = base.scene[CABLE.name]
    i1, i2 = cableshape._dof_index(art)
    q = torch.zeros_like(cableshape._t(art.data.joint_pos))
    keep = (~rel).astype(np.float64)[:, None] * sh.q
    q[0, i1] = torch.tensor(keep[:, 0], dtype=q.dtype, device=base.device)
    q[0, i2] = torch.tensor(keep[:, 1], dtype=q.dtype, device=base.device)
    art.set_joint_position_target_index(target=q)


# -- env handle (gripper TCPs and finger openings), per-episode memory --------------------------------------------
_ENV = {"env": None, "step": 0}
_MON = {"violated": "", "last_step": -1, "max_rate": 0.0, "max_loose": 0.0, "both_held": False, "handover": False}


def _reset_episode() -> None:
    _MON.update(violated="", last_step=-1, max_rate=0.0, max_loose=0.0, both_held=False, handover=False)
    _ENV["step"] = 0


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline import)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_drum_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            _ENV["step"] += 1
            _relax_off_drum(self._env)
            return base(self, *a, **k)

        _task_step._drum_hooked = True
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
HOLD_DIST = 0.030  # a closed gripper TCP within 3 cm of the cable or connector = holding the cable
RELEASE_DIST = 0.030  # at the end every TCP at least 3 cm from the cable and the connector
MAX_RATE = 2 * math.pi  # rad/s: one drum turn per second
FREE_RATE = math.radians(20.0)  # rad/s: the drum counts as turning above 20 deg/s
STILL_RATE = math.radians(3.0)  # rad/s: the drum counts as still below 3 deg/s
LOOSE_MAX = 0.02  # m: a wound turn may not stand further than this off its wound radius


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


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _cable_dist(objects, g) -> float:
    c = objects[CABLE.name]
    poly = c.polyline()
    d = min(_seg_dist(g, u, v) for u, v in zip(poly[:-1], poly[1:]))
    if c.end_plug is not None:
        p, a = np.asarray(c.end_plug), np.asarray(c.end_dir)
        d = min(d, _seg_dist(g, p - a * CONN.size[0] / 2, p + a * CONN.size[0] / 2))
    return d


def holders(objects) -> list[bool]:
    """Per arm: fingers closed and TCP within 3 cm of the cable or its connector."""
    return [f < CLOSED_MAX and _cable_dist(objects, g) <= HOLD_DIST for g, f in grippers()]


def released(objects) -> bool:
    return all(_cable_dist(objects, g) >= RELEASE_DIST for g, _ in grippers())


def drum_rate(objects) -> float:
    return float(abs(objects[DRUM.name].ang_vel[2]))


def loose_turn(objects) -> float:
    """How far (m) the cable stands off the wound radius anywhere before its last point on the hub: a loop thrown off
    by an overrunning drum, with cable further along still wound on."""
    pts = objects[CABLE.name].points
    d = np.hypot(pts[:, 0] - HUB_XY[0], pts[:, 1] - HUB_XY[1])
    on = np.where(d < WIND_R + 0.01)[0]
    return float(d[: on.max() + 1].max() - WIND_R) if len(on) else 0.0


def _finite(objects) -> bool:
    c = objects[CABLE.name]
    return bool(np.isfinite(c.points).all() and c.end_plug is not None and np.isfinite(c.end_plug).all())


def _local(p) -> tuple[float, float]:
    d = np.asarray(p[:2]) - CR_C
    return float(d @ U), float(d @ NV)


def in_cradle(objects) -> bool:
    """Connector centre between the cradle's walls (inside its 48 x 22 mm seat), lying on the table."""
    c = objects[CABLE.name]
    p = np.asarray(c.end_plug)
    u, v = _local(p)
    return bool(abs(u) <= IN_HALF_U - CONN.size[0] / 2 + 0.004 and abs(v) <= IN_HALF_V - CONN.size[1] / 2 + 0.003
                and p[2] <= TOP + CONN.size[2] / 2 + 0.006)


def in_corridor(objects) -> bool:
    """Every cable point more than 6 cm from the drum axle lies within 4 cm of the run centreline (between the tapes)
    and before the cradle's far wall."""
    pts = objects[CABLE.name].points
    d = np.hypot(pts[:, 0] - HUB_XY[0], pts[:, 1] - HUB_XY[1])
    q = pts[d > 0.06, :2] - T0
    return bool(len(q) == 0 or (np.all(np.abs(q @ NV) <= CORR_HALF) and np.all(q @ U <= CR_S + IN_HALF_U)))


def _track(objects) -> None:
    """Once per control step: latch overspeed (> 1 turn/s) and a free-turning drum (> 20 deg/s with no closed
    gripper within 3 cm of the cable)."""
    if not _finite(objects):
        _MON["violated"] = _MON["violated"] or "nan"
        return
    step = _ENV["step"]
    if step == _MON["last_step"] or _ENV["env"] is None:
        return
    _MON["last_step"] = step
    w = drum_rate(objects)
    _MON["max_rate"] = max(_MON["max_rate"], w)
    if os.environ.get("DRUM_DEBUG") and step % 5 == 0:
        c = objects[CABLE.name]
        print(f"[dbg] step {step} w={math.degrees(objects[DRUM.name].ang_vel[2]):.0f} plug={np.round(c.end_plug, 3).tolist()} "
              f"relaxed={None if _SH.get('relaxed') is None else int(_SH['relaxed'].sum())}", flush=True)
    h = holders(objects)
    if len(h) == 2 and all(h):
        _MON["both_held"] = True
    lo = loose_turn(objects)
    _MON["max_loose"] = max(_MON["max_loose"], lo)
    if lo > LOOSE_MAX and not _MON["violated"]:
        _MON["violated"] = f"loose turn {lo * 1000:.0f} mm at step {step}"
    if w > MAX_RATE and not _MON["violated"]:
        _MON["violated"] = f"overspeed {math.degrees(w):.0f} deg/s at step {step}"
    elif w > FREE_RATE and not any(h) and not _MON["violated"]:
        _MON["violated"] = f"unheld while turning {math.degrees(w):.0f} deg/s at step {step}"


def score(objects) -> float:
    """0 after any overspeed, free-turning or loose-turn moment, if a gripper is within 3 cm of the cable at the end,
    or if the connector is not in the cradle. 0.5 connector in the cradle but the cable off the taped run (or the drum
    still turning); 1.0 connector in the cradle, paid-out cable inside the taped run, drum still."""
    _track(objects)
    if _MON["violated"] or not _finite(objects):
        return 0.0
    if not released(objects) or not in_cradle(objects):
        return 0.0
    return 1.0 if in_corridor(objects) and drum_rate(objects) < STILL_RATE else 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects[CABLE.name].resting(max_speed=0.02)


def distance(objects) -> float:
    _track(objects)
    if _MON["violated"] or not _finite(objects):
        return 3.0
    return float(np.linalg.norm(np.asarray(objects[CABLE.name].end_plug)[:2] - CR_C))


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    _track(objects)
    ok = _finite(objects)
    plug = np.asarray(objects[CABLE.name].end_plug) if ok else np.zeros(3)
    start = run_pt(0.165)
    paid = float(np.clip((plug[:2] - start) @ U / (CR_S - 0.165), 0, 1)) if ok else 0.0
    return {"paid_out": round(paid, 2), "past_midline": int(ok and plug[1] < -0.02),
            "in_cradle": int(ok and in_cradle(objects)), "in_corridor": int(ok and in_corridor(objects)),
            "rate_kept": int(not _MON["violated"].startswith("overspeed")),
            "never_free": int(not _MON["violated"].startswith("unheld")),
            "coil_tight": int(not _MON["violated"].startswith("loose")),
            "released": int(ok and released(objects))}


VARIATION = {"obj:drum": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}, "cable:multicore": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------
HAND_S = 0.335  # connector centre at the hand-over, m along the run
YAW = HEAD  # fingers close across the run; wrist cameras look along the run, toward the cradle
PINCH = -0.030


def _log(ctx, tag):
    st = ctx.states()
    c = st[CABLE.name]
    p = np.asarray(c.end_plug)
    u, v = _local(p)
    print(f"[demo] {tag}: plug=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) cr_uv=({u:.3f},{v:.3f}) rate={math.degrees(drum_rate(st)):.0f} "
          f"max={math.degrees(_MON['max_rate']):.0f} loose={_MON['max_loose'] * 1000:.0f}mm hold={holders(st)} viol='{_MON['violated']}' "
          f"cradle={in_cradle(st)} corr={in_corridor(st)} score={score(st)}", flush=True)


def _plug_grip(ctx) -> tuple[np.ndarray, float]:
    c = ctx.states()[CABLE.name]
    p, a = np.asarray(c.end_plug), np.asarray(c.end_dir)
    yaw = (math.atan2(a[1], a[0]) + math.pi / 2) % math.pi - math.pi / 2
    if abs(yaw - YAW) > math.pi / 2:
        yaw += math.pi if yaw < 0 else -math.pi
    return p, yaw


def _wait_still(ctx, arm, limit=3.0):
    t = 0.0
    while t < limit and drum_rate(ctx.states()) > 0.05:
        yield from arm.wait(0.2)
        t += 0.2


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("DRUM_CHEAT", "")
    gz = TOP + CONN.size[2] / 2 + 0.010
    lift = gz + 0.006
    carry = gz + 0.018  # connector bottom ~15 mm up: clears the 12 mm cradle walls
    hi = TOP + 0.08
    _log(ctx, "start")
    if cheat == "nothing":
        yield from L.wait(4.0)
        _log(ctx, "end")
        return

    # 1) left arm takes the connector (fingers across it, wrist camera toward the run)
    p, yaw = _plug_grip(ctx)
    yield from L.move((p[0], p[1], hi), yaw=yaw, finger=PINCH, duration=1.6)
    p, yaw = _plug_grip(ctx)
    yield from L.move((p[0], p[1], gz), yaw=yaw, duration=1.0)
    yield from L.grip(FINGER_CLOSED, 0.6)
    yield from L.move((p[0], p[1], lift), yaw=yaw, duration=0.4)
    _log(ctx, "left has connector")

    # 2) left pays the cable out along the run to the hand-over point near the middle of the table
    h = run_pt(HAND_S)
    if cheat == "yank":
        yield from L.move((h[0], h[1], lift), yaw=YAW, duration=0.5)
    elif cheat == "overrun":  # stop dead at peak pull speed, still holding: the drum runs on and throws cable off
        pull = L.move((h[0], h[1], lift), yaw=YAW, duration=1.6)
        for _ in range(16):
            next(pull)
            yield
        pull.close()
        g = L.tcp()
        yield from L.move(g, yaw=YAW, duration=0.05)
        for k in range(4):
            yield from L.wait(0.25)
            _log(ctx, f"overrun {k}")
        yield from L.move((h[0], h[1], lift), yaw=YAW, duration=3.0)
    elif cheat == "letgo":  # open the fingers at peak pull speed (under one turn per second), then finish normally
        pull = L.move((h[0], h[1], lift), yaw=YAW, duration=2.0)
        for _ in range(20):
            next(pull)
            yield
        pull.close()
        _log(ctx, "letgo opens")
        g = L.tcp()
        yield from L.move(g, yaw=YAW, finger=FINGER_OPEN, duration=0.2)
        for k in range(6):
            yield from L.wait(0.25)
            _log(ctx, f"letgo coast {k}")
        yield from L.move((g[0], g[1], hi), yaw=YAW, duration=0.6)
        p, yaw = _plug_grip(ctx)
        yield from L.move((p[0], p[1], hi), yaw=yaw, duration=0.8)
        yield from L.move((p[0], p[1], gz), yaw=yaw, duration=0.8)
        yield from L.grip(FINGER_CLOSED, 0.6)
        yield from L.move((p[0], p[1], lift), yaw=yaw, duration=0.4)
        yield from L.move((h[0], h[1], lift), yaw=YAW, duration=3.0)
    else:
        yield from L.move((h[0], h[1], lift), yaw=YAW, duration=3.5)
    yield from L.move((h[0], h[1], gz), yaw=YAW, duration=0.4)
    yield from _wait_still(ctx, L)
    _log(ctx, "left at hand-over")
    if cheat == "onearm":  # left arm alone: push the connector as far toward the cradle as it reaches, let go
        far = run_pt(HAND_S + 0.10)
        yield from L.move((far[0], far[1], gz), yaw=YAW, duration=2.0)
        yield from _wait_still(ctx, L)
        yield from L.grip(FINGER_OPEN, 0.4)
        yield from L.move((far[0], far[1], hi), yaw=YAW, duration=0.8)
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    yield from L.grip(FINGER_OPEN, 0.4)
    g = L.tcp()
    yield from L.move((g[0], g[1], hi), yaw=YAW, duration=0.6)
    yield from L.move((0.25, 0.20, hi), yaw=0.0, duration=1.2)

    # 3) right arm takes the connector where it lies and pays out the rest of the run to the cradle
    p, yaw = _plug_grip(ctx)
    yield from Rt.move((p[0], p[1], hi), yaw=yaw, finger=PINCH, duration=1.6)
    p, yaw = _plug_grip(ctx)
    yield from Rt.move((p[0], p[1], gz), yaw=yaw, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED, 0.6)
    yield from Rt.move((p[0], p[1], carry), yaw=yaw, duration=0.5)
    _log(ctx, "right has connector")
    yield from Rt.move((CR_C[0], CR_C[1], carry), yaw=YAW, duration=3.5)
    yield from Rt.move((CR_C[0], CR_C[1], gz + 0.001), yaw=YAW, duration=0.8)
    yield from _wait_still(ctx, Rt)
    _log(ctx, "right at cradle")
    if cheat == "hold":
        yield from Rt.wait(3.0)
        _log(ctx, "end")
        return
    if cheat == "tier05":  # right arm keeps the connector seated while the left arm drags the middle of the run out over the tape: 0.5
        c = ctx.states()[CABLE.name]
        k = int(np.argmin(np.abs(c.points[:, 1] - 0.10)))
        q = c.points[k]
        off = q[:2] - 0.09 * NV
        yield from L.move((q[0], q[1], hi), yaw=YAW, finger=-0.014, duration=1.2)
        yield from L.move((q[0], q[1], q[2] + 0.010), yaw=YAW, duration=0.8)
        yield from L.grip(FINGER_CLOSED, 0.6)
        yield from L.move((off[0], off[1], q[2] + 0.014), yaw=YAW, duration=3.0)
        yield from _wait_still(ctx, L)
        yield from L.grip(-0.014, 0.5)
        yield from L.move((off[0], off[1], hi), yaw=YAW, duration=0.8)
        yield from L.move((0.25, 0.20, hi), yaw=0.0, duration=1.0)
    yield from Rt.grip(FINGER_OPEN, 0.5)
    yield from Rt.move((CR_C[0], CR_C[1], hi), yaw=YAW, duration=0.8)
    yield from Rt.move((0.25, -0.20, hi), yaw=0.0, duration=1.0)
    yield from Rt.wait(1.5)
    _log(ctx, "end")
