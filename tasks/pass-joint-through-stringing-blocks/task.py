"""Pass a line's rigid joint past a stringing block too narrow for it, keeping the line under tension.

Station table, a model of a stringing run. Three open-top grey blocks on 5 cm stands stand in a row along the table
at x 32 cm, each a U of two walls over a black roller pad. The right and left blocks (y -26 and +5 cm) have 3 cm
throats; the middle block (y -10 cm) has a 1.2 cm throat. A yellow 6 mm line (70 cm) runs out of an orange tensioner
tunnel on the right (a spring carriage, 0.25 N back-pull on its home stop) through all three throats. A grey rigid
joint (3.5 x 2 x 2 cm) is spliced into the line at y -18.5 cm, just right of the narrow block: it cannot pass the
1.2 cm throat. Past the left block the line's free end carries a black toggle bar across it and droops off the block.
A blue open-top pocket cleat stands beyond the left block, turned towards the robot, with a slot in its near wall:
a toggle dropped in, with the line in the slot, holds the line.

Act: the right arm lifts the joint so the line comes up out of the narrow throat and carries it over the narrow
block while the left arm draws the line in by its toggle; the right arm lowers the joint, which drops the line back
into the narrow throat, and holds it while the left arm drops the toggle into the cleat's pocket; then both let go.
Only the left arm reaches the cleat; only the right arm reaches the narrow block and the joint at its start.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
Test-only demo variants (env var STRING_CHEAT): none | pull | onearm | letgo | hold | tier05.
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

INSTRUCTION = ("Get the grey joint on the yellow line past the narrow middle block, to rest between it and the left block, and make the line fast "
               "with a toggle in the blue cleat, leaving the line running in all three blocks. Then let go.")

GREY = (0.62, 0.63, 0.65)
DARK = (0.10, 0.10, 0.11)
ORANGE = (0.90, 0.45, 0.08)
BLUE = (0.15, 0.35, 0.80)
YELLOW = (0.95, 0.80, 0.10)
JOINT_GREY = (0.45, 0.46, 0.48)

# -- layout (m; x forward from the robot, y to the robot's left; heights above the table) ---------------------------
X0 = 0.32  # the line's run along y
R = 0.003  # 6 mm line
Z_AX = 0.060  # line axis height in the throats
STAND_H = 0.050
PAD_TOP = Z_AX - R - 0.0005  # roller pad top
BL = 0.030  # block length along the line
WT = 0.012  # wall thickness
BLOCKS = {  # name: (centre y, throat width, wall top above the table)
    "blk_r": (-0.26, 0.030, Z_AX + 0.025),
    "blk_m": (-0.10, 0.012, Z_AX + 0.015),
    "blk_l": (0.05, 0.030, Z_AX + 0.025),
}
Y_M = BLOCKS["blk_m"][0]
Y_L = BLOCKS["blk_l"][0]
EXIT = np.array([X0, Y_L + BL / 2])  # where the line leaves the left block


def _block(name, y, throat, wall_top):
    w = throat + 2 * WT
    return [
        Box(f"{name}_stand", pos=(X0, y, TOP + STAND_H / 2), size=(w, BL, STAND_H), color=GREY),
        Box(f"{name}_roller", pos=(X0, y, TOP + (STAND_H + PAD_TOP) / 2), size=(throat, BL, PAD_TOP - STAND_H),
            color=DARK),
        Box(f"{name}_wall_n", pos=(X0 - (throat + WT) / 2, y, TOP + (STAND_H + wall_top) / 2),
            size=(WT, BL, wall_top - STAND_H), color=GREY),
        Box(f"{name}_wall_f", pos=(X0 + (throat + WT) / 2, y, TOP + (STAND_H + wall_top) / 2),
            size=(WT, BL, wall_top - STAND_H), color=GREY),
    ]


# tensioner: a tunnel the line's end slides in (spring carriage on a prismatic joint, see _install_spawn)
TUN_Y0, TUN_Y1 = -0.595, -0.345
TUN_IN = 0.014  # inner square
TUN_T = 0.008
_tl = TUN_Y1 - TUN_Y0
_tc = (TUN_Y0 + TUN_Y1) / 2
TENSIONER = [
    Box("tens_floor", pos=(X0, _tc, TOP + (Z_AX - TUN_IN / 2) / 2), size=(TUN_IN + 2 * TUN_T, _tl, Z_AX - TUN_IN / 2),
        color=ORANGE),
    Box("tens_roof", pos=(X0, _tc, TOP + Z_AX + TUN_IN / 2 + TUN_T / 2), size=(TUN_IN + 2 * TUN_T, _tl, TUN_T),
        color=ORANGE),
    Box("tens_side_n", pos=(X0 - TUN_IN / 2 - TUN_T / 2, _tc, TOP + Z_AX), size=(TUN_T, _tl, TUN_IN), color=ORANGE),
    Box("tens_side_f", pos=(X0 + TUN_IN / 2 + TUN_T / 2, _tc, TOP + Z_AX), size=(TUN_T, _tl, TUN_IN), color=ORANGE),
]

# the line
SEG = 0.02
LINE_Y0 = -0.575  # start tip, inside the tensioner
LINE = cable("line", start=(X0, LINE_Y0, TOP + Z_AX), yaw=math.pi / 2, length=0.70, radius=R, segment=SEG,
             cable_mass=0.05, bend_stiffness=0.004, bend_damping=0.0008, color=YELLOW)
J_SEG, T_IN_SEG = 19, 34  # segments carrying the joint and the toggle (the line's last segment)
J_LEN, J_W = 0.035, 0.020
TOG_L, TOG_T, TOG_H = 0.050, 0.010, 0.020  # toggle bar: across the line, thickness along it, height
ARC_JT = (T_IN_SEG - J_SEG) * SEG  # 0.30 m of line from the joint to the toggle


def _seg_y(i):
    return LINE_Y0 + (i + 0.5) * SEG


JOINT = Box("joint", pos=(X0, _seg_y(J_SEG), TOP + Z_AX), size=(J_LEN, J_W, J_W), yaw=math.pi / 2, mass=0.010,
            color=JOINT_GREY)
TOG_IN = Box("toggle", pos=(X0, _seg_y(T_IN_SEG), TOP + Z_AX), size=(TOG_T, TOG_L, TOG_H), yaw=math.pi / 2,
             mass=0.015, color=DARK)
WELDED = {JOINT.name: (J_SEG, (J_SEG - 1, J_SEG, J_SEG + 1)),
          TOG_IN.name: (T_IN_SEG, (T_IN_SEG - 1, T_IN_SEG))}

# the cleat: an open-top pocket the toggle drops into, with a slot in its near wall for the line (wide enough for the
# line to fall into, far too narrow for the 5 cm bar). It sits off the line's run, turned 20 deg towards the robot,
# placed so that the line from a joint between the narrow and left blocks just reaches into the pocket (measured in
# the sim: the line bends round the left block's exit, so a straight-line estimate is not used).
Y_F = -0.025  # where the joint is carried to and held while the toggle is laid
PHI = math.radians(20.0)
U_C = np.array([-math.sin(PHI), math.cos(PHI)])  # direction from the left block's exit to the cleat
V_C = np.array([U_C[1], -U_C[0]])  # across the cleat
S_WALL = 0.165  # m from the left block's exit along U_C to the inner face of the cleat's near (slotted) wall
CL_IN_L, CL_IN_W = 0.035, 0.088  # pocket inside: along the line, across
CL_WT, CL_SLOT = 0.006, 0.024
CL_RIM = Z_AX + 0.015
CL_FLOOR = Z_AX - TOG_H / 2 - 0.001
SLOT_FLOOR = Z_AX - R - 0.0005
FACE = EXIT + U_C * S_WALL  # inner face of the near wall (the toggle bears on it)
CL_YAW = math.atan2(U_C[1], U_C[0])
CL_MID = FACE + U_C * CL_IN_L / 2  # pocket centre


def _cbox(name, u0, u1, v0, v1, z0, z1):
    c = FACE + U_C * (u0 + u1) / 2 + V_C * (v0 + v1) / 2
    return Box(name, pos=(float(c[0]), float(c[1]), TOP + (z0 + z1) / 2), size=(u1 - u0, v1 - v0, z1 - z0),
               yaw=CL_YAW, color=BLUE)


def _cleat():
    hw, wt, L = CL_IN_W / 2, CL_WT, CL_IN_L
    return [
        _cbox("cleat_base", -wt, L + wt, -hw - wt, hw + wt, 0.0, CL_FLOOR),
        _cbox("cleat_near_a", -wt, 0.0, CL_SLOT / 2, hw + wt, CL_FLOOR, CL_RIM),
        _cbox("cleat_near_b", -wt, 0.0, -hw - wt, -CL_SLOT / 2, CL_FLOOR, CL_RIM),
        _cbox("cleat_slot_sill", -wt, 0.0, -CL_SLOT / 2, CL_SLOT / 2, CL_FLOOR, SLOT_FLOOR),
        _cbox("cleat_far", L, L + wt, -hw - wt, hw + wt, CL_FLOOR, CL_RIM),
        _cbox("cleat_side_a", 0.0, L, hw, hw + wt, CL_FLOOR, CL_RIM),
        _cbox("cleat_side_b", 0.0, L, -hw - wt, -hw, CL_FLOOR, CL_RIM),
    ]


OBJECTS = [*TENSIONER, *[b for k, (y, t, h) in BLOCKS.items() for b in _block(k, y, t, h)], *_cleat(),
           JOINT, TOG_IN, LINE]

REEL_K, REEL_TARGET, REEL_DAMP = 1.0, 0.25, 0.3  # 0.25 N on the home stop, ~0.40 N with 15 cm drawn out
REEL_PAYOUT = 0.20

TWIN_EXTRA = [
    {"name": "tensioner_spring", "kind": "joint", "real": "constant-force spring or 25 g weight over a pulley",
     "note": "sim: the line's end slides on a straight spring track inside the orange box; home stop at the start"},
    {"name": "joint_splice", "kind": "joint", "real": "joint crimped/glued onto the line", "note": "rigid weld"},
    {"name": "toggle_splices", "kind": "joint", "real": "toggle bars glued across the line", "note": "rigid welds"},
]


# -- spawn hook: welds for the joint and toggle, and the tensioner's spring carriage -----------------------------
def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_string_task", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != LINE.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        for name, (seg, near) in WELDED.items():
            body = f"{root}/{name}"
            j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/weld_{name}")
            j.CreateBody0Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_{seg}")])
            j.CreateBody1Rel().SetTargets([Sdf.Path(body)])
            j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateExcludeFromArticulationAttr(True)
            fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(body))
            for i in near:
                fp.GetFilteredPairsRel().AddTarget(Sdf.Path(f"{prim_path}/seg_{i}"))
        seg = f"{prim_path}/seg_0"
        world = UsdGeom.Xformable(stage.GetPrimAtPath(seg)).ComputeLocalToWorldTransform(0)
        q = world.ExtractRotationQuat()
        tip = world.Transform(Gf.Vec3d(-LINE.seg_len / 2, 0.0, 0.0))
        pj = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/reel")
        pj.CreateBody1Rel().SetTargets([Sdf.Path(seg)])
        pj.CreateAxisAttr("X")  # +x of seg_0 = along the line, out of the tensioner (payout)
        pj.CreateLocalPos0Attr(Gf.Vec3f(*tip))
        pj.CreateLocalRot0Attr(Gf.Quatf(q.GetReal(), *q.GetImaginary()))
        pj.CreateLocalPos1Attr(Gf.Vec3f(-LINE.seg_len / 2, 0.0, 0.0))
        pj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        pj.CreateLowerLimitAttr(0.0)  # home stop
        pj.CreateUpperLimitAttr(REEL_PAYOUT)
        pj.CreateExcludeFromArticulationAttr(True)
        drv = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), "linear")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(REEL_K)
        drv.CreateDampingAttr(REEL_DAMP)
        drv.CreateTargetPositionAttr(-REEL_TARGET)
        return prim

    spawn._string_task = True
    cablekit._spawn_cable = spawn


_install_spawn()

# -- env handle, per-episode memory -------------------------------------------------------------------------------
_ENV = {"env": None, "step": 0}
_FRESH = dict(last_step=-1, max_out=0.0, ran_back=False, butted=False, left_outer=False, nan=False, why="")
_MON = dict(_FRESH)


def _reset_episode() -> None:
    _MON.update(_FRESH)
    _ENV["step"] = 0


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline import)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_string_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            _ENV["step"] += 1
            return base(self, *a, **k)

        _task_step._string_hooked = True
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

RUNBACK = 0.05  # line drawn back towards the tensioner by more than this from the most it was drawn out: latched
RELEASE_DIST = 0.03  # at the end every gripper TCP at least this far from the line, joint and toggle
BUTT_GAP = 0.004  # joint end this close to a narrow-block end face, overlapping it, = butted against it


def grippers() -> list[np.ndarray]:
    """TCP position per arm in the env frame; [] offline."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- geometry -----------------------------------------------------------------------------------------------------
def _quat_axes(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _dense(poly, step=0.003):
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out.extend(a + (b - a) * (k / n) for k in range(1, n + 1))
    return np.asarray(out)


def line_pts(objects) -> np.ndarray:
    return _dense(np.asarray(objects[LINE.name].polyline(), dtype=float))


def drawn_out(objects) -> float:
    """How far the line's end has been drawn out of the tensioner since the start (m)."""
    return float(objects[LINE.name].start_tip[1] - LINE_Y0)


def _in_span(pts, name):
    y = BLOCKS[name][0]
    return pts[np.abs(pts[:, 1] - y) <= BL / 2]


def out_of_throat(pts, name) -> bool:
    """Some line point over the block's length is above its walls or outside its throat."""
    _, throat, wall_top = BLOCKS[name]
    p = _in_span(pts, name)
    if len(p) == 0:
        return True
    return bool(((p[:, 2] - TOP) > wall_top).any() or (np.abs(p[:, 0] - X0) > throat / 2 + 0.002).any())


def seated(pts, name) -> bool:
    """The line lies down in the throat on the roller over the block's whole length."""
    _, throat, _ = BLOCKS[name]
    p = _in_span(pts, name)
    return bool(len(p) > 0 and (np.abs(p[:, 0] - X0) <= throat / 2 + 0.001).all()
                and ((p[:, 2] - TOP) <= Z_AX + 0.006).all())


def joint_pose(objects):
    s = objects[JOINT.name]
    return np.asarray(s.pos, dtype=float), _quat_axes(s.quat_xyzw)


def butted(objects) -> bool:
    """The joint presses end-on against the narrow block (overlaps it across the line and below its wall top, and
    its near end is within 4 mm of a block end face)."""
    p, ax = joint_pose(objects)
    _, throat, wall_top = BLOCKS["blk_m"]
    half_y = abs(ax[1, 0]) * J_LEN / 2 + abs(ax[1, 1]) * J_W / 2 + abs(ax[1, 2]) * J_W / 2
    half_z = abs(ax[2, 0]) * J_LEN / 2 + abs(ax[2, 1]) * J_W / 2 + abs(ax[2, 2]) * J_W / 2
    if abs(p[0] - X0) > throat / 2 + WT + J_W / 2 or (p[2] - half_z - TOP) > wall_top:
        return False
    gap = abs(p[1] - Y_M) - BL / 2 - half_y
    return bool(gap <= BUTT_GAP)


def joint_past(objects) -> bool:
    """Joint wholly between the narrow block and the left block, at least 5 mm clear of the narrow block."""
    p, _ = joint_pose(objects)
    lo = Y_M + BL / 2 + J_LEN / 2 + 0.005
    hi = Y_L - BL / 2 - J_LEN / 2
    return bool(lo <= p[1] <= hi and abs(p[0] - X0) <= 0.02)


def toggle_in_cleat(objects, name) -> bool:
    """Toggle bar centre inside the cleat's pocket, below its rim, and the line down in the near wall's slot."""
    p = np.asarray(objects[name].pos, dtype=float)
    d = p[:2] - FACE
    u, v = float(d @ U_C), float(d @ V_C)
    if not (0.0 <= u <= CL_IN_L and abs(v) <= CL_IN_W / 2 and p[2] - TOP <= CL_RIM):
        return False
    pts = line_pts(objects)
    dd = pts[:, :2] - FACE
    uu, vv = dd @ U_C, dd @ V_C
    m = (uu >= -CL_WT) & (uu <= 0.0)
    return bool(m.any() and (np.abs(vv[m]) <= CL_SLOT / 2 + 0.001).all() and ((pts[m, 2] - TOP) <= CL_RIM).all())


def made_fast(objects) -> bool:
    return toggle_in_cleat(objects, TOG_IN.name)


def _parts(objects) -> np.ndarray:
    return np.vstack([line_pts(objects)] + [np.asarray(objects[n].pos, dtype=float)[None] for n in WELDED])


def released(objects) -> bool:
    pts = _parts(objects)
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def _finite(objects) -> bool:
    c = objects[LINE.name]
    return bool(np.isfinite(c.points).all() and all(np.isfinite(objects[n].pos).all() for n in WELDED))


_DBG_HIST: list = []


def _track(objects) -> None:
    """Once per control step: latch a run-back, a butted joint, the line leaving an outer block's throat, NaN."""
    step = _ENV["step"]
    if step == _MON["last_step"]:
        return
    _MON["last_step"] = step
    if not _finite(objects):
        if os.environ.get("STRING_DEBUG") and not _MON["nan"]:
            for row in _DBG_HIST:
                print("[nan-hist]", row, flush=True)
        _MON["nan"] = True
        return
    if os.environ.get("STRING_DEBUG"):
        t = objects[TOG_IN.name]
        lp = np.asarray(objects[LINE.name].points, dtype=float)[-4:]
        _DBG_HIST.append(f"s{step} tog={np.round(t.pos, 3).tolist()} q={np.round(t.quat_xyzw, 2).tolist()} "
                         f"line_end={np.round(lp - [0, 0, TOP], 3).tolist()} tcp={[np.round(g, 3).tolist() for g in grippers()]}")
        del _DBG_HIST[:-6]
    out = drawn_out(objects)
    _MON["max_out"] = max(_MON["max_out"], out)
    if out < _MON["max_out"] - RUNBACK and not _MON["ran_back"]:
        _MON["ran_back"] = True
        _MON["why"] += f" ranback@{step}"
    if butted(objects) and not _MON["butted"]:
        _MON["butted"] = True
        _MON["why"] += f" butted@{step}"
    pts = line_pts(objects)
    if (out_of_throat(pts, "blk_r") or out_of_throat(pts, "blk_l")) and not _MON["left_outer"]:
        _MON["left_outer"] = True
        _MON["why"] += f" outer@{step}"


def violated() -> bool:
    return bool(_MON["nan"] or _MON["ran_back"] or _MON["butted"] or _MON["left_outer"])


def score(objects) -> float:
    """0 after a latched violation (line ran back > 5 cm, joint butted against the narrow block, line out of an outer
    block's throat), if a gripper is within 3 cm of the line, joint or toggle at the end, if the toggle is not made fast
    in the cleat or the joint is not past the narrow block. Then 0.5; 1.0 if the line is also down in all three throats
    (the narrow one included)."""
    _track(objects)
    if violated() or not _finite(objects) or not released(objects):
        return 0.0
    if not (made_fast(objects) and joint_past(objects)):
        return 0.0
    pts = line_pts(objects)
    if all(seated(pts, k) for k in BLOCKS):
        return 1.0
    return 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects[LINE.name].resting(max_speed=0.02)


def distance(objects) -> float:
    _track(objects)
    if violated() or not _finite(objects):
        return 3.0
    p, _ = joint_pose(objects)
    return float(abs(p[1] - Y_F) + np.linalg.norm(np.asarray(objects[TOG_IN.name].pos)[:2] - FACE))


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    _track(objects)
    ok = _finite(objects)
    pts = line_pts(objects) if ok else None
    return {"drawn": round(float(np.clip(drawn_out(objects) / 0.135, 0, 1)), 2) if ok else 0.0,
            "joint_past": int(ok and joint_past(objects)), "made_fast": int(ok and made_fast(objects)),
            "mid_seated": int(ok and seated(pts, "blk_m")), "never_violated": int(not violated()),
            "released": int(ok and released(objects))}


VARIATION = {"cable:line": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}  # the line runs through fixed blocks and tensioner


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------
PINCH_J = -0.020  # finger opening on the 2 cm joint (pads ~40 mm apart)
OPEN_T = -0.047  # fingers wide open round the 5 cm toggle bar
OPEN_REL = -0.032  # release opening over the cleat (pads ~64 mm apart, the pocket is 88 mm wide)
GZ = 0.010  # TCP above a part's centre when pinching it
GZ_T = 0.004  # TCP above the toggle's centre (the bar is 20 mm tall)
CARRY_J = Z_AX + 0.035  # joint centre height while carried over the narrow block


def _log(ctx, tag):
    st = ctx.states()
    p, _ = joint_pose(st)
    ti = np.asarray(st[TOG_IN.name].pos)
    pts = line_pts(st)
    print(f"[demo t={ctx.t:5.1f}] {tag}: joint=({p[0]:.3f},{p[1]:.3f},{p[2] - TOP:.3f}) tog=({ti[0]:.3f},{ti[1]:.3f},"
          f"{ti[2] - TOP:.3f}) out={drawn_out(st):.3f} max={_MON['max_out']:.3f} past={joint_past(st)} "
          f"fast={made_fast(st)} seat=({seated(pts, 'blk_r')},{seated(pts, 'blk_m')},{seated(pts, 'blk_l')}) "
          f"rel={released(st)} viol={violated()}{_MON['why']} score={score(st)}", flush=True)
    if os.environ.get("STRING_DEBUG"):
        dd = pts[:, :2] - FACE
        uu, vv = dd @ U_C, dd @ V_C
        m = (uu >= -0.02) & (uu <= 0.01)
        print("[dbg] near-wall line u,v,z: " + " ".join(f"({a:.3f},{b:.3f},{c - TOP:.3f})" for a, b, c in
                                                     zip(uu[m][::3], vv[m][::3], pts[m, 2][::3])), flush=True)


def _span(st) -> float:
    """Path length of the line from the joint's segment to the toggle's segment (debug)."""
    p = np.asarray(st[LINE.name].points, dtype=float)[J_SEG:T_IN_SEG + 1]
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def _to_part(ctx, arm, name, target, yaw, duration, n=2, settle=0.0):
    """Closed loop: bring the part's centre to target (world xyz)."""
    for k in range(n):
        p = np.asarray(ctx.states()[name].pos, dtype=float)
        g = arm.tcp()
        d = np.asarray(target, dtype=float) - p
        if k > 0 and float(np.linalg.norm(p - g)) > 0.05:  # part no longer in the hand: do not chase it
            break
        if k > 0:  # a correction never pushes more than 1 cm past what the last move asked for (no fighting a stop)
            d = d * min(1.0, 0.01 / (float(np.linalg.norm(d)) + 1e-9))
        tgt = g + d
        yield from arm.move(tuple(tgt), yaw=yaw, duration=duration if k == 0 else 0.5)
        yield from arm.wait(settle)  # let the arm and the part come to rest before measuring again
        if os.environ.get("STRING_DEBUG"):
            st = ctx.states()
            q = np.asarray(st[name].pos, dtype=float)
            print(f"[dbg] {name} want={np.round(target, 3)} got={np.round(q, 3)} cmd={np.round(tgt, 3)} "
                  f"tcp={np.round(arm.tcp(), 3)} line_len_j2t={_span(st):.3f} real_tcp="
                  f"{[np.round(g, 3).tolist() for g in grippers()]}", flush=True)
            if name == TOG_IN.name:
                pp = np.asarray(st[LINE.name].points, dtype=float)[J_SEG:T_IN_SEG + 1]
                print("[dbg] line " + " ".join(f"({q[0]:.3f},{q[1]:.3f},{q[2] - TOP:.3f})" for q in pp), flush=True)


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("STRING_CHEAT", "")
    yield from L.wait(1.0)
    _log(ctx, "start")
    if cheat == "none":
        yield from L.wait(6.0)
        _log(ctx, "end")
        return

    yaw_r = math.pi / 2  # fingers across the joint; wrist camera on the +y side, towards the narrow block
    yaw_l0 = -math.pi / 2  # fingers along the toggle bar
    yaw_l1 = CL_YAW - math.pi  # fingers along the bar into the cleat (110 deg turned into the wrist range)

    def take_joint():
        p, _ = joint_pose(ctx.states())
        yield from Rt.move((p[0], p[1], p[2] + GZ + 0.05), yaw=yaw_r, finger=PINCH_J, duration=1.5)
        p, _ = joint_pose(ctx.states())
        yield from Rt.move((p[0], p[1], p[2] + GZ), yaw=yaw_r, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED, 0.6)

    tog = TOG_IN.name

    def take_toggle():
        t = np.asarray(ctx.states()[tog].pos, dtype=float)
        yield from L.move((t[0], t[1], t[2] + GZ_T + 0.05), yaw=yaw_l0, finger=OPEN_T, duration=1.5)
        t = np.asarray(ctx.states()[tog].pos, dtype=float)
        yield from L.move((t[0], t[1], t[2] + GZ_T), yaw=yaw_l0, duration=0.8)
        yield from L.grip(FINGER_CLOSED, 0.6)
        t = np.asarray(ctx.states()[tog].pos, dtype=float)
        print(f"[demo] toggle grip: tcp={np.round(L.tcp(), 3)} toggle={np.round(t, 3)}", flush=True)

    if cheat == "pull":  # left arm alone draws the line towards the cleat
        yield from take_toggle()
        tgt = CL_MID
        yield from L.move((tgt[0], tgt[1], TOP + Z_AX + GZ_T), yaw=yaw_l1, duration=3.0)
        _log(ctx, "pulled")
        yield from L.wait(2.0)
        _log(ctx, "end")
        return

    yield from ctx.both(take_joint(), take_toggle())
    _log(ctx, "both holding")

    # lift the joint so the line comes up out of the narrow throat
    g = Rt.tcp()
    yield from Rt.move((g[0], g[1], TOP + CARRY_J + GZ), yaw=yaw_r, duration=1.0)
    _log(ctx, "joint lifted")
    if cheat == "onearm":  # right arm alone: carry the joint over and set it down, nothing made fast
        yield from Rt.move((X0, Y_F, TOP + CARRY_J + GZ), yaw=yaw_r, duration=2.5)
        yield from Rt.move((X0, Y_F, TOP + Z_AX + GZ), yaw=yaw_r, duration=1.0)
        yield from Rt.grip(PINCH_J, 0.5)
        yield from Rt.move((0.20, -0.40, TOP + 0.16), yaw=0.0, duration=1.5)
        yield from L.wait(3.0)
        _log(ctx, "end")
        return

    # carry the joint over the narrow block while the left arm draws the line in, low, towards the cleat
    pre = FACE - U_C * 0.035  # toggle 3.5 cm short of the cleat's near wall, at line height
    # partial test (tier05): the joint is carried over and set down 1.5 cm off the run, so the line comes down on the
    # narrow block's wall top instead of into its throat
    xj = X0 - 0.015 if cheat == "tier05" else X0
    yield from ctx.both(Rt.move((xj, Y_F, TOP + CARRY_J + GZ), yaw=yaw_r, duration=3.0),
                        L.move((pre[0], pre[1], TOP + Z_AX + GZ_T), yaw=yaw_l1, duration=3.0))
    _log(ctx, "joint over")

    # lower the joint (still held) so the line drops back into the narrow throat
    yield from _to_part(ctx, Rt, JOINT.name, (xj, Y_F, TOP + Z_AX + 0.002), yaw_r, 1.0, n=2)
    _log(ctx, "joint lowered")

    def leave_left():
        yield from L.grip(OPEN_REL, 0.5)  # just clear of the bar's ends, inside the pocket's side walls
        yield from L.wait(0.6)
        _log(ctx, "left open")
        g = L.tcp()
        yield from L.move((g[0], g[1], TOP + 0.13), yaw=L._yaw, duration=0.8)
        yield from L.move((0.20, 0.42, TOP + 0.16), yaw=0.0, duration=1.2)

    def leave_right():
        yield from Rt.grip(PINCH_J, 0.6)
        g = Rt.tcp()
        yield from Rt.move((g[0], g[1], TOP + 0.13), yaw=yaw_r, duration=0.8)
        yield from Rt.move((0.20, -0.42, TOP + 0.16), yaw=0.0, duration=1.2)

    if cheat == "letgo":  # the toggle is let go short of the cleat, then the joint
        yield from leave_left()
        yield from leave_right()
        yield from L.wait(3.0)
        _log(ctx, "end")
        return


    # drop the toggle into the cleat's pocket: over the rim, above the pocket centre, down to just over the rim, open
    hi = TOP + CL_RIM + TOG_H / 2 + 0.003
    yield from _to_part(ctx, L, tog, (pre[0], pre[1], hi + 0.005), yaw_l1, 0.8, n=2, settle=0.4)
    drop = FACE + U_C * 0.015 - V_C * 0.021  # inside the pocket; the bar shifts about 2 cm along V_C as the fingers open
    yield from _to_part(ctx, L, tog, (drop[0], drop[1], hi + 0.005), yaw_l1, 1.2, n=3, settle=0.4)
    yield from _to_part(ctx, L, tog, (drop[0], drop[1], hi), yaw_l1, 0.8, n=2, settle=0.4)
    _log(ctx, "toggle over pocket")
    yield from leave_left()
    _log(ctx, "toggle laid")
    if cheat == "hold":
        yield from Rt.wait(4.0)
        _log(ctx, "end")
        return

    # let go of the joint: the line's pull seats the toggle against the cleat
    yield from leave_right()
    for _ in range(6):
        yield from L.wait(0.5)
    _log(ctx, "end")
