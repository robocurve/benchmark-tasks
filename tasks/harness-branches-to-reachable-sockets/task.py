"""Wire-harness assembly on the station table: a black round breakout clamp (7 cm diameter, 3 cm tall, fixed) sits in
front of the robot, a little left of centre, with the harness trunk leaving it towards the back-left. Three identical
grey 7 mm branches leave the clamp through grommets (each pinned at its exit, free to swing sideways), of clearly
different lengths, 20 / 33 / 46 cm, each ending in an identical dark-grey plug body (50 x 16 x 14 mm). Three identical
black socket modules (7 x 8 x 3.5 cm, fixed), each with one socket cavity facing the clamp, stand 21 cm (near: ahead,
left arm), 34 cm (middle: ahead-right) and 47 cm (far: to the right) from the clamp centre. Only one assignment
connects all three: short -> near, middle -> middle, long -> far; a shorter branch cannot reach a farther module.

Derangement: the branches spawn straight, fanned to the right: the short branch points straight at the middle
socket but its plug stops ~6 cm short of the opening; the middle branch's plug lies nearest the far socket (out of
its reach); the long branch's plug lies nearest the middle socket. Plugging each plug into the socket nearest where
it lies (as far as it reaches) puts the long plug in the middle socket, which scores 0.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (env var HARNESS_DEMO): correct (default) | nothing | nearest | greedy.
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
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Connect the wiring harness: plug each of the three grey branches into one of the three module "
               "sockets so that all three are plugged in, then let go.")

BRANCH_GREY = (0.55, 0.56, 0.58)
PLUG_GREY = (0.20, 0.20, 0.21)
MODULE = (0.12, 0.13, 0.15)
CLAMP_COL = (0.08, 0.08, 0.08)


def _dir(deg: float) -> np.ndarray:
    r = math.radians(deg)
    return np.array([math.cos(r), math.sin(r)])


# -- layout (x forward, y left; angles in degrees about z, 0 = +x) ---------------------------------------------------
CLAMP_XY = np.array([0.175, 0.15])
CLAMP_R, CLAMP_H = 0.035, 0.030  # round breakout clamp, 7 cm diameter
EXIT_GAP = 0.006  # branch exit point this far outside the clamp's surface (grommet)
R = 0.0035  # 7 mm branch
PL, PW, PH = 0.050, 0.016, 0.014  # plug body: length along the branch, width, height
PLUG_Z = TOP + PH / 2 + 0.0005
BRANCHES = ("short", "mid", "long")
LENGTH = {"short": 0.20, "mid": 0.33, "long": 0.46}
BRANCH_YAW = {"short": -43.0, "mid": -83.0, "long": -57.0}  # spawn heading of each branch
SOCKETS = ("near", "mid", "far")
FACE_R = {"near": 0.21, "mid": 0.34, "far": 0.47}  # clamp centre -> socket opening (m)
SOCKET_YAW = {"near": -5.0, "mid": -45.0, "far": -95.0}  # direction clamp -> socket; the socket opens towards the clamp
MATCH = {"short": "near", "mid": "mid", "long": "far"}

# socket module: 7 cm deep (along its axis) x 8 cm wide x 3.5 cm tall; cavity 20 mm wide x 17 mm tall x 28 mm deep
MOD_D, MOD_W, MOD_H = 0.07, 0.08, 0.035
CAV_W, CAV_H, CAV_D = PW + 0.004, PH + 0.003, 0.028


def exit_point(b: str) -> np.ndarray:
    return CLAMP_XY + _dir(BRANCH_YAW[b]) * (CLAMP_R + EXIT_GAP)


def face(k: str) -> np.ndarray:
    return CLAMP_XY + _dir(SOCKET_YAW[k]) * FACE_R[k]


def _branch(b: str):
    s = exit_point(b)
    return cable(f"branch_{b}", start=(float(s[0]), float(s[1]), PLUG_Z), yaw=math.radians(BRANCH_YAW[b]),
                 length=LENGTH[b], radius=R, anchor_start=True, cable_mass=0.08 * LENGTH[b],
                 end_plug=Plug(size=(PL, PW, PH), mass=0.015, color=PLUG_GREY), color=BRANCH_GREY,
                 bend_stiffness=0.01, bend_damping=0.001)


BRANCH_OBJS = {b: _branch(b) for b in BRANCHES}
_BY_NAME = {o.name: b for b, o in BRANCH_OBJS.items()}


def _install_exit_pivots() -> None:
    """Replace the kit's rigid world weld of each branch start by a pin about the vertical at its exit point: the
    branch leaves the clamp through a round grommet, held at the exit but free to swing sideways (a rigid weld would
    fix the first 2 cm segment at its spawn heading)."""
    orig = cablekit._spawn_cable
    if getattr(orig, "_harness_pivot", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        b = _BY_NAME.get(getattr(cfg, "cable_name", ""))
        if b is None:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdPhysics

        obj = BRANCH_OBJS[b]
        yaw = math.radians(BRANCH_YAW[b])
        stage = get_current_stage()
        stage.RemovePrim(f"{prim_path}/anchor")
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/exit_pivot")
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, obj.start)))
        j.CreateLocalRot0Attr(Gf.Quatf(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))
        j.CreateLocalPos1Attr(Gf.Vec3f(-obj.seg_len / 2, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateAxisAttr("Z")
        j.CreateExcludeFromArticulationAttr(True)
        return prim

    spawn._harness_pivot = True
    cablekit._spawn_cable = spawn


_install_exit_pivots()


def _module(k: str) -> list[Box]:
    """Fixed module block with one socket cavity opening towards the clamp (local u = into the socket)."""
    f = face(k)
    yaw = math.radians(SOCKET_YAW[k])
    u, v = _dir(SOCKET_YAW[k]), _dir(SOCKET_YAW[k] + 90)

    def box(tag, u0, u1, v0, v1, z0, z1):
        c = f + u * (u0 + u1) / 2 + v * (v0 + v1) / 2
        return Box(f"module_{k}_{tag}", pos=(float(c[0]), float(c[1]), TOP + (z0 + z1) / 2),
                   size=(u1 - u0, v1 - v0, z1 - z0), yaw=yaw, color=MODULE)

    hw, cw = MOD_W / 2, CAV_W / 2
    return [
        box("back", CAV_D, MOD_D, -hw, hw, 0.0, MOD_H),
        box("roof", 0.0, CAV_D, -hw, hw, CAV_H, MOD_H),
        box("wall_a", 0.0, CAV_D, cw, hw, 0.0, CAV_H),
        box("wall_b", 0.0, CAV_D, -hw, -cw, 0.0, CAV_H),
    ]


CLAMP = Cylinder("harness_clamp", pos=(float(CLAMP_XY[0]), float(CLAMP_XY[1]), TOP + CLAMP_H / 2), radius=CLAMP_R,
                 height=CLAMP_H, color=CLAMP_COL)
_TRUNK_D = 135.0
TRUNK = Box("harness_trunk", pos=(float(CLAMP_XY[0] + _dir(_TRUNK_D)[0] * 0.09), float(CLAMP_XY[1] + _dir(_TRUNK_D)[1] * 0.09),
                                  TOP + 0.011), size=(0.13, 0.022, 0.022), yaw=math.radians(_TRUNK_D), color=CLAMP_COL)
OBJECTS = [CLAMP, TRUNK, *BRANCH_OBJS.values(), *[b for k in SOCKETS for b in _module(k)]]

# -- scoring ---------------------------------------------------------------------------------------------------------
IN_MIN = 0.015  # matched plug: tip at least 15 mm inside its socket
WRONG_MIN = 0.005  # any plug tip 5 mm or more inside a socket that is not its match zeroes the score
LAT_MAX = 0.008  # plug centre within 8 mm of the socket axis (sideways) to count as in the socket
RELEASE_DIST = 0.035  # every gripper TCP at least 3.5 cm from every plug centre at the end
_ENV: dict = {"env": None}


def _install_hooks() -> None:
    """Keep a handle on the env so score() can see where the grippers are."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (no embodiment: offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_harness_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._harness_hooked = True
        cls._task_step = _task_step


_install_hooks()


def plug(objects, b: str) -> tuple[np.ndarray, np.ndarray]:
    """(plug centre, unit axis pointing out through its front end)."""
    st = objects[BRANCH_OBJS[b].name]
    p = np.asarray(st.end_plug if st.end_plug is not None else np.full(3, np.nan), dtype=float)
    a = p - np.asarray(st.points[-1], dtype=float)
    return p, a / (np.linalg.norm(a) + 1e-12)


def depth(p: np.ndarray, k: str) -> float:
    """How far the plug body (centre ``p``) reaches into socket ``k`` past its opening (m); -1 if the plug is not in
    line with the opening (centre more than 8 mm off the socket axis sideways, or lifted above the cavity)."""
    if not np.isfinite(p).all():
        return -1.0
    rel = p[:2] - face(k)
    u, v = _dir(SOCKET_YAW[k]), _dir(SOCKET_YAW[k] + 90)
    if abs(float(rel @ v)) > LAT_MAX or p[2] > TOP + CAV_H + 0.004:
        return -1.0
    return float(rel @ u) + PL / 2


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP; [] if unknown (offline)."""
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
    ps = [plug(objects, b)[0] for b in BRANCHES]
    return all(np.isfinite(p).all() and float(np.linalg.norm(p - g)) >= RELEASE_DIST for g in grippers() for p in ps)


def analyse(objects) -> dict:
    dep = {b: {k: depth(plug(objects, b)[0], k) for k in SOCKETS} for b in BRANCHES}
    wrong = [f"{b}->{k}" for b in BRANCHES for k in SOCKETS if k != MATCH[b] and dep[b][k] >= WRONG_MIN]
    good = [b for b in BRANCHES if dep[b][MATCH[b]] >= IN_MIN]
    return {"depth": dep, "wrong": wrong, "good": good}


def score(objects) -> float:
    """0 if at the end a gripper's TCP is within 3.5 cm of any plug centre, or if any plug reaches 5 mm or more into
    a socket that is not the one its branch is long enough for (short -> near, mid -> middle, long -> far module).
    Otherwise 1/3 per branch whose plug reaches at least 15 mm into its matching socket."""
    if not released(objects):
        return 0.0
    r = analyse(objects)
    if r["wrong"]:
        return 0.0
    return round(len(r["good"]) / 3, 4)


def success(objects) -> bool:
    return bool(score(objects) >= 0.999
                and all(np.linalg.norm(objects[o.name].lin_vel) < 0.02 for o in BRANCH_OBJS.values()))


def _seat(b: str) -> np.ndarray:
    k = MATCH[b]
    c = face(k) + _dir(SOCKET_YAW[k]) * (0.022 - PL / 2)
    return np.array([c[0], c[1], PLUG_Z])


def distance(objects) -> float:
    """Summed distance (m) of the three plugs to their seated poses in their matching sockets; 1 m once a plug is in
    a wrong socket."""
    if analyse(objects)["wrong"]:
        return 1.0
    return float(sum(np.linalg.norm(plug(objects, b)[0] - _seat(b)) for b in BRANCHES))


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score): per branch, 0..1 how far its plug is into its matching socket
    (1 at 15 mm), whether any plug sits 5 mm or more in a wrong socket, and whether the grippers let go."""
    r = analyse(objects)
    out = {f"{b}_seated": float(np.clip(r["depth"][b][MATCH[b]] / IN_MIN, 0.0, 1.0)) for b in BRANCHES}
    out["no_wrong_socket"] = 0.0 if r["wrong"] else 1.0
    out["released"] = 1.0 if released(objects) else 0.0
    return out


# Seeded reset variation for a later runner (the demo runs on the nominal layout). Branch exit headings vary by
# +-3 degrees, modules by +-8 mm / +-3 degrees: the length/distance margins (>= 4.8 cm slack, >= 5.4 cm shortfall)
# and the derangement (each plug >= 1.5x closer to a wrong socket) hold throughout. "obj:module_<k>" means the
# module's four boxes (module_<k>_back/roof/wall_a/wall_b) moved together about its socket opening.
VARIATION = {
    **{f"cable:branch_{b}": {"dx": 0.0, "dy": 0.0, "yaw": 0.05} for b in BRANCHES},
    **{f"obj:module_{k}": {"dx": 0.008, "dy": 0.008, "yaw": 0.05} for k in SOCKETS},
}


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
# Each plug is pinched 12 mm from its branch end, carried just above the table, turned so the wrist camera looks
# along the insertion direction (gripper yaw = socket axis), lined up 15 mm in front of the opening (closed loop on the
# measured plug pose), pushed in to ~21 mm, released; the arm backs off and parks.

FINGER_PINCH = -0.020
GRASP_BACK = 0.012  # pinch this far from the plug's branch end
GZ = TOP + PH + 0.003  # TCP height for the pinch
CARRY = TOP + 0.045
INSERT_TO = 0.021  # target tip depth
PARK = {"left": (0.22, 0.26, TOP + 0.15), "right": (0.22, -0.26, TOP + 0.15)}


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _log(ctx, tag=""):
    st = ctx.states()
    r = analyse(st)
    dep = {b: {k: round(v * 1000, 1) for k, v in r["depth"][b].items() if v > -1} for b in BRANCHES}
    pl = {b: np.round(plug(st, b)[0], 3).tolist() for b in BRANCHES}
    print(f"[t={ctx.t:5.2f}] {tag} depth_mm={dep} wrong={r['wrong']} good={r['good']} rel={released(st)} "
          f"score={score(st)} plugs={pl}", flush=True)


def _pick(ctx, arm, b, target_yaw):
    """Pinch plug ``b`` near its branch end and lift it to carry height. Returns the gripper yaw used, chosen so that
    turning the plug onto ``target_yaw`` is a turn of less than 180 degrees."""
    from yam_arena.constants import FINGER_CLOSED

    p, a = plug(ctx.states(), b)
    h = math.atan2(a[1], a[0])
    yaw = target_yaw - _wrap(target_yaw - h)
    g = p - a * (PL / 2 - GRASP_BACK)
    yield from arm.move((g[0], g[1], CARRY), yaw=yaw, finger=FINGER_PINCH, duration=1.5)
    p, a = plug(ctx.states(), b)
    g = p - a * (PL / 2 - GRASP_BACK)
    yield from arm.move((g[0], g[1], GZ), yaw=yaw, duration=0.9)
    yield from arm.grip(FINGER_CLOSED, 0.5)
    yield from arm.move((g[0], g[1], CARRY), yaw=yaw, duration=0.7)
    return yaw


def _insert(ctx, arm, b, k, via=None):
    """Carry the held plug of ``b`` to socket ``k`` (through waypoint ``via`` if given), line it up on the socket
    axis, push it in, release."""
    from yam_arena.constants import FINGER_CLOSED

    psi = math.radians(SOCKET_YAW[k])
    u, v = _dir(SOCKET_YAW[k]), _dir(SOCKET_YAW[k] + 90)
    f = face(k)
    pre_c = f - u * (PL / 2 + 0.015)  # plug centre with its tip 15 mm in front of the opening
    tcp = pre_c - u * (PL / 2 - GRASP_BACK)
    yaw = psi
    if via is not None:  # lay the spare length of an over-long branch out in a loop first
        yield from arm.move((via[0], via[1], CARRY), yaw=yaw, duration=1.6)
    yield from arm.move((tcp[0], tcp[1], CARRY), yaw=yaw, duration=2.0)
    for _ in range(4):  # closed loop on the plug pose (it sits a little off and turned in the grip)
        yield from arm.wait(0.25)
        p, a = plug(ctx.states(), b)
        dpsi = _wrap(math.atan2(a[1], a[0]) - psi)
        e = pre_c - p[:2]
        if np.linalg.norm(e) < 0.0015 and abs(dpsi) < 0.035:
            break
        if abs(dpsi) >= 0.035:
            yaw -= float(np.clip(dpsi, -0.3, 0.3))
        tcp = tcp + np.clip(e, -0.03, 0.03)
        yield from arm.move((tcp[0], tcp[1], CARRY), yaw=yaw, duration=0.6)
    yield from arm.move((tcp[0], tcp[1], GZ), yaw=yaw, duration=0.8)

    def front_err():
        p, a = plug(ctx.states(), b)
        front = p[:2] + a[:2] * (PL / 2 - 0.010)
        return float((f - front) @ v), p

    for _ in range(4):  # sideways error of the plug's front part, on the table
        yield from arm.wait(0.2)
        ev, _p = front_err()
        if abs(ev) < 0.0008:
            break
        tcp = tcp + v * ev
        yield from arm.move((tcp[0], tcp[1], GZ), yaw=yaw, duration=0.5)
    for attempt in range(4):
        p, _a = plug(ctx.states(), b)
        tip_out = -(float((p[:2] - f) @ u) + PL / 2)  # how far the tip still is in front of the opening
        tcp = tcp + u * (tip_out + INSERT_TO + 0.003)
        yield from arm.move((tcp[0], tcp[1], GZ), yaw=yaw, duration=1.6)
        yield from arm.wait(0.3)
        p, _a = plug(ctx.states(), b)
        dep = depth(p, k)
        if os.environ.get("HARNESS_DBG"):
            ev, _p = front_err()
            print(f"[insert {b}->{k}] attempt {attempt} depth_mm={dep * 1000:.1f} side_mm={ev * 1000:.1f} "
                  f"tcp={[np.round(g, 3).tolist() for g in grippers()]}", flush=True)
            if os.environ.get("HARNESS_DBG") == "2":
                print(f"  cable {b}: {np.round(ctx.states()[BRANCH_OBJS[b].name].points[:, :2], 3).tolist()}",
                      flush=True)
        if dep >= IN_MIN + 0.003:
            break
        # stalled at the opening: back off, re-centre sideways, try again
        p, _a = plug(ctx.states(), b)
        tcp = p[:2] - u * (PL / 2 - GRASP_BACK) - u * 0.008
        yield from arm.move((tcp[0], tcp[1], GZ + 0.002), yaw=yaw, duration=0.6)
        yield from arm.wait(0.2)
        ev, _p = front_err()
        if attempt >= 1 and abs(ev) > 0.002:
            # the plug keeps sitting turned in the grip (the branch's spare length pushes on it): let go, and
            # pinch it again squarely along the socket axis where it lies, 8 mm back from the opening
            yield from arm.grip(FINGER_PINCH, 0.4)
            yield from arm.move((tcp[0], tcp[1], GZ + 0.02), yaw=yaw, duration=0.4)
            p, a = plug(ctx.states(), b)
            g = p[:2] - a[:2] * (PL / 2 - GRASP_BACK)
            yaw = psi
            yield from arm.move((g[0], g[1], GZ + 0.02), yaw=yaw, duration=0.5)
            yield from arm.move((g[0], g[1], GZ), yaw=yaw, duration=0.5)
            yield from arm.grip(FINGER_CLOSED, 0.5)
            tcp = g
            yield from arm.wait(0.2)
            ev, _p = front_err()
        tcp = tcp + v * ev
        yield from arm.move((tcp[0], tcp[1], GZ), yaw=yaw, duration=0.5)
    yield from arm.grip(FINGER_PINCH, 0.5)
    tcp = tcp - u * 0.03
    yield from arm.move((tcp[0], tcp[1], CARRY + 0.04), yaw=yaw, duration=0.8)


def _connect(ctx, arm, b, k, via=None):
    yield from _pick(ctx, arm, b, math.radians(SOCKET_YAW[k]))
    yield from _insert(ctx, arm, b, k, via)
    _log(ctx, f"{b} -> {k}")


def _drop(ctx, arm, side, b, xy, yaw_deg):
    """Pick plug ``b`` and set it down flat at ``xy`` pointing along ``yaw_deg`` (a hand-over through the table)."""
    psi = math.radians(yaw_deg)
    yield from _pick(ctx, arm, b, psi)
    u = _dir(yaw_deg)
    tcp = np.asarray(xy) - u * (PL / 2 - GRASP_BACK)
    yield from arm.move((tcp[0], tcp[1], CARRY), yaw=psi, duration=2.0)
    yield from arm.move((tcp[0], tcp[1], GZ + 0.003), yaw=psi, duration=0.8)
    yield from arm.grip(FINGER_PINCH, 0.5)
    yield from arm.move((tcp[0], tcp[1], CARRY + 0.04), yaw=psi, duration=0.6)
    yield from _park(arm, side)


def _park(arm, side):
    yield from arm.move(PARK[side], yaw=0.0, duration=1.2)


def demo_script(ctx):
    mode = os.environ.get("HARNESS_DEMO", "correct")  # test only: correct | nothing | nearest | greedy
    arms = {"left": ctx.left, "right": ctx.right}
    _log(ctx, "start")
    if mode == "nothing":
        for _ in range(4):
            yield from ctx.left.wait(1.0)
        _log(ctx, "end")
        return
    plan = {
        # 1) in parallel: left arm short branch -> near module; right arm long branch -> far module
        # 2) right arm middle branch -> middle module while the left arm parks
        "correct": [{"left": ("short", "near"), "right": ("long", "far")},
                    {"left": "park", "right": ("mid", "mid")}],
        # nearest: each plug to the socket nearest where it lies, as far as it reaches: the long plug lies nearest
        # the middle socket (wrong; its 16 cm of spare length makes this insertion fiddly); the short
        # plug's nearest socket (middle) is out of its reach, its nearest reachable one is the near one (right)
        "nearest": [{"left": ("short", "near"), "right": "park"},
                    {"left": "park", "right": ("long", "mid")}],
        # greedy: the long branch reaches everything, so it goes into the closest module (near) first. The left arm
        # first swings the short branch out of its way (to the left); the right arm sets the long plug down where
        # the left arm can take it; the left arm lays the long branch's spare length out in a loop to the left and
        # plugs it into the near module; then the right arm plugs the middle
        # branch into the middle module (one arm at a time: both arms at the near and middle modules collide)
        "greedy": [{"left": ("drop", "short", (0.19, 0.31), 90.0), "right": "park"},
                   {"left": "park", "right": ("drop", "long", (0.32, 0.06), -5.0)},
                   {"left": ("long", "near", (0.29, 0.33)), "right": "park"},
                   {"left": "park", "right": ("mid", "mid")}],
    }
    for step in plan.get(mode, plan["correct"]):
        gens = []
        for side, item in step.items():
            if item == "park":
                gens.append(_park(arms[side], side))
            elif item[0] == "drop":
                gens.append(_drop(ctx, arms[side], side, *item[1:]))
            else:
                gens.append(_connect(ctx, arms[side], *item))
        if len(gens) == 2:
            yield from ctx.both(*gens)
        else:
            yield from gens[0]
    yield from ctx.both(_park(ctx.left, "left"), _park(ctx.right, "right"))
    for _ in range(2):
        yield from ctx.left.wait(1.0)
    _log(ctx, "end")

