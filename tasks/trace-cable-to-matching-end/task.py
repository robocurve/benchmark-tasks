"""On the station table: five identical black 8 mm cables leave five fixed device boxes (9 x 9 x 7 cm) standing in a
row at the far side of the table, seen from the robot: YELLOW (left), RED, GREEN (middle), BLUE, ORANGE (right).
Each cable runs straight towards the robot into a grey cable cover (a 15 x 86 cm plate on two end walls, 2.5 cm
clear of the table). Under the cover, out of sight, each cable bends round a low guide post, runs diagonally across
under the cover, crossing other cables, bends round a second post and comes out of the cover's robot side straight
towards the robot. The cables' free ends (identical black 35 x 16 x 12 mm plugs) lie in a row in front of the cover,
every one away from its own box: the plugs in front of the red and the blue box belong to other boxes, the red box's
plug lies at the far right and the blue box's at the far left. Two plug holders (45 x 40 mm slots between 8 mm walls,
closed on the robot side by a white end stop), one with blue walls on the left and one with red walls on the right,
are fixed near the robot.

The robot must put the red box's plug in the red holder and the blue box's plug in the blue holder and leave the
other three plugs where they lie. Which plug belongs to which box cannot be seen. It can be found by taking hold of a
cable where it leaves its box and pulling it sideways: the cable slides round its posts and its own plug slides
towards the cover. Reach makes this a two-arm job (for a gripper pointing down): only the left arm reaches the red box's cable and the blue box's
plug, only the right arm the blue box's cable and the red box's plug. The tug also leaves loose cable on the box
side of the cover, which is what lets the plug reach its holder. In simulation a cable's fixing at its box lets go above a 300 N
pull (a real taut cable would just stop the plug); a plug dragged hard against a cable still held taut round its posts pulls the cable out of its box, and that
scores 0.

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
import cableshape  # noqa: E402  (lays the cables in their starting shape at every reset)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Put the plug of the red box's cable in the red holder and the plug of the blue box's cable in the "
               "blue holder. Leave the other plugs where they are.")

# -- devices (fixed boxes) at the far side ----------------------------------------------------------------------------
DEV_SIZE = (0.09, 0.09, 0.07)
DEV_FACE_X = 0.53  # robot-facing side of the boxes
COLOURS = {
    "yellow": (0.92, 0.80, 0.10),
    "red": (0.85, 0.10, 0.08),
    "green": (0.10, 0.62, 0.22),
    "blue": (0.10, 0.30, 0.90),
    "orange": (0.95, 0.45, 0.05),
}
# box: (box y, plug y)
LAYOUT = {
    "yellow": (0.24, 0.02),
    "red": (0.12, -0.28),
    "green": (0.00, 0.16),
    "blue": (-0.12, 0.30),
    "orange": (-0.24, -0.12),
}
DEV_OBJS = [Box(f"{n}_box", pos=(DEV_FACE_X + DEV_SIZE[0] / 2, by, TOP + DEV_SIZE[2] / 2), size=DEV_SIZE,
                color=COLOURS[n]) for n, (by, _py) in LAYOUT.items()]

# -- cable cover: a plate on two end walls, 2.5 cm clear of the table ------------------------------------------------
COVER_X0, COVER_X1 = 0.250, 0.400  # robot side, far side
COVER_Y = 0.42  # half width (inner)
COVER_GAP, COVER_T = 0.025, 0.010
GREY = (0.45, 0.46, 0.48)
COVER = [
    Box("cover_plate", pos=((COVER_X0 + COVER_X1) / 2, 0.0, TOP + COVER_GAP + COVER_T / 2),
        size=(COVER_X1 - COVER_X0, 2 * COVER_Y + 0.02, COVER_T), color=GREY),
    *[Box(f"cover_end_{s}", pos=((COVER_X0 + COVER_X1) / 2, sg * (COVER_Y + 0.005), TOP + COVER_GAP / 2),
          size=(COVER_X1 - COVER_X0, 0.010, COVER_GAP), color=GREY) for s, sg in (("l", 1), ("r", -1))],
]

# -- cables -----------------------------------------------------------------------------------------------------------
R = 0.004  # 8 mm cord
Z0 = TOP + R + 0.0005  # cable centre lying on the table
Z_EXIT = TOP + 0.013  # cables leave their boxes 1.3 cm up (clears the guide posts while spawning)
EXIT_RAMP = 0.05  # each cable comes down to the table within 5 cm of its box
BLACK = (0.05, 0.05, 0.05)
PLUG = Plug(size=(0.035, 0.016, 0.012), mass=0.015, color=(0.06, 0.06, 0.06))
START_X = DEV_FACE_X - 0.006  # anchored end caps clear of the box face
END_X = 0.1875  # cable end (plug back face); plug centres at x 0.17
TARGETS = {"red": "cable_red", "blue": "cable_blue"}


# Under the cover each cable bends round two fixed guide pins (like the dividers in a cable duct): one just inside the
# far edge, one just inside the robot-side edge, with a straight diagonal run between them. The pins keep the hidden
# route from soaking up a pull, so a tug at one end moves the other end.
PIN_X = (0.375, 0.275)  # entry pins (far side), exit pins (robot side)
PIN_R, PIN_H = 0.020, 0.008  # low posts: the cables spawn straight over them for a moment before being laid
RHO = 0.027  # cable centre radius round a pin (pin + cable radius + 3 mm; 2 cm chords cut 1-2 mm inside)


def _turn(by: float, py: float) -> float:
    """Angle each bend turns through so the diagonal joins the two pins' arcs (bisection)."""
    s = 1.0 if py > by else -1.0
    dx = PIN_X[0] - PIN_X[1]
    lo, hi = 0.0, math.pi / 2
    for _ in range(60):
        th = (lo + hi) / 2
        f = math.atan2(abs((py - by) - 2 * s * RHO * (1 - math.cos(th))), dx - 2 * RHO * math.sin(th)) - th
        lo, hi = (th, hi) if f > 0 else (lo, th)
    return lo


def _pins(by: float, py: float) -> tuple[np.ndarray, np.ndarray]:
    s = 1.0 if py > by else -1.0
    return np.array([PIN_X[0], by + s * RHO]), np.array([PIN_X[1], py - s * RHO])


def _route(by: float, py: float) -> np.ndarray:
    """2D centreline: box face -> straight to the entry pin -> round it -> diagonal -> round the exit pin -> out."""
    s, th = (1.0 if py > by else -1.0), _turn(by, py)
    a = np.linspace(0, th, 10)
    arc1 = [(PIN_X[0] - RHO * math.sin(t), by + s * RHO * (1 - math.cos(t))) for t in a]
    arc2 = [(PIN_X[1] + RHO * math.sin(t), py - s * RHO * (1 - math.cos(t))) for t in a[::-1]]
    return np.array([(START_X, by), *arc1, *arc2, (END_X, py)])


def _dense(p: np.ndarray, step: float = 0.002) -> np.ndarray:
    out = []
    for a, b in zip(p[:-1], p[1:]):
        k = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * t for t in np.linspace(0, 1, k, endpoint=False)]
    return np.array(out + [p[-1]])


def _length(p: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def _crossings(a: np.ndarray, b: np.ndarray) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Intersections of two 2D polylines: (point, direction of a there, direction of b there)."""
    out = []
    for i in range(len(a) - 1):
        p, r = a[i], a[i + 1] - a[i]
        for j in range(len(b) - 1):
            q, s = b[j], b[j + 1] - b[j]
            den = r[0] * s[1] - r[1] * s[0]
            if abs(den) < 1e-12:
                continue
            t = ((q - p)[0] * s[1] - (q - p)[1] * s[0]) / den
            u = ((q - p)[0] * r[1] - (q - p)[1] * r[0]) / den
            if 0 <= t <= 1 and 0 <= u <= 1:
                out.append((p + t * r, r / np.linalg.norm(r), s / np.linalg.norm(s)))
    return out


ROUTES2D = {n: _dense(_route(*v)) for n, v in LAYOUT.items()}
PIN_POS = {n: _pins(*v) for n, v in LAYOUT.items()}
LIFT = Z0 + 2 * R + 0.0015 - Z0  # over-cable centre this much higher at a crossing
FLAT, BLEND = 0.016, 0.020
PATHS: dict[str, np.ndarray] = {}
CROSSINGS: list[tuple[str, str, np.ndarray]] = []  # (over cable, under cable, point)
_z = {n: np.full(len(p), Z0) for n, p in ROUTES2D.items()}
for _a, _b in [(a, b) for i, a in enumerate(LAYOUT) for b in list(LAYOUT)[i + 1:]]:
    for _pt, _da, _db in _crossings(ROUTES2D[_a], ROUTES2D[_b]):
        _over = _a if abs(_da[0]) > abs(_db[0]) else _b  # the part running along x (in or out) goes over
        CROSSINGS.append((_over, _b if _over == _a else _a, _pt))
        d = np.linalg.norm(ROUTES2D[_over] - _pt, axis=1) - FLAT
        w = np.clip(1 - d / BLEND, 0, 1)
        _z[_over] = np.maximum(_z[_over], Z0 + LIFT * (0.5 - 0.5 * np.cos(math.pi * w)))
for _n in LAYOUT:
    _ze = Z_EXIT
    _w = np.clip((ROUTES2D[_n][:, 0] - (START_X - EXIT_RAMP)) / EXIT_RAMP, 0, 1)
    PATHS[_n] = np.c_[ROUTES2D[_n], np.maximum(_z[_n], Z0 + (_ze - Z0) * (0.5 - 0.5 * np.cos(math.pi * _w)))]

SPAWN_TILT = {"yellow": 0.3, "red": 0.15, "orange": 0.1}  # rad, + = towards -y
ANCHOR_BREAK_N = 300.0  # N: each cable's fixing at its box lets go above this pull (only under a hard yank)
PULLED_OUT = 0.01  # m: a cable's first segment moved this far from its box = pulled out
_orig_spawn = cablekit._spawn_cable


def _spawn_cable_ext(prim_path, cfg, *a, **kw):
    """cablekit's cable, with a breakable box fixing: without it, an arm dragging a plug against a cable held taut
    round its posts stretches the segment chain until the simulation fails."""
    prim = _orig_spawn(prim_path, cfg, *a, **kw)
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    j = get_current_stage().GetPrimAtPath(f"{prim_path}/anchor")
    if j.IsValid() and ANCHOR_BREAK_N > 0:
        UsdPhysics.Joint(j).CreateBreakForceAttr(ANCHOR_BREAK_N)
        UsdPhysics.Joint(j).CreateBreakTorqueAttr(ANCHOR_BREAK_N * 0.05)
    return prim


cablekit._spawn_cable = _spawn_cable_ext

PIN_MU = 0.15  # smooth guide posts (polished / PTFE), friction "min"-combined


def _patch_pins(scene) -> None:
    """Give the guide posts a smooth material (task objects default to a rubbery one)."""
    if getattr(scene, "_trace_pins", False):
        return
    orig = scene.task_object

    def task_object(spec):
        obj = orig(spec)
        if spec.name.startswith("pin_"):
            import isaaclab.sim as sim_utils

            obj.spawner_cfg.physics_material = sim_utils.RigidBodyMaterialCfg(
                static_friction=PIN_MU, dynamic_friction=PIN_MU, friction_combine_mode="min",
                restitution_combine_mode="min")
        return obj

    scene.task_object = task_object
    scene._trace_pins = True


_orig_patch_scene = cablekit._patch_scene


def _patch_scene_ext(scene) -> None:
    _orig_patch_scene(scene)
    _patch_pins(scene)


cablekit._patch_scene = _patch_scene_ext
if "yam_arena.scene" in sys.modules:
    _patch_pins(sys.modules["yam_arena.scene"])

CABLES = {}
for _n, (_by, _py) in LAYOUT.items():
    _L = _length(ROUTES2D[_n])
    # Every cable spawns straight for a moment before the reset lays it in its route. Three of them are angled
    # (first segment only) so that straight spawn misses the arm bases, the holders and the other cables.
    _yaw = math.pi + SPAWN_TILT.get(_n, 0.0)
    CABLES[_n] = cable(f"cable_{_n}", start=(START_X, _by, Z_EXIT), yaw=_yaw, length=_L,
                       segment=_L / max(2, round(_L / 0.02)), radius=R, anchor_start=True, end_plug=PLUG, color=BLACK,
                       bend_stiffness=0.003, bend_damping=0.0005)
CABLE_OF = {f"cable_{n}": n for n in CABLES}
WRONG = [f"cable_{n}" for n in CABLES if n not in TARGETS]


def _shape(emb) -> None:
    if os.environ.get("TRACE_DBG"):
        from cablekit import _cable_state

        for n, spec in CABLES.items():
            pts = _cable_state(emb._env, spec).points
            print(f"[preshape] {n} finite={np.isfinite(pts).all()} first={np.round(pts[0], 3)} "
                  f"last={np.round(pts[-1], 3)}", flush=True)
    for n, spec in CABLES.items():
        p = PATHS[n]  # the anchored first segment is laid along the spawn yaw by cableshape: start after it
        cableshape.preshape(emb, spec, p[p[:, 0] <= START_X - spec.seg_len - 0.001], hold="ramp", settle_s=0)
    cableshape.settle(emb, 2.0)


cableshape.on_reset(_shape)

# -- plug holders: U-channels along x, closed on the robot side -------------------------------------------------------
SLOT_L, SLOT_W, SLOT_H, WALL_T = 0.045, 0.040, 0.008, 0.004
STOP_H = 0.020
WHITE = (0.95, 0.95, 0.95)
HOLDERS = {"blue": np.array([0.11, 0.18]), "red": np.array([0.11, -0.18])}  # slot floor centres
HOLDER_OBJS = []
for _c, _hc in HOLDERS.items():
    HOLDER_OBJS += [
        Box(f"holder_{_c}_wall_{s}", pos=(float(_hc[0]), float(_hc[1] + sg * (SLOT_W / 2 + WALL_T / 2)),
                                          TOP + SLOT_H / 2), size=(SLOT_L, WALL_T, SLOT_H), color=COLOURS[_c])
        for s, sg in (("l", 1), ("r", -1))
    ]
    HOLDER_OBJS.append(Box(f"holder_{_c}_stop", pos=(float(_hc[0] - SLOT_L / 2 - WALL_T / 2), float(_hc[1]),
                                                     TOP + STOP_H / 2),
                           size=(WALL_T, SLOT_W + 2 * WALL_T, STOP_H), color=WHITE))

PIN_OBJS = [Cylinder(f"pin_{n}_{k}", pos=(float(p[0]), float(p[1]), TOP + PIN_H / 2), radius=PIN_R,
                     height=PIN_H, color=GREY) for n, pp in PIN_POS.items() for k, p in zip(("in", "out"), pp)]

OBJECTS = [*DEV_OBJS, *COVER, *PIN_OBJS, *CABLES.values(), *HOLDER_OBJS]

# -- scoring ---------------------------------------------------------------------------------------------------------
WRONG_MOVE = 0.05  # one of the three other plugs moved this far (3D) from where it lay = moved/picked
PLUG_HALF_H = PLUG.size[2] / 2
RELEASE_DIST = 0.03  # every gripper TCP at least this far from a plug's centre = released


def seated(objects, name: str, holder: str) -> bool:
    """Plug of cable ``name`` inside ``holder``'s slot (sticking out of its open far end by at most 1 cm), lying on
    the table between the walls."""
    p = objects[name].end_plug
    if p is None:
        return False
    hc, half = HOLDERS[holder], PLUG.size[0] / 2
    x0, x1 = hc[0] - SLOT_L / 2, hc[0] + SLOT_L / 2
    ok_x = x0 + half - 0.003 <= p[0] <= x1 - half + 0.010
    ok_y = abs(p[1] - hc[1]) <= (SLOT_W - PLUG.size[1]) / 2 + 0.002
    ok_z = p[2] <= TOP + PLUG_HALF_H + 0.004
    return bool(ok_x and ok_y and ok_z)


# Episode memory, reset at every episode start (wrapped embodiment reset). Violations latch for the whole episode.
_EP: dict = {"start": None, "roots": None, "wrong": False, "swapped": False, "pulled": False, "max_wrong": 0.0,
             "max_root": 0.0}
_ENV: dict = {"env": None}


def _reset_episode() -> None:
    _EP.update(start=None, roots=None, wrong=False, swapped=False, pulled=False, max_wrong=0.0, max_root=0.0)


def _install_hooks() -> None:
    """Reset the episode memory on every embodiment reset; keep the sim env so score() can check the grippers."""
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
    if not getattr(cls._task_step, "_trace_hooked", False):
        base_step = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base_step(self, *a, **k)

        _task_step._trace_hooked = True
        cls._task_step = _task_step


_install_hooks()


def grippers() -> list[np.ndarray]:
    """World TCPs of both grippers, [] before the first step."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = (getattr(base.scene.env_origins, "torch", base.scene.env_origins)[0]).detach().cpu().numpy()
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


def released(objects, name: str) -> bool:
    p = objects[name].end_plug
    return p is not None and all(float(np.linalg.norm(p - g)) >= RELEASE_DIST for g in grippers())


def _track(objects) -> None:
    plugs = {n: objects[n].end_plug for n in CABLE_OF}
    if any(p is None or not np.isfinite(p).all() for p in plugs.values()):
        return
    if _EP["start"] is None:  # first look of the episode, after the cables were laid and settled
        _EP["start"] = {n: np.array(p) for n, p in plugs.items()}
        _EP["roots"] = {n: np.array(objects[n].points[0]) for n in CABLE_OF}
    for n in CABLE_OF:  # a cable pulled out of its box (its anchor let go)
        dr = float(np.linalg.norm(objects[n].points[0] - _EP["roots"][n]))
        _EP["max_root"] = max(_EP["max_root"], dr)
        if dr > PULLED_OUT:
            _EP["pulled"] = True
    for n in WRONG:
        d = float(np.linalg.norm(plugs[n] - _EP["start"][n]))
        _EP["max_wrong"] = max(_EP["max_wrong"], d)
        if d > WRONG_MOVE:
            _EP["wrong"] = True
    if seated(objects, TARGETS["red"], "blue") or seated(objects, TARGETS["blue"], "red"):
        _EP["swapped"] = True


def _bad() -> bool:
    return bool(_EP["wrong"] or _EP["swapped"] or _EP["pulled"])


def _done(objects) -> dict:
    return {c: seated(objects, TARGETS[c], c) and released(objects, TARGETS[c]) for c in TARGETS}


def success(objects) -> bool:
    _track(objects)
    return bool(not _bad() and all(_done(objects).values())
                and all(objects[TARGETS[c]].resting(0.02, 1.0) for c in TARGETS))


def score(objects) -> float:
    """0 if at any moment one of the three other plugs (yellow, green, orange box) was more than 5 cm from where it
    lay at the start, or the red box's plug lay in the blue holder or the blue box's plug in the red holder, or a
    cable was pulled out of its box (its first segment moved more than 1 cm). Otherwise 1.0 if at the end both plugs
    lie in their own holders, each released (no gripper within 3 cm); 0.5 if exactly one does; else 0."""
    _track(objects)
    if _bad():
        return 0.0
    return 0.5 * sum(_done(objects).values())


def distance(objects) -> float:
    """Sum of the two target plugs' distances to their holders' centres (m); 2 m after a violation."""
    _track(objects)
    if _bad():
        return 2.0
    tot = 0.0
    for c, n in TARGETS.items():
        p = objects[n].end_plug
        tot += float(np.linalg.norm(p[:2] - HOLDERS[c])) if p is not None else 1.0
    return tot


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    _track(objects)
    d = _done(objects)
    moved = {c: float(np.linalg.norm(objects[n].end_plug - _EP["start"][n])) if _EP["start"] else 0.0
             for c, n in TARGETS.items()}
    return {"red_plug_moved": float(moved["red"] > 0.03), "blue_plug_moved": float(moved["blue"] > 0.03),
            "red_placed": float(d["red"]), "blue_placed": float(d["blue"]),
            "others_kept": float(not _EP["wrong"]), "no_swap": float(not _EP["swapped"])}


# Seeded variation for later robustness runs (the demo and the submission use the nominal layout).
VARIATION = {"obj:holders": {"dx": 0.01, "dy": 0.01, "yaw": 0.0}, "cables:plug_row": {"dx": 0.01, "dy": 0.0}}

# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
HI = TOP + 0.075  # carry height
TUG_X = 0.43  # grasp point on a cable between the cover and its box
TUG_SIDE = 0.09  # pull the hooked cable this far sideways along the table
GRASP_OPEN = -0.014  # finger joint: pads ~28 mm apart, for pinching a cable lying on the table
GOAL_DX = 0.008  # aim the plug centre this far past the slot centre (towards the open end), clear of the end stop
TUG_HOOK = -0.006  # finger joint for the tug: pads ~12 mm apart round the 8 mm cable, so it slides through
TUG_YAW = 0.0  # (yaw pi has no IK solution here; at yaw 0 the wrist camera stays 2 cm short of the box)


def _log(ctx, tag=""):
    st = ctx.states()
    pl = " ".join(f"{CABLE_OF[n][:2]}={np.round(st[n].end_plug[:2], 3)}" for n in CABLE_OF)
    print(f"[t={ctx.t:5.2f}] {tag}: {pl} wrong={_EP['wrong']} maxwrong={_EP['max_wrong']:.3f} "
          f"swap={_EP['swapped']} pulled={_EP['pulled']} maxroot={_EP['max_root']:.3f} done={_done(st)} "
          f"score={score(st):.2f}", flush=True)


def _dbg(ctx, tag):
    if os.environ.get("TRACE_DBG"):
        _log(ctx, tag)


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _tug(ctx, arm, box: str, f_open, f_closed):
    """Hook ``box``'s cable between the cover and the box (fingers almost closed round it, so it can slide between
    them) and pull it sideways (away from the middle) along the table: that draws cable out from under the cover,
    so the matching plug slides. Then move back and let go.
    Returns the cable whose plug moved most."""
    by = LAYOUT[box][0]
    out = math.copysign(TUG_SIDE, by)
    before = {n: np.array(ctx.states()[n].end_plug) for n in CABLE_OF}
    pts0 = np.array(ctx.states()[f"cable_{box}"].points)
    gz = Z0 + 0.010
    yield from arm.move((TUG_X, by, HI), yaw=TUG_YAW, finger=GRASP_OPEN, duration=1.4)
    yield from arm.move((TUG_X, by, gz), yaw=TUG_YAW, duration=0.8)
    yield from arm.grip(TUG_HOOK, 0.5)
    _dbg(ctx, f"tug {box}: grasp tcp={np.round(arm.tcp(), 3)}")
    yield from arm.move((TUG_X, by + out, gz + 0.005), yaw=TUG_YAW, duration=1.5)
    yield from arm.wait(0.5)
    st = ctx.states()
    moved = {n: float(np.linalg.norm(st[n].end_plug[:2] - before[n][:2])) for n in CABLE_OF}
    pts = st[f"cable_{box}"].points
    _dbg(ctx, f"tug {box}: tcp={np.round(arm.tcp(), 3)} cable max |dy| near the box="
              f"{np.abs(pts[pts[:, 0] > COVER_X1, 1] - by).max():.3f}")
    _dbg(ctx, f"tug {box}: point moves (mm, from the box) "
              f"{np.round(np.linalg.norm(pts - pts0, axis=1)[::3] * 1000).astype(int).tolist()}")
    _dbg(ctx, f"tug {box}: moved " + " ".join(f"{CABLE_OF[n]}={v:.3f}" for n, v in moved.items()))
    yield from arm.move((TUG_X, by, gz + 0.005), yaw=TUG_YAW, duration=1.0)
    yield from arm.grip(f_open, 0.4)
    yield from arm.move((TUG_X, by, HI), yaw=TUG_YAW, duration=0.6)
    return max(moved, key=moved.get)


def _plug_to_holder(ctx, arm, name, holder, f_open, f_closed, release=True, goal=None):
    """Grasp ``name``'s plug where it lies, lift it, turn it to lie along x and lay it in ``holder``'s slot."""
    hc = HOLDERS[holder]
    goal = np.array([hc[0] + GOAL_DX, hc[1]]) if goal is None else np.asarray(goal)
    c = ctx.states()[name]
    pp = c.end_plug
    gy = _grip_yaw(math.atan2(c.end_dir[1], c.end_dir[0]))
    yield from arm.move((pp[0], pp[1], HI), yaw=gy, finger=f_open, duration=1.4)
    pp = ctx.states()[name].end_plug  # closed loop: grasp where the plug is now
    yield from arm.move((pp[0], pp[1], pp[2] + 0.010), yaw=gy, duration=1.0)
    yield from arm.grip(f_closed, 0.6)
    yield from arm.move((pp[0], pp[1], HI), yaw=gy, duration=0.8)
    yield from arm.move((float(goal[0]), float(goal[1]), HI), yaw=0.0, duration=1.6)
    off = ctx.states()[name].end_plug[:2] - arm.tcp()[:2]
    gx, gyy = goal - off
    yield from arm.move((gx, gyy, HI), yaw=0.0, duration=0.6)
    # lower in stages, re-centring the plug over the slot (twice at the last height)
    for z in (TOP + 0.045, TOP + 0.032, TOP + 0.023, TOP + 0.023):
        yield from arm.move((gx, gyy, z), yaw=0.0, duration=0.5)
        off = ctx.states()[name].end_plug[:2] - arm.tcp()[:2]
        gx, gyy = goal - off
        yield from arm.move((gx, gyy, z), yaw=0.0, duration=0.3)
    c = ctx.states()[name]
    _dbg(ctx, f"{name} lowered z={c.end_plug[2] - TOP:.4f} dir={np.round(c.end_dir, 2)}")
    if not release:
        return
    yield from arm.grip(f_open, 1.0)
    yield from arm.move((gx, gyy, HI), yaw=0.0, duration=0.8)
    c = ctx.states()[name]
    _dbg(ctx, f"{name} released z={c.end_plug[2] - TOP:.4f} dir={np.round(c.end_dir, 2)}")


def _park(arm, side: int):
    yield from arm.move((0.22, side * 0.36, HI + 0.05), yaw=0.0, duration=1.0)


def demo_script(ctx):
    """TRACE_DEMO=correct (default): the right arm tugs the blue box's cable, the left arm the red box's; each
    sees which plug slid, then the left arm lays the blue box's plug in the blue holder and the right arm the red
    box's plug in the red holder.
    Test variants: nothing | hold (both placed, grippers stay closed on them) | tier05 (only the blue box's plug
    placed: 0.5) | onearm (left arm only, no tugs, knowing the answer: it lifts the blue box's plug towards the blue
    holder, but with no cable drawn out at the box the cable goes taut round its posts and pulls out of its box: 0) |
    front (no tracing: the plug lying in front of each named box) | mirror (the plugs at the mirrored positions) |
    random (two other plugs: yellow's and green's) | swap (traced, but each plug in the other holder) | push (traced,
    the blue box's plug carried low and set down on the table just outside the blue holder, under 5 cm from its slot
    centre, released)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    mode = os.environ.get("TRACE_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    fo, fc = FINGER_OPEN, FINGER_CLOSED
    for _ in range(10):
        yield
    _log(ctx, "start")
    by_y = {}  # plug y at the start -> cable name
    st = ctx.states()
    for n in CABLE_OF:
        by_y[round(float(st[n].end_plug[1]), 2)] = n

    def plug_near(y):
        return min(CABLE_OF, key=lambda n: abs(ctx.states()[n].end_plug[1] - y))

    if mode == "nothing":
        yield from L.wait(5.0)
    elif mode in ("front", "mirror", "random"):
        if mode == "front":  # the plug in front of each named box
            bl, rd = plug_near(LAYOUT["blue"][0]), plug_near(LAYOUT["red"][0])
        elif mode == "mirror":
            bl, rd = plug_near(-LAYOUT["blue"][0]), plug_near(-LAYOUT["red"][0])
        else:  # two other plugs (the yellow and the green box's)
            bl, rd = plug_near(0.02), plug_near(0.16)
        _log(ctx, f"{mode}: blue holder <- {CABLE_OF[bl]}, red holder <- {CABLE_OF[rd]}")
        arm_b = L if ctx.states()[bl].end_plug[1] > -0.02 else Rt
        yield from _plug_to_holder(ctx, arm_b, bl, "blue", fo, fc)
        yield from _park(arm_b, 1 if arm_b is L else -1)
        arm_r = Rt if ctx.states()[rd].end_plug[1] < 0.02 else L
        yield from _plug_to_holder(ctx, arm_r, rd, "red", fo, fc)
        yield from _park(arm_r, 1 if arm_r is L else -1)
    elif mode == "onearm":
        yield from _plug_to_holder(ctx, L, TARGETS["blue"], "blue", fo, fc)
        yield from _park(L, 1)
    else:
        blue = yield from _tug(ctx, Rt, "blue", fo, fc)
        yield from _park(Rt, -1)
        red = yield from _tug(ctx, L, "red", fo, fc)
        _log(ctx, f"tugs: blue box -> {CABLE_OF[blue]}, red box -> {CABLE_OF[red]}")
        if mode == "swap":
            yield from _plug_to_holder(ctx, L, blue, "red", fo, fc)
            yield from _park(L, 1)
            yield from _plug_to_holder(ctx, Rt, red, "blue", fo, fc)
            yield from _park(Rt, -1)
        elif mode == "push":  # carried low, set down just outside the blue holder
            hc = HOLDERS["blue"]
            c = ctx.states()[blue]
            gy = _grip_yaw(math.atan2(c.end_dir[1], c.end_dir[0]))
            pp = c.end_plug
            yield from L.move((pp[0], pp[1], HI), yaw=gy, finger=fo, duration=1.4)
            pp = ctx.states()[blue].end_plug
            gz = pp[2] + 0.010
            yield from L.move((pp[0], pp[1], gz), yaw=gy, duration=1.0)
            yield from L.grip(fc, 0.6)
            goal = np.array([hc[0] + 0.010, hc[1] + 0.042])
            yield from L.move((pp[0], pp[1], gz + 0.015), yaw=gy, duration=0.5)
            yield from L.move((float(goal[0]), float(goal[1]), gz + 0.015), yaw=gy, duration=3.0)  # carried low
            off = ctx.states()[blue].end_plug[:2] - L.tcp()[:2]
            gx, gyy = goal - off
            yield from L.move((gx, gyy, gz + 0.015), yaw=gy, duration=0.6)
            yield from L.move((gx, gyy, gz), yaw=gy, duration=0.5)
            yield from L.grip(fo, 1.0)
            yield from L.move((gx, gyy, HI), yaw=gy, duration=0.8)
            _log(ctx, f"set down: plug {np.round(ctx.states()[blue].end_plug[:2], 3)}, "
                      f"{np.linalg.norm(ctx.states()[blue].end_plug[:2] - np.array(hc)) * 100:.1f} cm from the slot")
            yield from _park(L, 1)
        else:
            yield from _plug_to_holder(ctx, L, blue, "blue", fo, fc, release=(mode != "hold"))
            if mode != "hold":
                yield from _park(L, 1)
            if mode != "tier05":
                yield from _plug_to_holder(ctx, Rt, red, "red", fo, fc, release=(mode != "hold"))
                if mode != "hold":
                    yield from _park(Rt, -1)
    _log(ctx, "end")
    yield from L.wait(1.5)
    _log(ctx, "end+1.5s")
