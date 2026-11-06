"""Stage rigging mock-up on the station table: strike the spare lines from a pin rail, leave the lines in use.

A brown wooden pin rail (80 x 860 x 40 mm) runs left-right 22.5 cm in front of the robot. Four black belaying pins
(20 mm round, 45 mm tall) stand on it at y +36, +24, -24 and -36 cm. Each pin carries the dark red rectangular eye
(70 x 90 x 10 mm bars, 50 x 70 mm opening) of an 8 mm rope line. Every line leaves its eye towards the back and runs
into the fly well: a dark grey block (160 x 860 x 41 mm) at the back with an opaque lid, so nothing inside the well
can be seen. Two lines are in use (left outer pin y +36, right inner pin y -24): inside the well they hold a load, modelled as a
spring-loaded slide that pulls about 1 N; they run straight and taut from the eye to the well, 4.5 cm above the
table, pulling the eye against its pin. Lifted off its pin, such an eye is pulled back towards the well. The other two lines (left inner y +24, right outer y -36) are spare: their far ends lie loose in
the well and their spans hang slack from the rail onto the table in a loose S. Two blue open crates
(inner 105 x 105 mm, 35 mm tall, 6 mm walls) stand in front of the rail at y +-30 cm.

The robot must strike the spare lines (lift each eye off its pin and drop it in a crate) and leave both lines that
hold a load on their pins. Lifting a loaded line's eye off its pin would let its load run: score 0 for the run.

Two arms: each pin and each crate is reachable only by the arm on its side (offline IK), so each arm strikes one
spare line. A mirrored or fixed choice (both inner pins, both outer pins, the pin nearest the robot's own side)
always takes one loaded line.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the cables are simulated)
import cableshape  # noqa: E402
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Strike the free lines from the pin rail: lift their eyes off the pins and drop them in the crates. "
               "Leave any line that is holding a load.")

# -- pin rail and pins ---------------------------------------------------------------------------------------------
RAIL_H = 0.040
RAIL_X0, RAIL_X1 = 0.185, 0.265
PIN_R, PIN_H = 0.010, 0.045
PIN_TOP = TOP + RAIL_H + PIN_H
PINS = {"l_out": 0.36, "l_in": 0.24, "r_in": -0.24, "r_out": -0.36}  # pin y; x = PIN_X
LOADED = ("l_out", "r_in")
SPARE = ("l_in", "r_out")

R = 0.004  # 8 mm rope
BAR = 0.010
EYE_W = 0.070  # eye length along the line
EYE_A = 0.090  # eye width across the line
EYE_IN = EYE_W - 2 * BAR  # 50 mm opening along the line
EYE_INA = EYE_A - 2 * BAR  # 70 mm opening across
EYE_C = EYE_W / 2 - BAR / 2  # throat-bar centre to eye centre (30 mm)
EYE_CA = EYE_A / 2 - BAR / 2  # eye centre to side-bar centre (40 mm)
ZL = TOP + RAIL_H + BAR / 2 + 0.0005  # rope / eye-bar centre height on the rail
THROAT_X = 0.255  # throat-bar centre
EYE_X = THROAT_X - EYE_C  # eye centre 0.225
PIN_X = EYE_X - (EYE_IN / 2 - PIN_R - 0.001)  # far bar 1 mm behind the pin: 0.211

RAIL_W = 0.86  # rail, well and lid run y -0.43..0.43
RAIL = Box("pin_rail", pos=((RAIL_X0 + RAIL_X1) / 2, 0.0, TOP + RAIL_H / 2), size=(RAIL_X1 - RAIL_X0, RAIL_W, RAIL_H),
           color=(0.45, 0.30, 0.16))
PIN_OBJS = [Cylinder(f"pin_{k}", pos=(PIN_X, y, TOP + RAIL_H + PIN_H / 2), radius=PIN_R, height=PIN_H,
                     color=(0.06, 0.06, 0.06)) for k, y in PINS.items()]

# -- fly well (opaque) ------------------------------------------------------------------------------------------------
WELL_X0, WELL_X1 = 0.505, 0.665
WELL_H = 0.041
WELL = Box("fly_well", pos=((WELL_X0 + WELL_X1) / 2, 0.0, TOP + WELL_H / 2), size=(WELL_X1 - WELL_X0, RAIL_W, WELL_H),
           color=(0.22, 0.22, 0.24))
LID_Z0 = TOP + 0.062
WELL_LID = Box("well_lid", pos=((WELL_X0 + WELL_X1) / 2 + 0.003, 0.0, LID_Z0 + 0.005),
               size=(WELL_X1 - WELL_X0 - 0.006, RAIL_W, 0.010), color=(0.22, 0.22, 0.24))
WELL_SIDES = [Box(f"well_side_{n}", pos=((WELL_X0 + WELL_X1) / 2, s * (RAIL_W / 2 + 0.005), (TOP + WELL_H + LID_Z0) / 2),
                  size=(WELL_X1 - WELL_X0, 0.010, LID_Z0 - TOP - WELL_H), color=(0.22, 0.22, 0.24))
              for n, s in (("l", 1), ("r", -1))]

# -- crates ---------------------------------------------------------------------------------------------------------
CRATE_X = 0.120
CRATE_Y = {"l": 0.30, "r": -0.30}
CRATE_IN, CRATE_T, CRATE_H = 0.105, 0.006, 0.035
CRATES = []
for _n, _cy in CRATE_Y.items():
    _o = CRATE_IN / 2 + CRATE_T / 2
    _L = CRATE_IN + 2 * CRATE_T
    for _w, (dx, dy, sx, sy) in {"f": (-_o, 0, CRATE_T, _L), "b": (_o, 0, CRATE_T, _L),
                                  "s1": (0, _o, CRATE_IN, CRATE_T), "s2": (0, -_o, CRATE_IN, CRATE_T)}.items():
        CRATES.append(Box(f"crate_{_n}_{_w}", pos=(CRATE_X + dx, _cy + dy, TOP + CRATE_H / 2), size=(sx, sy, CRATE_H),
                          color=(0.10, 0.30, 0.80)))

# -- lines ------------------------------------------------------------------------------------------------------------
ROPE_COL = (0.95, 0.95, 0.92)  # white rope (reads well on the tan table from the top camera)
EYE_COL = (0.55, 0.06, 0.05)
LOAD_ANCHOR_X = 0.60
LOAD_LEN = 0.34  # well end to the eye's throat bar (17 segments)
LOAD_RUN = 0.25  # how far a released loaded line runs into the well
LOAD_OUT = 0.12  # how far a loaded line can be drawn out of the well (the load rises)
LOAD_K, LOAD_D = 4.0, 3.0  # slide spring (N/m, ~1 N pull on the line at rest) and damping (N s/m: a released line runs in at <= 0.35 m/s)


def _eye_plug():
    return Plug(size=(BAR, EYE_A, BAR), mass=0.010, color=EYE_COL)


SLACK_A = 0.045  # sideways swing of the free lines' S on the table


def _slack_path(y0: float) -> np.ndarray:
    """Spare line centreline from its eye (start) to the tail in the well: off the rail edge onto the table in an S,
    up onto the well block, along it under the lid."""
    zt = TOP + R + 0.0005
    xs = np.linspace(0.272, 0.500, 80)
    z = np.full_like(xs, zt)
    d = xs < 0.352
    z[d] = zt + (ZL - zt) * 0.5 * (1 + np.cos(np.pi * (xs[d] - 0.272) / 0.080))
    u = xs > 0.420
    z[u] = zt + (ZL - zt) * 0.5 * (1 - np.cos(np.pi * (xs[u] - 0.420) / 0.080))
    ys = y0 + SLACK_A * np.sin(2 * np.pi * (xs - 0.272) / 0.228)
    pts = [(0.260, y0, ZL), (0.268, y0, ZL)] + list(zip(xs, ys, z)) + [(0.520, y0, ZL), (0.660, y0, ZL)]
    return np.array(pts, dtype=float)


def _path_len(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


SLACK_LEN = 0.46  # 23 segments: S span (~0.34 m) + ~0.12 m tail under the lid

LINES = {}
for _k, _y in PINS.items():
    if _k in LOADED:
        LINES[_k] = cable(f"line_{_k}", start=(LOAD_ANCHOR_X, _y, ZL), yaw=math.pi, length=LOAD_LEN, radius=R,
                          anchor_start=True, end_plug=_eye_plug(), cable_mass=0.020, color=ROPE_COL,
                          bend_stiffness=0.02, bend_damping=0.002)
    else:
        LINES[_k] = cable(f"line_{_k}", start=(0.260, _y, ZL), yaw=0.0, length=SLACK_LEN, radius=R,
                          start_plug=_eye_plug(), cable_mass=0.027, color=ROPE_COL,
                          bend_stiffness=0.003, bend_damping=0.0005)

OBJECTS = [RAIL, *PIN_OBJS, WELL, WELL_LID, *WELL_SIDES, *CRATES, *LINES.values()]

TWIN_EXTRA = [
    {"name": f"eye_{k}", "kind": "body", "size_mm": [70, 90, 10], "colour": "dark red",
     "pose": [EYE_X, y, RAIL_H + BAR / 2],
     "note": "square rope eye of 10 mm bars (50 x 70 mm opening), printed in one piece; the line is spliced to its throat bar"}
    for k, y in PINS.items()
] + [
    {"name": "load_slide", "kind": "joint", "real": "line led over a pulley inside the well to a ~100 g weight",
     "note": "loaded lines l_out and r_in: well end on a prismatic slide with a ~1 N spring (runs up to 25 cm)"},
]

# -- eye colliders (recipe: eye bars on the plug body) -----------------------------------------------------------------


def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_strike_eyes", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        name = getattr(cfg, "cable_name", "")
        if not name.startswith("line_"):
            return prim
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

        stage = get_current_stage()
        key = name[len("line_"):]
        end = "end" if key in LOADED else "start"
        s = 1.0 if end == "end" else -1.0  # the eye lies beyond the throat bar, away from the rope
        plug = f"{prim_path}/{end}_plug"
        for bn, (cx, cy), size in (("eye_side_a", (s * EYE_C, EYE_CA), (EYE_W, BAR, BAR)),
                                   ("eye_side_b", (s * EYE_C, -EYE_CA), (EYE_W, BAR, BAR)),
                                   ("eye_far", (s * 2 * EYE_C, 0.0), (BAR, EYE_A, BAR))):
            g = UsdGeom.Cube.Define(stage, f"{plug}/{bn}")
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(cx, cy, 0.0))
            g.AddScaleOp().Set(Gf.Vec3f(*size))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(g.GetPath()), f"{prim_path}/Looks/cable_physics", stage=stage)
            bind_visual_material(str(g.GetPath()), f"{prim_path}/Looks/{end}_plug", stage=stage)
        if key in LOADED:  # the load: the line's well end runs on a spring-loaded slide instead of a fixed weld
            a = stage.GetPrimAtPath(f"{prim_path}/anchor")
            fj = UsdPhysics.FixedJoint(a)
            p0, r0 = fj.GetLocalPos0Attr().Get(), fj.GetLocalRot0Attr().Get()
            body1 = fj.GetBody1Rel().GetTargets()
            stage.RemovePrim(f"{prim_path}/anchor")
            j = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/load_slide")
            j.CreateBody1Rel().SetTargets(body1)
            j.CreateLocalPos0Attr(p0)
            j.CreateLocalRot0Attr(r0)
            j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            j.CreateAxisAttr("X")  # seg_0 +x points along the line towards the eye; -x runs into the well
            j.CreateLowerLimitAttr(-LOAD_RUN)
            j.CreateUpperLimitAttr(LOAD_OUT)
            d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
            d.CreateTypeAttr("force")
            d.CreateTargetPositionAttr(-LOAD_RUN)
            d.CreateStiffnessAttr(LOAD_K)
            d.CreateDampingAttr(LOAD_D)
            j.CreateExcludeFromArticulationAttr(True)
        return prim

    spawn._strike_eyes = True
    cablekit._spawn_cable = spawn


_install_spawn()


def _shape(emb):
    for k in SPARE:
        cableshape.preshape(emb, LINES[k], _slack_path(PINS[k]), hold="ramp", settle_s=0)
    cableshape.settle(emb, 2.0)
    _reset_episode()


cableshape.on_reset(_shape)

# -- env handle (gripper release check) --------------------------------------------------------------------------------
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_strike_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._strike_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- geometry -----------------------------------------------------------------------------------------------------------


def eye_frame(objects, k):
    """(eye centre (3,), u = horizontal unit from the eye centre towards the rope, v = across) of line k."""
    c = objects[f"line_{k}"]
    if k in LOADED:
        plug, seg = np.asarray(c.end_plug, float), np.asarray(c.points[-1], float)
    else:
        plug, seg = np.asarray(c.start_plug, float), np.asarray(c.points[0], float)
    d = seg - plug
    d = d / max(np.linalg.norm(d), 1e-9)
    ctr = plug - d * EYE_C
    u = np.array([d[0], d[1], 0.0])
    u /= max(np.linalg.norm(u), 1e-9)
    return ctr, u, np.array([-u[1], u[0], 0.0]), plug


def eye_points(objects, k) -> np.ndarray:
    ctr, u, v, _ = eye_frame(objects, k)
    t, ta = np.linspace(-EYE_C, EYE_C, 7), np.linspace(-EYE_CA, EYE_CA, 9)
    pts = [ctr + u * EYE_C + v * s for s in ta] + [ctr - u * EYE_C + v * s for s in ta]
    pts += [ctr + v * EYE_CA + u * s for s in t] + [ctr - v * EYE_CA + u * s for s in t]
    return np.array(pts)


def on_pin(objects, k) -> bool:
    """Pin axis passes through the eye's opening (top view) and the eye's underside is below the pin top."""
    ctr, u, v, _ = eye_frame(objects, k)
    d = np.array([PIN_X, PINS[k], 0.0]) - np.array([ctr[0], ctr[1], 0.0])
    return bool(abs(d @ u) < EYE_IN / 2 and abs(d @ v) < EYE_INA / 2 and ctr[2] - BAR / 2 < PIN_TOP)


def in_crate(objects, k) -> bool:
    """Eye centre within the crate's inner square (top view) and below the crate rim."""
    ctr, _, _, _ = eye_frame(objects, k)
    cy = CRATE_Y["l" if PINS[k] > 0 else "r"]
    return bool(abs(ctr[0] - CRATE_X) < CRATE_IN / 2 and abs(ctr[1] - cy) < CRATE_IN / 2
                and ctr[2] < TOP + CRATE_H)


EYE0 = {k: np.array([EYE_X, y, ZL]) for k, y in PINS.items()}
LOAD_MOVE = 0.02  # loaded eye moved more than this from its start = its line was taken off / its load let run
RELEASE_DIST = 0.035

_STATE = {"violated": False, "why": "", "off": {}}
HELD_DIST = 0.045  # a gripper TCP this close to an eye's bars = the eye is in hand


def _reset_episode():
    _STATE["violated"] = False
    _STATE["why"] = ""
    _STATE["off"] = {}


def _finite(objects) -> bool:
    return all(np.isfinite(objects[f"line_{k}"].points).all() for k in PINS)


def _track(objects) -> None:
    if not _finite(objects):
        _STATE["violated"], _STATE["why"] = True, "nan"
        return
    for k in LOADED:
        ctr, _, _, _ = eye_frame(objects, k)
        if float(np.linalg.norm(ctr - EYE0[k])) > LOAD_MOVE or not on_pin(objects, k):
            if not _STATE["violated"]:
                _STATE["why"] = f"loaded {k} moved"
            _STATE["violated"] = True
    g = grippers()
    for k in SPARE:  # a free eye must leave its pin in hand, not be dragged or flipped off by its rope
        if on_pin(objects, k):
            _STATE["off"].pop(k, None)
            continue
        if k in _STATE["off"]:
            continue
        pts = eye_points(objects, k)
        held = (not g) or any(float(np.linalg.norm(pts - x, axis=1).min()) < HELD_DIST for x in g)
        _STATE["off"][k] = held
        if not held:
            if not _STATE["violated"]:
                _STATE["why"] = f"free {k} pulled off its pin"
            _STATE["violated"] = True


def released(objects) -> bool:
    pts = np.vstack([eye_points(objects, k) for k in PINS])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def score(objects) -> float:
    """0 if a loaded line's eye ever moved more than 2 cm or left its pin, or a free eye left its pin with no gripper
    TCP within 4.5 cm of its bars (pulled off by its rope; both latched for the run), or a gripper TCP is
    within 3.5 cm of any eye at the end. Then: both spare eyes off their pins: 0.5, plus 0.25 for each spare eye in
    a crate (1.0 = both). Fewer than both spare eyes off their pins: 0."""
    _track(objects)
    if _STATE["violated"] or not released(objects):
        return 0.0
    if any(on_pin(objects, k) for k in SPARE):
        return 0.0
    return 0.5 + 0.25 * sum(in_crate(objects, k) for k in SPARE)


def success(objects) -> bool:
    return score(objects) >= 1.0 and all(objects[f"line_{k}"].resting(max_speed=0.02) for k in PINS)


def distance(objects) -> float:
    """Summed distance (m, top view) of the spare eyes from their crate centres; 1.0 once the loaded latch fired."""
    _track(objects)
    if _STATE["violated"]:
        return 1.0
    tot = 0.0
    for k in SPARE:
        ctr, _, _, _ = eye_frame(objects, k)
        tot += float(np.hypot(ctr[0] - CRATE_X, ctr[1] - CRATE_Y["l" if PINS[k] > 0 else "r"]))
    return tot


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    _track(objects)
    return {
        "loaded_kept": int(not _STATE["violated"]),
        "spare_off_pin": sum(int(not on_pin(objects, k)) for k in SPARE) / 2,
        "spare_in_crate": sum(int(in_crate(objects, k)) for k in SPARE) / 2,
        "released": int(released(objects)),
    }


# Seeded reset variation for a later runner: the whole rail moves with its pins and eyes; the closed-loop demo reads
# the live eyes. Crates may shift a little.
VARIATION = {
    "group:rail": {"dx": 0.01, "dy": 0.01, "yaw": 0.0,
                   "with": ["pin_rail", *[p.name for p in PIN_OBJS], *[f"line_{k}" for k in PINS]]},
    "fixed:crates": {"dx": 0.01, "dy": 0.01, "with": [c.name for c in CRATES]},
}

# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------------
GRIP_OPEN = -0.014  # pads ~28 mm apart round the 10 mm throat bar
GRIP_DZ = 0.008  # TCP above the throat-bar centre when pinching
PINCH_OFF = 0.020  # pinch the throat bar this far to the side of the rope


def demo_script(ctx):
    """Each arm strikes the spare line on its side: it pinches that eye's throat bar beside the rope (fingers closing
    along x, wrist camera looking along the rail), lifts the eye straight up off the pin, carries it over its crate
    (closed loop on the live eye centre), lowers it and lets go, then moves clear. The taut (loaded) lines are left
    alone. STRIKE_CHEAT selects a test variant (see README)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("STRIKE_CHEAT", "")

    def log(tag):
        st = ctx.states()
        on = {k: int(on_pin(st, k)) for k in PINS}
        cr = {k: int(in_crate(st, k)) for k in SPARE}
        print(f"[demo t={ctx.t:5.1f}] {tag}: on_pin={on} crate={cr} viol={_STATE['violated']}:{_STATE['why']} "
              f"rel={released(st)} score={score(st):.2f}", flush=True)

    def strike(arm, k, dest="crate", sign=1.0, let_go=True, pinch=None):
        """Pinch eye k, lift it off its pin, carry it to dest ('crate' | 'table' | 'drop'), let go, move clear."""
        yaw = sign * math.pi / 2
        _, u, v, plug = eye_frame(ctx.states(), k)
        side = 1.0 if PINS[k] > 0 else -1.0
        off = -side * PINCH_OFF if abs(PINS[k]) > 0.24 else side * PINCH_OFF  # outer eyes: towards the middle
        if pinch is not None:
            off = pinch
        g = plug + np.array([0.0, off, 0.0])
        hi = TOP + 0.13
        yield from arm.move((g[0], g[1], TOP + 0.10), yaw=yaw, finger=GRIP_OPEN, duration=1.6)
        yield from arm.move((g[0], g[1], plug[2] + GRIP_DZ), yaw=yaw, finger=GRIP_OPEN, duration=0.9)
        if os.environ.get("STRIKE_DEBUG"):
            print(f"[dbg {k}] down tcp={np.round(arm.tcp(), 4)}", flush=True)
        yield from arm.grip(FINGER_CLOSED)
        yield from arm.wait(0.3)
        if os.environ.get("STRIKE_DEBUG"):
            print(f"[dbg {k}] grip tcp={np.round(arm.tcp(), 4)} g={np.round(g, 4)} plug={np.round(plug, 4)}", flush=True)
        yield from arm.move((g[0], g[1], hi), yaw=yaw, duration=1.0)
        if os.environ.get("STRIKE_DEBUG"):
            _, _, _, p2 = eye_frame(ctx.states(), k)
            print(f"[dbg {k}] lifted tcp={np.round(arm.tcp(), 4)} plug={np.round(p2, 4)}", flush=True)
        if dest == "drop":  # lift it off and let it fall back by the pin
            yield from arm.grip(FINGER_OPEN)
            yield from arm.move((g[0] - 0.03, g[1], hi + 0.03), yaw=yaw, duration=0.6)
            return
        cy = CRATE_Y["l" if side > 0 else "r"]
        tgt = np.array([CRATE_X, cy]) if dest == "crate" else np.array([0.33, PINS[k] - side * 0.035])
        tcp = np.array([g[0], g[1]])
        for it in range(3):  # closed loop: put the live eye centre over the target
            ctr, _, _, _ = eye_frame(ctx.states(), k)
            tcp = tcp + (tgt - ctr[:2])
            yield from arm.move((tcp[0], tcp[1], hi), yaw=yaw, duration=1.4 if it == 0 else 0.6)
        low = TOP + CRATE_H + 0.035 if dest == "crate" else TOP + 0.030
        yield from arm.move((tcp[0], tcp[1], low), yaw=yaw, duration=0.8)
        if not let_go:  # hold-at-end shortcut: eye in the crate, still pinched
            return
        yield from arm.grip(FINGER_OPEN)
        yield from arm.wait(0.3)
        yield from arm.move((tcp[0], tcp[1], TOP + 0.15), yaw=yaw, duration=0.7)

    def park(arm, side):
        yield from arm.move((0.30, side * 0.20, TOP + 0.20), yaw=0.0, finger=FINGER_OPEN, duration=1.0)

    log("start")
    if cheat == "none":
        yield from L.wait(4.0)
        log("end")
        return
    if cheat == "inner":  # nearest pins to the middle on both sides
        yield from ctx.both(strike(L, "l_in"), strike(Rt, "r_in", sign=-1.0))
    elif cheat == "outer":
        yield from ctx.both(strike(L, "l_out"), strike(Rt, "r_out", sign=-1.0))
    elif cheat == "all":
        yield from ctx.both(strike(L, "l_out", dest="drop"), strike(Rt, "r_in", dest="drop", sign=-1.0))
        log("loaded lines lifted")
        yield from ctx.both(strike(L, "l_in"), strike(Rt, "r_out", sign=-1.0))
    elif cheat == "onearm":
        yield from strike(L, "l_in")
    elif cheat == "cross":  # right arm alone: its own free line, then reach across for the left free eye's inner end
        yield from strike(Rt, "r_out", sign=-1.0)
        yield from strike(Rt, "l_in", sign=-1.0, pinch=-(EYE_A / 2 - 0.008))
    elif cheat == "ropelift":  # right arm alone: own free eye to its crate, then lift/pull the left free rope
        yield from strike(Rt, "r_out", sign=-1.0)
        st = ctx.states()
        pts = np.asarray(st["line_l_in"].points, float)
        ok = (pts[:, 0] >= 0.20) & (pts[:, 0] <= 0.50)
        cand = pts[ok]
        i = int(np.argmin(np.hypot(cand[:, 0] - 0.0, cand[:, 1] + 0.305)))
        p = cand[i]
        j = int(np.where(ok)[0][i])
        d = pts[min(j + 1, len(pts) - 1)] - pts[max(j - 1, 0)]
        yw = (math.atan2(d[1], d[0]) + math.pi / 2) % math.pi - math.pi / 2
        print(f"[ropelift] grab rope at {np.round(p, 3)} yaw={yw:.2f}", flush=True)
        yield from Rt.move((p[0], p[1], TOP + 0.08), yaw=yw, finger=-0.014, duration=1.5)
        yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=yw, finger=-0.014, duration=1.0)
        print(f"[ropelift] tcp at grab {np.round(Rt.tcp(), 3)}", flush=True)
        yield from Rt.grip(FINGER_CLOSED)
        yield from Rt.move((p[0], p[1], TOP + 0.14), yaw=yw, duration=1.2)
        yield from Rt.move((0.22, -0.05, TOP + 0.14), yaw=yw, duration=1.5)
        yield from Rt.move((0.15, -0.10, TOP + 0.10), yaw=yw, duration=1.5)
        log("rope pulled")
        yield from Rt.grip(FINGER_OPEN)
        yield from Rt.move((0.15, -0.10, TOP + 0.18), yaw=yw, duration=0.8)
    elif cheat == "tier50":
        yield from ctx.both(strike(L, "l_in", dest="table"), strike(Rt, "r_out", dest="table", sign=-1.0))
    elif cheat == "hold":  # eyes into the crates but never let go
        yield from ctx.both(strike(L, "l_in", let_go=False), strike(Rt, "r_out", sign=-1.0, let_go=False))
        yield from L.wait(3.0)
        log("end")
        return
    elif cheat == "tier75":
        yield from ctx.both(strike(L, "l_in"), strike(Rt, "r_out", dest="table", sign=-1.0))
    else:
        yield from ctx.both(strike(L, "l_in"), strike(Rt, "r_out", sign=-1.0))
    log("struck")
    yield from ctx.both(park(L, 1.0), park(Rt, -1.0))
    for _ in range(4):
        yield from L.wait(0.5)
    log("end")
