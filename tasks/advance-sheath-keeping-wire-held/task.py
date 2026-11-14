"""A training phantom for the Seldinger technique lies on the station table. A skin-coloured block (20 x 6 x 3 cm) has
a 9 mm bore whose mouth (the port, with a dark grey collar) faces the robot's right front. A black 6 mm training wire
runs 8 cm into the bore and 40 cm out of it along one straight line (heading 70 deg); a white band on the wire sits
just outside the collar. A free blue sheath (16 cm sleeve, 14 x 12 mm outside, 9 x 7 mm bore) lies around the wire
with its nose 26 cm from the port and the wire's tail 2 cm inside its rear hub; a raised grip collar (32 mm wide,
20 mm tall) sits 2 cm behind its nose. A weighted valve pad in the
hub rests on the wire, so the sheath drags the wire with it unless the wire is held.

The goal: slide the sheath along the wire until it seats against the collar, then draw the wire out through the
sheath, without the wire moving at the port before the sheath is seated. One arm must hold the wire while the other
pushes the sheath; the holds swap when the tail comes out of the sheath (the left arm cannot reach the tail, the
right arm cannot reach the port).

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

INSTRUCTION = ("On this training phantom, slide the blue sheath along the wire until it seats against the port, "
               "then draw the wire out through the sheath. Let go of everything at the end.")

# -- layout: everything lies on one line through the port at heading 70 deg ------------------------------------------
# s = distance (m) from the port along the line towards the robot's right front; s < 0 is inside the phantom.
HEAD = math.radians(70.0)
D = np.array([math.cos(HEAD), math.sin(HEAD)])  # unit vector pointing INTO the phantom
NRM = np.array([-D[1], D[0]])
PORT = np.array([0.33, 0.14])  # the phantom's face (port mouth), table coordinates


def at(s: float, v: float = 0.0) -> np.ndarray:
    """Table xy of the point s along the line (s > 0 outside the phantom) and v to its side."""
    return PORT - s * D + v * NRM


SKIN = (0.93, 0.74, 0.62)
COLLAR_C = (0.30, 0.31, 0.33)
BLUE = (0.10, 0.35, 0.85)
BAND_COLOR = (0.97, 0.97, 0.97)

# phantom: fixed block from s 0 to s -0.20, bore 9 mm wide and 9 mm tall above a 1.5 mm floor
PH_L, PH_W, PH_H = 0.20, 0.06, 0.03
BORE_W, FLOOR_T, BORE_TOP = 0.009, 0.0015, 0.0105


def _pbox(name, s0, s1, v0, v1, z0, z1, color=SKIN):
    c = at((s0 + s1) / 2, (v0 + v1) / 2)
    return Box(name, pos=(float(c[0]), float(c[1]), TOP + (z0 + z1) / 2), size=(abs(s1 - s0), v1 - v0, z1 - z0),
               yaw=HEAD, color=color)


PHANTOM = [
    _pbox("phantom_floor", 0.0, -PH_L, -BORE_W / 2, BORE_W / 2, 0.0, FLOOR_T),
    _pbox("phantom_side_a", 0.0, -PH_L, BORE_W / 2, PH_W / 2, 0.0, PH_H),
    _pbox("phantom_side_b", 0.0, -PH_L, -PH_W / 2, -BORE_W / 2, 0.0, PH_H),
    _pbox("phantom_roof", 0.0, -PH_L, -BORE_W / 2, BORE_W / 2, BORE_TOP, PH_H),
    _pbox("phantom_end", -PH_L + 0.01, -PH_L, -BORE_W / 2, BORE_W / 2, FLOOR_T, BORE_TOP),
]
COLLAR_T = 0.004  # collar ring stands 4 mm proud of the phantom face; the sheath seats against it
COLLAR = [
    _pbox("collar_a", COLLAR_T, 0.0, BORE_W / 2, 0.018, 0.0, 0.022, COLLAR_C),
    _pbox("collar_b", COLLAR_T, 0.0, -0.018, -BORE_W / 2, 0.0, 0.022, COLLAR_C),
    _pbox("collar_top", COLLAR_T, 0.0, -BORE_W / 2, BORE_W / 2, BORE_TOP, 0.022, COLLAR_C),
]
SEAT_S = COLLAR_T  # s of the collar face

# training wire: 6 mm black cord, tip 8 cm inside the bore, tail 40 cm outside
R = 0.003
WIRE_Z = TOP + FLOOR_T + R + 0.0005
TIP_S, TAIL_S = -0.08, 0.40
WIRE_LEN = TAIL_S - TIP_S
WIRE_MU = float(os.environ.get("SHEATH_WIRE_MU", "0.5"))  # slippery coated wire; "min" friction combine (spawn hook)
_tip = at(TIP_S)
WIRE = cable("wire", start=(float(_tip[0]), float(_tip[1]), WIRE_Z), yaw=HEAD + math.pi, length=WIRE_LEN, radius=R,
             cable_mass=0.020, color=(0.08, 0.08, 0.09), bend_stiffness=float(os.environ.get("SHEATH_WIRE_K", "0.03")), bend_damping=0.003,
             friction=WIRE_MU)
BAND_SEG = int(round((0.0 - TIP_S) / WIRE.seg_len))  # the segment from s 0 to s +2 cm (just outside the collar)
BAND_S0 = TIP_S + (BAND_SEG + 0.5) * WIRE.seg_len  # its centre at the start

# sheath: free rigid sleeve, nose at s 0.26, rear at s 0.42; parts welded to the floor plate ("sheath")
SH_L, SH_W, SH_H = 0.16, 0.014, 0.012
NOSE0 = 0.26
SH_C = NOSE0 + SH_L / 2  # s of the sheath centre
WALL_T = (SH_W - BORE_W) / 2
ROOF_B = 0.0085  # bore roof underside (wire top at 0.008)
HUB_L, HUB_TOP = 0.06, 0.0275  # rear hub: taller cage holding the valve pad
VALVE_L, VALVE_W, VALVE_H, VALVE_M = 0.020, 0.008, 0.010, float(os.environ.get("SHEATH_VALVE_M", "0.010"))
BAR_T = 0.004
SH_SPAWN = 0.0005


def _sbox(name, s0, s1, v0, v1, z0, z1, mass, color=BLUE):
    c = at((s0 + s1) / 2, (v0 + v1) / 2)
    return Box(name, pos=(float(c[0]), float(c[1]), TOP + SH_SPAWN + (z0 + z1) / 2),
               size=(abs(s1 - s0), v1 - v0, z1 - z0), yaw=HEAD, mass=mass, color=color)


REAR0 = NOSE0 + SH_L
HUB0 = REAR0 - HUB_L  # hub front (s)
SHEATH = _sbox("sheath", NOSE0, REAR0, -SH_W / 2, SH_W / 2, 0.0, FLOOR_T, 0.025)
SH_PARTS = [
    _sbox("sheath_wall_a", NOSE0, REAR0, BORE_W / 2, SH_W / 2, FLOOR_T, SH_H, 0.003),
    _sbox("sheath_wall_b", NOSE0, REAR0, -SH_W / 2, -BORE_W / 2, FLOOR_T, SH_H, 0.003),
    _sbox("sheath_roof", NOSE0, HUB0, -BORE_W / 2, BORE_W / 2, ROOF_B, SH_H, 0.003),
    _sbox("hub_wall_a", HUB0, REAR0, BORE_W / 2, SH_W / 2, SH_H, HUB_TOP - 0.003, 0.001),
    _sbox("hub_wall_b", HUB0, REAR0, -SH_W / 2, -BORE_W / 2, SH_H, HUB_TOP - 0.003, 0.001),
    _sbox("hub_cap", HUB0, REAR0, -SH_W / 2, SH_W / 2, HUB_TOP - 0.003, HUB_TOP, 0.002),
    _sbox("hub_bar_f", HUB0, HUB0 + BAR_T, -BORE_W / 2, BORE_W / 2, ROOF_B, HUB_TOP - 0.003, 0.001),
    _sbox("hub_bar_r", HUB0 + BAR_T + VALVE_L + 0.004, HUB0 + 2 * BAR_T + VALVE_L + 0.004, -BORE_W / 2, BORE_W / 2,
          ROOF_B, HUB_TOP - 0.003, 0.001),
    _sbox("grip_collar_a", NOSE0 + 0.020, NOSE0 + 0.028, SH_W / 2, 0.016, 0.0, 0.020, 0.001),
    _sbox("grip_collar_b", NOSE0 + 0.020, NOSE0 + 0.028, -0.016, -SH_W / 2, 0.0, 0.020, 0.001),
    _sbox("grip_collar_top", NOSE0 + 0.020, NOSE0 + 0.028, -SH_W / 2, SH_W / 2, SH_H, 0.020, 0.001),
]
COLLAR_REAR = 0.028  # grip collar's rear face, behind the nose (m)
_v0 = HUB0 + BAR_T + 0.002
VALVE = _sbox("valve_pad", _v0, _v0 + VALVE_L, -VALVE_W / 2, VALVE_W / 2, ROOF_B, ROOF_B + VALVE_H, VALVE_M,
              color=(0.05, 0.20, 0.55))
_SC = np.array([*at(SH_C), TOP + SH_SPAWN + FLOOR_T / 2])
_RI = np.array([[math.cos(HEAD), math.sin(HEAD), 0.0], [-math.sin(HEAD), math.cos(HEAD), 0.0], [0.0, 0.0, 1.0]])
SH_REL = {p.name: _RI @ (np.asarray(p.pos) - _SC) for p in SH_PARTS}  # offsets in the sheath's own frame
# walls, roof and hub are extra colliders on the sheath body itself (one rigid body: a pinch cannot squeeze the bore)

OBJECTS = [*PHANTOM, *COLLAR, SHEATH, VALVE, WIRE]

TWIN_EXTRA = [
    *({"name": p.name, "kind": "body", "size_mm": [round(v * 1000, 1) for v in p.size], "colour": "blue",
       "pose": [round(float(v), 4) for v in (p.pos[0], p.pos[1], p.pos[2] - TOP)], "note": "part of the one-piece sheath"}
      for p in SH_PARTS),
    {"name": "wire_band", "kind": "decal", "size_mm": [20, 6], "colour": "white",
     "pose": [float(at(0.01)[0]), float(at(0.01)[1]), 0.005]},
]

VARIATION = {"cable:wire": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}  # the wire runs through the fixed bore: no variation


# -- spawn hook: weld the sheath parts, paint the band ----------------------------------------------------------------
def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_sheath_task", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != WIRE.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdPhysics

        stage = get_current_stage()
        from pxr import PhysxSchema

        mat = stage.GetPrimAtPath(f"{prim_path}/Looks/cable_physics")
        PhysxSchema.PhysxMaterialAPI.Apply(mat).CreateFrictionCombineModeAttr().Set("min")
        root = str(prim_path).rsplit("/", 1)[0]
        body = f"{root}/{SHEATH.name}"
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from pxr import UsdGeom
        from yam_arena.station import linear_rgb

        look = f"{prim_path}/Looks/sheath_blue"
        vm = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(BLUE), roughness=0.5)
        vm.func(look, vm)
        pmat = f"{prim_path}/Looks/sheath_physics"
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=1.2, dynamic_friction=1.0)
        pm.func(pmat, pm)
        size = {p.name: p.size for p in SH_PARTS}
        if os.environ.get("SHEATH_DEBUG") == "1":
            print(f"[sheath] body ops {[o.GetOpName() for o in UsdGeom.Xformable(stage.GetPrimAtPath(body)).GetOrderedXformOps()]} "
                  f"children {[c.GetName() for c in stage.GetPrimAtPath(body).GetChildren()]}", flush=True)
        for name, rel in SH_REL.items():
            g = UsdGeom.Cube.Define(stage, f"{body}/{name}")
            g.CreateSizeAttr(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, rel)))
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, size[name])))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(g.GetPath()), pmat, stage=stage)
            bind_visual_material(str(g.GetPath()), look, stage=stage)
        try:
            import isaaclab.sim as sim_utils
            from isaaclab.sim.utils import bind_visual_material
            from yam_arena.station import linear_rgb

            look = f"{prim_path}/Looks/band"
            mat = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(BAND_COLOR), roughness=0.5)
            mat.func(look, mat)
            bind_visual_material(f"{prim_path}/seg_{BAND_SEG}/geom", look, stage=stage)
        except Exception as e:  # pragma: no cover - never break the scene for a colour
            print(f"[sheath task] band colour not applied: {e}", flush=True)
        return prim

    spawn._sheath_task = True
    cablekit._spawn_cable = spawn


_install_spawn()

# -- episode memory (latched until the next reset) and env handle -----------------------------------------------------
_STATE = {"env": None, "band_moved": False, "seated_once": False, "max_band": 0.0}


def _reset_episode() -> None:
    _STATE.update(band_moved=False, seated_once=False, max_band=0.0)


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_sheath_env", False):
        base_step = cls._task_step

        def _task_step(self, *a, **k):
            _STATE["env"] = self._env
            return base_step(self, *a, **k)

        _task_step._sheath_env = True
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


def grippers() -> list[np.ndarray]:
    """World TCP positions of both grippers (env 0), [] offline."""
    env = _STATE["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- geometry ---------------------------------------------------------------------------------------------------------
BAND_TOL = 0.015  # the band may not move more than 1.5 cm before the sheath is seated
ADVANCE_MIN = 0.12  # sheath nose advanced at least 12 cm (from s 0.26 to s 0.14 or closer)
SEAT_TOL = 0.005  # nose within 5 mm of the collar face
AXIS_TOL = 0.010  # sheath centre line within 1 cm of the port line
TIP_OUT = 0.03  # wire tip at least 3 cm behind the sheath's rear
RELEASE = 0.035  # both grippers at least 3.5 cm from the wire, the sheath and its valve


def _rot(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = min(200, max(1, int(np.linalg.norm(b - a) / step)))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def s_of(p) -> float:
    return float((PORT - np.asarray(p, dtype=float)[:2]) @ D)


def v_of(p) -> float:
    return float((np.asarray(p, dtype=float)[:2] - PORT) @ NRM)


def _finite(objects) -> bool:
    w, sh = objects["wire"], objects["sheath"]
    return bool(np.isfinite(w.points).all() and np.isfinite(sh.pos).all() and np.isfinite(sh.quat_xyzw).all())


def sheath_ends(objects) -> tuple[np.ndarray, np.ndarray]:
    """(nose, rear) centre points of the sheath, world."""
    sh = objects["sheath"]
    ax = _rot(sh.quat_xyzw) @ np.array([1.0, 0.0, 0.0])  # local +x points into the phantom
    c = np.asarray(sh.pos, dtype=float)
    return c + ax * SH_L / 2, c - ax * SH_L / 2


def nose_s(objects) -> float:
    return s_of(sheath_ends(objects)[0])


def on_axis(objects) -> bool:
    n, r = sheath_ends(objects)
    return bool(max(abs(v_of(n)), abs(v_of(r))) <= AXIS_TOL and max(n[2], r[2]) <= TOP + 0.02)


def seated(objects) -> bool:
    return bool(on_axis(objects) and nose_s(objects) <= SEAT_S + SEAT_TOL)


def band_shift(objects) -> float:
    """How far (m) the band centre is from where it started."""
    b = np.asarray(objects["wire"].points[BAND_SEG], dtype=float)
    b0 = np.array([*at(BAND_S0), WIRE_Z])
    return float(np.linalg.norm(b - b0))


def tip_s(objects) -> float:
    return s_of(objects["wire"].start_tip)


def wire_out(objects) -> bool:
    """The wire's tip is out past the sheath's rear by at least 3 cm."""
    return bool(tip_s(objects) >= s_of(sheath_ends(objects)[1]) + TIP_OUT)


def _sheath_pts(objects) -> np.ndarray:
    n, r = sheath_ends(objects)
    pts = [n + (r - n) * t for t in np.linspace(0.0, 1.0, 17)]
    pts.append(np.asarray(objects["valve_pad"].pos, dtype=float))
    return np.array(pts)


def released(objects) -> bool:
    w = _dense(objects["wire"].polyline())
    sp = _sheath_pts(objects)
    return all(min(float(np.linalg.norm(w - g, axis=1).min()), float(np.linalg.norm(sp - g, axis=1).min())) >= RELEASE
               for g in grippers())


def _track(objects) -> None:
    """Latch, every control step: the band moving more than 1.5 cm at any time before the sheath has first been
    seated. Never cleared until the next trial."""
    if not _finite(objects):
        return
    if not _STATE["seated_once"] and seated(objects):
        _STATE["seated_once"] = True
    if not _STATE["seated_once"]:
        sh = band_shift(objects)
        _STATE["max_band"] = max(_STATE["max_band"], sh)
        if sh > BAND_TOL:
            _STATE["band_moved"] = True


def score(objects) -> float:
    """0 if: before the sheath was first seated, the white band moved more than 1.5 cm from its start at any time; a
    gripper is within 3.5 cm of the wire or the sheath at the end; the sheath is off the port line or its nose
    advanced less than 12 cm. Otherwise 0.3: nose advanced 12 cm or more, not seated; 0.6: seated (nose within 5 mm
    of the collar), wire not yet out; 1.0: seated and the wire tip at least 3 cm out past the sheath's rear."""
    if not _finite(objects):
        return 0.0
    _track(objects)
    if _STATE["band_moved"] or not released(objects) or not on_axis(objects):
        return 0.0
    if NOSE0 - nose_s(objects) < ADVANCE_MIN:
        return 0.0
    if not seated(objects):
        return 0.3
    return 1.0 if wire_out(objects) else 0.6


def success(objects) -> bool:
    return bool(score(objects) >= 1.0 and objects["sheath"].resting(0.01, 0.5) and objects["wire"].resting(0.02))


def distance(objects) -> float:
    if not _finite(objects):
        return 1.0
    _track(objects)
    if _STATE["band_moved"]:
        return 1.0
    return float(max(0.0, nose_s(objects) - SEAT_S) + max(0.0, s_of(sheath_ends(objects)[1]) + TIP_OUT - tip_s(objects)))


def progress(objects) -> dict:
    if not _finite(objects):
        return {"band_kept": 0, "advanced": 0, "seated": 0, "wire_out": 0, "released": 0}
    _track(objects)
    return {"band_kept": int(not _STATE["band_moved"]),
            "advanced": float(min(1.0, max(0.0, (NOSE0 - nose_s(objects)) / (NOSE0 - SEAT_S)))),
            "seated": int(seated(objects)), "wire_out": int(seated(objects) and wire_out(objects)),
            "released": int(released(objects))}


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
YAW = HEAD  # both wrist cameras look along the line towards the port (where the sheath goes)
GRASP_OPEN = -0.014  # pads ~28 mm apart
WIRE_GRIP_DZ = 0.010
SH_GRIP_Z = TOP + SH_H / 2 + 0.010
HI = TOP + 0.07


def _log(ctx, tag=""):
    st = ctx.states()
    if not _finite(st):
        print(f"[t={ctx.t:5.2f}] {tag}: NON-FINITE STATE", flush=True)
        return
    _track(st)
    print(f"[t={ctx.t:5.2f}] {tag}: nose_s={nose_s(st):.3f} band={band_shift(st) * 1000:.1f}mm "
          f"(max {_STATE['max_band'] * 1000:.1f}) tip_s={tip_s(st):.3f} rear_s={s_of(sheath_ends(st)[1]):.3f} "
          f"axis={on_axis(st)} seated={seated(st)} valve_z={st['valve_pad'].pos[2] - TOP:.4f} "
          f"buckle={buckle(st) * 1000:.1f}mm rel={released(st)} score={score(st):.2f}", flush=True)


def buckle(st) -> float:
    """Largest sideways offset (m) of the wire from the port line between the port and the sheath's nose."""
    ns = nose_s(st)
    v = [abs(v_of(q)) for q in st["wire"].points if 0.0 < s_of(q) < ns]
    return max(v) if v else 0.0


PUSH_Z = TOP + 0.029  # closed fingertips 4 mm above the sheath roof, below the grip collar's top


def _shoulder_push(arm, arm_i, ctx, goal_nose_s, step=0.025, dur=1.2, tries=24):
    """Closed fingers set down just behind the sheath's grip collar and push it along the line (no pinch)."""
    from yam_arena.constants import FINGER_CLOSED

    st = ctx.states()
    g = at(nose_s(st) + COLLAR_REAR + 0.016)
    yield from arm.move((g[0], g[1], HI), yaw=YAW, finger=FINGER_CLOSED, duration=1.5)
    yield from arm.move((g[0], g[1], PUSH_Z), yaw=YAW, duration=1.0)
    _log(ctx, "fingers behind grip collar")
    last = None
    for k in range(tries):
        st = ctx.states()
        ns = nose_s(st)
        if ns <= goal_nose_s + 0.002 or (last is not None and abs(last - ns) < 0.001 and ns <= goal_nose_s + 0.008):
            break
        last = ns
        # keep the fingers on the collar's rear face, centred on the line, and advance by the remaining gap
        tgt = min(ns + COLLAR_REAR + 0.008, s_of(_tcp(arm_i))) - min(step, ns - goal_nose_s + 0.001)
        g = at(tgt)
        yield from arm.move((g[0], g[1], PUSH_Z), yaw=YAW, duration=dur)
        if k % 3 == 2:
            _log(ctx, f"shoulder push {k}")


def _relax(arm, arm_i, ctx, max_back=0.03):
    """Back the pinched sheath off until the wire in front of it lies straight again."""
    for _ in range(6):
        st = ctx.states()
        if buckle(st) < 0.004:
            break
        t = _tcp(arm_i)
        g = at(s_of(t) + 0.005, 0.0)
        max_back -= 0.005
        if max_back < 0:
            break
        yield from arm.move((g[0], g[1], SH_GRIP_Z), yaw=YAW, duration=0.5)
    _log(ctx, "relaxed")


def _wire_at(st, s):
    """Live wire point nearest to the line coordinate s (by projection), and its height."""
    p = st["wire"].points
    i = int(np.argmin([abs(s_of(q) - s) for q in p]))
    return np.asarray(p[i], dtype=float)


def _pinch_wire(arm, ctx, s):
    from yam_arena.constants import FINGER_CLOSED

    p = _wire_at(ctx.states(), s)
    yield from arm.move((p[0], p[1], HI), yaw=YAW, finger=GRASP_OPEN, duration=1.5)
    p = _wire_at(ctx.states(), s)
    yield from arm.move((p[0], p[1], p[2] + WIRE_GRIP_DZ), yaw=YAW, duration=1.0)
    yield from arm.grip(FINGER_CLOSED, 0.6)


def _pinch_sheath(arm, ctx, from_nose):
    """Pinch the sheath across its walls `from_nose` metres behind its nose."""
    from yam_arena.constants import FINGER_CLOSED

    st = ctx.states()
    n, r = sheath_ends(st)
    p = n + (r - n) * (from_nose / SH_L)
    yield from arm.move((p[0], p[1], HI), yaw=YAW, finger=GRASP_OPEN, duration=1.5)
    yield from arm.move((p[0], p[1], SH_GRIP_Z), yaw=YAW, duration=1.0)
    yield from arm.grip(FINGER_CLOSED, 0.6)


def _tcp(arm_i):
    g = grippers()
    return g[arm_i] if g else None


def _push(arm, arm_i, ctx, goal_nose_s, step=0.03, dur=1.2, tries=24):
    """Slide the pinched sheath along the line until its nose reaches goal_nose_s, closed loop on the live nose and
    the live TCP (the sheath slips a few mm in the pinch under the valve drag)."""
    last = None
    for k in range(tries):
        st = ctx.states()
        ns = nose_s(st)
        if ns <= goal_nose_s + 0.002 or (last is not None and abs(last - ns) < 0.001 and ns <= goal_nose_s + 0.008):
            break
        last = ns
        ds = min(step, ns - goal_nose_s + 0.001)
        t = _tcp(arm_i)
        g = at(s_of(t) - ds, 0.0)
        yield from arm.move((g[0], g[1], SH_GRIP_Z), yaw=YAW, duration=dur * ds / step + 0.3)
        if k % 3 == 2:
            _log(ctx, f"push {k}")


def _pull(arm, arm_i, ctx, until, goal_s, z=TOP + 0.02, step=0.04, dur=1.2):
    """Draw the pinched wire away from the port along the line, step by step, until `until(states)` holds or the
    TCP reaches goal_s."""
    for k in range(20):
        st = ctx.states()
        if until(st):
            break
        t = _tcp(arm_i)
        s1 = min(goal_s, s_of(t) + step)
        if s1 <= s_of(t) + 0.002:
            break
        g = at(s1, 0.0)
        yield from arm.move((g[0], g[1], z), yaw=YAW, duration=dur)
        if k % 2 == 1:
            _log(ctx, f"pull {k}")


def _let_go(arm, arm_i, ctx):
    yield from arm.grip(GRASP_OPEN, 0.5)
    t = _tcp(arm_i)
    yield from arm.move((t[0], t[1], HI), yaw=YAW, duration=0.8)


def _clear(ctx):
    yield from ctx.both(ctx.left.move((0.20, 0.25, TOP + 0.18), yaw=0.0, duration=1.2),
                        ctx.right.move((0.20, -0.25, TOP + 0.18), yaw=0.0, duration=1.2))
    for _ in range(6):
        yield from ctx.left.wait(0.5)
    _log(ctx, "end")


def demo_script(ctx):
    """SHEATH_DEMO=correct (default). Variants: nothing | hold (correct, but the left arm keeps pinching the seated
    sheath) | pushfree (the right arm pushes the sheath as far as it reaches, wire never held) | pullfirst (left arm
    draws the wire 6 cm out of the port first, then the correct sequence) | onearm (right arm only: push 4 cm, let go,
    pinch the wire behind the sheath, pull it back, repeat) | tier03 (stop after the first push, let go) | tier06
    (stop once seated, let go). Feasibility gates: gate_held | gate_free."""
    mode = os.environ.get("SHEATH_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    yield from L.wait(1.0)
    _log(ctx, "settled")
    if mode == "nothing":
        yield from L.wait(5.0)
        _log(ctx, "end")
        return
    if mode in ("gate_held", "gate_free"):
        if mode == "gate_held":
            yield from ctx.both(_pinch_wire(L, ctx, 0.05), _pinch_sheath(Rt, ctx, 0.07))
        else:
            yield from _pinch_sheath(Rt, ctx, 0.07)
        yield from _push(Rt, 1, ctx, NOSE0 - 0.10)
        _log(ctx, "pushed 10")
        yield from Rt.wait(1.0)
        _log(ctx, "end")
        return
    if mode == "pushfree":
        yield from _pinch_sheath(Rt, ctx, 0.07)
        yield from _push(Rt, 1, ctx, 0.03)
        _log(ctx, "pushed")
        yield from _let_go(Rt, 1, ctx)
        yield from _clear(ctx)
        return
    if mode == "onearm":
        for rep in range(3):
            yield from _pinch_sheath(Rt, ctx, 0.07)
            yield from _push(Rt, 1, ctx, nose_s(ctx.states()) - 0.04)
            yield from _let_go(Rt, 1, ctx)
            rear = s_of(sheath_ends(ctx.states())[1])
            yield from _pinch_wire(Rt, ctx, min(rear + 0.02, s_of(ctx.states()["wire"].end_tip) - 0.01))
            _log(ctx, f"onearm wire pinched {rep}")
            yield from _pull(Rt, 1, ctx, lambda st: band_shift(st) < 0.004, s_of(_tcp(1)) + 0.06, step=0.01)
            yield from _let_go(Rt, 1, ctx)
            _log(ctx, f"onearm rep {rep}")
        yield from _clear(ctx)
        return
    if mode == "pullfirst":
        yield from _pinch_wire(L, ctx, 0.05)
        yield from _pull(L, 0, ctx, lambda st: False, 0.11, step=0.03)
        _log(ctx, "wire drawn first")
        yield from _let_go(L, 0, ctx)
    # 1) left arm holds the wire just outside the port; right arm pinches the sheath 7 cm behind its nose
    yield from ctx.both(_pinch_wire(L, ctx, 0.05), _pinch_sheath(Rt, ctx, 0.07))
    _log(ctx, "gripped")
    # 2) right arm slides the sheath 13 cm along the held wire
    yield from _push(Rt, 1, ctx, NOSE0 - 0.135)
    _log(ctx, "first push done")
    yield from _relax(Rt, 1, ctx)
    # 3) right arm lets go of the sheath and takes the wire's tail, now out of the sheath's rear
    yield from _let_go(Rt, 1, ctx)
    _log(ctx, "right let go of sheath")
    tail = s_of(ctx.states()["wire"].end_tip)
    yield from _pinch_wire(Rt, ctx, tail - 0.06)
    _log(ctx, "tail held")
    if mode == "tier03":
        yield from _let_go(L, 0, ctx)
        yield from _let_go(Rt, 1, ctx)
        yield from _clear(ctx)
        return
    # 4) left arm lets go of the wire, pinches the sheath 3 cm behind its nose and seats it against the collar
    yield from _let_go(L, 0, ctx)
    yield from _shoulder_push(L, 0, ctx, SEAT_S - 0.001)
    _log(ctx, "seated")
    if mode == "tier06":
        yield from _let_go(Rt, 1, ctx)
        yield from _let_go(L, 0, ctx)
        yield from _clear(ctx)
        return
    # 5) left arm keeps the sheath seated; right arm draws the wire out through it, re-pinching once behind the rear
    yield from _pull(Rt, 1, ctx, lambda st: False, s_of(_tcp(1)) + 0.20)
    _log(ctx, "first pull")
    yield from _let_go(Rt, 1, ctx)
    rear = s_of(sheath_ends(ctx.states())[1])
    yield from _pinch_wire(Rt, ctx, rear + 0.045)
    yield from _pull(Rt, 1, ctx, lambda st: tip_s(st) >= s_of(sheath_ends(st)[1]) + TIP_OUT + 0.02, 0.60)
    _log(ctx, "wire out")
    if mode == "hold":
        yield from _let_go(Rt, 1, ctx)
        yield from Rt.move((0.20, -0.25, TOP + 0.18), yaw=0.0, duration=1.2)
        yield from L.wait(3.0)
        _log(ctx, "end")
        return
    yield from _let_go(Rt, 1, ctx)
    yield from _let_go(L, 0, ctx)
    yield from _clear(ctx)
