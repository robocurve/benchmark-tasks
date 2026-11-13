"""Simulator-side helpers for the scribing task (imported by the task file; no simulator imports at module import).

1. Layout by seed: at every reset the trial's layout is seed mod 4. Right after the environment's own reset puts
   every object back, the shelf's root pose is written for that layout (the same calls the simulator uses for its
   seeded object jitter); the simulator's settle steps then run as usual. No fixed body moves.
2. Pencil line: a pre-authored grid of invisible graphite dots on the shelf's top face near its end. Each control
   step, dots within one dot radius of the lead's path are made visible while the lead touches the face. The
   graphics only change visibility; the scorer never reads them (it keeps its own record of the lead's path).
3. The wall's routed face gets a matte white paint look (the mesh kit's default look is metallic).
4. The compass's knurled top grip (its collider 0) gets its own friction, GRIP_FRICTION; the kit gives a part one
   material for all its colliders, and the rest of the compass keeps the brass-plated steel values.
"""
import functools
import importlib.abc
import importlib.machinery
import sys
import time

import numpy as np

_CFG = None
_ROOTS = {}
_TOKEN = object()
_INK = {"token": None, "prev": None, "prev_step": None}

DOT_PITCH = 0.0005
DOT_R = 0.00035
STRIP = (0.0005, 0.0170)     # ink strip: this far from the shelf's end (metres)
INK_LIFT = 0.00012           # dots float this far above the top face


def dot_grid(cfg):
    """Shelf-local (x, y) dot centres covering the strip next to the shelf's end."""
    xs = np.arange(-cfg["SHELF_D"] / 2 + DOT_PITCH / 2, cfg["SHELF_D"] / 2, DOT_PITCH)
    ys = cfg["SHELF_L"] / 2 - np.arange(STRIP[0], STRIP[1] + 1e-9, DOT_PITCH)
    x, y = np.meshgrid(xs, ys)
    return np.column_stack((x.ravel(), y.ravel()))


def near_segment(points, a, b, radius):
    d = b - a
    u = np.clip((points - a) @ d / max(float(d @ d), 1e-30), 0, 1)
    return np.linalg.norm(points - (a + u[:, None] * d), axis=1) <= radius


def _material(stage, path, color, roughness=0.9):
    from pxr import Gf, Sdf, UsdShade
    from yam_arena.station import linear_rgb
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr().Set("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(color)))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def spawn_ink(stage, root):
    from pxr import Gf, UsdGeom, UsdShade, Vt
    start = time.perf_counter()
    parent = root + "/Ink"
    UsdGeom.Xform.Define(stage, parent)
    mat = _material(stage, parent + "/Material", (0.16, 0.16, 0.18))
    grid = dot_grid(_CFG)
    inst = UsdGeom.PointInstancer.Define(stage, parent + "/Dots")
    prototype = parent + "/Dots/Prototype"
    disc = UsdGeom.Cylinder.Define(stage, prototype)
    disc.CreateRadiusAttr().Set(DOT_R)
    disc.CreateHeightAttr().Set(0.0001)
    disc.CreateAxisAttr().Set("Z")
    UsdShade.MaterialBindingAPI.Apply(disc.GetPrim()).Bind(mat)
    inst.CreatePrototypesRel().SetTargets([prototype])
    inst.CreateProtoIndicesAttr().Set(Vt.IntArray([0] * len(grid)))
    inst.CreateIdsAttr().Set(Vt.Int64Array(list(range(len(grid)))))
    z = float(_CFG["SHELF_T"] + INK_LIFT)
    inst.CreatePositionsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(float(x), float(y), z) for x, y in grid]))
    inst.CreateInvisibleIdsAttr().Set(Vt.Int64Array(list(range(len(grid)))))
    _ROOTS[root] = dict(grid=grid, visible=np.zeros(len(grid), dtype=bool), inst=inst)
    print(f"SCRIBE_INK_SPAWN dots={len(grid)} seconds={time.perf_counter() - start:.4f}", flush=True)


def matte_wall(stage, root):
    from pxr import UsdShade
    prim = stage.GetPrimAtPath(root + "/mesh_face")
    if prim and prim.IsValid():
        mat = _material(stage, root + "/matte_paint", _CFG["WALL_WHITE"], roughness=0.85)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


def knurled_grip(stage, root):
    import isaaclab.sim as sim_utils
    static, dynamic = _CFG["GRIP_FRICTION"]
    path = root + "/grip_material"
    cfg = sim_utils.RigidBodyMaterialCfg(static_friction=static, dynamic_friction=dynamic)
    cfg.func(path, cfg)
    sim_utils.bind_physics_material(root + "/shape_0", path, stage=stage)
    print(f"SCRIBE_GRIP_MATERIAL {root}/shape_0 static={static} dynamic={dynamic}", flush=True)


def _set_visible(record, changed, value):
    if not len(changed):
        return
    from pxr import Vt
    record["visible"][changed] = value
    record["inst"].GetInvisibleIdsAttr().Set(Vt.Int64Array(np.flatnonzero(~record["visible"]).tolist()))


def reset_ink():
    for record in _ROOTS.values():
        _set_visible(record, np.flatnonzero(record["visible"]), False)
    _INK.update(token=None, prev=None, prev_step=None)


def update_ink(objects):
    run = _CFG["_RUN"]
    token = (run["episode"], run["step"])
    if token == _INK["token"]:
        return
    if _INK["token"] is not None and token[0] != _INK["token"][0]:
        reset_ink()
    on, q = _CFG["marking"](objects["compass"], objects["shelf"])
    if on and _ROOTS:
        tip = np.asarray(q[:2], dtype=float)
        prev = _INK["prev"] if _INK["prev"] is not None and _INK["prev_step"] == token[1] - 1 else tip
        for record in _ROOTS.values():
            mask = near_segment(record["grid"], prev, tip, DOT_R)
            _set_visible(record, np.flatnonzero(mask & ~record["visible"]), True)
    _INK.update(token=token, prev=np.asarray(q[:2], dtype=float).copy() if on else None, prev_step=token[1])


def place_shelf(env):
    """Write the shelf's root pose for this trial's layout (env 0), as the simulator writes seeded jitter."""
    k = _CFG["current_layout"]()
    import torch
    base = env.unwrapped
    shelf = base.scene[_CFG["SHELF"].name]
    pose = shelf.data.default_root_pose.torch.clone()
    pose[:, :3] += base.scene.env_origins
    shift = _CFG["shelf_origin"](k) - _CFG["shelf_origin"](0)
    pose[:, 0] += float(shift[0])
    pose[:, 1] += float(shift[1])
    shelf.write_root_pose_to_sim_index(root_pose=pose)
    shelf.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(shelf.data.default_root_vel.torch))
    print(f"SCRIBE_LAYOUT layout={k} shelf_shift_mm=({shift[0] * 1000:.2f},{shift[1] * 1000:.2f})", flush=True)


def _wrap_spawn(obj, after):
    from isaaclab.sim.utils import clone
    original = getattr(obj.spawner_cfg.func, "__wrapped__", obj.spawner_cfg.func)

    def spawn(path, cfg, *args, **kwargs):
        prim = original(path, cfg, *args, **kwargs)
        after(prim.GetStage(), str(prim.GetPath()))
        return prim
    obj.spawner_cfg.func = clone(spawn)
    # Arena eagerly copies the spawn config; update its public copy too.
    obj.object_cfg.spawn.func = obj.spawner_cfg.func


def _patch_scene(scene):
    import yam_mesh
    yam_mesh._patch_scene(scene)  # mesh -> joints -> parts; keeps every kit wrapper
    if getattr(scene, "_scribe_token", None) is _TOKEN:
        return
    inner_object, inner_states = scene.task_object, scene.object_states

    @functools.wraps(inner_object)
    def task_object(spec):
        obj = inner_object(spec)
        if spec is _CFG["SHELF"]:
            _wrap_spawn(obj, spawn_ink)
        elif spec is _CFG["WALL"]:
            _wrap_spawn(obj, matte_wall)
        elif spec is _CFG["COMPASS"]:
            _wrap_spawn(obj, knurled_grip)
        return obj

    @functools.wraps(inner_states)
    def object_states(env, names):
        states = inner_states(env, names)
        if "compass" in states and "shelf" in states:
            update_ink(states)
        return states

    scene.task_object, scene.object_states = task_object, object_states
    scene._scribe_token = _TOKEN


class _Finder(importlib.abc.MetaPathFinder):
    _scribe_finder = True

    def find_spec(self, fullname, path=None, target=None):
        if fullname != "yam_arena.scene":
            return None
        import yam_parts
        spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        if spec is not None and spec.loader is not None:
            spec.loader = yam_parts._SceneLoader(spec.loader, _patch_scene)
        return spec


def install(module):
    global _CFG
    _CFG = module if isinstance(module, dict) else vars(module)
    if not any(getattr(f, "_scribe_finder", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    try:
        from yam_arena.inspect_embodiment import YamArenaEmbodiment as cls
    except ImportError:
        return
    if getattr(cls, "_scribe_wrapped", False):
        return
    inner_ensure, inner_reset = cls._ensure_sim, cls.reset

    @functools.wraps(inner_ensure)
    def _ensure_sim(self, *args, **kwargs):
        result = inner_ensure(self, *args, **kwargs)
        env = self._env
        if env is not None and not getattr(env, "_scribe_reset", False):
            env_reset = env.reset

            def reset_env(*a, **k):
                out = env_reset(*a, **k)
                place_shelf(env)
                return out
            env.reset = reset_env
            env._scribe_reset = True
        return result

    @functools.wraps(inner_reset)
    def reset(self, *args, **kwargs):
        seed = kwargs.get("seed")
        _CFG["ACTIVE_LAYOUT"] = 0 if seed is None else int(seed) % len(_CFG["LAYOUT_FRONT_X"])
        reset_ink()
        result = inner_reset(self, *args, **kwargs)
        reset_ink()
        _CFG["reset_path"]()
        return result

    cls._ensure_sim, cls.reset = _ensure_sim, reset
    cls._scribe_wrapped = True
