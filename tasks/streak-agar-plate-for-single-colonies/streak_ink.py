"""Visual-only streak lines and specimen area on the agar; runtime writes visibility only.

No simulator imports at module import. The dots are fixed, collision-free USD discs
under the dish body, so they move with the dish. A dot becomes visible where the loop
tip has ridden on the agar (the same pose-and-neck-flex test the scorer records). The
scorer and the demo never read these prims.
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
_LAST_TOKEN = None
_PREV_TIP = None
_PREV_STEP = None
DOT_PITCH = .0015
DOT_R = .0011
LINE_COLOR = (.93, .90, .78)
PATCH_COLOR = (.96, .94, .84)


def dot_grid(r_max):
    n = int(r_max/DOT_PITCH)
    a = np.arange(-n, n+1)*DOT_PITCH
    x, y = np.meshgrid(a, a)
    p = np.column_stack((x.ravel(), y.ravel()))
    return p[np.hypot(p[:, 0], p[:, 1]) <= r_max]


def near_segment(points, a, b, radius):
    d = b-a
    u = np.clip((points-a) @ d / max(float(d @ d), 1e-30), 0, 1)
    return np.linalg.norm(points-(a+u[:, None]*d), axis=1) <= radius


def _material(stage, path, color):
    from pxr import Gf, Sdf, UsdShade
    from yam_arena.station import linear_rgb
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path+"/Shader")
    shader.CreateIdAttr().Set("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(color)))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.6)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _instancer(stage, path, points, z, mat, visible):
    from pxr import Gf, UsdGeom, UsdShade, Vt
    inst = UsdGeom.PointInstancer.Define(stage, path)
    proto = path+"/Prototype"
    disc = UsdGeom.Cylinder.Define(stage, proto)
    disc.CreateRadiusAttr().Set(DOT_R)
    disc.CreateHeightAttr().Set(.0001)
    disc.CreateAxisAttr().Set("Z")
    UsdShade.MaterialBindingAPI.Apply(disc.GetPrim()).Bind(mat)
    inst.CreatePrototypesRel().SetTargets([proto])
    inst.CreateProtoIndicesAttr().Set(Vt.IntArray([0]*len(points)))
    inst.CreateIdsAttr().Set(Vt.Int64Array(list(range(len(points)))))
    inst.CreatePositionsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(float(x), float(y), z) for x, y in points]))
    inst.CreateInvisibleIdsAttr().Set(Vt.Int64Array([] if visible else list(range(len(points)))))
    return inst


def spawn_visuals(stage, root):
    from pxr import UsdGeom
    cfg = _CFG
    start = time.perf_counter()
    parent = root+"/Streak"
    UsdGeom.Xform.Define(stage, parent)
    grid = dot_grid(cfg["TRACE_R_MAX"])
    patch = np.array([p for p in dot_grid(cfg["PATCH_R"][1]) if cfg["in_patch"](p)])
    z = float(cfg["AGAR_TOP"]) + .00012
    _instancer(stage, parent+"/Specimen", patch, z-.00002, _material(stage, parent+"/SpecimenMat", PATCH_COLOR), True)
    inst = _instancer(stage, parent+"/Lines", grid, z, _material(stage, parent+"/LineMat", LINE_COLOR), False)
    _ROOTS[root] = dict(stage=stage, grid=grid, visible=np.zeros(len(grid), dtype=bool), inst=inst)
    print(f"STREAK_INK_SPAWN dots={len(grid)} specimen={len(patch)} seconds={time.perf_counter()-start:.4f}", flush=True)


def _set_visible(record, changed, value):
    if not len(changed):
        return
    from pxr import Vt
    record["visible"][changed] = value
    record["inst"].GetInvisibleIdsAttr().Set(Vt.Int64Array(np.flatnonzero(~record["visible"]).tolist()))


def reset_visuals():
    global _LAST_TOKEN, _PREV_TIP, _PREV_STEP
    for record in _ROOTS.values():
        _set_visible(record, np.flatnonzero(record["visible"]), False)
    _LAST_TOKEN = _PREV_TIP = _PREV_STEP = None


def update(loop, dish):
    global _LAST_TOKEN, _PREV_TIP, _PREV_STEP
    cfg = _CFG
    run = cfg["_RUN"]
    token = (run["episode"], run["step"])
    if token == _LAST_TOKEN:
        return
    if _LAST_TOKEN is not None and token[0] != _LAST_TOKEN[0]:
        reset_visuals()
    now, local = cfg["on_agar"](loop, dish)
    tip = np.asarray(local[:2], float)
    if now:
        previous = _PREV_TIP if _PREV_TIP is not None and _PREV_STEP == token[1]-1 else tip
        for record in _ROOTS.values():
            mask = near_segment(record["grid"], previous, tip, DOT_R+.0003)
            _set_visible(record, np.flatnonzero(mask & ~record["visible"]), True)
    _PREV_TIP = tip.copy() if now else None
    _PREV_STEP, _LAST_TOKEN = token[1], token


def _patch_scene(scene):
    import yam_mesh
    yam_mesh._patch_scene(scene)
    if getattr(scene, "_streak_ink_token", None) is _TOKEN:
        return
    inner_object, inner_states = scene.task_object, scene.object_states

    @functools.wraps(inner_object)
    def task_object(spec):
        obj = inner_object(spec)
        if spec is _CFG["DISH"]:
            from isaaclab.sim.utils import clone
            original_spawn = getattr(obj.spawner_cfg.func, "__wrapped__", obj.spawner_cfg.func)

            def spawn_dish(path, cfg, *args, **kwargs):
                prim = original_spawn(path, cfg, *args, **kwargs)
                spawn_visuals(prim.GetStage(), str(prim.GetPath()))
                return prim
            obj.spawner_cfg.func = clone(spawn_dish)
            obj.object_cfg.spawn.func = obj.spawner_cfg.func
        return obj

    @functools.wraps(inner_states)
    def object_states(env, names):
        states = inner_states(env, names)
        if "loop" in states and "dish" in states:
            _CFG["record_trace"](states["loop"], states["dish"])
            update(states["loop"], states["dish"])
        return states

    scene.task_object, scene.object_states = task_object, object_states
    scene._streak_ink_token = _TOKEN


class _Finder(importlib.abc.MetaPathFinder):
    _streak_ink_finder = True

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
    old = [f for f in sys.meta_path if getattr(f, "_streak_ink_finder", False)]
    for f in old:
        sys.meta_path.remove(f)
    sys.meta_path.insert(0, _Finder())
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    try:
        from yam_arena.inspect_embodiment import YamArenaEmbodiment as cls
    except ImportError:
        return
    if getattr(cls, "_streak_ink_reset", False):
        return
    inner_reset = cls.reset

    @functools.wraps(inner_reset)
    def reset(self, *args, **kwargs):
        reset_visuals()
        _CFG["reset_trace"]()
        result = inner_reset(self, *args, **kwargs)
        reset_visuals()
        _CFG["reset_trace"]()
        return result
    cls.reset = reset
    cls._streak_ink_reset = True
