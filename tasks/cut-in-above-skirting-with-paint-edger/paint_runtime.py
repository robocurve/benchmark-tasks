"""Simulator-side helpers for the cutting-in task (imported by the task file; no simulator imports at import time).

1. Paint film: a pre-authored grid of thin blue squares, one per 5 mm scoring cell, on the wall's face and on the
   skirting board's top and face, all hidden at the start. Each control step the task file's paint record (which
   cells the pad has painted) is updated, and newly painted cells are shown. The squares are visual only (no
   collision); the scorer reads the record, never the graphics.
2. The edger's two guide wheels (colliders 2 and 3 of its head) get their own material: free-turning wheels are
   modelled as low-friction rims, and the kit gives each body one material for all its colliders.
3. At every reset the record and the film are cleared.
"""
import functools
import importlib.abc
import importlib.machinery
import sys

import numpy as np

_CFG = None
_TOKEN = object()
_FILMS = {}          # prim path -> {"index": cell indices, "visible": bool array, "inst": instancer}
LIFT = 0.0002        # squares stand this far proud of the surface they are painted on


def _material(stage, path, color, roughness=0.85):
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


def _film_layout(which):
    """(cell indices, local positions, prototype index per cell, prototype scales) for the wall or the skirting."""
    c = _CFG
    cells = np.asarray(c["CELLS"])
    origin = np.array([c["WALL_X"], 0.0, c["TOP"]])
    if which == "wall":
        idx = np.arange(c["N_WALL"])
        pos = cells[idx] - origin + [-LIFT, 0.0, 0.0]
        proto = np.zeros(len(idx), int)
        scales = [(LIFT, c["CELL"] + 0.0002, c["CELL"] + 0.0002)]
    else:
        n_top = len(c["TOP_CELLS"])
        idx = np.arange(c["N_WALL"], len(cells))
        pos = cells[idx] - origin
        proto = np.where(idx < c["N_WALL"] + n_top, 0, 1)
        pos[proto == 0] += [0.0, 0.0, LIFT]
        pos[proto == 1] += [-LIFT, 0.0, 0.0]
        scales = [(c["BOARD_T"] / 3 + 0.0002, c["CELL"] + 0.0002, LIFT),
                  (LIFT, c["CELL"] + 0.0002, c["BOARD_H"] / 18 + 0.0002)]
    return idx, pos, proto, scales


def spawn_film(stage, root, which):
    from pxr import Gf, UsdGeom, UsdShade, Vt
    parent = root + "/PaintFilm"
    UsdGeom.Xform.Define(stage, parent)
    mat = _material(stage, parent + "/Material", _CFG["PAINT_BLUE"])
    idx, pos, proto, scales = _film_layout(which)
    inst = UsdGeom.PointInstancer.Define(stage, parent + "/Cells")
    paths = []
    for k, scale in enumerate(scales):
        path = f"{parent}/Cells/Prototype{k}"
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddScaleOp().Set(Gf.Vec3f(*scale))
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat)
        paths.append(path)
    inst.CreatePrototypesRel().SetTargets(paths)
    inst.CreateProtoIndicesAttr().Set(Vt.IntArray([int(p) for p in proto]))
    inst.CreateIdsAttr().Set(Vt.Int64Array(list(range(len(idx)))))
    inst.CreatePositionsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in pos]))
    inst.CreateInvisibleIdsAttr().Set(Vt.Int64Array(list(range(len(idx)))))
    _FILMS[root] = dict(index=idx, visible=np.zeros(len(idx), bool), inst=inst)
    print(f"EDGER_FILM {which} cells={len(idx)}", flush=True)


def wheel_material(stage, root):
    import isaaclab.sim as sim_utils
    static, dynamic = _CFG["WHEEL_FRICTION"]
    path = root + "/head/wheel_material"
    try:
        cfg = sim_utils.RigidBodyMaterialCfg(static_friction=static, dynamic_friction=dynamic,
                                             friction_combine_mode="min")
    except TypeError:  # an Isaac Lab without the field: author the combine mode on the USD material below
        cfg = sim_utils.RigidBodyMaterialCfg(static_friction=static, dynamic_friction=dynamic)
    cfg.func(path, cfg)
    combine = "cfg"
    try:
        from pxr import PhysxSchema
        api = PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(path))
        attr = api.GetFrictionCombineModeAttr() or api.CreateFrictionCombineModeAttr()
        attr.Set("min")
        combine = str(attr.Get())
    except Exception as e:  # noqa: BLE001 - keep the scene; the log says the mode was not set
        combine = f"unset ({e!r})"
    for k in _CFG["WHEEL_SHAPES"]:
        sim_utils.bind_physics_material(f"{root}/head/shape_{k}", path, stage=stage)
    print(f"EDGER_WHEEL_MATERIAL {root}/head shapes={tuple(_CFG['WHEEL_SHAPES'])} static={static} "
          f"dynamic={dynamic} combine={combine}", flush=True)


def _show(painted):
    from pxr import Vt
    painted = np.asarray(painted, bool)
    for film in _FILMS.values():
        want = painted[film["index"]]
        if np.array_equal(want, film["visible"]):
            continue
        film["visible"] = want.copy()
        film["inst"].GetInvisibleIdsAttr().Set(Vt.Int64Array(np.flatnonzero(~want).tolist()))


def clear_film():
    if _FILMS:
        _show(np.zeros(len(_CFG["CELLS"]), bool))


def update_film(objects):
    _CFG["paint_step"](objects)
    record = _CFG["_PAINT"]
    if record["new"] and _FILMS:
        _show(record["cells"])
        record["new"] = False


def _wrap_spawn(obj, after):
    from isaaclab.sim.utils import clone
    original = getattr(obj.spawner_cfg.func, "__wrapped__", obj.spawner_cfg.func)

    def spawn(path, cfg, *args, **kwargs):
        prim = original(path, cfg, *args, **kwargs)
        after(prim.GetStage(), str(prim.GetPath()))
        return prim
    # Keep the object's config (a mechanism's joint initial values live there); replace only the spawn function.
    obj.spawner_cfg.func = clone(spawn)
    obj.object_cfg.spawn.func = obj.spawner_cfg.func


def _patch_scene(scene):
    import yam_joints
    yam_joints._patch_scene(scene)  # joints -> parts; keeps every kit wrapper
    if getattr(scene, "_edger_token", None) is _TOKEN:
        return
    inner_object, inner_states = scene.task_object, scene.object_states

    @functools.wraps(inner_object)
    def task_object(spec):
        obj = inner_object(spec)
        if spec is _CFG["WALL"]:
            _wrap_spawn(obj, lambda stage, root: spawn_film(stage, root, "wall"))
        elif spec is _CFG["SKIRTING"]:
            _wrap_spawn(obj, lambda stage, root: spawn_film(stage, root, "skirting"))
        elif spec is _CFG["EDGER"]:
            _wrap_spawn(obj, wheel_material)
        return obj

    @functools.wraps(inner_states)
    def object_states(env, names):
        states = inner_states(env, names)
        if "edger" in states:
            update_film(states)
        return states

    scene.task_object, scene.object_states = task_object, object_states
    scene._edger_token = _TOKEN


class _Finder(importlib.abc.MetaPathFinder):
    _edger_finder = True

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
    if not any(getattr(f, "_edger_finder", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    try:
        from yam_arena.inspect_embodiment import YamArenaEmbodiment as cls
    except ImportError:
        return
    if getattr(cls, "_edger_wrapped", False):
        return
    inner_reset = cls.reset

    @functools.wraps(inner_reset)
    def reset(self, *args, **kwargs):
        clear_film()
        result = inner_reset(self, *args, **kwargs)
        _CFG["reset_paint"]()
        clear_film()
        return result

    cls.reset = reset
    cls._edger_wrapped = True
