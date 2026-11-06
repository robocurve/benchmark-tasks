"""Pre-authored fixed visual ink; runtime writes visibility attributes only.

No simulator imports at module import. Pose-only readout wraps the kit's state
reader; scorer and demo never inspect this renderer's independent history.
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
_WALL = None
_INTERVALS = []


def dot_grid(cfg):
    half = cfg["BOARD_SIZE"]/2 + .010
    n = int(round(2*half/cfg["DOT_PITCH"])) + 1
    a = np.linspace(-half, half, n)
    x, y = np.meshgrid(a, a)
    return np.column_stack((x.ravel(), y.ravel()))


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
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.9)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _disc(stage, path, radius, z, mat, xy=(0., 0.)):
    from pxr import Gf, UsdGeom, UsdShade
    disc = UsdGeom.Cylinder.Define(stage, path)
    disc.CreateRadiusAttr().Set(radius)
    disc.CreateHeightAttr().Set(.0001)
    disc.CreateAxisAttr().Set("Z")
    disc.AddTranslateOp().Set(Gf.Vec3d(float(xy[0]), float(xy[1]), z))
    UsdShade.MaterialBindingAPI.Apply(disc.GetPrim()).Bind(mat)
    return disc


def _bar(stage, path, xy, length, width, angle, z, mat):
    from pxr import Gf, UsdGeom, UsdShade
    geom = UsdGeom.Cube.Define(stage, path)
    geom.CreateSizeAttr().Set(1.)
    geom.AddTranslateOp().Set(Gf.Vec3d(float(xy[0]), float(xy[1]), z))
    geom.AddRotateZOp().Set(float(np.degrees(angle)))
    geom.AddScaleOp().Set(Gf.Vec3f(length, width, .0001))
    UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(mat)


def spawn_visuals(stage, root):
    from pxr import Gf, UsdGeom, Vt
    cfg = _CFG
    start = time.perf_counter()
    printed = root+"/Printed"
    UsdGeom.Xform.Define(stage, printed)
    grid_mat = _material(stage, printed+"/GridMaterial", (.10, .12, .15))
    mark_mat = _material(stage, printed+"/MarkMaterial", (.035, .10, .38))
    z = .0011
    for i, d in enumerate((-.0225, .0225)):
        _bar(stage, printed+f"/grid_x_{i}", (0., d), cfg["BOARD_SIZE"], .0025, 0., z, grid_mat)
        _bar(stage, printed+f"/grid_y_{i}", (d, 0.), cfg["BOARD_SIZE"], .0025, np.pi/2, z, grid_mat)
    prints = {}
    for cell in cfg["CELLS"]:
        centre = cfg["cell_center"](cell)-cfg["BOARD_XY"]
        for symbol in "XO":
            prefix = printed+f"/mark_{cell[0]}_{cell[1]}_{symbol}"
            group = UsdGeom.Xform.Define(stage, prefix)
            im = UsdGeom.Imageable(group.GetPrim())
            im.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
            prints[cell, symbol] = im
            if symbol == "X":
                for i, angle in enumerate((np.pi/4, -np.pi/4)):
                    _bar(stage, prefix+f"/bar_{i}", centre, np.sqrt(2)*.030, .004, angle, z, mark_mat)
            else:
                for i in range(64):
                    a,b = 2*np.pi*np.array([i,i+1])/64
                    p = centre+.015*np.array([np.cos(a),np.sin(a)])
                    q = centre+.015*np.array([np.cos(b),np.sin(b)])
                    _bar(stage,prefix+f"/bar_{i}",(p+q)/2,float(np.linalg.norm(q-p))+.0001,.004,
                         (a+b)/2+np.pi/2,z,mark_mat)
    parent = root+"/Ink"
    UsdGeom.Xform.Define(stage, parent)
    mat = _material(stage, parent+"/Material", (.018, .020, .026))
    grid = dot_grid(cfg)
    mode = cfg["INK_MODE"]
    if mode == "instancer":
        inst = UsdGeom.PointInstancer.Define(stage, parent+"/Dots")
        prototype = parent+"/Dots/Prototype"
        _disc(stage, prototype, cfg["DOT_R"], 0., mat)
        inst.CreatePrototypesRel().SetTargets([prototype])
        inst.CreateProtoIndicesAttr().Set(Vt.IntArray([0]*len(grid)))
        inst.CreateIdsAttr().Set(Vt.Int64Array(list(range(len(grid)))))
        inst.CreatePositionsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(float(x), float(y), .00115) for x, y in grid]))
        inst.CreateInvisibleIdsAttr().Set(Vt.Int64Array(list(range(len(grid)))))
        images = None
    else:
        inst, images = None, []
        for i, xy in enumerate(grid):
            disc = _disc(stage, parent+f"/dot_{i}", cfg["DOT_R"], .00115, mat, xy)
            im = UsdGeom.Imageable(disc.GetPrim())
            im.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
            images.append(im)
    _ROOTS[root] = dict(stage=stage, prints=prints, grid=grid+cfg["BOARD_XY"], visible=np.zeros(len(grid), dtype=bool), inst=inst, images=images)
    show_board()
    print(f"INK_SPAWN mode={mode} dots={len(grid)} seconds={time.perf_counter()-start:.6f}", flush=True)


def _set_visible(record, changed, value):
    if not len(changed): return
    from pxr import UsdGeom, Vt
    record["visible"][changed] = value
    if record["inst"] is not None:
        record["inst"].GetInvisibleIdsAttr().Set(Vt.Int64Array(np.flatnonzero(~record["visible"]).tolist()))
    else:
        for i in changed:
            record["images"][i].GetVisibilityAttr().Set(UsdGeom.Tokens.inherited if value else UsdGeom.Tokens.invisible)


def show_board():
    from pxr import UsdGeom
    board = _CFG["current_board"]()
    for record in _ROOTS.values():
        for (cell,symbol),im in record["prints"].items():
            visible = board[cell[0]*3+cell[1]] == symbol
            im.GetVisibilityAttr().Set(UsdGeom.Tokens.inherited if visible else UsdGeom.Tokens.invisible)


def reset_visuals():
    global _LAST_TOKEN, _PREV_TIP, _PREV_STEP, _WALL
    for record in _ROOTS.values():
        _set_visible(record, np.flatnonzero(record["visible"]), False)
    _LAST_TOKEN = _PREV_TIP = _PREV_STEP = _WALL = None
    _INTERVALS.clear()


def update(pen):
    global _LAST_TOKEN, _PREV_TIP, _PREV_STEP, _WALL
    cfg = _CFG
    run = cfg["_RUN"]
    token = (run["episode"], run["step"])
    wall = time.perf_counter()
    if token == _LAST_TOKEN: return
    if _LAST_TOKEN is not None and token[0] != _LAST_TOKEN[0]: reset_visuals()
    if _WALL is not None: _INTERVALS.append((wall-_WALL)*1000)
    _WALL = wall
    tip = cfg["nib_tip"](pen)
    now = cfg["writing"](pen)
    if now:
        previous = _PREV_TIP if _PREV_TIP is not None and _PREV_STEP == token[1]-1 else tip
        for record in _ROOTS.values():
            mask = near_segment(record["grid"], previous[:2], tip[:2], cfg["DOT_R"])
            _set_visible(record, np.flatnonzero(mask & ~record["visible"]), True)
    _PREV_TIP = tip.copy() if now else None
    _PREV_STEP, _LAST_TOKEN = token[1], token
    if token[1] % 20 == 0:
        visible = max((int(r["visible"].sum()) for r in _ROOTS.values()), default=0)
        ms = float(np.mean(_INTERVALS)) if _INTERVALS else 0.
        print(f"INK_STATE t={float(run['t'] or 0):.3f} visible={visible} wall_ms_per_step={ms:.3f} "
              f"nib_q_mm={pen.joints['nib']*1000:.3f} tip=({tip[0]:.6f},{tip[1]:.6f},{tip[2]:.6f}) writing={int(now)}", flush=True)
        _INTERVALS.clear()


def _patch_scene(scene):
    import yam_joints
    yam_joints._patch_scene(scene)
    if getattr(scene, "_builder4_ink_token", None) is _TOKEN: return
    inner_object, inner_states = scene.task_object, scene.object_states

    @functools.wraps(inner_object)
    def task_object(spec):
        obj = inner_object(spec)
        if spec is _CFG["SHEET"]:
            from isaaclab.sim.utils import clone
            original_spawn = getattr(obj.spawner_cfg.func, "__wrapped__", obj.spawner_cfg.func)
            def spawn_sheet(path, cfg, *args, **kwargs):
                prim = original_spawn(path, cfg, *args, **kwargs)
                spawn_visuals(prim.GetStage(), str(prim.GetPath()))
                return prim
            obj.spawner_cfg.func = clone(spawn_sheet)
            # Arena eagerly copies the spawn config; update its public copy too.
            obj.object_cfg.spawn.func = obj.spawner_cfg.func
        return obj

    @functools.wraps(inner_states)
    def object_states(env, names):
        states = inner_states(env, names)
        if "pen" in states:
            _CFG["record_path"](states["pen"])
            update(states["pen"])
        return states

    scene.task_object, scene.object_states = task_object, object_states
    scene._builder4_ink_token = _TOKEN


class _Finder(importlib.abc.MetaPathFinder):
    _builder4_ink_finder = True
    def find_spec(self, fullname, path=None, target=None):
        if fullname != "yam_arena.scene": return None
        import yam_parts
        spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        if spec is not None and spec.loader is not None:
            spec.loader = yam_parts._SceneLoader(spec.loader, _patch_scene)
        return spec


def install(module):
    global _CFG
    _CFG = module if isinstance(module, dict) else vars(module)
    if not any(getattr(f, "_builder4_ink_finder", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())
    if "yam_arena.scene" in sys.modules: _patch_scene(sys.modules["yam_arena.scene"])
    try:
        from yam_arena.inspect_embodiment import YamArenaEmbodiment as cls
    except ImportError:
        return
    if getattr(cls, "_builder4_ink_reset", False): return
    inner_reset = cls.reset
    @functools.wraps(inner_reset)
    def reset(self, *args, **kwargs):
        seed = kwargs.get("seed")
        _CFG["_CURRENT_BOARD"] = _CFG["LAYOUTS"][0 if seed is None else seed % 16]
        reset_visuals()
        _CFG["reset_path"]()
        if _ROOTS: show_board()
        # Selection and visibility precede the original reset and its first image.
        result = inner_reset(self, *args, **kwargs)
        reset_visuals()
        _CFG["reset_path"]()
        return result
    cls.reset = reset
    cls._builder4_ink_reset = True
