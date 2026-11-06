"""Pre-authored band windows, printed order and fixed ink dots.

After spawn only visibility attributes change. An independent pose/joint history
uses the task's press rule; no scoring result or test event drives this readout.
"""
import functools
import importlib.abc
import importlib.machinery
import sys
import time
import numpy as np

_CFG = None
_ROOTS, _BANDS = {}, {}
_TOKEN = object()
_HISTORY = None
_LAST = None


def glyph_mask(points, pose, yaw, day, month):
    c,s = np.cos(yaw),np.sin(yaw)
    local = (points-np.asarray(pose)[:2]) @ np.array([[c,-s],[s,c]])
    mask = np.zeros(len(points),dtype=bool)
    for xy,size in _CFG['active_rectangles'](day,month):
        mask |= np.all(np.abs(local-xy)<=size/2+1e-12,axis=1)
    return mask


def dot_grid():
    size = _CFG['DOCUMENT_SIZE']
    # Include the entire sheet without placing disc centres outside it.
    x = np.linspace(-size[0]/2,size[0]/2,int(np.ceil(size[0]/_CFG['DOT_PITCH']))+1)
    y = np.linspace(-size[1]/2,size[1]/2,int(np.ceil(size[1]/_CFG['DOT_PITCH']))+1)
    xx,yy = np.meshgrid(x,y)
    return np.column_stack((xx.ravel(),yy.ravel()))


def material(stage,path,color):
    from pxr import Gf,Sdf,UsdShade
    from yam_arena.station import linear_rgb
    mat = UsdShade.Material.Define(stage,path)
    shader = UsdShade.Shader.Define(stage,path+'/Shader')
    shader.CreateIdAttr().Set('UsdPreviewSurface')
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(color)))
    shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.9)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    return mat


def bar(stage,path,xy,size,z,mat):
    from pxr import Gf,UsdGeom,UsdShade
    geom = UsdGeom.Cube.Define(stage,path)
    geom.CreateSizeAttr().Set(1.)
    geom.AddTranslateOp().Set(Gf.Vec3d(float(xy[0]),float(xy[1]),z))
    geom.AddScaleOp().Set(Gf.Vec3f(float(size[0]),float(size[1]),.0001))
    UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(mat)
    return UsdGeom.Imageable(geom.GetPrim())


def glyph_name(digit,segment):
    return "dot" if digit == -1 else f"d{digit}_{segment}"


def spawn_stamp(stage,root):
    from pxr import Gf,UsdGeom,UsdShade
    # Visual-only die carrier, no collider: 40x18x8 mm, stamp-local z 4..12 mm at
    # rest; the face link frame sits at its joint anchor, stamp-local z 2 mm.
    carrier = UsdGeom.Cube.Define(stage,root+'/face/die_carrier')
    carrier.CreateSizeAttr().Set(1.)
    carrier.AddTranslateOp().Set(Gf.Vec3d(0.,0.,.006))
    carrier.AddScaleOp().Set(Gf.Vec3f(.040,.018,.008))
    UsdShade.MaterialBindingAPI.Apply(carrier.GetPrim()).Bind(
        material(stage,root+'/face/CarrierRubber',(.055,.060,.075)))
    dark = material(stage,root+'/base/DisplayInk',(.025,.03,.045))
    light = material(stage,root+'/base/WindowLight',(.98,.95,.79))
    bands = []
    for b,x in enumerate((.020,-.020)):
        offset = np.array([x,-.022])
        bar(stage,root+f'/base/window_{b}',offset,(.017,.013),.0571,light)
        for d,s,xy,size in _CFG['glyph_rectangles']():
            if d < 0 or d//2 != b: continue
            # Centre each two-digit group in its own window.
            centre = np.mean(_CFG['DIGIT_CENTRES'][2*b:2*b+2])
            image = bar(stage,root+f'/base/band_{glyph_name(d,s)}',xy-[centre,0]+offset,size,.0572,dark)
            image.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
            bands.append((d,s,image))
    # Underside coordinates match the paper coordinates in world x/y. Viewed
    # from below the raised die therefore reads mirrored, like a real stamp.
    for d,s,xy,size in _CFG['glyph_rectangles']():
        image = bar(stage,root+f'/face/die_{glyph_name(d,s)}',xy,size,-.0021,dark)
        image.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        bands.append((d,s,image))
    layout = _CFG['current_layout']()
    _BANDS[root] = dict(bars=bands,values=(layout[0],layout[1]))
    # The first spawn happens inside reset, before any object_states call.
    set_digits(bands,layout[0],layout[1])


def spawn_document(stage,root):
    from pxr import UsdGeom,UsdShade,Vt
    start = time.perf_counter()
    parent = root+'/Dater'
    UsdGeom.Xform.Define(stage,parent)
    dark = material(stage,parent+'/Ink',(.025,.03,.045))
    blue = material(stage,parent+'/HeaderInk',(.05,.15,.32))
    bar(stage,parent+'/Header',(0.,.057),(.120,.006),.0011,blue)
    hx,hy = _CFG['BOX_SIZE']/2
    box = _CFG['BOX_XY']-_CFG['DOC_XY']
    for i,x in enumerate((-hx,hx)):
        bar(stage,parent+f'/box_x{i}',box+[x,0],(_CFG['BOX_LINE'],2*hy),.0011,dark)
    for i,y in enumerate((-hy,hy)):
        bar(stage,parent+f'/box_y{i}',box+[0,y],(2*hx,_CFG['BOX_LINE']),.0011,dark)
    scale = _CFG['ORDER_DIGIT_HEIGHT']/_CFG['GLYPH_HEIGHT']
    prints = []
    for d,s,xy,size in _CFG['glyph_rectangles']():
        image = bar(stage,parent+f'/order_{glyph_name(d,s)}',
                    scale*xy+_CFG['ORDER_XY']-_CFG['DOC_XY'],scale*size,.0011,dark)
        image.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        prints.append((d,s,image))
    grid = dot_grid()
    inst = UsdGeom.PointInstancer.Define(stage,parent+'/Dots')
    disc = UsdGeom.Cylinder.Define(stage,parent+'/Dots/Prototype')
    disc.CreateRadiusAttr().Set(_CFG['DOT_R'])
    disc.CreateHeightAttr().Set(.0001)
    disc.CreateAxisAttr().Set('Z')
    UsdShade.MaterialBindingAPI.Apply(disc.GetPrim()).Bind(dark)
    inst.CreatePrototypesRel().SetTargets([str(disc.GetPath())])
    n = len(grid)
    inst.CreateProtoIndicesAttr().Set(Vt.IntArray.FromNumpy(np.zeros(n,dtype=np.int32)))
    ids = Vt.Int64Array.FromNumpy(np.arange(n,dtype=np.int64))
    inst.CreateIdsAttr().Set(ids)
    positions = np.column_stack((grid,np.full(n,.00115))).astype(np.float32)
    inst.CreatePositionsAttr().Set(Vt.Vec3fArray.FromNumpy(positions))
    inst.CreateInvisibleIdsAttr().Set(ids)
    _ROOTS[root] = dict(inst=inst,prints=prints,grid=grid+_CFG['DOC_XY'],visible=np.zeros(n,dtype=bool))
    show_order()
    print(f'DATER_SPAWN dots={n} seconds={time.perf_counter()-start:.6f}',flush=True)


def set_dots(record,mask):
    from pxr import Vt
    if np.array_equal(record['visible'],mask): return
    record['visible'][:] = mask
    record['inst'].GetInvisibleIdsAttr().Set(Vt.Int64Array.FromNumpy(np.flatnonzero(~mask).astype(np.int64)))


def set_digits(bars,day,month):
    from pxr import UsdGeom
    text = (None if day is None else f'{day:02d}',None if month is None else f'{month:02d}')
    for d,s,image in bars:
        on = d == -1 or (text[d//2] is not None and s in _CFG['DIGITS'][int(text[d//2][d%2])])
        image.GetVisibilityAttr().Set(UsdGeom.Tokens.inherited if on else UsdGeom.Tokens.invisible)


def show_order():
    layout = _CFG['current_layout']()
    for record in _ROOTS.values(): set_digits(record['prints'],layout[2],layout[3])


def reset_visuals():
    global _LAST
    for record in _ROOTS.values(): set_dots(record,np.zeros(len(record['visible']),dtype=bool))
    for record in _BANDS.values(): record['values'] = None
    if _HISTORY is not None: _HISTORY.reset()
    _LAST = None


def update(stamp):
    global _LAST
    cfg = _CFG; run = cfg['_RUN']; token = (run['episode'],run['step'])
    if token == _LAST: return
    if _LAST is not None and token[0] != _LAST[0]: reset_visuals()
    e = _HISTORY.update(stamp,token)
    values = tuple(cfg['band_reading'](stamp,n) for n in ('day','month'))
    for record in _BANDS.values():
        if record['values'] != values:
            set_digits(record['bars'],*values); record['values'] = values
    if e is not None:
        if e['on_sheet']:
            for record in _ROOTS.values():
                mask = glyph_mask(record['grid'],e['pose'],e['yaw'],e['day'],e['month'])
                set_dots(record,record['visible']|mask)
        print(f"IMPRESSION t={float(run['t'] or 0):.3f} pose={e['pose'].tolist()} "
              f"yaw_deg={np.degrees(e['yaw']):.3f} day={e['day']} month={e['month']} "
              f"on_sheet={int(e['on_sheet'])}",flush=True)
    if run['env'] is not None and token[1]%10 == 0:
        print(f"DATER t={float(run['t'] or 0):.3f} layout={cfg['current_layout']()} "
              f"day_deg={stamp.angle_deg('day'):.5f} month_deg={stamp.angle_deg('month'):.5f} "
              f"day_vel={stamp.joint_vel['day']:.4f} month_vel={stamp.joint_vel['month']:.4f} "
              f"face_q_mm={stamp.joints['face']*1000:.3f} bands={values} "
              f"inked={int(_HISTORY.inked)} impressions={len(_HISTORY.events)}",flush=True)
    _LAST = token


def _patch_scene(scene):
    import yam_joints
    yam_joints._patch_scene(scene)
    if getattr(scene,'_work_order_dater_token',None) is _TOKEN: return
    inner_object,inner_states = scene.task_object,scene.object_states
    @functools.wraps(inner_object)
    def task_object(spec):
        obj = inner_object(spec)
        author = spawn_stamp if spec is _CFG['STAMP'] else spawn_document if spec is _CFG['WORK_ORDER'] else None
        if author is not None:
            from isaaclab.sim.utils import clone
            original = getattr(obj.spawner_cfg.func,'__wrapped__',obj.spawner_cfg.func)
            def spawn(path,cfg,*args,**kwargs):
                prim = original(path,cfg,*args,**kwargs)
                author(prim.GetStage(),str(prim.GetPath()))
                return prim
            obj.spawner_cfg.func = obj.object_cfg.spawn.func = clone(spawn)
        return obj
    @functools.wraps(inner_states)
    def object_states(env,names):
        states = inner_states(env,names)
        if 'stamp' in states:
            _CFG['record_press'](states['stamp'])
            update(states['stamp'])
        return states
    scene.task_object,scene.object_states = task_object,object_states
    scene._work_order_dater_token = _TOKEN


class _Finder(importlib.abc.MetaPathFinder):
    _work_order_dater_finder = True
    def find_spec(self,fullname,path=None,target=None):
        if fullname != 'yam_arena.scene': return None
        import yam_parts
        spec = importlib.machinery.PathFinder.find_spec(fullname,path)
        if spec is not None and spec.loader is not None:
            spec.loader = yam_parts._SceneLoader(spec.loader,_patch_scene)
        return spec


def install(module):
    global _CFG,_HISTORY
    _CFG = module; _HISTORY = _CFG['PressHistory']()
    if not any(getattr(f,'_work_order_dater_finder',False) for f in sys.meta_path): sys.meta_path.insert(0,_Finder())
    if 'yam_arena.scene' in sys.modules: _patch_scene(sys.modules['yam_arena.scene'])
    try:
        from yam_arena.inspect_embodiment import YamArenaEmbodiment as cls
    except ImportError: return
    if getattr(cls,'_work_order_dater_reset',False): return
    inner = cls.reset
    @functools.wraps(inner)
    def reset(self,*args,**kwargs):
        # Arena's seed is keyword-only. The verbatim release wrapper erases
        # its signature, so read the forwarded keyword rather than introspect.
        seed = kwargs.get('seed')
        _CFG['_CURRENT_LAYOUT'] = _CFG['LAYOUTS'][0 if seed is None else seed%16]
        reset_visuals(); _CFG['reset_history']()
        if _ROOTS: show_order()
        layout = _CFG['current_layout']()
        for record in _BANDS.values():
            set_digits(record['bars'],layout[0],layout[1])
            record['values'] = (layout[0],layout[1])
        # The seed and printed date precede the original reset's first image.
        result = inner(self,*args,**kwargs)
        reset_visuals(); _CFG['reset_history']()
        return result
    cls.reset = reset
    cls._work_order_dater_reset = True
