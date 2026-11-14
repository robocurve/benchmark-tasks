"""Attached, pre-authored markings and instrument readouts; visibility only.

The shared physical instrument model drives the readouts. Scoring never reads
these prims. All pxr/Isaac imports are confined to spawned-scene code paths.
"""
import functools
import importlib.abc
import importlib.machinery
import math
import sys
import numpy as np

_CFG=None
_TOKEN=object()
_RECORDS={}
_LAST=None
_PREVIOUS={}
_LIFTS=()
_READY=False


def material(stage,path,color,opacity=1.):
    from pxr import Gf,Sdf,UsdShade
    from yam_arena.station import linear_rgb
    m=UsdShade.Material.Define(stage,path)
    shader=UsdShade.Shader.Define(stage,path+'/Shader')
    shader.CreateIdAttr().Set('UsdPreviewSurface')
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(color)))
    shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.55)
    shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(opacity)
    m.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    return m


def cube(stage,path,center,size,mat):
    from pxr import Gf,UsdGeom,UsdShade
    g=UsdGeom.Cube.Define(stage,path);g.CreateSizeAttr().Set(1.)
    g.AddTranslateOp().Set(Gf.Vec3d(*map(float,center)))
    g.AddScaleOp().Set(Gf.Vec3f(*map(float,size)))
    UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat)
    return UsdGeom.Imageable(g.GetPrim())


def show(image,on):
    from pxr import UsdGeom
    image.GetVisibilityAttr().Set(UsdGeom.Tokens.inherited if on else UsdGeom.Tokens.invisible)


def group(stage,path):
    from pxr import UsdGeom
    return UsdGeom.Imageable(UsdGeom.Xform.Define(stage,path).GetPrim())


def text_xy(xy,right=0.,up=0.):
    """Printed coordinates: up is world +x, reading direction is world -y."""
    return (xy[0]+up,xy[1]-right)


def text_cube(stage,path,xy,right,up,width,height,z,mat):
    return cube(stage,path,(*text_xy(xy,right,up),z),(height,width,.0001),mat)


def segments(stage,path,xy,height,z,mat):
    # 0.6 aspect, 0.15-height strokes, matching the printed card.
    out={}; w=.6*height;t=.15*height
    for s,dx,dy,sx,sy in (('a',0,(height-t)/2,w-t,t),('g',0,0,w-t,t),('d',0,-(height-t)/2,w-t,t),
                         ('b',(w-t)/2,(height-t)/4,t,(height-t)/2),('c',(w-t)/2,-(height-t)/4,t,(height-t)/2),
                         ('e',-(w-t)/2,-(height-t)/4,t,(height-t)/2),('f',-(w-t)/2,(height-t)/4,t,(height-t)/2)):
        out[s]=text_cube(stage,path+'_'+s,xy,dx,dy,sx,sy,z,mat)
    return out


def printed_number(stage,path,value,xy,height,z,mat):
    text=str(value); images=[]
    for i,ch in enumerate(text):
        bars=segments(stage,path+f'_d{i}',text_xy(xy,(i-(len(text)-1)/2)*height*.75),height,z,mat)
        for s,g in bars.items(): show(g,s in _CFG['DIGITS'][int(ch)])
        images.extend(bars.values())
    return images


def spawn(stage,root,spec):
    from pxr import UsdShade
    cfg=_CFG; rec={'kind':spec.name}
    if spec.name in ('cuvette_a','cuvette_b'):
        # Part colliders themselves retain the same shape/material definitions.
        glass=material(stage,root+'/Glass',(.65,.86,.91),.14)
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(root+'/shape_0')).Bind(glass)
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(root+'/shape_1')).Bind(glass)
        # White/blue visual heads cover the common collider; no collision APIs.
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(root+'/shape_2')).Bind(
            material(stage,root+'/HeadBase',(.8,.8,.8),0.))
        variants={}
        for blank in (False,True):
            cap=material(stage,root+('/White' if blank else '/Blue'),(.96,.96,.88) if blank else (.03,.22,.85))
            liquid=material(stage,root+('/Water' if blank else '/Dye'),(.88,.95,.97) if blank else (.02,.23,.90),.8)
            frost=material(stage,root+('/FrostBlank' if blank else '/FrostSample'),(.80,.84,.84))
            ridge=material(stage,root+('/RidgeBlank' if blank else '/RidgeSample'),(.35,.43,.46))
            for turned in (False,True):
                path=root+f'/variant_{int(blank)}_{int(turned)}'; image=group(stage,path)
                cube(stage,path+'/head',(0,0,.057),(.0142,.0142,.0122),cap)
                cube(stage,path+'/liquid',(0,0,.017),(.0105,.0105,.030),liquid)
                for sign in (-1,1):
                    i=0 if sign<0 else 1
                    c=(sign*.00635,0,.023) if turned else (0,sign*.00635,.023)
                    size=(.0002,.0115,.040) if turned else (.0115,.0002,.040)
                    cube(stage,path+f'/frost_{i}',c,size,frost)
                    for k in range(10):
                        c=(sign*.00646,0,.006+k*.0036) if turned else (0,sign*.00646,.006+k*.0036)
                        size=(.0001,.0115,.0007) if turned else (.0115,.0001,.0007)
                        cube(stage,path+f'/ridge_{i}_{k}',c,size,ridge)
                variants[(blank,turned)]=image
        rec['variants']=variants
    elif spec.name=='card':
        ink=material(stage,root+'/Ink',(.025,.03,.045))
        blue=material(stage,root+'/HeaderInk',(.05,.15,.32))
        cube(stage,root+'/header',(.030,0,.0011),(.006,.060,.0001),blue)
        rec['values']={}
        for nm in cfg['WAVELENGTHS']:
            path=root+f'/nm_{nm}';image=group(stage,path)
            printed_number(stage,path+'/value',nm,(0,0),cfg['CARD_DIGIT_HEIGHT'],.0011,ink)
            rec['values'][nm]=image
    elif spec.name=='colorimeter':
        p=root+'/base'
        ink=material(stage,p+'/Ink',(.025,.03,.045));light=material(stage,p+'/LabelInk',(.96,.96,.88))
        green=material(stage,p+'/Green',(.08,.96,.20));red=material(stage,p+'/Red',(.98,.06,.03))
        dark=material(stage,p+'/Dark',(.025,.04,.045));beam=material(stage,p+'/BeamInk',(.05,.72,.93))
        # Printed triangular marks are thin meshes attached to the housing top.
        from pxr import Gf,UsdGeom
        for i,sign in enumerate((-1,1)):
            x=.022+sign*.025; y=-.040
            g=UsdGeom.Mesh.Define(stage,p+f'/beam_triangle_{i}')
            g.CreatePointsAttr().Set([Gf.Vec3f(x-sign*.004,y,.0651),Gf.Vec3f(x+sign*.004,y-.004,.0651),Gf.Vec3f(x+sign*.004,y+.004,.0651)])
            g.CreateDoubleSidedAttr().Set(True)
            g.CreateFaceVertexCountsAttr().Set([3]);g.CreateFaceVertexIndicesAttr().Set([0,1,2])
            UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(beam)
        rec['lamps']=[]
        for i,nm in enumerate(cfg['WAVELENGTHS']):
            x=-.027+i*.018
            cube(stage,p+f'/lamp_dark_{i}',(x,.031,.0651),(.007,.005,.0001),dark)
            rec['lamps'].append(cube(stage,p+f'/lamp_on_{i}',(x,.031,.0652),(.007,.005,.0001),green))
            printed_number(stage,p+f'/label_{i}',nm,(x,.020),.005,.0651,light)
        # The open hood ends at y=+16 mm, below the printed lamp labels.
        display=cfg['DISPLAY_LOCAL']
        cube(stage,p+'/display_back',(*display,.0651),(.012,.038,.0001),dark)
        rec['display']=[segments(stage,p+f'/display_{i}',text_xy(display,-.007+i*.007),.010,.0652,green) for i in range(3)]
        rec['minus']=text_cube(stage,p+'/sign_minus',display,-.015,0,.004,.0015,.0652,green)
        rec['plus']=text_cube(stage,p+'/sign_plus',display,-.015,0,.0015,.004,.0652,green)
        rec['decimal']=text_cube(stage,p+'/decimal',display,-.0035,-.004,.0012,.0012,.0652,green)
        # CAL lamp is beside its labelled cap.
        cube(stage,p+'/cal_dark',(.036,.043,.0651),(.006,.005,.0001),dark)
        rec['cal_green']=cube(stage,p+'/cal_green',(.036,.043,.0652),(.006,.005,.0001),green)
        rec['cal_red']=cube(stage,p+'/cal_red',(.036,.043,.0653),(.006,.005,.0001),red)
        for name in ('less','more','cal_button'):
            # Child-link frame is at each button anchor; labels move with cap.
            mat=material(stage,root+'/'+name+'/Ink',(.02,.025,.03))
            if name=='cal_button':
                # C A L, drawn with bars rather than font assets.
                for j,text in enumerate(('adef','abcefg','def')):
                    bars=segments(stage,root+'/'+name+f'/letter_{j}',text_xy((0,0),(j-1)*.0045),.006,.0062,mat)
                    for s,g in bars.items():show(g,s in text)
            else:
                # Down-arrow shortens wavelength; up-arrow lengthens it.
                from pxr import Gf,UsdGeom
                direction=-1 if name=='less' else 1
                points=[Gf.Vec3f(direction*.004,0,.0061),
                        Gf.Vec3f(-direction*.003,-.0035,.0061),
                        Gf.Vec3f(-direction*.003,.0035,.0061)]
                g=UsdGeom.Mesh.Define(stage,root+'/'+name+'/arrow')
                g.CreatePointsAttr().Set(points);g.CreateDoubleSidedAttr().Set(True)
                g.CreateFaceVertexCountsAttr().Set([3]);g.CreateFaceVertexIndicesAttr().Set([0,1,2])
                UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat)
    else: return
    _RECORDS[root]=rec
    show_layout()
    if spec.name=='colorimeter': render(cfg['_INSTRUMENT'],0.)


def show_layout():
    if not _RECORDS: return
    cfg=_CFG;l=cfg['current_layout']();blank,_=cfg['identity_names']()
    for rec in _RECORDS.values():
        name=rec['kind']
        if 'variants' in rec:
            key=(name==blank,l[2] if name==blank else l[3])
            for k,image in rec['variants'].items():show(image,k==key)
        if 'values' in rec:
            for nm,image in rec['values'].items():show(image,nm==cfg['WAVELENGTHS'][l[0]])


def render(model,t):
    text=model.display();digits=None if model.reading is None else text[1:].replace('.','')
    lamp=model.cal_lamp(t)
    for rec in _RECORDS.values():
        if rec['kind']!='colorimeter':continue
        value=(model.w,lamp,text)
        if rec.get('last')==value:continue
        for i,image in enumerate(rec['lamps']):show(image,i==model.w)
        show(rec['cal_green'],lamp=='green');show(rec['cal_red'],lamp=='red')
        for i,bars in enumerate(rec['display']):
            active='g' if digits is None else _CFG['DIGITS'][int(digits[i])]
            for s,image in bars.items():show(image,s in active)
        show(rec['minus'],digits is not None);show(rec['plus'],digits is not None and text[0]=='+')
        show(rec['decimal'],digits is not None)
        rec['last']=value


def reset_diagnostics():
    global _LAST
    _LAST=None;_PREVIOUS.clear()


def reset_visuals():
    reset_diagnostics()
    for rec in _RECORDS.values():rec.pop('last',None)


def scoring_events(record,dropped):
    if not _CFG['DIAG_EVENTS']:return
    t=float(_CFG['_RUN']['t'] or 0.)
    for key,cond in (('record',record),('dropped',dropped)):
        if cond and not _PREVIOUS.get(key,False): print(f'EVENT t={t:.3f} {key}',flush=True)
        _PREVIOUS[key]=cond


def right_tcp_rotation():
    cfg=_CFG;env=cfg['_RUN']['env']
    if env is None:return None
    from yam_arena import ik,station
    from yam_arena.constants import ARM_JOINTS
    art=env.unwrapped.scene['right_robot'];ids=art.find_joints(ARM_JOINTS,preserve_order=True)[0]
    q=getattr(art.data.joint_pos,'torch',art.data.joint_pos)[0]
    if hasattr(q,'detach'):q=q.detach().cpu().numpy()
    return ik.tcp_pose(np.asarray(q)[ids],station.BASES['right'])[1]


def diagnostics(objects,model,t):
    cfg=_CFG; token=(cfg['_RUN']['episode'],cfg['_RUN']['step'])
    def edge(key,cond,yes,no=None):
        if cond!=_PREVIOUS.get(key,False) and (cond or no): print(f'EVENT t={t:.3f} {yes if cond else no}',flush=True)
        _PREVIOUS[key]=cond
    for event in model.events:
        if event[0]=='wavelength': print(f'EVENT t={t:.3f} wavelength w={cfg["WAVELENGTHS"][event[1]]}',flush=True)
        else:
            _,outcome,w,a=event
            print(f'EVENT t={t:.3f} CAL {outcome} w={cfg["WAVELENGTHS"][w]} A_ref={"none" if a is None else f"{a:.5f}"}',flush=True)
    value=None if model.reading is None else round(model.reading,4)
    if 'reading' not in _PREVIOUS or value!=_PREVIOUS['reading']:
        print(f'EVENT t={t:.3f} reading A={"none" if value is None else f"{value:.5f}"} valid={int(value is not None)}',flush=True)
    _PREVIOUS['reading']=value
    for name in ('cuvette_a','cuvette_b'):
        s=objects[name];p=s.origin;tilt=s.tilt_deg();low=float(cfg['bottom_corners'](s)[:,2].min())
        edge(name+'_seat',cfg['seated'](s),name+' seat',name+' unseat')
        for site,xy,plane in [('well',cfg['WELL_XY'],cfg['MOUTH']+cfg['LEAD_R'])]+[
                (f'pocket_{i}',xy,cfg['RACK_TOP']+cfg['LEAD_R']) for i,xy in enumerate(cfg['POCKETS'])]:
            key=name+'_'+site;prev=_PREVIOUS.get(key+'_bottom');near=np.max(np.abs(p[:2]-xy))<.025
            if low>plane+.001 or not near:_PREVIOUS[key+'_armed']=True
            if prev is not None and prev>plane>=low and near and _PREVIOUS.get(key+'_armed',False):
                dx,dy=(p[:2]-xy)*1000
                print(f'EVENT t={t:.3f} {name} mouth site={site} dx_mm={dx:.4f} dy_mm={dy:.4f} bottom_z={low:.6f} tilt_deg={tilt:.4f}',flush=True)
                _PREVIOUS[key+'_armed']=False
            _PREVIOUS[key+'_bottom']=low
    hands=cfg['grippers']() or {}
    for arm,(tcp,opening) in hands.items():
        prev=_PREVIOUS.get(arm+'_opening')
        if prev is not None and prev<=.20<opening:
            for name in ('cuvette_a','cuvette_b'):
                s=objects[name]
                if np.linalg.norm(s.to_world((0,0,cfg['GRIP_LOCAL']))-tcp)>=.025:continue
                site,xy=min([('well',cfg['WELL_XY'])]+[(f'pocket_{i}',p) for i,p in enumerate(cfg['POCKETS'])],key=lambda a:np.linalg.norm(s.origin[:2]-a[1]))
                dx,dy=(s.origin[:2]-xy)*1000
                print(f'EVENT t={t:.3f} {name} release site={site} arm={arm} dx_mm={dx:.4f} dy_mm={dy:.4f} bottom_z={s.origin[2]:.6f} tilt_deg={s.tilt_deg():.4f}',flush=True)
        _PREVIOUS[arm+'_opening']=opening
    for lift_t,identity,tag in _LIFTS:
        key=tag+'_inhand'
        if t<lift_t-1e-9 or _PREVIOUS.get(key,False) or 'right' not in hands:continue
        R=right_tcp_rotation()
        if R is None:continue
        name=cfg['identity_names']()[0 if identity=='blank' else 1];s=objects[name]
        offset=R.T@(s.origin-hands['right'][0]);relative=R.T@s.rotation()
        yaw=np.degrees(np.arctan2(relative[1,0],relative[0,0]));tilt=np.degrees(np.arccos(np.clip(-relative[2,2],-1,1)))
        print(f'EVENT t={t:.3f} inhand cuvette={name} dx_mm={offset[0]*1000:.4f} dy_mm={offset[1]*1000:.4f} yaw_deg={yaw:.4f} tilt_deg={tilt:.4f}',flush=True)
        _PREVIOUS[key]=True
    edge('lid_closed',cfg['closed'](objects['colorimeter']),'lid close','lid open')


def update(objects):
    global _LAST
    cfg=_CFG;run=cfg['_RUN'];token=(run['episode'],run['step'])
    if token==_LAST:return
    model=cfg['instrument'](objects);t=float(run['t'] or 0.)
    render(model,t)
    if cfg['DIAG_EVENTS']:diagnostics(objects,model,t)
    _LAST=token


def _patch_scene(scene):
    import yam_joints
    yam_joints._patch_scene(scene)
    if getattr(scene,'_colorimeter_token',None) is _TOKEN:return
    inner_object,inner_states=scene.task_object,scene.object_states
    @functools.wraps(inner_object)
    def task_object(spec):
        obj=inner_object(spec)
        if spec.name in ('colorimeter','cuvette_a','cuvette_b','card'):
            from isaaclab.sim.utils import clone
            original=getattr(obj.spawner_cfg.func,'__wrapped__',obj.spawner_cfg.func)
            def author(path,cfg,*args,**kwargs):
                prim=original(path,cfg,*args,**kwargs)
                spawn(prim.GetStage(),str(prim.GetPath()),spec)
                return prim
            obj.spawner_cfg.func=obj.object_cfg.spawn.func=clone(author)
        return obj
    @functools.wraps(inner_states)
    def object_states(env,names):
        states=inner_states(env,names)
        if all(n in states for n in ('colorimeter','cuvette_a','cuvette_b')):update(states)
        return states
    scene.task_object,scene.object_states=task_object,object_states
    scene._colorimeter_token=_TOKEN


class _Finder(importlib.abc.MetaPathFinder):
    _ccms_finder=True
    def find_spec(self,fullname,path=None,target=None):
        if fullname!='yam_arena.scene':return None
        import yam_parts
        spec=importlib.machinery.PathFinder.find_spec(fullname,path)
        if spec is not None and spec.loader is not None:spec.loader=yam_parts._SceneLoader(spec.loader,_patch_scene)
        return spec


def install(module):
    global _CFG,_LIFTS
    _CFG=module;elapsed=0.;lifts=[]
    for w in module['demo']()['right']:
        elapsed+=w.duration
        if w.label.endswith(' lift'):lifts.append((elapsed,w.label.split()[0],w.label.removesuffix(' lift')))
    _LIFTS=tuple(lifts)
    if not any(getattr(f,'_ccms_finder',False) for f in sys.meta_path):sys.meta_path.insert(0,_Finder())
    if 'yam_arena.scene' in sys.modules:_patch_scene(sys.modules['yam_arena.scene'])
    try:
        from yam_arena.inspect_embodiment import YamArenaEmbodiment as cls
    except ImportError:return
    if getattr(cls,'_ccms_reset',False):return
    inner=cls.reset
    @functools.wraps(inner)
    def reset(self,*args,**kwargs):
        global _READY
        seed=kwargs.get('seed');_CFG['_CURRENT_LAYOUT']=_CFG['LAYOUTS'][0 if seed is None else seed%16]
        reset_visuals();_CFG['reset_history']()
        # No USD work before the first original reset returns. First-spawn
        # authoring already uses the selected layout before its first image.
        if _READY:
            show_layout();render(_CFG['_INSTRUMENT'],0.)
        result=inner(self,*args,**kwargs)
        _READY=True
        reset_visuals();_CFG['reset_history']();show_layout();render(_CFG['_INSTRUMENT'],0.)
        return result
    cls.reset=reset;cls._ccms_reset=True
