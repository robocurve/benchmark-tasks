"""Permanent-record attacks, all seeded dates and the readout/reset adapters."""
from pathlib import Path
from dataclasses import replace
import math
import re
import sys
import types
import time
from contextlib import redirect_stdout
from io import StringIO
import numpy as np
import pytest
from yam_arena.task_file import load

MODULE = next(p for p in Path(__file__).parent.glob('*.py')
              if not p.name.startswith('test_') and p.stem not in ('dater_visuals','yam_parts','yam_joints'))
T = load(MODULE).module


@pytest.fixture(autouse=True)
def isolate():
    saved = dict(T._RUN)
    T._RUN.update(env=None,t=None,step=0,episode=T._RUN['episode']+1)
    T.TEST_EVENTS,T.TEST_LAYOUT,T.TEST_INKED,T.TEST_GRIPPERS = None,None,None,None
    T.reset_history()
    yield
    T.TEST_EVENTS,T.TEST_LAYOUT,T.TEST_INKED,T.TEST_GRIPPERS = None,None,None,None
    T._RUN.clear(); T._RUN.update(saved); T.reset_history()


def state(index=0,origin=None,face=0.,**kwargs):
    return T.STAMP.offline_state(np.r_[T.CRADLE_XY,T.SEAT_ROOT] if origin is None else origin,
                                0,{**T.goal_joints(index),'face':face},**kwargs)


def grade(events,index=0,stamp=None,inked=True):
    T.TEST_LAYOUT,T.TEST_EVENTS,T.TEST_INKED = T.LAYOUTS[index],events,inked
    return T.score({'stamp':state(index) if stamp is None else stamp})


@pytest.mark.parametrize('index',range(16))
def test_every_layout_and_wrong_date_transfer(index):
    T.TEST_LAYOUT=T.LAYOUTS[index]
    history=T.PressHistory()
    history.update(state(index,origin=(*T.PAD_XY,T.PAD_TOP-.002),face=.002),(1,0))
    e=history.update(state(index,origin=(*T.BOX_XY,T.PAPER_TOP-.0025),face=.0025),(1,1))
    assert e is not None and (e['day'],e['month'])==T.LAYOUTS[index][2:]
    assert T.correct_impression(e)
    assert grade([e],index)==1.
    assert T.success({'stamp':state(index)})
    for other in range(16):
        if T.LAYOUTS[index][2:] != T.LAYOUTS[other][2:]:
            assert grade([e],other)==0.


@pytest.mark.parametrize('index',range(10),ids=[x[0] for x in T.cheat_cases()])
def test_all_requested_permanent_errors(index):
    assert grade(T.cheat_cases()[index][1])==0.


@pytest.mark.parametrize('index',range(16))
def test_untouched_start_and_current_band_credit(index):
    T.TEST_LAYOUT=T.LAYOUTS[index]
    start=T.STAMP.offline_state(T.STAMP.pos,0,{})
    assert grade([],index,start,False)==0.
    assert grade([],index,inked=False)==.3
    q=T.goal_joints(index); q['month']=math.radians(30) if q['month']==0 else 0.
    assert grade([],index,T.STAMP.offline_state(T.STAMP.pos,0,q),False)==.15
    assert grade([],index,start,False)==0.  # No remembered partial credit.


def test_current_seat_release_and_motion_tiers():
    e=[T.nominal_event(0)]
    lying=state(origin=(*T.BOX_XY,T.TOP+.060),R=np.array([[0,0,1],[0,1,0],[-1,0,0]]))
    assert grade(e,stamp=lying)==.7
    for s in (replace(state(),lin_vel=np.array([.2,0,0])),
              replace(state(),ang_vel=np.array([0,0,.06])),
              state(qd={'day_plunger':.1})):
        assert grade(e,stamp=s)==.9
    T.TEST_GRIPPERS={'right':(state().origin+T.HANDLE,1.)}
    assert grade(e)==.9  # Open but still close.
    T.TEST_GRIPPERS={'left':(state().origin+T.HANDLE,0.),'right':(np.r_[T.PARK_XY,T.TRAVEL_TCP],1.)}
    assert grade(e)==.9
    T.TEST_GRIPPERS=None
    assert grade(e)==1.
    assert grade(e,stamp=state(origin=state().origin+[.006,0,0]))==1.
    assert grade(e,stamp=state(origin=state().origin+[.020,0,0]))==.7
    assert grade(e,stamp=state(origin=state().origin+[0,0,.004]))==.7
    assert grade(e,stamp=state(origin=(.8,-.8,T.TOP-.1)))==.7
    # After a correct permanent record the bands may change.
    assert grade(e,stamp=T.STAMP.offline_state(state().origin,0,{}))==1.


@pytest.mark.parametrize('name',('day','month'))
@pytest.mark.parametrize('offset,expected',((0.,1.),(1.,1.),(1.01,.9),(2.,.9),(-2.,.9)))
def test_knob_detent_stillness_ignores_solver_velocity(name,offset,expected):
    q=T.goal_joints(0); q[name]+=math.radians(offset)
    stamp=T.STAMP.offline_state(state().origin,0,q,qd={'day':.25,'month':-.18})
    assert grade([T.nominal_event(0)],stamp=stamp)==expected


@pytest.mark.parametrize('name',('face','day_plunger','month_plunger'))
@pytest.mark.parametrize('velocity',(-.1,.05,.1))
def test_moving_slide_limits_seated_released_stamp_to_point_nine(name,velocity):
    assert grade([T.nominal_event(0)],stamp=state(qd={name:velocity}))==.9


@pytest.mark.parametrize('motion',('knob','plunger'))
def test_detent_or_slide_motion_interrupts_continuous_hold(motion):
    good=[T.nominal_event(0)]
    q=T.goal_joints(0);q['day']+=math.radians(2.)
    moving=(T.STAMP.offline_state(state().origin,0,q) if motion=='knob'
            else state(qd={'day_plunger':.1}))
    resting=state(qd={'day':.25,'month':-.18})
    for step in range(42):
        T._RUN.update(t=step*.05,step=step)
        stamp=moving if step==20 else resting
        expected=1. if step==41 else .9
        assert grade(good,stamp=stamp)==expected


def test_sticky_error_restore_and_timer_interruptions():
    T._RUN.update(t=0.,step=0)
    good=[T.nominal_event(0)]
    for step in range(24):
        T._RUN.update(t=step*.05,step=step)
        expected=.9 if step<20 else 1.
        assert grade(good)==expected and grade(good)==expected
    T._RUN.update(t=1.2,step=24)
    assert grade(good,stamp=replace(state(),lin_vel=np.array([.2,0,0])))==.9
    T._RUN.update(t=1.25,step=25); assert grade(good)==.9
    T._RUN.update(t=3.,step=60); assert grade(good)==.9  # Skipped calls reset timer.
    T._RUN.update(t=3.05,step=61); assert grade(T.cheat_cases()[0][1])==0.
    T._RUN.update(t=4.,step=80); assert grade(good)==0.
    T._RUN.update(t=None,episode=T._RUN['episode']+1); assert grade(good)==1.


def test_pose_only_inking_rearming_and_second_table_press():
    T.TEST_LAYOUT=T.LAYOUTS[0]
    h=T.PressHistory()
    def sample(xy,top,q,zdelta=0.,**kw): return state(origin=(*xy,top-q+zdelta),face=q,**kw)
    token=0
    def tick(s):
        nonlocal token
        token+=1
        return h.update(s,(1,token))
    dry=sample(T.BOX_XY,T.PAPER_TOP,.0025)
    assert tick(dry) is None and not h.events
    assert tick(sample(T.PAD_XY,T.PAD_TOP,.002)) is None and h.inked
    e=tick(dry); assert e is not None and T.correct_impression(e)
    assert h.update(dry,(1,token)) is None and len(h.events)==1
    tick(sample(T.BOX_XY,T.PAPER_TOP,.0004,zdelta=.0029))
    assert tick(dry) is None  # Below 0.5 mm but not risen 3 mm.
    tick(sample(T.BOX_XY,T.PAPER_TOP,.0006,zdelta=.004))
    assert tick(dry) is None  # Risen but not below 0.5 mm.
    tick(sample(T.BOX_XY,T.PAPER_TOP,.0004,zdelta=.003))
    assert tick(sample(T.DOC_XY+[.100,0],T.TOP,.002)) is not None
    assert grade(h.events)==0.
    h.update(state(),(2,0)); assert not h.inked and not h.events


@pytest.mark.parametrize('dx,yaw_deg,smeared',((0.,0.,False),(.0019,0.,False),(.0021,0.,True),
                                               (0.,1.9,False),(0.,2.1,True)))
def test_smear_after_impression_is_sticky_zero(dx,yaw_deg,smeared,monkeypatch):
    T.TEST_LAYOUT=T.LAYOUTS[0]
    h,ep=T.PressHistory(),T._RUN['episode']
    a=math.radians(yaw_deg); Rz=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
    h.update(state(origin=(*T.PAD_XY,T.PAD_TOP-.002),face=.002),(ep,10))
    assert h.update(state(origin=(*T.BOX_XY,T.PAPER_TOP-.0025),face=.0025),(ep,11)) is not None
    h.update(state(origin=(*T.BOX_XY,T.PAPER_TOP-.0004+.001),face=.0004),(ep,12))  # bounce, no re-arm
    assert h.update(state(origin=(T.BOX_XY[0]+dx,T.BOX_XY[1],T.PAPER_TOP-.002),face=.002,R=Rz),(ep,13)) is None
    assert len(h.events)==1 and h.smeared==smeared
    monkeypatch.setattr(T,'_SCORE_HISTORY',h)
    assert grade(h.events)==(0. if smeared else 1.)


def edge_press(xy,top,tilt_deg,axis='y',q=.002):
    """Inked face pressed edge-first: its lowest bottom corner on `top`, face centre above `xy`."""
    a=math.radians(tilt_deg); c,s=math.cos(a),math.sin(a)
    R=(np.array([[c,0,s],[0,1,0],[-s,0,c]]) if axis=='y' else np.array([[1,0,0],[0,c,-s],[0,s,c]]))
    corner=min(T.FACE_CORNERS,key=lambda k:(R@k)[2])
    root=np.r_[xy,top]-R@(corner+[0,0,q])
    root[:2]+=np.asarray(xy)-(root+R@np.array([0,0,q]))[:2]  # centre the face over xy
    return state(origin=root,face=q,R=R)


@pytest.mark.parametrize('where',('paper','table'))
@pytest.mark.parametrize('order',('after','before'))
def test_tilted_edge_press_is_an_impression_and_scores_zero(where,order,monkeypatch):
    T.TEST_LAYOUT=T.LAYOUTS[0]
    h,ep=T.PressHistory(),T._RUN['episode']
    xy=T.BOX_XY if where=='paper' else T.DOC_XY+[.110,0]
    top=T.PAPER_TOP if where=='paper' else T.TOP
    good=state(origin=(*T.BOX_XY,T.PAPER_TOP-.0025),face=.0025)
    edge=edge_press(xy,top,8.)
    p,_,tilt=T.face_pose(edge)
    assert p[2]-top > T.SURFACE_Z and tilt==pytest.approx(8.)  # centre alone would miss it
    h.update(state(origin=(*T.PAD_XY,T.PAD_TOP-.002),face=.002),(ep,10))
    first,second=(good,edge) if order=='after' else (edge,good)
    assert h.update(first,(ep,11)) is not None
    h.update(state(origin=(*T.BOX_XY,T.PAPER_TOP+.004)),(ep,12))  # lift and re-arm
    assert h.update(second,(ep,13)) is not None and len(h.events)==2
    monkeypatch.setattr(T,'_SCORE_HISTORY',h)
    assert grade(h.events)==0.
    assert grade(h.events[1:] if order=='after' else h.events[:1])==0.  # the edge mark alone fails


@pytest.mark.parametrize('tilt,axis',((5.5,'y'),(7.,'y'),(13.,'x')))
def test_tilted_edge_press_alone_is_never_ignored(tilt,axis):
    T.TEST_LAYOUT=T.LAYOUTS[0]
    h,ep=T.PressHistory(),T._RUN['episode']
    h.update(state(origin=(*T.PAD_XY,T.PAD_TOP-.002),face=.002),(ep,1))
    e=h.update(edge_press(T.BOX_XY,T.PAPER_TOP,tilt,axis),(ep,2))
    assert e is not None and not T.correct_impression(e)


def test_ink_thresholds_blank_band_wrapping_and_bad_tilt():
    T.TEST_LAYOUT=T.LAYOUTS[0]
    for q,delta,tilt,want in ((.0009,0,0,False),(.001,0,0,True),(.002,.0021,0,False),
                               (.002,0,11,False),(.002,0,10,True)):
        a=math.radians(tilt); R=np.array([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])
        # Face point is rotated about its joint anchor, so account for its offset.
        root=np.r_[T.PAD_XY,T.PAD_TOP+delta]-R@np.array([0,0,q])
        s=state(origin=root,face=q,R=R); h=T.PressHistory(); h.update(s,(1,0))
        assert h.inked==want
    q=T.goal_joints(0)
    for extra,want in ((4.,5),(4.01,None),(6.,None)):
        s=T.STAMP.offline_state(T.STAMP.pos,0,{**q,'day':q['day']+math.radians(extra)})
        assert T.band_reading(s,'day')==want
    # Modulo reads the same number across band ends in either direction.
    for name,count in (('day',31),('month',12)):
        for detent in (-10,-1,0,1,10):
            s=T.STAMP.offline_state(T.STAMP.pos,0,{name:detent*2*math.pi/count})
            i=0 if name=='day' else 1
            assert T.band_reading(s,name)==(T.LAYOUTS[0][i]-1+detent)%count+1


def test_fixed_duration_lazy_yaw_after_reset():
    T.TEST_LAYOUT=T.LAYOUTS[0]; plan=T.demo()['right']; times=[w.duration for w in plan]
    for i,l in enumerate(T.LAYOUTS):
        T.TEST_LAYOUT=l
        assert [w.duration for w in plan]==times
        for j,name in enumerate(('day','month')):
            w=next(w for w in plan if w.label==name+' twist')
            assert math.degrees(w.yaw)==pytest.approx(T.changes(l)[j]*360/(31 if j==0 else 12))
    assert sum(times)+T.SUCCESS_HOLD_S<.6*T.RUN_LIMIT_STEPS/20


def recording_adapter():
    """A USD recording surface with no physics APIs; suitable for CPU auditing."""
    from types import SimpleNamespace as NS
    writes,prims=[],{}
    class Attr:
        def __init__(self,p,n): self.p,self.n=p,n
        def Set(self,x): writes.append((self.p,self.n,x)); prims[self.p].values[self.n]=x
        def SetTargets(self,x): self.Set(x)
        def ConnectToSource(self,*x): self.Set(x)
    class Prim:
        def __init__(self,p,k): self.path,self.kind,self.values=p,k,{}
        def GetPrim(self): return self
        def GetPath(self): return self.path
        def GetStage(self): return stage
        def ConnectableAPI(self): return self
        def Bind(self,mat): pass
        def CreateInput(self,n,t): return Attr(self.path,'input:'+n)
        def __getattr__(self,n):
            if n.startswith(('Create','Add')): return lambda *a:Attr(self.path,n)
            if n=='GetInvisibleIdsAttr': return lambda:Attr(self.path,'CreateInvisibleIdsAttr')
            if n=='GetVisibilityAttr': return lambda:Attr(self.path,'CreateVisibilityAttr')
            raise AttributeError(n)
    def define(stage,p,k):
        assert isinstance(p,str) and p.startswith('/'), f'Non-absolute USD prim path: {p!r}'
        assert all(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*',element)
                   for element in p[1:].split('/')), f'Invalid USD prim path: {p!r}'
        stage.define_paths.append(p)
        return prims.setdefault(p,Prim(p,k))
    def schema(k): return NS(Define=lambda stage,p:define(stage,p,k))
    geom=NS(**{k:schema(k) for k in ('Xform','Cube','Cylinder','PointInstancer')},
            Imageable=lambda p:p,Tokens=NS(invisible='invisible',inherited='inherited'))
    pxr=types.ModuleType('pxr'); pxr.Gf=NS(Vec3f=lambda *a:a,Vec3d=lambda *a:a)
    pxr.Sdf=NS(ValueTypeNames=NS(Color3f=0,Float=1)); pxr.UsdGeom=geom
    pxr.UsdShade=NS(Material=schema('Material'),Shader=schema('Shader'),MaterialBindingAPI=NS(Apply=lambda p:p))
    array=NS(FromNumpy=lambda x:x.copy())
    pxr.Vt=NS(IntArray=array,Int64Array=array,Vec3fArray=array)
    stage=NS(GetPrimAtPath=lambda p:prims[p],define_paths=[])
    return pxr,stage,prims,writes,Prim


def test_recording_adapter_rejects_invalid_prim_paths():
    pxr,stage,*_=recording_adapter()
    for schema in (pxr.UsdGeom.Xform,pxr.UsdGeom.Cube,pxr.UsdGeom.Cylinder,
                   pxr.UsdGeom.PointInstancer,pxr.UsdShade.Material,pxr.UsdShade.Shader):
        for path in ('stamp/face/die_dot','','/stamp/face/die_-1_.','/sheet/Dater/order_-1_.',
                     '/stamp//dot','/stamp/1dot','/stamp/dot/'):
            with pytest.raises(AssertionError): schema.Define(stage,path)
        schema.Define(stage,'/stamp/face/die_dot')
    assert len(stage.define_paths)==6


def test_visibility_only_renderer_seed_reset_and_scene_hook(monkeypatch):
    import dater_visuals as v
    import yam_joints
    from types import SimpleNamespace as NS
    pxr,stage,prims,writes,Prim=recording_adapter()
    monkeypatch.setitem(sys.modules,'pxr',pxr)
    v.install(vars(T)); v._ROOTS.clear(); v._BANDS.clear()
    start=time.perf_counter()
    v.spawn_stamp(stage,'/stamp'); v.spawn_document(stage,'/sheet')
    assert v._BANDS['/stamp']['values']==T.current_layout()[:2]
    elapsed=time.perf_counter()-start
    count=len(prims); rec=v._ROOTS['/sheet']
    assert len(rec['grid'])==30996 and not rec['visible'].any()
    assert sum(p.kind=='PointInstancer' for p in prims.values())==1
    writes.clear()
    T.TEST_LAYOUT=T.LAYOUTS[0]; T._RUN.update(t=.05,step=1)
    v.update(state()); assert v._BANDS['/stamp']['values']==(5,1)
    T._RUN['step']+=1
    v.update(state(origin=(*T.PAD_XY,T.PAD_TOP-.002),face=.002))
    T._RUN['step']+=1
    with redirect_stdout(StringIO()): v.update(state(origin=(*T.BOX_XY,T.PAPER_TOP-.0025),face=.0025))
    dots=int(rec['visible'].sum()); assert dots>100
    assert len(prims)==count and all(n in ('CreateVisibilityAttr','CreateInvisibleIdsAttr') for _,n,_ in writes)
    before=rec['visible'].copy(); v.update(state()); np.testing.assert_array_equal(before,rec['visible'])
    assert v._HISTORY is not T._SCORE_HISTORY
    for b in range(16):
        T.TEST_LAYOUT=T.LAYOUTS[b]; v.show_order()
    T.TEST_LAYOUT=None
    sim=types.ModuleType('isaaclab.sim.utils'); sim.clone=lambda fn:fn
    ie=types.ModuleType('yam_arena.inspect_embodiment')
    class Embodiment:
        def reset(self,scene=None,*,seed=None):
            assert T.current_layout()==T.LAYOUTS[0 if seed is None else seed%16]
            assert not rec['visible'].any() and not T._SCORE_HISTORY.events
            l=T.current_layout(); text=f'{l[2]:02d}{l[3]:02d}'
            for d,s,image in rec['prints']:
                want=d==-1 or s in T.DIGITS[int(text[d])]
                assert image.values['CreateVisibilityAttr']==('inherited' if want else 'invisible')
            return 'first observation'
    ie.YamArenaEmbodiment=Embodiment
    inner_reset=Embodiment.reset
    def release_wrapper(self,*args,**kwargs):
        return inner_reset(self,*args,**kwargs)
    Embodiment.reset=release_wrapper  # The real release block hides the signature.
    for key,value in {'isaaclab':types.ModuleType('isaaclab'),'isaaclab.sim':types.ModuleType('isaaclab.sim'),
                      'isaaclab.sim.utils':sim,'yam_arena.inspect_embodiment':ie}.items():
        monkeypatch.setitem(sys.modules,key,value)
    monkeypatch.setattr(yam_joints,'_patch_scene',lambda scene:None)
    configs=[]
    def factory(spec):
        original=lambda path,*a,**kw:prims.setdefault(path,Prim(path,'Object'))
        cfg=NS(spawn=NS(func=original),init_state=NS(joint_pos={'day_joint':0.,'month_joint':0.}),
               actuators={})
        configs.append((cfg,cfg.init_state,cfg.init_state.joint_pos))
        def rebuild(): raise AssertionError('Existing Arena object config must be preserved')
        return NS(spawner_cfg=NS(func=original),object_cfg=cfg,_init_object_cfg=rebuild)
    scene=NS(task_object=factory,object_states=lambda env,names:{'stamp':state()})
    v._patch_scene(scene); hook=scene.object_states; v._patch_scene(scene); assert scene.object_states is hook
    for spec,root in ((T.STAMP,'/hook_stamp'),(T.WORK_ORDER,'/hook_sheet')):
        obj=scene.task_object(spec)
        cfg,init_state,joint_pos=configs[-1]
        assert obj.object_cfg is cfg and cfg.init_state is init_state and cfg.init_state.joint_pos is joint_pos
        assert obj.spawner_cfg.func is cfg.spawn.func
        cfg.spawn.func(root,None)
    assert '/hook_stamp' in v._BANDS and '/hook_sheet' in v._ROOTS
    v.install(vars(T))
    plan=T.demo()['right']  # Construct BEFORE the seed selection.
    for seed in (None,0,1,7,15,16,31,-1):
        assert Embodiment().reset(seed=seed)=='first observation'
        j=0 if seed is None else seed%16
        assert next(w for w in plan if w.label=='day twist').yaw==pytest.approx(T.goal_joints(j)['day'])
    T._CURRENT_LAYOUT=T.LAYOUTS[0]
    v.reset_visuals()
    print(f'PRIM PATHS: {len(stage.define_paths)} Define paths checked; all absolute with valid USD identifiers; old decimal names rejected')
    print(f'RENDERER recording adapter: dots={len(rec["grid"])} nominal_visible={dots} author_seconds={elapsed:.6f}; runtime visibility only; seed/reset hooks PASS')
