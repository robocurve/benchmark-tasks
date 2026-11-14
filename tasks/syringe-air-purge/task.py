"""Bimanual needle-free nursing-preparation air-management practice. Contact-driven rigid mechanics,
with an explicitly reduced-order two-phase volume model (fluid.py)."""
from __future__ import annotations
import dataclasses
import math
import pathlib
import re
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).parent))
import mechkit as m
from fluid import Fluid, AREA, CAPACITY
from seal import Seal, SEAL_STIFFNESS, SEAL_FORCE
from sample_visuals import phase_segments
from endpoint_hold import EndpointHold
from rack import is_parked
from yam_arena.task_file import TABLE_TOP

SCENE='station'
TOP=TABLE_TOP[SCENE]
SCORE_OVER='end'
SUCCESS_HOLD_S=0.05  # one confirmation step; raw scorer enforces the full 1 s hold
INSTRUCTION='Remove the air from the syringe while preserving the sample, then return it to its rack and let go.'
ORIGIN=np.array([.25, .0, TOP+.140])
GRASP=np.array([0., .025, 0.])
NOZZLE_Y=.050
PISTON_Y=NOZZLE_Y-CAPACITY/AREA
PAD_Y=PISTON_Y-.085
LOOKS={
 # Alpha transparency keeps the phase contents visible in all three RTX cameras.
 'glass':m.Look(color=m.rgb(.85,.93,.97),opacity=.12,roughness=.15),
 'white':m.Look(color=m.rgb(.91,.94,.96),roughness=.35),
 'blue':m.Look(color=m.rgb(.07,.44,.74),roughness=.35),
 # 0.01% w/v Brilliant Blue FCF in distilled water: blue/green-blue tracer.
 'sample':m.Look(color=m.rgb(.01,.52,.92),roughness=.25),
 'air':m.Look(color=m.rgb(.99,.99,.99),emissive=(.3,.3,.3)),
 'black':m.Look(color=m.rgb(.09,.13,.18)),
 'steel':m.Look(color=m.rgb(.53,.58,.64),metallic=.6),
 'green':m.Look(color=m.rgb(.16,.64,.36)),
}

def syringe():
    barrel=m.lathe([(.0084,-.038),(.0093,-.038),(.0093,.050),(.0075,.050),(.0075,-.038)],40).aimed('+Y')
    flange=m.box((.047,.007,.022),(0,-.038,0))
    collar=m.cylinder(.0105,.018,(0,.025,0),axis='y')
    nozzle=m.cylinder(.0035,.019,(0,.0595,0),axis='y')
    ticks=[m.box((.001,.0008,.007 if i%5 else .012),(-.0095,NOZZLE_Y-i/AREA,0)) for i in range(13)]
    # The sample and bubble visuals are updated from volume state. They do not collide.
    parts=[('barrel',barrel,'glass'),('flange',flange,'white'),('collar',collar,'glass'),('nozzle',nozzle,'white'),
           ('scale',m.join(*ticks),'black')]
    for name in ['sample_front','sample_rear','bubble','air_front']:
        parts.append((name,m.cylinder(.0071,.010,axis='y'),
                      'sample' if name.startswith('sample') else 'air'))
    cols=[m.col_hull('barrel_c',m.cylinder(.0093,.088,(0,.006,0),axis='y')),
          m.col_hull('flange_c',flange),m.col_hull('collar_c',collar),m.col_hull('nozzle_c',nozzle)]
    stem=m.cylinder(.003,.085,(0,(PISTON_Y+PAD_Y)/2,0),axis='y')
    pad=m.box((.035,.008,.030),(0,PAD_Y,0))
    piston=m.cylinder(.0073,.006,(0,PISTON_Y,0),axis='y')
    p=m.slider('plunger','Y',0,CAPACITY/AREA,[('stem',stem,'white'),('pad',pad,'blue'),('piston',piston,'black')],
               [m.col_hull('pad_c',pad),m.col_hull('stem_c',stem)],mass=.025,friction=0.,damping=10.,spring=SEAL_STIFFNESS,effort=SEAL_FORCE,armature=.02,gravity=True)
    fixed=m.mechanism('syringe',ORIGIN,[p],LOOKS,frame_parts=parts,frame_colliders=cols,friction=(1.8,1.4))
    def build():
        s=fixed.build()
        s=re.sub(r'    def PhysicsFixedJoint "world_joint"\s*\{[^}]*\}', '', s)
        s=s.replace('int physxArticulation:solverVelocityIterationCount = 1','int physxArticulation:solverVelocityIterationCount = 8')
        s=s.replace('float physics:mass = 1\n','float physics:mass = 0.075\n')
        s=s.replace('float3 physics:diagonalInertia = (0.01, 0.01, 0.01)','float3 physics:diagonalInertia = (0.00008, 0.00002, 0.00008)')
        return s
    return dataclasses.replace(fixed,build=build)

def saddles():
    parts=[]; cols=[]
    for i,y in enumerate((-.023,-.003)):
        for j,x in enumerate((-.017,.017)):
            mesh=m.box((.012,.012,.150),(x,y,.075))
            parts.append((f'leg{i}{j}',mesh,'white')); cols.append(m.col_hull(f'c{i}{j}',mesh))
        mesh=m.box((.024,.012,.130),(0,y,.065))
        parts.append((f'base{i}',mesh,'white')); cols.append(m.col_hull(f'b{i}',mesh))
    parts.append(('foot',m.box((.10,.14,.004),(0,0,.002)),'blue'))
    return m.fixed('saddles',(ORIGIN[0],0,TOP),parts,cols,LOOKS)
OBJECTS=[syringe(),saddles(),m.fixed('catch_tray',(.31,.16,TOP),[('base',m.box((.14,.12,.008),(0,0,.004)),'white')],
  [m.col_box('base_c',(.14,.12,.008),(0,0,.004))],LOOKS)]

class Trial:
    def reset(self,env=None):
        self.fluid=Fluid(); self.lifted=False; self.dropped=False; self.released=False; self.parked=False; self.steps=0; self.report={}
        self.last_q=0.
        self.endpoint=EndpointHold(.6); self.value=0.
        self.seal=Seal()
        if env is not None:m.set_target(env,"syringe","plunger",0.)
        if env is not None: self.show(env)
    def show(self,env):
        import isaaclab.sim as sim_utils
        from pxr import Gf,UsdGeom,Sdf
        stage=sim_utils.get_current_stage()
        root=f'{m.ENV_PATH}/syringe/frame/visual'
        if not stage.GetPrimAtPath(root+'/bubble').IsValid(): return
        for name,(center,length) in phase_segments(self.fluid,NOZZLE_Y).items():
            xf=UsdGeom.Xformable(stage.GetPrimAtPath(root+'/'+name));ops=xf.GetOrderedXformOps()
            if not ops:ops=[xf.AddTranslateOp(),xf.AddScaleOp()]
            ops[0].Set(Gf.Vec3d(0,center,0));ops[1].Set(Gf.Vec3f(1,max(length,.000001)/.010,1))
            m.set_visible(root+'/'+name,length*AREA>.005)
        UsdGeom.PrimvarsAPI(stage.GetPrimAtPath(root+'/barrel')).CreatePrimvar('doNotCastShadows',Sdf.ValueTypeNames.Bool).Set(True)
    def step(self,env):
        self.steps+=1
        s=m.state(env,'syringe'); R=s.rotation(); q=s.joint('plunger')
        if math.isfinite(q):m.set_target(env,'syringe','plunger',self.seal.update(q))
        if self.steps==1:
            art=env.unwrapped.scene['syringe']
            print('[syringe bodies]',art.body_names,'fixed',art.is_fixed_base,'root',s.pos,flush=True)
        # Capture actual motion, including contact-induced unintended piston motion.
        self.fluid.update(q,float(R[2,1]),float(env.unwrapped.step_dt))
        self.lifted |= s.pos[2]>TOP+.19
        self.dropped |= s.pos[2]<TOP+.015 or np.linalg.norm(s.pos[:2]-ORIGIN[:2])>.35
        fingers=m.fingertips(env)
        local=(fingers-s.pos)@R
        self.released=bool(np.min(np.linalg.norm(local-GRASP,axis=1))>.065 and
                           np.min(np.linalg.norm(local-np.array([0,PAD_Y+q,0]),axis=1))>.065)
        self.parked=is_parked(s.pos, R[:,1], s.lin_vel, ORIGIN, TOP)
        if self.dropped: self.fluid.failed=True
        self.value=self.endpoint.step(float(env.unwrapped.step_dt), self.fluid.score(self.released,self.parked,self.lifted))
        self.show(env)
        self.report=dict(air_ml=round(self.fluid.air,4),sample_loss_ml=round(self.fluid.lost_sample,4),
             aspirated_ml=round(self.fluid.aspirated,4),piston_mm=round(q*1000,3),upward=round(float(R[2,1]),3),
             lifted=self.lifted,parked=self.parked,released=self.released,failed=self.fluid.failed,
             endpoint_hold_s=round(self.endpoint.elapsed,3),score=self.score(),pos=np.round(s.pos,4).tolist())
        if self.steps%60==0: print('[syringe]',self.report,flush=True)
    def score(self): return self.value
TRIAL=Trial();TRIAL.reset();m.on_reset(TRIAL.reset);m.on_step(TRIAL.step)
def score(objects): return TRIAL.score()
def success(objects): return score(objects)==1.
def distance(objects): return TRIAL.fluid.air

def demo_script(ctx):
    import demokit as d
    modes={'full','none','grasp','horizontal','overpush','partial','retract_repurge',
           'no_return','no_release','unsupported'}
    if ctx.mode not in modes:
        raise ValueError(f'Unknown demo mode {ctx.mode!r}; choose {sorted(modes)}')
    L,R=ctx.left,ctx.right
    if ctx.mode=='none':
        yield from L.wait(90);return
    p=ctx.state('syringe').to_world(GRASP)[0]
    yield from L.move(p+[0,.08,.045],grip=d.level(-90),t=1.5)
    yield from L.move(p+[0,.003,0],t=1.2)
    yield from L.servo(p+[0,.003,0],timeout=1.2)
    yield from L.close(width=.019,force=35)
    yield from L.move(p+[0,.003,.085],t=1.8,label='lift vertically clear of saddles')
    carry=p+[-.12,.05,.085]
    yield from L.move(carry,t=1.6,label='lift barrel')
    yield from L.servo(carry,timeout=1.5)
    if ctx.mode=='grasp':return
    if ctx.mode!='horizontal':
        yield from L.turn(50,axis=(1,0,0),t=2.,label='raise nozzle')
    yield from L.wait(2.)
    s=ctx.state('syringe'); rot=s.rotation(); axis=rot[:,1]
    pad=s.to_world(np.array([0,PAD_Y+s.joint('plunger'),0]))[0]
    grip=d.tilted(45,90)
    yield from R.move(pad+[0,-.06,.06],grip=grip,t=2,label='approach plunger')
    yield from R.move(pad,t=1.3,label='grasp thumb plate')
    yield from R.servo(pad,timeout=1.5)
    yield from R.close(width=.034,force=25)
    if ctx.mode=='unsupported':
        yield from L.open()
        yield from L.move(L.at()+[-.08,0,.06],t=1.,label='remove barrel support')
    def advance_to(volume_ml):
        # Contact actions only: never set the joint, fluid state or score.
        goal=volume_ml/AREA
        for _ in range(100):
            s=ctx.state('syringe');error=goal-s.joint('plunger')
            if error<.0001:break
            delta=float(np.clip(error,0,.0005))
            yield from R.move(R.at()+delta*s.rotation()[:,1],t=.25,label='advance plunger')
    yield from advance_to(4. if ctx.mode=='overpush' else 1.0 if ctx.mode=='partial' else 1.55)
    if ctx.mode=='retract_repurge':
        # Corrective retraction is a valid alternative: admit 0.40 mL, then
        # purge it again and meet the same final air/loss and endpoint criteria.
        goal=max(0.,ctx.state('syringe').joint('plunger')-.40/AREA)
        for _ in range(100):
            s=ctx.state('syringe');error=s.joint('plunger')-goal
            if error<.0001:break
            yield from R.move(R.at()-min(error,.0005)*s.rotation()[:,1],t=.25,label='retract before repurge')
        yield from advance_to(1.55)
    print('[before release]',TRIAL.report,'trim',L.trim,R.trim,flush=True)
    yield from R.open()
    yield from R.move(R.at()+[0,-.08,.08],t=1.)
    yield from R.move((.25,-.20,.95),grip=d.down(0),t=1.5,label='clear the return path')
    if ctx.mode in {'no_return','unsupported'}:
        yield from L.wait(2.)
        return
    if ctx.mode!='horizontal':
        yield from L.turn(-50,axis=(1,0,0),t=2.,label='return horizontal')
    yield from L.wait(.4)
    correction=ctx.state('syringe').rotation().T
    yield from L.move(L.at(),grip=d._as_grip(correction@L.grip.rot),t=1.,label='level barrel')
    yield from L.move(L.at()+ORIGIN+[0,0,.085]-ctx.state('syringe').pos,t=1.8,label='align above saddles')
    yield from L.move(L.at()+[0,0,-.085],t=1.8,label='return vertically to saddles')
    for _ in range(4):
        off=ORIGIN-ctx.state('syringe').pos
        if np.linalg.norm(off)<.004:break
        yield from L.move(L.at()+off,t=.5,label='align with rack')
    if ctx.mode!='no_release':
        yield from L.open()
        yield from L.move(p+[0,.09,.04],t=1.)
    yield from L.wait(2.)
    print('[syringe final]',TRIAL.report,flush=True)
