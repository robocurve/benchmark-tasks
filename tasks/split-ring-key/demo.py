"""Robot-only fixed-waypoint controller and evidence runner.

Controller inputs: setup dimensions, elapsed action index and YAM inverse
kinematics. No object pose, scorer metric or segmentation is read by script().
The runner records privileged geometry separately for auditable diagnostics.
"""
from pathlib import Path
import argparse, hashlib, json, math, os, time, traceback
import numpy as np
import geometry as G

HERE=Path(__file__).resolve().parent
HZ=20

class Arm:
    def __init__(self,side):
        from yam_arena import ik,station
        from yam_arena.constants import READY
        self.side=side; self.base=np.array(station.BASES[side]); self.q=np.array(READY,float); self.home_q=self.q.copy(); self.grip=1.
        self.pos,self.rot=ik.tcp_pose(self.q,self.base); self.home=(self.pos.copy(),self.rot.copy()); self.worst_mm=0.; self.worst_rot_deg=0.
    def aim(self,p,r):
        from yam_arena import ik
        q,err,rot_err=ik.solve(p,ik.matrix_to_quat_xyzw(r),q_init=self.q,base_pos=self.base,iters=80)
        self.q=q; self.pos=np.array(p,float); self.rot=np.array(r,float); self.worst_mm=max(self.worst_mm,err*1000); self.worst_rot_deg=max(self.worst_rot_deg,math.degrees(rot_err))
    def move(self,p,r=None,seconds=1.,grip=None):
        from yam_arena import ik
        p=np.array(p,float); r=self.rot if r is None else np.array(r,float)
        p0,r0,g0=self.pos.copy(),self.rot.copy(),self.grip; g1=g0 if grip is None else grip
        for k in range(1,max(1,round(seconds*HZ))+1):
            s=ik.smoothstep(k/max(1,round(seconds*HZ)))
            self.grip=g0+s*(g1-g0); self.aim(p0+s*(p-p0),ik.slerp_matrix(r0,r,s)); yield
    def return_home(self,seconds=2.):
        from yam_arena import ik
        q0=self.q.copy()
        for k in range(1,round(seconds*HZ)+1):
            u=ik.smoothstep(k/round(seconds*HZ));self.q=q0+u*(self.home_q-q0)
            self.pos,self.rot=ik.tcp_pose(self.q,self.base);yield
    def action(self): return np.r_[self.q,self.grip]

def wait(t):
    for _ in range(round(t*HZ)): yield

def together(*gens):
    active=list(map(iter,gens))
    while active:
        for g in active[:]:
            try: next(g)
            except StopIteration: active.remove(g)
        if active: yield

HANDLE_ROT=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
CRANK_ROT=HANDLE_ROT.copy()

def script(left,right,variant,stage):
    """Fixed authored geometry and IK only. No environment or scorer input."""
    if variant=='crank_only':
        stage['name']='crank_fixture_diagnostic'
        knob=G.crank_point(0.)
        yield from left.move(knob+[-.07,0,0],CRANK_ROT,3.)
        yield from left.move(knob,CRANK_ROT,2.)
        yield from left.move(knob,CRANK_ROT,1.,grip=0.)
        for q in np.linspace(0,math.radians(360),260):
            left.aim(G.crank_point(q),CRANK_ROT);yield
        yield from wait(2.)
        yield from left.move(left.pos,seconds=1.,grip=1.)
        yield from left.move(left.pos+[-.10,0,0],seconds=2.)
        yield from wait(4.);return
    if variant=='idle':
        stage['name']='idle';yield from wait(12.);return
    a=G.KEY_START_ANGLE;hole=G.KEY_START.copy();rr=G.grasp_rotation(a);grip=G.grasp_point(hole,a)
    stage['name']='grasp_key'
    yield from right.move(grip+[-.08,0,0],rr,3.)
    yield from right.move(grip,rr,2.)
    yield from right.move(grip,rr,1.,grip=0.)
    yield from wait(.5)
    stage['name']='lift_key'
    yield from right.move(grip+[0,0,.04],rr,2.)
    aa=G.OPENER_ANGLE;zz=G.PITCH*((aa+2*math.pi)/(2*math.pi)-.5)+G.PITCH/2
    button=G.CENTER+G.W@np.array([(G.R+G.OPENER_HANDLE+G.OPENER_RETRACT)*math.cos(aa),(G.R+G.OPENER_HANDLE+G.OPENER_RETRACT)*math.sin(aa),zz+G.OPENER_GRIP_Z])+[0,0,G.OPENER_LIFT]
    inward=G.W@np.array([-G.OPENER_RETRACT*math.cos(aa),-G.OPENER_RETRACT*math.sin(aa),0.])
    if variant!='no_open':
        stage['name']='open_spring_tip'
        yield from left.move(button+[-.06,0,0],HANDLE_ROT,3.)
        yield from left.move(button,HANDLE_ROT,2.)
        yield from left.move(button,HANDLE_ROT,1.,grip=0.)
        inward=G.W@np.array([-G.OPENER_RETRACT*math.cos(aa),-G.OPENER_RETRACT*math.sin(aa),0.])
        yield from left.move(button+inward,HANDLE_ROT,4.)
        yield from left.move(button+inward+[-.0048,0,0],HANDLE_ROT,2.)
        yield from wait(.5)
    if variant=='direct':
        stage['name']='direct_after_opening'
        a=math.radians(-100);rr=G.grasp_rotation(math.radians(-60));hole=G.hole_target(a,.0004)
        yield from right.move(G.grasp_point(hole+[-.04,0,0],a),rr,3.)
        yield from right.move(G.grasp_point(hole,a),rr,4.)
        yield from wait(1.)
        stage['name']='withdraw_after_direct_attempt'
        yield from left.move(left.pos-inward,HANDLE_ROT,4.)
        yield from left.move(left.pos,seconds=.7,grip=1.)
        yield from left.move(left.pos+[0,.06,.035],seconds=1.5)
        yield from left.move(left.pos+[-.09,0,0],seconds=1.5)
        yield from right.move(right.pos,seconds=1.,grip=1.)
        yield from right.move(right.pos+[-.12,0,0],G.grasp_rotation(G.KEY_ANGLE),seconds=2.)
        yield from wait(6.);return
    a=math.radians(-4);z=.0072
    offset=.005 if variant=='wrong_hole'else 0.
    def start_hole(a):return G.hole_target(a,z)+G.W@np.array([offset*math.cos(a),offset*math.sin(a),0.])
    stage['name']='approach_tip'
    yield from right.move(G.grasp_point(start_hole(a)+[-.025,0,0],a),G.grasp_rotation(a),3.)
    yield from right.move(G.grasp_point(start_hole(a),a),G.grasp_rotation(a),2.)
    stage['name']='engage_tip'
    for a in np.linspace(math.radians(-4),G.KEY_ANGLE,80):
        hole=start_hole(a)
        right.aim(G.grasp_point(hole,a),G.grasp_rotation(a));yield
    if variant!='no_open':
        stage['name']='withdraw_splitter'
        yield from left.move(left.pos-inward,HANDLE_ROT,4.)
        stage['name']='release_splitter'
        yield from left.move(left.pos,seconds=.7,grip=1.)
        yield from left.move(left.pos+[0,.06,.035],seconds=1.5)
        yield from left.move(left.pos+[-.09,0,0],seconds=1.5)
    if variant in ('no_open','wrong_hole'):
        stage['name']='release_failed_start'
        yield from right.move(right.pos,seconds=1.,grip=1.)
        yield from right.move(right.pos+[-.12,0,0],seconds=2.)
        yield from wait(6.);return
    stage['name']='advance_upper_strand'
    for a in np.linspace(G.KEY_ANGLE,math.radians(-130),140):
        right.aim(G.grasp_point(G.hole_target(a,z),a),G.grasp_rotation(a));yield
    stage['name']='support_key_outward'
    from scipy.spatial.transform import Rotation
    def supported_key(angle,tilt,axial):
        kr=G.key_rotation(angle)@Rotation.from_euler('y',tilt).as_matrix()
        right.aim(G.hole_target(angle,axial)+kr@G.GRIP,np.stack([-kr[:,1],kr[:,2],-kr[:,0]],1))
    for u in np.linspace(0,1,120):
        supported_key(math.radians(-130-50*u),math.radians(40*u),z);yield
    if variant=='partial':
        stage['name']='partial_supported_one_turn';yield from wait(6.);return
    stage['name']='grasp_handwheel'
    yield from left.move(np.array([left.pos[0],left.pos[1],1.0]),CRANK_ROT,2.)
    knob=G.crank_point(0.)
    yield from left.move(knob+[-.07,0,0],CRANK_ROT,2.)
    yield from left.move(knob,CRANK_ROT,2.)
    yield from left.move(knob,CRANK_ROT,1.,grip=0.)
    # Four visible knobs permit real regrasping within the clear lower-left
    # quadrant. No rotor joint target or object state enters this controller.
    for index,span in enumerate((90.,90.,60.)):
        stage['name']=f'feed_spiral_quadrant_{index+1}'
        for local in np.linspace(0,span,round(span/12*HZ)):
            total=index*90+local
            left.aim(G.crank_point(math.radians(local)),CRANK_ROT)
            u=np.clip((total-182)/10,0,1)
            supported_key(math.radians(-180),math.radians(40),z+u*(.0015-z));yield
        yield from wait(.5)
        stage['name']=f'regrasp_handwheel_{index+1}'
        left.grip=1.;yield from wait(.35)
        yield from left.move(np.array([left.pos[0],knob[1],left.pos[2]]),CRANK_ROT,1.5)
        yield from left.move(left.pos+[-.08,0,0],CRANK_ROT,1.5)
        if index<2:
            yield from left.move(knob+[-.08,0,0],CRANK_ROT,1.5)
            yield from left.move(knob,CRANK_ROT,1.5)
            yield from left.move(knob,CRANK_ROT,1.,grip=0.)
    stage['name']='withdraw_crank_hand'
    yield from left.return_home(2.)
    yield from wait(1.)
    stage['name']='release_completed_key'
    right.grip=1.;yield from wait(.35)
    yield from right.move(right.pos-.10*right.rot[:,2],seconds=2.)
    yield from right.return_home(2.)
    stage['name']='settled_retention'
    yield from wait(10.)

def capture_sources():
    return {p.relative_to(HERE).as_posix():{'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size}
            for p in sorted(HERE.rglob('*')) if p.is_file() and '__pycache__' not in str(p) and p.suffix in ('.py','.usda','.json')}

def run(args):
    out=Path(args.out).resolve()
    if out.exists() and any(out.iterdir()): raise ValueError('Require a fresh output directory')
    out.mkdir(parents=True,exist_ok=True); start=time.monotonic()
    manifest={'source_before':capture_sources(),'physical_gpu':os.environ.get('ROBOCURVE_GPU'),
              'controller_observation_contract':'Fixed setup waypoints and robot IK only; no object-state feedback',
              'variant':args.variant,'seed':0,'control_hz':HZ}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    result={'success':False,'task_score':None,'steps':0,'variant':args.variant}; writers={}; emb=None
    try:
        from inspect_robots import Scene
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack
        from yam_arena.inspect_embodiment import yam_arena_embodiment
        from PIL import Image
        import imageio.v2 as imageio
        emb=yam_arena_embodiment(task=str(HERE/'task.py'),object_jitter=0,control_hz=HZ,cam_width=args.width,cam_height=args.height)
        obs=emb.reset(Scene(id='split-ring-key',instruction=emb.task.module.INSTRUCTION),seed=0)
        task=emb.task.module
        from witness import WitnessCamera
        witness=WitnessCamera(emb._base)
        task.monitor(emb._env)
        asset=emb._base.scene['ring']
        manifest['runtime']={'physics_dt_s':float(emb._base.sim.get_physics_dt()),
            'control_dt_s':float(emb._base.step_dt),'physics_substeps_per_action':int(emb._base.cfg.decimation),
            'joint_stiffness':dict(zip(asset.joint_names,asset.data.joint_stiffness[0].detach().cpu().tolist())),
            'joint_damping':dict(zip(asset.joint_names,asset.data.joint_damping[0].detach().cpu().tolist()))}
        # Read-only diagnostics of the actually spawned collision attributes.
        import omni.usd
        from pxr import UsdPhysics
        offsets={name:[] for name in ('ring','key','stand')}
        for prim in omni.usd.get_context().get_stage().Traverse():
            if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
            parts=str(prim.GetPath()).split('/')
            for name in offsets:
                if name in parts:
                    co=prim.GetAttribute('physxCollision:contactOffset').Get()
                    ro=prim.GetAttribute('physxCollision:restOffset').Get()
                    if co is not None and ro is not None:offsets[name].append((float(co),float(ro)))
        manifest['runtime']['collision_offsets']={name:{'shapes':len(values),
            'contact_offset_m':sorted(set(x[0]for x in values)),
            'rest_offset_m':sorted(set(x[1]for x in values))}for name,values in offsets.items()}
        assert all(values and all(abs(co-G.CONTACT_OFFSET)<1e-9 and abs(ro-G.REST_OFFSET)<1e-9 for co,ro in values)for values in offsets.values()),'Spawned contact offsets differ from declared CAD'
        result['initial']=dict(task.STATE)
        print('RESET_METRICS',json.dumps(task.STATE),flush=True)
        left,right=Arm('left'),Arm('right'); stage={'name':'initial'}
        gen=script(left,right,args.variant,stage)
        for name,img in obs.images.items(): Image.fromarray(np.asarray(img)).save(out/f'setup_{name}.png')
        Image.fromarray(witness.read()).save(out/'setup_witness_cam.png')
        Image.fromarray(witness.setup_key_detail()).save(out/'setup_key_detail.png')
        def record(obs):
            if args.no_record:return
            for name,img in {**obs.images,'witness_cam':witness.read()}.items():
                if name not in writers:
                    writers[name]=imageio.get_writer(out/f'{name}.mp4',fps=HZ,codec='libx264',macro_block_size=1,
                                                     output_params=['-pix_fmt','yuv420p','-movflags','+faststart'])
                writers[name].append_data(np.asarray(img))
        record(obs); ended=None; info={}
        original_observation=emb._observation
        if args.no_record:
            from inspect_robots.types import Observation
            emb._base.render_enabled=False
            manager=emb._base.observation_manager
            def state_only(update_history=False):
                observations={'policy':manager.compute_group('policy',update_history=update_history)}
                manager._obs_buffer=observations
                return observations
            manager.compute=state_only
            emb._observation=lambda:Observation(images={},state={},instruction=emb._instruction)
        def fresh_observation():
            for _ in range(3):emb._base.sim.render()
            for name in ('top_cam','left_cam','right_cam'):emb._base.scene[name].update(0.,force_recompute=True)
            return original_observation()
        with (out/'trace.jsonl').open('w') as trace, (out/'actions.jsonl').open('w') as acts:
            for step in range(1,args.max_steps+1):
                try:next(gen)
                except StopIteration:ended=step if ended is None else ended
                action=Action(data=pack(left.action(),right.action()))
                acts.write(json.dumps({'step':step,'left':left.action().tolist(),'right':right.action().tolist()})+'\n')
                res=emb.step(action); info=res.info; record(res.observation)
                metrics=dict(task.STATE)
                if not np.isfinite(metrics.get('splitter_m',float('nan'))): raise RuntimeError('Nonfinite articulation state; failed physics probe')
                trace.write(json.dumps({'step':step,'stage':stage['name'],'score':info.get('score'),**metrics})+'\n');trace.flush()
                result.update(steps=step,task_score=float(info.get('score') or 0.),final=metrics)
                if step%40==0:
                    print(json.dumps({'step':step,'stage':stage['name'],'score':info.get('score'),'metrics':metrics}),flush=True)
                if args.snapshots and step%40==0:
                    snap=fresh_observation() if args.no_record else res.observation
                    for name,img in {**snap.images,'witness_cam':witness.read()}.items():Image.fromarray(np.asarray(img)).save(out/f'{step:04d}_{name}.png')
                if metrics.get('mechanics_invalid_latched'):
                    result['mechanics_failure']=True
                    break
                if res.terminated:
                    result['success']=res.termination_reason=='success'
                    if result['success']:result['task_score']=1.
                    break
                if ended is not None and step-ended>=20:break
        result['worst_ik_mm']={'left':left.worst_mm,'right':right.worst_mm}
        result['worst_ik_rotation_deg']={'left':left.worst_rot_deg,'right':right.worst_rot_deg}
        final_obs=fresh_observation() if args.no_record else res.observation
        for name,img in {**final_obs.images,'witness_cam':witness.read()}.items():Image.fromarray(np.asarray(img)).save(out/f'final_{name}.png')
        if args.reset_check and result['success']:
            from yam_arena.constants import READY
            ready=np.r_[np.array(READY,float),1.]
            idle=Action(data=pack(ready,ready))
            checks=[]
            with (out/'reset_trace.jsonl').open('w') as rt:
                for episode in range(2):
                    emb.reset(Scene(id=f'reset-check-{episode}',instruction=task.INSTRUCTION),seed=0)
                    valid=True
                    for j in range(200):
                        res=emb.step(idle); m=dict(task.STATE)
                        valid=valid and res.info.get('score')==0 and m['captured_strands']==0 and m['integrity'] and m['ring_closed']
                        rt.write(json.dumps({'reset':episode+1,'step':j+1,**m})+'\n');rt.flush()
                    checks.append({'reset':episode+1,'steps':200,'passed':bool(valid),'final':dict(task.STATE)})
                    print('RESET_CHECK',json.dumps(checks[-1]),flush=True)
            result['reset_checks']=checks

    except BaseException:
        result['error']=traceback.format_exc();raise
    finally:
        for w in writers.values():w.close()
        result['wall_seconds']=time.monotonic()-start
        manifest['source_after']=capture_sources(); manifest['unchanged']=manifest['source_before']==manifest['source_after']
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
        (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
        if emb is not None:
            emb._env.close()
            emb.close()
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--variant',choices=['full','idle','partial','wrong_hole','direct','no_open','crank_only'],default='full')
    p.add_argument('--max-steps',type=int,default=2400);p.add_argument('--no-record',action='store_true');p.add_argument('--snapshots',action='store_true')
    p.add_argument('--reset-check',action='store_true');p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=352)
    args=p.parse_args()
    result=run(args)
    expected=result['success'] if args.variant=='full' else not result['success'] and 'error' not in result
    raise SystemExit(0 if expected else 1)
