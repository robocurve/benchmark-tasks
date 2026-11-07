"""One isolated GPU practice episode; preserves policy, states, images and verdict."""
from pathlib import Path
import argparse,hashlib,json,shutil,subprocess,traceback,os
import numpy as np
from control import Controller,Stop,Done
from policy import run
from drawer_layout_v6 import *

class Recorder:
    def __init__(self,out,fps=5):self.out=out;self.last_phase=None;self.index=0;self.fps=fps
    def write(self,name,row):
        with (self.out/name).open('a') as f:f.write(json.dumps(row)+'\n')
    def capture(self,obs,step,phase):
        import cv2
        key=phase!=self.last_phase or step%40==0
        if phase!=self.last_phase:print('PHASE:',step,phase,flush=True)
        self.last_phase=phase
        sampled=step%(20//self.fps)==0
        if not sampled and not key:return
        for name,rgb in obs.images.items():
            bgr=cv2.cvtColor(np.asarray(rgb,dtype=np.uint8),cv2.COLOR_RGB2BGR)
            if sampled:
                d=self.out/'camera_frames'/name;d.mkdir(parents=True,exist_ok=True)
                p=d/f'{self.index:06d}.jpg'
                if not cv2.imwrite(str(p),bgr,[cv2.IMWRITE_JPEG_QUALITY,90]):raise Stop('Could not save '+str(p))
            if key:
                p=self.out/f'{step:04d}_{name}.jpg'
                if not cv2.imwrite(str(p),bgr,[cv2.IMWRITE_JPEG_QUALITY,95]):raise Stop('Could not save '+str(p))
        if sampled:
            self.write('frame_times.jsonl',dict(frame=self.index,step=step,time_s=step/20))
            self.index+=1
    def videos(self):
        ffmpeg=shutil.which('ffmpeg');results={}
        if not ffmpeg:return {'status':'ffmpeg missing; JPEG frames retained'}
        for d in (self.out/'camera_frames').glob('*'):
            if not d.is_dir():continue
            with (self.out/(d.name+'.ffmpeg.log')).open('w') as log:
                try:
                    p=subprocess.run([ffmpeg,'-n','-framerate',str(self.fps),'-i',str(d/'%06d.jpg'),'-c:v','libx264','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(self.out/(d.name+'.mp4'))],stdout=log,stderr=log,timeout=120,env={k:v for k,v in os.environ.items() if k not in ('LD_LIBRARY_PATH','LD_PRELOAD')})
                    results[d.name]='saved' if p.returncode==0 else 'encoding failed; JPEG frames retained'
                except Exception as e:results[d.name]=str(e)
        return results

def array(value):
    value=value.torch if hasattr(value,'torch') else value
    return value.detach().cpu().numpy() if hasattr(value,'detach') else np.asarray(value)

class Backend:
    def __init__(self,env,obs,rec):
        from inspect_robots_yam.packing import STATE_KEY
        self.env=env;self.obs=obs;self.rec=rec;self.key=STATE_KEY;self.info={};self.frames=0;self._state_frame=-1;self._force_frame=-1;self._fk_frame=-1
        for arm in ['left','right']:
            sensor=env._base.scene[arm+'_contacts']
            names=list(sensor.body_names)
            if not any('tip_' in n for n in names):raise Stop('Contact sensor does not contain finger bodies: '+str(names))
            rec.write('sensor_setup.jsonl',dict(arm=arm,bodies=names))
        self.contact_force();self.check_fk()
    def joints(self):return self.obs.state[self.key]
    def finger_positions(self):
        if getattr(self,'_finger_frame',-1)!=self.frames:
            from yam_arena.constants import FINGER_JOINTS
            values=[]
            for arm in ['left','right']:
                robot=self.env._base.scene[arm+'_robot']
                q=array(robot.data.joint_pos)[0]
                values.append([float(q[robot.joint_names.index(n)]) for n in FINGER_JOINTS])
            self._finger_cache=np.asarray(values,float)
            if not np.isfinite(self._finger_cache).all():raise Stop('Invalid measured finger positions')
            self._finger_frame=self.frames
        return self._finger_cache.copy()
    def state(self):
        from yam_arena.scene import object_states
        if self._state_frame!=self.frames:
            s=object_states(self.env._env,['drawer','garment_fold'])
            self._state_cache=dict(opening=float(DRAWER_CLOSED_BACK-s['drawer'].high[0]),compression=float(FOLD_POS[2]+FOLD_HIGH[2]-s['garment_fold'].high[2]))
            self._state_frame=self.frames
        return self._state_cache.copy()
    def contact_force(self):
        if self._force_frame==self.frames:return self._force_cache
        left_force=0.
        for arm in ['left','right']:
            sensor=self.env._base.scene[arm+'_contacts'];data=sensor.data
            force=array(data.net_normal_forces_w)[0]
            if force.shape!=(len(sensor.body_names),3) or not np.isfinite(force).all():raise Stop('Invalid contact sensor readings')
            values={n:float(np.linalg.norm(force[i])) for i,n in enumerate(sensor.body_names)}
            if arm=='left':left_force=max(v for n,v in values.items() if 'tip_' in n)
            self.rec.write('contact_forces.jsonl',dict(step=self.frames,arm=arm,normal_force_norms=values))
        self._force_frame=self.frames;self._force_cache=left_force
        return left_force
    def check_fk(self):
        if self._fk_frame==self.frames:return
        from yam_arena.analysis import fk_errors
        from yam_arena.constants import LINKS
        from yam_arena.station import BASES
        for arm in ['left','right']:
            robot=self.env._base.scene[arm+'_robot']
            q=array(robot.data.joint_pos)[0];root=array(robot.data.root_link_pose_w)[0]
            if np.linalg.norm(root[:3]-BASES[arm])>.001:raise Stop('Robot base differs from geometry model: '+arm)
            if np.linalg.norm(root[3:6])>.001:raise Stop('Robot base orientation differs from geometry model')
            errors=fk_errors(robot.body_names,array(robot.data.body_link_pose_w)[0],root,dict(zip(robot.joint_names,q)))
            if len(errors)!=len(LINKS) or max(v[0] for v in errors.values())>1.0 or max(v[1] for v in errors.values())>.5:
                raise Stop('Measured robot links do not match collision model: '+str(errors))
        # The geometry guard uses both measured finger coordinates directly.
        # The full per-link FK check above still verifies each finger link.
        fingers=self.finger_positions()
        self.rec.write('finger_tracking.jsonl',dict(step=self.frames,positions_m=fingers.tolist(),asymmetry_m=abs(fingers[:,0]-fingers[:,1]).tolist()))
        self._fk_frame=self.frames
    def step(self,q):
        from inspect_robots.types import Action
        result=self.env.step(Action(data=q));self.obs=result.observation;self.info=result.info or {};self.frames+=1
        if result.terminated and not self.info.get('success'):raise Stop('Simulator terminated without success')
        return self.info

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--mode',choices=['reach','cooperate'],default='cooperate');parser.add_argument('--quality',choices=['preview','capture'],default='preview');args=parser.parse_args()
    here=Path(__file__).resolve().parent;task=here.parent/'task_garment_v6.py'
    expected=json.loads((here/'expected_existing.json').read_text())
    for name,digest in expected.items():
        source=here.parent/name
        if not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest()!=digest:
            raise RuntimeError('Existing scene file differs from reviewed debug bundle: '+name)
    out=args.output.expanduser().resolve();out.mkdir(parents=True,exist_ok=False)
    if shutil.disk_usage(out).free<3*1024**3:raise RuntimeError('Need 3 GB free space for camera frames')
    rec=Recorder(out,fps=5 if args.quality=='preview' else 10);env=None;api=None;backend=None;status='startup_error';message='';success=False;diagnostic=False
    sources=[task,*here.glob('*.py'),here.parent/'task.py',here.parent/'task_visual.py']
    identity={}
    for p in sources:
        d=out/'source'/p.relative_to(here.parent);d.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,d)
        identity[str(p.relative_to(here.parent))]=hashlib.sha256(p.read_bytes()).hexdigest()
    for p in sorted((here.parent/'assets').rglob('*')):
        if p.is_file():identity[str(p.relative_to(here.parent))]=hashlib.sha256(p.read_bytes()).hexdigest()
    (out/'identity.json').write_text(json.dumps(dict(seed=101,scene_revision=6,controller_revision='6.0',mode=args.mode,quality=args.quality,policy='feedback scripted demo; privileged object states; no VLA',files=identity),indent=2))
    os.environ['JAM_TRACE_PATH']=str(out/'jam_trace.jsonl')
    try:
        from yam_arena.inspect_embodiment import YamArenaEmbodiment
        from inspect_robots import Scene
        class ContactEmbodiment(YamArenaEmbodiment):
            def _ensure_sim(self,goal):
                # Start Kit before importing the embodiment's Isaac modules.
                import atexit
                import yam_arena.inspect_embodiment as module
                if module._APP is None:
                    from isaaclab.app import AppLauncher
                    os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
                    os.environ['DISPLAY']=module.window_display(os.environ)
                    kw=dict(headless=False,device=self.device,enable_cameras=True,visualizer=['kit'])
                    if os.environ['DISPLAY']==module.DESKTOP_DISPLAY:
                        kw['window_width'],kw['window_height']=module.DESKTOP_SIZE
                    module._APP=AppLauncher(**kw).app
                    atexit.register(module._close_app_at_exit)
                import yam_arena.embodiment as embodiment
                original=embodiment.BimanualYamEmbodiment.__init__
                def with_contacts(emb,*a,**kw):
                    kw['contact_sensors']=True
                    original(emb,*a,**kw)
                embodiment.BimanualYamEmbodiment.__init__=with_contacts
                try:return super()._ensure_sim(goal)
                finally:embodiment.BimanualYamEmbodiment.__init__=original
        env=ContactEmbodiment(task=str(task),headless=False,object_jitter=0,control_hz=20,cam_width=640 if args.quality=='preview' else 960,cam_height=360 if args.quality=='preview' else 540)
        obs=env.reset(Scene(id='drawer-v6-development',instruction='Use both arms to release the trapped garment and open the drawer fully.'),seed=101)
        rec.capture(obs,0,'initial')
        backend=Backend(env,obs,rec);api=Controller(backend,rec)
        if args.mode=='reach':
            api.move('left',(HANDLE[0],HANDLE[1],.9075),3,'A approach; B held')
            api.move('left',HANDLE,3,'A lower to handle; B held')
            api.hold(.5)
            diagnostic=True;status='reach_pass';message='A reached hover and handle poses; no grasp or cooperation attempted.'
        else:
            run(api)
            status='policy_finished';message='Policy completed without evaluator success termination.'
    except Done:
        status='task_success';message='Evaluator success after coordinated sequence.';success=True
    except KeyboardInterrupt:
        status='interrupted';message='Run interrupted; recorded evidence retained.'
    except Exception as e:
        status='stopped';message=str(e);(out/'exception.txt').write_text(traceback.format_exc())
    finally:
        report=dict(status=status,message=message,mode=args.mode,quality=args.quality,reach_diagnostic_passed=diagnostic,task_success=success,
                    score=float(backend.info.get('score',0)) if backend else 0,
                    steps=api.steps if api else 0,phase=api.phase if api else 'setup',
                    geometry_checks=api.guard_passes if api else 0,
                    grasp_contact_observed=api.contact_verified if api else False,
                    safety_verdict='STOP_OR_UNVERIFIED' if not (success or diagnostic) else 'NO_GEOMETRIC_CONFLICT_OBSERVED',
                    safety_scope='URDF hull bounding boxes: inter-arm and cabinet/drawer panels, independent measured fingers and conservative finger-stroke envelopes, sampled arm paths, 8 mm inter-arm margin. Not a complete safety certificate; no inter-arm force classification.',
                    limitations=['spring proxy, not deformable cloth','scripted privileged-state demo, not a VLA','no full table/camera/self-collision certification'])
        if backend is not None:
            try:report['last_object_state']=backend.state()
            except Exception:pass
        (out/'verdict.json').write_text(json.dumps(report,indent=2))
        print('RESULT:',json.dumps(report),flush=True)
        try:
            report['videos']=rec.videos();(out/'verdict.json').write_text(json.dumps(report,indent=2))
        finally:
            if env is not None:env.close()
    return 0 if (success or diagnostic) else 2
if __name__=='__main__':raise SystemExit(main())
