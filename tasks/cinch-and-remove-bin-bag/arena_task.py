"""Cinch and remove a loaded drawstring liner from a small countertop bin.

An ordinary white drawstring liner (one string in the mouth hem, exposed as a
short loop at a cut-out on each side) holds three inert waste blocks. Pinch the
two loops, pull them so the string slides out of the hem and gathers the
mouth, lift the bag out by the strings and set it down upright on the table
beside the bin, then let go. There is no cord lock, handle or knot: the mouth
stays gathered only through string tension while carried and film contact
after release. Film and string forces are uncalibrated approximations.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
from removal_assets import WALL_H
import mechkit as m
from yam_arena.task_file import Box
from drawstring_score import DrawstringJudge

SCENE='station'
INSTRUCTION='Pull both drawstrings to close the full bag, lift it out of the bin by the drawstrings, and set it down upright on the table beside the bin without spilling any contents.'
SUCCESS_HOLD_S=0.
SCORE_OVER='end'
from removal_assets import BLOCK_SIZES,BLOCK_MASS,BLOCK_COUNT,block_positions,RIM_Z
OBJECTS=[Box(name=f'rubbish_{i}',pos=block_positions()[i],size=BLOCK_SIZES[i],mass=BLOCK_MASS) for i in range(BLOCK_COUNT)]
ANCHORS={}

original_patch=m._patch_scene
def patch_scene(scene):
    original_patch(scene)
    original_build=scene.build_task_environment
    def build(embodiment,task,control_hz=None):
        if getattr(task.module,'INSTRUCTION',None)!=INSTRUCTION:return original_build(embodiment,task,control_hz)
        from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
        from isaaclab_arena.scene.scene import Scene
        from yam_arena.embodiment import BimanualYamEmbodiment
        from scipy.spatial.transform import Rotation
        from removal_assets import build_assets
        assets,anchors,_=build_assets();ANCHORS.clear();ANCHORS.update(anchors)
        # anchors is populated during asset spawn, so retain its live mapping.
        TRACKER.anchors=anchors
        cameras=BimanualYamEmbodiment(rig=embodiment.rig,enable_cameras=True,overview_cam=True,cam_width=640,cam_height=360)
        embodiment.camera_config=cameras.camera_config
        eye=np.array([1.25,1.25,1.55]);forward=np.array([.12,0.,1.0])-eye;forward/=np.linalg.norm(forward)
        left=np.cross([0.,0.,1.],forward);left/=np.linalg.norm(left);up=np.cross(forward,left)
        embodiment.camera_config.overview_cam.offset.pos=tuple(eye)
        embodiment.camera_config.overview_cam.offset.rot=tuple(Rotation.from_matrix(np.stack([forward,left,up],axis=1)).as_quat())
        for side in ['left','right']:
            robot=getattr(embodiment.scene_config,side+'_robot')
            pass  # standard adapter gripper
            from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
            robot.spawn.collision_props=PhysxCollisionPropertiesCfg(contact_offset=.001,rest_offset=0.)
        def configure(cfg):
            cfg=scene.env_cfg_callback(control_hz)(cfg);cfg.sim.dt=1/480.;cfg.decimation=24;cfg.sim.render_interval=96;p=cfg.sim.physics
            p.gpu_max_rigid_contact_count=2**17;p.gpu_max_rigid_patch_count=2**15;p.gpu_found_lost_pairs_capacity=2**16
            p.gpu_found_lost_aggregate_pairs_capacity=2**18;p.gpu_total_aggregate_pairs_capacity=2**16
            p.gpu_max_soft_body_contacts=2**16;p.gpu_max_particle_contacts=2**16
            return cfg
        return IsaacLabArenaEnvironment(name='bin_removal_inspect',scene=Scene(assets),embodiment=embodiment,env_cfg_callback=configure)
    scene.build_task_environment=build
m._patch_scene=patch_scene
m._Finder.targets['yam_arena.scene']=patch_scene
if 'yam_arena.scene' in sys.modules:patch_scene(sys.modules['yam_arena.scene'])

# Remove the previous episode's force callback before reset settles the scene.
original_embodiment_patch=m._patch_embodiment
def patch_embodiment(module):
    original_embodiment_patch(module)
    cls=module.YamArenaEmbodiment
    if getattr(cls,'_bin_reset_hook',False):return
    original_reset=cls.reset
    def reset(self,*a,**kw):
        ours=getattr(getattr(self,'task',None),'module',None) is not None and getattr(self.task.module,'INSTRUCTION',None)==INSTRUCTION
        if ours and TRACKER.runtime is not None:
            TRACKER.runtime.detach();TRACKER.runtime=None
        return original_reset(self,*a,**kw)
    original_close=cls.close
    def close(self,*a,**kw):
        # Isaac can terminate before Python atexit runs. Persist the last
        # observations before simulator shutdown, including policy errors.
        task=getattr(getattr(self,'task',None),'module',None)
        if getattr(task,'INSTRUCTION',None)==INSTRUCTION:TRACKER.flush()
        return original_close(self,*a,**kw)
    cls.reset=reset;cls.close=close;cls._bin_reset_hook=True
m._patch_embodiment=patch_embodiment
m._Finder.targets['yam_arena.inspect_embodiment']=patch_embodiment
if 'yam_arena.inspect_embodiment' in sys.modules:patch_embodiment(sys.modules['yam_arena.inspect_embodiment'])

class Tracker:
    def __init__(self):
        self.runtime=None;self.anchors={};self.result={'score':0.};self.count=0;self.save_dir=None;self.trace=[];self.episode=-1
        import atexit
        atexit.register(self.flush)
    def flush(self):
        if self.save_dir is not None:
            import json
            (self.save_dir/'trace.json').write_text(json.dumps(self.trace))
            (self.save_dir/'live_result.json').write_text(json.dumps(self.result,indent=2))
    def reset(self,env):
        from removal_runtime import DrawstringRuntime
        # The string's rest length is taken from the settled reset scene.
        if self.runtime is not None:self.runtime.detach()
        self.runtime=DrawstringRuntime(env)
        self.count=0;self.trace=[];self.snapshots={};self.result={'score':0.}
        import os,json,hashlib
        parameters={'bin_top_z':RIM_Z,'bin_center_xy':[.22,0.],'bin_outer_radius_m':.0855,'sample_interval_steps':1,'require_table_deposit':True,'control_hz':20}
        self.judge=DrawstringJudge(parameters)
        if os.environ.get('BIN_EVIDENCE_DIR'):
            self.episode+=1;self.save_dir=Path(os.environ['BIN_EVIDENCE_DIR'])/f'episode_{self.episode:03d}'
            self.save_dir.mkdir(parents=True,exist_ok=False)
            (self.save_dir/'task_parameters.json').write_text(json.dumps(parameters,indent=2))
            (self.save_dir/'source_hashes.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')},indent=2))
        self.record(env)
    def record(self,env):
        from removal_runtime import observation
        s,v=observation(env,self.count,self.anchors['loaded_bag'])
        if self.runtime is not None and self.runtime.cable.last:s['string_tension_n']=float(self.runtime.cable.last.get('tension',0.))  # diagnostic only
        self.trace.append(s);self.snapshots[self.count]=v
        self.result=self.judge.update(s,v)
        if self.save_dir is not None:
            np.savez_compressed(self.save_dir/f'state_{self.count:04d}.npz',loaded_bag=v[None])
            if self.count%10==0:self.flush()
    def step(self,env):
        self.count+=1
        self.record(env)
TRACKER=Tracker()
m.on_reset(TRACKER.reset)
m.on_step(TRACKER.step)
def score(objects):return float(TRACKER.result.get('score',0.))
def success(objects):return score(objects)==1.

