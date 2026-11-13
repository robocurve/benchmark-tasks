"""Run a prototype unboxing trial with physical flap joints and audited tape cuts."""
import argparse
import atexit
import hashlib
import json
import os
from pathlib import Path
import numpy as np
from config import *
from scoring import UnboxScorer
import telemetry

HERE=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--policy',choices=['demo','dream'],default='demo')
    p.add_argument('--stage',choices=['full','opening'],default='full',
                   help='opening is a diagnostic with tape already cut; cannot earn task credit')
    p.add_argument('--max-steps',type=int,default=1800)
    p.add_argument('--log-dir',type=Path,required=True)
    args=p.parse_args()
    if not 1<=args.max_steps<=3600: p.error('max-steps must be 1..3600')
    args.log_dir=args.log_dir.expanduser().resolve()
    args.log_dir.mkdir(parents=True,exist_ok=False)
    (args.log_dir/'source').mkdir()
    for source in HERE.glob('*.py'):
        (args.log_dir/'source'/source.name).write_bytes(source.read_bytes())
    (args.log_dir/'source-sha256.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in HERE.glob('*.py')},indent=2))
    (args.log_dir/'mode.json').write_text(json.dumps(dict(stage=args.stage,diagnostic=args.stage=='opening',
        privileged_demo=args.policy=='demo'),indent=2))
    if args.stage=='opening' and args.policy!='demo': p.error('opening diagnostic is for the scripted controller only')
    from inspect_robots.cli import main as cli
    from inspect_robots.registry import register
    from yam_arena.inspect_embodiment import YamArenaEmbodiment
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN

    class ParcelEmbodiment(YamArenaEmbodiment):
        def _ensure_sim(self,goal):
            from yam_arena import inspect_embodiment as adapter
            if adapter._APP is None:
                os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
                from isaaclab.app import AppLauncher
                adapter._APP=AppLauncher(headless=True,device=self.device,enable_cameras=True).app
                atexit.register(adapter._close_app_at_exit)
            from yam_arena import scene as sc
            import isaaclab.sim as sim
            from mechanics import cutter_usd, add_flap, wrapped_tape_usd
            from pxr import UsdPhysics, UsdGeom, Gf
            import omni.usd
            original=sc.task_object
            cutter=cutter_usd(args.log_dir/'cutter.usda')
            tapes={end+'_tape':wrapped_tape_usd(args.log_dir/(end+'_tape.usda'),end) for end in ('front','back')}
            def make(spec):
                obj=original(spec)
                custom_cfg=None
                if spec.name in tapes:
                    custom_cfg=sim.UsdFileCfg(usd_path=tapes[spec.name])
                if spec.name=='cutter':
                    custom_cfg=sim.UsdFileCfg(usd_path=cutter)
                if spec.name.endswith('_flap'):
                    # Replace the scaled primitive with a rigid Xform whose child
                    # has the dimensions, so hinge local positions stay in metres.
                    side=spec.name.split('_')[0]
                    from mechanics import cube
                    def spawn(path,cfg,translation=None,orientation=None,**kwargs):
                        from isaaclab.sim.utils.stage import get_current_stage
                        stage=get_current_stage()
                        root=UsdGeom.Xform.Define(stage,path)
                        root.AddTranslateOp().Set(Gf.Vec3d(*translation))
                        root.AddOrientOp().Set(Gf.Quatf(orientation[3],Gf.Vec3f(*orientation[:3])))
                        UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
                        UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(.018)
                        cube(stage,path+'/cardboard',spec.size,(0.,0.,0.),spec.color)
                        sign=1 if side=='left' else -1
                        for i in range(22):
                            lo=max(FLAP_END_X[0],SEAM[0]+i*.01)
                            hi=min(FLAP_END_X[1],SEAM[0]+(i+1)*.01)
                            xx=(lo+hi)/2-CENTER[0]
                            cube(stage,path+'/tape_'+str(i),(hi-lo-.0002,TAPE_WIDTH/2,.0003),
                                 (xx,-sign*(FLAP_CENTER_Y-TAPE_WIDTH/4),.0017),(.34,.20,.08),False)
                        for end,xx in enumerate(END_CUT_X):
                            # Render the taped top corner; the vertical tail is
                            # a separate wall patch and remains after cutting.
                            cube(stage,path+'/fold_'+str(end),(.001,TAPE_WIDTH/2,.0004),
                                 (xx-CENTER[0],-sign*(FLAP_CENTER_Y-TAPE_WIDTH/4),.0019),(.34,.20,.08),False)
                        add_flap(stage,path,side)
                        return root.GetPrim()
                    custom_cfg=sim.UsdFileCfg(usd_path=cutter,func=spawn)
                if custom_cfg is not None:
                    return sc.Object(name=spec.name,object_type=sc.ObjectType.BASE if spec.name in tapes else sc.ObjectType.RIGID,
                                     spawner_cfg=custom_cfg,
                                     initial_pose=sc.Pose(position_xyz=tuple(spec.pos),rotation_xyzw=(0.,0.,0.,1.)))
                return obj
            sc.task_object=make
            try: super()._ensure_sim(goal)
            finally: sc.task_object=original

        def reset(self,scene,*,seed=None):
            if hasattr(self,'audit'): raise RuntimeError('one reset per trial; use a fresh process')
            observation=super().reset(scene,seed=seed)
            import omni.usd
            self.stage=self._base.scene.stage
            self.hinges={}
            from pxr import UsdPhysics
            self.flap_paths={}
            candidates=[prim for prim in self.stage.Traverse()
                        if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName()=='hinge']
            for side in ('left','right'):
                found=[prim for prim in candidates if side+'_flap' in str(prim.GetPath())]
                if len(found)!=1:
                    paths=[str(prim.GetPath()) for prim in self.stage.Traverse() if 'flap' in str(prim.GetPath())]
                    raise RuntimeError('expected one physical '+side+' hinge; found '+repr(paths))
                self.hinges[side]=UsdPhysics.RevoluteJoint(found[0])
                self.flap_paths[side]=str(found[0].GetParent().GetPath())
            for path in self.flap_paths.values():
                root=self.stage.GetPrimAtPath(path)
                colliders=[str(p.GetPath()) for p in self.stage.Traverse()
                           if str(p.GetPath()).startswith(path+'/') and p.HasAPI(UsdPhysics.CollisionAPI)]
                if colliders!=[path+'/cardboard']:raise RuntimeError('plain flap must have exactly one cardboard collider')
            for name in ('front_tape','back_tape'):
                path='/World/envs/env_0/'+name
                if any(str(p.GetPath()).startswith(path) and p.HasAPI(UsdPhysics.CollisionAPI) for p in self.stage.Traverse()):
                    raise RuntimeError('tape film must use fracture constraints, not solid colliders')
            (args.log_dir/'geometry-audit.json').write_text(json.dumps(dict(handles=False,
                flap_length=FLAP_LENGTH,flap_width=FLAP_WIDTH,seam_gap=SEAM_GAP,
                corner_clearance=.00025,tape_width=TAPE_WIDTH,wrap_drop=TAPE_DROP,
                flap_outward_limit_deg=FLAP_OUTWARD_LIMIT,tape_colliders=0,tape_cut_paths=[p['name'] for p in CUT_PATHS],blade_thickness=BLADE_THICKNESS),indent=2))
            self.task.module._tracker=UnboxScorer()
            self.task.module._diagnostic=args.stage=='opening'
            if args.stage=='opening':
                self.task.module._tracker.cut=[True]*CUT_SEGMENTS
                self.task.module._tracker.covered.update(range(CUT_CELLS))
            telemetry.latest=None;telemetry.events.clear()
            self.released=False
            self.audit=(args.log_dir/'trajectory.jsonl').open('x')
            import cv2
            cv2.imwrite(str(args.log_dir/'setup.png'),cv2.cvtColor(observation.images['top_cam'],cv2.COLOR_RGB2BGR))
            self._task_step()  # initial measured state before the demo policy resets
            return observation

        def _task_step(self):
            from pxr import UsdPhysics, UsdGeom, Gf
            def arr(v):
                v=v.torch if hasattr(v,'torch') else v
                return v.detach().cpu().numpy().astype(float)
            world=self._base.arena_world
            def state(name):
                pose=arr(world.get_pose_w(name))[0]
                q=pose[3:]
                from yam_arena.urdf import quat_xyzw_to_matrix
                rot=quat_xyzw_to_matrix(q)
                return dict(pos=pose[:3].tolist(),quat=q.tolist(),
                            speed=float(np.linalg.norm(arr(world.get_root_linear_velocity_w(name))[0]))),rot
            cutter,rot=state('cutter'); item,_=state('item')
            hands=[]
            for side in ('left','right'):
                name=side+'_robot'; r=self._base.scene[name]; ids,fids=self._ids[name]
                q=arr(r.data.joint_pos)[0]
                tcp,rotation=ik.tcp_pose(q[ids],self.rig.bases[side])
                hands.append((tcp,float(np.clip(q[fids].mean()/FINGER_OPEN,0,1)),rotation,q[ids]))
            held=any(np.linalg.norm(tcp-np.array(cutter['pos']))<.035 and .08<opening<.38 for tcp,opening,_,_ in hands)
            angles=[];flaps={}
            for side,sign in [('left',-1),('right',1)]:
                flaps[side],r=state(side+'_flap')
                angles.append(float(sign*np.degrees(np.arctan2(r[2,1],r[1,1]))))
            sample=dict(time=self._t,cutter=cutter,item=item,
                        tip=(np.array(cutter['pos'])+rot@np.array(TIP_LOCAL)).tolist(),
                        hands=[dict(tcp=tcp.tolist(),opening=opening,tcp_rotation=rotation.tolist(),joints=joints.tolist())
                               for tcp,opening,rotation,joints in hands],flaps=flaps,
                        held=bool(held),blade_down=float(rot[2,2]),blade_forward=rot[:,0].tolist(),flap_angles=angles,
                        item_released=all(np.linalg.norm(tcp-np.array(item['pos']))>.065 for tcp,_,_,_ in hands),diagnostic=args.stage=='opening')
            tracker=self.task.module._tracker
            sample['metrics']=tracker.update(sample)
            sample['controller']=telemetry.command
            if all(tracker.cut) and not self.released:
                for side in ('left','right'):
                    joint=self.hinges[side]
                    joint.GetLowerLimitAttr().Set(-FLAP_OUTWARD_LIMIT if side=='left' else -85.)
                    joint.GetUpperLimitAttr().Set(85. if side=='left' else FLAP_OUTWARD_LIMIT)
                self.released=True
            # Show cut segments on both halves. This changes appearance only.
            for side in ('left','right'):
                for i,cut in enumerate(tracker.cut[:22]):
                    if cut:
                        prim=self.stage.GetPrimAtPath(self.flap_paths[side]+'/tape_'+str(i))
                        if prim: UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(.12,.07,.03)])
            groups=tracker.groups()
            for side in ('left','right'):
                for i,name in enumerate(('front_fold','back_fold')):
                    if groups[name]['cut']==groups[name]['total']:
                        prim=self.stage.GetPrimAtPath(self.flap_paths[side]+'/fold_'+str(i))
                        if prim:UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(.12,.07,.03)])
            telemetry.latest=sample
            self.audit.write(json.dumps(sample,allow_nan=False)+'\n'); self.audit.flush()
            return super()._task_step()

        def close(self):
            if hasattr(self,'audit'):
                self.audit.close()
                summary=self.task.module._tracker.result()
                if args.stage=='opening': summary=dict(score=0.,diagnostic_precut=True,diagnostic_milestones=summary)
                (args.log_dir/'score-summary.json').write_text(json.dumps(summary,indent=2))
                (args.log_dir/'controller-events.json').write_text(json.dumps(telemetry.events,indent=2))
            super().close()

    register('embodiment','parcel_prototype')(ParcelEmbodiment)
    policy='dream'
    if args.policy=='demo':
        from demo_policy import register_policy
        policy=register_policy(opening_only=args.stage=='opening')
    argv=['run','--embodiment','parcel_prototype','--policy',policy,
          '-E',f'task={HERE / "task.py"}','-E','object_jitter=0','-E','control_hz=20',
          '-E','cam_width=320','-E','cam_height=176','--instruction',INSTRUCTION,
          '--scorer','task_score','--max-steps',str(args.max_steps),'--store-frames',
          '--log-dir',str(args.log_dir),'--seed','0','--no-prompt','--grader','none']
    return cli(argv)

if __name__=='__main__': main()
