"""Attach substep instrumentation to a single native YAM scene."""
import atexit
import json
import os
from pathlib import Path
from unittest.mock import patch
import numpy as np
from arm_clearance import Separation,local_bounds,rotation_xyzw,STOP_MM
from shutter_logic import NAMES, Trainer, reading, intact

def array(value):
    value=getattr(value,"torch",value)
    if hasattr(value,"detach"): value=value.detach().cpu()
    if hasattr(value,"numpy"): value=value.numpy()
    return np.asarray(value)

class Observer:
    def __init__(self,base,entry,seed):
        import omni.usd
        from isaaclab_physx.physics import PhysxManager
        from pxr import Usd,UsdPhysics
        from yam_arena.robot import asset_link_paths
        self.dt=float(base.sim.get_physics_dt())
        self.decimation=round(float(base.step_dt)/self.dt)
        if abs(self.dt-1/240)>1e-7 or self.decimation!=12:
            raise ValueError("Expected 240 Hz physics and 20 Hz control")
        self.trial=Trainer()
        self.tick=0;self.sub=0
        root="/World/envs/env_0/shutter_trainer"
        sim=PhysxManager.get_physics_sim_view()
        self.bodies={n:sim.create_rigid_body_view(root+"/"+n) for n in NAMES}
        if any(v.count!=1 for v in self.bodies.values()):
            raise ValueError("A shutter rigid body is missing")
        roots=[str(base.scene[k].cfg.prim_path).replace("env_.*","env_0").replace("env_[^/]+","env_0") for k in ("left_robot","right_robot")]
        stage=omni.usd.get_context().get_stage()
        self.links=sorted(str(p.GetPath()) for p in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies())
                          if p.HasAPI(UsdPhysics.RigidBodyAPI) and any(str(p.GetPath()).startswith(r+"/") for r in roots))
        self.arm_links=[[link for link in self.links if link.startswith(root_path+"/")] for root_path in roots]
        if [len(k) for k in self.arm_links]!=[9,9]:
            raise RuntimeError("Expected nine collision links per YAM arm")
        self.arm_bounds=np.stack([local_bounds(stage,links) for links in self.arm_links])
        self.separation=Separation(self.arm_bounds,self.arm_links)
        self.link_readers=[]
        for key,paths in zip(("left_robot","right_robot"),self.arm_links,strict=True):
            articulation=base.scene[key]
            view=getattr(articulation,"root_physx_view",None)
            read=getattr(view,"get_link_transforms",None)
            if callable(read):
                names=list(articulation.body_names)
                order=[names.index(path.rsplit("/",1)[-1]) for path in paths]
                self.link_readers.append(lambda read=read,order=order:array(read()).reshape(-1,7)[order])
            else:
                views=[sim.create_rigid_body_view(path) for path in paths]
                if any(v.count!=1 for v in views):raise RuntimeError("Arm-link tensor view mismatch")
                self.link_readers.append(lambda views=views:np.stack([array(v.get_transforms()).reshape(7) for v in views]))
        self.arm_owner={link:index for index,arm in enumerate(self.arm_links) for link in arm}
        self.robot_hits=set()
        self.hands={r+asset_link_paths()[k] for r in roots for k in ("tip_left","tip_right","gripper")}
        if len(self.hands)!=6 or not self.hands.issubset(self.links): raise ValueError("YAM link inventory mismatch")
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools
        self.fixture_paths={root+"/"+n:n for n in NAMES}
        self.robot_paths=set(self.links)
        self.decode=PhysicsSchemaTools.intToSdfPath
        self.impulses={}
        self.faces={}
        import carb.settings
        self.settings=carb.settings.get_settings()
        self.setting="/physics/disableContactProcessing"
        self.previous=self.settings.get(self.setting)
        self.settings.set_bool(self.setting,False)
        if self.settings.get(self.setting) != False:
            raise RuntimeError("Could not enable PhysX contact processing")
        self.callback_count=0
        self.header_count=0
        self.point_count=0
        self.matched_count=0
        self.raw_examples={}
        self.owners=self.robot_paths | set(self.fixture_paths)
        self.report_error=None
        self.subscription=get_physx_simulation_interface().subscribe_contact_report_events(self.on_contacts)
        view=self.bodies["door"]
        initial=view.get_transforms()
        self.motor_origin=array(initial).reshape(7).copy()
        self.origin_z=float(array(self.bodies["frame"].get_transforms()).reshape(7)[2])
        def buffers(t):
            if isinstance(initial,np.ndarray):
                return t,np.array([0],dtype=np.uint32)
            if hasattr(initial,"detach"):
                import torch
                return torch.as_tensor(t,device=initial.device),torch.zeros(1,dtype=torch.int32,device=initial.device)
            import warp as wp
            return wp.array(t,dtype=wp.float32,device=initial.device),wp.array([0],dtype=wp.uint32,device=initial.device)
        self.buffers=buffers
        log=os.environ.get("SHUTTER_TRACE")
        self.fp=Path(log).open("x",encoding="utf8") if log else None
        self.record(dict(kind="reset",dt=self.dt,substeps=self.decimation,seed=seed,filters=self.links,arm_links=self.arm_links,arm_bounds=self.arm_bounds.tolist(),hands=sorted(self.hands),disable_contact_processing_before=self.previous,disable_contact_processing_active=self.settings.get(self.setting)))
        self.measure()
    def record(self,obj):
        if self.fp: self.fp.write(json.dumps(obj,allow_nan=False,separators=(",",":"))+"\n")
    def check_grip(self, fn, value):
        try:
            fn(value)
        except ValueError as exc:
            if self.fp:
                self.fp.flush()
                path=Path(os.environ.get("SCALE_RESULT",Path(self.fp.name).with_name("qualification-stop.json")))
                path.write_text(json.dumps(dict(result="STOP",tick=self.tick,sub=self.sub,
                    phase=self.grip_gate.label,reason=str(exc)),indent=2))
            raise

    def drive(self):
        pose=self.motor_origin.copy()
        pose[2]=self.origin_z+self.trial.target/1000
        self.bodies["door"].set_kinematic_targets(*self.buffers(pose.reshape(1,7).astype(np.float32)))
    def owner(self, actor, collider):
        path=str(self.decode(actor))
        if path in self.owners:
            return path
        shape=str(self.decode(collider))
        while "/" in shape:
            if shape in self.owners:
                return shape
            shape=shape.rsplit("/",1)[0]
        return path

    def on_contacts(self, headers, points):
        try:
            self.callback_count+=1
            for h in headers:
                self.header_count+=1
                self.point_count+=h.num_contact_data
                if not h.num_contact_data:
                    continue
                a,b=self.owner(h.actor0,h.collider0),self.owner(h.actor1,h.collider1)
                if len(self.raw_examples)<24:
                    key=str((str(self.decode(h.actor0)),str(self.decode(h.actor1)),str(self.decode(h.collider0)),str(self.decode(h.collider1))))
                    self.raw_examples[key]=True
                if a in self.arm_owner and b in self.arm_owner and self.arm_owner[a]!=self.arm_owner[b]:
                    self.robot_hits.add((a,b) if self.arm_owner[a]==0 else (b,a))
                if a not in self.fixture_paths and b not in self.fixture_paths:
                    continue
                for source,other,sign in ((a,b,1),(b,a,-1)):
                    name=self.fixture_paths.get(source)
                    if name is None:
                        continue
                    if other not in self.robot_paths and not (name=="block" and self.fixture_paths.get(other) in ("left","right")):
                        continue
                    self.matched_count+=1
                    key=(name,other)
                    summed=self.impulses.setdefault(key,np.zeros(3))
                    for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                        point=points[i]
                        impulse=np.asarray(point.impulse,dtype=float)
                        if impulse.ndim==0:
                            impulse=impulse*np.asarray(point.normal,dtype=float)
                        if impulse.shape!=(3,) or not np.isfinite(impulse).all():
                            raise ValueError("Invalid contact report impulse")
                        summed+=sign*impulse
                        if getattr(self,"grip_gate",None) and name=="block" and other in self.hands:
                            collider=h.collider0 if source==a else h.collider1
                            if str(self.decode(collider)).endswith("/block/pickup"):
                                normal=np.asarray(point.normal,dtype=float)
                                if abs(normal[1])>.85 and abs(impulse[1])/self.dt>.04:
                                    self.faces.setdefault(other,set()).add(1 if normal[1]*sign>0 else -1)

        except Exception as exc:
            self.report_error=exc

    def measure(self):
        if self.report_error is not None:
            raise RuntimeError("PhysX contact report failed") from self.report_error
        link_poses=np.stack([read() for read in self.link_readers])
        clearance,closest=self.separation.measure(link_poses[:,:,:3],rotation_xyzw(link_poses[:,:,3:]))
        robot_hits=sorted(self.robot_hits)
        self.robot_hits.clear()
        if robot_hits:self.trial.reason=self.trial.reason or "inter-arm contact"
        poses={n:array(v.get_transforms()).reshape(7).tolist() for n,v in self.bodies.items()}
        pairs={n:{} for n in NAMES}
        forces=[0.,0.]
        for (name,other),impulse in self.impulses.items():
            force=float(np.linalg.norm(impulse)/self.dt)
            if other in self.robot_paths:
                if force>.04:
                    pairs[name][other]=force
            else:
                forces[("left","right").index(self.fixture_paths[other])]=force
        self.impulses.clear()
        s=reading(poses,forces,pairs,self.hands)
        if not intact(poses,self.motor_origin,self.origin_z):
            self.trial.reason=self.trial.reason or "geometry"
        mode=os.environ.get("SHUTTER_CONTROL")
        if self.fp and mode in ("idle","one_site","dead_edge","reference","alternate") and not os.environ.get("SCALE_RESULT"):
            structural={body:{link:f for link,f in contacts.items() if link not in self.hands or body=="door"}
                        for body,contacts in pairs.items()}
            structural={k:v for k,v in structural.items() if v}
            wrong_start=False
            if self.trial.mode=="open" and self.trial.armed and s.button>=5 and s.button_hand:
                targets={"idle":(), "one_site":("left",), "dead_edge":("right",),
                         "reference":("right","left"), "alternate":("left","right")}[mode]
                i=self.trial.cycles
                wrong_start=i>=len(targets) or s.site()!=targets[i] or (mode=="dead_edge" and abs(s.cassette-48)>3)
            if structural or wrong_start:
                report=dict(tick=self.tick,sub=self.sub,structural_contacts=structural,wrong_start=wrong_start,
                            poses=poses,pairs=pairs,carriage_mm=s.carriage,button_mm=s.button)
                Path(self.fp.name).with_name("qualification-stop.json").write_text(json.dumps(report,indent=2))
                self.fp.flush()
                raise RuntimeError("Qualification stopped at unsafe fixture contact or unexpected start; see qualification-stop.json")
        self.trial.observe(s,self.dt)
        row=dict(kind="sample",tick=self.tick,sub=self.sub,poses=poses,pairs=pairs,forces=forces,
                         score=self.trial.score,reason=self.trial.reason,mode=self.trial.mode,pickup_faces={k:sorted(v) for k,v in self.faces.items()})
        self.faces.clear()
        if getattr(self,"grip_gate",None):
            row.update(phase=self.grip_gate.label,arm=self.grip_gate.arm,completed_releases=list(self.grip_gate.legs))
        if os.environ.get("SCALE_RESULT"):row["arm_poses_xyzw"]=link_poses.tolist()
        row.update(arm_clearance_mm=clearance,closest_arm_links=closest,inter_arm_contacts=robot_hits)
        self.record(row)
        if (mode or os.environ.get("SCALE_RESULT")) and (clearance<=STOP_MM or robot_hits):
            stop=dict(result="STOP",reason="inter-arm clearance or contact",tick=self.tick,sub=self.sub,
                      clearance_mm=clearance,closest_links=closest,contacts=robot_hits)
            if self.fp:
                self.fp.flush()
                Path(os.environ.get("SCALE_RESULT",Path(self.fp.name).with_name("qualification-stop.json"))).write_text(json.dumps(stop,indent=2))
            raise RuntimeError(str(stop))
        if getattr(self,"grip_gate",None):
            self.check_grip(self.grip_gate.sample,row)
        self.sub+=1
    def close(self):
        self.subscription=None
        if self.previous is None:
            self.settings.destroy_item(self.setting)
        else:
            self.settings.set_bool(self.setting,self.previous)
        if self.fp and not self.fp.closed:
            report=dict(callbacks=self.callback_count,headers=self.header_count,points=self.point_count,
                        matched=self.matched_count,raw_examples=list(self.raw_examples),
                        disable_contact_processing_before=self.previous)
            Path(self.fp.name).with_name("contact-report.json").write_text(json.dumps(report,indent=2),encoding="utf8")
        if self.fp and not self.fp.closed:self.fp.close()

def install(entry):
    from yam_arena import inspect_embodiment as bridge
    cls=bridge.YamArenaEmbodiment
    if getattr(cls,'_shutter_entry',None)==entry:return
    ensure,reset,step,close=cls._ensure_sim,cls.reset,cls.step,cls.close
    def selected(bot):return getattr(bot,'task',None) is not None and bot.task.path==entry
    def create(bot,goal):
        if not selected(bot) or bot._env is not None:return ensure(bot,goal)
        if bridge._APP is None:
            from isaaclab.app import AppLauncher
            bridge._APP=AppLauncher(headless=bot.headless,enable_cameras=True,device=bot.device).app
            atexit.register(bridge._close_app_at_exit)
        from yam_arena import scene
        import isaaclab.sim as sim
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        fallback=scene.task_object
        def spawn(marker):
            if marker.name!='shutter_trainer':return fallback(marker)
            return Object(name=marker.name,object_type=ObjectType.BASE,spawner_cfg=sim.UsdFileCfg(usd_path=str(entry.with_name('shutter.usda'))),
                          initial_pose=Pose(position_xyz=tuple(marker.pos),rotation_xyzw=(0.,0.,0.,1.)))
        # Kit keeps the rendering device. The GPU rigid-body tensor view cannot set kinematic targets.
        with patch.object(scene, 'task_object', spawn), patch.object(bot, 'device', 'cpu'):
            return ensure(bot, goal)
    def start(bot,scene,*,seed=None):
        if not selected(bot):return reset(bot,scene,seed=seed)
        if getattr(bot,'_shutter',None) is not None:
            # Rebuild the compound. The arena only auto-resets top-level free objects.
            bot._shutter.close(); bot._env.close();bot._env=bot._base=None
        obs=reset(bot,scene,seed=seed)
        bot._shutter=Observer(bot._base,entry,seed)
        bot.task.module.monitor=bot._shutter
        if os.environ.get("SHUTTER_CONTROL") or os.environ.get("SCALE_RESULT"):
            from grip_gate import GripGate
            bot._shutter.grip_gate=GripGate(bot.task.module.demo())
            bot._shutter.policy_time=float(obs.state_time)
        return obs
    def advance(bot,action):
        if not selected(bot):return step(bot,action)
        from yam_arena import scene
        m=bot._shutter;m.tick+=1;m.sub=0
        if getattr(m,"grip_gate",None):
            m.check_grip(m.grip_gate.begin,m.policy_time+float(bot._base.step_dt))
        engine=bot._base.sim;advance_physics=engine.step
        def instrument(*args,**kwargs):
            m.drive()
            result=advance_physics(*args,**kwargs);m.measure();return result
        # The public task score reads the live compound-body tensor measurements.
        original_states=scene.object_states
        def states(env,names):
            if env is bot._env and tuple(names)==('shutter_trainer',):return {}
            return original_states(env,names)
        with patch.object(engine,'step',instrument), patch.object(scene,'object_states',states):
            result=step(bot,action)
        if getattr(m,"grip_gate",None):
            m.policy_time=float(result.observation.state_time)
        if m.sub!=m.decimation:raise RuntimeError(f'Missing physics substeps: expected {m.decimation}, measured {m.sub}')
        if m.fp:m.fp.flush()
        return result
    def finish(bot):
        if selected(bot) and getattr(bot,'_shutter',None):bot._shutter.close()
        return close(bot)
    cls._ensure_sim=create;cls.reset=start;cls.step=advance;cls.close=finish;cls._shutter_entry=entry
