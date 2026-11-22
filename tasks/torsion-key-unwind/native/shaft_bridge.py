"""YAM compound fixture and per-physics-step observation. This hook only instruments this task."""
import atexit
import json
import os
from pathlib import Path
from unittest.mock import patch
from shaft_contract import Trial, readings, HZ, PRELOAD
from shaft_demo_check import HubGrasp,ready,flank_trim
import numpy as np

def values(tensor):
    tensor=getattr(tensor,"torch",tensor)
    if hasattr(tensor,"detach"):tensor=tensor.detach().cpu()
    return np.asarray(tensor.numpy() if hasattr(tensor,"numpy") else tensor).reshape(7).tolist()

class Observer:
    def __init__(self,base,seed):
        from isaaclab_physx.physics import PhysxManager
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools
        import carb.settings
        from yam_arena.robot import asset_link_paths
        self.trial=Trial()
        self.ready_ticks=0
        self.flank_trimmed=False
        self.guard_passed=False
        self.hub_grasp=HubGrasp()
        self.turn_started=False
        self.hub_released=False
        self.left_rotor_contacts=0
        self.last_reading=None
        self.dt=float(base.sim.get_physics_dt())
        if abs(self.dt-1/HZ)>1e-7 or abs(float(base.step_dt)-.05)>1e-6:
            raise RuntimeError("Expected 240 Hz physics and 20 Hz actions")
        sim=PhysxManager.get_physics_sim_view()
        root="/World/envs/env_0/shaft_trainer"
        self.paths={root+"/"+name:name for name in ("frame","rotor","bearing","anchor","key")}
        self.views={name:sim.create_rigid_body_view(path) for path,name in self.paths.items()}
        if any(v.count!=1 for v in self.views.values()):raise ValueError("Missing native rigid body")
        self.robots=[str(base.scene[k].cfg.prim_path).replace("env_.*","env_0").replace("env_[^/]+","env_0") for k in ("left_robot","right_robot")]
        self.hands={r+asset_link_paths()[k] for r in self.robots for k in ("gripper","tip_left","tip_right")}
        self.decode=PhysicsSchemaTools.intToSdfPath
        self.events=set();self.key_seen=False;self.error=None
        self.settings=carb.settings.get_settings();self.setting="/physics/disableContactProcessing"
        self.previous=self.settings.get(self.setting);self.settings.set_bool(self.setting,False)
        self.sub=get_physx_simulation_interface().subscribe_contact_report_events(self.contact)
        self.stream=Path(os.environ["SHAFT_TRACE"]).open("x") if os.environ.get("SHAFT_TRACE") else None
        self.origin=values(self.views["frame"].get_transforms())
        if self.stream:
            self.stream.write(json.dumps(dict(kind="reset",dt=self.dt,seed=seed,origin=self.origin,robots=self.robots,hands=sorted(self.hands),fixture=self.paths))+"\n")
    def contact(self,headers,points):
        try:
            for h in headers:
                if h.num_contact_data:
                    self.events.add(tuple(sorted((str(self.decode(h.actor0)),str(self.decode(h.actor1))))))
        except Exception as e:self.error=e
    def measure(self):
        if self.error:raise self.error
        p={n:values(v.get_transforms()) for n,v in self.views.items()}
        a,r,k,ok=readings(p)
        ok=ok and np.allclose(p["frame"],self.origin,atol=.0002)
        touch=set()
        for x,y in self.events:
            if {self.paths.get(x),self.paths.get(y)}=={"key","rotor"}:self.key_seen=True
            robots=[next((i for i,path in enumerate(self.robots) if actor.startswith(path+"/")),None) for actor in (x,y)]
            if set(robots)=={0,1}:self.trial.failed=self.trial.failed or "inter-arm collision"
            for fixture,other,arm in ((x,y,robots[1]),(y,x,robots[0])):
                if fixture in self.paths and arm is not None:
                    touch.add((self.paths[fixture],other))
                    if other not in self.hands:self.trial.failed=self.trial.failed or "arm struck fixture"
        if self.trial.steps==120 and not self.key_seen:raise RuntimeError("No rotor-to-key contact observed at reset; inspect contact reporting and fixture.")
        self.trial.observe(a,r,k,not touch,ok)
        left_tips={link.rsplit("/",1)[-1] for part,link in touch
                   if part=="anchor" and link.startswith(self.robots[0]+"/")}
        self.hub_grasp.observe(a,{"tip_left","tip_right"}<=left_tips)
        self.left_rotor_contacts+=int(any(part=="rotor" and link.startswith(self.robots[0]+"/") for part,link in touch))
        key_tips={link.rsplit("/",1)[-1] for part,link in touch
                  if part=="key" and link.startswith(self.robots[1]+"/")}
        qualifies=ready(a,r,k,{"tip_left","tip_right"}<=key_tips,self.trial.failed)
        self.ready_ticks=self.ready_ticks+1 if qualifies else 0
        self.last_reading=dict(anchor_deg=a,rotor_deg=r,key_mm=k,residual_deg=PRELOAD+a-r,
                               key_tips=sorted(key_tips),ready_ticks=self.ready_ticks,
                               left_tips=sorted(left_tips),hub_grasp_ticks=self.hub_grasp.ticks,
                               bilateral_hub_turn_deg=self.hub_grasp.turn,left_rotor_contacts=self.left_rotor_contacts)
        if self.stream:
            self.stream.write(json.dumps(dict(tick=self.trial.steps,poses=p,touches=sorted(touch),pairs=sorted(self.events),score=self.trial.score,reason=self.trial.failed))+"\n")
        self.events.clear()
    def close(self):
        self.sub=None
        if self.previous is None:self.settings.destroy_item(self.setting)
        else:self.settings.set_bool(self.setting,self.previous)
        if self.stream:self.stream.close()

def install(entry):
    from yam_arena import inspect_embodiment as bridge
    cls=bridge.YamArenaEmbodiment
    if getattr(cls,"_shaft_entry",None)==entry:return
    from yam_arena.scripted import YamScriptedPolicy
    original_act=YamScriptedPolicy.act
    def act(policy,observation):
        task=policy._task
        if task is not None and task.path==entry and os.environ.get("SHAFT_CONTROL","reference") in ("reference","alternate","unload_only"):
            t=float(observation.state_time)+policy._dt
            label=policy._plans["right"].target(t)[3]
            m=task.module.monitor
            if m is None:raise RuntimeError("Missing scripted demonstration observer")
            ref=os.environ.get("SHAFT_CONTROL","reference")!="alternate"
            if not ref and label=="trim flank push" and not m.flank_trimmed:
                from shaft_motion import demo
                angle=m.last_reading["anchor_deg"]
                trim=flank_trim(angle)
                # Keep the left IK warm start, right arm, phase boundaries and guards.
                policy._plans["left"].waypoints=demo(trim_deg=trim)["left"]
                m.flank_trimmed=True
                if m.stream:
                    path=Path(m.stream.name).with_name("flank-trim.json")
                    path.write_text(json.dumps(dict(tick=m.trial.steps,input_deg=angle,
                        extra_deg=trim,command_deg=-34.0-trim),indent=2))
            reason=None
            if ref and m.left_rotor_contacts:
                reason="left arm contacted the white rotor"
            elif ref and label=="unwind input hub" and not m.turn_started:
                m.turn_started=m.hub_grasp.ticks>=12
                if not m.turn_started:reason="opposed left grasp incomplete before turn"
            elif ref and label=="release blue handle" and not m.hub_released:
                m.hub_released=m.hub_grasp.turn>=PRELOAD-6 and abs(PRELOAD+m.last_reading["anchor_deg"])<3
                if not m.hub_released:reason="opposed hub turn incomplete before release"
            elif label=="withdraw along rail" and not m.guard_passed:
                m.guard_passed=m.ready_ticks>=12
                if not m.guard_passed:reason="pre-pull check"
            if reason:
                if m.stream:
                    Path(m.stream.name).with_name("scripted-stop.json").write_text(
                        json.dumps(dict(reason=reason,measurement=m.last_reading),indent=2))
                from inspect_robots.types import Action,ActionChunk
                from inspect_robots_yam.packing import pack
                joints=[policy._plans[h].action7(max(0,t-policy._dt))[0] for h in ("left","right")]
                return ActionChunk(actions=[Action(data=pack(*joints),meta={"request_stop":True,"stop_detail":reason})],control_hz=1/policy._dt)
        return original_act(policy,observation)
    YamScriptedPolicy.act=act
    ensure,reset,step,close=cls._ensure_sim,cls.reset,cls.step,cls.close
    def selected(bot):return getattr(bot,"task",None) is not None and bot.task.path==entry
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
        def spawn(mark):
            if mark.name!="shaft_trainer":return fallback(mark)
            return Object(name=mark.name,object_type=ObjectType.BASE,spawner_cfg=sim.UsdFileCfg(usd_path=str(entry.with_name("shaft_gauges.usda"))),initial_pose=Pose(position_xyz=tuple(mark.pos),rotation_xyzw=(0.,0.,0.,1.)))
        with patch.object(scene,"task_object",spawn),patch.object(bot,"device","cpu"):
            return ensure(bot,goal)
    def start(bot,scene,*,seed=None):
        if not selected(bot):return reset(bot,scene,seed=seed)
        if getattr(bot,"_shaft",None):
            bot._shaft.close();bot._env.close();bot._env=bot._base=None
        obs=reset(bot,scene,seed=seed)
        bot._shaft=Observer(bot._base,seed);bot.task.module.monitor=bot._shaft
        return obs
    def advance(bot,action):
        if not selected(bot):return step(bot,action)
        from yam_arena import scene
        m=bot._shaft;engine=bot._base.sim;tick=engine.step;states=scene.object_states
        def sample(*a,**kw):
            v=tick(*a,**kw);m.measure();return v
        def get(env,names):
            return {} if env is bot._env and tuple(names)==("shaft_trainer",) else states(env,names)
        before=m.trial.steps
        with patch.object(engine,"step",sample),patch.object(scene,"object_states",get):
            result=step(bot,action)
        if m.trial.steps-before!=12:raise RuntimeError("Missing physics substeps")
        if m.stream:m.stream.flush()
        return result
    def finish(bot):
        if selected(bot) and getattr(bot,"_shaft",None):bot._shaft.close();bot._shaft=None
        return close(bot)
    cls._ensure_sim=create;cls.reset=start;cls.step=advance;cls.close=finish;cls._shaft_entry=entry

