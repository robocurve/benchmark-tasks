"""Contact-only synthetic expert: remove towel, then press and release COOK."""
import json
import time
from pathlib import Path
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY

FRONT=np.array([[0.,0,1],[-1,0,0],[0,-1,0]])


class Demo:
    def __init__(self,env,reader,*,recorder=None,output=None,parameters=None):
        self.env,self.reader,self.recorder=env,reader,recorder
        self.output=Path(output) if output else None
        self.parameters=parameters or {}
        self.command={side:np.r_[READY,1.] for side in ["left","right"]}
        self.steps,self.rows,self.phases=0,[],[]
        self.started=time.monotonic()
        self.worst_position_error=self.worst_rotation_error=0.

    def tick(self,label):
        import torch
        if self.steps>=2400:raise RuntimeError("Synthetic trial exceeded 2400 control steps")
        action=np.r_[self.command["left"],self.command["right"]]
        self.env.step(torch.as_tensor(action[None],device=self.reader.base.device,dtype=torch.float32))
        self.steps+=1;score=self.reader.score()
        if self.recorder:self.recorder.capture()
        row=dict(step=self.steps,time_s=self.steps/20,phase=label,action=action.tolist(),
            measured=self.reader.controls(),
            joint_pos=np.r_[self.reader.arm("left")[0],self.reader.arm("left")[1],
                            self.reader.arm("right")[0],self.reader.arm("right")[1]].tolist(),
            tcp={s:self.reader.arm(s)[2].tolist() for s in ["left","right"]},**score)
        self.rows.append(row)
        if self.output and self.steps%20==0:
            (self.output/"progress.json").write_text(json.dumps(row,indent=2)+"\n")
        return score

    def move(self,side,goal,rotation,seconds,grip,label,stop=None):
        q,g,start,r0=self.reader.arm(side);goal=np.array(goal,float)
        # Measured jaws stall on a held object. Reusing that opening as the
        # next command would remove grip preload at every phase transition.
        g=float(self.command[side][-1])
        self.phases.append(dict(label=label,start_step=self.steps,arm=side,target=goal.tolist()))
        print("DEMO_PHASE",label,goal.tolist(),flush=True)
        count=round(seconds*20);stopped=False
        for k in range(1,count+1):
            if stop is not None and stop():stopped=True;break
            t=ik.smoothstep(k/count)
            q,pe,re=ik.solve(start+t*(goal-start),
                ik.matrix_to_quat_xyzw(ik.slerp_matrix(r0,rotation,t)),q,
                base_pos=station.BASES[side],iters=120)
            self.worst_position_error=max(self.worst_position_error,float(pe))
            self.worst_rotation_error=max(self.worst_rotation_error,float(np.degrees(re)))
            if pe>.012 or re>np.radians(12):
                raise RuntimeError(f"Unreachable {label}: {pe*1000:.1f} mm / {np.degrees(re):.1f} degrees")
            self.command[side]=np.r_[q,g+t*(grip-g)]
            self.tick(label)
        if not stopped and np.linalg.norm(self.reader.arm(side)[2]-goal)>.025:
            raise RuntimeError(f"Blocked {label}: measured TCP missed its target")

    def hold(self,seconds,label):
        for _ in range(round(seconds*20)):result=self.tick(label)
        return result

    def grip(self,side,value,seconds,label):
        start=self.reader.arm(side)[1]
        for k in range(1,round(seconds*20)+1):
            self.command[side][-1]=start+ik.smoothstep(k/round(seconds*20))*(value-start)
            self.tick(label)

    def ready(self,side,seconds=2.):
        q,g,_,_=self.reader.arm(side);count=round(seconds*20)
        for k in range(1,count+1):
            t=ik.smoothstep(k/count)
            self.command[side]=np.r_[q+t*(np.array(READY)-q),g+t*(1-g)]
            self.tick(side+"_return_ready")

    def remove_towel(self):
        side="right";body=self.reader.body()
        a=np.radians(float(self.parameters.get("towel_pitch_deg",50)))
        x=np.array([0.,-1.,0.]);z=np.array([np.cos(a),0.,-np.sin(a)])
        rotation=np.column_stack([x,np.cross(z,x),z])
        grasp=body["grasp"].copy()
        self.move(side,grasp-z*.10,rotation,2.5,1.,"towel_hover")
        self.move(side,grasp,rotation,2.,1.,"towel_enter")
        self.grip(side,float(self.parameters.get("towel_grip",.30)),.7,"towel_grasp")
        before=self.reader.body()["pose"][:3].copy()
        self.move(side,self.reader.arm(side)[2]+[0,0,.065],rotation,2.5,
                  self.command[side][-1],"towel_lift")
        body=self.reader.body()
        if body["pose"][2]<before[2]+.035:raise RuntimeError("Towel was not physically lifted")
        offset=self.reader.arm(side)[2]-body["pose"][:3]
        tray=np.array(self.reader.specs["towel_tray"].position)
        root_goal=np.array([tray[0],tray[1],body["pose"][2]])
        self.move(side,root_goal+offset,rotation,3.,self.command[side][-1],"towel_over_tray")
        body=self.reader.body();floor=tray[2]+.008
        root_goal=body["pose"][:3].copy();root_goal[2]+=floor+.001-body["low"][2]
        offset=self.reader.arm(side)[2]-body["pose"][:3]
        self.move(side,root_goal+offset,rotation,3.,self.command[side][-1],"towel_lower",
                  stop=lambda:self.reader.body()["low"][2]<=floor+.002)
        self.hold(.35,"towel_settle_in_grip")
        self.grip(side,1.,.7,"towel_release")
        self.move(side,self.reader.arm(side)[2]+[0,0,.12],rotation,2.,1.,"towel_retreat")
        self.ready(side)
        self.hold(.6,"towel_safe_hold")
        state=self.reader.controls()
        if not state["vent_clear"] or not state["towel_supported"]:
            raise RuntimeError("Towel is not safely clear and supported after release")

    def press_cook(self):
        side="left";spec=self.reader.specs["cook_button"]
        root=self.reader.button_rotation;rotation=root@FRONT
        touch=np.array(spec.position)+root@np.array([-.025,0,0])
        direction=root[:,0]
        self.move(side,touch-direction*.075,rotation,2.5,0.,"cook_align")
        self.move(side,touch,rotation,2.,0.,"cook_touch")
        self.move(side,touch+direction*.017,rotation,2.,0.,"cook_press",
                  stop=lambda:self.reader.controls()["button_travel_m"]>=.006)
        self.hold(.4,"cook_press_hold")
        if not self.reader.scorer.started:raise RuntimeError("COOK did not activate from measured plunger travel")
        self.move(side,touch-direction*.080,rotation,2.,0.,"cook_withdraw")
        self.ready(side)
        return self.hold(1.5,"running_clear_stable_hold")

    def run(self):
        mode=self.parameters.get("mode","demo")
        if mode=="unsafe_start":
            self.press_cook()
            if not self.reader.scorer.unsafe_start_ever:raise RuntimeError("Unsafe start was not detected")
            self.remove_towel()
            return self.hold(1.5,"unsafe_recovery_stays_zero")
        self.remove_towel()
        if mode=="towel_only":return self.hold(1.,"clear_only_hold")
        return self.press_cook()

    def report(self):
        if self.output and self.rows:
            with (self.output/"trajectory.jsonl").open("w") as out:
                for row in self.rows:out.write(json.dumps(row)+"\n")
            np.savez_compressed(self.output/"trajectory.npz",
                actions=np.array([r["action"] for r in self.rows]),
                towel_poses=np.array([r["measured"]["towel_pose"] for r in self.rows]),control_hz=20)
        return dict(control_steps=self.steps,simulated_seconds=self.steps/20,
            wall_seconds=time.monotonic()-self.started,parameters=self.parameters,phases=self.phases,
            worst_ik_position_error_m=self.worst_position_error,
            worst_ik_rotation_error_deg=self.worst_rotation_error,
            object_motion="Robot contacts and friction only; no attachments or object/control pose writes during trial")


class Replay(Demo):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.actions=np.load(self.parameters["replay_file"])["actions"]
        if self.actions.ndim!=2 or self.actions.shape[1]!=14 or len(self.actions)>2380 or not np.isfinite(self.actions).all():
            raise ValueError("Expected finite 14-D reference commands within the run limit")

    def run(self):
        for row in self.actions:
            self.command["left"]=row[:7].copy();self.command["right"]=row[7:].copy()
            self.tick("joint_trajectory_replay")
        return self.hold(1.,"replay_final_hold")
