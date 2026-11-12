"""Contact-only expert: load the jug and lid, leaving the motor outside."""
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
        self.sample_label='initial';self.substep_count=0
        self.substep_file=(self.output/'physics_substeps.jsonl').open('w') if self.output else None
        base=self.reader.base
        if base._physics_handles_decimation:raise RuntimeError('Need individual physics updates for vibration screening')
        self.original_scene_update=base.scene.update
        def measured_update(dt):
            self.original_scene_update(dt)
            if not self.substep_file or not any(k in self.sample_label for k in ['settle','hold','initial_rest','stable']):return
            from demo_runtime import numpy
            row=dict(sample=self.substep_count,dt=float(dt),phase=self.sample_label,
                bodies={n:dict(pose=numpy(base.scene[n].data.root_link_pose_w)[0].tolist(),velocity=numpy(base.scene[n].data.root_com_vel_w)[0].tolist()) for n in self.reader.vertices},
                arms={side:dict(joint_pos=numpy(base.scene[side+'_robot'].data.joint_pos)[0].tolist(),
                    net_normal_contact_forces_n=numpy(base.scene[side+'_contacts'].data.net_normal_forces_w)[0].tolist(),
                    contact_body_names=list(base.scene[side+'_contacts'].body_names)) for side in ['left','right']})
            self.substep_file.write(json.dumps(row)+'\n');self.substep_count+=1
        base.scene.update=measured_update

    def tick(self,label):
        import torch
        self.sample_label=label
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

    def carry(self, side, name, goal, seconds, label, rotation_goal=None):
        body=self.reader.body(name)
        start,r0=body['pose'][:3].copy(),body['rotation'].copy()
        # Keep the grasp transform fixed during each segment. Re-estimating it
        # from contact motion and accumulating joint corrections creates an
        # outer feedback loop around the native joint PD drives.
        tcp_relative=r0.T@(self.reader.arm(side)[2]-start)
        wrist_relative=r0.T@self.reader.arm(side)[3]
        q=self.reader.arm(side)[0].copy()
        if rotation_goal is None:rotation_goal=r0
        goal=np.asarray(goal,float);grip=float(self.command[side][-1])
        count=round(seconds*20)
        seed=q.copy()
        # Near a straight elbow, a smooth Cartesian path can demand a fast
        # joint movement. Re-time the whole segment before executing it;
        # do not clamp individual joints and distort the grasp orientation.
        for attempt in range(5):
            q=seed.copy();trajectory=[]
            for k in range(1,count+1):
                t=ik.smoothstep(k/count);desired=start+t*(goal-start)
                desired_r=ik.slerp_matrix(r0,rotation_goal,t)
                target=desired+desired_r@tcp_relative
                rotation=desired_r@wrist_relative
                q,pe,re=ik.solve(target,ik.matrix_to_quat_xyzw(rotation),q,base_pos=station.BASES[side],iters=120)
                self.worst_position_error=max(self.worst_position_error,float(pe))
                self.worst_rotation_error=max(self.worst_rotation_error,float(np.degrees(re)))
                if pe>.012 or re>np.radians(12):raise RuntimeError(f'Unreachable {label}: {pe*1000:.1f} mm / {np.degrees(re):.1f} deg')
                trajectory.append(q.copy())
            peak=float(np.abs(np.diff(np.vstack([self.command[side][:6],trajectory]),axis=0)).max())
            if peak<=.06:break
            count=int(np.ceil(count*max(1.2,peak/.05)))
        else:raise RuntimeError('Could not generate a smooth joint path for '+label)
        if self.steps+count>2380:raise RuntimeError('Re-timed carry exceeds the trial budget')
        self.phases.append(dict(label=label,start_step=self.steps,arm=side,object=name,target=goal.tolist(),duration_s=count/20,joint_target_step_peak_rad=peak))
        print('DEMO_PHASE',label,goal.tolist(),'duration',count/20,flush=True)
        for q in trajectory:
            body=self.reader.body(name);_,_,tcp,_=self.reader.arm(side)
            if np.linalg.norm(tcp-body['grasp'])>.07:raise RuntimeError(name+' left physical grip in '+label)
            self.command[side]=np.r_[q,grip];self.tick(label)

    def transfer(self,name,side,destination):
        body=self.reader.body(name)
        if name=='blender_jug':
            # Pinch the vertical handle across Y, approaching from above/front.
            pitch=np.radians(float(self.parameters.get('jug_pitch_deg',15)))
            x=np.array([0.,-1.,0.]);z=np.array([np.cos(pitch),0.,-np.sin(pitch)])
            rotation=np.column_stack([x,np.cross(z,x),z])
            closed=float(self.parameters.get('jug_grip',.16))
        else:
            pitch=np.radians(float(self.parameters.get('lid_pitch_deg',65)))
            x=np.array([0.,-1.,0.]);z=np.array([np.cos(pitch),0.,-np.sin(pitch)])
            rotation=np.column_stack([x,np.cross(z,x),z])
            closed=float(self.parameters.get('lid_grip',.18))
        grasp=body['grasp']
        opened=.45 if name=='blender_jug' else .8
        self.move(side,grasp-rotation[:,2]*(.05 if name=='blender_jug' else .12),rotation,3.,opened,name+'_hover')
        self.move(side,grasp,rotation,2.5,opened,name+'_enter')
        self.grip(side,closed,1.,name+'_grasp')
        self.hold(1.,name+'_grip_settle')
        initial=body['low'][2]
        goal=self.reader.body(name)['pose'][:3].copy();goal[2]+=.16
        self.carry(side,name,goal,3.,name+'_lift')
        if self.reader.body(name)['low'][2]<initial+.10:raise RuntimeError(name+' did not lift')
        self.hold(1.,name+'_lift_hold')
        goal[:2]=destination
        self.carry(side,name,goal,5.,name+'_over_rack')
        body=self.reader.body(name);goal=body['pose'][:3].copy()
        floor=self.reader.specs['dishwasher_rack'].position[2]+.0083
        goal[2]+=floor+.002-body['low'][2]
        self.carry(side,name,goal,3.,name+'_lower')
        self.command[side][:6]=self.reader.arm(side)[0]
        self.grip(side,.8,1.,name+'_release')
        self.move(side,self.reader.arm(side)[2]-rotation[:,2]*.14,rotation,2.5,.8,name+'_retreat')
        self.ready(side,3.)
        return self.hold(2.,name+'_settled')

    def run(self):
        self.hold(1.,'initial_rest')
        if self.parameters.get('mode')=='jug_only':
            self.transfer('blender_jug','left',[.380,-.205])
            return self.hold(2.,'jug_only_final')
        # Load the low lid first so its carry path cannot sweep through a tall jug.
        self.transfer('blender_lid','right',[.380,-.333])
        self.move('right',[.27,-.38,1.04],self.reader.arm('right')[3],2.5,1.,'right_clear_loading_area')
        self.transfer('blender_jug','left',[.380,-.205])
        return self.hold(2.,'both_parts_released_stable')

    def report(self):
        self.reader.base.scene.update=self.original_scene_update
        if self.substep_file:self.substep_file.close()
        if self.output and self.rows:
            with (self.output/"trajectory.jsonl").open("w") as out:
                for row in self.rows:out.write(json.dumps(row)+"\n")
            np.savez_compressed(self.output/"trajectory.npz",
                actions=np.array([r["action"] for r in self.rows]),
                object_poses=np.array([[r["measured"]["bodies"][n]["pose"] for n in sorted(r["measured"]["bodies"])] for r in self.rows]),control_hz=20)
        return dict(control_steps=self.steps,simulated_seconds=self.steps/20,substep_samples=self.substep_count,
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
