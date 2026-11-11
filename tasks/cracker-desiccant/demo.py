"""Contact-only expert: set the packet aside and transfer all six crackers."""
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

    def remove_packet(self, unsafe=False):
        side='right' if unsafe else 'left';b=self.reader.body('desiccant_packet')
        pitch=np.radians(float(self.parameters.get('packet_pitch_deg',60)))
        x=np.array([0.,-1.,0.]);z=np.array([np.cos(pitch),0,-np.sin(pitch)])
        rotation=np.column_stack([x,np.cross(z,x),z])
        grasp=b['grasp']+np.array([0,0,float(self.parameters.get('packet_raise_m',.002))])
        self.move(side,grasp-z*.12,rotation,2.5,.80,'packet_hover')
        self.move(side,grasp,rotation,2.,.80,'packet_enter')
        self.grip(side,float(self.parameters.get('packet_grip',.35)),.8,'packet_grasp')
        self.hold(.25,'packet_grip_settle')
        initial=b['low'][2]
        goal=self.reader.body()['pose'][:3].copy();goal[2]+=.12
        self.carry(side,'desiccant_packet',goal,2.5,'packet_lift')
        if self.reader.body()['low'][2]<initial+.06:raise RuntimeError('Packet did not lift')
        goal[:2]=[.22,.30] if not unsafe else self.reader.specs['serving_basket'].position[:2]
        self.carry(side,'desiccant_packet',goal,3.,'packet_move_aside' if not unsafe else 'packet_over_basket')
        b=self.reader.body();goal=b['pose'][:3].copy()
        floor=self.reader.layout['table_top_z']+(.012 if unsafe else 0)
        goal[2]+=floor+.003-b['low'][2]
        self.carry(side,'desiccant_packet',goal,2.5,'packet_lower')
        self.command[side][:6]=self.reader.arm(side)[0]
        self.grip(side,.80,.7,'packet_release')
        self.move(side,self.reader.arm(side)[2]+[0,0,.13],self.reader.arm(side)[3],2.,.80,'packet_retreat')
        self.ready(side);return self.hold(1.,'packet_rest')

    def transfer_cracker(self,name,index):
        side='right';body=self.reader.body(name)
        pitch=np.radians(float(self.parameters.get('cracker_pitch_deg',45)))
        # Side pinch across the cracker's Y direction, with the wrist above/front.
        yaw=np.arctan2(body['rotation'][1,0],body['rotation'][0,0])
        x=np.array([np.sin(yaw),-np.cos(yaw),0.])
        z=np.array([np.cos(pitch)*np.cos(yaw),np.cos(pitch)*np.sin(yaw),-np.sin(pitch)])
        rotation=np.column_stack([x,np.cross(z,x),z])
        grasp=body['grasp']+[0,0,float(self.parameters.get('cracker_raise_m',.004))]
        self.move(side,grasp-z*.11,rotation,2.,.85,name+'_hover')
        self.move(side,grasp,rotation,1.7,.85,name+'_enter')
        self.grip(side,float(self.parameters.get('cracker_grip',.43)),.7,name+'_grasp')
        self.hold(.25,name+'_grip_settle')
        before=body['low'][2];goal=self.reader.body(name)['pose'][:3].copy();goal[2]+=.105
        if goal[0]>.43:goal[0]-=.035  # Bring the far crackers inward while lifting to preserve reach margin.
        self.carry(side,name,goal,2.,name+'_lift')
        if self.reader.body(name)['low'][2]<before+.06:raise RuntimeError(name+' did not lift')
        basket=np.array(self.reader.specs['serving_basket'].position)
        goal[:2]=basket[:2]+[(-.045 if index%2==0 else .045),0.]
        self.carry(side,name,goal,2.5,name+'_over_basket')
        body=self.reader.body(name);goal=body['pose'][:3].copy()
        # Release just over the open rim; gravity settles the rigid cracker.
        goal[2]+=basket[2]+.074-body['low'][2]
        self.carry(side,name,goal,1.8,name+'_lower')
        self.command[side][:6]=self.reader.arm(side)[0]
        self.grip(side,.85,.6,name+'_release')
        self.move(side,self.reader.arm(side)[2]+[0,0,.10],self.reader.arm(side)[3],1.5,.85,name+'_retreat')
        self.hold(1.,name+'_settle')

    def run(self):
        mode=self.parameters.get('mode','demo')
        self.remove_packet(unsafe=mode=='unsafe_packet')
        if mode in ['packet_only','unsafe_packet']:return self.hold(1.,'packet_only_final')
        for index,name in enumerate(self.parameters.get('cracker_order',['cracker_2','cracker_1','cracker_3','cracker_4','cracker_5','cracker_6'])):
            if self.reader.controls()["eligible"][name]:
                continue  # A physical grasp may carry the two initially stacked crackers together.
            self.transfer_cracker(name,index)
        self.ready('right')
        return self.hold(1.5,'all_crackers_released_stable')

    def report(self):
        if self.output and self.rows:
            with (self.output/"trajectory.jsonl").open("w") as out:
                for row in self.rows:out.write(json.dumps(row)+"\n")
            np.savez_compressed(self.output/"trajectory.npz",
                actions=np.array([r["action"] for r in self.rows]),
                object_poses=np.array([[r["measured"]["bodies"][n]["pose"] for n in sorted(r["measured"]["bodies"])] for r in self.rows]),control_hz=20)
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
