"""Physical scoop-and-serve policy; only robot joint and gripper actions."""
import json
from pathlib import Path
import time
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
from scoring import rotation

DOWN=np.array([[0,-1,0],[-1,0,0],[0,0,-1]],float)
GRASP=np.array([-.069,0,.027])
CUP=np.array([.054,0,.0035])
FRONT=np.array([.104,0,.0021])


def spoon_rotation(yaw=0,pitch=0):
    a,t=np.radians([yaw,pitch])
    rz=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
    ry=np.array([[np.cos(t),0,np.sin(t)],[0,1,0],[-np.sin(t),0,np.cos(t)]])
    return rz@ry


class Demo:
    def __init__(self,env,reader,*,recorder=None,output=None,parameters=None):
        import torch
        self.torch,self.env,self.reader=torch,env,reader
        self.recorder=recorder;self.output=Path(output) if output else None
        self.parameters=parameters or {}
        self.command={s:np.r_[READY,1.] for s in ('left','right')}
        self.steps=0;self.rows=[];self.phases=[];self.started=time.monotonic()
        self.worst_position_error=0.;self.worst_rotation_error=0.
        self.tool_calibration=None
        self.action_log=(self.output/'actions.jsonl').open('w') if self.output else None

    def tick(self,label):
        action=np.r_[self.command['left'],self.command['right']]
        self.env.step(self.torch.as_tensor(action[None],dtype=self.torch.float32,device=self.reader.base.device))
        self.steps+=1;score=self.reader.score()
        if self.action_log:self.action_log.write(json.dumps(action.tolist())+'\n')
        if self.recorder:self.recorder.capture()
        if self.steps%5==0:
            row={'step':self.steps,'phase':label,'spoon_pose':self.reader.body('CleanSpoon').pose.tolist(),
                 'meatball_poses':{n:self.reader.body(n).pose.tolist() for n in self.reader.scorer.balls},
                 'tcp':{s:self.reader.arm(s)[2].tolist() for s in ('left','right')},**score}
            self.rows.append(row)
            if self.output and self.steps%20==0:(self.output/'progress.json').write_text(json.dumps(row,indent=2)+'\n')
        return score

    def phase(self,label,**kwargs):
        self.phases.append({'label':label,'start_step':self.steps,**kwargs})
        print('DEMO_PHASE',label,kwargs,flush=True)

    def check_ik(self,pe,re,label):
        self.worst_position_error=max(self.worst_position_error,float(pe))
        self.worst_rotation_error=max(self.worst_rotation_error,float(np.degrees(re)))
        if pe>.015 or np.degrees(re)>12:
            raise RuntimeError(f'Unreachable {label}: {pe*1000:.1f} mm, {np.degrees(re):.1f} degrees')

    def move(self,goal,rgoal,seconds,grip,label):
        q,g0,start,r0=self.reader.arm('left');goal=np.array(goal,float)
        g0=float(self.command['left'][6])
        self.phase(label,tcp_target=goal.tolist())
        count=round(seconds*20)
        for k in range(1,count+1):
            t=ik.smoothstep(k/count);p=start+t*(goal-start);r=ik.slerp_matrix(r0,rgoal,t)
            q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),q,base_pos=station.BASES['left'],iters=100)
            self.check_ik(pe,re,label);self.command['left']=np.r_[q,g0+t*(grip-g0)]
            self.tick(label)

    def hold(self,seconds,grip,label):
        self.phase(label);q=self.command['left'][:6].copy();g0=float(self.command['left'][6])
        count=round(seconds*20)
        for k in range(1,count+1):
            t=ik.smoothstep(k/count);self.command['left']=np.r_[q,g0+t*(grip-g0)]
            self.tick(label)

    def cup_pose(self):
        spoon=self.reader.body('CleanSpoon');r=rotation(spoon.pose[3:])
        return spoon.pose[:3]+r@CUP,r

    def move_cup(self,goal,rgoal,seconds,label):
        """Preserve the physical grasp calibration, with joint tracking correction.

        Adapted from the existing clean_ice_fish_b1 scoop controller. Calibration
        is a command frame only; no physical attachment or pose write is used.
        """
        measured_q,_,tcp,er=self.reader.arm('left')
        if self.tool_calibration is None:
            spoon=self.reader.body('CleanSpoon');sr=rotation(spoon.pose[3:])
            self.relative_rotation=er.T@sr
            self.bucket_offset=er.T@(spoon.pose[:3]+sr@CUP-tcp)
            self.tracking_bias=self.command['left'][:6]-measured_q
            self.tool_calibration={'relative_rotation':self.relative_rotation.tolist(),
                                   'bucket_offset':self.bucket_offset.tolist()}
        start=tcp+er@self.bucket_offset;r0=er@self.relative_rotation
        goal=np.array(goal,float);count=round(seconds*20)
        self.phase(label,cup_target=goal.tolist())
        for k in range(1,count+9):
            t=ik.smoothstep(min(k/count,1.));desired=start+t*(goal-start)
            dr=ik.slerp_matrix(r0,rgoal,t);rr=dr@self.relative_rotation.T
            p=desired-rr@self.bucket_offset
            measured_q,_,tcp,er=self.reader.arm('left')
            spoon=self.reader.body('CleanSpoon');sr=rotation(spoon.pose[3:])
            if np.linalg.norm(tcp-(spoon.pose[:3]+sr@GRASP))>.055:
                raise RuntimeError('Scoop left the physical fingers during '+label)
            q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(rr),measured_q,
                             base_pos=station.BASES['left'],iters=150,damping=.025)
            self.check_ik(pe,re,label)
            error=q-measured_q
            self.tracking_bias=np.clip(self.tracking_bias+.06*error,-.18,.18)
            command=np.clip(q+self.tracking_bias+.25*error,ik.LOWER,ik.UPPER)
            command=np.clip(command,self.command['left'][:6]-.12,self.command['left'][:6]+.12)
            self.command['left']=np.r_[command,0.]
            self.tick(label)

    def ready(self):
        q,g,_,_=self.reader.arm('left');self.phase('return_ready')
        for k in range(1,41):
            t=ik.smoothstep(k/40);self.command['left']=np.r_[q+t*(np.array(READY)-q),g+t*(1-g)]
            self.tick('return_ready')

    def run(self):
        top=self.reader.layout['table_top_z']
        initial=self.reader.body('CleanSpoon');r=rotation(initial.pose[3:]);grasp=initial.pose[:3]+r@GRASP
        self.move(grasp+[0,0,.13],r@DOWN,2.25,.60,'above_clean_handle')
        self.move(grasp,r@DOWN,2.75,.60,'descend_to_handle')
        self.hold(1.25,0.,'grasp_clean_spoon')
        self.move(grasp+[0,0,.11],r@DOWN,2.75,0.,'lift_clean_spoon')
        self.hold(1.,0.,'hold_clean_spoon')
        ball_name=f"Meatball_{self.parameters.get('ball_index',1)}"
        center=self.reader.body(ball_name).center.copy()
        yaw=float(self.parameters.get('scoop_yaw_degrees',-45))
        pitch=float(self.parameters.get('scoop_pitch_degrees',0))
        entry_r=spoon_rotation(yaw,pitch);long=spoon_rotation(yaw,0)[:,0]
        lip=center.copy()-long*.025;lip[2]=top+float(self.parameters.get('entry_lip_z',.0064))
        entry=lip+entry_r@(CUP-FRONT)
        above=entry.copy();above[2]=top+float(self.parameters.get('above_cup_z',.075))
        self.move_cup(above,entry_r,4.,'above_source_bowl')
        self.move_cup(entry,entry_r,4.,'lower_scoop_lip')
        end=entry+long*float(self.parameters.get('scoop_stroke_m',.095))
        self.move_cup(end,entry_r,4.,'scoop_under_meatball')
        cup,_=self.cup_pose()
        centered=self.reader.body(ball_name).center.copy()-long*.008
        centered[2]=max(cup[2]+.006,top+float(self.parameters.get('curl_height_m',.060)))
        self.move_cup(centered,spoon_rotation(yaw,-15),float(self.parameters.get('curl_seconds',.6)),'curl_meatball_into_scoop')
        target=self.reader.body(ball_name).center.copy();target[2]=top+.095
        self.move_cup(target,spoon_rotation(yaw,-15),1.25,'level_and_lift_meatball')
        self.move_cup(target,spoon_rotation(yaw,-15),1.,'hold_scooped_meatball')
        if not self.reader.scorer.transported:
            raise RuntimeError('No physical scoop recorded; refusing to pour an empty spoon')
        plate=np.array(self.reader.layout['objects']['Plate']['position'])
        self.move_cup(plate+[0,0,.090],spoon_rotation(0,-10),5.,'carry_to_empty_plate')
        self.move_cup(plate+[0,0,.060],spoon_rotation(45,75),5.,'pour_one_meatball')
        self.move_cup(plate+[0,0,.060],spoon_rotation(45,75),2.,'wait_for_meatball_release')
        self.move_cup(plate+[.07,.08,.095],spoon_rotation(45,0),3.,'withdraw_spoon')
        clean=np.array(self.reader.layout['objects']['CleanSpoon']['position'])
        self.move_cup(clean+CUP+[0,0,.12],np.eye(3),4.,'return_spoon_above_clean_plate')
        self.move_cup(clean+CUP+[0,0,.002],np.eye(3),3.,'lower_spoon_to_clean_plate')
        self.hold(1.5,1.,'release_spoon')
        _,_,tcp,er=self.reader.arm('left')
        self.move(tcp+[0,0,.12],er,2.,1.,'retreat_from_spoon')
        self.ready()
        for _ in range(40):outcome=self.tick('final_hold')
        return outcome

    def report(self):
        if self.action_log and not self.action_log.closed:self.action_log.close()
        return {'control_steps':self.steps,'simulated_seconds':self.steps/20,
                'wall_seconds':time.monotonic()-self.started,'trajectory':self.rows,'phases':self.phases,
                'parameters':self.parameters,'grasp_calibration':self.tool_calibration,'worst_ik_position_error_m':self.worst_position_error,
                'worst_ik_rotation_error_deg':self.worst_rotation_error,
                'contacts':self.reader.contacts.report(),'scoring_events':self.reader.scorer.history,
                'object_motion':'Robot joint/gripper actions and physical contacts only; no attachments or object pose writes'}
