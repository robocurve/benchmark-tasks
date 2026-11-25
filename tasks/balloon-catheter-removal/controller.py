"""Measured-pose Cartesian waypoint servo; commands robot joints/grippers only.

The planner may read task poses, as allowed for scripted demos. Every object
continues to move through native contacts. No root-pose writes or grasp joints.
"""
import json
from pathlib import Path
import numpy as np
from yam_arena import ik, station
from yam_arena.demo import grip_rotation
from yam_arena.constants import PAD_TIP_PAST_TCP, READY
from mechanics import point, rotation
from layout import PLACEMENT_ROOT_TARGET,PLACEMENT_CARRY,PLACEMENT_APPROACH,PLACEMENT_LOWER_START

PHASES=[('right_hover',2.),('right_lower',3.),('right_close',2.),('right_lift',2.),
 ('right_orient',6.),('left_hover',3.),('left_lower',3.),('left_close',1.5),('left_lift',3.),
 ('preconnect',12.),('insert',6.),('drain',10.),('pull_off',3.),('receiver_hover',6.),
 ('receiver_lower',3.),('receiver_release',1.),('receiver_retreat',2.),('valve_release',6.),
 ('right_clear',2.),('hub_hover',3.),('hub_lower',8.),('hub_close',2.),
 ('align_shaft',4.),('withdraw',10.),('shaft_lift',2.),('shaft_carry',4.),
 ('shaft_lower',32.),('shaft_release',1.),('final_retreat',3.),('finish',4.)]

# A shallow downward shaft inclination lifts the proximal hub clear of its
# staging supports while the measured balloon stays on the neck centreline.
SHAFT_AXIS_XZ=np.array([0.,-.025])

def align_y(r):
    a=r[:,1];b=np.array([0.,1.,0.]);v=np.cross(a,b);c=float(np.dot(a,b))
    if c>.999999:return r.copy()
    if c<-.9999:raise RuntimeError('Catheter axial direction unexpectedly reversed')
    k=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return (np.eye(3)+k+k@k/(1+c))@r

class Controller:
    def __init__(self,env,plans):
        self.env=env.unwrapped;self.runtime=self.env._catheter_dynamics.runtime
        self.phase=-1;self.start=0.;self.last=0.;self.q={};self.hold={};self.bias={}
        self.rotation_bias={s:np.zeros(3) for s in ('left','right')}
        self.ik_feasible={s:True for s in ('left','right')}
        self.connector_bias=np.zeros(3)
        self.connection_time=None
        self.hub_joint_bias=np.zeros(6)
        self.shaft_position_bias=np.zeros(3);self.shaft_rotation_bias=np.zeros(3)
        self.last_command_q={};self.visual_q=None;self.body_servo_q={}
        for side in ('left','right'):
            q=self.joints(side);pos,rot=ik.tcp_pose(q,station.BASES[side])
            self.q[side]=q;self.hold[side]=(pos,rot,1.);self.bias[side]=np.zeros(3)
            self.last_command_q[side]=q.copy()
        self.grasps={};self.targets={};self.reference={};self.fixed={};self.log=Path('/outputs/catheter-controller.jsonl')
        self.valve_grip=.2875
        self.valve_open=.42
        self.receiver_grip=.15
        self.hub_grip=.18
        self.initial=self.runtime.poses();self.fail=None

    def joints(self,side):
        robot=self.env.scene[side+'_robot'];a=robot.data.joint_pos
        a=a.torch if hasattr(a,'torch') else a
        q=a[0].detach().cpu().numpy()
        return np.array([q[robot.joint_names.index('joint'+str(i))] for i in range(1,7)])

    def latch_grasp(self,side,name,p):
        tcp,r=ik.tcp_pose(self.joints(side),station.BASES[side]);rb=rotation(p[name][3:])
        self.grasps[(side,name)]=(rb.T@(tcp-p[name][:3]),rb.T@r)

    def valve_grasp_goal(self,p,closing=False):
        r=rotation(p['valve'][3:]);theta=np.deg2rad(60.)
        approach=np.cos(theta)*r[:,1]-np.sin(theta)*r[:,2];closing_axis=r[:,0]
        tool=np.column_stack([closing_axis,np.cross(approach,closing_axis),approach])
        # Oblique approach raises the back of the finger above the adjacent
        # hub; the original horizontal approach touched its blue tab.
        pick=point(p['valve'],[0.,.024,.014])-approach*PAD_TIP_PAST_TCP
        return pick,tool

    def body_goal(self,side,name,pos,r,grip=.0):
        if side=='left' and name=='barrel' and grip==0.:grip=self.receiver_grip
        offset,relative=self.grasps[(side,name)]
        return np.asarray(pos)+r@offset,r@relative,grip

    def body_servo(self,side,name,p,target,target_rot,grip):
        """Replan native IK from the measured freely contacted body/hand pose."""
        offset,relative=self.grasps[(side,name)]
        wanted_r=target_rot@relative.T;wanted_p=target-wanted_r@offset
        body_p=p[name][:3];body_r=rotation(p[name][3:])
        measured_q=self.joints(side);tcp,hand_r=ik.tcp_pose(measured_q,station.BASES[side])
        local_p=body_r.T@(tcp-body_p);local_r=body_r.T@hand_r
        wanted_tcp=wanted_p+wanted_r@local_p;wanted_hand=wanted_r@local_r
        q,pe,re=ik.solve(wanted_tcp,ik.matrix_to_quat_xyzw(wanted_hand),self.q[side],base_pos=station.BASES[side],iters=100)
        if pe>.010 or re>.15:
            failure={'phase':PHASES[self.phase][0],'previous_control_time_s':self.last,'body':name,
                'desired_tcp':wanted_tcp.tolist(),'desired_rotation':wanted_hand.tolist(),'measured_q':measured_q.tolist(),
                'solver_seed':self.q[side].tolist(),'solver_q':q.tolist(),'position_error_m':pe,'rotation_error_rad':re,
                'poses':{k:v.tolist()for k,v in p.items()}}
            Path('/outputs/receiver-ik-failure.json').write_text(json.dumps(failure,indent=2))
            raise RuntimeError('Measured receiver grasp left the validated IK workspace')
        bias=self.body_servo_q.get(side,np.zeros(6))
        bias=np.clip(bias+np.clip(.015*(q-measured_q),-.0005,.0005),-.10,.10);self.body_servo_q[side]=bias
        command=np.clip(q+bias,np.maximum(ik.LOWER,measured_q-.20),np.minimum(ik.UPPER,measured_q+.20))
        goal,rot=ik.tcp_pose(command,station.BASES[side])
        error=np.r_[wanted_p-body_p,.10*.5*sum((np.cross(body_r[:,i],wanted_r[:,i]) for i in range(3)),np.zeros(3))]
        diagnostic={'mode':'measured_object_pose_ik','body':name,'goal_tcp':goal.tolist(),'measured_tcp':tcp.tolist(),
            'goal_rotation':rot.tolist(),'measured_rotation':hand_r.tolist(),'body_goal_position':wanted_p.tolist(),
            'body_measured_position':body_p.tolist(),'body_goal_rotation':wanted_r.tolist(),'body_measured_rotation':body_r.tolist(),
            'feature_error':error.tolist(),'joint_bias':bias.tolist(),'ik_error_m':pe,'ik_error_rad':re}
        return command,goal,rot,diagnostic

    def shaft_servo(self,p,desired_y):
        """Increment native joint targets using the measured hand/object Jacobian.

        Balloon centering, shaft direction, level hub-tab roll and the next
        physical axial waypoint are controlled. Keeping the tab level clears
        its passive staging ledges before any axial pull.
        """
        q=self.joints('right');tcp,hand_r=ik.tcp_pose(q,station.BASES['right'])
        body_p=p['catheter'][:3];body_r=rotation(p['catheter'][3:])
        offset=hand_r.T@(body_p-tcp);relative=hand_r.T@body_r
        def features(pos,rot):
            ball=pos+.132*rot[:,1]
            return np.array([ball[0],ball[2],.10*rot[0,1],.10*rot[2,1],.10*rot[2,0],pos[1]])
        measured=features(body_p,body_r);target=np.array([.375,.851,.10*SHAFT_AXIS_XZ[0],.10*SHAFT_AXIS_XZ[1],0.,desired_y]);error=target-measured
        jac=np.zeros((6,6));eps=1.e-5
        for j in range(6):
            probe=q.copy();probe[j]+=eps;pt,rt=ik.tcp_pose(probe,station.BASES['right'])
            jac[:,j]=(features(pt+rt@offset,rt@relative)-measured)/eps
        unconstrained_delta=.20*(jac.T@np.linalg.solve(jac@jac.T+4.e-6*np.eye(6),error))
        # Preserve the coupled Jacobian direction. Clipping each joint
        # independently destroys elbow/wrist cancellation near a singularity.
        delta=unconstrained_delta*min(1.,.004/max(float(np.max(np.abs(unconstrained_delta))),1.e-12))
        if self.visual_q is None:self.visual_q=self.last_command_q['right'].copy()
        self.visual_q=np.clip(self.visual_q+delta,np.maximum(ik.LOWER,q-.10),np.minimum(ik.UPPER,q+.10))
        goal,rot=ik.tcp_pose(self.visual_q,station.BASES['right'])
        diagnostic={'mode':'measured_shaft_feature_servo','goal_tcp':goal.tolist(),'measured_tcp':tcp.tolist(),
            'goal_rotation':rot.tolist(),'measured_rotation':hand_r.tolist(),'feature_target':target.tolist(),
            'feature_measured':measured.tolist(),'feature_error':error.tolist(),'joint_increment':delta.tolist(),'unconstrained_joint_increment':unconstrained_delta.tolist(),'predicted_feature_change':(jac@delta).tolist(),
            'ik_error_m':None,'ik_error_rad':None}
        return self.visual_q.copy(),goal,rot,diagnostic

    def enter(self,t,p):
        self.phase+=1;self.start=t
        if self.phase>=len(PHASES):self.phase=len(PHASES)-1
        name=PHASES[self.phase][0]
        self.reference={s:(*ik.tcp_pose(self.joints(s),station.BASES[s]),self.hold[s][2]) for s in ('left','right')}
        if name=='right_hover':
            pick,tool=self.valve_grasp_goal(p)
            self.fixed['valve_pick']=(pick,tool)
            self.fixed['valve_hover']=pick-tool[:,2]*.035+np.array([0.,0.,.035])
            self.fixed['valve_center']=pick.copy()
        if name=='right_lift':
            robot=self.env.scene['right_robot'];a=robot.data.joint_pos;a=a.torch if hasattr(a,'torch') else a;a=a[0].detach().cpu().numpy()
            opening=-float(np.mean([a[robot.joint_names.index('joint'+str(i))] for i in (7,8)]))
            if opening<.0142:raise RuntimeError('Native valve tab pinch not established')
            self.fixed['valve_center']=self.hold['right'][0].copy()
            self.fixed['valve_pick']=(self.hold['right'][0].copy(),self.hold['right'][1].copy())
            self.latch_grasp('right','valve',p)
            self.fixed['valve_lift_anchor']=point(p['catheter'],[0.,-.087,0.])
            start_r=rotation(p['valve'][3:]);axis=start_r[:,0];yaw=np.arctan2(axis[1],axis[0]);pitch=np.deg2rad(20.)
            wanted=np.array([np.cos(pitch)*np.cos(yaw),np.cos(pitch)*np.sin(yaw),-np.sin(pitch)])
            cross=np.cross(axis,wanted);cosine=float(np.dot(axis,wanted))
            k=np.array([[0,-cross[2],cross[1]],[cross[2],0,-cross[0]],[-cross[1],cross[0],0]])
            self.fixed['valve_lift_start_r']=start_r
            self.fixed['valve_lift_end_r']=(np.eye(3)+k+k@k/(1+cosine))@start_r

        if name=='right_orient':
            if p['valve'][2]<self.initial['valve'][2]+.008:raise RuntimeError('Native valve grasp/lift not established')
            self.latch_grasp('right','valve',p)
            hub=point(self.initial['catheter'],np.array([0.,-.087,0.]))
            start_hub=point(p['catheter'],np.array([0.,-.087,0.]))
            # A slightly downward-pointing nozzle keeps the empty receiver's
            # free piston against its forward stop under gravity during carry.
            # Pressure return after connection still has to raise the piston.
            pitch=np.deg2rad(5.);start_rotation=rotation(p['valve'][3:])
            axis=start_rotation[:,0];yaw=np.arctan2(axis[1],axis[0])
            wanted=np.array([np.cos(pitch)*np.cos(yaw),np.cos(pitch)*np.sin(yaw),-np.sin(pitch)])
            cross=np.cross(axis,wanted);cosine=float(np.dot(axis,wanted))
            k=np.array([[0,-cross[2],cross[1]],[cross[2],0,-cross[0]],[-cross[1],cross[0],0]])
            # Keep the socket's unconstrained roll, avoiding an unreachable
            # wrist rotation that serves no connector-alignment purpose.
            desired=(np.eye(3)+k+k@k/(1+cosine))@start_rotation
            self.fixed['valve_anchor']=hub.copy()
            self.fixed['valve_start_anchor']=start_hub.copy()
            self.fixed['valve_start_rotation']=rotation(p['valve'][3:])
            self.fixed['valve']=(hub-desired@np.array([.039,.007,0.]),desired)
        if name=='preconnect' and p['barrel'][2]<.850:
            raise RuntimeError('Native receiver grasp/lift not established')
        if name in ('preconnect','insert','drain','pull_off','receiver_hover','receiver_lower','receiver_release'):
            # A new phase starts from the currently measured grasp. This keeps
            # its interpolation origin continuous after native contact slip.
            self.latch_grasp('left','barrel',p)
        if name=='left_lift':
            self.latch_grasp('left','barrel',p)
            self.fixed['barrel_pick']=p['barrel'][:3].copy()
        if name=='receiver_retreat':self.fixed['left_retreat_q']=self.joints('left').copy()
        if name=='valve_release':
            self.latch_grasp('right','valve',p)
        if name=='hub_hover':
            self.bias['right']=np.zeros(3);self.rotation_bias['right']=np.zeros(3)
            from grasp_planner import choose_hub_grasp
            pick,tool,plan=choose_hub_grasp(p,self.joints('right'))
            self.fixed['hub_tool']=tool;self.fixed['hub_pick']=pick
            Path('/outputs/hub-grasp-plan.json').write_text(json.dumps(plan,indent=2))
        if name=='align_shaft':
            robot=self.env.scene['right_robot'];q=robot.data.joint_pos
            q=q.torch if hasattr(q,'torch') else q;q=q[0].detach().cpu().numpy()
            opening=-float(np.mean([q[robot.joint_names.index('joint'+str(i))] for i in (7,8)]))
            if opening<.0126:raise RuntimeError('Native hub pinch contact not established')
            self.latch_grasp('right','catheter',p)
            self.fixed['shaft_rotation']=align_y(rotation(p['catheter'][3:]))
            self.fixed['shaft_start_y']=float(p['catheter'][1])
            self.visual_q=self.last_command_q['right'].copy()
        if name=='withdraw':
            self.fixed['withdraw_y']=float(p['catheter'][1]);self.shaft_position_bias[1]=0.
        if name=='shaft_lift':
            self.hub_joint_bias=np.clip(self.last_command_q['right']-self.joints('right'),-.15,.15)
            self.q['right']=self.joints('right');self.latch_grasp('right','catheter',p)
            self.fixed['shaft_rotation']=align_y(rotation(p['catheter'][3:]))
        if name in ('shaft_carry','shaft_lower'):
            self.latch_grasp('right','catheter',p)
            self.fixed['placement_start']=p['catheter'][:3].copy()
            self.fixed['placement_start_rotation']=rotation(p['catheter'][3:])
            if name=='shaft_carry':self.body_servo_q['right']=np.zeros(6)
            else:
                self.fixed['placement_path']=[p['catheter'][:3].copy(),np.array(PLACEMENT_APPROACH),np.array(PLACEMENT_LOWER_START),np.array(PLACEMENT_ROOT_TARGET)]
                self.fixed['placement_segment']=0
                self.fixed['placement_fraction']=0.
                self.fixed['placement_complete']=False
        if name=='shaft_release':self.fixed['release_hold_q']=self.last_command_q['right'].copy()
        if name=='final_retreat':self.fixed['right_retreat_q']=self.joints('right').copy()
        with self.log.open('a') as f:f.write(json.dumps({'event':'phase','time':t,'phase':name,'poses':{k:v.tolist() for k,v in p.items()}})+'\n')

    def goals(self,name,p):
        g={s:tuple(x.copy() if isinstance(x,np.ndarray) else x for x in self.hold[s]) for s in ('left','right')}
        barrel_r=rotation(p['barrel'][3:]);theta=np.deg2rad(30.)
        receiver_approach=np.sin(theta)*barrel_r[:,0]-np.cos(theta)*barrel_r[:,2]
        receiver_closing=-barrel_r[:,1]
        receiver_tool=np.column_stack([receiver_closing,np.cross(receiver_approach,receiver_closing),receiver_approach])
        bp=point(p['barrel'],np.array([-.010,0.,0.]))-receiver_approach*PAD_TIP_PAST_TCP
        down=grip_rotation();right_start=self.reference['right'][0]
        pick,tool=self.fixed['valve_pick']
        # The tested native grasp approaches from the branch's -Y side,
        # pinches the exposed upper/front strip of the tab, and recentres
        # while closing. Finite preload avoids ejecting this 25g part.
        if name=='right_hover':g['right']=(self.fixed['valve_hover'],tool,self.valve_open)
        elif name in ('right_lower','right_close'):
            pick,tool=self.valve_grasp_goal(p,name=='right_close')
            g['right']=(pick,tool,self.valve_grip if name=='right_close' else self.valve_open)
        elif name=='right_lift':
            r=self.fixed['valve_lift_end_r'];v=self.fixed['valve_lift_anchor']-r@np.array([.039,.007,0.])
            g['right']=self.body_goal('right','valve',v,r,self.valve_grip)
        if self.phase>=4 and self.phase<=16:
            v,r=self.fixed['valve']
            if self.connection_time is not None:
                # After the empty receiver is attached, raise its nozzle axis
                # gradually. This is an ordinary two-hand hold adjustment;
                # gravity must no longer oppose the free piston return.
                axis=r[:,0];yaw=np.arctan2(axis[1],axis[0]);pitch=np.deg2rad(15.)
                wanted=np.array([np.cos(pitch)*np.cos(yaw),np.cos(pitch)*np.sin(yaw),np.sin(pitch)])
                cross=np.cross(axis,wanted);cosine=float(np.dot(axis,wanted))
                k=np.array([[0,-cross[2],cross[1]],[cross[2],0,-cross[0]],[-cross[1],cross[0],0]])
                tilt=(np.eye(3)+k+k@k/(1+cosine))@r
                r=ik.slerp_matrix(r,tilt,ik.smoothstep(min((self.last-self.connection_time)/4.,1.)))
                v=self.fixed['valve_anchor']-r@np.array([.039,.007,0.])
            g['right']=self.body_goal('right','valve',v,r,self.valve_grip)
        if name=='left_hover':g['left']=(bp+[0,0,.09],receiver_tool,1.)
        elif name=='left_lower':g['left']=(bp,receiver_tool,1.)
        elif name=='left_close':g['left']=(self.reference['left'][0],self.reference['left'][1],self.receiver_grip)
        elif name=='left_lift':
            g['left']=self.body_goal('left','barrel',self.fixed['barrel_pick']+[0,0,.08],np.eye(3))
        if name in ('preconnect','insert','drain','pull_off'):
            r=rotation(p['valve'][3:]);mouth=point(p['valve'],np.array([-.014,0.,0.]))
            delta={'preconnect':-.020,'insert':.004,'drain':.004,'pull_off':-.018}[name]
            goal=mouth+r@np.array([delta-.062,0.,0.])
            # Match the nozzle axis while leaving the round barrel upright;
            # copying the valve's roll needlessly violates the left wrist limit.
            x=r[:,0]
            if name=='preconnect':
                # Empty the free piston by gravity before horizontal insertion.
                # This is an ordinary orientation of the held receiver.
                yaw=np.arctan2(x[1],x[0]);pitch=np.deg2rad(5.)
                x=np.array([np.cos(pitch)*np.cos(yaw),np.cos(pitch)*np.sin(yaw),-np.sin(pitch)])
            y=np.cross([0.,0.,1.],x);y/=np.linalg.norm(y)
            barrel_rotation=np.column_stack([x,y,np.cross(x,y)])
            roll=np.deg2rad(-25.)
            barrel_rotation=barrel_rotation@np.array([[1.,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
            g['left']=self.body_goal('left','barrel',goal,barrel_rotation)
        if name in ('receiver_hover','receiver_lower','receiver_release','receiver_retreat'):
            z={'receiver_hover':.960,'receiver_lower':.821,'receiver_release':.821,'receiver_retreat':.950}[name]
            grip=1. if name in ('receiver_release','receiver_retreat') else 0.
            g['left']=self.body_goal('left','barrel',np.array([.300,.300,z]),np.eye(3),grip)
        if name=='valve_release':
            g['right']=self.body_goal('right','valve',self.initial['valve'][:3],rotation(self.initial['valve'][3:]),1.)
        elif name=='right_clear':g['right']=(right_start+[0,0,.08],down,1.)
        hub=self.fixed.get('hub_pick',point(p['catheter'],np.array([0.,-.087,.010]))+np.array([0.,0.,PAD_TIP_PAST_TCP]))
        if name=='hub_hover':g['right']=(hub+[0,0,.08],self.fixed['hub_tool'],1.)
        elif name=='hub_lower':g['right']=(hub,self.fixed['hub_tool'],1.)
        elif name=='hub_close':g['right']=(hub,self.fixed['hub_tool'],self.hub_grip)
        if self.phase>=22:
            r=self.fixed['shaft_rotation']
            if name=='align_shaft':pos=[.375,self.fixed['shaft_start_y'],.851]
            elif name=='withdraw':pos=[.375,self.fixed['withdraw_y'],.851]
            elif name=='shaft_lift':pos=[.375,-.180,.930]
            elif name=='shaft_carry':pos=list(PLACEMENT_CARRY)
            elif name in ('shaft_lower','shaft_release'):pos=list(PLACEMENT_ROOT_TARGET)
            else:pos=[.250,-.330,.980]
            grip=1. if name in ('shaft_release','final_retreat','finish') else self.hub_grip
            if name in ('align_shaft','withdraw'):
                angle=float(np.linalg.norm(self.shaft_rotation_bias));correction=np.eye(3)
                if angle>1e-9:
                    x,y,z=self.shaft_rotation_bias/angle;k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                    correction+=np.sin(angle)*k+(1-np.cos(angle))*(k@k)
                r=correction@r;pos=np.array(pos)+self.shaft_position_bias
            g['right']=self.body_goal('right','catheter',np.array(pos),r,grip)
        return g

    def action(self,t):
        p=self.runtime.poses();m=self.runtime.last_metrics
        if m.get('connected') and self.connection_time is None:self.connection_time=t
        if m.get('failed'):raise RuntimeError('Mechanism invalidated: '+str(m.get('failure')))
        if self.phase<0:self.enter(t,p)
        elapsed=t-self.start;name,duration=PHASES[self.phase]
        if name in ('align_shaft','withdraw','shaft_lift','shaft_carry','shaft_lower'):
            robot=self.env.scene['right_robot'];q=robot.data.joint_pos;q=q.torch if hasattr(q,'torch') else q;q=q[0].detach().cpu().numpy()
            opening=-float(np.mean([q[robot.joint_names.index('joint'+str(i))] for i in (7,8)]))
            if opening<.0126:raise RuntimeError('Native hub grasp lost during extraction/placement')
        if name=='withdraw':
            balloon=point(p['catheter'],[0,.132,0]);axis=rotation(p['catheter'][3:])[:,1]
            radial=float(np.linalg.norm((balloon-[.375,0,.851])[[0,2]]))
            target_y=self.fixed['withdraw_y']
            # Advance small physical axial waypoints only after the last one
            # settles and the measured balloon remains centred in the bore.
            if abs(p['catheter'][1]-target_y)<.001 and radial<.0012 and np.linalg.norm(axis[[0,2]]-SHAFT_AXIS_XZ)<.015 and abs(rotation(p['catheter'][3:])[2,0])<.04:
                self.fixed['withdraw_y']=max(-.180,self.fixed['withdraw_y']-.0015)
        if elapsed>=duration and self.phase<len(PHASES)-1:
            if name=='right_lower' and (np.linalg.norm(ik.tcp_pose(self.joints('right'),station.BASES['right'])[0]-self.valve_grasp_goal(p)[0])>.0025 or np.linalg.norm(ik._rot_error(self.valve_grasp_goal(p)[1],ik.tcp_pose(self.joints('right'),station.BASES['right'])[1]))>.04):
                if elapsed>duration+3:raise RuntimeError('Native valve approach did not settle')
            elif name=='drain' and not m.get('collapsed'):
                if elapsed>duration+10:raise RuntimeError('Receiver did not connect/deflate within dwell')
            elif name=='hub_lower' and (np.linalg.norm(ik.tcp_pose(self.joints('right'),station.BASES['right'])[0]-self.fixed['hub_pick'])>.0015 or np.linalg.norm(ik._rot_error(self.fixed['hub_tool'],ik.tcp_pose(self.joints('right'),station.BASES['right'])[1]))>.025):
                if elapsed>duration+3:raise RuntimeError('Hub approach did not settle before closing')
            elif name=='align_shaft' and (np.linalg.norm((point(p['catheter'],[0,.132,0])-[.375,0,.851])[[0,2]])>.0015 or np.linalg.norm(rotation(p['catheter'][3:])[[0,2],1]-SHAFT_AXIS_XZ)>.015 or abs(rotation(p['catheter'][3:])[2,0])>.04):
                if elapsed>duration+16:raise RuntimeError('Native shaft alignment did not settle before withdrawal')
            elif name=='shaft_lower' and (np.linalg.norm(p['catheter'][:3]-PLACEMENT_ROOT_TARGET)>.003 or
                    not self.fixed['placement_complete'] or
                    abs(rotation(p['catheter'][3:])[2,1])>np.sin(np.deg2rad(2.))):
                if elapsed>duration+12:raise RuntimeError('Native final placement did not settle before release')
            elif name=='withdraw' and point(p['catheter'],[0,.168,0])[1]>.006:
                if elapsed>duration+15:raise RuntimeError('Native centred withdrawal did not complete within dwell')
            else:self.enter(t,p);elapsed=0.;name,duration=PHASES[self.phase]
        goals=self.goals(name,p);result=[];diagnostic={}
        for side in ('left','right'):
            if side=='right' and name in ('align_shaft','withdraw'):
                desired_y=self.fixed['shaft_start_y'] if name=='align_shaft' else self.fixed['withdraw_y']
                q,goal,rot,diagnostic[side]=self.shaft_servo(p,desired_y)
                result.extend([*q,self.hub_grip]);self.q[side]=q;self.last_command_q[side]=q.copy()
                self.hold[side]=(goal.copy(),rot.copy(),self.hub_grip)
                continue
            if side=='left' and self.phase>=16:
                f=ik.smoothstep(min(elapsed/2.,1.)) if name=='receiver_retreat' else 1.
                q=self.fixed['left_retreat_q']+f*(np.asarray(READY)-self.fixed['left_retreat_q'])
                actual,actual_rotation=ik.tcp_pose(self.joints(side),station.BASES[side]);goal,rot=ik.tcp_pose(q,station.BASES[side])
                result.extend([*q,1.]);self.q[side]=q;self.hold[side]=(goal.copy(),rot.copy(),1.)
                self.last_command_q[side]=q.copy()
                diagnostic[side]={'mode':'joint_space_retreat_to_ready','goal_tcp':goal.tolist(),'measured_tcp':actual.tolist(),'goal_rotation':rot.tolist(),'measured_rotation':actual_rotation.tolist(),'ik_error_m':None,'ik_error_rad':None}
                continue
            if side=='right' and name in ('shaft_release','final_retreat','finish'):
                if name=='shaft_release':
                    q=self.fixed['release_hold_q'].copy()
                    opening=self.hub_grip+min(1.,elapsed)*(1.-self.hub_grip)
                else:
                    f=ik.smoothstep(min(elapsed/3.,1.)) if name=='final_retreat' else 1.
                    start=self.fixed['right_retreat_q'];q=start+f*(np.array(READY)-start);opening=1.
                actual,actual_rotation=ik.tcp_pose(self.joints(side),station.BASES[side]);goal,rot=ik.tcp_pose(q,station.BASES[side])
                result.extend([*q,opening]);self.q[side]=q;self.last_command_q[side]=q.copy();self.hold[side]=(goal.copy(),rot.copy(),opening)
                diagnostic[side]={'mode':'hold_then_open' if name=='shaft_release' else 'joint_space_retreat_to_ready','goal_tcp':goal.tolist(),'measured_tcp':actual.tolist(),'goal_rotation':rot.tolist(),'measured_rotation':actual_rotation.tolist(),'ik_error_m':None,'ik_error_rad':None}
                continue
            start_pos,start_rot,start_grip=self.reference[side];goal,rot,grip=goals[side]
            f=ik.smoothstep(min(elapsed/duration,1.));target=start_pos+f*(goal-start_pos)
            if name=='right_lower' and side=='right':f=ik.smoothstep(min(elapsed/2.,1.));target=start_pos+f*(goal-start_pos)
            if name=='hub_lower' and side=='right':f=ik.smoothstep(min(elapsed/3.,1.));target=start_pos+f*(goal-start_pos)
            if name=='withdraw' and side=='right':f=1.;target=goal.copy()
            target_rot=ik.slerp_matrix(start_rot,rot,f);command_grip=start_grip+f*(grip-start_grip)
            if name in ('shaft_carry','shaft_lower') and side=='right':
                start_body=self.fixed['placement_start'];start_body_r=self.fixed['placement_start_rotation']
                desired_r=self.fixed['shaft_rotation']
                destination=np.array(PLACEMENT_ROOT_TARGET)
                if name=='shaft_carry':
                    destination=np.array(PLACEMENT_CARRY)
                    body_p=start_body+f*(destination-start_body)
                    body_r=ik.slerp_matrix(start_body_r,desired_r,f)
                else:
                    # Advance three measured segments while the freely hinged
                    # valve clears the tray. Only robot targets are written.
                    dt=float(np.clip(t-self.last,0.,.05))
                    path=self.fixed['placement_path'];segment=self.fixed['placement_segment']
                    fraction=self.fixed['placement_fraction']
                    origin=path[segment];destination=path[segment+1]
                    previous=origin+fraction*(destination-origin)
                    tracking=float(np.linalg.norm(p['catheter'][:3]-previous))
                    axis_error=float(np.linalg.norm(rotation(p['catheter'][3:])[:,1]-desired_r[:,1]))
                    if tracking<=.003 and axis_error<=.025:
                        fraction=min(1.,fraction+.10*dt)
                    body_p=origin+fraction*(destination-origin);body_r=desired_r
                    at_endpoint=bool(fraction>=1. and np.linalg.norm(p['catheter'][:3]-destination)<=.003 and abs(rotation(p['catheter'][3:])[2,1])<=np.sin(np.deg2rad(2.)))
                    if at_endpoint:
                        if segment<len(path)-2:segment+=1;fraction=0.
                        else:self.fixed['placement_complete']=True
                    self.fixed['placement_segment']=segment
                    self.fixed['placement_fraction']=fraction
                target,target_rot,command_grip=self.body_goal(side,'catheter',body_p,body_r,self.hub_grip)
                q,goal_tcp,goal_rot,diagnostic[side]=self.body_servo(side,'catheter',p,target,target_rot,self.hub_grip)
                result.extend([*q,self.hub_grip]);self.q[side]=q;self.last_command_q[side]=q.copy()
                self.hold[side]=(goal_tcp.copy(),goal_rot.copy(),self.hub_grip)
                continue
            if name=='preconnect' and side=='left':
                # Lift before crossing the hollow chamber. The complete
                # collision wall is higher than its cutaway visualization.
                raised_start=start_pos+np.array([0.,0.,.060])
                high_goal=goal+np.array([0.,0.,.100])
                if elapsed<2.:
                    f=ik.smoothstep(elapsed/2.);target=start_pos+f*(raised_start-start_pos);target_rot=start_rot
                elif elapsed<6.:
                    f=ik.smoothstep((elapsed-2.)/4.)
                    target=raised_start+f*(high_goal-raised_start);target_rot=ik.slerp_matrix(start_rot,rot,f)
                else:
                    f=ik.smoothstep(min((elapsed-6.)/4.,1.))
                    target=goal+np.array([0.,0.,.100])*(1-f);target_rot=rot

            if name=='receiver_hover' and side=='left':
                # Clear the chamber before returning across it, then lower
                # vertically at the separate left staging pad in the next phase.
                raised_start=start_pos+np.array([0.,0.,.120])
                if elapsed<2.:
                    f=ik.smoothstep(elapsed/2.);target=start_pos+f*(raised_start-start_pos);target_rot=start_rot
                else:
                    f=ik.smoothstep(min((elapsed-2.)/4.,1.));target=raised_start+f*(goal-raised_start);target_rot=ik.slerp_matrix(start_rot,rot,f)
            if name=='valve_release' and side=='right':
                f=ik.smoothstep(min(elapsed/4.,1.));target=start_pos+f*(goal-start_pos);target_rot=ik.slerp_matrix(start_rot,rot,f)
                command_grip=self.valve_grip if elapsed<4. else self.valve_grip+ik.smoothstep(min(elapsed-4.,1.))*(1.-self.valve_grip)
                if elapsed<4.:
                    q,goal_tcp,goal_rot,diagnostic[side]=self.body_servo(side,'valve',p,target,target_rot,self.valve_grip)
                else:
                    q=self.last_command_q[side].copy();goal_tcp,goal_rot=ik.tcp_pose(q,station.BASES[side]);actual,actual_r=ik.tcp_pose(self.joints(side),station.BASES[side])
                    diagnostic[side]={'mode':'hold_returned_valve_then_release','goal_tcp':goal_tcp.tolist(),'measured_tcp':actual.tolist(),'goal_rotation':goal_rot.tolist(),'measured_rotation':actual_r.tolist(),'ik_error_m':None,'ik_error_rad':None}
                result.extend([*q,command_grip]);self.q[side]=q;self.last_command_q[side]=q.copy();self.hold[side]=(goal_tcp.copy(),goal_rot.copy(),1.)
                continue
            if name=='right_lift' and side=='right':
                # Lift the valve around the real free branch joint rather than
                # lifting its anchor and dragging the retained catheter upward.
                body_rotation=ik.slerp_matrix(self.fixed['valve_lift_start_r'],self.fixed['valve_lift_end_r'],f)
                body_position=self.fixed['valve_lift_anchor']-body_rotation@np.array([.039,.007,0.])
                target,target_rot,command_grip=self.body_goal('right','valve',body_position,body_rotation,self.valve_grip)
            if name=='right_orient' and side=='right':
                # Rotate the physical valve around its actual branch anchor.
                # Interpolating only TCP endpoints sweeps that anchor through
                # the phantom and tugs the retained catheter sideways.
                body_rotation=ik.slerp_matrix(self.fixed['valve_start_rotation'],self.fixed['valve'][1],f)
                anchor=self.fixed['valve_start_anchor']+f*(self.fixed['valve_anchor']-self.fixed['valve_start_anchor'])
                body_position=anchor-body_rotation@np.array([.039,.007,0.])
                target,target_rot,command_grip=self.body_goal('right','valve',body_position,body_rotation,self.valve_grip)
            if side=='left' and name in ('left_lift','preconnect','insert','drain','pull_off'):
                q,goal_tcp,goal_rot,diagnostic[side]=self.body_servo(side,'barrel',p,target,target_rot,command_grip)
                result.extend([*q,float(np.clip(command_grip,0,1))]);self.q[side]=q;self.last_command_q[side]=q.copy()
                self.hold[side]=(goal.copy(),rot.copy(),grip)
                continue
            actual,actual_rotation=ik.tcp_pose(self.joints(side),station.BASES[side])
            joint_tracking=side=='right' and self.phase>=19
            if (elapsed>duration*.85 or (side=='right' and name=='right_lower' and elapsed>1.5)) and self.ik_feasible[side] and not joint_tracking:
                self.bias[side]=np.clip(self.bias[side]+np.clip(.025*(target-actual),-.0002,.0002),-.012,.012)
                angular_error=.5*sum((np.cross(actual_rotation[:,i],target_rot[:,i]) for i in range(3)),np.zeros(3))
                self.rotation_bias[side]=np.clip(self.rotation_bias[side]+np.clip(.015*angular_error,-.0004,.0004),-.12,.12)
            b=self.rotation_bias[side];angle=float(np.linalg.norm(b));correction=np.eye(3)
            if angle>1e-9:
                x,y,z=b/angle;k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                correction+=np.sin(angle)*k+(1-np.cos(angle))*(k@k)
            q,pe,re=ik.solve(target+self.bias[side],ik.matrix_to_quat_xyzw(correction@target_rot),self.q[side],base_pos=station.BASES[side],iters=60)
            if side=='right' and self.phase<=3 and (pe>.005 or re>.10):raise RuntimeError('Native valve waypoint left the validated IK range')
            self.ik_feasible[side]=bool(pe<.002 and re<.035)
            command_q=q
            if joint_tracking:
                # Compensate measured steady actuator error through ordinary
                # bounded joint-position commands, retaining native torque limits.
                if ((name=='hub_lower' and elapsed>3.5) or (name=='align_shaft' and elapsed>=duration)) and self.ik_feasible[side]:
                    self.hub_joint_bias=np.clip(self.hub_joint_bias+np.clip(.04*(q-self.joints(side)),-.001,.001),-.15,.15)
                command_q=np.clip(q+self.hub_joint_bias,ik.LOWER,ik.UPPER)
            self.q[side]=q;result.extend([*command_q,float(np.clip(command_grip,0,1))]);self.hold[side]=(goal.copy(),rot.copy(),grip)
            self.last_command_q[side]=command_q.copy()
            diagnostic[side]={'goal_tcp':target.tolist(),'measured_tcp':actual.tolist(),'goal_rotation':target_rot.tolist(),'measured_rotation':actual_rotation.tolist(),'rotation_bias_rad':self.rotation_bias[side].tolist(),'ik_error_m':pe,'ik_error_rad':re}
        if int(t*20)%10==0:
            with self.log.open('a') as f:f.write(json.dumps({'time':t,'phase':name,'waypoints':diagnostic,'mechanics':m})+'\n')
        self.last=t;return np.asarray(result)
