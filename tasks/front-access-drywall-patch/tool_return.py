"""Known-layout tool return after the repair. All motions use standard YAM joint targets."""
import numpy as np
from scipy.spatial.transform import Rotation as R
from yam_arena import ik, station
from yam_arena.constants import READY

PREFIX_ACTIONS = 6000
RETURN_ACTIONS = 1200
# A clear part of the table in front of the repair. The empty magazine points away from the arms.
PRESENT = np.array([-.075, .051, station.TABLE_TOP+.1625])
MOTOR_GRIP = np.array([.095, -.043, .001])
HANDOFF_ATT = R.from_euler('x',-25,degrees=True)
DOWN = R.from_euler('z',-12,degrees=True)*R.from_matrix([[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]])


def _move(q, side, delta, turn, limit=.002):
    p,m = ik.tcp_pose(q, station.BASES[side])
    pos = p+np.clip(delta, -limit, limit)
    rot = R.from_rotvec(np.clip(turn,-.012,.012))*R.from_matrix(m)
    new,d,a = ik.solve(pos,rot.as_quat(),q_init=q,base_pos=station.BASES[side],iters=25,tol=.00008)
    if d>.002 or a>.03:
        return q
    return np.clip(q+np.clip(new-q,-.045,.045),ik.LOWER, ik.UPPER)


class ReturnDriver:
    def __init__(self,last_action):
        self.action=last_action.copy()
        self.phase='Align lifting handle above rack'
        self.age=0
        self.confirm=0

    def advance(self,phase):
        self.phase=phase
        self.age=0
        self.confirm=0

    def step(self,state,body,loads):
        left,right=self.action[:7],self.action[7:]
        rp,rm=ik.tcp_pose(state[7:13],station.BASES['right'])
        lp,lm=ik.tcp_pose(state[:6],station.BASES['left'])
        shell=body['driver']
        rotation=R.from_quat(shell[3:])
        self.age+=1
        phase=self.phase
        def receiving_grip():
            # One pad may land on the attached magazine while the opposite pad
            # loads the motor casing. Use measured jaw closure and casing load
            # as a second feedback cue before opening the trigger-hand fingers.
            stalled=(right[6]<.10 and .18<state[13]<.80 and state[13]-right[6]>.15)
            return loads['right_pads'] or (stalled and abs(loads['right_lift_N'])>.25)
        def track(hand,side,at,att,goal,desired,speed=.002):
            hand[:6]=_move(hand[:6],side,.22*(goal-at),.15*(desired*att.inv()).as_rotvec(),speed)
        if phase=='Align lifting handle above rack' or phase=='Seat lifting handle behind posts':
            # The two original front posts retain the crossbar against the small
            # extraction force when the fingers back out of its capped sockets.
            c=body['carrier']
            h=.960 if phase.startswith('Align') else 0.852
            target=np.array([0.324,-0.327,h])
            rack_att=R.from_euler("y",0,degrees=True)
            track(right,'right',c[:3],R.from_quat(c[3:]),target,rack_att,.0016)
            distance=np.linalg.norm(c[:3]-target)
            if phase.startswith('Align') and distance<.005 and (rack_att.inv()*R.from_quat(c[3:])).magnitude()<.06:
                self.advance('Seat lifting handle behind posts')
            elif phase.startswith('Seat') and (distance<.004 or (self.age>95 and c[2]<.866)):
                self.advance('Release lifting handle')
        elif phase=='Release lifting handle':
            right[6]=.52
            if self.age==20:
                grip_rot=R.from_quat(body['carrier'][3:])*R.from_euler('y',28,degrees=True)
                self.retreat=rp-grip_rot.apply([.104,0.,0.])
                self.retreat[2]=min(self.retreat[2],rp[2]+.004)
                self.carrier_att=R.from_matrix(rm)
                self.advance('Clear lifting handle')
        elif phase=='Clear lifting handle':
            if self.age>=98 and loads['right_on_handle'] and not hasattr(self,'bench_goal'):
                self.bench_goal=rp.copy()
                self.bench_goal[2]=station.TABLE_TOP+.040
                self.bench_att=R.from_matrix(rm)
                self.bench_open=0
            if hasattr(self,'bench_goal'):
                if loads['carrier_table']>.22 or self.bench_open:
                    self.bench_open+=1
                    right[6]=min(.97,.52+.026*self.bench_open)
                    if self.bench_open==23:
                        self.bench_exit=rp-self.bench_att.apply([0.,0.,.09])
                    if self.bench_open>=23:
                        track(right,'right',rp,R.from_matrix(rm),self.bench_exit,self.bench_att,.0018)
                else:
                    track(right,'right',rp,R.from_matrix(rm),self.bench_goal,self.bench_att,.00135)
            else:
                track(right,'right',rp,R.from_matrix(rm),self.retreat,self.carrier_att,.0014)
            self.confirm=self.confirm+1 if not loads['right_on_handle'] else 0
            if self.age>=90 and self.confirm>=5:
                right[6]=.92
                self.presentation_att=HANDOFF_ATT
                self.advance('Present driver over table')
        elif phase=='Present driver over table':
            # Reaching the same casing with a loaded grip can shift the handle
            # in the left pads. Solve the transition from the actual TCP/tool
            # transform before using Cartesian feedback at the handoff.
            if self.age==1:
                from reach import solve
                local_tcp=rotation.inv().apply(lp-shell[:3])
                grip_att=rotation.inv()*R.from_matrix(lm)
                desired_position=PRESENT+self.presentation_att.apply(local_tcp)
                desired_att=self.presentation_att*grip_att
                q,err,angle=solve(desired_position,desired_att,[left[:6],[-1.55,1.52,1.48,-1.5,0.,0.]],'left')
                if err>.004 or angle>.055:raise ValueError('Unreachable driver presentation in held grasp')
                self.right_start=right[:6].copy()
                self.left_start=left[:6].copy()
                self.left_target=q
            # The emptied rack is far to the side of the driver. Fold the
            # receiving arm back into its unloaded ready pose while the other
            # hand presents the driver. From here it can approach from above.
            right_u=ik.smoothstep(min(1.,self.age/75))
            right[:6]=self.right_start+right_u*(np.asarray(READY)-self.right_start)
            right[6]=.92
            if self.age<=85:
                u=ik.smoothstep(self.age/85)
                left[:6]=self.left_start+u*(self.left_target-self.left_start)
            else:
                track(left,'left',shell[:3],rotation,PRESENT,self.presentation_att,.00125)
                position_error=np.linalg.norm(shell[:3]-PRESENT)
                angle_error=(self.presentation_att.inv()*rotation).magnitude()
                near=position_error<.004 and angle_error<.07
                if near:
                    self.handoff_goal=PRESENT.copy()
                elif self.age>=145 and position_error<.022 and angle_error<.17 and not loads['table'] and loads['switch_mm']>=2.:
                    # A loaded wrist may settle just short of its nominal pose.
                    # Accept a reachable casing pose while the trigger hand is
                    # still supporting the tool. The receiver targets the casing.
                    from reach import solve
                    goal=shell[:3]+rotation.apply(MOTOR_GRIP)+[0,0,.028]
                    _,err,angle=solve(goal,DOWN,[[1.,1.4,1.,-.95,0.,1.]],'right')
                    if err<.0015 and angle<.018:
                        self.handoff_goal=shell[:3].copy()
                        self.presentation_att=rotation
                if hasattr(self,'handoff_goal'):
                    self.advance('Reach above motor casing')
        elif phase in ('Reach above motor casing','Descend over motor casing','Close on motor casing'):
            if phase!='Close on motor casing':
                track(left,'left',shell[:3],rotation,self.handoff_goal,self.presentation_att,.001)
            center=shell[:3]+rotation.apply(MOTOR_GRIP)
            raised=center+[0,0,.028] if phase=='Reach above motor casing' else center
            if phase=='Reach above motor casing':
                if self.age==1:
                    target,dist,angle=ik.solve(raised,DOWN.as_quat(),q_init=np.array([1.,1.4,1.,-.95,0,1.]),base_pos=station.BASES['right'],iters=180,tol=.00012)
                    if dist>.002 or angle>.025:
                        from reach import solve
                        target,dist,angle=solve(raised,DOWN,[[1.,1.4,1.,-.95,0,1.],[1.,.0,-.8,.5,-.3,1.],state[7:13]],'right')
                    if dist>.002 or angle>.025:raise ValueError('Unreachable overhead grasp')
                    self.pre_q=target;self.pre_start=right[:6].copy()
                right[:6]=self.pre_start+ik.smoothstep(min(1.,self.age/100))*(self.pre_q-self.pre_start)
                if self.age>=105:self.advance('Descend over motor casing')
            else:
                track(right,'right',rp,R.from_matrix(rm),raised,DOWN,.00125)
                if phase=='Descend over motor casing' and np.linalg.norm(rp-raised)<.0025:
                    self.advance('Close on motor casing')
                if phase=='Close on motor casing':
                    right[6]=max(.035,right[6]-.026)
                    self.confirm=self.confirm+1 if receiving_grip() and not loads['table'] else 0
                    if self.confirm>=12:
                        self.unloaded_q=right[:6].copy()
                        # Standard station joint springs need a new equilibrium
                        # when weight moves from the trigger hand to the motor hand.
                        estimated_com=shell[:3]+rotation.apply([.024,-.002,0.])
                        lever=rm.T@(estimated_com-rp)
                        def height(q):
                            point,mat=ik.tcp_pose(q,station.BASES['right'])
                            return (point+mat@lever)[2]
                        eps=.0002
                        grad=np.array([(height(right[:6]+v*eps)-height(right[:6]-v*eps))/(2*eps) for v in np.eye(6)])
                        self.bias=12.36*grad/np.array([80.,80.,80.,10.,10.,10.])
                        self.advance('Share tool weight')
        elif phase=='Share tool weight':
            u=ik.smoothstep(min(1.,self.age/32))
            right[:6]=np.clip(self.unloaded_q+.7*u*self.bias,ik.LOWER,ik.UPPER)
            if self.age>=34 and receiving_grip() and not loads['table']:
                self.advance('Release trigger')
        elif phase=='Release trigger':
            if loads['table']:
                return self.action.copy()
            # Keep the receiving hand load-compensated and vertical. Let the
            # casing settle in the grip without chasing its rear end through space.
            right[:6]=np.clip(self.unloaded_q+self.bias,ik.LOWER,ik.UPPER)
            left[6]=min(.97,left[6]+.015)
            if left[6]>.95 and loads['switch_mm']<2:
                self.left_clear=lp+[0,.038,.076]
                self.left_att=R.from_matrix(lm)
                self.advance('Clear trigger hand')
        elif phase=='Clear trigger hand':
            track(left,'left',lp,R.from_matrix(lm),self.left_clear,self.left_att)
            if self.age>=55:self.advance('Lower driver')
        elif phase=='Lower driver':
            if loads['table_load']<9.:
                right[:6]=_move(right[:6],'right',np.array([0.,0.,-.00055]),np.zeros(3))
            self.confirm=self.confirm+1 if loads['table_load']>=8. else 0
            if self.confirm>=8:self.advance('Settle casing on table')
        elif phase=='Settle casing on table':
            # With the heel supported, ease the nose down towards a broad rest
            # before opening. Keep positive grip throughout the controlled roll.
            axis=rotation.apply([1.,0.,0.])
            tilt=np.arcsin(np.clip(axis[2],-1.,1.))
            normal=np.array([-axis[1],axis[0],0.])
            normal/=max(np.linalg.norm(normal),.001)
            turn=normal*np.clip(.08*tilt,-.008,.008)
            pivot=shell[:3]+rotation.apply([-.019,.060,-.014])
            target=pivot+R.from_rotvec(turn).apply(rp-pivot)
            drop=np.array([0.,0.,-.00035]) if loads['table_load']<11. else np.zeros(3)
            right[:6]=_move(right[:6],'right',target-rp+drop,turn,.0013)
            if (abs(tilt)<.16 and loads['table_load']>9.5) or self.age>=115:
                self.advance('Open receiving hand')
        elif phase=='Open receiving hand':
            right[6]=min(.97,right[6]+.018)
            if self.age>=65:
                self.right_clear=rp+[0.,0.,.095]
                self.advance('Clear parked driver')
        elif phase=='Clear parked driver':
            track(right,'right',rp,R.from_matrix(rm),self.right_clear,DOWN)
            if self.age>=75:self.advance('Observe parked driver')
        return self.action.copy()
