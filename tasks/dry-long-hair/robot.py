"""Measured, phase-gated physical slider actuation and handle lift.

All outputs are ordinary 14D robot targets. No object writes, grasp constraints,
external manipulation forces, software power flags, or timed success claims.
"""
import json
from pathlib import Path
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN
from yam_arena.urdf import quat_xyzw_to_matrix, rotation_angle_deg
from config import DRYER_START, SWITCH_TRAVEL
from switch_geometry import FingerGeometry, SLIDER_HALF, minimum_separation

LEFT_R = np.array([[-1.,0.,0.],[0.,0.,-1.],[0.,-1.,0.]])
RIGHT_JAW = -.028
GRIP = .006/abs(FINGER_OPEN)
RIGHT_GRIP = abs(RIGHT_JAW/FINGER_OPEN)
MAX_PROBE_STEPS = 1600
HANDLE_GRASP_LOCAL = np.array((-.019,-.003,-.090))
HANDLE_APPROACH_OFFSET = np.array((0.,.08,.020))


class DryerProbe:
    def __init__(self, env):
        self.env, self.base = env, env._base
        self.previous = env._ready.detach().cpu().numpy().reshape(-1).copy()
        self.bias = [np.zeros(3), np.zeros(3)]
        self.failure = None
        self.phase = 'approach_handle_brace'
        self.phase_start = 0.
        self.stable_frames = 0
        self.events = []
        self.geometry = FingerGeometry()
        config = json.loads(Path(__file__).with_name('switch_point.json').read_text())
        # This point now defines only lateral insertion alignment; its Z is not
        # used as a support height. Actual clipped collision surfaces do that.
        self.point = np.array(config['point_tcp'])
        self.switchR = np.array(config['rotation'])
        self.q, self.starts = [], []
        for side in ('left','right'):
            joints, fingers, tcp, rotation = self.read_arm(side)
            self.q.append(joints.copy())
            self.starts.append((tcp.copy(), rotation.copy()))
        self.hold = np.array(DRYER_START)+HANDLE_GRASP_LOCAL
        self.hand_clearance = None
        self.braced_dryer = None
        self.right_retreat = None
        self.lift_origin = None
        self.last_t = None
        self.payload_mass=None;self.payload_com_tcp=None;self.payload_bias=np.zeros(6)

    def capture_payload(self,measured,dryer,switch):
        from pxr import UsdPhysics
        def array(value):
            value=value.torch if hasattr(value,'torch') else value
            return value.detach().cpu().numpy() if hasattr(value,'detach') else np.asarray(value)
        weighted=np.zeros(3);total=0.
        for name,pose in (('dryer',dryer),('switch',switch)):
            mass=float(UsdPhysics.MassAPI(self.base.scene.stage.GetPrimAtPath('/World/envs/env_0/'+name)).GetMassAttr().Get())
            com=array(self.base.scene[name].data.body_com_pose_b).reshape(-1,7)[0,:3]
            if not np.isfinite(com).all() or not 0<mass<1:raise RuntimeError('invalid native payload mass/COM')
            weighted+=mass*(pose[:3]+quat_xyzw_to_matrix(pose[3:])@com);total+=mass
        self.payload_mass=total;self.payload_com_tcp=measured[0][3].T@(weighted/total-measured[0][2])
        self.original_payload_grasp=(measured[0][3].T@(dryer[:3]-measured[0][2]),measured[0][3].T@quat_xyzw_to_matrix(dryer[3:]))

    def payload_offset(self,measured_q,blend=1.):
        if getattr(self,'payload_mass',None) is None:return np.zeros(6)
        from payload_gravity import target_offset
        return target_offset(measured_q,self.payload_mass,self.payload_com_tcp,self.env.rig.bases['left'],blend)

    def check_payload_grasp(self,hand,dryer):
        if not hasattr(self,'original_payload_grasp'):return True
        p,R=hand[2:];d=R.T@(dryer[:3]-p);Q=R.T@quat_xyzw_to_matrix(dryer[3:]);old_p,old_R=self.original_payload_grasp
        self.payload_grasp_drift=float(np.linalg.norm(d-old_p));self.payload_grasp_angle=rotation_angle_deg(Q,old_R)
        if self.payload_grasp_drift>.005 or self.payload_grasp_angle>5.:
            self.failure='payload moved outside original pre-lift measured grasp';return False
        return True

    def read_arm(self, side):
        ids, fids = self.env._ids[side+'_robot']
        data = self.base.scene[side+'_robot'].data.joint_pos
        data = data.torch if hasattr(data,'torch') else data
        values = data.detach().cpu().numpy()[0]
        joints, fingers = values[ids], values[fids]
        tcp, rotation = ik.tcp_pose(joints, self.env.rig.bases[side])
        return joints, fingers, tcp, rotation

    def pose(self, name):
        data = self.base.arena_world.get_pose_w(name)
        data = data.torch if hasattr(data,'torch') else data
        return data.detach().cpu().numpy()[0]

    def transition(self, phase, t):
        self.events.append({'time':float(t),'from':self.phase,'to':phase})
        self.phase, self.phase_start, self.stable_frames = phase, t, 0
        # A contact correction accumulated in one phase must never supply the
        # next phase's stroke, especially with only 4 mm of slider travel.
        self.bias[1][:] = 0.

    def gate(self, condition, age, duration, timeout, next_phase, t):
        self.stable_frames = self.stable_frames+1 if condition and age >= duration else 0
        if self.stable_frames >= 5:
            self.transition(next_phase, t)
        elif age > duration+timeout:
            self.failure = self.phase+' did not settle within measured tolerances'

    @staticmethod
    def settled(actual, target, fingers=None, jaw=None):
        position, rotation = actual
        p, r = target
        return (np.linalg.norm(position-p) < .001
                and rotation_angle_deg(rotation,r) < 1.
                and (jaw is None or np.max(np.abs(fingers-jaw)) < .001))

    def command(self, t):
        dt = .05 if self.last_t is None else float(np.clip(t-self.last_t, 0., .1))
        self.last_t = t
        dryer, switch = self.pose('dryer'), self.pose('switch')
        rd = quat_xyzw_to_matrix(dryer[3:])
        slider = rd.T@(switch[:3]-dryer[:3])
        power = float(np.clip((slider[2]+.045)/SWITCH_TRAVEL,0,1))
        measured = [self.read_arm(side) for side in ('left','right')]
        self.check_payload_grasp(measured[0],dryer)
        lp0, lr0 = self.starts[0]
        rp0, rr0 = self.starts[1]
        _, fingers, actual_rp, actual_rr = measured[1]
        point = self.point.copy()
        # joint8 moves tip_right in -TCP-X. This witness is only for telemetry
        # and lateral alignment; geometry uses both measured finger joints.
        point[0] -= fingers[1]-RIGHT_JAW
        desired_point = np.array((-.0375,0.,0.))
        rotation = self.switchR
        local_rotation = rd.T@rotation
        tcp_local = desired_point-local_rotation@point
        tcp_local[2] = 0.
        support = self.geometry.support(fingers,tcp_local,local_rotation,slider)
        if support is None:
            raise RuntimeError('No actual collision surface below slider footprint')
        bottom = slider[2]-SLIDER_HALF[2]
        # Freeze the initial vertical switch datum: following slider height
        # during pushing would create positive feedback and unbounded overdrive.
        if not hasattr(self,'off_bottom'):
            self.off_bottom = -.052
        tcp_local[2] = self.off_bottom-.001-support[1]
        under = dryer[:3]+rd@tcp_local
        hover = under+rd@np.array((-.020,0,0))
        actual_local = rd.T@(actual_rp-dryer[:3])
        actual_support = self.geometry.support(fingers,actual_local,rd.T@actual_rr,slider)
        witness = actual_rp+actual_rr@point
        contact_target = under+rotation@point
        drift = 0. if self.braced_dryer is None else float(np.linalg.norm(dryer[:3]-self.braced_dryer[:3]))
        brace_rotation = 0. if self.braced_dryer is None else rotation_angle_deg(rd,quat_xyzw_to_matrix(self.braced_dryer[3:]))
        brace_ok = drift < .001 and brace_rotation < 1.
        age = t-self.phase_start
        phase = self.phase
        lp, lr, lg = self.hold.copy(), LEFT_R, GRIP
        rp, rr, rg = rp0.copy(), rr0, 1.
        right_condition = False
        left_actual = (measured[0][2],measured[0][3])
        right_actual = (actual_rp,actual_rr)
        # A force-closing grasp is blocked by the 30 mm handle; the measured
        # jaws must straddle it, rather than reach the unloaded -6 mm target.
        left_aperture = np.abs(measured[0][1])
        left_ok = (self.settled(left_actual,(self.hold,LEFT_R))
                   and np.all(left_aperture > .004) and np.all(left_aperture < .022))
        right_contact = phase in ('insert_below_switch','push_slider_up','hold_slider_on','retract_horizontal')
        if right_contact and (drift > .004 or brace_rotation > 3.):
            self.failure = 'dryer brace moved more than 4 mm or 3 degrees during switch contact'

        if phase == 'approach_handle_brace':
            s = ik.smoothstep(min(1.,age/3.))
            lp = lp0+s*(self.hold+HANDLE_APPROACH_OFFSET-lp0)
            lr = ik.slerp_matrix(lr0,LEFT_R,s)
            lg = 1.-.55*s
            self.gate(self.settled(left_actual,(lp,lr)),age,3.,5.,'enter_handle_brace',t)
        elif phase == 'enter_handle_brace':
            lp = self.hold+HANDLE_APPROACH_OFFSET*(1-ik.smoothstep(min(1.,age/2.)))
            lg = .45
            self.gate(self.settled(left_actual,(lp,lr)),age,2.,5.,'grip_and_brace',t)
        elif phase == 'grip_and_brace':
            lg = .45+(GRIP-.45)*ik.smoothstep(min(1.,age/2.))
            if self.stable_frames == 0:
                self.brace_sample = dryer.copy()
            stable_dryer = np.linalg.norm(dryer[:3]-self.brace_sample[:3]) < .001
            self.gate(left_ok and stable_dryer,age,2.,5.,'align_below_switch',t)
            if self.phase != phase:
                self.braced_dryer = dryer.copy()
                self.align_start = (actual_rp.copy(),actual_rr.copy())
        elif phase == 'align_below_switch':
            p0,r0 = self.align_start
            s = ik.smoothstep(min(1.,age/4.))
            rp,rr,rg = p0+s*(hover-p0), ik.slerp_matrix(r0,rotation,s), RIGHT_GRIP
            right_condition = self.settled(right_actual,(hover,rotation),fingers,RIGHT_JAW)
            if age>=4. and right_condition and brace_ok and left_ok:
                # Certify the entire translation rectangle containing insertion,
                # upward stroke and horizontal retraction, using the measured
                # loaded left fingers and TCP rotation. End-point vertices also
                # bound every intermediate translation of every triangle.
                left_pieces = [a@measured[0][3].T+measured[0][2]
                               for _,a in self.geometry.pieces(measured[0][1])]
                right_pieces = [np.concatenate([a@rotation.T+under+rd@np.array((x,0,z))
                                               for x in (-.025,0.) for z in (0.,.008)])
                                for _,a in self.geometry.pieces(fingers)]
                self.hand_clearance = minimum_separation(left_pieces,right_pieces)
                if self.hand_clearance < .001:
                    self.failure = 'measured loaded hands lack 1 mm certified clearance for switch sweep'
            self.gate(right_condition and brace_ok and left_ok and self.hand_clearance is not None
                      and self.hand_clearance>=.001,age,4.,5.,'insert_below_switch',t)
        elif phase == 'insert_below_switch':
            rp,rr,rg = hover+(under-hover)*ik.smoothstep(min(1.,age/2.)),rotation,RIGHT_GRIP
            clear_below = actual_support is not None and actual_support[1] <= bottom-.0002
            right_condition = self.settled(right_actual,(under,rotation),fingers,RIGHT_JAW)
            self.gate(right_condition and clear_below and brace_ok and left_ok,age,2.,5.,'push_slider_up',t)
            if self.phase != phase:
                self.press_start_power = power
                self.first_contact_stroke = max(0.,bottom-actual_support[1])
        elif phase == 'push_slider_up':
            stroke = min(.008,.001*age)
            rp,rr,rg = under+rd@np.array((0,0,stroke)),rotation,RIGHT_GRIP
            self.stable_frames = self.stable_frames+1 if power > .9 else 0
            if self.stable_frames >= 5:
                self.slide_end_local = rd.T@(rp-dryer[:3])
                self.transition('hold_slider_on',t)
            elif stroke > self.first_contact_stroke+.002 and power < self.press_start_power+.05:
                self.failure = 'no measured slider movement 2 mm beyond predicted first contact'
            elif age > 9.:
                self.failure = 'slider did not reach measured power > 0.9 within 8 mm stroke'
        elif phase == 'hold_slider_on':
            rp,rr,rg = dryer[:3]+rd@self.slide_end_local,rotation,RIGHT_GRIP
            if power < .7:
                self.failure = 'physical slider lost power while held'
            elif age >= .25:
                self.transition('retract_horizontal',t)
        elif phase == 'retract_horizontal':
            # Preserve Z until the mesh is physically clear of the slider.
            local = self.slide_end_local+np.array((-.025*ik.smoothstep(min(1.,age/2.)),0,0))
            rp,rr,rg = dryer[:3]+rd@local,rotation,RIGHT_GRIP
            self.gate(self.settled(right_actual,(rp,rr)) and brace_ok,age,2.,5.,'confirm_power_stays_on',t)
            if self.phase != phase:
                self.right_retreat = (rp.copy(),rr.copy())
        elif phase == 'confirm_power_stays_on':
            rp,rr = self.right_retreat
            rg = RIGHT_GRIP
            if power < .7:
                self.failure = 'physical slider did not remain on after horizontal retraction'
            elif age >= .5:
                self.transition('raise_clear_finger',t)
        elif phase == 'raise_clear_finger':
            p0,r0 = self.right_retreat
            rp,rr,rg = p0+np.array((0,0,.06*ik.smoothstep(min(1.,age/2.)))),r0,RIGHT_GRIP
            self.gate(self.settled(right_actual,(rp,rr)),age,2.,5.,'lift_dryer',t)
            if self.phase != phase:
                self.lift_origin = dryer[:3].copy()
                self.capture_payload(measured,dryer,switch)
        elif phase == 'lift_dryer':
            lp = self.hold+np.array((0,0,.10*ik.smoothstep(min(1.,age/3.))))
            p0,r0 = self.right_retreat
            s = ik.smoothstep(min(1.,age/3.))
            rp,rr,rg = p0+np.array((0,0,.06))+s*(rp0-p0-np.array((0,0,.06))),ik.slerp_matrix(r0,rr0,s),RIGHT_GRIP+(1-RIGHT_GRIP)*s
            lifted = dryer[2]-self.lift_origin[2] > .075
            self.gate(lifted and self.settled(left_actual,(lp,lr)) and power>.7,age,3.,5.,'verified_powered_lift_hold',t)
        elif phase == 'verified_powered_lift_hold':
            lp = self.hold+np.array((0,0,.10))
            if power < .7 or dryer[2]-self.lift_origin[2] < .065:
                self.failure = 'measured powered pickup was not maintained'
        else:
            raise RuntimeError('Unknown probe phase '+phase)

        targets = [(lp,lr,lg),(rp,rr,rg)]
        out = self.env._ready.detach().cpu().numpy().reshape(-1).copy()
        errors = []
        for i,side in enumerate(('left','right')):
            p,r,g = targets[i]
            actual,actual_rotation = measured[i][2:]
            error = p-actual
            # Integrate only settled free-space error. Freeze and decay during
            # contact; 18 mm windup previously overwhelmed a 4 mm slider.
            contact = right_contact or phase in ('hold_slider_on','confirm_power_stays_on','lift_dryer','verified_powered_lift_hold')
            if not contact and np.linalg.norm(error)<.02 and rotation_angle_deg(actual_rotation,r)<5.:
                self.bias[i] = np.clip(self.bias[i]+.8*dt*error,-.003,.003)
            elif contact:
                self.bias[i] *= .98
            self.q[i],pe,re = ik.solve(p+self.bias[i],ik.matrix_to_quat_xyzw(r),self.q[i],base_pos=self.env.rig.bases[side],iters=90)
            compensated=self.q[i].copy()
            if i==0 and phase in ('lift_dryer','verified_powered_lift_hold'):
                ramp=ik.smoothstep(min(1.,age/3.)) if phase=='lift_dryer' else 1.
                self.payload_bias=self.payload_offset(measured[i][0],ramp);compensated+=self.payload_bias
            out[7*i:7*i+6],out[7*i+6] = compensated,g
            errors.append({'side':side,'tcp':actual.tolist(),'tcp_rotation':actual_rotation.tolist(),'arm_joints':measured[i][0].tolist(),'target':p.tolist(),'tracking_error':float(np.linalg.norm(error)),
                           'tracking_rotation_error_deg':rotation_angle_deg(actual_rotation,r),'ik_error':float(pe),'ik_rotation_error':float(re),
                           'bias_m':self.bias[i].tolist(),'measured_finger_joints_m':measured[i][1].tolist()})
        delta = np.array([.04]*6+[.05]+[.04]*6+[.05])
        out = np.clip(np.clip(out,self.previous-delta,self.previous+delta),self.env._low,self.env._high)
        self.previous = out
        return out, {'complete':bool(phase=='verified_powered_lift_hold' and age>=1. and self.failure is None),'phase':phase,'phase_age_s':float(age),'next_phase':self.phase,'stable_frames':self.stable_frames,'arms':errors,
                     'payload_mass_kg':self.payload_mass,'payload_com_tcp':None if self.payload_com_tcp is None else self.payload_com_tcp.tolist(),'payload_target_offset_rad':self.payload_bias.tolist(),'measured_power_fraction':power,'switch_local_center':slider.tolist(),'switch_bottom_local_z':float(bottom),
                     'switch_contact_target':contact_target.tolist(),'measured_finger_contact_point':witness.tolist(),
                     'measured_clipped_support_z':None if actual_support is None else list(actual_support),
                     'predicted_under_support_top_z':float(self.off_bottom-.001),'dryer_brace_drift_m':drift,
                     'dryer_brace_rotation_deg':brace_rotation,'measured_loaded_hand_sweep_clearance_m':self.hand_clearance,'events':self.events[-3:],'failure':self.failure}


class DryerSweepProbe(DryerProbe):
    """Real pickup followed by a bounded upper-layer raster, explicitly partial.

    No inherited-power or synthetic held start: the ordinary switch/pickup
    gates must pass first. The last issued14D command is reused on entry.
    """
    def __init__(self,env):
        super().__init__(env)
        self.sweep=None;self.sweep_clock=0.;self.sweep_hold=0;self.sweep_last_t=None

    def command(self,t):
        from dryer_sweep import DryerSweepPlan
        if self.sweep is None:
            out,control=super().command(t)
            if control['complete'] and self.failure is None:
                measured=self.read_arm('left');dryer=self.pose('dryer');dr=quat_xyzw_to_matrix(dryer[3:])
                nominal_entry=out[:6]-getattr(self,'payload_bias',np.zeros(6))
                self.sweep=DryerSweepPlan(ik.tcp_pose(nominal_entry,self.env.rig.bases['left']),measured[2:],(dryer[:3],dr))
                self.sweep_previous=out.copy();self.sweep_last_t=t;self.sweep_start_t=t
                self.sweep_right=out[7:].copy()
                control.update(complete=False,phase='sweep_entry',next_phase='upper_layer_sweep',scope='Partial outer-layer motion only; not complete drying')
            return out,control
        dt=float(t-self.sweep_last_t);self.sweep_last_t=t
        measured=self.read_arm('left');dryer=self.pose('dryer');dr=quat_xyzw_to_matrix(dryer[3:])
        self.check_payload_grasp(measured,dryer)
        switch=self.pose('switch');power=float(np.clip((dr.T@(switch[:3]-dryer[:3]))[2]+.045,0,SWITCH_TRAVEL)/SWITCH_TRAVEL)
        drift,angle=self.sweep.grasp_error(measured[2:],(dryer[:3],dr))
        old=self.sweep_clock;current=self.sweep.target(old)
        error=float(np.linalg.norm(current[0]-measured[2]));rotation_error=rotation_angle_deg(current[1],measured[3])
        if not np.isfinite([dt,power,drift,angle,error,rotation_error]).all() or not 0<dt<=.1:self.failure='nonfinite or missing sweep state'
        elif power<.7:self.failure='physical slider lost power during sweep'
        elif drift>.005 or angle>5.:self.failure='dryer moved within measured grasp during sweep'
        elif t-self.sweep_start_t>self.sweep.duration+20.:self.failure='upper-layer sweep tracking deadline exceeded'
        if self.failure is None and error<=.005 and rotation_error<=3.:
            self.sweep_clock=min(self.sweep.duration,old+dt)
        target,rotation=self.sweep.target(self.sweep_clock)
        q,pe,re=ik.solve(target,ik.matrix_to_quat_xyzw(rotation),self.sweep_previous[:6]-self.payload_offset(measured[0]),base_pos=self.env.rig.bases['left'],iters=180,tol=1e-6)
        if pe>.0005 or re>.005:self.failure=self.failure or 'upper-layer sweep IK failed'
        # Live head/hair, tool and gripper clearance; no authored pose is read
        # back as achieved motion. AABB bounds are conservative absence tests.
        gap=self.sweep_clearance(measured,(target,rotation),(dryer[:3],dr))
        if not np.isfinite(gap) or gap<.003:self.failure=self.failure or 'sweep lacks 3 mm tool/hand clearance'
        out=self.sweep_previous.copy()
        if self.failure is None:
            # Same14D/joint/action bounds as the pickup controller, no I boost.
            out[:6]=np.clip(q+self.payload_offset(measured[0]),self.sweep_previous[:6]-.04,self.sweep_previous[:6]+.04)
            out=np.clip(out,self.env._low,self.env._high)
        out[7:]=self.sweep_right
        self.sweep_previous=out.copy();self.previous=out.copy()
        settled=self.sweep_clock>=self.sweep.duration and error<=.005 and rotation_error<=3.
        self.sweep_hold=self.sweep_hold+1 if settled else 0
        return out,{'phase':'upper_layer_sweep','complete':self.sweep_hold>=5 and self.failure is None,
                    'failure':self.failure,'path_clock_s':self.sweep_clock,'duration_s':self.sweep.duration,
                    'measured_power_fraction':power,'grasp_drift_m':drift,'grasp_rotation_deg':angle,
                    'position_error_m':error,'rotation_error_deg':rotation_error,'clearance_lower_bound_m':gap,'target':target.tolist(),
                    'target_rotation':rotation.tolist(),'ik_error_m':pe,'ik_error_rad':re,
                    'scope':'Partial upper-layer ordinary-motion diagnostic; moisture only from measured airflow'}


    def sweep_clearance(self,left,target,dryer):
        from dryer_sweep import bounds_gap,tool_and_hand_bounds
        from config import HEAD_RADIUS,RADIUS,LENGTH,TOP,LOCKS,SEGMENTS
        right=self.read_arm('right')
        pieces=self.geometry.pieces(left[1])
        actual=[a@left[3].T+left[2] for _,a in pieces]
        planned=[a@target[1].T+target[0] for _,a in pieces]
        planned_tool_r=target[1]@self.sweep.rotation.T
        planned_tool_p=target[0]-planned_tool_r@self.sweep.offset
        moving=[tool_and_hand_bounds(actual,dryer),tool_and_hand_bounds(planned,(planned_tool_p,planned_tool_r))]
        hair=self.base.scene['hair_model'];data=hair.data.body_link_pose_w
        data=data.torch if hasattr(data,'torch') else data
        values=data.detach().cpu().numpy()[0]
        mins=[];maxs=[]
        for name,v in zip(hair.body_names,values):
            if name=='head':
                # Enclose scalp, nose and neck, with the actual head rotation.
                hr=quat_xyzw_to_matrix(v[3:]);center=v[:3]+hr@np.array((0,0,-.05));half=np.abs(hr)@np.array((.075,.075,.125))
                lo=center-half;hi=center+half
            elif name.startswith('hair_'):
                axis=quat_xyzw_to_matrix(v[3:])[:,2]*(LENGTH/2-RADIUS)
                lo=v[:3]-np.abs(axis)-RADIUS;hi=v[:3]+np.abs(axis)+RADIUS
            else:continue
            mins.append(lo);maxs.append(hi)
        if len(mins)!=1+LOCKS*SEGMENTS:return 0.
        other=[a@right[3].T+right[2] for _,a in self.geometry.pieces(right[1])]
        obstacle=(np.r_[mins,[a.min((0,1)) for a in other]],np.r_[maxs,[a.max((0,1)) for a in other]])
        return min(min(float(a[0][:,2].min()-TOP),bounds_gap(a,obstacle)) for a in moving)
