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
            out[7*i:7*i+6],out[7*i+6] = self.q[i],g
            errors.append({'side':side,'tcp':actual.tolist(),'tcp_rotation':actual_rotation.tolist(),'arm_joints':measured[i][0].tolist(),'target':p.tolist(),'tracking_error':float(np.linalg.norm(error)),
                           'tracking_rotation_error_deg':rotation_angle_deg(actual_rotation,r),'ik_error':float(pe),'ik_rotation_error':float(re),
                           'bias_m':self.bias[i].tolist(),'measured_finger_joints_m':measured[i][1].tolist()})
        delta = np.array([.04]*6+[.05]+[.04]*6+[.05])
        out = np.clip(np.clip(out,self.previous-delta,self.previous+delta),self.env._low,self.env._high)
        self.previous = out
        return out, {'complete':bool(phase=='verified_powered_lift_hold' and age>=1. and self.failure is None),'phase':phase,'phase_age_s':float(age),'next_phase':self.phase,'stable_frames':self.stable_frames,'arms':errors,
                     'measured_power_fraction':power,'switch_local_center':slider.tolist(),'switch_bottom_local_z':float(bottom),
                     'switch_contact_target':contact_target.tolist(),'measured_finger_contact_point':witness.tolist(),
                     'measured_clipped_support_z':None if actual_support is None else list(actual_support),
                     'predicted_under_support_top_z':float(self.off_bottom-.001),'dryer_brace_drift_m':drift,
                     'dryer_brace_rotation_deg':brace_rotation,'measured_loaded_hand_sweep_clearance_m':self.hand_clearance,'events':self.events[-3:],'failure':self.failure}
