"""State-assisted scripted demo, issuing ordinary bounded robot joint targets.

This controller is explicitly privileged relative to Dream: it observes the
native wrench pose to estimate the current grasp. It never writes any object
pose, attachment, force, material, score or actuator limit. Dream does not use it.
"""
import numpy as np
import re
from thread_coupling import rotation


def nipple_alignment(nominal_position,nominal_rotation,measured_pose):
    """Choose the nearest equivalent square-flat orientation, before a turn.

    Rotation is latched for each quarter-turn, so successful nipple rotation
    never moves the turning target further away. A clear approach can be
    realigned once after its observed orientation settles. Centre translation may follow
    the physically deforming rim. The four equivalent square orientations
    keep each return stroke within the original wrist workspace.
    """
    from wheel_scene import spoke_phase
    pose=np.asarray(measured_pose,float)
    if pose.shape!=(7,) or not np.isfinite(pose).all():raise ValueError('Finite nipple pose required')
    actual=rotation(pose[3:]);nominal=np.asarray(nominal_rotation,float)
    alternatives=[actual@spoke_phase(k*np.pi/2)@nominal.T for k in range(4)]
    index=int(np.argmax([np.trace(r) for r in alternatives]))
    correction=alternatives[index]
    return correction,dict(square_symmetry_quarters=index,
        orientation_correction_rad=float(np.arccos(np.clip((np.trace(correction)-1)/2,-1.,1.))),
        centre_correction_m=float(np.linalg.norm(pose[:3]-nominal_position)))


def compensated_tcp(nominal_tcp, nominal_rotation, measured_tcp, measured_rotation,
                    measured_tool_pose, reference_grip_position, reference_grip_rotation):
    pose=np.asarray(measured_tool_pose,float)
    if pose.shape!=(7,) or not np.isfinite(pose).all():
        raise ValueError('Finite measured tool pose required')
    R=rotation(pose[3:])
    grasp_position=R.T@(np.asarray(measured_tcp)-pose[:3])
    grasp_rotation=R.T@measured_rotation
    desired_tool_rotation=nominal_rotation@reference_grip_rotation.T
    desired_tool_position=nominal_tcp-desired_tool_rotation@reference_grip_position
    desired_tcp=desired_tool_position+desired_tool_rotation@grasp_position
    desired_rotation=desired_tool_rotation@grasp_rotation
    return desired_tcp,desired_rotation,dict(
        measured_grasp_position_in_tool_m=grasp_position.tolist(),
        measured_grasp_rotation_in_tool=grasp_rotation.tolist(),
        desired_tool_position_m=desired_tool_position.tolist(),
        desired_tool_rotation=desired_tool_rotation.tolist(),
        current_tool_position_error_m=float(np.linalg.norm(pose[:3]-desired_tool_position)),
        tcp_correction_m=float(np.linalg.norm(desired_tcp-nominal_tcp)))


def ik_acceptance(stage, tcp_error_m, rotation_error_rad, tool_error_m, *, solved_mouth_clearance_m=None):
    """Bound a clear approach by the physical jaw lead-in, not an exact TCP.

    The 5 mm mouth admits a 3.23 mm nipple with 0.885 mm half-clearance.
    During the approach only, reserve at least 0.135 mm of that clearance and
    bound the position error of every jaw point within 10 mm of its centre.
    Up to0.01rad approach orientation error is allowed only inside that same
    pointwise bound; a separate near-zero angular threshold would reject a
    command that still fits the real lead-in. Native contact establishes fit.
    With the explicit longer-lead-in diagnostic, an insertion point still
    outside the mouth can use the same capture bound. Turning and seated
    insertion retain the original stricter gate. This
    is a command-quality check; only native contact can establish engagement.
    """
    values=np.asarray((tcp_error_m,rotation_error_rad,tool_error_m),float)
    if not np.isfinite(values).all() or np.any(values<0):
        return False,{'mode':'invalid','jaw_error_bound_m':None}
    jaw_error=tool_error_m+2*.010*np.sin(rotation_error_rad/2)
    exact=tcp_error_m<=.0002 and rotation_error_rad<=.002
    capture_stage=(stage.startswith('Approach nipple ') or
        (stage.endswith(' insert') and solved_mouth_clearance_m is not None and
         np.isfinite(solved_mouth_clearance_m) and solved_mouth_clearance_m>=0.))
    capture=(capture_stage and tcp_error_m<=.001
             and rotation_error_rad<=.01 and jaw_error<=.00075)
    # The longer diagnostic lead-in remains outside the mouth. Its command
    # may be approximate only if the solved full square is still at least2mm
    # beyond the jaw ends; the tight-slot capture allowance need not apply
    # before physical contact. Native/hull checks remain independently required.
    clear_lead=(solved_mouth_clearance_m is not None and
        np.isfinite(solved_mouth_clearance_m) and solved_mouth_clearance_m>=.002 and
        (stage.startswith('Approach nipple ') or stage.endswith(' insert')) and tcp_error_m<=.002 and
        tool_error_m<=.002 and rotation_error_rad<=.01)
    return bool(exact or capture or clear_lead),dict(mode='strict' if exact else 'lead_in' if capture else 'clear_lead_in' if clear_lead else 'rejected',
        jaw_error_bound_m=float(jaw_error),lead_in_half_clearance_m=.000885)


def orientations_settled(samples):
    """Five20Hz observations within0.01rad; no inferred or edited physics."""
    if len(samples)<5:return False
    last=np.asarray(samples[-1])
    angles=[np.arccos(np.clip((np.trace(np.asarray(R)@last.T)-1)/2,-1.,1.)) for R in samples[-5:]]
    return bool(np.isfinite(angles).all() and max(angles)<=.01)


def engagement_key(spoke,stage):
    """A clear approach and its first insertion are one physical engagement.

    Relatching at the stage-name boundary would chase nipple rotation caused
    by early mouth contact and abruptly rotate the long tool handle.
    """
    quarter=re.search(r'quarter (\d+)',stage)
    return int(spoke),1 if quarter is None else int(quarter.group(1))


def nipple_mouth_clearance_m(tool_pose,nipple_pose):
    """Signed clearance of the entire square flats beyond the open jaw ends."""
    tool=np.asarray(tool_pose,float);nipple=np.asarray(nipple_pose,float)
    if tool.shape!=(7,) or nipple.shape!=(7,) or not np.isfinite([tool,nipple]).all():
        raise ValueError('Finite tool and nipple poses required')
    R=rotation(tool[3:]);N=rotation(nipple[3:])
    relative=R.T@(nipple[:3]-tool[:3]);orientation=R.T@N
    half=np.array((.006,.00323,.00323))/2
    return float(relative[1]-np.abs(orientation[1])@half-.008)


def blended_nipple_target(position, orientation, nominal_centre, actual_centre,
                          correction, fraction=1.):
    """Smooth an optional clear-space correction without changing object state.

    A relocation diagnostic can blend measured nipple alignment along its
    clear-front transition, instead of demanding the entire correction at the
    first parked waypoint. Contact and turn waypoints use fraction1. The
    existing route is exactly preserved when the optional field is absent.
    """
    if not np.isfinite(fraction) or not 0.<=fraction<=1.:
        raise ValueError('Nipple feedback fraction must be between0 and1')
    position=np.asarray(position,float);orientation=np.asarray(orientation,float)
    nominal_centre=np.asarray(nominal_centre,float)
    if fraction == 0.:
        return np.asarray(position).copy(), np.asarray(orientation).copy()
    C = np.asarray(correction)
    if fraction != 1.:
        from scipy.spatial.transform import Rotation
        C = Rotation.from_rotvec(Rotation.from_matrix(C).as_rotvec()*fraction).as_matrix()
    centre = (np.asarray(actual_centre) if fraction == 1. else
              np.asarray(nominal_centre)+fraction*(np.asarray(actual_centre)-nominal_centre))
    return centre+C@(position-nominal_centre), C@orientation


class ToolPoseServo:
    def __init__(self,plan,*,nipple_feedback=False):
        cal=plan['held_grasp_calibration']
        self.position=np.array(cal['tcp_position_in_tool_m'])
        self.rotation=np.array(cal['tcp_rotation_in_tool'])
        self.first_step=int(cal['applies_from_step'])
        self.release_step=int(plan.get('released_tool',{}).get('from_step',10**9))
        self.plan=plan;self.nipple_feedback=bool(nipple_feedback)
        self.latched_key=None;self.nipple_rotation=None
        self.approach_settle_samples=[];self.realigned_approaches=set()
        self.last=None
        self.withdrawal_pose=None
        self.withdrawal_start=None
        self.references=[cal]+list(plan.get('additional_grasp_calibrations',[]))
        starts=[int(c['applies_from_step']) for c in self.references]
        if starts!=sorted(set(starts)):raise ValueError('Grasp references must have distinct increasing start actions')
        from withdrawal import axial_target
        for reference in self.references:
            axial_target(reference['tcp_position_in_tool_m'],reference['tcp_rotation_in_tool'],0.)
        first_withdrawal=plan.get('measured_axial_withdrawal')
        self.withdrawals=([first_withdrawal] if first_withdrawal is not None else [])+list(plan.get('additional_measured_axial_withdrawals',[]))
        previous=0
        for entry in self.withdrawals:
            if not previous<int(entry['from_step'])<int(entry['move_through_step'])<=int(entry['through_step'])<=len(plan['frames']):
                raise ValueError('Withdrawal intervals must be bounded, increasing and disjoint')
            if not 0.<float(entry['distance_m'])<=.05:raise ValueError('Bounded positive axial distance required')
            previous=int(entry['through_step'])

    def target_spoke(self,step):
        if not self.nipple_feedback or not 1<=step<=len(self.plan['frames']):return None
        match=re.search(r'[Nn]ipple (\d+)',self.plan['frames'][step-1]['stage'])
        return None if match is None else int(match.group(1))

    def command(self,step,nominal,measured_joint_positions,measured_tool_pose,measured_nipple_pose=None):
        from yam_arena import ik,station
        target=np.asarray(nominal,float).copy()
        if step<self.first_step or step>=self.release_step:
            self.last={'active':False};return target
        reference=next(c for c in reversed(self.references) if int(c['applies_from_step'])<=step)
        self.position=np.asarray(reference['tcp_position_in_tool_m'],float)
        self.rotation=np.asarray(reference['tcp_rotation_in_tool'],float)
        q=np.asarray(measured_joint_positions,float)[7:13]
        p,R=ik.tcp_pose(target[7:13],station.BASES['right'])
        nipple_diagnostic=None
        spoke=self.target_spoke(step)
        if spoke is not None:
            if measured_nipple_pose is None:raise ValueError('Native nipple feedback requested but missing')
            from plan_turns import nominal_nipple
            frame=self.plan['frames'][step-1]
            clear_margin=self.plan.get('require_clear_mouth_before_cross_plane_m')
            if clear_margin is not None and frame['stage'].endswith(' clear front'):
                clearance=nipple_mouth_clearance_m(measured_tool_pose,measured_nipple_pose)
                if clearance<float(clear_margin):
                    raise RuntimeError(f'Wrench remains engaged before cross-plane withdrawal: mouth clearance={clearance}m')
            centre,nR=nominal_nipple(spoke,np.deg2rad(self.plan['nipple_flat_phase_deg']),
                                   np.deg2rad(frame['nominal_wheel_clocking_deg']),
                                   fixture_layout=self.plan.get('fixture_layout','reference'))
            key=engagement_key(spoke,frame['stage'])
            clear_settle=frame['stage'].startswith('Approach nipple ') and frame['stage'].endswith(' settle')
            realign=False
            if frame.get('realign_square_before_withdrawal',False):
                if not frame['stage'].endswith('relax square fit'):
                    raise ValueError('Post-turn realignment requires an explicit unloaded-fit stage')
                realign=True
            if clear_settle and key not in self.realigned_approaches:
                candidate,_=nipple_alignment(centre,nR,measured_nipple_pose)
                self.approach_settle_samples.append(candidate)
                self.approach_settle_samples=self.approach_settle_samples[-5:]
                realign=orientations_settled(self.approach_settle_samples)
            elif not clear_settle:
                self.approach_settle_samples=[]
            if realign or key!=self.latched_key:
                self.nipple_rotation,nipple_diagnostic=nipple_alignment(centre,nR,measured_nipple_pose)
                self.latched_key=key
                if realign:self.realigned_approaches.add(key)
            else:
                nipple_diagnostic={'latched_from_earlier_frame':True}
            original_p=p.copy();original_R=R.copy()
            feedback_fraction=float(frame.get('nipple_feedback_fraction',1.))
            if feedback_fraction!=1. and not frame['stage'].startswith('Clear approach nipple '):
                raise ValueError('Only the clear-front transition may blend nipple feedback')
            p,R=blended_nipple_target(p,R,centre,np.asarray(measured_nipple_pose)[:3],
                                     self.nipple_rotation,feedback_fraction)
            nipple_diagnostic.update(spoke=spoke,quarter=key[1],rotation_is_latched=True,
                feedback_fraction=feedback_fraction,
                realigned_after_stable_approach=realign,
                target_translation_correction_m=float(np.linalg.norm(p-original_p)),
                target_orientation_correction_rad=float(np.arccos(np.clip((np.trace(R@original_R.T)-1)/2,-1,1))))
        withdrawal=next((entry for entry in self.withdrawals if entry['from_step']<=step<=entry['through_step']),None)
        if withdrawal is not None:
            from withdrawal import axial_target
            first=int(withdrawal['from_step']);last=int(withdrawal['move_through_step'])
            if not first<last<=int(withdrawal['through_step']):
                raise ValueError('Invalid bounded axial withdrawal interval')
            if self.withdrawal_start!=first:
                if step!=first:raise RuntimeError('Measured withdrawal must start at its first action')
                self.withdrawal_pose=(np.asarray(measured_tool_pose)[:3].copy(),rotation(np.asarray(measured_tool_pose)[3:]))
                self.withdrawal_start=first
            fraction=ik.smoothstep(min(1.,(step-first+1)/(last-first+1)))
            tool_p,tool_R=axial_target(*self.withdrawal_pose,float(withdrawal['distance_m'])*fraction)
            p=tool_p+tool_R@self.position;R=tool_R@self.rotation
            nipple_diagnostic=dict(nipple_diagnostic or {},measured_axial_withdrawal=True,
                commanded_withdrawal_m=float(withdrawal['distance_m'])*fraction,
                orientation_latched_from_native_tool=True)
        actual_p,actual_R=ik.tcp_pose(q,station.BASES['right'])
        desired_p,desired_R,diagnostic=compensated_tcp(p,R,actual_p,actual_R,measured_tool_pose,self.position,self.rotation)
        if diagnostic['tcp_correction_m']>.06:
            raise RuntimeError('Tool grasp changed by more than60mm; stop scripted servo')
        solution,pe,rotation_error=ik.solve(desired_p,ik.matrix_to_quat_xyzw(desired_R),target[7:13],
            station.BASES['right'],iters=150,damping=.008,tol=1e-7)
        solved_p,solved_R=ik.tcp_pose(solution,station.BASES['right'])
        solved_tool_R=solved_R@np.asarray(diagnostic['measured_grasp_rotation_in_tool']).T
        solved_tool_p=solved_p-solved_tool_R@np.asarray(diagnostic['measured_grasp_position_in_tool_m'])
        tool_error=float(np.linalg.norm(solved_tool_p-diagnostic['desired_tool_position_m']))
        solved_clearance=None
        if (measured_nipple_pose is not None and
            self.plan.get('require_clear_mouth_before_cross_plane_m') is not None):
            from wheel_scene import quat
            solved_clearance=nipple_mouth_clearance_m(np.r_[solved_tool_p,quat(solved_tool_R)],measured_nipple_pose)
        accepted,quality=ik_acceptance(self.plan['frames'][step-1]['stage'],pe,rotation_error,tool_error,
            solved_mouth_clearance_m=solved_clearance)
        quality['solved_mouth_clearance_m']=solved_clearance
        if not accepted:
            raise RuntimeError(f'Tool servo IK failed: position={pe}, rotation={rotation_error}')
        target[7:13]=solution
        self.last={'active':True,**diagnostic,'nipple_target':nipple_diagnostic,
            'ik_position_error_m':float(pe),'ik_rotation_error_rad':float(rotation_error),
            'ik_tool_position_error_m':tool_error,'ik_acceptance':quality}
        return target
