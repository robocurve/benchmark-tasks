"""Optional scripted remover-pose compensation; ordinary joint commands only.

The runner enables this only with an explicit scripted-demo flag. Native
object poses may guide a disclosed scripted feasibility demo,
but never Dream. Nothing here writes objects, forces, materials or attachments.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def compensate_grasp(nominal_tcp, nominal_rotation, measured_tcp, measured_rotation,
                     measured_tool_pose, reference_position, reference_rotation):
    pose=np.asarray(measured_tool_pose,float)
    if pose.shape!=(7,) or not np.isfinite(pose).all():
        raise ValueError('Finite remover pose required')
    tool_R=Rotation.from_quat(pose[3:]).as_matrix()
    grasp_position=tool_R.T@(np.asarray(measured_tcp)-pose[:3])
    grasp_rotation=tool_R.T@measured_rotation
    wanted_R=nominal_rotation@reference_rotation.T
    wanted_p=np.asarray(nominal_tcp)-wanted_R@reference_position
    target_p=wanted_p+wanted_R@grasp_position
    target_R=wanted_R@grasp_rotation
    return target_p,target_R,dict(
        grasp_position_in_tool_m=grasp_position.tolist(),
        grasp_rotation_in_tool=grasp_rotation.tolist(),
        desired_tool_position_m=wanted_p.tolist(),
        desired_tool_rotation=wanted_R.tolist(),
        current_tool_position_error_m=float(np.linalg.norm(wanted_p-pose[:3])),
        tcp_grasp_correction_m=float(np.linalg.norm(target_p-nominal_tcp)))


class RemoverPoseServo:
    def __init__(self,plan,velocity_damping_s=0.):
        self.plan=plan
        stages=[f['stage'] for f in plan['frames']]
        self.first=stages.index('Orient above the crown')+1
        self.release=stages.index('Release the remover and extracted staple')+1
        self.velocity_damping_s=float(velocity_damping_s)
        if not 0.<=self.velocity_damping_s<=.1:raise ValueError('Bounded tool damping required')
        self.contact_first=next((i+1 for i,s in enumerate(stages) if s=='Settle the heel6mm above paper'),self.first)
        self.last={'active':False}

    def command(self,step,nominal,measured_joint_positions,native_remover_pose,native_remover_velocity=None):
        if step<self.first or step>=self.release:
            self.last={'active':False}
            return np.asarray(nominal,float).copy()
        from yam_arena import ik,station
        from coordinate_frame import to_station
        from plan_reach import TOOL_TCP,GRIP
        target=np.asarray(nominal,float).copy()
        observed=np.asarray(measured_joint_positions,float)
        pose=np.asarray(native_remover_pose,float).copy()
        pose[:3]=to_station(pose[:3])
        p,R=ik.tcp_pose(target[7:13],station.BASES['right'])
        current_p,current_R=ik.tcp_pose(observed[7:13],station.BASES['right'])
        desired_p,desired_R,report=compensate_grasp(p,R,current_p,current_R,pose,TOOL_TCP,GRIP)
        correction=np.zeros(3)
        if self.velocity_damping_s and step>=self.contact_first:
            velocity=np.asarray(native_remover_velocity,float)
            if velocity.shape!=(6,) or not np.isfinite(velocity).all():raise ValueError('Native rigid velocity required for damping')
            correction=np.clip(-self.velocity_damping_s*velocity[:3],-.001,.001)
            desired_p=desired_p+correction
        report['velocity_damping_s']=self.velocity_damping_s
        report['bounded_velocity_correction_m']=correction.tolist()
        if report['tcp_grasp_correction_m']>.02:
            raise RuntimeError('Remover grasp changed by more than20mm')
        q,pe,re=ik.solve(desired_p,ik.matrix_to_quat_xyzw(desired_R),target[7:13],
                        station.BASES['right'],iters=150,damping=.008,tol=1e-7)
        if pe>.0002 or re>.002:
            raise RuntimeError(f'Remover pose target unreachable: {pe}m,{re}rad')
        target[7:13]=q
        self.last=dict(active=True,**report,ik_position_error_m=float(pe),ik_rotation_error_rad=float(re))
        return target
