"""Disclosed pose-assisted conversion of a fixed blade path to robot targets.

Only measured rigid poses are read. This helper never writes body states,
creates attachments or applies forces. The caller keeps ordinary joint limits
and rate limits, and the physical grasp must retain the tool throughout.
"""
import numpy as np
from scipy.spatial.transform import Rotation,Slerp


def assistance_stop_step(plan):
    """Last assisted action in a frozen plan; intentional release is unassisted.

    Old pickup/entry prefixes use compensation until their final action. A plan
    that puts the tool down must declare the final closed-grasp action, rather
    than asking the controller to follow a freely released handle.
    """
    count=len(plan['frames'])
    stop=plan.get('pose_assisted_entry_until_step',count)
    if plan.get('requires_pose_compensation_release_boundary') and 'pose_assisted_entry_until_step' not in plan:
        raise ValueError('Tool-release plan must declare the last assisted action')
    if type(stop) is not int or not 140<stop<=count:
        raise ValueError('Last assisted action must follow pickup and lie within the frozen plan')
    return stop


class ToolEntryCompensation:
    def __init__(self,tcp_offset,alpha=.1):
        self.offset=np.asarray(tcp_offset,float)
        if self.offset.shape!=(3,) or not np.isfinite(self.offset).all():raise ValueError('Invalid TCP offset')
        if not 0.<alpha<=1.:raise ValueError('Invalid filter coefficient')
        self.alpha=alpha;self.translation=None;self.rotation=None;self.updates=0
        self.maximum_grasp_position_change_m=0.;self.maximum_grasp_rotation_change_rad=0.

    def target(self,tool_position,tool_rotation,hand_pose,tool_pose):
        """Return TCP position/orientation for the measured, filtered grasp."""
        hand,tool=np.asarray(hand_pose,float),np.asarray(tool_pose,float)
        target=np.asarray(tool_position,float);target_r=np.asarray(tool_rotation,float)
        if (hand.shape!=(7,) or tool.shape!=(7,) or target.shape!=(3,) or target_r.shape!=(3,3)
                or not all(np.isfinite(x).all() for x in (hand,tool,target,target_r))):
            raise ValueError('Invalid measured tool-entry state')
        hand_r=Rotation.from_quat(hand[3:]).as_matrix();tool_r=Rotation.from_quat(tool[3:]).as_matrix()
        relative_p=hand_r.T@(tool[:3]-hand[:3]);relative_r=hand_r.T@tool_r
        if np.linalg.norm(relative_p-self.offset)>.04:
            raise RuntimeError('Tool is outside the physical grasp region; entry compensation stopped')
        if self.translation is None:
            self.translation=relative_p.copy();self.rotation=relative_r.copy()
            self.reference_p=relative_p.copy();self.reference_r=relative_r.copy()
        else:
            self.translation+=(relative_p-self.translation)*self.alpha
            self.rotation=Slerp([0.,1.],Rotation.from_matrix([self.rotation,relative_r]))(self.alpha).as_matrix()
        slip_p=float(np.linalg.norm(relative_p-self.reference_p))
        slip_r=float(np.linalg.norm(Rotation.from_matrix(relative_r@self.reference_r.T).as_rotvec()))
        self.maximum_grasp_position_change_m=max(self.maximum_grasp_position_change_m,slip_p)
        self.maximum_grasp_rotation_change_rad=max(self.maximum_grasp_rotation_change_rad,slip_r)
        # The handle is70mm long. Up to20mm of axial regrasp still leaves
        # the18mm pads on the handle; the40mm enclosure guard above remains.
        if slip_p>.020 or slip_r>np.deg2rad(15.):
            raise RuntimeError('Measured tool slip exceeds entry compensation domain')
        self.updates+=1
        desired_hand_r=target_r@self.rotation.T
        desired_hand_p=target-desired_hand_r@self.translation
        return desired_hand_p+desired_hand_r@self.offset,desired_hand_r

    def summary(self):
        return dict(policy='fixed blade path with filtered native tool-in-gripper pose compensation',
                    filter_alpha=self.alpha,updates=self.updates,
                    maximum_grasp_position_change_m=self.maximum_grasp_position_change_m,
                    maximum_grasp_rotation_change_rad=self.maximum_grasp_rotation_change_rad,
                    state_writes=False,hidden_forces_or_attachments=False)
