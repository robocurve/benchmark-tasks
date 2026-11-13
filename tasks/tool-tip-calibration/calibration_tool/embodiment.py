"""A pointed tool fixed to the left gripper, with measured reference contact."""

from pathlib import Path

import numpy as np

from .mechanism import MechanismEmbodiment
from .scoring import CalibrationScore, REFERENCE, TIP_LOCAL

TASK_FILE = Path(__file__).resolve().parents[1] / 'tool_tip_calibration.py'


class CalibrationEmbodiment(MechanismEmbodiment):
    ENV_NAME = 'yam_tool_calibration'
    TASK_FILE = TASK_FILE

    def builders(self):
        from .assets import BUILDERS
        return BUILDERS

    def layout(self):
        from yam_arena import ik, station
        from yam_arena.constants import READY
        pos, rot = ik.tcp_pose(np.asarray(READY), station.BASES['left'])
        return {'tool': np.r_[pos, ik.matrix_to_quat_xyzw(rot)],
                'reference': np.array([.27,.22,.7475,0.,0.,0.,1.])}

    def configure_robot(self, robot):
        from isaaclab.sensors import ContactSensorCfg
        from isaaclab_arena.utils.configclass import make_configclass
        sensor = ContactSensorCfg(prim_path='{ENV_REGEX_NS}/tool', update_period=0., history_length=1,
                                  filter_prim_paths_expr=['{ENV_REGEX_NS}/reference'])
        robot.scene_config = make_configclass('CalibrationSceneCfg',
            [('tip_contact', ContactSensorCfg, sensor)], bases=(type(robot.scene_config),))()

    def make_joints(self, js, origin):
        from yam_arena.constants import TCP_OFFSET
        from yam_arena.robot import asset_link_paths
        js.fixed('tool_mount', 'LeftArm'+asset_link_paths()['gripper'], 'tool', TCP_OFFSET)

    def on_reset(self):
        self.scorer = CalibrationScore(self.info.control_hz)

    def measured(self):
        from yam_arena.constants import TCP_OFFSET
        from yam_arena.urdf import quat_xyzw_to_matrix
        pose = self.poses()['tool']
        robot = self._base.scene['left_robot']
        body_ids, _ = robot.find_bodies('gripper')
        grip_pose = robot.data.body_link_pose_w.torch[0,body_ids[0]].cpu().numpy().astype(float)
        grip_rot = quat_xyzw_to_matrix(grip_pose[3:])
        grip_pos = grip_pose[:3]-self.origin()+grip_rot@np.asarray(TCP_OFFSET)
        rot = quat_xyzw_to_matrix(pose[3:])
        forces = self._base.scene['tip_contact'].data.force_matrix_w
        forces = forces.torch if hasattr(forces, 'torch') else forces
        force = float(np.linalg.norm(forces[0].cpu().numpy().sum(axis=(0,1))))
        return pose[:3], rot, grip_pos, grip_rot, force

    def measure(self):
        pos, rot, grip_pos, grip_rot, force = self.measured()
        score = self.scorer.update(pos, rot, grip_pos, grip_rot, force)
        tip = pos + rot @ TIP_LOCAL
        info = {'score': score, 'success': self.scorer.success, 'tip_m': tip,
                'tip_error_mm': (tip-REFERENCE)*1000., 'reference_contact_n': force,
                'mount_error_mm': float(np.linalg.norm(pos-grip_pos)*1000.),
                'mount_angle_deg': float(np.degrees(np.arccos(np.clip(
                    (np.trace(grip_rot.T@rot)-1.)/2.,-1.,1.)))),
                'tool_axis': -rot[:,2],
                'credited_axes': self.scorer.axes, 'credited_tips': self.scorer.tips,
                'held_steps': self.scorer.held}
        return info, self.scorer.success
