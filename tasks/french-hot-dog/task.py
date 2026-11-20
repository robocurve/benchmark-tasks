"""Prepare an end-filled hot dog by piercing the bun, seating it and inserting a sausage."""
import os
from pathlib import Path
import tempfile

import numpy as np

from resist_isaac.state import pose_state, reset_joints, tensor
from resist_isaac.usd_assets import create_asset
from .geometry import (
    BUN_FINAL, BUN_LENGTH, BUN_START, CRADLE_AXIS, SAUSAGE_LENGTH, SAUSAGE_ROTATION_XYZW,
    SAUSAGE_START, SPIKE_TIP, TABLE_Z, geometry,
)

CONDITIONS = ('recoverable',)
HOLE_DEPTH = 0.100
INSERTION = 0.095


def distance_to_axis(point, center, axis, half_length):
    offset = point - center
    return np.linalg.norm(offset - axis * np.clip(offset @ axis, -half_length, half_length))


class FrenchHotDog:
    physics_hz = 240
    hold_s = 1.0
    retry_blank_camera_frames = True

    def __init__(self):
        self.app_device = os.environ.get('HOTDOG_RENDER_DEVICE', 'cuda:0')
        self.descriptions = geometry()
        self.contacts = None
        self.subscription = None
        self.base = None
        self.saved_contact_setting = None
        self.source_dir = None

    def build_scene(self, embodiment):
        from isaaclab_arena.assets.object import Object, ObjectType
        from isaaclab_arena.utils.pose import Pose
        from pxr import PhysxSchema, Usd, UsdPhysics

        temp = Path(tempfile.gettempdir()).resolve()
        source = Path(__file__).resolve().parent
        if temp == source or source in temp.parents:
            raise RuntimeError('Set TMPDIR to a writable directory outside the task folder.')
        self.source_dir = tempfile.TemporaryDirectory(prefix='hotdog-usd-', dir=temp)
        types = dict(bun=ObjectType.ARTICULATION, sausage=ObjectType.RIGID,
                     fixture=ObjectType.BASE, mandrel=ObjectType.RIGID)
        poses = dict(bun=Pose(position_xyz=BUN_START),
                     sausage=Pose(position_xyz=SAUSAGE_START, rotation_xyzw=SAUSAGE_ROTATION_XYZW))
        objects = []
        for name, description in self.descriptions.items():
            path = create_asset(description, Path(self.source_dir.name) / f'{name}.usda', source)
            if name == 'bun':
                stage = Usd.Stage.Open(str(path))
                for prim in stage.Traverse():
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
                stage.GetRootLayer().Save()
            objects.append(Object(name=name, object_type=types[name], usd_path=str(path),
                                  initial_pose=poses.get(name, Pose(position_xyz=(0., 0., 0.)))))
        return objects

    def configure_native(self, cfg):
        cfg.scene.bun.init_state.joint_pos = {'cavity': 0.}
        cfg.scene.bun.init_state.joint_vel = {'cavity': 0.}
        for name in ('left_robot', 'right_robot'):
            getattr(cfg.scene, name).actuators['gripper'].joint_effort_limit = 40.

    def initial_action(self):
        from yam_arena.constants import READY
        return np.r_[READY, 1., READY, 1.]

    def reset(self, base):
        self.close()
        reset_joints(base, 'bun', {'cavity': 0.})
        self.qualified = 0.
        self.last_depth = 0.
        self.peak_force = 0.
        self.pierced = False
        self.seated = False
        self.out_of_order = False
        self.seat_steps = 0
        self.last_step = None
        self.sample_error = None
        self.loaded_steps = 0

    def pose_state(self, base, name, part=None):
        return pose_state(base.scene[name], part)

    def pose(self, base, name, part=None):
        from yam_arena.urdf import quat_xyzw_to_matrix
        p, q, v = self.pose_state(base, name, part)
        # Controller waypoints keep these values across native simulation steps.
        return p.cpu().numpy().copy(), quat_xyzw_to_matrix(q.cpu().numpy()), v.cpu().numpy().copy()

    def read_depth(self, base):
        asset = base.scene['bun']
        return -float(tensor(asset.data.joint_pos)[0, asset.joint_names.index('cavity')])

    def begin_episode(self, base):
        import omni.usd
        from omni.physx import get_physx_interface
        from pxr import Usd, UsdPhysics

        self.base = base
        self.saved_contact_setting = base.sim.get_setting('/physics/disableContactProcessing')
        base.sim.set_setting('/physics/disableContactProcessing', False)
        stage = omni.usd.get_context().get_stage()
        paths = [str(p.GetPath()) for p in stage.Traverse(Usd.TraverseInstanceProxies())
                 if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        crumb, = [p for p in paths if p.endswith('/bun/crumb')]
        spike, = [p for p in paths if p.endswith('/mandrel/pike')]
        view = base.sim.physics_manager.get_physics_sim_view()
        self.contacts = view.create_rigid_contact_view(crumb, filter_patterns=[spike], max_contact_data_count=128)
        if list(self.contacts.sensor_paths) != [crumb]:
            raise RuntimeError('Native bun crumb contact view did not bind.')
        self.bun_view = view.create_rigid_body_view(next(p for p in paths if p.endswith('/bun/shell')))
        self.bun_dof = base.scene['bun'].root_physx_view
        self.depth_index = base.scene['bun'].joint_names.index('cavity')
        self.last_depth = self.read_depth(base)
        self.subscription = get_physx_interface().subscribe_physics_on_step_events(
            self.sample, pre_step=False, order=0)
        self.validate_reset(base)

    def sample(self, dt):
        if self.sample_error is not None:
            return
        try:
            from yam_arena.urdf import quat_xyzw_to_matrix

            depth = -float(self.bun_dof.get_dof_positions().numpy()[0, self.depth_index])
            # This is the normal reaction between the pike and crumb, not a wrist measurement.
            force = np.asarray(self.contacts.get_contact_force_matrix(dt=float(dt)).numpy())[0, 0]
            if not np.isfinite(depth) or not np.isfinite(force).all():
                raise RuntimeError('Missing or nonfinite native puncture evidence.')
            if abs(force[2]) >= 1.5:
                self.loaded_steps += 1
                self.peak_force = max(self.peak_force, abs(float(force[2])))
                pose = np.asarray(self.bun_view.get_transforms().numpy())[0]
                rotation = quat_xyzw_to_matrix(pose[3:])
                local_tip = rotation.T @ (np.asarray(SPIKE_TIP) - pose[:3])
                aligned = (rotation[2, 2] > 0.985 and np.linalg.norm(local_tip[:2]) < 0.004
                           and abs(BUN_LENGTH / 2 - local_tip[2] - depth) < 0.005)
                # The credited frontier cannot jump across unobserved or other-actor travel.
                if aligned and depth > self.last_depth and self.last_depth <= self.qualified + 0.002:
                    self.qualified = max(self.qualified, depth)
            self.last_depth = depth
        except Exception as exc:
            self.sample_error = repr(exc)

    def validate_reset(self, base):
        bun, rotation, velocity = self.pose(base, 'bun', 'shell')
        sausage, _, meat_velocity = self.pose(base, 'sausage')
        if (np.linalg.norm(bun - BUN_START) > 0.004 or np.linalg.norm(sausage - SAUSAGE_START) > 0.005
                or rotation[2, 2] < 0.995 or abs(self.read_depth(base)) > 0.001
                or max(np.linalg.norm(velocity[:3]), np.linalg.norm(meat_velocity[:3])) > 0.02):
            raise RuntimeError(f'Invalid intact reset: bun={bun}, sausage={sausage}, depth={self.read_depth(base)}')

    def evaluate(self, base):
        from yam_arena.runtime import tcp_world

        if self.sample_error:
            raise RuntimeError(f'Native puncture observer failed: {self.sample_error}')
        bun, rotation, bun_velocity = self.pose(base, 'bun', 'shell')
        meat, meat_rotation, meat_velocity = self.pose(base, 'sausage')
        axis = rotation[:, 2]
        meat_axis = meat_rotation[:, 2]
        # Either end of the sausage can enter the roll.
        if meat_axis @ axis < 0.:
            meat_axis = -meat_axis
        depth = self.read_depth(base)
        local_tip = rotation.T @ (np.asarray(SPIKE_TIP) - bun)
        if (self.qualified >= HOLE_DEPTH and depth >= HOLE_DEPTH
                and (local_tip[2] > BUN_LENGTH / 2 + 0.010 or np.linalg.norm(local_tip[:2]) > 0.035)):
            self.pierced = True

        # The physical holder has a bottom stop and two supports. Pose and release
        # together distinguish a seated bun from one hovered over the holder.
        held_position = (np.linalg.norm(bun - BUN_FINAL) <= 0.012
                         and float(axis @ CRADLE_AXIS) >= 0.985)
        bun_stable = np.linalg.norm(bun_velocity[:3]) < 0.023 and np.linalg.norm(bun_velocity[3:]) < 0.20
        tcps = [np.asarray(tcp_world(base.scene[f'{side}_robot'])) for side in ('left', 'right')]
        bun_free = min(distance_to_axis(p, bun, axis, BUN_LENGTH / 2) for p in tcps) > 0.048
        step = base.sim.get_physics_step_count()
        if step != self.last_step:
            self.seat_steps = self.seat_steps + 1 if (self.pierced and held_position and bun_stable and bun_free) else 0
            self.last_step = step

        tip = meat - meat_axis * (SAUSAGE_LENGTH / 2)
        tail = meat + meat_axis * (SAUSAGE_LENGTH / 2)
        axial_tip = float((tip - bun) @ axis)
        axial_tail = float((tail - bun) @ axis)
        mouth = BUN_LENGTH / 2
        tip_radius = np.linalg.norm(tip - bun - axial_tip * axis)
        aperture_overlap = (float(meat_axis @ axis) > 0.98 and tip_radius < 0.013
                            and axial_tail >= mouth + 0.020 and axial_tip <= mouth)
        insertion = mouth - axial_tip if aperture_overlap else 0.
        if insertion > 0.002 and not (self.seated and held_position):
            self.out_of_order = True
        if self.seat_steps >= 7 and not self.out_of_order:
            self.seated = True
        meat_free = min(distance_to_axis(p, meat, meat_axis, SAUSAGE_LENGTH / 2) for p in tcps) > 0.045
        meat_stable = np.linalg.norm(meat_velocity[:3]) < 0.025 and np.linalg.norm(meat_velocity[3:]) < 0.25
        formed = self.pierced and depth >= HOLE_DEPTH
        placed = formed and self.seated and held_position and bun_stable
        success = (placed and not self.out_of_order and INSERTION <= insertion <= min(depth, self.qualified) + 0.003
                   and bun_free and meat_free and meat_stable)
        score = 0.
        if not self.out_of_order:
            score = 1. if success else 0.5 if placed and bun_free else 0.25 if formed else 0.
        return dict(success=bool(success), score=score,
                    cavity_mm=1000 * depth, qualified_mm=1000 * self.qualified,
                    spike_axial_peak_n=self.peak_force, loaded_physics_steps=self.loaded_steps,
                    hole_formed=self.pierced, stand_seated=self.seated,
                    insertion_mm=1000 * insertion, out_of_order=self.out_of_order)

    def scripted_actions(self, base, policy, max_steps):
        if policy == 'hold':
            return ((self.initial_action(), 'inspect intact reset', {}) for _ in range(min(40, max_steps)))
        from .controller import scripted_actions
        return scripted_actions(self, base, policy, max_steps)

    def close(self):
        self.subscription = None
        self.contacts = None
        if self.base is not None and self.saved_contact_setting is not None:
            self.base.sim.set_setting('/physics/disableContactProcessing', self.saved_contact_setting)
            self.saved_contact_setting = None


def make_task(condition):
    return FrenchHotDog()
