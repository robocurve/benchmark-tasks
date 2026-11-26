"""Physics recorder for scoring. The scripted policy never imports this module."""

import json
import math
import os
from pathlib import Path

import numpy as np
import omni.physics.tensors as tensors
from pxr import Usd, UsdPhysics, UsdUtils

from isaaclab.managers import RecorderTerm

from geometry import CONTROL_HZ, HINGE_FRICTION, PHYSICS_HZ
from scoring import TentScore


def static_collider(prim):
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        return False
    while prim and not prim.IsPseudoRoot():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return False
        prim = prim.GetParent()
    return True


class TentRecorder(RecorderTerm):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        env.tent_recorder = self
        self.active = False

    def start(self, control):
        env = self._env
        stage = env.sim.stage
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.view = tensors.create_simulation_view('torch', stage_id=stage_id)
        prefix, = env.scene.env_prim_paths
        laptop = f'{prefix}/laptop'
        self.body_paths = [f'{laptop}/Chassis', f'{laptop}/Display']
        self.bodies = self.view.create_rigid_body_view(self.body_paths)
        self.articulation = self.view.create_articulation_view(self.body_paths[0])
        if list(map(str, self.bodies.prim_paths)) != self.body_paths or self.articulation.count != 1:
            raise RuntimeError('Unexpected laptop articulation or body order')
        prims = tuple(stage.Traverse(Usd.TraverseInstanceProxies()))
        static = [str(p.GetPath()) for p in prims if static_collider(p)]
        desk = [p for p in static if p == f'{prefix}/table' or p.startswith(f'{prefix}/table/')]
        if len(desk) != 1:
            raise RuntimeError(f'Expected one desk contact collider, found {desk}')
        robot = [str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)
                 and str(p.GetPath()).startswith((f'{prefix}/LeftArm/', f'{prefix}/RightArm/'))]
        if len(robot) != 18:
            raise RuntimeError(f'Expected eighteen robot links, found {robot}')
        self.robot_end = 1 + len(robot)
        self.filters = [*desk, *robot, *(p for p in static if p not in desk)]
        self.contact = self.view.create_rigid_contact_view(
            self.body_paths, filter_patterns=[self.filters] * 2, max_contact_data_count=512
        )
        if self.contact.sensor_count != 2 or self.contact.filter_count != len(self.filters):
            raise RuntimeError('Laptop contact view failed to initialize')
        coeffs = self.articulation.get_dof_friction_properties().cpu().numpy()[0, 0]
        if not np.allclose(coeffs, HINGE_FRICTION):
            raise RuntimeError(f'Hinge resistance differs from the configured values: {coeffs}')
        self.result = TentScore()
        self.stage_label = ''
        self.actions = 0
        self.samples = []
        trace = os.getenv('LAPTOP_TENT_TRACE')
        self.trace = Path(trace) if trace else None
        self.active = True
        if self.trace is not None:
            self.trace.parent.mkdir(parents=True, exist_ok=True)
            self.trace.write_text(json.dumps(dict(
                action=0, control=control, physics_hz=PHYSICS_HZ, control_hz=CONTROL_HZ, filters=self.filters,
                physical_gpu=os.getenv('ROBOCURVE_GPU'), friction=coeffs.tolist(), hinge_deg=self.hinge_angle(),
                poses=self.bodies.get_transforms().cpu().tolist(),
            )) + '\n', encoding='utf-8')

    def hinge_angle(self):
        return math.degrees(self.articulation.get_dof_positions()[0, 0].item()) + 180.

    def record_post_physics_decimation_step(self):
        if not self.active:
            return None, None
        self.poses = self.bodies.get_transforms().cpu().numpy()
        velocities = self.bodies.get_velocities().cpu().numpy()
        forces, points, normals, _, counts, starts = (
            a.cpu().numpy() for a in self.contact.get_contact_data(1 / PHYSICS_HZ)
        )
        if int(counts.sum()) >= self.contact.max_contact_data_count:
            raise RuntimeError('Laptop contact buffer filled')

        desk_contacts, robot_force, other_force = [], 0., 0.
        hand_loads = np.zeros((2, 2))
        for link in range(2):
            for i, (count, start) in enumerate(zip(counts[link], starts[link], strict=True)):
                region = slice(int(start), int(start) + int(count))
                if i == 0:
                    desk_contacts.append((points[region], forces[region] * normals[region]))
                elif count:
                    load = float(np.abs(forces[region]).sum())
                    if i < self.robot_end:
                        robot_force += load
                        side = int('/RightArm/' in self.filters[i])
                        hand_loads[link, side] += load
                    else:
                        other_force += load

        self.result.sample(self.poses, velocities, self.hinge_angle(), desk_contacts, robot_force, other_force)
        self.result.metrics['arm_normal_N_by_link'] = hand_loads.tolist()
        if self.trace is not None:
            self.samples.append(self.result.metrics)
        return None, None

    def record_action(self, observation):
        self.actions += 1
        if self.result.ticks != self.actions * (PHYSICS_HZ // CONTROL_HZ):
            raise RuntimeError('Missing physics samples during laptop action')
        if self.trace is not None:
            with self.trace.open('a', encoding='utf-8') as out:
                out.write(json.dumps(dict(action=self.actions, stage=self.stage_label, score=self.result.score,
                                          metrics=self.result.metrics, poses=self.poses.tolist(),
                                          joints=observation.state['joint_pos'].tolist(), samples=self.samples),
                                     separators=(',', ':')) + '\n')
            self.samples.clear()
