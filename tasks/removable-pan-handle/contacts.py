"""Read-only native contact evidence. No attachment logic or physics changes."""
import json
from pathlib import Path

import numpy as np


class NativeContacts:
    """Record every physics frame, including counterpart identity and body pose."""

    def __init__(self, base, directory=None):
        import omni.usd
        from omni.physx import get_physx_interface
        from pxr import Usd, UsdPhysics, PhysxSchema
        from yam_arena.robot import asset_link_paths

        self.base = base
        self.stage = omni.usd.get_context().get_stage()
        self.dt = float(base.sim.get_physics_dt())
        self.step = base.sim.get_physics_step_count()
        self.frames, self.error, self.cache = [], None, {}
        self.file = (Path(directory) / 'native-pan-contacts.jsonl').open('w') if directory else None
        suffixes = {'pan': '/pan/pan', 'housing': '/handle/housing',
                    'jaw': '/handle/jaw', 'lever': '/handle/lever',
                    'button_short': '/handle/button_short', 'button_long': '/handle/button_long'}
        bodies = [p for p in self.stage.Traverse(Usd.TraverseInstanceProxies())
                  if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        self.paths = {}
        for name, suffix in suffixes.items():
            matches = [p for p in bodies if str(p.GetPath()).endswith(suffix)]
            if len(matches) != 1:
                raise RuntimeError(f'Cannot resolve unique native {name} body')
            prim = matches[0]
            if (not prim.HasAPI(PhysxSchema.PhysxContactReportAPI)
                    or PhysxSchema.PhysxContactReportAPI(prim).GetThresholdAttr().Get() != 0):
                raise RuntimeError(f'Native contact reporting missing for {name}')
            self.paths[name] = str(prim.GetPath())
        namespace = self.paths['pan'][:-len('/pan/pan')]
        self.supports = dict(table=namespace + '/table', trivet=namespace + '/fixture/trivet')
        if any(not self.stage.GetPrimAtPath(path).IsValid() for path in self.supports.values()):
            raise RuntimeError('Registered table or trivet is missing')
        links = asset_link_paths()
        self.tips = {side: [namespace + '/' + arm + links[n] for n in ('tip_left', 'tip_right')]
                     for side, arm in [('left', 'LeftArm'), ('right', 'RightArm')]}
        for pair in self.tips.values():
            for path in pair:
                if not self.stage.GetPrimAtPath(path).HasAPI(UsdPhysics.RigidBodyAPI):
                    raise RuntimeError('Native fingertip identity is missing')
        self.robot_prefixes = (namespace + '/LeftArm/', namespace + '/RightArm/')
        sim = base.sim.physics_manager.get_physics_sim_view()
        self.view = sim.create_rigid_contact_view(list(self.paths.values()),
                                                filter_patterns=[], max_contact_data_count=8192)
        self.sensors = list(self.view.sensor_paths)
        if set(self.sensors) != set(self.paths.values()) or len(self.sensors) != len(self.paths):
            raise RuntimeError('Native contact view did not bind all requested bodies')
        self.names = {path: name for name, path in self.paths.items()}
        self.body_views = {name: base.scene['pan_pan' if name == 'pan' else 'handle_' + name].root_view
                           for name in self.paths}
        if self.file:
            self.file.write(json.dumps(dict(event='binding', bodies=self.paths,
                                           fingertips=self.tips, supports=self.supports,
                                           physics_dt_s=self.dt,
                                           first_physics_step=self.step+1)) + '\n')
        self.subscription = get_physx_interface().subscribe_physics_on_step_events(
            self.sample, pre_step=False, order=0)

    def sample(self, dt):
        if self.error:
            return
        try:
            import warp as wp
            step = self.base.sim.get_physics_step_count()
            if step != self.step + 1 or abs(float(dt) - self.dt) > 1e-9:
                raise RuntimeError('Missing or inconsistent native contact frame')
            self.step = step
            data = [np.array(a.numpy(), copy=True) for a in self.view.get_raw_contact_data(dt=1.)]
            if len(data) != 7:
                raise RuntimeError('Expected seven native contact buffers')
            impulse, point, normal, separation, count, start, ids = data
            capacity = len(impulse)
            if (capacity <= 0 or impulse.shape != (capacity, 1) or point.shape != (capacity, 3)
                    or normal.shape != point.shape or separation.shape != impulse.shape
                    or ids.shape != (capacity,) or count.shape != (len(self.sensors),)
                    or start.shape != count.shape
                    or any(a.dtype.kind not in 'iu' for a in (ids, count, start))):
                raise RuntimeError('Unexpected native contact buffer layout')
            used, active = set(), []
            for sensor, n, first in zip(self.sensors, count, start):
                n, first = int(n), int(first)
                if n < 0 or (n and (first < 0 or first + n > capacity)):
                    raise RuntimeError('Native contact range is invalid')
                indices = set(range(first, first + n)) if n else set()
                if used & indices:
                    raise RuntimeError('Native contact ranges overlap')
                used |= indices
                active.extend((sensor, i) for i in sorted(indices))
            if len(used) >= capacity:
                raise RuntimeError('Native contact buffer is saturated')
            unknown = sorted({int(ids[i]) for _, i in active} - self.cache.keys())
            paths = self.view.get_other_actor_paths_from_ids(
                wp.array(unknown, dtype=wp.uint64, device='cpu')) if unknown else []
            if len(paths) != len(unknown):
                raise RuntimeError('Native counterpart actor is unresolved')
            for actor, path in zip(unknown, map(str, paths)):
                if not path.startswith('/') or not self.stage.GetPrimAtPath(path).IsValid():
                    raise RuntimeError('Native contact has an unknown actor')
                self.cache[actor] = path
            rows = []
            for sensor, i in active:
                if not np.isfinite(np.r_[impulse[i], point[i], normal[i], separation[i]]).all():
                    raise RuntimeError('Nonfinite native contact')
                rows.append(dict(body=self.names[sensor], other=self.cache[int(ids[i])],
                                 impulse_Ns=float(impulse[i, 0]), point_m=point[i].tolist(),
                                 normal=normal[i].tolist(), separation_m=float(separation[i, 0])))
            poses = {}
            for name, view in self.body_views.items():
                pose = np.array(view.get_transforms().numpy(), copy=True).reshape(-1, 7)
                if pose.shape != (1, 7) or not np.isfinite(pose).all():
                    raise RuntimeError('Native body pose is invalid')
                poses[name] = pose[0].tolist()
            pan_velocity = np.array(self.body_views['pan'].get_velocities().numpy(), copy=True).reshape(-1, 6)
            if pan_velocity.shape != (1, 6) or not np.isfinite(pan_velocity).all():
                raise RuntimeError('Native pan velocity is invalid')
            frame = dict(physics_step=step, rows=rows, poses_xyzw=poses,
                         pan_velocity_w=pan_velocity[0].tolist())
            self.frames.append(frame)
            if self.file:
                self.file.write(json.dumps(frame) + '\n')
        except Exception as error:
            self.error = str(error)

    def consume(self):
        if self.error:
            raise RuntimeError('Native contact observer failed: ' + self.error)
        frames, self.frames = self.frames, []
        if frames and len(frames) != round(float(self.base.step_dt) / self.dt):
            raise RuntimeError('Incomplete native contact coverage of control interval')
        if self.file:
            self.file.flush()
        return frames

    def close(self):
        self.subscription = None
        if self.file:
            self.file.close()
            self.file = None
