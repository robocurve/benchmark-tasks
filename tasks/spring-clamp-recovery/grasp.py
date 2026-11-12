"""Scorer-only native contact evidence; never a policy observation or actuator."""
import json
from pathlib import Path

import numpy as np


def bilateral_grasp(rows, fingertips):
    """Require opposing loaded contacts on both exact LEFT tips and no other support.

    Rows are fresh native normal-contact records, not geometric proximity. Unknown
    actors, speculative support contacts and missing positive contacts fail closed.
    This predicate does not calibrate contact forces or establish native delivery.
    """
    if len(fingertips) != 2 or len(set(fingertips)) != 2:
        raise ValueError('Two distinct fingertip bodies are required')
    normals = {path: [] for path in fingertips}
    for row in rows:
        values = np.asarray([row['impulse_Ns'], row['separation_m'], *row['normal']], dtype=float)
        if values.shape != (5,) or not np.isfinite(values).all():
            raise ValueError('Invalid native contact record')
        path = row['other_path']
        if path not in normals:
            return False
        normal = values[2:]
        length = np.linalg.norm(normal)
        if not .99 <= length <= 1.01:
            raise ValueError('Invalid contact normal')
        # Native raw impulses are signed in the contact frame.
        if abs(values[0]) > 1e-9 and values[1] <= 0:
            normals[path].append(normal / length)
    a, b = (normals[path] for path in fingertips)
    return bool(a and b and any(np.dot(x, y) <= -.5 for x in a for y in b))


class NativeLeftGrasp:
    """Read every physics frame; reject missing frames, buffer saturation and errors."""
    def __init__(self, base, evidence=None):
        import omni.usd
        from omni.physx import get_physx_interface
        from pxr import PhysxSchema, Usd, UsdPhysics
        from yam_arena.robot import asset_link_paths

        self.base = base
        self.file = None
        self.subscription = None
        self.error = None
        self.frames = 0
        self.good = True
        self.step = base.sim.get_physics_step_count()
        self.dt = float(base.sim.get_physics_dt())
        stage = omni.usd.get_context().get_stage()
        bodies = [prim for prim in stage.Traverse(Usd.TraverseInstanceProxies())
                  if prim.HasAPI(UsdPhysics.RigidBodyAPI)
                  and str(prim.GetPath()).endswith('/tool/tool')]
        if len(bodies) != 1:
            raise RuntimeError('Cannot identify the unique native screwdriver body')
        tool = bodies[0]
        path = str(tool.GetPath())
        namespace = path[:-len('/tool/tool')]
        links = asset_link_paths()
        self.tips = tuple(namespace + '/LeftArm' + links[name] for name in ('tip_left', 'tip_right'))
        for tip in self.tips:
            if not stage.GetPrimAtPath(tip).HasAPI(UsdPhysics.RigidBodyAPI):
                raise RuntimeError(f'Missing exact left fingertip: {tip}')
        if (not tool.HasAPI(PhysxSchema.PhysxContactReportAPI)
                or PhysxSchema.PhysxContactReportAPI(tool).GetThresholdAttr().Get() != 0):
            raise RuntimeError('Screwdriver contact reporting is not active at threshold zero')
        sim_view = base.sim.physics_manager.get_physics_sim_view()
        self.view = sim_view.create_rigid_contact_view([path], filter_patterns=[], max_contact_data_count=4096)
        if list(self.view.sensor_paths) != [path]:
            raise RuntimeError('Native grasp contact view resolved the wrong body')
        if evidence is not None:
            self.file = (Path(evidence) / 'native-left-grasp.jsonl').open('w')
            self.file.write(json.dumps(dict(event='binding', tool=path, left_fingertips=self.tips,
                                           physics_dt_s=self.dt)) + '\n')
        self.subscription = get_physx_interface().subscribe_physics_on_step_events(
            self.sample, pre_step=False, order=0)

    def sample(self, dt):
        try:
            import warp as wp
            step = self.base.sim.get_physics_step_count()
            if step != self.step + 1 or abs(float(dt) - self.dt) > 1e-9:
                raise RuntimeError('Missing or inconsistent native grasp frame')
            self.step = step
            data = [np.array(value.numpy(), copy=True) for value in self.view.get_raw_contact_data(dt=1.)]
            if len(data) != 7:
                raise RuntimeError('Native grasp requires seven raw contact buffers')
            impulse, point, normal, separation, count, start, actor_ids = data
            capacity = len(impulse)
            if (capacity <= 0 or impulse.shape != (capacity, 1) or point.shape != (capacity, 3)
                    or normal.shape != (capacity, 3) or separation.shape != (capacity, 1)
                    or actor_ids.shape != (capacity,) or count.shape != (1,) or start.shape != (1,)
                    or any(a.dtype.kind not in 'iu' for a in (count, start, actor_ids))):
                raise RuntimeError('Unexpected native grasp contact buffer layout')
            n, offset = int(count[0]), int(start[0])
            if n < 0 or n >= capacity or (n and (offset < 0 or offset + n > capacity)):
                raise RuntimeError('Native grasp contact buffer invalid or saturated')
            indices = range(offset, offset + n)
            ids = sorted({int(actor_ids[i]) for i in indices})
            paths = self.view.get_other_actor_paths_from_ids(wp.array(ids, dtype=wp.uint64, device='cpu')) if ids else []
            if len(paths) != len(ids) or any(not str(path).startswith('/') for path in paths):
                raise RuntimeError('Unresolved native grasp actor identity')
            resolved = dict(zip(ids, map(str, paths)))
            rows = []
            for i in indices:
                if not np.isfinite(point[i]).all():
                    raise RuntimeError('Invalid native grasp contact point')
                rows.append(dict(other_path=resolved[int(actor_ids[i])], impulse_Ns=float(impulse[i, 0]),
                                 separation_m=float(separation[i, 0]), normal=normal[i].tolist(),
                                 point=point[i].tolist()))
            qualified = bilateral_grasp(rows, self.tips)
            self.frames += 1
            self.good &= qualified
            if self.file:
                self.file.write(json.dumps(dict(physics_step=step, rows=rows, qualified=qualified)) + '\n')
                self.file.flush()
        except Exception as error:
            self.error = str(error)
            self.good = False

    def consume(self):
        if self.error is not None:
            raise RuntimeError('Native grasp observer failed: ' + self.error)
        if self.base.sim.get_physics_step_count() != self.step:
            raise RuntimeError('Native grasp callback missed the latest physics frame')
        if self.frames == 0:
            return False  # Reset-time inspection cannot establish a held grasp.
        expected = round(float(self.base.step_dt) / self.dt)
        if expected <= 0 or self.frames != expected:
            raise RuntimeError('Incomplete native grasp control interval')
        qualified = self.good
        self.frames, self.good = 0, True
        return qualified

    def close(self):
        self.subscription = None
        if self.file:
            self.file.close()
            self.file = None
