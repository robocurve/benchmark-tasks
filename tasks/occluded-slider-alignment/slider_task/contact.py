"""Privileged PhysX contact history. Does not add any policy sensor channel."""

import math
from .layout import PROTECTED


def robot_side(path):
    for side in ('LeftArm', 'RightArm'):
        if '/' + side + '/' in path:
            return side
    return None


def protected_pair(a, b):
    sa, sb = robot_side(a), robot_side(b)
    if sa and sb and sa != sb:
        return True
    for arm, other in ((sa, b), (sb, a)):
        if arm and any('/' + name + '/' in other or other.endswith('/' + name) for name in PROTECTED):
            return True
    return False


class ContactHistory:
    """Consumes per-physics-step contact reports (240 Hz); latches all safety maxima."""

    def __init__(self):
        self.subscription = None
        self.instrumented = False
        self.reset()

    def reset(self):
        self.durations = {}
        self.max_duration = 0.0
        self.max_penetration = 0.0
        self.peak_tab_force = 0.0
        self.events = []
        self.error = None
        self.active_slider_pairs = set()
        self.robot_slider_reports = 0
        self.callback_count = 0
        self.control_step = -1

    def lost(self, a, b):
        key = tuple(sorted((a, b)))
        self.durations[key] = 0.0
        self.active_slider_pairs.discard(key)
        self.events.append(dict(a=a, b=b, event='lost', control_step=self.control_step))

    def record(self, a, b, impulses, separations, dt=1 / 240):
        impulses = [tuple(float(v) for v in impulse) for impulse in impulses]
        separations = [float(s) for s in separations]
        values = [dt, *separations, *(v for impulse in impulses for v in impulse)]
        if (not all(math.isfinite(v) for v in values) or dt <= 0
                or any(len(impulse) != 3 for impulse in impulses)):
            self.error = 'Invalid contact data (nonfinite value, impulse shape or timestep)'
            self.instrumented = False
            raise ValueError(self.error)
        key = tuple(sorted((a, b)))
        force = sum(math.sqrt(sum(float(v) ** 2 for v in impulse)) for impulse in impulses) / dt
        penetration = max([0.0, *[-float(s) for s in separations]])
        forbidden = protected_pair(a, b)
        if forbidden:
            self.durations[key] = self.durations.get(key, 0.0) + dt
            self.max_duration = max(self.max_duration, self.durations[key])
            self.max_penetration = max(self.max_penetration, penetration)
        # Only fingers may contact the movable interface. Body/other links are unsafe.
        slider_pair = ('/slider/' in a and robot_side(b)) or ('/slider/' in b and robot_side(a))
        if slider_pair:
            self.active_slider_pairs.add(key)
            self.robot_slider_reports += 1
            arm_path = b if '/slider/' in a else a
            slider_path = a if '/slider/' in a else b
            finger = any(n in arm_path for n in ('tip_left', 'tip_right', 'finger'))
            if not finger or '/push_tab' not in slider_path:
                forbidden = True
                self.max_penetration = max(self.max_penetration, penetration)
                self.durations[key] = self.durations.get(key, 0.0) + dt
                self.max_duration = max(self.max_duration, self.durations[key])
            self.peak_tab_force = max(self.peak_tab_force, force)
        self.events.append(dict(a=a, b=b, normal_force_n=force, penetration_m=penetration,
                                prohibited=bool(forbidden), physics_dt=dt,
                                event='contact', control_step=self.control_step))

    def subscribe(self):
        from omni.physx import get_physx_simulation_interface
        from omni.physx.bindings._physx import ContactEventType
        from pxr import PhysicsSchemaTools

        def callback(headers, data):
            try:
                self.callback_count += 1
                for h in headers:
                    a = str(PhysicsSchemaTools.intToSdfPath(h.collider0))
                    b = str(PhysicsSchemaTools.intToSdfPath(h.collider1))
                    if h.type == ContactEventType.CONTACT_LOST:
                        self.lost(a, b)
                        continue
                    points = data[h.contact_data_offset:h.contact_data_offset + h.num_contact_data]
                    if points:
                        self.record(a, b, [p.impulse for p in points], [p.separation for p in points])
            except Exception as exc:
                self.error = repr(exc)
                self.instrumented = False

        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(callback)
        self.instrumented = True

    def bind_tensor_monitor(self, stage, base):
        """Read the official GPU contact tensors after every original physics step.

        The pinned direct-GPU step path does not emit the legacy Python reports.
        This uses the same tensor API as Isaac Lab's ContactSensor, privately.
        """
        import numpy as np
        from pxr import Usd, UsdPhysics
        from isaaclab.sim.utils.stage import get_current_stage_id
        import omni.physics.tensors as tensors
        from isaaclab_physx.physics.physx_manager import PhysxManager
        from . import layout as L

        root = '/World/envs/env_0/'
        sensors = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(root[:-1]))
                   if robot_side(str(p.GetPath())) and 'PhysxContactReportAPI' in p.GetAppliedSchemas()]
        fixed_colliders = []
        for obj in L.FIXED:
            matches = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(root+obj.name))
                       if p.HasAPI(UsdPhysics.CollisionAPI)]
            if not matches:
                raise RuntimeError(f'Protected fixture has no collider: {obj.name}')
            fixed_colliders.extend(matches)
        # A fixed-object root is an Xform, not a rigid actor. The tensor filter
        # must name its actual static collider, otherwise PhysX drops that pair.
        filters = fixed_colliders + [root+'slider'] + sensors
        self.tensor_sim = tensors.create_simulation_view('torch', stage_id=get_current_stage_id())
        self.tensor_sim.set_subspace_roots('/')
        self.tensor_slider_view = self.tensor_sim.create_rigid_body_view(root+'slider')
        view = self.tensor_sim.create_rigid_contact_view(sensors, [filters]*len(sensors),
                                                       max_contact_data_count=8192)
        self.tensor_view = view
        if view.sensor_count != len(sensors) or not view.filter_count:
            raise RuntimeError('Contact tensor view does not cover both complete robot arms')
        sensor_paths = list(view.sensor_paths)
        filter_paths = list(view.filter_paths)
        self.tensor_contract = dict(sensor_count=view.sensor_count, filter_count=view.filter_count,
            sensor_paths=sensor_paths, filter_paths=filter_paths, capacity=view.max_contact_data_count,
            physics_hz=240, backend='PhysX RigidContactView torch frontend on original GPU simulation')
        original_step = PhysxManager.step

        def sample():
            self.callback_count += 1
            forces, points, normals, separations, counts, starts = [
                v.detach().cpu().numpy() for v in view.get_contact_data(dt=1/240)]
            if int(counts.sum()) >= view.max_contact_data_count:
                raise RuntimeError('Contact tensor capacity reached; safety evidence is incomplete')
            current = set()
            seen = set()
            slider_pose = self.tensor_slider_view.get_transforms()[0].detach().cpu().numpy()
            from yam_arena.urdf import quat_xyzw_to_matrix
            rotation = quat_xyzw_to_matrix(slider_pose[3:7])
            for i,j in np.argwhere(counts > 0):
                a = sensor_paths[i]
                b = filter_paths[i][j] if isinstance(filter_paths[i], (list,tuple)) else filter_paths[j]
                if robot_side(a) and robot_side(a) == robot_side(b):
                    continue  # preserve original disabled intra-arm self collision
                key = tuple(sorted((a,b)))
                if key in seen:
                    continue
                seen.add(key)
                start, count = int(starts[i,j]), int(counts[i,j])
                sl = slice(start, start+count)
                if not all(np.isfinite(v[sl]).all() for v in (forces, points, normals, separations)):
                    raise ValueError('Nonfinite contact tensor data')
                if b.endswith('/slider'):
                    # A compound rigid-body filter aggregates its shapes. Classify
                    # real contact points in the slider frame, conservatively.
                    local = (points[sl] - slider_pose[:3]) @ rotation
                    tab = next(c for c in L.CHILDREN if c[0] == 'push_tab')
                    inside = np.all(np.abs(local - tab[1]) <= np.asarray(tab[2])/2 + .002, axis=1)
                    b += '/push_tab' if inside.all() else '/body'
                impulses = normals[sl] * forces[sl].reshape(-1,1) / 240
                self.record(a, b, impulses, separations[sl].reshape(-1))
                current.add(tuple(sorted((a,b))))
            # Every tensor sample is a full pair matrix, so absent means lost.
            for a,b in (set(self.durations) | self.active_slider_pairs) - current:
                if self.durations.get((a,b),0) or (a,b) in self.active_slider_pairs:
                    self.lost(a,b)

        def step(cls):
            original_step()
            try:
                sample()
            except Exception as exc:
                self.error = repr(exc)
                self.instrumented = False
                raise

        PhysxManager.step = classmethod(step)
        self.instrumented = True
