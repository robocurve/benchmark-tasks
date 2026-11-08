"""In-place PhysX timestep switch with native callback audit.

The only simulation configuration writes are dt, decimation and render_interval.
There are no state writers, scene edits, resets, clock changes or hidden steps.
Validated bindings: IsaacLab 17.0.2 / PhysX 6.0.0, source revision 91a0906.
"""
from pathlib import Path
import hashlib
import inspect
import json
import subprocess
import sys

import numpy as np


def _host(value):
    if hasattr(value, 'torch'):
        value = value.torch
    if hasattr(value, 'detach'):
        value = value.detach()
    if hasattr(value, 'cpu'):
        value = value.cpu()
    if hasattr(value, 'numpy'):
        value = value.numpy()
    return np.array(value, copy=True)


def _bindings():
    # Installed PhysxManager.register_callback explicitly accepts IsaacEvents;
    # generic PhysicsEvent has no per-step event. The returned handle provides
    # deregister(). POST_PHYSICS_STEP invokes the callback with float dt.
    from isaaclab_physx.physics.physx_manager import PhysxManager, IsaacEvents
    from isaaclab.sensors.camera.camera import Camera
    from isaaclab.utils.string import ResolvableString
    return PhysxManager, IsaacEvents.POST_PHYSICS_STEP, Camera, ResolvableString


def _verified_manager(raw, expected, proxy_type):
    """Resolve only the inspected class, exact instance, or exact lazy proxy.

    PhysxCfg.class_type is a ResolvableString in IsaacLab 17.0.2. SimulationContext
    stores it unchanged; __getattr__ delegates to the resolved class. Validate
    both its reference and bound classmethod identities without calling the
    proxy's private _resolve method or accepting arbitrary duck-typed backends.
    """
    reference = expected.__module__ + ':' + expected.__qualname__
    if raw is expected:
        kind = 'class'
    elif type(raw) is expected:
        kind = 'instance'
    elif type(raw) is proxy_type and str(raw) == reference:
        kind = 'ResolvableString'
    else:
        raise ValueError('Physics manager is not the exact inspected PhysxManager or its known lazy reference')
    for name in ('step', 'register_callback', 'set_decimation', 'handles_decimation'):
        actual, wanted = getattr(raw, name), getattr(expected, name)
        if getattr(actual, '__self__', None) is not expected or getattr(actual, '__func__', None) is not getattr(wanted, '__func__', None):
            raise ValueError('Physics manager delegates to an unexpected implementation: ' + name)
    return expected, {'representation': kind, 'reference': reference,
                      'raw_type': type(raw).__module__ + '.' + type(raw).__qualname__,
                      'callback_owner': reference}


def _source_provenance():
    result = {}
    for name in ('isaaclab', 'isaaclab_physx.physics.physx_manager',
                 'isaaclab.envs.manager_based_env', 'isaaclab.sim.simulation_context',
                 'isaaclab.sensors.camera.camera', 'isaaclab.sensors.sensor_base',
                 'isaaclab.renderers.render_context'):
        module = sys.modules.get(name)
        path = Path(inspect.getfile(module)).resolve() if module is not None else None
        entry = {'path': str(path) if path else None}
        if path:
            entry['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
            for parent in path.parents:
                if (parent / '.git').exists():
                    entry['git_root'] = str(parent)
                    entry['git_commit'] = subprocess.check_output(
                        ['git', '-C', str(parent), 'rev-parse', 'HEAD'], text=True).strip()
                    break
        result[name] = entry
    return result


def _state_digest(base):
    """Read rope poses/velocities and every robot's root/joint state."""
    fields = {}
    data = base.scene['cord'].data
    for key in ('body_link_pos_w', 'body_link_quat_w', 'body_link_lin_vel_w', 'body_link_ang_vel_w'):
        fields['cord.' + key] = _host(getattr(data, key))
    for name, robot in base.scene.articulations.items():
        for key in ('joint_pos', 'joint_vel', 'root_link_pose_w', 'root_link_vel_w'):
            fields[name + '.' + key] = _host(getattr(robot.data, key))
    digest = hashlib.sha256()
    for name, value in sorted(fields.items()):
        if not np.isfinite(value).all():
            raise ValueError('Nonfinite state before timestep switch: ' + name)
        digest.update(name.encode())
        digest.update(str((value.shape, value.dtype)).encode())
        digest.update(value.tobytes())
    return digest.hexdigest()


class AssayPhysicsSwitch:
    """Own one switch and audit every subsequent 20 Hz capture interval."""

    def __init__(self, base, target_hz, capture_time_s, *, diagnostic_only=False):
        self.base, self.target_hz = base, int(target_hz)
        self.last_time = float(capture_time_s)
        self.handle = None
        self.pending = None
        self.callback_dts = []
        self.callback_error = None
        self._closed = False
        self.report = {'status': 'not_started', 'diagnostic_only': bool(diagnostic_only),
                       'target_physics_hz': self.target_hz, 'control_hz': 20,
                       'switch_capture_time_s': self.last_time, 'intervals': [],
                       'state_writes': False, 'counter_writes': False,
                       'nominal_usd_rate': 'Not changed; native callback dt is the measured integration rate.'}

    def _counters(self):
        return {'native': int(self.base.sim.get_physics_step_count()),
                'environment': int(self.base._sim_step_counter)}

    def _cameras(self):
        return {name: {'frame': _host(camera.frame), 'timestamp': _host(camera._timestamp),
                       'last_update': _host(camera._timestamp_last_update)}
                for name, camera in self.base.scene.sensors.items()}

    def _callback(self, dt):
        # Do not rely on exceptions propagating through a native subscription.
        try:
            value = float(dt)
            if self.pending is None or not np.isfinite(value):
                raise ValueError('Physics callback outside an audited interval or nonfinite dt')
            self.callback_dts.append(value)
        except Exception as exc:
            self.callback_error = f'{type(exc).__name__}: {exc}'

    def start(self):
        """Apply config changes only after all preconditions pass; never step."""
        base, sim = self.base, self.base.sim
        expected_manager, event, camera_type, proxy_type = _bindings()
        old = (sim.cfg.dt, base.cfg.decimation, sim.cfg.render_interval)
        try:
            self.report['source'] = _source_provenance()
            self.report['physics_manager_raw_type'] = type(sim.physics_manager).__module__ + '.' + type(sim.physics_manager).__qualname__
            if base.cfg.sim is not sim.cfg:
                raise ValueError('Environment and simulation must share the same config object')
            manager, identity = _verified_manager(sim.physics_manager, expected_manager, proxy_type)
            self.report['physics_manager'] = identity
            if base._physics_handles_decimation or manager.handles_decimation():
                raise ValueError('Backend-managed decimation is unsupported')
            if self.target_hz not in (480, 960) or not np.isclose(old[0], 1/240, rtol=0, atol=1e-12):
                raise ValueError('Diagnostic switch must start at 240 Hz and end at 480 or 960 Hz')
            if old[1:] != (12, 12) or not np.isclose(base.step_dt, .05, rtol=0, atol=1e-12):
                raise ValueError('Expected 20 Hz control/rendering with 12 initial substeps')
            if not np.isclose(sim.get_rendering_dt(), .05, rtol=0, atol=1e-12):
                raise ValueError('Rendering clock must already be 20 Hz')
            sensors = base.scene.sensors
            if not sensors or 'review_cam' not in sensors or any(not isinstance(s, camera_type) for s in sensors.values()):
                raise ValueError('Camera-only scene required; contact and other cached-dt sensors are unsupported')
            cameras = self._cameras()
            if any(c['timestamp'].dtype != np.float32 or not np.isfinite(c['timestamp']).all() for c in cameras.values()):
                raise ValueError('Expected finite float32 camera timestamps from inspected SensorBase')
            state_before, counters_before = _state_digest(base), self._counters()
            self.report.update(old_physics_hz=1/old[0],
                               counters_before=counters_before, state_sha256_before=state_before)
            self.handle = manager.register_callback(self._callback, event, order=0,
                                                   name='rope_assay_timestep_audit', wrap_weak_ref=False)
            self.substeps = self.target_hz // 20
            sim.cfg.dt = 1 / self.target_hz
            base.cfg.decimation = self.substeps
            sim.cfg.render_interval = self.substeps
            manager.set_decimation(self.substeps)
            if not np.isclose(sim.get_physics_dt(), 1/self.target_hz, rtol=0, atol=1e-12) or not np.isclose(base.step_dt, .05, rtol=0, atol=1e-12):
                raise ValueError('Runtime timestep/control getters disagree after switch')
            state_after, counters_after = _state_digest(base), self._counters()
            self.report.update(counters_after=counters_after, state_sha256_after=state_after)
            if state_before != state_after or counters_before != counters_after:
                raise ValueError('State or counters changed while configuring the timestep')
            self.report.update(status='awaiting_native_validation', requested_substeps=self.substeps,
                               native_physics_dt_s=float(sim.get_physics_dt()))
        except Exception as exc:
            # No step has run: rollback configuration only and remove observer.
            sim.cfg.dt, base.cfg.decimation, sim.cfg.render_interval = old
            if self.handle is not None:
                self.handle.deregister()
                self.handle = None
            self.fail(exc)
            raise

    def fail(self, exc):
        self.report['status'] = 'failed'
        self.report['error'] = f'{type(exc).__name__}: {exc}'

    def before_step(self, capture_time_s, recorded_frames):
        if self.handle is None or self.pending is not None or self.report['status'] == 'failed':
            raise RuntimeError('Timestep observer is not ready for a new interval')
        if abs(float(capture_time_s) - self.last_time) > 1e-9:
            raise ValueError('Capture time changed outside the audited stepping interval')
        self.callback_dts = []
        self.pending = {'counters': self._counters(), 'cameras': self._cameras(),
                        'frames': int(recorded_frames), 'time': float(capture_time_s)}

    def after_step(self, capture_time_s, recorded_frames):
        before = self.pending
        if before is None:
            raise RuntimeError('No pending timestep audit')
        entry = {'capture_time_s': float(capture_time_s), 'native_substeps': len(self.callback_dts),
                 'native_callback_dt_s': list(self.callback_dts), 'cameras': {}}
        self.report['intervals'].append(entry)
        try:
            if self.callback_error:
                raise RuntimeError(self.callback_error)
            dts = np.asarray(self.callback_dts)
            if len(dts) != self.substeps or not np.allclose(dts, 1/self.target_hz, rtol=0, atol=1e-9):
                raise ValueError('Native callback count or dt does not match requested integration')
            if abs(float(capture_time_s) - before['time'] - .05) > 1e-9:
                raise ValueError('Capture timestamps must advance by exactly 0.05 seconds')
            counters = self._counters()
            entry['counter_deltas'] = {k: counters[k]-before['counters'][k] for k in counters}
            if any(delta != self.substeps for delta in entry['counter_deltas'].values()):
                raise ValueError('Simulation/environment counters disagree with native callbacks')
            if int(recorded_frames) - before['frames'] != 1:
                raise ValueError('Exactly one review video frame is required per control interval')
            for name, camera in self._cameras().items():
                previous = before['cameras'][name]
                frame_delta = camera['frame'] - previous['frame']
                # SensorBase accumulates float32 dt at every substep. Predict
                # that rounding explicitly instead of treating it as real time.
                expected = previous['timestamp'].copy()
                for dt in self.callback_dts:
                    expected = np.float32(expected + np.float32(dt))
                tolerance = np.maximum(np.spacing(np.abs(expected))*2, 1e-7)
                entry['cameras'][name] = {
                    'frame_delta': frame_delta.tolist(), 'timestamp_s': camera['timestamp'].tolist(),
                    'timestamp_delta_s': (camera['timestamp']-previous['timestamp']).tolist(),
                    'last_update_s': camera['last_update'].tolist(),
                    'float32_predicted_timestamp_s': expected.tolist()}
                if np.any(frame_delta != 1):
                    raise ValueError('Camera did not produce exactly one new frame: ' + name)
                if np.any(np.abs(camera['timestamp']-expected) > tolerance):
                    raise ValueError('Camera timestamp disagrees with native substeps: ' + name)
                if np.any(camera['last_update'] != camera['timestamp']):
                    raise ValueError('Camera image was not refreshed at the final substep: ' + name)
            self.last_time = float(capture_time_s)
            entry['validated'] = True
            self.report['status'] = 'validated_intervals'
        except Exception as exc:
            self.fail(exc)
            raise
        finally:
            self.pending = None

    def close(self):
        if self._closed:
            return
        if self.handle is not None:
            self.handle.deregister()
            self.handle = None
        self.report['callback_removed'] = True
        self.report['validated_control_intervals'] = sum(bool(row.get('validated')) for row in self.report['intervals'])
        if self.pending is not None:
            self.report['status'] = 'failed'
            self.report.setdefault('error', 'Incomplete native interval at shutdown')
        if self.callback_error:
            self.fail(RuntimeError(self.callback_error))
        # A callback after the final after_step() has no next interval to check
        # it. Also detect counter advances that did not deliver a callback.
        closure = {'schema': 'native-counter-closure-v1', 'complete': False,
                   'callback_error': self.callback_error}
        self.report['closure'] = closure
        try:
            before = self.report['counters_after']
            steps = self.report['validated_control_intervals'] * self.substeps
            expected = {key: before[key] + steps for key in ('native', 'environment')}
            observed = self._counters()
            closure.update(expected_counters=expected, observed_counters=observed)
            if observed != expected:
                raise ValueError('Native counters advanced outside completed audited intervals')
            closure['complete'] = self.pending is None and self.callback_error is None
        except (KeyError, TypeError, ValueError, AttributeError) as exc:
            if self.report.get('status') != 'failed':
                self.fail(exc)
        self._closed = True

    def save(self, path):
        Path(path).write_text(json.dumps(self.report, indent=2, allow_nan=False)+'\n')
