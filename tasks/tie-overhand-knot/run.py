"""Run one audited rope-knot trial using the guide's Inspect Robots workflow."""

from __future__ import annotations

import argparse
import atexit
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
import platform
import os
import subprocess
import sys
import time
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

import numpy as np

from scoring import KnotScorer
from config import CONFIG
import telemetry
SCORE_VERSION = "overhand-bimanual-v4"

HERE = Path(__file__).resolve().parent


def _inspect_execution(log_dir, previous_logs, cli_return_code, raised_errors=()):
    """Check Inspect's saved outcome independently of its CLI exit semantics."""
    errors = [f'{type(exc).__name__}: {exc}' for exc in raised_errors]
    logs = []
    # Older Inspect entry points return None, which SystemExit treats as zero.
    # The fresh evaluation log is still required to establish success.
    if cli_return_code is not None and (type(cli_return_code) is not int or cli_return_code != 0):
        errors.append(f'Inspect CLI returned {cli_return_code!r}, expected 0 or None')
    paths = sorted(path for path in Path(log_dir).glob('adhoc_*.json')
                   if path not in previous_logs and not path.name.endswith('.live.json'))
    if len(paths) != 1:
        errors.append(f'Expected one fresh Inspect evaluation log, found {len(paths)}')
    for path in paths:
        try:
            content = path.read_bytes()
            log = json.loads(content)
            result, samples = log['results'], log['samples']
            sample_errors = [dict(scene_id=sample.get('scene_id'), status=sample.get('status'),
                                  error=sample.get('error')) for sample in samples
                             if sample.get('status') != 'success' or sample.get('error')]
            logs.append(dict(file=path.name, sha256=hashlib.sha256(content).hexdigest(),
                             status=log['status'], error=log.get('error'),
                             errored_trials=result['errored_trials'], sample_errors=sample_errors))
            if (log['status'] != 'success' or log.get('error') or sample_errors
                    or result['errored_trials'] != 0 or result['total_trials'] != 1
                    or result['total_scenes'] != 1 or len(samples) != 1):
                details = [str(log.get('error') or log['status'])]
                details += [str(sample['error'] or sample['status']) for sample in sample_errors]
                errors.append('Inspect evaluation failed: ' + '; '.join(details))
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
            errors.append(f'Cannot validate Inspect log {path.name}: {type(exc).__name__}: {exc}')
    return dict(status='failed' if errors else 'complete', cli_return_code=cli_return_code,
                errors=errors, logs=logs)


def _record_execution_outcome(log_dir, metadata, execution):
    """Keep geometric diagnostics, but a crashed policy earns no task score."""
    metadata['execution'] = execution
    path = Path(log_dir) / 'score-summary.json'
    if not path.exists():
        if execution['status'] == 'complete':
            raise RuntimeError('Completed Inspect execution has no audited score summary')
        metadata['audited_final_score'] = 0.
        metadata['finalization_complete'] = False
        return
    summary = json.loads(path.read_text())
    summary['execution'] = execution
    if execution['status'] != 'complete':
        summary['score'] = 0.
        summary['status'] = 'Policy or evaluation execution failed'
        finalization = summary.setdefault('finalization', {})
        finalization['complete'] = False
        finalization.setdefault('errors', []).extend(execution['errors'])
        if 'finalization_certification' in summary:
            summary['finalization_certification']['complete'] = False
        metadata['audited_final_score'] = 0.
        metadata['finalization_complete'] = False
    path.write_text(json.dumps(summary, indent=2) + '\n')


def _register_dream_policy(rows_per_request, register, request_audit):
    """Use supported SDK hooks; a replaced server episode cannot continue silently."""
    import dream
    from inspect_robots_dream import DreamPolicy
    from inspect_robots_dream.session import DreamSession

    class ContinuousDreamClient(dream.DreamClient):
        def get_action(self, *, commanded=None):
            started = time.monotonic()
            previous = getattr(self, '_last_success_at', None)
            before = self.session_id
            event = dict(request=len(request_audit) + 1,
                         idle_wall_seconds=None if previous is None else started - previous)
            request_audit.append(event)
            try:
                reply = super().get_action(commanded=commanded)
                if before is not None and self.session_id != before:
                    raise RuntimeError('Dream server session changed; evaluation stopped before executing its reply')
            except dream.DreamSessionExpired as exc:
                event.update(status='session_replaced', reason=exc.reason)
                # The adapter retries DreamSessionExpired. A plain RuntimeError
                # instead reaches Inspect's normal failed-execution finalization.
                raise RuntimeError('Dream server session expired or was replaced; evaluation stopped before retry') from exc
            except Exception as exc:
                event.update(status='error', error_type=type(exc).__name__)
                raise
            else:
                self._last_success_at = time.monotonic()
                event.update(status='success', session_changed=False)
                return reply
            finally:
                event['wall_seconds'] = time.monotonic() - started

    def make_policy():
        return DreamPolicy(rows_per_request=rows_per_request,
                           session=DreamSession(make_client=ContinuousDreamClient))

    name = 'rope_dream'
    register('policy', name)(make_policy)
    return name


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", choices=("demo", "dream", "idle", "grasp", "loop", "thread", "jig"), default="demo")
    parser.add_argument("--max-steps", type=int, default=CONFIG['run_limit'])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--log-dir", type=Path, required=True)
    parser.add_argument("--headless", choices=("true", "false"), default="true")
    parser.add_argument('--dream-rows-per-request', type=int, choices=range(1, 49), default=8,
                        help='Dream action rows played before requesting a new chunk (default: 8)')
    parser.add_argument("--physics-hz", type=int, choices=(240, 480, 960), default=240,
                        help="Physics substeps; control and recorded policy observations remain 20 Hz")
    parser.add_argument("--assay-physics-hz", type=int, choices=(480, 960), default=960,
                        help="Physics rate used only after a released knot qualifies for the assessor")
    parser.add_argument("--development-jig", choices=("three-peg",),
                        help="Development fixture; requires separate task and contact validation")
    parser.add_argument("--diagnostic-layout", choices=("straight", "preformed"),
                        help="Physics diagnostic only; cannot earn a task score")
    parser.add_argument("--proof-probe", action="store_true",
                        help="Apply assessor forces to a diagnostic layout, never a robot trial")
    parser.add_argument("--skip-assay", action="store_true",
                        help="Development only: record policy outcome without the post-policy load test")
    args = parser.parse_args()
    from fixed_policy_physics import validate_schedule
    try:
        policy_physics_schedule = validate_schedule(CONFIG.get('policy_physics_schedule'), args.physics_hz)
    except ValueError as exc:
        parser.error(str(exc))
    if args.development_jig or args.policy == 'jig' or args.diagnostic_layout or args.proof_probe:
        parser.error('v3 fixture/preformed/load probes are historical: use their saved task-source snapshot; v4 has only a free, uncrossed cord')
    if args.policy in ('demo', 'loop', 'thread'):
        from table_policy import IMPLEMENTED_STAGES
        if args.policy not in IMPLEMENTED_STAGES:
            parser.error('requested controller stage is unavailable: '+args.policy)
    if args.policy in ('demo', 'dream'):
        if args.diagnostic_layout or args.proof_probe:
            parser.error('demo and dream use the ordinary task scene, not diagnostic layouts')
        if args.development_jig:
            parser.error('the task starts with an uncrossed cord and has no jig')
    if args.policy == 'jig':
        args.development_jig = 'three-peg'
    if args.development_jig and (args.diagnostic_layout or args.proof_probe):
        parser.error('development-jig cannot be combined with diagnostic-layout or proof-probe')
    if args.diagnostic_layout and args.policy != 'idle':
        parser.error('diagnostic layouts require the idle policy')
    if args.proof_probe and not args.diagnostic_layout:
        parser.error('proof-probe requires a diagnostic layout')
    # V4 has no post-policy force assay. Legacy fixtures and preformed layouts
    # remain diagnostics only and can never establish success for this task.
    CONFIG['score_version'] = SCORE_VERSION
    if args.development_jig or args.diagnostic_layout:
        CONFIG['diagnostic_only'] = True
    args.skip_assay = True
    if args.development_jig:
        import config
        import jig_layout
        nodes = jig_layout.initial_nodes()
        config.initial_nodes = lambda: nodes.copy()
        density = CONFIG['mass'] / CONFIG['length']
        length = float(np.linalg.norm(np.diff(nodes, axis=0), axis=1).sum())
        CONFIG.update(initial_nodes=nodes.tolist(), development_jig='three-peg',
                      instruction='Finish the overhand knot in the blue cord, remove it from the pegs and let go. The knot will be tested with a gentle pull.',
                      jig_geometry=jig_layout.geometry(),
                      length=length, mass=density*length, anchor_start=True)
    if args.diagnostic_layout:
        import config
        from diagnostic_layouts import diagnostic_nodes
        nodes = diagnostic_nodes(args.diagnostic_layout)
        config.initial_nodes = lambda: nodes.copy()
        CONFIG['diagnostic_only'] = True
        CONFIG['diagnostic_layout'] = args.diagnostic_layout
        CONFIG['initial_nodes'] = nodes.tolist()
        density = CONFIG['mass'] / CONFIG['length']
        CONFIG['length'] = float(np.linalg.norm(np.diff(nodes, axis=0), axis=1).sum())
        CONFIG['mass'] = density * CONFIG['length']
        CONFIG['proof_probe'] = args.proof_probe
    CONFIG['physics_hz'] = args.physics_hz
    CONFIG['assay_physics_hz'] = args.assay_physics_hz
    if not 1 <= args.max_steps <= CONFIG['run_limit']:
        parser.error(f"max-steps must be 1..{CONFIG['run_limit']} ({CONFIG['run_limit']/20:g} seconds at 20 Hz)")
    args.log_dir.mkdir(parents=True, exist_ok=True)
    if (args.log_dir / "trajectory.jsonl").exists():
        parser.error("choose a fresh log directory; existing audit data will not be overwritten")

    from inspect_robots.cli import main as inspect_main
    from inspect_robots.registry import register
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN, GRIPPER, SHOULDER, WRIST
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    instances = []

    class AuditedRopeEmbodiment(YamArenaEmbodiment):
        def __init__(self, **kwargs):
            self._close_finished = False
            self.audit = self.review_encoder = self.review_stderr = None
            self.endpoint_load = None
            self.physics_switch = None
            self.policy_physics_audit = None
            self._policy_physics_failure = None
            self._policy_reset_started = False
            self.assay_transition_required = False
            self._transition_failure = None
            instances.append(self)
            super().__init__(**kwargs)
            if not self.info.is_simulated:
                raise RuntimeError("this task runner is for simulation only")
            hz = self.info.control_hz
            per_arm = (SHOULDER["velocity"] / hz,) * 3 + (WRIST["velocity"] / hz,) * 3 + (GRIPPER["velocity"] / abs(FINGER_OPEN) / hz,)
            semantics = replace(self.info.action_space.semantics, max_step=per_arm * 2)
            self.info = replace(self.info, action_space=replace(self.info.action_space, semantics=semantics))
            metadata['action_limits'] = {
                'low': self._low.tolist(), 'high': self._high.tolist(),
                'max_step': list(self.info.action_space.semantics.max_step),
                'source': 'Audited embodiment action bounds and control-step semantics',
            }

        def _ensure_sim(self, goal):
            # The adapter imports scene construction only after Kit starts. Start
            # its shared app identically, then add this task's passive review
            # camera before construction. No installed adapter files are changed.
            from yam_arena import inspect_embodiment as adapter
            if adapter._APP is None:
                os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
                from isaaclab.app import AppLauncher
                options = {"headless": self.headless, "device": self.device, "enable_cameras": True}
                if not self.headless:
                    options["visualizer"] = ["kit"]
                    os.environ["DISPLAY"] = adapter.window_display(os.environ)
                    if os.environ["DISPLAY"] == adapter.DESKTOP_DISPLAY:
                        options["window_width"], options["window_height"] = adapter.DESKTOP_SIZE
                adapter._APP = AppLauncher(**options).app
                atexit.register(adapter._close_app_at_exit)
            from yam_arena import scene as scene_module
            from visuals import add_review_camera, install_scene_visuals
            make_env = scene_module.make_env

            def make_review_env(embodiment, **kwargs):
                add_review_camera(embodiment)
                return make_env(embodiment, **kwargs)

            scene_module.make_env = make_review_env
            original_physics_dt = scene_module.PHYSICS_DT
            scene_module.PHYSICS_DT = 1. / args.physics_hz
            try:
                super()._ensure_sim(goal)
            finally:
                scene_module.make_env = make_env
                scene_module.PHYSICS_DT = original_physics_dt
            self.visual_metadata = install_scene_visuals(self._base)
            from isaaclab.sim.utils.stage import get_current_stage
            from pxr import UsdPhysics
            physics = {'physics_dt': float(self._base.physics_dt),
                       'control_dt': float(self._base.step_dt), 'scenes': [],
                       'policy_physics_schedule': policy_physics_schedule,
                       'rope_requested': {key: CONFIG[key] for key in ('contact_offset', 'solver_position_iterations', 'solver_velocity_iterations')},
                       'rope_rest_offset_m': 0.}
            for prim in get_current_stage().Traverse():
                if prim.IsA(UsdPhysics.Scene):
                    attrs = {}
                    for attr in prim.GetAttributes():
                        if any(word in attr.GetName().lower() for word in ('ccd', 'gpu', 'solver', 'stabil', 'iteration')):
                            attrs[attr.GetName()] = str(attr.Get())
                    physics['scenes'].append({'path': str(prim.GetPath()), 'attributes': attrs})
            (args.log_dir / 'physics.json').write_text(json.dumps(physics, indent=2) + '\n')


        def reset(self, scene, *, seed=None):
            # A changed native timestep cannot be silently reused as the next
            # trial's 240 Hz prefix, nor can existing evidence be overwritten.
            if getattr(self, '_policy_reset_started', False):
                exc = RuntimeError('Use a fresh embodiment for each audited rope trial')
                self._policy_physics_failure = exc
                if self.policy_physics_audit is not None:
                    self.policy_physics_audit.fail(exc)
                raise exc
            self._policy_reset_started = True
            self.endpoint_load = None
            self.physics_switch = None
            self.assay_transition_required = False
            self._transition_failure = None
            self.proof_record = None
            self.assay_phase = None
            self.assay_attempted = False
            self.last_policy_action = None
            self.policy_steps = 0
            telemetry.latest_sample = None
            telemetry.demo_events.clear()
            observation = super().reset(scene, seed=seed)
            import cv2

            # Save the actual settled initial scene, before any policy action.
            rgb = self._base.scene["top_cam"].data.output["rgb"]
            rgb = rgb.torch if hasattr(rgb, "torch") else rgb
            rgb = rgb[0, ..., :3].detach().cpu().numpy().astype(np.uint8)
            if not cv2.imwrite(str(args.log_dir / "setup.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                raise RuntimeError("could not save the initial scene image")
            from bimanual_scoring import BimanualKnotScorer
            self.task.module._tracker = BimanualKnotScorer(CONFIG)
            self.audit = (args.log_dir / "trajectory.jsonl").open("x")
            self.review_encoder = None
            self.review_frames = 0
            (args.log_dir / "visuals.json").write_text(json.dumps(self.visual_metadata, indent=2) + "\n")
            self._audit_sample()
            if policy_physics_schedule is not None:
                from fixed_policy_physics import FixedPolicyPhysics
                if (not np.isclose(self._base.physics_dt, 1/240, rtol=0, atol=1e-12)
                        or not np.isclose(self._base.step_dt, .05, rtol=0, atol=1e-12)):
                    raise RuntimeError('Policy physics schedule requires a fresh 240 Hz simulation at 20 Hz control')
                self.policy_physics_audit = FixedPolicyPhysics(self._base)
                self.policy_physics_audit.save(args.log_dir / 'policy-physics-transition.json')
            return observation

        def step(self, action):
            if args.proof_probe and self._t >= 1.-1e-9:
                from proof_load import EndpointLoad
                from rope import rope_state
                if self.endpoint_load is None:
                    self.endpoint_load = EndpointLoad(self._base, magnitude=CONFIG['proof_load_n'])
                seating_seconds = self.task.module._tracker.config['seating_seconds']
                phase = 'seating' if self._t < 1.+seating_seconds-1e-9 else 'proof'
                if phase != self.assay_phase:
                    self.assay_phase = phase
                    self.endpoint_load.elapsed = 0.
                state = rope_state(self._base, 'cord')
                # Seat the knot gradually; an instantaneous point-force step
                # on a sub-gram end link is an impact, not a hanging-weight test.
                fraction = min(1., max(0., (self._t - 1.) / seating_seconds))
                self.endpoint_load.magnitude = CONFIG['proof_load_n'] * fraction
                self.endpoint_load.apply(state.start_tip, state.end_tip)
            if self.assay_phase is None:
                # Policies may reuse/mutate their action object after step().
                self.last_policy_action = deepcopy(action)
                self.policy_steps += 1
            policy_audit = getattr(self, 'policy_physics_audit', None)
            if policy_audit is None:
                if policy_physics_schedule is not None:
                    raise RuntimeError('Required policy physics audit was not initialized')
                return super().step(action)
            try:
                policy_audit.before_step(self._t, self.review_frames)
                result = super().step(action)
                policy_audit.after_step(self._t, self.review_frames)
                # Keep recoverable evidence without rewriting the growing
                # native interval log on every policy action.
                if (policy_audit.completed_actions == policy_physics_schedule['switch_after_completed_actions'] + 1
                        or policy_audit.completed_actions % 100 == 0):
                    policy_audit.save(args.log_dir / 'policy-physics-transition.json')
                return result
            except BaseException as exc:
                self._policy_physics_failure = exc
                policy_audit.fail(exc)
                raise

        def _run_assay(self):
            """Evaluate an already released knot after policy actions end."""
            if CONFIG.get('score_version') == 'overhand-bimanual-v4':
                return  # V4: all manipulation, including tightening, is robotic.
            if self.assay_attempted:
                return
            self.assay_attempted = True
            if args.skip_assay or args.diagnostic_layout or self.last_policy_action is None:
                return
            tracker = self.task.module._tracker
            policy_result = tracker.summary(final=True)
            (args.log_dir / 'policy-end-score.json').write_text(json.dumps(policy_result, indent=2)+'\n')
            metadata['inspect_policy_score'] = policy_result['score']
            if policy_result['score'] != .5 or policy_result['invalid'] is not None:
                return
            self.assay_transition_required = True
            from proof_load import EndpointLoad
            from rope import rope_state
            dt = float(self._base.step_dt)
            seating_seconds = tracker.config['seating_seconds']
            evidence = {'policy_steps': self.policy_steps, 'policy_end_time': float(self._t),
                        'robot_command': np.asarray(self.last_policy_action.data).reshape(-1).tolist(),
                        'assessor_only': True, 'policy_receives_no_observations_or_actions_during_assay': True,
                        'seating_seconds': seating_seconds, 'proof_seconds': CONFIG['proof_seconds'],
                        'target_endpoint_force_n': CONFIG['proof_load_n'], 'completed': False,
                        'policy_physics_hz': args.physics_hz, 'assay_physics_hz': args.assay_physics_hz,
                        'inspect_result_scope': 'Policy only; this assessor runs after Inspect finishes the episode.'}
            # Hold the policy's final joint command. The assessor adds only
            # endpoint forces, never body poses, new grasps or knot crossings.
            errors = []
            try:
                from assay_physics import AssayPhysicsSwitch
                self.physics_switch = AssayPhysicsSwitch(self._base, args.assay_physics_hz, self._t,
                                                        diagnostic_only=bool(CONFIG.get('diagnostic_only')))
                self.physics_switch.start()
                self._assay_expected_intervals = round((seating_seconds + CONFIG['proof_seconds']) / dt)
                self.endpoint_load = EndpointLoad(self._base, magnitude=0.)
                for phase, duration in (('seating', seating_seconds), ('proof', CONFIG['proof_seconds'])):
                    self.assay_phase = phase
                    self.endpoint_load.elapsed = 0.
                    count = round(duration/dt)
                    if count < 1 or not np.isclose(count*dt, duration, rtol=0, atol=1e-9):
                        raise ValueError('Assessor duration must contain a whole number of control intervals')
                    for index in range(count):
                        # Left endpoint of the integration interval: start at
                        # zero, then reach full force as the proof phase begins.
                        self.endpoint_load.magnitude = CONFIG['proof_load_n'] * (index/count if phase == 'seating' else 1.)
                        state = rope_state(self._base, 'cord')
                        self.endpoint_load.apply(state.start_tip, state.end_tip)
                        self.physics_switch.before_step(self._t, self.review_frames)
                        super().step(self.last_policy_action)
                        self.physics_switch.after_step(self._t, self.review_frames)
                        self.physics_switch.save(args.log_dir / 'assay-physics-transition.json')
                        if tracker.invalid is not None:
                            evidence['stopped_reason'] = tracker.invalid
                            return
                evidence['completed'] = True
            except BaseException as exc:
                self._transition_failure = exc
                if self.physics_switch is not None:
                    self.physics_switch.fail(exc)
                errors.append(exc)
            finally:
                if self.endpoint_load is not None:
                    try:
                        self.endpoint_load.clear()
                    except BaseException as exc:
                        errors.append(exc)
                if self.physics_switch is not None:
                    try:
                        self.physics_switch.close()
                        self.physics_switch.save(args.log_dir / 'assay-physics-transition.json')
                    except BaseException as exc:
                        self._transition_failure = exc
                        errors.append(exc)
                evidence['end_time'] = float(self._t)
                evidence['errors'] = [f'{type(exc).__name__}: {exc}' for exc in errors]
                try:
                    evidence['result'] = tracker.summary(final=True)
                    evidence['result_scope'] = 'Geometry and force telemetry only; authoritative score is in score-summary.json after native audit and finalization.'
                    (args.log_dir / 'assay.json').write_text(json.dumps(evidence, indent=2)+'\n')
                except BaseException as exc:
                    errors.append(exc)
                if errors:
                    raise BaseExceptionGroup('Post-policy assessor failed', errors)

        def _audit_sample(self):
            from hand_geometry import robot_collision_boxes
            def arr(value):
                value = value.torch if hasattr(value, 'torch') else value
                return value.detach().cpu().numpy().astype(float)
            origin = arr(self._base.scene.env_origins)[0]
            world = self._base.arena_world
            hands = []
            for side in ('left', 'right'):
                arm_name = side + '_robot'
                robot = self._base.scene[arm_name]
                arm_ids, finger_ids = self._ids[arm_name]
                q = arr(robot.data.joint_pos)[0]
                joints = q[arm_ids]
                tcp, tcp_rotation = ik.tcp_pose(joints, self.rig.bases[side])
                hands.append(dict(tcp=tcp.tolist(),
                    opening=float(np.clip(q[finger_ids].mean() / FINGER_OPEN, 0, 1)),
                    joints=joints.tolist(), tcp_rotation=tcp_rotation.tolist(),
                    finger_boxes=robot_collision_boxes(robot, origin=origin)))
            sample = {'time': float(self._t), 'hands': hands}
            from yam_arena.scene import object_states
            cord = object_states(self._env, ('cord',))['cord']
            sample['cord'] = cord.to_record()
            sample['diagnostic_only'] = bool(CONFIG.get('diagnostic_only'))
            if self.endpoint_load is not None and self.endpoint_load.active:
                sample['phase'] = self.assay_phase
                sample['proof'] = self.endpoint_load.record(self._base.step_dt)
            else:
                sample['phase'] = 'policy'
            self.task.module._tracker.update(sample)
            sample['metrics'] = self.task.module._tracker.summary()
            telemetry.latest_sample = sample
            self.audit.write(json.dumps(sample, allow_nan=False) + '\n')
            self.audit.flush()
            self._record_review_frame()

        def _record_review_frame(self):
            import cv2
            output = self._base.scene["review_cam"].data.output["rgb"]
            output = output.torch if hasattr(output, "torch") else output
            rgb = np.ascontiguousarray(output[0, ..., :3].detach().cpu().numpy(), dtype=np.uint8)
            if self.review_encoder is None:
                height, width = rgb.shape[:2]
                if not cv2.imwrite(str(args.log_dir / "setup-closeup.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                    raise RuntimeError("could not save the review setup image")
                self.review_stderr = (args.log_dir / "review-encoder.log").open("wb")
                self.review_encoder = subprocess.Popen([
                    "ffmpeg", "-hide_banner", "-loglevel", "error", "-n",
                    "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", f"{width}x{height}",
                    "-framerate", "20", "-i", "pipe:0", "-an", "-c:v", "libx264",
                    "-preset", "veryfast", "-crf", "18", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", str(args.log_dir / "review.mp4"),
                ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.review_stderr)
            self.review_encoder.stdin.write(rgb.tobytes())
            self.review_frames += 1

        def _task_step(self):
            self._audit_sample()
            return super()._task_step()

        def close(self):
            if self._close_finished:
                return
            # Never begin new simulation work while Inspect is unwinding an
            # error or interruption. Resource cleanup must still run in full.
            transition_failure = getattr(self, '_transition_failure', None)
            policy_failure = getattr(self, '_policy_physics_failure', None)
            unwinding = sys.exception() or getattr(self, '_skip_assay_reason', None) or transition_failure or policy_failure
            errors = [transition_failure] if transition_failure is not None else []
            if policy_failure is not None:
                errors.append(policy_failure)

            def attempt(operation):
                try:
                    return operation()
                except BaseException as exc:
                    errors.append(exc)
                    return None

            try:
                if self.audit is not None and not self.audit.closed and unwinding is None:
                    attempt(self._run_assay)
                if self.endpoint_load is not None:
                    attempt(self.endpoint_load.clear)
                policy_audit = getattr(self, 'policy_physics_audit', None)
                if policy_audit is not None:
                    if unwinding is not None:
                        policy_audit.fail(unwinding)
                    attempt(lambda: policy_audit.close(self._t))
                    # Preserve failure certificates even if close raised.
                    attempt(lambda: policy_audit.save(args.log_dir / 'policy-physics-transition.json'))
                elif policy_physics_schedule is not None:
                    errors.append(RuntimeError('Required policy physics audit is missing'))
                if getattr(self, 'assay_transition_required', False):
                    transition = getattr(self, 'physics_switch', None)
                    if transition is not None:
                        attempt(transition.close)
                        attempt(lambda: transition.save(args.log_dir / 'assay-physics-transition.json'))
                    def require_complete_transition():
                        report = transition.report if transition is not None else {}
                        expected = getattr(self, '_assay_expected_intervals', None)
                        if report.get('status') != 'validated_intervals' or expected is None or report.get('validated_control_intervals') != expected or not report.get('callback_removed'):
                            raise RuntimeError('Required assay timestep transition did not complete: ' +
                                               str(report.get('error', report.get('status', 'not started'))))
                    attempt(require_complete_transition)
                attempt(lambda: (args.log_dir / 'demo-corrections.json').write_text(
                    json.dumps(telemetry.demo_events, indent=2)+'\n'))
                encoder = self.review_encoder
                if encoder is not None:
                    # Broken pipes, timeouts and nonzero exits must not skip
                    # audit finalization, stderr closure or base.close().
                    if encoder.stdin is not None:
                        attempt(encoder.stdin.close)
                    result = None
                    try:
                        result = encoder.wait(timeout=30)
                        if result:
                            raise RuntimeError('review video encoder failed; see review-encoder.log')
                    except BaseException as exc:
                        errors.append(exc)
                    finally:
                        if attempt(encoder.poll) is None:
                            attempt(encoder.kill)
                            attempt(lambda: encoder.wait(timeout=5))
                        self.review_encoder = None
                    if result == 0:
                        attempt(lambda: (args.log_dir / 'review-video.json').write_text(json.dumps({
                            'camera': 'review_cam', 'frames': self.review_frames, 'fps': 20,
                            'first_time': 0.0, 'last_time': (self.review_frames-1)/20,
                            'sent_to_dream': False,
                        }, indent=2)+'\n'))
                if self.review_stderr is not None:
                    attempt(self.review_stderr.close)
                if self.audit is not None and not self.audit.closed:
                    attempt(self.audit.close)
            finally:
                # The independent audit score is authoritative for the whole
                # recording. Inspect's existing episode log remains an honest
                # policy-only result; never rewrite it to invent assay steps.
                tracker = getattr(getattr(getattr(self, 'task', None), 'module', None), '_tracker', None)
                attempt(super().close)
                if tracker is not None:
                    def save_summary():
                        from replay import certify_result
                        summary = tracker.summary(final=True)
                        bimanual = CONFIG.get('score_version') == 'overhand-bimanual-v4'
                        summary['score_scope'] = ('Robot policy only; no external load' if bimanual else
                                                  'Policy plus recorded assessor phases, when present')
                        summary['inspect_result_scope'] = ('Complete robot episode' if bimanual else
                                                          'Policy episode only; post-policy assessor runs during close')
                        transition = getattr(self, 'physics_switch', None)
                        report = transition.report if transition is not None else {}
                        summary['assay_physics_transition'] = {
                            'required': bool(getattr(self, 'assay_transition_required', False)),
                            'status': report.get('status', 'not_required'),
                            'error': report.get('error'),
                            'validated_control_intervals': report.get('validated_control_intervals', 0),
                            'expected_control_intervals': getattr(self, '_assay_expected_intervals', None),
                            'callback_removed': report.get('callback_removed'),
                        }
                        policy_audit = getattr(self, 'policy_physics_audit', None)
                        policy_report = policy_audit.report if policy_audit is not None else {}
                        summary['policy_physics_transition'] = {
                            'required': policy_physics_schedule is not None,
                            'status': policy_report.get('status', 'not_required' if policy_physics_schedule is None else 'missing'),
                            'error': policy_report.get('error'),
                            'completed_actions': policy_report.get('completed_actions'),
                            'expected_control_intervals': policy_report.get('expected_control_intervals'),
                            'closed': policy_report.get('closed'),
                            'certificate': 'policy-physics-transition.json' if policy_physics_schedule is not None else None,
                        }
                        policy_errors = []
                        if policy_physics_schedule is not None and (
                                policy_report.get('status') not in ('complete', 'not_reached')
                                or policy_report.get('closed') is not True
                                or policy_report.get('pending_interval') is not False
                                or policy_report.get('error')):
                            policy_errors.append(policy_report.get('error') or
                                                 'Required policy physics certificate is incomplete')
                            if not errors:
                                errors.append(RuntimeError(policy_errors[0]))
                        summary['finalization'] = {
                            'complete': not errors and unwinding is None,
                            'errors': [f'{type(exc).__name__}: {exc}' for exc in errors],
                            'interrupted_by': f'{type(unwinding).__name__}: {unwinding}' if unwinding else None,
                        }
                        if bimanual:
                            from bimanual_scoring import certify_result as certify_bimanual
                            summary = certify_bimanual(summary,
                                finalization_complete=summary['finalization']['complete'])
                        else:
                            summary = certify_result(summary, CONFIG, report or None,
                                finalization_complete=summary['finalization']['complete'])
                        summary['policy_physics_certification'] = {
                            'required': policy_physics_schedule is not None,
                            'status': 'incomplete' if policy_errors else 'complete' if policy_physics_schedule is not None else 'not_required',
                            'errors': policy_errors,
                        }
                        if policy_errors:
                            summary['score'] = 0.
                            summary['status'] = 'Policy physics evidence is incomplete'
                        if summary['assay_certification']['required'] and summary['assay_certification']['status'] != 'complete' and not errors:
                            errors.append(RuntimeError('Full assessor evidence is not certified: ' +
                                          '; '.join(summary['assay_certification']['errors'])))
                            summary['finalization']['complete'] = False
                            summary['finalization']['errors'] = [f'{type(exc).__name__}: {exc}' for exc in errors]
                        (args.log_dir / 'score-summary.json').write_text(json.dumps(summary, indent=2)+'\n')
                        metadata['audited_final_score'] = summary['score']
                        metadata['geometric_score'] = summary['geometric_score']
                        metadata['assay_certification'] = summary['assay_certification']
                        metadata['authoritative_score_file'] = 'score-summary.json'
                        metadata['finalization_complete'] = summary['finalization']['complete']
                        metadata['policy_physics_transition'] = summary['policy_physics_transition']
                        metadata['policy_physics_certification'] = summary['policy_physics_certification']
                    attempt(save_summary)
                self._close_finished = True
            if errors:
                raise BaseExceptionGroup('Rope trial finalization failed', errors)

    register("embodiment", "audited_rope")(AuditedRopeEmbodiment)
    policy_name = "dream"
    dream_request_audit = []
    if args.policy in ("demo", "idle", "grasp", "loop", "thread", "jig"):
        from demo_policy import register_policy
        policy_name = register_policy(mode=args.policy)
    elif args.policy == 'dream':
        policy_name = _register_dream_policy(args.dream_rows_per_request, register, dream_request_audit)
    cli_args = [
        "run", "--embodiment", "audited_rope", "--policy", policy_name,
        "-E", f"task={HERE / 'task.py'}", "-E", "object_jitter=0", "-E", "control_hz=20",
        "-E", "cam_width=320", "-E", "cam_height=176", "-E", f"headless={args.headless}",
        "--instruction", CONFIG["instruction"], "--scorer", "task_score", "--epochs", "1", "--seed", str(args.seed),
        "--max-steps", str(args.max_steps), "--store-frames", "--log-dir", str(args.log_dir),
        "--no-prompt", "--grader", "none",
    ]
    metadata = {"python": platform.python_version(), "policy": args.policy, "seed": args.seed, "max_steps": args.max_steps, "control_hz": 20, "score_version": SCORE_VERSION, "packages": {}}
    metadata['policy_physics_schedule'] = policy_physics_schedule
    if args.policy == "dream":
        metadata["dream_transport"] = "direct"
        metadata['dream_rows_per_request'] = args.dream_rows_per_request
        metadata['dream_session_replacement_policy'] = 'stop before retry or executing replacement reply'
        metadata['dream_request_audit'] = dream_request_audit
    metadata["task_config"] = CONFIG
    snapshot = args.log_dir / "task-source"
    snapshot.mkdir(exist_ok=True)
    metadata["source_sha256"] = {}
    for source in sorted(HERE.glob("*.py")):
        if source.name.startswith("."):
            continue  # Exclude filesystem metadata such as macOS AppleDouble files.
        content = source.read_bytes()
        (snapshot / source.name).write_bytes(content)
        metadata["source_sha256"][source.name] = hashlib.sha256(content).hexdigest()
    for package in ("numpy", "yam-arena", "inspect-robots", "inspect-robots-yam", "inspect-robots-dream", "dreamlabs-client", "isaaclab", "isaacsim"):
        try:
            metadata["packages"][package] = version(package)
        except PackageNotFoundError:
            metadata["packages"][package] = "not registered"
    started = time.monotonic()
    errors = []
    result = None
    previous_eval_logs = set(args.log_dir.glob('adhoc_*.json'))
    try:
        result = inspect_main(cli_args)
    except BaseException as exc:
        errors.append(exc)
    finally:
        # Also close instances if Inspect exits before its own close hook.
        for instance in instances:
            try:
                if errors:
                    instance._skip_assay_reason = errors[0]
                instance.close()
            except BaseException as exc:
                errors.append(exc)
        try:
            execution = _inspect_execution(args.log_dir, previous_eval_logs, result, errors)
            _record_execution_outcome(args.log_dir, metadata, execution)
            if execution['status'] != 'complete':
                errors.append(RuntimeError('; '.join(execution['errors'])))
        except BaseException as exc:
            errors.append(exc)
            metadata['finalization_complete'] = False
        metadata["elapsed_wall_seconds"] = round(time.monotonic() - started, 3)
        try:
            (args.log_dir / "runtime.json").write_text(json.dumps(metadata, indent=2) + "\n")
        except BaseException as exc:
            errors.append(exc)
    if not errors and metadata.get('finalization_complete') is False:
        errors.append(RuntimeError('Trial finalization is incomplete; see score-summary.json'))
    if errors:
        raise BaseExceptionGroup('Rope trial failed', errors)
    if 'audited_final_score' in metadata:
        print(f"Final audited task score: {metadata['audited_final_score']}; "
              f"see {args.log_dir / 'score-summary.json'}.")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
