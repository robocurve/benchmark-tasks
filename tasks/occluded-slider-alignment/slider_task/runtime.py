"""Version-pinned, process-local extension to official task spawns and evaluator reads.

No upstream source edits. No robot, camera, controller, rendering or policy input changes.
PhysX reporting was already enabled on the official robot's USD spawner.
"""

import atexit
import json
from pathlib import Path
import numpy as np

from . import layout as L
from .contact import ContactHistory
from .scoring import Evaluator, Measurement

HISTORY = ContactHistory()
EVALUATOR = Evaluator()
TRACE = []


def install_scene_extension():
    from yam_arena import scene
    from isaaclab.sim.utils import clone
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
    from .usd_scene import augment_slider
    original_object = scene.task_object
    original_states = scene.object_states

    def task_object(spec):
        obj = original_object(spec)
        if spec.name == 'slider':
            # The pinned config exposes a lazy callable reference; resolve the
            # known Box spawner explicitly before accessing its clone wrapper.
            from isaaclab.sim.spawners.shapes.shapes import spawn_cuboid
            original_spawn = spawn_cuboid

            @clone
            def spawn(path, cfg, *args, **kwargs):
                # Set after PhysxManager initialization but before scene parsing,
                # as Isaac Lab's own ContactSensor constructor does.
                import carb
                carb.settings.get_settings().set_bool('/physics/disableContactProcessing', False)
                prim = original_spawn.__wrapped__(path, cfg, *args, **kwargs)
                augment_slider(prim.GetStage(), str(prim.GetPath()))
                return prim

            obj.spawner_cfg.func = spawn
            # Tight task contacts; official robot collider configuration is untouched.
            obj.spawner_cfg.collision_props = PhysxCollisionPropertiesCfg(contact_offset=.001, rest_offset=0)
            obj.spawner_cfg.physics_material.static_friction = .20
            obj.spawner_cfg.physics_material.dynamic_friction = .15
            # Arena constructs object_cfg eagerly, copying the spawn config.
            # Mutating spawner_cfg alone leaves the actual scene as a plain box.
            obj.object_cfg.spawn = obj.spawner_cfg
        return obj

    def object_states(env, names):
        states = original_states(env, names)
        if 'slider' not in names:
            return states
        body = env.unwrapped.scene['slider']
        pose = body.data.root_pose_w.torch[0].detach().cpu().numpy()
        origin = env.unwrapped.scene.env_origins[0].detach().cpu().numpy()
        pos = pose[:3] - origin
        velocity = body.data.root_lin_vel_w.torch[0].detach().cpu().numpy()
        # Isaac Lab 3 uses xyzw throughout this pinned adapter.
        angle = 2 * np.arccos(np.clip(abs(pose[6]), 0, 1))
        fixture_error = max(np.linalg.norm(states[o.name].pos - np.asarray(o.pos)) for o in L.FIXED)
        import carb
        reporting = not carb.settings.get_settings().get('/physics/disableContactProcessing')
        # Physical translation with no observed robot contact is a broken monitor,
        # not evidence of a collision-free trial.
        covered = abs(pos[0] - L.HOME[0]) <= .001 or HISTORY.robot_slider_reports > 0
        m = Measurement(displacement=float(pos[0] - L.HOME[0]), speed=float(np.linalg.norm(velocity)),
                        lateral_error=float(np.linalg.norm(pos[1:] - np.asarray(L.HOME[1:]))),
                        angular_error=float(angle), fixture_error=float(fixture_error),
                        protected_contact_s=HISTORY.max_duration, penetration=HISTORY.max_penetration,
                        peak_tab_force=HISTORY.peak_tab_force,
                        instrumented=bool(HISTORY.instrumented and reporting and covered),
                        released=not HISTORY.active_slider_pairs)
        states['__evaluation__'] = EVALUATOR.update(m)
        robots = {}
        for name in ('left_robot', 'right_robot'):
            robot = env.unwrapped.scene[name]
            robots[name] = dict(joint_names=robot.joint_names,
                joint_pos=robot.data.joint_pos.torch[0].detach().cpu().tolist())
        TRACE.append(dict(step=len(TRACE), **states['__evaluation__'], robots=robots,
                          robot_slider_reports=HISTORY.robot_slider_reports,
                          active_slider_pairs=sorted(HISTORY.active_slider_pairs)))
        HISTORY.control_step = len(TRACE)
        return states

    scene.task_object = task_object
    scene.object_states = object_states


def install(output_dir, record_until_s=None):
    """Install before calling the official Inspect Robots CLI, delaying Kit until first reset."""
    from yam_arena import inspect_embodiment as official
    original_ensure = official.YamArenaEmbodiment._ensure_sim
    original_reset = official.YamArenaEmbodiment.reset
    original_step = official.YamArenaEmbodiment.step
    installed = False
    completed_episodes = []
    reset_index = -1

    def snapshot():
        return dict(kind='privileged_evaluation_trace', physics_executed=bool(TRACE),
                    scoring_version='visual-contact-v2', epoch=reset_index,
                    contact_callback_error=HISTORY.error,
                    contact_callback_count=HISTORY.callback_count,
                    robot_slider_reports=HISTORY.robot_slider_reports,
                    demo_record_until_s=record_until_s,
                    steps=list(TRACE), contacts=list(HISTORY.events))

    def ensure(self, goal):
        nonlocal installed
        if not self.task or self.task.path.parent != Path(__file__).resolve().parents[1]:
            raise RuntimeError('This task launcher cannot patch unrelated tasks')
        if self.info.control_hz != 20 or self.object_jitter != 0 or self.wrist_cam_mount != 'link':
            raise RuntimeError('Use the frozen 20 Hz, jitter=0, link-mounted camera contract')
        if self.rig != official.station.rig('calibrated') or (self.cam_width, self.cam_height) != (320, 176):
            raise RuntimeError('Original station calibration and 320x176 policy resolution required')
        if not installed:
            if official._APP is not None:
                raise RuntimeError('Start this task in a fresh process')
            # Identical headless AppLauncher options used by the pinned official embodiment.
            from isaaclab.app import AppLauncher
            official._APP = AppLauncher(headless=True, device=self.device, enable_cameras=True).app
            atexit.register(official._close_app_at_exit)
            install_scene_extension()
            installed = True
        original_ensure(self, goal)
        # Isaac Lab's PhysxManager disables reporting by default. This is a
        # task-process evaluator setting, matching its ContactSensor behavior;
        # no persistent/system settings or policy observation channels change.
        import carb
        settings = carb.settings.get_settings()
        settings.set_bool('/physics/disableContactProcessing', False)
        if settings.get('/physics/disableContactProcessing'):
            raise RuntimeError('PhysX contact processing is disabled')
        import omni.usd
        from pxr import Usd, UsdPhysics, UsdShade
        stage = omni.usd.get_context().get_stage()
        if not hasattr(HISTORY, 'tensor_view'):
            HISTORY.bind_tensor_monitor(stage, self._base)
        reporters = {str(p.GetPath()): p.GetAttribute('physxContactReport:threshold').Get()
                     for p in Usd.PrimRange(stage.GetPrimAtPath('/World/envs/env_0'))
                     if 'PhysxContactReportAPI' in p.GetAppliedSchemas()}
        if not reporters:
            raise RuntimeError('No contact-reporting rigid bodies in live scene')
        root = '/World/envs/env_0/slider'
        joint = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(root + '/guide_joint'))
        if not joint or not stage.GetPrimAtPath(root + '/push_tab'):
            raise RuntimeError('Runtime scene is missing the slider joint or handle')
        colliders = {}
        for name in ('geometry/mesh', 'push_tab'):
            prim = stage.GetPrimAtPath(root + '/' + name)
            material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
            colliders[name] = dict(material=str(material.GetPath()),
                contact_offset=prim.GetAttribute('physxCollision:contactOffset').Get())
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        (out / 'live_scene_contract.json').write_text(json.dumps(dict(
            joint_type='PhysicsPrismaticJoint', axis=joint.GetAxisAttr().Get(),
            lower=joint.GetLowerLimitAttr().Get(), upper=joint.GetUpperLimitAttr().Get(),
            body1=[str(p) for p in joint.GetBody1Rel().GetTargets()],
            colliders=colliders, contact_processing_enabled=True,
            contact_reporters=reporters, tensor_contact=HISTORY.tensor_contract), indent=2) + '\n')

    def reset(self, *args, **kwargs):
        nonlocal reset_index
        if reset_index >= 0:
            completed_episodes.append(snapshot())
        reset_index += 1
        EVALUATOR.reset()
        HISTORY.reset()
        TRACE.clear()
        obs = original_reset(self, *args, **kwargs)
        # Save real reset observations before any scripted manipulation. Native
        # images are the existing camera output, not a new camera or calibration.
        import cv2
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        shapes = {}
        for name, rgb in obs.images.items():
            rgb = np.asarray(rgb)
            np.save(out / f'setup_{name}_policy.npy', rgb)
            if not cv2.imwrite(str(out / f'setup_{name}_policy.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                raise RuntimeError('Could not save policy screenshot')
            native = self._base.scene[name].data.output['rgb']
            native = native.torch if hasattr(native, 'torch') else native
            native = native[0, ..., :3].detach().cpu().numpy()
            np.save(out / f'setup_{name}_native.npy', native)
            if not cv2.imwrite(str(out / f'setup_{name}_native.png'), cv2.cvtColor(native, cv2.COLOR_RGB2BGR)):
                raise RuntimeError('Could not save native screenshot')
            shapes[name] = dict(policy=list(rgb.shape), native=list(native.shape))
        (out / 'setup_capture.json').write_text(json.dumps(dict(
            kind='REAL_RESET_CAMERA_OUTPUT', action_count=0, cameras=shapes,
            joint_pos=np.asarray(obs.state['joint_pos']).tolist(),
            slider_pose=self._base.scene['slider'].data.root_pose_w.torch[0].detach().cpu().tolist(),
            note='Native output saved before original policy resize; no camera changes.'), indent=2) + '\n')
        import shutil
        reset_dir = out / 'resets' / f'epoch-{reset_index:03d}'
        reset_dir.mkdir(parents=True, exist_ok=True)
        for path in out.glob('setup_*'):
            if path.is_file():
                shutil.copy2(path, reset_dir / path.name)
        # Safety includes reset settling; do not erase dangerous initial contact.
        HISTORY.control_step = 0
        return obs

    official.YamArenaEmbodiment._ensure_sim = ensure
    official.YamArenaEmbodiment.reset = reset

    def step(self, action):
        result = original_step(self, action)
        # Demo recording only: keep applying normal actions and evaluating every
        # step through retreat. Later safety failures still invalidate the score.
        # Normal task success and score rules remain strategy-independent.
        if record_until_s is not None and self._t < record_until_s and result.termination_reason == 'success':
            from dataclasses import replace
            return replace(result, terminated=False, termination_reason=None)
        return result

    official.YamArenaEmbodiment.step = step

    def save():
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        payload = snapshot()
        payload['episodes'] = completed_episodes + ([snapshot()] if reset_index >= 0 else [])
        (out / 'evaluation_trace.json').write_text(json.dumps(payload, indent=2) + '\n')

    # Registered before official app shutdown: launch.py also calls this in finally.
    return save
