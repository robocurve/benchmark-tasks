"""An independent floor-bay adaptation of the accepted wheelchair architecture."""
from __future__ import annotations

import atexit
from dataclasses import dataclass, replace
import os

import numpy as np
from yam_arena import inspect_embodiment as adapter, station, task_file
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY
from yam_arena.rig import Camera
from yam_arena.scripted import YamScriptedPolicy
from yam_arena import ik
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.urdf import axis_angle_matrix

from geometry import BASES

CAMERA = (-1.90, -.40, 1.97)
TARGET = (.39, 0., 1.08)

def overview():
    up = np.array((0., 0., 1.))
    backwards = np.asarray(CAMERA) - TARGET
    backwards /= np.linalg.norm(backwards)
    right = np.cross(up, backwards)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(backwards, right), backwards))
    return Camera(CAMERA, tuple(matrix_to_quat_xyzw(rotation)), "opengl", station.top_camera().lens)

def room():
    import isaaclab.sim as sim
    from isaaclab_arena.assets.object_library import DomeLight
    from yam_arena.scene import _box

    steel = (.30,.31,.33)
    objects = [_box("floor", (20.,20.,.04), (0.,0.,-.02), (.55,.56,.55)),
               DomeLight(spawner_cfg=sim.DomeLightCfg(intensity=1900., color=(.87,.91,1.)))]
    deck_height = BASES["left"][2] - .7475
    tabletop = BASES["left"][2]
    # Existing upper deck, with the liner hanging into the lower bay in front.
    objects += [
        _box("upper_deck", (2.7,2.4,.16), (-1.27,0.,deck_height-.08), (.42,.44,.46)),
        _box("bench_top", (.80,1.2,.05), (-.30,0.,tabletop-.025), (.63,.51,.37)),
    ]
    leg_height = tabletop - .05 - deck_height
    for i, x in enumerate((-.65,.045)):
        for j, y in enumerate((-.54,.54)):
            objects.append(_box(f"bench_leg_{i}_{j}", (.06,.06,leg_height),
                                (x,y,deck_height + leg_height/2), steel))
    x,y,z = CAMERA
    objects += [_box("mast_foot", (.24,.24,.015), (x-.10,y,.0075), steel),
                _box("mast", (.045,.045,z-.015), (x-.10,y,(z+.015)/2), steel),
                _box("mast_boom", (.11,.04,.035), (x-.055,y,z+.019), steel)]
    return objects

class ShowerBay(adapter.YamArenaEmbodiment):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.rig = replace(self.rig, name="shower-bay", bases=BASES, top_cam=overview())
        eid = adapter.environment_id(self.rig.name, self.task)
        self.info = replace(self.info, name="yam_shower_bay", environment_id=eid)
        task_file.register(eid, self.task)

    def reset(self, scene, *, seed=None):
        if self._env is not None:
            from isaaclab_arena.utils.isaaclab_utils.simulation_app import teardown_simulation_app

            # Reconstruct the contact scene between trials. Retaining the GPU
            # contact state changed the settling pose of these small eyelets.
            self.task.module.release_environment()
            self._base.sim._disable_app_control_on_stop_handle = True
            self._env.close()
            self._env = self._base = None
            teardown_simulation_app(make_new_stage=True)
        return super().reset(scene, seed=seed)

    def _ensure_sim(self, goal):
        if self._env is not None:
            return
        if adapter._APP is None:
            from isaaclab.app import AppLauncher

            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            options = {"headless": self.headless, "device": self.device, "enable_cameras": True}
            if not self.headless:
                options["visualizer"] = ["kit"]
                os.environ["DISPLAY"] = adapter.window_display(os.environ)
            adapter._APP = AppLauncher(**options).app
            atexit.register(adapter._close_app_at_exit)

        import torch
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
        from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.terms.events import reset_articulation_pose_and_joints
        from yam_arena import scene
        from yam_arena.embodiment import BimanualYamEmbodiment

        config = BimanualYamEmbodiment(enable_cameras=True, wrist_cam_mount=self.wrist_cam_mount,
                                         contact_sensors=False, rig=self.rig,
                                         cam_width=self.cam_width, cam_height=self.cam_height)
        stock_config = scene.env_cfg_callback(self.info.control_hz)

        def configure(config):
            config = stock_config(config)
            for spec in self.task.objects:
                if getattr(spec, "articulation", False):
                    getattr(config.events, spec.name).func = reset_articulation_pose_and_joints
            return config

        environment = IsaacLabArenaEnvironment(
            name="full_size_shower", scene=Scene([*room(), *(scene.task_object(spec) for spec in self.task.objects)]),
            embodiment=config, env_cfg_callback=configure,
        )
        self._env = ArenaEnvBuilder(environment, ArenaEnvBuilderCfg(num_envs=1, device=self.device,
                                                                    solve_relations=False)).make_registered()
        self._base, self._goal, self._torch = self._env.unwrapped, goal, torch
        self._free_objects = self.task.free_objects
        self._ready = torch.tensor([[*READY, 1., *READY, 1.]], dtype=torch.float32, device=self._base.device)
        self.task.module.initialize_environment(self._env)
        self._ids = {
            name: tuple(self._base.scene[name].find_joints(joints, preserve_order=True)[0]
                        for joints in (ARM_JOINTS, FINGER_JOINTS))
            for name in adapter.ARMS
        }

@dataclass
class ShowerWaypoint(Waypoint):
    tilt: float = 0.


class ShowerPlan(ArmPlan):
    def target(self, time):
        pos, rot, grip = self.start_pos, self.start_rot, self.finger_start
        for waypoint in self.waypoints:
            goal = np.asarray(waypoint.pos)
            turn = axis_angle_matrix(np.array([0., 1., 0.]), waypoint.tilt)
            target_rot = turn @ grip_rotation(waypoint.yaw)
            if time < waypoint.duration:
                blend = ik.smoothstep(time / waypoint.duration)
                return (pos + blend * (goal - pos), ik.slerp_matrix(rot, target_rot, blend),
                        grip + blend * (waypoint.finger - grip), waypoint.label)
            time -= waypoint.duration
            pos, rot, grip = goal, target_rot, waypoint.finger
        return pos, rot, grip, "done"


class ShowerScripted(YamScriptedPolicy):
    def bind(self, embodiment_info):
        super().bind(embodiment_info)
        self._bases = BASES

    def reset(self, scene):
        self._scene = scene
        self._sensed_start = False
        self._plan(scene)

    def _plan(self, scene):
        super().reset(scene)
        self._plans = {arm: ShowerPlan(plan.waypoints, self._bases[arm])
                       for arm, plan in self._plans.items()}

    def act(self, observation):
        # Rebuild the trajectory after the embodiment has finished settling under gravity.
        if not self._sensed_start:
            self._plan(self._scene)
            self._sensed_start = True
        return super().act(observation)

def yam_shower_bay(**kwargs):
    return ShowerBay(**kwargs)

def shower_scripted(**kwargs):
    return ShowerScripted(**kwargs)
