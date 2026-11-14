"""Task-local YAM adapter: USD props, contact sensors, instrument display and scorer.

Only observations are read from the physics engine. Runtime writes affect the
meter's visual segments and decorative lead curves; moving bodies are never
teleported, attached or driven by task code.
"""

from dataclasses import dataclass
import importlib.abc
import importlib.util
import sys

import numpy as np
from yam_arena.task_file import Box

import scoring
import spec as S


@dataclass(frozen=True)
class Solid(Box):
    asset_file: str = ""
    quaternion: tuple = (0.0, 0.0, 0.0, 1.0)


def array(value):
    value = value.torch if hasattr(value, "torch") else value
    return value.detach().cpu().numpy().astype(float)


class Runtime:
    def __init__(self):
        self.objects = ()
        self.env = None
        self.last_step = None
        self.trial = scoring.Trial()
        self.bodies = {}
        self.forces = {}
        self.robot_boxes = []
        self.last_display = object()

    def reset(self):
        self.last_step = None
        self.trial = scoring.Trial()
        self.bodies = {}
        self.last_display = object()

    def update(self, env):
        from yam_arena import runtime as yam_runtime
        from yam_arena.analysis import world_hulls

        base = env.unwrapped
        step = int(array(base.episode_length_buf)[0])
        if self.env is not base or (self.last_step is not None and step < self.last_step):
            self.reset()
            self.env = base
        if self.last_step == step:
            return self.bodies
        self.last_step = step
        origin = array(base.scene.env_origins)[0]
        bodies = {}
        for solid in self.objects:
            if not solid.free:
                continue
            data = base.scene[solid.name].data
            pose = array(data.root_link_pose_w)[0]
            pos, quat = pose[:3]-origin, pose[3:]
            if solid.name in S.CELL_NAMES:
                low, high = S.cell_bounds(pos, quat)
            else:
                local_low, local_high = np.array([-0.012, -0.035, -0.012]), np.array([0.012, 0.117, 0.012])
                center, half = (local_high+local_low)/2, (local_high-local_low)/2
                rot = S.rotation(quat)
                center = pos + rot @ center
                ext = np.abs(rot) @ half
                low, high = center-ext, center+ext
            bodies[solid.name] = scoring.Body(pos, quat, array(data.root_lin_vel_w)[0],
                                              array(data.root_ang_vel_w)[0], low, high)
        forces = {}
        for name in S.PROBE_NAMES:
            data = base.scene[name + "_contacts"].data
            history = array(data.normal_force_matrix_w_history)[0]
            values = history.reshape(-1, len(S.CELL_NAMES), 3)
            mean_force = np.linalg.norm(values, axis=-1).mean(axis=0)
            forces[name] = dict(zip(S.CELL_NAMES, map(float, mean_force), strict=True))
        robot_boxes = []
        for side in ("left", "right"):
            poses = yam_runtime.link_poses(base.scene[side + "_robot"])
            for _, hull in world_hulls(poses):
                robot_boxes.append((hull.vertices.min(axis=0)-origin, hull.vertices.max(axis=0)-origin))
        self.bodies, self.forces, self.robot_boxes = bodies, forces, robot_boxes
        self.trial.update(float(base.step_dt), bodies, forces, robot_boxes)
        self._visuals(base)
        return bodies

    def _visuals(self, base):
        import omni.usd
        from pxr import Gf, UsdGeom

        stage = omni.usd.get_context().get_stage()
        prefix = "/World/envs/env_0/voltage_meter"
        reading = self.trial.reading_cv
        if reading != self.last_display:
            if reading is None:
                text = " ---"
            else:
                number = abs(reading)
                text = ("-" if reading < 0 else " ") + f"{number:03d}"
            glyphs = {"0": "abcdef", "1": "bc", "2": "abdeg", "3": "abcdg", "4": "bcfg",
                      "5": "acdfg", "6": "acdefg", "7": "abc", "8": "abcdefg", "9": "abcdfg",
                      "-": "g", " ": ""}
            for i, char in enumerate(text):
                for segment in "abcdefg":
                    prim = stage.GetPrimAtPath(f"{prefix}/digit_{i}_{segment}")
                    if prim.IsValid():
                        prim.GetAttribute("visibility").Set("inherited" if segment in glyphs[char] else "invisible")
            stage.GetPrimAtPath(prefix + "/decimal").GetAttribute("visibility").Set("inherited" if reading is not None else "invisible")
            self.last_display = reading
        for name, side in (("red_probe", 1), ("black_probe", -1)):
            body = self.bodies[name]
            end = body.pos + body.rotation @ np.array([0, -0.035, 0])
            points = [Gf.Vec3f(0.450, side*0.065, S.TOP+0.036),
                      Gf.Vec3f(0.43, side*0.43, S.TOP+0.012),
                      Gf.Vec3f(float(end[0])-0.05, side*0.43, S.TOP+0.016),
                      Gf.Vec3f(*map(float, end))]
            path = "/World/" + name + "_lead"
            curve = UsdGeom.BasisCurves.Get(stage, path)
            if not curve:
                curve = UsdGeom.BasisCurves.Define(stage, path)
                curve.CreateTypeAttr("linear")
                curve.CreateCurveVertexCountsAttr([4])
                curve.CreateWidthsAttr([0.003])
                curve.SetWidthsInterpolation(UsdGeom.Tokens.constant)
                curve.CreateDisplayColorAttr([(0.5, 0.025, 0.025) if side == 1 else (0.015, 0.015, 0.015)])
            curve.CreatePointsAttr(points)


STATE = Runtime()


def reset_trial_event(env, env_ids):
    STATE.reset()


def patch_scene(scene):
    if getattr(scene, "_battery_voltage_sort", False):
        return
    scene._battery_voltage_sort = True
    original_object = scene.task_object
    original_configure = scene.env_cfg_callback

    def task_object(solid):
        if not isinstance(solid, Solid):
            return original_object(solid)
        return scene.Object(name=solid.name, usd_path=str(S.DIRECTORY / "assets" / solid.asset_file),
                            object_type=scene.ObjectType.RIGID if solid.free else scene.ObjectType.BASE,
                            initial_pose=scene.Pose(position_xyz=tuple(map(float, solid.pos)),
                                                    rotation_xyzw=tuple(map(float, solid.quaternion))))

    def configure(hz=None):
        original = original_configure(hz)

        def apply(cfg):
            from isaaclab.sensors import ContactSensorCfg
            from isaaclab.managers import EventTermCfg
            from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg

            cfg = original(cfg)
            cfg.seed = 0
            cfg.events.battery_trial_reset = EventTermCfg(func=reset_trial_event, mode="reset")
            for side in ("left", "right"):
                getattr(cfg.scene, side+"_robot").spawn.collision_props = PhysxCollisionPropertiesCfg(
                    contact_offset=0.0005, rest_offset=0.0)
            for name in S.PROBE_NAMES:
                sensor = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/"+name, history_length=12,
                    track_air_time=False, filter_prim_paths_expr=["{ENV_REGEX_NS}/"+cell for cell in S.CELL_NAMES])
                setattr(cfg.scene, name+"_contacts", sensor)
            return cfg
        return apply

    def object_states(env, names):
        return STATE.update(env)

    scene.task_object = task_object
    scene.env_cfg_callback = configure
    scene.object_states = object_states


class AfterSceneImport(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        module_spec = importlib.util.find_spec(fullname)
        original = module_spec.loader.exec_module

        def execute(module):
            original(module)
            patch_scene(module)

        module_spec.loader.exec_module = execute
        return module_spec


def install(objects):
    STATE.objects = tuple(objects)
    if "yam_arena.scene" in sys.modules:
        patch_scene(sys.modules["yam_arena.scene"])
    elif not any(isinstance(finder, AfterSceneImport) for finder in sys.meta_path):
        sys.meta_path.insert(0, AfterSceneImport())
