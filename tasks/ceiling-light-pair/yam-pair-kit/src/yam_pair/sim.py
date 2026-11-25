"""Isaac side of yam_pair: four arms, six station cameras, cross-station contact sensors, the room and the task's
objects as one Arena environment. Import only after the Kit app has started (``yam_pair.embodiment`` does that)."""

from __future__ import annotations

import copy
import math
import re
import sys

import isaaclab.sim as sim_utils
import numpy as np
from isaaclab.assets import ArticulationCfg
from isaaclab.managers import EventTermCfg, ObservationTermCfg
from isaaclab.managers.recorder_manager import DatasetExportMode, RecorderManagerBaseCfg
from isaaclab.sensors import CameraCfg, ContactSensorCfg
from isaaclab.visualizers.visualizer_cfg import VisualizerCfg
from isaaclab_arena.assets.object import Object
from isaaclab_arena.assets.object_library import DomeLight
from isaaclab_arena.assets.object_type import ObjectType
from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
from isaaclab_arena.scene.scene import Scene
from isaaclab_arena.utils.cameras import ArenaCameraCfg
from isaaclab_arena.utils.configclass import make_configclass
from isaaclab_arena.utils.pose import Pose
from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg
from yam_arena import ik, station
from yam_arena import scene as yscene
from yam_arena.actions import YamArmActionCfg, YamGripperActionCfg, bimanual_joint_pos_14, reset_arms_to_default
from yam_arena.embodiment import RENDERER, BimanualYamEmbodiment, YamObservationsCfg, _PolicyObs
from yam_arena.robot import yam_arm_cfg
from yam_arena.station import linear_rgb
from yam_arena.urdf import quat_xyzw_to_matrix

from yam_pair.layout import SIDES, STATIONS, TABLE_TOP, Layout
from yam_pair.task import BodyState, Shape, UsdBody

TABLE_THICKNESS = 0.06
OVERVIEW_FOCAL_MM = 18.0
STAND_PARTS = {"stand_foot": "foot", "stand_mast": "mast", "stand_arm": "arm", "camera_housing": "housing"}


def arm_prim(s: str, side: str) -> str:
    return f"{s.upper()}{side.title()}Arm"


def look_at(eye, target) -> tuple[float, float, float, float]:
    """OpenGL-convention camera rotation (looks along -z, +y up) from ``eye`` towards ``target``, xyzw."""
    z = np.asarray(eye, float) - np.asarray(target, float)
    z /= np.linalg.norm(z)
    x = np.cross([0.0, 0.0, 1.0], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return tuple(float(v) for v in ik.matrix_to_quat_xyzw(np.column_stack([x, y, z])))


def overview_pose(layout: Layout, table_center) -> tuple[tuple, tuple]:
    """A third-person eye that sees both stations: from the side for facing stations, from the front-left
    otherwise."""
    cx, cy = table_center
    target = np.array([cx, cy, TABLE_TOP + 0.05])
    if layout.arrangement == "facing":
        h = layout.heading("a")
        across = np.array([math.sin(h), -math.cos(h), 0.0])  # to station A's right: the side between the stations
        along = np.array([math.cos(h), math.sin(h), 0.0])
        eye = target + 1.75 * across + 0.35 * along + np.array([0.0, 0.0, 1.05])
    else:
        h = layout.heading("a")
        fwd = np.array([math.cos(h), math.sin(h), 0.0])
        eye = target + 1.9 * fwd + np.array([0.0, 0.0, 1.2]) + 0.3 * np.array([-math.sin(h), math.cos(h), 0.0])
    return tuple(float(v) for v in eye), tuple(float(v) for v in target)


class FourArmEmbodiment(BimanualYamEmbodiment):
    """Two Dream-look YAM stations (four arms) placed by a :class:`Layout`."""

    name = "yam_pair_four_arm"

    def __init__(self, layout: Layout, decimation: int, overview: tuple[int, int] | None, table_center=(0.0, 0.0)):
        # Renders at yam_arena's 640x360 (the inspect embodiment resizes to the policy size, as yam_arena does).
        super().__init__(enable_cameras=True, contact_sensors=False, rig=station.rig())
        template = copy.deepcopy(self.camera_config)
        self.layout = layout
        self.scene_keys = tuple(f"{s}_{side}_robot" for s in STATIONS for side in SIDES)
        fields, actions, cameras = [], [], []
        links = list(self.link_paths.values())
        for s in STATIONS:
            R = layout.rotation(s)
            for side in SIDES:
                key, prim = f"{s}_{side}_robot", arm_prim(s, side)
                cfg = yam_arm_cfg("{ENV_REGEX_NS}/" + prim, tuple(float(v) for v in layout.to_world(s, station.BASES[side])),
                                  layout.quat_xyzw(s), dream_look=True)
                fields.append((key, ArticulationCfg, cfg))
                actions.append((f"{s}_{side}_arm", YamArmActionCfg, YamArmActionCfg(asset_name=key)))
                actions.append((f"{s}_{side}_grip", YamGripperActionCfg, YamGripperActionCfg(asset_name=key)))
                cam = copy.deepcopy(getattr(template, f"{side}_cam"))
                cam.prim_path = "{ENV_REGEX_NS}/" + prim + self.link_paths["gripper"] + f"/{s}_{side}_wrist_cam"
                cameras.append((f"{s}_{side}_cam", CameraCfg, cam))
                # Every link of this arm against every link of the other station's two arms; the history keeps each
                # physics substep of a control step, so a brief knock is not missed.
                other = "b" if s == "a" else "a"
                targets = ["{ENV_REGEX_NS}/" + arm_prim(other, o) + path for o in SIDES for path in links]
                sensor = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/" + prim + "/.*", history_length=decimation,
                                          filter_prim_paths_expr=targets)
                fields.append((f"{s}_{side}_contacts", ContactSensorCfg, sensor))
            top = self.rig.top_cam
            cam = copy.deepcopy(template.top_cam)
            cam.prim_path = "{ENV_REGEX_NS}/" + s.upper() + "TopCam"
            cam.offset.pos = tuple(float(v) for v in layout.to_world(s, top.pos))
            cam.offset.rot = tuple(float(v) for v in ik.matrix_to_quat_xyzw(R @ quat_xyzw_to_matrix(np.array(top.rot_xyzw))))
            cameras.append((f"{s}_top_cam", CameraCfg, cam))
        if overview is not None:
            width, height = overview
            eye, target = overview_pose(layout, table_center)
            cameras.append(("overview_cam", CameraCfg, CameraCfg(
                prim_path="{ENV_REGEX_NS}/OverviewCam", height=height, width=width, data_types=["rgb"],
                spawn=sim_utils.PinholeCameraCfg(focal_length=OVERVIEW_FOCAL_MM, clipping_range=(0.05, 20.0)),
                offset=CameraCfg.OffsetCfg(pos=eye, rot=look_at(eye, target), convention="opengl"),
                renderer_cfg=RENDERER)))
        self.scene_config = make_configclass("YamPairSceneCfg", fields)()
        self.action_config = make_configclass("YamPairActionsCfg", actions)()
        self.camera_config = make_configclass("YamPairCamerasCfg", cameras, bases=(ArenaCameraCfg,))()
        self.observation_config = YamObservationsCfg(policy=_PolicyObs(joint_pos=ObservationTermCfg(func=_all_states)))
        reset = EventTermCfg(func=reset_arms_to_default, mode="reset", params={"asset_names": self.scene_keys})
        self.event_config = make_configclass("YamPairEventsCfg", [("reset_arms", EventTermCfg, reset)])()


def _all_states(env):
    import torch

    return torch.cat([bimanual_joint_pos_14(env, f"{s}_left_robot", f"{s}_right_robot") for s in STATIONS], dim=1)


# -- the room ----------------------------------------------------------------------------------------------------------
def _stand(layout: Layout, s: str, top) -> list[Object]:
    out = []
    R = layout.rotation(s)
    for part, (kind, size, pos, rot) in station.camera_stand(top).items():
        name = f"{s}_stand_{STAND_PARTS[part]}"
        color = linear_rgb(station.HOUSING if part == "camera_housing" else station.STAND)
        wpos = tuple(float(v) for v in layout.to_world(s, pos))
        wrot = tuple(float(v) for v in ik.matrix_to_quat_xyzw(R @ quat_xyzw_to_matrix(np.array(rot))))
        if kind == "box":
            out.append(yscene._box(name, size, wpos, color, wrot))
        else:
            out.append(Object(name=name, object_type=ObjectType.BASE, spawner_cfg=sim_utils.CapsuleCfg(
                radius=size[0], height=size[1], collision_props=PhysxCollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.5)),
                initial_pose=Pose(position_xyz=wpos, rotation_xyzw=wrot)))
    return out


def table_geometry(task, layout: Layout) -> dict | None:
    if task.table is None:
        return None
    size, center = layout.default_table()
    t = dict(task.table)
    size = tuple(t.get("size", size))[:2]
    center = tuple(t.get("center", center))[:2]
    return {"size": size, "center": center, "thickness": float(t.get("thickness", TABLE_THICKNESS)),
            "color": tuple(t["color"]) if "color" in t else None}


def room(task, layout: Layout, top_cam) -> list[Object]:
    assets = [
        yscene._box("floor", (200.0, 200.0, 0.02), (0.0, 0.0, station.FLOOR_Z - 0.01), yscene.STATION_FLOOR_COLOR),
        DomeLight(spawner_cfg=sim_utils.DomeLightCfg(intensity=yscene.STATION_DOME_INTENSITY,
                                                     color=linear_rgb(station.SKY))),
    ]
    for s in STATIONS:  # each station's lamp where Dream's hangs, high on the right of its arms
        assets.append(Object(name=f"lamp_{s}", object_type=ObjectType.BASE,
                             spawner_cfg=sim_utils.SphereLightCfg(intensity=yscene.STATION_LAMP_INTENSITY,
                                                                  radius=yscene.STATION_LAMP_RADIUS),
                             initial_pose=Pose(position_xyz=tuple(float(v) for v in
                                                                  layout.to_world(s, station.LAMP_POS)))))
        assets.extend(_stand(layout, s, top_cam))
    table = table_geometry(task, layout)
    if table is not None:
        (sx, sy), (cx, cy), th = table["size"], table["center"], table["thickness"]
        color = linear_rgb(table["color"]) if table["color"] else yscene.STATION_TABLE_COLOR
        assets.append(yscene._box("table", (sx, sy, th), (cx, cy, TABLE_TOP - th / 2), color, cls=yscene._Table))
    return assets


# -- task objects ------------------------------------------------------------------------------------------------------
def _usd_object(item: UsdBody, folder) -> Object:
    from isaaclab.actuators import ImplicitActuatorCfg

    material = sim_utils.RigidBodyMaterialCfg(static_friction=item.friction[0], dynamic_friction=item.friction[1],
                                              friction_combine_mode="min" if item.friction_min else "average")
    addon = {"physics_material": material}
    if item.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=item.solver_iters, enabled_self_collisions=False)
    else:
        addon["rigid_props"] = sim_utils.RigidBodyPropertiesCfg(solver_position_iteration_count=item.solver_iters,
                                                                kinematic_enabled=item.mass is None)
        if item.torsional_patch:
            addon["collision_props"] = sim_utils.CollisionPropertiesCfg(
                torsional_patch_radius=item.torsional_patch, min_torsional_patch_radius=0.8 * item.torsional_patch)
        if item.mass is not None:
            addon["mass_props"] = sim_utils.MassPropertiesCfg(mass=item.mass)

    class UsdObject(Object):
        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            joints = dict(item.joint_pos)
            cfg.init_state.joint_pos = joints
            driven = {d[0] for d in item.drives}
            free = [n for n in joints if not any(re.fullmatch(d, n) for d in driven)]
            cfg.actuators = {}
            for name, k, d, *lim in item.drives:
                extra = {"joint_effort_limit": lim[0]} if lim else {}
                cfg.actuators[name] = ImplicitActuatorCfg(joint_names_expr=[name], stiffness=k, damping=d,
                                                          friction=0.0, **extra)
            if free or not driven:
                cfg.actuators["joints"] = ImplicitActuatorCfg(joint_names_expr=free or [".*"], stiffness=0.0,
                                                              damping=item.joint_damping, friction=0.0)
            return cfg

    obj = UsdObject(name=item.name, usd_path=str(folder / item.usd),
                    object_type=ObjectType.ARTICULATION if item.articulation else ObjectType.RIGID,
                    spawn_cfg_addon=addon)
    obj.set_initial_pose(Pose(position_xyz=tuple(float(v) for v in item.pos), rotation_xyzw=tuple(item.rot_xyzw)))
    return obj


def _shape_object(item: Shape) -> Object:
    body = {"visual_material": sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(item.color), roughness=0.6)}
    if item.collide:
        body["collision_props"] = sim_utils.CollisionPropertiesCfg()
    if item.kind == "box":
        cfg = sim_utils.CuboidCfg(size=tuple(item.size), **body)
    else:
        cfg = sim_utils.CapsuleCfg(radius=item.radius, height=item.height, axis=item.axis, **body)
    obj = Object(name=item.name, object_type=ObjectType.BASE, spawner_cfg=cfg)
    obj.set_initial_pose(Pose(position_xyz=tuple(float(v) for v in item.pos), rotation_xyzw=tuple(item.rot_xyzw)))
    return obj


def task_objects(task, layout: Layout) -> tuple[list[Object], list[str]]:
    """The task's Arena objects, and the names of those built by ``isaac_objects()``."""
    out = []
    for spec in task.objects:
        if isinstance(spec, UsdBody):
            out.append(_usd_object(spec, task.folder))
        elif isinstance(spec, Shape):
            out.append(_shape_object(spec))
        else:
            out.append(yscene.task_object(spec))  # yam_arena's (or a task-folder scenekit's, which patches it)
    extra = list(task.call_layout("isaac_objects", layout) or [])
    names = {o.name for o in out}
    from yam_pair.taskfile import RESERVED_NAMES

    for obj in extra:
        if obj.name in names or obj.name in RESERVED_NAMES:
            raise ValueError(f"isaac_objects(): {obj.name!r} is already taken")
        names.add(obj.name)
    return out + extra, [o.name for o in extra]


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def body_state(env, item) -> BodyState:
    if isinstance(item, Shape):
        half = np.asarray(item.size) / 2
        pos = np.asarray(item.pos, float)
        return BodyState(pos=pos, quat_xyzw=np.asarray(item.rot_xyzw, float), lin_vel=np.zeros(3),
                         ang_vel=np.zeros(3), low=pos - half, high=pos + half)
    base = env.unwrapped
    asset = base.scene[item.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0]
    joints = {}
    if item.articulation:
        q = _np(asset.data.joint_pos)[0]
        joints = {n: float(q[i]) for i, n in enumerate(asset.joint_names)}
    pos = pose[:3] - origin
    return BodyState(pos=pos, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=pos, high=pos,
                     joints=joints)


def object_states(env, task, extra_names) -> dict:
    own = {o.name: o for o in task.objects if isinstance(o, UsdBody | Shape)}
    plain = [n for n in (*task.names, *extra_names) if n not in own]
    out = yscene.object_states(env, plain) if plain else {}
    out.update({n: body_state(env, item) for n, item in own.items()})
    return out


# -- the environment ---------------------------------------------------------------------------------------------------
def env_cfg_callback(physics_hz: float, decimation: int, eye, lookat):
    def configure(env_cfg):
        env_cfg.sim.dt = 1.0 / physics_hz
        env_cfg.decimation = decimation
        env_cfg.sim.render_interval = decimation  # one render per control step, after its last substep
        env_cfg.terminations = make_configclass("TerminationsCfg", [])()
        env_cfg.recorders = RecorderManagerBaseCfg(dataset_export_mode=DatasetExportMode.EXPORT_NONE)
        env_cfg.metrics = None
        env_cfg.sim.default_visualizer_cfg = VisualizerCfg(eye=eye, lookat=lookat)
        return env_cfg

    return configure


def physics_hz_for(task) -> float:
    """The task's PHYSICS_HZ, or a task-folder scenekit's physics(hz) when the task set no PHYSICS_HZ."""
    if not hasattr(task.module, "PHYSICS_HZ"):
        sk = sys.modules.get("scenekit")
        dt = getattr(sk, "_PHYSICS", {}).get("dt") if sk else None
        if dt:
            return 1.0 / dt
    return task.physics_hz


def make_env(task, layout: Layout, *, control_hz: float, device: str, overview: tuple[int, int] | None):
    """Build, register and make the four-arm Arena env. Returns (env, FourArmEmbodiment, isaac_object_names)."""
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg

    physics_hz = physics_hz_for(task)
    ratio = physics_hz / control_hz
    decimation = round(ratio)
    if abs(ratio - decimation) > 1e-6 or decimation < 1:
        raise ValueError(f"PHYSICS_HZ ({physics_hz:g}) must be a whole multiple of control_hz ({control_hz:g})")
    table = table_geometry(task, layout)
    center = table["center"] if table else (0.0, 0.0)
    emb = FourArmEmbodiment(layout, decimation, overview, center)
    objects, extra = task_objects(task, layout)
    eye, lookat = overview_pose(layout, center)
    gym_name = "yam_pair_" + re.sub(r"[^A-Za-z0-9_]", "_", task.name)
    definition = IsaacLabArenaEnvironment(name=gym_name, scene=Scene([*room(task, layout, emb.rig.top_cam), *objects]),
                                          embodiment=emb, env_cfg_callback=env_cfg_callback(physics_hz, decimation,
                                                                                            eye, lookat))
    env = ArenaEnvBuilder(definition, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)).make_registered()
    return env, emb, extra


def read_contacts(base) -> dict[str, float]:
    """Each arm's largest normal force against any link of the other station over the last control step (N)."""
    import torch

    out = {}
    for s in STATIONS:
        for side in SIDES:
            data = base.scene[f"{s}_{side}_contacts"].data
            force = data.normal_force_matrix_w_history
            if force is None:
                force = data.normal_force_matrix_w
            if force is None:
                out[f"{s}_{side}"] = 0.0
                continue
            t = getattr(force, "torch", force)
            out[f"{s}_{side}"] = float(torch.linalg.vector_norm(t[0], dim=-1).max().item()) if t.numel() else 0.0
    return out


# -- joint helpers for task hooks (scenekit's, for any articulation in the scene) -----------------------------------------
def write_joint_positions(env, name: str, positions: dict) -> None:
    """Move an articulation's joints to these positions (by joint name), at rest."""
    import torch

    asset = env.unwrapped.scene[name]
    ids = asset.find_joints(list(positions), preserve_order=True)[0]
    value = torch.tensor([list(positions.values())], dtype=torch.float32, device=asset.device)
    asset.write_joint_position_to_sim_index(position=value, joint_ids=ids)
    asset.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(value), joint_ids=ids)


def set_joint_targets(env, name: str, targets: dict) -> None:
    """Position targets for an articulation's driven joints (see ``UsdBody.drives``), by joint name."""
    import torch

    asset = env.unwrapped.scene[name]
    ids = asset.find_joints(list(targets), preserve_order=True)[0]
    value = torch.tensor([list(targets.values())], dtype=torch.float32, device=asset.device)
    asset.actuators.target_command.set_position_index(value=value, joint_ids=ids)


def restore_usd_joints(env, task) -> None:
    """Every UsdBody articulation's ``joint_pos`` back at rest (a reset moves an articulation only by its root)."""
    for item in task.objects:
        if isinstance(item, UsdBody) and item.articulation and item.joint_pos:
            joints = dict(item.joint_pos)
            write_joint_positions(env, item.name, joints)
            set_joint_targets(env, item.name, joints)
