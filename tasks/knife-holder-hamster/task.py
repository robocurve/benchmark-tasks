"""Local USD task adapter for the supplied, unmodified YAM station runtime.

The task assets and poses come from task_objects.usda. This adapter builds an
Arena environment; it is not a task file for yam_arena's primitive-only loader.
Call build_environment only after starting Isaac Sim.
"""

from dataclasses import dataclass, replace
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parent
INSTRUCTION = "Put the knife into the holder."
SCENE = "station"
CONTROL_HZ = 20
POLICY_CAMERAS = ("top_cam", "left_cam", "right_cam")
POLICY_SIZE = (320, 176)
CAMERA_VIEW = {"profile": "holder_visibility", "lower_m": 0.10, "tilt_up_deg": 4.0}
# Match YAM-inspect-Isaac/yam_sim/scene.py and arena/yam_tabletop/arena/environment.py.
LIGHTING = {
    "profile": "original_repository_dome",
    "dome_intensity": 2500.0,
    "dome_color": (0.9, 0.9, 0.9),
}


@dataclass(frozen=True)
class UsdObject:
    name: str
    asset: Path
    position: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]
    rigid: bool
    kinematic: bool

    @property
    def movable(self):
        return self.rigid and not self.kinematic


def camera_rig():
    """Small local top-camera adjustment; keep calibrated lenses and wrist mounts."""
    import math
    import numpy as np
    from yam_arena import ik, station
    from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

    rig = station.rig("calibrated")
    top = rig.top_cam
    rotation = quat_xyzw_to_matrix(np.array(top.rot_xyzw))
    rotation = rotation @ axis_angle_matrix(
        np.array([1.0, 0.0, 0.0]), math.radians(CAMERA_VIEW["tilt_up_deg"])
    )
    top = replace(
        top,
        pos=(top.pos[0], top.pos[1], top.pos[2] - CAMERA_VIEW["lower_m"]),
        rot_xyzw=tuple(float(v) for v in ik.matrix_to_quat_xyzw(rotation)),
    )
    return replace(rig, name="station-holder-visibility", top_cam=top)


def object_specs():
    """Read the authored references, transforms and motion types without editing USD."""
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(str(ROOT / "task_objects.usda"))
    if stage is None or not stage.GetDefaultPrim():
        raise ValueError("task_objects.usda has no valid default prim")
    specs = []
    for prim in stage.GetDefaultPrim().GetChildren():
        references = prim.GetMetadata("references")
        if references is None or len(references.prependedItems) != 1:
            raise ValueError(f"{prim.GetPath()}: expected one local asset reference")
        asset = (ROOT / references.prependedItems[0].assetPath).resolve()
        if not asset.is_relative_to(ROOT) or not asset.is_file():
            raise ValueError(f"Missing or nonlocal asset: {asset}")
        # The authored scene uses rigid transforms, not scaled object instances.
        transform = UsdGeom.Xformable(prim).GetLocalTransformation()
        from pxr import Gf

        rigid_transform = Gf.Transform(transform)
        if any(abs(float(s) - 1.0) > 1e-6 for s in rigid_transform.GetScale()):
            raise ValueError(f"{prim.GetPath()}: scaled instances are unsupported")
        q = rigid_transform.GetRotation().GetQuat()
        name = re.sub(r"(?<=[a-z])(?=[A-Z])", "_", prim.GetName()).lower()
        if name == "loose_knife":
            name = "knife"
        specs.append(UsdObject(
            name=name,
            asset=asset,
            position=tuple(float(v) for v in rigid_transform.GetTranslation()),
            rotation_xyzw=tuple(float(v) for v in (*q.GetImaginary(), q.GetReal())),
            rigid=prim.HasAPI(UsdPhysics.RigidBodyAPI),
            kinematic=bool(prim.GetAttribute("physics:kinematicEnabled").Get()),
        ))
    if len(specs) != 7 or {s.name for s in specs if s.movable} != {"hamster", "knife"}:
        raise ValueError("Expected the holder, four fixed knives, hamster and loose knife")
    return tuple(specs)


def collision_vertices(spec):
    """Root-local support vertices for our mesh/box compound colliders.

    Rotating an asset's bounding box can put its empty corners below the table.
    These vertices give the actual extreme heights of the authored convex hulls
    and boxes, without that bounding-box overestimate.
    """
    import itertools
    import numpy as np
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(str(spec.asset))
    root = stage.GetDefaultPrim()
    cache = UsdGeom.XformCache()
    root_inverse = cache.GetLocalToWorldTransform(root).GetInverse()
    result = []
    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if UsdGeom.Mesh(prim):
            approximation = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
            if approximation not in (None, "none", "convexHull"):
                raise ValueError(f"Unsupported collider approximation: {approximation}")
            vertices = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        elif UsdGeom.Cube(prim):
            half = UsdGeom.Cube(prim).GetSizeAttr().Get() / 2
            vertices = itertools.product((-half, half), repeat=3)
        else:
            raise ValueError(f"Unsupported collider geometry: {prim.GetPath()}")
        transform = cache.GetLocalToWorldTransform(prim) * root_inverse
        result.extend(tuple(transform.Transform(Gf.Vec3d(*v))) for v in vertices)
    if not result:
        raise ValueError(f"{spec.name} has no supported collision geometry")
    return np.asarray(result, dtype=float)


def build_environment(*, control_hz=CONTROL_HZ, enable_cameras=True):
    """Use the station rig with the original repository's neutral dome lighting."""
    import isaaclab.sim as sim_utils
    from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_library import DomeLight
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.embodiment import BimanualYamEmbodiment
    from yam_arena.scene import env_cfg_callback, task_room

    specs = object_specs()
    # Leave native resolution and lens settings at the supplied runtime defaults.
    embodiment = BimanualYamEmbodiment(
        enable_cameras=enable_cameras, contact_sensors=False, rig=camera_rig()
    )
    objects = [
        obj for obj in task_room(SCENE, embodiment.rig.top_cam)
        if not isinstance(obj, DomeLight) and obj.name != "lamp"
    ]
    objects.append(DomeLight(spawner_cfg=sim_utils.DomeLightCfg(
        intensity=LIGHTING["dome_intensity"], color=LIGHTING["dome_color"],
    )))
    for spec in specs:
        kwargs = dict(
            name=spec.name,
            object_type=ObjectType.RIGID if spec.movable else ObjectType.BASE,
            initial_pose=Pose(position_xyz=spec.position, rotation_xyzw=spec.rotation_xyzw),
        )
        if spec.kinematic:
            kwargs["spawner_cfg"] = sim_utils.UsdFileCfg(
                usd_path=str(spec.asset),
                rigid_props=PhysxRigidBodyPropertiesCfg(kinematic_enabled=True),
            )
        else:
            kwargs["usd_path"] = str(spec.asset)
        objects.append(Object(**kwargs))

    def configure(cfg):
        cfg = env_cfg_callback(control_hz)(cfg)
        cfg.sim.enable_scene_query_support = True
        cfg.sim.physics.enable_scene_query_support = True
        return cfg

    config = IsaacLabArenaEnvironment(
        name="knife_holder_hamster_local",
        scene=Scene(objects),
        embodiment=embodiment,
        env_cfg_callback=configure,
    )
    env = ArenaEnvBuilder(
        config, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device="cuda:0")
    ).make_registered()
    return env, specs
