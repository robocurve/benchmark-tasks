"""Local USD task adapter for the supplied, unmodified YAM station runtime.

The task assets and poses come from task_objects.usda. This adapter builds an
Arena environment; it is not a task file for yam_arena's primitive-only loader.
Call build_environment only after starting Isaac Sim.
"""

from dataclasses import dataclass
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parent
INSTRUCTION = "Scoop one meatball onto the empty plate."
SCENE = "station"
CONTROL_HZ = 20
POLICY_CAMERAS = ("top_cam", "left_cam", "right_cam")
POLICY_SIZE = (320, 176)
CAMERA_VIEW = {"profile": "calibrated", "lower_m": 0.0, "tilt_up_deg": 0.0}
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
    from yam_arena import station
    return station.rig("calibrated")


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
        name = prim.GetName()
        specs.append(UsdObject(
            name=name,
            asset=asset,
            position=tuple(float(v) for v in rigid_transform.GetTranslation()),
            rotation_xyzw=tuple(float(v) for v in (*q.GetImaginary(), q.GetReal())),
            rigid=prim.HasAPI(UsdPhysics.RigidBodyAPI),
            kinematic=bool(prim.GetAttribute("physics:kinematicEnabled").Get()),
        ))
    import json
    layout = json.loads((ROOT/"layout.json").read_text())
    expected = {"CleanPlate", "RawPlate", "MeatballBowl", "Plate", "CleanSpoon", "UsedSpoon"}
    expected.update(n for n in layout['objects'] if n.startswith('Meatball_'))
    expected.update(f"RawChicken_{i}" for i in range(1,6))
    if {s.name for s in specs} != expected or len(specs) != len(expected):
        raise ValueError("USD objects must match the authored tool, food and dish layout")
    import json
    import numpy as np
    layout = json.loads((ROOT/"layout.json").read_text())
    for spec in specs:
        row = layout["objects"][spec.name]
        if not np.allclose(spec.position, row["position"], atol=1e-6, rtol=0) or spec.asset.name != row["asset"]:
            raise ValueError(f"USD and layout.json disagree for {spec.name}; update the scene and metadata together")
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
            kwargs["spawner_cfg"] = sim_utils.UsdFileCfg(
                usd_path=str(spec.asset), activate_contact_sensors=spec.movable,
            )
        objects.append(Object(**kwargs))

    def configure(cfg):
        cfg = env_cfg_callback(control_hz)(cfg)
        cfg.sim.enable_scene_query_support = True
        cfg.sim.physics.enable_scene_query_support = True
        return cfg

    config = IsaacLabArenaEnvironment(
        name="clean_spoon_meatball_local",
        scene=Scene(objects),
        embodiment=embodiment,
        env_cfg_callback=configure,
    )
    env = ArenaEnvBuilder(
        config, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device="cuda:0")
    ).make_registered()
    return env, specs
