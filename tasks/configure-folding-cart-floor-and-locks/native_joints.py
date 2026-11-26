"""Author passive native PhysX constraints after the rigid boxes spawn."""

from dataclasses import dataclass

JOINTS = []


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    parent: str | None
    child: str
    anchor0: tuple
    anchor1: tuple
    rotation0: tuple = (0., 0., 0., 1.)
    rotation1: tuple = (0., 0., 0., 1.)
    axis: str = "Z"
    limits: tuple = (-180., 180.)
    collisions: bool = True
    spring: tuple | None = None


def install():
    from isaaclab.scene.interactive_scene import InteractiveScene

    if getattr(InteractiveScene, "_folding_cart_native_joints", False):
        return
    stock_init = InteractiveScene.__init__

    def initialize(self, *args, **kwargs):
        stock_init(self, *args, **kwargs)
        for env_path in self.env_prim_paths:
            author(self.stage, env_path)

    InteractiveScene.__init__ = initialize
    InteractiveScene._folding_cart_native_joints = True


def author(stage, env_path):
    from pxr import Gf, Sdf, UsdPhysics

    paths = {}
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if path.startswith(f"{env_path}/") and path.endswith("/body"):
            paths[path.split("/")[-2].lower()] = path

    def vector(point):
        return Gf.Vec3f(*(float(v) for v in point))

    def quat(rotation):
        x, y, z, w = map(float, rotation)
        return Gf.Quatf(w, x, y, z)

    for spec in JOINTS:
        if spec.child not in paths or (spec.parent and spec.parent not in paths):
            raise RuntimeError(f"Missing body for {spec.name}: available {sorted(paths)}")
        child = paths[spec.child]
        path = child.rsplit("/", 1)[0] + "/joints/" + spec.name
        schema = {
            "revolute": UsdPhysics.RevoluteJoint,
            "prismatic": UsdPhysics.PrismaticJoint,
            "fixed": UsdPhysics.FixedJoint,
        }[spec.kind]
        joint = schema.Define(stage, path)
        if spec.parent:
            joint.CreateBody0Rel().SetTargets([Sdf.Path(paths[spec.parent])])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(child)])
        joint.CreateLocalPos0Attr().Set(vector(spec.anchor0))
        joint.CreateLocalPos1Attr().Set(vector(spec.anchor1))
        joint.CreateLocalRot0Attr().Set(quat(spec.rotation0))
        joint.CreateLocalRot1Attr().Set(quat(spec.rotation1))
        joint.CreateCollisionEnabledAttr().Set(spec.collisions)
        joint.CreateExcludeFromArticulationAttr().Set(True)
        if spec.kind != "fixed":
            joint.CreateAxisAttr().Set(spec.axis)
            joint.CreateLowerLimitAttr().Set(float(spec.limits[0]))
            joint.CreateUpperLimitAttr().Set(float(spec.limits[1]))
        if spec.spring is not None:
            stiffness, damping, rest = spec.spring
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular" if spec.kind == "revolute" else "linear")
            drive.CreateTypeAttr().Set("force")
            drive.CreateStiffnessAttr().Set(stiffness)
            drive.CreateDampingAttr().Set(damping)
            drive.CreateTargetPositionAttr().Set(rest)
