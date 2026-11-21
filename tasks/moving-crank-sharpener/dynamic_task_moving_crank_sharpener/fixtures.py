"""Native rigid-body hinges and spring sliders for moving tools and fixtures."""

import math


def attach_fixture_joint(child_prim, description, objects):
    """Author a passive or spring-loaded joint during spawn, before physics starts.

    The anchor is a world point at reset. Axis is in the parent's initial frame.
    Revolute limits/drive positions use degrees; prismatic values use metres.
    Every body continues to move through PhysX contacts and constraints.
    """
    from pxr import Gf, Sdf, UsdPhysics

    kind = description["kind"]
    if kind not in ("revolute", "prismatic", "fixed"):
        raise ValueError("fixture joint must be revolute, prismatic or fixed")
    parent = objects[description["parent"]]
    child = objects[child_prim.GetName()]
    anchor = tuple(float(x) for x in description["anchor"])
    if len(anchor) != 3 or not all(math.isfinite(x) for x in anchor):
        raise ValueError("fixture joint anchor must contain three finite coordinates")
    path = str(child_prim.GetPath())
    parent_path = path.rsplit("/", 1)[0]+"/"+parent.name
    joint_path = path+"_joint"
    cls = {"revolute": UsdPhysics.RevoluteJoint, "prismatic": UsdPhysics.PrismaticJoint,
           "fixed": UsdPhysics.FixedJoint}[kind]
    joint = cls.Define(child_prim.GetStage(), joint_path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([child_prim.GetPath()])

    def local(point, body):
        x, y, z = (a-b for a, b in zip(point, body.pos))
        c, s = math.cos(body.yaw), math.sin(body.yaw)
        return Gf.Vec3f(c*x+s*y, -s*x+c*y, z)

    joint.CreateLocalPos0Attr().Set(local(anchor, parent))
    joint.CreateLocalPos1Attr().Set(local(anchor, child))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1))
    angle = parent.yaw-child.yaw
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(math.cos(angle/2), 0, 0, math.sin(angle/2)))
    joint.CreateCollisionEnabledAttr().Set(False)
    if kind != "fixed":
        axis = description.get("axis", "X")
        if axis not in ("X", "Y", "Z"):
            raise ValueError("fixture joint axis must be X, Y or Z")
        joint.CreateAxisAttr(axis)
        if "lower" in description:
            joint.CreateLowerLimitAttr(float(description["lower"]))
        if "upper" in description:
            joint.CreateUpperLimitAttr(float(description["upper"]))
        drive = description.get("drive")
        if drive:
            api = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular" if kind == "revolute" else "linear")
            api.CreateTypeAttr("force")
            api.CreateStiffnessAttr(float(drive.get("stiffness", 0)))
            api.CreateDampingAttr(float(drive.get("damping", 0)))
            api.CreateMaxForceAttr(float(drive["max_force"]))
            api.CreateTargetPositionAttr(float(drive.get("target", 0)))
            api.CreateTargetVelocityAttr(0.0)
    return joint
