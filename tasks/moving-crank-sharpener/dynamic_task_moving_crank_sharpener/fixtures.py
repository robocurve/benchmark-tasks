"""Native rigid-body hinges and spring sliders for moving tools and fixtures."""

import math


def world_guide_settings(description):
    """Describe a world anchor, slider in metres, or hinge in degrees."""
    kind = description.get("kind", "prismatic")
    if kind == "fixed":
        if {"axis", "lower", "upper"} & description.keys():
            raise ValueError("a fixed world anchor cannot specify an axis or travel limits")
        return kind, None, None, None
    axis = description.get("axis", "Y" if kind == "prismatic" else "Z")
    lower, upper = float(description["lower"]), float(description["upper"])
    if kind not in ("prismatic", "revolute") or axis not in ("X", "Y", "Z"):
        raise ValueError("world guide requires a prismatic/revolute kind and X/Y/Z axis")
    if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
        raise ValueError("world guide limits must be finite and increasing")
    return kind, axis, lower, upper


def attach_world_guide(body, description, position, yaw=0.):
    """Constrain a real rigid support to the fixed machine frame before reset."""
    from pxr import Gf, Sdf, UsdPhysics
    kind, axis, lower, upper = world_guide_settings(description)
    cls = {"fixed": UsdPhysics.FixedJoint, "prismatic": UsdPhysics.PrismaticJoint,
           "revolute": UsdPhysics.RevoluteJoint}[kind]
    path = str(body.GetPath())
    joint = cls.Define(body.GetStage(), path+"_guide")
    joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float, position)))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(math.cos(yaw/2), 0, 0, math.sin(yaw/2)))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1))
    if axis is not None:
        joint.CreateAxisAttr(axis)
        joint.CreateLowerLimitAttr(lower)
        joint.CreateUpperLimitAttr(upper)
    return joint


def joint_break_limits(description):
    """Optional native break thresholds in newtons and newton metres.

    Omitting a threshold retains USD's unbreakable default. Explicit limits
    must be positive and finite; this prevents a typo silently breaking a
    fixture on the first physics step.
    """
    result = {}
    for key in ("break_force_N", "break_torque_Nm"):
        if key not in description:
            continue
        value = float(description[key])
        if not math.isfinite(value) or value <= 0:
            raise ValueError("joint break thresholds must be positive and finite")
        result[key] = value
    return result


def attach_fixture_joint(child_prim, description, objects):
    """Author a passive or spring-loaded joint during spawn, before physics starts.

    The anchor is a world point at reset. Axis is in the parent's initial frame.
    Revolute limits/drive positions use degrees; prismatic values use metres.
    Every body continues to move through PhysX contacts and constraints.
    """
    limits = joint_break_limits(description)
    collision_enabled = description.get("collision_enabled", False)
    if not isinstance(collision_enabled, bool):
        raise ValueError("fixture collision_enabled must be a boolean")
    enabled = description.get("enabled", True)
    if not isinstance(enabled, bool):
        raise ValueError("fixture enabled must be a boolean")
    from pxr import Gf, Sdf, UsdPhysics

    kind = description["kind"]
    if kind not in ("revolute", "prismatic", "fixed", "spherical"):
        raise ValueError("unsupported fixture joint kind")
    parent = objects[description["parent"]]
    child = objects[child_prim.GetName()]
    anchor = tuple(float(x) for x in description["anchor"])
    if len(anchor) != 3 or not all(math.isfinite(x) for x in anchor):
        raise ValueError("fixture joint anchor must contain three finite coordinates")
    path = str(child_prim.GetPath())
    parent_path = path.rsplit("/", 1)[0]+"/"+parent.name
    joint_path = path+"_joint"
    cls = {"revolute": UsdPhysics.RevoluteJoint, "prismatic": UsdPhysics.PrismaticJoint,
           "fixed": UsdPhysics.FixedJoint, "spherical": UsdPhysics.SphericalJoint}[kind]
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
    joint.CreateCollisionEnabledAttr().Set(collision_enabled)
    if not enabled:
        joint.CreateJointEnabledAttr().Set(False)
    if "break_force_N" in limits:
        joint.CreateBreakForceAttr().Set(limits["break_force_N"])
    if "break_torque_Nm" in limits:
        joint.CreateBreakTorqueAttr().Set(limits["break_torque_Nm"])
    if kind != "fixed":
        axis = description.get("axis", "X")
        if axis not in ("X", "Y", "Z"):
            raise ValueError("fixture joint axis must be X, Y or Z")
        joint.CreateAxisAttr(axis)
        if kind == "spherical":
            swing = float(description["swing_limit_degrees"])
            if not math.isfinite(swing) or not 0 < swing < 180:
                raise ValueError("spherical joint swing limit must be between 0 and 180 degrees")
            if {"lower", "upper", "drive"} & description.keys():
                raise ValueError("spherical joints require a passive cone limit")
            joint.CreateConeAngle0LimitAttr(swing)
            joint.CreateConeAngle1LimitAttr(swing)
            return joint
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
