"""Native USD/PhysX joints between task bodies, outside the robot articulation (reusable kit module).

Each joint is created disabled. The embodiment enables the set only after the start layout has been written, so
a reset never fights a joint (Isaac Lab writes each body's pose separately). Joints are excluded from
articulations, so they can close kinematic loops (slider-crank, four-bar) between plain rigid bodies.

Anchors: ``pos0``/``rot0`` are in body0's frame (the world frame, including the env origin, when body0 is None);
``pos1``/``rot1`` in body1's frame. With identity rotations the joint axis is the named axis of both frames.
Collisions between the two joined bodies are off. pxr is imported inside the methods: importing it before
Isaac Sim starts loads a different USD build and breaks Kit's extensions.

Adapted from the ratchet-lowering task plugin (robocurve/benchmark-tasks PR #538, MIT, Copyright (c) 2026 YAMbition
contributors; full notice in THIRD_PARTY_NOTICES.txt).
"""

from __future__ import annotations

ENV = "/World/envs/env_0"


class JointSet:
    def __init__(self, stage, root: str = "/World/MechanismJoints"):
        self.stage, self.root = stage, root
        self.joints: dict = {}
        self.drives: dict = {}

    def _define(self, cls, name, body0, body1, pos0, pos1=(0.0, 0.0, 0.0), rot0=None, rot1=None):
        from pxr import Gf, Sdf

        j = cls.Define(self.stage, f"{self.root}/{name}")
        j.CreateJointEnabledAttr(False)
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, pos1)))
        j.CreateLocalRot0Attr(Gf.Quatf(*rot0) if rot0 is not None else Gf.Quatf(1.0))  # (w, x, y, z)
        j.CreateLocalRot1Attr(Gf.Quatf(*rot1) if rot1 is not None else Gf.Quatf(1.0))
        j.CreateCollisionEnabledAttr(False)
        j.CreateExcludeFromArticulationAttr(True)
        if body0:
            j.CreateBody0Rel().SetTargets([Sdf.Path(f"{ENV}/{body0}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{ENV}/{body1}")])
        self.joints[name] = j
        return j

    def revolute(self, name, body0, body1, pos0, pos1=(0.0, 0.0, 0.0), axis="Z", limits_deg=None, **kw):
        from pxr import UsdPhysics

        j = self._define(UsdPhysics.RevoluteJoint, name, body0, body1, pos0, pos1, **kw)
        j.CreateAxisAttr(axis)
        if limits_deg is not None:
            j.CreateLowerLimitAttr(float(limits_deg[0]))
            j.CreateUpperLimitAttr(float(limits_deg[1]))
        return j

    def prismatic(self, name, body0, body1, pos0, pos1=(0.0, 0.0, 0.0), axis="X", limits_m=None, **kw):
        from pxr import UsdPhysics

        j = self._define(UsdPhysics.PrismaticJoint, name, body0, body1, pos0, pos1, **kw)
        j.CreateAxisAttr(axis)
        if limits_m is not None:
            j.CreateLowerLimitAttr(float(limits_m[0]))
            j.CreateUpperLimitAttr(float(limits_m[1]))
        return j

    def spherical(self, name, body0, body1, pos0, pos1=(0.0, 0.0, 0.0), **kw):
        from pxr import UsdPhysics

        return self._define(UsdPhysics.SphericalJoint, name, body0, body1, pos0, pos1, **kw)

    def fixed(self, name, body0, body1, pos0, pos1=(0.0, 0.0, 0.0), **kw):
        from pxr import UsdPhysics

        return self._define(UsdPhysics.FixedJoint, name, body0, body1, pos0, pos1, **kw)

    def drag(self, name, damping, kind="angular", stiffness=0.0, target=0.0, max_force=None):
        """A declared drive on a joint: pure damping (bearing drag) by default, or a spring with ``stiffness``.
        Angular units are per degree (USD): damping in N m per deg/s, stiffness in N m per deg."""
        from pxr import UsdPhysics

        d = UsdPhysics.DriveAPI.Apply(self.joints[name].GetPrim(), kind)
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(float(stiffness))
        d.CreateDampingAttr(float(damping))
        d.CreateTargetPositionAttr(float(target))
        d.CreateTargetVelocityAttr(0.0)
        if max_force is not None:
            d.CreateMaxForceAttr(float(max_force))
        self.drives[name] = d
        return d

    def enable(self, on: bool = True) -> None:
        for j in self.joints.values():
            j.GetJointEnabledAttr().Set(bool(on))
