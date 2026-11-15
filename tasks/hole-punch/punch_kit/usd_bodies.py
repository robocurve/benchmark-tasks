"""Generic USD rigid-body builder for mechanism parts (reusable kit module). Import after Isaac Sim starts.

One :class:`Body` is one rigid body written to its own .usda file, made of primitive shapes (boxes, cylinders,
convex prisms). Each shape either collides (PhysX collider with a physics material) or is visual only. Mass,
centre of mass and inertia can be declared, which a body needs when some of its shapes are visual only (PhysX
would otherwise derive them from the colliders alone). Files go to a per-process temporary folder, so nothing
generated lands in the task folder.

Adapted from the ratchet-lowering task plugin (robocurve/benchmark-tasks PR #538, MIT, Copyright (c) 2026 YAMbition
contributors; full notice in THIRD_PARTY_NOTICES.txt).
"""

from __future__ import annotations

import math
import os
import tempfile
from pathlib import Path

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

OUT_DIR = Path(tempfile.mkdtemp(prefix=f"mechanism_usd_{os.getpid()}_"))


def linear_rgb(c):
    """sRGB (what the camera shows) to the linear colour USD shaders take."""
    return tuple(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in c)


class Body:
    def __init__(self, name: str, *, mass: float | None = None, friction=(0.6, 0.5), com=None, inertia=None,
                 kinematic: bool = False, solver_iters: tuple[int, int] = (16, 4), contact_offset: float = 0.001):
        self.name = name
        self.path = OUT_DIR / f"{name}.usda"
        self.stage = Usd.Stage.CreateNew(str(self.path))
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        root = UsdGeom.Xform.Define(self.stage, "/Part")
        self.stage.SetDefaultPrim(root.GetPrim())
        prim = root.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(bool(kinematic))
        if mass is not None:
            m = UsdPhysics.MassAPI.Apply(prim)
            m.CreateMassAttr(float(mass))
            if com is not None:
                m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
            if inertia is not None:
                m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
                m.CreatePrincipalAxesAttr(Gf.Quatf(1.0))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr(int(solver_iters[0]))
        rb.CreateSolverVelocityIterationCountAttr(int(solver_iters[1]))
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        self.physics = UsdShade.Material.Define(self.stage, "/Part/physics_material")
        pm = UsdPhysics.MaterialAPI.Apply(self.physics.GetPrim())
        pm.CreateStaticFrictionAttr(float(friction[0]))
        pm.CreateDynamicFrictionAttr(float(friction[1]))
        pm.CreateRestitutionAttr(0.0)
        self.contact_offset = contact_offset
        self._looks: dict = {}
        self._n = 0

    def _look(self, color):
        color = tuple(float(c) for c in color)
        if color not in self._looks:
            mat = UsdShade.Material.Define(self.stage, f"/Part/look_{len(self._looks)}")
            sh = UsdShade.Shader.Define(self.stage, mat.GetPath().AppendChild("shader"))
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(color)))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            self._looks[color] = mat
        return self._looks[color]

    def _finish(self, prim, color, collide, mesh=False):
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            if mesh:
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            props = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            props.CreateContactOffsetAttr(float(self.contact_offset))
            props.CreateRestOffsetAttr(0.0)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.physics, materialPurpose="physics")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(self._look(color))

    def box(self, center, size, color, yaw: float = 0.0, collide: bool = True):
        """Axis-aligned box in the body frame, optionally turned ``yaw`` (rad) about the body's +z."""
        self._n += 1
        shape = UsdGeom.Cube.Define(self.stage, f"/Part/box_{self._n}")
        shape.CreateSizeAttr(1.0)
        shape.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        if yaw:
            shape.AddRotateZOp().Set(math.degrees(yaw))
        shape.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        self._finish(shape.GetPrim(), color, collide)

    def cylinder(self, center, radius: float, height: float, color, axis: str = "Z", collide: bool = True):
        self._n += 1
        shape = UsdGeom.Cylinder.Define(self.stage, f"/Part/cyl_{self._n}")
        shape.CreateAxisAttr(axis)
        shape.CreateRadiusAttr(float(radius))
        shape.CreateHeightAttr(float(height))
        shape.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        self._finish(shape.GetPrim(), color, collide)

    def prism_z(self, xy_points, z0: float, z1: float, color, collide: bool = True):
        """Convex prism: a polygon in the x-y plane extruded along z (convex-hull collider)."""
        self._n += 1
        mesh = UsdGeom.Mesh.Define(self.stage, f"/Part/prism_{self._n}")
        n = len(xy_points)
        pts = [Gf.Vec3f(float(x), float(y), float(z0)) for x, y in xy_points] + \
              [Gf.Vec3f(float(x), float(y), float(z1)) for x, y in xy_points]
        counts, idx = [n, n], list(range(n))[::-1] + list(range(n, 2 * n))
        for i in range(n):
            j = (i + 1) % n
            counts.append(4)
            idx += [i, j, n + j, n + i]
        mesh.CreatePointsAttr(pts)
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(idx)
        mesh.CreateSubdivisionSchemeAttr("none")
        self._finish(mesh.GetPrim(), color, collide, mesh=True)

    def save(self) -> Path:
        self.stage.GetRootLayer().Save()
        return self.path
