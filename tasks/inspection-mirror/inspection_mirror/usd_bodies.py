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
                 kinematic: bool = False, solver_iters: tuple[int, int] = (16, 4), contact_offset: float = 0.001,
                 torsional_patch: float | None = None):
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
        self.torsional_patch = torsional_patch  # m: contact patch radius for torsional friction (spin about the normal)
        self._looks: dict = {}
        self._n = 0

    def _look(self, color, metallic: float = 0.0, roughness: float = 0.5):
        key = (tuple(float(c) for c in color), float(metallic), float(roughness))
        if key not in self._looks:
            mat = UsdShade.Material.Define(self.stage, f"/Part/look_{len(self._looks)}")
            sh = UsdShade.Shader.Define(self.stage, mat.GetPath().AppendChild("shader"))
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(key[0])))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(key[2])
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(key[1])
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            self._looks[key] = mat
        return self._looks[key]

    def mirror(self, center, size, yaw: float = 0.0, collide: bool = True):
        """A flat mirror: a box with a metallic, perfectly smooth surface (path tracing renders its reflection)."""
        self._mirror_next = True
        self.box(center, size, (0.95, 0.95, 0.95), yaw=yaw, collide=collide)

    def mirror_disc(self, center, radius: float, thickness: float, yaw: float = 0.0, collide: bool = True,
                    color=None):
        """A round flat mirror: a thin disc (axis along body x before the yaw) with a perfectly smooth metal look.
        With ``color``, the same disc with a plain matte look instead (a mirror's back)."""
        if color is not None:
            self._n += 1
            shape = UsdGeom.Cylinder.Define(self.stage, f"/Part/cyl_{self._n}")
            shape.CreateAxisAttr("X")
            shape.CreateRadiusAttr(float(radius))
            shape.CreateHeightAttr(float(thickness))
            shape.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
            if yaw:
                shape.AddRotateZOp().Set(math.degrees(yaw))
            self._finish(shape.GetPrim(), color, collide)
            return
        self._n += 1
        shape = UsdGeom.Cylinder.Define(self.stage, f"/Part/cyl_{self._n}")
        shape.CreateAxisAttr("X")
        shape.CreateRadiusAttr(float(radius))
        shape.CreateHeightAttr(float(thickness))
        shape.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        if yaw:
            shape.AddRotateZOp().Set(math.degrees(yaw))
        self._mirror_next = True
        self._finish(shape.GetPrim(), (0.95, 0.95, 0.95), collide)

    def _finish(self, prim, color, collide, mesh=False):
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            if mesh:
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            props = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            props.CreateContactOffsetAttr(float(self.contact_offset))
            props.CreateRestOffsetAttr(0.0)
            if self.torsional_patch:
                props.CreateTorsionalPatchRadiusAttr(float(self.torsional_patch))
                props.CreateMinTorsionalPatchRadiusAttr(float(self.torsional_patch))
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.physics, materialPurpose="physics")
        if getattr(self, "_mirror_next", False):
            self._mirror_next = False
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self._look(color, metallic=1.0, roughness=0.0))
        else:
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

    def prism_x(self, yz_points, x0: float, x1: float, color, collide: bool = True):
        """Convex prism: a polygon in the y-z plane extruded along x (convex-hull collider)."""
        self._n += 1
        mesh = UsdGeom.Mesh.Define(self.stage, f"/Part/prismx_{self._n}")
        n = len(yz_points)
        pts = [Gf.Vec3f(float(x0), float(y), float(z)) for y, z in yz_points] + \
              [Gf.Vec3f(float(x1), float(y), float(z)) for y, z in yz_points]
        counts, idx = [n, n], list(range(n)) + list(range(n, 2 * n))[::-1]
        for i in range(n):
            j = (i + 1) % n
            counts.append(4)
            idx += [i, n + i, n + j, j]
        mesh.CreatePointsAttr(pts)
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(idx)
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        self._finish(mesh.GetPrim(), color, collide, mesh=True)

    def prism_y(self, xz_points, y0: float, y1: float, color, collide: bool = True):
        """Convex prism: a polygon in the x-z plane extruded along y (convex-hull collider)."""
        self._n += 1
        mesh = UsdGeom.Mesh.Define(self.stage, f"/Part/prismy_{self._n}")
        n = len(xz_points)
        pts = [Gf.Vec3f(float(x), float(y0), float(z)) for x, z in xz_points] + \
              [Gf.Vec3f(float(x), float(y1), float(z)) for x, z in xz_points]
        counts, idx = [n, n], list(range(n)) + list(range(n, 2 * n))[::-1]
        for i in range(n):
            j = (i + 1) % n
            counts.append(4)
            idx += [i, n + i, n + j, j]
        mesh.CreatePointsAttr(pts)
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(idx)
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        self._finish(mesh.GetPrim(), color, collide, mesh=True)

    def light(self, center, intensity: float, radius: float = 0.003, color=(1.0, 0.97, 0.92)):
        """A small sphere light that moves with the body (e.g. the LED of a lighted inspection mirror)."""
        from pxr import UsdLux

        self._n += 1
        lt = UsdLux.SphereLight.Define(self.stage, f"/Part/light_{self._n}")
        lt.CreateIntensityAttr(float(intensity))
        lt.CreateRadiusAttr(float(radius))
        lt.CreateColorAttr(Gf.Vec3f(*map(float, color)))
        # light the scene but do not draw the bulb itself (it sat in the camera's view and its glare hid the mirror)
        lt.GetPrim().CreateAttribute("visibleInPrimaryRay", Sdf.ValueTypeNames.Bool).Set(False)
        UsdGeom.Xformable(lt.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))

    def save(self) -> Path:
        self.stage.GetRootLayer().Save()
        return self.path
