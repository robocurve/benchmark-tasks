"""Small USD geometry helpers for custom fixture parts."""

import math

import numpy as np

ROOT = "/World/envs/env_0/BenchmarkFixture"


class Geometry:
    def __init__(self):
        import omni.usd
        from pxr import Gf, UsdGeom, Vt

        self.stage = omni.usd.get_context().get_stage()
        self.gf, self.usd, self.vt = Gf, UsdGeom, Vt
        self.ops = {}
        self.stage.RemovePrim(ROOT)
        UsdGeom.Xform.Define(self.stage, ROOT)

    def group(self, name, parent=ROOT):
        path = f"{parent}/{name}"
        model = self.usd.Xform.Define(self.stage, path)
        self.ops[path] = (model.AddTranslateOp(), model.AddRotateXYZOp())
        return path

    def move(self, path, xyz, yaw=0.0):
        translation, rotation = self.ops[path]
        translation.Set(self.gf.Vec3d(*map(float, xyz)))
        rotation.Set(self.gf.Vec3f(0, 0, math.degrees(float(yaw))))

    def box(self, name, xyz, size, color, parent=ROOT):
        obj = self.usd.Cube.Define(self.stage, f"{parent}/{name}")
        obj.CreateSizeAttr(1.0)
        obj.AddTranslateOp().Set(self.gf.Vec3d(*map(float, xyz)))
        obj.AddScaleOp().Set(self.gf.Vec3f(*map(float, size)))
        self.color(obj, color)
        return obj

    def cylinder(self, name, xyz, radius, height, color, parent=ROOT, axis="Z"):
        obj = self.usd.Cylinder.Define(self.stage, f"{parent}/{name}")
        obj.AddTranslateOp().Set(self.gf.Vec3d(*map(float, xyz)))
        obj.CreateRadiusAttr(radius)
        obj.CreateHeightAttr(height)
        obj.CreateAxisAttr(axis)
        self.color(obj, color)
        return obj

    def sphere(self, name, xyz, radii, color, parent=ROOT):
        obj = self.usd.Sphere.Define(self.stage, f"{parent}/{name}")
        obj.AddTranslateOp().Set(self.gf.Vec3d(*map(float, xyz)))
        obj.AddScaleOp().Set(self.gf.Vec3f(*map(float, radii)))
        obj.CreateRadiusAttr(1.0)
        self.color(obj, color)
        return obj

    def mesh(self, name, points, faces, color, parent=ROOT):
        obj = self.usd.Mesh.Define(self.stage, f"{parent}/{name}")
        obj.CreateSubdivisionSchemeAttr("none")
        obj.CreateDoubleSidedAttr(True)
        obj.CreateFaceVertexCountsAttr([len(face) for face in faces])
        obj.CreateFaceVertexIndicesAttr([vertex for face in faces for vertex in face])
        self.points(obj, points)
        self.color(obj, color)
        return obj

    def points(self, mesh, points):
        mesh.CreatePointsAttr().Set(self.vt.Vec3fArray.FromNumpy(np.asarray(points, dtype=np.float32)))

    def color(self, obj, color):
        obj.CreateDisplayColorAttr([self.gf.Vec3f(*map(float, color))])

    def show(self, obj, visible):
        obj.CreateVisibilityAttr().Set("inherited" if visible else "invisible")
