"""Shared original procedural geometry helpers, reused from the drain trainer."""
from pathlib import Path
import json
import math
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Gf, Sdf

ROOT = '/MedicalStation'


class Geometry:
    def __init__(self, stage):
        self.stage = stage
        self.materials = {}

    def material(self, name, rgb, roughness=.55, metallic=0.):
        m = UsdShade.Material.Define(self.stage, ROOT + '/Looks/' + name)
        sh = UsdShade.Shader.Define(self.stage, str(m.GetPath()) + '/Shader')
        sh.CreateIdAttr('UsdPreviewSurface')
        # Inputs use linear RGB; supplied palette is sRGB.
        color = [v / 12.92 if v <= .04045 else ((v + .055) / 1.055)**2.4 for v in rgb]
        sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
        sh.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metallic)
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
        self.materials[name] = m

    def finish(self, obj, material, collide=False):
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(self.materials[material])
        if collide:
            UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
            prim = obj.GetPrim()
            prim.AddAppliedSchema('PhysxCollisionAPI')
            prim.CreateAttribute('physxCollision:contactOffset', Sdf.ValueTypeNames.Float).Set(.001)
            prim.CreateAttribute('physxCollision:restOffset', Sdf.ValueTypeNames.Float).Set(0.)
        return obj

    def box(self, path, pos, size, material, collide=False):
        g = UsdGeom.Cube.Define(self.stage, ROOT + '/' + path)
        g.CreateSizeAttr(1.)
        g.AddTranslateOp().Set(Gf.Vec3d(*pos))
        g.AddScaleOp().Set(Gf.Vec3f(*size))
        return self.finish(g, material, collide)

    def cylinder(self, path, pos, radius, height, material, collide=False):
        g = UsdGeom.Cylinder.Define(self.stage, ROOT + '/' + path)
        g.CreateRadiusAttr(radius)
        g.CreateHeightAttr(height)
        g.CreateAxisAttr('Z')
        g.AddTranslateOp().Set(Gf.Vec3d(*pos))
        return self.finish(g, material, collide)

    def mesh(self, path, points, faces, material):
        g = UsdGeom.Mesh.Define(self.stage, ROOT + '/' + path)
        g.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
        g.CreateFaceVertexCountsAttr([len(f) for f in faces])
        g.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
        g.CreateSubdivisionSchemeAttr('none')
        g.CreateDoubleSidedAttr(True)
        return self.finish(g, material)

    def tube(self, path, points, radius, material, sides=12):
        points = [Gf.Vec3d(*p) for p in points]
        vertices = []
        for i, p in enumerate(points):
            t = points[min(i + 1, len(points)-1)] - points[max(i - 1, 0)]
            t.Normalize()
            v = Gf.Cross(t, Gf.Vec3d(0, 0, 1))
            if v.GetLength() < .01:
                v = Gf.Cross(t, Gf.Vec3d(0, 1, 0))
            v.Normalize()
            w = Gf.Cross(t, v)
            for k in range(sides):
                a = 2 * math.pi * k / sides
                vertices.append(p + radius * (math.cos(a)*v + math.sin(a)*w))
        faces = [(j*sides+k, j*sides+(k+1)%sides,
                  (j+1)*sides+(k+1)%sides, (j+1)*sides+k)
                 for j in range(len(points)-1) for k in range(sides)]
        return self.mesh(path, vertices, faces, material)

    def ring(self, path, center, radius, thickness, material):
        x, y, z = center
        return self.tube(path, [(x + radius*math.cos(k*2*math.pi/96),
                                 y + radius*math.sin(k*2*math.pi/96), z)
                                for k in range(97)], thickness, material)

    def body(self, path, pos, mass):
        g = UsdGeom.Xform.Define(self.stage, ROOT + '/' + path)
        g.AddTranslateOp().Set(Gf.Vec3d(*pos))
        UsdPhysics.RigidBodyAPI.Apply(g.GetPrim())
        UsdPhysics.MassAPI.Apply(g.GetPrim()).CreateMassAttr(mass)
        prim = g.GetPrim()
        prim.AddAppliedSchema('PhysxRigidBodyAPI')
        prim.CreateAttribute('physxRigidBody:solverPositionIterationCount', Sdf.ValueTypeNames.Int).Set(16)
        prim.CreateAttribute('physxRigidBody:solverVelocityIterationCount', Sdf.ValueTypeNames.Int).Set(4)
        return g

