"""Original compound rigid geometry; all solids use native PhysX contacts."""

import json
from pathlib import Path

import numpy as np


class Writer:
    def __init__(self, directory):
        from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

        self.Gf, self.Px, self.Usd, self.Geom, self.Phy, self.Shade = (
            Gf,
            PhysxSchema,
            Usd,
            UsdGeom,
            UsdPhysics,
            UsdShade,
        )
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.records = []

    def stage(self, name):
        s = self.Usd.Stage.CreateNew(str(self.directory / (name + ".usda")))
        self.Geom.SetStageUpAxis(s, "Z")
        self.Geom.SetStageMetersPerUnit(s, 1.0)
        root = self.Geom.Xform.Define(s, "/Asset")
        s.SetDefaultPrim(root.GetPrim())
        mat = self.Shade.Material.Define(s, "/Asset/Physics")
        p = self.Phy.MaterialAPI.Apply(mat.GetPrim())
        p.CreateStaticFrictionAttr(0.65)
        p.CreateDynamicFrictionAttr(0.5)
        p.CreateRestitutionAttr(0)
        return s

    def body(
        self,
        s,
        path,
        mass,
        position=(0, 0, 0),
        fixed=False,
        articulation=False,
        ccd=False,
    ):
        x = self.Geom.Xform.Define(s, path)
        x.AddTranslateOp().Set(self.Gf.Vec3d(*map(float, position)))
        x.AddOrientOp(precision=self.Geom.XformOp.PrecisionDouble).Set(
            self.Gf.Quatd(1, 0, 0, 0)
        )
        p = x.GetPrim()
        self.Phy.RigidBodyAPI.Apply(p)
        self.Phy.MassAPI.Apply(p).CreateMassAttr(mass)
        self.Px.PhysxRigidBodyAPI.Apply(p).CreateMaxDepenetrationVelocityAttr(0.5)
        if ccd:
            self.Px.PhysxRigidBodyAPI.Apply(p).CreateEnableCCDAttr(True)
        if articulation:
            self.Phy.ArticulationRootAPI.Apply(p)
            a = self.Px.PhysxArticulationAPI.Apply(p)
            a.CreateEnabledSelfCollisionsAttr(True)
            a.CreateSolverPositionIterationCountAttr(32)
            a.CreateSolverVelocityIterationCountAttr(8)
        if fixed:
            j = self.Phy.FixedJoint.Define(s, path + "/WorldFix")
            j.CreateBody1Rel().SetTargets([p.GetPath()])
            j.CreateLocalPos0Attr(self.Gf.Vec3f(*map(float, position)))
            j.CreateLocalPos1Attr(self.Gf.Vec3f(0, 0, 0))
        return path

    def _finish(self, s, geom, color, collide, contact=0.0004):
        geom.CreateDisplayColorAttr([self.Gf.Vec3f(*color)])
        if collide:
            p = geom.GetPrim()
            self.Phy.CollisionAPI.Apply(p)
            a = self.Px.PhysxCollisionAPI.Apply(p)
            a.CreateContactOffsetAttr(contact)
            a.CreateRestOffsetAttr(0)
            mat = self.Shade.Material.Get(s, "/Asset/Physics")
            self.Shade.MaterialBindingAPI.Apply(p).Bind(mat, materialPurpose="physics")
        return geom

    def box(self, s, path, size, position, color, rotation=None, collide=True):
        from pxr import Tf

        path = "/".join(Tf.MakeValidIdentifier(p) if p else "" for p in path.split("/"))
        x = self.Geom.Cube.Define(s, path)
        x.CreateSizeAttr(1)
        x.AddTranslateOp().Set(self.Gf.Vec3d(*map(float, position)))
        if rotation is not None:
            from yam_arena.ik import matrix_to_quat_xyzw

            q = matrix_to_quat_xyzw(np.asarray(rotation))
            x.AddOrientOp(precision=self.Geom.XformOp.PrecisionDouble).Set(
                self.Gf.Quatd(float(q[3]), self.Gf.Vec3d(*map(float, q[:3])))
            )
        x.AddScaleOp().Set(self.Gf.Vec3f(*map(float, size)))
        self._finish(s, x, color, collide)
        self.records.append(
            {
                "asset": Path(s.GetRootLayer().identifier).stem,
                "solid": path,
                "kind": "box",
                "size": list(size),
                "position": list(position),
                "color": list(color),
            }
        )

        return x

    def rounded_box(self, s, path, size, position, color, radius, bevel, collide=True):
        """A convex rounded rectangle with chamfered upper and lower edges."""
        from math import cos, pi, sin

        width, length, height = size
        points = []
        levels = ((-height / 2, bevel), (-height / 2 + bevel, 0),
                  (height / 2 - bevel, 0), (height / 2, bevel))
        corners = ((1, 1), (-1, 1), (-1, -1), (1, -1))
        for z, inset in levels:
            for corner, (sx, sy) in enumerate(corners):
                px = sx * (width / 2 - radius)
                py = sy * (length / 2 - radius)
                for sample in range(4):
                    angle = pi / 2 * (corner + sample / 3)
                    points.append(self.Gf.Vec3f(
                        position[0] + px + (radius - inset) * cos(angle),
                        position[1] + py + (radius - inset) * sin(angle),
                        position[2] + z,
                    ))
        count = len(corners) * 4
        faces = [list(reversed(range(count)))]
        for ring in range(len(levels) - 1):
            lower = ring * count
            upper = lower + count
            for i in range(count):
                next_i = (i + 1) % count
                faces.append([lower + i, lower + next_i, upper + next_i, upper + i])
        faces.append(list(range(len(points) - count, len(points))))
        mesh = self.Geom.Mesh.Define(s, path)
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
        mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
        mesh.CreateSubdivisionSchemeAttr("none")
        if collide:
            self.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
        self._finish(s, mesh, color, collide)
        self.records.append({"asset": Path(s.GetRootLayer().identifier).stem,
                             "solid": path, "kind": "rounded_box", "size": list(size),
                             "position": list(position), "radius": radius, "bevel": bevel,
                             "color": list(color)})

    def cylinder(
        self, s, path, radius, length, position, color, axis="Z", collide=True
    ):
        from pxr import Tf

        path = "/".join(Tf.MakeValidIdentifier(p) if p else "" for p in path.split("/"))
        x = self.Geom.Cylinder.Define(s, path)
        x.CreateRadiusAttr(radius)
        x.CreateHeightAttr(length)
        x.CreateAxisAttr(axis)
        x.AddTranslateOp().Set(self.Gf.Vec3d(*map(float, position)))
        self._finish(s, x, color, collide)
        self.records.append(
            {
                "asset": Path(s.GetRootLayer().identifier).stem,
                "solid": path,
                "kind": "cylinder",
                "radius": radius,
                "length": length,
                "axis": axis,
                "position": list(position),
                "color": list(color),
            }
        )

        return x

    def save(self, s):
        s.GetRootLayer().Save()
        return Path(s.GetRootLayer().identifier)

    def inventory(self):
        (self.directory / "geometry-inventory.json").write_text(
            json.dumps(self.records, indent=2) + "\n"
        )
