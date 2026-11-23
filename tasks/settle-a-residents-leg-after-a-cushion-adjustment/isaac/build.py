"""Export the settled scene to a native USD physics fixture. Build once per physical variant."""

import math
from pathlib import Path
import mujoco
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom as G, UsdPhysics as P, UsdShade
from task.station import make_model, reset


def v(x):
    return Gf.Vec3f(*map(float, x))


def quat(q):
    return Gf.Quatf(float(q[0]), v(q[1:]))


def report(p):
    p.AddAppliedSchema("PhysxContactReportAPI")
    p.CreateAttribute("physxContactReport:threshold", Sdf.ValueTypeNames.Float).Set(0.0)


def hull(s, path, pts, faces):
    g = G.Mesh.Define(s, path)
    g.CreatePointsAttr([v(p) for p in pts])
    g.CreateFaceVertexCountsAttr([len(f) for f in faces])
    g.CreateFaceVertexIndicesAttr([int(i) for f in faces for i in f])
    g.CreateSubdivisionSchemeAttr("none")
    return g


def ellipsoid(s, path, rr):
    from scipy.spatial import ConvexHull

    p = np.array(
        [
            [
                rr[0] * math.cos(b) * math.cos(a),
                rr[1] * math.cos(b) * math.sin(a),
                rr[2] * math.sin(b),
            ]
            for b in np.linspace(-math.pi / 2, math.pi / 2, 13)[1:-1]
            for a in np.arange(24) * math.pi / 12
        ]
        + [[0, 0, rr[2]], [0, 0, -rr[2]]]
    )
    h = ConvexHull(p)
    fs = [
        f
        if np.dot(np.cross(p[f[1]] - p[f[0]], p[f[2]] - p[f[0]]), p[f].mean(axis=0)) > 0
        else f[::-1]
        for f in h.simplices
    ]
    return hull(s, path, p, fs)


def build(path, settings):
    kw = {
        k: v
        for k, v in settings.items()
        if k in ("resident_y", "leg_mass", "stiffness", "cushion_y")
    }
    m = make_model(**kw)
    d = reset(m)
    s = Usd.Stage.CreateInMemory()
    G.SetStageMetersPerUnit(s, 1)
    G.SetStageUpAxis(s, "Z")
    s.SetDefaultPrim(G.Xform.Define(s, "/Fixture").GetPrim())
    mat = UsdShade.Material.Define(s, "/Fixture/Fabric")
    prop = P.MaterialAPI.Apply(mat.GetPrim())
    prop.CreateStaticFrictionAttr(0.32)
    prop.CreateDynamicFrictionAttr(0.27)
    prop.CreateRestitutionAttr(0)
    bodies = {
        "world": "frame",
        "upper_leg": "upper_leg",
        "shin": "shin",
        "cushion": "cushion",
        **{f"foam{i}": f"foam{i}" for i in range(6)},
    }
    for orig, name in bodies.items():
        b = m.body(orig)
        i = b.id
        root = "/Fixture/" + name
        g = G.Xform.Define(s, root)
        pos = d.xpos[i] if i else np.zeros(3)
        g.AddTranslateOp().Set(Gf.Vec3d(*pos))
        g.AddOrientOp().Set(quat(d.xquat[i] if i else [1, 0, 0, 0]))
        rb = P.RigidBodyAPI.Apply(g.GetPrim())
        report(g.GetPrim())
        if not i:
            rb.CreateKinematicEnabledAttr(True)
        else:
            g.GetPrim().AddAppliedSchema("PhysxRigidBodyAPI")
            for key, kind, value in (
                ("sleepThreshold", Sdf.ValueTypeNames.Float, 0.0),
                ("stabilizationThreshold", Sdf.ValueTypeNames.Float, 0.0),
                ("solverPositionIterationCount", Sdf.ValueTypeNames.Int, 16),
                ("solverVelocityIterationCount", Sdf.ValueTypeNames.Int, 4),
            ):
                g.GetPrim().CreateAttribute("physxRigidBody:" + key, kind).Set(value)
            mass = P.MassAPI.Apply(g.GetPrim())
            mass.CreateMassAttr(float(m.body_mass[i]))
            mass.CreateCenterOfMassAttr(v(m.body_ipos[i]))
            mass.CreateDiagonalInertiaAttr(v(m.body_inertia[i]))
            mass.CreatePrincipalAxesAttr(quat(m.body_iquat[i]))
        for k in np.flatnonzero(m.geom_bodyid == i):
            n = m.geom(k).name
            if n in ("floor", "plinth"):
                continue
            t = m.geom_type[k]
            sz = m.geom_size[k]
            pth = root + "/" + n
            if t == mujoco.mjtGeom.mjGEOM_BOX:
                sh = G.Cube.Define(s, pth)
                sh.CreateSizeAttr(1)
            elif t == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
                sh = ellipsoid(s, pth, sz)
            elif t == mujoco.mjtGeom.mjGEOM_CAPSULE:
                sh = G.Capsule.Define(s, pth)
                sh.CreateRadiusAttr(float(sz[0]))
                sh.CreateHeightAttr(float(2 * sz[1]))
            elif t == mujoco.mjtGeom.mjGEOM_MESH:
                j = m.geom_dataid[k]
                a = m.mesh_vertadr[j]
                f = m.mesh_faceadr[j]
                sh = hull(
                    s,
                    pth,
                    m.mesh_vert[a : a + m.mesh_vertnum[j]],
                    m.mesh_face[f : f + m.mesh_facenum[j]],
                )
            else:
                raise ValueError(n)
            sh.AddTranslateOp().Set(Gf.Vec3d(*m.geom_pos[k]))
            sh.AddOrientOp().Set(quat(m.geom_quat[k]))
            if t == mujoco.mjtGeom.mjGEOM_BOX:
                sh.AddScaleOp().Set(v(2 * sz))
            sh.CreateDisplayColorAttr([v(m.geom_rgba[k, :3])])
            if m.geom_contype[k] or m.geom_conaffinity[k]:
                p = sh.GetPrim()
                P.CollisionAPI.Apply(p)
                if p.IsA(G.Mesh):
                    P.MeshCollisionAPI.Apply(p).CreateApproximationAttr("convexHull")
                p.AddAppliedSchema("PhysxCollisionAPI")
                p.CreateAttribute(
                    "physxCollision:contactOffset", Sdf.ValueTypeNames.Float
                ).Set(0.001)
                p.CreateAttribute(
                    "physxCollision:restOffset", Sdf.ValueTypeNames.Float
                ).Set(0.0)
                UsdShade.MaterialBindingAPI.Apply(p).Bind(
                    mat, UsdShade.Tokens.weakerThanDescendants, "physics"
                )
    # The torso and upper-leg shells meet at the hip. Suppress internal shell intersections
    # at that anatomical joint; preserve bed bearing and contacts with either arm.
    # Visible station furniture under the mattress and both fixed arm bases.
    for n, x, y, z, height, width in [
        *(
            ("bedPost" + str(i), x, y, -0.395, 0.65, 0.045)
            for i, (x, y) in enumerate(
                ((0.015, -0.39), (0.71, -0.39), (0.015, 1.19), (0.71, 1.19))
            )
        ),
        ("rightMount", -0.185, 0, -0.348, 0.746, 0.14),
        ("leftMount", -0.185, 0.610, -0.348, 0.746, 0.14),
    ]:
        g = G.Cube.Define(s, "/Fixture/frame/" + n)
        g.CreateSizeAttr(1)
        g.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        g.AddScaleOp().Set(v([width, width, height]))
        g.CreateDisplayColorAttr([v([0.20, 0.26, 0.29])])
    for n in ("torso", "far_leg", "arm_rest", "far_foot", "head"):
        P.FilteredPairsAPI.Apply(
            s.GetPrimAtPath("/Fixture/frame/" + n)
        ).CreateFilteredPairsRel().SetTargets(
            ["/Fixture/upper_leg/thigh", "/Fixture/upper_leg/knee_shell"]
        )
    for joint in ("hip", "knee", *(f"compression{i}" for i in range(6))):
        j = m.joint(joint).id
        k = m.jnt_bodyid[j]
        par = m.body_parentid[k]
        rot = joint in ("hip", "knee")
        child = m.body(k).name
        parent = bodies[m.body(par).name]
        obj = (P.RevoluteJoint if rot else P.PrismaticJoint).Define(
            s, "/Fixture/" + joint
        )
        if joint == "hip":
            obj.CreateCollisionEnabledAttr(True)
        obj.CreateBody0Rel().SetTargets(["/Fixture/" + parent])
        obj.CreateBody1Rel().SetTargets(["/Fixture/" + child])
        obj.CreateLocalPos0Attr(v(m.body_pos[k]))
        obj.CreateLocalPos1Attr(v((0, 0, 0)))
        obj.CreateAxisAttr("X" if rot else "Z")
        factor = 180 / math.pi if rot else 1
        obj.CreateLowerLimitAttr(float(m.jnt_range[j, 0] * factor))
        obj.CreateUpperLimitAttr(float(m.jnt_range[j, 1] * factor))
        drv = P.DriveAPI.Apply(obj.GetPrim(), "angular" if rot else "linear")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(float(m.jnt_stiffness[j] / factor))
        drv.CreateDampingAttr(float(m.dof_damping[m.jnt_dofadr[j]] / factor))
    # Filter neighbouring foam strips, matching the independent compression model.
    for i in range(6):
        P.FilteredPairsAPI.Apply(
            s.GetPrimAtPath(f"/Fixture/foam{i}")
        ).CreateFilteredPairsRel().SetTargets(
            [f"/Fixture/foam{j}" for j in range(i + 1, 6)]
        )
    s.GetRootLayer().Export(str(path))


if __name__ == "__main__":
    import json, hashlib, argparse
    from tools.suite import CASES

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    destination = parser.parse_args().output.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    for geometry in sorted(
        {
            json.dumps({k: v for k, v in opts.items() if k != "delay"}, sort_keys=True)
            for _, opts, _ in CASES.values()
        }
    ):
        key = hashlib.sha256(geometry.encode()).hexdigest()[:12]
        build(destination / ("fixture-" + key + ".usda"), json.loads(geometry))
