"""USD builders for the bike's front triangle with its bottle cage (one fixed-base articulation: the frame, plus the cage's
sprung arm on a revolute joint) and the bottle (a free rigid body). The geometry comes from bike_cage.geom; pxr is
imported only when a builder is called. Created 11_16_26 (America/New_York).

Local frame of the bike prim: x across the frame (world x, + away from the robot), y = the frame's X (forward along the
bike), z = the frame's Z (up); origin at the bottom bracket's centre. The bottle's local frame: origin at the middle of
its base, z up its axis.
"""

import math

import numpy as np

from . import geom as G

MU = {"frame": 0.4, "cage": 0.35, "bottle": 0.6}  # painted aluminium; glass-fibre polyamide; textured PP (chosen)
COLORS = {  # sRGB base colour, metallic, roughness
    "frame": ((0.12, 0.30, 0.20), 0.5, 0.4), "cage": ((0.05, 0.05, 0.06), 0.0, 0.5), "arm": ((0.10, 0.10, 0.11), 0.0, 0.5),
    "steel": ((0.55, 0.56, 0.58), 1.0, 0.35), "black": ((0.04, 0.04, 0.04), 0.0, 0.7), "bottle": ((0.85, 0.88, 0.90), 0.0, 0.35),
    "cap": ((0.10, 0.10, 0.12), 0.0, 0.5), "tyre": ((0.08, 0.08, 0.08), 0.0, 0.9), "stand": ((0.20, 0.20, 0.22), 0.6, 0.5),
}
FRAME_MASS = 4.0  # kg; the frame is fixed to the world, so this only sets the articulation's root
SIDES = 20


def _schema(prim, schema, **attrs):
    from pxr import Sdf

    if not prim.ApplyAPI(schema):
        prim.AddAppliedSchema(schema)
    types = {bool: Sdf.ValueTypeNames.Bool, int: Sdf.ValueTypeNames.Int, float: Sdf.ValueTypeNames.Float}
    for name, value in attrs.items():
        prim.CreateAttribute(name, types[type(value)]).Set(value)


def _lin(c):
    return tuple(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in c)


def quat_wxyz(r):
    """(w, x, y, z) of a 3x3 rotation matrix."""
    r = np.asarray(r, float)
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((0.25 * s), (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    i = int(np.argmax(np.diag(r)))
    if i == 0:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        return ((r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s)
    if i == 1:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        return ((r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s)
    s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
    return ((r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s)


def rot_x(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array(((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c)))


def tube_pose(p0, p1):
    """(centre (3,), rotation) of a box running from frame point p0 to p1 (2D, frame plane), its local y along the tube."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    ang = math.degrees(math.atan2(d[1], d[0]))
    c = (p0 + p1) / 2
    return np.array((0.0, c[0], c[1])), rot_x(ang), float(np.linalg.norm(d))


class _Builder:
    def __init__(self, stage, root):
        self.stage, self.root = stage, root
        self.looks, self.phys = {}, {}

    def look(self, name):
        from pxr import Gf, Sdf, UsdShade

        if name not in self.looks:
            color, metal, rough = COLORS[name]
            mat = UsdShade.Material.Define(self.stage, f"{self.root}/Looks/{name}")
            sh = UsdShade.Shader.Define(self.stage, f"{self.root}/Looks/{name}/shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*_lin(color)))
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metal))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            self.looks[name] = mat
        return self.looks[name]

    def surface(self, name):
        from pxr import UsdPhysics, UsdShade

        if name not in self.phys:
            mat = UsdShade.Material.Define(self.stage, f"{self.root}/Physics/{name}")
            pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            pm.CreateStaticFrictionAttr(MU[name])
            pm.CreateDynamicFrictionAttr(MU[name])
            pm.CreateRestitutionAttr(0.0)
            self.phys[name] = mat
        return self.phys[name]

    def _finish(self, g, look, collide):
        from pxr import UsdPhysics, UsdShade

        prim = g.GetPrim()
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.look(look))
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            _schema(prim, "PhysxCollisionAPI", **{"physxCollision:contactOffset": 0.002, "physxCollision:restOffset": 0.0})
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.surface(collide), UsdShade.Tokens.weakerThanDescendants, "physics")
        return g

    @staticmethod
    def _xform(g, centre, rot=None, scale=None):
        from pxr import Gf

        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        if rot is not None:
            w, x, y, z = quat_wxyz(rot)
            g.AddOrientOp().Set(Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z))))
        if scale is not None:
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, scale)))

    def box(self, path, centre, size, look, collide=None, rot=None):
        from pxr import UsdGeom

        c = UsdGeom.Cube.Define(self.stage, path)
        c.CreateSizeAttr(1.0)
        self._xform(c, centre, rot, size)
        return self._finish(c, look, collide)

    def cyl(self, path, centre, r, h, look, rot=None, axis="Z"):
        from pxr import UsdGeom

        c = UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateAxisAttr(axis)
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(h))
        self._xform(c, centre, rot)
        return self._finish(c, look, None)

    def prism(self, path, r, z0, z1, look, collide, n=SIDES):
        """A convex n-sided prism along local z from z0 to z1, circumscribing radius r at its flats' midpoints
        (the collider of a round part)."""
        from pxr import Gf, UsdGeom, UsdPhysics, Vt

        rc = r / math.cos(math.pi / n)
        m = UsdGeom.Mesh.Define(self.stage, path)
        pts = [Gf.Vec3f(rc * math.cos(2 * math.pi * (k + 0.5) / n), rc * math.sin(2 * math.pi * (k + 0.5) / n), z) for z in (z0, z1) for k in range(n)]
        counts, idx = [], []
        for k in range(n):
            j = (k + 1) % n
            counts.append(4)
            idx += [k, j, n + j, n + k]
        counts += [n, n]
        idx += list(range(n - 1, -1, -1)) + list(range(n, 2 * n))
        m.CreatePointsAttr(Vt.Vec3fArray(pts))
        m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        m.CreateSubdivisionSchemeAttr("none")
        self._finish(m, look, collide)
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
        return m

    def rigid(self, prim, mass, com=(0.0, 0.0, 0.0), inertia=None, iters=32):
        from pxr import Gf, UsdPhysics

        UsdPhysics.RigidBodyAPI.Apply(prim)
        _schema(prim, "PhysxRigidBodyAPI", **{"physxRigidBody:solverPositionIterationCount": iters,
                                              "physxRigidBody:solverVelocityIterationCount": 4,
                                              "physxRigidBody:maxDepenetrationVelocity": 0.3,
                                              "physxRigidBody:enableCCD": True, "physxRigidBody:sleepThreshold": 0.0})
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))


def _hide(stage, path):
    from pxr import UsdGeom

    UsdGeom.Imageable(stage.GetPrimAtPath(path)).MakeInvisible()


def cage_box(lo_hi):
    """(centre, size, rotation) in the bike's local frame of a box given by its ranges in the cage frame (s, n, a)."""
    s, n, a = (np.mean(lo_hi[k]) for k in ("s", "n", "a"))
    size = np.array([lo_hi[k][1] - lo_hi[k][0] for k in ("s", "n", "a")])
    return G.cage_to_frame(s, n, a), size, G.cage_rotation()


def arm_boxes():
    """The sprung arm's band as boxes in the arm link's frame (the cage frame moved to the hinge; closed pose):
    [(name, centre, size, rotation)]. A short link joins the hinge to the band's far-back end."""
    out = []
    r0, r1 = G.ARM_R
    rm, t = (r0 + r1) / 2, r1 - r0
    a0, a1 = G.ARM_A
    phis = np.linspace(G.ARM_PHI[0], G.ARM_PHI[1], G.ARM_SEGS + 1)
    for k in range(G.ARM_SEGS):
        pa, pb = G.arm_point(phis[k], rm), G.arm_point(phis[k + 1], rm)
        mid = (pa + pb) / 2 - G.HINGE
        d = pb - pa
        ang = math.atan2(d[1], d[0])
        rot = np.array(((math.cos(ang), -math.sin(ang), 0.0), (math.sin(ang), math.cos(ang), 0.0), (0.0, 0.0, 1.0)))
        out.append((f"band{k}", np.array((mid[0], mid[1], (a0 + a1) / 2)), np.array((np.linalg.norm(d) + 0.002, t, a1 - a0)), rot))
    start = G.arm_point(G.ARM_PHI[0], rm) - G.HINGE
    ang = math.atan2(start[1], start[0])
    rot = np.array(((math.cos(ang), -math.sin(ang), 0.0), (math.sin(ang), math.cos(ang), 0.0), (0.0, 0.0, 1.0)))
    out.append(("root", np.array((start[0] / 2, start[1] / 2, (a0 + a1) / 2)), np.array((np.linalg.norm(start) + 0.002, t, a1 - a0)), rot))
    return out


def frame_boxes():
    """Colliding boxes of the frame link (bike local frame): [(name, centre, size, rotation, look)]."""
    out = []
    # down tube: straight part offset half its depth below its inner edge, and the bend into the shell
    c, r, ln = tube_pose(*G.DT_IN)
    c = c + np.r_[0.0, -G.N * (G.DT_DEPTH / 2)]
    out.append(("down_tube", c, np.array((G.WIDTH["dt"], ln + 0.06, G.DT_DEPTH)), r, "frame"))
    c, r, ln = tube_pose(*G.DT_BEND)
    nb = np.array((-(G.DT_BEND[1] - G.DT_BEND[0])[1], (G.DT_BEND[1] - G.DT_BEND[0])[0])) / ln
    out.append(("down_tube_bend", c + np.r_[0.0, -nb * (G.DT_DEPTH / 2)], np.array((G.WIDTH["dt"], ln + 0.02, G.DT_DEPTH)), r, "frame"))
    for k, (p0, p1) in enumerate(zip(G.TT_LOW[:-1], G.TT_LOW[1:])):
        c, r, ln = tube_pose(p0, p1)
        d = (p1 - p0) / ln
        up = np.array((-d[1], d[0]))
        out.append((f"top_tube{k}", c + np.r_[0.0, up * (G.TT_DEPTH / 2)], np.array((G.WIDTH["tt"], ln + 0.004, G.TT_DEPTH)), r, "frame"))
    for k, (p0, p1) in enumerate(zip(G.ST_FRONT[:-1], G.ST_FRONT[1:])):
        c, r, ln = tube_pose(p0, p1)
        d = (p1 - p0) / ln
        back = np.array((d[1], -d[0]))  # the seat tube lies behind (−X) its front edge
        if back[0] > 0:
            back = -back
        out.append((f"seat_tube{k}", c + np.r_[0.0, back * (G.ST_DEPTH / 2)], np.array((G.WIDTH["st"], ln + 0.004, G.ST_DEPTH)), r, "frame"))
    hd = np.array((math.cos(math.radians(G.HT_AXIS_DEG)), math.sin(math.radians(G.HT_AXIS_DEG))))
    c, r, ln = tube_pose(G.HT_CENTRE - hd * G.HT_LEN / 2, G.HT_CENTRE + hd * G.HT_LEN / 2)
    out.append(("head_tube", c, np.array((G.WIDTH["ht"], ln, 0.050)), r, "frame"))
    out.append(("bb_shell", np.zeros(3), np.array((G.BB_W, 2 * G.BB_R, 2 * G.BB_R)), np.eye(3), "frame"))
    # the near (non-drive) crank arm (175 mm), set level and pointing back, with its flat pedal (about 100 x 100 mm)
    out.append(("crank_near", np.array((-0.082, -0.085, 0.0)), np.array((0.016, 0.190, 0.030)), np.eye(3), "black"))
    out.append(("pedal_near", np.array((-0.145, -0.175, 0.0)), np.array((0.100, 0.100, 0.022)), np.eye(3), "black"))
    # cage parts fixed to the frame (the bosses bolt the cage's spine to the tube)
    for name, spec in (("cage_spine", G.SPINE), ("cage_floor", G.FLOOR), ("cage_lip_front", G.LIP_FRONT), ("cage_lip_near", G.LIP_NEAR),
                       ("cage_wall_far", G.WALL_FAR)):
        c, size, r = cage_box(spec)
        out.append((name, c, size, r, "cage"))
    return out


def build_bike(stage, path):
    """The bike under `path` (placed at the bottom bracket's centre): a fixed-base articulation of the frame (root link) and
    the cage's sprung arm."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    root_prim = stage.GetPrimAtPath(path)
    UsdPhysics.ArticulationRootAPI.Apply(root_prim)
    _schema(root_prim, "PhysxArticulationAPI", **{"physxArticulation:solverPositionIterationCount": 32,
                                                  "physxArticulation:solverVelocityIterationCount": 4,
                                                  "physxArticulation:enabledSelfCollisions": False})
    b = _Builder(stage, path)
    fr = f"{path}/frame"
    UsdGeom.Xform.Define(stage, fr)
    fp = stage.GetPrimAtPath(fr)
    b.rigid(fp, FRAME_MASS, com=(0.0, 0.15, 0.25), inertia=(0.3, 0.2, 0.2))
    for name, c, size, r, look in frame_boxes():
        b.box(f"{fr}/{name}", c, size, look, "cage" if name.startswith("cage") else "frame", rot=r)
    # drawn only: chain stays and seat stays to the rear axle (420-440 mm back), the seat post and the workstand clamp
    axle = np.array((-0.437, 0.062))  # 439 mm chain stay, 62 mm bottom-bracket drop (Salsa geometry, XS 29)
    st_top = np.array((-0.075, 0.332))
    for side in (-1, 1):
        for name, p0, p1, w in (("chainstay", np.zeros(2), axle, 0.022), ("seatstay", st_top, axle, 0.016)):
            c, r, ln = tube_pose(p0, p1)
            b.box(f"{fr}/v_{name}_{'n' if side < 0 else 'f'}", c + (side * 0.055, 0.0, 0.0), (w, ln, w), "frame", rot=r)
    sd = (G.ST_FRONT[-1] - G.ST_FRONT[-2]) / np.linalg.norm(G.ST_FRONT[-1] - G.ST_FRONT[-2])
    c, r, ln = tube_pose(st_top, st_top + sd * 0.16)
    b.box(f"{fr}/v_seatpost", c, (0.0316, ln, 0.0316), "steel", rot=r)
    clamp = st_top + sd * 0.12
    b.box(f"{fr}/v_stand_clamp", (0.0, clamp[0], clamp[1]), (0.07, 0.06, 0.07), "stand")
    b.box(f"{fr}/v_stand_arm", (0.0, clamp[0] - 0.12, clamp[1]), (0.035, 0.20, 0.035), "stand")
    b.cyl(f"{fr}/v_chainring", (0.075, 0.0, 0.0), 0.085, 0.004, "steel", axis="X")
    # fixed to the world where it stands
    fix = UsdPhysics.FixedJoint.Define(stage, f"{path}/fix")
    fix.CreateBody1Rel().SetTargets([Sdf.Path(fr)])
    xf = UsdGeom.Xformable(root_prim).ComputeLocalToWorldTransform(0)
    t = xf.ExtractTranslation()
    q = xf.ExtractRotationQuat()
    fix.CreateLocalPos0Attr(Gf.Vec3f(float(t[0]), float(t[1]), float(t[2])))
    fix.CreateLocalRot0Attr(Gf.Quatf(float(q.GetReal()), Gf.Vec3f(*map(float, q.GetImaginary()))))
    fix.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    fix.CreateLocalRot1Attr(Gf.Quatf(1.0))
    # the sprung arm
    hinge = G.cage_to_frame(G.HINGE[0], G.HINGE[1], 0.0)
    rc = G.cage_rotation()
    ap = f"{path}/arm"
    ax = UsdGeom.Xform.Define(stage, ap)
    b._xform(ax, hinge, rc)
    arm_prim = stage.GetPrimAtPath(ap)
    b.rigid(arm_prim, G.ARM_MASS, com=(0.04, 0.05, sum(G.ARM_A) / 2), inertia=(2e-5, 2e-5, 2e-5))
    for name, c, size, r in arm_boxes():
        b.box(f"{ap}/{name}", c, size, "arm", "cage", rot=r)
    j = UsdPhysics.RevoluteJoint.Define(stage, f"{path}/arm_joint")
    j.CreateAxisAttr("Z")
    j.CreateBody0Rel().SetTargets([Sdf.Path(fr)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(ap)])
    w, x, y, z = quat_wxyz(rc)
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, hinge)))
    j.CreateLocalRot0Attr(Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z))))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0))
    j.CreateLowerLimitAttr(0.0)
    j.CreateUpperLimitAttr(float(G.ARM_LIMIT_DEG))
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(float(G.SPRING_K))
    d.CreateDampingAttr(0.0004)
    d.CreateTargetPositionAttr(float(G.SPRING_AIM_DEG))
    d.CreateMaxForceAttr(10.0)
    return root_prim


def build_bottle(stage, path):
    """The bottle under `path` (an Xform at the middle of its base, z up its axis): one rigid body, two convex colliders."""
    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    m, r, h = G.BOTTLE_MASS, G.BOTTLE_R, G.BOTTLE_H
    b.rigid(prim, m, com=(0.0, 0.0, 0.085), inertia=(m * (3 * r * r + h * h) / 12, m * (3 * r * r + h * h) / 12, m * r * r))
    _schema(prim, "PhysxContactReportAPI", **{"physxContactReport:threshold": 0.0})
    b.prism(f"{path}/c_body", r, 0.0, G.BODY_TOP, "bottle", "bottle")
    b.prism(f"{path}/c_cap", G.CAP_R, G.BODY_TOP, h, "cap", "bottle")
    for n in ("c_body", "c_cap"):
        _hide(stage, f"{path}/{n}")
    b.cyl(f"{path}/v_body", (0.0, 0.0, G.BODY_TOP / 2), r, G.BODY_TOP, "bottle")
    b.cyl(f"{path}/v_band", (0.0, 0.0, 0.135), r + 0.0005, 0.012, "cap")
    b.cyl(f"{path}/v_cap", (0.0, 0.0, (G.BODY_TOP + h - 0.008) / 2), G.CAP_R, h - 0.008 - G.BODY_TOP, "cap")
    b.cyl(f"{path}/v_nozzle", (0.0, 0.0, h - 0.004), 0.011, 0.008, "cap")
    return prim
