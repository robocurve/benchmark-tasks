"""The saute pan, the potato dice and the burner. The geometry and mass constants are NumPy only, so the task file and the
tests use them without Isaac Sim; the USD builders import pxr when called.

Frames (m):
- Pan: origin at the centre of the cooking surface (the floor's top face), z up, +x away from the handle (the way food
  travels in a toss). The handle leaves the wall at -x and rises 20 degrees.
- Die: origin at the cube's centre. Its local -z face is the browned one, so a die starts with local +z up.
- Burner: origin at the centre of its top face.

The pan is a true-size 8 in (203 mm) aluminium fry pan: a 134 mm flat floor, a 20 mm corner radius, a wall flaring 30
degrees from vertical to a 203 mm rim 40 mm above the floor, 3 mm thick throughout. Its colliders are the real shell (a
32-sided floor prism and 4 rings of 32 convex wall pieces), not a filled bowl, so dice ride up the curved wall as on the
real pan. The handle is a flat strap with a silicone sleeve.
"""

import math

import numpy as np

# -- pan --------------------------------------------------------------------------------------------------------------
N_SEG = 32  # sides of the collision shell
R_FLOOR = 0.067  # flat cooking surface radius
R_CORNER = 0.020  # the corner radius between floor and wall
FLARE = math.radians(60.0)  # the wall's slope from horizontal (30 degrees from vertical)
DEPTH = 0.040  # rim above the cooking surface
T_WALL = 0.003  # aluminium thickness, floor and wall
ARC_STEPS = 3  # the corner radius as 3 flat rings of 20 degrees
RHO_AL = 2700.0

HANDLE_RISE = math.radians(20.0)
HANDLE_DIR = np.array((-math.cos(HANDLE_RISE), 0.0, math.sin(HANDLE_RISE)))  # from the wall out along the handle
HANDLE_UP = np.array((math.sin(HANDLE_RISE), 0.0, math.cos(HANDLE_RISE)))  # the handle's flat-face normal (the finger closing axis)
STRAP = dict(len=0.190, w=0.025, t=0.006, mass=0.060)  # stamped strap, riveted to the wall
SLEEVE = dict(s0=0.060, s1=0.190, w=0.030, t=0.018, mass=0.035)  # silicone sleeve over the strap's outer 130 mm
S_GRASP = 0.125  # where the fingertip midpoint holds the sleeve, along the handle from its root

# -- dice -------------------------------------------------------------------------------------------------------------
DIE = 0.013  # edge
DIE_MASS = 0.0024  # 2.2 cm^3 of par-cooked potato at 1.08 g/cm^3
N_DICE = 12
BROWN_LOCAL = np.array((0.0, 0.0, -1.0))  # the browned face's outward normal in the die's frame
# the fixed starting layout on the cooking surface: (x, y, yaw degrees) in the pan frame, browned face down
DICE_START = tuple(
    [(0.018 * math.cos(math.radians(a)), 0.018 * math.sin(math.radians(a)), y) for a, y in ((45, 10), (135, 35), (225, 60), (315, 20))]
    + [(0.047 * math.cos(math.radians(a)), 0.047 * math.sin(math.radians(a)), y)
       for a, y in ((0, 30), (45, 5), (90, 50), (135, 15), (180, 40), (225, 25), (270, 70), (315, 55))]
)

# -- burner -----------------------------------------------------------------------------------------------------------
BURNER = dict(size=0.240, h=0.040, plate_r=0.090)  # a cold portable hotplate: housing and a cast-iron plate, flat on top

# friction (static, dynamic) and restitution. PhysX averages the two bodies' values, so pan and potato carry the same
# numbers and the pair gets exactly these: oiled par-cooked potato on aluminium.
MU = {"potato": (0.35, 0.30, 0.10), "aluminium": (0.35, 0.30, 0.10), "silicone": (1.0, 1.0, 0.0), "iron": (0.5, 0.5, 0.0)}
COLORS = {  # sRGB base colour, metallic, roughness
    "aluminium": ((0.74, 0.75, 0.77), 1.0, 0.38), "strap": ((0.62, 0.63, 0.65), 1.0, 0.45), "sleeve": ((0.78, 0.12, 0.10), 0.0, 0.6),
    "rivet": ((0.50, 0.51, 0.53), 1.0, 0.3), "potato": ((0.93, 0.84, 0.52), 0.0, 0.7), "brown": ((0.36, 0.17, 0.05), 0.0, 0.8),
    "housing": ((0.10, 0.10, 0.11), 0.2, 0.6), "iron": ((0.05, 0.05, 0.055), 0.6, 0.7), "knob": ((0.75, 0.75, 0.75), 0.8, 0.4),
}


def profile():
    """The wall's inner profile as (r, z) points from the floor's edge to the rim, and each point's outward normal."""
    pts, nrm = [], []
    for k in range(ARC_STEPS + 1):
        th = FLARE * k / ARC_STEPS
        pts.append((R_FLOOR + R_CORNER * math.sin(th), R_CORNER * (1.0 - math.cos(th))))
        nrm.append((math.sin(th), -math.cos(th)))
    r, z = pts[-1]
    pts.append((r + (DEPTH - z) / math.tan(FLARE), DEPTH))
    nrm.append(nrm[-1])
    return np.array(pts), np.array(nrm)


PROFILE, PROFILE_N = profile()
R_RIM = float(PROFILE[-1, 0])  # the rim's inner radius: 101.6 mm, an 8 in pan
HANDLE_ROOT = np.array((-(R_RIM + T_WALL * math.sin(FLARE)) + 0.004, 0.0, DEPTH - 0.006))  # where the strap meets the wall
GRASP = HANDLE_ROOT + S_GRASP * HANDLE_DIR  # the fingertip midpoint on the sleeve, in the pan frame
HANDLE_END = HANDLE_ROOT + STRAP["len"] * HANDLE_DIR


def wall_radius(z):
    """The inner wall's radius at height z above the cooking surface (clamped to the floor's edge and the rim)."""
    return float(np.interp(z, PROFILE[:, 1], PROFILE[:, 0]))


def _samples():
    """Point masses over the shell and handle, for the pan's mass, centre of mass and inertia."""
    rng = np.random.default_rng(0)
    out = []
    # floor disc at mid-thickness
    n = 4000
    r = R_FLOOR * np.sqrt(rng.random(n))
    a = 2 * math.pi * rng.random(n)
    m = RHO_AL * math.pi * R_FLOOR**2 * T_WALL
    out.append((np.c_[r * np.cos(a), r * np.sin(a), np.full(n, -T_WALL / 2)], np.full(n, m / n)))
    # wall rings at mid-thickness
    mid = PROFILE + 0.5 * T_WALL * PROFILE_N
    for (r0, z0), (r1, z1) in zip(mid[:-1], mid[1:]):
        L = math.hypot(r1 - r0, z1 - z0)
        m = RHO_AL * math.pi * (r0 + r1) * L * T_WALL
        n = 2000
        s = rng.random(n)
        a = 2 * math.pi * rng.random(n)
        rr = r0 + s * (r1 - r0)
        out.append((np.c_[rr * np.cos(a), rr * np.sin(a), z0 + s * (z1 - z0)], np.full(n, m / n)))
    # strap and sleeve along the handle's axis
    for s0, s1, m in ((0.0, STRAP["len"], STRAP["mass"]), (SLEEVE["s0"], SLEEVE["s1"], SLEEVE["mass"])):
        s = np.linspace(s0, s1, 400)
        out.append((HANDLE_ROOT + s[:, None] * HANDLE_DIR, np.full(400, m / 400)))
    return np.concatenate([p for p, _ in out]), np.concatenate([w for _, w in out])


def mass_properties():
    """(mass kg, centre of mass, inertia tensor about it) in the pan frame."""
    p, w = _samples()
    m = float(w.sum())
    c = (p * w[:, None]).sum(0) / m
    d = p - c
    inertia = np.einsum("i,ijk->jk", w, np.einsum("ij,ij->i", d, d)[:, None, None] * np.eye(3) - np.einsum("ij,ik->ijk", d, d))
    return m, c, inertia


PAN_MASS, PAN_COM, PAN_INERTIA = mass_properties()


def _quat_from_matrix(r):
    """xyzw quaternion of a rotation matrix."""
    tr = np.trace(r)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w, x, y, z = (r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w, x, y, z = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w, x, y, z = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s
    return np.array((x, y, z, w))


def principal_inertia():
    """(diagonal inertia, principal axes as an xyzw quaternion) for USD's MassAPI."""
    vals, vecs = np.linalg.eigh(PAN_INERTIA)
    if np.linalg.det(vecs) < 0:
        vecs[:, 2] *= -1
    return vals, _quat_from_matrix(vecs)


def shell_pieces():
    """The wall's convex collision pieces: each an (8, 3) vertex array in the pan frame."""
    outer = PROFILE + T_WALL * PROFILE_N
    out = []
    for k in range(len(PROFILE) - 1):
        quad = (PROFILE[k], PROFILE[k + 1], outer[k + 1], outer[k])
        for j in range(N_SEG):
            a0, a1 = 2 * math.pi * j / N_SEG, 2 * math.pi * (j + 1) / N_SEG
            out.append(np.array([(r * math.cos(a), r * math.sin(a), z) for a in (a0, a1) for r, z in quad]))
    return out


def floor_piece():
    """The floor's convex collision prism: (2 * N_SEG, 3) vertices."""
    return np.array([(R_FLOOR * math.cos(2 * math.pi * j / N_SEG), R_FLOOR * math.sin(2 * math.pi * j / N_SEG), z)
                     for z in (0.0, -T_WALL) for j in range(N_SEG)])


# -- USD --------------------------------------------------------------------------------------------------------------
def _physx(prim, schema, **attrs):
    from pxr import Sdf

    if not prim.ApplyAPI(schema):
        prim.AddAppliedSchema(schema)
    types = {bool: Sdf.ValueTypeNames.Bool, int: Sdf.ValueTypeNames.Int, float: Sdf.ValueTypeNames.Float}
    for name, value in attrs.items():
        prim.CreateAttribute(name, types[type(value)]).Set(value)


def _lin(c):
    return tuple(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in c)


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
            s, d, e = MU[name]
            pm.CreateStaticFrictionAttr(s)
            pm.CreateDynamicFrictionAttr(d)
            pm.CreateRestitutionAttr(e)
            self.phys[name] = mat
        return self.phys[name]

    def finish(self, g, look, collide=None, offset=0.002, visible=True):
        from pxr import UsdGeom, UsdPhysics, UsdShade

        prim = g.GetPrim()
        if look:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.look(look))
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            _physx(prim, "PhysxCollisionAPI", **{"physxCollision:contactOffset": float(offset), "physxCollision:restOffset": 0.0})
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.surface(collide), UsdShade.Tokens.weakerThanDescendants, "physics")
        if not visible:
            UsdGeom.Imageable(prim).MakeInvisible()
        return g

    def box(self, path, center, size, look, collide=None, rot_y=0.0, rot_z=0.0, offset=0.002, visible=True):
        """A box of edge lengths ``size`` centred at ``center``, turned rot_z then rot_y (degrees) about its centre."""
        from pxr import Gf, UsdGeom

        c = UsdGeom.Cube.Define(self.stage, path)
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        if rot_z:
            c.AddRotateZOp().Set(float(rot_z))
        if rot_y:
            c.AddRotateYOp().Set(float(rot_y))
        c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        return self.finish(c, look, collide, offset, visible)

    def cyl(self, path, center, r, h, look, axis="Z", rot_y=0.0):
        from pxr import Gf, UsdGeom

        c = UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateAxisAttr(axis)
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(h))
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        if rot_y:
            c.AddRotateYOp().Set(float(rot_y))
        return self.finish(c, look)

    def hull(self, path, verts, collide, offset=0.002):
        """An invisible convex collider from its vertices (a triangle fan mesh, cooked by PhysX as its convex hull)."""
        from pxr import Gf, UsdGeom, UsdPhysics, Vt

        m = UsdGeom.Mesh.Define(self.stage, path)
        n = len(verts)
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, v)) for v in verts]))
        tris = [(0, i, j) for i in range(1, n) for j in range(i + 1, n)][: max(4, 2 * n)]
        m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(tris)))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for t in tris for i in t]))
        m.CreateSubdivisionSchemeAttr("none")
        self.finish(m, None, collide, offset, visible=False)
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
        return m

    def lathe(self, path, loop, look, n=96):
        """A closed (r, z) loop turned about z: the pan's visible shell."""
        from pxr import Gf, UsdGeom, Vt

        m = UsdGeom.Mesh.Define(self.stage, path)
        k = len(loop)
        pts = [Gf.Vec3f(float(r * math.cos(2 * math.pi * j / n)), float(r * math.sin(2 * math.pi * j / n)), float(z)) for j in range(n) for r, z in loop]
        counts, idx = [], []
        for j in range(n):
            j2 = (j + 1) % n
            for i in range(k):
                i2 = (i + 1) % k
                counts.append(4)
                idx += [j * k + i, j2 * k + i, j2 * k + i2, j * k + i2]
        m.CreatePointsAttr(Vt.Vec3fArray(pts))
        m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateDoubleSidedAttr(True)
        return self.finish(m, look)

    def rigid(self, prim, mass, com=None, inertia=None, axes=None, iters=24, depen=1.0, report=False):
        from pxr import Gf, UsdPhysics

        UsdPhysics.RigidBodyAPI.Apply(prim)
        _physx(prim, "PhysxRigidBodyAPI", **{"physxRigidBody:solverPositionIterationCount": iters, "physxRigidBody:solverVelocityIterationCount": 2,
                                             "physxRigidBody:maxDepenetrationVelocity": float(depen), "physxRigidBody:enableCCD": True,
                                             "physxRigidBody:sleepThreshold": 0.0005})
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        if com is not None:
            m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
        if axes is not None:
            x, y, z, w = map(float, axes)
            m.CreatePrincipalAxesAttr(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
        if report:
            _physx(prim, "PhysxContactReportAPI", **{"physxContactReport:threshold": 0.0})


def build_pan(stage, path):
    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    inertia, axes = principal_inertia()
    b.rigid(prim, PAN_MASS, PAN_COM, inertia, axes, iters=32, report=True)
    b.hull(f"{path}/c_floor", floor_piece(), "aluminium")
    for i, verts in enumerate(shell_pieces()):
        b.hull(f"{path}/c_wall_{i}", verts, "aluminium")
    deg = math.degrees(HANDLE_RISE)
    mid = HANDLE_ROOT + (STRAP["len"] / 2 - 0.004) * HANDLE_DIR
    b.box(f"{path}/c_strap", mid, (STRAP["len"] + 0.008, STRAP["w"], STRAP["t"]), "strap", "aluminium", rot_y=deg)
    smid = HANDLE_ROOT + (SLEEVE["s0"] + SLEEVE["s1"]) / 2 * HANDLE_DIR
    b.box(f"{path}/c_sleeve", smid, (SLEEVE["s1"] - SLEEVE["s0"], SLEEVE["w"], SLEEVE["t"]), "sleeve", "silicone", rot_y=deg)
    # the visible shell: inner profile up, over a rounded rim, outer profile down, across the underside
    outer = PROFILE + T_WALL * PROFILE_N
    fine_in, fine_out = [], []
    for k in range(13):
        th = FLARE * k / 12
        s, c = math.sin(th), math.cos(th)
        fine_in.append((R_FLOOR + R_CORNER * s, R_CORNER * (1 - c)))
        fine_out.append((R_FLOOR + (R_CORNER + T_WALL) * s, R_CORNER - (R_CORNER + T_WALL) * c))
    loop = [(0.0005, 0.0)] + fine_in + [tuple(PROFILE[-1]), tuple(outer[-1])] + fine_out[::-1] + [(0.0005, -T_WALL)]
    b.lathe(f"{path}/v_shell", loop, "aluminium")
    for k, dy in enumerate((-0.007, 0.007)):
        b.cyl(f"{path}/v_rivet{k}", HANDLE_ROOT + 0.008 * HANDLE_DIR + (0.0, dy, 0.0) + 0.0032 * HANDLE_UP, 0.003, 0.0016, "rivet", rot_y=deg)
    return prim


def build_die(stage, path):
    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    b.rigid(prim, DIE_MASS, iters=16, depen=0.5, report=True)
    b.box(f"{path}/c_cube", (0.0, 0.0, 0.0), (DIE, DIE, DIE), "potato", "potato", offset=0.006)
    b.box(f"{path}/v_brown", (0.0, 0.0, -DIE / 2), (DIE * 0.96, DIE * 0.96, 0.0006), "brown")
    return prim


def build_burner(stage, path):
    """A fixed, cold hotplate. Origin at the centre of its top face; one flat box collider."""
    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    s, h = BURNER["size"], BURNER["h"]
    b.box(f"{path}/c_housing", (0.0, 0.0, -h / 2), (s, s, h), "housing", "iron")
    b.cyl(f"{path}/v_plate", (0.0, 0.0, 0.0001), BURNER["plate_r"], 0.0004, "iron")
    b.cyl(f"{path}/v_knob", (-s / 2 - 0.004, 0.07, -h / 2), 0.012, 0.008, "knob", axis="X")
    return prim
