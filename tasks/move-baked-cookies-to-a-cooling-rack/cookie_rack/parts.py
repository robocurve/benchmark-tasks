"""The sheet pan, the cookies, the wire cooling rack, the offset turner and its rest.

Created 11_15_26 18:20 EST. Geometry constants are NumPy only, so the task file and the tests use them without Isaac Sim;
the USD builders import pxr when called.

Frames (m), all with z up:
- Pan: origin at the centre of its underside. A quarter sheet pan, long side along x: flat floor 305 x 216 mm, four
  flared walls to a 330 x 241 mm rim 25 mm up. One loose body. Its underside and its inside are separate colliders,
  because bare aluminium skates on a counter while a baked cookie's sugary base does not skate on the pan.
- Cookie: origin at the centre of its base. 76 mm across, 10 mm thick, 35 g, one rigid body. The base's outer 6 mm
  curls up 1.5 mm, as a drop cookie's edge does; that curl is the only way under it. It is brittle by a criterion fed with
  the contact forces PhysX reports on it (see `Loads` in embodiment.py): it breaks when the bending moment across any
  diameter passes BREAK_BEND, or the force squeezing it across any diameter passes BREAK_SQUEEZE. A broken cookie
  shows a crack.
- Rack: origin at the centre of its footprint. 305 x 216 mm, 2 mm wires 10 mm apart along x on three cross bars, a 4 mm
  frame, two loop feet. One loose body.
- Turner: origin at the middle of its handle, +x toward the blade's front edge. Handle 110 x 22 x 14 mm, a neck that
  drops 30 mm, a 90 x 65 mm stainless blade 0.6 mm thick. One loose body.
- Rest: origin on the table under the turner's origin. Two cradle posts under the handle and a pad under the blade,
  so the turner lies level; fixed to the counter.
"""

import math
import os

import numpy as np

# -- pan -----------------------------------------------------------------------------------------------------------
PAN_IN = (0.305, 0.216)  # flat floor, x by y
PAN_OUT = (0.330, 0.241)  # rim, x by y
PAN_RIM = 0.025
PAN_FLOOR_T = 0.0012  # 18 gauge aluminium, in two layers of collider
PAN_WALL_T = 0.0015
PAN_MASS = 0.40

# -- cookie --------------------------------------------------------------------------------------------------------
COOKIE_R = 0.038
COOKIE_T = 0.010
COOKIE_CURL = (0.006, 0.0015)  # the base's outer band: run and rise
COOKIE_CROWN = 0.005  # the top's outer band rounds down over this run, by 2 mm
COOKIE_MASS = 0.035
BREAK_BEND = 0.02  # N m of bending across a diameter: a 2 N load at the middle of a 40 mm span
BREAK_SQUEEZE = 3.0  # N pressed through the cookie from edge to edge, about ten times its weight
BREAK_PRESS = 10.0  # N pressed down on its top
LOAD_WINDOW = 4  # physics steps (1/60 s) the measured loads are averaged over before they are compared
DIAMETERS = 8  # the diameters the two loads are worked out across

# -- rack ----------------------------------------------------------------------------------------------------------
RACK = (0.305, 0.216)
WIRE_R = 0.001
WIRE_PITCH = 0.010
WIRE_Z = 0.016  # wire centre above the table
RACK_TOP = WIRE_Z + WIRE_R
FRAME_R = 0.002
RACK_MASS = 0.22

# -- turner (origin: handle centre; +x to the blade's front edge) ------------------------------------------------------
HANDLE = (0.110, 0.022, 0.014)
NECK_RUN, NECK_DROP = 0.030, 0.030
BLADE_L, BLADE_W, BLADE_T = 0.090, 0.065, 0.0006
BLADE_X0 = HANDLE[0] / 2 + NECK_RUN  # blade's heel
BLADE_TOP = -NECK_DROP  # blade's top face, below the handle's centre line
TIP = np.array((BLADE_X0 + BLADE_L, 0.0, BLADE_TOP - BLADE_T))  # middle of the front edge, underside
TURNER_MASS = {"handle": 0.035, "neck": 0.006, "blade": 0.0275}  # polypropylene handle; 0.6 mm 18/8 steel
TURNER_TOTAL = sum(TURNER_MASS.values())

# -- rest ----------------------------------------------------------------------------------------------------------
POST_H = 0.032
POST_X = (-0.045, 0.045)  # under the handle, either side of where the fingers close
POST = (0.010, 0.040)  # x by y
LIP = (0.003, 0.004)  # y thickness, height above the post: keeps the handle from rolling off

# friction: PhysX averages the two surfaces' coefficients, so these give cookie on pan 0.70, cookie on blade 0.25,
# cookie on rack 0.50, blade on pan 0.55. The pan's underside takes the lower of the pair (0.25 on the table).
MU = {"cookie": 0.4, "pan_in": 1.0, "pan_under": 0.25, "blade": 0.1, "rack": 0.6, "handle": 1.0, "rest": 0.6}
COMBINE_MIN = ("pan_under",)

COLORS = {  # sRGB base colour, metallic, roughness
    "pan": ((0.74, 0.75, 0.76), 1.0, 0.45), "cookie": ((0.78, 0.56, 0.30), 0.0, 0.9), "chip": ((0.22, 0.12, 0.07), 0.0, 0.6), "crack": ((0.9, 0.05, 0.05), 0.0, 0.8),
    "wire": ((0.80, 0.81, 0.83), 1.0, 0.3), "blade": ((0.86, 0.87, 0.88), 1.0, 0.25), "handle": ((0.08, 0.08, 0.09), 0.0, 0.5),
    "rest": ((0.25, 0.42, 0.62), 0.0, 0.6),
}
CHIPS = ((0.012, 0.016), (-0.015, 0.010), (0.020, -0.012), (-0.008, -0.020), (-0.024, -0.006), (0.004, 0.027), (0.026, 0.006))


def cookie_hull(n=12):
    """The cookie's vertices in its own frame: four rings, 48 points (PhysX's GPU hulls take up to 64)."""
    run, rise = COOKIE_CURL
    rings = ((COOKIE_R - run, 0.0), (COOKIE_R, rise), (COOKIE_R, COOKIE_T - 0.002), (COOKIE_R - COOKIE_CROWN, COOKIE_T))
    return np.array([(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n), z) for r, z in rings for k in range(n)])


def loads(points, forces, weight_up=1.0):
    """The bending moment (N m) and the squeeze (N) across each of DIAMETERS diameters, and the press (N), from the contact forces on one
    cookie: points (n, 3) and forces (n, 3) in the cookie's frame (z up through the cookie), forces acting on it.
    Bending across a diameter is the moment about that line of everything on one side of it: the vertical contact
    forces there, less that half's weight (weight_up: how much of gravity lies along the cookie's own z). Squeeze is
    the force pressed inward through the line from both sides, so a one-sided push that only slides the cookie is none.
    Press is the total force pushing down on the cookie (toward its base)."""
    points, forces = np.asarray(points, float).reshape(-1, 3), np.asarray(forces, float).reshape(-1, 3)
    half_w = COOKIE_MASS * 9.81 / 2 * weight_up
    arm = 4 * COOKIE_R / (3 * math.pi)
    bend, squeeze = np.zeros(DIAMETERS), np.zeros(DIAMETERS)
    for k in range(DIAMETERS):
        a = math.pi * k / DIAMETERS
        n = np.array((math.cos(a), math.sin(a)))
        d = points[:, :2] @ n  # signed distance from the diameter
        inward = -(forces[:, :2] @ n) * np.sign(d)  # each force's push toward the line
        sides = []
        for s in (1.0, -1.0):
            m = s * d > 0
            sides.append((float((s * d[m] * forces[m, 2]).sum()) - half_w * arm, float(np.clip(inward[m], 0.0, None).sum())))
        bend[k] = max(abs(sides[0][0]), abs(sides[1][0])) if len(points) else 0.0
        squeeze[k] = min(sides[0][1], sides[1][1])
    return bend, squeeze, float(np.clip(-forces[:, 2], 0.0, None).sum())


def wire_ys():
    n = int(round((RACK[1] - 2 * FRAME_R) / WIRE_PITCH)) - 1
    return [(-(n - 1) / 2 + k) * WIRE_PITCH for k in range(n)]


# -- USD -------------------------------------------------------------------------------------------------------------
def _lin(c):
    return tuple(v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in c)


def _api(prim, schema, **attrs):
    from pxr import Sdf

    if not prim.ApplyAPI(schema):
        prim.AddAppliedSchema(schema)
    types = {bool: Sdf.ValueTypeNames.Bool, int: Sdf.ValueTypeNames.Int, float: Sdf.ValueTypeNames.Float}
    for name, value in attrs.items():
        prim.CreateAttribute(name, types[type(value)]).Set(value)


class Builder:
    def __init__(self, stage, root):
        self.stage, self.root, self.looks, self.phys = stage, root, {}, {}

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
            from pxr import Sdf

            mat.GetPrim().AddAppliedSchema("PhysxMaterialAPI")  # patch friction that holds a resting body still
            mat.GetPrim().CreateAttribute("physxMaterial:improvePatchFriction", Sdf.ValueTypeNames.Bool).Set(os.environ.get("COOKIE_PATCH") == "1")
            if name in COMBINE_MIN:
                mat.GetPrim().CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set("min")
            self.phys[name] = mat
        return self.phys[name]

    def _finish(self, g, look, collide, offset):
        from pxr import UsdPhysics, UsdShade

        prim = g.GetPrim()
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.look(look))
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            _api(prim, "PhysxCollisionAPI", **{"physxCollision:contactOffset": float(offset), "physxCollision:restOffset": 0.0})
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.surface(collide), UsdShade.Tokens.weakerThanDescendants, "physics")
        return g

    def box(self, name, lo, hi, look, collide=None, rot=None, offset=0.001):
        """A box (low, high) before turning; rot = (axis, degrees) turns it about its own centre."""
        from pxr import Gf, UsdGeom

        lo, hi = np.array(lo, float), np.array(hi, float)
        g = UsdGeom.Cube.Define(self.stage, f"{self.root}/{name}")
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, (lo + hi) / 2)))
        if rot is not None:
            {"x": g.AddRotateXOp, "y": g.AddRotateYOp, "z": g.AddRotateZOp}[rot[0]]().Set(float(rot[1]))
        g.AddScaleOp().Set(Gf.Vec3f(*map(float, hi - lo)))
        return self._finish(g, look, collide, offset)

    def rod(self, name, center, r, length, axis, look, collide=None, offset=0.001):
        """A round rod with rounded ends (a capsule, exact in PhysX) along axis."""
        from pxr import Gf, UsdGeom

        g = UsdGeom.Capsule.Define(self.stage, f"{self.root}/{name}")
        g.CreateAxisAttr(axis)
        g.CreateRadiusAttr(float(r))
        g.CreateHeightAttr(float(length - 2 * r))
        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        return self._finish(g, look, collide, offset)

    def disc(self, name, center, r, h, look):
        from pxr import Gf, UsdGeom

        g = UsdGeom.Cylinder.Define(self.stage, f"{self.root}/{name}")
        g.CreateAxisAttr("Z")
        g.CreateRadiusAttr(float(r))
        g.CreateHeightAttr(float(h))
        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        return self._finish(g, look, None, 0.0)

    def hull(self, name, pts, look, collide, offset=0.002, rest=0.0):
        """A convex solid from its vertices (collides as its convex hull)."""
        from pxr import Gf, UsdGeom, UsdPhysics, Vt
        from scipy.spatial import ConvexHull

        pts = np.asarray(pts, float)
        h = ConvexHull(pts)
        c = pts.mean(0)
        faces = []
        for s, eq in zip(h.simplices, h.equations):
            a, b, d = pts[s]
            faces.append(s if np.dot(np.cross(b - a, d - a), eq[:3]) > 0 else s[::-1])
        m = UsdGeom.Mesh.Define(self.stage, f"{self.root}/{name}")
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in pts]))
        m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for f in faces for i in f]))
        m.CreateSubdivisionSchemeAttr("none")
        del c
        self._finish(m, look, collide, offset)
        m.GetPrim().GetAttribute("physxCollision:restOffset").Set(float(rest))
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
        return m

    def rigid(self, mass, iters=48):
        from pxr import UsdPhysics

        prim = self.stage.GetPrimAtPath(self.root)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        _api(prim, "PhysxRigidBodyAPI", **{"physxRigidBody:solverPositionIterationCount": iters, "physxRigidBody:solverVelocityIterationCount": 4,
                                           "physxRigidBody:maxDepenetrationVelocity": 0.3, "physxRigidBody:enableCCD": True,
                                           "physxRigidBody:sleepThreshold": 0.0})
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        return prim


def build_pan(stage, path):
    b = Builder(stage, path)
    lx, ly = PAN_IN[0] / 2, PAN_IN[1] / 2
    t = PAN_FLOOR_T
    b.box("under", (-lx, -ly, 0.0), (lx, ly, t / 2), "pan", "pan_under")
    b.box("floor", (-lx, -ly, t / 2), (lx, ly, t), "pan", "pan_in")
    flare = (PAN_OUT[0] - PAN_IN[0]) / 2
    slant, lean = math.hypot(PAN_RIM, flare), math.degrees(math.atan2(flare, PAN_RIM))
    for name, sx, sy in (("wall_xp", 1, 0), ("wall_xn", -1, 0), ("wall_yp", 0, 1), ("wall_yn", 0, -1)):
        if sx:
            c = (sx * (lx + flare / 2), 0.0, t + PAN_RIM / 2)
            half = (PAN_WALL_T / 2, PAN_OUT[1] / 2, slant / 2)
            rot = ("y", sx * lean)
        else:
            c = (0.0, sy * (ly + flare / 2), t + PAN_RIM / 2)
            half = (PAN_OUT[0] / 2, PAN_WALL_T / 2, slant / 2)
            rot = ("x", -sy * lean)
        b.box(name, np.array(c) - half, np.array(c) + half, "pan", "pan_in", rot=rot)
    return b.rigid(PAN_MASS)


def build_cookie(stage, path):
    from pxr import UsdGeom

    b = Builder(stage, path)
    b.hull("crumb", cookie_hull(), "cookie", "cookie", rest=float(os.environ.get("COOKIE_REST", "0.0003")))
    for k, (x, y) in enumerate(CHIPS):
        b.disc(f"chip_{k}", (x, y, COOKIE_T), 0.0035, 0.002, "chip")
    crack = b.box("crack", (-COOKIE_R, -0.0012, COOKIE_T - 0.001), (COOKIE_R, 0.0012, COOKIE_T + 0.0012), "crack")
    UsdGeom.Imageable(crack.GetPrim()).MakeInvisible()
    prim = b.rigid(COOKIE_MASS)
    _api(prim, "PhysxContactReportAPI", **{"physxContactReport:threshold": 0.0})
    return prim


def build_rack(stage, path):
    b = Builder(stage, path)
    lx, ly = RACK[0] / 2, RACK[1] / 2
    for k, y in enumerate(wire_ys()):
        b.rod(f"wire_{k:02d}", (0.0, y, WIRE_Z), WIRE_R, RACK[0] - 2 * FRAME_R, "X", "wire", "rack")
    for name, c, axis, length in (("frame_yp", (0.0, ly - FRAME_R, WIRE_Z), "X", RACK[0]), ("frame_yn", (0.0, -ly + FRAME_R, WIRE_Z), "X", RACK[0]),
                                  ("frame_xp", (lx - FRAME_R, 0.0, WIRE_Z), "Y", RACK[1]), ("frame_xn", (-lx + FRAME_R, 0.0, WIRE_Z), "Y", RACK[1])):
        b.rod(name, c, FRAME_R, length, axis, "wire", "rack")
    for k, x in enumerate((-0.10, 0.0, 0.10)):
        b.rod(f"bar_{k}", (x, 0.0, WIRE_Z - WIRE_R - 0.0015), 0.0015, RACK[1] - 2 * FRAME_R, "Y", "wire", "rack")
    for k, x in enumerate((-lx + 0.02, lx - 0.02)):
        b.box(f"foot_{k}", (x - 0.0015, -ly + 0.01, 0.0), (x + 0.0015, ly - 0.01, WIRE_Z - FRAME_R), "wire", "rack")
    return b.rigid(RACK_MASS)


def build_turner(stage, path):
    from pxr import Gf, UsdPhysics

    b = Builder(stage, path)
    hx, hy, hz = (v / 2 for v in HANDLE)
    b.box("handle", (-hx, -hy, -hz), (hx, hy, hz), "handle", "handle", offset=0.002)
    slope = math.hypot(NECK_RUN, NECK_DROP)
    c = np.array((hx + NECK_RUN / 2, 0.0, -NECK_DROP / 2))
    half = np.array((slope / 2 + 0.002, 0.010, 0.001))
    b.box("neck", c - half, c + half, "blade", "blade", rot=("y", math.degrees(math.atan2(NECK_DROP, NECK_RUN))))
    b.box("blade", (BLADE_X0, -BLADE_W / 2, BLADE_TOP - BLADE_T), (BLADE_X0 + BLADE_L, BLADE_W / 2, BLADE_TOP), "blade", "blade", offset=0.0008)
    prim = b.rigid(TURNER_TOTAL, iters=64)
    com = (TURNER_MASS["neck"] * c + TURNER_MASS["blade"] * np.array((BLADE_X0 + BLADE_L / 2, 0.0, BLADE_TOP))) / TURNER_TOTAL
    UsdPhysics.MassAPI(prim).CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
    return prim


def build_rest(stage, path):
    b = Builder(stage, path)
    for k, x in enumerate(POST_X):
        b.box(f"post_{k}", (x - POST[0] / 2, -POST[1] / 2, 0.0), (x + POST[0] / 2, POST[1] / 2, POST_H), "rest", "rest")
        for s in (-1, 1):
            y = s * (HANDLE[1] / 2 + 0.0015 + LIP[0] / 2)
            b.box(f"lip_{k}_{'n' if s < 0 else 'p'}", (x - POST[0] / 2, y - LIP[0] / 2, POST_H), (x + POST[0] / 2, y + LIP[0] / 2, POST_H + LIP[1]), "rest", "rest")
    top = POST_H + HANDLE[2] / 2 + TIP[2]  # the blade's underside when the handle lies on the posts
    b.box("pad", (BLADE_X0 + 0.015, -0.025, 0.0), (TIP[0] - 0.015, 0.025, top), "rest", "rest")
    return stage.GetPrimAtPath(path)
