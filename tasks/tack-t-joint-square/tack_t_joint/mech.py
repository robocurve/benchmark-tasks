"""The fit-up bench, the coupon and the TIG torch. The geometry at the top is NumPy only, so the task file and the tests use
it without Isaac Sim; the USD builders import pxr when called.

Frames (m):
- Bench: x, y as the world's, z from the station table's top. One fixed (kinematic) body: a steel fixture-table top, the
  base plate with two strap clamps, the torch's rest posts, the coupon's rest block, and the tack beads, which start
  hidden and are shown where a tack forms.
- Coupon: origin at its centre; x along its length (saw-cut ends at +-L/2, where the fingers hold it), y across its width
  (rolled edges at +-W/2, one of which goes down on the base plate), z through its thickness. It starts lying flat
  (z up). It is cut from 3 x 1/4 in hot-rolled flat bar, whose rolled edges are rounded: the collider's rolled edges carry
  a 1.5 mm chamfer each side, so the land it stands on is 3.35 mm wide, not 6.35 mm.
- Torch: origin at its grip block's centre, +x the way the tungsten leans, z up. The same air-cooled torch body as the
  seam-welding task (head axis 17.5 degrees off vertical, 14-18 mm cup, 2.4 mm tungsten), with the tungsten set at 8 mm
  stick-out to reach an inside corner.
"""

import math

import numpy as np

# -- bench ---------------------------------------------------------------------------------------------------------
SLAB = dict(x=(0.115, 0.405), y=(-0.135, 0.135), t=0.012)  # steel fixture-table top on the station table
BASE = dict(x=(0.195, 0.345), y=(-0.110, -0.010), t=0.00635)  # 150 x 100 x 6.35 mm (6 x 4 x 1/4 in) base plate
BT = SLAB["t"] + BASE["t"]  # the base plate's top, above the table
CLAMPS = ((0.195, -0.090, -1), (0.345, -0.090, 1))  # strap clamps: (x of the plate's end, y, side)

# -- coupon ----------------------------------------------------------------------------------------------------------
L = W = 0.0762  # 3 in
T = 0.00635  # 1/4 in
EDGE_R = 0.0015  # the rolled edges' rounding, as a chamfer each side
LAND = T - 2 * EDGE_R  # the flat a rolled edge stands on
COUPON_MASS = 7850.0 * L * W * T  # 0.289 kg
COUPON_HALF = np.array((L / 2, W / 2, T / 2))
REST = dict(x=0.250, y=-0.215, h=0.030, sx=0.040, sy=0.070)  # the coupon's rest block on the table, and its size
COUPON_START = (REST["x"], REST["y"], REST["h"] + T / 2)  # centre, above the table, lying flat

# -- the joint: where the coupon stands on the base plate ------------------------------------------------------------
XC, YC = 0.270, -0.060  # the upright coupon's centre plane
X0, X1 = XC - L / 2, XC + L / 2  # the end ticks
Y_LINE = YC + T / 2  # the scribed line: the coupon's left face (+y, the torch's side) stands on it
MARKS = (X0 + 0.016, XC, X1 - 0.016)  # tack marks: 16 mm (5/8 in) from each end, and the middle

# -- torch (torch frame: origin at the grip block's centre; +x the lean) ---------------------------------------------
TILT = math.radians(17.5)  # the head's axis from vertical
STICKOUT = 0.008  # tungsten past the cup
TUNGSTEN_D = 0.0024
CUP = dict(a0=STICKOUT, a1=STICKOUT + 0.025, r0=0.0070, r1=0.0090)  # along the head's axis from the tungsten tip
HEAD = dict(a0=CUP["a1"], a1=CUP["a1"] + 0.032, r=0.012)
BACKCAP = dict(a0=HEAD["a1"], a1=HEAD["a1"] + 0.010, r=0.008)
HEAD_A = HEAD["a0"] + 0.016  # the handle leaves the head at this point of its axis
HANDLE = dict(r=0.011, len=0.110)
GRIP = dict(x=0.030, y=0.036, z=0.026)  # glass-filled nylon grip block on the handle; the fingers close along x
DG = 0.045  # the grip block's centre is this far behind the head's axis
CABLE = dict(r=0.013, len=0.040)
UP = np.array((-math.sin(TILT), 0.0, math.cos(TILT)))  # up the head's axis
TIP = np.array((DG - HEAD_A * UP[0], 0.0, -(HEAD_A * UP[2] + HANDLE["r"] + GRIP["z"] / 2)))  # tungsten tip, torch frame
HEAD_C = TIP + HEAD_A * UP  # where the handle meets the head
HANDLE_Z = HEAD_C[2]
TORCH_MASS = {"head": 0.110, "handle": 0.060, "grip": 0.030, "cable": 0.055}
TORCH_TOTAL = sum(TORCH_MASS.values())
_cable_x = HEAD_C[0] - HANDLE["len"] - CABLE["len"] / 2
TORCH_COM = (TORCH_MASS["head"] * (TIP + (HEAD_A - 0.011) * UP) + TORCH_MASS["handle"] * np.array((HEAD_C[0] - HANDLE["len"] / 2, 0.0, HANDLE_Z))
             + TORCH_MASS["cable"] * np.array((_cable_x, 0.0, HANDLE_Z))) / TORCH_TOTAL
TORCH_INERTIA = (1.5e-4, 7.0e-4, 6.5e-4)
TORCH_BOXES = {  # axis-aligned colliders in the torch frame
    "grip": ((-GRIP["x"] / 2, -GRIP["y"] / 2, -GRIP["z"] / 2), (GRIP["x"] / 2, GRIP["y"] / 2, GRIP["z"] / 2)),
    "handle": ((HEAD_C[0] - HANDLE["len"], -HANDLE["r"], HANDLE_Z - HANDLE["r"]), (HEAD_C[0], HANDLE["r"], HANDLE_Z + HANDLE["r"])),
    "cable": ((HEAD_C[0] - HANDLE["len"] - CABLE["len"], -CABLE["r"], HANDLE_Z - CABLE["r"]), (HEAD_C[0] - HANDLE["len"], CABLE["r"], HANDLE_Z + CABLE["r"])),
}
_mid = (CUP["a0"] + CUP["a1"]) / 2
AXIS_BOXES = {  # square colliders along the head's axis: (from, to, half-width); each fits inside the round part it stands for
    "head": (HEAD["a0"], HEAD["a1"], HEAD["r"] * 0.75), "cup_lo": (CUP["a0"], _mid, CUP["r0"] * 0.72), "cup_hi": (_mid, CUP["a1"], (CUP["r0"] + CUP["r1"]) / 2 * 0.72),
    "tungsten": (0.0, CUP["a0"] + 0.004, TUNGSTEN_D / 2),
}
TORCH_HULL = (np.array((_cable_x - CABLE["len"] / 2, -GRIP["y"] / 2, TIP[2])), np.array((TIP[0] + 0.003, GRIP["y"] / 2, GRIP["z"] / 2)))
TORCH_REST = dict(x=0.200, y=0.210, lean=0.0)  # the grip's centre and the lean's heading when resting: the tungsten points along +x
TORCH_POST_H = 0.045
TORCH_REST_Z = TORCH_POST_H - TORCH_BOXES["handle"][0][2]  # the grip's centre above the table, handle on the posts
TORCH_POSTS = ((0.020, 0.040), (-0.060, -0.040))  # along the lean from the grip's centre

# -- the tack as a physical joint ------------------------------------------------------------------------------------
N_SLOTS = 8  # joints made ahead, switched on one per tack
TACK_K = 0.5  # N m per degree: a tack is a pin at its point, stiff against bending
TACK_C = 0.005  # N m s per degree
TACK_BREAK_DEG = 2.0  # it cracks when bent this far: 1.0 N m, the plastic moment of a 4 x 1.5 mm fused ligament

COLORS = {  # sRGB base colour, metallic, roughness, emissive
    "slab": ((0.20, 0.21, 0.22), 0.8, 0.55, None), "hole": ((0.03, 0.03, 0.03), 0.0, 0.9, None),
    "base": ((0.33, 0.34, 0.36), 0.7, 0.6, None), "coupon": ((0.50, 0.51, 0.53), 0.9, 0.45, None),
    "edge": ((0.70, 0.71, 0.72), 1.0, 0.3, None), "scribe": ((0.85, 0.86, 0.88), 1.0, 0.2, None),
    "white": ((0.95, 0.95, 0.92), 0.0, 0.7, None), "clamp": ((0.12, 0.30, 0.62), 0.4, 0.45, None),
    "bolt": ((0.55, 0.56, 0.58), 1.0, 0.35, None), "post": ((0.42, 0.43, 0.45), 0.9, 0.4, None),
    "cup": ((0.93, 0.72, 0.68), 0.0, 0.65, None), "head": ((0.06, 0.06, 0.07), 0.0, 0.5, None),
    "collet": ((0.80, 0.55, 0.25), 1.0, 0.3, None), "tungsten": ((0.62, 0.62, 0.66), 1.0, 0.3, None),
    "grip": ((0.80, 0.20, 0.08), 0.0, 0.55, None), "cable": ((0.04, 0.04, 0.04), 0.0, 0.7, None),
    "arc": ((0.75, 0.85, 1.0), 0.0, 0.3, (3.0, 4.0, 6.0)), "soot": ((0.18, 0.17, 0.16), 0.0, 0.95, None),
    "tack": ((0.74, 0.62, 0.42), 1.0, 0.28, None),
}
MU = {"steel": 0.5, "grip": 1.0, "rubber": 0.9}


# -- USD -------------------------------------------------------------------------------------------------------------
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
            color, metal, rough, emit = COLORS[name]
            mat = UsdShade.Material.Define(self.stage, f"{self.root}/Looks/{name}")
            sh = UsdShade.Shader.Define(self.stage, f"{self.root}/Looks/{name}/shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*_lin(color)))
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metal))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
            if emit is not None:
                sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emit))
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
            self.phys[name] = mat
        return self.phys[name]

    def _finish(self, g, look, collide, offset=0.001):
        from pxr import UsdPhysics, UsdShade

        prim = g.GetPrim()
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.look(look))
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            _physx(prim, "PhysxCollisionAPI", **{"physxCollision:contactOffset": float(offset), "physxCollision:restOffset": 0.0})
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.surface(collide), UsdShade.Tokens.weakerThanDescendants, "physics")
        return g

    @staticmethod
    def _xform(g, center, rot_y=0.0, rot_z=0.0, scale=None):
        from pxr import Gf

        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        if rot_z:
            g.AddRotateZOp().Set(float(rot_z))
        if rot_y:
            g.AddRotateYOp().Set(float(rot_y))
        if scale is not None:
            g.AddScaleOp().Set(Gf.Vec3f(*map(float, scale)))

    def box(self, path, lo, hi, look, collide=None, rot_y=0.0, rot_z=0.0, offset=0.001):
        """A box (low, high) before turning; rot_y, rot_z (degrees) turn it about its own centre."""
        from pxr import UsdGeom

        lo, hi = np.array(lo, float), np.array(hi, float)
        c = UsdGeom.Cube.Define(self.stage, path)
        c.CreateSizeAttr(1.0)
        self._xform(c, (lo + hi) / 2, rot_y, rot_z, hi - lo)
        return self._finish(c, look, collide, offset)

    def cyl(self, path, center, r, h, look, axis="Z", rot_y=0.0):
        from pxr import UsdGeom

        c = UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateAxisAttr(axis)
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(h))
        self._xform(c, center, rot_y)
        return self._finish(c, look, None)

    def ellipsoid(self, path, center, radii, look):
        from pxr import UsdGeom

        s = UsdGeom.Sphere.Define(self.stage, path)
        s.CreateRadiusAttr(1.0)
        self._xform(s, center, scale=radii)
        return self._finish(s, look, None)

    def frustum(self, path, base, r0, r1, h, look, rot_y=0.0, n=24):
        """A closed cone frustum along its own z from base (radius r0) to h (radius r1), turned rot_y about y at the base."""
        from pxr import Gf, UsdGeom, Vt

        m = UsdGeom.Mesh.Define(self.stage, path)
        pts = [Gf.Vec3f(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n), z) for z, r in ((0.0, r0), (h, r1)) for k in range(n)]
        pts += [Gf.Vec3f(0, 0, 0), Gf.Vec3f(0, 0, h)]
        counts, idx = [], []
        for k in range(n):
            j = (k + 1) % n
            counts += [4, 3, 3]
            idx += [k, j, n + j, n + k, 2 * n, j, k, 2 * n + 1, n + k, n + j]
        m.CreatePointsAttr(Vt.Vec3fArray(pts))
        m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        m.CreateSubdivisionSchemeAttr("none")
        self._xform(m, base, rot_y)
        return self._finish(m, look, None)

    def rigid(self, prim, kinematic=False, mass=None, com=None, inertia=None, iters=32):
        from pxr import Gf, UsdPhysics

        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(kinematic)
        _physx(prim, "PhysxRigidBodyAPI", **{"physxRigidBody:solverPositionIterationCount": iters, "physxRigidBody:solverVelocityIterationCount": 2,
                                             "physxRigidBody:maxDepenetrationVelocity": 0.2, "physxRigidBody:enableCCD": True})
        if mass is not None:
            m = UsdPhysics.MassAPI.Apply(prim)
            m.CreateMassAttr(float(mass))
            if com is not None:
                m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
            if inertia is not None:
                m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))


def _hide(stage, path):
    from pxr import UsdGeom

    UsdGeom.Imageable(stage.GetPrimAtPath(path)).MakeInvisible()


def build_coupon(stage, path):
    """The coupon: three box colliders (the full-thickness middle and the two chamfered rolled edges), drawn the same."""
    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    b.rigid(prim, mass=COUPON_MASS, inertia=tuple(COUPON_MASS / 12 * v for v in (W * W + T * T, L * L + T * T, L * L + W * W)))
    _physx(prim, "PhysxContactReportAPI", **{"physxContactReport:threshold": 0.0})  # so its contact forces can be read
    hx, hy, hz = COUPON_HALF
    b.box(f"{path}/body", (-hx, -hy + EDGE_R, -hz), (hx, hy - EDGE_R, hz), "coupon", "steel")
    for name, s in (("edge_a", -1), ("edge_b", 1)):
        y0, y1 = sorted((s * (hy - EDGE_R), s * hy))
        b.box(f"{path}/{name}", (-hx, y0, -LAND / 2), (hx, y1, LAND / 2), "edge", "steel")
    return prim


def _axis_box(b, path, a0, a1, half, look, collide):
    """A square box along the torch head's axis, from a0 to a1 above the tungsten tip."""
    c = TIP + (a0 + a1) / 2 * UP
    return b.box(path, c - (half, half, (a1 - a0) / 2), c + (half, half, (a1 - a0) / 2), look, collide, rot_y=-math.degrees(TILT))


def build_torch(stage, path):
    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    b.rigid(prim, mass=TORCH_TOTAL, com=TORCH_COM, inertia=TORCH_INERTIA)
    tilt = -math.degrees(TILT)
    for name, (lo, hi) in TORCH_BOXES.items():
        b.box(f"{path}/c_{name}", lo, hi, "grip" if name == "grip" else "cable", "grip" if name == "grip" else "rubber")
    for name, (a0, a1, half) in AXIS_BOXES.items():
        _axis_box(b, f"{path}/c_{name}", a0, a1, half, "head", "steel")
    for n in ("handle", "cable", *AXIS_BOXES):
        _hide(stage, f"{path}/c_{n}")
    hx0, hx1 = HEAD_C[0] - HANDLE["len"], HEAD_C[0]
    b.cyl(f"{path}/v_handle", ((hx0 + hx1) / 2, 0.0, HANDLE_Z), HANDLE["r"], HANDLE["len"], "head", axis="X")
    for k in range(6):
        b.cyl(f"{path}/v_ring{k}", (hx0 + 0.010 + 0.006 * k, 0.0, HANDLE_Z), HANDLE["r"] + 0.0008, 0.003, "cable", axis="X")
    b.cyl(f"{path}/v_cable", (hx0 - CABLE["len"] / 2, 0.0, HANDLE_Z), CABLE["r"], CABLE["len"], "cable", axis="X")
    b.cyl(f"{path}/v_hose", (hx0 - CABLE["len"] - 0.006, 0.0, HANDLE_Z), 0.007, 0.012, "cable", axis="X")
    b.box(f"{path}/v_grip_cap", (-GRIP["x"] / 2 + 0.002, -GRIP["y"] / 2 + 0.002, GRIP["z"] / 2), (GRIP["x"] / 2 - 0.002, GRIP["y"] / 2 - 0.002, GRIP["z"] / 2 + 0.0015), "cable")
    for name, (a0, a1, r, look) in {"v_head": (HEAD["a0"], HEAD["a1"], HEAD["r"], "head"), "v_collet": (CUP["a1"] - 0.002, HEAD["a0"] + 0.004, 0.0075, "collet"),
                                    "v_backcap": (BACKCAP["a0"], BACKCAP["a1"], BACKCAP["r"], "head")}.items():
        b.cyl(f"{path}/{name}", TIP + (a0 + a1) / 2 * UP, r, a1 - a0, look, rot_y=tilt)
    b.ellipsoid(f"{path}/v_backcap_end", TIP + BACKCAP["a1"] * UP, (BACKCAP["r"], BACKCAP["r"], 0.004), "head")
    b.frustum(f"{path}/v_cup", TIP + CUP["a0"] * UP, CUP["r0"], CUP["r1"], CUP["a1"] - CUP["a0"], "cup", rot_y=tilt)
    b.cyl(f"{path}/v_tungsten", TIP + (0.0015 + CUP["a0"] + 0.004) / 2 * UP, TUNGSTEN_D / 2, CUP["a0"] + 0.004 - 0.0015, "tungsten", rot_y=tilt)
    b.frustum(f"{path}/v_tip", TIP, 0.0002, TUNGSTEN_D / 2, 0.0015, "tungsten", rot_y=tilt, n=12)
    b.ellipsoid(f"{path}/fx_arc", TIP - 0.0012 * UP, (0.0028, 0.0028, 0.0028), "arc")  # shown while the arc is on
    b.frustum(f"{path}/fx_soot", TIP + (CUP["a0"] - 0.0003) * UP, CUP["r0"] + 0.0006, CUP["r0"] + 0.0012, 0.008, "soot", rot_y=tilt)  # after contamination
    for n in ("fx_arc", "fx_soot"):
        _hide(stage, f"{path}/{n}")
    return prim


def build_bench(stage, path):
    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    b.rigid(prim, kinematic=True)
    (sx0, sx1), (sy0, sy1), st = SLAB["x"], SLAB["y"], SLAB["t"]
    b.box(f"{path}/slab", (sx0, sy0, 0.0), (sx1, sy1, st), "slab", "steel")
    for i in range(6):  # the fixture table's hole grid, 50 mm pitch
        for j in range(6):
            x, y = sx0 + 0.02 + 0.05 * i, sy0 + 0.02 + 0.05 * j
            if not (BASE["x"][0] - 0.008 < x < BASE["x"][1] + 0.008 and BASE["y"][0] - 0.008 < y < BASE["y"][1] + 0.008):
                b.cyl(f"{path}/hole_{i}_{j}", (x, y, st + 0.0002), 0.007, 0.0004, "hole")
    (bx0, bx1), (by0, by1) = BASE["x"], BASE["y"]
    b.box(f"{path}/base", (bx0, by0, st), (bx1, by1, BT), "base", "steel")
    # the scribed joint line, its end ticks, and a white paint tick beside it at each tack mark (all flat, drawn only)
    b.box(f"{path}/scribe", (X0 - 0.012, Y_LINE - 0.0004, BT), (X1 + 0.012, Y_LINE + 0.0004, BT + 0.0002), "scribe")
    for k, x in enumerate((X0, X1)):
        b.box(f"{path}/tick{k}", (x - 0.0004, Y_LINE - 0.012, BT), (x + 0.0004, Y_LINE + 0.012, BT + 0.0002), "scribe")
    for k, x in enumerate(MARKS):
        b.box(f"{path}/mark{k}", (x - 0.0015, Y_LINE + 0.010, BT), (x + 0.0015, Y_LINE + 0.026, BT + 0.0002), "white")
    # strap clamps on the base plate's two ends: bar, heel block, stud and nut (the bar and heel collide)
    for k, (cx, cy, s) in enumerate(CLAMPS):
        b.box(f"{path}/clamp{k}_bar", (min(cx - s * 0.018, cx + s * 0.046), cy - 0.010, BT), (max(cx - s * 0.018, cx + s * 0.046), cy + 0.010, BT + 0.008), "clamp", "steel")
        hx = cx + s * 0.038
        b.box(f"{path}/clamp{k}_heel", (hx - 0.008, cy - 0.010, st), (hx + 0.008, cy + 0.010, BT), "clamp", "steel")
        b.cyl(f"{path}/clamp{k}_stud", (cx + s * 0.016, cy, st + 0.016), 0.004, 0.032, "bolt")
        b.cyl(f"{path}/clamp{k}_nut", (cx + s * 0.016, cy, BT + 0.011), 0.0075, 0.006, "bolt")
    # the torch's rest: two steel posts under the handle, on a steel base
    c, s = math.cos(TORCH_REST["lean"]), math.sin(TORCH_REST["lean"])
    for k, (a0, a1) in enumerate(TORCH_POSTS):
        am = (a0 + a1) / 2
        px, py = TORCH_REST["x"] + am * c, TORCH_REST["y"] + am * s
        hx, hy = abs(c) * (a1 - a0) / 2 + abs(s) * 0.018, abs(s) * (a1 - a0) / 2 + abs(c) * 0.018
        b.box(f"{path}/torch_post{k}", (px - hx, py - hy, 0.0), (px + hx, py + hy, TORCH_POST_H), "post", "steel")
    bx = sorted((TORCH_REST["x"] - 0.055 * c, TORCH_REST["x"] + 0.075 * c))
    b.box(f"{path}/torch_base", (bx[0], TORCH_REST["y"] - 0.025, 0.0), (bx[1], TORCH_REST["y"] + 0.025, 0.004), "post", "steel")
    # the coupon's rest: a steel block narrower than the coupon, so a hand can take the coupon by its two ends
    rx, ry = REST["x"], REST["y"]
    b.box(f"{path}/coupon_rest", (rx - REST["sx"] / 2, ry - REST["sy"] / 2, 0.0), (rx + REST["sx"] / 2, ry + REST["sy"] / 2, REST["h"]), "post", "steel")
    for k in range(N_SLOTS):  # tack beads, moved to each tack and shown when it forms
        b.ellipsoid(f"{path}/tack{k}", (XC, Y_LINE, BT), (0.0045, 0.0022, 0.0022), "tack")
        _hide(stage, f"{path}/tack{k}")
    return prim


def build_tack_joints(stage, coupon_path, root="/World/TackJoints"):
    """N_SLOTS joints from the world to the coupon, switched off. Each is a tack: the three translations locked (a pin
    at the tack's point), the three rotations held by a stiff spring. The embodiment sets a joint's frames to the coupon's
    pose when its tack forms and switches it on; the task switches it off when the tack is bent past TACK_BREAK_DEG."""
    from pxr import Gf, Sdf, UsdPhysics

    joints = []
    for k in range(N_SLOTS):
        j = UsdPhysics.Joint.Define(stage, f"{root}/tack{k}")
        j.CreateJointEnabledAttr(False)
        j.CreateBody1Rel().SetTargets([Sdf.Path(coupon_path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot0Attr(Gf.Quatf(1.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0))
        j.CreateCollisionEnabledAttr(True)
        j.CreateExcludeFromArticulationAttr(True)
        for axis in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), axis)
            lim.CreateLowAttr(1.0)  # low above high: locked
            lim.CreateHighAttr(-1.0)
        for axis in ("rotX", "rotY", "rotZ"):
            d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), axis)
            d.CreateTypeAttr("force")
            d.CreateStiffnessAttr(TACK_K)
            d.CreateDampingAttr(TACK_C)
            d.CreateTargetPositionAttr(0.0)
            d.CreateTargetVelocityAttr(0.0)
        joints.append(j)
    return joints


def lock_tack(joint, anchor_w, coupon_pos_w, coupon_quat_xyzw):
    """Set a tack joint at ``anchor_w`` (world, with the env origin) for the coupon's pose now, and switch it on."""
    from pxr import Gf

    x, y, z, w = (float(v) for v in coupon_quat_xyzw)
    R = np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)), (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                  (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))
    p1 = R.T @ (np.asarray(anchor_w, float) - np.asarray(coupon_pos_w, float))
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(*map(float, anchor_w)))
    joint.GetLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.GetLocalPos1Attr().Set(Gf.Vec3f(*map(float, p1)))
    joint.GetLocalRot1Attr().Set(Gf.Quatf(w, Gf.Vec3f(-x, -y, -z)))  # the world's axes, seen from the coupon
    joint.GetJointEnabledAttr().Set(True)
