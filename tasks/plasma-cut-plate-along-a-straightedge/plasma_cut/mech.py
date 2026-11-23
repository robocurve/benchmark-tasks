"""The cutting stand, the clamped plate and its offcut, the hand plasma torch and the straightedge. Geometry constants at
the top are NumPy only, so the task file and the tests can use them without Isaac Sim; the USD builders import pxr when called.

Frames (m):
- Bench: x, y as the world's, z from the station table's top. One kinematic body: the steel stand, the kept part of the
  plate (clamped), two strap clamps, the dross tray, the torch's rest posts and the straightedge's rest pad, plus the kerf
  pieces, which start hidden and are switched on from the task's note_step.
- Offcut: the overhanging part of the same 3 mm plate, a separate loose body. One fixed joint per 5 mm of the cut line
  ties it to the bench; the task switches a joint off when its segment is cut through, so the offcut drops only when the
  whole line is cut. Each joint breaks at the load that would fold or tear a 5 x 3 mm steel bridge (BREAK_TORQUE,
  BREAK_FORCE), so a last bridge or two can be snapped by pushing, as on real plate; the scorer never counts that as cut.
- Torch: origin at its grip block's centre, +x from the handle toward the head, z up. A vertical head with the drag shield
  at the bottom (the nozzle's orifice is recessed 2 mm above the shield's rim), a horizontal handle with the grip block on
  it, and a stub of lead behind as the counterweight, so the grip is over the balance point.
- Straightedge: origin at its handle block's centre, +x along the bar. A 25 x 5 mm steel flat bar whose working edge is
  on the -y side, a steel tab behind it and a nylon handle block on the tab.
"""

import math

import numpy as np

# -- bench ---------------------------------------------------------------------------------------------------------
STAND = dict(x=(0.160, 0.380), y=(-0.045, 0.100), h=0.040)  # welded box-section cutting stand
PLATE_T = 0.003
PT = STAND["h"] + PLATE_T  # the plate's top, above the table
X0, X1 = 0.170, 0.370  # the plate's near and far edges: the cut runs the full 200 mm between them
LINE_Y = -0.060  # the scribed cut line
KEPT_Y1 = 0.100  # the kept part runs from the line to here
OFF_W = 0.055  # the offcut: 200 x 55 x 3 mm, overhanging the stand
GAP = 0.0004  # drawn/collision gap between kept part and offcut at the line
# One uncut 5 x 3 mm bridge of mild steel (yield 250 MPa, tensile 400 MPa): it folds at its plastic bending moment,
# 250e6 x 0.005 x 0.003^2 / 4 = 2.8 N m, and tears at 400e6 x 15e-6 = 6 kN. Each joint breaks at these.
BREAK_TORQUE = 250e6 * 0.005 * PLATE_T**2 / 4
BREAK_FORCE = 400e6 * 0.005 * PLATE_T
SEG_L = 0.005
N_SEG = round((X1 - X0) / SEG_L)
OFFCUT = dict(x=(X0, X1), y=(LINE_Y - OFF_W, LINE_Y - GAP))
OFFCUT_MASS = 7850.0 * (X1 - X0) * (OFF_W - GAP) * PLATE_T  # 0.257 kg
OFFCUT_C = np.array(((X0 + X1) / 2, (OFFCUT["y"][0] + OFFCUT["y"][1]) / 2, PT - PLATE_T / 2))  # its centre, bench frame
CLAMPS = ((0.200, KEPT_Y1), (0.340, KEPT_Y1))  # strap clamps on the kept part's far-left edge
TRAY = dict(x=(0.140, 0.400), y=(-0.170, -0.050), base=0.002, rim=0.012, wall=0.002)  # steel dross tray under the line and offcut

# -- torch (torch frame: origin at the grip block's centre; +x toward the head) ---------------------------------------
SHIELD_R = 0.010  # drag shield, 20 mm across
SHIELD_H = 0.014
RECESS = 0.002  # the nozzle orifice sits this far above the shield's rim
HEAD = dict(r=0.013, a0=SHIELD_H, a1=0.062)  # above the rim
HANDLE = dict(r=0.013, len=0.190, a=0.045)  # the handle's axis is this far above the rim
GRIP = dict(x=0.030, y=0.036, z=0.026)  # glass-filled nylon grip block on the handle; the fingers close along x
DG = 0.090  # the grip block's centre is this far behind the head's axis, so the two wrists pass each other clear
LEAD = dict(r=0.014, len=0.060)
HANDLE_Z = -(HANDLE["r"] + GRIP["z"] / 2)
RIM = np.array((DG, 0.0, HANDLE_Z - HANDLE["a"]))  # centre of the shield's rim (its bottom face), torch frame
_hx0 = DG - HANDLE["len"]
_lead_x = _hx0 - LEAD["len"] / 2
TORCH_MASS = {"head": 0.140, "handle": 0.070, "grip": 0.030}
TORCH_MASS["lead"] = -(TORCH_MASS["head"] * DG + TORCH_MASS["handle"] * (DG - HANDLE["len"] / 2)) / _lead_x  # the lead stub balances the head: 0.094 kg
TORCH_TOTAL = sum(TORCH_MASS.values())
TORCH_COM = (TORCH_MASS["head"] * (RIM + (0.0, 0.0, 0.036)) + TORCH_MASS["handle"] * np.array((DG - HANDLE["len"] / 2, 0.0, HANDLE_Z))
             + TORCH_MASS["lead"] * np.array((_lead_x, 0.0, HANDLE_Z))) / TORCH_TOTAL
TORCH_INERTIA = (2.0e-4, 9.0e-4, 8.5e-4)
TORCH_BOXES = {
    "grip": ((-GRIP["x"] / 2, -GRIP["y"] / 2, -GRIP["z"] / 2), (GRIP["x"] / 2, GRIP["y"] / 2, GRIP["z"] / 2)),
    "handle": ((_hx0, -HANDLE["r"], HANDLE_Z - HANDLE["r"]), (DG, HANDLE["r"], HANDLE_Z + HANDLE["r"])),
    "lead": ((_hx0 - LEAD["len"], -LEAD["r"], HANDLE_Z - LEAD["r"]), (_hx0, LEAD["r"], HANDLE_Z + LEAD["r"])),
    "head": ((DG - 0.011, -0.011, RIM[2] + HEAD["a0"]), (DG + 0.011, 0.011, RIM[2] + HEAD["a1"])),
}
TORCH_HULL = (np.array((_hx0 - LEAD["len"], -GRIP["y"] / 2, RIM[2])), np.array((DG + HEAD["r"], GRIP["y"] / 2, GRIP["z"] / 2)))

# -- straightedge (guide frame: origin at the handle block's centre; +x along the bar; working edge at -y) ------------
BAR = dict(len=0.260, w=0.025, t=0.005)
HBLOCK = 0.030  # nylon handle block, a 30 mm cube; the fingers close along y
EDGE_Y = -0.095  # the working edge, guide frame: 95 mm from the handle's centre, so the two wrists pass each other clear
TAB = dict(x=0.030, y=(EDGE_Y + BAR["w"], 0.018))
GUIDE_Z0 = -(HBLOCK / 2 + BAR["t"])  # the underside
GUIDE_MASS = {"bar": 7850.0 * BAR["len"] * BAR["w"] * BAR["t"], "tab": 7850.0 * TAB["x"] * (TAB["y"][1] - TAB["y"][0]) * BAR["t"], "handle": 0.030}
GUIDE_TOTAL = sum(GUIDE_MASS.values())
GUIDE_COM = (GUIDE_MASS["bar"] * np.array((0.0, EDGE_Y + BAR["w"] / 2, GUIDE_Z0 + BAR["t"] / 2))
             + GUIDE_MASS["tab"] * np.array((0.0, (TAB["y"][0] + TAB["y"][1]) / 2, GUIDE_Z0 + BAR["t"] / 2))) / GUIDE_TOTAL
GUIDE_BOXES = {
    "bar": ((-BAR["len"] / 2, EDGE_Y, GUIDE_Z0), (BAR["len"] / 2, EDGE_Y + BAR["w"], GUIDE_Z0 + BAR["t"])),
    "tab": ((-TAB["x"] / 2, TAB["y"][0], GUIDE_Z0), (TAB["x"] / 2, TAB["y"][1], GUIDE_Z0 + BAR["t"])),
    "handle": ((-HBLOCK / 2,) * 3, (HBLOCK / 2,) * 3),
}
GUIDE_HULL = (np.array((-BAR["len"] / 2, EDGE_Y, GUIDE_Z0)), np.array((BAR["len"] / 2, TAB["y"][1], HBLOCK / 2)))
GUIDE_USE = dict(x=(X0 + X1) / 2, y=LINE_Y + SHIELD_R - EDGE_Y, z=PT - GUIDE_Z0)  # handle centre with the edge one shield radius from the line

# -- rests (bench frame) ---------------------------------------------------------------------------------------------
TORCH_REST = dict(x=0.220, y=-0.300, heading=math.pi / 2)  # the grip's centre; the torch's +x points along +y
TORCH_POST_H = 0.047
TORCH_REST_Z = TORCH_POST_H - (HANDLE_Z - HANDLE["r"])  # the grip's centre above the table, handle on the posts
TORCH_POSTS = ((0.030, 0.050), (-0.090, -0.070))  # along the torch's x from the grip's centre
GUIDE_PAD_T = 0.003
GUIDE_REST = dict(x=0.270, y=0.260, heading=0.0)  # the handle block's centre
GUIDE_REST_Z = GUIDE_PAD_T - GUIDE_Z0

COLORS = {  # sRGB base colour, metallic, roughness, emissive
    "stand": ((0.20, 0.21, 0.22), 0.8, 0.55, None), "plate": ((0.36, 0.37, 0.39), 0.7, 0.55, None),
    "line": ((0.95, 0.95, 0.90), 0.0, 0.7, None), "clamp": ((0.12, 0.30, 0.62), 0.4, 0.45, None),
    "bolt": ((0.55, 0.56, 0.58), 1.0, 0.35, None), "post": ((0.42, 0.43, 0.45), 0.9, 0.4, None),
    "tray": ((0.30, 0.30, 0.31), 0.8, 0.6, None), "pad": ((0.10, 0.10, 0.10), 0.0, 0.9, None),
    "head": ((0.07, 0.07, 0.08), 0.0, 0.5, None), "shield": ((0.72, 0.45, 0.20), 1.0, 0.35, None),
    "grip": ((0.85, 0.65, 0.05), 0.0, 0.55, None), "lead": ((0.04, 0.04, 0.04), 0.0, 0.7, None),
    "bar": ((0.70, 0.71, 0.73), 1.0, 0.3, None), "hblock": ((0.80, 0.15, 0.10), 0.0, 0.55, None),
    "arc": ((0.75, 0.85, 1.0), 0.0, 0.3, (4.0, 5.0, 8.0)), "kerf": ((0.02, 0.02, 0.02), 0.2, 0.9, None),
    "dross": ((0.55, 0.30, 0.10), 0.5, 0.8, None), "gouge": ((0.80, 0.80, 0.82), 1.0, 0.3, None),
}
MU = {"steel": 0.5, "grip": 1.0, "rubber": 0.9, "shield": 0.35}  # shield: copper on steel


def seg_x(i):
    """Centre of segment i along the line (0 at the near edge)."""
    return X0 + (i + 0.5) * SEG_L


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

    def _finish(self, g, look, collide, offset=0.001, show=True):
        from pxr import UsdGeom, UsdPhysics, UsdShade

        prim = g.GetPrim()
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.look(look))
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            _physx(prim, "PhysxCollisionAPI", **{"physxCollision:contactOffset": float(offset), "physxCollision:restOffset": 0.0})
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.surface(collide), UsdShade.Tokens.weakerThanDescendants, "physics")
        if not show:
            UsdGeom.Imageable(prim).MakeInvisible()
        return g

    def box(self, path, lo, hi, look, collide=None, offset=0.001, show=True):
        from pxr import Gf, UsdGeom

        lo, hi = np.array(lo, float), np.array(hi, float)
        c = UsdGeom.Cube.Define(self.stage, path)
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, (lo + hi) / 2)))
        c.AddScaleOp().Set(Gf.Vec3f(*map(float, hi - lo)))
        return self._finish(c, look, collide, offset, show)

    def cyl(self, path, center, r, h, look, axis="Z", collide=None, show=True):
        from pxr import Gf, UsdGeom

        c = UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateAxisAttr(axis)
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(h))
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        return self._finish(c, look, collide, 0.001, show)

    def ellipsoid(self, path, center, radii, look, show=True):
        from pxr import Gf, UsdGeom

        s = UsdGeom.Sphere.Define(self.stage, path)
        s.CreateRadiusAttr(1.0)
        s.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        s.AddScaleOp().Set(Gf.Vec3f(*map(float, radii)))
        return self._finish(s, look, None, show=show)

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


def build_torch(stage, path):
    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    b.rigid(prim, mass=TORCH_TOTAL, com=TORCH_COM, inertia=TORCH_INERTIA)
    for name, (lo, hi) in TORCH_BOXES.items():
        b.box(f"{path}/c_{name}", lo, hi, "grip" if name == "grip" else "lead", "grip" if name == "grip" else "rubber", show=name == "grip")
    # the drag shield: a real cylinder collider, so it rolls/slides along the straightedge's edge and rides on the plate
    b.cyl(f"{path}/c_shield", RIM + (0.0, 0.0, SHIELD_H / 2), SHIELD_R, SHIELD_H, "shield", collide="shield")
    hx0 = DG - HANDLE["len"]
    b.cyl(f"{path}/v_handle", ((hx0 + DG) / 2, 0.0, HANDLE_Z), HANDLE["r"], HANDLE["len"], "head", axis="X")
    b.cyl(f"{path}/v_lead", (hx0 - LEAD["len"] / 2, 0.0, HANDLE_Z), LEAD["r"], LEAD["len"], "lead", axis="X")
    b.cyl(f"{path}/v_head", RIM + (0.0, 0.0, (HEAD["a0"] + HEAD["a1"]) / 2), HEAD["r"], HEAD["a1"] - HEAD["a0"], "head")
    b.ellipsoid(f"{path}/v_cap", RIM + (0.0, 0.0, HEAD["a1"]), (HEAD["r"], HEAD["r"], 0.005), "head")
    b.box(f"{path}/v_trigger", (0.045, -0.006, HANDLE_Z - HANDLE["r"] - 0.006), (0.065, 0.006, HANDLE_Z - HANDLE["r"]), "hblock")
    b.ellipsoid(f"{path}/fx_arc", RIM + (0.0, 0.0, 0.001), (0.006, 0.006, 0.004), "arc", show=False)
    return prim


def build_guide(stage, path):
    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    b.rigid(prim, mass=GUIDE_TOTAL, com=GUIDE_COM)
    for name, (lo, hi) in GUIDE_BOXES.items():
        b.box(f"{path}/c_{name}", lo, hi, "hblock" if name == "handle" else "bar", "grip" if name == "handle" else "steel")
    return prim


def build_offcut(stage, path, bench_path):
    """The offcut and its N_SEG fixed joints to the bench (joint_i at segment i's centre on the line)."""
    from pxr import Gf, Sdf, UsdPhysics

    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    b.rigid(prim, mass=OFFCUT_MASS)
    hx, hy = (X1 - X0) / 2, (OFFCUT["y"][1] - OFFCUT["y"][0]) / 2
    b.box(f"{path}/c_plate", (-hx, -hy, -PLATE_T / 2), (hx, hy, PLATE_T / 2), "plate", "steel")
    for i in range(N_SEG):
        j = UsdPhysics.FixedJoint.Define(stage, f"{path}/joint_{i:02d}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(bench_path)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        p0 = np.array((seg_x(i), LINE_Y, PT - PLATE_T / 2))
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, p0)))
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, p0 - OFFCUT_C)))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
        j.CreateJointEnabledAttr(True)
        j.CreateBreakForceAttr(float(BREAK_FORCE))
        j.CreateBreakTorqueAttr(float(BREAK_TORQUE))
    return prim


def build_bench(stage, path):
    b = _Builder(stage, path)
    prim = stage.GetPrimAtPath(path)
    b.rigid(prim, kinematic=True)
    (sx0, sx1), (sy0, sy1), sh = STAND["x"], STAND["y"], STAND["h"]
    b.box(f"{path}/stand", (sx0, sy0, 0.0), (sx1, sy1, sh), "stand", "steel")
    b.box(f"{path}/kept", (X0, LINE_Y, sh), (X1, KEPT_Y1, PT), "plate", "steel")
    b.box(f"{path}/line", (X0, LINE_Y, PT), (X1, LINE_Y + 0.0012, PT + 0.0002), "line")
    for k, (cx, cy) in enumerate(CLAMPS):  # strap clamps: a bar over the plate's edge, a heel and a stud (bar and heel collide)
        b.box(f"{path}/clamp{k}_bar", (cx - 0.010, cy - 0.030, PT), (cx + 0.010, cy + 0.030, PT + 0.008), "clamp", "steel")
        b.box(f"{path}/clamp{k}_heel", (cx - 0.010, cy + 0.012, 0.0), (cx + 0.010, cy + 0.030, PT), "clamp", "steel")
        b.cyl(f"{path}/clamp{k}_nut", (cx, cy + 0.004, PT + 0.011), 0.0075, 0.006, "bolt")
    t = TRAY
    (tx0, tx1), (ty0, ty1) = t["x"], t["y"]
    b.box(f"{path}/tray_base", (tx0, ty0, 0.0), (tx1, ty1, t["base"]), "tray", "steel")
    for n, (lo, hi) in {"a": ((tx0, ty0), (tx1, ty0 + t["wall"])), "b": ((tx0, ty1 - t["wall"]), (tx1, ty1)),
                        "c": ((tx0, ty0), (tx0 + t["wall"], ty1)), "d": ((tx1 - t["wall"], ty0), (tx1, ty1))}.items():
        b.box(f"{path}/tray_{n}", (*lo, 0.0), (*hi, t["rim"]), "tray", "steel")
    # the torch's rest: two steel posts under the handle, on a base
    c, s = math.cos(TORCH_REST["heading"]), math.sin(TORCH_REST["heading"])
    for k, (a0, a1) in enumerate(TORCH_POSTS):
        am = (a0 + a1) / 2
        px, py = TORCH_REST["x"] + am * c, TORCH_REST["y"] + am * s
        hx, hy = abs(c) * (a1 - a0) / 2 + abs(s) * 0.020, abs(s) * (a1 - a0) / 2 + abs(c) * 0.020
        b.box(f"{path}/torch_post{k}", (px - hx, py - hy, 0.0), (px + hx, py + hy, TORCH_POST_H), "post", "steel")
    b.box(f"{path}/torch_base", (TORCH_REST["x"] - 0.028, TORCH_REST["y"] - 0.100, 0.0), (TORCH_REST["x"] + 0.028, TORCH_REST["y"] + 0.060, 0.004), "post", "steel")
    # the straightedge's rest: a rubber pad
    gx, gy = GUIDE_REST["x"], GUIDE_REST["y"]
    b.box(f"{path}/guide_pad", (gx - BAR["len"] / 2 - 0.01, gy + EDGE_Y - 0.01, 0.0), (gx + BAR["len"] / 2 + 0.01, gy + TAB["y"][1] + 0.01, GUIDE_PAD_T), "pad", "rubber")
    # kerf pieces on the kept part's edge, hidden until note_step shows them
    for i in range(N_SEG):
        x = seg_x(i)
        b.box(f"{path}/kerf_{i}", (x - SEG_L / 2, LINE_Y - 0.0002, PT - 0.0005), (x + SEG_L / 2, LINE_Y + 0.0016, PT + 0.0004), "kerf", show=False)
        b.ellipsoid(f"{path}/dross_{i}", (x, LINE_Y + 0.0008, PT + 0.0004), (0.0026, 0.0022, 0.0012), "dross", show=False)
        b.box(f"{path}/gouge_{i}", (x - SEG_L / 2, LINE_Y, PT), (x + SEG_L / 2, LINE_Y + 0.0014, PT + 0.0004), "gouge", show=False)
    return prim
