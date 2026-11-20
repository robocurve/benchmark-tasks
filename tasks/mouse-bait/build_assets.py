"""Write the scene's USD assets and textures from geometry.py.

    python build_assets.py          # needs pxr, PIL and the DejaVu fonts (Isaac Sim's Python), not the simulator

- assets/station.usda: the bait station, one floating articulation:
  - base (root): the floor, four walls 100 tall with a mouse entrance in each end, two inner baffles and four upright
    bait-securing rods (capsules, 4 across, 60 tall);
  - lid (revolute about the left end's top edge): a shallow cap (top plate and 14 mm skirt; one solid collider), the
    two key locks' keyholes at its free end (visual).
  Origin: the middle of the floor's underside. The lid is authored open at its 100 degree rest.
- assets/block.usda: a bait block, one rigid body: four boxes round a square hole through its broad face.
- assets/wall.usda: the pegboard panel (kinematic) and, at its foot on the bench, the printed droppings strip (visual).
- assets/pail.usda: the bait pail with its label (kinematic).
- assets/truck.usda, assets/cup.usda: the toddler's dump truck and sippy cup (kinematic).
- assets/textures/mb_atlas_1.png: drawn here, one atlas (the pail's label and lid, the station's lid, the pegboard).
- assets/print/*.pdf: the pail's label, its lid print and the droppings strip at true size, for the real build
  (`python build_assets.py --printables` writes only these).
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

import geometry as g

OUT = Path(__file__).resolve().parent / "assets"
TEX = OUT / "textures"
FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_C = "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf"
BLACK = (0.07, 0.07, 0.075)
FLOOR_GREY = (0.16, 0.16, 0.17)
STEEL = (0.72, 0.72, 0.74)
WAX = (0.80, 0.68, 0.46)
WHITE = (0.92, 0.92, 0.90)
OFF = (0.0005, 0.0)                 # contact and rest offsets of the small parts' colliders
OFF_BIG = (0.001, 0.0)


# -- textures ------------------------------------------------------------------------------------------------------------
def _text(d, xy, s, size, fill, font=FONT_B, anchor="mm"):
    d.text(xy, s, font=ImageFont.truetype(font, size), fill=fill, anchor=anchor)


def pail_label(path: Path) -> None:
    """The pail's wrap-round label (image left to right = round the pail, the front panel in the middle): the Tomcat
    Bait Chunx front panel, typeset here from the label's words."""
    w, h = 2400, 520
    im = Image.new("RGB", (w, h), (246, 244, 236))
    d = ImageDraw.Draw(im)
    c = w // 2
    d.rectangle([c - 560, 0, c + 560, h], fill=(18, 92, 46))                 # the green front panel
    d.rectangle([c - 560, 0, c + 560, 92], fill=(230, 196, 36))
    _text(d, (c, 48), "KEEP OUT OF REACH OF CHILDREN", 62, (10, 10, 10))
    _text(d, (c, 165), "TOMCAT", 120, (250, 250, 250))
    _text(d, (c, 250), "with Bromethalin", 46, (250, 250, 250), FONT)
    _text(d, (c, 325), "BAIT CHUNX", 84, (250, 220, 60))
    _text(d, (c, 400), "KILLS RATS, MICE AND MEADOW VOLES", 40, (250, 250, 250))
    d.rectangle([c - 560, 440, c + 560, h], fill=(230, 196, 36))
    _text(d, (c, 480), "CAUTION   Net wt 4 lb", 50, (10, 10, 10))
    for x0 in (24, c + 584):                                                   # back panels: small print
        for k, s in enumerate(("PRECAUTIONARY STATEMENTS", "Tamper-resistant bait stations must be used",
                               "wherever children, pets, non-target mammals", "or birds may have access to the bait",
                               "placement location.", "Bromethalin 0.01%   EPA Reg. No. 12455-132-3240")):
            _text(d, (x0, 80 + k * 62), s, 23, (40, 40, 40), FONT, anchor="lm")
    im.save(path)


def pail_lid(path: Path) -> None:
    """The pail's lid, from above (image up = +x, away from the robot): the label's front panel repeated."""
    n = 1024
    im = Image.new("RGB", (n, n), (240, 238, 230))
    d = ImageDraw.Draw(im)
    d.ellipse([40, 40, n - 40, n - 40], fill=(18, 92, 46))
    _text(d, (n / 2, 250), "TOMCAT", 170, (250, 250, 250))
    _text(d, (n / 2, 400), "BAIT CHUNX", 110, (250, 220, 60))
    d.rectangle([110, 480, n - 110, 830], fill=(230, 196, 36))
    _text(d, (n / 2, 545), "KEEP OUT", 120, (10, 10, 10))
    _text(d, (n / 2, 655), "OF REACH OF", 92, (10, 10, 10))
    _text(d, (n / 2, 765), "CHILDREN", 120, (10, 10, 10))
    _text(d, (n / 2, 900), "CAUTION", 70, (250, 250, 250))
    im.save(path)


def station_lid(path: Path) -> None:
    """The station's lid top (image left = the hinge end, image up = toward the wall): the moulded name."""
    w, h = 1205, 1110
    im = Image.new("RGB", (w, h), (20, 20, 22))
    d = ImageDraw.Draw(im)
    d.rectangle([30, 30, w - 30, h - 30], outline=(44, 44, 48), width=10)
    _text(d, (w / 2, h / 2 - 90), "TOMCAT", 210, (226, 226, 226))
    _text(d, (w / 2, h / 2 + 90), "RODENT STATION", 92, (190, 190, 190))
    _text(d, (w / 2, h / 2 + 220), "TAMPER-RESISTANT", 60, (150, 150, 150))
    im.save(path)


def pegboard(path: Path) -> None:
    """The pegboard's front face (image left = the robot's left, +y): hardboard with 1 inch hole pitch."""
    ppm = 2000                      # pixels per metre
    w, h = int(g.WALL_W * ppm), int(g.WALL_H * ppm)
    im = Image.new("RGB", (w, h), (150, 108, 66))
    d = ImageDraw.Draw(im)
    rng = np.random.default_rng(7)
    for _ in range(2500):                                                      # grain
        x, y = rng.integers(0, w), rng.integers(0, h)
        d.line([(x, y), (x + rng.integers(20, 120), y)], fill=(140, 99, 60), width=1)
    pitch = 0.0254 * ppm
    r = 0.003 * ppm
    for i in range(int(w / pitch)):
        for j in range(int(h / pitch)):
            cx, cy = (i + 0.5) * pitch + (w % pitch) / 2, (j + 0.5) * pitch + (h % pitch) / 2
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(38, 28, 18))
    im.save(path)


# One texture file for the whole scene (several textured materials have shown each other's images in this renderer):
# the artwork above packed into an atlas, each mesh's uv mapped into its region. The file name carries a version.
ATLAS = "mb_atlas_1.png"
ATLAS_PX = 2048
REGIONS = {"pegboard": (0, 0, 2048, 1024), "pail_label": (0, 1024, 2048, 444), "pail_lid": (0, 1500, 512, 512),
           "station_lid": (600, 1500, 556, 512)}             # x, y (from the image's top), width, height in pixels


def build_atlas(path: Path) -> None:
    tmp = TEX / "_parts"
    tmp.mkdir(exist_ok=True)
    pail_label(tmp / "pail_label.png")
    pail_lid(tmp / "pail_lid.png")
    station_lid(tmp / "station_lid.png")
    pegboard(tmp / "pegboard.png")
    im = Image.new("RGB", (ATLAS_PX, ATLAS_PX), (128, 128, 128))
    for name, (x, y, w, h) in REGIONS.items():
        im.paste(Image.open(tmp / f"{name}.png").convert("RGB").resize((w, h), Image.LANCZOS), (x, y))
    for f in tmp.iterdir():
        f.unlink()
    tmp.rmdir()
    im.save(path)


def auv(region: str, uv):
    """uv in [0, 1]^2 of one artwork (v up) mapped into its region of the atlas."""
    x, y, w, h = REGIONS[region]
    u, v = uv
    return ((x + u * w) / ATLAS_PX, 1.0 - (y + (1.0 - v) * h) / ATLAS_PX)


# -- USD helpers -------------------------------------------------------------------------------------------------------
def stage_for(name: str):
    path = OUT / f"{name}.usda"
    path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{name}")
    stage.SetDefaultPrim(root.GetPrim())
    return stage, root


def material(stage, path: str, color, roughness=0.5, texture: str | None = None, metallic=0.0):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    if texture:
        st = UsdShade.Shader.Define(stage, f"{path}/st")
        st.CreateIdAttr("UsdPrimvarReader_float2")
        st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tx = UsdShade.Shader.Define(stage, f"{path}/texture")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture)
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
        tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
    else:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
    UsdShade.MaterialBindingAPI(prim.GetPrim()).Bind(mat)


def offsets(prim, contact=0.0005, rest=0.0):
    prim = prim.GetPrim()
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(contact)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(rest)


def box(stage, path, lo, hi, mat=None, collide=True, off=OFF, visible=True):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, (lo + hi) / 2)))
    c.AddScaleOp().Set(Gf.Vec3f(*map(float, hi - lo)))
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        offsets(c, *off)
    if not visible:
        c.CreatePurposeAttr(UsdGeom.Tokens.guide)
    return c


def cylinder(stage, path, radius, center, length, axis, mat=None, collide=False, off=OFF):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(float(radius))
    c.CreateHeightAttr(float(length))
    c.CreateAxisAttr(axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        offsets(c, *off)
    return c


def capsule(stage, path, radius, center, height, axis, mat=None, collide=True, off=OFF):
    c = UsdGeom.Capsule.Define(stage, path)
    c.CreateRadiusAttr(float(radius))
    c.CreateHeightAttr(float(height))
    c.CreateAxisAttr(axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    if mat is not None:
        bind(c, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        offsets(c, *off)
    return c


def mesh(stage, path, V, F, mat=None, collide=False, off=OFF, uv=None, smooth=False):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in V])
    m.CreateFaceVertexCountsAttr([len(f) for f in F])
    m.CreateFaceVertexIndicesAttr([int(i) for f in F for i in f])
    m.CreateSubdivisionSchemeAttr("catmullClark" if smooth else "none")
    if uv is not None:
        UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                             UsdGeom.Tokens.vertex).Set([Gf.Vec2f(*map(float, t)) for t in uv])
    m.CreateDoubleSidedAttr(True)
    if mat is not None:
        bind(m, mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
        offsets(m, *off)
    return m


def quad(stage, path, corners, uv, mat):
    """A textured quad from four corners (counterclockwise seen from the side it faces) and their uv."""
    return mesh(stage, path, corners, [[0, 1, 2, 3]], mat, uv=uv)


def body(stage, path, mass, pos=(0.0, 0.0, 0.0), com=None, inertia=None, orient=None):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    if orient is not None:
        xf.AddOrientOp().Set(Gf.Quatf(*map(float, orient)))
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    m = UsdPhysics.MassAPI.Apply(xf.GetPrim())
    m.CreateMassAttr(float(mass))
    if com is not None:
        m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
    if inertia is not None:
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
    return xf


def revolute(stage, path, body0, body1, pos0, rot, limits):
    """A revolute joint about the joint frame's x; both bodies' joint frames turned by ``rot`` (w, x, y, z)."""
    j = UsdPhysics.RevoluteJoint.Define(stage, path)
    j.CreateAxisAttr("X")
    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
    j.CreateLocalRot0Attr(Gf.Quatf(*map(float, rot)))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(*map(float, rot)))
    j.CreateLowerLimitAttr(float(limits[0]))
    j.CreateUpperLimitAttr(float(limits[1]))
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0)
    drive.CreateTargetPositionAttr(0.0)
    return j


def box_inertia(m, sx, sy, sz):
    return (m * (sy * sy + sz * sz) / 12, m * (sx * sx + sz * sz) / 12, m * (sx * sx + sy * sy) / 12)


def kinematic_root(stage, root):
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.RigidBodyAPI(root.GetPrim()).CreateKinematicEnabledAttr(True)


# -- the station -------------------------------------------------------------------------------------------------------
def build_station():
    stage, root = stage_for("station")
    r = "/station"
    black = material(stage, f"{r}/Looks/black", BLACK, roughness=0.55)
    inner = material(stage, f"{r}/Looks/inner", FLOOR_GREY, roughness=0.7)
    steel = material(stage, f"{r}/Looks/steel", STEEL, roughness=0.3, metallic=0.8)
    lid_top = material(stage, f"{r}/Looks/lid_top", (1, 1, 1), roughness=0.5, texture=f"./textures/{ATLAS}")
    hole = material(stage, f"{r}/Looks/keyhole", (0.55, 0.55, 0.58), roughness=0.3, metallic=0.6)

    b = f"{r}/base"
    D, L, H, t, f = g.HALF_D, g.HALF_L, g.BASE_H, g.SIDE_T, g.FLOOR_T
    prim = body(stage, b, g.BASE_MASS, com=(0.0, 0.0, 0.035), inertia=box_inertia(g.BASE_MASS, g.ST_D, g.ST_L, 0.10)).GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxArticulationAPI")
    prim.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    prim.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    prim.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    box(stage, f"{b}/floor", (-D, -L, 0.0), (D, L, f), black, off=OFF_BIG)
    box(stage, f"{b}/floor_in", (-D + t, -L + t, f), (D - t, L - t, f + 0.0002), inner, collide=False)
    box(stage, f"{b}/wall_front", (-D, -L, f), (-D + t, L, H), black, off=OFF_BIG)
    box(stage, f"{b}/wall_back", (D - t, -L, f), (D, L, H), black, off=OFF_BIG)
    ew, eh = g.ENTRANCE_W / 2, f + g.ENTRANCE_H
    for s, name in ((1, "left"), (-1, "right")):
        y0, y1 = sorted((s * (L - t), s * L))
        box(stage, f"{b}/end_{name}_a", (-D + t, y0, f), (-ew, y1, H), black, off=OFF_BIG)
        box(stage, f"{b}/end_{name}_b", (ew, y0, f), (D - t, y1, H), black, off=OFF_BIG)
        box(stage, f"{b}/end_{name}_top", (-ew, y0, eh), (ew, y1, H), black, off=OFF_BIG)
        yb = s * g.BAFFLE_Y
        box(stage, f"{b}/baffle_{name}", (g.BAFFLE_X[0], yb - 0.001, f), (g.BAFFLE_X[1], yb + 0.001, f + g.BAFFLE_H),
            black, off=OFF)
    for k, (x, y) in enumerate(g.RODS):
        capsule(stage, f"{b}/rod_{k}", g.ROD_R, (x, y, f + g.ROD_H / 2), g.ROD_H - 2 * g.ROD_R, "Z", steel, off=OFF)

    # the lid, authored open at its rest (rotated about -x by LID_OPEN about the hinge)
    lid = f"{r}/lid"
    a = g.LID_OPEN
    q = (math.cos(-a / 2), math.sin(-a / 2), 0.0, 0.0)
    Ls, sk, lt = g.ST_L, g.LID_SKIRT, g.LID_T
    body(stage, lid, g.LID_MASS, pos=(0.0, g.HINGE_Y, g.HINGE_Z), orient=q, com=(0.0, -Ls / 2, sk * 0.6),
         inertia=box_inertia(g.LID_MASS, g.ST_D, Ls, sk))
    box(stage, f"{lid}/solid", (-D, -Ls, 0.0), (D, 0.0, sk), None, off=OFF_BIG, visible=False)
    box(stage, f"{lid}/plate", (-D, -Ls, sk - lt), (D, 0.0, sk), black, collide=False)
    box(stage, f"{lid}/skirt_front", (-D, -Ls, 0.0), (-D + t, 0.0, sk - lt), black, collide=False)
    box(stage, f"{lid}/skirt_back", (D - t, -Ls, 0.0), (D, 0.0, sk - lt), black, collide=False)
    box(stage, f"{lid}/skirt_free", (-D + t, -Ls, 0.0), (D - t, -Ls + t, sk - lt), black, collide=False)
    box(stage, f"{lid}/skirt_hinge", (-D + t, -t, 0.0), (D - t, 0.0, sk - lt), black, collide=False)
    z = sk + 0.0003
    quad(stage, f"{lid}/top_print", [(-D, 0.0, z), (-D, -Ls, z), (D, -Ls, z), (D, 0.0, z)],
         [auv("station_lid", t) for t in [(0, 0), (1, 0), (1, 1), (0, 1)]], lid_top)
    for k, x in enumerate(g.LOCK_XS):                     # the two key locks' slots at the free end (visual)
        box(stage, f"{lid}/keyhole_{k}", (x - 0.006, -Ls + 0.008, sk), (x + 0.006, -Ls + 0.014, sk + 0.0006), hole,
            collide=False)
    # the hinge knuckles along the left end (visual)
    for k in range(5):
        x = -D + (k + 0.5) * g.ST_D / 5
        cylinder(stage, f"{lid}/knuckle_{k}", 0.0035, (x, 0.0, 0.0), 0.02, "X", black)

    rot = (0.0, 0.0, 0.0, 1.0)            # the joint frames turned 180 degrees about z: the joint's x is -x
    revolute(stage, f"{r}/joints/lid_hinge", b, lid, (0.0, g.HINGE_Y, g.HINGE_Z), rot, (0.0, math.degrees(g.LID_OPEN)))
    stage.Save()


# -- the bait block ----------------------------------------------------------------------------------------------------
def build_block():
    stage, root = stage_for("block")
    p = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    m = UsdPhysics.MassAPI.Apply(p)
    m.CreateMassAttr(g.BLK_MASS)
    m.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(*box_inertia(g.BLK_MASS, g.BLK_L, g.BLK_W, g.BLK_T)))
    r = "/block"
    wax = material(stage, f"{r}/Looks/wax", WAX, roughness=0.75)
    edge = material(stage, f"{r}/Looks/edge", (0.66, 0.54, 0.34), roughness=0.8)
    hl, hw, ht, h = g.BLK_L / 2, g.BLK_W / 2, g.BLK_T / 2, g.HOLE / 2
    box(stage, f"{r}/end_a", (-hl, -hw, -ht), (-h, hw, ht), wax)
    box(stage, f"{r}/end_b", (h, -hw, -ht), (hl, hw, ht), wax)
    box(stage, f"{r}/side_a", (-h, -hw, -ht), (h, -h, ht), wax)
    box(stage, f"{r}/side_b", (-h, h, -ht), (h, hw, ht), wax)
    # gnawing grooves across the broad faces (visual)
    for s in (-1, 1):
        for x in (-0.017, -0.009, 0.009, 0.017):
            box(stage, f"{r}/groove_{'t' if s > 0 else 'b'}_{int(x * 1000) + 20}", (x - 0.0008, -hw, s * ht - 0.0001),
                (x + 0.0008, hw, s * ht + 0.0002 * s), edge, collide=False)
    stage.Save()


# -- the wall and the droppings ----------------------------------------------------------------------------------------
def build_wall():
    """Origin on the bench top at the wall's front face, y 0."""
    stage, root = stage_for("wall")
    kinematic_root(stage, root)
    r = "/wall"
    board = material(stage, f"{r}/Looks/board", (0.59, 0.42, 0.26), roughness=0.85)
    face = material(stage, f"{r}/Looks/face", (1, 1, 1), roughness=0.85, texture=f"./textures/{ATLAS}")
    paper = material(stage, f"{r}/Looks/paper", (0.86, 0.84, 0.79), roughness=0.9)
    dung = material(stage, f"{r}/Looks/dung", (0.05, 0.04, 0.035), roughness=0.9)
    W, H, T = g.WALL_W / 2, g.WALL_H, g.WALL_T
    box(stage, f"{r}/board", (0.0, -W, 0.0), (T, W, H), board, off=OFF_BIG)
    x = -0.0003
    quad(stage, f"{r}/face", [(x, W, 0.0), (x, -W, 0.0), (x, -W, H), (x, W, H)],
         [auv("pegboard", t) for t in [(0, 0), (1, 0), (1, 1), (0, 1)]], face)
    # the printed droppings strip, flat on the bench at the wall's foot (visual)
    x0, x1 = g.DROP_X[0] - g.WALL_X, g.DROP_X[1] - g.WALL_X
    box(stage, f"{r}/strip", (x0, g.DROP_Y[0], 0.0), (x1, g.DROP_Y[1], 0.0010), paper, collide=False)
    for k, (xc, y, a, ln, wd) in enumerate(droppings()):
        c, s = math.cos(a), math.sin(a)
        V, F = [], []
        for dz in (0.0010, 0.0032):
            for u, v in ((-ln / 2, -wd / 2), (ln / 2, -wd / 2), (ln / 2, wd / 2), (-ln / 2, wd / 2)):
                V.append((xc + c * u - s * v, y + s * u + c * v, dz))
        F = [[3, 2, 1, 0], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
        mesh(stage, f"{r}/dropping_{k:02d}", V, F, dung)
    stage.Save()


def droppings():
    """The twenty printed droppings: (x, y, angle, length, width), x and y in the wall's frame (x 0 at its face)."""
    x0, x1 = g.DROP_X[0] - g.WALL_X, g.DROP_X[1] - g.WALL_X
    rng = np.random.default_rng(11)
    ys = np.sort(rng.uniform(g.DROP_Y[0] + 0.01, g.DROP_Y[1] - 0.01, g.N_DROPPINGS))
    out = []
    for y in ys:
        xc = rng.uniform(x0 + 0.005, x1 - 0.005)
        a = rng.uniform(0, math.pi)
        ln, wd = rng.uniform(0.006, 0.008), 0.0026
        out.append((float(xc), float(y), float(a), float(ln), float(wd)))
    return out


# -- the pail -------------------------------------------------------------------------------------------------------------
def build_pail():
    """Origin at the bottom centre."""
    stage, root = stage_for("pail")
    kinematic_root(stage, root)
    r = "/pail"
    white = material(stage, f"{r}/Looks/white", WHITE, roughness=0.5)
    label = material(stage, f"{r}/Looks/label", (1, 1, 1), roughness=0.55, texture=f"./textures/{ATLAS}")
    lidm = material(stage, f"{r}/Looks/lid", (1, 1, 1), roughness=0.5, texture=f"./textures/{ATLAS}")
    R, H = g.PAIL_R, g.PAIL_H
    cylinder(stage, f"{r}/body", R, (0, 0, (H - 0.012) / 2), H - 0.012, "Z", white, collide=True, off=OFF_BIG)
    cylinder(stage, f"{r}/lid", R + 0.004, (0, 0, H - 0.006), 0.012, "Z", white, collide=True, off=OFF_BIG)
    # the label, wrapped round the body (front panel facing -x, the robot)
    n, z0, z1 = 72, 0.025, 0.155
    th0, th1 = math.pi - math.radians(160), math.pi + math.radians(160)
    V, uv, F = [], [], []
    rr = R + 0.0004
    for k in range(n + 1):
        th = th0 + (th1 - th0) * k / n
        for z, v in ((z0, 0.0), (z1, 1.0)):
            V.append((rr * math.cos(th), rr * math.sin(th), z))
            uv.append(auv("pail_label", (k / n, v)))
    for k in range(n):
        F.append([2 * k, 2 * k + 2, 2 * k + 3, 2 * k + 1])
    mesh(stage, f"{r}/label", V, F, label, uv=uv)
    # the lid's print, seen from above (image up = +x)
    m, zt, rl = 48, H + 0.0003, R - 0.002
    V = [(0.0, 0.0, zt)] + [(rl * math.cos(2 * math.pi * k / m), rl * math.sin(2 * math.pi * k / m), zt) for k in range(m)]
    uv = [auv("pail_lid", (0.5 - y / (2 * rl), 0.5 + x / (2 * rl))) for x, y, _ in V]
    F = [[0, 1 + k, 1 + (k + 1) % m] for k in range(m)]
    mesh(stage, f"{r}/lid_print", V, F, lidm, uv=uv)
    stage.Save()


# -- the toddler's things -----------------------------------------------------------------------------------------------
def build_truck():
    """A Green Toys Dump Truck, long side along y, cab toward -y (the droppings strip). Origin at its footprint's
    centre on the bench."""
    stage, root = stage_for("truck")
    kinematic_root(stage, root)
    r = "/truck"
    red = material(stage, f"{r}/Looks/red", (0.82, 0.12, 0.10), roughness=0.45)
    yellow = material(stage, f"{r}/Looks/yellow", (0.96, 0.76, 0.10), roughness=0.45)
    green = material(stage, f"{r}/Looks/green", (0.20, 0.62, 0.25), roughness=0.5)
    tyre = material(stage, f"{r}/Looks/tyre", (0.06, 0.06, 0.06), roughness=0.8)
    glass = material(stage, f"{r}/Looks/glass", (0.55, 0.75, 0.85), roughness=0.2)
    L, W, H = g.TRUCK_L / 2, g.TRUCK_W / 2, g.TRUCK_H
    wr, ww = 0.042, 0.030
    for sx in (-1, 1):
        for sy in (-1, 1):
            cylinder(stage, f"{r}/wheel_{'f' if sx > 0 else 'n'}{'l' if sy > 0 else 'r'}", wr,
                     (sx * (W - ww / 2), sy * (L - 0.055), wr), ww, "X", tyre, collide=True, off=OFF_BIG)
    box(stage, f"{r}/chassis", (-W + ww + 0.004, -L + 0.010, 0.030), (W - ww - 0.004, L - 0.010, 0.072), green, off=OFF_BIG)
    box(stage, f"{r}/cab", (-0.075, -L, 0.060), (0.075, -L + 0.085, 0.150), red, off=OFF_BIG)
    box(stage, f"{r}/roof", (-0.070, -L + 0.008, 0.150), (0.070, -L + 0.080, H), red, off=OFF_BIG)
    box(stage, f"{r}/windscreen", (-0.060, -L - 0.0005, 0.110), (0.060, -L + 0.002, 0.145), glass, collide=False)
    for s in (-1, 1):
        box(stage, f"{r}/side_window_{'a' if s > 0 else 'b'}", (s * 0.0755 - 0.001, -L + 0.020, 0.110),
            (s * 0.0755 + 0.001, -L + 0.070, 0.145), glass, collide=False)
    # the tipping bed: floor and walls
    y0, y1 = -L + 0.095, L
    box(stage, f"{r}/bed_floor", (-W, y0, 0.072), (W, y1, 0.082), yellow, off=OFF_BIG)
    box(stage, f"{r}/bed_front", (-W, y0, 0.082), (W, y0 + 0.010, 0.160), yellow, off=OFF_BIG)
    box(stage, f"{r}/bed_back", (-W, y1 - 0.008, 0.082), (W, y1, 0.140), yellow, off=OFF_BIG)
    box(stage, f"{r}/bed_left", (-W, y0, 0.082), (-W + 0.008, y1, 0.140), yellow, off=OFF_BIG)
    box(stage, f"{r}/bed_right", (W - 0.008, y0, 0.082), (W, y1, 0.140), yellow, off=OFF_BIG)
    stage.Save()


def build_cup():
    """A two-handled toddler's sippy cup. Origin at its bottom centre."""
    stage, root = stage_for("cup")
    kinematic_root(stage, root)
    r = "/cup"
    pink = material(stage, f"{r}/Looks/pink", (0.93, 0.45, 0.62), roughness=0.4)
    teal = material(stage, f"{r}/Looks/teal", (0.18, 0.70, 0.72), roughness=0.4)
    R, H = g.CUP_R, g.CUP_H
    cylinder(stage, f"{r}/body", R, (0, 0, 0.0575), 0.115, "Z", pink, collide=True, off=OFF_BIG)
    cylinder(stage, f"{r}/collar", R + 0.002, (0, 0, 0.127), 0.024, "Z", teal, collide=True, off=OFF_BIG)
    # the spout: a tapered block on the lid, toward +x
    box(stage, f"{r}/spout", (-0.010, -0.012, 0.139), (0.022, 0.012, H), teal, off=OFF_BIG)
    for s in (-1, 1):
        box(stage, f"{r}/handle_{'l' if s > 0 else 'r'}", (-0.008, s * R, 0.050), (0.008, s * (R + 0.030), 0.060),
            teal, off=OFF_BIG)
        box(stage, f"{r}/grip_{'l' if s > 0 else 'r'}", (-0.008, s * (R + 0.022) - 0.005, 0.050),
            (0.008, s * (R + 0.022) + 0.005, 0.105), teal, off=OFF_BIG)
    stage.Save()


# -- printables for the real build: true size (print at 100 %) ----------------------------------------------------------
PRINT = OUT / "print"
LABEL_MM = (2 * math.pi * (g.PAIL_R + 0.0004) * 320 / 360 * 1000, 130.0)   # the wrap: 320 degrees round, 130 tall
LID_MM = 2 * (g.PAIL_R - 0.002) * 1000                                     # the lid print's disc: 166 across
STRIP_MM = ((g.DROP_Y[1] - g.DROP_Y[0]) * 1000, (g.DROP_X[1] - g.DROP_X[0]) * 1000)


def save_pdf(im: Image.Image, path: Path, width_mm: float, height_mm: float, dpi: float) -> None:
    """Resample ``im`` to width_mm x height_mm at ``dpi`` and save it as a one-page PDF of exactly that size."""
    w, h = round(width_mm / 25.4 * dpi), round(height_mm / 25.4 * dpi)
    im.convert("RGB").resize((w, h), Image.LANCZOS).save(path, "PDF", resolution=dpi)


def printables() -> None:
    """assets/print/: the pail's wrap-round label, its lid print and the droppings strip, each a one-page PDF at true
    size, drawn by the same code as the simulator's textures and shapes."""
    PRINT.mkdir(parents=True, exist_ok=True)
    tmp = PRINT / "_label.png"
    pail_label(tmp)
    save_pdf(Image.open(tmp), PRINT / "pail_label.pdf", *LABEL_MM, 200)
    pail_lid(tmp)
    save_pdf(Image.open(tmp), PRINT / "pail_lid.pdf", LID_MM, LID_MM, 200)
    tmp.unlink()
    # the strip, seen from above with the wall at the top edge and the robot's left at the left edge
    ppm = 300 / 25.4 * 1000                       # 300 dpi in pixels per metre
    W, H = round(STRIP_MM[0] / 1000 * ppm), round(STRIP_MM[1] / 1000 * ppm)
    im = Image.new("RGB", (W, H), (219, 214, 201))
    d = ImageDraw.Draw(im)
    for xc, y, a, ln, wd in droppings():
        pts = []
        for t in np.linspace(0, 2 * math.pi, 28, endpoint=False):
            u, v = ln / 2 * math.cos(t), wd / 2 * math.sin(t)
            px, py = xc + math.cos(a) * u - math.sin(a) * v, y + math.sin(a) * u + math.cos(a) * v
            pts.append(((g.DROP_Y[1] - py) * ppm, -px * ppm))
        d.polygon(pts, fill=(18, 14, 11))
    im.save(PRINT / "droppings_strip.pdf", "PDF", resolution=300)


if __name__ == "__main__":
    import sys

    printables()
    if "--printables" in sys.argv:              # only the printables (the scene's assets are left as they are)
        for p in sorted(PRINT.iterdir()):
            print(p.relative_to(OUT), p.stat().st_size, "bytes")
        raise SystemExit
    TEX.mkdir(parents=True, exist_ok=True)
    for f in TEX.glob("*.png"):
        f.unlink()
    build_atlas(TEX / ATLAS)
    build_station()
    build_block()
    build_wall()
    build_pail()
    build_truck()
    build_cup()
    for p in sorted(OUT.rglob("*.*")):
        print(p.relative_to(OUT), p.stat().st_size, "bytes")
