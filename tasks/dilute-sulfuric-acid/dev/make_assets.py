"""Assets of tasks/dilute-sulfuric-acid, made from one NVIDIA USDCraft-10K model (Apache-2.0,
huggingface.co/datasets/nvidia/USDCraft-10K):

    python make_assets.py <folder with R482_01/model.usdc> <task assets folder>

  rigid/medical_outdoor_garden_transport_sports/R482_glass_beaker/01_low_form_250_ml_beaker/physx/model.usdc
  (a low-form borosilicate beaker with spout, 70 mm across and 95 mm tall: the size of an ISO 3819 low-form 250 mL beaker)

Writes, each one rigid body on /Asset with its origin at the centre of the beaker's foot, z up, spout toward -y:
  beaker_250.usdc   the model as it is (70 x 95 mm), 0.10 kg, with a printed label: SULFURIC ACID 95-98 % and the GHS
                    corrosive pictogram, at +x (square to the spout: the task turns this beaker so the label faces the robot)
  beaker_400.usdc   the same model scaled to 80 x 110 mm (an ISO 3819 low-form 400 mL beaker), 0.14 kg, label: WATER (deionised) at -x
  textures/         the two label images (drawn here with PIL)
The model's signed-distance collider is replaced by primitives: a disc floor and a ring of 36 wall boxes (a 2.5 mm wall
measured from the mesh, scaled for the 400 mL beaker). Needs pxr and PIL (the Isaac container has both).
"""
import math
import shutil
import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

# the 250 mL model, measured from its mesh (beaker_geom.py): wall r 32.5..35.0 mm, floor 0..5 mm, rim at 95 mm
R_OUT, WALL_T, FLOOR_T, HEIGHT = 0.035, 0.0025, 0.005, 0.095
BEAKERS = {
    # name: (xy scale, z scale, glass mass kg, label lines, label kind, label direction in the model frame (rad))
    "beaker_250": (1.0, 1.0, 0.10, ("SULFURIC ACID", "H2SO4  95-98 %", "DANGER"), "acid", 0.0),
    "beaker_400": (80 / 70, 110 / 95, 0.14, ("WATER", "deionised", ""), "water", math.pi),
}


def label_png(path: Path, lines, kind: str) -> None:
    """A white paper label 600 x 420 px: the GHS05 corrosive diamond (acid only) and the text."""
    from PIL import Image, ImageDraw, ImageFont

    W, H = 600, 420
    im = Image.new("RGB", (W, H), (250, 250, 246))
    d = ImageDraw.Draw(im)
    d.rectangle([4, 4, W - 5, H - 5], outline=(30, 30, 30), width=6)
    big = ImageFont.load_default(size=58)
    mid = ImageFont.load_default(size=48)
    if kind == "acid":
        # GHS05: a red-bordered diamond with liquid dripping from two test tubes onto a bar and a hand
        cx, cy, s = 150, 250, 120
        diamond = [(cx, cy - s), (cx + s, cy), (cx, cy + s), (cx - s, cy)]
        d.polygon(diamond, fill=(255, 255, 255), outline=(220, 0, 0))
        for k in range(14):
            d.polygon([(cx, cy - s + k), (cx + s - k, cy), (cx, cy + s - k), (cx - s + k, cy)], outline=(220, 0, 0))
        for tx, ang in ((cx - 45, -25), (cx + 25, 25)):  # two tilted tubes
            d.line([(tx, cy - 70), (tx + (15 if ang > 0 else -15), cy - 25)], fill=(0, 0, 0), width=14)
        for x0 in (cx - 50, cx + 30):  # drops
            for k in range(3):
                d.ellipse([x0 - 6, cy - 15 + 14 * k, x0 + 6, cy - 3 + 14 * k], fill=(0, 0, 0))
        d.rectangle([cx - 80, cy + 38, cx - 10, cy + 52], fill=(0, 0, 0))  # bar being eaten
        d.polygon([(cx + 5, cy + 30), (cx + 75, cy + 30), (cx + 75, cy + 55), (cx + 5, cy + 55)], fill=(0, 0, 0))  # hand
        x_text = 282
        d.text((x_text, 70), lines[0].split()[0], font=big, fill=(0, 0, 0))
        d.text((x_text, 140), lines[0].split()[1], font=big, fill=(0, 0, 0))
        d.text((x_text, 230), lines[1], font=ImageFont.load_default(size=40), fill=(0, 0, 0))
        d.text((x_text, 300), lines[2], font=mid, fill=(200, 0, 0))
    else:
        d.text((W // 2, 160), lines[0], font=ImageFont.load_default(size=110), fill=(0, 0, 0), anchor="mm")
        d.text((W // 2, 290), lines[1], font=mid, fill=(0, 0, 0), anchor="mm")
    im.save(path)


def label_mesh(stage, path: str, r: float, z0: float, z1: float, half_angle: float, centre_angle: float, png: str):
    """A curved strip on the cylinder of radius r from z0 to z1, spanning centre_angle +- half_angle, textured."""
    mat = UsdShade.Material.Define(stage, f"{path}_look")
    pbr = UsdShade.Shader.Define(stage, f"{path}_look/pbr")
    pbr.CreateIdAttr("UsdPreviewSurface")
    pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    reader = UsdShade.Shader.Define(stage, f"{path}_look/st")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, f"{path}_look/tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./textures/{png}")
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    rgb = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    mat.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")

    n = 16
    pts, uvs, idx = [], [], []
    for i in range(n + 1):
        u = i / n
        a = centre_angle - half_angle * (1 - 2 * u)  # u = 0 at the left as seen from outside the beaker
        for z, v in ((z0, 0.0), (z1, 1.0)):
            pts.append(Gf.Vec3f(r * math.cos(a), r * math.sin(a), z))
            uvs.append(Gf.Vec2f(u, v))
    for i in range(n):
        a, b = 2 * i, 2 * i + 2
        idx += [a, b, b + 1, a + 1]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr([4] * n)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set(Vt.Vec2fArray(uvs))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)


def make(src: Path, dst: Path, name: str) -> None:
    sxy, sz, mass, lines, kind, label_dir = BEAKERS[name]
    st = Usd.Stage.Open(str(src))
    link = st.GetPrimAtPath("/Asset/Links/beaker")
    st.RemovePrim("/Asset/Links/beaker/Collisions")
    for j in list(st.GetPrimAtPath("/Asset/Joints").GetChildren()):
        st.RemovePrim(j.GetPath())
    link.RemoveAPI(UsdPhysics.MassAPI)
    link.RemoveAPI(UsdPhysics.RigidBodyAPI)
    for a in [a.GetName() for a in link.GetAttributes() if a.GetName().startswith(("physics:", "physx"))]:
        link.RemoveProperty(a)
    link.SetMetadata("apiSchemas", Sdf.TokenListOp())
    vis = UsdGeom.Xformable(st.GetPrimAtPath("/Asset/Links/beaker/Visuals"))
    if sxy != 1.0 or sz != 1.0:
        vis.AddScaleOp().Set(Gf.Vec3f(sxy, sxy, sz))
    root = st.GetPrimAtPath("/Asset")
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(float(mass))
    # centre of mass of a glass cup: about 40 % of its height (heavy floor)
    UsdPhysics.MassAPI(root).CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0.40 * HEIGHT * sz))

    r_out, t, h, ft = R_OUT * sxy, WALL_T * sxy, HEIGHT * sz, FLOOR_T * sz
    pm_path = "/Asset/PhysicsMaterials/glass_contact"
    col = "/Asset/Links/beaker/Colliders"
    UsdGeom.Scope.Define(st, col)

    def collide(prim):
        UsdPhysics.CollisionAPI.Apply(prim)
        prim.AddAppliedSchema("PhysxCollisionAPI")
        prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.001)
        prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(st.GetPrimAtPath(pm_path)),
                                                     UsdShade.Tokens.weakerThanDescendants, "physics")
        UsdGeom.Imageable(prim).MakeInvisible()

    cyl = UsdGeom.Cylinder.Define(st, f"{col}/floor")
    cyl.CreateRadiusAttr(r_out)
    cyl.CreateHeightAttr(ft)
    cyl.CreateAxisAttr("Z")
    cyl.AddTranslateOp().Set(Gf.Vec3d(0, 0, ft / 2))
    collide(cyl.GetPrim())
    n = 36
    rc = r_out - t / 2
    for k in range(n):
        a = 2 * math.pi * k / n
        c = UsdGeom.Cube.Define(st, f"{col}/wall{k:02d}")
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(rc * math.cos(a), rc * math.sin(a), (ft + h) / 2))
        c.AddRotateZOp().Set(math.degrees(a))
        c.AddScaleOp().Set(Gf.Vec3f(t, 2 * math.pi * r_out / n * 1.06, h - ft))
        collide(c.GetPrim())

    (dst / "textures").mkdir(parents=True, exist_ok=True)
    for f in (src.parent / "textures").glob("*.png"):
        shutil.copy(f, dst / "textures" / f.name)
    png = f"label_{kind}.png"
    label_png(dst / "textures" / png, lines, kind)
    # 600 x 420 px label: 50 x 35 mm on the 250 mL beaker, 57 x 40 on the 400 mL one
    lw, lh = 0.050 * sxy, 0.035 * sxy
    zc = 0.050 * sz
    label_mesh(st, "/Asset/Links/beaker/Visuals/label", r_out + 0.0004, zc - lh / 2, zc + lh / 2, lw / 2 / r_out, label_dir, png)
    st.GetRootLayer().Export(str(dst / f"{name}.usdc"))
    print(name, f"outer r {r_out * 1000:.1f} mm, wall {t * 1000:.2f}, floor {ft * 1000:.2f}, height {h * 1000:.1f}, "
                f"inner r {(r_out - t) * 1000:.1f}, mass {mass}")


if __name__ == "__main__":
    src, dst = Path(sys.argv[1]) / "R482_01" / "model.usdc", Path(sys.argv[2])
    dst.mkdir(parents=True, exist_ok=True)
    for name in BEAKERS:
        make(src, dst, name)
