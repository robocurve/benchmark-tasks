"""Build the glove-box visual and the visual-only bench props of gondola-divider-planogram-respace from NVIDIA
SimReady packages (CC-BY-4.0), plus the planogram card.

Run: blender -b --factory-startup --python assets_build/build_props.py -- <pkg_dir> <simready_dir> <verified_dir>

<simready_dir> holds the downloaded nvidia/simready-assets packages (one folder per package, as on Hugging Face);
<verified_dir> holds optional pre-checked models as <name>/usd/asset.usda (none is used here).
Each package is flattened with every physics schema, joint, physics material and MDL shader stripped (its
UsdPreviewSurface PBR network stays), its textures copied next to it at <= 512 px (system python3 + Pillow), and
written to assets/props/<name>/<name>.usdc.
- assets/props/glove_box.usda: the box of nitrile exam gloves centred on its part origin (labkit VISUALS of glove_l and
  glove_r; their 178 x 117 x 56 mm box colliders stay invisible).
- assets/props/bench_props.usda: every bench prop placed in station world coordinates (metres, Z up, z = 0 on the table
  top); labkit.SCENERY references it once, outside every task object (no collider, no mass, no scene entity).
- assets/props/planogram_card.*: a 148 x 105 mm laminated card (texture drawn by assets_build/planogram_card.py).
"""
import json
import pathlib
import shutil
import subprocess
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

argv = sys.argv[sys.argv.index("--") + 1:]
PKG = pathlib.Path(argv[0]).resolve()
SIMREADY = pathlib.Path(argv[1]).resolve()
VERIFIED = pathlib.Path(argv[2]).resolve()
OUT = PKG / "assets" / "props"
TEX_PX = 512

# name, SimReady package, bench centre (x, y) m in station world, yaw deg, extra rotation (rx, ry) deg, scale (sx, sy, sz)
# Retail stocking context, all behind the shelf (x >= 0.48 m), where no arm waypoint goes (the arms work at
# x 0.26 m, |y| <= 0.08 m, and park at (0.16, +-0.30) m): a handheld barcode scanner in its cradle, a tape dispenser
# and two spare boxes of the same gloves stacked behind the shelf (restock). Packages with maker-style marks (scissors,
# packing tape) are not used; the mini basket was dropped for size (9 MB mesh).
PROPS = [
    ("barcode_scanner", "computer_scanner_barcode_d01", (0.640, 0.250), 200.0, (0.0, 0.0), (1.0, 1.0, 1.0)),
    ("tape_dispenser", "misc_tape_dispenser_a01", (0.560, -0.230), 75.0, (0.0, 0.0), (1.0, 1.0, 1.0)),
]
CARD_XY, CARD_YAW = (0.535, -0.060), -90.0  # planogram card on the bench behind the shelf
CARD_MM = (148.0, 105.0)
SPARE_XY, SPARE_YAW = (0.690, -0.065), 90.0  # two spare glove boxes, stacked


def resize_copy(src, dst):
    code = ("import sys; from PIL import Image; im = Image.open(sys.argv[1]); n = int(sys.argv[3]);"
            "im = im.resize((n, n * im.size[1] // im.size[0]), Image.LANCZOS) if max(im.size) > n else im;"
            "im.save(sys.argv[2], optimize=True)")
    subprocess.run(["python3", "-I", "-c", code, str(src), str(dst), str(TEX_PX)], check=True)


def flatten(pkg, name):
    verified = (VERIFIED / pkg / "usd" / "asset.usda").is_file()  # keep its OmniGlass (core MDL, found by name)
    dst = OUT / name
    if dst.exists():
        shutil.rmtree(dst)
    (dst / "Textures").mkdir(parents=True)
    src = VERIFIED / pkg / "usd" / "asset.usda" if verified else next((SIMREADY / pkg / "simready_usd").glob(f"sm_{pkg}_01.usd*"))
    st = Usd.Stage.Open(str(src))
    root = st.GetDefaultPrim()
    for vs in ("PhysX", "Newton", "MuJoCo"):
        if root.GetVariantSets().HasVariantSet(vs):
            root.GetVariantSets().GetVariantSet(vs).SetVariantSelection("Disabled")
    layer = st.Flatten()
    layer.documentation = f"Visual-only copy of {pkg}/{src.parent.name}/{src.name}"
    drop, missing = [], []

    def visit(path):
        spec = layer.GetObjectAtPath(path)
        if not isinstance(spec, Sdf.PrimSpec):
            return
        if (spec.name in ("Joints", "PhysicsMaterials", "Grasp", "ReferencePrims") or spec.typeName.startswith("Physics")
                or ("info:mdl:sourceAsset" in spec.attributes and not verified)):
            drop.append(spec.path)
            return
        if spec.typeName == "Material" and not verified:
            for a in list(spec.attributes):
                if a.name.startswith("outputs:mdl"):
                    spec.RemoveProperty(a)
        if spec.HasInfo("apiSchemas"):
            items = [s for s in spec.GetInfo("apiSchemas").GetAddedOrExplicitItems() if "Physic" not in s and "Physx" not in s]
            spec.SetInfo("apiSchemas", Sdf.TokenListOp.Create(prependedItems=items))
        if spec.HasInfo("instanceable"):
            spec.instanceable = False
        if "identifier" in spec.assetInfo:
            info = dict(spec.assetInfo)
            del info["identifier"]
            spec.assetInfo = info
        for a in list(spec.attributes):
            if a.name.startswith(("physics:", "physx")):
                spec.RemoveProperty(a)
            elif a.typeName == Sdf.ValueTypeNames.Asset and a.default:
                p = pathlib.Path(a.default.resolvedPath or (src.parent / "payloads" / a.default.path).resolve())
                if p.suffix.lower() in (".png", ".jpg", ".exr"):
                    if not p.is_file():
                        missing.append(p.name)
                        continue
                    if not (dst / "Textures" / p.name).exists():
                        resize_copy(p, dst / "Textures" / p.name)
                    a.default = Sdf.AssetPath(f"./Textures/{p.name}")
        for r in list(spec.relationships):
            if "physics" in r.name:
                spec.RemoveProperty(r)

    layer.Traverse(Sdf.Path.absoluteRootPath, visit)
    if missing:
        raise SystemExit(f"{pkg}: textures not found: {sorted(set(missing))}")
    for p in sorted(drop, key=lambda x: -len(str(x))):
        parent = layer.GetPrimAtPath(p.GetParentPath()) or layer.pseudoRoot
        del parent.nameChildren[p.name]
    out = dst / f"{name}.usdc"
    layer.Export(str(out))
    st = Usd.Stage.Open(str(out))
    report = {"meshes": 0, "no_preview_surface": [], "glass": []}
    for prim in st.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        report["meshes"] += 1
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        shader = mat.ComputeSurfaceSource()[0] if mat else None
        if shader and shader.GetSourceAsset("mdl"):
            report.setdefault("mdl", []).append(str(shader.GetSourceAsset("mdl").path))
            continue
        if not shader or shader.GetIdAttr().Get() != "UsdPreviewSurface":
            report["no_preview_surface"].append(str(prim.GetPath()))
            continue
        if "glass" in mat.GetPrim().GetName().lower():  # clear glass/plastic: translucent preview surface
            spec = shader.GetPrim().GetPrimStack()[-1]
            for key, v in (("inputs:opacity", 0.3), ("inputs:roughness", 0.05)):
                a = spec.attributes[key] if key in spec.attributes else Sdf.AttributeSpec(spec, key, Sdf.ValueTypeNames.Float)
                a.default = v
            report["glass"].append(mat.GetPrim().GetName())
    st.GetRootLayer().Save()
    lic = SIMREADY / pkg / ".metadata" / "LICENSE.txt"
    if lic.is_file():
        shutil.copy2(lic, dst / "LICENSE.txt")
    data = out.read_bytes()
    report["abs_path_leak"] = any(str(d).encode() in data for d in (SIMREADY, VERIFIED, PKG, pathlib.Path.home()))
    return out, report


def bounds(stage_path, xf_ops):
    """World AABB of the prop file after (rx, ry, yaw, scale)."""
    st = Usd.Stage.CreateInMemory()
    xf = UsdGeom.Xform.Define(st, "/p")
    UsdGeom.Xform.Define(st, "/p/model").GetPrim().GetReferences().AddReference(str(stage_path))
    rx, ry, yaw, s = xf_ops
    xf.AddRotateZOp(opSuffix="yaw").Set(yaw)
    xf.AddRotateYOp().Set(ry)
    xf.AddRotateXOp().Set(rx)
    xf.AddScaleOp().Set(Gf.Vec3f(*s))
    r = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(xf.GetPrim()).ComputeAlignedRange()
    return Gf.Vec3d(r.GetMin()), Gf.Vec3d(r.GetMax())


def card(path_png):
    """A 148 x 105 mm laminated card (0.5 mm) with the planogram texture on its top face, origin at its centre."""
    dst = OUT / "planogram_card"
    if dst.exists():
        shutil.rmtree(dst)
    (dst / "Textures").mkdir(parents=True)
    shutil.copy2(path_png, dst / "Textures" / "planogram_card.png")
    st = Usd.Stage.CreateNew(str(dst / "planogram_card.usda"))
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(st, "/card")
    st.SetDefaultPrim(root.GetPrim())
    w, h, t = CARD_MM[0] / 2000, CARD_MM[1] / 2000, 0.0005
    mesh = UsdGeom.Mesh.Define(st, "/card/face")
    mesh.CreatePointsAttr([(-w, -h, t), (w, -h, t), (w, h, t), (-w, h, t)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateExtentAttr([(-w, -h, t), (w, h, t)])
    uv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    uv.Set([(0, 0), (1, 0), (1, 1), (0, 1)])
    edge = UsdGeom.Cube.Define(st, "/card/body")
    edge.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(edge).SetScale(Gf.Vec3f(2 * w, 2 * h, t))
    UsdGeom.XformCommonAPI(edge).SetTranslate(Gf.Vec3d(0, 0, t / 2 - 0.00005))
    mat = UsdShade.Material.Define(st, "/card/Looks/laminated_card")
    sh = UsdShade.Shader.Define(st, "/card/Looks/laminated_card/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
    sh.CreateInput("clearcoat", Sdf.ValueTypeNames.Float).Set(0.6)
    rd = UsdShade.Shader.Define(st, "/card/Looks/laminated_card/st")
    rd.CreateIdAttr("UsdPrimvarReader_float2")
    rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tx = UsdShade.Shader.Define(st, "/card/Looks/laminated_card/tex")
    tx.CreateIdAttr("UsdUVTexture")
    tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set("./Textures/planogram_card.png")
    tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    white = UsdShade.Material.Define(st, "/card/Looks/card_edge")
    ws = UsdShade.Shader.Define(st, "/card/Looks/card_edge/surface")
    ws.CreateIdAttr("UsdPreviewSurface")
    ws.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.92, 0.92, 0.9))
    white.CreateSurfaceOutput().ConnectToSource(ws.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    UsdShade.MaterialBindingAPI.Apply(edge.GetPrim()).Bind(white)
    st.GetRootLayer().Save()
    return dst / "planogram_card.usda"


OUT.mkdir(parents=True, exist_ok=True)

# glove box: centred on the part origin (box collider centre)
gout, grep_ = flatten("box_glove_a01", "glove_box_model")
glo, ghi = bounds(gout, (0.0, 0.0, 0.0, (1.0, 1.0, 1.0)))
gst = Usd.Stage.CreateNew(str(OUT / "glove_box.usda"))
UsdGeom.SetStageMetersPerUnit(gst, 1.0)
UsdGeom.SetStageUpAxis(gst, UsdGeom.Tokens.z)
gx = UsdGeom.Xform.Define(gst, "/glove_box")
gst.SetDefaultPrim(gx.GetPrim())
UsdGeom.Xform.Define(gst, "/glove_box/model").GetPrim().GetReferences().AddReference("./glove_box_model/glove_box_model.usdc")
gx.AddTranslateOp().Set(Gf.Vec3d(-(glo[0] + ghi[0]) / 2, -(glo[1] + ghi[1]) / 2, -(glo[2] + ghi[2]) / 2))
gst.GetRootLayer().Save()
print(json.dumps({"glove_box_bbox_mm": [round((ghi[i] - glo[i]) * 1000, 1) for i in range(3)], **grep_}))

png = OUT.parent.parent / "assets_build" / "planogram_card.png"
subprocess.run(["python3", "-I", str(OUT.parent.parent / "assets_build" / "planogram_card.py"), str(png)], check=True)
card_usd = card(png)
png.unlink()

scene = Usd.Stage.CreateNew(str(OUT / "bench_props.usda"))
UsdGeom.SetStageMetersPerUnit(scene, 1.0)
UsdGeom.SetStageUpAxis(scene, UsdGeom.Tokens.z)
top = UsdGeom.Xform.Define(scene, "/props")
scene.SetDefaultPrim(top.GetPrim())
summary = []
for name, pkg, (x, y), yaw, (rx, ry), s in PROPS:
    out, rep = flatten(pkg, name)
    lo, hi = bounds(out, (rx, ry, yaw, s))
    xf = UsdGeom.Xform.Define(scene, f"/props/{name}")
    UsdGeom.Xform.Define(scene, f"/props/{name}/model").GetPrim().GetReferences().AddReference(f"./{name}/{name}.usdc")
    xf.AddTranslateOp().Set(Gf.Vec3d(x - (lo[0] + hi[0]) / 2, y - (lo[1] + hi[1]) / 2, -lo[2]))
    xf.AddRotateZOp(opSuffix="yaw").Set(yaw)
    xf.AddRotateYOp().Set(ry)
    xf.AddRotateXOp().Set(rx)
    xf.AddScaleOp().Set(Gf.Vec3f(*s))
    size = [round((hi[i] - lo[i]) * 1000, 1) for i in range(3)]
    summary.append({"name": name, "simready": pkg, "centre_xy_m": [x, y], "yaw_deg": yaw, "placed_bbox_mm": size,
                    "x_range_m": [round(x - size[0] / 2000, 3), round(x + size[0] / 2000, 3)],
                    "y_range_m": [round(y - size[1] / 2000, 3), round(y + size[1] / 2000, 3)], **rep})
cx = UsdGeom.Xform.Define(scene, "/props/planogram_card")
UsdGeom.Xform.Define(scene, "/props/planogram_card/model").GetPrim().GetReferences().AddReference(
    "./planogram_card/planogram_card.usda")
cx.AddTranslateOp().Set(Gf.Vec3d(CARD_XY[0], CARD_XY[1], 0.0))
cx.AddRotateZOp(opSuffix="yaw").Set(CARD_YAW)
summary.append({"name": "planogram_card", "centre_xy_m": list(CARD_XY), "size_mm": list(CARD_MM)})
for i in range(2):
    sx = UsdGeom.Xform.Define(scene, f"/props/spare_gloves_{i}")
    UsdGeom.Xform.Define(scene, f"/props/spare_gloves_{i}/model").GetPrim().GetReferences().AddReference("./glove_box.usda")
    sx.AddTranslateOp().Set(Gf.Vec3d(SPARE_XY[0], SPARE_XY[1] + 0.004 * i, (ghi[2] - glo[2]) * (i + 0.5)))
    sx.AddRotateZOp(opSuffix="yaw").Set(SPARE_YAW + 4.0 * i)
summary.append({"name": "spare_gloves x2", "centre_xy_m": list(SPARE_XY), "model": "glove_box.usda"})
scene.GetRootLayer().Save()
(OUT / "props.json").write_text(json.dumps(summary, indent=1) + "\n")
for row in summary:
    print(json.dumps(row))
