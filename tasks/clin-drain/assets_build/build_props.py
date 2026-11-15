"""Build the visual-only bench props of clin-drain from NVIDIA SimReady packages (CC-BY-4.0).

Run: blender -b --factory-startup --python assets_build/build_props.py -- <pkg_dir> <simready_dir> <verified_dir>

<simready_dir> holds the downloaded nvidia/simready-assets packages (one folder per package, as on Hugging Face);
<verified_dir> holds optional pre-checked models as <name>/usd/asset.usda (none is used by the current prop list).
Each package is flattened with every physics schema, joint, physics material and MDL shader stripped (its
UsdPreviewSurface PBR network stays), its textures copied next to it at <= 512 px (system python3 + Pillow), and
written to assets/props/<name>/<name>.usdc. assets/props/bench_props.usda then places every prop on the bench in station
world coordinates (metres, Z up, z = 0 on the table top): labkit.SCENERY references it once, outside every task
object, so the props have no collider, no mass and no scene entity.
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
# Clinical context of a wound-drain recharge at the bedside: stainless kidney dish (emptying), folded sterile drape,
# nitrile glove box, disinfectant trigger spray, specimen bag, Luer syringe.
# All at x >= 0.42 m and below 0.20 m: the demo's TCP targets stop at x 0.42 m (the closest, (0.375, -0.05), is 85 mm
# above the mat) and the arms park at x 0.20 m, so no prop is in a robot path.
PROPS = [
    ("kidney_dish", "container_system_sterilising_c01", (0.515, 0.040), 90.0, (0.0, 0.0), (1.0, 1.0, 1.0)),
    ("drape", "drape_surgical_a01", (0.705, -0.010), 90.0, (0.0, 0.0), (1.0, 1.0, 1.0)),
    # VERIFIED.md box_glove_a01: model 178 x 117 x 56 mm vs retail 100-ct boxes 222-241 x 121-127 x 51-76 mm
    ("glove_box", "box_glove_a01", (0.600, 0.300), 272.0, (0.0, 0.0), (1.27, 1.05, 1.15)),
    ("disinfectant", "disinfectant_a01", (0.715, 0.235), 205.0, (0.0, 0.0), (1.0, 1.0, 1.0)),
    ("specimen_bag", "medical_sample_bag_a01", (0.615, -0.300), 100.0, (0.0, -90.0), (1.0, 1.0, 1.0)),
    ("syringe", "syringe_a01", (0.510, -0.112), 12.0, (0.0, 0.0), (1.0, 1.0, 1.0)),
    # A clear glass beaker was left out: it shows only as a faint outline in the official top camera.
]


def resize_copy(src, dst):
    code = ("import sys; from PIL import Image; im = Image.open(sys.argv[1]); n = int(sys.argv[3]);"
            "im = im.resize((n, n * im.size[1] // im.size[0]), Image.LANCZOS) if max(im.size) > n else im;"
            "im.save(sys.argv[2])")
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
    leak = [s for s in (out.read_bytes().decode("latin-1"),) if "/mnt/" in s or "/home/" in s]
    report["abs_path_leak"] = bool(leak)
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


OUT.mkdir(parents=True, exist_ok=True)
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
    # the model keeps its own root transform (the beaker's x1.128 rescale) on a child prim
    UsdGeom.Xform.Define(scene, f"/props/{name}/model").GetPrim().GetReferences().AddReference(f"./{name}/{name}.usdc")
    # rotations first (about the model origin), then shift so the rotated bbox is centred on (x, y) with z min = 0
    xf.AddTranslateOp().Set(Gf.Vec3d(x - (lo[0] + hi[0]) / 2, y - (lo[1] + hi[1]) / 2, -lo[2]))
    xf.AddRotateZOp(opSuffix="yaw").Set(yaw)
    xf.AddRotateYOp().Set(ry)
    xf.AddRotateXOp().Set(rx)
    xf.AddScaleOp().Set(Gf.Vec3f(*s))
    size = [round((hi[i] - lo[i]) * 1000, 1) for i in range(3)]
    summary.append({"name": name, "simready": pkg, "centre_xy_m": [x, y], "yaw_deg": yaw, "rot_xy_deg": [rx, ry],
                    "scale": list(s), "placed_bbox_mm": size,
                    "x_range_m": [round(x - size[0] / 2000, 3), round(x + size[0] / 2000, 3)],
                    "y_range_m": [round(y - size[1] / 2000, 3), round(y + size[1] / 2000, 3)], **rep})
scene.GetRootLayer().Save()
(OUT / "props.json").write_text(json.dumps(summary, indent=1) + "\n")
for row in summary:
    print(json.dumps(row))
