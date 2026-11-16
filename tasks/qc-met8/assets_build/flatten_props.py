"""SimReady package -> visual-only USD for the bench dressing (run with any Python that has pxr and Pillow):

    python assets_build/flatten_props.py <folder holding the nvidia/simready-assets packages> <pkg> [<pkg> ...]

Writes assets/props/<pkg>/<pkg>.usdc + Textures/ + LICENSE.txt. The package as downloaded does not load in the arena: its
materials point at ../source_assets/OmniPBR.mdl / OmniGlass.mdl (not in the dataset) and its instanced material prims
expire when labkit un-instances them ("Accessed invalid expired 'Material' instance prim", GPU run 2026-11-14). So the
stage is flattened with the PhysX/Newton/MuJoCo variants disabled, physics and MDL shaders are dropped, instancing is
removed, and the UsdPreviewSurface PBR materials with their textures (downscaled to 512 px) are kept; clear glass gets
a translucent preview surface. Copied from the wet-stirr1 v2 package's flatten_simready.py (adapted from wet-scrape1's make_assets.py, GPU-proven)."""

from __future__ import annotations

import pathlib
import shutil
import sys

from PIL import Image
from pxr import Sdf, Usd, UsdGeom, UsdShade

OUT = pathlib.Path(__file__).resolve().parent.parent / "assets" / "props"


def _copy_texture(src: pathlib.Path, dst: pathlib.Path, size: int = 512) -> None:
    im = Image.open(src)
    if max(im.size) > size:
        im = im.resize((size, size * im.size[1] // im.size[0]), Image.LANCZOS)
    im.save(dst)


def flatten(root_dir: pathlib.Path, pkg: str) -> pathlib.Path:
    dst = OUT / pkg
    (dst / "Textures").mkdir(parents=True, exist_ok=True)
    src = root_dir / pkg / "simready_usd" / f"sm_{pkg}_01.usd"
    st = Usd.Stage.Open(str(src))
    root = st.GetDefaultPrim()
    for vs in ("PhysX", "Newton", "MuJoCo"):
        if root.GetVariantSets().HasVariantSet(vs):
            root.GetVariantSets().GetVariantSet(vs).SetVariantSelection("Disabled")
    layer = st.Flatten()
    layer.documentation = f"Visual-only copy of nvidia/simready-assets {pkg}/simready_usd/sm_{pkg}_01.usd"
    drop, missing = [], []

    def visit(path):
        spec = layer.GetObjectAtPath(path)
        if not isinstance(spec, Sdf.PrimSpec):
            return
        if (spec.name in ("Joints", "PhysicsMaterials", "Grasp", "ReferencePrims") or spec.typeName.startswith("Physics")
                or "info:mdl:sourceAsset" in spec.attributes):
            drop.append(spec.path)
            return
        if spec.typeName == "Material":
            for a in list(spec.attributes):
                if a.name.startswith("outputs:mdl"):
                    spec.RemoveProperty(a)
        info = spec.GetInfo("apiSchemas") if spec.HasInfo("apiSchemas") else None
        if info is not None:
            items = [s for s in info.GetAddedOrExplicitItems() if "Physic" not in s and "Physx" not in s]
            spec.SetInfo("apiSchemas", Sdf.TokenListOp.Create(prependedItems=items))
        if spec.HasInfo("instanceable"):
            spec.instanceable = False
        if "identifier" in spec.assetInfo:  # the absolute source path of the flattened file
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
                    _copy_texture(p, dst / "Textures" / p.name)
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
    out = dst / f"{pkg}.usdc"
    layer.Export(str(out))
    st = Usd.Stage.Open(str(out))
    for prim in st.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if mat and "glass" in mat.GetPrim().GetName():  # clear glass: translucent preview surface (MDL dropped)
            spec = mat.ComputeSurfaceSource()[0].GetPrim().GetPrimStack()[-1]
            for key, v in (("inputs:opacity", 0.3), ("inputs:roughness", 0.05)):
                a = spec.attributes[key] if key in spec.attributes else Sdf.AttributeSpec(spec, key, Sdf.ValueTypeNames.Float)
                a.default = v
    st.GetRootLayer().Save()
    shutil.copy2(root_dir / pkg / ".metadata" / "LICENSE.txt", dst / "LICENSE.txt")
    return out


if __name__ == "__main__":
    for pkg in sys.argv[2:]:
        out = flatten(pathlib.Path(sys.argv[1]), pkg)
        st = Usd.Stage.Open(str(out))
        r = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(
            st.GetPseudoRoot()).ComputeAlignedRange()
        print(out.relative_to(OUT.parent.parent), "bbox_min", [round(v, 4) for v in r.GetMin()], "bbox_max", [round(v, 4) for v in r.GetMax()])
