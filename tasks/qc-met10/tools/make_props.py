"""Visual-only bench props from NVIDIA SimReady packages (CC-BY-4.0), for the qc-met10 bench dressing.

    blender -b --factory-startup --python tools/make_props.py -- <simready packages dir>

Runs under Blender's bundled python (pxr); textures are converted with ffmpeg. For each prop in PROPS: flatten
sm_<pkg>_01.usd with every physics variant Disabled, drop physics schemas/attributes, joints, physics materials and
MDL shaders (the UsdPreviewSurface PBR networks stay), strip the absolute source path Flatten writes, copy each
texture as a 512 px JPEG next to it and write assets/qc_met10/props/<pkg>/<pkg>.usdc + LICENSE.txt. Then writes assets/qc_met10/bench_props.usda, which places every
prop (bottom centre on the bench top) in the bench_props part frame (station frame, z = 0 on the bench top).
"""

from __future__ import annotations

import json
import pathlib
import shutil
import subprocess
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

HERE = pathlib.Path(__file__).resolve().parent.parent
OUT = HERE / "assets" / "qc_met10"
SRC = pathlib.Path(sys.argv[sys.argv.index("--") + 1])
TEX_PX = 512  # props cover < 260 px of the 1280 x 704 top camera

# prop -> list of (x, y, yaw deg) placements in the station frame (bottom centre on the bench top). All outside the
# arms' swept volume of the demo (x >= 0.48 m is never reached below 0.30 m; mj_check Sim sweep) and off the robot path.
PROPS = {
    "computer_keyboard_a01": [(0.640, 0.080, 90.0)],  # test-stand PC keyboard behind the stand
    "computer_mouse_office_b01": [(0.640, -0.270, 90.0)],  # mouse on its pad, right of the keyboard
    "instrument_caliper_a01": [(0.520, 0.100, 90.0)],  # caliper for specimen width/thickness (ASTM D638 10.1)
    "ppe_eyeProtection_safetyGlasses_a01": [(0.600, 0.420, 20.0)],
}


def _jpeg(src: pathlib.Path, dst: pathlib.Path) -> None:
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", str(src), "-vf",
                    f"scale='min({TEX_PX},iw)':-2:flags=lanczos,format=yuvj444p", "-q:v", "3", str(dst)], check=True)


def flatten(pkg: str) -> dict:
    src = SRC / pkg / "simready_usd" / f"sm_{pkg}_01.usd"
    dst = OUT / "props" / pkg
    if dst.exists():
        shutil.rmtree(dst)
    (dst / "Textures").mkdir(parents=True)
    st = Usd.Stage.Open(str(src))
    root = st.GetDefaultPrim()
    for vs in ("PhysX", "Newton", "MuJoCo"):
        if root.GetVariantSets().HasVariantSet(vs):
            root.GetVariantSets().GetVariantSet(vs).SetVariantSelection("Disabled")
    layer = st.Flatten()
    layer.documentation = f"Visual-only copy of nvidia/simready-assets {pkg}/simready_usd/sm_{pkg}_01.usd (CC-BY-4.0)"
    drop, done = [], {}

    def visit(path):
        spec = layer.GetObjectAtPath(path)
        if not isinstance(spec, Sdf.PrimSpec):
            return
        if (spec.name in ("Joints", "PhysicsMaterials", "Grasp", "ReferencePrims") or spec.typeName.startswith("Physics")
                or "info:mdl:sourceAsset" in spec.attributes):  # MDL shaders render red on the arena hosts
            drop.append(spec.path)
            return
        if spec.typeName == "Material":
            for a in list(spec.attributes):
                if a.name.startswith("outputs:mdl"):
                    spec.RemoveProperty(a)
        if spec.HasInfo("apiSchemas"):
            items = [s for s in spec.GetInfo("apiSchemas").GetAddedOrExplicitItems() if "Physic" not in s and "Physx" not in s]
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
                        raise SystemExit(f"{pkg}: texture not found: {p.name}")
                    name = p.stem + ".jpg"
                    if name not in done:
                        _jpeg(p, dst / "Textures" / name)
                        done[name] = p.name
                    a.default = Sdf.AssetPath(f"./Textures/{name}")
        for r in list(spec.relationships):
            if "physics" in r.name:
                spec.RemoveProperty(r)

    layer.Traverse(Sdf.Path.absoluteRootPath, visit)
    for p in sorted(drop, key=lambda x: -len(str(x))):
        parent = layer.GetPrimAtPath(p.GetParentPath()) or layer.pseudoRoot
        del parent.nameChildren[p.name]
    out = dst / f"{pkg}.usdc"
    layer.Export(str(out))
    lic = SRC / pkg / ".metadata" / "LICENSE.txt"
    if lic.is_file():
        shutil.copy2(lic, dst / "LICENSE.txt")
    st = Usd.Stage.Open(str(out))
    for prim in st.Traverse():  # clear lenses: the dropped MDL glass leaves an opaque preview surface
        if prim.IsA(UsdShade.Material) and "glass" in prim.GetName():
            sh = UsdShade.Material(prim).ComputeSurfaceSource()[0]
            if sh:
                sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.25)
                sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.05)
    st.GetRootLayer().Save()
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(
        st.GetDefaultPrim()).ComputeAlignedRange()
    leaks = [str(p.GetPath()) for p in st.Traverse() if p.HasAPI(Usd.SchemaRegistry.GetTypeFromName("PhysicsRigidBodyAPI"))]
    return {"pkg": pkg, "usd": str(out.relative_to(HERE)), "min": list(bb.GetMin()), "max": list(bb.GetMax()),
            "textures": done, "rigid_left": leaks, "bytes": sum(f.stat().st_size for f in dst.rglob("*") if f.is_file())}


def compose(info: dict) -> None:
    """bench_props.usda: one Xform per placement; the prop's bottom-centre (its bbox centre in x/y, min z) on (x, y, 0)."""
    layer = Sdf.Layer.CreateNew(str(OUT / "bench_props.usda"))
    st = Usd.Stage.Open(layer)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(st, "/bench_props")
    st.SetDefaultPrim(root.GetPrim())
    for pkg, places in PROPS.items():
        lo, hi = Gf.Vec3d(*info[pkg]["min"]), Gf.Vec3d(*info[pkg]["max"])
        c = Gf.Vec3d((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2])
        for i, (x, y, yaw) in enumerate(places):
            xf = UsdGeom.Xform.Define(st, f"/bench_props/{pkg}_{i}")
            xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
            xf.AddRotateZOp().Set(float(yaw))
            inner = UsdGeom.Xform.Define(st, f"/bench_props/{pkg}_{i}/model")
            inner.AddTranslateOp().Set(-c)
            inner.GetPrim().GetReferences().AddReference(f"./props/{pkg}/{pkg}.usdc")
    layer.Save()


if __name__ == "__main__":
    info = {pkg: flatten(pkg) for pkg in PROPS}
    compose(info)
    (OUT / "props" / "props.json").write_text(json.dumps({"placements": PROPS, "props": info}, indent=1) + "\n")
    print(json.dumps({k: {"bytes": v["bytes"], "min": [round(x, 4) for x in v["min"]], "max": [round(x, 4) for x in v["max"]],
                          "rigid_left": v["rigid_left"]} for k, v in info.items()}))
