"""SimReady packages -> visual-only bench props + one placement layer (bench_props.usda) for the mat's visuals.

    blender -b --factory-startup --python assets/props/src/flatten_props.py -- <simready packages dir>

For each package in PROPS: the stage is flattened with the PhysX/Newton/MuJoCo variants disabled, physics prims/APIs,
MDL shaders and instancing are dropped, the UsdPreviewSurface PBR network is kept, textures are re-encoded next to the
copy (JPEG, albedo 512 px, ORM/normal 256 px; ffmpeg) and meshes over FACE_CAP faces are decimated in Blender, to keep
the package small; the result goes to
assets/props/<pkg>/<pkg>.usdc. Then assets/visual/bench_props.usda places every prop by reference in the mat frame
(mat body origin = MAT_POS in wet_ivc1.py), resting on the table top, centred on its station (x, y) with a yaw.
These are render-only meshes on the fixed mat body: no collider, no mass, outside every arm path (see MOVIE-NOTES.md).
Adapted from the visual-skin flatten used by earlier packages of this batch (MDL dropped, abs paths stripped).
"""

from __future__ import annotations

import json
import math
import pathlib
import subprocess
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

PKG = pathlib.Path(__file__).resolve().parents[3]
OUT = PKG / "assets" / "props"
MAT_POS = (0.200, 0.000)  # wet_ivc1.MAT_POS (x, y); z origin = table top

# name -> (station x, station y, yaw deg). Arm sweep envelope (mj_check rollout, link spheres): x <= 0.447 m for
# |y| < 0.2, x <= 0.343 for |y| 0.2..0.4, x <= 0.078 for |y| 0.4..0.5, nothing beyond |y| 0.5.
PROPS = {
    "disinfectant_a01": (0.540, +0.250, 200.0),          # trigger spray, nozzle towards the work area
    "disinfectant_c01": (0.700, +0.270, 15.0),           # 1 L disinfectant canister
    "ppe_gloves_lightDuty_a01": (0.640, +0.470, 70.0),   # pair of nitrile gloves lying flat
    "box_glove_a01": (0.550, -0.275, 90.0),              # nitrile glove dispenser box
    "facemask_medical_a01": (0.740, -0.280, 90.0),       # box of surgical masks
    "cleaning_wipes_a01": (0.650, -0.480, 0.0),          # disinfectant wipes pack
}
FACE_CAP = 6000  # faces per mesh after decimation
SIZE = {"alb": 512}
DEFAULT_SIZE = 256


def _texture(src: pathlib.Path, dst_dir: pathlib.Path) -> str:
    size = next((v for k, v in SIZE.items() if f"_{k}" in src.stem), DEFAULT_SIZE)
    name = src.stem + ".jpg"
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", str(src), "-vf", f"scale='min({size},iw)':-2",
                    "-q:v", "3", str(dst_dir / name)], check=True)
    return name


def _decimate(mesh: UsdGeom.Mesh) -> None:
    """Collapse-decimate one mesh in Blender to about FACE_CAP faces, keeping its UVs, per-face material subsets and
    smooth-by-angle normals (the packaged meshes run to 110k faces; the GPU hosts' link is ~50 KB/s)."""
    import bpy
    import numpy as np

    pts = np.array(mesh.GetPointsAttr().Get(), float)
    counts = np.array(mesh.GetFaceVertexCountsAttr().Get(), int)
    idx = np.array(mesh.GetFaceVertexIndicesAttr().Get(), int)
    pv = UsdGeom.PrimvarsAPI(mesh).GetPrimvar("st")
    assert pv.GetInterpolation() == UsdGeom.Tokens.faceVarying, mesh.GetPath()
    uv = np.array(pv.ComputeFlattened(), float)
    subsets = UsdGeom.Subset.GetAllGeomSubsets(mesh)
    mat_id = np.zeros(len(counts), int)
    for k, s in enumerate(subsets):
        mat_id[np.array(s.GetIndicesAttr().Get(), int)] = k
    starts = np.r_[0, np.cumsum(counts)[:-1]]
    me = bpy.data.meshes.new("dec")
    me.from_pydata(pts.tolist(), [], [idx[a:a + n].tolist() for a, n in zip(starts, counts)])
    me.uv_layers.new(name="st").data.foreach_set("uv", uv.ravel())
    me.polygons.foreach_set("material_index", mat_id)
    for _ in range(max(1, len(subsets))):
        me.materials.append(None)
    me.polygons.foreach_set("use_smooth", np.ones(len(counts), bool))
    me.set_sharp_from_angle(angle=math.radians(40.0))
    ob = bpy.data.objects.new("dec", me)
    bpy.context.scene.collection.objects.link(ob)
    mod = ob.modifiers.new("dec", "DECIMATE")
    mod.ratio = FACE_CAP / len(counts)
    out = bpy.data.meshes.new_from_object(ob.evaluated_get(bpy.context.evaluated_depsgraph_get()))
    out.set_sharp_from_angle(angle=math.radians(40.0))
    nv, nl, nf = len(out.vertices), len(out.loops), len(out.polygons)
    co = np.zeros(nv * 3)
    out.vertices.foreach_get("co", co)
    lv = np.zeros(nl, int)
    out.loops.foreach_get("vertex_index", lv)
    ltot = np.zeros(nf, int)
    out.polygons.foreach_get("loop_total", ltot)
    mids = np.zeros(nf, int)
    out.polygons.foreach_get("material_index", mids)
    luv = np.zeros(nl * 2)
    out.uv_layers["st"].data.foreach_get("uv", luv)
    nrm = np.zeros(nl * 3)
    out.corner_normals.foreach_get("vector", nrm)
    bpy.data.objects.remove(ob)
    bpy.data.meshes.remove(me)
    bpy.data.meshes.remove(out)

    mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(co.reshape(-1, 3).astype(np.float32)))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(ltot.astype(np.int32)))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(lv.astype(np.int32)))
    mesh.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(nrm.reshape(-1, 3).astype(np.float32)))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    pv.GetAttr().Set(Vt.Vec2fArray.FromNumpy(luv.reshape(-1, 2).astype(np.float32)))
    pv.BlockIndices()
    for other in UsdGeom.PrimvarsAPI(mesh).GetPrimvars():  # colour primvars sized to the old topology (dropped)
        val = other.Get()
        if other.GetPrimvarName() != "st" and (other.GetInterpolation() != UsdGeom.Tokens.constant or (val is not None and len(val) > 1)):
            other.GetAttr().Block()
            if other.IsIndexed():
                other.GetIndicesAttr().Block()
    for k, s in enumerate(subsets):
        s.GetIndicesAttr().Set(Vt.IntArray.FromNumpy(np.nonzero(mids == k)[0].astype(np.int32)))
    mesh.GetExtentAttr().Set(UsdGeom.Boundable.ComputeExtentFromPlugins(mesh, Usd.TimeCode.Default()))


def flatten(root_dir: pathlib.Path, pkg: str) -> tuple[pathlib.Path, Gf.Range3d]:
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
    drop, missing, done = [], [], {}

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
                    if p not in done:
                        done[p] = _texture(p, dst / "Textures")
                    a.default = Sdf.AssetPath(f"./Textures/{done[p]}")
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
    assert UsdGeom.GetStageMetersPerUnit(st) == 1.0 and UsdGeom.GetStageUpAxis(st) == "Z", pkg
    bad = [str(p.GetPath()) for p in st.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) or p.HasAPI(UsdPhysics.CollisionAPI)]
    assert not bad, f"{pkg}: physics left on {bad[:3]}"
    for prim in st.Traverse():
        if prim.IsA(UsdGeom.Mesh) and len(UsdGeom.Mesh(prim).GetFaceVertexCountsAttr().Get()) > FACE_CAP:
            _decimate(UsdGeom.Mesh(prim))
    # Re-flatten so the decimated overrides become the only mesh data, drop the (now unreferenced) instancing
    # prototypes holding the original meshes, and write a fresh file (Save() would append to the crate, not compact).
    flat = st.Flatten()
    for spec in list(flat.rootPrims):
        if spec.name.startswith("Flattened_Prototype"):
            del flat.rootPrims[spec.name]
    flat.documentation = layer.documentation
    tmp = out.with_name(out.stem + ".tmp.usdc")
    flat.Export(str(tmp))
    tmp.replace(out)
    st = Usd.Stage.Open(str(out))
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(
        st.GetDefaultPrim()).ComputeAlignedRange()
    return out, box


def main(root_dir: pathlib.Path) -> None:
    lines = ['#usda 1.0', '(', '    defaultPrim = "bench_props"', '    metersPerUnit = 1', '    upAxis = "Z"', ')', '',
             'def Xform "bench_props"', '{']
    report = {}
    for pkg, (x, y, yaw) in PROPS.items():
        _, box = flatten(root_dir, pkg)
        lo, hi = box.GetMin(), box.GetMax()
        cx, cy = 0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1])
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        tx = x - MAT_POS[0] - (c * cx - s * cy)
        ty = y - MAT_POS[1] - (s * cx + c * cy)
        tz = -lo[2] + 0.0005  # rest on the table top (0.5 mm lift against z-fighting)
        lines += [f'    def Xform "{pkg}" (', f'        prepend references = @../props/{pkg}/{pkg}.usdc@', '    )', '    {',
                  f'        double3 xformOp:translate = ({tx:.5f}, {ty:.5f}, {tz:.5f})',
                  f'        float xformOp:rotateZ = {yaw:.1f}',
                  '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ"]', '    }']
        size = [round(1000 * (hi[i] - lo[i]), 1) for i in range(3)]
        half = 0.5 * max(size[0], size[1]) / 1000
        report[pkg] = {"station_xy": [x, y], "yaw_deg": yaw, "size_mm": size,
                       "footprint_x": [round(x - half, 3), round(x + half, 3)], "footprint_y": [round(y - half, 3), round(y + half, 3)],
                       "bytes": sum(f.stat().st_size for f in (OUT / pkg).rglob("*") if f.is_file())}
    lines.append('}')
    (PKG / "assets" / "visual" / "bench_props.usda").write_text("\n".join(lines) + "\n")
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main(pathlib.Path(sys.argv[sys.argv.index("--") + 1]))
