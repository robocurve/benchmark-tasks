#!/usr/bin/env python3
"""Fetch pinned CC-BY NVIDIA visual sources; optionally author visual-only USD.

Network fetch uses Python's standard library. ``--prepare`` additionally needs
OpenUSD and Pillow and should run in the documented compute-node environment.
All authored references remain relative to this task directory.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
from pathlib import Path
import urllib.request

REVISION = "653d91e182011f357cf1644013cc94ddf194d349"
REPOSITORY = "nvidia/simready-assets"
PACKAGES = ("handTool_ratchet_b01", "handTool_socket_1_2in_b01")
ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / "assets"
VENDOR = ASSETS / "vendor"
BASE = f"https://huggingface.co/datasets/{REPOSITORY}/resolve/{REVISION}/"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fetch():
    rows = []
    for package in PACKAGES:
        api = (f"https://huggingface.co/api/datasets/{REPOSITORY}/tree/"
               f"{REVISION}/{package}?recursive=true&limit=1000")
        with urllib.request.urlopen(api, timeout=45) as response:
            listing = json.load(response)
        for item in listing:
            path = item["path"]
            # Preview thumbnails and texture-authoring duplicates are not USD
            # dependencies. All source MDL files and backend payloads remain.
            if item["type"] != "file" or "/.thumbs/" in path:
                continue
            if "/source_assets/" in path and path.endswith(".png"):
                continue
            rows.append((path, int(item["size"])))

    def download(row):
        relative, size = row
        target = VENDOR / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists() or target.stat().st_size != size:
            temporary = target.with_suffix(target.suffix + ".part")
            with urllib.request.urlopen(BASE + relative, timeout=90) as response:
                with temporary.open("wb") as output:
                    while chunk := response.read(1024 * 1024):
                        output.write(chunk)
            if temporary.stat().st_size != size:
                raise RuntimeError(f"Size mismatch: {relative}")
            temporary.replace(target)
        return {"path": relative, "url": BASE + relative,
                "bytes": size, "sha256": sha256(target)}

    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as executor:
        records = list(executor.map(download, rows))
    provenance = {
        "repository": REPOSITORY, "revision": REVISION,
        "attribution": "NVIDIA SimReady assets, NVIDIA, CC-BY-4.0",
        "license": "CC-BY-4.0",
        "license_files": [f"vendor/{p}/.metadata/LICENSE.txt" for p in PACKAGES],
        "omitted": "Preview thumbnails and unreferenced source_assets PNG duplicates",
        "sources": sorted(records, key=lambda item: item["path"]),
    }
    (ASSETS / "provenance_sources.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(json.dumps({"downloaded_files": len(records), "bytes": sum(x["bytes"] for x in records)}))


def prepare():
    """Bake only rendered meshes, retain their UVs, and author portable PBR looks.

    The ratchet output pivot is the source drive object's measured origin.
    Canonical axes put its handle along -X and its square drive along -Z.
    The socket origin is its square-entry plane, with its hex opening along -Z.
    These are visualization assets: simulation schemas are never copied.
    """
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade
    from PIL import Image
    report = {}
    modifications = []
    for package in PACKAGES:
        source = VENDOR / package / "simready_usd" / f"sm_{package}_01.usd"
        stage = Usd.Stage.Open(str(source))
        if not stage:
            raise RuntimeError(f"Cannot open {source}")
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
        cache = UsdGeom.XformCache()
        data = {"meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage), "prims": []}
        for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
            if prim.GetTypeName() in ("Mesh", "Xform"):
                rng = bounds.ComputeWorldBound(prim).ComputeAlignedRange()
                data["prims"].append({"path": str(prim.GetPath()), "type": prim.GetTypeName(),
                    "bounds_min": list(rng.GetMin()), "bounds_max": list(rng.GetMax()),
                    "transform": [list(row) for row in cache.GetLocalToWorldTransform(prim)],
                    "schemas": list(prim.GetAppliedSchemas()),
                    "purpose": str(UsdGeom.Imageable(prim).GetPurposeAttr().Get()),
                    "point_count": len(UsdGeom.Mesh(prim).GetPointsAttr().Get() or []) if prim.IsA(UsdGeom.Mesh) else None})
        report[package] = data
        ratchet = "ratchet" in package
        output_name = "ratchet_visual.usda" if ratchet else "socket_visual.usda"
        root_name = "/RatchetVisual" if ratchet else "/SocketVisual"
        output = Usd.Stage.CreateNew(str(ASSETS / output_name))
        UsdGeom.SetStageMetersPerUnit(output, 1.0)
        UsdGeom.SetStageUpAxis(output, UsdGeom.Tokens.z)
        root = UsdGeom.Xform.Define(output, root_name)
        output.SetDefaultPrim(root.GetPrim())
        root.GetPrim().SetDocumentation(
            "Visual-only derivative of NVIDIA SimReady, CC-BY-4.0. "
            "See provenance_sources.json and provenance_visuals.json.")
        if ratchet:
            pivot = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(
                "/RootNode/Geometry/drive_01_obj_00")).ExtractTranslation()
            groups = {name: UsdGeom.Xform.Define(output, root_name + "/" + name)
                      for name in ("Handle", "Drive")}
        else:
            rng = bounds.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
            pivot = Gf.Vec3d(0, 0, rng.GetMin()[2])
            groups = {"Geometry": UsdGeom.Xform.Define(output, root_name + "/Geometry")}
        canonical = Gf.Matrix4d().SetTranslate(-pivot) * Gf.Matrix4d().SetRotate(
            Gf.Rotation(Gf.Vec3d(1, 0, 0), 180))

        # Source maps are a shared UV atlas across all meshes of each tool.
        # Leave source files unchanged and record separate reduced derivatives.
        textures = {}
        texture_records = []
        derivative_root = VENDOR / "derived_textures" / package
        derivative_root.mkdir(parents=True, exist_ok=True)
        for suffix in ("alb", "nor", "orm"):
            original = VENDOR / package / "simready_usd" / "Textures" / f"t_handTool_socket_a01_{suffix}.png"
            reduced = derivative_root / original.name
            with Image.open(original) as im:
                original_size = list(im.size)
                im.thumbnail((1024, 1024), Image.Resampling.LANCZOS)
                im.save(reduced, optimize=True)
                size = list(im.size)
            textures[suffix] = reduced.relative_to(ASSETS).as_posix()
            texture_records.append({"source": original.relative_to(ASSETS).as_posix(),
                "source_sha256": sha256(original), "original_dimensions": original_size,
                "derivative": textures[suffix], "derivative_sha256": sha256(reduced),
                "dimensions": size, "operation": "Pillow LANCZOS resize, max dimension 1024; PNG"})

        mat = UsdShade.Material.Define(output, root_name + "/Looks/SourceAtlas")
        shader = UsdShade.Shader.Define(output, root_name + "/Looks/SourceAtlas/Surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.8)
        reader = UsdShade.Shader.Define(output, root_name + "/Looks/SourceAtlas/UV")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        for suffix in ("alb", "nor", "orm"):
            tex = UsdShade.Shader.Define(output, root_name + "/Looks/SourceAtlas/" + suffix)
            tex.CreateIdAttr("UsdUVTexture")
            tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(textures[suffix])
            tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB" if suffix == "alb" else "raw")
            tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
            tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            for channel in ("r", "g", "b"):
                tex.CreateOutput(channel, Sdf.ValueTypeNames.Float)
            if suffix == "alb":
                shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
            elif suffix == "orm":
                shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).ConnectToSource(tex.ConnectableAPI(), "g")
                shader.GetInput("metallic").ConnectToSource(tex.ConnectableAPI(), "b")
                shader.CreateInput("occlusion", Sdf.ValueTypeNames.Float).ConnectToSource(tex.ConnectableAPI(), "r")
            else:
                tex.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2, 2, 2, 1))
                tex.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1, -1, -1, 0))
                shader.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        # Each split root must be independently referenceable by the runtime.
        # Keep a complete material subtree beneath it, rather than a binding
        # target outside a subprim reference (which USD would discard).
        materials = {}
        old_prefix = mat.GetPath()
        for group, group_prim in groups.items():
            new_prefix = group_prim.GetPath().AppendPath("Looks/SourceAtlas")
            UsdGeom.Scope.Define(output, group_prim.GetPath().AppendChild("Looks"))
            Sdf.CopySpec(output.GetRootLayer(), old_prefix, output.GetRootLayer(), new_prefix)
            for child in Usd.PrimRange(output.GetPrimAtPath(new_prefix)):
                for attr in child.GetAttributes():
                    connections = attr.GetConnections()
                    if connections:
                        attr.SetConnections([p.ReplacePrefix(old_prefix, new_prefix) for p in connections])
            materials[group] = UsdShade.Material(output.GetPrimAtPath(new_prefix))
        output.RemovePrim(root_name + "/Looks")
        copied = []
        for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
            path = str(prim.GetPath())
            if not prim.IsA(UsdGeom.Mesh) or not path.startswith("/RootNode/Geometry/"):
                continue
            purpose = UsdGeom.Imageable(prim).ComputePurpose()
            if purpose not in ("default", "render"):
                continue
            group = ("Drive" if "/drive_" in path else "Handle") if ratchet else "Geometry"
            mesh_path = groups[group].GetPath().AppendChild("mesh_" + str(len(copied)))
            mesh = UsdGeom.Mesh.Define(output, mesh_path)
            for attribute in prim.GetAttributes():
                name = attribute.GetName()
                if name.startswith(("physics:", "physx", "xformOp", "semantics", "material:")):
                    continue
                value = attribute.Get()
                if value is None:
                    continue
                target = mesh.GetPrim().CreateAttribute(name, attribute.GetTypeName(),
                    attribute.IsCustom(), attribute.GetVariability())
                target.Set(value)
                for key, value in attribute.GetAllAuthoredMetadata().items():
                    if key not in ("typeName", "custom", "variability"):
                        target.SetMetadata(key, value)
            mesh.AddTransformOp().Set(cache.GetLocalToWorldTransform(prim) * canonical)
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials[group])
            copied.append({"source_prim": path, "output_prim": str(mesh_path),
                "points": len(mesh.GetPointsAttr().Get()), "faces": len(mesh.GetFaceVertexCountsAttr().Get())})
        if not copied:
            raise RuntimeError(f"No render meshes in {package}")
        output.GetRootLayer().Save()
        modifications.append({"source_package": package, "output": output_name,
            "scale": 1.0, "source_pivot_m": list(pivot), "canonical_rotation": "180 degrees about X",
            "canonical_axes": "handle -X; output/hex opening -Z; metres",
            "source_mass_kg": sum(float(a.Get()) for prim in stage.Traverse()
                for a in prim.GetAttributes() if a.GetName() == "pxr:usd:physics_mass"),
            "meshes": copied, "textures": texture_records,
            "modifications": "Retained render mesh points, normals, UVs and topology; baked world transforms about output origin; omitted all collision, rigid-body, joint and guide schemas; welded switch/button visually to handle; converted atlas materials to portable UsdPreviewSurface using 1K derivatives."})
    (ASSETS / "provenance_inspection.json").write_text(json.dumps(report, indent=2) + "\n")
    (ASSETS / "provenance_visuals.json").write_text(json.dumps({"license": "CC-BY-4.0",
        "attribution": "NVIDIA SimReady assets, NVIDIA; modified for confined ratchet fastening.",
        "revision": REVISION, "derivatives": modifications}, indent=2) + "\n")
    print(json.dumps(modifications, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--skip-fetch", action="store_true")
    args = parser.parse_args()
    if not args.skip_fetch:
        fetch()
    if args.prepare:
        prepare()
