"""The canister's printed label: a thin, non-colliding band wrapped round the body, textured with isopro.png
(make_png.py draws it) through a UsdPreviewSurface material, so the text is rendered in every camera."""

import math
import pathlib

PNG = pathlib.Path(__file__).resolve().parent / "isopro.png"


def wrap(radius, z0, z1, centre, arc, segments=24):
    """Points, quad indices, normals and texture coordinates of a band on a z-axis cylinder of ``radius`` (part
    frame, m) from height ``z0`` to ``z1``, centred on azimuth ``centre`` and spanning ``arc`` radians. The text's
    left edge as read from outside is at azimuth ``centre - arc/2``."""
    points, normals, st, counts, index = [], [], [], [], []
    for k in range(segments + 1):
        a = centre - arc / 2 + arc * k / segments
        c, s = math.cos(a), math.sin(a)
        for z, v in ((z0, 0.0), (z1, 1.0)):
            points.append((radius * c, radius * s, z))
            normals.append((c, s, 0.0))
            st.append((k / segments, v))
    for k in range(segments):
        counts.append(4)
        index += [2 * k, 2 * k + 2, 2 * k + 3, 2 * k + 1]
    return points, counts, index, normals, st


def attach(stage, root_path, name, band):
    """Put the label on the part at ``root_path`` as ``band`` (the result of ``wrap``)."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    points, counts, index, normals, uv = band
    path = f"{root_path}/{name}"
    if stage.GetPrimAtPath(path).IsValid():
        return
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*(float(v) for v in c)) for c in points])
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(index)
    mesh.CreateNormalsAttr([Gf.Vec3f(*(float(v) for v in n)) for n in normals])
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(False)
    st = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                           UsdGeom.Tokens.vertex)
    st.Set([Gf.Vec2f(*(float(v) for v in t)) for t in uv])

    mpath = f"{root_path}/{name}_look"
    material = UsdShade.Material.Define(stage, mpath)
    surface = UsdShade.Shader.Define(stage, f"{mpath}/surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.55)
    surface.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    reader = UsdShade.Shader.Define(stage, f"{mpath}/uv")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    texture = UsdShade.Shader.Define(stage, f"{mpath}/print")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(PNG)))
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(), "rgb")
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
