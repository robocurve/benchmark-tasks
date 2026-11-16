"""Write the bench-pad visual: assets/visual/bench_pad.usda + textures/bench_pad.png (deterministic, no pxr needed).

A 110 x 110 x 2 mm square of absorbent bench protector (quilted white absorbent paper top over a blue polyethylene
backing), the clean surface the loose snap caps wait on. Part frame: centred in x/y, bottom at z = 0.
Run from the package folder: python3 assets_build/make_pad.py
"""

from __future__ import annotations

import pathlib

import numpy as np
from PIL import Image

PKG = pathlib.Path(__file__).resolve().parent.parent
VIS = PKG / "assets" / "visual"
HX, HY, H = 0.055, 0.055, 0.002
BACK_H = 0.0004  # blue PE backing film at the bottom of the edge
TEX_PX = 512
QUILT_MM = 9.0  # diamond quilting pitch of the absorbent layer


def make_texture(path: pathlib.Path) -> None:
    n = TEX_PX
    mm = (np.arange(n) + 0.5) / n * (2 * HX * 1000.0)
    u, v = np.meshgrid(mm, mm)
    a = np.abs(((u + v) / QUILT_MM) % 1.0 - 0.5)
    b = np.abs(((u - v) / QUILT_MM) % 1.0 - 0.5)
    line = np.exp(-((0.5 - np.maximum(a, b)) / 0.035) ** 2)  # embossed quilt seams
    rng = np.random.default_rng(20261115)
    fibre = rng.normal(0.0, 1.0, (n, n))
    k = np.ones(3) / 3.0
    fibre = np.apply_along_axis(lambda r: np.convolve(r, k, mode="same"), 1, fibre)
    fibre = np.apply_along_axis(lambda c: np.convolve(c, k, mode="same"), 0, fibre)
    g = 0.925 - 0.09 * line + 0.012 * fibre
    rgb = np.stack([g * 0.995, g * 0.995, g * 0.985], axis=-1)
    Image.fromarray((np.clip(rgb, 0, 1) * 255).round().astype(np.uint8), "RGB").save(path, optimize=True)


def _box(x0, x1, y0, y1, z0, z1, top_uv: bool):
    p = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    normals = [(0, 0, -1), (0, 0, 1), (0, -1, 0), (1, 0, 0), (0, 1, 0), (-1, 0, 0)]
    pts, idx, nrm, st = [], [], [], []
    for f, nv in zip(faces, normals):
        for i in f:
            idx.append(len(pts))
            pts.append(p[i])
            nrm.append(nv)
            if top_uv and nv == (0, 0, 1):
                st.append(((p[i][0] - x0) / (x1 - x0), (p[i][1] - y0) / (y1 - y0)))
            else:
                st.append((0.02, 0.02))
    return pts, idx, nrm, st


def _fmt(vals):
    return ", ".join("(" + ", ".join(f"{c:.6g}" for c in v) + ")" for v in vals)


def _mesh(name, mat, box, top_uv):
    pts, idx, nrm, st = _box(*box, top_uv=top_uv)
    lo = [min(p[i] for p in pts) for i in range(3)]
    hi = [max(p[i] for p in pts) for i in range(3)]
    return f'''
    def Mesh "{name}" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{
        uniform bool doubleSided = 1
        float3[] extent = [({lo[0]:.6g}, {lo[1]:.6g}, {lo[2]:.6g}), ({hi[0]:.6g}, {hi[1]:.6g}, {hi[2]:.6g})]
        int[] faceVertexCounts = [{", ".join("4" for _ in range(6))}]
        int[] faceVertexIndices = [{", ".join(str(i) for i in idx)}]
        rel material:binding = </Root/Looks/{mat}>
        normal3f[] normals = [{_fmt(nrm)}] (
            interpolation = "faceVarying"
        )
        point3f[] points = [{_fmt(pts)}]
        texCoord2f[] primvars:st = [{_fmt(st)}] (
            interpolation = "faceVarying"
        )
        uniform token subdivisionScheme = "none"
    }}
'''


def _material(name, rough, color=None, texture=None):
    pbr_col = (f"asset inputs:diffuse_texture = @./textures/{texture}@" if texture
               else f"color3f inputs:diffuse_color_constant = ({color[0]}, {color[1]}, {color[2]})")
    prev_col = (f"color3f inputs:diffuseColor.connect = </Root/Looks/{name}/Tex.outputs:rgb>" if texture
                else f"color3f inputs:diffuseColor = ({color[0]}, {color[1]}, {color[2]})")
    tex = f'''
            def Shader "st"
            {{
                uniform token info:id = "UsdPrimvarReader_float2"
                token inputs:varname = "st"
                float2 outputs:result
            }}

            def Shader "Tex"
            {{
                uniform token info:id = "UsdUVTexture"
                asset inputs:file = @./textures/{texture}@
                token inputs:sourceColorSpace = "sRGB"
                float2 inputs:st.connect = </Root/Looks/{name}/st.outputs:result>
                float3 outputs:rgb
            }}
''' if texture else ""
    return f'''
        def Material "{name}"
        {{
            token outputs:mdl:surface.connect = </Root/Looks/{name}/PBR.outputs:out>
            token outputs:surface.connect = </Root/Looks/{name}/Preview.outputs:surface>

            def Shader "PBR"
            {{
                uniform token info:implementationSource = "sourceAsset"
                uniform asset info:mdl:sourceAsset = @OmniPBR.mdl@
                uniform token info:mdl:sourceAsset:subIdentifier = "OmniPBR"
                {pbr_col}
                float inputs:metallic_constant = 0
                float inputs:reflection_roughness_constant = {rough}
                token outputs:out
            }}

            def Shader "Preview"
            {{
                uniform token info:id = "UsdPreviewSurface"
                {prev_col}
                float inputs:ior = 1.5
                float inputs:metallic = 0
                float inputs:roughness = {rough}
                token outputs:surface
            }}
{tex}        }}
'''


def main() -> None:
    (VIS / "textures").mkdir(parents=True, exist_ok=True)
    make_texture(VIS / "textures" / "bench_pad.png")
    looks = _material("Absorbent", 0.92, texture="bench_pad.png") + _material("PE_backing", 0.5, color=(0.22, 0.45, 0.78))
    body = (_mesh("absorbent", "Absorbent", (-HX, HX, -HY, HY, BACK_H, H), True)
            + _mesh("backing", "PE_backing", (-HX, HX, -HY, HY, 0.0, BACK_H), False))
    usda = f'''#usda 1.0
(
    defaultPrim = "Root"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Root"
{{
    def Scope "Looks"
    {{{looks}    }}
{body}}}
'''
    (VIS / "bench_pad.usda").write_text(usda)


if __name__ == "__main__":
    main()
