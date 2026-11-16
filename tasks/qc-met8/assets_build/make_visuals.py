"""Author the render-only USD models of qc-met8 (run: python3 assets_build/make_visuals.py <simready samples dir>).

Every visible object of the task is drawn by one of these models; the task's boxes and cylinders stay as invisible
colliders (labkit ``hide_shapes``). Geometry follows the real products' dimensions used by qc_met8.py (part frames,
metres, Z up). Materials are UsdPreviewSurface PBR sets built from NVIDIA SimReady tiling textures (CC-BY-4.0,
huggingface.co/datasets/nvidia/simready-assets @ 7879e0e5), downscaled to 1024 px; printed labels and the LCD
screens are drawn here with Pillow. Output: assets/visual/*.usda + assets/visual/textures/*.png.
"""

import math
import pathlib
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = pathlib.Path(__file__).resolve().parent.parent
OUT = HERE / "assets" / "visual"
TEX = OUT / "textures"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_R = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

# Task constants (same numbers as qc_met8.py)
SHAFT_R, SHAFT_L, SHAFT_AXIS_Z = 0.021, 0.150, 0.035
VBLOCK_DY = 0.061
BODY_W, BODY_L, BODY_H = 0.023, 0.115, 0.026
REAR_Y, SADDLE_Y, STYLUS_Y = -0.070, 0.050, 0.088
V_CHEEK_DIST, V_AX_LOCAL_Z = 0.023, -0.036
THUMB_R, THUMB_H, THUMB_Z = 0.013, 0.014, 0.026
REAR_PAD_Z = -0.0275
CZ = 0.0521
CONSOLE = (0.160, 0.0658, 0.0521)
START_X = -0.045
LCD_C, LCD_W, LCD_H = (0.026, 0.0), 0.056, 0.046

# -- SimReady PBR tiles (source package, texture stem) ------------------------------------------------------------------
SIMREADY_TILES = {
    "chrome": ("syringe_a01", "t_metal_chrome_a01_tile"),
    "glossy_a": ("tray_sterilization_instrument_c02", "t_metal_glossy_a01_tile"),
    "glossy_b": ("tray_sterilization_instrument_c02", "t_metal_glossy_b01_tile"),
    "rubber": ("syringe_a01", "t_rubber_glossy_black_a01_tile"),
    "plastic_black": ("tray_sterilization_instrument_c02", "t_plastic_glossy_black_a01_tile"),
    "plastic_blue": ("tray_sterilization_instrument_c02", "t_plastic_blue_a01_tile"),
}


def copy_tiles(src_root: pathlib.Path) -> None:
    for pkg, stem in SIMREADY_TILES.values():
        for ch in ("alb", "nor", "orm"):
            dst = TEX / f"{stem}_{ch}.png"
            if dst.exists():
                continue
            im = Image.open(src_root / pkg / "simready_usd" / "Textures" / f"{stem}_{ch}.png")
            im = im.convert("RGB").resize((1024, 1024), Image.Resampling.LANCZOS)
            im.save(dst, optimize=True)


# -- materials --------------------------------------------------------------------------------------------------------
# tile: metres per texture repeat for box-projected UVs. tint scales the albedo map (or is the colour without one).
# Steel and brass use metallic 0.5-0.6 instead of the ORM map (metallic 1): the station has no environment map, so
# fully metallic parts rendered dark slate in the 1280 top camera (v2 GPU run); half-metallic keeps a lit diffuse part.
MATERIALS = {
    "ground_steel": dict(tex="chrome", tint=(1.0, 1.0, 1.0), metallic=0.5, roughness=0.3, tile=0.04),
    "steel_end": dict(tex="glossy_b", tint=(0.62, 0.63, 0.66), metallic=0.5, roughness=0.4, tile=0.03),
    "vblock_steel": dict(tex="glossy_a", tint=(0.85, 0.87, 0.90), metallic=0.5, roughness=0.35, tile=0.05),
    "anvil_steel": dict(tex="glossy_a", tint=(0.90, 0.92, 0.95), metallic=0.5, roughness=0.35, tile=0.05),
    "dark_steel": dict(tex="glossy_b", tint=(0.30, 0.31, 0.33), tile=0.03),
    "brass": dict(tex="chrome", tint=(1.0, 0.78, 0.36), metallic=0.6, roughness=0.3, tile=0.02),
    "rubber": dict(tex="rubber", tint=(0.55, 0.56, 0.60), tile=0.04),
    "rubber_grey": dict(tex="rubber", tint=(1.4, 1.45, 1.55), tile=0.04),
    "drive_paint": dict(tex="plastic_black", tint=(0.27, 0.29, 0.32), metallic=0.0, roughness=0.42, tile=0.05),
    "blue_paint": dict(tex="plastic_blue", tint=(0.55, 0.85, 1.05), tile=0.05),
    "abs_charcoal": dict(tex="plastic_black", tint=(0.20, 0.21, 0.23), metallic=0.0, roughness=0.55, tile=0.06),
    "abs_grey_key": dict(color=(0.42, 0.44, 0.47), metallic=0.0, roughness=0.5),
    "key_green": dict(color=(0.10, 0.62, 0.26), metallic=0.0, roughness=0.35),
    "orange": dict(color=(0.95, 0.38, 0.10), metallic=0.0, roughness=0.4),
}


def material_usda(name: str, root: str) -> str:
    m = MATERIALS.get(name)
    base = f"{root}/Looks/{name}"
    out = [f'        def Material "{name}"', "        {",
           f"            token outputs:surface.connect = <{base}/pbr.outputs:surface>",
           '            def Shader "pbr"', "            {", '                uniform token info:id = "UsdPreviewSurface"']
    tex_nodes = []
    if m is None:  # a printed decal or screen: name is a texture file, drawn and lit
        out += [f"                color3f inputs:diffuseColor.connect = <{base}/img.outputs:rgb>",
                f"                color3f inputs:emissiveColor.connect = <{base}/glow.outputs:rgb>",
                "                float inputs:metallic = 0", "                float inputs:roughness = 0.25"]
        tex_nodes.append(("img", f"./textures/{name}.png", "sRGB", (1, 1, 1, 1), (0, 0, 0, 0)))
        glow = 0.6 if name.startswith("lcd_") else 0.0
        tex_nodes.append(("glow", f"./textures/{name}.png", "sRGB", (glow, glow, glow, 1), (0, 0, 0, 0)))
    elif "tex" in m:
        stem = SIMREADY_TILES[m["tex"]][1]
        t = m["tint"]
        out.append(f"                color3f inputs:diffuseColor.connect = <{base}/alb.outputs:rgb>")
        tex_nodes.append(("alb", f"./textures/{stem}_alb.png", "sRGB", (*t, 1), (0, 0, 0, 0)))
        out.append(f"                normal3f inputs:normal.connect = <{base}/nor.outputs:rgb>")
        tex_nodes.append(("nor", f"./textures/{stem}_nor.png", "raw", (2, 2, 2, 1), (-1, -1, -1, 0)))
        if "metallic" in m:
            out.append(f"                float inputs:metallic = {m['metallic']}")
            out.append(f"                float inputs:roughness = {m['roughness']}")
        else:
            out.append(f"                float inputs:roughness.connect = <{base}/orm.outputs:g>")
            out.append(f"                float inputs:metallic.connect = <{base}/orm.outputs:b>")
            out.append(f"                float inputs:occlusion.connect = <{base}/orm.outputs:r>")
            tex_nodes.append(("orm", f"./textures/{stem}_orm.png", "raw", (1, 1, 1, 1), (0, 0, 0, 0)))
    else:
        out += [f"                color3f inputs:diffuseColor = {tuple(m['color'])}",
                f"                float inputs:metallic = {m['metallic']}",
                f"                float inputs:roughness = {m['roughness']}"]
    out += ["                token outputs:surface", "            }"]
    if tex_nodes:
        out += ['            def Shader "st"', "            {", '                uniform token info:id = "UsdPrimvarReader_float2"',
                '                string inputs:varname = "st"', "                float2 outputs:result", "            }"]
    for node, f, cs, scale, bias in tex_nodes:
        outs = "                float3 outputs:rgb\n                float outputs:r\n                float outputs:g\n                float outputs:b"
        out += [f'            def Shader "{node}"', "            {", '                uniform token info:id = "UsdUVTexture"',
                f"                asset inputs:file = @{f}@", f'                token inputs:sourceColorSpace = "{cs}"',
                f"                float2 inputs:st.connect = <{base}/st.outputs:result>",
                '                token inputs:wrapS = "repeat"', '                token inputs:wrapT = "repeat"',
                f"                float4 inputs:scale = {tuple(float(v) for v in scale)}",
                f"                float4 inputs:bias = {tuple(float(v) for v in bias)}", outs, "            }"]
    out.append("        }")
    return "\n".join(out)


# -- mesh building ----------------------------------------------------------------------------------------------------
class Mesh:
    """Unshared face-vertex mesh: every polygon carries its own points, normals and UVs."""

    def __init__(self, uv="box", tile=0.05):
        self.P, self.N, self.UV, self.counts = [], [], [], []
        self.uv, self.tile = uv, tile

    def poly(self, pts, nrm, uvs=None):
        pts, nrm = np.asarray(pts, float), np.asarray(nrm, float)
        if len(pts) < 3:
            return
        g = np.cross(pts[1] - pts[0], pts[2] - pts[0])
        if len(pts) == 4 and np.linalg.norm(g) < 1e-14:
            g = np.cross(pts[2] - pts[1], pts[3] - pts[1])
        if np.dot(g, nrm.sum(0)) < 0:  # wind counter-clockwise seen from the outside
            pts, nrm = pts[::-1], nrm[::-1]
            uvs = None if uvs is None else uvs[::-1]
        self.P += list(pts)
        self.N += list(nrm / np.linalg.norm(nrm, axis=1, keepdims=True))
        self.UV += list(uvs) if uvs is not None else [None] * len(pts)
        self.counts.append(len(pts))

    def transform(self, rot_deg=(0, 0, 0), t=(0, 0, 0)):
        R = rot_xyz(rot_deg)
        self.P = [R @ p + np.asarray(t) for p in self.P]
        self.N = [R @ n for n in self.N]
        return self

    def add(self, other):
        self.P += other.P
        self.N += other.N
        self.UV += other.UV
        self.counts += other.counts
        return self

    def uvs(self):
        out = []
        for p, n, uv in zip(self.P, self.N, self.UV):
            if uv is not None:
                out.append(uv)
                continue
            a = int(np.argmax(np.abs(n)))
            u, v = [k for k in range(3) if k != a]
            out.append((p[u] / self.tile, p[v] / self.tile))
        return out


def rot_xyz(deg):
    x, y, z = (math.radians(d) for d in deg)
    rx = np.array([[1, 0, 0], [0, math.cos(x), -math.sin(x)], [0, math.sin(x), math.cos(x)]])
    ry = np.array([[math.cos(y), 0, math.sin(y)], [0, 1, 0], [-math.sin(y), 0, math.cos(y)]])
    rz = np.array([[math.cos(z), -math.sin(z), 0], [math.sin(z), math.cos(z), 0], [0, 0, 1]])
    return rz @ ry @ rx  # USD rotateXYZ: X first, then Y, then Z


def rbox(size, radius, n=4):
    """A box of ``size`` centred on the origin with all edges rounded to ``radius`` (smooth normals)."""
    h = np.asarray(size, float) / 2
    r = min(radius, *(h * 0.999))
    phis = np.linspace(0, math.pi / 2, n)

    def ticks(hk):
        return np.concatenate([-(hk - r) - r * np.sin(phis[::-1]), (hk - r) + r * np.sin(phis)])

    m = Mesh()
    for axis in range(3):
        for sgn in (-1, 1):
            u, v = [k for k in range(3) if k != axis]
            tu, tv = ticks(h[u]), ticks(h[v])
            grid = np.zeros((len(tu), len(tv), 2, 3))
            for i, a in enumerate(tu):
                for j, b in enumerate(tv):
                    p = np.zeros(3)
                    p[axis], p[u], p[v] = sgn * h[axis], a, b
                    inner = np.clip(p, -(h - r), h - r)
                    d = p - inner
                    nd = d / np.linalg.norm(d)
                    grid[i, j, 0] = inner + r * nd
                    grid[i, j, 1] = nd
            for i in range(len(tu) - 1):
                for j in range(len(tv) - 1):
                    q = [grid[i, j], grid[i + 1, j], grid[i + 1, j + 1], grid[i, j + 1]]
                    if np.linalg.norm(np.cross(q[1][0] - q[0][0], q[3][0] - q[0][0])) < 1e-16 and \
                       np.linalg.norm(np.cross(q[2][0] - q[1][0], q[3][0] - q[1][0])) < 1e-16:
                        continue
                    m.poly([x[0] for x in q], [x[1] for x in q])
    return m


def lathe(profile, seg=64):
    """Surface of revolution about +Z; ``profile`` is a list of (r, z) from one end to the other, each straight
    segment a ring of quads with its own (hard-edged) normal. Returns {segment index: Mesh}."""
    th = np.linspace(0, 2 * math.pi, seg + 1)
    out = {}
    for k in range(len(profile) - 1):
        (r0, z0), (r1, z1) = profile[k], profile[k + 1]
        dr, dz = r1 - r0, z1 - z0
        nr, nz = dz, -dr
        L = math.hypot(nr, nz) or 1.0
        nr, nz = nr / L, nz / L
        m = Mesh()
        for i in range(seg):
            a, b = th[i], th[i + 1]
            pts = [(r0 * math.cos(a), r0 * math.sin(a), z0), (r0 * math.cos(b), r0 * math.sin(b), z0),
                   (r1 * math.cos(b), r1 * math.sin(b), z1), (r1 * math.cos(a), r1 * math.sin(a), z1)]
            nrm = [(nr * math.cos(a), nr * math.sin(a), nz), (nr * math.cos(b), nr * math.sin(b), nz),
                   (nr * math.cos(b), nr * math.sin(b), nz), (nr * math.cos(a), nr * math.sin(a), nz)]
            if r0 < 1e-9:
                pts, nrm = pts[1:], nrm[1:]
            elif r1 < 1e-9:
                pts, nrm = pts[:3], nrm[:3]
            m.poly(pts, nrm)
        out[k] = m
    return out


def knurl_wheel(R, H, ridges=48, depth=0.0005, chamfer=0.0008, seg_per=4):
    """Straight-knurled thumbwheel about +Z, centred on the origin: ridged side + chamfered flat faces."""
    n = ridges * seg_per
    th = np.linspace(0, 2 * math.pi, n + 1)
    rad = R - depth * (0.5 - 0.5 * np.cos(ridges * th))
    drad = -depth * 0.5 * ridges * np.sin(ridges * th)
    z0, z1 = -H / 2 + chamfer, H / 2 - chamfer
    side, faces = Mesh(), Mesh()
    for i in range(n):
        pts, nrm = [], []
        for idx, z in ((i, z0), (i + 1, z0), (i + 1, z1), (i, z1)):
            a, r, dr = th[idx], rad[idx], drad[idx]
            tx, ty = dr * math.cos(a) - r * math.sin(a), dr * math.sin(a) + r * math.cos(a)
            pts.append((r * math.cos(a), r * math.sin(a), z))
            nrm.append((ty, -tx, 0.0))
        side.poly(pts, nrm)
    for sgn in (-1, 1):
        zc, zf = sgn * (H / 2 - chamfer), sgn * H / 2
        rin = R - depth - chamfer
        for i in range(n):
            a, b = th[i], th[i + 1]
            pts = [(rad[i] * math.cos(a), rad[i] * math.sin(a), zc), (rad[i + 1] * math.cos(b), rad[i + 1] * math.sin(b), zc),
                   (rin * math.cos(b), rin * math.sin(b), zf), (rin * math.cos(a), rin * math.sin(a), zf)]
            c = 0.7071
            nrm = [(c * math.cos(a), c * math.sin(a), sgn * c), (c * math.cos(b), c * math.sin(b), sgn * c),
                   (c * math.cos(b), c * math.sin(b), sgn * c), (c * math.cos(a), c * math.sin(a), sgn * c)]
            faces.poly(pts, nrm)
            faces.poly([(0, 0, zf), (rin * math.cos(a), rin * math.sin(a), zf), (rin * math.cos(b), rin * math.sin(b), zf)],
                       [(0, 0, sgn)] * 3)
    return side, faces


def earclip(poly):
    """Triangulate a simple counter-clockwise 2D polygon."""
    idx = list(range(len(poly)))
    tris = []

    def inside(p, a, b, c):
        def s(p1, p2, p3):
            return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])
        d1, d2, d3 = s(p, a, b), s(p, b, c), s(p, c, a)
        return not ((d1 < 0 or d2 < 0 or d3 < 0) and (d1 > 0 or d2 > 0 or d3 > 0))

    guard = 0
    while len(idx) > 3 and guard < 10000:
        guard += 1
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) <= 0:
                continue
            if any(inside(poly[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
    tris.append(tuple(idx))
    return tris


def extrude_xz(poly, y0, y1):
    """A prism: counter-clockwise polygon in the XZ plane, extruded from y0 to y1."""
    m = Mesh()
    for t in earclip(poly):
        for y, ny in ((y0, -1.0), (y1, 1.0)):
            m.poly([(poly[i][0], y, poly[i][1]) for i in t], [(0, ny, 0)] * 3)
    for k in range(len(poly)):
        (x0, z0), (x1, z1) = poly[k], poly[(k + 1) % len(poly)]
        n = np.array([z1 - z0, 0.0, -(x1 - x0)])
        n /= np.linalg.norm(n)
        m.poly([(x0, y0, z0), (x1, y0, z1), (x1, y1, z1), (x0, y1, z0)], [n] * 4)
    return m


def decal(w_y, h_x, center, z):
    """Flat label in the XY plane facing up, printed so it reads upright in the top camera (image right = world -Y,
    image up = world +X): texture u runs along -Y over ``w_y``, v along +X over ``h_x``."""
    cx, cy = center
    m = Mesh(uv="quad")
    pts = [(cx - h_x / 2, cy + w_y / 2, z), (cx - h_x / 2, cy - w_y / 2, z),
           (cx + h_x / 2, cy - w_y / 2, z), (cx + h_x / 2, cy + w_y / 2, z)]
    m.poly(pts, [(0, 0, 1)] * 4, [(0, 0), (1, 0), (1, 1), (0, 1)])
    return m


def at(m, t=(0, 0, 0), rot=(0, 0, 0)):
    return m.transform(rot, t)


# -- writing ----------------------------------------------------------------------------------------------------------
def write_model(name, parts):
    """parts: list of (material name, Mesh)."""
    root = "/root"
    mats = sorted({mat for mat, _ in parts})
    lines = ["#usda 1.0", "(", '    defaultPrim = "root"', "    metersPerUnit = 1", '    upAxis = "Z"', ")", "",
             'def Xform "root"', "{", '    def Scope "Looks"', "    {"]
    lines += [material_usda(m, root) for m in mats]
    lines.append("    }")
    merged = {}
    for mat, mesh in parts:
        merged.setdefault(mat, Mesh(uv=mesh.uv, tile=MATERIALS.get(mat, {}).get("tile", 0.05))).add(mesh)
    for k, (mat, mesh) in enumerate(merged.items()):
        mesh.tile = MATERIALS.get(mat, {}).get("tile", 0.05)
        P = np.array(mesh.P)
        N = np.array(mesh.N)
        UV = mesh.uvs()
        f3 = lambda a: "[" + ", ".join(f"({x:.6g}, {y:.6g}, {z:.6g})" for x, y, z in a) + "]"  # noqa: E731
        lines += [f'    def Mesh "m{k}_{mat}" (', '        prepend apiSchemas = ["MaterialBindingAPI"]', "    )", "    {", '        uniform token subdivisionScheme = "none"',
                  "        uniform bool doubleSided = 1",
                  f"        int[] faceVertexCounts = {mesh.counts}",
                  f"        int[] faceVertexIndices = {list(range(len(P)))}",
                  f"        point3f[] points = {f3(P)}",
                  f"        normal3f[] normals = {f3(N)} (", '            interpolation = "vertex"', "        )",
                  "        texCoord2f[] primvars:st = [" + ", ".join(f"({u:.5g}, {v:.5g})" for u, v in UV) + "] (",
                  '            interpolation = "vertex"', "        )",
                  f"        float3[] extent = {f3([P.min(0), P.max(0)])}",
                  f"        rel material:binding = <{root}/Looks/{mat}>", "    }"]
    lines.append("}")
    (OUT / f"{name}.usda").write_text("\n".join(lines) + "\n")
    return OUT / f"{name}.usda"


# -- printed textures -------------------------------------------------------------------------------------------------
def font(px, bold=True):
    return ImageFont.truetype(FONT if bold else FONT_R, px)


def fit(d, text, px, width, bold=True):
    """The largest font no bigger than ``px`` that prints ``text`` within ``width`` pixels."""
    while px > 8 and d.textlength(text, font=font(px, bold)) > width:
        px -= 1
    return font(px, bold)


def lcd_image(name, title, color, body="", sub="", bar=None, trace=False):
    W, H = 560, 460  # 0.1 mm per pixel: LCD_W x LCD_H
    im = Image.new("RGB", (W, H), (8, 12, 18))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, W, 62], fill=(22, 40, 70))
    d.text((14, 12), "SURFACE ROUGHNESS", font=font(30), fill=(220, 230, 240))
    d.text((W - 120, 14), "N=5", font=font(28, False), fill=(170, 190, 210))
    d.rounded_rectangle([14, 78, W - 14, 188], radius=14, fill=color)
    f = fit(d, title, 64, W - 60)
    d.text(((W - d.textlength(title, font=f)) / 2, 133), title, font=f, fill=(10, 14, 20), anchor="lm")
    if body:
        d.text((24, 206), body, font=font(84), fill=(235, 240, 245))
    if sub:
        d.text((26, 306), sub, font=font(30, False), fill=(160, 175, 190))
    if bar is not None:
        d.rectangle([24, 380, W - 24, 420], outline=(120, 200, 230), width=3)
        d.rectangle([28, 384, 28 + int((W - 56) * bar), 416], fill=color)
    if trace:
        rng = np.random.default_rng(7)
        ys = 400 + np.cumsum(rng.normal(0, 3.0, W - 48)) * 0.4
        ys = ys - ys.mean() + 400 + 6 * np.sin(np.linspace(0, 40, W - 48))
        d.line([(24 + i, float(y)) for i, y in enumerate(ys)], fill=(120, 230, 150), width=3)
        d.line([(24, 400), (W - 24, 400)], fill=(70, 90, 110), width=1)
    im.save(TEX / f"{name}.png")


def label_image(name, size, bg, lines, fg=(245, 248, 252)):
    im = Image.new("RGB", size, bg)
    d = ImageDraw.Draw(im)
    for text, px, y, bold in lines:
        f = fit(d, text, px, size[0] - 24, bold)
        d.text(((size[0] - d.textlength(text, font=f)) / 2, y), text, font=f, fill=fg)
    im.save(TEX / f"{name}.png")


def make_textures():
    lcd_image("lcd_base", "--", (60, 70, 80), body="", sub="Place drive on work")
    lcd_image("lcd_oor", "OUT OF RANGE", (245, 150, 30), body="+3.6°", sub="Level the drive unit")
    lcd_image("lcd_ready", "READY", (60, 225, 110), body="0.0°", sub="Press START")
    lcd_image("lcd_meas", "MEASURING", (50, 205, 240), body="Ra ...", sub="Do not touch", bar=0.6)
    lcd_image("lcd_done", "DONE", (60, 225, 110), body="Ra 0.82µm", sub="λc 0.8 mm  x5", trace=True)
    lcd_image("lcd_fault", "ERROR", (235, 50, 50), body="E-02", sub="Vibration detected")
    label_image("lbl_console", (560, 140), (18, 96, 168),
                [("SURFACE ROUGHNESS TESTER", 40, 18, True), ("Ra  Rz  Rq   ·   ISO 4287", 34, 78, False)])
    label_image("lbl_start", (200, 240), (24, 150, 66), [("▶", 90, 30, True), ("START", 50, 150, True)])
    label_image("lbl_key_menu", (140, 120), (95, 100, 108), [("MENU", 34, 38, True)])
    label_image("lbl_key_ra", (140, 120), (18, 96, 168), [("Ra", 52, 28, True)])
    label_image("lbl_key_print", (140, 120), (95, 100, 108), [("▼", 56, 24, True)])
    label_image("lbl_drive", (600, 150), (18, 96, 168),
                [("DRIVE UNIT   ▶ DETECTOR", 44, 22, True), ("stylus 2 µm  60°  ·  0.75 mN", 36, 86, False)])
    label_image("lbl_plate", (300, 150), (196, 200, 206),
                [("Δ 0.8 mm", 46, 18, True), ("SKID  ·  2 µm", 40, 82, False)], fg=(30, 34, 40))


# -- the models -------------------------------------------------------------------------------------------------------
def shaft_model():
    s = SHAFT_L / 2
    prof = [(0.0, -s + 0.0016), (0.0025, -s), (SHAFT_R - 0.0012, -s), (SHAFT_R, -s + 0.0012), (SHAFT_R, s - 0.0012),
            (SHAFT_R - 0.0012, s), (0.0025, s), (0.0, s - 0.0016)]
    rings = lathe(prof, seg=96)
    mats = {0: "dark_steel", 1: "steel_end", 2: "ground_steel", 3: "ground_steel", 4: "ground_steel", 5: "steel_end",
            6: "dark_steel"}
    parts = [(mats[k], at(m, (0, 0, SHAFT_AXIS_Z), (-90, 0, 0))) for k, m in rings.items()]
    vb = [(-0.032, 0), (0.032, 0), (0.032, 0.012), (0.029, 0.012), (0.029, 0.018), (0.032, 0.018), (0.032, 0.030),
          (0.0247, 0.030), (0.0027, 0.008), (0.0027, 0.004), (-0.0027, 0.004), (-0.0027, 0.008), (-0.0247, 0.030),
          (-0.032, 0.030), (-0.032, 0.018), (-0.029, 0.018), (-0.029, 0.012), (-0.032, 0.012)]
    for dy in (-VBLOCK_DY, VBLOCK_DY):
        parts.append(("vblock_steel", extrude_xz(vb, dy - 0.012, dy + 0.012)))
    return write_model("shaft_vblocks", parts)


def anvil_model():
    parts = [("anvil_steel", at(rbox((0.056, 0.050, 0.042), 0.0015), (0, 0, 0.021)))]
    for x in (-0.0155, 0.0155):
        parts.append(("dark_steel", at(rbox((0.006, 0.042, 0.004), 0.0008), (x, 0, 0.044))))
    return write_model("rear_anvil", parts)


def pad_model():
    parts = [("rubber", at(rbox((0.048, 0.170, 0.005), 0.0015), (0, 0, 0.0025)))]
    for k in range(25):
        y = -0.078 + k * 0.0065
        if abs(y - REAR_Y) < 0.016:
            continue
        parts.append(("rubber", at(rbox((0.042, 0.0018, 0.0006), 0.0003, n=2), (0, y, 0.0051))))
    parts.append(("rubber_grey", at(rbox((0.036, 0.028, 0.0055), 0.0012), (0, REAR_Y, 0.005 + 0.00275))))
    return write_model("staging_pad", parts)


def drive_body_model():
    q2 = math.sqrt(2.0)
    parts = [
        ("drive_paint", rbox((BODY_W, BODY_L, BODY_H), 0.0032)),
        ("blue_paint", at(rbox((0.019, 0.096, 0.0025), 0.0009), (0, -0.004, BODY_H / 2 + 0.00125))),
        ("lbl_drive", decal(0.040, 0.0150, (0.0, -0.032), BODY_H / 2 + 0.0026)),
        ("ground_steel", at(rbox((0.015, 0.044, 0.0008), 0.0003, n=2), (0, 0.010, BODY_H / 2 + 0.0029))),
        ("lbl_plate", decal(0.020, 0.0100, (0.0, 0.014), BODY_H / 2 + 0.0034)),
        ("rubber", at(rbox((0.012, 0.012, 0.010), 0.0025), (0, -0.064, 0.001))),
        ("dark_steel", at(rbox((0.022, 0.026, 0.010), 0.0015), (0, REAR_Y, 0.0))),
        ("dark_steel", at(rbox((0.008, 0.012, 0.004), 0.001), (0, STYLUS_Y, -0.0125))),
    ]
    for sx in (-1, 1):
        parts.append(("ground_steel", at(rbox((0.008, 0.030, 0.005), 0.0008), (sx * 0.016, SADDLE_Y, -0.0135))))
        cx, cz = sx * V_CHEEK_DIST / q2, V_AX_LOCAL_Z + V_CHEEK_DIST / q2
        parts.append(("ground_steel", at(rbox((0.022, 0.030, 0.004), 0.0008), (cx, SADDLE_Y, cz), (0, sx * 45.0, 0))))
    y0, y1 = BODY_L / 2, BODY_L / 2 + 0.040
    prof = [(0.0, y0), (0.0070, y0), (0.0070, y0 + 0.004), (0.0066, y0 + 0.0045), (0.0066, y1 - 0.006),
            (0.0060, y1 - 0.0045), (0.0052, y1), (0.0, y1)]
    rings = lathe(prof, seg=48)
    for k, m in rings.items():
        mat = "dark_steel" if k in (1, 2) else "ground_steel"
        parts.append((mat, at(m, (0, 0, -0.004), (-90, 0, 0))))
    return write_model("drive_body", parts)


def drive_foot_model():
    parts = [("rubber", at(rbox((0.018, 0.018, 0.003), 0.001), (0, REAR_Y, REAR_PAD_Z)))]
    prof = [(0.0, -0.026)]
    z = -0.026
    while z < 0.018 - 1e-9:
        prof += [(0.0040, z + 0.0003), (0.0036, z + 0.0006), (0.0040, z + 0.0012)]
        z += 0.0012
    prof += [(0.0, 0.018)]
    for m in lathe(prof, seg=24).values():
        parts.append(("ground_steel", at(m, (0, REAR_Y, 0))))
    return write_model("drive_foot", parts)


def thumbwheel_model():
    side, faces = knurl_wheel(THUMB_R, THUMB_H)
    parts = [("brass", at(side, (0, REAR_Y, THUMB_Z))), ("brass", at(faces, (0, REAR_Y, THUMB_Z))),
             ("dark_steel", at(rbox((0.024, 0.003, 0.002), 0.0006, n=2), (0, REAR_Y, THUMB_Z + THUMB_H / 2 + 0.001)))]
    return write_model("thumbwheel", parts)


def stylus_model():
    parts = [("orange", at(rbox((0.015, 0.006, 0.005), 0.0012), (0, STYLUS_Y, 0.005)))]
    return write_model("stylus_carriage", parts)


def console_model():
    parts = [
        ("abs_charcoal", at(rbox(CONSOLE, 0.0045), (0, 0, CZ / 2))),
        ("lcd_base", decal(LCD_W, LCD_H, LCD_C, CZ + 0.0011)),
        ("dark_steel", at(rbox((LCD_H + 0.004, LCD_W + 0.004, 0.0010), 0.0008, n=2), (LCD_C[0], 0, CZ + 0.0005))),
        ("lbl_console", decal(0.056, 0.014, (0.068, 0.0), CZ + 0.0006)),
        ("dark_steel", at(rbox((0.014, 0.008, 0.014), 0.0015), (0.050, CONSOLE[1] / 2 + 0.004, 0.025))),
    ]
    for y, lbl, mat in ((-0.018, "lbl_key_print", "abs_grey_key"), (0.0, "lbl_key_ra", "blue_paint"),
                        (0.018, "lbl_key_menu", "abs_grey_key")):
        parts.append((mat, at(rbox((0.012, 0.014, 0.002), 0.0008, n=3), (-0.015, y, CZ + 0.001))))
        parts.append((lbl, decal(0.012, 0.0102, (-0.015, y), CZ + 0.0021)))
    return write_model("console", parts)


def start_key_model():
    parts = [("key_green", at(rbox((0.024, 0.020, 0.005), 0.0018), (START_X, 0, CZ + 0.0025))),
             ("lbl_start", decal(0.0165, 0.0198, (START_X, 0.0), CZ + 0.0051))]
    return write_model("start_key", parts)


def lcd_state_models():
    for k, name in enumerate(("lcd_oor", "lcd_ready", "lcd_meas", "lcd_done", "lcd_fault")):
        write_model(name, [(name, decal(LCD_W - 0.001, LCD_H - 0.001, LCD_C, CZ + 0.0016 + 0.0001 * k))])


def main():
    src = pathlib.Path(sys.argv[1])
    TEX.mkdir(parents=True, exist_ok=True)
    copy_tiles(src)
    make_textures()
    for f in (shaft_model, anvil_model, pad_model, drive_body_model, drive_foot_model, thumbwheel_model, stylus_model,
              console_model, start_key_model, lcd_state_models):
        f()
    for p in sorted(OUT.glob("*.usda")):
        print(f"{p.name}: {p.stat().st_size // 1024} KiB")


if __name__ == "__main__":
    main()
