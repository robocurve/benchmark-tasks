"""Author the cap1 visual models as USD (run with Blender's bundled pxr: blender -b --python make_visuals.py -- OUT).

Geometry follows SCENE.md (14 mL 17 x 100 mm PP round-bottom culture tube + dual-position LDPE snap cap, 3-place
17 mm PP rack; the bench pad is written by make_pad.py). Every model is in its labkit part frame (metres,
Z up, origin = the part's body origin) so labkit can reference it 1:1 over the part's invisible collision shapes.
Materials: MDL (OmniGlass / OmniPBR, Kit built-ins) for RTX, with a UsdPreviewSurface fallback on the same Material.
"""
import math
import pathlib
import sys

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

OUT = pathlib.Path(sys.argv[sys.argv.index("--") + 1])
MM = 1e-3

# Printed sector on the tube (must match make_textures.py)
PRINT_Z0, PRINT_Z1 = 0.010, 0.086
PRINT_A0, PRINT_A1 = 0.40, 0.70

T_R_OUT, T_R_IN = 8.5 * MM, 7.5 * MM
T_SPLIT = 0.086  # main body / sliding mouth collar split (cap1.py d1/d2 collar starts at 0.086)


def srgb_to_lin(c):
    return tuple(((x / 12.92) if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4) for x in c)


# ---------------------------------------------------------------------------------------------------------------- mesh
class MeshBuf:
    def __init__(self):
        self.p, self.n, self.st, self.counts, self.idx = [], [], [], [], []

    def vert(self, p, n, st=(0.0, 0.0)):
        self.p.append(tuple(map(float, p)))
        self.n.append(tuple(map(float, n)))
        self.st.append(tuple(map(float, st)))
        return len(self.p) - 1

    def face(self, ids):
        self.counts.append(len(ids))
        self.idx.extend(ids)

    def write(self, stage, path, material):
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in self.p]))
        m.CreateFaceVertexCountsAttr(Vt.IntArray(self.counts))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(self.idx))
        m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(*v) for v in self.n]))
        m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        m.CreateDoubleSidedAttr(True)
        pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec2fArray([Gf.Vec2f(*s) for s in self.st]))
        pts = np.array(self.p)
        m.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*pts.min(0)), Gf.Vec3f(*pts.max(0))]))
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material)
        return m


def lathe(buf, poly, segs=96, a0=0.0, a1=1.0, uv=None):
    """Revolve one smooth (r, z) polyline (counter-clockwise around the solid => outward normals)."""
    poly = np.asarray(poly, float)
    k = len(poly)
    tn = []
    for i in range(k - 1):
        dr, dz = poly[i + 1] - poly[i]
        L = math.hypot(dr, dz) or 1.0
        tn.append((dz / L, -dr / L))
    vn = []
    for i in range(k):
        a = np.array(tn[max(i - 1, 0)]) + np.array(tn[min(i, k - 2)])
        vn.append(a / (np.linalg.norm(a) or 1.0))
    base = len(buf.p)
    for j in range(segs + 1):
        f = a0 + (a1 - a0) * j / segs
        th = 2 * math.pi * f
        c, s = math.cos(th), math.sin(th)
        for i, (r, z) in enumerate(poly):
            nr, nz = vn[i]
            st = uv(f, r, z) if uv else (f, i / max(k - 1, 1))
            buf.vert((r * c, r * s, z), (nr * c, nr * s, nz), st)
    for j in range(segs):
        for i in range(k - 1):
            a = base + j * k + i
            b = base + (j + 1) * k + i
            buf.face([a, b, b + 1, a + 1])


def arc(cr, cz, rad, t0, t1, n):
    return [(cr + rad * math.cos(t), cz + rad * math.sin(t)) for t in np.linspace(t0, t1, n)]


def flat_poly(buf, pts, normal, st=None):
    ids = [buf.vert(p, normal, st[i] if st else (0, 0)) for i, p in enumerate(pts)]
    buf.face(ids)


def box(buf, lo, hi):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    flat_poly(buf, [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)], (0, 0, 1))
    flat_poly(buf, [(x0, y1, z0), (x1, y1, z0), (x1, y0, z0), (x0, y0, z0)], (0, 0, -1))
    flat_poly(buf, [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)], (0, -1, 0))
    flat_poly(buf, [(x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1)], (0, 1, 0))
    flat_poly(buf, [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)], (1, 0, 0))
    flat_poly(buf, [(x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1)], (-1, 0, 0))


def rounded_rect(cx, cy, hx, hy, rad, n=8):
    pts = []
    for (sx, sy, t0) in ((1, 1, 0), (-1, 1, 90), (-1, -1, 180), (1, -1, 270)):
        ox, oy = cx + sx * (hx - rad), cy + sy * (hy - rad)
        for t in np.linspace(math.radians(t0), math.radians(t0 + 90), n):
            pts.append((ox + rad * math.cos(t), oy + rad * math.sin(t)))
    return pts


def extrude_convex(buf, outline, z0, z1):
    """Prism over a convex CCW outline in XY (rounded top edges are not needed at this scale)."""
    flat_poly(buf, [(x, y, z1) for x, y in outline], (0, 0, 1))
    flat_poly(buf, [(x, y, z0) for x, y in reversed(outline)], (0, 0, -1))
    m = len(outline)
    for i in range(m):
        (xa, ya), (xb, yb) = outline[i], outline[(i + 1) % m]
        nx, ny = yb - ya, -(xb - xa)
        L = math.hypot(nx, ny) or 1.0
        flat_poly(buf, [(xa, ya, z0), (xb, yb, z0), (xb, yb, z1), (xa, ya, z1)], (nx / L, ny / L, 0))


def panel_xz(buf, x0, x1, y0, y1, z0, z1, rad, n=10):
    """End panel: an XZ outline with rounded top corners, extruded along Y."""
    out = [(x0, z0), (x1, z0)]
    out += arc(x1 - rad, z1 - rad, rad, 0, math.pi / 2, n)
    out += arc(x0 + rad, z1 - rad, rad, math.pi / 2, math.pi, n)
    flat_poly(buf, [(x, y0, z) for x, z in out], (0, -1, 0))
    flat_poly(buf, [(x, y1, z) for x, z in reversed(out)], (0, 1, 0))
    m = len(out)
    for i in range(m):
        (xa, za), (xb, zb) = out[i], out[(i + 1) % m]
        nx, nz = zb - za, -(xb - xa)
        L = math.hypot(nx, nz) or 1.0
        flat_poly(buf, [(xa, y1, za), (xb, y1, zb), (xb, y0, zb), (xa, y0, za)], (nx / L, 0, nz / L))


def deck_with_holes(buf, hx, y_edges, holes, r_hole, z0, z1, segs=64):
    """A plate |x| <= hx, y in [y_edges[0], y_edges[-1]], z0..z1, with one round hole per y cell (square-to-circle
    ring quads), hole walls, and the outer side walls."""
    for k, yc in enumerate(holes):
        ya, yb = y_edges[k], y_edges[k + 1]
        corners = [(hx, yb), (-hx, yb), (-hx, ya), (hx, ya)]
        ang = sorted({*(math.atan2(cy - yc, cx) % (2 * math.pi) for cx, cy in corners),
                      *np.linspace(0, 2 * math.pi, segs, endpoint=False)})
        ring = []
        for t in ang:
            c, s = math.cos(t), math.sin(t)
            sc = min(hx / abs(c) if abs(c) > 1e-9 else 1e9,
                     ((yb - yc) if s > 0 else (yc - ya)) / abs(s) if abs(s) > 1e-9 else 1e9)
            ring.append(((r_hole * c, yc + r_hole * s), (sc * c, yc + sc * s), (c, s)))
        m = len(ring)
        for i in range(m):
            (ia, oa, na), (ib, ob, nb) = ring[i], ring[(i + 1) % m]
            flat_poly(buf, [(*ia, z1), (*oa, z1), (*ob, z1), (*ib, z1)], (0, 0, 1))
            flat_poly(buf, [(*ia, z0), (*ib, z0), (*ob, z0), (*oa, z0)], (0, 0, -1))
            a = buf.vert((*ia, z0), (-na[0], -na[1], 0))
            b = buf.vert((*ib, z0), (-nb[0], -nb[1], 0))
            c_ = buf.vert((*ib, z1), (-nb[0], -nb[1], 0))
            d = buf.vert((*ia, z1), (-na[0], -na[1], 0))
            buf.face([a, d, c_, b])
    y0, y1 = y_edges[0], y_edges[-1]
    flat_poly(buf, [(hx, y0, z0), (hx, y1, z0), (hx, y1, z1), (hx, y0, z1)], (1, 0, 0))
    flat_poly(buf, [(-hx, y1, z0), (-hx, y0, z0), (-hx, y0, z1), (-hx, y1, z1)], (-1, 0, 0))
    flat_poly(buf, [(-hx, y0, z0), (hx, y0, z0), (hx, y0, z1), (-hx, y0, z1)], (0, -1, 0))
    flat_poly(buf, [(hx, y1, z0), (-hx, y1, z0), (-hx, y1, z1), (hx, y1, z1)], (0, 1, 0))


# ----------------------------------------------------------------------------------------------------------- materials
def _shader(stage, mat, name, mdl, inputs):
    sh = UsdShade.Shader.Define(stage, f"{mat.GetPath()}/{name}")
    sh.SetSourceAsset(Sdf.AssetPath(f"{mdl}.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier(mdl, "mdl")
    for k, (t, v) in inputs.items():
        sh.CreateInput(k, t).Set(v)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return sh


def _preview(stage, mat, color, rough, metal=0.0, opacity=1.0, ior=1.5, tex=None):
    ps = UsdShade.Shader.Define(stage, f"{mat.GetPath()}/Preview")
    ps.CreateIdAttr("UsdPreviewSurface")
    ps.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    ps.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
    ps.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(ior)
    if tex:
        rd = UsdShade.Shader.Define(stage, f"{mat.GetPath()}/st")
        rd.CreateIdAttr("UsdPrimvarReader_float2")
        rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tx = UsdShade.Shader.Define(stage, f"{mat.GetPath()}/Tex")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(tex))
        tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
        ps.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
        if opacity == "alpha":
            ps.CreateInput("opacity", Sdf.ValueTypeNames.Float).ConnectToSource(
                tx.CreateOutput("a", Sdf.ValueTypeNames.Float))
            ps.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.5)
    else:
        ps.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        ps.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    mat.CreateSurfaceOutput().ConnectToSource(ps.ConnectableAPI(), "surface")


def glass(stage, name, color_srgb, ior, frost=0.0):
    mat = UsdShade.Material.Define(stage, f"/Root/Looks/{name}")
    col = Gf.Vec3f(*srgb_to_lin(color_srgb))
    _shader(stage, mat, "Glass", "OmniGlass", {
        "glass_color": (Sdf.ValueTypeNames.Color3f, col),
        "glass_ior": (Sdf.ValueTypeNames.Float, ior),
        "frosting_roughness": (Sdf.ValueTypeNames.Float, frost),
        "thin_walled": (Sdf.ValueTypeNames.Bool, False),
    })
    _preview(stage, mat, col, 0.05, opacity=0.25, ior=ior)
    return mat


def plastic(stage, name, color_srgb, rough, spec=0.5):
    mat = UsdShade.Material.Define(stage, f"/Root/Looks/{name}")
    col = Gf.Vec3f(*srgb_to_lin(color_srgb))
    _shader(stage, mat, "PBR", "OmniPBR", {
        "diffuse_color_constant": (Sdf.ValueTypeNames.Color3f, col),
        "reflection_roughness_constant": (Sdf.ValueTypeNames.Float, rough),
        "metallic_constant": (Sdf.ValueTypeNames.Float, 0.0),
        "specular_level": (Sdf.ValueTypeNames.Float, spec),
    })
    _preview(stage, mat, col, rough)
    return mat


def printed(stage, name, tex, rough=0.5, cutout=False):
    """Printed ink / label: texture on OmniPBR (cutout from the PNG alpha when ``cutout``)."""
    mat = UsdShade.Material.Define(stage, f"/Root/Looks/{name}")
    ins = {
        "diffuse_texture": (Sdf.ValueTypeNames.Asset, Sdf.AssetPath(tex)),
        "reflection_roughness_constant": (Sdf.ValueTypeNames.Float, rough),
        "metallic_constant": (Sdf.ValueTypeNames.Float, 0.0),
    }
    if cutout:
        ins.update({
            "enable_opacity": (Sdf.ValueTypeNames.Bool, True),
            "enable_opacity_texture": (Sdf.ValueTypeNames.Bool, True),
            "opacity_texture": (Sdf.ValueTypeNames.Asset, Sdf.AssetPath(tex)),
            "opacity_mode": (Sdf.ValueTypeNames.Int, 0),  # mono_alpha
            "opacity_threshold": (Sdf.ValueTypeNames.Float, 0.5),
        })
    _shader(stage, mat, "PBR", "OmniPBR", ins)
    _preview(stage, mat, (1, 1, 1), rough, opacity="alpha" if cutout else 1.0, tex=tex)
    return mat


def new_stage(name):
    path = OUT / f"{name}.usda"
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Root")
    stage.SetDefaultPrim(root.GetPrim())
    stage.DefinePrim("/Root/Looks", "Scope")
    return stage


def save(stage):
    stage.GetRootLayer().Save()
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(stage.GetDefaultPrim())
    r = bb.ComputeAlignedRange()
    print(f"{stage.GetRootLayer().identifier}: min {tuple(round(v * 1e3, 2) for v in r.GetMin())} mm "
          f"max {tuple(round(v * 1e3, 2) for v in r.GetMax())} mm")


# ------------------------------------------------------------------------------------------------------------- models
def make_tube_body():
    """Main tube body z = 0 .. 86 mm: clarified PP wall (round bottom), 3 mL LB broth, printed scale + write-on patch."""
    st = new_stage("tube_body")
    pp = glass(st, "PP_clarified", (0.95, 0.97, 0.98), 1.49, frost=0.06)
    broth = glass(st, "LB_broth", (0.93, 0.70, 0.30), 1.34)
    ink = printed(st, "Print", "./textures/tube_print.png", rough=0.6, cutout=True)
    # Closed solid: outer hemisphere + wall up to the collar split, top annulus, inner wall + inner hemisphere.
    outer = arc(0.0, T_R_OUT, T_R_OUT, -math.pi / 2, 0.0, 20) + [(T_R_OUT, T_SPLIT)]
    inner = [(T_R_IN, T_SPLIT)] + arc(0.0, T_R_OUT, T_R_IN, 0.0, -math.pi / 2, 20)
    b = MeshBuf()
    lathe(b, outer)
    lathe(b, [(T_R_OUT, T_SPLIT), (T_R_IN, T_SPLIT)])
    lathe(b, inner)
    b.write(st, "/Root/wall", pp)
    # Broth: 3.0 mL (SCENE.md mass) => level 20.5 mm above the outer bottom, slight meniscus.
    lvl = 0.0010 + T_R_IN + (3.0 - 2 / 3 * math.pi * (T_R_IN * 100) ** 3) / (math.pi * (T_R_IN * 100) ** 2) / 100
    rb = T_R_IN - 0.00005
    lb = MeshBuf()
    lathe(lb, arc(0.0, T_R_OUT, rb, -math.pi / 2, 0.0, 16) + [(rb, lvl + 0.0006)])
    lathe(lb, [(rb, lvl + 0.0006), (rb * 0.8, lvl + 0.0001), (0.0, lvl)])
    lb.write(st, "/Root/broth", broth)
    # Printed sector just outside the wall.
    pb = MeshBuf()
    rp = T_R_OUT + 0.00004
    lathe(pb, [(rp, PRINT_Z0), (rp, PRINT_Z1)], segs=48, a0=PRINT_A0, a1=PRINT_A1,
          uv=lambda f, r, z: ((f - PRINT_A0) / (PRINT_A1 - PRINT_A0), (z - PRINT_Z0) / (PRINT_Z1 - PRINT_Z0)))
    pb.write(st, "/Root/print", ink)
    save(st)
    return lvl


def make_tube_mouth():
    """Sliding mouth collar z = 86 .. 100 mm (rides on the d2 detent): wall, snap bead, rounded rim."""
    st = new_stage("tube_mouth")
    pp = glass(st, "PP_clarified", (0.95, 0.97, 0.98), 1.49, frost=0.06)
    b = MeshBuf()
    outer = [(T_R_OUT, T_SPLIT), (T_R_OUT, 0.0930)] + arc(T_R_OUT, 0.0950, 0.0003, -math.pi / 2, math.pi / 2, 8)
    outer += [(T_R_OUT, 0.0975)] + arc(T_R_OUT - 0.0004, 0.0995, 0.0004, 0.0, math.pi / 2, 6)
    lathe(b, [(p[0], p[1]) for p in outer])
    lathe(b, [(T_R_OUT - 0.0004, 0.0999), (T_R_IN, 0.1000)])
    lathe(b, [(T_R_IN, 0.1000), (T_R_IN, T_SPLIT)])
    lathe(b, [(T_R_IN, T_SPLIT), (T_R_OUT, T_SPLIT)])
    b.write(st, "/Root/mouth", pp)
    save(st)


def make_cap():
    """Dual-position snap cap, open side down: 20.4 mm skirt, 21.2 mm flange crown, recessed top, natural LDPE."""
    st = new_stage("snap_cap")
    ldpe = plastic(st, "LDPE_natural", (0.93, 0.92, 0.88), 0.42)
    b = MeshBuf()
    r_in, r_sk, r_fl = 0.0096, 0.0102, 0.0106
    outer = [(r_in, 0.0)] + arc(r_sk - 0.0003, 0.0003, 0.0003, -math.pi / 2, 0.0, 5) + [(r_sk, 0.0150)]
    lathe(b, outer)
    lathe(b, [(r_sk, 0.0150), (r_fl, 0.0156)])
    lathe(b, [(r_fl, 0.0156), (r_fl, 0.0192)] + arc(r_fl - 0.0008, 0.0192, 0.0008, 0.0, math.pi / 2, 6))
    lathe(b, [(r_fl - 0.0008, 0.0200), (0.0074, 0.0200)])
    lathe(b, [(0.0074, 0.0200), (0.0070, 0.0196), (0.0, 0.0196)])  # recessed centre disc
    lathe(b, [(0.0, 0.0155), (r_in - 0.0005, 0.0155), (r_in, 0.0150), (r_in, 0.0)])
    b.write(st, "/Root/cap", ldpe)
    save(st)


def make_rack():
    """3-place 17 mm rack (56 x 160 x 62 mm, decks at 30 / 58 mm): moulded teal-blue PP end panels and two decks with round 18.4 mm wells,
    base plate on a dark anti-slip foot, white well-number strip on the -X face."""
    st = new_stage("tube_rack")
    pp = plastic(st, "PP_teal", (0.18, 0.42, 0.56), 0.48)
    foot = plastic(st, "TPE_foot", (0.16, 0.18, 0.20), 0.85, spec=0.3)
    label = printed(st, "Label", "./textures/rack_label.png", rough=0.55)
    b = MeshBuf()
    extrude_convex(b, rounded_rect(0, 0, 0.028, 0.080, 0.004), 0.001, 0.005)
    for sy in (-1, 1):
        y0, y1 = sorted((sy * 0.072, sy * 0.080))
        panel_xz(b, -0.028, 0.028, y0, y1, 0.005, 0.062, 0.006)  # end panels stand 4 mm proud of the deck
    edges = [-0.072, -0.025, 0.025, 0.072]
    deck_with_holes(b, 0.028, edges, (-0.050, 0.0, 0.050), 0.0092, 0.026, 0.030)
    deck_with_holes(b, 0.028, edges, (-0.050, 0.0, 0.050), 0.0092, 0.052, 0.058)
    b.write(st, "/Root/body", pp)
    f = MeshBuf()
    extrude_convex(f, rounded_rect(0, 0, 0.027, 0.079, 0.004), 0.0, 0.001)
    f.write(st, "/Root/foot", foot)
    lb = MeshBuf()
    x = -0.0281
    zl0, zl1 = 0.05275, 0.05725
    flat_poly(lb, [(x, 0.065, zl0), (x, -0.065, zl0), (x, -0.065, zl1), (x, 0.065, zl1)], (-1, 0, 0),
              st=[(0, 0), (1, 0), (1, 1), (0, 1)])
    lb.write(st, "/Root/label", label)
    save(st)


OUT.mkdir(parents=True, exist_ok=True)
lvl = make_tube_body()
make_tube_mouth()
make_cap()
make_rack()
print(f"broth level {lvl * 1e3:.1f} mm")
