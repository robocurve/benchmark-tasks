"""Author the visual models for estop-twist-release into estop_visuals.usdc (offline; the task never runs this file).

    python3 build_visuals.py tex          # Pillow: printed legends and decals into tex/
    blender -b --python build_visuals.py  # pxr (Blender's bundled USD): the meshes into estop_visuals.usdc

Each prim /V/<part>/<body> holds meshes in that body's local frame (labkit references it onto the matching rigid
body; the collider boxes and cylinders of the task file stay invisible). Coordinates mirror estop_twist_release.py.
Authored to the cited products (README): 16 mm-panel 40 mm mushroom push-to-lock / turn-to-reset e-stop in a yellow
1-hole station with a 60 mm EMERGENCY STOP legend plate; grey 2-hole station with a green START button and a green
RUN pilot lamp; single-platen 200 mm metallographic grinder-polisher. Bench dressing: NVIDIA SimReady models (CC-BY-4.0,
assets/props). Materials: UsdPreviewSurface PBR (the lit RUN lens is emissive).
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
TEX = HERE / "tex"
OUT = HERE / "estop_visuals.usdc"
NO_SHADOW: set = set()
def _font(px):
    from PIL import ImageFont

    for name in ("DejaVuSans-Bold.ttf", "DejaVuSans.ttf", "LiberationSans-Bold.ttf"):
        try:
            return ImageFont.truetype(name, px)
        except OSError:
            continue
    return ImageFont.load_default()





class Mesh:
    def __init__(self):
        self.p, self.f, self.st = [], [], []

    def add(self, pts, faces, st):
        base = len(self.p)
        self.p += [tuple(map(float, q)) for q in pts]
        self.st += [tuple(map(float, q)) for q in st]
        self.f += [[base + i for i in face] for face in faces]
        return self


def lathe(profile, seg=64, axis="z", center=(0, 0, 0), a0=0.0, a1=2 * math.pi):
    """Revolve (r, h) profile points about ``axis`` through ``center``. Returns a Mesh."""
    closed = abs(a1 - a0 - 2 * math.pi) < 1e-9
    na = seg if closed else seg + 1
    pts, st = [], []
    hs = [h for _, h in profile]
    span = (max(hs) - min(hs)) or 1.0
    for j, (r, h) in enumerate(profile):
        for i in range(na):
            a = a0 + (a1 - a0) * i / seg
            u, v = r * math.cos(a), r * math.sin(a)
            if axis == "z":
                q = (u, v, h)
            elif axis == "x":
                q = (h, u, v)
            else:
                q = (v, h, u)
            pts.append(np.add(q, center))
            st.append((i / seg, (h - min(hs)) / span))
    faces = []
    for j in range(len(profile) - 1):
        for i in range(seg):
            i2 = (i + 1) % na if closed else i + 1
            faces.append([j * na + i, j * na + i2, (j + 1) * na + i2, (j + 1) * na + i])
    m = Mesh().add(pts, faces, st)
    return m


def superbox(size, center=(0, 0, 0), e=0.18, nu=40, nv=24):
    """Rounded box (superellipsoid, exponent e) of full ``size`` around ``center``; UVs wrap around."""
    a = np.array(size) / 2.0

    def sgnpow(x, p):
        return np.sign(x) * abs(x) ** p

    pts, st = [], []
    for j in range(nv + 1):
        v = -math.pi / 2 + math.pi * j / nv
        for i in range(nu):
            u = -math.pi + 2 * math.pi * i / nu
            x = a[0] * sgnpow(math.cos(v), e) * sgnpow(math.cos(u), e)
            y = a[1] * sgnpow(math.cos(v), e) * sgnpow(math.sin(u), e)
            z = a[2] * sgnpow(math.sin(v), e)
            pts.append(np.add((x, y, z), center))
            st.append((i / nu, j / nv))
    faces = []
    for j in range(nv):
        for i in range(nu):
            i2 = (i + 1) % nu
            faces.append([j * nu + i, j * nu + i2, (j + 1) * nu + i2, (j + 1) * nu + i])
    return Mesh().add(pts, faces, st)


def slab(lo, hi, uv_scale=1.0):
    """Axis-aligned box mesh with per-face planar UVs (for tiled textures); uv_scale = tiles per metre."""
    lo, hi = np.array(lo, float), np.array(hi, float)
    m = Mesh()
    for ax in range(3):
        for side in (0, 1):
            o = [k for k in range(3) if k != ax]
            corners = []
            for a, b in ((0, 0), (1, 0), (1, 1), (0, 1)):
                q = np.zeros(3)
                q[ax] = hi[ax] if side else lo[ax]
                q[o[0]] = hi[o[0]] if a else lo[o[0]]
                q[o[1]] = hi[o[1]] if b else lo[o[1]]
                corners.append(q)
            st = [(q[o[0]] * uv_scale, q[o[1]] * uv_scale) for q in corners]
            face = [0, 1, 2, 3] if side else [0, 3, 2, 1]
            if ax == 1:
                face = face[::-1]
            m.add(corners, [face], st)
    return m


def sweep(path_pts, radius, seg=24, samples=80):
    """Tube of ``radius`` along a Catmull-Rom curve through ``path_pts``."""
    P = [np.array(p, float) for p in path_pts]
    P = [P[0]] + P + [P[-1]]
    curve = []
    for k in range(1, len(P) - 2):
        for t in np.linspace(0, 1, samples // (len(P) - 3), endpoint=k == len(P) - 3):
            p0, p1, p2, p3 = P[k - 1], P[k], P[k + 1], P[k + 2]
            curve.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t
                                + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3))
    tang = [curve[min(i + 1, len(curve) - 1)] - curve[max(i - 1, 0)] for i in range(len(curve))]
    tang = [t / (np.linalg.norm(t) + 1e-12) for t in tang]
    ref = np.array([0, 0, 1.0]) if abs(tang[0][2]) < 0.9 else np.array([1.0, 0, 0])
    n = np.cross(tang[0], ref)
    n /= np.linalg.norm(n)
    pts, st, L = [], [], 0.0
    for i, (c, t) in enumerate(zip(curve, tang)):
        n = n - np.dot(n, t) * t
        n /= np.linalg.norm(n) + 1e-12
        b = np.cross(t, n)
        if i:
            L += np.linalg.norm(c - curve[i - 1])
        for s in range(seg):
            a = 2 * math.pi * s / seg
            pts.append(c + radius * (math.cos(a) * n + math.sin(a) * b))
            st.append((s / seg, L * 20))
    faces = []
    for i in range(len(curve) - 1):
        for s in range(seg):
            s2 = (s + 1) % seg
            faces.append([i * seg + s, i * seg + s2, (i + 1) * seg + s2, (i + 1) * seg + s])
    return Mesh().add(pts, faces, st)



class Body:
    def __init__(self, stage, part, body, offset=(0, 0, 0)):
        self.stage = stage
        self.root = f"/V/{part}/{body}"
        xf = UsdGeom.Xform.Define(stage, self.root)
        if any(offset):
            xf.AddTranslateOp().Set(Gf.Vec3d(*(-np.array(offset, float))))
        UsdGeom.Scope.Define(stage, self.root + "/Looks")
        self.mats = {}
        self.n = 0

    def _tex(self, sh_path, file, st_reader, scale, srgb):
        t = UsdShade.Shader.Define(self.stage, sh_path)
        t.CreateIdAttr("UsdUVTexture")
        t.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./tex/{file}")
        t.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        t.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        t.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB" if srgb else "raw")
        if scale != 1.0:
            tr = UsdShade.Shader.Define(self.stage, sh_path + "_xf")
            tr.CreateIdAttr("UsdTransform2d")
            tr.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader.ConnectableAPI(), "result")
            tr.CreateInput("scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(scale, scale))
            t.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(tr.ConnectableAPI(), "result")
        else:
            t.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader.ConnectableAPI(), "result")
        return t

    def pbr(self, key, color=(0.8, 0.8, 0.8), rough=0.5, metal=0.0, tex=None, scale=1.0, clearcoat=0.0,
            tint=None, emissive=None):
        """tex = (albedo or None, orm or None, normal or None) file names in tex/."""
        if key in self.mats:
            return self.mats[key]
        mp = f"{self.root}/Looks/{key}"
        mat = UsdShade.Material.Define(self.stage, mp)
        sh = UsdShade.Shader.Define(self.stage, mp + "/pbr")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        if emissive is not None:
            sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
        if clearcoat:
            sh.CreateInput("clearcoat", Sdf.ValueTypeNames.Float).Set(clearcoat)
            sh.CreateInput("clearcoatRoughness", Sdf.ValueTypeNames.Float).Set(0.05)
        if tex:
            rd = UsdShade.Shader.Define(self.stage, mp + "/st")
            rd.CreateIdAttr("UsdPrimvarReader_float2")
            rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            alb, orm, nor = tex
            if alb:
                t = self._tex(mp + "/alb", alb, rd, scale, True)
                if tint is not None:
                    t.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*tint, 1.0))
                sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
                    t.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
            if orm:
                t = self._tex(mp + "/orm", orm, rd, scale, False)
                sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).ConnectToSource(
                    t.CreateOutput("g", Sdf.ValueTypeNames.Float))
                sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).ConnectToSource(
                    t.CreateOutput("b", Sdf.ValueTypeNames.Float))
                sh.CreateInput("occlusion", Sdf.ValueTypeNames.Float).ConnectToSource(
                    t.CreateOutput("r", Sdf.ValueTypeNames.Float))
            if nor:
                t = self._tex(mp + "/nor", nor, rd, scale, False)
                t.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2, 2, 2, 1))
                t.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1, -1, -1, 0))
                sh.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(
                    t.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[key] = mat
        return mat

    def glass(self, key, color=(0.92, 0.96, 1.0), ior=1.5, frost=0.0):
        """OmniGlass MDL (refractive), with a UsdPreviewSurface translucent fallback on the same material."""
        if key in self.mats:
            return self.mats[key]
        mp = f"{self.root}/Looks/{key}"
        mat = UsdShade.Material.Define(self.stage, mp)
        mdl = UsdShade.Shader.Define(self.stage, mp + "/glass")
        mdl.SetSourceAsset(Sdf.AssetPath("OmniGlass.mdl"), "mdl")
        mdl.SetSourceAssetSubIdentifier("OmniGlass", "mdl")
        mdl.CreateInput("glass_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        mdl.CreateInput("glass_ior", Sdf.ValueTypeNames.Float).Set(ior)
        mdl.CreateInput("thin_walled", Sdf.ValueTypeNames.Bool).Set(True)  # single-sheet shells (jar, lens, tube walls)
        mdl.CreateInput("frosting_roughness", Sdf.ValueTypeNames.Float).Set(frost)
        mat.CreateSurfaceOutput("mdl").ConnectToSource(mdl.ConnectableAPI(), "out")
        mat.CreateVolumeOutput("mdl").ConnectToSource(mdl.ConnectableAPI(), "out")
        mat.CreateDisplacementOutput("mdl").ConnectToSource(mdl.ConnectableAPI(), "out")
        ps = UsdShade.Shader.Define(self.stage, mp + "/preview")
        ps.CreateIdAttr("UsdPreviewSurface")
        ps.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        ps.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.05 + frost)
        ps.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.25)
        ps.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(ior)
        mat.CreateSurfaceOutput().ConnectToSource(ps.ConnectableAPI(), "surface")
        self.mats[key] = mat
        return mat

    def mesh(self, name, m: Mesh, mat, smooth=True):
        path = f"{self.root}/{name}"
        g = UsdGeom.Mesh.Define(self.stage, path)
        g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*q) for q in m.p]))
        g.CreateFaceVertexCountsAttr([len(f) for f in m.f])
        g.CreateFaceVertexIndicesAttr([i for f in m.f for i in f])
        g.CreateSubdivisionSchemeAttr("none")
        g.CreateOrientationAttr(UsdGeom.Tokens.rightHanded)
        g.CreateDoubleSidedAttr(True)
        pv = UsdGeom.PrimvarsAPI(g).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec2fArray([Gf.Vec2f(*q) for q in m.st]))
        # smooth vertex normals from face normals
        P = np.array(m.p)
        N = np.zeros_like(P)
        for f in m.f:
            a, b, c = P[f[0]], P[f[1]], P[f[2]]
            nrm = np.cross(b - a, c - a)
            for i in f:
                N[i] += nrm
        N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
        if smooth:
            g.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(*q) for q in N]))
            g.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        lo, hi = P.min(0), P.max(0)
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)]))
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat)
        if name in NO_SHADOW:
            UsdGeom.PrimvarsAPI(g).CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)
        self.n += len(m.f)
        return g


def _prop(stage, parent, origin, pkg, xy, yaw, anchor=None):
    """Reference a visual-only SimReady prop under a fixed body whose part frame sits at origin (x, y, z above the
    table): the model's footprint centre (or anchor) goes to world xy, its lowest point onto the table top."""
    ref = f"./props/{pkg}/{pkg}.usdc"
    src = Usd.Stage.Open(str(HERE / "props" / pkg / f"{pkg}.usdc"))
    rng = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(
        src.GetDefaultPrim()).ComputeAlignedRange()
    lo, hi = rng.GetMin(), rng.GetMax()
    cx, cy = anchor or (0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]))
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    t = (xy[0] - origin[0] - (c * cx - s * cy), xy[1] - origin[1] - (s * cx + c * cy), -origin[2] - lo[2])
    xf = UsdGeom.Xform.Define(stage, f"{parent}/prop_{pkg}")
    xf.AddTranslateOp().Set(Gf.Vec3d(*t))
    xf.AddRotateZOp().Set(yaw)
    xf.GetPrim().GetReferences().AddReference(ref)
    return xf.GetPath()


STEEL = ("steel_alb.png", "steel_orm.png", "steel_nor.png")



# -- printed legends and decals (Pillow) --------------------------------------------------------------------------------
def _ring_text(d, im, text, c, r, px, col, start_deg):
    from PIL import Image, ImageDraw

    f = _font(px)
    step = 1.0 * px * 180.0 / (math.pi * r)
    for i, ch in enumerate(text):
        a = start_deg + i * step
        tile = Image.new("RGBA", (2 * px, 2 * px), (0, 0, 0, 0))
        ImageDraw.Draw(tile).text((px, px), ch, fill=col, font=f, anchor="mm")
        tile = tile.rotate(-a, resample=Image.BICUBIC)
        t = math.radians(a)
        x, y = c + r * math.sin(t), c - r * math.cos(t)
        im.paste(tile, (int(x - px), int(y - px)), tile)


def textures():
    from PIL import Image, ImageDraw

    TEX.mkdir(exist_ok=True)
    n = 1024
    # 60 mm yellow EMERGENCY STOP legend plate (22 mm hole), black lettering round the ring
    im = Image.new("RGB", (n, n), (246, 196, 18))
    d = ImageDraw.Draw(im)
    _ring_text(d, im, "EMERGENCY STOP", n / 2, 0.405 * n, 74, (18, 18, 18), -78)
    im.save(TEX / "legend_estop.png")
    # mushroom cap top: red with two white clockwise turn-to-release arrows near the rim
    im = Image.new("RGB", (n, n), (176, 12, 10))
    d = ImageDraw.Draw(im)
    for a0 in (20.0, 200.0):
        box = [n * 0.17, n * 0.17, n * 0.83, n * 0.83]
        d.arc(box, a0, a0 + 110, fill=(245, 245, 245), width=34)
        t = math.radians(a0 + 110)
        cx, cy, r = n / 2, n / 2, n * 0.33
        tip = (cx + r * math.cos(t + 0.18), cy + r * math.sin(t + 0.18))
        b1 = (cx + (r + 46) * math.cos(t), cy + (r + 46) * math.sin(t))
        b2 = (cx + (r - 46) * math.cos(t), cy + (r - 46) * math.sin(t))
        d.polygon([tip, b1, b2], fill=(245, 245, 245))
    im.save(TEX / "cap_turn.png")
    # engraved legend plates (black, white text) for the 2-hole station
    for name, text in (("legend_start.png", "START"), ("legend_run.png", "RUN")):
        im = Image.new("RGB", (512, 224), (22, 22, 24))
        ImageDraw.Draw(im).text((256, 112), text, fill=(236, 236, 236), font=_font(120), anchor="mm")
        im.save(TEX / name)
    # machine maker-neutral rating label
    im = Image.new("RGB", (768, 256), (226, 228, 230))
    d = ImageDraw.Draw(im)
    d.text((384, 70), "GRINDER-POLISHER  200 mm", fill=(30, 30, 34), font=_font(54), anchor="mm")
    d.text((384, 160), "50-600 rpm   230 V~  50/60 Hz", fill=(60, 60, 64), font=_font(44), anchor="mm")
    im.save(TEX / "machine_label.png")
    # SiC abrasive paper: fine grey grain (rotationally uniform at platen scale)
    rng = np.random.default_rng(7)
    g = (rng.normal(58, 9, (512, 512))).clip(30, 90).astype(np.uint8)
    Image.fromarray(np.stack([g, g, g], -1)).save(TEX / "sic_paper.png")
    return sorted(p.name for p in TEX.iterdir())


# -- mesh bits ------------------------------------------------------------------------------------------------------------
def planar_top(m: Mesh, cx, cy, size):
    """Replace UVs with a top-down planar map of ``size`` metres centred at (cx, cy)."""
    m.st = [((q[0] - cx) / size + 0.5, (q[1] - cy) / size + 0.5) for q in m.p]
    return m


def disc_up(center, radius, seg=96):
    c = np.array(center, float)
    pts, st = [c], [(0.5, 0.5)]
    for i in range(seg):
        a = 2 * math.pi * i / seg
        pts.append(c + (radius * math.cos(a), radius * math.sin(a), 0.0))
        st.append((0.5 + 0.5 * math.cos(a), 0.5 + 0.5 * math.sin(a)))
    return Mesh().add(pts, [[0, 1 + i, 1 + (i + 1) % seg] for i in range(seg)], st)


def ring_up(center, r0, r1, seg=96):
    c = np.array(center, float)
    pts, st = [], []
    for i in range(seg):
        a = 2 * math.pi * i / seg
        for r in (r0, r1):
            pts.append(c + (r * math.cos(a), r * math.sin(a), 0.0))
            st.append((0.5 + 0.5 * r / r1 * math.cos(a), 0.5 + 0.5 * r / r1 * math.sin(a)))
    faces = [[2 * i, 2 * ((i + 1) % seg), 2 * ((i + 1) % seg) + 1, 2 * i + 1] for i in range(seg)]
    return Mesh().add(pts, faces, st)


def quad_up(cx, cy, z, sx, sy):
    pts = [(cx - sx / 2, cy - sy / 2, z), (cx + sx / 2, cy - sy / 2, z), (cx + sx / 2, cy + sy / 2, z),
           (cx - sx / 2, cy + sy / 2, z)]
    # text reads from the robot side (-x): image right = -y, image up = +x
    st = [(1, 0), (1, 1), (0, 1), (0, 0)]
    return Mesh().add(pts, [[0, 1, 2, 3]], st)


def screw(b, name, xy, z, mat):
    b.mesh(name, lathe([(0.0, z + 0.0012), (0.0022, z + 0.0010), (0.0028, z + 0.0004), (0.0028, z)],
                       seg=24, center=(xy[0], xy[1], 0.0)), mat)


# part-frame constants (mirror estop_twist_release.py)
TOP = 0.7475
PANEL = 0.035
ESTOP_POS = (0.300, -0.110, 0.035)
STATION_POS = (0.300, 0.000, 0.035)
MACHINE_POS = (0.520, 0.120, 0.060)
RESET_L = (0.0, -0.028)
LAMP_L = (0.0, 0.030)
PLATEN_L = (0.0, 0.0, 0.072)
PROPS = (  # (package, world xy, yaw deg): visual-only bench dressing, no collision, away from the arm's path
    ("instrument_caliper_a01", (0.300, 0.330), 80.0),
    ("consumable_spray_solvent_a01", (0.760, -0.080), 0.0),
    ("box_glove_a01", (0.770, -0.250), 90.0),
)


def build():
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

    globals().update(Gf=Gf, Sdf=Sdf, Usd=Usd, UsdGeom=UsdGeom, UsdShade=UsdShade, Vt=Vt)
    stage = Usd.Stage.CreateNew(str(OUT))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/V")
    stage.SetDefaultPrim(root.GetPrim())
    counts = {}

    # ---- yellow 1-hole e-stop station ----
    b = Body(stage, "estop_station", "body")
    yellow = b.pbr("yellow_pc", color=(0.93, 0.70, 0.04), rough=0.32, clearcoat=0.4)
    dark = b.pbr("dark_screw", color=(0.10, 0.10, 0.11), rough=0.45, metal=0.3)
    chrome = b.pbr("chrome", color=(0.80, 0.81, 0.83), rough=0.18, metal=1.0)
    cable = b.pbr("cable", color=(0.05, 0.05, 0.055), rough=0.55)
    b.mesh("enclosure", superbox((0.080, 0.080, 0.070), e=0.10), yellow)
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        screw(b, f"screw_{i}", (0.033 * sx, 0.033 * sy), PANEL, dark)
    legend = b.pbr("legend", tex=("legend_estop.png", None, None), rough=0.4)
    m = ring_up((0, 0, PANEL + 0.0012), 0.0112, 0.030)
    b.mesh("legend_plate", planar_top(m, 0, 0, 0.060), legend, smooth=False)
    b.mesh("legend_edge", lathe([(0.030, PANEL), (0.030, PANEL + 0.0012)], seg=96), yellow)
    b.mesh("bezel", lathe([(0.0112, PANEL + 0.0012), (0.0150, PANEL + 0.0014), (0.0150, PANEL + 0.0040),
                           (0.0135, PANEL + 0.0060), (0.0112, PANEL + 0.0060)], seg=64), chrome)
    b.mesh("gland", lathe([(0.0, 0.040), (0.008, 0.040), (0.008, 0.046), (0.006, 0.050), (0.0045, 0.050),
                           (0.0045, 0.054)], seg=32, axis="x", center=(0.0, 0.0, -0.012)), dark)
    # cable to the machine's terminal box (world -> part frame)
    o = np.array([ESTOP_POS[0], ESTOP_POS[1], ESTOP_POS[2]])
    path = [np.array(p) - o for p in ((0.354, -0.110, 0.023), (0.372, -0.108, 0.006), (0.384, -0.070, 0.0045),
                                      (0.378, -0.010, 0.0045), (0.362, 0.010, 0.020), (0.355, 0.012, 0.030))]
    b.mesh("cable", sweep(path, 0.0045, seg=16, samples=60), cable)
    counts["estop_station/body"] = b.n

    b = Body(stage, "estop_station", "head_press", offset=(0, 0, PANEL))
    black = b.pbr("black_pa", color=(0.08, 0.08, 0.085), rough=0.5)
    b.mesh("plunger", lathe([(0.0, PANEL + 0.0175), (0.0105, PANEL + 0.0175), (0.0105, PANEL + 0.003),
                             (0.0112, PANEL + 0.0005)], seg=48), black)
    counts["estop_station/head_press"] = b.n

    b = Body(stage, "estop_station", "head_twist", offset=(0, 0, PANEL))
    red = b.pbr("red_cap", color=(0.70, 0.03, 0.03), rough=0.28, clearcoat=0.6)
    red_top = b.pbr("red_top", tex=("cap_turn.png", None, None), rough=0.28, clearcoat=0.6)
    z0 = PANEL + 0.0164
    side = [(0.0105, z0), (0.0185, z0), (0.0198, z0 + 0.0012), (0.0200, z0 + 0.003), (0.0200, z0 + 0.0085),
            (0.0196, z0 + 0.0105)]
    b.mesh("cap_side", lathe(side, seg=96), red)
    top = [(0.0196, z0 + 0.0105), (0.0185, z0 + 0.0122), (0.016, z0 + 0.0133), (0.011, z0 + 0.0139),
           (0.005, z0 + 0.01415), (0.0, z0 + 0.0142)]
    b.mesh("cap_top", planar_top(lathe(top, seg=96), 0, 0, 0.040), red_top)
    b.mesh("cap_under", disc_up((0, 0, z0), 0.0185), red)
    counts["estop_station/head_twist"] = b.n

    # ---- grey 2-hole control station: green START + green RUN pilot lamp ----
    b = Body(stage, "control_station", "body")
    grey = b.pbr("grey_pc", color=(0.66, 0.68, 0.70), rough=0.38, clearcoat=0.2)
    dark = b.pbr("dark_screw", color=(0.10, 0.10, 0.11), rough=0.45, metal=0.3)
    chrome = b.pbr("chrome", color=(0.80, 0.81, 0.83), rough=0.18, metal=1.0)
    off_lens = b.pbr("lens_off", color=(0.025, 0.05, 0.03), rough=0.25, clearcoat=0.8)
    cable = b.pbr("cable", color=(0.05, 0.05, 0.055), rough=0.55)
    b.mesh("enclosure", superbox((0.080, 0.120, 0.070), e=0.10), grey)
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        screw(b, f"screw_{i}", (0.033 * sx, 0.053 * sy), PANEL, dark)
    for name, (lx, ly), tex in (("reset", RESET_L, "legend_start.png"), ("run", LAMP_L, "legend_run.png")):
        lg = b.pbr(f"legend_{name}", tex=(tex, None, None), rough=0.5)
        b.mesh(f"legend_{name}", quad_up(lx - 0.026, ly, PANEL + 0.0006, 0.012, 0.028), lg, smooth=False)
    b.mesh("reset_bezel", lathe([(0.0122, PANEL + 0.004), (0.0145, PANEL + 0.0035), (0.0150, PANEL + 0.0015),
                                 (0.0150, PANEL)], seg=64, center=(RESET_L[0], RESET_L[1], 0)), chrome)
    b.mesh("lamp_bezel", lathe([(0.0105, PANEL + 0.0085), (0.0140, PANEL + 0.0075), (0.0150, PANEL + 0.004),
                                (0.0150, PANEL)], seg=64, center=(LAMP_L[0], LAMP_L[1], 0)), chrome)
    b.mesh("lamp_off_lens", lathe([(0.0105, PANEL + 0.0080), (0.0090, PANEL + 0.0110), (0.0050, PANEL + 0.0128),
                                   (0.0, PANEL + 0.0132)], seg=48, center=(LAMP_L[0], LAMP_L[1], 0)), off_lens)
    b.mesh("gland", lathe([(0.0, 0.040), (0.008, 0.040), (0.008, 0.046), (0.006, 0.050), (0.0045, 0.050),
                           (0.0045, 0.054)], seg=32, axis="x", center=(0.0, 0.020, -0.012)), dark)
    o = np.array(STATION_POS)
    path = [np.array(p) - o for p in ((0.354, 0.020, 0.023), (0.362, 0.024, 0.008), (0.358, 0.040, 0.020),
                                      (0.355, 0.045, 0.030))]
    b.mesh("cable", sweep(path, 0.0045, seg=16, samples=30), cable)
    counts["control_station/body"] = b.n

    b = Body(stage, "control_station", "reset", offset=(RESET_L[0], RESET_L[1], PANEL))
    start_green = b.pbr("green_start_cap", color=(0.05, 0.55, 0.12), rough=0.25, clearcoat=0.6)
    b.mesh("cap", lathe([(0.0, PANEL + 0.0140), (0.006, PANEL + 0.0138), (0.0105, PANEL + 0.0132),
                         (0.0118, PANEL + 0.0122), (0.0120, PANEL + 0.0100), (0.0120, PANEL + 0.0010)],
                        seg=64, center=(RESET_L[0], RESET_L[1], 0)), start_green)
    counts["control_station/reset"] = b.n

    b = Body(stage, "control_station", "lamp", offset=(LAMP_L[0], LAMP_L[1], PANEL))
    lit = b.pbr("lens_lit", color=(0.45, 1.0, 0.50), rough=0.2, emissive=(1.2, 4.0, 1.4))
    b.mesh("lamp_lit_lens", lathe([(0.0108, PANEL + 0.0082), (0.0093, PANEL + 0.0114), (0.0052, PANEL + 0.0133),
                                   (0.0, PANEL + 0.0138)], seg=48, center=(LAMP_L[0], LAMP_L[1], 0)), lit)
    counts["control_station/lamp"] = b.n

    # ---- single-platen grinder-polisher (the guarded machine) + bench dressing ----
    b = Body(stage, "grinder_polisher", "body")
    paint = b.pbr("paint", color=(0.80, 0.82, 0.84), rough=0.42, clearcoat=0.2)
    trim = b.pbr("trim", color=(0.16, 0.17, 0.19), rough=0.5)
    bowl = b.pbr("bowl_abs", color=(0.90, 0.91, 0.92), rough=0.35)
    steel = b.pbr("steel", color=(0.74, 0.75, 0.77), rough=0.25, metal=1.0)
    label = b.pbr("label", tex=("machine_label.png", None, None), rough=0.5)
    b.mesh("base", superbox((0.330, 0.290, 0.120), e=0.08), paint)
    b.mesh("front_panel", superbox((0.006, 0.250, 0.070), center=(-0.164, 0.0, -0.004), e=0.2), trim)
    b.mesh("bowl", lathe([(0.030, 0.0605), (0.112, 0.0605), (0.114, 0.066), (0.121, 0.098), (0.127, 0.101),
                          (0.134, 0.100), (0.135, 0.0605)], seg=128), bowl)
    b.mesh("arm_post", lathe([(0.0, 0.062), (0.007, 0.062), (0.007, 0.112), (0.0, 0.112)], seg=24,
                             center=(0.105, 0.105, 0.0)), steel)
    b.mesh("water_arm", sweep([(0.105, 0.105, 0.110), (0.085, 0.085, 0.116), (0.050, 0.050, 0.112),
                               (0.040, 0.040, 0.100)], 0.0035, seg=16, samples=30), steel)
    b.mesh("label", quad_up(-0.110, 0.0, 0.0602, 0.035, 0.105), label, smooth=False)
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        b.mesh(f"foot_{i}", lathe([(0.0, -0.060), (0.014, -0.060), (0.014, -0.057), (0.0, -0.057)], seg=24,
                                  center=(0.140 * sx, 0.120 * sy, 0.0)), trim)
    for pkg, xy, yaw in PROPS:
        _prop(stage, b.root, MACHINE_POS, pkg, xy, yaw)
    counts["grinder_polisher/body"] = b.n

    b = Body(stage, "grinder_polisher", "platen", offset=PLATEN_L)
    alu = b.pbr("aluminium", color=(0.78, 0.79, 0.80), rough=0.3, metal=1.0)
    sic = b.pbr("sic_paper", tex=("sic_paper.png", None, None), rough=0.92, scale=6.0)
    steel = b.pbr("steel", color=(0.70, 0.71, 0.73), rough=0.22, metal=1.0)
    zp = PLATEN_L[2]
    b.mesh("platen", lathe([(0.0, zp), (0.100, zp), (0.100, zp + 0.010), (0.097, zp + 0.010)], seg=128), alu)
    b.mesh("paper", disc_up((0, 0, zp + 0.0102), 0.097, seg=128), sic)
    b.mesh("clamp_ring", lathe([(0.093, zp + 0.0102), (0.093, zp + 0.0135), (0.1015, zp + 0.0135),
                                (0.1015, zp + 0.008)], seg=128), steel)
    for i in range(4):
        a = math.pi / 2 * i
        c = (0.088 * math.cos(a), 0.088 * math.sin(a), zp + 0.0145)
        lug = superbox((0.018, 0.012, 0.006), center=(0, 0, 0), e=0.25)
        rot = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        lug.p = [tuple(rot @ np.array(q) + np.array(c)) for q in lug.p]
        b.mesh(f"lug_{i}", lug, steel)
    b.mesh("hub", lathe([(0.0, zp + 0.016), (0.012, zp + 0.016), (0.014, zp + 0.013), (0.014, zp + 0.0102)],
                        seg=6, center=(0, 0, 0)), steel)
    counts["grinder_polisher/platen"] = b.n

    stage.GetRootLayer().Save()
    return counts


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    print(textures() if "tex" in sys.argv[1:] else build())
