"""Builds the task's assets: the device (a fixed-base articulation with its two sliders), the three foot forms (rigid
bodies), their STL files for printing, and the printed scales. Run once before a trial (the task file runs it
when an asset is missing):

    python build_assets.py [OUT_DIR]        # default: ./assets ; STLs always go to ./forms

The device's outline, heel cup and slider shapes are modelled (sizing.DEVICE), not published by the maker. The
scales follow the published sizing rule (sizing.py). The forms are footform.py's own design.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import footform as F  # noqa: E402
import plan  # noqa: E402
import sizing as S  # noqa: E402
from sizing import DEVICE as D  # noqa: E402

MM = 0.001
DEVICE_COLOR = (0.08, 0.09, 0.10)  # black enamel (one of the maker's four colours)
METAL = (0.72, 0.73, 0.75)
FORM_COLOR = (0.86, 0.80, 0.70)  # natural / beige PLA
TAB_COLOR = (0.15, 0.45, 0.85)  # the printed grasp tabs (blue PLA), stuck on the sliders
FRICTION = dict(device=(0.45, 0.40), form=(0.45, 0.40))  # (static, dynamic): PLA on painted metal, modelled
CONTACT_OFFSET = 0.002
SLIDER_FRICTION_N = (1.5, 1.2)  # (static, dynamic) joint friction of each slider, N (modelled)
POINTER_MASS, BAR_MASS = 0.03, 0.08  # kg (modelled)


# -- small USD helpers ------------------------------------------------------------------------------------------------
def _stage(path: Path, root: str):
    from pxr import Usd, UsdGeom, UsdPhysics

    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        path.unlink()
    st = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(st, 1.0)
    prim = UsdGeom.Xform.Define(st, f"/{root}").GetPrim()
    st.SetDefaultPrim(prim)
    return st, prim


def _phys_material(st, path, mu):
    from pxr import UsdPhysics, UsdShade

    m = UsdShade.Material.Define(st, path)
    api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
    api.CreateStaticFrictionAttr(mu[0])
    api.CreateDynamicFrictionAttr(mu[1])
    api.CreateRestitutionAttr(0.0)
    return m


def _collide(prim, mat, approx=None):
    from pxr import Sdf, UsdPhysics, UsdShade

    UsdPhysics.CollisionAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(CONTACT_OFFSET)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
    if approx:
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approx)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")


def _box(st, path, lo, hi, color, mat=None, visible=True):
    """Axis-aligned box from corner lo to hi (mm, in the parent's frame)."""
    from pxr import Gf, UsdGeom

    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = UsdGeom.Cube.Define(st, path)
    c.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(c)
    x.AddTranslateOp().Set(Gf.Vec3d(*((lo + hi) / 2 * MM)))
    x.AddScaleOp().Set(Gf.Vec3f(*((hi - lo) * MM)))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if mat is not None:
        _collide(c.GetPrim(), mat)
    if not visible:
        c.CreatePurposeAttr("guide")
    return c


def _cyl(st, path, center, r, h, color, mat=None):
    from pxr import Gf, UsdGeom

    c = UsdGeom.Cylinder.Define(st, path)
    c.CreateRadiusAttr(r * MM)
    c.CreateHeightAttr(h * MM)
    c.CreateAxisAttr("Z")
    UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(*(np.asarray(center, float) * MM)))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if mat is not None:
        _collide(c.GetPrim(), mat)
    return c


def _mesh(st, path, verts_mm, faces, color=None, mat=None, approx=None, visible=True):
    from pxr import Gf, UsdGeom, Vt

    m = UsdGeom.Mesh.Define(st, path)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*(p * MM)) for p in np.asarray(verts_mm, float)]))
    m.CreateFaceVertexCountsAttr([3] * len(faces))
    m.CreateFaceVertexIndicesAttr([int(i) for i in np.asarray(faces).ravel()])
    m.CreateSubdivisionSchemeAttr("none")
    if color is not None:
        m.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if mat is not None:
        _collide(m.GetPrim(), mat, approx)
    if not visible:
        m.CreatePurposeAttr("guide")
    return m


def _decal(st, path, lo, hi, z, png_name):
    """A textured quad (visual only) from (u, v) corner lo to hi at height z (mm), showing png_name."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    (u0, v0), (u1, v1) = lo, hi
    m = UsdGeom.Mesh.Define(st, path)
    pts = [(u0, v0, z), (u1, v0, z), (u1, v1, z), (u0, v1, z)]
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*(np.asarray(p) * MM)) for p in pts]))
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    m.CreateSubdivisionSchemeAttr("none")
    st_pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying)
    # image x runs along +u, image y (down) runs along -v (so text reads from the lateral side, i.e. the arms)
    st_pv.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
    mat = UsdShade.Material.Define(st, path + "_mat")
    sh = UsdShade.Shader.Define(st, path + "_mat/pbr")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    tex = UsdShade.Shader.Define(st, path + "_mat/tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(png_name)
    rd = UsdShade.Shader.Define(st, path + "_mat/st")
    rd.CreateIdAttr("UsdPrimvarReader_float2")
    rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    return m


def _rigid(prim, mass, kinematic=False):
    from pxr import Sdf, UsdPhysics

    UsdPhysics.RigidBodyAPI.Apply(prim)
    if kinematic:
        prim.GetAttribute("physics:kinematicEnabled").Set(True)
    if mass is not None:
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    prim.AddAppliedSchema("PhysxRigidBodyAPI")
    prim.CreateAttribute("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(16)
    prim.CreateAttribute("physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(4)
    prim.CreateAttribute("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float).Set(0.5)


# -- printed scales ---------------------------------------------------------------------------------------------------
PX = 8  # pixels per mm


def _font(px):
    from PIL import ImageFont

    for name in ("DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"):
        try:
            return ImageFont.truetype(name, px)
        except OSError:
            pass
    return ImageFont.load_default()


def _canvas(lo, hi, bg=(20, 22, 25)):
    from PIL import Image

    (u0, v0), (u1, v1) = lo, hi
    return Image.new("RGB", (int((u1 - u0) * PX), int((v1 - v0) * PX)), bg)


def _xy(lo, hi, u, v):
    (u0, v0), (u1, v1) = lo, hi
    return (u - u0) * PX, (v1 - v) * PX  # image y grows toward -v


def scale_images(out: Path) -> dict:
    """PNG scales (white on black): length (under the foot), arch (on the guide top), width chart (lateral side)."""
    from PIL import ImageDraw

    imgs = {}
    # length scale: u 150..322 along the centre strip v -22..22, a tick at each half size, numbers at whole sizes
    lo, hi = (150.0, -22.0), (322.0, 22.0)
    im = _canvas(lo, hi)
    d = ImageDraw.Draw(im)
    f = _font(int(10 * PX))
    for k in range(10, 34):
        s = k / 2
        u = S.foot_length(s)
        if not lo[0] < u < hi[0]:
            continue
        whole, even = k % 2 == 0, k % 4 == 0
        x, _ = _xy(lo, hi, u, 0)
        d.line([(x, 0), (x, (13 if even else 9 if whole else 5) * PX)], fill="white", width=int(0.8 * PX))
        if even:
            d.text((x, 14 * PX), f"{int(s)}", font=f, fill="white", anchor="ma")
    d.text((2 * PX, 30 * PX), "LENGTH", font=_font(int(6 * PX)), fill=(220, 200, 80))
    im.save(out / "scale_length.png")
    imgs["length"] = (lo, hi)
    # arch scale on the guide's top face: u 130..250, v guide_v..guide_v+guide_w
    gv0, gv1 = D["guide_v"], D["guide_v"] + D["guide_w"]
    lo, hi = (128.0, gv0), (250.0, gv1)
    im = _canvas(lo, hi)
    d = ImageDraw.Draw(im)
    f = _font(int(6.5 * PX))
    for k in range(8, 34):
        s = k / 2
        u = S.arch_length(s)
        if not lo[0] + 2 < u < hi[0] - 2:
            continue
        x, _ = _xy(lo, hi, u, 0)
        whole, even = k % 2 == 0, k % 4 == 0
        d.line([(x, 0), (x, (5 if even else 3.5 if whole else 2) * PX)], fill="white", width=int(0.6 * PX))
        if even:
            d.text((x, 5 * PX), f"{int(s)}", font=f, fill="white", anchor="ma")
    im.save(out / "scale_arch.png")
    imgs["arch"] = (lo, hi)
    # width chart, beyond the toes where the bar's index line runs: one row per whole size 7..12 (u), a line at the
    # bar-face position of each width letter (v); read the letter at the index line on the form's size row
    lo, hi = (262.0, -74.0), (324.0, -22.0)
    im = _canvas(lo, hi)
    d = ImageDraw.Draw(im)
    fs, fl = _font(int(5 * PX)), _font(int(3.2 * PX))
    for i, s in enumerate(range(7, 13)):
        u = 267.0 + 10.6 * i
        x, _ = _xy(lo, hi, u, 0)
        d.text((x, 0.5 * PX), f"{s}", font=fs, fill=(220, 200, 80), anchor="ma")
        for letter in S.WIDTHS:
            v = D["guide_v"] - S.width(s, letter)
            if not lo[1] + 1 < v < hi[1] - 7:
                continue
            _, y = _xy(lo, hi, u, v)
            d.line([(x - 4 * PX, y), (x + 1 * PX, y)], fill="white", width=int(0.45 * PX))
            if letter in ("B", "C", "D", "E", "EE"):
                d.text((x + 1.3 * PX, y), letter, font=fl, fill="white", anchor="lm")
    im.save(out / "scale_width.png")
    imgs["width"] = (lo, hi)
    return imgs


# -- device -----------------------------------------------------------------------------------------------------------
def write_device(out: Path) -> Path:
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    path = out / "device.usda"
    st, root = _stage(path, "device")
    mat = _phys_material(st, "/device/pm", FRICTION["device"])
    UsdPhysics.ArticulationRootAPI.Apply(root)
    root.AddAppliedSchema("PhysxArticulationAPI")
    root.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    root.CreateAttribute("physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(32)
    root.CreateAttribute("physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(8)

    base = UsdGeom.Xform.Define(st, "/device/base").GetPrim()
    _rigid(base, 0.8)
    bu, bv, t = D["base_u"], D["base_v"], D["base_t"]
    _box(st, "/device/base/plate", (bu[0], bv[0], -t), (bu[1], bv[1], 0.0), DEVICE_COLOR, mat)
    # heel cup: a rounded back wall (segments) and two straight side walls
    r, w, h = D["cup_r"], D["cup_wall"], D["cup_h"]
    n = 9
    for i in range(n):
        a0 = math.pi / 2 + math.pi * i / n
        a1 = math.pi / 2 + math.pi * (i + 1) / n
        pts = []
        for a in (a0, a1):
            for rr in (r, r + w):
                for z in (0.0, h):
                    pts.append((r + rr * math.cos(a), rr * math.sin(a), z))
        import trimesh

        hull = trimesh.convex.convex_hull(np.array(pts))
        _mesh(st, f"/device/base/cup{i}", hull.vertices, hull.faces, DEVICE_COLOR, mat, "convexHull")
    for sgn, name in ((1, "cupL"), (-1, "cupR")):
        v0, v1 = sorted((sgn * r, sgn * (r + w)))
        _box(st, f"/device/base/{name}", (r, v0, 0.0), (D["cup_side_u"], v1, h), DEVICE_COLOR, mat)
    gv0, gv1 = D["guide_v"], D["guide_v"] + D["guide_w"]
    _box(st, "/device/base/guide", (D["guide_u"][0], gv0, 0.0), (D["guide_u"][1], gv1, D["guide_h"]), DEVICE_COLOR,
         mat)
    imgs = scale_images(out)
    _decal(st, "/device/base/len_scale", *imgs["length"], 0.15, "scale_length.png")
    _decal(st, "/device/base/arch_scale", *imgs["arch"], D["guide_h"] + 0.15, "scale_arch.png")
    _decal(st, "/device/base/width_chart", *imgs["width"], 0.15, "scale_width.png")
    fixed = UsdPhysics.FixedJoint.Define(st, "/device/anchor")
    fixed.CreateBody1Rel().SetTargets(["/device/base"])

    # pointer: geometry around its index line (u = 0 in its own frame), the link placed at the start
    ptr_x = UsdGeom.Xform.Define(st, "/device/pointer")
    ptr_x.AddTranslateOp().Set(Gf.Vec3d(S.PTR_START * MM, 0.0, 0.0))  # authored at its start: joint value = start
    ptr = ptr_x.GetPrim()
    _rigid(ptr, POINTER_MASS)
    cz = D["guide_h"]
    _box(st, "/device/pointer/carriage", (-14.0, gv0 - 2.0, cz), (14.0, gv1 + 2.0, cz + plan.PTR_CARR_H), METAL, mat)
    nose_c = -S.NOSE_TO_INDEX
    rn, reach = D["ptr_nose_r"], D["ptr_reach"]
    z0, z1 = D["ptr_nose_z"]
    _box(st, "/device/pointer/arm", (nose_c - 4.0, gv0 + 2.0, z0), (nose_c + 4.0, gv0 + 8.0, cz + plan.PTR_CARR_H),
         METAL, mat)
    _cyl(st, "/device/pointer/nose", (nose_c, gv0 - reach + rn, (z0 + z1) / 2), rn, z1 - z0, METAL, mat)
    _box(st, "/device/pointer/index", (-0.6, gv0 - 1.0, cz + plan.PTR_CARR_H), (0.6, gv1 + 1.0,
         cz + plan.PTR_CARR_H + 0.3), (0.9, 0.1, 0.1))  # red index line (visual)
    fz0, fz1 = plan.PTR_FIN_Z
    _box(st, "/device/pointer/flange", (-15.0, plan.PTR_FIN_V - 6.0, fz0 - plan.TAB_BASE), (15.0, plan.PTR_FIN_V + 6.0,
         fz0), TAB_COLOR, mat)
    _box(st, "/device/pointer/fin", (-12.0, plan.PTR_FIN_V - 2.5, fz0), (12.0, plan.PTR_FIN_V + 2.5, fz1), TAB_COLOR, mat)
    j = UsdPhysics.PrismaticJoint.Define(st, "/device/pointer_slide")
    j.CreateBody0Rel().SetTargets(["/device/base"])
    j.CreateBody1Rel().SetTargets(["/device/pointer"])
    j.CreateAxisAttr("X")  # joint value = index position - PTR_START (0 at the start)
    j.CreateLowerLimitAttr((D["ptr_travel"][0] - S.PTR_START) * MM)
    j.CreateUpperLimitAttr((D["ptr_travel"][1] - S.PTR_START) * MM)
    j.CreateLocalPos0Attr(Gf.Vec3f(S.PTR_START * MM, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))

    # width bar: geometry with its face on the guide face (d = 0) in its own frame, the link placed at d = start
    bar_x = UsdGeom.Xform.Define(st, "/device/bar")
    bar_x.AddTranslateOp().Set(Gf.Vec3d(0.0, -S.BAR_START * MM, 0.0))  # authored at its start: joint value = -start
    bar = bar_x.GetPrim()
    _rigid(bar, BAR_MASS)
    (u0, u1), (bz0, bz1), bt = D["bar_u"], D["bar_z"], D["bar_t"]
    _box(st, "/device/bar/beam", (u0, gv0 - bt, bz0), (u1, gv0, bz1), METAL, mat)
    _box(st, "/device/bar/index", (u1, gv0 - 1.0, 0.3), (324.0, gv0, 0.6), (0.9, 0.1, 0.1))  # index line (visual)
    fv = gv0 - bt - plan.BAR_FIN_OUT  # fin centre
    _box(st, "/device/bar/bridge", (plan.BAR_FIN_U - 15.0, fv - 8.0, bz1 - 8.0), (plan.BAR_FIN_U + 15.0, gv0 - bt, bz1),
         METAL, mat)
    _box(st, "/device/bar/flange", (plan.BAR_FIN_U - 15.0, fv - 6.0, plan.BAR_FIN_Z[0] - plan.TAB_BASE),
         (plan.BAR_FIN_U + 15.0, fv + 6.0, plan.BAR_FIN_Z[0]), TAB_COLOR, mat)
    _box(st, "/device/bar/fin", (plan.BAR_FIN_U - 12.0, fv - 2.5, plan.BAR_FIN_Z[0]),
         (plan.BAR_FIN_U + 12.0, fv + 2.5, plan.BAR_FIN_Z[1]), TAB_COLOR, mat)
    j = UsdPhysics.PrismaticJoint.Define(st, "/device/bar_slide")
    j.CreateBody0Rel().SetTargets(["/device/base"])
    j.CreateBody1Rel().SetTargets(["/device/bar"])
    j.CreateAxisAttr("Y")  # joint value = BAR_START - d (0 at the start, + closes the bar)
    j.CreateLowerLimitAttr((S.BAR_START - D["bar_travel"][1]) * MM)
    j.CreateUpperLimitAttr((S.BAR_START - D["bar_travel"][0]) * MM)
    j.CreateLocalPos0Attr(Gf.Vec3f(0, -S.BAR_START * MM, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    st.GetRootLayer().Save()
    return path


# -- forms ------------------------------------------------------------------------------------------------------------
def write_form(out: Path, spec: dict) -> tuple[Path, float]:
    from pxr import UsdGeom

    fo = F.Form(spec["L"], spec["A"], spec["W"], spec["name"])
    mass = F.print_mass_g(fo) / 1000.0
    path = out / f"form_{spec['name']}.usda"
    st, root = _stage(path, "form")
    mat = _phys_material(st, "/form/pm", FRICTION["form"])
    _rigid(root, mass)
    m = fo.mesh()
    _mesh(st, "/form/look", m.vertices, m.faces, FORM_COLOR)
    UsdGeom.Scope.Define(st, "/form/col")
    import trimesh

    for i, hv in enumerate(fo.hulls()):
        h = trimesh.convex.convex_hull(hv)
        _mesh(st, f"/form/col/h{i:02d}", h.vertices, h.faces, None, mat, "convexHull", visible=False)
    st.GetRootLayer().Save()
    stl = HERE / "forms" / f"foot_form_{spec['name']}.stl"
    stl.parent.mkdir(exist_ok=True)
    m.export(stl)
    return path, mass


def write_tab_stl() -> Path:
    """The grasp tab for real trials: a 24 x 5 mm fin on a 30 x 12 x 2 mm flange, printed in PLA and stuck to the top
    of a slider with the mounting tape (one per slider)."""
    import trimesh

    fin = trimesh.creation.box(extents=(24.0, 5.0, plan.FIN_H))
    fin.apply_translation((0, 0, plan.TAB_BASE + plan.FIN_H / 2))
    flange = trimesh.creation.box(extents=(30.0, 12.0, plan.TAB_BASE))
    flange.apply_translation((0, 0, plan.TAB_BASE / 2))
    path = HERE / "forms" / "grasp_tab.stl"
    trimesh.util.concatenate([fin, flange]).export(path)
    return path


def write_all(out: Path) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    res = {"device": write_device(out), "tab": write_tab_stl()}
    for spec in S.FORMS:
        res[spec["name"]] = write_form(out, spec)
    return res


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    out = Path(args[0]) if args else HERE / "assets"
    if "--device-only" in sys.argv:
        print(write_device(out), write_tab_stl())
    else:
        for k, v in write_all(out).items():
            print(k, v)
