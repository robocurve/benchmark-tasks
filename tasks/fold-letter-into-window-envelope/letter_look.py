"""Looks for the letter (white paper with laser-printed lines on its face side) and the #10 window envelope
(front panel with a window opening, a stamp, the open flap lying on the desk). Visual only."""

from __future__ import annotations

import numpy as np

import task as T

INK_OFF = 0.00008  # print sits this far off the paper surface, on its face side


def _ink_rects():
    """Printed rectangles in flat-sheet coords (u from the bottom edge, v from the letter's left edge), metres."""
    top = T.SX
    r = []
    r.append((top - 0.028, top - 0.018, 0.025, 0.085, (0.10, 0.20, 0.45)))  # letterhead
    r.append((top - 0.034, top - 0.031, 0.025, 0.120, (0.35, 0.35, 0.40)))  # letterhead line
    for k, ln in enumerate((0.075, 0.058, 0.066)):  # address block: three lines, 59.5-72 mm below the top
        u1 = top - 0.0595 - k * 0.0045
        r.append((u1 - 0.003, u1, 0.025, 0.025 + ln, (0.05, 0.05, 0.05)))
    u = top - 0.100
    rng = np.random.default_rng(3)
    while u > 0.030:
        c1, c2 = T.SX / 3, 2 * T.SX / 3  # keep text off the fold lines
        if abs(u - c1) > 0.006 and abs(u - c2) > 0.006 and abs(u - 0.0015 - c1) > 0.006 and abs(u - 0.0015 - c2) > 0.006:
            r.append((u - 0.0022, u, 0.025, 0.025 + float(rng.uniform(0.12, 0.166)), (0.22, 0.22, 0.22)))
        u -= 0.0062
    return r


class _Ink:
    """Bilinear sample points of the sheet for every printed rectangle (4 x 8 points each)."""

    def __init__(self, S):
        self.S = S
        uv, faces, cols = [], [], []
        nu, nv = 2, 9
        for (u0, u1, v0, v1, rgb) in _ink_rects():
            base = len(uv)
            for a in np.linspace(u0, u1, nu):
                for b in np.linspace(v0, v1, nv):
                    uv.append((a, b))
            for i in range(nu - 1):
                for j in range(nv - 1):
                    p, q, r_, s = base + i * nv + j, base + (i + 1) * nv + j, base + (i + 1) * nv + j + 1, base + i * nv + j + 1
                    faces += [(p, r_, q), (p, s, r_)]  # wound so the normal is the print side's (-z flat)
                    cols += [rgb, rgb]
        uv = np.array(uv)
        fi, fj = uv[:, 0] / S.dx, uv[:, 1] / S.dy
        i0 = np.clip(np.floor(fi).astype(int), 0, S.nx - 2)
        j0 = np.clip(np.floor(fj).astype(int), 0, S.ny - 2)
        a, b = fi - i0, fj - j0
        I = S.ids
        self.idx = np.stack([I[i0, j0], I[i0 + 1, j0], I[i0, j0 + 1], I[i0 + 1, j0 + 1]], 1)
        self.w = np.stack([(1 - a) * (1 - b), a * (1 - b), (1 - a) * b, a * b], 1)
        self.faces = np.array(faces, np.int32)
        self.cols = np.array(cols, np.float32)

    def points(self):
        S = self.S
        x = S.x
        f = S.faces
        fn = np.cross(x[f[:, 1]] - x[f[:, 0]], x[f[:, 2]] - x[f[:, 0]])
        vn = np.zeros_like(x)
        for k in range(3):
            np.add.at(vn, f[:, k], fn)
        vn /= np.linalg.norm(vn, axis=1, keepdims=True) + 1e-12
        p = (x[self.idx] * self.w[..., None]).sum(1)
        n = (vn[self.idx] * self.w[..., None]).sum(1)
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        return p - n * INK_OFF  # the blank side's normal is +z flat; print is on the other side


def _material(stage, path, rgb, rough=0.8, opacity=None):
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(float(c) for c in rgb)))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    if opacity is not None:
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _mesh(stage, path, pts, faces, mat, double=True):
    from pxr import UsdGeom, UsdShade, Vt

    m = UsdGeom.Mesh.Define(stage, path)
    m.CreateDoubleSidedAttr(double)
    m.CreateSubdivisionSchemeAttr("none")
    m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(np.asarray(pts, np.float32)))
    m.GetFaceVertexCountsAttr().Set([3] * len(faces))
    m.GetFaceVertexIndicesAttr().Set(np.asarray(faces).ravel().tolist())
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    return m


def render_letter(state, S, origin):
    from pxr import Vt
    from isaaclab.sim.utils.stage import get_current_stage

    if state is None or not state["paper"].GetPrim().IsValid():
        stage = get_current_stage()
        ink = _Ink(S)
        paper = _mesh(stage, "/World/Letter_paper", S.x + origin, S.faces,
                      _material(stage, "/World/LetterLook_paper", (0.97, 0.97, 0.96), 0.9))
        groups = {}
        for k, c in enumerate(map(tuple, ink.cols.round(3))):
            groups.setdefault(c, []).append(k)
        inks = []
        for g, (c, ks) in enumerate(groups.items()):
            m = _mesh(stage, f"/World/Letter_ink_{g}", ink.points() + origin, ink.faces[ks],
                      _material(stage, f"/World/LetterLook_ink_{g}", c, 0.7), double=False)
            inks.append(m)
        state = dict(paper=paper, ink=ink, inks=inks)
    state["paper"].GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy((S.x + origin).astype(np.float32)))
    ip = Vt.Vec3fArray.FromNumpy((state["ink"].points() + origin).astype(np.float32))
    for m in state["inks"]:
        m.GetPointsAttr().Set(ip)
    return state


def _quad(x0, x1, y0, y1, z):
    return np.array([(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]), np.array([(0, 1, 2), (0, 2, 3)])


def decorate_envelope(base):
    """Hide the plain collision box's look; draw the envelope: back panel, front panel with its window opening,
    a thin window edge, a stamp, and the open flap lying on the desk in front of the mouth."""
    from pxr import Usd, UsdGeom
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    origin = np.asarray(base.scene.env_origins[0].detach().cpu().numpy(), float)
    for p in stage.Traverse():
        ps = str(p.GetPath())
        if p.GetName() == "envelope" and ("env_0" in ps or "/envs/" not in ps):
            for q in Usd.PrimRange(p):
                if q.IsA(UsdGeom.Gprim):
                    UsdGeom.Imageable(q).MakeInvisible()
    if stage.GetPrimAtPath("/World/Envelope_front").IsValid():
        return
    paper = _material(stage, "/World/EnvelopeLook_paper", (0.93, 0.92, 0.88), 0.9)
    inner = _material(stage, "/World/EnvelopeLook_inner", (0.62, 0.66, 0.74), 0.9)  # tinted security lining
    rim = _material(stage, "/World/EnvelopeLook_rim", (0.42, 0.46, 0.52), 0.4)  # the window film's glued edge
    stamp = _material(stage, "/World/EnvelopeLook_stamp", (0.72, 0.12, 0.12), 0.7)
    cx, cy, cz = T.ENV_POS + origin
    hx, hy = T.ENV_H / 2, T.ENV_L / 2
    ztop = cz + T.ENV_TOT
    (wx0, wx1), (wy0, wy1) = T.WIN_X, T.WIN_Y
    pts, faces = [], []

    def add(q):
        p, f = q
        faces.extend((f + len(pts)).tolist())
        pts.extend(p.tolist())

    # front panel around the window opening
    add(_quad(cx - hx, cx + hx, cy - hy, cy + wy0, ztop))
    add(_quad(cx - hx, cx + hx, cy + wy1, cy + hy, ztop))
    add(_quad(cx - hx, cx + wx0, cy + wy0, cy + wy1, ztop))
    add(_quad(cx + wx1, cx + hx, cy + wy0, cy + wy1, ztop))
    # closed bottom edge and the two side edges (thin walls down to the desk)
    for (xa, xb, ya, yb) in ((cx + hx, cx + hx, cy - hy, cy + hy), (cx - hx, cx + hx, cy - hy, cy - hy),
                             (cx - hx, cx + hx, cy + hy, cy + hy)):
        b = len(pts)
        pts.extend([(xa, ya, cz), (xb, yb, cz), (xb, yb, ztop), (xa, ya, ztop)])
        faces.extend([(b, b + 1, b + 2), (b, b + 2, b + 3)])
    _mesh(stage, "/World/Envelope_front", pts, faces, paper)
    # back panel inside (seen through the window when empty, and at the mouth)
    p, f = _quad(cx - hx, cx + hx, cy - hy, cy + hy, cz + 0.0002)
    _mesh(stage, "/World/Envelope_back", p, f, inner)
    # the open flap on the desk: a trapezoid hinged at the mouth
    d = T.FLAP_D
    fp = np.array([(cx - hx, cy - hy, cz + 0.0001), (cx - hx, cy + hy, cz + 0.0001),
                   (cx - hx - d, cy + hy - 0.035, cz + 0.0001), (cx - hx - d, cy - hy + 0.035, cz + 0.0001)])
    _mesh(stage, "/World/Envelope_flap", fp, [(0, 1, 2), (0, 2, 3)], paper)
    # window edge (2.5 mm frame just above the front panel) and the stamp, top right as the envelope is read
    pts, faces = [], []
    e, z = 0.0025, ztop + 0.0001
    add(_quad(cx + wx0 - e, cx + wx1 + e, cy + wy0 - e, cy + wy0, z))
    add(_quad(cx + wx0 - e, cx + wx1 + e, cy + wy1, cy + wy1 + e, z))
    add(_quad(cx + wx0 - e, cx + wx0, cy + wy0, cy + wy1, z))
    add(_quad(cx + wx1, cx + wx1 + e, cy + wy0, cy + wy1, z))
    _mesh(stage, "/World/Envelope_window_edge", pts, faces, rim)
    p, f = _quad(cx - hx + 0.008, cx - hx + 0.033, cy + hy - 0.030, cy + hy - 0.008, ztop + 0.0001)
    _mesh(stage, "/World/Envelope_stamp", p, f, stamp)
