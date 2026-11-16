"""Generate the generic, metre-scale visual layers from the task's part geometry.

Run with NumPy and the arena runtime available. The layers add no colliders.
"""

import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import reacher_trigger_retrieve as task

def lin(c):
    return tuple(((x / 12.92) if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4) for x in c)


# -- mesh building ---------------------------------------------------------------------------------------------------


class MeshBuf:
    def __init__(self):
        self.pts, self.nrm, self.counts, self.idx = [], [], [], []

    def add(self, pts, nrm, faces):
        base = len(self.pts)
        self.pts += pts
        self.nrm += nrm
        for f in faces:
            self.counts.append(len(f))
            self.idx += [base + i for i in f]


def fmt3(v):
    return "(" + ", ".join(f"{x:.6g}" for x in v) + ")"


def mesh_usda(name, buf, mat, uv_rect=None):
    """uv_rect = (x0, x1, y0, y1): planar st from the mesh x, y (s along x, t along y), matching tex_from_view()."""
    pts = ", ".join(fmt3(p) for p in buf.pts)
    nrm = ", ".join(fmt3(n) for n in buf.nrm)
    lo = [min(p[i] for p in buf.pts) for i in range(3)]
    hi = [max(p[i] for p in buf.pts) for i in range(3)]
    st = ""
    if uv_rect:
        x0, x1, y0, y1 = uv_rect
        uv = ", ".join(f"({(p[0] - x0) / (x1 - x0):.6g}, {(p[1] - y0) / (y1 - y0):.6g})" for p in buf.pts)
        st = f'        texCoord2f[] primvars:st = [{uv}] (interpolation = "vertex")\n'
    return (f'    def Mesh "{name}" (prepend apiSchemas = ["MaterialBindingAPI"])\n    {{\n'
            f"        float3[] extent = [{fmt3(lo)}, {fmt3(hi)}]\n"
            f"        int[] faceVertexCounts = [{', '.join(map(str, buf.counts))}]\n"
            f"        int[] faceVertexIndices = [{', '.join(map(str, buf.idx))}]\n"
            f"        point3f[] points = [{pts}]\n"
            f'        normal3f[] normals = [{nrm}] (interpolation = "vertex")\n'
            f"{st}"
            '        uniform token subdivisionScheme = "none"\n'
            "        uniform bool doubleSided = 1\n"
            f"        rel material:binding = </Visual/Looks/{mat}>\n    }}\n")


def material_usda(name, color=None, rough=0.4, metal=0.0, clearcoat=0.0, clearcoat_rough=0.05, tex=None):
    nodes, inputs = [], []
    if tex:
        nodes.append('        def Shader "st"\n        {\n            uniform token info:id = "UsdPrimvarReader_float2"\n'
                     '            string inputs:varname = "st"\n            float2 outputs:result\n        }\n')
        nodes.append(f'        def Shader "alb"\n        {{\n            uniform token info:id = "UsdUVTexture"\n'
                     f"            asset inputs:file = @{tex}@\n"
                     '            token inputs:sourceColorSpace = "sRGB"\n'
                     f'            float2 inputs:st.connect = </Visual/Looks/{name}/st.outputs:result>\n'
                     '            token inputs:wrapS = "clamp"\n            token inputs:wrapT = "clamp"\n'
                     "            float3 outputs:rgb\n        }\n")
        inputs.append(f"            color3f inputs:diffuseColor.connect = </Visual/Looks/{name}/alb.outputs:rgb>\n")
    else:
        inputs.append(f"            color3f inputs:diffuseColor = {fmt3(lin(color))}\n")
    inputs.append(f"            float inputs:roughness = {rough}\n")
    inputs.append(f"            float inputs:metallic = {metal}\n            float inputs:clearcoat = {clearcoat}\n"
                  f"            float inputs:clearcoatRoughness = {clearcoat_rough}\n"
                  "            int inputs:useSpecularWorkflow = 0\n")
    return (f'        def Material "{name}"\n        {{\n'
            f"            token outputs:surface.connect = </Visual/Looks/{name}/pbr.outputs:surface>\n"
            f'            def Shader "pbr"\n            {{\n                uniform token info:id = "UsdPreviewSurface"\n'
            + "".join("    " + i for i in inputs)
            + "                token outputs:surface\n            }\n"
            + "".join(nodes)
            + "        }\n")


def plastic(name, c, rough=0.4, cc=0.15, **kw):
    return material_usda(name, c, rough=rough, clearcoat=cc, clearcoat_rough=0.12, **kw)


def write_layer(path, meshes, mats, extra=""):
    path.write_text('#usda 1.0\n(\n    defaultPrim = "Visual"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
                    f'def Xform "Visual"\n{{\n{"".join(meshes)}{extra}\n    def Scope "Looks"\n    {{\n'
                    f'{"".join(mats)}    }}\n}}\n')


# -- rounded boxes ---------------------------------------------------------------------------------------------------


def _axis_samples(h, r, nr, cuts=()):
    a = np.linspace(0.5 * math.pi, 0.0, nr)
    lo = [-h + r * (1 - math.sin(t)) for t in a]
    s = sorted(set(round(v, 7) for v in lo + [-x for x in lo] + [c for c in cuts if -h + r < c < h - r]))
    return np.array(s)


def rbox(buf, c, size, r=0.001, nr=3, yaw=0.0, pitch=0.0, faces=None, cuts=None, drop=None, warp=None):
    """Box centred at c with every edge rounded by r (a subdivided cube pushed onto the rounded surface).

    faces: set of (axis, sign) to emit (default all six). cuts: {axis: [local coords]} extra grid lines.
    drop(axis, sign, local_centre) -> True leaves that quad out (holes). warp(p, n) -> (p, n) in local coords.
    """
    h = np.asarray(size, float) / 2
    r = max(1e-5, min(r, 0.999 * float(h.min())))
    cuts = cuts or {}
    ax = [_axis_samples(h[i], r, nr, cuts.get(i, ())) for i in range(3)]
    rot = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]) @ \
        np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    inner = h - r
    for axis in range(3):
        for sign in (-1.0, 1.0):
            if faces is not None and (axis, int(sign)) not in faces:
                continue
            u, v = [i for i in range(3) if i != axis]
            pts, nrm, quads, loc = [], [], [], []
            nu, nv = len(ax[u]), len(ax[v])
            for a_ in ax[u]:
                for b_ in ax[v]:
                    q = np.zeros(3)
                    q[axis], q[u], q[v] = sign * h[axis], a_, b_
                    cl = np.clip(q, -inner, inner)
                    d = q - cl
                    n = np.linalg.norm(d)
                    d = d / n if n > 1e-12 else np.eye(3)[axis] * sign
                    p = cl + r * d
                    if warp:
                        p, d = warp(p, d)
                    loc.append(q)
                    pts.append(tuple(rot @ p + np.asarray(c, float)))
                    nrm.append(tuple(rot @ d))
            for i in range(nu - 1):
                for j in range(nv - 1):
                    a0, a1, b0, b1 = i * nv + j, (i + 1) * nv + j, (i + 1) * nv + j + 1, i * nv + j + 1
                    if drop and drop(axis, int(sign), (loc[a0] + loc[b0]) / 2):
                        continue
                    quads.append((a0, a1, b0, b1) if sign > 0 else (a0, b1, b0, a1))
            buf.add(pts, nrm, quads)


def quad(buf, x0, x1, y0, y1, z):
    buf.add([(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)], [(0, 0, 1)] * 4, [(0, 1, 2, 3)])


def disc(buf, cx, cy, z, rad, n=40):
    pts = [(cx, cy, z)] + [(cx + rad * math.cos(2 * math.pi * k / n), cy + rad * math.sin(2 * math.pi * k / n), z)
                           for k in range(n)]
    buf.add(pts, [(0, 0, 1)] * len(pts), [(0, 1 + k, 1 + (k + 1) % n) for k in range(n)])


def cyl(buf, cx, cy, z0, z1, rad, n=32):
    pts, nrm, faces = [], [], []
    for k in range(n):
        a = 2 * math.pi * k / n
        for z in (z0, z1):
            pts.append((cx + rad * math.cos(a), cy + rad * math.sin(a), z))
            nrm.append((math.cos(a), math.sin(a), 0.0))
    for k in range(n):
        k1 = (k + 1) % n
        faces.append((2 * k, 2 * k1, 2 * k1 + 1, 2 * k + 1))
    buf.add(pts, nrm, faces)
    disc(buf, cx, cy, z1, rad, n)


# -- textures --------------------------------------------------------------------------------------------------------
# A texture is drawn in the operator's view of the part (looking down, keyboard toward the operator): image rows run

MATERIALS = {
    "shell": ((.19, .205, .22), .36, .0),
    "nylon": ((.13, .15, .17), .45, .0),
    "shaft": ((.34, .375, .41), .29, .72),
    "pin": ((.53, .55, .57), .31, .62),
    "trigger": ((.72, .125, .085), .42, .0),
    "rubber": ((.25, .28, .31), .80, .0),
    "key": ((.37, .40, .42), .66, .0),
    "ink": ((.87, .89, .90), .77, .0),
    "mat": ((.35, .53, .57), .83, .0),
    "edge": ((.76, .83, .84), .74, .0),
}


def box_meshes(name, part, palette):
    meshes = []
    for i, (box, mat) in enumerate(zip(part.boxes, palette, strict=True)):
        buf = MeshBuf()
        yaw = 2 * math.atan2(box.quat[2], box.quat[3])
        if name == "reacher" and i == 0:
            # Extruded aluminium tube, with the internal rod concealed.
            buf = MeshBuf()
            half = box.size[0] / 2
            cyl(buf, 0, 0, -half, half, .0081)
            buf.pts = [(z+box.at[0], y+box.at[1], -x+box.at[2]) for x,y,z in buf.pts]
            buf.nrm = [(z,y,-x) for x,y,z in buf.nrm]
        else:
            rbox(buf, box.at, box.size, r=min(box.size)*.40, yaw=yaw)
        meshes.append(mesh_usda(f"Part{i}", buf, mat))
    return meshes


def make():
    palette = {
        task.body: ("shaft", "shell", "shell", "nylon", "rubber"),
        task.trigger: ("trigger", "trigger", "nylon"),
        task.jaw: ("nylon", "rubber", "nylon"),
        task.remote: ("shell",),
        task.mat: ("mat",),
    }
    for name, materials in palette.items():
        part = task.kit.parts[name]
        meshes = box_meshes(name, part, materials)
        if name in (task.body, task.jaw):
            x, y = (.083, 0.0) if name == task.body else (0.0, 0.0)
            pivot = MeshBuf()
            cyl(pivot, x, y, .0065, .0104, .0044)
            meshes.append(mesh_usda("Pivot", pivot, "pin"))
        if name == task.remote:
            keys, power, pad, legends = MeshBuf(), MeshBuf(), MeshBuf(), MeshBuf()
            for y in (-.046, -.036, -.026, .016, .026, .039, .050):
                for x in (-.011, 0, .011):
                    rbox(keys, (x, y, .0098), (.0066, .0059, .0018), r=.0008, nr=2)
                    quad(legends, x-.0011, x+.0011, y-.00045, y+.00045, .01075)
            cyl(power, -.012, .058, .0095, .0110, .0030)
            cyl(pad, 0, -.005, .0093, .0114, .0102)
            cyl(keys, 0, -.005, .0113, .0122, .0056)
            for b, n, m in ((pad, "DirectionRing", "nylon"), (keys, "Keys", "key"),
                            (power, "Power", "trigger"), (legends, "Legends", "ink")):
                meshes.append(mesh_usda(n, b, m))
        if name == task.mat:
            stripes = MeshBuf()
            w, h, line = task.MAT_SIZE[0] - .002, task.MAT_SIZE[1] - .005, .0015
            for x in (-w/2, w/2-line):
                quad(stripes, x, x+line, -h/2, h/2, .00206)
            for y in (-h/2, h/2-line):
                quad(stripes, -w/2+line, w/2-line, y, y+line, .00206)
            meshes.append(mesh_usda("StitchedBorder", stripes, "edge"))
        mats = [material_usda(n, c, rough=r, metal=m) for n, (c, r, m) in MATERIALS.items() if any(f"Looks/{n}>" in mesh for mesh in meshes)]
        write_layer(HERE / f"{name}.usda", meshes, mats)


if __name__ == "__main__":
    make()
