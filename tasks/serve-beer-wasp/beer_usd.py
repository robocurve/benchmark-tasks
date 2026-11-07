"""The customer, bar counter, beer machine, sink, mugs and wasp, written as plain-text USD (no pxr needed). All of it is ours; there are no third-party meshes.

Each part's frame: z up, x and y as the world's, origin on the table under the part. The customer faces the robot, along -x.
"""

from __future__ import annotations

import math
import os
import pathlib
from dataclasses import dataclass

import numpy as np
from beer_mesh import Mesh, UV_SCALE, block, lathe, rounded_rect, tube, usd_mesh

HERE = pathlib.Path(__file__).resolve().parent


@dataclass(frozen=True)
class Look:
    color: tuple[float, float, float] = (0.5, 0.5, 0.5)
    metallic: float = 0.0
    roughness: float = 0.5
    color_file: str = ""
    rough_file: str = ""
    normal_file: str = ""
    opacity: float = 1.0
    ior: float = 1.5


LOOKS = {
    "aluminium": Look((0.70, 0.71, 0.72), 0.9, 0.34),
    "steel": Look((0.62, 0.63, 0.64), 1.0, 0.28),
    "sink_steel": Look((0.55, 0.56, 0.57), 1.0, 0.36),
    "grate": Look((0.20, 0.20, 0.21), 0.8, 0.45),
    "black": Look((0.02, 0.02, 0.02), 0.0, 0.4),
    "drain": Look((0.03, 0.03, 0.03), 0.6, 0.5),
    "wood": Look((1.0, 1.0, 1.0), 0.0, 0.5, color_file="dark_wood.jpg"),
    "rubber": Look((0.13, 0.14, 0.15), 0.0, 0.85),
    "face_happy": Look((1.0, 1.0, 1.0), 0.0, 0.6, color_file="face_happy.png"),
    "face_upset": Look((1.0, 1.0, 1.0), 0.0, 0.6, color_file="face_upset.png"),
    "skin": Look((0.72, 0.50, 0.36), 0.0, 0.6),
    "hair": Look((0.10, 0.055, 0.035), 0.0, 0.55),
    "shirt": Look((0.05, 0.32, 0.36), 0.0, 0.8),
    "glass": Look((0.88, 0.94, 0.97), 0.0, 0.05, opacity=0.22),
    "glass_edge": Look((0.82, 0.88, 0.92), 0.0, 0.15),
    "handle_red": Look((0.55, 0.03, 0.02), 0.0, 0.35),
    "beer": Look((0.92, 0.50, 0.03), 0.0, 0.2),
    "foam": Look((0.97, 0.93, 0.82), 0.0, 0.8),
    "wasp_yellow": Look((0.95, 0.68, 0.03), 0.0, 0.35),
    "wasp_black": Look((0.02, 0.02, 0.02), 0.0, 0.35),
    "wing": Look((0.75, 0.78, 0.80), 0.0, 0.2, opacity=0.45),
}

# -- sizes and places, metres; x forward, y left, heights above the table -----------------------

HEAD = (0.58, 0.0, 0.24)  # the middle of her head
HEAD_TILT = 22.0  # degrees back from upright: she looks up at the robot, and the top camera sees her face more squarely
BAR = (0.405, 0.0, 0.13, 0.44, 0.06)  # centre x, y; sizes x, y; height: short, so that it stands between the arms and clear of their wrists
LEDGE = (0.28, 0.0, 0.12, 0.44, 0.012)  # the bar's low service shelf, holding the drip tray and the sink
BEAM_X, BEAM_Z = 0.40, 0.200  # far enough back that the wrist of an arm holding a glass at the spout clears it  # the tap's horizontal beam on its column, on the bar
SPOUT = (0.30, 0.0)
SPOUT_TIP = 0.180  # high over the tray: a glass standing on the tray is 6.75 cm short of it, and has to be held up to the tap
VALVE = (0.305, 0.19)  # the turning handle's axis: 19 cm from the spout, clear of the 16 to 17 cm where the wrists meet
HANDLE_Z = 0.180  # the middle of the handle
TRAY_TOP = 0.0175
TRAY_Y = 0.11  # the drip tray's size along y: it stops short of the sink's block
MUG_R, MUG_H = 0.035, 0.095
HANDLE_OUT = 0.075  # the mug's grip bar from its axis, along -y
MAT = (0.395, -0.17)  # a mat on the bar, in front of her
MAT_R = 0.05
SINK = (0.285, -0.14, 0.13, 0.16, 0.06)  # centre x, y; sizes x, y; height: a basin set into a wooden block of the bar, its rim flush with the block, beside the tap
MUGS = ((0.35, -0.30), (0.44, -0.355))  # where the two clean glasses stand: where a grip from the side reaches them at table level


def _scaled(m: Mesh, s) -> Mesh:
    out = Mesh()
    s = np.asarray(s, float)
    for v, n, t, f, base in zip(m.v, m.n, m.t, m.f, _bases(m)):
        nn = n / s
        nn /= np.linalg.norm(nn, axis=1, keepdims=True)
        out.add(v * s, nn, t, f - base)
    return out


def _bases(m: Mesh) -> list[int]:
    out, c = [], 0
    for v in m.v:
        out.append(c)
        c += len(v)
    return out


def ball(c=(0.0, 0.0, 0.0), r=(1.0, 1.0, 1.0), rows: int = 24, seg: int = 36) -> Mesh:
    """An ellipsoid."""
    a = np.linspace(0.0, np.pi, rows + 1)
    prof = [(float(np.sin(q)), float(-np.cos(q))) for q in a]
    v, n, t, f = [], [], [], []
    th = np.linspace(0.0, 2 * np.pi, seg + 1)
    for j, (pr, pz) in enumerate(prof):
        for i, q in enumerate(th):
            d = np.array([pr * math.cos(q), pr * math.sin(q), pz])
            v.append(d * r + c)
            nn = d / np.asarray(r)
            n.append(nn / max(np.linalg.norm(nn), 1e-9))
            t.append((q * 0.05 * UV_SCALE, j * 0.01 * UV_SCALE))
    k = seg + 1
    for j in range(rows):
        for i in range(seg):
            f += [[j * k + i, j * k + i + 1, (j + 1) * k + i + 1], [j * k + i, (j + 1) * k + i + 1, (j + 1) * k + i]]
    return Mesh().add(v, n, t, f)


def _tilt(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def head_parts(face: str):
    """Her head with the chosen face on it, and her hair: (name, mesh, look) in a frame centred on the head, facing -x."""
    W, D, H = 0.082, 0.090, 0.105
    a = np.linspace(0.0, np.pi, 41)
    th = np.linspace(0.0, 2 * np.pi, 73)
    v, n, t = [], [], []
    for p in a:
        for q in th:
            d = np.array([math.sin(p) * math.cos(q), math.sin(p) * math.sin(q), -math.cos(p)])
            z = d[2]
            jaw = 1.0 - 0.30 * max(0.0, -z) ** 2  # narrower towards the chin
            pt = np.array([d[0] * D * jaw, d[1] * W * jaw, z * H])
            v.append(pt)
            nn = d / np.array([D, W, H])
            n.append(nn / np.linalg.norm(nn))
            t.append((0.5 - pt[1] / (2 * W), 0.5 + pt[2] / (2 * H)))
    k = len(th)
    f = []
    for j in range(len(a) - 1):
        for i in range(len(th) - 1):
            f += [[j * k + i, j * k + i + 1, (j + 1) * k + i + 1], [j * k + i, (j + 1) * k + i + 1, (j + 1) * k + i]]
    head = Mesh().add(v, n, t, f)
    # The hair: a shell over the head, open in front of the face below the hairline, and a fall of hair behind and beside the neck.
    a = np.linspace(0.0, np.pi, 33)
    v, n, t, f, idx = [], [], [], [], {}
    for j, p in enumerate(a):
        for i, q in enumerate(th):
            d = np.array([math.sin(p) * math.cos(q), math.sin(p) * math.sin(q), -math.cos(p)])
            idx[(j, i)] = len(v)
            v.append(d * np.array([D + 0.007, W + 0.007, H + 0.008]))
            n.append(d)
            t.append((0.0, 0.0))
    keep = lambda j, i: not (v[idx[(j, i)]][0] < -0.012 and v[idx[(j, i)]][2] < 0.070) and v[idx[(j, i)]][2] > -0.055  # noqa: E731  (no hair in front of her face, or under her chin)
    for j in range(len(a) - 1):
        for i in range(len(th) - 1):
            q = [(j, i), (j, i + 1), (j + 1, i + 1), (j + 1, i)]
            if all(keep(*c) for c in q):
                f += [[idx[q[0]], idx[q[1]], idx[q[2]]], [idx[q[0]], idx[q[2]], idx[q[3]]]]
    hair = Mesh().add(v, n, t, f)
    hair += ball((0.035, 0.0, -0.012), (0.075, 0.100, 0.078))  # a bob, showing beside her face and stopping above the jaw: lower, it reads as a beard
    return [("head", head, f"face_{face}"), ("hair", hair, "hair")]


def customer_parts(faces=("happy", "upset")):
    """Her head with both faces on it, as meshes `head_happy` and `head_upset`: the task shows one."""
    R = _tilt(HEAD_TILT)
    out = []
    for k, face in enumerate(faces):
        for n, m, look in head_parts(face):
            if n == "head":
                out.append((f"head_{face}", m.moved(HEAD, rot=R), look))
            elif k == 0:
                out.append((n, m.moved(HEAD, rot=R), look))
    hx, hy, hz = HEAD
    neck = lathe([(0.0, hz - 0.17), (0.036, hz - 0.17), (0.033, hz - 0.06), (0.0, hz - 0.06)], 24).moved((hx + 0.012, hy, 0.0))
    out.append(("neck", neck, "skin"))
    body = ball((hx + 0.03, hy, hz - 0.225), (0.075, 0.150, 0.075))  # head and shoulders: the top camera looks down on them
    body += ball((hx + 0.04, hy, hz - 0.33), (0.085, 0.135, 0.17))
    out.append(("body", body, "shirt"))
    return out


def plain_box(cx: float, cy: float, sx: float, sy: float, z0: float, z1: float) -> Mesh:
    """A box with flat faces, tiled by UV_SCALE: for something long and thin, which the rounded plate's triangulation leaves spiked."""
    lo, hi = np.array([cx - sx / 2, cy - sy / 2, z0]), np.array([cx + sx / 2, cy + sy / 2, z1])
    out = Mesh()
    for axis in range(3):
        u, w = (axis + 1) % 3, (axis + 2) % 3
        for sign, value in ((1.0, hi[axis]), (-1.0, lo[axis])):
            pts, nrm, uv = [], [], []
            for a, b in ((lo[u], lo[w]), (hi[u], lo[w]), (hi[u], hi[w]), (lo[u], hi[w])):
                p = np.zeros(3)
                p[axis], p[u], p[w] = value, a, b
                pts.append(p)
                n = np.zeros(3)
                n[axis] = sign
                nrm.append(n)
                uv.append((a * UV_SCALE * 0.25, b * UV_SCALE * 0.25))
            out.add(pts, nrm, uv, [[0, 1, 2], [0, 2, 3]] if sign > 0 else [[0, 2, 1], [0, 3, 2]])
    return out


def counter_parts():
    """The bar, with its low service shelf towards the arms."""
    cx, cy, sx, sy, h = BAR
    lx, ly, lsx, lsy, lh = LEDGE
    drying = plain_box(0.5 * (MUGS[0][0] + MUGS[1][0]), 0.5 * (MUGS[0][1] + MUGS[1][1]), 0.20, 0.15, 0.0, 0.003)
    return [("counter", plain_box(cx, cy, sx, sy, 0.0, h) + plain_box(lx, ly, lsx, lsy, 0.0, lh), "wood"), ("drying", drying, "rubber")]


NECK_Z = 0.190
VALVE_Z = 0.155  # the valve branches down from the beam, within the arm's reach


def tray_parts():
    tx, ty = SPOUT[0] - 0.015, SPOUT[1]
    out = [("tray", block(tx, ty, 0.10, TRAY_Y, LEDGE[4], TRAY_TOP - 0.0015, r=0.008, chamfer=0.001), "sink_steel")]
    grate = Mesh()
    for i in range(-3, 4):
        grate += block(tx + i * 0.013, ty, 0.007, TRAY_Y - 0.015, TRAY_TOP - 0.0015, TRAY_TOP, r=0.001, chamfer=0.0002)
    out.append(("grate", grate, "grate"))
    return out


def machine_parts():
    """A beer tap built on the bar: a column and beam, a spout over the drip tray, and along the beam a valve, whose red handle is a separate jointed part. The spout's neck runs low, so that a gripper's palm passes over it."""
    sx, sy = SPOUT
    vx, vy = VALVE
    h = BAR[4]
    out = []
    colx, coly = BEAM_X, 0.5 * (sy + vy)
    column = lathe([(0.0, h), (0.036, h), (0.036, h + 0.006), (0.020, h + 0.014), (0.018, BEAM_Z), (0.0, BEAM_Z)], 36).moved((colx, coly, 0.0))
    beam = tube([(BEAM_X, sy - 0.018, BEAM_Z), (BEAM_X, vy + 0.018, BEAM_Z)], 0.014, 20)
    beam += ball((BEAM_X, sy - 0.018, BEAM_Z), (0.014, 0.014, 0.014), 10, 16) + ball((BEAM_X, vy + 0.018, BEAM_Z), (0.014, 0.014, 0.014), 10, 16)
    out.append(("tower", column + beam, "steel"))
    neck = tube([(BEAM_X, sy, BEAM_Z), (BEAM_X - 0.016, sy, NECK_Z + 0.004), (sx + 0.012, sy, NECK_Z), (sx + 0.002, sy, NECK_Z)], 0.0065, 14)
    neck += lathe([(0.0, SPOUT_TIP), (0.007, SPOUT_TIP), (0.008, SPOUT_TIP + 0.003), (0.0085, NECK_Z + 0.006), (0.0, NECK_Z + 0.006)], 20).moved((sx, sy, 0.0))
    out.append(("spout", neck, "steel"))
    valve = tube([(BEAM_X, vy, BEAM_Z), (BEAM_X, vy, VALVE_Z + 0.012), (BEAM_X - 0.012, vy, VALVE_Z), (vx + 0.010, vy, VALVE_Z)], 0.008, 14)
    valve += lathe([(0.0, VALVE_Z - 0.012), (0.015, VALVE_Z - 0.012), (0.015, HANDLE_Z - 0.012), (0.0, HANDLE_Z - 0.012)], 24).moved((vx, vy, 0.0))
    out.append(("valve", valve, "steel"))
    return out + tray_parts()


def handle_look_parts():
    """The red handle, about its own axis at the origin: a flat wing along y when shut, on a short stem."""
    wing = plain_box(0.0, 0.0, 0.010, 0.060, HANDLE_Z - 0.010, HANDLE_Z + 0.010)
    wing += lathe([(0.0, HANDLE_Z - 0.012), (0.009, HANDLE_Z - 0.012), (0.009, HANDLE_Z - 0.009), (0.0, HANDLE_Z - 0.009)], 16)
    return [("wing", wing, "handle_red")]


def sink_parts():
    """A bar sink, set in: a wooden block on the service shelf with a steel basin sunk into it, and a thin steel rim flush with the block's top."""
    cx, cy, sx, sy, h = SINK
    wall, shelf, t = 0.008, LEDGE[4], 0.002
    ix, iy = sx / 2 - wall, sy / 2 - wall  # the basin's half sizes
    wood = plain_box(cx + ix + wall / 2, cy, wall, sy, shelf, h) + plain_box(cx - ix - wall / 2, cy, wall, sy, shelf, h)
    wood += plain_box(cx, cy + iy + wall / 2, sx, wall, shelf, h) + plain_box(cx, cy - iy - wall / 2, sx, wall, shelf, h)
    basin = plain_box(cx + ix - t / 2, cy, t, 2 * iy, shelf, h) + plain_box(cx - ix + t / 2, cy, t, 2 * iy, shelf, h)
    basin += plain_box(cx, cy + iy - t / 2, 2 * ix, t, shelf, h) + plain_box(cx, cy - iy + t / 2, 2 * ix, t, shelf, h)
    basin += plain_box(cx, cy, 2 * ix, 2 * iy, shelf, shelf + 0.002)
    rim = plain_box(cx + ix + 0.002, cy, 0.005, 2 * iy + 0.009, h, h + 0.0015) + plain_box(cx - ix - 0.002, cy, 0.005, 2 * iy + 0.009, h, h + 0.0015)
    rim += plain_box(cx, cy + iy + 0.002, 2 * ix, 0.005, h, h + 0.0015) + plain_box(cx, cy - iy - 0.002, 2 * ix, 0.005, h, h + 0.0015)
    drain = lathe([(0.0, shelf + 0.0026), (0.016, shelf + 0.0026)], 24).moved((cx, cy, 0.0))
    return [("block", wood, "wood"), ("basin", basin, "sink_steel"), ("rim", rim, "steel"), ("drain", drain, "drain")]


def mug_parts(fill: bool = False, wasp: bool = False):
    """A glass mug on its base, axis at the origin, the handle towards -y. Not used by the task: `glass_usda` builds its glasses."""
    r, h, b = MUG_R, MUG_H, 0.006
    foam_top = h - 0.006
    wall_from = foam_top if fill else 0.002  # a full glass shows its beer: the shell stops at the foam, so that nothing greys it
    glass = lathe([(r, wall_from), (r, h - 0.004), (r - 0.003, h - 0.004), (r - 0.003, wall_from if fill else b)], 40)
    edge = lathe([(0.0, 0.0), (r - 0.002, 0.0), (r, 0.002), (r, b), (0.0, b)], 40)  # a thick base, and a rim, that show when the glass is empty
    edge += lathe([(r, h - 0.004), (r, h), (r - 0.003, h), (r - 0.003, h - 0.004)], 40)
    edge += tube([(0.0, -r + 0.003, h - 0.016), (0.0, -r - 0.018, h - 0.012), (0.0, -HANDLE_OUT, h - 0.024), (0.0, -HANDLE_OUT, 0.030), (0.0, -r - 0.018, 0.018), (0.0, -r + 0.003, 0.016)], 0.006, 14)
    out = [("glass", glass, "glass"), ("edge", edge, "glass_edge")]
    if fill:
        out.append(("beer", lathe([(0.0, b), (r - 0.0002, b), (r - 0.0002, h - 0.022), (0.0, h - 0.022)], 36), "beer"))
        out.append(("foam", lathe([(r - 0.0002, h - 0.022), (r - 0.0002, foam_top), (0.0, foam_top + 0.002)], 36), "foam"))
    if wasp:
        out += wasp_parts((0.006, 0.004, h - 0.0035), 0.6)
    return out


POUR_TILT = 55.0  # degrees


def pouring_mug_parts():
    """The wasp's mug over the sink, tipped away from its handle, with the beer running out over its rim into the basin. For a still: in the origin of the sink's middle, 12 cm up. Not used by the task."""
    a = math.radians(POUR_TILT)
    c, s_ = math.cos(a), math.sin(a)
    rot = np.array([[1.0, 0.0, 0.0], [0.0, c, -s_], [0.0, s_, c]])
    z0 = 0.095
    out = []
    for name, mesh, look in mug_parts():
        out.append((name, mesh.moved((0.0, 0.0, z0), rot=rot), look))
    r, h = MUG_R - 0.0002, MUG_H
    beer = lathe([(0.0, 0.006), (r, 0.006), (r, 0.040), (0.0, 0.040)], 36)
    out.append(("beer", beer.moved((0.0, 0.0, z0), rot=rot), "beer"))
    lip = rot @ np.array([0.0, MUG_R - 0.002, h]) + np.array([0.0, 0.0, z0])
    stream = tube([tuple(lip), (0.0, float(lip[1]) + 0.004, float(lip[2]) - 0.01), (0.0, float(lip[1]) + 0.012, 0.03), (0.0, float(lip[1]) + 0.014, LEDGE[4] + 0.003)], 0.006, 12)
    out.append(("stream", stream, "beer"))
    return out


def wasp_parts(at, yaw: float):
    """A dead wasp 2.2 cm long, on its side of the foam."""
    yellow, black = Mesh(), Mesh()
    black += ball((0.008, 0.0, 0.002), (0.0045, 0.0035, 0.0035), 10, 14)  # thorax
    black += ball((0.0135, 0.0, 0.002), (0.0028, 0.0028, 0.0026), 8, 12)  # head
    for i, x in enumerate(np.linspace(0.002, -0.0095, 6)):
        r = 0.0034 * (1.0 - 0.6 * (i / 5.0) ** 2)
        (yellow if i % 2 == 0 else black).__iadd__(ball((float(x), 0.0, 0.002), (0.0026, r, r), 8, 12))
    wings = Mesh()
    for s in (1.0, -1.0):
        wings += ball((0.0, s * 0.0065, 0.0052), (0.0085, 0.0034, 0.0004), 6, 12)
    return [("wasp_y", yellow.moved(at, yaw=yaw), "wasp_yellow"), ("wasp_k", black.moved(at, yaw=yaw), "wasp_black"), ("wings", wings.moved(at, yaw=yaw), "wing")]


def mat_parts():
    h = BAR[4]
    return [("mat", lathe([(0.0, h), (MAT_R, h), (MAT_R, h + 0.003), (0.0, h + 0.003)], 56).moved((MAT[0], MAT[1], 0.0)), "rubber")]


def _looks(root: str, used) -> str:
    out = ""
    for name in sorted(used):
        look = LOOKS[name]
        base = f"/{root}/Looks/{name}"
        c = look.color
        nodes = ""

        def reader(shader: str, file: str, space: str, extra: str, output: str) -> str:
            return f"""
            def Shader "{shader}"
            {{
                uniform token info:id = "UsdUVTexture"
                asset inputs:file = @{HERE / "textures" / file}@
                float2 inputs:st.connect = <{base}/st.outputs:result>
                token inputs:sourceColorSpace = "{space}"
                token inputs:wrapS = "repeat"
                token inputs:wrapT = "repeat"{extra}
                {output}
            }}"""

        if look.color_file:
            tint = ""
            color_line = f"color3f inputs:diffuseColor.connect = <{base}/color.outputs:rgb>"
            nodes += reader("color", look.color_file, "sRGB", tint, "float3 outputs:rgb")
        else:
            color_line = f"color3f inputs:diffuseColor = ({c[0]}, {c[1]}, {c[2]})"
        if look.rough_file:
            rough_line = f"float inputs:roughness.connect = <{base}/rough.outputs:r>"
            nodes += reader("rough", look.rough_file, "raw", "", "float outputs:r")
        else:
            rough_line = f"float inputs:roughness = {look.roughness}"
        normal_line = ""
        if look.normal_file:
            normal_line = f"normal3f inputs:normal.connect = <{base}/normal.outputs:rgb>"
            remap = """
                float4 inputs:scale = (0.7, 0.7, 2, 1)
                float4 inputs:bias = (-0.35, -0.35, -1, 0)"""
            nodes += reader("normal", look.normal_file, "raw", remap, "float3 outputs:rgb")
        st = f"""
            def Shader "st"
            {{
                uniform token info:id = "UsdPrimvarReader_float2"
                string inputs:varname = "st"
                float2 outputs:result
            }}""" if nodes else ""
        out += f"""
        def Material "{name}"
        {{
            token outputs:surface.connect = <{base}/Shader.outputs:surface>
            def Shader "Shader"
            {{
                uniform token info:id = "UsdPreviewSurface"
                {color_line}
                float inputs:metallic = {look.metallic}
                float inputs:opacity = {look.opacity}
                float inputs:ior = {look.ior}
                {rough_line}
                {normal_line}
                token outputs:surface
            }}{st}{nodes}
        }}"""
    return f"""
    def Scope "Looks"
    {{{out}
    }}"""


def _physics_materials() -> str:
    def one(name: str, mu: float) -> str:
        return f"""
    def Material "{name}" (
        prepend apiSchemas = ["PhysicsMaterialAPI"]
    )
    {{
        float physics:staticFriction = {mu}
        float physics:dynamicFriction = {mu}
        float physics:restitution = 0
    }}"""

    return one("Slick", 0.25) + one("Grip", 1.0)


COLLIDER = """prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI", "MaterialBindingAPI"]"""


def _quat_z(deg: float) -> str:
    h = math.radians(deg) / 2
    return f"({math.cos(h):.6f}, 0, 0, {math.sin(h):.6f})"


def _offsets(contact: float) -> str:
    return f"""float physxCollision:contactOffset = {contact}
        float physxCollision:restOffset = 0"""


def box(root: str, name: str, centre, size, yaw_deg: float = 0.0, material: str = "Slick", contact: float = 0.002) -> str:
    c, s = centre, size
    return f"""
    def Cube "{name}" (
        {COLLIDER}
    )
    {{
        rel material:binding:physics = </{root}/{material}>
        {_offsets(contact)}
        uniform token purpose = "guide"
        double size = 1
        double3 xformOp:translate = ({c[0]:.6f}, {c[1]:.6f}, {c[2]:.6f})
        quatf xformOp:orient = {_quat_z(yaw_deg)}
        float3 xformOp:scale = ({s[0]:.6f}, {s[1]:.6f}, {s[2]:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
    }}"""


def cyl(root: str, name: str, x: float, y: float, z0: float, z1: float, r: float, material: str = "Slick", contact: float = 0.002) -> str:
    return f"""
    def Cylinder "{name}" (
        {COLLIDER}
    )
    {{
        rel material:binding:physics = </{root}/{material}>
        {_offsets(contact)}
        uniform token purpose = "guide"
        uniform token axis = "Z"
        double height = {z1 - z0:.6f}
        double radius = {r:.6f}
        double3 xformOp:translate = ({x:.6f}, {y:.6f}, {0.5 * (z0 + z1):.6f})
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}"""



def _collect(parts: list[tuple[str, Mesh, str]], root: str) -> str:
    return "".join(usd_mesh(n, m, f"/{root}/Looks/{look}", "        ") for n, m, look in parts)


def _write(path: str, text: str) -> str:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)
    return path


def _header(root: str) -> str:
    return f"""#usda 1.0
(
    defaultPrim = "{root}"
    metersPerUnit = 1
    upAxis = "Z"
)
"""


def _static(path: str, root: str, colliders: str, parts) -> str:
    return _write(path, f"""{_header(root)}
def Xform "{root}"
{{{_physics_materials()}{_looks(root, {p[2] for p in parts})}
{colliders}
    def Xform "visual"
    {{{_collect(parts, root)}
    }}
}}
""")


def write_part(path: str, root: str, parts) -> str:
    """A fixed part with one small collider, for stills."""
    return _static(path, root, box(root, "base", (0.0, 0.0, -0.0005), (0.01, 0.01, 0.001)), parts)


# -- parts with physics: each function returns one USD file's text, for kitchenkit.Part ------------


def _text(root: str, colliders: str, parts, root_api: str = "", attrs: str = "", extra: str = "", looks=()) -> str:
    api = f"(\n    prepend apiSchemas = [{root_api}]\n)" if root_api else ""
    used = {p[2] for p in parts} | set(looks)
    return f"""{_header(root)}
def Xform "{root}" {api}
{{{attrs}{_physics_materials()}{_looks(root, used)}
{colliders}
    def Xform "visual"
    {{{_collect(parts, root)}{extra}
    }}
}}
"""


def bar_usda() -> str:
    """The bar with its shelf, sink, tap, drip tray, pad and drying mat: everything fixed that is touched. Origin: on the table at x = 0, y = 0."""
    root = "Bar"
    cx, cy, sx, sy, h = BAR
    lx, ly, lsx, lsy, lh = LEDGE
    col = box(root, "bar", (cx, cy, h / 2), (sx, sy, h)) + box(root, "shelf", (lx, ly, lh / 2), (lsx, lsy, lh))
    tx, ty = SPOUT[0] - 0.015, SPOUT[1]
    col += box(root, "tray", (tx, ty, 0.5 * (lh + TRAY_TOP)), (0.10, TRAY_Y, TRAY_TOP - lh), material="Grip")
    px, py, psx, psy, ph = SINK
    wall, shelf = 0.008, lh
    ix, iy = psx / 2 - wall, psy / 2 - wall
    col += box(root, "sink_x1", (px + ix + wall / 2, py, 0.5 * (shelf + ph)), (wall, psy, ph - shelf)) + box(root, "sink_x2", (px - ix - wall / 2, py, 0.5 * (shelf + ph)), (wall, psy, ph - shelf))
    col += box(root, "sink_y1", (px, py + iy + wall / 2, 0.5 * (shelf + ph)), (psx, wall, ph - shelf)) + box(root, "sink_y2", (px, py - iy - wall / 2, 0.5 * (shelf + ph)), (psx, wall, ph - shelf))
    col += box(root, "sink_floor", (px, py, shelf + 0.001), (2 * ix, 2 * iy, 0.002), material="Grip")
    col += cyl(root, "pad", MAT[0], MAT[1], h, h + 0.003, MAT_R, material="Grip")
    col += cyl(root, "column", BEAM_X, 0.5 * (SPOUT[1] + VALVE[1]), h, BEAM_Z, 0.020) + box(root, "beam", (BEAM_X, 0.5 * (SPOUT[1] + VALVE[1]), BEAM_Z), (0.028, VALVE[1] - SPOUT[1] + 0.05, 0.028))
    col += box(root, "neck", (0.5 * (SPOUT[0] + BEAM_X), SPOUT[1], NECK_Z), (BEAM_X - SPOUT[0], 0.014, 0.013))
    col += box(root, "valve_stub", (0.5 * (VALVE[0] + BEAM_X), VALVE[1], VALVE_Z), (BEAM_X - VALVE[0], 0.016, 0.016)) + cyl(root, "valve_body", VALVE[0], VALVE[1], VALVE_Z - 0.012, HANDLE_Z - 0.012, 0.015)
    col += box(root, "valve_drop", (BEAM_X, VALVE[1], 0.5 * (VALVE_Z + BEAM_Z)), (0.016, 0.016, BEAM_Z - VALVE_Z))
    parts = counter_parts() + sink_parts() + machine_parts() + mat_parts()
    return _text(root, col, parts)


def handle_usda(drive) -> str:
    """The red handle, on a revolute joint about z through its origin: 0 degrees shut, along y; +90 degrees open, across the beam. Origin: on the table, on the handle's axis."""
    root = "Handle"
    col = box(root, "wing_col", (0.0, 0.0, HANDLE_Z), (0.010, 0.060, 0.020), material="Grip").replace('uniform token purpose = "guide"', 'token visibility = "invisible"')
    return f"""{_header(root)}
def Xform "{root}" (
    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]
)
{{
    bool physxArticulation:enabledSelfCollisions = 0
    int physxArticulation:solverPositionIterationCount = 16
    int physxArticulation:solverVelocityIterationCount = 1
{_physics_materials()}{_looks(root, {"handle_red"})}
    def Xform "frame" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    )
    {{
        float physics:mass = 1.0
        float3 physics:diagonalInertia = (0.01, 0.01, 0.01)
    }}

    def Xform "wing" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]
    )
    {{
        float physics:mass = 0.05
        float3 physics:diagonalInertia = (0.00002, 0.00002, 0.00002)
        bool physxRigidBody:disableGravity = 1
{_collect(handle_look_parts(), root)}
{col}
    }}

    def PhysicsFixedJoint "world_joint"
    {{
        rel physics:body1 = </{root}/frame>
    }}

    def PhysicsRevoluteJoint "turn_joint" (
        prepend apiSchemas = ["PhysicsDriveAPI:angular"]
    )
    {{
        uniform token drive:angular:physics:type = "force"
        float drive:angular:physics:stiffness = 0
        float drive:angular:physics:damping = {drive[1]}
        float drive:angular:physics:maxForce = {drive[2]}
        rel physics:body0 = </{root}/frame>
        rel physics:body1 = </{root}/wing>
        uniform token physics:axis = "Z"
        point3f physics:localPos0 = (0, 0, 0)
        quatf physics:localRot0 = (1, 0, 0, 0)
        point3f physics:localPos1 = (0, 0, 0)
        quatf physics:localRot1 = (1, 0, 0, 0)
        float physics:lowerLimit = 0
        float physics:upperLimit = 90
    }}
}}
"""


def _hidden(text: str, prim: str) -> str:
    """The text with this prim hidden from its first frame: what the task shows later must not show before the task has run."""
    key = f'def {prim}'
    i = text.index(key)
    j = text.index("{", i)
    return text[:j + 1] + '\n        token visibility = "invisible"' + text[j + 1:]


def customer_usda() -> str:
    """Happy from the first frame: the upset face is hidden in the file itself."""
    root = "Customer"
    return _hidden(_text(root, box(root, "base", (0.6, 0.0, -0.0005), (0.01, 0.01, 0.001)), customer_parts()), 'Mesh "head_upset"')


BEER_FROM = 0.006  # the glass's thick base
BEER_TO = MUG_H - 0.020  # the beer's height, full
FOAM = 0.014
FILL_STEPS = 8


def glass_usda() -> str:
    """A pint glass: a body with a thick base and a rim, one cylinder collider, and hidden beer, foam and wasp that the task shows. The beer lies a hair outside the glass's surface: real transparency greys what is behind it. Origin: the base's middle."""
    root = "Glass"
    r, h = MUG_R, MUG_H
    shell = lathe([(r - 0.0012, 0.002), (r - 0.0012, h - 0.004), (r - 0.003, h - 0.004), (r - 0.003, BEER_FROM)], 40)
    edge = lathe([(0.0, 0.0), (r - 0.002, 0.0), (r, 0.002), (r, BEER_FROM), (0.0, BEER_FROM)], 40) + lathe([(r, h - 0.004), (r, h), (r - 0.003, h), (r - 0.003, h - 0.004)], 40)
    def group(name, inner):
        return f"""
        def Xform "{name}"
        {{
            token visibility = "invisible"{inner}
        }}"""

    extra = ""
    for k in range(1, FILL_STEPS + 1):  # one prim per fill level, shown in turn
        top = BEER_FROM + (BEER_TO - BEER_FROM) * k / FILL_STEPS
        beer = lathe([(0.0, BEER_FROM), (r - 0.0002, BEER_FROM), (r - 0.0002, top), (0.0, top)], 36)
        foam = lathe([(r - 0.0002, top), (r - 0.0002, top + FOAM), (0.0, top + FOAM + 0.002)], 36)
        extra += group(f"level_{k}", usd_mesh(f"beer_{k}", beer, f"/{root}/Looks/beer", "            ") + usd_mesh(f"foam_{k}", foam, f"/{root}/Looks/foam", "            "))
    wasp = wasp_parts((0.006, 0.004, BEER_TO + FOAM + 0.0005), 0.6)
    extra += group("wasp", "".join(usd_mesh(n, m, f"/{root}/Looks/{look}", "            ") for n, m, look in wasp))
    col = cyl(root, "body", 0.0, 0.0, 0.0, h, r, material="Grip")
    return _text(root, col, [("shell", shell, "glass"), ("edge", edge, "glass_edge")], root_api='"PhysicsRigidBodyAPI", "PhysicsMassAPI"', extra=extra, looks=("beer", "foam", "wasp_yellow", "wasp_black", "wing"))


def wasp_fly_usda() -> str:
    """The wasp in flight, wings out, about its own origin: the task moves it. Nothing collides."""
    root = "WaspFly"
    parts = wasp_parts((0.0, 0.0, 0.0), 0.0)
    return _hidden(_text(root, box(root, "base", (0.0, 0.0, -2.0), (0.001, 0.001, 0.001)), parts), 'Xform "visual"')


def stream_usda() -> str:
    """The beer running from the spout down to the tray, hidden until the tap is open. Inside a filling glass, it is hidden by the beer. Origin: on the table."""
    root = "Stream"
    stream = lathe([(0.0, TRAY_TOP), (0.0045, TRAY_TOP), (0.0045, SPOUT_TIP), (0.0, SPOUT_TIP)], 12).moved((SPOUT[0], SPOUT[1], 0.0))
    return _hidden(_text(root, box(root, "base", (0.0, 0.0, -2.0), (0.001, 0.001, 0.001)), [("stream", stream, "beer")]), 'Xform "visual"')
