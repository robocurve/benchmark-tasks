"""Writes assets/*.usda and assets/textures/*.png from layout.py. Plain Python with NumPy and Pillow; no simulator.

    python build_assets.py

- ``room.usda``: the chrome wire shelf on the station table (four posts on their feet and the shelf), standing on two
  2x4 battens, as one fixed body with box colliders.
- ``soup_case.usda``, ``new_case.usda``: the cases, each a printed box mesh over its colliders.
- ``hand_hole_template.svg``: a 1:1 template for cutting N's hand holes.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import layout as L  # noqa: E402
import usdkit as u  # noqa: E402

ASSETS = HERE / "assets"
TEX = ASSETS / "textures"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_REG = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def srgb(c):
    """sRGB 0..1 to linear, for UsdPreviewSurface colours."""
    c = np.asarray(c, float)
    return tuple(float(x) for x in np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4))


LOOKS = {
    "timber": u.Look(color=srgb((0.80, 0.66, 0.46)), roughness=0.8),        # untreated pine
    "chrome": u.Look(color=srgb((0.80, 0.81, 0.83)), metallic=0.9, roughness=0.25),
    "foot": u.Look(color=srgb((0.15, 0.15, 0.16)), roughness=0.6),
    "soup_case": u.Look(texture="soup_case.png", roughness=0.85),
    "new_case": u.Look(texture="rice_case.png", roughness=0.85),
}


# -- textures -------------------------------------------------------------------------------------------------------------
CELL = 512
# atlas cells (column, row) for each face: +x far end, -x near end, +y left side, -y right side, +z top, -z bottom
CELLS = {"+x": (0, 0), "-x": (1, 0), "+y": (2, 0), "-y": (0, 1), "+z": (1, 1), "-z": (2, 1)}


def _kraft(w, h, seed):
    from PIL import Image

    rng = np.random.default_rng(seed)
    base = np.array([176, 136, 92], float)
    img = base + rng.normal(0, 5, (h, w, 1)) + 6 * np.sin(np.arange(w)[None, :, None] / 3.0)
    return Image.fromarray(np.clip(img, 0, 255).astype(np.uint8))


def _arrows(draw, x, y, s, color):
    for dx in (0, int(s * 0.9)):
        draw.polygon([(x + dx, y), (x + dx + s // 2, y - s), (x + dx + s, y)], fill=color)
        draw.rectangle([x + dx + s // 3, y, x + dx + 2 * s // 3, y + s], fill=color)


def textures() -> None:
    from PIL import Image, ImageDraw, ImageFont

    TEX.mkdir(parents=True, exist_ok=True)
    big, mid, small = ImageFont.truetype(FONT, 64), ImageFont.truetype(FONT, 40), ImageFont.truetype(FONT_REG, 30)
    for name, seed in (("soup_case", 1), ("new_case", 2)):
        atlas = Image.new("RGB", (3 * CELL, 2 * CELL))
        for face, (cx, cy) in CELLS.items():
            cell = _kraft(CELL, CELL, seed * 10 + cx + 3 * cy)
            d = ImageDraw.Draw(cell)
            side = face in ("+y", "-y", "-x", "+x")
            if name == "soup_case" and side:
                d.rectangle([20, 150, CELL - 20, 370], fill=(196, 36, 40))
                d.rectangle([20, 150, CELL - 20, 185], fill=(240, 236, 226))
                d.text((CELL // 2, 270), "SOUP", font=big, fill=(250, 246, 238), anchor="mm")
                d.text((CELL // 2, 335), "12 x 10.75 OZ", font=mid, fill=(250, 246, 238), anchor="mm")
                d.text((CELL // 2, 440), "CONDENSED", font=small, fill=(60, 40, 25), anchor="mm")
            elif name == "soup_case" and face == "+z":
                d.line([(0, CELL // 2), (CELL, CELL // 2)], fill=(150, 112, 74), width=10)     # the flaps' seam
                d.text((CELL // 2, 140), "SOUP", font=big, fill=(170, 30, 34), anchor="mm")
            elif name == "new_case" and face in ("+y", "-y"):
                # the hand hole: the cell spans the face's length (x) and height (z)
                u0 = 0.5 - L.HOLE_L / 2 / L.CASE[0]
                u1 = 0.5 + L.HOLE_L / 2 / L.CASE[0]
                v_top = 0.5 + L.HOLE_TOP / L.CASE[2]
                v_bot = 0.5 + L.HOLE_BOT / L.CASE[2]
                box_ = [int(u0 * CELL), int((1 - v_top) * CELL), int(u1 * CELL), int((1 - v_bot) * CELL)]
                d.rounded_rectangle(box_, radius=40, fill=(22, 16, 12))
                d.rectangle([20, 20, 200, 150], fill=(245, 243, 236))
                d.text((110, 60), "RICE", font=mid, fill=(30, 30, 30), anchor="mm")
                d.text((110, 115), "1.5 KG", font=small, fill=(30, 30, 30), anchor="mm")
                _arrows(d, 330, 100, 60, (20, 20, 20))
                d.text((CELL // 2, 440), "THIS SIDE UP", font=mid, fill=(20, 20, 20), anchor="mm")
            elif name == "new_case" and side:
                d.rectangle([30, 60, CELL - 30, 250], fill=(245, 243, 236))
                d.text((CELL // 2, 115), "RICE", font=big, fill=(30, 30, 30), anchor="mm")
                d.text((CELL // 2, 195), "1.5 KG", font=mid, fill=(30, 30, 30), anchor="mm")
                _arrows(d, 150, 380, 90, (20, 20, 20))
                d.text((CELL // 2, 470), "THIS SIDE UP", font=mid, fill=(20, 20, 20), anchor="mm")
            elif name == "new_case" and face == "+z":
                d.line([(0, CELL // 2), (CELL, CELL // 2)], fill=(150, 112, 74), width=10)
                d.text((CELL // 2, 150), "THIS SIDE UP", font=mid, fill=(20, 20, 20), anchor="mm")
                d.text((CELL // 2, 380), "RICE", font=big, fill=(30, 30, 30), anchor="mm")
            atlas.paste(cell, (cx * CELL, cy * CELL))
        atlas.save(TEX / LOOKS[name].texture)


def case_mesh(size) -> u.Mesh:
    """A box with each face mapped to its cell of the atlas, text upright on the sides and reading from outside."""
    hx, hy, hz = (s / 2 for s in size)
    faces = {   # face: four corners counter-clockwise from outside, as (u, v) = (0,0) (1,0) (1,1) (0,1) of the cell
        "+x": [(hx, -hy, -hz), (hx, hy, -hz), (hx, hy, hz), (hx, -hy, hz)],
        "-x": [(-hx, hy, -hz), (-hx, -hy, -hz), (-hx, -hy, hz), (-hx, hy, hz)],
        "+y": [(hx, hy, -hz), (-hx, hy, -hz), (-hx, hy, hz), (hx, hy, hz)],
        "-y": [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, -hy, hz), (-hx, -hy, hz)],
        "+z": [(-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)],
        "-z": [(-hx, hy, -hz), (hx, hy, -hz), (hx, -hy, -hz), (-hx, -hy, -hz)],
    }
    v, f, uv, nrm = [], [], [], []
    for face, corners in faces.items():
        k = len(v)
        v += corners
        cx, cy = CELLS[face]
        cuv = [((cx + a) / 3.0, 1.0 - (cy + 1 - b) / 2.0) for a, b in ((0, 0), (1, 0), (1, 1), (0, 1))]
        n = np.cross(np.subtract(corners[1], corners[0]), np.subtract(corners[2], corners[0]))
        n = n / np.linalg.norm(n)
        for tri in ((0, 1, 2), (0, 2, 3)):
            f.append([k + t for t in tri])
            uv += [cuv[t] for t in tri]
            nrm += [n] * 3
    m = u.Mesh(np.array(v, float), np.array(f))
    m.uv = np.array(uv)
    m.normals = np.array(nrm)
    return m


def n_colliders():
    """N's walls as boxes, with the two hand holes cut out of the long side walls, and the rice bags as one block
    filling the box below the holes. Returns (size, centre) pairs."""
    hx, hy, hz = (v / 2 for v in L.CASE)
    t = L.WALL
    out = [((2 * hx, 2 * hy, t), (0, 0, -hz + t / 2)), ((2 * hx, 2 * hy, t), (0, 0, hz - t / 2))]
    for sx in (-1, 1):
        out.append(((t, 2 * hy - 2 * t, 2 * hz - 2 * t), (sx * (hx - t / 2), 0, 0)))
    xl, zb, zt = L.HOLE_L / 2, L.HOLE_BOT, L.HOLE_TOP
    for sy in (-1, 1):
        y = sy * (hy - t / 2)
        ix = hx - t
        out += [((2 * ix, t, zb - (-hz + t)), (0, y, (zb + (-hz + t)) / 2)),          # below the hole
                ((2 * ix, t, (hz - t) - zt), (0, y, (zt + hz - t) / 2)),              # above it
                ((ix - xl, t, zt - zb), (-(ix + xl) / 2, y, (zt + zb) / 2)),          # to its near side
                ((ix - xl, t, zt - zb), ((ix + xl) / 2, y, (zt + zb) / 2))]           # to its far side
    top = zb - 0.005
    out.append(((2 * hx - 2 * t, 2 * hy - 2 * t, top - (-hz + t)), (0, 0, (top + (-hz + t)) / 2)))   # the rice
    return out


def cases() -> None:
    for name in ("soup_case", "new_case"):
        cols = ([u.collider(f"col{k:02d}", size, c, name) for k, (size, c) in enumerate(n_colliders())] if name == "new_case"
                else [u.collider("collider", L.CASE, (0, 0, 0), name)])
        u.write_usda(str(ASSETS / f"{name}.usda"), name, [("box", case_mesh(L.CASE), name)], LOOKS, cols,
                     mass=L.N_KG if name == "new_case" else L.SOUP_KG, friction=(L.CASE_MU, L.CASE_MU), solver_iters=16)


# -- the room -----------------------------------------------------------------------------------------------------------
def bx(lo, hi) -> u.Mesh:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return u.box(hi - lo, (lo + hi) / 2)


def shelving():
    """The chrome wire unit: four posts on levelling feet, and the shelf, a wire deck on a front and back truss.
    Returns {look: [Mesh]}."""
    out = {"chrome": [], "foot": []}
    x0, x1 = L.SHELF_X
    y0, y1 = L.SHELF_Y
    b = L.BATTEN_H
    for px, py in L.UNIT_POSTS:
        out["chrome"].append(u.cylinder(L.POST_R, L.POST_H - L.FOOT_H,
                                        (px, py, L.H(b + L.FOOT_H + (L.POST_H - L.FOOT_H) / 2)), segments=20))
        out["foot"].append(u.cylinder(0.016, L.FOOT_H, (px, py, L.H(b + L.FOOT_H / 2)), segments=16))
    zt = L.H(L.WORK_TOP)
    for y in np.arange(y0 + 0.02, y1 - 0.01, 0.0254):            # deck wires, front to back
        out["chrome"].append(bx((x0 + 0.004, y - 0.002, zt - 0.004), (x1 - 0.004, y + 0.002, zt)))
    for x in (x0 + 0.004, x1 - 0.004):                             # truss: top and bottom bars, zigzag
        out["chrome"].append(bx((x - 0.004, y0, zt - 0.008), (x + 0.004, y1, zt)))
        out["chrome"].append(bx((x - 0.003, y0, zt - L.SHELF_T), (x + 0.003, y1, zt - L.SHELF_T + 0.006)))
        ys = np.linspace(y0 + 0.02, y1 - 0.02, 22)
        for a, b in zip(ys[:-1], ys[1:]):
            za, zb = (zt - 0.006, zt - L.SHELF_T + 0.003) if len(out["chrome"]) % 2 else (zt - L.SHELF_T + 0.003, zt - 0.006)
            out["chrome"].append(u.tube([(x, a, za), (x, b, zb)], 0.002, 6))
    for y in np.linspace(y0 + 0.27, y1 - 0.27, 3):                  # under-supports, front to back
        out["chrome"].append(bx((x0, y - 0.003, zt - 0.012), (x1, y + 0.003, zt - 0.004)))
    for px, py in L.UNIT_POSTS:                                     # corner collars (split sleeves)
        out["chrome"].append(u.cylinder(L.POST_R + 0.006, 0.05, (px, py, zt - 0.02), segments=20))
    return out


def room() -> dict:
    parts: dict[str, list] = {k: [] for k in LOOKS}
    cols = []
    for look, ms in shelving().items():
        parts[look] += ms
    for y0_, y1_ in L.BATTEN_Y:
        lo, hi = (L.BATTEN_X[0], y0_, L.H(0.0)), (L.BATTEN_X[1], y1_, L.H(L.BATTEN_H))
        parts["timber"].append(bx(lo, hi))
        cols.append((lo, hi))
    x0, x1 = L.SHELF_X
    y0, y1 = L.SHELF_Y
    cols.append(((x0, y0, L.H(L.WORK_TOP - L.SHELF_T)), (x1, y1, L.H(L.WORK_TOP))))
    for px, py in L.UNIT_POSTS:
        cols.append(((px - L.POST_R, py - L.POST_R, L.H(L.BATTEN_H)), (px + L.POST_R, py + L.POST_R, L.H(L.BATTEN_H + L.POST_H))))
    return {"parts": parts, "cols": cols}


def write_room() -> None:
    r = room()
    meshes = []
    for look, ms in r["parts"].items():
        if not ms:
            continue
        smooth = [m for m in ms if getattr(m, "normals", None) is not None]
        flat = [m for m in ms if getattr(m, "normals", None) is None]
        if flat:
            meshes.append((look, u.join(*flat), look))
        for k, m in enumerate(smooth):
            meshes.append((f"{look}_body{k}", m, look))
    colliders = []
    for k, (lo, hi) in enumerate(r["cols"]):
        lo, hi = np.asarray(lo), np.asarray(hi)
        colliders.append(u.collider(f"col{k:02d}", hi - lo, (lo + hi) / 2, "room"))
    u.write_usda(str(ASSETS / "room.usda"), "room", meshes, LOOKS, colliders, mass=None, friction=(L.ROOM_MU, L.ROOM_MU),
                 kinematic=True)
    print(f"room: {len(meshes)} meshes, {len(colliders)} colliders")


def hand_hole_template() -> None:
    """A 1:1 cutting template for N's hand holes (print at 100 %): the long side panel's outline, 11 5/8 x 5 5/8 in, and
    the 4 x 1-1/4 in hole with round ends, centred along it, its top edge 2 in below the panel's top edge."""
    W, Hh = 11.625, 5.625
    hl, hh, top = 4.0, 1.25, 2.0
    x0, y0 = (W - hl) / 2, top
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{W + 1}in" height="{Hh + 1.6}in" viewBox="-0.5 -0.5 {W + 1} {Hh + 1.6}">
  <rect x="0" y="0" width="{W}" height="{Hh}" fill="none" stroke="black" stroke-width="0.02"/>
  <rect x="{x0}" y="{y0}" width="{hl}" height="{hh}" rx="{hh / 2}" ry="{hh / 2}" fill="none" stroke="black" stroke-width="0.03"/>
  <line x1="{W / 2}" y1="-0.3" x2="{W / 2}" y2="{Hh + 0.3}" stroke="black" stroke-width="0.01" stroke-dasharray="0.1,0.1"/>
  <text x="0" y="{Hh + 0.45}" font-size="0.22" font-family="sans-serif">Hand-hole template, print at 100 %: long side panel 11 5/8 x 5 5/8 in (top edge up). Cut the slot, 4 x 1 1/4 in, top 2 in below the top edge.</text>
  <text x="0" y="{Hh + 0.8}" font-size="0.22" font-family="sans-serif">Check: this bar is 4 in long.</text>
  <line x1="4" y1="{Hh + 0.75}" x2="8" y2="{Hh + 0.75}" stroke="black" stroke-width="0.04"/>
</svg>
"""
    (HERE / "hand_hole_template.svg").write_text(svg)


if __name__ == "__main__":
    ASSETS.mkdir(exist_ok=True)
    textures()
    cases()
    write_room()
    hand_hole_template()
