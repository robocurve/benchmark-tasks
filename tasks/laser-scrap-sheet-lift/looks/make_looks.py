"""make_looks.py: model the scene's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

or, in a running Blender, ``exec`` this file and call ``build()``. The physics stays the task file's boxes (the
sheet's collider is boxes built by ``plate.py``, the parts are boxes and convex prisms). Each look is a render-only mesh
that ``looks.py`` puts on one of those boxes, whose plain cuboid is then hidden but still sets its bounds. Every look
is built in its box's own frame, around the box's centre (metres, z up), and checked to stay inside the box (the bin's
look rides on its floor box and draws the walls too).

- ``sheet.usdc``: the clear, light blue tinted acrylic sheet, 22 x 20 cm, 3 mm, with the four holes the laser cut.
- ``disc.usdc``, ``rect_long.usdc``, ``rect_wide.usdc``, ``triangle.usdc``: the cut parts, clear acrylic with orange
  masking paper on top.
- ``cutter_body.usdc``: the laser cutter's fixed body (walls, back block with its control panel and exhaust duct,
  the honeycomb bed, the gantry and the laser head), on the box of its front wall.
- ``cutter_lid_open.usdc``: its lid standing open at 80 degrees, with the two gas struts, on the open lid's box.
- ``bin.usdc``: the black plastic scrap bin.

Sizes come from ``layout.py`` and ``cutter.py`` (the task file uses the same). The machine is modelled in
``cutter_looks.py`` (the same file is in the open-laser-cutter-lid task), which also holds the helpers used here.
Textures are drawn with numpy and saved in ``textures/``. Nothing is downloaded. Everything this script makes in
Blender is named with the ``laser-scrap-sheet-lift__`` prefix, and only those are ever removed.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
for _m in ("cutter", "cutter_looks", "layout"):  # another task folder's copy may be loaded in this Blender
    sys.modules.pop(_m, None)
import cutter as C  # noqa: E402
import cutter_looks as K  # noqa: E402
import layout as L  # noqa: E402
from cutter_looks import Look, bm_box, bm_prism, boolean_cut, circle, material  # noqa: E402

PREFIX = "laser-scrap-sheet-lift__"


# -- the looks -------------------------------------------------------------------------------------------------------

ACRYLIC = dict(color=(0.72, 0.86, 0.97), rough=0.06, alpha=0.55)
PAPER = (0.93, 0.47, 0.10)


def build_sheet():
    sx, sy, t = L.SHEET_SIZE
    look = Look("sheet", L.SHEET_SIZE)
    clear = material("acrylic_sheet", **ACRYLIC)
    ob = look.mesh("plate", bm_box((-sx / 2, -sy / 2, -t / 2), (sx / 2, sy / 2, t / 2)), clear)
    cutters = []
    for name, (u, v) in L.PART_UV.items():
        if name == "disc":
            pts = circle(u, v, L.DISC_R + L.PLAY, 96)
        elif name == "triangle":
            pts = L.triangle_outline(u, v, grow=L.PLAY)
        else:
            pts = L.rect_outline(u, v, *L.PART_SIZE[name], grow=L.PLAY)
        cutters.append(bm_prism(pts, -t, t))
    boolean_cut(look, ob, cutters)
    return look


def build_part(name):
    sx, sy = L.PART_SIZE[name]
    t = L.SHEET_SIZE[2]
    look = Look(name, (sx, sy, t))
    clear = material("acrylic_part", **ACRYLIC)
    paper = material("masking_paper", PAPER, 0.8)
    if name == "disc":
        pts = circle(0.0, 0.0, L.DISC_R, 72)
    elif name == "triangle":
        pts = L.triangle_outline()
    else:
        pts = L.rect_outline(0.0, 0.0, sx, sy)
    paper_t = 0.0003
    look.mesh("body", bm_prism(pts, -t / 2, t / 2 - paper_t), clear)
    inset = [(x * 0.995, y * 0.995) for x, y in pts]  # the paper's edge 0.5 % in, burnt back by the beam
    look.mesh("paper", bm_prism(inset, t / 2 - paper_t, t / 2), paper)
    return look


def build_bin():
    sx, sy, h = L.BIN_OUTSIDE
    w, f = L.BIN_WALL, L.BIN_FLOOR
    look = Look("bin", None)  # rides on the floor box (centred at the floor's centre), draws the walls too
    plastic = material("bin_plastic", (0.06, 0.06, 0.07), 0.6)
    z0 = -f / 2
    look.mesh("floor", bm_box((-sx / 2, -sy / 2, z0), (sx / 2, sy / 2, z0 + f)), plastic, bevel=0.002)
    for name, lo, hi in [
        ("near", (-sx / 2, -sy / 2, z0), (-sx / 2 + w, sy / 2, z0 + h)),
        ("far", (sx / 2 - w, -sy / 2, z0), (sx / 2, sy / 2, z0 + h)),
        ("right", (-sx / 2 + w, -sy / 2, z0), (sx / 2 - w, -sy / 2 + w, z0 + h)),
        ("left", (-sx / 2 + w, sy / 2 - w, z0), (sx / 2 - w, sy / 2, z0 + h)),
    ]:
        look.mesh(name, bm_box(lo, hi), plastic, bevel=0.002)
    lip = 0.006  # a rolled rim round the top, outside the walls
    for name, lo, hi in [
        ("lip_near", (-sx / 2 - lip, -sy / 2 - lip, z0 + h - 0.008), (-sx / 2, sy / 2 + lip, z0 + h)),
        ("lip_far", (sx / 2, -sy / 2 - lip, z0 + h - 0.008), (sx / 2 + lip, sy / 2 + lip, z0 + h)),
        ("lip_right", (-sx / 2, -sy / 2 - lip, z0 + h - 0.008), (sx / 2, -sy / 2, z0 + h)),
        ("lip_left", (-sx / 2, sy / 2, z0 + h - 0.008), (sx / 2, sy / 2 + lip, z0 + h)),
    ]:
        look.mesh(name, bm_box(lo, hi), plastic, bevel=0.001)
    return look


# -- export ----------------------------------------------------------------------------------------------------------


def build(keep=False):
    """Model every look, export it, and (unless ``keep``) remove it from Blender again."""
    K.setup(PREFIX, HERE)
    K.clear()
    front = next(c for n, c, _ in C.body_boxes() if n == "cutter_front")
    looks = [build_sheet(), *(build_part(n) for n in L.PART_UV), K.build_body(C, front),
             K.build_open_lid(C, C.OPEN_DEG, L.LID_OPEN_CENTRE), build_bin()]
    out = [K.export(look) for look in looks]
    if not keep:
        K.clear()
    return out


if __name__ == "__main__":
    print(build())
