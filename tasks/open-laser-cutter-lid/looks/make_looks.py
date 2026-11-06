"""make_looks.py: model the laser cutter's visual meshes in Blender and export them as USD next to this file.

    blender -b -P looks/make_looks.py          (Blender 4.2 or later, from the task folder)

or, in a running Blender, ``exec`` this file and call ``build()``. The physics stays the task file's boxes. Each look
is a render-only mesh that ``looks.py`` puts on one box, whose plain cuboid is then hidden but still sets its bounds.

- ``cutter_body.usdc``: the machine's fixed body (walls, back block with its control panel and exhaust duct, the
  honeycomb bed, the gantry and the laser head), on the box of its front wall.
- ``cutter_lid.usdc``: the lid (top panel with a tinted window, front flap, handle bar), on the lid's box, inside it.

The modelling is in ``cutter_looks.py`` (the same file is in the laser-scrap-sheet-lift task), the sizes in
``cutter.py``. Everything this script makes in Blender is named with the ``open-laser-cutter-lid__`` prefix.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
for _m in ("cutter", "cutter_looks"):  # another task folder's copy may be loaded in this Blender
    sys.modules.pop(_m, None)
import cutter as C  # noqa: E402
import cutter_looks as K  # noqa: E402
PREFIX = "open-laser-cutter-lid__"


def build(keep=False):
    """Model every look, export it, and (unless ``keep``) remove it from Blender again."""
    K.setup(PREFIX, HERE)
    K.clear()
    front = next(c for n, c, _ in C.body_boxes() if n == "cutter_front")
    looks = [K.build_body(C, front), K.build_lid(C)]
    out = [K.export(look) for look in looks]
    if not keep:
        K.clear()
    return out


if __name__ == "__main__":
    print(build())
