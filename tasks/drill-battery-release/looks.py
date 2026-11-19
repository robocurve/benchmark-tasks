"""looks: detailed, render-only meshes on the task file's boxes.

The physics of the scene stays the task file's boxes. :func:`dress` names a USD file in ``looks/`` for a box: when
the box spawns, its plain cuboid is made invisible (it still collides, and still sets the box's bounds) and the file
is referenced under it as ``<box>/look``, so the mesh moves with the box. The meshes carry no physics. They are
modelled in Blender by ``looks/make_looks.py``, each inside its box, and a box can also be hidden with no mesh of
its own when another box's mesh draws it.

Pure Python at import time: ``pxr`` is imported when a box spawns, inside Isaac Sim.
"""

from __future__ import annotations

from pathlib import Path

from bonds import on_spawn

FOLDER = Path(__file__).resolve().parent / "looks"
_LOOKS: dict[str, Path | None] = {}


def dress(looks: dict[str, str], hide=()) -> None:
    """Give each box named in ``looks`` the mesh in ``looks/<file>``, and hide the boxes in ``hide``."""
    for name, file in looks.items():
        path = FOLDER / file
        if not path.is_file():
            raise FileNotFoundError(f"{name}: no {path} (run looks/make_looks.py in Blender)")
        _LOOKS[name] = path
    for name in hide:
        _LOOKS.setdefault(name, None)


@on_spawn
def _dress(name: str, prim) -> None:
    if name not in _LOOKS:
        return
    from pxr import Usd, UsdGeom

    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Gprim):
            UsdGeom.Imageable(p).MakeInvisible()
    path = _LOOKS[name]
    if path is not None:
        look = UsdGeom.Xform.Define(prim.GetStage(), prim.GetPath().AppendChild("look"))
        look.GetPrim().GetReferences().AddReference(str(path))
