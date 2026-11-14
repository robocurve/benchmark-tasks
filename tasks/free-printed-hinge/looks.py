"""looks: detailed, render-only meshes on the task file's boxes.

The physics of the scene stays the task file's boxes. :func:`dress` names one or more USD files in ``looks/`` for a
box: when the box spawns, its plain cuboid is made invisible (it still collides, and still sets the box's bounds) and
each file is referenced under it as ``<box>/look`` (``look1``, ...), so the meshes move with the box. A file can be
given with an offset (metres, in the box's frame) to place a look made for another box on this one. The meshes carry
no physics. They are modelled in Blender by ``looks/make_looks.py``, each inside its box.

Pure Python at import time: ``pxr`` is imported when a box spawns, inside Isaac Sim.
"""

from __future__ import annotations

from pathlib import Path

from bonds import on_spawn

FOLDER = Path(__file__).resolve().parent / "looks"
_LOOKS: dict[str, list[tuple[Path, tuple[float, float, float]]]] = {}


def dress(looks: dict, hide=()) -> None:
    """Give each box named in ``looks`` its mesh: a file in ``looks/``, or a list of files and (file, offset) pairs.
    The boxes in ``hide`` are hidden with no mesh."""
    for name, value in looks.items():
        items = value if isinstance(value, list) else [value]
        entries = []
        for item in items:
            file, offset = (item, (0.0, 0.0, 0.0)) if isinstance(item, str) else item
            path = FOLDER / file
            if not path.is_file():
                raise FileNotFoundError(f"{name}: no {path} (run looks/make_looks.py in Blender)")
            entries.append((path, tuple(float(v) for v in offset)))
        _LOOKS[name] = entries
    for name in hide:
        _LOOKS.setdefault(name, [])


def looks() -> dict:
    """Every dressed box's files and offsets."""
    return dict(_LOOKS)


@on_spawn
def _dress(name: str, prim) -> None:
    if name not in _LOOKS:
        return
    from pxr import Gf, Usd, UsdGeom

    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Gprim):
            UsdGeom.Imageable(p).MakeInvisible()
    for i, (path, offset) in enumerate(_LOOKS[name]):
        look = UsdGeom.Xform.Define(prim.GetStage(), prim.GetPath().AppendChild("look" if i == 0 else f"look{i}"))
        if any(offset):
            look.AddTranslateOp().Set(Gf.Vec3d(*offset))
        look.GetPrim().GetReferences().AddReference(str(path))
