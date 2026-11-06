"""looks: detailed, render-only meshes on the task file's boxes, with parts shown or hidden at run time.

The physics of the scene stays the task file's boxes. :func:`dress` names a USD file in ``looks/`` for a box: when
the box spawns, its plain cuboid is made invisible (it still collides, and still sets the box's bounds) and the file
is referenced under it as ``<box>/look``, so the mesh moves with the box. The meshes carry no physics. They are
modelled in Blender by ``looks/make_looks.py``, each inside its box, and a box can also be hidden with no mesh of
its own when another box's mesh draws it.

:func:`show` turns one named part of a box's look (a prim of the look's file, for example a phone's lit screen) on
or off while the simulation runs, by authoring its USD visibility. Parts named in ``dress(..., hidden=...)`` start
hidden.

Pure Python at import time: ``pxr`` is imported when a box spawns, inside Isaac Sim.
"""

from __future__ import annotations

from pathlib import Path

from bonds import on_spawn

FOLDER = Path(__file__).resolve().parent / "looks"
_LOOKS: dict[str, Path | None] = {}
_HIDDEN: set[str] = set()  # parts of looks that start hidden
_PARTS: dict[tuple[str, str], object] = {}  # (box, part) -> the part's prim, once spawned
_SHOWN: dict[tuple[str, str], bool] = {}


def dress(looks: dict[str, str], hide=(), hidden=()) -> None:
    """Give each box named in ``looks`` the mesh in ``looks/<file>``, hide the boxes in ``hide``, and start the
    look parts named in ``hidden`` hidden."""
    for name, file in looks.items():
        path = FOLDER / file
        if not path.is_file():
            raise FileNotFoundError(f"{name}: no {path} (run looks/make_looks.py in Blender)")
        _LOOKS[name] = path
    for name in hide:
        _LOOKS.setdefault(name, None)
    _HIDDEN.update(hidden)


def _set(prim, visible: bool) -> None:
    from pxr import UsdGeom

    img = UsdGeom.Imageable(prim)
    img.MakeVisible() if visible else img.MakeInvisible()


def show(name: str, part: str, visible: bool = True) -> None:
    """Show (or hide) the part ``part`` of box ``name``'s look. A no-op if it already is, or outside the simulation
    (the box has not spawned)."""
    key = (name, part)
    prim = _PARTS.get(key)
    if prim is None or _SHOWN.get(key) == visible:
        return
    _set(prim, visible)
    _SHOWN[key] = visible


def shown(name: str, part: str) -> bool:
    """Whether the part is showing (False outside the simulation)."""
    return bool(_SHOWN.get((name, part), False))


@on_spawn
def _dress(name: str, prim) -> None:
    if name not in _LOOKS:
        return
    from pxr import Usd, UsdGeom

    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Gprim):
            UsdGeom.Imageable(p).MakeInvisible()
    path = _LOOKS[name]
    if path is None:
        return
    look = UsdGeom.Xform.Define(prim.GetStage(), prim.GetPath().AppendChild("look"))
    look.GetPrim().GetReferences().AddReference(str(path))
    for p in Usd.PrimRange(look.GetPrim()):
        if p.GetName() in _HIDDEN and (name, p.GetName()) not in _PARTS:
            _PARTS[(name, p.GetName())] = p
            _set(p, False)
            _SHOWN[(name, p.GetName())] = False
