"""ownassets: load our own USD models (in ./assets) through yam_arena's Asset(asset="<name>").

yam_arena builds an Asset by asking Arena's AssetRegistry for the entry with that name and reading its usd_path. This
module swaps the AssetRegistry name inside yam_arena.scene for a subclass that answers our registered names itself
and passes every other name to Arena unchanged. Nothing in yam_arena or Arena is changed on disk.

    from ownassets import register
    register("t13_foam_slab", "assets/foam_slab.usda")      # path relative to this folder
"""

from __future__ import annotations

from pathlib import Path

from stationhooks import after_import

HERE = Path(__file__).resolve().parent
_OURS: dict = {}


def register(name: str, relative_usd: str) -> None:
    path = str(HERE / relative_usd)
    _OURS[name] = type(f"Entry_{name}", (), {"name": name, "tags": ["object"], "usd_path": path})


def _install(scene_module) -> None:
    arena_registry = scene_module.AssetRegistry
    if getattr(arena_registry, "_ownassets", False):
        return

    class RegistryWithOurs(arena_registry):
        _ownassets = True

        def get_asset_by_name(self, name):
            if name in _OURS:
                return _OURS[name]
            return super().get_asset_by_name(name)

    scene_module.AssetRegistry = RegistryWithOurs
    for name, entry in _OURS.items():
        print(f"[ownassets] {name} -> {entry.usd_path}")


after_import("yam_arena.scene", _install)
