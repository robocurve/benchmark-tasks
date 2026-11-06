"""ledasset: make our own LED model (assets/led.usda) loadable as Asset(asset="t13_led") in a yam_arena task file.

yam_arena builds an Asset by asking Arena's AssetRegistry for the entry with that name and reading only its
usd_path (yam_arena.scene._library_item). Rather than adding to Arena's registry, this module swaps the AssetRegistry
name inside yam_arena.scene for a subclass that answers our one name itself and passes every other name through to
Arena unchanged. It does this as soon as yam_arena.scene loads; nothing in yam_arena or Arena is changed on disk.

The LED's anode (+, red tab) is at the model's local +x end; its cathode (-, black tab) at local -x.
"""

from __future__ import annotations

from pathlib import Path

from stationhooks import after_import

LED_NAME = "t13_led"
LED_USD = str(Path(__file__).resolve().parent / "assets" / "led.usda")


class _LedEntry:
    """What yam_arena reads from a library entry: its name, tags and usd_path."""

    name = LED_NAME
    tags = ["object"]
    usd_path = LED_USD


def _use_our_led(scene_module) -> None:
    arena_registry = scene_module.AssetRegistry
    if getattr(arena_registry, "_t13_led", False):
        return

    class RegistryWithLed(arena_registry):
        _t13_led = True

        def get_asset_by_name(self, name):
            if name == LED_NAME:
                return _LedEntry
            return super().get_asset_by_name(name)

    scene_module.AssetRegistry = RegistryWithLed
    print(f"[ledasset] {LED_NAME} -> {LED_USD}")


after_import("yam_arena.scene", _use_our_led)
