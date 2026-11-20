"""Models that are not in Arena's object library, kept next to the task that uses them (pure Python).

Here: the instant photo's camera and print, written by its ``assets/build.py``. ``Asset(..., asset=<name>)`` uses
them like library assets: ``register()`` (sim side, called by ``rta.scene``) adds them to Arena's asset registry,
and ``docs/assets/bounds.json`` holds their sizes for ``rta.assets``.
"""

from __future__ import annotations

from rta import TASKS_DIR

LOCAL = {
    # built by tasks/h07_instant_photo/assets/build.py (original geometry, plain colours)
    "h07_camera": TASKS_DIR / "h07_instant_photo" / "assets" / "camera.usda",
    "h07_print": TASKS_DIR / "h07_instant_photo" / "assets" / "print.usda",
}


def register() -> None:
    """Add every local model to Arena's asset registry (idempotent). Import after app start."""
    from isaaclab_arena.assets.object_library import LibraryObject
    from isaaclab_arena.assets.register import register_asset
    from isaaclab_arena.assets.registries import AssetRegistry

    for name, path in LOCAL.items():
        if AssetRegistry().is_registered(name, ensure_loaded=False):
            continue
        register_asset(type(f"Local_{name}", (LibraryObject,), {"name": name, "tags": ["object"], "usd_path": str(path)}))
