"""Sizes of Arena object-library assets, for task files and tests that place ``Asset`` objects (pure Python).

``docs/assets/bounds.json`` holds each registry entry's unscaled local bounding box, measured from the USD mirror
(``scripts/asset_mirror.py`` + ``scripts/asset_bounds.py``). An ``Asset(extent=...)`` is scaled uniformly so its
widest horizontal extent equals ``extent`` and rests on the table (yam_arena ``station.placement``), so its size
on the bench follows from the bounds alone.
"""

from __future__ import annotations

import json
import math
from functools import cache
from pathlib import Path

BOUNDS_FILE = Path(__file__).resolve().parents[1] / "docs" / "assets" / "bounds.json"
REST_GAP = 0.001  # yam_arena.station.REST_GAP: the bottom of a placed asset sits this high over the table


@cache
def bounds() -> dict[str, dict]:
    return json.loads(BOUNDS_FILE.read_text())


def unscaled_size(asset: str) -> tuple[float, float, float]:
    """The asset's local bounding-box size (m) at scale 1."""
    try:
        return tuple(bounds()[asset]["size"])
    except KeyError as e:
        raise KeyError(f"{asset!r} has no entry in {BOUNDS_FILE.name}; run scripts/asset_mirror.py + asset_bounds.py") from e


def scale(asset: str, extent: float) -> float:
    sx, sy, _ = unscaled_size(asset)
    return extent / max(sx, sy)


def size(asset: str, extent: float, yaw: float = 0.0) -> tuple[float, float, float]:
    """World-aligned bounding-box size (m) of the asset scaled to ``extent`` and turned ``yaw`` about the vertical."""
    s = scale(asset, extent)
    sx, sy, sz = (v * s for v in unscaled_size(asset))
    c, n = abs(math.cos(yaw)), abs(math.sin(yaw))
    return (sx * c + sy * n, sx * n + sy * c, sz)


def height(asset: str, extent: float) -> float:
    return size(asset, extent)[2]


def spawn_pos(spec, table_top: float) -> tuple[float, float, float]:
    """Where the asset's bounding-box centre is after spawning: over ``spec.xy``, resting on the table."""
    h = height(spec.asset, spec.extent)
    return (float(spec.xy[0]), float(spec.xy[1]), table_top + REST_GAP + h / 2)


def half_extents(spec) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Local (low, high) about the bounding-box centre, world-aligned, for test snapshots."""
    sx, sy, sz = size(spec.asset, spec.extent, spec.yaw)
    return (-sx / 2, -sy / 2, -sz / 2), (sx / 2, sy / 2, sz / 2)
