"""Display tiles for air_fryer_basket_release_button_v2 using Isaac Lab VisualizationMarkers."""

from __future__ import annotations

import warnings
from typing import Any

import numpy as np

from geometry import (
    COLOR_AMBER,
    HOUSING_POS,
    LAMP_TILE_LOCAL,
    TABLE_TOP,
    VENT_GLOW_TILES_LOCAL,
)

_NUM_TILES = 5
_PARK = np.tile([0.0, 0.0, -10.0], (_NUM_TILES, 1)).astype(np.float32)
_IDENTITY = np.tile([0.0, 0.0, 0.0, 1.0], (_NUM_TILES, 1)).astype(np.float32)


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


class _Display:
    def __init__(self) -> None:
        self._markers: Any = None
        self._off = False
        self._shown = False

    def _make(self):
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        amber = tuple(_linear(c) for c in COLOR_AMBER)
        tile = sim_utils.CuboidCfg(
            size=(0.015, 0.015, 0.002),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=amber, roughness=0.3),
        )
        cfg = VisualizationMarkersCfg(prim_path="/Visuals/af2_hot_cues", markers={"tile": tile})
        return VisualizationMarkers(cfg)

    def _world_positions(self, env_origin: np.ndarray) -> np.ndarray:
        h_base = np.asarray(HOUSING_POS, dtype=float) + env_origin
        pts = []
        # 1 lamp tile
        pts.append(h_base + np.asarray(LAMP_TILE_LOCAL, dtype=float))
        # 4 vent glow tiles
        for v in VENT_GLOW_TILES_LOCAL:
            pts.append(h_base + np.asarray(v, dtype=float))
        return np.asarray(pts, dtype=np.float32)

    def reset(self, environment) -> None:
        """Place tiles at their world positions at reset; never call set_visibility mid-episode."""
        if self._off:
            return
        try:
            if self._markers is None:
                self._markers = self._make()
            origin = np.zeros(3, dtype=float)
            try:
                base = environment.unwrapped
                origin = np.asarray(base.scene.env_origins[0].detach().cpu().numpy(), dtype=float)
            except Exception:
                pass
            pts = self._world_positions(origin)
            self._markers.visualize(
                translations=pts,
                orientations=_IDENTITY,
                scales=np.full((_NUM_TILES, 3), 1.0, dtype=np.float32),
            )
            self._shown = True
        except ImportError:
            self._off = True  # Offline / unit tests without Isaac Sim
        except Exception as error:
            warnings.warn(f"Display disabled: {error}", RuntimeWarning, stacklevel=2)
            self._off = True


DISPLAY = _Display()
