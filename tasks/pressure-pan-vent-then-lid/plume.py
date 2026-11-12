"""The steam plume, drawn as a column of pale spheres with Isaac Lab's VisualizationMarkers so camera-based policies can
see it. Display only: the markers have no collision and no physics, they never feed the scorer, and any exception turns
them off with one warning while the episode carries on (the same approach as the painting task's wet-paint tiles).
Outside the simulator (unit tests), nothing is drawn."""

from __future__ import annotations

import math
import warnings
from typing import Any

import numpy as np

from geometry import COLOR_STEAM, KEEPOUT_Z_MAX_M, PLUME_STEPS, TABLE_TOP, VALVE_XY

_PUFFS = 7
_Z0 = 0.095           # lowest puff, just above the valve (z above the table)
_Z1 = 0.20            # top of the column that is drawn


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


class _Plume:
    def __init__(self) -> None:
        self._markers: Any = None
        self._off = False
        self._shown = False

    def hide(self) -> None:
        if self._markers is None or self._off:
            return
        try:
            self._markers.set_visibility(False)
            self._shown = False
        except Exception as error:  # cosmetic only
            self._disable(error)

    def update(self, environment, active: bool, step: int) -> None:
        if self._off:
            return
        if not active and not self._shown:
            return
        try:
            if not active:
                if self._markers is not None:
                    self._markers.set_visibility(False)
                self._shown = False
                return
            if self._markers is None:
                self._markers = self._make()
            import parts

            origin = parts._to_numpy(environment.unwrapped.scene.env_origins[0])
            fade = max(0.35, 1.0 - 0.65 * step / PLUME_STEPS)
            pts, scales = [], []
            for i in range(_PUFFS):
                frac = i / (_PUFFS - 1)
                rise = (frac + 0.08 * step) % 1.0 if i % 2 else frac
                z = _Z0 + (_Z1 - _Z0) * rise
                wobble = 0.006 * math.sin(0.7 * step + 1.7 * i)
                pts.append([VALVE_XY[0] + wobble, VALVE_XY[1] - wobble, TABLE_TOP + min(z, KEEPOUT_Z_MAX_M)])
                scales.append(fade * (0.6 + 0.9 * rise))
            pts = np.asarray(pts, dtype=float) + origin
            quats = np.tile([0.0, 0.0, 0.0, 1.0], (_PUFFS, 1))
            sc = np.asarray(scales, dtype=float)[:, None] * np.ones((1, 3))
            self._markers.set_visibility(True)
            self._markers.visualize(
                translations=pts.astype(np.float32), orientations=quats.astype(np.float32), scales=sc.astype(np.float32)
            )
            self._shown = True
        except ImportError:
            self._off = True  # no simulator: unit tests
        except Exception as error:  # cosmetic only
            self._disable(error)

    def _disable(self, error: Exception) -> None:
        self._off = True
        warnings.warn(f"steam plume markers turned off ({type(error).__name__}: {error}); scoring is unaffected")

    @staticmethod
    def _make() -> Any:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        puff = sim_utils.SphereCfg(
            radius=0.02,
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=tuple(_linear(c) for c in COLOR_STEAM), roughness=0.9, opacity=0.6
            ),
        )
        return VisualizationMarkers(VisualizationMarkersCfg(prim_path="/Visuals/steam_plume", markers={"puff": puff}))


PLUME = _Plume()
