"""The printer's status lamp and the paper's orange free-end tape, drawn with Isaac Lab's VisualizationMarkers so camera-based
policies can see them. Display only: the markers have no collision and no physics, they never feed the scorer, and any
exception turns them off with one warning while the episode carries on.

Every marker is created ONCE (at the first reset) and afterwards only MOVED. An unused lamp colour is parked 10 m under the
floor. Switching a marker's visibility on and off mid-episode makes the renderer wash the frame grey, so set_visibility is
never called here. Outside the simulator (unit tests), nothing is drawn."""

from __future__ import annotations

import warnings
from typing import Any

import numpy as np

from geometry import (
    COLOR_LAMP_AMBER,
    COLOR_LAMP_GREEN,
    COLOR_LAMP_RED,
    COLOR_TAPE,
    LAMP_CENTRE,
    LAMP_SIZE,
    PRINTER_XY,
    TABLE_TOP,
    TAPE_MARKER_LOCAL,
    TAPE_MARKER_SIZE,
)
from logic import rotation_from_quat

_PARK = np.array([[0.0, 0.0, -10.0]], dtype=np.float32)
_IDENTITY = np.array([[0.0, 0.0, 0.0, 1.0]], dtype=np.float32)
_LAMP_WORLD = np.array([PRINTER_XY[0] + LAMP_CENTRE[0], PRINTER_XY[1] + LAMP_CENTRE[1], TABLE_TOP + LAMP_CENTRE[2]])
_LAMPS = {"red": COLOR_LAMP_RED, "green": COLOR_LAMP_GREEN, "amber": COLOR_LAMP_AMBER}


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _to_np(v) -> np.ndarray:
    v = getattr(v, "torch", v)
    if hasattr(v, "detach"):
        v = v.detach()
    if hasattr(v, "cpu"):
        v = v.cpu()
    if hasattr(v, "numpy"):
        v = v.numpy()
    return np.asarray(v, dtype=float)


class _Display:
    def __init__(self) -> None:
        self._markers: dict[str, Any] = {}
        self._last: dict[str, np.ndarray] = {}
        self._off = False

    # -- public ------------------------------------------------------------------------------------------------------
    def update(self, environment, lamp: str, roll_pos, roll_quat) -> None:
        """Show the lamp colour ``lamp`` and put the tape on the roll's free end. Creates the markers on first use."""
        if self._off:
            return
        try:
            if not self._markers:
                self._markers = self._make_all()

            origin = _to_np(environment.unwrapped.scene.env_origins)[0]
            for name in _LAMPS:
                self._move(f"lamp_{name}", (_LAMP_WORLD + origin) if name == lamp else None)
            tape = np.asarray(roll_pos, dtype=float) + rotation_from_quat(roll_quat) @ np.array(TAPE_MARKER_LOCAL)
            self._move("tape", tape + origin)
        except ImportError:
            self._off = True  # no simulator: unit tests
        except Exception as error:  # cosmetic only
            self._disable(error)

    def reset(self, environment, lamp: str, roll_pos, roll_quat) -> None:
        """Create the markers if needed and draw the start state; forget the last positions so everything is redrawn."""
        self._last.clear()
        self.update(environment, lamp, roll_pos, roll_quat)

    # -- internals ---------------------------------------------------------------------------------------------------
    def _move(self, name: str, point) -> None:
        """Move one marker to ``point`` (world), or park it 10 m under the floor when ``point`` is None."""
        target = _PARK if point is None else np.asarray(point, dtype=np.float32).reshape(1, 3)
        previous = self._last.get(name)
        if previous is not None and float(np.abs(previous - target).max()) < 2e-4:
            return
        self._markers[name].visualize(translations=target, orientations=_IDENTITY)
        self._last[name] = target

    def _disable(self, error: Exception) -> None:
        self._off = True
        warnings.warn(f"printer lamp and tape markers turned off ({type(error).__name__}: {error}); scoring is unaffected")

    @staticmethod
    def _make_all() -> dict[str, Any]:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        def make(prim: str, size, rgb, glow: bool):
            kwargs = dict(diffuse_color=tuple(_linear(c) for c in rgb), roughness=0.4)
            try:
                material = sim_utils.PreviewSurfaceCfg(
                    **kwargs, **({"emissive_color": tuple(_linear(c) * 0.8 for c in rgb)} if glow else {})
                )
            except TypeError:
                material = sim_utils.PreviewSurfaceCfg(**kwargs)
            box_cfg = sim_utils.CuboidCfg(size=tuple(size), visual_material=material)
            return VisualizationMarkers(VisualizationMarkersCfg(prim_path=f"/Visuals/{prim}", markers={"m": box_cfg}))

        out = {f"lamp_{name}": make(f"rp_lamp_{name}", LAMP_SIZE, rgb, True) for name, rgb in _LAMPS.items()}
        out["tape"] = make("rp_tape", TAPE_MARKER_SIZE, COLOR_TAPE, False)
        return out


DISPLAY = _Display()
