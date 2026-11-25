"""Where the two YAM stations stand. Pure Python (NumPy).

Each station keeps yam_arena's station frame: x forward from its arms, y to their left, z up, arm bases at
(0, +-0.305, 0.75), table top at z 0.7475. Station ``s`` maps to the world by ``world = origin(s) + Rz(heading(s)) @
local``. The world origin is the midpoint between the two station origins (a station's origin is the midpoint between
its arm bases, on the floor), so z is the same in every frame.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

STATIONS = ("a", "b")
SIDES = ("left", "right")
ARRANGEMENTS = ("facing", "side_by_side")
DEFAULT_SEPARATION = {"facing": 0.8, "side_by_side": 1.0}
# yam_arena.station: BASES, TABLE_TOP, TABLE_SIZE, TABLE_CENTER (kept here so this module needs no yam_arena import)
BASES_LOCAL = {"left": (0.0, 0.305, 0.75), "right": (0.0, -0.305, 0.75)}
TABLE_TOP = 0.7475
STATION_TABLE_SIZE = (1.10, 1.30)
STATION_TABLE_CENTER = (0.36, 0.0)


def rz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


@dataclass(frozen=True)
class Layout:
    """The two stations' placement. ``yaw_a``/``yaw_b`` are degrees on top of the arrangement's headings."""

    arrangement: str = "facing"
    separation: float | None = None  # metres between station origins; None: the arrangement's default
    offset: float = 0.0  # metres, across the separation axis (facing: along y; side by side: along x)
    yaw_a: float = 0.0
    yaw_b: float = 0.0

    def __post_init__(self) -> None:
        if self.arrangement not in ARRANGEMENTS:
            raise ValueError(f"layout must be one of {ARRANGEMENTS}, got {self.arrangement!r}")
        for name in ("offset", "yaw_a", "yaw_b"):
            if not math.isfinite(float(getattr(self, name))):
                raise ValueError(f"{name} must be finite")
        if self.separation is not None and not float(self.separation) > 0:
            raise ValueError(f"separation must be positive metres, got {self.separation!r}")

    # -- placement -----------------------------------------------------------------------------------------------
    @property
    def sep(self) -> float:
        return float(self.separation) if self.separation is not None else DEFAULT_SEPARATION[self.arrangement]

    def origin(self, station: str) -> np.ndarray:
        sign = -1.0 if _check(station) == "a" else 1.0
        if self.arrangement == "facing":
            return np.array([sign * self.sep / 2, sign * self.offset / 2, 0.0])
        return np.array([sign * self.offset / 2, -sign * self.sep / 2, 0.0])

    def heading(self, station: str) -> float:
        """Radians: the station's +x direction in the world, measured from world +x."""
        base = math.pi if (self.arrangement == "facing" and _check(station) == "b") else 0.0
        return base + math.radians(self.yaw_a if station == "a" else self.yaw_b)

    def rotation(self, station: str) -> np.ndarray:
        return rz(self.heading(station))

    def quat_xyzw(self, station: str) -> tuple[float, float, float, float]:
        h = self.heading(station)
        return (0.0, 0.0, math.sin(h / 2), math.cos(h / 2))

    # -- conversions ---------------------------------------------------------------------------------------------
    def to_world(self, station: str, local) -> np.ndarray:
        return self.origin(station) + self.rotation(station) @ np.asarray(local, float)

    def to_local(self, station: str, world) -> np.ndarray:
        return self.rotation(station).T @ (np.asarray(world, float) - self.origin(station))

    def dir_to_world(self, station: str, v) -> np.ndarray:
        return self.rotation(station) @ np.asarray(v, float)

    def dir_to_local(self, station: str, v) -> np.ndarray:
        return self.rotation(station).T @ np.asarray(v, float)

    def rot_to_world(self, station: str, r_local) -> np.ndarray:
        return self.rotation(station) @ np.asarray(r_local, float)

    def rot_to_local(self, station: str, r_world) -> np.ndarray:
        return self.rotation(station).T @ np.asarray(r_world, float)

    def yaw_to_world(self, station: str, yaw: float) -> float:
        return _wrap(yaw + self.heading(station))

    def yaw_to_local(self, station: str, yaw: float) -> float:
        return _wrap(yaw - self.heading(station))

    def bases(self, station: str) -> dict[str, np.ndarray]:
        """World positions of the station's two arm bases."""
        return {side: self.to_world(station, BASES_LOCAL[side]) for side in SIDES}

    def table_footprint(self, station: str) -> np.ndarray:
        """The station's own (single-station) table corners, world xy, shape (4, 2)."""
        (sx, sy), (cx, cy) = STATION_TABLE_SIZE, STATION_TABLE_CENTER
        corners = [(cx + dx * sx / 2, cy + dy * sy / 2, 0.0) for dx in (-1, 1) for dy in (-1, 1)]
        return np.array([self.to_world(station, c)[:2] for c in corners])

    def default_table(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """(size xy, centre xy) of the axis-aligned table covering both stations' own tables."""
        pts = np.vstack([self.table_footprint(s) for s in STATIONS])
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        return (float(hi[0] - lo[0]), float(hi[1] - lo[1])), (float((hi[0] + lo[0]) / 2), float((hi[1] + lo[1]) / 2))

    def with_overrides(self, **kw) -> Layout:
        """This layout with any of arrangement/separation/offset/yaw_a/yaw_b replaced (None keeps the field)."""
        kw = {k: v for k, v in kw.items() if v is not None}
        if "arrangement" in kw and kw["arrangement"] != self.arrangement and "separation" not in kw:
            kw["separation"] = None  # a different arrangement takes its own default separation
        return replace(self, **kw)

    def describe(self) -> str:
        parts = [f"{self.arrangement}, {self.sep:g} m apart"]
        if self.offset:
            parts.append(f"offset {self.offset:g} m")
        if self.yaw_a or self.yaw_b:
            parts.append(f"yaw A {self.yaw_a:g} deg, B {self.yaw_b:g} deg")
        return "; ".join(parts)


def _check(station: str) -> str:
    if station not in STATIONS:
        raise ValueError(f"station must be one of {STATIONS}, got {station!r}")
    return station


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi
