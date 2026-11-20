"""Where the hands go for the bike light (dry check (a) and (b) figures, 2026-11-20): the lever hand (right) from
behind, its approach tipped LEVER_TILT down, closed fingertips' ends on the tab's top; the light hand (left) from
above, pinching the light across its 31 mm at its middle; the button pressed by the left hand's closed fingertips
from above. A gripper rotation has the closing axis, the pads' width axis and the approach axis as columns."""

from __future__ import annotations

import math

import numpy as np

TIP = 0.013  # m the fingertips' ends stand ahead of the grasp point along the approach
LEVER_TILT = math.radians(45.0)
PRESS_BEHIND = 0.007  # m the fingertips press the tab this far behind the rail's rear end (its middle is 6 mm behind)
ABOVE = 0.0015  # m over the tab's (or the button's) top where the fingertips' ends stop before they press


def lever_rot() -> np.ndarray:
    a = np.array([math.cos(LEVER_TILT), 0.0, -math.sin(LEVER_TILT)])
    c = np.array([0.0, 1.0, 0.0])
    return np.column_stack([c, np.cross(a, c), a])


LIGHT_TURNED = True  # the light hand turned half a turn about its approach (the pads are alike)


def light_rot(turned: bool | None = None) -> np.ndarray:
    """From above, closing across the light (along y). (The dry checks of 12:52 had the columns (0,1,0), (-1,0,0),
    (0,0,-1): a reflection, not a rotation; its figures were redone at 13:15.)"""
    turned = LIGHT_TURNED if turned is None else turned
    r = np.column_stack([(0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0)]).astype(float)
    return r @ np.diag([-1.0, -1.0, 1.0]) if turned else r


def press_point(m) -> np.ndarray:
    """On the tab's top, where the lever hand's fingertips press it."""
    return np.array([m.RAIL_X0 - PRESS_BEHIND, m.BRACKET_Y, m.TAB_TOP])


def lever_touch(m) -> np.ndarray:
    """The lever hand's grasp point with its closed fingertips' ends ABOVE over the press point."""
    return press_point(m) + np.array([0.0, 0.0, ABOVE]) - TIP * lever_rot()[:, 2]


def light_grasp(m, slid: float = 0.0) -> np.ndarray:
    """The light hand's grasp point: the light's middle, ``slid`` m along the rail."""
    return np.asarray(m.LIGHT_AT, dtype=float) + np.array([slid, 0.0, 0.0])


def button_touch(m) -> np.ndarray:
    """The left hand's grasp point with its closed fingertips' ends ABOVE over the button (the light on the rail)."""
    return np.asarray(m.BUTTON_AT, dtype=float) + np.array([0.0, 0.0, 0.0015 + ABOVE + TIP])
