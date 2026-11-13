"""seg7: a 7-segment numeric display built from the boxes of a robokit Part, lit by showing/hiding segment boxes.

Usage (task file; robokit.py and seg7.py next to it, or kit/ on sys.path):

    from robokit import part
    from seg7 import Seg7

    DISP = Seg7(centre=(0.0, 0.0, 0.041), digits=3, height=0.032, dp_after=2, up="+x")
    housing = [((0, 0, 0.02), (0.07, 0.12, 0.04), DARK)]            # the Part's other boxes come first
    PANEL = part("display", pos=(0.34, 0.15, TOP), mass=5.0, boxes=[*housing, *DISP.boxes(first=len(housing))])

    DISP.show("display", "386")   # from an on_step/on_reset hook: shows 38.6 (decimal point is fixed after digit 2)
    DISP.read("display")          # -> "386": decodes which segments are visible (what the cameras see)

- ``centre`` is the middle of the digit row on the face, in the Part frame; segments are 1 mm thick boxes sitting on
  that plane (the face is horizontal: z is the face normal).
- ``up`` is the direction, in the Part frame, of the digits' tops: "+x" (tops away from the robots, read upright
  from the top camera behind the arms), "-x", "+y" or "-y". Digits run left to right as seen with tops up.
- Every segment box always exists; ``show`` hides the unlit ones (robokit.set_visible). Hidden boxes still collide,
  so keep the display where the arms don't work. Characters: 0-9, "-", " ". ``show`` only touches prims that change.
- ``boxes(first=...)`` must be told the index its first box will have in the Part's box list (robokit names box
  prims body/box_<index>).
"""

from __future__ import annotations

import numpy as np

SEGMENTS = "abcdefg"
GLYPHS = {"0": "abcdef", "1": "bc", "2": "abdeg", "3": "abcdg", "4": "bcfg", "5": "acdfg", "6": "acdefg",
          "7": "abc", "8": "abcdefg", "9": "abcdfg", "-": "g", " ": ""}
_DECODE = {frozenset(v): k for k, v in GLYPHS.items()}
RED_LED = (1.0, 0.12, 0.08)


class Seg7:
    def __init__(self, centre, digits: int, height: float = 0.03, dp_after: int | None = None, up: str = "+x",
                 color=RED_LED, thickness: float | None = None):
        self.centre = np.asarray(centre, float)
        self.digits, self.h, self.dp_after, self.color = digits, height, dp_after, color
        self.w = 0.55 * height                       # digit width
        self.t = thickness or 0.14 * height          # segment stroke
        self.gap = 0.30 * height                     # space between digits
        upv = {"+x": (1, 0), "-x": (-1, 0), "+y": (0, 1), "-y": (0, -1)}[up]
        self._up = np.array([*upv, 0.0])
        self._right = np.cross(self._up, [0, 0, 1.0])  # tops up, seen from above: right = up x z
        self.first = None
        self._shown: dict[str, str] = {}

    def _layout(self):
        """(digit, segment) -> (centre, size) in the Part frame; plus the decimal point."""
        h, w, t = self.h, self.w, self.t
        pitch = w + self.gap
        x0 = -(self.digits - 1) / 2 * pitch
        local = {  # (right, up) centre, (along-right, along-up) size, in a digit's frame
            "a": ((0, h / 2 - t / 2), (w, t)), "g": ((0, 0), (w, t)), "d": ((0, -h / 2 + t / 2), (w, t)),
            "f": ((-w / 2 + t / 2, h / 4), (t, h / 2 - 1.2 * t)), "b": ((w / 2 - t / 2, h / 4), (t, h / 2 - 1.2 * t)),
            "e": ((-w / 2 + t / 2, -h / 4), (t, h / 2 - 1.2 * t)), "c": ((w / 2 - t / 2, -h / 4), (t, h / 2 - 1.2 * t)),
        }
        out = []
        for d in range(self.digits):
            for s in SEGMENTS:
                (cr, cu), (sr, su) = local[s]
                c = self.centre + (x0 + d * pitch + cr) * self._right + cu * self._up + [0, 0, 0.0005]
                size = np.abs(sr * self._right + su * self._up) + [0, 0, 0.001]
                if s in "adg":   # horizontal strokes: trim so they don't overlap the verticals at the corners
                    size = size - np.abs(1.6 * t * self._right)
                out.append(((d, s), tuple(c), tuple(size)))
        dp = None
        if self.dp_after is not None:
            c = self.centre + (x0 + (self.dp_after - 0.5) * pitch) * self._right + (-h / 2 + t / 2) * self._up \
                + [0, 0, 0.0005]
            dp = (tuple(c), tuple(np.abs(t * self._right + t * self._up) + [0, 0, 0.001]))
        return out, dp

    def boxes(self, first: int):
        """Box specs (centre, size, colour) for the Part; ``first`` is the index of the first one in the box list."""
        self.first = first
        segs, dp = self._layout()
        self._index = {key: first + i for i, (key, _, _) in enumerate(segs)}
        out = [(c, s, self.color) for _, c, s in segs]
        if dp is not None:
            out.append((dp[0], dp[1], self.color))
        return out

    def show(self, part_name: str, text: str) -> None:
        """Light ``text`` (exactly ``digits`` characters from 0-9, '-', ' ')."""
        from robokit import set_visible

        if len(text) != self.digits:
            raise ValueError(f"{text!r} must have {self.digits} characters")
        old = self._shown.get(part_name, None)
        for d, ch in enumerate(text):
            if old is not None and old[d] == ch:
                continue
            lit = GLYPHS[ch]
            for s in SEGMENTS:
                set_visible(part_name, f"body/box_{self._index[(d, s)]}", s in lit)
        self._shown[part_name] = text

    def read(self, part_name: str) -> str:
        """Decode the display from which segment prims are visible (what the cameras render); '?' if unreadable."""
        from pxr import UsdGeom

        from robokit import _shape_prim

        out = ""
        for d in range(self.digits):
            lit = frozenset(s for s in SEGMENTS if UsdGeom.Imageable(
                _shape_prim(part_name, f"body/box_{self._index[(d, s)]}")).ComputeVisibility() != UsdGeom.Tokens.invisible)
            out += _DECODE.get(lit, "?")
        return out

    def reset_cache(self) -> None:
        self._shown.clear()
