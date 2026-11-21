"""The score of a trial, from a nut on each splice and the cover. Pure NumPy.

The instruction asks only for the cover. The job is done right when the bare black splice is capped first: a wire nut
twisted on it so no copper shows (capping.py), with the white splice's nut still on, and the cover closed.

| Score | Outcome |
|---|---|
| 1.0  | A nut on each splice (the black splice capped: its nut holds and hides all the copper; the white splice's nut still on); the cover closed |
| 0.5  | A nut on each splice, the cover not closed |
| 0    | The black splice not capped (the cover closed over bare copper, or nothing done), or the white splice's nut off |

A nut that came off, or that bit with copper still showing at its mouth, leaves the splice bare (0). A wire left
sticking out of the box keeps the cover from closing (the cover collides with the wires), as it would on a real box.
"""

from __future__ import annotations

from dataclasses import dataclass

import layout as L


@dataclass
class Reading:
    capped: bool = False
    white_on: bool = False
    closed: bool = False
    score: float = 0.0


def cover_closed(q: float, latched: bool) -> bool:
    return bool(latched) and abs(q - L.CLOSED) <= L.COVER_LATCH + 1e-9


def judge(capped: bool, white_on: bool, closed: bool) -> Reading:
    r = Reading(bool(capped), bool(white_on), bool(closed))
    if not (r.capped and r.white_on):
        s = 0.0
    elif not r.closed:
        s = 0.5
    else:
        s = 1.0
    r.score = s
    return r
