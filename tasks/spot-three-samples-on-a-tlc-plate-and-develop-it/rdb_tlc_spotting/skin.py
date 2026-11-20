"""Show what the rules have made: each lane's spot (its sample's colour, the drawn size nearest its diameter, drawn
on the tick), bare glass where a lane was gouged, and the sample in each filled spotter's tip. The marks are drawn in
the scene file, hidden, on the plate and the spotters (``tools/build_assets.py``); the adapter switches the right
ones visible every step. Display only: the scorer never reads them. Same technique as the ukulele package's lamps
(``UsdGeom.Imageable(prim).MakeVisible()`` / ``MakeInvisible()``).
"""

import math

from . import layout as L

COLOURS = (*L.DYES, "mixed")
SPOT_SIZES = (.00125, .00175, .00225, .00275, .00325, .00375)     # as tools/build_assets.py draws them


def colour(sample):
    return "mixed" if sample == "mixed" else L.DYES[sample]


def wanted(grade):
    """The names of the marks to show for a scorer state."""
    names = set()
    for k, lane in enumerate(grade["lanes"]):
        tx, ty, _ = L.tick_point(k)
        near = [s for s in grade["spots"] if math.hypot(s["x"] / 1000 - tx, s["y"] / 1000 - ty) <= L.LANE_HALF]
        if near:
            spot = min(near, key=lambda s: math.hypot(s["x"] / 1000 - tx, s["y"] / 1000 - ty))
            size = min(range(len(SPOT_SIZES)), key=lambda i: abs(SPOT_SIZES[i] - spot["diameter_mm"] / 1000))
            names.add(f"plate/spot_{k + 1}_{colour(spot['sample'])}_{size}")
        if lane["gouged"] or any(math.hypot(g["x"] / 1000 - tx, g["y"] / 1000 - ty) <= L.LANE_HALF for g in grade["gouges"]):
            names.add(f"plate/bare_{k + 1}")
    for k, sample in enumerate(grade["loads"]):
        if sample is not None:
            names.add(f"spotter_{k + 1}/load_{colour(sample)}")
    return names


class MarkSkin:
    def __init__(self, stage, actors):
        from pxr import UsdGeom

        self.marks = {}
        paths = [f"plate/spot_{k}_{c}_{s}" for k in (1, 2, 3) for c in COLOURS for s in range(len(SPOT_SIZES))]
        paths += [f"plate/bare_{k}" for k in (1, 2, 3)] + [f"spotter_{k}/load_{c}" for k in (1, 2, 3) for c in COLOURS]
        for name in paths:
            prim = stage.GetPrimAtPath(f"{actors}/{name}")
            if not prim.IsValid():
                raise RuntimeError(f"Mark {name} is missing under {actors}")
            self.marks[name] = UsdGeom.Imageable(prim)
        self.shown = set()

    def update(self, grade):
        names = wanted(grade)
        for name in names - self.shown:
            self.marks[name].MakeVisible()
        for name in self.shown - names:
            self.marks[name].MakeInvisible()
        self.shown = names
