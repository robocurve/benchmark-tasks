"""Show the clip-on tuner's lit lamps. The note lamps (F#, G, G#) and the green in-tune lamp are drawn in the
scene file, hidden (``/Fixture/Lamps``); the lit ones are switched visible whenever the display changes. Display
only: the scorer never reads them (``UsdGeom.Imageable(prim).MakeVisible()`` / ``MakeInvisible()``).
"""

from . import layout as L

NAMES = (*L.NOTE_LAMP.values(), L.IN_TUNE_LAMP[0])


class LampSkin:
    def __init__(self, stage, scope):
        from pxr import UsdGeom

        self.lamps = {}
        for name in NAMES:
            prim = stage.GetPrimAtPath(f"{scope}/{name}")
            if not prim.IsValid():
                raise RuntimeError(f"Tuner lamp {name} is missing under {scope}")
            self.lamps[name] = UsdGeom.Imageable(prim)
        self.shown = None

    def update(self, lit):
        """``lit``: {lamp name: on}."""
        key = tuple(bool(lit.get(name, False)) for name in NAMES)
        if key == self.shown:
            return
        for name, on in zip(NAMES, key):
            if on:
                self.lamps[name].MakeVisible()
            else:
                self.lamps[name].MakeInvisible()
        self.shown = key
