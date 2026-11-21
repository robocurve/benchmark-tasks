"""The air rules and the breath scoring of the infant-bvm task (pure Python; task.py feeds it every 50 ms step).

No air is simulated. The bag's squeeze becomes volume, the volume reaches the lungs only through a seated mask and an
open airway, and the lungs move the chest (the flag's dot); nothing else moves the chest.

- Bag: V_bag = pi/4 (d_lr^2 + d_tb^2) x 60 mm from the walls' squeeze depths (29.5 mL at 25 mm), at most 150 mL.
- Lungs: a step's increase of V_bag (dV > 0) is added to V_lung only while the mask is seated and the head is in
  the airway window; V_lung is capped at 60 mL. It falls (time constant 0.5 s) only once the squeeze is let off (V_bag
  1 mL or more below the squeeze's peak, or under 2 mL; it stays let off until the bag is squeezed again) or the mask
  is not seated (the seal is broken); while a squeeze is held, it holds.
- Chest: rise = 0.25 mm per mL of V_lung.
- Breath detector: an episode starts when the rise reaches 5 mm; its onset is the last time the rise left 0.5 mm; it
  ends when the rise is back within 2 mm. From the onset to the end the mask must be seated and nothing but the mask
  and the hand holding it may touch the manikin ("clean"). Then: a peak over 12 mm is TOO BIG (whatever its timing);
  otherwise a peak reached 0.5 s to 2 s after the onset is a BREATH; anything else is neither.
"""

from __future__ import annotations

import math

import geometry as g

DT = 0.05                  # one control step
REL_EPS = 0.05             # mL per step: the bag being squeezed
HOLD_TOL = 1.0             # mL: a squeeze let off by more than this from its peak is released
SQUEEZED = 2.0             # mL: a bag squeezed less than this is open


class Lungs:
    """The bag -> lungs -> chest rule."""

    def __init__(self):
        self.v_lung = 0.0
        self.v_bag = None
        self.v_top = 0.0           # the current squeeze's peak bag volume
        self.released = True       # let off since that peak (latched until the bag is squeezed again)
        self.dv = 0.0

    def step(self, v_bag: float, seated: bool, airway_open: bool) -> float:
        dv = 0.0 if self.v_bag is None else v_bag - self.v_bag
        self.v_bag, self.dv = v_bag, dv
        if seated and airway_open and dv > 0:
            self.v_lung = min(g.LUNG_CAP_ML, self.v_lung + dv)
        if dv > REL_EPS:                                   # being squeezed: a new squeeze starts from here
            if self.released:
                self.v_top, self.released = v_bag, False
            self.v_top = max(self.v_top, v_bag)
        elif v_bag < self.v_top - HOLD_TOL or v_bag < SQUEEZED:
            self.released = True
        if self.released or not seated:
            self.v_lung *= math.exp(-DT / g.LUNG_TAU)
        return self.rise

    @property
    def rise(self) -> float:
        return g.RISE_PER_ML * self.v_lung


class Breaths:
    """Counts breaths and too-big breaths from the chest's rise, step by step."""

    def __init__(self):
        self.t = 0.0
        self.t_left = 0.0          # the last time the rise was within 0.5 mm
        self.ok_since = True       # seated and clean at every step since then
        self.open = None           # the episode in progress: dict(t0, peak, t_peak, ok)
        self.breaths, self.too_big, self.void = [], [], []

    def step(self, rise: float, seated: bool, clean: bool) -> None:
        self.t += DT
        ok = bool(seated and clean)
        if rise <= g.ONSET:
            self.t_left, self.ok_since = self.t, True
        else:
            self.ok_since = self.ok_since and ok
        e = self.open
        if e is None:
            if rise >= g.BREATH_START:
                self.open = dict(t0=self.t_left, peak=rise, t_peak=self.t, ok=self.ok_since)
            return
        e["ok"] = e["ok"] and ok
        if rise > e["peak"] + 5e-5:                       # a new peak (0.05 mm hysteresis on a held squeeze)
            e["peak"], e["t_peak"] = rise, self.t
        if rise <= g.BREATH_END:
            self._close(e)
            self.open = None

    def _kind(self, e) -> str:
        if not e["ok"]:
            return "void"
        if e["peak"] > g.BREATH_MAX:
            return "too_big"
        dt = e["t_peak"] - e["t0"]
        if g.PEAK_WINDOW[0] <= dt <= g.PEAK_WINDOW[1]:
            return "breath"
        return "void"

    def _close(self, e) -> None:
        kind = self._kind(e)
        rec = dict(onset=round(e["t0"], 2), peak_mm=round(e["peak"] * 1000, 2),
                   to_peak_s=round(e["t_peak"] - e["t0"], 2), end=round(self.t, 2), ok=e["ok"])
        {"breath": self.breaths, "too_big": self.too_big, "void": self.void}[kind].append(rec)

    def counts(self) -> tuple[int, int]:
        """(breaths, too big), counting an unfinished seated-and-clean episode already over 12 mm as too big."""
        big = len(self.too_big)
        if self.open is not None and self.open["ok"] and self.open["peak"] > g.BREATH_MAX:
            big += 1
        return len(self.breaths), big


def tier(n_breath: int, n_big: int, held_off: bool, seated_2s: bool) -> float:
    """The end-of-run score, given no zero latch."""
    if n_breath == 2 and n_big == 0:
        return 1.0 if held_off else 0.75
    if n_breath >= 1:
        return 0.5
    if seated_2s:
        return 0.25
    return 0.0


if __name__ == "__main__":
    # offline checks of the rule chain
    def run(profile, seated=lambda t: True, clean=lambda t: True, airway=lambda t: True):
        L, B = Lungs(), Breaths()
        peak = 0.0
        for k, d in enumerate(profile):
            t = k * DT
            r = L.step(g.bag_volume_ml(d, 0.0), seated(t), airway(t))
            peak = max(peak, r)
            B.step(r, seated(t), clean(t))
        return B.counts(), round(peak * 1000, 2), B.breaths + B.too_big + B.void

    def squeeze(depth, t_in=1.0, hold=0.3, t_out=0.7, wait=1.5):
        n_in, n_h, n_out, n_w = (round(x / DT) for x in (t_in, hold, t_out, wait))
        p = [depth * (k + 1) / n_in for k in range(n_in)] + [depth] * n_h
        p += [depth * (1 - (k + 1) / n_out) for k in range(n_out)] + [0.0] * n_w
        return p

    for name, prof in (("25 mm over 1 s", squeeze(0.025)), ("two", squeeze(0.025) * 2),
                       ("held at 7 mm", [0.007] * 5 + [max(0.007, x) for x in squeeze(0.025)] + [0.007] * 30),
                       ("two at 7 mm", ([0.007] * 5 + [max(0.007, x) for x in squeeze(0.025)] + [0.007] * 30) * 2),
                       ("20 mm", squeeze(0.020)), ("21 mm", squeeze(0.021)), ("31 mm", squeeze(0.031)),
                       ("33 mm", squeeze(0.033)), ("40 mm", squeeze(0.040)), ("snap 0.2 s", squeeze(0.025, 0.2, 0.0)),
                       ("held 3 s", squeeze(0.025, 1.0, 3.0))):
        print(f"{name:16s}", run(prof))
    print("unseated half way", run(squeeze(0.025), seated=lambda t: t < 0.6))
    print("chest touched", run(squeeze(0.025), clean=lambda t: not (0.5 < t < 0.6)))
