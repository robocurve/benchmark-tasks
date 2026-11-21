"""The score, from the model's state each control step (task.json's `scoring`, line by line).

1.0 when, held for 1 s:
  - cream: at least 90 % of the wound-and-margin zone creamed (>= 0.05 mm), no single smear larger than 4 mm^2 (2 x 2 mm)
    outside the band (8 mm round the wound's edge), and the applicator dropped in the waste pot;
  - plaster: on the forearm with its pad over the whole creamed zone, the pad's centre within 4 mm of the wound's centre
    and its axis within 10 degrees of the arm's;
  - both liners off and in the waste pot; both wings tacked over every node and lying flat (no lifted end);
  - both grippers at least 3 cm from the plaster.
0.75: the cream done, the plaster's pad over the zone, both liners off, and at least one wing tacked over every node.
0.5: the cream done, the plaster's pad over the zone, one liner off and its wing tacked over every node.
0.25: the cream done.
Short cream (under 90 % covered; no smear past the band; the applicator in the pot, or never taken): 75-90 % covered,
the plaster's level as above but at most 0.5; under 75 % (or none), 0.25 if the plaster is otherwise 1.0 (held 1 s).
0 if, at any time: a gripper touched the wound (uncovered), or the pad's face or the adhesive; cream under the adhesive;
the bud dipped again after touching skin, or touched anything but the cream in the cup and the skin (the cup's wall,
rim or bare floor, the pot while still held, the trays, the riser, the underpad, the table, the plaster or its tabs, a
gripper); the plaster fell off the forearm, or was let fall from the air with adhesive bared. Otherwise 0.
"""

from __future__ import annotations

import numpy as np

import firstaid as F
import scene as S

COVER_MIN = 0.90
COVER_PART = 0.75  # short cream: at least this covered caps the score at 0.5; less (or none), 0.25 at most
STRAY_MAX = 4e-6  # m^2: the largest single smear allowed outside the band
CENTRE_MAX = 0.004  # m
AXIS_MAX = 10.0  # deg
CLEAR = 0.03  # m
HOLD_STEPS = 20  # 1 s at 20 Hz


def _wound_points():
    ys = np.linspace(-S.WOUND_HALF[0], S.WOUND_HALF[0], 9)
    ss = np.linspace(-S.WOUND_HALF[1], S.WOUND_HALF[1], 5)
    Y, Sv = np.meshgrid(ys, ss, indexing="ij")
    keep = S.wound_dist(Y, Sv) <= 0
    p, n = S.ARM.point(Y[keep], Sv[keep])
    return p + n * 0.0003, Y[keep], Sv[keep]


class Tracker:
    def __init__(self):
        self.wp, self.wy, self.ws = _wound_points()
        self.reset()

    def reset(self):
        self.latched = None
        self.pfull = 0
        self.plvl = 0.0
        self.lvl = 0.0
        self.facts = {}
        self.steps = 0

    def _covered(self, pl: F.Plaster):
        if pl.state != "skin":
            return np.zeros(len(self.wy), bool)
        y0, s0, psi = pl.place
        c, s = np.cos(psi), np.sin(psi)
        dy, ds = self.wy - y0, self.ws - s0
        return (np.abs(dy * c + ds * s) <= S.PL_LEN / 2) & (np.abs(-dy * s + ds * c) <= S.PL_WID / 2)

    def update(self, w: F.World, hands: dict):
        self.steps += 1
        pl, cr, app = w.pl, w.cream, w.app
        if self.latched is None:
            open_ = ~self._covered(pl)
            if open_.any():
                for h in hands.values():
                    if F.boxes_contain(h.posed(), self.wp[open_], pad=0.0005).any():
                        self.latched = "a gripper touched the wound"
            if pl.touched_face:
                self.latched = "a gripper touched the pad's face or the adhesive"
            elif pl.creamed.any():
                self.latched = "cream under the adhesive"
            elif cr.double_dip:
                self.latched = "the applicator was dipped again after touching skin"
            elif app.touched_other:
                self.latched = f"the cream end touched the {app.touched_other}"
            elif pl.state == "fallen":
                self.latched = "the plaster fell (off the forearm, or bared from the air)"
        cover, stray = cr.coverage(), cr.stray_area()
        clean = stray <= STRAY_MAX  # no smear past the band
        cream_ok = cover >= COVER_MIN and clean and app.state == "binned"
        # short cream: the applicator binned (or never taken), no smear past the band, but under 90 % covered
        short_ok = clean and app.state in ("binned", "pouch") and not cream_ok
        over = pl.covers_zone(cr)
        off = pl.pad_offset()
        centred = off is not None and off[0] <= CENTRE_MAX and off[1] <= AXIS_MAX
        liners_off = [k for k in F.SIDES if pl.liners[k].state != "on"]
        binned = all(pl.liners[k].state == "binned" for k in F.SIDES)
        wings = [k for k in F.SIDES if pl.wing_done(k)]
        nodes, _ = pl.nodes()
        clear = min(F.boxes_dist(h.posed(), nodes.reshape(-1, 3)) for h in hands.values()) >= CLEAR
        # the plaster's own level, as if the cream were done
        if over and centred and len(liners_off) == 2 and binned and len(wings) == 2 and clear:
            plvl = 1.0
        elif over and len(liners_off) == 2 and len(wings) >= 1:
            plvl = 0.75
        elif over and len(liners_off) >= 1 and any(k in wings for k in liners_off):
            plvl = 0.5
        else:
            plvl = 0.0
        self.pfull = self.pfull + 1 if plvl == 1.0 else 0
        if plvl == 1.0 and self.pfull < HOLD_STEPS:
            plvl = 0.75  # the full plaster counts once held 1 s
        if cream_ok:
            lvl = max(plvl, 0.25)
        elif short_ok and cover >= COVER_PART:
            lvl = min(plvl, 0.5)
        elif short_ok and plvl == 1.0:
            lvl = 0.25
        else:
            lvl = 0.0
        self.plvl = plvl
        self.lvl = lvl
        self.facts = dict(cover=round(cover, 3), stray_mm2=round(stray * 1e6, 1), app=app.state, load_ul=round(cr.load * 1e9, 1),
                          dips=cr.dips, plaster=pl.state, offset_mm=None if off is None else round(off[0] * 1000, 1),
                          axis_deg=None if off is None else round(off[1], 1), over=over,
                          liners={k: (pl.liners[k].state, round(pl.liners[k].p * 1000, 1)) for k in F.SIDES},
                          tacked={k: round(pl.wing_tacked_frac(k), 2) for k in F.SIDES}, wings=wings, clear=clear, plaster_lvl=plvl,
                          slid_mm=round(pl.slid * 1000, 1))

    def score(self) -> float:
        if self.latched:
            return 0.0
        return self.lvl

    def success_now(self) -> bool:
        return self.latched is None and self.lvl == 1.0
