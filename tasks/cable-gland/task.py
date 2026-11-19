"""On the station table, in front of the arms: a grey polycarbonate enclosure (120 deep x 160 wide x 60 mm tall,
3 mm walls, lid off, fixed in place) with an M20 cable gland fitted in the middle of its near wall (the wall facing
the arms), its axis 35 mm above the table and pointing straight at the arms. The gland (black-grey polyamide) has a
domed cap nut (24 mm across flats) which is loose, on the thread's first turn: its seal is open (14.5 mm), ready for a cable.
In front of the box a grey PVC-sheathed control cable (3 cores x 1.5 mm2, 8 mm round) lies on the table in a curve.
Its far end is held in a black cable clamp on the table to the front right; its free end is prepared: the sheath stripped
back 75 mm, the three cores (brown, blue, green-yellow) taped together.

The cap nut tightens clockwise (as seen from the arms). As it turns down, the seal closes on the cable and grips it;
3 turns bring the seal onto the cable, and from there between 1 and 1.5 turns more makes it snug. Past 1.5 turns it
is over-tightened, and turned 1.75 turns or more past first contact, the seal crushes the sheath.

Task: fit the cable through the cable gland into the box until its sheath reaches at least 5 mm past the gland's
inner end, then tighten the cap nut snug, between the marks, neither short of them nor past them. Then let go of both.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cg_parts as cg  # noqa: E402

from yam_arena.task_file import TABLE_TOP  # noqa: E402

cg.install()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

MM = 1e-3
GLAND_ORIGIN = (0.27, 0.0, TOP + cg.AXIS_H)  # the gland frame: on its axis, in the wall's outer face
CLAMP_POS = (0.10, -0.24, TOP + cg.CLAMP_H)  # the cable clamp on the table, front right of the box (the cable's centre
# where it leaves the clamp) ...
CLAMP_YAW = math.radians(30.0)  # ... pointing forward-left
SHEATH_END_GOAL = (0.16, 0.08, math.radians(-15.0))  # the sheath's cut end lies here (env x, y, heading): the cable
# arcs left in front of the box, its cores' tip ~4 cm in front of the wall and ~4 cm left of the gland's axis
# (Tried and dropped, for the twist rule: a clamp on a 120 mm stand, and a clamp straight behind the gland with the
# cable bowed; the slack's bow then had to be dragged straight on every push and the cable slid through the grip,
# demo runs 21-31. Kept as options: a drape from a raised clamp and a straight end piece.)
STRAIGHT_LINKS = 0  # the last sheath links laid straight
DROP_LINKS = 0  # a clamp on a stand: the cable drapes down to the table over these links, turning ...
DROP_YAW = 0.0  # ... by this much (deg)
ARM_BASES = ((0.0, 0.305), (0.0, -0.305))


def _drop_pitches(amp_deg: float):
    """Pitch (deg, + = down) of the draping links: up to amp_deg and back to level, a half sine."""
    return [amp_deg * math.sin(math.pi * (i + 0.5) / DROP_LINKS) for i in range(DROP_LINKS)]


def _drop_amp() -> float:
    """The half-sine amplitude that brings the cable from the clamp's height down onto the table."""
    if DROP_LINKS == 0:
        return 0.0
    lo, hi = 0.0, 89.0
    for _ in range(60):
        mid = (lo + hi) / 2
        drop = sum(cg.LINK_LEN * math.sin(math.radians(p)) for p in _drop_pitches(mid))
        lo, hi = (mid, hi) if drop < cg.CLAMP_H - cg.CABLE_R else (lo, mid)
    return (lo + hi) / 2


def _bow(a, b, c, t):
    """Heading (deg) along the lying cable, t from 0 to 1: a cubic a t + b t^2 + c t^3."""
    import numpy as np

    return a * t + b * t * t + c * t ** 3


def _headings(amp: float, b: float, c: float, d: float):
    """(yaw, pitch) per link: the drape (if any), then the lying links bowed (see _bow), the cores straight on."""
    drop = [(DROP_YAW * (i + 0.5) / DROP_LINKS, p) for i, p in enumerate(_drop_pitches(amp))]
    n = cg.SHEATH_LINKS - DROP_LINKS - STRAIGHT_LINKS
    lie = [(DROP_YAW + float(_bow(b, c, d, t)), 0.0) for t in (i / (n - 1) for i in range(n))]
    lie += [(math.degrees(SHEATH_END_GOAL[2]), 0.0)] * STRAIGHT_LINKS
    return drop + lie + [lie[-1]] * cg.CORE_LINKS


def lay_cable():
    """The gentlest lying curve whose sheath end lands within 3 mm and 15 degrees of SHEATH_END_GOAL, clear of the box
    and the arm bases: a coarse grid, then finer grids round the best (pure numpy)."""
    import numpy as np

    amp = _drop_amp()
    drop = _drop_pitches(amp)
    yaws = [math.radians(DROP_YAW * (i + 0.5) / DROP_LINKS) for i in range(DROP_LINKS)]
    x0 = CLAMP_POS[0] + cg.CLAMP_OUT * math.cos(CLAMP_YAW)  # (no drape: the cable starts at the clamp's exit)
    y0 = CLAMP_POS[1] + cg.CLAMP_OUT * math.sin(CLAMP_YAW)
    n = cg.SHEATH_LINKS - DROP_LINKS - STRAIGHT_LINKS
    t = np.arange(n) / (n - 1)
    # the bow ends where the straight end piece begins
    gx = SHEATH_END_GOAL[0] - STRAIGHT_LINKS * cg.LINK_LEN * math.cos(SHEATH_END_GOAL[2])
    gy = SHEATH_END_GOAL[1] - STRAIGHT_LINKS * cg.LINK_LEN * math.sin(SHEATH_END_GOAL[2])
    centre, step = np.zeros(3), 10.0
    for it in range(5):
        g = np.arange(-40, 41) * step if it == 0 else np.arange(-10, 11) * step
        B, C, D = (a.ravel() for a in np.meshgrid(centre[0] + g, centre[1] + g, centre[2] + g, indexing="ij"))
        H = CLAMP_YAW + np.radians(DROP_YAW + _bow(B[:, None], C[:, None], D[:, None], t))
        x = x0 + np.cumsum(cg.LINK_LEN * np.cos(H), axis=1)
        y = y0 + np.cumsum(cg.LINK_LEN * np.sin(H), axis=1)
        err = np.hypot(x[:, -1] - gx, y[:, -1] - gy)
        herr = np.abs((H[:, -1] - SHEATH_END_GOAL[2] + math.pi) % (2 * math.pi) - math.pi)
        bend = np.abs(np.diff(H, axis=1)).max(axis=1)
        clear = ~((x > GLAND_ORIGIN[0] - 0.06) & (np.abs(y) < cg.BOX_WIDTH / 2 + 0.03)).any(axis=1)
        for bx, by in ARM_BASES:
            clear &= (np.hypot(x - bx, y - by) > 0.15).all(axis=1)
        cost = err / MM + np.degrees(herr) + 0.2 * np.degrees(bend)
        cost[~clear | (bend > math.radians(cg.BEND_LIMIT * 0.6))] = np.inf
        k = int(np.argmin(cost))
        centre, step = np.array([B[k], C[k], D[k]]), step / 4
    if not (err[k] < 3 * MM and herr[k] < math.radians(15.0) and np.isfinite(cost[k])):
        raise RuntimeError(f"cable layout: no curve fits ({err[k] / MM:.1f} mm, {math.degrees(herr[k]):.1f} deg)")
    return float(err[k]), _headings(amp, *map(float, centre))


CABLE_HEADINGS = lay_cable()[1]
cg.CABLE_HEADINGS[:] = CABLE_HEADINGS  # (written into the cable's USD when it spawns)

OBJECTS = [
    cg.part("box", "box", GLAND_ORIGIN),
    cg.part("gland", "gland", GLAND_ORIGIN),
    cg.part("cable", "cable", CLAMP_POS, yaw=CLAMP_YAW),
]

# -- scoring (final state; "crushed" is remembered over the whole run) ---------------------------------------------
CREDIT = {"inserted": 0.25, "holds": 0.5, "overtight": 0.75, "snug": 1.0}


def inserted(objects) -> bool:
    """A: the cable runs through the gland and its sheath reaches at least 5 mm past the gland's inner end; measured
    before the pull test (a cable that fails it is pulled back out: in a real trial the sheath is measured first)."""
    c, g = objects["cable"], objects["gland"]
    x = max(c.sheath_x, g.pull_sheath_x) if g.pull == "failed" else c.sheath_x
    return bool(getattr(c, "through", False) and x >= cg.GLAND_INNER_X + cg.SHEATH_PAST)


def snug(objects) -> bool:
    """B: the nut between the marks: 1 to under 1.5 turns past the seal's first contact."""
    g = objects["gland"]
    return bool(g.contact_turns is not None and cg.SNUG_TURNS[0] <= g.past_contact < cg.SNUG_TURNS[1])


def overtight(objects) -> bool:
    """The nut past the upper mark (1.5 turns past first contact) but not yet crushing the sheath (1.75 turns)."""
    g = objects["gland"]
    return bool(g.contact_turns is not None and g.past_contact >= cg.SNUG_TURNS[1] and not g.crushed)


def holds(objects) -> bool:
    """C: the latest pull test (30 N, hands off) moved the sheath less than 2 mm."""
    return objects["gland"].pull == "passed"


def score(objects) -> float:
    """0 crushed (ever turned 1.75 turns past first contact) or the sheath not 5 mm past the gland's inner end;
    0.25 sheath past; 0.5 also holds the pull test; 0.75 holds but the nut past the upper mark (over-tightened);
    1.0 holds and the nut between the marks (snug)."""
    g = objects["gland"]
    if g.crushed or not inserted(objects):
        return 0.0
    if not holds(objects):
        return CREDIT["inserted"]
    if snug(objects):
        return CREDIT["snug"]
    if overtight(objects):
        return CREDIT["overtight"]
    return CREDIT["holds"]


def success(objects) -> bool:
    """Full score (inserted, holds, snug; the pull test needs both hands off) and the cable and the nut still."""
    return score(objects) >= 1.0 - 1e-9 and objects["cable"].resting() and objects["gland"].resting()


def distance(objects) -> float:
    """Metres the sheath's end still has to go to 5 mm past the gland's inner end (a progress measure)."""
    return max(0.0, cg.GLAND_INNER_X + cg.SHEATH_PAST - objects["cable"].sheath_x)
