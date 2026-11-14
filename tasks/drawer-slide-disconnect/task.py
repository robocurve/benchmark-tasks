# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Take a drawer off its ball-bearing slides and set it down on the table.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A plywood drawer bay (300 x 316 x 200 mm, front at x 0.38) stands fixed on the table on an 85 mm plinth. Each side
  has a slide channel (web, bottom rail) on the bay. The drawer (280 x 240 x 75 mm, 1.2 kg) is a free body: its two
  steel inner slide members rest on the channel rails and nothing else holds it. It stands pulled out 129 mm.
- Each inner member carries a spring-return release lever with a latch dog at its rear end. The left (+y) lever is a
  tab pushed UP, the right (-y) lever a tab pushed DOWN; both move 6 mm. A dog that is not lifted (left) or lowered
  (right) at least 4.5 mm out of the way stops against a fixed block in its channel at 130 mm, so both levers must be
  held while the drawer is drawn out. Past the block the inner members run free; the drawer leaves the rails when its
  rear is 290 mm from closed and then hangs from the grippers alone.
- Full score: the drawer is off both slides, flat on the table, at rest, and both grippers have let go.
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import linkkit as lk  # noqa: E402
import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
VARIANT = os.environ.get("DEMO_VARIANT", "")
MM = 0.001

B = 0.38  # bay front, x
Z0 = 0.10  # drawer bottom above the table while on the rails
START_EXT = 0.129
STOP = 0.130  # latch dogs meet their blocks (m of extension)
D_LEN = 0.28  # drawer depth
TRAVEL = 0.006
ON = 0.0045  # lever pressed
BAY_CLEAR = 0.010  # drawer rear this far in front of the bay front counts as off the slides
FLAT_TOL = 0.004  # bottom corners within this of the table top
CLEAR_M = 0.03
TCP_STILL = 0.01
DRAWER_STILL = 0.005
HOLD_S = 1.0
EPS = 1e-6

WOOD = (0.70, 0.55, 0.35)
PLY = (0.86, 0.76, 0.56)
STEEL = (0.62, 0.63, 0.66)
RED = (0.85, 0.15, 0.12)
BLUE = (0.15, 0.30, 0.85)


def bx(x0, x1, y0, y1, z0, z1, color=None):
    """Box from drawer-frame ranges (m); z is measured from the drawer bottom, lifted by Z0 to the tree frame."""
    return lk.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2 + Z0), color=color)


def bay_box(x0, x1, y0, y1, z0, z1, color=None):
    return lk.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), color=color)


# Bay: plinth, walls, top, back, and per side a slide channel (8 mm web, 11 mm-wide bottom rail under the inner
# member, latch block). Channels run from 20 mm in front of the bay face to its back.
_b = lk.Tree("bay", pos=(B, 0.0, TOP), fixed=True, color=WOOD)
_chan = []
for _s in (1.0, -1.0):
    _lo, _hi = sorted((_s * 0.1215, _s * 0.1454))
    _wl, _wh = sorted((_s * 0.1372, _s * 0.1454))
    _bl, _bh = sorted((_s * 0.1330, _s * 0.1372))
    _chan += [
        bay_box(-0.02, 0.288, _lo, _hi, 0.1075, 0.1145, color=STEEL),                      # bottom rail
        bay_box(-0.02, 0.288, _wl, _wh, 0.1075, 0.1555, color=STEEL),                      # web
    ]
    if _s > 0:
        _chan.append(bay_box(0.126, 0.136, _bl, _bh, 0.1145, 0.1200, color=STEEL))         # block, left (dog drops onto it)
    else:
        _chan.append(bay_box(0.126, 0.136, _bl, _bh, 0.1450, 0.1555, color=STEEL))         # block, right (dog rises into it)
_b.link("bay", [
    bay_box(0.0, 0.30, -0.158, 0.158, 0.0, 0.085),                  # plinth
    bay_box(0.0, 0.30, 0.1454, 0.158, 0.085, 0.20),                 # side walls
    bay_box(0.0, 0.30, -0.158, -0.1454, 0.085, 0.20),
    bay_box(0.0, 0.30, -0.158, 0.158, 0.19, 0.20),                  # top
    bay_box(0.288, 0.30, -0.158, 0.158, 0.085, 0.20),               # back
] + _chan, mass=8.0)
BAY = _b.build()

# Drawer: a free articulation (drawer body + two levers). Design pose = closed; placed START_EXT out.
_t = lk.Tree("drawer", pos=(B - START_EXT, 0.0, TOP), color=PLY)
_t.link("drawer", [
    bx(0.0, 0.015, -0.12, 0.12, 0.0, 0.060),                        # front (inset)
    bx(0.015, 0.28, -0.12, 0.12, 0.0, 0.006),                       # bottom
    bx(0.015, 0.28, 0.108, 0.12, 0.0, 0.065),                       # walls
    bx(0.015, 0.28, -0.12, -0.108, 0.0, 0.065),
    bx(0.268, 0.28, -0.12, 0.12, 0.0, 0.065),                       # back
    bx(0.015, 0.07, 0.12, 0.1235, 0.0, 0.065, color=STEEL),         # left bracket leg and flange (on top)
    bx(0.015, 0.07, 0.1235, 0.145, 0.060, 0.065, color=STEEL),
    bx(0.015, 0.07, -0.1235, -0.12, 0.034, 0.039, color=STEEL),     # right bracket: flange with a slot for the stem
    bx(0.015, 0.07, -0.145, -0.1368, 0.034, 0.039, color=STEEL),
    bx(0.015, 0.07, -0.1330, -0.1235, 0.034, 0.039, color=STEEL),
    lk.box((0.23, 0.0127, 0.035), (0.155, 0.12635, 0.1325), color=STEEL),   # inner slide members (on the rails)
    lk.box((0.23, 0.0127, 0.035), (0.155, -0.12635, 0.1325), color=STEEL),
], mass=1.2, color=PLY, contact_offset=0.001, friction=0.5)
_t.link("levL", [bx(0.010, 0.045, 0.1235, 0.145, 0.031, 0.034),               # tab (pushed UP, pad span to flange 34 mm)
                 bx(0.045, 0.278, 0.1330, 0.1368, 0.028, 0.036),              # latch bar along the member
                 bx(0.266, 0.278, 0.1330, 0.1368, 0.0165, 0.036)],            # dog (hangs down at the rear)
        mass=0.02, color=RED, contact_offset=0.001)
_t.link("levR", [bx(0.010, 0.045, -0.145, -0.1235, 0.065, 0.068),             # tab (pushed DOWN)
                 bx(0.045, 0.052, -0.1368, -0.1330, 0.0285, 0.068),           # stem through the flange slot
                 bx(0.052, 0.278, -0.1368, -0.1330, 0.0285, 0.0375),          # latch bar
                 bx(0.266, 0.278, -0.1368, -0.1330, 0.0285, 0.0485)],         # dog (stands up at the rear)
        mass=0.02, color=BLUE, contact_offset=0.001)
_t.prismatic("pressL", "drawer", "levL", anchor=(0.0275, 0.134, Z0 + 0.0325), axis=(0, 0, 1), limits=(0.0, TRAVEL),
             stiffness=1200.0, damping=8.0)
_t.prismatic("pressR", "drawer", "levR", anchor=(0.0275, -0.134, Z0 + 0.0665), axis=(0, 0, -1), limits=(0.0, TRAVEL),
             stiffness=1200.0, damping=8.0)
DRAWER = _t.build()
OBJECTS = [BAY, DRAWER]

_CORNERS = [(x, y, z) for x in (0.0, D_LEN) for y in (-0.145, 0.145) for z in (Z0, Z0 + 0.068)]
_BOTTOM = [(x, y, Z0) for x in (0.0, D_LEN) for y in (-0.12, 0.12)]


def measure(d, tools) -> dict:
    bz = [float(d.point("drawer", p)[2]) - TOP for p in _BOTTOM]
    rear = max(float(d.point("drawer", p)[0]) for p in _CORNERS)
    return {
        "lL": float(d.q["pressL"]), "lR": float(d.q["pressR"]),
        "rear_x": rear, "bottom_z": bz,
        "d_speed": float(d.max_speed()),
        "clear": {s: float(lk.clearance(tools[s], d)) for s in ("left", "right")},
        "tcp_speed": {s: float(tools[s].speed) for s in ("left", "right")},
    }


def off_slides(m: dict) -> bool:
    return m["rear_x"] <= B - BAY_CLEAR


def on_table(m: dict) -> bool:
    return all(abs(z) <= FLAT_TOL for z in m["bottom_z"])


class Record:
    """Whole-run record (a linkkit History fed every control step, cleared only by the simulator reset)."""

    def __init__(self):
        self.H = lk.History()
        self.last = None
        self.n = 0
        self.trace: list = []

    def reset(self) -> None:
        self.H.reset()
        self.last, self.n, self.trace = None, 0, []

    def update(self, m: dict, t: float | None = None) -> None:
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        self.last = m
        H.ever("both_pressed", m["lL"] >= ON and m["lR"] >= ON)
        H.peak("lL_max", m["lL"])
        H.peak("lR_max", m["lR"])
        H.held("down", off_slides(m) and on_table(m) and m["d_speed"] < DRAWER_STILL)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    """Final state only: where the drawer is at the end. Nothing earlier in the run is held against it."""
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if not off_slides(m):
        return 0.0, "the drawer is still on its slides at the end"
    if not (on_table(m) and H.runs.get("down", 0.0) >= HOLD_S - EPS and m["d_speed"] < DRAWER_STILL):
        return 0.25, "the drawer is off both slides but not flat on the table and at rest for 1 s"
    if H.runs.get("released", 0.0) >= HOLD_S - EPS:
        return 1.0, "drawer off both slides, flat on the table, at rest, both grippers let go"
    return 0.5, "drawer off the slides and at rest on the table; not yet: both grippers 30 mm clear and still 1 s"


REC = Record()


@lk.on_reset
def _reset(tick):
    REC.reset()


@lk.on_step
def _track(tick):
    d = tick.states["drawer"]
    m = measure(d, tick.tools)
    REC.update(m)
    if REC.n % 5 == 0:
        REC.trace.append((round(tick.t, 2), round(1000 * (B - m["rear_x"] + D_LEN), 1), round(1000 * m["lL"], 2),
                          round(1000 * m["lR"], 2), [round(1000 * z) for z in m["bottom_z"]],
                          [round(1000 * m["clear"][s]) for s in ("left", "right")]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():
    return {}


def demo_report() -> dict:
    return {"ladder": ladder(REC), "peaks": dict(REC.H.peaks), "evers": dict(REC.H.evers), "runs": dict(REC.H.runs),
            "trace": REC.trace[::2], "variant": VARIANT}


# -- scripted demo ----------------------------------------------------------------------------------------------

VARIANTS = ("", "nothing", "left_only", "right_only", "pull_release")
SQUEEZE_W = 0.0255
OPEN_W = 0.094
LET_GO_W = 0.045
CONTACT = 0.0115  # pad contact centre past the TCP along the approach
PITCH = 15.0  # deg the approach points down
PULL = 0.183  # 129 -> 312 mm out: the drawer's rear is 32 mm clear of the rails
PULL_RELEASE = 0.095  # the old shortcut: pull a bit and let go
LOWER = 0.098
LEFT_Z = 0.048  # mid-height of the pad pair (tab underside 31 mm, flange top 65 mm)
RIGHT_Z = 0.051  # (flange underside 34 mm, tab top 68 mm)
WALL_MID = 0.1325
LEV_X = 0.0275


def demo_script(ctx):
    from linkdemo import grip_frame

    yield from ctx.wait(1.0)
    seen = ctx.observe()

    def P(loc):
        return seen.point("drawer", "drawer", (loc[0], loc[1], loc[2] + Z0))

    p = math.radians(PITCH)
    a = np.array([math.cos(p), 0.0, -math.sin(p)])
    r_right = grip_frame(a, (0, 0, 1))
    r_left = grip_frame(a, (0, 0, 1), ref=np.column_stack([-r_right[:, 0], -r_right[:, 1], r_right[:, 2]]))
    var = VARIANT
    use_left = var not in ("nothing", "right_only")
    use_right = var not in ("nothing", "left_only")

    def side(arm, c, rot, use):
        if not use:
            return
        tcp = c - a * CONTACT
        out = tcp - np.array([0.08, 0.0, 0.0])
        yield from arm.move(out, rot, width=OPEN_W, duration=3.0)
        yield from arm.move(out, rot, duration=1.0)
        yield from arm.move(tcp, rot, width=OPEN_W, duration=1.5)
        yield from arm.grip(SQUEEZE_W, 1.0)
        yield from arm.wait(0.5)
        if var == "pull_release":
            yield from arm.move(tcp - [PULL_RELEASE, 0.0, 0.0], rot, duration=5.0)
            yield from arm.wait(0.5)
            yield from arm.grip(OPEN_W, 0.6)
            yield from arm.move(tcp - [PULL_RELEASE + 0.07, 0.0, 0.0], rot, duration=1.5)
            return
        yield from arm.move(tcp - [PULL - 0.03, 0.0, 0.0], rot, duration=6.0)   # dogs pass their blocks
        yield from arm.move(tcp - [PULL, 0.0, 0.0], rot, duration=2.0)           # the rear leaves the rails
        down = tcp - [PULL, 0.0, LOWER]
        yield from arm.move(down, rot, duration=4.0)                              # set it down on the table
        yield from arm.wait(1.0)
        yield from arm.grip(LET_GO_W, 1.0)
        yield from arm.wait(0.5)
        yield from arm.move(down + [0.0, 0.0, 0.09], rot, duration=1.5)

    yield from ctx.both(side(ctx.left, P((LEV_X, WALL_MID, LEFT_Z)), r_left, use_left),
                        side(ctx.right, P((LEV_X, -WALL_MID, RIGHT_Z)), r_right, use_right))
    yield from ctx.wait(4.0)
