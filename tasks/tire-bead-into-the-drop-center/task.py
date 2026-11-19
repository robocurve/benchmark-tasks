"""A 12-inch kids' bike wheel (ISO 203 mm bead seat, 57-203 tire with its tube in) lies flat on a hub stand. One bead
is fully on the rim and the second is on except the last 60 degrees, which sits up on the rim's flange over the valve;
the valve stem leans 20 degrees off straight. A plastic tire lever lies on the table. "Finish putting this tire on the
rim." A trained mechanic pushes the bead that is already on down into the rim's center channel (the drop center, a
deeper groove round the middle of the rim) on the side away from the valve, which shortens its path round the rim and
frees about 4 mm of bead per 60 degrees, then, holding the bead beside the last section down (it pops back up there),
rolls the last section over the flange with the other hand, sets the valve straight toward the hub and lets go. A lever
pushed in under the bead can pinch the tube between the lever and the rim; a lever used with its tip kept out from under
the bead is allowed.
"""

import math
import os
import sys
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import guardkit as gk  # noqa: E402
import mechkit as mk  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled wheel; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Finish putting this tire on the rim."

# --- The wheel, in its own frame: origin on the hub axis in the rim's middle plane, z up (the wheel lies flat).
WX, WY = 0.28, 0.0  # hub axis on the table (m)
ZR = 0.085  # the rim's middle plane above the table
R_SEAT = 0.1015  # bead seat radius: ISO 203 mm bead seat diameter
R_FLANGE = 0.106  # top of the flange: 4.5 mm above the bead seat
R_WELL = 0.0975  # floor of the center channel (drop center): 4 mm below the bead seat
R_BED = 0.090  # the rim's spoke bed (inside of the rim), where the valve comes through
Z_FLANGE = 0.0105  # flange top above the middle plane (rim 21 mm wide outside)
Z_BEAD = 0.0085  # the upper bead's height when seated on its shoulder
DROP = 0.009  # m: the bead's travel from its shoulder down into the channel (the channel is in the rim's middle)

# Bead sectors that are on (each a 60-degree arc of the upper bead and the sidewall next to it, sliding down into the
# channel), centered at these angles (degrees from +x, counterclockwise seen from above). The last section, over the
# valve, is centered at 90 degrees (the robot's left).
SECTORS = {"s30": 30.0, "s150": 150.0, "s210": 210.0, "s270": 270.0, "s330": 330.0}
SPAN = math.radians(60.0)
LAST_AT = 90.0
SLACK_PER_SECTOR = (R_SEAT - R_WELL) * SPAN  # 4.2 mm: a 60-degree arc of bead moved 4 mm closer to the hub
# The last section needs 10 mm of bead to ride over the flange: lifting a 60-degree arc 4.5 mm (seat to flange top)
# takes 4.7 mm, and the bead's climb onto the flange at each end of the section about 2.5 mm more each, rounded up.
SLACK_NEED = 0.010
SEAT_ZONE = 0.003  # m: a sector within 3 mm of its shoulder or the channel floor clicks there (a third of the travel)
# The two arcs next to the last section (at 30 and 150 degrees) do not stay in the channel by themselves: lifting the
# last section pulls the bead beside it back up onto the shoulder, so one of them has to be held down while the other
# hand rolls the section over. The arcs farther round stay where they are pushed.
NEXT_TO_LAST = ("s30", "s150")
CREEP = 1300.0  # N/m: the pull back up onto the shoulder, 11.7 N at the channel floor (a firm palm holds it)
# Slack reaches the last section only through bead that is in the channel: a stretch still up on its shoulder is held
# there by the tube's pressure. So the slack counted runs from each end of the last section round the wheel, arc by
# arc, while each arc is at least 6 mm down (in the channel's click zone).
CHAIN = (("s30", "s330", "s270", "s210", "s150"), ("s150", "s210", "s270", "s330", "s30"))

# The last section: a flap of tire (upper sidewall and bead) turning about a line along x through the crown at the
# robot's left. 0 = the bead seated inside the flange; it starts turned 20 degrees up, its bead lip up on the flange.
HINGE_R, HINGE_Z = 0.145, 0.0
FLAP_START = math.radians(-20.0)
FLAP_LO = math.radians(-25.0)
BLOCK_AT = math.radians(-6.0)  # the lip reaches the flange's edge here: without slack it can't go further
FLAP_SEAT_ZONE = math.radians(2.5)  # within 2.5 degrees of seated it snaps in (the lip is below the flange top)
# The bead's resistance to the press (the review asked for a pairing whose mounting force is within the arm's reach).
# A hand presses the last section's sidewall straight down at PRESS_ARM from its hinge line. Rolling the section down
# to the flange's edge bends the sidewall (BEND_N at the hand); from there the lip has to stretch over the flange, and
# the press needed rises to SEAT_N just before it snaps in. SEAT_N is the in-spec ceiling of the real rig's setup check
# (README: the named pairing seats with at most 40 N on a scale), with margin. The YAM presses about 52 N straight down
# there at its servos' rated (continuous) torque, 107 N at peak (README, "Bead force").
BEND_N = 8.0
SEAT_N = 30.0
PRESS_ARM = 0.145 - 0.118  # m: hinge line to the middle press spot, seated (HINGE_R - its radius)

# The valve: a Schrader stem through the rim bed at 90 degrees, pointing to the hub, turning about the vertical when
# the tube drags it. Positive = its tip toward +x.
VALVE_LEN = 0.032  # exposed stem, rim bed to the end of the dust cap
VALVE_START = math.radians(20.0)
VALVE_TOL = math.radians(10.0)  # straight: within 10 degrees (the tip within 5.6 mm of the line to the hub)

# The tire lever on the table (a 11.5 cm nylon lever, spoon tip toward +x).
LEVER_POS = (0.20, 0.215)
LEVER = (0.115, 0.022, 0.008)
LEVER_TIP = (0.052, 0.0, 0.0)  # its spoon tip in the lever's own frame

# --- Scoring thresholds (reasons in score() and the README)
PINCH = gk.Latch(steps=1)  # no debounce: the lever lies on the table, so only a hand can put its tip there
TOE = 0.001  # m: a lever tip more than 1 mm inside the bead seat line (radius 10.15 cm), down in the rim where the tube
#              lies, can trap the tube against the rim; a tip that only hooks the bead (on the flange or the shoulder,
#              under the raised lip) does not reach it, so careful lever use is allowed
LIP_ARM = math.hypot(R_SEAT - HINGE_R, Z_BEAD - HINGE_Z)  # 4.4 cm from the section's hinge line to its bead lip
DT = 0.05  # s: one control step at 20 Hz
STRIKE_SPEED = 0.12  # m/s: the last section's bead lip driven toward its seat faster than 12 cm/s (its whole
#                      20-degree roll in about 0.13 s) is a strike, not a push: nearly 4 times a careful half-second
#                      roll (3.1 cm/s) and over 4 times the demo's fastest step (2.8 cm/s). A bead struck down over the
#                      flange can trap the tube under it and pinch it. The sim has no tube, so a strike is only a
#                      stand-in for a pinched tube:
STRIKE_CAP = 0.5  # it caps the score at 0.5 and never zeroes it on speed alone; the real trial's dunk test (40 psi, in
#                   water) checks the tube itself, and a leak there scores 0
PROGRESS_ARCS = 2  # partial credit: at least two bead arcs left down in the channel (the slack started: two arcs
#                    fully down free 8.4 mm of the 10 mm needed)
SWEEP = gk.Sweep(step=0.003, max_jump=0.05)

RUBBER = kit.Look((0.07, 0.07, 0.075), 0.85)
GUMWALL = kit.Look((0.72, 0.55, 0.33), 0.8)  # tan sidewall (a gumwall kids' tire): shows where each part sits
RIM = kit.Look((0.80, 0.81, 0.83), 0.25, metallic=1.0)
RIM_TAPE = kit.Look((0.75, 0.12, 0.10), 0.6)  # red rim strip in the channel: visible where the bead has gone down
STEEL = kit.Look((0.55, 0.56, 0.58), 0.3, metallic=1.0)
BRASS = kit.Look((0.78, 0.62, 0.28), 0.3, metallic=0.8)
BLACK = kit.Look((0.05, 0.05, 0.05), 0.5)
LEVER_LOOK = kit.Look((0.10, 0.35, 0.80), 0.45)  # blue nylon
STAND = kit.Look((0.22, 0.23, 0.25), 0.5)


def _sweep_mesh(profile, look, a0=0.0, a1=2 * math.pi, n=None, shift=(0.0, 0.0, 0.0)):
    """A closed (r, z) profile swept about the wheel's axis from angle a0 to a1 (rad), with end caps on an arc. `shift`
    is subtracted from every point (a link's anchor, so link meshes are in the link's frame)."""
    full = abs(a1 - a0 - 2 * math.pi) < 1e-9
    n = n or max(4, int(abs(a1 - a0) / math.radians(6)))
    rings = n if full else n + 1
    m = len(profile)
    pts, counts, idx = [], [], []
    for k in range(rings):
        t = a0 + (a1 - a0) * k / n
        for r, z in profile:
            pts.append((r * math.cos(t) - shift[0], r * math.sin(t) - shift[1], z - shift[2]))
    for k in range(n):
        k2 = (k + 1) % rings
        for i in range(m):
            i2 = (i + 1) % m
            counts.append(4)
            idx += [k * m + i, k * m + i2, k2 * m + i2, k2 * m + i]
    if not full:
        counts += [m, m]
        idx += list(range(m))[::-1] + [(rings - 1) * m + i for i in range(m)]
    return kit._mesh(pts, counts, idx, look)


def _shell(outer, t=0.003):
    """A closed profile of a tire wall `t` thick along an (r, z) polyline (inner face toward the section's middle)."""
    c = np.array([R_SEAT + (0.126 - R_SEAT) * TIRE_SCALE, 0.0])
    inner = [tuple(c + (np.array(p) - c) * (1 - t / max(np.linalg.norm(np.array(p) - c), 1e-6))) for p in outer]
    return list(outer) + inner[::-1]


# The tire's outer face from the lower bead round the tread to the upper bead (r, z): a 57 mm-wide section, drawn to a
# 30.3 cm tread and scaled out from the bead seat to the K124's nominal 12.5 in (31.75 cm) across the tread.
R_TREAD = 0.15875  # m: half of 12.5 in, the tire's nominal size
TIRE_SCALE = (R_TREAD - R_SEAT) / (0.1515 - R_SEAT)


def _scaled(profile):
    return [(R_SEAT + (r - R_SEAT) * TIRE_SCALE, z) for r, z in profile]


LOWER = [(0.1015, -0.0085), (0.106, -0.0125), (0.113, -0.022), (0.122, -0.0275), (0.133, -0.0285), (0.143, -0.024),
         (0.149, -0.014), (0.1515, 0.0)]
CROWN = [(0.1515, 0.0), (0.149, 0.014), (0.143, 0.024), (0.133, 0.0285)]
INNER = [(0.133, 0.0285), (0.122, 0.0275), (0.113, 0.022), (0.106, 0.0125), (0.1015, Z_BEAD)]
LOWER, CROWN, INNER = _scaled(LOWER), _scaled(CROWN), _scaled(INNER)
PRESS_R = 0.116  # where a hand pushes a sector down: on the sidewall 1 cm outside the flange
PRESS_Z = 0.025  # its top there


def _at(deg, r=PRESS_R):
    a = math.radians(deg)
    return (r * math.cos(a), r * math.sin(a))


ARC_CORE = 20.0  # deg each side of an arc's middle drawn on its own (rigid) link; the 10 deg either side of each joint
#                   between arcs is drawn as slices blending the two arcs' heights (_arc_blends)


def _arc_meshes(lo_deg, hi_deg):
    a0, a1 = math.radians(lo_deg), math.radians(hi_deg)
    return (_sweep_mesh(_shell(INNER), GUMWALL, a0, a1),
            _sweep_mesh(_shell(INNER[-2:] + [(0.1005, 0.0045)], 0.002), RUBBER, a0, a1))


def _arc_blends(slices=4):
    """The bead either side of each joint between two arcs (and between an arc and the last section), drawn as thin
    slices whose drop blends linearly from one arc's to the next (to the seated height at the last section, where its
    own taper ends), so the bead bends down into the channel instead of stepping. Drawn only: no colliders, no effect
    on the physics or the score; with every arc on its shoulder or every arc in the channel they line up flush."""
    links = []
    half = 30.0 - ARC_CORE
    by_edge = {}
    for n, c in SECTORS.items():
        by_edge[(c - 30.0) % 360.0] = by_edge.get((c - 30.0) % 360.0, {}) | {"right": n}
        by_edge[(c + 30.0) % 360.0] = by_edge.get((c + 30.0) % 360.0, {}) | {"left": n}
    for edge, pair in sorted(by_edge.items()):
        left, right = pair.get("left"), pair.get("right")  # the arc below the joint's angle, the arc above it
        # beside the last section only the arc's side is blended (down to seated at the joint); the section's own
        # slices (_blend_slices) draw the other side
        z0 = edge - half if left else edge
        span = (2 * half if left and right else half)
        step = span / slices
        for k in range(slices):
            lo = z0 + k * step
            mid = lo + step / 2
            if left and right:
                wr = (mid - z0) / span
                wl = 1.0 - wr
            elif left:
                wl, wr = (edge - mid) / half, 0.0
            else:
                wl, wr = 0.0, (mid - edge) / half

            def follow(qs, left=left, right=right, wl=wl, wr=wr):
                return (wl * qs[left] if left else 0.0) + (wr * qs[right] if right else 0.0)

            links.append(kit.Link(f"arcblend{int(edge):03d}_{k}", "prismatic", "Z", (0.0, 0.0, 0.0), lower=-DROP,
                                  upper=0.0, start=0.0, mass=0.002, meshes=_arc_meshes(lo, lo + step),
                                  floating=True, follow=follow, follow_hold=1.0, follow_rate=0.003))
    return tuple(links)


def _sector(name, center):
    meshes = _arc_meshes(center - ARC_CORE, center + ARC_CORE)
    # Press spots on the sidewall. The arcs next to the last section have theirs on their far half only (10 and 20
    # degrees from the far end), so no single hand, 9.4 cm across fully open, spans one of them and a press spot of the
    # last section (60 degrees apart, 9.7 cm edge to edge).
    offsets = (-20.0, 0.0, 20.0)
    if name in NEXT_TO_LAST:
        offsets = (10.0, 20.0) if center > LAST_AT else (-20.0, -10.0)
    colliders = tuple(kit.Collider((*_at(center + d), PRESS_Z - 0.003), radius=0.010, height=0.006) for d in offsets)
    law = Creep() if name in NEXT_TO_LAST else mk.Detents(positions=(0.0, -DROP), labels=("SHOULDER", "CHANNEL"),
                                                          zone=SEAT_ZONE, pull=200.0, damp=3.0, drag=2.0)
    return kit.Link(name, "prismatic", "Z", (0.0, 0.0, 0.0), lower=-DROP, upper=0.0, start=0.0, mass=0.05,
                    meshes=meshes, colliders=colliders, floating=True, law=law)


@dataclass(frozen=True)
class Creep:
    """An arc next to the last section: pulled back up onto its shoulder unless held down (a force spring: a
    stiffness-only drive does not move a prismatic link)."""

    k: float = CREEP
    damp: float = 3.0

    def drive(self, key, q, qd, J, mem):
        return q, 0.0, -self.k * q - self.damp * qd, self.damp


def slack(J):
    """Bead freed for the last section (m): from each end of it round the wheel, every arc at least 6 mm down into the
    channel in an unbroken run gives its drop (as a share of the 9 mm travel) times 4.2 mm."""
    run = set()
    for order in CHAIN:
        for n in order:
            if -J[n] < DROP - SEAT_ZONE:
                break
            run.add(n)
    return sum(min(-J[n] / DROP, 1.0) * SLACK_PER_SECTOR for n in run)


def press_needed(q):
    """The straight-down press (N) at the middle press spot that holds the last section at flap angle q against the
    bead: BEND_N while the sidewall rolls down to the flange's edge, then rising linearly to SEAT_N as the lip stretches
    over the flange, up to the 2.5 degrees where it snaps in."""
    if q <= BLOCK_AT:
        return BEND_N
    u = min((q - BLOCK_AT) / (-FLAP_SEAT_ZONE - BLOCK_AT), 1.0)
    return BEND_N + (SEAT_N - BEND_N) * u


@dataclass(frozen=True)
class LastSection:
    """The last section's law. Without 10 mm of slack its lip stops on the flange's edge (6 degrees short of seated: the
    joint's upper limit is moved there, a hard stop, whenever it is not yet within 2.5 degrees of seated, so a hard
    press that overshoots the stop for a step is pushed back); with it, it rolls over. Short of the seat zone the bead
    pushes back (press_needed: 8 N to roll the sidewall down, rising to 30 N over the flange), so a section let go
    there springs back up. Within 2.5 degrees of seated it snaps in and stays."""

    pull: float = 0.5
    damp: float = 0.02
    drag: float = 0.05
    sets_limits: bool = True

    def drive(self, key, q, qd, J, mem):
        blocked = slack(J) < SLACK_NEED and q < -FLAP_SEAT_ZONE
        mem["blocked"] = blocked
        mem["limits"] = (FLAP_LO, BLOCK_AT) if blocked else None
        if q >= -FLAP_SEAT_ZONE:
            return 0.0, self.pull, 0.0, self.damp
        return q, 0.0, -press_needed(q) * PRESS_ARM, self.drag


def _hinge_vec(r, z, alpha):
    """A point of the last section (seated at radius r, height z on the 90-degree line) turned by alpha about the
    hinge: (radius, height)."""
    y, zz = r - HINGE_R, z - HINGE_Z
    return HINGE_R + y * math.cos(alpha) - zz * math.sin(alpha), HINGE_Z + y * math.sin(alpha) + zz * math.cos(alpha)


def lip_height(alpha):
    """The last section's bead lip above the rim's middle plane at flap angle alpha."""
    return _hinge_vec(R_SEAT, Z_BEAD, alpha)[1]


ANCHOR = (0.0, HINGE_R, HINGE_Z)  # the hinge line's point on the 90-degree radius (base frame)
FLAP_PRESS = [(d, (0.118, 0.026)) for d in (-10.0, 0.0, 10.0)]  # press spots: angle offset, (r, z) seated


FLAP_CORE = 15.0  # deg each side of the last section's middle drawn on its (rigid) link: its press spots (10 deg out, 1.1 cm round) lie inside
BLEND_SLICES = 6  # drawn-only slices on each side, from FLAP_CORE out to 30 deg, following the flap at a falling share


def _flap_meshes(lo_deg, hi_deg):
    a0, a1 = math.radians(lo_deg), math.radians(hi_deg)
    return (_sweep_mesh(_shell(CROWN), RUBBER, a0, a1, shift=ANCHOR),
            _sweep_mesh(_shell(INNER), GUMWALL, a0, a1, shift=ANCHOR),
            _sweep_mesh(_shell(INNER[-2:] + [(0.1005, 0.0045)], 0.002), RUBBER, a0, a1, shift=ANCHOR))


def _blend_slices():
    """The last section's rubber either side of its rigid middle, drawn as thin slices that follow the flap's angle at
    a share falling linearly to the arcs beside it (11/12 down to 1/12), so the lifted bead tapers down into the
    seated tire instead of standing up as one block. Drawn only: no colliders, no mass to speak of, no effect on the
    physics or the score; seated, every slice lines up with the rest of the tire."""
    links = []
    width = (30.0 - FLAP_CORE) / BLEND_SLICES
    for side in (-1, 1):
        for k in range(BLEND_SLICES):
            lo, hi = FLAP_CORE + k * width, FLAP_CORE + (k + 1) * width
            w = 1.0 - (k + 0.5) / BLEND_SLICES
            a, b = (LAST_AT + lo, LAST_AT + hi) if side > 0 else (LAST_AT - hi, LAST_AT - lo)
            links.append(kit.Link(f"flapblend{'p' if side > 0 else 'm'}{k}", "revolute", "X", ANCHOR, lower=FLAP_LO,
                                  upper=0.0, start=FLAP_START * w, mass=0.002, meshes=_flap_meshes(a, b),
                                  floating=True, follow=lambda qs, w=w: w * qs["flap"], follow_hold=1.0,
                                  follow_rate=0.25))
    return tuple(links)


def _flap():
    meshes = _flap_meshes(LAST_AT - FLAP_CORE, LAST_AT + FLAP_CORE)
    colliders = []
    for d, (r, z) in FLAP_PRESS:
        x, y = _at(LAST_AT + d, r)
        colliders.append(kit.Collider((x - ANCHOR[0], y - ANCHOR[1], z - 0.004 - ANCHOR[2]), radius=0.011,
                                      height=0.008))
    return kit.Link("flap", "revolute", "X", ANCHOR, lower=FLAP_LO, upper=0.0, start=FLAP_START, mass=0.04,
                    meshes=meshes, colliders=tuple(colliders), floating=True, law=LastSection())


def _valve():
    """The valve stem from the rim bed toward the hub (link frame: the hole in the rim bed, stem along -y)."""
    meshes = (kit.cylinder((0, -0.012, 0), 0.004, 0.024, STEEL, axis="y"),  # threaded stem
              kit.cylinder((0, -0.003, 0), 0.0055, 0.003, BRASS, axis="y"),  # rim nut
              kit.cylinder((0, -0.026, 0), 0.0045, 0.012, BLACK, axis="y"))  # dust cap
    colliders = (kit.Collider((0, -0.018, 0), size=(0.009, 0.026, 0.009)),)
    return kit.Link("valve", "revolute", "Z", (0.0, R_BED, 0.0), lower=math.radians(-35), upper=math.radians(35),
                    start=VALVE_START, mass=0.02, meshes=meshes, colliders=colliders, floating=True,
                    law=mk.Free(drag=0.002))


def _wheel():
    meshes, colliders = [], []
    # Rim: a steel single-wall rim (bed, sidewalls, flanges; drawn closed) with the red rim strip in its channel.
    rim = [(R_BED, -0.008), (0.096, -0.0105), (0.1075, -0.0105), (0.1075, -0.0095), (R_FLANGE, -0.008),
           (R_SEAT, -0.0075), (R_SEAT, -0.0045), (R_WELL, -0.003), (R_WELL, 0.003), (R_SEAT, 0.0045),
           (R_SEAT, 0.0075), (R_FLANGE, 0.008), (0.1075, 0.0095), (0.1075, 0.0105), (0.096, 0.0105), (R_BED, 0.008)]
    meshes.append(_sweep_mesh(rim, RIM, n=90))
    meshes.append(_sweep_mesh([(R_WELL + 0.0002, -0.0028), (R_WELL + 0.0002, 0.0028), (R_WELL + 0.0005, 0.0028),
                               (R_WELL + 0.0005, -0.0028)], RIM_TAPE, n=90))
    # Tire: lower half and tread all round; the crown of the upper half except over the last section.
    meshes.append(_sweep_mesh(_shell(LOWER), RUBBER, n=90))
    meshes.append(_sweep_mesh(_shell(LOWER[:5], 0.002), GUMWALL, n=90))  # lower sidewall's tan band
    meshes.append(_sweep_mesh(_shell(CROWN), RUBBER, math.radians(LAST_AT + 30), math.radians(LAST_AT + 330)))
    # Hub, 20 spokes (the named WheelMaster wheel is 20-hole) and the stand (a post into the hub from a weighted foot).
    meshes.append(kit.cylinder((0, 0, 0), 0.016, 0.06, STEEL))
    meshes.append(kit.cylinder((0, 0, 0), 0.024, 0.004, STEEL))
    for k in range(20):
        a = 2 * math.pi * k / 20
        side = 0.012 if k % 2 else -0.012
        p0 = np.array([0.022 * math.cos(a), 0.022 * math.sin(a), side])
        p1 = np.array([R_BED * math.cos(a + 0.2), R_BED * math.sin(a + 0.2), 0.0])
        meshes.append(kit.cylinder(tuple((p0 + p1) / 2), 0.0009, float(np.linalg.norm(p1 - p0)), STEEL, axis=p1 - p0,
                                   n=6))
    meshes.append(kit.cylinder((0, 0, -ZR / 2 - 0.015), 0.009, ZR - 0.03, STAND))
    meshes.append(kit.box((0, 0, -ZR + 0.004), (0.12, 0.12, 0.008), STAND))
    colliders.append(kit.Collider((0, 0, -ZR + 0.004), size=(0.12, 0.12, 0.008)))
    colliders.append(kit.Collider((0, 0, 0), radius=0.016, height=0.06))
    links = tuple(_sector(n, c) for n, c in SECTORS.items()) + (_flap(), _valve()) + _blend_slices() + _arc_blends()
    return kit.Mechanism("wheel", pos=(WX, WY, TOP + ZR), size=(0.31, 0.31, 2 * ZR), meshes=tuple(meshes),
                         colliders=tuple(colliders), links=links)


def _lever():
    L, W, H = LEVER
    meshes = (kit.box((0, 0, 0), (L - 0.012, W, H), LEVER_LOOK),
              kit.box((L / 2 - 0.006, 0, -0.001), (0.012, W - 0.004, H - 0.002), LEVER_LOOK),  # thinner spoon tip
              kit.box((-L / 2 + 0.006, 0, 0.002), (0.012, W - 0.006, H + 0.004), LEVER_LOOK))  # spoke hook
    return kit.Prop("lever", pos=(LEVER_POS[0], LEVER_POS[1], TOP + H / 2), size=LEVER, mass=0.02, meshes=meshes,
                    colliders=(kit.Collider((0, 0, 0), size=LEVER),))


OBJECTS = [_wheel(), _lever()]
GRIPPERS = ("left_gripper", "right_gripper")


# --- Readouts ---------------------------------------------------------------------------------------------------------
def _wheel_frame(objects, p):
    """World point p as (radius from the hub axis, angle in degrees, height above the rim's middle plane)."""
    q = gk.local(objects, "wheel", p)
    return float(math.hypot(q[0], q[1])), math.degrees(math.atan2(q[1], q[0])) % 360.0, float(q[2])


def flap_angle(objects):
    return kit.joint(objects, "wheel/flap").pos


def bead_on(objects):
    """The last section's bead lip inside the flange: the flap within 2.6 degrees of seated, where its lip is below the
    flange top. The other sectors can only sit on their shoulder or in the channel, both inside the flange."""
    return lip_height(flap_angle(objects)) < Z_FLANGE


def valve_angle(objects):
    return kit.joint(objects, "wheel/valve").pos


def bead_height(objects, ang):
    """The upper bead's height above the rim's middle plane at angle ang (degrees): the last section's lip, or an
    arc's bead on its shoulder (8.5 mm) or as far down into the channel as it has been pushed."""
    if abs((ang - LAST_AT + 180.0) % 360.0 - 180.0) <= 30.0:
        return lip_height(flap_angle(objects))
    name = min(SECTORS, key=lambda n: abs((ang - SECTORS[n] + 180.0) % 360.0 - 180.0))
    return Z_BEAD + kit.joint(objects, f"wheel/{name}").pos


def under_bead(objects, r, ang, h):
    """A point in the tube's space: inside the bead seat line by more than 1 mm (radius 8.8 to 10.05 cm: the rim's
    well and channel, where the tube lies), from the rim's lower half up to the bead's own height there. A lever tip
    hooking the bead on the flange or the shoulder (radius 10.05 cm or more), or lying on top of the bead, does not
    count."""
    return R_BED - 0.002 <= r <= R_SEAT - TOE and -Z_FLANGE <= h <= bead_height(objects, ang)


def lever_tip(objects):
    s = objects["lever"]
    return np.asarray(s.pos, float) + gk._rot(s.quat_xyzw) @ np.asarray(LEVER_TIP, float)


def lever_pinch(objects):
    """The lever's spoon tip under the bead at this step or anywhere on its path since the last step."""
    def test(p):
        return under_bead(objects, *_wheel_frame(objects, p))
    return SWEEP.crossed(objects, "tip", lever_tip(objects), test)


# A hand on the tire is contact (gk.touching): the gripper's own shape (palm and both fingers, boxes fitted to the
# YAM's collision meshes, at their present opening) within 2 mm (the solver's resting gap plus the box fit) of the
# tire, the rim or the valve stem, in the wheel's frame. The tire is its outer profile cut into 0.5 mm rings (each as
# tall as the profile's highest point over it, so a ring stands at most 0.5 mm proud of the real wall); the rim is its
# bed to its flange, 21 mm tall; the valve is a 9 mm stem from the rim bed toward the hub, turned with the valve.
TIRE_PROFILE = tuple(reversed(INNER)) + tuple(reversed(CROWN))[1:]  # (r, z) from the bead out to the tread


def _profile_z(r):
    return float(np.interp(r, [p[0] for p in TIRE_PROFILE], [p[1] for p in TIRE_PROFILE]))


def _tire_rings(step=0.0005):
    rings, r0 = [], TIRE_PROFILE[0][0]
    while r0 < TIRE_PROFILE[-1][0] - 1e-9:
        r1 = min(r0 + step, TIRE_PROFILE[-1][0])
        top = max([_profile_z(r0), _profile_z(r1)] + [z for r, z in TIRE_PROFILE if r0 <= r <= r1])
        rings.append(gk.Ring(r0, r1, -top, top))
        r0 = r1
    return tuple(rings)


TIRE_SHAPE = _tire_rings()
TIRE_OUTLINE = gk.Ring(R_SEAT, R_TREAD, -0.0285, 0.0285)  # one ring round all of them: a quick first test
RIM_SHAPE = gk.Ring(R_BED, 0.1075, -Z_FLANGE, Z_FLANGE)
VALVE_R = 0.0045  # the stem's collider is 9 mm square


def valve_shape(objects):
    """The valve stem as a capsule in the wheel's frame, for the hands-clear check: its core line runs 2.6 cm toward the
    hub from 5 mm inside the rim bed (at 90 degrees), and its 4.5 mm radius covers the whole 3.2 cm stem (its rounded
    end reaches about 3.5 cm); turned about the vertical by the valve's angle (positive: its tip toward +x)."""
    a = valve_angle(objects)
    return gk.Seg((0.0, R_BED - 0.005, 0.0), (0.026 * math.sin(a), R_BED - 0.005 - 0.026 * math.cos(a), 0.0), VALVE_R)


def on_tire(objects, g):
    """Gripper g touching the tire, the rim or the valve: its own shape within 2 mm of them (gk.touching). A hand
    hovering over the tire, however close past 2 mm, is not on it."""
    if gk.touching(objects, "wheel", (RIM_SHAPE, valve_shape(objects)), grippers=(g,)):
        return True
    return bool(gk.touching(objects, "wheel", TIRE_OUTLINE, grippers=(g,))
                and gk.touching(objects, "wheel", TIRE_SHAPE, grippers=(g,)))


def hands_clear(objects):
    """No gripper touching the tire, rim or valve (not on_tire)."""
    return not any(on_tire(objects, g) for g in GRIPPERS if g in objects)


class _Seating(gk._PerStep):
    """How fast the last section's bead lip moves toward its seat: its angle change since the last control step times
    the lip's 4.4 cm from the hinge line, over the step (so a slam between two samples is still seen). Only the travel
    up to 2.5 degrees short of seated counts, since from there the bead snaps in by itself. `on` latches once that
    speed passes STRIKE_SPEED."""

    def _fresh(self):
        self.prev, self.peak, self.on = None, 0.0, False

    def feed(self, objects) -> bool:
        if self._tick(objects):
            a = flap_angle(objects)
            if self.prev is not None and self.prev < -FLAP_SEAT_ZONE:
                v = LIP_ARM * (min(a, -FLAP_SEAT_ZONE) - self.prev) / DT
                self.peak = max(self.peak, v)
                self.on = self.on or v > STRIKE_SPEED
            self.prev = a
        return self.on


SEATING = _Seating()


def _watch(objects):
    PINCH.feed(objects, lever_pinch(objects))
    SEATING.feed(objects)


def arcs_down(objects):
    """How many of the five bead arcs sit at least 6 mm down in the channel now."""
    return sum(-kit.joint(objects, f"wheel/{n}").pos >= DROP - SEAT_ZONE for n in SECTORS)


def _job(objects):
    if PINCH.on:  # a lever's tip reached the tube's space at any step or between two: the stand-in for a pinched tube
        return 0.0
    if bead_on(objects):
        if abs(valve_angle(objects)) > VALVE_TOL or not hands_clear(objects):
            done = 0.5  # on, but the valve left leaning, or a hand still on the tire
        else:
            done = 1.0
    elif arcs_down(objects) >= PROGRESS_ARCS:
        done = 0.25  # not on yet, but the bead pushed into the center channel: the slack started
    else:
        done = 0.0
    if SEATING.on:  # the last section struck down faster than 12 cm/s: a stand-in for a tube pinched under the bead,
        done = min(done, STRIKE_CAP)  # so a cap, not a zero; the real trial's dunk test checks the tube itself
    return done


def success(objects):
    _watch(objects)
    return _job(objects) == 1.0


def score(objects):
    """Scored on what a mechanic can see and test. 0 (irreversible) if the tire lever's spoon tip reached the tube's
    space at any control step or on its path between two: more than 1 mm inside the bead seat line, down in the rim
    (radius 8.8 to 10.05 cm, at or below the bead's height there); a lever that only hooks the bead on the flange or
    shoulder is careful use and allowed. Then, at the end: 1.0 with the last section's bead on (its lip below the
    flange top, within 2.6 degrees of seated), the valve stem within 10 degrees of the line to the hub (its tip
    within 5.6 mm), and no gripper touching the tire, rim or valve (its own shape, palm and fingers at their present
    opening, within 2 mm of them); 0.5 with the bead on but the valve leaning or a hand still touching the tire;
    0.25 with the bead not on but at least two bead arcs at least 6 mm down in the center channel (the slack
    started); 0 otherwise. A struck bead caps the score at 0.5: the last section's bead lip driven toward its seat
    faster than 12 cm/s over any control step (its whole roll in about 0.13 s), up to 2.5 degrees short of seated,
    where it snaps in by itself. That is a stand-in for a tube pinched under the bead, so it caps rather than
    zeroes; the real trial checks the tube with the dunk test, and a leak there scores 0. How the bead got on (one
    hand or two, a lever, the order) is not scored, and where the arms end is not scored. Possible: 0, 0.25, 0.5, 1."""
    _watch(objects)
    return _job(objects)


# --- Demo -------------------------------------------------------------------------------------------------------------
TIP_UP = PAD_TIP_PAST_TCP  # the tool point sits this far above the pad tips when the gripper points down
HOLD_N = 16.0  # N: the right hand's hold on the arc beside the last section (it pushes back 11.7 N at the floor)
ROLL_MARGIN_N = 5.0  # N: pressed this much past what the bead takes, so the section keeps rolling


def _w(x, y, h):
    """Wheel-frame x, y and height above the rim's middle plane to a world point."""
    return (WX + x, WY + y, TOP + ZR + h)


_GIVE = {}


def _give(side, tcp):
    """The arm's give at its pad tips (m/N, 3x3, world) with the tool point at `tcp`, gripping straight down (yaw 0):
    J K^-1 J^T with the YAM's servo stiffness (80 N*m/rad shoulder joints, 10 wrist). Here about 2 mm per newton
    straight down, 5.5-7 mm per newton sideways, and a downward press drags the pads 0.4-0.7 mm/N sideways."""
    key = (side, *(round(float(v), 3) for v in tcp))
    if key not in _GIVE:
        import yam_arena.demo as yd
        from yam_arena.constants import SHOULDER, WRIST
        from yam_arena.station import BASES
        base = np.asarray(BASES[side], float)
        q, _, _ = yd.ik.solve(np.asarray(tcp, float), yd.ik.matrix_to_quat_xyzw(yd.grip_rotation(0.0)), q_init=None,
                              base_pos=base)

        def tip(q):
            p, rm = yd.ik.tcp_pose(q, base)
            return np.asarray(p) + TIP_UP * np.asarray(rm)[:, 2]

        jac = np.zeros((3, 6))
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = 1e-6
            jac[:, i] = (tip(q + dq) - tip(q - dq)) / 2e-6
        kinv = np.diag([1 / SHOULDER["stiffness"]] * 3 + [1 / WRIST["stiffness"]] * 3)
        _GIVE[key] = jac @ kinv @ jac.T
    return _GIVE[key]


def _pressed(side, contact, press):
    """The tool point's target for closed fingers resting on `contact` (wheel frame x, y, h of the pad tips) to press
    straight down with `press` N: aimed past the pads by the arm's give (so the pads stay over the spot)."""
    tcp = np.array(_w(contact[0], contact[1], contact[2] + TIP_UP))
    return tuple(float(v) for v in tcp + _give(side, tcp) @ np.array([0.0, 0.0, -press]))


def demo():
    """Right arm: pushes the bead down into the channel at 228, 270 and 312 degrees (the side away from the valve),
    pressing each arc's sidewall 1 cm outside the flange with closed fingers, then pushes the arc beside the last
    section down at 170 degrees and holds it there. Left arm: pushes the valve stem straight toward the hub from its
    leaning side, then, while the right hand holds, rolls the last section over by pressing its sidewall down along
    its arc. Both let go and go home."""
    shut = FINGER_CLOSED
    right, left = [], []
    for deg in (228.0, 270.0, 312.0, 170.0):
        x, y = _at(deg)
        if deg == 170.0:  # the held arc pushes back up (11.7 N at the floor): press with 16 N at the floor
            down = _pressed("right", (x, y, PRESS_Z - DROP), HOLD_N)
        else:
            down = _w(x, y, PRESS_Z - DROP - 0.004 + TIP_UP)
        if deg == 170.0:  # round the near side of the wheel, clear of the hub and the left arm
            right += [Waypoint(1.6, _w(*_at(215.0, 0.13), PRESS_Z + 0.06 + TIP_UP), shut, "round the near side")]
        right += [Waypoint(1.6, _w(x, y, PRESS_Z + 0.04 + TIP_UP), shut, "over the arc"),
                  Waypoint(0.8, _w(x, y, PRESS_Z + 0.002 + TIP_UP), shut, "onto the sidewall"),
                  Waypoint(1.0 if deg != 170.0 else 1.6, down, shut, "push the bead into the channel"),
                  Waypoint(0.4, down, shut, "hold"),
                  Waypoint(0.6, _w(x, y, PRESS_Z + 0.04 + TIP_UP), shut, "lift")]
    hold_at = right[-2]
    right = right[:-2]  # the arc beside the last section: keep holding it down while the left hand rolls
    t_held = sum(w.duration for w in right)

    # Left: the valve first (its stem leans its tip toward +x; push it back from that side).
    vy = R_BED - 0.022  # 2.2 cm along the stem from the rim bed
    vx = 0.022 * math.sin(VALVE_START)
    side = 0.012  # closed fingers' half-thickness along x, roughly
    left += [Waypoint(2.5, _w(vx + side + 0.012, vy, 0.06), shut, "over the valve"),
             Waypoint(1.2, _w(vx + side + 0.012, vy, -0.001 + TIP_UP), shut, "down beside the stem"),
             Waypoint(1.5, _w(side + 0.001, vy, -0.001 + TIP_UP), shut, "push it straight"),
             Waypoint(0.8, _w(side + 0.012, vy, -0.001 + TIP_UP), shut, "back off"),
             Waypoint(1.0, _w(side + 0.012, vy, 0.06), shut, "up")]
    # Once the right hand holds the arc beside it down, roll the last section over: press its sidewall at 90 degrees.
    t_left = sum(w.duration for w in left)
    wait = max(0.5, t_held + 0.4 - t_left)
    r0, z0 = FLAP_PRESS[1][1]
    start = _hinge_vec(r0, z0, FLAP_START)
    left += [Waypoint(wait, _w(0.0, start[0], start[1] + 0.04 + TIP_UP), shut, "over the last section")]
    # Roll it down, pressing with what the bead takes there plus a margin (press_needed + ROLL_MARGIN_N), aimed past
    # the sidewall by the arm's give. From the flange's edge (-6 deg) the steps are finer and slower, as the press
    # climbs toward 30 N, so the lip never runs at the bead faster than a careful roll.
    angles = [FLAP_START + (BLOCK_AT - FLAP_START) * k / 5 for k in range(6)]
    angles += [BLOCK_AT + (-FLAP_SEAT_ZONE - BLOCK_AT) * k / 6 for k in range(1, 7)] + [0.0]
    for k, a in enumerate(angles):
        r, h = _hinge_vec(r0, z0, a)
        press = press_needed(min(a, -FLAP_SEAT_ZONE)) + ROLL_MARGIN_N
        dur = 1.2 if k == 0 else (0.4 if a <= BLOCK_AT else 0.6)
        left.append(Waypoint(dur, _pressed("left", (0.0, r, h), press), shut, "roll the bead over"))
    r, h = _hinge_vec(r0, z0, 0.0)
    left += [Waypoint(0.6, _pressed("left", (0.0, r, h), SEAT_N), shut, "seat it"),
             Waypoint(0.6, _pressed("left", (0.0, r, h), 4.0), shut, "ease off")]
    t_seated = sum(w.duration for w in left)
    left += [Waypoint(0.8, _w(0.0, r, h + 0.05 + TIP_UP), shut, "let go, up"),
             kit.Home(2.0), kit.Home(1.0, label="hold at home")]
    right += [Waypoint(t_seated + 0.4 - t_held, hold_at.pos, shut, "hold the arc down while it rolls"),
              Waypoint(0.6, (hold_at.pos[0], hold_at.pos[1], hold_at.pos[2] + 0.05), shut, "let go, up"),
              kit.Home(2.0), kit.Home(1.0, label="hold at home")]
    t_left, t_right = sum(w.duration for w in left), sum(w.duration for w in right)
    if t_right < t_left:
        right.insert(-1, kit.Home(t_left - t_right, label="wait at home"))
    elif t_left < t_right:
        left.insert(-1, kit.Home(t_right - t_left, label="wait at home"))
    return {"left": left, "right": right}
