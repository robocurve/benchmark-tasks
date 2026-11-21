"""The cream, the applicator, the plaster and its two liners: a reduced model, advanced once per control step from the
grippers' measured poses and jaw openings, the rigid-body poses of the applicator and the two liner tabs, and the native
PhysX contact force of each fingertip on the forearm and on those bodies. Plain NumPy.

Nothing here reads a demo, a phase or a goal. The scorer (rules.py) reads its state.

The parts, and what is reduced (each number is nominal, to be checked against the real parts; see the README):

* **Cream** (a brown antiseptic ointment) is a height field on the forearm's skin (0.5 mm cells over 80 x 70 mm round
  the wound), in the manner of the spackle task's paste model. The applicator's cotton bud (a 5 mm sphere at its end)
  works it where it touches the skin (within 0.3 mm, or pressed into the cotton up to 3 mm): a dab leaves a dot up to
  0.4 mm thick from what the bud carries; a wipe (the bud sliding at least 1.5 mm in 0.15 s along the skin) leaves a
  film 0.12 mm thick under its contact patch, filled from the bud's load, and takes up whatever stands thicker. The bud
  loads only by dipping into the dollop in the medicine cup (up to 0.08 mL). A dip after the bud has touched skin is a
  double dip.
* **The applicator** is a loose rigid body in the simulator (props.py). Here it is only its pose and who holds it.
* **Grips** are native contact: a hand holds a part while both of its fingertip pads press the part's body with at
  least GRIP_N, and has let go once either pad stays under LET_GO_N for LET_GO_STEPS control steps.
* **The plaster** (1.75 x 4 in fabric strip, 38 mm pad across its whole width, adhesive on the two wings) is a grid of
  35 x 9 nodes. While a liner is on it is flat and stiff (liner paper plus fabric). Lying on the forearm it lies on the
  skin (the forearm is a straight elliptic prism here, so the strip lies on it without stretch). It is carried by its
  liners' tabs, which are loose rigid bodies (hinged at their roots until a hand holds them); the strip hangs between the
  places the fingers hold them. Let fall, it settles on the arm only from within DROP_LAND of the skin with no adhesive
  bared; otherwise it has fallen.
* **The liners** are C-folded: each covers one wing and half the pad, is folded back on itself at the strip's middle,
  and its lower leaf runs out past the strip's end as the 25 mm tab. Pulling a tab outward along the strip, at most
  60 degrees above the skin, peels the liner from the middle outward at half the tab's travel, if the strip is held
  against the peel force; pulling steeper lifts the wing's end instead. Two tabs pulled apart in the air peel both. A
  liner that is off is its tab body: carried, dropped and binned as that body is.
* **Peel force:** the liner's release force (PEEL_N, a start-up peak PEEL_BREAK_N, a light PEEL_PAD_N under the pad)
  pulls the held tab back, as a taut string the simulator applies to the tab body at every physics step (`tether`), and
  pulls the strip toward the tab at the peel front (`front_force`). The strip resists with friction on the skin under
  its weight and under pressing fingertip pads, and with the shear of its tacked nodes; past that, the other tab (held
  taut by its hand) takes the rest of the pull, or else the strip slides with the pull.
* **Tack:** an exposed adhesive node lying on the skin tacks only under a fingertip pad whose own native PhysX contact
  force on the forearm is at least 1 N, within 2 mm of that pad. Tack never spreads. A node that lies down on cream
  (more than 0.02 mm within 2 mm) is spoiled: cream on the adhesive.
"""

from __future__ import annotations

import math

import numpy as np

import scene as S

# -- model numbers -----------------------------------------------------------------------------------------------------
CELL = 0.0005
FIELD_Y = (-0.040, 0.040)
FIELD_S = (-0.035, 0.035)
FILM = 0.00012  # m left by a wipe
DAB = 0.0004  # m: a dab's dot
LOAD_CAP = 0.08e-6  # m^3 (0.08 mL) a bud takes up
LOAD_RATE = 0.04e-6  # m^3 per control step in the dollop
CONTACT_GAP = 0.0003  # bud within this of the skin touches it
BUD_SQUASH = 0.003  # the cotton squashes up to this; deeper is counted as squashed to this
SLIDE_MIN, SLIDE_STEPS = 0.0015, 3
COVER_H = 0.00005  # a cell counts as creamed at 0.05 mm
STRAY_H = 0.00002  # cream thicker than 0.02 mm counts as cream (outside the band; under the adhesive)
PRESS_N = 1.0  # N
STEEP_DEG = 60.0
DROP_LAND = 0.010  # m: a dropped strip with its liners on lands flat only from this close to the skin
EXT_MIN = 0.001  # m of tab pulled taut before it does anything
GRIP_N = 1.0  # N: a part is held while both fingertip pads of one hand press on it with at least this (native contact)
TAKE_STEPS = 3  # ... for this many control steps in a row (a jaw brushing a part in passing does not take it),
JAW_SLACK = 0.003  # ... with the jaw closed to within this of the part's own thickness
PART_W = {"applicator": 2 * S.APP_STICK_R, "tab_left": S.TAB_THICK, "tab_right": S.TAB_THICK}
LET_GO_N, LET_GO_STEPS = 0.2, 3  # ... and let go once either pad stays under this for this many control steps
APP_REST_Z = S.TRAY_SIZE[2] + 0.0003 + S.APP_TIP_R  # the stick's axis lying on the opened pouch

# release-liner peel (README, "Peel force"). Siliconised medical liners release at about 5-20 g per 2 in (easy) to
# 10-50 g per 2 in (easy-medium) in a 180 degree peel; the liner here is taken at 0.15 N per 25 mm, so 0.27 N over the
# strip's 44.5 mm width, with a start-up peak of 1.6 times that over the first 3 mm of adhesive. A C-folded liner is
# itself a 180 degree peel, so the tab feels that force; the strip feels it, equal and opposite, at the peel front. Under
# the pad (no adhesive) the liner only slides and unfolds: a light 0.05 N.
PEEL_N = 0.15 / 0.025 * S.PL_WID  # N: steady peel over the adhesive
PEEL_BREAK_N = 1.6 * PEEL_N  # N: the start-up peak where the front first meets the adhesive
BREAK_LEN = 0.003  # m of front travel into the adhesive that the peak lasts
PEEL_PAD_N = 0.05  # N: the liner sliding off the pad and unfolding
STRIP_MASS = 0.0008  # kg: the plaster
G = 9.81
MU_SKIN = 0.5  # the strip's fabric and pad on skin
TACK_SHEAR = 0.1  # N each tacked node holds in shear (about 17 mm^2 of adhesive pressed onto skin)
TETHER_RAMP = 0.001  # m: a tab's pull rises from nothing to full over its last millimetre of slack
TAUT_TOL = 0.0005  # m: a tab within this of taut counts as taut (holds the strip)

SIDES = ("left", "right")
SIGN = {"left": 1.0, "right": -1.0}  # the left hand's tab is the plaster's +u end (toward +y)


def boxes_contain(boxes, pts, pad=0.0):
    """boxes: [(R, t, lo, hi)] (TCP frame boxes posed in the world). True where a point is inside any box."""
    pts = np.atleast_2d(pts)
    hit = np.zeros(len(pts), bool)
    for R, t, lo, hi in boxes:
        q = (pts - t) @ R
        hit |= np.all((q >= lo - pad) & (q <= hi + pad), axis=1)
    return hit


def boxes_dist(boxes, pts):
    """Least distance from the points to any of the boxes."""
    pts = np.atleast_2d(pts)
    best = np.inf
    for R, t, lo, hi in boxes:
        q = (pts - t) @ R
        d = np.linalg.norm(np.maximum(np.maximum(lo - q, q - hi), 0.0), axis=1)
        best = min(best, float(d.min()))
    return best


class Hand:
    """One gripper this step (station frame): TCP rotation R (x: closing axis, z: along the fingers), position t, jaw
    opening ap (m), its collision boxes [(tag, lo, hi)] in the TCP frame, and each fingertip's contact force on the
    forearm (N) by link name."""

    def __init__(self, side, R, t, ap, boxes, force):
        self.side, self.R, self.t, self.ap = side, np.asarray(R, float), np.asarray(t, float), float(ap)
        self.boxes, self.force = boxes, force

    def posed(self, which=None):
        return [(self.R, self.t, lo, hi) for tag, lo, hi in self.boxes if which is None or tag in which]

    def local(self, p):
        return (np.asarray(p, float) - self.t) @ self.R

    def pads(self):
        """[(R, t, lo, hi, force)] for the two fingertip pads, each with its own native contact force on the forearm."""
        out = []
        for tag, lo, hi in self.boxes:
            if tag[0] in ("tip_left", "tip_right") and tag[1] == 0:
                out.append((self.R, self.t, lo, hi, float(self.force.get(tag[0], 0.0))))
        return out


# -- the cream -----------------------------------------------------------------------------------------------------------
class Cream:
    def __init__(self):
        self.ys = np.arange(FIELD_Y[0] + CELL / 2, FIELD_Y[1], CELL)
        self.ss = np.arange(FIELD_S[0] + CELL / 2, FIELD_S[1], CELL)
        self.Y, self.Sg = np.meshgrid(self.ys, self.ss, indexing="ij")
        d = S.wound_dist(self.Y, self.Sg)
        self.wound = d <= 0.0
        self.zone = d <= S.MARGIN
        self.band = d <= S.BAND
        self.reset()

    def reset(self):
        self.h = np.zeros_like(self.Y)
        self.load = 0.0
        self.loaded_total = 0.0
        self.touched_skin = False
        self.double_dip = False
        self.dips = 0
        self.in_pot_now = False
        self.mids = []
        self.dabs = 0
        self.wipe_steps = 0

    def coverage(self) -> float:
        return float(np.mean(self.h[self.zone] >= COVER_H))

    def stray_area(self) -> float:
        """The largest single smear of cream (cells joined side or corner) more than 3 mm past the margin (m^2)."""
        from scipy import ndimage

        lab, n = ndimage.label((self.h >= STRAY_H) & ~self.band, structure=np.ones((3, 3), int))
        if n == 0:
            return 0.0
        return float(np.bincount(lab.ravel())[1:].max()) * CELL * CELL

    def at(self, y, s, r=0.002) -> float:
        """The most cream within r of (y, s) (m)."""
        m = (np.abs(self.Y - y) <= r) & (np.abs(self.Sg - s) <= r)
        return float(self.h[m].max()) if m.any() else 0.0

    def _patch(self, y, s, rho):
        i0, i1 = np.searchsorted(self.ys, [y - rho, y + rho])
        j0, j1 = np.searchsorted(self.ss, [s - rho, s + rho])
        if i1 <= i0 or j1 <= j0:
            return None
        sub = (slice(i0, i1), slice(j0, j1))
        m = (self.Y[sub] - y) ** 2 + (self.Sg[sub] - s) ** 2 <= rho * rho
        return sub, m

    def step(self, prev_tip, tip, held: bool) -> dict:
        """The bud's centre moved from prev_tip to tip this step. Returns what it touched."""
        out = {"skin": False, "dollop": False}
        self.mids = (self.mids + [np.asarray(tip, float)])[-(SLIDE_STEPS + 1):]
        # dip: the bud in the dollop on the cup's floor
        cx, cy = S.CUP_C
        r = math.hypot(tip[0] - cx, tip[1] - cy)
        if r < S.DOLLOP_R:
            surf = S.CUP_Z + 0.002 + S.DOLLOP_H * (1 - (r / S.DOLLOP_R) ** 2)
            if tip[2] - S.APP_TIP_R < surf - 0.0005:
                out["dollop"] = True
                if not self.in_pot_now:
                    self.dips += 1
                    if self.touched_skin:
                        self.double_dip = True
                self.in_pot_now = True
                if held:
                    take = min(LOAD_RATE, LOAD_CAP - self.load)
                    if take > 0:
                        self.load += take
                        self.loaded_total += take
            else:
                self.in_pot_now = False
        else:
            self.in_pot_now = False
        # skin: sub-steps of at most 0.25 mm
        prev_tip = np.asarray(prev_tip, float)
        tip = np.asarray(tip, float)
        n_sub = int(min(40, max(1, math.ceil(np.linalg.norm(tip - prev_tip) / 0.00025))))
        sliding = False
        if len(self.mids) > SLIDE_STEPS:
            d = self.mids[-1] - self.mids[0]
            _, nrm, _ = S.ARM.signed(tip)
            dt = d - (d @ nrm[0]) * nrm[0]
            sliding = float(np.linalg.norm(dt)) >= SLIDE_MIN
        for k in range(1, n_sub + 1):
            p = prev_tip + (tip - prev_tip) * k / n_sub
            if abs(p[0] - S.XW) > S.ARM_A + 0.005 or p[2] > S.ARM_TOP + 0.01 or abs(p[1]) > S.ARM_Y_FLAT:
                continue
            d, nrm, ys = S.ARM.signed(p)
            gap = float(d[0]) - S.APP_TIP_R
            y, s = float(ys[0, 0]), float(ys[0, 1])
            if gap > CONTACT_GAP or abs(y) > 0.2:
                continue
            out["skin"] = True
            self.touched_skin = True
            if not held:
                continue
            squash = min(BUD_SQUASH, max(0.0, -gap))
            rho = 0.0012 + 0.25 * squash  # the contact patch: 1.2 mm lightly, 1.95 mm squashed
            pm = self._patch(y, s, rho)
            if pm is None:
                continue
            sub, m = pm
            h = self.h[sub]
            if sliding:
                hi = m & (h > FILM)
                self.load += float(np.sum(h[hi] - FILM)) * CELL * CELL
                h[hi] = FILM
                lo = np.argwhere(m & (h < FILM))
                for a, b in lo:
                    if self.load <= 1e-15:
                        break
                    give = min((FILM - h[a, b]) * CELL * CELL, self.load)
                    h[a, b] += give / (CELL * CELL)
                    self.load -= give
            else:
                lo = np.argwhere(m & (h < DAB))
                for a, b in lo:
                    if self.load <= 1e-15:
                        break
                    give = min((DAB - h[a, b]) * CELL * CELL, self.load * 0.25)
                    h[a, b] += give / (CELL * CELL)
                    self.load -= give
            self.h[sub] = h
        if out["skin"] and held:
            if sliding:
                self.wipe_steps += 1
            else:
                self.dabs += 1
        return out


# -- the applicator --------------------------------------------------------------------------------------------------------
class Applicator:
    """The applicator's state, from its rigid body's pose (back end b, unit direction d toward the bud) and from who
    holds it (native contact, see Grips). It is a real PhysX body: lifted, turned and pressed by the fingers' friction,
    dropped by gravity."""

    def __init__(self):
        self.reset()

    def reset(self):
        tx, ty = S.TRAY_C
        lx = S.TRAY_SIZE[0]
        self.b = np.array([tx + lx / 2 + S.APP_OVER, ty, APP_REST_Z])  # the stick's back end
        self.d = np.array([-1.0, 0.0, 0.0])  # toward the bud
        self.state = "pouch"  # pouch, held, dropped, binned
        self.side = None
        self.touched_other = None
        self.lifted_once = False
        self.start_z = float(self.tip()[2])

    def tip(self):
        """The bud's centre."""
        return self.b + self.d * (S.APP_LEN - S.APP_TIP_R)

    def mid(self):
        return self.b + self.d * S.APP_LEN / 2

    def set_pose(self, b, d, side):
        """This step's pose, and the hand holding it (None: nobody)."""
        self.b = np.asarray(b, float)
        self.d = np.asarray(d, float) / np.linalg.norm(d)
        self.side = side
        if side is not None:
            self.state = "held"
        elif self.state in ("held", "dropped", "binned"):
            m = self.mid()
            inside = math.hypot(m[0] - S.POT_C[0], m[1] - S.POT_C[1]) < S.POT_R - 0.004 and m[2] < S.POT_H
            self.state = "binned" if inside else "dropped"


def support_height(x, y):
    """Height of whatever fixed surface is under (x, y): the forearm's skin, the trays, the underpad, the table."""
    if abs(x - S.XW) < S.ARM_A and abs(y) < 0.2:
        dx = (x - S.XW) / S.ARM_A
        return S.ARM_ZC + S.ARM_B * math.sqrt(max(0.0, 1 - dx * dx))
    for (cx, cy), (sx, sy, sz) in ((S.TRAY_C, S.TRAY_SIZE), (S.STAND_C, S.STAND_SIZE), (S.CUP_C, (*S.CUP_TRAY, S.CUP_Z))):
        if abs(x - cx) < sx / 2 and abs(y - cy) < sy / 2:
            return sz
    if abs(x - S.XW) < S.PAD_SIZE[0] / 2 and abs(y - S.PAD_YC) < S.PAD_SIZE[1] / 2:
        return S.PAD_Z
    return 0.0


BUD_TOUCH = 0.0005  # m: a bud sphere within this of a surface touches it


def bud_points(b, d):
    """The centres of the bud's three 5 mm spheres (as in props.py), for a stick with back end b and direction d."""
    L, rb, lb = S.APP_LEN, S.APP_TIP_R, S.APP_TIP_LEN
    return np.array([b + d * x for x in (L - rb, L - lb / 2, L - lb + rb)])


def _ring_dist(p, c, r_in, r_out, z0, z1):
    """Distance from p to an upright ring (an annulus r_in..r_out about centre c, from height z0 to z1)."""
    r = math.hypot(p[0] - c[0], p[1] - c[1])
    dr = max(r_in - r, 0.0, r - r_out)
    dz = max(z0 - p[2], 0.0, p[2] - z1)
    return math.hypot(dr, dz)


def _box_dist(p, centre, R, half):
    q = (np.asarray(p, float) - centre) @ R
    return float(np.linalg.norm(np.maximum(np.abs(q) - half, 0.0)))


def _fine_strip(pts):
    """The strip's node grid (NU, NW, 3) refined four times each way (bilinear), so that a point's distance to the
    nearest refined node is its distance to the strip to within a fraction of a millimetre."""
    k = 4
    a = np.linspace(0, NU - 1, (NU - 1) * k + 1)
    b = np.linspace(0, NW - 1, (NW - 1) * k + 1)
    i0 = np.minimum(np.floor(a).astype(int), NU - 2)
    j0 = np.minimum(np.floor(b).astype(int), NW - 2)
    fa, fb = (a - i0)[:, None, None], (b - j0)[None, :, None]
    p00, p10 = pts[i0][:, j0], pts[i0 + 1][:, j0]
    p01, p11 = pts[i0][:, j0 + 1], pts[i0 + 1][:, j0 + 1]
    return ((1 - fa) * (1 - fb) * p00 + fa * (1 - fb) * p10 + (1 - fa) * fb * p01 + fa * fb * p11).reshape(-1, 3)


def bud_contact(buds, hands: dict, pl, held_firm: bool) -> str | None:
    """What the bud touches other than the skin and the cream in the cup, if anything: the cup's wall or rim or its bare
    floor, the waste pot (while a hand still holds the applicator: falling in as it is binned is allowed), the trays, the
    riser, the underpad, the table, the plaster (strip or tabs) or either gripper. buds: the bud spheres' centres."""
    rb, eps = S.APP_TIP_R, BUD_TOUCH
    cx, cy = S.CUP_C
    top = S.CUP_Z + S.CUP_H
    for p in buds:
        x, y, z = p
        # the medicine cup: its wall (the collider ring at the bottom radius, up to the drawn rim) and its floor
        if _ring_dist(p, S.CUP_C, S.CUP_R_BOT + 0.0005, S.CUP_R_TOP, S.CUP_Z, top) <= rb + eps:
            return "cup's rim or wall"
        r_cup = math.hypot(x - cx, y - cy)
        if r_cup < S.CUP_R_BOT and z - rb <= S.CUP_Z + 0.0012 + eps and r_cup > S.DOLLOP_R:
            return "cup's bare floor"
        # the waste pot
        if held_firm:
            if _ring_dist(p, S.POT_C, S.POT_R - 0.0015, S.POT_R + 0.0015, 0.0, S.POT_H) <= rb + eps:
                return "waste pot"
            if math.hypot(x - S.POT_C[0], y - S.POT_C[1]) < S.POT_R and z - rb <= 0.004 + eps:
                return "waste pot"
        # the fixed surfaces under it (not over the forearm, the cup or the pot)
        if not (abs(x - S.XW) < S.ARM_A + 0.002 and abs(y) < 0.2) and r_cup >= S.CUP_R_TOP + 0.003 \
                and math.hypot(x - S.POT_C[0], y - S.POT_C[1]) >= S.POT_R + 0.002:
            h = support_height(x, y)
            if z - rb <= h + eps:
                for name, (c, size) in (("applicator tray", (S.TRAY_C, S.TRAY_SIZE)), ("plaster stand", (S.STAND_C, S.STAND_SIZE)),
                                        ("cup's tray", (S.CUP_C, S.CUP_TRAY))):
                    if abs(x - c[0]) < size[0] / 2 and abs(y - c[1]) < size[1] / 2:
                        return name
                return "underpad" if h > 0 else "table"
        # the grippers
        for k, h in hands.items():
            if boxes_dist(h.posed(), p) <= rb + eps:
                return f"{k} gripper"
        # the liner tabs (and a loose liner, which is its tab body)
        for k in SIDES:
            tp, tR = pl.tab_pose[k]
            centre = tp + np.asarray(tR) @ np.array([0.0, SIGN[k] * S.TAB_OUT / 2, 0.0])
            if _box_dist(p, centre, np.asarray(tR), np.array([S.PL_WID / 2, S.TAB_OUT / 2, S.TAB_THICK / 2])) <= rb + eps:
                return "plaster's liner tab" if pl.liners[k].state == "on" else "loose liner"
    # the strip itself
    pts, _ = pl.nodes()
    lo, hi = pts.reshape(-1, 3).min(0) - 0.01, pts.reshape(-1, 3).max(0) + 0.01
    near = [p for p in buds if np.all(p >= lo) and np.all(p <= hi)]
    if near and pl.state != "fallen":
        fine = _fine_strip(pts)
        for p in near:
            if float(np.min(np.linalg.norm(fine - p, axis=1))) <= rb + S.PL_T / 2 + eps:
                return "plaster"
    return None


# -- the plaster and its liners --------------------------------------------------------------------------------------------
NU, NW = 35, 9


def peel_force(p) -> float:
    """The force (N) to move a liner's peel front on from p (m from the strip's middle)."""
    if p < S.PAD_HALF:
        return PEEL_PAD_N
    if p < S.PAD_HALF + BREAK_LEN:
        return PEEL_BREAK_N
    return PEEL_N


def tab_rest_pose(k):
    """Tab k's body as it lies on the stand, unrotated: its origin at the strip's end (the middle of the tab's root)."""
    cx, cy = S.STAND_C
    return np.array([cx, cy + SIGN[k] * S.PL_LEN / 2, S.STAND_SIZE[2] + S.PL_T / 2 + 0.0003]), np.eye(3)


def tab_out(k, R):
    """The direction out along tab k (from its root) for its body's rotation R."""
    return np.asarray(R) @ np.array([0.0, SIGN[k], 0.0])


def tether_force(tt, g) -> np.ndarray:
    """The force (N, station frame) on a tab whose grip point is at g, from its pull tt = (anchor, taut length, pull,
    constant force): along g -> anchor, full once |g - anchor| reaches the taut length, falling to nothing over the last
    TETHER_RAMP of slack."""
    anchor, taut, pull, const = tt
    v = np.asarray(g, float) - anchor
    d = float(np.linalg.norm(v))
    s = min(1.0, max(0.0, (d - (taut - TETHER_RAMP)) / TETHER_RAMP))
    return -v / max(d, 1e-9) * pull * s + const


class Liner:
    def __init__(self, side):
        self.side = side
        self.p = 0.0  # the peel front's distance from the strip's middle (m): the liner covers |u| >= p on its side
        self.state = "on"  # on, held, dropped, binned
        self.pos = None  # where the loose liner is (its middle), once off
        self.held_by = None


class Plaster:
    def __init__(self):
        self.u = np.linspace(-S.PL_LEN / 2, S.PL_LEN / 2, NU)
        self.w = np.linspace(-S.PL_WID / 2, S.PL_WID / 2, NW)
        self.U, self.W = np.meshgrid(self.u, self.w, indexing="ij")
        self.pad = np.abs(self.U) <= S.PAD_HALF
        self.wing = ~self.pad
        self.reset()

    def reset(self):
        self.state = "stand"  # stand, air, skin, fallen
        cx, cy = S.STAND_C
        self.c = np.array([cx, cy, S.STAND_SIZE[2] + S.PL_T / 2 + 0.0003])
        self.eu, self.ew, self.en = np.array([0, 1.0, 0]), np.array([-1.0, 0, 0]), np.array([0, 0, 1.0])
        self.place = None  # (y0, s0, psi) on the skin
        self.liners = {k: Liner(k) for k in SIDES}
        self.tab_held = {k: None for k in SIDES}  # the hand holding tab k (by native contact), or None
        self.tab_off = {k: None for k in SIDES}  # where it holds it, in the tab body's frame
        self.tab_len = {k: S.TAB_GRIP for k in SIDES}  # m of tab from the strip's end to that grip
        self.tab_pose = {k: tab_rest_pose(k) for k in SIDES}  # each tab body's (origin, rotation) this step
        self.rooted = {k: True for k in SIDES}  # the tab still hinged at its root, where it lies on the opened wrapper
        self.unhinge = []  # tabs to free from their root hinges in the simulator this step
        self.firm = {k: False for k in SIDES}
        self.tacked = np.zeros((NU, NW), bool)
        self.laid = np.zeros((NU, NW), bool)
        self.creamed = np.zeros((NU, NW), bool)
        self._held_tab = {}
        self.lift = {k: None for k in SIDES}  # a lifted wing end: the tab point it is lifted to
        self.events = []
        self.touched_face = False
        self.slid = 0.0  # m the strip was dragged along the skin while lying on it
        self.pressed_now = False
        # forces: what pulls on each tab body this step, as a taut-string pull the simulator applies to that body at every
        # physics step: {k: (anchor (station frame), length at which it is taut (m), pull when taut (N), constant
        # force (N, 3))}. The pull acts along the line from the tab's grip point to the anchor, rising from nothing to
        # full over the last TETHER_RAMP of slack
        self.tether = {}
        self.front_force = np.zeros((NU, NW, 3))  # N on each strip node from the liners' peel fronts this step
        self.hold_n = 0.0  # N the strip can resist on the skin this step (friction, pressed pads, tack)
        self.drag_n = 0.0  # N the liners pull it with

    # -- geometry --
    def side_of(self):
        return {k: (self.U * SIGN[k] > 0) for k in SIDES}

    def exposed(self):
        """Nodes whose underside has no liner: the pad and wing behind each peel front."""
        out = np.zeros((NU, NW), bool)
        for k, ln in self.liners.items():
            out |= (self.U * SIGN[k] > 0) & ((np.abs(self.U) < ln.p) | (ln.state != "on"))
        return out

    def skin_coords(self):
        y0, s0, psi = self.place
        c, s = math.cos(psi), math.sin(psi)
        Y = y0 + self.U * c - self.W * s
        Sv = s0 + self.U * s + self.W * c
        return Y, Sv

    def nodes(self):
        """Node positions (NU, NW, 3) and top normals."""
        if self.state == "skin":
            Y, Sv = self.skin_coords()
            p, n = S.ARM.point(Y, Sv)
            p = p + n * (S.PL_T / 2)
            for k in SIDES:
                if self.lift[k] is not None:
                    p = self._lifted(p, k)
            return p, n
        p = self.c + self.U[..., None] * self.eu + self.W[..., None] * self.ew
        return p, np.broadcast_to(self.en, p.shape).copy()

    def _hinge_row(self, k):
        """Outermost row on side k that cannot lift: a tacked row, or the pad's end."""
        idx = np.where(np.abs(self.u) <= S.PAD_HALF)[0]
        rows = [int(i) for i in range(NU) if self.u[i] * SIGN[k] > 0 and self.tacked[i].any()]
        cand = rows + [int(idx.max() if SIGN[k] > 0 else idx.min())]
        return max(cand, key=lambda i: abs(self.u[i]))

    def _lifted(self, p, k):
        """Rows beyond the hinge swing up as a straight flap toward the lifted tab."""
        h = self._hinge_row(k)
        rows = [i for i in range(NU) if abs(self.u[i]) > abs(self.u[h]) and self.u[i] * SIGN[k] > 0]
        if not rows:
            return p
        base = p[h].mean(axis=0)
        d = self.lift[k] - base
        d = d / (np.linalg.norm(d) or 1.0)
        for i in rows:
            off = p[i] - p[i].mean(axis=0)
            p[i] = base + d * abs(self.u[i] - self.u[h]) + off
        return p

    def end_point(self, k):
        """The middle of the strip's end on side k (world)."""
        p, n = self.nodes()
        i = NU - 1 if SIGN[k] > 0 else 0
        return p[i, NW // 2], n[i, NW // 2]

    def tab_point(self, k):
        """Where tab k is held: held, the grip on its body; free, the point TAB_GRIP out along its body."""
        if self.tab_held[k] is not None and k in self._held_tab:
            return self._held_tab[k]
        p, R = self.tab_pose[k]
        return p + tab_out(k, R) * S.TAB_GRIP

    def set_tabs(self, poses: dict, holders: dict, hands: dict, firm: dict | None = None):
        """The tab bodies' poses this step and which hand holds each (None: nobody). A tab taken while its liner is on
        becomes a handle of the strip: it is freed from the wrapper and the grip's place along it is kept. A liner
        that is off is its tab body: carried while held, dropped by gravity when let go, binned when it lies in the
        waste pot."""
        self.firm = dict(firm) if firm is not None else {k: holders[k] is not None for k in SIDES}
        for k in SIDES:
            self.tab_pose[k] = (np.asarray(poses[k][0], float), np.asarray(poses[k][1], float))
            p, R = self.tab_pose[k]
            h = holders[k]
            ln = self.liners[k]
            if ln.state == "on":
                if h is not None and self.tab_held[k] is None and self.state != "fallen":
                    x = float(np.clip((hands[h].t - p) @ tab_out(k, R), 0.003, S.TAB_OUT))
                    self.tab_off[k], self.tab_len[k], self.tab_held[k] = np.array([0.0, SIGN[k] * x, 0.0]), x, h
                    if self.rooted[k]:
                        self.rooted[k] = False
                        self.unhinge.append(k)
                elif h is None:
                    self.tab_held[k] = None
                continue
            ln.pos = p + tab_out(k, R) * (S.TAB_OUT / 2)
            if ln.state == "held" and h is None:
                ln.state, ln.held_by = "dropped", None
                self._event(f"{k} liner let go")
            elif ln.state != "held" and h is not None:
                ln.state, ln.held_by = "held", h
            if ln.state in ("dropped", "binned"):
                inside = math.hypot(ln.pos[0] - S.POT_C[0], ln.pos[1] - S.POT_C[1]) < S.POT_R - 0.004 and ln.pos[2] < S.POT_H
                new = "binned" if inside else "dropped"
                if new != ln.state:
                    self._event(f"{k} liner {new}")
                ln.state = new

    # -- the step --
    def step(self, hands: dict, cream: Cream):
        self.tether = {}
        self.front_force = np.zeros((NU, NW, 3))
        self.drag_n = 0.0
        self._held_tab = {}
        for k in SIDES:
            if self.tab_held[k] is not None:
                p, R = self.tab_pose[k]
                self._held_tab[k] = p + R @ self.tab_off[k]
        held = [k for k in SIDES if self.tab_held[k] is not None]
        if self.state == "stand":
            if held:
                if any(np.linalg.norm(self.tab_pose[k][0] - tab_rest_pose(k)[0]) > 0.004 for k in held):
                    self.state = "air"
                    self._event("lifted off the stand")
        if self.state == "air":
            self._air(held, cream)
        elif self.state == "skin":
            self._skin(held, hands, cream)
        self._face_check(hands)

    def _event(self, text):
        self.events.append(text)

    def _air(self, held, cream):
        if len(held) == 2:
            gL, gR = self._held_tab["left"], self._held_tab["right"]
            D = float(np.linalg.norm(gL - gR))
            eu = (gL - gR) / max(D, 1e-9)
            en = np.array([0, 0, 1.0]) - eu[2] * eu
            en /= np.linalg.norm(en)
            self.eu, self.en, self.ew = eu, en, np.cross(en, eu)
            L = (self.tab_len["left"] + self.tab_len["right"]) / 2
            hz = min(max(D / 2 - S.PL_LEN / 2, 0.0), L)
            shift = (self.tab_len["left"] - self.tab_len["right"]) / 2  # the longer tab hangs its end further from its hand
            self.c = (gL + gR) / 2 - eu * shift - en * math.sqrt(max(L ** 2 - hz ** 2, 0.0))  # the strip hangs on its tabs
            excess = D - (S.PL_LEN + 2 * L)
            if excess > 0.003:  # pulled apart in the air: both liners peel
                for k in SIDES:
                    self._peel(k, excess / 2)
            # the strip hangs on its tabs (its weight shared), and once they are pulled apart the liners' peel pulls each
            # tab back toward the other (one tension through the strip)
            on = [self.liners[k].p for k in SIDES if self.liners[k].state == "on"]
            T = float(np.mean([peel_force(p) for p in on])) if len(on) == 2 else 0.0
            onset = S.PL_LEN + self.tab_len["left"] + self.tab_len["right"] + max(0.003, 2 * sum(on))
            for k in SIDES:
                o = "right" if k == "left" else "left"
                self.tether[k] = (self._held_tab[o].copy(), onset, T, np.array([0.0, 0.0, -STRIP_MASS * G / 2]))
            # touching down on the skin
            p, _ = self.nodes()
            d, _, _ = S.ARM.signed(p[:, NW // 2])
            firm = all(self.firm.get(k, True) for k in SIDES)  # not while a hand is letting go
            if firm and np.abs(p[:, NW // 2, 1]).max() < 0.2 and float(d.min()) <= S.PL_T / 2 + 0.0005:
                self._land()
        elif len(held) == 1:
            k = held[0]
            g = self._held_tab[k]
            self.eu = SIGN[k] * np.array([0, 0, 1.0])
            self.c = g - np.array([0, 0, self.tab_len[k] + S.PL_LEN / 2])
            self.en = np.array([1.0, 0, 0])
            self.ew = np.cross(self.en, self.eu)
            self.tether[k] = (g.copy(), 1.0, 0.0, np.array([0.0, 0.0, -STRIP_MASS * G]))
        else:
            self._drop()

    def _land(self):
        d, nrm, ys = S.ARM.signed(self.c[None])
        e = self.eu
        t = S.ARM.tangent_s(ys[0, 1])
        psi = math.atan2(float(e @ t), float(e[1]))
        self.place = (float(ys[0, 0]), float(ys[0, 1]), psi)
        self.state = "skin"
        self._event("laid on the skin")

    def _drop(self):
        """Nothing holds the strip in the air: it falls. It settles flat on the forearm only from a short drop (its
        lowest point within DROP_LAND of the skin), roughly level (its axis within 30 degrees of horizontal) and with no
        adhesive bared; a bared strip, one dropped from higher or hanging from one end, or one falling beside the arm,
        crumples or lands crooked or off the arm: fallen."""
        x, y = self.c[0], self.c[1]
        p, _ = self.nodes()
        d, _, _ = S.ARM.signed(p.reshape(-1, 3))
        bared = any(ln.state != "on" or ln.p > 0.0 for ln in self.liners.values())
        level = abs(float(self.eu[2])) < math.sin(math.radians(30.0))  # a strip hanging from one tab lands crooked
        if abs(x - S.XW) < S.ARM_A * 0.8 and abs(y) < 0.15 and float(d.min()) <= DROP_LAND and not bared and level:
            self._land()
            self._event("dropped onto the forearm")
        else:
            self.state = "fallen"
            h = support_height(x, y)
            self.c = np.array([x, y, h + S.PL_T / 2])
            eu = np.array([self.eu[0], self.eu[1], 0.0])
            self.eu = eu / (np.linalg.norm(eu) or 1.0) if np.linalg.norm(eu) > 1e-6 else np.array([0, 1.0, 0])
            self.en = np.array([0, 0, 1.0])
            self.ew = np.cross(self.en, self.eu)
            self._event("dropped with its adhesive bared: crumpled" if bared else "dropped off the forearm")

    def hold_capacity(self, press_n: float) -> float:
        """The pull (N) the strip lying on the skin resists before it slides: friction under its own weight and under the
        fingertip pads pressing on it (press_n, N in all), plus the shear of its tacked adhesive."""
        return MU_SKIN * (STRIP_MASS * G + press_n) + TACK_SHEAR * int((self.tacked & self.laid).sum())

    def _tug(self, k, pull, taut_at, anchor=None):
        """Tab k pulled back toward its end of the strip (or toward anchor) with `pull` N once the grip is taut_at (m of
        tab travel) out."""
        if k in self.tether and self.tether[k][2] >= pull:
            return  # already pulled harder this step (a tab both pulled and anchoring takes the larger)
        e, _ = self.end_point(k)
        self.tether[k] = (e.copy() if anchor is None else anchor, self.tab_len[k] + taut_at, float(pull), np.zeros(3))

    def _front(self, k, pull):
        """The liner's peel front pulls the strip toward the tab with `pull` N, spread over the front's row of nodes, along
        the skin."""
        ln = self.liners[k]
        mine = np.flatnonzero(self.u * SIGN[k] > 0)
        i = int(mine[np.argmin(np.abs(np.abs(self.u[mine]) - ln.p))])
        p, n = self.nodes()
        v = self._held_tab[k] - p[i, NW // 2]
        v = v - (v @ n[i, NW // 2]) * n[i, NW // 2]
        v /= max(np.linalg.norm(v), 1e-9)
        self.front_force[i] += v * (pull / NW)
        self.drag_n += pull

    def _peel(self, k, ext):
        ln = self.liners[k]
        if ln.state != "on":
            return
        newp = max(ln.p, ext / 2)
        if newp > ln.p:
            ln.p = newp
            if ln.p >= S.PL_LEN / 2:
                ln.p = S.PL_LEN / 2
                ln.state = "held" if self.tab_held[k] is not None else "dropped"
                ln.held_by = self.tab_held[k]
                self.tab_held[k] = None
                self._event(f"{k} liner off")

    def _skin(self, held, hands, cream):
        # what presses on the strip: each fingertip pad over it, with its own native contact force
        p, n = self.nodes()
        flat = p.reshape(-1, 3)
        press_hits = np.zeros(NU * NW, bool)
        press_n = 0.0
        for h in hands.values():
            for R, t, lo, hi, f in h.pads():
                inside = boxes_contain([(R, t, lo, hi)], flat, pad=0.002)
                if inside.any():
                    press_n += f
                if f >= PRESS_N:
                    press_hits |= inside
        self.pressed_now = press_n >= 0.5
        cap = self.hold_capacity(press_n)
        self.hold_n = cap
        # lifted off the skin by both tabs (nothing holding it down)
        free = not (self.tacked & self.laid).any() and not self.pressed_now
        if len(held) == 2 and free and min(self._held_tab[k][2] for k in held) > S.ARM_TOP + 0.035:
            self.state = "air"
            self._event("lifted off the skin")
            self._air(held, cream)
            return

        def ext_of(k):
            g = self._held_tab[k]
            e, en = self.end_point(k)
            v = g - e
            dist = float(np.linalg.norm(v))
            elev = math.degrees(math.asin(max(-1.0, min(1.0, float(v @ en) / max(dist, 1e-9)))))
            return dist - self.tab_len[k], elev

        def taut_at(k):  # the tab's travel at which its liner pulls: twice the peel front's (180 degree peel), 1 mm at least
            return max(EXT_MIN, 2 * self.liners[k].p)

        def anchor_of(k):
            """The other tab, if it holds the strip: its hand still gripping it firmly, its liner on, and taut."""
            o = "right" if k == "left" else "left"
            if o in held and self.liners[o].state == "on" and self.firm.get(o, True) and ext_of(o)[0] >= EXT_MIN - TAUT_TOL:
                return o
            return None

        # each pulled liner: its peel resistance pulls the tab back and, at the peel front, the strip toward the tab. The
        # strip resists with its hold on the skin and, through the other liner, the other tab if a hand holds that taut.
        # Held well enough, the front moves on (half the tab's travel); otherwise the strip slides with the pull
        for k in sorted(held, key=lambda q: -ext_of(q)[0]):
            if self.liners[k].state != "on":
                continue
            e, elev = ext_of(k)
            P = peel_force(self.liners[k].p)
            if e <= taut_at(k):
                self.lift[k] = None
                if self.liners[k].p > 0 and e > taut_at(k) - TETHER_RAMP:  # a peel paused, still taut: it pulls back
                    self._tug(k, P if (cap >= P or anchor_of(k)) else cap, taut_at(k))
                continue
            P = peel_force(min(max(self.liners[k].p, e / 2), S.PL_LEN / 2 - 1e-6))  # where the front would move to
            o = "right" if k == "left" else "left"
            if elev > STEEP_DEG:  # pulled steeply: the wing's end lifts (if the strip is held) instead of peeling
                if cap >= P or (o in held and self.liners[o].state == "on"):
                    if self.lift[k] is None:
                        self._event(f"{k} wing lifted by a steep pull")
                    self.lift[k] = self._held_tab[k]
                continue
            self.lift[k] = None
            if cap < P and anchor_of(k) is None:
                # dragged: the strip slides with the tab until the other tab (if a hand holds it) comes taut
                slack = max(0.0, EXT_MIN - ext_of(o)[0]) if (o in held and self.liners[o].state == "on") else np.inf
                slide = min(e - taut_at(k), slack)
                if slide > 1e-5:
                    y0, s0, psi = self.place
                    self.place = (y0 + SIGN[k] * slide * math.cos(psi), s0 + SIGN[k] * slide * math.sin(psi), psi)
                    self.slid += slide
                self._tug(k, cap, taut_at(k))
                self._front(k, cap)
                if anchor_of(k) is None:
                    continue
                e, elev = ext_of(k)
                P = peel_force(min(max(self.liners[k].p, e / 2), S.PL_LEN / 2 - 1e-6))
            self._peel(k, e)
            if self.liners[k].state != "on":
                continue
            self._tug(k, P, taut_at(k))
            self._front(k, P)
            a = anchor_of(k)
            if a is not None and P > cap:
                self._tug(a, P - cap, EXT_MIN)  # the rest of the pull comes onto the other tab and the hand holding it
        for k in SIDES:
            if self.tab_held[k] is None and self.lift[k] is not None:
                self.lift[k] = None
        # exposed adhesive lies down; cream check; tack under a pressing pad
        Y, Sv = self.skin_coords()
        exp = self.exposed()
        lifted = np.zeros((NU, NW), bool)
        for k in SIDES:
            if self.lift[k] is not None:
                h = self._hinge_row(k)
                lifted |= (np.abs(self.U) > abs(self.u[h])) & (self.U * SIGN[k] > 0)
        lay = exp & ~lifted
        new = lay & ~self.laid
        for i, j in np.argwhere(new & self.wing):
            if cream.at(Y[i, j], Sv[i, j]) > STRAY_H:
                if not self.creamed.any():
                    self._event("cream on the adhesive")
                self.creamed[i, j] = True
        self.laid = lay
        hit = press_hits.reshape(NU, NW) & self.laid & self.wing
        self.tacked |= hit

    def _face_check(self, hands):
        """A gripper touching the pad's wound face or the adhesive (the underside with no liner) where the strip is off
        the skin."""
        exp = self.exposed()
        if self.state == "skin":
            off = np.zeros((NU, NW), bool)
            for k in SIDES:
                if self.lift[k] is not None:
                    h = self._hinge_row(k)
                    off |= (np.abs(self.U) > abs(self.u[h])) & (self.U * SIGN[k] > 0)
        else:
            off = np.ones((NU, NW), bool)
        m = exp & off
        if not m.any():
            return
        p, n = self.nodes()
        under = (p - n * (S.PL_T / 2 + 0.0003))[m]
        for h in hands.values():
            if boxes_contain(h.posed(), under, pad=0.0005).any():
                if not self.touched_face:
                    self._event("a gripper touched the pad face or the adhesive")
                self.touched_face = True

    # -- facts for the scorer --
    def pad_offset(self):
        """(distance of the pad's centre from the wound's centre, along the skin (m); the axis's turn from the arm's
        axis (deg))."""
        if self.state != "skin":
            return None
        y0, s0, psi = self.place
        return math.hypot(y0 - S.WOUND_C[0], s0 - S.WOUND_C[1]), abs(math.degrees(psi))

    def covers_zone(self, cream: Cream) -> bool:
        """The pad lies over every cell of the creamed zone (the wound and its margin)."""
        if self.state != "skin":
            return False
        y0, s0, psi = self.place
        c, s = math.cos(psi), math.sin(psi)
        dy, ds = cream.Y[cream.zone] - y0, cream.Sg[cream.zone] - s0
        u = dy * c + ds * s
        w = -dy * s + ds * c
        return bool(np.all((np.abs(u) <= S.PAD_HALF) & (np.abs(w) <= S.PL_WID / 2)))

    def wing_done(self, k) -> bool:
        m = self.wing & (self.U * SIGN[k] > 0)
        return bool(self.liners[k].state != "on" and self.lift[k] is None and np.all(self.tacked[m] & self.laid[m]))

    def wing_tacked_frac(self, k) -> float:
        m = self.wing & (self.U * SIGN[k] > 0)
        return float(np.mean(self.tacked[m]))


class Grips:
    """Which hand holds which part, from native PhysX contact: a hand takes a part when both of its fingertip pads press
    on that part's body with at least GRIP_N for TAKE_STEPS control steps in a row, its jaw closed on the part (opening
    within JAW_SLACK of the part's thickness); it has let go once either pad's force stays under LET_GO_N for
    LET_GO_STEPS control steps. A part moves only as its rigid body does (friction in the grip, gravity)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.holding = {k: None for k in SIDES}
        self.weak = {k: 0 for k in SIDES}
        self.firm_for = {k: {} for k in SIDES}  # control steps in a row each part has been pressed by both pads

    def firm(self, part) -> str | None:
        """The hand holding `part` with both pads still pressing this step (not in the middle of letting go)."""
        for k in SIDES:
            if self.holding[k] == part and self.weak[k] == 0:
                return k
        return None

    def step(self, contact: dict, openings: dict | None = None):
        """contact[side][part] = (force of tip_left, force of tip_right) on that part's body (N); openings[side]: the
        jaw's opening (m)."""
        for k in SIDES:
            c = contact.get(k, {})
            what = self.holding[k]
            if what is not None:
                f = c.get(what, (0.0, 0.0))
                self.weak[k] = self.weak[k] + 1 if min(f) < LET_GO_N else 0
                if self.weak[k] >= LET_GO_STEPS:
                    self.holding[k], self.weak[k] = None, 0
                continue
            run = self.firm_for[k]
            for part in list(run):
                if part not in c:
                    run.pop(part)
            ap = 0.0 if openings is None else openings[k]
            for part, f in c.items():
                closed = ap <= PART_W.get(part, 0.0) + JAW_SLACK
                run[part] = run.get(part, 0) + 1 if min(f) >= GRIP_N and closed else 0
            for part, n in run.items():
                if n >= TAKE_STEPS and part not in self.holding.values():
                    self.holding[k] = part
                    run.clear()
                    break


PARTS = ("applicator", "tab_left", "tab_right")


class World:
    """One trial: the cream, the applicator, the plaster, and who holds what."""

    def __init__(self):
        self.cream = Cream()
        self.app = Applicator()
        self.pl = Plaster()
        self.grips = Grips()
        self.n = 0

    def reset(self):
        self.cream.reset()
        self.app.reset()
        self.pl.reset()
        self.grips.reset()
        self.n = 0

    def step(self, hands: dict, parts: dict):
        """parts: {"pose": {part: (origin, rotation)}, "contact": {side: {part: (f_tip_left, f_tip_right)}}}, read from
        the simulator (the applicator's and the two tab bodies' poses, the fingertips' native contact forces on them)."""
        self.n += 1
        self.grips.step(parts["contact"], {k: h.ap for k, h in hands.items()})
        who = {v: k for k, v in self.grips.holding.items() if v is not None}
        prev_tip = self.app.tip()
        p, R = parts["pose"]["applicator"]
        self.app.set_pose(p, np.asarray(R)[:, 0], who.get("applicator"))
        tip = self.app.tip()
        if self.app.state != "pouch":
            self.cream.step(prev_tip, tip, self.app.state == "held")
            if self.app.state == "held" and tip[2] > self.app.start_z + 0.01:
                self.app.lifted_once = True
        self.pl.set_tabs({k: parts["pose"][f"tab_{k}"] for k in SIDES}, {k: who.get(f"tab_{k}") for k in SIDES}, hands,
                         {k: self.grips.firm(f"tab_{k}") is not None for k in SIDES})
        self.pl.step(hands, self.cream)
        # the bud touching anything but the skin and the cream in the cup, once lifted from its pouch and until binned
        if self.app.lifted_once and self.app.state != "binned" and self.app.touched_other is None:
            other = bud_contact(bud_points(self.app.b, self.app.d), hands, self.pl, self.grips.firm("applicator") is not None)
            if other:
                self.app.touched_other = other


def view(w: World, table_z: float) -> dict:
    """What the cameras show, in the world frame: the applicator's pose, where each liner tab is, the loose liners. The
    demo reads only this (and the fixed scene, and its own joints)."""
    up = np.array([0.0, 0.0, table_z])
    return {"app_back": w.app.b + up, "app_dir": w.app.d.copy(), "app_tip": w.app.tip() + up,
            "ends": {k: w.pl.end_point(k)[0] + up for k in SIDES},
            "tab": {k: w.pl.tab_point(k) + up for k in SIDES if w.pl.liners[k].state == "on"},
            "tab_root": {k: w.pl.tab_pose[k][0] + up for k in SIDES},
            "liner": {k: (None if w.pl.liners[k].pos is None else w.pl.liners[k].pos + up) for k in SIDES}}
