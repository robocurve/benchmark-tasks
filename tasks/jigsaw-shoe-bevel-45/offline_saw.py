"""Kinematics of the saw's shoe and lever for use ONLY where there is no simulator: tools/offline/check_task.py (whose
own carry proxy can only carry and drop a gripped box) and the tests. The simulator never takes a result from here:
there the slide, the bevel and the lever are PhysX joints moved by the fingers' contacts, and shoerule.py's targets
are written to their drives every control step.

What this file assumes is what the design claims, so the offline prediction is the design's, not a measurement. It
moves the joints from the arms' COMMANDED pad-tip points and openings (rigstate's offline feed). Every gripper is
taken to point straight down with its fingers closing along world y (wrist yaw 0, as the demo has it):

- a CLOSED gripper (pads under 4 mm apart) is a solid block 20 mm along x, the two pads and their gap along y, from
  the pad tips 38 mm up. Where the block overlaps the lever or the shoe, that joint is turned (or slid, for none:
  the slide is only pulled) the least amount, either way, that clears the block, within its limits and only if the
  rule lets it move (shoerule: the bevel is held at 0 on the stop and everything is held when clamped). A blocked
  joint stays: the arm's push then goes into the saw, which offline does not move;
- an OPEN gripper whose pads straddle the lever (its 6 mm plate between the fingers, the pads' 20 x 38 mm side over
  some of it) and then close below 6 mm pinches it: the fingers close along the lever's hinge axis, so the lever turns
  with the pinched spot (over the pad tips by half the height of lever they cover) about its pivot, within 0 to 70
  deg; it lets go above 7.5 mm;
- an OPEN gripper whose pads straddle the shoe's edge (axis within 6 mm of the edge's middle, pad tips 2 to 20 mm
  under the edge, the pads over the shoe's length) and then close below the edge's 10 mm holds it; held, the slide
  follows the gripper's x travel (0 to 10 mm) when the rule lets it slide; it lets go above 11.5 mm;
- with no block on it, the lever goes to its rest (flush under 25 deg, 60 deg above) and a free bevel within 3 deg of
  a detent goes to it;
- the body does not move: no push is strong enough to slide 2.45 kg on rubber (the scorer's off-the-mat zero is
  for the simulator).
"""

from __future__ import annotations

import math

import numpy as np

import layout as L
import rigstate
import shoerule as R

PUSH_GAP = 0.004  # m: closed
PAD_W = 20.0  # mm along x
PAD_T = 8.0
PAD_LEN = 38.0
HOLD_AXIS_TOL = 6.0  # mm
RELEASE_GAP = (L.EDGE_W + 1.5) * L.MM
LEVER_T = L.LEVER_Y[1] - L.LEVER_Y[0]  # 6 mm
LEVER_MID_Y = (L.LEVER_Y[0] + L.LEVER_Y[1]) / 2
LEVER_RELEASE_GAP = (LEVER_T + 1.5) * L.MM
STEP_DEG = 0.25
SEARCH_DEG = 12.0  # a block moves a part at most this far in one step: it never jumps through it
SEARCH_DIR_DEG = 30.0  # the same once the push's way is known (the lever turns 20 deg for 2 mm near its tab's foot)
SETTLE_DEG_PER_STEP = 3.0


class OfflineSaw:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.slide = 0.0  # mm
        self.bevel = 0.0  # deg
        self.lever = 0.0  # deg
        self.held = None  # (side, tcp x at grasp mm, slide at grasp)
        self.pinch = None  # (side, the pinched spot's angle about the lever pivot (deg), lever at the pinch, the spot's
        # height over the pad tips (mm))
        self.around_lever = {s: False for s in rigstate.ARMS}
        self.around = {s: False for s in rigstate.ARMS}
        self.events: list = []
        self.push_dir: dict = {}
        self.prev_tips: dict = {}

    # -- geometry ---------------------------------------------------------------------------------------------------
    @staticmethod
    def to_b(p) -> np.ndarray:
        return (np.asarray(p, dtype=float) - np.asarray(L.ORIGIN_M)) / L.MM

    @staticmethod
    def block(tips_b, gap_mm):
        lo = np.array([tips_b[0] - PAD_W / 2, tips_b[1] - gap_mm / 2 - PAD_T, tips_b[2]])
        hi = np.array([tips_b[0] + PAD_W / 2, tips_b[1] + gap_mm / 2 + PAD_T, tips_b[2] + PAD_LEN])
        return lo, hi

    @staticmethod
    def overlaps(pts, blk, margin: float = 0.3) -> bool:
        lo, hi = blk
        inside = np.all((pts > lo + margin) & (pts < hi - margin), axis=1)
        return bool(np.any(inside))

    def _lever_hit(self, deg, blocks) -> bool:
        pts = L.lever_points(math.radians(deg))
        return any(self.overlaps(pts, b) for b in blocks)

    @staticmethod
    def tip_angle(tips_b) -> float:
        """A point's angle (deg) about the lever's hinge, in the lever's own sense (turning the lever by +a turns a
        point riding on it by +a)."""
        r = np.asarray(tips_b, dtype=float) - np.asarray(L.LEVER_PIVOT)
        return -math.degrees(math.atan2(r[2], r[0]))

    def _lever_between(self, tips_b, gap_mm=None) -> bool:
        """The pads' side (20 mm along x, 38 up) over some of the lever's plate and, given the fingers' gap, the plate
        between them."""
        if gap_mm is not None and abs(tips_b[1] - LEVER_MID_Y) > (gap_mm - LEVER_T) / 2:
            return False
        pts = L.lever_points(math.radians(self.lever))
        inx = np.abs(pts[:, 0] - tips_b[0]) <= PAD_W / 2
        inz = (pts[:, 2] >= tips_b[2]) & (pts[:, 2] <= tips_b[2] + PAD_LEN)
        return bool(np.any(inx & inz))

    def _pinch_up(self, tips_b) -> float:
        """How far over the pad tips the pinched spot is: the middle of the height of the lever the pads cover."""
        pts = L.lever_points(math.radians(self.lever), n=15)
        sel = pts[(np.abs(pts[:, 0] - tips_b[0]) <= PAD_W / 2) & (pts[:, 2] >= tips_b[2])
                  & (pts[:, 2] <= tips_b[2] + PAD_LEN)]
        return 0.0 if len(sel) == 0 else float(sel[:, 2].max() - tips_b[2]) / 2

    def _shoe_hit(self, deg, blocks) -> bool:
        pts = L.shoe_points(math.radians(deg), self.slide)
        return any(self.overlaps(pts, b) for b in blocks)

    def _clear_dir(self, key, hit, start, lo, hi, hint: int = 0):
        """The angle at which ``hit`` is false, turning the part only the way the push turns it: ``hint`` (+1 / -1, the
        sign of the block's motion's moment about the joint) at a push's first contact, then the same way while the
        contact lasts (a push does not reverse). Without a hint, the nearest clear angle either way."""
        d = self.push_dir.get(key) or hint
        if d:
            n = int(math.ceil(SEARCH_DIR_DEG / STEP_DEG))
            for k in range(0, n + 1):
                a = start + d * k * STEP_DEG
                if not lo - 1e-9 <= a <= hi + 1e-9:
                    break
                if not hit(a):
                    self.push_dir[key] = d
                    return a
            return None
        a = self._clear(hit, start, lo, hi)
        if a is not None and a != start:
            self.push_dir[key] = 1 if a > start else -1
        return a

    def _hint(self, pts, pivot, axis, blocks, motions) -> int:
        """The sign of the moment about ``axis`` through ``pivot`` of the blocks' motions where they overlap."""
        m = 0.0
        for blk, v in zip(blocks, motions):
            lo, hi = blk
            inside = np.all((pts > lo + 0.3) & (pts < hi - 0.3), axis=1)
            if np.any(inside) and np.linalg.norm(v) > 1e-6:
                r = pts[inside].mean(axis=0) - np.asarray(pivot)
                m += float(np.dot(np.cross(r, v), axis))
        return 0 if abs(m) < 1e-9 else (1 if m > 0 else -1)

    @staticmethod
    def _clear(hit, start, lo, hi):
        """The angle nearest ``start`` within [lo, hi] and within SEARCH_DEG of it at which ``hit`` is false (None if
        there is none: the part is against its limit, and the block cannot pass through it)."""
        if not hit(start):
            return start
        n = int(math.ceil(SEARCH_DEG / STEP_DEG))
        for k in range(1, n + 1):
            for a in (start + k * STEP_DEG, start - k * STEP_DEG):
                if lo - 1e-9 <= a <= hi + 1e-9 and not hit(a):
                    return a
        return None

    # -- one control step -------------------------------------------------------------------------------------------
    def step(self, arms) -> None:
        read = rigstate.read_arms(arms)
        if read is None:
            return
        grips = {}
        for side, (tcp, tips, opening) in zip(rigstate.ARMS, read):
            gap = float(np.clip(opening, 0.0, 1.0)) * rigstate.FULL_GAP
            grips[side] = (self.to_b(tcp), self.to_b(tips), gap)
        # the pinch on the lever
        for side, (tcp, tips, gap) in grips.items():
            if self.pinch is not None and self.pinch[0] == side:
                if gap > LEVER_RELEASE_GAP:
                    self.pinch = None
                    self.events.append(("release lever", side))
                continue
            if (self.pinch is None and self.around_lever[side] and gap < LEVER_T * L.MM
                    and self._lever_between(tips)):
                up = self._pinch_up(tips)
                self.pinch = (side, self.tip_angle(tips + np.array([0.0, 0.0, up])), self.lever, up)
                self.events.append(("pinch lever", side))
            if gap > (LEVER_T + 2.0) * L.MM:  # open astride it; kept while the fingers close over it
                self.around_lever[side] = self._lever_between(tips, gap / L.MM)
            else:
                self.around_lever[side] = bool(self.around_lever[side] and self._lever_between(tips))
        holders = {h[0] for h in (self.held, self.pinch) if h is not None}
        blocks, motions = [], []
        for side, (tcp, tips, gap) in grips.items():
            prev = self.prev_tips.get(side)
            self.prev_tips[side] = tips
            if gap < PUSH_GAP and side not in holders:  # a hand holding a part is not a block pushing it
                blocks.append(self.block(tips, gap / L.MM))
                motions.append(np.zeros(3) if prev is None else tips - prev)
        # the lever
        lever_pushed = blocks and self._lever_hit(self.lever, blocks)
        if self.pinch is not None:
            side, a_at, lever_at, up = self.pinch
            spot = grips[side][1] + np.array([0.0, 0.0, up])
            self.lever = float(np.clip(lever_at + self.tip_angle(spot) - a_at, 0.0, L.LEVER_MAX_DEG))
            self.push_dir.pop("lever", None)
        elif lever_pushed:
            hint = self._hint(L.lever_points(math.radians(self.lever)), L.LEVER_PIVOT, (0.0, 1.0, 0.0), blocks, motions)
            a = self._clear_dir("lever", lambda d: self._lever_hit(d, blocks), self.lever, 0.0, L.LEVER_MAX_DEG, hint)
            if a is not None:
                self.lever = a
        else:
            self.push_dir.pop("lever", None)
            rest = 0.0 if self.lever < L.LEVER_SNAP_DEG else L.LEVER_OPEN_DEG
            d = float(np.clip(rest - self.lever, -SETTLE_DEG_PER_STEP, SETTLE_DEG_PER_STEP))
            if d and not (blocks and self._lever_hit(self.lever + d, blocks)):
                self.lever += d
        # the grip on the shoe's edge and the slide
        top = L.Z_C + L.SHOE_HALF_W
        mid_y = L.EDGE_W / 2
        for side, (tcp, tips, gap) in grips.items():
            if self.held is not None and self.held[0] == side:
                if gap > RELEASE_GAP:
                    self.held = None
                    self.events.append(("release shoe", side))
                continue
            x0, x1 = L.SHOE_X[0] + self.slide, L.SHOE_X[1] + self.slide
            near = (abs(self.bevel) < 1.0 and abs(tcp[1] - mid_y) <= HOLD_AXIS_TOL
                    and top - 20.0 <= tips[2] <= top - 2.0
                    and x0 + PAD_W / 2 <= tcp[0] <= x1 - PAD_W / 2)
            if near and self.around[side] and gap < L.EDGE_W * L.MM and self.held is None:
                self.held = (side, float(tcp[0]), self.slide)
                self.events.append(("grasp shoe", side))
            if gap > (L.EDGE_W + 2.0) * L.MM:  # open astride it; kept while the fingers close over it
                self.around[side] = near
            else:
                self.around[side] = bool(self.around[side] and near)
        if self.held is not None and not R.clamped(math.radians(self.lever)):
            side, x_at, s_at = self.held
            lo = L.SLIDE_REL if abs(self.bevel) > L.TILTED_DEG else 0.0
            self.slide = float(np.clip(s_at + grips[side][0][0] - x_at, lo, L.SLIDE_TRAVEL))
        # the bevel
        free = (not R.clamped(math.radians(self.lever))) and R.off_stop(self.slide * L.MM, math.radians(self.bevel))
        if blocks and self._shoe_hit(self.bevel, blocks):
            if free:
                piv = np.array(L.PIVOT) + np.array([self.slide, 0.0, 0.0])
                hint = self._hint(L.shoe_points(math.radians(self.bevel), self.slide), piv, (1.0, 0.0, 0.0), blocks,
                                  motions)
                a = self._clear_dir("bevel", lambda d: self._shoe_hit(d, blocks), self.bevel, -L.BEVEL_MAX_DEG,
                                    L.BEVEL_MAX_DEG, hint)
                if a is not None:
                    self.bevel = a
        else:
            self.push_dir.pop("bevel", None)
        if free and not (blocks and self._shoe_hit(self.bevel, blocks)):
            d = L.nearest_detent_deg(self.bevel)
            if abs(self.bevel - d) <= L.DETENT_CAPTURE_DEG and not (blocks and self._shoe_hit(d, blocks)):
                self.bevel = d

    def joints(self) -> dict:
        return {"slide": self.slide * L.MM, "bevel": math.radians(self.bevel), "lever": math.radians(self.lever)}
