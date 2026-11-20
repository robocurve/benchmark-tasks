"""The scoring rules, as a plain state machine fed once per control step with what the overhead videos, a blot, a depth gauge, a meter and
a scale would show. Pure Python and NumPy: test it on hand-built states with test_cellrules.py.

The tiers follow task.json's `scoring`, line by line (see `CellJudge.tier`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

ORDER = ("zinc_1", "separator", "zinc_2", "spacer", "spring", "cap")
STACK = ("zinc_1", "separator", "zinc_2", "spacer", "spring")  # the cap rests on the spring by its top's inside
DIAM = {"zinc_1": 0.012, "zinc_2": 0.012, "separator": 0.019, "spacer": 0.0158, "spring": 0.015, "cap": 0.0193}
THICK = {"zinc_1": 0.0001, "zinc_2": 0.0001, "separator": 0.00026, "spacer": 0.0010, "spring": 0.0012}
CAP_H, CAP_TOP_T = 0.0020, 0.00025
CAN_ID = 0.0195
GAUGE_R = 0.0065  # m: the depth gauge reads the spacer's top at 4 points on this radius, 90 degrees apart
GAUGE_SPREAD = 0.0003  # m: the 4 readings agree within this (the stack under the spacer lies flat)
GAUGE_HIGH = 0.0003  # m: and none is higher than the nominal stack by more than this (nothing doubled, folded or raised)
NOMINAL_STACK = 0.0001 + 0.00026 + 0.0001 + 0.0010  # m: zinc, separator, zinc, spacer, floor to the spacer's top
SEP_GAP = 0.00005  # m: the separator lies above the first zinc and under the second (the meter: no zinc-to-zinc contact)
FINGER_NEAR = 0.002  # m: a fingertip this close to a part's outline counts as touching it
MOVED = 0.0010  # m: a part moved this far in all by gripper fingertips (summed over every contact) is displaced
OUT_UP, OUT_SIDE = 0.006, 0.0115  # m: a part this far above the rim, or this far from the axis (past the wall), is out
CLEAR = 0.030  # m: fingers clear of the tools and of the cell
CAN_R = 0.010  # m: the can's outside radius
CLEAR_S = 1.0
ARM_AWAY = 0.05  # m: the left fingers this far from the can's axis: "the arm is away at a dish"
BAND_FULL, BAND_HALF = (0.080, 0.120), (0.050, 0.150)


@dataclass
class Layout:
    top: float  # the table
    axis: tuple  # the can's axis, x y
    floor: float  # the can's floor (inside)
    rim: float  # the can's rim
    stand: tuple  # the tweezer stand's slot centre x y, and the stand's top z
    holder: tuple  # the pipette's home origin (tip end) x y z


@dataclass
class Part:
    pos: np.ndarray  # the origin: centre of the bottom face (for the cap: of the skirt's bottom)
    rot: np.ndarray  # 3 x 3

    @property
    def up(self) -> np.ndarray:
        return self.rot[:, 2]

    def tilt_deg(self) -> float:
        return math.degrees(math.acos(max(-1.0, min(1.0, float(self.up[2])))))

    def centre_xy(self, height: float) -> np.ndarray:
        """The centre of the part's mid-plane, seen from above."""
        return (self.pos + self.up * height / 2)[:2]


def gauge_mm(spacer: Part, floor: float) -> list:
    """The depth gauge: the spacer's top at 4 points on GAUGE_R, 90 degrees apart, as heights over the can's floor, mm.
    Read after the cap and the spring are lifted off, so the reading needs neither."""
    out = []
    for a in (0.0, 90.0, 180.0, 270.0):
        r = math.radians(a)
        local = np.array([GAUGE_R * math.cos(r), GAUGE_R * math.sin(r), THICK["spacer"]])
        out.append(round(float((spacer.pos + spacer.rot @ local)[2] - floor) * 1000.0, 3))
    return out


@dataclass
class Snapshot:
    t: float
    parts: dict  # name -> Part
    left: np.ndarray  # (n, 3) fingertip points
    right: np.ndarray
    tweezers: tuple  # (tips' centre, head) world points
    tweezers_tilt: float
    pipette: tuple  # (tip end, button top)
    pipette_tilt: float
    can_g: float
    spilled_g: float
    dispense_times: tuple  # times of dispenses onto the separator
    cap_pressing: bool = False  # the right arm's fingers bear on the cap's top


@dataclass
class CellJudge:
    lay: Layout
    arrived: dict = field(default_factory=dict)  # part -> time it first lay in the can
    frame: dict = field(default_factory=dict)  # part -> its centre xy in its placement frame
    frozen: set = field(default_factory=set)
    gripper_moved: list = field(default_factory=list)
    contact: dict = field(default_factory=dict)  # (side, part) -> the part's position when that contact began
    finger_travel: dict = field(default_factory=dict)  # part -> its travel over finished contacts with the fingertips, m
    left_can: list = field(default_factory=list)  # parts that left the can after they had been laid in it
    peak: dict = field(default_factory=dict)  # part -> (highest over the rim, farthest from the axis) since it arrived, m
    pre_press_sep: bool | None = None  # the separator between the zinc discs in the latest step before the press began
    press_started: bool = False
    blot_wet: bool = False
    left_table: list = field(default_factory=list)
    pressed: bool = False
    clear_since: float | None = None
    last: Snapshot | None = None
    notes: list = field(default_factory=list)

    # -- per step ---------------------------------------------------------------------------------------------------
    def in_can(self, name: str, p: Part) -> bool:
        off = np.linalg.norm(p.pos[:2] - np.asarray(self.lay.axis))
        return off < CAN_ID / 2 and self.lay.floor - 0.0005 < p.pos[2] < self.lay.rim + 0.0005

    def offset(self, name: str, p: Part) -> float:
        h = THICK.get(name, CAP_H)
        return float(np.linalg.norm(p.centre_xy(h) - np.asarray(self.lay.axis)))

    def update(self, s: Snapshot) -> None:
        lay = self.lay
        for name, p in s.parts.items():
            if p.pos[2] < lay.top - 0.005:
                if name not in self.left_table:
                    self.left_table.append(name)
            if name not in self.arrived and self.in_can(name, p) and p.tilt_deg() < 45:
                self.arrived[name] = s.t
                k = ORDER.index(name)
                for below in ORDER[:k]:  # a part laid on top hides the ones below: their frames are final
                    self.frozen.add(below)
            elif name in self.arrived:
                up, side = p.pos[2] - lay.rim, float(np.linalg.norm(p.pos[:2] - np.asarray(lay.axis)))
                hi, far = self.peak.get(name, (-math.inf, 0.0))
                self.peak[name] = (max(hi, up), max(far, side))
                if (up > OUT_UP or side > OUT_SIDE) and name not in self.left_can:
                    self.left_can.append(name)  # each part once: lifted back out after it was laid in
        if s.cap_pressing and "cap" in self.arrived:
            self.press_started = True
        if not self.press_started:
            self.pre_press_sep = self.separated(s)
        away = s.left.size == 0 or float(np.min(np.linalg.norm(s.left[:, :2] - np.asarray(lay.axis), axis=1))) > ARM_AWAY
        for name in self.arrived:
            if name in self.frozen or name not in s.parts:
                continue
            p = s.parts[name]
            if away and self.in_can(name, p):
                self.frame[name] = p.centre_xy(THICK.get(name, CAP_H))
        # a gripper's direct contact displacing a part: its total travel over one contact (the cap press is exempt)
        if self.last is not None:
            for name, p in s.parts.items():
                q = self.last.parts.get(name)
                if q is None:
                    continue
                for side, pts in (("left", s.left), ("right", s.right)):
                    key = (side, name)
                    if side == "right" and (s.cap_pressing or self.last.cap_pressing) and self.under_cap(s, name):
                        self.contact.pop(key, None)  # the press: the fingers on the cap's top move the cap and the stack under it
                        continue
                    if not (touching(pts, p, name) or touching(getattr(self.last, side), q, name)):
                        start = self.contact.pop(key, None)
                        if start is not None:  # a contact ends: its travel adds to the part's total
                            self.finger_travel[name] = self.finger_travel.get(name, 0.0) + float(np.linalg.norm(q.pos - start))
                        continue
                    start = self.contact.setdefault(key, q.pos.copy())
                    total = self.finger_travel.get(name, 0.0) + float(np.linalg.norm(p.pos - start))
                    if total >= MOVED and not any(g[1:] == key for g in self.gripper_moved):
                        self.gripper_moved.append((round(s.t, 2), side, name))
        # the blot: a fingertip in the wet separator's well
        if s.can_g > 0 and not ("cap" in s.parts and self.in_can("cap", s.parts["cap"])):
            for pts in (s.left, s.right):
                if pts.size and np.any((np.linalg.norm(pts[:, :2] - np.asarray(lay.axis), axis=1) < CAN_ID / 2) & (pts[:, 2] < lay.rim + 0.0015)):
                    self.blot_wet = True
        # the press: the right fingers on the cap, the cap seated and pushed below where the spring holds it
        if s.cap_pressing and "cap" in s.parts:
            rest = self.cap_rest(s)
            if rest is not None and self.seated(s) and s.parts["cap"].pos[2] < rest - 0.0002:
                self.pressed = True
        # fingers clear of the tools and of the cell (the press hand lifted away)
        if fingers_clear(s) and self.clear_of_cell(s):
            if self.clear_since is None:
                self.clear_since = s.t
        else:
            self.clear_since = None
        self.last = s

    def clear_of_cell(self, s: Snapshot) -> bool:
        """Every fingertip at least CLEAR from the can and the cap in its mouth."""
        pts = np.vstack([p for p in (s.left, s.right) if p.size]) if (s.left.size or s.right.size) else np.zeros((0, 3))
        if pts.size == 0:
            return True
        radial = np.maximum(np.linalg.norm(pts[:, :2] - np.asarray(self.lay.axis), axis=1) - CAN_R, 0.0)
        top = self.lay.rim + CAP_H
        vertical = np.maximum(np.maximum(pts[:, 2] - top, self.lay.top - pts[:, 2]), 0.0)
        return bool(np.hypot(radial, vertical).min() >= CLEAR)

    def under_cap(self, s: Snapshot, name: str) -> bool:
        """The cap, or a part lying in the can under the cap, while the cap is in the can's mouth."""
        cap = s.parts.get("cap")
        if cap is None or not self.in_can("cap", cap):
            return False
        return name == "cap" or (name in s.parts and self.in_can(name, s.parts[name]))

    # -- the state's checks -----------------------------------------------------------------------------------------
    def cap_rest(self, s: Snapshot) -> float | None:
        """The cap's origin height when its top's inside rests on the uncompressed spring, standing on the spacer."""
        if "spacer" not in s.parts:
            return None
        sp = s.parts["spacer"]
        return sp.pos[2] + sp.up[2] * THICK["spacer"] + THICK["spring"] - (CAP_H - CAP_TOP_T)

    def seated(self, s: Snapshot) -> bool:
        """Open side down, square, the gasket inside the can's mouth all round."""
        c = s.parts.get("cap")
        if c is None or c.up[2] < 0:
            return False
        return c.tilt_deg() < 2.0 and self.offset("cap", c) < 0.0004 and c.pos[2] < self.lay.rim and self.in_can("cap", c)

    def placed(self, s: Snapshot, names) -> tuple[bool, str]:
        """Each part lies in the can (seen in the overhead video)."""
        for name in names:
            p = s.parts.get(name)
            if p is None or name not in self.arrived or not self.in_can(name, p):
                return False, f"{name} not in the can"
        return True, ""

    def separated(self, s: Snapshot) -> bool:
        """The separator lies above the first zinc and under the second, so the zinc discs cannot touch."""
        z1, sep, z2 = (s.parts.get(n) for n in ("zinc_1", "separator", "zinc_2"))
        if None in (z1, sep, z2):
            return False
        return bool(sep.pos[2] > z1.pos[2] + SEP_GAP and z2.pos[2] > sep.pos[2] + SEP_GAP)

    def gauge(self, s: Snapshot) -> tuple[bool, str]:
        """The depth gauge on the spacer's top: the stack under it flat, and nothing doubled, folded or raised."""
        sp = s.parts.get("spacer")
        if sp is None or sp.up[2] < 0.5:
            return False, "spacer not lying in the can"
        h = gauge_mm(sp, self.lay.floor)
        if max(h) - min(h) > GAUGE_SPREAD * 1000 + 1e-9:
            return False, f"not flat: spacer top {h} mm over the floor (spread over {GAUGE_SPREAD * 1000:.1f} mm)"
        if max(h) > (NOMINAL_STACK + GAUGE_HIGH) * 1000 + 1e-9:
            return False, f"stack too high: spacer top {h} mm over the floor (over {(NOMINAL_STACK + GAUGE_HIGH) * 1000:.2f} mm)"
        return True, ""

    def sinks_mm(self, s: Snapshot) -> dict:
        """Each part's height over the top of the one below (negative: sunk into it), mm; for the log."""
        out, below = {}, self.lay.floor
        for name in STACK:
            p = s.parts.get(name)
            if p is None:
                break
            out[name] = round((p.pos[2] - below) * 1000, 3)
            below = p.pos[2] + p.up[2] * THICK[name]
        return out

    def stack_ok(self, s: Snapshot) -> tuple[bool, str]:
        """Zinc, separator, zinc and spacer in the can, and the depth gauge on the spacer passes."""
        ok, why = self.placed(s, STACK[:4])
        if not ok:
            return ok, why
        return self.gauge(s)

    def in_order(self, names) -> bool:
        times = [self.arrived.get(n) for n in names]
        return all(t is not None for t in times) and all(a < b for a, b in zip(times, times[1:]))

    def no_short(self, s: Snapshot) -> bool:
        """The meter, cap and spring lifted off, probe on the spacer, reads over R0: the separator lies between the zinc
        discs (judged before the press, since the simulated thin discs can sink into each other under it) and covers
        both, and the spacer does not touch the can's wall."""
        z1, sep, z2 = (s.parts.get(n) for n in ("zinc_1", "separator", "zinc_2"))
        if None in (z1, sep, z2):
            return False
        sc = sep.centre_xy(THICK["separator"])
        cover = (DIAM["separator"] - DIAM["zinc_1"]) / 2
        if np.linalg.norm(z1.centre_xy(1e-4) - sc) > cover or np.linalg.norm(z2.centre_xy(1e-4) - sc) > cover:
            return False
        if not (self.pre_press_sep if self.press_started else self.separated(s)):
            return False
        if "spacer" not in s.parts or self.offset("spacer", s.parts["spacer"]) >= (CAN_ID - DIAM["spacer"]) / 2:
            return False
        return True

    def tools_stowed(self, s: Snapshot) -> bool:
        sx, sy, stop = self.lay.stand
        tips = np.asarray(s.tweezers[0])
        tw = abs(tips[0] - sx) < 0.004 and abs(tips[1] - sy) < 0.017 and tips[2] < stop - 0.010 and s.tweezers_tilt < 20.0
        d = np.asarray(s.pipette[0]) - np.asarray(self.lay.holder)  # the hook's collar on the cradle's prongs
        pip = -0.012 < d[0] < 0.015 and abs(d[1]) < 0.004 and abs(d[2]) < 0.002 and s.pipette_tilt < 10.0
        return bool(tw and pip)

    # -- the tier --------------------------------------------------------------------------------------------------
    def tier(self, s: Snapshot) -> tuple[float, str]:
        """task.json `scoring`: 1.0, 0.5, 0.25 or 0, and why."""
        fail = self.hard_fail(s)
        if fail:
            return 0.0, fail
        g = s.can_g
        zinc_off = max(self.frame_offset("zinc_1"), self.frame_offset("zinc_2"))
        zz = self.zinc_distance()
        lower_ok, why = self.stack_ok(s)
        if not lower_ok:
            return 0.0, why
        if not self.in_order(STACK[:4]):
            return 0.0, "zinc, separator, zinc, spacer not in that order"
        if not self.electrolyte_in_order():
            return 0.0, "electrolyte not dispensed between the separator and the second zinc"
        if not self.no_short(s):  # the meter: the separator between the zinc discs, the spacer off the wall
            return 0.0, "short: under R0"
        sp_off = self.frame_offset("spacer")
        if sp_off >= (CAN_ID - DIAM["spacer"]) / 2:  # the spacer on the wall: out of every tier
            return 0.0, f"spacer offset {sp_off * 1000:.2f} mm: on the wall"
        if "spring" in self.frame and self.frame_offset("spring") >= (CAN_ID - DIAM["spring"]) / 2:  # seen before the cap
            return 0.0, f"spring offset {self.frame_offset('spring') * 1000:.2f} mm: on the wall (a short once crimped)"
        quarter = zinc_off <= 0.002 and zz <= 0.002 and BAND_HALF[0] <= g <= BAND_HALF[1]
        top_ok, why_top = self.placed(s, ("spring",))
        complete = (top_ok and self.in_order(ORDER) and self.seated(s) and self.pressed
                    and sp_off < (CAN_ID - DIAM["spacer"]) / 2)
        if not complete:
            if quarter:
                return 0.25, "spring or cap missing, inverted, not seated or not pressed" + (f" ({why_top})" if why_top else "")
            return 0.0, "the upper parts are not right and the lower stack is out of the 0.25 tier"
        stowed = self.tools_stowed(s)
        clear = self.clear_since is not None and s.t - self.clear_since >= CLEAR_S - 1e-9
        full = zinc_off <= 0.001 and zz <= 0.001 and sp_off <= 0.0015 and BAND_FULL[0] <= g <= BAND_FULL[1] and stowed and clear
        if full:
            return 1.0, "all checks hold"
        half = zinc_off <= 0.002 and zz <= 0.002 and BAND_HALF[0] <= g <= BAND_HALF[1]
        if half:
            misses = []
            if zinc_off > 0.001:
                misses.append(f"zinc offset {zinc_off * 1000:.2f} mm")
            if zz > 0.001:
                misses.append(f"zinc-to-zinc {zz * 1000:.2f} mm")
            if sp_off > 0.0015:
                misses.append(f"spacer offset {sp_off * 1000:.2f} mm")
            if not BAND_FULL[0] <= g <= BAND_FULL[1]:
                misses.append(f"electrolyte {g:.3f} g")
            if not stowed:
                misses.append("a tool not in its stand")
            if not clear:
                misses.append("fingers not 3 cm clear of the tools and the cell for 1 s")
            return 0.5, "; ".join(misses)
        return 0.0, "out of every tier"

    def hard_fail(self, s: Snapshot) -> str:
        if self.left_table:
            return f"{self.left_table[0]} left the table"
        if self.gripper_moved:
            t, side, name = self.gripper_moved[0]
            return f"the {side} gripper moved the {name} at {t} s"
        if self.left_can:
            return f"{self.left_can[0]} left the can after it was laid in"
        if self.blot_wet:
            return "electrolyte on a gripper"
        if s.spilled_g > 0:
            return "electrolyte dispensed outside the separator"
        if s.can_g < BAND_HALF[0] or s.can_g > BAND_HALF[1]:
            return f"electrolyte {s.can_g:.3f} g, out of 0.050-0.150 g"
        for n in ("zinc_1", "zinc_2"):
            if n in self.frame and self.frame_offset(n) > 0.002:
                return f"{n} offset over 2.0 mm"
        if "zinc_1" in self.frame and "zinc_2" in self.frame and self.zinc_distance() > 0.002:
            return "zinc-to-zinc centre distance over 2.0 mm"
        return ""

    def frame_offset(self, name: str) -> float:
        if name not in self.frame:
            return math.inf
        return float(np.linalg.norm(self.frame[name] - np.asarray(self.lay.axis)))

    def zinc_distance(self) -> float:
        if "zinc_1" not in self.frame or "zinc_2" not in self.frame:
            return math.inf
        return float(np.linalg.norm(self.frame["zinc_1"] - self.frame["zinc_2"]))

    def electrolyte_in_order(self) -> bool:
        if self.last is None or not self.last.dispense_times:
            return False
        a, b = self.arrived.get("separator"), self.arrived.get("zinc_2")
        return a is not None and b is not None and all(a < t < b for t in self.last.dispense_times)


def touching(pts: np.ndarray, p: Part, name: str) -> bool:
    """Fingertip points within FINGER_NEAR of the part's outline (a flat cylinder, or the cap's 2 mm body)."""
    if pts is None or pts.size == 0:
        return False
    h = THICK.get(name, CAP_H)
    local = (pts - p.pos) @ p.rot
    r = np.linalg.norm(local[:, :2], axis=1)
    return bool(np.any((r < DIAM[name] / 2 + FINGER_NEAR) & (local[:, 2] > -FINGER_NEAR) & (local[:, 2] < h + FINGER_NEAR)))


def _seg_dist(pts: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ab = b - a
    t = np.clip((pts - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(pts - (a + t[:, None] * ab), axis=1)


TWEEZER_R, PIPETTE_R = 0.008, 0.015  # m: the tools' half widths about their axes


def fingers_clear(s: Snapshot) -> bool:
    pts = np.vstack([p for p in (s.left, s.right) if p.size]) if (s.left.size or s.right.size) else np.zeros((0, 3))
    if pts.size == 0:
        return True
    tw = _seg_dist(pts, np.asarray(s.tweezers[0]), np.asarray(s.tweezers[1])).min() - TWEEZER_R
    pp = _seg_dist(pts, np.asarray(s.pipette[0]), np.asarray(s.pipette[1])).min() - PIPETTE_R
    return bool(tw >= CLEAR and pp >= CLEAR)
