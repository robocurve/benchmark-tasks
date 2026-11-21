"""Strum C, then A minor, on a concert ukulele, checking each chord's fretted string, on ourkit.

On the station table, across the robot's front: a Kala KA-C concert ukulele lying face up in a padded cradle screwed
to the table, its neck to the robot's left, the G string on the far side and the A string nearest the robot.

- Each string is a drawn line except for its moving parts. The two fret windows the chords use are each a 2 mm wide
  piece of string on a sprung slider: the A string between the 2nd and 3rd fret wires (C = 0003) and the G string
  between the 1st and 2nd (A minor = 2000), 0.5 N preload at rest, 2.0 N pressed the 1.5 mm onto its fret wires.
  Each string's 80 mm strummed segment is a collider on a sideways slider sprung at the real string's stiffness
  there (550-660 N/m, from the Aquila 103U tensions on the 376.2 mm scale), with a drop slider under it.
- Fretted: that window pressed at least 1.35 mm down, and no gripper touching the string past the window's
  bridge-side fret wire (that would stop the string on the next fret instead).
- A strike: a fingertip carries a strummed segment 1.5 mm sideways (0.82-0.99 N); the segment then drops under the
  tip and comes back up once both grippers are 3 mm clear of its line (the declared slip rule). A string is touched
  when a gripper hull reaches its line anywhere else, or its segment where it is now; a struck string rings if
  nothing touches it for 0.5 s after (the fretting tip on its own window is the fret, not a touch).
- A downstrum: G, C, E and A struck in that order within 0.3 s. A check pluck: one outer string struck alone (no
  other string struck within 0.3 s), ringing with its chord kept fretted for the 0.5 s after it.

Scored from the sequence: C strums, the C check pluck, A minor strums, its G and A check plucks, the timing, and the
final hold.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import ourkit as k  # noqa: E402
from ourkit import demo as D  # noqa: E402
from ourkit import primitives as P  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the full-score rung already needs the 2 s hold
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Play C on the ukulele: four even downstrums, then pluck its fretted string once to check it; then four "
               "of A minor and check its G and A strings the same way, and keep holding the A minor.")

# -- the instrument (Kala KA-C; m, measured along the strings from the nut) ------------------------------------------
SCALE_M = 0.3762  # 14.8125 in
FRET_1 = SCALE_M * (1.0 - 2.0 ** (-1 / 12.0))  # 21.1 mm
FRET_2 = SCALE_M * (1.0 - 2.0 ** (-2 / 12.0))  # 41.0 mm
FRET_3 = SCALE_M * (1.0 - 2.0 ** (-3 / 12.0))  # 59.9 mm
FRET_M = [SCALE_M * (1.0 - 2.0 ** (-n / 12.0)) for n in range(19)]  # every fret wire
NUT_W, NUT_SPREAD, SADDLE_SPREAD = 0.0365, 0.0280, 0.0400  # fingerboard at the nut; G-to-A across nut and saddle
JOIN_S = FRET_M[14]  # the neck meets the body at the 14th fret
BOARD_END_S = SCALE_M * (1.0 - 2.0 ** (-18 / 12.0))  # the fingerboard ends at the 18th fret
BODY_END_S = JOIN_S + 0.250  # the body is 250 mm long
HEAD_L = 0.150  # headstock beyond the nut
UPPER_W, LOWER_W, WAIST_S = 0.180, 0.230, JOIN_S + 0.100  # bout widths and where the waist is
BODY_DEPTH = 0.070  # 2.75 in

# -- placement (x forward from the arm bases' midpoint, y left, z up from the table top) -----------------------------
CX = 0.220  # the centre line
NUT_Y = 0.230  # the nut; the strings run toward -y
CRADLE_H = 0.012  # foam pads under the body
BODY_TOP = CRADLE_H + BODY_DEPTH  # 82 mm
STR_H = BODY_TOP + 0.010  # the strings' centre line, 92 mm above the table
STR_R = 0.0005  # string radius
PRESS_TRAVEL_M = 0.0015  # string centre to resting on the fret wires
FRET_TOP = STR_H - STR_R - PRESS_TRAVEL_M  # fret wire crowns
BOARD_TOP = FRET_TOP - 0.001  # frets stand 1 mm proud of the rosewood
NECK_UNDER = BOARD_TOP - 0.026  # fingerboard 6 mm on a 20 mm neck
UKE_KG = 30.0  # stands in for the strapped, screwed-down cradle

STRINGS = ("G", "C", "E", "A")  # far side to near side
NOTE = {"G": "G4", "C": "C4", "E": "E4", "A": "A4"}


def s_y(s: float) -> float:
    return NUT_Y - s


def spread(s: float) -> float:
    return NUT_SPREAD + (SADDLE_SPREAD - NUT_SPREAD) * s / SCALE_M


def string_x(name: str, s: float) -> float:
    i = STRINGS.index(name)
    return CX + spread(s) * (1.5 - i) / 3.0  # G at +1.5 spacings (far), A at -1.5 (near)


def board_w(s: float) -> float:
    return NUT_W + (spread(s) - NUT_SPREAD)


SPACING_FRET_M = (NUT_SPREAD + (SADDLE_SPREAD - NUT_SPREAD) * FRET_3 / SCALE_M) / 3.0  # 10.0 mm at the third fret
STRUM_S0, STRUM_ZONE_M = BOARD_END_S, 0.080  # the strum zone: 80 mm toward the bridge from the fingerboard's end
SPACING_HOLE_M = (NUT_SPREAD + (SADDLE_SPREAD - NUT_SPREAD) * (STRUM_S0 + STRUM_ZONE_M / 2) / SCALE_M) / 3.0  # 12.3 mm

# -- the strummed strings: Aquila 103U Super Nylgut concert set (high G), tension per string -------------------------
T_G_KG, T_C_KG, T_E_KG, T_A_KG = 4.37, 3.91, 4.41, 4.72
TENSION_KG = {"G": T_G_KG, "C": T_C_KG, "E": T_E_KG, "A": T_A_KG}
STRUM_MID_S = STRUM_S0 + STRUM_ZONE_M / 2  # the strum zone's middle: 283 mm from the nut
SADDLE_GAP_M = SCALE_M - STRUM_MID_S  # ... and 93 mm from the saddle
SLIP_M = 0.0015  # a fingertip carries a string 1.5 mm sideways before it slips off (nylon plucks: 0-4 mm)
SIDE_LIM_M = 0.011  # the segment's sideways travel (wide, so the tip never meets a stop between control steps)
DROP_M = 0.0085  # a slipped string drops this far below the strings' line until the fingertip has passed
PASS_RESTORE_M = 0.003  # ... and comes back once both grippers are this far from its resting line
DROP_RATE, DROP_HOLD = 2000.0, 0.002  # the drop slider's spring and its preload past either stop
PASS_G_N = T_G_KG * 9.81 * (1.0 / SADDLE_GAP_M + 1.0 / STRUM_MID_S) * SLIP_M  # 0.92 N
PASS_C_N = T_C_KG * 9.81 * (1.0 / SADDLE_GAP_M + 1.0 / STRUM_MID_S) * SLIP_M  # 0.82 N
PASS_E_N = T_E_KG * 9.81 * (1.0 / SADDLE_GAP_M + 1.0 / STRUM_MID_S) * SLIP_M  # 0.93 N
PASS_A_N = T_A_KG * 9.81 * (1.0 / SADDLE_GAP_M + 1.0 / STRUM_MID_S) * SLIP_M  # 0.99 N


def side_rate(name: str) -> float:
    """The string's sideways stiffness at the strum zone's middle (N/m): T (1/a + 1/(L - a)) for a string of
    tension T and length L pushed a from the saddle."""
    return TENSION_KG[name] * 9.81 * (1.0 / SADDLE_GAP_M + 1.0 / STRUM_MID_S)


# -- the two fret windows ---------------------------------------------------------------------------------------------
CHORDS = {"C": ("A", 2, 3), "Am": ("G", 1, 2)}  # chord: (string, fret wire on the nut side, on the bridge side)
SEG_W, SEG_T = 0.002, 0.001  # the moving piece of string: 2 mm wide (x), 1 mm thick
PRESS_F0, PRESS_N = 0.5, 2.0  # N at rest (preload) and pressed full travel
PRESS_RATE = (PRESS_N - PRESS_F0) / PRESS_TRAVEL_M  # 1000 N/m
FRETTED_FRAC = 0.9  # pressed at least 90 % of the travel (1.35 mm)

# -- the rules and the scorer -----------------------------------------------------------------------------------------
STRUM_S = 0.3  # a downstrum strikes all four within 0.3 s
RING_S = 0.5  # a struck string rings if untouched for 0.5 s after
PLUCK_ALONE_S = 0.3  # a check pluck: no other string struck within 0.3 s
TIP_MOVE_M = 0.002  # the fretting tip moves at most 2 mm from a block's first strum to its check plucks
EVEN_FRAC = 0.25  # strum intervals within 25 % of the block's mean
CHANGE_S = 2.5  # the end of the C check's 0.5 s ring to the first A minor strum
HOLD_S = 2.0  # A minor held still at the end ...
CLEAR_MM = 20.0  # ... with the strumming gripper at least 20 mm from every string
UKE_MOVE = 0.005  # the ukulele moved further than this, at any time: score 0
POINT_STEP = 0.002  # strings sampled every 2 mm

WOOD, DARK, BOARD, NICKEL, NYLON, FOAM, PLASTIC = (0.62, 0.38, 0.22), (0.10, 0.07, 0.05), (0.22, 0.14, 0.10), \
    (0.80, 0.80, 0.78), (0.95, 0.93, 0.85), (0.20, 0.20, 0.22), (0.15, 0.15, 0.15)


def _seg_box(chord: str):
    name, f0, f1 = CHORDS[chord]
    s0, s1 = FRET_M[f0] + 0.001, FRET_M[f1] - 0.001  # clear of both fret wires
    sm = (s0 + s1) / 2
    return k.box((0.0, 0.0, 0.0), (SEG_W, s1 - s0, SEG_T), NYLON, label=f"{name.lower()}_window"), \
        (string_x(name, sm), s_y(sm), STR_H)


def _body() -> list:
    s = k.span
    out = []
    # cradle: body rest and neck rest (foam on wood), fixed to the table
    out.append(s((CX - 0.10, s_y(BODY_END_S - 0.02), 0), (CX + 0.10, s_y(WAIST_S - 0.02), CRADLE_H), FOAM))
    neck_rest_s = (0.100, 0.160)
    out.append(s((CX - 0.03, s_y(neck_rest_s[1]), 0), (CX + 0.03, s_y(neck_rest_s[0]), NECK_UNDER), FOAM))
    # body: upper and lower bout as two blocks, top face at BODY_TOP
    out.append(s((CX - UPPER_W / 2, s_y(WAIST_S), CRADLE_H), (CX + UPPER_W / 2, s_y(JOIN_S), BODY_TOP), WOOD))
    out.append(s((CX - LOWER_W / 2, s_y(BODY_END_S), CRADLE_H), (CX + LOWER_W / 2, s_y(WAIST_S), BODY_TOP), WOOD))
    # soundhole (drawn), bridge and saddle
    hole_s, hole_r = STRUM_S0 + 0.030, 0.026
    out.append(s((CX - hole_r, s_y(hole_s + hole_r), BODY_TOP), (CX + hole_r, s_y(hole_s - hole_r), BODY_TOP + 0.0005),
                 DARK, ghost=True))
    out.append(s((CX - 0.040, s_y(SCALE_M + 0.012), BODY_TOP), (CX + 0.040, s_y(SCALE_M - 0.006), BODY_TOP + 0.006),
                 DARK))
    out.append(s((CX - 0.030, s_y(SCALE_M + 0.001), BODY_TOP), (CX + 0.030, s_y(SCALE_M - 0.001), STR_H - STR_R),
                 NYLON))
    # neck and fingerboard in four tapered steps, nut, frets
    steps = np.linspace(0.0, BOARD_END_S, 5)
    for a, b in zip(steps[:-1], steps[1:]):
        w = board_w((a + b) / 2)
        if a < JOIN_S:
            out.append(s((CX - w / 2, s_y(min(b, JOIN_S)), NECK_UNDER), (CX + w / 2, s_y(a), BOARD_TOP - 0.006), WOOD))
        out.append(s((CX - w / 2, s_y(b), BOARD_TOP - 0.006), (CX + w / 2, s_y(a), BOARD_TOP), BOARD))
    out.append(s((CX - NUT_W / 2, s_y(0.0), BOARD_TOP - 0.006), (CX + NUT_W / 2, s_y(-0.005), STR_H - STR_R), NYLON))
    for n in range(1, 19):
        w = board_w(FRET_M[n])
        out.append(s((CX - w / 2, s_y(FRET_M[n]) - 0.001, BOARD_TOP), (CX + w / 2, s_y(FRET_M[n]) + 0.001, FRET_TOP),
                     NICKEL))
    # headstock with a clip-on tuner
    out.append(s((CX - 0.030, s_y(-0.005), NECK_UNDER + 0.004), (CX + 0.030, s_y(-0.005 - HEAD_L), NECK_UNDER + 0.018),
                 WOOD))
    out.append(s((CX - 0.020, s_y(-0.070), NECK_UNDER + 0.018), (CX + 0.020, s_y(-0.100), NECK_UNDER + 0.038),
                 PLASTIC))
    # the strings, drawn: nut to saddle, skipping the two moving windows
    for name in STRINGS:
        cuts = [(FRET_M[f0] + 0.001, FRET_M[f1] - 0.001) for st, f0, f1 in CHORDS.values() if st == name]
        cuts.append((STRUM_S0, STRUM_S0 + STRUM_ZONE_M))  # the strummed segment is a moving part
        edges = [0.0] + [c for cut in cuts for c in cut] + [SCALE_M]
        for a, b in zip(edges[::2], edges[1::2]):
            n = max(2, int(math.ceil((b - a) / 0.03)))
            for i in range(n):
                sa, sb = a + (b - a) * i / n, a + (b - a) * (i + 1) / n
                xa, xb = string_x(name, sa), string_x(name, sb)
                out.append(k.box(((xa + xb) / 2, s_y((sa + sb) / 2), STR_H), (0.001, sb - sa, 0.001), NYLON,
                                 ghost=True))
    return out


_A_BOX, _A_AT = _seg_box("C")
_G_BOX, _G_AT = _seg_box("Am")
A_WIN = k.slider("a_window", _A_AT, "Z", -PRESS_TRAVEL_M, 0.0, [_A_BOX], rate=PRESS_RATE,
                 target=PRESS_F0 / PRESS_RATE, damping=0.5, mass=0.005, weightless=True)
G_WIN = k.slider("g_window", _G_AT, "Z", -PRESS_TRAVEL_M, 0.0, [_G_BOX], rate=PRESS_RATE,
                 target=PRESS_F0 / PRESS_RATE, damping=0.5, mass=0.005, weightless=True)
def _zone_joints(name: str) -> list:
    """The string's strummed segment: a sideways spring at the real string's stiffness, carrying a drop slider."""
    x0, x1 = string_x(name, STRUM_S0), string_x(name, STRUM_S0 + STRUM_ZONE_M)
    at = ((x0 + x1) / 2, s_y(STRUM_MID_S), STR_H)
    yaw = math.atan2(x1 - x0, STRUM_ZONE_M)  # about 1 degree: the strings fan out toward the saddle
    seg = k.box((0.0, 0.0, 0.0), (0.001, STRUM_ZONE_M, 0.001), NYLON, turn=(0.0, 0.0, yaw),
                label=f"{name.lower()}_strummed")
    carrier = k.box((0.0, 0.0, -0.012), (0.002, 0.004, 0.002), NYLON, hidden=True)  # inside the body: never reached
    side = k.slider(f"{name.lower()}_side", at, "X", -SIDE_LIM_M, SIDE_LIM_M, [carrier], rate=side_rate(name), target=0.0,
                    damping=0.3, mass=0.002, weightless=True)
    drop = k.slider(f"{name.lower()}_drop", at, "Z", -DROP_M, 0.0, [seg], rate=DROP_RATE, target=DROP_HOLD,
                    damping=2.0, mass=0.002, weightless=True, parent=f"{name.lower()}_side")
    return [side, drop]


ZONE_JOINTS = [j for n in STRINGS for j in _zone_joints(n)]
UKE = k.mech("ukulele", (0.0, 0.0, TOP + 0.0005), _body(), UKE_KG, joints=[A_WIN, G_WIN] + ZONE_JOINTS, color=WOOD,
             surface=0.6)
OBJECTS = [UKE]
WIN_JOINT = {"A": "a_window", "G": "g_window"}


def string_points(name: str, s0: float = 0.0, s1: float = SCALE_M) -> np.ndarray:
    """World points along a string's centre line between s0 and s1 (from the nut), every 2 mm."""
    ss = np.arange(s0, s1 + 1e-9, POINT_STEP)
    return np.array([(string_x(name, s), s_y(s), TOP + 0.0005 + STR_H) for s in ss])


ZONE = {n: string_points(n, STRUM_S0, STRUM_S0 + STRUM_ZONE_M) for n in STRINGS}
WHOLE = {n: string_points(n) for n in STRINGS}
OUTSIDE = {n: np.vstack([string_points(n, 0.0, STRUM_S0 - 0.001), string_points(n, STRUM_S0 + STRUM_ZONE_M + 0.001)])
           for n in STRINGS}  # the string apart from its strummed segment
BEYOND = {c: string_points(st, FRET_M[f1] + 0.001) for c, (st, f0, f1) in CHORDS.items()}
NUT_SIDE = {c: string_points(st, 0.0, FRET_M[f1] - 0.001) for c, (st, f0, f1) in CHORDS.items()}
ALL_POINTS = np.vstack(list(WHOLE.values()))


# -- the rules: touches, strikes, strums, plucks --------------------------------------------------------------------
def _touching(points, hulls) -> bool:
    return bool(hulls) and k.score.clearance(points, [hulls], beyond=0.01) <= STR_R


class Ledger:
    """Everything the scorer reads, rebuilt every step from the grippers' hulls and the windows' travel."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.t = 0.0
        self.dropped = {n: False for n in STRINGS}  # slipped off the fingertip, dropped until the tip has passed
        self.touch = {n: False for n in STRINGS}  # touched now (by anything that isn't the fret)
        self.r_touch = {n: False for n in STRINGS}
        self.last_touch_end = {n: -math.inf for n in STRINGS}
        self.strikes: list[tuple[float, str]] = []  # (time, string): right-hand touch starts in the strum zone
        self.pending: list[dict] = []  # strikes waiting for their 0.5 s ring verdict
        self.rung: dict[tuple[float, str], bool] = {}
        self.chord_at: dict[float, str | None] = {}
        self.held: dict[float, bool] = {}  # the chord at a strike stayed fretted for RING_S after it
        self.hold_pending: list[float] = []
        self.tip_at: dict[float, np.ndarray] = {}
        self.tip = None
        self.events: list[str] = []

    # what the left hand frets now
    def fretted(self, states, left) -> str | None:
        u = states["ukulele"]
        out = None
        for chord, (st, f0, f1) in CHORDS.items():
            pressed = -float(u.q[WIN_JOINT[st]]) >= FRETTED_FRAC * PRESS_TRAVEL_M
            if pressed and not _touching(BEYOND[chord], left):
                others = [n for n in STRINGS if n != st]
                if not any(_touching(WHOLE[n], left) for n in others):
                    out = chord if out is None else "both"
        return None if out == "both" else out

    def step(self, states, grips, dt):
        self.t += dt
        if not grips:
            return
        left, right = grips
        chord = self.fretted(states, left)
        tips = [np.asarray(h.vertices).mean(axis=0) for name, h in left if "tip" in name and hasattr(h, "vertices")]
        self.tip = np.mean(tips, axis=0) if tips else None
        u = states["ukulele"]
        for n in STRINGS:
            j = n.lower()
            dx, dz = float(u.q[f"{j}_side"]), float(u.q[f"{j}_drop"])
            here = ZONE[n] + np.array([dx, 0.0, dz])  # the strummed segment where it is now
            outside = OUTSIDE[n]
            fret_ok = chord is not None and CHORDS[chord][0] == n
            l_now = _touching(outside, left) and not (fret_ok and not _touching(BEYOND[chord], left))
            l_now = l_now or _touching(here, left)
            r_now = _touching(outside, right) or _touching(here, right)
            # a strike: a fingertip has carried the string SLIP_M sideways (at least the real string's force); it slips
            # off, so the segment drops below the fingertip until both grippers are clear of its resting line
            if not self.dropped[n] and abs(dx) >= SLIP_M:
                self.dropped[n] = True
                k.set_targets("ukulele", {f"{j}_drop": -DROP_M - DROP_HOLD})
                self.strikes.append((self.t, n))
                self.pending.append({"t": self.t, "n": n})
                self.chord_at[self.t] = chord
                if self.t not in self.held:
                    self.hold_pending.append(self.t)
                if self.tip is not None:
                    self.tip_at[self.t] = self.tip.copy()
                r_now = l_now = False  # it has slipped off: the touch that struck it ends here
                self.last_touch_end[n] = self.t
            elif self.dropped[n] and min(k.score.clearance(ZONE[n], [h], beyond=0.01) for h in grips if h) \
                    >= PASS_RESTORE_M:
                self.dropped[n] = False
                k.set_targets("ukulele", {f"{j}_drop": DROP_HOLD})
            now = r_now or l_now
            if self.touch[n] and not now:
                self.last_touch_end[n] = self.t
            self.touch[n], self.r_touch[n] = now, r_now
        # ring verdicts: untouched for RING_S after the strike's touch ended
        keep = []
        for p in self.pending:
            n = p["n"]
            if self.touch[n] and self.t - p["t"] > dt * 1.5 and self.last_touch_end[n] >= p["t"]:
                self.rung[(p["t"], n)] = False  # touched again before ringing out
            elif not self.touch[n] and self.last_touch_end[n] >= p["t"] and self.t - self.last_touch_end[n] >= RING_S:
                self.rung[(p["t"], n)] = True
            else:
                keep.append(p)
                continue
        self.pending = keep
        # held verdicts: the strike's chord still fretted (same chord) through its ring window
        still = []
        for t0 in self.hold_pending:
            if chord != self.chord_at[t0]:
                self.held[t0] = False
            elif self.t - t0 >= RING_S:
                self.held[t0] = True
            else:
                still.append(t0)
        self.hold_pending = still

    # -- reading the record ----------------------------------------------------------------------------------------
    def strums(self):
        """Downstrums: G, C, E, A struck in order within STRUM_S. Returns [(t_first, t_last, [(t, n)...])]."""
        out, i, ss = [], 0, self.strikes
        while i < len(ss):
            if ss[i][1] == "G":
                group, j = [ss[i]], i + 1
                while j < len(ss) and len(group) < 4 and ss[j][0] - ss[i][0] <= STRUM_S:
                    if ss[j][1] == STRINGS[len(group)]:
                        group.append(ss[j])
                    j += 1
                if len(group) == 4:
                    out.append((group[0][0], group[-1][0], group))
                    i = j
                    continue
            i += 1
        return out

    def plucks(self):
        """Check plucks: an outer string struck with no other string struck within PLUCK_ALONE_S."""
        out = []
        for t, n in self.strikes:
            if n in ("G", "A") and not any(n2 != n and abs(t2 - t) <= PLUCK_ALONE_S for t2, n2 in self.strikes):
                out.append((t, n))
        return out


LEDGER = Ledger()
UKE_GONE = k.Sticky()
RUN = {"uke0": None, "end_ok": []}
REC = k.run_record.RunRecord(__file__, "ukulele")


def _strum_ok(strum, chord) -> bool:
    """Every strike of a strum made with ``chord`` fretted, and the chord's open strings rang."""
    t0, t1, group = strum
    fret_string = CHORDS[chord][0]
    for t, n in group:
        if LEDGER.chord_at.get(t) != chord:
            return False
        if n != fret_string and LEDGER.rung.get((t, n)) is not True:
            return False
    return True


def _even(ts) -> bool:
    if len(ts) != 4:
        return False
    gaps = np.diff(ts)
    return bool(np.all(np.abs(gaps - gaps.mean()) <= EVEN_FRAC * gaps.mean() + 1e-9))


def _tip_still(t_ref, t) -> bool:
    a, b = LEDGER.tip_at.get(t_ref), LEDGER.tip_at.get(t)
    return a is not None and b is not None and float(np.linalg.norm(a - b)) <= TIP_MOVE_M + 1e-9


def _check_ok(t, n, chord, t_ref) -> bool:
    """A check pluck: struck with ``chord`` fretted, the chord kept fretted and the string ringing for RING_S after,
    the fretting tip within TIP_MOVE_M of where it was at the block's first strum."""
    return (LEDGER.chord_at.get(t) == chord and LEDGER.held.get(t) is True and LEDGER.rung.get((t, n)) is True
            and _tip_still(t_ref, t))


def analyse():
    """Walk the record: the C block, its check, the A minor block, its two checks. Returns a dict of milestones."""
    strums, plucks = LEDGER.strums(), LEDGER.plucks()
    out = {"c4": False, "c_check": False, "am": False, "timing": False}
    c_strums = [st for st in strums if _strum_ok(st, "C")]
    if len(c_strums) < 4:
        return out
    out["c4"] = True
    c_first, c_fourth = c_strums[0][0], c_strums[3][1]
    c_check = next((t for t, n in plucks if n == "A" and t > c_fourth and _check_ok(t, n, "C", c_first)), None)
    if c_check is None:
        return out
    out["c_check"] = True
    am_strums = [st for st in strums if st[0] > c_check and _strum_ok(st, "Am")]
    if len(am_strums) < 4:
        return out
    a_first, a_fourth = am_strums[0][0], am_strums[3][1]
    g_check = next((t for t, n in plucks if n == "G" and t > a_fourth and _check_ok(t, n, "Am", a_first)), None)
    if g_check is None:
        return out
    a_check = next((t for t, n in plucks if n == "A" and t > g_check and _check_ok(t, n, "Am", a_first)), None)
    if a_check is None:
        return out
    out["am"] = True
    c_all = [st[0] for st in strums if c_first <= st[0] <= c_check]
    a_all = [st[0] for st in strums if c_check < st[0] <= g_check]
    out["timing"] = _even(c_all) and _even(a_all) and a_first - (c_check + RING_S) <= CHANGE_S + 1e-9
    return out


def grade(states) -> float:
    if UKE_GONE:
        return 0.0
    m = analyse()
    end = len(RUN["end_ok"]) >= int(round(HOLD_S * k.score.HZ)) and all(RUN["end_ok"][-int(round(HOLD_S * k.score.HZ)):])
    return k.ladder((0.25, m["c4"]), (0.5, m["c_check"]), (0.75, m["am"]), (1.0, m["timing"] and end))


def score(states) -> float:
    return grade(states)


def success(states) -> bool:
    return grade(states) >= 1.0


def distance(states) -> float:
    return 1.0 - grade(states)


def diag(states, grips) -> str:
    """One trace line of evidence: what the fretting hand touches, the last strikes with their chord and ring verdicts."""
    if not grips:
        return "no grippers"
    left = grips[0]
    lt = "".join(n for n in STRINGS if _touching(WHOLE[n], left))
    bt = "".join(c for c in CHORDS if _touching(BEYOND[c], left))
    tip = "-" if LEDGER.tip is None else ",".join(f"{v * 1000:.0f}" for v in LEDGER.tip)
    last = [(round(t, 2), n, LEDGER.chord_at.get(t), LEDGER.rung.get((t, n))) for t, n in LEDGER.strikes[-5:]]
    u = states["ukulele"]
    side = " ".join(f"{n}{u.q[n.lower() + '_side'] * 1000:+.1f}{'v' if LEDGER.dropped[n] else ''}" for n in STRINGS)
    return (f"chord {LEDGER.fretted(states, left)} left-touch {lt or '-'} beyond {bt or '-'} tip {tip} side {side} "
            f"last {last}")


@k.on_reset
def _reset(states) -> None:
    LEDGER.reset(), UKE_GONE.reset()
    RUN.update(uke0=np.asarray(states["ukulele"].pos, dtype=float), end_ok=[], tip_end=None)
    REC.reset()


@k.on_step
def _watch(states) -> None:
    u = states["ukulele"]
    if np.linalg.norm(np.asarray(u.pos) - RUN["uke0"]) > UKE_MOVE:
        UKE_GONE.trip("ukulele moved")
    grips = k.score.grippers()
    LEDGER.step(states, grips, 1.0 / k.score.HZ)
    ok = False
    if grips:
        held = LEDGER.fretted(states, grips[0]) == "Am"
        clear = k.score.clearance(ALL_POINTS, [grips[1]], beyond=CLEAR_MM / 1000.0) >= CLEAR_MM / 1000.0 - 1e-9
        tip = LEDGER.tip
        still = tip is not None and RUN.get("tip_end") is not None and \
            float(np.linalg.norm(tip - RUN["tip_end"])) <= TIP_MOVE_M
        RUN["tip_end"] = tip if not still or RUN.get("tip_end") is None else RUN["tip_end"]
        ok = held and clear and still
    RUN["end_ok"].append(ok)
    REC.tick(states, lambda: f"A {-u.q['a_window'] * 1000:.2f} mm, G {-u.q['g_window'] * 1000:.2f} mm, "
                             f"strikes {len(LEDGER.strikes)}, strums {len(LEDGER.strums())}, {analyse()}, "
                             f"score {grade(states):g} | {diag(states, grips)}")


# -- the demo ---------------------------------------------------------------------------------------------------------
DOWN = D.Grip.deg(yaw=0.0)  # straight down, jaws closing along y: the closed tips are 27 mm along the string, 15 across
PAD_HALF_X, PAD_HALF_Y = 0.0075, 0.0135
OVERHANG = 0.0045  # the pad's centre this far outside its string: inner edge 3 mm past the string, 7 mm short of the next
PRESS_FORCE = 3.0  # N commanded: the window's 2 N plus margin
LIFT = 0.012  # m the fretting tip rises to change chords
STRUM_DEPTH = 0.004  # strumming tip bottom below the strings' centre line
STRUM_HIGH = 0.020  # the return pass, above the strings
STRUM_OUT = PAD_HALF_X + 0.006  # a strum starts and ends with the tip's edge 6 mm outside the outer strings
PLUCK_PAST = 0.005  # a check pluck's leading edge goes 5 mm past its string, 7 mm short of the next
SETTLE_T = 0.5  # the arm lags ~20 mm in a fast move: let it reach string depth before the first strum
STRUM_T, GAP_T = 0.40, 1.00  # one downstrum's sweep, and the rest of the beat (1.4 s a strum)
Z0 = TOP + 0.0005


def fret_contact(chord: str) -> tuple[float, float, float]:
    """Where the fretting tips' centre meets the window: the pad overhangs the fingerboard's outer edge so its inner
    edge stays clear of the open neighbour, and its bridge-side edge sits 4 mm behind the fret wire."""
    name, f0, f1 = CHORDS[chord]
    s_c = FRET_M[f1] - 0.004 - PAD_HALF_Y
    side = 1.0 if name == "G" else -1.0  # G on the far (+x) side, A on the near side
    x = string_x(name, FRET_M[f1] - 0.010) + side * OVERHANG
    return (x, s_y(s_c), Z0 + STR_H + STR_R)


def _tcp(contact, overshoot=0.0, up=0.0):
    a = P.approach_axis(DOWN)
    c = np.asarray(contact, dtype=float)
    return tuple(c - P.PAD_TIP * a + np.array([0.0, 0.0, up - overshoot]))


def left_hand(how: str) -> D.Plan:
    o = P.overshoot_for(PRESS_FORCE, True)
    p = D.Plan("left")
    c = fret_contact("C")
    if how == "flat":  # a flat pad centred across the A and E strings
        c = (string_x("A", FRET_M[3]) + 0.005, c[1], c[2])
    p.move(2.5, _tcp(c, up=0.040), finger=D.FINGER_CLOSED, grip=DOWN, label="over the C shape")
    p.move(1.5, _tcp(c, up=0.004), label="down to the A string")
    if how == "open":  # strum without fretting anything
        p.hold(16.0, "hover")
        return p
    p.move(1.0, _tcp(c, overshoot=o), label="press C")
    p.hold(C_BLOCK_S, "hold C through the strums and the check")
    if how == "stay_c":
        return p.hold(14.0, "keep C")
    a = fret_contact("Am")
    p.move(0.3, _tcp(c, up=LIFT), label="lift off")
    p.move(0.6, _tcp(a, up=LIFT), label="over the G string")
    p.move(0.7, _tcp(a, overshoot=o), label="press A minor")
    return p.hold(16.0, "hold A minor to the end")


def _strum(p: D.Plan) -> D.Plan:
    sm = STRUM_S0 + 0.035
    far = string_x("G", sm) + STRUM_OUT
    near = string_x("A", sm) - STRUM_OUT
    y = s_y(sm)
    zl, zh = Z0 + STR_H - STRUM_DEPTH - PAD_HALF_X * 0, Z0 + STR_H + STRUM_HIGH
    p.move(STRUM_T, _tcp((near, y, zl)), label="downstrum")
    p.move(0.15, _tcp((near, y, zh)), label="up off the strings")
    p.move(0.30, _tcp((far, y, zh)), label="back over the far side")
    p.move(GAP_T - 0.65, _tcp((far, y, zl)), label="down beyond the G string")
    return p.hold(0.20, "settle at string depth")


def _pluck(p: D.Plan, name: str) -> D.Plan:
    sm = STRUM_S0 + 0.035
    y = s_y(sm)
    side = 1.0 if name == "G" else -1.0
    xs = string_x(name, sm)
    outside, inside = xs + side * 0.022, xs - side * (PLUCK_PAST - PAD_HALF_X)  # leading edge 5 mm past it
    zl, zh = Z0 + STR_H - STRUM_DEPTH, Z0 + STR_H + STRUM_HIGH
    here = np.asarray(p.last.pos, dtype=float)
    p.move(0.3, (here[0], here[1], here[2] + (zh - zl)), label="straight up off the strings")
    p.move(0.5, _tcp((outside, y, zh)), label=f"over the {name} string's outside")
    p.move(0.4, _tcp((outside, y, zl)), label="down beside it")
    p.move(0.3, _tcp((inside, y, zl)), label=f"pluck the {name} string")
    p.move(0.3, _tcp((inside, y, zh)), label="up off the strings")
    return p.move(0.3, _tcp((outside, y, zh)), label="back out over it")


PLUCK_T = 2.1  # one check pluck, start to end (the strike is about 1.4 s in)
WAIT_AM = 0.6  # the strumming hand waits for the A minor press after the C check
C_BLOCK_S = 4 * (STRUM_T + GAP_T) + PLUCK_T + 0.4 + SETTLE_T + 0.1  # from the press: descent, four strums, the whole pluck (its ring)


def right_hand(how: str) -> D.Plan:
    p = D.Plan("right")
    sm = STRUM_S0 + 0.035
    far = (string_x("G", sm) + STRUM_OUT, s_y(sm), Z0 + STR_H + STRUM_HIGH)
    p.move(3.0, _tcp(far), finger=D.FINGER_CLOSED, grip=DOWN, label="over the strum zone, far side")
    p.hold(2.0, "wait for the C shape")
    p.move(0.4, _tcp((far[0], far[1], Z0 + STR_H - STRUM_DEPTH)), label="down beyond the G string")
    p.hold(SETTLE_T, "settle at string depth")
    n = 5 if how == "five" else 4
    for _ in range(n):
        _strum(p)
    if how != "no_check":
        _pluck(p, "A")
    if how == "stay_c":
        for _ in range(4):
            _strum(p)
        return p.home()
    p.hold(WAIT_AM, "wait for the A minor press")
    p.move(0.6, _tcp(far), label="back over the far side")
    p.move(0.4, _tcp((far[0], far[1], Z0 + STR_H - STRUM_DEPTH)), label="down beyond the G string")
    p.hold(SETTLE_T, "settle at string depth")
    for _ in range(4):
        _strum(p)
    _pluck(p, "G")
    _pluck(p, "A")
    return p.home()


def demo():
    def two(left_how, right_how):
        return D.together(left_hand(left_how), right_hand(right_how))

    return D.choose(demo=lambda: two("demo", "demo"), none=D.idle,
                    slam=lambda: two("flat", "demo"), wrong_order=lambda: two("open", "demo"),
                    no_check=lambda: two("demo", "no_check"), stay_c=lambda: two("stay_c", "stay_c"),
                    five=lambda: two("demo", "five"))


# -- what the template tests read ---------------------------------------------------------------------------------------
TEXT_NUMBERS = [("608 mm", 0.608), ("376.2 mm", SCALE_M), ("36.5 mm", NUT_W), ("21.1 mm", FRET_1),
                ("41.0 mm", FRET_2), ("59.9 mm", FRET_3), ("10.0 mm", SPACING_FRET_M), ("12.3 mm", SPACING_HOLE_M), ("9.2 cm", STR_H), ("22 cm", CX), ("1.5 mm", PRESS_TRAVEL_M),
                ("2 N", PRESS_N), ("80 mm", STRUM_ZONE_M), ("0.3 s", STRUM_S), ("0.5 s", RING_S),
                ("2 mm", TIP_MOVE_M), ("25%", EVEN_FRAC), ("2.5 s", CHANGE_S), ("last 2 s", HOLD_S),
                ("20 mm", CLEAR_MM / 1000)]
CHEATS = {"none": 0.0, "slam": 0.0, "wrong_order": 0.0, "no_check": 0.25, "stay_c": 0.5, "five": 0.75}
HOLD = {"ukulele.a_window": 0.2, "ukulele.g_window": 0.2}  # joint: the most it may move (mm or deg)
