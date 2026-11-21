#!/usr/bin/env python3
"""The calculator cover's scripted full-score run, from the start (keys down in the cover, the open display end
past the notebook's edge):

    scripts/rta demo <id> mech/tasks/calc_cover [--quick] [--norec]

1. The left hand takes the cover from above by its walls near the closed end (a deep pinch, leaning back 35 deg);
   the right hand takes the calculator from above by its bare strip over the cover's walls near the open end (a
   shallow pinch) and draws it 100 mm out along the rails, the left holding the cover back; still on the rails, the
   right takes it again over its middle (out of the cover: its sides bare, a pinch over their whole height) and draws
   it out the rest of the way. A pinch across the width holds a lever only by the pads' twist (the calculator swung
   35 deg held by its end, run w12c14): the calculator is carried in the air only by a pinch over its middle.
2. The right hand lifts the calculator; in the air the left hand takes its ON-key end, level, across its thickness;
   the right lets go and moves up; the left rolls it half a turn, keys up; the right takes it back over its middle;
   the left lets go.
3. The left hand takes the cover by its walls near the closed end again; the right brings the calculator, ON-key end
   first, in line with the cover's back rails (its pose read from the simulator, twice) and pushes it 30 mm in; on the
   rails, the right takes it again by the strip near its display end and pushes it home through the click, the left
   holding the cover back. Both let go and go back to READY, held 2 s.
Each stage is planned when the one before ends, from the parts' poses as the simulator has them (a scripted demo may
read the simulator's state); --quick runs the plan through the IK from the parts' built poses, without the simulator.
"""

from __future__ import annotations

import json
import math
import os
import sys

if "--quick" in sys.argv:  # the dry run's hull distances are many small products: one thread does them fastest
    for _var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ.setdefault(_var, "1")
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from clearance import hand_under, plan_apart  # noqa: E402

OPEN = 0.0939  # m between the pads, fully open
TIP = 0.0128  # m: the fingertips' end past the grasp point
OVER_WALLS = 0.0025  # m the fingertips stay over the cover's walls in the shallow pinch (the pads hold 5.6 mm of the 8.9 mm strip)
OVER_BOOK = 0.003  # m the fingertips stay over the notebook in the deep pinch
DRAW_PAST = 0.012  # m the right hand draws past the rails' length (the cover lets go 1 mm past it; the arm lags 5 mm against the rails: w12c16)
GRASP_IN = 0.020  # m from the calculator's (or the cover's) end to the grasp point
FLIP_AT = np.array([0.28, -0.140, 0.140])  # m: the left hand's grasp point at the flip, over the bench (dry checks: margins 21-30 deg)
NEAR = 0.003  # m out of the cover's open end where the calculator waits, in line, before it goes in
SQUEEZE = 0.004  # m the grip is closed past the part's width (the pads press)
IN_FIRST = 0.030  # m the calculator goes into the rails held over its middle, before the right hand takes it by its end
DRAW_FIRST = 0.100  # m drawn out by the strip near the display end, before the right hand takes the calculator over its middle
FLIP_AWAY = np.array([0.0, -0.010, 0.065])  # m the right hand moves from the calculator's middle before the left rolls it
UNDER = 0.003  # m the fingertips reach under the calculator in the pinch over its middle (its sides bare, out of the cover)
PAST_HOME = 0.002  # m the right hand's last target lies past home: the end wall stops the calculator, the arm presses


def grip(width: float) -> float:
    return min(1.0, max(0.0, width / OPEN))


def down(yaw: float) -> np.ndarray:
    from yam_arena.demo import grip_rotation

    return grip_rotation(yaw)


def level(approach, closing) -> np.ndarray:
    a, c = np.asarray(approach, float), np.asarray(closing, float)
    return np.column_stack([c, np.cross(a, c), a])


def roll(rot: np.ndarray, angle: float) -> np.ndarray:
    a = rot[:, 2]
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return (np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k) @ rot


R_DOWN = down(math.pi / 2)  # the right hand from above, closing along x
L_DOWN = down(-math.pi / 2)  # the left hand from above, closing along x (+90 deg ends at its wrist's limit)
L_LEVEL = level((0.0, -1.0, 0.0), (0.0, 0.0, 1.0))  # the left hand level, pointing -y, closing up-down
L_LEAN = math.radians(35.0)  # the left hand on the cover's walls (closed end, mouth) leans back towards its own base
# Straight down at the closed end, 117 mm from the right hand's grasp, the two wrists (link 4) overlapped by 37 mm
# (run w12c10: they collided); leaning back 35 deg they stay 50 mm apart (clearance.py, checked over the whole plan).
L_HOLD = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(-L_LEAN), -math.sin(-L_LEAN)], [0.0, math.sin(-L_LEAN), math.cos(-L_LEAN)]]) @ L_DOWN


class Stop(Exception):
    """The plan cannot go on as planned (a grip or a part is not where it must be): both arms go back to READY."""


class Plan:
    """The stages, each planned from the parts' poses when it starts (the simulator's, or as built in a dry run)."""

    def __init__(self, m, live: bool) -> None:
        from demo_kit import Move

        self.m, self.live, self.Move = m, live, Move
        self.mk = m.mk
        self.stages = [self.take, self.pinch_strip, self.draw_part, self.regrip_middle, self.pinch_middle, self.draw_rest, self.lift, self.hand_over, self.flip,
                       self.retake, self.pinch_middle, self.release, self.to_cover, self.align, self.align, self.enter, self.regrip_end, self.pinch_strip,
                       self.home, self.away]  # fmt: skip
        self.i = 0
        self.log: list = []

    # --- the parts' poses ---
    def pose(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        if self.live:
            p, r = self.mk.pose(name)
            return np.array(p, dtype=float), np.array(r, dtype=float)
        return self.dry[name]

    def calc_frame_point(self, local) -> np.ndarray:
        p, r = self.pose("calc")
        return p + r @ np.asarray(local, dtype=float)

    def start_dry(self) -> None:
        """Where the parts are when each stage begins, in a dry run: as built, moved as the plan moves them."""
        m = self.m
        self.dry = {name: (np.array(m.mk.MECH.bodies[name].pos, dtype=float), m.mk.MECH.bodies[name].rotation()) for name in ("calc", "cover")}

    # --- the stages: each returns {side: [Move, ...]} ---
    def take(self) -> dict:
        m, Move = self.m, self.Move
        vp, vr = self.pose("cover")
        closed_end = vp + vr @ np.array([0.0, m.COVER_Y0 + GRASP_IN, 0.0])  # the cover's closed end (its own -y)
        l_at = np.array([closed_end[0], closed_end[1], m.BOOK_TOP + OVER_BOOK + hand_under(L_HOLD)])
        r_at = self.strip_point()
        return {
            "left": [Move(2.0, l_at + [0, 0, 0.05], L_HOLD, 1.0, "L over cover"), Move(1.2, l_at, None, 1.0, "L down"), Move(0.6, None, None, grip(m.COVER_W - SQUEEZE), "L grips cover")],
            "right": [Move(2.0, r_at + [0, 0, 0.05], R_DOWN, 1.0, "R over calc"), Move(1.2, r_at, None, 1.0, "R down"), Move(0.6, None, None, None, "R settles")],
        }  # fmt: skip

    def strip_point(self) -> np.ndarray:
        """The right hand's grasp point for the shallow pinch: over the calculator's middle line, GRASP_IN from its display
        end, the fingertips OVER_WALLS over the walls' top (the lips' far face, LIP_T / 2 over the calculator's
        mid-plane), whichever side is up."""
        m = self.m
        r_at = self.calc_frame_point([0.0, m.CALC[1] / 2 - GRASP_IN, 0.0])  # on its mid-plane there (it may hang tilted)
        r_at[2] += m.LIP_T / 2 + OVER_WALLS + TIP
        return r_at

    def middle_point(self) -> np.ndarray:
        """The right hand's grasp point for the pinch over the calculator's middle, out of the cover: its sides bare over
        their whole height, the fingertips UNDER beneath its lower face."""
        m = self.m
        r_at = self.calc_frame_point([0.0, 0.0, 0.0])
        r_at[2] += TIP - m.CALC[2] / 2 - UNDER
        return r_at

    def fitted(self, side: str, target) -> np.ndarray:
        """The command that brings the hand's grasp point to ``target``: its last command moved by what the arm misses
        there (the simulator's grasp point against the command; nothing in a dry run)."""
        target = np.asarray(target, dtype=float)
        if not self.live:
            return target
        return target + self.hand(side) - np.array(self.mk.gripper(side)[0], dtype=float)

    def pinch(self, target: np.ndarray) -> dict:
        """The right hand's pinch closed where it should be: the grasp point set by the arm's own miss, then the grip;
        the calculator's place in the hand (from its middle, as commanded) kept for the later stages."""
        m, Move = self.m, self.Move
        cp, _ = self.pose("calc")
        r_cmd = self.fitted("right", target)
        self.r_hold = r_cmd - cp
        return {"right": [Move(0.8, r_cmd, None, 1.0, "R fits"), Move(0.6, None, None, grip(m.CALC[0] - SQUEEZE), "R grips calc")], "left": [Move(1.4, None, None, None, "L holds")]}

    def pinch_strip(self) -> dict:
        return self.pinch(self.strip_point())

    def pinch_middle(self) -> dict:
        return self.pinch(self.middle_point())

    def draw_part(self) -> dict:
        """Out of the click and DRAW_FIRST along the rails by the strip near the display end, the left holding the cover
        back; then neither pushes (the calculator stays on the rails)."""
        m, Move = self.m, self.Move
        self.require_grip("right", m.CALC[0], "the calculator")
        self.require_grip("left", m.COVER_W, "the cover's walls")
        cp, cr = self.pose("calc")
        out = cr @ np.array([0.0, 1.0, 0.0])  # the calculator leaves through the open end: its display end first
        r_at = self.hand("right")
        l_at = self.hand("left")
        if not self.live:
            self.dry["calc"] = (cp + out * DRAW_FIRST, cr)
        return {
            "left": [Move(0.6, l_at, None, None, "L holds back", push=-7.0 * out), Move(2.4, l_at, None, None, "L holds", push=-2.0 * out),
                     Move(0.4, None, None, None, "L eases", push=np.zeros(3))],
            "right": [Move(0.6, r_at + out * 0.004, None, None, "R out of click", push=7.0 * out), Move(2.4, r_at + out * DRAW_FIRST, None, None, "R draws out", push=2.0 * out),
                      Move(0.4, None, None, None, "R eases", push=np.zeros(3))],
        }  # fmt: skip

    def regrip_middle(self) -> dict:
        """The right hand lets go of the strip and comes down over the calculator's middle (open), the left holding."""
        Move = self.Move
        r_at = self.hand("right")
        mid = self.middle_point()
        return {
            "right": [Move(0.6, None, None, 1.0, "R lets go"), Move(0.8, r_at + [0, 0, 0.04], None, None, "R up"), Move(1.2, mid + [0, 0, 0.04], R_DOWN, None, "R over middle"),
                      Move(1.0, mid, None, None, "R down"), Move(0.6, None, None, None, "R settles")],
            "left": [Move(4.2, None, None, None, "L holds")],
        }  # fmt: skip

    def draw_rest(self) -> dict:
        """The rest of the way out by the pinch over the middle, the left holding the cover back until it comes free."""
        m, Move = self.m, self.Move
        self.require_grip("right", m.CALC[0], "the calculator")
        cp, cr = self.pose("calc")
        out = cr @ np.array([0.0, 1.0, 0.0])
        drawn = float(self.mk.joint("rails")) if self.live else DRAW_FIRST
        rest = m.RAILS + DRAW_PAST - drawn
        r_at = self.hand("right")
        l_at = self.hand("left")
        if not self.live:
            self.dry["calc"] = (cp + out * rest, cr)
        return {  # both pushes held until the calculator is free (eased off, the arm stopped 6 mm short of the rails' end: w12c16)
            "left": [Move(0.4, l_at, None, None, "L holds back", push=-2.0 * out), Move(2.0, l_at, None, None, "L holds")],
            "right": [Move(0.4, None, None, None, "R presses", push=2.0 * out), Move(2.0, r_at + out * rest, None, None, "R draws out")],
        }  # fmt: skip

    def lift(self) -> dict:
        m, Move = self.m, self.Move
        if self.live and m.STATE["on"] is not None:
            raise Stop(f"the cover is still on the {m.STATE['on']} (rails at {1000 * float(self.mk.joint('rails')):.2f} mm)")
        cp, cr = self.pose("calc")
        on_end = cp + cr @ np.array([0.0, -m.CALC[1] / 2, 0.0])  # its ON-key end (its own -y)
        target_middle = FLIP_AT + np.array([0.0, 0.0, m.TOP]) - (on_end - cp) * (1.0 - GRASP_IN / (m.CALC[1] / 2))
        r_at = target_middle + self.r_hold
        if not self.live:
            self.dry["calc"] = (target_middle, cr)
        l_start = self.hand("left")
        l_pre = FLIP_AT + np.array([0.0, 0.0, m.TOP]) + np.array([0.0, 0.07, 0.0])
        return {
            "left": [Move(0.5, None, None, 1.0, "L lets go", push=np.zeros(3)), Move(1.0, l_start + [0, 0, 0.06], None, None, "L up"), Move(2.0, l_pre, L_LEVEL, 0.45, "L to flip")],
            "right": [Move(0.5, None, None, None, "R waits", push=np.zeros(3)), Move(3.0, r_at, None, None, "R lifts calc")],
        }  # fmt: skip

    def hand_over(self) -> dict:
        m, Move = self.m, self.Move
        cp, cr = self.pose("calc")
        on_end = cp + cr @ np.array([0.0, -m.CALC[1] / 2, 0.0])
        l_at = on_end + (cp - on_end) / np.linalg.norm(cp - on_end) * GRASP_IN
        return {
            "left": [Move(1.5, l_at, L_LEVEL, 0.45, "L to ON end"), Move(0.6, None, None, grip(m.CALC[2] - SQUEEZE), "L grips end")],
            "right": [Move(2.1, None, None, None, "R holds")],
        }

    def flip(self) -> dict:
        Move = self.Move
        self.require_grip("left", self.m.CALC[2], "the calculator's thickness")
        r_at = self.hand("right")
        l_at = self.hand("left")
        cp, cr = self.pose("calc")
        if not self.live:  # the dry run keeps the calculator where the left hand turns it
            self.dry["calc"] = (cp, (roll(L_LEVEL, math.pi) @ L_LEVEL.T) @ cr)
        return {
            # the right hand straight up out of the calculator's roll (its corners sweep 41.6 mm round its long axis)
            "right": [Move(0.6, None, None, 1.0, "R lets go"), Move(1.2, r_at + FLIP_AWAY, None, None, "R away"), Move(2.6, None, None, None, "R waits")],
            "left": [Move(1.8, None, None, None, "L waits"), Move(2.6, l_at, roll(L_LEVEL, math.pi), None, "L rolls")],
        }

    def retake(self) -> dict:
        Move = self.Move
        r_at = self.middle_point()  # over the middle again, now keys up
        return {"right": [Move(1.6, r_at + [0, 0, 0.05], R_DOWN, 1.0, "R over calc"), Move(1.0, r_at, None, 1.0, "R down"), Move(0.6, None, None, None, "R settles")],
                "left": [Move(3.2, None, None, None, "L holds")]}  # fmt: skip

    def release(self) -> dict:
        """The left hand lets the calculator go once the right holds it, and moves out."""
        Move = self.Move
        self.require_grip("right", self.m.CALC[0], "the calculator")
        l_at = self.hand("left")
        return {"left": [Move(0.6, None, None, 0.45, "L lets go"), Move(1.0, l_at + [0, 0.07, 0], None, None, "L out")], "right": [Move(1.6, None, None, None, "R holds")]}

    def to_cover(self) -> dict:
        """The left hand takes the cover by its walls near the closed end again (leaning back); the right brings the
        calculator to NEAR out of the cover's open end, in line with its back rails."""
        m, Move = self.m, self.Move
        vp, vr = self.pose("cover")
        closed = vp + vr @ np.array([0.0, m.COVER_Y0 + GRASP_IN, 0.0])  # the cover near its closed end (its own -y)
        l_at = np.array([closed[0], closed[1], m.BOOK_TOP + OVER_BOOK + hand_under(L_HOLD)])
        want_p, want_r = self.entry(NEAR)
        r_at = want_p + self.r_hold
        if not self.live:
            self.dry["calc"] = (want_p, want_r)
        return {
            "left": [Move(2.0, l_at + [0, 0, 0.05], L_HOLD, 1.0, "L over cover"), Move(1.2, l_at, None, 1.0, "L down"), Move(0.6, None, None, grip(m.COVER_W - SQUEEZE), "L grips cover")],
            "right": [Move(2.4, r_at + [0, 0, 0.03], None, None, "R to entry"), Move(1.4, r_at, None, None, "R at entry")],
        }  # fmt: skip

    def entry(self, out: float) -> tuple[np.ndarray, np.ndarray]:
        """The calculator's pose (world) keys up in line with the cover's back rails, its ON-key end ``out`` m out of the
        open end."""
        m = self.m
        vp, vr = self.pose("cover")
        r = vr @ m.FLIP
        return vp - r @ np.array([0.0, -(m.RAILS + out), 0.0]), r

    def align(self) -> dict:
        """The calculator brought onto the line of the cover's back rails as the simulator has it: the right hand moved
        and turned by the calculator's own error (where and how it lies against where and how it should), so that the
        calculator, held as it is, comes onto the line."""
        Move = self.Move
        cp, cr = self.pose("calc")
        want_p, want_r = self.entry(NEAR)
        turn = want_r @ cr.T
        if self.live and (np.linalg.norm(want_p - cp) > 0.020 or math.degrees(self.m.angle(turn)) > 15.0):
            raise Stop(f"the calculator is {1000 * np.linalg.norm(want_p - cp):.1f} mm and {math.degrees(self.m.angle(turn)):.1f} deg off its entry pose")
        hand_p, hand_r = self.arms["right"].state(1e9)[:2]
        r_at = want_p + turn @ (np.asarray(hand_p, dtype=float) - cp)
        return {"right": [Move(1.0, r_at, turn @ np.asarray(hand_r, dtype=float), None, "R aligns")], "left": [Move(1.0, None, None, None, "L holds")]}

    def enter(self) -> dict:
        """The calculator's ON-key end into the cover's open end, in line, IN_FIRST in, held over its middle; the left
        hand holds the cover back by its walls near the closed end."""
        m, Move = self.m, self.Move
        self.require_grip("left", m.COVER_W, "the cover's walls")
        cp, cr = self.pose("calc")
        into = -(cr @ np.array([0.0, 1.0, 0.0]))  # the ON-key end first
        r_at = self.hand("right")
        if not self.live:
            self.dry["calc"] = (cp + into * (NEAR + IN_FIRST), cr)
        return {
            "right": [Move(1.6, r_at + into * (NEAR + IN_FIRST), None, None, "R enters", push=2.0 * into), Move(0.4, None, None, None, "R eases", push=np.zeros(3))],
            "left": [Move(1.6, None, None, None, "L holds back", push=-2.0 * into), Move(0.4, None, None, None, "L eases", push=np.zeros(3))],
        }  # fmt: skip

    def regrip_end(self) -> dict:
        """On the back rails, the right hand lets go of the middle and comes down over the strip near the display end
        (its fingertips over the walls' top), the left holding the cover."""
        m, Move = self.m, self.Move
        if self.live and m.STATE["on"] != "back":
            o = m.offer()
            raise Stop(f"not on the back rails: {1000 * (m.RAILS - o['travel']):.1f} mm in, {1000 * o['across']:.2f} mm across, {1000 * o['up']:.2f} mm up, {math.degrees(o['tilt']):.2f} deg")
        r_at = self.hand("right")
        end = self.strip_point()
        return {
            "right": [Move(0.6, None, None, 1.0, "R lets go"), Move(0.8, r_at + [0, 0, 0.04], None, None, "R up"), Move(1.4, end + [0, 0, 0.04], R_DOWN, None, "R over end"),
                      Move(1.0, end, None, None, "R down"), Move(0.6, None, None, None, "R settles")],
            "left": [Move(4.4, None, None, None, "L holds")],
        }  # fmt: skip

    def home(self) -> dict:
        """The right hand pushes the calculator home along the rails and through the click; the left holds the cover back
        by its walls near the closed end."""
        m, Move = self.m, self.Move
        if self.live and m.STATE["on"] != "back":
            o = m.offer()
            raise Stop(f"not on the back rails: {1000 * (m.RAILS - o['travel']):.1f} mm in, {1000 * o['across']:.2f} mm across, {1000 * o['up']:.2f} mm up, {math.degrees(o['tilt']):.2f} deg")
        cp, cr = self.pose("calc")
        into = -(cr @ np.array([0.0, 1.0, 0.0]))
        to_go = float(m.offer()["travel"]) if self.live else m.RAILS - IN_FIRST  # how far the calculator is from home
        self.require_grip("right", m.CALC[0], "the calculator")
        r_at = self.hand("right")
        r_home = r_at + into * (to_go + PAST_HOME)
        if not self.live:
            self.dry["calc"] = (cp + into * to_go, cr)
        return {
            "left": [Move(0.6, None, None, None, "L holds back", push=-2.5 * into), Move(4.0, None, None, None, "L holds"), Move(1.6, None, None, None, "L holds for click", push=-9.0 * into)],
            "right": [Move(0.6, None, None, None, "R presses", push=2.0 * into), Move(4.0, r_home - into * (PAST_HOME + m.DETENT_FROM + 0.001), None, None, "R pushes in"),
                      Move(1.6, r_home, None, None, "R clicks home", push=8.0 * into)],
        }  # fmt: skip

    def away(self) -> dict:
        from yam_arena import ik, station
        from yam_arena.constants import READY

        Move = self.Move
        ready = {side: ik.tcp_pose(np.array(READY, dtype=float), station.BASES[side]) for side in ("left", "right")}
        out = {}
        for side in ("left", "right"):
            at = self.hand(side)
            out[side] = [Move(0.6, None, None, 1.0, f"{side[0].upper()} lets go", push=np.zeros(3)), Move(1.0, at + [0, 0, 0.06], None, None, f"{side[0].upper()} up"),
                         Move(2.4, ready[side][0], ready[side][1], 1.0, f"{side[0].upper()} ready")]  # fmt: skip
        return out

    # --- the arms ---
    def hand(self, side: str) -> np.ndarray:
        """Where the plan has the hand's grasp point now (the end of its last move)."""
        return np.array(self.arms[side].state(1e9)[0], dtype=float)

    def require_grip(self, side: str, width: float, what: str) -> None:
        """Stop unless the ``side`` hand has closed on ``width`` (within 1.5 mm): on ``what``, not on something else."""
        if not self.live:
            return
        opening = OPEN * float(self.mk.gripper(side)[1])
        if abs(opening - width) > 0.0015:
            raise Stop(f"the {side} hand closed at {1000 * opening:.1f} mm, not on {what} ({1000 * width:.1f} mm)")

    def show(self, t: float, name: str) -> None:
        """One line on where the parts and the hands are as a stage starts (the hands: as read, and as commanded)."""
        if not self.live:
            return
        mm = lambda v: [round(1000 * float(x), 1) for x in v]  # noqa: E731
        (cp, cr), (vp, _) = self.pose("calc"), self.pose("cover")
        hands = {side: {"at": mm(self.mk.gripper(side)[0]), "asked": mm(self.hand(side)), "open (mm)": round(1000 * OPEN * float(self.mk.gripper(side)[1]), 1)} for side in ("left", "right")}
        print("STAGE " + json.dumps({"t": round(t, 2), "stage": name, "calc": mm(cp), "keys": [round(float(v), 3) for v in cr[:, 2]], "display end": [round(float(v), 3) for v in cr[:, 1]],
                                     "cover": mm(vp), "on": self.m.STATE["on"], "rails (mm)": round(1000 * float(self.mk.joint("rails")), 2), "hands": hands}), flush=True)  # fmt: skip

    def director(self, t: float, arms, pictures) -> None:
        """Before every step: when both arms have done their moves, the next stage is planned and appended (both arms
        end each stage together)."""
        self.arms = arms
        if self.live and "calc" not in self.mk.MECH.poses:
            return  # before the first step the simulator has not been read: the plan starts one step later
        if self.i >= len(self.stages) or t + 1e-9 < max(arm.seconds for arm in arms.values()):
            return
        stage = self.stages[self.i]
        self.show(t, stage.__name__)
        try:
            moves = stage()
            self.log.append({"t": round(t, 2), "stage": stage.__name__})
        except Stop as stop:
            print(f"PLAN STOPPED before {stage.__name__}: {stop}", flush=True)
            self.log.append({"t": round(t, 2), "stopped before": stage.__name__, "why": str(stop)})
            moves, self.i = self.away(), len(self.stages) - 1
        for side, arm in arms.items():
            arm.moves.extend(moves.get(side, []))
        last = max(arm.seconds for arm in arms.values())
        for side, arm in arms.items():
            if arm.seconds < last:
                arm.moves.append(self.Move(last - arm.seconds, None, None, None, "waits"))
        self.i += 1


def main() -> int:
    import demo_kit as kit
    from yam_arena import station

    quick = "--quick" in sys.argv
    record = "--norec" not in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    out = Path(args[0]) if args else HERE / "dry"
    if quick:
        from yam_arena import task_file

        task = task_file.load(HERE / "task.py")
        m = task.module
        plan = Plan(m, live=False)
        plan.start_dry()
        result = kit.dry_run({"left": [], "right": []}, station.BASES, director=plan.director, limit_s=120.0)
        print(f"DRY {json.dumps(result, default=str)}", flush=True)
        apart = plan_apart({side: plan.arms[side].moves for side in ("left", "right")}, station.BASES)
        print(f"ARMS APART {json.dumps(apart, default=str)}", flush=True)
        print(f"STAGES {json.dumps(plan.log)}", flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    plan = Plan(m, live=True)
    hz = float(embodiment.info.control_hz)

    def extra() -> dict:
        o = m.offer()
        (cp, cr), (vp, _) = m.mk.pose("calc"), m.mk.pose("cover")
        return {"on": m.STATE.get("on"), "travel": round(float(m.mk.joint("rails")), 5), "cover off home": round(o["travel"], 5), "across": round(o["across"], 5),
                "up": round(o["up"], 5), "tilt": round(math.degrees(o["tilt"]), 3), "score": m.score(None), "calc": [round(float(v), 4) for v in cp],
                "keys up": round(float(cr[2, 2]), 3), "cover": [round(float(v), 4) for v in vp]}  # fmt: skip

    folder = out / "full"
    result = kit.run(embodiment, {"left": [], "right": []}, folder, seed=0, director=plan.director, extra=extra, limit_s=m.RUN_LIMIT_STEPS / hz,
                     tail_s=3.0, sheet_frames=16, record=record)  # fmt: skip  # the task ends it SUCCESS_HOLD_S after 1.0, with both arms at ready
    print(f"STAGES {json.dumps(plan.log)}", flush=True)
    print(f"SCORE {m.score(None)} report {json.dumps(m.report(), default=str)}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
