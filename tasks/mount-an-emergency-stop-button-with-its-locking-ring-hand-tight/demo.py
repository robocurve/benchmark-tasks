#!/usr/bin/env python3
"""Reference runs of the emergency-stop task, the short trial of the hand's tightening, and the short trials the spec
asked for first.

    scripts/rta demo <id> mech/tasks/estop_mount [run,run,...] [--quick | --check]

``full``: the right hand takes the operator from above by its head, levels it at the hand-over and turns so that its
fingers close from above and below; the left hand unscrews the ring (four grips of 230 degrees) and keeps it; the right
hand turns the tab to the top, brings the operator in line with the hole (correcting by where it sees the barrel's axis
meet the panel), presses it in and holds it in; the left hand sets the ring on the barrel's end behind the panel and
screws it on in grips (from the second grip holding it 0.5 mm behind its fingertips) until it is 1 mm short of the
panel, then takes it with its front face 0.5 mm ahead of the fingertips and turns it home against the panel's back and
TIGHT_GRIPS grips more: hand-tight. It lets go of the ring and backs off; the right hand lets go of the head (the
hand-tight ring keeps the button in the hole) and goes clear. ``fingers``: as ``full`` without the last grips: the
ring left 1 mm short of the panel. ``seated``: the operator into the hole, the ring laid flat on the bench, both
hands let go. ``search``: the operator brought to the hole with its tab 20 degrees off the top, pushed in a step at a
time until the tab stands on the panel's face, then turned a degree at a time under a light push until it is felt to
go in (the full run instead reads the tab's angle from the simulator: the cameras do not show it, see the README).

``tighten`` (the short trial of the one act): the start set directly (the button seated and pushed in with 2 N, the
ring screwed by a torque to 0.52 mm short of the panel); the left hand takes the ring with its front face 0.5 mm ahead
of the fingertips, runs it to the panel and turns it on past the touch in TIGHTEN_GRIPS grips: how far does the pinch
turn it against the spring before it slips?

``ring`` (trial 1): the right hand, pointing down, closes on the operator's head, lifts it and turns it level at the
hand-over (x 0.38), pointing at the panel's side (+y) with the head in it. The left hand comes in line from behind,
pointing at the head, open, until the ring's front face is 4 mm behind its fingertips' ends, and unscrews the ring in
grips of 230 degrees (closes, turns, opens, turns back; it follows the ring 0.64 mm a grip). After the fourth grip it
draws the ring off the stem and holds it 6 cm away: is the ring turned 230 degrees a grip, taken off, and kept?
``seat`` (trial 2): as ``ring``, then the right hand turns the operator about its axis until the tab is at the top,
brings it in line with the hole with the stem's end 10 mm in front of the panel (where the arm arrives is read: how
far its axis is off the hole's and tilted), and pushes it in to the panel.

What the scripts read from the simulator: where the operator and the ring are, to put a hand on them and to aim them
(a person sees them), and in the full run the tab's angle; nothing else: not the thread's state, not the score.

``--quick``: every run's plan against the arms' reach and joint limits, without the simulator. ``--check``: in the
simulator without videos (a check of the plan, not a recording).
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

RUNS = ("ring", "seat", "look", "full", "search", "fingers", "seated", "tighten")
TIGHTEN_SET = -0.00252  # the tighten trial: the ring screwed by a torque to this far from the gasket (0.52 mm short of the panel's back, where the fingers left it in e2t)
TIGHTEN_WAIT, TIGHTEN_GRIPS = 17.0, 8  # s the hands wait while the ring is screwed on (7 turns at 0.03 N m took 15 s: e1l); grips past the touch
AHEAD_TIGHT = 0.0135  # m: the ring's front face this far ahead of the grasp point: 0.5 mm ahead of the fingertips' ends, so that it bears
# on the panel with the tips 0.5 mm clear (designer 11:56, point 3a; held 0.5 mm behind them, the tips came within 0.02 mm of the panel: e2t)
LOOK = 0.10  # m from the operator's bezel to the left hand's grasp point in the cue check
SEARCH_DEG = 20.0  # the search run: the tab this far off the top as the operator comes to the hole
ON_GRIPS_MOST, TIGHT_GRIPS = 18, 3  # grips the fingers give the ring at most; grips past its touch on the panel's back (hand-tight: the tighten trial)
FINGERS_STOP = 0.001  # m: the fingers run the ring until its front face is this short of the panel's back (fingertips 0.5 mm off it)
AHEAD_ON = 0.0125  # m: screwing it on (from the second grip), the ring's front face this far ahead of the grasp point: 0.5 mm behind the
# fingertips' ends, so that the fingers run it to 1 mm short of the panel (held 4 mm behind them, they stopped 4.5 mm short)
HOME_SEEN = 0.0001  # m: the ring's front face this near the panel's back: it is home (seen; it stops there, felt as the turn stiffening)
HOLD_N = 8.0  # N the right hand presses the operator into the hole with once it is in, until it lets go (by position alone the arm gave about 1 N: e2c)
RING_ON_AT = 0.0003  # m: the ring set with its front face this far onto the thread's first turn (within the start's 1 mm either way)
RING_DOWN = (0.40, 0.18)  # where the seated run lays the ring flat on the bench
SEARCH_PUSH, SEARCH_PAST, FLANGE_SEEN = 2.0, 10.0, 0.0005  # N the search pushes with; degrees it turns past the top at most; m: the flange this near the panel, the operator is in
INSERT_STEP, INSERT_S, INSERT_MOST = 0.0015, 0.5, 24  # the search's insertion: m a step, s a step, steps at most
OPEN, RING_OPEN, CLOSE = 0.70, 0.42, 0.0  # the hand's opening (0 closed .. 1 open, 94 mm): 66 mm; 39 mm round the 28.4 mm ring; closed, 50 N a finger
HANDOVER = np.array([0.38, -0.040, 0.16])  # the right hand's grasp point (over the bench) with the operator level: in line with the left hand
AHEAD = 0.009  # m the ring's front face lies ahead of the left grasp point: 4 mm behind the fingertips' ends (13 mm ahead of it)
GRIP_DEG, GRIPS = 230.0, 4  # a grip's turn; the grips that take the ring off (two turns on: 3.1 grips)
LEFT_ROLL0 = 205.0  # the left hand's roll at the start of an unscrewing grip (joint 6 at +115 degrees)
TURN_S, LET_S, BACK_S, CLOSE_S = 1.0, 0.3, 0.8, 0.3  # a grip: turn, open, turn back, close (2.4 s)
DRAW = 0.060  # m the left hand draws the ring back off the stem
FRONT_GAP = 0.010  # m: the stem's end this far in front of the panel when the arm has brought the operator in line
AIM_TRIES, AIM_OK = 3, 0.0003  # looks at most, and the miss (m) at the panel's front it accepts before pushing in


def hand_rot(approach, roll_deg: float) -> np.ndarray:
    """The hand pointing along ``approach``, its fingers closing along the axis that is turned ``roll_deg`` from the
    up direction (for a level approach; from +x for a vertical one) about the approach axis: columns closing axis,
    pads' width axis, approach axis. Joint 6 is about roll - 90 degrees for the level poses used here."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    ref = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.95 else np.array([1.0, 0.0, 0.0])
    u = ref - (ref @ a) * a
    u /= np.linalg.norm(u)
    v = np.cross(a, u)
    roll = math.radians(roll_deg)
    c = math.cos(roll) * u + math.sin(roll) * v
    return np.column_stack([c, np.cross(a, c), a])


def turn_onto(a, b) -> np.ndarray:
    """The least rotation that turns direction ``a`` onto direction ``b``."""
    a = np.asarray(a, dtype=float) / np.linalg.norm(a)
    b = np.asarray(b, dtype=float) / np.linalg.norm(b)
    c, w = float(a @ b), np.cross(a, b)
    s = float(np.linalg.norm(w))
    if s < 1e-12:
        if c > 0.0:
            return np.eye(3)
        raise ValueError("turn_onto: opposite directions, the turn is not defined")
    k = np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]]) / s
    angle = math.atan2(s, c)
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def tab_roll(m, seed) -> tuple[float, float]:
    """(the right hand's roll with the operator level, and at the hole with the tab at the top), from the operator's
    start angle (its tab's direction, which a person sees on the bench): rolls within joint 6's reach, the second as
    near 90 degrees (joint 6 at 0) as can be. The hand takes the head from above with its fingers closing at angle
    psi from +x; level, its roll is 90 - psi, and the tab stands at its start angle phi from +x towards +z."""
    phi = math.degrees(m.start_angle(seed))
    best = None
    for change in (phi - 90.0, phi - 90.0 - 360.0, phi - 90.0 + 360.0):  # the roll that brings the tab to +z
        for at_hole in np.arange(-25.0, 205.1, 5.0):
            level = at_hole - change
            if -25.0 <= level <= 205.0 and abs(change) <= 240.0:
                key = (abs(at_hole - 90.0), abs(change))
                if best is None or key < best[0]:
                    best = (key, level, at_hole)
    if best is None:
        raise ValueError(f"no roll brings the tab from {phi} degrees to the top within joint 6's reach")
    return best[1], best[2]


class Plan:
    """The moves, step by step; ``see()`` gives the operator's and the ring's poses (None: the plan's own guesses)."""

    def __init__(self, m, seed=0, see=None) -> None:
        import demo_kit as kit

        self.m, self.see, self.kit, self.Move = m, see, kit, kit.Move
        self.level_roll, self.hole_roll = tab_roll(m, seed)
        self.hold_roll = min((r for r in (0.0, 180.0)), key=lambda r: abs(r - self.level_roll))  # the right hand's roll with its fingers closing vertically, nearest the level one
        self.top = m.TOP
        self.hand_rot_now: dict = {"right": None, "left": None}  # each hand's turn as the plan has it now
        self.held = {"operator": np.array([0.0, 0.0, (m.HEAD_FROM + m.HEAD_TO) / 2])}  # dry run: frame from grasp, hand frame
        self.aim_misses: list[float] = []  # mm the operator's axis missed the hole's centre on arrival, before each correction
        self.right_free = False  # the right hand has let go of the head

    def operator(self) -> tuple[np.ndarray, np.ndarray]:
        if self.see is not None:
            return self.see()["operator"]
        return self.guess_operator

    def ring_front(self) -> tuple[np.ndarray, np.ndarray]:
        """(the ring's front face's middle, the operator's axis towards the head), where they are now."""
        if self.see is not None:
            seen = self.see()
            p, r = seen["operator"]
            q, _ = seen["ring"]
            axis = r @ self.m.Z
            return p + axis * float(axis @ (q - p)), axis
        p, r = self.guess_operator
        return p + r @ np.array([0.0, 0.0, self.guess_along]), r @ self.m.Z

    def both(self, left: list, right: list) -> dict:
        """One step for both hands, the shorter list padded with a wait so that both end together."""
        Move = self.Move
        lt, rt = sum(mv.seconds for mv in left), sum(mv.seconds for mv in right)
        if lt < rt:
            left = left + [Move(rt - lt, label="waits")]
        elif rt < lt:
            right = right + [Move(lt - rt, label="waits")]
        return {"left": left, "right": right}

    def take_and_level(self, upright: bool = True):
        """The right hand takes the operator from above by its head, brings it level to the hand-over and
        (``upright``) turns so that its fingers close from above and below (``hold_roll``, 0 or 180 degrees)."""
        m, Move = self.m, self.Move
        p, r = (m.OPERATOR_START, m.mk.axis_angle(m.Z, m.start_angle(0))) if self.see is None else self.operator()
        head = p + r @ np.array([0.0, 0.0, (m.HEAD_FROM + m.HEAD_TO) / 2])  # the head's middle: the fingertips reach its rear edge
        psi = 90.0 - self.level_roll  # the closing axis's angle from +x, from above, that turns into this roll when level
        down = self.kit.about([1.0, 0.0, 0.0], -math.pi / 2) @ hand_rot([0.0, 1.0, 0.0], 90.0 - psi)  # level pose turned back to pointing down
        level = hand_rot([0.0, 1.0, 0.0], self.level_roll)
        handover = np.array([HANDOVER[0], HANDOVER[1], self.top + HANDOVER[2]])
        up = np.array([0.0, 0.0, 0.06])
        yield self.both([], [Move(2.5, pos=head + up, rot=down, grip=OPEN, label="over the head"), Move(1.5, pos=head, label="down to the head"),
                             Move(0.6, grip=CLOSE, label="closes on the head"), Move(1.5, pos=head + np.array([0.0, 0.0, 0.10]), label="lifts it"),
                             Move(3.0, pos=handover, rot=level, label="level at the hand-over"), Move(0.5, label="holds")])  # fmt: skip
        self.hand_rot_now["right"] = level
        if upright:  # the fingers closing from above and below while the ring comes off: drawn off, the ring pulls the head sideways, and a head pulled along the fingers' closing line slides between them (e2d at roll 150, e2f at 90: 16.7 mm)
            hold = hand_rot([0.0, 1.0, 0.0], self.hold_roll)
            yield self.both([], [Move(1.5, rot=hold, label="holds it from above and below"), Move(0.3, label="holds")])
            self.hand_rot_now["right"] = hold
        self.guess_operator = (handover + np.array([0.0, (m.HEAD_FROM + m.HEAD_TO) / 2, 0.0]), self.kit.about([1.0, 0.0, 0.0], math.pi / 2) @ m.mk.axis_angle(m.Z, m.start_angle(0)))
        self.guess_along = m.A_START

    def unscrew(self):
        """The left hand in line behind the ring takes it off in grips of GRIP_DEG and draws it off the stem."""
        m, Move = self.m, self.Move
        front, axis = self.ring_front()
        grasp = front - AHEAD * axis
        r0, r1, r2 = LEFT_ROLL0, LEFT_ROLL0 - GRIP_DEG / 2, LEFT_ROLL0 - GRIP_DEG
        step = m.LEAD * GRIP_DEG / 360.0  # the ring comes back along the stem this far a grip
        yield self.both([Move(2.5, pos=grasp - 0.06 * axis, rot=hand_rot(axis, 90.0), grip=OPEN, label="behind the ring"), Move(2.0, pos=grasp, label="onto the ring"),
                         Move(1.2, rot=hand_rot(axis, r0), label="turns open round it")], [])  # fmt: skip
        for k in range(1, GRIPS + 1):
            front, axis = self.ring_front()
            grasp = front - AHEAD * axis
            moves = [Move(CLOSE_S, pos=grasp, rot=hand_rot(axis, r0), grip=CLOSE, label=f"grip {k}"),
                     Move(TURN_S / 2, pos=grasp - step / 2 * axis, rot=hand_rot(axis, r1), label=f"unscrews {k}"),
                     Move(TURN_S / 2, pos=grasp - step * axis, rot=hand_rot(axis, r2), label=f"unscrews {k}")]  # fmt: skip
            if k < GRIPS:
                moves += [Move(LET_S, grip=RING_OPEN, label=f"lets go {k}"), Move(BACK_S / 2, rot=hand_rot(axis, r1), label=f"back {k}"),
                          Move(BACK_S / 2, rot=hand_rot(axis, r0), label=f"back {k}")]  # fmt: skip
            self.guess_along -= step
            yield self.both(moves, [])
        yield self.both([Move(1.5, pos=grasp - step * axis - 0.020 * axis, label="draws it off"), Move(1.5, pos=grasp - DRAW * axis, label="away with it"),
                         Move(1.0, label="keeps it")], [])  # fmt: skip
        self.hand_rot_now["left"] = hand_rot(axis, r2)

    def grasp_for(self, body: str, rot, origin) -> np.ndarray:
        """The grasp point that puts ``body``'s frame at ``origin`` with the hand turned as ``rot``, from where the body
        sits in the hand now (the right hand for the operator, the left for the ring; the plan's guess
        in a dry run)."""
        if self.see is None:
            return np.asarray(origin, dtype=float) - rot @ self.held.get(body, np.zeros(3))
        side = "right" if body == "operator" else "left"
        p, _ = self.see()[body]
        held = self.hand_rot_now[side].T @ (p - self.see()["grippers"][side])
        return np.asarray(origin, dtype=float) - rot @ held

    def aimed(self, body: str, rot, axis_to) -> np.ndarray:
        """``rot`` turned the least that brings ``body``'s axis, as the body sits in the hand now, onto ``axis_to``
        (its axis is seen: the operator's head and barrel, the ring's round ends)."""
        if self.see is None:
            return rot
        side = "right" if body == "operator" else "left"
        _, r = self.see()[body]
        return turn_onto(rot @ (self.hand_rot_now[side].T @ (r @ self.m.Z)), axis_to) @ rot

    def to_hole(self, search: bool = False):
        """The right hand turns the tab to the top (``search``: SEARCH_DEG short of it), brings the operator's axis
        onto the hole's with the stem's end FRONT_GAP in front of the panel, and pushes it in. Searching, it pushes
        lightly (SEARCH_PUSH) until the tab stands on the panel's face, then turns it a degree at a time towards the
        top until it sees the flange meet the panel (FLANGE_SEEN) and pushes it home."""
        m, Move = self.m, self.Move
        roll = self.hole_roll - (SEARCH_DEG if search else 0.0)
        rot = hand_rot([0.0, 1.0, 0.0], roll)
        yield self.both([], [Move(2.0, rot=rot, label="tab to the top" if not search else "tab 20 degrees off the top"), Move(0.3, label="holds")])
        self.hand_rot_now["right"] = rot
        if self.see is not None and not search:  # the tab's angle read from the state (README), the rest of the turn made up
            roll += math.degrees(m.seat_reading()["turn"])
            rot = hand_rot([0.0, 1.0, 0.0], roll)
            yield self.both([], [Move(0.8, rot=rot, label="tab to the top")])
            self.hand_rot_now["right"] = rot
        hole = np.array([0.0, -1.0, 0.0])  # the hole's axis towards a seated operator's head
        rot = self.aimed("operator", rot, hole)
        front = m.SEAT_AT + np.array([0.0, m.STEM_FROM - FRONT_GAP, 0.0])  # the operator's origin with its stem's end FRONT_GAP in front of the panel
        at = self.grasp_for("operator", rot, front)
        self.hand_rot_now["right"] = rot
        yield self.both([], [Move(2.5, pos=at, rot=rot, label="in line with the hole"), Move(1.0, label="arrives")])
        for k in range(AIM_TRIES):  # where its axis meets the panel, seen against the hole: the hand corrects by the miss
            if self.see is None:
                break
            miss = self.hole_miss()
            self.aim_misses.append(round(1000 * float(np.linalg.norm(miss)), 3))
            if np.linalg.norm(miss) <= AIM_OK:
                break
            at = at - miss  # the hand sent as far the other way: what the arm gives under the load stays as it was
            yield self.both([], [Move(1.0, pos=at, label="corrects its aim"), Move(0.5, label="arrives")])
        home = at + (m.SEAT_AT - front)  # the corrected aim carried along the hole's axis
        if not search:
            yield self.both([], [Move(2.5, pos=home, label="pushes it in"), Move(1.0, push=(0.0, HOLD_N, 0.0), label="holds it in")])
            return
        # The barrel goes in INSERT_STEP at a time, the hand corrected sideways by the seen miss each time (the bore leaves
        # the thread 0.15 mm a side; pushed in one move it drifted 2 mm and stuck at the bore's edge: e2f), until it stops
        # going in: the tab stands on the panel's face.
        pos, depth = at, float(self.operator()[0][1]) if self.see is not None else 0.0
        for k in range(INSERT_MOST):
            pos = pos + np.array([0.0, INSERT_STEP, 0.0])
            if self.see is not None:
                pos = pos - self.hole_miss()
            yield self.both([], [Move(INSERT_S, pos=pos, label="pushes it in"), Move(0.3, label="steadies")])  # read once the arm has caught up (read at once, the arm's lag looked like the tab on the face: e2i)
            if self.see is None:
                if pos[1] >= (home + np.array([0.0, m.TAB_FROM, 0.0]))[1]:
                    break
                continue
            now = float(self.operator()[0][1])
            if now - depth < INSERT_STEP / 3:  # it went in less than a third of a step: the tab stands on the panel's face
                break
            depth = now
        self.insert_steps = k + 1
        yield self.both([], [Move(0.5, push=(0.0, SEARCH_PUSH, 0.0), label="the tab on the panel's face"), Move(0.3, label="steadies")])
        axis_point = np.array([m.HOLE[0], 0.0, m.HOLE[2]])  # on the hole's axis: the hand turns about it, the operator in the bore
        for k in range(1, int(SEARCH_DEG + SEARCH_PAST) + 1):
            about = self.kit.about([0.0, 1.0, 0.0], math.radians(k))  # k degrees further towards the top, about the hole's axis
            turned = about @ rot
            self.hand_rot_now["right"] = turned
            yield self.both([], [Move(0.1, pos=axis_point + about @ (pos - axis_point), rot=turned, label="turns it under the push")])
            if self.see is not None and float(self.operator()[0][1]) >= m.SEAT_AT[1] - FLANGE_SEEN:  # in: the flange at the panel (the tab on the face holds it 1.2 to 2 mm off; leaning in the bore's play it went 0.5 mm further: e2k)
                break
        self.search_turned = k
        pos = axis_point + about @ (pos - axis_point)
        home = pos + np.array([0.0, float(m.SEAT_AT[1] - (self.operator()[0][1] if self.see is not None else m.SEAT_AT[1])), 0.0])
        yield self.both([], [Move(1.5, pos=home, push=(0.0, HOLD_N, 0.0), label="pushes it home"), Move(1.0, label="holds it in")])

    def hole_miss(self) -> np.ndarray:
        """Where the operator's axis meets the panel's front plane, less the hole's centre (m, x and z): both seen."""
        m = self.m
        p, r = self.operator()
        axis = r @ m.Z
        miss = p + (m.FRONT_Y - p[1]) / axis[1] * axis - m.HOLE
        miss[1] = 0.0
        return miss

    def ring_on(self):
        """The left hand, still holding the ring as it came off, sets it on the barrel's end behind the panel in line and
        screws it on in grips of GRIP_DEG until its fingertips near the panel's back (1 mm short), the hand left open round it."""
        m, Move = self.m, self.Move
        p, r = self.operator()
        axis = r @ m.Z  # towards the head: -y, seated
        roll0, roll1, roll2 = LEFT_ROLL0 - GRIP_DEG, LEFT_ROLL0 - GRIP_DEG / 2, LEFT_ROLL0
        rot = self.aimed("ring", hand_rot(axis, roll0), axis)
        start = self.grasp_for("ring", rot, p + axis * (m.A_END + RING_ON_AT))
        self.hand_rot_now["left"] = rot
        step = m.LEAD * GRIP_DEG / 360.0
        yield self.both([Move(3.0, pos=start - 0.03 * axis, rot=rot, label="behind the panel with the ring")], [])
        p, r = self.operator()  # where operator and ring are now, for the last 3 cm
        axis = r @ m.Z
        rot = self.aimed("ring", rot, axis)
        start = self.grasp_for("ring", rot, p + axis * (m.A_END + RING_ON_AT))  # its front face on the thread's first turn
        self.hand_rot_now["left"] = rot
        yield self.both([Move(2.0, pos=start, rot=rot, label="ring to the barrel's end")], [])
        grasp, self.ring_grips = start, 0
        for k in range(1, ON_GRIPS_MOST + 1):
            front, axis = self.ring_front()
            on = float((front - self.operator()[0]) @ axis)  # the ring's front face from the gasket along the barrel (seen)
            if on >= m.A_TOUCH - FINGERS_STOP:  # the fingertips near the panel's back: the last grips take it further back on the ring
                break
            self.ring_grips = k
            if k > 1:  # re-grips the ring where it is on the barrel, near the fingertips
                grasp = front - AHEAD_ON * axis
                rot = hand_rot(axis, roll0)
            yield self.both([Move(CLOSE_S, pos=grasp, rot=rot, grip=CLOSE, label=f"screws on {k}"),
                             Move(TURN_S / 2, pos=grasp + step / 2 * axis, rot=hand_rot(axis, roll1), label=f"screws on {k}"),
                             Move(TURN_S / 2, pos=grasp + step * axis, rot=hand_rot(axis, roll2), label=f"screws on {k}"),
                             Move(LET_S, grip=RING_OPEN, label=f"lets go {k}"), Move(BACK_S / 2, rot=hand_rot(axis, roll1), label=f"back {k}"),
                             Move(BACK_S / 2, rot=hand_rot(axis, roll0), label=f"back {k}")], [])  # fmt: skip
            self.guess_along += step
        self.left_at = grasp

    def left_off(self):
        """The left hand lets go of the ring and backs off along its axis."""
        Move = self.Move
        _, axis = self.ring_front()
        yield self.both([Move(0.4, grip=OPEN, label="lets go of the ring"), Move(1.5, pos=self.left_at - 0.06 * axis, label="back from the ring")], [])

    def tighten_on(self, past_most: int):
        """The left hand, open round the ring, takes it with its front face 0.5 mm ahead of the fingertips (AHEAD_TIGHT), so
        that it bears on the panel with the tips clear, and turns it in grips of GRIP_DEG until it sees it home against
        the panel's back and ``past_most`` grips more: hand-tight. It follows the ring along the thread, never past the
        panel, and is left open round it."""
        m, Move = self.m, self.Move
        r0, r1, r2 = LEFT_ROLL0 - GRIP_DEG, LEFT_ROLL0 - GRIP_DEG / 2, LEFT_ROLL0
        past, self.tighten_grips = 0, 0
        for k in range(1, ON_GRIPS_MOST + 1):
            front, axis = self.ring_front()
            gap = m.A_TOUCH - float((front - self.operator()[0]) @ axis)  # the ring's front face short of the panel's back (seen)
            if gap <= HOME_SEEN:
                past += 1
                if past > past_most:
                    break
            self.tighten_grips = k
            grasp = front - AHEAD_TIGHT * axis
            advance = max(0.0, min(m.LEAD * GRIP_DEG / 360.0, gap))
            yield self.both([Move(CLOSE_S, pos=grasp, rot=hand_rot(axis, r0), grip=CLOSE, label=f"tightens {k}"),
                             Move(TURN_S / 2, pos=grasp + advance / 2 * axis, rot=hand_rot(axis, r1), label=f"tightens {k}"),
                             Move(TURN_S / 2, pos=grasp + advance * axis, rot=hand_rot(axis, r2), label=f"tightens {k}"),
                             Move(LET_S, grip=RING_OPEN, label=f"lets go {k}"), Move(BACK_S / 2, rot=hand_rot(axis, r1), label=f"back {k}"),
                             Move(BACK_S / 2, rot=hand_rot(axis, r0), label=f"back {k}")], [])  # fmt: skip
            self.guess_along = min(m.A_TOUCH, self.guess_along + advance)
            self.left_at = grasp

    def release_right(self):
        """The right hand lets go of the operator's head and goes clear while the left hand waits: a hand-tight ring keeps the
        button in the hole and the tab in the notch keeps it from turning. It opens with its push still on and eases the push off after: eased with the fingers
        closed, the hand swung 15 mm sideways along the head and drew the button 2.1 mm out of its seat (e2p fingers)."""
        m, Move = self.m, self.Move
        head_mid = (m.HEAD_FROM + m.HEAD_TO) / 2
        home = m.SEAT_AT + np.array([0.0, -head_mid, 0.0])
        yield self.both([Move(1.0, label="waits")], [Move(0.6, grip=1.0, label="lets go of the operator"), Move(0.9, push=(0.0, 0.0, 0.0), label="eases off"),
                                                     Move(1.5, pos=home + np.array([0.0, -0.07, 0.0]), label="back off"), Move(1.5, pos=home + np.array([0.0, -0.10, 0.08]), label="clear")])  # fmt: skip
        self.right_free = True

    def let_go(self):
        """The right hand lets go of the head if it still holds it; both hands still for 2 s."""
        Move = self.Move
        if not self.right_free:
            yield from self.release_right()
        yield self.both([Move(2.0, label="still")], [Move(2.0, label="still")])

    def ring_down(self):
        """The left hand, holding the ring as it came off, turns it flat (pointing down) and lays it on the bench at
        RING_DOWN, lets go and goes up clear."""
        m, Move = self.m, self.Move
        approach = self.hand_rot_now["left"][:, 2]
        level = hand_rot(approach, 90.0)
        down = self.kit.about([1.0, 0.0, 0.0], math.pi / 2) @ level  # a quarter turn about x: pointing down, the ring's face on the bench
        on = self.grasp_for("ring", down, (RING_DOWN[0], RING_DOWN[1], m.TOP + 0.001))
        self.hand_rot_now["left"] = down
        yield self.both([Move(1.2, rot=level, label="turns its hand"), Move(3.0, pos=on + np.array([0.0, 0.0, 0.06]), rot=down, label="over the bench with the ring"),
                         Move(1.5, pos=on, label="lays it down"), Move(0.5, grip=OPEN, label="lets go of the ring"), Move(1.5, pos=on + np.array([0.0, 0.0, 0.10]), label="clear")], [])  # fmt: skip

    def look(self):
        """The cue check: with the operator level in the right hand, the left hand's camera 10 cm from its bezel, from
        above and from the far side (+x), before and after the right hand turns the tab to the top."""
        m, Move = self.m, self.Move
        p, r = self.operator()
        bezel = p + r @ np.array([0.0, 0.0, (m.BEZEL_FROM + m.HEAD_FROM) / 2])
        above, side = bezel + np.array([0.0, 0.0, LOOK]), bezel + np.array([LOOK, 0.0, 0.0])
        yield self.both([Move(3.0, pos=above, rot=hand_rot([0.0, 0.0, -1.0], 0.0), grip=OPEN, label="looks from above"), Move(1.0, label="looks from above"),
                         Move(2.5, pos=side, rot=hand_rot([-1.0, 0.0, 0.0], 90.0), label="looks from the side"), Move(1.0, label="looks from the side")], [])  # fmt: skip
        yield self.both([], [Move(2.0, rot=hand_rot([0.0, 1.0, 0.0], self.hole_roll), label="tab to the top"), Move(0.5, label="holds")])
        yield self.both([Move(2.5, pos=above, rot=hand_rot([0.0, 0.0, -1.0], 0.0), label="looks from above, tab up"), Move(1.0, label="looks from above, tab up"),
                         Move(2.5, pos=side, rot=hand_rot([-1.0, 0.0, 0.0], 90.0), label="looks from the side, tab up"), Move(1.0, label="looks from the side, tab up")], [])  # fmt: skip

    def tighten(self):
        """The tighten trial (designer 11:56, points 3 and 4): the director sets the start directly (the button seated and
        pushed in with 2 N, the ring screwed by a torque to TIGHTEN_SET); the left hand comes in line behind the ring,
        takes it with its front face 0.5 mm ahead of the fingertips, runs it to the panel and turns it on past the touch in
        grips of GRIP_DEG, TIGHTEN_GRIPS of them: how far does the pinch turn it against the spring before it slips?"""
        m, Move = self.m, self.Move
        self.guess_operator, self.guess_along = (np.array(m.SEAT_AT, dtype=float), m.SEATED_ROT), TIGHTEN_SET
        yield self.both([Move(TIGHTEN_WAIT, label="waits")], [Move(TIGHTEN_WAIT, label="waits")])
        front, axis = self.ring_front()
        grasp = front - AHEAD_TIGHT * axis
        r0, r1, r2 = LEFT_ROLL0 - GRIP_DEG, LEFT_ROLL0 - GRIP_DEG / 2, LEFT_ROLL0
        yield self.both([Move(2.5, pos=grasp - 0.06 * axis, rot=hand_rot(axis, 90.0), grip=OPEN, label="behind the ring"), Move(2.0, pos=grasp, label="onto the ring"),
                         Move(1.2, rot=hand_rot(axis, r0), grip=RING_OPEN, label="turns open round it")], [])  # fmt: skip
        self.hand_rot_now["left"] = hand_rot(axis, r0)
        yield from self.tighten_on(TIGHTEN_GRIPS)
        yield from self.left_off()
        yield self.both([Move(1.5, label="still")], [Move(1.5, label="still")])

    def steps(self, name: str):
        if name == "tighten":
            yield from self.tighten()
            return
        yield from self.take_and_level(upright=name != "look")
        if name == "look":
            yield from self.look()
            return
        yield from self.unscrew()
        if name in ("seat", "search"):
            yield from self.to_hole(search=name == "search")
        if name == "seated":  # the lowest level: the button in the panel, the ring put down, both hands off
            yield from self.to_hole()
            yield from self.ring_down()
            yield from self.let_go()
        if name in ("full", "fingers"):
            yield from self.to_hole()
            self.guess_operator = (np.array(self.m.SEAT_AT, dtype=float), self.m.SEATED_ROT)
            self.guess_along = self.m.A_END
            yield from self.ring_on()
            if name == "full":
                yield from self.tighten_on(TIGHT_GRIPS)
            yield from self.left_off()
            yield from self.let_go()

    def whole(self, name: str) -> dict:
        moves: dict = {"left": [], "right": []}
        for step in self.steps(name):
            for side in moves:
                moves[side] += step[side]
        return moves


def set_seated(m) -> None:
    """The tighten trial's start, set directly: the button written seated in the hole (as the seat takes it) with what rides on
    it as built (the ring two turns on its thread), and joined to the seat's slide."""
    mk = m.mk
    p0, r0 = mk.built_pose("operator")
    at, rot = np.array(m.SEAT_AT, dtype=float), m.SEATED_ROT
    for name in ("operator", "ring", "carrier", "turner"):
        position, rotation = mk.built_pose(name)
        mk.write_pose(name, at + rot @ (r0.T @ (position - p0)), mk.matrix_to_quat(rot @ (r0.T @ rotation)))
    mk.write_pose("seat", at, mk.matrix_to_quat(rot))
    mk.make("seated")
    m.STATE["seated"] = True
    m.STATE["events"].append((mk.now(), "set directly: the button seated, the ring two turns on"))


def main() -> int:
    import demo_kit as kit

    quick, check = "--quick" in sys.argv, "--check" in sys.argv  # --check: in the simulator without videos (a check, not a recording)
    args = [a for a in sys.argv[1:] if a not in ("--quick", "--check")]
    out = Path(args[0])
    runs = args[1].split(",") if len(args) > 1 else ["ring"]
    if any(run not in RUNS for run in runs):
        raise SystemExit(f"a run is one of {sorted(RUNS)}; got {runs}")
    if quick:
        from yam_arena import station, task_file

        m = task_file.load(str(HERE / "task.py")).module
        for run in runs:
            for seed in (0, 1, 2, 3):
                plan = Plan(m, seed)
                try:
                    found = json.dumps(kit.dry_run(plan.whole(run), station.BASES, limit_s=m.RUN_LIMIT_STEPS / 20.0), default=str)
                except ValueError as error:
                    found = f"REFUSED: {error}"
                print(f"DRY {run} seed {seed} (rolls level {plan.level_roll:.0f}, at the hole {plan.hole_roll:.0f}): {found}", flush=True)
        return 0
    embodiment = kit.open_task(HERE / "task.py")
    m = embodiment.task.module
    mk = m.mk
    hz = float(embodiment.info.control_hz)
    failed = []
    for run in runs:
        try:
            plan = Plan(m, 0, lambda: {**{name: mk.pose(name) for name in ("operator", "ring")}, "grippers": {side: mk.gripper(side)[0] for side in ("left", "right")}})
            script = plan.steps(run)
            state = {"over": False, "first": True}

            def director(t: float, arms: dict, pictures, script=script, state=state, run=run) -> None:
                if pictures is None or mk.MECH.environment is None or not mk.MECH.poses:
                    return
                state["first"] = False
                if run == "tighten":  # the start set directly: the button seated and pushed in with 2 N, the ring screwed on by a torque
                    if not state.get("set"):
                        set_seated(m)
                        state["set"] = True
                    mk.push("operator", force=(0.0, 2.0, 0.0))
                    lift = (0.0, 0.0, m.CARRIED["ring"] * m.GRAVITY)
                    if not m.STATE["tight"] and m.ring_along() < TIGHTEN_SET:
                        mk.push("ring", force=lift, torque=tuple(0.03 * (mk.pose("operator")[1] @ m.Z)))
                    else:
                        mk.push("ring", force=lift)
                while not state["over"] and t >= max(arm.seconds for arm in arms.values()) - 1e-6:  # the next step once both hands are through
                    try:
                        step = next(script, None)
                    except Exception as error:  # the plan failed: the run goes on to its end (hands holding) so that it is recorded and read
                        import traceback

                        traceback.print_exc()
                        state["over"], state["error"] = True, repr(error)
                        print(f"PLAN FAILED at {t:.2f} s: {error!r}", flush=True)
                        break
                    if step is None:
                        state["over"] = True
                        break
                    for side in ("left", "right"):
                        arms[side].moves.extend(step[side])
                    first = {side: next((mv.label for mv in step[side] if mv.label), "") for side in ("left", "right")}
                    print(f"STEP {t:7.2f} L {first['left']!r} R {first['right']!r} {json.dumps(extra(), default=str)}", flush=True)

            def extra() -> dict:
                g = m.seat_reading()
                left, right = mk.gripper("left")[0], mk.gripper("right")[0]
                ring_p, ring_r = mk.pose("ring")
                op_p, op_r = mk.pose("operator")
                return {"ring on": m.STATE["ring_on"], "turn (turns)": round(mk.joint("turn") / (2 * math.pi), 4), "ring along (mm)": round(1000 * m.ring_along(), 3),
                        "ring off axis (mm, deg)": [round(1000 * v, 3) if k == 0 else round(math.degrees(v), 2) for k, v in enumerate(m.ring_off_axis())],
                        "ring from left grasp (mm)": [round(1000 * float(v), 2) for v in ring_p - left], "operator from right grasp (mm)": [round(1000 * float(v), 2) for v in op_p - right],
                        "operator axis": [round(float(v), 4) for v in op_r @ m.Z], "ring axis": [round(float(v), 4) for v in ring_r @ m.Z],
                        "grips": [round(float(mk.gripper("left")[1]), 3), round(float(mk.gripper("right")[1]), 3)],
                        "seat": {"off (mm)": round(1000 * g["off"], 3), "tilt (deg)": round(math.degrees(g["tilt"]), 2), "tab (deg)": round(math.degrees(g["turn"]), 2), "rear in (mm)": round(1000 * g["rear in"], 2)},
                        "seated": m.STATE["seated"], "seat (mm)": round(1000 * mk.joint("seat"), 3),
                        "gasket gap (mm)": round(1000 * g["gasket gap"], 3), "tight": m.STATE["tight"], "lock (deg)": round(math.degrees(m.STATE["lock"]), 2),
                        "past touch (deg)": round(math.degrees(mk.joint("tight")), 2), "hand on ring": m.hand_on_ring(),
                        "hands from the parts (mm)": {side: round(1000 * min(m.distance_to(name, mk.gripper(side)[0]) for name in m.PARTS), 1) for side in ("left", "right")},
                        "events": len(m.STATE["events"]), "last event": m.STATE["events"][-1] if m.STATE["events"] else None}  # fmt: skip

            folder = out / run
            result = kit.run(embodiment, {"left": [], "right": []}, folder, seed=0, director=director, extra=extra, limit_s=m.RUN_LIMIT_STEPS / hz, tail_s=1.0, sheet_frames=16, record=not check)
            rows = result["rows"]
            readings = {"report": result["report"], "worst IK": result["worst_ik"], "seconds": rows[-1]["t"] if rows else 0.0}
            if run in ("ring", "seat"):
                grips = []
                for k in range(1, GRIPS + 1):  # the ring's turn over each grip's turning moves
                    turning = [row for row in rows if row["left"] == f"unscrews {k}"]
                    if turning:
                        grips.append({"grip": k, "ring turned (deg)": round(360.0 * (turning[-1]["extra"]["turn (turns)"] - turning[0]["extra"]["turn (turns)"]), 1),
                                      "ring from left grasp at start, end (mm)": [turning[0]["extra"]["ring from left grasp (mm)"], turning[-1]["extra"]["ring from left grasp (mm)"]],
                                      "ring on after": turning[-1]["extra"]["ring on"]})  # fmt: skip
                kept = [row for row in rows if row["left"] in ("away with it", "keeps it")]
                readings.update({"per grip": grips, "events": result["report"]["events"] if result["report"] else None,
                                 "ring from left grasp while drawn away (mm, first and last)": [kept[0]["extra"]["ring from left grasp (mm)"], kept[-1]["extra"]["ring from left grasp (mm)"]] if kept else None,
                                 "operator from right grasp (mm, at the hand-over and at the end)": [next((row["extra"]["operator from right grasp (mm)"] for row in rows if row["left"] == "grip 1"), None), rows[-1]["extra"]["operator from right grasp (mm)"]]})  # fmt: skip
            if run == "seat":
                arrived = [row for row in rows if row["right"] == "arrives"]
                readings["arrival (last step of 'arrives')"] = arrived[-1]["extra"]["seat"] if arrived else None
                readings["end"] = {k: rows[-1]["extra"][k] for k in ("seated", "seat (mm)", "seat")}
            if run in ("full", "fingers", "search", "seated"):
                readings["plan failed"] = state.get("error")
                readings["events"] = result["report"]["events"] if result["report"] else None
                readings["plan"] = {"screw-on grips": getattr(plan, "ring_grips", None), "tightening grips": getattr(plan, "tighten_grips", None), "search turned (deg)": getattr(plan, "search_turned", None), "insert steps": getattr(plan, "insert_steps", None), "aim misses (mm)": plan.aim_misses}

                def span(label: str) -> list:
                    return [row for row in rows if row["left"] == label]

                per = []
                for k in range(1, ON_GRIPS_MOST + 1):
                    rs = span(f"screws on {k}")
                    if rs:
                        a, b = rs[0]["extra"], rs[-1]["extra"]
                        per.append({"grip": k, "ring along (mm)": [a["ring along (mm)"], b["ring along (mm)"]], "ring off axis (mm, deg)": [a["ring off axis (mm, deg)"], b["ring off axis (mm, deg)"]],
                                    "ring on": [a["ring on"], b["ring on"]], "turned (deg)": round(360.0 * (b["turn (turns)"] - a["turn (turns)"]), 1), "seat (mm)": b["seat (mm)"]})  # fmt: skip
                readings["per screw-on grip"] = per
                if rows:
                    readings["end"] = {k: rows[-1]["extra"][k] for k in ("seated", "seat (mm)", "gasket gap (mm)", "seat", "ring on", "ring along (mm)", "tight", "lock (deg)", "past touch (deg)", "hands from the parts (mm)")}
            if run in ("tighten", "full"):  # per tightening grip: how far the ring went on and past its touch; the most it was turned (the lock) is what the pinch held
                per = []
                for k in range(1, ON_GRIPS_MOST + 1):
                    rs = [row for row in rows if row["left"] == f"tightens {k}"]
                    if rs:
                        a, b = rs[0]["extra"], rs[-1]["extra"]
                        per.append({"grip": k, "ring along (mm)": [a["ring along (mm)"], b["ring along (mm)"]], "tight": b["tight"],
                                    "past touch (deg) first, last, most": [a["past touch (deg)"], b["past touch (deg)"], max(r["extra"]["past touch (deg)"] for r in rs)],
                                    "lock (deg)": b["lock (deg)"], "seat (mm)": b["seat (mm)"], "grip opening": b["grips"][0]})  # fmt: skip
                readings["per tightening grip"] = per
                lock = rows[-1]["extra"]["lock (deg)"] if rows else 0.0
                readings["most turned past the touch (deg)"] = lock
                readings["torque the pinch carried (N m, at the scene's 2.05 N m/rad)"] = round(m.TIGHT_READ * math.radians(lock), 3)
                readings["events"] = result["report"]["events"] if result["report"] else None
                if rows:
                    readings["end"] = {k: rows[-1]["extra"][k] for k in ("seated", "seat (mm)", "gasket gap (mm)", "ring on", "ring along (mm)", "tight", "lock (deg)", "past touch (deg)", "hands from the parts (mm)")}
            (folder / "readings.json").write_text(json.dumps(readings, indent=1, default=str))
            print(f"READINGS {run}: {json.dumps(readings, default=str)}", flush=True)
        except Exception as error:
            import traceback

            traceback.print_exc()
            print(f"RUN {run} FAILED: {error!r}", flush=True)
            failed.append(run)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
