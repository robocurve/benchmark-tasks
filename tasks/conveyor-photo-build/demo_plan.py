"""The scripted demo: open loop, both arms, from the fixed layout. NumPy only.

The left arm, upstream, catches the red cube and the yellow beam as they ride past and parks them on the
staging mat, lets the square post, the orange beam and the blue cube ride on, then catches the first tall
green cylinder and stands it on the left mark. The right arm, downstream, lets the short cylinder pass, catches
the second tall cylinder and stands it on the right mark, then moves clear. The left arm then sets the yellow
beam across the cylinders and the red cube on the middle of the beam.

A part is caught from the running belt by parking the open fingers across the belt at grip height where the
part will be, letting it ride in between them, and closing while the hand moves along with the belt.
"""
from __future__ import annotations

import math
import os

from yam_arena.demo import FINGER_OPEN, Waypoint

from layout import (BELT_TOP, BELT_X, BEAM_TOP, GATE_X, GATE_Y, MARK_TOP, MARKS, PARTS, POST_TOP, SPEED,
                    STAGE_TOP, TOP, start_y)

YAW = {'left': -math.pi / 2, 'right': math.pi / 2}  # fingers close along world x, across the belt
TRAVEL = TOP + 0.165
SQUEEZE = 0.008  # close to this much narrower than the part: about 20 N per finger, under the 50 N cap
RELEASE_GAP = 0.004  # a part is let go with its underside this far above where it will stand
GRIP_BELOW_TOP = {'cap': 0.012, 'beam': 0.010, 'post_a': 0.016, 'post_b': 0.016, 'square_post': 0.016}  # TCP below top
STAGED = {'cap': (0.16, 0.24), 'beam': (0.245, 0.32)}  # where the demo parks parts on the staging mat
# When each arm starts to close on a part riding past (s): the left arm 0.15 to 0.45 m left of the midline,
# the right arm 0.10 to 0.45 m right of it.
CATCH = {'cap': 3.0, 'beam': 13.0, 'post_a': 23.8, 'post_b': 48.75, 'square_post': 20.0}
KEPT = (0.20, -0.30)  # keep_extra: where the right arm sets the square post down
RIGHT_CLEAR = (0.15, -0.25, TRAVEL)


def finger_for(name: str) -> float:
    """Finger joint target that closes the pads SQUEEZE narrower than the part's width across the belt."""
    width = PARTS[name][0][0]
    return -max(0.0, width - SQUEEZE) / 2


def grip_z(name: str, bottom: float) -> float:
    """TCP height to hold a part whose underside is at ``bottom``."""
    return bottom + PARTS[name][0][2] - GRIP_BELOW_TOP[name]


def part_y(name: str, t: float) -> float:
    """An untouched part's centre along the belt at time t."""
    return start_y(name) - SPEED * t


class Planner:
    """Waypoints for one arm, with the plan's clock."""

    def __init__(self, arm: str):
        self.arm, self.yaw = arm, YAW[arm]
        self.out: list[Waypoint] = []
        self.t = 0.0

    def go(self, duration, pos, finger, label):
        self.out.append(Waypoint(float(duration), tuple(float(v) for v in pos), float(finger), label, self.yaw))
        self.t += duration

    def hold_until(self, t, label):
        """Stay where the last waypoint left the hand until time t."""
        last = self.out[-1] if self.out else None
        if t - self.t > 1e-6:
            if last is None:
                raise ValueError('hold_until needs a previous waypoint')
            self.go(t - self.t, last.pos, last.finger, label)

    def catch(self, name: str, at: float, hold_on_belt: float = 0.0):
        """Catch ``name`` off the belt, closing on it from ``at`` (s); ends lifted to travel height.
        ``hold_on_belt`` (s): a baseline that first holds the closed hand still, pinning the part on the belt."""
        y = part_y(name, at)
        z = grip_z(name, BELT_TOP)
        hold = finger_for(name)
        if self.t + 1.6 + 1.2 > at + 1e-6:
            raise ValueError(f'the {self.arm} arm is not free in time to catch the {name} at {at} s '
                             f'(free at {self.t:.2f} s)')
        self.go(1.6, (BELT_X, y, TRAVEL), FINGER_OPEN, f'over the belt for the {name}')
        self.hold_until(at - 1.2, 'wait for it')
        self.go(1.2, (BELT_X, y, z), FINGER_OPEN, 'down across the belt')
        y -= SPEED * 0.6
        self.go(0.6, (BELT_X, y, z), hold, f'close on the {name} riding in')
        if hold_on_belt:
            self.go(hold_on_belt, (BELT_X, y, z), hold, 'hold it still on the running belt')
        else:
            y -= SPEED * 0.4  # keep moving with the belt while lifting it off
        self.go(0.4, (BELT_X, y, z + 0.02), hold, 'lift off the belt')
        self.go(0.8, (BELT_X - 0.03, y, TRAVEL), hold, 'up')

    def pick(self, name: str, xy, bottom: float):
        z = grip_z(name, bottom)
        self.go(1.6, (*xy, TRAVEL), FINGER_OPEN, f'over the {name}')
        self.go(1.0, (*xy, z + 0.02), FINGER_OPEN, 'down')
        self.go(0.6, (*xy, z), FINGER_OPEN, 'down slowly')
        self.go(0.6, (*xy, z), finger_for(name), f'close on the {name}')
        self.go(1.0, (*xy, TRAVEL), finger_for(name), 'lift')

    def place(self, name: str, xy, bottom: float, label: str):
        z = grip_z(name, bottom + RELEASE_GAP)
        hold = finger_for(name)
        self.go(1.8, (*xy, TRAVEL), hold, f'carry the {name} {label}')
        self.go(1.0, (*xy, z + 0.02), hold, 'lower')
        self.go(0.8, (*xy, z), hold, 'lower slowly')
        self.go(0.3, (*xy, z), hold, 'settle')
        self.go(0.5, (*xy, z), FINGER_OPEN, 'let go')
        self.go(0.8, (*xy, TRAVEL), FINGER_OPEN, 'up')


VARIANTS = ('demo', 'no_staging', 'hold_on_belt', 'keep_extra')


def plans(variant: str = 'demo') -> tuple[Planner, Planner]:
    """The full-score demo, or a baseline that shows one rule at work:

    - ``no_staging``: build in build order only, taking each part when it is needed; the cube and the beam
      ride off before the cylinders stand, so only the two cylinders count (0.4).
    - ``hold_on_belt``: the demo, but the right arm first holds the second tall cylinder still on the running
      belt for 2.5 s before lifting it off; the line jams and the trial scores 0 although the model gets built.
    - ``keep_extra``: the demo, but the right arm also takes the green square post off the belt and sets it on
      the table; the model is complete, and keeping the extra part costs 0.25 (0.75, no success).
    """
    if variant not in VARIANTS:
        raise ValueError(f'unknown plan {variant!r}; expected one of {VARIANTS}')
    right = Planner('right')
    right.go(2.0, RIGHT_CLEAR, FINGER_OPEN, 'stand by, clear of the belt')
    if variant == 'keep_extra':
        right.hold_until(CATCH['square_post'] - 2.8 - 0.5, 'wait for the square post')
        right.catch('square_post', CATCH['square_post'])
        right.place('square_post', KEPT, TOP, 'to the table')
        right.go(1.5, RIGHT_CLEAR, FINGER_OPEN, 'clear')
    right.hold_until(CATCH['post_b'] - 2.8 - 0.5, 'wait for the second tall cylinder')
    right.catch('post_b', CATCH['post_b'], hold_on_belt=2.5 if variant == 'hold_on_belt' else 0.0)
    right.place('post_b', MARKS[1], MARK_TOP, 'to the right mark')
    right.go(1.5, RIGHT_CLEAR, FINGER_OPEN, 'clear of the gate')

    left = Planner('left')
    if variant == 'no_staging':
        left.go(2.0, (0.20, 0.30, TRAVEL), FINGER_OPEN, 'wait for a post: nothing else can go on yet')
        left.catch('post_a', CATCH['post_a'])
        left.place('post_a', MARKS[0], MARK_TOP, 'to the left mark')
        left.go(1.5, (0.20, 0.30, TRAVEL), FINGER_OPEN, 'the cube and the beam have ridden off')
        return left, right
    left.catch('cap', CATCH['cap'])
    left.place('cap', STAGED['cap'], STAGE_TOP, 'to the staging mat')
    left.catch('beam', CATCH['beam'])
    left.place('beam', STAGED['beam'], STAGE_TOP, 'to the staging mat')
    left.catch('post_a', CATCH['post_a'])
    left.place('post_a', MARKS[0], MARK_TOP, 'to the left mark')
    left.pick('beam', STAGED['beam'], STAGE_TOP)
    left.hold_until(right.t, 'hold the beam until the right arm is clear')
    left.place('beam', (GATE_X, GATE_Y), POST_TOP, 'across the posts')
    left.pick('cap', STAGED['cap'], STAGE_TOP)
    left.place('cap', (GATE_X, GATE_Y), BEAM_TOP, 'onto the middle of the beam')
    left.go(1.5, (0.20, 0.30, TRAVEL), FINGER_OPEN, 'clear of the gate')
    return left, right


def demo() -> dict[str, list[Waypoint]]:
    """The plan ``yam_scripted`` plays: the full-score demo, or a baseline named by CONVEYOR_PLAN
    (``idle`` parks both arms clear of the belt and holds them there while every part rides off)."""
    variant = os.environ.get('CONVEYOR_PLAN', 'demo')
    if variant == 'idle':
        park = {'left': (0.15, 0.30, TRAVEL), 'right': RIGHT_CLEAR}
        return {arm: [Waypoint(2.0, park[arm], FINGER_OPEN, 'park', YAW[arm]),
                      Waypoint(100.0, park[arm], FINGER_OPEN, 'hold', YAW[arm])] for arm in park}
    left, right = plans(variant)
    return {'left': left.out, 'right': right.out}
