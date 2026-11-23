"""Full-score demonstration: one hand holds the pillow down at the frame, the other pushes the drawer in.

The demonstration reads the robot's own joint and gripper readings and, as the review rules allow for a
scripted demonstration, simulator state: how far the drawer is open and how high each of the pillow's 16 pads
stands. It changes nothing in the simulation. The pillow goes down only where the fingertips press it, and
the drawer moves only because the other hand's fingertips push its front.

Plan. The right hand, closed, sets its fingertips against the drawer front. The left hand, open, hangs over
the pillow 22 mm in front of the edge the pillow catches on (the front lower edge of the frame's top), one
fingertip over each column of pads. Then, for each part of the pillow that stands above the opening:

1. push the drawer in until that part is at the frame (it catches there if it gets that far);
2. press it down with the left fingertips until it is 5 mm below the opening;
3. keep it there and push the drawer in 12 mm, so the frame's top takes over holding it down;
4. lift the left hand out through the gap between the frame and the drawer front.

When nothing stands above the opening any more, push the drawer shut, take both hands away and wait.

``variant`` gives two controls that must not reach full score: ``"no_press"`` only pushes; ``"no_hold"``
presses each part but lifts the hand before pushing.
"""

from inspect_robots import Action, ActionChunk
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, READY

from . import layout
from .fixed_scene import ARMS, BASES, CONTROL_HZ

PRESS, PUSH = ARMS                      # the left hand presses, the right hand pushes
OPEN = float(FINGER_OPEN)
TIP = float(PAD_TIP_PAST_TCP)           # the fingertips end this far beyond the gripper's centre point
DT = 1. / CONTROL_HZ
PITCH = {PRESS: np.radians(45.), PUSH: np.radians(40.)}     # each hand points forward and this far from straight down

PRESS_Y = .030                          # the pressing hand's middle: one fingertip on each column, clear of the other arm
PRESS_X = layout.RAIL_X - .022          # fingertips this far in front of the frame while pressing
CLEAR_Z = .200                          # fingertip height that clears the pillow left alone (its top is at 172 mm)
PRESS_BELOW = .005                      # press until the part is this far below the opening
PRESS_SLACK = .012                      # and ease off if it is pressed further than this
PRESS_FLOOR = layout.RAIL_Z - .040      # never press deeper than this
PRESS_SPEED = .030                      # m/s
AT_FRAME = .004                         # "at the frame": leading edge within this of it
FEED = .012                             # how far a held-down part is pushed in under the frame's top
CAPTURED = .007                         # a part this far under the frame's top is held by it
PUSH_Y, PUSH_Z = -.150, .075            # where the right fingertips meet the drawer front
PUSH_SPEED = .030                       # m/s
PUSH_LEAD = .035                        # the command may run this far ahead of the drawer front (pushing force)
OVER = (.050, .220)                     # the pushing hand first goes over the drawer: this far behind its front, this high
ABOVE = (-.030, .172)                   # then back over the front panel to this point in front of it, and down
STANDOFF = .015                         # fingertips this far from the drawer front before the first push
BACK_OFF = .008                         # and this far from it to take the push off (the tip's corner leads by 5 mm)
STALL_LEAD = .015                       # pushing this hard ...
STALL_STEPS = 16                        # ... for this long without the drawer moving: something holds it
PARK = {PRESS: (.250, .130, .250), PUSH: (.220, -.150, .220)}       # fingertips, after the drawer is shut
# Joint angles near the two working poses (from a reach study); they seed the inverse kinematics.
SEED = {PRESS: (-.9063, 1.8682, 1.7219, -.8717, -.5904, -.7342), PUSH: (.9858, 1.1511, 1.1918, -1.1776, .5655, .8579)}


def hand_rotation(pitch):
    """Gripper orientation: pointing forward (+x) and down, ``pitch`` from straight down, fingers side by side
    across the drawer (they open and close along y)."""
    approach = np.array((np.sin(pitch), 0., -np.cos(pitch)))
    across = np.array((0., -1., 0.))
    return np.column_stack((across, np.cross(approach, across), approach))


ROTATION = {arm: hand_rotation(PITCH[arm]) for arm in ARMS}
QUAT = {arm: ik.matrix_to_quat_xyzw(ROTATION[arm]) for arm in ARMS}


def fingertips(joint_pos, arm):
    """Middle of the two fingertips' ends, from joint readings (tabletop frame)."""
    start = 7 * ARMS.index(arm)
    point, rotation = ik.tcp_pose(np.asarray(joint_pos, dtype=float)[start:start + 6], BASES[arm])
    return point + TIP * rotation[:, 2]


class DrawerReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self, variant="full"):
        if variant not in ("full", "no_press", "no_hold"):
            raise ValueError(variant)
        self.variant = variant
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.aperture = {PRESS: OPEN, PUSH: OPEN}
        self.tip = {}                                   # commanded fingertip points
        self.trim_x = 0.                                # correction of the pressing hand's reach
        self.still = 0                                  # steps the drawer has not moved under a firm push
        self.last_open = None
        self.worst_ik = 0.
        self.tick = 0
        self.phases = []
        self.log = {"variant": variant, "cycles": []}
        self.report = self.log

    # -- readings -------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        if state is not None:
            self.state = state

    def measured_tip(self, arm):
        return fingertips(self.measured, arm)

    def opening(self, arm):
        return float(self.measured[7 * ARMS.index(arm) + 6])

    def drawer_open(self):
        return float(self.state["open"])

    def front_x(self):
        return layout.RAIL_X - self.drawer_open()

    def lead(self, row):
        return float(self.state["pad_lead"][row])

    def row_top(self, row):
        return float(np.max(self.state["pad_top"][row]))

    def next_row(self):
        """The part of the pillow nearest the frame that stands above the opening and is not yet under it."""
        for row in reversed(layout.PROUD_ROWS):
            if self.lead(row) < layout.RAIL_X + CAPTURED:
                return row
        return None

    # -- motion ---------------------------------------------------------------------------
    def solve(self, arm, point, seed=None, iters=40):
        target = np.asarray(point, dtype=float) - TIP * ROTATION[arm][:, 2]
        joints, error, _ = ik.solve(target, QUAT[arm], self.joints[arm] if seed is None else seed, BASES[arm],
                                    iters=iters)
        self.worst_ik = max(self.worst_ik, error)
        return joints

    def emit(self):
        for arm, point in self.tip.items():
            self.joints[arm] = self.solve(arm, point)
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.aperture[a] / OPEN, 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def glide(self, seconds, targets, aperture):
        """Move in joint space to the given fingertip points (smooth start and stop)."""
        goal = {arm: self.solve(arm, point, seed=np.asarray(SEED[arm]), iters=400) for arm, point in targets.items()}
        start = {arm: self.joints[arm].copy() for arm in targets}
        jaws = {arm: self.aperture[arm] for arm in aperture}
        self.tip = {arm: point for arm, point in self.tip.items() if arm not in targets}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm in targets:
                self.joints[arm] = start[arm] + alpha * (goal[arm] - start[arm])
            for arm, value in aperture.items():
                self.aperture[arm] = jaws[arm] + alpha * (value - jaws[arm])
            yield self.emit()
        for arm, point in targets.items():
            self.tip[arm] = np.asarray(point, dtype=float).copy()

    def line(self, seconds, targets):
        """Move fingertips in straight lines (smooth start and stop)."""
        start = {arm: self.tip[arm].copy() for arm in targets}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm, goal in targets.items():
                self.tip[arm] = start[arm] + alpha * (np.asarray(goal, dtype=float) - start[arm])
            yield self.emit()

    def run(self, seconds, *, push_to=None, press=None, done=None, settle=3):
        """Servo for up to ``seconds``: push the drawer toward an opening and/or hold a row pressed. Stops early
        once ``done()`` has held for ``settle`` steps. Returns True if it did."""
        count = 0
        self.still = 0
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            if push_to is not None:
                self._push_step(push_to)
            if press is not None:
                self._press_step(press)
            yield self.emit()
            count = count + 1 if done is not None and done() else 0
            if count >= settle:
                return True
        return False

    def _push_step(self, target_open):
        front = self.front_x()
        error = self.drawer_open() - target_open
        tip = self.tip[PUSH]
        moved = self.last_open is None or abs(self.drawer_open() - self.last_open) > .0002
        self.still = 0 if moved or tip[0] < front + STALL_LEAD else self.still + 1
        self.last_open = self.drawer_open()
        if error > .0005:           # run the command ahead of the drawer front: the arm's springiness makes the force
            tip[0] = min(tip[0] + min(PUSH_SPEED * DT, .5 * error + .0005), front + PUSH_LEAD)
        else:                       # there: take the force off
            tip[0] = max(tip[0] - 2. * PUSH_SPEED * DT, min(tip[0], front - BACK_OFF))

    def _press_step(self, row):
        top = self.row_top(row)
        tip = self.tip[PRESS]
        if top > layout.RAIL_Z - PRESS_BELOW:
            tip[2] -= PRESS_SPEED * DT
        elif top < layout.RAIL_Z - PRESS_SLACK:
            tip[2] += .0005
        tip[2] = max(tip[2], PRESS_FLOOR)
        # The pillow sliding under the fingertips drags the hand toward the frame: hold its measured place.
        drift = float(self.measured_tip(PRESS)[0]) - PRESS_X
        self.trim_x = float(np.clip(self.trim_x - .2 * drift, -.015, .015))
        tip[0] = PRESS_X + self.trim_x

    def phase(self, name):
        self.phases.append((self.tick, name))

    def snapshot(self, **extra):
        """What the demonstration saw at this moment, for the run log (millimetres)."""
        row = {"tick": self.tick, "open_mm": round(1000 * self.drawer_open(), 1),
               "push_tip_command_mm": [round(1000 * float(v), 1) for v in self.tip[PUSH]],
               "push_tip_measured_mm": [round(1000 * float(v), 1) for v in self.measured_tip(PUSH)],
               "press_tip_command_mm": [round(1000 * float(v), 1) for v in self.tip[PRESS]],
               "press_tip_measured_mm": [round(1000 * float(v), 1) for v in self.measured_tip(PRESS)],
               "gripper_opening": [round(self.opening(arm), 3) for arm in ARMS]}
        row.update(extra)
        return row

    # -- the demonstration ------------------------------------------------------------------
    def sequence(self):
        self.phase("reach")
        front = layout.drawer_x(layout.OPEN_START)
        # The pushing hand comes in above the pillow, goes back over the drawer front and down in front of it.
        yield from self.glide(2.5, {PRESS: (PRESS_X, PRESS_Y, CLEAR_Z), PUSH: (front + OVER[0], PUSH_Y, OVER[1])},
                              {PUSH: 0.})
        yield from self.line(1., {PUSH: (front + ABOVE[0], PUSH_Y, ABOVE[1])})
        yield from self.line(1., {PUSH: (front - STANDOFF, PUSH_Y, PUSH_Z)})
        yield from self.run(.5)
        self.tip[PUSH][0] = self.front_x() - STANDOFF           # where the drawer front really is
        self.log["at_start"] = self.snapshot()
        for cycle in range(2 * len(layout.PROUD_ROWS)):
            row = self.next_row()
            if row is None:
                break
            entry = {"row": row, "proud_mm": round(1000 * layout.ROW_PROUD[row], 1)}
            self.phase(f"bring row {row} to the frame")
            target = self.drawer_open() - max(0., layout.RAIL_X - AT_FRAME / 2 - self.lead(row))
            entry["at_frame"] = yield from self.run(8., push_to=target, done=lambda: self.lead(row) >= layout.RAIL_X - AT_FRAME
                                                    or self._stalled())
            yield from self.run(.3, push_to=self.drawer_open() + .01)          # take the push off
            entry["before_press"] = self.snapshot(row_top_mm=round(1000 * self.row_top(row), 1),
                                                  lead_to_frame_mm=round(1000 * (layout.RAIL_X - self.lead(row)), 1))
            if self.variant != "no_press":
                self.phase(f"press row {row}")
                entry["pressed"] = yield from self.run(
                    4., press=row, done=lambda: self.row_top(row) <= layout.RAIL_Z - PRESS_BELOW)
                yield from self.run(.3, press=row)
                entry["after_press"] = self.snapshot(row_top_mm=round(1000 * self.row_top(row), 1),
                                                     both_pads_mm=[round(1000 * float(v), 1) for v in self.state["pad_top"][row]])
                if self.variant == "no_hold":
                    self.phase(f"lift off row {row}")
                    yield from self.line(.6, {PRESS: (PRESS_X, PRESS_Y, CLEAR_Z)})
            self.phase(f"feed row {row} under the frame")
            target = self.drawer_open() - (layout.RAIL_X + FEED - self.lead(row))
            holding = row if self.variant == "full" else None
            entry["fed"] = yield from self.run(6., push_to=target, press=holding,
                                               done=lambda: self.lead(row) >= layout.RAIL_X + CAPTURED + .002 or self._stalled())
            entry["fed"] = bool(self.lead(row) >= layout.RAIL_X + CAPTURED)
            yield from self.run(.3, push_to=self.drawer_open() + .01, press=holding)
            entry["after_feed"] = self.snapshot(row_top_mm=round(1000 * self.row_top(row), 1),
                                                lead_past_frame_mm=round(1000 * (self.lead(row) - layout.RAIL_X), 1))
            if self.variant == "full":
                self.phase(f"lift off row {row}")
                self.trim_x = 0.
                yield from self.line(.6, {PRESS: (PRESS_X, PRESS_Y, CLEAR_Z)})
            self.log["cycles"].append(entry)
            if not entry["fed"]:
                break                                           # caught on the frame: stop rather than force it
        self.phase("push shut")
        self.log["shut"] = yield from self.run(10., push_to=.0005, done=lambda: self.drawer_open() <= .002 or self._stalled())
        self.log["before_letting_go"] = self.snapshot()
        self.phase("hands away")
        yield from self.line(1., {PUSH: self.tip[PUSH] + (-.060, 0., .030), PRESS: (PRESS_X - .060, PRESS_Y, CLEAR_Z + .030)})
        yield from self.line(1.5, {arm: PARK[arm] for arm in ARMS})
        self.log["parked"] = self.snapshot()
        self.phase("wait")
        while True:
            yield self.emit()

    def _stalled(self):
        """The drawer has not moved under a firm push, or the push command is as far ahead of the drawer front
        as it may go: something holds the drawer."""
        return bool(self.still >= STALL_STEPS or self.tip[PUSH][0] >= self.front_x() + PUSH_LEAD - 1e-6)
