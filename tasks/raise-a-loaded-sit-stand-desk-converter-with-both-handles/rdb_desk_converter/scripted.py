"""Full-score demonstration: both hands take the handles, squeeze both paddles, lift the top along its arc past the
ninth notch, settle it back onto the notch, release both paddles in the same step, let go and move back.

The demonstration reads the robot's joint and gripper readings and, as the review rules allow for a scripted
demonstration, simulator state: the lift arm's angle and the two latches' heights. It changes nothing in the
simulation; the top moves only because the fingers push on the paddles and the work surface.

How a handle is held. The hand comes in level from the front (fingers pointing +x), turned so that the jaws
close vertically: the upper pad lands on the work surface beside the edge, the lower pad under the paddle. Closing
the jaws lifts the paddle 16 mm until it stops under the board, which pulls that side's pin out of its notch;
the pads then clamp the 33 mm stack (board + gap + paddle) with about 20 N each. Lifting goes through the lower
pad and the paddle; pressing down through the upper pad and the board.

Every motion that matters is closed on measured state: the hands are first trimmed onto the grasp points by
their joint readings (the arms sag), the lift's commanded arm angle never runs more than LEAD ahead of the
measured one, and the release waits until the measured angle sits on the ninth notch.

Controls for the dry run and for Isaac (``variant``): "one_paddle" (the right hand goes to its handle but never
squeezes) and "release_mismatch" (the left hand lets go of its paddle at setting 7.6 while the right lifts on).
"""

from inspect_robots import Action, ActionChunk
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from yam_arena import ik
from yam_arena.constants import READY

from . import layout as L
from .fixed_scene import ARMS, BASES, CONTROL_HZ

UP = np.array((0., 0., 1.))
GRIP_ROT = np.column_stack((UP, np.cross((1., 0., 0.), UP), (1., 0., 0.)))   # closing axis vertical, fingers +x
SEEDS = {"left": (.14, 1.69, .85, .84, .14, -1.57), "right": (-.14, 1.69, .85, .84, -.14, -1.57)}
FULL_GAP = .0939                 # jaw gap at command 1.0
OPEN_V = .80                     # 75 mm: the lower pad passes 5 mm under the released paddle
SQUEEZE_V = .024 / FULL_GAP      # 24 mm commanded on the 33 mm stack: about 20 N a pad
RELEASE_V = .80
PRE_BACK = .070                  # pre-grasp this far in front of the grasp point
SQUEEZED_M = L.PADDLE_TRAVEL - .0025       # latch at least this high: the pin is out
LIFT_RATE = .055                 # rad/s of arm angle commanded (about 2 cm/s along the path)
LEAD = .035                      # the command never runs more than this ahead of the measured angle (2 deg)
OVERSHOOT = 9.25                 # lift to this setting, then settle back onto the ninth notch
SETTLE = .0015                   # aim this far (rad) above the notch before releasing
STALL_S = 4.0
TRIM_MAX = np.array((.006, .015, .015))   # x, y, z: largest trim of a command beyond its goal (sag is about 2 mm)
UNLOAD = .006                    # rad of arm angle commanded above the measured while squeezing: takes the top's
                                 # weight off the pins (the user lifts slightly while squeezing)
MAX_JUMP = .15                   # rad: largest joint change one control step may command
PRESS = .028                     # open jaws: the upper pad is 21 mm above the board; this presses about 7 mm


class ConverterReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self, variant=None):
        self.variant = variant
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.jaw = dict.fromkeys(ARMS, 1.)
        self.target = {arm: None for arm in ARMS}
        self.offset = {arm: np.zeros(3) for arm in ARMS}     # commanded minus nominal grasp point (sag trim)
        self.theta_cmd = None
        self.worst_ik = 0.
        self.phases = []
        self.tick = 0
        self.log = {"events": [], "servo": [], "trim": []}
        self.released_early = False
        self.ik_refused = 0

    # -- readings ---------------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        if state is not None:
            self.state = state

    def hand(self, arm):
        i = 7 * ARMS.index(arm)
        return ik.tcp_pose(self.measured[i:i + 6], BASES[arm])

    def opening(self, arm):
        return float(self.measured[7 * ARMS.index(arm) + 6])

    def theta(self):
        return float(self.state["theta"])

    def latch(self, arm):
        return float(self.state["latch"][arm])

    def event(self, text, **fields):
        self.log["events"].append({"tick": self.tick, "event": text, **fields})

    def phase(self, name):
        self.phases.append((self.tick, name))

    # -- motion -----------------------------------------------------------------------------------
    def emit(self):
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.jaw[a], 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def solve(self, arm, point, rotation, seed=None, iters=40):
        q, error, _ = ik.solve(point, ik.matrix_to_quat_xyzw(rotation),
                               self.joints[arm] if seed is None else seed, BASES[arm], iters=iters)
        self.worst_ik = max(self.worst_ik, error)
        return q

    def command(self, arm, point, rotation=GRIP_ROT):
        target = (np.asarray(point, dtype=float).copy(), np.asarray(rotation, dtype=float).copy())
        q = self.solve(arm, *target)
        jump = float(np.max(np.abs(q - self.joints[arm])))
        if jump > MAX_JUMP:
            # an IK branch change (run 3: the last retreat step swung joint 1 by 0.52 rad and the arm into the
            # converter); keep the last safe joints instead
            self.ik_refused += 1
            if self.ik_refused <= 5:
                self.event("ik jump refused", arm=arm, jump_rad=round(jump, 3))
            return
        self.target[arm] = target
        self.joints[arm] = q

    def hold(self, seconds):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield self.emit()

    def glide(self, seconds, goals):
        start = {a: self.joints[a].copy() for a in goals}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            s = step / steps
            s = s * s * (3. - 2. * s)
            for a, goal in goals.items():
                self.joints[a] = start[a] + s * (np.asarray(goal) - start[a])
            yield self.emit()
        for a in goals:
            self.target[a] = None

    def lines(self, seconds, goals):
        """Move each named arm's commanded gripper pose in a straight line (and steady turn) to its goal."""
        starts = {}
        for a, (p, r) in goals.items():
            if self.target[a] is None:
                self.target[a] = self.hand(a)
            starts[a] = self.target[a]
        turns = {a: Slerp([0., 1.], Rotation.from_matrix(np.stack((starts[a][1], goals[a][1])))) for a in goals}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            s = step / steps
            s = s * s * (3. - 2. * s)
            for a, (p, r) in goals.items():
                self.command(a, starts[a][0] + s * (np.asarray(p) - starts[a][0]), turns[a](s).as_matrix())
            yield self.emit()

    def jaws(self, seconds, values):
        start = dict(self.jaw)
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            for a, v in values.items():
                self.jaw[a] = start[a] + step / steps * (v - start[a])
            yield self.emit()

    def trim(self, goals, repeats=3, limit=.03):
        """Shift each arm's command until its gripper, by joint readings, is at its goal point."""
        for _ in range(repeats):
            moves = {}
            for a, point in goals.items():
                p, r = self.target[a] if self.target[a] is not None else self.hand(a)
                shift = np.clip(np.asarray(point) - self.hand(a)[0], -.008, .008)
                # never command more than TRIM_MAX past the goal: a hand that stops short is blocked by something,
                # and pushing on (run 2 went to 24 mm and 100 N) only loads the locks
                command = np.asarray(point) + np.clip(p + shift - point, -TRIM_MAX, TRIM_MAX)
                if np.linalg.norm(command - point) <= limit:
                    moves[a] = (command, r)
            if not moves:
                break
            yield from self.lines(.35, moves)
            yield from self.hold(.15)
            self.log["trim"].append({"tick": self.tick, **{a: [round(1000 * float(v), 2) for v in
                                                              self.hand(a)[0] - goals[a]] for a in goals}})

    def grasp_goal(self, arm, angle):
        return L.grasp_point(arm, angle)

    # -- the task ---------------------------------------------------------------------------------
    def sequence(self):
        yield from self.hold(.2)
        a0 = self.theta()
        self.phase("reach")
        # joint-space glide to a point 40 mm above the pre-grasp (a joint-space path passes close over the tray),
        # then straight down to it
        high = {a: self.solve(a, self.grasp_goal(a, a0) - (PRE_BACK, 0., -.040), GRIP_ROT, seed=SEEDS[a], iters=300)
                for a in ARMS}
        self.jaw = dict.fromkeys(ARMS, OPEN_V)
        yield from self.glide(3.0, high)
        yield from self.lines(1.0, {a: (self.grasp_goal(a, a0) - (PRE_BACK, 0., 0.), GRIP_ROT) for a in ARMS})
        yield from self.hold(.3)
        yield from self.trim({a: self.grasp_goal(a, a0) - (PRE_BACK, 0., 0.) for a in ARMS})
        self.phase("approach")
        goals = {a: (self.grasp_goal(a, a0), GRIP_ROT) for a in ARMS}
        yield from self.lines(1.6, goals)
        yield from self.hold(.3)
        yield from self.trim({a: self.grasp_goal(a, a0) for a in ARMS})
        for a in ARMS:
            self.offset[a] = self.target[a][0] - self.grasp_goal(a, a0)
        self.phase("squeeze")
        squeezers = [a for a in ARMS if not (self.variant == "one_paddle" and a == "right")]
        for attempt in range(1, 3):
            yield from self.jaws(1.0, {a: SQUEEZE_V for a in squeezers})
            yield from self.hold(.6)
            out = {a: self.latch(a) for a in ARMS}
            if not all(out[a] >= SQUEEZED_M for a in squeezers):
                # lift a little while squeezing, so the top's weight is off the pins, and squeeze on
                self.follow(self.theta() + UNLOAD)
                yield from self.hold(.6)
                out = {a: self.latch(a) for a in ARMS}
            self.event("squeeze", attempt=attempt, latch_mm={a: round(1000 * v, 2) for a, v in out.items()},
                       opening={a: round(self.opening(a), 3) for a in ARMS},
                       hand_minus_grasp_mm={a: [round(1000 * float(v), 1) for v in
                                                self.hand(a)[0] - self.grasp_goal(a, self.theta())] for a in ARMS})
            if all(out[a] >= SQUEEZED_M for a in squeezers):
                break
            # a paddle did not come up: open, re-centre on the measured grasp point, close again
            yield from self.jaws(.5, {a: OPEN_V for a in squeezers})
            a_now = self.theta()
            yield from self.trim({a: self.grasp_goal(a, a_now) for a in squeezers})
        yield from self.lift()
        yield from self.settle()
        yield from self.release()
        self.phase("retreat")
        # back out at the release opening until the fingertips are clear of the paddle (the lower pad stays 5 mm under
        # it), then up, and stay there (run 3: a further step back changed IK branch); no rise while a finger is under
        # a paddle, and no joint-space glide back to
        # READY, which would sweep the arms through the raised tray (at setting 9 it hangs at z 0.31 m)
        for step in ((-.068, 0., 0.), (0., 0., .050)):
            goals = {a: (self.target[a][0] + step, GRIP_ROT) for a in ARMS}
            yield from self.lines(1.0, goals)
        self.phase("hold")
        while True:
            yield self.emit()

    def follow(self, angle):
        """Command both hands to where their grasp points are with the arm at ``angle`` (plus the sag trim)."""
        for a in ARMS:
            self.command(a, self.grasp_goal(a, angle) + self.offset[a])

    def lift(self):
        self.phase("lift")
        goal = L.theta(OVERSHOOT)
        self.theta_cmd = self.theta()
        best, best_tick = self.theta(), self.tick
        while True:
            measured = self.theta()
            if measured > best + np.radians(.5):
                best, best_tick = measured, self.tick
            if measured >= goal - np.radians(.8):        # run 3: the compliant arms held it 0.6 deg under the command
                self.event("lift reached", setting=round(float(L.setting_of(measured)), 3))
                break
            if (self.tick - best_tick) > STALL_S * CONTROL_HZ:
                self.event("lift stalled", setting=round(float(L.setting_of(measured)), 3),
                           latch_mm={a: round(1000 * self.latch(a), 2) for a in ARMS})
                break
            if (self.variant == "release_mismatch" and not self.released_early
                    and L.setting_of(measured) >= 7.6):
                self.released_early = True
                self.jaw["left"] = RELEASE_V
                self.event("left paddle released early (control)", setting=round(float(L.setting_of(measured)), 3))
            self.theta_cmd = min(self.theta_cmd + LIFT_RATE / CONTROL_HZ, measured + LEAD, goal)
            if not (self.variant == "release_mismatch" and self.released_early):
                self.follow(self.theta_cmd)
            else:
                self.command("right", self.grasp_goal("right", self.theta_cmd) + self.offset["right"])
            self._servo_row("lift")
            yield self.emit()

    def settle(self):
        """Come back down (still squeezing) until the measured angle is just above the ninth notch."""
        self.phase("settle")
        goal = L.theta(L.GOAL_SETTING) + SETTLE
        for _ in range(round(4. * CONTROL_HZ)):
            measured = self.theta()
            if abs(measured - goal) < np.radians(.12):
                break
            step = np.clip(goal - self.theta_cmd, -.6 * LIFT_RATE / CONTROL_HZ, .6 * LIFT_RATE / CONTROL_HZ)
            self.theta_cmd = float(np.clip(self.theta_cmd + step, measured - LEAD, measured + LEAD))
            if not self.released_early:
                self.follow(self.theta_cmd)
            self._servo_row("settle")
            yield self.emit()
        yield from self.hold(.3)
        self.event("settled", setting=round(float(L.setting_of(self.theta())), 3))

    def release(self):
        """Both paddles released in the same step; then, if a pin has not dropped in, press the top down lightly."""
        self.phase("release")
        yield from self.jaws(.4, dict.fromkeys(ARMS, RELEASE_V))
        yield from self.hold(1.0)
        for attempt in range(1, 3):
            seated = {a: self.latch(a) <= .002 for a in ARMS}
            self.event("release", attempt=attempt, setting=round(float(L.setting_of(self.theta())), 3),
                       latch_mm={a: round(1000 * self.latch(a), 2) for a in ARMS})
            if all(seated.values()):
                return
            # press down through the upper pads (jaws open), as the maker's sheet says: "press down lightly"
            goals = {a: (self.target[a][0] - (0., 0., PRESS), GRIP_ROT) for a in ARMS}
            yield from self.lines(.6, goals)
            yield from self.hold(.5)
            goals = {a: (self.target[a][0] + (0., 0., PRESS), GRIP_ROT) for a in ARMS}
            yield from self.lines(.6, goals)
            yield from self.hold(.4)

    def _servo_row(self, stage):
        if self.tick % 2:
            return
        row = [self.tick, stage, round(float(np.degrees(self.theta_cmd)), 3), round(float(np.degrees(self.theta())), 3),
               round(1000 * self.latch("left"), 2), round(1000 * self.latch("right"), 2)]
        for a in ARMS:
            row.append([round(1000 * float(v), 1) for v in self.hand(a)[0] - self.grasp_goal(a, self.theta())])
        self.log["servo"].append(row)
