"""Full-score demonstration: the right hand pinches the G string's tuning button from above and turns it by rolling
its wrist; the left hand plucks the G string over the soundhole. After each pluck the tuner's reading is taken and
the button is turned by -error / 0.22 degrees (plus the gear's free play when the turn reverses), then the string is
plucked again, until the reading is within 2 cents. Then the final check: the right hand lets go of the button and
backs away, and the left hand plucks once more, so that the last pluck comes after the button's last movement (full
marks need it). If that reading is off (letting go turned the button), the hand takes the button again, corrects and
checks again. Both hands then withdraw.

The demonstration reads the robot's own joint and gripper readings and simulator state (a scripted demonstration
may; no policy does): each button's angle, each string's computed pitch (what the tuner shows while it reads), where
the G button sits in its free play, the G string's displacement and the pluck count. It changes nothing in the
simulation: the button turns only because the fingers turn it, the string moves only because a fingertip pushes it.

Right hand. The ukulele stands on its side, so the G button sticks straight up from the headstock's upper edge, its
shaft vertical; it is the one of the two buttons there nearer the nut. The hand comes down onto it from 30 mm above,
pointing straight down along the shaft, pads either side of the flat button, and turns it with joint 6 alone (the
roll about the hand's own axis, here the shaft's axis). Joint 6 gives about 224 degrees (its limits less a margin);
when it runs out the hand opens, rises 20 mm, rolls back by half a turn or more (a flat button looks the same every
half turn) and pinches again.

Left hand. Closed fingers pointing down and tipped 18 degrees toward the soundboard (straight down, the fingers'
upper part would rub the soundboard: the G string is 9 mm in front of it and the body's top edge 48 mm above it), the
fingertip just over the G string where it crosses the soundhole. It comes there, and leaves at the end, through a
waypoint 40 mm in front of the string and 40 mm above the body's top edge, moving in straight lines below that edge
(a joint-space move from the ready pose would bring the fingertips down on the body's edge). It presses the string
down until it is 4.5 mm out (read from the state: the arm gives under the string's spring), then slides toward the
robot so the string slips off the fingertip and springs back, then lifts.

``RDB_UKULELE_DEMO`` (or ``variant``) selects a control: "wrong_button" turns the C string's button (beside the G
string's on the same edge, farther from the nut) by the G string's error instead, once, and must score 0; "hold_on"
tunes but never lets go of the button and must score 0.75 at the step limit; "blind" tunes from the simulator's pitch
without ever plucking, lets go and withdraws, and must score 0.75 (in tune, but no pluck after the last turn).
"""

import os

from inspect_robots import Action, ActionChunk
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, READY

from . import fixed_scene as fixture
from . import layout as L
from . import tuning
from .fixed_scene import ARMS, CONTROL_HZ

OPEN = float(FINGER_OPEN)
DT = 1. / CONTROL_HZ
TIP = float(PAD_TIP_PAST_TCP)          # the pad tips are this far past the tool point
FACE = .004                            # closed fingers: each pad's outer face is 4 mm from the tool axis at the tips
                                       # (measured on the finger hulls)
BASES = {arm: np.asarray(fixture.BASES[arm], dtype=float) for arm in ARMS}
SIDE = {"left": 1., "right": -1.}
SEEDS = [np.array(s, dtype=float) for s in (      # starting guesses for inverse kinematics
    (0, 1, 1, -.6, 0, 0), (-.35, 2.2, .8, 1.4, 1.2, -1.57), (.35, 2.2, .8, 1.4, -1.2, 0), (0, 2.1, 1.0, -.4, 1.5, -.1),
    (.4, 2.5, 1.4, -1.6, 1.1, 0), (0, 1.6, 1.3, -1.2, 0, 0))]
SOLVE_ITERS = 150                      # multi-seed solves (one is about 0.4 s)

# -- the G button (shaft vertical: the hand points down along it) --------------------------------------------------
AXIS = np.array((0., 0., -1.))          # the right hand's pointing direction: down the button's shaft
SHAFT = np.array(fixture.direction_to_station(L.button_axis("g")))     # the button joints' axis on the table (+z)
TIPS_IN = L.BUTTON_LEN - .0005          # pad tips this far down from the button's top (not onto the shaft)
APPROACH_BACK = .030                    # the hand comes down along the shaft from this far above ...
REGRIP_BACK = .020                      # ... rises this far to roll back between strokes (pad tips 8.5 mm clear) ...
RELEASE_BACK = .030                     # ... and this far when it lets go
BUTTON_APPROACH = -.010                 # pads 20 mm apart going on and coming off (the C button is 37 mm away)
BUTTON_PINCH = 0.                       # fully closed: each pad 3 mm inside the 6 mm button, about 15 N a finger
HELD_M = .0045                          # pads stopped at least this far apart: the button is between them
STROKE = (np.radians(-150.), np.radians(150.))      # joint 6 is used within this, and within its limits less ...
LIMIT_MARGIN = np.radians(8.)                       # ... this margin
MIN_ROOM = np.radians(4.)               # regrip when less roll than this is left
TURN_RATE = np.radians(120.)            # fastest turn of the button
ROLL_BACK_RATE = np.radians(360.)
TOLERANCE_DEG = .5                      # the button within this of where it was meant to go
DONE_CENTS = 2.                         # stop when the tuner reads within this
MAX_CYCLES = 8
# -- plucking: press the G string down, slide off toward the robot -------------------------------------------------
PLUCK_TILT = np.radians(18.)            # fingers tipped this far toward the soundboard from straight down
PLUCK_A = np.array((np.sin(PLUCK_TILT), 0., -np.cos(PLUCK_TILT)))
PLUCK_C = -np.array((np.cos(PLUCK_TILT), 0., np.sin(PLUCK_TILT)))  # closing across: the fingers' thin side to the soundboard
TIP_PAST_STRING = .001                  # the fingertip's middle this far past the string toward the soundboard
PLUCK_GAP = .002                        # the press starts with the fingertip this far above the string
PLUCK_WANT = .0045                      # press the stretch this far down (2 mm counts as a pluck)
PLUCK_MAX = .022                        # the command goes at most this far down (the arm gives under the string's spring)
PLUCK_RATE = .020                       # m/s of the command while pressing
PLUCK_SLIDE = .014                      # then toward the robot this far ...
PLUCK_SLIDE_S = .15                     # ... this quickly, so the string slips off the fingertip
PLUCK_LIFT = .025
HOVER = .030                            # wait this far above the plucking height (below the body's upper edge)
CLEAR_FRONT = .040                      # the left hand comes to the string and leaves it through a point this far in
CLEAR_ABOVE = .040                      # front of it and this far above the body's upper edge: a joint-space move
                                        # from the ready pose would bring the fingertips down onto that edge


def grip_frame(approach, closing):
    """Gripper rotation (columns: closing direction, pad width, approach) from two directions."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float)
    c = c - a * (c @ a)
    c /= np.linalg.norm(c)
    return np.column_stack((c, np.cross(a, c), a))


def button_rotation(psi):
    """Hand on the G button: pointing down its shaft, pads closing along (-sin psi, cos psi, 0). At rest the button's
    thickness lies along y (psi 0); it turns about +z with its joint, so psi equal to the joint angle (mod 180
    degrees) has the pads square to it."""
    return grip_frame(AXIS, (-np.sin(psi), np.cos(psi), 0.))


def finger_roll(rotation):
    """The psi of a hand rotation pointing down: the closing direction's angle about +z from +y."""
    c = np.asarray(rotation)[:, 0]
    return float(np.arctan2(-c[0], c[1]))


PLUCK_ROT = grip_frame(PLUCK_A, PLUCK_C)


class TuningReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self, variant=None):
        self.variant = variant or os.environ.get("RDB_UKULELE_DEMO", "full")
        self.button = "c" if self.variant == "wrong_button" else L.TUNED
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float).copy() for arm in ARMS}
        self.aperture = dict.fromkeys(ARMS, OPEN)
        self.offset = {arm: np.zeros(3) for arm in ARMS}     # command minus wanted tool point (sag), from settling
        self.left_cmd = None
        self.grip_cmd = None
        self.j6 = self.psi = None
        self.s6 = 1.
        self.holding = False                                   # the right hand is closed on the button
        self.worst_ik = 0.
        self.j6_shift = 0.
        self.phases = []
        self.tick = 0
        self.regrips = 0
        self.cycles = 0
        self.log = {"events": [], "variant": self.variant}
        self.done = False

    # -- readings -------------------------------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        self.state = state

    def actual(self, arm):
        start = 7 * ARMS.index(arm)
        return ik.tcp_pose(self.measured[start:start + 6], BASES[arm])[0]

    def opening(self, arm):
        """Distance between the pads from the gripper reading, metres."""
        return float(self.measured[7 * ARMS.index(arm) + 6]) * 2 * abs(OPEN)

    def cents(self):
        """The tuned string's pitch, cents from G4 (what the tuner shows while it reads)."""
        return float(self.state["cents"][L.TUNED])

    def button_deg(self):
        """The held button's measured angle in tightening degrees."""
        return L.tighten_sign(self.button) * float(np.degrees(self.state["buttons"][self.button]))

    def paddle_roll(self):
        """Which way the button's thickness points now: its joint angle about +z (it looks the same every half turn)."""
        return float(self.state["buttons"][self.button])

    def note(self, name, **values):
        self.log["events"].append({"tick": self.tick, "event": name,
                                   **{k: (v if isinstance(v, str) else round(float(v), 4) if np.isscalar(v)
                                          else [round(float(x), 4) for x in v]) for k, v in values.items()}})

    def phase(self, name):
        self.phases.append((self.tick, name))

    # -- motion ---------------------------------------------------------------------------------------------------
    def emit(self):
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.aperture[a] / OPEN, 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def hold(self, seconds):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield self.emit()

    def glide(self, seconds, targets, apertures=None):
        """Move arms in joint space to the given joints, grippers to the given apertures (smooth start and stop)."""
        start = {arm: self.joints[arm].copy() for arm in targets}
        first = {arm: self.aperture[arm] for arm in (apertures or {})}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm, goal in targets.items():
                self.joints[arm] = start[arm] + alpha * (np.asarray(goal, dtype=float) - start[arm])
            for arm, goal in (apertures or {}).items():
                self.aperture[arm] = first[arm] + alpha * (goal - first[arm])
            yield self.emit()

    def pose_joints(self, arm, point, rotations):
        """Joint angles for a pose, from several starting guesses and any of the given equivalent ``rotations``: the
        solution furthest from the joint limits. Returns (joints, rotation index)."""
        best = None
        for k, rot in enumerate(rotations):
            for seed in SEEDS:
                seed = seed * np.array((SIDE[arm], 1, 1, 1, SIDE[arm], SIDE[arm]))
                joints, error, turn = ik.solve(point, ik.matrix_to_quat_xyzw(rot), seed, BASES[arm], iters=SOLVE_ITERS)
                if error < 5e-4 and turn < 5e-3:
                    margin = float(np.minimum(joints - ik.LOWER, ik.UPPER - joints).min())
                    if best is None or margin > best[1]:
                        best = (np.asarray(joints, dtype=float), margin, k)
        if best is None:
            raise RuntimeError(f"{arm} arm cannot reach {np.round(point, 3)}")
        self.note("pose", arm=arm, joint_margin_deg=np.degrees(best[1]), rotation=best[2])
        return best[0].copy(), best[2]

    def solve(self, arm, point, rot, seed=None, iters=60):
        start = self.joints[arm] if seed is None else seed
        joints, error, _ = ik.solve(point, ik.matrix_to_quat_xyzw(rot), start, BASES[arm], iters=iters)
        self.worst_ik = max(self.worst_ik, float(error))
        return np.asarray(joints, dtype=float).copy()

    def right_q(self, point, psi, j6):
        """Right hand at ``point`` on the button's shaft with finger roll ``psi`` and joint 6 at ``j6`` (joint 6 is
        kept where the stroke wants it)."""
        seed = self.joints["right"].copy()
        seed[5] = j6
        q = self.solve("right", point, button_rotation(psi), seed=seed, iters=100)
        self.j6_shift = max(self.j6_shift, abs(float(q[5]) - j6))
        q[5] = j6
        return q

    def left_move(self, point, seconds):
        """Move the left hand's tool point in a straight line (smooth start and stop), keeping its pluck pose."""
        p0 = self.left_cmd.copy()
        p1 = np.asarray(point, dtype=float)
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            self.left_cmd = p0 + alpha * (p1 - p0)
            self.joints["left"] = self.solve("left", self.left_cmd, PLUCK_ROT)
            yield self.emit()

    def settle(self, arm, want, tries=3):
        """Shift the arm's command by its measured error until its tool point is on ``want`` (the arms sag a
        little)."""
        for _ in range(tries):
            yield from self.hold(.25)
            error = want - self.actual(arm)
            if np.linalg.norm(error) < .001:
                break
            self.offset[arm] = np.clip(self.offset[arm] + np.clip(error, -.004, .004), -.02, .02)
            if arm == "right":
                yield from self.glide(.3, {arm: self.right_q(want + self.offset[arm], self.psi, self.j6)})
            else:
                yield from self.left_move(want + self.offset[arm], .3)
        self.note("settled", arm=arm, error_mm=1000 * (want - self.actual(arm)), offset_mm=1000 * self.offset[arm])

    # -- the button -----------------------------------------------------------------------------------------------
    def stroke(self):
        low = max(STROKE[0], float(ik.LOWER[5]) + LIMIT_MARGIN)
        high = min(STROKE[1], float(ik.UPPER[5]) - LIMIT_MARGIN)
        if high - low < np.pi + 2 * MIN_ROOM:
            raise RuntimeError(f"Joint 6 turns {np.degrees(high - low):.0f} degrees; regrips need more than half a turn")
        return low, high

    def roll_sign(self, q):
        """+1 if raising joint 6 turns the right hand's fingers positively about the button's shaft (+z). Also logs
        how far the tool point moves when joint 6 alone turns (it should not: joint 6 rolls about the hand's axis)."""
        p0, r0 = ik.tcp_pose(q, BASES["right"])
        q1 = np.array(q, dtype=float)
        q1[5] += .1
        p1, r1 = ik.tcp_pose(q1, BASES["right"])
        self.note("joint 6 check", tool_moved_mm=1000 * float(np.linalg.norm(p1 - p0)),
                  approach_turned_deg=np.degrees(np.arccos(np.clip(r0[:, 2] @ r1[:, 2], -1., 1.))))
        return 1. if float(np.cross(r0[:, 0], r1[:, 0]) @ SHAFT) > 0 else -1.

    def tightening_heading(self):
        """Which way joint 6 turns (+1, -1) to tighten the held button."""
        return 1. if tuning.roll_change(1., L.tighten_sign(self.button), self.s6) > 0 else -1.

    def grip_point(self):
        """Tool point for the pinch: on the shaft's axis, pad tips TIPS_IN down from the button's top."""
        return np.asarray(fixture.to_station(L.button_tip(self.button))) + AXIS * (TIPS_IN - TIP)

    def pluck_points(self):
        """Left hand: where the press starts (the fingertip's middle PLUCK_GAP above the G string over the soundhole,
        TIP_PAST_STRING toward the soundboard) and where it waits above that."""
        string = np.asarray(fixture.to_station(L.pluck_point()))
        tip = string + (TIP_PAST_STRING, 0., L.STRING_R + PLUCK_GAP)
        start = tip - PLUCK_A * TIP
        return start, start + (0., 0., HOVER)

    def clear_point(self):
        """Left hand: the waypoint through which it comes to the string and leaves it, CLEAR_FRONT in front of the
        string (toward the robot) and its fingertip CLEAR_ABOVE above the body's upper edge near the soundhole. Below
        that edge the hand moves only in straight lines, in front of the soundboard."""
        string = np.asarray(fixture.to_station(L.pluck_point()))
        edge = max(L.body_half_width(L.PLUCK_Y + dy) for dy in np.linspace(-.04, .04, 9))
        rim = fixture.to_station((edge, L.PLUCK_Y, L.TOP_Z))[2]
        tip = np.array((string[0] - CLEAR_FRONT, string[1], rim + CLEAR_ABOVE))
        return tip - PLUCK_A * TIP

    def reach(self):
        """Both arms at once: the right hand 30 mm above the G button, pointing down, pads open, its roll chosen among
        the equivalent ones (every half turn) to leave the most room for tightening; the left hand to the waypoint in
        front of the soundboard and above the body (``clear_point``), then straight down to wait above the string."""
        self.phase("reach")
        psi = self.paddle_roll()
        back = self.grip_point() - AXIS * APPROACH_BACK
        q, k = self.pose_joints("right", back, [button_rotation(psi), button_rotation(psi + np.pi)])
        psi += k * np.pi
        self.s6 = self.roll_sign(q)
        low, high = self.stroke()
        j6 = tuning.equivalent_roll(float(q[5]), low, high, self.tightening_heading())
        if j6 is None:
            raise RuntimeError(f"No roll of joint 6 within {np.degrees((low, high))} grips the button")
        self.psi = psi + self.s6 * (j6 - float(q[5]))
        self.j6 = j6
        q[5] = j6
        start, hover = self.pluck_points()
        clear = self.clear_point()
        q_left, _ = self.pose_joints("left", clear, [PLUCK_ROT])
        self.left_cmd = clear.copy()
        yield from self.glide(.3, {}, {"right": BUTTON_APPROACH, "left": 0.})
        yield from self.glide(2.6, {"right": q, "left": q_left})
        self.joints["left"] = q_left
        yield from self.left_move(hover, .8)
        yield from self.hold(.3)

    def close_on_button(self):
        yield from self.glide(.5, {}, {"right": BUTTON_PINCH})
        yield from self.hold(.2)
        held = self.opening("right") >= HELD_M
        self.holding = held
        self.note("button pinched", pads_mm=1000 * self.opening("right"), held=float(held), joint6_deg=np.degrees(self.j6))
        return held

    def release_button(self):
        """Open the right hand and lift it RELEASE_BACK up the shaft, clear of the button."""
        self.phase("let go of the button")
        yield from self.glide(.5, {}, {"right": BUTTON_APPROACH})
        yield from self.glide(.8, {"right": self.right_q(self.grip_cmd - AXIS * RELEASE_BACK, self.psi, self.j6)})
        self.holding = False
        self.note("let go of the button", button_deg=self.button_deg())

    def retake(self, error):
        """Take the button again after letting go (the final check found it off): the fingers lined up with the button
        as it is now, the roll chosen to leave the most room for the turn ``error`` needs, then pinch as at first."""
        self.phase("take the button again")
        heading = self.tightening_heading() * (1. if error < 0 else -1.)
        low, high = self.stroke()
        aligned = self.j6 + self.s6 * (self.paddle_roll() - self.psi)
        j6 = tuning.equivalent_roll(aligned, low, high, heading)
        if j6 is None:
            raise RuntimeError(f"No roll of joint 6 within {np.degrees((low, high))} grips the button")
        psi = self.paddle_roll() + self.s6 * (j6 - aligned)
        back = self.grip_cmd - AXIS * APPROACH_BACK
        yield from self.glide(max(.4, abs(j6 - self.j6) / ROLL_BACK_RATE), {"right": self.right_q(back, psi, j6)})
        self.j6, self.psi = j6, psi
        held = yield from self.pinch()
        return held

    def pinch(self):
        arm = "right"
        grip = self.grip_point()
        for attempt in range(3):
            self.phase(f"pinch the button {attempt + 1}")
            yield from self.settle(arm, grip - AXIS * APPROACH_BACK)
            yield from self.glide(1.2, {arm: self.right_q(grip + self.offset[arm], self.psi, self.j6)})
            yield from self.settle(arm, grip)
            self.grip_cmd = grip + self.offset[arm]
            held = yield from self.close_on_button()
            if held:
                return True
            yield from self.glide(.3, {}, {arm: BUTTON_APPROACH})
            yield from self.glide(.6, {arm: self.right_q(self.grip_cmd - AXIS * APPROACH_BACK, self.psi, self.j6)})
        return False

    def roll(self, change):
        """Turn joint 6 alone by ``change`` radians (smooth start and stop): the held button turns with it."""
        j0 = self.j6
        steps = max(1, round(max(.3, abs(change) * 1.5 / TURN_RATE) * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            self.j6 = j0 + alpha * change
            self.joints["right"] = self.joints["right"].copy()
            self.joints["right"][5] = self.j6
            yield self.emit()
        self.psi += self.s6 * change

    def regrip(self, heading):
        """Open, rise along the shaft, roll joint 6 to the equivalent grip with the most room toward ``heading`` (half
        a turn back or more, lined up with the button as measured), come down again and pinch."""
        arm = "right"
        self.regrips += 1
        self.phase(f"regrip {self.regrips}")
        yield from self.glide(.3, {}, {arm: BUTTON_APPROACH})
        back = self.grip_cmd - AXIS * REGRIP_BACK
        yield from self.glide(.3, {arm: self.right_q(back, self.psi, self.j6)})
        yield from self.hold(.1)                          # the arm lags its command: clear of the button before rolling
        low, high = self.stroke()
        aligned = self.j6 + self.s6 * (self.paddle_roll() - self.psi)     # fingers square to the button as it is now
        j6 = tuning.equivalent_roll(aligned, low, high, heading)
        if j6 is None or (high - j6 if heading > 0 else j6 - low) < MIN_ROOM:
            raise RuntimeError(f"Regrip found no room for joint 6 (at {np.degrees(self.j6):.1f} degrees)")
        psi = self.paddle_roll() + self.s6 * (j6 - aligned)
        yield from self.glide(max(.3, abs(j6 - self.j6) / ROLL_BACK_RATE), {arm: self.right_q(back, psi, j6)})
        self.j6, self.psi = j6, psi
        yield from self.glide(.3, {arm: self.right_q(self.grip_cmd, self.psi, self.j6)})
        held = yield from self.close_on_button()
        return held

    def turn(self, tighten_deg):
        """Turn the held button by ``tighten_deg`` tightening degrees (negative loosens): joint 6 open loop, regripping
        when it runs out of roll, then checked against the button's measured angle and corrected."""
        self.phase(f"turn {tighten_deg:+.1f} degrees")
        start = self.button_deg()
        goal = start + tighten_deg
        for attempt in range(4):
            remaining = goal - self.button_deg()
            if abs(remaining) <= TOLERANCE_DEG:
                break
            change = tuning.roll_change(remaining, L.tighten_sign(self.button), self.s6)
            heading = 1. if change > 0 else -1.
            while abs(change) > 1e-6:
                low, high = self.stroke()
                room = (high - self.j6) if heading > 0 else (self.j6 - low)
                if room < MIN_ROOM:
                    held = yield from self.regrip(heading)
                    if not held:
                        self.note("lost the button", button_deg=self.button_deg() - start)
                        return False
                    continue
                step = heading * min(abs(change), room)
                yield from self.roll(step)
                change -= step
            yield from self.hold(.3)                      # let the wrist come to rest before measuring
            self.note("turned", attempt=attempt + 1, wanted_deg=tighten_deg, button_deg=self.button_deg() - start,
                      cents=self.cents(), pads_mm=1000 * self.opening("right"))
        return True                                       # still holding it; the next pluck says how close it is

    # -- plucking -------------------------------------------------------------------------------------------------
    def pluck(self):
        """Pluck the G string once over the soundhole; True when the scene counted a pluck (the tuner then reads)."""
        arm = "left"
        count = int(self.state["plucks"])
        start, hover = self.pluck_points()
        for attempt in range(3):
            self.phase(f"pluck {count + 1}" + (f" try {attempt + 1}" if attempt else ""))
            yield from self.left_move(start + self.offset[arm], .5)
            yield from self.settle(arm, start, tries=2)
            command = start + self.offset[arm]
            pressed, peak = 0., 0.
            while pressed < PLUCK_MAX + .004 * attempt:
                out = abs(float(self.state["pluck"]))
                peak = max(peak, out)
                if out >= PLUCK_WANT:
                    break
                pressed += PLUCK_RATE * DT
                self.left_cmd = command - (0., 0., pressed)
                self.joints[arm] = self.solve(arm, self.left_cmd, PLUCK_ROT)
                yield self.emit()
            yield from self.left_move(self.left_cmd + (-PLUCK_SLIDE, 0., 0.), PLUCK_SLIDE_S)
            yield from self.left_move(self.left_cmd + (0., 0., PLUCK_LIFT), .25)
            for _ in range(int(.5 * CONTROL_HZ)):
                if int(self.state["plucks"]) > count:
                    break
                yield self.emit()
            plucked = int(self.state["plucks"]) > count
            self.note("plucked" if plucked else "pluck not counted", attempt=attempt + 1, pressed_mm=1000 * pressed,
                      out_mm=1000 * peak)
            yield from self.left_move(hover + self.offset[arm], .5)
            if plucked:
                return True
        return False

    # -- the demonstration ----------------------------------------------------------------------------------------
    def let_go(self):
        self.phase("let go")
        if self.holding:
            yield from self.release_button()
        yield from self.left_move(self.clear_point(), .8)       # up and out in front of the soundboard first
        yield from self.glide(2.2, {arm: np.asarray(READY, dtype=float) for arm in ARMS}, dict.fromkeys(ARMS, OPEN))

    def sequence(self):
        while self.state is None:
            yield self.emit()
        self.note("start", cents=self.cents(), button=self.button)
        yield from self.reach()
        held = yield from self.pinch()
        if not held:
            self.note("gave up: the button could not be pinched")
        controls = ("wrong_button", "hold_on", "blind")
        cycles = 1 if self.variant == "wrong_button" else MAX_CYCLES
        while held and self.cycles < cycles:
            self.cycles += 1
            plucked = False
            if self.variant != "blind":
                plucked = yield from self.pluck()
            error = self.cents()                               # what the tuner shows while it reads
            shown = self.state["tuner"]
            self.note("tuner" if self.holding else "final check", cycle=self.cycles, plucked=float(plucked),
                      cents=error, note=str(shown["note"]), needle_deg=shown["needle_deg"])
            if abs(error) > DONE_CENTS:
                if not self.holding:                          # letting go turned it: take it again and correct
                    held = yield from self.retake(error)
                    if not held:
                        break
                held = yield from self.turn(tuning.button_turn_for(error, float(self.state["play_deg"])))
                continue
            if not self.holding or self.variant in controls:
                break                                         # in tune, read after the last movement (or a control)
            # The final check: let go first, then pluck once more, so the last pluck comes after the last movement.
            yield from self.release_button()
        if self.variant == "hold_on":
            yield from self.left_move(self.clear_point(), .8)
            yield from self.glide(2.2, {"left": np.asarray(READY, dtype=float)}, {"left": OPEN})
        else:
            yield from self.let_go()
        self.done = True
        self.phase("rest")
        while True:
            yield self.emit()
