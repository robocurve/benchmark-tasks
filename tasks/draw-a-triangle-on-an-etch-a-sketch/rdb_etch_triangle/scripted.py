"""Full-score demonstration: each arm takes the knob on its side from above and turns it by rolling its wrist.

The demonstration reads the robot's own joint and gripper readings and, as the review rules allow for a
scripted demonstration, simulator state: where the stylus is, the two knob angles, whether each knob's string
slack is taken up, and where each knob is. It changes nothing in the simulation; the stylus moves only because
the grippers turn the knobs.

A knob is turned by the last wrist joint alone (joint 6, the roll about the gripper's axis, which points straight
down through the knob's middle), so the fingers turn about the knob's axis and nothing else moves. Joint 6
turns 120 degrees either way, so each grip turns a knob at most half a turn (a stroke of 180 degrees between
-90 and +90); then the gripper opens, lifts 8 mm, rolls back half a turn and closes again (a regrip). The
gripper is symmetric, so after half a turn the fingers sit across the knob exactly as before.

Plan: (1) both grippers above their knobs, down, close. (2) Bottom side: the left knob clockwise until the
stylus is 80 mm right of the dot. (3) Upright side: the right knob clockwise until the stylus is 60 mm up.
(4) Slanted side: both knobs counter-clockwise together. The slack of both strings is taken up first (no line
is drawn while it is), then a target point moves along the ideal slanted line at 10 mm/s and each knob is
turned at the rate that keeps the stylus on it (the 4 to 3 ratio as feed-forward, plus a correction from the
measured stylus). Whenever either hand needs a regrip the target stops and both knobs stand still, so the line
pauses but does not run along one axis alone. The target also slows down before either grip runs out of roll
and speeds up again over 0.8 s after a regrip, so that neither knob coasts or starts on its own. (5) Open both grippers, lift and move the arms away.
"""

from inspect_robots import Action, ActionChunk
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, READY

from . import layout
from .fixed_scene import ARMS, BASES, CONTROL_HZ, KNOB_OF, knob_station

OPEN = float(FINGER_OPEN)
DT = 1. / CONTROL_HZ
SIDE = {"left": 0, "right": 1}                 # index of each arm's knob in the knob-angle readings
PAD_TIP_Z = .029                               # pad tips 0.9 mm above the knob's underside, 5 mm above the frame face
GRIP_Z = PAD_TIP_Z + PAD_TIP_PAST_TCP          # TCP height while holding a knob
LIFT = .008                                    # how far a gripper lifts to roll back over the screen's lip
ABOVE_Z = .100                                 # start of the descent onto a knob
SQUEEZE = .004                                 # each finger commanded this far inside the knob: about 20 N a finger
KNOB_FINGER = -(layout.KNOB_R - SQUEEZE)
REGRIP_OPEN = -.026                            # fingers 52 mm apart for a regrip
STROKE = (np.radians(-90.), np.radians(90.))   # joint 6 range used while turning
TURN_RATE = np.radians(110.)                   # fastest knob turn, rad/s
ROLL_BACK_RATE = np.radians(300.)
SLANT_SPEED = .010                             # m/s of the target point along the slanted side
ALONG_GAIN, ACROSS_GAIN = 2.5, 6.                # 1/s
LEAD_LIMIT = .002                              # the target never runs more than 2 mm ahead of the stylus
RAMP_S = .8                                    # the target speeds up over this long, so the knobs start in step
CLOSE_ENOUGH_M = .00015
SEED = {"left": (.61, 1.99, 1.47, -1.03, 0., 0.), "right": (-.61, 1.99, 1.47, -1.03, 0., 0.)}


def down_rotation(yaw):
    """Gripper pointing straight down, its fingers closing along the horizontal direction ``yaw``."""
    z = np.array((0., 0., -1.))
    x = np.array((np.cos(yaw), np.sin(yaw), 0.))
    return np.column_stack((x, np.cross(z, x), z))


class EtchReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self):
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.aperture = dict.fromkeys(ARMS, OPEN)
        self.knob = {arm: knob_station(KNOB_OF[arm]) for arm in ARMS}       # centre of the knob's top face
        self.offset = {arm: np.zeros(3) for arm in ARMS}
        self.base = {}                                                       # joints 1-5 of the grip
        self.yaw0 = {}                                                       # finger direction at joint 6 = 0
        self.worst_ik = 0.
        self.worst_j6_shift = 0.                                            # diagnostics only
        self.phases = []
        self.tick = 0
        self.regrips = dict.fromkeys(ARMS, 0)
        self.log = {"stroke_ends": [], "holding": [], "grip_error_mm": [], "centre_error_mm": []}

    # -- readings -------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        self.state = state

    def actual_tcp(self, arm):
        start = 7 * ARMS.index(arm)
        return ik.tcp_pose(self.measured[start:start + 6], BASES[arm])[0]

    def opening(self, arm):
        return float(self.measured[7 * ARMS.index(arm) + 6])

    def stylus(self):
        return np.asarray(self.state["stylus"], dtype=float) if self.state else np.zeros(2)

    def knob_top(self, arm):
        if self.state and "knob_tops" in self.state:
            return np.asarray(self.state["knob_tops"][SIDE[arm]], dtype=float)
        return self.knob[arm]

    # -- motion ---------------------------------------------------------------------------
    def emit(self):
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.aperture[a] / OPEN, 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def phase(self, name):
        self.phases.append((self.tick, name))

    def plan(self, arm, height, j6=None):
        """Joints with the TCP ``height`` above the table over the knob, joint 6 at ``j6`` (default: as now)."""
        top = self.knob_top(arm)
        point = np.array((top[0], top[1], height)) + self.offset[arm]
        j6 = self.joints[arm][5] if j6 is None else j6
        if arm not in self.yaw0:
            # find which finger direction joint 6 = 0 gives over this knob (joint 6 is the roll about the axis)
            q, _, _ = ik.solve(point, ik.matrix_to_quat_xyzw(down_rotation(0.)), np.asarray(SEED[arm]),
                               BASES[arm], iters=400)
            rotation = ik.tcp_pose(q, BASES[arm])[1]
            self.yaw0[arm] = float(np.arctan2(rotation[1, 0], rotation[0, 0]) + q[5])
        yaw = self.yaw0[arm] - j6
        seed = self.base.get(arm, np.asarray(SEED[arm]))
        q, error, _ = ik.solve(point, ik.matrix_to_quat_xyzw(down_rotation(yaw)), np.r_[seed[:5], j6], BASES[arm],
                               iters=200)
        self.worst_ik = max(self.worst_ik, error)
        # Joint 1 turns the whole arm a little when the grip point is corrected, which changes the finger direction
        # that a given joint 6 angle gives by a degree or two. On a round knob the finger direction does not matter,
        # so joint 6 is kept where the stroke wants it (it only rolls about the downward axis through the TCP).
        self.worst_j6_shift = max(self.worst_j6_shift, abs(float(q[5]) - j6))
        q[5] = j6
        self.base[arm] = q.copy()
        return q

    def glide(self, seconds, targets, apertures=None):
        start = {arm: self.joints[arm].copy() for arm in targets}
        first = {arm: self.aperture[arm] for arm in (apertures or {})}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm, goal in targets.items():
                self.joints[arm] = start[arm] + alpha * (np.asarray(goal) - start[arm])
            for arm, goal in (apertures or {}).items():
                self.aperture[arm] = first[arm] + alpha * (goal - first[arm])
            yield self.emit()

    def hold(self, seconds):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield self.emit()

    def centre(self, arm, height, tries=2):
        """Shift the arm's command by its measured error over the knob (the arms sag a little). A finger that
        meets the knob before the other pushes the whole toy, so the grip is centred within 1 mm before closing
        (run 1: the toy crept 4.6 mm and 2 degrees over 16 regrips)."""
        for _ in range(tries):
            yield from self.hold(.3)                     # let the arm come to rest before measuring it
            top = self.knob_top(arm)
            want = np.array((top[0], top[1], height))
            error = want - self.actual_tcp(arm)
            self.log["centre_error_mm"].append([self.tick, arm, *(round(1000 * float(v), 2) for v in error)])
            if np.linalg.norm(error[:2]) < .001 and abs(error[2]) < .002:
                break
            self.offset[arm] = np.clip(self.offset[arm] + np.clip(error, -.004, .004), -.015, .015)
            yield from self.glide(.3, {arm: self.plan(arm, height)})

    def close(self, arms, seconds=.6):
        yield from self.glide(seconds, {}, {arm: KNOB_FINGER for arm in arms})
        yield from self.hold(.25)
        for arm in arms:
            holding = self.opening(arm) > KNOB_FINGER / OPEN + .02 if self.state else True
            self.log["holding"].append([self.tick, arm, bool(holding), round(self.opening(arm), 4)])
            top = self.knob_top(arm)
            error = self.actual_tcp(arm) - np.array((top[0], top[1], GRIP_Z))
            self.log["grip_error_mm"].append([self.tick, arm, *(round(1000 * float(v), 1) for v in error)])

    def regrip(self, arm, to_j6):
        """Open, lift over the lip, roll joint 6 back to ``to_j6``, come down and close again."""
        self.regrips[arm] += 1
        yield from self.glide(.35, {}, {arm: REGRIP_OPEN})
        yield from self.glide(.3, {arm: self.plan(arm, GRIP_Z + LIFT)})
        seconds = max(.3, abs(to_j6 - self.joints[arm][5]) / ROLL_BACK_RATE)
        yield from self.glide(seconds, {arm: self.plan(arm, GRIP_Z + LIFT, to_j6)})
        yield from self.glide(.3, {arm: self.plan(arm, GRIP_Z)})
        yield from self.centre(arm, GRIP_Z)
        yield from self.close([arm], .5)

    def turn(self, arm, knob_rate):
        """Turn the held knob at ``knob_rate`` (rad/s, counter-clockwise seen from above) for one tick.
        Joint 6 turns the other way (it rolls about the downward gripper axis)."""
        self.joints[arm] = self.joints[arm].copy()
        self.joints[arm][5] = float(np.clip(self.joints[arm][5] - knob_rate * DT, *STROKE))

    def room(self, arm, direction):
        """Knob turn (rad) left in this grip in ``direction`` (+1 counter-clockwise)."""
        j6 = self.joints[arm][5]
        return j6 - STROKE[0] if direction > 0 else STROKE[1] - j6

    def ready_for(self, arm, direction, need=np.radians(12.)):
        if self.room(arm, direction) < need:
            self.log["stroke_ends"].append([self.tick, arm, round(float(np.degrees(self.joints[arm][5])), 1)])
            yield from self.regrip(arm, STROKE[1] if direction > 0 else STROKE[0])

    # -- the demonstration ------------------------------------------------------------------
    def take_knobs(self):
        self.phase("reach")
        # clockwise strokes first: joint 6 starts at the low end of its stroke
        above = {arm: self.plan(arm, ABOVE_Z, STROKE[0]) for arm in ARMS}
        yield from self.glide(2.5, above)
        yield from self.hold(.3)
        for arm in ARMS:
            yield from self.centre(arm, ABOVE_Z)
        self.phase("descend")
        yield from self.glide(1.4, {arm: self.plan(arm, GRIP_Z + .012) for arm in ARMS})
        yield from self.glide(.6, {arm: self.plan(arm, GRIP_Z) for arm in ARMS})
        for arm in ARMS:
            yield from self.centre(arm, GRIP_Z)
        self.phase("close")
        yield from self.close(ARMS)

    def one_knob(self, arm, axis, goal_m, name):
        """Turn one knob until the stylus coordinate ``axis`` (0 right, 1 up) reaches ``goal_m``."""
        self.phase(name)
        direction = -1. if goal_m > self.stylus()[axis] else 1.     # clockwise moves the stylus right / up
        while True:
            error = goal_m - self.stylus()[axis]
            if abs(error) < CLOSE_ENOUGH_M or -direction * error < 0:
                break
            yield from self.ready_for(arm, direction)
            need = abs(error) / layout.PER_RAD
            engaged = self.state["engaged"][SIDE[arm]][0 if direction > 0 else 1] if self.state else True
            rate = TURN_RATE if not engaged else float(np.clip(3. * need, np.radians(4.), TURN_RATE))
            self.turn(arm, direction * rate)
            yield self.emit()
        yield from self.hold(.3)

    def slant(self):
        self.phase("slant: take up slack")
        for arm in ARMS:                                                  # both knobs now turn counter-clockwise
            yield from self.ready_for(arm, +1., need=np.radians(60.))
        for _ in range(4 * CONTROL_HZ):
            slack = [not self.state["engaged"][SIDE[a]][0] if self.state else False for a in ARMS]
            if not any(slack):
                break
            for arm, loose in zip(ARMS, slack, strict=True):
                if loose:
                    self.turn(arm, np.radians(40.))
            yield self.emit()
        self.phase("slant: both knobs")
        corner = np.array((layout.BASE_LEN, layout.RISE))
        heading = -corner / np.linalg.norm(corner)
        normal = np.array((-heading[1], heading[0]))
        s = 0.
        moving = 0                                       # ticks since the target last started moving
        while True:
            for arm in ARMS:
                before = self.tick
                yield from self.ready_for(arm, +1.)
                if self.tick != before:
                    moving = 0                           # after a regrip both knobs start again together, slowly
            offset = self.stylus() - corner
            along = float(offset @ heading)               # progress along the slanted side
            across = float(offset @ normal)               # distance off the ideal line
            if s >= layout.SLANT_LEN and along >= layout.SLANT_LEN - CLOSE_ENOUGH_M:
                break
            moving += 1
            ramp = min(1., moving / (RAMP_S * CONTROL_HZ))
            # slow down before either grip runs out of roll, so that neither knob coasts on alone into a regrip
            room = min(self.room(arm, +1.) for arm in ARMS)
            ramp = min(ramp, float(np.clip((room - np.radians(10.)) / np.radians(25.), .15, 1.)))
            speed = SLANT_SPEED * ramp if s < layout.SLANT_LEN else 0.
            s = min(layout.SLANT_LEN, s + speed * DT, along + LEAD_LIMIT)
            # wanted stylus velocity: along the line at the target's pace, and back onto the line if off it
            wanted = (speed + ALONG_GAIN * (s - along)) * heading - ACROSS_GAIN * across * normal
            for arm in ARMS:
                # -> knob rate (clockwise moves right / up); never back: a reversal would only take up the slack
                self.turn(arm, float(np.clip(-wanted[SIDE[arm]] / layout.PER_RAD, 0., TURN_RATE)))
            yield self.emit()
        yield from self.hold(.3)

    def let_go(self):
        self.phase("let go")
        yield from self.glide(.5, {}, dict.fromkeys(ARMS, OPEN))
        yield from self.glide(.6, {arm: self.plan(arm, GRIP_Z + .04) for arm in ARMS})
        away = {}
        for arm in ARMS:
            q = np.asarray(READY, dtype=float).copy()
            away[arm] = q
        yield from self.glide(2., away)
        while True:
            yield from self.hold(1.)

    def sequence(self):
        yield from self.take_knobs()
        yield from self.one_knob("left", 0, layout.BASE_LEN, "bottom side")
        yield from self.one_knob("right", 1, layout.RISE, "upright side")
        yield from self.slant()
        yield from self.let_go()
