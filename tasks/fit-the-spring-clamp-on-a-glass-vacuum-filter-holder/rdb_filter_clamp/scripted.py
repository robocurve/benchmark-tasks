"""Scripted reference: the left hand seats the funnel on the base and holds it down; the right hand takes
the clamp by its handles, squeezes it open, pushes it round both flanges and lets it close; then the left hand proof-lifts the assembly, sets it down and releases it.

The demonstration reads the robot's own joint and gripper readings and, as the review rules allow for a
scripted demonstration, simulator state: the poses of the holder, the funnel and the clamp and the jaws' angles.
It changes nothing in the simulation; the funnel and the clamp move only because the fingers hold them.

Grips (found with several-seed inverse kinematics over the whole path; README "Reach"):
- Left on the funnel: the pads close along x on two of the 16 flats of its 64 mm body, 65 mm above its ground
  face, fingers pointing to the right (-y) and 30 degrees down. The same grip picks it off the wipe and holds it
  on the base, so the funnel is never turned in the hand.
- Right on the clamp: the pads close along y on the two handles, fingers pointing forward (+x) and 45 degrees
  down. Closing fully presses the handles together (about 17 N against the spring) and opens the jaws to 27
  degrees each; the same grip lifts it off the bench and carries it, open, to the joint.
Every placing move is closed on the measured pose of the part being placed, because the arms sag by a
centimetre or more. Each stage checks its result and is repeated if it failed.
"""

import os

from inspect_robots import Action, ActionChunk
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY

from . import layout as L
from .grading import LIFT_HEIGHT, MARK_TOL, MAX_LIFT_HEIGHT, RELEASE_TICKS, SEAT_GAP, TABLE_TOL, flask_clearance
from .fixed_scene import ARMS, BASES, CONTROL_HZ, HOLDER_START, MAX_STEPS

OPEN = abs(float(FINGER_OPEN))
PAD_SPAN = 2 * OPEN                          # pads' separation (m) at a gripper reading of 1.0
DT = 1. / CONTROL_HZ
VARIANT = os.environ.get("DEMO_VARIANT", "")  # "no_hold": the control run (README); anything else: the demonstration


def grip_rotation(closing, approach):
    """Gripper rotation, columns: closing axis, pad-width axis, finger direction."""
    a = np.asarray(approach, float) / np.linalg.norm(approach)
    c = np.asarray(closing, float) - (np.asarray(closing, float) @ a) * a
    c /= np.linalg.norm(c)
    return np.column_stack((c, np.cross(a, c), a))


FUNNEL_PITCH, CLAMP_PITCH = np.radians(30.), np.radians(45.)
R_FUNNEL = grip_rotation((-1., 0., 0.), (0., -np.cos(FUNNEL_PITCH), -np.sin(FUNNEL_PITCH)))
R_CLAMP = grip_rotation((0., -1., 0.), (np.cos(CLAMP_PITCH), 0., -np.sin(CLAMP_PITCH)))
GRIP_Z = .065                                # fingertip centre this far above the funnel's ground face
FUNNEL_SQUEEZE = .004                        # each pad commanded this far inside the flat: about 20 N
FUNNEL_FINGER = (L.BODY_R - FUNNEL_SQUEEZE) / OPEN
FUNNEL_HELD_MM = 60.                         # measured pad separation above this: the pads stopped on the funnel
CLAMP_SQUEEZED_DEG = 24.                     # both jaws opened at least this far: the handles are pressed together
RELEASE_CLAMP = (.40, .62)                   # gripper readings while letting the clamp close, then clear of the handles
LOWER_STEPS = 10 * CONTROL_HZ
LOWER_SETTLE_S = .5
FUNNEL_RELEASE_TIMES = (.6, .3, 1., 2.2, .3)  # open, settle, withdraw, return home, open fully
ABOVE = .022                                 # funnel's ground face this far above the base's on the way in
VIA = np.array((.310, .050, L.JOINT_Z + ABOVE))   # funnel's ground face on the way over (reach study)
BEHIND = .075                               # clamp ring this far behind the joint before it is pushed on
SEAT_TOL = .0006
SEAT_TRIES = 3
RETREAT = (np.array((-.130, -.060, .002)), np.array((-.140, -.170, -.038)))   # right fingertip centre, from the joint
SERVO_GAIN, SERVO_EVERY = .6, 4               # move and settle: 60 % of the measured error every 4th step
SQUARE_TOL = np.radians(.8)                  # funnel's axis within this of the holder's before it is lowered
LIGHT_FINGER = (L.BODY_R - .0012) / OPEN      # about 6 N a pad: the funnel can square itself on the flange
PRESS = .008                                 # pressed this far past touching (a few newtons through the soft arm)
SEEDS = {"funnel": (.06, 1.78, .90, -.80, 1.04, 1.69), "clamp": (.69, 1.09, .18, -.01, .47, .53),
         "joint_left": (-.52, 1.95, 2.26, -1.17, .85, .72), "joint_right": (1.10, 1.68, 1.86, -1.33, .68, .95)}


def unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def yaw_of(rotation):
    return float(np.arctan2(rotation[1, 0], rotation[0, 0]))


class FilterClampReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self):
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.aperture = dict.fromkeys(ARMS, 1.)
        self.target = dict.fromkeys(ARMS)
        self.worst_ik = 0.
        self.phases = []
        self.tick = 0
        self.funnel_in_hand = None
        self.clamp_in_hand = None
        # servo rows: tick, stage, arm, part position error x, y, z (mm), part tilt (deg), gripper opening (mm)
        self.log = {"events": [], "grasps": [], "seatings": [], "fittings": [], "servo": [], "variant": VARIANT or None,
                    "worst_ik_mm_by_phase": {}}

    # -- readings -------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        if state is not None:
            self.state = state

    def hand(self, arm):
        start = 7 * ARMS.index(arm)
        return ik.tcp_pose(self.measured[start:start + 6], BASES[arm])

    def opening_mm(self, arm):
        return 1000. * PAD_SPAN * float(self.measured[7 * ARMS.index(arm) + 6])

    def part(self, name):
        p, r = self.state[name]
        return np.asarray(p, dtype=float), np.asarray(r, dtype=float)

    def seat(self):
        return L.funnel_on_base(*self.part("holder"), *self.part("funnel"))

    def ring(self):
        return L.clamp_on_joint(*self.part("holder"), *self.part("clamp"), self.state["jaw_open_deg"])

    def joint_point(self):
        """Centre of the joint (top of the base's flange) from the holder's measured pose."""
        p, r = self.part("holder")
        return p + r @ np.array((0., 0., L.JOINT_Z))

    def event(self, text, **fields):
        self.log["events"].append({"tick": self.tick, "event": text, **fields})

    def phase(self, name):
        self.phases.append((self.tick, name))

    # -- motion ---------------------------------------------------------------------------
    def emit(self):
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.aperture[a], 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def solve(self, arm, point, rotation, seed=None, iters=40):
        joints, error, _ = ik.solve(point, ik.matrix_to_quat_xyzw(rotation),
                                    self.joints[arm] if seed is None else np.asarray(seed, float), BASES[arm], iters=iters)
        self.worst_ik = max(self.worst_ik, error)
        stage = self.phases[-1][1] if self.phases else "start"
        if error > self.log["worst_ik_mm_by_phase"].get(stage, 0.) / 1000:
            self.log["worst_ik_mm_by_phase"][stage] = round(1000 * float(error), 2)
        return joints

    def command(self, arm, point, rotation):
        self.target[arm] = (np.asarray(point, dtype=float).copy(), np.asarray(rotation, dtype=float).copy())
        self.joints[arm] = self.solve(arm, *self.target[arm])

    def hold(self, seconds):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield self.emit()

    def glide(self, arm, seconds, goal):
        start = self.joints[arm].copy()
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            a = step / steps
            a *= a * (3. - 2. * a)
            self.joints[arm] = start + a * (np.asarray(goal) - start)
            yield self.emit()
        self.target[arm] = None

    def line(self, arm, seconds, point, rotation=None):
        if self.target[arm] is None:
            self.target[arm] = self.hand(arm)
        p0, r0 = self.target[arm]
        rotation = r0 if rotation is None else rotation
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            a = step / steps
            a *= a * (3. - 2. * a)
            self.command(arm, p0 + a * (np.asarray(point) - p0), rotation)
            yield self.emit()

    def trim(self, arm, point, repeats=3):
        """Shift the command until the fingertip centre, by the joint readings, is at ``point``."""
        if self.target[arm] is None:
            self.target[arm] = self.hand(arm)
        for _ in range(repeats):
            p, r = self.target[arm]
            shift = np.clip(np.asarray(point) - self.hand(arm)[0], -.008, .008)
            if np.linalg.norm(shift) < .0015 or np.linalg.norm(p + shift - point) > .03:
                break
            yield from self.line(arm, .35, p + shift, r)
            yield from self.hold(.1)

    def grip(self, arm, finger, seconds):
        start = self.aperture[arm]
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            self.aperture[arm] = start + step / steps * (finger - start)
            yield self.emit()

    def servo(self, arm, part, goal, seconds, *, speed=.04, tolerance=.001, good=3, label="", gain=SERVO_GAIN,
              every=SERVO_EVERY, min_seconds=0., square=False):
        """Drive a held part's origin to ``goal()`` by feedback on its measured position.

        Move and settle: a moving goal is followed feed-forward every step; the measured error is corrected only
        every ``every`` steps, by ``gain`` of it (speed-limited), so that the arm's lag of two to three control
        steps cannot turn the loop into an oscillation (run 1: a correction of 35 % every step made the held
        funnel and clamp shake by 2 to 4 mm). With ``square`` the hand is also turned, about the part's origin,
        by ``gain`` of the angle between the part's axis and the holder's (the wrist sags by a few degrees).
        Returns True once within tolerance for ``good`` consecutive checks."""
        streak = 0
        if self.target[arm] is None:
            self.target[arm] = self.hand(arm)
        earliest = round(min_seconds * CONTROL_HZ)
        previous = np.asarray(goal(), dtype=float)
        for count in range(max(1, round(seconds * CONTROL_HZ))):
            want = np.asarray(goal(), dtype=float)
            point, rotation = self.target[arm]
            point = point + (want - previous)                      # feed-forward of the goal's own motion
            previous = want
            now, rot = self.part(part)
            error = want - now
            tilt_vec = np.zeros(3)
            if square:
                tilt_vec = np.cross(rot[:, 2], self.part("holder")[1][:, 2])
            if count % every == 0:
                inside = np.linalg.norm(error) < tolerance and (not square or np.linalg.norm(tilt_vec) < np.sin(SQUARE_TOL))
                streak = streak + 1 if inside else 0
                if streak >= good and count >= earliest:
                    self.target[arm] = (point, rotation)
                    return True
                step = gain * error
                size = float(np.linalg.norm(step))
                if size > speed * DT * every:
                    step *= speed * DT * every / size
                point = point + step
                if square and np.linalg.norm(tilt_vec) > 1e-6:
                    turn = Rotation.from_rotvec(gain * tilt_vec).as_matrix()
                    point = now + turn @ (point - now)
                    rotation = turn @ rotation
            self.command(arm, point, rotation)
            if self.tick % 2 == 0:
                self.log["servo"].append([self.tick, label, arm, *(round(1000 * float(v), 2) for v in error),
                                          round(L.tilt_deg(rot), 2), round(self.opening_mm(arm), 1)])
            yield self.emit()
        return False

    # -- the funnel -----------------------------------------------------------------------
    def take_funnel(self):
        arm = "left"
        for attempt in range(1, 3):
            self.phase(f"funnel reach {attempt}")
            yield from self.grip(arm, 1., .3)
            fp, fr = self.part("funnel")
            grasp = fp + fr @ np.array((0., 0., GRIP_Z)) + (attempt - 1) * np.array((0., 0., -.004))
            back = grasp - R_FUNNEL[:, 2] * .045
            yield from self.glide(arm, 2.4, self.solve(arm, back, R_FUNNEL, seed=SEEDS["funnel"], iters=300))
            self.target[arm] = (back, R_FUNNEL)
            yield from self.hold(.3)
            yield from self.trim(arm, back)
            self.phase(f"funnel approach {attempt}")
            fp, fr = self.part("funnel")
            grasp = fp + fr @ np.array((0., 0., GRIP_Z)) + (attempt - 1) * np.array((0., 0., -.004))
            yield from self.line(arm, 1.2, grasp + (self.target[arm][0] - self.hand(arm)[0]), R_FUNNEL)
            yield from self.trim(arm, grasp, repeats=2)
            self.phase(f"funnel close {attempt}")
            yield from self.grip(arm, FUNNEL_FINGER, .8)
            yield from self.hold(.4)
            fp, fr = self.part("funnel")
            point, rotation = self.hand(arm)
            held = self.opening_mm(arm) > FUNNEL_HELD_MM
            self.log["grasps"].append({"part": "funnel", "attempt": attempt, "tick": self.tick, "holding": bool(held),
                                       "opening_mm": round(self.opening_mm(arm), 2),
                                       "funnel_in_hand_mm": [round(1000 * float(v), 2) for v in rotation.T @ (fp - point)]})
            if held:
                return True
            self.event("funnel grip closed on nothing; trying again")
            yield from self.grip(arm, 1., .4)
            yield from self.line(arm, .8, self.target[arm][0] - R_FUNNEL[:, 2] * .04)
        return False

    def seat_funnel(self):
        """Carry the funnel over the base, centre it there, lower it straight down and press it on."""
        arm = "left"
        self.phase("funnel lift")
        point, rotation = self.target[arm]
        yield from self.line(arm, 1.4, point + (0., 0., .135), rotation)
        self.phase("funnel carry")
        # up and across first, so that the funnel is above the flange's height before it comes near the holder
        yield from self.servo(arm, "funnel", lambda: VIA, 4., speed=.12, tolerance=.004, label="via")
        above = lambda: self.joint_point() + (0., 0., ABOVE)
        yield from self.servo(arm, "funnel", above, 5., speed=.12, tolerance=.002, label="carry")
        for attempt in range(1, SEAT_TRIES + 1):
            self.phase(f"funnel align {attempt}")
            # centre it 5 mm over the joint and turn the hand until the funnel's axis is the holder's
            yield from self.servo(arm, "funnel", lambda: self.joint_point() + (0., 0., .005), 4., speed=.02,
                                  tolerance=.0005, good=3, label="align", square=True)
            self.phase(f"funnel lower {attempt}")
            # straight down at 8 mm/s with no sideways correction (sliding on the membrane would crease it)
            point, rotation = self.target[arm]
            contact = None
            for k in range(1, round(2.5 * CONTROL_HZ) + 1):
                self.command(arm, point - (0., 0., .008 * DT * k), rotation)
                yield self.emit()
                if self.seat()["gap"] < .0005:
                    contact = self.target[arm][0].copy()
                    break
            if contact is None:
                contact = self.target[arm][0].copy()
                self.event("funnel did not reach the base while lowering", gap_mm=round(1000 * self.seat()["gap"], 2))
            # loosen the pinch so that the ground faces can square up under the press, then grip again
            yield from self.grip(arm, LIGHT_FINGER, .4)
            yield from self.line(arm, .6, contact - (0., 0., PRESS), rotation)
            yield from self.hold(.5)
            loose = self.seat()
            yield from self.grip(arm, FUNNEL_FINGER, .5)
            yield from self.hold(.3)
            seat = self.seat()
            self.log["seatings"].append({"attempt": attempt, "tick": self.tick, "offset_mm": round(1000 * seat["offset"], 2),
                                         "gap_mm": round(1000 * seat["gap"], 2), "tilt_deg": round(seat["tilt_deg"], 2),
                                         "tilt_loose_deg": round(loose["tilt_deg"], 2),
                                         "funnel_on_base_n": self.state.get("funnel_on_holder_n")})
            if seat["gap"] < .001 and seat["offset"] < .0011 and seat["tilt_deg"] < 2.2:
                return True
            if attempt == SEAT_TRIES:
                break                          # leave it down where it is rather than hover over the base
            self.event("funnel not square or off centre; lifting it to set it down again",
                       offset_mm=round(1000 * seat["offset"], 2), tilt_deg=round(seat["tilt_deg"], 2))
            point, rotation = self.target[arm]
            yield from self.line(arm, .6, point + (0., 0., .012 + PRESS), rotation)
        return False

    # -- the clamp ------------------------------------------------------------------------
    def take_clamp(self):
        arm = "right"
        for attempt in range(1, 3):
            self.phase(f"clamp reach {attempt}")
            yield from self.grip(arm, 1., .3)
            cp, cr = self.part("clamp")
            rotation = L.rz(yaw_of(cr)) @ R_CLAMP
            grasp = cp + cr @ L.GRIP_IN_CLAMP + (attempt - 1) * cr @ np.array((.003, 0., -.002))
            back = grasp - rotation[:, 2] * .05
            yield from self.glide(arm, 2.4, self.solve(arm, back, rotation, seed=SEEDS["clamp"], iters=300))
            self.target[arm] = (back, rotation)
            yield from self.hold(.3)
            yield from self.trim(arm, back)
            self.phase(f"clamp approach {attempt}")
            yield from self.line(arm, 1.2, grasp + (self.target[arm][0] - self.hand(arm)[0]), rotation)
            yield from self.trim(arm, grasp, repeats=2)
            self.phase(f"clamp squeeze {attempt}")
            yield from self.grip(arm, 0., 1.0)
            yield from self.hold(.5)
            jaws = self.state["jaw_open_deg"]
            cp, cr = self.part("clamp")
            point, hand_rot = self.hand(arm)
            ok = min(jaws) >= CLAMP_SQUEEZED_DEG
            self.log["grasps"].append({"part": "clamp", "attempt": attempt, "tick": self.tick, "squeezed": bool(ok),
                                       "jaw_open_deg": [round(v, 2) for v in jaws], "opening_mm": round(self.opening_mm(arm), 2),
                                       "clamp_in_hand_mm": [round(1000 * float(v), 2) for v in hand_rot.T @ (cp - point)],
                                       "clamp_tilt_deg": round(L.tilt_deg(cr), 2)})
            if ok:
                return True
            self.event("clamp not squeezed open; trying again", jaw_open_deg=jaws)
            yield from self.grip(arm, 1., .5)
            yield from self.line(arm, .8, self.target[arm][0] - rotation[:, 2] * .05)
        return False

    def fit_clamp(self):
        arm = "right"
        self.phase("clamp lift")
        point, rotation = self.target[arm]
        yield from self.line(arm, 1.2, point + (0., 0., .13), rotation)
        self.phase("clamp carry")
        behind = lambda: self.joint_point() + (-BEHIND, 0., 0.)
        yield from self.servo(arm, "clamp", behind, 5., speed=.12, tolerance=.002, label="carry clamp")
        self.phase("clamp line up")
        yield from self.servo(arm, "clamp", behind, 2.5, speed=.02, tolerance=.0007, good=4, label="line up")
        self.phase("clamp push on")
        clock = {"d": BEHIND}

        def moving():
            clock["d"] = max(0., clock["d"] - .015 * DT)
            return self.joint_point() + (-clock["d"], 0., 0.)

        yield from self.servo(arm, "clamp", moving, BEHIND / .015 + 2.5, speed=.03, tolerance=.0008, good=5,
                              label="push on", min_seconds=BEHIND / .015)
        ring = self.ring()
        self.log["fittings"].append({"stage": "pushed on", "tick": self.tick, "off_axis_mm": round(1000 * ring["off_axis"], 2),
                                     "height_mm": round(1000 * ring["height"], 2), "tilt_deg": round(ring["tilt_deg"], 2),
                                     "jaw_open_deg": [round(v, 2) for v in ring["jaw_open_deg"]],
                                     "funnel_offset_mm": round(1000 * self.seat()["offset"], 2)})
        self.phase("clamp release")
        yield from self.grip(arm, RELEASE_CLAMP[0], 1.5)
        yield from self.grip(arm, RELEASE_CLAMP[1], .5)
        yield from self.hold(.5)
        ring = self.ring()
        self.log["fittings"].append({"stage": "released", "tick": self.tick, "off_axis_mm": round(1000 * ring["off_axis"], 2),
                                     "height_mm": round(1000 * ring["height"], 2), "tilt_deg": round(ring["tilt_deg"], 2),
                                     "jaw_open_deg": [round(v, 2) for v in ring["jaw_open_deg"]], "closed": ring["jaws_closed"],
                                     "funnel_offset_mm": round(1000 * self.seat()["offset"], 2)})
        self.phase("clamp hand out")
        yield from self.grip(arm, .9, .3)                  # pads 85 mm apart: clear of the handles' spread ends
        point, rotation = self.target[arm]
        yield from self.line(arm, 1.0, point + (-.05, 0., .008), rotation)
        # then out sideways, away from the clamp's handles, before swinging home in joint space (run 2: the
        # joint-space swing from just behind the handles hooked one and dragged the clamped holder 34 mm)
        joint = self.joint_point()
        for offset in RETREAT:
            yield from self.line(arm, .9, joint + offset, rotation)
        yield from self.glide(arm, 2.2, np.asarray(READY, dtype=float))
        yield from self.grip(arm, 1., .3)
        return ring["jaws_closed"]

    def proof_lift(self):
        """Move only the hand. Read the free flask and the funnel independently."""
        arm = "left"
        self.phase("proof lift by upper funnel")
        mark_xy = HOLDER_START[:2]
        # The extra load pitches the soft wrist. Correct absolute vertical rather than
        # relative alignment between two pieces that can lean together.
        target_clearance = (LIFT_HEIGHT + MAX_LIFT_HEIGHT) / 2
        yaw = 0.
        airborne = False
        held_ticks = 0
        # A slow preload can use the remaining episode time after reserving the full set-down and release.
        release_steps = sum(round(seconds * CONTROL_HZ) for seconds in FUNNEL_RELEASE_TIMES)
        finish_steps = LOWER_STEPS + round(LOWER_SETTLE_S * CONTROL_HZ) + release_steps + RELEASE_TICKS
        proof_steps = max(0, MAX_STEPS - self.tick - finish_steps)
        # Read the result of the last allowed command before deciding how the lift ended.
        for k in range(proof_steps + 1):
            foot, body = self.part("holder")
            gap = flask_clearance(foot, body)
            if self.seat()["gap"] > SEAT_GAP:
                self.event("proof lift stopped: glass flanges separated")
                break
            if gap > MAX_LIFT_HEIGHT:
                self.event("proof lift stopped: flask above gauge", foot_clearance_mm=1000 * gap)
                break
            airborne = airborne or gap > .003
            held_ticks = held_ticks + 1 if LIFT_HEIGHT <= gap <= MAX_LIFT_HEIGHT else 0
            if held_ticks == 1:
                self.phase("proof hold")
            if held_ticks >= 28:
                break
            if k == proof_steps:
                self.event("proof lift step budget exhausted", foot_clearance_mm=round(1000 * gap, 2),
                           held_ticks=held_ticks)
                break
            if k % 4 == 0:
                point, rotation = self.target[arm]
                # The flask foot can drift while the funnel stays near the mark.
                xy_error = mark_xy - foot[:2]
                xy = np.clip(.45 * xy_error, -.0012, .0012)
                dz = np.clip(.45 * (target_clearance - gap), -.0012, .0026)
                if abs(gap) <= TABLE_TOL:
                    # Leave room for motion lag before reaching the grader's mark tolerance.
                    margin = max(0., 1. - np.linalg.norm(xy_error) / (MARK_TOL / 2))
                    dz = min(dz, .0012 * margin)
                # In flight a modest yaw draws the elbow inward and keeps the loaded workspace reachable.
                turn = .32 * np.cross(body[:, 2], (0., 0., 1.))
                size = np.linalg.norm(turn)
                if size > np.radians(.65):
                    turn *= np.radians(.65) / size
                dyaw = min(np.radians(.7), np.radians(16.) - yaw) if airborne else 0.
                yaw += dyaw
                rotation = Rotation.from_rotvec(turn + (0., 0., dyaw)).as_matrix() @ rotation
                self.command(arm, point + np.r_[xy, dz], rotation)
            yield self.emit()
        self.event("proof hold ended", foot_clearance_mm=round(1000 * gap, 2), held_ticks=held_ticks)
        self.phase("lower assembly")
        # Descend onto the original foot mark and maintain world-up orientation.
        for k in range(LOWER_STEPS):
            foot, body = self.part("holder")
            gap = flask_clearance(foot, body)
            if gap < .0005 and L.tilt_deg(body) < 1.5:
                break
            if k % 4 == 0:
                point, rotation = self.target[arm]
                xy = np.clip(.35 * (mark_xy - foot[:2]), -.001, .001)
                dz = -np.clip(.40 * gap, .00035, .0024)
                turn = np.clip(.22 * np.cross(body[:, 2], (0., 0., 1.)), -.009, .009)
                self.command(arm, point + np.r_[xy, dz], Rotation.from_rotvec(turn).as_matrix() @ rotation)
            yield self.emit()
        yield from self.hold(LOWER_SETTLE_S)

    def let_go_of_funnel(self):
        arm = "left"
        open_s, settle_s, withdraw_s, home_s, open_fully_s = FUNNEL_RELEASE_TIMES
        self.phase("funnel release")
        yield from self.grip(arm, .88, open_s)
        yield from self.hold(settle_s)
        point, rotation = self.target[arm]
        yield from self.line(arm, withdraw_s, point - rotation[:, 2] * .06, rotation)
        self.phase("funnel hand home")
        yield from self.glide(arm, home_s, np.asarray(READY, dtype=float))
        yield from self.grip(arm, 1., open_fully_s)

    def sequence(self):
        yield from self.hold(.5)
        if not (yield from self.take_funnel()):
            self.event("could not take the funnel")
        elif not (yield from self.seat_funnel()):
            self.event("could not seat the funnel centred")
        if VARIANT == "no_hold":
            self.event("control run: the funnel is let go before the clamp is fitted")
            yield from self.let_go_of_funnel()
        if (yield from self.take_clamp()):
            closed = yield from self.fit_clamp()
            if not closed:
                self.event("clamp did not close on the flanges")
        else:
            self.event("could not take the clamp")
            yield from self.glide("right", 2., np.asarray(READY, dtype=float))
        if VARIANT != "no_hold":
            if self.ring()["jaws_closed"]:
                yield from self.proof_lift()
            yield from self.let_go_of_funnel()
        self.phase("hands off")
        while True:
            yield self.emit()
