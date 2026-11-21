"""Full-score demonstration.

Left hand, three times (spotter k, sample k, tick k): pinch the spotter standing in the rack from above, lift it
out, dip its tip 4 mm under the sample's surface for 0.4 s, lift it out, bring the tip 1 mm over its tick, dab it
down (the tip lands and presses lightly) and lift it once the spot is 2.3 mm across, take the spotter back over its
hole along the rack's row and lower it in, let go.

Right hand: pinch the plate's two long edges at its top end, only the last 7 mm of them between the pads and firmly
enough to lift it (1 mm narrower than the plate, about 3 N on the pads), so the pads hold the plate up but let it
turn about the line between them; lift: the plate's bottom end slides along the bar and the plate swings down until
it hangs from the pads, spotted face toward the robot, 14 to 32 degrees off vertical (the pinch's friction holds it
there; a lighter pinch lets it slide out). Stand its bottom edge on the bench just behind the bar's far end and move
the hand toward the robot: the bar's end face stops the bottom edge and the plate turns in the pinch until it is 4
degrees short of upright; lift it. Carry it into the tank and stand its bottom edge on the floor 35 mm in front of
the back wall, so that the pads (26 mm wide, standing out 23 to 30 mm behind the plate's top) clear the wall, then
open the pads: the plate settles back against the back wall. Then pinch the lid across its width, set it on the
tank, let go, and go back.

It reads the robot's joint and gripper readings and, as the review rules allow for a scripted demonstration,
simulator state: the bodies' poses, each spotter's contact force on the plate and what the rules have recorded. It
changes nothing in the simulation: everything moves only because the fingers move it.

``RDB_TLC_DEMO`` (or ``variant``) selects a part or a control: "plate_only" skips the spotting (at most 3/12),
"spot_only" and "one_lane" only spot (probes), "press_hard" presses the second spotter until the force passes 1.5 N
(that lane gouged), "no_dip" touches without filling (no spots).
"""

import os

from inspect_robots import Action, ActionChunk
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, READY

from . import fixed_scene as fixture
from . import layout as L
from . import tlc
from .fixed_scene import ARMS, CONTROL_HZ

SPAN = 2 * abs(float(FINGER_OPEN))     # pads' separation at a gripper command of 1.0
DT = 1. / CONTROL_HZ
TIP = float(PAD_TIP_PAST_TCP)          # the pad tips are this far past the tool point
PAD_HALF_WIDTH = .0132                 # the pad's tip region is 26.4 mm wide (YAM finger hulls), 15.8 mm long
BASES = {arm: np.asarray(fixture.BASES[arm], dtype=float) for arm in ARMS}
SIDE = {"left": 1., "right": -1.}
SEEDS = [np.array(s, dtype=float) for s in (
    (0, 1, 1, -.6, 0, 0), (-.35, 2.2, .8, 1.4, 1.2, -1.57), (.35, 2.2, .8, 1.4, -1.2, 0), (0, 2.1, 1.0, -.4, 1.5, -.1),
    (.4, 2.5, 1.4, -1.6, 1.1, 0), (0, 1.6, 1.3, -1.2, 0, 0))]
DOWN = np.array((0., 0., -1.))

# -- spotters -------------------------------------------------------------------------------------------------------
GRIP_UP = .050                         # the pad tips this far above the spotter's tip
TCP_UP = GRIP_UP + TIP                 # the tool point this far above the spotter's tip
CAP_OPEN = .020                        # pads open around a spotter
CAP_SHUT = 0.                          # pads shut on it (1.5 mm between them: about 3.7 N a pad)
TRAVEL_TIP = .058                      # carried spotter's tip height between stations (vials 45 mm, rack 25 mm)
DIP_DEPTH = .004                       # tip this far under the sample's surface
DIP_S = .4
FACE_GAP = .003                        # approach the silica to this far above it, line up, settle (the slow arm
                                       # wanders sideways when it creeps down: a slow press landed 1.2 mm off its tick)
PRESS_RATE = .001                      # the press_hard control comes down at three times this
TOUCH_HOLD_S = .20                     # stay down this long: spots about 2.3 mm
DAB_FROM = .001                        # the dab starts with the tip this far above the silica ...
DAB_PAST = .0012                       # ... and is commanded this far below it ...
DAB_STEPS = 2                          # ... in this many steps; then the tip is kept pressing between ...
HOLD_LOW_N, HOLD_HIGH_N = .08, .7      # ... these forces (nudged down or up)
NUDGE_DOWN, NUDGE_UP = .00025, .00015
DAB_MAX_STEPS = 16
SPOT_STOP_MM = 2.3                      # lift once the spot is this wide (it ends about 2.4 to 2.6 mm)
HOLE_TIP = .0015                       # lower a returned spotter until its tip is this far above the hole's floor
FRONT_X = .205                         # spotters leave and reach the rack along its row, from this far forward
# -- the plate ------------------------------------------------------------------------------------------------------
OVERLAP = .007                         # the pads hold the plate's last 7 mm (the pad's tip corners are rounded)
PINCH_BELOW = .006                     # pad tips this far below the plate's lower face
PLATE_OPEN = .035
PLATE_SQUEEZE = float(os.environ.get("RDB_TLC_SQUEEZE", ".001"))   # pads commanded this much closer than the plate is wide
                                       # (a 0.2 mm squeeze gave only 0.03 N in PhysX: the plate slid out)
CARRY_Z = .190                         # tool point height over the tank (the hanging plate clears its 108 mm rim)
APPROACH_Y = .095                      # the hanging plate comes to the tank's left side (60.5 mm from its middle), low,
                                       # rises there, then moves over it
BAR_BEHIND = .004                      # the plate's bottom edge goes down this far behind the bar's far end ...
STRAIGHT_DEG = 4.                      # ... and the hand moves toward the robot until the plate is this near upright
STRAIGHT_RATE = .008                   # m/s
FINGER_BACK = .0317                    # the pads' widest part inside the tank stands out this far behind the plate's top
                                       # (29.7 mm) plus 2 mm: the plate's top is let go at least this far from the wall
TIP_BACK_MIN = .0025                   # the plate is let go at least this far tipped back (top behind bottom, 2 degrees)
FLOOR_PRESS = .001                     # press the plate's bottom edge onto the floor this much
# -- the lid ---------------------------------------------------------------------------------------------------------
LID_OPEN = .0939                       # pads fully open (the lid is 87 mm wide)
LID_SQUEEZE = .002
LID_PINCH_Z = .0015                    # pad tips this far above the table when pinching the lid's edges (4 mm thick)
LID_DROP = .0015                       # let go this far above the rim


def grip_frame(approach, closing):
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float)
    c = c - a * (c @ a)
    c /= np.linalg.norm(c)
    return np.column_stack((c, np.cross(a, c), a))


def down(closing):
    return grip_frame(DOWN, closing)


ALONG_X = [down((1., 0., 0.)), down((-1., 0., 0.))]
ALONG_Y = [down((0., 1., 0.)), down((0., -1., 0.))]
ANY = ALONG_X + ALONG_Y + [down((np.cos(a), np.sin(a), 0.)) for a in np.radians((45., 135., -45., -135.))]


class TLCReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self, variant=None):
        self.variant = variant or os.environ.get("RDB_TLC_DEMO", "full")
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float).copy() for arm in ARMS}
        self.aperture = dict.fromkeys(ARMS, SPAN)
        self.cmd = dict.fromkeys(ARMS)
        self.rot = dict.fromkeys(ARMS)
        self.offset = {arm: np.zeros(3) for arm in ARMS}
        self.worst_ik = 0.
        self.worst_ik_at = None
        self.phases = []
        self.tick = 0
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
        return float(self.measured[7 * ARMS.index(arm) + 6]) * SPAN

    def pose(self, name):
        p, q = self.state["poses"][name]
        return np.asarray(p, dtype=float), np.asarray(q, dtype=float)

    def tip(self, k):
        return np.asarray(tlc.spotter_tip(self.pose(fixture.SPOTTERS[k])))

    def force(self, k):
        forces = self.state["forces"]
        return 0. if forces is None else float(forces["plate"][k])

    def note(self, name, **values):
        self.log["events"].append({"tick": self.tick, "event": name,
                                   **{k: (v if isinstance(v, str) else round(float(v), 4) if np.isscalar(v)
                                          else [round(float(x), 4) for x in v]) for k, v in values.items()}})

    def phase(self, name):
        self.phases.append((self.tick, name))

    # -- motion ---------------------------------------------------------------------------------------------------
    def emit(self):
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.aperture[a] / SPAN, 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def hold(self, seconds):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield self.emit()

    def glide(self, seconds, targets, apertures=None):
        """Arms in joint space to the given joints, grippers to the given pad separations (smooth start and stop)."""
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

    def grip(self, arm, separation, seconds=.4):
        yield from self.glide(seconds, {}, {arm: separation})

    def pose_joints(self, arm, point, rotations):
        """Joints for a pose from several seeds and any of the equivalent ``rotations``: the solution furthest from
        the joint limits (the ukulele package's pose_joints)."""
        best = None
        for k, rot in enumerate(rotations):
            for seed in SEEDS:
                seed = seed * np.array((SIDE[arm], 1, 1, 1, SIDE[arm], SIDE[arm]))
                joints, error, turn = ik.solve(point, ik.matrix_to_quat_xyzw(rot), seed, BASES[arm], iters=150)
                if error < 5e-4 and turn < 5e-3:
                    margin = float(np.minimum(joints - ik.LOWER, ik.UPPER - joints).min())
                    if best is None or margin > best[1]:
                        best = (np.asarray(joints, dtype=float), margin, k)
        if best is None:
            raise RuntimeError(f"{arm} arm cannot reach {np.round(point, 4)}")
        self.note("pose", arm=arm, joint_margin_deg=np.degrees(best[1]), rotation=best[2])
        return best[0].copy(), rotations[best[2]]

    def solve(self, arm, point, rot):
        joints, error, _ = ik.solve(point, ik.matrix_to_quat_xyzw(rot), self.joints[arm], BASES[arm], iters=60)
        if error > self.worst_ik:
            self.worst_ik = float(error)
            self.worst_ik_at = (self.tick, self.phases[-1][1] if self.phases else "", arm, [round(float(v), 4) for v in point])
        return np.asarray(joints, dtype=float).copy()

    def go(self, arm, point, rotations, seconds):
        """Joint-space move to a pose (for long moves), the best of ``rotations``."""
        q, rot = self.pose_joints(arm, np.asarray(point, dtype=float) + self.offset[arm], rotations)
        yield from self.glide(seconds, {arm: q})
        self.cmd[arm], self.rot[arm] = np.asarray(point, dtype=float).copy(), rot

    def line(self, arm, point, seconds):
        """Straight-line move of the tool point (smooth start and stop), keeping the hand's rotation."""
        p0 = self.cmd[arm].copy()
        p1 = np.asarray(point, dtype=float)
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            self.cmd[arm] = p0 + alpha * (p1 - p0)
            self.joints[arm] = self.solve(arm, self.cmd[arm] + self.offset[arm], self.rot[arm])
            yield self.emit()

    def step_to(self, arm, point):
        """One control step's move of the tool point."""
        self.cmd[arm] = np.asarray(point, dtype=float).copy()
        self.joints[arm] = self.solve(arm, self.cmd[arm] + self.offset[arm], self.rot[arm])
        yield self.emit()

    def settle(self, arm, want, tries=3):
        """Shift the arm's command by its measured error until its tool point is on ``want`` (the arms sag)."""
        for _ in range(tries):
            yield from self.hold(.25)
            error = want - self.actual(arm)
            if np.linalg.norm(error) < .0008:
                break
            self.offset[arm] = np.clip(self.offset[arm] + np.clip(error, -.004, .004), -.02, .02)
            yield from self.line(arm, want, .25)
        self.note("settled", arm=arm, error_mm=1000 * (want - self.actual(arm)), offset_mm=1000 * self.offset[arm])

    def align(self, arm, measure, want, tries=3, tolerance=.0004):
        """Move the hand so that the point ``measure()`` (read from the simulator: a spotter's tip, the plate's
        bottom edge) comes over ``want`` in plan; heights are left to the caller."""
        for _ in range(tries):
            yield from self.hold(.2)
            error = np.asarray(want, dtype=float) - measure()
            error[2] = 0.
            if np.linalg.norm(error) < tolerance:
                break
            yield from self.line(arm, self.cmd[arm] + np.clip(error, -.006, .006), .25)
        self.note("aligned", arm=arm, error_mm=1000 * (np.asarray(want, dtype=float) - measure()))

    # -- spotting -------------------------------------------------------------------------------------------------
    def spot_lane(self, k):
        arm = "left"
        hole = np.asarray(L.hole_point(k))
        tip0 = self.tip(k)
        self.phase(f"lane {k + 1}: pick spotter {k + 1}")
        over = np.array((tip0[0], tip0[1], tip0[2] + TCP_UP + .060))
        if self.cmd[arm] is None:
            yield from self.go(arm, over, ALONG_X, 2.4)
        else:
            yield from self.line(arm, (FRONT_X, tip0[1], over[2]), .8)
            yield from self.line(arm, over, .8)
        yield from self.grip(arm, CAP_OPEN, .2)
        grip = np.array((tip0[0], tip0[1], tip0[2] + TCP_UP))
        yield from self.line(arm, grip + (0., 0., .015), .9)
        yield from self.settle(arm, grip + (0., 0., .015), tries=2)
        yield from self.line(arm, grip, .6)
        yield from self.grip(arm, CAP_SHUT, .4)
        yield from self.hold(.15)
        self.note("spotter pinched", lane=k + 1, pads_mm=1000 * self.opening(arm))
        # lift it out slowly (the arm wanders sideways when it moves fast) and take it forward along the rack's row
        yield from self.line(arm, self.cmd[arm] + (0., 0., .040), 1.6)
        yield from self.line(arm, (FRONT_X, self.cmd[arm][1], TRAVEL_TIP + TCP_UP), .8)
        # dip
        if self.variant != "no_dip":
            self.phase(f"lane {k + 1}: fill")
            vial = np.asarray(L.vial_point(k))
            yield from self.line(arm, (vial[0], vial[1], TRAVEL_TIP + TCP_UP), .9)
            yield from self.align(arm, lambda: self.tip(k), vial)
            # to just over the neck, line up again, then in slowly (8.5 mm neck, 1.5 mm spotter)
            yield from self.line(arm, self.cmd[arm] + (0., 0., L.VIAL_H + .005 - self.tip(k)[2]), .5)
            yield from self.align(arm, lambda: self.tip(k), vial, tries=3, tolerance=.00025)
            yield from self.line(arm, self.cmd[arm] + (0., 0., vial[2] - DIP_DEPTH - self.tip(k)[2]), 2.4)
            yield from self.hold(DIP_S)
            self.note("dipped", lane=k + 1, tip_mm=1000 * self.tip(k), load=str(self.state["grade"]["loads"][k]))
            yield from self.line(arm, self.cmd[arm] + (0., 0., L.VIAL_H + .005 - self.tip(k)[2]), 2.0)
            yield from self.line(arm, self.cmd[arm] + (0., 0., TRAVEL_TIP - self.tip(k)[2]), .5)
        # spot: over the tick, down to 3 mm above the silica, settle, line up; to 1 mm above, settle, line up again;
        # then a quick dab (the procedure's "quick up-and-down"): 2.2 mm down in 0.1 s, so the tip lands and presses
        # about 1 mm into the arm's give, 0.2 s, up. The scorer counts the touch only between 0.05 and 1 N.
        self.phase(f"lane {k + 1}: spot")
        plate = self.pose("plate")
        tick = np.asarray(tlc.to_world(plate, L.tick_point(k)))
        yield from self.line(arm, self.cmd[arm] + (tick[0] - self.tip(k)[0], tick[1] - self.tip(k)[1], 0.), 1.2)
        yield from self.line(arm, self.cmd[arm] + (0., 0., tick[2] + FACE_GAP - self.tip(k)[2]), 1.0)
        yield from self.hold(.3)
        yield from self.align(arm, lambda: self.tip(k), tick, tries=4, tolerance=.00015)
        yield from self.line(arm, self.cmd[arm] + (0., 0., tick[2] + DAB_FROM - self.tip(k)[2]), .6)
        yield from self.hold(.3)
        yield from self.align(arm, lambda: self.tip(k), tick, tries=3, tolerance=.00015)
        start = self.cmd[arm].copy()
        forces = []
        if self.variant == "press_hard" and k == 1:            # control: press on until the tip pushes 1.5 N
            pressed = 0.
            while self.force(k) <= 1.5 and pressed < .016:
                pressed += 3 * PRESS_RATE * DT
                yield from self.step_to(arm, start - (0., 0., pressed))
                forces.append(self.force(k))
            for _ in range(max(1, round(TOUCH_HOLD_S * CONTROL_HZ))):
                yield self.emit()
                forces.append(self.force(k))
        else:
            depth = DAB_FROM + DAB_PAST
            for i in range(DAB_STEPS):
                yield from self.step_to(arm, start - (0., 0., depth * (i + 1) / DAB_STEPS))
                forces.append(self.force(k))
            # stay in light contact: the wrist's stick-slip can leave the tip resting just off the silica after it
            # lands (one run: a 1.8 N landing spike, then nothing), so nudge down while there is no force and up if it
            # presses hard; lift as soon as the spot (read from the rules' state) is SPOT_STOP across (a fixed hold
            # made spots from 1.8 to 3.5 mm on this arm)
            extra = 0.
            for _ in range(DAB_MAX_STEPS):
                lane = self.state["grade"]["lanes"][k]
                if lane["spots"] == 1 and lane["diameter_mm"] is not None and lane["diameter_mm"] >= SPOT_STOP_MM:
                    break
                f = self.force(k)
                extra += NUDGE_DOWN if f < HOLD_LOW_N else (-NUDGE_UP if f > HOLD_HIGH_N else 0.)
                yield from self.step_to(arm, start - (0., 0., depth + extra))
                forces.append(self.force(k))
        yield from self.line(arm, self.cmd[arm] + (0., 0., .012), .3)
        self.note("dab", lane=k + 1, forces_n=forces, tip_off_tick_mm=1000 * (self.tip(k) - tick))
        grade = self.state["grade"]
        self.note("spot", lane=k + 1, **{key: (v if v is not None else -1.) for key, v in grade["lanes"][k].items()
                                         if key in ("diameter_mm", "offset_mm")})
        lane = grade["lanes"][k]
        if self.variant == "full" and not (lane["covered"] and lane["small"] and lane["clean"]):
            # stop the trial at once with its diagnostics rather than finish a run that cannot reach 1.0
            raise RuntimeError(f"lane {k + 1} spot failed: {lane}; dab forces {self.log['events'][-2].get('forces_n')}")
        # back to the rack: over its hole along its row
        self.phase(f"lane {k + 1}: return spotter {k + 1}")
        yield from self.line(arm, self.cmd[arm] + (0., 0., TRAVEL_TIP - self.tip(k)[2]), .5)
        yield from self.line(arm, (FRONT_X, hole[1], self.cmd[arm][2]), 1.4)
        yield from self.line(arm, (hole[0] + (self.cmd[arm][0] - self.tip(k)[0]), hole[1] + (self.cmd[arm][1] - self.tip(k)[1]),
                                   self.cmd[arm][2] - (TRAVEL_TIP - (L.RACK[2] + .010))), 1.0)
        yield from self.align(arm, lambda: self.tip(k), hole, tries=4, tolerance=.0003)
        yield from self.line(arm, self.cmd[arm] + (0., 0., L.RACK[2] + .003 - self.tip(k)[2]), .8)
        yield from self.hold(.3)
        yield from self.align(arm, lambda: self.tip(k), hole, tries=4, tolerance=.0002)
        # down into the hole, stopping if the tip catches on the rack (more than 0.4 N from anything but the pads)
        start, lowered = self.cmd[arm].copy(), 0.
        depth = self.tip(k)[2] - (hole[2] + HOLE_TIP)
        while lowered < depth:
            other = 0. if self.state["forces"] is None else float(self.state["forces"]["other"][k])
            if other > .4:
                self.note("caught on the rack", lane=k + 1, force_n=other, tip_mm=1000 * self.tip(k))
                yield from self.line(arm, self.cmd[arm] + (0., 0., .003), .3)
                yield from self.align(arm, lambda: self.tip(k), hole, tries=3, tolerance=.0002)
                start, lowered = self.cmd[arm].copy(), 0.
                depth = self.tip(k)[2] - (hole[2] + HOLE_TIP)
                continue
            lowered = min(depth, lowered + .010 * DT)
            yield from self.step_to(arm, start - (0., 0., lowered))
        yield from self.hold(.2)
        yield from self.grip(arm, CAP_OPEN, .3)
        yield from self.line(arm, self.cmd[arm] + (0., 0., .065), .9)
        self.note("returned", lane=k + 1, racked=str(self.state["grade"]["racked"][k]))

    # -- the plate ------------------------------------------------------------------------------------------------
    def plate_bottom(self):
        """The middle of the plate's bottom edge, from its pose."""
        return np.asarray(tlc.to_world(self.pose("plate"), (-L.PLATE_LEN / 2, 0., 0.)))

    def take_plate(self):
        arm = "right"
        self.phase("plate: pinch its top end")
        plate = self.pose("plate")
        top = np.asarray(tlc.to_world(plate, (L.PLATE_LEN / 2, 0., -L.PLATE_T / 2)))
        x = top[0] - OVERLAP + PAD_HALF_WIDTH
        grip = np.array((x, top[1], top[2] - PINCH_BELOW + TIP))
        yield from self.go(arm, grip + (0., 0., .045), ALONG_Y, 2.6)
        yield from self.grip(arm, PLATE_OPEN, .3)
        yield from self.settle(arm, grip + (0., 0., .045), tries=2)
        yield from self.line(arm, grip, 1.0)
        yield from self.settle(arm, grip, tries=2)
        yield from self.grip(arm, L.PLATE_WIDE - PLATE_SQUEEZE, .6)
        yield from self.hold(.3)
        self.note("plate pinched", pads_mm=1000 * self.opening(arm))
        self.phase("plate: lift, it swings down")
        yield from self.line(arm, self.cmd[arm] + (0., 0., .090), 3.0)
        yield from self.hold(.6)
        p, q = self.pose("plate")
        self.note("plate hanging", lean_deg=tlc.tilt_deg((p, q), 0), bottom_mm=1000 * self.plate_bottom())
        yield from self.straighten()
        # toward the tank low first (the right arm cannot point down high in front of the midline), then up
        tank = self.pose("tank")[0]
        yield from self.line(arm, (self.cmd[arm][0], tank[1] + APPROACH_Y, self.cmd[arm][2]), 1.6)
        yield from self.line(arm, (self.cmd[arm][0], tank[1] + APPROACH_Y, CARRY_Z), 1.2)

    def plate_up(self):
        """The plate's length direction, bottom to top, on the table."""
        return np.asarray(tlc.axis(self.pose("plate"), 0))

    def tilt(self):
        """How far the plate's top is behind its bottom edge, as an angle (degrees; positive: top toward the back)."""
        up = self.plate_up()
        return float(np.degrees(np.arctan2(up[0], up[2])))

    def straighten(self):
        """The plate hangs 14 to 22 degrees off vertical (a lighter pinch lets it slide out: 0.8 mm dropped it). Stand its
        bottom edge on the table just behind the bar's far end and move the hand toward the robot: the bottom edge slides
        into the bar's end face, which stops it, and the plate turns nearer upright in the pinch. It is stopped about
        STRAIGHT_DEG short of upright, leaning away from the bar, so its silica face never touches the bar (the bar's top
        edge stays a millimetre off the face, the spots 0.9 mm). Then lift it: the pinch's friction keeps the angle."""
        arm = "right"
        self.phase("plate: straighten it against the bar's end")
        end = L.PLATE_X + L.BAR[0] / 2
        bottom = self.plate_bottom()
        yield from self.line(arm, self.cmd[arm] + (end + BAR_BEHIND - bottom[0], 0., 0.), 1.2)
        yield from self.align(arm, self.plate_bottom, np.array((end + BAR_BEHIND, L.PLATE_Y, 0.)), tries=2, tolerance=.0008)
        drop = self.plate_bottom()[2] - .001
        yield from self.line(arm, self.cmd[arm] - (0., 0., drop), 1.2)
        start, lowered = self.cmd[arm].copy(), 0.
        while self.plate_bottom()[2] > .0004 and lowered < .006:
            lowered += .004 * DT
            yield from self.step_to(arm, start - (0., 0., lowered))
        before = self.tilt()
        start, moved = self.cmd[arm].copy(), 0.
        while self.tilt() > STRAIGHT_DEG and moved < .035:
            moved += STRAIGHT_RATE * DT
            yield from self.step_to(arm, start - (moved, 0., 0.))
        yield from self.hold(.3)
        self.note("plate straightened", tilt_before_deg=before, tilt_deg=self.tilt(), moved_mm=1000 * moved,
                  bottom_mm=1000 * self.plate_bottom())
        yield from self.line(arm, self.cmd[arm] + (0., 0., .030), 1.0)
        yield from self.hold(.4)
        self.note("plate lifted", tilt_deg=self.tilt())

    def place_plate(self):
        """Over the tank, with the plate's bottom edge where its top, tipped back as it hangs, keeps FINGER_BACK off the
        back wall (and at least TIP_BACK_MIN behind the bottom edge); down until the bottom edge stands on the floor,
        press, open the pads: the plate settles back against the back wall."""
        arm = "right"
        self.phase("plate: carry it over the tank")
        tank = self.pose("tank")[0]
        wall = tank[0] + L.TANK_IN[0] / 2                         # the back wall's inside face
        lean = max(np.sin(np.radians(self.tilt())) * L.PLATE_LEN, TIP_BACK_MIN)
        want = np.array((wall - FINGER_BACK - lean, tank[1], 0.))
        bottom = self.plate_bottom()
        yield from self.line(arm, self.cmd[arm] + (want[0] - bottom[0], want[1] - bottom[1], 0.), 2.0)
        yield from self.hold(.5)
        yield from self.align(arm, self.plate_bottom, want, tries=3, tolerance=.0006)
        self.note("plate over the tank", tilt_deg=self.tilt(), bottom_to_wall_mm=1000 * (wall - self.plate_bottom()[0]))
        self.phase("plate: lower it in")
        floor = tank[2] + L.TANK_FLOOR
        drop = self.plate_bottom()[2] - (floor + .004)
        yield from self.line(arm, self.cmd[arm] - (0., 0., drop), 2.0)
        start, lowered = self.cmd[arm].copy(), 0.
        while self.plate_bottom()[2] > floor + .0006 and lowered < .010:
            lowered += .004 * DT
            yield from self.step_to(arm, start - (0., 0., lowered))
        yield from self.line(arm, self.cmd[arm] - (0., 0., FLOOR_PRESS), .3)
        yield from self.hold(.3)
        tilt = self.tilt()
        if np.sin(np.radians(tilt)) * L.PLATE_LEN < TIP_BACK_MIN:      # nearly upright: tip it back a little first
            yield from self.line(arm, self.cmd[arm] + (TIP_BACK_MIN - np.sin(np.radians(tilt)) * L.PLATE_LEN, 0., 0.), .6)
            yield from self.hold(.3)
        self.note("plate stands", tilt_deg=self.tilt(), bottom_to_wall_mm=1000 * (wall - self.plate_bottom()[0]),
                  tool_to_wall_mm=1000 * (wall - self.actual(arm)[0]))
        self.phase("plate: let go")
        yield from self.grip(arm, L.PLATE_WIDE + .002, .15)
        yield from self.line(arm, self.cmd[arm] + (0., 0., .030), .5)
        yield from self.hold(1.0)
        state = self.state["grade"]["plate"]
        self.note("plate leans", lean_deg=state["lean_deg"], baseline_above_mm=state["baseline_above_mm"],
                  face_out=float(state["face_out"]), on_wall=float(state["on_wall"]),
                  tank_mm=1000 * (self.pose("tank")[0] - tank))
        yield from self.line(arm, self.cmd[arm] + (0., 0., .060), .8)

    def cover(self):
        arm = "right"
        self.phase("lid: pinch it across its width")
        lid = self.pose("lid")[0]
        grip = np.array((lid[0], lid[1], lid[2] - L.LID[2] / 2 + LID_PINCH_Z + TIP))
        yield from self.go(arm, (grip[0], grip[1], .16), ALONG_X, 2.0)
        yield from self.line(arm, grip + (0., 0., .030), 1.0)
        yield from self.grip(arm, LID_OPEN, .4)
        yield from self.settle(arm, grip + (0., 0., .030), tries=2)
        yield from self.line(arm, grip, .8)
        yield from self.settle(arm, grip, tries=2)
        yield from self.grip(arm, L.LID[0] - LID_SQUEEZE, .6)
        yield from self.hold(.3)
        self.note("lid pinched", pads_mm=1000 * self.opening(arm))
        yield from self.line(arm, self.cmd[arm] + (0., 0., .130), 1.6)
        self.phase("lid: onto the tank")
        tank = self.pose("tank")[0]
        here = self.pose("lid")[0]
        yield from self.line(arm, self.cmd[arm] + (tank[0] - here[0], tank[1] - here[1], 0.), 1.8)
        yield from self.align(arm, lambda: self.pose("lid")[0], tank, tries=3, tolerance=.0008)
        rest = tank[2] + L.TANK[2] + L.LID[2] / 2
        yield from self.line(arm, self.cmd[arm] - (0., 0., self.pose("lid")[0][2] - (rest + LID_DROP)), 1.4)
        yield from self.grip(arm, LID_OPEN, .4)
        yield from self.hold(.5)
        self.note("lid on", **{k: v for k, v in self.state["grade"]["lid"].items() if k != "on"})
        yield from self.line(arm, self.cmd[arm] + (0., 0., .060), .8)

    # -- the demonstration ----------------------------------------------------------------------------------------
    def sequence(self):
        while self.state is None:
            yield self.emit()
        self.note("start")
        if self.variant != "plate_only":
            for k in range(1 if self.variant == "one_lane" else 3):
                yield from self.spot_lane(k)
            self.phase("left hand back")
            yield from self.glide(2.0, {"left": np.asarray(READY, dtype=float)}, {"left": SPAN})
        if self.variant not in ("one_lane", "spot_only"):
            yield from self.take_plate()
            yield from self.place_plate()
            yield from self.cover()
        self.phase("right hand back")
        yield from self.glide(2.0, {"right": np.asarray(READY, dtype=float)}, {"right": SPAN})
        self.done = True
        self.phase("rest")
        while True:
            yield self.emit()
