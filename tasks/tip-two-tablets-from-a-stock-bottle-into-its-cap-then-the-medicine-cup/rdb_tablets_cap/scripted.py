"""Joint-space reference for opening the safety cap, metering the dose and transferring it to the cup."""

from inspect_robots import Action, ActionChunk
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY

from . import grading
from . import layout as L
from .fixed_scene import ARMS, BASES, BOTTLE_XY, CAP_XY, CONTROL_HZ

OPENING_STATION = (.345, .000, .065)
OPENING_TURN_DEG = 48.
TURNOVER_DEG = 122.
TURNOVER_RELEASE_Z = .034
CAP_DOWN_XY = CAP_XY                     # where the cap is put down at the end

OPEN = float(FINGER_OPEN)
DT = 1. / CONTROL_HZ
POUR_AZIMUTH = np.radians(135.)          # the bottle tips toward the robot's left and back, toward the cap's hand
POUR_DIR = np.array((np.cos(POUR_AZIMUTH), np.sin(POUR_AZIMUTH), 0.))
BOTTLE_GRIP_Z = .032                     # grip point on the bottle's axis, above its base
BOTTLE_SQUEEZE = .003                    # each finger commanded this far inside the wall: about 15 N a finger
CAP_GRIP_Z = .016
CAP_SQUEEZE = .002                       # about 10 N a finger
FAST_TILT, SLOW_TILT, BACK_TILT = np.radians(45.), np.radians(8.), np.radians(150.)
FINE_TILT, BACK_TILT_FIRST = np.radians(2.), np.radians(60.)
SETTLE_AT, SETTLE_S = np.radians(105.), 1.0              # hold here once, so the pile settles on the shoulder
BACKOFF, BACKOFF_RATE = np.radians(8.), np.radians(30.)  # a tablet in the neck: tip back this far, this fast
TILT_START, TILT_REST, TILT_MAX = np.radians(70.), np.radians(55.), np.radians(135.)
LIP_LEAD = .012                          # the bottle's lip stops this far short of the cap's centre
LIP_CLEAR = .020                         # the bottle's underside stays this far above the cap's rim (and so clear
                                         # of the fingers holding the cap, whose pads stand 9 mm above the rim)
NECK_WAIT_S = 3.0                        # after this long with a tablet still in the neck, tip slowly further
LIP_COMMITTED = .008                     # a tablet this close to the lip leaves anyway: count it as out
HOLD_AT = (.380, -.080, .060)            # where the bottle is held upright between pours (its base centre)
CAP_HOLD = (.340, .000, .080)            # the left hand holds the cap level here (its underside 80 mm up) for the pour
CAP_PARK = (.290, .090, .110)            # the cap is held here, out of the way, while the bottle is stood down
RETURN_XY = (.420, .000)                 # where the bottle stands, let go, while extra tablets are tipped back into it
# The cap's wall slopes down to its rim only once the cap is tipped past 90 degrees: tablets stay in a dish
# tipped less than that (the first Isaac run held two tablets in the cap at 65 degrees).
CAP_FAST_TO, CAP_FAST, CAP_SLOW, CAP_BACK = np.radians(80.), np.radians(40.), np.radians(8.), np.radians(120.)
CAP_TIP_MAX = np.radians(130.)
CAP_SETTLE_S = 1.5                       # after a tablet has left, hold the tilt this long before tipping on
CAP_STUCK_S = 4.0                        # at the largest tilt, give up after this long
DROP_BOTTLE = .026                       # the cap's inner rim is held this far above the bottle's lip (its underside
                                         # then clears the lip by 9 mm) ...
DROP_CUP = .030                          # ... and this far above the cup's rim (the fingers then clear it)
RIM_BACK = .006                          # ... and this far short of the opening's centre, so tablets slide across it
MAX_POURS = 8
SEEDS = ((0., 1., 1., -.6, 0., 0.), (-.4, 1.4, .8, -.3, .3, .5), (.4, 1.4, .8, -.3, -.3, -.5))


def level_axis(degrees):
    return np.array((np.cos(np.radians(degrees)), np.sin(np.radians(degrees)), 0.))


def grasp_rotation(azimuth, pitch):
    """Gripper orientation whose approach axis points along ``azimuth`` and ``pitch`` below level, fingers
    closing level, wrist camera on top."""
    a = np.array((np.cos(azimuth) * np.cos(pitch), np.sin(azimuth) * np.cos(pitch), -np.sin(pitch)))
    x = np.array((np.sin(azimuth), -np.cos(azimuth), 0.))
    return np.column_stack((x, np.cross(a, x), a))


BOTTLE_GRIP = grasp_rotation(np.radians(105.), np.radians(20.)) @ np.diag((-1., -1., 1.))   # from the right and a little
                                         # behind, 20 degrees below level, wrist camera underneath (keeps joint 6 in range)
OPENING_GRIP = grasp_rotation(np.radians(-108.), np.radians(30.))
CAP_GRIP = grasp_rotation(np.radians(-60.), np.radians(30.))        # from the left and the robot's side, 30 degrees below level
# The cap is tipped about a level axis, its rim on the side (axis x up) going down. 345 degrees is the axis the
# arm can follow to 130 degrees both over the cup and over the bottle's mouth (from the reach study).
TIP_AXIS_CUP = level_axis(345.)           # the rim toward the far side and a little to the left goes down
TIP_AXIS_BOTTLE = level_axis(345.)
LIP_LOCAL = np.array((L.NECK_R_IN * POUR_DIR[0], L.NECK_R_IN * POUR_DIR[1], L.BOTTLE_H))


def tilt_rotation(theta):
    """The bottle tipped ``theta`` from upright toward POUR_DIR."""
    return Rotation.from_rotvec(np.array((-POUR_DIR[1], POUR_DIR[0], 0.)) * theta).as_matrix()


def cap_rotation(tau, axis):
    """The cap tipped ``tau`` about the level ``axis``, its rim on the side axis x up going down."""
    return Rotation.from_rotvec(np.asarray(axis) * tau).as_matrix()


def bottle_underside(theta, reach=.040):
    """How far the bottle's underside hangs below its lip, within ``reach`` behind the lip (the part over the cap)."""
    lowest = 0.
    for z, r in ((L.BOTTLE_H, L.NECK_R_OUT + .0012), (L.SHOULDER_H, L.NECK_R_OUT + .0012), (L.BODY_H, L.BOTTLE_R),
                 (L.BODY_H - .02, L.BOTTLE_R), (L.BODY_H - .04, L.BOTTLE_R), (0., L.BOTTLE_R)):
        dr, dz = r - L.NECK_R_IN, z - L.BOTTLE_H
        back = -(dr * np.cos(theta) + dz * np.sin(theta))
        if back <= reach:
            lowest = min(lowest, -dr * np.sin(theta) + dz * np.cos(theta))
    return -lowest


def lip_height(theta, cap_base=0.):
    """Height of the lip above the table that keeps the bottle clear of the cap under it, whose underside is
    ``cap_base`` above the table."""
    return cap_base + L.CAP_H + LIP_CLEAR + bottle_underside(theta)


class TabletReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self):
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.aperture = dict.fromkeys(ARMS, OPEN)
        self.cmd = {arm: ik.tcp_pose(np.asarray(READY, dtype=float), BASES[arm]) for arm in ARMS}
        self.offset = {arm: np.zeros(3) for arm in ARMS}
        self.grip = {}                                 # arm -> (hand rotation, hand position) in the held object's frame
        self.worst_ik = 0.
        self.phases = []
        self.tick = 0
        self.log = {"pours": [], "grasps": {}, "events": []}
        self.held = {}                                 # arm -> commanded (origin, rotation) of the object it holds
        self.bottle_ok = True

    # -- readings -------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        self.state = state

    def actual_tcp(self, arm):
        start = 7 * ARMS.index(arm)
        return ik.tcp_pose(self.measured[start:start + 6], BASES[arm])

    def opening(self, arm):
        return float(self.measured[7 * ARMS.index(arm) + 6])

    def pose(self, name):
        origin, rotation = self.state[name]
        return np.asarray(origin, dtype=float), np.asarray(rotation, dtype=float)

    def counts(self):
        tablets = self.state["tablets"]
        bottled, capped, cupped = grading.regions(self.state["bottle"], self.state["cap"], self.state["cup"], tablets)
        height = grading._local(self.state["bottle"], tablets)[bottled][:, 2]
        return {"bottle": int(bottled.sum()), "cap": int(capped.sum()), "cup": int(cupped.sum()),
                "neck": int((height > L.SHOULDER_H).sum()),
                "at_lip": int((height > L.BOTTLE_H - LIP_COMMITTED).sum())}

    # -- motion ---------------------------------------------------------------------------
    def emit(self):
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.aperture[a] / OPEN, 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def phase(self, name):
        self.phases.append((self.tick, name))

    def solve(self, arm, position, rotation, seed=None, iters=40):
        joints, error, _ = ik.solve(np.asarray(position) + self.offset[arm], ik.matrix_to_quat_xyzw(rotation),
                                    self.joints[arm] if seed is None else np.asarray(seed, dtype=float), BASES[arm],
                                    iters=iters)
        return joints, error

    def drive(self, arm, position, rotation):
        joints, error = self.solve(arm, position, rotation)
        self.worst_ik = max(self.worst_ik, error)
        self.joints[arm] = joints
        self.cmd[arm] = (np.asarray(position, dtype=float), np.asarray(rotation, dtype=float))
        return error

    def hold(self, seconds):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield self.emit()

    def glide(self, arm, position, rotation, seconds):
        """Joint-space move to a hand pose found from several starting guesses (used from and to the ready pose)."""
        found = [self.solve(arm, position, rotation, seed, iters=300) for seed in (self.joints[arm], *SEEDS)]
        good = [item for item in found if item[1] < .001]
        # Of the solutions that reach the pose, take the one farthest from every joint limit.
        best = max(good, key=lambda item: float(np.minimum(item[0] - ik.LOWER, ik.UPPER - item[0]).min())) if good \
            else min(found, key=lambda item: item[1])
        self.worst_ik = max(self.worst_ik, best[1])
        yield from self.glide_joints(arm, best[0], seconds)
        self.cmd[arm] = (np.asarray(position, dtype=float), np.asarray(rotation, dtype=float))

    def glide_joints(self, arm, goal, seconds):
        start = self.joints[arm].copy()
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            self.joints[arm] = start + alpha * (np.asarray(goal) - start)
            yield self.emit()

    def move(self, arm, position, rotation, seconds, finger=None):
        """Straight-line move of the hand (smooth start and stop), turning evenly; optionally moving the fingers."""
        p0, r0 = self.cmd[arm]
        turn = Slerp([0., 1.], Rotation.from_matrix(np.stack((r0, rotation))))
        f0 = self.aperture[arm]
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            smooth = alpha * alpha * (3. - 2. * alpha)
            if finger is not None:
                self.aperture[arm] = f0 + alpha * (finger - f0)
            self.drive(arm, p0 + smooth * (np.asarray(position) - p0), turn(smooth).as_matrix())
            yield self.emit()

    def trim(self, arm, repeats=3, limit=.025):
        """Shift the command by the measured error of the hand (the arms sag a little under their own weight)."""
        for _ in range(repeats):
            want = self.cmd[arm][0]
            self.offset[arm] = np.clip(self.offset[arm] + np.clip(want - self.actual_tcp(arm)[0], -.006, .006), -limit, limit)
            yield from self.move(arm, *self.cmd[arm], .35)

    def close(self, arm, finger, seconds=.8):
        yield from self.move(arm, *self.cmd[arm], seconds, finger=finger)
        yield from self.hold(.4)

    # -- a held object ----------------------------------------------------------------------
    def take(self, arm, name):
        """Remember where the commanded hand pose is on the object it has just closed on, so that later commands
        can be given as poses of the object. The sag correction found while reaching is kept."""
        origin, rotation = self.pose(name)
        point, hand = self.cmd[arm]
        self.grip[arm] = (rotation.T @ hand, rotation.T @ (point - origin))

    def hand_for(self, arm, origin, rotation):
        relative_rotation, relative_point = self.grip[arm]
        return np.asarray(origin) + rotation @ relative_point, rotation @ relative_rotation

    def carry(self, arm, name, origin, rotation, seconds, reference=None, servo=False):
        """Move the held object to a pose. With ``servo``, the command is shifted until the object's reference
        point (object frame) is where it should be, as read from the simulator."""
        p0, r0 = self.held[arm]
        turn = Slerp([0., 1.], Rotation.from_matrix(np.stack((r0, rotation))))
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            # Servo only near the end: while the target is still moving, the arm's lag is not an error to correct.
            self.set_held(arm, name, p0 + alpha * (np.asarray(origin) - p0), turn(alpha).as_matrix(), reference,
                          servo and step > .8 * steps)
            yield self.emit()
        if servo:
            for _ in range(round(.6 * CONTROL_HZ)):                 # hold still and let the correction settle
                self.set_held(arm, name, origin, rotation, reference, True)
                yield self.emit()

    def set_held(self, arm, name, origin, rotation, reference=None, servo=False):
        self.held[arm] = (np.asarray(origin, dtype=float), np.asarray(rotation, dtype=float))
        if servo and reference is not None and self.state is not None:
            actual_origin, actual_rotation = self.pose(name)
            error = (origin + rotation @ reference) - (actual_origin + actual_rotation @ reference)
            self.offset[arm] = np.clip(self.offset[arm] + np.clip(.2 * error, -.002, .002), -.035, .035)
        return self.drive(arm, *self.hand_for(arm, origin, rotation))


    # -- the right arm and the bottle ---------------------------------------------------------
    def take_bottle(self):
        self.phase("bottle_reach")
        origin, rotation = self.pose("bottle")
        if rotation[2, 2] < np.cos(np.radians(15.)) or origin[2] > .01:
            self.bottle_ok = False                              # not standing on the table: do not grab at it
            self.log["events"].append({"tick": self.tick, "bottle_not_standing": [round(1000 * float(v), 1) for v in origin]})
            return
        grip = origin + (0., 0., BOTTLE_GRIP_Z)
        # Come straight down onto the bottle, fingers open either side of it: the arm cannot back off far along
        # this grasp's approach direction this close to its base.
        yield from self.glide("right", grip + (0., 0., .055), BOTTLE_GRIP, 2.5)
        yield from self.hold(.3)
        yield from self.trim("right")
        self.phase("bottle_approach")
        yield from self.move("right", grip, BOTTLE_GRIP, 1.5)
        origin, _ = self.pose("bottle")                       # centre on the bottle as it stands now
        self.cmd["right"] = (origin + (0., 0., BOTTLE_GRIP_Z), BOTTLE_GRIP)
        yield from self.trim("right", repeats=3)
        self.log["grasps"].setdefault("bottle_centre_error_mm", []).append([round(1000 * float(v), 1) for v in
                                                        self.actual_tcp("right")[0] - self.cmd["right"][0]])
        self.phase("bottle_close")
        target = -(L.BOTTLE_R - BOTTLE_SQUEEZE)
        yield from self.close("right", target)
        self.log["grasps"].setdefault("bottle_opening", []).append(round(self.opening("right"), 4))
        self.take("right", "bottle")
        self.held["right"] = self.pose("bottle")
        self.phase("bottle_lift")
        yield from self.carry("right", "bottle", HOLD_AT, np.eye(3), 2.0)
        origin, _ = self.pose("bottle")
        self.log["grasps"].setdefault("bottle_lifted_mm", []).append(round(1000 * float(origin[2]), 1))
        self.bottle_ok = origin[2] > .03                        # it came up with the hand

    def pour(self, want):
        """Tip the bottle over the cap until ``want`` more tablets have left it; return what the cap then holds."""
        self.phase("pour_approach")
        cap, _ = self.pose("cap")                             # held level in the left hand
        over = cap[:2] - LIP_LEAD * POUR_DIR[:2]
        cap_base = float(cap[2])
        base = L.N_TABLETS - self.counts()["bottle"]
        row = {"tick": self.tick, "want": want, "cap_before": self.counts()["cap"]}

        def bottle_at(theta, lip_z):
            rotation = tilt_rotation(theta)
            return np.array((*over, lip_z)) - rotation @ LIP_LOCAL, rotation

        yield from self.carry("right", "bottle", *bottle_at(np.radians(30.), max(.120, cap_base + .110)), 2.0)
        yield from self.carry("right", "bottle", *bottle_at(TILT_START, lip_height(TILT_START, cap_base) + .010), 1.5,
                              LIP_LOCAL, True)
        self.phase("pour")
        # The tablets lie on the shoulder until the bottle is tipped to about 120 degrees; there the pile can let go
        # all at once (the second Isaac run lost ten tablets in a quarter of a second at 125 degrees, six of them
        # onto the table). So: settle the pile at 105 degrees, then tip very slowly, and the moment a tablet enters
        # the neck, tip back 8 degrees. The neck alone still lets a tablet through (it slopes down 25 degrees
        # and more there, steeper than the 19 degrees friction allows) but the pile on the shoulder stops.
        theta, waited, left, backoff_from, settled = TILT_START, 0., 0, None, 0.
        trace = []
        while theta < TILT_MAX:
            now = self.counts()
            left = L.N_TABLETS - now["bottle"] - base
            if self.tick % 2 == 0:
                trace.append([self.tick, round(float(np.degrees(theta)), 1), now["neck"], now["at_lip"], left, now["cap"]])
            if left + now["at_lip"] >= want:
                break                                         # enough have left or are about to
            if now["neck"]:
                backoff_from = theta if backoff_from is None else backoff_from
                theta = max(backoff_from - BACKOFF, theta - BACKOFF_RATE * DT)
                waited += DT
                if waited > NECK_WAIT_S:                      # one is lodged in the neck: tip a little further
                    backoff_from += FINE_TILT * DT
            else:
                backoff_from, waited = None, 0.
                if theta >= SETTLE_AT and settled < SETTLE_S:
                    settled += DT                             # let the pile come to rest against the shoulder
                else:
                    theta += (SLOW_TILT if theta < SETTLE_AT else FINE_TILT) * DT
            self.set_held("right", "bottle", *bottle_at(theta, lip_height(theta, cap_base)), LIP_LOCAL, True)
            yield self.emit()
        row["trace"] = trace
        row.update(stopped_at_deg=round(float(np.degrees(theta)), 1), left_at_stop=left)
        self.phase("pour_back")
        while theta > TILT_REST:                              # back: moderately at first, so the lip stays over the cap
            theta = max(TILT_REST, theta - (BACK_TILT if theta < np.radians(95.) else BACK_TILT_FIRST) * DT)
            self.set_held("right", "bottle", *bottle_at(theta, lip_height(theta, cap_base) + .010))
            yield self.emit()
        yield from self.carry("right", "bottle", *bottle_at(np.radians(30.), max(.120, cap_base + .110)), 1.0)
        yield from self.carry("right", "bottle", HOLD_AT, np.eye(3), 1.5)
        yield from self.hold(.8)
        now = self.counts()
        row.update(cap_after=now["cap"], bottle_after=now["bottle"], strays=L.N_TABLETS - sum(now[k] for k in ("bottle", "cap", "cup")))
        self.log["pours"].append(row)
        return now["cap"]

    def put_bottle_down(self, xy):
        self.phase("bottle_down")
        yield from self.carry("right", "bottle", (xy[0], xy[1], .030), np.eye(3), 1.5)
        yield from self.carry("right", "bottle", (xy[0], xy[1], .002), np.eye(3), 1.0)
        yield from self.hold(.3)
        yield from self.release("right")

    def release(self, arm):
        point, rotation = self.cmd[arm]
        yield from self.close(arm, OPEN, .6)
        yield from self.move(arm, point + (0., 0., .06), rotation, 1.2)          # straight up, clear of what it held
        yield from self.glide_joints(arm, np.asarray(READY, dtype=float), 2.0)
        self.cmd[arm] = ik.tcp_pose(np.asarray(READY, dtype=float), BASES[arm])
        self.offset[arm] = np.zeros(3)

    # -- the left arm and the cap --------------------------------------------------------------
    def take_cap(self):
        self.phase("cap_reach")
        origin, _ = self.pose("cap")
        grip = origin + (0., 0., CAP_GRIP_Z)
        # Straight down onto the cap, fingers opened only 12 mm wider than it each side (clear of the cup nearby).
        self.aperture["left"] = -(L.CAP_R + .012)
        yield from self.glide("left", grip + (0., 0., .050), CAP_GRIP, 2.5)
        yield from self.hold(.3)
        yield from self.trim("left")
        yield from self.move("left", grip, CAP_GRIP, 1.5)
        origin, _ = self.pose("cap")
        self.cmd["left"] = (origin + (0., 0., CAP_GRIP_Z), CAP_GRIP)
        yield from self.trim("left", repeats=3)
        self.log["grasps"].setdefault("cap_centre_error_mm", []).append([round(1000 * float(v), 1) for v in
                                                     self.actual_tcp("left")[0] - self.cmd["left"][0]])
        self.phase("cap_close")
        target = -(L.CAP_R - CAP_SQUEEZE)
        yield from self.close("left", target)
        self.log["grasps"].setdefault("cap_opening", []).append(round(self.opening("left"), 4))
        self.take("left", "cap")
        self.held["left"] = self.pose("cap")
        self.level = self.held["left"][1]                       # the cap's attitude as it lay on the table
        yield from self.carry("left", "cap", self.held["left"][0] + (0., 0., .06), self.level, 1.2)

    def open_bottle(self):
        """Depress and turn the cap with the left gripper while the right holds the bottle."""
        self.phase("opening_position")
        station = np.asarray(OPENING_STATION)
        yield from self.carry("right", "bottle", station, np.eye(3), 1.8, np.zeros(3), True)
        origin, r = self.pose("cap")
        grasp = origin + r @ (0., 0., .007)
        reach = OPENING_GRIP
        self.aperture["left"] = -(L.CAP_R + .010)
        yield from self.glide("left", grasp + (0., 0., .050), reach, 2.3)
        yield from self.trim("left")
        self.phase("opening_grasp")
        yield from self.move("left", grasp, reach, 1.2)
        yield from self.trim("left")
        yield from self.close("left", -(L.CAP_R - .0025))
        self.take("left", "cap")
        self.held["left"] = self.pose("cap")
        origin, cap_r = self.held["left"]
        bottle, bottle_r = self.pose("bottle")
        # Compress the passive collar and keep the upper lug below the detent
        # during the first part of the turn. Feedback is on the physical poses.
        self.phase("opening_press")
        target = bottle + bottle_r @ (0., 0., L.CLOSED_Z - .0035)
        yield from self.carry("left", "cap", target, cap_r, 1., np.zeros(3), True)
        self.phase("opening_turn")
        for angle in np.linspace(0., OPENING_TURN_DEG, 81):
            bottle, _ = self.pose("bottle")
            pressed = bottle + bottle_r @ (0., 0., L.CLOSED_Z - .0035)
            twist = bottle_r @ Rotation.from_euler("z", angle, degrees=True).as_matrix() @ bottle_r.T
            self.set_held("left", "cap", pressed, twist @ cap_r, np.zeros(3), True)
            self.set_held("right", "bottle", station, bottle_r, np.zeros(3), True)
            yield self.emit()
        cap_r = twist @ cap_r
        self.phase("opening_lift")
        bottle, _ = self.pose("bottle")
        yield from self.carry("left", "cap", bottle + bottle_r @ (0., 0., L.CLOSED_Z + .042),
                              cap_r, 1.3, np.zeros(3), True)
        if self.state["opening_stage"] != "opened":
            self.log["events"].append({"tick": self.tick, "opening_failed": self.state["opening_stage"]})
            return False
        # Move the bottle well clear before turning the removed cap over.
        yield from self.carry("right", "bottle", HOLD_AT, np.eye(3), 1.8)
        self.phase("turn_cap_over")
        # Tilt the empty cap over its landing point. Once the open side is up,
        # lower the rim to the table and release; gravity settles the cap on its
        # flat top face. Regrasp in the normal dispensing orientation.
        yield from self.carry("left", "cap", (*CAP_DOWN_XY, .145), cap_r, 1.4)
        axis = TIP_AXIS_CUP
        for degrees in np.linspace(0., TURNOVER_DEG, 81):
            turn = cap_rotation(np.radians(degrees), axis) @ cap_r
            self.set_held("left", "cap", (*CAP_DOWN_XY, .145), turn)
            yield self.emit()
        # Keep fingers clear of the table; the empty cap drops a short distance.
        yield from self.carry("left", "cap", (*CAP_DOWN_XY, TURNOVER_RELEASE_Z), turn, 1.7)
        yield from self.release("left")
        yield from self.hold(.6)
        if self.pose("cap")[1][2, 2] < .94:
            self.log["events"].append({"tick": self.tick, "empty_cap_failed_to_settle_open_side_up": True})
            return False
        return True

    def tip_cap(self, opening, into, keep, drop, axis):
        """Hold the cap's low inner rim over ``opening`` (centre of the bottle's mouth or the cup's top) and tip the cap
        until it holds ``keep`` tablets or fewer, closed loop on the counted tablets; then tip it back level."""
        self.phase(f"cap_to_{into}")
        low = np.cross(axis, (0., 0., 1.))
        rim_point = np.asarray(opening) - RIM_BACK * low + (0., 0., drop)
        rim_local = self.level.T @ (L.CAP_R_IN * low) + (0., 0., L.CAP_H)

        def cap_at(tau, lift=0.):
            rotation = cap_rotation(tau, axis) @ self.level
            return rim_point + (0., 0., lift) - rotation @ rim_local, rotation

        high = cap_at(0., .05)
        here = self.held["left"][0]
        if here[2] < high[0][2]:                                # rise first, then move across, then come down
            yield from self.carry("left", "cap", (here[0], here[1], high[0][2]), high[1], 1.0)
        yield from self.carry("left", "cap", *high, 2.0)
        yield from self.carry("left", "cap", *cap_at(0.), 1.2, rim_local, True)
        yield from self.hold(.3)
        self.phase(f"tip_into_{into}")
        start = self.counts()
        tau, settle, stuck, first_out = 0., 0., 0., None
        row = {"tick": self.tick, "into": into, "keep": keep, "cap_before": start["cap"], "trace": []}
        while True:
            now = self.counts()
            if first_out is None and now["cap"] < row["cap_before"]:
                first_out = round(float(np.degrees(tau)), 1)
            if self.tick % 5 == 0:                                 # commanded tilt, measured tilt, tablets in the cap
                tilt = float(np.degrees(np.arccos(np.clip(self.pose("cap")[1][2, 2], -1., 1.))))
                row["trace"].append([self.tick, round(float(np.degrees(tau)), 1), round(tilt, 1), now["cap"]])
            if now["cap"] <= keep:
                break
            if now["cap"] < start["cap"] or settle > 0.:          # one has gone: let the rest settle before going on
                settle = settle + DT if settle < CAP_SETTLE_S else 0.
                start = now if settle == 0. else start
            elif tau < CAP_TIP_MAX:
                rate = CAP_FAST if tau < CAP_FAST_TO else CAP_SLOW
                step = min(CAP_TIP_MAX, tau + rate * DT)
                cmd = cap_at(step)
                if self.solve("left", *self.hand_for("left", *cmd))[1] < .002:
                    tau = step                                      # never tip further than the arm can follow
                else:
                    stuck += DT
            else:
                stuck += DT
            if stuck > CAP_STUCK_S:
                break
            self.set_held("left", "cap", *cap_at(tau), rim_local, True)
            yield self.emit()
        row.update(stopped_at_deg=round(float(np.degrees(tau)), 1), first_out_deg=first_out, gave_up=stuck > CAP_STUCK_S)
        while tau > 0.:
            tau = max(0., tau - CAP_BACK * DT)
            self.set_held("left", "cap", *cap_at(tau, .01 * (1. - tau / max(CAP_TIP_MAX, 1e-6))))
            yield self.emit()
        yield from self.carry("left", "cap", *cap_at(0., .03), 1.0)
        yield from self.hold(.5)
        row.update(self.counts())
        self.log["events"].append(row)

    def hold_cap(self):
        """Hold the cap level, open side up, where the bottle will be tipped over it."""
        self.phase("cap_hold")
        here = self.held["left"][0]
        top = max(here[2], CAP_HOLD[2]) + .02
        yield from self.carry("left", "cap", (here[0], here[1], top), self.level, 1.0)
        yield from self.carry("left", "cap", (CAP_HOLD[0], CAP_HOLD[1], top), self.level, 1.5)
        yield from self.carry("left", "cap", CAP_HOLD, self.level, 1.0, np.array((0., 0., 0.)), True)

    def park_cap(self):
        """Hold the cap up and out of the way (still open side up) while the right arm stands the bottle down."""
        self.phase("cap_park")
        here = self.held["left"][0]
        yield from self.carry("left", "cap", (here[0], here[1], CAP_PARK[2]), self.level, 1.0)
        yield from self.carry("left", "cap", CAP_PARK, self.level, 1.5)

    def put_cap_down(self, xy):
        self.phase("cap_down")
        here = self.held["left"][0]
        yield from self.carry("left", "cap", (here[0], here[1], max(here[2], .120)), self.level, 1.0)
        yield from self.carry("left", "cap", (xy[0], xy[1], max(here[2], .120)), self.level, 1.5)
        yield from self.carry("left", "cap", (xy[0], xy[1], .050), self.level, 1.0)
        yield from self.carry("left", "cap", (xy[0], xy[1], .003), self.level, 1.0)
        yield from self.hold(.3)
        yield from self.release("left")

    # -- the demonstration ------------------------------------------------------------------
    def sequence(self):
        while self.state is None:
            yield self.emit()
        # Both hands work together for the pour: the left holds the cap level under the bottle's lip while the
        # right tips the bottle. The cap stays in the left hand until it has been emptied into the cup.
        yield from self.take_bottle()
        if not self.bottle_ok or not (yield from self.open_bottle()):
            self.phase("opening_failed")
            while True:
                yield self.emit()
        yield from self.take_cap()
        yield from self.hold_cap()
        bottle_in_hand = self.bottle_ok
        for attempt in range(MAX_POURS):
            now = self.counts()
            missed = L.N_TABLETS - now["bottle"] - now["cap"] - now["cup"]
            if now["cap"] == L.DOSE or missed >= 2 or not self.bottle_ok:
                break                                           # the dose is in the cap, or things went wrong: stop
            if now["cap"] > L.DOSE:                             # too many: tip the extra back into the bottle
                yield from self.park_cap()
                yield from self.put_bottle_down(RETURN_XY)      # the bottle stands alone while the cap is tipped over it
                bottle_in_hand = False
                bottle, rotation = self.pose("bottle")
                yield from self.tip_cap(bottle + rotation @ (0., 0., L.BOTTLE_H), "bottle", L.DOSE, DROP_BOTTLE, TIP_AXIS_BOTTLE)
                if self.counts()["cap"] == L.DOSE:
                    break
                yield from self.park_cap()                      # too few left: take the bottle again (cap out of the way),
                yield from self.take_bottle()                   # then hold the cap up again and pour again
                bottle_in_hand = self.bottle_ok
                yield from self.hold_cap()
                continue
            yield from self.pour(L.DOSE - now["cap"])
        if bottle_in_hand:
            yield from self.put_bottle_down(BOTTLE_XY)
        if self.counts()["cap"] > 0:                            # one hand: tip the cap's tablets into the cup
            cup, rotation = self.pose("cup")
            yield from self.tip_cap(cup + rotation @ (0., 0., L.CUP_H), "cup", 0, DROP_CUP, TIP_AXIS_CUP)
        yield from self.put_cap_down(CAP_DOWN_XY)
        self.phase("done")
        while True:
            yield self.emit()
