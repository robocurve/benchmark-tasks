"""Full-score demonstration: stand the collapsed bag up around the box of crackers, load the pack with both arms,
then lift the bag by its handles.

The demonstration reads the robot's own joint and gripper readings and, as the review rules allow for a scripted
demonstration, simulator state: where the bag's base board, walls and handles and the two items are, and how hard
the arms press on the pack. It changes nothing in the simulation: bag and items move only where fingers push or
hold them.

1. ``carton``: the right hand reaches across, pinches the box of crackers by its two large faces near the top,
   lifts it and holds it over the near end of the collapsed bag.
2. ``open``: the left hand pinches the left handle's grip where it lies on the table and raises it. The left wall
   comes up with it and the corner seams pull the other three walls up.
3. ``stand``: the right hand lowers the box onto the base board across the near end and lets go; the left hand lets
   go of the handle. Both large walls now lean on the box and the bag stands.
4. ``pack``: both hands close and come down either side of the pack, fingers pointing away from the robot and
   down. Each presses the side edge of its fingertips on one of the pack's two large faces, 40 mm below the top and
   over the middle of the pack's length, so that the pack hangs level between them. They lift it, carry it over the
   bag and lower it until the fingertips are just above the rim.
5. ``let down``: the hands ease apart until the pack slides between the fingers and comes to rest on the base, then
   draw away sideways.
6. ``lift``: each hand passes its open fingers either side of a handle's grip where it hangs beside the wall, swings
   the handle up over the rim, re-centres the pads on where the grip came to rest, closes on it (once more if a
   pad misses) and lifts. The bag is held up 2 s, set down and let go.

``variant`` selects shortened runs and the controls that must not reach full score; see ``VARIANTS``.
"""

from inspect_robots import Action, ActionChunk
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, READY

from . import bag_model as m
from .fixed_scene import ARMS, BASES, CONTROL_HZ

LEFT, RIGHT = ARMS
SIDE = {LEFT: 1., RIGHT: -1.}           # which side of the midline an arm stands on
WALL = {LEFT: "left", RIGHT: "right"}   # the large wall, and handle, on an arm's side
OPEN = float(FINGER_OPEN)               # finger joint value with the pads 94 mm apart (negative)
TIP = float(PAD_TIP_PAST_TCP)           # the fingertips end this far beyond the gripper's centre point
DT = 1. / CONTROL_HZ
VARIANTS = ("full",                     # everything: 1.0
            "carton",                   # stop once the bag stands around the box: 0.25
            "no_lift",                  # stop once both items are in: 0.75
            "large_first",              # control: load the pack first, into the collapsed bag
            "no_opening",               # control: put the box in without raising the bag first
            "drop_from_rim")            # control: let the pack go at once instead of easing it down

# -- the box of crackers (right hand) ------------------------------------------------------------------------
CARTON_PITCH = np.radians(15.)          # the right hand's fingers point this much below level, so its forearm stays high
CARTON_BITE = .070                      # the pads reach this far along the box's large faces from its near edge
CARTON_DOWN = .020                      # the gripper's middle is this far below the box's top
CARTON_GAP = .086                       # pads apart on the way in
CARTON_PINCH = .044                     # commanded pads apart on the 60 mm box: about 40 N a finger
OVER_RIM = .020                         # the box is carried with its underside this far above the bag's rim
PACK_OVER_RIM = .030                    # ... and the pack, which hangs free between two fingertips, this far
CARTON_SET = .006                       # the box is let go with its underside this far above the base board
ROOT_HALF = .034                        # a finger is this wide either side of its middle where it leaves the gripper
RAIL_HALF = .051                        # the gripper's cross rail reaches this far either side along the closing line

# -- a handle on the table (left hand) -----------------------------------------------------------------------
HANDLE_PITCH = np.radians(50.)          # fingers point this steeply down on the grip lying on the table
HANDLE_ALONG = .020                     # taken this far beyond the grip's middle, away from the robot
HANDLE_GAP = .040                       # pads apart coming down over the 18 mm grip
HANDLE_PINCH = .008                     # commanded pads apart on it: about 25 N a finger
RAISE_TO = m.RIM - .0008                # the grip is raised until the wall's rim is this high: the wall pulled straight

# -- the pack (both hands) -----------------------------------------------------------------------------------
SIDE_YAW = np.radians(28.)              # closed fingers point this much toward the pack: more than their own taper
SIDE_PITCH = np.radians(35.)            # (20 degrees), so that the side edge of the fingertips is what touches; and
CORNER = np.array((0., -.0077, .013))   # this much below level, so that wrist and forearm stay above the bag. That
PACK_DOWN = .040                        # edge, from the gripper's centre point; it presses this far below the pack's top
TIP_LOW = .009                          # the fingertips' lowest corner is this far below that edge's middle
PACK_STAND_OFF = .012                   # fingers this far off the pack's faces coming down beside it
SQUEEZE_N = 6.                          # each arm presses the pack with at least this: three times what holds its
                                        # 3.5 N by friction. Twice as hard (tried) bends the wrists 13 degrees and
                                        # the pack ends 18 degrees off level over the bag
SQUEEZE_STEP = .0020                    # per control step while pressing in
LEVEL_DEG = 4.                          # the pack is set down and taken again if it hangs more than this off level
AHEAD = .012                            # first try: press this far beyond the middle of the pack's length
KEEP_N = 5.                             # carried, the pack is pressed with at least this: an arm reaching across
KEEP_STEP = .0003                       # presses less than it did where it took the pack, and is moved in to make up
KEEP_MAX = .025
HOLD_TWIST = False                      # tried and switched off (runs h3-h6): holding the hands' orientation keeps
LEVEL_GAIN = 0.                         # the pack from pitching but tips it sideways; levelling it by moving one
                                        # hand against the other wrings it out of the grasp. Kept for reference.
TWIST_RATE = np.radians(.5)             # pressing on the pack bends the springy wrists ten degrees and more, and the
TWIST_MAX = np.radians(25.)             # fingers then lie along its face instead of touching it with their tips. The
                                        # hands are turned back by what the joint readings say they are off
LEVEL_MAX = np.array((.040, 0., .050))   # ... and, carried, one hand is moved against the other (forward, up) by
                                        # how far the pack is turned or tipped between them: under the same push
                                        # an arm gives 4 cm one way reaching across and another way close in
GIVE = np.array((.476, .825, .305))     # which way a fingertip is pushed off its commanded place by the pack's push on
                                        # it (measured in the simulator: forward, outward and up; the wrists are springy)
SQUEEZE_MAX = .095
RIM_CLEAR = .006                        # the grippers stop with their fingertips this far above the rim
EASE_STEP = .0006                       # the hands part by this per step while the pack is eased down
EASE_MAX = .030

# -- a hanging handle (either hand) --------------------------------------------------------------------------
HOOK_YAW, HOOK_PITCH = np.radians(50.), np.radians(12.)
HOOK_GAP = .030                         # pads apart, one above and one below the grip
HOOK_OUT = .075                         # start this far back from the grip along the fingers, and slide in lengthwise
HOOK_PAST = .028                        # ... right through the handle: the fingertips' ends this far past the grip's
                                        # middle, so that the grip rides on the middle of the lower finger
HOOK_ROOM = .008                        # while a handle swings the pads stay this much further apart than the grip is
                                        # thick between them: it rides on the lower finger without being held
SWING_S = 8.                            # the swing up takes this long: slow enough for the arms to keep to their path
HOOK_TOP = np.radians(160.)             # the handles are swung up to this, short of straight over the rim
HOOK_LEVEL = np.radians(60.)            # from here up the fingers come level: pads closed on a grip at a slant would
                                        # hold the whole bag at that slant
HOOK_ALONG = -.022                      # taken this far from the grip's middle, toward the box (the bag's heavy end)
HOOK_PINCH = .006                       # commanded pads apart on the 18 mm grip: about 30 N a finger
LIFT_BASE = .060                        # the base board's lowest corner is raised to this
LIFT_MAX = .150
HOLD_S = 2.4

SEEDS = (np.array(READY), np.array((0., 2.0, 1.8, -1.2, 0., 0.)), np.array((0., 1.6, .8, .8, 0., 0.)),
         np.array((.4, 2.2, 2.0, -1.2, 0., 1.0)), np.array((-.4, 2.2, 2.0, -1.2, 0., -1.0)),
         np.array((0., 1.9, .5, 1.3, .5, -1.5)), np.array((0., 1.9, .5, 1.3, -.5, 1.5)),
         np.array((-.8, 1.2, .6, .3, 1.0, 0.)), np.array((.8, 1.2, .6, .3, -1.0, 0.)),
         np.array((1.0, 1.8, 1.2, .5, -.6, 0.)), np.array((-1.0, 1.8, 1.2, .5, .6, 0.)))


def frame(approach, closing):
    """Gripper orientation: fingers point along ``approach``; they open and close along ``closing``."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float)
    c = c - a * (c @ a)
    c /= np.linalg.norm(c)
    return np.column_stack((c, np.cross(a, c), a))


def aim(yaw_in, pitch, side):
    """Unit vector pointing away from the robot, turned ``yaw_in`` toward the midline for the arm on ``side`` and
    ``pitch`` below level."""
    return np.array((np.cos(yaw_in) * np.cos(pitch), -side * np.sin(yaw_in) * np.cos(pitch), -np.sin(pitch)))


def fingertips(joint_pos, arm):
    """Middle of the two fingertips' ends, from joint readings (tabletop frame)."""
    start = 7 * ARMS.index(arm)
    point, rotation = ik.tcp_pose(np.asarray(joint_pos, dtype=float)[start:start + 6], BASES[arm])
    return point + TIP * rotation[:, 2]


def mm(values):
    return [round(1000 * float(v), 1) for v in np.ravel(values)]


class BagReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self, variant="full"):
        if variant not in VARIANTS:
            raise ValueError(variant)
        self.variant = variant
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float) for arm in ARMS}
        self.gap = {arm: 2 * abs(OPEN) for arm in ARMS}            # commanded distance between the pads
        self.tip = {}                                               # commanded fingertip points (Cartesian moves)
        self.rot = {}
        self.worst_ik = 0.
        self.squeeze = {arm: 0. for arm in ARMS}
        self.twist, self.flat = {arm: np.zeros(3) for arm in ARMS}, {}
        self.level = np.zeros(3)
        self.tick = 0
        self.phases = []
        self.log = {"variant": variant, "events": []}

    # -- readings -------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        if state is not None:
            self.state = state

    def measured_tip(self, arm):
        return fingertips(self.measured, arm)

    def opening(self, arm):
        """Measured distance between the pads, m."""
        return float(self.measured[7 * ARMS.index(arm) + 6]) * 2 * abs(OPEN)

    def item(self, name):
        pos, rot = self.state["items"][name]
        return np.asarray(pos, dtype=float), np.asarray(rot, dtype=float)

    def in_bag(self, local):
        """Tabletop point of a point given in the base board's frame."""
        pos, rot = self.state["base"]
        return np.asarray(pos, dtype=float) + np.asarray(rot, dtype=float) @ np.asarray(local, dtype=float)

    def base_top(self):
        return float(self.state["base"][0][2]) + m.BASE_T

    def base_low(self):
        """Height of the base board's lowest corner above the table."""
        pos, rot = (np.asarray(v, dtype=float) for v in self.state["base"])
        return float(pos[2] - abs(rot[2, 0]) * m.BASE_X / 2 - abs(rot[2, 1]) * m.BASE_Y / 2)

    def press(self, arm):
        """How hard an arm presses on the pack now, N."""
        return float(self.state.get("forces", {}).get(f"{arm}_on_pack", 0.))

    # -- motion ---------------------------------------------------------------------------
    def solve(self, arm, point, rotation, seed=None, iters=40):
        target = np.asarray(point, dtype=float) - TIP * rotation[:, 2]
        joints, error, _ = ik.solve(target, ik.matrix_to_quat_xyzw(rotation),
                                    self.joints[arm] if seed is None else seed, BASES[arm], iters=iters)
        return joints, error

    def best(self, arm, point, rotation):
        """Joint angles for a pose, searched from several seeds and with the gripper either way round (a parallel
        gripper turned half a turn about its fingers is the same grasp). Returns (joints, rotation used)."""
        found = None
        for flip in (1., -1.):
            rot = rotation * np.array((flip, flip, 1.))
            for seed in (self.joints[arm], *SEEDS):
                joints, error = self.solve(arm, point, rot, seed=seed, iters=150)
                if error < 1e-3:
                    rot_err = np.linalg.norm(ik.tcp_pose(joints, BASES[arm])[1] - rot)
                    margin = float(np.minimum(joints - ik.LOWER, ik.UPPER - joints).min())
                    cost = 1000. * error + .002 * float(np.abs(joints - self.joints[arm]).sum()) + 50. * rot_err
                    cost += .2 * max(0., .25 - margin)                     # keep off the joint stops
                    if rot_err < .05 and (found is None or cost < found[0]):
                        found = (cost, joints, rot, error)
        if found is None:
            raise RuntimeError(f"{arm} arm cannot reach {np.round(point, 3)}")
        self.worst_ik = max(self.worst_ik, found[3])
        return found[1], found[2]

    def emit(self):
        for arm, point in self.tip.items():
            self.joints[arm], error = self.solve(arm, point, self.rot[arm])
            self.worst_ik = max(self.worst_ik, error)
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.gap[a] / (2 * abs(OPEN)), 0., 1.)] for a in ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def glide(self, seconds, targets, gaps=None):
        """Move in joint space to fingertip poses {arm: (point, rotation)} (smooth start and stop)."""
        gaps = gaps or {}
        goal, rots = {}, {}
        for arm, (point, rotation) in targets.items():
            goal[arm], rots[arm] = self.best(arm, point, rotation)
        start = {arm: self.joints[arm].copy() for arm in targets}
        jaws = {arm: self.gap[arm] for arm in gaps}
        self.tip = {arm: point for arm, point in self.tip.items() if arm not in targets}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm in targets:
                self.joints[arm] = start[arm] + alpha * (goal[arm] - start[arm])
            for arm, value in gaps.items():
                self.gap[arm] = jaws[arm] + alpha * (value - jaws[arm])
            yield self.emit()
        for arm, (point, _) in targets.items():
            self.tip[arm] = np.asarray(point, dtype=float).copy()
            self.rot[arm] = rots[arm]

    def line(self, seconds, targets, rotations=None, gaps=None):
        """Move fingertips in straight lines (smooth start and stop), turning to ``rotations`` on the way."""
        rotations = {arm: self.same_way(arm, rot) for arm, rot in (rotations or {}).items()}
        gaps = gaps or {}
        start = {arm: self.tip[arm].copy() for arm in targets}
        start_rot = {arm: self.rot[arm] for arm in rotations}
        jaws = {arm: self.gap[arm] for arm in gaps}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm, goal in targets.items():
                self.tip[arm] = start[arm] + alpha * (np.asarray(goal, dtype=float) - start[arm])
            for arm, goal in rotations.items():
                self.rot[arm] = ik.slerp_matrix(start_rot[arm], goal, alpha)
            for arm, value in gaps.items():
                self.gap[arm] = jaws[arm] + alpha * (value - jaws[arm])
            yield self.emit()

    def same_way(self, arm, rotation):
        """``rotation`` or its half-turn about the fingers, whichever is nearer the hand's present orientation."""
        flipped = rotation * np.array((-1., -1., 1.))
        now = self.rot[arm]
        return rotation if np.trace(now.T @ rotation) >= np.trace(now.T @ flipped) else flipped

    def grip(self, seconds, gaps):
        yield from self.line(seconds, {}, gaps=gaps)

    def shift(self, seconds, moves):
        """Move fingertips by the given offsets."""
        yield from self.line(seconds, {arm: self.tip[arm] + np.asarray(by, dtype=float) for arm, by in moves.items()})

    def home(self, arms, seconds):
        """Open the fingers and swing the arms back to their ready pose in joint space."""
        for arm in arms:
            self.tip.pop(arm, None)
        start = {arm: self.joints[arm].copy() for arm in arms}
        gap = {arm: self.gap[arm] for arm in arms}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm in arms:
                self.joints[arm] = start[arm] + alpha * (np.asarray(READY) - start[arm])
                self.gap[arm] = gap[arm] + min(1., 3. * alpha) * (2 * abs(OPEN) - gap[arm])
            yield self.emit()

    def wait(self, seconds):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield self.emit()

    def event(self, name, **extra):
        state = self.state
        row = {"tick": self.tick, "event": name,
               "rims_mm": [round(1000 * state["rims"][w]) for w in m.WALLS],
               "base_mm": mm(state["base"][0]), "carton_mm": mm(self.item("carton")[0]), "pack_mm": mm(self.item("pack")[0]),
               "grips_mm": {w: mm(state["grips"][w]) for w in m.HANDLES},
               "tips_command_mm": {arm: mm(self.tip[arm]) for arm in self.tip},
               "tips_measured_mm": {arm: mm(self.measured_tip(arm)) for arm in ARMS},
               "pads_apart_mm": [round(1000 * self.opening(arm), 1) for arm in ARMS]}
        row.update(extra)
        self.log["events"].append(row)

    def phase(self, name):
        self.phases.append((self.tick, name))

    # -- the box of crackers ----------------------------------------------------------------
    CARTON_AIM = np.array((0., np.cos(CARTON_PITCH), -np.sin(CARTON_PITCH)))     # right hand: fingers point to the
    CARTON_ROT = frame(CARTON_AIM, (1., 0., 0.))                                 # robot's left and down, close front to back

    def take_carton(self):
        self.phase("carton: reach across")
        pos, _ = self.item("carton")
        edge = pos[1] - m.CARTON[1] / 2
        height = pos[2] + m.CARTON[2] / 2 - CARTON_DOWN
        # the left hand goes to wait above its handle, clear of the box's way up
        grip = np.asarray(self.state["grips"]["left"], dtype=float)
        approach = aim(0., HANDLE_PITCH, 1.)
        park = (grip + (HANDLE_ALONG + .020, .135, .190), frame(approach, (0., 1., 0.)))
        point = np.array((pos[0], edge + CARTON_BITE, height))
        yield from self.glide(2.8, {RIGHT: (point - .115 * self.CARTON_AIM, self.CARTON_ROT), LEFT: park},
                              {RIGHT: CARTON_GAP, LEFT: HANDLE_GAP})
        yield from self.line(1.4, {RIGHT: point})
        self.phase("carton: pinch and lift")
        yield from self.grip(.5, {RIGHT: CARTON_PINCH})
        yield from self.wait(.2)
        held = self.opening(RIGHT) > CARTON_PINCH + .008
        self.event("carton pinched", held=held)
        self.carton_turn = self.carton_roll = 0.
        return held

    def carton_under(self):
        """Height of the held box's lowest corner above the table."""
        pos, rot = self.item("carton")
        half = np.asarray(m.CARTON) / 2
        return float(pos[2] - np.abs(rot[2]) @ half)

    def carton_foot(self):
        """Middle of the held box's underside (tabletop frame) and how far the box leans (rad): about the hand's
        closing line (x) and about the line along its fingers (y)."""
        pos, rot = self.item("carton")
        return pos - rot[:, 2] * m.CARTON[2] / 2, (float(np.arctan2(-rot[1, 2], rot[2, 2])), float(np.arctan2(rot[0, 2], rot[2, 2])))

    def carton_to(self, seconds, underside, xy=None, settle=.5):
        """Bring the held box's lowest corner to ``underside`` (and the middle of its underside over ``xy``),
        standing it upright in the hand and correcting for what the arm gives under its weight."""
        steps = max(1, round(seconds * CONTROL_HZ))
        start = self.tip[RIGHT].copy()
        foot, _ = self.carton_foot()
        move = np.array((0., 0., underside - self.carton_under()))
        if xy is not None:
            move[:2] = np.asarray(xy, dtype=float) - foot[:2]
        trim = np.zeros(3)
        for step in range(1, steps + round(settle * CONTROL_HZ) + 1):
            alpha = min(1., step / steps)
            alpha *= alpha * (3. - 2. * alpha)
            foot, (lean, roll) = self.carton_foot()
            turn = float(np.clip(-.2 * lean, -.006, .006))       # turn the hand about its closing line to stand the box up
            if abs(self.carton_turn + turn) < np.radians(15.):
                self.carton_turn += turn
                c, k = np.cos(turn), np.sin(turn)
                self.rot[RIGHT] = np.array(((1., 0., 0.), (0., c, -k), (0., k, c))) @ self.rot[RIGHT]
            turn = float(np.clip(-.2 * roll, -.004, .004))       # ... and about its fingers
            if abs(self.carton_roll + turn) < np.radians(8.):
                self.carton_roll += turn
                c, k = np.cos(turn), np.sin(turn)
                self.rot[RIGHT] = np.array(((c, 0., k), (0., 1., 0.), (-k, 0., c))) @ self.rot[RIGHT]
            if step > steps // 3:                                # then close on what is measured
                error = np.array((0., 0., underside - self.carton_under()))
                if xy is not None:
                    error[:2] = np.asarray(xy, dtype=float) - foot[:2]
                trim += np.clip(.2 * (error - (1. - alpha) * move), -.002, .002)
            self.tip[RIGHT] = start + alpha * move + trim
            yield self.emit()

    def carry_carton(self):
        yield from self.carton_to(1.6, m.RIM + OVER_RIM, settle=0.)
        self.phase("carton: over the bag")
        yield from self.carton_to(2.4, m.RIM + OVER_RIM, self.in_bag((m.CARTON_SLOT[0], m.CARTON_SLOT[1], 0.))[:2])
        self.event("carton over the bag", under_mm=round(1000 * self.carton_under(), 1))

    def set_carton(self):
        self.phase("carton: lower onto the base")
        # The left wall is held straight; the other three hang from their corner seams, a little slack, so their
        # creases stand a little into the bag. The box goes down midway between the two large walls' creases.
        place = self.in_bag((m.CARTON_SLOT[0], m.CARTON_SLOT[1], 0.))[:2]
        crease = {wall: self.state["lowers"][wall][0] + self.state["lowers"][wall][1] @ (0., 0., m.SEG / 2)
                  for wall in ("left", "right")}
        place[1] = float(crease["left"][1] + crease["right"][1]) / 2
        self.event("creases", left_mm=mm(crease["left"]), right_mm=mm(crease["right"]), carton_y_mm=round(1000 * place[1], 1),
                   room_mm=round(1000 * float(crease["left"][1] - crease["right"][1] - m.WALL_T), 1),
                   carton_lean_deg=[round(float(np.degrees(v)), 1) for v in self.carton_foot()[1]])
        yield from self.carton_to(1.2, m.RIM + OVER_RIM, place, settle=.5)
        yield from self.carton_to(3.4, self.base_top() + CARTON_SET, place, settle=.4)
        self.event("carton down", above_base_mm=round(1000 * (self.carton_under() - self.base_top()), 1))
        yield from self.grip(.4, {RIGHT: CARTON_GAP})
        yield from self.wait(.3)
        self.phase("carton: let go")
        yield from self.shift(1.2, {RIGHT: -.125 * self.rot[RIGHT][:, 2] + (0., 0., .020)})

    # -- a handle lying on the table ----------------------------------------------------------
    def raise_handle(self, arm=LEFT):
        wall = WALL[arm]
        self.phase("open: pinch the handle on the table")
        grip = np.asarray(self.state["grips"][wall], dtype=float)
        approach = aim(0., HANDLE_PITCH, SIDE[arm])
        rot = frame(approach, (0., 1., 0.))
        point = np.array((grip[0] + HANDLE_ALONG, grip[1], max(.002, grip[2] - .004)))
        yield from self.glide(2.4, {arm: (point - .070 * approach, rot)}, {arm: HANDLE_GAP})
        yield from self.line(.9, {arm: point})
        yield from self.grip(.5, {arm: HANDLE_PINCH})
        yield from self.wait(.2)
        held = self.opening(arm) > HANDLE_PINCH + .006
        self.event("handle pinched", held=held)
        self.phase("open: raise the wall")
        # Bring the wall's rim straight up over the wall's foot until the wall is pulled straight, steering by where
        # the rim is; never so far that the base board leaves the table.
        want = RAISE_TO
        for _ in range(round(7. * CONTROL_HZ)):
            rim = np.asarray(self.state["handles"][wall][0], dtype=float)
            foot = self.in_bag(m.hinge_point(wall))
            error = np.array((0., foot[1] - rim[1], want - rim[2]))
            if error[2] < .0006 and abs(error[1]) < .003:
                break
            if float(self.state["base"][0][2]) > .0010:                 # the bag is coming up with it: ask for less
                want -= .0004
            self.tip[arm] = self.tip[arm] + np.clip(.25 * error, (0., -.0015, -.0030), (0., .0015, .0030))
            yield self.emit()
        yield from self.wait(.4)
        self.event("wall raised", held=self.opening(arm) > HANDLE_PINCH + .006, rim_mm=mm(self.state["handles"][wall][0]),
                   base_up_mm=round(1000 * float(self.state["base"][0][2]), 1))

    def release_handle(self, arm=LEFT):
        self.phase("stand: let go of the handle")
        yield from self.grip(.4, {arm: HANDLE_GAP})
        approach = self.rot[arm][:, 2]
        yield from self.shift(.8, {arm: -.060 * approach + (0., SIDE[arm] * .030, .030)})
        yield from self.wait(.5)
        self.event("bag standing", upright=bool(min(self.state["rims"].values()) >= .8 * m.RIM))

    # -- the pack ---------------------------------------------------------------------------
    def side_rot(self, arm):
        """Gripper orientation for pressing on the pack's large face on the arm's side with the side edge of the
        closed fingertips: fingers pointing away from the robot, SIDE_PITCH below level, turned SIDE_YAW toward
        the pack; the two fingertips one above the other along the face."""
        toward = np.array((0., -SIDE[arm], 0.))
        along = np.array((np.cos(SIDE_PITCH), 0., -np.sin(SIDE_PITCH)))
        approach = np.cos(SIDE_YAW) * along + np.sin(SIDE_YAW) * toward
        across = np.sin(SIDE_YAW) * along - np.cos(SIDE_YAW) * toward
        return np.column_stack((np.cross(across, approach), across, approach))

    def beside(self, arm, off, height=None, ahead=0.):
        """Fingertip point that puts the pressing edge of the arm's fingertips ``off`` from the pack's face on its
        side, ``ahead`` of the middle of the pack's length and PACK_DOWN below its top (or at ``height``)."""
        pos, _ = self.item("pack")
        z = pos[2] + m.PACK[2] / 2 - PACK_DOWN if height is None else height
        touch = np.array((pos[0] + ahead, pos[1] + SIDE[arm] * (m.PACK[1] / 2 + off), z))
        rot = self.side_rot(arm)
        return touch - rot @ CORNER + TIP * rot[:, 2]

    @staticmethod
    def give(arm):
        return GIVE * (1., SIDE[arm], 1.)

    def touch(self, arm):
        """Where the pressing edge of the arm's fingertips is now, from the joint readings."""
        rot = self.rot[arm]
        return self.measured_tip(arm) + rot @ CORNER - TIP * rot[:, 2]

    def hand_off(self, arm):
        """Rotation (vector, tabletop frame) that would take the hand from where the joint readings put it to
        side_rot, the orientation it is meant to press the pack in."""
        start = 7 * ARMS.index(arm)
        measured = ik.tcp_pose(self.measured[start:start + 6], BASES[arm])[1]
        want = self.side_rot(arm)
        if np.trace(measured.T @ want) < np.trace(measured.T @ (want * np.array((-1., -1., 1.)))):
            want = want * np.array((-1., -1., 1.))
        turn = want @ measured.T
        angle = float(np.arccos(np.clip((np.trace(turn) - 1.) / 2., -1., 1.)))
        axis = np.array((turn[2, 1] - turn[1, 2], turn[0, 2] - turn[2, 0], turn[1, 0] - turn[0, 1]))
        return axis * (.5 if angle < 1e-6 else angle / (2. * np.sin(angle)))

    def hold(self, arm, tip=None, settle=True):
        """Command the hand that presses on the pack: its fingertip point would be ``tip`` with the fingers as
        side_rot has them; the hand is turned by ``self.twist`` about its pressing edge, which stays where it
        is. With ``settle`` the twist first takes up some of what the hand is still off."""
        if tip is not None:
            self.flat[arm] = np.asarray(tip, dtype=float).copy()
        flat = self.side_rot(arm)
        nominal = self.flat[arm] + flat @ CORNER - TIP * flat[:, 2]
        if settle and HOLD_TWIST:
            twist = self.twist[arm] + np.clip(.2 * self.hand_off(arm), -TWIST_RATE, TWIST_RATE)
            size = float(np.linalg.norm(twist))
            self.twist[arm] = twist if size <= TWIST_MAX else twist * (TWIST_MAX / size)
        angle = float(np.linalg.norm(self.twist[arm]))
        rot = flat
        if angle > 1e-9:
            k = self.twist[arm] / angle
            cross = np.array(((0., -k[2], k[1]), (k[2], 0., -k[0]), (-k[1], k[0], 0.)))
            rot = (np.eye(3) + np.sin(angle) * cross + (1. - np.cos(angle)) * cross @ cross) @ flat
        edge = nominal + SIDE[arm] * self.level
        self.tip[arm], self.rot[arm] = edge - rot @ CORNER + TIP * rot[:, 2], self.same_way(arm, rot)

    def reaches(self, arm, tip):
        """Whether the arm can put its fingertips at ``tip`` as it holds the pack."""
        before = self.flat[arm], self.tip[arm], self.rot[arm]
        self.hold(arm, tip, settle=False)
        _, error = self.solve(arm, self.tip[arm], self.rot[arm])
        self.flat[arm], self.tip[arm], self.rot[arm] = before
        return error < .001

    def pack_foot(self):
        """The held pack hangs up to 20 degrees off level, and will come down on its lower bottom edge and rock
        flat about it. Returns (x, y) of where its middle will then be, and the height of that lower edge."""
        pos, rot = self.item("pack")
        lean = float(np.arctan2(rot[0, 2], rot[2, 2]))          # positive: the far end hangs low
        half, high = m.PACK[0] / 2, m.PACK[2] / 2
        x = pos[0] + np.sign(lean) * half * (np.cos(lean) - 1.) - high * np.sin(lean)
        return np.array((x, pos[1], pos[2] - half * abs(np.sin(lean)) - high * np.cos(lean)))

    def take_pack(self):
        self.phase("pack: hands over it")
        # straight up first, turn the hands high and to the side, then across level: nothing passes low over the bag
        high = m.RIM + .135
        yield from self.line(1.2, {arm: (self.tip[arm][0], self.tip[arm][1], high) for arm in ARMS if arm in self.tip})
        above = {arm: self.beside(arm, PACK_STAND_OFF, m.PACK[2] + .060) for arm in ARMS}
        turned = {LEFT: ((above[LEFT][0], m.BAG_XY[1] + .190, high), self.side_rot(LEFT)),
                  RIGHT: ((above[RIGHT][0], above[RIGHT][1] - .030, high), self.side_rot(RIGHT))}
        yield from self.glide(2.4, turned, {arm: 0. for arm in ARMS})
        yield from self.line(2.4, above)
        ahead, held = AHEAD, False
        for attempt in range(3):
            self.squeeze = {arm: 0. for arm in ARMS}
            self.twist = {arm: np.zeros(3) for arm in ARMS}
            self.level = np.zeros(3)
            self.phase("pack: hands down beside it")
            back = PACK_STAND_OFF / GIVE[1]                    # start this far back along the way the hands will press
            beside = {arm: self.beside(arm, 0., ahead=ahead) + back * self.give(arm) for arm in ARMS}
            if attempt == 0:
                yield from self.line(1.6, beside)
            else:
                yield from self.glide(.8, {arm: (beside[arm], self.side_rot(arm)) for arm in ARMS})
            self.flat = {arm: self.tip[arm].copy() for arm in ARMS}
            self.phase("pack: press from both sides")
            # Press in. An arm gives under the push, and not straight back: commanded straight at the pack, a
            # fingertip would slide forward along the film and load the pack sideways. So each hand is commanded
            # against the way it gives; its pressing edge then stays where it first touched.
            pressed = 0.
            while pressed < SQUEEZE_MAX and min(self.press(arm) for arm in ARMS) < SQUEEZE_N:
                pressed += SQUEEZE_STEP
                for arm in ARMS:
                    if self.press(arm) < SQUEEZE_N + 3.:
                        self.flat[arm] = self.flat[arm] - self.give(arm) * SQUEEZE_STEP
                    self.hold(arm)
                yield self.emit()
            for _ in range(round(.4 * CONTROL_HZ)):
                for arm in ARMS:
                    self.hold(arm)
                yield self.emit()
            held = min(self.press(arm) for arm in ARMS) >= .6 * SQUEEZE_N
            self.event("pack pressed", press_n=[round(self.press(arm), 1) for arm in ARMS], travel_mm=round(1000 * pressed, 1),
                       held=held, attempt=attempt)
            if not held:
                break
            # A short lift: the pack hangs from the two pressing edges. If they are not over its middle it hangs
            # off level; it is then set down and taken again that much further along.
            self.phase("pack: try the lift")
            yield from self.pack_height(1.2, .030, keep=False)
            yield from self.wait(.5)
            pos, rot = self.item("pack")
            lean = float(np.arctan2(rot[0, 2], rot[2, 2]))      # positive: the far end hangs low
            self.event("pack hanging", lean_deg=round(float(np.degrees(lean)), 1), attempt=attempt,
                       twist_deg={arm: [round(float(v), 1) for v in np.degrees(self.twist[arm])] for arm in ARMS},
                       off_middle_mm=[round(1000 * (self.touch(arm)[0] - float(pos[0])), 1) for arm in ARMS])
            if abs(lean) <= np.radians(LEVEL_DEG) or attempt == 2:
                break
            ahead += (m.PACK[2] / 2 - PACK_DOWN) * float(np.tan(lean))
            self.phase("pack: set it down and take it again")
            yield from self.pack_height(1.0, .001, keep=False)
            yield from self.shift(.8, {arm: (pressed + back) * self.give(arm) for arm in ARMS})
            yield from self.wait(.3)
        return held

    def pack_height(self, seconds, underside, xy=None, keep=True, within=None):
        """Bring the held pack's lower bottom edge to ``underside`` (and where its middle will be once it stands
        flat over ``xy``), correcting for what the arms give under its weight; with ``keep``, also keeping it
        pressed on the way. With ``within``, stay until it is that close to ``xy`` (the two arms give unequally,
        and the pack drifts some centimetres sideways between them on the way across)."""
        steps = max(1, round(seconds * CONTROL_HZ))
        start = {arm: self.flat[arm] + self.give(arm) * self.squeeze[arm] for arm in ARMS}
        foot = self.pack_foot()
        move = np.array((0., 0., underside - foot[2]))
        if xy is not None:
            move[:2] = np.asarray(xy, dtype=float) - foot[:2]
        trim = np.zeros(3)
        for step in range(1, steps + round((.6 if within is None else 5.) * CONTROL_HZ) + 1):
            alpha = min(1., step / steps)
            alpha *= alpha * (3. - 2. * alpha)
            foot = self.pack_foot()
            if within is not None and step > steps + 8 and float(np.linalg.norm(np.asarray(xy) - foot[:2])) < within:
                break
            if keep and LEVEL_GAIN:
                # The pack hangs on the line through the two pressing edges: it is tipped sideways by as much as
                # one edge is higher than the other, and turned by as much as one is ahead of the other.
                rot = self.item("pack")[1]
                droop = float(np.arctan2(rot[1, 2], rot[2, 2]))       # positive: its left side hangs low
                swung = float(np.arctan2(rot[1, 0], rot[0, 0]))       # positive: its far end is turned to the left
                want = m.PACK[1] / 2 * np.array((np.tan(swung), 0., np.tan(droop)))
                self.level = np.clip(self.level + np.clip(LEVEL_GAIN * want, -.001, .001), -LEVEL_MAX, LEVEL_MAX)
            for arm in ARMS if keep else ():
                if self.press(arm) < KEEP_N:
                    self.squeeze[arm] = min(KEEP_MAX, self.squeeze[arm] + KEEP_STEP)
                elif self.press(arm) > KEEP_N + 5.:
                    self.squeeze[arm] = max(0., self.squeeze[arm] - KEEP_STEP / 2)
            want = np.array((0., 0., underside)) if xy is None else np.array((*xy, underside))
            if step > steps // 2:                                # late in the move, close on what is measured
                error = want - foot
                if xy is None:
                    error[:2] = 0.
                tried = np.clip(trim + np.clip(.15 * (error - (1. - alpha) * move), -.0015, .0015),
                                (-.050, -.050, -.030), (.050, .050, .030))
                # a correction is taken only if both arms can reach where it puts them: one that cannot leaves
                # its hand behind, the pack turns between the hands, and the correction grows
                if all(self.reaches(arm, start[arm] + alpha * move + tried - self.give(arm) * self.squeeze[arm])
                       for arm in ARMS):
                    trim = tried
            for arm in ARMS:
                self.hold(arm, start[arm] + alpha * move + trim - self.give(arm) * self.squeeze[arm])
            yield self.emit()
            if keep and step % 20 == 0:
                self.event("carrying", lean_deg=round(float(np.degrees(np.arctan2(*self.item("pack")[1][[0, 2], 2]))), 1),
                           press_n=[round(self.press(arm), 1) for arm in ARMS],
                           twist_deg={arm: [round(float(v), 1) for v in np.degrees(self.twist[arm])] for arm in ARMS},
                           off_deg={arm: round(float(np.degrees(np.linalg.norm(self.hand_off(arm)))), 1) for arm in ARMS},
                           level_mm=mm(self.level),
                           tilt_deg=round(float(np.degrees(np.arccos(np.clip(self.item("pack")[1][2, 2], -1., 1.)))), 1),
                           squeeze_mm=mm([self.squeeze[arm] for arm in ARMS]), trim_mm=mm(trim))

    def carry_pack(self):
        self.phase("pack: lift")
        yield from self.pack_height(1.4, .090)
        self.event("pack lifted", press_n=[round(self.press(arm), 1) for arm in ARMS])
        self.phase("pack: up and over to the bag")
        pos, _ = self.item("pack")
        place = self.in_bag(m.PACK_SLOT)[:2]
        wall = self.in_bag(m.hinge_point("right"))[1]              # the bag's wall on the pack's side
        short = np.array((pos[0], wall - m.PACK[1] / 2 - .035))     # as far as it goes before it must be over the rim
        yield from self.pack_height(2.2, m.RIM + PACK_OVER_RIM, short)
        yield from self.pack_height(3.4, m.RIM + PACK_OVER_RIM, place, within=.004)
        self.event("pack over the bag", off_mm=mm(place - self.pack_foot()[:2]), press_n=[round(self.press(arm), 1) for arm in ARMS],
                   lean_deg=round(float(np.degrees(np.arctan2(*self.item("pack")[1][[0, 2], 2]))), 1),
                   squeeze_mm=mm([self.squeeze[arm] for arm in ARMS]), foot_mm=mm(self.pack_foot()))

    def lower_pack(self):
        self.phase("pack: lower to the rim")
        rim = max(self.state["rims"][w] for w in ("left", "right"))
        touch = rim + RIM_CLEAR + TIP_LOW                        # the pressing edge with the fingertips clear of the rim
        underside = max(self.base_top() + .004, touch + PACK_DOWN - m.PACK[2])
        yield from self.pack_height(2.4, underside, self.in_bag(m.PACK_SLOT)[:2])
        self.event("pack at the rim", above_base_mm=round(1000 * (self.item("pack")[0][2] - m.PACK[2] / 2 - self.base_top()), 1))

    def let_pack_down(self, at_once=False):
        self.phase("let down: ease the hands apart")
        start = float(self.item("pack")[0][2])
        eased = 0.
        while not at_once and eased < EASE_MAX:
            eased += EASE_STEP
            for arm in ARMS:
                self.tip[arm][1] += SIDE[arm] * EASE_STEP
            yield self.emit()
            height = float(self.item("pack")[0][2])
            if height - m.PACK[2] / 2 - self.base_top() < .002:
                break
            if start - height > .004:                             # it is sliding: hold the hands where they are
                for _ in range(round(.15 * CONTROL_HZ)):
                    yield self.emit()
                start = float(self.item("pack")[0][2])
        self.event("pack let down", eased_mm=round(1000 * eased, 1),
                   above_base_mm=round(1000 * (self.item("pack")[0][2] - m.PACK[2] / 2 - self.base_top()), 1))
        self.phase("let down: draw the hands away")
        yield from self.shift(.9 if not at_once else .25, {arm: (0., SIDE[arm] * .045, 0.) for arm in ARMS})
        yield from self.wait(.4)
        yield from self.shift(.8, {arm: (-.030, SIDE[arm] * .020, .110) for arm in ARMS})
        self.event("pack in")

    # -- the handles, hanging --------------------------------------------------------------
    def hook_pitch(self, angle):
        """How far below level the fingers point with the handle swung ``angle`` up: HOOK_PITCH low down, where the
        arm needs it to reach, level by the time the handle is over the rim."""
        return HOOK_PITCH * float(np.clip((HOOK_TOP - angle) / (HOOK_TOP - HOOK_LEVEL), 0., 1.))

    def hook_rot(self, arm, angle=0.):
        return frame(aim(HOOK_YAW, self.hook_pitch(angle), SIDE[arm]), (0., 0., 1.))

    def handle_angle(self, arm):
        """How far the arm's handle has swung up from hanging straight down (0) toward standing over the rim (pi)."""
        wall = WALL[arm]
        offset = np.asarray(self.state["grips"][wall], dtype=float) - np.asarray(self.state["handles"][wall][0], dtype=float)
        angle = float(np.arctan2(SIDE[arm] * offset[1], -offset[2]))
        return angle if angle > -.6 else angle + 2 * np.pi          # a little past straight up is still "up"

    def hook_point(self, arm, angle, past=HOOK_PAST, rim=None):
        """Fingertip point with the fingertips' ends ``past`` beyond where the middle of the handle's grip is when
        the handle has swung ``angle`` up from hanging; taken HOOK_ALONG from the middle of the grip's length.
        ``rim``: the middle of the wall's rim, if not where it is now."""
        wall = WALL[arm]
        live, rot = self.state["handles"][wall]
        reach = m.HANDLE_DROP - m.GRIP_W / 2
        along = np.asarray(rot, dtype=float)[:, 0]
        centre = (np.asarray(live if rim is None else rim, dtype=float) + along * HOOK_ALONG * float(np.sign(along[0]))
                  + reach * np.array((0., SIDE[arm] * np.sin(angle), -np.cos(angle))))
        return centre + past * aim(HOOK_YAW, self.hook_pitch(angle), SIDE[arm])

    def swing(self, seconds, begin, rim, first, last):
        """Both hands carry their handles along the handles' circles about the rims, from share ``first`` to
        ``last`` of the way between each handle's ``begin`` angle and HOOK_TOP, together and on a fixed schedule.
        The circles are drawn about where the rims were (``rim``): hands that followed a moving bag would carry
        it off across the table."""
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            share = first + alpha * (last - first)
            for arm in ARMS:
                angle = begin[arm] + share * (HOOK_TOP - begin[arm])
                self.tip[arm] = self.hook_point(arm, angle, rim=rim[arm])
                self.rot[arm] = self.same_way(arm, self.hook_rot(arm, angle))
                turned = self.handle_angle(arm)
                between = m.GRIP_W * abs(np.cos(turned)) + m.GRIP_T * abs(np.sin(turned))
                self.gap[arm] = min(HOOK_GAP, max(self.gap[arm] - .002, between + HOOK_ROOM))
            yield self.emit()

    def grip_point(self, arm):
        """Fingertip point for closing on the arm's handle where its grip is now: the fingertips' ends HOOK_PAST
        beyond the grip's middle, HOOK_ALONG toward the box, the fingers pitched for the handle's present angle."""
        wall = WALL[arm]
        along = np.asarray(self.state["handles"][wall][1], dtype=float)[:, 0]
        angle = self.handle_angle(arm)
        centre = np.asarray(self.state["grips"][wall], dtype=float) + along * HOOK_ALONG * float(np.sign(along[0]))
        return centre + HOOK_PAST * aim(HOOK_YAW, self.hook_pitch(angle), SIDE[arm]), angle

    def onto_grips(self, seconds):
        aims = {arm: self.grip_point(arm) for arm in ARMS}
        rots = {arm: self.same_way(arm, self.hook_rot(arm, aims[arm][1])) for arm in ARMS}
        yield from self.line(seconds, {arm: aims[arm][0] for arm in ARMS}, rots)

    def unhook(self, begin):
        """Let the handles back down part of the way on the fingers and draw the fingers out lengthwise."""
        rim = {arm: np.asarray(self.state["handles"][WALL[arm]][0], dtype=float).copy() for arm in ARMS}
        yield from self.grip(.4, {arm: HOOK_GAP for arm in ARMS})
        yield from self.swing(2.4, begin, rim, 1., .30)
        yield from self.shift(1.2, {arm: -(HOOK_OUT + .020) * self.rot[arm][:, 2] for arm in ARMS})

    def lift_bag(self):
        self.phase("lift: fingers through the handles")
        # each hand comes down beside its handle and slides its fingers in lengthwise, one above and one below
        # the grip, until the grip is over the middle of the lower finger
        begin = {arm: self.handle_angle(arm) for arm in ARMS}
        rim = {arm: np.asarray(self.state["handles"][WALL[arm]][0], dtype=float).copy() for arm in ARMS}
        back = {arm: self.hook_point(arm, begin[arm]) - HOOK_OUT * aim(HOOK_YAW, HOOK_PITCH, SIDE[arm]) for arm in ARMS}
        yield from self.glide(3.0, {arm: (back[arm] + (0., 0., .060), self.hook_rot(arm)) for arm in ARMS},
                              {arm: HOOK_GAP for arm in ARMS})
        yield from self.line(.9, back)
        yield from self.line(1.2, {arm: self.hook_point(arm, begin[arm]) for arm in ARMS})
        self.event("fingers through the handles", handle_deg=[round(float(np.degrees(begin[arm])), 0) for arm in ARMS])
        self.phase("lift: swing the handles up")
        # the grips ride on the lower fingers; both hands go up together, slowly
        yield from self.swing(SWING_S, begin, rim, 0., 1.)
        yield from self.wait(.4)
        up = [self.handle_angle(arm) for arm in ARMS]
        self.event("handles up", handle_deg=[round(float(np.degrees(v)), 0) for v in up])
        held = [False, False]
        if min(up) >= HOOK_TOP - np.radians(30.):
            for attempt in range(2):
                # a handle can come to rest a little short of or past where it was carried to: bring the pads
                # back to where each grip actually is before closing on it
                yield from self.onto_grips(.6)
                yield from self.grip(.5, {arm: HOOK_PINCH for arm in ARMS})
                yield from self.wait(.2)
                held = [self.opening(arm) > HOOK_PINCH + .006 for arm in ARMS]
                self.event("grips pinched", held=held, attempt=attempt + 1)
                if all(held):
                    break
                yield from self.grip(.4, {arm: HOOK_GAP for arm in ARMS})
        if not all(held):                                       # never lift the bag by one handle: it would tip over
            self.phase("lift: let go, a handle was missed")
            yield from self.unhook(begin)
            return
        self.phase("lift: up")
        start = {arm: self.tip[arm].copy() for arm in ARMS}
        rise = 0.
        for _ in range(round(5. * CONTROL_HZ)):
            short = LIFT_BASE - self.base_low()
            if short < .001 or rise >= LIFT_MAX:
                break
            rise = min(LIFT_MAX, rise + min(.0025, max(.0006, .08 * short)))
            for arm in ARMS:
                self.tip[arm] = start[arm] + (0., 0., rise)
            yield self.emit()
        self.event("bag up", rise_mm=round(1000 * rise, 1), base_low_mm=round(1000 * self.base_low(), 1))
        self.phase("lift: hold, and back over the marks")
        # held up, the bag is brought back over its tape marks: whatever it crept across the table earlier
        back = np.zeros(3)
        for step in range(round(HOLD_S * CONTROL_HZ)):
            off = np.asarray(m.BAG_XY, dtype=float) - np.asarray(self.state["base"][0], dtype=float)[:2]
            back[:2] = np.clip(back[:2] + np.clip(.08 * off, -.0015, .0015), -.070, .070)
            for arm in ARMS:
                self.tip[arm] = start[arm] + (0., 0., rise) + back
            yield self.emit()
        self.event("bag held", base_tilt_deg=round(float(np.degrees(np.arccos(np.clip(self.state["base"][1][2, 2], -1., 1.)))), 1),
                   base_low_mm=round(1000 * self.base_low(), 1), moved_back_mm=mm(back[:2]))
        self.phase("lift: set down")
        down = 0.
        for _ in range(round(5. * CONTROL_HZ)):
            if float(self.state["base"][0][2]) < .0008 and self.base_low() < .0008:
                break
            down = min(rise + .010, down + .0022)
            for arm in ARMS:
                self.tip[arm] = start[arm] + (0., 0., rise - down) + back
            yield self.emit()
        yield from self.wait(.4)
        self.phase("lift: let go")
        yield from self.unhook(begin)
        self.event("bag set down")

    # -- the demonstration ------------------------------------------------------------------
    def sequence(self):
        variant = self.variant
        if variant == "large_first":
            if (yield from self.take_pack()):
                yield from self.carry_pack()
                yield from self.lower_pack()
                yield from self.let_pack_down()
        else:
            yield from self.take_carton()
            yield from self.carry_carton()
            if variant != "no_opening":
                yield from self.raise_handle()
            yield from self.set_carton()
            if variant != "no_opening":
                yield from self.release_handle()
            if variant != "carton" and (yield from self.take_pack()):
                yield from self.carry_pack()
                yield from self.lower_pack()
                yield from self.let_pack_down(at_once=variant == "drop_from_rim")
                if variant != "no_lift":
                    yield from self.lift_bag()
        self.phase("away")
        yield from self.home(ARMS, 2.2)
        self.event("parked")
        self.phase("wait")
        while True:
            yield self.emit()
