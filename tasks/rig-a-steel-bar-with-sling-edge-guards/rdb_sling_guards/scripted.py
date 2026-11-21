"""Full-score demonstration. Each arm works its own side of the bar (the left arm the left edges and the left horn).

1. Guards: first each bottom-edge guard, one hand at a time: the hand takes it off its tray's block by its two end
   walls (pads closing along the guard's length, the hand above and outside), slides it in under the bar's bottom face
   5 mm low, raises it to 2 mm off its seat and lets go; the magnets pull it home. Then both top-edge guards at once:
   each hand takes the tent-shaped guard out of its tray by the end walls, turns it 45 degrees and brings it in toward
   the edge from above and outside to 2 mm off its seat, and lets go.
2. The right fitting: the right hand pinches the fitting's side bar nearer the robot, 18 mm from the base bar, pads
   square to the bar, and lifts it off the table; carries it beside the hook's right horn. The left hand pinches the
   eye's left side wall and holds the hook still while the right hand brings the apex bar over the horn's tip, onto
   the latch, presses the latch down, slides the apex bar past the latch's end into the bowl and opens its fingers a
   little: the fitting drops onto the bowl's floor and the latch springs back over it. Both hands come away.
3. The left fitting the same way, mirrored (the right hand holds the eye's right side wall: by then the right
   fitting's triangle wraps the right horn).
4. Both arms go back to their ready poses and stay there while the operator's test runs (the scorer's hoist).

Positions are in the tabletop frame. The demonstration reads the robot's own joint and gripper readings and simulator
state (each guard's and fitting's pose, the hook's frame); it changes nothing in the simulation.

``RDB_RIGGING_DEMO`` (or ``variant``) selects a control:
- "no_guards": hooks both fittings and clears without placing any guard (must score 0);
- "top_only": places only the two top-edge guards, then hooks both and clears (must score 0: the test fails);
- "guards_only": places all four guards and stops (must score 0.4);
- "not_past_latch": guards, then the right fitting's apex bar is set on its closed latch and held there, never pressed
  past it (must score 0.4: no fitting in a bowl; released there, the fitting's own weight would open this light latch);
- "one_fitting": guards, then only the right fitting is hooked, and the hands clear (must score 0.6).
"dev_bottom" (development only) places the two bottom-edge guards and stops.
"""

import math
import os

from inspect_robots import Action, ActionChunk
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY

from . import fixed_scene as F
from . import grading
from . import layout as L

OPEN = float(FINGER_OPEN)                    # finger joint, fully open (pads 93.9 mm apart)
CONTROL_HZ = F.CONTROL_HZ
BASES = {arm: np.asarray(F.BASES[arm], dtype=float) for arm in F.ARMS}
SIDE = {"left": 1., "right": -1.}
SEEDS = [np.array(s, dtype=float) for s in (      # starting guesses for inverse kinematics (probe/ik_scan.py)
    (0, 1, 1, -.6, 0, 0), (0, 1.6, 1.3, -1.2, 0, 0), (0, 2.1, 1.0, -.4, 1.5, -.1), (0, 2.1, 1.0, -.4, -1.5, .1),
    (.35, 2.2, .8, 1.4, -1.2, 0), (-.35, 2.2, .8, 1.4, 1.2, -1.57), (.4, 2.5, 1.4, -1.6, 1.1, 0),
    (0, 1.2, 1.6, -1.0, 0, 1.57), (0, 1.2, 1.6, -1.0, 0, -1.57), (0, 2.4, 2.0, -1.4, 0, 0),
    (-.6, 1.5, 1.2, -.3, 0, 0), (.6, 1.5, 1.2, -.3, 0, 0), (0, .6, .6, .6, 0, 0), (0, 1.0, .5, .9, 0, 0))]
SOLVE_ITERS = 150
POS_TOL, ROT_TOL = 6e-4, 6e-3

# -- guards ---------------------------------------------------------------------------------------------------------
# How each hand holds a guard by its two end tabs (pads closing along the guard's length), as set at the guard's seat:
# the tool point this far out from the side face (dy) and up (dz) from the inside corner, the approach in the plane
# across the edge, psi degrees below the side face's outward normal, and whether the fingers are swapped (flip).
# probe/guard_scan.py and probe/bl_grid.py chose these.
GUARD_GRIP = {"top": (.0168, .0139, 30., True), "bottom": (.016, .016, 65., False)}
GUARD_SQUEEZE = .010                             # pads commanded this much closer than the guard's length
GUARD_OPEN_GAP = L.GUARD_LEN + .014              # pads this far apart round the guard before closing
RELEASE_OFF = .002                               # let go with the guard this far off its seat (both faces)
BOTTOM_LOW = .005                                # the bottom-edge guard slides in this far under its seat ...
BOTTOM_OUT = .045                                # ... from this far outside it
TOP_OUT = .030                                   # the top-edge guard comes in along its bisector from this far out

# -- fittings -------------------------------------------------------------------------------------------------------
FIT_UG = .018                                    # the hand holds a side bar this far along it from the base bar ...
FIT_SIDE = -1.                                   # ... the side bar nearer the robot (x below the sling's plane)
FIT_GAMMA = 75.                                  # approach turned this far from base->apex toward the hand's face normal
FIT_GRIP_BACK = .0100                            # tool point this far back along the approach from the bar's centre
                                                 # (picking: the pad tips clear the web under the fitting)
CARRY_Y, CARRY_Z = .25, .12                      # the carry passes over this point (|y|, z) on its way to the horn
FIT_OPEN_GAP = .030
FIT_CLOSE_GAP = 0.
FIT_RELEASE_GAP = 2 * L.FIT_SIDE_R + .003       # let go of the hung fitting by opening only this far (the inner pad
                                                 # is a few millimetres from the horn)
FIT_DX = .003                                    # hang the fitting this far off the horn's middle toward the held
                                                 # side bar (more room for the inner pad)
# the hang (probe/hang_plan.py, path "f35"): apex-bar centre (u out along the horn's throat, v up, from its bowl centre)
# and the fitting's angle phi (apex to base below the outward horizontal)
HANG_PATH = (("approach", (.070, .047), 35.), ("over the tip", (.025, .047), 35.), ("on the latch", (.0247, .032), 35.),
             ("pressing", (.016, .0195), 30.), ("past the latch", (.010, .005), 25.))
# The approach is level with the point over the tip, not above it: the higher the first fitting is carried, the more web
# it pulls from under the bar, dragging the other fitting toward the bar (a native run left it at the bar's side face)
STEADY_EYE = (.035, .055)                        # the steadying hand pinches the eye's side wall here (|y|, z from the
                                                 # midpoint of the bowl centres; probe/steady_diag.py)
STEADY_DOWN, STEADY_TILT = 30., 30.              # in from outside, 30 degrees down, turned 30 degrees forward


def grip_frame(approach, closing):
    """Gripper rotation (columns: closing direction, pad width, approach) from two directions."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, dtype=float)
    c = c - a * (c @ a)
    c /= np.linalg.norm(c)
    return np.column_stack((c, np.cross(a, c), a))


def rot(axis, ang):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    K = np.array(((0., -axis[2], axis[1]), (axis[2], 0., -axis[0]), (-axis[1], axis[0], 0.)))
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def gap_to_finger(gap):
    """Finger joint command for a pad gap (m)."""
    return float(np.clip(-gap / 2, OPEN, 0.))


def seat_frame(edge):
    return np.array(grading.seat_rotation(edge))


def edge_point(edge):
    (y, z), _, _ = L.EDGES[edge]
    return np.array((L.XS, y, z))


def guard_grip_local(edge):
    """(tool point, hand rotation) in the guard's own frame (inside corner at its origin)."""
    dy, dz, psi, flip = GUARD_GRIP["top" if edge in ("TL", "TR") else "bottom"]
    Rs = seat_frame(edge)
    n2, up = Rs[:, 1], np.array((0., 0., 1.))
    a = -(math.cos(math.radians(psi)) * n2 + math.sin(math.radians(psi)) * up)
    R = grip_frame(a, Rs[:, 0])
    if flip:
        R = R @ np.diag((-1., -1., 1.))
    return Rs.T @ (dy * n2 + dz * up), Rs.T @ R


def guard_grip(edge, corner, Rg):
    """Tool point and hand rotation that hold a guard shaped for ``edge`` (its inside corner at ``corner``, rotation
    ``Rg``) by its two end tabs."""
    lp, lR = guard_grip_local(edge)
    return corner + Rg @ lp, Rg @ lR


class RiggingReference:
    """Emit one absolute 14-D joint action per control step."""

    def __init__(self, variant=None):
        self.variant = variant or os.environ.get("RDB_RIGGING_DEMO", "full")
        self.measured = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.state = None
        self.joints = {arm: np.asarray(READY, dtype=float).copy() for arm in F.ARMS}
        self.aperture = dict.fromkeys(F.ARMS, OPEN)
        self.cmd = {arm: None for arm in F.ARMS}          # commanded tool point and rotation per arm
        self.offset = {arm: np.zeros(3) for arm in F.ARMS}
        self.worst_ik = 0.
        self.phases = []
        self.tick = 0
        self.log = {"events": [], "variant": self.variant}
        self.done = False

    # -- readings -------------------------------------------------------------------------------------------------
    def observe(self, joint_pos, state=None):
        self.measured = np.asarray(joint_pos, dtype=float).copy()
        self.state = state

    def actual(self, arm):
        start = 7 * F.ARMS.index(arm)
        return ik.tcp_pose(self.measured[start:start + 6], BASES[arm])

    def opening(self, arm):
        return float(self.measured[7 * F.ARMS.index(arm) + 6]) * 2 * abs(OPEN)

    def guard(self, name):
        p, q = self.state["guards"][name]
        return np.asarray(p, dtype=float), np.array(grading.quat_matrix(q))

    def fitting(self, name):
        return self.state["fits"][name]

    def note(self, name, **values):
        def clean(v):
            if isinstance(v, str):
                return v
            if np.isscalar(v):
                return round(float(v), 4)
            return [clean(x) for x in v]
        self.log["events"].append({"tick": self.tick, "event": name, **{k: clean(v) for k, v in values.items()}})

    def phase(self, name):
        self.phases.append((self.tick, name))

    # -- inverse kinematics ---------------------------------------------------------------------------------------
    def solve(self, arm, p, R, seed=None, search=True):
        """Joints for the tool point ``p`` and rotation ``R``: from ``seed`` (or the current command) first; if that
        misses, every starting guess, keeping the solution furthest from the joint limits."""
        quat = ik.matrix_to_quat_xyzw(R)
        first = self.joints[arm] if seed is None else seed
        q, err, rerr = ik.solve(np.asarray(p, dtype=float), quat, first, BASES[arm], iters=SOLVE_ITERS)
        margin = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
        if (err < POS_TOL and rerr < ROT_TOL and margin > math.radians(2.)) or not search:
            self.worst_ik = max(self.worst_ik, float(err))
            return np.asarray(q, dtype=float)
        best = None
        s = SIDE[arm]
        for base in SEEDS:
            for sd in (np.array((s, 1, 1, 1, s, s)), np.array((-s, 1, 1, 1, -s, -s))):
                q, err, rerr = ik.solve(np.asarray(p, dtype=float), quat, base * sd, BASES[arm], iters=SOLVE_ITERS)
                if err < POS_TOL and rerr < ROT_TOL:
                    margin = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
                    if best is None or margin > best[1]:
                        best = (np.asarray(q, dtype=float), margin)
        if best is None:
            raise RuntimeError(f"{arm} arm cannot reach {np.round(1000 * np.asarray(p), 1)} mm")
        self.note("ik search", arm=arm, margin_deg=math.degrees(best[1]), point_mm=1000 * np.asarray(p))
        return best[0]

    def margin(self, arm, p, R):
        try:
            q = self.solve(arm, p, R)
        except RuntimeError:
            return -1.
        return float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())

    def better_roll(self, arm, p, R):
        """Of a grip and the same grip turned half a turn about the approach (the fingers swapped), the one with more
        room from the joint limits."""
        flipped = R @ np.diag((-1., -1., 1.))
        return max((R, flipped), key=lambda r: self.margin(arm, p, r))

    # -- motion ---------------------------------------------------------------------------------------------------
    def emit(self):
        values = np.concatenate([np.r_[self.joints[a], np.clip(self.aperture[a] / OPEN, 0., 1.)] for a in F.ARMS])
        self.tick += 1
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=CONTROL_HZ)

    def hold(self, seconds):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield self.emit()

    def glide(self, seconds, targets, apertures=None):
        """Joint-space moves (smooth start and stop) to {arm: joints}, grippers to {arm: finger}."""
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

    def fingers(self, seconds, apertures):
        yield from self.glide(seconds, {}, apertures)

    def reach(self, seconds, poses, apertures=None):
        """Joint-space move to Cartesian poses {arm: (p, R)} (for free-space moves)."""
        targets = {}
        for arm, (p, R) in poses.items():
            targets[arm] = self.solve(arm, p + self.offset[arm], R)
            self.cmd[arm] = (np.asarray(p, dtype=float), R)
        yield from self.glide(seconds, targets, apertures)

    def line(self, seconds, poses, apertures=None):
        """Straight-line moves of the tool points with the rotations turned evenly, {arm: (p, R)}; IK every step."""
        start = {arm: self.cmd[arm] for arm in poses}
        first = {arm: self.aperture[arm] for arm in (apertures or {})}
        steps = max(1, round(seconds * CONTROL_HZ))
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha *= alpha * (3. - 2. * alpha)
            for arm, (p1, R1) in poses.items():
                p0, R0 = start[arm]
                p = p0 + alpha * (np.asarray(p1) - p0)
                R = ik.slerp_matrix(R0, R1, alpha)
                self.joints[arm] = self.solve(arm, p + self.offset[arm], R)
                self.cmd[arm] = (p, R)
            for arm, goal in (apertures or {}).items():
                self.aperture[arm] = first[arm] + alpha * (goal - first[arm])
            yield self.emit()

    def settle(self, arm, tries=3, tolerance=.0008, seconds=.3, free=False):
        """Shift the arm's command by its measured tool-point error until the tool point is where it is wanted (the
        arm sags under load; rdb_spark_plug/scripted.py). ``free``: nothing near the hand, so a large error is sag
        (an arm carrying a fitting and its web sags about 20 mm) and is taken out in full."""
        want, R = self.cmd[arm]
        limit, step = (.035, .012) if free else (.008, .003)
        for _ in range(tries):
            yield from self.hold(.25)
            error = want - self.actual(arm)[0]
            if np.linalg.norm(error) < tolerance:
                break
            if not free and np.linalg.norm(error) > .006:     # held back by contact, not sag: do not push harder
                self.note("settle refused", arm=arm, error_mm=1000 * error)
                break
            self.offset[arm] = np.clip(self.offset[arm] + np.clip(error, -step, step), -limit, limit)
            self.joints[arm] = self.solve(arm, want + self.offset[arm], R)
            yield from self.hold(seconds)
        self.note("settled", arm=arm, error_mm=1000 * (want - self.actual(arm)[0]), offset_mm=1000 * self.offset[arm])

    def ready(self, seconds, arms=F.ARMS):
        yield from self.glide(seconds, {arm: np.asarray(READY, dtype=float) for arm in arms},
                              {arm: OPEN for arm in arms})
        for arm in arms:
            self.cmd[arm] = None
            self.offset[arm] = np.zeros(3)

    def start_cmd(self, arm):
        if self.cmd[arm] is None:
            self.cmd[arm] = ik.tcp_pose(self.joints[arm], BASES[arm])

    # -- guards ---------------------------------------------------------------------------------------------------
    def guard_plan(self, name):
        side, edge, _ = F.tray_slots()[name]
        arm = side
        p0, R0 = self.guard(name)
        grip_p, grip_R = guard_grip(edge, p0, R0)
        return arm, edge, (p0, R0), (grip_p, grip_R)

    def pick_guards(self, names):
        """Both arms at once: over each guard, down onto its tabs, pinch, lift."""
        plans = {}
        for name in names:
            arm, edge, _, (gp, gR) = self.guard_plan(name)
            plans[arm] = (name, edge, gp, gR)
        above = {arm: (gp + np.array((0., 0., .045)), gR) for arm, (_, _, gp, gR) in plans.items()}
        yield from self.reach(2.0, above, {arm: gap_to_finger(GUARD_OPEN_GAP) for arm in plans})
        yield from self.line(1.2, {arm: (gp, gR) for arm, (_, _, gp, gR) in plans.items()})
        for arm in plans:
            yield from self.settle(arm, tries=2)
        yield from self.fingers(.6, {arm: gap_to_finger(L.GUARD_LEN - GUARD_SQUEEZE) for arm in plans})
        yield from self.hold(.3)
        for arm, (name, _, _, _) in plans.items():
            self.note("guard pinched", arm=arm, guard=name, pads_mm=1000 * self.opening(arm))
        # where each guard sits in its hand, from the readings: the hand's measured pose and the guard's
        self.held = {}
        for arm, (name, edge, _, _) in plans.items():
            hp, hR = self.actual(arm)
            p, R = self.guard(name)
            self.held[arm] = (name, edge, hR.T @ (p - hp), hR.T @ R)
        yield from self.line(1.0, {arm: (self.cmd[arm][0] + np.array((0., 0., .05)), self.cmd[arm][1]) for arm in plans})

    def hand_for_guard(self, arm, p, R):
        """The hand's pose that puts the held guard's corner at ``p`` with rotation ``R``."""
        _, _, lp, lR = self.held[arm]
        hR = R @ lR.T
        return p - hR @ lp, hR

    def place_guards(self):
        """Both arms at once: carry the held guards to their edges and let go just off the seats."""
        arms = list(self.held)
        moves = {}
        for arm in arms:
            name, edge, _, _ = self.held[arm]
            seat, Rs = edge_point(edge), seat_frame(edge)
            b = Rs @ np.array((0., 1., 1.)) / math.sqrt(2.)
            if edge in ("TL", "TR"):
                path = [seat + TOP_OUT * b, seat + RELEASE_OFF * math.sqrt(2.) * b]
            else:
                out = np.array((0., math.copysign(1., Rs[1, 1]), 0.))          # the side face's outward normal
                low = np.array((0., 0., -BOTTOM_LOW))
                path = [seat + BOTTOM_OUT * out + low, seat + RELEASE_OFF * out + low,
                        seat + RELEASE_OFF * out - RELEASE_OFF * np.array((0., 0., 1.))]
            moves[arm] = (name, edge, Rs, path)
        # over to the first point of each path
        first = {arm: self.hand_for_guard(arm, path[0] + np.array((0., 0., .03)), Rs) for arm, (_, _, Rs, path) in moves.items()}
        yield from self.reach(2.4, first)
        yield from self.line(1.0, {arm: self.hand_for_guard(arm, path[0], Rs) for arm, (_, _, Rs, path) in moves.items()})
        for k in range(1, max(len(m[3]) for m in moves.values())):
            step = {arm: self.hand_for_guard(arm, path[min(k, len(path) - 1)], Rs) for arm, (_, _, Rs, path) in moves.items()}
            yield from self.line(1.2, step)
        for arm in arms:
            yield from self.settle(arm, tries=2)
        # correct by where the guard actually is (the grip may have slipped)
        for arm, (name, edge, Rs, path) in moves.items():
            p, _ = self.guard(name)
            err = path[-1] - p
            self.note("guard before release", arm=arm, guard=name, error_mm=1000 * err)
            if np.linalg.norm(err) > .001:
                hp, hR = self.cmd[arm]
                self.cmd[arm] = (hp + np.clip(err, -.004, .004), hR)
                self.joints[arm] = self.solve(arm, self.cmd[arm][0] + self.offset[arm], hR)
        yield from self.hold(.4)
        yield from self.fingers(.5, {arm: gap_to_finger(GUARD_OPEN_GAP) for arm in arms})
        yield from self.hold(.6)
        for arm, (name, edge, _, _) in moves.items():
            p, q = self.state["guards"][name]
            seated, det = grading.guard_seat(p, q)
            self.note("guard released", arm=arm, guard=name, seated=str(seated), corner_mm=det["corner_mm"],
                      angle_deg=det["angle_deg"], along_mm=det["along_mm"])
        back = {}
        for arm, (name, edge, Rs, path) in moves.items():
            hp, hR = self.cmd[arm]
            if edge in ("TL", "TR"):
                back[arm] = (hp - .05 * hR[:, 2], hR)
            else:                       # out from the side face first, clear of the top guard above
                back[arm] = (hp + .045 * Rs[:, 1] + np.array((0., 0., .01)), hR)
        yield from self.line(1.0, back)
        self.held = {}

    def guards(self, top=True, bottom=True):
        """Bottom-edge guards first, one hand at a time (two hands under the hook at once can squeeze it between
        them), then both top-edge guards at once: a top guard's end walls stand out from the side face over the
        bottom edge, where the hand placing the bottom guard passes."""
        if bottom:
            for name in ("guard_2", "guard_4"):
                self.phase(f"bottom guard {name}")
                yield from self.pick_guards((name,))
                yield from self.place_guards()
                yield from self.ready(1.6, (F.tray_slots()[name][0],))
        if top:
            self.phase("top guards")
            yield from self.pick_guards(("guard_1", "guard_3"))
            yield from self.place_guards()

    # -- the hook -------------------------------------------------------------------------------------------------
    def hook(self):
        return np.asarray(self.state["hook_O"], dtype=float), np.asarray(self.state["hook_R"], dtype=float)

    def horn_frame(self, side):
        """(bowl centre, throat direction t, up v, normal) of ``side``'s horn ('L' +y, 'R' -y) in the tabletop frame."""
        O, Rh = self.hook()
        s = 1. if side == "L" else -1.
        centre = O + Rh @ np.array((0., s * L.RAM_B / 2, 0.))
        t = Rh @ np.array((0., s, 0.))
        v = Rh @ np.array((0., 0., 1.))
        return centre, t, v, np.cross(t, v)

    def steady_pose(self, side):
        """The steadying hand on the eye's side wall on ``side`` ('L' +y, 'R' -y), above that horn's latch: pads
        closing across the forging's thickness, the hand in from outside and above. (Not the horn itself: once a
        fitting hangs in a horn, its triangle wraps that horn's outer wall.)"""
        O, Rh = self.hook()
        s = 1. if side == "L" else -1.
        p = O + Rh @ np.array((0., s * STEADY_EYE[0], STEADY_EYE[1]))
        t, v = Rh @ np.array((0., s, 0.)), Rh @ np.array((0., 0., 1.))
        a = -(math.cos(math.radians(STEADY_DOWN)) * t + math.sin(math.radians(STEADY_DOWN)) * v)
        a = rot(np.cross(a, (1., 0., 0.)), math.radians(STEADY_TILT)) @ a
        return p, grip_frame(a, (1., 0., 0.))

    # -- fittings -------------------------------------------------------------------------------------------------
    def fit_frame(self, name):
        """(base-bar centre, u base->apex, e along the bars with (e x u) toward the hand's side, n = e x u)."""
        f = self.fitting(name)
        base, u = np.asarray(f["base"], dtype=float), np.asarray(f["u"], dtype=float)
        e = np.array((1., 0., 0.))
        n = np.cross(e, u)
        if n[2] < 0:
            e, n = -e, -n
        return base, u, e, n

    def fit_grip(self, base, u, e, n, gamma=FIT_GAMMA):
        """Tool point and hand rotation that hold a fitting (frame from fit_frame) by the side bar nearer the robot,
        pads closing across it."""
        xoff = L.FIT_BASE_HALF - (L.FIT_BASE_HALF - L.FIT_APEX_HALF) * FIT_UG / L.FIT_H
        x = np.array((1., 0., 0.))
        side_bar = base + FIT_UG * u + FIT_SIDE * xoff * x
        # the side bar runs in from the base bar's end toward the apex bar's: the pads close square to it, in the
        # fitting's plane, so that each pad lies along the bar (a line contact, not a point at one edge of the pad)
        along = -FIT_SIDE * (L.FIT_BASE_HALF - L.FIT_APEX_HALF) * x + L.FIT_H * u
        along /= np.linalg.norm(along)
        across = np.cross(along, n)
        g = math.radians(gamma)
        a = math.cos(g) * along - math.sin(g) * n
        return side_bar - FIT_GRIP_BACK * a, grip_frame(a, across)

    def hang_fit_frame(self, side, A, phi):
        """(base-bar centre, u base->apex) that put the apex bar at (u, v) of ``side``'s horn at angle ``phi``."""
        centre, t, v, _ = self.horn_frame(side)
        apex = centre + A[0] * t + A[1] * v
        ph = math.radians(phi)
        u = -t * math.cos(ph) + v * math.sin(ph)
        return apex - L.FIT_H * u, u

    def roll_for_the_hang(self, arm, side, pick, R, base, u, e):
        """Of the grip and the same grip turned half a turn about the approach (the fingers swapped), the one whose
        worst joint margin over the pick, the carry's corner and every point of the hang is larger: the grip fixes how
        the hand holds the fitting all the way to the horn."""
        Rf0 = np.column_stack((e, u, np.cross(e, u)))
        best = None
        for cand in (R, R @ np.diag((-1., -1., 1.))):
            lp, lR = cand.T @ (base - pick), cand.T @ Rf0
            poses = [(pick, cand), (np.array((pick[0], math.copysign(CARRY_Y, pick[1]), CARRY_Z)), cand)]
            for _, A, phi in HANG_PATH:
                fb, fu = self.hang_fit_frame(side, A, phi)
                fb = fb + FIT_SIDE * FIT_DX * np.array((1., 0., 0.))
                hR = np.column_stack((e, fu, np.cross(e, fu))) @ lR.T
                poses.append((fb - hR @ lp, hR))
            margin = min(self.margin(arm, p, Rm) for p, Rm in poses)
            self.note("grip roll", arm=arm, worst_margin_deg=math.degrees(margin))
            if best is None or margin > best[0]:
                best = (margin, cand)
        return best[1]

    def regrip_reading(self, name, arm, e, what):
        """Read again where the fitting sits in the hand (its weight and the web's pull turn it a little about the
        side bar on the way): the path is planned for the fitting, not for the hand."""
        hp, hR = self.actual(arm)
        b, uu, _, _ = self.fit_frame(name)
        Rf = np.column_stack((e, uu, np.cross(e, uu)))
        new = (hR.T @ (b - hp), hR.T @ Rf)
        turn = math.degrees(math.acos(np.clip((np.trace(new[1].T @ self.fit_in_hand[1]) - 1) / 2, -1., 1.)))
        self.note(what, arm=arm, fitting=name, moved_in_hand_mm=1000 * float(np.linalg.norm(new[0] - self.fit_in_hand[0])),
                  turned_in_hand_deg=turn)
        self.fit_in_hand = new

    def hang_fitting(self, name, side, arm, steady_arm, past_latch=True):
        """Pick ``name`` off the table, hang it on ``side``'s horn, the other arm steadying the hook."""
        self.phase(f"pick {name}")
        base, u, e, n = self.fit_frame(name)
        pick, grip_R = self.fit_grip(base, u, e, n)
        self.start_cmd(arm)
        grip_R = self.roll_for_the_hang(arm, side, pick, grip_R, base, u, e)
        # to well above the fitting first, then down (the fittings lie out beyond the trays, away from the hook)
        yield from self.reach(2.0, {arm: (pick + np.array((0., 0., .12)), grip_R)}, {arm: gap_to_finger(FIT_OPEN_GAP)})
        yield from self.line(1.0, {arm: (pick - .05 * grip_R[:, 2], grip_R)})
        yield from self.line(1.0, {arm: (pick, grip_R)})
        yield from self.settle(arm, tries=2)
        yield from self.fingers(.6, {arm: gap_to_finger(FIT_CLOSE_GAP)})
        yield from self.hold(.3)
        self.note("fitting pinched", arm=arm, fitting=name, pads_mm=1000 * self.opening(arm))
        hp, hR = self.actual(arm)
        b, uu, _, _ = self.fit_frame(name)
        Rf = np.column_stack((e, uu, np.cross(e, uu)))
        self.fit_in_hand = (hR.T @ (b - hp), hR.T @ Rf)
        yield from self.line(1.6, {arm: (self.cmd[arm][0] + np.array((0., 0., .08)), self.cmd[arm][1])})
        yield from self.hold(.3)
        hp, hR = self.actual(arm)
        b, _, _, _ = self.fit_frame(name)
        slip = float(np.linalg.norm(hR.T @ (b - hp) - self.fit_in_hand[0]))
        self.note("fitting lifted", arm=arm, fitting=name, slip_mm=1000 * slip, pads_mm=1000 * self.opening(arm))
        if slip > .010:
            raise RuntimeError(f"{name} slipped in the {arm} hand by {1000 * slip:.0f} mm")

        def hand_for(A, phi):
            fb, fu = self.hang_fit_frame(side, A, phi)
            fb = fb + FIT_SIDE * FIT_DX * np.array((1., 0., 0.))
            Rf = np.column_stack((e, fu, np.cross(e, fu)))
            lp, lR = self.fit_in_hand
            hR = Rf @ lR.T
            return fb - hR @ lp, hR

        self.phase(f"hang {name}")
        (_, A0, ph0) = HANG_PATH[0]
        p0, R0 = hand_for(A0, ph0)
        # straight up, across to beside the horn and only then turned to the hanging attitude, in straight lines (a
        # joint-space move swung the fitting into its tray; turning it on the way across ran the arm to its limits)
        hp, hR = self.cmd[arm]
        mid = np.array((hp[0], math.copysign(CARRY_Y, hp[1]), CARRY_Z))     # in toward the bar, a little higher
        yield from self.line(1.0, {arm: (hp + np.array((0., 0., .04)), hR)})
        yield from self.line(2.0, {arm: (mid, hR)})
        yield from self.line(2.4, {arm: (p0, R0)})
        yield from self.settle(arm, tries=4, free=True)
        self.regrip_reading(name, arm, e, "fitting beside the horn")
        p0, R0 = hand_for(A0, ph0)
        yield from self.line(.6, {arm: (p0, R0)})
        yield from self.settle(arm, tries=3, free=True)
        self.regrip_reading(name, arm, e, "fitting at the horn")
        # the other hand takes hold of the hook
        self.start_cmd(steady_arm)
        sp, sR = self.steady_pose("L" if side == "R" else "R")
        sR = self.better_roll(steady_arm, sp, sR)
        yield from self.reach(2.0, {steady_arm: (sp - .04 * sR[:, 2], sR)}, {steady_arm: gap_to_finger(.040)})
        yield from self.line(.8, {steady_arm: (sp, sR)})
        yield from self.fingers(.5, {steady_arm: gap_to_finger(.012)})
        yield from self.hold(.3)
        self.note("hook steadied", arm=steady_arm, pads_mm=1000 * self.opening(steady_arm))
        path = HANG_PATH[1:] if past_latch else HANG_PATH[1:3]
        for wname, A, phi in path:
            if wname == "over the tip":           # last reading of the fitting in the hand before it touches the hook
                self.regrip_reading(name, arm, e, "fitting over the tip")
            yield from self.line(1.0, {arm: hand_for(A, phi)})
            centre, t, v, _ = self.horn_frame(side)
            apex = np.asarray(self.fitting(name)["apex"]) - centre
            self.note("hang waypoint", arm=arm, waypoint=wname, latch_deg=self.state["latch_deg"]["L" if side == "L" else "R"],
                      apex_wanted_mm=(1000 * A[0], 1000 * A[1]), apex_mm=(1000 * float(apex @ t), 1000 * float(apex @ v)),
                      apex_x_mm=1000 * float(apex[0]))
        yield from self.hold(.3)
        if not past_latch:              # the control: the fitting rests on its closed latch and is never pressed past
            self.note("fitting held on the latch", arm=arm, fitting=name)
            return
        yield from self.fingers(.4, {arm: gap_to_finger(FIT_RELEASE_GAP)})
        yield from self.hold(.8)
        f = self.fitting(name)
        O, Rh = self.hook()
        self.note("fitting released", arm=arm, fitting=name, apex_hook_frame_mm=1000 * (Rh.T @ (np.asarray(f["apex"]) - O))[1:],
                  latch_deg=self.state["latch_deg"]["L" if side == "L" else "R"])
        hp, hR = self.cmd[arm]
        yield from self.line(1.0, {arm: (hp - .06 * hR[:, 2], hR)})
        yield from self.fingers(.4, {steady_arm: gap_to_finger(.040)})
        sp, sR = self.cmd[steady_arm]
        yield from self.line(.8, {steady_arm: (sp - .05 * sR[:, 2], sR)})
        yield from self.ready(2.0, (arm, steady_arm))

    # -- the whole demonstration ----------------------------------------------------------------------------------
    def sequence(self):
        try:
            yield from self.hold(.5)
            for arm in F.ARMS:
                self.start_cmd(arm)
            v = self.variant
            if v == "dev_bottom":                  # development only: the bottom-edge guards alone
                yield from self.guards(top=False, bottom=True)
                v = "guards_only"
            elif v != "no_guards":
                yield from self.guards(top=True, bottom=(v != "top_only"))
                yield from self.ready(2.0)
            if v == "not_past_latch":
                yield from self.hang_fitting("fitB", "R", "right", "left", past_latch=False)
                self.phase("holding on the latch")
            elif v != "guards_only":
                yield from self.hang_fitting("fitB", "R", "right", "left")
                if v != "one_fitting":
                    yield from self.hang_fitting("fitA", "L", "left", "right")
            if v != "not_past_latch":
                self.phase("clear")
                yield from self.ready(1.5)
        except RuntimeError as error:
            self.note("demonstration stopped", error=str(error))
        self.done = True
        while True:
            yield self.emit()
