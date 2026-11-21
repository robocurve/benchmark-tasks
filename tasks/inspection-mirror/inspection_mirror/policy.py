"""Policy ``inspection_mirror_scripted``: a scripted demonstration that finds the leak with the mirror.

It reads only what a policy gets: the right wrist camera image and the joint angles, plus the fixed setup positions
(the gap, the cup, the mirror's rest). It never reads the leak's place from the simulator.

    1. right gripper: take the mirror by its grip, slide the head into the gap from its open end, stop it at each of
       the five places along the hidden face and count green pixels in the right wrist camera; the place with the
       most is the leak; go back to it and hold the mirror still on the trace; withdraw the mirror in one move and
       lay it back on its supports;
    2. left gripper (meanwhile): take the paint marker out of its cup and wait clear of the right arm; once the leak
       is found and the mirror is out, touch the tip onto the cover strip directly above it, then put the marker back.
It outputs ordinary 14-D joint targets through the stock IK; it never sets a part's pose. One privileged read: before
the put-back it reads the marker's pose from the simulator (twice, over the cup) to stand it upright over the bore.

``-P mode=demo`` (default). Checks (for the README):
``-P mode=guess``  skips the mirror and marks the middle place (what a blind policy does);
``-P mode=sweepguess``  sweeps the mirror past every place, ignores the image, withdraws and marks the middle place;
``-P mode=touch``  sweeps with the head 7 mm toward the hidden face (the hand 13 mm, less the 6 mm sag allowance), so
                   it rubs the face (the trace is smeared);
``-P mode=hover``  the demo, but the marker tip is aimed 3 mm above the strip instead of touching it (no mark);
``-P mode=edge``   the demo, but first the head's rim is pressed onto the hidden face's near edge at the gap's mouth,
                   its centre still outside the gap (a smear);
``-P mode=idle``   keeps both arms at the ready pose.
``-P aim_mm=<mm>`` shifts the mirror's path toward the face and the mark along the gap by that much (margin runs).
"""

from __future__ import annotations

import math

import numpy as np
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.packing import pack
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.scripted import YamScriptedPolicy

from . import design as d
from .motion import Arm, Move

HANDLE_SQUEEZE = -0.0020  # 4 mm finger gap on the 12.7 mm grip: a firm hold (the head end is heavy)
MARKER_SQUEEZE = -0.0040  # 8 mm finger gap on the 16 mm barrel
HANDLE_IN = 0.070  # the hand point 70 mm in from the grip's free end, 10 mm short of the balance point
HANDLE_TILT = math.radians(25.0)  # approach pitched down 25 degrees: the wrist camera then looks level along the tool
# The marker stands tip down in its cup. The left hand takes it from the side at its middle (balanced, so it does
# not pivot in the fingers), approaching along -y, and keeps that hold throughout: no turn in the hand.
R_DAB = np.column_stack([[-1.0, 0, 0], [0, 0, -1.0], [0, -1.0, 0]])  # approach along -y, TCP y down
P_TIP = np.array([0.0, 0.071 + 0.008, 0.007])  # the tip in the TCP frame (axis 7 mm past the TCP: pads straddle it)
LOOK_S = 1.2  # hold at each place before reading the image
HOVER = 0.003  # mode=hover: the tip aimed 3 mm above the strip (a 1.5 mm aim dipped onto it: the arm overshoots ~1.4 mm)
EDGE_SHIFT = 0.016  # mode=edge: the head 16 mm toward the face at the mouth (its rim reaches the face's near edge)
CONFIRM_S = 1.5  # back at the place found: hold the mirror on the trace before withdrawing (the view the mark uses)
SAG_BIAS = 0.006  # the hanging head sagged 3 to 6 mm toward the face (tool rolls slightly in the hand; most at the deepest place); aim 6 mm wallward
HEAD_LIFT = 0.008  # the tool pitches nose-down in the hand: the head hung 13 to 18 mm below its nominal height
# (measured 2026-11-17), which put the top of the trace past the glass's edge; aim the hand 8 mm higher
MARK_TRIM = 0.004  # the marks landed 4 mm short at every place (fixed in-hand offset of this grasp, measured 2026-11-16)
DAB_PRESS = 0.001  # the tip aimed 1 mm into the strip (2 mm tipped the marker up to 1.5 degrees in the fingers)
CUP_HOVER = 0.060  # the hand over the cup with the tip 8 mm above its rim: the re-measure is made here
RELEASE_UP = 0.004  # let go with the tip 4 mm above the cup's floor: the barrel is 48 mm down the bore by then
CAMERA = "right_cam"
READ_COL = 148  # count green only right of this image column: there the mirror shows the face straight across from the
# head; a trace at the neighbouring place shows nearer the mirror's near edge (left of it), measured 2026-11-16


def handle_rotation() -> np.ndarray:
    """TCP rotation: approach (TCP z) along +x pitched down HANDLE_TILT, fingers closing (TCP x) along world +y."""
    z = np.array([math.cos(HANDLE_TILT), 0.0, -math.sin(HANDLE_TILT)])
    x = np.array([0.0, 1.0, 0.0])
    return np.column_stack([x, np.cross(z, x), z])


def green_pixels(img) -> int:
    """Count bright green pixels (the coolant dye) in an RGB image."""
    a = np.asarray(img, dtype=np.int32)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    return int(np.count_nonzero((g > 90) & (g > r + 40) & (g > b + 40)))


class InspectionMirrorScriptedPolicy(YamScriptedPolicy):
    def __init__(self, mode: str = "demo", aim_mm: float = 0.0, **kwargs):
        super().__init__(**kwargs)
        from dataclasses import replace

        try:
            self.info = replace(self.info, name="inspection_mirror_scripted")
        except Exception:  # noqa: BLE001 - the name is cosmetic
            pass
        self.mode = str(mode)
        self.aim = float(aim_mm) / 1000.0
        assert self.mode in ("demo", "guess", "sweepguess", "touch", "idle", "peek", "hover", "edge"), self.mode

    def reset(self, scene):
        self._built = False
        self._end = None
        self.seen = [0] * len(d.LEAK_X)
        self.found = None
        self.mark_pushed = False

    # -- plan -------------------------------------------------------------------------------------------------
    @staticmethod
    def _c(p, rot):
        return lambda sim, p=np.asarray(p, float), r=rot: (p, r)

    def _build(self):
        self._built = True
        self.arms = {a: Arm(self._bases[a]) for a in ("left", "right")}
        self.park = {a: ik.tcp_pose(np.array(READY, float), np.asarray(self._bases[a], float)) for a in self.arms}
        if self.mode == "idle":
            return
        if self.mode == "peek":
            self.arms["right"].push(*self.peek_moves())
            self.peek_seen = {}
            return
        if self.mode == "sweepguess":  # the sweep with no reading and no return, then out
            self.arms["right"].push(*self.mirror_moves())
        elif self.mode != "guess":
            self.arms["right"].push(*self.sweep_moves())  # the return and withdrawal follow the reading
        self.arms["left"].push(*self.marker_pick_moves())
        if self.mode in ("guess", "sweepguess"):
            self.found = len(d.LEAK_X) // 2

    def _mirror_geometry(self):
        rot = handle_rotation()
        end = d.TOOL_REST[0] - d.GRIP[0] / 2  # the grip's free end, toward the robot
        grip = np.array([end + HANDLE_IN, d.TOOL_REST[1], d.TOP + d.TOOL_REST[2]])
        reach = d.HEAD_REACH + d.GRIP[0] / 2 - HANDLE_IN  # hand point to head centre, along +x
        off = 0.013 if self.mode == "touch" else self.aim  # toward the hidden face (-y)
        gy = d.FACE_Y + d.GAP / 2 + SAG_BIAS - off
        gz = d.TOP + d.LEAK_Z + d.HEAD_DROP + HEAD_LIFT  # the head hangs HEAD_DROP below the hand (and sags)
        hand = lambda head_x: np.array([head_x - reach, gy, gz])  # noqa: E731
        return rot, grip, hand, d.GAP_X[0] - 0.040  # last: the head 40 mm short of the gap's open end

    @staticmethod
    def _aside(hand, grip, outside):
        """A hand pose in line with the supports with the head 10 mm short of the gap's open end: the mirror goes
        between the supports and the line-up through it, so its head never swings across the housing's near corner
        (a straight move did; 12 mm clearance this way, checked offline)."""
        return np.array([hand(outside)[0] + 0.050, grip[1], grip[2] + 0.040])

    def sweep_moves(self):
        """Take the grip from above near its balance point (fingers across it), slide the head into the gap and stop it
        at each place in turn (the image is read at each stop)."""
        rot, grip, hand, outside = self._mirror_geometry()
        c = self._c
        m = [Move(2.5, c(grip + [0, 0, 0.050], rot), FINGER_OPEN, "mirror approach"),
             Move(1.0, c(grip + [0, 0, 0.050], rot), FINGER_OPEN, "mirror settle above the grip"),
             Move(1.5, c(grip, rot), FINGER_OPEN, "mirror down onto the grip"),
             Move(1.0, c(grip, rot), HANDLE_SQUEEZE, "mirror grip"),
             Move(2.0, c(grip + [0, 0, 0.040], rot), HANDLE_SQUEEZE, "mirror lift"),
             Move(2.5, c(self._aside(hand, grip, outside), rot), HANDLE_SQUEEZE, "mirror back, clear of the housing"),
             Move(2.5, c(hand(outside), rot), HANDLE_SQUEEZE, "mirror lined up with the gap"),
             Move(1.5, c(hand(outside), rot), HANDLE_SQUEEZE, "mirror settle"),
             *self.edge_moves(hand, rot),
             Move(4.0, c(hand(d.GAP_X[0] + 0.005), rot), HANDLE_SQUEEZE, "mirror slowly into the gap"),
             Move(1.0, c(hand(d.GAP_X[0] + 0.005), rot), HANDLE_SQUEEZE, "mirror settle in the gap")]
        for k, x in enumerate(d.LEAK_X):
            m += [Move(1.5, c(hand(x), rot), HANDLE_SQUEEZE, f"mirror to {k}"),
                  Move(LOOK_S, c(hand(x), rot), HANDLE_SQUEEZE, f"look {k}")]
        return m

    def edge_moves(self, hand, rot):
        """``mode=edge``: before going in, press the head's rim onto the hidden face's near edge at the gap's mouth,
        its centre still outside the gap, then back out to the line-up pose."""
        if self.mode != "edge":
            return []
        c = self._c
        x = d.GAP_X[0] - 0.002
        p_in = hand(x) + [0, -EDGE_SHIFT, 0]
        return [Move(2.0, c(hand(x), rot), HANDLE_SQUEEZE, "edge: head centre at the mouth"),
                Move(1.5, c(p_in, rot), HANDLE_SQUEEZE, "edge: rim onto the face's near edge"),
                Move(1.0, c(p_in, rot), HANDLE_SQUEEZE, "edge: hold"),
                Move(1.5, c(hand(x - 0.038), rot), HANDLE_SQUEEZE, "edge: back to the line-up")]

    def withdraw_moves(self, back_to=None):
        """Optionally go back to the place found and hold the mirror on the trace (the view the mark is made from),
        then withdraw it in one move and lay it back on its supports."""
        rot, grip, hand, outside = self._mirror_geometry()
        c = self._c
        m = []
        if back_to is not None:
            x = d.LEAK_X[back_to]
            m += [Move(2.5, c(hand(x), rot), HANDLE_SQUEEZE, f"mirror back to {back_to}"),
                  Move(CONFIRM_S, c(hand(x), rot), HANDLE_SQUEEZE, f"confirm {back_to}")]
        m += [Move(3.0, c(hand(outside), rot), HANDLE_SQUEEZE, "mirror out"),
              Move(2.5, c(self._aside(hand, grip, outside), rot), HANDLE_SQUEEZE, "mirror aside, clear of the housing"),
              Move(3.0, c(grip + [0, 0, 0.040], rot), HANDLE_SQUEEZE, "mirror over its supports"),
              Move(1.5, c(grip + [0, 0, 0.002], rot), HANDLE_SQUEEZE, "mirror down"),
              Move(0.8, c(grip + [0, 0, 0.002], rot), FINGER_OPEN, "mirror let go"),
              Move(1.2, c(grip + [0, 0, 0.050], rot), FINGER_OPEN, "mirror withdraw"),
              Move(2.0, lambda sim: self.park["right"], FINGER_OPEN, "right park")]
        return m

    def mirror_moves(self):
        """The whole mirror plan with no reading (the sweep, then out): for the reach check and sweep-then-guess."""
        return self.sweep_moves() + self.withdraw_moves()

    def peek_moves(self):
        """Control (no mirror): the empty right hand, fingers closed, at the gap's entrance looking in from the
        middle and from either side, turned up to 35 degrees toward the hidden face or away from it."""
        base = handle_rotation()
        gz = d.TOP + d.LEAK_Z + d.HEAD_DROP
        m = []
        for x in (d.GAP_X[0] - 0.060, d.GAP_X[0] - 0.030, d.GAP_X[0] - 0.015):
            for y in (-0.020, 0.0, 0.020):
                for yaw in (0, -20, -35, -45, 20):
                    a = math.radians(yaw)
                    rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
                    m += [Move(1.2, self._c([x, y, gz], rz @ base), 0.0, f"peek move {x:.3f} {y:+.3f} {yaw:+d}"),
                          Move(1.0, self._c([x, y, gz], rz @ base), 0.0, f"peek at {x:.3f} {y:+.3f} {yaw:+d}")]
        return m + [Move(2.0, lambda sim: self.park["right"], FINGER_OPEN, "right park")]

    @staticmethod
    def _marker_tcp(tip_xyz, rot):
        """TCP position that puts the marker's tip at ``tip_xyz`` for a hand rotation (the grasp is fixed)."""
        return np.asarray(tip_xyz, float) - rot @ P_TIP

    def _pick_tcp(self):
        x, y = (float(v) for v in d.HOLDER_XY)
        return self._marker_tcp([x, y, d.TOP + d.HOLDER_STAND + d.HOLDER_FLOOR], R_DAB)  # the tip on the cup's floor

    def marker_pick_moves(self):
        grasp = self._pick_tcp()
        wait = self._marker_tcp([0.24, 0.12, d.TOP + d.CAP_TOP + 0.030], R_DAB)  # clear of the right arm
        c = self._c
        return [Move(2.5, c(grasp + [0, 0, 0.090], R_DAB), FINGER_OPEN, "marker approach, above it"),
                Move(1.0, c(grasp + [0, 0, 0.090], R_DAB), FINGER_OPEN, "marker settle"),
                Move(1.5, c(grasp, R_DAB), FINGER_OPEN, "marker hand down around it"),
                Move(1.0, c(grasp, R_DAB), MARKER_SQUEEZE, "marker grip"),
                Move(2.0, c(grasp + [0, 0, 0.075], R_DAB), MARKER_SQUEEZE, "marker out of the cup"),
                Move(2.5, c(wait, R_DAB), MARKER_SQUEEZE, "marker wait")]

    def mark_moves(self, place: int):
        x = d.LEAK_X[place] + MARK_TRIM + (self.aim if self.mode != "touch" else 0.0)
        y = d.FACE_Y + d.GAP / 2
        top = d.TOP + d.CAP_TOP
        above = self._marker_tcp([x, y, top + 0.030], R_DAB)
        dab = self._marker_tcp([x, y, top - DAB_PRESS], R_DAB)  # just into the strip: compliant arm, soft felt tip
        if self.mode == "hover":
            dab = self._marker_tcp([x, y, top + HOVER], R_DAB)  # the tip held just above the strip: no contact
        grasp = self._pick_tcp()
        c = self._c
        self._cup_hand = (grasp + [0, 0, CUP_HOVER], R_DAB)  # nominal hand pose over the cup; the re-measure corrects it
        return [Move(2.5, c(above, R_DAB), MARKER_SQUEEZE, f"marker above place {place}"),
                Move(1.0, c(above, R_DAB), MARKER_SQUEEZE, "marker settle"),
                Move(2.0, c(dab, R_DAB), MARKER_SQUEEZE, "marker touch"),
                Move(0.6, c(dab, R_DAB), MARKER_SQUEEZE, "marker hold"),
                Move(1.2, c(above + [0, 0, 0.010], R_DAB), MARKER_SQUEEZE, "marker lift"),
                Move(3.0, c(grasp + [0, 0, CUP_HOVER], R_DAB), MARKER_SQUEEZE, "marker over its cup"),
                Move(1.0, None, MARKER_SQUEEZE, "marker settle over its cup"),
                Move(1.2, self._cup_upright, MARKER_SQUEEZE, "marker upright over the bore"),
                Move(1.0, None, MARKER_SQUEEZE, "marker settle upright"),
                Move(0.8, self._cup_upright, MARKER_SQUEEZE, "marker upright again"),
                Move(0.8, None, MARKER_SQUEEZE, "marker settle upright again"),
                Move(3.0, self._cup_down, MARKER_SQUEEZE, "marker straight down into the cup"),
                Move(0.5, None, MARKER_SQUEEZE, "marker settle in the cup"),
                Move(0.8, None, FINGER_OPEN, "marker let go"),
                Move(1.5, lambda sim: (self._cup_hand[0] + [0, 0, 0.090], self._cup_hand[1]), FINGER_OPEN,
                     "marker withdraw, up"),
                Move(2.0, lambda sim: self.park["left"], FINGER_OPEN, "left park")]

    # -- put-back re-measure (privileged: the marker's pose from the simulator) ----------------------------------
    def _cup_upright(self, sim):
        """Read the marker's pose and turn and shift the hand so the barrel stands upright with its axis on the bore's
        centre line, at the height it has now. The grasp holds the marker 2 to 4 degrees off upright (the dab adds
        up to 1.5), which puts the barrel's bottom 3 to 4 mm off centre; the bore leaves 1 mm round the barrel."""
        p_h, r_h = self._cup_hand
        m = marker_pose()
        if m is None:  # offline (reach check): the nominal pose
            return p_h, r_h
        p_m, axis = m[:3], d.rot(m[3:])[:, 2]
        r_new = rotation_between(axis, np.array([0.0, 0.0, 1.0])) @ r_h
        g = r_h.T @ (p_m - p_h)  # the marker's centre in the commanded hand frame (holds the arm's tracking offset)
        want = np.array([d.HOLDER_XY[0], d.HOLDER_XY[1], p_m[2]])
        self._cup_hand = (want - r_new @ g, r_new)
        print(f"[inspect-demo] put-back re-measure: marker {math.degrees(math.acos(min(1.0, axis[2]))):.1f} deg off "
              f"upright, axis {1000 * (p_m[0] - d.HOLDER_XY[0]):+.1f}/{1000 * (p_m[1] - d.HOLDER_XY[1]):+.1f} mm "
              f"off the bore", flush=True)
        return self._cup_hand

    def _cup_down(self, sim):
        """Straight down until the tip is RELEASE_UP above the cup's floor (measured), keeping the upright hold."""
        p_h, r_h = self._cup_hand
        m = marker_pose()
        seated = d.TOP + d.HOLDER_STAND + d.HOLDER_FLOOR + d.MARKER_TIP[0] + d.MARKER[0] / 2  # centre, tip on floor
        dz = -(CUP_HOVER - RELEASE_UP) if m is None else (seated + RELEASE_UP) - m[2]
        self._cup_hand = (p_h + [0, 0, dz], r_h)
        return self._cup_hand

    def waypoints(self):
        """Every (arm, label, position, rotation) target of the plan, for the offline reach check."""
        self.park = {a: ik.tcp_pose(np.array(READY, float), np.asarray(self._bases[a], float))
                     for a in ("left", "right")}
        out = [("right", m.label, *m.target(None)) for m in self.mirror_moves() if m.label != "right park"]
        out += [("left", m.label, *m.target(None)) for m in self.marker_pick_moves()]
        for k in range(len(d.LEAK_X)):
            out += [("left", m.label, *m.target(None)) for m in self.mark_moves(k) if m.label != "left park"]
        return out

    # -- control ----------------------------------------------------------------------------------------------
    def act(self, observation):
        t = float(observation.state_time) + self._dt
        if not self._built:
            self._build()
        if self.mode == "idle":
            out = [np.r_[READY, 1.0], np.r_[READY, 1.0]]
        else:
            right = self.arms["right"]
            lab = right.label
            if self.mode == "peek":
                if lab.startswith("peek at") and t - right.t0 > 0.6:
                    for cam, img in observation.images.items():
                        g = dim_green_pixels(img)
                        self.peek_seen[cam] = max(self.peek_seen.get(cam, 0), g)
                        self.peek_seen[(cam, lab)] = g
                if self._end is None and right.idle and t > 5:
                    self._end = t + 1.0
                    per_cam = {k: v for k, v in self.peek_seen.items() if not isinstance(k, tuple)}
                    worst = sorted(((v, k) for k, v in self.peek_seen.items() if isinstance(k, tuple)), reverse=True)
                    print(f"[inspect-demo] peek max green per camera {per_cam}; top poses {worst[:4]}", flush=True)
                out = [np.r_[READY, 1.0], right.act(None, t)]
                meta = {"request_stop": True} if self._end is not None and t > self._end else {}
                return ActionChunk(actions=[Action(data=pack(*out), meta=meta)], control_hz=1.0 / self._dt)
            if lab.startswith("look ") and t - right.t0 > LOOK_S * 0.6 and CAMERA in observation.images:
                k = int(lab.split()[1])
                self.seen[k] = max(self.seen[k], green_pixels(observation.images[CAMERA][:, READ_COL:]))
            if self.found is None and right.idle:  # the sweep is over: go back to the place with the most green
                self.found = int(np.argmax(self.seen))
                print(f"[inspect-demo] t={t:6.2f} green pixels per place {self.seen} -> place {self.found}",
                      flush=True)
                right.push(*self.withdraw_moves(back_to=self.found))
            left = self.arms["left"]
            clear = self.mode == "guess" or lab in ("mirror over its supports", "mirror down", "mirror let go",
                                                     "mirror withdraw", "right park") \
                or (right.idle and self.found is not None)  # idle after its plan (not between sweep and return)
            if self.found is not None and not self.mark_pushed and left.idle and clear:
                self.mark_pushed = True
                left.push(*self.mark_moves(self.found))
            out = [self.arms[a].act(None, t) for a in ("left", "right")]
            if self._end is None and self.mark_pushed and all(self.arms[a].idle for a in self.arms):
                self._end = t + 3.0
                print(f"[inspect-demo] t={t:6.2f} procedure finished", flush=True)
        meta = {"request_stop": True} if self._end is not None and t > self._end else {}
        return ActionChunk(actions=[Action(data=pack(*out), meta=meta)], control_hz=1.0 / self._dt)


def marker_pose():
    """The paint marker's pose (xyz in the env frame + quat xyzw) from the live simulator; None offline."""
    try:
        from . import mechanism
    except Exception:  # noqa: BLE001 - no simulator here (reach check, tests)
        return None
    if mechanism.ACTIVE is None:
        return None
    return np.asarray(mechanism.ACTIVE.poses()["marker"], float)


def rotation_between(a, b) -> np.ndarray:
    """The smallest rotation taking unit vector ``a`` onto unit vector ``b``."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    b = np.asarray(b, float) / np.linalg.norm(b)
    v, c = np.cross(a, b), float(np.dot(a, b))
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        return np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k * ((1 - c) / s ** 2)


def dim_green_pixels(img) -> int:
    """A loose count for the no-mirror control: any pixel clearly greener than it is red or blue, however dim."""
    a = np.asarray(img, dtype=np.int32)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    return int(np.count_nonzero((g > 30) & (g > r + 12) & (g > b + 12)))
