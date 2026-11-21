"""On the station table: a folding steel step stool (Vive Folding Step Stool, LVA2068BLK) lies upside down, its black
platform face down and its two legs folded flat against the platform's underside. Open its legs the way its maker
shows: each leg swung up and out until its lock clicks.

Each leg is a steel U-frame (two posts and a crossbar at the foot) on a hinge at one end of the platform. It has its own
spring lock, which catches only once the leg is opened a little past square to the platform (98 degrees from folded;
its stop is at 100, the leg splayed 10 degrees). Short of the lock, the lock's spring pushes the leg back, so a leg can
look open and still fold. With both hands off the stool, each leg is pushed back towards folded (a stand-in for the
stool being set on its feet and stood on): a locked leg holds; an unlocked one folds.

Run it:  inspect-robots run --embodiment yam_arena -E task=folding_step_stool.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fittingkit  # noqa: E402
from fittingkit import Hinge, body, box, cyl  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

fittingkit.install()
SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001
INCH = 0.0254
X_AXIS, Y_AXIS, Z_AXIS = np.eye(3)

PLATFORM_BLACK = (0.10, 0.10, 0.11)
MAT_GREY = (0.32, 0.33, 0.35)  # the textured mat, drawn a shade lighter so the platform's outline shows
FRAME_BLACK = (0.14, 0.14, 0.15)
FOOT_GREY = (0.45, 0.45, 0.47)
LATCH_RED = (0.80, 0.15, 0.12)  # the latch buttons on the legs' inside, drawn red so they show on camera


def rot_x(angle: float) -> np.ndarray:
    return axis_angle_matrix(X_AXIS, angle)


# -- the stool: Vive Folding Step Stool, product number LVA2068BLK ------------------------------------------------------
# Vive's product page (read 2026-11-21): platform 15 x 10.5 in (the page's text says 15 x 10 in; its dimensioned photo
# says 10.5), 6.5 in off the ground (the photo: 6 in), 4 lb, coated steel frame, textured mat, angled legs with
# "reinforced locking mechanisms", legs folded "by pushing in the locking latch on the inside of each leg and moving them
# inward", 250 lb capacity, UPC 810041984739. Its "Step 1-4" photos open it upside down: each leg swung out until it
# locks, then the stool turned over. The same stool is sold as VonHaus 3500099 (37 x 26 cm tread, 17 cm, 1.8 kg, spring
# locks that engage on opening). Where the page gives no size (tube, hinge, latch), the numbers are estimates.
STOOL_X = 0.30
TOP_L, TOP_D = 15 * INCH, 10.5 * INCH  # the platform: along y (the legs at its -y and +y ends), along x
TOP_T = 0.025  # the platform's thickness, mat included [estimate]
STAND_TOP = 6.5 * INCH  # the platform's top above the floor, open
HINGE_IN = 0.020  # each leg's hinge axis, this far in from the platform's end [estimate]
HINGE_Y = TOP_L / 2 - HINGE_IN
KNUCKLE_R = 8.0 * MM  # the hinge's knuckle round its axis; the axis is this far under the platform [estimate]
TUBE = 0.019  # the legs' square steel tube [estimate]
LEG_W = TOP_D - 0.03  # a leg's width across the stool (x), post to post [estimate]
BAR_R = 9.0 * MM  # the crossbar at a leg's foot, a round tube [estimate]
OPEN_DEG, LATCH_DEG = 100.0, 98.0  # a leg's stop (10 degrees splayed) and where its lock catches [estimate]
# The leg's length, hinge axis to the middle of its crossbar, from the platform's height when standing open:
LEG_L = (STAND_TOP - TOP_T - KNUCKLE_R - BAR_R) / math.cos(math.radians(OPEN_DEG - 90.0))
TOP_MASS, LEG_MASS = 1.20, 0.30  # 1.8 kg in all (VonHaus); the split is an estimate
SPRING_FROM_DEG = 80.0  # from here to the lock, the lock's spring pushes the leg back [estimate]
PUSH_BACK = (0.10, 0.165)  # that push back (N m) at 80 and just short of 98 degrees [estimate]
HOLD_TORQUE = 2.0  # N m: a locked leg is held open against its stop [estimate: well above the check below]
CHECK_TORQUE = 0.6  # N m folding each leg once both hands are off: about 5 N on its crossbar [estimate]
# WHY a rule, not left to the contacts: once checked, a leg that isn't locked is folded by this torque; a locked leg is
# held by HOLD_TORQUE, so it stays at its stop.

# Upside down: the platform's mat face down on the table, the legs folded on its underside (now facing up). Frames: the
# platform's frame has z out of its mat (so it points down here), x across the stool, y along it.
UPSIDE_DOWN = rot_x(math.pi)
TOP_Z = TOP + TOP_T / 2
TOP_AT = np.array([STOOL_X, 0.0, TOP_Z])
FLIP_Q = (1.0, 0.0, 0.0, 0.0)  # xyzw: half a turn about x


def _top_shapes() -> list:
    """In the platform's frame (origin at its middle, z out of the mat): the platform, the hinges' ears under it at
    each end, the mat drawn on its top face."""
    hz = -(TOP_T / 2 + KNUCKLE_R)
    solid = [box((0.0, 0.0, 0.0), (TOP_D, TOP_L, TOP_T), PLATFORM_BLACK)]
    for s in (-1.0, 1.0):
        for x in (-(LEG_W / 2 + 0.012), LEG_W / 2 + 0.012):  # an ear outside each post
            solid.append(box((x, s * HINGE_Y, (hz - TOP_T / 2) / 2), (0.006, 2 * KNUCKLE_R, -TOP_T / 2 - hz),
                             FRAME_BLACK))
    seen = [box((0.0, 0.0, TOP_T / 2 + 0.0005), (TOP_D - 0.02, TOP_L - 0.02, 0.001), MAT_GREY)]
    return solid + seen


_TS = _top_shapes()
STOOL_TOP = body("stool_top", pos=tuple(TOP_AT), mass=TOP_MASS, friction=0.8, shapes=_TS,
                 visual_only=(len(_TS) - 1,), color=PLATFORM_BLACK, quat=FLIP_Q)

INWARD = {"leg_a": 1.0, "leg_b": -1.0}  # the folded leg's direction (platform y) from its hinge
SIDE = {"leg_a": -1.0, "leg_b": 1.0}  # which end of the platform


def _leg_shapes(s: float) -> list:
    """In the leg's frame (origin on its hinge axis, axes those of the platform with the leg folded); ``s`` is its
    inward direction (y). Two posts along the leg, the knuckle round the hinge, the crossbar at the foot, the latch
    buttons on the posts' inside."""
    post_len = LEG_L - BAR_R
    out = [cyl((0.0, 0.0, 0.0), KNUCKLE_R, LEG_W + 2 * TUBE, FRAME_BLACK, axis="X")]
    for x in (-LEG_W / 2, LEG_W / 2):
        out.append(box((x, s * post_len / 2, 0.0), (TUBE, post_len, TUBE), FRAME_BLACK))
    out.append(cyl((0.0, s * LEG_L, 0.0), BAR_R, LEG_W + TUBE, FOOT_GREY, axis="X"))
    for x in (-LEG_W / 2 + TUBE / 2 + 0.003, LEG_W / 2 - TUBE / 2 - 0.003):
        out.append(box((x, s * 0.03, 0.0), (0.006, 0.012, 0.012), LATCH_RED))
    return out


def hinge_local(leg: str) -> np.ndarray:
    """The leg's hinge axis point in the platform's frame."""
    return np.array([0.0, SIDE[leg] * HINGE_Y, -(TOP_T / 2 + KNUCKLE_R)])


LEGS = {}
for _leg in ("leg_a", "leg_b"):
    _at = TOP_AT + UPSIDE_DOWN @ hinge_local(_leg)
    _shapes = _leg_shapes(INWARD[_leg])
    LEGS[_leg] = body(_leg, pos=tuple(_at), mass=LEG_MASS, friction=1.0, shapes=_shapes,
                      visual_only=(len(_shapes) - 2, len(_shapes) - 1),
                      com=(0.0, INWARD[_leg] * LEG_L * 0.55, 0.0), color=FRAME_BLACK, quat=FLIP_Q)
    # the -y leg opens turning about -x, the +y leg about +x (a PhysX revolute angle is the child's turn about +x)
    _lo, _hi = (-math.radians(OPEN_DEG), 0.0) if _leg == "leg_a" else (0.0, math.radians(OPEN_DEG))
    fittingkit.add_hinge(Hinge("stool_top", _leg, anchor=tuple(hinge_local(_leg)), axis=(1.0, 0.0, 0.0), damping=0.002,
                               lower=_lo, upper=_hi))

OBJECTS = [STOOL_TOP, LEGS["leg_a"], LEGS["leg_b"]]


# -- the locks ------------------------------------------------------------------------------------------------------
def leg_angle_deg(top_rot: np.ndarray, leg_rot: np.ndarray, leg: str) -> float:
    """How far (degrees) the leg is open from folded: 0 folded flat against the platform, 90 square to it, 100 at its
    stop (the same whichever way up the stool is)."""
    s = INWARD[leg]
    d = top_rot.T @ leg_rot @ np.array([0.0, s, 0.0])  # the leg's direction, hinge to foot, in the platform's frame
    return float(np.degrees(math.atan2(-d[2], s * d[1])))


def opening_axis(top_rot: np.ndarray, leg: str) -> np.ndarray:
    """The world axis about which the leg turns to open (right hand)."""
    return top_rot @ np.array([-INWARD[leg], 0.0, 0.0])


def push_back_nm(opening_deg: float) -> float:
    """The lock's spring folding the leg back while it rides the lock (from 80 degrees to the catch at 98)."""
    if not SPRING_FROM_DEG <= opening_deg < LATCH_DEG:
        return 0.0
    f = (opening_deg - SPRING_FROM_DEG) / (LATCH_DEG - SPRING_FROM_DEG)
    return PUSH_BACK[0] + f * (PUSH_BACK[1] - PUSH_BACK[0])


class Locks:
    """Runs before every physics substep (240 Hz): each leg's spring lock, from the legs' and the platform's poses. A
    leg short of its lock is pushed back by the lock's spring from 80 degrees on; at 98 degrees the lock catches, and
    from then on the leg is held open against its stop. Once the stool lies still with both hands off it, each leg is
    pushed back towards folded: an unlocked leg folds."""

    def __init__(self):
        self.origin = np.zeros(3)
        self.log = os.environ.get("STOOL_LOG")
        self.trace = os.environ.get("STOOL_TRACE")
        self.clear()

    @property
    def bodies(self) -> list[str]:
        return ["stool_top", "leg_a", "leg_b"]

    def clear(self) -> None:
        self._n = 0
        self.locked = {"leg_a": False, "leg_b": False}
        self.angle = {"leg_a": 0.0, "leg_b": 0.0}
        self.top_pos = TOP_AT.copy()
        self.top_rot = UPSIDE_DOWN.copy()
        self.hands_off = True  # updated by the 10 Hz check below; offline, no hands
        self.checked = False
        self.gave_way = {"leg_a": False, "leg_b": False}
        self.ever_held = False  # a hand has been on the stool at some point (the check waits for that)

    def update(self, frames: dict, acc: dict) -> None:
        top = frames.get("stool_top")
        if top is None:
            return
        self._n += 1
        self.top_pos, self.top_rot = top.pos - self.origin, top.rot
        for leg in ("leg_a", "leg_b"):
            f = frames.get(leg)
            if f is None:
                continue
            opening = leg_angle_deg(top.rot, f.rot, leg)
            self.angle[leg] = opening
            if not self.locked[leg] and opening >= LATCH_DEG:
                self.locked[leg] = True
                self._note(f"{self._n / 240:7.2f}s {leg} locked at {opening:5.1f} deg")
            if self.checked and not self.locked[leg] and not self.gave_way[leg]:
                self.gave_way[leg] = True
                self._note(f"{self._n / 240:7.2f}s {leg} folds under the check at {opening:5.1f} deg")
            if self.locked[leg]:
                torque = HOLD_TORQUE
            elif self.checked:  # only while the check is on: a hand may come back and open the leg again
                torque = -CHECK_TORQUE
            elif SPRING_FROM_DEG <= opening < LATCH_DEG:
                torque = -push_back_nm(opening)
            else:
                continue
            axis = opening_axis(top.rot, leg)
            fittingkit._torque(acc, leg, torque * axis)
            fittingkit._torque(acc, "stool_top", -torque * axis)
        if self._n % 24 == 0 and fittingkit._ENV["env"] is not None:
            self.hands_off = hands_clear_of(self.top_pos, self.top_rot, frames, self.origin)
            if not self.hands_off:
                self.ever_held = True
        was = self.checked
        self.checked = self.ever_held and self.hands_off and lying_still(self.top_pos, self.top_rot, top)
        if self.checked != was:
            self._note(f"{self._n / 240:7.2f}s check {'on' if self.checked else 'off'}")
        if self.trace and self._n % 24 == 0:
            self._write_trace(top)

    def _write_trace(self, top) -> None:
        """Debug only (STOOL_TRACE=<file>): every 0.1 s, the platform's pose, the legs' angles and both TCPs (mm)."""
        p = 1000 * (top.pos - self.origin)
        roll = math.degrees(math.atan2(top.rot[2, 1], top.rot[2, 2]))
        yaw = math.degrees(math.atan2(top.rot[1, 0], top.rot[0, 0]))
        tcps = ""
        environment = fittingkit._ENV["env"]
        if environment is not None:
            from yam_arena.runtime import tcp_world
            scene = environment.unwrapped.scene
            for name in fittingkit.ROBOTS:
                q = 1000 * (tcp_world(scene[name]) - self.origin)
                tcps += f" {name[0]} tcp ({q[0]:6.1f},{q[1]:6.1f},{q[2] - 1000 * TOP:6.1f})"
        with open(self.trace, "a") as fh:
            fh.write(f"{self._n / 240:6.2f}s top ({p[0]:6.1f},{p[1]:6.1f},{p[2] - 1000 * TOP:6.1f}) roll {roll:6.1f} yaw {yaw:5.1f} | "
                     f"a {self.angle['leg_a']:6.1f} {int(self.locked['leg_a'])} b {self.angle['leg_b']:6.1f} "
                     f"{int(self.locked['leg_b'])} | off {int(self.hands_off)} check {int(self.checked)} |{tcps}\n")

    def _note(self, line: str) -> None:
        if self.log:
            with open(self.log, "a") as fh:
                fh.write(line + "\n")


LEVEL_DEG = 5.0
CLEAR = 0.02


def lying_still(top_pos, top_rot, frame=None) -> bool:
    """Upside down and level (within 5 degrees), its platform on the table (within 1 cm), and not moving."""
    down = float(np.clip(-top_rot[2, 2], -1.0, 1.0))
    on_table = abs(float(top_pos[2]) - TOP_Z) < 0.01
    still = frame is None or (np.linalg.norm(frame.vel) < 0.02 and np.linalg.norm(frame.spin) < 0.2)
    return math.degrees(math.acos(down)) < LEVEL_DEG and on_table and still


def _part_boxes(top_pos, top_rot, frames, origin):
    """(centre, rotation, half sizes) of the platform and of each leg, in table coordinates."""
    out = [(top_pos, top_rot, np.array([TOP_D, TOP_L, TOP_T]) / 2)]
    for leg in ("leg_a", "leg_b"):
        f = frames.get(leg)
        if f is None:
            continue
        s = INWARD[leg]
        out.append((f.pos - origin + f.rot @ np.array([0.0, s * LEG_L / 2, 0.0]), f.rot,
                    np.array([LEG_W + TUBE, LEG_L + BAR_R, 2 * BAR_R]) / 2))
    return out


def hands_clear_of(top_pos, top_rot, frames, origin) -> bool:
    """No gripper point within 2 cm of the platform or a leg; unknown (offline) counts as clear."""
    pts = fittingkit.finger_points()
    if len(pts) == 0:
        return True
    pts = pts - origin
    for c, r, half in _part_boxes(top_pos, top_rot, frames, origin):
        local = np.abs((pts - c) @ r) - half
        if (np.max(local, axis=1) < CLEAR).any():
            return False
    return True


MODEL = fittingkit.add_model(Locks())

# -- scoring -------------------------------------------------------------------------------------------------------
COMPLETE_SCORE = 0.9


def both_locked() -> bool:
    return MODEL.locked["leg_a"] and MODEL.locked["leg_b"]


def success(objects) -> bool:
    """Both legs locked and holding under the check, the stool lying still, both hands clear. Must hold for 1 s."""
    return both_locked() and MODEL.checked


def score(objects) -> float:
    """0.9 for both legs locked and holding with the hands off (1.0 once success() has held 1 s); 0.5 for both legs
    locked; 0.25 for one; 0 otherwise. A leg that only looks open earns nothing."""
    if both_locked():
        return COMPLETE_SCORE if MODEL.checked else 0.5
    return 0.25 if (MODEL.locked["leg_a"] or MODEL.locked["leg_b"]) else 0.0


# -- the scripted demo -----------------------------------------------------------------------------------------------
# One leg at a time, each by the hand on its side: the hand pinches the leg's crossbar where it lies folded on the
# platform's underside and swings it up and out round its hinge, past square, until the lock clicks; then lets go and
# moves off. The round crossbar turns between the pads as the leg turns. The hand points down throughout, or turns with
# the leg up to FOLLOW_DEG and then holds that turn (offline IK: a hand turning with the leg reaches only to about 84
# degrees; pointing down, it reaches the whole swing).
def _allow_matrix_grips() -> None:
    """Let a Waypoint's ``yaw`` be a full 3x3 TCP rotation instead of an angle (yam_arena.demo builds each waypoint's
    rotation with grip_rotation(yaw), which this wraps)."""
    if getattr(_demo, "_matrix_grips", False):
        return
    plain = _demo.grip_rotation

    def grip_rotation_any(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else plain(yaw)

    _demo.grip_rotation = grip_rotation_any
    _demo._matrix_grips = True


_allow_matrix_grips()
OPEN, CLOSED = -0.04695, 0.0
PAD_TIP = 13.0 * MM  # the pads' tips past the TCP
DOWN = grip_rotation(0.0)  # pointing down, the fingers closing along y: across the crossbar (which runs along x)
WIDE = -(BAR_R + 0.010)  # the fingers 1 cm clear of the crossbar either side
BELOW = float(os.environ.get("STOOL_BELOW", "0.004"))  # the pads' tips this far past the crossbar's middle
# WHY: with the tips level with its middle, the pads touched only its upper half and squeezed it down (run vive3)
SQUEEZE = float(os.environ.get("STOOL_SQUEEZE", "0.0025"))  # the fingers aimed this far into the crossbar
# WHY a medium pinch: closed fully (about 45 N) the crossbar couldn't turn between the pads and slipped out at 13 degrees
# (run vive2); aimed 1.5 mm in, with the pads high on it, the leg didn't lift (vive3)
PINCH = -(BAR_R - SQUEEZE)
# WHY turn with the leg to 60 degrees: pointing down throughout, the left hand lost its leg at 83 degrees (run vivef0,
# score 0.25); turning to 60 and holding that, both legs locked (vivef60, 1.0)
FOLLOW_DEG = float(os.environ.get("STOOL_FOLLOW", "60.0"))
SWING_PAST = float(os.environ.get("STOOL_SWING_PAST", "6.0"))  # aim this far past the stop (degrees)


def bar_centre(leg: str, opening_deg: float) -> np.ndarray:
    """The middle of the leg's crossbar (world) with the leg open ``opening_deg``, the stool where it starts."""
    s, p = INWARD[leg], math.radians(opening_deg)
    d_local = np.array([0.0, s * math.cos(p), -math.sin(p)])  # hinge to foot, platform frame
    return TOP_AT + UPSIDE_DOWN @ (hinge_local(leg) + LEG_L * d_local)


def turn_sign(leg: str) -> float:
    """The sign of the leg's opening turn about world x (the stool upside down)."""
    return 1.0 if float(bar_centre(leg, 0.0)[1]) < 0 else -1.0


def swing_grip(leg: str, opening_deg: float, up: float = 0.0):
    """The pinching hand at the leg's opening: pointing down, turned with the leg up to FOLLOW_DEG; the pads across the
    crossbar's middle (tips BELOW past it)."""
    turn = min(opening_deg, FOLLOW_DEG)
    rot = axis_angle_matrix(X_AXIS, turn_sign(leg) * math.radians(turn)) @ DOWN
    tcp = bar_centre(leg, opening_deg) - rot[:, 2] * (PAD_TIP - BELOW) + np.array([0.0, 0.0, up])
    return tuple(float(v) for v in tcp), rot


PARK_RIGHT, PARK_LEFT = (0.16, -0.32, TOP + 0.16), (0.16, 0.32, TOP + 0.16)


def _open_leg(leg: str) -> list:
    tcp0, rot0 = swing_grip(leg, 0.0)
    plan = [
        Waypoint(1.5, swing_grip(leg, 0.0, up=0.05)[0], WIDE, "over the crossbar", rot0),
        Waypoint(1.0, tcp0, WIDE, "fingers either side of it", rot0),
        # WHY 1.5 s and a hold: the fingers close slower than commanded (run vive1: the swing began before they held)
        Waypoint(1.5, tcp0, PINCH, "pinch the crossbar", rot0),
        Waypoint(0.5, tcp0, PINCH, "hold", rot0),
    ]
    for d in (15.0, 30.0, 45.0, 60.0, 75.0, 88.0, 96.0, OPEN_DEG + SWING_PAST):
        tcp, rot = swing_grip(leg, d)
        plan.append(Waypoint(0.8, tcp, PINCH, f"swing the leg to {d:g} degrees", rot))
    end, rot_end = swing_grip(leg, OPEN_DEG + SWING_PAST)
    out = -1.0 if bar_centre(leg, 0.0)[1] < 0 else 1.0
    plan += [
        Waypoint(0.6, end, PINCH, "hold: the lock clicks", rot_end),
        Waypoint(0.6, end, WIDE, "let go", rot_end),
        Waypoint(0.8, (end[0], end[1] + out * 0.02, end[2] + 0.05), WIDE, "up off it", rot_end),
    ]
    return plan


def demo():
    """The right hand opens the leg on its side, then the left hand the other, each waiting out of the way while the
    other works (both near the middle at once, the arms would meet)."""
    right = _open_leg("leg_b")  # upside down, leg_b (the platform's +y end) lies on the robot's right
    t_right = sum(w.duration for w in right)
    left = [Waypoint(t_right, PARK_LEFT, CLOSED, "wait for the right hand", DOWN)] + _open_leg("leg_a")
    t_left = sum(w.duration for w in left)
    right += [Waypoint(1.5, PARK_RIGHT, CLOSED, "clear", DOWN),
              Waypoint(max(t_left - t_right - 1.5, 0.1) + 3.0, PARK_RIGHT, CLOSED, "stay", DOWN)]
    left += [Waypoint(1.5, PARK_LEFT, CLOSED, "clear", DOWN), Waypoint(3.0, PARK_LEFT, CLOSED, "stay", DOWN)]
    return {"left": left, "right": right}
