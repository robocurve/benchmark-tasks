"""Instant photo with a Polaroid Go Generation 2 (as modelled: its outer size from the maker's page, 105 x 84 x 62 mm,
no name or logo). On the bench, in front of the two arms: the camera (free, 259 g with its film pack) standing 28 cm
out, 5.5 cm left of the centre line, its lens pointing away from the arms at a red apple (fixed, 7 cm) 15 cm further
out. The red shutter button is on the camera's top, at its front right; the print leaves through a slot at the bottom
of the front face, 5 mm above the bench.

The shutter is a button on a spring: it fires when the button has been pressed home (9/10 of its 2 mm travel), not
on where a fingertip is. The camera holds one exposure: the first press home takes the photo, which is good only if
at that moment the camera is level (within 20 degrees), still (under 2 cm/s), and the apple is within 12 degrees of
the lens axis and 6 to 40 cm from the lens; otherwise the exposure is wasted and the episode ends. As the maker's
manual says, the print comes out when the button is let go: over the next 1.5 s it comes 4.5 cm out of the slot,
picture side up; its last 2.2 cm stay between the camera's rollers, which hold it wherever the camera goes until a
gripper pinches it across its faces (a real print bends when it is squeezed by its edges) and pulls it out. Pulling
drags on the camera with 1.5 N, and the camera's smooth underside slides on the bench under about 0.8 N: unless the
camera is held, it follows the print. The slot is 5 mm above the bench and faces away from the arms: the print cannot
be pinched there, so the camera has to be picked up and turned first.

Take a photo of the apple, pull the print out of the camera and lay it face down on the bench (the maker: "place it
face down to continue shielding it from light as it develops"), but not before it has stood in the slot 5 s after it
came out (the maker: "Leave your photo under the film shield for about 5 seconds"; the shield itself is not
modelled); leave the camera standing on the bench, both hands off, for 3 s. The picture's development (10 to 15
minutes for Go film) is not simulated and not scored.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint

from rta.plans import RELEASE_S, SETTLE_S, SQUEEZE_S, Pose, close_time, grip, opening, wait
from rta.spec import TABLE_TOP, Asset, quat_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
REST = TOP + 0.001  # models are spawned 1 mm over the bench (yam_arena station.REST_GAP)
L = json.loads((Path(__file__).resolve().parent / "assets" / "layout.json").read_text())  # written by assets/build.py
INSTRUCTION = ("Take a photo of the apple with the instant camera, leave the print in the slot for 5 seconds before you "
               "pull it out, lay it face down on the bench, and leave the camera standing on the bench.")
ACT_AFTER_S = 1.0  # nothing to watch: the arms are free after a short settle
MEMORY = False  # the wait is a settle only: nothing is shown, so the policy is told no memory-task rule

# --- the camera (its frame: x out of the lens, y to its left, z up, origin on the bench under its centre) --------
C = L["camera"]
CAM_SIZE = tuple(C["size"])  # depth, width, height up to the button's top
CAM_TOP = C["top"]  # the body's top
FRONT = C["front"]  # the front face, where the slot is
BOX_C = np.asarray(C["box_centre"], float)  # the simulator's centroid of the model (the lens stands out in front)
GRIP = tuple(C["grip"])  # where a hand from above holds the camera's rear part across its width
SHUTTER = tuple(C["shutter"])  # the button's top centre at rest
SHUTTER_TRAVEL = C["shutter_travel"]
LENS = tuple(C["lens"])  # the lens's front centre
T = L["tunnel"]
FLOOR = T["floor"]
P = L["print"]
PRINT = tuple(P["size"])  # length (along the slot), width, thickness
HIDDEN_X, EJECTED_X = P["hidden_x"], P["ejected_x"]  # the print's centre in the camera's frame: inside / ejected
PRINT_Z = FLOOR + 0.0005  # the print's underside in the camera's frame while it is in the tunnel

CAM_XY = (0.279, 0.055)  # the camera's origin
APPLE_XY, APPLE_EXT = (CAM_XY[0] + 0.15, CAM_XY[1]), 0.07
A = L["assets"]
OBJECTS = [
    Asset("camera", asset=A["camera"], xy=(CAM_XY[0] + BOX_C[0], CAM_XY[1] + BOX_C[1]), extent=L["extent"]["camera"], mass=C["mass"]),
    Asset("print", asset=A["print"], xy=(CAM_XY[0] + HIDDEN_X, CAM_XY[1]), extent=L["extent"]["print"], mass=P["mass"]),
    Asset("apple", asset="apple_02_objaverse_robolab", xy=APPLE_XY, extent=APPLE_EXT),
]
MOUNTED = {("print", "camera")}  # the print starts inside the camera: not a spawn overlap
# the camera is an articulation (assets/build.py): its shutter button on a prismatic joint, held up by this spring
JOINTS = {"camera": {"shutter": (C["shutter_spring"], C["shutter_damping"])}}

# --- rules -------------------------------------------------------------------------------------------------------
PRESSED = 0.9 * SHUTTER_TRAVEL  # the shot: the button pressed home
RELEASED = 0.25 * SHUTTER_TRAVEL  # let go: the button back within a quarter of its travel of rest
LEVEL_DEG, AIM_DEG, AIM_DIST = 20.0, 12.0, (0.06, 0.40)
STEADY = 0.02  # m/s (its box centre's speed; the simulator's spin reading is noisy for a body held in a grip)
EJECT_S = 1.5  # the print takes this long to come out, once the button is let go
SHIELD_S = 5.0  # the maker: "Leave your photo under the film shield for about 5 seconds" before it is drawn out
RETAIN_N = 1.5  # the rollers' drag on a print being pulled out, felt by the camera
RETAIN_V = 0.02  # m/s: the drag reaches RETAIN_N at this pulling speed
GRAVITY = 9.81
PINCH_DIST, PINCH_FINGER = 0.045, -0.003  # a gripper pinches the print: within 4.5 cm of its centre, fingers under 6 mm apart
PINCH_AXIS_DEG = 30.0  # ... and across its faces: the fingers' closing line within this of the print's face normal
OUT = 0.003  # the print is out when its inner end is this far in front of the slot
FACE_DOWN_DEG = 15.0
ON_BENCH = 0.006  # lying / standing on the bench: the lowest point within this of it
CLEAR = 0.04  # hands off: every tool centre point at least this far from the object's box
KNOCKED_DEG = 60.0


def setup(rng, trial) -> dict:
    return {"shot_t": None, "release_t": None, "out_t": None, "freed_t": None, "fail": None, "depth": HIDDEN_X, "loose": False}


def placements(state) -> dict:
    return {"print": (CAM_XY[0] + HIDDEN_X, CAM_XY[1], REST + PRINT_Z)}


# --- geometry ----------------------------------------------------------------------------------------------------
def frame(cam):
    """The camera's rotation (columns: lens axis, left, up) and its origin (on its underside, under its centre)."""
    rot = quat_matrix(cam.quat_xyzw)
    return rot, np.asarray(cam.pos, float) - rot @ BOX_C


def to_world(cam, local) -> np.ndarray:
    rot, origin = frame(cam)
    return origin + rot @ np.asarray(local, float)


def to_local(cam, world) -> np.ndarray:
    rot, origin = frame(cam)
    return rot.T @ (np.asarray(world, float) - origin)


def tilt_deg(o) -> float:
    return math.degrees(math.acos(float(np.clip(quat_matrix(o.quat_xyzw)[2, 2], -1.0, 1.0))))


def box_distance(o, p) -> float:
    p = np.asarray(p, float)
    return float(np.linalg.norm(p - np.clip(p, o.low, o.high)))


def shutter_in(objects) -> float:
    """How far (m) the shutter button is pressed in from rest: the camera's own joint."""
    return -float(objects.joints.get("camera", {}).get("shutter", 0.0))


def aim(objects) -> tuple[float, float]:
    """(angle in degrees between the lens axis and the apple, distance from the lens to the apple), on the bench plane."""
    cam, apple = objects["camera"], objects["apple"]
    rot, _ = frame(cam)
    v = np.asarray(apple.pos, float)[:2] - to_world(cam, LENS)[:2]
    dist = float(np.linalg.norm(v))
    fwd = rot[:2, 0] / max(float(np.linalg.norm(rot[:2, 0])), 1e-9)
    return math.degrees(math.acos(float(np.clip(fwd @ v / max(dist, 1e-9), -1.0, 1.0)))), dist


def good_shot(objects) -> bool:
    cam = objects["camera"]
    angle, dist = aim(objects)
    return bool(tilt_deg(cam) <= LEVEL_DEG and angle <= AIM_DEG and AIM_DIST[0] <= dist <= AIM_DIST[1]
                and cam.resting(max_speed=STEADY, max_spin=math.inf))


def print_in_camera(objects) -> np.ndarray:
    """The print's centre in the camera's frame."""
    return to_local(objects["camera"], objects["print"].pos)


def in_rollers(objects) -> bool:
    q = print_in_camera(objects)
    return bool(q[0] - PRINT[0] / 2 < FRONT - 0.001 and abs(q[1]) <= T["width"] / 2 and -0.005 <= q[2] <= 0.02)


def print_out(objects) -> bool:
    """No part of the print is inside the camera any more."""
    q = print_in_camera(objects)
    return bool(q[0] - PRINT[0] / 2 >= FRONT + OUT or abs(q[1]) >= CAM_SIZE[1] / 2 + PRINT[1] / 2 or q[2] >= FLOOR + T["gap"] + 0.01
                or q[2] <= -0.01 or q[0] <= -CAM_SIZE[0] / 2 - PRINT[0] / 2)


def pinched(objects) -> bool:
    """A gripper has the print between its fingers, pinched across its faces. A real print is a thin sheet that bends
    when it is squeezed by its edges; the simulated one is a rigid card, so a pinch by its edges does not count."""
    pr = objects["print"]
    normal = quat_matrix(pr.quat_xyzw)[:, 2]
    across = math.cos(math.radians(PINCH_AXIS_DEG))
    for arm, tcp in objects.tcp.items():
        rot = objects.tcp_rot.get(arm)
        if (rot is not None and np.linalg.norm(tcp - pr.pos) <= PINCH_DIST and objects.finger.get(arm, -1.0) >= PINCH_FINGER
                and abs(float(rot[:, 0] @ normal)) >= across):
            return True
    return False


def hands_off(objects, name: str) -> bool:
    return all(box_distance(objects[name], tcp) >= CLEAR for tcp in objects.tcp.values())


def picture_up(pr) -> float:
    """Cosine of the angle between the picture side (the print's top while it is in the camera) and straight up."""
    return float(quat_matrix(pr.quat_xyzw)[2, 2])


def print_face_down(objects) -> bool:
    """Out of the camera, flat on the bench with the picture side down (the maker's instruction), still, let go."""
    pr = objects["print"]
    return bool(print_out(objects) and picture_up(pr) <= -math.cos(math.radians(FACE_DOWN_DEG)) and pr.low[2] <= TOP + ON_BENCH
                and pr.resting(max_speed=0.01, max_spin=math.inf) and hands_off(objects, "print"))


def camera_standing(objects) -> bool:
    cam = objects["camera"]
    return bool(tilt_deg(cam) <= LEVEL_DEG and cam.low[2] <= TOP + ON_BENCH and cam.resting(max_speed=0.01, max_spin=math.inf)
                and hands_off(objects, "camera"))


# --- what the camera does ----------------------------------------------------------------------------------------
def ejected(t: float, state: dict) -> bool:
    return state["release_t"] is not None and t >= state["release_t"] + EJECT_S


def waited(state: dict) -> bool:
    """The print stayed in the slot SHIELD_S or more after it came out before it was drawn free of the rollers."""
    return state["out_t"] is not None and state["freed_t"] is not None and state["freed_t"] - state["out_t"] >= SHIELD_S


def events(t: float, state: dict, world) -> None:
    """The rollers: they hold the print in the tunnel wherever the camera goes (inside until the button is let go
    after the shot), push it 4.5 cm out of the slot over EJECT_S once it is let go, and keep holding it there until a
    gripper pinches it; a pinched print is left to the gripper and the tunnel, and once it is out of the slot it is
    free for good."""
    objects = world.objects
    if objects is None:
        return
    if not state["loose"]:
        if ejected(t, state) and not state.get("_came_out"):
            state["depth"], state["_came_out"] = EJECTED_X, True
            state["out_t"] = state["release_t"] + EJECT_S
        if state["release_t"] is not None and t < state["release_t"] + EJECT_S:
            s = (t - state["release_t"]) / EJECT_S
            state["depth"] = HIDDEN_X + s * s * (3 - 2 * s) * (EJECTED_X - HIDDEN_X)
            held = True
        elif ejected(t, state) and not in_rollers(objects):
            state["loose"], state["freed_t"] = True, t
            held = False
        elif ejected(t, state) and pinched(objects):
            state["depth"] = float(np.clip(print_in_camera(objects)[0], HIDDEN_X, FRONT + PRINT[0] / 2))  # where it is let go, it stays
            held = False
        else:
            held = True
        if held:
            cam = objects["camera"]
            world.set_pose("print", to_world(cam, (state["depth"], 0.0, PRINT_Z)), quat=tuple(float(v) for v in cam.quat_xyzw))


def forces(objects, state) -> dict:
    """The rollers' drag: while a gripper pulls the print out of the slot, the camera is pulled along with up to
    RETAIN_N (in proportion to the pulling speed up to RETAIN_V). While the rollers hold the print (``events`` puts
    it in its place each step), they also carry its weight, so it does not drop between steps. A force once pushed
    stays until the next push, so both objects are named every step."""
    out = {"camera": (0.0, 0.0, 0.0), "print": (0.0, 0.0, 0.0)}
    if state["loose"]:
        return out
    out_now = ejected(objects.t, state)
    if not (out_now and (pinched(objects) or not in_rollers(objects))):
        out["print"] = (0.0, 0.0, P["mass"] * GRAVITY)
        return out
    if not in_rollers(objects):
        return out
    cam, pr = objects["camera"], objects["print"]
    rot, _ = frame(cam)
    fwd = rot[:, 0]
    v = float((np.asarray(pr.lin_vel, float) - np.asarray(cam.lin_vel, float)) @ fwd)
    if v > 0.0:
        out["camera"] = tuple(float(RETAIN_N * min(1.0, v / RETAIN_V) * c) for c in fwd)
    return out


# --- scoring (privileged state) ----------------------------------------------------------------------------------
MILESTONES = ("photo_taken", "print_out", "print_face_down", "camera_standing_hands_off")
WEIGHTS = (0.25, 0.25, 0.25, 0.25)
# the photo and the print drawn free are events (they count once reached); the print face down and the camera
# standing are end states, read at every step: they count only while they hold, and the last one, the success
# condition, pays only after it has held SUCCESS_HOLD_S (rta.spec)
LATCHED = (True, True, False, False)
SUCCESS_HOLD_S = 3.0
ZERO_ON: set = set()


def _track(objects, state) -> None:
    """Once per step: the shutter fires on the first press home; the print comes out when the button is let go."""
    if state.get("_t") == objects.t:
        return
    state["_t"] = objects.t
    pressed_in = shutter_in(objects)
    if state["shot_t"] is None and state["fail"] is None and pressed_in >= PRESSED:
        if good_shot(objects):
            state["shot_t"] = objects.t
        else:
            state["fail"] = "wasted_exposure"
    if state["shot_t"] is not None and state["release_t"] is None and pressed_in <= RELEASED:
        state["release_t"] = objects.t


def milestones(objects, state) -> list[bool]:
    _track(objects, state)
    taken = state["shot_t"] is not None
    out = taken and ejected(objects.t, state) and print_out(objects)
    down = out and print_face_down(objects)
    return [taken, out, down, down and camera_standing(objects) and waited(state)]


def success(objects, state) -> bool:
    return milestones(objects, state)[-1]


def failure(objects, state) -> str | None:
    _track(objects, state)
    if state["fail"]:
        return state["fail"]
    cam = objects["camera"]
    if tilt_deg(cam) > KNOCKED_DEG and hands_off(objects, "camera") and cam.low[2] <= TOP + ON_BENCH:
        return "camera_knocked_over"  # lying on the bench, let go (a held camera may be turned any way)
    if any(objects[n].high[2] < TOP - 0.05 for n in ("camera", "print")):
        return "dropped_off_bench"
    return None


# --- privileged oracle -------------------------------------------------------------------------------------------
TIP = 0.013  # the closed fingertips reach this far past the tool centre point
HALF_PI = math.pi / 2


def _rodrigues(axis, angle: float) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


Y = np.array([0.0, 1.0, 0.0])
LEAN = math.radians(-15)  # the holding hand's fingers point back towards the arms this far: turned lens up, it then points
# 15 degrees down, within the arm's reach (straight down or leaning back, it would have to point level or up)
HOLD_ROT = grip((math.sin(LEAN), 0.0, -math.cos(LEAN)), (0.0, -1.0, 0.0))  # from above, across the camera's width
CAM_OPEN = opening(0.047)  # fully open: 5 mm either side of the 84 mm camera
CAM_HOLD = opening(CAM_SIZE[1] / 2 - 0.006)  # a firm, moderate squeeze
HOLD = np.array([CAM_XY[0] + GRIP[0], CAM_XY[1] + GRIP[1], TOP + GRIP[2]])
SHUTTER_AT = np.array([CAM_XY[0] + SHUTTER[0], CAM_XY[1] + SHUTTER[1], TOP + SHUTTER[2]])  # the button's top at rest
PRESS_PAST = 0.018  # the fingertips are sent this far past the button's travel: the arm gives 3.2 mm per newton
# downwards there (its joint stiffness, 80 and 10 N m/rad), so it pushes about 5 N against the spring's 4 N
LIFT = 0.03  # the camera is lifted this far before it is tipped: its lowest corner then stays 2.5 cm over the bench
TIP_UP, TIP_STEPS, TIP_S = HALF_PI, 3, 2.7  # the hand turns the lens up, about the line between its fingers
TIPPED = _rodrigues(Y, -TIP_UP)  # the camera's rotation once the lens is up
HELD_UP = HOLD + np.array([0.0, 0.0, LIFT])
LIFT_SAG = np.array([-0.003, 0.001, -0.0125])  # the holding arm, loaded, sits this far from its command (measured)
PRINT_AXIS = TIPPED @ np.array([1.0, 0.0, 0.0])  # the way the print points (and comes out) once tipped: up
SLOT_UP = HELD_UP + TIPPED @ (np.array([FRONT, 0.0, PRINT_Z + PRINT[2] / 2]) - np.asarray(GRIP))  # the print's mid-plane at the slot
PRINT_OPEN = 0.012  # per finger, around the print's thickness: the open fingers pass 5 mm over the lens
PINCH_IN, PINCH_BELOW_TIP = 0.012, 0.010  # the fingers reach this far in from the print's edge, this far below its end
SAG = np.array([0.0, 0.0, 0.0])  # the pinching arm, unloaded, reaches the pinch within 2 mm (measured)
PINCH = SLOT_UP + (P["protrude"] - PINCH_BELOW_TIP) * PRINT_AXIS + np.array([0.0, -PRINT[1] / 2 + PINCH_IN, 0.0]) + SAG
PINCH_ROT = grip(Y, (1.0, 0.0, 0.0))  # from the right, along the print's width; closing across its thickness
BESIDE = PINCH - 0.05 * Y  # the open fingers level with the pinch, clear of the print's edge
PULL, PULL_S = 0.06, 2.0  # 2.2 cm of the print are in the slot
LAY_LEAN = math.radians(20)  # stood on its end leaning this far, towards its picture side: let go, it falls face down
LAY_ROT = _rodrigues(Y, -LAY_LEAN) @ PINCH_ROT
LAY_XY = (0.38, -0.04)  # the print lies to the right of the camera and the apple, clear of both
LAY_Z = TOP + (PRINT[0] - PINCH_BELOW_TIP) * math.cos(LAY_LEAN) + 0.004  # its lower end 4 mm over the bench


def demo(state) -> dict:
    """The right hand presses the shutter button home from above while the camera stands on the bench, lets it go and
    moves off; the print comes out; the left comes down on the camera's rear part and holds it across its width, lifts
    it and turns the lens up about the line between its fingers; the right comes in from the side, pinches the
    print's end across its thickness and pulls it up out of the slot, then stands it on its end leaning towards its
    picture side and lets go: it falls face down; meanwhile the left turns the camera back, sets it down and lets go;
    both move off."""
    sx, sy, sz = SHUTTER_AT
    touch_z = sz + TIP + 0.004
    press_z = sz + TIP - SHUTTER_TRAVEL - PRESS_PAST
    high = touch_z + 0.03  # higher, a hand pointing straight down cannot reach so far out
    shoot = [
        Waypoint(2.0, (sx, sy, high), FINGER_CLOSED, "over the shutter", 0.0),
        Waypoint(1.0, (sx, sy, touch_z), FINGER_CLOSED, "onto the button", 0.0),
        Waypoint(1.0, (sx, sy, press_z), FINGER_CLOSED, "press", 0.0),
        Waypoint(0.5, (sx, sy, press_z), FINGER_CLOSED, "hold press", 0.0),
        Waypoint(0.8, (sx, sy, touch_z + 0.006), FINGER_CLOSED, "let go", 0.0),
        Waypoint(1.0, (sx, sy, high), FINGER_CLOSED, "up", 0.0),
    ]
    h = HOLD
    over = h + np.array([0.0, 0.0, 0.08])
    take = [
        Pose(2.0, tuple(over), CAM_OPEN, "over the camera", rot=HOLD_ROT),
        Pose(1.5, tuple(h), CAM_OPEN, "down to it", rot=HOLD_ROT),
        Pose(close_time(0.047), tuple(h), CAM_HOLD, "close", rot=HOLD_ROT),
        Pose(SQUEEZE_S, tuple(h), CAM_HOLD, "squeeze", rot=HOLD_ROT),
    ]
    angles = [TIP_UP * (k + 1) / TIP_STEPS for k in range(TIP_STEPS)]
    rolls = [_rodrigues(Y, -a) @ HOLD_ROT for a in angles]
    up = tuple(HELD_UP - LIFT_SAG)  # commanded so that the camera is where the pinch is planned
    tip = [
        Pose(1.2, up, CAM_HOLD, "lift", rot=HOLD_ROT),
        *[Pose(TIP_S / TIP_STEPS, up, CAM_HOLD, "tip the lens up", rot=r) for r in rolls],
        Pose(SETTLE_S, up, CAM_HOLD, "settle", rot=rolls[-1]),
    ]
    down_at = (h[0], h[1], h[2] + 0.002)
    back = [
        *[Pose(TIP_S / TIP_STEPS, up, CAM_HOLD, "tip back", rot=r) for r in [*rolls[-2::-1], HOLD_ROT]],
        Pose(1.4, down_at, CAM_HOLD, "set down", rot=HOLD_ROT),
        Pose(SETTLE_S, down_at, CAM_HOLD, "settle", rot=HOLD_ROT),
        Pose(RELEASE_S, down_at, CAM_OPEN, "release", rot=HOLD_ROT),
        Pose(1.2, tuple(over), CAM_OPEN, "up and off", rot=HOLD_ROT),
    ]
    open_print = opening(PRINT_OPEN)
    pulled = PINCH + PULL * PRINT_AXIS
    fetch = [
        Pose(2.0, tuple(BESIDE + np.array([0.0, 0.0, 0.03])), open_print, "beside the print", rot=PINCH_ROT),
        Pose(0.8, tuple(BESIDE), open_print, "level with it", rot=PINCH_ROT),
        Pose(1.2, tuple(PINCH), open_print, "onto the print", rot=PINCH_ROT),
        Pose(close_time(PRINT_OPEN), tuple(PINCH), FINGER_CLOSED, "close", rot=PINCH_ROT),
        Pose(SQUEEZE_S, tuple(PINCH), FINGER_CLOSED, "squeeze", rot=PINCH_ROT),
        Pose(PULL_S, tuple(pulled), FINGER_CLOSED, "pull out", rot=PINCH_ROT),
    ]
    lx, ly = LAY_XY
    lay_out = np.array([lx, ly - 0.06, LAY_Z + 0.02])
    lay = [
        Pose(1.5, (pulled[0], pulled[1] - 0.06, pulled[2]), FINGER_CLOSED, "away from the camera", rot=PINCH_ROT),
        Pose(2.0, (lx, ly, LAY_Z + 0.04), FINGER_CLOSED, "carry", rot=LAY_ROT),
        Pose(1.2, (lx, ly, LAY_Z), FINGER_CLOSED, "stand it on its end", rot=LAY_ROT),
        Pose(SETTLE_S, (lx, ly, LAY_Z), FINGER_CLOSED, "settle", rot=LAY_ROT),
        Pose(RELEASE_S, (lx, ly, LAY_Z), opening(0.004), "let go", rot=LAY_ROT),
        Pose(1.5, tuple(lay_out), opening(0.004), "back out", rot=LAY_ROT),
    ]
    d = lambda wps: sum(w.duration for w in wps)  # noqa: E731
    d_shoot, d_take, d_tip, d_fetch, d_lay, d_back = (d(p) for p in (shoot, take, tip, fetch, lay, back))
    take_at = d_shoot + 0.8  # the right hand is on its way off
    tip_at = take_at + d_take
    fetch_at = tip_at + d_tip
    back_at = fetch_at + d_fetch + 1.5  # the print is out and away from the camera
    end = max(fetch_at + d_fetch + d_lay, back_at + d_back) + SUCCESS_HOLD_S + 2.0
    left = [
        wait("left", take_at),
        *take,
        *tip,
        Pose(back_at - fetch_at, up, CAM_HOLD, "hold", rot=rolls[-1]),
        *back,
    ]
    left.append(wait("left", end - d(left)))
    right = [
        *shoot,
        wait("right", fetch_at - d_shoot, finger=FINGER_CLOSED),
        *fetch,
        *lay,
    ]
    right.append(wait("right", end - d(right)))
    return {"left": (0.0, left), "right": (0.0, right)}
