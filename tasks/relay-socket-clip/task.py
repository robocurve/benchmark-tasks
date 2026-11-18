"""On the bench lies a control-panel back plate with a DIN rail and a 14-pin plug-in relay socket. The old relay has been
pulled; the socket's hold-down clip stands open, leaning back on its stop. The replacement relay stands upright in a
foam parts tray beside the plate, blades down and the right way round for the socket.

The robot must fit the new relay: push its blades straight down into the socket until the relay sits flat on it, then
swing the hold-down clip over the relay and press it until it snaps under its catch, then let go. The clip only closes
fully over a relay that is pushed all the way home.

Everything is a real product at true size: a 4-pole plug-in relay (21.5 x 28 x 36 mm body, 14 blades, 35 g), its
DIN-rail socket and its hold-down clip, as used in machine control panels.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import (Hinge, PartState, Slider, finger_points, on_reset, on_step, part,  # noqa: E402
                     set_hinge_stiffness, set_hinge_target, set_slider_target)
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Fit the new relay in the empty socket and latch its retaining clip."

GREY = (0.55, 0.57, 0.60)
RAIL = (0.78, 0.78, 0.80)
SOCKET = (0.20, 0.21, 0.23)
CLEAR_BLUE = (0.55, 0.70, 0.85)
CLIP = (0.85, 0.85, 0.82)
FOAM = (0.15, 0.15, 0.16)

# Socket part frame: origin on the bench under the socket's centre; x along the rail.
PLATE_T, RAIL_T = 0.005, 0.0075
SOCKET_H = 0.025
S_TOP = PLATE_T + RAIL_T + SOCKET_H          # the socket's top face above the bench (37.5 mm)
CAV = (0.011, 0.008, 0.007)                  # cavity half-length (x), half-width (y), depth: 22 x 16 x 7 mm
S_HALF = (0.035, 0.0148)                     # socket body half-length, half-width (70 x 29.5 mm)
SOCKET_XY = (0.300, -0.100)
POST_X = (0.021, 0.027)                      # the clip's hinge post at the socket's +x end
RELAY_H = 0.036                              # relay body height
PIVOT = (0.024, 0.0, S_TOP + RELAY_H + 0.001)
CLIP_L = 0.044                               # from the pivot to the clip's free end, just past the relay's end
CLIP_OPEN = math.radians(120.0)              # its stop when open (leaning back 30 degrees past upright)
CLIP_PRESS = math.radians(-1.5)              # pressed this far past flat it snaps under its catch
CONTACT_TRAVEL = 0.0064                      # the blades' full insertion depth
CONTACT_K, CONTACT_F = 20000.0, 3.0           # the contacts' grip: they hold until pushed with more than 3 N

SOCKET_BOXES = [
    ((0.0, 0.0, PLATE_T / 2), (0.200, 0.160, PLATE_T), GREY),                                   # back plate
    ((0.0, 0.0, PLATE_T + RAIL_T / 2), (0.200, 0.035, RAIL_T), RAIL),                           # DIN rail
    # socket body around the blade cavity
    ((0.0, 0.0, PLATE_T + RAIL_T + 0.009), (2 * S_HALF[0], 2 * S_HALF[1], 0.018), SOCKET),      # base under cavity
    ((-(CAV[0] + S_HALF[0]) / 2, 0.0, S_TOP - 0.0035), (S_HALF[0] - CAV[0], 2 * S_HALF[1], 0.007), SOCKET),
    (((CAV[0] + S_HALF[0]) / 2, 0.0, S_TOP - 0.0035), (S_HALF[0] - CAV[0], 2 * S_HALF[1], 0.007), SOCKET),
    ((0.0, (CAV[1] + S_HALF[1]) / 2, S_TOP - 0.0035), (2 * CAV[0], S_HALF[1] - CAV[1], 0.007), SOCKET),
    ((0.0, -(CAV[1] + S_HALF[1]) / 2, S_TOP - 0.0035), (2 * CAV[0], S_HALF[1] - CAV[1], 0.007), SOCKET),
    # the clip's hinge post at the +x end, up to the relay's top
    (((POST_X[0] + POST_X[1]) / 2, 0.0, (S_TOP + PIVOT[2]) / 2), (POST_X[1] - POST_X[0], 0.024, PIVOT[2] - S_TOP),
     SOCKET),
]
# The contacts: a plate the blades bear on, held by friction wherever it is pushed (the blades' forks)
CONTACTS = Slider("contacts", rest=(0.0, 0.0, S_TOP - 0.0005), boxes=(
    ((0.0, 0.0, S_TOP - 0.0015), (2 * CAV[0] - 0.002, 2 * CAV[1] - 0.002, 0.002), (0.75, 0.60, 0.25)),),
    travel=CONTACT_TRAVEL, rate=CONTACT_K, damping=20.0, effort=CONTACT_F, mass=0.005, axis="Z", sign=-1)
_CLOSED_BOXES = (  # in its closed pose: a hood over the relay's top, hinged at the post
    ((PIVOT[0] - CLIP_L / 2, 0.0, PIVOT[2] + 0.001), (CLIP_L, 0.024, 0.002)),
    ((PIVOT[0] - CLIP_L + 0.002, 0.0, PIVOT[2] - 0.003), (0.004, 0.024, 0.008)),          # its catch lip
)


def _open_pose(centre, size):
    """A closed-pose box turned up about the pivot by CLIP_OPEN: the clip's geometry at hinge angle 0 (open)."""
    c, s_ = math.cos(CLIP_OPEN), math.sin(CLIP_OPEN)
    rx, rz = centre[0] - PIVOT[0], centre[2] - PIVOT[2]
    pos = (PIVOT[0] + rx * c + rz * s_, centre[1], PIVOT[2] - rx * s_ + rz * c)
    return (pos, size, CLIP, (math.cos(CLIP_OPEN / 2), 0.0, math.sin(CLIP_OPEN / 2), 0.0))


CLIP_BOXES = tuple(_open_pose(c, sz) for c, sz in _CLOSED_BOXES)
SOCKET_PART = part("relay_socket", pos=(*SOCKET_XY, TOP), mass=2.0, color=SOCKET, friction=0.8,
                   boxes=SOCKET_BOXES, sliders=(CONTACTS,), hinges=(
                       Hinge("clip", pivot=PIVOT, boxes=CLIP_BOXES, lower=CLIP_PRESS - CLIP_OPEN, upper=0.0,
                             stiffness=0.0, damping=0.01, target=0.0, mass=0.01, axis="Y"),))

# The relay. Part frame: origin under the middle of its body's underside; the blade block below.
BODY = (0.028, 0.0215)                       # body length (x), width (y)
BLADE_BLOCK = (0.020, 0.014, CONTACT_TRAVEL)  # the 14 blades' envelope
RELAY_MASS = 0.035
TRAY_XY = (0.200, -0.245)                   # clear of the back plate (its edge at y -0.180)
TRAY_H = 0.010
RELAY = part("relay", pos=(*TRAY_XY, TOP + 0.004 + CONTACT_TRAVEL + 0.0005), mass=RELAY_MASS, color=CLEAR_BLUE, friction=0.6, boxes=[
    ((0.0, 0.0, RELAY_H / 2), (BODY[0], BODY[1], RELAY_H), CLEAR_BLUE),
    ((0.0, 0.0, -BLADE_BLOCK[2] / 2), BLADE_BLOCK, (0.80, 0.78, 0.70)),
])
# The foam tray: a pocket the blades stand in
TRAY = [Box("tray_base", pos=(TRAY_XY[0], TRAY_XY[1], TOP + 0.002), size=(0.060, 0.050, 0.004), color=FOAM),
        Box("tray_x0", pos=(TRAY_XY[0] - 0.0205, TRAY_XY[1], TOP + 0.007), size=(0.019, 0.050, 0.006), color=FOAM),
        Box("tray_x1", pos=(TRAY_XY[0] + 0.0205, TRAY_XY[1], TOP + 0.007), size=(0.019, 0.050, 0.006), color=FOAM),
        Box("tray_y0", pos=(TRAY_XY[0], TRAY_XY[1] - 0.0165, TOP + 0.007), size=(0.022, 0.017, 0.006), color=FOAM),
        Box("tray_y1", pos=(TRAY_XY[0], TRAY_XY[1] + 0.0165, TOP + 0.007), size=(0.022, 0.017, 0.006), color=FOAM)]
OBJECTS = [SOCKET_PART, RELAY, *TRAY]

# -- scoring ------------------------------------------------------------------------------------------------------------
SEAT_Z = 0.001                   # the relay's body within 1 mm of the socket's top face
SEAT_XY = 0.0015
SEAT_TILT = math.radians(3.0)
ENTERED = 0.002                  # partial credit: the blades 2 mm into the socket
PLATE_MOVE = 0.005               # the plate pushed 5 mm on the bench: 0 for the rest of the trial
CLEAR = 0.030
STILL_STEPS = 20
STILL_TOL = 0.001
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(plate0=None, failed=False, why="", latched=False, entered=False, clear=False, recent=[], steps=0,
                  contact_hold=0.0)


_reset_state()


def tilt(state: PartState) -> float:
    return math.acos(max(-1.0, min(1.0, float(state.rotation()[2, 2]))))


def relay_offset(relay: PartState, sock: PartState) -> np.ndarray:
    """The relay's origin in the socket's frame, relative to the cavity's centre at the socket's top face."""
    return sock.to_local(relay.pos) - np.array([0.0, 0.0, S_TOP])


def seated(relay: PartState, sock: PartState) -> bool:
    d = relay_offset(relay, sock)
    return abs(float(d[2])) <= SEAT_Z and math.hypot(float(d[0]), float(d[1])) <= SEAT_XY and tilt(relay) <= SEAT_TILT


def blades_in(relay: PartState, sock: PartState) -> bool:
    d = relay_offset(relay, sock)
    return float(d[2]) <= BLADE_BLOCK[2] - ENTERED and math.hypot(float(d[0]), float(d[1])) <= CAV[1]


def clip_angle(sock: PartState) -> float:
    """The clip's angle above flat on a seated relay (rad): CLIP_OPEN when open on its stop, negative when pressed."""
    return float(sock.hinges.get("clip", 0.0)) + CLIP_OPEN


def box_distance(point, state: PartState, centre, half) -> float:
    p = state.to_local(point) - np.asarray(centre, float)
    return float(np.linalg.norm(np.maximum(0.0, np.abs(p) - np.asarray(half, float))))


def observe(relay: PartState, sock: PartState, fingers) -> None:
    s = _STATE
    if s["plate0"] is None:
        s["plate0"] = np.array(sock.pos, float)
    if float(np.linalg.norm(np.asarray(sock.pos) - s["plate0"])) > PLATE_MOVE:
        s["failed"], s["why"] = True, s["why"] or "back plate moved"
    if tilt(relay) > math.radians(60.0) or float(relay.pos[2]) < TOP - 0.01:
        s["failed"], s["why"] = True, s["why"] or "relay dropped"
    if blades_in(relay, sock):
        s["entered"] = True
    if clip_angle(sock) <= CLIP_PRESS + math.radians(0.3):
        s["latched"] = True                      # the catch snaps once the clip is pressed past flat
    if clip_angle(sock) > math.radians(5.0):
        s["latched"] = False                     # lifted off again: unlatched
    rel_c, rel_h = np.array([0.0, 0.0, RELAY_H / 2]), np.array([BODY[0] / 2, BODY[1] / 2, RELAY_H / 2])
    s["clear"] = fingers is not None and all(box_distance(f, relay, rel_c, rel_h) >= CLEAR for f in fingers)
    s["recent"].append(np.concatenate([np.asarray(relay.pos, float), [clip_angle(sock) * 0.04]]))
    del s["recent"][:-STILL_STEPS]
    s["steps"] += 1


@on_step
def _catch(environment) -> None:
    """The clip's catch: once pressed past flat it holds the clip there (a stiff drive at its latched angle)."""
    from partkit import _part_state

    sock, relay = _part_state(environment, SOCKET_PART), _part_state(environment, RELAY)
    try:
        fingers = [np.asarray(p, float) for p in finger_points(environment)]
    except Exception as exc:  # noqa: BLE001 - an unreadable gripper never counts as clear
        if not _STATE.get("warned"):
            print(f"relay-socket-clip: gripper check unavailable: {exc!r}", flush=True)
            _STATE["warned"] = True
        fingers = None
    observe(relay, sock, fingers)
    # the contacts' friction: they stick where they are until pushed past the drive's 6 N, then slip along with it
    pos, slip = float(sock.sliders.get("contacts", 0.0)), CONTACT_F / CONTACT_K
    hold = _STATE["contact_hold"]
    if pos < hold - slip:
        hold = pos + slip
    elif pos > hold + slip:
        hold = pos - slip
    _STATE["contact_hold"] = hold
    set_slider_target(environment, SOCKET_PART.name, "contacts", hold)
    if _STATE["latched"]:
        set_hinge_target(environment, SOCKET_PART.name, "clip", CLIP_PRESS - CLIP_OPEN)
        set_hinge_stiffness(environment, SOCKET_PART.name, "clip", 5.0)
    else:
        set_hinge_stiffness(environment, SOCKET_PART.name, "clip", 0.0)


def _still() -> bool:
    recent = _STATE["recent"]
    return len(recent) == STILL_STEPS and float(np.ptp(np.array(recent), axis=0).max()) < STILL_TOL


def distance(objects) -> float:
    """Metres from the relay's seated pose."""
    return float(np.linalg.norm(relay_offset(objects["relay"], objects["relay_socket"])))


def _level(objects) -> tuple[float, bool]:
    if _STATE["failed"]:
        return 0.0, False
    relay, sock = objects["relay"], objects["relay_socket"]
    if seated(relay, sock) and _STATE["latched"]:
        if _still() and _STATE["clear"]:
            return 1.0, True
        return 0.75, False
    if seated(relay, sock) and _STATE["clear"]:   # a seated relay counts only once both grippers have let go
        return 0.5, False
    if _STATE["entered"]:
        return 0.25, False
    return 0.0, False


def success(objects) -> bool:
    """The relay seated flat on its socket (within 1 mm, 1.5 mm of centre, 3 degrees square) with the hold-down clip
    snapped under its catch, both grippers 3 cm clear, nothing moving for 1 s; the plate never pushed 5 mm, the relay
    never dropped."""
    return _level(objects)[1]


def score(objects) -> float:
    """1.0 success; 0.75 seated and clipped but a gripper is near or something moves; 0.5 seated with the clip not
    latched and both grippers 3 cm clear; 0.25 the blades went 2 mm into the socket at some point; else 0. 0 once the
    plate was pushed 5 mm or the relay dropped."""
    return _level(objects)[0]
