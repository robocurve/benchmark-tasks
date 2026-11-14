"""Replace the cartridge on the razor handle with the replacement in the dispenser. Put the old cartridge in the
collection tray.

A Gillette Mach3-type manual razor lies on the mat, release button up, its old cartridge (orange mark) fitted. A
dispenser to the robot's right holds the replacement cartridge (green mark) blade-down in its pocket, connector socket
up; the collection tray is to the robot's left.

How the parts work (all by contact, springs and stops; nothing is attached, welded or released by a rule):
- The handle's connector is a post that fits down into a cartridge's socket. Two sprung latch teeth on the post's
  sides (slides with 45-degree ramps) snap out into windows in the socket's side walls: pushing the handle down onto a
  cartridge ramps them in and they snap out; pulling apart takes several newtons.
- Each latch is one piece with a release button standing out of the head's side. Pinching the head's sides pushes both
  buttons flush and both teeth back into the post: the cartridge is free, and lifting the handle leaves it behind.
- Each cartridge's blade unit pivots on its connector block (a sprung hinge, separate from the latch).

Scoring (final state):
- 0: anything fell off the table, the old cartridge is still (or again) latched on the handle, or nothing below holds.
- 0.25: old cartridge detached (unlatched and 10 mm or more from the handle), not stowed; replacement not retained.
- 0.5: old cartridge stowed in the tray; replacement not retained.
- 0.75: replacement retained (latched on the handle, out of the dispenser, released, and still latched through the
  retention load);
  old cartridge detached but not stowed.
- 1.0: replacement retained and old cartridge stowed.
Retention load: once the handle and replacement have been untouched for 2 s with the replacement latched, the
handle's connector is pulled away from the cartridge with twice the handle's weight for 2 s (the cartridge held down
with the same force): the hanging check, where the handle's own weight pulls on the connector, with a margin.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

import household as H  # noqa: E402
from housekit import Joint, Mech, Shape, box, every_step, on_reset, push, store  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
KEY = "replace-a-razor-cartridge"
MM = 0.001
G = 9.81


def b(at_mm, size_mm, color, tilt=(1.0, 0.0, 0.0, 0.0)):
    return box(tuple(v * MM for v in at_mm), tuple(v * MM for v in size_mm), color, tilt)


def tilt_x(deg):
    a = math.radians(deg) / 2
    return (math.cos(a), math.sin(a), 0.0, 0.0)


def tilt_y(deg):
    a = math.radians(deg) / 2
    return (math.cos(a), 0.0, math.sin(a), 0.0)


GREY, DARK, SILVER, BLUE = (0.55, 0.57, 0.60), (0.16, 0.17, 0.19), (0.80, 0.81, 0.83), (0.20, 0.38, 0.75)
RED = (0.80, 0.12, 0.12)
ORANGE, GREEN, CLEAR, WHITE = (0.95, 0.45, 0.05), (0.15, 0.70, 0.25), (0.75, 0.85, 0.90), (0.92, 0.92, 0.90)

# -- the handle. Frame: origin on the mat directly under the connector's axis; x along the handle toward its head, z up.
# Estimates (no Mach3 was measured): 131 mm long, 35 g. -----------------------------------------------------------------
HANDLE_MASS = 0.035
POST = (5.5, 7.5)                 # connector post, mm in x and y
POST_Z = (7.0, 14.5)              # from its tip (seated on the socket's floor) up to the head
TIP = (3.5, 5.5, 2.0)             # the post's narrower tip: a lead-in
HEAD = (-8.0, 8.0, 14.5, 24.5)    # head block x and z
HEAD_W = 16.0
LATCH_Z = 11.2                    # the latch teeth's centre height when seated
LATCH_HALF = 0.8                  # half a tooth's diagonal (a 1.13 mm square turned 45 degrees)
LATCH_OUT = 0.95                  # how far a tooth's point stands into its window, past the socket hole's face
LATCH_K, LATCH_PRELOAD = 1000.0, 0.002   # N/m; aimed 2 mm past its stop: about 2 N per tooth
BUTTON_OUT = 1.3                  # how far each release button stands out of the head's side


HOLE = (6.5, 8.5)                 # socket hole (x, y)
BLOCK = (12.0, 14.0)              # connector block outside (x, y)
SOCK_Z = (6.5, 14.0)              # block bottom to the socket's top
FLOOR_T = 0.5
BEVEL = 1.5                       # the socket's front and back walls are bevelled 45 degrees at the mouth: a lead-in
YBEVEL = 1.2                      # and its side walls, above the latch windows
WINDOW = (10.2, 12.2)             # the latch windows in the two side (y) walls


def _handle_shapes():
    px, py = POST
    a = math.degrees(math.atan2(19.0 - 6.0, 110.0))      # the shaft slopes down from the head to the butt
    return [
        b((0, 0, (POST_Z[0] + TIP[2] + POST_Z[1]) / 2), (px, py, POST_Z[1] - POST_Z[0] - TIP[2]), SILVER),
        b((0, 0, POST_Z[0] + TIP[2] / 2), (TIP[0], TIP[1], TIP[2]), SILVER),
        b(((HEAD[0] + HEAD[1]) / 2, 0, (HEAD[2] + HEAD[3]) / 2), (HEAD[1] - HEAD[0], HEAD_W, HEAD[3] - HEAD[2]), DARK),
        b((-63, 0, 12.5), (111, 13, 10), GREY, tilt_y(-a)),
        b((-118, 0, 3.0), (12, 15, 6), DARK),                # rubber butt, resting on the mat
        b((-63, 0, 18.3), (60, 9, 1.0), DARK, tilt_y(-a)),    # grip inlay
    ]


def _latch(side):
    """A release latch: a tooth on the post's side face (in the socket's window when seated), a stem up inside the post,
    a bar inside the head, and a button standing out of the head's side. It slides across the handle (y); its spring
    holds it out. Pinching the head's sides pushes both buttons flush and both teeth back into the post."""
    face = POST[1] / 2 + (HOLE[1] - POST[1]) / 2           # the socket hole's side face
    y0 = face + LATCH_OUT - LATCH_HALF                     # the tooth's centre
    d = LATCH_HALF * math.sqrt(2.0)
    button = HEAD_W / 2 + BUTTON_OUT
    shapes = [
        b((0, 0, 0), (5.0, d, d), SILVER, (math.cos(math.pi / 8), math.sin(math.pi / 8), 0.0, 0.0)),
        b((0, -2.6, 1.95), (3.0, 1.0, 4.9), SILVER),                                      # stem, inside the post
        b((0, (-3.1 + button - y0) / 2, 4.4), (3.0, button - y0 + 3.1, 0.8), SILVER),      # bar, inside the head
        b((0, button - y0 - 0.65, 8.3), (10.0, 1.3, 8.0), RED),                            # the button
    ]
    if side < 0:
        shapes = [Shape(sh.kind, (sh.at[0], -sh.at[1], sh.at[2]), sh.size, sh.color, sh.tilt) for sh in shapes]
    return Joint(f"latch_{'l' if side > 0 else 'r'}", "slide", axis="y", at=(0.0, side * y0 * MM, LATCH_Z * MM),
                 low=-0.0018 if side > 0 else 0.0, high=0.0 if side > 0 else 0.0018, spring=LATCH_K,
                 target=side * LATCH_PRELOAD, damping=2.0, mass=0.0015, friction=0.3, shapes=shapes)


# -- a cartridge. Frame: origin on the mat under the socket's axis; blade unit 38 wide (y) x 14 deep (x) x 6 tall, a
# connector block with the socket above it. About 3.7 g. --------------------------------------------------------------


def _cartridge(name, mark, xy, z0=0.0, yaw=0.0):
    hx, hy = HOLE[0] / 2, HOLE[1] / 2
    ox, oy = BLOCK[0] / 2, BLOCK[1] / 2
    z0b, z1 = SOCK_Z
    wall = [
        *[b((s * (hx + BEVEL + ox) / 2, 0, (z0b + z1) / 2), (ox - hx - BEVEL, BLOCK[1], z1 - z0b), BLUE)      # x walls,
          for s in (1, -1)],                                                                                   # outer
        *[b((s * (hx + BEVEL / 2), 0, (z0b + z1 - BEVEL) / 2), (BEVEL, HOLE[1], z1 - BEVEL - z0b), BLUE)      # inner,
          for s in (1, -1)],                                                                                   # lower
        *[b((s * (hx + BEVEL / 2 + 0.1), 0, z1 - BEVEL / 2 - 0.1), (BEVEL * 1.414, HOLE[1], 0.3), BLUE,       # and the
            tilt_y(-45.0 * s)) for s in (1, -1)],                                                              # lead-in
        *[b((0, s * (hy + oy) / 2, (z0b + WINDOW[0]) / 2), (HOLE[0], oy - hy, WINDOW[0] - z0b), BLUE)          # y walls,
          for s in (1, -1)],                                                                                   # below
        *[b((0, s * (hy + YBEVEL + oy) / 2, (WINDOW[1] + z1) / 2), (HOLE[0], oy - hy - YBEVEL, z1 - WINDOW[1]),
            BLUE) for s in (1, -1)],                                                                           # above,
        *[b((0, s * (hy + YBEVEL / 2), (WINDOW[1] + z1 - YBEVEL) / 2), (HOLE[0], YBEVEL, z1 - YBEVEL - WINDOW[1]),
            BLUE) for s in (1, -1)],                                                                           # with a
        *[b((0, s * (hy + YBEVEL / 2 + 0.1), z1 - YBEVEL / 2 - 0.1), (HOLE[0], YBEVEL * 1.414, 0.3), BLUE,     # lead-in
            tilt_x(45.0 * s)) for s in (1, -1)],
        b((0, 0, z0b + FLOOR_T / 2), (HOLE[0], HOLE[1], FLOOR_T), BLUE),
    ]
    head = Joint("pivot", "hinge", axis="y", at=(0.0, 0.0, 6.25 * MM), low=-0.3, high=0.3, spring=0.03, damping=0.003,
                 mass=0.0025, friction=0.001, shapes=[
                     b((0, 0, -3.25), (14, 38, 6), CLEAR),
                     b((-1.5, 0, -6.0), (8, 36, 0.5), SILVER),                # the blades' edges, underneath
                     b((5.5, 0, -0.6), (2.5, 36, 0.8), GREEN if mark == "new" else WHITE),   # lubricating strip
                     b((-4.5, 13.0, -0.55), (3, 3, 0.9), ORANGE if mark == "old" else GREEN),  # identity mark
                 ])
    return Mech(name, pos=(xy[0], xy[1], H.MAT_TOP + z0), yaw=yaw, mass=0.0012, friction=0.5, shapes=wall,
                joints=[head])


# -- layout. The brief's x is the station's -y; its y is the station's x. Razor at the mat's origin (its middle),
# handle axis along the brief's y with the cartridge end away from the robot; dispenser at (+180, -80); tray at
# (-180, -80). ---------------------------------------------------------------------------------------------------------
RAZOR_MID = -57.5                  # handle frame x of the razor's middle (butt end at -124, head at +8)
RAZOR_XY = (H.brief_xy(0.0, 0.0)[0] - RAZOR_MID * MM, 0.0)      # where the handle frame's origin sits
HANDLE = Mech("handle", pos=(RAZOR_XY[0], RAZOR_XY[1], H.MAT_TOP), mass=HANDLE_MASS, friction=0.6,
              shapes=_handle_shapes(), joints=[_latch(1), _latch(-1)])
OLD = _cartridge("old", "old", RAZOR_XY)

DISP_XY = H.brief_xy(180.0, -80.0)
DISP_FLOOR = 4.0
POCKET = (20.0, 44.0)              # pocket inside (x, y): the blade unit can shift 3 mm either way, so the lead-in
                                   # can centre it under the post
DISP_WALL = 8.0
DISP = (40.0, 60.0)                # outside (x, y)


def _dispenser_shapes():
    px, py = POCKET[0] / 2, POCKET[1] / 2
    ox, oy = DISP[0] / 2, DISP[1] / 2
    top = DISP_FLOOR + DISP_WALL
    return [
        b((0, 0, DISP_FLOOR / 2), (DISP[0], DISP[1], DISP_FLOOR), WHITE),
        b((-(px + ox) / 2, 0, DISP_FLOOR + DISP_WALL / 2), (ox - px, DISP[1], DISP_WALL), WHITE),
        b(((px + ox) / 2, 0, DISP_FLOOR + DISP_WALL / 2), (ox - px, DISP[1], DISP_WALL), WHITE),
        *[b((0, s * (py + oy) / 2, DISP_FLOOR + DISP_WALL / 2), (POCKET[0], oy - py, DISP_WALL), WHITE)
          for s in (1, -1)],
        b((0, (POCKET[1] + DISP[1]) / 4, top + 0.25), (DISP[0] - 6, 5, 0.5), BLUE),   # label stripe, on the far wall
    ]


DISP_YAW = math.radians(120.0)   # the dispenser is turned so that a handle meeting the replacement the right way
                                 # round points its butt away from the right arm's base
DISPENSER = Mech("dispenser", pos=(DISP_XY[0], DISP_XY[1], H.MAT_TOP), yaw=DISP_YAW, mass=0.030, friction=0.6,
                 shapes=_dispenser_shapes())
NEW = _cartridge("new", "new", DISP_XY, z0=DISP_FLOOR * MM, yaw=DISP_YAW)
TRAY_XY = H.brief_xy(-180.0, -80.0)
TRAY = H.tray("tray", TRAY_XY)
OBJECTS = [H.mat(H.LIGHT_MAT), HANDLE, OLD, NEW, DISPENSER, TRAY]


# -- checks -------------------------------------------------------------------------------------------------------------

def latched(handle, cart) -> bool:
    """The cartridge's socket is on the handle's post, seated, square to it, and the latch stands out into its window:
    the connection that holds."""
    Rh, Rc = handle.rotation(), cart.rotation()
    if float(Rh[:, 2] @ Rc[:, 2]) < math.cos(math.radians(12)) or float(Rh[:, 0] @ Rc[:, 0]) < math.cos(math.radians(12)):
        return False                                       # tilted off the post, or on the wrong way round
    rel = Rh.T @ (cart.origin - handle.origin)               # the socket's axis point, in the handle's frame
    if abs(rel[0]) > 0.0012 or abs(rel[1]) > 0.0012 or abs(rel[2]) > 0.0012:
        return False
    return handle.joints.get("latch_l", 0.0) > -0.0004 and handle.joints.get("latch_r", 0.0) < 0.0004


def in_dispenser(cart, dispenser) -> bool:
    """Any part of the cartridge over the dispenser's footprint and below its walls' top: still in (or on) it."""
    pts = (H.corners(NEW, cart) - dispenser.origin) @ dispenser.rotation()
    over = (np.abs(pts[:, 0]) < DISP[0] / 2 * MM) & (np.abs(pts[:, 1]) < DISP[1] / 2 * MM)
    return bool((over & (pts[:, 2] < (DISP_FLOOR + DISP_WALL + 1.0) * MM)).any())


def apart(handle, cart, spec) -> float:
    return H.gap(HANDLE, handle, spec, cart)


S = store(KEY)
LOAD = 2.0 * HANDLE_MASS * G


def _fresh(_env=None) -> None:
    S.clear()
    S.update(fell=False, stowed_t=0.0, quiet_t=0.0, load_t=0.0, passed=False, failed=False)


_fresh()
on_reset(_fresh)


def _touched(objs, fingers, *names):
    specs = {"handle": HANDLE, "old": OLD, "new": NEW}
    return any(H.touching(specs[n], objs[n], fingers) for n in names)


@every_step(KEY, ["handle", "old", "new", "dispenser", "tray"])
def _watch(objs, fingers) -> None:
    h, old, new = objs["handle"], objs["old"], objs["new"]
    for n in ("handle", "old", "new", "dispenser", "tray"):
        if H.fallen(objs[n]):
            S["fell"] = True
    # the old cartridge in the tray, untouched, for 2 s
    ok = H.in_tray(OLD, old, TRAY, objs["tray"]) and not _touched(objs, fingers, "old") and not latched(h, old)
    S["stowed_t"] = S["stowed_t"] + 0.05 if ok else 0.0
    # the retention check: latched and untouched for 2 s, then the load for 2 s; it must stay latched throughout
    on = latched(h, new) and not in_dispenser(new, objs["dispenser"])
    if not on or _touched(objs, fingers, "handle", "new"):
        if S["load_t"] > 0.0 and not on:
            S["failed"] = True
        S["quiet_t"] = S["load_t"] = 0.0
        S["passed"] = S["passed"] and on and not _touched(objs, fingers, "handle", "new")
        return
    if S["passed"]:
        return
    if S["quiet_t"] < 2.0:
        S["quiet_t"] += 0.05
        return
    S["load_t"] += 0.05
    # pull the connector up out of the socket, hold the cartridge down: the handle's weight on the connector, doubled
    connector = h.to_world((0.0, 0.0, POST_Z[0] * MM))
    push("handle", (0.0, 0.0, LOAD), connector)
    push("new", (0.0, 0.0, -LOAD))
    if S["load_t"] >= 2.0:
        S["passed"], S["failed"] = True, False


def detached(objs, name) -> bool:
    spec = OLD if name == "old" else NEW
    return not latched(objs["handle"], objs[name]) and apart(objs["handle"], objs[name], spec) >= 0.010


def score(objs) -> float:
    if S["fell"] or any(H.fallen(objs[n]) for n in ("handle", "old", "new", "dispenser", "tray")):
        return 0.0
    h = objs["handle"]
    if latched(h, objs["old"]):
        return 0.0
    retained = S["passed"] and latched(h, objs["new"]) and not in_dispenser(objs["new"], objs["dispenser"])
    stowed = S["stowed_t"] >= 2.0
    old_off = detached(objs, "old")
    if retained:
        return 1.0 if stowed else (0.75 if old_off else 0.0)
    if stowed:
        return 0.5
    return 0.25 if old_off else 0.0
# -- scripted demo, both arms.
# Right hand: pinch the handle's head (both release buttons in) and lift the handle off the old cartridge; turn it to
# the dispenser's heading, carry it over and lower its post into the replacement's socket; let go (the latches snap out
# into the socket's windows). Take the handle again by the shaft near the head, lift the razor out of the dispenser,
# set it down on the mat and let go.
# Left hand: once the handle is off it, take the old cartridge by its connector block to the tray. --------------------

PAD_TIP = 0.0138
HOVER = 0.060                     # above the mat, for carrying
PINCH_HEAD_Z = 16.0               # fingertips' height in the handle's frame when pinching the head
SHAFT_AT, SHAFT_Z = -20.0, 13.5   # the shaft grip: along the handle, fingertips' height in its frame
SHAFT_HALF = 6.5
PLACE_XY = (0.330, -0.020)        # where the razor is set down (its head)
SQUEEZE = 0.004                   # each pad commanded this far past the surface it stops on (about 20 N)
HEAD_SQUEEZE = 0.0075             # on the head (about 37 N each, under the fingers' 50 N limit): it carries the handle
PRESS = 2.5                       # mm the post is commanded past seated (the arm gives about 1.5 mm per newton)
LIFT = 15.0                       # mm: how high the handle's head is carried; its butt stays on the mat, so the handle
                                  # cannot tip in the pinch and its post stays within 2 degrees of upright


def _hz(frame_z_mm, frame_mm):
    """A TCP height for fingertips at ``frame_mm`` in a handle frame whose origin is ``frame_z_mm`` above the mat."""
    return H.MAT_TOP + (frame_z_mm + frame_mm) * MM + PAD_TIP


def _along(xy, yaw, mm):
    return (xy[0] + mm * MM * math.cos(yaw), xy[1] + mm * MM * math.sin(yaw))


def demo():
    hx, hy = RAZOR_XY
    dx, dy = DISP_XY
    yw = DISP_YAW
    seat = DISP_FLOOR                                    # the handle frame's height when seated in the dispenser
    open_head = -(HEAD_W / 2 + BUTTON_OUT + 3.0) * MM
    pinch_head = -(HEAD_W / 2 * MM - HEAD_SQUEEZE)
    pinch_shaft = -(SHAFT_HALF * MM - SQUEEZE)
    sx, sy = _along((dx, dy), yw, SHAFT_AT)
    qx, qy = _along(PLACE_XY, yw, SHAFT_AT)
    right = [
        Waypoint(2.5, (hx, hy, HOVER + H.MAT_TOP), open_head, "over the razor's head"),
        Waypoint(2.0, (hx, hy, _hz(0.0, PINCH_HEAD_Z)), open_head, "down either side of the head"),
        Waypoint(1.0, (hx, hy, _hz(0.0, PINCH_HEAD_Z)), pinch_head, "pinch the head: both release buttons in"),
        Waypoint(2.0, (hx, hy, _hz(LIFT, PINCH_HEAD_Z)), pinch_head, "lift the handle off the old cartridge"),
        Waypoint(2.5, (hx, hy, _hz(LIFT, PINCH_HEAD_Z)), pinch_head, "turn it to the dispenser's heading", yw),
        Waypoint(4.0, (dx, dy, _hz(seat + LIFT, PINCH_HEAD_Z)), pinch_head, "over the dispenser", yw),
        Waypoint(1.5, (dx, dy, _hz(seat + 12.0, PINCH_HEAD_Z)), pinch_head, "down to just above the socket", yw),
        Waypoint(1.0, (dx, dy, _hz(seat + 12.0, PINCH_HEAD_Z)), pinch_head, "settle", yw),
        Waypoint(2.0, (dx, dy, _hz(seat + 4.0, PINCH_HEAD_Z)), pinch_head, "post down into the socket", yw),
        Waypoint(1.5, (dx, dy, _hz(seat - PRESS, PINCH_HEAD_Z)), pinch_head, "press the post home", yw),
        Waypoint(1.0, (dx, dy, _hz(seat - PRESS, PINCH_HEAD_Z)), open_head, "let go: the latches snap out", yw),
        Waypoint(1.0, (dx, dy, _hz(seat + 25.0, PINCH_HEAD_Z)), open_head, "up", yw),
        Waypoint(1.5, (sx, sy, _hz(seat + 25.0, SHAFT_Z)), open_head, "over the shaft", yw),
        Waypoint(1.5, (sx, sy, _hz(seat, SHAFT_Z)), open_head, "down either side of the shaft", yw),
        Waypoint(1.0, (sx, sy, _hz(seat, SHAFT_Z)), pinch_shaft, "grip the shaft", yw),
        Waypoint(2.0, (sx, sy, _hz(seat + 25.0, SHAFT_Z)), pinch_shaft, "lift the razor out of the dispenser", yw),
        Waypoint(4.0, (qx, qy, _hz(25.0, SHAFT_Z)), pinch_shaft, "over a clear part of the mat", yw),
        Waypoint(2.0, (qx, qy, _hz(1.0, SHAFT_Z)), pinch_shaft, "set the razor down", yw),
        Waypoint(1.0, (qx, qy, _hz(1.0, SHAFT_Z)), open_head, "let go", yw),
        Waypoint(1.5, (qx, qy, _hz(45.0, SHAFT_Z)), open_head, "up and away", yw),
        Waypoint(5.0, (qx, qy, _hz(45.0, SHAFT_Z)), open_head, "wait through the retention check", yw),
    ]
    # the left hand: the old cartridge's connector block, pinched across, to the tray
    tx, ty = TRAY_XY
    block_half = BLOCK[1] / 2 * MM
    open_l, pinch_l = -(block_half + 0.006), -(block_half - SQUEEZE)
    grip_z = 11.0                    # fingertips on the block's sides, clear of the blade unit as it pivots (2.1 mm)
    drop = H.TRAY_FLOOR_T / MM + 3.0
    left = [
        Waypoint(11.5, (hx - 0.06, hy + 0.15, HOVER + H.MAT_TOP), open_l, "wait for the handle to be lifted away"),
        Waypoint(2.0, (hx, hy, HOVER + H.MAT_TOP), open_l, "over the old cartridge"),
        Waypoint(1.5, (hx, hy, _hz(0.0, grip_z)), open_l, "down either side of its connector block"),
        Waypoint(1.0, (hx, hy, _hz(0.0, grip_z)), pinch_l, "pinch the block"),
        Waypoint(1.5, (hx, hy, _hz(40.0, grip_z)), pinch_l, "lift it"),
        Waypoint(3.0, (tx, ty, _hz(40.0, grip_z)), pinch_l, "over the tray"),
        Waypoint(1.5, (tx, ty, _hz(drop, grip_z)), pinch_l, "lower into the tray"),
        Waypoint(1.0, (tx, ty, _hz(drop, grip_z)), open_l, "let go"),
        Waypoint(1.5, (tx, ty, HOVER + H.MAT_TOP), open_l, "up and away"),
    ]
    return {"left": left, "right": right}


# -- counterexamples (check_demo.py --counter NAME): plans that must not earn full marks ------------------------------

def _cut(plan, label):
    return plan[:next(i for i, w in enumerate(plan) if w.label.startswith(label)) + 1]


def _pull_without_release():
    """The handle taken by the shaft and lifted without pressing the buttons: the old cartridge comes with it."""
    hx, hy = RAZOR_XY
    sx, sy = _along((hx, hy), 0.0, SHAFT_AT)
    open_w, pinch = -(SHAFT_HALF * MM + 0.012), -(SHAFT_HALF * MM - SQUEEZE)
    right = [
        Waypoint(2.5, (sx, sy, HOVER + H.MAT_TOP), open_w, "over the shaft"),
        Waypoint(2.0, (sx, sy, _hz(0.0, SHAFT_Z)), open_w, "down either side of the shaft"),
        Waypoint(1.0, (sx, sy, _hz(0.0, SHAFT_Z)), pinch, "grip the shaft"),
        Waypoint(2.0, (sx, sy, _hz(30.0, SHAFT_Z)), pinch, "pull the razor up"),
        Waypoint(6.0, (sx, sy, _hz(30.0, SHAFT_Z)), pinch, "hold it"),
    ]
    return {"left": [Waypoint(14.0, demo()["left"][0].pos, demo()["left"][0].finger, "wait")], "right": right}


def _old_left_on_mat():
    """The replacement fitted and retained, but the old cartridge left where it came off."""
    plan = demo()
    w = plan["left"][0]
    return {"left": [Waypoint(40.0, w.pos, w.finger, "wait")], "right": plan["right"]}


def _leave_in_dispenser():
    """The old cartridge stowed and the handle fitted to the replacement, but the razor left in the dispenser."""
    plan = demo()
    r = _cut(plan["right"], "up")
    return {"left": plan["left"], "right": r + [Waypoint(14.0, r[-1].pos, r[-1].finger, "wait", r[-1].yaw)]}


def _keep_holding():
    """Everything done, but the razor is still held at the end: the retention check never starts."""
    plan = demo()
    r = _cut(plan["right"], "set the razor down")
    return {"left": plan["left"], "right": r + [Waypoint(10.0, r[-1].pos, r[-1].finger, "keep holding", r[-1].yaw)]}


COUNTEREXAMPLES = {
    "pull_without_release": (_pull_without_release, 0.0, "the latches hold: the old cartridge stays on the handle"),
    "old_left_on_mat": (_old_left_on_mat, 0.75, "replacement retained, old cartridge detached but not stowed"),
    "leave_in_dispenser": (_leave_in_dispenser, 0.5, "old cartridge stowed; the replacement only sits in the dispenser"),
    "keep_holding": (_keep_holding, 0.5, "old cartridge stowed; the razor is never released, so nothing is retained"),
}
