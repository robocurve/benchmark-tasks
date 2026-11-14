"""On the station table a framed picture lies face down on two carpet-covered blocks, ready to be packed for a move:
an oak frame 30 cm front to back and 40 cm across, 2.5 cm thick, with its glass, mount and hardboard back. The blocks
run front to back under its two 30 cm sides, so all four corners stick out past them. Its two right-hand corners
already have white polyethylene-foam corner protectors on them; the other two protectors lie on the table beside the
two bare left-hand corners, each already turned to face its corner.

A protector is a foam block 7 cm square and 4.5 cm tall with an L-shaped pocket cut into two adjacent sides: 4.5 cm
deep each way and as tall as the frame is thick. Its foam squeezes the frame, so it has to be pushed onto the corner,
level with the frame, until the corner reaches the back of the pocket. Pushed on, it grips the corner by itself.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import _SPECS, _part_state, Slider, on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402


SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Fit the two loose foam corner protectors onto the picture frame's two bare corners, then let go of them."

OAK = (0.62, 0.45, 0.27)
KRAFT = (0.55, 0.42, 0.30)
CARPET = (0.32, 0.33, 0.36)
FOAM = (0.93, 0.94, 0.95)

# -- the padded blocks: two carpet-covered pine 2x4 offcuts, fixed, under the frame's 30 cm sides ---------------------
BLOCK = (0.160, 0.089, 0.038)  # x (front to back), y, height
BLOCK_X = 0.30
BLOCK_Y = 0.185  # centred under each 30 cm side's moulding

# -- the picture frame, face down; origin at the middle of its front (the face on the blocks) --------------------------
FRAME_AT = (0.30, 0.0)
OUT = (0.30, 0.40)  # outside, x by y
MOULD_W, THICK = 0.030, 0.025  # moulding width; the frame's thickness, front to back
SLAB_Z = (0.004, 0.020)  # glass, mount, picture and hardboard back between the moulding's front lip and its back
FRAME_MASS = 1.1
FB = TOP + BLOCK[2]  # the frame's front (its underside) rests on the blocks at this height
HALF = (OUT[0] / 2, OUT[1] / 2)


def _frame_boxes():
    hx, hy = HALF
    w, t = MOULD_W, THICK
    rails = [((0.0, s * (hy - w / 2), t / 2), (OUT[0], w, t), OAK) for s in (-1, 1)]
    rails += [((s * (hx - w / 2), 0.0, t / 2), (w, OUT[1] - 2 * w, t), OAK) for s in (-1, 1)]
    slab = [((0.0, 0.0, sum(SLAB_Z) / 2), (OUT[0] - 2 * w + 0.002, OUT[1] - 2 * w + 0.002, SLAB_Z[1] - SLAB_Z[0]),
             KRAFT)]
    return rails + slab

# -- a foam corner protector, in its own frame: the origin is where the frame's corner sits, on the frame's front plane;
# the pocket opens toward -x and -y and the two closed walls are on +x and +y -----------------------------------------
POCKET = 0.045  # how far the corner goes in, each way
WALL = 0.025  # closed walls' thickness: the block is POCKET + WALL = 7 cm square
LIP = 0.010  # foam under and over the pocket
CLEAR = 0.0015  # the pocket's roof sits this far above the frame's back with the floor pressed down
PROUD = 0.0020  # unpressed, the foam floor stands this far above the frame's front: it squeezes the frame 0.5 mm
LEAD_R = 0.0050  # the open edges' rounded lips: spheres of this radius, crest level with the floor and roof
LEAD_PITCH = 0.004  # ... spaced this far apart along each edge
FLOOR_T = 0.006  # the sprung floor's thickness
ROOF = THICK + CLEAR
FLOOR_RATE = 2500.0  # N/m: about 1.25 N of squeeze at 0.5 mm, six times the protector's weight
FLOOR_TRAVEL = 0.005
PROTECTOR_MASS = 0.020  # cross-linked PE foam, about 100 kg/m3
FOAM_FRICTION = 0.3


def _protector_shapes():
    """The protector's colliders: (boxes, spheres) of its body and (boxes, spheres) of its sprung floor, all in its own
    frame at rest. The pocket's two open edges have rounded lips: a row of spheres along each edge on the floor and
    under the roof, so a frame edge meeting them rides in and presses the floor down. (Spheres, not turned boxes:
    PhysX builds turned boxes differently from how USD draws them, which closed the pocket's mouth on Brev.)"""
    p, w, lip, r = POCKET, WALL, LIP, LEAD_R
    z0, z1 = -lip, ROOF + lip
    inner = p - r  # the flat floor and roof reach this far from the back, to the spheres' crests
    walls = [
        ((w / 2, (w - p) / 2, (z0 + z1) / 2), (w, p + w, z1 - z0), FOAM),  # wall on +x, full length
        ((-p / 2, w / 2, (z0 + z1) / 2), (p, w, z1 - z0), FOAM),  # wall on +y
    ]
    roof = [
        ((-inner / 2, -inner / 2, ROOF + lip / 2), (inner, inner, lip), FOAM),  # the roof over the flat
        ((-p + r / 2, -p / 2, ROOF + r + (lip - r) / 2), (r, p, lip - r), FOAM),  # skin over the -x lip
        ((-p / 2, -p + r / 2, ROOF + r + (lip - r) / 2), (p, r, lip - r), FOAM),  # skin over the -y lip
    ]
    floor_boxes = [((-inner / 2, -inner / 2, PROUD - FLOOR_T / 2), (inner, inner, FLOOR_T), FOAM)]
    run = p - r  # along each edge the spheres stay inside the block: none sticks out past the other open face
    n = math.ceil(run / LEAD_PITCH) + 1
    along = [-p + r + k * run / (n - 1) for k in range(n)]
    roof_sph, floor_sph = [], []
    for a in along:
        for xy in ((-p + r, a), (a, -p + r)):  # the -x edge's row, then the -y edge's
            roof_sph.append(((xy[0], xy[1], ROOF + r), r, FOAM))
            floor_sph.append(((xy[0], xy[1], PROUD - r), r, FOAM))
    return (walls + roof, roof_sph), (floor_boxes, floor_sph)


# corner name -> (sign along x, sign along y): the frame's corner at (sx * 15, sy * 20) cm from its middle
ALL_CORNERS = {"far_left": (1, 1), "near_left": (-1, 1), "far_right": (1, -1), "near_right": (-1, -1)}
CORNERS = {n: ALL_CORNERS[n] for n in ("far_left", "near_left")}  # bare at the start: the task fits these two
FITTED = ("far_right", "near_right")  # already have a protector on at the start (part of the frame's rigid body)
# the protector's yaw that turns its pocket (open toward its -x, -y) onto that corner
YAW = {"far_left": 0.0, "near_left": math.pi / 2, "far_right": -math.pi / 2, "near_right": math.pi}
START_OUT = 0.100  # each protector starts on the table this far out sideways (y) from its seated place


def _rot(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def corner_xy(name: str) -> np.ndarray:
    sx, sy = ALL_CORNERS[name]
    return np.array([FRAME_AT[0] + sx * HALF[0], FRAME_AT[1] + sy * HALF[1]])


def start_xy(name: str) -> np.ndarray:
    return corner_xy(name) + np.array([0.0, CORNERS[name][1] * START_OUT])


def _protector(name: str):
    (shapes, sph), (floor, floor_sph) = _protector_shapes()
    x, y = (float(v) for v in start_xy(name))
    z = TOP + LIP + 0.0005
    slider = Slider(f"floor_{name}", rest=(0.0, 0.0, PROUD), boxes=tuple(floor), spheres=tuple(floor_sph),
                    travel=FLOOR_TRAVEL, rate=FLOOR_RATE, damping=8.0, mass=0.004, friction=FOAM_FRICTION)
    return part(f"protector_{name}", pos=(x, y, z), boxes=shapes, spheres=sph, mass=PROTECTOR_MASS,
                yaw=YAW[name], color=FOAM, friction=FOAM_FRICTION, sliders=(slider,))


PROTECTORS = {n: _protector(n) for n in CORNERS}


def _fitted_boxes(name: str) -> list:
    """A protector already pushed home on corner ``name``, as boxes in the frame's own frame (it moves with the frame):
    the two closed walls, the roof and the floor pressed flat against the frame's front. Turned by multiples of 90
    degrees, so every box stays unturned (sizes swapped where needed)."""
    p, w, lip = POCKET, WALL, LIP
    z0, z1 = -lip, ROOF + lip
    local = [
        ((w / 2, (w - p) / 2, (z0 + z1) / 2), (w, p + w, z1 - z0)),  # wall on +x
        ((-p / 2, w / 2, (z0 + z1) / 2), (p, w, z1 - z0)),  # wall on +y
        ((-p / 2, -p / 2, ROOF + lip / 2), (p, p, lip)),  # roof
        ((-p / 2, -p / 2, -lip / 2), (p, p, lip)),  # floor, pressed flat
    ]
    sx, sy = ALL_CORNERS[name]
    c, s = round(math.cos(YAW[name])), round(math.sin(YAW[name]))
    out = []
    for (cx, cy, cz), (lx, ly, lz) in local:
        centre = (sx * HALF[0] + c * cx - s * cy, sy * HALF[1] + s * cx + c * cy, cz)
        size = (lx, ly, lz) if c != 0 else (ly, lx, lz)
        out.append((centre, size, FOAM))
    return out


FRAME = part("frame", pos=(FRAME_AT[0], FRAME_AT[1], FB + 0.0005), mass=FRAME_MASS + len(FITTED) * PROTECTOR_MASS,
             color=OAK, friction=0.6, boxes=_frame_boxes() + [b for n in FITTED for b in _fitted_boxes(n)])

OBJECTS = [
    *[Box(f"block_{side}", pos=(BLOCK_X, s * BLOCK_Y, TOP + BLOCK[2] / 2), size=BLOCK, color=CARPET)
      for side, s in (("left", 1), ("right", -1))],
    FRAME,
    *PROTECTORS.values(),
]

# -- scoring -----------------------------------------------------------------------------------------------------------
IN_TOL = 0.004  # the corner within 4 mm of the back of the pocket, both ways
OVER_TOL = 0.002  # (the corner can't go into the wall: allow for contact)
Z_TOL = 0.003  # the frame's front within 3 mm of the pocket floor's line
TURN_TOL = 6.0  # degrees: the protector square to the corner
TILT_TOL = 6.0  # degrees: level with the frame
SQUEEZE = 0.0003  # the floor pressed down at least this far: the foam grips the frame
LET_GO = 0.040  # each arm's TCP at least this far from the protector's block: the finger surfaces reach about 24 mm
#                 from the TCP (measured on Brev, fingers pressing the outer face), so this leaves them about 1.5 cm clear
DROPPED_TILT = 15.0  # the frame ever tilted more than this, or fell off a block: dropped
FLAT_TILT = 3.0  # at the end the frame lies flat on both blocks
OFF_BLOCK = 0.015  # its front this far below the blocks' top: it slid off a block


def _fresh() -> dict:
    return {"dropped": False, "steps": 0, "tcps": []}


_S = _fresh()
_DEBUG = bool(os.environ.get("CORNERS_DEBUG"))


def _tilt_deg(state) -> float:
    z = state.rotation()[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    c = (np.trace(a.T @ b) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, float(c)))))


def _rz(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def corner_world(frame, name: str) -> np.ndarray:
    sx, sy = ALL_CORNERS[name]
    return frame.to_world((sx * HALF[0], sy * HALF[1], 0.0))


def protector_middle(prot) -> np.ndarray:
    return prot.to_world(((WALL - POCKET) / 2, (WALL - POCKET) / 2, ROOF / 2))


def squeeze(prot) -> float:
    """How far the protector's foam floor is pressed down (m, positive)."""
    return -min(0.0, float(next(iter(prot.sliders.values()), 0.0))) if prot.sliders else 0.0


def seated_on(frame, prot, name: str) -> bool:
    """The protector is fully on corner ``name``: the corner at the back of its pocket, square and level with the
    frame, and the foam floor pressed down so it grips."""
    local = prot.to_local(corner_world(frame, name))
    if not (-IN_TOL <= local[0] <= OVER_TOL and -IN_TOL <= local[1] <= OVER_TOL and abs(local[2]) <= Z_TOL):
        return False
    want = frame.rotation() @ _rz(YAW[name])
    return _angle_deg(want, prot.rotation()) <= max(TURN_TOL, TILT_TOL) and squeeze(prot) >= SQUEEZE


BLOCK_LO = np.array([-POCKET, -POCKET, -LIP])  # the protector's outer block, in its own frame
BLOCK_HI = np.array([WALL, WALL, ROOF + LIP])


def block_distance(prot, point) -> float:
    """How far ``point`` (world) is from the protector's outer block (0 inside it)."""
    local = prot.to_local(np.asarray(point, float))
    return float(np.linalg.norm(np.maximum(0.0, np.maximum(BLOCK_LO - local, local - BLOCK_HI))))


def released(prot) -> bool:
    """Let go: each arm's TCP at least LET_GO from the protector's block. (A TCP-to-middle distance let fingers still
    pressing the outer face count as let go on Brev.)"""
    return bool(_S["tcps"]) and all(block_distance(prot, t) >= LET_GO for t in _S["tcps"])


def frame_flat(frame) -> bool:
    return _tilt_deg(frame) <= FLAT_TILT and float(frame.pos[2]) > FB - OFF_BLOCK


def protected(objects) -> list[str]:
    """The corners that have a protector seated on them, let go."""
    frame = objects[FRAME.name]
    out = []
    for name in CORNERS:
        for spec in PROTECTORS.values():
            prot = objects[spec.name]
            if seated_on(frame, prot, name) and released(prot):
                out.append(name)
                break
    return out


def grippers(environment) -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads, 13 mm above the pad tips, from each gripper
    link's pose as yam_arena.runtime.tcp_world computes it)."""
    from yam_arena.runtime import tcp_world

    base = environment.unwrapped
    origin = _np_vec(base.scene.env_origins[0])
    return [tcp_world(base.scene[key]) - origin for key in ("left_robot", "right_robot")]


def _np_vec(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


@on_reset
def _reset(environment=None):
    _S.clear()
    _S.update(_fresh())


@on_step
def _follow(environment) -> None:
    _S["steps"] += 1
    frame = _part_state(environment, _SPECS[FRAME.name])
    if _tilt_deg(frame) > DROPPED_TILT or float(frame.pos[2]) < FB - OFF_BLOCK:
        _S["dropped"] = True
    try:
        _S["tcps"] = grippers(environment)
    except (KeyError, ValueError, AttributeError):
        _S["tcps"] = []
    if _DEBUG and _S["steps"] % 5 == 0:
        parts = []
        for name, spec in PROTECTORS.items():
            prot = _part_state(environment, _SPECS[spec.name])
            local = prot.to_local(corner_world(frame, name))
            r = prot.rotation()
            ypr = (math.degrees(math.atan2(r[1, 0], r[0, 0])), math.degrees(math.asin(-max(-1.0, min(1.0, r[2, 0])))),
                   math.degrees(math.atan2(r[2, 1], r[2, 2])))
            parts.append(f"{name}: corner {np.round(local * 1000, 1).tolist()}mm squeeze {squeeze(prot) * 1000:.2f}mm "
                         f"pos {np.round(prot.pos, 4).tolist()} ypr {np.round(ypr, 1).tolist()} "
                         f"seated {seated_on(frame, prot, name)}")
        print(f"[corners] step {_S['steps']} frame {np.round(frame.pos, 4)} tilt {_tilt_deg(frame):.1f} "
              f"dropped {_S['dropped']} tcps {[np.round(t, 3).tolist() for t in _S['tcps']]} | "
              + " | ".join(parts), flush=True)


def score(objects) -> float:
    frame = objects[FRAME.name]
    if _S["dropped"] or not frame_flat(frame):
        return 0.0
    n = len(protected(objects))
    if n < len(CORNERS):
        return 0.5 * n
    still = frame.resting() and all(objects[s.name].resting() for s in PROTECTORS.values())
    return 1.0 if still else 0.75


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- the scripted demo -------------------------------------------------------------------------------------------------
# Each arm fits the two protectors on its own side while the other arm holds the frame down on its back. A protector is
# pinched across its closed wall (fingers closing along world y), carried to the frame's height beside its corner, and
# pushed straight on sideways (along y); then the gripper opens and backs away.

TIP = 0.013  # pad tips below the TCP
FINGER_HALF_W = 0.0137  # each finger pad is 27.4 mm wide (across the pinch); 40 mm wide 3 cm up the finger
GRIP_YAW = 0.0  # the fingers close along world y, the push's direction, so the pads hold the protector against the
#               roll a push puts on it (closing along x, it pivoted about that axis between the pads)
GRIP_SIDE = 0.023  # the pads' middle this far out (in x) past the corner, on the closed wall beyond the frame's end
# In this sim only one finger closes: the other stays at its open stop, STILL_PAD from the TCP along STILL_DIR (toward
# -y at GRIP_YAW 0; measured on Brev), and the closing finger shoves the part against it. So a held protector's middle
# sits STILL_PAD - its half width (35 mm) along STILL_DIR from the TCP, and the pick aims the still finger clear of it.
STILL_PAD = 0.049
STILL_DIR = np.array([math.sin(GRIP_YAW), -math.cos(GRIP_YAW)])
HALF_BLOCK = (POCKET + WALL) / 2
GRASP_OFF = tuple((STILL_PAD - HALF_BLOCK) * STILL_DIR)
PICK_CLEAR = 0.012  # the still finger comes down this far off the block (closer, it catches the top edge and tips
#                     it); the closing finger then slides the block flat along the table against it
PINCH_UP = 0.010  # TCP above the protector's origin while pinching it: the pads span about the middle of the pocket, so the
#                    push acts near the middle of the frame's thickness and doesn't roll the wrist
HOVER = TOP + 0.10
APPROACH = 0.050  # start the push this far out from seated, along y
X_CLEAR = 0.004  # ... and this far out along x, so the closed wall clears the frame's end ...
RELEASE_SHIFT = 0.006
LIFT_CLEAR = 0.010  # after letting go, lift until the pad tips are this far above the protector's top
PRESS_IN = 0.030  # the closed finger presses the outer face this far in from the corner (the still finger 49 mm out)
PRESS_START = 0.060  # the fingers come down this far beyond the seated face (clear of a protector still well out)
PRESS_OVER = 0.010  # ... and the press aims this far past seated: the back of the pocket stops it on the corner
PRESS_BACK = 0.030
PRESS_X = 0.010  # the press also aims this far inward along x, so the fingers' friction slides the protector
#                  into the corner along the frame's end (on Brev it stayed 4.6 mm out, from the approach's clearance)
MID_ON = 0.0  # the first push aims at seated: the arm lags well behind against the foam and stops partway on
X_IN = 0.004  # ... and this far into the frame's end along x (the far arm drifts outward under the push)
Z_AIM = -0.0003  # the protector's origin this far below the frame's front: the middle of the bevels' window
PRESS = 0.012  # the holding arm's closed fingertips aimed this far below the back's surface
HOLD_Y = 0.10  # the holding arm presses on the back this far either side of the middle
# the loaded arm misses its commanded pose by a pose-dependent offset (x, y, z); the demo aims the other way. Tune on
# Brev with CORNERS_SAG="dx,dy,dz".
SAG = tuple(float(v) for v in os.environ.get("CORNERS_SAG", "0,0,0").split(","))


def grip_world(name: str, origin_xy) -> np.ndarray:
    """Where the pinch closes on the protector (world xy) when its origin (the back of its pocket) is at ``origin_xy``:
    across its full width along y, on the closed wall beyond the frame's end, GRIP_SIDE out past the corner."""
    sx, sy = CORNERS[name]
    return np.asarray(origin_xy, float) + np.array([sx * GRIP_SIDE, -sy * (POCKET - WALL) / 2])


def tcp_for(grip_xy) -> np.ndarray:
    """The TCP aim that closes the pinch on ``grip_xy``."""
    return np.asarray(grip_xy, float) - np.array(GRASP_OFF)


def _fit(name: str) -> list[Waypoint]:
    """Pick the protector by its closed end wall, bring it level beside the corner and push it partway on (over the
    lip, where the squeeze holds it), let go, then press it home with both closed fingers flat on its closed outer
    face. (Pinching it all the way home failed on Brev: under the push the arm drifts along x and the pads, which
    can only hold the 25 mm end wall, slid off it. Beyond the outer face there is no frame, so the fingers can press
    across the face's middle, and the back of the pocket stops the protector on the corner.)"""
    sx, sy = CORNERS[name]
    seat = corner_xy(name)
    g_start = grip_world(name, start_xy(name)) - (STILL_PAD - HALF_BLOCK - PICK_CLEAR) * STILL_DIR
    g_pre = tcp_for(grip_world(name, seat + np.array([sx * X_CLEAR, sy * APPROACH])))
    g_mid = tcp_for(grip_world(name, seat + np.array([-sx * X_IN, sy * MID_ON])))  # into the frame's end: the wall slides along it
    z_pick = TOP + LIP + 0.0005 + PINCH_UP
    z_fit = FB + Z_AIM + PINCH_UP + SAG[2]
    z_clear = FB - CLEAR + ROOF + LIP + TIP + LIFT_CLEAR  # pad tips LIFT_CLEAR above the seated protector's top
    dx, dy = SAG[0], SAG[1]

    def at(g, z):
        return (float(g[0] + dx), float(g[1] + dy), float(z))

    # the press: wrist turned so the fingers close along x, the still finger outward; the closed finger (at the TCP)
    # and the still finger both bear on the outer face, straddling its middle
    press_yaw = sx * math.pi / 2
    px = seat[0] - sx * PRESS_IN  # the closed finger's x; the still finger is STILL_PAD further out
    face = seat[1] + sy * WALL  # the outer face's y once seated
    p_start = np.array([px, face + sy * (FINGER_HALF_W + PRESS_START)])
    p_home = np.array([px - sx * PRESS_X, face + sy * (FINGER_HALF_W - PRESS_OVER)])  # and inward: drags it into the corner
    p_back = np.array([px - sx * PRESS_X, face + sy * (FINGER_HALF_W + PRESS_BACK)])
    g_off = g_mid + RELEASE_SHIFT * STILL_DIR
    y = GRIP_YAW
    return [
        Waypoint(1.8, (float(g_start[0]), float(g_start[1]), HOVER), FINGER_OPEN, f"{name}: above the protector", y),
        Waypoint(1.4, (float(g_start[0]), float(g_start[1]), z_pick), FINGER_OPEN, f"{name}: down round it", y),
        Waypoint(0.8, (float(g_start[0]), float(g_start[1]), z_pick), FINGER_CLOSED, f"{name}: pinch", y),
        Waypoint(1.0, (float(g_start[0]), float(g_start[1]), z_fit + 0.03), FINGER_CLOSED, f"{name}: lift", y),
        Waypoint(1.4, at(g_pre, z_fit + 0.03), FINGER_CLOSED, f"{name}: beside the corner", y),
        Waypoint(1.0, at(g_pre, z_fit), FINGER_CLOSED, f"{name}: down to the frame's height", y),
        Waypoint(0.5, at(g_pre, z_fit), FINGER_CLOSED, f"{name}: steady", y),
        Waypoint(3.0, at(g_mid, z_fit), FINGER_CLOSED, f"{name}: push it on", y),
        Waypoint(0.5, at(g_mid, z_fit), FINGER_CLOSED, f"{name}: settle on the lip", y),
        # open where the push stopped: easing the arm back first released its sideways deflection, which twisted the
        # protector on Brev, and pulled it back off the lip
        Waypoint(0.8, at(g_mid, z_fit), FINGER_OPEN, f"{name}: let go", y),
        Waypoint(0.6, at(g_off, z_fit), FINGER_OPEN, f"{name}: still finger off it", y),
        Waypoint(1.2, at(g_off, z_clear), FINGER_OPEN, f"{name}: up", y),
        Waypoint(0.6, at(g_off, z_clear), FINGER_CLOSED, f"{name}: close the fingers", y),
        Waypoint(1.2, at(p_start, z_clear), FINGER_CLOSED, f"{name}: turn the wrist, behind its outer face",
                 press_yaw),
        Waypoint(0.8, at(p_start, z_fit), FINGER_CLOSED, f"{name}: down beside the face", press_yaw),
        Waypoint(3.5, at(p_home, z_fit), FINGER_CLOSED, f"{name}: push it home with the fingers", press_yaw),
        Waypoint(0.6, at(p_home, z_fit), FINGER_CLOSED, f"{name}: settle", press_yaw),
        Waypoint(0.8, at(p_back, z_fit), FINGER_CLOSED, f"{name}: back off", press_yaw),
        Waypoint(1.0, at(p_back, HOVER), FINGER_OPEN, f"{name}: lift away", press_yaw),
    ]


def _hold(side: str, duration: float) -> list[Waypoint]:
    """Closed fingers pressed down on the frame's back, beside the middle on ``side``, for ``duration`` s in all."""
    y = HOLD_Y if side == "left" else -HOLD_Y
    x = FRAME_AT[0]
    z_back = FB + SLAB_Z[1]
    z = z_back + TIP - PRESS
    down, up = 1.4, 0.8
    return [
        Waypoint(1.6, (x, y, HOVER), FINGER_CLOSED, f"{side}: above the frame's back"),
        Waypoint(down, (x, y, z), FINGER_CLOSED, f"{side}: press the frame down"),
        Waypoint(max(0.1, duration - 1.6 - down - up), (x, y, z), FINGER_CLOSED, f"{side}: hold it"),
        Waypoint(up, (x, y, HOVER), FINGER_CLOSED, f"{side}: up off the frame"),
    ]


def _wait(duration: float, pos, finger=FINGER_OPEN, label="wait") -> list[Waypoint]:
    return [Waypoint(duration, pos, finger, label)] if duration > 0 else []


def _length(wps) -> float:
    return sum(w.duration for w in wps)


PARK = {"left": (0.18, 0.34, TOP + 0.16), "right": (0.18, -0.34, TOP + 0.16)}


def demo():
    """The left arm fits the two loose protectors (far, then near) while the right arm holds the frame down on its
    back beside the middle."""
    variant = os.environ.get("DEMO_VARIANT", "full")
    if variant == "nothing":
        return {"left": _wait(3.0, PARK["left"]), "right": _wait(3.0, PARK["right"])}
    fit = _fit("far_left") + _fit("near_left")
    if variant == "one_only":  # the far corner only
        fit = _fit("far_left")
    if variant == "near_only":  # Brev probe: the near corner only
        fit = _fit("near_left")
    left = fit + [Waypoint(1.5, PARK["left"], FINGER_OPEN, "park")]
    if variant == "keep_holding":  # both fitted, but the fingers stay pressed on the second one to the end
        fit = fit[:-2] + [Waypoint(60.0, fit[-3].pos, FINGER_CLOSED, "keep pressing", fit[-3].yaw)]
        left = fit
    if variant == "no_hold":  # nobody holds the frame
        return {"left": left, "right": _wait(_length(left), PARK["right"])}
    right = _hold("right", _length(fit) + 0.5) + [Waypoint(1.5, PARK["right"], FINGER_OPEN, "park")]
    return {"left": left, "right": right}
