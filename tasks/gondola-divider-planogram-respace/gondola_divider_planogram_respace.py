"""Re-space a gondola shelf's wire dividers to a new planogram: end state = the middle lane is 6 holes wide (lanes
7-6-7), each moved divider with its rear foot down in the planogram hole and its front hook over the fence top wire,
the boxed product in the outer lanes still standing in its lane, and both grippers off the shelf.

Real product (catalog numbers; README provenance): a 36 x 13 in gondola shelf with two rows of 1/4 in (6.35 mm)
perforations on 1 in (25.4 mm) centres, 3 in standard wire front fencing (vertical wires on 1 in centres) and
standard wire dividers for a 13 in shelf, about 2 3/4 in shorter than the nominal shelf depth (260 mm), 3 in high.
Wire diameters (4 mm divider, 4.5 mm fence top wire), foot / hook leg length (10 / 14 mm) and divider mass (90 g, from that geometry)
are estimates (SCENE.md states the method and uncertainty).

Bench layout in station coordinates (TOP = station.TABLE_TOP, +X forward, +Y left; arm bases at y = +-0.305 m):
- shelf (fixed): 914 x 330 mm shelf deck on two bench stands, deck top at SHELF_Z; fence top wire along y at
  x = X_FENCE; rear perforation row at x = X_HOLE, holes at y = k * PITCH (k = -17..17). Two outer dividers are part
  of the fixed shelf at holes +-10 (they bound the planogram section and are not moved).
- divider_l / divider_r: the middle lane's two wire dividers, each one rigid wire frame. The frame rides a
  six-axis joint chain (x, y, z slides, then yaw, roll, pitch hinges about its centre, no springs, no drives, no
  joint friction): mechanically a loose rigid body, but it starts exactly seated in its hole (a seated divider has no
  position scatter, and the runtime's reset jitter would otherwise push a seated foot into the deck).
- glove_l / glove_r: a box of nitrile exam gloves lying in each outer lane.
- visual-only bench props (labkit.SCENERY): no collider, no mass, outside the robot's path.

Run it:
  inspect-robots run --embodiment yam_arena -E task=gondola_divider_planogram_respace.py -E scene=station \\
    -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<see task.json>" ...
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import labkit  # noqa: E402
from labkit import Joint, PartState, on_reset, on_step, part  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402

# -- geometry (single source of truth for score(), success(), demo()) ---------------------------------------------

SCENE = "station"
TOP = station.TABLE_TOP

PITCH = 0.0254  # perforations and fence vertical wires on 1 in centres
HOLE_HALF = 0.00635 / 2  # 1/4 in perforation (collider: 6.35 mm square opening)
N_HOLES = 17  # holes k = -17..17 along the 36 in deck
SHELF_Z = 0.015  # deck top above the bench (bench stands)
DECK_T = 0.004  # collider thickness of the deck (sheet steel ~1.2 mm, thickened for the solver)
X_FRONT, X_BACK = 0.115, 0.445  # 13 in deck, front (bullnose) to back
SHELF_HALF_LEN = 0.457  # 36 in
X_FENCE = 0.130  # fence plane (front perforations)
X_HOLE = 0.390  # rear perforation row (divider length 260 mm behind the fence)
FENCE_H = 0.0762  # 3 in fencing
FENCE_R = 0.00225  # top wire radius (4.5 mm wire, estimate)
FENCE_ZC = FENCE_H - FENCE_R  # top wire centre above the deck

WIRE_R = 0.002  # divider wire radius (4 mm wire, estimate)
X_POST = X_FENCE + 0.006  # front post just behind the fence wire
X_HOOK = X_FENCE - 0.006  # hook leg just in front of it
RAIL_Z = FENCE_ZC + FENCE_R + WIRE_R + 0.0005  # top rail runs forward over the fence top wire
HOOK_LEG = 0.014  # hook leg length below the rail centre (longer than the foot: hooked with the foot on the rim)
FOOT = 0.010  # rear foot below the deck top
STAYS = (X_POST + 0.085, X_HOLE - 0.085)  # two vertical stays
BASE_W = 0.024  # cross-wire base at each end of the bottom rail (stance against tipping, estimate)
BASE_X = (X_POST + 0.010, X_HOLE - 0.012)
DIV_MASS = 0.090  # ~0.91 m of 4 mm steel wire at 0.099 g/mm (estimated geometry)
ANCHOR = ((X_POST + X_HOLE) / 2, 0.0, 0.040)  # centre of the frame: pivot of the yaw / roll / pitch hinges

K_START = 2  # middle-lane dividers start at holes +-2 (lanes 8-4-8)
K_TARGET = 3  # planogram: holes +-3 (lanes 7-6-7)
K_OUTER = 10  # fixed outer dividers

GLOVE = (0.178, 0.117, 0.056)  # box of 100 nitrile exam gloves, lying flat (x depth, y width, height)
GLOVE_MASS = 0.45  # estimate (box of 100 size-M gloves)
GLOVE_X = X_FENCE + 0.010 + GLOVE[0] / 2
GLOVE_Y = 0.165  # centre of each outer lane's box (|y|)

# -- scoring thresholds (shared by on_step, score, success) -------------------------------------------------------

FOOT_TOL = HOLE_HALF  # foot axis inside the planogram hole
FOOT_DOWN = -0.002  # foot tip at least 2 mm below the deck top
HOOK_TOL = PITCH / 4  # hook within a quarter pitch of the planogram hole's line
CARTON_TILT_DEG = 10.0
HAND_CLEAR_M = 0.050
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
PASS_SCORE = 1.0 - 1e-9
S_PART = 0.2  # each of: left foot, left hook, right foot, right hook
S_CLEAR = 0.2  # all four seated and both grippers 5 cm off the dividers

# -- colours (colliders; the visual meshes replace them in the render) ----------------------------------------------

STEEL = (0.80, 0.80, 0.78)
CHROME = (0.70, 0.71, 0.73)
GLOVE_BLUE = (0.55, 0.75, 0.90)


def _shelf_boxes():
    s = SHELF_Z - DECK_T / 2
    hy = SHELF_HALF_LEN
    depth_f = (X_HOLE - HOLE_HALF) - X_FRONT
    depth_b = X_BACK - (X_HOLE + HOLE_HALF)
    boxes = [
        ((X_FRONT + depth_f / 2, 0.0, s), (depth_f, 2 * hy, DECK_T), STEEL),
        ((X_BACK - depth_b / 2, 0.0, s), (depth_b, 2 * hy, DECK_T), STEEL),
    ]
    seg = PITCH - 2 * HOLE_HALF
    for k in range(-N_HOLES, N_HOLES):
        boxes.append(((X_HOLE, (k + 0.5) * PITCH, s), (2 * HOLE_HALF, seg, DECK_T), STEEL))
    end = hy - N_HOLES * PITCH - HOLE_HALF
    for sgn in (1, -1):
        boxes.append(((X_HOLE, sgn * (hy - end / 2), s), (2 * HOLE_HALF, end, DECK_T), STEEL))
        # bench stand under each end of the deck
        boxes.append((((X_FRONT + X_BACK) / 2, sgn * (hy - 0.030), (SHELF_Z - DECK_T) / 2),
                      (X_BACK - X_FRONT, 0.040, SHELF_Z - DECK_T), STEEL))
    return tuple(boxes)


def _frame_cyls(y=0.0, z0=0.0):
    """One divider's wire frame (part frame, seated pose), as (centre, radius, length, colour, axis)."""
    r, c = WIRE_R, CHROME
    cyl = []
    zr = z0 + RAIL_Z
    cyl.append((((X_POST + X_HOLE) / 2, y, z0 + r), r, X_HOLE - X_POST, c, "X"))  # bottom rail
    cyl.append((((X_HOOK + X_HOLE) / 2, y, zr), r, X_HOLE - X_HOOK, c, "X"))  # top rail, forward over the fence
    for x in (X_POST, X_HOLE) + STAYS:  # posts and stays
        cyl.append(((x, y, z0 + (r + zr) / 2), r, zr - r, c, "Z"))
    cyl.append(((X_HOOK, y, zr - HOOK_LEG / 2), r, HOOK_LEG, c, "Z"))  # hook leg
    cyl.append(((X_HOLE, y, z0 + (r - FOOT) / 2), r, FOOT + r, c, "Z"))  # rear foot
    for x in BASE_X:  # cross-wire base
        cyl.append(((x, y, z0 + r), r, BASE_W, c, "Y"))
    return cyl


def _shelf_cyls():
    cyl = [((X_FENCE, 0.0, SHELF_Z + FENCE_ZC), FENCE_R, 2 * SHELF_HALF_LEN, CHROME, "Y"),
           ((X_FENCE, 0.0, SHELF_Z + 0.006), 0.0015, 2 * SHELF_HALF_LEN, CHROME, "Y")]
    for k in (K_OUTER, -K_OUTER):
        cyl += _frame_cyls(k * PITCH, SHELF_Z)
    return tuple(cyl)


SHELF = part("shelf", (0.0, 0.0, TOP), fixed=True, friction=0.5, boxes=_shelf_boxes(), cylinders=_shelf_cyls())


def _ghost(name):
    """A 1 mm cube at the hinge centre: the shape every intermediate link of the chain needs (no self-collision)."""
    return ((ANCHOR, (0.001, 0.001, 0.001), CHROME),)


# Invisible 4 mm square collider over the gripped mid-span of the top rail (same width as the wire): PhysX lets the
# closing pads pass through the 4 mm round rail alone (Isaac run 2026-11-15, left divider never lifted).
GRIP_BOX = (((ANCHOR[0], 0.0, RAIL_Z), (0.060, 2 * WIRE_R, 2 * WIRE_R), CHROME),)


# Chain damping (no spring, no hold): the undamped six-joint chain wobbled on its own under PhysX (Isaac run 2026-11-15:
# roll 0.10 rad before contact, a seated divider pitched off its hook). Slides about critical for a ~10 Hz wobble of the
# 0.1 kg / 1.4e-4 kg m^2 frame; drag at the 25 mm/s lift is 0.13 N against the 0.9 N weight.
SLIDE_DAMP = 5.0  # N s/m
HINGE_DAMP = 0.005  # N m s/rad (0.02 resists seating: MuJoCo friction x0.7 case 0.4)


def _divider(name, k):
    a = (ANCHOR[0], 0.0, ANCHOR[2])
    sd, hd = dict(kind="slide", damping=SLIDE_DAMP), dict(kind="hinge", damping=HINGE_DAMP)
    js = (
        Joint("tx", **sd, axis="X", anchor=a, boxes=_ghost(name), lower=-0.05, upper=0.05, mass=0.002),
        Joint("ty", **sd, axis="Y", anchor=a, parent="tx", boxes=_ghost(name), lower=-0.08, upper=0.08, mass=0.002),
        Joint("tz", **sd, axis="Z", anchor=a, parent="ty", boxes=_ghost(name), lower=-0.045, upper=0.15, mass=0.002),
        Joint("yaw", **hd, axis="Z", anchor=a, parent="tz", boxes=_ghost(name), lower=-1.2, upper=1.2, mass=0.002),
        Joint("roll", **hd, axis="X", anchor=a, parent="yaw", boxes=_ghost(name), lower=-1.75, upper=1.75, mass=0.002),
        Joint("pitch", **hd, axis="Y", anchor=a, parent="roll", cylinders=tuple(_frame_cyls()), boxes=GRIP_BOX,
              lower=-1.2, upper=1.2, mass=DIV_MASS),
    )
    base = ((ANCHOR, (0.001, 0.001, 0.001), CHROME),)
    return part(name, (0.0, k * PITCH, TOP + SHELF_Z), fixed=True, friction=0.5, boxes=base, joints=js)


DIVIDER_L = _divider("divider_l", K_START)
DIVIDER_R = _divider("divider_r", -K_START)
GLOVE_L = part("glove_l", (GLOVE_X, GLOVE_Y, TOP + SHELF_Z + GLOVE[2] / 2), mass=GLOVE_MASS, friction=0.5,
               boxes=(((0.0, 0.0, 0.0), GLOVE, GLOVE_BLUE),))
GLOVE_R = part("glove_r", (GLOVE_X, -GLOVE_Y, TOP + SHELF_Z + GLOVE[2] / 2), mass=GLOVE_MASS, friction=0.5,
               boxes=(((0.0, 0.0, 0.0), GLOVE, GLOVE_BLUE),))

OBJECTS = (SHELF, DIVIDER_L, DIVIDER_R, GLOVE_L, GLOVE_R)

# Visual meshes (assets/visual, built by assets_build/build_visuals.py; the glove box from SimReady box_glove_a01 via
# assets_build/build_props.py); the shapes above stay as invisible colliders.
_ASSETS = pathlib.Path(_HERE) / "assets"
labkit.VISUALS.update({
    "shelf/body": str(_ASSETS / "visual" / "shelf.usdc"),
    "divider_l/pitch": str(_ASSETS / "visual" / "divider.usdc"),
    "divider_r/pitch": str(_ASSETS / "visual" / "divider.usdc"),
    "glove_l/body": str(_ASSETS / "props" / "glove_box.usda"),
    "glove_r/body": str(_ASSETS / "props" / "glove_box.usda"),
})
# Visual-only bench props behind the shelf (x >= 0.48 m, no arm waypoint there): handheld barcode scanner in its cradle,
# tape dispenser, two spare glove boxes, the planogram card.
labkit.SCENERY["shelf"] = [(str(_ASSETS / "props" / "bench_props.usda"), (0.0, 0.0, TOP))]
DIVIDERS = {"divider_l": (K_TARGET, "glove_l"), "divider_r": (-K_TARGET, "glove_r")}

# -- trial state ---------------------------------------------------------------------------------------------------

_trial: dict = {}
_ids: dict = {}


def _fresh() -> dict:
    return {"done_s": 0.0, "tcps": None, "step": -1}


_trial.update(_fresh())


def _set_test_trial(**kw) -> None:
    """Explicit test hook for offline scoring tests (never called by demo())."""
    for k, v in kw.items():
        if k not in _trial:
            raise KeyError(k)
        _trial[k] = tuple(np.asarray(p, float) for p in v) if k == "tcps" and v is not None else v


@on_reset
def _reset(_env) -> None:
    _trial.clear()
    _trial.update(_fresh())
    _ids.clear()


def _arm_tcps(env):
    try:
        base = env.unwrapped
        out = []
        for side in ("left", "right"):
            asset = base.scene[f"{side}_robot"]
            key = f"arm_{side}"
            if key not in _ids:
                _ids[key] = asset.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = labkit._np(asset.data.joint_pos)[0, _ids[key]]
            tcp_p, _ = ik.tcp_pose(q, station.BASES[side])
            out.append(tcp_p)
        return (out[0], out[1])
    except Exception:
        return None


# -- divider pose from its joint chain -----------------------------------------------------------------------------

def _rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    if axis == "X":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "Y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def frame_point(st: PartState, p) -> np.ndarray:
    """World position of a point given in the divider's seated part frame (relative to its part origin)."""
    j = st.joints
    t = np.array([float(j.get("tx", 0.0)), float(j.get("ty", 0.0)), float(j.get("tz", 0.0))])
    r = _rot("Z", float(j.get("yaw", 0.0))) @ _rot("X", float(j.get("roll", 0.0))) @ _rot("Y", float(j.get("pitch", 0.0)))
    a = np.array([ANCHOR[0], 0.0, ANCHOR[2]])
    origin = np.asarray(st.pos, float)  # fixed base: the part origin (x 0, y k*PITCH, deck top)
    return origin + t + a + r @ (np.asarray(p, float) - a)


def divider_feature(st: PartState) -> dict:
    """Foot tip, hook leg tip and front post (rail height) in the world, plus the frame's up-axis tilt (deg)."""
    foot = frame_point(st, (X_HOLE, 0.0, -FOOT))
    hook = frame_point(st, (X_HOOK, 0.0, RAIL_Z - HOOK_LEG))
    post = frame_point(st, (X_POST, 0.0, RAIL_Z))
    up = frame_point(st, (ANCHOR[0], 0.0, ANCHOR[2] + 1.0)) - frame_point(st, (ANCHOR[0], 0.0, ANCHOR[2]))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))
    return {"foot": foot, "hook": hook, "post": post, "tilt_deg": tilt}


def foot_ok(st: PartState, k: int) -> bool:
    """Rear foot down in planogram hole k: axis within the hole, tip >= 2 mm below the deck top."""
    f = divider_feature(st)["foot"]
    deck = TOP + SHELF_Z
    return bool(abs(f[0] - X_HOLE) <= FOOT_TOL and abs(f[1] - k * PITCH) <= FOOT_TOL and f[2] <= deck + FOOT_DOWN)


def hook_ok(st: PartState, k: int) -> bool:
    """Front hook over the fence top wire at hole k's pitch: hook leg in front of the wire and its tip below the
    wire's top, front post behind the wire, both within a quarter pitch of hole k's line."""
    d = divider_feature(st)
    wire_x, wire_top = X_FENCE, TOP + SHELF_Z + FENCE_ZC + FENCE_R
    h, p = d["hook"], d["post"]
    return bool(h[0] < wire_x - FENCE_R and p[0] > wire_x + FENCE_R and h[2] < wire_top
            and abs(h[1] - k * PITCH) <= HOOK_TOL and abs(p[1] - k * PITCH) <= HOOK_TOL)


def carton_ok(st: PartState, side: int) -> bool:
    """Product box still lying flat in its outer lane (between the moved divider's hole and the outer divider)."""
    y = float(st.pos[1]) * side
    on_deck = float(st.pos[2]) > TOP + SHELF_Z + GLOVE[2] / 2 - 0.01
    return bool(st.tilt_deg() <= CARTON_TILT_DEG and on_deck and K_START * PITCH < y < K_OUTER * PITCH
            and X_FRONT < float(st.pos[0]) < X_BACK)


def _hands_clear(tcps) -> bool:
    if tcps is None or len(tcps) < 2:
        return False
    lo = np.array([X_FRONT, -(K_TARGET + 1) * PITCH, TOP]) - HAND_CLEAR_M
    hi = np.array([X_BACK, (K_TARGET + 1) * PITCH, TOP + SHELF_Z + RAIL_Z]) + HAND_CLEAR_M
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def _parts(objects) -> dict:
    """Per divider: (foot ok, hook ok, its lane's box ok)."""
    out = {}
    for name, (k, glove) in DIVIDERS.items():
        st = objects[name]
        side = 1 if k > 0 else -1
        out[name] = (foot_ok(st, k), hook_ok(st, k), carton_ok(objects[glove], side))
    return out


SUBACTIONS = (
    ("left divider foot in planogram hole +3", "divider_l"),
    ("left divider hook over fence wire at +3", "divider_l"),
    ("right divider foot in planogram hole -3", "divider_r"),
    ("right divider hook over fence wire at -3", "divider_r"),
    ("left lane glove box standing", "glove_l"),
    ("right lane glove box standing", "glove_r"),
)


def _log_subaction_end(objects, step: int, run_end: bool) -> None:
    """Observe-only: one SUBACTION_END JSON line per sub-action on every step of the success hold."""
    vals = []
    for name in ("divider_l", "divider_r"):
        d = divider_feature(objects[name])
        vals += [[round(float(v), 5) for v in d["foot"]], [round(float(v), 5) for v in d["hook"]]]
    vals += [round(objects["glove_l"].tilt_deg(), 2), round(objects["glove_r"].tilt_deg(), 2)]
    for (name, key), end in zip(SUBACTIONS, vals):
        rec = {"name": name, "joint_or_body": key, "end_value": end, "step": step, "run_end": run_end}
        print("SUBACTION_END " + json.dumps(rec), flush=True)


@on_step
def _watch(env) -> None:
    base = env.unwrapped
    objects = {n: labkit._part_state(env, labkit._SPECS[n]) for n in ("divider_l", "divider_r", "glove_l", "glove_r")}
    _trial["step"] += 1
    _trial["tcps"] = _arm_tcps(env)
    dt = float(getattr(base, "step_dt", 0.05))
    _trial["done_s"] = _trial["done_s"] + dt if score(objects) >= PASS_SCORE else 0.0
    if _trial["done_s"] > 0.0:
        _log_subaction_end(objects, _trial["step"], _trial["done_s"] >= SUCCESS_HOLD_S - 1.5 * dt)


# -- scoring -------------------------------------------------------------------------------------------------------

def score(objects: dict[str, PartState]) -> float:
    """End state only (top-camera photo + ruler: hole count, foot down, hook over the wire); no history:
    - per moved divider, 0.2 for its rear foot down in its planogram hole (+3 / -3; any other hole or resting on the
      deck = 0) and 0.2 for its front hook over the fence top wire at that hole's pitch (+-6.35 mm);
    - a divider whose lane's glove box is knocked over (tilt > 10 deg), off the deck or out of its lane gets 0;
    - +0.2 when both dividers are fully seated (all four parts) and both grippers are 5 cm off the shelf's middle
      section (tool-centre points outside the dividers' area grown by 5 cm).
    Possible scores: 0, 0.2, 0.4, 0.6, 0.8, 1.0. Lanes 7-6-7 (counted in holes) follow from the two planogram holes.
    """
    total, full = 0.0, True
    for foot, hook, box in _parts(objects).values():
        if not box:
            full = False
            continue
        total += S_PART * (int(foot) + int(hook))
        full = full and foot and hook
    if full and _hands_clear(_trial["tcps"]):
        total += S_CLEAR
    return round(total, 6)


def success(objects: dict[str, PartState]) -> bool:
    return score(objects) >= PASS_SCORE


def distance(objects: dict[str, PartState]) -> float:
    """Summed distance (m) of both feet from their planogram holes."""
    out = 0.0
    for name, (k, _) in DIVIDERS.items():
        f = divider_feature(objects[name])["foot"]
        out += float(np.linalg.norm(f - np.array([X_HOLE, k * PITCH, TOP + SHELF_Z - FOOT])))
    return out


# -- scripted demo (fixed scene geometry only) ---------------------------------------------------------------------

GRIP_DEPTH = 0.010  # pad tips 10 mm below the rail centre (full pad face on the wire; nothing below mid-span)
GRIP_CLOSED = FINGER_CLOSED - 0.001  # pads aimed 2 mm apart on the 4 mm wire: ~5 N per pad (5000 N/m x 1 mm)
PLACE_DY = 0.0024  # measured MuJoCo set-down offset of the gripped divider (TCP lag + grip offset), both arms


def _move(side: int):
    """One arm moves its divider one hole outward: grip the top rail mid-span, lift 25 mm (foot out of the hole, hook
    leg over the fence wire), carry one pitch, lower (foot into the planogram hole, hook leg in front of the wire),
    release, lift clear."""
    W = _demo.Waypoint
    c = GRIP_CLOSED
    wide = 0.25 * FINGER_OPEN  # ~23 mm jaw around the 4 mm rail
    xg = ANCHOR[0]
    y0, y1 = side * K_START * PITCH, side * K_TARGET * PITCH
    yp = y1 - side * PLACE_DY  # aim inboard: the gripped rail lands PLACE_DY outboard of the commanded TCP
    rail = TOP + SHELF_Z + RAIL_Z
    grip = rail - GRIP_DEPTH + PAD_TIP_PAST_TCP  # pad tips GRIP_DEPTH below the rail centre
    lift = grip + 0.025
    hover = grip + 0.045
    return [
        W(1.4, (xg, y0, hover), wide, "over the divider's top rail"),
        W(0.9, (xg, y0, grip), wide, "lower around the rail"),
        W(0.7, (xg, y0, grip), c, "grip the rail"),
        W(1.0, (xg, y0, lift), c, "lift: foot out of the hole, hook off the fence wire"),
        W(1.2, (xg, yp, lift), c, "carry one hole outward"),
        W(1.2, (xg, yp, grip + 0.001), c, "lower: foot into the planogram hole, hook over the wire"),
        W(0.5, (xg, yp, grip + 0.001), wide, "release"),
        W(0.7, (xg, yp, hover), wide, "lift clear"),
    ]


def demo():
    """Left arm moves the left divider (+2 -> +3) while the right arm waits parked; then the left arm parks and the
    right arm moves the right divider (-2 -> -3); both park clear and hold."""
    W = _demo.Waypoint
    wide = 0.25 * FINGER_OPEN
    park_l = (0.16, 0.30, TOP + 0.20)
    park_r = (0.16, -0.30, TOP + 0.20)
    left = _move(+1)
    t_left = sum(w.duration for w in left)
    left += [W(1.2, park_l, wide, "park clear")]
    t_park = sum(w.duration for w in left)
    right = [W(1.0, park_r, wide, "wait parked"), W(t_park - 1.0, park_r, wide, "wait parked")]
    right += _move(-1)
    right += [W(1.2, park_r, wide, "park clear"), W(2.5, park_r, wide, "hold clear")]
    left += [W(sum(w.duration for w in right) - t_park, park_l, wide, "hold clear")]
    del t_left
    return {"left": left, "right": right}
