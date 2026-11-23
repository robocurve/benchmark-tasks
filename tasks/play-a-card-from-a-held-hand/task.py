"""The left gripper holds a hand of five playing cards, fanned 20 degrees apart and pinched together at one corner:
an ace, a two, a six, a four and a five, the ace in front, faces up toward the robot's overhead camera. The fan's
plane is reclined 30 degrees from level; its top edge points to the robot's right. On the table in front of the
right arm is a blue cross, the place to play a card.

The job is the card player's: take one card out of the hand and lay it on the table, while the rest stay in the
hand. The left gripper presses the five corners together, so the ace is held between its pad and the card behind
it. A pinch of one 0.29 mm card cannot pull it out against a firm grip: the left hand has to ease its grip enough
to let the ace slide, and no more, or the other four slip out of the hand too. The right hand can pinch the ace only
where no other card lies behind it.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cardkit as K  # noqa: E402
from cardkit import Card, Mark, on_reset, on_start, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]           # the table's surface: heights below are measured from it

INSTRUCTION = ("Play the ace from the hand of cards you are holding: take it out of the hand and lay it face up on the "
               "blue cross. Keep the other four cards in your hand.")

# -- the hand (m; x away from the robot, y left, z up) ---------------------------------------------------------------
_OTHER_RANKS = (2, 6, 4, 5)
_ACE_AT = int(os.environ.get("CARDS_ACE_AT", "0"))
RANKS = _OTHER_RANKS[:_ACE_AT] + (1,) + _OTHER_RANKS[_ACE_AT:]   # the cards' ranks from the viewer's left to right
PLAY = RANKS.index(1)            # the card to play: the ace
N = len(RANKS)
SPREAD = math.radians(float(os.environ.get("CARDS_SPREAD", "20")))   # between neighbouring cards in the fan
INSET = 0.008                    # the pivot is this far in from each card's lower right corner (both edges)
RECLINE = math.radians(float(os.environ.get("CARDS_RECLINE", "30")))  # the fan's plane rises this far from level
GAP = 0.00002                    # each card starts this far from the next
# Where the left gripper pinches the five corners together (x, y, height above the table).
_P = [float(v) for v in os.environ.get("CARDS_PIVOT", "0.260,0.080,0.150").split(",")]
PIVOT = np.array([_P[0], _P[1], TOP + _P[2]])

# The fan's plane. UP runs up the plane away from the robot; RIGHT is the viewer's right (the robot's right, -y);
# FACE is the faces' side of the plane, up and toward the robot (toward the overhead camera).
UP = np.array([math.cos(RECLINE), 0.0, math.sin(RECLINE)])
RIGHT = np.array([0.0, -1.0, 0.0])
FACE = np.cross(RIGHT, UP)
# The fan is turned in its plane by TURN (clockwise as the viewer sees it): its top edge points to the robot's right,
# toward the right arm, and the left gripper holds its pivot from the left. FAN_UP is the way the middle card points
# from the pivot; FAN_RIGHT is the fan's own right, as the hand is held.
TURN = math.radians(float(os.environ.get("CARDS_TURN", "90")))
FAN_UP = math.cos(TURN) * UP + math.sin(TURN) * RIGHT
FAN_RIGHT = math.cos(TURN) * RIGHT - math.sin(TURN) * UP

CARDS = tuple(f"card_{k}" for k in range(N))    # card_0 the leftmost (and frontmost), card_4 the rightmost
ACE = CARDS[PLAY]
OTHERS = tuple(n for n in CARDS if n != ACE)

# -- the table ---------------------------------------------------------------------------------------------------------
CROSS_XY = (0.330, -0.170)       # the blue cross: where the ace is to be laid
CROSS = 0.040                    # the cross is two strips this long
MARK_W = 0.005                   # ... and this wide
CROSS_MARK = "play_mark"

# -- scoring -----------------------------------------------------------------------------------------------------------
IN_HAND = 0.012                  # a card is in the hand when the left gripper's pads are within this of it...
HAND_UP = 0.030                  # ... and no corner of it is lower than this above the table
OUT = 0.030                      # the ace is out of the hand when the left gripper's pads are further than this from it
ON_TABLE = 0.005                 # a card lies on the table when no corner of it is higher than this above the table
FLAT = 10.0                      # degrees: and it lies within this of level
NEAR = 0.050                     # the ace is on the cross when its centre is within this of the cross's centre
NEAR_ISH = 0.100                 # ... and near it within this
CLEAR = 0.030                    # the right gripper at least this far from the ace
STILL = 0.02                     # m/s
HOLD = 20                        # control steps (1 s) the full-score state has to last before the trial ends


def fan_angle(k: int) -> float:
    """How far card ``k`` is turned in the fan's plane (rad, anticlockwise as the viewer sees it): the middle card
    points straight up the plane, the ones to its left lean left."""
    return SPREAD * ((N - 1) / 2 - k)


def _r2(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


# A card in the viewer's 2-D frame of the fan's plane (x to the viewer's right, y up the plane), before it is turned:
# its middle, relative to the pivot; and its corners.
CARD_MID = np.array([-(K.CARD_W / 2 - INSET), K.CARD_L / 2 - INSET])
CARD_CORNERS = np.array([[-(K.CARD_W - INSET), -INSET], [INSET, -INSET], [INSET, K.CARD_L - INSET],
                         [-(K.CARD_W - INSET), K.CARD_L - INSET]])


def card_polygon(k: int) -> np.ndarray:
    """Card ``k``'s outline in the fan's plane (viewer's 2-D frame, relative to the pivot), anticlockwise."""
    return CARD_CORNERS @ _r2(fan_angle(k)).T


def to_world(p2) -> np.ndarray:
    """A point of the fan's own 2-D frame (x the fan's right, y its up, relative to the pivot) in the world."""
    p2 = np.asarray(p2, float)
    return PIVOT + p2[..., :1] * FAN_RIGHT + p2[..., 1:2] * FAN_UP


def plane_dir(d2) -> np.ndarray:
    """A direction of the fan's own 2-D frame in the world."""
    return d2[0] * FAN_RIGHT + d2[1] * FAN_UP


def to_plane(p) -> np.ndarray:
    """A world point in the fan's own 2-D frame (and its height off the plane, along FACE)."""
    d = np.asarray(p, float) - PIVOT
    return np.stack([d @ FAN_RIGHT, d @ FAN_UP, d @ FACE], axis=-1)


def layer(k: int) -> float:
    """How far card ``k``'s middle plane is in front of the fan's middle plane (toward the viewer): the leftmost card
    is in front."""
    return ((N - 1) / 2 - k) * (K.CARD_T + GAP)


def card_start(k: int) -> tuple:
    """Where card ``k`` is at the start: its centre (world) and quaternion. A card's own x points to the viewer's left,
    its y up the card, its z out of its back (away from the viewer)."""
    rot2 = _r2(fan_angle(k))
    pos = to_world(rot2 @ CARD_MID) + FACE * layer(k)
    x = plane_dir(rot2 @ np.array([-1.0, 0.0]))
    y = plane_dir(rot2 @ np.array([0.0, 1.0]))
    return pos, K._quat(np.column_stack([x, y, -FACE]))


def stack_thickness() -> float:
    """The five cards' thickness together at the pinch."""
    return N * K.CARD_T + (N - 1) * GAP


# -- the left hand's hold ----------------------------------------------------------------------------------------------
HOLD_APPROACH = math.radians(float(os.environ.get("CARDS_HOLD_APPROACH", "90")))   # in the fan's frame: the left
#                                       gripper points this way (anticlockwise from the fan's right), into the pinch:
#                                       straight up the fan, from below its pivot
HOLD_DEPTH = 0.006               # its tool point is this far past the pivot along that way
HOLD_Q = None                    # the left arm's joints at the start; worked out (IK) when the scene starts, see hold_q


def hold_frame() -> tuple:
    """The left gripper at the start: tool point (world) and rotation (approach z in the plane, pointing into the
    pinch; fingers closing along x, across the plane)."""
    z = plane_dir(np.array([math.cos(HOLD_APPROACH), math.sin(HOLD_APPROACH)]))
    x = FACE.copy()
    tcp = PIVOT + HOLD_DEPTH * z
    return tcp, np.column_stack([x, np.cross(z, x), z])


def hold_q(base=None) -> np.ndarray:
    """The left arm's joints for ``hold_frame`` (IK on the URDF, the arm's own, so it needs no simulator). Either way
    round of the fingers will do (the gripper is symmetric); seeds are fixed, so the answer is too."""
    from yam_arena import ik, station

    base = station.BASES["left"] if base is None else base
    tcp, rot = hold_frame()
    flipped = np.column_stack([-rot[:, 0], -rot[:, 1], rot[:, 2]])
    rng = np.random.default_rng(0)
    seeds = [np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0])] + [rng.uniform(ik.LOWER, ik.UPPER) for _ in range(40)]
    best = None
    for r in (rot, flipped):
        for seed in seeds:
            q, pe, re = ik.solve(tcp, ik.matrix_to_quat_xyzw(r), seed, base_pos=base, iters=300)
            cost = pe + 0.02 * re
            if best is None or cost < best[1]:
                best = (q, cost)
            if pe < 2e-4 and re < 2e-3:
                return q
    return best[0]


# -- the scene ---------------------------------------------------------------------------------------------------------
def _objects() -> list:
    tup = lambda v: tuple(float(x) for x in v)   # noqa: E731
    cross = (((0.0, 0.0), (CROSS, MARK_W)), ((0.0, 0.0), (MARK_W, CROSS)))
    objs = [Mark(CROSS_MARK, pos=(CROSS_XY[0], CROSS_XY[1], TOP + K.MARK_T / 2), size=(CROSS, CROSS, K.MARK_T),
                 color=K.TAPE, strips=cross)]
    for k, name in enumerate(CARDS):
        pos, quat = card_start(k)
        objs.append(Card(name, pos=tup(pos), quat=quat, size=(K.CARD_W, K.CARD_L, K.CARD_T), mass=K.CARD_MASS,
                         pips=RANKS[k]))
    return objs


OBJECTS = _objects()
SETTLE = 10                      # control steps the starting hold settles for (0.5 s at 20 Hz) before the policy acts


@on_start
def _start(embodiment) -> None:
    """Put the hand of cards in the left gripper: the left arm at its holding pose with its fingers on the five cards'
    faces, the right arm where the embodiment's reset left it, every card at its place in the fan; then hold for half
    a second with the left gripper commanded closed."""
    import torch
    from yam_arena.constants import READY

    global HOLD_Q
    if HOLD_Q is None:
        HOLD_Q = hold_q()
    env_ = embodiment._env
    finger = -stack_thickness() / 2          # each finger's joint value with its pad on the stack's outer face
    K.place_arm(env_, "left_robot", HOLD_Q, finger)
    for k, name in enumerate(CARDS):
        K.place_object(env_, name, *card_start(k))
    base = env_.unwrapped
    act = torch.tensor([[*HOLD_Q, 0.0, *READY, 1.0]], dtype=torch.float32, device=base.device)
    for _ in range(SETTLE):
        env_.step(act)


# -- what the trial looks like now ------------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(steps=0, held=0, score=0.0, in_hand=[True] * N, ace_out=False, ace_table=False, ace_face_up=False,
                  ace_flat=False, ace_dist=1.0, rest=True, clear=True, ever_dropped=[], hold_mm=[0.0] * N)


_clear()


def judge(in_hand, ace_out: bool, ace_table: bool, face_up: bool, flat: bool, dist: float, rest: bool,
          clear: bool) -> float:
    """Pure scoring rule. ``in_hand``: for each card (left to right), whether it is still held in the left hand.
    ``ace_out``: the ace is out of the hand. ``ace_table``: it lies on the table (no corner more than 5 mm up).
    ``face_up``, ``flat``: it lies face up, within 10 degrees of level. ``dist``: its centre from the cross's centre
    (m). ``rest``: every card is still. ``clear``: the right gripper is at least 3 cm from the ace.

    0 if any card other than the ace has left the hand, or the ace has not. Otherwise: 0.25 with the ace out of the
    hand; 0.5 with it lying on the table; 0.75 with it lying flat and face up within 10 cm of the cross; 1.0 with it
    lying flat and face up within 5 cm of the cross, the right gripper clear of it and every card still."""
    others = [h for k, h in enumerate(in_hand) if k != PLAY]
    if not all(others) or not ace_out:
        return 0.0
    if not ace_table:
        return 0.25
    if not (face_up and flat) or dist > NEAR_ISH:
        return 0.5
    if dist <= NEAR and clear and rest:
        return 1.0
    return 0.75


def sort_cards(cards: dict, pads, right_pts) -> dict:
    """What the cards show. ``cards``: name -> (pos, quat). ``pads``: points on the left gripper's pads (world).
    ``right_pts``: points on the right gripper. Returns each card's distance from the left pads and whether it is in
    the hand; and for the ace whether it is out of the hand, on the table, face up, flat, and how far from the cross."""
    in_hand, hold = [], []
    for k, name in enumerate(CARDS):
        pos, quat = cards[name]
        d = K.card_distance(pads, pos, quat)
        low = float(K.corners(pos, quat)[:, 2].min() - TOP)
        hold.append(d)
        in_hand.append(bool(d <= IN_HAND and low >= HAND_UP))
    pos, quat = cards[ACE]
    pos = np.asarray(pos, float)
    up = K.back_up(quat)
    high = float(K.corners(pos, quat)[:, 2].max() - TOP)
    return {"in_hand": in_hand, "hold": hold, "ace_out": bool(hold[PLAY] > OUT),
            "ace_table": bool(high <= ON_TABLE), "face_up": bool(up < 0.0),
            "flat": bool(abs(up) >= math.cos(math.radians(FLAT))),
            "dist": float(np.hypot(pos[0] - CROSS_XY[0], pos[1] - CROSS_XY[1])),
            "clear": bool(K.card_distance(right_pts, pos, quat) >= CLEAR)}


def pad_points(environment, arm: str = "left", sides=(-1.0, 1.0)) -> np.ndarray:
    """Points on a gripper's pads (both, or the one on ``sides``): a grid over each pad's face, at the fingers'
    current opening."""
    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    origin = K._origin(environment)
    robot = base.scene[f"{arm}_robot"]
    pose = K._np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
    rot = K._rotation(pose[3:7])
    tcp = pose[:3] + rot @ np.array(TCP_OFFSET, float) - origin
    half = K.finger_gaps(environment)[0 if arm == "left" else 1] / 2
    pts = []
    for s in sides:
        for along in np.linspace(-0.005, 0.013, 4):
            for across in np.linspace(-0.0135, 0.0135, 4):
                pts.append(tcp + rot[:, 0] * s * half + rot[:, 2] * along + rot[:, 1] * across)
    return np.array(pts)


@on_reset
def _reset(environment) -> None:
    _clear()


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states   # importable only once the simulator has started

    s = _STATE
    s["steps"] += 1
    st = object_states(environment, list(CARDS))
    cards = {n: (st[n].pos, st[n].quat_xyzw) for n in CARDS}
    out = sort_cards(cards, pad_points(environment), K.gripper_points(environment)[1])
    rest = bool(max(float(np.linalg.norm(st[n].lin_vel)) for n in CARDS) < STILL)
    for k, h in enumerate(out["in_hand"]):
        if not h and k != PLAY and CARDS[k] not in s["ever_dropped"]:
            s["ever_dropped"].append(CARDS[k])
    s.update(in_hand=out["in_hand"], ace_out=out["ace_out"], ace_table=out["ace_table"], ace_face_up=out["face_up"],
             ace_flat=out["flat"], ace_dist=out["dist"], rest=rest, clear=out["clear"],
             hold_mm=[round(1000 * h, 1) for h in out["hold"]])
    # The other four must stay in the hand the whole trial: one that has left it counts as gone even if put back.
    held = [h and (k == PLAY or CARDS[k] not in s["ever_dropped"]) for k, h in enumerate(out["in_hand"])]
    s["score"] = judge(held, out["ace_out"], out["ace_table"], out["face_up"], out["flat"], out["dist"],
                       rest, out["clear"])
    s["held"] = s["held"] + 1 if s["score"] >= 1.0 else 0


# -- scoring --------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    """Scored at the end of the trial, from where the cards are then. See ``judge``."""
    return float(_STATE["score"])


def success(objects) -> bool:
    return _STATE["held"] >= HOLD


def report(objects) -> dict:
    s = _STATE
    return {"score": score(objects), "in_hand": list(s["in_hand"]), "hold_mm": list(s["hold_mm"]),
            "ace_out": s["ace_out"], "ace_table": s["ace_table"], "ace_face_up": s["ace_face_up"],
            "ace_flat": s["ace_flat"], "ace_dist_mm": round(1000 * s["ace_dist"], 1), "rest": s["rest"],
            "clear": s["clear"], "ever_dropped": list(s["ever_dropped"])}


def demo(objects=None, seed=None):
    return {}
