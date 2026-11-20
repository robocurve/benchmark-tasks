"""On the station table lies a dealing board (a smooth laminate-faced board, fixed). On it a full deck of 52 playing
cards lies face down, squared, on its mark in front of the robot, and three places are marked round it: one to the
left, one near the robot on the right, one to the right.

The job is the dealer's: take the top card off the deck, one card at a time, and deal it face down to the places in
turn, so that each place gets a card. A card is 0.29 mm thick, so it cannot be picked off the pile;
it is slid off by the friction of a fingertip on its back, which has to move that card and no other, and then slid
on across the board to its place.

The deck is in a known order, so the cards at each place at the end show whether they were dealt one at a time and
in turn: the left place must hold the deck's first card, the near place its second, the right place its third. Two
cards that came off together end up at the same place, which is then wrong, and so is every place after it.

Where the places are changes with ``CARDS_LOT`` (A or B); the deck is on the same mark in both.

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
from cardkit import Board, Card, Mark, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TABLE = TABLE_TOP[SCENE]
TOP = TABLE + K.BOARD_T          # the top of the dealing board: every height below is measured from it
BOARD = "dealing_board"
BOARD_XY, BOARD_SIZE = (0.325, 0.000), (0.470, 0.660)   # the board's centre, and its size along x and y

INSTRUCTION = ("Deal one card to each place, in turn: left, then near, then right. Slide each card off the top of the "
               "deck, one card at a time, and leave it face down at its place. Leave the rest of the deck squared on "
               "its mark.")

# -- layout (m; x away from the robot, y left, heights above the table) ---------------------------------------------------
LOT = os.environ.get("CARDS_LOT", "A")
# deck: the centre of the deck's mark. yaw: the deck's length, in degrees anticlockwise (seen from above) from
# pointing to the robot's left. places: the centre of each place's mark, in dealing order (left, near, right).
LAYOUT = {
    "A": {"deck": (0.240, 0.000), "yaw": 0.0, "places": ((0.330, 0.225), (0.200, -0.190), (0.345, -0.225))},
    "B": {"deck": (0.240, 0.000), "yaw": 0.0, "places": ((0.305, 0.205), (0.215, -0.170), (0.360, -0.205))},
}[LOT]
DECK_XY = LAYOUT["deck"]
DECK_YAW = math.radians(LAYOUT["yaw"])
PLACES = LAYOUT["places"]
COUNT = int(os.environ.get("CARDS_COUNT", "52"))   # cards in the deck (52; fewer only for solver probes)
GAP = 0.00002                    # each card starts this far above the one under it

CARDS = tuple(f"card_{k:02d}" for k in range(COUNT))       # card_00 is the bottom card, the last one the top card
DECK_MARK = "deck_mark"
PLACE_MARKS = tuple(f"place_mark_{k}" for k in range(len(PLACES)))

# -- scoring -----------------------------------------------------------------------------------------------------------
EACH = 1                 # cards asked for at each place
PLACE_R = 0.070          # a card is at a place when its centre is within this of the place's mark
LEFT_AT = 0.040          # a card is still on the deck when its centre is within this of the middle of the deck's mark
FLAT = 10.0              # degrees: a card lies flat within this of level
ON_TABLE = 0.005         # ... with no corner higher than this above the board
SQUARE = 0.010           # the deck is square when no corner of its cards is further than this outside its mark
CLEAR = 0.030            # both grippers at least this far from every card
STILL = 0.02             # m/s
HOLD = 20                # control steps (1 s) the full-score state has to last before the trial ends
MARK_W = 0.004           # width of the lines the marks are drawn with
CROSS = 0.030            # a place mark is a cross of two strips this long


def dealt(place: int) -> tuple:
    """The names of the cards that belong at ``place`` (0 left, 1 near, 2 right) when the cards are dealt in turn
    from the top of the deck (one name for each round dealt)."""
    n = len(PLACES)
    return tuple(CARDS[COUNT - 1 - (r * n + place)] for r in range(EACH))


def deck_quat() -> tuple:
    return K.yaw_quat(DECK_YAW)


def card_start(k: int) -> tuple:
    """Where card ``k`` (0 is the bottom card) lies at the start: position (its centre) and quaternion."""
    z = TOP + K.CARD_T / 2 + GAP + k * (K.CARD_T + GAP)
    return np.array([DECK_XY[0], DECK_XY[1], z]), deck_quat()


def _objects() -> list:
    tup = lambda v: tuple(float(x) for x in v)   # noqa: E731
    w, half_w, half_l = MARK_W, K.CARD_W / 2, K.CARD_L / 2
    outline = (((0.0, half_l + w / 2), (K.CARD_W + 2 * w, w)), ((0.0, -half_l - w / 2), (K.CARD_W + 2 * w, w)),
               ((half_w + w / 2, 0.0), (w, K.CARD_L)), ((-half_w - w / 2, 0.0), (w, K.CARD_L)))
    objs = [Board(BOARD, pos=(BOARD_XY[0], BOARD_XY[1], TABLE + K.BOARD_T / 2), size=(*BOARD_SIZE, K.BOARD_T),
                  color=K.BOARD_GREEN)]
    objs += [Mark(DECK_MARK, pos=(DECK_XY[0], DECK_XY[1], TOP + K.MARK_T / 2), quat=deck_quat(),
                 size=(K.CARD_W + 2 * w, K.CARD_L + 2 * w, K.MARK_T), color=K.TAPE, strips=outline)]
    cross = (((0.0, 0.0), (CROSS, w)), ((0.0, 0.0), (w, CROSS)))
    for name, xy in zip(PLACE_MARKS, PLACES):
        objs.append(Mark(name, pos=(xy[0], xy[1], TOP + K.MARK_T / 2), size=(CROSS, CROSS, K.MARK_T), color=K.TAPE,
                         strips=cross))
    for k, name in enumerate(CARDS):
        pos, quat = card_start(k)
        objs.append(Card(name, pos=tup(pos), quat=quat, size=(K.CARD_W, K.CARD_L, K.CARD_T), mass=K.CARD_MASS,
                         pips=COUNT - k if COUNT - k <= 3 else 0))
    return objs


OBJECTS = _objects()


# -- what the trial looks like now ------------------------------------------------------------------------------------
_STATE: dict = {}


def _clear() -> None:
    _STATE.update(steps=0, left={}, at=[0] * len(PLACES), good=[[False] * EACH for _ in PLACES], deck=COUNT, stray=0,
                  face_up=0, excess=0.0, square=True, rest=True, near=1.0, clear=True, held=0, score=0.0, moved=[])


_clear()


def sort_cards(cards: dict) -> dict:
    """Sort the cards by where they are at the end. ``cards``: name -> (pos, quat). Returns, per place, how many
    cards are at it (``at``) and whether the place's own card is there lying flat and face down (``good``: one flag
    for each round dealt); how many cards are still on the deck's mark (``deck``) and how far the worst of them reaches outside the
    mark's outline (``excess``, m; any of them face up makes it infinite); how many cards are neither (``stray``);
    and how many cards lie face up or on edge anywhere (``face_up``)."""
    at, good = [0] * len(PLACES), [[False] * EACH for _ in PLACES]
    own = [dealt(k) for k in range(len(PLACES))]
    deck = stray = face_up = 0
    excess = 0.0
    for name, (pos, quat) in cards.items():
        pos = np.asarray(pos, float)
        up = K.back_up(quat)
        flat = up >= math.cos(math.radians(FLAT))
        if not flat:
            face_up += 1
        d_deck = float(np.hypot(pos[0] - DECK_XY[0], pos[1] - DECK_XY[1]))
        if d_deck <= LEFT_AT:
            deck += 1
            excess = max(excess, K.outline_excess(pos, quat, DECK_XY, DECK_YAW) if flat else float("inf"))
            continue
        dist = [float(np.hypot(pos[0] - p[0], pos[1] - p[1])) for p in PLACES]
        k = int(np.argmin(dist))
        if dist[k] > PLACE_R:
            stray += 1
            continue
        at[k] += 1
        low = float(K.corners(pos, quat)[:, 2].max() - TOP) <= ON_TABLE
        if flat and low and name in own[k]:
            good[k][own[k].index(name)] = True
    return {"at": at, "good": good, "deck": deck, "excess": excess, "stray": stray, "face_up": face_up}


def score_from(at, good, stray: int, face_up: int, excess: float, rest: bool, clear: bool) -> float:
    """Pure scoring rule. ``at``: per place, how many cards are at it. ``good``: per place, whether its own card is
    there, lying flat and face down (see ``sort_cards``). A place that holds exactly one card, its own, is complete
    and earns 0.25; any other place earns nothing. 1.0 when all three places are complete and also: no card lies
    anywhere else, or face up or on edge; the rest of the deck is square on its mark (no corner more than 1 cm
    outside the outline); every card is at rest; both grippers are at least 3 cm from every card."""
    full = sum(1 for n, g in zip(at, good) if n == EACH and all(g))
    if full == len(PLACES) and not stray and not face_up and excess <= SQUARE and rest and clear:
        return 1.0
    return 0.25 * full


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
    for n, (pos, _) in cards.items():
        if n not in s["left"] and np.hypot(pos[0] - DECK_XY[0], pos[1] - DECK_XY[1]) > LEFT_AT:
            s["left"][n] = s["steps"]
    out = sort_cards(cards)
    rest = bool(max(float(np.linalg.norm(st[n].lin_vel)) for n in CARDS) < STILL)
    hands = np.array([p for side in K.gripper_points(environment) for p in side])
    # the deck as one block, and every card that has left it
    top = TOP + out["deck"] * (K.CARD_T + GAP)
    block = (np.array([DECK_XY[0], DECK_XY[1], (TOP + top) / 2]), K._rotation(deck_quat()),
             (K.CARD_W, K.CARD_L, max(top - TOP, K.CARD_T)))
    near = float(K._obb(hands, *block).min()) if out["deck"] else 1.0
    for n in s["left"]:
        near = min(near, K.card_distance(hands, *cards[n]))
    clear = bool(near >= CLEAR)
    s.update(at=out["at"], good=out["good"], deck=out["deck"], stray=out["stray"], face_up=out["face_up"],
             excess=out["excess"], square=bool(out["excess"] <= SQUARE), rest=rest, near=near, clear=clear)
    s["moved"] = [[n[-2:], *(round(1000 * float(v), 1) for v in cards[n][0] - [0.0, 0.0, TOP])] for n in list(s["left"])[:10]]
    s["score"] = score_from(out["at"], out["good"], out["stray"], out["face_up"], out["excess"], rest, clear)
    s["held"] = s["held"] + 1 if s["score"] >= 1.0 else 0


# -- scoring --------------------------------------------------------------------------------------------------------
def score(objects) -> float:
    """Scored at the end of the trial, from where the cards are then and which cards they are. See ``score_from``
    and ``sort_cards``."""
    return float(_STATE["score"])


def success(objects) -> bool:
    return _STATE["held"] >= HOLD


def report(objects) -> dict:
    s = _STATE
    ex = float(s["excess"])
    return {"score": score(objects), "lot": LOT, "at": list(s["at"]), "good": [list(g) for g in s["good"]],
            "deck": s["deck"], "stray": s["stray"], "face_up": s["face_up"],
            "excess_mm": round(1000 * ex, 1) if math.isfinite(ex) else None, "square": s["square"], "rest": s["rest"],
            "near_mm": round(1000 * float(s["near"]), 1), "clear": s["clear"],
            "left_at": dict(s["left"]), "moved_mm": list(s["moved"])}


def demo(objects=None, seed=None):
    return {}
