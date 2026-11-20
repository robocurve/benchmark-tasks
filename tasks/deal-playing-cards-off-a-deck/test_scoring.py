"""Offline checks for the card-dealing task: the scoring rule, how cards are sorted, the shapes, both layouts, the
task description and the demo's geometry. No simulator needed.

    python -m pytest -q test_scoring.py
"""
import importlib
import json
import math
import os
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def load(lot: str):
    os.environ["CARDS_LOT"] = lot
    os.environ.pop("CARDS_COUNT", None)
    for name in ("task", "demo"):
        sys.modules.pop(name, None)
    from yam_arena import task_file

    return task_file.load(str(HERE / "task.py")).module


@pytest.fixture(params=["A", "B"])
def T(request):
    return load(request.param)


ALL = [[True]] * 3
NONE = [[False]] * 3
FULL = dict(stray=0, face_up=0, excess=0.0, rest=True, clear=True)


def start_cards(T) -> dict:
    return {n: T.card_start(k) for k, n in enumerate(T.CARDS)}


def lay(T, cards: dict, name: str, place: int, dx=0.0, dy=0.0, quat=None, z=None) -> None:
    p = T.PLACES[place]
    cards[name] = (np.array([p[0] + dx, p[1] + dy, T.TOP + T.K.CARD_T / 2 if z is None else z]),
                   T.K.yaw_quat(0.3) if quat is None else quat)


def dealt_cards(T) -> dict:
    cards = start_cards(T)
    for place in range(3):
        lay(T, cards, T.dealt(place)[0], place, dx=0.01)
    return cards


def total(T, cards, rest=True, clear=True) -> float:
    out = T.sort_cards(cards)
    return T.score_from(out["at"], out["good"], out["stray"], out["face_up"], out["excess"], rest, clear)


# -- the rule ----------------------------------------------------------------------------------------------------------
def test_full_score(T):
    assert T.score_from([1, 1, 1], ALL, **FULL) == 1.0


def test_nothing_dealt_scores_zero(T):
    assert T.score_from([0, 0, 0], NONE, **FULL) == 0.0


@pytest.mark.parametrize("places, want", [(1, 0.25), (2, 0.5), (3, 0.75)])
def test_each_place_earns_a_quarter(T, places, want):
    at = [1] * places + [0] * (3 - places)
    good = [[True]] * places + [[False]] * (3 - places)
    assert T.score_from(at, good, stray=0, face_up=0, excess=0.0, rest=True, clear=False) == want


def test_two_cards_at_a_place_earn_nothing_there(T):
    assert T.score_from([2, 1, 1], ALL, **FULL) == 0.5


def test_a_card_that_is_not_the_places_own_earns_nothing_there(T):
    assert T.score_from([1, 1, 1], [[False], [True], [True]], **FULL) == 0.5


@pytest.mark.parametrize("fault", [dict(stray=1), dict(face_up=1), dict(excess=0.0101), dict(rest=False), dict(clear=False)])
def test_last_quarter_needs_everything_clean(T, fault):
    assert T.score_from([1, 1, 1], ALL, **{**FULL, **fault}) == 0.75


def test_deck_square_at_one_centimetre(T):
    assert T.score_from([1, 1, 1], ALL, **{**FULL, "excess": 0.010}) == 1.0


# -- which cards belong where -----------------------------------------------------------------------------------------
def test_deal_order(T):
    top = [T.CARDS[-1 - k] for k in range(3)]
    assert [T.dealt(k) for k in range(3)] == [(top[0],), (top[1],), (top[2],)]


def test_start_sorts_as_a_whole_deck(T):
    out = T.sort_cards(start_cards(T))
    assert out["deck"] == T.COUNT == 52 and out["stray"] == 0 and out["face_up"] == 0
    assert out["at"] == [0, 0, 0] and out["excess"] < 1e-9
    assert total(T, start_cards(T)) == 0.0


def test_dealt_in_turn_sorts_as_full(T):
    out = T.sort_cards(dealt_cards(T))
    assert out["at"] == [1, 1, 1] and out["good"] == ALL
    assert out["deck"] == 49 and out["stray"] == 0 and out["face_up"] == 0
    assert total(T, dealt_cards(T)) == 1.0
    assert total(T, dealt_cards(T), clear=False) == 0.75
    assert total(T, dealt_cards(T), rest=False) == 0.75


def test_dealt_out_of_turn_puts_the_wrong_cards_down(T):
    """Far, then left, then right: every place has one card; only the right place's is its own."""
    cards = start_cards(T)
    top = [T.CARDS[-1 - k] for k in range(3)]
    for name, place in zip(top, (1, 0, 2)):
        lay(T, cards, name, place)
    out = T.sort_cards(cards)
    assert out["at"] == [1, 1, 1] and out["good"] == [[False], [False], [True]]
    assert total(T, cards) == 0.25


def test_a_double_puts_the_wrong_cards_down(T):
    """The first two cards come off together and go to the left place; the next two go to the far and right
    places: the left place has two cards, and the others have cards that are not their own."""
    cards = start_cards(T)
    top = [T.CARDS[-1 - k] for k in range(4)]
    for name, place, dy in ((top[0], 0, 0.02), (top[1], 0, -0.02), (top[2], 1, 0.0), (top[3], 2, 0.0)):
        lay(T, cards, name, place, dy=dy)
    out = T.sort_cards(cards)
    assert out["at"] == [2, 1, 1] and out["good"] == [[True], [False], [False]]
    assert total(T, cards) == 0.0


def test_face_up_card_does_not_count(T):
    cards = dealt_cards(T)
    lay(T, cards, T.dealt(1)[0], 1, quat=T.K.face_up_quat(0.3))
    out = T.sort_cards(cards)
    assert out["face_up"] == 1 and out["good"][1] == [False] and out["at"][1] == 1
    assert total(T, cards) == 0.5


def test_card_seven_centimetres_from_its_mark(T):
    cards = dealt_cards(T)
    name = T.dealt(2)[0]
    lay(T, cards, name, 2, dx=0.0695)
    assert T.sort_cards(cards)["at"][2] == 1 and total(T, cards) == 1.0
    lay(T, cards, name, 2, dx=0.0705)
    out = T.sort_cards(cards)
    assert out["at"][2] == 0 and out["stray"] == 1 and total(T, cards) == 0.5


def test_card_propped_up_does_not_count(T):
    cards = dealt_cards(T)
    name = T.dealt(0)[0]
    tilt = T.K._quat(T.K._rz(0.3) @ np.array([[1, 0, 0], [0, math.cos(0.3), -math.sin(0.3)], [0, math.sin(0.3), math.cos(0.3)]]))
    lay(T, cards, name, 0, quat=tilt, z=T.TOP + 0.015)
    out = T.sort_cards(cards)
    assert out["good"][0] == [False] and out["face_up"] == 1


def test_whole_deck_pushed_to_a_place_scores_zero(T):
    cards = {}
    for k, n in enumerate(T.CARDS):
        pos, quat = T.card_start(k)
        p = T.PLACES[1]
        cards[n] = (np.array([p[0], p[1], pos[2]]), quat)
    out = T.sort_cards(cards)
    assert out["at"][1] == 52 and out["deck"] == 0
    assert total(T, cards) == 0.0


def test_deck_pushed_eleven_millimetres_is_not_square(T):
    cards = dealt_cards(T)
    for k, n in enumerate(T.CARDS[:49]):
        pos, quat = T.card_start(k)
        cards[n] = (pos + np.array([0.011, 0.0, 0.0]), quat)
    out = T.sort_cards(cards)
    assert out["deck"] == 49 and out["excess"] > T.SQUARE
    assert total(T, cards) == 0.75


def test_card_left_hanging_off_the_deck_is_not_square(T):
    cards = start_cards(T)
    pos, quat = T.card_start(51)
    cards[T.CARDS[51]] = (pos + np.array([0.03, 0.0, 0.0]), quat)
    out = T.sort_cards(cards)
    assert out["deck"] == 52 and out["excess"] > 0.02


def test_card_between_deck_and_place_is_stray(T):
    cards = start_cards(T)
    p, d = np.array(T.PLACES[1]), np.array(T.DECK_XY)
    mid = (p + d) / 2
    cards[T.CARDS[51]] = (np.array([mid[0], mid[1], T.TOP + T.K.CARD_T / 2]), T.deck_quat())
    out = T.sort_cards(cards)
    assert out["stray"] == 1 and out["deck"] == 51


# -- shapes and layout -----------------------------------------------------------------------------------------------
def test_card_is_poker_size(T):
    K = T.K
    assert (K.CARD_W, K.CARD_L) == (0.0635, 0.0889)
    assert 52 * K.CARD_T == pytest.approx(0.01515, abs=0.0002)        # the measured deck was 15.15 mm thick
    assert 10 * K.CARD_MASS == pytest.approx(0.0155)                   # ten cards weighed 15.5 g


def test_deck_starts_in_order_face_down(T):
    z = [T.card_start(k)[0][2] for k in range(T.COUNT)]
    assert all(b > a for a, b in zip(z, z[1:]))
    assert z[0] - T.TOP == pytest.approx(T.K.CARD_T / 2 + T.GAP)
    assert z[-1] - T.TOP < 0.0162
    assert T.K.back_up(T.deck_quat()) == pytest.approx(1.0)
    assert T.K.back_up(T.K.face_up_quat(0.4)) == pytest.approx(-1.0)


def test_objects(T):
    names = [o.name for o in T.OBJECTS]
    assert len(names) == len(set(names)) == 52 + 5
    board = T.OBJECTS[0]
    assert isinstance(board, T.Board) and board.pos[2] + board.size[2] / 2 == pytest.approx(T.TOP)
    # the deck, every place and every spot a card can count at are on the board, 3 cm or more from its edge
    for p in T.PLACES:
        for i in range(2):
            assert abs(p[i] - T.BOARD_XY[i]) + T.PLACE_R <= T.BOARD_SIZE[i] / 2 - 0.03
    for i in range(2):
        assert abs(T.DECK_XY[i] - T.BOARD_XY[i]) + T.K.CARD_L / 2 <= T.BOARD_SIZE[i] / 2 - 0.02
    cards = [o for o in T.OBJECTS if isinstance(o, T.Card)]
    assert [c.pips for c in cards[-3:]] == [3, 2, 1]                   # the top card is the ace
    assert all(c.pips == 0 for c in cards[:-3])
    assert all(c.mass == T.K.CARD_MASS for c in cards)


def test_places_are_apart_and_clear_of_the_deck(T):
    d = np.array(T.DECK_XY)
    for i, p in enumerate(T.PLACES):
        # a card at the edge of a place's circle cannot count as on the deck, nor reach its outline
        assert np.linalg.norm(np.array(p) - d) > T.PLACE_R + T.LEFT_AT + 0.03
        for q in T.PLACES[i + 1:]:
            assert np.linalg.norm(np.array(p) - np.array(q)) > 2 * T.PLACE_R


def test_layouts_differ():
    a, b = load("A"), load("B")
    assert all(np.hypot(p[0] - q[0], p[1] - q[1]) >= 0.02 for p, q in zip(a.PLACES, b.PLACES))


def test_outline_excess(T):
    K = T.K
    pos, quat = T.card_start(0)
    assert K.outline_excess(pos, quat, T.DECK_XY, T.DECK_YAW) == pytest.approx(0.0, abs=1e-9)
    assert K.outline_excess(pos + [0.004, 0.0, 0.0], quat, T.DECK_XY, T.DECK_YAW) == pytest.approx(0.004, abs=1e-6)
    turned = K.yaw_quat(T.DECK_YAW + math.radians(5))
    assert 0.002 < K.outline_excess(pos, turned, T.DECK_XY, T.DECK_YAW) < 0.005


def test_card_distance(T):
    K = T.K
    pos, quat = T.card_start(0)
    assert K.card_distance([pos + [0.0, 0.0, 0.03]], pos, quat) == pytest.approx(0.03 - K.CARD_T / 2)


# -- the description ---------------------------------------------------------------------------------------------------
def test_task_json(T):
    d = json.loads((HERE / "task.json").read_text())
    for key in ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"):
        assert d.get(key), key
    assert d["instruction"] == T.INSTRUCTION
    assert d["category"] == "Societal relatability"
    assert len(d["title"]) <= 120 and len(d["fidelity"]) <= 500 and len(d["context"]) <= 1000 and len(d["scores"]) <= 300
    assert len(d["setup"]) <= 1000 and len(d["scoring"]) <= 1000
    assert "media" not in d or d["media"].startswith("M-")


# -- the demo's geometry -------------------------------------------------------------------------------------------------
def test_demo_spots_count_at_their_places(T):
    import demo as D

    importlib.reload(D)
    cards = start_cards(T)
    for place in range(3):
        xy = D.spot(T, place)
        cards[T.dealt(place)[0]] = (np.array([xy[0], xy[1], T.TOP + T.K.CARD_T / 2]), T.deck_quat())
    assert total(T, cards) == 1.0
    # a card pushed straight away from the robot leaves the deck clear of every place's circle
    for p in T.PLACES:
        assert np.hypot(p[0] - (T.DECK_XY[0] + 2 * D.deck_edge(T, D.AWAY)), p[1] - T.DECK_XY[1]) > 0.08


def test_demo_touch_point(T):
    import demo as D

    importlib.reload(D)
    for name in ("left", "right"):
        touch = np.array([0.25, 0.01, T.TOP + 0.015])
        tool, rot = D.tool_for(name, touch), D.hand_rot(name)
        tip = tool + D.TIP * rot[:, 2]
        # the rear edge of the fingertips' end is the lowest point of the hand
        ends = [tip + s * D.TIP_THICK * rot[:, 0] for s in (-1.0, 1.0)]
        low = min(ends, key=lambda p: p[2])
        assert np.allclose(low, touch, atol=1e-9)
        assert low[0] < tip[0]


def test_deck_edge(T):
    import demo as D

    importlib.reload(D)
    along = T.K._rz(T.DECK_YAW) @ np.array([0.0, 1.0, 0.0])
    across = T.K._rz(T.DECK_YAW) @ np.array([1.0, 0.0, 0.0])
    assert D.deck_edge(T, along[:2]) == pytest.approx(T.K.CARD_L / 2)
    assert D.deck_edge(T, across[:2]) == pytest.approx(T.K.CARD_W / 2)
    assert D.deck_top(T, 0) - T.TOP == pytest.approx(52 * T.K.CARD_T)
    # the deck's edges run along and across the table, so a card pushed straight away leaves over one whole edge
    assert min(abs(along[0]), abs(along[1])) < 1e-9
