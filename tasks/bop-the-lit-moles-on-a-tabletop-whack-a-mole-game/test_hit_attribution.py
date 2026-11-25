"""Offline checks of hit attribution (no GPU): a lit mole pressed past its switch earns a hit only when the head of
the held mallet is on it. The handle, a loose mallet, a finger or the gripper body earn no hit, and a finger press
still scores the whole trial 0. The moles' switch and the game's rules are the real whack_kit.Game; the poses are
built the way the simulator reports them (the unit and its pressed mole, the mallet, and the gripper's points).

    pytest bop-the-lit-moles-on-a-tabletop-whack-a-mole-game    (in an environment with yam_arena installed)
"""

import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena.task_file import load

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.modules.pop("yam_parts", None)
import yam_parts  # noqa: E402
sys.modules.pop("yam_joints", None)
import yam_joints  # noqa: E402,F401

TASK = load(Path(__file__).with_name("bop_the_lit_moles_on_a_tabletop_whack_a_mole_game.py"))
T = TASK.module
wk = T.wk
OBJ = {o.name: o for o in T.OBJECTS}
PRESSED = -0.005               # the mole 5 mm down: past the 4 mm switch
PAD_HALF = 0.00885             # half the jaw gap measured on the GPU with the handle pinched (17.7 mm)


def game_state(pressed=None):
    q = {m: (PRESSED if m == pressed else 0.0) for m in T.MOLES}
    return OBJ["game"].offline_state((T.UX, T.UY, T.TOP), 0.0, q)


def mallet_state(head):
    origin = np.asarray(head, float)
    R = np.eye(3)
    low, high = OBJ["mallet"].aabb(origin, R)
    return yam_parts.PartState((low + high) / 2, np.array([0.0, 0.0, 0.0, 1.0]), np.zeros(3), np.zeros(3), low, high,
                               origin)


def dome(mole, pressed=True):
    mx, my = T.MOLES[mole]
    return np.array([T.UX + mx, T.UY + my, T.TOP + T.DOME_C + (PRESSED if pressed else 0.0)])


def hand(mallet, grip_x, half=PAD_HALF):
    """A top-down gripper whose pads close across the handle at grip_x along it (mallet frame): tool point, two pad
    centres, two fingertips 13 mm below them."""
    tcp = mallet.to_world((grip_x, 0.0, -0.0008))
    pads = [tcp + np.array([0.0, half, 0.0]), tcp - np.array([0.0, half, 0.0])]
    tips = [p - np.array([0.0, 0.0, 0.013]) for p in pads]
    return {"right": np.array([tcp, *pads, *tips])}


def head_on_dome(mole):
    """The mallet's head resting on the pressed mole's dome (the head's underside on the dome's top)."""
    return mallet_state(dome(mole) + np.array([0.0, 0.0, T.DOME_R + T.HEAD_R]))


def handle_on_dome(mole, at=-0.060):
    """The handle (at `at` along it, behind the head) resting on the pressed mole's dome; head beyond the mole."""
    return mallet_state(dome(mole) + np.array([-at, 0.0, T.DOME_R + T.HANDLE_R]))


def who_presses(case, mole):
    g = game_state(mole)
    if case == "held head":
        m = head_on_dome(mole)
        return T._who(g, m, hand(m, -T.GRIP_BACK))(mole)
    if case == "held handle":
        m = handle_on_dome(mole)
        return T._who(g, m, hand(m, -0.110))(mole)
    if case == "loose head":                        # head on the mole, nothing holding the mallet; hand parked
        return T._who(g, head_on_dome(mole), {"right": np.array([T.PARK["right"]] * 5)})(mole)
    if case == "open jaws round head-down mallet":  # hand at the handle but the jaws open (not holding)
        m = head_on_dome(mole)
        return T._who(g, m, hand(m, -T.GRIP_BACK, half=0.040))(mole)
    if case == "finger":                            # closed fingertips pressing the dome; mallet on the desk
        tips = dome(mole) + np.array([0.0, 0.0, T.DOME_R])
        tcp = tips + np.array([0.0, 0.0, 0.013])
        pts = np.array([tcp, tcp + [0, 0.004, 0], tcp - [0, 0.004, 0], tips + [0, 0.004, 0], tips - [0, 0.004, 0]])
        return T._who(g, mallet_state((T.MALLET0[0], T.MALLET0[1], T.TOP + T.HEAD_R)), {"right": pts})(mole)
    if case == "gripper body":                      # the hand itself (open jaws either side) pushing the dome down
        tcp = dome(mole) + np.array([0.0, 0.0, T.DOME_R + 0.002])
        pts = np.array([tcp, tcp + [0, 0.04, 0], tcp - [0, 0.04, 0], tcp + [0, 0.04, -0.013], tcp - [0, 0.04, -0.013]])
        return T._who(g, mallet_state((T.MALLET0[0], T.MALLET0[1], T.TOP + T.HEAD_R)), {"right": pts})(mole)
    raise ValueError(case)


def play_round(pressers):
    """One round of the real game script: the start signal, then each lighting is pressed by pressers[i]."""
    game = wk.Game(list(T.MOLES))
    game.reset(11)
    t, up = 0.0, {m: 0.0 for m in T.MOLES}
    game.update(t, up, started=True)
    for presser in pressers:
        while game.lit is None and not game.done:
            t += 0.05
            game.update(t, up)
        mole = game.lit
        t += 0.05
        game.update(t, {**up, mole: PRESSED}, who=lambda m, mole=mole, presser=presser: who_presses(presser, m))
        for _ in range(8):                          # lift off: the mole comes back up, its switch opens
            t += 0.05
            game.update(t, up)
    return game


@pytest.mark.parametrize("mole", list(T.MOLES))
def test_the_head_of_the_held_mallet_on_a_lit_mole_is_a_hit(mole):
    m = head_on_dome(mole)
    assert T.held(m, hand(m, -T.GRIP_BACK))
    assert who_presses("held head", mole) == "head"
    game = play_round(["held head"])
    assert (game.hits, game.foul, game.wrong, game.bare) == (1, 0, 0, 0)


@pytest.mark.parametrize("case, label", [("held handle", "handle"), ("loose head", "loose"),
                                         ("open jaws round head-down mallet", "loose"), ("finger", "gripper"),
                                         ("gripper body", "gripper")])
def test_a_press_by_anything_but_the_held_mallets_head_earns_no_hit(case, label):
    assert who_presses(case, "m1") == label
    game = play_round([case])
    assert game.hits == 0
    assert game.foul + game.bare >= 1 and game.lit is None   # the light went out; the lighting is lost


def test_the_handle_of_a_held_mallet_is_held_but_not_the_head():
    m = handle_on_dome("m0")
    assert T.held(m, hand(m, -0.110))
    assert not T.head_on(m, game_state("m0"), "m0") and T.handle_on(m, game_state("m0"), "m0")


def test_a_full_round_with_the_held_mallets_head_scores_full(monkeypatch):
    game = play_round(["held head"] * wk.N_LIT)
    assert (game.hits, game.wrong, game.bare, game.foul) == (8, 0, 0, 0) and game.done
    monkeypatch.setattr(T, "TEST_GAME", (game.hits, game.wrong, game.bare))
    start = {"game": game_state(), "mallet": mallet_state((T.MALLET0[0], T.MALLET0[1], T.TOP + T.HEAD_R))}
    assert T.score(start) == 1.0


@pytest.mark.parametrize("case, expected", [("held handle", 0.75), ("loose head", 0.75), ("finger", 0.0),
                                            ("gripper body", 0.0)])
def test_one_lit_mole_pressed_some_other_way_costs_its_hit(case, expected, monkeypatch):
    game = play_round(["held head"] * (wk.N_LIT - 1) + [case])
    assert game.hits == wk.N_LIT - 1
    monkeypatch.setattr(T, "TEST_GAME", (game.hits, game.wrong, game.bare))
    start = {"game": game_state(), "mallet": mallet_state((T.MALLET0[0], T.MALLET0[1], T.TOP + T.HEAD_R))}
    assert T.score(start) == pytest.approx(expected)   # 7 hits = 0.75; a finger or gripper press = 0
