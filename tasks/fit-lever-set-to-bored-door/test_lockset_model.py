"""CPU checks of the screw model in lockset_runtime.py (no simulator): the draw follows the screw's turns and snug
fixes the set; the shanks hold the base on the posts only while the screws' tips are in them; a screw
catches with the outside half standing proud.

    pytest test_lockset_model.py    (in an environment with yam_arena installed)
"""
from pathlib import Path

import numpy as np
import pytest
from yam_arena.task_file import load

T = load(Path(__file__).with_name("fit_lever_set_to_bored_door.py")).module
LR = T.LR


def _frames(out_dy=0.0, in_dy=0.0, in_dz=0.0, out_dz=0.0):
    z = np.zeros(3)
    return {"outside": (T.OUT_SEATED + (0.0, out_dy, out_dz), np.eye(3), z, z),
            "inside": (T.IN_SEATED + (0.0, in_dy, in_dz), np.eye(3), z, z),
            "driver": (np.asarray(T.DRIVER_START, float), np.eye(3), z, z)}


def test_the_draw_follows_the_turns_and_snug_fixes_the_set():
    M = LR.Model()
    fr = _frames(out_dy=-0.0024)  # an unheld outside half slid 2.4 mm off the door, as on the simulator
    pulls = []
    for k in range(1, 21):
        M.turns = [0.1 * k, 0.0]
        M.update(fr, {})
        pulls.append(M.pull(0))
        if 0.1 * k <= LR.CLAMP_FROM:
            assert M.pull(0) <= LR.DRAW_N + 1e-9 and not M.welded
    assert pulls[0] < 0.5  # a screw that has just caught does not yank the halves together
    assert M.welded and pulls[-1] == pytest.approx(LR.CLAMP_N)


def test_the_shanks_hold_the_base_on_the_posts_only_while_the_screws_reach_them():
    M = LR.Model()
    acc = {}
    M.update(_frames(in_dz=-0.0005), acc)  # base sagged 0.5 mm on the posts
    assert all(M.shank)
    assert acc["inside"][0][2] > 0 and acc["outside"][0][2] < 0  # equal and opposite, base lifted back on the axis
    M2 = LR.Model()
    M2.update(_frames(in_dy=0.035), {})  # base 35 mm off the door: the screws' tips (31.7 mm in when seated) are short
    assert not any(M2.shank)


def _q(x, y, z, w):
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                     (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def _driver_in(inside, screw, angle):
    """The screwdriver's bit in a screw's recess, turned forward by angle (rad)."""
    pi, Ri, z, _ = inside
    R = Ri @ T.axis_angle_matrix(np.array((0.0, -1.0, 0.0)), angle)
    return (pi + Ri @ (np.asarray(LR.SCREWS[screw]) + (0.0, 0.0005, 0.0)), R, z, z)


def test_a_screw_catches_with_the_outside_half_standing_proud():
    # poses read on the simulator with the outside half left unheld (DEMO_HOLD = False): it slid 4 mm off the door,
    # dropped 1.5 mm and pitched 4 deg in its posts' play and drooped 2.3 deg; the base sits square on the door face
    z = np.zeros(3)
    fr = {"outside": (T.OUT_SEATED + (0.0, -0.00405, -0.0015), _q(0.0359, -0.0197, -0.0007, 0.9992), z, z),
          "inside": (T.IN_SEATED + (0.0002, 0.00035, 0.0004), _q(0.0077, -0.0135, 0.0001, 0.9999), z, z),
          "driver": (np.asarray(T.DRIVER_START, float), np.eye(3), z, z)}
    M = LR.Model()
    M.update(fr, {})
    assert all(M.shank) and T._on_thread(fr)  # the 1-1/2 in screws' tips are in the posts' mouths
    for k in range(13):
        fr["driver"] = _driver_in(fr["inside"], 0, np.radians(10.0 * k))
        M.update(fr, {})
    assert M.turns[0] == pytest.approx(120.0 / 360.0, abs=1e-6) and M.turns[1] == 0.0
    assert M.pull(0) > 0.0  # the caught screw draws the proud half in


def test_the_shanks_let_the_base_on_but_hold_it_against_being_drawn_off():
    M = LR.Model()
    M.update(_frames(), {})  # pushed on, seated
    acc = {}
    M.update(_frames(in_dy=0.001), acc)  # drawn 1 mm off: pulled back toward the door, at most F_SHANK_AX a screw
    assert all(M.shank) and acc["inside"][0][1] < 0 and acc["outside"][0][1] > 0
    assert -acc["inside"][0][1] <= 2 * LR.F_SHANK_AX + 1e-9
