"""Offline tests of the scorer with made-up end states (no simulator): every level, every cap, every cheat."""
import dataclasses
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import drivekit as dk  # noqa: E402
import lube_a_bike_chain as t  # noqa: E402
from labkit import set_gripper_points  # noqa: E402

AWAY = dict(left=(0.32, 0.305, t.TOP + 0.26), right=(0.32, -0.305, t.TOP + 0.26))


def drive(wet=0, spills=0, drops=None, speed=0.0, off=0.0):
    lubed = np.zeros(dk.N_LINKS, int)
    lubed[:wet] = 1
    pins = dk.start_pins(t.ORIGIN)
    if off:
        pins = pins + np.array([off, 0.0, 0.0])  # pushed across, off the teeth (the bike lies along y)
    return dataclasses.replace(dk.state(t.ORIGIN, pins), lubed=lubed, spills=spills,
                               drops=wet + spills if drops is None else drops, crank_speed=speed)


def bottle(upright=True, lifted=0.0, speed=0.0):
    q = (0, 0, 0, 1.0) if upright else (math.sin(math.pi / 4), 0, 0, math.cos(math.pi / 4))
    return dk.bottle_state(t.BOTTLE_AT + np.array([0, 0, lifted]), q, lin=(speed, 0, 0))


def s(d, b, grips=AWAY):
    set_gripper_points(**grips)
    return t.score({"drivetrain": d, "lube_bottle": b})


def test_nothing_done_scores_zero():
    assert s(drive(0), bottle()) == 0.0


def test_a_few_links_under_a_still_nozzle_score_zero():
    assert s(drive(13), bottle()) == 0.0


def test_ladder():
    assert s(drive(14), bottle()) == 0.25
    assert s(drive(27), bottle()) == 0.25
    assert s(drive(28), bottle()) == 0.5
    assert s(drive(49), bottle()) == 0.5
    assert s(drive(50), bottle(upright=False)) == 0.75
    assert s(drive(50), bottle()) == 1.0
    assert s(drive(56), bottle()) == 1.0


def test_full_marks_need_the_bottle_put_back_and_let_go():
    full = drive(56)
    assert s(full, bottle(upright=False)) == 0.75  # left lying on its side
    assert s(full, bottle(lifted=0.10)) == 0.75  # still in the air
    assert s(full, bottle(speed=0.05)) == 0.75  # still moving
    held = dict(AWAY, left=tuple(t.BOTTLE_AT + np.array([0, 0, 0.04])))
    assert s(full, bottle(), held) == 0.75  # a hand still on the bottle
    on_pedal = dict(AWAY, right=tuple(t.pedal_at(0.0)))
    assert s(full, bottle(), on_pedal) == 0.75  # a hand still on the pedal
    assert s(drive(56, speed=1.0), bottle()) == 0.75  # cranks still spinning


def test_mess_caps_the_score_at_half():
    assert s(drive(56, spills=13), bottle()) == 0.5  # thirteen drops off the chain
    assert s(drive(56, spills=12), bottle()) == 1.0  # twelve are allowed
    assert s(drive(56, drops=141), bottle()) == 0.5  # flooded: more than two and a half drops a link
    assert s(drive(56, drops=140), bottle()) == 1.0
    assert s(drive(20, spills=30), bottle()) == 0.25


def test_chain_knocked_off_caps_at_half():
    assert s(drive(56, off=0.02), bottle()) == 0.5
    assert s(drive(56, off=0.004), bottle()) == 1.0


def test_success_only_at_full_marks():
    set_gripper_points(**AWAY)
    assert t.success({"drivetrain": drive(56), "lube_bottle": bottle()})
    assert not t.success({"drivetrain": drive(56), "lube_bottle": bottle(upright=False)})


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
