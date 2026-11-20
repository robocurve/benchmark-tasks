"""Offline checks of the scorer on hand-built states: every tier and every zero. No simulator.

    python -m pytest test_cactus_rules.py      (or: python test_cactus_rules.py)
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cactus_rules as rules  # noqa: E402
import cactusassets as ca  # noqa: E402

TOP = 0.7475
FLOOR = TOP + 0.003
OLD = np.array([0.35, -0.06, FLOOR])
NEW = np.array([0.35, 0.13, FLOOR])
UP = (0.0, 0.0, 0.0, 1.0)
STEM = np.array([0.0, 0.0, ca.BALL_H])


def quat(axis, deg: float):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    h = math.radians(deg) / 2
    return (*(a * math.sin(h)), math.cos(h))


def ball_at(base, q=UP):
    """Ball base origin and the cactus on it, bonded as at the start."""
    base = np.asarray(base, float)
    return {"ball": (base, q), "cactus": (base + rules.rot(q) @ STEM, q)}


def seated(dx: float = 0.0, dy: float = 0.0, tilt_deg: float = 0.0, lift: float = 0.0):
    """The ball on the new pot's base layer, shifted and tilted about x, its lowest base point on the layer."""
    q = quat((1, 0, 0), tilt_deg) if tilt_deg else UP
    base = NEW + np.array([dx, dy, ca.LAYER_TOP + lift])
    base[2] += ca.BALL_R0 * math.sin(math.radians(abs(tilt_deg)))
    return ball_at(base, q)


def obs(parts=None, **kw):
    o = {"old_pot": (OLD.copy(), UP), "new_pot": (NEW.copy(), UP), "flex": 0.0, "off_collar": False, "pad_on_ball": False, "held": False,
         "fall": 0.0, "speed": 0.0, "home": True, "top": TOP}
    o.update(ball_at(OLD + np.array([0, 0, ca.BALL_Z0])))
    if parts:
        o.update(parts)
    o.update(kw)
    return o


def run(*observations) -> dict:
    t = rules.Trial()
    m = None
    for o in observations:
        m = t.update(o)
    return m


def squeeze(n: int = rules.SQUEEZE_STEPS):
    return [obs(flex=rules.FLEX + 0.0005, held=True, home=False)] * n


def full_trial(end):
    """Squeeze for 1 s, lift the ball out, then the given end state, held for 1 s."""
    lifted = obs(ball_at(OLD + np.array([0, 0, 0.09])), held=True, home=False)
    return [obs(), *squeeze(), lifted, *[end] * rules.HOME_STEPS]


# -- tiers ---------------------------------------------------------------------------------------------------------

def test_full_score():
    m = run(*full_trial(obs(seated())))
    assert abs(m["depth"] - 0.015) < 1e-3 and m["off"] < 1e-6 and m["tilt"] < 1e-6
    assert m["score"] == 1.0


def test_off_centre_tiers():
    assert run(*full_trial(obs(seated(dx=0.0035))))["score"] == 1.0
    assert run(*full_trial(obs(seated(dx=0.0055))))["score"] == 0.75
    assert run(*full_trial(obs(seated(dy=-0.0085))))["score"] == 0.5


def test_tilt_tiers():
    m = run(*full_trial(obs(seated(tilt_deg=4.0))))
    assert m["tilt"] < 0.008 and m["score"] == 1.0
    m = run(*full_trial(obs(seated(dy=0.0036, tilt_deg=7.5))))  # base set back so the top edge stays centred
    assert 0.008 < m["tilt"] <= 0.015 and m["score"] == 0.75, m
    m = run(*full_trial(obs(seated(tilt_deg=12.0))))
    assert m["tilt"] > 0.015 and m["score"] == 0.5


def test_depth_tiers():
    base = {"fault": False, "whole": True, "out": True, "released": True, "max_risen": 0.09, "on_layer": True,
            "upright": True, "resting": True, "held": False, "home": True, "on_table": True, "off": 0.0, "tilt": 0.0}
    for depth, want in ((0.015, 1.0), (0.010, 1.0), (0.020, 1.0), (0.007, 0.75), (0.024, 0.75), (0.003, 0.5),
                        (0.028, 0.5)):
        assert rules.tier({**base, "depth": depth}) == want, depth


def test_not_on_layer_is_half():
    # standing on the mat beside the pots, upright and whole
    assert run(*full_trial(obs(ball_at([0.30, 0.03, FLOOR]))))["score"] == 0.5
    # hung up on the new pot's rim, 3 cm above the layer, still upright
    assert run(*full_trial(obs(seated(lift=0.03))))["score"] == 0.5


def test_quarter():
    risen = obs(ball_at(OLD + np.array([0, 0, ca.BALL_Z0 + 0.015])), home=False)
    assert run(obs(), *squeeze(), risen)["score"] == 0.25
    # it sank back afterwards: the rise was still made
    assert run(obs(), *squeeze(), risen, obs())["score"] == 0.25
    # squeezed but never risen 1 cm
    assert run(obs(), *squeeze(), obs(ball_at(OLD + np.array([0, 0, ca.BALL_Z0 + 0.008]))))["score"] == 0.0


# -- zeros ---------------------------------------------------------------------------------------------------------

def test_nothing_done():
    assert run(obs())["score"] == 0.0
    assert run(obs(), *squeeze(), obs())["score"] == 0.0


def test_pulled_out_of_a_bound_pot():
    risen = obs(ball_at(OLD + np.array([0, 0, ca.BALL_Z0 + 0.011])), held=True, home=False)
    m = run(obs(), risen, obs(seated()))
    assert m["score"] == 0.0 and m["fault"]
    # a squeeze 1 step short of 1 s does not free it
    m = run(obs(), *squeeze(rules.SQUEEZE_STEPS - 1), risen, obs(seated()))
    assert m["score"] == 0.0
    # a squeeze that moves the wall in less than 3 mm does not count
    weak = [obs(flex=rules.FLEX - 0.0002, held=True)] * 40
    assert run(obs(), *weak, risen, obs(seated()))["score"] == 0.0
    # a squeeze broken in the middle starts again
    broken = squeeze(10) + [obs()] + squeeze(10)
    assert run(obs(), *broken, risen, obs(seated()))["score"] == 0.0


def test_touched_outside_the_collar():
    assert run(*full_trial(obs(seated())), obs(seated(), off_collar=True), obs(seated()))["score"] == 1.0  # one step
    m = run(obs(), obs(off_collar=True), obs(off_collar=True), *full_trial(obs(seated()))[1:])
    assert m["score"] == 0.0


def test_finger_on_the_root_ball():
    assert run(*full_trial(obs(seated())), obs(seated(), pad_on_ball=True), obs(seated()))["score"] == 1.0  # one step
    m = run(obs(), obs(pad_on_ball=True), obs(pad_on_ball=True), *full_trial(obs(seated()))[1:])
    assert m["score"] == 0.0 and m["fault"]


def test_finger_on_ball_geometry():
    in_pot = lambda p: np.asarray(p) + [0, 0, ca.BALL_Z0]  # noqa: E731  the ball seated in the old pot
    top = (0.01, 0.0, ca.BALL_H)
    assert rules.pad_on_ball_point(top, in_pot(top))                       # a finger on the soil face
    edge = (ca.BALL_R1 - 0.0001, 0.0, ca.BALL_H - 0.0002)
    assert not rules.pad_on_ball_point(edge, in_pot(edge))                 # through the squeezed wall at its top edge
    side = (ca.BALL_R0 + 0.001, 0.0, 0.02)
    assert not rules.pad_on_ball_point(side, in_pot(side))                 # through the wall, lower down
    lifted = np.array(side) + [0, 0, 0.09]                                  # the same side, the ball lifted out
    assert rules.pad_on_ball_point(side, lifted)


def test_old_pot_lifted():
    up = obs(ball_at(OLD + np.array([0, 0, 0.05])), old_pot=(OLD + [0, 0, 0.021], UP), held=True, home=False)
    assert run(obs(), *squeeze(), up, obs(seated()))["score"] == 0.0
    ok = obs(ball_at(OLD + np.array([0, 0, 0.05])), old_pot=(OLD + [0, 0, 0.015], UP), held=True, home=False)
    assert run(obs(), *squeeze(), ok, *[obs(seated())] * rules.HOME_STEPS)["score"] == 1.0


def test_fell_or_lay_over():
    assert run(*full_trial(obs(seated(), fall=0.031)))["score"] == 0.0
    assert run(*full_trial(obs(seated(), fall=0.02)))["score"] == 1.0
    lying = obs(ball_at([0.30, 0.03, FLOOR + 0.04], quat((1, 0, 0), 80)), home=False)
    assert run(*full_trial(lying), obs(seated()))["score"] == 0.0


def test_new_pot_moved_or_tipped():
    moved = obs(seated(dx=0.021), new_pot=(NEW + [0.021, 0, 0], UP))
    assert run(*full_trial(moved))["score"] == 0.0
    nudged = obs(seated(dx=0.01), new_pot=(NEW + [0.01, 0, 0], UP))
    assert run(*full_trial(nudged))["score"] == 1.0
    tipped = obs(seated(), new_pot=(NEW, quat((1, 0, 0), 20)))
    assert run(*full_trial(tipped))["score"] == 0.0


def test_left_the_table():
    off = obs(ball_at([0.30, 0.70, TOP - 0.3]), home=False)
    assert run(*full_trial(off), obs(seated()))["score"] == 0.0


def test_torn_off_its_ball():
    parts = seated()
    parts["cactus"] = (parts["cactus"][0] + [0.006, 0, 0], UP)
    assert run(*full_trial(obs(parts)))["score"] == 0.0


def test_end_state_conditions():
    assert run(*full_trial(obs(seated(), held=True)))["score"] == 0.0        # still held at the end
    # freed and risen, but grippers not home, or still moving: 0.25 (the 0.5 conditions not met)
    assert run(*full_trial(obs(seated(), home=False)))["score"] == 0.25
    assert run(*full_trial(obs(seated(), speed=0.05)))["score"] == 0.25


def test_quarter_is_monotonic():
    # freed, fully out and upright on the mat, but the grippers not home: still 0.25, not 0
    on_mat = obs(ball_at([0.30, 0.03, FLOOR]), home=False)
    assert run(*full_trial(on_mat))["score"] == 0.25
    # the same, still held: 0
    assert run(*full_trial(obs(ball_at([0.30, 0.03, FLOOR]), home=False, held=True)))["score"] == 0.0
    # never freed (no squeeze) but nothing rose: 0
    assert run(obs(), *[obs(home=False)] * 5)["score"] == 0.0


def test_home_needs_fingertips_clear_for_1s():
    far = [np.array([0.0, 0.5, 1.2])] * 4
    def at_end(tips, n=rules.HOME_STEPS):
        return [obs(seated(), tcp_home=True, tips=tips)] * n
    assert run(*full_trial(obs(seated()))[:-rules.HOME_STEPS], *at_end(far))["score"] == 1.0
    # one step short of 1 s
    assert run(*full_trial(obs(seated()))[:-rules.HOME_STEPS], *at_end(far, rules.HOME_STEPS - 1))["score"] == 0.25
    # a fingertip 2 cm above the cactus's crown (its TCP may still be near the start pose)
    crown = NEW + np.array([0, 0, ca.LAYER_TOP + ca.BALL_H + ca.CACTUS_H + 0.02])
    assert run(*full_trial(obs(seated()))[:-rules.HOME_STEPS], *at_end([crown] + far[1:]))["score"] == 0.25
    # 3.5 cm off the new pot's rim: clear
    rim = NEW + np.array([ca.NEW_RIM_R + 0.035, 0, ca.NEW_H])
    assert run(*full_trial(obs(seated()))[:-rules.HOME_STEPS], *at_end([rim] + far[1:]))["score"] == 1.0
    # TCP not at its start pose
    assert run(*full_trial(obs(seated()))[:-rules.HOME_STEPS],
               *[obs(seated(), tcp_home=False, tips=far)] * rules.HOME_STEPS)["score"] == 0.25


def test_cylinder_gap():
    assert abs(rules.cylinder_gap((0.05, 0, 0.01), (0, 0, 0), UP, 0.04, 0.0, 0.02) - 0.01) < 1e-9
    assert abs(rules.cylinder_gap((0.0, 0, 0.05), (0, 0, 0), UP, 0.04, 0.0, 0.02) - 0.03) < 1e-9
    assert rules.cylinder_gap((0.01, 0, 0.01), (0, 0, 0), UP, 0.04, 0.0, 0.02) == 0.0


def test_collar_rule_geometry():
    zc = (ca.COLLAR_Z0 + ca.COLLAR_Z1) / 2
    assert rules.collar_point((ca.COLLAR_R_OUT, 0.0, zc))           # on the band's outside
    assert rules.collar_point((0.0, ca.COLLAR_R_OUT, ca.COLLAR_Z1))  # on its top edge
    assert not rules.collar_point((0.026, 0.0, ca.COLLAR_Z1 + 0.008))  # the body above it
    assert not rules.collar_point((0.022, 0.0, 0.003))                  # the body below it
    assert not rules.collar_point((0.0, 0.0, ca.CACTUS_H))              # the crown


if __name__ == "__main__":
    names = [n for n in dir() if n.startswith("test_")]
    for n in names:
        globals()[n]()
    print(f"{len(names)} checks passed")
