"""Offline checks of the wrench model and scorer: step-by-step joint histories, near misses included, no simulator.

    python test_task.py        (with the yam_arena package importable; prints "all pass")
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
PARK = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
PER_MM = 2 * math.pi / (m.LEAD * 1000)  # worm radians per mm of jaw travel
SNUG = 1.0 * PER_MM  # the worm turned until the 1 mm of play is gone


def st(pos):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


class Trial:
    """One trial's joints; each step() is one control step through the model and the scorer."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.j = {"nut": m.NUT0, "lift": 0.0, "wr_turn": m.TURN0, "wr_x": 0.0, "wrench": 0.0,
                  "jaw": m.AF / 2 - m.OPEN0, "wheel": 0.0}
        self.g = dict(PARK)
        self.op = {g: 1.0 for g in PARK}  # finger openings (0 closed, 1 open)
        self.s = None
        self.step()  # the trial's first step, everything at rest (the model takes its start values here)

    def step(self, n=1, **j):
        self.j.update(j)
        for _ in range(n):
            mod = m.model_step(dict(self.j))
            self.j["nut"] = mod.nut  # the nut's law sets it to the model's angle
            for k, v in self.j.items():
                kit._JOINTS[f"base/{k}"] = kit.JointState(v, 0.0, False)
            objs = {g: st(p) for g, p in self.g.items()}
            kit._OPENING.update(self.op)
            m.success(objs)
            self.s = m.score(objs)
        return self

    def turn(self, deg, steps=20):
        """Swing the wrench `deg` degrees (ccw +) from where it is, in steps."""
        t0 = self.j["wr_turn"]
        for k in range(1, steps + 1):
            self.step(wr_turn=t0 + math.radians(deg) * k / steps)
        return self

    def take_off(self, dist=0.05):
        self.step(3, lift=0.03)
        self.step(3, wr_x=dist)
        return self.step(3, lift=0.0)

    def hand_at_handle(self, side="left_gripper", opening=0.19):
        p = m.wrench_point(self.j, (m.GRIP_X, m.HANDLE_Y))
        self.g[side] = (p[0], p[1], m.TOP + m.NZ + self.j["lift"] + 0.012)
        self.op[side] = opening  # 0.19: closed on the 18 mm handle
        return self

    def home(self):
        self.g = dict(PARK)
        self.op = {g: 1.0 for g in PARK}
        return self.step(5)


def full(t, deg=66.0):
    t.step(3).step(3, wheel=SNUG)
    t.turn(deg + math.degrees(m.BREAK))
    t.step(3, wr_turn=t.j["wr_turn"] - m.BREAK)  # let go: the flex springs back, the nut stays
    return t.take_off().home()


CASES = []


def case(fn):
    CASES.append(fn)
    return fn


@case
def do_nothing():
    t = Trial().step(60)
    assert t.s == 0.0, t.s


@case
def jaw_closing_does_not_turn_the_nut():
    """Review, Nov 10: the wrench nudged 13 degrees either way inside the loose jaw's play, then the jaw snugged (in one
    step or gradually): the jaw squares the wrench onto the flats, the nut does not turn, nothing is latched."""
    for deg in (13, -10):
        for one_step in (True, False):
            t = Trial()
            t.turn(deg, 10)
            if one_step:
                t.step(wheel=SNUG)
            else:
                for k in range(1, 21):
                    t.step(wheel=SNUG * k / 20)
            mo = m.model()
            assert abs(m.loosened_deg(mo)) < 0.5 and not mo.rounded and not mo.sprung, (deg, one_step, m.loosened_deg(mo))
            assert t.s == 0.0, t.s


@case
def nudged_then_snugged_then_full_job():
    """The same nudge before snugging, then the job done right: full credit."""
    t = Trial()
    t.turn(-8, 10)
    t = full(t)
    assert t.s == 1.0, (t.s, m.loosened_deg(m.model()))


@case
def demo_like_full_job():
    t = full(Trial())
    assert t.s == 1.0, (t.s, m.loosened_deg(m.model()))


@case
def arms_not_home_still_full_credit():
    """Review, Nov 12: the score must not depend on the arms going back to their start."""
    t = full(Trial())
    t.g["right_gripper"] = (0.25, -0.15, m.TOP + 0.15)
    t.g["left_gripper"] = (0.40, 0.20, m.TOP + 0.10)
    t.step(3)
    assert t.s == 1.0, t.s


@case
def open_hand_resting_near_wrench_is_not_holding():
    """An open hand left 3 cm from the laid-down wrench is not holding it (no proximity penalty); a closed one is."""
    t = full(Trial())
    t.hand_at_handle(opening=0.9).step(3)
    assert t.s == 1.0, t.s
    t.hand_at_handle(opening=0.19).step(3)
    assert t.s == 0.0, t.s


@case
def loose_jaw_pull_rounds_the_corners():
    t = Trial().step(3)
    t.turn(70)  # never touched the worm: 1 mm of play
    assert m.model().rounded
    assert t.s == 0.0, t.s


@case
def loose_jaw_round_then_snug_and_redo_still_zero():
    t = Trial().step(3)
    t.turn(30)
    assert m.model().rounded
    t.step(3, wheel=SNUG)
    t.turn(70)
    t.take_off().home()
    assert t.s == 0.0, t.s  # irreversible


@case
def play_just_under_limit_ok():
    t = Trial().step(3, wheel=0.65 * PER_MM)  # 0.35 mm of play left
    t.turn(66 + math.degrees(m.BREAK + m.play_angle(0.00035)))
    assert not m.model().rounded
    t.step(3, wr_turn=t.j["wr_turn"] - m.BREAK).take_off().home()
    assert t.s == 1.0, (t.s, m.loosened_deg(m.model()))


@case
def play_just_over_limit_rounds():
    t = Trial().step(3, wheel=0.5 * PER_MM)  # 0.5 mm of play
    t.turn(20)
    assert m.model().rounded and t.s == 0.0


@case
def jaw_reopened_before_a_later_pull_is_caught():
    """Review, Nov 12: play must be tracked over every pull, not only before the first. Snug, a good first pull, the
    wrench reset off the nut, then the worm rolled back 0.7 mm and a second pull: 0, and still 0 after re-snugging."""
    t = Trial().step(3, wheel=SNUG)
    t.turn(30 + math.degrees(m.BREAK))
    assert not m.model().rounded
    t.step(3, lift=0.03)
    t.step(3, wr_turn=t.j["wr_turn"] - math.radians(40))
    t.step(3, wheel=0.3 * PER_MM)  # opened again: 0.7 mm of play
    t.step(3, lift=0.0)
    t.turn(30)
    assert m.model().rounded, m.loosened_deg(m.model())
    t.step(3, wheel=SNUG)
    t.turn(40 + math.degrees(m.BREAK))
    t.step(3, wr_turn=t.j["wr_turn"] - m.BREAK).take_off().home()
    assert t.s == 0.0, t.s


@case
def jaw_readjusted_snug_between_pulls_is_fine():
    """Adjusting the worm between pulls is allowed: opened off the nut, snugged again before the next pull, full
    credit."""
    t = Trial().step(3, wheel=SNUG)
    t.turn(30 + math.degrees(m.BREAK))
    t.step(3, lift=0.03)
    t.step(3, wr_turn=t.j["wr_turn"] - math.radians(40))
    t.step(3, wheel=0.3 * PER_MM)
    t.step(3, lift=0.0)
    t.step(3, wheel=SNUG)  # snug again on the flats before pulling
    t.turn(40 + math.degrees(m.BREAK))
    t.step(3, wr_turn=t.j["wr_turn"] - m.BREAK).take_off().home()
    mo = m.model()
    assert not mo.rounded and not mo.sprung and t.s == 1.0, (t.s, m.loosened_deg(mo))


@case
def wrong_pull_loads_movable_jaw():
    t = Trial().step(3, wheel=SNUG)
    t.turn(-10)  # pulled clockwise: the nut forced tighter
    mod = m.model()
    assert mod.sprung and mod.open > m.AF + 0.002, (mod.sprung, mod.open)
    t.turn(80)
    t.take_off().home()
    assert t.s == 0.0, t.s


@case
def small_clockwise_tap_within_flex_is_fine():
    t = Trial().step(3, wheel=SNUG)
    t.turn(-math.degrees(m.BREAK) * 0.9)  # inside the thread's flex: the nut does not move
    t.step(3, wr_turn=m.TURN0)
    assert not m.model().sprung
    t.turn(66 + math.degrees(m.BREAK))
    t.step(3, wr_turn=t.j["wr_turn"] - m.BREAK).take_off().home()
    assert t.s == 1.0, t.s


@case
def swing_back_on_the_nut_counts_as_wrong_pull():
    t = Trial().step(3, wheel=SNUG)
    t.turn(40)
    t.turn(-20)  # ratcheting back with the wrench still on the nut drives it tighter: load on the movable jaw
    assert m.model().sprung and t.s == 0.0


@case
def wiggle_within_breakaway_is_not_progress():
    t = Trial().step(3, wheel=SNUG)
    t.turn(3)
    t.step(3, wr_turn=m.TURN0)
    assert abs(m.loosened_deg(m.model())) < 0.01
    t.take_off().home()
    assert t.s == 0.0, t.s


@case
def partial_loosen():
    t = Trial().step(3, wheel=SNUG)
    t.turn(30 + math.degrees(m.BREAK))
    t.step(3, wr_turn=t.j["wr_turn"] - m.BREAK).take_off().home()
    assert t.s == 0.25, t.s


@case
def full_turn_wrench_left_on_nut():
    t = Trial().step(3, wheel=SNUG)
    t.turn(66 + math.degrees(m.BREAK))
    t.step(3, wr_turn=t.j["wr_turn"] - m.BREAK).home()
    assert t.s == 0.5, t.s


@case
def wrench_set_down_too_close():
    t = Trial().step(3, wheel=SNUG)
    t.turn(66 + math.degrees(m.BREAK))
    t.step(3, wr_turn=t.j["wr_turn"] - m.BREAK)
    t.step(3, lift=0.03).step(3, wr_x=0.02)
    t.step(3, lift=0.0)  # the lift law keeps it up on the stud here; the scorer sees it not laid down either way
    t.home()
    assert t.s == 0.5, t.s


@case
def still_held_at_end_scores_zero():
    t = full(Trial())
    t.hand_at_handle().step(3)
    assert t.s == 0.0, t.s
    t = Trial().step(3, wheel=SNUG)
    t.turn(30 + math.degrees(m.BREAK))
    t.hand_at_handle().step(3)
    assert t.s == 0.0, t.s  # the partial credit needs the hands off too


@case
def lifted_wrench_turned_in_air_does_not_turn_nut():
    t = Trial().step(3)
    t.step(3, lift=0.03)
    t.turn(90)
    mod = m.model()
    assert abs(m.loosened_deg(mod)) < 1e-6 and not mod.rounded
    t.step(3, lift=0.0)
    assert t.s == 0.0


@case
def reseated_wrench_couples_again():
    t = Trial().step(3, wheel=SNUG)
    t.turn(40 + math.degrees(m.BREAK))
    t.step(3, lift=0.03)
    t.step(3, wr_turn=t.j["wr_turn"] - math.radians(50))  # swung back off the nut, as mechanics reset a wrench
    t.step(3, lift=0.0)
    assert m.model().coupled and not m.model().sprung
    t.turn(30 + math.degrees(m.BREAK))
    t.step(3, wr_turn=t.j["wr_turn"] - m.BREAK).take_off().home()
    assert t.s == 1.0, (t.s, m.loosened_deg(m.model()))


@case
def jaw_opened_past_the_corners_spins_and_rounds():
    t = Trial().step(3, wheel=-1.0 * PER_MM)  # 2 mm of play: wider than the nut's corners
    t.turn(40)
    mod = m.model()
    assert mod.rounded and abs(m.loosened_deg(mod)) < 1e-6 and t.s == 0.0


@case
def jaw_cannot_close_past_the_flats():
    t = Trial().step(3, wheel=5 * PER_MM)
    assert abs(m.model().open - m.AF) < 1e-9
    lo, hi = m.wheel_limits(m.model())
    assert abs(hi - SNUG) < 1e-6, hi


@case
def tighten_stop_holds():
    t = Trial().step(3, wheel=SNUG)
    t.turn(-40)
    assert m.loosened_deg(m.model()) >= -math.degrees(m.TIGHT_STOP) - 1e-6


@case
def reset_clears_history():
    Trial().step(3).turn(30)
    assert m.model().rounded
    t = Trial().step(3)
    assert not m.model().rounded and t.s == 0.0


if __name__ == "__main__":
    for fn in CASES:
        fn()
        print("ok", fn.__name__)
    print("all pass")
