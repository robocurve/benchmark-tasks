"""Offline checks of the router-table scorer: step-by-step joint histories, near misses included, no simulator.

    python test_task.py        (with the yam_arena package importable; prints "all pass")
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, pv = m.kit, m.pv
PARK = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
TH0 = m.START[2]
PHI0 = math.atan2(*(m.BIT - m.PIN - m.C_L)[::-1])  # where the pivot puts the bearing on the curve


DOWN = (1.0, 0.0, 0.0, 0.0)  # a gripper pointing straight down (its pad tips below the tool point)


def st(pos, tip=0.0, q=(0, 0, 0, 1.0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(q, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - np.array([0.01, 0.01, tip]), high=p + 0.01)


class Trial:
    """One trial's joints; each call to step() is one control step through the kit's record and the scorer."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.j = {"board_turn": TH0, "board_x": m.START[0], "board": m.START[1]}

    def step(self, left=None, right=None, opening=0.0, **j):
        self.j.update(j)
        for k, v in self.j.items():
            kit._JOINTS[f"router_table/{k}"] = kit.JointState(v, 0.0, False)
        pv.step(m.KEY, dict(self.j))
        g = dict(PARK)
        if left is not None:
            g["left_gripper"] = left
        if right is not None:
            g["right_gripper"] = right
        for k in g:
            kit._OPENING[k] = opening  # closed, as in the demo, unless given
        objs = {"router_table": st((m.PIN[0], m.PIN[1], m.TOP)),
                **{k: st(p, m.PAD_TIP_PAST_TCP, DOWN) for k, p in g.items()}}
        ok = m.success(objs)
        return m.score(objs), ok

    def press(self):
        """The pads on the board's press point now."""
        th, s = self.j["board_turn"], np.array([self.j["board_x"], self.j["board"]])
        return m._press_at(th, s)

    def move(self, n=10, press=True, **target):
        """Joints moved linearly to `target` over n steps, the pads on the board throughout."""
        a = dict(self.j)
        out = None
        for k in range(1, n + 1):
            self.j.update({key: a[key] + (v - a[key]) * k / n for key, v in target.items()})
            out = self.step(left=self.press() if press else None)
        return out

    def slides_for(self, world_shift, turn=None):
        """Slides that move the board by a world shift from where it is (at turn `turn`)."""
        th = self.j["board_turn"] if turn is None else turn
        return np.array([self.j["board_x"], self.j["board"]]) + m._rot(-th) @ np.asarray(world_shift, float)

    def rest(self, n=25, **kw):
        out = None
        for _ in range(n):
            out = self.step(**kw)
        return out


def check(got, want, what):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"{what}: got {got}, want {want}")
    print(f"ok   {what}: {got}")


def brace(t):
    """Slide the board from its start onto the pin (pure slide, start turn)."""
    t.rest(2)
    t.move(12, board_x=0.0, board=0.0)


def pivot(t, to=0.0):
    t.move(14, board_turn=to)


def feed(t, phi_end=-m.ALPHA - math.radians(4.0), n=60, into=0.001):
    for phi in np.linspace(PHI0, phi_end, n):
        s = m._feed_slides(phi, into=into)
        t.j.update(board_turn=0.0, board_x=float(s[0]), board=float(s[1]))
        t.step(left=t.press())


def braced_slides(turn):
    """Slides that keep the curve on both the pin and the bearing at board turn `turn` (the board turned about the
    curve's center): the bearing then meets the curve at angle PHI0 - turn."""
    return m._rot(-turn) @ m.C_L - m.C_L


LEAD = PHI0 - m.ALPHA  # the turn whose braced pose puts the bearing right at the curve's leading end


def slides_at(phi, turn, into=0.001):
    """Slides (at board turn `turn`) that put the curve's contact with the bearing at angle `phi` on the curve."""
    c = m.BIT - (m.R + m.RB - into) * np.array([math.cos(phi + turn), math.sin(phi + turn)])
    return m._rot(-turn) @ (c - m.PIN) - m.C_L


def brace_at(t, turn):
    """Slide the board onto the pin (at turn + 14 deg) where a pivot to `turn` brings the bearing on, then pivot."""
    s = braced_slides(turn)
    t.rest(2)
    t.move(12, board_turn=turn + TH0, board_x=float(s[0]), board=float(s[1]))
    t.move(14, board_turn=turn)


def feed_at(t, turn, phi_from, phi_to, n=60):
    for phi in np.linspace(phi_from, phi_to, n):
        s = slides_at(phi, turn)
        t.j.update(board_turn=turn, board_x=float(s[0]), board=float(s[1]))
        t.step(left=t.press())


def finish_at(t, turn, home=True):
    s = slides_at(-m.ALPHA - math.radians(4.0), turn, into=-0.004) + m._rot(-turn) @ np.array([0.0, 0.03])
    t.move(10, board_x=float(s[0]), board=float(s[1]))
    p = t.press()
    t.step(left=(p[0], p[1], p[2] + 0.10))
    return t.rest(25) if home else t.rest(25, left=(0.30, 0.25, m.TOP + 0.15))


def whole_one_pass(home=True):
    t = Trial()
    brace_at(t, LEAD)
    feed_at(t, LEAD, m.ALPHA, -m.ALPHA - math.radians(4.0))
    return t, finish_at(t, LEAD, home)


def finish(t, home=True, past=True):
    if past:
        s = m._feed_slides(-m.ALPHA - math.radians(4.0), into=-0.004) + np.array([0.0, 0.03])
        t.move(10, board_x=float(s[0]), board=float(s[1]))
    p = t.press()
    t.step(left=(p[0], p[1], p[2] + 0.10))  # lift off
    return t.rest(25) if home else t.rest(25, left=(0.30, 0.25, m.TOP + 0.15))


def test_geometry():
    j = {"board_turn": TH0, "board_x": 0.0, "board": 0.0}
    check(float(m.braced(j)), 1.0, "braced pose at the start turn is on the pin")
    check(float(m.bit_dist(j) > m.RC), 1.0, "braced pose at the start turn is clear of the cutter")
    j0 = {"board_turn": 0.0, "board_x": 0.0, "board": 0.0}
    check(float(m.on_bearing(j0) and not m.braced(j0)), 1.0, "turned to 0 the curve rides the bearing, off the pin")
    js = {"board_turn": TH0, "board_x": m.START[0], "board": m.START[1]}
    check(float(m.pin_gap(js) > 0.02 and m.bit_dist(js) > 0.04), 1.0, "start: 2.4 cm off the pin, 4.4 cm off the bit")
    check(float(abs(m.ARC - 0.2579) < 0.001), 1.0, "curve is 25.8 cm long")


def test_do_nothing():
    t = Trial()
    check(t.rest(60)[0], 0.0, "do nothing")


def test_demo_plan_offline():
    """The demo's waypoints (the board assumed to follow the pads exactly): no grab, no hands near the bit, the whole
    curve routed, past the bit: 1.0. The sim decides whether the board really follows; this checks the plan."""
    plan = m.demo()["left"]
    t = Trial()
    t.rest(2)
    s_pin = None
    for w in plan:
        lab = getattr(w, "label", "")
        if lab in ("brace the board against the starting pin", "settle on the pin"):
            s_pin = m.BRACED_LEAD
        if lab == "brace the board against the starting pin":
            t.move(6, board_turn=TH0, board_x=float(m.BRACED_LEAD[0]), board=float(m.BRACED_LEAD[1]))
        elif lab == "pivot about the pin into the bit":
            t.move(3, board_turn=max(m.LEAD, t.j["board_turn"] - (TH0 - m.LEAD) / 14))
    assert s_pin is not None
    feed_at(t, m.LEAD, m.ALPHA, -m.ALPHA - math.radians(4.0), n=400)  # the sim feeds ~0.6 mm a step
    sc, ok = finish_at(t, m.LEAD)
    check(sc, 1.0, f"demo plan through the scorer (routed {m.routed_cm():.1f} cm)")
    check(float(m.bit_dist({"board_turn": TH0, "board_x": float(m.BRACED_LEAD[0]), "board": float(m.BRACED_LEAD[1])})
                > m.RC), 1.0, "braced at the start turn, before the pivot, the board is clear of the cutter")
    check(float(abs(m.LEAD - LEAD) < 1e-12), 1.0, "task and test agree on the leading-end turn")


def test_old_demo_start_is_not_whole():
    """review, Nov 12: the old demo's start (bearing on 4.1 cm in from the leading end) leaves that stretch unrouted."""
    t = Trial()
    brace(t)
    pivot(t)
    feed(t)
    sc, ok = finish(t)
    check(sc, 0.75, f"braced, pivoted, fed round and past, but {m.ARC * 100 - m.routed_cm():.1f} cm unrouted at the "
                    f"leading end (routed {m.routed_cm():.1f} cm)")
    check(float(ok), 0.0, "no success")


def test_whole_curve_one_pass():
    t, (sc, ok) = whole_one_pass()
    check(sc, 1.0, f"braced so the bearing meets the leading end, pivoted, fed round and past (routed "
                   f"{m.routed_cm():.1f} of {m.ARC * 100:.1f} cm)")
    check(float(ok), 1.0, "success() agrees")
    check(float(m._H.hands or m._H.grab), 0.0, "no hands near the bit, no grab")


def test_whole_curve_two_passes():
    """review, Nov 12: a safe second pass braced on the pin is allowed: the old start, pulled back off the bit, then
    braced again at the leading end and fed through."""
    t = Trial()
    brace(t), pivot(t), feed(t)
    finish(t)
    first = m.routed_cm()
    back = t.slides_for((-0.06, 0.0))  # pulled 6 cm straight back toward the robot, off the bit
    t.move(8, board_x=float(back[0]), board=float(back[1]))
    s = braced_slides(LEAD)
    away = s + m._rot(-(LEAD + TH0)) @ np.array([-0.06, 0.0])
    t.move(20, board_turn=LEAD + TH0, board_x=float(away[0]), board=float(away[1]))  # slid round, 6 cm back
    check(float(m._H.grab), 0.0, "carried back round the bit without touching it")
    t.move(12, board_x=float(s[0]), board=float(s[1]))  # onto the pin
    t.move(14, board_turn=LEAD)  # pivoted in about it
    feed_at(t, LEAD, m.ALPHA, -m.ALPHA - math.radians(4.0))
    sc, ok = finish_at(t, LEAD)
    check(sc, 1.0, f"first pass {first:.1f} cm, second pass braced on the pin at the leading end: whole curve")
    check(float(m._H.grab), 0.0, "the second, braced entry is not a grab")


def test_second_pass_off_pin_is_a_grab():
    t = Trial()
    brace(t), pivot(t), feed(t)
    finish(t)
    s = slides_at(m.ALPHA - 0.01, LEAD, into=-0.03)
    t.move(5, press=False, board_turn=LEAD, board_x=float(s[0]), board=float(s[1]))
    s = slides_at(m.ALPHA - 0.01, LEAD)
    t.move(8, board_x=float(s[0]), board=float(s[1]))  # straight into the bit at the leading end, off the pin
    check(float(m._H.grab), 1.0, "second pass fed in off the pin: grab")
    check(t.rest(25)[0], 0.0, "and the whole run scores 0")


def test_whole_curve_left_on_bit():
    t = Trial()
    brace_at(t, LEAD)
    feed_at(t, LEAD, m.ALPHA, -m.ALPHA + 0.003)
    p = t.press()
    t.step(left=(p[0], p[1], p[2] + 0.10))
    check(t.rest(25)[0], 0.75, f"routed {m.routed_cm():.1f} cm, board left on the bit")


def test_not_home():
    t, (sc, ok) = whole_one_pass(home=False)
    check(sc, 1.0, "done right, arms not home (not scored)")
    check(float(ok), 1.0, "success() agrees")
    check(float(hasattr(m, "REQUIRE_ARMS_HOME") or hasattr(m, "REQUIRED")), 0.0, "no arm-home rule, no 18 cm full")


def test_still_pressing():
    t = Trial()
    brace(t), pivot(t), feed(t)
    s = m._feed_slides(-m.ALPHA - math.radians(4.0), into=-0.004) + np.array([0.0, 0.03])
    t.move(10, board_x=float(s[0]), board=float(s[1]))
    check(t.rest(25, left=t.press())[0], 0.0, "board routed and past but still pressed at the end")
    p = t.press()
    away = t.rest(25, left=(p[0], p[1], p[2] + 0.10))[0]  # what the board earns with the hand lifted away
    check(float(away > 0), 1.0, f"hand lifted 10 cm: {away}")
    check(t.rest(25, left=(p[0], p[1], p[2] + 0.0045))[0], 0.0,
          "pads resting on the board's top (0.5 mm over it, within the 2 mm contact tolerance): still on it")
    check(t.rest(25, left=(p[0], p[1], p[2] + 0.012))[0], away,
          "pads hovering 8 mm over the board, touching nothing: let go (the old 1.5 cm zone scored 0)")
    check(t.rest(25, left=(p[0], p[1], p[2] + 0.007), opening=1.0)[0], away,
          "open hand hovering 3 mm over the board, touching nothing: let go")


def test_hand_against_the_side():
    """A hand pushing the board's back edge from behind (fingers down beside it) is on it; 3 mm off it is not."""
    t = Trial()
    brace(t), pivot(t), feed(t)
    s = m._feed_slides(-m.ALPHA - math.radians(4.0), into=-0.004) + np.array([0.0, 0.03])
    t.move(10, board_x=float(s[0]), board=float(s[1]))
    th, sl = t.j["board_turn"], np.array([t.j["board_x"], t.j["board"]])
    z = m.TOP + m.GAP + m.T / 2 + m.PAD_TIP_PAST_TCP  # pad tips at the board's mid-thickness
    half_x = 0.0032 * 2  # a closed hand's tip boxes reach 6.4 mm out along x each way from the tool point
    away = t.rest(25)[0]
    for gap, want, what in ((0.001, 0.0, "fingers 1 mm off the back edge: touching, still on it"),
                            (0.003, away, "fingers 3 mm off the back edge: not touching, let go")):
        w = m._board_world(th, sl, (m.BACK - half_x - gap, float(m.C_L[1])))
        check(t.rest(25, left=(float(w[0]), float(w[1]), z))[0], want, what)
    check(float(len(m.on_board(m._TOUCH["objs"]))), 0.0, "on_board() agrees")


def test_straight_in():
    """The trap: the curve fed straight into the bit, never touching the pin."""
    t = Trial()
    t.rest(2)
    # turn to 0 first, well away from both, then slide the curve straight onto the bearing from the robot's side
    t.move(10, board_turn=0.0, board_x=-0.04, board=0.0)
    check(float(m.braced(t.j)), 0.0, "clear of the pin")
    t.move(10, board_x=0.0)
    check(float(m._H.grab), 1.0, "fed into the bit off the pin is a grab")
    feed(t)
    check(finish(t)[0], 0.0, "grab, then fed round and past perfectly: still 0 (irreversible)")


def test_feed_along_then_in():
    """Slide the board along to the left first, then into the bit beside the pin (off it)."""
    t = Trial()
    t.rest(2)
    t.move(10, board_turn=0.0, board_x=-0.03, board=0.06)
    t.move(10, board_x=0.004)
    check(float(m._H.grab), 1.0, "curve pushed into the bit to the left of the pin: grab")
    check(t.rest(30)[0], 0.0, "scores 0")


def test_pivot_only():
    t = Trial()
    brace(t)
    pivot(t)
    p = t.press()
    t.step(left=(p[0], p[1], p[2] + 0.10))
    check(t.rest(25)[0], 0.25, "braced and pivoted onto the bearing, then let go")


def test_brace_no_pivot():
    t = Trial()
    brace(t)
    p = t.press()
    t.step(left=(p[0], p[1], p[2] + 0.10))
    check(t.rest(25)[0], 0.0, "braced on the pin only")
    t2 = Trial()
    brace(t2)
    pivot(t2, to=math.radians(4.0))  # in the cutter but not yet on the bearing
    p = t2.press()
    t2.step(left=(p[0], p[1], p[2] + 0.10))
    check(t2.rest(25)[0], 0.0, "pivoted into the cutter but short of the bearing")


def test_pivot_away():
    t = Trial()
    brace(t)
    pivot(t, to=math.radians(30.0))  # swung the wrong way about the pin
    check(t.rest(25)[0], 0.0, "turned the wrong way about the pin")


def test_halfway():
    t = Trial()
    brace(t), pivot(t)
    feed(t, phi_end=PHI0 - 0.11 / m.R, n=30)
    s = m._feed_slides(PHI0 - 0.11 / m.R) - np.array([0.03, 0.0])  # pulled back off the bit
    t.move(10, board_x=float(s[0]), board=float(s[1]))
    p = t.press()
    t.step(left=(p[0], p[1], p[2] + 0.10))
    sc = t.rest(25)[0]
    check(sc, 0.5, f"routed {m.routed_cm():.1f} cm then pulled back")


def test_full_not_past():
    t = Trial()
    brace(t), pivot(t)
    feed(t, phi_end=-m.ALPHA + 0.01, n=60)
    check(finish(t, past=False)[0], 0.75, f"routed {m.routed_cm():.1f} cm (over 18), board left on the bit")


def test_climb():
    """Pivoted onto the bearing at the trailing end and fed the wrong way (left to right): routes nothing."""
    t = Trial()
    brace(t), pivot(t)
    phi = PHI0
    for phi in np.linspace(PHI0, PHI0 + 0.03 / m.R, 15):  # the contact moving toward the leading end
        s = m._feed_slides(phi)
        t.j.update(board_turn=0.0, board_x=float(s[0]), board=float(s[1]))
        t.step(left=t.press())
    check(float(m.routed_cm() <= 0.2), 1.0, f"climb feed routes nothing ({m.routed_cm():.1f} cm)")


def test_reentry_off_pin():
    """A proper start, then the board pulled off the bit and pushed back in further along, off the pin."""
    t = Trial()
    brace(t), pivot(t)
    feed(t, phi_end=PHI0 - 0.05 / m.R, n=20)
    s = m._feed_slides(PHI0 - 0.05 / m.R) - np.array([0.03, 0.0])
    t.move(8, board_x=float(s[0]), board=float(s[1]))
    check(float(m._H.cutting), 0.0, "out of the cutter")
    s2 = m._feed_slides(PHI0 - 0.07 / m.R)
    t.move(8, board_x=float(s2[0]), board=float(s2[1]))
    check(float(m._H.grab), 1.0, "pushed back into the bit off the pin: grab")
    feed(t)
    check(finish(t)[0], 0.0, "and stays 0")


def test_shove_onto_pin_and_bit():
    """Pin contact and cutter entry in the same step (a shove) is not a braced entry."""
    t = Trial()
    t.rest(2)
    t.move(1, board_x=0.0, board=0.0, board_turn=math.radians(2.0))
    check(float(m._H.grab), 1.0, "shoved onto pin and bit at once")


def test_hands():
    t = Trial()
    t.rest(2)
    t.step(left=(m.BIT[0] - 0.06, m.BIT[1], m.TOP + 0.04))
    check(float(m._H.hands), 1.0, "tool point 6 cm from the bit, pads low")
    brace(t), pivot(t), feed(t)
    check(finish(t)[0], 0.0, "hands near the bit is irreversible")
    t2 = Trial()
    t2.rest(2)
    t2.step(left=(m.BIT[0] - 0.06, m.BIT[1], m.TOP + 0.10))
    check(float(m._H.hands), 0.0, "high over the bit is fine")


def test_reset():
    t = Trial()
    t.rest(2)
    t.move(1, board_x=0.0, board=0.0, board_turn=math.radians(2.0))
    check(float(m._H.grab), 1.0, "grab in one trial")
    t2 = Trial()
    t2.rest(2)
    check(float(m._H.grab or m._H.started or m._H.routed.any()), 0.0, "next trial starts clean (kit.episode())")
    brace_at(t2, LEAD)
    feed_at(t2, LEAD, m.ALPHA, -m.ALPHA - math.radians(4.0))
    check(finish_at(t2, LEAD)[0], 1.0, "and can score 1.0")


def test_kit_lock_agrees():
    """pivotkit's own record: the pivot was made while engaged (turned about the pin), with no slide on it."""
    t = Trial()
    brace(t), pivot(t)
    check(float(pv.turned_deg(None, m.KEY) < -12.0), 1.0, f"turned {pv.turned_deg(None, m.KEY):.1f} deg about the pin")
    check(float(pv.slid(None, m.KEY)), 0.0, "no slide while braced")


def test_model():
    """The model choices the demo relies on: the bearing's collider is frictionless, the pin's is not; the turn's damping
    is physical on the pin and 10 times that off it; the slides act at the board's center."""
    usd = kit._usda(m.OBJECTS[1])
    check(float(usd.count("material:binding:physics") == 1 and "Slick" in usd), 1.0, "only the bearing is slick")
    on = {"board_turn": TH0, "board_x": 0.0, "board": 0.0}
    off = {"board_turn": TH0, "board_x": m.START[0], "board": m.START[1]}
    check(float(m.braced(on) and not m.braced(off)), 1.0, "braced on the pin, not at the start")
    d_on, d_off = m.drag(on)[1], m.drag(off)[1]
    check(float(0.03 < d_on < 0.1 and d_off > 5 * d_on), 1.0, f"turn damping {d_on:.3f} on the pin, {d_off:.3f} off it")
    links = {lk.name: lk for lk in m.OBJECTS[0].links}
    check(float(abs(links["board"].anchor[0] - m.CENTRE[0]) < 1e-9), 1.0, "slides act at the board's center")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
    print("all pass")
