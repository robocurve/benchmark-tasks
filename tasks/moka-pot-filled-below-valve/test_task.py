"""Offline checks of the moka pot scorer: hand-made step-by-step states of the pot, the carafe and the grippers, no
simulator. The water in the base is set directly (pourkit's pour model is tested by its pilot, acid-into-water).

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, pk = m.kit, m.pk
HOME = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
BASE = np.array((m.BX, m.BY, m.FLOOR_Z))
RIM = BASE + (0, 0, m.RIM)
SEAT = RIM.copy()  # the basket in: its flange's bottom on the rim
TOPON = RIM + (0, 0, m.K_FLANGE_T)  # the upper chamber on the basket's flange
KSTART = np.array(m.BASKET_POS)
TOP0 = np.array((*m.TOP_XY, m.TOP + m.COLLAR_H + 0.0005))
GRIP_OPEN = 0.58  # fingers closed on the top's body (as read in the sim)


def q_yaw(deg, tilt=0.0):
    """Yaw about z (degrees), then optionally tilted about x."""
    a, t = math.radians(deg) / 2, math.radians(tilt) / 2
    qz = np.array([0.0, 0.0, math.sin(a), math.cos(a)])
    qx = np.array([math.sin(t), 0.0, 0.0, math.cos(t)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


DOWN = np.array((0.70711, 0.70711, 0.0, 0.0))  # a gripper pointing straight down, fingers closing along y at yaw 0


def hand(pos, yaw=0.0):
    """A gripper's state: straight down, turned `yaw` degrees about the vertical."""
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=qmul(q_yaw(yaw), DOWN), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.02, high=p + 0.02)


def st(pos, yaw=0.0, tilt=0.0, half=0.02):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=q_yaw(yaw, tilt), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p - half,
                       high=p + half)


def frame(top=None, top_yaw=0.0, top_tilt=0.0, jug=None, left=None, left_yaw=0.0, left_open=1.0, right=None,
          right_yaw=0.0, right_open=1.0, basket=None, basket_tilt=0.0, jug_tilt=0.0):
    """One step's states. The basket is in the base unless `basket` says where it is (BASKET is the default)."""
    g = {"left_gripper": (left if left is not None else HOME["left_gripper"], left_yaw, left_open),
         "right_gripper": (right if right is not None else HOME["right_gripper"], right_yaw, right_open)}
    objs = {"stove": st((m.BX, m.BY, m.TOP)), "base": st(BASE),
            "top": st(TOP0 if top is None else top, top_yaw, top_tilt),
            "jug": st(m.JUG_POS if jug is None else jug, 0.0, jug_tilt),
            "basket": st(BASKET if basket is None else basket, 0.0, basket_tilt),
            "cup": st((*m.CUP_XY, m.TOP))}
    objs.update({k: hand(p, y) for k, (p, y, _) in g.items()})
    return objs, {k: o for k, (_, _, o) in g.items()}


BASKET = SEAT


def run(frames, water=0.0, spill=0.0):
    """frames: list of (steps, (objects, openings)). A fresh trial each time (the kit's reset counter moves on); the
    base is given `water` mL and the table `spill` mL after the first step."""
    kit._EPISODE["count"] += 1
    out = (0.0, False)
    first = True
    for n, (objs, opens) in [(2, frame(basket=KSTART))] + frames:
        kit._OPENING.update(opens)
        for _ in range(n):
            pk.update(objs)
            if first:
                pk._S.contents["base"] = {"water": water} if water else {}
                pk._S.contents["jug"] = {"water": max(m.JUG0 - water - spill, 0.0)}
                pk._S.spilled = {"water": spill} if spill else {}
                first = False
            ok = m.success(objs)
            out = (m.score(objs), ok)
    return out


GRIP_Z = RIM[2] + m.GRIP_DZ


def place(yaw=0.0, holder="left"):
    """Carry the top over and set it on the rim, held."""
    g = (RIM[0], RIM[1], GRIP_Z)
    return [(3, frame(top=TOPON + (0, 0, 0.03), top_yaw=yaw, **{holder: g + np.array((0, 0, 0.03)),
                                                              f"{holder}_open": GRIP_OPEN}))] + \
           [(3, frame(top=TOPON, top_yaw=yaw, **{holder: g, f"{holder}_open": GRIP_OPEN}))]


def screw(deg, start=0.0, holder="left", grip_turns=True, other=None, step=10.0, open_=None, tilt=0.0, hover=None):
    """Turn the seated top clockwise (seen from above) by `deg` in `step`-degree steps. With `grip_turns` the holder's
    wrist rolls with it (regrasping every 90 degrees, as the demo does); otherwise the holder's wrist stays put (or
    wiggles +-10 degrees with `hover="wiggle"`) while something else turns the top."""
    f, yaw, w0 = [], start, 0.0
    g = (RIM[0], RIM[1], GRIP_Z)
    op = GRIP_OPEN if open_ is None else open_
    done = 0.0
    while done < deg - 1e-9:
        s = min(step, deg - done)
        done += s
        yaw -= s
        if grip_turns:
            w = w0 - (done % 90.0 if done % 90.0 else 90.0)
        elif hover == "wiggle":
            w = 10.0 if int(done / step) % 2 else -10.0
        else:
            w = 0.0
        kw = {holder: g, f"{holder}_open": op, f"{holder}_yaw": w}
        if other:
            kw.update(other)
        f.append((1, frame(top=TOPON, top_yaw=yaw, top_tilt=tilt, **kw)))
        if grip_turns and done % 90.0 == 0:  # let go, roll back open, grip again
            f.append((1, frame(top=TOPON, top_yaw=yaw, top_tilt=tilt, **{holder: g, f"{holder}_open": 1.0,
                                                                         f"{holder}_yaw": w})))
            f.append((1, frame(top=TOPON, top_yaw=yaw, top_tilt=tilt, **{holder: g, f"{holder}_open": 1.0,
                                                                         f"{holder}_yaw": w0})))
            f.append((1, frame(top=TOPON, top_yaw=yaw, top_tilt=tilt, **{holder: g, f"{holder}_open": op,
                                                                         f"{holder}_yaw": w0})))
    return f, yaw


def let_go(yaw, home=True, tilt=0.0):
    return [(3, frame(top=TOPON, top_yaw=yaw, top_tilt=tilt, left=(RIM[0], RIM[1], GRIP_Z + 0.05)))] + \
           ([(5, frame(top=TOPON, top_yaw=yaw, top_tilt=tilt))] if home else [])


GOOD = 140.0


def good(deg=540.0, water=GOOD, spill=0.0, home=True):
    f, yaw = screw(deg)
    return run(place() + f + let_go(yaw, home), water=water, spill=spill)


def check(name, got, want):
    assert abs(got[0] - want) < 1e-9, f"{name}: score {got[0]}, wanted {want}"
    print(f"ok  {name}: {got[0]}")


def high_grip_screw(deg_per_stroke=90.0, strokes=7, opening=0.83):
    """Screwed on by the wider upper body (tool point 6.5 cm up, fingers at 0.83 for its 8 cm), regripping each stroke."""
    g, f, yaw = (RIM[0], RIM[1], RIM[2] + 0.065), [], 0.0
    for _ in range(strokes):
        for k in range(1, int(deg_per_stroke / 10) + 1):
            yaw -= 10
            f.append((1, frame(top=TOPON, top_yaw=yaw, left=g, left_open=opening, left_yaw=-10 * k)))
        f.append((1, frame(top=TOPON, top_yaw=yaw, left=g, left_open=1.0, left_yaw=-deg_per_stroke)))
        f.append((1, frame(top=TOPON, top_yaw=yaw, left=g, left_open=1.0, left_yaw=0)))
    return run(place() + f + let_go(yaw), water=GOOD)


def pour_frames(steps=60, tilt=100.0):
    """The carafe tipped `tilt` degrees about x, its lip 5 cm over the middle of the base's mouth, for `steps` steps."""
    probe = frame(jug=np.array(m.JUG_POS), jug_tilt=tilt)[0]
    _, _, _, _, lip = pk._pose(probe, pk._CFG["vessels"]["jug"])
    at = np.array(m.JUG_POS) + (RIM + (0, 0, 0.05)) - lip
    return [(steps, frame(jug=at, jug_tilt=tilt, basket=BASKET))]


def basket_tests():
    global BASKET
    f, yaw = screw(540.0)
    # the basket never put in (left in its cup) under a screwed top: no screwing credit, so filled only
    BASKET = KSTART
    check("basket left out, top screwed on", run(place() + f + let_go(yaw), water=GOOD), 0.25)
    # ... or propped crooked in the mouth (10 degrees, 5 mm up) under the top
    crooked = st(SEAT + (0.002, 0, 0.005), 0.0, 10.0)
    tilted = [(n, ({**o, "basket": crooked}, op)) for n, (o, op) in place() + f + let_go(yaw)]
    check("basket propped crooked in the mouth", run(tilted, water=GOOD), 0.25)
    # flat on the rim against the mouth's wall (the modeled cup can sit 2.9 mm off center): still in
    BASKET = SEAT + (0.0, 0.0029, 0.0)
    check("basket flat on the rim, 2.9 mm off center", run(place() + f + let_go(yaw), water=GOOD), 1.0)
    BASKET = SEAT
    # tipped 60 degrees for three steps on the way over (the grounds fall out), then put in: filled only
    tip = [(3, frame(top=TOP0, basket=SEAT + (0, -0.1, 0.06), basket_tilt=60.0))]
    check("basket tipped 60 degrees on the way, then put in", run(tip + place() + f + let_go(yaw), water=GOOD), 0.25)
    # a one-step physics pop past 45 degrees is not a spill
    pop = [(1, frame(top=TOP0, basket=SEAT + (0, -0.1, 0.06), basket_tilt=60.0)),
           (2, frame(top=TOP0, basket=SEAT + (0, -0.1, 0.06)))]
    check("one-step pop past 45 degrees", run(pop + place() + f + let_go(yaw), water=GOOD), 1.0)
    # the coffee's patch is not drawn offline, and the latch keeps where it fell
    assert m._H.coffee_lost is None
    run(tip, water=GOOD)
    assert m._H.coffee_lost is not None and abs(m._H.coffee_lost[1] - (SEAT[1] - 0.1)) < 1e-9
    # basket in, top on but not turned: 0.5
    check("basket in, top set on, not turned", run(place() + let_go(0.0), water=GOOD), 0.5)
    # top screwed with no basket, then the basket dropped on top of it (lying on the lid)
    BASKET = KSTART
    f2 = place() + f + let_go(yaw)
    f2 += [(5, frame(top=TOPON, top_yaw=yaw, basket=TOPON + (0, 0, m.T_H + 0.012), basket_tilt=0.0))]
    check("top screwed, basket dropped on its lid", run(f2, water=GOOD), 0.25)
    # top screwed with no basket under it, then the basket appears in the base (put in after the top: no credit)
    f3 = place() + f + let_go(yaw) + [(5, frame(top=TOPON, top_yaw=yaw, basket=SEAT))]
    check("basket in only after the top was screwed", run(f3, water=GOOD), 0.5)
    BASKET = SEAT
    # weak fill with everything else done, and a weak fill with only the basket in
    check("short pour, basket in, top snug", good(water=m.LOW_ML - 5), 0.25)
    check("short pour, basket in, top left off", run([(5, frame())], water=m.LOW_ML - 5), 0.0)
    # the order of water and basket: poured with the basket already in, the stream lands on the grounds (spilled)
    BASKET = KSTART
    run(pour_frames())
    dry_first = pk.volume(None, "base")
    assert dry_first > 100.0, f"pouring into the open base should fill it, got {dry_first:.1f} mL"
    BASKET = SEAT
    run(pour_frames())
    assert pk.volume(None, "base") == 0.0 and pk.spilled() > 100.0, "a stream onto the seated basket must spill"
    BASKET = SEAT
    seq_bad = pour_frames(20, 80.0) + place() + f + let_go(yaw)
    check("basket in first, then poured (onto the grounds)", run(seq_bad), 0.0)
    BASKET = KSTART
    seq = pour_frames(20, 80.0) + [(2, frame(basket=KSTART))]  # 129.9 mL, then the carafe stood back up
    BASKET = SEAT
    check("poured 130 mL into the open base, then the basket and the top", run(seq + place() + f + let_go(yaw)), 1.0)
    assert m._lets_water_in(frame(basket=KSTART, top=TOP0)[0])
    assert not m._lets_water_in(frame(basket=SEAT, top=TOP0)[0])


class _Body:
    """A rigid body as the physics-step hook sees it: pose (x, y, z, qx, qy, qz, qw) and velocity, and the writes."""

    def __init__(self, pos, q, vel=(0.0,) * 6):
        self.data = self
        self.set(pos, q, vel)

    def set(self, pos, q, vel=(0.0,) * 6):
        import torch
        self.root_link_pose_w = torch.tensor([[*pos, *q]], dtype=torch.float64)
        self.root_link_quat_w = self.root_link_pose_w[:, 3:7]
        self.root_com_vel_w = torch.tensor([list(vel)], dtype=torch.float64)

    def write_root_pose_to_sim_index(self, root_pose):
        self.set(root_pose[0, :3].tolist(), root_pose[0, 3:7].tolist(), self.root_com_vel_w[0].tolist())

    def write_root_velocity_to_sim_index(self, root_velocity):
        self.root_com_vel_w = root_velocity.clone()


def seat_hold_tests():
    """The thread's seat and stop, applied by the physics-step hook: screwed past the seat and down on the base, a
    gripper still turning, pushing up or tipping it (a policy that keeps turning at the stop) leaves it on the seat."""
    import types
    f, yaw = screw(500.0)
    run(place() + f)  # 500 degrees in: past the seat, 40 degrees short of the stop
    assert m._STOP["seat"] and m._STOP["yaw"] is not None, m._STOP
    seat = (m.BX, m.BY, TOPON[2])
    top, base = _Body(seat, q_yaw(yaw)), _Body(BASE, q_yaw(0.0))
    env = types.SimpleNamespace(scene={"top": top, "base": base})
    m._hold_at_stop(env)  # first sight of it seated: the pose it is held at
    # a stroke lifts it 6 mm, slides it 3 mm, tips it 7 degrees and turns it 20 degrees on, moving up and tipping
    top.set((seat[0] + 0.003, seat[1], seat[2] + 0.006), q_yaw(yaw - 20.0, tilt=7.0), (0.01, 0.0, 0.3, 2.0, 1.0, -1.0))
    m._hold_at_stop(env)
    p, v = top.root_link_pose_w[0].numpy(), top.root_com_vel_w[0].numpy()
    assert np.allclose(p[:3], seat, atol=1e-9), p
    assert m._tilt_deg(p[3:7]) < 1e-6, m._tilt_deg(p[3:7])
    assert abs(m.tk._wrap(m._yaw(p[3:7]) - math.radians(yaw - 20.0))) < 1e-6, math.degrees(m._yaw(p[3:7]))
    assert np.allclose(v[:5], 0.0) and v[5] == -1.0, v  # turning on is still free short of the stop
    print("ok  on the seat a stroke still turns it but cannot lift, slide or tip it")
    # turned 30 degrees past the stop (70 from where it was): back at the stop, its clockwise spin taken away
    top.set(seat, q_yaw(yaw - 70.0), (0.0, 0.0, 0.0, 0.0, 0.0, -1.0))
    m._hold_at_stop(env)
    p, v = top.root_link_pose_w[0].numpy(), top.root_com_vel_w[0].numpy()
    assert abs(m.tk._wrap(m._yaw(p[3:7]) - math.radians(yaw - 40.0))) < 1e-6, math.degrees(m._yaw(p[3:7]))
    assert v[5] == 0.0, v
    print("ok  turned past the stop: back at the stop, no clockwise spin")
    # unscrewed back off the seat (to 400 degrees): the hold lets go
    back = [(1, frame(top=TOPON, top_yaw=yaw + 10.0 * k, left=(RIM[0], RIM[1], GRIP_Z), left_open=GRIP_OPEN,
                      left_yaw=10.0 * k)) for k in range(1, 11)]
    run(place() + f + back)
    assert not m._STOP["seat"] and m._STOP["yaw"] is None, m._STOP
    top.set((seat[0], seat[1], seat[2] + 0.006), q_yaw(yaw + 100.0), (0.0, 0.0, 0.3, 0.0, 0.0, 0.0))
    m._hold_at_stop(env)
    assert abs(top.root_link_pose_w[0, 2].item() - seat[2] - 0.006) < 1e-9 and m._STOP["pose"] is None
    print("ok  unscrewed off the seat: free to lift")
    run([])  # a new trial: nothing left over
    assert m._STOP["yaw"] is None and not m._STOP["seat"] and m._STOP["pose"] is None, m._STOP
    print("ok  new trial: no seat held")
    # give at the stop: screwed to 460 by hand, then four times the opening fingers let it spring back 6 degrees and
    # the open hand coming round it nudges it back (no credit for either): still on its seat, still snug
    f, yaw = screw(460.0)
    open_hand = {"left": (RIM[0], RIM[1], GRIP_Z), "left_open": 1.0}
    give = []
    for _ in range(4):
        give += [(1, frame(top=TOPON, top_yaw=yaw + 6.0, **open_hand)), (1, frame(top=TOPON, top_yaw=yaw, **open_hand))]
    check("snug, then give at the stop four times", run(place() + f + give + let_go(yaw), water=GOOD), 1.0)
    # ... but knocked back 20 degrees off the seat it is loose
    check("snug, then knocked back off the seat", run(place() + f + give[:1] + [(1, frame(
        top=TOPON, top_yaw=yaw + 20.0, **open_hand))] + let_go(yaw + 20.0), water=GOOD), 0.5)
    # and a top screwed only 440 by hand and then swiped on past the seat by an open hand never reached it by hand
    f, yaw = screw(440.0)
    swipe = [(1, frame(top=TOPON, top_yaw=yaw - 10.0 * k, **open_hand)) for k in range(1, 4)]
    check("440 by hand, swiped on to 470", run(place() + f + swipe + let_go(yaw - 30.0), water=GOOD), 0.5)


def main():
    check("full job", good(), 1.0)
    # review, Nov 10
    check("screwed on gripping the upper body (630 degrees)", high_grip_screw(), 1.0)
    f2, y2 = screw(540.0)
    check("job done, left hand left clamped on the upper body", run(place() + f2 + [(10, frame(
        top=TOPON, top_yaw=y2, left=(RIM[0], RIM[1], RIM[2] + 0.065), left_open=0.83))], water=GOOD), 0.75)
    check("job done, left hand left closed on the lid knob", run(place() + f2 + [(10, frame(
        top=TOPON, top_yaw=y2, left=(RIM[0], RIM[1], RIM[2] + m.T_H + 0.005), left_open=0.17))], water=GOOD), 0.75)
    assert good()[1], "success() should be true"
    check("do nothing", run([]), 0.0)
    check("top screwed on an empty base", good(water=0.0), 0.0)
    check("water above the valve", good(water=m.VALVE_ML + 2), 0.0)
    check("carafe emptied in (175 mL)", good(water=m.JUG0), 0.0)
    check("weak: 1.5 cm under the valve", good(water=m.ml_below(m.TOP_LINE_Z - 0.015)), 0.25)
    check("just under the valve", good(water=m.VALVE_ML - 1), 1.0)
    check("just above the low line", good(water=m.LOW_ML + 1), 1.0)
    check("filled, basket and top left out", run([], water=GOOD), 0.25)
    check("filled, basket in, top left off", run([(5, frame())], water=GOOD), 0.5)
    check("a dribble poured in, basket in, top left off", run([(5, frame())], water=60.0), 0.0)
    check("weak fill, top left off", run([], water=m.LOW_ML - 1), 0.0)
    check("filled, top set on but not turned", run(place() + let_go(0.0), water=GOOD), 0.5)
    check("loose: a turn", good(deg=360.0), 0.5)
    check("snug band low end", good(deg=460.0), 1.0)
    check("just short of the seat (440 degrees)", good(deg=440.0), 0.5)
    check("at the stop (540 degrees)", good(deg=540.0), 1.0)
    # the thread's stop: 500 degrees in, it sits 40 degrees short of the stop, which the scorer hands to the physics
    # hook as a relative yaw; far from it (a turn in) there is no stop to hold
    good(deg=500.0)
    want = math.radians(-500.0) - math.radians(40.0)
    assert m._STOP["yaw"] is not None and abs(m.tk._wrap(m._STOP["yaw"] - want)) < 1e-6, m._STOP
    print("ok  stop set 40 degrees on from 500")
    good(deg=360.0)
    assert m._STOP["yaw"] is None, m._STOP
    print("ok  no stop a turn in")
    seat_hold_tests()
    check("spilled 10 mL", good(spill=10.0), 0.75)
    check("arms not home (not scored)", good(home=False), 1.0)
    # an arm parked right over the screwed-on top with its hand open: let go, so full credit
    check("open hand parked just over the top", run(place() + screw(540.0)[0] + [(8, frame(
        top=TOPON, top_yaw=screw(540.0)[1], left=(RIM[0], RIM[1], GRIP_Z + 0.02), left_open=1.0))], water=GOOD), 1.0)
    assert not hasattr(m, "REQUIRE_ARMS_HOME") and not hasattr(m, "_arms_home"), "arm-home rule should be gone"
    # an empty closed fist resting on the top while it is swiped round earns nothing
    f, yaw = screw(540.0, grip_turns=True, open_=0.05)
    check("empty fist rolling on the top", run(place() + f + let_go(yaw), water=GOOD), 0.5)
    # screwed 540 by hand, then dragged back 200 degrees by an open hand: looser by that much
    f, yaw = screw(540.0)
    drag = [(1, frame(top=TOPON, top_yaw=yaw + 10.0 * k, right=(RIM[0], RIM[1] - 0.04, GRIP_Z), right_open=1.0))
            for k in range(1, 21)]
    check("screwed, then dragged back 200 unheld", run(place() + f + drag + let_go(yaw + 200.0), water=GOOD), 0.5)
    # still holding the pot at the end
    f, yaw = screw(540.0)
    held = [(5, frame(top=TOPON, top_yaw=yaw, left=(RIM[0], RIM[1], GRIP_Z), left_open=GRIP_OPEN, left_yaw=-45.0))]
    check("pot left held", run(place() + f + held, water=GOOD), 0.75)
    # carafe left on the stove plate, or still held
    f, yaw = screw(540.0)
    end = let_go(yaw)[:1] + [(5, frame(top=TOPON, top_yaw=yaw, jug=(m.BX + 0.05, m.BY - 0.05, m.TOP + m.JUG_H / 2)))]
    check("carafe left on the stove", run(place() + f + end, water=GOOD), 0.75)
    # twist holes: the other arm swipes the top round while the left hand hovers open around it
    swipe = {"right": (RIM[0], RIM[1] - 0.04, GRIP_Z), "right_open": 0.0}
    f, yaw = screw(540.0, grip_turns=False, open_=1.0, other=swipe)
    check("swiped while a hand hovers open", run(place() + f + let_go(yaw), water=GOOD), 0.5)
    # ... or while the left hand is closed on it but only wiggles its wrist +-10 degrees
    f, yaw = screw(540.0, grip_turns=False, other=swipe, hover="wiggle")
    check("swiped while a closed hand wiggles", run(place() + f + let_go(yaw), water=GOOD), 0.5)
    f, yaw = screw(1500.0, grip_turns=False, other=swipe, hover="wiggle")
    check("swiped 4 turns while a closed hand wiggles", run(place() + f + let_go(yaw), water=GOOD), 0.5)
    # ... or while the closed hand stays still
    f, yaw = screw(540.0, grip_turns=False, other=swipe)
    check("swiped under a still closed hand", run(place() + f + let_go(yaw), water=GOOD), 0.5)
    # screwed, then lifted off and set back without turning: the count starts again
    f, yaw = screw(540.0)
    off = [(3, frame(top=TOPON + (0, 0, 0.02), top_yaw=yaw, left=(RIM[0], RIM[1], GRIP_Z + 0.02),
                     left_open=GRIP_OPEN))]
    check("screwed, lifted off, set back", run(place() + f + off + place(yaw) + let_go(yaw), water=GOOD), 0.5)
    # cross-threaded: the top crooked 6 degrees on the rim while it is turned
    f, yaw = screw(540.0, tilt=6.0)
    check("cross-threaded (6 degrees crooked)", run(place() + f + let_go(yaw, tilt=6.0), water=GOOD), 0.5)
    # unscrewing takes the credit back: on 540, back 360
    f, yaw = screw(540.0)
    back = []
    for k in range(1, 37):
        w = (10.0 * k) % 90.0
        back.append((1, frame(top=TOPON, top_yaw=yaw + 10.0 * k, left=(RIM[0], RIM[1], GRIP_Z), left_open=GRIP_OPEN,
                              left_yaw=w if w else 90.0)))
        if w == 0:
            back += [(1, frame(top=TOPON, top_yaw=yaw + 10.0 * k, left=(RIM[0], RIM[1], GRIP_Z), left_yaw=90.0)),
                     (1, frame(top=TOPON, top_yaw=yaw + 10.0 * k, left=(RIM[0], RIM[1], GRIP_Z), left_yaw=0.0))]
    check("screwed 540 then backed off 360", run(place() + f + back + let_go(yaw + 360.0), water=GOOD), 0.5)
    # held to the end while the score is taken: no credit for the job until it is let go
    f, yaw = screw(540.0)
    held = [(20, frame(top=TOPON, top_yaw=yaw, left=(RIM[0], RIM[1], GRIP_Z), left_open=GRIP_OPEN, left_yaw=0.0))]
    check("job done but the top still gripped at the end", run(place() + f + held, water=GOOD), 0.75)
    # the carafe still in the right hand at the end
    f, yaw = screw(540.0)
    end = let_go(yaw)[:1] + [(5, frame(top=TOPON, top_yaw=yaw, jug=(0.23, -0.20, m.TOP + 0.10),
                                         right=(0.23, -0.20, m.TOP + 0.10), right_open=0.3))]
    check("carafe still held at the end", run(place() + f + end, water=GOOD), 0.75)
    # contact, not distance (audit, Nov 12): a closed hand 1 cm clear of a part is not on it; a fingertip resting on it is
    f, yaw = screw(540.0)
    # closed, at yaw 0 the fingers (2.4 cm each side of the tool point) close along y, beside the 8 cm upper body
    side = (RIM[0], RIM[1] + m.T_TOP_R + 0.024 + 0.010, GRIP_Z)
    check("closed hand parked 1 cm beside the top at the end", run(place() + f + let_go(yaw)[:1] + [(5, frame(
        top=TOPON, top_yaw=yaw, left=side, left_open=0.0))], water=GOOD), 1.0)
    touch = (RIM[0], RIM[1] + m.T_TOP_R + 0.024, GRIP_Z)
    check("closed fingertips resting on the top's side at the end", run(place() + f + let_go(yaw)[:1] + [(5, frame(
        top=TOPON, top_yaw=yaw, left=touch, left_open=0.0))], water=GOOD), 0.75)
    j = np.array(m.JUG_POS)
    check("closed hand 1 cm beside the carafe at the end", run(place() + f + let_go(yaw)[:1] + [(5, frame(
        top=TOPON, top_yaw=yaw, right=j + (0, m.JUG_R + 0.024 + 0.010, 0.02), right_open=0.0))],
        water=GOOD), 1.0)
    check("open hand round the carafe, a pad on its side", run(place() + f + let_go(yaw)[:1] + [(5, frame(
        top=TOPON, top_yaw=yaw, right=j + (0, m.JUG_R - abs(m.FINGER_OPEN), 0.02),
        right_open=1.0))], water=GOOD), 0.75)
    # the top knocked off the base after being screwed (lying beside it): the count is gone
    f, yaw = screw(540.0)
    knocked = [(5, frame(top=TOPON + (0.09, 0.0, -0.05), top_yaw=yaw, top_tilt=80.0))]
    check("screwed, then knocked off", run(place() + f + let_go(yaw)[:1] + knocked, water=GOOD), 0.5)
    # the top only hovering just over the collar at the end (not down on the rim)
    f, yaw = screw(540.0)
    hover = [(5, frame(top=TOPON + (0, 0, 0.008), top_yaw=yaw))]
    check("screwed, then left raised 8 mm", run(place() + f + let_go(yaw)[:1] + hover, water=GOOD), 0.5)
    # --- the funnel basket (review, Nov 12)
    basket_tests()
    # reset: a full job, then a new trial where nothing happens
    assert good()[0] == 1.0
    check("reset after a full job, then nothing", run([]), 0.0)
    print("all pass")


if __name__ == "__main__":
    main()
