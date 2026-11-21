"""Near-successes and unintended strategies, scored offline by the offline checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so before the
cases this file writes into the task's per-trial record what the step callback records in a trial where the film's
near end was cut open and both hands ended 50 mm from everything. The cases then vary the end state. _run() first
drives the step-side function observe() directly, state by state, to check the cut rule (a cut in the unwelded film
opens the end; a cut only through the seal band, a closing with the blades lying along the film, a closing begun
more than 2 s after the blades were open, or a closing on the fish does not), and the whole-trial rules (film in the
dish, fish on the counter or in the scrap bowl, an open blade tip in the fish, the dish tipped), which stay at 0
after the scene is put right. It raises if anything scores wrongly.
"""

import math

import numpy as np

import lots
import parts
import task as T

CLEAR = {"salmon": 0.05, "pouch": 0.05, "shears": 0.05, "dish": 0.05}


def q_of(m):
    return tuple(parts._matrix_to_quat(np.asarray(m, float)))


def qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def q_tilt(deg, about="x"):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a)) if about == "x" else (0.0, math.sin(a), 0.0, math.cos(a))


def scene(lot="A", **over):
    lots.use(lot)
    L = lots.current()
    out = {
        "board": T.BOARD.state(pos=L.pos("board")),
        "pouch": T.POUCH.state(pos=L.pos("pouch")),
        "salmon": T.FISH_PART.state(pos=L.pos("salmon")),
        "shears": T.SHEARS.state(pos=L.pos("shears")),
        "dish": T.DISH.state(pos=L.pos("dish")),
        "bowl": T.BOWL.state(pos=L.pos("bowl")),
    }
    out.update(over)
    return out


def fish_in_dish(lot="A", dx=0.0, dz=0.0, quat=(0.0, 0.0, 0.0, 1.0)):
    lots.use(lot)
    x, y, _ = lots.current().pos("dish")
    return T.FISH_PART.state(pos=(x + dx, y, T.TOP + T.DISH_T + T.FISH[2] / 2 + dz), quat_xyzw=quat)


def pouch_in_bowl(lot="A", dz=0.0, dx=0.0):
    lots.use(lot)
    x, y, _ = lots.current().pos("bowl")
    return T.POUCH.state(joints={"end_nx": 0.4}, pos=(x + dx, y, T.TOP + 0.006 + dz))


def shears_down(lot="A", where=None, z=None):
    lots.use(lot)
    L = lots.current()
    side = L.side
    if where is None:
        where = L.pos("shears")[:2]  # put back where they lay
    zz = T.BOARD_TOP + T.HANDLE_T / 2 if z is None else z
    return T.SHEARS.state(pos=(where[0], where[1], zz), quat_xyzw=qz(-T.HOLD_JAW / 2), joints={"jaw": T.HOLD_JAW})


def finished(lot="A", **over):
    base = dict(salmon=fish_in_dish(lot), pouch=pouch_in_bowl(lot), shears=shears_down(lot))
    base.update(over)
    return scene(lot, **base)


def preset(opened=True, gaps=CLEAR):
    T.reset_record()
    T._rec["opened"]["nx"] = bool(opened)
    T._rec["gaps"] = dict(gaps)


# -- the cut, driven through observe() ------------------------------------------------------------------------------


def at_pour(lot="A", pitch=T.LEAN):
    """The portion held as the demo holds it: turned across the arms, leaned ``pitch`` degrees over the dish, the
    fish against the near end."""
    lots.use(lot)
    L = lots.current()
    x, y, _ = L.pos("dish")
    g, _, _, psi = T.pour_geometry((x, y), L.side)
    r = T._lean_rot(psi, pitch) @ T._down(0.0).T
    o = g - r @ np.array([T.END_X - T.GRIP_IN, 0.0, T.GRIP_UP])
    pouch = T.POUCH.state(pos=o, quat_xyzw=q_of(r))
    fish = T.FISH_PART.state(pos=o + r @ np.array([-(T.L_IN / 2 - T.FISH[0] / 2), 0.0, T.WALL + T.FISH[2] / 2]),
                             quat_xyzw=q_of(r))
    return pouch, fish


def shears_at_cut(lot="A", jaw=T.SHUT, dz=0.0, dx=0.0, dy=0.0):
    """The shears where the demo closes them (TCP shifted by dx, dy, dz), at jaw angle ``jaw``, held symmetrically;
    their blades point away from the arms (+x)."""
    lots.use(lot)
    L = lots.current()
    side = L.side
    plan = T.demo()
    arm = "left" if side < 0 else "right"
    tcp = np.array([w for w in plan[arm] if w.label == "close the shears"][0].pos) + [dx, dy, dz]
    # the demo aims the hand off its target by SHEAR_SAG to cancel the sag measured on the GPU; offline the shears
    # sit exactly where the hand aims, so take that correction back out
    tcp = tcp - np.array([T.SHEAR_SAG[0], -side * T.SHEAR_SAG[1], T.SHEAR_SAG[2]])
    axis = np.array([1.0, 0.0, 0.0])
    yaw = -jaw / 2
    piv = tcp + axis * T.grip_reach(jaw) - [0.0, 0.0, T.SH_TCP_UP]
    return T.SHEARS.state(joints={"jaw": jaw}, pos=piv, quat_xyzw=qz(yaw))


def close_on(states_open, states_shut, t0=10.0, dt=0.5):
    """Feed observe() an open then a shut step; return the cut it recorded (or None)."""
    T.observe(states_open, CLEAR, t0)
    T.observe(states_shut, CLEAR, t0 + dt)
    return T._rec["cuts"][-1] if T._rec["cuts"] else None


def expect(label, got, want):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"near_misses _run: {label}: score {got}, expected {want}")


def _run():
    for lot in ("A", "B"):
        pouch, fish = at_pour(lot)
        # 1. a cut in the unwelded film opens the near end; the finished scene then scores 1.0
        T.reset_record()
        cut = close_on(scene(lot, pouch=pouch, salmon=fish, shears=shears_at_cut(lot, 0.0)),
                       scene(lot, pouch=pouch, salmon=fish, shears=shears_at_cut(lot, T.SHUT)))
        assert cut and cut["end"] == "nx" and not cut["fish"], (lot, cut)
        T._rec["gaps"] = dict(CLEAR)
        expect(f"{lot} cut then finished", T.score(finished(lot)), 1.0)
        # 2. never cut: the same finished scene scores 0 (the fish cannot leave a sealed film)
        T.reset_record()
        T._rec["gaps"] = dict(CLEAR)
        expect(f"{lot} not cut", T.score(finished(lot)), 0.0)
        # 3. a closing only through the seal band (10 mm lower) opens nothing
        T.reset_record()
        cut = close_on(scene(lot, pouch=pouch, salmon=fish, shears=shears_at_cut(lot, 0.0, dz=-0.010)),
                       scene(lot, pouch=pouch, salmon=fish, shears=shears_at_cut(lot, T.SHUT, dz=-0.010)))
        assert cut and cut["end"] is None and cut["band"], (lot, cut)
        # 4. a closing 16 mm higher takes in the fish: 0 for the trial, even with the finished scene after
        T.reset_record()
        cut = close_on(scene(lot, pouch=pouch, salmon=fish, shears=shears_at_cut(lot, 0.0, dz=0.016)),
                       scene(lot, pouch=pouch, salmon=fish, shears=shears_at_cut(lot, T.SHUT, dz=0.016)))
        assert cut and cut["fish"], (lot, cut)
        T._rec["opened"]["nx"] = True
        T._rec["gaps"] = dict(CLEAR)
        expect(f"{lot} fish cut", T.score(finished(lot)), 0.0)
        # 5. a closing that ends more than 2 s after the blades were last open cuts nothing
        T.reset_record()
        T.observe(scene(lot, pouch=pouch, salmon=fish, shears=shears_at_cut(lot, 0.0)), CLEAR, 10.0)
        mid = shears_at_cut(lot, T.SHUT - math.radians(5.0))
        T.observe(scene(lot, pouch=pouch, salmon=fish, shears=mid), CLEAR, 11.0)
        T.observe(scene(lot, pouch=pouch, salmon=fish, shears=shears_at_cut(lot, T.SHUT)), CLEAR, 12.5)
        assert not T._rec["cuts"] and not any(T._rec["opened"].values()), (lot, T._rec["cuts"])
        # 6. blades lying along the film (the film flat on the board, shears flat beside its end): nothing opens
        T.reset_record()
        L = lots.current()
        px, py, pz = L.pos("pouch")
        flat_tcp = np.array([px - T.END_X - T.CUT_AT, py - L.side * 0.15, pz + 0.0008])  # blade plane at the film
        yaw = math.atan2(L.side, 0.0)
        sh_open = T.SHEARS.state(joints={"jaw": 0.0}, pos=flat_tcp + np.array([0, L.side, 0]) * T.grip_reach(0.0),
                                 quat_xyzw=qz(yaw))
        sh_shut = T.SHEARS.state(joints={"jaw": T.SHUT}, pos=flat_tcp + np.array([0, L.side, 0]) * T.grip_reach(T.SHUT),
                                 quat_xyzw=qz(yaw - T.HALF))
        cut = close_on(scene(lot, shears=sh_open), scene(lot, shears=sh_shut))
        assert cut and cut["end"] is None and cut["cross"] < 20.0, (lot, cut)
        # 6b. the same with the shears rolled 30 degrees so the blades cross the flat film: the lower blade would have
        # to pass into the board, so nothing is cut
        T.reset_record()
        roll = T._rot(qz(yaw)) @ parts._rot((1, 0, 0), math.radians(30.0))  # rolled about the hand's line
        roll_s = T._rot(qz(yaw - T.HALF)) @ parts._rot(T._u1, math.radians(30.0))
        sh_open = T.SHEARS.state(joints={"jaw": 0.0}, pos=flat_tcp + roll @ np.array([T.grip_reach(0.0), 0, 0]),
                                 quat_xyzw=q_of(roll))
        sh_shut = T.SHEARS.state(joints={"jaw": T.SHUT}, pos=flat_tcp + roll_s @ (T._u1 * T.grip_reach(T.SHUT)),
                                 quat_xyzw=q_of(roll_s))
        cut = close_on(scene(lot, shears=sh_open), scene(lot, shears=sh_shut))
        assert cut and cut["end"] is None and cut["solid"], (lot, cut)
        # 6c. snips add up: one 41 mm snip leaves the end shut, a second, longer one opens it
        T.reset_record()
        back = shears_at_cut(lot, 0.0, dx=-0.045), shears_at_cut(lot, T.SHUT, dx=-0.045)
        close_on(scene(lot, pouch=pouch, salmon=fish, shears=back[0]), scene(lot, pouch=pouch, salmon=fish, shears=back[1]))
        assert not any(T._rec["opened"].values()), (lot, T._rec["cuts"])
        close_on(scene(lot, pouch=pouch, salmon=fish, shears=shears_at_cut(lot, 0.0)),
                 scene(lot, pouch=pouch, salmon=fish, shears=shears_at_cut(lot, T.SHUT)), t0=14.0)
        assert T._rec["opened"]["nx"] and not T._rec["zero"], (lot, T._rec["cuts"])
        # 6d. a second snip from further in cuts the strip right off over the dish: it falls in, 0 for the trial
        close_on(scene(lot, pouch=pouch, salmon=fish, shears=shears_at_cut(lot, 0.0, dx=0.03)),
                 scene(lot, pouch=pouch, salmon=fish, shears=shears_at_cut(lot, T.SHUT, dx=0.03)), t0=18.0)
        assert T._rec["zero"] and "strip" in T._rec["why"], (lot, T._rec["why"], T._rec["cuts"])
        # 6e. closing the jaw with the fish between the blades (20 mm too high): 0 for the trial at once
        T.reset_record()
        T.observe(scene(lot, pouch=pouch, salmon=fish, shears=shears_at_cut(lot, 0.0, dz=0.020)), CLEAR, 10.0)
        T.observe(scene(lot, pouch=pouch, salmon=fish, shears=shears_at_cut(lot, 0.15, dz=0.020)), CLEAR, 10.05)
        assert T._rec["zero"] and "closed on the fish" in T._rec["why"], (lot, T._rec["why"])
        # 6f. closing the shears on a fingertip between the blades: 0 for the trial
        T.reset_record()
        st0, st1 = shears_at_cut(lot, 0.0), shears_at_cut(lot, 0.15)
        o, r = T._frame(st1)
        finger = o + r @ np.array([0.06, -0.005, 0.0]) + np.random.default_rng(0).normal(0, 0.001, (30, 3))
        T.observe(scene(lot, shears=st0), CLEAR, 10.0, hands=finger)
        T.observe(scene(lot, shears=st1), CLEAR, 10.05, hands=finger)
        assert T._rec["zero"] and "gripper" in T._rec["why"], (lot, T._rec["why"])
        # 7. an open blade tip pushed into the fish: 0 for the trial
        T.reset_record()
        st = shears_at_cut(lot, 0.0)
        tip = T.blade_tips(scene(lot, shears=st))[0]
        f2 = T.FISH_PART.state(pos=tuple(tip + np.array([0.0, 0.0, 0.0])))
        T.observe(scene(lot, salmon=f2, shears=st), CLEAR, 5.0)
        T._rec["opened"]["nx"] = True
        expect(f"{lot} blade tip in the fish", T.score(finished(lot)), 0.0)
        # 8. the sealed portion set in the dish at any moment: 0, even after it is taken out and finished
        T.reset_record()
        lots.use(lot)
        dxy = lots.current().pos("dish")
        T.observe(scene(lot, pouch=T.POUCH.state(pos=(dxy[0], dxy[1], T.TOP + T.DISH_T)),
                        salmon=T.FISH_PART.state(pos=(dxy[0], dxy[1], T.TOP + T.DISH_T + T.WALL + T.FISH[2] / 2))),
                  CLEAR, 3.0)
        T._rec["opened"]["nx"] = True
        expect(f"{lot} film was in the dish", T.score(finished(lot)), 0.0)
        # 9. the fish on the counter at any moment: 0, even after it is put in the dish
        T.reset_record()
        lots.use(lot)
        bx, by, _ = lots.current().pos("board")
        T.observe(scene(lot, salmon=T.FISH_PART.state(pos=(0.12, by, T.TOP + T.FISH[2] / 2))), CLEAR, 3.0)
        T._rec["opened"]["nx"] = True
        expect(f"{lot} fish touched the counter", T.score(finished(lot)), 0.0)
        # 10. the dish tipped past 30 degrees at any moment: 0
        T.reset_record()
        lots.use(lot)
        T.observe(scene(lot, dish=T.DISH.state(pos=lots.current().pos("dish"), quat_xyzw=q_tilt(35))), CLEAR, 3.0)
        T._rec["opened"]["nx"] = True
        expect(f"{lot} dish tipped", T.score(finished(lot)), 0.0)
        # 11. hands not known (the simulator could not read them): 0
        preset()
        T._rec["known"] = False
        expect(f"{lot} hands unknown", T.score(finished(lot)), 0.0)
        # 12. a hand 20 mm from the shears at the end: 0.75; touching the fish: 0.25
        preset(gaps=dict(CLEAR, shears=0.020))
        expect(f"{lot} hand 20 mm from the shears", T.score(finished(lot)), 0.75)
        preset(gaps=dict(CLEAR, salmon=0.002))
        expect(f"{lot} hand still on the fish", T.score(finished(lot)), 0.25)
    preset()
    lots.use("A")


_run()
preset()
lots.use("A")
_A = lots.current()
_dish = _A.pos("dish")
_bowl = _A.pos("bowl")

CASES = [
    ("finished: fish in the dish, film in the bowl, shears on the board", finished("A"), 1.0, True),
    ("finished in lot B's layout", finished("B"), 1.0, True),
    ("empty film left on the board instead of the bowl", finished("A", pouch=T.POUCH.state(joints={"end_nx": 0.4}, pos=_A.pos("pouch"))), 1.0, True),
    ("shears put down on the counter beside the board", finished("A", shears=shears_down("A", where=(0.12, 0.30), z=T.TOP + T.HANDLE_T / 2)), 1.0, True),
    ("shears laid in the scrap bowl", finished("A", shears=shears_down("A", where=(_bowl[0], _bowl[1] - 0.04), z=T.TOP + 0.006 + T.HANDLE_T / 2)), 1.0, True),
    ("shears still held up where they cut, over the dish", finished("A", shears=shears_at_cut("A", 0.0)), 0.5, False),
    ("shears laid in the dish beside the fish", finished("A", shears=shears_down("A", where=(_dish[0] + 0.0, _dish[1]), z=T.TOP + T.DISH_T + T.HANDLE_T / 2)), 0.5, False),
    ("empty film still held over the dish", finished("A", pouch=T.POUCH.state(joints={"end_nx": 0.4}, pos=(_dish[0], _dish[1], T.TOP + 0.10))), 0.5, False),
    ("empty film in the dish with the fish", finished("A", pouch=T.POUCH.state(joints={"end_nx": 0.4}, pos=(_dish[0], _dish[1] + 0.03, T.TOP + T.DISH_T + T.FISH[2]))), 0.0, False),
    ("fish out of the film but left on the board", finished("A", salmon=T.FISH_PART.state(pos=(0.30, 0.10, T.BOARD_TOP + T.FISH[2] / 2))), 0.25, False),
    ("fish propped on the dish's rim", finished("A", salmon=fish_in_dish("A", dx=-0.07, dz=0.03, quat=q_tilt(25, "y"))), 0.25, False),
    ("fish half over the dish's wall", finished("A", salmon=fish_in_dish("A", dx=0.08)), 0.25, False),
    ("fish on the counter", finished("A", salmon=T.FISH_PART.state(pos=(0.12, -0.12, T.TOP + T.FISH[2] / 2))), 0.0, False),
    ("fish in the scrap bowl", finished("A", salmon=T.FISH_PART.state(pos=(_bowl[0], _bowl[1], T.TOP + 0.006 + T.FISH[2] / 2))), 0.0, False),
    ("fish leaning on the scrap bowl's rim", finished("A", salmon=T.FISH_PART.state(pos=(_bowl[0] + 0.14, _bowl[1], T.TOP + 0.13), quat_xyzw=q_tilt(60, "y"))), 0.0, False),
    ("dish knocked onto its side", finished("A", dish=T.DISH.state(pos=_dish, quat_xyzw=q_tilt(80))), 0.0, False),
    ("scrap bowl fallen off the counter", finished("A", bowl=T.BOWL.state(pos=(_bowl[0], _bowl[1], T.TOP - 0.3))), 0.0, False),
]
