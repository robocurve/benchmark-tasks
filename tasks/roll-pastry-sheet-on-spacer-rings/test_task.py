"""Offline checks of the pastry-rolling model and scorer: step-by-step pin and gripper histories at 20 Hz, no simulator.

    python test_task.py        (with the yam_arena package, or the offline stub, importable)
"""

import importlib.util
import math
import os

import numpy as np
from yam_arena.task_file import ObjectState

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("pastry_task", os.path.join(HERE, "task.py"))
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)
kit, dk = m.kit, m.dk

PARK = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
GRIPPED = 0.27  # finger opening on the 2.5 cm handle


def st(pos, quat=(0, 0, 0, 1.0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def qx(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


class Trial:
    """A fresh trial: the pin on the board behind the pastry, arms at PARK. step() runs one control step as the hook
    does: the kit's update, then success() and score()."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.pin = np.array(m.PIN_START, float)
        self.quat = (0, 0, 0, 1.0)
        self.g = dict(PARK)
        self.open = {g: 1.0 for g in PARK}
        self.last = None
        self.step()

    def objects(self):
        objs = {"board": st((*m.BOARD, m.TOP)), "pin": st(self.pin, self.quat)}
        objs.update({g: st(p, (1.0, 0.0, 0.0, 0.0)) for g, p in self.g.items()})  # pointing down
        return objs

    def step(self, n=1):
        for _ in range(n):
            kit._OPENING.update(self.open)
            objs = self.objects()
            dk.update(objs)
            ok = m.success(objs)
            self.last = (m.score(objs), ok)
        return self.last

    def put(self, x, lift=0.0, tilt=0.0, hands=("left_gripper", "right_gripper")):
        """The pin's barrel line at x, its middle `lift` above resting on its rings, tilted `tilt` degrees about x
        (positive lifts the -y end ... about its middle), held by `hands`."""
        self.pin = np.array([x, 0.0, m.AXIS_Z + lift])
        self.quat = qx(tilt)
        a = dk._rotm(self.quat) @ np.array([0.0, 1.0, 0.0])
        for g, s in (("left_gripper", 1), ("right_gripper", -1)):
            if g in hands:
                self.g[g] = tuple(self.pin + s * m.GRIP_Y * a)
                self.open[g] = GRIPPED

    def roll(self, x1, speed=m.SPEED, **kw):
        x0 = float(self.pin[0])
        n = max(1, int(round(abs(x1 - x0) / (speed * 0.05))))
        for k in range(1, n + 1):
            self.put(x0 + (x1 - x0) * k / n, **kw)
            self.step()

    def lift_to(self, x):
        self.put(float(self.pin[0]), lift=m.LIFT)
        self.step(5)
        self.put(x, lift=m.LIFT)
        self.step(10)
        self.put(x)
        self.step(5)

    def let_go(self, home=True):
        for g in self.g:
            self.open[g] = 1.0
        self.step(3)
        for g in self.g:
            p = self.g[g]
            self.g[g] = (p[0], p[1], p[2] + 0.08)
        self.step(3)
        if home:
            self.g = dict(PARK)
        return self.step(25)

    def strokes(self, ends, **kw):
        for e in ends:
            self.roll(e, **kw)


def demo_rolling(t, **kw):
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES, **kw)


# -- the model --------------------------------------------------------------------------------------------------------

def test_volume_conserved():
    t = Trial()
    v0 = dk.volume()
    assert abs(v0 - float(np.prod(m.PASTRY_SIZE))) < 1e-9, v0
    demo_rolling(t)
    t.roll(0.45)  # and off the end
    assert abs(dk.volume() - v0) < 1e-12, (dk.volume(), v0)


def test_level_pin_even_sheet():
    t = Trial()
    demo_rolling(t)
    on, length, thin = m.sheet()
    assert on >= m.SPEC_SHARE and length >= m.MIN_LEN and thin >= 0.0029, (on, length, thin)
    tt = dk.grid()
    rows = [r[r > 0] for r in tt]
    assert max(abs(r.mean() - rows[0].mean()) for r in rows) < 1e-6  # every strip alike


def test_tilted_pin_wedge():
    t = Trial()
    t.lift_to(m.CENTER_X)
    # one end held 4 mm high over the 26 cm barrel: about 0.9 degrees
    tilt = math.degrees(math.atan2(0.004, 2 * m.RING_OUT))
    t.strokes(m.STROKES, lift=0.002, tilt=tilt)
    tt = dk.grid()
    lo_side = tt[0][tt[0] > 0].mean()  # -y strip
    hi_side = tt[-1][tt[-1] > 0].mean()
    assert hi_side - lo_side > 0.002 or lo_side - hi_side > 0.002, (lo_side, hi_side)


def test_edge_over_roll_not_sealed():
    """Review, Nov 12: rolling past an end presses that end like any other slice; the rings keep the barrel 3 mm up,
    so nothing goes below the gap."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.roll(0.44)  # straight off the far end
    tt = dk.grid()
    assert tt[tt > 0].min() >= m.TARGET_T - 1e-9, tt[tt > 0].min()


def test_lifted_pin_does_nothing():
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.put(m.CENTER_X, lift=0.02)
    t.step()
    for e in m.STROKES:
        t.roll(e, lift=0.02)
    assert abs(dk.lengths().min() - m.PASTRY_SIZE[0]) < 1e-9


# -- the scorer -------------------------------------------------------------------------------------------------------

def test_do_nothing_zero():
    t = Trial()
    assert t.step(40)[0] == 0.0


def test_demo_full_score():
    t = Trial()
    demo_rolling(t)
    s, ok = t.let_go()
    assert s == 1.0 and ok, s


def test_not_home_full():
    t = Trial()
    demo_rolling(t)
    assert t.let_go(home=False)[0] == 1.0  # where the arms end is not scored
    assert not hasattr(m, "REQUIRE_ARMS_HOME") and not hasattr(m, "HOME")


def test_still_held_capped():
    t = Trial()
    demo_rolling(t)
    t.g = {g: p for g, p in t.g.items()}
    assert t.step(25)[0] == 0.5  # fingers closed on the handles: the sheet is done, the pin not let go
    for g in t.g:
        t.open[g] = 1.0
    assert t.step(25)[0] == 1.0  # opened wide around the handles, touching nothing: let go (the old 1 cm zone gave 0.5)
    t.g["left_gripper"] = PARK["left_gripper"]
    t.g["right_gripper"] = (m.CENTER_X, -0.10, m.AXIS_Z + 0.02)  # a hand resting on the barrel, not the handle
    t.open["right_gripper"] = 0.0
    assert t.step(25)[0] == 0.5


def test_release_is_contact_not_nearness():
    """Let go is judged by contact (the gripper's own shape within 2 mm of the pin), not a distance zone."""
    t = Trial()
    demo_rolling(t)
    t.g = dict(PARK)
    for g in t.g:
        t.open[g] = 0.0
    top = m.AXIS_Z + m.R_BARREL + 0.0128  # closed fingertips (12.8 mm past the tool point) just on the barrel's top
    t.g["right_gripper"] = (m.CENTER_X, -0.05, top + 0.005)
    assert t.step(25)[0] == 1.0  # closed fingertips 5 mm over the barrel, not touching (the old 1 cm zone gave 0.5)
    t.g["right_gripper"] = (m.CENTER_X, -0.05, top + 0.001)
    assert t.step(25)[0] == 0.5  # 1 mm over it: within the 2 mm contact tolerance, a hand on the pin
    t.g["right_gripper"] = (m.CENTER_X, -(m.HANDLE_IN + m.HANDLE_OUT) / 2, m.AXIS_Z + m.R_HANDLE + 0.0128)
    assert t.step(25)[0] == 0.5  # fingertips resting on the right handle


def test_hover_over_handles_not_held():
    """Fingers set to the handle's width but hovering 9 mm over each handle while the pin moves through the demo's
    strokes: not touching the handles, so nothing counts (the old test, tool point within 1 cm, counted it)."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    base = t.put
    hover = 0.28  # each finger's inner face 13.1 mm from the tool point, just wider than the 12.5 mm handle

    def put(x, lift=0.0, tilt=0.0, hands=None):
        base(x, lift, tilt)
        for g in t.g:
            p = t.g[g]
            t.g[g] = (p[0], p[1], p[2] + m.R_HANDLE + 0.009)
            t.open[g] = hover

    t.put = put
    t.put(m.CENTER_X)
    objs = t.objects()
    kit._OPENING.update(t.open)
    assert not m.gk.touching(objs, "pin", m.PIN_SHAPE) and not m._gate(objs)  # the old 1 cm zone passed it
    before = dk.lengths().min()
    for e in m.STROKES:
        t.roll(e)
    assert abs(dk.lengths().min() - before) < 1e-9


def test_pin_held_up_capped():
    t = Trial()
    demo_rolling(t)
    t.put(m.CENTER_X, lift=0.03)
    t.step()
    t.pin = t.pin.copy()
    t.g = dict(PARK)
    for g in t.g:
        t.open[g] = 1.0
    assert t.step(25)[0] == 0.5  # "let go" with the pin hanging in the air (not resting on the board)


def test_one_pass_zero():
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES[:4])
    assert dk.lengths().min() < m.MIN_LEN
    assert t.let_go()[0] == 0.0


def test_one_arm_tilt_zero():
    """One hand on one handle, the other end of the pin 4 mm up: the high side stays thick and short."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    tilt = math.degrees(math.atan2(0.004, 2 * m.RING_OUT))
    for e in m.STROKES + (0.39, 0.30, 0.205, 0.30):  # extra passes
        t.roll(e, lift=0.002, tilt=tilt, hands=("left_gripper",))
    t.put(m.CENTER_X)
    t.step()
    assert dk.lengths().min() < m.MIN_LEN
    assert t.let_go()[0] == 0.0


def test_three_passes_half():
    """Three of the four passes: 19.8 cm and 85 % on spec, 0.5."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES[:12])
    on, length, _ = m.sheet()
    assert m.PART_LEN <= length < m.MIN_LEN, (on, length)
    assert t.let_go()[0] == 0.5


def test_two_even_passes_quarter():
    """Two level passes: 17.2 cm and every strip alike, 0.25 (safe progress, about two passes from done)."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES[:8])
    _, length, _ = m.sheet()
    assert m.START_LEN <= length < m.PART_LEN and m.wedge() < 1e-9, (length, m.wedge())
    assert t.let_go()[0] == 0.25


def test_wedge_from_lifted_end_no_quarter():
    """One end of the pin kept 1 mm up through eight passes: 16.8 cm long, but 0.73 mm thicker on one side, 0."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    tilt = math.degrees(math.atan2(0.001, 2 * m.RING_OUT))
    t.strokes(m.STROKES + (0.40, 0.295, 0.19, 0.295) * 3, lift=0.0005, tilt=tilt)
    _, length, _ = m.sheet()
    assert m.START_LEN <= length < m.PART_LEN and m.wedge() > m.WEDGE_MAX, (length, m.wedge())
    assert t.let_go()[0] == 0.0


def test_wedge_blocks_full_credit():
    """Scorer check: a sheet long enough and all on spec, but 0.55 mm thicker on one side than the other (2.65 to 3.20
    mm across the width), is not even: 0.5, not 1.0."""
    t = Trial()
    demo_rolling(t)
    S = dk._state()
    for j in range(S.t.shape[0]):
        S.t[j][:] = 0.00265 + 0.00055 * j / (S.t.shape[0] - 1)
    on, length, _ = m.sheet()
    assert on >= m.SPEC_SHARE and length >= m.MIN_LEN and m.wedge() > m.WEDGE_MAX, (on, length, m.wedge())
    assert t.let_go()[0] == 0.5
    for j in range(S.t.shape[0]):
        S.t[j][:] = 0.00265 + 0.00045 * j / (S.t.shape[0] - 1)  # 0.45 mm: within the 0.5 mm
    assert t.step()[0] == 1.0


def test_roll_over_edges_scored_on_the_sheet():
    """Review, Nov 12: full-length strokes over both ends are not an automatic 0; the sheet they leave is scored."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes((0.43, 0.17, 0.43, 0.17, 0.43, 0.17, 0.43, 0.17, 0.30))
    on, length, thin = m.sheet()
    want = 1.0 if (length >= m.MIN_LEN and on >= m.SPEC_SHARE and m.wedge() <= m.WEDGE_MAX) else (
        0.5 if length >= m.PART_LEN else 0.0)
    assert thin >= m.TARGET_T - 1e-9 and want > 0, (on, length, thin)
    assert t.let_go()[0] == want


def test_flick_past_end_no_penalty():
    t = Trial()
    demo_rolling(t)
    end = dk.ends()[:, 1].max()
    t.roll(end + 0.012, speed=0.1)  # a quick 1.2 cm overshoot (0.12 s) ...
    t.roll(m.CENTER_X, speed=0.1)  # ... straight back
    assert t.let_go()[0] == 1.0


def test_one_step_pop_past_end_not_sealed():
    """A single-step physics pop of the pin 7 mm past the end and straight back: debounced, not sealed."""
    t = Trial()
    demo_rolling(t)
    end = dk.ends()[:, 1].max()
    t.roll(end + 0.007, speed=0.5)
    t.roll(end - 0.03, speed=0.5)
    t.roll(m.CENTER_X)
    assert t.let_go()[0] == 1.0


def test_hand_off_the_pin_pauses_counting():
    """A stroke with one hand lifted off the pin does not press; once both hands are back on it, rolling counts again
    and the sheet is scored as it ends."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES[:4])
    base = t.put

    def put(x, lift=0.0, tilt=0.0, hands=None):
        base(x, lift, tilt, hands=("right_gripper",))
        t.g["left_gripper"] = (x, 0.10, m.AXIS_Z + 0.06)  # lifted 3 cm clear of the barrel
        t.open["left_gripper"] = 0.53

    t.put = put
    before = dk.lengths().min()
    t.roll(0.36)
    t.roll(m.CENTER_X)
    assert abs(dk.lengths().min() - before) < 1e-9  # nothing pressed while a hand was off the pin
    t.put = base
    t.strokes(m.STROKES)
    assert t.let_go()[0] == 1.0


def test_one_arm_center_grip_zero():
    """QC attack: one gripper closed on the barrel's middle, the pin level by symmetry: nothing counts."""
    t = Trial()

    def put1(x, lift=0.0, tilt=0.0, hands=None):
        t.pin = np.array([x, 0.0, m.AXIS_Z + lift])
        t.quat = qx(tilt)
        t.g["left_gripper"] = tuple(t.pin)
        t.open["left_gripper"] = 0.53

    t.put = put1
    demo_rolling(t)
    assert abs(dk.lengths().min() - m.PASTRY_SIZE[0]) < 1e-9  # nothing counts
    t.pin = np.array(m.PIN_START, float)
    assert t.let_go()[0] == 0.0


def test_one_handle_one_barrel_counts():
    """Mock review, Nov 12: the instruction does not ask for the handles, so a hand on the barrel counts. Right hand on
    its handle, left hand closed on the barrel near its ring: both hands on the pin, the strokes press, 1.0."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    base = t.put

    def put(x, lift=0.0, tilt=0.0, hands=None):
        base(x, lift, tilt, hands=("right_gripper",))
        t.g["left_gripper"] = (x, 0.10, m.AXIS_Z + lift)  # closed on the barrel, inboard of the left ring
        t.open["left_gripper"] = 0.53

    t.put = put
    for e in m.STROKES:
        t.roll(e)
    assert dk.lengths().min() >= m.MIN_LEN
    assert t.let_go()[0] == 1.0


def test_palms_on_barrel_count():
    """Both hands pressing down on top of the barrel with closed fists (fingertips resting on it, no grip), as a baker
    rolls with the palms: both hands on the pin, the strokes press, 1.0."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    base = t.put
    top = m.R_BARREL + 0.0128 + 0.001  # closed fingertips 1 mm over the barrel's top: within the 2 mm contact

    def put(x, lift=0.0, tilt=0.0, hands=None):
        base(x, lift, tilt)
        for g, y in (("left_gripper", 0.06), ("right_gripper", -0.06)):
            t.g[g] = (x, y, m.AXIS_Z + lift + top)
            t.open[g] = 0.0

    t.put = put
    for e in m.STROKES:
        t.roll(e)
    assert dk.lengths().min() >= m.MIN_LEN
    t.put = base
    assert t.let_go()[0] == 1.0


def test_one_hand_on_handle_other_hovering_no_press():
    """One hand on its handle, the other hovering 5 mm over the barrel (not touching): one hand on the pin, no
    press."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    base = t.put
    top = m.R_BARREL + 0.0128 + 0.005

    def put(x, lift=0.0, tilt=0.0, hands=None):
        base(x, lift, tilt, hands=("right_gripper",))
        t.g["left_gripper"] = (x, 0.06, m.AXIS_Z + lift + top)
        t.open["left_gripper"] = 0.0

    t.put = put
    for e in m.STROKES:
        t.roll(e)
    assert abs(dk.lengths().min() - m.PASTRY_SIZE[0]) < 1e-9


def test_open_hands_pushing_handles_no_press():
    """Both hands open wide around the handles, touching nothing while the pin moves: not on the pin, nothing
    presses."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    for e in m.STROKES:
        x0 = float(t.pin[0])
        n = max(1, int(round(abs(e - x0) / (m.SPEED * 0.05))))
        for k in range(1, n + 1):
            t.put(x0 + (e - x0) * k / n)
            for g in t.open:
                t.open[g] = 1.0
            t.step()
    assert abs(dk.lengths().min() - m.PASTRY_SIZE[0]) < 1e-9


def test_fast_correct_rolling_full():
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES, speed=0.4)  # 2 cm per control step
    assert t.let_go()[0] == 1.0


def test_pin_set_down_off_center_ok():
    t = Trial()
    demo_rolling(t)
    t.lift_to(m.PIN_START[0])  # back where it started (it lifts clear of the pastry)
    assert t.let_go()[0] == 1.0


def test_pin_back_in_rest_full():
    t = Trial()
    demo_rolling(t)
    t.put(m.CENTER_X, lift=m.LIFT)
    t.step(5)
    t.put(m.REST_X, lift=m.LIFT)
    t.step(10)
    t.put(m.REST_X, lift=m.REST_AXIS - m.AXIS_Z)
    t.step(5)
    assert t.let_go()[0] == 1.0


def test_numbers_in_text():
    doc = m.score.__doc__
    for s in ("20 cm", "18 cm", "16 cm", "90 %", "3 mm", "0.5 mm", "3 degrees", "5 mm", "1 cm", "0.25"):
        assert s in doc, s
    assert m.MIN_LEN == 0.20 and m.SPEC_SHARE == 0.90 and m.TOL_T == 0.0005 and m.LEVEL_DEG == 3.0
    assert m.PART_LEN == 0.18 and m.START_LEN == 0.16 and m.WEDGE_MAX == 0.0005
    assert "2 mm" in doc and m.PIN_DOWN == 0.005 and m.gk.CONTACT == 0.002
    assert "both hands on the pin" in doc and "sealed" not in doc and "torn" not in doc
    assert not hasattr(m, "TOUCH") and not hasattr(m, "CLEAR") and not hasattr(m, "BARREL")  # no distance zones


if __name__ == "__main__":
    for name, f in list(globals().items()):
        if name.startswith("test_"):
            f()
            print("ok", name)
    print("all pass")
