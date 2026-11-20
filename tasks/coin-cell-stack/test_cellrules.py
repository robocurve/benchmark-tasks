"""Offline checks of the scoring rules on hand-built trials: no simulator.  python -m unittest test_cellrules"""

from __future__ import annotations

import unittest

import numpy as np

import cellrules as cr

TOP = 0.7475
AXIS = (0.32, -0.06)
FLOOR = TOP + 0.019 + 0.00025
RIM = TOP + 0.019 + 0.0029
LAY = cr.Layout(top=TOP, axis=AXIS, floor=FLOOR, rim=RIM, stand=(0.28, 0.02, TOP + 0.031), holder=(0.20, 0.09, TOP + 0.0193))
HOME = {"zinc_1": (0.295, -0.17), "separator": (0.32, -0.17), "zinc_2": (0.345, -0.17), "spacer": (0.195, -0.13), "spring": (0.22, -0.13), "cap": (0.245, -0.13)}
FAR = np.array([[0.25, 0.25, TOP + 0.2]] * 12)  # fingertips well away from everything


def rot_x(deg):
    a = np.radians(deg)
    return np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])


def stacked(offsets=None, tilt=None, spring_give=0.0, cap_drop=0.0):
    """Parts lying in the can, each on the one below; `offsets` shifts parts in x (m)."""
    offsets, tilt = offsets or {}, tilt or {}
    out, z = {}, FLOOR
    for n in cr.STACK:
        dz = -spring_give if n == "spring" else 0.0
        out[n] = cr.Part(np.array([AXIS[0] + offsets.get(n, 0.0), AXIS[1], z + dz]), rot_x(tilt.get(n, 0.0)))
        z = z + dz + cr.THICK[n]
    out["cap"] = cr.Part(np.array([AXIS[0] + offsets.get("cap", 0.0), AXIS[1], z - (cr.CAP_H - cr.CAP_TOP_T) - cap_drop]), rot_x(tilt.get("cap", 0.0)))
    return out


def at_home():
    return {n: cr.Part(np.array([*HOME[n], TOP + 0.0035]), np.eye(3)) for n in cr.ORDER}


def snap(t, parts, left=FAR, right=FAR, can_g=0.1048, spilled=0.0, disp=(2.5,), pressing=False, stowed=True):
    tw = ((0.28, 0.02, TOP + 0.007), (0.28, 0.02, TOP + 0.126)) if stowed else ((0.40, 0.10, TOP + 0.001), (0.40, 0.22, TOP + 0.001))
    pip = (LAY.holder, np.array(LAY.holder) + [0, 0, 0.262])
    return cr.Snapshot(t=t, parts=parts, left=np.asarray(left, float), right=np.asarray(right, float), tweezers=tw, tweezers_tilt=0.0 if stowed else 90.0,
                       pipette=pip, pipette_tilt=0.0, can_g=can_g, spilled_g=spilled, dispense_times=disp, cap_pressing=pressing)


def trial(final, *, frames=None, can_g=0.1048, spilled=0.0, disp=(2.5,), press=True, stowed=True, left=FAR, right=FAR, skip=(), order=cr.ORDER, hold=1.2,
          pre=None, sink=0.0):
    """Lay the parts in `order`, one per second (the dispense at 2.5 s), press the cap, then hold `final`."""
    j = cr.CellJudge(LAY)
    frames = frames or final
    parts = at_home()
    t = 0.0
    j.update(snap(t, dict(parts), can_g=0.0, disp=()))
    for k, n in enumerate(order):
        if n in skip:
            continue
        t = k + 1.0
        parts[n] = frames[n]
        g = can_g if t > 2.5 else 0.0
        j.update(snap(t, dict(parts), can_g=g, spilled=spilled if t > 2.5 else 0.0, disp=tuple(d for d in disp if d < t)))
        if pre:
            pre(j, t, n, parts)
    t += 1.0
    parts = {**parts, **{n: final[n] for n in final if n not in skip}}
    j.update(snap(t, dict(parts), can_g=can_g, spilled=spilled, disp=disp))
    if press and "cap" not in skip:
        pressed = dict(parts)
        pressed["cap"] = cr.Part(parts["cap"].pos - [0, 0, 0.0005], parts["cap"].rot)
        pressed["spring"] = cr.Part(parts["spring"].pos - [0, 0, 0.0], parts["spring"].rot)
        t += 0.05
        j.update(snap(t, pressed, can_g=can_g, spilled=spilled, disp=disp, pressing=True, right=FAR))
        t += 0.05
        if sink:  # the press leaves the upper parts sunk into the ones below (simulated thin discs)
            parts = {n: cr.Part(p.pos - [0, 0, sink], p.rot) if n in ("zinc_2", "spacer", "spring", "cap") else p for n, p in parts.items()}
        j.update(snap(t, dict(parts), can_g=can_g, spilled=spilled, disp=disp))
    t += 0.05  # the left hand lets go of the tweezers in the stand
    j.update(snap(t, dict(parts), left=np.array([[0.28, 0.035, TOP + 0.09]] * 12), can_g=can_g, spilled=spilled, disp=disp, stowed=stowed))
    s = None
    for _ in range(int(hold / 0.05)):
        t += 0.05
        s = snap(t, dict(parts), left=left, right=right, can_g=can_g, spilled=spilled, disp=disp, stowed=stowed)
        j.update(s)
    return j.tier(s)


class Tiers(unittest.TestCase):
    def test_full(self):
        self.assertEqual(trial(stacked())[0], 1.0)

    def test_not_held_a_second(self):
        self.assertEqual(trial(stacked(), hold=0.5)[0], 0.5)

    def test_zinc_off_centre(self):
        self.assertEqual(trial(stacked({"zinc_2": 0.0015, "spacer": 0.0015}))[0], 0.5)
        self.assertEqual(trial(stacked({"zinc_2": 0.0025, "spacer": 0.0015}))[0], 0.0)

    def test_hidden_layer_distance(self):
        # each zinc 0.8 mm off, on opposite sides: offsets pass, the centre distance (1.6 mm) does not
        f = stacked({"zinc_1": -0.0008, "zinc_2": 0.0008})
        self.assertEqual(trial(f)[0], 0.5)

    def test_spacer(self):
        self.assertEqual(trial(stacked({"spacer": 0.0012}))[0], 1.0)
        self.assertEqual(trial(stacked({"spacer": 0.0017}))[0], 0.5)
        self.assertEqual(trial(stacked({"spacer": 0.0019}))[0], 0.0)  # it touches the wall: a short

    def test_spacer_frame_on_the_wall(self):
        # the spacer's frame offset at or past the wall gives 0 before the 0.25 branch, even if the meter read passes
        orig = cr.CellJudge.no_short
        cr.CellJudge.no_short = lambda self, s: True
        try:
            for press in (True, False):
                self.assertEqual(trial(stacked({"spacer": 0.00186}), press=press)[0], 0.0)
            self.assertEqual(trial(stacked({"spacer": 0.0018}), press=False)[0], 0.25)
        finally:
            cr.CellJudge.no_short = orig

    def test_electrolyte(self):
        self.assertEqual(trial(stacked(), can_g=0.065)[0], 0.5)
        self.assertEqual(trial(stacked(), can_g=0.04)[0], 0.0)
        self.assertEqual(trial(stacked(), can_g=0.0, disp=())[0], 0.0)
        self.assertEqual(trial(stacked(), spilled=0.1)[0], 0.0)
        self.assertEqual(trial(stacked(), disp=(3.5,))[0], 0.0)  # after the second zinc

    def test_tools(self):
        self.assertEqual(trial(stacked(), stowed=False)[0], 0.5)
        near = np.array([[0.28, 0.02 + 0.02, TOP + 0.1]] * 12)
        self.assertEqual(trial(stacked(), left=near)[0], 0.5)
        over_cell = np.array([[AXIS[0], AXIS[1], RIM + 0.025]] * 12)  # the press hand 25 mm over the rim: 23 mm over the cap
        self.assertEqual(trial(stacked(), right=over_cell)[0], 0.5)
        self.assertEqual(trial(stacked(), right=over_cell + [0, 0, 0.008])[0], 1.0)

    def test_cap(self):
        self.assertEqual(trial(stacked(), press=False)[0], 0.25)
        self.assertEqual(trial(stacked(), skip=("cap",))[0], 0.25)
        flipped = stacked()
        flipped["cap"] = cr.Part(flipped["cap"].pos, rot_x(180))
        self.assertEqual(trial(flipped)[0], 0.25)
        self.assertEqual(trial(stacked({"cap": 0.001}))[0], 0.25)

    def test_flat_and_order(self):
        # the depth gauge on the spacer: a raised edge under it tilts it (2 degrees: 0.45 mm across the gauge's points)
        self.assertEqual(trial(stacked(tilt={"spacer": 2.0}))[0], 0.0)
        self.assertEqual(trial(stacked(tilt={"spacer": 1.0}))[0], 1.0)  # 0.23 mm spread: within 0.3
        # a stack 0.4 mm too high (a folded separator, say): out
        high = stacked()
        high = {n: cr.Part(p.pos + [0, 0, 0.0004], p.rot) if n in ("zinc_2", "spacer", "spring", "cap") else p for n, p in high.items()}
        self.assertTrue(trial(high)[1].startswith("stack too high"))
        swapped = ("separator", "zinc_1", "zinc_2", "spacer", "spring", "cap")
        self.assertEqual(trial(stacked(), order=swapped)[0], 0.0)
        self.assertEqual(trial(stacked(), skip=("separator",))[0], 0.0)

    def test_gripper_moves_a_part(self):
        # a fingertip pushes the spring on its dish: 1.1 mm in one contact is displaced, 0.5 mm is not
        def poke(dist):
            def f(j, t, n, parts):
                if n == "spacer":
                    p = parts["spring"]
                    fingers = np.array([p.pos + [0.0085, 0, 0.0005]] * 12)
                    moved = dict(parts)
                    moved["spring"] = cr.Part(p.pos + [dist, 0, 0], p.rot)
                    j.update(snap(t + 0.05, moved, left=fingers, disp=(2.5,)))
                    j.update(snap(t + 0.1, moved, disp=(2.5,)))
            return f
        score, why = trial(stacked(), pre=poke(0.0011))
        self.assertEqual(score, 0.0)
        self.assertTrue(why.startswith("the left gripper moved the spring"), why)
        self.assertEqual(trial(stacked(), pre=poke(0.0005))[0], 1.0)

    def test_press_exemption_is_narrow(self):
        # the right hand presses the cap while its fingers also shove a part lying outside the can: still a fault
        def shove(j, t, n, parts):
            if n == "cap":
                moved = dict(parts)
                p = cr.Part(np.array([0.40, 0.0, TOP + 0.0035]), np.eye(3))
                moved["spring"] = p
                q = dict(moved)
                q["spring"] = cr.Part(p.pos + [0.001, 0, 0], p.rot)
                fingers = np.array([p.pos + [0.008, 0, 0.0005]] * 12)
                j.update(snap(t + 0.05, moved, right=fingers, disp=(2.5,), pressing=True))
                j.update(snap(t + 0.1, q, right=fingers, disp=(2.5,), pressing=True))
        self.assertEqual(trial(stacked(), pre=shove)[1].startswith("the right gripper moved the spring"), True)

    def test_slow_carry_by_the_fingers(self):
        # the left fingers carry the cap into the can at 0.25 mm per step: under the per-step limit, over the total
        def carry(j, t, n, parts):
            if n != "spring":
                return
            start, goal = parts["cap"].pos.copy(), stacked()["cap"].pos + [0, 0, 0.001]
            legs = ((start, start + [0, 0, 0.02]), (start + [0, 0, 0.02], goal + [0, 0, 0.02]), (goal + [0, 0, 0.02], goal))
            tt = t
            for a, b in legs:
                k = int(np.ceil(np.linalg.norm(b - a) / 0.00025))
                for i in range(1, k + 1):
                    pos = a + (b - a) * i / k
                    parts["cap"] = cr.Part(pos, np.eye(3))
                    fingers = np.array([pos + [0.0102, 0, 0.001]] * 6 + [pos + [-0.0102, 0, 0.001]] * 6)
                    tt += 1e-5
                    j.update(snap(tt, dict(parts), left=fingers, disp=(2.5,)))
            j.update(snap(tt + 1e-5, dict(parts), disp=(2.5,)))
        score, why = trial(stacked(), pre=carry)
        self.assertEqual(score, 0.0)
        self.assertTrue(why.startswith("the left gripper moved the cap"), why)

    def test_small_nudge_in_one_contact_is_allowed(self):
        # a fingertip brushes the cap on its dish and shifts it 0.2 mm in all: under the limit
        def nudge(j, t, n, parts):
            if n == "spacer":
                p = parts["cap"]
                fingers = np.array([p.pos + [0.011, 0, 0.001]] * 12)
                for k in range(1, 5):
                    j.update(snap(t + 0.05 * k, {**parts, "cap": cr.Part(p.pos + [0.00005 * k, 0, 0], p.rot)}, left=fingers, disp=(2.5,)))
                j.update(snap(t + 0.3, dict(parts), disp=(2.5,)))
        self.assertEqual(trial(stacked(), pre=nudge)[0], 1.0)
        # the same brush carried on to 1.2 mm in all: displaced
        def push(j, t, n, parts):
            if n == "spacer":
                p = parts["cap"]
                fingers = np.array([p.pos + [0.011, 0, 0.001]] * 12)
                for k in range(1, 25):
                    j.update(snap(t + 0.05 * k, {**parts, "cap": cr.Part(p.pos + [0.00005 * k, 0, 0], p.rot)}, left=fingers, disp=(2.5,)))
                j.update(snap(t + 0.5, dict(parts), disp=(2.5,)))
        self.assertEqual(trial(stacked(), pre=push)[0], 0.0)

    def test_finger_travel_adds_up_over_contacts(self):
        # two separate brushes of 0.6 mm each: 1.2 mm in all, displaced; one alone is allowed
        def brushes(count):
            def f(j, t, n, parts):
                if n != "spacer":
                    return
                p, tt = parts["cap"], t
                for c in range(count):
                    for k in range(1, 13):
                        tt += 0.01
                        q = cr.Part(p.pos + [0.00005 * k, 0, 0], p.rot)
                        j.update(snap(tt, {**parts, "cap": q}, left=np.array([q.pos + [0.011, 0, 0.001]] * 12), disp=(2.5,)))
                    p = q
                    tt += 0.01
                    j.update(snap(tt, {**parts, "cap": p}, disp=(2.5,)))
                parts["cap"] = p
            return f
        self.assertEqual(trial(stacked(), pre=brushes(1))[0], 1.0)
        score, why = trial(stacked(), pre=brushes(2))
        self.assertEqual(score, 0.0)
        self.assertTrue(why.startswith("the left gripper moved the cap"), why)

    def test_part_leaves_the_can(self):
        # the first zinc is lifted back out of the can after it was laid in, then laid again
        def lift(j, t, n, parts):
            if n == "zinc_1":
                j.update(snap(t + 0.05, {**parts, "zinc_1": cr.Part(parts["zinc_1"].pos + [0, 0, 0.01], np.eye(3))}, can_g=0.0, disp=()))
        score, why = trial(stacked(), pre=lift)
        self.assertEqual(score, 0.0)
        self.assertTrue(why.startswith("zinc_1 left the can"), why)

    def test_physical_order_swapped(self):
        # separator under zinc_1, the zinc discs touching, arrival order right: swapped after the cap went in
        f = stacked()
        sw = dict(f)
        sw["separator"] = cr.Part(np.array([*AXIS, FLOOR]), np.eye(3))
        sw["zinc_1"] = cr.Part(np.array([*AXIS, FLOOR + cr.THICK["separator"]]), np.eye(3))
        self.assertEqual(trial(sw, frames=f)[0], 0.0)
        # and the same stack in place as each part arrives (zinc_1 held up while the separator went under it)
        self.assertEqual(trial(sw)[0], 0.0)

    def test_sink_before_and_after_the_press(self):
        # the press leaves the upper discs sunk 0.3 mm (simulated thin discs): the gauge reads lower, the meter is
        # judged from before the press, so it still scores
        self.assertEqual(trial(stacked(), sink=0.0003)[0], 1.0)
        # but the second zinc sunk through the separator before any press: the zinc discs touch, a short
        f = stacked()
        sunk = dict(f)
        sunk["zinc_2"] = cr.Part(f["separator"].pos.copy(), f["zinc_2"].rot)
        self.assertEqual(trial(sunk, press=False)[0], 0.0)

    def test_spring_on_the_wall(self):
        # the spring 2.3 mm off the axis touches the can's wall, seen in its frame before the cap covers it
        score, why = trial(stacked({"spring": 0.0023}))
        self.assertEqual(score, 0.0)
        self.assertTrue(why.startswith("spring offset"), why)

    def test_quarter_tier_without_cap_or_spring(self):
        # the meter and the scale need neither: the 0.25 tier with the cap, or both the cap and the spring, missing
        self.assertEqual(trial(stacked(), skip=("cap",))[0], 0.25)
        self.assertEqual(trial(stacked(), skip=("spring", "cap"))[0], 0.25)
        # and a zinc-to-zinc contact (separator under the first zinc) still reads as a short without them
        f = stacked()
        sw = dict(f)
        sw["separator"] = cr.Part(np.array([*AXIS, FLOOR]), np.eye(3))
        sw["zinc_1"] = cr.Part(np.array([*AXIS, FLOOR + cr.THICK["separator"]]), np.eye(3))
        self.assertEqual(trial(sw, skip=("cap",))[0], 0.0)

    def test_wet_gripper(self):
        def dip(j, t, n, parts):
            if n == "zinc_2":
                fingers = np.array([[AXIS[0], AXIS[1], FLOOR + 0.001]] * 12)
                j.update(snap(t + 0.05, dict(parts), left=fingers, disp=(2.5,)))
        self.assertEqual(trial(stacked(), pre=dip)[0], 0.0)

    def test_left_table(self):
        f = stacked()
        f["spring"] = cr.Part(np.array([0.5, 0.5, TOP - 0.3]), np.eye(3))
        self.assertEqual(trial(f)[0], 0.0)

    def test_nothing(self):
        j = cr.CellJudge(LAY)
        s = snap(1.0, at_home(), can_g=0.0, disp=())
        j.update(s)
        self.assertEqual(j.tier(s)[0], 0.0)


if __name__ == "__main__":
    unittest.main()
