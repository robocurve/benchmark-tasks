"""Offline checks of the formula-bottle scorer: step-by-step scoop poses through dosekit's powder model, near misses
included, and a replay of the scripted demo's waypoints (perfect tracking), no simulator.

    PY=R/venv/bin/python R/yam-kits/chat/offline/runtest.sh test_task.py      (prints "all pass")
"""

import importlib.util
import math
import os

import numpy as np
from yam_arena.task_file import ObjectState

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("formula_task", os.path.join(HERE, "task.py"))
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)
kit, dk, pk = m.kit, m.dk, m.pk

TOP = m.TOP
PARK = {"left_gripper": np.array([0.20, 0.27, TOP + 0.20]), "right_gripper": np.array([0.20, -0.27, TOP + 0.20])}
TX, TY = m.TIN
RIM = m.RIM
SCOOP0 = m.HANDLE0.copy()


BAR = TY + m.BAR_Y
DOWN = (1.0, 0.0, 0.0, 0.0)  # a gripper pointing down (turned 180 deg about x): its palm above the tool point
HOLD = m.HANDLE_R / 0.047  # the opening of fingers closed on the 2.5 cm handle (the pads open 94 mm apart)


def clamp(mouth, tilt):
    """The bar's collider: a level bowl whose rim footprint overlaps the bar and whose rim would be inside the bar is
    stopped at the bar's underside (the sim's contact; offline poses are otherwise teleported)."""
    mouth = np.array(mouth, float)
    if abs(tilt) < 1e-6 and abs(mouth[1] - BAR) < m.BOWL_R_OUT + m.BAR_W / 2 and RIM < mouth[2] < RIM + m.BAR_T:
        mouth[2] = RIM
    return mouth


def qy(deg):
    a = math.radians(deg) / 2
    return np.array([0.0, math.sin(a), 0.0, math.cos(a)])


def st(pos, q=(0, 0, 0, 1.0), vel=(0, 0, 0), half=0.01):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(q, float), lin_vel=np.array(vel, float), ang_vel=np.zeros(3),
                       low=p - half, high=p + half)


class Trial:
    """A fresh trial: the scene at its start, the arms parked at home."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.scoop, self.q = SCOOP0.copy(), np.array([0, 0, 0, 1.0])
        self.g = {k: v.copy() for k, v in PARK.items()}
        self.vel = np.zeros(3)
        self.out = None
        kit._OPENING.update({"left_gripper": 1.0, "right_gripper": 1.0})
        self.step()

    def objs(self):
        return {"tin": st((TX, TY, TOP)), "stand": st((*m.STAND, TOP)), "bottle": st((*m.BOTTLE, TOP)),
                "scoop": st(self.scoop, self.q, self.vel), **{k: st(v, DOWN) for k, v in self.g.items()}}

    def step(self, n=1):
        for _ in range(n):
            o = self.objs()
            dk.update(o)
            ok = m.success(o)
            self.out = (m.score(o), ok)
        return self.out

    # -- moving the scoop (held in the right hand: the tool point follows the handle) --
    def put(self, mouth, tilt=0.0, n=1):
        """Scoop turned `tilt` deg about y, its rim center at `mouth`, over `n` steps from where it is."""
        q = qy(tilt)
        target = clamp(mouth, tilt) - pk._rotm(q) @ m.MOUTH_OFF
        start = self.scoop.copy()
        for k in range(1, n + 1):
            self.scoop = start + (target - start) * k / n
            self.q = q
            self.g["right_gripper"] = self.scoop + [0, m.GRIP_DY, 0]
            kit._OPENING["right_gripper"] = HOLD
            self.step()

    def put_lip(self, xy, z, tilt):
        q = qy(tilt)
        fr = dk._frame({"scoop": st((0, 0, 0), q)}, m.SCOOP_BIN)
        self.scoop = np.array([xy[0], xy[1], z]) - fr["lip"]
        self.q = q
        self.g["right_gripper"] = self.scoop + [0, m.GRIP_DY, 0]
        kit._OPENING["right_gripper"] = HOLD
        self.step()

    def dip(self):
        surf = dk.surface_z(self.objs(), "tin")
        self.put((TX, TY - 0.025, TOP + 0.15), n=4)
        self.put((TX, TY - 0.025, surf - 0.010), n=8)
        self.step(10)
        self.put((TX, TY - 0.017, surf + 0.05), n=6)

    def strike(self, dz=0.002, enter=-0.001):
        """As the demo: rim just under the bar, leading wall under it, press up (aim `dz` over the underside: the
        collider stops it there), draw along the bar, drop 4 mm, back out, lift."""
        self.put((TX, TY - 0.017, RIM + enter), n=4)
        self.put((TX, BAR - 0.016, RIM + enter), n=10)
        self.put((TX, BAR - 0.016, RIM + dz), n=3)
        self.put((TX, BAR + 0.014, RIM + dz), n=40)
        self.put((TX, BAR + 0.014, RIM - 0.004), n=3)
        self.put((TX, BAR - 0.028, RIM - 0.004), n=30)
        self.put((TX, BAR - 0.028, TOP + 0.15), n=4)

    def shed(self):
        """Tip the heaped scoop 44 degrees over the tin (just under the angle the body starts to pour): the model
        sheds exactly the heap back into the tin."""
        self.put((TX, TY - 0.02, TOP + 0.14), n=4)
        self.put((TX, TY - 0.02, TOP + 0.14), tilt=44, n=6)
        self.step(20)
        self.put((TX, TY - 0.02, TOP + 0.15), n=4)

    def pour(self, xy=m.BOTTLE):
        self.put((xy[0] - 0.02, xy[1], TOP + 0.16), n=10)
        for t in np.linspace(0, 129, 20):
            self.put_lip(xy, TOP + m.BOTTLE_H + 0.035, t)
        self.step(20)
        for t in np.linspace(129, 0, 10):
            self.put_lip(xy, TOP + m.BOTTLE_H + 0.035, t)

    def press(self):
        """The left fingertip pushed 1 cm into the scoop's powder and back out (packing it down), twice."""
        rim = self.scoop + pk._rotm(self.q) @ m.MOUTH_OFF
        for _ in range(2):
            self.g["left_gripper"] = rim + [0, 0, 0.02]
            self.step()
            self.g["left_gripper"] = rim + [0, 0, -0.010]
            self.step(2)
            self.g["left_gripper"] = rim + [0, 0, 0.03]
            self.step()
        self.g["left_gripper"] = PARK["left_gripper"].copy()

    def set_down(self, home=True, n=30):
        self.put(SCOOP0 + m.MOUTH_OFF, n=8)
        kit._OPENING["right_gripper"] = 1.0
        self.g["right_gripper"] = PARK["right_gripper"].copy() if home else np.array([0.25, -0.15, TOP + 0.12])
        return self.step(n)

    def scoops(self, n=2, dz=0.002, packed=False, strike=True, enter=-0.001, shed=False):
        for _ in range(n):
            self.dip()
            if packed:
                self.press()
            if shed:
                self.shed()
            if strike:
                self.strike(dz, enter)
            else:
                self.put((TX, TY - 0.017, TOP + 0.15), n=4)
            self.pour()


fails = 0


def expect(name, got, want, extra=""):
    global fails
    ok = abs(got[0] - want) < 1e-9 and got[1] == (want == 1.0)
    fails += not ok
    print(f"{'ok  ' if ok else 'FAIL'} {got[0]:.2f} (want {want:.2f}) {name} {extra}")


def g_in():
    return f"[bottle {dk.amount(None, 'bottle'):.2f} g, spilled {dk.spilled():.2f} g]"


# --- the numbers the scoring rests on
print(f"level scoop {m.LEVEL_G:.2f} g, dose {m.DOSE_G:.2f} g, window {m.DOSE_G * 0.93:.2f}-{m.DOSE_G * 1.07:.2f} g")

t = Trial()
expect("do nothing", t.step(40), 0.0, g_in())

t = Trial()
t.scoops(2)
expect("two level scoops, scoop back on its stand, let go", t.set_down(), 1.0, g_in())

t = Trial()
t.scoops(2)
expect("two level scoops, right hand left open 4 cm over the scoop (arm-home is not scored)", t.set_down(home=False),
       1.0, g_in())

t = Trial()
t.scoops(2)
t.put(SCOOP0 + m.MOUTH_OFF, n=8)
expect("two level scoops, scoop on its stand, still closed on the handle", t.step(30), 0.5, g_in())
kit._OPENING["right_gripper"] = 1.0
expect("... fingers opened around the handle, not touching it (let go)", t.step(30), 1.0, g_in())
t.g["right_gripper"] = t.scoop + [-0.0345, m.GRIP_DY, 0]
expect("... open hand moved over so one pad rests against the handle (touching)", t.step(30), 0.5, g_in())
t.g["right_gripper"] = t.scoop + [-0.0345, m.GRIP_DY, 0.03]
expect("... lifted 3 cm, the pad 5 mm clear of the handle (no contact, however close)", t.step(30), 1.0, g_in())

t = Trial()
t.scoops(2)
t.set_down()
t.vel = np.array([0.0, 0.05, 0.0])
expect("two level scoops, scoop still sliding", t.step(), 0.5)

t = Trial()
t.scoops(1)
expect("one level scoop (half strength)", t.set_down(), 0.25, g_in())

t = Trial()
t.scoops(1, strike=False)
expect("one heaped scoop (not leveled)", t.set_down(), 0.0, g_in())

t = Trial()
t.scoops(1, strike=False)
t.scoops(1)
expect("one heaped scoop and one level: too strong, latched", t.set_down(), 0.0, g_in())

t = Trial()
t.scoops(2, strike=False)
expect("two heaped scoops", t.set_down(), 0.0, g_in())

t = Trial()
t.scoops(2, packed=True)
expect("two scoops pressed down with a finger, then leveled", t.set_down(), 0.0, g_in())

t = Trial()
t.scoops(3)
expect("three level scoops", t.set_down(), 0.0, g_in())

t = Trial()
t.scoops(2, dz=-0.010)
expect("drawn 1 cm under the bar's height: still heaped", t.set_down(), 0.0, g_in())

for dz, want in ((0.0, 1.0), (-0.001, 1.0), (-0.003, 0.0)):
    t = Trial()
    t.scoops(2, dz=dz, enter=min(dz, -0.001))
    expect(f"rim drawn {dz * 1000:+.0f} mm from the bar's underside", t.set_down(), want, g_in())

t = Trial()
t.scoops(2, dz=0.0035)
expect("pressed up 3.5 mm into the bar: its collider stops the rim at the underside, level", t.set_down(), 1.0,
       g_in())

t = Trial()
t.scoops(2, dz=0.005, enter=0.005)
expect("carried over the bar's top (rim 5 mm up, bar 4 mm thick): not leveled", t.set_down(), 0.0, g_in())

t = Trial()
t.scoops(2, strike=False, shed=True)
expect("heap shed by tipping the scoop 44 deg over the tin, no bar: 0", t.set_down(), 0.0, g_in())

t = Trial()
t.scoops(1)
t.scoops(1, strike=False, shed=True)
expect("one scoop leveled at the bar, the second only tipped: 0 (latched)", t.set_down(), 0.0, g_in())

t = Trial()
t.scoops(2, shed=True)
expect("tipped over the tin and then drawn along the bar: 1.0", t.set_down(), 1.0, g_in())

t = Trial()
t.scoops(1)
t.dip()
t.strike()
t.pour(xy=(m.BOTTLE[0] - 0.06, m.BOTTLE[1]))
expect("second scoop poured beside the bottle: one level scoop in", t.set_down(), 0.25, g_in())

t = Trial()
t.scoops(2)
t.dip()
t.strike()
t.pour(xy=(m.BOTTLE[0] - 0.06, m.BOTTLE[1]))
expect("two level scoops, then a third poured on the table (spilled)", t.set_down(), 0.5, g_in())

t = Trial()
t.dip()
t.strike()
t.put((m.BOTTLE[0], m.BOTTLE[1], TOP + 0.16), n=10)
t.step(10)
expect("level scoop held over the bottle, never tipped (hovering)", t.set_down(), 0.0, g_in())

t = Trial()
t.scoop = np.array([0.22, -0.13, TOP + 0.02])
t.q = qy(90)
expect("scoop knocked off its stand onto the counter", t.step(30), 0.0, g_in())
t.scoop = np.array([0.22, -0.13, TOP - 0.30])
t.q = qy(0)
expect("scoop knocked off the table", t.step(30), 0.0)

t = Trial()
t.g["left_gripper"] = np.array([TX, TY, RIM - 0.02])
t.step(3)
t.g["left_gripper"] = PARK["left_gripper"].copy()
t.step(3)
t.g["right_gripper"] = np.array([m.BOTTLE[0], m.BOTTLE[1], TOP + m.BOTTLE_H + 0.01])
t.step(10)
t.g["right_gripper"] = PARK["right_gripper"].copy()
expect("fingers poked into the tin and hovered at the bottle, no scoop", t.step(30), 0.0, g_in())

t = Trial()
t.scoops(2)
t.dip()
t.strike()
t.put((m.BOTTLE[0] - 0.02, m.BOTTLE[1], TOP + 0.16), n=10)
for a in np.linspace(0, 129, 20):
    t.put_lip(m.BOTTLE, TOP + m.BOTTLE_H + 0.035, a)
    if dk.amount(None, "bottle") > m.DOSE_G + 2.0:
        break  # a third scoop only partly tipped in (about 2 g over), then stopped
for a in np.linspace(a, 0, 10):
    t.put_lip(m.BOTTLE, TOP + m.BOTTLE_H + 0.035, a)
t.put((TX, TY - 0.02, TOP + 0.14), n=6)
t.put((TX, TY - 0.02, TOP + 0.14), tilt=129, n=10)  # the rest tipped back into the tin
t.put((TX, TY - 0.02, TOP + 0.15), n=6)
expect("third level scoop partly tipped in, rest back into the tin: over-strong, latched", t.set_down(), 0.0, g_in())

t = Trial()
t.scoops(2)
t.dip()
t.put((TX, TY - 0.02, TOP + 0.14), n=6)
t.put((TX, TY - 0.02, TOP + 0.14), tilt=129, n=10)  # a third, extra scoop dumped back into the tin, not the bottle
t.put((TX, TY - 0.02, TOP + 0.15), n=6)
expect("two level scoops, a third dipped and dumped back into the tin: 1.0", t.set_down(), 1.0, g_in())

# QC Nov 11: a hand on the scoop anywhere along its body (not only near the handle's center) is still holding it
for where, off in (("neck", (0.0, 0.06, -0.02)), ("bowl", (0.0, m.BOWL_Y, m.RIM_Z + 0.01))):
    t = Trial()
    t.scoops(2)
    t.put((m.BOTTLE[0] - 0.08, m.BOTTLE[1] - 0.05, TOP + 0.15), n=10)
    t.g["right_gripper"] = t.scoop + np.array(off)
    kit._OPENING["right_gripper"] = 0.1
    expect(f"two level scoops, scoop held still in the air by its {where}", t.step(30), 0.5, g_in())


def fast_strike(t, n, dz=0.002, enter=-0.001):
    """The demo's strike with the 3 cm draw along the bar done in `n` control steps."""
    t.put((TX, TY - 0.017, RIM + enter), n=4)
    t.put((TX, BAR - 0.016, RIM + enter), n=10)
    t.put((TX, BAR - 0.016, RIM + dz), n=3)
    t.put((TX, BAR + 0.014, RIM + dz), n=n)
    t.put((TX, BAR + 0.014, RIM - 0.004), n=3)
    t.put((TX, BAR - 0.028, RIM - 0.004), n=30)
    t.put((TX, BAR - 0.028, TOP + 0.15), n=4)


# QC Nov 11: a fast draw is swept over the whole band the mouth crossed between steps (dosekit, local fix)
for n in (10, 5, 3):
    t = Trial()
    for _ in range(2):
        t.dip()
        fast_strike(t, n)
        t.pour()
    expect(f"draw along the bar in {n} steps ({30 / n:.0f} mm a step)", t.set_down(), 1.0, g_in())
for n, dz, enter, what in ((3, 0.005, 0.005, "carried over the bar's top"), (3, -0.010, -0.010, "1 cm under the bar")):
    t = Trial()
    for _ in range(2):
        t.dip()
        fast_strike(t, n, dz, enter)
        t.pour()
    expect(f"fast draw ({n} steps) {what}: not leveled", t.set_down(), 0.0, g_in())

t = Trial()
t.scoops(1, strike=False)
t.scoops(1)
expect("one heaped scoop then one level (the strike-off rule catches it, whatever the grams)", t.set_down(), 0.0,
       g_in())

# history reset with kit.episode(): an over-strong trial does not carry its latch into the next one
t = Trial()
t.scoops(3)
t.set_down()
t = Trial()
t.scoops(2)
expect("new trial after an over-strong one", t.set_down(), 1.0, g_in())


# --- where the scoop is put down after two level scoops: on the table or its stand counts, in the tin or across the
# bottle's neck does not (it is in the way of capping the bottle)
for where, pos, q, want in (("back on its stand", SCOOP0, (0, 0, 0, 1.0), 1.0),
                            ("lying on the table", np.array([0.20, -0.20, TOP + 0.058]), (0, 0, 0, 1.0), 1.0),
                            ("dropped into the tin, resting on its floor", np.array([TX, TY - 0.03, TOP + 0.060]),
                             (0, 0, 0, 1.0), 0.5),
                            ("laid across the bottle's neck", np.array([m.BOTTLE[0], m.BOTTLE[1] - 0.04,
                                                                        TOP + m.BOTTLE_H + 0.06]), (0, 0, 0, 1.0), 0.5)):
    t = Trial()
    t.scoops(2)
    t.set_down()
    t.scoop, t.q, t.vel = pos.copy(), np.array(q, float), np.zeros(3)
    expect(f"two level scoops, the scoop let go {where}", t.step(30), want, g_in())


# --- the dig: the bowl tipped 35 deg toward -x, lip first under the surface, dragged 3 cm, turned level still under
t = Trial()
surf = dk.surface_z(t.objs(), "tin")
t.put((m.DIP_X0, m.DIP_Y, TOP + 0.15), n=4)
t.put((m.DIP_X0, m.DIP_Y, surf + 0.03), tilt=-35, n=6)
t.put((m.DIP_X0, m.DIP_Y, surf - 0.008), tilt=-35, n=8)
t.put((m.DIP_X0 + m.DIP_DX, m.DIP_Y, surf - 0.008), tilt=-35, n=12)
tipped = dk.amount(None, "scoop")
t.put((m.DIP_X0 + m.DIP_DX - 0.008, m.DIP_Y, surf - 0.008), n=12)
t.put((m.DIP_X0 + m.DIP_DX - 0.008, m.DIP_Y, surf + 0.05), n=8)
heaped = dk.amount(None, "scoop")
ok = tipped < heaped and abs(heaped - 11.5) < 0.1 and dk.heaped(None, "scoop")
fails += not ok
print(f"{'ok  ' if ok else 'FAIL'} dug in tipped ({tipped:.2f} g while tipped), turned level under the powder and "
      f"lifted out heaped: {heaped:.2f} g (want 11.5)")
t.strike()
ok = abs(dk.amount(None, "scoop") - m.LEVEL_G) < 0.2
fails += not ok
print(f"{'ok  ' if ok else 'FAIL'} ... then drawn along the bar: {dk.amount(None, 'scoop'):.2f} g (one level scoop "
      f"{m.LEVEL_G:.2f} g)")

# --- the visuals: the heap is drawn in the powder's own cream color (no highlight color on the scoop)
cols = dk.marker_colors()
ok = "heap_scoop" not in cols and cols["mat_formula"] == m.FORMULA.color
fails += not ok
print(f"{'ok  ' if ok else 'FAIL'} the heap is drawn in the powder's color {m.FORMULA.color}")


# --- the scripted demo, replayed with perfect tracking: the right tool point follows the waypoints (smoothstep, as
# yamkit's planner), the scoop rides in the hand from the grip to the release.
SAG = 0.0055  # measured in the sim: the loaded arm tracks 3.5 mm low and the scoop sits 2 mm low in the grip


def replay():
    from yam_arena import ik
    from yam_arena.demo import grip_rotation
    from scipy.spatial.transform import Rotation

    def orient(w):
        return kit._rot_xyz(w.roll, w.pitch) @ grip_rotation(w.yaw)

    w = m.demo()["right"]
    t = Trial()
    grip_rot = orient(kit.Tilted(0, (0, 0, 0), 0, "", m.YAW, m.ROLL, m.P0))
    tcp, rot = PARK["right_gripper"].copy(), grip_rot
    in_hand = False
    for k, wp in enumerate(w):
        if isinstance(wp, kit.Home):
            goal, grot = PARK["right_gripper"], rot
        else:
            goal, grot = np.array(wp.pos, float), orient(wp)
        n = max(1, int(round(wp.duration / m.DT)))
        start, r0 = tcp.copy(), rot
        for i in range(1, n + 1):
            s = i / n
            s = s * s * (3 - 2 * s)
            tcp, rot = start + s * (goal - start), ik.slerp_matrix(r0, grot, s)
            t.g["right_gripper"] = tcp.copy()
            if in_hand:
                rel = rot @ grip_rot.T  # the scoop has turned with the hand since the grip
                prev = t.scoop.copy()
                t.scoop = tcp + rel @ (SCOOP0 - m.TCP0) - [0.0, 0.0, SAG]
                if np.allclose(rel, np.eye(3), atol=1e-6):  # level: the bar's collider stops the rim at its underside
                    mouth = t.scoop + m.MOUTH_OFF
                    t.scoop = clamp(mouth, 0.0) - m.MOUTH_OFF
                t.vel = (t.scoop - prev) / m.DT
                t.q = Rotation.from_matrix(rel).as_quat()
            t.step()
        if wp.label == "grip":
            in_hand = True
            kit._OPENING["right_gripper"] = HOLD
        if wp.label == "let go":
            in_hand = False
            kit._OPENING["right_gripper"] = 1.0
            t.scoop, t.q, t.vel = SCOOP0.copy(), np.array([0, 0, 0, 1.0]), np.zeros(3)  # drops 4 mm onto the stand
        if wp.label.endswith("into the bottle: hold"):
            left = dk.amount(t.objs(), "scoop")
            t.left_in_scoop = max(getattr(t, "left_in_scoop", 0.0), left)
    return t, sum(x.duration for x in w)


t, dur = replay()
expect("scripted demo replayed (perfect tracking)", t.step(20), 1.0,
       f"{g_in()}, {dur:.1f} s = {int(dur / m.DT)} steps, struck {dk.struck_grams(None, 'scoop'):.2f} g")
ok = t.left_in_scoop < 0.25
fails += not ok
print(f"{'ok  ' if ok else 'FAIL'} what the tipped scoop keeps at the pour (the wrist turns it 123 deg): "
      f"{t.left_in_scoop:.3f} g left in it")
fails += dur / m.DT > 1500
print(("ok  " if dur / m.DT <= 1500 else "FAIL") + " demo fits in the README's 1,500-step demo run")

print("all pass" if not fails else f"{fails} FAILED")
