"""Offline checks of the dipstick scorer: step-by-step dipstick, rag and gripper histories, no simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
T = m.TOP
HOME = {"left_gripper": (0.10, 0.25, T + 0.20), "right_gripper": (0.10, -0.25, T + 0.20)}
RAG_HOME = m.RAG_START
OUTQ = m.BLADE + 0.012
PUCK_GRIP = m.PUCK_R / abs(m.FINGER_OPEN)  # opening of fingers closed on the 3 cm puck
HANDLE_GRIP = m.HANDLE[0] / 2 / abs(m.FINGER_OPEN)  # opening of fingers closed across the handle's 1.8 cm thickness


def st(pos, half=0.01):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - half, high=p + half)


def grip_st(pos, across_y=False):
    """A gripper pointing straight down (turned half a turn about x): the pad tips below the tool point, the palm above.
    Its fingers close along x, as in the demo's grip across the handle's 1.8 cm thickness; across_y: also turned a
    quarter turn about z, so the fingers close along y."""
    p = np.array(pos, float)
    q = [0.70710678, 0.70710678, 0.0, 0.0] if across_y else [1.0, 0.0, 0.0, 0.0]
    return ObjectState(pos=p, quat_xyzw=np.array(q), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def puck_st(pos):
    p = np.array(pos, float)
    h = np.array([m.PUCK_R, m.PUCK_R, m.PUCK_H / 2])
    return ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - h, high=p + h)


def in_pocket(k):
    """The puck lying flat on pocket k's floor."""
    return (*m.POCKETS[k], T + m.FLOOR + m.PUCK_H / 2)


def rag_st(pos):
    p = np.array(pos, float)
    h = np.array(m.RAG) / 2
    return ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - h, high=p + h)


def handle_at(q):
    return (m.SX, m.SY, T + m.MOUTH + q + m.HANDLE_C)


class Trial:
    """One trial: q (dipstick), who holds it, where the rag and the grippers are."""

    def __init__(self, case="ok"):
        kit._EPISODE["count"] += 1
        os.environ["OIL_CASE"] = case
        self.case = case
        self.puck = m.PUCK_START
        self.q, self.qd = 0.0, 0.0
        self.right, self.ropen = HOME["right_gripper"], 1.0
        self.left, self.lopen = HOME["left_gripper"], 1.0
        self.across_y = False
        self.off = (0.0, 0.0, 0.0)  # the right hand's offset from the handle's middle while it rides along
        self.rag = RAG_HOME
        self.out = (0.0, False)
        self.step()  # the trial's first step, with the arms home

    def step(self, n=1):
        for _ in range(n):
            kit._JOINTS["engine/stick"] = kit.JointState(self.q, self.qd, False)
            kit._OPENING["right_gripper"], kit._OPENING["left_gripper"] = self.ropen, self.lopen
            objs = {"engine": st(m.BASE), "engine/stick": st((m.SX, m.SY, T + m.MOUTH + self.q)),
                    "rag": rag_st(self.rag), "puck": puck_st(self.puck), "card": st((*m.CARD_C, T + 0.005)),
                    "right_gripper": grip_st(self.right, across_y=self.across_y), "left_gripper": grip_st(self.left)}
            ok = m.success(objs)
            self.out = (m.score(objs), ok)
        return self.out

    # moves
    def grab(self):
        self.right, self.ropen = handle_at(self.q), HANDLE_GRIP
        return self.step(3)

    def slide(self, q1, n=20):
        q0 = self.q
        for k in range(1, n + 1):
            self.q = q0 + (q1 - q0) * k / n
            self.qd = (q1 - q0) / n * 20
            if self.ropen < 0.7:
                self.right = tuple(np.add(handle_at(self.q), self.off))
            self.step()
        self.qd = 0.0
        return self.step()

    def hold(self, n):
        return self.step(n)

    def take_rag(self):
        self.left, self.lopen = (RAG_HOME[0], RAG_HOME[1], RAG_HOME[2]), m.RAG[0] / 2 / abs(m.FINGER_OPEN)
        self.step(2)
        self.rag = (RAG_HOME[0], RAG_HOME[1], T + 0.15)
        self.left = self.rag
        return self.step(2)

    def wipe(self, z0, z1, n=30, dy=0.0225, off=1):
        for k in range(n + 1):
            self.rag = (m.SX, m.SY + dy, z0 + (z1 - z0) * k / n)
            self.left = self.rag
            self.step()
        self.rag = (m.SX, m.SY + 0.08, z1)  # rag lifted off the blade for `off` steps
        self.left = self.rag
        return self.step(off)

    def rag_down(self):
        self.rag = RAG_HOME
        self.left = RAG_HOME
        self.step(2)
        self.lopen = 1.0
        self.left = HOME["left_gripper"]
        return self.step(2)

    def report(self, k=None, release=True, at=None):
        """The left hand picks up the puck, sets it down in pocket k (default: the true level's) or at `at`, and lets
        go (or not)."""
        self.left, self.lopen = self.puck, PUCK_GRIP  # fingers closed on the puck (stopped by its 3 cm)
        self.step(2)
        dest = at if at is not None else in_pocket(k or self.case)
        for k_ in range(1, 11):
            a = k_ / 10
            self.puck = tuple(np.array(self.puck) * (1 - a) + np.array(dest) * a)
            self.left = self.puck
            self.step()
        self.step(3)
        if release:
            self.lopen = 1.0
            self.step(2)
            self.left = HOME["left_gripper"]
        return self.step(3)

    def let_go(self):
        self.ropen = 1.0
        self.step(2)
        self.right = HOME["right_gripper"]
        return self.step(3)


WT, WB = T + m.MOUTH + 0.055, T + m.MOUTH + 0.016  # a full wipe stroke (rag middle)


def full(t, reseat_q=0.0, read=30, final_q=0.0, put_rag=True, release=True, wipe=(WT, WB), dy=0.0225, report=True,
         pocket=None, keep_puck=False):
    t.grab()
    t.slide(OUTQ)
    t.take_rag()
    if wipe:
        t.wipe(*wipe, dy=dy)
    t.slide(reseat_q)
    t.hold(5)
    t.slide(OUTQ)
    t.hold(read)
    t.slide(final_q)
    if release:
        t.let_go()
    if put_rag:
        t.rag_down()
    if report:
        t.report(pocket, release=not keep_puck)
    return t.hold(25)


def check(name, got, want):
    ok = abs(got - want) < 1e-9
    print(f"{'ok ' if ok else 'BAD'} {name}: {got} (want {want})")
    return ok


results = []
t = Trial()
results.append(check("do nothing", t.hold(40)[0], 0.0))

t = Trial()
results.append(check("full procedure", full(t)[0], 1.0))
results.append(check("full procedure succeeds", float(t.out[1]), 1.0))

t = Trial()
full(t)
t.right = (0.20, -0.22, T + 0.18)  # done, the right arm parked away from home and moving: arms are not scored
t.left = (0.15, 0.10, T + 0.10)
results.append(check("done, arms not home", t.step(3)[0], 1.0))
results.append(check("done, arms not home still succeeds", float(t.out[1]), 1.0))
assert not hasattr(m, "REQUIRE_ARMS_HOME") and not hasattr(m, "_arms_home"), "arm-home rule should be gone"

t = Trial()  # single pull: pull, read, push back in (no wipe)
t.grab(); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.take_rag(); t.rag_down()
results.append(check("single pull, read, reinsert", t.hold(25)[0], 0.0))

t = Trial()
results.append(check("wipe skipped (pull, reseat, pull, read, reseat)", full(t, wipe=None)[0], 0.0))

t = Trial()
results.append(check("rag far from the blade", full(t, dy=0.06)[0], 0.0))

t = Trial()  # a rag passed 3 cm from the axis hovers about 8 mm clear of the blade's face: not a wipe
results.append(check("rag hovering 3 cm from the blade", full(t, dy=0.03)[0], 0.0))

t = Trial()
results.append(check("short wipe (3 cm of the 4 cm span)", full(t, wipe=(WB + 0.03, WB))[0], 0.0))

S0 = T + m.MOUTH + OUTQ - m.BLADE  # the tip's height when the test pulls the blade out (the rag's middle here: s = 0)


def strokes(t, *legs, off=10):
    """Pull, wipe with the given strokes (rag middle heights above the tip), each followed by `off` steps off the blade,
    then reseat, read, reseat, let go and put the rag down."""
    t.grab(); t.slide(OUTQ); t.take_rag()
    for z0, z1 in legs:
        t.wipe(S0 + z0, S0 + z1, off=off)
    t.slide(0.0); t.hold(5); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down(); t.report()
    return t.hold(25)[0]


t = Trial()  # the rag pressed on the blade and held there 1.5 s: touching is not wiping
results.append(check("dab (rag held still on the blade)", strokes(t, (0.03, 0.03)), 0.0))

t = Trial()  # several dabs up and down the blade, lifted off between them: no sliding, nothing wiped
results.append(check("dabs at 4, 2.5 and 0.5 cm, lifted between", strokes(t, (0.04, 0.04), (0.025, 0.025), (0.005, 0.005)),
                     0.0))

t = Trial()  # the middle slid from 4 cm to 2 cm: the tip and the crosshatch stay smeared
results.append(check("half the length (4 to 2 cm)", strokes(t, (0.04, 0.02)), 0.0))

t = Trial()  # scrubbed up and down the upper half ten times: the union is still half the span
results.append(check("half the length scrubbed back and forth", strokes(t, *[(0.04, 0.02), (0.02, 0.04)] * 5, off=0), 0.0))

t = Trial()  # review, Nov 10: a full stroke with the rigid pad held 4.5 mm clear of the blade (middle still in RAG_TOUCH)
results.append(check("full procedure, rag 4.5 mm clear of the blade", full(t, dy=0.026)[0], 0.0))

t = Trial()  # review: two dabs 4.2 cm apart with 3 steps off between: nothing credited across the gap
results.append(check("dab at 4.2 cm, 3 steps off, dab at the tip", strokes(t, (0.042, 0.042), (0.0, 0.0), off=3), 0.0))

t = Trial()  # review: one touching step at 4.2 cm, the next at the tip: a jump, not a slide
t.grab(); t.slide(OUTQ); t.take_rag()
t.wipe(S0 + 0.042, S0 + 0.0, n=1, off=10)  # two touching steps, 4.2 cm apart
t.slide(0.0); t.hold(5); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down()
results.append(check("one-step jump from 4.2 cm to the tip", t.hold(25)[0], 0.0))

t = Trial()  # one stroke, middle from 4.2 cm to the tip
results.append(check("full stroke (4.2 cm to the tip)", strokes(t, (0.042, 0.0)), 1.0))

t = Trial()  # the same stroke drawn upward
results.append(check("full stroke upward", strokes(t, (0.0, 0.042)), 1.0))

t = Trial()  # 3.4 cm of the 4 cm: 85%
results.append(check("stroke stopping 6 mm short of the tip", strokes(t, (0.04, 0.006)), 1.0))

t = Trial()  # two strokes, rag lifted off 0.5 s between, together covering the span (each alone is half): counted,
# because together they rub every part of the blade a single stroke would
results.append(check("two half strokes covering the span", strokes(t, (0.042, 0.02), (0.021, 0.0)), 1.0))

t = Trial()  # half a stroke, the tip dipped back into the tube (smeared again), out, the other half: not a wipe
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(S0 + 0.042, S0 + 0.02, off=10); t.slide(0.06); t.slide(OUTQ)
t.wipe(S0 + 0.021, S0, off=10); t.slide(0.0); t.hold(5); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down()
results.append(check("half stroke, tip back in the tube, other half", t.hold(25)[0], 0.0))

t = Trial()  # a full stroke with the contact lost 2 steps in the middle (the pad chattering): still one stroke
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(S0 + 0.042, S0 + 0.022, n=15, off=2); t.wipe(S0 + 0.02, S0, n=15, off=10)
t.slide(0.0); t.hold(5); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down(); t.report()
results.append(check("full stroke with a 2-step contact gap", t.hold(25)[0], 1.0))

t = Trial()  # rag touched at the top, lifted 0.5 s, touched at the bottom: the gap is not counted as slid over
results.append(check("touch at the top, lift, touch at the bottom", strokes(t, (0.042, 0.042), (0.0, 0.0)), 0.0))

t = Trial()
results.append(check("half-seated reinsert before the second pull (never seated)", full(t, reseat_q=0.045)[0], 0.0))

t = Trial()
results.append(check("second pull not held still 1 s", full(t, read=10, report=False)[0], 0.25))

t = Trial()
results.append(check("left out at the end (right report)", full(t, final_q=OUTQ)[0], 0.75))

t = Trial()
results.append(check("left half in at the end (right report)", full(t, final_q=0.04)[0], 0.75))

t = Trial()
results.append(check("still held at the end (right report)", full(t, release=False)[0], 0.75))

t = Trial()
results.append(check("rag still in hand at the end", full(t, put_rag=False)[0], 0.75))

t = Trial()  # the rag swiped by the other arm while the stick is held still: a real wipe, counts
results.append(check("rag drawn down the still blade by the other arm", full(t)[0], 1.0))

t = Trial()  # the wiped blade pushed half in and out again, then wiped again and done right: recovered
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(WT, WB); t.slide(0.04); t.slide(OUTQ)
t.wipe(WT, WB); t.slide(0.0); t.hold(5); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down(); t.report()
results.append(check("half-dip then re-wiped and done right", t.hold(25)[0], 1.0))

t = Trial()  # dropped back in after the wipe (not pushed by hand), then the rest
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(WT, WB); t.ropen = 1.0; t.right = HOME["right_gripper"]; t.slide(0.0)
t.grab(); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down()
results.append(check("dropped back in instead of pushed (never seated by hand)", t.hold(25)[0], 0.0))

t = Trial()  # after the wipe the dipstick is let fall back to its seat while an EMPTY closed fist (opening 0) hovers
# at the handle's seated height: not pushed in by hand, so no reseat; then the rest done right still scores 0
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(WT, WB)
t.ropen, t.right = 0.0, handle_at(0.0)  # fingers fully shut on nothing, beside where the handle will seat
t.slide(0.0, n=3); t.hold(5)
t.grab(); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down()
results.append(check("dropped in beside an empty fist (not a hand push)", t.hold(25)[0], 0.0))

t = Trial()  # the same with the fingers really closed on the handle (stopped by its 3.5 cm) is a push by hand
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(WT, WB); t.slide(0.0); t.hold(5)
t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down(); t.report()
results.append(check("pushed in with the handle gripped", t.hold(25)[0], 1.0))

t = Trial()  # the same push with the fingers across the handle's 3.5 cm width (the other way round) is also a grip
t.across_y = True
t.grab(); t.ropen = m.HANDLE[1] / 2 / abs(m.FINGER_OPEN); t.slide(OUTQ); t.take_rag(); t.wipe(WT, WB); t.slide(0.0)
t.hold(5); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down(); t.report()
results.append(check("pushed in gripped across the handle's width", t.hold(25)[0], 1.0))

# Holding is contact (Nov 12): both fingers on the handle, their opening stopped by it. A hand riding along beside the
# handle without both fingers on it is not pushing it home; the reseat does not count and nothing true is read.
t = Trial()  # 60 % open round the handle, each finger 1.9 cm off it (the old distance rule, fingers under 75 % open
# and the tool point 3 cm or less from the handle's middle, called this a grip)
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(WT, WB)
t.ropen = 0.6; t.slide(0.0, n=3); t.hold(5)
t.ropen = HANDLE_GRIP; t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down(); t.report()
results.append(check("reseated with an open hand round the handle, not touching it", t.hold(25)[0], 0.0))
t = Trial()  # closed to the handle's thickness but 5 mm to one side: one finger on it, the other 5 mm off
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(WT, WB)
t.off = (0.005, 0.0, 0.0); t.slide(0.0, n=3); t.hold(5)
t.off = (0.0, 0.0, 0.0); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down(); t.report()
results.append(check("reseated with one finger on the handle", t.hold(25)[0], 0.0))
t = Trial()  # the same hand closed to the handle's thickness with its pad tips 5 mm above the handle's top
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(WT, WB)
t.off = (0.0, 0.0, m.HANDLE[2] / 2 + 0.0128 + 0.005); t.slide(0.0, n=3); t.hold(5)
t.off = (0.0, 0.0, 0.0); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down(); t.report()
results.append(check("reseated by a closed hand hovering 5 mm over the handle", t.hold(25)[0], 0.0))

t = Trial()  # an empty fist at the handle cannot take the reading either (the handle is not held)
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(WT, WB); t.slide(0.0); t.hold(5); t.slide(OUTQ)
t.ropen = 0.0
results.append(check("second pull 'held' by an empty fist: no reading", t.hold(30)[0], 0.25))
t.ropen = HANDLE_GRIP; t.slide(0.0); t.let_go(); t.rag_down(); t.report()
results.append(check("then reported without a true reading (a guess)", t.hold(25)[0], 0.0))

t = Trial()  # wiping while the blade is still in the tube does nothing
t.take_rag(); t.wipe(T + m.MOUTH + 0.03, T + m.MOUTH + 0.005)
results.append(check("rag rubbed on the cap with the stick seated", t.hold(5)[0], 0.0))


# --- The report: the level varies per trial and the puck goes in the ADD, OK or OVER pocket
for case in m.CASES:
    t = Trial(case)
    results.append(check(f"{case}: full procedure, right pocket ({m.POCKET_NAME[case]})", full(t)[0], 1.0))
    results.append(check(f"{case}: succeeds", float(t.out[1]), 1.0))
    for wrong in m.CASES:
        if wrong != case:
            t = Trial(case)
            results.append(check(f"{case}: full procedure, wrong pocket {m.POCKET_NAME[wrong]}",
                                 full(t, pocket=wrong)[0], 0.0))
    t = Trial(case)
    results.append(check(f"{case}: no report (puck left on the table)", full(t, report=False)[0], 0.5))
    t = Trial(case)
    results.append(check(f"{case}: right report, rag in hand", full(t, put_rag=False)[0], 0.75))

t = Trial("low")  # the first pull's smear reads past FULL: a report taken from it is OVER, wrong here
t.grab(); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.take_rag(); t.rag_down(); t.report("over")
results.append(check("low: first pull read, OVER reported", t.hold(25)[0], 0.0))

t = Trial("ok")
results.append(check("puck in the right pocket, not let go", full(t, keep_puck=True)[0], 0.5))
results.append(check("puck not let go: no success", float(t.out[1]), 0.0))

t = Trial("ok")  # let go of the puck, the open hand left hovering 1 cm over it without touching: released (contact,
# not nearness, is what counts)
full(t, keep_puck=True)
t.lopen = 1.0
t.left = (t.puck[0], t.puck[1], t.puck[2] + m.PUCK_H / 2 + 0.01)
results.append(check("puck let go, open hand hovering 1 cm over it", t.hold(5)[0], 1.0))
t.lopen = PUCK_GRIP
t.left = t.puck
results.append(check("then fingers closed on it again at the end", t.hold(5)[0], 0.5))

t = Trial("ok")  # the puck set on the wall between the ADD and OK pockets, touching both: no pocket is marked
full(t, report=False)
t.report(at=(m.CARD_C[0], m.CARD_C[1] + m.POCKET_DY / 2, T + m.CARD[2] + m.PUCK_H / 2))
results.append(check("puck straddling ADD and OK (on the wall)", t.hold(25)[0], 0.5))

t = Trial("ok")  # set on the rim of the right pocket, not down in it
full(t, report=False)
t.report(at=(m.CARD_C[0] + m.POCKET / 2, m.CARD_C[1], T + m.CARD[2] + m.PUCK_H / 2))
results.append(check("puck on the right pocket's rim, not in it", t.hold(25)[0], 0.5))

t = Trial("ok")  # first put in OVER, then moved to OK, both after the reading: the pocket it ends in is the report
full(t, pocket="over")
t.report("ok")
results.append(check("wrong pocket after the reading, then moved to the right one", t.hold(25)[0], 1.0))

t = Trial("ok")  # right pocket, then moved to a wrong one: the end state counts
full(t)
t.report("low")
results.append(check("right pocket, then moved to ADD", t.hold(25)[0], 0.0))

t = Trial("ok")  # the puck put in the right pocket before the second pull, everything else done right: a guess
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(WT, WB); t.slide(0.0); t.hold(5)
t.report()
t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down()
results.append(check("reported (right pocket) before the second pull", t.hold(25)[0], 0.0))

t = Trial("ok")  # dipped into a pocket before the reading and taken out again, then reported right: still a guess
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(WT, WB); t.slide(0.0); t.hold(5)
t.report("ok", release=False); t.report(at=m.PUCK_START)
t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down(); t.report()
results.append(check("puck dipped in a pocket before the reading, then reported right", t.hold(25)[0], 0.0))

t = Trial("ok")  # the puck popping into a pocket for a single step before the reading (a physics pop) is not a report
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(WT, WB); t.slide(0.0); t.hold(5)
t.puck = in_pocket("ok"); t.step(1); t.puck = m.PUCK_START; t.step(2)
t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down(); t.report()
results.append(check("one-step pop into a pocket before the reading, then done right", t.hold(25)[0], 1.0))

t = Trial("ok")  # reported right with the dipstick never wiped: the reading is not true, so it is a guess
results.append(check("right pocket without a wipe", full(t, wipe=None)[0], 0.0))

t = Trial("over")  # a wrong report with the dipstick left out
results.append(check("wrong report, dipstick left out", full(t, final_q=OUTQ, pocket="ok")[0], 0.0))

# Guessing one pocket every trial, with the procedure done right, averages 1/3: below the 0.5 for reading and not
# reporting.
for g in m.CASES:
    mean = np.mean([full(Trial(c), pocket=g)[0] for c in m.CASES])
    results.append(check(f"always guessing {m.POCKET_NAME[g]}: mean over the three levels", round(mean, 4),
                         round(1 / 3, 4)))

# The trial's level: OIL_CASE sets it; otherwise the rag's jitter picks it, in place = between the marks.
os.environ.pop("OIL_CASE", None)
results.append(check("no jitter: between the marks", float(m._trial_case(m.RAG_START[:2]) == "ok"), 1.0))
rng = np.random.default_rng(0)
got = [m._trial_case(np.array(m.RAG_START[:2]) + rng.uniform(-0.03, 0.03, 2)) for _ in range(3000)]
share = [got.count(c) / len(got) for c in m.CASES]
results.append(check("seeded jitter spreads the three levels (each 28-39%)", float(all(0.28 < x < 0.39 for x in share)),
                     1.0))
for c in m.CASES:
    os.environ["OIL_CASE"] = c
    labels = [w.label for w in m.demo()["left"]]
    results.append(check(f"demo told {c}: aims for the {m.POCKET_NAME[c]} pocket",
                         float(f"over the {m.POCKET_NAME[c]} pocket" in labels), 1.0))
t = Trial("over")  # the drawn band follows the case
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(WT, WB); t.slide(0.0); t.hold(5)
results.append(check("over: the drawn band reaches 2.7 cm", m._oil_segments()[0][1], 0.027))
for c, lv in m.LEVELS.items():  # each level 4 mm or more from both marks
    results.append(check(f"{c} level clear of the marks", float(min(abs(lv - m.ADD_MARK), abs(lv - m.FULL_MARK)) >= 0.004 - 1e-9), 1.0))
os.environ.pop("OIL_CASE", None)

full(Trial())  # reset: a new trial after a perfect one starts from 0
t = Trial()
results.append(check("history reset after a full trial", t.hold(10)[0], 0.0))

print("all pass" if all(results) else "FAILURES")
