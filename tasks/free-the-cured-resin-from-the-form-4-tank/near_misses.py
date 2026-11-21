"""Near-successes and unintended strategies, scored offline by the offline checker.

Each case: (label, {object: state}, expected score, expected success). Offline there is no robot, so before the
cases this file writes into the task's per-trial record what the step callback records for a trial in which the raft
came free, no zero rule fired, the hands ended 50 mm from the scraper and everything has been still for 1 s. The
cases then vary where the raft and the two scrapers end up. _run() afterwards drives the step-side function observe()
directly, step by step, to check every whole-trial zero rule (a hand on the raft or the film, the metal scraper in the
tank, a corner dig, a steep stab), the freed record, the 30 mm let-go distance and the 1 s holds, and raises if
anything scores wrongly.
"""

import math

import numpy as np

import task
from task import (FILM_Z, PLASTIC, METAL, RAFT, RAFT_AT, RAFT_H, RAFT_X, RAFT_Y, RIM_Z, TOP, TR_H, TRAY_C, TRAY_IN,
                  TX0, TX1, TY0, TY1)

CLEAR = 0.05
START = {o.name: o.state() for o in task.OBJECTS}


def raft(x, y, z, quat=(0.0, 0.0, 0.0, 1.0)):
    return RAFT.state(pos=(x, y, z), quat_xyzw=quat)


def tool_state(part, edge, pitch, roll=0.0, yaw=0.0):
    r = task.tool_rotation(pitch)
    if roll:
        c, s = math.cos(roll), math.sin(roll)
        r = r @ np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    if yaw:
        c, s = math.cos(yaw), math.sin(yaw)
        r = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ r
    import parts

    return part.state(pos=tuple(float(v) for v in edge), quat_xyzw=tuple(parts._matrix_to_quat(r)))


def scene(**kw):
    out = dict(START)
    out.update(kw)
    return out


def q_about(axis, deg):
    a = math.radians(deg) / 2
    v = np.asarray(axis, float) / np.linalg.norm(axis)
    return (*(v * math.sin(a)), math.cos(a))


IN_TRAY = raft(TRAY_C[0] + 0.03, TRAY_C[1], TOP + 0.003)
ON_DECK = raft(task.PX0 + 0.015, 0.0, TOP + task.DECK)  # on the printer's deck in front of the tank
ON_BENCH = raft(0.20, -0.20, TOP)
IN_TANK_FREE = raft(RAFT_AT[0] + 0.012, RAFT_AT[1], FILM_Z)
HOME = START[PLASTIC.name]
EDGE0 = np.array([*task.PLASTIC_EDGE, TOP + task.EDGE_UP])


def preset(freed=True):
    task.reset_record()
    task._rec.update(steps=1200, t=60.0, still=58.0, raft_still=58.0, free=58.0, gap=CLEAR, freed=freed)


preset()
CASES = [
    ("raft in the waste tray, scraper back in its rest, hands 50 mm clear, still 1 s", scene(cured_resin=IN_TRAY),
     1.0, True),
    ("raft in the tray against its right wall", scene(cured_resin=raft(TRAY_C[0], TRAY_IN[2] + RAFT_Y / 2, TOP + 0.003)),
     1.0, True),
    ("raft in the tray leaning on its wall", scene(cured_resin=raft(TRAY_IN[1] - 0.020, TRAY_C[1], TOP + 0.01,
                                                                     q_about((0, 1, 0), 40))), 1.0, True),
    ("raft's middle 1 mm outside the tray's wall (resting on its rim)",
     scene(cured_resin=raft(TRAY_IN[1] + 0.001, TRAY_C[1], TOP + TR_H - RAFT_H / 2 - 0.004)), 0.5, False),
    ("raft in the tray, scraper put back 12 mm from its place",
     scene(cured_resin=IN_TRAY, plastic_scraper=tool_state(PLASTIC, EDGE0 + [0.0, 0.012, 0.0], task.REST_PITCH)),
     1.0, True),
    ("raft in the tray, scraper put back 18 mm from its place",
     scene(cured_resin=IN_TRAY, plastic_scraper=tool_state(PLASTIC, EDGE0 + [0.0, 0.018, 0.0], task.REST_PITCH)),
     0.75, False),
    ("raft in the tray, scraper in its rest turned 15 degrees",
     scene(cured_resin=IN_TRAY, plastic_scraper=tool_state(PLASTIC, EDGE0, task.REST_PITCH, yaw=math.radians(15))),
     0.75, False),
    ("raft in the tray, scraper left lying on the printer's deck",
     scene(cured_resin=IN_TRAY, plastic_scraper=tool_state(PLASTIC, (0.25, -0.05, TOP + task.DECK), 0.0)), 0.75,
     False),
    ("raft in the tray on the scraper's blade, the scraper let go in the tray",
     scene(cured_resin=raft(TRAY_C[0], TRAY_C[1], TOP + 0.003 + task.PS_T),
           plastic_scraper=tool_state(PLASTIC, (TRAY_C[0] + 0.03, TRAY_C[1], TOP + 0.003), 0.0)), 0.75, False),
    ("raft dumped on the printer's deck", scene(cured_resin=ON_DECK), 0.5, False),
    ("raft dropped on the bench", scene(cured_resin=ON_BENCH), 0.5, False),
    ("raft freed but left in the tank", scene(cured_resin=IN_TANK_FREE), 0.25, False),
    ("raft freed, tipped up against the tank's back wall",
     scene(cured_resin=raft(TX1 - 0.010, RAFT_AT[1], FILM_Z + 0.02, q_about((0, 1, 0), -70))), 0.25, False),
    ("raft on the blade of a scraper let go lying across the tank's walls, 3 cm above the rim",
     scene(cured_resin=raft(RAFT_AT[0], RAFT_AT[1], RIM_Z + 0.03 + task.PS_T),
           plastic_scraper=tool_state(PLASTIC, (RAFT_AT[0] + 0.02, RAFT_AT[1], RIM_Z + 0.03), 0.0)), 0.5, False),
]

# -- step-by-step trials ------------------------------------------------------------------------------------------------
HZ = 20.0
FAR = np.array([[0.30, -0.30, TOP + 0.30]])  # hand points well away from everything


def near(target, dz=0.0):
    return np.array([[target[0], target[1], target[2] + dz]])


class Trial:
    def __init__(self):
        task.reset_record()
        self.t = 0.0
        self.s = dict(START)
        self.step(n=2)

    def step(self, hands=FAR, n=1):
        for _ in range(n):
            self.t += 1.0 / HZ
            task.observe(self.s, hands, self.t)

    def scrape(self, roll=0.0, pitch=25.0, tool=PLASTIC, hands=FAR):
        """The edge goes along the film into the raft; the raft comes free and is pushed 12 mm to the back wall."""
        x0 = RAFT_AT[0] - RAFT_X / 2 - 0.010
        for k in range(11):
            x = x0 + 0.045 * k / 10
            self.s[tool.name] = tool_state(tool, (x, RAFT_AT[1], FILM_Z), math.radians(pitch), math.radians(roll))
            if k >= 4:
                self.s[RAFT.name] = raft(RAFT_AT[0] + 0.012 * (k - 4) / 6, RAFT_AT[1], FILM_Z + 0.001 * (k > 4),
                                         q_about((0, 1, 0), -4.0 * (k > 4)))
            self.step(hands)

    def lift_out(self, to=None, hands=FAR):
        """The raft carried out on the blade and set down in the tray (or ``to``)."""
        end = IN_TRAY if to is None else to
        for k in range(1, 9):
            p = np.array([RAFT_AT[0], RAFT_AT[1], FILM_Z + 0.06]) * (1 - k / 8) + np.asarray(end.pos) * (k / 8)
            self.s[RAFT.name] = raft(*p)
            self.s[PLASTIC.name] = tool_state(PLASTIC, p + [0.03, 0.0, -0.003], math.radians(8))
            self.step(hands)
        self.s[RAFT.name] = end
        self.s[PLASTIC.name] = tool_state(PLASTIC, np.asarray(end.pos) + [-0.05, 0.0, 0.04], math.radians(40))
        self.step(hands, n=3)

    def finish(self, home=True, hands=None, still_s=1.2):
        self.s[PLASTIC.name] = HOME if home else tool_state(PLASTIC, (0.25, -0.05, TOP + task.DECK), 0.0)
        self.step(near(np.asarray(HOME.pos), 0.10), n=4)
        self.step(near(np.asarray(HOME.pos), 0.10) if hands is None else hands, n=int(round(still_s * HZ)))

    def score(self):
        return task.score(self.s)


def expect(label, got, want):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"near_misses step test '{label}': score {got}, expected {want}; record {task._rec}")


def _run():
    tr = Trial()
    expect("nothing done", tr.score(), 0.0)
    tr.scrape()
    expect("raft freed, still in the tank", tr.score(), 0.25)
    tr.lift_out()
    tr.finish()
    expect("raft scraped free, carried out on the blade into the tray, scraper back, hands 10 cm clear", tr.score(),
           1.0)

    tr = Trial()
    tr.scrape()
    tr.lift_out()
    tr.finish(hands=near(np.asarray(HOME.pos), 0.025))
    expect("as above but a hand stays 25 mm from the scraper", tr.score(), 0.75)

    tr = Trial()
    tr.scrape()
    tr.lift_out()
    tr.finish(still_s=0.8)
    expect("hands clear only 0.8 s before the end", tr.score(), 0.75)

    tr = Trial()
    tr.scrape()
    tr.lift_out(to=ON_DECK)
    tr.finish()
    expect("raft left on the printer's deck, scraper back", tr.score(), 0.5)

    tr = Trial()
    tr.scrape(roll=12.0)
    tr.lift_out()
    tr.finish()
    expect("edge tilted 12 degrees with a corner on the film (corner dig)", tr.score(), 0.0)

    tr = Trial()
    tr.scrape(roll=9.0)
    tr.lift_out()
    tr.finish()
    expect("edge tilted 9 degrees", tr.score(), 1.0)

    tr = Trial()
    tr.scrape(pitch=55.0)
    tr.lift_out()
    tr.finish()
    expect("blade at 55 degrees with its edge on the film (stab)", tr.score(), 0.0)

    tr = Trial()
    tr.scrape(pitch=45.0)
    tr.lift_out()
    tr.finish()
    expect("blade at 45 degrees", tr.score(), 1.0)

    tr = Trial()
    tr.s[METAL.name] = tool_state(METAL, (RAFT_AT[0] - 0.04, RAFT_AT[1], FILM_Z + 0.02), math.radians(30))
    tr.step()
    tr.s[METAL.name] = START[METAL.name]
    tr.scrape()
    tr.lift_out()
    tr.finish()
    expect("the metal scraper dipped into the tank once, then the plastic one used", tr.score(), 0.0)

    tr = Trial()
    tr.s[METAL.name] = tool_state(METAL, (RAFT_AT[0] - 0.04, RAFT_AT[1], RIM_Z + 0.005), math.radians(10))
    tr.step()
    tr.s[METAL.name] = START[METAL.name]
    tr.scrape()
    tr.lift_out()
    tr.finish()
    expect("the metal scraper held over the tank, above its rim", tr.score(), 1.0)

    tr = Trial()
    tr.scrape()
    tr.lift_out()
    tr.s[RAFT.name] = raft(TRAY_C[0], TRAY_C[1], TOP + 0.05 + task.PS_T)
    tr.s[PLASTIC.name] = tool_state(PLASTIC, (TRAY_C[0] + 0.03, TRAY_C[1], TOP + 0.05), 0.0)
    tr.step(hands=near(np.asarray(tr.s[PLASTIC.name].pos) + [-0.10, 0.0, 0.03]), n=30)
    expect("raft held on the blade over the tray at the end, scraper in hand", tr.score(), 0.25)

    tr = Trial()
    tr.scrape(pitch=70.0, roll=8.0)
    tr.lift_out()
    tr.finish()
    expect("blade at 70 degrees, edge tilted 8 degrees, one corner on the film (steep chop)", tr.score(), 0.0)

    tr = Trial()
    tr.scrape()
    tr.s[RAFT.name] = raft(RAFT_AT[0], RAFT_AT[1], RIM_Z + 0.03)
    tr.step(hands=near((RAFT_AT[0], RAFT_AT[1], RIM_Z + 0.03 + RAFT_H), 0.001))
    tr.lift_out()
    tr.finish()
    expect("a fingertip touches the raft once it is above the tank (out of the well)", tr.score(), 1.0)

    tr = Trial()
    tr.scrape()
    tr.step(hands=near((RAFT_AT[0], RAFT_AT[1], FILM_Z + RAFT_H), 0.0015))
    tr.lift_out()
    tr.finish()
    expect("a fingertip 1.5 mm from the raft's top", tr.score(), 0.0)

    tr = Trial()
    tr.step(hands=near(((TX0 + TX1) / 2, (TY0 + TY1) / 2, FILM_Z), 0.0015))
    tr.scrape()
    tr.lift_out()
    tr.finish()
    expect("a fingertip 1.5 mm above the film", tr.score(), 0.0)

    tr = Trial()
    tr.step(hands=near(((TX0 + TX1) / 2, (TY0 + TY1) / 2, FILM_Z), 0.0025))
    tr.scrape()
    tr.lift_out()
    tr.finish()
    expect("a fingertip 2.5 mm above the film", tr.score(), 1.0)

    tr = Trial()
    tr.scrape()
    tr.lift_out()
    tr.finish(home=False)
    expect("raft in the tray, scraper left on the printer", tr.score(), 0.75)

    tr = Trial()
    tr.scrape()
    tr.lift_out()
    tr.step(hands=None)
    tr.finish()
    expect("the arms could not be read at one step", tr.score(), 0.0)

    tr = Trial()
    tr.s[RAFT.name] = raft(RAFT_AT[0], RAFT_AT[1], FILM_Z + 0.0005)
    tr.step(n=10)
    tr.finish()
    expect("raft moved 0.5 mm (still stuck)", tr.score(), 0.0)


_run()
preset()  # the cases above are scored with this record (check.py scores them after importing this file)
