"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py).

Each case: (label, {object: ready Part state}, expected score, expected success). Cases run in order against one copy
of the task module, so the whole-trial "touched the counter" record carries from one case to the next until a case
puts everything back at its start (a reset).
"""

import math

import numpy as np

from task import CT, D0, D1, MOUTH, PEEL, PIZZA, R, STONE_HALF_Y, YC

BLADE_LEN = 0.110  # peel origin (blade rear) to blade tip


def quat(axis, deg):
    a = math.radians(deg) / 2
    v = np.asarray(axis, float) / np.linalg.norm(axis) * math.sin(a)
    return (*v, math.cos(a))


def pizza(x=0.810, y=YC, low=CT, q=None, vel=None):
    """The pizza with its centre over (x, y) and its lowest point at ``low``."""
    st = PIZZA.state(pos=(x, y, CT), quat_xyzw=q, lin_vel=vel)
    return PIZZA.state(pos=(x, y, CT + low - st.low[2]), quat_xyzw=q, lin_vel=vel)


def peel(tip_x=0.700, y=YC, z=CT, q=None):
    return PEEL.state(pos=(tip_x - BLADE_LEN, y, z), quat_xyzw=q)


OUT = peel()  # peel back on the counter, tip 40 mm short of the mouth
START = {"peel": PEEL.state(), "pizza": PIZZA.state()}

CASES = [
    ("pizza flat on the stone, peel back on the counter", {"pizza": pizza(), "peel": OUT}, 1.0, True),
    ("pizza's near edge exactly on the stone's front edge", {"pizza": pizza(x=D0 + R), "peel": OUT}, 1.0, True),
    ("pizza's far edge exactly on the stone's back edge", {"pizza": pizza(x=D1 - R), "peel": OUT}, 1.0, True),
    ("pizza 1 mm over the stone's back edge (against the back wall)", {"pizza": pizza(x=D1 - R + 0.001),
                                                                        "peel": OUT}, 0.0, False),
    ("pizza's side exactly on the stone's left edge", {"pizza": pizza(y=YC + STONE_HALF_Y - R), "peel": OUT},
     1.0, True),
    ("pizza 1 mm over the stone's right edge", {"pizza": pizza(y=YC - STONE_HALF_Y + R - 0.001), "peel": OUT},
     0.0, False),
    ("pizza tilted 5 deg, rim on the stone", {"pizza": pizza(q=quat((0, 1, 0), 5)), "peel": OUT}, 1.0, True),
    ("pizza tilted 6 deg, rim on the stone", {"pizza": pizza(q=quat((0, 1, 0), 6)), "peel": OUT}, 0.0, False),
    ("pizza upside down on the stone", {"pizza": pizza(q=quat((1, 0, 0), 180)), "peel": OUT}, 0.0, False),
    ("pizza on edge against the back wall", {"pizza": pizza(x=D1 - 0.006, low=CT, q=quat((0, 1, 0), 90)),
                                             "peel": OUT}, 0.0, False),
    ("pizza on the stone, still sliding", {"pizza": pizza(vel=(-0.05, 0.0, 0.0)), "peel": OUT}, 0.0, False),
    ("pizza lowest point 2 mm above the stone", {"pizza": pizza(low=CT + 0.002), "peel": OUT}, 1.0, True),
    ("pizza lowest point 2.5 mm above the stone (held or still on something)",
     {"pizza": pizza(low=CT + 0.0025), "peel": OUT}, 0.0, False),
    ("pizza on the stone, peel tip exactly at the mouth line", {"pizza": pizza(), "peel": peel(tip_x=MOUTH)},
     1.0, True),
    ("pizza on the stone, peel tip 1 mm past the mouth line", {"pizza": pizza(), "peel": peel(tip_x=MOUTH + 0.001)},
     0.0, False),
    ("pizza on the stone, peel left in the oven under nothing", {"pizza": pizza(x=0.840), "peel": peel(tip_x=0.880,
                                                                                                 y=YC - 0.05)},
     0.0, False),
    ("pizza on the stone, peel held in the air outside the oven",
     {"pizza": pizza(), "peel": peel(tip_x=0.650, z=CT + 0.060)}, 1.0, True),
    ("pizza on the stone, peel held up, handle end raised 20 deg, tip short of the mouth",
     {"pizza": pizza(), "peel": peel(tip_x=0.640, z=CT + 0.045, q=quat((0, 1, 0), 20))}, 1.0, True),
    ("pizza on the stone, peel knocked off onto the table",
     {"pizza": pizza(), "peel": PEEL.state(pos=(0.500, YC + 0.300, CT - 0.040))}, 0.0, False),
    ("pizza on the stone, peel standing on its handle end against the counter",
     {"pizza": pizza(), "peel": PEEL.state(pos=(0.120, YC, CT - 0.040 + 0.420), quat_xyzw=quat((0, 1, 0), -90))},
     0.0, False),
    ("pizza on the stone, peel's lowest point exactly 5 mm below the counter top",
     {"pizza": pizza(), "peel": peel(z=CT - 0.005)}, 1.0, True),
    ("pizza on the stone, peel's lowest point 6 mm below the counter top",
     {"pizza": pizza(), "peel": peel(z=CT - 0.006)}, 0.0, False),
    ("pizza still on the peel, both in the oven", {"pizza": pizza(x=0.830, low=CT + 0.004),
                                                   "peel": peel(tip_x=0.885)}, 0.0, False),
    ("pizza brought back out on the peel (slow pull)", {"pizza": pizza(x=0.645, low=CT + 0.004),
                                                         "peel": peel(tip_x=0.700)}, 0.0, False),
    ("pizza half on the stone, half over the counter", {"pizza": pizza(x=D0), "peel": OUT}, 0.0, False),
    ("then pushed fully onto the stone (the counter touch is remembered)", {"pizza": pizza(), "peel": OUT},
     0.0, False),
    ("reset: everything at its start", START, 0.0, False),
    ("after the reset: pizza on the stone, peel out", {"pizza": pizza(), "peel": OUT}, 1.0, True),
    ("pizza dumped on the counter beside the peel", {"pizza": pizza(x=0.500, y=YC + 0.100)}, 0.0, False),
    ("then put on the stone", {"pizza": pizza(), "peel": OUT}, 0.0, False),
    ("reset again", START, 0.0, False),
    ("pizza fallen onto the table", {"pizza": pizza(x=0.500, y=YC + 0.250, low=CT - 0.040)}, 0.0, False),
    ("then on the stone", {"pizza": pizza(), "peel": OUT}, 0.0, False),
    ("reset once more", START, 0.0, False),
    ("pizza on the stone at the trial's last step", {"pizza": pizza(), "peel": OUT}, 1.0, True),
]

if __name__ == "__main__":
    import task

    bad = 0
    for label, changes, want, want_ok in CASES:
        st = {**{o.name: o.state() for o in task.OBJECTS}, **changes}
        got_ok = task.success(st)
        got = 1.0 if got_ok else task.score(st)
        flag = "ok  " if (got == want and got_ok == want_ok) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:.2f} {label}")
    print("all ok" if not bad else f"{bad} failing")
