"""Offline checks of the coin-changer scorer: coin and tray poses through tally() and grade(), no simulator.

    python -B tasks/coin-changer/test_task.py      (prints "all pass")
"""

import importlib.util
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
_spec = importlib.util.spec_from_file_location("coin_task", os.path.join(HERE, "task.py"))
t = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(t)
m = t.m

fails = 0


def check(cond, what):
    global fails
    print(("ok  " if cond else "FAIL") + " " + what)
    fails += not cond


def tray_state(pos, quat=(0, 0, 0, 1)):
    p = np.array(pos, float)
    return m.PartState(pos=p, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p - 0.01, high=p + 0.01)


TRAY = tray_state((0.395, 0.0, t.TOP + t.TRAY_HOLD_Z))
in_tray_pt = np.array([0.395, 0.0, t.TOP + t.TRAY_HOLD_Z + t.TRAY_FLOOR + 0.001])
in_changer_pt = np.array([t.BX, 0.0, t.TOP + t.STAND + t.BASE_T + 0.001])
table_pt = np.array([0.35, 0.0, t.TOP + 0.001])


def coins(spec):
    """spec: per barrel, how many of its 8 coins are 'tray' / 'table'; the rest are in the changer."""
    pos = []
    for k, (cname, *_rest) in enumerate(t.COINS):
        n_tray, n_table = spec.get(k, (0, 0))
        for i in range(t.COINS[k][6]):
            pos.append(in_tray_pt if i < n_tray else table_pt if i < n_tray + n_table else in_changer_pt)
    return pos


def score(spec, tilt=0.0):
    return t.grade(t.tally(TRAY, coins(spec)), tilt)


check(score({}) == 0.0, "nothing vended scores 0")
check(score({0: (5, 0), 1: (1, 0)}) == 1.0, "5 quarters + 1 dime in the tray scores 1.0 (= $1.35)")
check(score({0: (5, 0), 1: (1, 0)}, tilt=25.0) == 0.0, "tray tilted over 20 degrees scores 0")
check(score({0: (5, 0), 1: (1, 1)}) == 0.5, "one more dime on the table: 0.5")
check(score({0: (6, 0), 1: (1, 0)}) == 0.5, "a sixth quarter in the tray: 0.5")
check(score({0: (5, 0), 1: (1, 0), 3: (1, 0)}) == 0.5, "a penny in the tray as well: 0.5")
check(score({0: (4, 0), 1: (1, 0)}) == 0.25, "only 4 quarters: 0.25")
check(score({0: (0, 6), 1: (0, 1)}) == 0.0, "no tray: all coins on the table scores 0")
check(score({0: (3, 2)}) == 0.25, "3 quarters in the tray, 2 on the table: 0.25")
tray_tilted = t.tally(tray_state((0.395, 0.0, t.TOP + t.TRAY_HOLD_Z), (0.2588, 0, 0, 0.9659)), coins({}))
check(sum(tray_tilted["tray"]) == 0, "tray tilt does not invent coins")
vals = [c[1:] for c in t.COINS]
check(abs(sum(c[7] * v for c, v in zip(t.COINS, (0.25, 0.10, 0.05, 0.01))) - 1.35) < 1e-9, "the target is worth exactly $1.35")
check(abs(t.PITCH - 0.040) < 1e-9 and len(t.COINS) == 4, "four barrels at 40 mm pitch")
for k, c in enumerate(t.COINS):
    check(c[6] >= c[7], f"{c[0]}: enough coins loaded for the target")
print("all pass" if not fails else f"{fails} FAILED")
sys.exit(1 if fails else 0)
