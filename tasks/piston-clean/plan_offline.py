"""Szybki podgląd planu demo bez symulatora: pokrycie komórek dla zaplanowanych pozycji gąbki (zakładając
dokładne śledzenie) i osiągalność IK. Narzędzie do strojenia, nie część zadania.

Uruchomienie (środowisko Areny): python plan_offline.py piston_clean.py
"""

import math
import sys

import numpy as np
from yam_arena import demo as yd
from yam_arena import task_file

task = task_file.load(sys.argv[1])
m = task.module
ps = m.ps
plans = task.demo()


def yaw_quat(a):
    return np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])


# wycieranie: etykiety "wipe"/"hold" lewej ręki; faza (yaw prawej) z etykiet "turn N"
led = ps.Ledger()
phase = 0
for w in plans["right"]:
    pass
t_right = []  # (czas końca waypointu, yaw fazy)
t = 0.0
cur = 0
for w in plans["right"]:
    t += w.duration
    if w.label.startswith("turn"):
        cur = int(w.label.split()[1])
    t_right.append((t, cur))


def phase_at(time):
    for end, ph in t_right:
        if time <= end:
            return ph
    return t_right[-1][1]


t = 0.0
for w in plans["left"]:
    t_mid = t + w.duration / 2
    t += w.duration
    if w.label not in ("wipe", "hold", "dip", "rinse", "wipe inside", "press crown"):
        continue
    ph = phase_at(t_mid)
    centre, rot = m.hang(ph)
    stick_dir = w.yaw
    tcp = np.asarray(w.pos)
    pad = tcp + np.array([m.L_PAD * math.cos(stick_dir), m.L_PAD * math.sin(stick_dir), -m.STICK_DZ])
    pad[2] -= getattr(m, "SAG_Z", 0.0)  # kijek opada
    q_piston = m._piston_quat_for_grid(yaw_quat(rot))
    lo = np.asarray(centre) - 0.04
    before = led.dirty.copy()
    led.step(np.asarray(centre), q_piston, np.zeros(3), lo, lo + 0.08, pad, yaw_quat(stick_dir),
             np.array([*m.TRAY_CENTER, 0.0]), m.TRAY_FLOOR_Z)
    if "-v" in sys.argv and w.label == "hold":
        new = before & ~led.dirty
        print(f"  t={t_mid:5.1f} phase {ph:4d} stick {math.degrees(stick_dir):6.0f} new {new.sum():3d} load {led.load}")
d = led.dirty
g1, g2 = ps.N_SKIRT, ps.N_SKIRT + ps.N_CROWN
print(f"clean {led.clean_fraction:.2f}  dirty skirt {d[:g1].sum()}/{g1}  crown {d[g1:g2].sum()}/{ps.N_CROWN}  "
      f"inner {d[g2:].sum()}/{ps.N_INNER}  rinses {led.rinses}")
az = np.degrees(np.arctan2(ps.CELLS_LOCAL[:g1, 1], ps.CELLS_LOCAL[:g1, 0])).round()
print("dirty skirt azimuths (piston frame):", sorted(set(az[d[:g1]].astype(int).tolist())))

for arm, base in (("left", yd.LEFT_BASE_POS), ("right", yd.RIGHT_BASE_POS)):
    plan = yd.ArmPlan(plans[arm], base)
    bad, tt = {}, 0.0
    while tt < plan.duration:
        plan.worst_ik_mm = 0.0
        _, label = plan.action7(tt)
        if plan.worst_ik_mm > 5:
            bad[label] = max(bad.get(label, 0), plan.worst_ik_mm)
        tt += 0.05
    print(f"{arm}: {plan.duration:.1f} s, IK > 5 mm: " + ", ".join(f"{k} {v:.0f}" for k, v in list(bad.items())[:8]))
