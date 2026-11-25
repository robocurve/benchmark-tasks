"""dev/torque_table.py RUN.npz [RUN2.npz ...]: per arm and joint, the peak |torque| and the highest 1 s and 10 s
moving averages, against the rated torque, from runs saved with CLP_TORQUE_LOG (Markdown). With several runs, the
highest over all of them. Also lists where any peak passed 0.9 of rated."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import geometry as g  # noqa: E402

R = np.array(g.RATED)
ARMS = (("a_left", "A left"), ("a_right", "A right"), ("b_left", "B left"), ("b_right", "B right"))
res = {}
for path in sys.argv[1:]:
    d = np.load(path)
    for key, _ in ARMS:
        m, x = d[f"{key}_mean"], d[f"{key}_max"]
        c = np.cumsum(np.vstack([np.zeros(6), m]), axis=0)
        a1 = ((c[20:] - c[:-20]) / 20).max(axis=0)
        a10 = ((c[200:] - c[:-200]) / 200).max(axis=0)
        r = res.setdefault(key, {"peak": np.zeros(6), "avg1": np.zeros(6), "avg10": np.zeros(6)})
        r["peak"] = np.maximum(r["peak"], x.max(axis=0))
        r["avg1"] = np.maximum(r["avg1"], a1)
        r["avg10"] = np.maximum(r["avg10"], a10)
        hi = np.nonzero((x / R).max(axis=1) > 0.9)[0]
        if len(hi):
            print(f"<!-- {os.path.basename(path)} {key}: peaks over 0.9 of rated at steps {(hi + 1).tolist()[:20]} -->")
print("| Arm | measure | " + " | ".join(f"J{i}" for i in range(1, 7)) + " |")
print("|---|---|" + "---|" * 6)
print("| all | rated (continuous) | " + " | ".join(f"{v:g}" for v in g.RATED) + " |")
print("| all | peak rating | " + " | ".join(f"{v:g}" for v in g.PEAK) + " |")
worst = {}
for key, name in ARMS:
    for m, label in (("peak", "peak"), ("avg1", "highest 1 s average"), ("avg10", "highest 10 s average")):
        v = res[key][m]
        print(f"| {name} | {label} | " + " | ".join(f"{a:.2f} ({a / r:.2f})" for a, r in zip(v, R)) + " |")
        k = float((v / R).max())
        if k > worst.get(m, (0.0, ""))[0]:
            worst[m] = (k, f"{name} J{int((v / R).argmax()) + 1}")
print()
for m, (k, where) in worst.items():
    print(f"- highest {m} / rated: {k:.2f} ({where})")
