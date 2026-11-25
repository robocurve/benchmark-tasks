"""dev/results.py RUN: a run's score, events, end state, and per-arm joint torques (peak and highest 1 s and 10 s
moving averages of |torque|, against the rated torque) from dev/RUN.log (Markdown)."""
import json
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import geometry as g  # noqa: E402

D = os.path.dirname(os.path.abspath(__file__))
run = sys.argv[1]
text = open(f"{D}/{run}.log", errors="replace").read()
score = re.findall(r"^\s+pair_task_score: ([0-9.]+)", text, re.M)
reps = re.findall(r"^\[rep\] step (\d+) (\{.*\})$", text, re.M)
rep = json.loads(reps[-1][1]) if reps else {}
print(f"**{run}**: pair_task_score {score[-1] if score else 'none'} (last report at step {reps[-1][0] if reps else '-'})\n")
for step, ev in re.findall(r"^\[clp\] event step (\d+): (.*)$", text, re.M):
    print(f"- step {step} ({int(step) / 20:.1f} s): {ev}")
print()
for s in ("a_left", "a_right", "b_left", "b_right"):
    print(f"- {s}: hung {rep.get(f'{s}_hung')}, tight {rep.get(f'{s}_tight')}, clamp {rep.get(f'{s}_clamp', 0):.0f} N, "
          f"head gap {rep.get(f'{s}_gap', 0) * 1000:.2f} mm, edge gap {rep.get(f'{s}_edge_gap', 0) * 1000:.2f} mm, "
          f"{rep.get(f'{s}_turns', 0):.2f} turns, shank {rep.get(f'{s}_along', 0) * 1000:.1f} mm from the centre hole")
print(f"- dropped {rep.get('dropped')}, no robot touching for 3 s {rep.get('own')}, cross-station contact peak "
      f"{rep.get('cross_contact_peak_n', 0):.2f} N\n")
arms = (("a_left", "A left"), ("a_right", "A right"), ("b_left", "B left"), ("b_right", "B right"))
print("| Arm | measure | " + " | ".join(f"J{i}" for i in range(1, 7)) + " |")
print("|---|---|" + "---|" * 6)
print("| | rated | " + " | ".join(f"{v:g}" for v in g.RATED) + " |")
print("| | peak rating | " + " | ".join(f"{v:g}" for v in g.PEAK) + " |")
worst = {}
for key, name in arms:
    for m, label in (("peak", "peak"), ("avg1", "1 s avg"), ("avg10", "10 s avg")):
        js = [rep.get(f"tq_{key}_{m}_j{i}", 0.0) for i in range(1, 7)]
        print(f"| {name} | {label} | " + " | ".join(f"{v:.2f}" for v in js) + " |")
        r = max(v / rr for v, rr in zip(js, g.RATED))
        if r > worst.get(m, (0.0, ""))[0]:
            worst[m] = (r, name)
print()
for m, (r, name) in worst.items():
    print(f"- highest {m} / rated: {r:.2f} ({name})")
