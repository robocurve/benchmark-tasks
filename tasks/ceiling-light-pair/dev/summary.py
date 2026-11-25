"""dev/summary.py NAME[:VARIANT:STEPS] ...: each run's final score and its last report line from dev/NAME.log."""
import json
import os
import re
import sys

D = os.path.dirname(os.path.abspath(__file__))
SCREWS = ("a_left", "a_right", "b_left", "b_right")
for arg in sys.argv[1:]:
    name, *rest = arg.split(":")
    text = open(f"{D}/{name}.log", errors="replace").read()
    score = re.findall(r"^\s+pair_task_score: ([0-9.]+)", text, re.M)
    reps = re.findall(r"^\[rep\] step (\d+) (\{.*\})$", text, re.M)
    rep = json.loads(reps[-1][1]) if reps else {}
    events = re.findall(r"^\[clp\] event step (\d+): (.*)$", text, re.M)
    print(f"{name:16s} variant={rest[0] if rest else '':13s} score={score[-1] if score else 'NONE':5s} "
          f"last_rep_step={reps[-1][0] if reps else '-'} dropped={rep.get('dropped')} own={rep.get('own')} "
          f"n_hung={rep.get('n_hung')} cross_contact={rep.get('cross_contact_peak_n')}")
    for s in SCREWS:
        print(f"    {s}: hung {rep.get(f'{s}_hung')} tight {rep.get(f'{s}_tight')} clamp {rep.get(f'{s}_clamp', 0):.0f} N "
              f"head gap {rep.get(f'{s}_gap', 0) * 1000:.2f} mm edge gap {rep.get(f'{s}_edge_gap', 0) * 1000:.2f} mm "
              f"along {rep.get(f'{s}_along', 0) * 1000:.1f} mm turns {rep.get(f'{s}_turns', 0):.2f}")
    for step, text_ in events:
        print(f"    step {step} ({int(step) / 20:.1f} s): {text_}")
