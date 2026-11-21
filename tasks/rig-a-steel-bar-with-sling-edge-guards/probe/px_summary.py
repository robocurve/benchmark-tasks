"""Compact view of a probe run.   python px_summary.py out/px-guards-1 [every]"""

import json
import sys
from pathlib import Path

run = Path(sys.argv[1])
every = int(sys.argv[2]) if len(sys.argv) > 2 else 10
log = json.loads((run / "log.json").read_text())
s = json.loads((run / "summary.json").read_text())
E = ("TL", "BL", "TR", "BR")
print(f"{run}: case={s['case']} swing={s['swing_deg']} reached={s['reached']} final={s['final_phase']} "
      f"blowup={s['nonfinite_or_blowup']} sim={s['sim_s']}s wall={s['wall_s']}s")
print("   t   phase  lift  pull |  bare-steel<=15mm TL BL TR BR | on-guard TL BL TR BR | sepG(min) sepB | guard off mm TL BL TR BR | v_max latch swing")
for r in log[::every] + [log[-1]]:
    bare = " ".join(f"{r['web_on_bare_steel_within_15mm_n'][e]:5.2f}" for e in E)
    on = " ".join(f"{r['web_on_guard_n'][e]:6.2f}" for e in E)
    sep = min(r["web_guard_min_sep_mm"].values())
    off = " ".join(f"{r['guard_offset_mm_deg'][e][0]:5.2f}" for e in E)
    print(f"{r['t']:6.3f} {r['phase'][:6]:6s} {r['lift_mm']:6.1f} {abs(r['pull_z_n']):6.2f} | {bare} | {on} | {sep:6.2f} {r['web_bar_min_sep_mm']:6.2f} "
          f"| {off} | {r['web_speed_max_mm_s']:7.1f} {r['latch_deg']:5.1f} {r['swing_deg']}")
if "criterion" in s:
    print("criterion:", json.dumps(s["criterion"]))
print("guard offset end settle:", s["guard_offset_end_settle"], " end hold:", s["guard_offset_end_hold"])
h = s["last_1s_of_hold"]
print("last 1 s of hold: pull", h["pull_z_n"], " v_max", h["web_speed_max_mm_s"], " corner-in-bar", h["web_corner_in_bar_mm"],
      " fit-fit", h["fit_on_fit_n"], " latch", h["latch_deg"])
print("whole run min web-guard sep (mm):", {e: s["whole_run"]["web_guard_min_sep_mm"][e]["min"] for e in E},
      " web-bar sep:", s["whole_run"]["web_bar_min_sep_mm"]["min"], " corner-in-bar max:", s["whole_run"]["web_corner_in_bar_mm"]["max"])
