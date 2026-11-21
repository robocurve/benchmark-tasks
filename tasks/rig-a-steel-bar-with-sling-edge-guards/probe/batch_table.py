"""One line per run of a batch (run_batch.sh).   python batch_table.py out/final"""

import hashlib
import json
import sys
from pathlib import Path

E = ("TL", "BL", "TR", "BR")
root = Path(sys.argv[1])
print(f"{'run':18s} {'case':10s} {'swing':>5s} {'v':>5s} | {'reach t/lift/pull':>20s} | {'bare max (N) TL BL TR BR':>26s} | "
      f"{'on-guard min (N) TL BL TR BR':>28s} | ok | guard off max mm/deg | pull last1s | v_max | sepG min mm | log md5")
for d in sorted(p for p in root.iterdir() if p.is_dir()):
    f = d / "summary.json"
    if not f.exists():
        print(f"{d.name:18s} (no summary)")
        continue
    s = json.loads(f.read_text())
    c = s.get("criterion", {})
    r = s["reached"] or {}
    bare = " ".join(f"{c['bare_steel_max_n'][e]:6.2f}" for e in E) if c else "-"
    on = " ".join(f"{c['on_guard_min_n'][e]:6.2f}" for e in E) if c else "-"
    offs = [v for k, v in s["guard_offset_end_hold"].items()
            if not (s["case"] == "noguards" or (s["case"] == "bottomless" and k in ("BL", "BR")))]
    off = f"{max(o[0] for o in offs):.2f}/{max(o[1] for o in offs):.2f}" if offs else "-"
    h = s["last_1s_of_hold"]
    sepg = min(s["whole_run"]["web_guard_min_sep_mm"][e]["min"] for e in E)
    md5 = hashlib.md5((d / "log.json").read_bytes()).hexdigest()[:8]
    print(f"{d.name:18s} {s['case']:10s} {s['swing_deg']:5.1f} {1000 * s.get('lift_speed_m_s', .03):5.0f} | "
          f"{r.get('t', '-')!s:>5}/{r.get('lift_mm', '-')!s:>6}/{r.get('pull_n', '-')!s:>6} | {bare} | {on} | "
          f"{'Y' if c.get('all_protected') else 'N'}  | {off:>9s} | {h['pull_z_n']['mean']:5.1f}+-{h['pull_z_n']['std']:.1f} | "
          f"{h['web_speed_max_mm_s']['mean']:5.1f} | {sepg:7.3f} | {md5}")
