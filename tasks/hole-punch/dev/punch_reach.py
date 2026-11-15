"""Offline reach check for the hole-punch layout on the stock yam_arena IK (gate: <0.5 mm, <0.1 deg). No GPU."""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path.home() / "three-ready-tasks" / "reach"))  # the team's shared reach-check helpers
from reach_check import solve, world, rotation, near_limit, fmt_q
# x forward from the arm bases, y left, h = TCP height above the table (pad tips 13 mm below the TCP)
PTS = [
    ("right", "card slide start, pads on card", 0.21, -0.04, 0.035, 0),
    ("right", "card held home (25 mm behind its edge)", 0.245, -0.04, 0.035, 0),
    ("right", "card held home, 1 mm aim error", 0.246, -0.041, 0.035, 0),
    ("right", "above card", 0.23, -0.04, 0.10, 0),
    ("left", "above lever end", 0.30, 0.10, 0.12, 0),
    ("left", "lever end, top of stroke", 0.30, 0.10, 0.075, 0),
    ("left", "lever end, bottom of stroke", 0.30, 0.10, 0.050, 0),
]
for arm, label, x, y, h, yaw in PTS:
    r = solve(arm, world(x, y, h), rotation(yaw))
    print(json.dumps({"arm": arm, "point": label, "x": x, "y": y, "tcp_h": h, "ok": r["ok"],
                      "pe_mm": round(r["pe_mm"], 3), "re_deg": round(r["re_deg"], 4),
                      "near_limit": near_limit(r["q"]), "q_deg": fmt_q(r["q"])}))
print("gripper separation at the press (m):", round(((0.30 - 0.245) ** 2 + (0.10 + 0.04) ** 2) ** 0.5, 3))
