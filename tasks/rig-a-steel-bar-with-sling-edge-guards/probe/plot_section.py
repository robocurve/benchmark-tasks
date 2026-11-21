"""Cross-section (y-z plane at the sling) of probe runs: bar, guards at their seats, the hook's bowl at its hung
height, and the web's link centres at the end of settling and at the end of the hold.
python plot_section.py out.png run_dir [run_dir ...]"""

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import sling_model as m  # noqa: E402

out, runs = sys.argv[1], sys.argv[2:]
fig, axes = plt.subplots(1, len(runs), figsize=(5.2 * len(runs), 5.6), squeeze=False)
for ax, run in zip(axes[0], runs):
    log = json.loads((Path(run) / "log.json").read_text())
    s = json.loads((Path(run) / "summary.json").read_text())
    h = m.HALF * 1000
    ax.add_patch(plt.Rectangle((-h, m.Z_BOT * 1000), 2 * h, m.BAR_S * 1000, color="0.55"))
    if s["case"] != "noguards":
        for g in m.EDGES:
            if s["case"] == "bottomless" and g in ("BL", "BR"):
                continue
            _, _, _, (c1, h1), (c2, h2) = m.guard_boxes(g)
            for c, hh in ((c1, h1), (c2, h2)):
                ax.add_patch(plt.Rectangle(((c[1] - hh[1]) * 1000, (c[2] - hh[2]) * 1000), 2000 * hh[1], 2000 * hh[2],
                                           color="tab:blue"))
    th = np.linspace(np.pi, 2 * np.pi, 50)
    end_settle = next(r for r in log if r["phase"] == "end_settle")
    last = log[-1]
    for row, col in ((end_settle, "tab:orange"), (last, "k")):      # the bowl where the lift has it (swing ignored)
        dz = row["lift_mm"]
        for r, ls in ((m.BOWL_R_IN, "-"), (m.RC + m.HOOK_R, "--")):
            ax.plot(r * 1000 * np.cos(th), m.HOOK_O[2] * 1000 + dz + r * 1000 * np.sin(th), ls, color=col, lw=.8)
    for row, col, lab in ((end_settle, "tab:orange", "end of settle (slack)"), (last, "k", f"end of hold ({abs(last['pull_z_n']):.0f} N)")):
        ax.plot(row["web_y_mm"], row["web_z_mm"], "o-", color=col, ms=3, lw=1.2, label=lab)
    ax.plot([-400, 400], [0, 0], color="0.3", lw=2)
    ax.set_aspect("equal")
    ax.set_xlim(-130, 130)
    ax.set_ylim(-10, 300)
    c = s.get("criterion", {})
    ax.set_title(f"{Path(run).name}: case={s['case']}\nall edges protected: {c.get('all_protected')}", fontsize=9)
    ax.set_xlabel("y (mm, + = left arm)")
    ax.set_ylabel("z above table (mm)")
    ax.legend(fontsize=7, loc="upper right")
fig.tight_layout()
fig.savefig(out, dpi=110)
print("wrote", out)
