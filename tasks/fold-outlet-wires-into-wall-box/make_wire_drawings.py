"""Writes the wires' starting shapes for a real rig, from the same numbers the simulator uses (outlet_logic.py):

    wire_start_points.csv   points every 5 mm along each wire, in cm
    wire_start_shapes.pdf   one page per wire, printed at 100% it is a 1:1 template (side view and top view)
    wire_start_shapes.png   the three wires on one sheet, for a quick look (not to scale)

    python make_wire_drawings.py        (needs numpy and matplotlib; no simulator)

Coordinates are the task's: x forward from the midpoint between the arm bases, y to the robot's left, heights above
the table, in cm.
"""
from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402

import outlet_logic as L  # noqa: E402

HERE = Path(__file__).resolve().parent
LOOK = {"hot": ("black", "#111111"), "neutral": ("white", "#8a8f96"), "ground": ("bare", "#b87333")}
MM = 1 / 25.4  # inches per mm


def shape(wire) -> dict:
    """The wire's start curve (dense), its length along the wire, and its two starting folds."""
    curve = L._s_curve(wire, L._s_amplitude(wire))
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(curve, axis=0), axis=1))])
    a, b = np.asarray(wire.root, float), L.terminal_world(wire)
    d = (b - a) / np.linalg.norm(b - a)
    up = np.array([0.0, 0.0, 1.0]) - d[2] * d
    up /= np.linalg.norm(up)
    side = np.cross(d, up)
    folds = []
    for at, u, v in (L.KINKS if wire.kinks is None else wire.kinks):
        i = int(round(at * (len(curve) - 1)))
        way = u * up + v * side
        folds.append({"from_clamp": cum[i], "from_terminal": cum[-1] - cum[i], "apex": curve[i],
                      "way": way / np.linalg.norm(way)})
    return {"curve": curve, "cum": cum, "length": cum[-1], "folds": folds}


def cm(p) -> tuple:
    """A world point (m) as task coordinates in cm: x forward, y left, height above the table."""
    return 100 * p[0], 100 * p[1], 100 * (p[2] - L.TOP)


def words(way) -> str:
    """Which way a fold points, in plain words (x forward is away from the robot, y to its left)."""
    parts = []
    if abs(way[2]) > 0.3:
        parts.append("up" if way[2] > 0 else "down")
    if abs(way[1]) > 0.3:
        parts.append("to the robot's left" if way[1] > 0 else "to the robot's right")
    return " and ".join(parts)


def draw(ax, s, view: str, color: str) -> None:
    c = np.array([cm(p) for p in s["curve"]])
    x, y = 10 * c[:, 0], 10 * (c[:, 2] if view == "side" else c[:, 1])  # mm
    ax.plot(x, y, color=color, lw=2.2, solid_capstyle="round")
    wall = 1000 * L.WALL_X
    ax.axvline(wall, color="#555555", lw=1.0)
    ax.axvline(wall + 1000 * L.BOX_D, color="#2a52be", lw=0.8, ls=":")
    for k, f in enumerate(s["folds"], 1):
        p = cm(f["apex"])
        ax.plot(10 * p[0], 10 * (p[2] if view == "side" else p[1]), "o", ms=5, mfc="none", mec="#cc3333")
        ax.annotate(f"fold {k}", (10 * p[0], 10 * (p[2] if view == "side" else p[1])), textcoords="offset points",
                    xytext=(0, -13), ha="center", fontsize=7, color="#cc3333")
    ax.plot(x[0], y[0], "s", ms=5, color="#2a52be")
    ax.plot(x[-1], y[-1], "D", ms=5, color="#444444")
    ax.set_aspect("equal")
    ax.invert_xaxis()  # the wall on the right, the robot on the left, as the robot's right-hand side sees it
    ax.tick_params(labelsize=6)
    ax.grid(True, lw=0.3, color="#cccccc")
    ax.set_xlabel("x forward (mm)", fontsize=7)
    ax.set_ylabel("height above the table (mm)" if view == "side" else "y to the robot's left (mm)", fontsize=7)


def page(pdf, wire, s) -> None:
    """One wire at 1:1 on a 279 x 216 mm (US Letter, landscape) page."""
    fig = plt.figure(figsize=(279 * MM, 216 * MM))
    name, color = LOOK[wire.name]
    c = np.array([cm(p) for p in s["curve"]]) * 10
    x0, x1 = c[:, 0].min() - 8, c[:, 0].max() + 8
    for view, col, bottom in (("side", 2, 120), ("top", 1, 46)):
        lo, hi = c[:, col].min() - 8, c[:, col].max() + 8
        w, h = (x1 - x0), (hi - lo)
        ax = fig.add_axes([30 / 279, bottom / 216, w / 279, h / 216])  # 1 mm of data is 1 mm of paper
        ax.set_xlim(x0, x1)
        ax.set_ylim(lo, hi)
        draw(ax, s, view, color)
        ax.set_title(("Side view" if view == "side" else "Top view") + " (1:1 when printed at 100%)", fontsize=8, loc="left")
    fig.text(30 / 279, 205 / 216, f"The {name} wire's starting shape: {1000 * s['length']:.0f} mm from the cable entry "
             f"(blue square) to its terminal (grey diamond)", fontsize=9)
    lines = [f"fold {k}: apex {1000 * f['from_terminal']:.0f} mm from the terminal along the wire, "
             f"{1000 * L.KINK_H:.0f} mm high over {2000 * L.KINK_HALF:.0f} mm of wire, pointing {words(f['way'])}"
             for k, f in enumerate(s["folds"], 1)]
    fig.text(30 / 279, 13 / 216, "\n".join(lines) + "\nGrey line: the wall's face. Dotted blue line: the back of the box."
             "   Check the print: the bar below is 50 mm.", fontsize=7, va="bottom")
    bar = fig.add_axes([30 / 279, 8 / 216, 50 / 279, 1.2 / 216])
    bar.set_facecolor("black")
    bar.set_xticks([])
    bar.set_yticks([])
    pdf.savefig(fig)
    plt.close(fig)


def main() -> None:
    shapes = {w.name: (w, shape(w)) for w in L.wires()}
    with (HERE / "wire_start_points.csv").open("w", newline="") as f:
        out = csv.writer(f)
        out.writerow(["wire", "mm_along_wire_from_cable_entry", "x_forward_cm", "y_left_cm", "height_cm"])
        for name, (w, s) in shapes.items():
            for at in np.append(np.arange(0.0, s["length"], 0.005), s["length"]):
                p = [np.interp(at, s["cum"], s["curve"][:, k]) for k in range(3)]
                out.writerow([LOOK[name][0], f"{1000 * at:.0f}", *(f"{v:.2f}" for v in cm(p))])
    with PdfPages(HERE / "wire_start_shapes.pdf") as pdf:
        for name, (w, s) in shapes.items():
            page(pdf, w, s)
    fig, axes = plt.subplots(2, 3, figsize=(13, 5.2), gridspec_kw={"height_ratios": [1.6, 1]})
    for j, (name, (w, s)) in enumerate(shapes.items()):
        draw(axes[0, j], s, "side", LOOK[name][1])
        draw(axes[1, j], s, "top", LOOK[name][1])
        axes[0, j].set_title(f"{LOOK[name][0]} wire, {1000 * s['length']:.0f} mm: side view", fontsize=9, pad=10)
        axes[1, j].set_title("top view", fontsize=9, pad=10)
    fig.tight_layout()
    fig.savefig(HERE / "wire_start_shapes.png", dpi=110)
    plt.close(fig)
    for name, (w, s) in shapes.items():
        print(f"{LOOK[name][0]:6s} {1000 * s['length']:.0f} mm: " + "; ".join(
            f"fold {k} {1000 * f['from_terminal']:.0f} mm from the terminal, pointing {words(f['way'])}"
            for k, f in enumerate(s["folds"], 1)))


if __name__ == "__main__":
    main()
