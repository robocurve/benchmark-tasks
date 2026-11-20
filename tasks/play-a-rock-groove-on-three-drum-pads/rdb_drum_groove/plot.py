"""The score-vs-hits plot of one trial (English only, ASCII text), from ``judge_midi.judge``'s result.

2400 x 900 pixels. Rows from the top: Hi-hat, Snare, Tom, Other (a rim, an unmapped note). The x axis is the fitted
grid's beats, the first written note at 0, with a line and "Bar n" every 4 beats and "1 & 2 & 3 & 4 &" under each
bar. Written notes are hollow circles (loud 14 pt, medium 11 pt, soft 8 pt); hits are filled markers at their real
times, their shape the stick (circle left, square right, diamond not recorded) and their colour the result (green on
time within 25 ms, orange off the grid from 25 to 50 ms with its signed error, red on the wrong pad, purple extra,
grey count-in), with the MIDI velocity under each. A missed note stays hollow, marked "missed". Each pair's two hits
are joined by a thin line, green when together (two sticks within 40 ms), red when not, with their spread in ms.

Font: DejaVu Sans, bundled with matplotlib (DejaVu Fonts licence, a free licence derived from the Bitstream Vera
licence; see LICENSE).
"""

from pathlib import Path

from . import layout as L

ROWS = ("hihat", "snare", "tom", "other")
ROW_LABEL = {"hihat": "Hi-hat", "snare": "Snare", "tom": "Tom", "other": "Other"}
COLOR = {"on time": "#1a9850", "off grid": "#f08c00", "wrong pad": "#d73027", "extra": "#7b3294",
         "count-in": "#8c8c8c"}
MARKER = {"left": "o", "right": "s", None: "D"}
SIZE = {"loud": 14, "medium": 11, "soft": 8}
SOURCES = {"sim": "simulator 240 Hz hit log written as MIDI", "real": "drum module MIDI, real trial"}


def footer(source="sim", notes=None, score_name="slow 4/4 rock-ballad groove"):
    n = len(L.NOTES) if notes is None else notes
    return (f"Written score: {score_name}, {n} notes at {L.BPM:.0f} bpm. Hits: {SOURCES[source]}. "
            "Diagnostic plot; not a policy input.")


def plot_score(result, path, name, source="sim"):
    """Draw ``result`` (``judge_midi.judge``) to ``path``; returns every text string drawn (for the ASCII test)."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    from .judge_midi import title_line

    plt.rcParams["font.family"] = "DejaVu Sans"
    texts = []

    def text(ax, *args, **kw):
        texts.append(str(args[2]))
        return ax.text(*args, **kw)

    fig, ax = plt.subplots(figsize=(24, 9), dpi=100)
    y = {pad: len(ROWS) - 1 - k for k, pad in enumerate(ROWS)}
    bpm, t0 = result.get("bpm"), result.get("t0")
    p = 60. / bpm if bpm else 60. / L.BPM
    t0 = t0 if t0 is not None else (min((h["t"] for h in result["classified"]), default=0.))

    def beat(t):
        return (t - t0) / p

    hits = result["classified"]
    xs = [beat(h["t"]) for h in hits] + [n[0] for n in L.NOTES]
    lo, hi = min(xs + [0.]) - .5, max(xs + [16.]) + .6
    bars = range(0, 21, 4)
    for b in range(int(lo // .5) - 1, int(hi // .5) + 2):
        x = b * .5
        if lo <= x <= hi:
            ax.axvline(x, color="#eeeeee" if b % 2 else "#d9d9d9", lw=.6, zorder=0)
    for k, b in enumerate(bars):
        if lo <= b <= hi:
            ax.axvline(b, color="#444444", lw=1.6, zorder=0)
            if b < 17:
                text(ax, b + .06, len(ROWS) - .45, f"Bar {k + 1}", fontsize=13, color="#444444")
        for j, label in enumerate("1 & 2 & 3 & 4 &".split()):
            x = b + j * .5
            if lo <= x <= hi and b < 17:
                text(ax, x, -.92, label, fontsize=10, ha="center", color="#555555")
    # written notes
    for i, (b, pad, dyn) in enumerate(L.NOTES):
        row = result["note_rows"][i]
        ax.plot([b], [y[pad]], marker="o", ms=SIZE[dyn] * 1.5, mfc="none", mec="#222222", mew=1.4, ls="", zorder=2)
        if row["hit"] is None:
            text(ax, b, y[pad] - .34, "missed", fontsize=10, ha="center", color="#d73027")
    # pairs
    for r in result.get("pairs", []):
        if r["spread_ms"] is None:
            continue
        i_h = next(i for i, n in enumerate(L.NOTES) if n[0] == r["beat"] and n[1] == "hihat")
        i_s = next(i for i, n in enumerate(L.NOTES) if n[0] == r["beat"] and n[1] == "snare")
        th = hits[result["note_rows"][i_h]["hit"]]["t"]
        ts = hits[result["note_rows"][i_s]["hit"]]["t"]
        color = "#1a9850" if r["together"] else "#d73027"
        ax.plot([beat(th), beat(ts)], [y["hihat"] - .12, y["snare"] + .12], color=color, lw=1.2, zorder=1)
        text(ax, (beat(th) + beat(ts)) / 2 + .07, (y["hihat"] + y["snare"]) / 2, f"{r['spread_ms']:.0f} ms",
             fontsize=9, color=color)
    # hits
    for h in hits:
        row = h["pad"] if h["pad"] in y else "other"
        x = beat(h["t"])
        ax.plot([x], [y[row]], marker=MARKER.get(h.get("stick"), "D"), ms=9, color=COLOR[h["cls"]], mec="black",
                mew=.5, ls="", zorder=3)
        text(ax, x, y[row] - .22, str(h["velocity"]), fontsize=9, ha="center", va="top", color="#333333")
        if h["cls"] == "off grid" and h.get("error_ms") is not None:
            text(ax, x, y[row] + .2, f"{h['error_ms']:+.0f} ms", fontsize=9, ha="center", color=COLOR["off grid"])
        if h["pad"] not in y:
            text(ax, x, y[row] + .2, "rim" if h["pad"] == "snare_rim" else f"note {h['note']}", fontsize=9,
                 ha="center", color=COLOR["wrong pad"])
    ax.set_yticks([y[r] for r in ROWS])
    labels = [ROW_LABEL[r] for r in ROWS]
    texts.extend(labels)
    ax.set_yticklabels(labels, fontsize=14)
    ax.set_ylim(-1.1, len(ROWS) - .25)
    ax.set_xlim(lo, hi)
    xlabel = "beats of the fitted grid (the first written note at 0)"
    texts.append(xlabel)
    ax.set_xlabel(xlabel, fontsize=12)
    title = title_line(result, name)
    texts.append(title)
    ax.set_title(title, fontsize=15)
    entries = [("o", "none", "#222222", 14, "written note (loud)"), ("o", "none", "#222222", 11, "written note (medium)"),
               ("o", "none", "#222222", 8, "written note (soft)"),
               ("o", COLOR["on time"], "black", 9, "on time (within 25 ms)"),
               ("o", COLOR["off grid"], "black", 9, "off the grid (25 to 50 ms)"),
               ("o", COLOR["wrong pad"], "black", 9, "wrong pad"), ("o", COLOR["extra"], "black", 9, "extra hit"),
               ("o", COLOR["count-in"], "black", 9, "count-in"), ("o", "#bbbbbb", "black", 9, "left stick"),
               ("s", "#bbbbbb", "black", 9, "right stick"), ("D", "#bbbbbb", "black", 9, "stick not recorded")]
    handles = [Line2D([], [], marker=m, ls="", mfc=fc, mec=ec, ms=ms * (1.5 if fc == "none" else 1.), label=lab)
               for m, fc, ec, ms, lab in entries]
    handles.append(Line2D([], [], color="#1a9850", lw=1.2, label="pair together (two sticks within 40 ms)"))
    handles.append(Line2D([], [], color="#d73027", lw=1.2, label="pair not together"))
    texts.extend(h.get_label() for h in handles)
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(.5, -.1), ncol=7, fontsize=11, frameon=False)
    foot = footer(source)
    texts.append(foot)
    fig.text(.5, .012, foot, fontsize=11, ha="center", color="#555555")
    fig.tight_layout(rect=(0, .03, 1, 1))
    fig.savefig(Path(path), dpi=100)
    plt.close(fig)
    return texts
