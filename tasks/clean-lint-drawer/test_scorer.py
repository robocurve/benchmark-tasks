"""Offline checks of the scorer against made-up outcomes (no simulator). Run: python test_scorer.py"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parent))
import clean_lint_drawer as t  # noqa: E402

ID = (0.0, 0.0, 0.0, 1.0)


def yawq(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def st(pos, q=ID):
    return SimpleNamespace(pos=pos, quat_xyzw=q, resting=lambda: True)


S, L = t.SCREEN, t.LINT
screen0 = st(S.pos)
lint0 = st(L.pos)
lint_bin = st((t.BIN_X, t.BIN_Y, t.BIN_FLOOR_TOP + L.size[2] / 2))
seat_x = t.STOP_X - S.size[0] / 2
screen_seated = st((seat_x, t.PY, t.TOP + S.size[2] / 2))
screen_half = st((seat_x - t.DEPTH / 2, t.PY, t.TOP + S.size[2] / 2))
on_top = lambda s: st((s.pos[0], s.pos[1], t.TOP + S.size[2] + L.size[2] / 2))  # noqa: E731
lint_table = st((0.12, -0.25, t.TOP + L.size[2] / 2))

cases = {
    "start (nothing done)": ({"lint_screen": screen0, "lint_pad": lint0}, 0.0),
    "lint binned only": ({"lint_screen": screen0, "lint_pad": lint_bin}, 0.9 * 0.35),
    "lint binned, screen half in": ({"lint_screen": screen_half, "lint_pad": lint_bin}, 0.9 * (0.35 + 0.325)),
    "lint binned, screen seated": ({"lint_screen": screen_seated, "lint_pad": lint_bin}, 0.9),
    "screen seated with lint on it (dirty)": ({"lint_screen": screen_seated, "lint_pad": on_top(screen_seated)}, 0.0),
    "lint scraped onto table, screen seated": ({"lint_screen": screen_seated, "lint_pad": lint_table}, 0.0),
    "lint brushed onto table only": ({"lint_screen": screen0, "lint_pad": lint_table}, 0.0),
    "screen thrown in the bin": ({"lint_screen": st((t.BIN_X, t.BIN_Y, t.BIN_FLOOR_TOP + 0.03), yawq(0)),
                                  "lint_pad": lint_bin}, 0.0),
    "screen fell off table": ({"lint_screen": st((0.2, -0.12, t.TOP - 0.3)), "lint_pad": lint_bin}, 0.0),
    "screen on top of housing roof": ({"lint_screen": st((seat_x, t.PY, t.TOP + 0.04)), "lint_pad": lint_bin},
                                      0.9 * 0.35),
    "screen seated but 10 deg skew (walls would stop this; scorer check)":
        ({"lint_screen": st(screen_seated.pos, yawq(10)), "lint_pad": lint_bin}, None),
}

ok = True
for name, (objs, want) in cases.items():
    got = t.score(objs)
    done = t.all_done(objs)
    line = f"{name:62s} score={got:.3f} done={done} hard0={t.hard_zero(objs)}"
    if want is not None and abs(got - want) > 1e-6:
        ok = False
        line += f"  <-- expected {want:.3f}"
    print(line)

assert t.all_done({"lint_screen": screen_seated, "lint_pad": lint_bin})
assert not t.all_done({"lint_screen": st(screen_seated.pos, yawq(10)), "lint_pad": lint_bin})
assert not t.all_done({"lint_screen": screen_half, "lint_pad": lint_bin})
d = t.demo()
print("demo seconds:", round(sum(w.duration for w in d["right"]), 1), "steps at 20 Hz:",
      round(20 * sum(w.duration for w in d["right"])))
print("ALL OK" if ok else "MISMATCH")
sys.exit(0 if ok else 1)
